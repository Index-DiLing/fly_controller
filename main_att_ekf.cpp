#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_gpio_profile.h"
#include "dlx_usart.hpp"
#include "dlx_bytebuffer.hpp"
#include "stdio.h"
#include "dlx.hpp"
#include "rtthread.h"
#include "BMI088/dlx_bmi088.hpp"
#include "DL_AHRS/MadgwickAHRS.hpp"
#include "flight_control/flight_control_fliter.hpp"
#include "BME280/dlx_bme280.hpp"
using namespace dlx;

//===================================================================================================
// main_att_ekf.cpp
// 姿态估计对比: Madgwick(四元数, 走 QuatW) vs 16 维 EKF(走 EKFW)
// 同一份纯 BMI088 IMU(陀螺2000Hz/加速度计1600Hz) 输入, 陀螺全速率估计姿态.
//
// 结构参考 main_dlx.cpp: SPI1 + BMI088(FIFO) + RT-Thread 硬定时器 + KSP 协议上传.
// 与 main_dlx 的差异:
//   - EKF 拆成"快路径 + 慢路径":
//       快路径 integrateNominal 每个陀螺样本(2000Hz, dt=0.5ms)推进名义状态(用满 IMU 数据);
//       慢路径每 kEkfDecim(=8) 个样本(250Hz)做一次协方差传播 propagateCovariance + 倾角更新;
//     协方差/量测是慢变量, 无需每样本刷新, 这样既保住全速率姿态又不让循环满载。
//   - 定时器每 kImuPeriodTicks(2ms) 唤醒一次, 批处理若干个 FIFO 样本;
//   - Madgwick 结果经 Dalx Protocol QuatW(类型3, 四元数 w,x,y,z) 上传;
//     EKF 结果经 EKFW(类型12, 16 个 float: 四元数4+速度3+位置3+陀螺零偏3+加速度零偏3, 外加 height 与垂速 vh) 上传.
//   - 二者四元数约定一致 (机体 -> 世界), 可直接在接收端换算欧拉角或取点积对比.
//
// 注意: kImuRateHz 必须与 dlx_bmi088_config.h 的陀螺 ODR 一致, 否则每个样本的 dt 会错。
//===================================================================================================

// 估计/采样周期
// BMI088: 陀螺 ODR=2000Hz, 加速度计 ODR=1600Hz (见 dlx_bmi088_config.h)
// 陀螺样本间隔 = 0.5ms; 定时器仍每 2ms 唤醒一次, 每次批处理 ~4 个样本。
constexpr float kImuRateHz         = 2000.0f;           // 陀螺 ODR
constexpr float kImuDt             = 1.0f / kImuRateHz; // 每样本间隔 = 0.5ms
constexpr uint32_t kImuPeriodTicks = 2;                 // 定时器唤醒周期 2ms
constexpr uint32_t kEkfDecim       = 8;                 // 每 8 个样本做一次慢更新 = 250Hz

// Madgwick 参数 (sampleFreq 必须与估计周期一致; beta 越大跟踪越快、但越抖)
constexpr float kMadgwickBeta = 0.1f;

char g_fixed_str_buffer[128];
#define gfsb (int8_t *)g_fixed_str_buffer, (uint16_t)strlen(g_fixed_str_buffer)
int fixed_sprintf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int ret = vsnprintf(g_fixed_str_buffer, 128, format, args);
    va_end(args);
    return ret;
}

//---------------------------------------------------------------------------------------------------
// 传感器轴重映射: 若 IMU 相对机体绕 z 偏转 kSensorYawDeg(CCW+), 用它与主飞控保持一致.
// 0 = IMU 与机体对齐 (与 main_dlx 相同).
//---------------------------------------------------------------------------------------------------
constexpr int kSensorYawDeg = 0;
static inline void remapSensorFrame(float &x, float &y, float &z)
{
    switch (kSensorYawDeg) {
        case 90: {
            const float t = x;
            x             = y;
            y             = -t;
            break;
        }
        case 180: {
            x = -x;
            y = -y;
            break;
        }
        case 270: {
            const float t = x;
            x             = -y;
            y             = t;
            break;
        }
        default:
            break; // 0°
    }
    (void)z;
}

FlightControlFilterParams ekfParams;
FlightControlFilter ekf;
uint32_t loopCostCnt = 0; // 统计周期内主循环次数
uint32_t loopCostAcc = 0; // 累计主循环 CPU 周期(含 sem 等待, 不含底部 DMA)
uint32_t loopCostMax = 0; // 单次主循环最大 CPU 周期(不含底部 DMA)

// ---- EkfCompare(类型 16) 对比包状态 ----
uint32_t cmpSeq        = 0; // 自增帧号
uint32_t lastLoopStart = 0; // 上一次循环起点(DWT), 用于实测循环周期
// 1 = 每轮都发(500Hz); 2 = 每两轮发一次(250Hz); 4 = 125Hz。带宽紧张时调大。
constexpr uint8_t kCmpDecim = 1;
uint8_t cmpDiv              = 0;

AccelerometerG lastAcc      = {{0.0f, 0.0f, 1.0f}};
GyroscopeRads lastGyro      = {{0.0f, 0.0f, 0.0f}};
AccelerometerRaw lastAccRaw = {{0, 0, 0}};
GyroscopeRaw lastGyroRaw    = {{0, 0, 0}};
int main()
{
    DLX_NVIC_AutoConfig();

    // ---- 高分辨率计时: 使能 DWT 周期计数器(168MHz, 用于测 EKF 单周期耗时) ----
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // ---- 调试口(仅作指示) ----
    GPIO pf3(GPIOProfile::F3);
    pf3.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    pf3 = 0;

    // ---- USART1 / KSP 协议 ----
    uint8_t rxBuffer[256];
    ByteBuffer rxBuf(rxBuffer, 256);
    RingByteBuffer buf(rxBuf);
    auto usart1 = USART::USART1_TA9_RAA();
    usart1.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, 2000000, buf);

    // 每轮要发 Quat(18B) + EKF(26B) + EkfCompare(146B) = 190B, 缓冲区必须放得下,
    // 否则生成代码里 buffer.remaining() 不够会直接 return false(静默丢帧, 不报错).
    uint8_t dmaB[256];
    ByteBuffer dmaBuffer(dmaB, 256);
    uint8_t fr[256];
    ByteBuffer frame(fr, 256);
    DLX_ProtocolBuffer protocol(dmaBuffer, &buf, &frame);


    bool atom = true;

    protocol.setGroundCmdCallbackFunction(+[](GroundCmd *, void * at){
        *static_cast<bool*>(at) = false;
    },&atom);

    while (atom)
    {
        protocol.check();
    }

    pf3 = 1;

    
    //----BME280----

    IICBus bus = IICBus::IIC1_SB6_DB7();

    bus.init(IICBusModeProfile::ACK_E_DC16_9_ADDR7, 400000, 13); // 100kHz 更稳, 与已验证的主程序一致

    BME280 bme(bus);

    // 必须检查初始化结果! init() 失败=校验 ChipId 失败, 未应用 Normal 模式配置,
    // 传感器会停在默认 sleep 状态, 气压读数恒定, 高度就不随环境变化。
    const bool bmeOk = bme.init();
    float baroRef    = 0.0f; // 起飞点气压高度基准(相对零点), 延迟到"第一个稳定读数"再取
    bool baroRefSet  = false;
    if (!bmeOk) {
        fixed_sprintf("BME280 init FAILED! baro 不可用\n");
        protocol.SimpleLogW(gfsb);
    }

    // ---- BMI088 ----
    auto spi = SPI::SPI1_SA5_MIA6_MOA7(GPIOProfile::A3);
    // BR16 = APB2(84MHz)/16 = 5.25MHz(<= BMI088 上限 10MHz), 比原 BR256(328kHz) 快 16 倍
    spi.init(SPIModeProfile::FD_M_8B_CH_2E_NS_BR16_MSB);
    auto bmi     = BMI088(spi, GPIOProfile::BC, GPIOProfile::BD);
    uint8_t self = bmi.init();

    // ---- 定时器: 按 kImuPeriodTicks 唤醒 (500Hz) ----
    auto mutex = rt_sem_create("sensor", 0, RT_IPC_FLAG_PRIO);
    auto timer = rt_timer_create("imut", +[](void *mut) { rt_sem_release(static_cast<rt_sem_t>(mut)); }, mutex, kImuPeriodTicks, RT_TIMER_FLAG_HARD_TIMER | RT_TIMER_FLAG_PERIODIC);

    // ---- FIFO 队列 ----
    uint8_t acbb[256];
    uint8_t gybb[256];
    ByteBuffer acb(acbb, 256);
    ByteBuffer gyb(gybb, 256);
    auto ac = Queue<AccelerometerRaw>(acb);
    auto gy = Queue<GyroscopeRaw>(gyb);

    // ---- 两个估计器: 同一份 IMU, 同一周期 ----
    MadgwickAHRS ahrs(kImuRateHz, kMadgwickBeta); // Quat 通道 (Madgwick)
    ekfParams.estimate_gyro_bias = true;
    ekfParams.estimate_accel_bias = false;        // 纯 IMU: 加速度零偏保持常数(不可观)
    // 倾角修正的瞬时门限: | |a|-g | > 0.20 就丢。0.05 太严 —— 它低于加速度计自身的噪声底
    // (实测静止时 | |a|-g | 的 p95 就有 6.5~22 mg, 单位 m/s^2 即 0.065~0.22), 结果静止时
    // 大部分修正也被拒, 姿态收敛不动(第二份数据: 稳态误差 2.45°, 而 Madgwick 只有 0.23°)。
    ekfParams.accel_tilt_gate_mss = 0.20f;

    // ---- 倾角修正的三道保险(2026-09-15 两轮手晃数据 + 500Hz 录播回放扫参数) ----
    // 病根一: 手晃时 |a| 每周期两次穿过 1g 让门限打开, 而那一刻比力方向可能被切向/向心加速度
    //          污染几十度; 原来 P 已涨大而 R 只有 0.05 => 增益接近 1, 一次更新把姿态拽反(单帧 41°)。
    // 病根二: 门限 0.05 m/s^2 低于噪声底 => 静止时修正也被大量拒绝; 限幅 0.05rad 又卡住正常修正
    //          (实测门限打开时的 innovation 中位就有 4.88°), 于是"最后那一点角度"收敛极慢。
    // 回放扫参结果(两份数据都验):
    //   参数                                 单步最大   反向   静止段误差   手晃段误差(中位/p95)
    //   原固件(gate .05/无限幅/σ.05)        41.7°     93   0.17/0.36   0.24/4.80
    //   上版(gate .05/限幅.05/σ.15)          2.3°      0   0.18/0.37   1.11/16.4   <- 太保守
    //   本版(gate .20/限幅.15/σ.08)          3.0°      0   0.17/0.36   0.24/4.00   <- 精度追平/略胜 Madgwick
    //   更激进(gate .30/限幅.20/σ.05)        4.4°     10   0.17/0.36   0.21/3.31   <- 反向回来了, 别越界
    //   (Madgwick 参照:                     2.3°      0   0.16/0.35   0.20/3.54)
    // 收敛速度(第二份数据, 手晃停下后): 起始误差 6.64°->1.41°, 0.3s 后 5.11°->0.54°, 稳态 2.45°->0.31°
    ekfParams.accel_tilt_inno_limit_rad     = 0.15f; // 单次修正 innovation 限幅 ≈8.6°(别关: 这是防单帧跳变的根)
    ekfParams.accel_tilt_sigma     = 0.08f;  // 门限/限幅放宽后, 增益要相应恢复到 ~0.16 才能收敛得动
    ekfParams.freeze_gyro_bias_when_dynamic = true;  // 高动态期间冻结陀螺零偏
    // 可选: 想把"高动态期完全不修正"作为第二道保险, 打开下面这组(代价是中等角速度段跟随变慢)
    ekfParams.accel_hold_enable    = false;  // 关=保持瞬时门限行为; 开=要求持续准静止
    ekfParams.accel_hold_gate_mss  = 0.15f;  // | |a|-g | <= 15mg (静止实测 p95≈6.5mg)
    ekfParams.accel_hold_rate_dps  = 150.0f; // 角速度 <= 150deg/s (静止实测 p95≈4deg/s; 50 会掉精度)
    ekfParams.accel_hold_time_s    = 0.05f;  // 需要连续满足 50ms
    ekfParams.baro_sigma_m        = 1.0f;         // BME280 气压高度本征噪声约 ±0.5~1m, 放宽才不会被带着抖
    ekf.set(ekfParams);
    ekf.reset(); // 姿态默认水平, 速度/位置 0

    fixed_sprintf("Hello,DLX Att-EKF @%.0fHz\n", kImuRateHz);
    protocol.SimpleLogW(gfsb);

    auto serialDMA = usart1.setDMASend(dmaBuffer);
    pf3            = 0;
    rt_timer_start(timer);

    // ---- BME280 30ms 门控 & EKF 单周期耗时统计 ----
    const uint32_t kBaroIntervalMs = 20;
    uint32_t lastBmeTick           = rt_tick_get();
    uint32_t lastLogTick           = 0; // 按时间触发日志, 避免循环变慢导致打印间隔漂移
    float lastBaroAbs              = 0.0f; // 最近一次读到的绝对海拔(诊断用)
    float lastBaroObs              = 0.0f; // 最近一次喂给 EKF 的观测值(相对起飞点)
    uint32_t imuCnt                = 0;    // 快路径累计样本数, 用于决定何时跑慢路径
    float imuDtAccum               = 0.0f; // 慢路径累计 dt

    while (true) {
        rt_sem_take(mutex, RT_WAITING_FOREVER);
        
        const uint32_t loopStart = DWT->CYCCNT; // 本次循环起点(在 rt_sem_take 之后: 只统计纯工作量, 不含等待与底部 DMA)
        bool    baroUpdatedThisFrame = false;   // 本帧是否有气压观测送进 EKF
        uint8_t slowCnt              = 0;       // 本帧慢路径(协方差+倾角)执行次数
        uint8_t gateReject           = 0;       // 本帧倾角修正被门限拒绝次数
        bmi.fifoRead(ac, gy);
        const auto gySize = gy.size();
        const auto acSize = ac.size();
        AccelerometerRaw acr;
        AccelerometerG curAcc = lastAcc;
        uint16_t nearestAC    = 0;
        if (acSize > 0) {
            ac.pop(acr);
            curAcc = BMI088::getAccelerationG(acr);
            remapSensorFrame(curAcc.data[0], curAcc.data[1], curAcc.data[2]);
            lastAcc    = curAcc;
            lastAccRaw = acr;
            nearestAC  = 1;
        }
        uint16_t gyCnt = 0;
        GyroscopeRaw gyr;
        while (gy.pop(gyr)) {
            if (nearestAC < acSize && ((gyCnt * acSize + gySize / 2) / gySize > nearestAC)) {
                if (ac.pop(acr)) {
                    curAcc = BMI088::getAccelerationG(acr);
                    remapSensorFrame(curAcc.data[0], curAcc.data[1], curAcc.data[2]);
                    lastAcc    = curAcc;
                    lastAccRaw = acr;
                    nearestAC++;
                }
            }
            GyroscopeRads gyroR = BMI088::getAngularVelocityRads(gyr);
            remapSensorFrame(gyroR.data[0], gyroR.data[1], gyroR.data[2]);
            lastGyro    = gyroR;
            lastGyroRaw = gyr;
            ahrs.MadgwickAHRSupdateIMU(StandardIMUGRads{.accel = curAcc, .gyro = gyroR});

            // ---- 快路径: 每个 IMU 样本只做名义积分(用满 IMU 数据, 不含协方差) ----
            ekf.integrateNominal(gyroR, curAcc, kImuDt);

            // ---- 慢路径: 每 kEkfDecim 个样本做一次协方差传播 + 加速度计更新 ----
            imuDtAccum += kImuDt;
            if (++imuCnt >= kEkfDecim) {
                ekf.propagateCovariance(gyroR, curAcc, imuDtAccum);
                // 传入慢路径累计 dt: 准静止计数器按时间累加(与 kEkfDecim 无关)
                if (!ekf.updateAccelerometer(curAcc, imuDtAccum)) { ++gateReject; } // 被门限/判定拒绝
                ++slowCnt;
                imuCnt     = 0;
                imuDtAccum = 0.0f;
            }

            gyCnt++;
        }
        while (ac.pop(acr)) {
            lastAcc = BMI088::getAccelerationG(acr);
            remapSensorFrame(lastAcc.data[0], lastAcc.data[1], lastAcc.data[2]);
            lastAccRaw = acr;
        }
        ac.clear();
        gy.clear();

        // ---- BME280 高度观测: 每 30ms 读一次气压并送入 EKF(仅主线程, 不开独立线程) ----
        if (bmeOk && (uint32_t)(rt_tick_get() - lastBmeTick) >= kBaroIntervalMs) {
            lastBmeTick             = rt_tick_get();
            const EnviromentRaw raw = bme.getRaw();
            lastBaroAbs             = bme.getAltitude(raw);
            // 复位/刚上电的第一次读数不可靠, 用循环里第一个已稳定的读数作为基准
            if (!baroRefSet) {
                baroRef    = lastBaroAbs;
                baroRefSet = true;
            }
            lastBaroObs = lastBaroAbs - baroRef; // 相对起飞点高度
            ekf.updateBarometer(lastBaroObs);
            baroUpdatedThisFrame = true;
        }

        // ---- 上报 ----
        dmaBuffer.reset();

        float quatMadgwick[4];
        const Quaternion qM = ahrs.getQuaternion();
        quatMadgwick[0]     = qM.data[0];
        quatMadgwick[1]     = qM.data[1];
        quatMadgwick[2]     = qM.data[2];
        quatMadgwick[3]     = qM.data[3];
        protocol.QuatW(quatMadgwick); // 类型 3: 16 字节四元数

        FlightFilterState16 st = ekf.getState16();
        protocol.EKFW(st.data, ekf.getHeight(), ekf.getVerticalVelocity()); // 类型 12: 16 float 状态 + 高度 + 垂速(vh)

        // ---- 对比包: Madgwick vs EKF (类型 16, 144B 负载) ----
        // 两路姿态 + EKF 内部量(速度/零偏/协方差对角) + 输入快照 + 健康计数, 便于上位机逐帧比对。
        if (++cmpDiv >= kCmpDecim) {
            cmpDiv = 0;

            const Quaternion qE = ekf.getQuaternion();
            const Vector3f   vE = ekf.getVelocity();
            const Vector3f   bE = ekf.getGyroBias();

            float ekfQuatArr[4] = {qE.data[0], qE.data[1], qE.data[2], qE.data[3]};
            float velArr[3]     = {vE.x(), vE.y(), vE.z()};
            float bgArr[3]      = {bE.x(), bE.y(), bE.z()};
            float attVar[3]     = {ekf.getAttVar(0), ekf.getAttVar(1), ekf.getAttVar(2)};
            float velVar[3]     = {ekf.getVelVar(0), ekf.getVelVar(1), ekf.getVelVar(2)};
            float bgVar[3]      = {ekf.getGyroBiasVar(0), ekf.getGyroBiasVar(1),
                                   ekf.getGyroBiasVar(2)};

            // 实测循环周期: DWT 168MHz, /(SystemCoreClock/1e6) = us (第一帧无意义, 会被夹住)
            uint32_t periodUs = (uint32_t)(loopStart - lastLoopStart) / (SystemCoreClock / 1000000u);
            lastLoopStart     = loopStart;
            if (periodUs > 0xFFFFu) { periodUs = 0xFFFFu; }

            uint16_t cmpFlags = 0;
            if (bmeOk)                { cmpFlags |= 1u << 0; } // 气压计可用
            if (baroRefSet)           { cmpFlags |= 1u << 1; } // 气压基准已锁
            if (baroUpdatedThisFrame) { cmpFlags |= 1u << 2; } // 本帧有气压更新
            if (slowCnt)              { cmpFlags |= 1u << 3; } // 本帧跑了慢路径
            if (gateReject)           { cmpFlags |= 1u << 4; } // 倾角被门限拒绝
            if (ekf.isValid())        { cmpFlags |= 1u << 5; }
            if (!(isfinite(qE.data[0]) && isfinite(qE.data[1]) && isfinite(qE.data[2]) &&
                  isfinite(qE.data[3]) && isfinite(ekf.getHeight()) && isfinite(vE.x()))) {
                cmpFlags |= 1u << 6;                           // 检出 NaN/Inf
            }

            protocol.EkfCompareW(
                ++cmpSeq, (uint32_t)rt_tick_get(), (uint16_t)periodUs, cmpFlags,
                (uint8_t)gySize, (uint8_t)acSize, slowCnt, gateReject,
                lastGyro.data, lastAcc.data,
                quatMadgwick, ekfQuatArr,
                velArr, bgArr, ekf.getHeight(), lastBaroObs,
                attVar, velVar, bgVar, ekf.getPosVar(2));
        }

        // ---- 主循环耗时, 每 200ms 打一条(按时间, 不看循环次数) ----
        if ((uint32_t)(rt_tick_get() - lastLogTick) >= 200) {
            lastLogTick       = rt_tick_get();
            const float avgUs = (loopCostCnt > 0)
                                    ? (float)loopCostAcc / (float)loopCostCnt / SystemCoreClock * 1e6f
                                    : 0.0f;
            const float maxUs = (float)loopCostMax / SystemCoreClock * 1e6f;
            fixed_sprintf("loop avg=%.1fus max=%.1fus  abs=%.2f  rel=%.2f  h=%.2f  vz=%.2f\n",
                          avgUs, maxUs, lastBaroAbs, lastBaroObs,
                          ekf.getHeight(), ekf.getVerticalVelocity());
            protocol.SimpleLogW(gfsb);
            loopCostAcc = 0;
            loopCostMax = 0;
            loopCostCnt = 0;
        }

        // 结算本次主循环耗时(到此为止, 不含下面这段 DMA)
        {
            const uint32_t loopCycles = DWT->CYCCNT - loopStart;
            loopCostAcc += loopCycles;
            if (loopCycles > loopCostMax) loopCostMax = loopCycles;
            loopCostCnt++;
        }

        serialDMA.reset(protocol.buffer.used());
        serialDMA.start();

        serialDMA.wait();
    }
}
