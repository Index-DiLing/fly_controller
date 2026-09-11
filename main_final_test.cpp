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
#include "NRF24L01/dlx_nrf24l01.hpp"
#include "flight_control/flight_controller.hpp"
#include "flight_control/flight_control_mixer.hpp"
#include "dlx_dshot.hpp"
#include "dlx_timer.hpp"
#include "W25Q128/dlx_w25q128.hpp"
using namespace dlx;
//=================================================================================================
// 运行参数
//=================================================================================================
constexpr float    kDegToRad    = 0.017453292519943295f;
constexpr float    kLoopHz      = 500.0f;
constexpr float    kLoopDt      = 1.0f / kLoopHz;
constexpr uint32_t kCtrlTickMs  = 2;
constexpr uint32_t kTelTickMs   = 20;
constexpr uint32_t kLogTickMs   = 20;
constexpr uint32_t kSerBaud     = 2000000;
// ------------------------------------------------------------------ //
// 启动握手 / 日志模式
//   上电后 kUnblockTimeoutMs 内收到串口 UnBlock:
//     param == kParamFlashDump -> 纯回传: 读 flash 并串口转发(DLX 原样)
//     param == kParamSerialLog(其它) -> 串口回传 + 正常飞行(台架)
//   超过超时未收到 UnBlock -> flash 写入模式(飞行): 写 flash, 关串口回传
// ------------------------------------------------------------------ //
constexpr uint32_t kUnblockTimeoutMs = 12000;
constexpr uint8_t  kParamSerialLog  = 14;
constexpr uint8_t  kParamFlashDump  = 13;
enum class LogMode : uint8_t { SerialTelemetry, FlashDump, FlashLog };
// ------------------------------------------------------------------ //
// Flash 日志区: 一次性使用. 上电(flash 模式)整块 64KB 块擦除, 之后顺序写 DLX 帧,
// 帧紧邻; 读取端以"首两字节 0xFFFF / 非法 DLX 头"为终止(表示从未写过).
// ------------------------------------------------------------------ //
constexpr uint32_t kFlashLogStart  = 0x000000u;
constexpr uint32_t kFlashLogBlocks = 32u;
constexpr uint32_t kFlashLogSize   = kFlashLogBlocks * W25Q128_BLOCK64K_SIZE;
constexpr uint16_t kFlightDebugMaxFrame = 256;
#define FLASH_CS GPIOProfile::D1
constexpr SPIModeProfile FLASH_SPI_MODE = SPIModeProfile::FD_M_8B_CL_1E_NS_BR8_MSB;
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
// ------------------------------------------------------------------ //
// 传感器轴重映射: 若 IMU 相对机体绕 z 轴偏转 kSensorYawDeg(CCW+, 0/90/180/270)
// 导致 roll/pitch "标反", 就把它设为对应角度; 0 表示 IMU 与机体帧对齐(默认).
// 判定法: 手动抬机头(绕横轴)应使 euler_deg[1](pitch) 变; 左右滚应使 euler_deg[0](roll) 变.
//         若相反, 把 kSensorYawDeg 设为 90 或 270(试到一致为止).
// ------------------------------------------------------------------ //
constexpr int kSensorYawDeg = 0;
static inline void remapSensorFrame(float &x, float &y, float &z)
{
    switch (kSensorYawDeg) {
        case 90:  { const float t = x; x =  y; y = -t; break; }
        case 180: { x = -x; y = -y; break; }
        case 270: { const float t = x; x = -y; y =  t; break; }
        default:  break; // 0°
    }
    (void)z;
}
//=================================================================================================
// 共享飞行状态(跨线程)
//=================================================================================================
struct IntegratedFlightState
{
    uint16_t              command        = 9;
    float                 manualThrottle = 0.0f;
    FlightControlSetpoint setpoint{};
    uint16_t              motorValue[4]  = {50, 50, 50, 50};
    bool                  startMotor     = false;
    bool                  stopMotor      = false;
    uint32_t              lastValidCtrl  = 0;
    bool     enabledMotor    = false;
    bool     startingMotor   = false;
    uint32_t motorStartTime  = 0;
    struct Telemetry
    {
        Quaternion quat{{1.0f, 0.0f, 0.0f, 0.0f}};
        uint16_t   motor[4]  = {0, 0, 0, 0};
        float      height_m  = 0.0f;
        uint16_t   error     = 0;
        bool       enabled   = false;
    } telem;
    Dshot         *ds  = nullptr;
    AdvancedTimer *dst = nullptr;
};
//=================================================================================================
// 控制线程 -> 日志线程 用的一帧日志(原始 DLX 帧字节, latest-wins)
//=================================================================================================
struct SharedLogFrame
{
    volatile bool fresh    = false;
    volatile bool logError = false;
    uint16_t      len      = 0;
    uint8_t       data[kFlightDebugMaxFrame];
};
//=================================================================================================
// 结构化调试帧(类型 11)
//=================================================================================================
struct FlightDebugData
{
    uint16_t command;
    uint16_t state_flags;
    uint16_t status_flags;
    uint16_t event_flags;
    uint16_t motor[4];
    float    quat[4];
    float    euler_deg[3];
    float    body_rate[3];
    float    rate_setpoint[3];
    float    torque[3];
    float    throttle;
    float    manual_throttle;
    float    target_pitch_deg;
    float    target_roll_deg;
    float    target_height_m;
    float    accel_g[3];
    uint16_t gyro_raw[3];
    uint16_t accel_raw[3];
    float    height_m;
    float    vertical_vel_mps;
    uint16_t link_age_ms;
    uint16_t error;
};
//=================================================================================================
// NRF 链路线程上下文(50Hz)
//=================================================================================================
struct NrfThreadCtx
{
    NRF24L01             *nrf           = nullptr;
    DLX_ProtocolBuffer   *nrfTxProtocol = nullptr;
    ByteBuffer           *nrfTxByteBuf  = nullptr;
    DLX_ProtocolBuffer   *nrfRxProtocol = nullptr;
    RingByteBuffer       *nrfRing       = nullptr;
    IntegratedFlightState *fc           = nullptr;
    rt_sem_t              telSem        = nullptr;
};
//=================================================================================================
// 日志线程上下文(20Hz)
//=================================================================================================
struct LogThreadCtx
{
    SharedLogFrame *shared          = nullptr;
    DMA            *serialDma       = nullptr;
    ByteBuffer     *serialBuf       = nullptr;
    W25Q128        *flash           = nullptr;
    bool            flashMode       = false;
    uint32_t        flashWriteAddr  = 0;
    uint32_t        flashRegionEnd  = 0;
    rt_sem_t        logSem          = nullptr;
};
//=================================================================================================
// NRF 链路线程
//=================================================================================================
static void nrfTelThreadEntry(void *arg)
{
    NrfThreadCtx *ctx = static_cast<NrfThreadCtx *>(arg);
    uint8_t nrfPkt[NRF24L01_RX_PACKET_SIZE];
    while (true) {
        rt_sem_take(ctx->telSem, RT_WAITING_FOREVER);
        Quaternion quat;
        uint16_t   motor[4];
        bool       enabled;
        float      height;
        uint16_t   err;
        rt_enter_critical();
        quat    = ctx->fc->telem.quat;
        enabled = ctx->fc->telem.enabled;
        height  = ctx->fc->telem.height_m;
        err     = ctx->fc->telem.error;
        for (int i = 0; i < 4; i++) motor[i] = ctx->fc->telem.motor[i];
        rt_exit_critical();
        ctx->nrfTxByteBuf->reset();
        if (ctx->nrfTxProtocol->FlightCoreStatusW(
                quat.data[0], quat.data[1], quat.data[2], quat.data[3],
                enabled ? motor[0] : 0, enabled ? motor[1] : 0,
                enabled ? motor[2] : 0, enabled ? motor[3] : 0,
                height, err)) {
            ctx->nrf->write(ctx->nrfTxByteBuf->src, ctx->nrfTxByteBuf->used(),
                            NRF24L01_WAIT_TIMEOUT_MS);
        }
        if (ctx->nrf->read(nrfPkt, NRF24L01_RX_PACKET_SIZE, NRF24L01_WAIT_TIMEOUT_MS)) {
            ctx->nrfRing->write(nrfPkt, NRF24L01_RX_PACKET_SIZE);
            while (ctx->nrf->available() >= NRF24L01_RX_PACKET_SIZE) {
                if (!ctx->nrf->read(nrfPkt, NRF24L01_RX_PACKET_SIZE, 0)) break;
                ctx->nrfRing->write(nrfPkt, NRF24L01_RX_PACKET_SIZE);
            }
        }
        while (ctx->nrfRing->available() >= NRF24L01_RX_PACKET_SIZE) {
            const uint16_t before = ctx->nrfRing->available();
            bool consumed         = ctx->nrfRxProtocol->check();
            if (consumed) {
                rt_enter_critical();
                ctx->fc->lastValidCtrl = rt_tick_get_millisecond();
                rt_exit_critical();
            }
            if (!consumed && ctx->nrfRing->available() == before) {
                ctx->nrfRing->read(nrfPkt, NRF24L01_RX_PACKET_SIZE);
            }
        }
    }
}
//=================================================================================================
// 日志线程
//=================================================================================================
static void logThreadEntry(void *arg)
{
    LogThreadCtx *ctx = static_cast<LogThreadCtx *>(arg);
    uint8_t frame[kFlightDebugMaxFrame];
    while (true) {
        rt_sem_take(ctx->logSem, RT_WAITING_FOREVER);
        uint16_t len = 0;
        bool     got = false;
        rt_enter_critical();
        if (ctx->shared->fresh) {
            len = ctx->shared->len;
            memcpy(frame, ctx->shared->data, len);
            ctx->shared->fresh = false;
            got = (len > 0);
        }
        rt_exit_critical();
        if (!got) continue;
        if (ctx->flashMode) {
            if (!ctx->flash || ctx->flashWriteAddr + len > ctx->flashRegionEnd) {
                rt_enter_critical();
                ctx->shared->logError = true;   // 无 flash / 日志区写满
                rt_exit_critical();
            } else if (!ctx->flash->write(ctx->flashWriteAddr, frame, len)) {
                rt_enter_critical();
                ctx->shared->logError = true;
                rt_exit_critical();
            } else {
                ctx->flashWriteAddr += len;
            }
        } else {
            ctx->serialBuf->reset();
            ctx->serialBuf->write(frame, len);
            ctx->serialDma->wait();
            ctx->serialDma->reset(ctx->serialBuf->used());
            ctx->serialDma->start();
        }
    }
}
//=================================================================================================
// 纯回传: 读 flash -> 串口
//=================================================================================================
static uint16_t dumpPayloadLen(uint16_t type)
{
    switch (type) {
        case 11: return 136;
        case 10: return 30;
        case  9: return 30;
        case  6: return 8;
        case  2: return 1;
        case  0: return 1;
        default: return 0;
    }
}
static void flashDumpToSerial(W25Q128 &flash, DMA &dma, ByteBuffer &dmaBuf)
{
    uint8_t frame[kFlightDebugMaxFrame];
    uint32_t addr = kFlashLogStart;
    while (addr + 2 <= kFlashLogStart + kFlashLogSize) {
        uint8_t h[2];
        flash.read(addr, h, 2);
        const uint16_t head = (uint16_t)(((uint16_t)h[0] << 8) | h[1]);
        if (!((head & (1u << 15)) && !(head & 1u))) break;
        const uint16_t type = (head >> 5) & 0x3ff;
        const uint16_t payloadLen = dumpPayloadLen(type);
        const uint16_t totalLen = 2 + payloadLen;
        if (payloadLen == 0 || addr + totalLen > kFlashLogStart + kFlashLogSize) break;
        frame[0] = h[0];
        frame[1] = h[1];
        if (!flash.read(addr + 2, frame + 2, payloadLen)) break;
        dmaBuf.reset();
        dmaBuf.write(frame, totalLen);
        dma.wait();
        dma.reset(dmaBuf.used());
        dma.start();
        addr += totalLen;
    }
}
int main()
{
    DLX_NVIC_AutoConfig();
    GPIO pf3(GPIOProfile::F3);
    pf3.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    pf3 = 0;
    GPIO pf2(GPIOProfile::F2);          // 擦除指示灯(低电平点亮)
    pf2.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    pf2 = 1;                            // 初始熄灭(0=亮, 1=灭)
    uint8_t rxBuffer[256];
    ByteBuffer rxBuf(rxBuffer, 256);
    RingByteBuffer buf(rxBuf);
    auto usart1 = USART::USART1_TA9_RAA();
    usart1.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, kSerBaud, buf);
    uint8_t dmaB[256];
    ByteBuffer dmaBuffer(dmaB, 256);
    uint8_t fr[128];
    ByteBuffer frame(fr, 128);
    DLX_ProtocolBuffer protocol(dmaBuffer, &buf, &frame);
    bool    atom = true;
    uint8_t unblockParam = 0;
    struct { bool *armed; uint8_t *param; } ubCtx = { &atom, &unblockParam };
    protocol.setUnBlockCallbackFunction(+[](UnBlock *u, void *c) {
        auto *ctx_ = static_cast<decltype(ubCtx) *>(c);
        *ctx_->param = u->param();
        *ctx_->armed = false;
    }, &ubCtx);
    const uint32_t handshakeStart = rt_tick_get_millisecond();
    while (atom) {
        if (protocol.check()) pf3 = 1;
        if ((rt_tick_get_millisecond() - handshakeStart) > kUnblockTimeoutMs) break;
    }
    LogMode mode;
    if (atom) {
        mode = LogMode::FlashLog;
    } else if (unblockParam == kParamFlashDump) {
        mode = LogMode::FlashDump;
    } else {
        mode = LogMode::SerialTelemetry;
    }
    fixed_sprintf("Hello,DLX,mode=%d\n", (int)mode);
    protocol.SimpleLogW(gfsb);
    auto serialDMA = usart1.setDMASend(dmaBuffer);
    serialDMA.start();
    auto spiFlash = SPI::SPI2_SB10_MIC2_MOC3(FLASH_CS);
    W25Q128 flash(spiFlash);
    bool flashOk = false;
    if (mode == LogMode::FlashLog || mode == LogMode::FlashDump) {
        spiFlash.init(FLASH_SPI_MODE);
        flashOk = flash.init();
    }
    if (mode == LogMode::FlashDump) {
        if (flashOk) flashDumpToSerial(flash, serialDMA, dmaBuffer);
        pf3 = 1;
        while (true) rt_thread_mdelay(1000);
    }
    bool flashReady = false;
    if (mode == LogMode::FlashLog) {
        if (flashOk) {
            bool ok = true;
            bool ledOn = false;            // 0=亮, 1=灭
            for (uint32_t a = kFlashLogStart; a < kFlashLogStart + kFlashLogSize;
                 a += W25Q128_BLOCK64K_SIZE) {
                if (!flash.eraseBlock64K(a)) {
                    ok = false;
                    break;
                }
                ledOn = !ledOn;
                pf2 = ledOn ? 0 : 1;       // 每擦一块翻转一次 -> 闪烁
            }
            pf2 = 1;                       // 结束熄灭
            flashReady = ok;
        }
    }
    auto spi = SPI::SPI1_SA5_MIA6_MOA7(GPIOProfile::A3);
    spi.init(SPIModeProfile::FD_M_8B_CH_2E_NS_BR16_MSB);
    auto bmi = BMI088(spi, GPIOProfile::BC, GPIOProfile::BD);
    const uint8_t self = bmi.init();
    const bool bmiOk = (self != 0);
    auto ctrlSem = rt_sem_create("ctrl", 0, RT_IPC_FLAG_PRIO);
    auto ctrlTimer = rt_timer_create("ctrlt",
        +[](void *s) { rt_sem_release(static_cast<rt_sem_t>(s)); }, ctrlSem,
        kCtrlTickMs, RT_TIMER_FLAG_HARD_TIMER | RT_TIMER_FLAG_PERIODIC);
    uint8_t acbb[256];
    uint8_t gybb[256];
    ByteBuffer acb(acbb, 256);
    ByteBuffer gyb(gybb, 256);
    auto ac = Queue<AccelerometerRaw>(acb);
    auto gy = Queue<GyroscopeRaw>(gyb);
    MadgwickAHRS ahrs(2000, 0.078);
    auto spi3 = SPI::SPI3_SCA_MICB_MOCC(GPIOProfile::A4);
    spi3.init(SPIModeProfile::FD_M_8B_CL_1E_NS_BR32_MSB);
    uint8_t nrfRxRaw[128];
    ByteBuffer nrfRxByteBuf(nrfRxRaw, sizeof(nrfRxRaw));
    NRF24L01 nrf(spi3, GPIOProfile::C7, GPIOProfile::C8, nrfRxByteBuf);
    nrf.init();
    uint8_t nrfTxRaw[32];
    ByteBuffer nrfTxByteBuf(nrfTxRaw, sizeof(nrfTxRaw));
    DLX_ProtocolBuffer nrfTxProtocol(nrfTxByteBuf);
    uint8_t nrfDummyRaw[64];
    ByteBuffer nrfDummyByteBuf(nrfDummyRaw, sizeof(nrfDummyRaw));
    uint8_t nrfFrRaw[32];
    ByteBuffer nrfFrame(nrfFrRaw, sizeof(nrfFrRaw));
    uint8_t nrfRingRaw[256];
    ByteBuffer nrfRingBuf(nrfRingRaw, sizeof(nrfRingRaw));
    RingByteBuffer nrfRing(nrfRingBuf);
    DLX_ProtocolBuffer nrfRxProtocol(nrfDummyByteBuf, &nrfRing, &nrfFrame);
    AdvancedTimer timer1 = AdvancedTimer(AdvancedTimerProfile::TIM1_Up_DIV1);
    Dshot dshot = timer1.setDshot(DshotProfile::Dshot300);
    FlightController controller;
    IntegratedFlightState fc;
    fc.ds  = &dshot;
    fc.dst = &timer1;
    nrfRxProtocol.setControlToFlightCallbackFunction(
        +[](ControlToFlight *ctl, void *ctx) {
            auto *fcc = static_cast<IntegratedFlightState *>(ctx);
            rt_enter_critical();
            fcc->command = ctl->command();
            if (fcc->command == 1) {
                fcc->motorValue[0] = ctl->motor0();
                fcc->motorValue[1] = ctl->motor1();
                fcc->motorValue[2] = ctl->motor2();
                fcc->motorValue[3] = ctl->motor3();
            }
            if (fcc->command == 0) {
                fcc->manualThrottle = ctl->throttle();
                fcc->setpoint.attitude = quatFromEulerZYX(0,
                    ctl->targetPitch() * kDegToRad, ctl->targetRoll() * kDegToRad);
            }
            if (fcc->command == 2) {
                fcc->startMotor = true;
            }
            if (fcc->command == 3) {
                fcc->stopMotor = true;
            } else if (fcc->command > 3) {
                fcc->dst->stop();
                fcc->dst->disableOutputs();
                fcc->ds->stop();
                fcc->stopMotor = true;
            }
            rt_exit_critical();
        },
        &fc);
    auto telSem = rt_sem_create("tel", 0, RT_IPC_FLAG_PRIO);
    auto telTimer = rt_timer_create("telt",
        +[](void *s) { rt_sem_release(static_cast<rt_sem_t>(s)); }, telSem,
        kTelTickMs, RT_TIMER_FLAG_HARD_TIMER | RT_TIMER_FLAG_PERIODIC);
    NrfThreadCtx nrfCtx;
    nrfCtx.nrf           = &nrf;
    nrfCtx.nrfTxProtocol = &nrfTxProtocol;
    nrfCtx.nrfTxByteBuf  = &nrfTxByteBuf;
    nrfCtx.nrfRxProtocol = &nrfRxProtocol;
    nrfCtx.nrfRing       = &nrfRing;
    nrfCtx.fc            = &fc;
    nrfCtx.telSem        = telSem;
    auto nrfTid = rt_thread_create("nrf_tel", nrfTelThreadEntry, &nrfCtx, 3072, 24, 20);
    if (nrfTid != RT_NULL) rt_thread_startup(nrfTid);
    static SharedLogFrame sharedLog;
    auto logSem = rt_sem_create("log", 0, RT_IPC_FLAG_PRIO);
    auto logTimer = rt_timer_create("logt",
        +[](void *s) { rt_sem_release(static_cast<rt_sem_t>(s)); }, logSem,
        kLogTickMs, RT_TIMER_FLAG_HARD_TIMER | RT_TIMER_FLAG_PERIODIC);
    LogThreadCtx logCtx;
    logCtx.shared         = &sharedLog;
    logCtx.serialDma      = &serialDMA;
    logCtx.serialBuf      = &dmaBuffer;
    logCtx.flash          = (mode == LogMode::FlashLog && flashReady) ? &flash : nullptr;
    logCtx.flashMode      = (mode == LogMode::FlashLog);
    logCtx.flashWriteAddr = kFlashLogStart;
    logCtx.flashRegionEnd = kFlashLogStart + kFlashLogSize;
    logCtx.logSem         = logSem;
    auto logTid = rt_thread_create("logger", logThreadEntry, &logCtx, 3072, 26, 20);
    if (logTid != RT_NULL) rt_thread_startup(logTid);
    AccelerometerG lastAcc   = {{0.0f, 0.0f, 1.0f}};
    GyroscopeRads  lastGyro  = {{0.0f, 0.0f, 0.0f}};
    AccelerometerRaw lastAccRaw  = {{0, 0, 0}};
    GyroscopeRaw     lastGyroRaw = {{0, 0, 0}};
    FlightControlState fcs{};
    uint32_t lastDbgMs = 0;
    uint16_t eventFlags = 0;
    uint16_t lastCmd = 9;
    rt_timer_start(ctrlTimer);
    rt_timer_start(telTimer);
    rt_timer_start(logTimer);
    while (true) {
        rt_sem_take(ctrlSem, RT_WAITING_FOREVER);
        pf3 = 1;
        uint16_t cmd;
        float    manThr;
        FlightControlSetpoint sp;
        uint16_t motor[4];
        bool     en, starting, start, stop;
        uint32_t lastValid, motorStart;
        rt_enter_critical();
        cmd = fc.command;
        manThr = fc.manualThrottle;
        sp = fc.setpoint;
        for (int i = 0; i < 4; i++) motor[i] = fc.motorValue[i];
        en = fc.enabledMotor;
        starting = fc.startingMotor;
        start = fc.startMotor;
        stop  = fc.stopMotor;
        lastValid  = fc.lastValidCtrl;
        motorStart = fc.motorStartTime;
        fc.startMotor = false;
        fc.stopMotor  = false;
        rt_exit_critical();
        const uint32_t now_ms = rt_tick_get_millisecond();
        bmi.fifoRead(ac, gy);
        const auto gySize = gy.size();
        const auto acSize = ac.size();
        AccelerometerRaw acr;
        AccelerometerG  curAcc = lastAcc;
        uint16_t        nearestAC = 0;
        if (acSize > 0) {
            ac.pop(acr);
            curAcc    = BMI088::getAccelerationG(acr);
            remapSensorFrame(curAcc.data[0], curAcc.data[1], curAcc.data[2]);
            lastAcc   = curAcc;
            lastAccRaw = acr;
            nearestAC = 1;
        }
        uint16_t    gyCnt = 0;
        GyroscopeRaw gyr;
        while (gy.pop(gyr)) {
            if (nearestAC < acSize && ((gyCnt * acSize + gySize / 2) / gySize > nearestAC)) {
                if (ac.pop(acr)) {
                    curAcc    = BMI088::getAccelerationG(acr);
                    remapSensorFrame(curAcc.data[0], curAcc.data[1], curAcc.data[2]);
                    lastAcc   = curAcc;
                    lastAccRaw = acr;
                    nearestAC++;
                }
            }
            GyroscopeRads gyroR = BMI088::getAngularVelocityRads(gyr);
            remapSensorFrame(gyroR.data[0], gyroR.data[1], gyroR.data[2]);
            lastGyro    = gyroR;
            lastGyroRaw = gyr;
            ahrs.MadgwickAHRSupdateIMU(StandardIMUGRads{ .accel = curAcc, .gyro = gyroR });
            gyCnt++;
        }
        while (ac.pop(acr)) {
            lastAcc    = BMI088::getAccelerationG(acr);
            remapSensorFrame(lastAcc.data[0], lastAcc.data[1], lastAcc.data[2]);
            lastAccRaw = acr;
        }
        ac.clear();
        gy.clear();
        fcs.vertical_velocity_valid = false;
        fcs.attitude                = ahrs.getQuaternion();
        fcs.body_rate               = lastGyro;
        bool failsafe = false;
        if ((now_ms - lastValid) > 5000 && (en || starting)) {
            stop    = true;
            failsafe = true;
        }
        if (start) {
            fc.ds->preloadThrottle(0, 0, 0, 0);
            fc.dst->start();          // 先启动定时器时基(计数)
            fc.dst->enableOutputs();  // 再使能定时器主输出(MOE), 让 DShot 信号出现在引脚
            fc.ds->start();           // 最后启动 DMA 搬运新帧
            motorStart = now_ms;
            starting   = true;
        }
        if (stop) {
            fc.dst->stop();           // 关闭定时器计数
            fc.dst->disableOutputs(); // 关闭主输出(MOE), 引脚回到安全电平
            fc.ds->stop();            // 停止 DMA 搬运并清 transferStatus
            en       = false;
            starting = false;
        }
        if ((now_ms - motorStart) > 3200 && !en && starting) en = true;
        FlightControlOutput out;
        bool pidSat = false;
        if (cmd == 0 && en && !stop) {
            controller.updateAngle(sp, fcs, kLoopDt, out);
            out.throttle = max(manThr, 0.01f);
            const auto ctrlMotor = mixMotors(out, controller.params());
            // 逻辑电机 -> 物理 DShot 通道映射:
            //   逻辑 M0 左前 -> 通道0(电机1 左上);  M1 右前 -> 通道2(电机3 右上)
            //   M2 右后 -> 通道3(电机4 右下);        M3 左后 -> 通道1(电机2 左下)
            static const int kPhysFromLogical[4] = {0, 2, 3, 1};
            for (int i = 0; i < 4; i++) {
                motor[kPhysFromLogical[i]] = (uint16_t)(ctrlMotor.motor[i] * 1900.0f + 50.0f);
                if (motor[kPhysFromLogical[i]] <= 55u || motor[kPhysFromLogical[i]] >= 1945u) pidSat = true;
            }
        }
        if (en && !stop) fc.ds->preloadThrottle(motor);
        rt_enter_critical();
        for (int i = 0; i < 4; i++) fc.motorValue[i] = motor[i];
        fc.enabledMotor   = en;
        fc.startingMotor  = starting;
        fc.motorStartTime = motorStart;
        fc.telem.quat     = ahrs.getQuaternion();
        for (int i = 0; i < 4; i++) fc.telem.motor[i] = motor[i];
        fc.telem.height_m = fcs.height_m;
        fc.telem.error    = 0;
        fc.telem.enabled  = en;
        rt_exit_critical();
        if (cmd != lastCmd) { eventFlags |= 0x0001u; lastCmd = cmd; }
        if (start)           eventFlags |= 0x0002u;
        if (stop)            eventFlags |= 0x0004u;
        if (failsafe)        eventFlags |= 0x0008u;
        if (pidSat)          eventFlags |= 0x0010u;
        if (sharedLog.logError) eventFlags |= 0x0080u;
        if ((now_ms - lastDbgMs) >= 50) {
            lastDbgMs = now_ms;
            uint16_t status = 0;
            if ((now_ms - lastValid) <= 5000) status |= 0x01u;
            status |= 0x02u;
            if (bmiOk) status |= 0x04u;
            if (cmd == 0 && en && !stop) status |= 0x08u;
            if ((now_ms - lastValid) > 5000) status |= 0x10u;
            if (en) status |= 0x20u;
            if (mode == LogMode::FlashLog && flashReady) status |= 0x40u;
            if (starting || en) status |= 0x80u;
            FlightDebugData dbg = {};
            dbg.command     = cmd;
            dbg.state_flags = (en ? 1u : 0u) | (starting ? 2u : 0u) | (stop ? 4u : 0u) | (start ? 8u : 0u);
            dbg.status_flags = status;
            dbg.event_flags  = eventFlags;
            for (int i = 0; i < 4; i++) dbg.motor[i] = motor[i];
            const Quaternion q = ahrs.getQuaternion();
            for (int i = 0; i < 4; i++) dbg.quat[i] = q.data[i];
            const Vector3f eul = quatToEuler(q);
            dbg.euler_deg[0] = eul.z() * 57.29578f;
            dbg.euler_deg[1] = eul.y() * 57.29578f;
            dbg.euler_deg[2] = eul.x() * 57.29578f;
            for (int i = 0; i < 3; i++) dbg.body_rate[i]     = fcs.body_rate.data[i];
            for (int i = 0; i < 3; i++) dbg.rate_setpoint[i] = out.rate_setpoint.data[i];
            for (int i = 0; i < 3; i++) dbg.torque[i]        = out.torque.data[i];
            dbg.throttle        = out.throttle;
            dbg.manual_throttle = manThr;
            const Vector3f sp_e = quatToEuler(sp.attitude);
            dbg.target_pitch_deg = sp_e.y() * 57.29578f;
            dbg.target_roll_deg  = sp_e.z() * 57.29578f;
            dbg.target_height_m  = sp.height_m;
            for (int i = 0; i < 3; i++) dbg.accel_g[i] = lastAcc.data[i];
            for (int i = 0; i < 3; i++) dbg.gyro_raw[i]  = lastGyroRaw.data[i];
            for (int i = 0; i < 3; i++) dbg.accel_raw[i] = lastAccRaw.data[i];
            dbg.height_m         = fcs.height_m;
            dbg.vertical_vel_mps = fcs.vertical_velocity_mps;
            uint32_t link = now_ms - lastValid;
            dbg.link_age_ms = (uint16_t)(link > 0xFFFFu ? 0xFFFFu : link);
            dbg.error = 0;
            rt_enter_critical();
            ByteBuffer dbgBuf(sharedLog.data, kFlightDebugMaxFrame);
            DLX_ProtocolBuffer dbgProto(dbgBuf);
            const bool ok = dbgProto.FlightDebugW(
                dbg.command, dbg.state_flags, dbg.status_flags, dbg.event_flags,
                dbg.motor, dbg.quat, dbg.euler_deg, dbg.body_rate, dbg.rate_setpoint, dbg.torque,
                dbg.throttle, dbg.manual_throttle, dbg.target_pitch_deg, dbg.target_roll_deg,
                dbg.target_height_m, dbg.accel_g, dbg.gyro_raw, dbg.accel_raw,
                dbg.height_m, dbg.vertical_vel_mps, dbg.link_age_ms, dbg.error);
            sharedLog.len   = ok ? dbgProto.buffer.used() : 0;
            sharedLog.fresh = ok;
            rt_exit_critical();
        }
    }
}
