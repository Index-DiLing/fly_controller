#pragma once
#include "stm32f4xx.h"
#include "rtthread.h"
#include "dlx_gpio.hpp"
#include "dlx_iic.hpp"
#include "dlx_spi.hpp"
#include "dlx_usart.hpp"
#include "dlx_exti.hpp"
#include "dlx_adc.hpp"
#include "dlx_delay.hpp"
#include "dlx_sample_channel.hpp"
#include "BMI088/dlx_bmi088.hpp"
#include "app/boot/app_bringup.hpp"
#include "BMP388/dlx_bmp388.hpp"
#include "VL53L1X/dlx_vl53l1x.hpp"
#include "PMW3901/dlx_pmw3901.hpp"
#include "GNSS/dlx_gnss_nmea.hpp"
#include "app/runtime/release_header.h"
#include "app/runtime/app_exception_handler.h"
#include "app/sensors/app_health.hpp"
#include "app/sensors/app_measurement_reference.hpp"
#include "app/boot/app_boot_report.hpp"
#include "app/runtime/app_stack.hpp"
#include "hardware_claims.hpp"

namespace dlx
{

// 二、采集层
/** 传感器硬件、采集线程及样本通道；构造不访问寄存器，err 仅在接口失败时有效。 */
struct Sensors
{
    // 硬件对象
#if CFG_FEATURE_BARO
    IICBus baroBus{IICBus::IIC1_SB6_DB7()};
    BMP388 baro{baroBus, cfg::hw::baro::kAddr};
#endif

    // ---- 测距: I2C2(PF1=SCL/PF0=SDA) + XSHUT(PD2) + GPIO1 数据就绪(PA3 → EXTI3) ----
    GPIO     rangeXshut{cfg::hw::range_::kXshut};
    IICBus   rangeBus{IICBus::IIC2_SF1_DF0()};
    VL53L1X  range{rangeBus, cfg::hw::range_::kAddr};
    EXTILine rangeInt{cfg::hw::range_::kInt,
                      // 器件的中断极性是配置项, EXTI 的边沿必须跟着它走 —— 两个配置项
                      // 描述的是同一根线, 分开写就会对不上(而且症状是"偶尔丢一帧", 极难查)。
                      (cfg::sensor::range::kIntPolarity == cfg::RangeIntPolarity::ActiveLow)
                          ? EXTITrigger::Falling
                          : EXTITrigger::Rising};

    // ---- 光流: SPI3(PC10/11/12) + CS(PA4, 由驱动自己翻转) ----
    // 用**无片选**工厂: PMW3901 的 CS 是普通 GPIO 且要求地址→数据之间 ≥35µs,
    // 交给 SPI 类的软件片选反而会在中间插入不该有的时序。
    SPI     flowSpi{SPI::SPI3_SCA_MICB_MOCC()};
    PMW3901 flow{flowSpi, cfg::hw::flow::kCs};

    // ---- GNSS: USART2(TX=PD5/RX=PD6) 循环 DMA + IDLE 切帧 ----
    USART            gnssUsart{USART::USART2_TD5_RD6()};
    uint8_t          gnssDmaBuf[cfg::hw::gnss::kDmaRxBufBytes]{};
    USARTDmaReceiver gnssRx{gnssUsart, gnssDmaBuf, sizeof(gnssDmaBuf)};
    GnssNmeaParser   gnss{cfg::sensor::gnss::kNmeaMask};
    uint16_t         gnssReadPos{0}; ///< 本线程已经搬走的字节数(DMA 写指针由 writePos() 给)

    // ---- 电压/电流: ADC1 的 PA0(分压 9:1) / PA1(直连) ----
    ADC1Sampler adc{};

    // 快照与健康
    Health health{};
    BaroReference baroReference{};
    GnssReference gnssReference{};
    SampleSlot<BaroSample> baroSnap{};
    SampleSlot<RangeSample> rangeSnap{};
    SampleStream<FlowSample, app::FlowQueueCapacity> flowSnap{};
    SampleSlot<GnssSample> gnssSnap{};
    SampleSlot<PowerSample> powerSnap{};
    SampleStream<ImuSample, app::ImuQueueCapacity> imuSamples{};
    SPI spiImu{SPI::SPI1_SA5_MIA6_MOA7()};
    BMI088 bmi{spiImu, cfg::hw::imu::kAccelCs, cfg::hw::imu::kGyroCs};
    uint8_t accelStorage[256]{};
    uint8_t gyroStorage[256]{};
    ByteBuffer accelBuffer{accelStorage, sizeof(accelStorage)};
    ByteBuffer gyroBuffer{gyroStorage, sizeof(gyroStorage)};
    SPSCQueue<AccelerometerRaw> accelQueue{accelBuffer};
    SPSCQueue<GyroscopeRaw> gyroQueue{gyroBuffer};
    bool imuInitialized{false};
    rt_thread_t tidImu{RT_NULL};

    bool healthSet(SensorHealth source) { return health.success(source, rt_tick_get_millisecond()); }
    bool healthClear(SensorHealth source) { return health.failure(source); }

    bool bringUpImu(ExceptionRecorder &rec)
    {
        imuInitialized = dlx::bringUpImu(spiImu, bmi, rec);
        return imuInitialized;
    }

    static void imuEntry(void *context) { static_cast<Sensors *>(context)->imuLoop(); }

    void imuLoop()
    {
        paintThreadWatermark();
        AccelerometerG lastAcceleration{{0.0f, 0.0f, 1.0f}};
        uint32_t sequence = 0u;
        bool queueOverflow = false;
        uint32_t lastAccelerationMs = 0u;
        bool hasAcceleration = false;
        while (true) {
            rt_thread_mdelay(app::ImuPollMs);
            if (!imuInitialized) continue;
            bmi.fifoRead(accelQueue, gyroQueue);
            const uint16_t gyroCount = gyroQueue.size();
            const uint16_t accelCount = accelQueue.size();
            AccelerometerRaw acceleration;
            uint16_t consumedAccel = 0u;
            if (accelQueue.pop(acceleration)) {
                lastAcceleration = BMI088::getAccelerationG(acceleration);
                consumedAccel = 1u;
                lastAccelerationMs = rt_tick_get_millisecond();
                hasAcceleration = true;
            }
            bool lostSample = false;
            for (uint16_t index = 0u; index < gyroCount; ++index) {
                if (consumedAccel < accelCount && ((index * accelCount + gyroCount / 2u) / gyroCount > consumedAccel) && accelQueue.pop(acceleration)) {
                    lastAcceleration = BMI088::getAccelerationG(acceleration);
                    ++consumedAccel;
                    lastAccelerationMs = rt_tick_get_millisecond();
                    hasAcceleration = true;
                }
                GyroscopeRaw angularVelocity;
                if (!gyroQueue.pop(angularVelocity)) break;
                ImuSample sample;
                sample.acceleration = lastAcceleration;
                sample.angularVelocity = BMI088::getAngularVelocityRads(angularVelocity);
                remapSensorFrame(cfg::hw::imu::kYawDeg, sample.acceleration.data[0], sample.acceleration.data[1], sample.acceleration.data[2]);
                remapSensorFrame(cfg::hw::imu::kYawDeg, sample.angularVelocity.data[0], sample.angularVelocity.data[1], sample.angularVelocity.data[2]);
                sample.tickMs = rt_tick_get_millisecond();
                sample.sequence = ++sequence;
                if (!hasAcceleration || !sampleFresh(sample.tickMs, lastAccelerationMs, app::ImuFreshnessMs)) {
                    health.failure(SensorHealth::IMU);
                    continue;
                }
                if (!imuSamples.publish(sample)) lostSample = true;
                health.success(SensorHealth::IMU, sample.tickMs);
            }
            while (accelQueue.pop(acceleration)) {
                lastAcceleration = BMI088::getAccelerationG(acceleration);
                lastAccelerationMs = rt_tick_get_millisecond();
                hasAcceleration = true;
            }
            if (lostSample && !queueOverflow) DLX_HANDLE_EXCEPTION(ExceptionCode::IMU_FIFO_OVERRUN);
            queueOverflow = lostSample;
        }
    }

    /** 启动探测结论与运行期健康独立保存；可选器件启动失败时避免重复执行无意义的读取。 */
    struct UpFlags
    {
        bool baro{false};  ///< BMP388: init 成功(写者: bringUpBaro)
        bool range{false}; ///< VL53L1X: init 成功(写者: bringUpRange; EXTI 订阅失败**不改这里**, 见该函数)
        bool flow{false};  ///< PMW3901: init 成功; 热插拔自愈成功时由光流线程置位(写者: bringUpFlow/probeFlow)
        bool gnss{false};  ///< USART2 + 循环 DMA 起来了(写者: bringUpGnss; 有没有定位看 SensorHealth::GNSS)
        bool power{false}; ///< ADC1 起来了(写者: bringUpPower)
    };
    UpFlags up{};

    // ---- 线程 ----
    rt_sem_t    rangeSem{RT_NULL}; ///< EXTI 唤醒(带超时兜底, 见 rangeLoop)
    rt_thread_t tidBaro{RT_NULL};
    rt_thread_t tidRange{RT_NULL};
    rt_thread_t tidFlow{RT_NULL};
    rt_thread_t tidGnss{RT_NULL};

    // 启动初始化：成功看返回值，异常码仅在失败分支有效。

    /** 气压计 BMP388(I2C1)。可选件: 坏了退回纯 IMU 积分(高度/垂速会漂, 定高不可用) */
    void bringUpBaro(ExceptionRecorder &rec)
    {
#if CFG_FEATURE_BARO
        ExceptionCode err;
        baroBus.init(IICBusModeProfile::ACK_E_DC16_9_ADDR7, cfg::hw::baro::kSpeed, cfg::hw::baro::kAddr);
        const bool ok = recordCall(rec, baro.init(err), err);
        up.baro = ok;
        bootLogLine("[Dev ] BMP388  : I2C1 PB6/PB7 0x%02X init=%s CHIP_ID=0x%02X 失败计数=%u\r\n",
                static_cast<unsigned>(cfg::hw::baro::kAddr), ok ? "OK" : "FAIL",
                static_cast<unsigned>(baro.lastChipId()), static_cast<unsigned>(baro.failCount()));
#else
        // 主动禁用气压计时不访问 I2C1，也不报告器件故障。
        (void)rec;
        bootLogLine("[Dev ] BMP388  : 配置关闭(feature.kBaro=false): 高度只用测距/纯惯性积分\r\n");
#endif
    }

    /** VL53L1X 使用 I2C2、XSHUT 和 EXTI3；XSHUT 必须先保持低电平再释放。 */
    void bringUpRange(ExceptionRecorder &rec)
    {
#if CFG_FEATURE_RANGE
        ExceptionCode err;
        rangeXshut.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
        rangeXshut = 0; // 保持硬件待机(与上电期间的状态一致)
        delay_ms(2);
        rangeBus.init(IICBusModeProfile::ACK_E_DC16_9_ADDR7, cfg::hw::range_::kSpeed, cfg::hw::range_::kAddr);
        rangeXshut = 1; // 释放 XSHUT
        delay_ms(2);    // 等启动(驱动内部还会轮询 0x00E5 bit0, 这里只是给它一个干净的起点)

        bool rangeUp = range.init(err);
        // 设备初始化结果与 EXTI 注册结果独立；中断不可用时仍可超时轮询。
        up.range = rangeUp;
        if (!rangeUp) {
            rec.throws(err);
        }
        if (rangeUp) {
            // 数据就绪中断: 一条线一个订阅者(见 dlx_exti.hpp); 重复注册会返回 false 而不是静默覆盖。
            // 中断里只许可 rt_sem_release(它在下半部跑, 不是真的在 ISR 里做 I2C)。
            if (!rangeInt.init(NVICPriorityProfile::P2_S2, &Sensors::onRangeIrq, this)) {
                // 中断订阅失败与"器件没响应"是两件事: 前者查 EXTI 线/引脚, 后者查 I2C 与供电。
                err = ExceptionCode::RANGE_INT_REGISTER_FAILED;
                rec.throws(err);
                rangeUp = false;
            }
        }
        if (rangeUp) {
            bootLogLine("[Dev ] VL53L1X : I2C2 PF1/PF0 0x%02X XSHUT=PD2 INT=PA3(EXTI%u) init=OK\r\n",
                    static_cast<unsigned>(cfg::hw::range_::kAddr),
                    static_cast<unsigned>(static_cast<uint8_t>(cfg::hw::range_::kInt) & 0x0Fu));
        } else {
            // 成功时**不打 err**: 那条路上它没有意义(见本结构体头的约定), 打了只会被人当成一个码去查。
            bootLogLine("[Dev ] VL53L1X : I2C2 PF1/PF0 0x%02X XSHUT=PD2 INT=PA3(EXTI%u) init=FAIL "
                    "**err=0x%04X**\r\n",
                    static_cast<unsigned>(cfg::hw::range_::kAddr),
                    static_cast<unsigned>(static_cast<uint8_t>(cfg::hw::range_::kInt) & 0x0Fu),
                    static_cast<unsigned>(err));
        }
        // 三个 ID 都要带**期望值**一起打: 身份校验同时看 modelId 与 moduleType, 而两者不一致时
        // `init()` 都返回 RANGE_NOT_FOUND —— 只打读到的值, 读日志的人没法判断是"没读到"还是"读到了但不对"。
        bootLogLine("[Dev ] VL53L1X : ID 读回 ModelID=0x%02X(期望 0x%02X) 模块类型=0x%02X(期望 0x%02X) "
                "Rev=0x%02X(期望 0x%02X) 中断极性=%s\r\n",
                static_cast<unsigned>(range.getModelId()), static_cast<unsigned>(VL53L1X_MODEL_ID),
                static_cast<unsigned>(range.getModuleType()), static_cast<unsigned>(VL53L1X_MODULE_TYPE),
                static_cast<unsigned>(range.getRevisionId()), static_cast<unsigned>(VL53L1X_REVISION_ID),
                range.isActiveLow() ? "低有效" : "高有效");
        if (!rangeUp) {
            bootLogLine("[Dev ] VL53L1X : 诊断: %s\r\n", rangeErrorHint(err));

            if (err == ExceptionCode::RANGE_ULD_LOAD_FAILED) {
                    /** ULD 写失败后回读对应寄存器，区分写入未完成与成功判据不匹配。 */
                    uint8_t    wb[2] = {0x00u, 0x2Du}; // 寄存器 0x002D(高字节在前, DS §4.2)
                    uint8_t    rbv[16] = {0u};
                    ByteBuffer wbuf(wb, sizeof(wb));
                    ByteBuffer rbuf(rbv, sizeof(rbv));
                    wbuf.write(wb, sizeof(wb));
                    IICDevice dev(rangeBus, cfg::hw::range_::kAddr);
                    if (dev.read(rbuf, wbuf, sizeof(rbv))) {
                        for (uint8_t i = 0u; i < 16u; i += 8u) {
                            bootLogLine("[Dev ] VL53L1X : ULD 回读 0x%04X = %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                                    0x002Du + i, static_cast<unsigned>(rbv[i]), static_cast<unsigned>(rbv[i + 1u]),
                                    static_cast<unsigned>(rbv[i + 2u]), static_cast<unsigned>(rbv[i + 3u]),
                                    static_cast<unsigned>(rbv[i + 4u]), static_cast<unsigned>(rbv[i + 5u]),
                                    static_cast<unsigned>(rbv[i + 6u]), static_cast<unsigned>(rbv[i + 7u]));
                        }
                        for (uint8_t i = 0u; i < 16u; i += 8u) {
                            bootLogLine("[Dev ] VL53L1X : ULD 期望 0x%04X = %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                                    0x002Du + i,
                                    static_cast<unsigned>(VL51L1X_DEFAULT_CONFIGURATION[i]), static_cast<unsigned>(VL51L1X_DEFAULT_CONFIGURATION[i + 1u]), static_cast<unsigned>(VL51L1X_DEFAULT_CONFIGURATION[i + 2u]), static_cast<unsigned>(VL51L1X_DEFAULT_CONFIGURATION[i + 3u]), static_cast<unsigned>(VL51L1X_DEFAULT_CONFIGURATION[i + 4u]), static_cast<unsigned>(VL51L1X_DEFAULT_CONFIGURATION[i + 5u]), static_cast<unsigned>(VL51L1X_DEFAULT_CONFIGURATION[i + 6u]), static_cast<unsigned>(VL51L1X_DEFAULT_CONFIGURATION[i + 7u]));
                        }
                        bootLogLine("[Dev ] VL53L1X : ⇒ 一致 = 其实写进去了(判据太严); "
                                "不一致 = 表内容或 93 字节事务有问题\r\n");
                        // 把**超时的是哪个 I2C 事件**也打出来: 这是"总线异常"里唯一有信息量的部分
                        bootLogLine("[Dev ] VL53L1X : I2C2 事件超时=%u 最近超时事件掩码=0x%08X"
                                "(对 RM0090 事件表: 0x00030001=TXE|BTF|BUSY 等)\r\n",
                                static_cast<unsigned>(rangeBus.eventTimeouts()),
                                static_cast<unsigned>(rangeBus.lastFailedEvent()));
                    } else {
                        bootLogLine("[Dev ] VL53L1X : ULD 回读**也失败** ⇒ 问题在总线层(93 字节事务发不出去), "
                                "不是表内容\r\n");
                    }
            }
        }
#else
        (void)rec;
        bootLogLine("[Dev ] VL53L1X : 配置关闭(feature.kRange=false): 高度没有测距来源\r\n");
#endif
    }

    /** 将测距初始化失败映射为具体排查提示。 */
    static const char *rangeErrorHint(ExceptionCode err)
    {
        switch (err) {
        case ExceptionCode::RANGE_NOT_FOUND:
            return "模型 ID 读不到 ⇒ 查 I2C2(PF1/PF0) 上拉与供电、XSHUT(PD2) 是否真的被拉高";
        case ExceptionCode::RANGE_BOOT_TIMEOUT:
            return "XSHUT 释放后器件没在预期时间内起来(0x00E5 bit0 一直不清) ⇒ 查供电与 XSHUT 时序";
        case ExceptionCode::RANGE_IO_VOLTAGE_FAILED:
            return "2V8 I/O 模式没切过去(0x002E/0x002F) ⇒ 不切就可能完全不通";
        case ExceptionCode::RANGE_ULD_LOAD_FAILED:
            return "**91 字节 ULD 配置表写入失败** —— 这是本驱动唯一没有第二份来源的部分(欠账 #11), "
                   "优先怀疑表内容/写入时序, 而不是接线(I2C 已经能读 ID 了)";
        case ExceptionCode::RANGE_INVALID:
            return "量程无效(目标太近/太远/反光): 运行时提示, 不是初始化故障";
        case ExceptionCode::RANGE_DATA_STALE:
            return "长时间没有新测距: 查中断线(PA3/EXTI3)与测量周期";
        default:
            return "未分类的测距故障";
        }
    }

    /** 光流 PMW3901(SPI3 + CS=PA4)。可选件: 坏了速度环就不能进 VEL_HOLD */
    void bringUpFlow(ExceptionRecorder &rec)
    {
#if CFG_FEATURE_FLOW
        ExceptionCode err;
        flowSpi.init(cfg::hw::flow::kSpi);
        const bool flowUp = flow.init(err);
        up.flow = flowUp;
        if (!flowUp) {
            rec.throws(err);
        }
        // SPI3 由光流独占，避免其他设备重新配置 SPI 模式。
        logSpiStatus("SPI3 光流", SPI3, cfg::hw::flow::kSpi, apb1ClockHz());
        if (flowUp) {
            bootLogLine("[Dev ] PMW3901 : CS=PA4 init=OK PID=0x%02X(期望 0x49) "
                    "反码=0x%02X(期望 0xB6) 回读校验=%s\r\n",
                    static_cast<unsigned>(flow.productIdReg()),
                    static_cast<unsigned>(flow.inverseProductIdReg()), flow.initVerifyOk() ? "过" : "不过");
        } else {
            // 成功时不打 err(那条路上它没意义, 见本结构体头的约定)
            bootLogLine("[Dev ] PMW3901 : CS=PA4 init=FAIL **err=0x%04X** PID=0x%02X(期望 0x49) "
                    "反码=0x%02X(期望 0xB6) 回读校验=%s\r\n",
                    static_cast<unsigned>(err),
                    static_cast<unsigned>(flow.productIdReg()),
                    static_cast<unsigned>(flow.inverseProductIdReg()), flow.initVerifyOk() ? "过" : "不过");
        }
        if (!flowUp) {
            // ID 读出来是什么, 直接决定该查哪一段 —— 这几种情况的排查动作毫无交集:
            const uint8_t pid = flow.productIdReg();
            const uint8_t inv = flow.inverseProductIdReg();
            const char   *hint;
            if (pid == PMW3901_PRODUCT_ID && inv == PMW3901_INVERSE_PRODUCT_ID) {
                // 2026-10-03 实测遇到: 接线修好后 ID 全对, 但**回读校验**不过(err=0x0603)。
                // 这一档的结论很重要: **SPI 读是通的, 问题在写**(厂家初始化表没落地)。
                hint = "**ID 全对但回读校验不过** ⇒ SPI 读通了、问题在**写**: 厂家初始化表没落地。"
                       "看下面的逐条明细 —— 全 0/全 FF = 写完全没生效; 少数几条 = 表里期望值抄错";
            } else if (pid == 0x00u && inv == 0x00u) {
                hint = "**MISO 恒低** ⇒ 模块没供电 / MISO(PC11) 没接 / CS(PA4) 没接或没被拉低";
            } else if (pid == 0xFFu && inv == 0xFFu) {
                hint = "**MISO 恒高** ⇒ MISO 被别的外设驱动着(查 PC11 是否被复用), 或模块没上电";
            } else {
                hint = "ID 读到了但不对 ⇒ 十有八九是 bit7 读写方向或 SPI 模式(本驱动按模式 3)";
            }
            bootLogLine("[Dev ] PMW3901 : 诊断: %s\r\n", hint);
            /** 输出光流关键寄存器回读值，区分 SPI 无响应、配置表错误及 bank 切换失败。 */
            bootLogLine("[Dev ] PMW3901 : 回读校验: 不一致 %u/%u 条, 第一条下标=%u\r\n",
                    static_cast<unsigned>(flow.verifyBadCount()),
                    static_cast<unsigned>(PMW3901_VERIFY_TABLE_SIZE),
                    static_cast<unsigned>(flow.verifyFirstBadIndex()));
            uint8_t shown = 0u;
            for (uint8_t i = 0u; i < static_cast<uint8_t>(PMW3901_VERIFY_TABLE_SIZE) && shown < 8u; ++i) {
                const PMW3901_VerifyStep &st  = PMW3901_VERIFY_TABLE[i];
                const uint8_t             got = flow.verifyRawAt(i);
                if (got == st.val) {
                    continue; // 只打不一致的, 免得刷屏
                }
                ++shown;
                bootLogLine("[Dev ] PMW3901 :   #%u bank=%u 寄存器=0x%02X 期望=0x%02X **实得=0x%02X**\r\n",
                        static_cast<unsigned>(i), static_cast<unsigned>(st.bank),
                        static_cast<unsigned>(st.reg), static_cast<unsigned>(st.val),
                        static_cast<unsigned>(got));
            }
            if (shown == 0u) {
                bootLogLine("[Dev ] PMW3901 :   (没有记录到不一致的条目 —— 校验过程本身没跑完)\r\n");
            }
        } else if (flow.verifyBadCount() > 0u) {
            /** 非严格模式允许部分保留寄存器不匹配，但必须输出回读证据。 */
            const uint8_t bi = flow.verifyFirstBadIndex();
            rec.throws(ExceptionCode::FLOW_VERIFY_FAILED);
            bootLogLine("[Dev ] PMW3901 : ⚠ 回读校验 %u/%u 条不符(**非严格模式已放行**): 第 %u 条 "
                    "期望0x%02X 实得0x%02X\r\n",
                    static_cast<unsigned>(flow.verifyBadCount()),
                    static_cast<unsigned>(PMW3901_VERIFY_TABLE_SIZE), static_cast<unsigned>(bi),
                    static_cast<unsigned>(PMW3901_VERIFY_TABLE[bi].val),
                    static_cast<unsigned>(flow.verifyRawAt(bi)));
        }
#else
        (void)rec;
        bootLogLine("[Dev ] PMW3901 : 配置关闭(feature.kFlow=false): 速度环不能进 VEL_HOLD\r\n");
#endif
    }

    /** GNSS NEO-7M(USART2 + 循环 DMA)。可选件: 没有它只是没有水平速度观测 */
    void bringUpGnss(ExceptionRecorder &rec)
    {
#if CFG_FEATURE_GNSS
        // 循环 DMA + IDLE 切帧。缓冲必须 ≥ 一帧长 + 余量(见 dlx_usart.hpp 的三条约束):
        // NMEA 最长 82 字符, 512 字节给了 6 倍余量, 1Hz/9600bps 下绝不会被覆盖。
        gnssRx.start(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, cfg::hw::gnss::kBaud,
                     NVICPriorityProfile::P1_S1);
        // 串口本身起来了; "有没有定位"是运行期的事 —— 那一项由 gnssLoop 维护(SensorHealth::GNSS)。
        up.gnss = true;
        gnssReadPos = 0;
        bootLogLine("[Dev ] GNSS    : USART2 TX=PD5 RX=PD6 %u 8N1 循环DMA=%uB + IDLE 切帧, "
                "语句掩码=0x%02X(1=RMC 2=GGA 4=GSA 8=VTG) 过期=%ums 最小星数=%u HDOP≤%u\r\n",
                static_cast<unsigned>(cfg::hw::gnss::kBaud),
                static_cast<unsigned>(cfg::hw::gnss::kDmaRxBufBytes),
                static_cast<unsigned>(cfg::sensor::gnss::kNmeaMask),
                static_cast<unsigned>(cfg::sensor::gnss::kStaleMs),
                static_cast<unsigned>(cfg::sensor::gnss::kMinSats),
                static_cast<unsigned>(cfg::sensor::gnss::kMaxHdop * 10.0f + 0.5f));
#else
        (void)rec;
        bootLogLine("[Dev ] GNSS    : 配置关闭(feature.kGnss=false): 没有水平速度观测\r\n");
#endif
    }

    /** 电压/电流 ADC1(PA0 分压 9:1 / PA1 直连)。**必需件**: 没有它就没有低压保护 */
    void bringUpPower(ExceptionRecorder &rec)
    {
#if CFG_FEATURE_ADC
        ExceptionCode err;
        ADC1Config    cfgAdc;
        cfgAdc.voltageChannel      = cfg::hw::adc::kVoltageCh;
        cfgAdc.currentChannel      = cfg::hw::adc::kCurrentCh;
        cfgAdc.sampleCount         = cfg::hw::adc::kSampleCount;
        cfgAdc.vrefMv              = cfg::power::kVrefMv;
        cfgAdc.voltageDividerRatio = cfg::power::kVoltageDividerRatio;
        cfgAdc.voltageOffsetV      = static_cast<float>(cfg::power::kVoltageOffsetMv) * 0.001f;
        cfgAdc.currentOffsetV      = cfg::power::kCurrentOffsetV;
        cfgAdc.currentAmpsPerVolt  = cfg::power::kCurrentAmpsPerVolt;
        cfgAdc.cellCount           = cfg::power::kCellCount;
        adc.setConfig(cfgAdc);
        const bool powerUp = recordCall(rec, adc.init(claims::kAdcVoltagePin, claims::kAdcCurrentPin, err), err);
        up.power = powerUp;
        bootLogLine("[Dev ] 电源    : PA0=电压(1/%u 分压) PA1=电流(%.2f A/V, 零点 %.2fV) "
                "init=%s ⇒ 满量程约 %.0fA\r\n",
                static_cast<unsigned>(cfg::power::kVoltageDividerRatio),
                static_cast<double>(cfg::power::kCurrentAmpsPerVolt),
                static_cast<double>(cfg::power::kCurrentOffsetV), powerUp ? "OK" : "FAIL",
                static_cast<double>((cfg::power::kVrefMv * 0.001f - cfg::power::kCurrentOffsetV) *
                                    cfg::power::kCurrentAmpsPerVolt));
#else
        (void)rec;
        rec.throws(ExceptionCode::POWER_ADC_FAILED);
        bootLogLine("[Dev ] 电源    : 配置关闭(feature.kAdc=false): **没有低压保护**, 因此禁止解锁\r\n");
#endif
    }

    // 采集线程
    static void onRangeIrq(uint8_t line, void *ctx)
    {
        (void)line;
        Sensors *self = static_cast<Sensors *>(ctx);
        if (self->rangeSem != RT_NULL) {
            rt_sem_release(self->rangeSem); // RT-Thread 的 IPC 允许在中断里调用
        }
    }

    /** 线程入口(把 this 传进来; 采集循环都在成员函数里) */
    static void baroEntry(void *arg) { static_cast<Sensors *>(arg)->baroLoop(); }
    static void rangeEntry(void *arg) { static_cast<Sensors *>(arg)->rangeLoop(); }
    static void flowEntry(void *arg) { static_cast<Sensors *>(arg)->flowLoop(); }
    static void gnssEntry(void *arg) { static_cast<Sensors *>(arg)->gnssLoop(); }

    /** IMU 使用独立采集线程；其他采集线程按功能开关创建。栈大小和优先级由运行期常量定义。 */
    void startThreads(uint8_t prio, ExceptionRecorder &rec)
    {
        auto noteThread = [&rec](rt_thread_t &tid) {
            if (tid != RT_NULL && rt_thread_startup(tid) != RT_EOK) {
                rt_thread_delete(tid);
                tid = RT_NULL;
            }
            if (tid == RT_NULL) {
                rec.throws(ExceptionCode::SCHED_THREAD_START_FAILED);
            }
        };
        tidImu = rt_thread_create("sns_imu", &Sensors::imuEntry, this, app::ImuThreadStackBytes, app::ImuThreadPriority, 20);
        noteThread(tidImu);
#if CFG_FEATURE_RANGE
        rangeSem = rt_sem_create("rng", 0, RT_IPC_FLAG_PRIO);
        if (rangeSem != RT_NULL) tidRange = rt_thread_create("sns_rng", &Sensors::rangeEntry, this, app::SensorThreadStackBytes, prio, 20);
        noteThread(tidRange);
#endif
#if CFG_FEATURE_BARO || CFG_FEATURE_ADC
        // 气压计线程同时负责电源采样(见 samplePower): 只要两者有一个开着就得建它
        tidBaro = rt_thread_create("sns_baro", &Sensors::baroEntry, this, app::SensorThreadStackBytes, prio, 20);
        noteThread(tidBaro);
#endif
#if CFG_FEATURE_FLOW
        tidFlow = rt_thread_create("sns_flow", &Sensors::flowEntry, this, app::SensorThreadStackBytes, prio, 20);
        noteThread(tidFlow);
#endif
#if CFG_FEATURE_GNSS
        tidGnss = rt_thread_create("sns_gnss", &Sensors::gnssEntry, this, app::SensorThreadStackBytes, prio, 20);
        noteThread(tidGnss);
#endif
    }

    /** 实际上起来了几个采集线程(诊断行用: 与"该建几个"对比着看才能分清"关掉"和"建失败") */
    uint8_t startedThreads() const
    {
        uint8_t n = tidImu != RT_NULL ? 1u : 0u;
        if (tidRange != RT_NULL) ++n;
        if (tidBaro != RT_NULL) ++n;
        if (tidFlow != RT_NULL) ++n;
        if (tidGnss != RT_NULL) ++n;
        return n;
    }

    /** 上电时该建几个采集线程(用于把"建失败"与"配置关掉"分开上报) */
    static constexpr uint8_t expectedThreads()
    {
        return static_cast<uint8_t>(1u +
#if CFG_FEATURE_RANGE
            1u +
#endif
#if CFG_FEATURE_BARO || CFG_FEATURE_ADC
            1u +
#endif
#if CFG_FEATURE_FLOW
            1u +
#endif
#if CFG_FEATURE_GNSS
            1u +
#endif
            0u);
    }

    // 气压计线程同时采样电源；有效样本更新时间，失败只在可用性变化时报告。
    void baroLoop()
    {
        paintThreadWatermark();
#if CFG_FEATURE_BARO
        /**
         * 连续失败计数(2026-10-18): 第二道防线 —— 连续失败到 cfg::sensor::baro::kFailLimit 就在**本线程**
         * 里做一次"总线恢复 + 器件重初始化"。
         *
         * 为什么不把恢复交给控制线程(它确实会收到 0x404):
         *   1) `healthClear()` 只在"可用→不可用"的**边沿**返回 true(`app_health.cpp:55-62`), 所以抛给控制线程的
         *      异常**只会出现一次** —— 之后连续失败不再上报, 靠它触发恢复等于只有一次机会;
         *   2) 总线有唯一使用者(本线程)。让控制线程去 re-init 会把"正在 waitEvent 里的本线程"和"改外设的
         *      控制线程"叠在一起, 属于跨线程抢同一条总线, 破坏所有权假设;
         *   3) 空转的代价发生在**本线程**里(台架实测单次超时 ≈64ms), 恢复动作离故障点越近越好。
         *   控制线程那边保留原有语义: 收 0x404 → 刷新健康 → 决定降级(不用气压计), 与本恢复互不影响。
         *
         * 为什么用局部计数而不是 baro.failCount(): 后者只统计 I2C 事务失败, 覆盖不到 comp 类失败
         * (事务成功但补偿结果非法); 这里按"任意一次 read() 失败"计, 两类都能兜住。
         */
        uint32_t consecutiveFails = 0u;
        while (true) {
            rt_thread_mdelay(cfg::time::io::kTickMs);
            samplePower(); // 电源采样搭这趟车(见 samplePower 的说明: 它是最便宜的一次总线事务)
            if (!up.baro) {
                // 启动探测失败时跳过重复读取，避免阻塞同线程的电源采样。
                continue;
            }
            ExceptionCode err;
            float         pressurePa = 0.0f, temperatureC = 0.0f;
            const BMP388::ReadResult result = baro.read(pressurePa, temperatureC, err);
            if (result == BMP388::ReadResult::NewData) {
                consecutiveFails = 0u;
                BaroSample sample;
                sample.pressurePa = pressurePa;
                sample.temperatureC = temperatureC;
                sample.altitudeM = baro.getAltitude(pressurePa);
                sample.tickMs = rt_tick_get_millisecond();
                if (!baroReference.apply(sample)) {
                    if (healthClear(SensorHealth::BARO)) DLX_HANDLE_EXCEPTION(ExceptionCode::BARO_READ_FAILED);
                    continue;
                }
                baroSnap.publish(sample);
                baroReference.published();
                health.success(SensorHealth::BARO, sample.tickMs);
            } else if (result == BMP388::ReadResult::Failed) {
                if (++consecutiveFails >= cfg::sensor::baro::kFailLimit) {
                    consecutiveFails = 0u;
                    ExceptionCode initErr = err;
                    // 先救总线(现在会重跑 I2C_Init + 清 AF/BERR/ARLO/OVR), 再让器件重新走一遍
                    // "软复位 -> 校验 CHIP_ID -> 读标定 -> 写配置 -> 回读"。两条都要: 总线级故障只需前者,
                    // 器件自己卡住/NVM 读坏只有后者能救。
                    baroBus.recover();
                    const FunctionResult initResult = baro.tryInit(initErr);
                    // 成功时 initErr 没有被写(驱动的约定), 不能拿它去打印 —— 先算一个"只在失败时有效"的码。
                    const uint16_t initErrCode =
                        (initResult == FunctionResult::SUCCESS) ? 0u : static_cast<uint16_t>(initErr);
                    logLine("[Baro] 连续失败 %u 次 -> 总线恢复 + 器件重初始化: %s (err=0x%04X)\r\n",
                            static_cast<unsigned>(cfg::sensor::baro::kFailLimit),
                            initResult == FunctionResult::SUCCESS ? "OK" : "FAIL",
                            static_cast<unsigned>(initErrCode));
                }
                if (healthClear(SensorHealth::BARO)) DLX_HANDLE_EXCEPTION(err);
            }
        }
#else
        while (true) {
            rt_thread_mdelay(cfg::time::io::kTickMs);
            samplePower(); // 气压计被编译期关掉时, 这个线程仍然负责电源采样
        }
#endif
    }

    // 测距由 EXTI 信号唤醒，并使用超时轮询兜底。
    void rangeLoop()
    {
        paintThreadWatermark();
        const uint32_t fallbackTicks =
            rt_tick_from_millisecond(cfg::time::range::kFallbackMs > 0u ? cfg::time::range::kFallbackMs : 100u);
        while (true) {
            rt_sem_take(rangeSem, fallbackTicks);
            if (!up.range) {
                continue;
            }
            ExceptionCode err;
            float         range_m = 0.0f;
            uint8_t       status  = 0;
            // 分流看**返回值**: "还没测完(NotReady)"与"真失败(Failed)"过去都靠 err 是 0 还是别的来分,
            // 于是驱动不得不造一个"没有异常"的码出来。现在状态由 RangeRead 直接回答(见 dlx_vl53l1x.hpp)。
            switch (range.readIfReady(range_m, status, err)) {
            case VL53L1X::RangeRead::NewData:
                {
                    if (!std::isfinite(range_m) || range_m < cfg::sensor::range::kMinValidMm * 0.001f ||
                        range_m > cfg::sensor::range::kMaxValidMm * 0.001f) {
                        health.failure(SensorHealth::RANGE);
                        break;
                    }
                    RangeSample s;
                    s.rangeM     = range_m;
                    s.status     = status;
                    s.rawMm      = range.getLastRawMm();
                    s.peakSignal = range.getLastPeakSignal();
                    s.ambient    = range.getLastAmbient();
                    s.tickMs     = rt_tick_get_millisecond();
                    rangeSnap.publish(s);
                    health.success(SensorHealth::RANGE, s.tickMs);
                }
                break;
            case VL53L1X::RangeRead::NotReady:
                // 这一帧还没测完: **不是故障**(间隔寄存器 50ms > 唤醒周期时这最常见), 什么都不做
                break;
            case VL53L1X::RangeRead::Invalid:
                health.failure(SensorHealth::RANGE);
                break;
            case VL53L1X::RangeRead::Failed:
                if (healthClear(SensorHealth::RANGE)) {
                    DLX_HANDLE_EXCEPTION(err); // 从"能用"翻到"不能用": 只报这一次
                }
                break;
            }
        }
    }

    // 光流采集使用固定周期；初始化失败时按退避策略继续探测。
    void flowLoop()
    {
        paintThreadWatermark();
        uint32_t lastOkMs = rt_tick_get_millisecond();
        while (true) {
            rt_thread_mdelay(cfg::time::flow::kPeriodMs > 0u ? cfg::time::flow::kPeriodMs : 10u);
            if (!up.flow) {
                // 光流未就绪时持续退避探测，不设固定失败次数上限。
                probeFlow();
                continue;
            }
            ExceptionCode err;
            int16_t       dx = 0, dy = 0;
            uint8_t       squal = 0;
            if (flow.readMotion(dx, dy, squal, err)) {
                const uint32_t nowMs = rt_tick_get_millisecond();
                FlowSample     s;
                s.dx     = dx;
                s.dy     = dy;
                s.squal  = squal;
                s.dtMs   = static_cast<uint16_t>((nowMs - lastOkMs) > 0xFFFFu ? 0xFFFFu : (nowMs - lastOkMs));
                s.tickMs = nowMs;
                lastOkMs = nowMs;
                if (s.squal >= app::FlowMinimumQuality) health.success(SensorHealth::FLOW, s.tickMs);
                else health.failure(SensorHealth::FLOW);
                if (!flowSnap.publish(s)) health.failure(SensorHealth::FLOW); // 唯一的出口: 控制环从快照里取, 光流本身不再有日志
            } else if (err != ExceptionCode::FLOW_LOW_QUALITY) {
                // 质量低不是"读失败"(器件照样给了数据), 只是这一帧不该用; 只有真读错才动健康位
                if (healthClear(SensorHealth::FLOW)) {
                    DLX_HANDLE_EXCEPTION(err); // 从"能用"翻到"不能用": 只报这一次
                }
            }
        }
    }

    // 运行期光流探测执行复位与配置；失败按退避策略重试，成功后进入采集。
    void probeFlow()
    {
        const uint32_t now = rt_tick_get_millisecond();
        if ((now - lastFlowProbeMs) < kFlowProbePeriodMs) {
            return; // 周期限制在 1s: 一轮 init 最长 ~200ms, 别把线程占满
        }
        lastFlowProbeMs = now;
        ++flowProbes;

        ExceptionCode err;
        const bool    ok  = flow.init(err); // 内含: CS 复位脉冲 + 写 0x3A + 读 ID(重试 3 次)
        const uint8_t pid = flow.productIdReg();
        const uint8_t inv = flow.inverseProductIdReg();
        if (ok) {
            logLine("[Flow] 探测 #%u: SPI3 读 0x00→0x%02X(期望 0x49) 0x5F→0x%02X(期望 0xB6) "
                    "init=OK\r\n",
                    static_cast<unsigned>(flowProbes), static_cast<unsigned>(pid), static_cast<unsigned>(inv));
        } else {
            logLine("[Flow] 探测 #%u: SPI3 读 0x00→0x%02X(期望 0x49) 0x5F→0x%02X(期望 0xB6) "
                    "init=FAIL err=0x%04X\r\n",
                    static_cast<unsigned>(flowProbes), static_cast<unsigned>(pid), static_cast<unsigned>(inv),
                    static_cast<unsigned>(err));
        }
        if (ok) {
            // 热插拔自愈: 从这一刻起这个器件算"起来了", 采集循环下一轮就去正常读它
            up.flow = true;
            logLine("[Flow] **探测成功** —— 模块接上了, 已自动完成初始化(不用重新烧录)\r\n");
            return;
        }
        // 诊断只在头几次打, 免得 1Hz 的日志被同一段话刷满
        if (flowProbes <= 3u) {
            const char *hint;
            if (pid == 0x00u && inv == 0x00u) {
                hint = "**MISO 恒低** ⇒ 先查模块 3.3V 供电, 再查 MISO(PC11) 与 CS(PA4) 两根线";
            } else if (pid == 0xFFu && inv == 0xFFu) {
                hint = "**MISO 恒高** ⇒ MISO 被别的外设驱动(查 PC11 复用)或模块没上电";
            } else if (pid == 0x80u) {
                hint = "MISO 读回的就是 MOSI 发出去的地址字节 ⇒ **MISO 与 MOSI 短接**";
            } else {
                hint = "ID 读到了但不对 ⇒ 查 SPI 模式(本驱动按模式 3)、SCK(PC10)、以及丝印 MISO/SDA 是否接反";
            }
            logLine("[Flow] 诊断: %s\r\n", hint);
            logLine("[Flow] 抓波形要点: CS=PA4(每轮先一个 2ms 复位脉冲), SCK=PC10(1.3125MHz, 模式3), "
                    "MOSI=PC12(先 0x80 再 0x00), MISO=PC11(应在第 2 字节给出 0x49); "
                    "每轮按 %ums 重复\r\n",
                    static_cast<unsigned>(kFlowProbePeriodMs));
        }
        if (flowProbes == 1u) {
            DLX_HANDLE_EXCEPTION(ExceptionCode::FLOW_NOT_FOUND); // 只报一次(后面每轮探测都报会刷屏)
        }
    }

    /** 探测周期 [ms] */
    static constexpr uint32_t kFlowProbePeriodMs = 1000u;
    uint32_t lastFlowProbeMs{0};
    uint32_t flowProbes{0};

    // GNSS 在线程中轮询 DMA 缓冲并解析 NMEA，避免在 IDLE 中断中运行解析器。
    void gnssLoop()
    {
        uint32_t lastPublishedSolution = 0u;
        paintThreadWatermark();
        while (true) {
            rt_thread_mdelay(cfg::time::gnss::kPeriodMs > 0u ? cfg::time::gnss::kPeriodMs : 100u);
            const uint16_t cap = gnssRx.capacity();
            if (cap == 0u) {
                continue;
            }
            const uint32_t nowMs = rt_tick_get_millisecond();
            uint16_t       pos   = gnssRx.writePos();
            // 从"上次读到的地方"搬到"DMA 刚写到的地方"。跨回绕要分两段 ——
            // 这不是理论问题: 512 字节在 9600bps 下约 0.53s 就转一圈。
            while (gnssReadPos != pos) {
                const uint16_t take = static_cast<uint16_t>(
                    (pos > gnssReadPos) ? (pos - gnssReadPos) : (cap - gnssReadPos));
                for (uint16_t i = 0; i < take; ++i) {
                    gnss.feed(gnssDmaBuf[(gnssReadPos + i) % cap], nowMs);
                }
                gnssReadPos = static_cast<uint16_t>((gnssReadPos + take) % cap);
            }
            // 覆盖过 = 缓冲太小或上层太慢, 必须能看见; 只在计数**变化**时报一次(否则每 100ms 一条)
            static uint32_t reportedOverruns = 0u;
            if (gnssRx.overruns() != reportedOverruns) {
                reportedOverruns = gnssRx.overruns();
                // 只报"溢出过"这一个事实: 异常出口的签名只有 ExceptionCode(app_exception_handler.h),
                // 计数在这里只用来做"变化才报一次"的判据, 不往外带。
                DLX_HANDLE_EXCEPTION(ExceptionCode::GNSS_UART_OVERRUN);
            }
            // 解析器每接受一句 solSeq +1; 只有真的变了才发布新快照(否则会把同一份解重复喂给 EKF)
            const GnssSolution &sol = gnss.solution();
            if (sol.seq != lastPublishedSolution) {
                const GnssSample sample = gnssReference.convert(sol,
                    static_cast<uint8_t>(cfg::sensor::gnss::kMinSats),
                    static_cast<uint16_t>(cfg::sensor::gnss::kMaxHdop * 10.0f + 0.5f));
                if (sample.posValid) {
                    health.success(SensorHealth::GNSS, sample.positionMs);
                } else if (healthClear(SensorHealth::GNSS)) {
                    DLX_HANDLE_EXCEPTION(ExceptionCode::GNSS_NO_FIX);
                }
                gnssSnap.publish(sample);
                lastPublishedSolution = sol.seq;
            }
            ExceptionCode ge;
            if (gnss.takeError(ge)) {
                DLX_HANDLE_EXCEPTION(ge); // 校验和错/溢出/无定位: 统一出口
            }
        }
    }

    // 电源采样复用气压计线程，无需额外线程栈。
    void samplePower()
    {
        if (!up.power) {
            return;
        }
        ExceptionCode err;
        float         packV = 0.0f, amps = 0.0f;
        if (adc.readSupply(packV, amps, err) == FunctionResult::SUCCESS) {
            PowerSample s;
            s.packV  = packV;
            s.amps   = amps;
            s.tickMs = rt_tick_get_millisecond();
            s.valid = true;
            powerSnap.publish(s);
            health.success(SensorHealth::POWER, s.tickMs);
        } else if (healthClear(SensorHealth::POWER)) {
            // 采样失败 → 没有可信电压 → 低压保护失效。只在翻转时报一次(这里 20ms 一拍, 报多了就是刷屏)。
            DLX_HANDLE_EXCEPTION(ExceptionCode::POWER_SAMPLE_FAILED);
        }
    }

    /** 线程栈水位线(只有 rtthread 依赖; 见 app_stack.hpp) */
    static void paintThreadWatermark() { paintStackWatermark(); }
};

} // namespace dlx
