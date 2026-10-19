#pragma once
// 飞行运行入口：启动流程、控制周期及控制线程私有状态。
#include "app/channels/app_flight_shared.hpp"
#include "app/link/app_link_thread.hpp"
#include "app/log/app_log_thread.hpp"
#include "app/sensors/app_sensors.hpp"
#include "app/runtime/app_exception_handler.h"
#include "app/sensors/app_health.hpp"
#include "app/boot/app_bringup.hpp"
#include "app/boot/app_boot_report.hpp"
#include "app/runtime/app_stack.hpp"
#include "dlx_gpio.hpp"
#include "dlx_spi.hpp"
#include "dlx_iic.hpp"
#include "dlx_dma.hpp"
#include "dlx_delay.hpp"
#include "dlx_dshot.hpp"
#include "dlx_timer.hpp"
#include "BMI088/dlx_bmi088.hpp"
#include "flight_control/flight_controller.hpp"
#include "flight_control/flight_control_mixer.hpp"
#include "app/control/app_state.h"
#include "dlx_locked_cell.hpp"
#include "app/control/app_task.hpp"
#include "flight_control/flight_control_fliter.hpp"
#include "flight_control/optical_flow_estimator.hpp"
#include "flight_control/velocity_controller.hpp"
#include "W25Q128/dlx_flash_manager.hpp"

namespace dlx
{

/** 控制线程拥有的飞行状态及生命周期；构造顺序遵循硬件依赖，构造时不访问寄存器。 */
struct FlightStack
{
    // 0. 外部上下文(main 建好的串口/flash/协议对象)与本次运行的结论
    ReleaseContext &app;

    // 1. 共享区 + 线程(控制 / 链路 / 日志)
    PublishedFlight published{};
    LogStatus cycleLogStatus{};
    FlightShared sh{};
    LinkThreadCtx linkCtx{};
    LogThreadCtx  logCtx{};
    rt_sem_t      ctrlSem{RT_NULL};
    rt_timer_t    ctrlTimer{RT_NULL};
    rt_sem_t      linkSem{RT_NULL};
    rt_timer_t    linkTimer{RT_NULL};
    rt_sem_t      logSem{RT_NULL};
    rt_timer_t    logTimer{RT_NULL};

    // 2. 硬件对象(构造不碰寄存器; 真正的初始化在各 bringUpXxx 里)

    //传感器们
    Sensors sensors{};

    USART linkUsart{USART::USART3_TD8_RD9()};
    E220  link{linkUsart, cfg::hw::link::kM0, cfg::hw::link::kM1, cfg::hw::link::kAux};
    uint8_t        linkDmaBuf[cfg::hw::link::kDmaRxBufBytes]{};
    /** 链路 v2 接收: USART3 循环 DMA + IDLE 切帧。IDLE 是"帧末"的硬件证据(也是 D0 时刻的依据),
     *  比 v1 的 RXNE 逐字节环形缓冲多一个关键信息: 每段突发的**到达时刻**(ISR 侧 DWT 周期戳)。 */
    USARTDmaReceiver linkRx{linkUsart, linkDmaBuf, sizeof(linkDmaBuf)};

    // ---- 电机(DShot) ----
    // 注意: `dshot` 的构造就是 `setDshot()`, 会配 TIM1 时基和 DMA —— 见本文件头部说明。
    AdvancedTimer timer1{cfg::hw::motor::kTimer};
    Dshot         dshot{timer1.setDshot(cfg::hw::motor::kDshot)};

    // 控制算法实例，仅由控制线程访问。
    FlightControlFilter ekf{};             ///< 16 维 EKF(慢路径的 15×15 协方差就在里面: ~1.1KB)
    FlightController    controller{};      ///< 姿态/角速度控制器(PID 见 flight_control_params.hpp)

    // ---- 控制环输入队列(IMU FIFO 出队用) ----

    // ---- 链路报文的收发缓冲 ----
    bool startupReady{false};

    // 观测融合状态，由控制线程更新并派生遥测和日志。
    uint32_t lastBaroStamp{0};   ///< 上次喂给 EKF 的气压样本发布序号
    uint32_t lastRangeStamp{0};  ///< 上次喂给 EKF 的测距快照序号
    uint32_t lastGnssVelocitySequence{0};   ///< 上次消费的 GNSS 速度测量序号
    bool     flowValid{false};   ///< 本圈光流估计有效(速度环/日志的许可位)
    FlowSample lastFlow{};       ///< 最近一份光流快照(日志要记原始增量与质量)
    uint16_t flowFlags{0};       ///< LogEntry::flowFlags(哪几路水平观测此刻有效)
    float    rangeM{0.0f};       ///< 最近一次有效测距 [m](日志用)
    OpticalFlowEstimator flowEst{}; ///< counts → 机体系速度(参数在 initObservers 里从 cfg 填)
    VelocityController   velCtrl{}; ///< 速度环(参数同上)

    uint32_t lastBaroReferenceSequence{0u};

    // 控制环状态
    FlightControlState fcs{};
    AccelerometerG     lastAcc{{0.0f, 0.0f, 1.0f}};
    GyroscopeRads      lastGyro{{0.0f, 0.0f, 0.0f}};
    FlightControlOutput out{};
    Quaternion          q{};        ///< 本圈发布出去的姿态(EKF)
    uint16_t            eventFlags{0};
    uint16_t            lastCmd{9};
    uint16_t            imuCnt{0};
    float               imuDtAccum{0.0f};
    bool                baroUpdated{false}; ///< 本周期是否更新了气压高度(关闭气压计时恒 false)
    bool                pidSat{false};      ///< 本圈有一路电机输出饱和
    uint32_t            lastLogMs{0};
    uint32_t            lastPerfMs{0};
    uint32_t            lastLoopCyc{0};
    uint32_t            logCounter{0};
    uint32_t publishSequence{0u};
    uint32_t            perfWarmup{cfg::time::perf::kWarmupLoops}; ///< 起跑热身圈(见 stepPerf)
    const uint32_t      logPeriodMs; ///< 飞行 50Hz / 调试 100Hz(见 cfg::time::log)

    // ---- 本圈从共享区取出来的量(stepSampleShared 填, 后面几步读) ----
    uint32_t              now_ms{0};
    uint32_t              loopStartCyc{0};
    uint32_t              lastValid{0};
    uint16_t              cmd{9};
    uint16_t              motor[4]{50, 50, 50, 50};
    float                 manThr{0.0f};
    FlightControlSetpoint sp{};
    bool                  bootReadyNow{false}; ///< 上电期门槛(stepStateMachine 每圈算一次)
    static constexpr uint8_t kTasksPerCycle = app::RequestsPerCycle;
    app::Request taskBatch[kTasksPerCycle]{};
    app::ControlInput input{};
    HealthSnapshot cycleHealth{};
    uint32_t lastImuSequence{0u};
    uint32_t lastFlowAcceptedMs{0u};
    uint32_t lastHeightAcceptedMs{0u};
    bool heightObserved{false};
    bool flowObserved{false};
    bool imuGapThisCycle{false};
    bool loopOverrun{false};
    bool emergencyThisCycle{false};
    uint32_t handledEmergencyId{0u};
    uint32_t directInputFloor{0u};
    bool emergencyAckPending{false};
    app::RequestResult pendingResults[app::RequestsPerCycle]{};
    uint8_t pendingResultCount{0u};
    bool faultStopLatched{false};

    // ---- 融合状况调试采样(仅串口调试模式) ----
    /**
     * 只有 SerialDebug 模式有串口出口(日志线程 debugStream); Flight 模式调试串口静音,
     * 因此不产生调试采样 —— 这样飞行热路径上连队列写入都不会发生。
     */
    bool debugSamplesEnabled{false};
    /** 自上次调试记录以来各观测通道的最新样本与处置结果(控制线程私有, 不跨线程)。 */
    struct DebugSampling
    {
        BaroSample  baro{};
        RangeSample range{};
        FlowSample  flow{};
        uint16_t    flags{0};
    };
    DebugSampling debugSampling{};

    bool healthOk(SensorHealth source) const
    {
        return cycleHealth.ok(source, now_ms);
    }

    bool inputFresh() const
    {
        return app::controlInputFresh(input, mode, now_ms);
    }

    void stopOutput(bool emergency)
    {
        applyEffects(app::effectStopPower | app::effectPurgeDirect);
        state = app.mode == BootMode::Flight ? app::SystemState::Standby : app::SystemState::Maintenance;
        stateEnteredMs = now_ms;
        operation = app::Operation::None;
        opStage = 0u;
        hoverLandActive = false;
        altTargetValid = false;
        hardStop = hardStop || emergency;
        for (uint8_t index = 0u; index < 4u; ++index) motor[index] = 0u;
        sh.control.safetyEpoch.update([](uint32_t &epoch) { ++epoch; });
    }

    void flushResults()
    {
        if (emergencyAckPending) {
            app::RequestResult result;
            result.kind = app::ResultKind::EmergencyStop;
            result.id = handledEmergencyId;
            if (sh.control.results.tryPush(result)) emergencyAckPending = false;
        }
        uint8_t sent = 0u;
        while (sent < pendingResultCount && sh.control.results.tryPush(pendingResults[sent])) ++sent;
        for (uint8_t index = sent; index < pendingResultCount; ++index) pendingResults[index - sent] = pendingResults[index];
        pendingResultCount = static_cast<uint8_t>(pendingResultCount - sent);
    }

    void stepSafety()
    {
        emergencyThisCycle = false;
        if (input.emergencyStopPending && input.emergencyStopId != handledEmergencyId) {
            stopOutput(true);
            handledEmergencyId = input.emergencyStopId;
            emergencyAckPending = true;
            emergencyThisCycle = true;
        }
        flushResults();
    }
    uint8_t               taskCount{0u};      ///< 本圈开始时排空的请求数

    // 生命周期、模式与操作由控制线程统一维护。
    app::SystemState         state{app::SystemState::Boot};                 ///< 7 态扁平枚举
    app::SystemState         armOrigin{app::SystemState::Standby};           ///< 本次解锁前所在地面态
    app::Mode           mode{app::Mode::Stabilize};              ///< 由控制环写(链路只发请求)
    app::Operation      operation{app::Operation::None};         ///< 会自己结束的那一类请求
    uint8_t             opStage{0};                              ///< 操作子阶段(驱动斜坡, 不上线)
    /** 失效保护缓降状态，由控制线程维护。 */
    bool                hoverLandActive{false};
    /** 控制侧急停锁存；内部 ACK 不解除此状态。 */
    bool                hardStop{false};
    uint32_t            stateEnteredMs{0u}; ///< 控制线程独占: 进入当前状态的时间

    // 高度目标及降级交接。
    /** 四种飞行阶段**共用**的那个高度指令 [m](定高/定速/起降/失效保护降落都从这里取) */
    float            altTargetM{0.0f};
    bool             altTargetValid{false};
    app::Mode        modePrev{app::Mode::Stabilize}; ///< 上一圈的模式(用来抓"进入/掉出某模式"的边沿)
    /** 降级或操作交接时先保持悬停油门，待飞行员给出非零油门后交还。 */
    float       manThrEff{0.0f};
    bool        manThrHold{false};

    app::AltitudeUse taskAlt{app::AltitudeUse::None};
    /** 操作把高度交回自稳时的一次性交接标志(见 stepPilotThrottle: 那一档油门不能被交给 0) */
    bool             taskHandoverToStab{false};

    // 状态迁移的执行侧：每次迁移只执行一次对应硬件动作。

    /** 执行状态迁移对应的硬件动作。 */
    void applyEffects(uint8_t effects)
    {
        if ((effects & app::effectStartPower) != 0u) {
            dshot.preloadThrottle(0, 0, 0, 0);
            timer1.start();         // 先跑时基
            timer1.enableOutputs(); // 再开主输出(MOE), 让 DShot 信号上引脚
            dshot.start();          // 最后启动 DMA
            eventFlags |= kLogEventArm;
            if (sensors.up.baro) sensors.baroReference.request();
        }
        if ((effects & app::effectTakeover) != 0u) {
            stepBaroObs();
            ekf.reset(fcs.attitude, Vector3f{}, Vector3f{});
            fcs.height_m = 0.0f;
            fcs.vertical_velocity_mps = 0.0f;
            eventFlags |= kLogEventEkfRezero;
        }
        if ((effects & app::effectStopPower) != 0u) {
            timer1.stop();
            timer1.disableOutputs();
            dshot.stop();
            eventFlags |= kLogEventDisarm;
        }
        if ((effects & app::effectPurgeDirect) != 0u) {
            /** 清除控制线程本地直控值，避免复用旧电机指令。 */
            rt_enter_critical();
            for (uint8_t i = 0u; i < link::upMotor::kMotorCount; ++i) {
                input.motorValue[i] = 0u;
            }
            rt_exit_critical();
        }
    }

    // 生命周期
    explicit FlightStack(ReleaseContext &ctx)
        : app(ctx)
        , logPeriodMs((ctx.mode == BootMode::Flight) ? cfg::time::log::kFlightPeriodMs
                                                     : cfg::time::log::kDebugPeriodMs)
    {
        // 用 ctx.mode 而不是 app.mode: 成员初始化顺序虽然保证 app 先于 logPeriodMs, 但构造函数体
        // 里直接读**入参**最不依赖顺序。这一段跑在 "app.mode 已经定下来" 之后(见 release.cpp)。
        if (ctx.mode == BootMode::Flight && ctx.fs == nullptr) {
            DLX_HANDLE_EXCEPTION(ExceptionCode::LOG_NO_SINK);
        }
        // 融合状况调试流只走串口调试模式: 其他模式没有出口(或在跑另一套循环), 不产生队列写入。
        debugSamplesEnabled = (ctx.mode == BootMode::SerialDebug);
    }

    // 传感器初始化：具体总线访问及诊断由 Sensors 完成。
    void bringUpSensors(ExceptionRecorder &bringUpRec)
    {
        bootLogLine("[CFG ] 编译期外设: 气压计=%s 测距=%s 光流=%s GNSS=%s 电源=%s 链路=%s 详细日志=%s\r\n",
                cfg::feature::kBaro ? "开" : "**关**", cfg::feature::kRange ? "开" : "**关**",
                cfg::feature::kFlow ? "开" : "**关**", cfg::feature::kGnss ? "开" : "**关**",
                cfg::feature::kAdc ? "开" : "**关**", cfg::feature::kLink ? "开" : "**关**",
                cfg::feature::kVerboseBootLog ? "开" : "**关**");
        const bool imuUp = sensors.bringUpImu(bringUpRec);
        (void)imuUp;
        // 慢速传感器: 每路一个函数(接线 + 失败分类 + 结论进同一个记录器, 见 app_sensors.hpp)
        sensors.bringUpBaro(bringUpRec);
        sensors.bringUpRange(bringUpRec);
        sensors.bringUpFlow(bringUpRec);
        sensors.bringUpGnss(bringUpRec);
        sensors.bringUpPower(bringUpRec);
    }

    // 生命周期 2: 电机(时基已在构造里配好, 这里只做寄存器回读与 USART 状态)
    void bringUpMotors()
    {
        logDshotRegisterCheck(); // 必须打在 setDshot() 之后: 对比期望值与寄存器实际值
        logUsartStatus();
        // 这一行是自检报告里最该先看的一行: 三个"能不能干活"的前提。
        // 注意用词 —— 气压计被编译期关掉时写 OFF(而不是 FAIL): 主动关掉不等于坏掉。
        // flash 的判据是**文件系统真的就绪**(isReady), 不是"指针非空":
        // 指针非空只说明句柄存在, init 失败时它指向的是一个未就绪的对象(见 release.cpp).
        bootLogLine("[Boot] BMI088=%s BARO=%s flash=%s\r\n", sensors.imuInitialized ? "OK" : "FAIL",
                !cfg::feature::kBaro ? "OFF(编译期)" : (sensors.up.baro ? "OK" : "FAIL"),
                (app.fs != nullptr && app.fs->isReady()) ? "OK"
                                                        : (app.fs != nullptr ? "未就绪" : "-"));
    }

    // 无线链路初始化：走线诊断、配置及透传自检在线程启动前完成。
    struct LinkBootReport
    {
        char linkVer[24]{};
        char linkType[28]{};
        uint32_t linkAttempts{0u};
        bool linkKeyWritten{false};
    };

    bool bringUpLink(ExceptionRecorder &bringUpRec)
    {
#if CFG_FEATURE_LINK
        LinkBootReport report;
        /** 走线诊断在 begin() 前执行：比较上拉/下拉电平，并恢复 AUX 与串口引脚复用。 */
        const E220::LineHealth lh = E220::probeLines(cfg::hw::link::kAux, cfg::hw::link::kRx);
        bootLogLine("[Dev ] E220    : 走线自检 AUX=%s/外部驱动=%s 模块TXD=%s/外部驱动=%s(内部下拉压不住 = 有驱动)\r\n",
                lh.auxHigh ? "高" : "低", lh.auxDriven ? "有" : "**无**",
                lh.rxHigh ? "高" : "低", lh.rxDriven ? "有" : "**无**");
        if (!lh.rxDriven) {
            bootLogLine("[Dev ] E220    : ⇒ 模块 TXD 这根线上没有外部驱动: 模块没上电、或 TXD→PD9 断线 —— "
                    "这是「收字节一直=0」的头号嫌疑(AT 能通只证明 PD8 出去的那一半还活着)\r\n");
        }
        if (!lh.auxDriven) {
            bootLogLine("[Dev ] E220    : ⇒ AUX 上没有外部驱动: 只影响自检判据(运行期不读 AUX), 但会让"
                    "「忙脉冲」这条证据失效 —— 见 configureLink 的 ⑤\r\n");
        }
        linkUsart.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, cfg::hw::link::kCfgBaud);
        bool linkUp = false;
        // 整段重试(而不是每一小步各重试): 这条链路上任何一步失败, 后面的步骤都没有意义,
        // 而且电气故障(供电/接线)往往表现为"第一步就先失败"。
        for (uint32_t attempt = 0; attempt < cfg::hw::link::kCfgRetry && !linkUp; ++attempt) {
            ++report.linkAttempts;
            linkUp = configureLink(bringUpRec, report);
            if (!linkUp) {
                delay_ms(50); // 给模块一点时间(它刚被 XSHUT/上电时序折腾过)
            }
        }
        if (linkUp) {
            bootLogLine("[Boot] · 链路: 启动 DMA+IDLE 接收(USART3 @%u)\r\n",
                    static_cast<unsigned>(cfg::hw::link::kBaud));
            // 链路 v2: 循环 DMA + IDLE 切帧(不再是 RXNE 逐字节)。IDLE = 帧末的硬件证据;
            // D0 的发送时刻 = "U3 末字节 + guard", 没有 IDLE 时间戳就只能靠 6ms 定时器(精度不够)。
            // 先装 handler 再启动接收: start() 里会在清完 IDLE 挂起后使能中断,
            // 若此时 _handler 还是空, 第一段真实数据会被丢弃(启动瞬间正好是 RC 发 HELLO 的时候)。
            linkRx.setHandler(&linkRxBurstIsr, &linkCtx);
            linkRx.start(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, cfg::hw::link::kBaud,
                         NVICPriorityProfile::P1_S0);
            bootLogLine("[Boot] · 链路: 接收已启动(IDLE 切帧)\r\n");
            /** 透传自检通过发送短帧观察 AUX 忙窗口，区分无线发送与仍停留在配置模式；仅在线程启动前执行。 */
            uint8_t  probe[8] = {0u};
            uint32_t busyMs   = 0u;
            const bool txProved = link.txProbe(probe, sizeof(probe), busyMs);
            bootLogLine("[Dev ] E220    : 透传自检=%s(AUX 忙 %ums, 8B; v2 guard=%ums)\r\n",
                    txProved ? "OK" : "**FAIL**", static_cast<unsigned>(busyMs),
                    static_cast<unsigned>(LinkGuardUs / 1000u));
            if (!txProved) {
                bootLogLine("[Dev ] E220    : ⇒ 模块没在透传模式(大概率还留在配置模式): 空口既不发也不收。"
                        "查 M0/M1 是否真的到了模块(上面 ⓪ 的引脚自检)、以及 AUX 线\r\n");
            }
            // 先置上健康项: 否则链路线程第一次 poll 会把它当成"从失同步恢复"而报一条 LINK_RESYNCED,
            // 每个上电周期都多一条假提示(边沿限流的代价就是初值必须是对的)。

        }
        bootLogLine("[Dev ] E220    : USART3 PD8(TX)/PD9(RX) M0=PD3 M1=PD4 AUX=PA2 配置=%s(试了 %u 轮) "
                "空速码=%u(寄存器 %u ⇒ %u bps) baud=%u ch=%u\r\n",
                linkUp ? "OK" : "FAIL", static_cast<unsigned>(report.linkAttempts),
                static_cast<unsigned>(cfg::link::e220::kAirRate), static_cast<unsigned>(airRateCode()),
                static_cast<unsigned>(e220AirRateBps(static_cast<E220AirRateCode>(airRateCode()))),
                static_cast<unsigned>(cfg::hw::link::kBaud),
                static_cast<unsigned>(cfg::link::e220::kChannel));
        bootLogLine("[Dev ] E220    : 型号=%s 固件=%s(AUX超时=%u 忙脉冲缺=%u 应答异常=%u 寄存器不一致=%u)\r\n",
                (report.linkType[0] != '\0') ? report.linkType : "(没答话)",
                (report.linkVer[0] != '\0') ? report.linkVer : "(没答话)",
                static_cast<unsigned>(link.auxTimeouts()), static_cast<unsigned>(link.auxBusyMisses()),
                static_cast<unsigned>(link.badReplies()), static_cast<unsigned>(link.regMismatches()));
        // 密钥: 写不写得进只能靠"链路能通"间接证明(AT+KEY 只写不可读), 但至少要说明**我们写过**,
        // 且两端写的是同一个值 —— 遥控器侧写 CRYPT_H/L={0,0}, 这里是 0x%04X。
        bootLogLine("[Dev ] E220    : 密钥=%s(0x%04X, 只写不可读; 遥控器侧必须写同一把)\r\n",
                report.linkKeyWritten ? "已写" : "**写入失败**",
                static_cast<unsigned>(cfg::link::e220::kKey));
        /** 记录 AUX 初始电平和等待时长，区分模块忙与接线异常。 */
        {
            const bool auxHigh        = link.auxLevel();
            const bool auxHighAtBoot  = link.auxLevelAtBoot();
            const unsigned waitMs     = static_cast<unsigned>(link.auxWaitMsLast());
            const char *verdict =
                (!auxHighAtBoot && waitMs >= static_cast<unsigned>(E220_POWERUP_AUX_TIMEOUT_MS))
                    ? "AUX 上电就读到低且等到超时 ⇒ 查 AUX 接线/上拉(拔掉模块量 PA2 应 ~3.3V; 0V 说明板上有下拉, "
                      "模块上电会被它拉低 >1s 而进 IAP 升级模式, 必须去掉下拉)"
                    : (!auxHigh ? "AUX 现在为低(模块忙或线被拉低); 若配置成功则不影响使用"
                                : "AUX 正常(高 = 空闲)");
            bootLogLine("[Dev ] E220    : AUX(PA2) 现在=%s 上电时=%s 最近一次等待=%ums ⇒ %s\r\n",
                    auxHigh ? "高" : "低", auxHighAtBoot ? "高" : "低", waitMs, verdict);
        }
        /** 回读 M0/M1 实际电平，检查配置成功后是否真正切回透传。 */
        bootLogLine("[Dev ] E220    : M0/M1 焊盘回读=%s/%s 与目标不符=%u 次(不符 ⇒ 线被外部拽住, "
                "模块会停在别的模式)\r\n",
                link.m0PadHigh() ? "高" : "低", link.m1PadHigh() ? "高" : "低",
                static_cast<unsigned>(link.padMismatches()));
        if (!linkUp) {
            bootLogLine("[Dev ] E220    : 诊断: %s\r\n", linkErrorHint(bringUpRec));
            // 失败要报出去: 没有链路 = 收不到指令 = 解不了锁, 这跟"线程建失败"是同一类后果。
            // 具体是哪一种失败(模块没答话 / AUX 不动 / 值被钳位)已经在记录器里了 —— 这里不重复记。
        }
        return linkUp;
#else
        (void)bringUpRec;
        // 链路被编译期关掉: 不碰 USART3, 也不建链路线程与软定时器。
        // 后果必须说清楚: **没有遥控 = 收不到任何指令 = 解不了锁**, 只能当地面观察用。
        bootLogLine("[Dev ] 链路    : 配置关闭(feature.kLink=false): 收不到指令 ⇒ 不能解锁; 不建链路线程\r\n");
        return false;
#endif
    }

#if CFG_FEATURE_LINK
    /** 配置空中速率与 REG0 寄存器码不是同一枚举，必须显式映射。 */
    static uint8_t airRateCode()
    {
        switch (cfg::link::e220::kAirRate) {
        case cfg::AirRate::k2_4k:  return static_cast<uint8_t>(E220AirRateCode::K2_4_C); // 码 0/1/2 都是 2.4k, 取出厂默认
        case cfg::AirRate::k4_8k:  return static_cast<uint8_t>(E220AirRateCode::K4_8);
        case cfg::AirRate::k9_6k:  return static_cast<uint8_t>(E220AirRateCode::K9_6);
        case cfg::AirRate::k19_2k: return static_cast<uint8_t>(E220AirRateCode::K19_2);
        case cfg::AirRate::k38_4k: return static_cast<uint8_t>(E220AirRateCode::K38_4);
        case cfg::AirRate::k62_5k: return static_cast<uint8_t>(E220AirRateCode::K62_5);
        default:                   return static_cast<uint8_t>(E220AirRateCode::K62_5);
        }
    }

    /** 本工程期望的 REG0..REG3(回读校验的"期望值"与自检打印都用它, 只有这一份) */
    static E220Regs expectedLinkRegs()
    {
        E220Regs regs;
        regs.reg0 = E220::buildReg0(static_cast<uint8_t>(cfg::link::e220::kUartBaud),
                                    static_cast<uint8_t>(cfg::link::e220::kParity), airRateCode());
        regs.reg1 = E220::buildReg1(static_cast<uint8_t>(cfg::link::e220::kPacketLen),
                                    static_cast<uint8_t>(cfg::link::e220::kTxPower), cfg::link::e220::kDrssi);
        regs.reg2 = static_cast<uint8_t>(cfg::link::e220::kChannel);
        regs.reg3 = E220::buildReg3(cfg::link::e220::kDrssi, false /* 透传, 不用定点发送 */,
                                    cfg::link::e220::kLbt, 0 /* WOR 周期: 本工程不用 WOR */);
        return regs;
    }

    /** 将驱动失败原因映射为启动异常码。 */
    static ExceptionCode linkFailCode(E220::Fail f)
    {
        switch (f) {
        case E220::Fail::AuxTimeout:  return ExceptionCode::LINK_AUX_TIMEOUT;
        case E220::Fail::RxTimeout:   return ExceptionCode::LINK_AT_TIMEOUT;
        case E220::Fail::BadReply:    return ExceptionCode::LINK_AT_REPLY_BAD;
        case E220::Fail::RegMismatch: return ExceptionCode::LINK_REG_CLAMPED;
        case E220::Fail::BadParam:    return ExceptionCode::LINK_BAD_ARGUMENT;
        case E220::Fail::None:
        default:                      return ExceptionCode::EXCEPTION_UNKNOWN;
        }
    }

    /** 一条 AT 指令失败时记下"为什么"(成功什么都不记: 正常路径不该往记录器里塞东西) */
    void noteLinkAt(bool ok, ExceptionRecorder &rec)
    {
        if (!ok) {
            rec.throws(linkFailCode(link.lastFail()));
        }
    }

    /** 使用 AT 配置并回读校验；直接写 REG0 会改变串口速率，破坏后续配置通信。 */
    bool applyLinkConfig(ExceptionRecorder &rec, LinkBootReport &report)
    {
        const E220Regs regs = expectedLinkRegs();
        char           cmd[40];

        // ① 串口速率 + 校验(改的是模块自己的 UART, 必须由模块来切)
        snprintf(cmd, sizeof(cmd), "AT+UART=%u,%u", static_cast<unsigned>(cfg::link::e220::kUartBaud),
                 static_cast<unsigned>(cfg::link::e220::kParity));
        noteLinkAt(link.atSet(cmd), rec);

        // ② 空中速率 / 分包 / 信道 / 发射功率
        noteLinkAt(link.setAirRate(airRateCode()), rec);
        noteLinkAt(link.setPacketCode(static_cast<uint8_t>(cfg::link::e220::kPacketLen)), rec);
        noteLinkAt(link.setChannel(static_cast<uint8_t>(cfg::link::e220::kChannel)), rec);
        snprintf(cmd, sizeof(cmd), "AT+POWER=%u", static_cast<unsigned>(cfg::link::e220::kTxPower));
        noteLinkAt(link.atSet(cmd), rec); // 驱动没包装 AT+POWER(手册 §7.1.21), 直接发

        // ③ 透传 / LBT / 数据RSSI / 地址 / 断帧 / AUX
        //    地址**无条件写**(0 也要写): 模块里残留上一次实验的值会表现为"配置全对但收不到"。
        noteLinkAt(link.setTransparentMode(true), rec);
        noteLinkAt(link.setLbt(cfg::link::e220::kLbt), rec);
        noteLinkAt(link.setDrssi(cfg::link::e220::kDrssi), rec);
        noteLinkAt(link.setAddr(static_cast<uint16_t>(cfg::link::e220::kAddr)), rec);
        noteLinkAt(link.setURxt(static_cast<uint8_t>(cfg::link::e220::kUrxt)), rec);
        /** 写入链路密钥并保留成功结果；该寄存器不支持回读校验。 */
        report.linkKeyWritten = link.setKey(static_cast<uint16_t>(cfg::link::e220::kKey));
        noteLinkAt(report.linkKeyWritten, rec);
        /** 开启 AT+UAUX，使 AUX 指示无线发送忙闲，供时分调度使用。 */
        noteLinkAt(link.setUAux(cfg::link::e220::kUaux), rec);

        // 回读地址与 REG0..REG3；密钥不可回读，单独保存写入结果。
        const uint16_t addr    = static_cast<uint16_t>(cfg::link::e220::kAddr);
        const uint8_t  addrH   = static_cast<uint8_t>((addr >> 8) & 0xFFu);
        const uint8_t  addrL   = static_cast<uint8_t>(addr & 0xFFu);
        uint8_t        rb[6]   = {0u, 0u, 0u, 0u, 0u, 0u};
        const bool     rbOk    = link.readRegs(static_cast<uint8_t>(E220Reg::Addh), rb, 6u);
        const bool     match   = rbOk && rb[0] == addrH && rb[1] == addrL && rb[2] == regs.reg0 &&
                             rb[3] == regs.reg1 && rb[4] == regs.reg2 && rb[5] == regs.reg3;
        if (!rbOk) {
            rec.throws(linkFailCode(link.lastFail()));
        } else if (!match) {
            rec.throws(ExceptionCode::LINK_CONFIG_VERIFY_FAILED);
        }

        // 诊断(无条件调用: 打不打由 logLine 自己决定, 见 app_log.hpp)
        bootLogLine("[Dev ] E220    : 配置(全程 AT)回读=%s ADDR=%02X%02X REG0..3=%02X %02X %02X %02X"
                "(期望 ADDR=%02X%02X REG0..3=%02X %02X %02X %02X)\r\n",
                rbOk ? "OK" : "失败", static_cast<unsigned>(rb[0]), static_cast<unsigned>(rb[1]),
                static_cast<unsigned>(rb[2]), static_cast<unsigned>(rb[3]), static_cast<unsigned>(rb[4]),
                static_cast<unsigned>(rb[5]), static_cast<unsigned>(addrH), static_cast<unsigned>(addrL),
                static_cast<unsigned>(regs.reg0), static_cast<unsigned>(regs.reg1),
                static_cast<unsigned>(regs.reg2), static_cast<unsigned>(regs.reg3));
        if (!match) {
            // **原始回读**是这一步唯一能定位的东西: FF FF FF = 命令格式不对; C1 开头但参数不同 = 值被钳位;
            // 空 = 模块根本没回话。见 E220::lastHexReply() 的说明。
            char           hex[64];
            const uint8_t  n = link.lastHexReplyLength();
            const uint8_t *p = link.lastHexReply();
            uint16_t       w = 0;
            if (n == 0u) {
                w = static_cast<uint16_t>(snprintf(hex, sizeof(hex), "(空: 模块一个字都没回)"));
            } else {
                for (uint8_t i = 0u; i < n && (w + 4u) < sizeof(hex); ++i) {
                    w = static_cast<uint16_t>(w + snprintf(hex + w, sizeof(hex) - w, "%02X ", p[i]));
                }
                hex[w] = '\0';
            }
            bootLogLine("[Dev ] E220    : HEX 回读(%u 字节)= %s\r\n", static_cast<unsigned>(n), hex);
        }
        return match;
    }

    /** 完成引脚初始化、进入配置模式、写入参数、回读校验及返回透传。 */
    bool configureLink(ExceptionRecorder &rec, LinkBootReport &report)
    {
        /** AUX 超时仅作诊断，模块是否存在由配置模式的串口应答确认。 */
        bool       auxUnverified = false;
        const bool auxReady      = link.begin();
        if (!link.enterConfigMode()) { // ② 切配置模式 + 用 AT 主动自证(判据在 AT)
            rec.throws(linkFailCode(link.lastFail()));
            return false;
        }
        if (!auxReady) {
            auxUnverified = true; // AUX 没在超时内变高: 留痕, 但不拦配置
        }
        uint16_t vlen = 0;
        if (link.queryVersion(report.linkVer, sizeof(report.linkVer), vlen)) {
            report.linkVer[sizeof(report.linkVer) - 1u] = '\0';
        } else {
            report.linkVer[0] = '\0';
            // 版本读不到**不拦配置**: 固件世代未知不是致命故障, 但必须留痕
            rec.throws(linkFailCode(link.lastFail()));
        }
        // 型号与固件同批读: 两端型号不同时, 同一份寄存器**可能**对应不同的空速/信道表 ——
        // 那种故障的症状恰好是"配置逐字节一致却互相收不到", 只有型号能把它和空口问题分开。
        {
            uint16_t tlen = 0;
            if (link.queryDeviceType(report.linkType, sizeof(report.linkType), tlen)) {
                report.linkType[sizeof(report.linkType) - 1u] = '\0';
            } else {
                report.linkType[0] = '\0'; // 读不到型号同样不拦配置(老固件可能不支持这条查询)
            }
        }
        if (!applyLinkConfig(rec, report)) {
            return false;
        }
        /** 返回透传后诊断 AUX；AUX 接线异常不覆盖已取得的串口配置证据。 */
        if (!link.enterTransparentMode()) {
            if (link.lastFail() == E220::Fail::AuxTimeout) {
                delay_ms(E220_AUX_RELOAD_FALLBACK_MS);
                auxUnverified = true;
            } else {
                rec.throws(linkFailCode(link.lastFail()));
                return false;
            }
        }
        if (auxUnverified) {
            rec.throws(ExceptionCode::LINK_AUX_TIMEOUT); // 最后只记一笔, 不重复计数
        }
        return true;
    }
#endif // CFG_FEATURE_LINK

    /** 为链路初始化失败提供排查提示。 */
    static const char *linkErrorHint(const ExceptionRecorder &rec)
    {
        if (rec.has(ExceptionCode::LINK_AUX_TIMEOUT)) {
            // 2026-10-06: AUX 超时**不再**等于"链路配不上"(见 configureLink 的 ①⑤)。所以这条提示
            // 必须把"现在还算不算故障"说清楚, 否则读日志的人会去拆一根其实无关的线。
            return "AUX(PA2) 没在超时内变空闲 ⇒ 查 AUX 接线与上拉(拔掉模块量 PA2 应约 3.3V; 读 0V = 板上有下拉, "
                   "会让模块上电进 IAP 升级模式, 必须去掉); 若上面那行 `配置=OK` 则链路照常可用 —— "
                   "运行期不读 AUX(kAuxGate 未接线)";
        }
        if (rec.has(ExceptionCode::LINK_MODULE_NOT_FOUND)) {
            return "AT 完全不答话 ⇒ 查模块供电(22dBm 型号要 ≥3.3V)、TX/RX 是否**交叉**(PD8→模块RXD, PD9←模块TXD)、"
                   "M0/M1 是否真的到了 (1,1)";
        }
        if (rec.has(ExceptionCode::LINK_AT_TIMEOUT)) {
            return "AT 通道没自证成功 ⇒ 查 TX/RX 是否**交叉**、模块是否真的进了配置模式、串口是否切到 9600";
        }
        if (rec.has(ExceptionCode::LINK_AT_REPLY_BAD)) {
            return "模块回了、但不是期望的应答 ⇒ 看上面那行 HEX 回读: FF FF FF = 命令格式不对";
        }
        if (rec.has(ExceptionCode::LINK_REG_CLAMPED) || rec.has(ExceptionCode::LINK_CONFIG_VERIFY_FAILED)) {
            return "寄存器写+回读不一致 ⇒ 看上面那行 HEX 回读: C1 开头但参数不同 = 值被模块钳位(改值); "
                   "空 = 模块根本没回话";
        }
        return "未分类的链路故障";
    }
    // 生命周期 4: 线程(控制环在 run() 里, 这两个是链路与日志)
    static rt_timer_t createPeriodicTimer(const char *name, rt_sem_t sem, uint32_t periodMs)
    {
        if (sem == RT_NULL) return RT_NULL;
        return rt_timer_create(name, +[](void *context) {
            const rt_sem_t signal = static_cast<rt_sem_t>(context);
            const InterruptLock lock;
            if (signal->value == 0u) rt_sem_release(signal);
        }, sem, rt_tick_from_millisecond(periodMs), RT_TIMER_FLAG_HARD_TIMER | RT_TIMER_FLAG_PERIODIC);
    }

    static void startThread(rt_thread_t &thread)
    {
        if (thread != RT_NULL && rt_thread_startup(thread) != RT_EOK) {
            rt_thread_delete(thread);
            thread = RT_NULL;
        }
    }

    void startThreads(ExceptionRecorder &bringUpRec)
    {
        ctrlSem   = rt_sem_create("ctrl", 0, RT_IPC_FLAG_PRIO);
        ctrlTimer = createPeriodicTimer("ctrlt", ctrlSem, app::ControlPeriodMs);
        if (ctrlTimer == RT_NULL) bringUpRec.throws(ExceptionCode::SCHED_THREAD_START_FAILED);

        rt_thread_t linkTid = RT_NULL; ///< 编译期关掉链路时保持 RT_NULL(下面的判据与启动都要看它)
#if CFG_FEATURE_LINK
        linkSem   = rt_sem_create("link", 0, RT_IPC_FLAG_PRIO);
        // 6ms 定时器只做 watchdog/统计; 正常路径的 D0 由 IDLE 中断唤醒(规格 §1.2 "IDLE 是硬要求")
        linkTimer = createPeriodicTimer("linkt", linkSem, LinkTickMs);
        linkCtx.sh    = &sh;
        linkCtx.radio = &link;
        linkCtx.rx    = &linkRx;
        linkCtx.sem   = linkSem;
        // ---- 轮次从机参数: 全部来自冻结的 app/link/link_v2_config.hpp(与遥控器逐名逐值一致) ----
        {
            link::RoundParams rp{};
            rp.roundMs           = LinkRoundMs;
            rp.frameSpacingMs    = LinkFrameSpacingMs;
            rp.guardUs           = LinkGuardUs;
            rp.u3JitterUs        = LinkU3JitterUs;
            rp.dlTimeoutMs       = LinkDlTimeoutMs;
            rp.acquireMs         = LinkAcquireMs;
            rp.airtimeBaseUs     = LinkAirtimeBaseUs;
            rp.airtimePerByteUs  = LinkAirtimePerByteUs;
            rp.uartByteUs        = LinkUartByteUs;
            rp.dlBudgetSlackUs   = LinkDlBudgetSlackUs;
            rp.roundReopenUs     = kRoundReopenUs; // 规格 §4 "本轮关闭": 60ms
            // 收帧末修正: E220 整帧收完才开始吐 UART ⇒ IDLE = 空中结束 + (n + LinkRxIdleBytes)×174us,
            // P0 实测 LinkRxIdleBytes=1。必须按帧长算, 否则 t0 会偏大 n×174us。
            rp.rxIdleBytes       = LinkRxIdleBytes;
            rp.protoVersion      = static_cast<uint8_t>(LinkProtoVersion);
            rp.caps              = link::hello::kCapsCmdId; // FC 能力: 支持 cmdId/ACK
            rp.helloMax          = static_cast<uint8_t>(LinkHelloMax);
            rp.auxGate           = true; // §2.3: 发前读 AUX, 忙则放弃本轮
            if (!linkCtx.round.init(rp)) {
                bootLogLine("[Boot] 链路轮次参数非法: RoundSlave 保持未初始化, 收指令不受影响, "
                            "但不会发下行\r\n");
            }

            // ---- 下行选帧(规格 §6): 优先级 + 到期; GPS 由 GNSS 特性与"是否真有数据"共同决定 ----
            link::DownlinkParamsV2 dp{};
            dp.stateEvery = kDlStateEvery; // 4 轮
            dp.sysEvery   = kDlSysEvery;   // 4 轮
            dp.sysOffset  = kDlSysOffset;  // 偏移 2, 与 STATE 错开
            dp.gpsEvery   = kDlGpsEvery;   // 16 轮
            // 本轮 GNSS 观测还没发布到 FlightTelemetry(要改 app_sensors 一侧): false ⇒ 永不选 GPS,
            // 不白占 1/16 下行槽; 接好后把这里打开即可(选择器/DL_GPS 编码已就绪)。
            dp.gpsEnabled = cfg::feature::kGnss;
            if (!linkCtx.dl.init(dp)) {
                bootLogLine("[Boot] 下行选帧参数非法: 不发下行(收指令不受影响)\r\n");
            }
        }
        if (linkTimer != RT_NULL) linkTid = rt_thread_create("link_tel", linkTelemetryThreadEntry, &linkCtx, 3072, 24, 20);
        startThread(linkTid);
#endif

        logSem        = rt_sem_create("log", 0, RT_IPC_FLAG_PRIO);
        logTimer      = createPeriodicTimer("logt", logSem, cfg::time::log::kTickMs);
        logCtx.sh     = &sh;
        logCtx.fs     = (app.mode == BootMode::Flight) ? app.fs : nullptr;
        logCtx.serialDma = app.serialDma;
        logCtx.txBuf     = app.txBuf;
        logCtx.protocol  = app.protocol;
        logCtx.rx = app.uartRx;
        logCtx.sem       = logSem;
        logCtx.debugStream = (app.mode != BootMode::Flight);
        rt_thread_t logTid = RT_NULL;
        if (logTimer != RT_NULL) logTid = rt_thread_create("logger", logThreadEntry, &logCtx, 3072, 26, 20);
        startThread(logTid);
        sh.tidLink = linkTid;
        sh.tidLog  = logTid;
        // 采集线程优先级低于控制线程、高于链路和日志；原始样本由控制线程融合。
        sensors.startThreads(app::SensorThreadPriority, bringUpRec);
        // 线程启动失败记入启动阻断条件，不允许解锁。
#if CFG_FEATURE_LINK
        if (linkTid == RT_NULL || logTid == RT_NULL) {
#else
        // 链路是编译期关掉的: 它不该被算成"线程缺失"。但要说清楚后果 ——
        // **没有链路 = 收不到任何指令 = 解不了锁**, 这份固件只能当地面观察用。
        if (logTid == RT_NULL) {
#endif
            bringUpRec.throws(ExceptionCode::SCHED_THREAD_START_FAILED);
            bootLogLine("[Boot] 线程创建失败(link=%s logger=%s): 检查 board.c 的 RT_HEAP_SIZE!\r\n",
                    (linkTid == RT_NULL) ? "FAIL" : "OK", (logTid == RT_NULL) ? "FAIL" : "OK");
        }
        bootLogLine("[Dev ] 软定时器: 控制=%ums 链路=%ums(仅 watchdog) 日志=%ums (硬定时器, 到期只释放信号量)\r\n",
                static_cast<unsigned>(app::ControlPeriodMs), static_cast<unsigned>(LinkTickMs),
                static_cast<unsigned>(cfg::time::log::kTickMs));
        logThreadReport(linkTid, logTid); // 线程栈 + 建线程后的堆水位(创建失败的原因八成在这里)
        bootLogLine("[Dev ] 采集线程: 该建 %u 个, 起来 %u 个\r\n",
                static_cast<unsigned>(Sensors::expectedThreads()),
                static_cast<unsigned>(sensors.startedThreads()));
    }

    bool threadsReady() const
    {
        return ctrlTimer != RT_NULL && logTimer != RT_NULL && sh.tidLog != RT_NULL &&
            sensors.startedThreads() == Sensors::expectedThreads() &&
            (!cfg::feature::kLink || (linkTimer != RT_NULL && sh.tidLink != RT_NULL));
    }

    // 生命周期 5: EKF 参数与气压基准
    void initEkf()
    {
        // EKF 整定参数集中读取 cfg::ekf。
        FlightControlFilterParams ekfParams{}; // 基表用结构体自带默认值
        ekfParams.estimate_accel_bias           = cfg::ekf::kEstimateAccelBias;
        ekfParams.accel_tilt_gate_mss           = cfg::ekf::kAccelTiltGateMss;
        ekfParams.accel_tilt_sigma              = cfg::ekf::kAccelTiltSigma;
        ekfParams.accel_tilt_inno_limit_rad     = cfg::ekf::kAccelTiltInnoLimitRad;
        ekfParams.freeze_gyro_bias_when_dynamic = cfg::ekf::kFreezeGyroBiasWhenDynamic;
        ekfParams.accel_hold_enable             = cfg::ekf::kAccelHoldEnable;
        ekfParams.accel_hold_gate_mss           = cfg::ekf::kAccelHoldGateMss;
        ekfParams.accel_hold_rate_dps           = cfg::ekf::kAccelHoldRateDps;
        ekfParams.accel_hold_time_s             = cfg::ekf::kAccelHoldTimeS;
        ekfParams.baro_sigma_m                  = cfg::ekf::kBaroSigmaM;
        // 过程噪声、门限及偏置限制均显式接入 EKF 参数。
        ekfParams.gyro_bias_limit     = cfg::ekf::kGyroBiasLimitRads;
        ekfParams.rangefinder_sigma_m = cfg::ekf::kRangeSigmaM;
        ekfParams.range_gate_sigma    = cfg::ekf::kRangeGateSigma;
        ekfParams.flow_vel_sigma_mps  = cfg::ekf::kFlowVelSigmaMps;
        ekfParams.flow_inno_limit_mps = cfg::ekf::kFlowInnoLimitMps;
        ekfParams.flow_gate_sigma     = cfg::ekf::kFlowGateSigma;
        ekfParams.vel_xy_var_max      = cfg::ekf::kVelXyVarMax;
        ekfParams.pos_xy_var_max      = cfg::ekf::kPosXyVarMax;
        // 重力常数也要显式接: 结构体里那份 9.80665 只是兜底, 不接的话"改配置里的 g"不会生效。
        ekfParams.gravity_mss         = cfg::control::kGravityMss;
        ekf.set(ekfParams);
        ekf.reset();
    }

    // 控制周期各阶段

    /** 读取输入及健康快照，优先处理急停，再有界取出普通请求。 */
    void stepSampleShared()
    {
        loopStartCyc = DWT->CYCCNT;
        input = sh.control.input.load();
        cycleLogStatus = sh.logStatus.load();
        cycleHealth = sensors.health.snapshot();
        cycleHealth.sources[static_cast<uint8_t>(SensorHealth::LINK)] = linkCtx.health.snapshot().sources[static_cast<uint8_t>(SensorHealth::LINK)];
        now_ms = rt_tick_get_millisecond();
        input.velocityValid = input.velocityValid && sampleFresh(now_ms, input.velocityReceivedMs, app::ControlFreshnessMs);
        input.stickValid = input.stickValid && sampleFresh(now_ms, input.stickReceivedMs, app::ControlFreshnessMs);
        cmd = input.command;
        manThr = input.stickValid ? input.manualThrottle : 0.0f;
        sp = input.stickValid ? input.setpoint : FlightControlSetpoint{};
        lastValid = input.receivedMs;
        stepSafety();
        taskCount = 0u;
        const uint8_t room = static_cast<uint8_t>(app::RequestsPerCycle - pendingResultCount);
        const uint16_t ready = sh.control.requests.size();
        const uint8_t limit = static_cast<uint8_t>(ready < room ? ready : room);
        while (taskCount < limit && sh.control.requests.tryPop(taskBatch[taskCount])) ++taskCount;
    }

    /** 有界消费 IMU 样本，执行名义积分及降频协方差更新。 */
    void stepImu()
    {
        const uint32_t startCycles = DWT->CYCCNT;
        imuGapThisCycle = false;
        ImuSample sample;
        for (uint8_t count = 0u; count < app::ImuSamplesPerCycle && sensors.imuSamples.read(sample); ++count) {
            if (!sampleFresh(now_ms, sample.tickMs, app::ImuFreshnessMs)) {
                imuGapThisCycle = true;
                continue;
            }
            if (lastImuSequence != 0u && static_cast<uint32_t>(sample.sequence - lastImuSequence) != 1u) imuGapThisCycle = true;
            lastImuSequence = sample.sequence;
            lastAcc = sample.acceleration;
            lastGyro = sample.angularVelocity;
            ekf.integrateNominal(lastGyro, lastAcc, cfg::kImuDt);
            imuDtAccum += cfg::kImuDt;
            if (++imuCnt >= cfg::ekf::kDecim) {
                const uint32_t covarianceStart = DWT->CYCCNT;
                ekf.propagateCovariance(lastGyro, lastAcc, imuDtAccum);
                ekf.clampHorizontalCovariance();
                ekf.updateAccelerometer(lastAcc);
                perfAdd(sh.perf.ekfSum, sh.perf.ekfCnt, DWT->CYCCNT - covarianceStart);
                imuCnt = 0u;
                imuDtAccum = 0.0f;
            }
        }
        perfAdd(sh.perf.ioSum, sh.perf.ioCnt, DWT->CYCCNT - startCycles);
    }

    /** 集中设置光流估计和速度环参数。 */
    void initObservers()
    {
        OpticalFlowEstimatorParams &fp = flowEst.params();
        fp.rad_per_count               = cfg::sensor::flow::kRadPerCount;
        fp.lp_tau                      = cfg::sensor::flow::kLpfTauS;
        fp.max_rate                    = cfg::sensor::flow::kMaxRateRads;
        fp.max_vel                     = cfg::sensor::flow::kMaxVelMps;
        fp.min_h                       = cfg::sensor::flow::kMinHeightM;
        fp.max_h                       = cfg::sensor::flow::kMaxHeightM;
        fp.timeout                     = cfg::sensor::flow::kTimeoutS;
        fp.max_delta                   = cfg::sensor::flow::kMaxDelta;
        fp.sat_delta                   = cfg::sensor::flow::kSatDelta;
        fp.sign_x                      = static_cast<int8_t>(cfg::sensor::flow::kSignX);
        fp.sign_y                      = static_cast<int8_t>(cfg::sensor::flow::kSignY);
        fp.yaw_deg                     = cfg::sensor::flow::kYawDeg;
        fp.min_squal                   = static_cast<uint8_t>(cfg::sensor::flow::kMinSqual);
        fp.fail_limit                  = static_cast<uint8_t>(cfg::sensor::flow::kFailLimit);
        flowEst.reset();

        VelocityControllerParams &vp = velCtrl.params();
        vp.kp                        = cfg::control::kVelXyP;
        vp.ki                        = cfg::control::kVelXyI;
        vp.kd                        = cfg::control::kVelXyD;
        vp.int_limit                 = cfg::control::kVelXyIntLimit;
        vp.vel_max_mps               = cfg::control::kVelXyMaxMps;
        vp.acc_max_mss               = cfg::control::kVelAccMaxMss;
        vp.tilt_max_rad              = cfg::control::kVelTiltMaxRad;
        vp.flow_timeout_s            = cfg::control::kVelFlowTimeoutS;
        vp.hover_throttle            = cfg::control::kHoverThrottle;
        vp.throttle_min              = cfg::control::kThrottleMin;
        vp.throttle_max              = cfg::control::kThrottleMax;
        vp.tilt_comp_max_rad         = cfg::control::kTiltMaxRad;
        velCtrl.reset();
    }

    /** 观测层只消费样本通道，不访问总线；最新值按发布序号去重，增量流有界读取。 */
    void stepObservers()
    {
        baroUpdated   = false;
        flowValid = flowObserved && healthOk(SensorHealth::FLOW) && sampleFresh(now_ms, lastFlowAcceptedMs, app::FlowFreshnessMs);
        flowFlags     = 0;

        stepBaroObs();
        stepRangeObs();
        stepFlowObs();
        stepGnssObs();
    }

    /** 气压高度观测(可选件) */
    void stepBaroObs()
    {
#if CFG_FEATURE_BARO
        BaroSample bs;
        if (!sensors.baroSnap.readNew(bs, lastBaroStamp)) return;
        if (debugSamplesEnabled) { // 先记原始样本, 再走门限 —— 被拒的样本也要能在曲线上看到
            debugSampling.baro = bs;
            debugSampling.flags |= kDebugBaroNew;
        }
        // 快照里没有"有效位": 采集层**只在读成功时**才发布(见 app_sensors.hpp 的说明),
        // 所以"拿到快照"就等于"这份读数可用"。
        if (!sampleFresh(now_ms, bs.tickMs, app::BaroFreshnessMs)) return;
        if (bs.referenceSequence == 0u) return;
        if (bs.referenceSequence != lastBaroReferenceSequence) {
            lastBaroReferenceSequence = bs.referenceSequence;
            ekf.reset(ekf.getQuaternion(), Vector3f{}, Vector3f{});
            imuCnt = 0u;
            imuDtAccum = 0.0f;
            heightObserved = false;
            flowObserved = false;
            flowValid = false;
            flowEst.reset();
            eventFlags |= kLogEventEkfRezero;
            if (debugSamplesEnabled) {
                debugSampling.flags |= kDebugBaroRefReset;
            }
        }
        if (!ekf.updateBarometer(bs.relativeAltitudeM)) return;
        if (debugSamplesEnabled) {
            debugSampling.flags |= kDebugBaroOk;
        }
        heightObserved = true;
        lastHeightAcceptedMs = bs.tickMs;
        baroUpdated = true;
#endif
    }

    /** 测距高度观测(VL53L1X): 只在高度可解释的范围内喂 EKF */
    void stepRangeObs()
    {
        RangeSample rs;
        if (!sensors.rangeSnap.readNew(rs, lastRangeStamp)) return;
        if (debugSamplesEnabled) { // 与气压同理: 被拒的测距也要留下原始读数
            debugSampling.range = rs;
            debugSampling.flags |= kDebugRangeNew;
        }
        if (!sampleFresh(now_ms, rs.tickMs, app::RangeFreshnessMs)) return;
        // σ 与门限都在 EKF 参数里(rangefinder_sigma_m / range_gate_sigma, 见 initEkf)——
        // 不在这里传, 免得同一条策略有两个真相源: "把门限放宽"只应该改一个地方。
        if (ekf.updateRangeHeight(rs.rangeM)) {
            if (debugSamplesEnabled) {
                debugSampling.flags |= kDebugRangeOk;
            }
            heightObserved = true;
            lastHeightAcceptedMs = rs.tickMs;
            rangeM = rs.rangeM;
            flowFlags |= kFlowFlagRangeOk;
        }
    }

    /** 结合控制线程当前高度与角速度，将光流增量换算为世界系速度并送入 EKF。 */
    void stepFlowObs()
    {
        for (uint8_t count = 0u; count < app::FlowSamplesPerCycle && sensors.flowSnap.read(lastFlow); ++count) {
            if (debugSamplesEnabled) { // 记原始增量(未乘尺度), 用于区分"没数据/被拒/尺度错"
                debugSampling.flow = lastFlow;
                debugSampling.flags |= kDebugFlowNew;
            }
            if (!sampleFresh(now_ms, lastFlow.tickMs, app::FlowFreshnessMs)) continue;
            const auto &estimate = flowEst.feed(static_cast<float>(lastFlow.dx), static_cast<float>(lastFlow.dy),
                lastFlow.squal, static_cast<float>(lastFlow.dtMs) * 0.001f, ekf.getHeight(), lastFlow.tickMs,
                lastGyro.data[0], lastGyro.data[1]);
            flowValid = false;
            flowObserved = false;
            if (!estimate.valid) continue;
            const Vector3f worldVelocity = quatRotate(ekf.getQuaternion(), estimate.velocity_body_mps);
            if (ekf.updateHorizontalVelocity(worldVelocity.x(), worldVelocity.y())) {
                if (debugSamplesEnabled) {
                    debugSampling.flags |= kDebugFlowOk;
                }
                flowValid = true;
                flowObserved = true;
                lastFlowAcceptedMs = lastFlow.tickMs;
                flowFlags |= kFlowFlagVelHold;
            }
        }
    }

    void stepGnssObs()
    {
        GnssSample sample;
        if (!cfg::sensor::gnss::kUseInEkf || !sensors.gnssSnap.read(sample) || !sample.velocityValid ||
            sample.velocitySequence == lastGnssVelocitySequence ||
            !sampleFresh(now_ms, sample.velocityMs, app::GnssFreshnessMs)) return;
        lastGnssVelocitySequence = sample.velocitySequence;
        if (ekf.updateHorizontalVelocity(sample.velocityEastMps, sample.velocityNorthMps)) {
            flowFlags |= kFlowFlagGnssVel;
        }
    }

    /** 5.4 组装控制器输入 */
    void stepAssembleInputs()
    {
        fcs.attitude                = ekf.getQuaternion();
        fcs.body_rate               = lastGyro;
        fcs.height_m                = ekf.getHeight();
        fcs.vertical_velocity_mps   = ekf.getVerticalVelocity();
        fcs.vertical_velocity_valid = heightObserved && (healthOk(SensorHealth::BARO) || healthOk(SensorHealth::RANGE)) && sampleFresh(now_ms, lastHeightAcceptedMs, app::BaroFreshnessMs); // 有高度观测时垂速才可信(定高用)
    }

    /** 处理已准入的模式及操作请求，不直接切换生命周期状态。 */
    void applyInPlace(app::Task on)
    {
        if (app::isModeTask(on)) {
            mode = app::modeOf(on);
            if (mode == app::Mode::Manual) {
                applyEffects(app::effectPurgeDirect); // 进测试模式那一拍清直控缓存
            }
            // 复用"指令变了"这一位: 模式也是指令的一种, 而新加一个日志位要动 LogEntry 布局
            // (布局版本 +1 + 上位机同步), 不值得为一个可选位付出那个代价。
            eventFlags |= kLogEventCommandChanged;
            return;
        }
        operation = (on == app::Task::RequestTakeoff) ? app::Operation::Takeoff
                    : (on == app::Task::RequestLand)  ? app::Operation::Land
                                                      : app::Operation::None;
        // 换操作 ⇒ 阶段复位。于是"operation == None ⇒ stage == 0"成为一条不变量 ——
        // 否则交付时留下的 stage 会一直挂在快照上, 日后有人读它(日志/UI)会以为操作还在跑。
        opStage = 0u;
        eventFlags |= kLogEventCommandChanged;
    }

    /** 在控制线程内执行故障动作及健康降级。 */
    void stepFaults()
    {
        const uint32_t actions = sh.faults.takeActions();
        ExceptionCode code;
        for (uint8_t count = 0u; count < app::FaultsPerCycle && sh.faults.take(code); ++count) {}
        if ((actions & app::StopOutput) != 0u) {
            faultStopLatched = true;
            stopOutput(false);
        }
        const bool imuUsable = healthOk(SensorHealth::IMU) && !imuGapThisCycle;
        if (app::canOutput(state) && mode != app::Mode::Manual && !imuUsable) {
            stopOutput(false);
            DLX_HANDLE_EXCEPTION(ExceptionCode::IMU_DATA_STALE);
        }
        if (!app::canOutput(state)) return;
        if ((mode == app::Mode::AltHold || mode == app::Mode::VelHold || operation != app::Operation::None) && !fcs.vertical_velocity_valid) {
            taskHandoverToStab = operation != app::Operation::None;
            mode = app::Mode::Stabilize;
            operation = app::Operation::None;
            opStage = 0u;
            hoverLandActive = false;
            DLX_HANDLE_EXCEPTION(ExceptionCode::STATE_MODE_DEGRADED);
        } else if (mode == app::Mode::VelHold && !flowValid) {
            mode = app::Mode::AltHold;
            DLX_HANDLE_EXCEPTION(ExceptionCode::STATE_MODE_DEGRADED);
        }
        if (!inputFresh() || hoverLandActive) {
            if (app::isTakeover(state) && mode != app::Mode::Manual && fcs.vertical_velocity_valid) {
                if (!hoverLandActive) {
                    hoverLandActive = true;
                    operation = app::Operation::Land;
                    opStage = 0u;
                    eventFlags |= kLogEventFailsafe;
                }
            } else {
                stopOutput(false);
            }
        }
    }

    app::GroundStatus admitRequest(const app::Request &request) const
    {
        if (request.task == app::Task::RequestDisarm) return app::GroundStatus::Ok;
        if (emergencyThisCycle || input.emergencyStopPending || faultStopLatched ||
            request.safetyEpoch != sh.control.safetyEpoch.load()) return app::GroundStatus::RejectedSafety;
        if (!sampleFresh(now_ms, request.issuedMs, app::RequestFreshnessMs)) return app::GroundStatus::Timeout;
        if (request.task == app::Task::RequestArm) {
            if (!app::isGround(state)) return app::GroundStatus::RejectedState;
            if (!bootReadyNow) return app::GroundStatus::RejectedArming;
            if (!inputFresh()) return app::GroundStatus::RejectedSafety;
            if (!healthOk(SensorHealth::POWER)) return app::GroundStatus::RejectedHealth;
            if (mode != app::Mode::Manual && (!input.stickValid || input.throttleCounts > static_cast<uint16_t>(app::ArmThrottleLowFraction * 2047.0f) ||
                abs(input.rollCounts) > app::ArmStickCenterFraction * 1023.0f ||
                abs(input.pitchCounts) > cfg::link::arm::kStickCenterPct * 1023.0f)) return app::GroundStatus::RejectedSafety;
            if (mode == app::Mode::Manual && (input.motorValue[0] != 0u || input.motorValue[1] != 0u || input.motorValue[2] != 0u || input.motorValue[3] != 0u)) return app::GroundStatus::RejectedSafety;
            if (mode != app::Mode::Manual && (!healthOk(SensorHealth::IMU) || imuGapThisCycle)) return app::GroundStatus::RejectedHealth;
            if ((mode == app::Mode::AltHold || mode == app::Mode::VelHold) && !fcs.vertical_velocity_valid) return app::GroundStatus::RejectedHealth;
            if (mode == app::Mode::VelHold && !flowValid) return app::GroundStatus::RejectedHealth;
            return app::GroundStatus::Ok;
        }
        if (app::isModeTask(request.task)) {
            if (!app::isGround(state)) return app::GroundStatus::RejectedState;
            if (request.task == app::Task::RequestModeManual && state != app::SystemState::Maintenance) return app::GroundStatus::RejectedSafety;
            return app::GroundStatus::Ok;
        }
        if (app::isOperationTask(request.task)) {
            if (!app::isTakeover(state)) return app::GroundStatus::RejectedState;
            if (hoverLandActive) return app::GroundStatus::RejectedSafety;
            if (request.task != app::Task::RequestCancel && (!fcs.vertical_velocity_valid || mode == app::Mode::Manual)) return app::GroundStatus::RejectedHealth;
            return app::GroundStatus::Ok;
        }
        return app::GroundStatus::BadCommand;
    }

    void stepStateMachine()
    {
        bootReadyNow = startupReady;
        for (uint8_t index = 0u; index < taskCount; ++index) {
            const app::Request &request = taskBatch[index];
            const app::GroundStatus result = admitRequest(request);
            if (result == app::GroundStatus::Ok) {
                if (request.task == app::Task::RequestDisarm) {
                    stopOutput(false);
                } else if (request.task == app::Task::RequestArm) {
                    armOrigin = state;
                    hardStop = false;
                    directInputFloor = input.motorSequence;
                    applyEffects(app::effectStartPower);
                    state = app::SystemState::Arming;
                    stateEnteredMs = now_ms;
                } else {
                    applyInPlace(request.task);
                }
            } else if (request.task == app::Task::RequestArm) {
                eventFlags |= kLogEventArmRejected;
            }
            if (request.source == app::RequestSource::Link) {
                app::RequestResult &feedback = pendingResults[pendingResultCount++];
                feedback.kind = app::ResultKind::Request;
                feedback.id = request.id;
                feedback.status = result;
                feedback.wireSequence = request.wireSequence;
                feedback.wireOperation = request.wireOperation;
            }
        }
        if (state == app::SystemState::Arming && static_cast<uint32_t>(now_ms - stateEnteredMs) >= app::MotorArmDelayMs) {
            if (sensors.up.baro && sensors.baroReference.pending()) {
                stopOutput(false);
                DLX_HANDLE_EXCEPTION(ExceptionCode::BARO_DATA_STALE);
            } else {
                applyEffects(app::effectTakeover);
                state = app::SystemState::Armed;
                stateEnteredMs = now_ms;
            }
        }
        stepAltitudeTarget();
        stepPilotThrottle();
        modePrev = mode;
        flushResults();
    }

    /** 生成各模式、自动起降和失效保护共用的高度目标。 */
    void stepAltitudeTarget()
    {
        const bool entered = (mode != modePrev);

        // 解锁边沿: 高度基准归零(控制接管时 EKF 高度也被归零, 两者必须同源)
        if (state == app::SystemState::Arming && entered) {
            altTargetM     = 0.0f;
            altTargetValid = false;
        }

        if (mode == app::Mode::AltHold) {
            // 定高: 目标 = 遥控器积分出来的**绝对值**(增量式摇杆在遥控器里积成绝对量再发)
            altTargetM     = input.altitudeValid ? input.altitudeTargetM : fcs.height_m;
            altTargetValid = true;
        } else if (mode == app::Mode::VelHold) {
            // 定速: 高度**保持进入模式那一刻的值**, Z 轴交给 vz 前馈 —— 位置环不去追速度指令
            if (entered || !altTargetValid) {
                altTargetM     = fcs.height_m;
                altTargetValid = true;
            }
        }
        // 自稳/测试: 高度环不参与, 目标不更新(切回定高时由上面的分支接管)

        /** 操作及失效保护的高度目标优先于模式目标。 */
        {
            app::OperationIn pin;
            pin.operation  = operation;
            pin.stage      = opStage;
            pin.altTargetM = altTargetM;

            app::OperationCtx pctx;
            pctx.dt        = cfg::kLoopDt;
            pctx.heightM   = fcs.height_m;
            pctx.vzMps     = fcs.vertical_velocity_mps;
            pctx.canOutput = app::canOutput(state);
            pctx.hoverLand = hoverLandActive;

            const app::OperationOut pout = app::stepOperation(pin, pctx);
            const app::AltitudeUse  altPrev = taskAlt;
            operation    = pout.operation;
            opStage      = pout.stage;
            altTargetM   = pout.altTargetM;
            taskAlt      = pout.altitude;

            /** 操作退出高度控制时平滑交还油门，并约束保持目标，避免高度目标突跳。 */
            if (altPrev == app::AltitudeUse::Hold && taskAlt == app::AltitudeUse::None &&
                pin.operation == app::Operation::Takeoff && operation == app::Operation::None) {
                if (mode == app::Mode::Stabilize) {
                    taskHandoverToStab = true;
                } else if (mode == app::Mode::VelHold) {
                    altTargetM     = cfg::control::kTakeoffHeightM; // 交付高度 = 起飞高度, 定速保持它
                    altTargetValid = true;
                }
            }
        }

        altTargetM = constrain(altTargetM, cfg::control::kAltMinM, cfg::control::kAltMaxM);
    }

    /** 飞行员油门的"降级兜底"交接(见 manThrEff 的说明) */
    void stepPilotThrottle()
    {
        const bool fellToStab = (mode == app::Mode::Stabilize) &&
                                (modePrev == app::Mode::AltHold || modePrev == app::Mode::VelHold);
        if (fellToStab) {
            manThrHold = true;
            manThrEff  = cfg::control::kHoverThrottle; // 先钉在悬停油门, 绝不交给 0
            DLX_HANDLE_EXCEPTION(ExceptionCode::STATE_MODE_DEGRADED);
        }
        /** 自动起飞交回自稳时复用悬停油门保持，避免接回被忽略的零油门。 */
        if (taskHandoverToStab) {
            manThrHold         = true;
            taskHandoverToStab = false;
            DLX_HANDLE_EXCEPTION(ExceptionCode::STATE_TASK_HANDOVER);
        }
        // 飞行员一旦给了非零油门(遥控器看到模式变化后会开始发真值), 兜底就结束
        if (manThr > 0.05f) {
            manThrHold = false;
        }
        manThrEff = manThrHold ? cfg::control::kHoverThrottle : manThr;
    }

    /** 5.6 按模式分派: 水平速度环 / 角度环 / 高度环 / 混控 + DShot */
    void stepControl()
    {
        const bool prevSat = pidSat; // 上一圈的混控饱和: 速度环的积分抗饱和要用它
        out                = {};
        pidSat             = false;
        const uint32_t ctrlStartCyc = DWT->CYCCNT;
        if (app::isTakeover(state) && !faultStopLatched) {
            FlightControlSetpoint spEff = sp;
            // 自稳/降级兜底用的集合油门(降级交接见 manThrEff 的说明)
            float throttle =
                (manThrEff > cfg::control::kAngleLoopMinThrottle) ? manThrEff : cfg::control::kAngleLoopMinThrottle;

            /** Manual 直控；Stabilize 使用角度环；AltHold/VelHold 使用高度环，VelHold 额外使用水平速度环。操作可优先接管高度目标。 */
            if (mode == app::Mode::Manual) {
                /** Manual 跳过控制器和混控，但仍受 Armed、新鲜电机帧及最终输出门控限制。 */
                if (state == app::SystemState::Armed) {
                    uint16_t counts[4];
                    rt_enter_critical();
                    for (uint8_t i = 0u; i < link::upMotor::kMotorCount; ++i) {
                        counts[i] = inputFresh() && input.motorSequence != directInputFloor && sampleFresh(now_ms, input.motorReceivedMs, app::ControlFreshnessMs) ? input.motorValue[i] : 0u;
                    }
                    rt_exit_critical();
                    /** 直控与混控共用逻辑电机到物理 DShot 通道的映射。 */
                    for (uint8_t i = 0u; i < link::upMotor::kMotorCount; ++i) {
                        motor[cfg::hw::motor::kMap[i]] = counts[i];
                    }
                }
                // ⚠ 本分支**不进**下面的混控: 进了就等于"控制器算出的油门盖掉直控值"。
            } else {
                if (taskAlt == app::AltitudeUse::Idle) {
                    /** 触地后退出高度环，仅保留姿态环及最低油门，等待显式停机请求。 */
                    controller.updateAngle(spEff, fcs, cfg::kLoopDt, out);
                    out.throttle = cfg::control::kAngleLoopMinThrottle;
                } else if (mode == app::Mode::VelHold && cfg::control::kVelHoldEnable && flowValid &&
                           velCtrl.paramsSane()) {
                    const FlightFilterState16 st16 = ekf.getState16();
                    const float vxSp = input.velocityValid ? input.velocityTargetMps[0] : 0.0f;
                    const float vySp = input.velocityValid ? input.velocityTargetMps[1] : 0.0f;
                    spEff.attitude  = velCtrl.update(vxSp, vySp, st16.data[FF_VX], st16.data[FF_VY], cfg::kLoopDt,
                                                     ekf.getHeight(), flowValid, now_ms, prevSat);
                    spEff.height_m  = altTargetM;
                    /** 速度目标失效时垂速前馈归零；操作控制高度期间不叠加飞行员垂速。 */
                    spEff.vz_ff_mps = (taskAlt == app::AltitudeUse::Hold)
                                          ? 0.0f
                                          : (input.velocityValid ? input.velocityTargetMps[2] : 0.0f);
                    // 集合油门由**高度环**给(不再是 velHoldCollectiveThrottle): 定速模式的高度也要守住,
                    controller.updateAngleHeight(spEff, fcs, cfg::kLoopDt, out);
                } else if (mode == app::Mode::AltHold || mode == app::Mode::VelHold ||
                           taskAlt == app::AltitudeUse::Hold) {
                    /** 速度环不可用时仍保留高度环，不能退到读取零操纵油门的分支。 */
                    spEff.height_m  = altTargetM;
                    spEff.vz_ff_mps = 0.0f;
                    controller.updateAngleHeight(spEff, fcs, cfg::kLoopDt, out);
                } else {
                    // 自稳(以及任何没在上面列出的档) —— 角度环 + 飞行员油门。
                    // 落到这里是**更稳**的方向: 将来加新模式忘了接线, 表现是"像自稳一样能飞",
                    // 而不是"没有姿态控制"。
                    controller.updateAngle(spEff, fcs, cfg::kLoopDt, out);
                    out.throttle = throttle;
                }

                const auto ctrlMotor = mixMotors(out, controller.params());
                // 逻辑电机 -> 物理 DShot 通道(cfg::hw::motor::kMap)
                for (int i = 0; i < 4; ++i) {
                    const int phys = cfg::hw::motor::kMap[i];
                    motor[phys] = (uint16_t)(ctrlMotor.motor[i] * cfg::hw::motor::kUnit + cfg::hw::motor::kOffset);
                    if (motor[phys] <= 55u || motor[phys] >= 1945u) {
                        pidSat = true;
                    }
                }
            }
        }
        perfAdd(sh.perf.ctrlSum, sh.perf.ctrlCnt, DWT->CYCCNT - ctrlStartCyc);
        if (app::isTakeover(state) && !hardStop && !faultStopLatched) {
            dshot.preloadThrottle(motor);
        }
        if (pidSat) {
            eventFlags |= kLogEventMotorSat;
        }
    }

    /** 同一组控制事实分别编码为遥测位和日志位。 */
    struct LoopFacts
    {
        bool linkOk, imuOk, baroOk, angleLoop, motorEnabled, motorStarting;
        bool hardStop, failsafe, logFull, logError, flashOk, groundMode;
    };

    LoopFacts gatherFacts() const
    {
        LoopFacts f;
        f.linkOk        = inputFresh() && healthOk(SensorHealth::LINK);
        f.imuOk         = healthOk(SensorHealth::IMU);
        f.baroOk        = healthOk(SensorHealth::BARO);
        f.angleLoop     = (app::isTakeover(state) && mode != app::Mode::Manual); // 测试(直通)模式没有角度环
        f.motorEnabled  = app::isTakeover(state);              // 派生自状态: 没有第二份真相
        f.motorStarting = (state == app::SystemState::Arming);
        f.hardStop      = hardStop;
        f.failsafe      = hoverLandActive;
        f.logFull       = cycleLogStatus.full;
        f.logError      = cycleLogStatus.error || (app.mode == BootMode::Flight && app.fs == nullptr);
        f.flashOk       = ((app.mode == BootMode::Flight) && app.fs != nullptr && app.fs->isReady());
        f.groundMode    = (app.mode != BootMode::Flight);
        return f;
    }

    /** 整体发布控制状态和遥测快照。 */
    void stepPublish()
    {
        q = ekf.getQuaternion();
        const LoopFacts f = gatherFacts();

        // 回传位域: status / error 都是**实时**值(这一刻怎么样就报怎么样, 不锁存)
        uint8_t telStatus = 0;
        if (f.motorEnabled) telStatus |= kTelStatusMotorEnabled;
        if (f.motorStarting) telStatus |= kTelStatusMotorStarting;
        if (f.angleLoop) telStatus |= kTelStatusAngleLoop;
        if (f.linkOk) telStatus |= kTelStatusLinkOk;
        if (f.hardStop) telStatus |= kTelStatusHardStop;
        if (f.failsafe) telStatus |= kTelStatusFailsafe;
        if (f.logFull) telStatus |= kTelStatusLogFull;
        // 气压计可用 ⇒ height 字段装的是垂速; 否则装高度。判据就是健康位(唯一真相)。
        if (f.baroOk) telStatus |= kTelStatusBaroOk;
        uint8_t telError = 0;
        if (f.logError) telError |= kTelErrLogWrite;
        if (!f.imuOk) telError |= kTelErrImu;
        if (!f.baroOk && cfg::feature::kBaro) telError |= kTelErrBaro; // 主动禁用不算故障
        rt_enter_critical();
        for (int i = 0; i < 4; ++i) {
            published.telemetry.motor[i] = motor[i];
        }
        published.telemetry.quat             = q;
        published.telemetry.height_m         = fcs.height_m;
        published.telemetry.vertical_vel_mps = fcs.vertical_velocity_mps;
        published.telemetry.status           = telStatus;
        published.telemetry.error            = telError; // 实时: 问题解决就回 0(粘滞交给显示端做)
        published.telemetry.enabled          = app::isTakeover(state);
        // 发布世界系速度、位置和电源遥测。
        {
            const FlightFilterState16 st16 = ekf.getState16();
            published.telemetry.velWorldMps        = Vector3f{st16.data[FF_VX], st16.data[FF_VY], fcs.vertical_velocity_mps};
            // 水平位置本轮没有观测源(光流给的是速度不是位置): x/y 保持 0, 不编假数据
            published.telemetry.posWorldM = Vector3f{0.0f, 0.0f, fcs.height_m};
        }
        // 电池: PowerSample 一直在采, 但此前**没有任何地方读它**(见 docs 2026-10-02_upgrade_progress
        // 第 21 项"低压分级保护没做")。这里至少让它在遥测里可见 —— 低压保护本身是另一件事。
        {
            PowerSample ps;
            if (sensors.powerSnap.read(ps)) {
                published.telemetry.batteryV = ps.packV;
                published.telemetry.batteryA = ps.amps;
            }
        }
        // ---- 系统状态快照: 本栈持有, 链路线程经上下文只读, 与遥测在同一发布窗口更新 ----
        app::Snapshot snap;
        snap.state     = state;
        snap.mode      = mode;
        snap.operation = operation;
        snap.health    = cycleHealth.bits(now_ms);
        snap.hardStop = hardStop;
        snap.bootReady = bootReadyNow;
        snap.publishedMs = now_ms;
        snap.sequence = ++publishSequence;
        published.snapshot = snap;
        sh.published.store(published);
        rt_exit_critical();

        // ---- 5.8 事件位 ----
        if (cmd != lastCmd) {
            eventFlags |= kLogEventCommandChanged;
            lastCmd = cmd;
        }
        if (f.logError || f.logFull) {
            eventFlags |= kLogEventLogFault;
        }
        if (baroUpdated) {
            eventFlags |= kLogEventBaroUpdate;
        }
    }

    /**
     * @brief 生成一条"原始观测 + 同刻融合结果"的调试采样(仅串口调试模式)。
     *
     * 调用点在 stepLog 里, 因此节奏与飞行日志帧完全一致(调试模式 100Hz)。各通道用"自上次
     * 记录以来有没有新样本/有没有进 EKF"的位标记; 没有新样本的周期里数值沿用上一次 —— 曲线的
     * 阶梯就是传感器的真实更新率, 与同一横轴上的 EKF 曲线可以直接对比。
     */
    void pushDebugRecord()
    {
        if (!debugSamplesEnabled) return;

        DebugSample sample;
        sample.tickMs               = now_ms;
        sample.flags                = debugSampling.flags;
        sample.pressurePa           = debugSampling.baro.pressurePa;
        sample.altitudeM            = debugSampling.baro.altitudeM;
        sample.relativeAltitudeM    = debugSampling.baro.relativeAltitudeM;
        sample.flowDx               = debugSampling.flow.dx;
        sample.flowDy               = debugSampling.flow.dy;
        sample.flowDtMs             = debugSampling.flow.dtMs;
        sample.flowSqual            = debugSampling.flow.squal;
        sample.rangeRawMm           = debugSampling.range.rawMm;
        sample.rangeStatus          = debugSampling.range.status;
        sample.rangeM               = debugSampling.range.rangeM;

        // 本周期结束时的 EKF 估计: fcs 在 stepAssembleInputs 里由 EKF 直接填充, 和本记录同一圈。
        const FlightFilterState16 state16 = ekf.getState16();
        sample.ekfHeightM            = fcs.height_m;
        sample.ekfVertVelMps         = fcs.vertical_velocity_mps;
        sample.ekfVelXMps            = state16.data[FF_VX];
        sample.ekfVelYMps            = state16.data[FF_VY];

        debugQueuePush(sh.debugQueue, sample);
        debugSampling.flags = 0u; // 只统计"到这一条记录为止"的窗口
    }

    /** 5.9 日志: 按周期打包入队(事件位随这一条日志归档后清零) */
    void stepLog()
    {
        if ((now_ms - lastLogMs) < logPeriodMs) {
            lastLoopCyc = loopStartCyc;
            return;
        }
        lastLogMs = now_ms;
        ++logCounter;
        LogEntry e  = {};
        e.tickMs    = now_ms;
        e.linkAgeMs = static_cast<uint16_t>(((now_ms - lastValid) > 0xFFFFu) ? 0xFFFFu : (now_ms - lastValid));
        e.loopPeriodUs = static_cast<uint16_t>(((loopStartCyc - lastLoopCyc) / (SystemCoreClock / 1000000u)) & 0xFFFFu);
        e.events       = eventFlags;

        uint16_t flags = 0;
        const LoopFacts f = gatherFacts(); // 与遥测位同一批事实(见 gatherFacts 的说明)
        if (f.linkOk) flags |= kLogFlagLinkOk;
        if (f.imuOk) flags |= kLogFlagImuOk;
        if (f.baroOk) flags |= kLogFlagBaroOk;
        if (f.angleLoop) flags |= kLogFlagAngleLoop;
        if (f.motorEnabled) flags |= kLogFlagMotorEnabled;
        if (f.motorStarting) flags |= kLogFlagMotorStarting;
        if (f.hardStop) flags |= kLogFlagHardStop;
        if (f.flashOk) flags |= kLogFlagFlashOk;
        if (f.logFull) flags |= kLogFlagLogFull;
        if (f.logError) flags |= kLogFlagLogError;
        if (pidSat) flags |= kLogFlagMotorSat;
        if (f.failsafe) flags |= kLogFlagFailsafe;
        if (f.groundMode) flags |= kLogFlagGroundMode;
        e.flags = flags;

        e.quat[0] = q.data[0];
        e.quat[1] = q.data[1];
        e.quat[2] = q.data[2];
        e.quat[3] = q.data[3];
        for (int i = 0; i < 3; ++i) {
            e.bodyRate[i]     = fcs.body_rate.data[i];
            e.accelG[i]       = lastAcc.data[i];
            e.rateSetpoint[i] = out.rate_setpoint.data[i];
            e.torque[i]       = out.torque.data[i];
        }
        const Vector3f spEuler = quatToEuler(sp.attitude);
        e.targetPitchDeg       = spEuler.y() * cfg::kRadToDeg;
        e.targetRollDeg        = spEuler.z() * cfg::kRadToDeg;
        e.targetHeightM        = sp.height_m;
        e.throttle             = out.throttle;
        e.manualThrottle       = manThr;
        e.heightM              = fcs.height_m;
        e.vertVelMps           = fcs.vertical_velocity_mps;
        BaroSample baroSample;
        if (sensors.baroSnap.read(baroSample)) {
            e.baroRelM = baroSample.relativeAltitudeM;
            e.baroAbsM = baroSample.altitudeM;
        }

        // ---- 布局 v2: 水平状态 ----
        // 16 个状态一次取全(水平速度/位置与陀螺零偏都在里面), 免得为省几十纳秒调两遍。
        // 记录 EKF 水平速度和位置，供事后核对观测融合及控制效果。
        const FlightFilterState16 state16 = ekf.getState16();
        e.velXyMps[0]                = state16.data[FF_VX];
        e.velXyMps[1]                = state16.data[FF_VY];
        e.posXyM[0]                  = state16.data[FF_PX];
        e.posXyM[1]                  = state16.data[FF_PY];
        if ((logCounter % cfg::ekf::kLogDivider) == 0u) { // 零偏变化慢, 隔几条记一次
            e.gyroBias[0] = state16.data[FF_BGX];
            e.gyroBias[1] = state16.data[FF_BGY];
            e.gyroBias[2] = state16.data[FF_BGZ];
        }
        // ---- 布局 v2: 光流 / 测距 ----
        // 记的是**原始增量 + 质量 + 有效位**, 不是换算后的速度: 事后要能分清
        // "光流没数据"、"有数据但被门限拒了"、"数据进了 EKF 但尺度算错" 这三种情况。
        e.rangeM    = rangeM;
        e.flowDtMs  = lastFlow.dtMs;
        e.flowDx    = lastFlow.dx;
        e.flowDy    = lastFlow.dy;
        e.flowSqual = lastFlow.squal;
        e.flowFlags = static_cast<uint8_t>(flowFlags);
        for (int i = 0; i < 4; ++i) {
            e.motor[i] = motor[i];
        }
        logQueuePush(sh.logQueue, e);
        pushDebugRecord(); // 融合状况调试流: 与飞行日志同一节奏(仅串口调试模式真正入队)
        eventFlags  = 0;
        lastLoopCyc = loopStartCyc;
    }

    /** 统计控制周期负载；启动热身期不计入统计窗口。 */
    bool stepPerf()
    {
        const uint32_t loopCycles = DWT->CYCCNT - loopStartCyc;
        const bool overrun = loopCycles > SystemCoreClock / 1000u * app::ControlPeriodMs;
        if (overrun && !loopOverrun) DLX_HANDLE_EXCEPTION(ExceptionCode::SCHED_LOOP_OVERRUN);
        loopOverrun = overrun;
        if (perfWarmup > 0u) {
            --perfWarmup;
            lastPerfMs  = now_ms;
            lastLoopCyc = loopStartCyc;
            sh.perf     = {};
            return false;
        }
        ++sh.perf.samples;
        if (loopCycles > sh.perf.loopMax) {
            sh.perf.loopMax = loopCycles;
        }
        sh.perf.loopSum += loopCycles;
        if ((now_ms - lastPerfMs) >= cfg::time::log::kPerfReportMs) {
            lastPerfMs     = now_ms;
            PerfStats perf = {};
            rt_enter_critical();
            perf.loopAvgCycle = sh.perf.samples ? (sh.perf.loopSum / sh.perf.samples) : 0u;
            perf.loopMaxCycle = sh.perf.loopMax;
            perf.ekfCycle     = sh.perf.ekfCnt ? (sh.perf.ekfSum / sh.perf.ekfCnt) : 0u;
            perf.ctrlCycle    = sh.perf.ctrlCnt ? (sh.perf.ctrlSum / sh.perf.ctrlCnt) : 0u;
            perf.ioCycle      = sh.perf.ioCnt ? (sh.perf.ioSum / sh.perf.ioCnt) : 0u;
            perf.logCycle     = sh.perf.logCnt ? (sh.perf.logSum / sh.perf.logCnt) : 0u;
            perf.samples      = sh.perf.samples;
            perf.drops        = sh.logQueue.dropped;
            sh.performance.store(perf);
            sh.perf.loopSum   = 0;
            sh.perf.loopMax   = 0;
            sh.perf.ekfSum    = 0;
            sh.perf.ekfCnt    = 0;
            sh.perf.ctrlSum   = 0;
            sh.perf.ctrlCnt   = 0;
            sh.perf.ioSum     = 0;
            sh.perf.ioCnt     = 0;
            sh.perf.logSum    = 0;
            sh.perf.logCnt    = 0;
            sh.perf.samples   = 0;
            sh.logQueue.dropped = 0;
            sh.debugQueue.dropped = 0;
            rt_exit_critical();
        }
        return true;
    }

    // 主流程
    void initialize()
    {
        ExceptionRecorder bringUpRec;
        state = app::SystemState::InitLow;
        stateEnteredMs = rt_tick_get_millisecond();
        bootLogLine("\r\n[Boot] ===== %s: 启动自检开始 =====\r\n",
                (app.mode == BootMode::Flight) ? "飞行模式" : "串口调试模式");
        logSystemReport();

        // 初始化阶段由控制线程直接推进，不伪装成跨线程请求。
        state = app::SystemState::InitHigh;
        stateEnteredMs = rt_tick_get_millisecond();
        bringUpSensors(bringUpRec);

        bringUpMotors();

        const bool linkReady = bringUpLink(bringUpRec);
        /** 初始化失败仍进入可观察的地面态，是否允许解锁由启动门槛独立决定。 */
        // flash 由 main 初始化；此处仅报告探测与文件系统状态。
        bootLogLine("[Boot] · 检查点: flash 只读探测\r\n");
        // 判据用 isReady(): "文件系统就绪 会话=0 预留=0 槽 上限=0 条" 这组全 0 的数字,
        // 恰恰是 init 根本没跑成功的特征(预留槽至少是 1). 未就绪时必须退回只读探测,
        // 至少还能看出"芯片在不在、JEDEC 是几"
        if (app.fs != nullptr && app.fs->isReady()) {
            bootLogLine("[Dev ] W25Q128 : 文件系统就绪 会话=%u 预留=%u 槽 上限=%u 条 已写=%u 条\r\n",
                    static_cast<unsigned>(app.fs->currentSessionId()),
                    static_cast<unsigned>(app.fs->sessionReservedSlots()),
                    static_cast<unsigned>(app.fs->sessionEntryLimit()),
                    static_cast<unsigned>(app.fs->currentEntryCount()));
        } else {
            logFlashProbe();
        }
        bootLogLine("[Dev ] 电机    : TIM1 PE9/PE11/PE13/PE14 DShot%ukHz\r\n",
                static_cast<unsigned>(cfg::hw::motor::kDshot));

        bootLogLine("[Boot] · 检查点: EKF + 观测层参数\r\n");
        initEkf();
        initObservers();

        bootLogLine("[Boot] · 检查点: 建线程(堆最大的一次开销)\r\n");
        if (app.mode != BootMode::Flight) {
            replyGround(app, static_cast<uint16_t>(GroundOp::Unlock), app::GroundStatus::Ok, 0,
                        static_cast<uint32_t>(app.mode));
        }

        app.serialDma->wait();
        app.protocol->setUnBlockCallbackFunction(nullptr, nullptr);
        app.protocol->setGroundCmdCallbackFunction(nullptr, nullptr);
        app::bindControlChannels(&sh.control);
        app::bindFaultChannel(&sh.faults);
        startThreads(bringUpRec);

        state = (app.mode == BootMode::Flight) ? app::SystemState::Standby
                                                : app::SystemState::Maintenance;
        armOrigin = state;
        stateEnteredMs = rt_tick_get_millisecond();

        if (app.mode != BootMode::Flight) {
            bootLogLine("[Boot] ===== 启动自检结束: BMI088=%s 链路=%s flash=%s 线程=%s =====\r\n",
                    sensors.imuInitialized ? "OK" : "FAIL",
                    linkReady ? "E220 OK" : "E220 FAIL",
                    (app.fs != nullptr && app.fs->isReady()) ? "文件系统"
                                                            : "不可用(见上面的 JEDEC/err)",
                    threadsReady() ? "OK" : "失败");
            // 注意: 这一行拆成两条打. 合成一条是 168 字节(UTF-8 汉字 3 字节/个), 超过
            // textmbox::LINE_MAX_BYTES(160) 会被截断 —— 现场就看到它被切成
            // "...kDebugStatMs  " + ")" 两行(而且切断点还落在汉字中间).
            bootLogLine("[Boot] 进入串口调试模式(可台架试车): 日志 %ums 一条 + 负载统计 %ums 一条\r\n",
                    static_cast<unsigned>(cfg::time::log::kDebugPeriodMs),
                    static_cast<unsigned>(cfg::time::log::kPerfReportMs));
            bootLogLine("[Boot] 运行状态 %ums 一条(改 cfg::time::log::kDebugStatMs 可关)\r\n",
                    static_cast<unsigned>(cfg::time::log::kDebugStatMs));
        }
        bootLogLine("[Boot] · 检查点: 启动软定时器(心跳灯从这一刻起才开始翻转)\r\n");
        const auto startTimer = [&](rt_timer_t &timer) {
            if (timer != RT_NULL && rt_timer_start(timer) != RT_EOK) {
                rt_timer_delete(timer);
                timer = RT_NULL;
                bringUpRec.throws(ExceptionCode::SCHED_THREAD_START_FAILED);
            }
        };
        startTimer(ctrlTimer);
#if CFG_FEATURE_LINK
        if (sh.tidLink != RT_NULL) startTimer(linkTimer);
#endif
        ENABLE_EXCEPTION_DLX();
        bootLogLine("[Boot] · 检查点: 进入控制环(自检到此结束); 上电期共记录 %u 条异常\r\n",
                static_cast<unsigned>(bringUpRec.size()));
        if (app.channel != nullptr && app.channel->exceptionMailbox != nullptr) {
            for (uint8_t index = 0u; index < bringUpRec.size(); ++index) {
                app.channel->exceptionMailbox->push(bringUpRec.at(index));
            }
        }
        flushBootLog(app);
        if (sh.tidLog != RT_NULL) startTimer(logTimer);
        startupReady = threadsReady();
    }

    void run()
    {
        initialize();
        while (true) {
            if (ctrlTimer != RT_NULL) rt_sem_take(ctrlSem, RT_WAITING_FOREVER);
            else rt_thread_mdelay(app::ControlPeriodMs);
            stepSampleShared();
            stepImu();
            stepObservers();
            stepAssembleInputs();
            stepFaults();
            stepStateMachine();
            stepControl();
            stepPublish();
            stepLog();
            stepPerf();
        }
    }
};

} // namespace dlx
