#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "dlx_bytebuffer.hpp"
#include "dlx_gpio.hpp"
#include "dlx_usart.hpp"
#include "dlx_dma.hpp"
#include "dlx.hpp" // 自动生成的协议(现在有 #pragma once, 可以多处包含)
#include "W25Q128/dlx_flash_manager_config.h" // LogEntry / 帧常量(FlashManager 类用前置声明)
#include "dlx_sensor_data.hpp"
#include "dlx_gpio_profile.h"
#include "dlx_spi_profile.hpp"
#include "dlx_timer_profile.hpp"
#include "dlx_dshot_profile.hpp"
#include "flight_control/flight_controller.hpp"
#include "flight_control/flight_control_fliter.hpp"

//===================================================================================================
// flight_config_struct.hpp —— 发布项唯一的公共头文件
//
// 内容:
//   0. 编译期总开关(目前一个: DLX_BARO_ENABLED, 一键禁用气压计);
//   1. FlightConfig / kConfig —— 全部编译期配置: 时序、日志、地面模式、传感器/执行器、EKF、硬件引脚;
//      (飞控 PID / Mixer 参数不在这里, 它们属于 flight_control_params.hpp)
//   2. 上电模式 BootMode / BootDecision;
//   3. 地面模式命令与应答的"字段约定"(枚举), 报文本体由 KSP 从 Kotlin 类生成 → DL_LIB/dlx.hpp;
//   4. 运行时状态 FlightRuntimeState(跨线程共享) 与日志状态位/事件位;
//   5. 发布项共用的上下文 ReleaseContext 与串口/日志小工具(logLine/flushSerial/replyGround/
//      drainDlx/onFlashBusy/writeLogEntry) —— release.cpp 与 release_log_manage.hpp 都用它们;
//   6. 少量纯函数辅助(传感器轴重映射)。
//
// 报文写法: 用生成出来的 protocol.GroundCmdW()/GroundReplyW()/SessionInfoW()/PerfMonitorW()
//           /FlightLogW() 和 setGroundCmdCallbackFunction()/setUnBlockCallbackFunction(),
//           见 notebook/2026-09-11_dlx_protocol.md。
//===================================================================================================
namespace dlx
{

    class FlashManager; // 只用指针, 前置声明即可(避免头文件依赖整个文件系统实现)

    /** 发布项版本(改配置/日志含义时 +1, 便于事后分辨日志是哪一版) */
    constexpr uint16_t FLIGHT_CONFIG_VERSION = 1;

    //===============================================================================================
    // 零、编译期总开关(改这一处就能裁掉整块功能, 不用进 release.cpp 里翻代码)
    //===============================================================================================
    /**
     * 气压计(BME280)总开关: 1 = 启用(默认), 0 = 完全禁用。
     *
     * 置 0 是**编译期裁剪**, 不是运行时跳过开关:
     *   - 不初始化 I2C1(PB6/PB7)、不驱动 SDO 地址脚(PD2)、不做初始化重试(省掉最多 2s 的开机等待);
     *   - 控制环里不再读气压、不往 EKF 喂气压高度 ⇒ 高度/垂速退回纯 IMU 积分(会缓慢漂移),
     *     FlightControlState.vertical_velocity_valid 恒 false(定高不可用);
     *   - 遥测 height 字段语义退回"高度"(kTelStatusBaroOk 恒 0), 日志里 kLogFlagBaroOk 恒 0;
     *     这是主动关掉, 所以不再置 kTelErrBaro —— 不把"关了"当成"坏了"报上去。
     *
     * 想临时做 A/B 对比(不改这两个文件的代码), 就在编译器的全局宏里定义 DLX_BARO_ENABLED=0
     * 覆盖本文件(Keil: Options for Target → C/C++ → Define)。
     */
#ifndef DLX_BARO_ENABLED
#define DLX_BARO_ENABLED 1
#endif

    /** 气压计开关的 C++ 常量视图(给 #if 不方便写的地方用; 恒定表达式 ⇒ 优化后无分支) */
    constexpr bool kBaroEnabled = (DLX_BARO_ENABLED != 0);

    //===============================================================================================
    // 一、上电模式
    //===============================================================================================
    /**
     * 上电后可能进入的三种模式:
     *   Flight      : 正常飞行。跳线帽没接时**直接**进入, 完全不看串口;
     *   SerialDebug : 串口直连调试(台架)。接跳线帽 + 上位机"解锁"命令选中; 日志走串口不写 flash, 可解锁电机;
     *   LogManage   : 日志管理。接跳线帽 + 上位机"解锁"命令选中; 只读/删/回传/格式化 flash, **永不输出电机**。
     */
    enum class BootMode : uint8_t
    {
        Flight      = 0, ///< 正常飞行(日志写 flash)
        SerialDebug = 1, ///< 串口直连调试(日志走串口, 可台架试车)
        LogManage   = 2, ///< 日志管理(不动电机)
    };

    /** 上电模式判定的结论 */
    struct BootDecision
    {
        BootMode mode;       ///< 最终进入的模式
        bool     groundMode; ///< true = 跳线帽接上了(地面模式)
        bool     byCommand;  ///< true = 模式由上位的"解锁"命令指定, false = 跳线帽/默认
        uint8_t  rawParam;   ///< 命令里带的原始模式参数(便于回显/排查)
    };

    //===============================================================================================
    // 二、地面模式命令的字段约定(报文本体由 KSP 生成, 见 dlx.hpp)
    //
    //   GroundCmd(op,mode,sessionId,count) / GroundReply(op,status,sessionId,value)
    //   SessionInfo(sessionId,slotCount,entryCount,active,reserved) / PerfMonitor(6×cycle,samples,drops)
    //===============================================================================================
    /** 地面命令 op: 与 Kotlin 侧 GroundOp 一一对应(两边同时改) */
    enum class GroundOp : uint16_t
    {
        Unlock       = 1, ///< 解锁并进入 mode 指定的模式(0 飞行 / 1 串口调试 / 2 日志管理)
        ListSessions = 2, ///< 列出全部日志会话(逐条 SessionInfo + 一条汇总应答)
        DeleteOldest = 3, ///< 丢掉最老的 count 个会话(环形队列语义, 腾出的槽并入当前会话)
        DumpSession  = 4, ///< 按原始日志帧流回传整个会话
        FormatFlash  = 5, ///< 整片擦除重建(丢日志+参数, 几十秒)
        Ping         = 6, ///< 空指令, 回一条应答用于探链路
    };

    /** 地面应答 status */
    enum class GroundStatus : uint16_t
    {
        Ok          = 0, ///< 成功
        BadCommand  = 1, ///< 未知命令
        BadArgument = 2, ///< 参数非法
        FlashFailed = 3, ///< flash 操作失败
        NotFound    = 4, ///< 找不到会话/条目
        Busy        = 5, ///< 忙(擦除中等)
    };

    //===============================================================================================
    // 三、编译期配置
    //===============================================================================================
    /** 硬件接线: 引脚 / SPI 模式 / 设备地址 / 缓冲大小 */
    struct HardwareConfig
    {
        // ---- 指示灯与跳线 ----
        GPIOProfile ledRun{GPIOProfile::F3};    ///< 运行心跳 / 地面模式等待闪烁(高电平点亮)
        GPIOProfile ledFlash{GPIOProfile::F2};  ///< flash 擦除进度(低电平点亮)
        GPIOProfile jumpA{GPIOProfile::B8};     ///< 地面模式跳线 A
        GPIOProfile jumpB{GPIOProfile::B9};     ///< 地面模式跳线 B

        // ---- SPI 设备 ----
        GPIOProfile   flashCs{GPIOProfile::D1};                        ///< W25Q128 片选
        SPIModeProfile flashSpi{SPIModeProfile::FD_M_8B_CL_1E_NS_BR8_MSB};  ///< SPI2 @5.25MHz
        GPIOProfile   imuSpiNss{GPIOProfile::A3};                      ///< BMI088 所在 SPI1 的 NSS(AF 用)
        SPIModeProfile imuSpi{SPIModeProfile::FD_M_8B_CH_2E_NS_BR16_MSB};   ///< SPI1 @5.25MHz
        GPIOProfile   imuAccelCs{GPIOProfile::BC};                     ///< BMI088 加速度计 CS
        GPIOProfile   imuGyroCs{GPIOProfile::BD};                      ///< BMI088 陀螺 CS
        GPIOProfile   nrfSpiNss{GPIOProfile::A4};                      ///< NRF24L01 所在 SPI3 的 NSS(AF 用)
        SPIModeProfile nrfSpi{SPIModeProfile::FD_M_8B_CL_1E_NS_BR32_MSB};   ///< SPI3 @1.3MHz
        GPIOProfile   nrfCe{GPIOProfile::C7};                          ///< NRF24L01 CE
        GPIOProfile   nrfCsn{GPIOProfile::C8};                         ///< NRF24L01 CSN

        /**
         * BME280 的 SDO(地址选择)脚: 必须**恒为低** ⇒ 从机地址固定 0x76
         *
         * 这块板上 SDO 接的是 PD2 而不是直接接地。SDO 悬空/被扰动时电平会漂, 从机地址就在
         * 0x76/0x77 之间翻(实测被 flash 擦除那阵干扰一激就翻), 表现为气压计"原地址不 ACK(AF=1)"
         * —— 地址是每次 I2C 事务实时读的, 不是上电锁存的, 所以必须由 MCU 钉住。
         */
        GPIOProfile   bmeSdo{GPIOProfile::D2};                         ///< BME280 SDO(地址选择, 驱动为低)

        // ---- I2C 气压计 ----
        uint16_t bmeAddr{0x76};      ///< BME280 7 位地址(SDO 接高改成 0x77)
        /**
         * I2C1 速率 [Hz]
         *
         * 100000 是稳妥值。想提"最坏 slot 里那次 BME280 读"的耗时可以试 400000
         * (调试模式下实测 400k 稳定), 前提是 SDO 地址脚已经钉死(见 hw.bmeSdo) ——
         * 2026-09-13 那次"飞行模式必失败"其实是 SDO 悬空导致地址翻转, 不是速率的锅,
         * 详见 notebook/2026-09-11_release_item.md §8.4。
         */
        uint32_t bmeSpeed{400000};   ///< I2C1 速率(100000 = 标准模式; 想提速可试 400000)

        // ---- 电机 ----
        AdvancedTimerProfile motorTimer{AdvancedTimerProfile::TIM1_Up_DIV1}; ///< TIM1 -> PE9/PE11/PE13/PE14
        DshotProfile         dshotMode{DshotProfile::Dshot300};              ///< DShot300

        // ---- 缓冲大小 ----
        uint32_t rxBufBytes{256};    ///< 串口接收环形缓冲
        uint32_t txBufBytes{2048};   ///< 串口发送缓冲(绑 DMA; 也是日志流/会话回传缓冲)
        uint32_t frameBufBytes{256}; ///< DLX 解帧临时区(不小于最大负载: FlightDebug 136B)
        uint32_t textLineBytes{160}; ///< 文本日志行缓冲
    };

    /** 行为配置: 时序 / 日志 / 地面模式 / 传感器 / EKF */
    struct FlightConfig
    {
        // ---- 循环 / 时序 ----
        float    loopHz{500.0f};      ///< 控制环频率 [Hz](与 ctrlTickMs 对应)
        uint32_t ctrlTickMs{2};       ///< 控制线程唤醒周期 [ms]
        uint32_t telTickMs{20};       ///< NRF 遥测线程周期 [ms] (50Hz)
        uint32_t logTickMs{5};        ///< 日志线程唤醒周期 [ms]
        uint32_t serialBaud{2000000}; ///< 调试串口波特率

        // ---- 日志 ----
        uint32_t flightLogPeriodMs{20};       ///< 飞行模式日志周期 [ms] (20ms = 50Hz)
        uint32_t debugLogPeriodMs{10};        ///< 串口调试模式日志周期 [ms] (10ms = 100Hz)
        uint32_t perfReportMs{200};           ///< 串口调试模式负载上报周期 [ms]
        uint32_t flightLogSlots{32};          ///< 飞行模式单次日志预留槽数(1 槽 = 128KB)
        // 日志管理模式的预留槽数: 管理模式要调 init() 才能读会话(ready 由 startSession 置位),
        // 所以必然建一个空会话, 这里填 1 就是"最小合法值"(FlashManager 内部 clamp 到 1~槽数, 填 0 也会变 1)
        uint32_t manageLogSlots{1};           ///< 日志管理模式预留槽数(最小 1, 不飞行不写日志)
        bool     autoFormatOnMismatch{false}; ///< 布局不符(改过 LogEntry)时自动整片重建: 会丢旧日志, 默认关
        uint16_t logQueueDepth{16};           ///< 控制线程 -> 日志线程的队列深度(条)

        // ---- 地面模式(跳线帽) ----
        uint32_t groundWaitTimeoutMs{0}; ///< 等"解锁"命令的超时 [ms]; 0 = 一直等
        uint32_t groundBlinkMs{120};     ///< 等命令时状态灯闪烁周期 [ms]
        uint32_t manageBlinkMs{500};     ///< 日志管理模式的慢闪周期 [ms]

        // ---- 传感器 / 执行器 ----
        float    imuRateHz{2000.0f};      ///< BMI088 陀螺 ODR(必须与 dlx_bmi088_config.h 一致)
        int32_t  imuYawDeg{0};            ///< IMU 相对机体绕 z 的偏转(0/90/180/270)
        int32_t  motorMap[4]{0, 2, 3, 1}; ///< 逻辑电机(0..3) -> 物理 DShot 通道
        float    dshotUnit{1900.0f};      ///< 归一化油门 -> DShot 计数的比例
        float    dshotOffset{50.0f};      ///< DShot 计数偏置
        uint32_t motorArmDelayMs{3200};   ///< 解锁后到控制接管的等待 [ms]
        uint32_t linkTimeoutMs{5000};     ///< 遥控超时保护 [ms]
        float    minThrottle{0.01f};      ///< 角度环的最小集体油门(混控要有非零油门才能出力矩)

        // ---- 传感器融合(EKF) ----
        FlightControlFilterParams ekf{};      ///< EKF 参数基表(下面三项是实测整定覆盖)
        bool     ekfEstimateAccelBias{false}; ///< 在线估加速度零偏(纯 IMU 不可观, 默认关)
        float    ekfAccelTiltGateMss{2.0f};   ///< 倾角修正门限 [m/s^2](与 main_att_ekf 一致)
        float    ekfBaroSigmaM{1.0f};         ///< 气压高度噪声 [m](BME280 本征噪声约 ±0.5~1m)
        uint32_t ekfDecim{8};                 ///< 每 N 个 IMU 样本做一次协方差传播 + 倾角更新
        uint32_t baroPeriodMs{20};            ///< 气压计读取周期 [ms]
        uint32_t ekfLogDivider{10};           ///< 每 N 条日志记一次 EKF 零偏(降带宽)

        uint32_t perfWarmupLoops{5};        ///< 起跑热身圈数: 前 N 圈不计入负载统计(含 FIFO 积压/EKF 首次整定)
        bool     logStackWatermark{false};  ///< 串口调试模式每 10s 打一行线程栈水位线(量栈预算时临时开)

        // ---- 气压计健壮性 ----
        /**
         * 气压计初始化重试: 失败就隔 baroInitRetryMs 再试, 最多 baroInitRetry 次。
         *
         * 存在的理由: 开机要先初始化 flash, 那一段(片内擦写)的干扰可能让气压计一时不应答;
         * 与其固定加一段开机延时, 不如重试 —— 健康机器第一次就成、一点不多等, 有问题的机器
         * 自动等到总线可用(实测 2026-09-13: SDO 悬空导致地址在 0x76/0x77 之间翻就是这种表现)。
         */
        uint32_t baroInitRetry{20};         ///< 气压计初始化最多重试几次
        uint32_t baroInitRetryMs{100};      ///< 重试间隔 [ms](默认 20 × 100ms = 最多等 2s)
        /**
         * 运行时连续读失败多少次就认定气压计不可用(默认 10 × 20ms = 200ms)。
         *
         * 判定后: 不再往 EKF 喂气压观测、遥测里 height 字段语义切回"高度"、error 的
         * kTelErrBaro 置位; 之后只要读成功一次就恢复。目的是让"气压计中途掉线"这件事
         * 在遥控器/日志里看得见, 而不是悄悄把垂速当高度显示。
         */
        uint32_t baroFailLimit{10};         ///< 气压计连续读失败阈值(次)

        // ---- 硬件 ----
        HardwareConfig hw{};
    };

    /** 全局唯一配置实例(改这里就等于改配置) */
    constexpr FlightConfig kConfig{};

    // ---- 由配置推导的常量 ----
    constexpr float kLoopDt   = 1.0f / kConfig.loopHz;         ///< 控制周期 [s]
    constexpr float kImuDt    = 1.0f / kConfig.imuRateHz;      ///< IMU 样本间隔 [s]
    constexpr float kRadToDeg = 57.29578f;
    constexpr float kDegToRad = 0.017453292f;

    //===============================================================================================
    // 四、日志状态位 / 事件位(LogEntry::flags / events)
    //===============================================================================================
    /** flags —— 电平型状态(这一条日志"当时处于什么状态") */
    enum : uint16_t
    {
        kLogFlagLinkOk        = 0x0001, ///< 遥控链路在线(链路延迟 < linkTimeoutMs)
        kLogFlagImuOk         = 0x0002, ///< BMI088 自检通过
        kLogFlagBaroOk        = 0x0004, ///< 气压计可用
        kLogFlagAngleLoop     = 0x0008, ///< 角度环正在工作(command==0 且已使能)
        kLogFlagMotorEnabled  = 0x0010, ///< 电机已使能(控制已接管)
        kLogFlagMotorStarting = 0x0020, ///< 电机启动序列进行中
        kLogFlagHardStop      = 0x0040, ///< 收到硬停机命令
        kLogFlagFlashOk       = 0x0080, ///< flash 日志可用(飞行模式)
        kLogFlagLogFull       = 0x0100, ///< flash 日志写满, 已停止记录
        kLogFlagLogError      = 0x0200, ///< flash 写入出错
        kLogFlagMotorSat      = 0x0400, ///< 有一路电机输出饱和
        kLogFlagFailsafe      = 0x0800, ///< 失控保护触发过(本会话)
        kLogFlagGroundMode    = 0x1000, ///< 处于地面模式(串口调试)
    };

    /** events —— 边沿型事件(这一条日志"期间发生了什么"), 随日志归档后清零 */
    enum : uint16_t
    {
        kLogEventCommandChanged = 0x0001, ///< 遥控指令字变化
        kLogEventArm            = 0x0002, ///< 解锁(启动电机)
        kLogEventDisarm         = 0x0004, ///< 停机
        kLogEventFailsafe       = 0x0008, ///< 触发失控保护
        kLogEventMotorSat       = 0x0010, ///< 出现电机饱和
        kLogEventLogFault       = 0x0020, ///< 日志写满/写错
        kLogEventBaroUpdate     = 0x0040, ///< 本周期更新了气压高度
        kLogEventEkfRezero      = 0x0080, ///< EKF 高度归零(解锁瞬间)
    };

    //===============================================================================================
    // 四之二、遥控回传位域(FlightCoreStatus 的 status / error, 各 8 位)
    //===============================================================================================
    /**
     * status —— 实时状态位(每次回传都按当前状态重算, 不是锁存的)
     *
     * 注意: 判断 height 字段里装的是"垂速"还是"高度", 看的是 kTelStatusBaroOk;
     * error 里的 kTelErrBaro 是同一事实的"锁存版", 不能拿它来判断(见下)。
     */
    enum : uint8_t
    {
        kTelStatusMotorEnabled  = 0x01, ///< 电机已使能(解锁完成, 控制已接管)
        kTelStatusMotorStarting = 0x02, ///< 启动序列进行中
        kTelStatusAngleLoop     = 0x04, ///< 角度环正在工作
        kTelStatusLinkOk        = 0x08, ///< 遥控链路在线
        kTelStatusHardStop      = 0x10, ///< 收到硬停机命令
        kTelStatusFailsafe      = 0x20, ///< 失控保护触发过(本会话锁存)
        kTelStatusLogFull       = 0x40, ///< flash 日志写满, 已停止记录
        kTelStatusBaroOk        = 0x80, ///< 气压计可用 ⇒ height 字段装的是垂速 [m/s], 否则是高度 [m]
    };

    /**
     * error —— 错误位(只放真故障, 平时 0)
     *
     * 飞控只报**这一时刻**的错误位: 问题解决了就回 0, 不做锁存。
     * "错误一闪而过也不要漏看"是显示端的事(遥控器 / 上位机自己按位或锁存),
     * 见 notebook/2026-09-13_remote_compat.md。
     */
    enum : uint8_t
    {
        kTelErrLogWrite = 0x01, ///< flash 日志写入失败
        kTelErrImu      = 0x02, ///< BMI088 自检失败
        kTelErrBaro     = 0x04, ///< 气压计不可用
    };

    //===============================================================================================
    // 五、跨线程运行时状态
    //===============================================================================================
    /** 遥测(控制线程 -> NRF 线程) */
    struct FlightTelemetry
    {
        Quaternion quat{{1.0f, 0.0f, 0.0f, 0.0f}}; ///< EKF 姿态(机体 -> 世界)
        uint16_t   motor[4]{0, 0, 0, 0};           ///< 四路电机输出(DShot 计数)
        float      height_m{0.0f};                 ///< EKF 高度 [m]
        float      vertical_vel_mps{0.0f};         ///< EKF 垂向速度 [m/s]
        uint8_t    status{0};                      ///< 回传状态位(实时, 见 kTelStatus*)
        uint8_t    error{0};                       ///< 回传错误位(粘滞, 见 kTelErr*)
        bool       enabled{false};                 ///< 电机是否已使能
    };

    /** 计算负载统计(控制线程填, 日志线程发) */
    struct PerfStats
    {
        uint32_t loopAvgCycle{0}; ///< 主循环平均耗时 [CPU 周期]
        uint32_t loopMaxCycle{0}; ///< 主循环最大耗时
        uint32_t ekfCycle{0};     ///< EKF 慢路径平均耗时
        uint32_t ctrlCycle{0};    ///< 控制器 + 混控平均耗时
        uint32_t logCycle{0};     ///< 日志打包/落盘平均耗时
        uint32_t ioCycle{0};      ///< IMU FIFO + 气压计读取平均耗时
        uint16_t samples{0};      ///< 本统计窗口内的主循环次数
        uint16_t drops{0};        ///< 本窗口内因为队列满被丢掉的日志条数
    };

    /**
     * 飞行运行时状态 —— 取代原来的 IntegratedFlightState。
     *
     * 线程约定:
     *   - 遥控(NRF)线程只写"指令"部分, 控制线程只写"状态/遥测"部分, 两边都用 rt_enter_critical() 保护;
     *   - 日志线程只写 logError/logFull/logEntries(volatile), 控制线程读;
     *   - 硬件对象(Dshot/定时器)不放进来: 遥控线程只置标志, 真正停定时器由控制线程做。
     */
    struct FlightRuntimeState
    {
        // ---------------- 遥控线程 -> 控制线程 ----------------
        uint16_t              command{9};                    ///< 遥控指令字(9 = 无指令/待机)
        float                 manualThrottle{0.0f};          ///< 手动油门(0~1)
        FlightControlSetpoint setpoint{};                    ///< 期望姿态 / 期望高度
        uint16_t              motorValue[4]{50, 50, 50, 50}; ///< 直接电调测试用的四路输出
        bool                  startMotor{false};             ///< 收到"启动电机"
        bool                  stopMotor{false};              ///< 收到"停机"
        bool                  hardStop{false};               ///< 收到硬停机(立刻关定时器/DMA)
        uint32_t              lastValidCtrl{0};              ///< 最近一次有效遥控指令的时间戳 [ms]

        // ---------------- 控制线程内部 ----------------
        bool     enabledMotor{false};  ///< 控制已接管(角度环在工作)
        bool     startingMotor{false}; ///< 启动序列进行中
        uint32_t motorStartTime{0};    ///< 启动序列起点 [ms]

        // ---------------- 控制线程 -> 遥测线程 ----------------
        FlightTelemetry telem{};

        // ---------------- 控制线程 -> 日志线程 ----------------
        PerfStats perf{}; ///< 最近一个统计窗口的负载(每 perfReportMs 更新一次)
        // ---------------- 日志线程 -> 控制线程 ----------------
        volatile bool     logError{false}; ///< 日志写入出错
        volatile bool     logFull{false};  ///< 日志写满
        volatile uint32_t logEntries{0};   ///< 已写入的日志条数
    };

    //===============================================================================================
    // 六、小工具
    //===============================================================================================
    /** 一条日志经 DLX 打包后的字节数(负载 140 + 帧头 2) */
    constexpr uint16_t kFlightLogPayloadBytes = 140;
    constexpr uint16_t kFlightLogFrameBytes   = kFlightLogPayloadBytes + 2u;

    /**
     * @brief 两个源文件共享的上下文(硬件句柄 + 当前模式)
     *
     * release.cpp(飞行/调试 + 上电判定) 与 release_log_manage.hpp(日志管理) 都拿这个结构当参数。
     */
    struct ReleaseContext
    {
        USART              *usart;     ///< 调试串口
        DMA                *serialDma; ///< 串口 DMA 发送
        ByteBuffer         *txBuf;     ///< 发送缓冲(与 DMA 绑定)
        RingByteBuffer     *uartRx;    ///< 串口接收环形缓冲
        DLX_ProtocolBuffer *protocol;  ///< DLX 协议(生成代码; 收发地面模式报文与日志)
        FlashManager       *fs;        ///< 文件系统(串口调试模式为 nullptr)
        GPIO               *ledRun;    ///< 运行指示灯
        BootMode            mode;      ///< 当前模式
    };

    /** 文本日志出口(飞行模式置 nullptr = 完全静音) */
    inline USART *&debugPort()
    {
        static USART *port = nullptr;
        return port;
    }

    /** 文本日志要用的串口/协议上下文(main 里绑定一次; 没绑定就不打印) */
    inline ReleaseContext *&logContext()
    {
        static ReleaseContext *ctx = nullptr;
        return ctx;
    }

    /** 文本日志行缓冲(单线程使用: 地面模式只有主线程打印) */
    inline char *textLineBuffer()
    {
        static char buf[kConfig.hw.textLineBytes];
        return buf;
    }

    /**
     * @brief "我们启动的 DMA 发送还没回收" 标志
     *
     * **重要坑**: `DMA::init()` 结尾会 `DMA_ClearFlag(TC)`(见 dlx_dma.hpp), 而 `wait()` 是
     * `while (TC == RESET);` —— 在还没启动过任何传输时直接 `wait()` 会**死等**,
     * 表现为 MCU 上电即"断片"(LED 不再心跳、串口全无)。
     * 所以本工程统一用这个标志: **只有我们自己 start() 过的传输才允许 wait()**。
     * (旧版 main_final_test.cpp 是在 setDMASend 之后先 `start()` 一次把 TC 置起来, 这里不依赖这个副作用。)
     */
    inline bool &serialDmaBusy()
    {
        static bool busy = false;
        return busy;
    }

    /** 启动一次 DMA 发送(先把上一次收掉, 不阻塞本次) */
    inline void dmaSendStart(ReleaseContext &ctx)
    {
        if (serialDmaBusy()) {
            ctx.serialDma->wait();
            serialDmaBusy() = false;
        }
        if (ctx.txBuf->used() == 0) {
            return;
        }
        ctx.serialDma->reset(ctx.txBuf->used());
        ctx.serialDma->start();
        serialDmaBusy() = true;
    }

    /** 把 txBuf 里的内容阻塞发完(仅地面模式用; 飞行/调试模式的流转发在日志线程里) */
    inline void flushSerial(ReleaseContext &ctx)
    {
        dmaSendStart(ctx);
        if (serialDmaBusy()) {
            ctx.serialDma->wait(); // 这次传输是我们自己启动的, 一定会完成
            serialDmaBusy() = false;
        }
        ctx.txBuf->reset();
    }

    /**
     * @brief 打印一行文本(打成 DLX 的 SimpleLog 报文发出去)
     *
     * 关键: 串口上**只能有 DLX 帧**。早期版本用 usart1.send() 直接发 ASCII 文本, 上位机把它当帧解析,
     * 会刷出 "Header错误5b42 / 未知Header" 这种乱码(5b42 = '[' 'B', 来自 "[Boot] ..."), 而且一旦错位
     * 生成代码按 2 字节步进, 很难自己恢复。所以这里统一走 SimpleLog(类型 0), 上位机解析后显示在屏幕上。
     *
     * 1) 长度补齐成偶数, 保证每一帧都是偶数长度(减少上位机从中间接入时的错位概率);
     * 2) 阻塞发送, 保证一帧完整出现在线上。
     */
    inline void logLine(const char *fmt, ...)
    {
        if (debugPort() == nullptr) {
            return; // 飞行模式: 完全静音
        }
        ReleaseContext *ctx = logContext();
        if (ctx == nullptr || ctx->txBuf == nullptr || ctx->protocol == nullptr) {
            return;
        }
        char *line = textLineBuffer();
        va_list args;
        va_start(args, fmt);
        int n = vsnprintf(line, kConfig.hw.textLineBytes, fmt, args);
        va_end(args);
        if (n < 0) {
            n = 0;
        }
        if (n > (int)kConfig.hw.textLineBytes - 1) {
            n = (int)kConfig.hw.textLineBytes - 1;
        }
        if ((n & 1) != 0) {         // 补齐到偶数长度(SimpleLog 帧 = 4 + len)
            line[n++] = ' ';
            line[n]   = '\0';
        }
        if (n == 0) {
            return;
        }
        ctx->txBuf->reset();
        if (!ctx->protocol->SimpleLogW(reinterpret_cast<int8_t *>(line), static_cast<uint16_t>(n))) {
            return;
        }
        flushSerial(*ctx); // 阻塞发完, 一帧不拆
    }

    /**
     * @brief 把接收缓冲里能解析的帧全部解析掉
     *
     * check() 一次调用只推进一帧(噪声字节更是一次只吞 1 个), 所以要把积压推平。
     *
     * 退出条件就是 FAILED_INSUFFICIENT —— 它的语义是"数据不足, 不算错"(半帧或连帧头都不够),
     * 且约定一个字节都不消费(变长帧先 peek 长度, 没凑够就不动缓冲)。其余返回值都是
     * "消费了但失败/成功": SUCCESS 与 FAILED_CRC 吞整帧(2 + len), FAILED_HEAD 吞 1 个,
     * 所以循环每轮至少前进 1 个字节, 单调推进、一定会退出, 剩下的半帧留给下一次调用。
     * (FAILED_OVERFLOW 是预留项: 可变长帧超过解帧缓冲区属于设计失误, 当前不处理。)
     */
    inline void drainDlx(DLX_ProtocolBuffer &protocol, RingByteBuffer &rx)
    {
        while (rx.available() >= 2u && protocol.check() != DLXCheckResult::FAILED_INSUFFICIENT) {
        }
    }

    /** 发一条地面应答(用 KSP 生成的 GroundReplyW) */
    inline void replyGround(ReleaseContext &ctx, uint16_t op, GroundStatus status, uint32_t sessionId,
                            uint32_t value)
    {
        ctx.txBuf->reset();
        ctx.protocol->GroundReplyW(op, static_cast<uint16_t>(status), sessionId, value);
        flushSerial(ctx);
    }

    /** 擦除进度灯的当前相位(低电平点亮); 用函数局部静态保存, 便于结束时统一熄灭 */
    inline bool &flashLedPhase()
    {
        static bool phase = false;
        return phase;
    }

    /** flash 擦除/格式化进度回调: 每擦完一个 64KB 块翻一次擦除指示灯(低电平点亮) */
    inline void onFlashBusy()
    {
        static GPIO led(kConfig.hw.ledFlash);
        static bool inited = false;
        if (!inited) {
            led.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
            led    = 1;
            inited = true;
        }
        flashLedPhase() = !flashLedPhase();
        led             = flashLedPhase() ? 0 : 1;
    }

    /**
     * @brief 擦除/格式化结束时把进度灯熄灭
     *
     * 每擦完一个块只翻一次相位, 擦完正好停在"亮"的概率是 50% —— 不熄的话板上就多一颗
     * 含义不明的常亮灯, 排查时会被当成故障线索。
     */
    inline void onFlashIdle()
    {
        static GPIO led(kConfig.hw.ledFlash);
        static bool inited = false;
        if (!inited) {
            led.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
            inited = true;
        }
        flashLedPhase() = false;
        led             = 1; // 熄灭(低电平点亮)
    }

    /**
     * @brief 把一条日志(flash 里的 POD)打包成 DLX 的 FlightLog 帧发出去
     *
     * flash 里存的是 dlx::LogEntry(POD, CRC16, 掉电安全); 回传/调试流用 DLX 报文,
     * 上位机用生成代码解析, 不用手写字节偏移。
     *
     * @note 帧会追加到 protocol 的 buffer(也就是 txBuf)当前写指针处, 不 reset; 
     *       调用前请确认 remaining() >= kFlightLogFrameBytes。
     */
    inline bool writeLogEntry(DLX_ProtocolBuffer &protocol, const LogEntry &e)
    {
        return protocol.FlightLogW(e.sessionId, e.seq, e.tickMs, e.flags, e.events, e.linkAgeMs,
                                   e.loopPeriodUs, const_cast<float *>(e.quat),
                                   const_cast<float *>(e.bodyRate), const_cast<float *>(e.accelG),
                                   const_cast<float *>(e.rateSetpoint), const_cast<float *>(e.torque),
                                   e.targetPitchDeg, e.targetRollDeg, e.targetHeightM, e.throttle,
                                   e.manualThrottle, e.heightM, e.vertVelMps, e.baroRelM, e.baroAbsM,
                                   const_cast<float *>(e.gyroBias), const_cast<uint16_t *>(e.motor));
    }

    /**
     * @brief 传感器轴重映射: IMU 相对机体绕 z 轴偏转 yawDeg(CCW+, 0/90/180/270)
     *
     * 判定法: 手动抬机头应让 euler pitch 变、左右滚应让 euler roll 变;
     *         若两者相反, 把 kConfig.imuYawDeg 换成 90 或 270 试到一致。
     */
    inline void remapSensorFrame(int32_t yawDeg, float &x, float &y, float &z)
    {
        switch (yawDeg) {
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
} // namespace dlx
