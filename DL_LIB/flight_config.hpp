#pragma once
//===================================================================================================
// DL_LIB/flight_config.hpp —— 配置的"胶水层"：把生成出来的 cfg:: 常量树接进固件
//
// 分工(改配置时只动第 1 层, 绝不手改下面第 2、3 层里的**数值**):
//   1. kmpFly 的 Kotlin 声明（唯一真相）
//      E:\kmpFly\shared\src\jvmMain\kotlin\kmpfly\settings\FlightConfig.kt
//      类型 / 范围 / 单位 / 说明 / 取值 / 删除记录都在那里; 跨项约束在 FlightConfigRules.kt。
//   2. DL_LIB/generated/flight_config_gen.hpp  ← 由上面的声明导出(**禁止手改**)
//   3. 本文件                                  派生量 + 编译期守卫(static_assert)
//
// 流程(只有这一条路):
//      cd E:\kmpFly
//      .\gradlew.bat :shared:exportFlightConfig                   # 生成第 2 层 + 同名 JSON
//      .\gradlew.bat :shared:exportFlightConfig --args="--check"   # 只比对: 产物与声明是否一致
//      然后回本仓库编译。
//
//   ⚠ 2026-10-06 之前这里走的是 "config/flight_config.json → tools/gen_config_docs.py"。
//     那条路已退役: 它让「Kotlin 声明」与「固件实际拿到的常量」各说各话(13 个新增项
//     只存在于 JSON 里、kVelHoldEnable 两边取值不同, 而两端测试全在报"通过")。
//     config/flight_config.{json,schema.json} 现在只剩"导入前的遗留镜像"这一身份,
//     不再由任何人生成, 也不再被任何测试读取。
//
// 为什么要有这一层:
//   生成器只搬"配置项本身", 不该知道"控制周期 = 1/频率"这种派生关系, 也不该知道
//   "UART 波特率必须 ≤ 1.25×空中速率"这类跨项约束。这些放这里 —— 而且全部是 constexpr,
//   所以越界/自相矛盾的配置**在编译期就报错**, 不用等上板才发现。
//
// 与旧 kConfig 的关系: 旧版是一个聚合 POD `constexpr FlightConfig kConfig{}`, 成员是运行期类型,
//   既不能当模板参数也不能 static_assert。现在是命名空间里的散装 constexpr, 上面两件事都能做。
//   路径对应关系(旧 → 新)见 docs/2026-10-02_upgrade_plan_v3.md 与迁移脚本 tools/migrate_kconfig.py。
//===================================================================================================

#include "generated/flight_config_gen.hpp"
#include "W25Q128/dlx_flash_manager_config.h" // 只为下面的日志布局契约(纯类型定义, 无依赖)

namespace cfg
{

    //===============================================================================================
    // 一、由配置推导的常量
    //===============================================================================================
    /** 控制周期 [s] —— 与 time::ctrl::kRateHz 互为倒数(下面有 static_assert) */
    constexpr float kLoopDt = 1.0f / time::ctrl::kRateHz;
    /** IMU 样本间隔 [s] —— 与 hw::imu::kRateHz 互为倒数, 必须与 dlx_bmi088_config.h 的 ODR 一致 */
    constexpr float kImuDt = 1.0f / hw::imu::kRateHz;

    constexpr float kRadToDeg = 57.29578f;
    constexpr float kDegToRad = 0.017453292f;

    /** 固件配置版本(旧 FLIGHT_CONFIG_VERSION) */
    constexpr uint16_t kFirmwareVersion = version::kFirmware;

    //===============================================================================================
    // 二、枚举 → 数值的 constexpr 映射(static_assert 要用)
    //===============================================================================================
    /** 空中速率档 → bps。手册: 2.4/4.8/9.6/19.2/38.4/62.5 kbps(其中前三个编码都是 2.4k) */
    constexpr uint32_t airRateBps(AirRate r)
    {
        return (r == AirRate::k2_4k)    ? 2400u
               : (r == AirRate::k4_8k)  ? 4800u
               : (r == AirRate::k9_6k)  ? 9600u
               : (r == AirRate::k19_2k) ? 19200u
               : (r == AirRate::k38_4k) ? 38400u
                                        : 62500u; // k62_5k
    }

    /** E220 串口速率档 → bps(手册 REG0 位域) */
    constexpr uint32_t e220BaudBps(E220Baud b)
    {
        return (b == E220Baud::k1200)   ? 1200u
               : (b == E220Baud::k2400) ? 2400u
               : (b == E220Baud::k4800) ? 4800u
               : (b == E220Baud::k9600) ? 9600u
               : (b == E220Baud::k19200) ? 19200u
               : (b == E220Baud::k38400) ? 38400u
               : (b == E220Baud::k57600) ? 57600u
                                         : 115200u; // k115200
    }

    //===============================================================================================
    // 三、编译期守卫(跨项约束 —— 生成器管不了, 只能在这里守)
    //===============================================================================================
    // ---- 时序自洽 ----
    static_assert(kLoopDt == time::ctrl::kTickMs / 1000.0f,
                  "控制周期与控制线程唤醒周期对不上: 1/time.ctrl.kRateHz 必须等于 time.ctrl.kTickMs/1000");
    static_assert(time::link::kTickMs % time::ctrl::kTickMs == 0,
                  "链路槽长必须是控制周期的整数倍, 否则 TDMA 相位与控制环会互相啃");
    // 注意: 日志线程**不是**相位锁定的(它有自己的 5ms 定时器, 只负责把队列排空), 所以
    // time.log.kTickMs 不要求是控制周期的整数倍 —— 别在这里加那条断言(加过, 编译期就报了)。
    static_assert(link::tdma::kGuardMs < time::link::kTickMs,
                  "TDMA guard 必须小于槽长, 否则下行窗口装不下");
    // 窗口延时 = 槽长 − 主端帧收发时延 ⇒ 时延估计必须**小于**槽长, 否则窗口会开在下一个槽里
    static_assert(link::tdma::kMasterFrameLatencyUs <
                      static_cast<uint32_t>(link::profile::normal::kSlotMs) * 1000u,
                  "link.tdma.kMasterFrameLatencyUs 必须小于槽长(否则 windowDelayFromLatency 会退化成 0, "
                  "窗口开得太早, 会压在还在到达的主端帧上)");

    // ---- 链路预算 ----
    // 手册没给数值, 按吞吐推算: 持续灌数据时 UART 侧不能比空口快, 否则 E220 的 400 字节缓冲会溢出
    // 且是**静默**丢数据。经验界: 波特率 ≤ 1.25 × 空中速率。
    static_assert(hw::link::kBaud <= airRateBps(link::e220::kAirRate) * 5u / 4u,
                  "UART 波特率超过 1.25×空中速率: E220 缓冲会溢出且静默丢包。62.5k 时最高用 57600。");
    // 两端模块串口速率必须一致, 否则线上全是乱码
    static_assert(hw::link::kBaud == e220BaudBps(link::e220::kUartBaud),
                  "hw.link.kBaud 与 link.e220.kUartBaud 不一致: 一个配 MCU、一个配模块, 必须相等");

    // ---- 一帧必须能装进一个 E220 射频包 ----
    // REG1[7:6] 设 32B ⇒ 每个射频包 ≤32B。将来帧表变长时这里会先报错。
    static_assert(link::e220::kPacketLen == E220PacketLen::k32B,
                  "本版帧表最大 20B(DL_GPS), 依赖 AT+PACKET=32B 保证 一帧=一个射频包");

    // ---- 传感器/控制器尺寸 ----
    static_assert(hw::motor::kMap[0] < 4 && hw::motor::kMap[1] < 4 && hw::motor::kMap[2] < 4 &&
                      hw::motor::kMap[3] < 4,
                  "电机映射必须是 0..3 的物理通道");

    // ---- 日志布局契约(布局版本 2) ----
    // 上位机是按**固定字节偏移**解析 flash 里的 LogEntry 的, 所以结构体尺寸就是协议的一部分。
    // 这条断言把"结构体"和"配置里的布局版本号"钉在一起: 谁改了 LogEntry 而没有同步 +kLayoutVersion,
    // 编译期就报错 —— 否则表现是"日志能读出来但每个字段都错位", 比读不出来还难查。
    // 逐字段偏移表见 docs/_research/project_conventions.md 第 4.2 节。
    static_assert(sizeof(dlx::LogEntry) == 172u,
                  "LogEntry 尺寸变了: 改完字段必须同步 cfg::log::kLayoutVersion(并经日志管理模式 op=5 格式化), "
                  "然后更新本断言与 project_conventions.md 的偏移表");
    static_assert(log::kLayoutVersion == 2u, "LogEntry 当前是布局 v2; 换布局要 +1 并同步上面的断言");
    static_assert(log::kEntryHasFlowFields,
                  "布局 v2 含光流/测距字段; 要关掉必须先删 dlx_flash_manager_config.h 里那一段结构体成员");

} // namespace cfg
