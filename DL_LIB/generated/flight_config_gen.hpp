// ==========================================================================
//  本文件由 kmpFly 上位机的 Settings DSL 自动导出,请勿手改!
//  下次导出会整份覆盖;要改配置请改 Kotlin 侧 Settings{} 的声明,或在界面上改值后重新导出。
//
//  文件     : flight_config_gen.hpp
//  生成时间 : 2026-10-16 21:49:22
//  配置项   : 319 项(枚举 47 个,数组 1 个)
//  命名空间 : 48 个(根 cfg)
// ==========================================================================

#pragma once

#include <stdint.h>
#include "dlx_gpio_profile.h"
#include "dlx_spi_profile.hpp"
#include "dlx_timer_profile.hpp"
#include "dlx_dshot_profile.hpp"

namespace cfg {

    enum class AirRate : uint8_t {
        k2_4k = 0,
        k4_8k = 1,
        k9_6k = 2,
        k19_2k = 3,
        k38_4k = 4,
        k62_5k = 5,
    };

    enum class E220Baud : uint8_t {
        k1200 = 0,
        k2400 = 1,
        k4800 = 2,
        k9600 = 3,
        k19200 = 4,
        k38400 = 5,
        k57600 = 6,
        k115200 = 7,
    };

    enum class E220Parity : uint8_t {
        k8N1 = 0,
        k8O1 = 1,
        k8E1 = 2,
    };

    enum class E220PacketLen : uint8_t {
        k200B = 0,
        k128B = 1,
        k64B = 2,
        k32B = 3,
    };

    enum class E220TxPower : uint8_t {
        k22dBm = 0,
        k17dBm = 1,
        k14dBm = 2,
        k10dBm = 3,
    };

    enum class LinkProfile : uint8_t {
        NEAR = 0,
        NORMAL = 1,
        FAR = 2,
        SAFE = 3,
    };

    enum class RangeDistanceMode : uint8_t {
        Short = 1,
        Medium = 2,
        Long = 3,
    };

    enum class RangeIntPolarity : uint8_t {
        ActiveHigh = 0,
        ActiveLow = 1,
    };

    enum class LogFullPolicy : uint8_t {
        Halt = 0,
        OverwriteOldest = 1,
    };

    enum class GroundFallbackMode : uint8_t {
        Flight = 0,
        SerialDebug = 1,
        LogManage = 2,
    };

    // 版本 — 改协议或日志含义时 +1，便于事后分辨日志是哪一版。
    namespace version {

        // 改协议或日志含义时 +1，便于事后分辨日志是哪一版。
        constexpr uint16_t kFirmware = 2u;
        // 依据：配置重构 v3 的固件配置版本：旧 kConfig 里是 FLIGHT_CONFIG_VERSION = 1，本轮改了协议帧格式（E220 定长帧）、日志布局与传感器集合，语义变了一版，
        // 故为 2。

    } // namespace version

    // 编译期总开关 — 置 0 = 整块功能被编译期裁掉（不是运行期跳过）。生成器据此决定是否输出依赖项，配合 if constexpr / #if 裁代码。
    namespace feature {

        // 气压计（BMP388）总开关。false = **编译期裁剪**：不初始化 I2C1、不建气压采集线程、不喂气压观测，自检报告里记一条『配置关闭』(不是故障——主动关掉不等于坏)。
        constexpr bool kBaro = true;
        // 依据：2026-10-16 启用 BMP388，准备焊接并验证 I2C1 PB6/PB7、SDO 接地的实板；初始化、采集与 EKF 气压观测均随此开关启用。

        // VL53L1X 测距总开关。true = 启用; false = 不初始化 I2C2/XSHUT/EXTI3、不建测距线程；高度只剩气压/IMU 积分。
        constexpr bool kRange = true;
        // 依据：2026-10-03 第二次台架验证配置：接上除 GPS/气压计之外的所有外设 ⇒ 测距/光流/链路打开。链路是 **E220-400T22S**(USART3 PD8/PD9, 
        // M0=PD3 M1=PD4 AUX=PA2) —— NRF24L01 已按决策 9 退休, 不要再接 NRF 模块。

        // PMW3901 光流总开关。true = 启用; false = 不初始化 SPIC3/CS、不建光流线程；VEL_HOLD 因无观测而永远进不去(它本来就要求 flowValid)。
        constexpr bool kFlow = true;
        // 依据：2026-10-03 第二次台架验证配置：接上除 GPS/气压计之外的所有外设 ⇒ 测距/光流/链路打开。链路是 **E220-400T22S**(USART3 PD8/PD9, 
        // M0=PD3 M1=PD4 AUX=PA2) —— NRF24L01 已按决策 9 退休, 不要再接 NRF 模块。

        // NEO-7M 总开关。false = 不初始化 USART2/DMA、不建 GNSS 线程；位置/水平速度观测都没有。
        constexpr bool kGnss = false;
        // 依据：2026-10-03 首次烧录验证配置：本次只接 LED / 跳线帽 / IMU，这四个外设编译期裁掉；ADC 保留（PA0/PA1 是片内引脚，不需要外接任何东西，而它是『必需件』——关掉会被判成禁止解锁）
        // 。这些开关现在是**真的**：管住『不初始化 + 不建线程』；对象本身仍在(驱动构造函数不碰寄存器)，所以省的是外设与线程，不是 RAM。

        // 电压/电流采样总开关。false 时没有低压保护。
        constexpr bool kAdc = true;
        // 依据：新增。低压保护需要真实电压，关掉等于放弃 J 组的分级保护。

        // 无线链路总开关。true = 启用; false = 不初始化链路 SPI、不建链路线程、不建链路软定时器。**后果必须清楚：收不到任何指令 ⇒ 解不了锁**，这份固件只能当地面观察用(有线 
        // USART1 独立可用)。
        constexpr bool kLink = true;
        // 依据：2026-10-03 第二次台架验证配置：接上除 GPS/气压计之外的所有外设 ⇒ 测距/光流/链路打开。链路是 **E220-400T22S**(USART3 PD8/PD9, 
        // M0=PD3 M1=PD4 AUX=PA2) —— NRF24L01 已按决策 9 退休, 不要再接 NRF 模块。

        // false = 日志走串口调试流（台架），不写 flash。
        constexpr bool kLogToFlash = true;
        // 依据：沿用现有"串口调试模式"语义，改成编译期开关；飞行模式必须为 true。

        // 详细启动/自检日志开关。true = 串口打全部 [Boot]/[Dev]/[SPI]/[Flash] 诊断行；false = **整段编译期剔除**（不是运行期判断），真 release 
        // 用 false。v2.1 起这是**唯一**的一道闸：判断就写在 `logLine()` 第一行，调用点一律直接调 logLine()，不再有 DLX_BOOT_LOG 这类包装宏。
        constexpr bool kVerboseBootLog = true;
        // 依据：2026-10-02 加：release.cpp 里那几百行自检打印是**排查硬件的主力工具**（IMU 片选、气压计重试次数、flash JEDEC ID、DShot 定时器寄存器…）
        // ，但它们只在台架有用；真 release 里留着既占 flash 又占串口时间。做成**编译期**开关而不是运行期 if 的理由：① 优化器能把整段删干净，运行期 if 做不到；② 但代码仍然参与编译，
        // 所以格式串写错、变量改名这类错误在 release 构建里**照样会被发现**（用 `#if` 宏剔除就会把这类错误一起藏起来，直到哪天打开开关才炸）。⚠ 闸在函数体内 ⇒ 关掉时**实参仍会被求值**：
        // 纯日志调用无所谓，但『为了打印才做一次硬件读』的调用点（bmi.readChipIds / logFlashProbe 那种）必须把硬件访问留在自己的判断后面。值由本声明导出（不再经 config/flight_config.json）
        // 。

    } // namespace feature

    // 时序与调度 — 各线程唤醒周期。控制环 500Hz(2ms) 是基准节拍：链路槽长必须是它的整数倍，控制周期必须与 kRateHz 互为倒数（static_assert）。
    namespace time {

        // 控制环节拍
        namespace ctrl {

            // [Hz] 控制环频率。必须与 kTickMs 互为倒数（生成器加 static_assert）。
            constexpr float kRateHz = 500.0f;
            // 依据：沿用现有 kConfig.loopHz = 500.0f，实测最坏 slot 负载 ~73%，本轮把总线操作全部移出主环。

            // [ms] 控制线程唤醒周期。
            constexpr uint32_t kTickMs = 2u;
            // 依据：500Hz 硬定时器信号量，沿用现有 ctrlTickMs = 2。

        } // namespace ctrl

        // 链路线槽
        namespace link {

            // 链路线程的轮询周期 [ms] — 链路线程(E220 收字节 + 时分相位 + 下行窗口)的唤醒周期。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kTickMs = 8u;
            // 依据：§3.5: 这是**轮询周期**而不是槽长 —— 槽长在 link.profile.*.kSlotMs(改档不改这个)。取 **8ms**(2026-10-06 由 10ms 改回整数关系)
            // , 三条约束一起满足: ① 是 time.ctrl.kTickMs(2ms) 的整数倍; ② **整除槽长**(24/8=3) —— 这条不是审美: 主端帧的到达时刻只能在轮询时刻被观察到, 
            // 所以观测间隔被量化到轮询网格上; 槽长 24ms 配 10ms 轮询会稳定产生 20/30ms 交替的观测间隔, 而 30ms 正好等于「一个槽 + 容差」(24 + 24/4) ⇒ 差值卡在判据边界上, 
            // 任何抖动都会被 onSlotGrid() 判成 OffGrid(不改相位 ⇒ 窗口不开、连续上行计数清零)。整除时观测间隔恒为 24ms、差值为 0; ③ 一个槽里至少醒 3 次, 收字节与窗口服务都不会被拖过一个槽。

        } // namespace link

        // 气压/ADC 采集
        namespace io {

            // [ms] 气压计 + ADC 采集线程周期（50Hz）。
            constexpr uint32_t kTickMs = 20u;
            // 依据：新增。把 100kHz I²C 读气压（~1ms）从控制环里挪出去，主环最坏 slot 才能降下来。

        } // namespace io

        // 光流采样
        namespace flow {

            // [ms] PMW3901 读取周期（100Hz）。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kPeriodMs = 10u;
            // 依据：新增。光流采样率要高于速度环带宽（kLpfTauS=0.08s ⇒ 2Hz），10ms 足够且 SPI 只占 ~1.31MHz/8bit。

        } // namespace flow

        // 测距兜底轮询
        namespace range {

            // [ms] 测距兜底轮询周期；正常由 EXTI3（PA3 GPIO1 上升沿）唤醒。
            // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kFallbackMs = 100u;
            // 依据：新增。中断丢了也不能停更，兜底 100ms ⇒ 最坏 10Hz 高度更新。

        } // namespace range

        // GNSS 解析
        namespace gnss {

            // [ms] GNSS 解析线程兜底周期。
            // 仅当 cfg::feature::kGnss 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kPeriodMs = 100u;
            // 依据：新增。模块 5Hz 输出，100ms 兜底；实际由 USART2 IDLE + DMA 唤醒。

        } // namespace gnss

        // 日志周期
        namespace log {

            // [ms] 日志线程唤醒周期。
            constexpr uint32_t kTickMs = 5u;
            // 依据：沿用现有 logTickMs = 5。

            // [ms] 飞行模式日志周期（50Hz）。
            constexpr uint32_t kFlightPeriodMs = 20u;
            // 依据：沿用现有 flightLogPeriodMs = 20。

            // [ms] 串口调试流周期（100Hz）。
            constexpr uint32_t kDebugPeriodMs = 10u;
            // 依据：沿用现有 debugLogPeriodMs = 10。

            // [ms] PerfMonitor 上报周期。
            constexpr uint32_t kPerfReportMs = 200u;
            // 依据：沿用现有 perfReportMs = 200。

            // [ms] 调试模式运行状态文本行周期，0 = 关。
            constexpr uint32_t kDebugStatMs = 1000u;
            // 依据：沿用现有 debugStatMs = 1000；终端里盯队列溢出/堆/遥控有无包，比看图快。

        } // namespace log

        // 性能统计
        namespace perf {

            // [圈] 起跑热身圈数，前 N 圈不计入负载统计。
            constexpr uint32_t kWarmupLoops = 5u;
            // 依据：沿用现有 perfWarmupLoops = 5（含 FIFO 积压与 EKF 首次整定）。

            // [ms] 栈水位线打印周期，0 = 关。
            constexpr uint32_t kStackWatermarkMs = 10000u;
            // 依据：现有是 bool logStackWatermark；改成周期量：新增 5 个静态栈线程后要能量栈预算，0 表示关。

        } // namespace perf

    } // namespace time

    namespace hw {

        // 硬件 / 指示灯与跳线 — C1：心跳/flash 指示灯与地面模式跳线。PF3 同时用作硬件故障闪码。
        namespace led {

            // 心跳 / 地面模式等待闪烁指示灯，高电平点亮。
            constexpr dlx::GPIOProfile kRun = dlx::GPIOProfile::F3;
            // 依据：沿用现有 kConfig.hw.ledRun = GPIOProfile::F3；PF3 同时用作硬件故障闪码（§8.2 B0）。

            // flash 擦除进度指示灯，低电平点亮（每擦完 64KB 翻转一次）。
            constexpr dlx::GPIOProfile kFlash = dlx::GPIOProfile::F2;
            // 依据：沿用现有 kConfig.hw.ledFlash = GPIOProfile::F2。

        } // namespace led

        // 地面模式跳线
        namespace jumper {

            // 地面模式跳线 A。
            constexpr dlx::GPIOProfile kA = dlx::GPIOProfile::B8;
            // 依据：沿用现有 kConfig.hw.jumpA = GPIOProfile::B8。

            // 地面模式跳线 B。
            constexpr dlx::GPIOProfile kB = dlx::GPIOProfile::B9;
            // 依据：沿用现有 kConfig.hw.jumpB = GPIOProfile::B9。

        } // namespace jumper

        // 硬件 / IMU（BMI088） — C2：SPI1 软 NSS。CS 已按现有代码确认为 PB12/PB13 —— release.cpp:954 那句 “CS: 加速度计=PC4 陀螺=PC5” 是错的日志，B0 一并改掉。
        namespace imu {

            // BMI088 加速度计片选（软件 NSS）。
            constexpr dlx::GPIOProfile kAccelCs = dlx::GPIOProfile::BC;
            // 依据：2026-10-02 用户确认：以现有代码为准 PB12/PB13（kConfig.hw.imuAccelCs = GPIOProfile::BC）；release.cpp:954 
            // 那句 "CS: 加速度计=PC4 陀螺=PC5" 是错的日志，B0 一并改掉。不再是待确认项。

            // BMI088 陀螺仪片选（软件 NSS）。
            constexpr dlx::GPIOProfile kGyroCs = dlx::GPIOProfile::BD;
            // 依据：同上：与 kConfig.hw.imuGyroCs = GPIOProfile::BD 一致，2026-10-02 确认。

            // SPI1 模式：全双工主机 8bit，CPOL=1 / CPHA=2E，软 NSS，APB2/16 = 5.25MHz。
            constexpr dlx::SPIModeProfile kSpi = dlx::SPIModeProfile::FD_M_8B_CH_2E_NS_BR16_MSB;
            // 依据：沿用现有 kConfig.hw.imuSpi；SPI1 挂 APB2(84MHz)，BR=16 ⇒ 5.25MHz，BMI088 上限 10MHz 以内。

            // [Hz] BMI088 陀螺 ODR，必须与 dlx_bmi088_config.h 一致。
            constexpr float kRateHz = 2000.0f;
            // 依据：沿用现有 kConfig.imuRateHz = 2000.0f；推导量 kImuDt = 1/2000。

            // [deg] IMU 相对机体绕 z 的偏转（0/90/180/270），remapSensorFrame() 用。
            constexpr int32_t kYawDeg = 0;
            // 依据：沿用现有 kConfig.imuYawDeg = 0。判定法：手动抬机头应让 euler pitch 变、左右滚应让 euler roll 变；反之换成 90/270。

        } // namespace imu

        // 硬件 / 存储（W25Q128） — C3：SPI2 + PD1 片选。日志与参数都存这里。
        namespace flash {

            // W25Q128 片选。
            constexpr dlx::GPIOProfile kCs = dlx::GPIOProfile::D1;
            // 依据：沿用现有 kConfig.hw.flashCs = GPIOProfile::D1（PD1）。

            // SPI2 模式：CPOL=0 / CPHA=1E，软 NSS，APB1/8 = 5.25MHz。
            constexpr dlx::SPIModeProfile kSpi = dlx::SPIModeProfile::FD_M_8B_CL_1E_NS_BR8_MSB;
            // 依据：沿用现有 kConfig.hw.flashSpi；SPI2 挂 APB1(42MHz)，BR=8 ⇒ 5.25MHz。注意同一 BR 码在 APB1/APB2 上时钟不同（见 diag.kVerifySpiPrescaler）
            // 。

        } // namespace flash

        // 硬件 / 气压计（BMP388） — I2C1 PB6/PB7。SDO 直接焊到 GND，固定地址 0x76；PD2 已分配给 VL53L1X XSHUT，不连接 BMP388。
        namespace baro {

            // I2C1 时钟线。
            // 仅当 cfg::feature::kBaro 为 true 时存在(编译期整块裁掉)。
            constexpr dlx::GPIOProfile kScl = dlx::GPIOProfile::B6;
            // 依据：沿用现有接线（PB6，AF4 I2C1）；BMP388 替换 BME280 后仍是 I2C1。

            // I2C1 数据线。
            // 仅当 cfg::feature::kBaro 为 true 时存在(编译期整块裁掉)。
            constexpr dlx::GPIOProfile kSda = dlx::GPIOProfile::B7;
            // 依据：沿用现有接线（PB7，AF4 I2C1）。

            // BMP388 7 位 I2C 地址：SDO 接 GND = 0x76，接 VDDIO = 0x77。
            constexpr uint8_t kAddr = 0x76;
            // 依据：本板 BMP388 的 SDO 直接焊到 GND，固定 7 位 I2C 地址 0x76；PD2 只用于 VL53L1X XSHUT。

            // [Hz] I2C1 速率。100000 是稳妥值，400000 是调试模式实测稳定值（前提是 SDO 已钉死）。
            // 仅当 cfg::feature::kBaro 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kSpeed = 400000u;
            // 依据：沿用现有 kConfig.hw.bmeSpeed = 400000 —— 2026-09-13 那次"飞行模式必失败"其实是 SDO 悬空，不是速率的锅；本轮 I/O 移出控制环后，
            // 速率只影响 io 线程耗时。

        } // namespace baro

        // 硬件 / 测距（VL53L1X） — C5/E2：I2C2 独占，SCL=PF1 / SDA=PF0（注意与常见写法相反），GPIO1=PA3 走 EXTI3 上升沿。
        namespace range_ {

            // I2C2 时钟线。注意本板是 SCL=PF1 / SDA=PF0，与常见写法相反。
            constexpr dlx::GPIOProfile kScl = dlx::GPIOProfile::F1;
            // 依据：§2.1 接线表明确标注顺序相反；需新增工厂 IIC2_SF1_DF0()，不要照抄 IIC2_SDA_PF0_SCL_PF1 那种命名直觉。

            // I2C2 数据线（PF0）。
            constexpr dlx::GPIOProfile kSda = dlx::GPIOProfile::F0;
            // 依据：§2.1 接线表。

            // VL53L1X 的 XSHUT（低有效，需 10k 上拉）。**已定案用 PD2**：BMP388 的 SDO 直接焊到 GND，PD2 这个网络不再连到传感器，因此可以让给 XSHUT。
            // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
            constexpr dlx::GPIOProfile kXshut = dlx::GPIOProfile::D2;
            // 依据：2026-10-02 确认：用户把 BMP388 的 SDO 焊 GND ⇒ 地址固定 0x76，PD2 释放。注意 XSHUT 上电必须先为低（硬件待机），MCU 初始化时再拉高启动，
            // 所以初始化顺序不能颠倒。

            // VL53L1X GPIO1（数据就绪），默认高有效 ⇒ EXTI 配上升沿。
            // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
            constexpr dlx::GPIOProfile kInt = dlx::GPIOProfile::A3;
            // 依据：§0.1 事实 8：GPIO1 默认高有效，与常见描述相反；PA3 原本是 kConfig.hw.imuSpiNss，改软 NSS 后释放（§2.2 冲突②）。EXTI3 已存在（§2.3）
            // 。

            // VL53L1X 7 位 I2C 地址（出厂默认）。
            constexpr uint8_t kAddr = 0x29;
            // 依据：§4.2：VL53L1X 独占 I2C2，总线上没有第二个同地址器件，用默认 0x29 即可，不必用 XSHUT 改址。

            // [Hz] I2C2 速率（器件上限 400kHz）。
            // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kSpeed = 400000u;
            // 依据：§4.2；VL53L1X 支持 Fast Mode 400kHz，独占总线可以跑满。

        } // namespace range_

        // 硬件 / 光流（PMW3901） — C6/E3：SPI3 软件 NSS、模式 3、1.31MHz。板卡是成品光流模块：VDD 1.8V 由模块自带 LDO 供给，VDDIO 直接接 3.3V，因此 MCU 侧既不需要额外供电也不需要电平转换（裸 COB 才必须自备 1.8V）。
        namespace flow {

            // PMW3901 片选（软件 NSS / 普通 GPIO）。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr dlx::GPIOProfile kCs = dlx::GPIOProfile::A4;
            // 依据：§2.1：NRF24L01 卸载后 SPI3 独占；PA4 原为 nrfSpiNss，本轮释放给光流。软件 NSS 是必须的：地址字节与数据之间要保持 CS 低且中间有 ≥35µs 
            // 真间隙，用硬件 NSS 做不到两次传输。

            // SPI3 模式：全双工主机 8bit，CPOL=1 / CPHA=2E（模式 3），软 NSS，APB1/32 = 1.31MHz。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr dlx::SPIModeProfile kSpi = dlx::SPIModeProfile::FD_M_8B_CH_2E_NS_BR32_MSB;
            // 依据：§4.3：模式 3 是 PMW3901 的要求；SPI3 挂 APB1(42MHz)，BR=32 ⇒ 1.31MHz ≤ 2MHz 规格上限。BR=16 会到 2.625MHz，超规格。

            // SPI3 时钟线 PC10。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr dlx::GPIOProfile kSck = dlx::GPIOProfile::CA;
            // 依据：§2.1；NRF 卸载后保留原 SPI3 引脚组。

            // SPI3 主机输入 PC11。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr dlx::GPIOProfile kMiso = dlx::GPIOProfile::CB;
            // 依据：§2.1。

            // SPI3 主机输出 PC12。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr dlx::GPIOProfile kMosi = dlx::GPIOProfile::CC;
            // 依据：§2.1。

            // [us] 地址字节 → 数据字节之间的真间隙（规格 ≥35µs）。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kAddrToDataUs = 50u;
            // 依据：§0.1 事实 5：PMW3901 的 bit7=1 是写、bit7=0 是读（与 PMW3360/ADNS-9800 相反），且地址与数据之间要 ≥35µs 真间隙 —— 这是"读回全 
            // 0"的头号原因；实现上用两次传输保持 CS 低 + 延时。

            // [us] 写操作末字节 → CS 拉高的保持时间（规格 ≥35µs）。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kWriteTailUs = 50u;
            // 依据：同上，手册 tSWW/tSCS 类要求；实测低于 35µs 会出现寄存器写不进（初始化序列静默失败）。

            // [ms] 写 0x3A←0x5A（复位）后的等待时间（规格 tMOT-RST = 50ms）。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kResetWaitMs = 50u;
            // 依据：§4.3 初始化序列；不等会让后续寄存器写入全部落在复位窗口里，表现为"初始化后 motion 恒 0"。

            // [ms] 初始化序列中段强制暂停。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kInitPauseMs = 10u;
            // 依据：§4.3 初始化序列里有一步必须等待（性能/自检相关），10ms 是留余量的统一停顿。

        } // namespace flow

        // 硬件 / GNSS（NEO-7M） — C7：USART2 PD5/PD6，DMA+IDLE 接收。位置/高度只回传，只有水平速度进 EKF。
        namespace gnss {

            // NEO-7M 所在 USART2 的 MCU 发送脚。
            constexpr dlx::GPIOProfile kTx = dlx::GPIOProfile::D5;
            // 依据：§2.1 接线表；工厂 USART2_TD5_RD6() 已存在。

            // NEO-7M 所在 USART2 的 MCU 接收脚（模块 TX → MCU RX）。
            constexpr dlx::GPIOProfile kRx = dlx::GPIOProfile::D6;
            // 依据：§2.1；模块 3.3V 电平，无需电平转换。

            // [bps] USART2 波特率，模块出厂默认。要 5~10Hz 需先用 UBX CFG-RATE/CFG-PRT 改模块。
            // 仅当 cfg::feature::kGnss 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kBaud = 9600u;
            // 依据：NEO-7M 出厂 9600 8N1；解析线程只吃 NMEA，9600 足够 1Hz，提高速率前先改模块配置。

            // [B] DMA 循环接收缓冲大小。
            // 仅当 cfg::feature::kGnss 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kDmaRxBufBytes = 512u;
            // 依据：新增。NMEA 一句最长约 82 字节，512B 可容纳多句突发；DMA1_Stream5_Ch4（§2.3）。

        } // namespace gnss

        // 无线（NRF24L01，待退役） — ⚠️ 本组只在 B1~B6 期间存在：release.cpp 里 NRF 驱动仍在编译并占用 SPI3/PC7/PC8。B7 切到 E220 后整组删除、SPI3 让给 PMW3901。保留它只是为了让配置重构本身保持行为不变。
        namespace nrf {

            // NRF24L01 CE。B7 删除（PC7 腾给 VL53L1X XSHUT 备选）
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr dlx::GPIOProfile kCe = dlx::GPIOProfile::C7;
            // 依据：现状 release.cpp:1015 使用。

            // NRF24L01 CSN。B7 删除
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr dlx::GPIOProfile kCsn = dlx::GPIOProfile::C8;
            // 依据：同上。

            // NRF 所在 SPI3 模式（APB1/42MHz ÷32 = 1.3125MHz）。B7 删除
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr dlx::SPIModeProfile kSpi = dlx::SPIModeProfile::FD_M_8B_CL_1E_NS_BR32_MSB;
            // 依据：现状 kConfig.hw.nrfSpi。

            // SPI3 的 NSS 脚（软 NSS，仅被驱动为高）。B7 删除，PA4 让给 PMW3901 的 CS
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr dlx::GPIOProfile kSpiNss = dlx::GPIOProfile::A4;
            // 依据：现状 kConfig.hw.nrfSpiNss。

        } // namespace nrf

        // 硬件 / 无线链路（E220） — C8：USART3 + M0/M1/AUX。RESET 未引出到 MCU，故本版没有 kReset 配置项：M0/M1 必须外部 10k 下拉，AUX 只做输入（10k 上拉，绝不驱动）。
        namespace link {

            // E220 所在 USART3 的 MCU 发送脚。
            constexpr dlx::GPIOProfile kTx = dlx::GPIOProfile::D8;
            // 依据：§2.1；需 USART3_TD8_RD9 的 DMA+IDLE 接收（工厂已存在）。

            // E220 所在 USART3 的 MCU 接收脚。
            constexpr dlx::GPIOProfile kRx = dlx::GPIOProfile::D9;
            // 依据：§2.1。

            // E220 模式脚 0。透传模式 M0=0。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr dlx::GPIOProfile kM0 = dlx::GPIOProfile::D3;
            // 依据：§3.1：工作模式 M0=0/M1=0 常驻透传，运行期绝不切模式。必须外部 10k 下拉到 GND（模块对 M0/M1 只有极弱上拉，悬空 ⇒ 停在模式 3 不发射）。

            // E220 模式脚 1。透传模式 M1=0。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr dlx::GPIOProfile kM1 = dlx::GPIOProfile::D4;
            // 依据：同上；MCU 复位期间 GPIO 高阻，只有外部下拉能保证模块落在透传模式。

            // E220 忙/数据就绪输入（只输入，10k 上拉）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr dlx::GPIOProfile kAux = dlx::GPIOProfile::A2;
            // 依据：§2.2 冲突④c：AUX 绝不能被驱动、绝不能被下拉 —— 上电 5ms 窗口内被拉低 >1s 会强制进 IAP 升级模式（要通电 60s 才出得来）；RESET 已裁掉，没有软恢复手段。
            // EXTI2 已存在。

            // [bps] USART3 传输波特率，必须与 link.e220.kUartBaud 一致（生成器加 static_assert）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kBaud = 57600u;
            // 依据：硬约束 波特率 ≤ 1.25 × 空中速率：62.5k × 1.25 = 78125 ⇒ 最高档是 57600，不能用 115200（§0.1 事实 3）。

            // [次] 上电配置流程的重试次数。E220 的 RESET 未引出到 MCU，配置流程是唯一的软恢复手段，所以失败要重试整段而不是放弃。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kCfgRetry = 3u;
            // 依据：2026-10-02 确认：E220 的 RESET 脚被裁掉，模块一旦卡死只能整机断电，因此把配置做成可重入并在 INIT_HIGH 重试。

            // [bps] 配置模式(M0=M1=1)下的串口波特率。E220 的 AT/HEX 配置只在配置模式下被接受，而配置模式强制 9600 8N1，与传输波特率不同 —— 所以上电要做一次双波特率切换。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kCfgBaud = 9600u;
            // 依据：手册 AT 指令章节：配置模式串口固定 9600 8N1；AT+UART 设的传输波特率要切回透传模式后才生效。

            // [B] DMA 循环接收缓冲大小。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kDmaRxBufBytes = 256u;
            // 依据：新增。定长帧最大 20B，256B 够容纳多个粘连帧并留重同步余量。

        } // namespace link

        // 硬件 / 电机（DShot） — C9：TIM1 四路 DShot300（PE9/PE11/PE13/PE14）。
        namespace motor {

            // TIM1 向上计数、不分频，输出 PE9/PE11/PE13/PE14 四路 DShot。
            constexpr dlx::AdvancedTimerProfile kTimer = dlx::AdvancedTimerProfile::TIM1_Up_DIV1;
            // 依据：沿用现有 kConfig.hw.motorTimer；TIM1 挂 APB2，DShot300 位宽 2.5µs 由 ARR 定，预分频必须为 0。

            // DShot 速率档：300kbps（位宽 2.5µs）。
            constexpr dlx::DshotProfile kDshot = dlx::DshotProfile::Dshot300;
            // 依据：沿用现有 kConfig.hw.dshotMode；DShot300 在 168MHz 下有足够分辨率（600 需要更严的时序与更好的线材）。

            // 逻辑电机(0..3) → 物理 DShot 通道。
            constexpr uint8_t kMap[4] = {0, 2, 3, 1};
            // 依据：沿用现有 kConfig.motorMap = {0,2,3,1}；逻辑布局见 flight_control_params.hpp 头部示意（M0 左前 / M1 右前 / M2 
            // 右后 / M3 左后）。

            // [count] 归一化油门(0~1) → DShot 计数的比例：count = throttle*unit + offset。
            constexpr float kUnit = 1900.0f;
            // 依据：沿用现有 kConfig.dshotUnit = 1900.0f；上限 = 1900 + 50 = 1950，落在 DShot 0..2047 内且留出命令余量。

            // [count] DShot 计数偏置（0 油门对应的计数，即怠速）。
            constexpr float kOffset = 50.0f;
            // 依据：沿用现有 kConfig.dshotOffset = 50.0f。

            // [count] 饱和判据下限：低于此值认为输出触底。
            constexpr uint16_t kSatLow = 55u;
            // 依据：由 kOffset=50 推出（怠速 50 + 余量）；需台架按实际怠速复核定稿，故 evidence 只是推导不是实测。

            // [count] 饱和判据上限：高于此值认为输出顶满。
            constexpr uint16_t kSatHigh = 1945u;
            // 依据：由 kUnit/kOffset 推出（1900+50−5=1945）；触顶即失去姿态控制权限，置 kLogFlagMotorSat。

        } // namespace motor

        // 硬件 / ADC（电压电流） — C10：ADC1 扫描 IN0(PA0)/IN1(PA1) + DMA2_Stream0 循环，软件触发（io 线程 50Hz）。采样值由 power.* 换算成物理量。
        namespace adc {

            // [ch] 电池电压采样通道 = ADC1_IN0 = PA0。
            // 仅当 cfg::feature::kAdc 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kVoltageCh = 0u;
            // 依据：§2.1/§4.5：PA0 空闲且是模拟脚，外部 8k 上拉 / 1k 下拉分压。

            // [ch] 电流采样通道 = ADC1_IN1 = PA1。
            // 仅当 cfg::feature::kAdc 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kCurrentCh = 1u;
            // 依据：§2.1/§4.5：PA1 空闲，电流计输出直连（假定已在 0~3.3V 内）。

            // [次] 每次上报前的平均样本数。
            // 仅当 cfg::feature::kAdc 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kSampleCount = 8u;
            // 依据：新增。ADC1 扫描由 DMA 循环写入，8 次平均把量化噪声降到 1/√8；io 线程 50Hz 有充足采样机会。

        } // namespace adc

        // 硬件 / 调试串口与缓冲 — C11：USART1 PA9/PA10 2Mbaud，以及各段缓冲大小。串口上只能有 DLX 帧（文本走 SimpleLog）。
        namespace console {

            // [bps] 调试/地面站串口 USART1（PA9/PA10）波特率。
            constexpr uint32_t kBaud = 2000000u;
            // 依据：沿用现有 kConfig.serialBaud = 2000000；日志流 100Hz 帧 + 会话回传都走这一路。

        } // namespace console

        // 缓冲大小
        namespace buf {

            // [B] 串口接收环形缓冲大小。
            constexpr uint32_t kRx = 256u;
            // 依据：沿用现有 kConfig.hw.rxBufBytes = 256。

            // [B] 串口发送缓冲（绑 DMA，也是日志流/会话回传缓冲）。
            constexpr uint32_t kTx = 2048u;
            // 依据：沿用现有 kConfig.hw.txBufBytes = 2048。

            // [B] DLX 解帧临时区，必须不小于最大负载。
            constexpr uint32_t kFrame = 256u;
            // 依据：沿用现有 kConfig.hw.frameBufBytes = 256；注释里写的 136 已过期，现最大负载 144B，256 仍有余量。

            // [B] 文本日志行缓冲（SimpleLog 一行文本的上限）。
            constexpr uint32_t kTextLine = 160u;
            // 依据：沿用现有 kConfig.hw.textLineBytes = 160；logLine() 按此截断并补齐偶数长度。

        } // namespace buf

    } // namespace hw

    namespace link {

        // 链路 / E220 寄存器 — D1：两端必须一致，上电逐条回读校验。RESET 缺失 ⇒ 配置流程必须可重入（整段重试 3 次），运行期绝不切模式。
        namespace e220 {

            // 射频信道：f = 410.125 + CH×1MHz ⇒ CH=23(0x17) = 433.125MHz。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kChannel = 23u;
            // 依据：**这里原来写的是 17(427.125MHz)** —— 值与 doc 里的公式对不上(410.125+17=427.125，不是 433.125)。2026-10-03 由 E220 
            // 驱动实现者按手册算术发现并上报。裁决取 **23 = 模块出厂默认(手册 §6.4 REG2=0x17)**：这样遥控器侧一块新模块**不用写任何寄存器**就能对上，飞控侧也只需要回读校验而不必写 
            // REG2；433.125MHz 同时是 CN/EU 的 433 ISM 中心。相邻干扰大时可换信道，但**两端必须同值**，改一处没用。

            // 空中速率档：2.4/4.8/9.6/19.2/38.4/62.5 kbps。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr AirRate kAirRate = AirRate::k62_5k;
            // 依据：§3.1 AT+RATE=7 ⇒ 62.5kbps：8B 包空口 2.90ms、12B 3.22ms、20B 4.50ms；80/20Hz 时分下占空比 29.6%，有余量。

            // 模块串口速率，必须与 hw.link.kBaud 一致（生成器加 static_assert）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr E220Baud kUartBaud = E220Baud::k57600;
            // 依据：§0.1 事实 3：硬约束 波特率 ≤ 1.25×空中速率 ⇒ 62.5k 时最高 57600。注意 AT 指令只在配置模式（M0=M1=1）被接受，而配置模式串口被强制成 9600 
            // —— 一次上电要换两次波特率（§3.1.1）。

            // 校验与数据位：8N1 / 8O1 / 8E1。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr E220Parity kParity = E220Parity::k8N1;
            // 依据：§3.1；8N1 与 USART3 的 USARTModeProfile 必须一致，否则每帧都像被噪声打乱。

            // 子包长度设置：200/128/64/32 字节。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr E220PacketLen kPacketLen = E220PacketLen::k32B;
            // 依据：§3.1 AT+PACKET=3 → 32B：我们最大帧 20B，设 32B 保证"一帧 = 一个射频包"，不会被 200B 分包逻辑从中间切开（切开后 URXT 会把一帧拆成两个空口包）
            // 。

            // ★ 必须 true：打开后 AUX 才表示"射频发射忙"（出厂 0 时不表示）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr bool kUaux = true;
            // 依据：§0.1 事实 2：uaux=0 时 AUX 在帧间隔超时就拉高、早于空口发完，"AUX 高才写入"的节流办法直接失效。写入需要进配置模式（§3.1.1 第 ③ 步），且必须回读校验。

            // [字节时间] 收端分包间隔，@57600 ≈ 521µs。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kUrxt = 3u;
            // 依据：§0.1 事实 4：主机中途停顿超过它，模块就把一帧拆成两个射频包 ⇒ 每帧必须一次性 DMA 连续写完，不能分片写。

            // 发射功率：22/17/14/10 dBm。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr E220TxPower kTxPower = E220TxPower::k22dBm;
            // 依据：§3.1 AT+POWER=0 ⇒ 22dBm（模块型号标称 -T22S）；近场调试可降到 17/14 减少邻道干扰与功耗。

            // 透传模式地址（在包内、不参与射频）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint16_t kAddr = 0u;
            // 依据：§3.1 AT+ADDR=0，两端相同；注意必须用透传模式（AT+TRANS=0）——固定点模式会吃掉前 3 字节（地址+信道），帧头就没了。

            // 发射前监听（Listen Before Talk）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr bool kLbt = false;
            // 依据：§3.1：LBT 最坏要等 2 秒才强发，对手动飞行不可接受。

            // 是否给每个收到的包追加 1 字节 RSSI。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr bool kDrssi = false;
            // 依据：§3.1：打开会破坏"定长帧"解析（每包长度都变）；链路质量改用 SEQ 丢包率统计，需要 RSSI 时用 HEX C1 按需读寄存器。

            // 收发密钥，两端必须一致。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint16_t kKey = 0u;
            // 依据：§3.1 AT+KEY=0；不一致的表现和"距离不够"一模一样（收不到就是收不到），所以必须回读校验。

            // [ms] 上电后等 AUX 变高再等这段时间才发第一个字节。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kBootWaitMs = 20u;
            // 依据：§3.1：上电 16ms 就绪、AT+MODE 后 15ms，取 20ms 留余量；RESET 未引出，上电时序只能靠等待兜住。

            // 配置后逐条回读校验（AT+X=? 比对）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr bool kVerifyRegs = true;
            // 依据：§3.1.1：NRF24L01 那次"只写不读照样返回 true"的教训；校验失败就整段重试（最多 3 次），仍失败则 health.link=0，退化为仅有线可用。

        } // namespace e220

        // 链路 / TDMA 时分 — D2：遥控器为时分主端，N 个上行槽 1 个下行槽；槽长 10ms ⇒ 80Hz 上行 / 20Hz 下行，占空比 29.6%。
        namespace tdma {

            // [槽] N：N 个上行槽 1 个下行槽，4 上 1 下 ⇒ 80Hz 上行 / 20Hz 下行。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kUplinkPerDownlink = 4u;
            // 依据：§3.5：槽长 10ms，N=4 ⇒ 上行 80Hz、下行 20Hz，占空比 29.6%。待拍板项 #7（是否按草图的"约 13 上 1 下"）——默认取 4，改这一个值即可。

            // [ms] 最后一帧上行结束 → 本机发车的最小间隔。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kGuardMs = 3u;
            // 依据：§3.5：覆盖模块收发切换（AUX 忙 9~11ms 是模式切换，不是这个）与空口传播；太小会撞上别人的尾包。

            // 主端一帧的时延估计 [us] — 主端一帧从开始发到本机收完的时延估计(空口 + 串口)。窗口延时 = 槽长 - 本值。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kMasterFrameLatencyUs = 16500u;
            // 依据：**上板实测(2026-10-06, 两次)**。遥控器 'f' 的单帧空口探针(暂停链路→排空→发一帧→量 AUX 低电平时长, 每档 3 次取最小): 第一次 8B=10199us, 
            // 第二次(链路按 40Hz 正常跑着时)8B=**16518us**、12B=18180、20B=21199、32B=25368。**取第二次**: 它与同一份日志里的在役数字互相印证 —— 
            // 39 帧/s × 16.5ms = 64% ≈ 实测 AUX 低 69%(用第一次的 10.2ms 只能算出 40%, 对不上)。反推 8B 帧的**端到端**(空口+串口)≈16.5ms。
            // ⚠ 时延估**小**是安全侧(窗口 = 槽长 − 时延 ⇒ 估小则窗口开得晚), 所以取实测下限 16500 而不再上调。上板后仍可用遥控器 'f' 的“槽3→下行”实测反推复核。

            // [槽] 连续丢多少个槽就静默、只收不发。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kResyncSilentSlots = 2u;
            // 依据：§3.5：防止"时隙相位丢了还照发"把整网拖垮；静默后靠收帧重新对齐相位。

            // [槽] 连续收到多少个有效上行才恢复发送。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kResyncGoodSlots = 2u;
            // 依据：同上：恢复门槛与静默门槛对称，避免抖动式反复开关。

            // 发车前是否再查一次 AUX 为高。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr bool kAuxGate = true;
            // 依据：§3.5：uaux=1 之后 AUX 表示射频忙，可以当节流用；实测不可靠就关掉，退化为纯 TDMA+guard（最坏晚发一个槽，不会丢相位）。

            // [槽] 下行槽轮转：每 1 槽发 DL_ATT（姿态 + 速度 + 状态）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kDlAttEvery = 1u;
            // 依据：§3.3 帧表：DL_ATT ≈14~20Hz 是最高频回传，必须每槽都发。

            // [槽] 每 2 槽发 DL_STATE（状态机 / 健康 / 失控层次）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kDlStateEvery = 2u;
            // 依据：§3.3：DL_STATE ≈5Hz，够上位机显示状态机与健康位。

            // [槽] 每 4 槽发 DL_SYS（位置 + 电流电压 + 电机值）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kDlSysEvery = 4u;
            // 依据：§3.3：DL_SYS ≈5Hz；电机值只在调试时看，不占高频槽。

            // [槽] 每 16 槽发 DL_GPS（经纬高 + 速度 + 星数）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kDlGpsEvery = 16u;
            // 依据：§3.3：DL_GPS ≈1Hz，与 GNSS 输出率同级（20Hz÷16≈1.25Hz）。

        } // namespace tdma

        // 链路 / 帧常量 — D3：定长帧 [TYPE4|SEQ4] + payload + [CRC8]。类型唯一决定帧长 ⇒ 无需长度字段即可自定界；位宽一改长度就变，生成器据此算长度并 static_assert。
        namespace frame {

            // [位] §3.2：帧头 byte0 = [TYPE(4)
            constexpr uint8_t kTypeShift = 4u;
            // 依据：SEQ(4)]；类型唯一决定帧长（只有 8 种类型，长度是常量表），因此无需长度字段就能自定界。

            // [–] SEQ 占低 4 位，方向内自增、模 16。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kSeqMask = 0x0F;
            // 依据：§3.2：两个用途 —— 丢包率统计，以及 TDMA 相位判定（丢包不会累积错位，因为 SEQ 只是计数不是相位源）。

            // [–] CRC-8 多项式（init 0x00、不反射、不异或输出，即 CRC-8/ATM）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kCrcPoly = 0x07;
            // 依据：§3.2：单字节尾从 [CRC4+Type4] 改成"头上帧 + CRC8"，漏检率从 1/16 降到 1/256，同时换来自定界。

            // [位] UP_CTRL 负载位宽合计：roll/pitch/yaw/thr 各 11b + flags 4b = 48b（6 字节）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kUpCtrlBits = 0x30;
            // 依据：§3.3 帧表：UP_CTRL 共 8B = 帧头 1 + 载荷 6 + CRC 1；生成器据此算长度并 static_assert，位宽一改这里就报错。

            // [位] 四元数分量位宽（3 分量；q0 由归一化还原，符号统一到 q0≥0）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kQuatBits = 12u;
            // 依据：§3.3 DL_ATT：q1,q2,q3 各 12b = 36b；12 位定点在 ±1 范围内分辨率约 5e-4，姿态显示看不出台阶。

            // [位] 速度分量位宽，单位 dm/s。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kVelBits = 8u;
            // 依据：§3.3；8 位有符号 ±12.7 m/s，覆盖四旋翼速度域且不浪费字节。

            // [位] 位置分量位宽，单位 dm。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kPosBits = 12u;
            // 依据：§3.3 DL_SYS：x,y,z 各 12b ⇒ ±204.7m，室内/近场足够。

            // [位] 电机位宽（DShot 计数 0..2047）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kMotorBits = 11u;
            // 依据：§3.3：DShot 计数本身就是 11 位，取 11 位无损；与 kThrMax 同宽，可复用同一套打包/解包代码。

            // [count] 杆量满量程（11 位无符号满值）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint16_t kThrMax = 2047u;
            // 依据：§3.3；与 hw.motor.kUnit/kOffset 的 DShot 计数域一致（1900+50=1950 < 2047）。

        } // namespace frame

        // 命令重传
        namespace cmd {

            // [ms] 命令重传间隔。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kRetryMs = 100u;
            // 依据：§3.7：需要 ACK 的 UP_CMD 在 100ms 无应答后重发；100ms 相对 57600bps 的一帧 4B 有充足余量。

            // [次] 最大重传次数。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kMaxRetry = 5u;
            // 依据：§3.7：5 × 100ms = 500ms 内定夺；再多会一直占上行槽，且用户在 0.5s 内已能看出"没生效"。

        } // namespace cmd

        // 解锁门槛
        namespace arm {

            // [帧] 解锁前必须连续收到的有效上行帧数。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kMinValidUplink = 8u;
            // 依据：§3.7；80Hz 上行 ⇒ 8 帧 = 100ms。要求"连续"是为了挡住 CRC 撞运气或单包重放就解锁。

            // [–] 解锁前油门必须低于此比例。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr float kThrottleLowPct = 0.05f;
            // 依据：§3.7：防止"杆没回中直接解锁"导致飞机猛然起跳。

            // [–] 解锁前 roll/pitch 必须在中位此范围内。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr float kStickCenterPct = 0.05f;
            // 依据：§3.7：同上，作用是让解锁瞬间的期望姿态≈0。

        } // namespace arm

        // 链路 / 遥控映射上限 — M3：摇杆 → 线上量的换算率(增量式的油门/高度)与各轴满杆上限(绝对式的角度/速度)。遥控器固件拿不到本头文件(两端是两套构建)，所以「必须同值」只能靠 test/link_check/check_timing_params.py 对拍 RC_* 宏 —— 一边改了忘另一边，表现只是「上限不一样」，屏幕上完全看不出来。
        namespace rc {

            // 自稳油门增量速率 [1/s] — 自稳模式: 满杆时油门的增量速率(增量式: 推住才变, 松手保持)。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr float kThrottleRate = 0.6f;
            // 依据：0.6/s 下从 0 到满油门约 1.7s; 自回中摇杆用增量式更安全(绝对式会在回中时停在 50%)。与遥控器宏 RC_THROTTLE_RATE 必须同值。

            // 定高高度增量速率 [m/s] — 定高模式: 左摇杆满杆时期望高度的变化率(增量式)。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr float kAltRateMps = 0.6f;
            // 依据：0.6 m/s 下从 0 到 3m 约 5s, 与人的操作节奏匹配; 与遥控器宏 RC_ALT_RATE_MPS 必须同值。

            // 定速水平满杆速度 [m/s] — 定速模式: 右摇杆满杆对应的水平期望速度(绝对式)。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr float kVelXyMaxMps = 2.0f;
            // 依据：本机型 VEL_HOLD 设计域 ±2 m/s; 帧上 int8@5cm/s 能表示 ±6.35 ⇒ 判据有牙, 且与准入门的 maxSpeedMps(= 本值 × 1.5)配套。
            // 与遥控器宏 RC_VEL_MAX_MPS 必须同值。

            // 定速垂向满杆速度 [m/s] — 定速模式: 左摇杆满杆对应的垂向期望速度(绝对式)。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr float kVzMaxMps = 0.8f;
            // 依据：与飞控的 VZ_MAX_DOWN(0.8) 取同值: 指令再大也会被垂速环夹住, 不如一开始就对齐。与遥控器宏 RC_VZ_MAX_MPS 必须同值。

        } // namespace rc

        // 分级失控保护
        namespace failsafe {

            // [ms] 上行丢失超过此值 → WARN（只上报/闪灯）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kWarnMs = 300u;
            // 依据：§3.9；遥控器侧 LINK_TIMEOUT_MS = 300 与此对齐（"多久开始告警"的语义）。

            // [ms] 上行丢失超过此值 → DEGRADE（自动降级 VEL_HOLD → ALT_HOLD → STABILIZE）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kDegradeMs = 1000u;
            // 依据：§3.9：降级而不是停机，保持姿态可控。

            // [ms] 上行丢失超过此值且高度源可用 → HOVER_LAND（保持悬停 + 缓降 + 落地检测 + 自动上锁）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kHoverLandMs = 2000u;
            // 依据：§3.9：高度源不可用或 IMU 失效时才是 CUT（立即断动力）。

            // [m/s] 缓降速度。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr float kHoverLandDescentMps = 0.5f;
            // 依据：§3.9；与 control.kVzMaxDown = 0.8 兼容，留出控制余量（下降限幅先起作用）。

            // [ms] 遥控链路超时保护时间。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kLinkTimeoutMs = 2000u;
            // 依据：改：现有 kConfig.linkTimeoutMs = 5000 太粗——对 VEL_HOLD 等于"悬停中突然断动力"；本轮与 kHoverLandMs 对齐为 2000，取代 
            // 5000。

        } // namespace failsafe

        // 链路 / 速率 profile — D5：可从有线侧切换（切模式有 9~11ms 忙，飞行中不做）。改档不用重编译，但要同时改两端与 E220 寄存器。
        namespace profile {

            // 当前生效的链路 profile（NEAR / NORMAL / FAR / SAFE）。
            // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
            constexpr LinkProfile kActive = LinkProfile::NORMAL;
            // 依据：§3.8：默认 NORMAL（62.5k / 10ms / N=4 ⇒ 80Hz 上行 / 20Hz 下行）。改档不用重编译，但必须同时改两端与 E220 寄存器，且切换要走有线（切模式有 
            // 9~11ms 忙）。

            // NEAR 近场档
            namespace near {

                // NEAR profile 的空中速率。
                // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
                constexpr AirRate kAirRate = AirRate::k62_5k;
                // 依据：§3.8：近场用 62.5k，槽长压到 6ms 换实时曲线（125Hz 上行 / 42Hz 下行）。

                // [ms] NEAR profile 的 TDMA 槽长。
                // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
                constexpr uint32_t kSlotMs = 6u;
                // 依据：§3.8；6ms 槽 ⇒ 125Hz 上行，代价是每槽空口预算更紧（20B 帧需 4.50ms）。

                // [槽] NEAR profile 的 N（3 上 1 下）。
                // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
                constexpr uint8_t kUplinkPerDownlink = 3u;
                // 依据：§3.8：125Hz / 42Hz。

            } // namespace near

            // NORMAL 默认档
            namespace normal {

                // NORMAL profile 的空中速率（默认档）。
                // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
                constexpr AirRate kAirRate = AirRate::k62_5k;
                // 依据：§3.8：62.5k + 10ms + N=4 ⇒ 80Hz 上行 / 20Hz 下行，占空比 29.6%（§0 结论 1）。

                // [ms] NORMAL profile 的 TDMA 槽长。
                // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
                constexpr uint32_t kSlotMs = 24u;
                // 依据：**上板实测(2026-10-06)**。原来 10ms 是按“8B@62.5k ≈ 2.9ms 空口”推的, 而遥控器 'f' 的单帧空口实测是 **16.5ms/帧(8B)**、
                // 32B 25.4ms(固定开销约 13.5ms 占主导, 不是按 8B 线性算)。取 **24ms**: ① 上行: 16.5 + guard 3 = 19.5 ⇒ 20ms 已够(实测 
                // 20ms 时遥控器 38.9/39.0 帧/s、丢帧=2), 24ms 留 4.5ms 余量; ② **下行才是有约束的那一侧**: 飞控一帧 DL_SYS(14B)≈18.7ms(空口+串口)
                // , 而窗口只能开在“主端帧收完”之后、并且必须在下一个超帧的主端帧之前发完 —— 20ms 槽只剩 0.5ms 余量(必然偶发撞车), 24ms 给到约 4ms。配 N=4 ⇒ 超帧 120ms、
                // 上行 33Hz、下行 8.3Hz。必须是 time.ctrl.kTickMs(2ms) 的整数倍(static_assert)。

                // [槽] NORMAL profile 的 N（4 上 1 下）。
                // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
                constexpr uint8_t kUplinkPerDownlink = 4u;
                // 依据：§3.8；与 link.tdma.kUplinkPerDownlink 默认值一致。

            } // namespace normal

            // FAR 远程档
            namespace far {

                // FAR profile 的空中速率。
                // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
                constexpr AirRate kAirRate = AirRate::k19_2k;
                // 依据：§3.8：距离优先，降到 19.2k（灵敏度更高）；注意 UART 波特率硬约束随之变成 ≤ 24000，两端与模块都要同步降。

                // [ms] FAR profile 的 TDMA 槽长。
                // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
                constexpr uint32_t kSlotMs = 20u;
                // 依据：§3.8；20ms ⇒ 40Hz 上行 / 10Hz 下行。

                // [槽] FAR profile 的 N（4 上 1 下）。
                // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
                constexpr uint8_t kUplinkPerDownlink = 4u;
                // 依据：§3.8：40Hz / 10Hz。

            } // namespace far

            // SAFE 安全档
            namespace safe {

                // SAFE profile 的空中速率（最后手段）。
                // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
                constexpr AirRate kAirRate = AirRate::k9_6k;
                // 依据：§3.8：9.6k 是最低档，链路最稳但只能跑 25Hz 上行；进入 SAFE 应视为"准备降落"。

                // [ms] SAFE profile 的 TDMA 槽长。
                // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
                constexpr uint32_t kSlotMs = 40u;
                // 依据：§3.8；40ms ⇒ 25Hz 上行 / 8Hz 下行。

                // [槽] SAFE profile 的 N（3 上 1 下）。
                // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
                constexpr uint8_t kUplinkPerDownlink = 3u;
                // 依据：§3.8：25Hz / 8Hz。

            } // namespace safe

        } // namespace profile

    } // namespace link

    namespace sensor {

        // 传感器 / 气压计（BMP388） — E1：寄存器配置与健壮性阈值。注意器件已由 BME280 换成 BMP388（CHIP_ID 在 0x00、值 0x50），驱动必须新写。
        namespace baro {

            // BMP388 过采样寄存器 0x1C：压力 ×8 / 温度 ×1。
            constexpr uint8_t kOsr = 0x03;
            // 依据：新增。定高靠气压的长时基准，压力过采样 ×8 把噪声压到 ~0.1m 量级；温度变化慢，×1 足够省时间。位域：osr_t[5:3]=0、osr_p[2:0]=3。

            // BMP388 输出数据率寄存器 0x1D = 50Hz。
            constexpr uint8_t kOdr = 0x02;
            // 依据：新增。io 线程 20ms（50Hz）读一次，ODR 与读取周期对齐可拿到每周期一个新样本，不会重复读同一次转换。**但手册 Table 22 说 ×8 过采样 + ×1 温度时 
            // T_conv 最坏 21.53ms > 20ms 周期**（最坏有效速率约 46.4Hz，典型 18.69ms 则够）。这是手册 Table 10 “Drone” 行的推荐组合，所以不改值 
            // —— 代价是**最坏情况下偶尔拿不到新样本**，驱动返回 NotReady，不发布快照、不刷新采样时间戳或健康时间戳；只有 NewData 才进入参考高度计算与 EKF。真要“每周期必有一个新样本”的硬保证，
            // 把本项改 3（25Hz）或把 sensor.baro.kOsr 降到 ×4。

            // BMP388 配置寄存器 0x1F：IIR 滤波系数 3（bits3:1 = 0b010）。
            constexpr uint8_t kConfig = 0x04;
            // 依据：新增。IIR 系数 3 对应数据手册的滤波器档位 3，抑制螺旋桨引起的压力脉动；再大（15/31）会引入高度滞后，定高会晃。

            // BMP388 中断配置寄存器 0x19 的值（数据就绪中断：推挽、高有效、不锁存、仅 drdy 使能）。
            constexpr uint8_t kIntCtrl = 0x44;
            // 依据：§2.3 的中断资源表里没有气压计 INT（只有 E220 AUX 的 EXTI2 与 VL53L1X GPIO1 的 EXTI3）⇒ 本板走 io 线程 20ms 轮询，这一项按手册配好即可、
            // 不作功能依赖（推理项，上板回读确认）。

            // BMP388 电源控制寄存器 0x1B 的值：bit0=press_en、bit1=temp_en、bit4/5=mode（00 睡眠 / 01 强制 / 10 正常）⇒ 0x33 = 
            // 正常模式 + 温度 + 气压。
            constexpr uint8_t kPwrCtrl = 0x33;
            // 依据：2026-10-02 补：写驱动的子代理指出这一项**当时不在 schema 里**，而它是 init 序列里唯一决定“器件到底跑不跑”的寄存器 —— 值只能硬编码在驱动头文件里，
            // 等于配置的唯一真相源漏了一项（正好是本轮一直在治的那类病）。取值与数据手册 Table 10 的“Drone”行、以及 ArduPilot 的默认一致。

            // [次] 气压计初始化最多重试次数。
            constexpr uint32_t kInitRetry = 20u;
            // 依据：沿用现有 kConfig.baroInitRetry = 20。开机先初始化 flash，那一段片内擦写干扰可能让气压计一时不应答；重试让健康机器第一次就成、有问题的机器自动等到总线可用。

            // [ms] 初始化重试间隔（20 × 100ms = 最多等 2s）。
            constexpr uint32_t kInitRetryMs = 100u;
            // 依据：沿用现有 kConfig.baroInitRetryMs = 100。

            // 连续读失败阈值（只影响上报的错误码） [次] — 连续读失败阈值。**v2.1 之后它不再驱动 health.baro**：采集层改成『一次读失败即清 kHealthBaro、一次成功即置回』，
            // 遥测 height 语义与 kTelErrBaro 都跟着那一位走。它现在只剩一个作用：BMP388 驱动拿它决定何时把 err 从 BARO_READ_FAILED 升级成 BARO_DATA_STALE（10 
            // × 20ms = 200ms）。
            constexpr uint32_t kFailLimit = 10u;
            // 依据：沿用现有 kConfig.baroFailLimit = 10。v2.0 的用法是『连续失败到阈值 ⇒ health.baro = 0』，那条路径在 v2.1 被删掉了（传感器读是原子的：
            // 一次失败 = 这一次没数据，一次成功 = 又能用；连续计数只让『现在到底能不能用』更难回答）。

            // [s] 额外软件低通时间常数，0 = 不滤（靠 BMP388 的 IIR）。
            constexpr float kTimeConstS = 0.0f;
            // 依据：新增。器件 IIR 已经足够，再加一级软件滤波会给高度环贡献相位滞后，默认关。

        } // namespace baro

        // 传感器 / 测距（VL53L1X） — E2：量程模式、时序预算、有效性门限与标定值。读数 0 = 无效，绝不当“贴地”。
        namespace range {

            // VL53L1X 测距模式：Short(1.3m) / Medium(2.9m) / Long(3.6m)。
            // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
            constexpr RangeDistanceMode kDistanceMode = RangeDistanceMode::Short;
            // 依据：§4.2：Short 抗环境光最好，室内定高与缓降够用；室外需求明确后再上 Long（Long 对光照和盖板玻璃更敏感）。enum 值对应 ST API 的 1/2/3。

            // [ms] 单次测距的时间预算（20..1000ms）。
            // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
            constexpr uint16_t kTimingBudgetMs = 33u;
            // 依据：§4.2：Short + 33ms 实测有效距离 ≈1.5~2m，与 kMaxValidMm=2000 自洽。预算越大越抗光但也越慢。

            // [ms] 连续测距间隔周期，必须 ≥ kTimingBudgetMs，上限 50Hz。
            // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
            constexpr uint16_t kInterMeasMs = 50u;
            // 依据：§4.2；50ms ⇒ 20Hz 高度更新，比控制环慢两个数量级，靠 EKF 的测距观测（σ=0.05m）补相位。

            // GPIO1 中断极性：默认高有效 ⇒ EXTI 上升沿。
            // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
            constexpr RangeIntPolarity kIntPolarity = RangeIntPolarity::ActiveHigh;
            // 依据：§0.1 事实 8：VL53L1X GPIO1 默认高有效，与常见描述相反。也可以写 0x0030 bit4=1 改成低有效，但改了要与 EXTI 触发边沿、kInt 极性一起改。

            // 3.3V 供电必开：写 0x002E bit0=1 与 0x002F bit0=1 切到 2V8 I/O 模式。
            // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
            constexpr bool kIOMode2V8 = true;
            // 依据：§0.1 事实 7：VL53L1X 没有 DVDD/IOVDD 引脚，I/O 是"模式"（默认 1V8）；不写这两个寄存器，I²C/GPIO 高电平阈值不对，可能直接不通。

            // [mm] 最小可用距离；读数 0 = 无效，绝不当"贴地"。
            // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
            constexpr uint16_t kMinValidMm = 40u;
            // 依据：§4.2：0 是"无目标/超量程"的返回值，把它当 0m 会让定高把飞机往地上压；40mm 以下本就超出模块光学设计。

            // [mm] 超过即标为无效。
            // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
            constexpr uint16_t kMaxValidMm = 2000u;
            // 依据：与 kTimingBudgetMs=33 + Short 模式的实测量程一致（§4.2 约 1.5~2m）。

            // 是否做 crosstalk/增益 2% 修正：mm = (raw × 2011 + 0x400) / 0x800。
            // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
            constexpr bool kGainCorrect = true;
            // 依据：§4.2 沿用 ST API 的 gain correction；盖板玻璃的 crosstalk 另由 kCalibCrosstalk 处理。

            // 连续无效/失败阈值（只影响上报的错误码） [次] — 连续无效/失败阈值。**v2.1 之后它不再驱动 health.range**：采集层改成『一次真实失败即清 kHealthRange、
            // 一次成功即置回』，而量程无效（RANGE_INVALID）不算失败、不动健康位。它现在只剩一个作用：VL53L1X 驱动拿它决定何时把 err 升级成 RANGE_DATA_STALE。
            // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kFailLimit = 5u;
            // 依据：新增；与 sensor.baro.kFailLimit 同风格。v2.0 的用法是『超过则 health.range = 0』，那条路径在 v2.1 被删掉了，见 baro 的同一处说明。

            // [count] 盖板玻璃 crosstalk 标定值，0 = 未标定。
            // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
            constexpr uint16_t kCalibCrosstalk = 0u;
            // 依据：§4.2：装透明盖板后必须标定，否则近距离读数偏小（标定前 kMinValidMm 要留足余量）。

            // [mm] 零偏标定值（安装面到光学面的机械偏差）。
            // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
            constexpr int16_t kCalibOffsetMm = 0;
            // 依据：§4.2：上板后用卷尺量一次（已知高度 vs 读数差）写入，避免把机械偏差算进 EKF 高度。

            // [个] RefSPAD 标定值，0 = 用出厂默认。
            // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kRefSpadCount = 0u;
            // 依据：§4.2：只有在极弱回波（远距离/低反射率）下才需要自己做 RefSPAD 标定，先保持 0。

        } // namespace range

        // 传感器 / 光流（PMW3901） — E3：尺度标定、符号、质量门限与失效阈值。尺度 kRadPerCount 直接决定速度量纲，必须实测标定。
        namespace flow {

            // [rad/count] 光流标定尺度：≈409.3 counts/rad（默认 1 rad 视角）。必须标定。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr float kRadPerCount = 0.002443389f;
            // 依据：§4.3：这个值直接决定速度量纲，错了速度环就是错的；默认值来自器件典型视场，必须用"已知平移距离 + 已知高度"实测标定后覆盖。

            // [deg] 光流模块相对机体的偏转（0/90/180/270）。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr int32_t kYawDeg = 0;
            // 依据：§4.3；装歪了先改这个，别去改符号（符号只管镜像/反相）。

            // X 轴符号（v_x = −ω_x · h）。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr int8_t kSignX = -1;
            // 依据：§4.3：器件坐标与机体坐标反向，默认 −1；装反了只改这两个符号，不动标定。

            // PMW3901 的 y 轴符号。实测(2026-10-03 台架): 模块的**左右轴与 IMU 的 +y 相反**, 所以这里要翻号。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr int8_t kSignY = 1;
            // 依据：2026-10-03 台架实测标定(有线连接, 固定飞机、推动下方有纹理的纸面): **前后方向与 IMU +x 同向**(因此 kSignX 保持 -1 不动), **左右方向与 
            // IMU +y 相反** ⇒ 本项从 -1 改为 **+1**。测法: 把飞机压在离地 15~20cm 处, 沿机头方向推下方纸面 10cm, 看固件输出的'机体位移 前/左' —— 前后为正、
            // 左右为正才说明两个轴都对了; 本次实测 前 为正(对) 而 左 为负(反), 故只翻 y。⚠️ 当时的读数来自光流线程里一条 **4Hz 的 `[Flow] ...` 台架行**, 该行已在标定完成后**按'用完即拆'删掉**(见 
            // docs/2026-10-02_upgrade_progress.md §九 的复核步骤), 所以再次复核时要**临时**把那一行加回去, 不要把它变回常驻打印。

            // SQUAL 质量门限（0 = 完全无特征，如纯色地面/悬空）。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kMinSqual = 25u;
            // 依据：§4.3 沿用 PX4 判据：低于门限直接拒绝该样本，否则低质量样本会以"零速"的形式毒化 EKF。

            // [count] 单帧位移超过即视为 SPI 垃圾数据（PX4 判据）。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr uint16_t kMaxDelta = 240u;
            // 依据：§4.3；SPI 读错时 delta 常见为全 1（0xFFFF）或全 0，物理上不可能一帧跳 240 counts。

            // [count] 饱和门限。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr uint16_t kSatDelta = 2047u;
            // 依据：§4.3：接近饱和的样本没有线性度，宁可丢弃（丢样本比喂错速度安全）。

            // [m] 低于此高度光流不可用（焦点/景深限制）。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr float kMinHeightM = 0.15f;
            // 依据：§4.3；与 sensor.range.kMinValidMm=40 一起构成"近地保护"：低于此高度速度环自动降级为 ALT_HOLD。

            // [m] 高于此高度光流不可用（纹理分辨不足）。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr float kMaxHeightM = 8.0f;
            // 依据：§4.3；超过即拒绝光流观测，避免把"看不出纹理"当成"零速度"。

            // [s] 速度低通时间常数，放在光流前端，不要滤 EKF 输出。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr float kLpfTauS = 0.08f;
            // 依据：§4.3/§6.3：滤前端只影响观测本身；滤 EKF 输出会破坏协方差一致性，是"看起来平滑但估计发散"的经典来源。

            // [rad/s] 器件的最大角速率（超出即饱和）。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr float kMaxRateRads = 7.4f;
            // 依据：§4.3 器件规格；超过此值的样本一定已被 kSatDelta 截断。

            // [m/s] 换算后速度上限，超过即拒绝。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr float kMaxVelMps = 3.0f;
            // 依据：§4.3；与 control.kVelXyMaxMps=2.0 配合：观测允许略大于控制上限，但不接受明显离谱的值。

            // 连续拒绝阈值（当前不改变行为） [次] — 连续拒绝阈值。⚠ **目前不改变任何行为**：它只喂给 `OpticalFlowEstimator::healthy()`，而控制环并没有读 
            // healthy() —— 速度环进不进 VEL_HOLD 看的是『本帧估计是否有效』(est.valid) 与参数自洽性，health.flow 则由采集线程按『一次读失败即清位』维护。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kFailLimit = 5u;
            // 依据：新增。v2.0 的意图是『连续拒绝 ≥ 阈值 ⇒ health.flow = 0』，但那条 health 位现在由采集层直接维护（见 sensor.baro.kFailLimit 
            // 的同一处说明）。保留本项是因为估算器内部确实在用它判 healthy()；要么后续把 healthy() 接进速度环的准入，要么把这一项删掉（v2.1 审计已把它列进『配置存在但控制不了任何东西』的清单）
            // 。

            // [s] 超过即视为过期样本。
            // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
            constexpr float kTimeoutS = 0.3f;
            // 依据：新增；与 control.kVelFlowTimeoutS 同量级，双保险：前端不给旧数据，控制端再兜一层。

        } // namespace flow

        // 传感器 / GNSS（NEO-7M） — E4：只把水平速度进 EKF（updateGpsVelocity）；位置/高度永远只回传。
        namespace gnss {

            // [Hz] 期望的模块输出率，需要先用 UBX CFG-RATE 配模块。
            // 仅当 cfg::feature::kGnss 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kRateHz = 5u;
            // 依据：§4.4：模块出厂 1Hz/9600；5Hz 需要先改模块（配置保存在模块里，代码只做校验与解析）。

            // [ms] 数据过期阈值。**500 → 1500（2026-10-02 改）**：NEO-7M 出厂是 9600bps @ **1Hz** 输出，句子到达后 500ms 就判过期，于是 
            // speedValid 每秒钟闪断一半时间，B10 会看到"速度观测时有时无"。1Hz 下必须 ≥1100ms；若把模块用 u-center 配到 5Hz（UBX CFG-RATE），可以收紧回 
            // 500。
            // 仅当 cfg::feature::kGnss 为 true 时存在(编译期整块裁掉)。
            constexpr uint32_t kStaleMs = 1500u;
            // 依据：§4.4 的 500ms 是按"5Hz 输出"算的，但模块的 5Hz 是**要另外配的**，而固件里没有任何 UBX 配置代码 —— 也就是说不配模块它跑的是 1Hz 默认值。改 
            // 1500 是让"开箱即用"自洽：1Hz 下 1500ms ≈ 允许连续丢 0.5 帧，5Hz 下则允许连丢 7 帧（偏松但在可接受范围）。真正想两头都对，得在 B10 里加一句 UBX 
            // CFG-RATE 或让用户用 u-center 配好。

            // [颗] 少于该卫星数不喂速度观测。
            // 仅当 cfg::feature::kGnss 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kMinSats = 6u;
            // 依据：§4.4；少星时速度误差可达数 m/s，比不喂观测更糟。

            // HDOP 门限。
            // 仅当 cfg::feature::kGnss 为 true 时存在(编译期整块裁掉)。
            constexpr float kMaxHdop = 2.0f;
            // 依据：§4.4；HDOP > 2 说明几何分布差，水平速度不可信。

            // [m/s] GPS 速度观测噪声（1σ）。
            // 仅当 cfg::feature::kGnss 为 true 时存在(编译期整块裁掉)。
            constexpr float kVelSigmaMps = 0.3f;
            // 依据：§4.4；NEO-7M 多普勒测速精度约 0.1~0.3 m/s，取 0.3 偏保守。

            // 只影响水平速度观测（updateGpsVelocity）；位置/高度永远只回传不进控制。
            // 仅当 cfg::feature::kGnss 为 true 时存在(编译期整块裁掉)。
            constexpr bool kUseInEkf = true;
            // 依据：§4.4 融合策略：位置米级精度、延迟数十毫秒，进控制是负收益；速度观测正好补 FF_VX/VY 从来没量测的洞。

            // 只解析需要的 NMEA 语句以省 CPU。位定义见 enums.NmeaMaskBits。
            // 仅当 cfg::feature::kGnss 为 true 时存在(编译期整块裁掉)。
            constexpr uint8_t kNmeaMask = 0x07;
            // 依据：§4.4：RMC 给速度/时间/定位有效性、GGA 给定位质量与卫星数、GSA 给 HDOP；GSV/VTG/GLL/ZDA 不解析。

        } // namespace gnss

    } // namespace sensor

    // EKF / 状态估计 — F：倾角修正整定（2026-09-18 按 sim/REPORT.md 仿真结果同步）+ 光流/测距观测门限 + 协方差钳位。
    namespace ekf {

        // [样本] 每 N 个 IMU 样本做一次协方差传播 + 倾角更新。
        constexpr uint32_t kDecim = 8u;
        // 依据：沿用现有 kConfig.ekfDecim = 8；IMU 2000Hz ÷ 8 = 250Hz 慢路径，主环最坏 slot 才压得住。

        // 在线估计加速度零偏（纯 IMU 不可观）。
        constexpr bool kEstimateAccelBias = false;
        // 依据：沿用现有 ekfEstimateAccelBias = false；不可观的量估出来只会漂，还会污染倾角观测。

        // [m/s²] 瞬时快筛：‖a‖−g 超过就丢掉这次倾角观测（旧值 2.0）。
        constexpr float kAccelTiltGateMss = 0.2f;
        // 依据：2026-09-18 仿真整定（sim/REPORT.md 附录 A/B）：旧值 2.0 等于不过滤 —— 多旋翼加速度计测的是推力方向，机动时把它当重力方向用会把姿态"拉平"（真前倾 
        // 45° 而估计只有 6°）。

        // [–] 倾角修正量测噪声（方向，无量纲）；越小越信加速度计。
        constexpr float kAccelTiltSigma = 0.08f;
        // 依据：沿用现有 ekfAccelTiltSigma = 0.08；配合门限 + innovation 限幅 + 零偏冻结，静态精度不牺牲。

        // [rad] 单次修正 innovation 限幅（≈8.6°）。
        constexpr float kAccelTiltInnoLimitRad = 0.15f;
        // 依据：沿用现有 ekfAccelTiltInnoLimitRad = 0.15；防单帧把姿态拽走（起飞瞬间最危险）。

        // 只有通过判定的那次更新才允许改陀螺零偏。
        constexpr bool kFreezeGyroBiasWhenDynamic = true;
        // 依据：沿用现有 ekfFreezeGyroBiasWhenDynamic = true；机动中用被污染的数据调零偏正是"估计跑到 −17°/s"的成因之一。

        // 准静止判定开关。
        constexpr bool kAccelHoldEnable = true;
        // 依据：沿用现有 ekfAccelHoldEnable = true（注意 mains/main_att_ekf.cpp 里是 false，两个文件行为不一致，B0 统一）。

        // [m/s²] 准静止判据：‖a‖−g 上限（静止实测 p95 < 6.5mg）。
        constexpr float kAccelHoldGateMss = 0.15f;
        // 依据：沿用现有 ekfAccelHoldGateMss = 0.15；0.15 m/s² ≈ 15mg，比静止噪声高一个量级。

        // [deg/s] 准静止判据：角速度上限。
        constexpr float kAccelHoldRateDps = 150.0f;
        // 依据：沿用现有 ekfAccelHoldRateDps = 150；手抛/强风下不满足，倾角修正自动暂停。

        // [s] 准静止判据：需要连续满足的时长。
        constexpr float kAccelHoldTimeS = 0.05f;
        // 依据：沿用现有 ekfAccelHoldTimeS = 0.05；50ms 足以滤掉振动引起的瞬时"假静止"，又不会让修正等太久。

        // [m] 气压高度噪声（1σ）。
        constexpr float kBaroSigmaM = 2.0f;
        // 依据：改：flight_log_analysis/params_to_change.md 的实测结论是气压误差 1.6~4.6m，原来的 1.0 偏乐观（相当于把气压当比测距还准的源）。
        // 取 2.0 作为长时基准。

        // [m] 测距高度噪声（1σ）。
        constexpr float kRangeSigmaM = 0.05f;
        // 依据：新增：VL53L1X 在量程内噪声就是厘米级，σ=0.05 让它成为量程内主导的高度源（§6.4 第 6 条"高度源仲裁"）。

        // [σ] 测距 innovation 门限（Mahalanobis）。
        constexpr float kRangeGateSigma = 3.0f;
        // 依据：新增：§6.4 第 2/5 条 —— 现在 applyHeightUpdate 是 void、无门限、无有效性检查，超量程的 0 值会被当真实观测注进去；3σ 是工程常用值。

        // [rad/s] 陀螺零偏钳位。
        constexpr float kGyroBiasLimitRads = 0.03f;
        // 依据：改：0.3 等于没限（真实零偏只有 0.1~0.7°/s = 0.002~0.012 rad/s），实测估计却跑到 −17°/s ⇒ 0.03 rad/s（≈1.7°/s）足够容纳真实零偏又挡住发散。

        // [(rad/s)²] 初始零偏方差。
        constexpr float kInitGyroBiasVar = 1.0E-5f;
        // 依据：改：1e-4 太松，开机头几秒零偏修正权重过大；1e-5 与"零偏极限 0.03 rad/s"自洽（σ≈0.003）。

        // [m/s] 光流速度观测噪声（在 1m 高度处；实际按 z_agl 缩放）。
        constexpr float kFlowVelSigmaMps = 0.2f;
        // 依据：新增：光流误差随高度线性放大（角速率固定时速度误差 ∝ 高度），所以 σ 必须随 z_agl 缩放，否则高处会过度信任。

        // [m/s] 光流 innovation 逐轴限幅。
        constexpr float kFlowInnoLimitMps = 1.5f;
        // 依据：新增：§6.4 第 2 条；kalmanUpdate 没有通用 innovation 门限，光流被污染（落到无纹理地面）时会直接把姿态和速度拽走。

        // [σ] 光流 Mahalanobis 门限（χ²₃(0.997) = 14.16）。
        constexpr float kFlowGateSigma = 3.0f;
        // 依据：新增：与 control.kVelFlowTimeoutS 一起构成"坏数据进不来"的双层防护。

        // [(m/s)²] 水平速度方差上限（(5 m/s)²）。
        constexpr float kVelXyVarMax = 25.0f;
        // 依据：新增：§6.4 第 3 条 —— FF_VX/VY 从来没有量测，P 无界增长（60s 后 σ_v≈1.16 m/s，第一次光流更新的增益≈0.97，等于直接跳变）；钳位是加光流观测的前置条件。

        // [m²] 水平位置方差上限（(10 m)²）。
        constexpr float kPosXyVarMax = 100.0f;
        // 依据：新增：同上，位置同样无量测，钳住防止 P 把所有更新的增益都推到 1。

        // [条] 每 N 条日志记一次 EKF 零偏（降带宽）。
        constexpr uint32_t kLogDivider = 10u;
        // 依据：沿用现有 kConfig.ekfLogDivider = 10；零偏变化慢，50Hz 日志里每 10 条记一次即可。

        // 解锁时保留已收敛的陀螺零偏。
        constexpr bool kKeepGyroBiasOnArm = true;
        // 依据：新增：现在 ekf.reset() 会把零偏清掉，解锁瞬间等于把刚收敛好的零偏丢掉重新学 —— 首秒的姿态漂移就来自这里。

    } // namespace ekf

    // 控制器（对应 FlightParamId） — G：这些运行期可写（PARAM_SET，台架试参），所以保留 FlightControlParams 运行期表；cfg::control::kDefaults[] 只提供上电默认值。顺序必须与 FlightParamId 一致（static_assert 校验长度）。
    namespace control {

        // [m] 机臂长度。
        constexpr float kArmLengthM = 0.125f;
        // 依据：FlightParamId::ARM_LENGTH_M 默认 0.125f；顺序与 kFlightControlParamDefaults[] 一致（static_assert 校验长度）
        // 。

        // [kg] 总质量。
        constexpr float kMassKg = 1.566f;
        // 依据：电机 600g + 机架 250g + 电池 716g（flight_control_params.hpp 注释）。

        // [kg·m²] 绕机体系 x 轴转动惯量。
        constexpr float kInertiaXx = 0.0234f;
        // 依据：4×150g 电机@长边半距 + 250g 机架细杆 + 716g 电池@z=5cm，折算到重心。

        // [kg·m²] 绕机体系 y 轴转动惯量。
        constexpr float kInertiaYy = 0.0124f;
        // 依据：同上（短边方向，惯量更小）。

        // [kg·m²] 绕机体系 z 轴转动惯量。
        constexpr float kInertiaZz = 0.0338f;
        // 依据：同上（偏航惯量最大，所以偏航环增益最小）。

        // [m] 旋翼偏航反力矩等效力臂（力矩/推力比），混控换算用。
        constexpr float kYawTorqueArmM = 0.03f;
        // 依据：kFlightControlParamDefaults[] 中 YAW_TORQUE_ARM_M = 0.03f。

        // [–] 悬停油门（归一化 0~1）。**实测约 0.28**（2026-10-02 确认），不是 0.25 也不是 0.20。
        constexpr float kHoverThrottle = 0.28f;
        // 依据：它有两处用途，改它等于同时改两件事：① 定高模式的基准油门 throttle = hover*(1+acc_z/g)；② 混控把"期望力矩"换算成"油门差动"的比例 k = m*g/(4*hover)
        //  —— hover 调小 ⇒ k 变大 ⇒ 同样力矩需要的差动变小 ⇒ 姿态环等效增益按 1/hover 变化。0.25→0.28 相当于姿态环等效增益降到原来的 0.89 倍；若手感偏软，
        // 把 RATE_ROLL/PITCH_P 从 9.0 提到 ~10.1 补回来。

        // [m/s²] 重力加速度标称值。
        constexpr float kGravityMss = 9.80665f;
        // 依据：2026-10-02 补：写速度环的子代理发现这个常数**在仓库里有两份独立的硬编码** —— `flight_controller.hpp` 的 `FlightControlFilterParams:
        // :gravity_mss` 结构体默认值、以及速度环新加的一份，以后按当地纬度标定 g 时会只改一处。放进配置以后两边都从这里取（结构体默认值仍留一份兜底，但接线处必须显式赋 cfg::
        // control::kGravityMss）。范围 9.7~9.9 覆盖赤道到极地，超出这个范围一定是写错了。

        // [–] 油门下限（归一化）。
        constexpr float kThrottleMin = 0.05f;
        // 依据：kFlightControlParamDefaults[] = 0.05f。

        // 角度环的最小集体油门。混控要靠非零油门才能出力矩（T=0 ⇒ 混控比例 s=0 ⇒ 不出力矩），所以解锁后油门不能真的到 0。
        constexpr float kAngleLoopMinThrottle = 0.01f;
        // 依据：现状 kConfig.minThrottle = 0.01（release.cpp:1409 `out.throttle = max(manThr, minThrottle)`）。
        // 注意它与 THROTTLE_MIN(0.05) 不是一回事：后者是遥控杆量的下限/高度环限幅。

        // [ms] 解锁到控制接管的等待时间（启动序列）。这段时间内推油门无效，用来等电调初始化 + 累积气压基准。
        constexpr uint32_t kMotorArmDelayMs = 3200u;
        // 依据：现状 kConfig.motorArmDelayMs = 3200（release.cpp:1383）。

        // [–] 油门上限（归一化）。
        constexpr float kThrottleMax = 1.0f;
        // 依据：kFlightControlParamDefaults[] = 1.0f。

        // [rad] 允许的最大倾角（≈20°，首飞保守）。
        constexpr float kTiltMaxRad = 0.35f;
        // 依据：kFlightControlParamDefaults[] = 0.35f。注意：全工程目前无人使用这一项，速度环的倾角限幅用 kVelTiltMaxRad（§1.3）。

        // [–] roll 角度环比例增益。
        constexpr float kAttRollP = 3.0f;
        // 依据：kFlightControlParamDefaults[] = 3.0f（首飞保守）。

        // [–] pitch 角度环比例增益。
        constexpr float kAttPitchP = 3.0f;
        // 依据：同 roll。

        // [–] yaw 角度环比例增益（预留）。
        constexpr float kAttYawP = 3.0f;
        // 依据：kFlightControlParamDefaults[] = 3.0f；单 IMU 偏航不准，实际由 ATT_YAW_WEIGHT=0 关闭。

        // [–] 偏航权重。
        constexpr float kAttYawWeight = 0.0f;
        // 依据：kFlightControlParamDefaults[] = 0.0f：单 IMU 偏航不准，保持 0。

        // [rad/s] roll 角速度限幅（≈114°/s）。
        constexpr float kRateRollMax = 2.0f;
        // 依据：kFlightControlParamDefaults[] = 2.0f。

        // [rad/s] pitch 角速度限幅。
        constexpr float kRatePitchMax = 2.0f;
        // 依据：同上。

        // [rad/s] yaw 角速度限幅（≈57°/s）。
        constexpr float kRateYawMax = 1.0f;
        // 依据：kFlightControlParamDefaults[] = 1.0f：偏航惯量大，限幅更紧。

        // [–] roll 角速度环比例增益。
        constexpr float kRateRollP = 9.0f;
        // 依据：kFlightControlParamDefaults[] = 9.0f（首飞保守，原 8.0 因悬停油门下调而提高）。若 kHoverThrottle 再降，需按 1/hover 
        // 关系继续补。

        // [–] roll 角速度环积分增益。
        constexpr float kRateRollI = 10.0f;
        // 依据：2026-09-18 仿真整定：2.0 → 10.0（×5）。常值力矩扰动（重心偏前/四路推力不一致）只有积分能抵消；x5 落在"抗扰够用 + 动态不退化"的窗口（x10 最优但方波超调变大，
        // x40 明显变差）。台架若见振荡退回 6.0。

        // [–] roll 角速度环微分增益（作用于实测角加速度）。
        constexpr float kRateRollD = 0.015f;
        // 依据：kFlightControlParamDefaults[] = 0.015f。

        // [–] pitch 角速度环比例增益。
        constexpr float kRatePitchP = 9.0f;
        // 依据：同 roll。

        // [–] pitch 角速度环积分增益。
        constexpr float kRatePitchI = 10.0f;
        // 依据：同 roll（x5 整定）。

        // [–] pitch 角速度环微分增益。
        constexpr float kRatePitchD = 0.015f;
        // 依据：kFlightControlParamDefaults[] = 0.015f。

        // [–] yaw 角速度环比例增益。
        constexpr float kRateYawP = 5.0f;
        // 依据：kFlightControlParamDefaults[] = 5.0f。

        // [–] yaw 角速度环积分增益。
        constexpr float kRateYawI = 1.0f;
        // 依据：kFlightControlParamDefaults[] = 1.0f。

        // [–] yaw 角速度环微分增益。
        constexpr float kRateYawD = 0.01f;
        // 依据：kFlightControlParamDefaults[] = 0.01f。

        // [–] roll 角速度前馈（预留）。
        constexpr float kRateRollFf = 0.0f;
        // 依据：kFlightControlParamDefaults[] = 0.0f，暂未使用。

        // [–] pitch 角速度前馈（预留）。
        constexpr float kRatePitchFf = 0.0f;
        // 依据：同 roll。

        // [–] yaw 角速度前馈（预留）。
        constexpr float kRateYawFf = 0.0f;
        // 依据：同上。

        // [rad/s²] 期望角加速度限幅（防首飞过冲）。
        constexpr float kAngAccMax = 40.0f;
        // 依据：kFlightControlParamDefaults[] = 40.0f。

        // [rad/s²] 角速度环积分限幅。
        constexpr float kRateIntLimit = 5.0f;
        // 依据：kFlightControlParamDefaults[] = 5.0f；可抵消的常值力矩 = I × RATE_INT_LIMIT，与 RATE_*_I 的收敛速度共同决定抗扰能力。

        // [s] 角速度微分项低通时间常数。
        constexpr float kRateDTauS = 0.01f;
        // 依据：kFlightControlParamDefaults[] = 0.01f。

        // [1/s] 高度误差 → 期望垂向速度。
        constexpr float kPosZP = 2.0f;
        // 依据：kFlightControlParamDefaults[] = 2.0f。

        // [1/s] 垂向速度误差 → 期望垂向加速度。
        constexpr float kVelZP = 3.5f;
        // 依据：kFlightControlParamDefaults[] = 3.5f。

        // [1/s²] 垂向速度积分增益。
        constexpr float kVelZI = 0.6f;
        // 依据：kFlightControlParamDefaults[] = 0.6f。

        // [m/s] 最大爬升速度。
        constexpr float kVzMaxUp = 1.5f;
        // 依据：kFlightControlParamDefaults[] = 1.5f。

        // [m/s] 最大下降速度（正值）。
        constexpr float kVzMaxDown = 0.8f;
        // 依据：kFlightControlParamDefaults[] = 0.8f；缓降（failsafe 0.5 m/s）留在这之内。

        // 期望高度下界 [m] — 期望高度的下界(地面软地板)。
        // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
        constexpr float kAltMinM = 0.0f;
        // 依据：高度源是测距(feature.kRange), 地面附近读数会抖; 下界防止"目标被写成负高度"导致一直往下压。

        // 期望高度上界 [m] — 定高与自动起降的高度上限(期望高度目标被夹到这里)。
        // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
        constexpr float kAltMaxM = 10.0f;
        // 依据：室内/首飞场景 10m 已经远超操作半径; 上限的作用是挡住"遥控器积分器跑飞"造成的持续爬升。

        // 自动起飞目标高度 [m] — 自动起飞的目标高度。
        // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
        constexpr float kTakeoffHeightM = 1.2f;
        // 依据：室内首飞取 1.2m: 高于地面效应区、又低于一般层高; 与遥控器侧 RC_TAKEOFF_HEIGHT_M 必须同值。

        // 自动起飞爬升速率 [m/s] — 自动起飞的爬升速率(高度目标斜坡)。
        // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
        constexpr float kTakeoffClimbMps = 0.5f;
        // 依据：取 0.5 m/s: 1.2m 约 2.4s 到位, 比 VZ_MAX_UP(1.5)留足余量, 位置环不会饱和。

        // 自动降落下降速率 [m/s] — 自动降落的下降速率(用户明确要求"慢降, 触地判定由人做")。
        // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
        constexpr float kLandDescentMps = 0.25f;
        // 依据：0.25 m/s 下最后 20cm 要 0.8s —— 人来得及在触地时上锁; 比标准 0.5 更慢是刻意的。

        // 自动降落软地板 [m] — 自动降落的软地板: 高度低于它就停止下降(但**不自动上锁**)。
        // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
        constexpr float kLandFloorM = 0.1f;
        // 依据：高度测量不可信, 所以只当"别往地里压"的软墙; 最后的触地判定与上锁由人完成(见方案 D7)。

        // [s] 垂向速度估计低通时间常数。
        constexpr float kVzEstTauS = 0.25f;
        // 依据：kFlightControlParamDefaults[] = 0.25f。

        // [m] 每个电机到中心的纵向（前后）偏移（前向半距）。
        constexpr float kArmForwardM = 0.12933f;
        // 依据：由对角轴距 445mm 与 长:短 = 1.4:1 反推（flight_control_params.hpp 注释）。

        // [m] 每个电机到中心的横向（左右）偏移（横向半距）。
        constexpr float kArmLateralM = 0.18106f;
        // 依据：= 1.4 × kArmForwardM。

        // [1/s] 水平速度环比例增益。
        constexpr float kVelXyP = 0.8f;
        // 依据：新增（追加到 FlightParamId 末尾，纯追加安全）：τ≈1.25s；1 m/s 误差 → 0.8 m/s² → 4.7° 倾角，与 kVelTiltMaxRad=10° 
        // 匹配。

        // [1/s²] 水平速度环积分增益。
        constexpr float kVelXyI = 0.4f;
        // 依据：新增：Kp/Ki = 2s，能在 ~2s 内消掉常值风阻偏差。

        // [–] 水平速度环微分增益。
        constexpr float kVelXyD = 0.0f;
        // 依据：新增：不做微分 —— 光流噪声大（百年一遇的坏样本已被门限挡住，但残差仍是高频），微分只会把它放大。

        // [m/s] 水平速度环积分限幅。
        constexpr float kVelXyIntLimit = 0.6f;
        // 依据：新增：0.6 m/s × VelXyI → 0.24 m/s² = 1.4° 权限；积分只该干"配平风阻"这件事，不该攒出能翻飞机的权限。

        // [m/s] 水平速度指令上限。
        constexpr float kVelXyMaxMps = 2.0f;
        // 依据：新增：定速模式的手动上限；与 sensor.flow.kMaxVelMps=3.0 一起界定"合理速度"区间。

        // [rad] 速度环输出倾角限幅（10°）。
        constexpr float kVelTiltMaxRad = 0.17f;
        // 依据：新增：比 kTiltMaxRad(0.35) 紧 —— 速度环只在此倾角内工作，超出的机动交给手动模式（避免速度环为了追速度把飞机压到 20°）。

        // [m/s²] 水平加速度限幅。
        constexpr float kVelAccMaxMss = 1.5f;
        // 依据：新增：先于倾角限幅生效（加速度是倾角的积分前量），限制速度环的"猛拉"。

        // [s] 光流超时时间。
        constexpr float kVelFlowTimeoutS = 0.3f;
        // 依据：新增：超时 → 冻结积分 + 降级 ALT_HOLD（无观测时速度环积分会发散）；与 sensor.flow.kTimeoutS 同量级，前后端各兜一层。

        // 水平速度环（VEL_HOLD，目标速度 0 = 钉在原地）总开关。
        // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
        constexpr bool kVelHoldEnable = true;
        // 依据：M3(2026-10-05) 由 false 改成 true：原来关着是因为「定速只有水平速度环、没有高度环」—— 加半个高度环比不加更危险；现在 updateAngleHeight()
        //  + vz_ff 前馈把高度环接上了，这条理由不成立。**仍需先标定** sensor.flow.kSignX/kSignY/kYawDeg（符号反了速度环会把飞机往反方向推，欠账 #18）
        // 。失效是自动的：光流无数据/被门限拒/参数不自洽 ⇒ 无条件退回角度环 + 手动油门，不是靠这个开关兜底。
        // ⚠️ 待确认：见上一条依据。

    } // namespace control

    // 状态机 / 事件表 — H：4 维正交状态 + 健康位域 + 保护层；迁移表是数据不是 if。v2.1 起**事件表已删除**（异常改走 ExceptionRecorder + 统一出口），本组的 kEventSlotCount / kEventUnboundLog 随之作废。
    namespace state {

        // 迁移表放 flash（默认）还是 RAM（台架改表用）。
        constexpr bool kTransitionTableInRam = false;
        // 依据：新增：默认 flash 省 RAM；台架阶段想在不重编译的情况下改迁移规则才置 true。

        // 健康降级自动生效。
        constexpr bool kDegradeAuto = true;
        // 依据：新增：健康位变差即自动降级（§7.2 迁移规则是数据不是 if）。

        // 健康恢复后是否自动升回。
        constexpr bool kUpgradeAuto = false;
        // 依据：刻意默认关：降级后自动升回会让"时好时坏"的故障反复抖动（如接触不良的 I²C），要人确认才升。

        // FAULT 时锁死，只能复位。
        constexpr bool kFaultLatchOnReset = true;
        // 依据：新增：FAULT 属于"必须落地检查"的等级，自恢复等于把隐患带上天。

    } // namespace state

    // 日志 / 存储 — I：槽位规划、队列与布局版本。改 LogEntry 布局必须同步 +kLayoutVersion，否则上位机报 FLASH_GEOMETRY_MISMATCH。
    namespace log {

        // [槽] 飞行模式单次日志预留槽数（1 槽 = 128KB ≈ 885 条 ≈ 17.7s @50Hz）。
        // 仅当 cfg::feature::kLogToFlash 为 true 时存在(编译期整块裁掉)。
        constexpr uint32_t kFlightSlots = 32u;
        // 依据：沿用现有 kConfig.flightLogSlots = 32；32 槽 ≈ 4MB ≈ 9.4 分钟 @50Hz，W25Q128 有 16MB。

        // [槽] 日志管理模式预留槽数（最小 1）。
        // 仅当 cfg::feature::kLogToFlash 为 true 时存在(编译期整块裁掉)。
        constexpr uint32_t kManageSlots = 1u;
        // 依据：沿用现有 manageLogSlots = 1：管理模式要调 init() 才能读会话（ready 由 startSession 置位），必然建一个空会话，1 就是最小合法值（FlashManager 
        // 内部会 clamp 到 1~槽数）。

        // [条] 控制线程 → 日志线程队列深度。
        constexpr uint16_t kQueueDepth = 16u;
        // 依据：沿用现有 kConfig.logQueueDepth = 16；队列满即丢弃并计入 perf.drops（drops 会打到调试状态行里）。

        // [条] 日志线程一次排空最多取多少条。
        constexpr uint16_t kMaxEntriesPerRun = 24u;
        // 依据：新增：给"一次唤醒最多花多久"设上限，防止排空独占 CPU（控制环优先级 10 低于日志 30，日志不下让会拖控制周期）。

        // 布局不符时自动整片重建（会丢日志）。
        // 仅当 cfg::feature::kLogToFlash 为 true 时存在(编译期整块裁掉)。
        constexpr bool kAutoFormatOnMismatch = false;
        // 依据：沿用现有 autoFormatOnMismatch = false：飞行中自动格式化等于把上次事故的日志抹掉。

        // 写满后的策略：Halt（停止记录）或 OverwriteOldest（覆盖最老会话）。
        // 仅当 cfg::feature::kLogToFlash 为 true 时存在(编译期整块裁掉)。
        constexpr LogFullPolicy kFullPolicy = LogFullPolicy::Halt;
        // 依据：沿用现有行为（写满即停 + 置 kLogFlagLogFull），抽象成枚举；飞行中擦除会带来几十秒的写阻塞（擦 64KB 块），不可接受。

        // [–] LogEntry 布局版本。**本版从 1 升到 2**：追加了光流/水平速度字段。版本变了旧日志会报 FLASH_GEOMETRY_MISMATCH，只能靠日志管理模式 op=5 
        // 整片格式化恢复。
        // 仅当 cfg::feature::kLogToFlash 为 true 时存在(编译期整块裁掉)。
        constexpr uint16_t kLayoutVersion = 2u;
        // 依据：2026-10-02 确认：LogEntry 与配置同步升级（加水平速度观测字段），所以布局版本必须 +1，否则新固件会把旧日志按新布局解析成垃圾。

        // LogEntry 是否包含光流/水平速度字段（v2 布局）。**本版为 true**：追加 vx/vy（EKF 世界系水平速度）、flow_dx/flow_dy/flow_squal、range_m、
        // flow_valid 标志。
        // 仅当 cfg::feature::kLogToFlash 为 true 时存在(编译期整块裁掉)。
        constexpr bool kEntryHasFlowFields = true;
        // 依据：2026-10-02 确认『LogEntry 应当同步升级』。新增字段约 +28 字节 ⇒ LogEntry 132→160 字节业务区，步长 148→176 字节，单槽 885→744 
        // 条（50Hz 下约 14.9 分钟/槽，32 槽约 8 小时）。

        // [µs] loopMax 超过此值给一次告警。
        constexpr uint32_t kPerfWarnUs = 800u;
        // 依据：新增：上位机现在硬编码 1600µs 告警线（500Hz ⇒ 2000µs 预算的 80%）；控制环瘦身（总线操作全部移出）后最坏 slot 应显著下降，告警线重定到 800µs（40%）
        // ，这样"变慢"能在早期被发现。

    } // namespace log

    // ADC / 电源保护 — J：电压分压比 9.0（8k/1k，PA0）；电流直连 PA1（无分压）⇒ 零点与斜率必须由传感器型号或台架两点标定确认。
    namespace power {

        // [mV] ADC 参考电压（= VDDA）。
        // 仅当 cfg::feature::kAdc 为 true 时存在(编译期整块裁掉)。
        constexpr uint16_t kVrefMv = 3300u;
        // 依据：§4.5 建议值；换算 V_adc = raw × kVrefMv / 4096（12 位）。

        // [–] 电池电压分压比：V_bat = V_adc × ratio。
        // 仅当 cfg::feature::kAdc 为 true 时存在(编译期整块裁掉)。
        constexpr float kVoltageDividerRatio = 9.0f;
        // 依据：改：本板 PA0 外部实为 8k（上）/ 1k（下）⇒ ratio = (8+1)/1 = 9.0，不是 §4.5 建议电路的 100k/22k = 5.545。满量程 3.3V 
        // × 9 = 29.7V；**电池是 6S**（用户 2026-10-05 确认）⇒ 满充 25.2V 落在量程 85% 处，有余量；分压阻抗 9k，25.2V 时约 2.8mA，可接受。
        // （原文里的『4S 16.8V 落在 57% 处 / 1.9mA』按 4S 写的，已按 6S 更正。）

        // [mV] 分压零点修正。
        // 仅当 cfg::feature::kAdc 为 true 时存在(编译期整块裁掉)。
        constexpr int16_t kVoltageOffsetMv = 0;
        // 依据：§4.5 建议 0；用万用表比对一次读数后可写入偏差（例如分压电阻公差带来的固定偏置）。

        // [V] 电流传感器零点电压（0A 时的输出）。
        // 仅当 cfg::feature::kAdc 为 true 时存在(编译期整块裁掉)。
        constexpr float kCurrentOffsetV = 1.65f;
        // 依据：仍未确认：1.65V 是"3.3V 供电、双向测量、零点取 VDDA/2"的典型值（§4.5）。**灵敏度已经确定是 12.5mV/A（2026-10-02 用户提供）**，零点仍建议台架两点标定覆盖：
        // 0A 时读一次 → 零点；已知负载读一次 → 斜率。⚠️ PA1 直连没有分压，输出必须留在 0..VDDA(3.3V) 内。
        // ⚠️ 待确认：见上一条依据。

        // [A/V] 电流换算斜率：I[A] = (V_adc − kCurrentOffsetV) × kCurrentAmpsPerVolt。**80 A/V = 1/0.0125，即 12.5mV/A**（2026-10-02 
        // 用户提供，已确认）。
        // 仅当 cfg::feature::kAdc 为 true 时存在(编译期整块裁掉)。
        constexpr float kCurrentAmpsPerVolt = 80.0f;
        // 依据：25 → **80**（2026-10-02）：用户给出电流计是 **12.5mV/A**，所以斜率 = 1/0.0125 = 80 A/V（原来按 40mV/A 猜的 25 A/V 
        // 偏保守 3.2 倍，会把实际电流读小三倍多）。**量程核对**：零点 1.65V 时正向可读到 (3.3−1.65)×80 = **132 A**，1.65V 负向同理 —— 远超这条动力系统的实际电流，
        // 且用户确认模块输出不会超过 3.3V，所以 PA1 直连安全。量程这么宽也意味着**小电流分辨率不高**（12 位 ADC 约 0.8mV/LSB ⇒ 每 LSB ≈ 0.065 A），
        // 够看趋势、不够算 mAh 精确值。

        // [节] 电池节数（6S）。只影响 perCellVolts() 的除数（每节电压 = 整包 / 节数），分级保护的三个阈值本身就是『每节』，换电池不用改阈值。
        // 仅当 cfg::feature::kAdc 为 true 时存在(编译期整块裁掉)。
        constexpr uint8_t kCellCount = 6u;
        // 依据：**用户 2026-10-05 确认：电池是 6S**（这是唯一真相来源；导入时的『4S』是错的，已于同日更正）。量程核对：分压比 9.0 ⇒ 满量程 9 × 3.3 = 29.7V，
        // 6S 满充 25.2V（4.2V/cell）落在量程内、留约 15% 余量；换 4S(16.8V) 反而只用掉 57% 量程。注意**导出值一度是 6 而声明默认是 4**，两者不一致让 
        // kmpFly 的『导出的头文件与下位机现有头文件逐项一致』长期报 `导出 4u, 参考 6u` —— 本次一并纠正。低压保护尚未实现，所以这一项目前只被 `ADC1Sampler::perCellVolts()
        // ` 使用（暂无调用者）。

        // [mV] 单节电压低于此值 → WARN。
        // 仅当 cfg::feature::kAdc 为 true 时存在(编译期整块裁掉)。
        constexpr uint16_t kWarnPerCellMv = 3500u;
        // 依据：§4.5/§3.9 的低压保护分级：WARN（3.5V/cell）→ DEGRADE（3.4V/cell）→ HOVER_LAND（3.3V/cell）。

        // [mV] 单节电压低于此值 → DEGRADE。
        // 仅当 cfg::feature::kAdc 为 true 时存在(编译期整块裁掉)。
        constexpr uint16_t kDegradePerCellMv = 3400u;
        // 依据：同上。

        // [mV] 单节电压低于此值 → HOVER_LAND。
        // 仅当 cfg::feature::kAdc 为 true 时存在(编译期整块裁掉)。
        constexpr uint16_t kLandPerCellMv = 3300u;
        // 依据：同上；3.3V 是 LiPo 的空载下限附近，带载时已接近危险区。

        // 用电流积分估算已消耗容量（回传/日志用）。
        // 仅当 cfg::feature::kAdc 为 true 时存在(编译期整块裁掉)。
        constexpr bool kMahEstimate = true;
        // 依据：§4.5；注意是"估算"：零点/斜率未标定前，这个数字只能当趋势看（见 kCurrentOffsetV 的未确认标记）。

    } // namespace power

    // 地面模式 / 维护 — K：跳线帽 + 上位机“解锁”命令的模式判定。日志管理模式永不输出电机。
    namespace ground {

        // [ms] 等"解锁"命令的超时，0 = 一直等。
        constexpr uint32_t kWaitTimeoutMs = 0u;
        // 依据：沿用现有 groundWaitTimeoutMs = 0。

        // [ms] 等命令时状态灯闪烁周期。
        constexpr uint32_t kBlinkMs = 120u;
        // 依据：沿用现有 groundBlinkMs = 120。

        // [ms] 日志管理模式慢闪周期。
        constexpr uint32_t kManageBlinkMs = 500u;
        // 依据：沿用现有 manageBlinkMs = 500：与"等待"的 120ms 快闪区分开，一眼能看出当前在哪个模式。

        // 超时后进入的模式（安全，不动电机）。
        constexpr GroundFallbackMode kFallbackMode = GroundFallbackMode::LogManage;
        // 依据：沿用现有语义（超时默认进日志管理模式）；enum 值对齐 dlx::BootMode（Flight=0 / SerialDebug=1 / LogManage=2），保证不动电机。

    } // namespace ground

    // 线程 — L：优先级 / 栈。新线程走静态栈并放 CCM（0x1000_0000，64KB 现完全未用；DMA 不碰栈，安全）。
    namespace thread {

        // [优先级] 控制环线程优先级。
        constexpr uint8_t kMainPrio = 10u;
        // 依据：沿用现有：主线程 10 级（数字越小越高），是最高的应用线程 —— 总线操作移出后它不再被阻塞。

        // [优先级] 链路线程（E220 时分调度）优先级。
        // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
        constexpr uint8_t kLinkPrio = 24u;
        // 依据：upgrade_plan §5.1：原 nrf_tel 24 级 → link_tel，优先级不变。

        // [优先级] 气压计 + ADC 采集线程优先级。
        constexpr uint8_t kIoPrio = 26u;
        // 依据：新增（§5.1）：低于链路、高于日志；I²C 读气压 ~1ms，必须在控制环之外。

        // [优先级] 测距线程优先级。
        // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
        constexpr uint8_t kRangePrio = 27u;
        // 依据：新增（§5.1）：主要由 EXTI3 唤醒，优先级只需高于日志。

        // [优先级] 光流线程优先级。
        // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
        constexpr uint8_t kFlowPrio = 28u;
        // 依据：新增（§5.1）：10ms 周期（100Hz），SPI 1.31MHz 读几个字节，时间很短。

        // [优先级] GNSS 解析线程优先级。
        // 仅当 cfg::feature::kGnss 为 true 时存在(编译期整块裁掉)。
        constexpr uint8_t kGnssPrio = 29u;
        // 依据：新增（§5.1）：纯解析 + 低频，最低的应用优先级。

        // [优先级] 日志线程优先级。
        constexpr uint8_t kLogPrio = 30u;
        // 依据：沿用现有 logger 30 级：最低，任何业务线程都能抢占它。

        // [B] 控制环线程栈（从 RT-Thread 堆分配）。
        constexpr uint32_t kMainStackBytes = 24576u;
        // 依据：沿用现有：栈用量实测 ≈19.0KB，24576 有 ~5KB 余量。

        // [B] 链路线程静态栈。
        // 仅当 cfg::feature::kLink 为 true 时存在(编译期整块裁掉)。
        constexpr uint32_t kLinkStackBytes = 3072u;
        // 依据：新增（§5.1）：从 RT-Thread 堆改静态栈 —— RT_HEAP 只有 40KB 且已用 ~31KB，新增 5 个线程不能抢堆。

        // [B] io 线程静态栈。
        constexpr uint32_t kIoStackBytes = 2048u;
        // 依据：新增（§5.1）；I²C + ADC + DMA 调用，2048 足够（无浮点大数组）。

        // [B] 测距线程静态栈。
        // 仅当 cfg::feature::kRange 为 true 时存在(编译期整块裁掉)。
        constexpr uint32_t kRangeStackBytes = 2048u;
        // 依据：新增（§5.1）。

        // [B] 光流线程静态栈。
        // 仅当 cfg::feature::kFlow 为 true 时存在(编译期整块裁掉)。
        constexpr uint32_t kFlowStackBytes = 2048u;
        // 依据：新增（§5.1）。

        // [B] GNSS 线程静态栈。
        // 仅当 cfg::feature::kGnss 为 true 时存在(编译期整块裁掉)。
        constexpr uint32_t kGnssStackBytes = 2048u;
        // 依据：新增（§5.1）；NMEA 解析缓冲不大（一句 ≤82B），但如果将来加 UBX 二进制解析要重新算。

        // [B] 日志线程栈（从堆分配）。
        constexpr uint32_t kLogStackBytes = 3072u;
        // 依据：沿用现有 logger 3072B；注意它仍是堆栈（历史原因），若要统一放 CCM 需一并改。

        // 新线程的静态栈是否放 CCM（0x1000_0000，64KB，现完全未用）。
        constexpr bool kStaticStackInCcm = true;
        // 依据：新增（§5.1）：CCM 不被 DMA 访问，而栈本来就不该给 DMA 用 ⇒ 放 CCM 完全安全，且 64KB 目前一点没用。

        // [B] RT-Thread 堆大小（board.c 里配置）。
        constexpr uint32_t kHeapBytes = 40960u;
        // 依据：沿用现有 40KB；新线程改静态栈后不必再增大（现在是"已用 ~31KB"的紧张状态）。

    } // namespace thread

    // 自检报告 / 诊断 — M：回读校验“写进去的是不是真生效”。飞行模式串口必须静音，详细自检只在调试模式打印。
    namespace diag {

        // 详细自检只在串口调试模式打印。
        constexpr bool kFullReportInDebugOnly = true;
        // 依据：沿用现有语义：飞行模式的串口必须静音（否则地面站/日志流被文本污染）。

        // 串口调试模式是否每 kStackWatermarkMs… — 串口调试模式是否每 kStackWatermarkMs 打一行线程栈水位线（量栈预算时临时开）。
        constexpr bool kLogStackWatermark = false;
        // 依据：2026-10-02 首次烧录时临时打开: `FlightStack`(含 2.7KB 日志队列)刚被放上 24KB 主线程栈, 每 10s 一行实测栈用量把那条风险直接验掉了(结果记在 
        // docs/2026-10-02_upgrade_progress.md)。**2026-10-04 关掉**: 问题已闭环, 而它每 10s 仍占一行串口 —— 量栈这种事属于'要用时临时开', 
        // 不该常驻。要再量就把本项改回 true 重编。

        // 上电读 RCC_CSR 复位原因并清标志。
        constexpr bool kLogResetCause = true;
        // 依据：沿用现有；复位原因（看门狗/掉电/软复位）是排查"莫名其妙重启"的第一条线索，而 CSR 里的标志读一次就该清。

        // 打印 SYSCLK/HCLK/PCLK1/PCLK2。
        constexpr bool kLogClockTree = true;
        // 依据：沿用现有；外设速率算错（BR 码在 APB1/APB2 上不同）几乎都源于时钟树与预期不符。

        // 回读 SPI_CR1.BR 与期望值对比。
        constexpr bool kVerifySpiPrescaler = true;
        // 依据：沿用现有；同一个 BR 码在 APB1(42MHz) 与 APB2(84MHz) 上得到的实际时钟差一倍 —— 这正是光流"读回全 0"的候选原因之一。

        // 回读 BRR 反算实际波特率。
        constexpr bool kVerifyUsartBrr = true;
        // 依据：沿用现有；2Mbaud 调试口与 57600 链路口的容差不同，反算能立刻发现"分频比取整错了"。

        // setDshot() 之前打印期望的 ARR/PSC。
        constexpr bool kVerifyDshotTiming = true;
        // 依据：沿用现有：算不出预分频时 setDshot() 会 while(1) 死等，打一行期望值就能定位（现象是"上电就卡死"）。

        // E220 配置回读校验。
        constexpr bool kVerifyE220Regs = true;
        // 依据：沿用现有思路（见 link.e220.kVerifyRegs）；两处都开是为了让"模块没配上"在自检阶段就暴露，而不是等到空中丢链路。

        // 回读各传感器 ID：BMI088(0x1E/0x0F)、BMP388(0x50)、PMW3901(0x49,0xB6)、VL53L1X(0xEA,0xCC,0x10)。
        constexpr bool kVerifySensorIds = true;
        // 依据：§0.1 事实 9：BMP388 的 CHIP_ID 在 0x00、值是 0x50（BME280 是 0xD0→0x60）—— 旧探针在 BMP388 上读到的是保留寄存器，所以 
        // ID 校验必须按新器件重写。

    } // namespace diag

    // ----------------------------------------------------------------------------------------------
    // 本版已删除、因此不生成的项(保留记录,避免以后有人以为是漏了):
    //   cfg::hw::imu::kNssPin —— SPI1 的 NSS 脚。**本版已删除**：BMI088 的片选是 kAccelCs/kGyroCs 两个独立 GPIO（PB12/PB13），这个脚从来没被真正当过片选。B4 给 `SPI` 加了「无片选」构造（`SPI1_SA5_MIA6_MOA7()` 不带 nss 参数，`select()/deselect()` 变成空操作），PA3 释放给 VL53L1X 的 GPIO1。
    //   cfg::hw::baro::kSdo —— （本版已删除）BMP388 的 SDO 由**板上直接焊到 GND**，MCU 不再驱动任何引脚 ⇒ 地址固定 0x76。
    //   cfg::state::kEventSlotCount —— （v2.1 已删除）事件表的定长槽数。v2.1 把 `DL_LIB/dlx_event_table.hpp` + `app_events.hpp` 整体删除：异常改由 **ExceptionRecorder（栈上、按作用域）+ app_exception_handler 的统一出口**处理，不再需要『给错误码绑定处理器』这一层，因此也没有『表满』与『槽数』。
    //   cfg::state::kEventUnboundLog —— （v2.1 已删除）『未绑定的错误码要不要打日志』。它存在的前提是事件表里**可能有没有处理器的码**；v2.1 只剩一个统一出口，不存在『没人管』的码，所以这一项连同它的告警路径一起消失。
    // ----------------------------------------------------------------------------------------------

    // feature 开关的预处理器镜像(与上面的 constexpr 同源,仅供 #if 使用)
#define CFG_FEATURE_BARO             1
#define CFG_FEATURE_RANGE            1
#define CFG_FEATURE_FLOW             1
#define CFG_FEATURE_GNSS             0
#define CFG_FEATURE_ADC              1
#define CFG_FEATURE_LINK             1
#define CFG_FEATURE_LOG_TO_FLASH     1
#define CFG_FEATURE_VERBOSE_BOOT_LOG 1

} // namespace cfg

