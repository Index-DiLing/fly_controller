#pragma once
//===================================================================================================
// DL_LIB/BMP388/dlx_bmp388_config.h —— BMP388 气压计的"寄存器约定 + 工作参数"
//
// 职责:
//   1. 把数据手册里的寄存器地址、位定义写成有名字的常量(驱动里不出现裸魔数);
//   2. 把**工作参数**从唯一的真相源 cfg:: 取过来(BMP388 自己的数值这里一个都不写);
//   3. 用 static_assert 在**编译期**挡住"配置项填进了保留位"这类错误。
//
// 与 BME280 的差异(为什么不能复用 dlx_bme280_config.h):
//   BME280 的枚举值语义是"直接写进某个寄存器的字节"(CtrlMeas=0x93 这种), 而 BMP388 的
//   四个配置寄存器(OSR/ODR/CONFIG/PWR_CTRL)是**分开的四个字节**, PWR_CTRL 还要把 mode 与
//   press_en/temp_en 拼起来 —— 沿用 BME280 那种"一个枚举 = 一个寄存器值"的写法只会让人以为
//   OSR/ODR 是同一个寄存器。所以这里只留寄存器地址 + 位掩码, 数值全部由 cfg:: 提供。
//
// 依据: Bosch BMP388 数据手册 BST-BMP388-DS001-07 Rev 1.7 (§3.11.1 Table 23 标定系数表,
//       §4.3 寄存器表, §9 Appendix 补偿公式)。本节数值均已与本仓库的 PDF 原文逐条核对。
//===================================================================================================
#include <stdint.h>
// 为什么不是 "flight_config.hpp": 那一层是给固件用的胶水(派生量 + 日志布局契约), 会把
// W25Q128 的 flash 结构体一起拖进任何一个 include 了本文件的编译单元; 传感器驱动只读 cfg
// 的常量树, 直接吃生成文件即可。生成文件本身由 config/flight_config.{schema.json,json} 负责。
#include "generated/flight_config_gen.hpp"

namespace dlx
{
    /**
     * @brief BMP388 7 位 I2C 地址
     *
     * 手册 §5.1: SDO 接 GND = 0x76, SDO 接 VDDIO = 0x77。
     * 本板 SDO 已被硬件钉到 GND(cfg::hw::baro::kAddr 的 evidence), 所以恒为 0x76。
     * 为什么不在这个头文件里写 0x76: 地址是"接线决定"的参数, 属于配置; 驱动里写死一个
     * 数字, 换板子时就要改驱动 —— 改配置才是对的。
     */
    constexpr uint8_t BMP388_IIC_ADDRESS = cfg::hw::baro::kAddr;

    /**
     * @brief BMP388 寄存器地址(手册 §4.3 寄存器映射表)
     *
     * 注意与 BME280 的对照(这是最容易被"照抄旧驱动"害死的地方):
     *   - CHIP_ID 在 **0x00**, 值是 **0x50**; BME280 在 0xD0 且值是 0x60。
     *     0xD0 在 BMP388 上是**保留寄存器**, 去读它只会得到一个无意义的值, 更不能当 ID 用。
     *   - 软复位在 **0x7E**(写 0xB6); BME280 在 0xE0。
     *   - 数据寄存器在 0x04~0x09(24bit 无符号); BME280 在 0xF7~0xFE(20bit 带 4bit 填充)。
     */
    enum class BMP388_Reg : uint8_t
    {
        ChipId   = 0x00, ///< 芯片 ID, BMP388 恒为 0x50
        ErrReg   = 0x02, ///< [2]conf_err [1]cmd_err [0]fatal_err(读清零)
        Status   = 0x03, ///< [6]drdy_temp [5]drdy_press [4]cmd_rdy(1=就绪)
        DataStart = 0x04, ///< 数据区起址: 0x04~0x09 = press(3B) + temp(3B), 小端
        Event    = 0x10, ///< [0]por_detected(读清零)
        IntStatus = 0x11, ///< [3]drdy [1]ffull_int [0]fwm_int(读清零)
        IntCtrl  = 0x19, ///< [6]drdy_en [4]ffull_en [3]fwtm_en [2]latch [1]level [0]od
        IfConf   = 0x1A, ///< [2]i2c_wdt_sel [1]i2c_wdt_en [0]spi3 —— 保持默认 0x00
        PwrCtrl  = 0x1B, ///< [5:4]mode [1]temp_en [0]press_en
        Osr      = 0x1C, ///< [5:3]osr_t [2:0]osr_p(0=×1, 1=×2, 2=×4, 3=×8, 4=×16, 5=×32)
        Odr      = 0x1D, ///< [4:0]odr_sel, 采样周期 = 5ms × 2^n(0=200Hz, 2=50Hz, 17=最慢)
        Config   = 0x1F, ///< [3:1]iir_filter(0/1/3/7/15/31/63/127), 其余位保留
        CalibStart = 0x31, ///< 标定系数起址: 0x31~0x45 共 **21 字节**(见下面 BMP388_CALIB_LEN)
        Cmd      = 0x7E, ///< 命令口: 0x00 nop / 0xB0 fifo_flush / 0xB6 softreset。**恒回读 0x00**
    };

    // --------------------------------------------------------------------------------------------
    // 常量: 手册里写死的那些"器件性质", 不是工作参数
    // --------------------------------------------------------------------------------------------

    /** CHIP_ID(0x00) 的期望值。为什么写成常量而不是在驱动里写 0x50: 见 BMP388_Reg::ChipId 注释 */
    constexpr uint8_t BMP388_CHIP_ID_VALUE = 0x50;

    /** 写入 CMD(0x7E) 触发软复位(手册 §4.3.29) */
    constexpr uint8_t BMP388_CMD_SOFT_RESET = 0xB6;

    /**
     * 软复位后的等待时间 [ms]。
     *
     * 手册没有给"复位完成"的显式时序, 但给了上电后**首次通信**需要 2ms(§5 Table 2 Start-up time),
     * 且 §3.2 明确软复位会把所有用户配置恢复默认 —— 所以写 0xB6 之后必须等器件重新就绪再读配置。
     * 为什么是 2ms 而不是更长: 这一段是**阻塞**等待, 气压计的 init 在开机路径上, 每多 1ms 都是
     * 开机时间; 2ms 是手册给的器件下限, 再长没有依据(ArduPilot 用 10ms, 属于留余量, 不是手册要求)。
     * 为什么不用 EVENT.por_detected 轮询代替: 那要多一次 I2C 事务, 而事务本身的耗时在总线上
     * 比 2ms 更容易受从机时钟拉伸影响, 得不偿失。
     */
    constexpr uint32_t BMP388_SOFT_RESET_WAIT_MS = 2u;

    /** 标定系数块长度: 0x31~0x45 = 21 字节(手册 §3.11.1 Table 23) */
    constexpr uint8_t BMP388_CALIB_LEN = 21u;

    /**
     * 数据突发读长度: STATUS(0x03) + 6 字节数据(0x04~0x09) = 7。
     * 为什么把 STATUS 一起读进来: 手册 §3.9.2 明确"必须一次突发读完数据寄存器, 分几次读可能
     * 读到不一致的数据"(阴影寄存器只在一次突发内保证原子)。顺手多读 1 个状态字节, 就能判断
     * "这份数据是新的还是上一次的", 而且状态与数据来自同一个事务 —— 分成两次读会出现
     * "新状态配旧数据"的错配。
     */
    constexpr uint8_t BMP388_DATA_BURST_LEN = 7u;

    /** STATUS(0x03) 位: 温度数据就绪 */
    constexpr uint8_t BMP388_STATUS_DRDY_TEMP = 0x40u;
    /** STATUS(0x03) 位: 气压数据就绪(判"这一轮有没有新样本"就看它) */
    constexpr uint8_t BMP388_STATUS_DRDY_PRESS = 0x20u;

    /**
     * PWR_CTRL(0x1B) 位定义。bit4/bit5 才是 press_en/temp_en —— 这是 BMP388 与 BME280
     * (通道使能在 ctrl_meas 的过采样位里)最大的写法差异, 错一位的表现是"能读 ID 但永远没有新数据"。
     */
    constexpr uint8_t BMP388_PWR_CTRL_PRESS_EN = 0x01u; ///< [0] press_en
    constexpr uint8_t BMP388_PWR_CTRL_TEMP_EN  = 0x02u; ///< [1] temp_en
    constexpr uint8_t BMP388_PWR_CTRL_MODE_NORMAL = 0x30u; ///< [5:4]=11 连续转换

    /**
     * 实际写进 PWR_CTRL(0x1B) 的值(mode | temp_en | press_en)。
     *
     * 为什么这里要"拼"而不是从 cfg 直接取: config/flight_config.schema.json 的 sensor.baro
     * 目前只有 kOsr/kOdr/kConfig/kIntCtrl/kInitRetry/kInitRetryMs/kFailLimit/kTimeConstS,
     * **没有 kPwrCtrl/kMode 这一项**(已在交付报告里列为缺失配置项, 由 Lead 决定是否加)。
     * 在配置项补上之前, 这里按手册 §3.8 的位定义拼出 0x33(连续模式 + 温度 + 气压), 与手册
     * Table 10 "Drone" 行和 ArduPilot 的取值一致; 一旦 cfg 补了 kPwrCtrl, 只改这一行即可。
     */
    constexpr uint8_t BMP388_PWR_CTRL_VALUE =
        static_cast<uint8_t>(BMP388_PWR_CTRL_MODE_NORMAL | BMP388_PWR_CTRL_TEMP_EN | BMP388_PWR_CTRL_PRESS_EN);

    /** IF_CONF(0x1A) 的期望值: 软复位默认 0x00 = 4 线 SPI 位关闭 + **I2C 看门狗关闭** */
    constexpr uint8_t BMP388_IF_CONF_VALUE = 0x00u;

    /**
     * 配置回读的起始地址与长度: 0x19(INT_CTRL) ~ 0x1D(ODR) 共 5 字节。
     * 为什么读取到这里为止、不等长读到 0x1F(CONFIG): 0x1E 是**保留寄存器**, 写它手册明令禁止,
     * 读它手册没保证 —— 为了省一次 I2C 事务去踩一个没有依据的地址不值得, 所以 CONFIG 单独读。
     */
    constexpr uint8_t BMP388_CFG_READBACK_ADDR = 0x19u;
    constexpr uint8_t BMP388_CFG_READBACK_LEN  = 5u;

    // --------------------------------------------------------------------------------------------
    // 编译期守卫: 配置项只能落在手册定义的位域里
    // 为什么要有: OSR 只有 bits[5:0]、ODR 只有 bits[4:0]、CONFIG 只有 bits[3:1]; 多写一位就是
    // 往保留位里写数据, 手册禁止(§4.3.29 "Do not write reserved registers"), 而硬件的表现是
    // **不报错、不生效** —— 上板后表现为"配置回读不一致"或"噪声比预期大", 极难定位。
    // 有了这几条断言, 配置改错的后果是编译不过, 而不是飞起来才发现。
    // --------------------------------------------------------------------------------------------
    static_assert((cfg::sensor::baro::kOsr & 0xC0u) == 0u,
                  "sensor.baro.kOsr 越界: OSR(0x1C) 只有 bits[5:3]osr_t 与 bits[2:0]osr_p");
    static_assert((cfg::sensor::baro::kOdr & 0xE0u) == 0u,
                  "sensor.baro.kOdr 越界: ODR(0x1D) 只有 bits[4:0]odr_sel");
    static_assert(cfg::sensor::baro::kOdr <= 0x11u,
                  "sensor.baro.kOdr 超出 odr_sel 有效范围 0..17(更大的值会被硬件饱和到 17)");
    static_assert((cfg::sensor::baro::kConfig & 0xF1u) == 0u,
                  "sensor.baro.kConfig 越界: CONFIG(0x1F) 只有 bits[3:1]iir_filter");
    static_assert((cfg::sensor::baro::kIntCtrl & 0xA0u) == 0u,
                  "sensor.baro.kIntCtrl 越界: INT_CTRL(0x19) 的 bit7/bit5 是保留位(有效位是 6,4,3,2,1,0)");
    static_assert(cfg::sensor::baro::kInitRetry >= 1u,
                  "sensor.baro.kInitRetry 至少为 1: init() 至少要试一次, 否则永远初始化不了");
    static_assert(cfg::sensor::baro::kFailLimit >= 1u,
                  "sensor.baro.kFailLimit 至少为 1: 否则第一次读失败就被判成'长时间无数据'");
} // namespace dlx
