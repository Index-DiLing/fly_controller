#pragma once
#include "flight_config.hpp"
#include "dlx_gpio_profile.h"

//===================================================================================================
// hardware_claims.hpp —— **接线事实**的编译期断言（应用层，不属于 DL_LIB）
//
// 为什么这些检查不放在 DL_LIB 里
//   DL_LIB 是可复用驱动层，它的规矩是"只接受参数，不认识 cfg::"。一旦库里
//   `#include "flight_config.hpp"` 再写 `static_assert(cfg::hw::adc::kVoltageCh == 0, ...)`，
//   换一块板子、或者想把这个驱动复用到另一个工程，就得改**库文件**；而且库和某个工程的
//   schema 从此绑死 —— 这不是洁癖，是"库还能不能复用"的问题。
//
//   于是分工变成：
//     DL_LIB/xxx.hpp        —— "给我参数，我按手册把外设配好"（不知道引脚接在哪）
//     hardware_claims.hpp   —— "这块板子上，哪个引脚接哪个外设"（全部集中在这里）
//   本文件是整个工程里**唯一**允许把 cfg:: 的引脚/通道和具体外设绑在一起的地方。
//
// 为什么这些错误值得用 static_assert 拦
//   它们全都是"不报错但读数看起来像那么回事"的那一类：通道号填错只会读到另一个引脚（或悬空），
//   I2C 引脚填错只会读不到器件，SPI CS 填错只会在别人说话时把总线抢过来。这些错**上板才会发现**，
//   而且常常先被当成"传感器坏了"。写在编译期，代价是零。
//
// 加一条断言的门槛很低，但要记住：**断言里必须能写清楚"为什么是这两个数"**，
// 否则以后改硬件的人只会把它删掉（而不是去改对）。
//===================================================================================================

namespace dlx
{
    namespace claims
    {
        /** GPIOProfile 的低 4 位 = 引脚号 */
        constexpr uint8_t pinNo(GPIOProfile p) { return static_cast<uint8_t>(p) & 0x0F; }
        /** GPIOProfile 的高 4 位 = 端口(A=0) */
        constexpr uint8_t portNo(GPIOProfile p) { return static_cast<uint8_t>(static_cast<uint8_t>(p) >> 4); }

        /**
         * @brief 由 ADC 通道号推出引脚(A0 + ch)
         *
         * 为什么可以直接加: F4 的 ADC1_INx 在硅片上就是 PAx(IN0=PA0 … IN15=PA15), 而
         * `GPIOProfile` 的低 4 位恰好就是引脚号、高 4 位是端口号(A=0) —— 这不是巧合, 是
         * dlx_gpio_profile.h 的定义方式。所以"通道号 → 引脚"是一次**纯计算**, 不需要配置项。
         *
         * 为什么值得写成一个函数而不是各写一遍: ADC 驱动收的是两个 GPIOProfile, 而配置里写的是
         * 两个**通道号**。中间这层转换写错一次(例如把 A0 写成 A1), 症状是"电压表读的是电流" ——
         * 两个数都像模像样, 只有对着接线图才能发现。
         */
        constexpr GPIOProfile adcPinOfChannel(uint8_t ch)
        {
            return static_cast<GPIOProfile>(ch); // 端口 A(高 4 位 = 0) + 通道号(低 4 位)
        }

        // 电压 = PA0 / 电流 = PA1(上面两条 static_assert 已经钉死了通道号, 这里再钉一次引脚换算)
        constexpr GPIOProfile kAdcVoltagePin = adcPinOfChannel(cfg::hw::adc::kVoltageCh);
        constexpr GPIOProfile kAdcCurrentPin = adcPinOfChannel(cfg::hw::adc::kCurrentCh);
        static_assert(portNo(kAdcVoltagePin) == 0 && pinNo(kAdcVoltagePin) == 0,
                      "电压通道必须落在 PA0: adcPinOfChannel 只对 ADC1_IN0..IN15(A0..A15) 成立");
        static_assert(portNo(kAdcCurrentPin) == 0 && pinNo(kAdcCurrentPin) == 1,
                      "电流通道必须落在 PA1");

        //===========================================================================================
        // 一、ADC —— 通道号与引脚在芯片里是硬绑定的
        //===========================================================================================
        // ADC1_IN0 物理上就是 PA0、IN1 就是 PA1。配置里写别的通道号不会报错，只会读到另一个引脚。
        static_assert(cfg::hw::adc::kVoltageCh == 0, "电压必须是 ADC1_IN0(PA0): 方案 §4.5 接线表(8k/1k 分压)");
        static_assert(cfg::hw::adc::kCurrentCh == 1, "电流必须是 ADC1_IN1(PA1): 方案 §4.5 接线表(直连)");
        static_assert(cfg::hw::adc::kSampleCount >= 1 && cfg::hw::adc::kSampleCount <= 64,
                      "平均次数 1~64; 再多应该用硬件过采样或换算法");
        static_assert(cfg::power::kVrefMv > 0, "VREF 必须为正");
        static_assert(cfg::power::kVoltageDividerRatio > 1.0f,
                      "分压比 >1: 电池电压经分压后才进 ADC; 填 1.0 会读出 1/9 的假电压");
        static_assert(cfg::power::kCurrentAmpsPerVolt > 0.0f, "电流灵敏度必须为正");

        //===========================================================================================
        // 二、EXTI 中断线 —— 引脚号直接决定线号, 线号决定中断向量
        //===========================================================================================
        // VL53L1X 的 GPIO1 是"数据就绪"输出。它接在哪个引脚上, 就落在哪条 EXTI 线上,
        // 而 EXTI3 是独立向量、EXTI5..9 与 10..15 各自共用 —— 接错引脚会让 B6 的 range 线程
        // 永远等不到中断, 表现是"测距一直没数据"。
        static_assert(pinNo(cfg::hw::range_::kInt) == 3,
                      "VL53L1X GPIO1 必须接在 PA3(EXTI3): stm32f4xx_it.cpp 里 EXTI3 有独立向量; "
                      "换引脚必须同时确认向量没和别人共用");
        static_assert(portNo(cfg::hw::range_::kInt) == 0, "同上: 端口必须是 A");

        /**
         * PD2 的所有权: **VL53L1X 的 XSHUT**, 不是 BMP388 的 SDO。
         *
         * 2026-10-02 的决定: 板上把 BMP388 的 SDO **直接焊到 GND**(固定地址 0x76), 而不是像原来
         * 的 BME280 那样由 MCU 驱动 PD2 为低 —— MCU 驱动的做法在复位期间会浮空, 地址会在
         * 0x76/0x77 之间翻, 症状是"有时能读到、有时读不到"。焊死以后 PD2 就空出来了, 正好给
         * VL53L1X 的 XSHUT(XSHUT 在上电期间必须是低电平 = 硬件待机)。
         *
         * `hw.baro.kSdo` 因此已从 schema 标 removed —— 不是"这个脚不用了", 而是"这个脚**归别人**了",
         * 留着它一定会有人照着接。
         */
        static_assert(cfg::hw::range_::kXshut != cfg::hw::baro::kScl &&
                          cfg::hw::range_::kXshut != cfg::hw::baro::kSda,
                      "XSHUT(PD2) 与气压计 I2C 引脚不能撞车");

        //===========================================================================================
        // 三、串口 —— USARTx 的默认 DMA 流/通道是按外设号固定的
        //===========================================================================================
        // 引脚本身可以由 `USART::USARTx_T.._R..()` 工厂配成, 但**外设号不能随便换**:
        // USART::getDefaultDMARxProfile() 按 profile 返回固定的 DMA 流。接错外设 ⇒ DMA 收不到数据,
        // 而 UART 寄存器看起来一切正常(能发不能收的经典现象)。
        static_assert(pinNo(cfg::hw::link::kTx) == 8 && pinNo(cfg::hw::link::kRx) == 9,
                      "E220 挂在 USART3 的 PD8/PD9 上: 用 USART::USART3_TD8_RD9()");
        static_assert(portNo(cfg::hw::link::kTx) == 3 && portNo(cfg::hw::link::kRx) == 3,
                      "E220 的 TX/RX 都必须在端口 D");
        static_assert(pinNo(cfg::hw::gnss::kTx) == 5 && pinNo(cfg::hw::gnss::kRx) == 6,
                      "GNSS 挂在 USART2 的 PD5/PD6 上: 用 USART::USART2_TD5_RD6()");
        static_assert(portNo(cfg::hw::gnss::kTx) == 3 && portNo(cfg::hw::gnss::kRx) == 3,
                      "GNSS 的 TX/RX 都必须在端口 D");
        // E220 的串口速率必须落在模块支持的档位上(手册 REG0), 而且 ≤1.25×空中速率 ——
        // 后一条在 flight_config.hpp 里已经守了, 这里只保证它是个"档位"值。
        static_assert(cfg::hw::link::kBaud == 1200u || cfg::hw::link::kBaud == 2400u ||
                          cfg::hw::link::kBaud == 4800u || cfg::hw::link::kBaud == 9600u ||
                          cfg::hw::link::kBaud == 19200u || cfg::hw::link::kBaud == 38400u ||
                          cfg::hw::link::kBaud == 57600u || cfg::hw::link::kBaud == 115200u,
                      "E220 串口速率必须是模块支持的档位之一");
        // NEO-7M 出厂是 9600bps @ **1Hz** 输出, 而固件里没有任何 UBX 配置代码 ——
        // 过期阈值小于一个输出周期, speedValid 就会周期性闪断(实测过: 500ms 时每秒断一半)。
        static_assert(cfg::sensor::gnss::kStaleMs >= 1100u,
                      "GNSS 数据过期阈值必须 ≥1100ms: 模块出厂 1Hz 输出(除非另行用 u-center 配到 5Hz)");

        //===========================================================================================
        // 四、I²C —— 两条总线各接一个器件, 引脚不能重叠
        //===========================================================================================
        static_assert(portNo(cfg::hw::baro::kScl) == 1 && pinNo(cfg::hw::baro::kScl) == 6 &&
                          portNo(cfg::hw::baro::kSda) == 1 && pinNo(cfg::hw::baro::kSda) == 7,
                      "气压计在 I2C1 的 PB6/PB7 上: 用 IICBus::IIC1_SB6_DB7()");
        static_assert(portNo(cfg::hw::range_::kScl) == 5 && pinNo(cfg::hw::range_::kScl) == 1 &&
                          portNo(cfg::hw::range_::kSda) == 5 && pinNo(cfg::hw::range_::kSda) == 0,
                      "测距在 I2C2 的 PF1(SCL)/PF0(SDA) 上: 用 IICBus::IIC2_SF1_DF0()");
        static_assert(cfg::hw::baro::kAddr != cfg::hw::range_::kAddr,
                      "两个 I²C 器件地址相同会同时应答 —— 虽然分属两条总线, 仍视为配置写错");

        //===========================================================================================
        // 五、SPI —— 片选脚不能和时钟/数据脚重复, 也不能和别的外设抢同一个脚
        //===========================================================================================
        static_assert(portNo(cfg::hw::flow::kSck) == 2 && pinNo(cfg::hw::flow::kSck) == 10,
                      "光流在 SPI3 的 PC10/PC11/PC12 上: 用 SPI::SPI3_SCA_MICB_MOCC()(无片选重载)");
        static_assert(cfg::hw::flow::kCs != cfg::hw::flow::kSck &&
                          cfg::hw::flow::kCs != cfg::hw::flow::kMiso &&
                          cfg::hw::flow::kCs != cfg::hw::flow::kMosi,
                      "PMW3901 的片选(PA4)不能和 SPI3 的时钟/数据脚是同一个引脚");
        static_assert(cfg::hw::flow::kCs != cfg::hw::range_::kInt,
                      "PMW3901 片选(PA4) 与 VL53L1X 中断(PA3) 不能撞车 —— 都在端口 A 但不同脚, 这里防手滑");
        static_assert(cfg::hw::flow::kAddrToDataUs >= 35u,
                      "PMW3901 地址字节与数据字节之间必须 ≥35µs 真间隙(手册): 这是'读回全 0'的头号原因");

        //===========================================================================================
        // 六、电机 —— 物理通道必须是 0..3 的一个排列(重复映射会让两路输出同一电机)
        //===========================================================================================
        static_assert(cfg::hw::motor::kMap[0] != cfg::hw::motor::kMap[1] &&
                          cfg::hw::motor::kMap[0] != cfg::hw::motor::kMap[2] &&
                          cfg::hw::motor::kMap[0] != cfg::hw::motor::kMap[3] &&
                          cfg::hw::motor::kMap[1] != cfg::hw::motor::kMap[2] &&
                          cfg::hw::motor::kMap[1] != cfg::hw::motor::kMap[3] &&
                          cfg::hw::motor::kMap[2] != cfg::hw::motor::kMap[3],
                      "电机映射必须互不相同: 两路映射到同一个物理通道 = 那一路输出两倍");

    } // namespace claims
} // namespace dlx
