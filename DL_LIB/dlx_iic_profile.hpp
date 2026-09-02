#pragma once
#include <stdint.h>
namespace dlx
{
    // ------------------------------------------------------------------
    // IICBusProfile 枚举值沿用 stm32f4xx.h 中的事件中断号(与 USARTProfile/SPIProfile 风格一致)
    // 当前工程为 STM32F407, 提供 I2C1~I2C3
    // ------------------------------------------------------------------
    enum class IICBusProfile : uint8_t {
        I2C1_Profile = (0x1F), // I2C1_EV_IRQn = 31
        I2C2_Profile = (0x21), // I2C2_EV_IRQn = 33
        I2C3_Profile = (0x48), // I2C3_EV_IRQn = 72
    };

    // ------------------------------------------------------------------
    // IICBusModeProfile: 固定使用 IIC 模式(SMBUS 不使用), 按位拼装(uint16_t):
    //   bit[10]  ACK       : 0 = Disable, 1 = Enable
    //   bit[14]  DutyCycle : 0 = 2,       1 = 16/9
    //   bit[15]  AddrMode  : 0 = 7bit,    1 = 10bit(本机地址长度)
    // 命名规则: ACK_开关_占空比_地址长度, 例如
    //   ACK_E_DC16_9_ADDR7 = 使能ACK / 16:9占空比 / 7位地址
    // 掩码 0x0400 / 0x4000 即 stm32f4xx_i2c.h 的 I2C_Ack_Enable / I2C_DutyCycle_16_9,
    // 10bit 地址需要转换为 0xC000, 由 init() 统一处理.
    // ------------------------------------------------------------------
    enum class IICBusModeProfile : uint16_t {
        ACK_D_DC2_ADDR7     = (0x0000),
        ACK_D_DC2_ADDR10    = (0x8000),
        ACK_D_DC16_9_ADDR7  = (0x4000),
        ACK_D_DC16_9_ADDR10 = (0xC000),
        ACK_E_DC2_ADDR7     = (0x0400),
        ACK_E_DC2_ADDR10    = (0x8400),
        ACK_E_DC16_9_ADDR7  = (0x4400),
        ACK_E_DC16_9_ADDR10 = (0xC400),
    };
} // namespace dlx
