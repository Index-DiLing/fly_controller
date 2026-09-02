#pragma once

#include <stdint.h>

namespace dlx
{
    // STM32 标准库中断通道枚举, 数值与 stm32f4xx.h 中 IRQn_Type(STM32F40_41xxx)一致
    enum class NVICITProfile : uint8_t
    {
        WWDG_IRQn               = (0x00), // IRQn = 0
        PVD_IRQn                = (0x01), // IRQn = 1
        TAMP_STAMP_IRQn         = (0x02), // IRQn = 2
        RTC_WKUP_IRQn           = (0x03), // IRQn = 3
        FLASH_IRQn              = (0x04), // IRQn = 4
        RCC_IRQn                = (0x05), // IRQn = 5
        EXTI0_IRQn              = (0x06), // IRQn = 6
        EXTI1_IRQn              = (0x07), // IRQn = 7
        EXTI2_IRQn              = (0x08), // IRQn = 8
        EXTI3_IRQn              = (0x09), // IRQn = 9
        EXTI4_IRQn              = (0x0A), // IRQn = 10
        DMA1_Stream0_IRQn       = (0x0B), // IRQn = 11
        DMA1_Stream1_IRQn       = (0x0C), // IRQn = 12
        DMA1_Stream2_IRQn       = (0x0D), // IRQn = 13
        DMA1_Stream3_IRQn       = (0x0E), // IRQn = 14
        DMA1_Stream4_IRQn       = (0x0F), // IRQn = 15
        DMA1_Stream5_IRQn       = (0x10), // IRQn = 16
        DMA1_Stream6_IRQn       = (0x11), // IRQn = 17
        ADC_IRQn                = (0x12), // IRQn = 18
        CAN1_TX_IRQn            = (0x13), // IRQn = 19
        CAN1_RX0_IRQn           = (0x14), // IRQn = 20
        CAN1_RX1_IRQn           = (0x15), // IRQn = 21
        CAN1_SCE_IRQn           = (0x16), // IRQn = 22
        EXTI9_5_IRQn            = (0x17), // IRQn = 23
        TIM1_BRK_TIM9_IRQn      = (0x18), // IRQn = 24
        TIM1_UP_TIM10_IRQn      = (0x19), // IRQn = 25
        TIM1_TRG_COM_TIM11_IRQn = (0x1A), // IRQn = 26
        TIM1_CC_IRQn            = (0x1B), // IRQn = 27
        TIM2_IRQn               = (0x1C), // IRQn = 28
        TIM3_IRQn               = (0x1D), // IRQn = 29
        TIM4_IRQn               = (0x1E), // IRQn = 30
        I2C1_EV_IRQn            = (0x1F), // IRQn = 31
        I2C1_ER_IRQn            = (0x20), // IRQn = 32
        I2C2_EV_IRQn            = (0x21), // IRQn = 33
        I2C2_ER_IRQn            = (0x22), // IRQn = 34
        SPI1_IRQn               = (0x23), // IRQn = 35
        SPI2_IRQn               = (0x24), // IRQn = 36
        USART1_IRQn             = (0x25), // IRQn = 37
        USART2_IRQn             = (0x26), // IRQn = 38
        USART3_IRQn             = (0x27), // IRQn = 39
        EXTI15_10_IRQn          = (0x28), // IRQn = 40
        RTC_Alarm_IRQn          = (0x29), // IRQn = 41
        OTG_FS_WKUP_IRQn        = (0x2A), // IRQn = 42
        TIM8_BRK_TIM12_IRQn     = (0x2B), // IRQn = 43
        TIM8_UP_TIM13_IRQn      = (0x2C), // IRQn = 44
        TIM8_TRG_COM_TIM14_IRQn = (0x2D), // IRQn = 45
        TIM8_CC_IRQn            = (0x2E), // IRQn = 46
        DMA1_Stream7_IRQn       = (0x2F), // IRQn = 47
        FSMC_IRQn               = (0x30), // IRQn = 48
        SDIO_IRQn               = (0x31), // IRQn = 49
        TIM5_IRQn               = (0x32), // IRQn = 50
        SPI3_IRQn               = (0x33), // IRQn = 51
        UART4_IRQn              = (0x34), // IRQn = 52
        UART5_IRQn              = (0x35), // IRQn = 53
        TIM6_DAC_IRQn           = (0x36), // IRQn = 54
        TIM7_IRQn               = (0x37), // IRQn = 55
        DMA2_Stream0_IRQn       = (0x38), // IRQn = 56
        DMA2_Stream1_IRQn       = (0x39), // IRQn = 57
        DMA2_Stream2_IRQn       = (0x3A), // IRQn = 58
        DMA2_Stream3_IRQn       = (0x3B), // IRQn = 59
        DMA2_Stream4_IRQn       = (0x3C), // IRQn = 60
        ETH_IRQn                = (0x3D), // IRQn = 61
        ETH_WKUP_IRQn           = (0x3E), // IRQn = 62
        CAN2_TX_IRQn            = (0x3F), // IRQn = 63
        CAN2_RX0_IRQn           = (0x40), // IRQn = 64
        CAN2_RX1_IRQn           = (0x41), // IRQn = 65
        CAN2_SCE_IRQn           = (0x42), // IRQn = 66
        OTG_FS_IRQn             = (0x43), // IRQn = 67
        DMA2_Stream5_IRQn       = (0x44), // IRQn = 68
        DMA2_Stream6_IRQn       = (0x45), // IRQn = 69
        DMA2_Stream7_IRQn       = (0x46), // IRQn = 70
        USART6_IRQn             = (0x47), // IRQn = 71
        I2C3_EV_IRQn            = (0x48), // IRQn = 72
        I2C3_ER_IRQn            = (0x49), // IRQn = 73
        OTG_HS_EP1_OUT_IRQn     = (0x4A), // IRQn = 74
        OTG_HS_EP1_IN_IRQn      = (0x4B), // IRQn = 75
        OTG_HS_WKUP_IRQn        = (0x4C), // IRQn = 76
        OTG_HS_IRQn             = (0x4D), // IRQn = 77
        DCMI_IRQn               = (0x4E), // IRQn = 78
        CRYP_IRQn               = (0x4F), // IRQn = 79
        HASH_RNG_IRQn           = (0x50), // IRQn = 80
        FPU_IRQn                = (0x51), // IRQn = 81
    };
} // namespace dlx
