
#include "stm32f4xx.h"
#include "dl_gpio.hpp"
#pragma once
namespace dl
{

    struct __SPI_Config {
        SPI_TypeDef *spix;

        uint32_t SPI_RCC_Periph;

        uint16_t SPI_Direction_; // 方向

        uint16_t SPI_Mode_; // 主从模式

        uint16_t SPI_DataSize_; // 数据大小

        uint16_t SPI_CPOL_; // 时钟极性

        uint16_t SPI_CPHA_; // 时钟相位

        uint16_t SPI_NSS_; // 片选管理

        uint16_t SPI_BaudRatePrescaler_; // 波特率预分频

        uint16_t SPI_FirstBit_; // 数据位顺序

        uint16_t SPI_CRCPolynomial_; // CRC多项式

        GPIO_TypeDef *SPI_IO_group;

        uint32_t SPI_IO_RCC_Periph;

        uint16_t sck_pin;
        uint16_t miso_pin;
        uint16_t mosi_pin;

        uint8_t SPI_AF_Source;
        uint8_t SCK_Pin_Source;
        uint8_t MISO_Pin_Source;
        uint8_t MOSI_Pin_Source;
    };

#define dSPI1_SA5_MIA6_MOA7                                        \
    dl::__SPI_Config                                               \
    {                                                              \
        .spix                   = SPI1,                            \
        .SPI_RCC_Periph         = RCC_APB2Periph_SPI1,             \
        .SPI_Direction_         = SPI_Direction_2Lines_FullDuplex, \
        .SPI_Mode_              = SPI_Mode_Master,                 \
        .SPI_DataSize_          = SPI_DataSize_8b,                 \
        .SPI_CPOL_              = SPI_CPOL_High,                   \
        .SPI_CPHA_              = SPI_CPHA_2Edge,                  \
        .SPI_NSS_               = SPI_NSS_Soft,                    \
        .SPI_BaudRatePrescaler_ = SPI_BaudRatePrescaler_8,         \
        .SPI_FirstBit_          = SPI_FirstBit_MSB,                \
        .SPI_CRCPolynomial_     = 7,                               \
        .SPI_IO_group           = GPIOA,                           \
        .SPI_IO_RCC_Periph      = RCC_AHB1Periph_GPIOA,            \
        .sck_pin                = GPIO_Pin_5,                      \
        .miso_pin               = GPIO_Pin_6,                      \
        .mosi_pin               = GPIO_Pin_7,                      \
        .SPI_AF_Source          = GPIO_AF_SPI1,                    \
        .SCK_Pin_Source         = GPIO_PinSource5,                 \
        .MISO_Pin_Source        = GPIO_PinSource6,                 \
        .MOSI_Pin_Source        = GPIO_PinSource7                  \
    }
#define dSPI1_SB3_MIB4_MOB5                                        \
    dl::__SPI_Config                                               \
    {                                                              \
        .spix                   = SPI1,                            \
        .SPI_RCC_Periph         = RCC_APB2Periph_SPI1,             \
        .SPI_Direction_         = SPI_Direction_2Lines_FullDuplex, \
        .SPI_Mode_              = SPI_Mode_Master,                 \
        .SPI_DataSize_          = SPI_DataSize_8b,                 \
        .SPI_CPOL_              = SPI_CPOL_Low,                    \
        .SPI_CPHA_              = SPI_CPHA_2Edge,                  \
        .SPI_NSS_               = SPI_NSS_Soft,                    \
        .SPI_BaudRatePrescaler_ = SPI_BaudRatePrescaler_2,         \
        .SPI_FirstBit_          = SPI_FirstBit_MSB,                \
        .SPI_CRCPolynomial_     = 7,                               \
        .SPI_IO_group           = GPIOB,                           \
        .SPI_IO_RCC_Periph      = RCC_AHB1Periph_GPIOB,            \
        .sck_pin                = GPIO_Pin_3,                      \
        .miso_pin               = GPIO_Pin_4,                      \
        .mosi_pin               = GPIO_Pin_5,                      \
        .SPI_AF_Source          = GPIO_AF_SPI1,                    \
        .SCK_Pin_Source         = GPIO_PinSource3,                 \
        .MISO_Pin_Source        = GPIO_PinSource4,                 \
        .MOSI_Pin_Source        = GPIO_PinSource5                  \
    }

#define dSPI2_SB13_MIB14_MISOB15                                   \
    dl::__SPI_Config                                               \
    {                                                              \
        .spix                   = SPI2,                            \
        .SPI_RCC_Periph         = RCC_APB1Periph_SPI2,             \
        .SPI_Direction_         = SPI_Direction_2Lines_FullDuplex, \
        .SPI_Mode_              = SPI_Mode_Master,                 \
        .SPI_DataSize_          = SPI_DataSize_8b,                 \
        .SPI_CPOL_              = SPI_CPOL_Low,                    \
        .SPI_CPHA_              = SPI_CPHA_1Edge,                  \
        .SPI_NSS_               = SPI_NSS_Soft,                    \
        .SPI_BaudRatePrescaler_ = SPI_BaudRatePrescaler_8,         \
        .SPI_FirstBit_          = SPI_FirstBit_MSB,                \
        .SPI_CRCPolynomial_     = 7,                               \
        .SPI_IO_group           = GPIOB,                           \
        .SPI_IO_RCC_Periph      = RCC_AHB1Periph_GPIOB,            \
        .sck_pin                = GPIO_Pin_13,                     \
        .miso_pin               = GPIO_Pin_14,                     \
        .mosi_pin               = GPIO_Pin_15,                     \
        .SPI_AF_Source          = GPIO_AF_SPI2,                    \
        .SCK_Pin_Source         = GPIO_PinSource13,                \
        .MISO_Pin_Source        = GPIO_PinSource14,                \
        .MOSI_Pin_Source        = GPIO_PinSource15                 \
    }
#define dSPI2_SB10_MIB14_MISOB15                                   \
    dl::__SPI_Config                                               \
    {                                                              \
        .spix                   = SPI2,                            \
        .SPI_RCC_Periph         = RCC_APB1Periph_SPI2,             \
        .SPI_Direction_         = SPI_Direction_2Lines_FullDuplex, \
        .SPI_Mode_              = SPI_Mode_Master,                 \
        .SPI_DataSize_          = SPI_DataSize_8b,                 \
        .SPI_CPOL_              = SPI_CPOL_Low,                    \
        .SPI_CPHA_              = SPI_CPHA_2Edge,                  \
        .SPI_NSS_               = SPI_NSS_Soft,                    \
        .SPI_BaudRatePrescaler_ = SPI_BaudRatePrescaler_2,         \
        .SPI_FirstBit_          = SPI_FirstBit_MSB,                \
        .SPI_CRCPolynomial_     = 7,                               \
        .SPI_IO_group           = GPIOB,                           \
        .SPI_IO_RCC_Periph      = RCC_AHB1Periph_GPIOB,            \
        .sck_pin                = GPIO_Pin_10,                     \
        .miso_pin               = GPIO_Pin_14,                     \
        .mosi_pin               = GPIO_Pin_15,                     \
        .SPI_AF_Source          = GPIO_AF_SPI2,                    \
        .SCK_Pin_Source         = GPIO_PinSource10,                \
        .MISO_Pin_Source        = GPIO_PinSource14,                \
        .MOSI_Pin_Source        = GPIO_PinSource15                 \
    }

    class SPI
    {
    public:
        GPIO_Pin_Type nss;
        SPI_TypeDef *spix;
        SPI(dl::__SPI_Config config, GPIO_Pin_Config NSS_GPIO = dGPIOAPin15)
        {
            SPI_InitTypeDef init = SPI_InitTypeDef{
                .SPI_Direction         = config.SPI_Direction_,
                .SPI_Mode              = config.SPI_Mode_,
                .SPI_DataSize          = config.SPI_DataSize_,
                .SPI_CPOL              = config.SPI_CPOL_,
                .SPI_CPHA              = config.SPI_CPHA_,
                .SPI_NSS               = config.SPI_NSS_,
                .SPI_BaudRatePrescaler = config.SPI_BaudRatePrescaler_,
                .SPI_FirstBit          = config.SPI_FirstBit_,
                .SPI_CRCPolynomial     = config.SPI_CRCPolynomial_};
            spix = config.spix;
            nss  = NSS_GPIO.pin;
            RCC_AHB1PeriphClockCmd(config.SPI_IO_RCC_Periph, ENABLE);

            if (spix == SPI2 || spix == SPI3) {
                RCC_APB1PeriphClockCmd(config.SPI_RCC_Periph, ENABLE);
            } else {
                RCC_APB2PeriphClockCmd(config.SPI_RCC_Periph, ENABLE);
            }

            // 初始化IO
            dl::GPIO sck(config.SPI_IO_group, config.sck_pin, GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP);
            dl::GPIO miso(config.SPI_IO_group, config.miso_pin, GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
            dl::GPIO mosi(config.SPI_IO_group, config.mosi_pin, GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP);
            // IO复用
            GPIO_PinAFConfig(config.SPI_IO_group, config.SCK_Pin_Source, config.SPI_AF_Source);
            GPIO_PinAFConfig(config.SPI_IO_group, config.MISO_Pin_Source, config.SPI_AF_Source);
            GPIO_PinAFConfig(config.SPI_IO_group, config.MOSI_Pin_Source, config.SPI_AF_Source);

            // SPI初始化
            SPI_Init(config.spix, &init);
            SPI_Cmd(config.spix, ENABLE);

            dl::GPIO_Pin_Init(NSS_GPIO, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP);

            nss = 1;
        }
        void send(uint8_t data)
        {
            nss = 0;
            dl::delay_ms(1);
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(spix, data);

            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_BSY) == SET);
            nss = 1;
            dl::delay_ms(1);
        }
        void send(uint8_t *data, uint16_t size)
        {
            nss = 0;
            dl::delay_ms(1);
            while (size--) {
                while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET);
                SPI_I2S_SendData(spix, *data);
                data++;
            }
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_BSY) == SET);
            nss = 1;
            dl::delay_ms(1);
        }

        SPI &write(uint8_t data)
        {
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET);

            SPI_I2S_SendData(spix, data);

            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_BSY) == SET);
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_RXNE) == RESET);
            SPI_I2S_ReceiveData(spix);

            return *this;
        }
        SPI &write(uint8_t *data, uint16_t size)
        {
            while (size--) {
                while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET);
                SPI_I2S_SendData(spix, *data);
                data++;
                while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_RXNE) == RESET);
                SPI_I2S_ReceiveData(spix);
            }
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_BSY) == SET);
            return *this;
        }
        SPI &read(uint8_t *d)
        {
            dl::delay_ticks(10);

            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(spix, 0x00); // 发送dummy数据以产生时钟

            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_RXNE) == RESET);
            uint8_t data = SPI_I2S_ReceiveData(spix);
            *d           = data;
            return *this;
        }
        SPI &read(uint8_t *data, uint16_t size)
        {
            while (size--) {

                while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET);
                SPI_I2S_SendData(spix, 0x00); // 发送dummy数据以产生时钟

                while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_RXNE) == RESET);
                *data = SPI_I2S_ReceiveData(spix);
                data++;
            }

            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_BSY) == SET);
            return *this;
        }

        SPI &swap(uint8_t *data, uint16_t size)
        {
            while (size--) {
                while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET);
                SPI_I2S_SendData(spix, *data);

                while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_RXNE) == RESET);
                *data = SPI_I2S_ReceiveData(spix);
                data++;
            }
            return *this;
        }

        SPI &swap(uint8_t *data)
        {
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(spix, *data);

            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_RXNE) == RESET);
            *data = SPI_I2S_ReceiveData(spix);

            return *this;
        }

        // 同步,需要等待
        uint16_t receive()
        {
            nss = 0;

            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(spix, 0x00); // 发送dummy数据以产生时钟

            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_RXNE) == RESET);

            return SPI_I2S_ReceiveData(spix);
            nss = 1;
        }

        void receive(uint8_t *data, uint16_t size)
        {
            nss = 0;
            while (size--) {
                *data = SPI_I2S_ReceiveData(spix);
                data++;
            }
            nss = 1;
        }

        void clearRXBuffer()
        {
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_RXNE) == SET) {
                SPI_I2S_ReceiveData(spix);
                while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_BSY) == SET) {
                    /* code */
                }
            }
        }

        SPI &select()
        {
            nss = 0;
            dl::delay_ms(1);
            return *this;
        }
        SPI &deselect()
        {
            nss = 1;
            dl::delay_ms(1);
            return *this;
        }
    };
}