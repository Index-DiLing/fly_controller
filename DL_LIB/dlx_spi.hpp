#pragma once
#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_spi_profile.hpp"
namespace dlx
{
    class SPI
    {
    private:
        GPIO nss;
        SPIProfile profile;

        inline SPI_TypeDef *getSPI_TypeDef()
        {
            switch (profile) {
                case SPIProfile::SPI1_Profile:
                    return SPI1;
                case SPIProfile::SPI2_Profile:
                    return SPI2;
                case SPIProfile::SPI3_Profile:
                    return SPI3;
            }
            return SPI1;
        }

        // SPI1 挂在 APB2, SPI2/SPI3 挂在 APB1
        inline bool isAPB2Periph()
        {
            return profile == SPIProfile::SPI1_Profile;
        }

        inline uint32_t getRCC_APB2Periph()
        {
            switch (profile) {
                case SPIProfile::SPI1_Profile:
                    return RCC_APB2Periph_SPI1;
                default:
                    return 0;
            }
        }

        inline uint32_t getRCC_APB1Periph()
        {
            switch (profile) {
                case SPIProfile::SPI2_Profile:
                    return RCC_APB1Periph_SPI2;
                case SPIProfile::SPI3_Profile:
                    return RCC_APB1Periph_SPI3;
                default:
                    return 0;
            }
        }

    public:
        SPI(GPIOProfile nssProfile, SPIProfile profile)
            : nss(nssProfile), profile(profile)
        {
#warning [Experimental]
        }
        ~SPI()
        {
        }

        // 供工厂方法初始化 SCK/MISO/MOSI 使用: SPI1/SPI2 复用 AF5, SPI3 复用 AF6
        inline GPIOModeProfile getGPIOAFProfile()
        {
            return profile == SPIProfile::SPI3_Profile
                       ? GPIOModeProfile::AF6_PP_NOPULL_50MHz
                       : GPIOModeProfile::AF5_PP_NOPULL_50MHz;
        }

        // ------------------------------------------------------------------
        // 工厂函数: 默认使用软件 NSS(片选由 GPIO 控制),
        // 因此需要额外传入 NSS 引脚, 与其他引脚一起传递给构造函数.
        // SCK/MISO/MOSI 在这里按对应 AF 配置, NSS 由 init() 配为推挽输出并拉高.
        // ------------------------------------------------------------------
        static inline SPI make(GPIOProfile nssPin, GPIOProfile sckPin, GPIOProfile misoPin, GPIOProfile mosiPin, SPIProfile profile)
        {
            SPI spi(nssPin, profile);
            GPIO sck(sckPin);
            sck.init(spi.getGPIOAFProfile());
            GPIO miso(misoPin);
            miso.init(spi.getGPIOAFProfile());
            GPIO mosi(mosiPin);
            mosi.init(spi.getGPIOAFProfile());
            return spi;
        }

        // SPI1: SCK=A5, MISO=A6, MOSI=A7
        static inline SPI SPI1_SA5_MIA6_MOA7(GPIOProfile nssPin)
        { return make(nssPin, GPIOProfile::A5, GPIOProfile::A6, GPIOProfile::A7, SPIProfile::SPI1_Profile); }
        // SPI1: SCK=B3, MISO=B4, MOSI=B5
        static inline SPI SPI1_SB3_MIB4_MOB5(GPIOProfile nssPin)
        { return make(nssPin, GPIOProfile::B3, GPIOProfile::B4, GPIOProfile::B5, SPIProfile::SPI1_Profile); }
        // SPI2: SCK=BD, MISO=BE, MOSI=BF
        static inline SPI SPI2_SB13_MIB14_MOB15(GPIOProfile nssPin)
        { return make(nssPin, GPIOProfile::BD, GPIOProfile::BE, GPIOProfile::BF, SPIProfile::SPI2_Profile); }
        // SPI2: SCK=B10, MISO=C2, MOSI=C3
        static inline SPI SPI2_SB10_MIC2_MOC3(GPIOProfile nssPin)
        { return make(nssPin, GPIOProfile::BA, GPIOProfile::C2, GPIOProfile::C3, SPIProfile::SPI2_Profile); }
        // SPI3: SCK=C10, MISO=C11, MOSI=C12
        static inline SPI SPI3_SCA_MICB_MOCC(GPIOProfile nssPin)
        { return make(nssPin, GPIOProfile::CA, GPIOProfile::CB, GPIOProfile::CC, SPIProfile::SPI3_Profile); }
        // SPI3: SCK=B3, MISO=B4, MOSI=B5
        static inline SPI SPI3_SB3_MIB4_MOB5(GPIOProfile nssPin)
        { return make(nssPin, GPIOProfile::B3, GPIOProfile::B4, GPIOProfile::B5, SPIProfile::SPI3_Profile); }

        /**
         * @brief 初始化 SPI 外设并开启使能, 同时把 NSS 引脚配置为推挽输出并拉高
         * @param mode SPIModeProfile, 按位拼装的模式配置, 见 dlx_spi_profile.hpp
         * @note SCK/MISO/MOSI 引脚由工厂方法配置(见 getGPIOAFProfile)
         */
        void init(SPIModeProfile mode)
        {
            if (isAPB2Periph()) {
                RCC_APB2PeriphClockCmd(getRCC_APB2Periph(), ENABLE);
            } else {
                RCC_APB1PeriphClockCmd(getRCC_APB1Periph(), ENABLE);
            }

            // NSS 引脚: 推挽输出 + 上拉, 默认拉高(片选无效)
            nss.init(GPIOModeProfile::OUT_PP_UP_50MHz);
            nss = 1;

            uint16_t modeVal = static_cast<uint16_t>(mode);

            // 从按位拼装的配置中按掩码取出各字段, 掩码结果即 stm32f4xx_spi.h 宏
            SPI_InitTypeDef spiInit;
            spiInit.SPI_Direction         = modeVal & 0xC400; // 位[15:14] + 位[10]
            spiInit.SPI_Mode              = modeVal & 0x0104; // 位[8] + 位[2]
            spiInit.SPI_DataSize          = modeVal & 0x0800; // 位[11]
            spiInit.SPI_CPOL              = modeVal & 0x0002; // 位[1]
            spiInit.SPI_CPHA              = modeVal & 0x0001; // 位[0]
            spiInit.SPI_NSS               = modeVal & 0x0200; // 位[9]
            spiInit.SPI_BaudRatePrescaler = modeVal & 0x0038; // 位[5:3]
            spiInit.SPI_FirstBit          = modeVal & 0x0080; // 位[7]
            spiInit.SPI_CRCPolynomial     = 7;
            SPI_Init(getSPI_TypeDef(), &spiInit);
            SPI_Cmd(getSPI_TypeDef(), ENABLE);
        }

        /**
         * @brief 同步&阻塞 交换一个单位,在一个周期同时收发一个单位
         * 
         * @param data 发送的字节
         * @return uint16_t 接收到的字节
         */
        uint16_t swap(uint16_t data)
        {
            SPI_TypeDef *spix = getSPI_TypeDef();
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(spix, data);
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_RXNE) == RESET);
            return SPI_I2S_ReceiveData(spix);
        }

        /**
         * @brief 同步&阻塞 交换一个单位,在一个周期同时收发一个单位
         * 
         * @param data 发送的字节
         * @return uint16_t 接收到的字节
         */
        uint8_t swap(uint8_t data)
        {
            SPI_TypeDef *spix = getSPI_TypeDef();
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(spix, data);
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_RXNE) == RESET);
            return SPI_I2S_ReceiveData(spix);
        }

        /**
         * @brief 发送一个字节
         * 
         * @param data 要发送的数据
         */
        void send(uint16_t data)
        {
            SPI_TypeDef *spix = getSPI_TypeDef();
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(spix, data);
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_BSY) == SET);
            if (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_RXNE) == SET) {
                SPI_I2S_ReceiveData(spix);
            }
        }


        SPI& write(uint16_t data)
        {
            SPI_TypeDef *spix = getSPI_TypeDef();
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(spix, data);
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_BSY) == SET);
            if (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_RXNE) == SET) {
                SPI_I2S_ReceiveData(spix);
            }
            return *this;
        }
        SPI& read(uint16_t* data)
        {
            SPI_TypeDef *spix = getSPI_TypeDef();
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(spix, 0x00);
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_RXNE) == RESET);
            *data = SPI_I2S_ReceiveData(spix);
            return *this;
        }

        SPI& read(uint8_t* data)
        {
            SPI_TypeDef *spix = getSPI_TypeDef();
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(spix, 0x00);
            while (SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_RXNE) == RESET);
            *data = SPI_I2S_ReceiveData(spix);
            return *this;
        }


        SPI& select()
        {
            nss = 0;
            return *this;
        }
        SPI& deselect()
        {
            nss = 1;
            return *this;
        }

        



    };

} // namespace dlx
