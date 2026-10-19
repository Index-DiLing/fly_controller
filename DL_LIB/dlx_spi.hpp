#pragma once
#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_spi_profile.hpp"
namespace dlx
{
    /**
     * @brief SPI 主机（std-periph，阻塞式收发）
     *
     * 关于片选 —— 这个类有**两种**用法，必须由调用方说清楚是哪一种：
     *
     *   1. **带软件片选**：`SPI2_SB10_MIC2_MOC3(cs)`。传进来的脚由本类初始化为推挽输出并拉高，
     *      收发前后调 `select()/deselect()` 拉低/拉高。W25Q128（`hw.flash.kCs = D1`）与
     *      NRF24L01（`hw.nrf.kCsn = C8`）就是这样用的。
     *   2. **无片选**：`SPI1_SA5_MIA6_MOA7()`（不传脚）。这条总线上的每个器件有自己的 CS GPIO
     *      （BMI088 是 PB12 加速度计 / PB13 陀螺，由驱动自己翻转），本类**一个引脚都不碰**，
     *      `select()/deselect()` 是空操作。
     *
     * 为什么要有第 2 种：原来无论如何都会把传进来的脚初始化成输出并拉高，于是
     * `SPI1_SA5_MIA6_MOA7(hw.imu.kNssPin = A3)` 把 **PA3 抢过来拉高**了 —— 而 PA3 要接
     * VL53L1X 的 GPIO1（数据就绪中断输入）。那个脚其实从来不是任何器件的片选，纯属冗余参数。
     * 注意**不能**简单改成"软 NSS 就不初始化"：W25Q128/NRF 用的也是软 NSS，那样会把
     * 两条总线的片选一起弄丢（见 config/flight_config.schema.json 的 hw.imu.kNssPin 依据）。
     */
    class SPI
    {
    private:
        GPIO nss;
        bool hasNss;
        SPIProfile profile;
        /**
         * 最近一次 init() 传进来的模式(CPOL/CPHA/分频/位序)。
         *
         * 为什么要存下来: applyProfile() 要在**每次事务之前**把模式重新写进外设寄存器,
         * 而它必须知道"本条总线要的是什么模式"。见 applyProfile() 的完整理由。
         * 初值给 SPI1 的 16 分频: 没调用过 init() 就调 applyProfile() 时不会写进一个随机的模式
         * (但那种调用顺序本身就是错的 —— init() 必须先调)。
         */
        SPIModeProfile mode{SPIModeProfile::FD_M_8B_CH_2E_NS_BR16_MSB};

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
            : nss(nssProfile), hasNss(true), profile(profile)
        {
#warning [Experimental]
        }

        /** 无软件片选：本类不碰任何片选脚（器件自带 CS）。nss 成员保持未初始化且从不使用。 */
        explicit SPI(SPIProfile profile)
            : nss(GPIOProfile::A0), hasNss(false), profile(profile)
        {
#warning [Experimental]
        }

        ~SPI()
        {
        }

        /** 这条总线是否由本类驱动软件片选（false = 调用方自己管 CS，select/deselect 是空操作） */
        bool usesSoftCs() const { return hasNss; }

        // 供工厂方法初始化 SCK/MISO/MOSI 使用: SPI1/SPI2 复用 AF5, SPI3 复用 AF6
        inline GPIOModeProfile getGPIOAFProfile()
        {
            return profile == SPIProfile::SPI3_Profile
                       ? GPIOModeProfile::AF6_PP_NOPULL_50MHz
                       : GPIOModeProfile::AF5_PP_NOPULL_50MHz;
        }

        /** 三个数据/时钟脚按本总线的 AF 配置(片选脚不在这里, 见 init()) */
        inline void configPins(GPIOProfile sckPin, GPIOProfile misoPin, GPIOProfile mosiPin)
        {
            const GPIOModeProfile af = getGPIOAFProfile();
            GPIO sck(sckPin);
            sck.init(af);
            GPIO miso(misoPin);
            miso.init(af);
            GPIO mosi(mosiPin);
            mosi.init(af);
        }

        // ------------------------------------------------------------------
        // 工厂函数 —— **每个引脚组合都有两个版本**, 由调用方选:
        //
        //   SPIx_Sxx_MIxx_MOxx(nssPin)  带软件片选: 传进来的脚由 init() 配为推挽输出并拉高,
        //                               收发前后用 select()/deselect() 拉低/拉高
        //   SPIx_Sxx_MIxx_MOxx()        无片选: 本类**一个片选脚都不碰**, select()/deselect() 是空操作
        //
        // 什么时候用无片选: 这条总线上的每个器件都有**自己的 CS GPIO**(BMI088 的 PB12/PB13、
        // PMW3901 的 PA4), 由各驱动自己翻转。这时再传一个"nss"脚进来纯属多余, 而且会**白占一个引脚**
        // (PA3 就是这么被占掉的, 见类注释)。
        //
        // 两个重载签名不冲突(参数个数不同), 名字只差参数表 —— 这是刻意的: 写代码时一眼能看出
        // "这条总线要不要软件片选", 而不是靠记住某个 bool 参数的含义。
        // ------------------------------------------------------------------
        /** 带软件片选 */
        static inline SPI make(GPIOProfile nssPin, GPIOProfile sckPin, GPIOProfile misoPin, GPIOProfile mosiPin, SPIProfile profile)
        {
            SPI spi(nssPin, profile);
            spi.configPins(sckPin, misoPin, mosiPin);
            return spi;
        }

        /** 无软件片选(不初始化、不碰任何片选脚) */
        static inline SPI make(SPIProfile profile, GPIOProfile sckPin, GPIOProfile misoPin, GPIOProfile mosiPin)
        {
            SPI spi(profile);
            spi.configPins(sckPin, misoPin, mosiPin);
            return spi;
        }

        // ---- SPI1 ----
        // SPI1: SCK=A5, MISO=A6, MOSI=A7
        static inline SPI SPI1_SA5_MIA6_MOA7(GPIOProfile nssPin)
        { return make(nssPin, GPIOProfile::A5, GPIOProfile::A6, GPIOProfile::A7, SPIProfile::SPI1_Profile); }
        /** 无片选: BMI088 的 CS 是 PB12/PB13 两个独立 GPIO, 用这个重载 (PA3 因此不被占用) */
        static inline SPI SPI1_SA5_MIA6_MOA7()
        { return make(SPIProfile::SPI1_Profile, GPIOProfile::A5, GPIOProfile::A6, GPIOProfile::A7); }

        // SPI1: SCK=B3, MISO=B4, MOSI=B5
        static inline SPI SPI1_SB3_MIB4_MOB5(GPIOProfile nssPin)
        { return make(nssPin, GPIOProfile::B3, GPIOProfile::B4, GPIOProfile::B5, SPIProfile::SPI1_Profile); }
        /** 无片选版本 */
        static inline SPI SPI1_SB3_MIB4_MOB5()
        { return make(SPIProfile::SPI1_Profile, GPIOProfile::B3, GPIOProfile::B4, GPIOProfile::B5); }

        // ---- SPI2 ----
        // SPI2: SCK=BD, MISO=BE, MOSI=BF
        static inline SPI SPI2_SB13_MIB14_MOB15(GPIOProfile nssPin)
        { return make(nssPin, GPIOProfile::BD, GPIOProfile::BE, GPIOProfile::BF, SPIProfile::SPI2_Profile); }
        /** 无片选版本 */
        static inline SPI SPI2_SB13_MIB14_MOB15()
        { return make(SPIProfile::SPI2_Profile, GPIOProfile::BD, GPIOProfile::BE, GPIOProfile::BF); }

        // SPI2: SCK=B10, MISO=C2, MOSI=C3
        static inline SPI SPI2_SB10_MIC2_MOC3(GPIOProfile nssPin)
        { return make(nssPin, GPIOProfile::BA, GPIOProfile::C2, GPIOProfile::C3, SPIProfile::SPI2_Profile); }
        /** 无片选版本 */
        static inline SPI SPI2_SB10_MIC2_MOC3()
        { return make(SPIProfile::SPI2_Profile, GPIOProfile::BA, GPIOProfile::C2, GPIOProfile::C3); }

        // ---- SPI3 ----
        // SPI3: SCK=C10, MISO=C11, MOSI=C12
        static inline SPI SPI3_SCA_MICB_MOCC(GPIOProfile nssPin)
        { return make(nssPin, GPIOProfile::CA, GPIOProfile::CB, GPIOProfile::CC, SPIProfile::SPI3_Profile); }
        /** 无片选版本: SPI3 上挂多片(PMW3901 用 PA4 做 CS)时用它 */
        static inline SPI SPI3_SCA_MICB_MOCC()
        { return make(SPIProfile::SPI3_Profile, GPIOProfile::CA, GPIOProfile::CB, GPIOProfile::CC); }

        // SPI3: SCK=B3, MISO=B4, MOSI=B5
        static inline SPI SPI3_SB3_MIB4_MOB5(GPIOProfile nssPin)
        { return make(nssPin, GPIOProfile::B3, GPIOProfile::B4, GPIOProfile::B5, SPIProfile::SPI3_Profile); }
        /** 无片选版本 */
        static inline SPI SPI3_SB3_MIB4_MOB5()
        { return make(SPIProfile::SPI3_Profile, GPIOProfile::B3, GPIOProfile::B4, GPIOProfile::B5); }

        /**
         * @brief 初始化 SPI 外设并开启使能
         * @param mode SPIModeProfile, 按位拼装的模式配置, 见 dlx_spi_profile.hpp
         * @note SCK/MISO/MOSI 引脚由工厂方法配置(见 getGPIOAFProfile);
         *       片选脚**只在这条总线真的用软件片选时**才初始化(见类注释)
         */
        void init(SPIModeProfile mode)
        {
            if (isAPB2Periph()) {
                RCC_APB2PeriphClockCmd(getRCC_APB2Periph(), ENABLE);
            } else {
                RCC_APB1PeriphClockCmd(getRCC_APB1Periph(), ENABLE);
            }

            // NSS 引脚: 推挽输出 + 上拉, 默认拉高(片选无效)。
            // 无片选的总线(hasNss == false)一个引脚都不碰 —— 这正是 PA3 被释放出来的原因。
            if (hasNss) {
                nss.init(GPIOModeProfile::OUT_PP_UP_50MHz);
                nss = 1;
            }

            this->mode = mode;
            applyProfile();
        }

        /**
         * @brief 把本条总线自己的模式(CPOL/CPHA/分频/位序)**重新写进外设寄存器**
         *
         * 为什么需要它(2026-10-03, B6 加光流时发现):
         *   一个 SPI 外设上可以挂多个器件, 而**不同器件的 SPI 模式可以不同** ——
         *   本板上 SPI3 同时挂着 NRF24L01(模式 0: CPOL=0/CPHA=1 边沿取样)和 PMW3901(模式 3)。
         *   模式寄存器是**外设级**的, 所以后 init 的那个会把先 init 的器件悄悄改掉: 编译通过、
         *   时序看起来正常、器件不报错, 只是**读回来的全是垃圾**(或写进去的全没落地)。
         *   这是一个"另一个器件的代码动了我这条总线的配置"的典型事故, 而且只在两个器件
         *   同时被使用时才出现 —— 单独测谁都好。
         *
         * 做法: 每次开始一次事务之前重新声明本器件要的模式。
         *   - 软件片选的总线: 在 select() 里自动做(NRF/W25Q128 这类不用改调用点);
         *   - 自己管片选的总线: 由驱动在拉低 CS 之后显式调 applyProfile()(见 PMW3901::beginTransfer)。
         * 代价是每次事务多几条寄存器写(几十 ns), 相对一次 8 位传输(µs 级)可以忽略。
         */
        void applyProfile()
        {
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
            if (hasNss) {
                // 每次事务前重申本条总线的模式: 见 applyProfile() 的说明 ——
                // 同一外设上挂两个模式不同的器件时, 这一步是"另一个器件别改我的配置"的唯一保证。
                applyProfile();
                nss = 0;
            }
            return *this;
        }
        SPI& deselect()
        {
            if (hasNss) {
                nss = 1;
            }
            return *this;
        }

        



    };

} // namespace dlx
