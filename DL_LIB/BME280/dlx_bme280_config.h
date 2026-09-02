#pragma once
#include <stdint.h>

namespace dlx
{
    /** BME280 7 位 I2C 地址(SDO 接低 0x76, 接高 0x77) */
    constexpr uint8_t BME280_IIC_ADDRESS = 0x76;

    /** BME280 寄存器地址 */
    enum class BME280_Reg : uint8_t
    {
        ChipId   = 0xD0, ///< 芯片 ID, 恒为 0x60
        Reset    = 0xE0, ///< 软复位, 写 0xB6
        CtrlHum  = 0xF2, ///< 湿度控制: [7:5] osrs_h
        Status   = 0xF3, ///< [3] measuring, [0] im_update
        CtrlMeas = 0xF4, ///< [7:5] osrs_t, [4:2] osrs_p, [1:0] mode
        Config   = 0xF5, ///< [7:5] t_sb, [4:2] filter, [0] spi3w_en
        PressMsb = 0xF7, ///< 气压数据起址(0xF7~0xF9)
        TempMsb  = 0xFA, ///< 温度数据起址(0xFA~0xFC)
        HumMsb   = 0xFD, ///< 湿度数据起址(0xFD~0xFE)
        CalibStart = 0x88, ///< 校准起址(0x88~0x9F: dig_T1~dig_P9, 24B)
        CalibH1    = 0xA1, ///< dig_H1(1B)
        CalibH2    = 0xE1, ///< dig_H2~dig_H6(0xE1~0xE7, 7B)
    };

    /**
     * @brief BME280 模块配置(与 BMI088_Config 同思路: 枚举值 = 写入对应寄存器的值)
     * 需要调整工作参数时直接修改这里的数值, init() 会自动应用.
     */
    enum class BME280_Config : uint8_t
    {
        /**
         * CTRL_HUM (0xF2): [7:5] osrs_h
         *  0x0=skip, 0x1=×1, 0x2=×2, 0x3=×4, 0x4=×8, 0x5=×16
         */
        CtrlHum = 0x01, ///< 湿度过采样 ×1(默认)

        /**
         * CTRL_MEAS (0xF4): [7:5] osrs_t, [4:2] osrs_p, [1:0] mode
         *  osrs: 0x0=skip, 0x1=×1 ... 0x5=×16; mode: 0=sleep, 1/2=forced, 3=normal
         */
        CtrlMeas = 0x93, ///< 温度×2 气压×1 正常模式(与旧驱动一致)

        /**
         * CONFIG (0xF5): [7:5] t_sb, [4:2] filter, [0] spi3w_en
         *  t_sb: 0x0=0.5ms, 0x1=62.5ms, 0x2=125ms ... 0x7=1000ms
         *  filter: 0x0=off, 0x1=×2, 0x2=×4, 0x3=×8, 0x4=×16
         */
        Config = 0x2C, ///< 待机 62.5ms, 滤波 ×8, SPI 3 线关闭(与旧驱动一致)
    };
} // namespace dlx
