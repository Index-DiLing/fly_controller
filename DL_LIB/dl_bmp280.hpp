
#pragma once
#include "stm32f4xx.h"
#include "dl_iic.hpp"
namespace dl
{
    // 包含一些传感器补偿数据
    // 设置一些奇怪的类型
    // 这里丢一些奇怪的变量,照理来说其实是常量

    namespace BME280
    {

        struct dataType {
            uint64_t pressure; // Q24.8
            uint32_t temperature;
            uint32_t humidity; // Q22.10
        } rcvData;//这个地方实际就是已经解码补偿好的数据
        /**
         * @brief 单例对象
         *
         */
        struct calibData {
            uint16_t dig_T1;
            int16_t dig_T2;
            int16_t dig_T3;
            uint16_t dig_P1;
            int16_t dig_P2;
            int16_t dig_P3;
            int16_t dig_P4;
            int16_t dig_P5;
            int16_t dig_P6;
            int16_t dig_P7;
            int16_t dig_P8;
            int16_t dig_P9;
            uint8_t dig_H1;
            int16_t dig_H2;
            int8_t dig_H3;
            int16_t dig_H4;
            int16_t dig_H5;
            int8_t dig_H6;
        } calib;
        int32_t t_fine;

        /**
         * @brief 兼容IIC接口,初始化时写入寄存器参数,读取校准值
         * @todo TODO: 非默认参数的情况
         *
         * @param bme
         */
        void init(dl::IICDevice &bme)
        {
            bme.read(0x88, 26);
            // 解析前 12 个校准参数（dig_T1~dig_P9）
            memcpy(&dl::BME280::calib, &bme.buffer, 26);
            // 再读取 0xE1~0xE7（7字节）
            bme.read(0xE1, 7);
            // 写入剩下七个字节
            calib.dig_H2 = (bme.buffer[1] << 8) | bme.buffer[0];
            calib.dig_H3 = bme.buffer[2];
            calib.dig_H4 = (bme.buffer[3] << 4) | (bme.buffer[4] & 0x0F);
            calib.dig_H5 = (bme.buffer[5] << 4) | ((bme.buffer[4] >> 4) & 0x0F);
            calib.dig_H6 = bme.buffer[6];

            bme.write(0xF2, 0x01); // 湿度 ×1
            bme.write(0xF4, 0x57); // 温度 ×2，压力 ×16，正常模式
            bme.write(0xF5, 0x10); // 待机 0.5ms，滤波器 16
        }

        /**
         * @brief 通过ADC的T值计算出实际的温度值,单位0.01°C
         *
         * @param adc_T
         * @return int32_t
         */
        int32_t compensate_T_int32(int32_t adc_T)
        {
            int32_t var1, var2, T;
            var1   = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
            var2   = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
            t_fine = var1 + var2;
            T      = (t_fine * 5 + 128) >> 8;
            return T;
        }
        // Q24.8
        uint32_t compensate_P_int64(int32_t adc_P)
        {
            int64_t var1, var2, p;
            var1 = ((int64_t)t_fine) - 128000;
            var2 = var1 * var1 * (int64_t)calib.dig_P6;
            var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
            var2 = var2 + (((int64_t)calib.dig_P4) << 35);
            var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
            var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
            if (var1 == 0) {
                return 0; // avoid exception caused by division by zero
            }
            p    = 1048576 - adc_P;
            p    = (((p << 31) - var2) * 3125) / var1;
            var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
            var2 = (((int64_t)calib.dig_P8) * p) >> 19;
            p    = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
            return (uint32_t)p;
        }
        // Q22.10
        uint32_t compensate_H_int32(int32_t adc_H)
        {
            int32_t v_x1_u32r;
            v_x1_u32r = (t_fine - ((int32_t)76800));
            v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) - (((int32_t)calib.dig_H5) * v_x1_u32r)) +
                           ((int32_t)16384)) >>
                          15) *
                         (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) *
                              (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) + ((int32_t)32768))) >>
                             10) +
                            ((int32_t)2097152)) *
                               ((int32_t)calib.dig_H2) +
                           8192) >>
                          14));
            v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib.dig_H1)) >> 4));
            v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
            v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
            return (uint32_t)(v_x1_u32r >> 12);
        }

        /**
         * @brief 读取数据并将其存放到rcvData中
         * 
         * @note 原始数据
         * @param bme 
         */
        void readTPH(dl::IICDevice &bme)
        {
            bme.read(0xF7, 8);
            rcvData.temperature =compensate_T_int32(((uint32_t)bme.buffer[3] << 12)| ((uint32_t)bme.buffer[4] << 4)  | ((uint32_t)bme.buffer[5] >> 4));
            rcvData.pressure    =compensate_P_int64(((uint32_t)bme.buffer[0] << 12) | ((uint32_t)bme.buffer[1] << 4)  | ((uint32_t)bme.buffer[2] >> 4));
            rcvData.humidity    =compensate_H_int32(((uint16_t)bme.buffer[6])<< 8) | ((uint16_t)bme.buffer[7]);
            
        }

    } // namespace BME280

} // namespace dl
