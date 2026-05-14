
#pragma once
#include "stm32f4xx.h"
#include "dl_iic.hpp"
#include "dl_log.h"
namespace dl
{
    // 包含一些传感器补偿数据
    // 设置一些奇怪的类型
    // 这里丢一些奇怪的变量,照理来说其实是常量

    namespace BME280
    {
        struct bmeData {
            uint32_t pressure; // 实际24位
            uint32_t temperature;
            uint16_t humidity;
        };
        struct BME280_CalibData {
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

        int32_t BME280_compensate_T_int32(int32_t adc_T)
        {
            int32_t var1, var2, T;
            var1   = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
            var2   = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
            t_fine = var1 + var2;
            T      = (t_fine * 5 + 128) >> 8;
            return T;
        }
        // Q24.8
        uint32_t BME280_compensate_P_int64(int32_t adc_P)
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
        uint32_t bme280_compensate_H_int32(int32_t adc_H)
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

        // 需要测试TODO:
        void bme280_init(dl::IIC *bus)
        {
            uint8_t set = 0b10010011;

            uint8_t buf[34];

            bus->read(0x76, 0x88, buf, 24);

            bus->read(0x76, 0xA1, buf + 24, 1);

            bus->read(0x76, 0xE1, buf + 25, 8);

            memcpy(&calib, buf, 33);

            bus->write(0x76, 0xF4, &set, 1);

            // 4. 设置 config（待机时间 62.5ms，滤波器关闭）
            uint8_t config = 0b00101100; // t_sb = 001 (62.5ms), filter = 011 (8x)
            bus->write(0x76, 0xF5, &config, 1);
        }

        void bme280_read(dl::IIC *bus, bmeData *data)
        {

            uint8_t rawData[8];

            bus->read(0x76, 0xF7, rawData, 8);

            // 压力
            uint32_t press_raw = ((uint32_t)rawData[0] << 12) | ((uint32_t)rawData[1] << 4) | ((uint32_t)(rawData[2] >> 4));
            // 温度
            uint32_t temp_raw = ((uint32_t)rawData[3] << 12) | ((uint32_t)rawData[4] << 4) | ((uint32_t)(rawData[5] >> 4));

            // 湿度
            uint16_t hum_raw = ((uint16_t)rawData[6] << 8) | (uint16_t)rawData[7];

            data->humidity    = bme280_compensate_H_int32(hum_raw);
            data->pressure    = BME280_compensate_P_int64(press_raw);
            data->temperature = BME280_compensate_T_int32(temp_raw);
        }

    } // namespace BME280

} // namespace dl
