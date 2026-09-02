#pragma once
#include <stdint.h>
#include <math.h>
#include "dlx_iic.hpp"
#include "dlx_delay.hpp"
#include "dlx_sensor_data.hpp"
#include "dlx_bme280_config.h"

namespace dlx
{
    /**
     * @brief BME280 温湿度气压传感器驱动(I2C 模式, 基于 dlx IICBus/IICDevice)
     *
     * 用法: 先按 dlx_iic 的工厂方法建好总线并 init, 再构造本类:
     *   IICBus bus = IICBus::IIC2_SB10_DB11();
     *   bus.init(IICBusModeProfile::ACK_E_DC16_9_ADDR7, 400000, 0x51);
     *   BME280 bme(bus);          // 默认地址 0x76
     *   bme.init();
     *   EnviromentRaw raw = bme.getRaw();
     *   float t = bme.getTemperature(raw); // °C
     *   float p = bme.getPressure(raw);    // Pa
     *   float h = bme.getHumidity(raw);    // %RH
     *   float a = bme.getAltitude(raw);    // m
     *
     * 修正函数只依赖 EnviromentRaw + 校准参数, 可以随时对同一份原始数据重复调用.
     */
    class BME280
    {
    private:
        IICDevice dev;

        /** 校准参数(0x88/0xA1/0xE1 读出) */
        struct CalibData {
            uint16_t dig_T1;
            int16_t  dig_T2;
            int16_t  dig_T3;
            uint16_t dig_P1;
            int16_t  dig_P2;
            int16_t  dig_P3;
            int16_t  dig_P4;
            int16_t  dig_P5;
            int16_t  dig_P6;
            int16_t  dig_P7;
            int16_t  dig_P8;
            int16_t  dig_P9;
            uint8_t  dig_H1;
            int16_t  dig_H2;
            int8_t   dig_H3;
            int16_t  dig_H4;
            int16_t  dig_H5;
            int8_t   dig_H6;
        } calib;

        int32_t t_fine = 0; // 温度补偿中间量, 压力/湿度补偿复用它

        /** 写一个寄存器 */
        void writeReg(BME280_Reg reg, uint8_t value)
        {
            uint8_t buf[2] = { static_cast<uint8_t>(reg), value };
            ByteBuffer w(buf, 2);
            dev.write(w);
        }

        /** 从 reg 开始突发读 n 字节 */
        void readRegs(BME280_Reg reg, uint8_t *out, uint16_t n)
        {
            uint8_t addr[1] = { static_cast<uint8_t>(reg) };
            ByteBuffer w(addr, 1);
            ByteBuffer r(out, n);
            dev.read(r, w, n);
        }

        uint8_t readReg(BME280_Reg reg)
        {
            uint8_t v;
            readRegs(reg, &v, 1);
            return v;
        }

        /** 读取全部校准参数并解析(含 H4/H5 的 12bit 符号扩展) */
        void readCalib()
        {
            uint8_t buf[24];
            readRegs(BME280_Reg::CalibStart, buf, 24);
            calib.dig_T1 = (uint16_t)((uint16_t)buf[1] << 8 | buf[0]);
            calib.dig_T2 = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
            calib.dig_T3 = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
            calib.dig_P1 = (uint16_t)((uint16_t)buf[7] << 8 | buf[6]);
            calib.dig_P2 = (int16_t)((uint16_t)buf[9] << 8 | buf[8]);
            calib.dig_P3 = (int16_t)((uint16_t)buf[11] << 8 | buf[10]);
            calib.dig_P4 = (int16_t)((uint16_t)buf[13] << 8 | buf[12]);
            calib.dig_P5 = (int16_t)((uint16_t)buf[15] << 8 | buf[14]);
            calib.dig_P6 = (int16_t)((uint16_t)buf[17] << 8 | buf[16]);
            calib.dig_P7 = (int16_t)((uint16_t)buf[19] << 8 | buf[18]);
            calib.dig_P8 = (int16_t)((uint16_t)buf[21] << 8 | buf[20]);
            calib.dig_P9 = (int16_t)((uint16_t)buf[23] << 8 | buf[22]);

            uint8_t h1;
            readRegs(BME280_Reg::CalibH1, &h1, 1);
            calib.dig_H1 = h1;

            uint8_t hb[7];
            readRegs(BME280_Reg::CalibH2, hb, 7);
            calib.dig_H2 = (int16_t)((uint16_t)hb[1] << 8 | hb[0]);
            calib.dig_H3 = (int8_t)hb[2];
            // H4/H5 是 12bit 有符号数: 高 8 位在 hb[3]/hb[5], 低 4 位在 hb[4]
            int16_t h4 = (int16_t)((uint16_t)(hb[3] << 4) | (hb[4] & 0x0F));
            int16_t h5 = (int16_t)((uint16_t)(hb[5] << 4) | (hb[4] >> 4));
            calib.dig_H4 = (h4 & 0x0800) ? (int16_t)(h4 | 0xF000) : h4; // 12bit 符号扩展
            calib.dig_H5 = (h5 & 0x0800) ? (int16_t)(h5 | 0xF000) : h5;
            calib.dig_H6 = (int8_t)hb[6];
        }

        /* ==================== 修正算法(Bosch 官方公式) ==================== */

        /** 温度修正, 返回 0.01°C 定点值, 同时更新 t_fine 供压力/湿度使用 */
        int32_t compensateTemperature(int32_t adc_T)
        {
            int32_t var1, var2, T;
            var1   = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
            var2   = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
            t_fine = var1 + var2;
            T      = (t_fine * 5 + 128) >> 8;
            return T; // Q24.8, 实际单位 0.01°C
        }

        /** 气压修正, 返回 Pa*256 定点值(Q24.8) */
        uint32_t compensatePressure(int32_t adc_P)
        {
            int64_t var1, var2, p;
            var1 = ((int64_t)t_fine) - 128000;
            var2 = var1 * var1 * (int64_t)calib.dig_P6;
            var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
            var2 = var2 + (((int64_t)calib.dig_P4) << 35);
            var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
            var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
            if (var1 == 0) {
                return 0; // 避免除零
            }
            p    = 1048576 - adc_P;
            p    = (((p << 31) - var2) * 3125) / var1;
            var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
            var2 = (((int64_t)calib.dig_P8) * p) >> 19;
            p    = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
            return (uint32_t)p;
        }

        /** 湿度修正, 返回 %RH*1024 定点值(Q22.10) */
        uint32_t compensateHumidity(int32_t adc_H)
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

    public:
        /**
         * @param bus 已 init 的 I2C 总线
         * @param address 7 位从机地址, 默认 0x76(SDO 接低)
         */
        BME280(IICBus &bus, uint16_t address = BME280_IIC_ADDRESS) : dev(bus, address)
        {
        }

        ~BME280()
        {
        }

        /**
         * @brief 软复位 -> 校验 ID -> 读校准参数 -> 应用 BME280_Config 配置
         * @return true 表示芯片正常
         */
        bool init()
        {
            writeReg(BME280_Reg::Reset, 0xB6);
            delay_ms(2); // 复位后约 2ms 内不可访问

            if (readReg(BME280_Reg::ChipId) != 0x60) {
                return false;
            }

            readCalib();

            // 湿度控制需在 ctrl_meas 之前写入才生效
            writeReg(BME280_Reg::CtrlHum, static_cast<uint8_t>(BME280_Config::CtrlHum));
            writeReg(BME280_Reg::CtrlMeas, static_cast<uint8_t>(BME280_Config::CtrlMeas));
            writeReg(BME280_Reg::Config, static_cast<uint8_t>(BME280_Config::Config));
            return true;
        }

        /**
         * @brief 阻塞读取气压/温度/湿度原始 ADC 值
         * @return EnviromentRaw 三个原始值, 可交给下面的修正函数换算
         */
        EnviromentRaw getRaw()
        {
            uint8_t raw[8]; // 0xF7~0xFE
            readRegs(BME280_Reg::PressMsb, raw, 8);

            EnviromentRaw r;
            r.pressure    = ((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | ((uint32_t)(raw[2] >> 4));
            r.temperature = ((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | ((uint32_t)(raw[5] >> 4));
            r.humidity    = (uint16_t)(((uint16_t)raw[6] << 8) | raw[7]);
            return r;
        }

        /** @brief 温度修正: °C */
        float getTemperature(EnviromentRaw raw)
        {
            return (float)compensateTemperature((int32_t)raw.temperature) / 100.0f;
        }

        /** @brief 气压修正: Pa */
        float getPressure(EnviromentRaw raw)
        {
            (void)compensateTemperature((int32_t)raw.temperature); // 先算 t_fine
            return (float)compensatePressure((int32_t)raw.pressure) / 256.0f;
        }

        /** @brief 湿度修正: %RH */
        float getHumidity(EnviromentRaw raw)
        {
            (void)compensateTemperature((int32_t)raw.temperature);
            return (float)compensateHumidity((int32_t)raw.humidity) / 1024.0f;
        }

        /**
         * @brief 高度修正: m(基于气压高度公式)
         * @param seaLevelPressure 海平面气压, 默认 101325Pa
         */
        float getAltitude(EnviromentRaw raw, float seaLevelPressure = 101325.0f)
        {
            return 44330.0f * (1.0f - powf(getPressure(raw) / seaLevelPressure, 0.190295f));
        }
    };
} // namespace dlx
