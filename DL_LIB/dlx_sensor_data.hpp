#pragma once
#include <stdint.h>
namespace dlx
{
    struct AccelerometerRaw {
        uint16_t data[3];
    };
    struct AccelerometerG {
        float data[3];
    };

    struct GyroscopeRaw {
        uint16_t data[3];
    };

    struct GyroscopeDps {
        float data[3];
    };

    struct GyroscopeRads {
        float data[3];
    };

    struct StandardIMURaw {
        AccelerometerRaw accel;
        GyroscopeRaw gyro;
    };

    struct StandardIMUGDps {
        AccelerometerG accel;
        GyroscopeDps gyro;
    };

    struct StandardIMUGRads {
        AccelerometerG accel;
        GyroscopeRads gyro;
    };

    // BME280 环境传感器原始值(ADC 原始码, 未修正)
    struct EnviromentRaw {
        uint32_t pressure;    // 气压原始值, 实际 20bit
        uint32_t temperature; // 温度原始值, 实际 20bit
        uint16_t humidity;    // 湿度原始值, 16bit
    };

    struct Quaternion {
        float data[4];
    };

    // GNSS 本地切平面位置 (ENU: 东/北/天 [m], 原点在起飞点)
    struct GnssPosition {
        float data[3];
    };

    // GNSS 地速 (ENU [m/s])
    struct GnssVelocity {
        float data[3];
    };

} // namespace dlx
