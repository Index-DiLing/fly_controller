#pragma once
#include "stm32f4xx.h"

#include "mpu/driver_mpu9250_basic.h"

namespace dl
{

    struct IMUGData {
        float accel[3];
        float gyro[3];
        float mag[3];
        float accelBias[3];
        float gyroBias[3];
        void calibrate()
        {
            // 磁力计校准参数针对于这个MPU9250
            accel[0] -= accelBias[0];
            accel[1] -= accelBias[1];
            // accel[2] -= accelBias[2];
            gyro[0] -= gyroBias[0];
            gyro[1] -= gyroBias[1];
            gyro[2] -= gyroBias[2];

            constexpr float mag_norm_factor = 1.000000f; // 实际上不需要，因为已经是1
            constexpr float mag_offset[3]   = {9.359709f, 14.870286f, -25.516094f};
            constexpr float mag_scale[3][3] = {
                {0.034009f, 0.001110f, 0.010324f},
                {0.000643f, 0.036398f, -0.000965f},
                {0.008008f, -0.001293f, 0.032011f},
            };
            // 1. 减去偏移
            float x = mag[0] - mag_offset[0];
            float y = mag[1] - mag_offset[1];
            float z = mag[2] - mag_offset[2];

            // 2. 应用缩放矩阵（矩阵乘法）
            mag[0] = mag_scale[0][0] * x + mag_scale[0][1] * y + mag_scale[0][2] * z;
            mag[1] = mag_scale[1][0] * x + mag_scale[1][1] * y + mag_scale[1][2] * z;
            mag[2] = mag_scale[2][0] * x + mag_scale[2][1] * y + mag_scale[2][2] * z;

            // 3. 如果归一化因子不是1，则乘以归一化因子
            if (mag_norm_factor != 1.0f) {
                mag[0] *= mag_norm_factor;
                mag[1] *= mag_norm_factor;
                mag[2] *= mag_norm_factor;
            }
        }
    };

    struct IMUData {
        float accel[3];
        float gyro[3];
        float accelBias[3];
        float gyroBias[3];
        void calibrate()
        {
            // 磁力计校准参数针对于这个MPU9250
            accel[0] -= accelBias[0];
            accel[1] -= accelBias[1];
            accel[2] -= accelBias[2];
            gyro[0] -= gyroBias[0];
            gyro[1] -= gyroBias[1];
            gyro[2] -= gyroBias[2];
        }
    };

    class IMU
    {
    public:
        virtual bool read() = 0;
        virtual ~IMU() {};
    };

    class MPU9250 : public IMU
    {
    private:
        IMUGData *data;
        mpu9250_address_t address;

    public:
        // 初始化分配
        MPU9250(mpu9250_address_t addr, IMUGData *idata) : address(addr), data(idata) {};
        bool init(uint16_t calibrationCnt, IIC *bus);
        bool read() override; //
        ~MPU9250();
    };

    bool MPU9250::init(uint16_t calibrationCnt, IIC *bus)
    {

        MPU9250_IIC_BUS = bus;
        if (mpu9250_basic_init(MPU9250_INTERFACE_IIC, address) != 0) {
            return false;
        }
        dl::delay_ms(300);

        // logger<< "MPU9250 Initing..." << LCMD::NFLUSH;

        DEBUG_FUNC(logger << "MPU9250 Inited" << LCMD::FLUSH;);
        DEBUG_FUNC(logger << "Calibrating MPU9250..." << LCMD::FLUSH;);
        for (uint16_t i = 0; i < calibrationCnt; i++) {
            float g[3] = {0}, dps[3] = {0}, ut[3] = {0};
            if (mpu9250_basic_read(g, dps, ut) != 0) {
                return false;
            }
            data->accelBias[0] += g[0];
            data->accelBias[1] += g[1];
            data->accelBias[2] += g[2];

            data->gyroBias[0] += dps[0];
            data->gyroBias[1] += dps[1];
            data->gyroBias[2] += dps[2];

            DEBUG_FUNC(logger << "Calibrating MPU9250: " << i << LCMD::NFLUSH;);
        }

        data->accelBias[0] /= calibrationCnt;
        data->accelBias[1] /= calibrationCnt;
        data->accelBias[2] /= calibrationCnt;

        data->accelBias[0] += 0;
        data->accelBias[1] += 0;
        data->accelBias[2] += 0; // z轴分量为重力加速度

        data->gyroBias[0] /= calibrationCnt;
        data->gyroBias[1] /= calibrationCnt;
        data->gyroBias[2] /= calibrationCnt;

        DEBUG_FUNC(logger << "MPU9250 Calibrated" << LCMD::FLUSH;);

        return true;
    }

    bool MPU9250::read()
    {
        if (mpu9250_basic_read(data->accel, data->gyro, data->mag) != 0) {
            return false;
        }
        data->calibrate();

        return true;
    }

    MPU9250::~MPU9250()
    {
        free(data);
    }

} // namespace dl
