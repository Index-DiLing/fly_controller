#pragma once
#include "stm32f4xx.h"
#include "dl_iic.hpp"
namespace dl
{
    namespace MPU9250
    {

        void init(IICDevice &mpu)
        {
            mpu.write(0x6B, 0x80); // 清除之前的设置

            dl::delay_ms(50);//休息一下

            mpu.write(0x6B, 0x01); // 设置电源模式/时钟源 自动采用最佳时钟

            mpu.write(0X1B, 0x18); //量程2000dps,无自检,FS_SEL=11

            mpu.write(0x1C, 0x00); //量程2g,无自检

            mpu.write(0x19, 0x03); // SMPLRT_DIV=3，采样率 = 内部采样率/(1+3)

            mpu.write(0x1A, 0x03); // CONFIG[2:0]=011，陀螺仪带宽 41Hz，加速度带宽 41Hz

            mpu.write(0x6C, 0x00); // PWR_MGMT_2，所有轴都开启

            mpu.write(0x6A, 0x20); // USER_CTRL[5]=1，使能 I2C 主模式

            mpu.write(0x37, 0x02); // INT_PIN_CFG[1]=1，使能旁路模式，允许主机直接访问 AK8963

        }

    }
    // namespace MPU9250

} // namespace dl
