
#include "stm32f4xx.h"
#include "dl_gpio.hpp"
#include "dl_nvic_it.h"
#include "dl_delay.hpp"
#include "dl_esp.hpp"
#include "dl_socket.hpp"
#include "dl_usart.hpp"
#include "dl_timer.hpp"
#include "dl_dma.hpp"
#include "dl_dshot.hpp"
#include "dl_iic.hpp"

#include "DL_AHRS/MadgwickAHRS.h"

#include <string>

#include "mpu/driver_mpu9250_basic.h"

#include "mpu/driver_mpu9250.h"
#define MAX_16_BITS 65536

#define CODE_END    while (1);

inline void init()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    debugInit;
    GPIO_ResetBits(debugGPIO);
}
// TX:PA9

static char str[300];
static uint8_t rx_buf[128];
/**
 * @brief 占用IO:
 *
 * 接线:
 * PA11 - Debug LED接灯
 * PE9  TIM  CH1  接信号线1
 * PE11 TIM  CH2
 * PE13 TIM  CH3
 * PE14 TIM  CH4
 *
 * 电调负极线与单片机负极线相连
 *
 * @return int
 */
#include "dl_log.h"
#include "dl_math.hpp"
void rcv_callback(uint8_t type)
{
}

// 定义校准参数 
float mag_offset[3] = {16.242731f, 41.162977f, -41.102233f};
float mag_scale[3][3] = {
    {0.020279f, -0.000733f, 0.000153f},
    {-0.001008f, 0.022500f, 0.000607f},
    {0.000232f, 0.000673f, 0.020377f}
};
float mag_norm_factor = 1.000000f;  // 实际上不需要，因为已经是1

// 校准函数
void calibrate_magnetometer(float mag_raw[3], float mag_cal[3])
{
    // 1. 减去偏移
    float x = mag_raw[0] - mag_offset[0];
    float y = mag_raw[1] - mag_offset[1];
    float z = mag_raw[2] - mag_offset[2];

    // 2. 应用缩放矩阵（矩阵乘法）
    mag_cal[0] = mag_scale[0][0] * x + mag_scale[0][1] * y + mag_scale[0][2] * z;
    mag_cal[1] = mag_scale[1][0] * x + mag_scale[1][1] * y + mag_scale[1][2] * z;
    mag_cal[2] = mag_scale[2][0] * x + mag_scale[2][1] * y + mag_scale[2][2] * z;

    // 3. 如果归一化因子不是1，则乘以归一化因子
    if (mag_norm_factor != 1.0f) {
        mag_cal[0] *= mag_norm_factor;
        mag_cal[1] *= mag_norm_factor;
        mag_cal[2] *= mag_norm_factor;
    }
}

int main()
{

    init();
    float g[3];
    float dps[3];
    float ut[3];
    float mag[3];
    float degrees;
    
    dl::USART usart1X(dUSART1_tA9_rA10, rx_buf, sizeof(rx_buf), 1, 1, 1520000);
    dl::Socket socket(usart1X);

    log_func = [&socket](const char *msg) {
        socket.sendString(msg);
    };
    logger << "Program Started: \n"
           << LCMD::FLUSH;

    dl::IIC bus(I2C1, 400000, I2C_Mode_I2C, I2C_DutyCycle_2, 0x51, I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit);

    MPU9250_IIC_BUS = &bus;

    if (mpu9250_basic_init(
            MPU9250_INTERFACE_IIC,
            MPU9250_ADDRESS_AD0_LOW) != 0) {
        socket.sendString("mpu9250_fifo_init error\n");
    }
    dl::delay_ms(300);
    logger << "Inited" << LCMD::FLUSH;

    while (0) {
        if (mpu9250_basic_read(g, dps, ut) != 0) {
            socket.sendString("mpu9250_read_all error\n");
        }
        logger << "UT: " << ut[0] << " " << ut[1] << " " << ut[2] << LCMD::NFLUSH;
    }

    float g_bias[3]   = {0.0f, 0.0f, 0.0f};
    float dps_bias[3] = {0.0f, 0.0f, 0.0f};

    dPA11             = 1;
    uint16_t bias_cnt = 256;
    for (uint16_t i = 0; i < bias_cnt; i++) {
        if (mpu9250_basic_read(g, dps, ut) != 0) {
            socket.sendString("mpu9250_read_all error\n");
        }
        g_bias[0] += g[0];
        g_bias[1] += g[1];
        g_bias[2] += g[2];

        dps_bias[0] += dps[0];
        dps_bias[1] += dps[1];
        dps_bias[2] += dps[2];

        logger << "Bias: " << i << LCMD::NFLUSH;
    }
    dPA11 = 0;
    g_bias[0] /= bias_cnt;
    g_bias[1] /= bias_cnt;
    g_bias[2] /= bias_cnt;

    g_bias[0] += 0;
    g_bias[1] += 0;
    g_bias[2] -= 1;

    dps_bias[0] /= bias_cnt;
    dps_bias[1] /= bias_cnt;
    dps_bias[2] /= bias_cnt;

    float d[3] = {0.0f, 0.0f, 0.0f};
    while (true) {
        dPA11 = 0;
        if (mpu9250_basic_read(g, dps, ut) != 0) {
            socket.sendString("mpu9250_dmp_read_all error\n");
        }
        calibrate_magnetometer(ut, mag);
        MadgwickAHRSupdate((dps[0] - dps_bias[0]) / 57.29578, (dps[1] - dps_bias[1]) / 57.29578, (dps[2] - dps_bias[2]) / 57.29578, g[0] - g_bias[0], g[1] - g_bias[1], g[2] - g_bias[2], mag[0], mag[1], mag[2]);

        logger << "AC:" << g[0] - g_bias[0] << " " << g[1] - g_bias[1] << " " << g[2] - g_bias[2] << " " << LCMD::NFLUSH;
        logger << "GR: " << dps[0] - dps_bias[0] << " " << dps[1] - dps_bias[1] << " " << dps[2] - dps_bias[2] << " " << LCMD::NFLUSH;
        d[0] += (dps[0] - dps_bias[0]) * 0.021f;
        d[1] += (dps[1] - dps_bias[1]) * 0.021f;
        d[2] += (dps[2] - dps_bias[2]) * 0.021f;
        logger << "UT: " << mag[0] << " " << mag[1] << " " << mag[2] << LCMD::NFLUSH;
        logger << "D:   " << d[0] << " " << d[1] << " " << d[2] << " " << LCMD::NFLUSH;

        float pitch = asinf(-2 * q1 * q3 + 2 * q0 * q2) * 57.3f;
        float roll  = atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3f;
        float yaw   = atan2f(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3f;
        logger << "YPR: " << yaw << " " << pitch << " " << roll << " " << LCMD::NFLUSH;
        dPA11 = 1;

        // 四组，忽略
    }
    while (1);
}
