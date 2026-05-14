
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

#include <string>

#include "mpu/driver_mpu6050_fifo.h"

#include "mpu/driver_mpu6050.h"

#include "DL_AHRS/MadgwickAHRS.h"

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

int16_t (*accel_raw)[3] = new int16_t[128][3];
int16_t (*gyro_raw)[3]  = new int16_t[128][3];
float (*accel_g)[3]     = new float[128][3];
float (*gyro_dps)[3]    = new float[128][3];

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

int main()
{

    init();

    dl::USART usart1X(dUSART1_tA9_rA10, rx_buf, sizeof(rx_buf), 1, 1, 1520000);
    dl::Socket socket(usart1X);

    log_func = [&socket](const char *msg) {
        socket.sendString(msg);
    };
    logger << "Program Started: \n"
           << LCMD::FLUSH;

    dl::IIC bus(I2C1, 100000, I2C_Mode_I2C, I2C_DutyCycle_2, 0x51, I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit);

    MPU6050_IIC_BUS = &bus;

    if (mpu6050_fifo_init(
            MPU6050_ADDRESS_AD0_LOW) != 0) {
        socket.sendString("mpu9250_fifo_init error\n");
    }
    dl::delay_ms(300);
    logger << "Inited" << LCMD::FLUSH;
    uint16_t l = 128;
    dl::delay_ms(200);

    /*
    float gyro_bias[3];

    uint16_t cnt = 0;
    for (uint8_t i = 0; i < 64; i++) {
        if (i < 32) {
            continue;
        }

        if (mpu6050_fifo_read(accel_raw, accel_g, gyro_raw, gyro_dps, &l) != 0) {
            socket.sendString("mpu9250_dmp_read_all error\n");
        }
        gyro_bias[0] += dl::sum(gyro_dps[0], l);
        gyro_bias[1] += dl::sum(gyro_dps[1], l);
        gyro_bias[2] += dl::sum(gyro_dps[2], l);

        l = l > 8 ? 8 : l;
        for (uint8_t i = 0; i < l; i++) {
            logger << "AC:" << accel_g[i][0] << " " << accel_g[i][1] << " " << accel_g[i][2] << "\n";
            logger << "GR:" << gyro_dps[i][0]*0.09 << " " << gyro_dps[i][1]*0.09 << " " << gyro_dps[i][2] *0.09<< "\n";
            logger << LCMD::FLUSH;
        }

        logger << "bias: " << gyro_bias[0] << "  " << gyro_bias[1] << "  " << gyro_bias[2] << LCMD::NFLUSH;
        cnt += l;
        l = 128;
        dl::delay_ms(100);
    }
    gyro_bias[0] /= cnt;
    gyro_bias[1] /= cnt;
    gyro_bias[2] /= cnt;

    logger << "bias:  " << gyro_bias[0] << "  " << gyro_bias[1] << "  " << gyro_bias[2] << LCMD::NFLUSH;
    dl::delay_ms(1100);
    */

    dPA11 = 1;
    while (true) {
        if (mpu6050_fifo_read(accel_raw, accel_g, gyro_raw, gyro_dps, &l) != 0) {
            socket.sendString("mpu9250_dmp_read_all error\n");
        }
        l = l > 8 ? 8 : l;
        for (uint8_t i = 0; i < l; i++) {
            logger << "AC:" << accel_g[i][0] << " " << accel_g[i][1] << " " << accel_g[i][2] << "\n";
            logger << "GR:" << gyro_dps[i][0]*0.09<< " " << gyro_dps[i][1]*0.09 << " " << gyro_dps[i][2]*0.09 << "\n";
            logger << LCMD::FLUSH;
        }
        /*

        l = 128;
        logger << "quat4: " << q0 << ", " << q1 << ", " << q2 << ", " << q3 << LCMD::NFLUSH;
        float pitch = asinf(-2 * q1 * q3 + 2 * q0 * q2) * 57.3f;                                      
        float roll  = atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3f;      
        float yaw   = atan2f(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3f; /
        logger << "pitch " << pitch << "roll " << roll << "yaw " << yaw << LCMD::NFLUSH;

        */
    dl::delay_ms(50);
        // 四组，忽略
    }

    dl::delay_ms(50);
    while (1);
}
