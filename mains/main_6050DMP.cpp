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

#include "mpu/driver_mpu6050_dmp.h"

#include "mpu/driver_mpu6050.h"
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
int32_t (*gs_quat)[4]   = new int32_t[128][4];
static float gs_pitch[128];
static float gs_roll[128];
static float gs_yaw[128];

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
static void a_dmp_orient_callback(uint8_t orientation)
{
}
static void a_dmp_tap_callback(uint8_t count, uint8_t direction)
{
}
static void a_receive_callback(uint8_t type)
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

    if (mpu6050_dmp_init(
            MPU6050_ADDRESS_AD0_LOW, a_receive_callback, a_dmp_tap_callback, a_dmp_orient_callback) != 0) {
        socket.sendString("mpu9250_dmp_init error\n");
    }
    dl::delay_ms(200);
    logger << "Inited" << LCMD::FLUSH;

    uint16_t len = 128;
    dPA11 = 1;
    while (true) {
        if (mpu6050_dmp_read_all(accel_raw, accel_g,
                                 gyro_raw, gyro_dps,
                                 gs_quat,
                                 gs_pitch, gs_roll, gs_yaw,
                                 &len) != 0) {
            (void)mpu6050_dmp_deinit();
            return 1;
        }
        for (uint8_t i = 0; i < len; i++) {
            logger << "AC:" << accel_g[i][0] << " " << accel_g[i][1] << " " << accel_g[i][2] << "\n";
            logger << "GR:" << gyro_dps[i][0] << " " << gyro_dps[i][1] << " " << gyro_dps[i][2] << "\n";
            //打印四元数
            logger << gs_pitch[i] << " " << gs_roll[i] << " " << gs_yaw[i]<< "\n";
            logger << LCMD::FLUSH;
        }
        len = 128;
        dl::delay_ms(200);
        // 四组，忽略
    }
    while (1);
}
