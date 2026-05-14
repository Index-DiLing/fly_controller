
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

#include "DL_AHRS/gyro_ekf.h"

#include "DL_AHRS/MadgwickAHRS.h"

#include <string>

#include "mpu/driver_mpu9250_fifo.h"

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

int16_t (*accel_raw)[3]  = new int16_t[128][3];
int16_t (*gyro_raw)[3]   = new int16_t[128][3];
float (*accel_g)[3]      = new float[128][3];
float (*gyro_dps)[3]     = new float[128][3];
int16_t (*gs_mag_raw)[3] = new int16_t[128][3];
float (*gs_mag_ut)[3]    = new float[128][3];

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

float (*gyro_filtered)[128]  = new float[3][128];
float (*accel_filtered)[128] = new float[3][128];

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

    MPU9250_IIC_BUS = &bus;

    if (mpu9250_fifo_init(
            MPU9250_INTERFACE_IIC,
            MPU9250_ADDRESS_AD0_LOW) != 0) {
        socket.sendString("mpu9250_fifo_init error\n");
    }
    dl::delay_ms(300);
    logger << "Inited" << LCMD::FLUSH;

    float accel[3];
    float gyro[3];

    // int32_t quat[4];
    // float pitch, roll, yaw;

    float gyro_bias[3];
    float accel_bias[3];

    uint16_t l = 128;
    dl::delay_ms(200);

    gyro_ekf_t gyro_ekf;
    gyro_ekf_init(&gyro_ekf);

    dPA11 = 1;
    while(true){
        if (mpu9250_fifo_read(accel_raw, accel_g, gyro_raw, gyro_dps, gs_mag_raw, gs_mag_ut, &l) != 0) {
            socket.sendString("mpu9250_dmp_read_all error\n");
        }
        l = l > 16 ? 16 : l;
        for (uint8_t i = 0; i < l; i++) {
            logger << "AC:" << accel_g[i][0] << " " << accel_g[i][1] << " " << accel_g[i][2] << "\n";
            logger << "GR:" << gyro_dps[i][0]*0.09 << " " << gyro_dps[i][1]*0.09 << " " << gyro_dps[i][2]*0.09 << "\n";
            logger << LCMD::FLUSH;
        }
        l = 128;
        dl::delay_ms(50);
        // 四组，忽略
    }


    logger["Keep Stand now"];




    // 四轮校准
    uint16_t biasC[2][3];
    for (uint8_t ini = 0; ini < 8; ini++) {
        if (mpu9250_fifo_read(accel_raw, accel_g, gyro_raw, gyro_dps, gs_mag_raw, gs_mag_ut, &l) != 0) {
            socket.sendString("mpu9250_dmp_read_all error\n");
        }
        uint8_t lenx[2][3]; // lenx[0][φ]是φ轴角速度
        l         = 128;
        uint8_t c = l;
        while (c--) {
            gyro_filtered[0][c] = gyro_dps[c][0];
            gyro_filtered[1][c] = gyro_dps[c][1];
            gyro_filtered[2][c] = gyro_dps[c][2];

            accel_filtered[0][c] = accel_g[c][0];
            accel_filtered[1][c] = accel_g[c][1];
            accel_filtered[2][c] = accel_g[c][2];
        }
        for (uint8_t i = 0; i < 3; i++) {
            lenx[0][i] = dl::removeOutliers(gyro_filtered[i], l);
            biasC[0][i] += lenx[0][i];
        }

        for (uint8_t i = 0; i < 3; i++) {
            lenx[1][i] = dl::removeOutliers(accel_filtered[i], l);
            biasC[1][i] += lenx[1][i];
        }
        for (uint8_t i = 0; i < 3; i++) {
            for (uint8_t j = 0; j < lenx[0][i]; j++) {
                gyro_bias[i] += gyro_filtered[i][j];
            }
            for (uint8_t j = 0; j < lenx[1][i]; j++) {
                accel_bias[i] += accel_filtered[i][j];
            }
        }
        l = 128;
        dl::delay_ms(100);
        logger<<"i = "<<ini<<LCMD::NFLUSH;
    }
    
    for (uint8_t i = 0; i < 3; i++) {
        gyro_bias[i] /= biasC[0][i];
        accel_bias[i] /= biasC[1][i];
    }
    accel_bias[2] -= 1.0f;
    logger << "gyro_bias: " << gyro_bias[0] << " " << gyro_bias[1] << " " << gyro_bias[2] << LCMD::NFLUSH;
    logger << "accel_bias: " << accel_bias[0] << " " << accel_bias[1] << " " << accel_bias[2] << LCMD::NFLUSH;
    
    dl::delay_ms(1000);
    
    logger << "START" << LCMD::NFLUSH;

    while (true) {
        /* code */
        if (mpu9250_fifo_read(accel_raw, accel_g, gyro_raw, gyro_dps, gs_mag_raw, gs_mag_ut, &l) != 0) {
            socket.sendString("mpu9250_dmp_read_all error\n");
        }
        // gyro_dps[0]-=gyro_bias[0];
        // gyro_dps[1]-=gyro_bias[1];
        // gyro_dps[2]-=gyro_bias[2];
        // gyro_ekf_update(&gyro_ekf,gyro_dps,gyro_filtered);
        // MadgwickAHRSupdateIMU(accel_g[0],accel_g[1],accel_g[2],gyro_filtered[0],gyro_filtered[1],gyro_filtered[2]);

        uint8_t lenx[2][3]; // lenx[0][φ]是φ轴角速度
        l         = 128;
        uint8_t c = l;
        while (c--) {
            gyro_filtered[0][c] = gyro_dps[c][0];
            gyro_filtered[1][c] = gyro_dps[c][1];
            gyro_filtered[2][c] = gyro_dps[c][2];

            accel_filtered[0][c] = accel_g[c][0];
            accel_filtered[1][c] = accel_g[c][1];
            accel_filtered[2][c] = accel_g[c][2];
        }

        for (uint8_t i = 0; i < 3; i++) {
            lenx[0][i] = dl::removeOutliers(gyro_filtered[i], l);
        }

        for (uint8_t i = 0; i < 3; i++) {
            lenx[1][i] = dl::removeOutliers(accel_filtered[i], l);
        }
        for (uint8_t i = 0; i < 3; i++) {
            for (uint8_t j = 0; j < lenx[0][i]; j++) {
                gyro[i] += gyro_filtered[i][j];
            }
            gyro[i] /= lenx[0][i];
            for (uint8_t j = 0; j < lenx[1][i]; j++) {
                accel[i] += accel_filtered[i][j];
            }
            accel[i] /= lenx[1][i];
        }
        logger << "accel:" << accel[0] - accel_bias[0] << " " << accel[1] - accel_bias[1] << " " << accel[2] - accel_bias[2] << LCMD::NFLUSH;
        logger << "gyro:" << gyro[0] - gyro_bias[0] << " " << gyro[1] - gyro_bias[1]-95.0f << " " << gyro[2] - gyro_bias[2] << LCMD::NFLUSH;
        logger << "l=" << l << LCMD::NFLUSH;
        l = 128;
        // logger<<"quat4:"<<q0<<" "<<q1<<" "<<q2<<" "<<q3<<"\n"<<LCMD::FLUSH;

        /*
        sprintf(str, "accel_raw: %d %d %d\n  accel_g: %.2f %.2f %.2f\n  gyro_raw: %d %d %d\n  gyro_dps: %.2f %.2f %.2f\n  gs_mag_raw: %d %d %d\n  gs_mag_ut: %.2f %.2f %.2f\n",
                accel_raw[0], accel_raw[1], accel_raw[2],
                accel_g[0], accel_g[1], accel_g[2],
                gyro_raw[0], gyro_raw[1], gyro_raw[2],
                gyro_dps[0], gyro_dps[1], gyro_dps[2],
                gs_mag_raw[0], gs_mag_raw[1], gs_mag_raw[2],
                gs_mag_ut[0], gs_mag_ut[1], gs_mag_ut[2]);
        */

        dl::delay_ms(50);
    }
    while (1);
}
