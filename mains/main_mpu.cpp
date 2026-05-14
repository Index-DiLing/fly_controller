
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
#include "dl_bmp280.hpp"
#include "mpu/driver_mpu9250_dmp.h"
#include "mpu/driver_mpu9250_interface.h"

#include <string>
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
void rcv_callback(uint8_t type)
{
}

int main()
{
    init();
    // ESP RST PA3
    // DEBUG A11

    dl::USART usart1X(1520000, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, USART_Mode_Rx | USART_Mode_Tx, USART_HardwareFlowControl_None, 1, 1, USART1);
    dl::Socket socket(usart1X);

    socket.sendString("Start\n");

    dl::IIC bus(I2C1, 100000, I2C_Mode_I2C, I2C_DutyCycle_2, 0x51, I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit);

    uint8_t data = 0;

    
    MPU9250_IIC_BUS = &bus;
    
    if (mpu9250_dmp_init(
            MPU9250_INTERFACE_IIC,
            MPU9250_ADDRESS_AD0_LOW,
            rcv_callback,
            NULL,
            NULL) != 0) {
        socket.sendString("mpu9250_dmp_init error\n");
    }
    int16_t accel_raw[3], gyro_raw[3];
    float accel_g[3], gyro_dps[3];
    int32_t quat[4];
    float pitch, roll, yaw;
    uint16_t l = 1;
    dl::delay_ms(200);

    while (true)
    {
      /* code */
    if (mpu9250_dmp_read_all(&accel_raw, &accel_g, &gyro_raw, &gyro_dps, &quat, &pitch, &roll, &yaw, &l) != 0) {
        socket.sendString("mpu9250_dmp_read_all error\n");
    }
    sprintf(str, "accel_raw: %d %d %d\n  accel_g: %.2f %.2f %.2f\n  gyro_raw: %d %d %d\n  gyro_dps: %.2f %.2f %.2f\n  quat: %d %d %d %d\n  pitch: %.2f\n  roll: %.2f\n  yaw: %.2f\n",
            accel_raw[0], accel_raw[1], accel_raw[2],
            accel_g[0], accel_g[1], accel_g[2],
            gyro_raw[0], gyro_raw[1], gyro_raw[2],
            gyro_dps[0], gyro_dps[1], gyro_dps[2],
            quat[0], quat[1], quat[2], quat[3],
            pitch, roll, yaw);
    socket.sendString(str);
    dl::delay_ms(20);
    }

    /*
    while (1)
    {
      socket.sendString("memory:\n");
      for (uint8_t i = 0; i < 8; i++)
      {
        for (uint8_t j = 0; j < 8; j++)
        {

        socket.sendCommand(0xA0 + doc.realTimeData[i*8 + j]);

        }
        socket.sendString("\n");

      }
      socket.sendString("----\n");
    }
    */
    while (1);
}
