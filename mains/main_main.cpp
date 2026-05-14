
#include "stm32f4xx.h"
#include "dl_gpio.hpp"
#include "dl_nvic_it.h"
#include "dl_delay.hpp"
#include "dl_socket.hpp"
#include "dl_usart.hpp"
#include "dl_iic.hpp"
#include "global.h"
#include "dl_imu.hpp"

#include "dl_dshot.hpp"

#include "DL_AHRS/MadgwickAHRS.h"

#define MAX_16_BITS 65536

#define CODE_END    while (1);
inline void GPIOClockInit()
{

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
}

inline void init()
{
    GPIOClockInit();

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Config(SystemCoreClock / 1000);

    dl::delay_ms(100);
}
// TX:PA9

static uint8_t rx_buf[256];
/**
 * @brief 占用IO:
 *
 * 接线:
 * PA11 - Debug LED接灯
 * PE9  TIM  CH1  接信号线176
 * PE11 TIM  CH2
 * PE13 TIM  CH3
 * PE14 TIM  CH4
 *
 * 电调负极线与单片机负极线相连
 *
 * @return int
 */
#include "dl_log.h"
#include "dl_message.hpp"

int main()
{

    init();
    dl::USART usart1X(dUSART1_tA9_rA10, rx_buf, sizeof(rx_buf), 1, 1, 1520000);
    dl::Socket socket(usart1X);
    log_func = [&socket](const char *msg) {
        socket.sendString(msg);
    };
    logger << "Hello, world!" << LCMD::NFLUSH;

    dl::GPIO_Pin_Init(dGPIOFPin3, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    dl::GPIO_Pin_Init(dGPIOFPin4, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    dl::GPIO_Pin_Init(dGPIOFPin5, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);

    dPF3 = 1;
    dPF4 = 1;
    dPF5 = 1;

    dl::MessageManager msg([&usart1X](uint8_t *data, uint16_t len) {
        usart1X.send(data, len);
    });

    uint8_t key[6];
    while (usart1X.rx_buf_size() < 6) {
    }
    usart1X.read(key, 6);
    dl::DShot ds(dl::DSHOT_TIM::DSHOT_TIM1, dl::DSHOT_RATE::DSHOT_300);

    ds.start();
    dPF3               = 0;
    dPF4               = 0;
    uint16_t dvalue[4] = {200, 200, 200, 200};
    ds.encodePreLoadDshotData(dvalue);
    
    dPF5=0;
    
    logger<<"MOTORS ON"<<LCMD::NFLUSH;
    uint16_t cnt = 0;
    while (cnt<150) {

        // while (usart1X.rx_buf_size() < 6) {
        // }
        // usart1X.read(key, 6);
        for (uint16_t i = 0; i < 10; i++)
        {
            dvalue[0] +=1;
            dvalue[1] +=1;
            dvalue[2] +=1;
            dvalue[3] +=1;
            dl::delay_ms(10);
            ds.encodePreLoadDshotData(dvalue);
        }
        dl::delay_ms(200);
        logger << "dvalue: " << dvalue[0] << " " << dvalue[1] << " " << dvalue[2] << " " << dvalue[3] << LCMD::NFLUSH;
        logger << "cnt: " << cnt << LCMD::NFLUSH;
        cnt++;
    }

    dl::IIC bus(dIIC2_cB10_dB11);
    dl::IMUGData data;
    dl::MPU9250 mpu(MPU9250_ADDRESS_AD0_LOW, &data);

    mpu.init(256, &bus);

    dl::delay_ms(100);

    while (true) {
        mpu.read();
        MadgwickAHRSupdate(data.gyro[0] / 57.29578, data.gyro[1] / 57.29578, data.gyro[2] / -57.29578, data.accel[0], data.accel[1], data.accel[2], data.mag[0], data.mag[1], data.mag[2]);
        // msg.sendAttitudeMessage(q0,q1,q2,q3);

        dl::delay_ms(10);
    }
}
