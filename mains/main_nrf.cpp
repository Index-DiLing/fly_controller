// For test SD

#include "stm32f4xx.h"
#include "dl_gpio.hpp"
#include "dl_nvic_it.h"
#include "dl_delay.hpp"
#include "dl_socket.hpp"
#include "dl_usart.hpp"
#include "global.h"

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

    dl::GPIO_Pin_Init(dGPIOFPin3, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    dl::GPIO_Pin_Init(dGPIOFPin4, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    dl::GPIO_Pin_Init(dGPIOFPin5, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);
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
#include "dl_error.hpp"
#include "dl_spi.hpp"
#include "dl_nrf24l01.hpp"

void fuc(dl::SPI &spix)
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
    dl::delay_ms(1000);
    logger << "Hello, world!" << LCMD::NFLUSH;

    dl::SPI spi2X(dSPI2_SB13_MIB14_MISOB15, dGPIOCPin9);

    uint8_t NRF24L01_TxAddress[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
    uint8_t NRF24L01_RxAddress[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
    dl::nrf24l01 nrf(spi2X, dGPIOCPin1, NRF24L01_RxAddress, NRF24L01_TxAddress,4,16);

    nrf.init();
    logger << "inited" << LCMD::NFLUSH;
    logger<<"CONFIG:"<< nrf.readRegister(dl::NRF24L01_REGISTERS::CONFIG) << LCMD::NFLUSH;
    dl::delay_ms(1000);
    uint8_t nrfBuf[32];
    uint16_t cnt = 256;
    uint32_t ti = SystemClockMilliseconds;
    while (1) {
        nrf.receive(nrfBuf);

        if (cnt-- ==0)
        {
            logger<<SystemClockMilliseconds-ti<<LCMD::NFLUSH;
            ti = SystemClockMilliseconds;
            cnt = 256;
        }
        
        
        
    }
}
