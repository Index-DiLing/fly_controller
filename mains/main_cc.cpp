
#include "stm32f4xx.h"
#include "dl_gpio.hpp"
#include "dl_nvic_it.h"
#include "dl_delay.hpp"
#include "dl_socket.hpp"
#include "dl_usart.hpp"
#include <string>

#define MAX_16_BITS 65536

#define CODE_END    while (1);

inline void init()
{
    //这里是需要开启所有要用的GPIO
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    SysTick_Config(SystemCoreClock / 1000);
    debugInit;
    GPIO_ResetBits(debugGPIO);
}
// TX:PA9

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

int main()
{

    init();

    dl::USART usart1X(dUSART1_tA9_rA10, rx_buf, sizeof(rx_buf), 1, 1, 1520000);
    dl::Socket socket(usart1X);

    log_func = [&socket](const char *msg) {
        socket.sendString(msg);
    };
    bool x = 0;
    while (1)
    {
        logger << "Hello, world!\n"<< LCMD::FLUSH;
        
        dl::delay_ms(1000);
    }
    


    logger << "Program Started: \n"
           << LCMD::FLUSH;

    while (1);
}
