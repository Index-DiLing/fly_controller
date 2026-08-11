// For test SD

#include "stm32f4xx.h"
#include "dl_gpio.hpp"
#include "dl_nvic_it.h"
#include "dl_delay.hpp"
#include "dl_socket.hpp"
#include "dl_usart.hpp"
#include "global.h"
#include "dl_bytebuffer.hpp"
#include "rt-thread/include/rthw.h"
#include "rt-thread/bsp/rtconfig.h"
inline void GPIOClockInit()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
}
volatile uint32_t SystemClockMilliseconds = 0;

inline void init()
{

    GPIOClockInit();


    dl::GPIO_Pin_Init(dGPIOFPin3, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);

    dl::GPIO_Pin_Init(dGPIOFPin4, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);

    dl::GPIO_Pin_Init(dGPIOFPin5, GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL);

    dPF3 = 1;
    dPF4 = 0;
    dPF5 = 1;
}
// TX:PA9
static uint8_t rx_buf[256];

#include "dl_log.h"

auto &allocator = dl::StaticByteBufferAllocator<4096>::instance();

int main()
{
    init();
    dl::USART usart1X(dUSART1_tA9_rA10, rx_buf, sizeof(rx_buf), 1, 1, 1520000);

    dl::Socket socket(usart1X);

    dPF3 = 0;

    socket.sendString("Hello,RT-Thread!\n");
    
    dPF3 = 1;
    
    auto t1 = rt_thread_create("rt1", +[](void *ctx) {
        while (true)
        {
            dPF3 = 0;
            rt_thread_mdelay(100);
            dPF3 = 1;
            rt_thread_mdelay(100);
        } }, nullptr, 256, 4, 10);

    auto t2 = rt_thread_create("rt2", +[](void *ctx) {
        while (true)
        {
            dPF4 = 0;
            rt_thread_mdelay(150);
            dPF4 = 1;
            
            rt_thread_mdelay(150);
        } }, nullptr, 256, 4, 10);

    auto t3 = rt_thread_create("rt3", +[](void *ctx) {
        while (true)
        {
            dPF5 = 0;
            rt_thread_mdelay(1000);
            dPF5 = 1;
            
            rt_thread_mdelay(1000);
        } }, nullptr, 256, 4, 10);

    rt_thread_startup(t1);

    rt_thread_startup(t2);

    rt_thread_startup(t3);

    while (true)
    {
        socket.sendString("Hello,RT-Thread!\n");
        rt_thread_mdelay(20); 
    }
}