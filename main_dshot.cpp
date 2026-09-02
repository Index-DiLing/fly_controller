
#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_gpio_profile.h"
#include "dlx_usart.hpp"
#include "dlx_bytebuffer.hpp"
#include "stdio.h"
#include "dlx.hpp"
#include "rtthread.h"
#include "dlx_timer.hpp"
#include "dlx_delay.hpp"
using namespace dlx;

char g_fixed_str_buffer[128];
#define gfsb (int8_t *)g_fixed_str_buffer, (uint16_t)strlen(g_fixed_str_buffer)
int fixed_sprintf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int ret = vsnprintf(g_fixed_str_buffer, 128, format, args);
    va_end(args);
    return ret;
}

int main()
{

    DLX_NVIC_AutoConfig();

    GPIO pf3(GPIOProfile::F3);

    pf3.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);

    pf3 = 0;
    delay_ms(3000);

    uint8_t rxBuffer[256];

    ByteBuffer rxBuf(rxBuffer, 256);

    RingByteBuffer buf(rxBuf);

    auto usart1 = USART::USART1_TA9_RAA(); // 自动配置 A9/A10(AA) 为 USART1 的 AF 引脚

    usart1.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, 115200, buf);

    uint8_t dmaB[128];

    ByteBuffer dmaBuffer(dmaB, 128);

    uint8_t fr[128];
    ByteBuffer frame(fr, 128);
    DLX_ProtocolBuffer protocol(dmaBuffer, &buf, &frame);

    bool atom = true;

    protocol.setUnBlockCallbackFunction(+[](UnBlock *, void *at) {
        bool* p = static_cast<bool*>(at);
        *p = false; }, &atom);
    while (atom) {
        if (protocol.check()) {
            pf3 = 1;
        }
    }
    uint16_t manual[4] = {150,150,200,200};


    protocol.setTargetThrottleValueCallbackFunction(+[](TargetThrottleValue* data,void* ctx){
        uint16_t* m = static_cast<uint16_t*>(ctx);
        m[0] = data->data(0);

        m[1] = data->data(1);
        
        m[2] = data->data(2);
        
        m[3] = data->data(3);
        auto i =  GPIO(GPIOProfile::F3);
        i = 0;
    },manual);

    pf3 = 1;

    AdvancedTimer timer = AdvancedTimer(AdvancedTimerProfile::TIM1_Up_DIV1);

    Dshot dshot = timer.setDshot(DshotProfile::Dshot300);

    dshot.start();
    delay_ms(3000);

    auto serialDMA = usart1.setDMASend(dmaBuffer);

    while (true)
    { 
        dshot.preloadThrottle(manual[0],manual[1],manual[2],manual[3]);

        dmaBuffer.reset();
        
        protocol.ThrottleValueW(manual);

        serialDMA.reset(protocol.buffer.used());    

        serialDMA.start();

        serialDMA.wait();


        delay_ms(100);

        protocol.check();

    }

    while (true) {
        serialDMA.start();
        serialDMA.wait();
        serialDMA.reset();
    }
}
