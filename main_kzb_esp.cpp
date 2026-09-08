
#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_gpio_profile.h"
#include "dlx_usart.hpp"
#include "dlx_bytebuffer.hpp"
#include "stdio.h"
#include "dlx.hpp"
#include "rtthread.h"
#include "BMI088/dlx_bmi088.hpp"
#include "DL_AHRS/MadgwickAHRS.hpp"
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
#define str(s) (uint8_t *)(s), strlen(s)

int main()
{

    DLX_NVIC_AutoConfig();

    GPIO pf3(GPIOProfile::F3);

    pf3.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);

    pf3 = 0;

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

    // bool atom = true;

    // protocol.setUnBlockCallbackFunction(+[](UnBlock *, void *at) {
    //     bool* p = static_cast<bool*>(at);
    //     *p = false; }, &atom);
    // while (atom) {
    //     if (protocol.check()) {
    //         pf3 = 1;
    //     }
    // }

    fixed_sprintf("Hello,DLX\n");

    protocol.SimpleLogW(gfsb);

    auto pb8 = GPIO(GPIOProfile::B9);
    pb8.init(GPIOModeProfile::OUT_PP_NOPULL_25MHz);

    uint8_t b1[256];
    ByteBuffer eb(b1, 256);

    RingByteBuffer espBuffer(eb);

    auto usart3 = USART::USART3_TD8_RD9();

    auto pc0 = GPIO(GPIOProfile::C0);
    pc0.init(GPIOModeProfile::OUT_PP_UP_50MHz);

    pc0 = 1;

    usart3.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, 115200, espBuffer);

    uint8_t atb[4];

    delay_ms(1000);
    uint8_t atbb[4] = {'A', 'T', '\r', '\n'};

    ByteBuffer AT(atb, 4);

    AT.write(atbb, 4);

    usart3.send(AT);

    while (espBuffer.available() == 0) {
        // AT.write(atbb, 4);

        // usart3.send(AT);
        // pb8 = 1;
        // delay_ms(100);
        // pb8 = 0;
        delay_ms(1000);

        AT.write(atbb, 4);

        usart3.send(AT);
    }
    pf3 = 1;

    auto serialDMA = usart1.setDMASend(dmaBuffer);

    while (true) {
        serialDMA.start();
        serialDMA.wait();
        serialDMA.reset();
    }
}
