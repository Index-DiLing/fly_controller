
#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_gpio_profile.h"
#include "dlx_usart.hpp"
#include "dlx_bytebuffer.hpp"
#include "stdio.h"
#include "dlx.hpp"
#include "rtthread.h"
#include "dlx_delay.hpp"
#include "dsp/matrix_functions.h"
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

    auto usart1 = USART::USART1_TA9_RAA(); 

    usart1.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, 115200, buf);

    ByteBuffer dmaBuffer((uint8_t*) g_fixed_str_buffer, 128);


    arm_matrix_instance_f32 mat;

    mat.numCols = 16;
    mat.numRows = 16;


    
    arm_matrix_instance_f32 dst;




    fixed_sprintf("Hello,DLX\n");

    auto serialDMA = usart1.setDMASend(dmaBuffer);

    while (true) {
        serialDMA.start();
        serialDMA.wait();
        serialDMA.reset();
    }
}
