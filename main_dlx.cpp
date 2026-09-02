
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

    bool atom = true;

    protocol.setUnBlockCallbackFunction(+[](UnBlock *, void *at) {
        bool* p = static_cast<bool*>(at);
        *p = false; }, &atom);
    while (atom) {
        if (protocol.check()) {
            pf3 = 1;
        }
    }

    auto spi = SPI::SPI1_SA5_MIA6_MOA7(GPIOProfile::A3);

    spi.init(SPIModeProfile::FD_M_8B_CH_2E_NS_BR256_MSB);

    auto bmi = BMI088(spi, GPIOProfile::BC, GPIOProfile::BD);

    uint8_t self = bmi.init();

    auto mutex = rt_sem_create("sensor", 0, RT_IPC_FLAG_PRIO);

    auto timer = rt_timer_create("imut", +[](void *mut) { rt_sem_release(static_cast<rt_sem_t>(mut)); }, mutex, 50, RT_TIMER_FLAG_HARD_TIMER | RT_TIMER_FLAG_PERIODIC);

    uint8_t acbb[256];
    uint8_t gybb[256];
    ByteBuffer acb(acbb, 256);
    ByteBuffer gyb(gybb, 256);

    auto ac = Queue<AccelerometerRaw>(acb);
    auto gy = Queue<GyroscopeRaw>(gyb);

    fixed_sprintf("Hello,DLX\n");

    protocol.SimpleLogW(gfsb);

    auto serialDMA = usart1.setDMASend(dmaBuffer);

    MadgwickAHRS ahrs(200,0.078);

    rt_timer_start(timer);
    while (true) {
        rt_sem_take(mutex, RT_WAITING_FOREVER);

        bmi.fifoRead(ac, gy);
        dmaBuffer.reset();
        auto l = fixed_sprintf("size:%u,%u", ac.size(), gy.size());
        
        while (ac.size() > 0 && gy.size()>0)
        {
            AccelerometerRaw acr;
            ac.pop(acr);

            // protocol.AccelRawW(acr.data);

            GyroscopeRaw gyr;
            gy.pop(gyr);

            // protocol.GyroRawW(gyr.data);
            
            ahrs.MadgwickAHRSupdateIMU(StandardIMUGRads{
                .accel = BMI088::getAccelerationG(acr),
                .gyro = BMI088::getAngularVelocityRads(gyr)
            });
        }
        ac.clear();
        gy.clear();
        
        protocol.SimpleLogW(gfsb);
        protocol.QuatW(ahrs.getQuaternion().data);

        serialDMA.reset(protocol.buffer.used());

        serialDMA.start();

        serialDMA.wait();
    }
    while (true) {
        serialDMA.start();
        serialDMA.wait();
        serialDMA.reset();
    }
}
