#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_gpio_profile.h"
#include "dlx_usart.hpp"
#include "dlx_bytebuffer.hpp"
#include "stdio.h"
#include "dlx.hpp"
#include "rtthread.h"
#include "dlx_timer.hpp"
#include "dlx_exit.hpp"
#include "dlx_delay.hpp"
#include "dlx_iic.hpp"
#include "BME280/dlx_bme280.hpp"

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

    IICBus bus = IICBus::IIC1_SB6_DB7();

    // 先用 100kHz 更稳妥(400kHz 在走线/上拉不佳时容易读坏), 确认能读再提速到 400000
    bus.init(IICBusModeProfile::ACK_E_DC16_9_ADDR7, 100000, 13);

    // BME280 地址: SDO 接低=0x76, 接高=0x77。若模块 SDO 拉高把这里改成 0x77。
    uint16_t bmeAddr = 0x76;
    BME280 bme(bus, bmeAddr);

    bool bmeOk = bme.init();

    // 先把串口 DMA 建好, 这样即使传感器没找到也能输出诊断
    auto serialDMA = usart1.setDMASend(dmaBuffer);

    // 直读 0x76 的 ChipId(0xD0), 确认上面到底是什么芯片
    uint8_t chipId = 0;
    {
        uint8_t reg = 0xD0;
        ByteBuffer w(&reg, 1);
        ByteBuffer r(&chipId, 1);
        IICDevice rawDev(bus, bmeAddr);
        rawDev.read(r, w, 1);
    }
    fixed_sprintf("chipId@0x%02x = 0x%02x\n", bmeAddr, chipId);
    protocol.SimpleLogW(gfsb);

    if (!bmeOk) {
        fixed_sprintf("BME280 init FAILED! chipId=0x%02x (0x60=BME280, 0x58=BMP280)\n", chipId);
        protocol.SimpleLogW(gfsb);
        while (true) {
            dmaBuffer.reset();
            protocol.SimpleLogW(gfsb);
            serialDMA.reset(dmaBuffer.used());
            serialDMA.start();
            serialDMA.wait();
            delay_ms(500);
        }
    }

    fixed_sprintf("Hello,DLX  BME280 ok(addr=0x%02x)\n", bmeAddr);
    protocol.SimpleLogW(gfsb);

    while (true) {

        EnviromentRaw raw = bme.getRaw();
        auto p = bme.getPressure(raw);
        auto h = bme.getHumidity(raw);
        auto t = bme.getTemperature(raw);

        dmaBuffer.reset();

        // 打印原始 ADC 值, 便于判断是"通信坏"还是"换算坏"
        fixed_sprintf("raw P=0x%06lx T=0x%06lx H=0x%04x  P:%f,H:%f,T:%f\n",
                      (unsigned long)raw.pressure,
                      (unsigned long)raw.temperature,
                      (unsigned)raw.humidity,
                      p, h, t);
        protocol.SimpleLogW(gfsb);

        serialDMA.reset(dmaBuffer.used());
        serialDMA.start();
        serialDMA.wait();
        delay_ms(100);
    }
}
