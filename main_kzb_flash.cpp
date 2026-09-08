#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_gpio_profile.h"
#include "dlx_spi.hpp"
#include "dlx_usart.hpp"
#include "dlx_bytebuffer.hpp"
#include "W25Q128/dlx_w25q128.hpp"
#include "rtthread.h"
#include "stdio.h"
#include "stdarg.h"
using namespace dlx;

char g_log_buffer[128];

// Flash 片选(软件 NSS): /CS -> PD1
#define FLASH_CS GPIOProfile::D1

// W25Q128 用 Mode0(CL=低, 1E), 软 NSS, /8 分频, MSB
constexpr SPIModeProfile FLASH_SPI_MODE = SPIModeProfile::FD_M_8B_CL_1E_NS_BR8_MSB;

int main()
{
    DLX_NVIC_AutoConfig();

    // 状态 LED(两者都通过才亮)
    GPIO pf3(GPIOProfile::F3);
    pf3.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    pf3 = 0;

    // 调试串口: USART1(TX=A9, RX=A10) @115200
    uint8_t rxBuffer[256];
    ByteBuffer rxBuf(rxBuffer, 256);
    RingByteBuffer buf(rxBuf);
    auto usart1 = USART::USART1_TA9_RAA();
    usart1.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, 115200, buf);

    // 打印辅助: 纯 ASCII 逐条发出
    auto logLine = [&](const char *fmt, ...) {
        va_list args;
        va_start(args, fmt);
        int n = vsnprintf(g_log_buffer, sizeof(g_log_buffer), fmt, args);
        va_end(args);
        if (n < 0) n = 0;
        if (n > (int)sizeof(g_log_buffer) - 1) n = (int)sizeof(g_log_buffer) - 1;
        usart1.send((const uint8_t *)g_log_buffer, (uint16_t)n);
    };

    logLine("=== 综合测试: Flash ID + ESP AT ===\r\n");

    bool flashOk = false;
    bool espOk = false;

    // ================= 1. Flash ID =================
    // SPI2: SCK=PB10, MISO=PC2, MOSI=PC3, 片选=/PD1
    SPI spi2 = SPI::SPI2_SB10_MIC2_MOC3(FLASH_CS);
    spi2.init(FLASH_SPI_MODE);

    W25Q128 flash(spi2);
    const uint32_t jedec = flash.readJEDEC();
    const uint16_t devId = flash.readDeviceID();
    flashOk = flash.init();

    logLine("[Flash] JEDEC=%06X DeviceID=%04X init=%d\r\n",
            (unsigned)jedec, (unsigned)devId, flashOk ? 1 : 0);
    if (flashOk) {
        logLine("[Flash] PASS\r\n");
    } else {
        logLine("[Flash] FAIL\r\n");
    }

    // ================= 2. ESP8266 AT =================
    // USART3(TX=PD8, RX=PD9) @115200, PC0 作为 ESP EN(拉高)
    uint8_t espBuf[256];
    ByteBuffer eb(espBuf, 256);
    RingByteBuffer espBuffer(eb);

    GPIO pc0(GPIOProfile::C0);
    pc0.init(GPIOModeProfile::OUT_PP_UP_50MHz);
    pc0 = 1; // 拉高 ESP EN

    auto usart3 = USART::USART3_TD8_RD9();
    usart3.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, 115200, espBuffer);

    delay_ms(1000); // 等 ESP 上电就绪

    uint8_t atb[4];
    uint8_t atbb[4] = {'A', 'T', '\r', '\n'};
    ByteBuffer AT(atb, 4);

    for (int attempt = 0; attempt < 3 && !espOk; ++attempt) {
        AT.write(atbb, 4);
        usart3.send(AT);
        logLine("[ESP] AT -> (try %d)\r\n", attempt + 1);

        uint32_t t = 0;
        while (espBuffer.available() == 0 && t < 3000) { // 单个 try 等待约 3s
            delay_ms(1);
            ++t;
        }

        if (espBuffer.available() != 0) {
            espOk = true;
        } else {
            delay_ms(500);
        }
    }

    if (espOk) {
        uint8_t tmp[128];
        uint16_t got = 0;
        uint8_t c;
        while (got < sizeof(tmp) && espBuffer.read(c)) {
            tmp[got++] = c;
        }

        char hex[256];
        int p = 0;
        for (uint16_t i = 0; i < got && p < (int)sizeof(hex) - 4; ++i) {
            p += snprintf(hex + p, sizeof(hex) - p, "%02X ", tmp[i]);
        }
        logLine("[ESP] reply %uB: %s\r\n[ESP] PASS\r\n", (unsigned)got, hex);
    } else {
        logLine("[ESP] FAIL (无应答)\r\n");
    }

    if (flashOk && espOk) {
        logLine("===== 综合测试 PASS =====\r\n");
        pf3 = 1;
    } else {
        logLine("===== 综合测试 FAIL =====\r\n");
        pf3 = 0;
    }

    while (true) {
        delay_ms(1000);
    }
}
