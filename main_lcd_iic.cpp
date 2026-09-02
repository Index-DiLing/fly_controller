// ============================================================================
// main_lcd_iic.cpp
// 测试流程: 服务器 -> ESP8266(AT 透传) -> USART3 -> DLX 协议解析
//           -> DrawFrameSlice(1/20 帧, 128x160 RGB565) -> IIC ST7735S
//
// 说明:
//  - 只新增本文件并切换 EIDE 编译入口, 不动其它业务代码;
//  - ST7735 为 IIC 版本: 每条 I2C 事务先发控制字节(0x00=命令, 0x40=数据),
//    旧 DL_LIB/old/dl_lcd_st7735s.hpp 是 SPI 版, 这里仅替换了传输层;
//  - 一片 DrawFrameSlice = 2048 字节 = 1024 像素 = 8 行, 20 片 = 1 帧;
//  - 协议里像素是小端 RGB565, ST7735 需要高字节在前, 发送前逐像素交换;
//  - 进入透传后, MCU 先给服务器发任意内容, 触发服务器开始推流.
// ============================================================================

#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_gpio_profile.h"
#include "dlx_usart.hpp"
#include "dlx_bytebuffer.hpp"
#include "dlx_iic.hpp"
#include "dlx_delay.hpp"
#include "dlx.hpp"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
using namespace dlx;

// ------------------------- 需要按实际情况修改 -------------------------
#define ESP_WIFI_SSID       "YOUR_WIFI_SSID"      // TODO: WiFi 名称
#define ESP_WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"  // TODO: WiFi 密码
#define ESP_SERVER_IP       "192.168.1.100"       // TODO: 服务器 IP
#define ESP_SERVER_PORT     "8080"                // TODO: 服务器端口
#define ESP_TRIGGER         "start\r\n"           // 任意内容, 触发服务器开始推流

// ------------------------- LCD / IIC -------------------------
#define LCD_I2C_ADDR        0x3C        // 常见 I2C ST7735 模块地址(7bit)
#define LCD_RST_PIN         GPIOProfile::A1  // 复位引脚(与旧板一致, 无 RST 引出则忽略)
#define LCD_WIDTH           128
#define LCD_HEIGHT          160
#define SLICES_PER_FRAME    20          // 160 行 / 每片 8 行
#define SLICE_PIXELS        (LCD_WIDTH * 8)   // 1024 像素
#define SLICE_BYTES         (SLICE_PIXELS * 2) // 2048 字节
// I2C 版 ST7735 模块内部桥接芯片的 FIFO 一般较小, 像素数据分小块发送更稳
#define LCD_DATA_CHUNK      32          // 每块像素数(单次 I2C 事务的数据字节 = 块*2)

// ------------------------- 引脚 -------------------------
// 调试串口: USART1(TX=A9, RX=A10)
// ESP8266:  USART3(TX=PD8, RX=PD9), EN=PC0(拉高)
// LCD:      I2C1(SCL=PB6, SDA=PB7), RST=PA1

static USART *g_dbg = nullptr;
static USART *g_espUsart = nullptr;
static RingByteBuffer *g_espRing = nullptr;
static IICDevice *g_lcd = nullptr;
static char g_logBuf[128];

static void logLine(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(g_logBuf, sizeof(g_logBuf), fmt, args);
    va_end(args);
    if (n < 0) n = 0;
    if (n > (int)sizeof(g_logBuf) - 1) n = (int)sizeof(g_logBuf) - 1;
    g_dbg->send((const uint8_t *)g_logBuf, (uint16_t)n);
}

// ============================================================================
// ST7735S over I2C(控制字节协议)
// ============================================================================

// 发一条命令: [0x00][cmd]
static void lcdCmd(uint8_t cmd)
{
    uint8_t tx[2] = {0x00, cmd};
    ByteBuffer bb(tx, 2);
    bb.cur = tx + 2;
    g_lcd->write(bb);
}

// 发数据: [0x40][payload...], 供初始化/窗口设置等少量字节使用
static void lcdData(const uint8_t *payload, uint16_t len)
{
    static uint8_t tx[1 + 16];
    tx[0] = 0x40;
    memcpy(tx + 1, payload, len);
    ByteBuffer bb(tx, 1 + len);
    bb.cur = tx + 1 + len;
    g_lcd->write(bb);
}

static void lcdSetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t d[4];
    d[0] = (uint8_t)(x0 >> 8); d[1] = (uint8_t)(x0 & 0xFF);
    d[2] = (uint8_t)(x1 >> 8); d[3] = (uint8_t)(x1 & 0xFF);
    lcdCmd(0x2A); // CASET
    lcdData(d, 4);
    d[0] = (uint8_t)(y0 >> 8); d[1] = (uint8_t)(y0 & 0xFF);
    d[2] = (uint8_t)(y1 >> 8); d[3] = (uint8_t)(y1 & 0xFF);
    lcdCmd(0x2B); // RASET
    lcdData(d, 4);
}

// 初始化序列与旧 SPI 驱动(DL_LIB/old/dl_lcd_st7735s.hpp)一致
static void lcdInit()
{
    // 硬件复位(模块无 RST 引出时此段无效果, 由后续软件复位兜底)
    GPIO rst(LCD_RST_PIN);
    rst.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    rst = 1;
    delay_ms(10);
    rst = 0;
    delay_ms(10);
    rst = 1;
    delay_ms(120);

    lcdCmd(0x01); // 软件复位
    delay_ms(120);
    lcdCmd(0x11); // Sleep out
    delay_ms(120);

    // 帧率
    uint8_t b1[3] = {0x05, 0x3C, 0x3C};
    lcdCmd(0xB1); lcdData(b1, 3);
    uint8_t b2[2] = {0x05, 0x3C};
    lcdCmd(0xB2); lcdData(b2, 2);
    uint8_t b3[6] = {0x05, 0x3C, 0x3C, 0x05, 0x3C, 0x3C};
    lcdCmd(0xB3); lcdData(b3, 6);
    uint8_t b4[1] = {0x03};
    lcdCmd(0xB4); lcdData(b4, 1); // Dot inversion

    // 电源
    uint8_t c0[3] = {0x28, 0x08, 0x04};
    lcdCmd(0xC0); lcdData(c0, 3);
    uint8_t c1[1] = {0xC0};
    lcdCmd(0xC1); lcdData(c1, 1);
    uint8_t c2[2] = {0x0D, 0x00};
    lcdCmd(0xC2); lcdData(c2, 2);
    uint8_t c3[2] = {0x8D, 0x2A};
    lcdCmd(0xC3); lcdData(c3, 2);
    uint8_t c4[2] = {0x8D, 0xEE};
    lcdCmd(0xC4); lcdData(c4, 2);
    uint8_t c5[1] = {0x1A};
    lcdCmd(0xC5); lcdData(c5, 1); // VCOM
    uint8_t madctl[1] = {0xC0};   // MX|MY, RGB(与旧驱动一致)
    lcdCmd(0x36); lcdData(madctl, 1);

    // Gamma
    uint8_t e0[16] = {0x04, 0x22, 0x07, 0x0A, 0x2E, 0x30, 0x25, 0x2A,
                      0x28, 0x26, 0x2E, 0x3A, 0x00, 0x01, 0x03, 0x13};
    lcdCmd(0xE0); lcdData(e0, 16);
    uint8_t e1[16] = {0x04, 0x16, 0x06, 0x0D, 0x2D, 0x26, 0x23, 0x27,
                      0x27, 0x25, 0x2D, 0x3B, 0x00, 0x01, 0x04, 0x13};
    lcdCmd(0xE1); lcdData(e1, 16);

    // 65K 模式 + 开显示
    uint8_t p65k[1] = {0x05};
    lcdCmd(0x3A); lcdData(p65k, 1);
    lcdCmd(0x29);
}

// 全屏刷一种颜色, 用来验证 I2C 通路
static void lcdFill(uint16_t color)
{
    lcdSetWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    lcdCmd(0x2C); // RAMWR

    static uint8_t tx[1 + LCD_DATA_CHUNK * 2];
    tx[0] = 0x40;
    for (uint16_t i = 0; i < LCD_DATA_CHUNK; ++i) {
        tx[1 + i * 2]     = (uint8_t)(color >> 8);
        tx[1 + i * 2 + 1] = (uint8_t)(color & 0xFF);
    }
    uint32_t pixels = LCD_WIDTH * LCD_HEIGHT;
    while (pixels) {
        uint16_t n = (pixels > LCD_DATA_CHUNK) ? LCD_DATA_CHUNK : (uint16_t)pixels;
        ByteBuffer bb(tx, 1 + n * 2);
        bb.cur = tx + 1 + n * 2;
        g_lcd->write(bb);
        pixels -= n;
    }
}

// 把一片(8 行)DrawFrameSlice 写到对应行窗口
static void drawSliceToLcd(DrawFrameSlice *slice)
{
    static uint32_t sliceCount = 0;
    uint16_t sliceIdx = (uint16_t)(sliceCount % SLICES_PER_FRAME);
    uint16_t y0 = sliceIdx * 8;

    lcdSetWindow(0, y0, LCD_WIDTH - 1, y0 + 7);
    lcdCmd(0x2C);

    static uint8_t tx[1 + LCD_DATA_CHUNK * 2];
    tx[0] = 0x40;
    uint16_t i = 0;
    while (i < SLICE_PIXELS) {
        uint16_t n = SLICE_PIXELS - i;
        if (n > LCD_DATA_CHUNK) n = LCD_DATA_CHUNK;
        for (uint16_t j = 0; j < n; ++j) {
            uint16_t p = slice->data(i + j); // 协议内为小端 RGB565
            tx[1 + j * 2]     = (uint8_t)(p >> 8); // ST7735 需要高字节在前
            tx[1 + j * 2 + 1] = (uint8_t)(p & 0xFF);
        }
        ByteBuffer bb(tx, 1 + n * 2);
        bb.cur = tx + 1 + n * 2;
        g_lcd->write(bb);
        i += n;
    }
    ++sliceCount;
}

// ============================================================================
// ESP8266 AT 辅助
// ============================================================================

static void espSend(const char *s)
{
    g_espUsart->send((const uint8_t *)s, (uint16_t)strlen(s));
}

// 轮询环形缓冲等待应答关键词, 期间丢弃无关字节; 超时返回 false
static bool espWaitToken(const char *token, uint32_t timeoutMs)
{
    uint16_t tokLen = (uint16_t)strlen(token);
    uint16_t matched = 0;
    for (uint32_t t = 0; t < timeoutMs; ++t) {
        uint8_t c;
        if (g_espRing->read(c)) {
            if (c == (uint8_t)token[matched]) {
                if (++matched == tokLen) return true;
            } else {
                matched = (c == (uint8_t)token[0]) ? 1 : 0;
            }
        } else {
            delay_ms(1);
        }
    }
    return false;
}

static bool espCmd(const char *cmd, const char *token, uint32_t timeoutMs)
{
    espSend(cmd);
    espSend("\r\n");
    return espWaitToken(token, timeoutMs);
}

// ============================================================================
// DLX 协议回调: 收到一片 DrawFrameSlice
// ============================================================================

static void onDrawFrameSlice(DrawFrameSlice *slice, void *)
{
    drawSliceToLcd(slice);
}

// ============================================================================
// main
// ============================================================================

int main()
{
    DLX_NVIC_AutoConfig();

    // 状态 LED
    GPIO pf3(GPIOProfile::F3);
    pf3.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    pf3 = 0;

    // 调试串口: USART1(TX=A9, RX=A10) @115200
    uint8_t dbgRxBuf[64];
    ByteBuffer dbgRx(dbgRxBuf, 64);
    RingByteBuffer dbgRing(dbgRx);
    auto usart1 = USART::USART1_TA9_RAA();
    usart1.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, 115200, dbgRing);
    g_dbg = &usart1;

    logLine("=== LCD(IIC) + ESP8266 DrawFrameSlice 测试 ===\r\n");

    // ---------------- LCD: I2C1(SCL=PB6, SDA=PB7) ----------------
    auto iic = IICBus::IIC1_SB6_DB7();
    iic.init(IICBusModeProfile::ACK_E_DC16_9_ADDR7, 400000, 0x51);
    IICDevice lcd(iic, LCD_I2C_ADDR);
    g_lcd = &lcd;

    lcdInit();
    lcdFill(0x001F); // 先刷一屏蓝色, 验证 I2C 通路
    logLine("[LCD] init + fill done\r\n");

    // ---------------- ESP8266: USART3(TX=PD8, RX=PD9), EN=PC0 ----------------
    GPIO pc0(GPIOProfile::C0);
    pc0.init(GPIOModeProfile::OUT_PP_UP_50MHz);
    pc0 = 1; // 拉高 ESP EN
    delay_ms(1000); // 等 ESP 上电

    uint8_t espRxBuf[8192];
    ByteBuffer espRx(espRxBuf, 8192);
    RingByteBuffer espRing(espRx);
    espRing.setOverwrite(true); // 刷屏较慢时丢旧数据, 不卡死

    auto usart3 = USART::USART3_TD8_RD9();
    usart3.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, 115200, espRing);
    g_espUsart = &usart3;
    g_espRing = &espRing;

    // AT 探测
    bool ok = false;
    for (int i = 0; i < 3 && !ok; ++i) {
        ok = espCmd("AT", "OK", 3000);
    }
    logLine("[ESP] AT = %s\r\n", ok ? "OK" : "FAIL");

    // WiFi 连接
    ok = espCmd("AT+CWMODE=1", "OK", 3000);
    logLine("[ESP] CWMODE = %s\r\n", ok ? "OK" : "FAIL");

    char atBuf[96];
    snprintf(atBuf, sizeof(atBuf), "AT+CWJAP=\"%s\",\"%s\"",
             ESP_WIFI_SSID, ESP_WIFI_PASSWORD);
    ok = espCmd(atBuf, "OK", 15000);
    logLine("[ESP] CWJAP = %s\r\n", ok ? "OK" : "FAIL");

    // TCP 透传
    ok = espCmd("AT+CIPMODE=1", "OK", 3000);
    logLine("[ESP] CIPMODE = %s\r\n", ok ? "OK" : "FAIL");

    snprintf(atBuf, sizeof(atBuf), "AT+CIPSTART=\"TCP\",\"%s\",%s",
             ESP_SERVER_IP, ESP_SERVER_PORT);
    ok = espCmd(atBuf, "OK", 10000);
    logLine("[ESP] CIPSTART = %s\r\n", ok ? "OK" : "FAIL");

    ok = espCmd("AT+CIPSEND", ">", 5000);
    logLine("[ESP] CIPSEND = %s\r\n", ok ? "OK" : "FAIL");

    // 进入透传后, 清掉残留的 AT 回复, 避免污染协议解析
    espRing.reset();

    // 发任意内容触发服务器推流
    espSend(ESP_TRIGGER);
    logLine("[ESP] trigger sent, waiting for DrawFrameSlice...\r\n");

    // ---------------- DLX 协议解析 ----------------
    uint8_t txBuf[64];
    ByteBuffer dmaBuf(txBuf, 64); // 本测试只收不发, 发送缓冲占位
    uint8_t fr[SLICE_BYTES];
    ByteBuffer frame(fr, SLICE_BYTES);
    DLX_ProtocolBuffer protocol(dmaBuf, &espRing, &frame);
    protocol.setDrawFrameSliceCallbackFunction(onDrawFrameSlice, nullptr);

    bool led = false;
    while (true) {
        if (espRing.available() >= 2) {
            if (protocol.check()) {
                led = !led;
                pf3 = led; // 每收到一片翻一次
                continue;
            }
            // check() 失败: 要么帧没收全, 要么头部不同步
            uint16_t head = espRing.peek_be<uint16_t>();
            uint16_t type = (head >> 5) & 0x3ff;
            bool full = espRing.available() >= SLICE_BYTES + 2;
            if (type != 8 || full) {
                uint8_t c;
                espRing.read(c); // 逐字节丢弃, 直到重新同步
            }
        }
    }
}
