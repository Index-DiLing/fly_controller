//===================================================================================================
// 上板自测: W25Q128 + FlashManager(简易日志 / 持久化参数文件系统)
//
// 接线(与 main_kzb_flash.cpp 一致, 需要时自行改):
//   调试串口 USART1: TX=A9, RX=A10, 115200
//   Flash SPI2: SCK=PB10, MISO=PC2, MOSI=PC3, 片选 /CS=PD1
//
// 第一次上板时芯片上没有文件系统, 会先整片擦除(几十秒), 串口会打印 FLASH_SYSTEM_CREATED;
// 之后每次上电都会占用一个新的日志槽(擦除 128KB 约 0.3s), 并继续往本次上电的会话里写日志。
//
// 注意: 本文件在 EIDE 工程里默认被排除在编译之外, 要用时把 main_att_ekf.cpp 换成排除本文件即可。
//===================================================================================================
#include "stm32f4xx.h"
#include "dlx_gpio.hpp"
#include "dlx_gpio_profile.h"
#include "dlx_spi.hpp"
#include "dlx_usart.hpp"
#include "dlx_bytebuffer.hpp"
#include "W25Q128/dlx_flash_manager.hpp"
#include "stdio.h"
#include "stdarg.h"
using namespace dlx;

// Flash 片选(软件 NSS): /CS -> PD1
#define FLASH_CS GPIOProfile::D1
// W25Q128 用 Mode0(CL=低, 1E), 软 NSS, /8 分频, MSB
constexpr SPIModeProfile FLASH_SPI_MODE = SPIModeProfile::FD_M_8B_CL_1E_NS_BR8_MSB;

static char g_logBuffer[128];
static USART *g_debug = nullptr;

static void logLine(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(g_logBuffer, sizeof(g_logBuffer), fmt, args);
    va_end(args);
    if (n < 0) {
        n = 0;
    }
    if (n > (int)sizeof(g_logBuffer) - 1) {
        n = (int)sizeof(g_logBuffer) - 1;
    }
    if (g_debug != nullptr) {
        g_debug->send(reinterpret_cast<const uint8_t *>(g_logBuffer), static_cast<uint16_t>(n));
    }
}

// ---------------- 读取时的回调 ----------------
static bool onSession(const FlashLogSessionInfo &info, void *ctx)
{
    uint32_t *count = static_cast<uint32_t *>(ctx);
    *count += 1u;
    logLine("  会话 %u: 起始槽 %u, 占用 %u 槽(%u 字节)%s\r\n",
            static_cast<unsigned>(info.sessionId),
            static_cast<unsigned>(info.firstSlot),
            static_cast<unsigned>(info.slotCount),
            static_cast<unsigned>(info.regionBytes),
            info.active ? "  <== 本次上电" : "");
    return true;
}

static bool onEntry(const LogEntry &e, void *ctx)
{
    uint32_t *count = static_cast<uint32_t *>(ctx);
    *count += 1u;
    if (*count <= 5u) { // 只打印前几条, 免得太长
        logLine("    #%u tick=%ums thr=%.2f flags=%u\r\n",
                static_cast<unsigned>(e.seq),
                static_cast<unsigned>(e.tickMs),
                static_cast<double>(e.throttle),
                static_cast<unsigned>(e.flags));
    }
    return true;
}

int main()
{
    DLX_NVIC_AutoConfig();

    // ---- 调试串口 ----
    uint8_t rxBuffer[256];
    ByteBuffer rxBuf(rxBuffer, 256);
    RingByteBuffer buf(rxBuf);
    auto usart1 = USART::USART1_TA9_RAA();
    usart1.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, 115200, buf);
    g_debug = &usart1;

    logLine("\r\n=== W25Q128 + FlashManager 自测 ===\r\n");

    // ---- Flash ----
    SPI spi2 = SPI::SPI2_SB10_MIC2_MOC3(FLASH_CS);
    spi2.init(FLASH_SPI_MODE);
    W25Q128 flash(spi2);
    if (!flash.init()) {
        logLine("[Flash] 识别失败(JEDEC=%06X)\r\n", static_cast<unsigned>(flash.readJEDEC()));
        while (true) {
            delay_ms(1000);
        }
    }
    logLine("[Flash] JEDEC=%06X OK\r\n", static_cast<unsigned>(flash.readJEDEC()));

    // ---- 状态 LED + "正在干活"回调: 每擦完一个 64KB 块翻一次(整片擦除约 40s, 期间能看出在动) ----
    GPIO led(GPIOProfile::F3);
    led.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    bool ledOn = false;
    uint32_t busySteps = 0;
    auto onBusy = [&]() {
        ++busySteps;
        ledOn = !ledOn;
        led = ledOn ? 1 : 0;
    };

    // ---- 文件系统 ----
    FlashManager fs(flash);
    // 需要更大/更小的单次日志时在这里改(单位: 槽, 每槽 128KB), 下一次 init 生效, 默认 8
    // fs.setSessionSlotLimit(16);
    ExceptionCode err = ExceptionCode::FLASH_OK;
    busySteps = 0;
    FunctionResult result = fs.init(FlashFullPolicy::Halt, onBusy, err);
    logLine("[FS] init=%d err=0x%04X 会话号=%u 预留槽=%u 单次上限=%u 条 (0x0101=本次新建了文件系统)\r\n",
            static_cast<int>(result), static_cast<unsigned>(err),
            static_cast<unsigned>(fs.currentSessionId()),
            static_cast<unsigned>(fs.sessionReservedSlots()),
            static_cast<unsigned>(fs.sessionEntryLimit()));
    logLine("[FS] 擦除进度回调 %u 次\r\n", static_cast<unsigned>(busySteps));
    if (result != FunctionResult::SUCCESS) {
        logLine("[FS] 初始化失败, 停止\r\n");
        while (true) {
            delay_ms(1000);
        }
    }

    // ---- 1. 列出所有日志会话 ----
    logLine("[1] 现有日志会话:\r\n");
    uint32_t sessionCount = 0;
    fs.forEachSession(onSession, &sessionCount, err);
    logLine("    共 %u 个会话\r\n", static_cast<unsigned>(sessionCount));

    // ---- 2. 本次上电写几条日志 ----
    logLine("[2] 写入 5 条日志...\r\n");
    for (uint32_t i = 1; i <= 5; ++i) {
        LogEntry entry = {};
        // 只需要填业务字段, 前面几个字段由 FlashManager 覆盖
        entry.tickMs = i * 100u;
        entry.rollRad = 0.01f * static_cast<float>(i);
        entry.pitchRad = -0.01f * static_cast<float>(i);
        entry.yawRad = 0.0f;
        entry.gyroX = 0.0f;
        entry.gyroY = 0.0f;
        entry.gyroZ = 0.0f;
        entry.throttle = 0.3f + 0.05f * static_cast<float>(i);
        for (uint32_t m = 0; m < 4; ++m) {
            entry.motor[m] = static_cast<uint16_t>(1000u + i * 10u + m);
        }
        entry.flags = static_cast<uint16_t>(i);

        if (fs.appendLog(entry, err) != FunctionResult::SUCCESS) {
            logLine("    第 %u 条写入失败, err=0x%04X\r\n", static_cast<unsigned>(i), static_cast<unsigned>(err));
            break;
        }
        delay_ms(10);
    }
    logLine("    本次会话已写 %u 条\r\n", static_cast<unsigned>(fs.currentEntryCount()));

    // ---- 3. 回读本次会话 ----
    logLine("[3] 回读本次会话(会话号 %u):\r\n", static_cast<unsigned>(fs.currentSessionId()));
    uint32_t entryCount = 0;
    if (fs.countEntries(fs.currentSessionId(), entryCount, err) == FunctionResult::SUCCESS) {
        logLine("    共 %u 条\r\n", static_cast<unsigned>(entryCount));
    }
    uint32_t printed = 0;
    fs.forEachEntry(fs.currentSessionId(), onEntry, &printed, err);

    // ---- 4. 持久化参数 ----
    logLine("[4] 持久化参数:\r\n");
    FlashParamData params;
    if (fs.loadParams(params, err) == FunctionResult::SUCCESS) {
        logLine("    读回: version=%u value[0]=%.3f value[1]=%.3f\r\n",
                static_cast<unsigned>(params.version),
                static_cast<double>(params.value[0]), static_cast<double>(params.value[1]));
    } else {
        logLine("    还没有保存过参数(err=0x%04X), 按默认值使用\r\n", static_cast<unsigned>(err));
        params.version = 1;
        for (uint32_t i = 0; i < FLASH_PARAM_VALUE_COUNT; ++i) {
            params.value[i] = static_cast<float>(i);
        }
    }

    // 改一个值再存一次, 下次上电应该能读到新值
    params.value[0] += 1.0f;
    if (fs.saveParams(params, err) == FunctionResult::SUCCESS) {
        FlashParamData check;
        fs.loadParams(check, err);
        logLine("    已保存: value[0]=%.3f (写回读校验)\r\n", static_cast<double>(check.value[0]));
    } else {
        logLine("    保存失败, err=0x%04X\r\n", static_cast<unsigned>(err));
    }

    // ---- 5. 容量提示 ----
    logLine("[5] 本次会话: 已用 %u 槽 / 预留 %u 槽, 上限 %u 条; 退出(复位后本次日志保留为历史会话)\r\n",
            static_cast<unsigned>(fs.sessionUsedSlots()),
            static_cast<unsigned>(fs.sessionReservedSlots()),
            static_cast<unsigned>(fs.sessionEntryLimit()));
    logLine("    上电时只擦上次真正用掉的那几个槽(没用到的预留槽上次已经擦好, 不再重复擦)\r\n");

    // ---- 6. 清理日志(默认不执行, 需要时打开) ----
    // 环形队列语义: 丢的永远是最老的那几次上电, 写指针不动;
    // 腾出来的槽直接并进"当前这次会话"的预留区, 所以清完还能接着往下写、上限变大。
    //   fs.dropSessions(1, onBusy, err);   // 丢掉最老 1 个会话
    //   fs.clearHistory(onBusy, err);      // 清空全部历史(保留当前会话), 常用
    // 想连当前会话的数据一起清掉(恢复出厂)用 fs.format(err), 但参数也会一起没。
    logLine("[6] 清理接口: dropSessions(n,...) 丢最老 n 个 / clearHistory(...) 清历史(本测试未调用)\r\n");

    while (true) {
        delay_ms(1000);
    }
}
