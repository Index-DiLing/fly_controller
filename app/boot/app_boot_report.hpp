#pragma once
#include "flight_config.hpp"
#include "app/runtime/release_header.h"
#include "dlx_spi.hpp"
#include "W25Q128/dlx_w25q128.hpp"
// 启动诊断输出：时钟、总线、存储、电机及线程资源。

namespace dlx
{
/** APB1(SPI2/SPI3、USART2..)总线时钟 */
inline uint32_t apb1ClockHz()
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    return clocks.PCLK1_Frequency;
}

/** APB2(SPI1、USART1、TIM1)总线时钟 */
inline uint32_t apb2ClockHz()
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    return clocks.PCLK2_Frequency;
}

/**
 * @brief 上电复位原因(RCC_CSR 的复位标志)
 *
 * 读完立刻 RCC_ClearFlag(): 这些标志是"保持型"的, 不清掉的话下次上电会读到上一次的残留,
 * 变成"每次都报 IWDG 复位"的假线索。冷启动通常同时置 POR + PIN 两个位。
 */
inline void logResetCause()
{
    static const struct ResetFlag {
        uint8_t     flag;
        const char *name;
    } kFlags[] = {
        {RCC_FLAG_PORRST, "POR"},   {RCC_FLAG_BORRST, "BOR"},   {RCC_FLAG_PINRST, "PIN"},
        {RCC_FLAG_SFTRST, "SOFT"},  {RCC_FLAG_IWDGRST, "IWDG"}, {RCC_FLAG_WWDGRST, "WWDG"},
        {RCC_FLAG_LPWRRST, "LPWR"},
    };
    char text[64];
    int  n = 0;
    for (uint32_t i = 0; i < sizeof(kFlags) / sizeof(kFlags[0]); ++i) {
        if (RCC_GetFlagStatus(kFlags[i].flag) == RESET) {
            continue;
        }
        const int w = snprintf(&text[n], sizeof(text) - (size_t)n, (n == 0) ? "%s" : "+%s",
                               kFlags[i].name);
        if (w > 0) {
            n += w;
        }
        if ((size_t)n >= sizeof(text) - 8u) {
            break;
        }
    }
    RCC_ClearFlag();
    bootLogLine("[Sys ] 复位原因: %s\r\n", (n > 0) ? text : "无标志(冷启动或已被读走)");
}

/** 芯片时钟 / RT-Thread 堆 / 关键编译期配置: 排查"以为配了其实没配" */
inline void logSystemReport()
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    bootLogLine("[Sys ] 版本=%u 时钟: SYSCLK=%uHz HCLK=%uHz PCLK1=%uHz PCLK2=%uHz SystemCoreClock=%u\r\n",
            static_cast<unsigned>(cfg::kFirmwareVersion), static_cast<unsigned>(clocks.SYSCLK_Frequency),
            static_cast<unsigned>(clocks.HCLK_Frequency), static_cast<unsigned>(clocks.PCLK1_Frequency),
            static_cast<unsigned>(clocks.PCLK2_Frequency), static_cast<unsigned>(SystemCoreClock));
    logResetCause();

    rt_size_t heapTotal = 0, heapUsed = 0, heapMax = 0;
    rt_memory_info(&heapTotal, &heapUsed, &heapMax);
    bootLogLine("[Sys ] RT-Thread 堆: 总=%u 已用=%u 峰值=%u 空闲=%u 字节\r\n",
            static_cast<unsigned>(heapTotal), static_cast<unsigned>(heapUsed),
            static_cast<unsigned>(heapMax), static_cast<unsigned>(heapTotal - heapUsed));

    bootLogLine("[Cfg ] 控制环=%uHz(%ums) 串口=%u 波特 日志周期: 飞行=%ums 调试=%ums 负载上报=%ums\r\n",
            static_cast<unsigned>(cfg::time::ctrl::kRateHz), static_cast<unsigned>(cfg::time::ctrl::kTickMs),
            static_cast<unsigned>(cfg::hw::console::kBaud), static_cast<unsigned>(cfg::time::log::kFlightPeriodMs),
            static_cast<unsigned>(cfg::time::log::kDebugPeriodMs), static_cast<unsigned>(cfg::time::log::kPerfReportMs));
    bootLogLine("[Cfg ] 解锁延迟=%ums 链路超时=%ums IMU偏转=%d° 电机映射=%d,%d,%d,%d DShot=%ukHz 日志队列=%u条\r\n",
            static_cast<unsigned>(cfg::control::kMotorArmDelayMs), static_cast<unsigned>(cfg::link::failsafe::kLinkTimeoutMs),
            static_cast<int>(cfg::hw::imu::kYawDeg), static_cast<int>(cfg::hw::motor::kMap[0]),
            static_cast<int>(cfg::hw::motor::kMap[1]), static_cast<int>(cfg::hw::motor::kMap[2]),
            static_cast<int>(cfg::hw::motor::kMap[3]), static_cast<unsigned>(cfg::hw::motor::kDshot),
            static_cast<unsigned>(cfg::log::kQueueDepth));
    bootLogLine("[Cfg ] 气压计=%s EKF: 解算降频=%u 倾角限幅=%umrad 准静止门限=%um/s^2 零偏冻结=%s\r\n",
            cfg::feature::kBaro ? "开" : "关(编译期)", static_cast<unsigned>(cfg::ekf::kDecim),
            static_cast<unsigned>(cfg::ekf::kAccelTiltInnoLimitRad * 1000.0f),
            static_cast<unsigned>(cfg::ekf::kAccelHoldGateMss * 1000.0f),
            cfg::ekf::kFreezeGyroBiasWhenDynamic ? "开" : "关");
}

/**
 * @brief 一路 SPI 的"期望分频 vs 寄存器实际值"
 *
 * SPIModeProfile 是位拼装值, 其中 bit[5:3] 就是 SPI_CR1 的 BR[2:0](dlx_spi.hpp 用同一份位段
 * 交给 SPI_Init()), 所以从 profile 里解出来的分频是**期望值**, 再从 SPIx->CR1 回读一次对比 ——
 * 位段/掩码一旦对不上(改过 profile 或驱动), 这里立刻能看出来, 不用拿逻辑分析仪量时钟。
 * 实际波特率 = 总线时钟 / 2^(BR+1)。
 */
inline void logSpiStatus(const char *name, SPI_TypeDef *spi, SPIModeProfile mode, uint32_t pclkHz)
{
    const uint32_t brWant = (static_cast<uint16_t>(mode) & 0x0038u) >> 3;
    const uint32_t brNow  = (static_cast<uint32_t>(spi->CR1) & SPI_CR1_BR) >> 3;
    const uint32_t baud   = pclkHz >> (brNow + 1u);
    bootLogLine("[Dev ] %s: 使能=%s 主机=%s 波特=%uHz 分频 BR=%u(期望%u) 回读=%s\r\n", name,
            (spi->CR1 & SPI_CR1_SPE) ? "是" : "否", (spi->CR1 & SPI_CR1_MSTR) ? "是" : "否",
            static_cast<unsigned>(baud), static_cast<unsigned>(brNow), static_cast<unsigned>(brWant),
            (brNow == brWant) ? "一致" : "不一致!");
}

/**
 * @brief DShot 时基**期望值**自检(必须打在 AdvancedTimer::setDshot() 之前)
 *
 * 算式与 setDshot() 完全一致(每 bit 40 个计数周期, 见 dlx_timer.hpp): 它算不出预分频会直接
 * while(1) 死等 —— 现场表现为"上电就卡死、一行日志都没有"。所以先按同一个算式把期望值打出来,
 * 真卡住时最后一行日志就是定位点(寄存器回读在 setDshot() 之后, 见下面的 logDshotRegisterCheck)。
 */
inline void logDshotTiming()
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    const uint32_t ppre2  = (RCC->CFGR & RCC_CFGR_PPRE2) >> 13;
    const uint32_t timClk = (ppre2 != 0u) ? (clocks.PCLK2_Frequency * 2u) : clocks.PCLK2_Frequency;
    const uint32_t rate   = static_cast<uint32_t>(cfg::hw::motor::kDshot); // kHz
    const uint32_t cntClk = timClk / (rate * 1000u * 40u); // 目标计数时钟(0 = 配置非法)
    if (cntClk == 0u || cntClk > 65536u) {
        bootLogLine("[Dev ] DShot   : %ukHz 配不出时基(计数时钟=%u): setDshot() 会在此死等!\r\n",
                static_cast<unsigned>(rate), static_cast<unsigned>(cntClk));
        return;
    }
    const uint32_t psc    = cntClk - 1u;         // 期望 TIM_PSC
    const uint32_t realHz = timClk / (psc + 1u); // 实际计数时钟
    const uint32_t bitNs  = static_cast<uint32_t>(40ull * 1000000000ull / realHz); // 单 bit 周期 [ns]
    bootLogLine("[Dev ] DShot   : 期望 %ukHz: 定时器时钟=%uHz 预分频=%u 计数=%uHz 单bit=%uns(理论%uns)\r\n",
            static_cast<unsigned>(rate), static_cast<unsigned>(timClk), static_cast<unsigned>(psc),
            static_cast<unsigned>(realHz), static_cast<unsigned>(bitNs),
            static_cast<unsigned>(1000000u / rate));
    bootLogLine("[Dev ] DShot   : 每 bit=40 计数, 帧=16bit 数据+6bit 间隔, 理论帧率=%uHz\r\n",
            static_cast<unsigned>(realHz / 22u));
}

/** DShot 时基**寄存器回读**(必须打在 setDshot() 之后): 配置写歪/没写进去, 只有回读能证伪 */
inline void logDshotRegisterCheck()
{
    if (cfg::hw::motor::kTimer != AdvancedTimerProfile::TIM1_Up_DIV1) {
        bootLogLine("[Dev ] TIM1    : 电机定时器不是 TIM1_Up_DIV1, 跳过寄存器回读(只看上面的期望值)\r\n");
        return;
    }
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    const uint32_t ppre2  = (RCC->CFGR & RCC_CFGR_PPRE2) >> 13;
    const uint32_t timClk = (ppre2 != 0u) ? (clocks.PCLK2_Frequency * 2u) : clocks.PCLK2_Frequency;
    const uint32_t rate    = static_cast<uint32_t>(cfg::hw::motor::kDshot);
    const uint32_t cntClk  = timClk / (rate * 1000u * 40u);
    const uint32_t pscWant = (cntClk != 0u) ? (cntClk - 1u) : 0xFFFFu; // 0 = 配置非法(上一步已报)
    const uint32_t pscNow  = TIM1->PSC;
    const uint32_t arrNow  = TIM1->ARR;
    bootLogLine("[Dev ] TIM1    : 回读 PSC=%u(期望%u) ARR=%u(期望39) 计数=%s 主输出=%s -> %s\r\n",
            static_cast<unsigned>(pscNow), static_cast<unsigned>(pscWant), static_cast<unsigned>(arrNow),
            (TIM1->CR1 & TIM_CR1_CEN) ? "运行" : "停",
            (TIM1->BDTR & TIM_BDTR_MOE) ? "开(!)" : "关(解锁后才开, 安全)",
            (pscNow == pscWant && arrNow == 39u) ? "一致" : "不一致!");
}

/** 串口: BRR 反算实际波特率 + 使能/中断/DMA 位(F4: 波特率 = 外设时钟 / BRR) */
inline void logUsartStatus()
{
    const uint32_t brr  = static_cast<uint32_t>(USART1->BRR) & 0xFFFFu;
    const uint32_t baud = (brr != 0u) ? (apb2ClockHz() / brr) : 0u;
    bootLogLine("[Dev ] USART1  : 使能=%s 发送=%s 接收=%s 接收中断=%s DMA发送=%s DMA接收=%s\r\n",
            (USART1->CR1 & USART_CR1_UE) ? "是" : "否", (USART1->CR1 & USART_CR1_TE) ? "开" : "关",
            (USART1->CR1 & USART_CR1_RE) ? "开" : "关",
            (USART1->CR1 & USART_CR1_RXNEIE) ? "开" : "关", (USART1->CR3 & USART_CR3_DMAT) ? "开" : "关",
            (USART1->CR3 & USART_CR3_DMAR) ? "开" : "关");
    bootLogLine("[Dev ] USART1  : 波特=%u(请求%u) BRR=0x%04X 缓冲: 收=%uB 发=%uB 解帧=%uB 文本行=%uB\r\n",
            static_cast<unsigned>(baud), static_cast<unsigned>(cfg::hw::console::kBaud),
            static_cast<unsigned>(brr), static_cast<unsigned>(cfg::hw::buf::kRx),
            static_cast<unsigned>(cfg::hw::buf::kTx), static_cast<unsigned>(cfg::hw::buf::kFrame),
            static_cast<unsigned>(cfg::hw::buf::kTextLine));
}

/** 线程与堆: 线程创建失败就是堆被吃光(RT_HEAP_SIZE), 这时不允许解锁飞行 */
inline void logThreadReport(rt_thread_t linkTid, rt_thread_t logTid)
{
    rt_thread_t self = rt_thread_self();
    bootLogLine("[Dev ] 线程    : 主(控制) 优先级=%u 栈=%uB\r\n",
            static_cast<unsigned>(self ? self->current_priority : 0u),
            static_cast<unsigned>(self ? self->stack_size : 0u));
    // 链路被编译期关掉时**不能报"失败"** —— 那个线程压根没打算建。
    // 报"失败"会让人去查堆(查不出问题), 正是"主动关掉 ≠ 坏掉"这条规矩要防的事。
    const char *linkState;
#if CFG_FEATURE_LINK
    linkState = (linkTid != RT_NULL) ? "OK" : "失败";
#else
    linkState = "关闭(编译期)";
#endif
    bootLogLine("[Dev ] 线程    : link_tel=%s 栈=%uB | logger=%s 栈=%uB\r\n", linkState,
            static_cast<unsigned>(linkTid ? linkTid->stack_size : 0u), (logTid != RT_NULL) ? "OK" : "失败",
            static_cast<unsigned>(logTid ? logTid->stack_size : 0u));
    rt_size_t heapTotal = 0, heapUsed = 0, heapMax = 0;
    rt_memory_info(&heapTotal, &heapUsed, &heapMax);
    bootLogLine("[Dev ] 堆      : 建线程后 已用=%u 峰值=%u 空闲=%u / 总=%u 字节\r\n",
            static_cast<unsigned>(heapUsed), static_cast<unsigned>(heapMax),
            static_cast<unsigned>(heapTotal - heapUsed), static_cast<unsigned>(heapTotal));
}

/**
 * @brief 只读探测 W25Q128(串口调试模式本来不挂文件系统, 但"芯片在不在"必须看得见)
 *
 * 刻意**不**调 flash.init(): 它会写状态寄存器、清块保护。这里只唤醒 + 读 JEDEC/状态寄存器,
 * 对 I2C/气压计没有任何干扰, 也不会改动 flash 里的日志和参数。
 */
inline void logFlashProbe()
{
    SPI spiFlash = SPI::SPI2_SB10_MIC2_MOC3(cfg::hw::flash::kCs);
    spiFlash.init(cfg::hw::flash::kSpi);
    logSpiStatus("SPI2 FLASH", SPI2, cfg::hw::flash::kSpi, apb1ClockHz());
    W25Q128 flash(spiFlash);
    flash.releasePowerDown(); // 上一轮若进过 Power-down, 先唤醒(幂等)
    const uint32_t jedec = flash.readJEDEC();
    const uint8_t  sr1   = flash.readStatus1();
    const bool     known = (jedec != 0x000000u && jedec != 0xFFFFFFu);
    bootLogLine("[Dev ] W25Q128 : 只读探测 JEDEC=0x%06X 识别=%s SR1=0x%02X (本模式不挂文件系统)\r\n",
            static_cast<unsigned>(jedec), known ? "有应答" : "无应答(接线/供电?)",
            static_cast<unsigned>(sr1));
}
} // namespace dlx
