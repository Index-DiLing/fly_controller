//===================================================================================================
// test/bench/link_bench_main.cpp —— 链路 v2 的 P0 台架固件(飞控端)
//
// 这不是主线固件, 也不参与主线构建(EIDE srcDirs = app/DL_LIB/rt-thread, 不含 test/)。
// 由 tools/bench/build_fc_bench.ps1 单独编译: 复用 build/FLY/compile_commands.json 的 armclang
// 参数, 用本文件的 main 替换 release.cpp 的 main 参加链接, 产物在 build/bench/。
//
// 目的(协议规格 §11 / 实施计划书 §5 P0): 把 62.5k 空速真伪、F(n)(每帧空中时间 vs 长度)、
// 收↔发切换代价、UART 帧末延迟、裂包、22ms 节拍抖动测出来, 供 owner 冻结常量头。
//
// 硬件(引脚全部取自 cfg::hw::*, 不在此写死; 依据见 DL_LIB/generated/flight_config_gen.hpp):
//   USART1 PA9/PA10  控制台文本命令 + 结果, 115200 8N1(台架自定, 主线是 2Mbaud; 只影响打印, 不影响测量)
//   USART3 PD8/PD9   E220。DMA 循环接收 + IDLE 切帧(USARTDmaReceiver); 发送用 DMA1_Stream3(不阻塞、不裂包)
//   M0 PD3 / M1 PD4  只驱动为低(模式 0 透传), 不进配置模式、不发 AT、不写寄存器
//   AUX PA2          输入 + 内部上拉(外部另有 10k 上拉); EXTI2 双边沿, 边沿时刻用 DWT 打戳
//
// 三条必须知道的测量前提(否则数字会被误解, 详见 docs/链路层v2_P0台架.md):
//   1. IDLE 时间戳 ≠ 最后一字节到达时刻: 硬件要连续一个字符时间无新数据才置 IDLE。
//      @57600 8N1 一个字符 = 173.6us, 再加 ISR 延迟(1~2us) ⇒ "帧末" = IDLE 戳 − 173.6us。
//      同一 bench 内所有到达戳带同一偏置 ⇒ 差值(轮长/间隔)不受影响, 只有绝对量要减。
//   2. AUX 极性按驱动合同"高=空闲, 低=忙"(DL_LIB/E220/dlx_e220_config.h:243-244)。
//      接反/悬空会看到"永远低"或边沿乱跳; `aux` 命令会打印当前电平与累计边沿数。
//   3. E220 配置一律不碰: 62.5k/57600/信道23/32B 分包/UAUX=1/URXT=3 是主线固件写进模块 flash 的。
//      ⇒ 全新模块要先烧一次主线固件, 再烧这个 bench。
//
// 命令(USART1 输入, 大小写不敏感, 提示符 `ok>`):
//   help | cfg | aux
//   sweep [reps]             n=1..32 逐长度发帧: DMA→TC 耗时 / AUX 下降沿延迟 / AUX 低电平宽度 + F(n) 拟合
//   tx <n> <count> <gap_ms>  定长定时发送, 测稳态吞吐与发送间隔抖动
//   rx [seconds]             只收: 每段突发打印 到戳/间隔/首末字节/AUX 电平
//   echo <guard_ms> [n]      收到一段后等 guard_ms 回同长(或 n 字节), 测往返/收↔发切换
//   round [s_ms] [guard_ms] [dl_len]
//                            准协议节拍: 假定主端每 s_ms 一帧、4 上一轮; 收到第 4 帧(U3)后
//                            等 guard_ms 发一帧 D0(dl_len 字节), 统计轮长与 D0 触发偏差
//   stop                     退出 rx/echo/round
//   stat | reset             统计 / 清统计
//   rc <text>                经 E220 发一行 `@<text>\n` 给遥控器 bench(切模式/取遥控器统计)
//   send <hex..>             发任意字节, 例: send 70 02 5A 49 89
//
// 每个测量循环里, 只要串口来了一个字节就退出(不吞命令); 主循环再解析它。
//===================================================================================================
#include "stm32f4xx.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "dlx_bytebuffer.hpp"
#include "dlx_delay.hpp"
#include "dlx_dma.hpp"
#include "dlx_exti.hpp"
#include "dlx_gpio.hpp"
#include "dlx_nvic_it.h"
#include "dlx_timebase.hpp"
#include "dlx_usart.hpp"
#include "flight_config.hpp" // cfg::hw::link::* / cfg::hw::led::*

using namespace dlx;

namespace
{
    //-------------------------------------------------------------------------------------------
    // 一、常量
    //-------------------------------------------------------------------------------------------
    /** 台架控制台波特率。主线 cfg::hw::console::kBaud = 2Mbaud(日志流), 台架面向"普通 USB-TTL",
     *  固定 115200。只影响打印耗时, 不影响任何被测时序。 */
    constexpr uint32_t kConsoleBaud = 115200;

    /**
     * bench 固件版本 —— 留给自动化脚本判断"板上是不是最新 bench"。
     * 用法: 上电 banner 会打一行 `[ver] p0 bench r3 2026-10-18`; 交互也可以敲 `ver`。
     * 每次改 bench 行为(口径/命令/输出字段)都要 **+1** 并把日期改掉, 否则脚本会把旧镜像
     * 当成新镜像, 拿着旧口径的数据去拟合。
     */
    constexpr const char *kBenchVersion = "p0 bench r3 2026-10-18";

    constexpr uint32_t kLinkBaud         = cfg::hw::link::kBaud; ///< 57600(硬约束 ≤1.25×62.5k)
    constexpr uint16_t kRxDmaBytes       = 512;                   ///< 循环 DMA 缓冲(单帧 ≤32B)
    constexpr uint16_t kConsoleRxBytes   = 256;
    constexpr uint8_t  kMaxFrameBytes    = 32;                    ///< 台架单帧上限 = 模块分包长度
    constexpr uint32_t kAuxIdleTimeoutUs = 200000u;               ///< 等 AUX 空闲 200ms
    constexpr uint32_t kTxTimeoutUs      = 50000u;                ///< 等 DMA TC 50ms
    constexpr uint32_t kBusyTimeoutUs    = 60000u;                ///< 等 AUX 忙窗口结束 60ms
    constexpr uint16_t kAuxRing          = 128;
    constexpr uint16_t kRxRing           = 8;

    //-------------------------------------------------------------------------------------------
    // 二、时基
    //-------------------------------------------------------------------------------------------
    inline uint32_t cycNow() { return DWT->CYCCNT; }
    inline uint32_t cycPerUs() { return static_cast<uint32_t>(SystemCoreClock / 1000000u); }
    /** 周期差 → 微秒。只用**差值**, 差值远小于 CYCCNT 的 25.6s 回绕周期, 无符号减法天然正确。 */
    inline uint32_t usBetween(uint32_t laterCyc, uint32_t earlierCyc)
    {
        return static_cast<uint32_t>(laterCyc - earlierCyc) / cycPerUs();
    }
    inline uint32_t usToCyc(uint32_t us) { return us * cycPerUs(); }
    /** 等到某个周期戳(忙等; 台架精度优先, 不让出 CPU)。 */
    inline void waitUntilCyc(uint32_t targetCyc)
    {
        while (static_cast<int32_t>(cycNow() - targetCyc) < 0)
        {
        }
    }

    //-------------------------------------------------------------------------------------------
    // 三、全局状态
    //-------------------------------------------------------------------------------------------
    struct AuxEdge
    {
        uint32_t cyc;   ///< 边沿时刻(DWT 周期)
        uint8_t  level; ///< 边沿之后电平(1 = 高 = 空闲)
    };
    volatile AuxEdge  g_auxRing[kAuxRing];
    volatile uint16_t g_auxW       = 0;
    volatile uint16_t g_auxR       = 0;
    volatile uint32_t g_auxEdges   = 0;
    volatile uint32_t g_auxDropped = 0;
    volatile uint8_t  g_auxLevel   = 1;

    struct RxBurst
    {
        uint32_t cyc; ///< IDLE 戳(= 帧末 + 173.6us + ISR 延迟)
        uint16_t len;
        uint8_t  data[kMaxFrameBytes];
        uint8_t  auxAtIdle;
        uint8_t  truncated;
    };
    volatile RxBurst  g_rxRing[kRxRing];
    volatile uint16_t g_rxW       = 0;
    volatile uint16_t g_rxR       = 0;
    volatile uint32_t g_rxBursts  = 0;
    volatile uint32_t g_rxBytes   = 0;
    volatile uint32_t g_rxDropped = 0;

    volatile uint32_t g_txStartCyc = 0;
    volatile uint32_t g_txDoneCyc  = 0;
    volatile bool     g_txDone     = false;
    volatile uint32_t g_txDmaIrqs  = 0;

    USART      *g_con      = nullptr; ///< USART1 控制台
    RingByteBuffer *g_conRing = nullptr;
    USART      *g_link     = nullptr; ///< USART3
    DMA        *g_txDma    = nullptr; ///< USART3 TX DMA
    ByteBuffer *g_txBuf    = nullptr; ///< 发送暂存(DMA 传完前不许释放)
    GPIO       *g_auxPin   = nullptr; ///< AUX 电平
    uint32_t    g_bootCyc  = 0;       ///< 打印用时间零点

    //-------------------------------------------------------------------------------------------
    // 四、统计
    //-------------------------------------------------------------------------------------------
    struct Stat
    {
        static const uint16_t kCap = 400;
        uint32_t v[kCap];
        uint16_t n;
        uint32_t sum;
        uint32_t minV;
        uint32_t maxV;

        void reset()
        {
            n    = 0;
            sum  = 0;
            minV = 0xFFFFFFFFu;
            maxV = 0;
        }
        void add(uint32_t x)
        {
            if (n >= kCap)
            {
                return;
            }
            v[n++] = x;
            sum += x;
            if (x < minV) minV = x;
            if (x > maxV) maxV = x;
        }
        uint32_t avg() const { return (n == 0) ? 0 : (sum / n); }
        uint32_t min() const { return (n == 0) ? 0 : minV; }
        uint32_t p99() const
        {
            if (n == 0)
            {
                return 0;
            }
            static uint32_t scratch[kCap];
            const uint16_t  m = n;
            for (uint16_t i = 0; i < m; ++i)
            {
                scratch[i] = v[i];
            }
            for (uint16_t i = 1; i < m; ++i)
            {
                const uint32_t key = scratch[i];
                int16_t        j   = static_cast<int16_t>(i) - 1;
                while (j >= 0 && scratch[j] > key)
                {
                    scratch[j + 1] = scratch[j];
                    --j;
                }
                scratch[j + 1] = key;
            }
            uint32_t idx = static_cast<uint32_t>((static_cast<uint64_t>(m) * 99u) / 100u);
            if (idx >= m) idx = static_cast<uint32_t>(m - 1u);
            return scratch[idx];
        }
    };

    Stat g_stUartUs;    ///< DMA start → TC
    Stat g_stBusyUs;    ///< AUX 低电平宽度
    Stat g_stFallUs;    ///< DMA start → AUX 下降沿
    Stat g_stGapUs;     ///< 相邻两次发送起点的间隔
    Stat g_stRoundUs;   ///< 轮长(U0 到 U0)
    Stat g_stD0OffUs;   ///< D0 写起点 − U3 IDLE 戳(软件口径)
    Stat g_stD0AuxUs;   ///< D0 写起点 − 最近一次 AUX 上升沿(= U3 射频忙结束, 物理收↔发切换)
    Stat g_stEchoUs;    ///< 回发起点 − 收到帧的 IDLE 戳
    Stat g_stDlBusyUs;  ///< round: D0 的 AUX 忙时长

    uint32_t g_cmdCount = 0;
    /** 最近一次 AUX 上升沿(低→高 = 射频忙结束)。round 用它给 D0 偏移做物理口径。 */
    uint32_t g_lastAuxRiseCyc = 0;
} // namespace

//===================================================================================================
// 五、中断
//===================================================================================================
static void auxEdgeIsr(uint8_t /*line*/, void * /*ctx*/)
{
    const uint32_t c     = DWT->CYCCNT;
    const uint8_t  level = static_cast<uint8_t>(g_auxPin->read() ? 1u : 0u);
    g_auxLevel           = level;
    ++g_auxEdges;
    const uint16_t w    = g_auxW;
    const uint16_t next = static_cast<uint16_t>((w + 1u) % kAuxRing);
    if (next == g_auxR)
    {
        ++g_auxDropped;
        return;
    }
    g_auxRing[w].cyc   = c;
    g_auxRing[w].level = level;
    g_auxW             = next;
}

static void linkBurstIsr(const uint8_t *data, uint16_t len, void * /*ctx*/)
{
    if (len == 0u)
    {
        return;
    }
    const uint32_t c    = DWT->CYCCNT;
    const uint16_t w    = g_rxW;
    const uint16_t next = static_cast<uint16_t>((w + 1u) % kRxRing);
    if (next == g_rxR)
    {
        ++g_rxDropped;
        return;
    }
    RxBurst *b  = const_cast<RxBurst *>(&g_rxRing[w]);
    b->cyc      = c;
    b->len      = (len <= kMaxFrameBytes) ? len : kMaxFrameBytes;
    b->truncated = (len > kMaxFrameBytes) ? 1u : 0u;
    b->auxAtIdle = static_cast<uint8_t>(g_auxPin->read() ? 1u : 0u);
    memcpy(b->data, data, b->len);
    g_rxW = next;
    ++g_rxBursts;
    g_rxBytes += len;
}

/**
 * USART3 TX 的 DMA 完成中断(DMA1_Stream3)。
 * stm32f4xx_it.cpp 只给 DMA2_Stream5(GNSS) 留了转发器, 这条没有 —— startup 里是 WEAK, 这里覆盖。
 * 只在本 bench 镜像里存在(主线不编译 test/bench)。
 */
extern "C" void DMA1_Stream3_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
        g_txDoneCyc = DWT->CYCCNT;
        g_txDone    = true;
        ++g_txDmaIrqs;
    }
}

//===================================================================================================
// 六、控制台
//===================================================================================================
static void conPrintf(const char *fmt, ...)
{
    char    buf[192];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0 || g_con == nullptr)
    {
        return;
    }
    const uint16_t len = static_cast<uint16_t>((n < static_cast<int>(sizeof(buf))) ? n : static_cast<int>(sizeof(buf) - 1));
    g_con->send(reinterpret_cast<const uint8_t *>(buf), len);
}
static void printPrompt() { g_con->send(reinterpret_cast<const uint8_t *>("ok> "), 4); }
static bool consoleAvailable() { return g_conRing->available() > 0u; }

//===================================================================================================
// 七、底层时序动作
//===================================================================================================
static bool waitAuxIdle(uint32_t timeoutUs)
{
    const uint32_t t0 = cycNow();
    while (g_auxPin->read() == 0)
    {
        if (usBetween(cycNow(), t0) > timeoutUs)
        {
            return false;
        }
    }
    return true;
}

static void auxDrop() { g_auxR = g_auxW; }

static bool auxPop(AuxEdge &e)
{
    if (g_auxR == g_auxW)
    {
        return false;
    }
    const uint16_t idx = g_auxR;
    e.cyc              = g_auxRing[idx].cyc;
    e.level            = g_auxRing[idx].level;
    g_auxR             = static_cast<uint16_t>((g_auxR + 1u) % kAuxRing);
    return true;
}

/**
 * 消费掉所有待处理的 AUX 边沿, 只记住"最近一次上升沿"。
 *
 * 为什么必须有它(owner 实测抓到的口径问题): `rx`/`echo`/`round` 这些**长时间**循环如果不消费
 * AUX 环形缓冲, 边沿会在几秒内把 128 深度的环撑爆 → `auxDrop` 疯涨(实测 round 5min 后 24512)。
 * 那个数字**不是丢帧**, 只是没人读环。所以: 每个循环每轮都调用本函数, 把环清空;
 * 同时顺手记下上升沿, 给 round 的"D0 相对 U3 射频结束"提供物理口径。
 */
static void drainAuxEdges()
{
    AuxEdge e;
    while (auxPop(e))
    {
        if (e.level != 0u)
        {
            g_lastAuxRiseCyc = e.cyc;
        }
    }
}

static bool rxPop(RxBurst &out)
{
    if (g_rxR == g_rxW)
    {
        return false;
    }
    const uint16_t idx = g_rxR;
    out.cyc            = g_rxRing[idx].cyc;
    out.len            = g_rxRing[idx].len;
    out.truncated      = g_rxRing[idx].truncated;
    out.auxAtIdle      = g_rxRing[idx].auxAtIdle;
    for (uint16_t i = 0u; i < out.len; ++i)
    {
        out.data[i] = g_rxRing[idx].data[i];
    }
    g_rxR = static_cast<uint16_t>((g_rxR + 1u) % kRxRing);
    return true;
}

/** 一次连续 DMA 写。payload=nullptr ⇒ 发台阶字节 0xA0+i(sweep 用)。 */
static bool txStart(const uint8_t *payload, uint16_t n)
{
    if (g_txDma == nullptr || g_txBuf == nullptr || n == 0u || n > kMaxFrameBytes)
    {
        return false;
    }
    g_txDone = false;
    g_txBuf->reset();
    if (payload != nullptr)
    {
        g_txBuf->write(payload, n);
    }
    else
    {
        for (uint16_t i = 0; i < n; ++i)
        {
            g_txBuf->write(static_cast<uint8_t>(0xA0u + (i & 0x1Fu)));
        }
    }
    g_txStartCyc = cycNow();
    g_txDma->reset(n);
    g_txDma->start();
    return true;
}

static bool waitTxDone(uint32_t timeoutUs)
{
    const uint32_t t0 = cycNow();
    while (!g_txDone)
    {
        if (usBetween(cycNow(), t0) > timeoutUs)
        {
            return false;
        }
    }
    return true;
}

/** 等一个完整 AUX 忙窗口: startCyc 之后第一个下降沿 → 它之后第一个上升沿。 */
static bool waitBusyWindow(uint32_t startCyc, uint32_t &fallUs, uint32_t &busyUs, uint32_t timeoutUs)
{
    bool     haveFall = false;
    uint32_t fallCyc  = 0;
    AuxEdge  e;
    while (usBetween(cycNow(), startCyc) <= timeoutUs)
    {
        while (auxPop(e))
        {
            if (!haveFall)
            {
                if (e.level == 0u)
                {
                    haveFall = true;
                    fallCyc  = e.cyc;
                    fallUs   = usBetween(e.cyc, startCyc);
                }
            }
            else if (e.level != 0u)
            {
                busyUs = usBetween(e.cyc, fallCyc);
                return true;
            }
        }
    }
    return false;
}

struct FrameMeas
{
    bool     ok;
    uint32_t uartUs;
    uint32_t fallUs;
    uint32_t busyUs;
};

static FrameMeas measureFrame(const uint8_t *payload, uint16_t n)
{
    FrameMeas m;
    m.ok     = false;
    m.uartUs = 0;
    m.fallUs = 0;
    m.busyUs = 0;
    if (!waitAuxIdle(kAuxIdleTimeoutUs))
    {
        return m; // 模块一直忙: 不硬灌(灌了只会裂包/丢)
    }
    auxDrop();
    if (!txStart(payload, n))
    {
        return m;
    }
    if (!waitTxDone(kTxTimeoutUs))
    {
        return m;
    }
    m.uartUs = usBetween(g_txDoneCyc, g_txStartCyc);
    if (!waitBusyWindow(g_txStartCyc, m.fallUs, m.busyUs, kBusyTimeoutUs))
    {
        return m;
    }
    m.ok = true;
    g_stUartUs.add(m.uartUs);
    g_stBusyUs.add(m.busyUs);
    g_stFallUs.add(m.fallUs);
    return m;
}

//===================================================================================================
// 八、命令实现
//===================================================================================================
static void printCfg()
{
    conPrintf("[cfg] USART1 PA9/PA10 %u 8N1 (bench-only)\r\n", static_cast<unsigned>(kConsoleBaud));
    conPrintf("[cfg] USART3 PD8/PD9 %u 8N1  M0=PD3 M1=PD4 AUX=PA2 EXTI2 both\r\n",
              static_cast<unsigned>(kLinkBaud));
    conPrintf("[cfg] frameMax=%uB (module packet=32B) air=62.5k  bench never writes E220 config\r\n",
              static_cast<unsigned>(kMaxFrameBytes));
    conPrintf("[cfg] core=%uHz  cycPerUs=%u  IDLE char=%uus\r\n", static_cast<unsigned>(SystemCoreClock),
              static_cast<unsigned>(cycPerUs()), static_cast<unsigned>((10u * 1000000u) / kLinkBaud));
}

static void printAux()
{
    conPrintf("[aux] now=%s edges=%u drop=%u\r\n", g_auxPin->read() ? "HIGH(idle)" : "LOW(busy)",
              static_cast<unsigned>(g_auxEdges), static_cast<unsigned>(g_auxDropped));
}

static void printBurst(const RxBurst &b, uint32_t prevCyc)
{
    conPrintf("[rx] len=%u t=%uus", static_cast<unsigned>(b.len),
              static_cast<unsigned>(usBetween(b.cyc, g_bootCyc)));
    if (prevCyc != 0u)
    {
        conPrintf(" dt=%uus", static_cast<unsigned>(usBetween(b.cyc, prevCyc)));
    }
    conPrintf(" aux=%u%s [", static_cast<unsigned>(b.auxAtIdle), b.truncated ? " TRUNC" : "");
    const uint16_t show = (b.len < 8u) ? b.len : 8u;
    for (uint16_t i = 0; i < show; ++i)
    {
        conPrintf("%02X ", b.data[i]);
    }
    conPrintf("]\r\n");
}

static void cmdSweep(int reps)
{
    if (reps < 1) reps = 1;
    if (reps > 5) reps = 5;
    conPrintf("[sweep] n=1..32 reps=%d\r\n", reps);
    conPrintf("   n   uartUs   fallUs   busyUs\r\n");
    int64_t  sx = 0, sy = 0, sxx = 0, sxy = 0;
    uint32_t pts = 0;
    for (uint16_t n = 1u; n <= kMaxFrameBytes; ++n)
    {
        uint32_t bestU = 0xFFFFFFFFu, bestF = 0xFFFFFFFFu, bestB = 0xFFFFFFFFu;
        uint32_t got   = 0;
        for (int r = 0; r < reps; ++r)
        {
            const FrameMeas m = measureFrame(nullptr, n);
            if (!m.ok)
            {
                continue;
            }
            ++got;
            if (m.uartUs < bestU) bestU = m.uartUs;
            if (m.fallUs < bestF) bestF = m.fallUs;
            if (m.busyUs < bestB) bestB = m.busyUs;
            if (consoleAvailable())
            {
                break;
            }
        }
        if (got == 0u)
        {
            conPrintf("  %2u   (no window)\r\n", static_cast<unsigned>(n));
            continue;
        }
        conPrintf("  %2u  %7u  %7u  %7u\r\n", static_cast<unsigned>(n), static_cast<unsigned>(bestU),
                  static_cast<unsigned>(bestF), static_cast<unsigned>(bestB));
        if (n >= 8u)
        {
            sx += n;
            sy += bestB;
            sxx += static_cast<int64_t>(n) * n;
            sxy += static_cast<int64_t>(n) * bestB;
            ++pts;
        }
    }
    if (pts >= 3u)
    {
        const int64_t den = static_cast<int64_t>(pts) * sxx - sx * sx;
        if (den != 0)
        {
            const int64_t b100 = ((static_cast<int64_t>(pts) * sxy - sx * sy) * 100) / den;
            const int64_t a100 = ((sy * 100) - b100 * sx) / pts;
            const long long aI = static_cast<long long>(a100 / 100);
            const long long aF = static_cast<long long>((a100 % 100 < 0) ? -(a100 % 100) : (a100 % 100));
            const long long bI = static_cast<long long>(b100 / 100);
            const long long bF = static_cast<long long>((b100 % 100 < 0) ? -(b100 % 100) : (b100 % 100));
            conPrintf("[fit] F(n)=%lld.%02lldus + %lld.%02lldus/B*n  (n>=8, %u pts)\r\n", aI, aF, bI, bF,
                      static_cast<unsigned>(pts));
            conPrintf("[fit] => LinkAirtimeBaseUs=%lld LinkAirtimePerByteUs=%lld (取整)\r\n", aI, bI);
        }
    }
    else
    {
        conPrintf("[fit] too few points (need >=3 valid rows with n>=8)\r\n");
    }
}

static void cmdTx(int n, int count, int gapMs)
{
    if (n < 1 || n > static_cast<int>(kMaxFrameBytes) || count < 1 || count > 1000 || gapMs < 0 || gapMs > 5000)
    {
        conPrintf("[tx] usage: tx <1..32> <1..1000> <0..5000ms>\r\n");
        return;
    }
    conPrintf("[tx] n=%d count=%d gap=%dms\r\n", n, count, gapMs);
    uint32_t prevStart = 0;
    uint32_t okCount = 0, miss = 0;
    for (int i = 0; i < count; ++i)
    {
        drainAuxEdges(); // 长时间发送时也要清 AUX 环(这里的边沿由 measureFrame 内部消费)
        if (i > 0)
        {
            const uint32_t target = prevStart + usToCyc(static_cast<uint32_t>(gapMs) * 1000u);
            waitUntilCyc(target);
        }
        const FrameMeas m = measureFrame(nullptr, static_cast<uint16_t>(n));
        if (!m.ok)
        {
            ++miss;
            continue;
        }
        ++okCount;
        if (prevStart != 0u)
        {
            g_stGapUs.add(usBetween(g_txStartCyc, prevStart));
        }
        prevStart = g_txStartCyc;
        if ((i % 10) == 0)
        {
            conPrintf("  #%d uart=%u fall=%u busy=%u\r\n", i, static_cast<unsigned>(m.uartUs),
                      static_cast<unsigned>(m.fallUs), static_cast<unsigned>(m.busyUs));
        }
        if (consoleAvailable())
        {
            break;
        }
    }
    conPrintf("[tx] ok=%u miss=%u\r\n", static_cast<unsigned>(okCount), static_cast<unsigned>(miss));
}

static void cmdRx(int seconds)
{
    conPrintf("[rx] listen %ds (any key ends)\r\n", seconds);
    const uint32_t t0   = cycNow();
    uint32_t       prev = 0;
    uint32_t       seen = 0;
    while (true)
    {
        // 长时间循环必须消费 AUX 边沿, 否则 128 深的环会被撑爆(见 drainAuxEdges 注释)
        drainAuxEdges();
        RxBurst b;
        if (rxPop(b))
        {
            printBurst(b, prev);
            prev = b.cyc;
            ++seen;
            continue;
        }
        if (seconds > 0 && usBetween(cycNow(), t0) >= static_cast<uint32_t>(seconds) * 1000000u)
        {
            break;
        }
        if (consoleAvailable())
        {
            break;
        }
    }
    conPrintf("[rx] end bursts=%u\r\n", static_cast<unsigned>(seen));
    if (seen == 0u)
    {
        conPrintf("[rx] 什么也没收到: 查遥控器是否在发、两端 信道/空速/地址 是否一致、AUX 是否一直低\r\n");
    }
}

static void cmdEcho(int guardMs, int n)
{
    if (guardMs < 0 || guardMs > 200)
    {
        conPrintf("[echo] usage: echo <0..200ms> [n<=32]\r\n");
        return;
    }
    conPrintf("[echo] guard=%dms n=%d\r\n", guardMs, n);
    uint32_t prev = 0, replies = 0;
    while (true)
    {
        drainAuxEdges(); // 同上: 别让 AUX 环撑爆
        RxBurst b;
        if (rxPop(b))
        {
            printBurst(b, prev);
            prev = b.cyc;
            const uint16_t len = (n > 0) ? static_cast<uint16_t>(n) : b.len;
            waitUntilCyc(b.cyc + usToCyc(static_cast<uint32_t>(guardMs) * 1000u));
            const uint32_t got = usBetween(cycNow(), b.cyc);
            g_stEchoUs.add(got);
            if (!txStart(b.data, len) || !waitTxDone(kTxTimeoutUs))
            {
                conPrintf("  -> reply FAILED\r\n");
                continue;
            }
            const uint32_t start = g_txStartCyc;
            uint32_t fallUs = 0, busyUs = 0;
            if (waitBusyWindow(start, fallUs, busyUs, kBusyTimeoutUs))
            {
                g_stBusyUs.add(busyUs);
                conPrintf("  -> reply len=%u guardUs=%u busyUs=%u\r\n", static_cast<unsigned>(len),
                          static_cast<unsigned>(got), static_cast<unsigned>(busyUs));
            }
            ++replies;
            continue;
        }
        if (consoleAvailable())
        {
            break;
        }
    }
    conPrintf("[echo] end replies=%u\r\n", static_cast<unsigned>(replies));
}

static void cmdRound(int sMs, int guardMs, int dlLen)
{
    if (sMs < 10 || sMs > 100 || guardMs < 0 || guardMs > 50 || dlLen < 1 || dlLen > static_cast<int>(kMaxFrameBytes))
    {
        conPrintf("[round] usage: round <10..100ms> <0..50ms> <1..32>\r\n");
        return;
    }
    /**
     * 口径说明(写进打印里, 防止 owner 再按 v2 的 120ms 轮次理解):
     *   本命令的"一轮"是 **4 个连续 RX burst(U0..U3) + 1 个 D0**, 用到来判断"能不能在 4 个上行
     *   之后插入一帧下行"。它**不是** v2 协议的 120ms 轮次(n=4,m=1,s=22ms 的 5 槽超帧), 也不是
     *   "轮长应当恒等于 4*s"——只要 D0 的空中时间 > 一个槽, 下一帧 U0 就会被推迟一个槽,
     *   实测轮长就会是 5*s。两种结果都要如实打印, 不做静默修正。
     */
    conPrintf("[round] s=%dms guard=%dms dlLen=%d\r\n", sMs, guardMs, dlLen);
    conPrintf("[round] 口径: 一轮 = 4 个 RX burst(U0..U3) + 1 个 D0; 不是 v2 的 120ms 轮次\r\n");
    conPrintf("[round] ideal: 4*s=%dms; 若 D0 占满一个槽(21B≈25ms>s), 下一帧晚到 => 5*s=%dms 也属正常\r\n",
              4 * sMs, 5 * sMs);
    uint8_t  idx          = 0;
    uint32_t prevU0Cyc    = 0;
    uint32_t u3Cyc        = 0;
    uint32_t prevBurstCyc = 0;
    bool     dlPending    = false;
    uint32_t rounds = 0, dlSent = 0, bursts = 0, txtSkip = 0;
    const uint32_t rxDrop0  = g_rxDropped;
    const uint32_t auxDrop0 = g_auxDropped;

    while (true)
    {
        // 先把 AUX 边沿消费掉: 否则 5 分钟能把 128 深的环撑爆, auxDrop 会变成一个假故障
        // (owner 实测 24512 就是这个原因, 不是丢帧)。
        drainAuxEdges();

        RxBurst b;
        if (rxPop(b))
        {
            /**
             * 过滤"不是测量载荷"的 burst(2026-10-18 修, owner 实测 roundUs 131/176ms 甚至 4.89s):
             *   遥控器 bench 每 5s 自动回一行 `#STAT`(约 120B 可打印 ASCII), 飞控 bench 的
             *   `rc ...` 也会收到 `#ACK/#PONG/#BOOT`。它们若参与 idx/轮次统计, 会把
             *   "4 个上行 + 1 个 D0"的节奏切开, 让轮长出现离群值。
             * 判据: 首字节在 0x20..0x7E(可打印 ASCII) ⇒ 文本行, 跳过并单独计数。
             * 测量载荷首字节是 0xA0..(见 txStart 的台阶字节), 不会命中这个区间。
             */
            if (b.len > 0u && b.data[0] >= 0x20u && b.data[0] <= 0x7Eu)
            {
                ++txtSkip;
                continue;
            }
            ++bursts;
            if (prevBurstCyc != 0u && usBetween(b.cyc, prevBurstCyc) > static_cast<uint32_t>(sMs) * 2000u)
            {
                idx = 0; // 间隔超过 2 槽: 认为重开一轮
            }
            prevBurstCyc = b.cyc;
            if (idx == 0u)
            {
                if (prevU0Cyc != 0u)
                {
                    const uint32_t rl = usBetween(b.cyc, prevU0Cyc);
                    g_stRoundUs.add(rl);
                    ++rounds;
                    const char *tag = "";
                    if (rl >= static_cast<uint32_t>(sMs) * 4500u && rl < static_cast<uint32_t>(sMs) * 5500u)
                    {
                        tag = "  [=5*s: D0 吃掉了下一帧上行]";
                    }
                    conPrintf("[round #%u] len=%uus (ideal 4*s=%d, 5*s=%d)%s\r\n", static_cast<unsigned>(rounds),
                              static_cast<unsigned>(rl), 4 * sMs, 5 * sMs, tag);
                }
                prevU0Cyc = b.cyc;
            }
            else if (idx == 3u)
            {
                u3Cyc     = b.cyc;
                dlPending = true;
            }
            idx = static_cast<uint8_t>((idx + 1u) & 0x03u);
        }

        if (dlPending)
        {
            if (!waitAuxIdle(kAuxIdleTimeoutUs))
            {
                dlPending = false; // AUX 一直低: 这一轮放弃 D0(记不到样本, 但不会撞车)
                continue;
            }
            drainAuxEdges(); // 这时 U3 的 AUX 上升沿已经进了环: 取出来当物理口径
            waitUntilCyc(u3Cyc + usToCyc(static_cast<uint32_t>(guardMs) * 1000u));
            const uint32_t off = usBetween(cycNow(), u3Cyc);
            g_stD0OffUs.add(off);
            if (txStart(nullptr, static_cast<uint16_t>(dlLen)))
            {
                const uint32_t start = g_txStartCyc;
                // 物理口径: D0 写起点 − U3 射频忙结束(AUX 上升沿)。它不含 IDLE 的 173.6us 偏置。
                uint32_t offAux = 0u;
                const int32_t sinceRise = static_cast<int32_t>(start - g_lastAuxRiseCyc);
                if (g_lastAuxRiseCyc != 0u && sinceRise > 0)
                {
                    offAux = static_cast<uint32_t>(sinceRise) / cycPerUs();
                    g_stD0AuxUs.add(offAux);
                }
                if (waitTxDone(kTxTimeoutUs))
                {
                    ++dlSent;
                    uint32_t fallUs = 0, busyUs = 0;
                    if (waitBusyWindow(start, fallUs, busyUs, kBusyTimeoutUs))
                    {
                        g_stDlBusyUs.add(busyUs);
                    }
                    conPrintf("  -> D0 off=%uus(软件, 从 IDLE) offAux=%uus(物理, 从 AUX↑) busy=%uus\r\n",
                              static_cast<unsigned>(off), static_cast<unsigned>(offAux),
                              static_cast<unsigned>(busyUs));
                }
            }
            dlPending = false;
            idx       = 0; // D0 之后下一帧就是下一轮 U0
        }

        if (consoleAvailable())
        {
            break;
        }
    }
    // 自检: 每轮应当恰好 4 个 burst(U0..U3)。>4 说明多/少算了一个(RX 队列积压、D0 被当成 RX、
    // 或丢帧后 idx 重对齐); 这里只报比例, 由 owner 结合 roundUs 判断原因。
    // 结束行字段顺序(自动化脚本按这个解析, 不要改顺序):
    //   rounds= / dlSent= / bursts= / bursts/round= / txtSkip=
    const uint32_t bpr100 = (rounds == 0u) ? 0u : static_cast<uint32_t>((static_cast<uint64_t>(bursts) * 100u) / rounds);
    conPrintf("[round] end rounds=%u dlSent=%u bursts=%u bursts/round=%u.%02u txtSkip=%u (expect 4.00; 5.00 => D0 吃掉一帧)\r\n",
              static_cast<unsigned>(rounds), static_cast<unsigned>(dlSent), static_cast<unsigned>(bursts),
              static_cast<unsigned>(bpr100 / 100u), static_cast<unsigned>(bpr100 % 100u),
              static_cast<unsigned>(txtSkip));
    conPrintf("[round] drop: rxDrop=%u (+%u) auxDrop=%u (+%u)  <- 只看 rxDrop; auxDrop 已由 drain 归零\r\n",
              static_cast<unsigned>(g_rxDropped), static_cast<unsigned>(g_rxDropped - rxDrop0),
              static_cast<unsigned>(g_auxDropped), static_cast<unsigned>(g_auxDropped - auxDrop0));
}

static void cmdStat()
{
    conPrintf("---- stat ----\r\n");
    conPrintf("cmd=%u rxBursts=%u rxBytes=%u rxDrop=%u auxEdges=%u auxDrop=%u txIrq=%u\r\n",
              static_cast<unsigned>(g_cmdCount), static_cast<unsigned>(g_rxBursts), static_cast<unsigned>(g_rxBytes),
              static_cast<unsigned>(g_rxDropped), static_cast<unsigned>(g_auxEdges),
              static_cast<unsigned>(g_auxDropped), static_cast<unsigned>(g_txDmaIrqs));
    conPrintf("note: 某个统计 n=0 时显示 -- (无样本); min 不再把 0 当未初始化哨兵\r\n");
    struct Row
    {
        const char *name;
        const Stat *s;
    };
    static const Row rows[] = {
        {"uartUs (start->TC)   ", &g_stUartUs},
        {"auxBusyUs            ", &g_stBusyUs},
        {"auxFallUs (start->f) ", &g_stFallUs},
        {"gapUs (start-start)  ", &g_stGapUs},
        {"roundUs (U0->U0)     ", &g_stRoundUs},
        {"d0OffUs (U3->D0 soft)", &g_stD0OffUs},
        {"d0AuxUs (auxRise->D0)", &g_stD0AuxUs},
        {"echoUs (rx->reply)   ", &g_stEchoUs},
        {"dlBusyUs (D0 busy)   ", &g_stDlBusyUs},
    };
    for (uint32_t i = 0; i < sizeof(rows) / sizeof(rows[0]); ++i)
    {
        const Stat &s = *rows[i].s;
        if (s.n == 0u)
        {
            // 空样本: 不能打印 0（0 是合法测量值, 会被 owner 当成"最小就是 0"）。
            conPrintf("%s n=0 (--)\r\n", rows[i].name);
            continue;
        }
        conPrintf("%s n=%u min=%u avg=%u max=%u p99=%u\r\n", rows[i].name, static_cast<unsigned>(s.n),
                  static_cast<unsigned>(s.min()), static_cast<unsigned>(s.avg()), static_cast<unsigned>(s.maxV),
                  static_cast<unsigned>(s.p99()));
    }
}

/** 清空全部统计 + 诊断计数(上电时也调一次: 否则 min 的初值 0 会被当成真实最小值打印出来)。 */
static void resetAllStats()
{
    g_stUartUs.reset();
    g_stBusyUs.reset();
    g_stFallUs.reset();
    g_stGapUs.reset();
    g_stRoundUs.reset();
    g_stD0OffUs.reset();
    g_stD0AuxUs.reset();
    g_stEchoUs.reset();
    g_stDlBusyUs.reset();
    g_cmdCount        = 0;
    g_rxDropped       = 0;
    g_auxDropped      = 0;
    g_lastAuxRiseCyc  = 0;
}

static void cmdReset()
{
    resetAllStats();
    conPrintf("[reset] ok\r\n");
}

static void cmdRc(const char *text)
{
    char line[80];
    const int n = snprintf(line, sizeof(line), "@%s\n", text);
    if (n <= 0)
    {
        return;
    }
    const uint16_t len =
        static_cast<uint16_t>((n < static_cast<int>(sizeof(line))) ? n : static_cast<int>(sizeof(line) - 1));
    const FrameMeas m = measureFrame(reinterpret_cast<const uint8_t *>(line), len);
    conPrintf("[rc] %s sent=%u uart=%uus busy=%uus\r\n", m.ok ? "ok" : "FAILED", static_cast<unsigned>(len),
              static_cast<unsigned>(m.uartUs), static_cast<unsigned>(m.busyUs));
}

static int hexVal(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void cmdSendHex(const char *args)
{
    uint8_t  buf[kMaxFrameBytes];
    uint16_t n    = 0;
    const char *p = args;
    while (*p != '\0' && n < kMaxFrameBytes)
    {
        while (*p == ' ') ++p;
        if (*p == '\0') break;
        const int hi = hexVal(*p++);
        if (hi < 0 || *p == '\0') break;
        const int lo = hexVal(*p++);
        if (lo < 0) break;
        buf[n++] = static_cast<uint8_t>((hi << 4) | lo);
    }
    if (n == 0u)
    {
        conPrintf("[send] usage: send 70 02 5A 49 89\r\n");
        return;
    }
    const FrameMeas m = measureFrame(buf, n);
    conPrintf("[send] n=%u %s uart=%uus busy=%uus\r\n", static_cast<unsigned>(n), m.ok ? "ok" : "FAILED",
              static_cast<unsigned>(m.uartUs), static_cast<unsigned>(m.busyUs));
}

//===================================================================================================
// 九、命令解析
//===================================================================================================
static char    g_line[96];
static uint8_t g_lineLen = 0;

static int parseUInt(const char *s, int fallback)
{
    if (s == nullptr || *s == '\0')
    {
        return fallback;
    }
    int v = 0;
    while (*s >= '0' && *s <= '9')
    {
        v = v * 10 + (*s - '0');
        ++s;
    }
    return v;
}

/** 取下一个空格分隔的参数(改写串内的空格为 '\0')。 */
static char *nextArg(char **cursor)
{
    char *p = *cursor;
    if (p == nullptr)
    {
        return nullptr;
    }
    while (*p == ' ') ++p;
    if (*p == '\0')
    {
        *cursor = p;
        return nullptr;
    }
    char *start = p;
    while (*p != '\0' && *p != ' ') ++p;
    if (*p == ' ')
    {
        *p++ = '\0';
    }
    *cursor = p;
    return start;
}

static void dispatch(char *line)
{
    ++g_cmdCount;
    while (*line == ' ') ++line;
    char *end = line + strlen(line);
    while (end > line && end[-1] == ' ') --end;
    *end = '\0';
    if (*line == '\0')
    {
        return;
    }
    for (char *p = line; *p != '\0'; ++p)
    {
        if (*p == '_') *p = ' ';
        else if (*p >= 'A' && *p <= 'Z') *p = static_cast<char>(*p - 'A' + 'a');
    }
    char *cur = line;
    char *cmd = nextArg(&cur);
    if (cmd == nullptr)
    {
        return;
    }

    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0)
    {
        conPrintf("ver (bench fw version) | help cfg aux\r\n");
        conPrintf("sweep [reps] | tx <n> <count> <gap_ms> | rx [s] | echo <guard_ms> [n]\r\n");
        conPrintf("round [s_ms] [guard_ms] [dl_len] | stop | stat | reset | rc <text> | send <hex..>\r\n");
    }
    else if (strcmp(cmd, "ver") == 0 || strcmp(cmd, "version") == 0)
    {
        conPrintf("[ver] %s\r\n", kBenchVersion);
    }
    else if (strcmp(cmd, "cfg") == 0)
    {
        printCfg();
    }
    else if (strcmp(cmd, "aux") == 0)
    {
        printAux();
    }
    else if (strcmp(cmd, "sweep") == 0)
    {
        cmdSweep(parseUInt(nextArg(&cur), 3));
    }
    else if (strcmp(cmd, "tx") == 0)
    {
        cmdTx(parseUInt(nextArg(&cur), 0), parseUInt(nextArg(&cur), 0), parseUInt(nextArg(&cur), 0));
    }
    else if (strcmp(cmd, "rx") == 0)
    {
        cmdRx(parseUInt(nextArg(&cur), 0));
    }
    else if (strcmp(cmd, "echo") == 0)
    {
        cmdEcho(parseUInt(nextArg(&cur), 0), parseUInt(nextArg(&cur), 0));
    }
    else if (strcmp(cmd, "round") == 0)
    {
        cmdRound(parseUInt(nextArg(&cur), 22), parseUInt(nextArg(&cur), 3), parseUInt(nextArg(&cur), 21));
    }
    else if (strcmp(cmd, "stop") == 0)
    {
        conPrintf("[stop] idle\r\n");
    }
    else if (strcmp(cmd, "stat") == 0)
    {
        cmdStat();
    }
    else if (strcmp(cmd, "reset") == 0)
    {
        cmdReset();
    }
    else if (strcmp(cmd, "rc") == 0)
    {
        // 其余整行原样带给遥控器(允许带空格)
        char *rest = cur;
        while (rest != nullptr && *rest == ' ') ++rest;
        cmdRc(rest ? rest : "");
    }
    else if (strcmp(cmd, "send") == 0)
    {
        char *rest = cur;
        while (rest != nullptr && *rest == ' ') ++rest;
        cmdSendHex(rest ? rest : "");
    }
    else
    {
        conPrintf("[?] unknown '%s' (help)\r\n", cmd);
    }
}

/** 从控制台环形缓冲取一行并执行(一次只处理一行, 保持"命令之间可插入测量"的语义)。 */
static void serviceConsole()
{
    while (true)
    {
        uint8_t b = 0;
        if (!g_conRing->read(b))
        {
            return;
        }
        if (b == '\r')
        {
            continue;
        }
        if (b == '\n')
        {
            g_line[g_lineLen] = '\0';
            dispatch(g_line);
            g_lineLen = 0;
            printPrompt();
            continue;
        }
        if (g_lineLen < sizeof(g_line) - 1u)
        {
            g_line[g_lineLen++] = static_cast<char>(b);
        }
    }
}

//===================================================================================================
// 十、main
//===================================================================================================
int main()
{
    // ① 中断分组 + DWT 微秒时基(168MHz ⇒ 168 周期 = 1us)
    DLX_NVIC_AutoConfig();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_bootCyc = cycNow();

    // ② M0/M1 = 0 → 模式 0 透传(只驱动电平, 不进配置模式)
    GPIO m0(cfg::hw::link::kM0);
    m0.init(GPIOModeProfile::OUT_PP_NOPULL_2MHz);
    m0 = 0;
    GPIO m1(cfg::hw::link::kM1);
    m1.init(GPIOModeProfile::OUT_PP_NOPULL_2MHz);
    m1 = 0;

    // ③ AUX 输入(内部上拉; 绝不输出/下拉 —— 上电被拉低 >1s 会把模块打进 IAP 升级模式)
    GPIO auxPin(cfg::hw::link::kAux);
    auxPin.init(GPIOModeProfile::IN_UP);
    g_auxPin   = &auxPin;
    g_auxLevel = static_cast<uint8_t>(auxPin.read() ? 1u : 0u);

    // ④ USART1 控制台
    static uint8_t conRxStorage[kConsoleRxBytes];
    static ByteBuffer conRxView(conRxStorage, sizeof(conRxStorage));
    static RingByteBuffer conRing(conRxView);
    g_conRing = &conRing;
    static USART con = USART::USART1_TA9_RAA();
    con.init(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, kConsoleBaud, conRing);
    g_con = &con;

    // ⑤ USART3: DMA 循环接收 + IDLE 切帧
    static USART linkUart = USART::USART3_TD8_RD9();
    g_link                = &linkUart;
    static uint8_t rxStorage[kRxDmaBytes];
    static USARTDmaReceiver rxRecv(linkUart, rxStorage, sizeof(rxStorage));
    rxRecv.setHandler(&linkBurstIsr, nullptr);
    rxRecv.start(USARTModeProfile::WL8_SB1_PN_RXTX_FCN, kLinkBaud, NVICPriorityProfile::P0_S3);

    // ⑥ USART3 发送 DMA(一次连续写)
    static uint8_t  txStorage[kMaxFrameBytes];
    static ByteBuffer txView(txStorage, sizeof(txStorage));
    g_txBuf          = &txView;
    static DMA txDma = linkUart.setDMASend(txView);
    g_txDma          = &txDma;
    txDma.setITRequest(DMAITProfile::TC, ENABLE);
    txDma.initNVIC(NVICPriorityProfile::P0_S3);

    // ⑦ AUX EXTI2 双边沿
    static EXTILine auxLine(cfg::hw::link::kAux, EXTITrigger::Both);
    const bool      auxOk = auxLine.init(NVICPriorityProfile::P0_S2, &auxEdgeIsr, nullptr);

    // ⑧ banner
    // 统计必须在任何测量之前清零: Stat 的 minV 初值由 reset() 设成 0xFFFFFFFF; 不清零时全局对象的
    // minV=0 会被当成"真实最小值"打印出来(owner 记录里 "stat min=0" 就是这个, 不是测量结果)。
    resetAllStats();
    delay_ms(2);
    conPrintf("\r\n\r\n=== DLX link bench (P0) ===\r\n");
    // 版本行放在 banner 第一条: 自动化脚本靠它判断"板上是不是最新 bench"。
    conPrintf("[ver] %s\r\n", kBenchVersion);
    printCfg();
    printAux();
    conPrintf("[aux] EXTI2 %s\r\n", auxOk ? "armed (both edges)" : "**FAILED** (line already bound)");
    conPrintf("type 'help'\r\n");
    printPrompt();

    // ⑨ 主循环: 控制台是唯一的"输入", 其余命令都是阻塞式测量(内部检查 consoleAvailable 以便中途退出)
    while (true)
    {
        serviceConsole();
        drainAuxEdges(); // 停在提示符时对端可能还在发: 不消费边沿会把 AUX 环撑爆
        // 空闲时也把收到的突发打出来(rx 之外的模式下也顺便看得到对端在发)
        RxBurst b;
        if (rxPop(b))
        {
            printBurst(b, 0);
            printPrompt();
        }
    }
}
