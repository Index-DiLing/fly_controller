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

using namespace dlx;

//===================================================================================================
// main_hsr.cpp —— HC-SR04 超声波测距(软件侧优化版)
//
// 硬件接法不变: Trigger = PB9(推挽输出), Echo = PB1(EXTI 双边沿), TIM7 = 1MHz 自由计数器。
//
// 本版针对"读数乱跳/忽大忽小/偶尔离谱"做的软件改动:
//   1) 触发周期固定 >=60ms。数据手册要求两次测量之间留安静期; 上一版是"测到就立刻再发",
//      近处目标一发只有 1~2ms, 上一次的余振和多次反射还没停就发了下一发, 回波互相串扰。
//   2) 中断里加"武装(armed) + 一发只认一个回波(inPulse)"状态机:
//      armed   由主循环在触发前置位, 只有这一发的回波才计数;
//      inPulse 要求回波必须"先上升后下降"成对出现, 抑制重复上升沿与多次反射的第二个回波;
//      超时(没回波)时主循环清 armed, 迟到的回波不会算到下一发头上。
//   3) 有效窗口过滤: 回波宽度必须落在 [150us, 25000us](约 2.6cm ~ 4.3m) 才采纳;
//      过窄 = 模块余振/串扰, 过宽 = 噪声或量程外。
//   4) 等回波上限 40ms < TIM7 的 65.5ms 溢出周期, 避免"计数器已回绕"的短数被当成有效距离。
//   5) 不在中断里清零计数器, 改为记录上升/下降沿计数值做回绕安全差值, 顺带输出
//      "触发 -> 回波"延迟 dly, 用来判断是不是余振/串扰造成的假回波。
//   6) 声速按温度修正: c = 331.3 + 0.606*T (m/s), 距离 = 脉宽 * c / 2。
//   7) 中值滤波(窗口 5) + 跳变抑制: 单次离群值直接丢; 连续 3 次一致的大跳变说明
//      目标/环境真的变了, 清窗口重新跟, 避免滤波器"粘"在旧值上。
//   8) 每发打印一行诊断: 原始脉宽 / 回波延迟 / 原始距离 / 滤波距离 / 窗口离散度 /
//      丢失与剔除计数, 每秒再打一行汇总, 便于直接对比滤波前后效果。
//
// 还能更进一步(本版未做, 属于架构层面): 用 TIMx 的输入捕获(PWM 输入模式)由硬件
// 记录边沿时间戳, 可以把中断响应延迟完全从测量里去掉。
//===================================================================================================

// ---- 触发时序 ----
constexpr uint32_t kPingPeriodMs  = 60;    // 两次触发的最小间隔(数据手册要求 >=60ms)
constexpr uint32_t kTriggerHighUs = 12;    // Trig 高电平持续时间(数据手册要求 >=10us)
constexpr uint32_t kEchoTimeoutUs = 40000; // 等回波上限(约 6.9m); 必须 < 65535us(TIM7 16 位)
constexpr uint32_t kEchoPollUs    = 50;    // 等待轮询步长

// ---- 有效回波窗口 ----
constexpr uint32_t kEchoMinUs    = 150;   // 下限约 2.6cm, 更短判为余振/串扰
constexpr uint32_t kEchoMaxUs    = 25000; // 上限约 4.3m(HC-SR04 标称量程 4m)
constexpr uint32_t kEchoOffsetUs = 0;     // 固定偏置校准: 卡一个已知距离, 按 mm 的偏差再填这里

// ---- 声速(温度修正) ----
constexpr float kAmbientTempC = 25.0f;                                     // 环境温度(以后可接 BME280)
constexpr float kCmPerUs      = (331.3f + 0.606f * kAmbientTempC) * 1e-4f; // m/s -> cm/us

// ---- 滤波 ----
constexpr uint8_t  kMedianWindow = 5;   // 中值窗口: 越大越稳, 滞后越大
constexpr uint16_t kJumpGateMm   = 400; // 单次跳变超过 40cm 先当离群
constexpr uint8_t  kJumpAccept   = 3;   // 连续这么多次离群 => 认为目标真的移动/被遮挡

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

static inline uint16_t absDiffMm(uint16_t a, uint16_t b)
{
    return (a > b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

/**
 * @brief 中值滤波 + 跳变抑制(单位 mm)
 *
 * 只有"离群"样本会被丢掉, 正常抖动靠中值压平:
 *   - 单次大跳变(超过 kJumpGateMm)先不进窗口, 认为是超声波典型的毛刺/多径;
 *   - 连续 kJumpAccept 次都离群, 说明是真动了(或被遮挡后换了目标),
 *     清空窗口重新跟踪, 否则滤波器会"粘"在旧值上不动。
 */
class RobustMedianFilter
{
public:
    void push(uint16_t mm)
    {
        if (count_ == kMedianWindow && absDiffMm(mm, value_) > kJumpGateMm) {
            if (++outlierRun_ < kJumpAccept) {
                rejected_++;
                return; // 疑似离群: 不进窗口
            }
            windowReset(); // 连续离群: 按真实大位移处理
        } else {
            outlierRun_ = 0;
        }

        win_[head_] = mm;
        head_       = (uint8_t)((head_ + 1) % kMedianWindow);
        if (count_ < kMedianWindow) {
            count_++;
        }
        recompute();
    }

    uint16_t value() const { return value_; }       // 中值(滤波后距离)
    uint8_t  count() const { return count_; }       // 窗口内样本数(0~kMedianWindow)
    uint16_t spread() const { return spread_; }     // 窗口内 max-min, 反映当前抖动
    uint32_t rejected() const { return rejected_; } // 被当成离群丢掉的次数

private:
    uint16_t win_[kMedianWindow] = {0};
    uint8_t  count_      = 0; // 窗口内有效样本数
    uint8_t  head_       = 0; // 下一个写入位置
    uint8_t  outlierRun_ = 0; // 连续被判为离群的次数
    uint16_t value_      = 0; // 中值
    uint16_t spread_     = 0; // 窗口内极差
    uint32_t rejected_   = 0;

    void windowReset()
    {
        count_      = 0;
        head_       = 0;
        outlierRun_ = 0;
    }

    void recompute()
    {
        if (count_ == 0) {
            value_  = 0;
            spread_ = 0;
            return;
        }

        uint16_t t[kMedianWindow];
        for (uint8_t i = 0; i < count_; i++) {
            t[i] = win_[i];
        }
        for (uint8_t i = 1; i < count_; i++) { // 插入排序: 最多 5 个元素
            const uint16_t v = t[i];
            uint8_t j        = i;
            while (j > 0 && t[j - 1] > v) {
                t[j] = t[j - 1];
                j--;
            }
            t[j] = v;
        }

        value_  = (count_ & 1u) ? t[count_ / 2]
                                : (uint16_t)((t[count_ / 2 - 1] + t[count_ / 2]) / 2);
        spread_ = (count_ > 1) ? (uint16_t)(t[count_ - 1] - t[0]) : 0;
    }
};

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

    fixed_sprintf("Hello,DLX HSR04\n");

    protocol.SimpleLogW(gfsb);

    // ---- TIM7: 微秒自由计数器, 不用更新中断(避免高频中断拖垮 RT-Thread) ----
    // 84MHz / 84 = 1MHz, 即每格 1µs; ARR=0xFFFF 约 65.5ms 才溢出,
    // 足够覆盖 HC-SR04 量程(4m 回波约 23.3ms), 也大于本版的 40ms 等待上限。
    BasicTimer btimer(BasicTimerProfile::TIM7_Profile);
    btimer.init(83, 0xFFFF);

    // Echo 边沿中断的共享状态: 主循环写控制位, 中断只写数据位, 全部 volatile
    struct hsrCtx {
        BasicTimer *btimer = nullptr;
        volatile bool     armed   = false; // 主循环已触发, 允许这一发的回波计数
        volatile bool     inPulse = false; // 已捕获上升沿, 正在等下降沿
        volatile uint32_t riseUs  = 0;     // 上升沿时刻
        volatile uint32_t widthUs = 0;     // 最近一次完整回波宽度(µs)
        volatile uint32_t trigUs  = 0;     // 本次触发时刻(诊断用)
        volatile uint32_t seq     = 0;     // 完整回波计数, 主循环用它判断这一发有没有测到
    } ctx;
    ctx.btimer = &btimer;

    // Echo 边沿中断: 上升沿(回波开始)记时刻, 下降沿(回波结束)算宽度。
    // 注意: ISR 是裸中断(未包 rt_interrupt_enter/leave), 里面绝不能调 RT-Thread 内核 API
    // (如 rt_sem_release -> rt_schedule 会在中断里做上下文切换导致崩溃), 因此只读写 volatile 状态。
    auto hsr = EXTI_Line(GPIOProfile::B1);
    hsr.init(EXTIModeProfile::RisingFalling, NVICPriorityProfile::P2_S2, +[](void *tt) {
        auto ct            = static_cast<hsrCtx *>(tt);
        const uint32_t now = ct->btimer->getCounter();
        if (GPIO(GPIOProfile::B1).read()) {          // 高电平 = 上升沿
            if (!ct->armed || ct->inPulse) return;   // 未触发/重复上升沿: 丢弃(抑制串扰)
            ct->riseUs  = now;
            ct->inPulse = true;
        } else {                                     // 低电平 = 下降沿
            if (!ct->inPulse) return;                // 没有配对的上升沿: 丢弃
            ct->inPulse = false;
            ct->armed   = false;                     // 一发只认一个回波(抑制多次反射)
            const uint32_t width = (now - ct->riseUs) & 0xFFFFu; // 自由计数器, 差值天然回绕安全
            // 超过等待上限的回波直接作废: 否则它的"回绕后短数"可能被当成一个有效距离
            if (width > kEchoTimeoutUs) return;
            ct->widthUs = width;
            ct->seq++;
        }
    }, &ctx);

    auto trigger = GPIO(GPIOProfile::B9);
    trigger.init(GPIOModeProfile::OUT_PP_DOWN_50MHz);

    auto serialDMA = usart1.setDMASend(dmaBuffer);

    // 串口发一行(阻塞到本次发完), 顺序与工程里已验证的用法一致
    auto sendLog = [&]() {
        dmaBuffer.reset();
        protocol.SimpleLogW(gfsb);
        serialDMA.reset(dmaBuffer.used());
        serialDMA.start();
        serialDMA.wait();
    };

    RobustMedianFilter filter;
    // 统计量按"每秒"清零, 这样 1s 汇总行直接就是命中率与抖动范围
    uint32_t statPing     = 0;      // 本秒发数
    uint32_t statMiss     = 0;      // 本秒完全没等到回波
    uint32_t statBad      = 0;      // 本秒等到回波但宽度不在有效窗口
    uint32_t lastRej      = 0;      // 上一秒末的离群剔除累计值
    uint32_t lastPingTick = rt_tick_get();
    uint32_t lastStatTick = rt_tick_get();
    uint16_t statMinMm    = 0xFFFF; // 本秒最小/最大原始距离
    uint16_t statMaxMm    = 0;

    while (true) {
        // ---- 1. 固定触发周期: 给模块留出 >=60ms 的安静期 ----
        {
            const uint32_t sincePing = (uint32_t)(rt_tick_get() - lastPingTick);
            if (sincePing < kPingPeriodMs) {
                rt_thread_mdelay(kPingPeriodMs - sincePing); // 让出 CPU 给其它线程
            }
            lastPingTick = rt_tick_get();
        }

        // ---- 2. 武装本次接收并发出触发脉冲 ----
        ctx.inPulse = false;               // 清掉上一轮可能残留的半截脉冲
        ctx.armed   = true;                // 只接受这一发的回波
        ctx.trigUs  = btimer.getCounter(); // 触发时刻(算 dly 用)
        const uint32_t seq0 = ctx.seq;

        trigger = 1;                       // HC-SR04 要求 >=10µs 高电平
        delay_us(kTriggerHighUs);
        trigger = 0;

        // ---- 3. 有界等待回波(遮挡时也不会整轮卡死) ----
        uint32_t waitedUs = 0;
        while (ctx.seq == seq0 && waitedUs < kEchoTimeoutUs) {
            delay_us(kEchoPollUs);
            waitedUs += kEchoPollUs;
        }
        const bool gotEcho = (ctx.seq != seq0);
        if (!gotEcho) {
            ctx.armed   = false; // 超时: 取消接收资格, 迟到的回波不能算到下一发
            ctx.inPulse = false;
        }

        // ---- 4. 有效窗口校验 + 距离换算 ----
        statPing++;
        const uint32_t widthUs = gotEcho ? ctx.widthUs : 0;
        const uint32_t delayUs = gotEcho ? ((ctx.riseUs - ctx.trigUs) & 0xFFFFu) : 0;

        bool     valid = gotEcho && widthUs >= kEchoMinUs && widthUs <= kEchoMaxUs;
        uint16_t rawMm = 0;
        if (valid) {
            const uint32_t useUs = (widthUs > kEchoOffsetUs) ? (widthUs - kEchoOffsetUs) : 0u;
            rawMm                = (uint16_t)(useUs * (kCmPerUs * 5.0f) + 0.5f); // ×10(mm) / 2(往返)
            filter.push(rawMm);
            if (rawMm < statMinMm) statMinMm = rawMm;
            if (rawMm > statMaxMm) statMaxMm = rawMm;
        } else if (gotEcho) {
            statBad++;
        } else {
            statMiss++;
        }

        pf3 = valid; // 收到有效回波点亮, 便于无串口时观察命中率

        // ---- 5. 每发一行明细: 原始值 vs 滤波值 ----
        fixed_sprintf("us=%5u dly=%5u mm=%4u f=%4u win=%u sp=%3u\n",
                      (unsigned)widthUs, (unsigned)delayUs, (unsigned)rawMm,
                      (unsigned)filter.value(), (unsigned)filter.count(),
                      (unsigned)filter.spread());
        sendLog();

        // ---- 6. 每秒一行汇总: 命中率与抖动范围 ----
        if ((uint32_t)(rt_tick_get() - lastStatTick) >= 1000) {
            const uint32_t rej = filter.rejected() - lastRej;
            fixed_sprintf("1s ping=%u miss=%u bad=%u rej=%u min=%umm max=%umm\n",
                          (unsigned)statPing, (unsigned)statMiss, (unsigned)statBad,
                          (unsigned)rej,
                          (unsigned)(statMinMm == 0xFFFF ? 0 : statMinMm),
                          (unsigned)statMaxMm);
            sendLog();
            lastStatTick = rt_tick_get();
            statPing     = 0;
            statMiss     = 0;
            statBad      = 0;
            lastRej      = filter.rejected();
            statMinMm    = 0xFFFF;
            statMaxMm    = 0;
        }
    }
}
