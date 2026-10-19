#pragma once
#include "rtthread.h"
#include "app/runtime/release_header.h"
#include "app/control/app_state.h"
#include "app/channels/app_control_channel.hpp"
#include "app/channels/app_fault_channel.hpp"
#include "dlx_queue.hpp"
#include "W25Q128/dlx_flash_manager_config.h"

namespace dlx
{

constexpr uint16_t kMaxEntriesPerRun = 24;

// 一、控制线程 -> 日志线程 的日志队列(单生产者单消费者)
struct LogQueue
{
    static constexpr uint16_t kCapacity = (uint16_t)cfg::log::kQueueDepth; ///< 16 条 × 10ms = 160ms 缓冲
    LogEntry items[kCapacity]{};
    uint16_t head{0};            ///< 读位置(日志线程)
    uint16_t tail{0};            ///< 写位置(控制线程)
    uint16_t count{0};           ///< 当前条数
    uint16_t dropped{0};         ///< 本负载窗口内因为满而丢掉的条数(控制线程每个统计窗口清零)
    uint32_t droppedTotal{0};    ///< 开机至今累计丢掉的条数(只增; 调试模式状态行与告警用)
};

/** 控制线程投递日志；满时丢最老项并累计丢弃数。 */
inline void logQueuePush(LogQueue &q, const LogEntry &e)
{
    rt_enter_critical();
    if (q.count >= LogQueue::kCapacity) {
        q.head = static_cast<uint16_t>((q.head + 1u) % LogQueue::kCapacity);
        q.count = static_cast<uint16_t>(q.count - 1u);
        ++q.dropped;
        ++q.droppedTotal;
    }
    q.items[q.tail] = e;
    q.tail         = static_cast<uint16_t>((q.tail + 1u) % LogQueue::kCapacity);
    ++q.count;
    rt_exit_critical();
}

/** 出队(消费者: 日志线程); 空则返回 false */
inline bool logQueuePop(LogQueue &q, LogEntry &out)
{
    bool ok = false;
    rt_enter_critical();
    if (q.count > 0u) {
        out    = q.items[q.head];
        q.head = static_cast<uint16_t>((q.head + 1u) % LogQueue::kCapacity);
        --q.count;
        ok = true;
    }
    rt_exit_critical();
    return ok;
}

/** 没有日志出口时有界丢弃队列内容。 */
inline void logQueueDrain(LogQueue &q)
{
    LogEntry scratch;
    for (uint16_t count = 0u; count < kMaxEntriesPerRun && logQueuePop(q, scratch); ++count) {
    }
}

// 二、控制线程 -> 日志线程 的调试采样队列(单生产者单消费者)
/**
 * 与 LogQueue 同一套语义: 满时丢最老项并累计丢弃数, 生产者/消费者各只有一方。
 *
 * 容量按"串口调试 100Hz 采样 × 80ms"给: 日志线程被 DMA/一帧文本阻塞时, 仍能保留一小段
 * 连续曲线; 更久的积压直接丢最老样本(实时曲线要的是"最近发生了什么", 不是完整历史)。
 * ⚠ 队列元素是 48 字节 × 容量, 整体放在 FlightStack 里(主线程栈) —— 主线程栈预算本来就紧
 *   (见 docs/runtime_channels.md 的栈基线), 所以这里只给 8 条; 要放大先量栈水位。
 */
struct DebugQueue
{
    static constexpr uint16_t kCapacity = 8;
    DebugSample items[kCapacity]{};
    uint16_t    head{0};
    uint16_t    tail{0};
    uint16_t    count{0};
    uint16_t    dropped{0};      ///< 当前统计窗口内丢弃数(控制线程清零)
    uint32_t    droppedTotal{0}; ///< 开机至今累计丢弃数(只增)
};

/** 控制线程投递调试采样; 满时丢最老项并累计丢弃数。 */
inline void debugQueuePush(DebugQueue &q, const DebugSample &sample)
{
    rt_enter_critical();
    if (q.count >= DebugQueue::kCapacity) {
        q.head = static_cast<uint16_t>((q.head + 1u) % DebugQueue::kCapacity);
        q.count = static_cast<uint16_t>(q.count - 1u);
        ++q.dropped;
        ++q.droppedTotal;
    }
    q.items[q.tail] = sample;
    q.tail         = static_cast<uint16_t>((q.tail + 1u) % DebugQueue::kCapacity);
    ++q.count;
    rt_exit_critical();
}

/** 出队(消费者: 日志线程); 空则返回 false */
inline bool debugQueuePop(DebugQueue &q, DebugSample &out)
{
    bool ok = false;
    rt_enter_critical();
    if (q.count > 0u) {
        out    = q.items[q.head];
        q.head = static_cast<uint16_t>((q.head + 1u) % DebugQueue::kCapacity);
        --q.count;
        ok = true;
    }
    rt_exit_critical();
    return ok;
}

/** 没有调试出口时(飞行模式)有界丢空, 避免队列永久满。 */
inline void debugQueueDrain(DebugQueue &q)
{
    DebugSample scratch;
    for (uint16_t count = 0u; count < kMaxEntriesPerRun && debugQueuePop(q, scratch); ++count) {
    }
}

// 三、DWT 负载统计累加器(控制线程写; 日志线程只写 logSum/logCnt, 用临界区)
struct PerfAccum
{
    uint32_t loopSum{0}, loopMax{0};
    uint32_t ekfSum{0}, ekfCnt{0};
    uint32_t ctrlSum{0}, ctrlCnt{0};
    uint32_t ioSum{0}, ioCnt{0};
    uint32_t logSum{0}, logCnt{0};
    uint16_t samples{0};
};

/** 控制线程用: 独占这些累加器, 不进临界区 */
inline void perfAdd(uint32_t &sum, uint32_t &cnt, uint32_t cycles)
{
    sum += cycles;
    ++cnt;
}

inline void perfAddLocked(uint32_t &sum, uint32_t &count, uint32_t cycles)
{
    rt_enter_critical();
    perfAdd(sum, count, cycles);
    rt_exit_critical();
}

struct PublishedFlight
{
    app::Snapshot snapshot{};
    FlightTelemetry telemetry{};
};

struct LogStatus
{
    bool full{false};
    bool error{false};
    uint32_t writeFailures{0u};
};

struct FlightShared
{
    app::ControlChannels control{};
    app::FaultChannel faults{};
    LockedCell<PublishedFlight> published{};
    LockedCell<PerfStats> performance{};
    LockedCell<LogStatus> logStatus{};
    LockedCell<LinkStatistics> linkStatistics{};
    LogQueue             logQueue{};///< 控制线程 -> 日志线程
    DebugQueue           debugQueue{};///< 控制线程 -> 日志线程(串口调试模式的融合状况曲线)
    PerfAccum            perf{};    ///< 控制线程写, 日志线程读
    rt_thread_t          tidMain{RT_NULL}; ///< 主线程(控制环)句柄: 栈水位线用
    rt_thread_t          tidLink{RT_NULL}; ///< 链路线程(原 nrf_tel)
    rt_thread_t          tidLog{RT_NULL};  ///< 日志线程

    bool tryPostRequest(const app::Request &request) { return control.post(request); }
};

} // namespace dlx
