#pragma once
// 日志线程独占运行期串口协议、TX 缓冲和 DMA；各生产者只向信箱或日志队列投递。
#include "app/channels/app_flight_shared.hpp"
#include "app/runtime/app_stack.hpp"
#include "app/channels/app_log_channel.hpp"
#include "W25Q128/dlx_flash_manager.hpp"
#include "dlx_gpio.hpp"
#include "dlx_dma.hpp"

namespace dlx
{

// 日志线程局部参数使用 constexpr。
namespace logcfg
{
/** 一轮最多排空几条固定长日志 */
constexpr uint16_t MAX_ENTRIES_PER_RUN = 24;

/** 给"文本 + 状态行"留的保底空间: 必须 ≥ 一帧整池文本(4 + textmbox::POOL_BYTES = 516) */
constexpr uint16_t TEXT_RESERVE_BYTES = 600;

/** 一条固定长日志帧的字节数(负载 140 + 帧头 2) */
constexpr uint16_t FLIGHT_LOG_FRAME_BYTES = 142;

/** 一条调试采样帧的字节数(负载 48 + 帧头 2; 字段布局见 DebugSample / Protocol.kt:DebugSensor) */
constexpr uint16_t DEBUG_SAMPLE_FRAME_BYTES = 50;

/** 一轮最多发几条调试采样(按 100Hz 采样、5ms 一轮, 正常 1 条/轮; 这里给积压留上限) */
constexpr uint8_t MAX_DEBUG_PER_RUN = 16;

/** SimpleLog 帧头(2B header + 2B len) */
constexpr uint16_t SIMPLE_LOG_HEAD_BYTES = 4; // KSP 协议常量(dlx.hpp: SimpleLogW)

/** 一条异常帧的字节数(2B header + 2B code) */
constexpr uint16_t EXCEPTION_FRAME_BYTES = 4; // KSP 协议常量(dlx.hpp: DLXExceptionW)

/** 一轮最多发几条异常(剩余空间还允许更少时就按空间来) */
constexpr uint8_t MAX_EXCEPTIONS_PER_RUN = 8;

/** 心跳灯翻转周期 */
constexpr uint32_t HEARTBEAT_MS = 500;

/** 负载统计帧周期(串口模式) */
constexpr uint32_t PERF_REPORT_MS = 200;

/** 运行状态文本行周期(串口模式; 0 = 关) */
constexpr uint32_t DEBUG_STAT_MS = 1000;

/** 栈水位线周期(仅当下面那个开关打开时才有意义) */
constexpr uint32_t STACK_REPORT_MS = 10000;

/** 栈水位打印开关(量栈预算时打开) */
constexpr bool STACK_WATERMARK_ENABLED = false;
} // namespace logcfg

/** 文本预留空间必须容纳拥有者配置的整池文本及帧头。 */
static_assert(logcfg::TEXT_RESERVE_BYTES >= (logcfg::SIMPLE_LOG_HEAD_BYTES + textmbox::POOL_BYTES),
              "文本保底空间放不下一帧整池文本: 调大 logcfg::TEXT_RESERVE_BYTES");

struct LogThreadCtx
{
    FlightShared       *sh{nullptr};
    FlashManager       *fs{nullptr};        ///< 非空 = 写 flash(飞行模式)
    DMA                *serialDma{nullptr}; ///< 非空 = 发串口(串口调试模式)
    ByteBuffer         *txBuf{nullptr};
    DLX_ProtocolBuffer *protocol{nullptr};  ///< 负载统计报文走生成代码
    rt_sem_t            sem{RT_NULL};
    RingByteBuffer *rx{nullptr};
    LogStatus status{};
    bool                debugStream{false}; ///< true = 走串口
    uint32_t            seq{0};             ///< 调试流的条目序号(flash 模式下由文件系统填)
    uint32_t            lastPerfMs{0};      ///< 上次发负载统计的时间
    uint32_t            lastStackMs{0};     ///< 上次打印栈水位线的时间
    uint32_t            lastStatMs{0};      ///< 上次打印运行状态文本行的时间
};

/** 当前日志 owner 直接向本轮协议缓冲追加文本，不重置缓冲，也不经过生产者信箱。
 *  行尾整理(截断补换行 / UTF-8 边界 / 不补空格)统一走 release_header.h 的 sanitizeTextLine:
 *  现场那两个问题(长行被切断后与下一行粘连、行尾多出一个只有空格的"空行")在这里同样要防。 */
inline void appendTextLine(LogThreadCtx &ctx, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)))
{
    if (ctx.txBuf->remaining() <= logcfg::SIMPLE_LOG_HEAD_BYTES) {
        return;
    }
    char    text[textmbox::LINE_MAX_BYTES];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);
    const uint16_t len = sanitizeTextLine(text, n);
    if (len == 0u) {
        return;
    }
    ctx.protocol->SimpleLogW(reinterpret_cast<int8_t *>(text), len);
}

/** 没有对应出口时仍有界消费并丢弃信箱内容，避免信箱永久满。 */
inline void discardMailbox(LogChannel *channel)
{
    if (channel == nullptr) {
        return;
    }
    if (channel->exceptionMailbox != nullptr) {
        ExceptionCode scratch[logcfg::MAX_EXCEPTIONS_PER_RUN];
        channel->exceptionMailbox->take(scratch, logcfg::MAX_EXCEPTIONS_PER_RUN);
    }
    if (channel->textMailbox != nullptr) {
        channel->textMailbox->discardAll();
    }
}

inline void logThreadEntry(void *arg)
{
    LogThreadCtx *ctx = static_cast<LogThreadCtx *>(arg);
    FlightShared &sh  = *ctx->sh;
    LogEntry      entry;
    uint32_t      lastBlinkMs = rt_tick_get_millisecond();
    bool          ledOn       = false;
    GPIO          ledRun(cfg::hw::led::kRun);
    ledRun.init(GPIOModeProfile::OUT_PP_NOPULL_50MHz);
    paintStackWatermark(); // 记录本线程栈水位线(调试模式会打印)
    ctx->lastStackMs = rt_tick_get_millisecond();
    ctx->lastStatMs  = ctx->lastStackMs;

    while (true) {
        rt_sem_take(ctx->sem, RT_WAITING_FOREVER);

        const uint32_t nowMs = rt_tick_get_millisecond();

        // 渠道指针每轮重读一次: owner 可能在别处换人(信箱由外部拥有, 可以转移)
        ReleaseContext *appCtx  = logContext;
        LogChannel     *channel = (appCtx != nullptr) ? appCtx->channel : nullptr;

        // ---- 心跳灯: 500ms 翻转一次, 说明系统在跑(与有没有日志出口无关) ----
        if ((nowMs - lastBlinkMs) >= logcfg::HEARTBEAT_MS) {
            lastBlinkMs = nowMs;
            ledOn       = !ledOn;
            ledRun      = ledOn ? 1 : 0;
        }

        if (ctx->debugStream && ctx->rx != nullptr) {
            uint32_t consumed = 0u;
            while (ctx->rx->available() >= 2u && consumed < app::SerialRxBytesPerRun) {
                const uint16_t before = ctx->rx->available();
                if (ctx->protocol->check() == DLXCheckResult::FAILED_INSUFFICIENT) break;
                const uint16_t after = ctx->rx->available();
                consumed += before > after ? before - after : 1u;
            }
        }
        if (ctx->fs != nullptr) {
            // ---------------- 飞行模式: 固定长报文写 flash ----------------
            uint16_t n = 0;
            while (n < logcfg::MAX_ENTRIES_PER_RUN && logQueuePop(sh.logQueue, entry)) {
                ExceptionCode        error       = ExceptionCode::EXCEPTION_NONE;
                const uint32_t       startCycles = DWT->CYCCNT;
                const FunctionResult result      = ctx->fs->appendLog(entry, error);
                perfAddLocked(sh.perf.logSum, sh.perf.logCnt, DWT->CYCCNT - startCycles);
                if (result == FunctionResult::FAILED) {
                    const bool wasError = ctx->status.error;
                    ctx->status.full = error == ExceptionCode::FLASH_LOG_FULL;
                    ctx->status.error = !ctx->status.full;
                    ++ctx->status.writeFailures;
                    if (ctx->status.error && !wasError) DLX_HANDLE_EXCEPTION(error);
                } else {
                    ctx->status.error = false;
                    ctx->status.full = ctx->fs->isLogFull();
                }
                ++n;
            }
            // 飞行模式没有串口出口: 两条渠道都必须取走, 否则会永久满
            discardMailbox(channel);
            debugQueueDrain(sh.debugQueue);
        } else if (ctx->debugStream) {
            // ---------------- 串口调试模式: 唯一的 protocol owner ----------------
            if (!ctx->serialDma->isComplete()) continue;
            ctx->txBuf->reset();

            // ---- 1. 固定长帧优先: 飞行数据不等人 ----
            // 留 TEXT_RESERVE_BYTES 保底: 队列积压时不能把文本与状态行一起饿死。
            uint16_t count = 0;
            while (count < logcfg::MAX_ENTRIES_PER_RUN &&
                   ctx->txBuf->remaining() >=
                       (logcfg::FLIGHT_LOG_FRAME_BYTES + logcfg::TEXT_RESERVE_BYTES) &&
                   logQueuePop(sh.logQueue, entry)) {
                entry.sessionId   = 0u;         // 不写 flash, 没有会话号
                entry.seq         = ++ctx->seq; // 用自增序号代替
                const uint32_t t0 = DWT->CYCCNT;
                writeLogEntry(*ctx->protocol, entry);
                perfAddLocked(sh.perf.logSum, sh.perf.logCnt, DWT->CYCCNT - t0);
                ++count;
            }

            // ---- 2. 调试采样: 原始观测 + 同刻融合结果(仅串口调试模式) ----
            // 与固定长帧同一优先级; 同样给文本与状态行留出保底空间, 且每轮有上限,
            // 避免积压时把一整轮 TX 缓冲塞满、把 DMA 在飞时间拖长。
            {
                uint8_t     debugSent = 0u;
                DebugSample debugSample;
                while (debugSent < logcfg::MAX_DEBUG_PER_RUN &&
                       ctx->txBuf->remaining() >=
                           (logcfg::DEBUG_SAMPLE_FRAME_BYTES + logcfg::TEXT_RESERVE_BYTES) &&
                       debugQueuePop(sh.debugQueue, debugSample)) {
                    writeDebugSample(*ctx->protocol, debugSample);
                    ++debugSent;
                }
            }

            appendLogMailbox(channel, *ctx->protocol, *ctx->txBuf);

            // ---- 4. 周期帧: 负载统计 ----
            if ((nowMs - ctx->lastPerfMs) >= logcfg::PERF_REPORT_MS) {
                ctx->lastPerfMs = nowMs;
                PerfStats perf;
                rt_enter_critical();
                perf = sh.performance.load();
                rt_exit_critical();
                // 走 KSP 生成的报文(追加在日志流后面)
                ctx->protocol->PerfMonitorW(perf.loopAvgCycle, perf.loopMaxCycle, perf.ekfCycle,
                                            perf.ctrlCycle, perf.logCycle, perf.ioCycle, perf.samples,
                                            perf.drops);
            }

            // ---- 5. 运行状态文本行(默认 1s 一条; 给"拿串口终端盯台架"的人看) ----
            uint32_t queueCount = 0, dropsTotal = 0, bytesIn = 0, rxFrames = 0, rejects = 0;
            uint32_t applied = 0, txFrames = 0, txFail = 0;
            uint32_t debugQueueCount = 0, debugQueueDrops = 0;
            uint32_t lastValid = 0;
            uint16_t cmdNow    = 9;
            uint8_t  telStatus = 0;
            bool     motorEn   = false;
            PerfStats perfSnap = {};
            uint32_t admitRej  = 0;
            uint32_t txSkipGps = 0;
            uint8_t  mutedNow = 0, goodStreak = 0, windowArmed = 0, ulLossPct = 0; // 时分相位(见 struct)
            uint32_t txWinBytes = 0, pos0 = 0, pos1 = 0, pos2 = 0, pos3 = 0;
            const LinkStatistics statistics = sh.linkStatistics.load();
            const PublishedFlight published = sh.published.load();
            const app::ControlInput input = sh.control.input.load();
            rt_enter_critical();
            queueCount = sh.logQueue.count;
            dropsTotal = sh.logQueue.droppedTotal;
            debugQueueCount = sh.debugQueue.count;
            debugQueueDrops = sh.debugQueue.droppedTotal;
            bytesIn    = statistics.linkBytesIn;
            rxFrames   = statistics.linkRxFrames;
            rejects    = statistics.linkRxRejects;
            admitRej   = statistics.linkAdmitRejects;
            applied    = statistics.linkCmdsApplied;
            txFrames   = statistics.linkTxFrames;
            txFail     = statistics.linkTxFail;
            txSkipGps  = statistics.linkTxSkipGps;
            mutedNow    = statistics.linkMuted;
            goodStreak  = statistics.linkGoodStreak;
            windowArmed = statistics.linkWindowArmed;
            ulLossPct   = statistics.linkUlLossPct;
            txWinBytes  = statistics.linkTxWinBytes;
            pos0        = statistics.linkRxPos[0];
            pos1        = statistics.linkRxPos[1];
            pos2        = statistics.linkRxPos[2];
            pos3        = statistics.linkRxPos[3];
            lastValid  = input.receivedMs;
            cmdNow     = input.command;
            telStatus  = published.telemetry.status;
            motorEn    = published.telemetry.enabled;
            perfSnap   = sh.performance.load();
            rt_exit_critical();

            // 信箱的丢弃量(32 位对齐的字读取不会撕裂, 不必进临界区)
            uint32_t textDroppedLines = 0, textDroppedBytes = 0, exceptionDropped = 0;
            if (channel != nullptr) {
                if (channel->textMailbox != nullptr) {
                    textDroppedLines = channel->textMailbox->droppedLines();
                    textDroppedBytes = channel->textMailbox->droppedBytes();
                }
                if (channel->exceptionMailbox != nullptr) {
                    exceptionDropped = channel->exceptionMailbox->droppedCount();
                }
            }

            if (logcfg::DEBUG_STAT_MS != 0u && (nowMs - ctx->lastStatMs) >= logcfg::DEBUG_STAT_MS) {
                ctx->lastStatMs      = nowMs;
                const bool linkOk    = ((nowMs - lastValid) <= cfg::link::failsafe::kLinkTimeoutMs);
                rt_size_t  heapTotal = 0, heapUsed = 0, heapMax = 0;
                rt_memory_info(&heapTotal, &heapUsed, &heapMax);
                /** 分别报告飞行日志、文本和异常的丢弃计数。 */
                appendTextLine(*ctx,
                               "[Stat] 运行=%us 日志序号=%u 队列=%u/%u 队列丢=%u 文本丢=%u行/%uB 异常丢=%u "
                               "堆空闲=%u/%uB 峰值=%uB\n",
                               static_cast<unsigned>(nowMs / 1000u), static_cast<unsigned>(ctx->seq),
                               static_cast<unsigned>(queueCount),
                               static_cast<unsigned>(LogQueue::kCapacity),
                               static_cast<unsigned>(dropsTotal),
                               static_cast<unsigned>(textDroppedLines),
                               static_cast<unsigned>(textDroppedBytes),
                               static_cast<unsigned>(exceptionDropped),
                               static_cast<unsigned>(heapTotal - heapUsed),
                               static_cast<unsigned>(heapTotal), static_cast<unsigned>(heapMax));
                // 分别报告接收、采信、执行和时分状态计数，便于定位链路故障。
                // 拆成两条: 合成一条在 UTF-8 下约 186 字节, 会被 LINE_MAX_BYTES(160) 截断
                // (现场 [Link] 行就断在"相位=跟随中(连"中间, 后半段丢失).
                appendTextLine(*ctx,
                               "[Link] 链路=%s(距上次指令 %ums) 指令=%u 收字节=%u 收帧=%u 拒=%u 采信拒=%u\n",
                               linkOk ? "在线" : "超时", static_cast<unsigned>(nowMs - lastValid),
                               static_cast<unsigned>(cmdNow), static_cast<unsigned>(bytesIn),
                               static_cast<unsigned>(rxFrames), static_cast<unsigned>(rejects),
                               static_cast<unsigned>(admitRej));
                appendTextLine(*ctx,
                               "[Link] 采纳=%u 发帧=%u 发失败=%u 跳过GPS=%u 相位=%s(连续%u 上行丢%u%%) 电机=%s\n",
                               static_cast<unsigned>(applied),
                               static_cast<unsigned>(txFrames), static_cast<unsigned>(txFail),
                               static_cast<unsigned>(txSkipGps),
                               mutedNow ? "静默(只收不发)" : (windowArmed ? "窗口开" : "跟随中"),
                               static_cast<unsigned>(goodStreak), static_cast<unsigned>(ulLossPct),
                               motorEn ? "已使能"
                                       : ((telStatus & kTelStatusMotorStarting) ? "启动中" : "停"));
                // 短诊断行: 上行丢帧位置 + 发送窗口内被丢弃的 RX 字节(上位机把 [Link] 截断在 80 字符)
                appendTextLine(*ctx, "[Lk2] U0..3=%u/%u/%u/%u 上行丢=%u%% tx窗RX=%uB\n",
                               static_cast<unsigned>(pos0), static_cast<unsigned>(pos1),
                               static_cast<unsigned>(pos2), static_cast<unsigned>(pos3),
                               static_cast<unsigned>(ulLossPct), static_cast<unsigned>(txWinBytes));
                const uint32_t usPerCycle = (SystemCoreClock / 1000000u);
                appendTextLine(*ctx,
                               "[Perf] 主循环 平均=%uus 最大=%uus(%u圈) EKF=%uus 控制=%uus IO=%uus 日志=%uus\n",
                               static_cast<unsigned>(perfSnap.loopAvgCycle / usPerCycle),
                               static_cast<unsigned>(perfSnap.loopMaxCycle / usPerCycle),
                               static_cast<unsigned>(perfSnap.samples),
                               static_cast<unsigned>(perfSnap.ekfCycle / usPerCycle),
                               static_cast<unsigned>(perfSnap.ctrlCycle / usPerCycle),
                               static_cast<unsigned>(perfSnap.ioCycle / usPerCycle),
                               static_cast<unsigned>(perfSnap.logCycle / usPerCycle));
                // 融合状况调试流(原始观测 + EKF): 队列积压/丢弃说明串口侧跟不上 100Hz 采样。
                appendTextLine(*ctx, "[Dbg ] 调试采样 队列=%u/%u 丢=%u\n",
                               static_cast<unsigned>(debugQueueCount),
                               static_cast<unsigned>(DebugQueue::kCapacity),
                               static_cast<unsigned>(debugQueueDrops));
            }

            // 每 10s 打一行栈水位线: 默认关, 量栈预算时把 logcfg::STACK_WATERMARK_ENABLED 打开
            if (logcfg::STACK_WATERMARK_ENABLED && (nowMs - ctx->lastStackMs) >= logcfg::STACK_REPORT_MS) {
                ctx->lastStackMs = nowMs;
                appendTextLine(*ctx, "stack used: main=%u/%u link=%u/%u logger=%u/%u\n",
                               static_cast<unsigned>(stackUsedBytes(sh.tidMain)),
                               static_cast<unsigned>(sh.tidMain ? sh.tidMain->stack_size : 0u),
                               static_cast<unsigned>(stackUsedBytes(sh.tidLink)),
                               static_cast<unsigned>(sh.tidLink ? sh.tidLink->stack_size : 0u),
                               static_cast<unsigned>(stackUsedBytes(sh.tidLog)),
                               static_cast<unsigned>(sh.tidLog ? sh.tidLog->stack_size : 0u));
            }

            // ---- 6. 提交: 有内容才启动 DMA(顺序不能换: reset 清 TC, 必须紧跟 start) ----
            if (ctx->txBuf->used() > 0u) {
                ctx->serialDma->reset(ctx->txBuf->used());
                ctx->serialDma->start();
            }
        } else {
            // 没有日志出口(flash 坏了): 只丢队列, 别卡住控制线程
            logQueueDrain(sh.logQueue);
            discardMailbox(channel);
            debugQueueDrain(sh.debugQueue);
            ctx->status.error = true;
        }
        sh.logStatus.store(ctx->status);
    }
}

} // namespace dlx
