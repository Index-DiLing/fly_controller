#pragma once
//===================================================================================================
// app/link/app_link_thread.hpp —— 链路 v2 飞控端线程(USART DMA+IDLE 收 / RoundSlave 调度 / DMA 发)
//
// 架构(依据 docs/链路层v2_协议规格.md §2/§4/§6/§8):
//   ┌ USART3 RX ─ DMA 循环缓冲 ─ IDLE 中断(帧末事件 + DWT 周期戳) ─┐
//   │        ↓ ISR 只把"这一段字节 + 时间戳"塞进无锁突发队列, 并 release 信号量
//   │   链路线程(6ms 定时唤醒 + IDLE 立即唤醒):
//   │     ① 收: 突发队列 → 组装缓冲 → tryDecode(自定界/CRC-16) → L0/L1/L2/L3 分层准入
//   │     ② 分发: ControlInput(LockedCell) / 命令请求(MPSC) / 回执队列
//   │     ③ 发: RoundSlave.decide() 到点 → 组帧 → E220.sendDma(一帧一次连续写)
//   └ ④ 发布统计到 LinkStatistics(LockedCell, 读者=日志线程)
//
// 三条"不能错"的时序约束:
//   1. **D0 触发只由"收到 pos3"或"t0 推算的 deadline"驱动**; 6ms 定时器只做 watchdog/统计 ——
//      绝不能让定时器自己当发送源(那会把 D0 抖动放大到 6ms, 而预算只有 ~9ms)。
//   2. IDLE 是帧末的硬件证据; E220 整帧收完才开始吐 UART, 所以 IDLE 时刻要按帧长减掉
//      (n + LinkRxIdleBytes)×174us 才是"空中结束" —— 修正量由 RoundParams.rxIdleBytes 注入
//      (P0 实测 1, 台架复核后只改一个数; 主机测试可注入不同值)。
//   3. 本轮关闭后(D0 已发/已跳过), 迟到帧只更新计数, 不得再触发第二帧 D0(规格 §4)。
//
// 与 v1 的区别(为什么整文件重写而不是打补丁):
//   - v1 的"绝对时间槽网格/锚点"换成"轮次位置 = SEQ&3"; 相位不再需要被跟踪, 只剩 t0 估计;
//   - 收路径从 RXNE 逐字节环形缓冲换成 DMA+IDLE(帧末事件是 D0 精度的前提);
//   - 发路径从阻塞 TXE 轮询换成 DMA(线程被抢占 >0.5ms 会把一帧劈成两个射频包);
//   - 下行从取模轮转换成"优先级 + 到期"选帧(见 dlx_link_downlink.hpp)。
//===================================================================================================
#include <stdint.h>
#include <string.h>

#include "app/channels/app_flight_shared.hpp"
#include "app/control/app_state.h"
#include "app/link/app_link_cmd.hpp"
#include "app/link/app_link_state.hpp"
#include "app/link/link_v2_config.hpp"
#include "app/runtime/app_exception_handler.h"
#include "app/runtime/app_stack.hpp"
#include "app/sensors/app_health.hpp"
#include "dlx_delay.hpp"
#include "dlx_interrupt_lock.hpp"
#include "dlx_locked_cell.hpp"
#include "dlx_timebase.hpp"
#include "E220/dlx_e220.hpp"
#include "link/dlx_link_downlink.hpp"
#include "link/dlx_link_frame.hpp"
#include "link/dlx_link_round.hpp"

namespace dlx
{

//===================================================================================================
// 一、接收突发队列(ISR 生产者 / 链路线程消费者, 无锁 SPSC)
//===================================================================================================
/** IDLE 切出来的一段字节(透传下一个射频包 = 一段 = 一帧; 仍按"一段字节"存, 由上层自定界)。 */
struct LinkRxBurst
{
    uint32_t cycles; ///< IDLE 回调入口的 DWT->CYCCNT(未修正; 修正量在 RoundSlave 里减)
    uint16_t len;
    uint8_t  data[E220::kTxDmaCapacity]; ///< 与 E220 单帧上限一致(32B): 一段不会超过一个分包
};

/** ISR → 线程的突发队列。
 *  为什么无锁: ISR 里只能做"拷一段 + release 信号量"; 单生产者(只有 USART3 IDLE)+单消费者
 *  (链路线程), head/tail 是 8 位对齐读写(M4 上原子), 不需要关中断。 */
class LinkRxBurstQueue
{
public:
    static constexpr uint8_t kCapacity = 8u;

    /** ISR 侧: 满了丢**新**突发并计数(丢老的会让 head/tail 回绕关系变复杂; 8 段 ≈ 240ms 流量) */
    bool pushFromIsr(const uint8_t *data, uint16_t len, uint32_t cycles)
    {
        if (data == nullptr || len == 0u) {
            return false;
        }
        if (len > (uint16_t)sizeof(LinkRxBurst::data)) {
            ++m_overlong;
            return false;
        }
        const uint8_t next = static_cast<uint8_t>((m_tail + 1u) % kCapacity);
        if (next == m_head) {
            ++m_dropped;
            return false;
        }
        LinkRxBurst &b = m_items[m_tail];
        b.cycles       = cycles;
        b.len          = len;
        memcpy(b.data, data, len);
        m_tail = next;
        return true;
    }

    /** 线程侧 */
    bool pop(LinkRxBurst &out)
    {
        if (m_head == m_tail) {
            return false;
        }
        out    = m_items[m_head];
        m_head = static_cast<uint8_t>((m_head + 1u) % kCapacity);
        return true;
    }

    uint32_t droppedCount() const { return m_dropped; }
    uint32_t overlongCount() const { return m_overlong; }

private:
    LinkRxBurst       m_items[kCapacity];
    volatile uint8_t  m_head = 0u;
    volatile uint8_t  m_tail = 0u;
    volatile uint32_t m_dropped  = 0u;
    volatile uint32_t m_overlong = 0u;
};

//===================================================================================================
// 二、下行调度策略常量(规格 §6: STATE 每 4 轮 / SYS 每 4 轮(偏移 2) / GPS 每 16 轮)
//===================================================================================================
constexpr uint8_t kDlStateEvery = 4u;
constexpr uint8_t kDlSysEvery   = 4u;
constexpr uint8_t kDlSysOffset  = 2u;
constexpr uint8_t kDlGpsEvery   = 16u;

/** "本轮关闭后, 反推 t0 差多少才算进了新轮"(规格 §4 的 60ms)。只影响飞控从端的轮次判定,
 *  不是两端共享常量, 所以不进 link_v2_config.hpp。 */
constexpr uint32_t kRoundReopenUs = 60000u;

/** 接收链路拒绝原因(规格 §8.4 "最后拒绝原因")。 */
enum class LinkReject : uint8_t
{
    None = 0,
    TypeNotUplink, ///< 收到下行型/非法型(角色配错或噪声)
    AcquireOnly,   ///< ACQUIRE 里收到 0x8–0xD(只认 UP_ROUND)
    BadSession,    ///< UP_ROUND.session==0(保留值)
    NoRound,       ///< 轮次层没采信(时间基异常等)
    Build,         ///< D0 选到的帧本轮组不出来(如 GPS 无数据) —— 放弃而不是发空帧
    Count,
};

inline const char *linkRejectName(LinkReject r)
{
    switch (r) {
    case LinkReject::None:          return "NONE";
    case LinkReject::TypeNotUplink: return "TYPE";
    case LinkReject::AcquireOnly:   return "ACQUIRE";
    case LinkReject::BadSession:    return "SESSION";
    case LinkReject::NoRound:       return "NOROUND";
    case LinkReject::Build:         return "BUILD";
    default:                        return "?";
    }
}

//===================================================================================================
// 三、线程上下文
//===================================================================================================
struct LinkThreadCtx
{
    FlightShared *sh{nullptr};
    Health        health{};
    app::ControlInput controlInput{};
    app::EmergencyStopHandshake emergencyStop{};

    LinkStatistics  statistics{};
    E220           *radio{nullptr};
    USARTDmaReceiver *rx{nullptr};
    rt_sem_t        sem{RT_NULL};

    // ---- v2 协议状态 ----
    link::RoundSlave       round{}; ///< 轮次从机(ACQUIRE/RUN/t0/D0 时机)
    link::DownlinkSelector dl{};    ///< 下行选帧(优先/到期)
    LinkRxBurstQueue       rxq{};   ///< ISR → 线程

    // ---- 字节流组装(一段突发可能粘连/裂开; 用"最后一段"的时间戳当帧末) ----
    uint8_t  asmBuf[64]{};
    uint16_t asmL{0};
    uint32_t asmEndUs{0};

    // ---- 命令/回执 ----
    app::CommandHistory cmdHist{};
    uint8_t  downlinkSeq{0}; ///< 下行所有帧共用, 模 16(含 HELLO 应答; 规格 §1.3)
    uint8_t  requestsInFlight{0};
    uint8_t  lastCmdId{0};
    uint8_t  lastCmdResult{0};

    // ---- 准入/统计(规格 §8.4) ----
    LinkReject lastReject{LinkReject::None};
    uint32_t   rejectsBy[(uint8_t)LinkReject::Count]{};
    uint32_t   bytesIn{0};
    uint32_t   rxFrames{0};      ///< L0 通过(CRC/类型/长度)的上行帧
    uint32_t   decodeRejects{0}; ///< tryDecode 的坏帧处数(每处只前进 1 字节)
    uint32_t   admitRejects{0};  ///< L1/L2/L3 拒收帧数
    uint32_t   applied{0};       ///< 真正写进 ControlInput/动作的帧数
    uint32_t   cmdsPosted{0};
    uint32_t   txFrames{0};
    uint32_t   txFail{0};    ///< 本轮没发出去(含 AUX 忙/预算不足/sendDma 失败)
    uint32_t   skipAux{0};
    uint32_t   skipBudget{0};
    uint32_t   skipUnavailable{0};
    uint32_t   helloRequests{0};
    uint32_t   helloVersionMismatch{0};
    uint32_t   sessionChanges{0};
    uint32_t   estopLatches{0};
    uint32_t   motorClamped{0};
    uint32_t   lateFrames{0};
    uint32_t   ulRounds{0}; ///< 有活动的轮次数(上行丢包窗口)
    uint32_t   ulFramesExpected{0};
    uint32_t   ulFramesGot{0};
    uint32_t   dlRoundsClosed{0};
    uint32_t   dlRoundsSent{0};
    uint32_t   lastRxUs{0};
    uint8_t    peerVersion{0};
    bool       peerVersionSeen{false};
    bool       rcWantsGps{false}; ///< 对端 HELLO.caps.bit0(想不想要 DL_GPS)

    // ---- D0 发送窗口内的 RX 抑制(半双工换向/串扰瞬变, 见 docs/链路层v2_已知问题_20261018.md) ----
    uint32_t txWinStartCycles{0};
    uint32_t txWinEndCycles{0};   ///< 0 = 当前不在发送窗口
    uint32_t txWinBursts{0};
    uint32_t txWinBytes{0};
};

//===================================================================================================
// 四、时基工具
//===================================================================================================
/** 单调微秒时基(DWT->CYCCNT 展开)。只在链路线程里调用(ISR 只记原始周期戳)。 */
inline uint32_t linkMicros()
{
    // 函数内静态: POD 零初始化 ⇒ 常量初始化, 不需要运行期守卫(不会在 RTOS 切换点上抢锁)
    static UsExpander s_exp{};
    return usExpanderStep(s_exp, DWT->CYCCNT, static_cast<uint32_t>(SystemCoreClock / 1000000u));
}

/** 把 ISR 记下的"当时周期数"换算成微秒时刻: end = now - (now_cycles - then_cycles)/cyclesPerUs。
 *  为什么不直接在 ISR 里调 linkMicros(): 那个 UsExpander 的状态不是可重入的; 而周期差在
 *  "回绕周期 25.57s 内"永远正确, 且链路线程每 ≤6ms 就推进一次, 远小于回绕周期。 */
inline uint32_t linkCyclesToUs(uint32_t thenCycles, uint32_t nowUs, uint32_t nowCycles)
{
    const uint32_t cyclesPerUs = static_cast<uint32_t>(SystemCoreClock / 1000000u);
    if (cyclesPerUs == 0u) {
        return nowUs;
    }
    const uint32_t deltaCycles = static_cast<uint32_t>(nowCycles - thenCycles);
    const uint32_t ageUs       = deltaCycles / cyclesPerUs;
    return static_cast<uint32_t>(nowUs - ageUs);
}

/** 这个 RX 突发的到达时刻是否落在"本端正在发 D0"的窗口内。
 *  半双工协议下这段时间对端不可能发合法数据, 所以窗口内的字节只可能是换向/串扰瞬变;
 *  把它们丢弃并不进解析器, 才能让 `拒` 反映真实问题(见 2026-10-18 已知问题文档)。 */
inline bool linkCyclesInTxWindow(const LinkThreadCtx *ctx, uint32_t cycles)
{
    if (ctx->txWinEndCycles == 0u) {
        return false;
    }
    return (int32_t)(cycles - ctx->txWinStartCycles) >= 0 && (int32_t)(cycles - ctx->txWinEndCycles) < 0;
}

/** USART3 IDLE 回调(ISR): 只记"这段字节 + 回调入口的周期数", 并唤醒链路线程。
 *  时间戳换算在 RoundSlave 里做(按帧长减 (n+rxIdleBytes)×174us) —— 见 RoundParams.rxIdleBytes。 */
inline void linkRxBurstIsr(const uint8_t *data, uint16_t len, void *ctx)
{
    LinkThreadCtx *c = static_cast<LinkThreadCtx *>(ctx);
    if (c == nullptr) {
        return;
    }
    c->rxq.pushFromIsr(data, len, DWT->CYCCNT);
    if (c->sem != RT_NULL) {
        rt_sem_release(c->sem); // 正常路径靠这个立即唤醒(6ms 定时器只做 watchdog)
    }
}

//===================================================================================================
// 五、统计/工具
//===================================================================================================
inline uint8_t linkPopCount4(uint8_t v)
{
    v = static_cast<uint8_t>(v & 0x0Fu);
    uint8_t n = 0u;
    for (uint8_t i = 0u; i < 4u; ++i) {
        n = static_cast<uint8_t>(n + static_cast<uint8_t>((v >> i) & 1u));
    }
    return n;
}

inline uint8_t linkUlLossPct(const LinkThreadCtx *ctx)
{
    if (ctx->ulFramesExpected == 0u) {
        return 0u;
    }
    const uint32_t lost = ctx->ulFramesExpected - ctx->ulFramesGot;
    const uint32_t pct  = (lost * 100u) / ctx->ulFramesExpected;
    return static_cast<uint8_t>(pct > 100u ? 100u : pct);
}

inline uint8_t linkDlLossPct(const LinkThreadCtx *ctx)
{
    if (ctx->dlRoundsClosed == 0u) {
        return 0u;
    }
    const uint32_t lost = ctx->dlRoundsClosed - ctx->dlRoundsSent;
    const uint32_t pct  = (lost * 100u) / ctx->dlRoundsClosed;
    return static_cast<uint8_t>(pct > 100u ? 100u : pct);
}

/** 一轮关闭时的上行位图统计(D0 发出/放弃后调用; 迟到帧不算"本轮收到" —— 它是 D0 之后才到的) */
inline void linkCloseRoundStats(LinkThreadCtx *ctx, bool sent)
{
    const uint8_t got = linkPopCount4(ctx->round.rxBitmap());
    ++ctx->ulRounds;
    ctx->ulFramesExpected = static_cast<uint32_t>(ctx->ulFramesExpected + 4u);
    ctx->ulFramesGot      = static_cast<uint32_t>(ctx->ulFramesGot + got);
    ++ctx->dlRoundsClosed;
    if (sent) {
        ++ctx->dlRoundsSent;
    }
}

//===================================================================================================
// 六、下行数据源
//===================================================================================================
/** DL_STATE 的 failsafe 两字段(链路 v2 §8.3): 由控制线程发布的**真实**状态派生, 不再恒 0。
 *
 *  映射依据(release_header.h 的 kTelStatus*):
 *    kTelStatusHardStop(硬停机锁存)     → lvl=4(CUT)
 *    kTelStatusFailsafe(失控保护已锁存)  → lvl=3(HOVER_LAND)
 *    都未置位                           → lvl=0(NONE)
 *  为什么只给 0/3/4: 飞控这边实际只有"无 / 缓降 / 停机"三种真实状态(v1 的 WARN/DEGRADE 阶梯
 *  只存在于配置里, 没有消费者); 填一个自己都没实现的等级, 等于在显示端说"看起来有, 其实没有"。
 *  rsn 恒 0: 原因码表由遥控器定义, 飞控当前只能分辨层级。
 */
struct LinkWireFailsafe
{
    uint8_t lvl{0};
    uint8_t rsn{0};
};

inline LinkWireFailsafe linkDeriveFailsafe(uint8_t telStatus)
{
    LinkWireFailsafe w;
    if ((telStatus & kTelStatusHardStop) != 0u) {
        w.lvl = 4u;
    } else if ((telStatus & kTelStatusFailsafe) != 0u) {
        w.lvl = 3u;
    }
    return w;
}

inline void linkFillDownlinkSource(LinkThreadCtx *ctx, link::DownlinkSource &s)
{
    const PublishedFlight  published = ctx->sh->published.load();
    const FlightTelemetry &tm        = published.telemetry;
    const app::Snapshot   &snap      = published.snapshot;
    const app::WireState   wire      = app::deriveWire(snap);
    const LinkWireFailsafe fs        = linkDeriveFailsafe(tm.status);

    s.attitude    = tm.quat;
    s.velWorldMps = tm.velWorldMps;
    s.status      = tm.status;
    s.error       = tm.error;
    s.arming      = wire.arming;
    s.mode        = wire.mode;
    s.posM        = tm.posWorldM;
    s.currentA    = tm.batteryA;
    s.voltageV    = tm.batteryV;
    for (uint8_t i = 0u; i < link::dlSys::kMotorCount; ++i) {
        s.motor[i] = tm.motor[i];
    }
    s.lifecycle   = wire.lifecycle;
    s.task        = wire.task;
    s.failsafeLvl = fs.lvl; // 真实值(不再恒 0): 0=NONE / 3=HOVER_LAND / 4=CUT
    s.failsafeRsn = fs.rsn;
    s.health      = snap.health;
    s.ulLoss      = linkUlLossPct(ctx);
    s.dlLoss      = linkDlLossPct(ctx);
    s.rssiDbm     = 0;                     // E220 未开 DRSSI: 如实填 0, 不假装有值
    s.rxBitmap    = ctx->round.rxBitmap(); // 本轮 U0..U3 接收位图(规格 §3.5)

    // DL_GPS: 本轮未接线 —— GNSS 观测还没有发布到 FlightTelemetry(要改 app_sensors 一侧),
    // 所以 gpsValid=false; 选择器也不会选它(gpsEnabled 来自 cfg::feature::kGnss, 当前 false)。
    s.gpsValid = false;
}

//===================================================================================================
// 七、上行帧处理(L1: 版本/会话; L2: 位置(交 RoundSlave); L3: 语义夹取)
//===================================================================================================
inline void linkCountReject(LinkThreadCtx *ctx, LinkReject reason)
{
    ctx->lastReject                       = reason;
    ctx->rejectsBy[(uint8_t)reason]       = ctx->rejectsBy[(uint8_t)reason] + 1u;
    ++ctx->admitRejects;
}

/** ESTOP 锁存: 任一上行帧 flags.bit0=1 ⇒ 请求硬停机, 直到新的 ARM(与控制线程的握手保持原样) */
inline void linkLatchEstop(LinkThreadCtx *ctx)
{
    if (ctx->emergencyStop.pending) {
        return; // 已经锁存: 不重复请求(否则每帧都会产生一个新 requestId)
    }
    ctx->controlInput.emergencyStopId      = ctx->emergencyStop.request();
    ctx->controlInput.emergencyStopPending = true;
    ctx->sh->control.input.store(ctx->controlInput);
    ++ctx->estopLatches;
}

/** 把一帧杆量写进共享输入(UP_ROUND 与 UP_CTRL 同量纲, 共用这一条路径) */
inline void linkApplyStick(LinkThreadCtx *ctx, int16_t roll, int16_t pitch, uint16_t thr, uint32_t nowMs)
{
    const float rollN  = static_cast<float>(roll) / 1023.0f;
    const float pitchN = static_cast<float>(pitch) / 1023.0f;
    const float thrN   = static_cast<float>(thr) / 2047.0f;

    ctx->controlInput.command           = 0u;
    ctx->controlInput.manualThrottle    = thrN;
    ctx->controlInput.setpoint.attitude = quatFromEulerZYX(
        0.0f, pitchN * cfg::control::kTiltMaxRad, rollN * cfg::control::kTiltMaxRad);
    ctx->controlInput.setpoint.height_m = 0.0f;
    ctx->controlInput.receivedMs        = nowMs;
    ctx->controlInput.rollCounts        = roll;
    ctx->controlInput.pitchCounts       = pitch;
    ctx->controlInput.throttleCounts    = thr;
    ctx->controlInput.stickReceivedMs   = nowMs;
    ctx->controlInput.stickValid        = true;
    ctx->controlInput.valid             = true;
    ++ctx->controlInput.sequence;
    if (++ctx->controlInput.motorSequence == 0u) {
        ++ctx->controlInput.motorSequence; // 0 是"从没收到过"的哨兵
    }
    ctx->controlInput.motorReceivedMs = nowMs;
    ctx->sh->control.input.store(ctx->controlInput);
    ctx->health.success(SensorHealth::LINK, nowMs);
    ++ctx->applied;
}

/** UP_CMD: cmd_id 幂等 + 通道权限 + 投递到控制线程(结果回来时补 ACK) */
inline void linkHandleCommand(LinkThreadCtx *ctx, const link::UpCmdPayload &cmd, uint32_t nowMs)
{
    if (cmd.cmdId == 0u) {
        return; // cmd_id=0 无效(规格 §3.3)
    }
    uint8_t historyResult = 0u;
    if (ctx->cmdHist.find(cmd.cmdId, historyResult)) {
        // 重复(同一 session 内的重传): 只重发 ACK, 不重复执行; 结果还没回来就不回(等结果)
        if (historyResult != app::kCmdResultPending) {
            ctx->dl.pushAck(cmd.cmdId, historyResult);
        }
        return;
    }

    app::CommandOp    op;
    app::GroundStatus result = app::GroundStatus::BadCommand;
    bool              posted = false;

    if (app::mapUpCmdOp(cmd.op, op)) {
        result = app::admitCommandChannel(op, true);
        if (result == app::GroundStatus::Ok && op == app::CommandOp::EmergencyStop) {
            linkLatchEstop(ctx);
            ctx->dl.pushAck(cmd.cmdId, static_cast<uint8_t>(app::GroundStatus::Ok));
            ctx->cmdHist.record(cmd.cmdId, static_cast<uint8_t>(app::GroundStatus::Ok));
            ctx->lastCmdId     = cmd.cmdId;
            ctx->lastCmdResult = static_cast<uint8_t>(app::GroundStatus::Ok);
            return;
        }
        if (result == app::GroundStatus::Ok && op == app::CommandOp::Ping) {
            ctx->dl.pushAck(cmd.cmdId, static_cast<uint8_t>(app::GroundStatus::Ok));
            ctx->cmdHist.record(cmd.cmdId, static_cast<uint8_t>(app::GroundStatus::Ok));
            ctx->lastCmdId     = cmd.cmdId;
            ctx->lastCmdResult = static_cast<uint8_t>(app::GroundStatus::Ok);
            return;
        }
        if (result == app::GroundStatus::Ok) {
            app::Request request;
            if (app::taskOfCommand(op, cmd.arg, request.task)) {
                request.source        = app::RequestSource::Link;
                request.id            = static_cast<uint32_t>(cmd.cmdId); // 1..255 唯一标识这一条
                request.issuedMs      = nowMs;
                request.safetyEpoch   = ctx->sh->control.safetyEpoch.load();
                request.wireSequence  = cmd.cmdId; ///< v2: 这个字段承载 cmd_id(配对 DL_ACK)
                request.wireOperation = cmd.op;
                if (!ctx->sh->tryPostRequest(request)) {
                    result = app::GroundStatus::Busy;
                } else {
                    posted = true;
                    ctx->cmdHist.record(cmd.cmdId, app::kCmdResultPending);
                    ++ctx->requestsInFlight;
                    ++ctx->cmdsPosted;
                    ctx->lastCmdId = cmd.cmdId;
                }
            } else {
                result = app::GroundStatus::BadCommand;
            }
        }
    }

    if (!posted) {
        ctx->cmdHist.record(cmd.cmdId, static_cast<uint8_t>(result));
        ctx->dl.pushAck(cmd.cmdId, static_cast<uint8_t>(result));
        ctx->lastCmdId     = cmd.cmdId;
        ctx->lastCmdResult = static_cast<uint8_t>(result);
    }
}

/** 处理一帧已解出(CRC/长度/类型均合法)的上行帧 */
inline void linkHandleUpFrame(LinkThreadCtx *ctx, const link::Frame &f, uint32_t frameEndUs)
{
    const uint32_t nowMs = rt_tick_get_millisecond();
    ++ctx->rxFrames;
    ctx->lastRxUs = frameEndUs;

    // ---- HELLO: 双向同型, 任何时候都接受(版本不一致也回, 让对方看到本端版本) ----
    if (f.type == link::FrameType::Hello) {
        ++ctx->helloRequests;
        ctx->peerVersion     = f.u.hello.ver;
        ctx->peerVersionSeen = true;
        ctx->rcWantsGps      = ((f.u.hello.caps & link::hello::kCapsGnss) != 0u);
        if (f.u.hello.ver != static_cast<uint8_t>(LinkProtoVersion)) {
            ++ctx->helloVersionMismatch; // 只记: ACQUIRE 不进 RUN 由 RoundSlave 保证(它只认 UP_ROUND)
        }
        ctx->round.onHelloRequest(f.u.hello.ver, f.u.hello.caps, f.u.hello.nonce, frameEndUs);
        ++ctx->applied;
        return;
    }

    // ---- L1: 角色类型(L0 已过 CRC/长度; 这里拒下行型与保留型) ----
    bool isUplinkType = false;
    switch (f.type) {
    case link::FrameType::UpRound:
    case link::FrameType::UpCtrl:
    case link::FrameType::UpCmd:
    case link::FrameType::UpVel:
    case link::FrameType::UpPos:
    case link::FrameType::UpSetpoint:
    case link::FrameType::UpMotor:
        isUplinkType = true;
        break;
    default:
        break;
    }
    if (!isUplinkType) {
        linkCountReject(ctx, LinkReject::TypeNotUplink);
        return;
    }

    // ---- L2: 轮次层(位置/会话/迟到); 上行帧长度由类型唯一决定 ----
    const uint16_t frameBytesLen = link::frameBytes(f.type);
    const uint8_t  session = (f.type == link::FrameType::UpRound) ? f.u.round.session : 0u;
    const link::RoundFrameResult rr =
        ctx->round.onFrame(f.type, f.seq, frameBytesLen, frameEndUs, session);

    if (!rr.accepted) {
        LinkReject why = LinkReject::NoRound;
        if (!ctx->round.run()) {
            why = LinkReject::AcquireOnly;
        } else if (f.type == link::FrameType::UpRound && session == 0u) {
            why = LinkReject::BadSession;
        }
        linkCountReject(ctx, why);
        return;
    }
    if (rr.lateIgnored) {
        ++ctx->lateFrames; // 只计数: 控制量照常采信(仍是新鲜杆量), 但不再参与 D0 决策
    }
    if (rr.sessionChanged) {
        // 新会话: 上一会话的命令去重历史/未发 ACK 全部作废(规格 §8.1)
        ctx->cmdHist.clear();
        ctx->dl.onSessionChanged();
        ++ctx->sessionChanges;
    }

    // ---- L3: 语义(按类型落到 ControlInput / 命令通道) ----
    switch (f.type) {
    case link::FrameType::UpRound: {
        if ((f.u.round.flags & link::upCtrl::kFlagEstop) != 0u ||
            (f.u.round.flags8 & link::upRound::kFlag8Estop) != 0u) {
            linkLatchEstop(ctx);
        }
        linkApplyStick(ctx, f.u.round.roll, f.u.round.pitch, f.u.round.thr, nowMs);
        break;
    }
    case link::FrameType::UpCtrl: {
        if ((f.u.ctrl.flags & link::upCtrl::kFlagEstop) != 0u) {
            linkLatchEstop(ctx);
        }
        linkApplyStick(ctx, f.u.ctrl.roll, f.u.ctrl.pitch, f.u.ctrl.thr, nowMs);
        break;
    }
    case link::FrameType::UpCmd:
        linkHandleCommand(ctx, f.u.cmd, nowMs);
        break;
    case link::FrameType::UpVel: {
        ctx->controlInput.velocityTargetMps[0] = f.u.vel.vx;
        ctx->controlInput.velocityTargetMps[1] = f.u.vel.vy;
        ctx->controlInput.velocityTargetMps[2] = f.u.vel.vz;
        ctx->controlInput.velocityValid        = true;
        ctx->controlInput.velocityReceivedMs   = nowMs;
        ctx->controlInput.receivedMs           = nowMs;
        ctx->controlInput.valid                = true;
        ++ctx->controlInput.sequence;
        ctx->sh->control.input.store(ctx->controlInput);
        ctx->health.success(SensorHealth::LINK, nowMs);
        ++ctx->applied;
        break;
    }
    case link::FrameType::UpSetpoint: {
        if ((f.u.setpoint.flags & link::upSetpoint::kFlagAltValid) != 0u) {
            ctx->controlInput.altitudeTargetM = f.u.setpoint.altM;
            ctx->controlInput.altitudeValid   = true;
        }
        ctx->controlInput.receivedMs = nowMs;
        ctx->controlInput.valid      = true;
        ++ctx->controlInput.sequence;
        ctx->sh->control.input.store(ctx->controlInput);
        ctx->health.success(SensorHealth::LINK, nowMs);
        ++ctx->applied;
        break;
    }
    case link::FrameType::UpMotor: {
        for (uint8_t i = 0u; i < link::upMotor::kMotorCount; ++i) {
            const uint16_t raw = f.u.motor.throttle[i];
            // L3: 2000..2047 是 DShot 特殊命令区(beacon/遥测), 当成油门会让电机"唱歌"。
            // 夹到 0..1999 并计数 —— 不整帧拒: 四路里坏一路不该让另外三路失效。
            if (raw > link::kMotorMax) {
                ++ctx->motorClamped;
                ctx->controlInput.motorValue[i] = link::kMotorMax;
            } else {
                ctx->controlInput.motorValue[i] = raw;
            }
        }
        ctx->controlInput.receivedMs = nowMs;
        ctx->controlInput.valid      = true;
        ++ctx->controlInput.sequence;
        ctx->sh->control.input.store(ctx->controlInput);
        ctx->health.success(SensorHealth::LINK, nowMs);
        ++ctx->applied;
        break;
    }
    case link::FrameType::UpPos:
        // 位置注入本轮协议保留未用(0xB): 解码通过但业务不接, 计数而不是静默丢弃
        ++ctx->applied;
        break;
    default:
        break;
    }
}

//===================================================================================================
// 八、接收: 突发队列 → 组装缓冲 → 自定界 → 分发
//===================================================================================================
inline void linkDrainRx(LinkThreadCtx *ctx, uint32_t nowUs, uint32_t nowCycles)
{
    LinkRxBurst burst;
    while (ctx->rxq.pop(burst)) {
        if (burst.len == 0u) {
            continue;
        }
        // 本端发 D0 期间收到的字节: 半双工下不可能是合法对端数据, 直接丢弃(单独计数)。
        if (linkCyclesInTxWindow(ctx, burst.cycles)) {
            ctx->txWinBytes += burst.len;
            ++ctx->txWinBursts;
            continue;
        }
        ctx->bytesIn += burst.len;
        const uint32_t endUs = linkCyclesToUs(burst.cycles, nowUs, nowCycles);

        // ---- 追加到组装缓冲 ----
        if (static_cast<uint16_t>(ctx->asmL + burst.len) > static_cast<uint16_t>(sizeof(ctx->asmBuf))) {
            // 正常不会发生(单段 ≤32B, 半帧残留 ≤20B); 真溢出就丢掉旧残留, 从头重同步
            ctx->asmL = 0u;
            ++ctx->decodeRejects;
            if (burst.len > static_cast<uint16_t>(sizeof(ctx->asmBuf))) {
                continue;
            }
        }
        memcpy(ctx->asmBuf + ctx->asmL, burst.data, burst.len);
        ctx->asmL     = static_cast<uint16_t>(ctx->asmL + burst.len);
        ctx->asmEndUs = endUs;

        // ---- 切帧: 解出/半帧/坏帧 三种结果(坏帧只前进 1 字节) ----
        uint16_t off = 0u;
        while (ctx->asmL > off) {
            link::Frame frame;
            uint16_t    consumed = 0u;
            const int   r        = link::tryDecode(ctx->asmBuf + off,
                                                   static_cast<uint16_t>(ctx->asmL - off), frame, consumed);
            if (r == link::kTryNeedMore) {
                break; // 半帧: 一个字节都不能丢
            }
            if (r == link::kTryDecoded) {
                linkHandleUpFrame(ctx, frame, ctx->asmEndUs);
            } else {
                ++ctx->decodeRejects;
            }
            off = static_cast<uint16_t>(off + consumed);
        }
        if (off > 0u) {
            memmove(ctx->asmBuf, ctx->asmBuf + off, static_cast<size_t>(ctx->asmL - off));
            ctx->asmL = static_cast<uint16_t>(ctx->asmL - off);
        }
    }
}

//===================================================================================================
// 九、发送: RoundSlave 到点 → 组帧 → DMA 连续写
//===================================================================================================
inline link::FrameType linkPickType(link::DlPickKind k)
{
    switch (k) {
    case link::DlPickKind::Gps:   return link::FrameType::DlGps;
    case link::DlPickKind::Sys:   return link::FrameType::DlSys;
    case link::DlPickKind::State: return link::FrameType::DlState;
    case link::DlPickKind::Att:   return link::FrameType::DlAtt;
    default:                      return link::FrameType::DlAtt;
    }
}

inline uint16_t linkPlannedBytes(link::DlPickKind k)
{
    switch (k) {
    case link::DlPickKind::Hello: return link::kHelloBytes;
    case link::DlPickKind::Ack:   return link::kDlAckBytes;
    case link::DlPickKind::Gps:   return link::kDlGpsBytes;
    case link::DlPickKind::Sys:   return link::kDlSysBytes;
    case link::DlPickKind::State: return link::kDlStateBytes;
    case link::DlPickKind::Att:   return link::kDlAttBytes;
    default:                      return 0u;
    }
}

inline void linkServiceTx(LinkThreadCtx *ctx, uint32_t &nowUs)
{
    if (ctx->radio == nullptr || !ctx->round.initialized() || !ctx->dl.initialized()) {
        return;
    }
    // 上一帧还没发完: 本轮不再排队(每轮最多 1 帧, 排队只会压到下一轮)
    if (ctx->radio->txBusy()) {
        return;
    }

    for (uint8_t attempt = 0u; attempt < 2u; ++attempt) {
        // 选帧是**纯查询**: 预算不过/发送失败时不 consume, ACK 不会被吃掉
        const link::DlPick pick    = ctx->dl.pick(ctx->round.helloReplyPending(), ctx->rcWantsGps);
        const uint16_t     planned = linkPlannedBytes(pick.kind);
        if (planned == 0u) {
            return;
        }
        // AUX 是发送前最后兜底(§2.3 第 1 条); 预算用"所选帧的整帧长度"(§2.3 第 2 条)
        const bool                   auxIdle = ctx->radio->auxReady();
        link::RoundSlave::TxDecision d       = ctx->round.decide(nowUs, auxIdle, planned);

        if (!d.send) {
            if (d.skip == link::RoundSkipReason::NotDue && d.sendAtUs != 0u && attempt == 0u &&
                !link::roundTimeReached(nowUs, d.sendAtUs)) {
                const uint32_t waitUs = static_cast<uint32_t>(d.sendAtUs - nowUs);
                // 只等一个 tick 之内(guard≈3ms): 忙等换 D0 精度; 更远的时刻交给下一次中断/定时唤醒
                if (waitUs > 0u && waitUs <= LinkTickMs * 1000u) {
                    delay_us(waitUs);
                    nowUs = linkMicros();
                    continue; // 醒来重新读 AUX 再判
                }
            }
            if (d.skip == link::RoundSkipReason::AuxBusy || d.skip == link::RoundSkipReason::Budget) {
                ctx->round.onDownlinkSkipped(d.skip);
                ++ctx->txFail; // "本轮没发出去": 与 sendDma 失败同口径(观测 §8.4)
                if (d.skip == link::RoundSkipReason::AuxBusy) {
                    ++ctx->skipAux;
                } else {
                    ++ctx->skipBudget;
                }
                if (ctx->round.run()) {
                    // 只有 RUN 里的 D0 才计入"本轮关闭"; ACQUIRE 的 HELLO 放弃不算一轮(否则上行/下行
                    // 丢包统计会把握手期算成"丢了一轮")
                    linkCloseRoundStats(ctx, false);
                }
            }
            return;
        }

        // ---- 组帧(长度已在上面做过预算) ----
        uint8_t         frame[link::kMaxFrameBytes];
        uint16_t        n       = 0u;
        const bool      isHello = d.isHello;

        if (isHello) {
            const link::HelloPayload reply = ctx->round.takeHelloReply();
            n                             = link::encodeHello(reply, ctx->downlinkSeq++, frame);
        } else if (pick.kind == link::DlPickKind::Ack) {
            n = link::encodeDlAck(pick.ack, ctx->downlinkSeq++, frame);
        } else if (pick.kind == link::DlPickKind::Hello) {
            return; // pick 返回 Hello 时 decision.isHello 一定为真, 走不到
        } else {
            const link::FrameType type = linkPickType(pick.kind);
            link::DownlinkSource  src;
            linkFillDownlinkSource(ctx, src);
            n = link::buildDownlinkFrame(type, ctx->downlinkSeq++, src, frame);
        }

        if (n == 0u) {
            // 选到的帧本轮组不出来(例如 DL_GPS 但没有 GPS 数据): 放弃, 绝不发空帧
            ctx->lastReject = LinkReject::Build;
            ++ctx->skipUnavailable;
            ctx->round.onDownlinkSkipped(link::RoundSkipReason::Unavailable);
            if (ctx->round.run()) {
                linkCloseRoundStats(ctx, false);
            }
            return;
        }

        // ---- 发送: DMA 一帧一次连续写(写前 AUX 已在 decide 里判过) ----
        // 先开"RX 抑制窗口": [发送起点, 空中结束 + guard + 2ms]。窗口内的 RX 突发只可能是
        // 换向/串扰瞬变, 由 linkDrainRx 丢弃并计数(否则会表现为 `拒` 稳定增长)。
        {
            const uint32_t usPerCycle = static_cast<uint32_t>(SystemCoreClock / 1000000u);
            const uint32_t airUs      = link::roundAirtimeUs(ctx->round.params(), n);
            ctx->txWinStartCycles     = DWT->CYCCNT;
            ctx->txWinEndCycles =
                ctx->txWinStartCycles + (airUs + static_cast<uint32_t>(LinkGuardUs) + 2000u) * usPerCycle;
        }
        if (!ctx->radio->sendDma(frame, n)) {
            ctx->txWinEndCycles = 0u; // 没真正发出去: 不抑制
            ++ctx->txFail;
            ctx->round.onDownlinkSkipped(link::RoundSkipReason::AuxBusy);
            ++ctx->skipAux;
            if (ctx->round.run()) {
                linkCloseRoundStats(ctx, false);
            }
            return;
        }
        ++ctx->txFrames;
        ctx->round.onDownlinkSent(isHello, d.path, nowUs, n);
        if (!isHello) {
            ctx->dl.consume(pick); // 遥测推进轮号 / ACK 出队; HELLO 不占轮次
            linkCloseRoundStats(ctx, true);
        }
        return;
    }
}

//===================================================================================================
// 十、线程
//===================================================================================================
inline void linkPublishStatistics(LinkThreadCtx *ctx)
{
    LinkStatistics &s = ctx->statistics;
    s.linkBytesIn      = ctx->bytesIn;
    s.linkRxFrames     = ctx->rxFrames;
    s.linkRxRejects    = ctx->decodeRejects;
    s.linkAdmitRejects = ctx->admitRejects;
    s.linkCmdsApplied  = ctx->applied;
    s.linkTxFrames     = ctx->txFrames;
    s.linkTxFail       = ctx->txFail;
    s.linkTxSkipGps    = ctx->skipAux + ctx->skipBudget + ctx->skipUnavailable; // v2: 本轮 D0 放弃次数
    s.linkMuted        = ctx->round.acquire() ? 1u : 0u;                       // ACQUIRE = 只收不发
    s.linkGoodStreak   = ctx->round.rxBitmap();                                // 本轮 U0..U3 接收位图
    s.linkWindowArmed  = ctx->round.roundActive() ? 1u : 0u;
    s.linkUlLossPct    = linkUlLossPct(ctx);
    s.uplinkStreak     = static_cast<uint16_t>(ctx->round.rxCountAtPos(3u));
    s.linkTxWinBytes   = ctx->txWinBytes;
    for (uint8_t i = 0u; i < 4u; ++i) {
        s.linkRxPos[i] = ctx->round.rxCountAtPos(i);
    }
}

inline void linkTelemetryThreadEntry(void *arg)
{
    LinkThreadCtx *ctx = static_cast<LinkThreadCtx *>(arg);
    FlightShared  &sh  = *ctx->sh;
    paintStackWatermark();

    while (true) {
        // 6ms 超时 = watchdog 唤醒(只做超时/统计); IDLE 中断 release = 正常路径立即唤醒。
        // 返回值不用: 超时是正常现象(没有上行帧时每 6ms 都会超时一次)。
        (void)rt_sem_take(ctx->sem, rt_tick_from_millisecond(static_cast<rt_int32_t>(LinkTickMs)));

        const uint32_t nowCycles = DWT->CYCCNT;
        uint32_t       nowUs     = linkMicros();

        // ---- ① 控制线程的结果 → DL_ACK 队列(cmd_id 配对) ----
        app::RequestResult feedback;
        while (sh.control.results.tryPop(feedback)) {
            if (ctx->requestsInFlight > 0u) {
                --ctx->requestsInFlight;
            }
            if (feedback.kind == app::ResultKind::EmergencyStop) {
                if (ctx->emergencyStop.acknowledge(feedback.id)) {
                    ctx->controlInput.emergencyStopPending = false;
                    sh.control.input.store(ctx->controlInput);
                }
                ctx->dl.pushAck(feedback.wireSequence, static_cast<uint8_t>(app::GroundStatus::Ok));
                ctx->cmdHist.update(feedback.wireSequence, static_cast<uint8_t>(app::GroundStatus::Ok));
            } else {
                const uint8_t res = static_cast<uint8_t>(feedback.status);
                ctx->dl.pushAck(feedback.wireSequence, res);
                ctx->cmdHist.update(feedback.wireSequence, res);
                ctx->lastCmdId     = feedback.wireSequence;
                ctx->lastCmdResult = res;
            }
        }

        // ---- ② 收: 突发 → 组装 → 自定界 → 分层准入 → 分发 ----
        linkDrainRx(ctx, nowUs, nowCycles);

        // ---- ③ watchdog: >200ms 无合法上行 → ACQUIRE(只收不发) ----
        const link::RoundPollResult polled = ctx->round.poll(nowUs);
        if (polled.enteredAcquire) {
            // 失同步 = 只收不发; 用健康项的边沿限流报一次(否则 6ms 循环会刷爆异常队列)
            if (ctx->health.failure(SensorHealth::LINK)) {
                DLX_HANDLE_EXCEPTION(ExceptionCode::LINK_DESYNC);
            }
        } else if (ctx->round.run() && ctx->controlInput.valid &&
                   ctx->health.success(SensorHealth::LINK, ctx->controlInput.receivedMs)) {
            DLX_HANDLE_EXCEPTION(ExceptionCode::LINK_RESYNCED); // 提示性: 已重新对上遥控器轮次
        }
        // ---- ④ 发: 到点就发一帧(每轮最多 1 帧; HELLO 应答占槽但不计遥测轮次) ----
        linkServiceTx(ctx, nowUs);

        // ---- ⑤ 发布统计(日志线程按同一时刻读) ----
        linkPublishStatistics(ctx);
        rt_enter_critical();
        sh.linkStatistics.store(ctx->statistics);
        rt_exit_critical();
    }
}

} // namespace dlx
