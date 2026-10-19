//===================================================================================================
// test/link_v2/fc/round_test.cpp —— 飞控链路 v2 轮次从机(RoundSlave) + 下行选帧器 主机单测
//
// 依据: docs/链路层v2_协议规格.md §2(时序)、§4(飞控状态机)、§6(下行调度)、§13 E4/E5(超时口径)。
// 覆盖:
//   1. ACQUIRE/RUN 切换: ACQUIRE 只认 UP_ROUND; 0x8–0xD 在 ACQUIRE 被忽略; >200ms 回 ACQUIRE;
//   2. t0 反推(pos0..3 × 各自帧长), session 变化/非法 session(0);
//   3. D0 触发: pos3 正常路径(t0+86038)、U3 丢失兜底路径(t0+89038)、每轮最多 1 帧;
//   4. 发送前检查: AUX 忙放弃、预算不足放弃(§2.3); 放弃后本轮不再重试;
//   5. HELLO: ACQUIRE 尽快回、版本不一致也回但 ACQUIRE 不进 RUN、RUN 忙时占 D0 槽、nonce 回显;
//   6. 时间戳模型: 收帧末修正按帧长算(rawIdle = airEnd + (n+rxIdleBytes)×174us), n=5/9/11 锚点回归;
//   7. 下行选帧: 优先级 HELLO>ACK>GPS>SYS>STATE>ATT、到期节拍、ACK 队列顺序/容量、session 清队。
//
// 时间基准(P0 冻结值, 与 app/link/link_v2_config.hpp 同值): F(n)=13555+387n
//   F(UP_CTRL 9B)=17038us  F(UP_ROUND 11B)=17812us  F(DL_ATT 13B)=18586us  F(DL_GPS 21B)=21682us
//   收帧末: rawIdle = airEnd + (n + LinkRxIdleBytes(1))×174us
//   U3 空中结束(期望) = t0 + 3×22000 + 17038 = t0 + 83038
//   U3 兜底 deadline  = t0 + 3×22000 + 17038 + 3000(抖动) = t0 + 86038
//   D0 正常发送时刻   = u3End + guard = t0 + 86038   (guard 与抖动同值 3000 时两者相等)
//   D0 兜底发送时刻   = u3Deadline + guard = t0 + 89038
//   预算(21B) = F(21)21682 + 21×174(3654) + 1000 = 26336us
//
// 构建:
//   g++ -std=c++11 -Wall -Wextra -I test/stub -I DL_LIB -I . test/link_v2/fc/round_test.cpp -o round_test.exe
//===================================================================================================
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "link/dlx_link_downlink.hpp"
#include "link/dlx_link_frame.hpp"
#include "link/dlx_link_round.hpp"

using namespace dlx::link;

namespace
{

int g_checks   = 0;
int g_failures = 0;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        ++g_checks;                                                                                \
        if (!(cond)) {                                                                             \
            ++g_failures;                                                                          \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);                                 \
        }                                                                                          \
    } while (0)

#define CHECK_EQ_U(a, b)                                                                           \
    do {                                                                                           \
        ++g_checks;                                                                                \
        const unsigned long va_ = (unsigned long)(a);                                              \
        const unsigned long vb_ = (unsigned long)(b);                                              \
        if (va_ != vb_) {                                                                          \
            ++g_failures;                                                                          \
            printf("FAIL %s:%d  %s=%lu != %s=%lu\n", __FILE__, __LINE__, #a, va_, #b, vb_);        \
        }                                                                                          \
    } while (0)

//---------------------------------------------------------------------------------------------------
// 公共常量: 与 app/link/link_v2_config.hpp 同值(主机测试不引用配置头, 避免把 app 依赖带进来)
//---------------------------------------------------------------------------------------------------
constexpr uint32_t kRoundMs     = 120u;
constexpr uint32_t kSpacingMs   = 22u;
constexpr uint32_t kGuardUs     = 3000u;
constexpr uint32_t kJitterUs    = 3000u;
constexpr uint32_t kDlTimeoutMs = 117u;
constexpr uint32_t kAcquireMs   = 200u;
constexpr uint32_t kBaseUs      = 13555u; // P0 冻结: LinkAirtimeBaseUs
constexpr uint32_t kPerByteUs   = 387u;   // P0 冻结: LinkAirtimePerByteUs
constexpr uint32_t kUartByteUs  = 174u;   // LinkUartByteUs
constexpr uint32_t kRxIdleBytes = 1u;     // LinkRxIdleBytes(P0: 1)
constexpr uint32_t kBudgetSlackUs = 1000u; // LinkDlBudgetSlackUs
constexpr uint32_t kRoundReopenUs = 60000u; // 规格 §4 "本轮关闭" 阈值
constexpr uint32_t kT0          = 1000000u; // 任意 t0(避开 0, 便于看出"少减")

RoundParams makeParams()
{
    RoundParams p       = {};
    p.roundMs           = kRoundMs;
    p.frameSpacingMs    = kSpacingMs;
    p.guardUs           = kGuardUs;
    p.u3JitterUs        = kJitterUs;
    p.dlTimeoutMs       = kDlTimeoutMs;
    p.acquireMs         = kAcquireMs;
    p.airtimeBaseUs     = kBaseUs;
    p.airtimePerByteUs  = kPerByteUs;
    p.uartByteUs        = kUartByteUs;
    p.dlBudgetSlackUs   = kBudgetSlackUs;
    p.roundReopenUs     = kRoundReopenUs;
    p.rxIdleBytes       = kRxIdleBytes;
    p.protoVersion      = 2u;
    p.caps              = hello::kCapsCmdId;
    p.helloMax          = 5u;
    p.auxGate           = true;
    return p;
}

/** 一帧的 IDLE 回调入口时刻(raw): airEnd + (len + rxIdleBytes)×uartByteUs
 *  = t0 + pos*spacing + F(len) + (len + 1)×174 */
uint32_t rawEndFor(uint32_t t0, uint8_t pos, uint16_t frameLen)
{
    const RoundParams p = makeParams();
    return (uint32_t)(t0 + (uint32_t)pos * kSpacingMs * 1000u + roundAirtimeUs(p, frameLen) +
                      (uint32_t)(frameLen + kRxIdleBytes) * kUartByteUs);
}

/** 新预算公式(规格 §2.3, 2026-10-18 修订): airtime + 帧长×174 + 1000 */
uint32_t budgetNeed(uint16_t frameBytes)
{
    const RoundParams p = makeParams();
    return (uint32_t)(roundAirtimeUs(p, frameBytes) + (uint32_t)frameBytes * kUartByteUs + kBudgetSlackUs);
}

/** 把一帧喂进 RoundSlave(用与 app 相同的"位置 + 长度"约定) */
RoundFrameResult feed(RoundSlave &rs, FrameType type, uint8_t seq, uint16_t len, uint32_t rawEnd,
                      uint8_t session = 0u)
{
    return rs.onFrame(type, seq, len, rawEnd, session);
}

//---------------------------------------------------------------------------------------------------
// 1. 参数校验 / F(n)
//---------------------------------------------------------------------------------------------------
void testParams()
{
    RoundParams p = makeParams();
    CHECK(roundParamsValid(p));

    // P0 冻结的 F(n) = 13555 + 387n
    CHECK_EQ_U(roundAirtimeUs(p, 5u), 15490u);
    CHECK_EQ_U(roundAirtimeUs(p, 9u), 17038u);
    CHECK_EQ_U(roundAirtimeUs(p, 11u), 17812u);
    CHECK_EQ_U(roundAirtimeUs(p, 13u), 18586u);
    CHECK_EQ_U(roundAirtimeUs(p, 21u), 21682u);
    // 收帧末修正按帧长算: rawIdle = airEnd + (n+1)×174
    RoundSlave probe;
    CHECK(probe.init(p));
    CHECK_EQ_U(probe.airEndUs(1000000u + 10u * kUartByteUs, 9u), 1000000u);
    CHECK_EQ_U(probe.airEndUs(1000000u + 12u * kUartByteUs, 11u), 1000000u);

    RoundSlave rs;
    CHECK(rs.init(p));
    CHECK(rs.initialized());
    CHECK(rs.acquire());
    CHECK(!rs.run());

    // 各种非法参数: init() 必须失败且保持未初始化
    RoundParams b = p;
    b.guardUs     = 0u;
    CHECK(!RoundSlave().init(b));
    b = p;
    b.roundMs = 88u; // 4*22 >= 88
    CHECK(!RoundSlave().init(b));
    b = p;
    b.dlTimeoutMs = 120u;
    CHECK(!RoundSlave().init(b));
    b = p;
    b.protoVersion = 0u;
    CHECK(!RoundSlave().init(b));
    b = p;
    b.protoVersion = 16u;
    CHECK(!RoundSlave().init(b));
    b = p;
    b.rxIdleBytes = 9u; // > 8: 单位/标定写错
    CHECK(!RoundSlave().init(b));
    b = p;
    b.rxIdleBytes = 8u; // 上界仍然合法(容错范围)
    CHECK(RoundSlave().init(b));
    b = p;
    b.helloMax = 0u;
    CHECK(!RoundSlave().init(b));
    b = p;
    b.uartByteUs = 0u;
    CHECK(!RoundSlave().init(b));
    b = p;
    b.uartByteUs = 500u; // > airtimePerByteUs: 单位/参数写错
    CHECK(!RoundSlave().init(b));
    b = p;
    b.dlBudgetSlackUs = 20000u;
    CHECK(!RoundSlave().init(b));
    b = p;
    b.roundReopenUs = 0u;
    CHECK(!RoundSlave().init(b));
    b = p;
    b.roundReopenUs = 120000u; // >= 轮长
    CHECK(!RoundSlave().init(b));

    RoundSlave bad;
    CHECK(!bad.init(b));
    CHECK(!bad.initialized());
    // 未初始化时 decide 必须返回 Disabled(而不是"可以发")
    const RoundSlave::TxDecision d = bad.decide(0u);
    CHECK(!d.send);
    CHECK(d.skip == RoundSkipReason::Disabled);
}

//---------------------------------------------------------------------------------------------------
// 2. ACQUIRE: 只认 UP_ROUND; 0x8–0xD 忽略; session 校验
//---------------------------------------------------------------------------------------------------
void testAcquire()
{
    RoundSlave rs;
    CHECK(rs.init(makeParams()));

    // 0x8–0xD 在 ACQUIRE 里必须被忽略(规格 §4 最后一条: 这就是 v1 RC 不会误控制的保证)
    const RoundFrameResult r1 = feed(rs, FrameType::UpCtrl, 1u, kUpCtrlBytes, rawEndFor(kT0, 1u, kUpCtrlBytes));
    CHECK(!r1.accepted);
    CHECK_EQ_U(rs.ignoredInAcquireCount(), 1u);
    CHECK(rs.acquire());

    // HELO 在 ACQUIRE 只排队应答, 不进 RUN
    rs.onHelloRequest(2u, hello::kCapsGnss, 0x5Au, 500000u);
    CHECK(rs.helloReplyPending());
    CHECK(rs.acquire());
    CHECK_EQ_U(rs.helloEarliestUs(),
               500000u - (kHelloBytes + kRxIdleBytes) * kUartByteUs + kGuardUs);

    // session=0 的 UP_ROUND 非法: 不采信、不进 RUN
    const uint32_t raw0 = rawEndFor(kT0, 0u, kUpRoundBytes);
    const RoundFrameResult rs0 = feed(rs, FrameType::UpRound, 0u, kUpRoundBytes, raw0, 0u);
    CHECK(!rs0.accepted);
    CHECK(!rs0.startedRun);
    CHECK(rs.acquire());
    CHECK_EQ_U(rs.badSessionCount(), 1u);

    // 合法的 UP_ROUND(pos0) 进 RUN
    const RoundFrameResult rp = feed(rs, FrameType::UpRound, 4u, kUpRoundBytes, raw0, 0x3Cu);
    CHECK(rp.accepted);
    CHECK(rp.startedRun);
    CHECK(rp.sessionChanged);
    CHECK_EQ_U(rp.pos, 0u);
    CHECK(rs.run());
    CHECK_EQ_U(rs.session(), 0x3Cu);
    CHECK_EQ_U(rs.roundT0Us(), kT0); // t0 反推必须精确
    CHECK_EQ_U(rs.rxBitmap(), 0x01u);
}

//---------------------------------------------------------------------------------------------------
// 3. t0 反推: pos0..3 用各自帧长, t0 不变; pos3 记 u3End
//---------------------------------------------------------------------------------------------------
void testT0Reconstruction()
{
    RoundSlave rs;
    CHECK(rs.init(makeParams()));

    feed(rs, FrameType::UpRound, 0u, kUpRoundBytes, rawEndFor(kT0, 0u, kUpRoundBytes), 0x11u);
    CHECK_EQ_U(rs.roundT0Us(), kT0);

    feed(rs, FrameType::UpCtrl, 1u, kUpCtrlBytes, rawEndFor(kT0, 1u, kUpCtrlBytes), 0u);
    CHECK_EQ_U(rs.roundT0Us(), kT0);
    CHECK_EQ_U(rs.rxBitmap(), 0x03u);

    // U2 可能是 UP_CMD(6B), 长度不同也必须反推出同一个 t0
    feed(rs, FrameType::UpCmd, 2u, kUpCmdBytes, rawEndFor(kT0, 2u, kUpCmdBytes), 0u);
    CHECK_EQ_U(rs.roundT0Us(), kT0);
    CHECK_EQ_U(rs.rxBitmap(), 0x07u);
    CHECK(!rs.u3Seen());

    // U3: 记 u3End, deadline = t0 + 3*22ms + F(UP_CTRL) + jitter
    const RoundFrameResult r3 =
        feed(rs, FrameType::UpCtrl, 3u, kUpCtrlBytes, rawEndFor(kT0, 3u, kUpCtrlBytes), 0u);
    CHECK(r3.accepted);
    CHECK(r3.u3Seen);
    CHECK(rs.u3Seen());
    CHECK_EQ_U(rs.rxBitmap(), 0x0Fu);
    CHECK_EQ_U(rs.u3EndUs(), kT0 + 3u * 22000u + 17038u);
    CHECK_EQ_U(rs.u3DeadlineUs(), kT0 + 3u * 22000u + 17038u + kJitterUs);
    CHECK_EQ_U(rs.rxCountAtPos(0u), 1u);
    CHECK_EQ_U(rs.rxCountAtPos(3u), 1u);
}

//---------------------------------------------------------------------------------------------------
// 4. D0 正常路径(pos3 + guard) / 每轮最多 1 帧
//---------------------------------------------------------------------------------------------------
void testD0Pos3Path()
{
    RoundSlave rs;
    CHECK(rs.init(makeParams()));
    feed(rs, FrameType::UpRound, 0u, kUpRoundBytes, rawEndFor(kT0, 0u, kUpRoundBytes), 0x22u);
    feed(rs, FrameType::UpCtrl, 3u, kUpCtrlBytes, rawEndFor(kT0, 3u, kUpCtrlBytes), 0u);

    const uint32_t sendAt = kT0 + 3u * 22000u + 17038u + kGuardUs; // = t0+86038

    // 早 1us: 不能发
    RoundSlave::TxDecision d = rs.decide(sendAt - 1u, true, kDlAttBytes);
    CHECK(!d.send);
    CHECK(d.skip == RoundSkipReason::NotDue);

    // 正好到点: 可以发, 路径 = POS3, sendAtUs 精确
    d = rs.decide(sendAt, true, kDlAttBytes);
    CHECK(d.send);
    CHECK(d.path == RoundTxPath::Pos3);
    CHECK(!d.isHello);
    CHECK_EQ_U(d.sendAtUs, sendAt);
    CHECK(rs.budgetAllows(sendAt, kDlAttBytes));

    // 预算边界(公式: airtime + len×174 + 1000): 21B 需要 26336us, 在 deadline 前刚好放得下 /
    // 差 1us 就放不下
    const uint32_t dlDeadline = kT0 + kDlTimeoutMs * 1000u;
    CHECK_EQ_U(budgetNeed(kUpCtrlBytes), 17038u + 9u * 174u + 1000u);
    CHECK_EQ_U(budgetNeed(kDlAttBytes), 18586u + 13u * 174u + 1000u);
    CHECK_EQ_U(budgetNeed(kDlGpsBytes), 21682u + 21u * 174u + 1000u);
    CHECK_EQ_U(budgetNeed(kDlGpsBytes), 26336u);
    CHECK(rs.budgetAllows(dlDeadline - budgetNeed(kDlGpsBytes), kDlGpsBytes));
    CHECK(!rs.budgetAllows(dlDeadline - budgetNeed(kDlGpsBytes) + 1u, kDlGpsBytes));

    // 记发送: 本轮闭合, 再问就不能再发
    rs.onDownlinkSent(false, d.path, sendAt, kDlAttBytes);
    CHECK_EQ_U(rs.d0SentCount(), 1u);
    CHECK_EQ_U(rs.lastSendJitterUs(), 0u);
    CHECK(!rs.roundActive());
    RoundSlave::TxDecision d2 = rs.decide(sendAt + 1000u, true, kDlAttBytes);
    CHECK(!d2.send);
    CHECK(d2.skip == RoundSkipReason::AlreadySent);

    // 下一轮 pos0(新 t0)后重新可发
    const uint32_t t0b = kT0 + kRoundMs * 1000u;
    feed(rs, FrameType::UpRound, 4u, kUpRoundBytes, rawEndFor(t0b, 0u, kUpRoundBytes), 0x22u);
    feed(rs, FrameType::UpCtrl, 7u, kUpCtrlBytes, rawEndFor(t0b, 3u, kUpCtrlBytes), 0u);
    CHECK(rs.decide(t0b + 3u * 22000u + 17038u + kGuardUs, true, kDlAttBytes).send);
}

//---------------------------------------------------------------------------------------------------
// 5. D0 兜底路径(U3 丢失): t0 推算 deadline + guard = t0+89038
//---------------------------------------------------------------------------------------------------
void testD0FallbackPath()
{
    RoundSlave rs;
    CHECK(rs.init(makeParams()));
    feed(rs, FrameType::UpRound, 0u, kUpRoundBytes, rawEndFor(kT0, 0u, kUpRoundBytes), 0x33u);
    feed(rs, FrameType::UpCtrl, 1u, kUpCtrlBytes, rawEndFor(kT0, 1u, kUpCtrlBytes), 0u);
    feed(rs, FrameType::UpCtrl, 2u, kUpCtrlBytes, rawEndFor(kT0, 2u, kUpCtrlBytes), 0u);
    CHECK(!rs.u3Seen());

    const uint32_t fallback = kT0 + 3u * 22000u + 17038u + kJitterUs + kGuardUs; // t0+89038
    RoundSlave::TxDecision d = rs.decide(fallback - 1u, true, kDlAttBytes);
    CHECK(!d.send);
    CHECK(d.skip == RoundSkipReason::NotDue);

    d = rs.decide(fallback, true, kDlAttBytes);
    CHECK(d.send);
    CHECK(d.path == RoundTxPath::Fallback);
    CHECK_EQ_U(d.sendAtUs, fallback);

    // 兜底路径的实际时刻: t0 + 3×22ms + F(UP_CTRL 9B) + jitter + guard = t0+89038
    CHECK_EQ_U(fallback - kT0, 89038u);

    // 预算检查: 21B 帧在兜底时刻仍放得下:
    //   89038 + F(21)=21682 = 110720(空中结束); + 21×174=3654 串口收完 = 114374 < 117000;
    //   预算余量 27962 − 26336 = 1626us(>0, 够用)
    CHECK(rs.budgetAllows(fallback, kDlGpsBytes));

    rs.onDownlinkSent(false, d.path, fallback, kDlAttBytes);
    CHECK_EQ_U(rs.d0SentCount(), 1u);
    CHECK_EQ_U(rs.fallbackRoundCount(), 1u);
}

//---------------------------------------------------------------------------------------------------
// 6. 发送前检查: AUX 忙 / 预算不足 → 放弃本轮, 且不再重试
//---------------------------------------------------------------------------------------------------
void testSendChecks()
{
    RoundSlave rs;
    CHECK(rs.init(makeParams()));
    feed(rs, FrameType::UpRound, 0u, kUpRoundBytes, rawEndFor(kT0, 0u, kUpRoundBytes), 0x44u);
    feed(rs, FrameType::UpCtrl, 3u, kUpCtrlBytes, rawEndFor(kT0, 3u, kUpCtrlBytes), 0u);

    const uint32_t sendAt = kT0 + 3u * 22000u + 17038u + kGuardUs;

    // AUX 忙: 放弃
    RoundSlave::TxDecision d = rs.decide(sendAt, false, kDlAttBytes);
    CHECK(!d.send);
    CHECK(d.skip == RoundSkipReason::AuxBusy);
    rs.onDownlinkSkipped(d.skip);
    CHECK_EQ_U(rs.d0SkipAuxCount(), 1u);
    CHECK(!rs.roundActive());
    CHECK(!rs.decide(sendAt + 100u, true, kDlAttBytes).send); // 本轮不再重试

    // 预算不足: 新一轮里拖到 deadline 之后才被调度
    const uint32_t t0b = kT0 + kRoundMs * 1000u;
    feed(rs, FrameType::UpRound, 4u, kUpRoundBytes, rawEndFor(t0b, 0u, kUpRoundBytes), 0x44u);
    feed(rs, FrameType::UpCtrl, 7u, kUpCtrlBytes, rawEndFor(t0b, 3u, kUpCtrlBytes), 0u);
    const uint32_t lateNow = t0b + kDlTimeoutMs * 1000u - 1000u; // 只剩 1ms, 21B 需要 26.34ms
    d = rs.decide(lateNow, true, kDlGpsBytes);
    CHECK(!d.send);
    CHECK(d.skip == RoundSkipReason::Budget);
    rs.onDownlinkSkipped(d.skip);
    CHECK_EQ_U(rs.d0SkipBudgetCount(), 1u);

    // auxGate=false 时忽略 AUX(但仍要过预算)
    RoundParams p  = makeParams();
    p.auxGate      = false;
    RoundSlave rs2;
    CHECK(rs2.init(p));
    feed(rs2, FrameType::UpRound, 0u, kUpRoundBytes, rawEndFor(kT0, 0u, kUpRoundBytes), 0x44u);
    feed(rs2, FrameType::UpCtrl, 3u, kUpCtrlBytes, rawEndFor(kT0, 3u, kUpCtrlBytes), 0u);
    d = rs2.decide(sendAt, false, kDlAttBytes);
    CHECK(d.send);
}

//---------------------------------------------------------------------------------------------------
// 7. HELLO: ACQUIRE 尽快回 / 版本不一致也回 / nonce 回显 / RUN 忙时占 D0 槽
//---------------------------------------------------------------------------------------------------
void testHello()
{
    RoundSlave rs;
    CHECK(rs.init(makeParams()));

    const uint32_t helloRaw  = 600000u;
    // HELLO 5B: 收帧末修正 = (5 + rxIdleBytes)×174
    const uint32_t helloCorr = (uint32_t)(kHelloBytes + kRxIdleBytes) * kUartByteUs;
    rs.onHelloRequest(2u, hello::kCapsGnss, 0x5Au, helloRaw);
    CHECK(rs.helloReplyPending());
    CHECK(rs.peerVersionMatch());
    CHECK_EQ_U(rs.peerVersion(), 2u);
    CHECK_EQ_U(rs.helloEarliestUs(), helloRaw - helloCorr + kGuardUs);

    // 未到 earliest 不发; 到了发(HELLO 应答不受 D0 预算约束, 因为它没有本轮 t0)
    RoundSlave::TxDecision d = rs.decide(helloRaw - helloCorr + kGuardUs - 1u, true, kHelloBytes);
    CHECK(!d.send);
    d = rs.decide(helloRaw - helloCorr + kGuardUs, true, kHelloBytes);
    CHECK(d.send);
    CHECK(d.isHello);
    CHECK(d.path == RoundTxPath::HelloAcquire);

    const HelloPayload reply = rs.takeHelloReply();
    CHECK_EQ_U(reply.ver, 2u);
    CHECK_EQ_U(reply.nonce, 0x5Au);
    CHECK_EQ_U(reply.caps, hello::kCapsCmdId); // 本端能力, 不是对端

    rs.onDownlinkSent(true, d.path, helloRaw, kHelloBytes);
    CHECK(!rs.helloReplyPending());
    CHECK_EQ_U(rs.helloReplyCount(), 1u);
    CHECK_EQ_U(rs.d0SentCount(), 0u); // ACQUIRE 的 HELLO 不占 D0
    CHECK(rs.acquire());

    // 版本不一致: 仍然回 HELLO(让对方看到本端版本), 但 ACQUIRE 不进 RUN
    rs.onHelloRequest(3u, 0x01u, 0x11u, helloRaw + 1000u);
    CHECK(rs.helloReplyPending());
    CHECK(rs.peerVersionMismatch());
    CHECK_EQ_U(rs.versionMismatchCount(), 1u);
    CHECK(rs.acquire());
    CHECK_EQ_U(rs.takeHelloReply().ver, 2u);
    CHECK_EQ_U(rs.takeHelloReply().nonce, 0x11u);

    // 同一请求重复: 允许重发(每 60ms 一次), 但 helloMax 次后丢弃(不会永远挂着)
    RoundSlave rs3;
    CHECK(rs3.init(makeParams()));
    rs3.onHelloRequest(2u, 0u, 0x01u, 100000u);
    for (uint8_t i = 0u; i < 5u; ++i) {
        rs3.onDownlinkSkipped(RoundSkipReason::AuxBusy);
    }
    CHECK(!rs3.helloReplyPending());
    CHECK_EQ_U(rs3.helloDroppedCount(), 1u);

    // RUN 且本轮忙: HELLO 不在 ACQUIRE 路径抢发, 而是占 D0 槽(优先级 #1)
    RoundSlave rs4;
    CHECK(rs4.init(makeParams()));
    feed(rs4, FrameType::UpRound, 0u, kUpRoundBytes, rawEndFor(kT0, 0u, kUpRoundBytes), 0x55u);
    feed(rs4, FrameType::UpCtrl, 2u, kUpCtrlBytes, rawEndFor(kT0, 2u, kUpCtrlBytes), 0u);
    rs4.onHelloRequest(2u, 0u, 0x77u, kT0 + 40000u);
    RoundSlave::TxDecision d4 = rs4.decide(kT0 + 50000u, true, kHelloBytes);
    CHECK(!d4.send); // 本轮还没到 D0 时刻; RUN 忙时不单独发 HELLO
    const uint32_t fallback = kT0 + 89038u;
    d4 = rs4.decide(fallback, true, kHelloBytes);
    CHECK(d4.send);
    CHECK(d4.isHello);
    CHECK(d4.path == RoundTxPath::Fallback);
    rs4.onDownlinkSent(d4.isHello, d4.path, fallback, kHelloBytes);
    CHECK_EQ_U(rs4.helloReplyCount(), 1u);
    // 规格 §6: HELLO 应答占用 D0 槽, 但**不推进任何遥测计数**(d0Sent/fallback 都只统计遥测帧)
    CHECK_EQ_U(rs4.d0SentCount(), 0u);
    CHECK_EQ_U(rs4.fallbackRoundCount(), 0u);
    CHECK(rs4.roundClosed()); // 但本轮槽已用掉, 不能再发第二帧
    CHECK(!rs4.decide(fallback + 1000u, true, kDlAttBytes).send);
}

//---------------------------------------------------------------------------------------------------
// 7b. 回归: RUN + 无活动轮次 + helloPending ⇒ HELLO 应答不得被 D0 预算误杀
//     (早先判据是 path != HelloAcquire, 于是 RUN 空闲期走 HelloInRound 也会跑 budgetAllows();
//      而 budgetAllows() 在 m_haveRound==false 时直接返回 false ⇒ 应答要等 200ms 退回 ACQUIRE)
//---------------------------------------------------------------------------------------------------
void testHelloInRunIdle()
{
    RoundSlave rs;
    CHECK(rs.init(makeParams()));

    // 先进 RUN 并跑完一轮(发掉 D0) ⇒ RUN + 本轮已关闭(m_haveRound==false), 但仍在 200ms 内
    feed(rs, FrameType::UpRound, 0u, kUpRoundBytes, rawEndFor(kT0, 0u, kUpRoundBytes), 0x21u);
    feed(rs, FrameType::UpCtrl, 3u, kUpCtrlBytes, rawEndFor(kT0, 3u, kUpCtrlBytes), 0u);
    const uint32_t        d0 = kT0 + 3u * 22000u + 17038u + kGuardUs;
    RoundSlave::TxDecision d = rs.decide(d0, true, kDlAttBytes);
    CHECK(d.send);
    rs.onDownlinkSent(false, d.path, d0, kDlAttBytes);
    CHECK(rs.run());
    CHECK(!rs.roundActive());
    CHECK(!rs.poll(d0 + 1000u).enteredAcquire); // 确认还在 RUN

    // RUN 空闲期收到 HELLO 请求: 应答应尽快(helloEnd+guard), 不受本轮预算约束
    const uint32_t helloEnd = d0 + 1000u;
    rs.onHelloRequest(2u, 0u, 0x42u, helloEnd);
    const uint32_t earliest = helloEnd - (kHelloBytes + kRxIdleBytes) * kUartByteUs + kGuardUs;

    RoundSlave::TxDecision hd = rs.decide(earliest - 1u, true, kHelloBytes);
    CHECK(!hd.send); // 还没到 helloEnd+guard

    hd = rs.decide(earliest, true, kHelloBytes);
    CHECK(hd.send);
    CHECK(hd.isHello);
    CHECK(hd.path == RoundTxPath::HelloInRound); // RUN 状态(不是 ACQUIRE)
    CHECK(hd.skip != RoundSkipReason::Budget);   // 回归点: 不被预算误杀

    // AUX 检查对 HELLO 应答同样生效(§2.3 第 1 条)
    const RoundSlave::TxDecision hd2 = rs.decide(earliest, false, kHelloBytes);
    CHECK(!hd2.send);
    CHECK(hd2.skip == RoundSkipReason::AuxBusy);

    // 发出后: pending 清、只进 helloReplies; 那一帧是 HELLO, 不算 D0
    rs.onDownlinkSent(hd.isHello, hd.path, earliest, kHelloBytes);
    CHECK(!rs.helloReplyPending());
    CHECK_EQ_U(rs.helloReplyCount(), 1u);
    CHECK_EQ_U(rs.d0SentCount(), 1u); // 只有前面那帧真 D0
    CHECK_EQ_U(rs.fallbackRoundCount(), 0u);
}

//---------------------------------------------------------------------------------------------------
// 8. session 变化 / poll 回 ACQUIRE
//---------------------------------------------------------------------------------------------------
void testSessionAndPoll()
{
    RoundSlave rs;
    CHECK(rs.init(makeParams()));
    feed(rs, FrameType::UpRound, 0u, kUpRoundBytes, rawEndFor(kT0, 0u, kUpRoundBytes), 0x66u);
    CHECK_EQ_U(rs.sessionChangeCount(), 1u);

    // 同 session: 不算变化
    feed(rs, FrameType::UpRound, 1u, kUpRoundBytes, rawEndFor(kT0, 1u, kUpRoundBytes), 0x66u);
    CHECK_EQ_U(rs.sessionChangeCount(), 1u);

    // 新 session: 变化 + 重新建轮
    const uint32_t t0b = kT0 + 30000u;
    const RoundFrameResult r =
        feed(rs, FrameType::UpRound, 0u, kUpRoundBytes, rawEndFor(t0b, 0u, kUpRoundBytes), 0x67u);
    CHECK(r.sessionChanged);
    CHECK_EQ_U(rs.sessionChangeCount(), 2u);
    CHECK_EQ_U(rs.session(), 0x67u);
    CHECK_EQ_U(rs.roundT0Us(), t0b);
    CHECK_EQ_U(rs.rxBitmap(), 0x01u); // 新会话的位图从这一帧重新开始

    // poll: 200ms 内不回 ACQUIRE; 到点回
    RoundPollResult pr = rs.poll(rs.lastFrameEndUs() + kAcquireMs * 1000u - 1u);
    CHECK(!pr.enteredAcquire);
    CHECK(rs.run());
    pr = rs.poll(rs.lastFrameEndUs() + kAcquireMs * 1000u);
    CHECK(pr.enteredAcquire);
    CHECK(rs.acquire());
    CHECK(!rs.haveSession());
    CHECK_EQ_U(rs.acquireEntryCount(), 1u);

    // 回 ACQUIRE 后 0x8–0xD 又被忽略, 直到新的 UP_ROUND
    CHECK(!feed(rs, FrameType::UpCtrl, 1u, kUpCtrlBytes, rawEndFor(t0b, 1u, kUpCtrlBytes)).accepted);
    CHECK(rs.acquire());
}

//---------------------------------------------------------------------------------------------------
// 9. 锚点回归(收帧末按帧长修正): 给定真值 t0 构造 rawIdle, 反推误差 ≤1us (n=5/9/11)
//    rawIdle = t0 + pos×22ms + F(n) + (n + rxIdleBytes)×174
//---------------------------------------------------------------------------------------------------
void testTimestampCorrection()
{
    RoundSlave rs;
    CHECK(rs.init(makeParams()));

    struct Case
    {
        uint16_t len;
        uint8_t  pos;
    };
    // n=5 / 9 / 11(HELLO / UP_CTRL / UP_ROUND)。老实现只减固定 174us 时, n=11 会偏大 1.9ms。
    const Case cases[3] = {{kHelloBytes, 0u}, {kUpCtrlBytes, 1u}, {kUpRoundBytes, 2u}};
    for (uint8_t i = 0u; i < 3u; ++i) {
        const uint32_t      t0  = kT0 + (uint32_t)i * 1000u;
        const RoundParams   p   = makeParams();
        const uint32_t      raw = (uint32_t)(t0 + (uint32_t)cases[i].pos * kSpacingMs * 1000u +
                                             roundAirtimeUs(p, cases[i].len) +
                                             (uint32_t)(cases[i].len + kRxIdleBytes) * kUartByteUs);
        const uint32_t      got = rs.reconstructT0Us(raw, cases[i].pos, cases[i].len);
        // 误差 ≤1us(全整数运算, 实际为 0)
        CHECK(got >= t0 ? (got - t0) <= 1u : (t0 - got) <= 1u);
        CHECK_EQ_U(got, t0);
    }

    // 真喂一帧: onFrame 内部必须用同一模型(UP_ROUND pos0, 11B)
    const uint32_t    t0  = kT0;
    const RoundParams p   = makeParams();
    const uint32_t    raw = (uint32_t)(t0 + roundAirtimeUs(p, kUpRoundBytes) +
                                       (uint32_t)(kUpRoundBytes + kRxIdleBytes) * kUartByteUs);
    feed(rs, FrameType::UpRound, 0u, kUpRoundBytes, raw, 0x77u);
    CHECK_EQ_U(rs.roundT0Us(), t0);

    // 注入 rxIdleBytes=2: 同一 rawIdle 反推的 t0 小 174us —— 证明修正是"按帧长/按 rxIdleBytes",
    // 不是一个写死的常数
    RoundParams pb = makeParams();
    pb.rxIdleBytes = 2u;
    RoundSlave b;
    CHECK(b.init(pb));
    CHECK_EQ_U(b.reconstructT0Us(raw, 0u, kUpRoundBytes), (uint32_t)(t0 - kUartByteUs));
}

//---------------------------------------------------------------------------------------------------
// 9b. 本轮关闭(规格 §4): D0 之后的迟到 U3 只计数, 不得触发第二帧 D0
//---------------------------------------------------------------------------------------------------
void testRoundClosedLateFrame()
{
    RoundSlave rs;
    CHECK(rs.init(makeParams()));
    feed(rs, FrameType::UpRound, 0u, kUpRoundBytes, rawEndFor(kT0, 0u, kUpRoundBytes), 0x77u);
    feed(rs, FrameType::UpCtrl, 1u, kUpCtrlBytes, rawEndFor(kT0, 1u, kUpCtrlBytes), 0u);
    feed(rs, FrameType::UpCtrl, 2u, kUpCtrlBytes, rawEndFor(kT0, 2u, kUpCtrlBytes), 0u);

    const uint32_t fallback = kT0 + 89038u;
    RoundSlave::TxDecision d = rs.decide(fallback, true, kDlAttBytes);
    CHECK(d.send);
    CHECK(d.path == RoundTxPath::Fallback);
    rs.onDownlinkSent(false, d.path, fallback, kDlAttBytes);
    CHECK(rs.roundClosed());
    CHECK_EQ_U(rs.d0SentCount(), 1u);

    // U3 迟到 10ms(D0 之后才到): 反推 t0 与记录值只差 10ms(<60ms) ⇒ 只计数
    const RoundFrameResult late =
        feed(rs, FrameType::UpCtrl, 3u, kUpCtrlBytes, rawEndFor(kT0, 3u, kUpCtrlBytes) + 10000u);
    CHECK(late.accepted);
    CHECK(late.lateIgnored);
    CHECK(!late.u3Seen);
    CHECK(!rs.u3Seen());                 // 不得回填 u3End(否则会算出第二个 D0 时刻)
    CHECK_EQ_U(rs.lateFrameCount(), 1u);
    CHECK_EQ_U(rs.rxCountAtPos(3u), 1u); // 计数照常
    CHECK(rs.roundClosed());

    // 关键: 不能再发第二帧 D0
    RoundSlave::TxDecision d2 = rs.decide(fallback + 20000u, true, kDlAttBytes);
    CHECK(!d2.send);
    CHECK(d2.skip == RoundSkipReason::AlreadySent);
    CHECK_EQ_U(rs.d0SentCount(), 1u);

    // 下一轮 UP_ROUND 才开新轮
    const uint32_t t0b = kT0 + kRoundMs * 1000u;
    const RoundFrameResult nr =
        feed(rs, FrameType::UpRound, 4u, kUpRoundBytes, rawEndFor(t0b, 0u, kUpRoundBytes), 0x77u);
    CHECK(nr.accepted);
    CHECK(!nr.lateIgnored);
    CHECK_EQ_U(rs.roundT0Us(), t0b);
    CHECK(!rs.roundClosed());
    CHECK_EQ_U(rs.rxBitmap(), 0x01u); // 新轮位图从 0 开始

    // U0 丢了也能开新轮: 关闭后收到下一轮的 pos1, t0 变化 120ms >= 60ms
    RoundSlave rs2;
    CHECK(rs2.init(makeParams()));
    feed(rs2, FrameType::UpRound, 0u, kUpRoundBytes, rawEndFor(kT0, 0u, kUpRoundBytes), 0x11u);
    feed(rs2, FrameType::UpCtrl, 1u, kUpCtrlBytes, rawEndFor(kT0, 1u, kUpCtrlBytes), 0u);
    feed(rs2, FrameType::UpCtrl, 2u, kUpCtrlBytes, rawEndFor(kT0, 2u, kUpCtrlBytes), 0u);
    RoundSlave::TxDecision d3 = rs2.decide(fallback, true, kDlAttBytes);
    CHECK(d3.send);
    rs2.onDownlinkSent(false, d3.path, fallback, kDlAttBytes);
    CHECK(rs2.roundClosed());
    const RoundFrameResult next = feed(rs2, FrameType::UpCtrl, 5u, kUpCtrlBytes,
                                       rawEndFor(kT0 + kRoundMs * 1000u, 1u, kUpCtrlBytes));
    CHECK(next.accepted);
    CHECK(!next.lateIgnored); // t0 变化 120ms ⇒ 新轮(不是迟到帧)
    CHECK_EQ_U(rs2.roundT0Us(), kT0 + kRoundMs * 1000u);
    CHECK(!rs2.roundClosed());
}

//---------------------------------------------------------------------------------------------------
// 10. 下行选帧器: 优先级 / 到期 / ACK 队列
//---------------------------------------------------------------------------------------------------
DownlinkParamsV2 makeDlParams()
{
    DownlinkParamsV2 p = {};
    p.stateEvery       = 4u;
    p.sysEvery         = 4u;
    p.sysOffset        = 2u;
    p.gpsEvery         = 16u;
    p.gpsEnabled       = false;
    return p;
}

void testDownlinkSelector()
{
    DownlinkSelector sel;
    DownlinkParamsV2 p = makeDlParams();
    CHECK(sel.init(p));
    CHECK(sel.initialized());

    // 非法参数
    DownlinkParamsV2 bad = p;
    bad.stateEvery = 0u;
    CHECK(!DownlinkSelector().init(bad));
    bad = p;
    bad.sysOffset = 4u;
    CHECK(!DownlinkSelector().init(bad));

    // 无 HELLO/ACK 时的到期顺序: round0 STATE, round1 ATT, round2 SYS, round3 ATT, round4 STATE ...
    CHECK(sel.pick(false, false).kind == DlPickKind::State);
    sel.consume(sel.pick(false, false));
    CHECK_EQ_U(sel.sentRounds(), 1u);
    CHECK(sel.pick(false, false).kind == DlPickKind::Att);
    sel.consume(sel.pick(false, false));
    CHECK(sel.pick(false, false).kind == DlPickKind::Sys);
    sel.consume(sel.pick(false, false));
    CHECK(sel.pick(false, false).kind == DlPickKind::Att);
    sel.consume(sel.pick(false, false));
    CHECK(sel.pick(false, false).kind == DlPickKind::State); // 第 4 轮
    sel.consume(sel.pick(false, false));
    CHECK_EQ_U(sel.sentRounds(), 5u);
    CHECK_EQ_U(sel.pickCount(DlPickKind::State), 2u);
    CHECK_EQ_U(sel.pickCount(DlPickKind::Sys), 1u);
    CHECK_EQ_U(sel.pickCount(DlPickKind::Att), 2u);

    // HELLO 优先级最高, 且不推进轮号
    const uint32_t roundsBefore = sel.sentRounds();
    CHECK(sel.pick(true, false).kind == DlPickKind::Hello);
    sel.consume(sel.pick(true, false));
    CHECK_EQ_U(sel.sentRounds(), roundsBefore);

    // ACK 次优先, 按产生顺序, 不推进轮号
    CHECK(sel.pushAck(7u, 0u));
    CHECK(sel.pushAck(8u, 3u));
    CHECK_EQ_U(sel.pendingAckCount(), 2u);
    DlPick a = sel.pick(false, false);
    CHECK(a.kind == DlPickKind::Ack);
    CHECK_EQ_U(a.ack.cmdId, 7u);
    CHECK_EQ_U(a.ack.result, 0u);
    sel.consume(a);
    a = sel.pick(false, false);
    CHECK(a.kind == DlPickKind::Ack);
    CHECK_EQ_U(a.ack.cmdId, 8u);
    CHECK_EQ_U(a.ack.result, 3u);
    sel.consume(a);
    CHECK_EQ_U(sel.pendingAckCount(), 0u);
    CHECK_EQ_U(sel.sentRounds(), roundsBefore); // ACK 不占轮次

    // ACK 容量 4: 第 5 条挤掉最老的
    for (uint8_t i = 1u; i <= 5u; ++i) {
        CHECK(sel.pushAck(i, 0u));
    }
    CHECK_EQ_U(sel.pendingAckCount(), 4u);
    CHECK_EQ_U(sel.ackDroppedCount(), 1u);
    a = sel.pick(false, false);
    CHECK_EQ_U(a.ack.cmdId, 2u); // cmdId=1 被挤掉
    sel.consume(a);

    // session 变化: 清 ACK 队列 + 节拍归零
    sel.onSessionChanged();
    CHECK_EQ_U(sel.pendingAckCount(), 0u);
    CHECK_EQ_U(sel.sentRounds(), 0u);

    // GPS: 必须"本端开 + 对端想要"才选; 且每 16 轮一次
    sel.init(p);
    CHECK(sel.pick(false, true).kind != DlPickKind::Gps); // gpsEnabled=false
    p.gpsEnabled = true;
    CHECK(sel.init(p));
    CHECK(sel.pick(false, false).kind != DlPickKind::Gps); // 对端不想要
    DlPick g = sel.pick(false, true);
    CHECK(g.kind == DlPickKind::Gps); // round 0 到期
    sel.consume(g);
    for (uint16_t i = 1u; i < 16u; ++i) {
        sel.consume(sel.pick(false, true));
    }
    CHECK_EQ_U(sel.sentRounds(), 16u);
    CHECK(sel.pick(false, true).kind == DlPickKind::Gps); // 第 16 轮又到期
    CHECK_EQ_U(sel.pickCount(DlPickKind::Gps), 1u);
}

} // namespace

int main()
{
    testParams();
    testAcquire();
    testT0Reconstruction();
    testD0Pos3Path();
    testD0FallbackPath();
    testSendChecks();
    testHello();
    testHelloInRunIdle();
    testSessionAndPoll();
    testTimestampCorrection();
    testRoundClosedLateFrame();
    testDownlinkSelector();

    printf("\nchecks=%d failures=%d\n", g_checks, g_failures);
    if (g_failures != 0) {
        printf("RESULT: FAIL\n");
        return 1;
    }
    printf("RESULT: PASS\n");
    return 0;
}
