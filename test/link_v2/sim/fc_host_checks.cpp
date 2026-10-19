//===================================================================================================
// test/link_v2/sim/fc_host_checks.cpp —— 飞控 RoundSlave 口径断言（验证者独立主机检查）
//
// 只调用被测实现 DL_LIB/link/dlx_link_round.hpp（不修改它），覆盖：
//   H0 P0 接收模型锚点回归：airEnd = rawIdle − (frameBytes + LinkRxIdleBytes)×uartByteUs，
//      reconstructT0 对 n=5/6/9/11 与真值误差 ≤1us（P0 实测 IDLE ≈ 空中结束 + n×174 + 174）
//   H1 ACQUIRE 收到 HELLO：应答时刻 = helloEnd+guard，只计 helloReplies，不计 D0
//   H2 RUN 本轮忙时 HELLO 应答占用 D0 槽：只计 helloReplies，不计 d0Sent/fallbackRounds，
//      但本轮仍必须关闭（否则同轮会再发第二帧）
//   H3 普通遥测 D0：计 d0Sent，不计 helloReplies
//   H4 本轮关闭：D0 后同 t0 的迟到帧只计数、不再触发发送
//   H5 预算公式：now + airtime + n×uartByteUs + slack ≤ t0+117ms
// 构建: g++ -std=c++11 -O2 -I test/stub -I DL_LIB -I . test/link_v2/sim/fc_host_checks.cpp -o fc_host_checks
//===================================================================================================
#include <cstdio>
#include <cstdint>
#include "link/dlx_link_round.hpp"
#include "app/link/link_v2_config.hpp"

using namespace dlx::link;

static int g_fail = 0;
static void check(const char *name, bool ok, const char *detail = "")
{
    std::printf("%-58s %s%s\n", name, ok ? "PASS" : "FAIL", ok ? "" : "  <= ");
    if (!ok) {
        std::printf("    %s\n", detail);
        ++g_fail;
    }
}

static RoundParams params()
{
    RoundParams p{};
    p.roundMs          = LinkRoundMs;
    p.frameSpacingMs   = LinkFrameSpacingMs;
    p.guardUs          = LinkGuardUs;
    p.u3JitterUs       = LinkU3JitterUs;
    p.dlTimeoutMs      = LinkDlTimeoutMs;
    p.acquireMs        = LinkAcquireMs;
    p.airtimeBaseUs    = LinkAirtimeBaseUs;
    p.airtimePerByteUs = LinkAirtimePerByteUs;
    p.uartByteUs       = LinkUartByteUs;
    p.dlBudgetSlackUs  = LinkDlBudgetSlackUs;
    p.roundReopenUs    = 60000u;
    p.rxIdleBytes      = LinkRxIdleBytes;   ///< P0 实测: 帧末 IDLE 比空中结束多 (n+1) 个字符
    p.protoVersion     = (uint8_t)LinkProtoVersion;
    p.caps             = 0u;
    p.helloMax         = (uint8_t)LinkHelloMax;
    p.auxGate          = false;
    return p;
}

static uint16_t airtime(uint16_t bytes)
{
    return (uint16_t)(LinkAirtimeBaseUs + LinkAirtimePerByteUs * bytes);
}

/** 把"真实空中结束时刻"换算成实现会看到的原始 USART IDLE 回调时刻 */
static uint32_t raw_idle_end(uint32_t airEndUs, uint16_t bytes)
{
    return (uint32_t)(airEndUs + (uint32_t)(bytes + LinkRxIdleBytes) * LinkUartByteUs);
}

/** 兜底 D0 的最早发送时刻（规格 §2.1/E4）：t0 + 3·spacing + F(UP_CTRL) + U3jitter + guard */
static uint32_t fallback_send_at(uint32_t t0)
{
    return t0 + 3u * LinkFrameSpacingMs * 1000u +
           airtime((uint16_t)frameBytes(FrameType::UpCtrl)) + LinkU3JitterUs + LinkGuardUs;
}

int main()
{
    const RoundParams P = params();
    const uint16_t upRoundBytes = frameBytes(FrameType::UpRound);

    // ---- H0: P0 接收模型锚点回归（n=5/6/9/11）----
    {
        RoundSlave rs;
        check("init", rs.init(P));
        struct { FrameType t; uint8_t pos; } cases[] = {
            { FrameType::Hello,    0u },
            { FrameType::UpCmd,    2u },
            { FrameType::UpCtrl,   1u },
            { FrameType::UpRound,  0u },
        };
        bool all_ok = true;
        char detail[160] = {0};
        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
            const uint16_t n  = frameBytes(cases[i].t);
            const uint32_t t0 = 100000u + i * 1000u;
            const uint32_t airEnd = t0 + cases[i].pos * LinkFrameSpacingMs * 1000u + airtime(n);
            const uint32_t raw    = raw_idle_end(airEnd, n);
            const uint32_t back   = rs.airEndUs(raw, n);
            const uint32_t t0back = rs.reconstructT0Us(raw, cases[i].pos, n);
            if (back != airEnd || t0back != t0) {
                all_ok = false;
                std::snprintf(detail, sizeof(detail),
                              "n=%u airEnd %u->%u t0 %u->%u", (unsigned)n, airEnd, back, t0, t0back);
                break;
            }
        }
        check("H0 P0 t0 锚点回归(n=5/6/9/11, 误差≤1us)", all_ok, detail);
        // 旧口径（只减常数 174）会偏大 n×174us：这里显式确认不是那个行为
        const uint16_t n11 = frameBytes(FrameType::UpRound);
        const uint32_t raw11 = raw_idle_end(200000u, n11);
        check("H0 t0 不含 n×174 系统偏差",
              rs.reconstructT0Us(raw11, 0u, n11) == 200000u - airtime(n11));
    }

    // ---- H1: ACQUIRE 收 HELLO -> 立即应答（helloEnd+guard），只计 hello ----
    {
        RoundSlave rs;
        rs.init(P);
        const uint16_t helloBytes = frameBytes(FrameType::Hello);
        const uint32_t helloEnd = 100000u;                        // 真实空中结束
        rs.onHelloRequest((uint8_t)LinkProtoVersion, 0u, 0x5Au,
                          raw_idle_end(helloEnd, helloBytes));
        check("H1 HELLO 应答排队", rs.helloReplyPending());
        check("H1 应答最早时刻 = helloEnd+guard",
              rs.helloEarliestUs() == helloEnd + LinkGuardUs);
        RoundSlave::TxDecision d = rs.decide(helloEnd + LinkGuardUs - 1u, true, 5u);
        check("H1 未到时刻不发", !d.send);
        d = rs.decide(helloEnd + LinkGuardUs, true, 5u);
        check("H1 到时刻发 HELLO 应答", d.send && d.isHello && d.path == RoundTxPath::HelloAcquire);
        rs.onDownlinkSent(true, d.path, helloEnd + LinkGuardUs, 5u);
        check("H1 helloReplies=1", rs.helloReplyCount() == 1u);
        check("H1 D0 计数不推进", rs.d0SentCount() == 0u && rs.fallbackRoundCount() == 0u);
    }

    // ---- H2: RUN 本轮忙 -> HELLO 占 D0 槽；只计 hello，但本轮必须关闭 ----
    {
        RoundSlave rs;
        rs.init(P);
        const uint32_t u0End  = 200000u;                          // 真实空中结束
        RoundFrameResult rr = rs.onFrame(FrameType::UpRound, 0u, upRoundBytes,
                                         raw_idle_end(u0End, upRoundBytes), 0x3Cu);
        check("H2 收到 UP_ROUND 进 RUN", rr.accepted && rr.startedRun && rs.run());
        const uint32_t t0 = u0End - airtime(upRoundBytes);
        check("H2 t0 反推正确", rs.roundT0Us() == t0);
        // 本轮忙（已有 pos0，未发 D0）时收到 HELLO 请求
        rs.onHelloRequest((uint8_t)LinkProtoVersion, 0u, 0x11u,
                          raw_idle_end(t0 + 30000u, frameBytes(FrameType::Hello)));
        RoundSlave::TxDecision d = rs.decide(t0 + 33000u, true, 5u);
        check("H2 本轮忙时不提前发 HELLO", !d.send);
        d = rs.decide(fallback_send_at(t0), true, 5u);
        check("H2 到兜底时刻发 HELLO(占 D0 槽)",
              d.send && d.isHello && d.path == RoundTxPath::Fallback);
        rs.onDownlinkSent(true, d.path, fallback_send_at(t0), 5u);
        check("H2 helloReplies=1 且 d0Sent=0 且 fallback=0",
              rs.helloReplyCount() == 1u && rs.d0SentCount() == 0u &&
              rs.fallbackRoundCount() == 0u);
        check("H2 本轮仍被关闭", rs.roundClosed() && !rs.roundActive());
    }

    // ---- H3: 普通遥测 D0 只计 d0Sent ----
    {
        RoundSlave rs;
        rs.init(P);
        const uint32_t u0End = 300000u;
        const uint32_t t0    = u0End - airtime(upRoundBytes);
        rs.onFrame(FrameType::UpRound, 0u, upRoundBytes, raw_idle_end(u0End, upRoundBytes), 0x3Cu);
        RoundSlave::TxDecision d = rs.decide(fallback_send_at(t0), true, 13u);
        check("H3 兜底时刻可发 D0", d.send && !d.isHello && d.path == RoundTxPath::Fallback);
        rs.onDownlinkSent(false, d.path, fallback_send_at(t0), 13u);
        check("H3 d0Sent=1 fallback=1 hello=0",
              rs.d0SentCount() == 1u && rs.fallbackRoundCount() == 1u &&
              rs.helloReplyCount() == 0u);
        check("H3 本轮关闭", rs.roundClosed() && !rs.roundActive());
    }

    // ---- H4: 本轮关闭后同 t0 迟到帧只计数、不再触发发送 ----
    {
        RoundSlave rs;
        rs.init(P);
        const uint32_t u0End = 400000u;
        const uint32_t t0    = u0End - airtime(upRoundBytes);
        rs.onFrame(FrameType::UpRound, 0u, upRoundBytes, raw_idle_end(u0End, upRoundBytes), 0x3Cu);
        RoundSlave::TxDecision d = rs.decide(fallback_send_at(t0), true, 13u);
        rs.onDownlinkSent(false, d.path, fallback_send_at(t0), 13u);
        const uint16_t ctrlBytes = frameBytes(FrameType::UpCtrl);
        const uint32_t lateEnd = t0 + 22000u + airtime(ctrlBytes);   // 与当轮同 t0
        RoundFrameResult rr = rs.onFrame(FrameType::UpCtrl, 1u, ctrlBytes,
                                         raw_idle_end(lateEnd, ctrlBytes), 0u);
        check("H4 迟到帧被接受为统计", rr.accepted && rr.lateIgnored);
        check("H4 迟到帧不重开轮次", !rs.roundActive() && rs.roundClosed());
        check("H4 迟到计数=1", rs.lateFrameCount() == 1u);
        RoundSlave::TxDecision d2 = rs.decide(t0 + 94000u, true, 13u);
        check("H4 不再发第二帧 D0", !d2.send);
    }

    // ---- H5: 预算公式（含接收端串口排空 + 1ms 余量）----
    {
        RoundSlave rs;
        rs.init(P);
        const uint32_t u0End = 500000u;
        const uint32_t t0    = u0End - airtime(upRoundBytes);
        rs.onFrame(FrameType::UpRound, 0u, upRoundBytes, raw_idle_end(u0End, upRoundBytes), 0x3Cu);
        check("H5 21B 在 t0+95ms 应拒绝",
              !rs.budgetAllows(t0 + 95000u, (uint16_t)frameBytes(FrameType::DlGps)));
        check("H5 21B 在 t0+89ms 应允许",
              rs.budgetAllows(t0 + 89000u, (uint16_t)frameBytes(FrameType::DlGps)));
        check("H5 13B 在 t0+89ms 应允许",
              rs.budgetAllows(t0 + 89000u, (uint16_t)frameBytes(FrameType::DlAtt)));
    }

    if (g_fail == 0) {
        std::printf("\nFC RoundSlave 口径断言：全部 PASS\n");
        return 0;
    }
    std::printf("\nFC RoundSlave 口径断言：%d 项 FAIL\n", g_fail);
    return 1;
}
