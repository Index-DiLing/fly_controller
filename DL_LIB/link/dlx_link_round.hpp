#pragma once
//===================================================================================================
// DL_LIB/link/dlx_link_round.hpp —— 链路 v2 飞控端轮次从机(RoundSlave, 纯逻辑)
//
// 依据: docs/链路层v2_协议规格.md §2(轮次时序)、§4(飞控状态机)、§6(下行调度输入)、
//       §8(观测)。实施计划书 §13 E4/E5 指定: D0 触发以 **本轮 t0 锚定**, 不用"从末帧末尾累加"。
//
// 本文件做四件事(全是纯逻辑, 不读硬件、不 include app/ 或 RTOS):
//   1. ACQUIRE / RUN 两态状态机; ACQUIRE 只采信 UP_ROUND(这是"v1 遥控器不会误控制"的保证);
//   2. 由"收到哪一位置的帧 + 该帧末字节时刻"反推本轮 t0(§2.2/E5), 并维护 u3End/u3Deadline;
//   3. D0 发送时机判定: 收到 pos3 → u3End+guard(正常路径); 没收到 → t0 推算 deadline(兜底);
//      以及发送前检查(§2.3): AUX 忙、预算不足 → 放弃本轮;
//   4. HELLO 应答排队(双向同型帧; 版本不一致也回, 但 ACQUIRE 不进 RUN)。
//
// ---- 收帧末修正必须按**帧长**算(2026-10-18 P0 实测结论) ----
//   E220 是"整帧收完才开始往 UART 吐字节": 首字节出现在"空中结束 + 174us", 帧末 IDLE 出现在
//   "空中结束 + n×174us + 174us"。所以:
//       airEnd = rawIdle − (frameBytes + rxIdleBytes) × uartByteUs      (rxIdleBytes 实测 1)
//   若像早先那样只减一个固定 174us(只减了 IDLE 那一个字符), 就会**少减 n×174us**:
//   n=11 时 t0 系统性偏大约 1.9ms, 而兜底 D0 的预算余量本来只有几毫秒 —— 直接把余量吃掉一大半。
//   所以 RoundSlave 吃"原始 IDLE 回调时刻", 自己按帧长换算; 台架标定只需要改 rxIdleBytes(通常 1),
//   主机测试可以注入不同的 rxIdleBytes 并断言 t0 精确平移。
//
// ---- 为什么不用 v1 的 dlx_link_sched.hpp ----
//   v1 的 SchedSlave 维护"绝对时间槽网格/锚点", 丢帧会累积相位误差; v2 的位置信息直接编码在
//   SEQ&3 里(帧计数轮次), 时间只用来算 D0 何时发。两者模型不同, 混用会把两套"同步"定义塞进
//   一个对象(实测中最难查的一类 bug)。v1 文件保留不动, 本文件不 include 它。
//
// C++11 / 无异常 / 无 RTTI / 无动态分配 / header-only。
//===================================================================================================
#include <stdint.h>

#include "dlx_link_frame.hpp" // 同目录: FrameType/帧长/seqPos/HelloPayload

namespace dlx
{
namespace link
{

    //===============================================================================================
    // 一、参数(由调用方从 app/link/link_v2_config.hpp 填; 本文件不引用任何配置头)
    //===============================================================================================
    /** 轮次从机参数。**全部字段必填**(没有默认值): 轮长/帧距/guard 是两端必须一致的约定,
     *  让编译器替调用方挑默认值, 等于把"两端不一致"变成静默行为。 */
    struct RoundParams
    {
        uint32_t roundMs;            ///< 一轮总长 [ms](LinkRoundMs: 120)
        uint32_t frameSpacingMs;     ///< 上行帧起点间距 [ms](LinkFrameSpacingMs: 22)
        uint32_t guardUs;            ///< 末字节 → 允许换向发送 [us](LinkGuardUs: 3000)
        uint32_t u3JitterUs;         ///< U3 未到时, 期望末字节之后再等的抖动余量 [us](3000)
        uint32_t dlTimeoutMs;        ///< 遥控器等本轮下行的时间 [ms](LinkDlTimeoutMs: 117)
        uint32_t acquireMs;          ///< 多久无合法上行 → 回 ACQUIRE [ms](LinkAcquireMs: 200)
        uint32_t airtimeBaseUs;      ///< F(n) = base + perByte*n
        uint32_t airtimePerByteUs;
        uint32_t uartByteUs;         ///< 每字节串口时间 [us](57600 8N1 ≈ 174): 对端还要把整帧收完
        uint32_t dlBudgetSlackUs;    ///< D0 预算的额外调度余量 [us](1000)
        uint32_t roundReopenUs;      ///< 本轮关闭后, 反推 t0 差 ≥ 此值才开新轮 [us](60000, 规格 §4)
        uint32_t rxIdleBytes;        ///< 帧末 IDLE 判定额外占用的字符数(P0 实测: 1)
        uint8_t  protoVersion;       ///< 本端协议版本(填进 HELLO.ver)
        uint8_t  caps;               ///< 本端能力位(填进 HELLO.caps)
        uint8_t  helloMax;           ///< 一次 HELLO 应答最多尝试几次(LinkHelloMax: 5)
        bool     auxGate;            ///< true: 发送前要求 AUX 空闲(LinkAuxGate)
    };

    inline bool roundParamsValid(const RoundParams &p)
    {
        if (p.roundMs == 0u || p.frameSpacingMs == 0u) {
            return false;
        }
        // 4 帧上行必须整体落在轮次内, 且 D0 窗口在下一轮之前(与 link_v2_config.hpp 的守卫同义)
        if ((uint64_t)p.frameSpacingMs * 4u >= (uint64_t)p.roundMs) {
            return false;
        }
        if (p.guardUs == 0u || p.guardUs > 8000u) {
            return false;
        }
        if (p.dlTimeoutMs == 0u || p.dlTimeoutMs >= p.roundMs) {
            return false;
        }
        if (p.acquireMs == 0u) {
            return false;
        }
        if (p.airtimeBaseUs == 0u || p.airtimePerByteUs == 0u) {
            return false;
        }
        // 串口排空时间必须小于一帧空口时间(否则单位写错; 174us/字节 vs F(n) 的 387us/字节)
        if (p.uartByteUs == 0u || p.uartByteUs > p.airtimePerByteUs) {
            return false;
        }
        if (p.dlBudgetSlackUs > 10000u) {
            return false; // 预算余量不该是"秒级": 更像是单位写错
        }
        if (p.roundReopenUs == 0u || p.roundReopenUs >= p.roundMs * 1000u) {
            return false; // 新轮判定阈值必须在 (0, 轮长) 内
        }
        // rxIdleBytes 是"帧末 IDLE 判定额外占用的字符数": 实测 1; 超过 8 说明单位或标定写错
        if (p.rxIdleBytes > 8u) {
            return false;
        }
        if (p.protoVersion == 0u || p.protoVersion > 15u) {
            return false; // HELLO.ver 只有 4bit, 0 保留
        }
        if (p.helloMax == 0u) {
            return false;
        }
        return true;
    }

    /** 单帧空口成本 F(n) = base + perByte*n [us] */
    constexpr uint32_t roundAirtimeUs(const RoundParams &p, uint16_t frameBytes)
    {
        return (uint32_t)(p.airtimeBaseUs + p.airtimePerByteUs * (uint32_t)frameBytes);
    }

    //===============================================================================================
    // 二、时间基(无符号差值, 回绕安全; 与 v1 sched 同一套推理, 这里独立一份避免 v2 依赖旧文件)
    //===============================================================================================
    inline uint32_t roundTimeSince(uint32_t nowUs, uint32_t thenUs)
    {
        return (uint32_t)(nowUs - thenUs);
    }

    /** 到点了吗。前提: |now - deadline| < 2^31us(≈35.8min) —— 对 ms 级调度永远成立。 */
    inline bool roundTimeReached(uint32_t nowUs, uint32_t deadlineUs)
    {
        return (uint32_t)(nowUs - deadlineUs) < 0x80000000u;
    }

    //===============================================================================================
    // 三、枚举与结果类型
    //===============================================================================================
    enum class RoundState : uint8_t
    {
        Acquire = 0, ///< 只收不发(HELLO 除外): 等 UP_ROUND 才进 RUN
        Run     = 1, ///< 按本轮 t0 维护位置位图, 在 u3End/u3Deadline 之后发 D0
    };

    /** D0 的触发来源(观测 §8.4: "D0 触发来源"). */
    enum class RoundTxPath : uint8_t
    {
        None        = 0, ///< 本轮还没发过
        HelloAcquire = 1, ///< ACQUIRE 里的 HELLO 应答(不属于 D0)
        HelloInRound = 2, ///< RUN 空闲/占用 D0 槽的 HELLO 应答
        Pos3        = 3, ///< 正常路径: 收到 pos3 的末字节 + guard
        Fallback    = 4, ///< 兜底路径: U3 没来, 按 t0 推算的 deadline + guard
    };

    /** 放弃本轮 D0 的原因(§2.3 的每一条都要能单独计数). */
    enum class RoundSkipReason : uint8_t
    {
        None      = 0,
        AuxBusy   = 1, ///< 发前 AUX 仍忙: 放弃本轮, 绝不等待压过下一轮
        Budget    = 2, ///< now + airtime > t0 + dlTimeoutMs: 发了也赶不上遥控器窗口
        NotDue    = 3, ///< 还没到发送时刻 / 本轮没有活动
        AlreadySent = 4, ///< 本轮已经发过(每轮最多 1 帧)
        Disabled  = 5, ///< 未初始化
        Unavailable = 6, ///< 选到的帧本轮组不出来(如 DL_GPS 无数据): 放弃本轮, 不发空帧
    };

    inline const char *roundSkipReasonName(RoundSkipReason r)
    {
        switch (r) {
        case RoundSkipReason::None:        return "NONE";
        case RoundSkipReason::AuxBusy:     return "AUX_BUSY";
        case RoundSkipReason::Budget:      return "BUDGET";
        case RoundSkipReason::NotDue:      return "NOT_DUE";
        case RoundSkipReason::AlreadySent: return "ALREADY_SENT";
        case RoundSkipReason::Disabled:    return "DISABLED";
        case RoundSkipReason::Unavailable: return "UNAVAILABLE";
        default:                           return "?";
        }
    }

    inline const char *roundTxPathName(RoundTxPath p)
    {
        switch (p) {
        case RoundTxPath::None:         return "NONE";
        case RoundTxPath::HelloAcquire: return "HELLO_ACQ";
        case RoundTxPath::HelloInRound: return "HELLO_RUN";
        case RoundTxPath::Pos3:         return "POS3";
        case RoundTxPath::Fallback:     return "FALLBACK";
        default:                        return "?";
        }
    }

    /** onFrame() 的结果。sessionChanged 给 app 层用(清 cmd 去重表/ACK 队列). */
    struct RoundFrameResult
    {
        bool    accepted;       ///< 这帧是否被轮次层采信(用于定位/计时)
        bool    sessionChanged; ///< UP_ROUND 的 session 与上次不同(或首次)
        bool    startedRun;     ///< 本帧让状态机从 ACQUIRE 进入 RUN
        bool    u3Seen;         ///< 本帧就是 pos3 ⇒ 正常路径已就绪
        bool    lateIgnored;    ///< 本轮已关闭的迟到帧: 只更新计数, 不参与本轮计时/D0
        uint8_t pos;            ///< seq&3(HELLO 等无位置帧不调用本函数)
    };

    /** poll() 的结果(观测: "进入 ACQUIRE" 事件). */
    struct RoundPollResult
    {
        bool     enteredAcquire; ///< 本次 poll 因超时回到 ACQUIRE
        uint32_t silentUs;       ///< 距最后一帧合法上行的时长
    };

    //===============================================================================================
    // 四、RoundSlave
    //===============================================================================================
    /** 飞控端轮次从机。
     *
     * 用法(顺序很重要, 与协议规格 §4 逐条一致):
     *   RoundSlave rs; rs.init(params);
     *   // 收到一帧(已过 L0 帧/CRC、L1 版本/会话以外的类型检查):
     *   if (type == Hello) rs.onHelloRequest(ver, caps, nonce, rawEndUs);
     *   else rs.onFrame(type, seq, frameLen, rawEndUs, sessionOfUpRound);
     *   // 每 6ms watchdog(只做"该到没到"的判定与统计, 不做 D0 触发):
     *   rs.poll(nowUs);
     *   // 每轮最多一次 D0: 先选帧(selector.pick), 再问 rs.decide(now, auxIdle, frameLen)
     *   if (d.send) { radio.sendDma(...); rs.onDownlinkSent(d.path, nowUs, len); }
     *   else if (d.skip == AuxBusy || d.skip == Budget) { rs.onDownlinkSkipped(d.skip); }
     */
    class RoundSlave
    {
    public:
        RoundSlave() : m_p(), m_init(false) { reset(); }

        bool init(const RoundParams &p)
        {
            reset();
            if (!roundParamsValid(p)) {
                return false;
            }
            m_p    = p;
            m_init = true;
            return true;
        }

        bool initialized() const { return m_init; }
        bool run() const { return m_state == RoundState::Run; }
        bool acquire() const { return m_state == RoundState::Acquire; }
        RoundState state() const { return m_state; }
        const RoundParams &params() const { return m_p; }

        /** 回到上电状态(参数保留)。session 变化、换 profile、启动重同步时调用。 */
        void reset()
        {
            m_state            = RoundState::Acquire;
            m_haveRound        = false;
            m_roundSent        = false;
            m_roundClosed      = false;
            m_u3Seen           = false;
            m_posMask          = 0u;
            m_t0Us             = 0u;
            m_u3EndUs          = 0u;
            m_u3DeadlineUs     = 0u;
            m_lastFrameEndUs   = 0u;
            m_haveLastFrame    = false;
            m_haveSession      = false;
            m_session          = 0u;
            m_helloPending     = false;
            m_helloReply       = HelloPayload{};
            m_helloEarliestUs  = 0u;
            m_helloAttempts    = 0u;
            m_peerVer          = 0u;
            m_peerCaps         = 0u;
            m_peerVerMismatch  = false;
            m_lastPeerHelloUs  = 0u;
            m_havePeerHello    = false;
            m_lastSendPath     = RoundTxPath::None;
            m_lastSendAtUs     = 0u;
            m_lastSendDeadlineUs = 0u;
            m_lastSendJitterUs = 0u;
            m_haveSend         = false;
            m_rxPerPos[0]      = m_rxPerPos[1] = m_rxPerPos[2] = m_rxPerPos[3] = 0u;
            m_uplinkAccepted   = 0u;
            m_ignoredInAcquire = 0u;
            m_badSessionFrames = 0u;
            m_sessionChanges   = 0u;
            m_helloRequests    = 0u;
            m_helloReplies     = 0u;
            m_helloDropped     = 0u;
            m_versionMismatch  = 0u;
            m_d0Sent           = 0u;
            m_d0SkipAux        = 0u;
            m_d0SkipBudget     = 0u;
            m_pos3Rounds       = 0u;
            m_fallbackRounds   = 0u;
            m_lateFrames       = 0u;
            m_acquireEntries   = 0u;
        }

        //--------------------------------------------------------------------------- 接收侧
        /**
         * @brief 一帧上行(类型为 0x6/0x8..0xD)通过 L0(CRC/类型)之后进入轮次层
         *
         * @param type        帧类型
         * @param seq         帧头 SEQ(低 2 位 = 位置)
         * @param frameBytes  整帧长度(用于按 F(n) 反推该帧的占用时长)
         * @param rawEndUs    IDLE 回调入口的 DWT 微秒时刻(**未修正**)
         * @param session     UP_ROUND 的 session(其他帧传 0)
         *
         * ACQUIRE: 只有 UP_ROUND 被采信(⇒ 进 RUN); 0x8–0xD 一律忽略(返回 accepted=false)。
         * RUN:     0x6/0x8–0xD 都用于定位; session 变化 ⇒ 清本轮历史并重新建轮。
         */
        RoundFrameResult onFrame(FrameType type, uint8_t seq, uint16_t frameBytes, uint32_t rawEndUs,
                                 uint8_t session)
        {
            RoundFrameResult r;
            r.accepted       = false;
            r.sessionChanged = false;
            r.startedRun     = false;
            r.u3Seen         = false;
            r.lateIgnored    = false;
            r.pos            = 0u;

            if (!m_init || !isRoundUplink(type)) {
                return r;
            }

            const uint32_t endUs = airEndUs(rawEndUs, frameBytes);
            const uint8_t  pos   = seqPos(seq);
            r.pos                = pos;

            if (type == FrameType::UpRound) {
                if (session == 0u) {
                    ++m_badSessionFrames; // session 0 保留不用: 不当成新会话, 也不采信
                    return r;
                }
                if (!m_haveSession || session != m_session) {
                    m_session      = session;
                    m_haveSession  = true;
                    r.sessionChanged = true;
                    ++m_sessionChanges;
                    // 新会话: 丢掉上一会话的轮次历史(不清 session 本身)
                    m_haveRound    = false;
                    m_roundSent    = false;
                    m_roundClosed  = false;
                    m_u3Seen       = false;
                    m_posMask      = 0u;
                }
            } else if (!m_haveSession) {
                // 还没见过 UP_ROUND(不知道 session): RUN 里不可能出现, 但 ACQUIRE 里 0x8–0xD 要忽略
                if (m_state == RoundState::Acquire) {
                    ++m_ignoredInAcquire;
                    return r;
                }
            }

            if (m_state == RoundState::Acquire) {
                if (type != FrameType::UpRound) {
                    ++m_ignoredInAcquire; // ACQUIRE 只认 UP_ROUND(规格 §4: v1 RC 不会误控制)
                    return r;
                }
                m_state          = RoundState::Run;
                r.startedRun     = true;
                m_haveRound      = false; // 由下面按本帧重建
                m_roundClosed    = false;
            }

            // ---- 反推 t0(§2.2 / E5): t0 = frameEnd - (pos*spacing + F(len)) ----
            const uint32_t spacingUs = m_p.frameSpacingMs * 1000u;
            const uint32_t offsetUs  = (uint32_t)pos * spacingUs + roundAirtimeUs(m_p, frameBytes);
            if (endUs < offsetUs) {
                // 时间基刚回绕/参数错: 不采信这一帧的计时(但类型/位置已经由类型层判过)
                return r;
            }
            const uint32_t t0Us = (uint32_t)(endUs - offsetUs);

            // ---- 本轮关闭后的迟到帧(规格 §4 "本轮关闭") ----
            //   D0 发出/跳过后 roundClosed=true; 之后到来的帧若反推出"同一个 t0"(差 < roundReopenUs),
            //   只能更新计数/统计, **不得再触发第二帧 D0**。收到 UP_ROUND 或 t0 变化 >= roundReopenUs
            //   (U0 丢了、已进下一轮)才开新轮。
            if (m_state == RoundState::Run && m_roundClosed && !m_haveRound &&
                type != FrameType::UpRound) {
                if (withinTolerance(t0Us, m_t0Us, m_p.roundReopenUs)) {
                    ++m_rxPerPos[pos];
                    ++m_uplinkAccepted;
                    ++m_lateFrames;
                    m_lastFrameEndUs = endUs;
                    m_haveLastFrame  = true;
                    r.accepted    = true;
                    r.lateIgnored = true;
                    r.pos         = pos;
                    return r;
                }
            }

            // 走到这里 = 采信为本轮(刷新)或开启新轮:
            //   m_haveRound 为真 → 本轮还开着, 刷新 t0/位图;
            //   为假 → 首帧 / 新 UP_ROUND / t0 已跨过 60ms 的新轮 ⇒ 开新轮。
            const bool openNew = !m_haveRound;
            if (openNew) {
                m_haveRound   = true;
                m_roundClosed = false;
                m_roundSent   = false;
                m_posMask     = 0u;
                m_u3Seen      = false;
                m_u3EndUs     = 0u;
            }
            m_t0Us           = t0Us;
            m_posMask        = (uint8_t)(m_posMask | (uint8_t)(1u << pos));
            m_lastFrameEndUs = endUs;
            m_haveLastFrame  = true;
            ++m_rxPerPos[pos];
            ++m_uplinkAccepted;

            // 兜底 deadline: t0 + 3*spacing + F(UP_CTRL) + jitter(协议规格 §2.1 / E4)
            m_u3DeadlineUs = (uint32_t)(m_t0Us + 3u * spacingUs + roundAirtimeUs(m_p, kUpCtrlBytes) +
                                        m_p.u3JitterUs);
            if (pos == 3u) {
                m_u3Seen    = true;
                m_u3EndUs   = endUs;
                r.u3Seen    = true;
                ++m_pos3Rounds;
            }
            r.accepted = true;
            return r;
        }

        /**
         * @brief 收到 HELLO 请求(双向同型; 版本不一致也回, 只是 ACQUIRE 不进 RUN)
         *
         * @param rawEndUs IDLE 回调入口时刻(未修正)
         * 应答内容: ver=本端版本, caps=本端能力, nonce=原样回显(§3.2)。
         * ACQUIRE: 尽快回(helloEnd+guard); RUN 且本轮空闲: 尽快回; RUN 且本轮忙: 占用 D0 槽(优先级最高)。
         */
        void onHelloRequest(uint8_t peerVer, uint8_t peerCaps, uint8_t nonce, uint32_t rawEndUs)
        {
            if (!m_init) {
                return;
            }
            const uint32_t endUs = airEndUs(rawEndUs, kHelloBytes);
            ++m_helloRequests;
            m_peerVer        = peerVer;
            m_peerCaps       = peerCaps;
            m_lastPeerHelloUs = endUs;
            m_havePeerHello  = true;
            if (peerVer != m_p.protoVersion) {
                ++m_versionMismatch;
                m_peerVerMismatch = true;
            } else {
                m_peerVerMismatch = false;
            }

            m_helloReply.ver   = m_p.protoVersion;
            m_helloReply.caps  = m_p.caps;
            m_helloReply.nonce = nonce;
            m_helloPending     = true;
            m_helloAttempts    = 0u;
            m_helloEarliestUs  = (uint32_t)(endUs + m_p.guardUs);
        }

        //--------------------------------------------------------------------------- 时间戳模型
        /** 从 IDLE 回调入口时刻与帧长反推该帧的"空中结束"时刻(P0 收帧模型):
         *    airEnd = rawIdle − (frameBytes + rxIdleBytes) × uartByteUs
         *  为什么按帧长: E220 是**整帧收完才开始往 UART 吐字节**, 所以 IDLE 比空中结束晚
         *  (n + 1) 个字符时间(n 个数据字符 + IDLE 判定的 1 个)。只减固定 174us 会让 t0 偏大
         *  n×174us(n=11 时 ~1.9ms), 直接吃掉兜底 D0 的预算余量。 */
        uint32_t airEndUs(uint32_t rawEndUs, uint16_t frameBytes) const
        {
            if (!m_init) {
                return rawEndUs;
            }
            const uint32_t corr =
                (uint32_t)((uint32_t)frameBytes + m_p.rxIdleBytes) * m_p.uartByteUs;
            return (uint32_t)(rawEndUs - corr);
        }

        /** 由任意一帧反推本轮 t0(规格 §2.2/E5): t0 = airEnd − (pos×spacing + F(frameBytes))。
         *  主机测试的"锚点回归"直接用它验证时间戳模型(n=5/9/11 与真值误差 ≤1us)。 */
        uint32_t reconstructT0Us(uint32_t rawEndUs, uint8_t pos, uint16_t frameBytes) const
        {
            const uint32_t endUs   = airEndUs(rawEndUs, frameBytes);
            const uint32_t spacing = (uint32_t)m_p.frameSpacingMs * 1000u;
            const uint32_t offset  = (uint32_t)pos * spacing + roundAirtimeUs(m_p, frameBytes);
            return (endUs >= offset) ? (uint32_t)(endUs - offset) : endUs;
        }

        bool helloReplyPending() const { return m_helloPending; }
        uint32_t helloEarliestUs() const { return m_helloEarliestUs; }
        uint8_t helloAttempts() const { return m_helloAttempts; }
        RoundTxPath lastSendPath() const { return m_lastSendPath; }

        /** 取走待发应答(调用方随后 encodeHello + 发送, 再调 onHelloSent/onDownlinkSent). */
        HelloPayload takeHelloReply()
        {
            return m_helloReply;
        }

        //--------------------------------------------------------------------------- 发送侧
        struct TxDecision
        {
            bool             send;      ///< true: 现在可以发(调用方还需用预算检查/发送结果回报)
            RoundTxPath      path;      ///< 触发来源
            RoundSkipReason  skip;      ///< 不能发时的原因
            uint32_t         sendAtUs;  ///< 本帧"最早可发"的时刻(绝对 us); send=true 时 now>=sendAtUs
            bool             isHello;   ///< true: 发 HELLO 应答(不走 D0 选帧); false: 发 D0
        };

        /** 纯查询: 现在能不能发、该发 HELLO 还是 D0(不看 AUX/预算; 见 decide 重载)。
         *  @param plannedBytes 拟发帧的整帧长度(用于预算检查; 不传则跳过预算) */
        TxDecision decide(uint32_t nowUs) const
        {
            TxDecision d;
            d.send     = false;
            d.path     = RoundTxPath::None;
            d.skip     = RoundSkipReason::NotDue;
            d.sendAtUs = 0u;
            d.isHello  = false;
            if (!m_init) {
                d.skip = RoundSkipReason::Disabled;
                return d;
            }

            // ---- 1. HELLO 应答优先(ACQUIRE 随时回; RUN 里若本轮空闲也回, 忙则占 D0 槽) ----
            if (m_helloPending) {
                const bool roundBusy = (m_state == RoundState::Run) && m_haveRound && !m_roundSent;
                if (!roundBusy && roundTimeReached(nowUs, m_helloEarliestUs)) {
                    d.send     = true;
                    d.path     = (m_state == RoundState::Run) ? RoundTxPath::HelloInRound
                                                              : RoundTxPath::HelloAcquire;
                    d.sendAtUs = m_helloEarliestUs;
                    d.isHello  = true;
                    d.skip     = RoundSkipReason::None;
                    return d;
                }
                // RUN 且本轮忙: 落到下面的 D0 槽, 由 D0 优先级决定(HELLO 在 §6 里排第 1)
            }

            // ---- 2. D0: 收到 pos3 → u3End+guard; 否则按 t0 推算(兜底) ----
            if (m_state != RoundState::Run) {
                return d;
            }
            if (m_roundSent) {
                d.skip = RoundSkipReason::AlreadySent; // 本轮已经发过/已放弃
                return d;
            }
            if (!m_haveRound) {
                return d; // 本轮还没有任何上行(还没建立 t0)
            }
            const uint32_t sendAt = (uint32_t)((m_u3Seen ? m_u3EndUs : m_u3DeadlineUs) + m_p.guardUs);
            d.sendAtUs            = sendAt;
            d.path                = m_u3Seen ? RoundTxPath::Pos3 : RoundTxPath::Fallback;
            if (!roundTimeReached(nowUs, sendAt)) {
                return d;
            }
            d.send    = true;
            d.skip    = RoundSkipReason::None;
            d.isHello = m_helloPending; // 本轮忙时 HELLO 应答占用 D0 槽(§6 优先级 #1)
            return d;
        }

        /** 带发送前检查的重载(§2.3): AUX 忙 / 预算不足 → 放弃本轮。
         *  @param auxIdle 发前读到的 AUX(高=空闲); auxGate=false 时忽略
         *  @param plannedBytes 拟发帧整帧长度(21B 最坏)
         *
         *  ⚠ 预算只约束 **D0**(Pos3/Fallback 两条路径): 它们的发送时刻由"本轮 t0 + 117ms 窗口"
         *    定义。HELLO 应答不属于 D0 —— ACQUIRE 的应答没有本轮 t0, RUN 空闲时的应答(路径
         *    HelloInRound)同样没有活动轮次(此时 m_haveRound=false, budgetAllows() 会直接返回
         *    false)。早先这里写的是 "path != HelloAcquire", 于是 RUN 空闲期(200ms 内没帧、
         *    还没转 ACQUIRE)的 HELLO 应答会被预算误杀 —— 恢复要等状态机退回 ACQUIRE 才变快,
         *    与规格 §4 "RUN 里本轮无活动也尽快回 HELLO" 不符。
         *  AUX 检查则对 HELLO 应答同样生效(§2.3 第 1 条: 发前读 AUX, 忙则不发)。 */
        TxDecision decide(uint32_t nowUs, bool auxIdle, uint16_t plannedBytes) const
        {
            TxDecision d = decide(nowUs);
            if (!d.send) {
                return d;
            }
            if (m_p.auxGate && !auxIdle) {
                d.skip = RoundSkipReason::AuxBusy;
                d.send = false;
                return d;
            }
            const bool isD0 = (d.path == RoundTxPath::Pos3 || d.path == RoundTxPath::Fallback);
            if (isD0 && !budgetAllows(nowUs, plannedBytes)) {
                d.skip = RoundSkipReason::Budget;
                d.send = false;
            }
            return d;
        }

        /** 预算(规格 §2.3 第 2 条, 2026-10-18 修订):
         *    now + airtime(所选帧) + 帧长×uartByteUs + dlBudgetSlackUs > t0 + dlTimeoutMs*1000
         *  ⇒ 放弃本轮 D0。
         *  为什么不能只算"空中结束进窗": 遥控器要把**整帧从串口收完**才算收到 —— 13B 还要 ~2.3ms、
         *  21B 还要 ~3.7ms 的排空时间; 再留 1ms 调度余量。少算这一段的表现是"飞控认为发了,
         *  遥控器按 117ms 判 DL 丢失", 而且越长的帧越糟(恰好是最需要 GPS 的时候)。 */
        bool budgetAllows(uint32_t nowUs, uint16_t plannedBytes) const
        {
            if (!m_init || !m_haveRound) {
                return false;
            }
            const uint32_t deadline = (uint32_t)(m_t0Us + m_p.dlTimeoutMs * 1000u);
            if (roundTimeReached(nowUs, deadline)) {
                return false; // 已过遥控器窗口
            }
            const uint32_t need = (uint32_t)(roundAirtimeUs(m_p, plannedBytes) +
                                             (uint32_t)plannedBytes * m_p.uartByteUs +
                                             m_p.dlBudgetSlackUs);
            return (uint32_t)(deadline - nowUs) >= need;
        }

        /** 记录"真的发出去了"(调用方在 sendDma 成功后调用)。每轮最多 1 帧由这里闭合。
         *  @param isHello   这一帧是不是 HELLO 应答
         *  @param path      decide() 给出的触发来源
         *
         *  ⚠ 计数口径(规格 §6): HELLO 应答**占用本轮的 D0 槽, 但不推进任何遥测计数** ——
         *    `m_d0Sent`/`m_fallbackRounds` 只统计"发出了一帧**遥测/ACK** D0"; HELLO 只进
         *    `m_helloReplies`。但"本轮槽已用掉/本轮关闭"必须照常置位, 否则同一轮会再发第二帧。
         */
        void onDownlinkSent(bool isHello, RoundTxPath path, uint32_t nowUs, uint16_t frameBytes)
        {
            (void)frameBytes;
            m_lastSendAtUs     = nowUs;
            m_lastSendJitterUs = 0u;
            m_haveSend         = true;
            if (path == RoundTxPath::Pos3 || path == RoundTxPath::Fallback) {
                m_lastSendDeadlineUs = (uint32_t)((m_u3Seen ? m_u3EndUs : m_u3DeadlineUs) + m_p.guardUs);
                m_lastSendJitterUs   = (uint32_t)(nowUs - m_lastSendDeadlineUs);
            }
            m_lastSendPath = path;

            if (isHello) {
                m_helloPending = false;
                ++m_helloReplies; // 唯一的 HELLO 计数(不碰 m_d0Sent / m_fallbackRounds)
            } else if (path == RoundTxPath::Pos3 || path == RoundTxPath::Fallback) {
                ++m_d0Sent; // 遥测/ACK 的 D0(HELLO 不算)
                if (path == RoundTxPath::Fallback) {
                    ++m_fallbackRounds;
                }
            }
            if (path == RoundTxPath::Pos3 || path == RoundTxPath::Fallback) {
                m_roundSent   = true; // 本轮下行槽已用掉(HELLO 占槽也一样)
                m_roundClosed = true; // 本轮关闭: 迟到帧不得再触发第二帧 D0(规格 §4)
                m_haveRound   = false;
            }
        }

        /** 记录"放弃本轮"(AUX 忙/预算不足): 本轮不再尝试(§2.3: 绝不等待压过下一轮). */
        void onDownlinkSkipped(RoundSkipReason reason)
        {
            if (reason == RoundSkipReason::AuxBusy) {
                ++m_d0SkipAux;
            } else if (reason == RoundSkipReason::Budget) {
                ++m_d0SkipBudget;
            }
            m_roundSent = true;
            m_roundClosed = true; // 本轮关闭(放弃也算关闭): 本轮不再重试
            m_haveRound = false;
            if (m_helloPending) {
                // HELLO 应答也占用本轮: 记一次尝试, 超限则丢弃(避免永远挂着)
                ++m_helloAttempts;
                if (m_helloAttempts >= m_p.helloMax) {
                    m_helloPending = false;
                    ++m_helloDropped;
                }
            }
        }

        //--------------------------------------------------------------------------- watchdog
        /** 每 LinkTickMs(6ms) 调用一次: 只做"该到没到"的超时判定与状态回退, 绝不是 D0 触发源。 */
        RoundPollResult poll(uint32_t nowUs)
        {
            RoundPollResult r;
            r.enteredAcquire = false;
            r.silentUs       = 0u;
            if (!m_init) {
                return r;
            }
            if (m_haveLastFrame) {
                r.silentUs = roundTimeSince(nowUs, m_lastFrameEndUs);
            }
            if (m_state == RoundState::Run && (!m_haveLastFrame || r.silentUs >= m_p.acquireMs * 1000u)) {
                m_state          = RoundState::Acquire;
                m_haveRound      = false;
                m_roundSent      = false;
                m_roundClosed    = false; // ACQUIRE 不跟踪轮次; 下一次 UP_ROUND 重新开轮
                m_haveSession    = false; // 会话信息也作废: 下次 UP_ROUND 重新建
                m_session        = 0u;
                m_helloPending   = false;
                m_helloAttempts  = 0u;
                r.enteredAcquire = true;
                ++m_acquireEntries;
            }
            return r;
        }

        //--------------------------------------------------------------------------- 查询/观测
        /** 本轮已收到的位置位图(bit0..3 = pos0..3) —— 直接填 DL_STATE.rxBitmap(§3.5). */
        uint8_t rxBitmap() const { return (uint8_t)(m_posMask & 0x0Fu); }
        bool    roundActive() const { return m_haveRound && !m_roundSent; }
        bool    roundSent() const { return m_roundSent; }
        /** 本轮是否已关闭(D0 发出或跳过): 关闭后迟到帧不得再触发 D0(规格 §4). */
        bool    roundClosed() const { return m_roundClosed; }
        uint32_t roundT0Us() const { return m_t0Us; }
        uint32_t u3EndUs() const { return m_u3EndUs; }
        uint32_t u3DeadlineUs() const { return m_u3DeadlineUs; }
        bool     u3Seen() const { return m_u3Seen; }
        uint32_t lastFrameEndUs() const { return m_lastFrameEndUs; }
        uint8_t  session() const { return m_session; }
        bool     haveSession() const { return m_haveSession; }
        uint8_t  peerVersion() const { return m_peerVer; }
        uint8_t  peerCaps() const { return m_peerCaps; }
        bool     peerVersionMatch() const { return m_havePeerHello && !m_peerVerMismatch; }
        bool     peerVersionMismatch() const { return m_peerVerMismatch; }
        bool     havePeerHello() const { return m_havePeerHello; }
        uint32_t peerHelloEndUs() const { return m_lastPeerHelloUs; }

        /** 最近一次 D0 的抖动(实际发送 - 计划时刻; 观测 §8.4 "D0 触发时刻偏差"). */
        bool     haveLastSend() const { return m_haveSend; }
        uint32_t lastSendJitterUs() const { return m_lastSendJitterUs; }
        uint32_t lastSendAtUs() const { return m_lastSendAtUs; }

        uint16_t rxCountAtPos(uint8_t pos) const { return pos < 4u ? m_rxPerPos[pos] : 0u; }
        uint32_t acceptedCount() const { return m_uplinkAccepted; }
        uint32_t ignoredInAcquireCount() const { return m_ignoredInAcquire; }
        uint32_t badSessionCount() const { return m_badSessionFrames; }
        uint32_t sessionChangeCount() const { return m_sessionChanges; }
        uint32_t helloRequestCount() const { return m_helloRequests; }
        uint32_t helloReplyCount() const { return m_helloReplies; }
        uint32_t helloDroppedCount() const { return m_helloDropped; }
        uint32_t versionMismatchCount() const { return m_versionMismatch; }
        uint32_t d0SentCount() const { return m_d0Sent; }
        uint32_t d0SkipAuxCount() const { return m_d0SkipAux; }
        uint32_t d0SkipBudgetCount() const { return m_d0SkipBudget; }
        uint32_t pos3RoundCount() const { return m_pos3Rounds; }
        uint32_t fallbackRoundCount() const { return m_fallbackRounds; }
        uint32_t acquireEntryCount() const { return m_acquireEntries; }
        uint32_t droppedHelloCount() const { return m_helloDropped; }
        /** 本轮关闭后到达、只更新计数而未参与本轮计时的迟到帧数(规格 §4). */
        uint32_t lateFrameCount() const { return m_lateFrames; }

    private:
        /** |a-b| <= tol(回绕安全): 64 位差取小者比, 避免 32 位无符号减法的方向陷阱 */
        static bool withinTolerance(uint32_t a, uint32_t b, uint32_t tol)
        {
            if (a == b) {
                return true;
            }
            return (a > b) ? ((uint32_t)(a - b) <= tol) : ((uint32_t)(b - a) <= tol);
        }

        static bool isRoundUplink(FrameType t)
        {
            switch (t) {
            case FrameType::UpRound:
            case FrameType::UpCtrl:
            case FrameType::UpCmd:
            case FrameType::UpVel:
            case FrameType::UpPos:
            case FrameType::UpSetpoint:
            case FrameType::UpMotor:
                return true;
            default:
                return false;
            }
        }

        RoundParams m_p;
        bool        m_init;

        RoundState m_state;
        bool       m_haveRound; ///< 本轮有活动且还没发过 D0
        bool       m_roundSent; ///< 本轮已经发过(或已放弃) —— 每轮最多 1 帧
        bool       m_roundClosed; ///< 本轮已关闭(规格 §4): 迟到帧不得再触发 D0
        bool       m_u3Seen;
        uint8_t    m_posMask;
        uint32_t   m_t0Us;
        uint32_t   m_u3EndUs;
        uint32_t   m_u3DeadlineUs;
        uint32_t   m_lastFrameEndUs;
        bool       m_haveLastFrame;

        bool    m_haveSession;
        uint8_t m_session;

        bool         m_helloPending;
        HelloPayload m_helloReply;
        uint32_t     m_helloEarliestUs;
        uint8_t      m_helloAttempts;
        uint8_t      m_peerVer;
        uint8_t      m_peerCaps;
        bool         m_peerVerMismatch;
        uint32_t     m_lastPeerHelloUs;
        bool         m_havePeerHello;

        RoundTxPath m_lastSendPath;
        uint32_t    m_lastSendAtUs;
        uint32_t    m_lastSendDeadlineUs;
        uint32_t    m_lastSendJitterUs;
        bool        m_haveSend;

        uint16_t m_rxPerPos[4];
        uint32_t m_uplinkAccepted;
        uint32_t m_ignoredInAcquire;
        uint32_t m_badSessionFrames;
        uint32_t m_sessionChanges;
        uint32_t m_helloRequests;
        uint32_t m_helloReplies;
        uint32_t m_helloDropped;
        uint32_t m_versionMismatch;
        uint32_t m_d0Sent;
        uint32_t m_d0SkipAux;
        uint32_t m_d0SkipBudget;
        uint32_t m_pos3Rounds;
        uint32_t m_fallbackRounds;
        uint32_t m_acquireEntries;
        uint32_t m_lateFrames;
    };

} // namespace link
} // namespace dlx
