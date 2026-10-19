#pragma once
//===================================================================================================
// DL_LIB/link/dlx_link_downlink.hpp —— 下行组帧(纯逻辑, 无硬件依赖)
//
// 职责: 把"控制环已经算好的物理量"定点化并编成一帧下行帧。**不做 I/O、不碰寄存器、不管时隙**。
//   为什么单独一个文件(而不是写在 app_link_thread.hpp 里):
//     ① app_link_thread.hpp 依赖 RT-Thread / E220 / 共享区, 主机上编不了 —— 而"姿态怎么变成
//        12 位四元数虚部"、"世界系速度怎么变成机体速度"这类换算恰恰是**最需要主机单测**的部分;
//     ② 遥控器侧做"差值曲线"用的就是这些字段, 换算错了曲线会随偏航"转", 那种错在屏幕上看不出原因。
//
// 线上约定(与 dlx_link_frame.hpp 的帧表逐位一致, 这里不另立第二份真相):
//   DL_ATT   q1,q2,q3 = round(x*2047) 饱和 ±2047;  vx,vy,vz = **机体**速度 dm/s(round(mps*10))
//   DL_SYS   位置 dm(round(m*10)); 电流/电压 0.1 单位; motor[4] = DShot 计数 0..1999
//   DL_STATE 状态机 4 维 + 健康 + 丢包(夹到 100%) + RSSI(有符号)
//
// ⚠ 速度坐标系: EKF 给的是**世界系**速度, 而 DL_ATT 的 vx,vy,vz 线上是**机体速度**
//   (规格 §3.3)。所以这里必须做一次 q* 旋转。遥控器拿到机体速度后要用回传姿态旋回世界系
//   才能与"定速模式的期望速度(世界系)"作差 —— 两端各转一次, 职责清楚。
//===================================================================================================

#include <stdint.h>

#include "dlx_link_frame.hpp" // 帧表 + 编解码(同目录)
#include "../dlx_math.hpp"    // Quaternion / Vector3f / quatRotate / quatCanonicalize

namespace dlx
{
namespace link
{
    /**
     * 下行组帧的数据源。
     *
     * 字段取"数据源的天然表示": 控制环/EKF 出来就是 float 物理量, 就存 float; 本来就是计数/位域的
     * 就存整数原样。定点化只在本文件里做一次 —— 调用方**不许**自己 round。
     */
    struct DownlinkSource
    {
        // ---- DL_ATT ----
        Quaternion attitude{{1.0f, 0.0f, 0.0f, 0.0f}}; ///< 世界系姿态(机体系→世界系), 与 Madgwick/EKF 同系
        Vector3f   velWorldMps{};                      ///< **世界系**速度 [m/s](EKF 输出; 本模块负责转机体)
        uint8_t    status{0};                          ///< kTelStatus* 位域(实时, 不锁存)
        uint8_t    error{0};                           ///< kTelErr* 位域(实时, 不锁存)
        uint8_t    arming{0};                          ///< 解锁状态码 0..3(由链路适配层映射)
        uint8_t    mode{0};                            ///< 模式码 0..3(DL_ATT 只有 2 位, 4 档正好)
        // ---- DL_SYS ----
        Vector3f   posM{};        ///< 世界系位置 [m]: x 北 / y 东 / **z = 高度(向上为正)**
        float      currentA{0.0f};///< 电流 [A]
        float      voltageV{0.0f};///< 电压 [V]
        uint16_t   motor[4]{0u, 0u, 0u, 0u}; ///< DShot 计数 0..1999(超出会被夹)
        // ---- DL_STATE ----
        uint8_t    lifecycle{0};   ///< 生命周期码 0..7(由链路适配层映射)
        uint8_t    task{0};        ///< 操作码 0..2(由链路适配层映射)
        uint8_t    failsafeLvl{0}; ///< 旧协议等级码 0..4(线上 3 位), 应用当前填 0
        uint8_t    failsafeRsn{0}; ///< 旧协议原因码 0..8(线上 4 位), 应用当前填 0
        uint8_t    health{0};      ///< 健康位域: bit N = 上报方定义的"第 N 项"(位序由 app 层给出)
        uint8_t    ulLoss{0};      ///< 上行丢包 %(本机=从端测的; 夹到 100)
        uint8_t    dlLoss{0};      ///< 下行丢包 %(本端填; 夹到 100)
        int8_t     rssiDbm{0};     ///< RSSI [dBm]。E220 未开 DRSSI ⇒ 暂时恒 0(如实填, 不假装有值)
        // ---- DL_STATE v2: 本轮上行接收位图(bit0..3 = pos0..3; 由 RoundSlave.rxBitmap() 给) ----
        uint8_t    rxBitmap{0};
        // ---- DL_GPS(可选; v2 只在"GNSS 已接 + RC 想看"时才选它) ----
        bool       gpsValid{false}; ///< false = 本端没有 GPS 数据(选择器应永不选 DL_GPS)
        int32_t    latE7{0};        ///< 纬度 度×1e7
        int32_t    lonE7{0};        ///< 经度 度×1e7
        int32_t    altMslMm{0};     ///< 海拔(MSL) mm
        int16_t    gspeedCms{0};    ///< 地速 cm/s
        uint16_t   courseCdeg{0};   ///< 航向 0.01°
        uint8_t    gpsSats{0};      ///< 星数 0..63
        uint8_t    gpsFix{0};       ///< 定位类型 0..3
        float      gpsHdop{0.0f};   ///< HDOP
    };

    /** 该类型是不是本模块会组的下行帧(角色判据, 防"拿上行类型来组下行") */
    inline bool isDownlinkType(FrameType t)
    {
        return t == FrameType::DlAtt || t == FrameType::DlSys || t == FrameType::DlState || t == FrameType::DlAck;
    }

    /**
     * @brief 世界系速度 → 机体速度
     *
     * 约定与 dlx_math.hpp 一致: 四元数是"机体系→世界系"(v_world = q ⊗ v_body ⊗ q*),
     * 所以反向就是 q* ⊗ v_world ⊗ q* 的共轭旋转 = quatRotate(quatConjugate(q), v_world)。
     * 姿态先归一化并规范化(w≥0) —— 姿态估计偶尔会给出未归一化的四元数, 直接拿去旋转会让
     * "静止时速度不为零", 而那种错在曲线上看起来像"飞机在飘"。
     */
    inline Vector3f worldVelToBody(const Quaternion &attitude, const Vector3f &velWorld)
    {
        Quaternion q = quatNormalize(attitude);
        quatCanonicalize(q);
        return quatRotate(quatConjugate(q), velWorld);
    }

    /** 丢包率夹到 100%(">100%"不是更糟, 是数错了) */
    inline uint8_t clampLoss(uint8_t pct)
    {
        return (pct > kLossMaxPercent) ? kLossMaxPercent : pct;
    }

    /**
     * @brief 按类型组一帧下行帧
     *
     * @param t   帧类型(必须是 isDownlinkType() 认可的那几种; DL_GPS 本轮不组, 返回 0)
     * @param seq 本方向自增的 SEQ(模 16, 由调用方维护)
     * @param s   数据源
     * @param out 输出缓冲(至少 kMaxFrameBytes 字节)
     * @return 整帧长度(字节); 0 = 参数非法或该类型本轮不组
     */
    inline uint16_t buildDownlinkFrame(FrameType t, uint8_t seq, const DownlinkSource &s, uint8_t *out)
    {
        switch (t) {
        case FrameType::DlAtt: {
            DlAttPayload p = {};
            Quaternion    q = quatNormalize(s.attitude);
            quatCanonicalize(q); // w ≥ 0: 遥控器要用 w²=1-(q1²+q2²+q3²) 取正根还原, 负半球会还原错
            p.q1 = q.data[1];
            p.q2 = q.data[2];
            p.q3 = q.data[3];
            const Vector3f vb = worldVelToBody(q, s.velWorldMps);
            p.vx     = vb.x();
            p.vy     = vb.y();
            p.vz     = vb.z();
            p.status = s.status;
            p.error  = s.error;
            p.arming = s.arming;
            p.mode   = s.mode;
            return encodeDlAtt(p, seq, out);
        }
        case FrameType::DlSys: {
            DlSysPayload p = {};
            p.x = s.posM.x();
            p.y = s.posM.y();
            p.z = s.posM.z();
            p.currentA = s.currentA;
            p.voltageV = s.voltageV;
            for (uint8_t i = 0u; i < dlSys::kMotorCount; ++i) {
                p.motor[i] = s.motor[i];
            }
            return encodeDlSys(p, seq, out);
        }
        case FrameType::DlState: {
            DlStatePayload p = {};
            p.lifecycle   = s.lifecycle;
            p.arming      = s.arming;
            p.mode        = s.mode;
            p.task        = s.task;
            p.failsafeLvl = s.failsafeLvl;
            p.failsafeRsn = s.failsafeRsn;
            p.health      = s.health;
            p.ulLoss      = clampLoss(s.ulLoss);
            p.dlLoss      = clampLoss(s.dlLoss);
            p.rssiDbm     = s.rssiDbm;
            p.rxBitmap    = (uint8_t)(s.rxBitmap & dlState::kRxBitmapMask);
            return encodeDlState(p, seq, out);
        }
        case FrameType::DlAck:
            // DL_ACK 的内容是"某条命令的应答", 不是状态快照 —— 由调用方直接 encodeDlAck 组帧。
            // 这里返回 0 而不是"编一个空 ACK": 空的 ACK 会让地面端以为某条命令被拒了。
            return 0u;
        case FrameType::DlGps: {
            // v2: 只有本端真的有 GPS 数据(gpsValid)才组帧。没数据返回 0, 调用方按"跳过"计数 ——
            // 绝不发一帧"零经纬度", 那会被对端当成"飞机在几内亚湾"。
            if (!s.gpsValid) {
                return 0u;
            }
            DlGpsPayload p = {};
            p.latE7      = s.latE7;
            p.lonE7      = s.lonE7;
            p.altMslMm   = s.altMslMm;
            p.gspeedCms  = s.gspeedCms;
            p.courseCdeg = s.courseCdeg;
            p.sats       = s.gpsSats;
            p.fix        = s.gpsFix;
            p.hdop       = s.gpsHdop;
            return encodeDlGps(p, seq, out);
        }
        default:
            return 0u; // 上行类型(或非法类型)走进来 = 调用方角色配错, 返回 0 让它立刻可见
        }
    }

    //===============================================================================================
    // 八、下行选帧器 v2(协议规格 §6) —— 优先级 + "到期"调度, 取代 v1 的取模轮转
    //
    //   每轮 D0 只发 1 帧, 优先级(高→低):
    //     1. HELLO 应答(由 RoundSlave 的 helloReplyPending 给; 它替代本轮 D0)
    //     2. DL_ACK(按产生顺序, 一轮一条)
    //     3. DL_GPS(本端 GNSS 开启 且 对端 caps.bit0 想要 且 到期)
    //     4. DL_SYS(每 4 轮, 偏移 2)
    //     5. DL_STATE(每 4 轮)
    //     6. DL_ATT(兜底, 每轮都可用)
    //
    //   "到期"用**发送轮号**判: 只有真的发出遥测帧才 ++轮号 ⇒ ACK/HELLO 不占轮次、不推进计数
    //   (规格 §6 "ACK 不占轮次计数; HELLO 应答也不推进任何遥测计数")。没发出去的到期帧不会把
    //   后续节拍整体拉走 —— 它只是等下一个到期点(SYS/STATE 最多晚 4 轮, GPS 最多晚 16 轮)。
    //
    //   为什么把选帧和发送分开(pick 纯查询 / consume 提交): RoundSlave 的预算检查要用
    //   "拟发帧的长度", 而长度又取决于选到哪一型; 先 pick(不耗状态) → 预算 → consume,
    //   任何一步失败都不会把 ACK 队列吃掉(那就等于"发不出去还把回执弄丢了")。
    //===============================================================================================
    enum class DlPickKind : uint8_t
    {
        Hello = 0,
        Ack   = 1,
        Gps   = 2,
        Sys   = 3,
        State = 4,
        Att   = 5,
        None  = 6,
    };

    inline const char *dlPickKindName(DlPickKind k)
    {
        switch (k) {
        case DlPickKind::Hello: return "HELLO";
        case DlPickKind::Ack:   return "DL_ACK";
        case DlPickKind::Gps:   return "DL_GPS";
        case DlPickKind::Sys:   return "DL_SYS";
        case DlPickKind::State: return "DL_STATE";
        case DlPickKind::Att:   return "DL_ATT";
        default:                return "NONE";
        }
    }

    /** 下行调度参数(全部来自 link_v2_config.hpp 的策略部分; 帧长不在这里). */
    struct DownlinkParamsV2
    {
        uint8_t stateEvery; ///< 每几轮发一次 DL_STATE(4)
        uint8_t sysEvery;   ///< 每几轮发一次 DL_SYS(4)
        uint8_t sysOffset;  ///< DL_SYS 的相位偏移(2: 与 STATE 错开)
        uint8_t gpsEvery;   ///< 每几轮发一次 DL_GPS(16)
        bool    gpsEnabled; ///< 本端 GNSS 真的可用(没数据时必须 false, 否则白占 1/16 槽)
    };

    inline bool downlinkParamsValidV2(const DownlinkParamsV2 &p)
    {
        return p.stateEvery >= 1u && p.sysEvery >= 1u && p.gpsEvery >= 1u && p.sysOffset < p.sysEvery;
    }

    /** 一次选帧结果。kind=Ack 时 ack 是队首那条(未弹出; consume 才弹). */
    struct DlPick
    {
        DlPickKind   kind;
        DlAckPayload ack;
    };

    class DownlinkSelector
    {
    public:
        static constexpr uint8_t kAckCapacity = 4u; ///< 在途命令结果上限(§3.4 "一轮一条")

        DownlinkSelector() : m_p(), m_init(false) { reset(); }

        bool init(const DownlinkParamsV2 &p)
        {
            reset();
            if (!downlinkParamsValidV2(p)) {
                return false;
            }
            m_p    = p;
            m_init = true;
            return true;
        }

        bool initialized() const { return m_init; }

        void reset()
        {
            m_sentRounds = 0u;
            m_head       = 0u;
            m_count      = 0u;
            m_ackDropped = 0u;
            m_picks      = 0u;
            for (uint8_t i = 0u; i < 6u; ++i) {
                m_pickCount[i] = 0u;
            }
        }

        /** session 变化: 清 ACK 队列与节拍计数(旧会话的未发结果全部作废, §8.1). */
        void onSessionChanged()
        {
            m_head       = 0u;
            m_count      = 0u;
            m_sentRounds = 0u;
        }

        /** 入队一条命令结果。满(4 条)则丢**最老**的并计数: 回执的价值在"最近那条命令". */
        bool pushAck(uint8_t cmdId, uint8_t result)
        {
            if (m_count >= kAckCapacity) {
                // 丢最老: head 前移一位, count 先减再加
                m_head = (uint8_t)((m_head + 1u) % kAckCapacity);
                --m_count;
                ++m_ackDropped;
            }
            const uint8_t tail = (uint8_t)((m_head + m_count) % kAckCapacity);
            m_ack[tail].cmdId  = cmdId;
            m_ack[tail].result = result;
            ++m_count;
            return true;
        }

        /** 纯查询(DlPickKind::None = 未初始化 / 无可用帧; 正常情况下 ATT 兜底所以不会是 None). */
        DlPick pick(bool helloPending, bool rcWantsGps) const
        {
            DlPick p;
            p.kind   = DlPickKind::None;
            p.ack    = DlAckPayload{};
            if (!m_init) {
                return p;
            }
            if (helloPending) {
                p.kind = DlPickKind::Hello;
                return p;
            }
            if (m_count > 0u) {
                p.kind      = DlPickKind::Ack;
                p.ack       = m_ack[m_head];
                return p;
            }
            if (m_p.gpsEnabled && rcWantsGps && due(m_p.gpsEvery, 0u)) {
                p.kind = DlPickKind::Gps;
                return p;
            }
            if (due(m_p.sysEvery, m_p.sysOffset)) {
                p.kind = DlPickKind::Sys;
                return p;
            }
            if (due(m_p.stateEvery, 0u)) {
                p.kind = DlPickKind::State;
                return p;
            }
            p.kind = DlPickKind::Att;
            return p;
        }

        /** 提交一次选帧: 遥测类推进"发送轮号"; ACK 弹队首; HELLO 不占轮次. */
        void consume(const DlPick &p)
        {
            switch (p.kind) {
            case DlPickKind::Hello:
                break;
            case DlPickKind::Ack:
                if (m_count == 0u) {
                    break;
                }
                m_head = (uint8_t)((m_head + 1u) % kAckCapacity);
                --m_count;
                break;
            case DlPickKind::Gps:
            case DlPickKind::Sys:
            case DlPickKind::State:
            case DlPickKind::Att:
                ++m_sentRounds;
                ++m_pickCount[(uint8_t)p.kind];
                ++m_picks;
                break;
            default:
                break;
            }
        }

        uint32_t sentRounds() const { return m_sentRounds; }
        uint8_t  pendingAckCount() const { return m_count; }
        uint32_t ackDroppedCount() const { return m_ackDropped; }
        uint32_t pickCount(DlPickKind k) const
        {
            return ((uint8_t)k < 6u) ? m_pickCount[(uint8_t)k] : 0u;
        }
        uint32_t consumeCount() const { return m_picks; }
        const DownlinkParamsV2 &params() const { return m_p; }

    private:
        bool due(uint8_t every, uint8_t offset) const
        {
            if (every == 0u) {
                return false;
            }
            return (uint32_t)((m_sentRounds + every - offset) % every) == 0u;
        }

        DownlinkParamsV2 m_p;
        bool             m_init;
        uint32_t         m_sentRounds;
        DlAckPayload     m_ack[kAckCapacity];
        uint8_t          m_head;
        uint8_t          m_count;
        uint32_t         m_ackDropped;
        uint32_t         m_picks;
        uint32_t         m_pickCount[6];
    };

} // namespace link
} // namespace dlx
