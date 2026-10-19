#pragma once
//===================================================================================================
// DL_LIB/link/dlx_link_sched.hpp —— 从端时分调度(TDMA) + 帧准入前置(纯协议, **零业务依赖**)
//
// 本文件是**纯逻辑**: 只吃"单调递增的本地微秒时基"与"已解出的帧", 不读 DWT/rt_tick, 不 include
// 任何 stm32/RT-Thread 头 —— 所以它能在主机上跑满单测(见 test/sched_check), 也能原样搬到别的板子。
// 代价是"现在几点"必须由调用方每个事件喂进来(nowUs), 调度器自己永远不去问时间。
//
// 四块职责(每块都有单测, 见 test/sched_check/check.cpp):
//   1. SchedSlave       从端相位: 只在主端最后一个上行时隙之后开自己的发送窗口; 失同步就静默。
//   2. DownlinkPlanner  下行时隙的内容轮转(§3.5"下行槽内容轮转") + 下行 SEQ 自增计数器。
//   3. 线上编号表         "这个 op / result 字节在不在协议表里"(**不是** CommandOp/GroundStatus:
//                        那两个枚举是业务词汇, 属于 app 层, 见 app_link_cmd.hpp)。
//   4. FrameAdmission   帧准入前置: "解出来了" ≠ "这是对端发的", 见下面"为什么必须有准入前置"。
//
// ---- 为什么必须有准入前置(本文件最重要的依据) ----
//   §3.2 把帧尾从 [CRC4|Type4] 换成 CRC8, 漏检率从 1/16 降到 **1/256** —— 但 1/256 不是 0:
//   每约 256 个字节位置就会出现 1 个"CRC 恰好合法"的假帧, 而帧长又是由那个假类型字节推出来的。
//   于是一帧假数据完全可能带着合法 CRC、合法类型、任意 payload 进入上层。B3 的单测已经把这条
//   代价固定下来了(见 dlx_link_frame.hpp 文件头"漏检率 1/256"与 test/link_check)。
//   结论: **CRC 只回答"这串字节没被信道改坏", 不回答"这是对端发的"**。后者要靠四条判据:
//     ① SEQ 是否落在合理窗口(重放/伪造的 SEQ 通常重复, 或跳得远得不像同一串上行);
//     ② 帧类型是否在**本角色**允许的集合里(飞控不该收到 DL_GPS, 遥控器不该收到 UP_CTRL);
//     ③ 关键字段是否在物理量级内(纬度 ±90°、四元数模长 ≤1、DL_STATE 的 7 个恒 0 空位、丢包率 ≤100%…);
//     ④ 命令帧的 op 是否在协议表里、arg 是否在取值域。(2026-10-11 起这一条**只判协议事实**:
//        "命令现在能不能执行"是业务准入, 已搬去 app 层 —— 混进来的时候, 状态不允许的命令会被
//        当成假帧静默丢掉, 一个 DL_ACK 都不回。见下方 checkCommand 的注释。)
//   这四条里每一条都必须能被**单独构造反例**, 否则它就只是注释里的一句好听话。所以每条判据都做成
//   可枚举的拒绝理由(AdmitVerdict), 而且每条都能被单独关掉(checkFlags) —— 关掉之后反例必须
//   "漏过去", 这正是 test/sched_check 里成对断言的东西。
//
// ---- 为什么调度器只吃参数, 不认识 cfg:: ----
//   时隙长度/超帧结构/guard 是**两端必须一致**的运行时约定(§3.8 的 profile 切换不重编译), 而且
//   遥控器工程也包含本文件 —— 它连飞控的 cfg:: 生成头都拿不到。所以参数由调用方从 cfg:: 填进来,
//   本文件里**一个 4/80Hz/20Hz/10ms 都不出现**(单测里出现, 因为单测要拿方案 §3.5 的真值去对拍)。
//
// ---- 时基: 为什么全部用无符号差值 ----
//   微秒计数器 32 位: 0xFFFFFFFF µs ≈ 71.6 分钟就回绕一次。写 `if (now > deadline)` 在回绕点上
//   会静默变成"永远不到点"(或者永远到点), 而且只在飞行 71 分钟以后才发作。本文件所有时刻比较
//   都走 timeReached()/timeSince()(无符号差值), 前提是两个时刻相差 < 2^31 µs(≈35.8 分钟) ——
//   对"槽长 10ms 级"的调度来说这个前提永远成立。
//
// ---- 最小用法(与代码逐字一致; 具体的数由 cfg:: 侧填) ----
//   dlx::link::SchedParams  sp = {};                       // 调度参数(全部字段必填, 没有默认值)
//   sp.slotUs=10000; sp.uplinkSlots=4; sp.downlinkSlot=4;  // 10ms 槽, 4 上 1 下        ⇒ §3.5
//   sp.guardUs=3000; sp.windowDelayUs=5700;                // 规则③ 的 3ms; 窗口延时算法见 windowDelayFromLatency()
//   sp.silentSlotsToStop=2; sp.goodSlotsToResume=2;        // §3.5 规则⑤
//   sp.seqMode = dlx::link::MasterSeqMode::PerUplinkFrame; // 主端 SEQ 语义(矛盾点 A, 见文件尾)
//   dlx::link::SchedSlave sd;          sd.init(sp);
//   dlx::link::FrameAdmission ad;      ad.init(al);        // al: 允许类型位掩码 + 量级上限(cfg:: 填)
//   dlx::link::DownlinkPlanner dp;     dp.init(dp16_4_2_1);
//   if (ad.admit(raw, len, fr, nowUs) == dlx::link::AdmitVerdict::Ok) { sd.onMasterFrame(nowUs, fr.seq); }
//   sd.poll(nowUs);
//   if (sd.canStartTx(nowUs, auxIsHigh())) { const dlx::link::DownlinkFrame f = dp.next(); sendFrame(f); }
//
// C++11 / 无异常 / 无 RTTI / 无动态分配 / header-only。
//===================================================================================================

#include <stdint.h>
#include "dlx_link_frame.hpp" // 同目录: 帧类型/帧长表/tryDecode/seqGap/crc8Atm/bits::
// 注意: 本文件**刻意不 include 任何业务头**(曾经 include 过 dlx_system_state.hpp, 见方案 §4)。

namespace dlx
{
namespace link
{

    //===============================================================================================
    // 一、时基(无符号差值, 回绕安全)
    //===============================================================================================
    /** 距上次事件的微秒数(回绕安全): 即使 thenUs 在 0xFFFFFF00 附近、nowUs 已经绕到 0x00000100,
     *  无符号减法给出的仍然是正确的小正数。**不要**写成"先减再和负数比大小"。 */
    inline uint32_t timeSince(uint32_t nowUs, uint32_t thenUs)
    {
        return (uint32_t)(nowUs - thenUs);
    }

    /** 到点了吗(回绕安全)。前提: |nowUs - deadlineUs| < 2^31 µs(≈35.8 分钟), 见文件头。
     *  为什么写成 `差值 < 2^31` 而不是 `(int32_t)(now - deadline) >= 0`: 后者依赖"无符号转有符号
     *  是补码回绕"这一实现定义行为; 前者是纯无符号运算, AC6 与 g++ 的语义完全一致。 */
    inline bool timeReached(uint32_t nowUs, uint32_t deadlineUs)
    {
        return (uint32_t)(nowUs - deadlineUs) < 0x80000000u;
    }

    /** 还剩多少微秒(已到点则 0)。给"还要等多少微秒"这种查询用 */
    inline uint32_t timeLeft(uint32_t nowUs, uint32_t deadlineUs)
    {
        return timeReached(nowUs, deadlineUs) ? 0u : (uint32_t)(deadlineUs - nowUs);
    }

    //===============================================================================================
    // 二、时分参数(全部由调用方从配置填, 本文件不出现任何具体数字)
    //===============================================================================================
    /** 主端(遥控器)的 SEQ 到底怎么数 —— 这不是风格问题, 它决定"哪个主端帧之后轮到我发"。
     *
     *  依据 §3.5 的时隙图: `槽0 u 槽1 u 槽2 u 槽3 u 槽4 d`, 从端窗口紧跟主端**最后一个上行时隙**
     *  (槽 3)之后。所以从端真正需要回答的只有一句: "刚收到的这一帧, 是不是主端超帧里的最后一个上行帧?"
     *  SEQ 只有 4 位、超帧有 5 个槽, 而 16 与 5 互质 —— 见文件尾"矛盾点 A": 方案原文写的
     *  "槽号 = 上行 SEQ 模 5" 在 SEQ 回绕后必然错位。因此这里把两种可能的语义都显式列出来,
     *  由接线处按主端实际实现选定, 而不是替主端猜。 */
    enum class MasterSeqMode : uint8_t
    {
        /** 主端每**发一帧** SEQ+1(§3.2 "SEQ 低 4 位: 方向内自增计数器(模 16)") ⇒ 用 seq % N 定位。
         *  4 与 16 有整除关系, 回绕不破坏相位 —— 默认取这个。 */
        PerUplinkFrame = 0,
        /** §3.5 字面写法: SEQ 是"槽计数器"(主端在自己的下行槽里也让计数器走一格, 只是不发) ⇒ seq % (N+1)。
         *  这一支在第 16 个槽之后必然错位(推导见文件尾), 保留它只为"照方案字面实现"可选项与对拍。 */
        PerSlotCounter = 1,
    };

    /** 从端调度参数。**没有默认值**: 时隙长度/超帧结构是两端必须一致的约定, 让编译器替调用方
     *  挑一个默认值, 等于把"两端不一致"这种事故变成静默行为。填错会在 init() 里被判非法。
     *
     *  超帧结构与"主端占哪几个时隙": 主端占 0..N 里**除去本机那一个**的全部时隙(§3.5 的 4 上 1 下),
     *  本机时隙号 = downlinkSlot。超帧长(帧周期)不是独立参数, 而是派生量 (N+1)*slotUs —— 见
     *  superframeSlots()。为什么不做成"主端时隙号数组": §3.5/§3.8 的四个 profile 都是"连续 N 个上行
     *  + 1 个下行", 数组会引入一个当前用不到的自由度, 而自由度就是"两端各填一份、然后不一致"的机会。
     *  将来真要不连续的时隙分配, 再把这个参数扩成位掩码(改动点只有 masterFrameOpensWindow 与
     *  失同步判据两处)。
     */
    struct SchedParams
    {
        uint32_t      slotUs;            ///< 一个时隙的长度 [µs](§3.5 NORMAL: 10 ms)
        uint8_t       uplinkSlots;       ///< 一个超帧里主端占用的上行时隙数 N(§3.5: 4)
        uint8_t       downlinkSlot;      ///< 本机(从端)占用的时隙号, 0..N(§3.5: 4 = 最后一个)
        uint32_t      guardUs;           ///< 距最后一帧有效上行至少这么久才发车(§3.5 规则③: 3 ms)
        uint32_t      windowDelayUs;     ///< 自"最后一帧有效上行接收完毕"起, 到本机窗口开启的延时
        /** 连续丢这么多槽就静默只收不发(§3.5 规则⑤ 定 2, 配置项 link.tdma.kResyncSilentSlots)。
         *  为什么是 2 而不是 1: 单帧丢失在 80Hz 上就是常态(而且见 missToleranceUs —— 抖动本身
         *  不该触发静默), 为一次丢包停发一整个超帧是自伤。
         *  为什么是 2 而不是 3: 从端窗口紧跟主端**最后一个**上行帧, 而"连续 2 个槽没收到"已经意味着
         *  "开窗那一帧很可能就是丢的那一帧" —— 此时相位无法自证, 再等第三个槽只会多一次盲发风险。 */
        uint8_t       silentSlotsToStop;
        /** 连续收到这么多有效上行才恢复发送(§3.5 规则⑤ 定 2, 配置项 link.tdma.kResyncGoodSlots)。
         *  为什么必须 ≥2 且与静默阈值对称: 用 1 帧恢复等于"静默"形同虚设 —— 任何一帧噪声后脚就能
         *  解锁发送; 对称取 2 才能既快速恢复(2 个槽 = 20ms)又不被单帧偶发事件来回拨动(迟滞)。 */
        uint8_t       goodSlotsToResume;
        MasterSeqMode seqMode;           ///< 主端 SEQ 语义(矛盾点 A)
    };

    /** 超帧一共几个时隙 = N 个上行 + 1 个下行(§3.5: 4+1=5) */
    constexpr uint8_t superframeSlots(const SchedParams &p)
    {
        return (uint8_t)(p.uplinkSlots + 1u);
    }

    /** 抖动容差: 主端帧到达时刻的允许偏差。
     *  为什么取槽长的 1/4 而不是 0: 链路另一端的晶振/中断延迟会带来几百微秒抖动, 容差为 0 会把
     *  正常抖动误判成丢帧(然后静默 —— 一个"看起来像链路问题"的自伤)。取 1/4 槽长是与"丢帧判据
     *  至少要能容忍一个真实抖动"配套的: §3.5 的槽长 10ms ⇒ 容差 2.5ms, 远大于 80Hz 下任何合理
     *  的收发抖动, 又远小于一个槽(否则丢帧就数不出来了)。 */
    inline uint32_t missToleranceUs(const SchedParams &p)
    {
        return (uint32_t)(p.slotUs / 4u);
    }

    /** 参数自洽性。不合法就必须 init() 失败并保持"未初始化", 绝不带着半套参数跑调度。 */
    inline bool schedParamsValid(const SchedParams &p)
    {
        if (p.slotUs == 0u) {
            return false;
        }
        if (p.uplinkSlots < 1u || p.downlinkSlot > p.uplinkSlots) {
            return false; // 时隙号只有 0..N 这几个, 且至少要有一个上行槽
        }
        if (p.silentSlotsToStop < 1u || p.goodSlotsToResume < 1u) {
            return false; // 阈值为 0 表示"永远静默/永远不发", 不是配置意图
        }
        // guard 必须落在窗口里: guard ≥ 2 个槽长时"发车条件②"永远无法与窗口同时成立
        if (p.guardUs >= p.slotUs * 2u) {
            return false;
        }
        // 窗口必须在主端下一个上行帧开始之前关闭: 窗口 = [anchor+delay, anchor+delay+slotUs),
        // 主端下一个上行帧在 anchor+2*slotUs 开始 ⇒ delay ≤ slotUs 才不会压到它头上。
        if (p.windowDelayUs > p.slotUs) {
            return false;
        }
        // 窗口与 guard 不能互相抵消, 否则就是"窗口开着但永远等不到 guard" = 一直不发
        if (p.windowDelayUs + p.slotUs <= p.guardUs) {
            return false;
        }
        return true;
    }

    /** 窗口延时 = 槽长 − 主端一帧"从开始发到本机收完"的时延。
     *
     *  为什么需要这个换算: 相位锚点是"主端帧**收完**的时刻", 而时隙边界是"主端帧**开始发**的时刻";
     *  两者差一个收发时延(空口 + 串口)。时延估不准时只有一个方向是安全的:
     *  **时延取小 ⇒ 窗口延时不偏大 ⇒ 发得偏晚**。晚发只是缩短自己的窗口(最坏丢掉一次下行),
     *  早发则会压在还在到达的主端帧/模块收发的尾巴上(半双工, 两边都白丢)。所以估计时延要取
     *  偏小的一侧(例如认为空口与串口并行, 只算串口时间), 而不是"平均一下"。
     *  依据: §3.1 空口时间表(8B→2.90ms) + 57600 8N1 的 10bit/字节。 */
    inline uint32_t windowDelayFromLatency(uint32_t slotUs, uint32_t masterFrameLatencyUs)
    {
        if (masterFrameLatencyUs >= slotUs) {
            return 0u; // 时延估计已经超过一个槽: 说明参数填错了, 退化成"收到就开窗"
        }
        return (uint32_t)(slotUs - masterFrameLatencyUs);
    }

    /** "刚收到的这一帧之后, 就轮到本机发车了吗?" —— 从端相位的**唯一**判据(§3.5 规则②)。
     *
     *  语义: seq 标识的这个主端帧, 是否就是本机时隙**紧前面**的那一个主端上行帧。
     *  两种 SeqMode 的区别只在"怎么从 seq 定位它是第几个上行帧":
     *    PerUplinkFrame: 主端第 i 个上行帧(0..N-1)的 seq ≡ i (mod N) ⇒ 找 i == downlinkSlot-1(回绕后)
     *    PerSlotCounter: 槽号 ≡ seq (mod N+1)            ⇒ 找 (seq+1) mod (N+1) == downlinkSlot
     *  两者在 N=4/downlinkSlot=4 时分别退化成 "seq % 4 == 3" 与 "seq % 5 == 3" —— 第一个超帧里
     *  它们给出同一个答案(seq=3), 之后 PerSlotCounter 会漂(文件尾矛盾点 A 有推导, 单测有数字)。 */
    constexpr bool masterFrameOpensWindow(const SchedParams &p, uint8_t seq)
    {
        return (p.uplinkSlots < 1u)
                   ? false
                   : (p.seqMode == MasterSeqMode::PerUplinkFrame)
                         ? ((uint8_t)(seq % p.uplinkSlots) ==
                            (uint8_t)((p.downlinkSlot + p.uplinkSlots - 1u) % p.uplinkSlots))
                         : ((uint8_t)(((uint8_t)(seq % (p.uplinkSlots + 1u)) + 1u) % (p.uplinkSlots + 1u)) ==
                            (uint8_t)(p.downlinkSlot % (p.uplinkSlots + 1u)));
    }

    //===============================================================================================
    // 三、从端调度
    //===============================================================================================
    /** onMasterFrame() 的结果。为什么要把"没被采信"的几种情况分开: 它们的处置完全不同 ——
     *  重复帧只说明链路还活着(绝不能拿它改相位, 否则一次重放就能把窗口挪到主端槽里),
     *  离网格帧说明这一帧的**到达时刻**不可信(照单全收 = 让自己的相位被别人的抖动带偏),
     *  时间倒退说明时基出了大问题(必须忽略并上报, 而不是"当作一次很长的间隔")。 */
    enum class MasterFrameResult : uint8_t
    {
        Accepted       = 0, ///< 落在时隙网格上且 SEQ 连续(seqGap==1): 相位已更新
        AcceptedJump   = 1, ///< 落在网格上但有丢帧(seqGap>1): 相位已更新(丢包不累积错位, §3.5 规则②)
        Duplicate      = 2, ///< seqGap==0: 重复/重放帧 —— 只刷新"链路还活着", **不改相位窗口**
        OffGrid        = 3, ///< 到达时刻不在本机维护的时隙网格上(抖动过大/伪造): **不改相位窗口**
        TimeBackwards  = 4, ///< nowUs 早于上次锚点: 忽略(见下方注释)
        NotInitialized = 5, ///< 未 init() 就调用
    };

    inline const char *masterFrameResultName(MasterFrameResult r)
    {
        switch (r) {
        case MasterFrameResult::Accepted:       return "ACCEPTED";
        case MasterFrameResult::AcceptedJump:   return "ACCEPTED_JUMP";
        case MasterFrameResult::Duplicate:      return "DUPLICATE";
        case MasterFrameResult::OffGrid:        return "OFF_GRID";
        case MasterFrameResult::TimeBackwards:  return "TIME_BACKWARDS";
        case MasterFrameResult::NotInitialized: return "NOT_INIT";
        default:                                return "?";
        }
    }

    /**
     * 从端时分调度器。
     *
     * 它只做一件事: 在**正确的时候**允许发送, 并且在**说不准的时候**拒绝发送。
     * 它不发送、不读寄存器、不知道 AUX 是什么(③ 由调用方以 bool 传进来)。
     *
     * 相位模型(为什么不用"本地自由跑一个槽计数器"):
     *   每个被采信的主端帧都把相位**重新锚定**一次(anchorUs = 收到该帧的本地时刻)。从端的相位
     *   因此永远不会自由漂移: 丢包只会让"下一次锚定"晚一点到, 不会让窗口一点点挪进主端的槽里
     *   (§3.5 规则② 的"丢包不累积错位")。代价是"主端不说话时从端没有任何可信的窗口" —— 而这
     *   正好就是失同步保护要的行为(§3.5 规则⑤): 说不准就不发。
     */
    class SchedSlave
    {
    public:
        SchedSlave() : m_p(), m_init(false) { reset(); }

        /** 装载参数。参数不合法 ⇒ 返回 false 且保持"未初始化"(此时所有发送判据都返回 false) */
        bool init(const SchedParams &p)
        {
            reset();
            if (!schedParamsValid(p)) {
                return false;
            }
            m_p    = p;
            m_init = true;
            return true;
        }

        bool initialized() const { return m_init; }
        const SchedParams &params() const { return m_p; }

        // ------------------------------------------------------------------ 事件
        /** 喂一个**已通过准入前置**的主端帧(顺序很重要: 先 FrameAdmission::admit, 再喂这里)。
         *  为什么要强调顺序: 本函数只判断"相位能不能用", 不判断"这帧是不是对端发的"; 把没过滤的帧
         *  喂进来, CRC 合法的假帧就能把发送窗口挪到主端的时隙上(见文件头"为什么必须有准入门")。 */
        MasterFrameResult onMasterFrame(uint32_t nowUs, uint8_t seq)
        {
            if (!m_init) {
                return MasterFrameResult::NotInitialized;
            }
            if (m_haveAnchor && !timeReached(nowUs, m_anchorUs)) {
                /**
                 * 时基倒退 = 微秒计数器回绕(见 dlx_timebase.hpp): **相位已经不可考 ⇒ 丢弃相位、重新捕获**。
                 *
                 * ⚠ 旧写法只 `++m_timeBackwards` 就 return —— 在 nowUs 重新追上旧锚点之前(最长可达
                 *   计数器半程, 25.6 秒量级)每一帧都走这条路, 从端等于**永久失聪**。上板实测里
                 *   "链路正常约 20 秒后突然稳定超时"的另一半就是它(另一半在 FrameAdmission::admit)。
                 *
                 * 重新捕获的代价只有一帧: 这一帧按"首次捕获"处理(不查网格、不查 SEQ 连续性),
                 *   而**窗口**仍要等 seq%N==downlinkSlot-1 的那一帧才开 —— 不会因此乱发。
                 */
                ++m_timeBackwards;
                m_haveAnchor    = false;
                m_windowArmed   = false;
                m_expectedGapUs = 0u;
                m_goodStreak    = 0u;
                // 不 return: 落到下面的"首次捕获"路径
            }

            // 先按**旧**锚点算"这一帧之前丢了几个槽": 它是"连续"的唯一依据(§3.5 规则⑤要的是
            // 连续 N 帧有效上行, 不是"累计收到 N 帧")。放在这里而不是放在 poll() 里, 是为了让
            // "不调用 poll 的调用方"也不会退化成"隔了 1 秒来的第 1 帧就敢发车"。
            const uint8_t missedBefore = m_haveAnchor ? missedMasterSlots(nowUs) : (uint8_t)0u;
            if (missedBefore > 0u) {
                m_goodStreak = 0u; // 中间断过 ⇒ 连续性从头算
                if (missedBefore >= m_p.silentSlotsToStop && !m_mutedLatched) {
                    m_mutedLatched = true; // 迟到的这帧不能立刻解锁发送, 还要再连续来够 goodSlotsToResume 帧
                    ++m_desyncEvents;
                }
            }

            const uint8_t gap = m_haveAnchor ? seqGap(m_lastSeq, seq) : (uint8_t)1u;

            if (m_haveAnchor && gap == 0u) {
                // 同一个 SEQ 又来一次: 重放、回声, 或者"恰好丢了 16 帧"。都**不能**改相位 ——
                // 只有"链路还活着"这一条可以采信(§3.2 seqGap 注释: 丢满 16 帧会被看成重复帧)。
                ++m_duplicates;
                m_lastAliveUs = nowUs;
                m_goodStreak  = 0u; // 它不是一帧新的有效上行, 连续性断在这里
                return MasterFrameResult::Duplicate;
            }

            if (m_haveAnchor && !onSlotGrid(nowUs)) {
                // 到达时刻不在网格上: 相位不可信(主端被挤了一下, 或者一帧伪造数据正好落在
                // "最后一个上行帧"的 SEQ 上)。不改相位, 只记存活与计数。
                ++m_offGrid;
                m_lastAliveUs = nowUs;
                m_goodStreak  = 0u;
                return MasterFrameResult::OffGrid;
            }

            // ---- 采信: 重新锚定相位 ----
            const bool hadMiss   = m_haveAnchor && (gap != 1u);
            m_anchorUs           = nowUs;
            m_lastAliveUs        = nowUs;
            m_lastSeq            = seq;
            m_lastGap            = gap;
            m_haveAnchor         = true;
            m_windowArmed        = masterFrameOpensWindow(m_p, seq);
            m_windowOpenUs       = (uint32_t)(nowUs + m_p.windowDelayUs);
            m_windowCloseUs      = (uint32_t)(m_windowOpenUs + m_p.slotUs);
            // 下一个主端帧什么时候该到: 本机窗口插在中间的那一次要等两个槽, 其余等一个槽。
            m_expectedGapUs      = (uint32_t)(m_windowArmed ? (m_p.slotUs * 2u) : m_p.slotUs);
            ++m_accepted;
            if (hadMiss) {
                m_lostBySeq += (uint32_t)(gap - 1u);
            }
            // 连续有效上行的计数与"恢复发送"门槛(§3.5 规则⑤后半句)
            m_goodStreak = (m_goodStreak < 0xFFu) ? (uint8_t)(m_goodStreak + 1u) : (uint8_t)0xFFu;
            if (m_mutedLatched && m_goodStreak >= m_p.goodSlotsToResume) {
                m_mutedLatched = false;
                ++m_resyncEvents;
            }
            return hadMiss ? MasterFrameResult::AcceptedJump : MasterFrameResult::Accepted;
        }

        /** 周期性调用(建议每个控制周期, 至少每槽一次): 维护"静默/恢复"锁存与计数器。
         *
         *  为什么静默还需要锁存, 而不只看"现在距上次采信帧多久": 发送判据 canStartTx() 已经把时间
         *  条件算进去了, onMasterFrame() 也会在"迟到的帧"上补上锁存 —— 所以**不调用本函数也不会
         *  发错车**。锁存的作用是: ① 把失同步变成一个**及时**可上报的事件(LINK_DESYNC /
         *  LINK_RESYNCED), 而不是等下一帧到了才知道; ② 让"恢复"必须是连续 N 帧有效上行
         *  (§3.5 规则⑤), 而不是"时间上刚好够久"。 */
        void poll(uint32_t nowUs)
        {
            if (!m_init) {
                return;
            }
            ++m_polls;
            if (!m_mutedLatched && m_haveAnchor && missedMasterSlots(nowUs) >= m_p.silentSlotsToStop) {
                m_mutedLatched = true;
                m_goodStreak   = 0u; // 进入静默时"连续"重新开始算: 恢复必须是静默**之后**的连续 N 帧
                ++m_desyncEvents;
            }
            if (m_mutedLatched && m_goodStreak >= m_p.goodSlotsToResume) {
                m_mutedLatched = false;
                ++m_resyncEvents;
            }
        }

        // ------------------------------------------------------------------ 相位查询
        /** 是否已经有可信相位(收到过至少 1 个采信帧)。注意它**不代表现在还能发**: 见 synced() */
        bool phaseValid() const { return m_init && m_haveAnchor; }

        /** "现在同步着吗" = 有相位 且 失同步计数还没到静默阈值。等价于 §3.5 规则⑤的前半句 */
        bool synced(uint32_t nowUs) const
        {
            return phaseValid() && missedMasterSlots(nowUs) < m_p.silentSlotsToStop;
        }

        /** 是否处于静默(只收不发)。两个来源取"更保守"的那个:
         *  ① 时间条件(距上次采信帧已跨过 silentSlotsToStop 个槽)—— 不依赖 poll;
         *  ② poll() 锁存的失同步事件 —— 它只在"连续 goodSlotsToResume 帧有效上行"后才解除。 */
        bool muted(uint32_t nowUs) const
        {
            return m_mutedLatched || (phaseValid() && missedMasterSlots(nowUs) >= m_p.silentSlotsToStop);
        }

        /** 自上次采信帧以来**已经该到而没到**的主端帧数(0..15, 饱和)。
         *
         *  推导: 主端帧都落在 anchor + k*slotUs 的网格上(同一条链路里每帧的收发时延相同, 所以相邻
         *  到达间隔正好是槽长的整数倍)。因此在 anchor + expectedGap + 容差 之前一律算 0; 之后每个
         *  槽长加 1。容差见 missToleranceUs(): 没有它, ±5% 的抖动会被数成丢帧。
         *  为什么饱和在 15: 这个数只用来驱动"静默/上报", 数到 15 已经远超任何阈值(§3.9 的
         *  WARN=300ms 对应 30 个槽), 再往上是给日志添乱。 */
        uint8_t missedMasterSlots(uint32_t nowUs) const
        {
            if (!phaseValid()) {
                return 0u;
            }
            const uint32_t since    = timeSince(nowUs, m_anchorUs);
            const uint32_t deadline = (uint32_t)(m_expectedGapUs + missToleranceUs(m_p));
            if (since < deadline) {
                return 0u;
            }
            const uint32_t n = (uint32_t)(1u + (uint32_t)((since - deadline) / m_p.slotUs));
            return (n > 15u) ? (uint8_t)15u : (uint8_t)n;
        }

        uint8_t  lastSeq() const { return m_lastSeq; }
        uint32_t lastMasterRxUs() const { return m_lastAliveUs; }
        /** 相对上一次采信帧的 SEQ 距离(1 = 连续, >1 = 中间丢了, 0 = 重复帧) */
        uint8_t  lastSeqGap() const { return m_lastGap; }
        /** 最后一帧主端帧是否打开了本机窗口(即它是不是主端超帧的最后一个上行帧) */
        bool     windowArmed() const { return m_windowArmed; }

        // ------------------------------------------------------------------ 发送窗口查询
        /** 本机窗口开启的绝对时刻 [µs](仅 windowArmed() 为 true 时有意义) */
        uint32_t windowOpenUs() const { return m_windowArmed ? m_windowOpenUs : 0u; }
        /** 本机窗口关闭的绝对时刻 [µs] = 主端下一个上行帧开始发的时刻(过了就不能再发) */
        uint32_t windowCloseUs() const { return m_windowArmed ? m_windowCloseUs : 0u; }
        /** 还要等多少微秒才到窗口(0 = 已经到点, 或本轮没有窗口) */
        uint32_t usUntilWindowOpen(uint32_t nowUs) const
        {
            return m_windowArmed ? timeLeft(nowUs, m_windowOpenUs) : 0u;
        }
        /** 现在正处于本机窗口内吗(纯时间包含判断, 与是否被静默无关) */
        bool windowIsOpen(uint32_t nowUs) const
        {
            return m_windowArmed && timeReached(nowUs, m_windowOpenUs) && !timeReached(nowUs, m_windowCloseUs);
        }
        /** 本机**最早**可以开始发送的时刻 = max(窗口开启, 最后一帧有效上行 + guard)。
         *  这就是 §3.5 规则③ 的①②两条合并后的结果: 相位到窗口(①) 且 距最后一帧有效上行 ≥ guard(②)。
         *  返回 0 表示本轮没有窗口。 */
        uint32_t nextTxUs() const
        {
            if (!m_windowArmed) {
                return 0u;
            }
            const uint32_t guardDeadline = (uint32_t)(m_anchorUs + m_p.guardUs);
            return timeReached(guardDeadline, m_windowOpenUs) ? guardDeadline : m_windowOpenUs;
        }

        /** 可以开始发送了吗 —— §3.5 规则③ 的 ①②(相位到窗口 + guard 满) 加上失同步保护。
         *  规则③的 ③(AUX 为高)是硬件信号, 用下面那个带 rfIdle 的重载传进来。 */
        bool canStartTx(uint32_t nowUs) const
        {
            if (!m_windowArmed || m_mutedLatched || !synced(nowUs)) {
                return false;
            }
            if (!timeReached(nowUs, nextTxUs())) {
                return false; // 还没到"窗口开 + guard 满"的时刻
            }
            if (timeReached(nowUs, m_windowCloseUs)) {
                return false; // 窗口已经关了: 再发就压到主端下一个上行帧上
            }
            return true;
        }

        /** 规则③的完整形式: ①②③ 全满足才允许发车。rfIdle = AUX 表示射频空闲(E220 UAUX=1 时有效)。 */
        bool canStartTx(uint32_t nowUs, bool rfIdle) const
        {
            return rfIdle && canStartTx(nowUs);
        }

        // ------------------------------------------------------------------ 统计(上报/日志用)
        uint32_t acceptedCount() const { return m_accepted; }
        uint32_t duplicateCount() const { return m_duplicates; }
        uint32_t offGridCount() const { return m_offGrid; }
        uint32_t timeBackwardsCount() const { return m_timeBackwards; }
        uint32_t desyncCount() const { return m_desyncEvents; }
        uint32_t resyncCount() const { return m_resyncEvents; }
        uint32_t pollCount() const { return m_polls; }
        uint8_t  goodStreak() const { return m_goodStreak; }

        /** 上行丢包率(0..100, 直接可填 DL_STATE.ulLoss —— §3.6 屏幕行 "UL:00%")。
         *  为什么用 SEQ 距离而不是"数时间槽": 时间法会把主端没发(而不是丢了)的槽也算成丢包;
         *  SEQ 法量的是"主端发了多少个、我们收到多少个"。
         *  已知代价: SEQ 只有 4 位, 一次丢满 16 帧会被看成 0(重复帧), 于是那一圈不计入丢失 ——
         *  80Hz 下 16 帧 = 200ms, 那时 §3.9 的 WARN(300ms)/失控保护早就介入了, 所以不另设圈数状态。 */
        uint8_t uplinkLossPercent(uint32_t nowUs) const
        {
            const uint32_t lost = m_lostBySeq + (uint32_t)missedMasterSlots(nowUs);
            const uint32_t all  = m_accepted + lost;
            if (all == 0u) {
                return 0u;
            }
            uint32_t pct = (uint32_t)((lost * 100u) / all);
            if (pct > 100u) {
                pct = 100u;
            }
            return (uint8_t)pct;
        }

        /** 回到"刚上电"的状态(参数保留)。为什么留这个: 链路 profile 切换(§3.8)后槽长/结构全变,
         *  旧相位必须整体丢弃, 而不是"慢慢适应"。 */
        void reset()
        {
            m_haveAnchor       = false;
            m_windowArmed      = false;
            m_mutedLatched     = false;
            m_anchorUs         = 0u;
            m_windowOpenUs     = 0u;
            m_windowCloseUs    = 0u;
            m_expectedGapUs    = 0u;
            m_lastAliveUs      = 0u;
            m_lastSeq          = 0u;
            m_lastGap          = 0u;
            m_goodStreak       = 0u;
            m_accepted         = 0u;
            m_duplicates       = 0u;
            m_offGrid          = 0u;
            m_timeBackwards    = 0u;
            m_desyncEvents     = 0u;
            m_resyncEvents     = 0u;
            m_polls            = 0u;
            m_lostBySeq        = 0u;
        }

    private:
        /** 到达时刻是否落在"anchor + k*slotUs"的网格上(容差 missToleranceUs)。
         *  为什么要这条判据(而不只是看 SEQ): SEQ 只说明"这是主端第几个上行帧", 到达**时刻**才是窗口
         *  的依据。一次被挤晚的主端帧如果照单全收, 就会把本机窗口一起拖到主端下一个上行槽里 ——
         *  即"自己的相位被别人的抖动带偏"。反过来, 网格判据不会拒绝真实的丢帧重同步: 丢 k 帧后的
         *  到达时刻仍然是网格点(相差正好 k 个槽长), 只是 k 更大而已。 */
        bool onSlotGrid(uint32_t nowUs) const
        {
            const uint32_t since = timeSince(nowUs, m_anchorUs);
            if (since > (uint32_t)0x7FFFFFFFu) {
                // 时基跨过半程(35.8 分钟): 网格关系已经不可考。只有"本来就已失同步"时才允许
                // 当作重新捕获 —— 否则一次远跳就能把相位带到一个随意的地方。
                return missedMasterSlots(nowUs) >= m_p.silentSlotsToStop;
            }
            const uint32_t k     = (uint32_t)((since + (m_p.slotUs / 2u)) / m_p.slotUs); // 最近的网格点
            const uint32_t ideal = (uint32_t)(k * m_p.slotUs);
            const uint32_t diff  = (since > ideal) ? (uint32_t)(since - ideal) : (uint32_t)(ideal - since);
            return diff <= missToleranceUs(m_p);
        }

        SchedParams m_p;
        bool        m_init;

        bool     m_haveAnchor;    ///< 是否已有相位锚点
        bool     m_windowArmed;   ///< 当前锚点是否打开了本机窗口
        bool     m_mutedLatched;  ///< poll() 锁存的失同步(只能在连续 N 帧有效上行后解除)
        uint32_t m_anchorUs;      ///< 相位锚点: 最后一次采信的主端帧的本地到达时刻
        uint32_t m_windowOpenUs;  ///< 本机窗口开启时刻
        uint32_t m_windowCloseUs; ///< 本机窗口关闭时刻
        uint32_t m_expectedGapUs; ///< 下一帧主端帧距锚点的期望间隔(1 或 2 个槽长)
        uint32_t m_lastAliveUs;   ///< 最后一次"链路还活着"的时刻(含重复帧)
        uint8_t  m_lastSeq;       ///< 最后一次采信帧的 SEQ
        uint8_t  m_lastGap;       ///< 最后一次采信帧相对上一帧的 seqGap
        uint8_t  m_goodStreak;    ///< 连续有效上行帧数
        uint32_t m_accepted;      ///< 采信帧总数
        uint32_t m_duplicates;    ///< 重复帧数
        uint32_t m_offGrid;       ///< 离网格帧数
        uint32_t m_timeBackwards; ///< 时基倒退次数
        uint32_t m_desyncEvents;  ///< 进入静默次数(对应 LINK_DESYNC)
        uint32_t m_resyncEvents;  ///< 恢复发送次数(对应 LINK_RESYNCED)
        uint32_t m_polls;         ///< poll() 调用次数
        uint32_t m_lostBySeq;     ///< 由 SEQ 距离累加出来的丢失帧数
    };

    //===============================================================================================
    // 四、下行槽内容轮转(§3.5 "下行槽内容轮转" + §3.3 帧表频率)
    //===============================================================================================
    /** 轮转周期(单位: 下行槽数)。为什么把它和调度器分开放: 调度器回答"什么时候能发", 轮转回答
     *  "发哪种"; 两者只有"每个下行槽调用一次 next()"这一个耦合点, 分开以后"槽长/profile 变了"
     *  不会碰到轮转逻辑。 */
    struct DownlinkParams
    {
        uint8_t gpsEvery;   ///< 每这么多个下行槽发一次 DL_GPS(§3.5: 16 ⇒ 1.25Hz)
        uint8_t sysEvery;   ///< 每这么多个下行槽发一次 DL_SYS(§3.5: 4  ⇒ 3.75Hz)
        uint8_t stateEvery; ///< 每这么多个下行槽发一次 DL_STATE(§3.5: 2 ⇒ 5Hz)
        uint8_t attEvery;   ///< 其余槽发 DL_ATT(§3.5: 1 ⇒ 10Hz)
    };

    inline bool downlinkParamsValid(const DownlinkParams &p)
    {
        return p.gpsEvery >= 1u && p.sysEvery >= 1u && p.stateEvery >= 1u && p.attEvery >= 1u;
    }

    /** 一个下行槽要发的东西: 类型 + 本方向自增的 SEQ + 帧长。
     *  为什么把"帧长"一起给出去: §3.5 规则④"飞控发送长度必须固定", 而长度只由类型决定
     *  (frameBytes), 让调用方每次自己查表就多一个查错的机会。 */
    struct DownlinkFrame
    {
        FrameType type;
        uint8_t   seq;
        uint16_t  bytes;
    };

    /**
     * 下行槽轮转器。§3.5 的规则就是"按优先级取第一个满足 d % Every == 0 的", 越稀有的越优先:
     *   d % 16 == 0 → DL_GPS(1.25Hz)   d % 4 == 0 → DL_SYS(3.75Hz)
     *   d % 2  == 0 → DL_STATE(5Hz)    其余        → DL_ATT(10Hz)
     * 16 帧里它们分别是 1 / 3 / 4 / 8 次(注意 d=0 归 GPS, 所以 SYS 是 3 次而不是 4 次), 除以
     * 20Hz 的下行槽率正好是 §3.3 帧表里的 1.25 / 3.75 / 5 / 10 Hz —— 单测里有这条频率对拍。
     */
    class DownlinkPlanner
    {
    public:
        DownlinkPlanner() : m_p(), m_init(false), m_count(0u), m_seq(0u) { reset(); }

        bool init(const DownlinkParams &p)
        {
            reset();
            if (!downlinkParamsValid(p)) {
                return false;
            }
            m_p    = p;
            m_init = true;
            return true;
        }

        bool initialized() const { return m_init; }

        /** 取下一个下行帧并前进一格。**每个下行槽只调用一次**(它同时推进轮转与 SEQ)。 */
        DownlinkFrame next()
        {
            const DownlinkFrame f = peek();
            ++m_count;
            ++m_typeCount[(uint8_t)f.type & 0x0Fu];
            m_seq = (uint8_t)((m_seq + 1u) & 0x0Fu); // §3.2: 方向内自增, 模 16
            return f;
        }

        /** 看一眼下一个是什么(不前进) —— 给"发之前要先知道长度"的调用方(如 DMA 长度) */
        DownlinkFrame peek() const
        {
            DownlinkFrame f;
            f.type  = pick(m_count);
            f.seq   = m_seq;
            f.bytes = frameBytes(f.type);
            return f;
        }

        uint32_t frameCount() const { return m_count; }
        uint32_t typeCount(FrameType t) const { return m_typeCount[(uint8_t)t & 0x0Fu]; }

        void reset()
        {
            for (uint8_t i = 0u; i < 16u; ++i) {
                m_typeCount[i] = 0u;
            }
            m_count = 0u;
            m_seq   = 0u;
        }

    private:
        FrameType pick(uint32_t d) const
        {
            if (!m_init) {
                // 未初始化就取帧: 只给最短的那一型, 且**绝不**拿 0 当除数(除零是 UB, 不是"返回 0")
                return FrameType::DlAtt;
            }
            if ((uint32_t)(d % m_p.gpsEvery) == 0u) {
                return FrameType::DlGps;
            }
            if ((uint32_t)(d % m_p.sysEvery) == 0u) {
                return FrameType::DlSys;
            }
            if ((uint32_t)(d % m_p.stateEvery) == 0u) {
                return FrameType::DlState;
            }
            return FrameType::DlAtt;
        }

        DownlinkParams m_p;
        bool           m_init;
        uint32_t       m_count;         ///< 已经取走的帧数 d(从 0 开始 ⇒ 第一帧是 DL_GPS)
        uint8_t        m_seq;           ///< 下行 SEQ(§3.2: 方向内自增, 模 16)
        uint32_t       m_typeCount[16]; ///< 每型帧计数(下标 = 类型号), 用于频率对拍/日志
    };

    //===============================================================================================
    // 五、线上编号(协议事实)
    //
    //   这个库里**没有** CommandOp / GroundStatus 这类枚举: 它们是**业务词汇**(哪条命令对应
    //   哪个动作、无线能不能发), 属于 app 层(见 app_link_cmd.hpp)。链路库是与业务无关、可迁移
    //   的基础代码 —— 遥控器侧也包含同一批头 —— 它只该知道"协议表里有哪些字节", 不该知道
    //   "ARM 是什么"。所以这里只有编号 + "这个字节在不在表里"。
    //
    //   为什么仍然是一张**显式表**而不是范围判断: 0x00..0x73 里有大段空号, "落在范围内"不等于
    //   "在表里"。未知 op 必须是**拒绝**, 不许当成"最近的那个合法值"。
    //===============================================================================================
    enum : uint8_t
    {
        kWireOpPing             = 0x00, ///< 探链路: **故障态也要能回**, 否则地面端分不清"飞机死了"和"链路死了"
        kWireOpArm              = 0x01,
        kWireOpDisarm           = 0x02,
        kWireOpEmergencyStop    = 0x03,
        kWireOpSetMode          = 0x10, ///< arg = 模式号 0..kWireModeMax
        kWireOpTakeoff          = 0x20, ///< arg = 目标高度 [dm]
        kWireOpLand             = 0x21,
        kWireOpCancelTask       = 0x22,
        kWireOpCalStart         = 0x30,
        kWireOpCalAbort         = 0x31,
        kWireOpParamGet         = 0x40,
        kWireOpParamSet         = 0x41,
        kWireOpParamList        = 0x42,
        kWireOpLinkSetProfile   = 0x50,
        kWireOpProtoVersionGet  = 0x51,
        kWireOpEnterMaintenance = 0x60, ///< arg = 维护种类 0..kWireServiceMax
        kWireOpExitMaintenance  = 0x61,
        kWireOpLogList          = 0x70,
        kWireOpLogDump          = 0x71,
        kWireOpLogErase         = 0x72,
        kWireOpLogFormat        = 0x73,
    };

    /** `SET_MODE.arg` 的上界: 四个飞行模式(0 = Manual … 3 = VelHold) */
    constexpr uint8_t kWireModeMax = 3u;

    /** `ENTER_MAINTENANCE.arg` 的上界: 维护种类 0..3(§3.6) */
    constexpr uint8_t kWireServiceMax = 3u;

    /** `DL_STATE.failsafeLvl` 的上界: 0..4(None/Warn/Degrade/HoverLand/Cut) */
    constexpr uint8_t kWireFailsafeLvlMax = 4u;

    /** `DL_STATE.failsafeRsn` 的上界: 0..8(None..LogFull)。线上 4 位, 装得下 0..15 */
    constexpr uint8_t kWireFailsafeRsnMax = 8u;

    /** DL_ACK.result 的合法上界: 0..11(§3.7 结果码表)。12..255 是**非法值**, 必须拒 ——
     *  它们既不能当 OK, 也不能当成"最近的那个合法拒绝码"。 */
    constexpr uint8_t kWireResultMax = 11u;

    /** 这个 op 字节在协议表里吗 */
    inline bool isKnownWireOp(uint8_t wireOp)
    {
        switch (wireOp) {
        case kWireOpPing:
        case kWireOpArm:
        case kWireOpDisarm:
        case kWireOpEmergencyStop:
        case kWireOpSetMode:
        case kWireOpTakeoff:
        case kWireOpLand:
        case kWireOpCancelTask:
        case kWireOpCalStart:
        case kWireOpCalAbort:
        case kWireOpParamGet:
        case kWireOpParamSet:
        case kWireOpParamList:
        case kWireOpLinkSetProfile:
        case kWireOpProtoVersionGet:
        case kWireOpEnterMaintenance:
        case kWireOpExitMaintenance:
        case kWireOpLogList:
        case kWireOpLogDump:
        case kWireOpLogErase:
        case kWireOpLogFormat:
            return true;
        default:
            return false;
        }
    }

    /** 这个 result 字节在协议表里吗(范围检查即可: 结果码是密集的 0..11) */
    inline bool isKnownWireResult(uint8_t wireResult)
    {
        return wireResult <= kWireResultMax;
    }

    //===============================================================================================
    // 七、帧准入前置("这帧能不能被采信")
    //===============================================================================================
    /** 拒绝理由。每一条都对应一个**独立的判据**, 所以每一条都能被单独构造反例(单测要求)。
     *  为什么不做成 bool: 现场排查时"帧被拒了"这句话没有用, "被哪一条拒的"才能定位
     *  (是 SEQ 重放? 是量级离谱? 还是状态不允许?)。 */
    enum class AdmitVerdict : uint8_t
    {
        Ok = 0,
        NotInitialized,        ///< 没 init() 就调用
        RawLenMismatch,        ///< 原始长度/帧头与帧表对不上(调用方切帧切错了)
        TypeNotAllowed,        ///< 帧类型不在本角色允许的集合里(飞控收到 DL_GPS 之类)
        TooSoon,               ///< 距上一帧采信帧太近, 或到达时刻倒退(TDMA 下两帧不可能挤在一起)
        SeqDuplicate,          ///< SEQ 与上一帧相同(重放/回声; 也覆盖"恰好丢满 16 帧"的歧义)
        SeqJumpTooBig,         ///< SEQ 跳得太远, 不像同一串上行(拒一次并转入重新捕获)
        UnknownOp,             ///< UP_CMD.op 不在命令表里
        CmdArgOutOfRange,      ///< UP_CMD.arg 超出该命令的取值域(如 SET_MODE 给了 7)
        UnknownResult,         ///< DL_ACK.result 不在结果码表里
        QuatNormOutOfRange,    ///< DL_ATT: q1²+q2²+q3² > 1(单位四元数不可能)
        SpeedOutOfRange,       ///< 速度超出物理量级
        PositionOutOfRange,    ///< 位置/高度超出物理量级
        VoltageOutOfRange,     ///< DL_SYS 电压超出 [min,max]
        CurrentOutOfRange,     ///< DL_SYS 电流超上限
        MotorOutOfRange,       ///< DL_SYS 电机计数 > DShot 可用上限(2000..2047 是特殊命令区)
        GpsLatOutOfRange,      ///< DL_GPS 纬度超 ±90°
        GpsLonOutOfRange,      ///< DL_GPS 经度超 ±180°
        GpsCourseOutOfRange,   ///< DL_GPS 航向 > 359.99°(u16 能表示 655.35°, 所以这条有牙)
        GpsSpeedOutOfRange,    ///< DL_GPS 地速为负(物理上不存在)
        HdopOutOfRange,        ///< DL_GPS HDOP 超上限
        LossOutOfRange,        ///< DL_STATE 丢包率 > 100%(kLossMaxPercent)
        ModeOutOfRange,        ///< DL_STATE.mode 超出线上模式值域(0..kWireModeMax; 线上是 3 位, 装得下 0..7)
        FailsafeLvlOutOfRange, ///< DL_STATE.failsafeLvl 超出 FailsafeLevel 值域
        FailsafeRsnOutOfRange, ///< DL_STATE.failsafeRsn 超出 FailsafeReason 值域
        SpareBitsNotZero,      ///< DL_STATE 的 7 个保留位不为 0(§3.3 更正 1: 编码时恒写 0)
        StickCodeUnused,       ///< UP_CTRL 摇杆用了 §3.3 明确不用的 -1024 码点(解码层已把它夹回 -1023,
                               ///< 所以只能看线上原始位 —— 出现它 = 不是本工程编出来的帧)
        StickStepTooBig,       ///< UP_CTRL 摇杆单帧变化超过物理可能(人手/舵机都做不到)
        Count                  ///< 哨兵: 数组大小用, 不是判据
    };

    inline const char *admitVerdictName(AdmitVerdict v)
    {
        switch (v) {
        case AdmitVerdict::Ok:                    return "OK";
        case AdmitVerdict::NotInitialized:        return "NOT_INIT";
        case AdmitVerdict::RawLenMismatch:        return "RAW_LEN";
        case AdmitVerdict::TypeNotAllowed:        return "TYPE";
        case AdmitVerdict::TooSoon:               return "TOO_SOON";
        case AdmitVerdict::SeqDuplicate:          return "SEQ_DUP";
        case AdmitVerdict::SeqJumpTooBig:         return "SEQ_JUMP";
        case AdmitVerdict::UnknownOp:             return "OP";
        case AdmitVerdict::CmdArgOutOfRange:      return "ARG";
        case AdmitVerdict::UnknownResult:         return "RESULT";
        case AdmitVerdict::QuatNormOutOfRange:    return "QUAT";
        case AdmitVerdict::SpeedOutOfRange:       return "SPEED";
        case AdmitVerdict::PositionOutOfRange:    return "POS";
        case AdmitVerdict::VoltageOutOfRange:     return "VOLT";
        case AdmitVerdict::CurrentOutOfRange:     return "CURR";
        case AdmitVerdict::MotorOutOfRange:       return "MOTOR";
        case AdmitVerdict::GpsLatOutOfRange:      return "LAT";
        case AdmitVerdict::GpsLonOutOfRange:      return "LON";
        case AdmitVerdict::GpsCourseOutOfRange:   return "COURSE";
        case AdmitVerdict::GpsSpeedOutOfRange:    return "GSPD";
        case AdmitVerdict::HdopOutOfRange:        return "HDOP";
        case AdmitVerdict::LossOutOfRange:        return "LOSS";
        case AdmitVerdict::ModeOutOfRange:        return "MODE";
        case AdmitVerdict::FailsafeLvlOutOfRange: return "FS_LVL";
        case AdmitVerdict::FailsafeRsnOutOfRange: return "FS_RSN";
        case AdmitVerdict::SpareBitsNotZero:      return "SPARE";
        case AdmitVerdict::StickCodeUnused:       return "STICK_CODE";
        case AdmitVerdict::StickStepTooBig:       return "STICK_STEP";
        default:                                  return "?";
        }
    }

    /** 判据开关(位掩码)。为什么允许关: ① 单测要能构造"关掉这条判据, 反例就会漏过去"的对照;
     *  ② 现场排查时可以先关一条, 确认是不是它误杀。**默认全开**(AdmissionLimits 里显式填)。 */
    enum : uint8_t
    {
        kCheckType    = 0x01, ///< 帧类型是否在本角色允许集合里
        kCheckSeq     = 0x02, ///< SEQ 窗口(重复/跳变)与最小到达间隔
        kCheckRange   = 0x04, ///< 关键字段物理量级
        kCheckCmdArg  = 0x08, ///< 命令帧的 op 在不在表里 / arg 在不在取值域(**不含"状态允不允许"**)
        kCheckAll     = 0x0F,
    };

    /** 类型位掩码: bit n = 帧类型号 n。角色允许集合就是几个这样的位或起来 */
    constexpr uint16_t typeMaskOf(FrameType t)
    {
        return (uint16_t)(1u << ((uint8_t)t & 0x0Fu));
    }

    /** 从端(飞控)允许收到的类型: 只有上行帧。收到下行帧 = 要么是回声, 要么是假帧 */
    constexpr uint16_t kSlaveInboundMask =
        (uint16_t)(typeMaskOf(FrameType::UpCtrl) | typeMaskOf(FrameType::UpCmd) | typeMaskOf(FrameType::UpVel) |
                   typeMaskOf(FrameType::UpPos) | typeMaskOf(FrameType::UpSetpoint) | typeMaskOf(FrameType::UpMotor));
    /** 主端(遥控器)允许收到的类型: 只有下行帧 */
    constexpr uint16_t kMasterInboundMask = (uint16_t)(typeMaskOf(FrameType::DlAtt) | typeMaskOf(FrameType::DlSys) |
                                                       typeMaskOf(FrameType::DlGps) | typeMaskOf(FrameType::DlState) |
                                                       typeMaskOf(FrameType::DlAck));

    /** 量级上限。**所有上限为 0 = 不检查该项** —— 这样调用方只需要填它真正有物理依据的那几条,
     *  而不是被逼着给每个字段编一个数字。上限的取值必须是"这个飞行器物理上不可能超过"的量级,
     *  而不是"正常值"(正常值会把一次真实的急机动判成假帧)。 */
    struct AdmissionLimits
    {
        uint16_t allowedTypes;        ///< 允许的帧类型位掩码(用 kSlaveInboundMask / kMasterInboundMask)
        uint8_t  checks;              ///< kCheck* 位掩码
        uint8_t  maxSeqGapWhenSynced; ///< 同步状态下允许的最大 SEQ 跳变(0 = 不检查)
        uint32_t minFrameIntervalUs;  ///< 两帧采信帧之间的最小间隔(0 = 不检查; TDMA 下就是槽长的量级)
        float    maxSpeedMps;         ///< 速度上限: UP_VEL(三轴) / DL_ATT(0 = 不检查)
        float    maxPositionM;        ///< 位置/高度上限: UP_POS / UP_SETPOINT / DL_SYS(0 = 不检查)
        float    minVoltageV;         ///< DL_SYS 电压下界(与 maxVoltageV 同时 >0 才检查)
        float    maxVoltageV;         ///< DL_SYS 电压上界
        float    maxCurrentA;         ///< DL_SYS 电流上限
        uint16_t maxMotorCount;       ///< 电机计数上限: DL_SYS.motor 与 UP_MOTOR.throttle(0 = 不检查; §3.3: 0..1999)
        float    maxStickStepNorm;    ///< UP_CTRL 单帧杆量最大变化(归一化 0..2; 0 = 不检查)
        float    quatNormMax;         ///< DL_ATT 四元数虚部模长上限(取 1.0 + 一点余量; 0 = 不检查)
        uint8_t  maxLossPercent;      ///< DL_STATE 丢包率上限(0 = 不检查; 依据 kLossMaxPercent=100)
        float    maxHdop;             ///< DL_GPS HDOP 上限(0 = 不检查)
        uint8_t  maxTakeoffDm;        ///< UP_CMD(TAKEOFF) 目标高度上限 [dm](0 = 不检查)
    };

    /**
     * 帧准入前置。
     *
     * admit() 只回答一个问题: **"这一帧值不值得采信"**。它的输出被三处消费:
     *   ① 调度器: 只有 Ok 的帧才允许 onMasterFrame()(否则假帧能把发送窗口挪进主端时隙);
     *   ② 控制: UP_CTRL 的杆量只有 Ok 才刷新 setpoint(§3.9 "上行丢失时保持/温和外推");
     *   ③ 命令: UP_CMD 只有 Ok 才继续往下走 —— 但**状态允不允许是 app 层的事**(见方案 §4);
     *      本层的 Ok 只说明"这一帧像本工程的编码器编出来的"。
     *
     * 状态只在**采信**时前进(SEQ/杆量历史/时刻): 被拒的帧既不能改状态也不能改历史, 否则
     * "连续 N 帧有效上行"这种判据会被伪造帧喂饱(§3.6 解锁门槛要数够 8 帧)。
     */
    class FrameAdmission
    {
    public:
        FrameAdmission() : m_lim(), m_init(false) { reset(); }

        bool init(const AdmissionLimits &lim)
        {
            m_lim = lim;
            reset();
            m_init = true;
            return true;
        }

        /** 清空历史(保留上限)。换 profile / 上电初始化后调用 */
        void reset()
        {
            m_haveSeq    = false;
            m_synced     = false;
            m_lastSeq    = 0u;
            m_lastRxUs   = 0u;
            m_haveStick  = false;
            m_lastRoll   = 0;
            m_lastPitch  = 0;
            m_lastYaw    = 0;
            m_lastThr    = 0u;
            m_accepts    = 0u;
            for (uint8_t i = 0u; i < (uint8_t)AdmitVerdict::Count; ++i) {
                m_rejects[i] = 0u;
            }
        }

        /**
         * @brief 采信判据
         * @param raw    指向**帧头**的原始字节(tryDecode 消费的那一段), 用于核对长度与"恒 0 空位"
         * @param rawLen 原始字节数(应等于 frameBytes(f.type); 多给的部分不看)
         * @param f      tryDecode() 解出来的帧
         * @param nowUs  本地单调微秒时刻
         * @param st     当前系统状态快照; 传 0 = 不做"当前状态是否允许"这一问(纯协议解析时用)
        /**
         * @param raw    指向**帧头**的原始字节(tryDecode 消费的那一段), 用于核对长度与"恒 0 空位"
         * @param rawLen 原始字节数(应等于 frameBytes(f.type); 多给的部分不看)
         * @param f      tryDecode() 解出来的帧
         * @param nowUs  本地单调微秒时刻
         * @return Ok 表示可以采信; 其它值表示被哪一条判据拒绝(名字见 admitVerdictName)
         *
         * ⚠ 这里**不再**有"当前系统状态"这个入参(2026-10-11 方案甲): 采信是"这一帧像不像
         *   本工程编出来的", 而"命令现在能不能执行"是业务准入 —— 后者在 app 层, 且那一层
         *   本来就会回 DL_ACK。混在一起的时候, 状态不允许的命令会被当成"帧不可信"
         *   静默丢掉, 一个回执都不发。
         *   ⚠ 2026-10-13: app 层那份准入(`admitTask`)已随裁剪删除, 现在 app 层只剩通道级权限 ——
         *   这个文件**没有**因此改变(它本来就不认识业务准入), 但 DL_ACK 的语义变弱了。
         */
        AdmitVerdict admit(const uint8_t *raw, uint16_t rawLen, const Frame &f, uint32_t nowUs)
        {
            if (!m_init) {
                return reject(AdmitVerdict::NotInitialized);
            }
            const uint16_t need = frameBytes(f.type);
            if (raw == 0 || need == 0u || rawLen < need) {
                return reject(AdmitVerdict::RawLenMismatch);
            }
            // 帧头字节必须与解出来的类型/SEQ 一致(把"上一帧的缓冲"传进来时, 这条会立刻报)
            if (frameTypeOf(raw[0]) != f.type || seqOf(raw[0]) != f.seq) {
                return reject(AdmitVerdict::RawLenMismatch);
            }

            // ---- ① 类型: 在本角色允许的集合里吗 ----
            if ((m_lim.checks & kCheckType) != 0u) {
                if ((m_lim.allowedTypes & typeMaskOf(f.type)) == 0u) {
                    return reject(AdmitVerdict::TypeNotAllowed);
                }
            }

            // ---- ② 序号窗口 + 到达间隔 ----
            if ((m_lim.checks & kCheckSeq) != 0u && m_haveSeq) {
                const uint32_t since = timeSince(nowUs, m_lastRxUs);
                /**
                 * 时基不连续(微秒计数器回绕) ⇒ **重新对齐本判据的时基**, 而不是把这一帧判死 ——
                 * 更不能"只 reject 了事": `m_lastRxUs` 只在**采信**时才更新, 一旦 nowUs 掉到它后面,
                 * `since` 会一直是"半程以上的巨大值", 直到 nowUs 重新追上它(最长可达计数器半程)。
                 *
                 * ⚠ 上板实测(2026-10-06, 这是"链路正常约 20 秒后突然稳定超时"的真元凶):
                 *   微秒时基每 25.57 秒回绕一次 ⇒ 回绕那一刻起 **每一帧都被判 TooSoon**:
                 *   `收帧` +33/s(= 遥控器上行 33.3Hz, 说明空口完全正常) 而 `采信拒` +33/s,
                 *   `采纳`/`发帧` 永久冻结, 飞控 2 秒后判链路超时。
                 *
                 * 为什么"重新对齐"是安全的: 它只影响"到达间隔"这一条判据; 类型 / SEQ 窗口 /
                 *   物理量级 / 命令 四条判据照样在下面逐条执行 —— 假帧不会因为一次回绕就变得可采信。
                 * (时基本身也已经在 dlx_timebase 里展开成单调值; 这一条是第二道防线:
                 *  71.6 分钟的回绕、时基被重置、调试器改 CYCCNT —— 都不该让链路永久失聪。)
                 */
                if (since > (uint32_t)0x7FFFFFFFu) {
                    m_lastRxUs = nowUs; // 重新对齐, 这一帧继续走后面的判据
                } else if (m_lim.minFrameIntervalUs > 0u && since < m_lim.minFrameIntervalUs) {
                    return reject(AdmitVerdict::TooSoon);
                }
                if (m_synced) {
                    const uint8_t gap = seqGap(m_lastSeq, f.seq);
                    if (gap == 0u) {
                        return reject(AdmitVerdict::SeqDuplicate);
                    }
                    if (m_lim.maxSeqGapWhenSynced != 0u && gap > m_lim.maxSeqGapWhenSynced) {
                        // 跳得太远: 拒掉这一帧并转入"重新捕获"(下一帧不再受窗口约束)。
                        // 为什么要转: 真丢了一大串以后, 下一帧的 SEQ 必然也"跳得远" —— 如果每次都拒,
                        // 就永远回不来了。代价只是晚一帧恢复。
                        m_synced = false;
                        return reject(AdmitVerdict::SeqJumpTooBig);
                    }
                }
            }

            // ---- ③ 关键字段的物理量级 ----
            const AdmitVerdict rv = checkRanges(raw, f);
            if (rv != AdmitVerdict::Ok) {
                return reject(rv);
            }

            // ---- ④ 命令帧: op 在不在表里 / arg 在不在取值域 ----
            const AdmitVerdict cv = checkCommand(f);
            if (cv != AdmitVerdict::Ok) {
                return reject(cv);
            }

            // ---- 采信: 到这里才允许改历史 ----
            if (f.type == FrameType::UpCtrl) {
                m_haveStick = true;
                m_lastRoll  = f.u.ctrl.roll;
                m_lastPitch = f.u.ctrl.pitch;
                m_lastYaw   = f.u.ctrl.yaw;
                m_lastThr   = f.u.ctrl.thr;
            }
            // (2026-10-11 删掉了"记住最后一帧命令的 op/arg"那三项: 三个 getter 的调用者都是 0,
            //  而且它们的语义是"命令能不能执行", 那件事已经搬去 app 层了。) 
            m_lastSeq  = f.seq;
            m_lastRxUs = nowUs;
            m_haveSeq  = true;
            m_synced   = true;
            ++m_accepts;
            return AdmitVerdict::Ok;
        }

        bool     initialized() const { return m_init; }
        bool     inSync() const { return m_synced; }
        uint8_t  lastSeq() const { return m_lastSeq; }
        uint32_t lastRxUs() const { return m_lastRxUs; }
        uint32_t acceptCount() const { return m_accepts; }
        uint32_t rejectCount(AdmitVerdict v) const { return m_rejects[(uint8_t)v]; }
        uint32_t rejectTotal() const
        {
            uint32_t n = 0u;
            for (uint8_t i = 0u; i < (uint8_t)AdmitVerdict::Count; ++i) {
                n += m_rejects[i];
            }
            return n;
        }
        const AdmissionLimits &limits() const { return m_lim; }
    private:
        AdmitVerdict reject(AdmitVerdict v)
        {
            m_rejects[(uint8_t)v] = m_rejects[(uint8_t)v] + 1u;
            return v;
        }

        static bool enabled(float lim)
        {
            return lim > 0.0f;
        }
        static bool overAbs(float v, float lim)
        {
            const float a = (v < 0.0f) ? -v : v;
            return enabled(lim) && (a > lim);
        }

        /** ③ 关键字段的物理量级。集中在**一个**函数里而不是散在各处: 这些数字是"物理上不可能"的
         *  判据, 必须能一眼看完、能被单测逐条打反例。 */
        AdmitVerdict checkRanges(const uint8_t *raw, const Frame &f) const
        {
            if ((m_lim.checks & kCheckRange) == 0u) {
                return AdmitVerdict::Ok;
            }
            switch (f.type) {
            case FrameType::UpCtrl:
                // 摇杆/油门的位宽与语义域完全相同(11 位 ↔ ±1023 / 0..2047), 所以"范围检查"在这里
                // **必须看线上原始位**, 不能看解码后的值: unpackUpCtrl 已经把不可用的 -1024 码点夹回
                // -1023 了(§3.3 "解码时把它夹回"), 所以解码值永远合法, 而线上出现 -1024 恰恰是
                // "这帧不是本工程的编码器编出来的"的硬证据。丢掉一帧控制输入(10ms 用旧 setpoint)
                // 比接受一个"夹回来正好是满舵"的码点安全得多。
                if (bits::getSignedBits(raw + 1, upCtrl::kPayloadBytes, upCtrl::kRollBit, upCtrl::kRollBits) <
                        upCtrl::kStickMin ||
                    bits::getSignedBits(raw + 1, upCtrl::kPayloadBytes, upCtrl::kPitchBit, upCtrl::kPitchBits) <
                        upCtrl::kStickMin ||
                    bits::getSignedBits(raw + 1, upCtrl::kPayloadBytes, upCtrl::kYawBit, upCtrl::kYawBits) <
                        upCtrl::kStickMin) {
                    return AdmitVerdict::StickCodeUnused;
                }
                // 有牙的是"单帧变化量": 80Hz 下人手推杆的极限变化量就是那么几十~几百 counts,
                // 一帧从 -1023 跳到 +1023 只可能是噪声或伪造。
                if (enabled(m_lim.maxStickStepNorm) && m_haveStick) {
                    const float stepRoll  = (float)(f.u.ctrl.roll - m_lastRoll) / 1023.0f;
                    const float stepPitch = (float)(f.u.ctrl.pitch - m_lastPitch) / 1023.0f;
                    const float stepYaw   = (float)(f.u.ctrl.yaw - m_lastYaw) / 1023.0f;
                    const float stepThr   = (float)((int32_t)f.u.ctrl.thr - (int32_t)m_lastThr) / 2047.0f;
                    if (overAbs(stepRoll, m_lim.maxStickStepNorm) || overAbs(stepPitch, m_lim.maxStickStepNorm) ||
                        overAbs(stepYaw, m_lim.maxStickStepNorm) || overAbs(stepThr, m_lim.maxStickStepNorm)) {
                        return AdmitVerdict::StickStepTooBig;
                    }
                }
                return AdmitVerdict::Ok;

            case FrameType::UpVel: // int8 @5cm/s ⇒ 线上能表示 ±6.4 m/s, 物理上限由调用方给
                // 三轴(2026-10-05 起): vz 是"定速模式的 Z 轴绝对目标", 与 vx/vy 同一量纲与上限
                if (overAbs(f.u.vel.vx, m_lim.maxSpeedMps) || overAbs(f.u.vel.vy, m_lim.maxSpeedMps) ||
                    overAbs(f.u.vel.vz, m_lim.maxSpeedMps)) {
                    return AdmitVerdict::SpeedOutOfRange;
                }
                return AdmitVerdict::Ok;

            case FrameType::UpPos: // int16 @1dm ⇒ 线上能表示 ±3276.8 m
                if (overAbs(f.u.pos.x, m_lim.maxPositionM) || overAbs(f.u.pos.y, m_lim.maxPositionM)) {
                    return AdmitVerdict::PositionOutOfRange;
                }
                return AdmitVerdict::Ok;

            case FrameType::UpSetpoint: // int16 @1cm ⇒ 线上能表示 ±327.67 m, 上限由调用方给
                if (overAbs(f.u.setpoint.altM, m_lim.maxPositionM)) {
                    return AdmitVerdict::PositionOutOfRange;
                }
                return AdmitVerdict::Ok;

            case FrameType::UpMotor:
                // ① 2 个保留位必须为 0: 与 DL_STATE 的 7 个空位同一约定 —— 出现非 0 说明这一帧
                //    不是本工程的编码器编出来的(编码侧每次都整帧清零)
                if (!upMotorSpareBitsZero(raw)) {
                    return AdmitVerdict::SpareBitsNotZero;
                }
                // ② DShot 计数 2000..2047 是特殊命令区(beacon/遥测), 当油门发出去会"电机唱歌"
                // 2026-10-12: 一帧四路 ⇒ **四路都要查**(漏一路就等于放行一路"电机唱歌")
                if (enabled((float)m_lim.maxMotorCount)) {
                    for (uint8_t i = 0u; i < upMotor::kMotorCount; ++i) {
                        if (f.u.motor.throttle[i] > m_lim.maxMotorCount) {
                            return AdmitVerdict::MotorOutOfRange;
                        }
                    }
                }
                return AdmitVerdict::Ok;

            case FrameType::UpCmd:
                return AdmitVerdict::Ok; // 命令帧的量级检查在 ④ (op/arg/状态) 里

            case FrameType::DlAck:
                if (!isKnownWireResult(f.u.ack.result)) {
                    return AdmitVerdict::UnknownResult;
                }
                return AdmitVerdict::Ok;

            case FrameType::DlAtt: {
                // 单位四元数: q1²+q2²+q3² ≤ 1(虚部三轴的模长不超过 1)。这条判据很硬: 12 位定点
                // 满量程下三个分量都接近 ±1 时, 平方和会明显超过 1, 而合法姿态永远做不到。
                // 依据 §3.3: q 各 12b, round(x*2047)。
                if (enabled(m_lim.quatNormMax)) {
                    const float n2 = f.u.att.q1 * f.u.att.q1 + f.u.att.q2 * f.u.att.q2 + f.u.att.q3 * f.u.att.q3;
                    if (n2 > m_lim.quatNormMax * m_lim.quatNormMax) {
                        return AdmitVerdict::QuatNormOutOfRange;
                    }
                }
                if (overAbs(f.u.att.vx, m_lim.maxSpeedMps) || overAbs(f.u.att.vy, m_lim.maxSpeedMps) ||
                    overAbs(f.u.att.vz, m_lim.maxSpeedMps)) {
                    return AdmitVerdict::SpeedOutOfRange;
                }
                return AdmitVerdict::Ok;
            }

            case FrameType::DlSys:
                if (overAbs(f.u.sys.x, m_lim.maxPositionM) || overAbs(f.u.sys.y, m_lim.maxPositionM) ||
                    overAbs(f.u.sys.z, m_lim.maxPositionM)) {
                    return AdmitVerdict::PositionOutOfRange;
                }
                if (enabled(m_lim.maxVoltageV) &&
                    (f.u.sys.voltageV < m_lim.minVoltageV || f.u.sys.voltageV > m_lim.maxVoltageV)) {
                    return AdmitVerdict::VoltageOutOfRange;
                }
                if (enabled(m_lim.maxCurrentA) && (f.u.sys.currentA > m_lim.maxCurrentA)) {
                    return AdmitVerdict::CurrentOutOfRange;
                }
                if (m_lim.maxMotorCount != 0u) {
                    for (uint8_t i = 0u; i < dlSys::kMotorCount; ++i) {
                        if (f.u.sys.motor[i] > m_lim.maxMotorCount) {
                            return AdmitVerdict::MotorOutOfRange;
                        }
                    }
                }
                return AdmitVerdict::Ok;

            case FrameType::DlGps:
                // 纬度 ±90°、经度 ±180°、航向 < 360°、地速 ≥ 0: 这几条是世界坐标/物理的硬边界,
                // 与机型无关, 所以它们**不设"0 = 不检查"**, 永远检查。随机噪声几乎不可能落进这些域。
                if (f.u.gps.latE7 < -900000000 || f.u.gps.latE7 > 900000000) {
                    return AdmitVerdict::GpsLatOutOfRange;
                }
                if (f.u.gps.lonE7 < -1800000000 || f.u.gps.lonE7 > 1800000000) {
                    return AdmitVerdict::GpsLonOutOfRange;
                }
                if (f.u.gps.courseCdeg > dlGps::kCourseMaxCdeg) {
                    return AdmitVerdict::GpsCourseOutOfRange;
                }
                if (f.u.gps.gspeedCms < 0) {
                    return AdmitVerdict::GpsSpeedOutOfRange;
                }
                if (enabled(m_lim.maxHdop) && (f.u.gps.hdop > m_lim.maxHdop)) {
                    return AdmitVerdict::HdopOutOfRange;
                }
                return AdmitVerdict::Ok;

            case FrameType::DlState:
                // 3 位/4 位的位域能装下比语义域更多的值 —— 于是"超域"本身就是一个可用的假帧判据。
                if (f.u.state.mode > kWireModeMax) {
                    return AdmitVerdict::ModeOutOfRange;
                }
                if (f.u.state.failsafeLvl > kWireFailsafeLvlMax) {
                    return AdmitVerdict::FailsafeLvlOutOfRange;
                }
                if (f.u.state.failsafeRsn > kWireFailsafeRsnMax) {
                    return AdmitVerdict::FailsafeRsnOutOfRange;
                }
                // 丢包率同样要看**线上原始位**: unpackDlState 会把 >100 的值夹到 100(为了显示不出格),
                // 于是解码值永远 ≤100, 而线上真的出现 >100 就说明这不是本工程的编码器发的。
                {
                    const uint32_t ulRaw =
                        bits::getBits(raw + 1, dlState::kPayloadBytes, dlState::kUlLossBit, dlState::kUlLossBits);
                    const uint32_t dlRaw =
                        bits::getBits(raw + 1, dlState::kPayloadBytes, dlState::kDlLossBit, dlState::kDlLossBits);
                    if (ulRaw > kLossMaxPercent || dlRaw > kLossMaxPercent) {
                        return AdmitVerdict::LossOutOfRange;
                    }
                }
                // 7 个保留位(§3.3 更正 1)编码时恒写 0 ⇒ 不为 0 就一定不是本工程编出来的帧
                if (bits::getBits(raw + 1, dlState::kPayloadBytes, dlState::kSpareBit, dlState::kSpareBits) != 0u) {
                    return AdmitVerdict::SpareBitsNotZero;
                }
                return AdmitVerdict::Ok;

            default:
                // 理论上到不了(帧长表已经挡掉了未知类型); 真到了说明新增了帧类型而这里没跟上,
                // 保守选择"拒"而不是"放"。
                return AdmitVerdict::TypeNotAllowed;
            }
        }

        /**
         * ④ 命令帧: op 在不在协议表里 / arg 在不在取值域。
         *
         * ⚠ 这里**只判协议事实**, 不判"这条命令现在能不能执行"(2026-10-11 方案甲)。
         *   原来的 `admitCommand(m.op, true, *st)` 把业务准入混进了"这一帧值不值得采信", 后果是
         *   带锁按 SET_MODE、未解锁按 TAKEOFF 这类命令被当成**假帧**丢掉, 一个 DL_ACK 都不回
         *   (调用方 `++linkAdmitRejects; return;`), 而注释说的"按拒绝码回 DC_ACK"从未实现 ——
         *   那三个 getter 的调用者是 0。准入现在只剩 app 层一处, 而那一处本来就会回 ACK。
         */
        AdmitVerdict checkCommand(const Frame &f)
        {
            if ((m_lim.checks & kCheckCmdArg) == 0u || f.type != FrameType::UpCmd) {
                return AdmitVerdict::Ok;
            }
            if (!isKnownWireOp(f.u.cmd.op)) {
                return AdmitVerdict::UnknownOp;
            }
            // arg 的取值域只对"有确定枚举域"的命令检查; 其余的不检查 —— 编一个上限比不检查更危险
            // (它会把合法命令判成假帧)。依据: §3.6 模式请求 0..3 / 维护种类 0..3 / TAKEOFF 目标高度 [dm]。
            switch (f.u.cmd.op) {
            case kWireOpSetMode:
                if (f.u.cmd.arg > kWireModeMax) {
                    return AdmitVerdict::CmdArgOutOfRange;
                }
                break;
            case kWireOpEnterMaintenance:
                if (f.u.cmd.arg > kWireServiceMax) {
                    return AdmitVerdict::CmdArgOutOfRange;
                }
                break;
            case kWireOpTakeoff:
                if (m_lim.maxTakeoffDm != 0u && f.u.cmd.arg > m_lim.maxTakeoffDm) {
                    return AdmitVerdict::CmdArgOutOfRange;
                }
                break;
            default:
                break;
            }
            return AdmitVerdict::Ok;
        }

        AdmissionLimits m_lim;
        bool            m_init;
        bool            m_haveSeq;
        bool            m_synced;
        uint8_t         m_lastSeq;
        uint32_t        m_lastRxUs;
        bool            m_haveStick;
        int16_t         m_lastRoll;
        int16_t         m_lastPitch;
        int16_t         m_lastYaw;
        uint16_t        m_lastThr;
        uint32_t        m_accepts;
        uint32_t        m_rejects[(uint8_t)AdmitVerdict::Count];
    };

    //===============================================================================================
    // 附:矛盾点 A —— 为什么 masterFrameOpensWindow() 有两个 SeqMode
    //
    //   §3.5 写: "时间轴(槽号 = 上行 SEQ 模 5)" 与规则② "收到 seq=k 后, 下一个槽号是 (k+1) mod 5,
    //   等于 4 就是自己的窗口"。按字面, 主端要让 SEQ 每**槽**走一格(在槽 4 也让计数器走一格但不发),
    //   这样 seq%5 才能一直等于槽号。但 §3.2 同时定死 SEQ 是 4 位、模 16: 16 与 5 互质, 于是
    //   "槽号 = seq mod 5" 在第 16 个槽之后必然错位:
    //       超帧 0: 槽 0..4 → seq 0,1,2,3,(4 不发)   ⇒ seq=3 时 (3+1)%5=4 = 我的窗口 ✓
    //       超帧 3: 槽 15..19 → seq 15,0,1,2,(3 不发) ⇒ seq=2 时 (2+1)%5=3 ≠ 4 ✗ (规则②给出的
    //               "下一个槽号"与真实槽号已经差了一个, 照它发车就会撞上主端的上行帧)
    //   单测里有一条断言把这个漂移**钉成数字**(check.cpp 第 3 组): 字面规则只在头 16 个槽里成立。
    //   所以本实现:
    //     · 默认 PerUplinkFrame(= 主端每发一帧 +1, 与 §3.2 "方向内自增计数器" 一致), 回绕自洽;
    //     · 保留 PerSlotCounter 供"照 §3.5 字面实现主端"时对拍;
    //     · 两种模式都只影响"哪个主端帧打开我的窗口", 不影响失同步保护/guard/时基回绕这些安全逻辑。
    //   这不是"擅自改方案", 而是把方案里"算不通的那半句"显式化成参数: 主端到底怎么数 SEQ, 只有
    //   主端实现者能拍板; 从端猜错的表现是"在自己的窗口里不发"或"在错的槽里发", 后者会撞主端。
    //===============================================================================================

} // namespace link
} // namespace dlx
