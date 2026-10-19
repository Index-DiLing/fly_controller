#pragma once
#include <stdint.h>

#include "app/control/app_state.h"
// 链路协议适配：命令映射、通道权限及 Task 转换；状态与前置条件由控制线程审核。

namespace app
{

enum class GroundStatus : uint16_t
{
    Ok             = 0,
    BadCommand     = 1,
    BadArgument    = 2,
    FlashFailed    = 3,
    NotFound       = 4,
    Busy           = 5,
    RejectedState  = 6,
    RejectedArming = 7,
    RejectedHealth = 8,
    RejectedSafety = 9,
    Timeout        = 10,
    AuthFail       = 11,
};

/** 链路线协议命令。该类型只在协议适配层存在，控制线程只接收 Task。 */
enum class CommandOp : uint8_t
{
    Ping = 0x00,
    Arm = 0x01,
    Disarm = 0x02,
    EmergencyStop = 0x03,
    SetMode = 0x10,
    Takeoff = 0x20,
    Land = 0x21,
    CancelTask = 0x22,
    CalStart = 0x30,
    CalAbort = 0x31,
    ParamGet = 0x40,
    ParamSet = 0x41,
    ParamList = 0x42,
    LinkSetProfile = 0x50,
    ProtoVersionGet = 0x51,
    EnterMaintenance = 0x60,
    ExitMaintenance = 0x61,
    LogList = 0x70,
    LogDump = 0x71,
    LogErase = 0x72,
    LogFormat = 0x73,
};

/** 显式映射线上命令字节；未知操作码返回失败。 */
inline bool mapUpCmdOp(uint8_t wireOp, CommandOp &out)
{
    switch (wireOp) {
    case 0x00u: out = CommandOp::Ping;             return true;
    case 0x01u: out = CommandOp::Arm;              return true;
    case 0x02u: out = CommandOp::Disarm;           return true;
    case 0x03u: out = CommandOp::EmergencyStop;    return true;
    case 0x10u: out = CommandOp::SetMode;          return true;
    case 0x20u: out = CommandOp::Takeoff;          return true;
    case 0x21u: out = CommandOp::Land;             return true;
    case 0x22u: out = CommandOp::CancelTask;       return true;
    case 0x30u: out = CommandOp::CalStart;         return true;
    case 0x31u: out = CommandOp::CalAbort;         return true;
    case 0x40u: out = CommandOp::ParamGet;         return true;
    case 0x41u: out = CommandOp::ParamSet;         return true;
    case 0x42u: out = CommandOp::ParamList;        return true;
    case 0x50u: out = CommandOp::LinkSetProfile;   return true;
    case 0x51u: out = CommandOp::ProtoVersionGet;  return true;
    case 0x60u: out = CommandOp::EnterMaintenance; return true;
    case 0x61u: out = CommandOp::ExitMaintenance;  return true;
    case 0x70u: out = CommandOp::LogList;          return true;
    case 0x71u: out = CommandOp::LogDump;          return true;
    case 0x72u: out = CommandOp::LogErase;         return true;
    case 0x73u: out = CommandOp::LogFormat;        return true;
    default:   return false;
    }
}

/** 仅允许飞行相关命令和 PING 通过无线；维护类命令返回 AuthFail。 */
constexpr bool commandAllowsWireless(CommandOp op)
{
    return op == CommandOp::Ping || op == CommandOp::Arm || op == CommandOp::Disarm ||
           op == CommandOp::EmergencyStop || op == CommandOp::SetMode || op == CommandOp::Takeoff ||
           op == CommandOp::Land || op == CommandOp::CancelTask;
}

/** 只判断命令通道权限；状态、健康及安全前置条件由控制线程审核。 */
inline GroundStatus admitCommandChannel(CommandOp op, bool wireless)
{
    if (wireless && !commandAllowsWireless(op)) {
        return GroundStatus::AuthFail;
    }
    return GroundStatus::Ok;
}

// 链路 v2 约定: app::Request/app::RequestResult 里的 wireSequence 字段承载 **cmd_id**(不再是 v1 的帧头 SEQ)。
// 这样控制线程无需知道链路协议版本, 只把 u8 原样带回; 链路层用它配对 DL_ACK(§3.4)。

/** 将命令与参数转为 Task；模式参数展开为各个模式请求，非状态请求返回 false。 */
inline bool taskOfCommand(CommandOp op, uint8_t arg, Task &out)
{
    switch (op) {
    case CommandOp::Arm:
        out = Task::RequestArm;
        return true;
    case CommandOp::Disarm:
        out = Task::RequestDisarm;
        return true;
    case CommandOp::EmergencyStop:
        out = Task::RequestEStop;
        return true;
    case CommandOp::Takeoff:
        out = Task::RequestTakeoff;
        return true;
    case CommandOp::Land:
        out = Task::RequestLand;
        return true;
    case CommandOp::CancelTask:
        out = Task::RequestCancel;
        return true;
    case CommandOp::SetMode:
        switch (arg) {
        case 0u: out = Task::RequestModeManual;    return true;
        case 1u: out = Task::RequestModeStabilize; return true;
        case 2u: out = Task::RequestModeAltHold;   return true;
        case 3u: out = Task::RequestModeVelHold;   return true;
        default: return false;
        }
    default:
        return false;
    }
}

/** 内部占位结果: 命令已投递、结果未回来。**不进线上 payload**(只在去重表里占位)。 */
constexpr uint8_t kCmdResultPending = 0xFEu;

/**
 * @brief 命令去重表(链路 v2 · 协议规格 §3.3/§8.1)
 *
 * 遥控器重传同一 cmd_id 直到收到 ACK; 飞控按 (session, cmd_id) 去重:
 *   命中且已有结果 → **只重发 ACK, 不重复执行**;
 *   命中但仍是 pending → 本轮不回(等结果), 也不重复执行;
 *   未命中 → 执行并记录。
 * 容量 8 条(规格定的"最近 8 条"), 满则覆盖最老的一条 —— 命令是手按出来的, 8 条足够覆盖
 * 一次重传窗口; 覆盖最老而不是拒绝新命令, 保证"最新的命令一定能被执行"。
 * session 变化由调用方 `clear()`(session 变化 ⇒ 上一会话的重传历史全部作废)。
 */
class CommandHistory
{
public:
    static constexpr uint8_t kCapacity = 8u;

    void clear()
    {
        for (uint8_t i = 0u; i < kCapacity; ++i) {
            m_entry[i].used   = false;
            m_entry[i].cmdId  = 0u;
            m_entry[i].result = 0u;
        }
        m_next = 0u;
    }

    CommandHistory() { clear(); }

    /** 命中返回 true 并给出结果(可能是 kCmdResultPending) */
    bool find(uint8_t cmdId, uint8_t &result) const
    {
        for (uint8_t i = 0u; i < kCapacity; ++i) {
            if (m_entry[i].used && m_entry[i].cmdId == cmdId) {
                result = m_entry[i].result;
                return true;
            }
        }
        return false;
    }

    /** 记录一条(已执行/已投递)的命令结果 */
    void record(uint8_t cmdId, uint8_t result)
    {
        if (cmdId == 0u) {
            return; // cmd_id=0 是无效值, 不进历史
        }
        for (uint8_t i = 0u; i < kCapacity; ++i) {
            if (m_entry[i].used && m_entry[i].cmdId == cmdId) {
                m_entry[i].result = result;
                return;
            }
        }
        m_entry[m_next].used   = true;
        m_entry[m_next].cmdId  = cmdId;
        m_entry[m_next].result = result;
        m_next                 = (uint8_t)((m_next + 1u) % kCapacity);
    }

    /** 结果回来后更新已记录项(找不到就补一条) */
    void update(uint8_t cmdId, uint8_t result)
    {
        record(cmdId, result);
    }

private:
    struct Entry
    {
        uint8_t cmdId;
        uint8_t result;
        bool    used;
    };
    Entry   m_entry[kCapacity];
    uint8_t m_next;
};

} // namespace app
