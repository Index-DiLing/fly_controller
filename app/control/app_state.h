#pragma once
#include <stdint.h>
namespace app
{
enum class SystemState : uint8_t
{
    Boot = 0,   ///< 复位到时钟就绪
    InitLow,    ///< 一级初始化(时钟/GPIO/串口/看门狗)
    InitHigh,   ///< 二级初始化(存储→IMU→链路→传感器), 产出初始化报告
    Standby,    ///< 地面待命 = **未解锁**; 停机动作也回到这里
    Maintenance,///< 维护模式: 允许 ARM + Manual 试车
    Arming,     ///< 输出初始化序列进行中，控制尚未接管
    Armed,      ///< 控制已接管
};

enum : uint8_t
{
    effectStartPower = 1u,
    effectTakeover = 2u,
    effectStopPower = 4u,
    effectPurgeDirect = 8u,
};

constexpr bool canOutput(SystemState s)
{
    return s == SystemState::Arming || s == SystemState::Armed;
}

constexpr bool isTakeover(SystemState s)
{
    return s == SystemState::Armed;
}

constexpr bool isGround(SystemState s)
{
    return s == SystemState::Standby || s == SystemState::Maintenance;
}

// 请求词汇: Task
enum class Task : uint8_t
{
    RequestEStop = 0, ///< 立即停机请求
    RequestDisarm,    ///< 停止输出并回到地面态
    RequestArm,       ///< 启动输出并进入解锁序列
    RequestCancel,    ///< 取消当前操作
    RequestLand,      ///< 自动降落
    RequestTakeoff,   ///< 自动起飞
    RequestModeStabilize, ///< 模式选择
    RequestModeAltHold,
    RequestModeVelHold,
    RequestModeManual,
    Count,
    None = Count,
};

constexpr bool isRequest(Task t)
{
    return static_cast<uint8_t>(t) < static_cast<uint8_t>(Task::Count);
}
enum class Mode : uint8_t
{
    Manual = 0, ///< 直通(油门/姿态直接给): 只在未解锁时允许
    Stabilize,  ///< 自稳: 需要 IMU
    AltHold,    ///< 定高: 需要 IMU + 高度源
    VelHold,    ///< 定速(悬停不移动): 需要 IMU + 高度源 + 光流
};

constexpr bool isModeTask(Task t)
{
    return t == Task::RequestModeManual || t == Task::RequestModeStabilize ||
           t == Task::RequestModeAltHold || t == Task::RequestModeVelHold;
}

constexpr Mode modeOf(Task t)
{
    return t == Task::RequestModeManual   ? Mode::Manual
           : t == Task::RequestModeStabilize ? Mode::Stabilize
           : t == Task::RequestModeAltHold   ? Mode::AltHold
                                              : Mode::VelHold;
}

constexpr Task modeTask(Mode m)
{
    return m == Mode::Manual    ? Task::RequestModeManual
           : m == Mode::Stabilize ? Task::RequestModeStabilize
           : m == Mode::AltHold   ? Task::RequestModeAltHold
                                  : Task::RequestModeVelHold;
}

enum class Operation : uint8_t
{
    None = 0,   ///< 没有操作(高度目标由模式那一支决定)
    Takeoff,    ///< 自动起飞
    Land,       ///< 自动降落(失效保护缓降也走这一支的下降逻辑, 只是速率不同)
};

/**
 * 会不会自己结束的那一类请求(原地类: 不移动 `State`, 只换 `Operation`)。
 */
constexpr bool isOperationTask(Task t)
{
    return t == Task::RequestTakeoff || t == Task::RequestLand || t == Task::RequestCancel;
}

// 发布的快照
/**
 * 控制环发布、别的线程(链路/日志)读的那一份。
 *
 * 为什么是一个整体快照而不是让读的人逐个取: 逐个取会拼出"新状态 + 旧模式"这种撕裂组合
 * (链路层拿它回 ACK 与填遥测, 撕了就会自相矛盾)。取一份 = 一个临界区。
 */
struct Snapshot
{
    SystemState         state{SystemState::Boot};
    Mode           mode{Mode::Stabilize};
    Operation      operation{Operation::None};
    /**
     * 健康位域: **bit N = `dlx::SensorHealth` 的第 N 项**(打包在 app_health.cpp 的 `healthBits()`)。
     */
    uint8_t        health{0};
    bool hardStop{false};
    bool bootReady{false};
    uint32_t publishedMs{0u};
    uint32_t sequence{0u};
};

} // namespace app
