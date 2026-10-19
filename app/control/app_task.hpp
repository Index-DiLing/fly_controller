#pragma once
#include <math.h>
#include <stdint.h>

#include "app/control/app_state.h"
#include "flight_config.hpp"

// 自动起降与失效保护缓降的纯逻辑；不访问硬件或线程通道。

namespace app
{

/** 高度读数的可信下界 [m]。低于它只可能是估计器坏了(飞机不可能在地下 0.5m)。 */
constexpr float heightSanityMinM = -0.5f;

/** 起飞到位判据(方案 §2.8): 1.2 ± 0.15 m 且垂速 ≈ 0 */
constexpr float takeoffReachM = 0.15f;
constexpr float takeoffVzMps  = 0.15f;

/**
 * 操作子阶段(与方案 §2.8 的表一致, 随 `Snapshot::stage` 上报给遥控器/日志)。
 *
 * ⚠ **同一个数值在不同操作里有不同含义** —— 这是刻意的(省带宽), 但也是唯一的坑:
 *   Takeoff: 1 = 爬升;  Land: 1 = 已触地。
 *   所以名字里必须带操作前缀, 而且**任何跨操作的阶段传递都要先归一化**(见 stepOperation)。
 */
constexpr uint8_t takeoffStageSpinUp    = 0u; ///< 起转(等启动序列走完)
constexpr uint8_t takeoffStageClimb     = 1u; ///< 爬升
constexpr uint8_t takeoffStageDelivered = 2u; ///< 已交付(随后 operation = None)
constexpr uint8_t landStageDescend      = 0u; ///< 下降
constexpr uint8_t landStageTouchdown    = 1u; ///< 已触地, 等**人**上锁

/** 本圈高度环的用法 */
enum class AltitudeUse : uint8_t
{
    None = 0, ///< 不接管(自稳油门 / 无操作)
    Hold = 1, ///< 高度环接管, 目标 = altTargetM
    Idle = 2, ///< 已触地: 姿态环 + 最低油门, 等**人**上锁
};

struct OperationIn
{
    Operation operation{Operation::None};
    uint8_t   stage{0};
    float     altTargetM{0.0f}; ///< 上一圈的高度目标(斜坡从这里开始)
};

struct OperationCtx
{
    float dt{0.0f};          ///< 控制周期 [s](cfg::kLoopDt)
    float heightM{0.0f};     ///< 当前高度估计 [m]
    float vzMps{0.0f};       ///< 当前垂速 [m/s]
    bool  canOutput{false};  ///< Armed 或 Arming(动力可以出力)
    /**
     * 失效保护要求**原地缓降**(优先于 `operation`)。
     *
     * ⚠ 它不是"操作"而是"为什么": 它只改两件事 —— 下降速率(`kHoverLandDescentMps`)与
     *   "不接 CANCEL"(那条由准入判)。所以它与 `operation` **正交**, 不能折进 Operation
     *   (见 app_state.h 的 Operation 注释: 折进去会让已触地的 Land 从软地板重新下降)。
     */
    bool  hoverLand{false};
};

struct OperationOut
{
    Operation   operation{Operation::None};
    uint8_t     stage{0};
    float       altTargetM{0.0f};
    AltitudeUse altitude{AltitudeUse::None};
    bool        heightUsable{false}; ///< 高度读数可信(软地板/触地/到位判据才可用)
};

/** 高度读数可信吗(见文件头规则 3) */
inline bool heightUsable(float heightM)
{
    return isfinite(heightM) && heightM >= heightSanityMinM && heightM <= cfg::control::kAltMaxM;
}

/**
 * 操作/失效保护的状态机一步。**纯函数**: 不读硬件、不写共享区, 只吃 (in, ctx) 吐 out。
 */
inline OperationOut stepOperation(const OperationIn &in, const OperationCtx &ctx)
{
    OperationOut out;
    out.operation    = in.operation;
    out.stage        = in.stage;
    out.altTargetM   = in.altTargetM;
    out.altitude     = AltitudeUse::None;
    out.heightUsable = heightUsable(ctx.heightM);

    // ---- 规则 2: 动力不能出力(未解锁 / 正在上锁) ⇒ 操作立即作废 ----
    // 注意它**优先于**失效保护: 人上锁之后不该再有任何"自动下降"在执行。
    if (!ctx.canOutput) {
        out.operation = Operation::None;
        out.stage     = 0u;
        return out;
    }

    const bool landing = ctx.hoverLand || (in.operation == Operation::Land);

    // ================= 降落(自动降落 与 失效保护缓降 共用) =================
    if (landing) {
        /**
         * ⚠ 阶段编号必须在**操作语义**里解释, 不能跨语义复用:
         *   Takeoff: stage 1 = 爬升;  Land: stage 1 = 已触地。
         * 失效保护在"起飞爬升途中"接管时, 若直接沿用 stage = 1, 就会**跳进"已触地"分支** ——
         * 空中把高度环撤掉、油门收到怠速。所以这里先把它归一化回"下降"。
         *
         * 为什么条件是 `in.operation != Land`: 已经在 Land 时 stage 就是 Land 的语义
         * (1 = 已触地), 归一化会把**已经落地**的飞机重新推进下降。
         */
        uint8_t stage = out.stage;
        if (ctx.hoverLand && in.operation != Operation::Land) {
            stage = landStageDescend;
        }
        out.operation = Operation::Land; // 失效保护期间如实显示"在降落"(DL_STATE/日志都看这一项)
        out.stage     = stage;

        const float rate =
            ctx.hoverLand ? cfg::link::failsafe::kHoverLandDescentMps : cfg::control::kLandDescentMps;

        if (stage == landStageDescend) {
            out.altitude   = AltitudeUse::Hold;
            out.altTargetM = in.altTargetM - rate * ctx.dt;
            /**
             * 触地判据只在读数**可信**时成立。
             * 读数坏了 → heightUsable = false → 这一支不进, 目标继续往下走(规则 3):
             * 飞机以设定速率继续缓降, 直到人上锁; 绝不会因为"读不到高度"而停在半空。
             */
            if (out.heightUsable && ctx.heightM < cfg::control::kLandFloorM) {
                out.stage      = landStageTouchdown;
                out.altitude   = AltitudeUse::Idle; // 停止下降: 姿态环 + 怠速
                out.altTargetM = cfg::control::kAltMinM;
                // 操作不结束 —— 由**人**上锁时才结束(见 canOutput 分支)。这就是规则 1。
            }
            return out;
        }

        // 已触地(landStageTouchdown): 保持姿态 + 怠速, 等 disarmRequest。
        out.altitude   = AltitudeUse::Idle;
        out.altTargetM = cfg::control::kAltMinM;
        return out;
    }

    // ================= 起飞 =================
    if (in.operation == Operation::Takeoff) {
        out.altitude = AltitudeUse::Hold;

        if (out.stage == takeoffStageSpinUp) {
            /**
             * 起转阶段: 启动序列(motorArmDelayMs)必须已经走完 —— 也就是 `SystemStae::Armed`,
             * 而 canOutput 在 Arming 时也成立。所以这里等的是 `Armed` 那一档, 由调用方通过
             * canOutput + 状态保证; 能走到这里说明已经可以在空中出力了。
             * 目标**从当前高度接着走**(而不是写死 0): 否则地面高度估到 3cm 时"目标 0"会先
             * 压一小段, 那一小段是纯噪音。同时把目标夹到不高于起飞高度 —— 见下面 stage 1。
             */
            if (in.altTargetM > cfg::control::kTakeoffHeightM) {
                out.altTargetM = cfg::control::kTakeoffHeightM;
            }
            out.stage = takeoffStageClimb;
            return out;
        }

        // 爬升: 高度目标按 kTakeoffClimbMps 斜坡上升, 到 kTakeoffHeightM 停(速率不过 VZ_MAX_UP,
        // 所以位置环不会饱和 —— 见方案 §2.8 的斜率选择理由)。
        out.altTargetM = in.altTargetM + cfg::control::kTakeoffClimbMps * ctx.dt;
        if (out.altTargetM > cfg::control::kTakeoffHeightM) {
            out.altTargetM = cfg::control::kTakeoffHeightM;
        }
        out.stage = takeoffStageClimb;

        // 交付判据: 到 1.2 ± 0.15 m **且**垂速≈0(两个都要, 只看高度会把"冲过头"也判成到位)
        if (out.heightUsable && fabsf(ctx.heightM - cfg::control::kTakeoffHeightM) <= takeoffReachM &&
            fabsf(ctx.vzMps) <= takeoffVzMps) {
            out.operation  = Operation::None; // 交回当前模式(定高保持 1.2m; 定速保持高度)
            out.stage      = takeoffStageDelivered;
            out.altitude   = AltitudeUse::None;
            out.altTargetM = cfg::control::kTakeoffHeightM;
        }
        return out;
    }

    // 无操作时透传高度目标，并将操作阶段清零。
    out.operation = Operation::None;
    out.stage     = 0u;
    return out;
}

} // namespace app
