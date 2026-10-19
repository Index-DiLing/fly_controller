//===================================================================================================
// test/state_step_test.cpp —— Task / SystemState 主机侧测试
//
// 怎么跑(不需要板子、不需要交叉工具链), 一条命令:
//   g++ -std=c++11 -Wall -Wextra -I test/stub -I DL_LIB -I .
//       test/state_step_test.cpp app/sensors/app_health.cpp -o state_step_test
//   ./state_step_test
//
// 为什么 test/stub 必须在 -I 的最前面: dlx_interrupt_lock.hpp 只用 CMSIS 的三个原语,
// 那个目录里的 stm32f4xx.h 是单线程替身(记下 PRIMASK / 什么都不做), 语义与真实现一致。
//
// 覆盖统一 Task 词汇、空口状态映射、快照、操作状态机、链路命令与健康状态。
//===================================================================================================
#include <stdio.h>
#include <string.h>

#include "app/control/app_state.h"
#include "app/channels/app_control_channel.hpp"
#include "app/link/app_link_state.hpp"
#include "dlx_locked_cell.hpp"
#include "app/sensors/app_health.hpp"
#include "app/control/app_task.hpp"     // 顺带验证这个头本身能编(它连着 flight_config.hpp)
#include "app/link/app_link_cmd.hpp" // 命令词汇/映射/通道准入(方案甲搬到 app 的那一层)

using namespace app;
// 健康项的访问器在 dlx 里(历史原因: app_health 一直挂在 dlx 命名空间下)
using dlx::SensorHealth;
using dlx::Health;
using dlx::HealthSnapshot;

static int g_checks   = 0;
static int g_failures = 0;

#define CHECK(condition)                                                 \
    do {                                                                 \
        ++g_checks;                                                      \
        if (!(condition)) {                                              \
            ++g_failures;                                                \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #condition); \
        }                                                                \
    } while (0)

//===================================================================================================
// 一、统一请求词汇
//===================================================================================================
static void testTaskVocabulary()
{
    printf("-- 统一请求词汇\n");
    for (uint8_t i = 0u; i < static_cast<uint8_t>(Task::Count); ++i) {
        CHECK(isRequest(static_cast<Task>(i)));
    }
    CHECK(!isRequest(Task::None));
    CHECK(isGround(SystemState::Standby));
    CHECK(isGround(SystemState::Maintenance));
    CHECK(!isGround(SystemState::Arming));
    CHECK(canOutput(SystemState::Arming));
    CHECK(canOutput(SystemState::Armed));
    CHECK(!canOutput(SystemState::Standby));
    CHECK(isTakeover(SystemState::Armed));
    CHECK(!isTakeover(SystemState::Arming));
    CHECK(modeTask(Mode::Manual) == Task::RequestModeManual);
    CHECK(modeTask(Mode::Stabilize) == Task::RequestModeStabilize);
    CHECK(modeTask(Mode::AltHold) == Task::RequestModeAltHold);
    CHECK(modeTask(Mode::VelHold) == Task::RequestModeVelHold);
    CHECK(modeOf(Task::RequestModeManual) == Mode::Manual);
    CHECK(modeOf(Task::RequestModeStabilize) == Mode::Stabilize);
    CHECK(modeOf(Task::RequestModeAltHold) == Mode::AltHold);
    CHECK(modeOf(Task::RequestModeVelHold) == Mode::VelHold);
    CHECK(isOperationTask(Task::RequestTakeoff));
    CHECK(isOperationTask(Task::RequestLand));
    CHECK(isOperationTask(Task::RequestCancel));
    CHECK(!isOperationTask(Task::RequestArm));
}

//===================================================================================================
// 三、空口映射: 逐值对照旧位域(遥控器那边要照这张表同步)
//===================================================================================================
static void testWire()
{
    printf("-- 空口映射\n");
    struct WireCase
    {
        SystemState state;
        uint8_t lifecycle;
        uint8_t arming;
    };
    static const WireCase kWire[] = {
        {SystemState::Boot, 0u, 0u},        {SystemState::InitLow, 1u, 0u},   {SystemState::InitHigh, 2u, 0u},
        {SystemState::Standby, 3u, 0u},     {SystemState::Maintenance, 5u, 0u},
        {SystemState::Arming, 4u, 1u},      {SystemState::Armed, 4u, 2u},
    };
    for (uint32_t i = 0; i < 7u; ++i) {
        Snapshot s;
        s.state             = kWire[i].state;
        const WireState w   = deriveWire(s);
        if (w.lifecycle != kWire[i].lifecycle || w.arming != kWire[i].arming) {
            printf("  state=%u: 得到 lc=%u arm=%u, 期望 lc=%u arm=%u\n", (unsigned)kWire[i].state,
                   w.lifecycle, w.arming, kWire[i].lifecycle, kWire[i].arming);
        }
        CHECK(w.lifecycle == kWire[i].lifecycle);
        CHECK(w.arming == kWire[i].arming);
    }
    // 永不发出的三个值: disarming(3) / Fault(6) / Shutdown(7)
    for (uint32_t i = 0; i < 7u; ++i) {
        Snapshot s;
        s.state = kWire[i].state;
        CHECK(deriveWire(s).lifecycle != 6u);
        CHECK(deriveWire(s).lifecycle != 7u);
        CHECK(deriveWire(s).arming != 3u);
    }
    // task 字段就是 Operation 的线上值
    Snapshot s;
    s.operation = Operation::Takeoff;
    CHECK(deriveWire(s).task == 1u);
    s.operation = Operation::Land;
    CHECK(deriveWire(s).task == 2u);
    s.operation = Operation::None;
    CHECK(deriveWire(s).task == 0u);

    const Mode modes[] = {Mode::Manual, Mode::Stabilize, Mode::AltHold, Mode::VelHold};
    for (uint8_t index = 0u; index < 4u; ++index) {
        s.mode = modes[index];
        CHECK(deriveWire(s).mode == index);
        Task request = Task::None;
        CHECK(taskOfCommand(CommandOp::SetMode, index, request));
        CHECK(modeOf(request) == s.mode);
    }
}

//===================================================================================================
// 四、快照 + 操作状态机的一步(顺带验证 app_task.hpp 与 app_state 的接线)
//===================================================================================================
static void testSnapshotAndOperation()
{
    printf("-- 快照 / 操作状态机\n");
    dlx::LockedCell<Snapshot> published;
    dlx::LockedCell<Snapshot> independent;
    const dlx::LockedCell<Snapshot> *reader = &published;
    const Snapshot initial = reader->load();
    CHECK(initial.state == SystemState::Boot);
    CHECK(initial.mode == Mode::Stabilize);
    CHECK(initial.operation == Operation::None);
    CHECK(initial.health == 0u);
    Snapshot p;
    p.state     = SystemState::Armed;
    p.mode      = Mode::AltHold;
    p.operation = Operation::Takeoff;
    p.health    = 0x5Au;
    published.store(p);

    const Snapshot q = reader->load();
    CHECK(q.state == SystemState::Armed);
    CHECK(q.mode == Mode::AltHold);
    CHECK(q.operation == Operation::Takeoff);
    CHECK(q.health == 0x5Au);

    CHECK(independent.load().state == SystemState::Boot);
    CHECK(independent.load().health == 0u);
    p.state = SystemState::Standby;
    independent.store(p);
    CHECK(independent.load().state == SystemState::Standby);
    CHECK(reader->load().state == SystemState::Armed);

    // 起飞: 从当前高度斜坡上升, 到 kTakeoffHeightM 停, 到位且垂速≈0 才交付
    OperationIn in;
    in.operation  = Operation::Takeoff;
    in.stage      = takeoffStageSpinUp;
    in.altTargetM = 0.0f;
    OperationCtx ctx;
    ctx.dt         = 0.02f;
    ctx.heightM    = 0.03f;
    ctx.vzMps      = 0.0f;
    ctx.canOutput  = true;
    OperationOut out = stepOperation(in, ctx);
    CHECK(out.operation == Operation::Takeoff);
    CHECK(out.stage == takeoffStageClimb);
    CHECK(out.altitude == AltitudeUse::Hold);

    // 动力不能出力(人上锁了): 操作立即作废, 连阶段一起归零
    ctx.canOutput = false;
    out           = stepOperation(in, ctx);
    CHECK(out.operation == Operation::None);
    CHECK(out.stage == 0u);
    CHECK(out.altitude == AltitudeUse::None);

    // 链接管爬升: 必须归一化回"下降"(否则 stage=1 会被当成"已触地", 空中撤掉高度环)
    ctx.canOutput = true;
    ctx.hoverLand = true;
    in.operation  = Operation::Takeoff;
    in.stage      = takeoffStageClimb;
    in.altTargetM = 0.5f;
    ctx.heightM   = 0.5f;
    out           = stepOperation(in, ctx);
    CHECK(out.operation == Operation::Land);
    CHECK(out.stage == landStageDescend);
    CHECK(out.altitude == AltitudeUse::Hold);

    // ⚠ 但已经在 Land 且**已触地**时, 缓降不许把它重新推进下降
    in.operation  = Operation::Land;
    in.stage      = landStageTouchdown;
    ctx.heightM   = 0.02f;
    out           = stepOperation(in, ctx);
    CHECK(out.stage == landStageTouchdown);
    CHECK(out.altitude == AltitudeUse::Idle);

    // 坏高度读数: 触地判据失效, 但目标照常往下走(规则 3)
    in.operation  = Operation::Land;
    in.stage      = landStageDescend;
    in.altTargetM = 1.0f;
    ctx.heightM   = 999.0f; // 超出 kAltMaxM ⇒ 不可信
    ctx.hoverLand = false;
    out           = stepOperation(in, ctx);
    CHECK(!out.heightUsable);
    CHECK(out.stage == landStageDescend);
    CHECK(out.altitude == AltitudeUse::Hold);
    CHECK(out.altTargetM < in.altTargetM);
}

//===================================================================================================
// 五、命令映射 / 通道准入(app_link_cmd)
//===================================================================================================
static void testLinkCmd()
{
    printf("-- 命令映射 / 通道准入\n");

    const GroundStatus statuses[] = {
        GroundStatus::Ok, GroundStatus::BadCommand, GroundStatus::BadArgument,
        GroundStatus::FlashFailed, GroundStatus::NotFound, GroundStatus::Busy,
        GroundStatus::RejectedState, GroundStatus::RejectedArming, GroundStatus::RejectedHealth,
        GroundStatus::RejectedSafety, GroundStatus::Timeout, GroundStatus::AuthFail,
    };
    for (uint16_t index = 0u; index < sizeof(statuses) / sizeof(statuses[0]); ++index) {
        CHECK(static_cast<uint16_t>(statuses[index]) == index);
    }

    // ① 显式映射: 白名单里的每条都要能映射, 且映射结果正确
    struct OpCase
    {
        uint8_t     wire;
        CommandOp   op;
        const char *name;
    };
    static const OpCase kOps[] = {
        {0x00u, CommandOp::Ping, "PING"},        {0x01u, CommandOp::Arm, "ARM"},
        {0x02u, CommandOp::Disarm, "DISARM"},    {0x03u, CommandOp::EmergencyStop, "ESTOP"},
        {0x10u, CommandOp::SetMode, "SET_MODE"}, {0x20u, CommandOp::Takeoff, "TAKEOFF"},
        {0x21u, CommandOp::Land, "LAND"},        {0x22u, CommandOp::CancelTask, "CANCEL"},
        {0x30u, CommandOp::CalStart, "CAL_START"},{0x31u, CommandOp::CalAbort, "CAL_ABORT"},
        {0x40u, CommandOp::ParamGet, "PARAM_GET"},{0x41u, CommandOp::ParamSet, "PARAM_SET"},
        {0x42u, CommandOp::ParamList, "PARAM_LIST"},{0x50u, CommandOp::LinkSetProfile, "LINK_PROFILE"},
        {0x51u, CommandOp::ProtoVersionGet, "PROTO_VER"},{0x60u, CommandOp::EnterMaintenance, "ENTER_MAINT"},
        {0x61u, CommandOp::ExitMaintenance, "EXIT_MAINT"},{0x70u, CommandOp::LogList, "LOG_LIST"},
        {0x71u, CommandOp::LogDump, "LOG_DUMP"},{0x72u, CommandOp::LogErase, "LOG_ERASE"},
        {0x73u, CommandOp::LogFormat, "LOG_FORMAT"},
    };
    const uint32_t nOps = static_cast<uint32_t>(sizeof(kOps) / sizeof(kOps[0]));
    for (uint32_t i = 0; i < nOps; ++i) {
        CommandOp got = CommandOp::Ping;
        CHECK(mapUpCmdOp(kOps[i].wire, got));
        if (got != kOps[i].op) {
            printf("  0x%02X 映射成 %u, 期望 %u(%s)\n", kOps[i].wire, (unsigned)got, (unsigned)kOps[i].op,
                   kOps[i].name);
        }
        CHECK(got == kOps[i].op);
    }

    // ② 穷举 0..255: 能映射的**恰好**是那 21 个 —— 多一个少一个都要在这里露出来
    {
        uint32_t mapped = 0;
        for (uint32_t v = 0u; v < 256u; ++v) {
            CommandOp     got = CommandOp::Ping;
            const bool    ok  = mapUpCmdOp(static_cast<uint8_t>(v), got);
            const uint32_t want = [&]() -> uint32_t {
                for (uint32_t i = 0; i < nOps; ++i) {
                    if (kOps[i].wire == v) {
                        return 1u;
                    }
                }
                return 0u;
            }();
            if ((ok ? 1u : 0u) != want) {
                printf("  0x%02X 映射=%d 期望=%u\n", v, (int)ok, want);
            }
            CHECK((ok ? 1u : 0u) == want);
            if (ok) {
                ++mapped;
            }
        }
        CHECK(mapped == nOps);
    }

    // ③ 通道: 无线只能发飞行相关的 8 条 + PING
    CHECK(commandAllowsWireless(CommandOp::Ping));
    CHECK(commandAllowsWireless(CommandOp::Arm));
    CHECK(commandAllowsWireless(CommandOp::Disarm));
    CHECK(commandAllowsWireless(CommandOp::EmergencyStop));
    CHECK(commandAllowsWireless(CommandOp::SetMode));
    CHECK(commandAllowsWireless(CommandOp::Takeoff));
    CHECK(commandAllowsWireless(CommandOp::Land));
    CHECK(commandAllowsWireless(CommandOp::CancelTask));
    for (uint32_t i = 0; i < nOps; ++i) {
        if (!commandAllowsWireless(kOps[i].op)) {
            CHECK(admitCommandChannel(kOps[i].op, /*wireless=*/true) == GroundStatus::AuthFail);
        }
    }
    CHECK(admitCommandChannel(CommandOp::ParamGet, /*wireless=*/false) == GroundStatus::Ok);
    // ⚠ `admitCommandChannel()` 现在**就是准入的全部**: 它只答"通道对不对",
    //   不再答"状态允不允许"(那一半随 `admitTask()` 删除, 见 docs 的连锁问题 ②)。

    // ④ 命令 → Task: 载荷(arg)只在这里被消化成"一位一档"
    {
        app::Task t = app::Task::None;
        CHECK(taskOfCommand(CommandOp::Takeoff, 0u, t) && t == app::Task::RequestTakeoff);
        CHECK(taskOfCommand(CommandOp::Land, 0u, t) && t == app::Task::RequestLand);
        CHECK(taskOfCommand(CommandOp::CancelTask, 0u, t) && t == app::Task::RequestCancel);
        CHECK(taskOfCommand(CommandOp::Arm, 0u, t) && t == app::Task::RequestArm);
        CHECK(taskOfCommand(CommandOp::Disarm, 0u, t) && t == app::Task::RequestDisarm);
        CHECK(taskOfCommand(CommandOp::EmergencyStop, 0u, t) && t == app::Task::RequestEStop);
        CHECK(taskOfCommand(CommandOp::SetMode, 0u, t) && t == app::Task::RequestModeManual);
        CHECK(taskOfCommand(CommandOp::SetMode, 1u, t) && t == app::Task::RequestModeStabilize);
        CHECK(taskOfCommand(CommandOp::SetMode, 2u, t) && t == app::Task::RequestModeAltHold);
        CHECK(taskOfCommand(CommandOp::SetMode, 3u, t) && t == app::Task::RequestModeVelHold);
        // 越界的 arg **必须**返回 false(否则会落进"最后那个 else"被当成某一档模式执行)
        CHECK(!taskOfCommand(CommandOp::SetMode, 4u, t));
        CHECK(!taskOfCommand(CommandOp::SetMode, 255u, t));
        // PING 与没接线的命令没有对应请求
        CHECK(!taskOfCommand(CommandOp::Ping, 0u, t));
        CHECK(!taskOfCommand(CommandOp::ParamGet, 0u, t));
    }
}

//===================================================================================================
// 六、健康项: 存储是"一项一个 bool", 上报是**一个字节** —— 位序必须与 SensorHealth 顺序一致
//===================================================================================================
static void testEmergencyHandshake()
{
    EmergencyStopHandshake handshake;
    const uint32_t first = handshake.request();
    CHECK(handshake.pending);
    CHECK(!handshake.acknowledge(first + 1u));
    const uint32_t second = handshake.request();
    CHECK(second != first);
    CHECK(!handshake.acknowledge(first));
    CHECK(handshake.acknowledge(second));
    CHECK(!handshake.pending);
}

static void testHealthAge()
{
    Health health;
    CHECK(health.success(SensorHealth::IMU, 100u));
    HealthSnapshot snapshot = health.snapshot();
    CHECK(snapshot.sources[static_cast<uint8_t>(SensorHealth::IMU)].lastSuccessMs == 100u);
    CHECK(snapshot.ok(SensorHealth::IMU, 100u));
    CHECK(!snapshot.ok(SensorHealth::IMU, 100u + app::ImuFreshnessMs + 1u));
    CHECK(health.failure(SensorHealth::IMU));
    CHECK(!health.snapshot().ok(SensorHealth::IMU, 100u));
}

static void testHealth()
{
    Health health;
    CHECK(health.snapshot().bits(0u) == 0u);
    CHECK(health.success(SensorHealth::BARO, 0u));
    CHECK(!health.success(SensorHealth::BARO, 0u));
    CHECK(health.snapshot().ok(SensorHealth::BARO, 0u));
    CHECK(!health.snapshot().ok(SensorHealth::RANGE, 0u));
    CHECK(health.failure(SensorHealth::BARO));
    CHECK(!health.failure(SensorHealth::BARO));
    CHECK(!health.snapshot().ok(SensorHealth::BARO, 0u));
    CHECK(static_cast<uint8_t>(SensorHealth::TOTAL_SIZE) == 7u);
    for (uint8_t index = 0u; index < static_cast<uint8_t>(SensorHealth::TOTAL_SIZE); ++index) {
        CHECK(health.success(static_cast<SensorHealth>(index), 0u));
        CHECK(health.snapshot().bits(0u) == ((1u << (index + 1u)) - 1u));
    }
    CHECK(health.failure(SensorHealth::FLOW));
    CHECK(health.snapshot().bits(0u) == 0x77u);
    CHECK(health.snapshot().ok(SensorHealth::RANGE, 0u));
    CHECK(health.snapshot().ok(SensorHealth::GNSS, 0u));
    CHECK(!health.success(SensorHealth::TOTAL_SIZE, 0u));
    CHECK(!health.failure(SensorHealth::TOTAL_SIZE));
    CHECK(!health.snapshot().ok(SensorHealth::TOTAL_SIZE, 0u));
    CHECK(dlx::sampleFresh(4u, UINT32_MAX - 5u, 10u));
    CHECK(!dlx::sampleFresh(5u, UINT32_MAX - 5u, 10u));
    Health independent;
    CHECK(independent.snapshot().bits(0u) == 0u);
}

int main()
{
    printf("=== Task / SystemState 主机测试 ===\n");
    testTaskVocabulary();
    testWire();
    testSnapshotAndOperation();
    testLinkCmd();
    testHealth();
    testHealthAge();
    testEmergencyHandshake();

    printf("=== %d 项检查, %d 项失败 ===\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
