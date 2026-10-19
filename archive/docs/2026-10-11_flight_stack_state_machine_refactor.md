# 2026-10-11 flight_stack 状态机重构（实施计划）

> **状态**：**已实施**（2026-10-11）。本文是**定稿规格**，已按落地结果修订；实施过程中的坑与经验
> 在 [工作记录](2026-10-11_flight_stack_refactor_notes.md)，本文不重复。
> **范围**：只重写状态机（`State` / 请求通道 / 迁移与准入）+ 把业务准入移出 `dlx_link_sched`。不含飞行算法，不改空口格式（**除 §3.6 那条 lifecycle 语义纠正**）。
> **行号**：附录里的 `文件:行` 是 **重构前**的口径，实施后已失效，只作依据。

---

## 1. 目标

1. **一个扁平状态枚举**（7 态）+ **一个请求位图**（每 Task 一位，无载荷）+ **迁移表与准入表（数据）**；
2. **业务词汇与准入策略离开 DL_LIB**，DL_LIB 只留与业务无关的可迁移基础代码；
3. 派生量一律删除改成派生：`en`/`starting`/`motorStart`/`stop`/`hardStop`/`st.health`/`stateSnap` 等；
4. 顺带修掉两个"线上有字段/有拒绝原因、但没人消费或没人回报"的洞（§4、§7.4）。

---

## 2. 决策表

| # | 结论 | 一句理由 |
|---|---|---|
| D1 | 扁平 **7 态**：`Boot/InitLow/InitHigh/Standby/Maintenance/Arming/Armed`；`Standby` = 未解锁 | lifecycle×arming 本不独立（旧的 `kLifeNeedsDisarmed` 就是为"两维能拼出非法组合"而存在）；合并后非法组合**结构上不存在** |
| D2 | 删 `Fault` / `Shutdown` / `Disarming` | 前两个不可达；`Disarming` 是空态（停机同步），空口 `arming=3` 由 wire 映射处理 |
| D3 | 上电自检失败**不进 Fault**，停 `Standby`/`Maintenance`，由解锁门槛拒绝 | 保持既有取舍：Fault 会把参数/日志/维护一起关掉，而那正是排查要用的 |
| D4 | `Maintenance` = 原 `BootMode::SerialDebug` 语义（**允许 ARM + Manual 试车**）；由 `InitHigh` 出口按 `app.mode` 选；`LogManage` 留在 `release.cpp` 分发层 | `LogManage` 根本不进 FlightStack（`release.cpp:145`）；`BootMode` 是"本次运行的意图"，`Maintenance` 是它的运行期表达 |
| D5 | 请求通道 = **一个 32 位位图**（每位一个外部 Task）；申请=置位，控制环在**同一个临界区里读出并清除** | 一次读就能做碰撞检测；读-改-写只在一处保护 |
| D6 | **Task 不带任何载荷**（无 `arg` 字节）；需要选择值的请求按"**一位一档**"展开 | 位图模型里没有载荷的位置；"最后一个值赢"这种隐式状态不值得引入 |
| D7 | 同时多位置位 → **抛异常 + 按固定优先级执行一位**（§3.4） | "同一时刻只有一个有效"是设计不变量，违反即为 bug，要能被发现 |
| D8 | `Task` 与 `Event` **合并成一个枚举**（都表示"状态转移请求"）；但**存储不合并** | 合并词汇 ⇒ 一张表、一个入口；不合并存储 ⇒ 共享字保持**单写者**（内部事件是控制环局部变量） |
| D9 | 飞行模式**由控制环写**；链路只发模式请求 → 准入 → 控制环提交 | 若链路直写，`degradeMode()` 自动降档下一帧被覆盖 ⇒ 自动降级失效、准入表形同虚设 |
| D10 | 链路状态**派生**：`linkStateOf(health, streak)` 纯函数，不新增存储 | 已有两个真相源（`health::link` + `uplinkStreak`），再放一个可变变量就是第三份 |
| D11 | **不加** ARMING 超时 | 没法检测：`timer1.start/stop/enableOutputs`、`dshot.start/stop` **全部返回 void**；真正检测要等双向 DShot |
| D12 | 空口**格式不变**：内部扁平 + `deriveWire(State)` 映射回旧位域 | 遥控器零改动 |
| D13 | 链路丢失（空中，已接管）→ **缓降 HoverLand**；持续 `hoverLandMs` 才动作 | 空中断动力 = 自由落体；地面/ARMING 中才是停机（§3.8） |
| D14 | 链路恢复 → **立即交还控制权**（不锁存） | Operation 仍是 Land 到底，操作人可 CANCEL |
| D15 | 迁移与准入**都用数据表**（迁移表每行带效果位 + dwell），不用 switch-case | 加状态/事件时必须有行（行数可数）；纯数据可主机测 |
| D16 | **方案甲**：命令准入与"线上↔业务枚举"映射搬出 `dlx_link_sched`，库里只留协议事实 | §4，含一个实锤 bug |
| D17 | DL_LIB = **与业务无关、可迁移的基础代码**；控制器/PID/滤波留在库里是**另外的决策**，本轮不动 | 推论：`app_state` 是业务词汇 ⇒ **DL_LIB 绝不 include 它** |
| D18 | `flight_task.hpp` 搬出 DL_LIB → `app_task.hpp` | 它是操作策略（何时结束、边界条件），不是可迁移积木 |
| D19 | 单核上**不设可自旋的锁**，互斥一律用关中断临界区（`dlx::InterruptLock`） | 真自旋锁是 SMP 原语：持有者可能是更低优先级线程（链路 24），自旋者是更高优先级（控制环 10）⇒ **必然死锁** |
| D20 | **不用** RT-Thread 的 `rt_mb`/`rt_event` | `rt_mb` 是队列语义（与"操作手意愿=最新值"冲突、抹掉碰撞检测、装不下 ESTOP 锁存、满时把安全决策塞进返回值）；`rt_event` 语义接近但 `RT_USING_EVENT` **未开启**、传不了值、控制环 500Hz 每圈一次 IPC、把 RTOS 拖进叶子头 |
| D21 | `FatalFault` 从"故障枚举"降级为**内部事件 `Cut`**；`InitFailed` 一并消失 | **记录交出去、动作不能交出去**：异常处理器可能在任意上下文（含中断）被调用，不能碰状态机与硬件 |

**硬约束**：`app_state.h` 是业务词汇（叶子头：只 `stdint`）；线上编号/位宽是协议事实，留 DL_LIB；业务枚举与准入策略在 app 层。

---

## 3. 规格

### 3.1 词汇表（`app_state.h`）

```cpp
namespace app {

enum class State : uint8_t {          // 7 态
    Boot, InitLow, InitHigh,
    Standby,                          // 地面待命 = 未解锁; 停机动作也回到这里
    Maintenance,                      // 台架串口调试(原 BootMode::SerialDebug)
    Arming, Armed
};

// 一个枚举 = "状态转移请求"的全部词汇。不带载荷(D6)。
enum class Task : uint8_t {
    // ---- 外部: 值号 = 位号, 顺序即优先级; 只有这些占位、只有这些进共享字 ----
    EStop = 0, Disarm, Arm, Cancel, Land, Takeoff,
    ModeStabilize, ModeAltHold, ModeVelHold, ModeManual,   // 模式选择 = 一位一档
    foreignCount,                      // 外部请求个数 = 位图宽度
    None = foreignCount,               // 这一圈没有请求
    // ---- 内部: 控制环本地产生, 不占位、不进共享字 ----
    PowerOn, InitLowDone, InitHighDone, ExitMaint, StartDone, Cut,
};
constexpr bool isForeign(Task t);     // t < foreignCount
constexpr bool isInternal(Task t);    // t > None
constexpr uint32_t taskBit(Task t);   // 只有外部请求有掩码
constexpr bool isModeTask(Task t);    // ModeStabilize/ModeAltHold/ModeVelHold/ModeManual
constexpr bool isOperationTask(Task t);// Takeoff/Land/Cancel(原地类: 不动 State)
constexpr Mode modeOf(Task t);
constexpr Task modeTask(Mode m);

enum class Mode : uint8_t { Manual = 0, Stabilize, AltHold, VelHold };   // 值沿用 dlx 那套, 线上编号不动
enum class Operation : uint8_t { None = 0, Takeoff, Land };              // 持续操作(原 TaskId) + stage + 目标高度
// ⚠ 失效保护缓降**不在** Operation 里, 它是正交的输入(见 app_task.hpp 的 OperationCtx::hoverLand)
//    与 [工作记录 §2](2026-10-11_flight_stack_refactor_notes.md): 折进去会让已触地的 Land 重新下降。
enum class LinkState : uint8_t { Down, Syncing, Steady };                // 派生, 无存储

} // namespace app
```

**命名冲突一并解决**：扁平枚举叫 `State`（旧的 `app_state.h` 里那个 `SystemState` enum 与之撞名，删）；快照结构叫 `Snapshot`（旧的 `dlx::SystemState` struct）；`FlightMode` 的两份定义合并为 `Mode`，值取 dlx 那套。

### 3.2 请求通道

```cpp
void postTask(Task t);     // 申请(链路线程): 临界区里置位。只有外部请求能被申请
uint32_t takeTasks(void);  // 消费(控制环): **同一个**临界区里读出并清空(**连 ESTOP 那一位一起取走**)
bool estopLatched(void);   // ESTOP 锁存着吗(守卫)
void clearEstop(void);     // 清锁存 —— 只允许在"一次走通完整前置的 ARM"之后
```

- **申请**（链路线程）：`postTask(t)`，在 `dlx::InterruptLock` 临界区里置位。
- **消费**（控制环）：`takeTasks()` 在**同一个临界区**里读出并清空。等价于"申请置位 / 处理完清除"，但没有"读出与清除之间丢请求"的窗口——期间链路新置的位留到下一圈。
- **碰撞**（D7）：一次读出的位数 > 1 → 抛异常 + 计数，然后按优先级执行一位、其余清除。
- **ESTOP**：**位与锁存分开**（方案原文写的是"消费路径跳过这一位"，那样**单按急停什么都不会发生** —— 详见[工作记录 §1.1](2026-10-11_flight_stack_refactor_notes.md)）。
  - **位**照常被取走 ⇒ 这一拍就是 `Task::EStop` ⇒ 停机（不依赖链路补第二位）；
  - **锁存**独立持久化 ⇒ 置位期间除 `Arm`/`Disarm` 外一律拒绝，且禁止任何自动缓降重新给动力；
  - 锁存的**唯一清除路径**是"一次走通完整前置的 ARM"；之后按 ARM 时字里只有 ARM 一位，不会撞成碰撞。
- **临界区**用 `DL_LIB/dlx_interrupt_lock.hpp` 的 `InterruptLock`（PRIMASK 保存/恢复），**不是** `rt_enter_critical()`（后者只挡调度、不挡中断），**不是**自旋锁（D19）。
- **载荷**：没有。任何将来需要传值的请求只能"一位一档"（见 §7.4 的 Takeoff 高度）。线上 `SET_MODE.arg` 在 `app_link_cmd.hpp` 的 `taskOfCommand()` 里被消化成 4 个模式位之一 —— 那是**唯一**一处"载荷被消化掉"的地方。

**执行优先级**（碰撞时取一位执行，准入见 §3.4）：
```
EStop > Disarm > Arm > Cancel > Land > Takeoff > ModeStabilize > ModeAltHold > ModeVelHold > ModeManual
```
模式组内部按"**控制权限从低到高**"排（Stabilize 先于 AltHold 先于 VelHold），`ModeManual` 放最后（它只在未解锁/维护态准入）。

### 3.3 迁移表 + 效果

效果位（**边沿执行一次**，由状态机这个唯一执行点跑）：

| 效果 | 内容 | 用在哪 |
|---|---|---|
| `StartPower` | `timer.start` + `enableOutputs` + `dshot.preload(0,0,0,0)` + `dshot.start` + 开始气压基准累积 | `→Arming` |
| `Takeover` | 控制接管 + 提交气压基准 + `ekf.reset` 归零 | `Arming→Armed` |
| `StopPower` | `timer.stop` + `disableOutputs` + `dshot.stop` + 取消基准累积 | `→Standby` |
| `PurgeDirect` | 清 UP_MOTOR 直控缓存 | 进 Manual 那一拍(原地类, 不走迁移表) |
| `None` | — | 其余 |

| from | Task（外=共享字 / 内=本地） | to | 效果 | 备注 |
|---|---|---|---|---|
| Boot | PowerOn（内） | InitLow | — | |
| InitLow | InitLowDone（内） | InitHigh | — | |
| InitHigh | InitHighDone（内） | **Standby** | 自检汇报 | `guards.maintenance == false` |
| InitHigh | InitHighDone（内） | **Maintenance** | 自检汇报 | 否则 |
| Maintenance | ExitMaint（内） | Standby | — | 今天无触发者，见 §8 |
| Maintenance | Arm（前置已过） | Arming | StartPower | 台架试车 |
| Standby | Arm（前置已过） | Arming | StartPower | |
| Arming | StartDone（内，固定 dwell `motorArmDelayMs`） | Armed | Takeover | |
| Arming | Disarm / EStop / Cut（内） | **armOrigin** | StopPower | `Cut` = `fsLevel==Cut` / 链路超时且未接管 |
| Armed | Disarm / EStop / Cut（内） | **armOrigin** | StopPower | |
| Standby | Disarm / EStop | Standby | StopPower | 幂等自环，**不报非法** |
| Maintenance | Disarm / EStop | Maintenance | StopPower | 幂等自环 |
| Boot / InitLow / InitHigh | Disarm / EStop | 原位 | — | 幂等自环；硬件没起来 ⇒ 效果位是 `None` |

- **`armOrigin`**（§8-1 的决定）：Arm 那一拍记下"从哪解锁的"，Disarm 回到那里 —— 与重构前逐字一致
  （那时 lifecycle 在解锁/上锁时不动），台架试车后不必重进维护态。非法/未设的值一律回 `Standby`。
- **原地类 Task**（`State` 不动）：模式类（`ModeXxx`）→ `Mode`（查准入 + `modeSwitchAllowed`）；`Takeoff`/`Land`/`Cancel` → `Operation`（推进在 `app_task.hpp`，纯函数）。
- **启动三态**由顺序代码推进（`PowerOn`/`InitLowDone`/`InitHighDone`），**不可被外部请求打断**：它们只让"上电三段"在日志/遥测里可见，不参与运行期裁决。**但它们照样走同一个执行点**（`FlightStack::startPhase()`），于是"`state` 在哪儿被改"仍然只有一个答案。
- **内部事件绕过准入**：`admitTask()` 对 `isInternal(t)` 直接放行 —— 否则 ESTOP 锁存会把上电三段挡死在 `Boot` 里。`Cut` 的优先级高于一切外部请求。
- **状态内计时**：`stateEnteredMs`（迁移那一拍记录）。`Arming` 只用 minDwell，无 timeout（D11）。
- **唯一执行点**：

```cpp
Step stepState(State from, Task on, const Guards &g);   // 纯函数, 主机可测
```

`Guards` 里放的是"从外部算得出来的事实"（`stateElapsedMs` 而不是两个时间戳：状态机只需要"待了多久"，
传 `nowMs` + `enteredMs` 会让它多一个可以算错的减法）。

状态只在这里被改；副作用从"每圈判一次"改成"迁移那一拍执行一次"，于是 `en`/`starting`/`motorStart`/`stop`/`hardStop` 全部消失或派生。

### 3.4 准入表（数据，链路与控制环读同一份）

| Task | 允许的 State | 附加前置 |
|---|---|---|
| `Arm` | Standby, Maintenance | `bootReady` + IMU 正常 + 链路稳 + 油门低位 + 杆量中位（**顺序即报错码**：从"不可操作"到"操作手自己能改"） |
| `Disarm` | 任意（含上电三段，幂等） | 无（永远有效） |
| `EStop` | 任意（含上电三段，幂等） | 无（永远有效，置锁存） |
| `ModeXxx`（4 个） | Standby, Maintenance（= 未解锁） | `modeSwitchAllowed(当前, X, state, health, explicit=true)`；不通过 → `RejectedHealth` |
| `Takeoff` / `Land` | Armed | `checkTaskPreconditions`（模式非 Manual、未高于起飞高度） |
| `Cancel` | Armed | `checkTaskPreconditions`（缓降期间不接受） |
| 内部事件（`PowerOn`…`Cut`） | 看迁移表有没有那一行 | **直接放行** —— 它们由控制环自己排队，没有通道与"操作手前置"可言；否则 ESTOP 锁存会把上电三段挡死在 `Boot` |
| 其余有线命令（`CalStart/Abort`、`ParamGet/Set/List`、`LinkSetProfile`、`ProtoVersionGet`、`Enter/ExitMaintenance`、`Log*`） | — | 无线侧一律 `AuthFail`（`commandAllowsWireless`）；有线侧"表里允许但未接线 → 回 `BadCommand`" |

**EST0P 锁存守卫**：锁存期间除 `Arm`/`Disarm` 外一律 `RejectedSafety`，且禁止任何自动缓降重新给动力。

> **未实现的一处**（原方案 §3.4 曾写"Maintenance 下放行 IMU 类故障 = 台架例外"）：
> 本轮**保持行为不变**（任何状态都要求 IMU）。理由是那会把"台架例外"变成"没装/IMU 坏了也能转桨"，
> 属于**产品策略**而不是重构，应该单独决策。见 §8。

### 3.5 链路状态（派生）

```cpp
inline LinkState linkStateOf(uint8_t healthBits, uint16_t uplinkStreak)
{
    if ((healthBits & health::link) == 0u) return LinkState::Down;
    return (uplinkStreak < minValidUplink) ? LinkState::Syncing : LinkState::Steady;
}
// 链路/控制环共用的那个谓词在 app_guards.hpp:
inline bool linkIsSteady(uint8_t healthBits, uint16_t uplinkStreak);   // 两个条件缺一不可
```

既有真相源：`app::health::link`（链路线程在失同步/重同步边沿翻转）、`uplinkStreak`、`lastValid`。
注：`linkTimeoutMs` 与 `hoverLandMs` 现在都是 2000，语义不同但当前对齐，配置注释里写明。

### 3.6 空口映射

| State | DL_STATE.lifecycle | DL_STATE.arming / DL_ATT.arming |
|---|---|---|
| Boot / InitLow / InitHigh | 0 / 1 / 2 | 0 |
| Standby | 3 (STBY) | 0 |
| Maintenance | 5 (MAINT) | 0 |
| Arming | 4 (ACT) | 1 |
| Armed | 4 (ACT) | 2 |

`disarming(3)` / `Fault(6)` / `Shutdown(7)` 永不发出。
`DL_STATE.task` = `Operation` 的线上值（0/1/2）；**值 3 历史上是 `Calibrate`（从未接线），不要复用**。

**⚠ 语义变化（决定：纠正为 3）**：今天"链路稳但未解锁"是 `Active(4)`，新模型下 `Standby` 就是"未解锁"，
一律发 `3(STBY)`。原方案曾建议"wire 层保留旧口径（链路稳也发 4）"，但那要求把"链路稳"塞进 `deriveWire()`，
使它不再是纯函数。**改为纠正语义，由遥控器侧同步**（用户 2026-10-11 确认）。
所以 `deriveWire(const Snapshot &)` 是纯函数、可主机逐值对拍。

### 3.7 单写者表

| 变量 | 写者 | 读者 |
|---|---|---|
| 请求位图 | 链路线程（`postTask`） | 状态机（`takeTasks` 取走即清） |
| ESTOP 锁存 | 链路线程置位 / 控制环在成功 ARM 后清除 | 所有人（`estopLatched`） |
| `State` | `stepState()`（控制环） | 所有人（经 `snapshot()`） |
| `Mode`、`Operation`(+stage+目标高度) | 状态机 | 所有人 |
| `FailsafeLevel/Reason` + latch | 状态机（每圈推导的结果） | 链路 / 日志 |
| health bits | 各数据源（`app_health.hpp`） | 所有人 |

`FailsafeLevel/Reason/latch` **不是状态**，不进 `State`。

### 3.8 链路丢失的动作分层

| State | 链路丢失 | 为什么 |
|---|---|---|
| Standby / Maintenance | **无状态动作** | 未解锁，链路只是准入条件 |
| Arming（桨已转、控制未接管） | **StopPower → Standby** | 地面上"停"永远比"降"安全，此刻没有"降"这回事 |
| Armed（控制已接管） | **缓降 HoverLand** | 持续 `hoverLandMs` 才动作，`kHoverLandDescentMps` 降到软地板等人上锁 |
| 恢复 | **立即交还控制权** | 不锁存；Operation 仍 Land 到底，可 CANCEL |
| DISARM / ESTOP | **任何状态永远有效** | 人上锁是最后出口 |

判据从今天的 `linkTimedOut && !en && starting` 变成 `State == Arming`（**Armed ⟺ 控制已接管**）。链路丢失由**控制环推导**成内部事件，不由链路层发停机请求。

---

## 4. DL_LIB 隔离（方案甲）

**实锤 bug（方案甲要修的就是它）**：`lim.checks = kCheckAll`（含 `kCheckCommand`）⇒ 每条 UP_CMD 都进 `checkCommand()` ⇒ 调 `admitCommand(m.op, true, *st)`；状态不允许时回 `app_link_thread.hpp:267` 的 `++linkAdmitRejects; return;` —— **一个 ACK 都不回**。而注释说的"按 `lastRejectStatus()` 回 DL_ACK"从未实现（那三个 getter **零调用者**）。后果：带锁按 SET_MODE、未解锁按 TAKEOFF、维护态发大多数命令 → 遥控器上**什么都没有**。未知 op 同样静默丢弃，app 层那段 `BadCommand` 是死代码。

**归属划分**：

| 内容 | 归属 | 落地位置 |
|---|---|---|
| 帧表 / CRC / SEQ 窗口 / 到达间隔 / 字段量级 | 留 DL_LIB（纯协议/物理） | `dlx_link_sched.hpp` 不变 |
| `arg` 域检查、`UnknownOp` | 留 DL_LIB，改用**线上 op 编号常量**（`kWireOpSetMode` / `kWireOpTakeoff` / `kWireOpEnterMaintenance`） | 同上 |
| `admitCommand` 调用、`StateNotAllowed`、`m_lastReject`、三个零调用者、`admit()` 的 `st` 参数 | **删 / 搬 app** | 删除；准入 → `app_state.cpp` 的 `admitTask()` |
| `CommandOp`、`GroundStatus`、`Mode`、`State` | 搬 app（业务词汇）；DL_LIB 只留"这个字节在不在协议表里"（`isKnownWireOp` / `isKnownWireResult`）+ 线上编号 | `app_state.h` + `app_link_cmd.hpp` |
| `mapUpCmdOp` / `mapDlAckResult`（线上↔业务枚举） | 搬 app（业务接线） | `app_link_cmd.hpp`：`mapUpCmdOp(wireOp, out)`（bool 出参）/ `wireOpOf(op)` / **`isKnownWireResult()`** 留在 DL_LIB |
| `checkCommand` 的位名 | 改为 `kCheckCmdArg`（不再含"状态"语义） | 同上 |
| **新增**：`Mode` 的线上下界、`failsafeLvl/Rsn` 的线上下界 | DL_LIB（协议事实） | `kWireModeMax` / `kWireFailsafeLvlMax` / `kWireFailsafeRsnMax` |
| **协议变更（外围，2026-10-12）**：`UP_MOTOR` 由"单路 4B"改成"四路一次发全 8B" | 见[工作记录 §14](2026-10-11_flight_stack_refactor_notes.md) | `dlx_link_frame.hpp` 的 `upMotor`；连带删掉 `motorCmdIndex` / `motorDirectRejects` / `applyMotorDirect` |

收益：不用写两份；`dlx_link_sched.hpp` **零业务依赖**（连 `CommandOp`/`GroundStatus` 都不认识）；上面的洞自动消失（准入只剩 app 一处，而那一处本来就会回 ACK）。副作用：`采信拒` 计数语义变准（只剩协议/时序类拒绝），状态类拒绝改由 `cmdsRejected` 计数。

下行**已经是**"协议结构体由外面填 raw 数字"的模式（填 `deriveWire(snapshot)` 出来的 raw 值），本次只是让上行与它一致。

**另外加了一层防漂**：`DL_ACK` 的依据除了"准入表"还有**量化前置**（油门低位 / 杆量中位），阈值来自配置。
链路层与控制环要读**同一份阈值**，所以阈值、比较、"链路稳不稳"都收在 `app_guards.hpp` 里，
两个调用点只填各自拥有的字段。详见[工作记录 §5](2026-10-11_flight_stack_refactor_notes.md)。

---

## 5. 文件清单与实施顺序

### 5.1 新建 / 改写 / 删除

| 动作 | 文件 | 说明 |
|---|---|---|
| 新建 | `app_state.h` / `app_state.cpp` | 叶子头（7 态 + `Task` + `Mode` + `Operation` + `LinkState` + 健康位 + 失效保护 + `GroundStatus` + `CommandOp` + 位图/快照访问器声明 + `stepState()`/准入/`deriveWire()`）；`.cpp` 放共享量（匿名命名空间）、迁移表、准入表、纯函数（**无硬件**；效果的**执行**在 flight_stack） |
| 新建 | `app_task.hpp` | 原 `DL_LIB/flight_control/flight_task.hpp` 的操作状态机（`Operation` + stage + 目标高度），纯函数原样搬 + `checkTaskPreconditions()` |
| 新建 | `app_guards.hpp` | `app::Guards` 的两个"量"判据 + 阈值：链路与控制环**共用一份** |
| 新建 | `app_link_cmd.hpp` | app 侧链路接线：命令词汇、显式映射、通道准入、`taskOfCommand()`（把 `SET_MODE.arg` 消化成模式位） |
| 新建 | `app_health.cpp` | 健康位的存储（匿名命名空间）+ 四个访问器（替掉"返回静态变量引用"那种写法） |
| 新建 | `test/state_step_test.cpp` | 主机测试（一条 `g++` 命令） |
| 新建 | `tools/build_check.ps1` | 重放 EIDE 的 `compile_commands.json`，用真 armclang 做编译/链接检查 |
| 改写 | `app_flight_stack.hpp` | `stepStateMachine` 段替换为唯一执行点 + `applyEffects()` / `applyInPlace()` / `startPhase()`；成员表删影子量与 `stateSnap` 副本 |
| 改写 | `app_link_thread.hpp` | 写 `postTask`（替代 5 个请求位）；读 `snapshot()` 回 ACK 与填下行；链路丢失**不再**由它发停机 |
| 改写 | `app_health.hpp`、`release_header.h`、`app_boot_mode.hpp`、`release_log_manage.hpp` | 词汇来源改为 `app_state.h`；`GroundStatus` → `app::GroundStatus`；健康位 → `app::health::*` |
| 改写 | `DL_LIB/link/dlx_link_sched.hpp` | 按 §4 移除业务准入与 `st` 参数 |
| 删除 | `DL_LIB/dlx_system_state.hpp` | 805 行，内容按 §3/§4 分流 |
| 删除 | `DL_LIB/flight_control/flight_task.hpp` | 已搬成 `app_task.hpp` |
| 删除 | `flight_config_struct.hpp` | 已弃用（无人 include），全部由 `release_header.h` 取代 |
| 删除 | 旧 `app_state.h` 里的 `SystemState`/`FlightMode`/`SystemTask` 与三个 setter | 死代码（无 include 者） |
| 删除 | `release_header.h` 的 `stateSnap` 与 `enabledMotor`/`startingMotor`/`motorStartTime` | 状态的第三份副本 / 影子量 |

### 5.2 顺序（实际执行）

1. `app_state.h/.cpp` + `app_task.hpp` + `app_guards.hpp` + `app_link_cmd.hpp`（纯逻辑，不动现有代码）→ 主机测试跑通迁移/准入/映射（§6.1）。
2. `dlx_link_sched.hpp` 摘除业务准入（§4）。
3. `app_flight_stack.hpp` 换状态机（唯一执行点 + 迁移效果）→ 删影子量。
4. `app_link_thread.hpp` 换请求字 + 快照来源。
5. 删除旧文件与死代码；`.eide/eide.yml` 登记两个新 .cpp；全量编译检查（§6.3）。

---

## 6. 验证清单

### 6.1 主机测试（`test/state_step_test.cpp`，一条 `g++` 命令，不引入构建系统）

```
g++ -std=c++11 -Wall -Wextra -I test/stub -I DL_LIB -I . \
    test/state_step_test.cpp app_state.cpp app_health.cpp -o state_step_test && ./state_step_test
```

**结果：786 项检查，0 失败**（`-Wall -Wextra` 零告警）。覆盖：

- 迁移表**每行**（24 行，含 §3.3 新增的幂等自环）；16 条非法迁移 → `matched=false` 且原地不动；
- `Arming` 的 dwell 到点才进 `Armed`（差 1 ms 不行）；
- ESTOP：置位即停机 + 锁存；锁存期间除 `Arm/Disarm` 外全拒；**只有走通完整前置的 ARM 才清除**；
- 同时多位置位 → 检测到 + 优先级正确（含模式组内部顺序）；`Task` 值号序即优先级序；
- 链路丢失三态分层（Standby 无动作 / Arming 停机 / Armed **持续**够久才缓降）；
- `armOrigin`（含非法来源值回退到 Standby）；
- 准入表 + **结构不变量**："准入通过 ⟹ 状态级准入通过"（两张表不许互相矛盾）；
- `deriveWire()` 逐值对照旧位域（含"永不发出的三个值"）；
- 命令映射**穷举 0..255**（能映射的恰好是协议表里那 21 个）、通道准入、`SET_MODE.arg` 越界必须拒；
- 杆量判据的边界（恰好等于阈值算通过，多 1 不算）。

### 6.2 上板回归（**未做**）

解锁全流程（含每条拒绝原因）、ARMING 启动、DISARM 停机、ESTOP 锁存→重解锁、失效保护缓降、链路恢复交还、Manual 直控、模式切换与自动降级、`LogManage→SerialDebug/Flight` 交接。

### 6.3 编译检查（真 armclang，`tools/build_check.ps1`）

| 范围 | 结果 |
|---|---|
| 全工程 33 个 TU | **32 通过** |
| `release.cpp` | **失败**，但只错在它自己 3 行引用了工作树里**根本没有定义**的符号：`DLX_SET_EXCEPTION_TEXT`/`_REPORT`（133-134）、`isOkCode`（235）。另在 `app_flight_stack.hpp` 有 4 处同类：`applyMotorDirect`、`DLX_SET_EXCEPTION_IMMEDIATE`/`_LED`/`_PHASE`（含未定义的 `ExceptionPhase`）。**与本次重构无关**（引用行本次一行未动） |
| `app_state.cpp` / `app_health.cpp` | 0 错误 |
| `app_flight_stack.hpp` + `app_link_thread.hpp` | 一次性空壳探针下完整类型检查 0 错误 |

### 6.4 风险点（上板重点盯）

1. **链路层 ACK 路径**（改读 `snapshot()`；§4 的洞修好后拒绝码才会出现）；
2. **遥测组装**（删 `stateSnap` 后的来源）；
3. **`Arming` 驻留期间"桨在转但控制未接管"**必须与今天逐字一致（`timer1.start/enableOutputs/dshot.start` 的顺序与时机）；
4. **空口**：`deriveWire()` 前后逐值对照（§3.6 的语义变化要在遥控器侧确认）。

---

## 7. 本轮不改（记录）

### 7.1 `linkVer` / `linkType` / `linkAttempts` / `linkKeyWritten` → 局部量

**状态：本轮未做**（与本轮范围无关，保持原样）。

- `linkVer`/`linkType`：**读到就地打**，不要存下来拖到函数末尾；省 52 字节常驻。
- `linkAttempts`：重试循环本就在 `bringUpLink` 里 → 局部量。
- `linkKeyWritten`：删。**但必须先补 `LINK_KEY_WRITE_FAILED = 0x0210`**：今天 `setKey` 失败走 `linkFailCode(link.lastFail())`，报告里看不出是"密钥这一步"失败，唯一点明它的是那行日志。

### 7.2 `FlowSample` 两份 / `rangeM`

- **不是冗余**：`sensors.flowSnap` 是生产者信箱；`FlightStack::lastFlow` 是控制环**最近一次真正喂进估计器**的那一份。`stepFlowObs` 用 `written()` 门控"同一份只喂一次"（不判序号会把同一观测喂 25 次 ≈ 量测方差除以 25）；日志记"这一圈估计器吃的是什么"，现读快照会记成"还没被吃的那一份"。
- `rangeM` 与 EKF **不冗余**：EKF 里是估计值，`rangeM` 是最近一次通过门限的**原始量测**（EKF 不保留量测）。
- **该做的**：合并成 `ObservationFrame`（`baro/range/flow/gnss` + `xxxNew` + stamp + `flowFlags`），每圈填一次，估计器与日志都只读它 → `rangeObsValid`（死代码）直接消失，"喂进 EKF 的观测"与"写进日志的观测"构造上一致。

### 7.3 其他

**本轮已做**：
- 命名：DL_LIB 维持 `k` 前缀；**新写的 app 层**按新约定（配置项 `lowerCamel` 不带 `k`，类型/枚举值 `PascalCase`，函数与成员 `lowerCamel`）。
  健康位从 `dlx::kHealth*` 改成 `app::health::*`。
- 删掉的死代码：`diffHealth`/`HealthDelta`、`ServiceKind`、19 行不可达迁移表、`TaskId::Calibrate`、
  `lastRejectStatus()`/`lastCommandOp()`/`lastCommandArg()`（零调用者）、`taskProgress`（恒 0）、
  `en`/`starting`/`motorStart`/`stop`/`hardStop` 影子量、`stateSnap`、`appTasks` 外的 5 个请求位。
- **`kTaskMachineWired` 开关删了**：它是 `constexpr true`，而且现在准入只有一条路（`admitTask`），
  留着只会在"回 Ok 而飞机不动"这个老问题上多开一个口子。要回退就改 `commandIsUnwired()` 的归类。

### 7.4 待接线项：`TAKEOFF` 的目标高度被静默丢弃（本轮仍维持丢弃）

协议里 `TAKEOFF.arg` = 目标高度 [dm]，链路层专门校验了上界（`lim.maxTakeoffDm = 100u`，即 10 m），
但链路层只把它交给 `taskOfCommand()` 判"有没有这条命令"、**从不保存 arg**，操作机固定爬升到
`cfg::control::kTakeoffHeightM = 1.2f`。即遥控器的"起飞高度"字段**今天被忽略**。

接线前置条件：

- 必须先确认**遥控器侧实际发的是什么**（飞控一直忽略它，很可能发 0；直接接线会把起飞目标变成 0 ⇒ 当场破功能）；
- 按 D6（无载荷），接线只能做成"**几个固定档位各占一位**"（例如 1/2/3/5 m），否则就要破例加载荷字节；
- 无论哪种，都要在 `checkTaskPreconditions` 增加上下限校验（今天只有链路层的上界）。

---

## 8. 实施时待定 → 本轮的决定

| # | 事项 | 本轮决定 |
|---|---|---|
| 1 | 从 `Maintenance` 解锁后 DISARM 回到哪 | **加 `armOrigin`**（Arm 那一拍记来源，1 字节）→ 回到来源态。与重构前逐字一致（那时 lifecycle 在解锁/上锁时不动），台架试车后不必重进维护态 |
| 2 | `ExitMaint` 迁移今天没有触发者 | **保留那一行** + `app_state.cpp` 里注释"无触发者"。运行期退出维护态今天是靠 `release.cpp` 重新 `runFlightStack`，本次不动 |
| 3 | `Maintenance` 下的 `ServiceKind` | **删**。`LogManage` 不是 FlightStack 状态（`release.cpp` 分发层拦下），`Calibrate` 未接线，只剩 `SerialDebug` 语义 —— 那个语义现在就等于"`guards.maintenance == true`" |
| 4 | `stateSnap` 删还是保留 | **删**，改 `snapshot()`（存储搬进 `app_state.cpp`） |
| 5 | 模式位碰撞时的语义 | 按保守序取一位 + 抛异常。**上板日志里要能看出这条**（异常码 `STATE_COMMAND_REJECTED` + `pickTask().pending > 1`） |
| 6 | **新**：`Maintenance` 下是否豁免 IMU 类故障（原 §3.4 提到） | **不豁免**（保持行为不变）。那是产品策略不是重构，要单独决策 —— 豁免会把"台架例外"变成"IMU 坏了也能转桨" |
| 7 | **新**：`app_bringup.cpp` 未登记进 `app_exception_handler.cpp` 一类构建缺口 | 见[工作记录 §8](2026-10-11_flight_stack_refactor_notes.md)：`app_exception_handler.cpp` 不在 `.eide/eide.yml` 里，需要补 |

---

## 附录：证据索引（2026-10-11 行号）

| 事实 | 位置 |
|---|---|
| 4 维 `SystemState` / 19 行迁移（5 行可达） / `kLifeNeedsDisarmed` | `DL_LIB/dlx_system_state.hpp:215 / 256-276 / 236` |
| `diffHealth`、`ArmingEvent::ArmRequest`（零调用者/不可达） | `dlx_system_state.hpp:560-575 / 322` |
| `ServiceKind` | `dlx_system_state.hpp:78` |
| 命令准入表 / `admitCommand` | `dlx_system_state.hpp:602-638 / 656` |
| `app_state.h` 死代码（无 include 者） | `app_state.h` / `app_state.cpp` |
| `st.health` 副本 | `app_flight_stack.hpp:1296` |
| 影子量 `en/starting/motorStart/stop/hardStop` | `app_flight_stack.hpp:209-231` |
| 停机副作用靠影子 bool 每圈判 | `app_flight_stack.hpp:1398-1430` |
| `rangeObsValid` 只写不读 | `app_flight_stack.hpp:165 / 1130 / 1188` |
| `linkVer/linkType/linkAttempts/linkKeyWritten` | `app_flight_stack.hpp:152-158`（使用点 412/454/460/467/626/718/729） |
| 链路丢失的当前位置 | `app_flight_stack.hpp:1327-1352` |
| `kCheckAll` 含 `kCheckCommand` / `checkCommand` 调 `admitCommand` | `DL_LIB/link/dlx_link_sched.hpp:906-907 / 1301-1340` |
| 状态被拒 ⇒ 静默丢弃（无 ACK） | `app_link_thread.hpp:266-270` |
| 三个零调用者（`lastRejectStatus` 等） | `DL_LIB/link/dlx_link_sched.hpp:1106-1110` |
| 唯一活载荷：`SetMode` 的模式值 | `app_link_thread.hpp:365` |
| `TAKEOFF.arg` 上界校验（10 m）/ 被丢弃 | `dlx_link_sched.hpp:1323-1326`；`app_flight_stack.hpp:861`；`app_link_thread.hpp:368-395` |
| 起飞高度固定 1.2 m | `flight_task.hpp:165-177`；`flight_config_gen.hpp:1518` |
| 下行已是"外面填 raw" | `app_link_thread.hpp:153`；`dlx_link_downlink.hpp:113/131-134` |
| LogManage 不进 FlightStack | `release.cpp:145`；切模式 `release.cpp:161-186` |
| 控制环优先级 10，链路 24 / 采集 22 / 日志 26 | `rt-thread/src/components.c:28`；`app_flight_stack.hpp:877 / 890 / 901` |
| `RT_USING_EVENT` 未开启 | `rt-thread/bsp/rtconfig.h`；`rt-thread/src/ipc.c:1229-1735` |
| 启动/停机调用全部返回 void | `DL_LIB/dlx_timer.hpp:111/322/333`；`DL_LIB/dlx_dshot.hpp:165/173` |
| `linkTimeoutMs == hoverLandMs == 2000` / `minValidUplink = 8` | `flight_config_gen.hpp:892 / 882 / 826` |
| Keil include 路径含 `.` 与 `DL_LIB` | `dl_fly.uvprojx` 的 `VariousControls/IncludePath` |
