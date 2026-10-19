# 2026-10-13 状态层裁剪（第一轮：只删不加）

## 2026-10-16 后续修正

- 已删除无产生者的 `FailsafeLevel` / `FailsafeReason`、控制环与 `Snapshot` 中的对应字段，以及无调用者的 `hoverLandDemanded()` / `checkTaskPreconditions()`。
- 同步删除不可达的失效等级判定与 `failsafeLatch`；链路的失效标志、`DL_STATE.failsafeLvl` / `failsafeRsn` 仍保留原位宽并填零，不改变空口布局。此前已裁掉的失效保护策略不在本次恢复。
- `FlightShared` 只持有任务队列对象，通过构造参数接入外部 `ByteBuffer`。16 项底层存储与 `ByteBuffer` 移至 `FlightStack`，先于共享区构造；`FlightStack` 在主线程栈上创建，控制循环不返回，存储覆盖队列及各线程的整个使用期。
- `SystemState` / `Task` 继续承担状态与请求机制；仍被控制、操作状态机和协议使用的 `Mode` / `Operation` / `GroundStatus` 等类型保留。

### 快照存储与协议归属

- `Snapshot` 仍是业务状态值，不再提供全局 `publishSnapshot()` / `snapshot()`。`FlightStack` 在主线程栈上持有 `LockedCell<app::Snapshot>`，控制环通过 `store()` 发布。
- `LinkThreadCtx` 接收 `const LockedCell<app::Snapshot>*`，在线程启动前接线，读取使用 `load()`。日志线程没有直接读取状态快照，不传入无用依赖。快照与控制循环、上下文同寿命，不存在指向短期局部变量的悬空引用。
- 下行仍在同一外层临界区读取遥测与状态快照，`LockedCell::load()` 的嵌套锁按 PRIMASK 保存/恢复，不会提前放开外层中断屏蔽。
- `WireState` / `deriveWire()` 移至 `app_link_state.hpp`，系统状态、模式和操作均显式映射线上值。`GroundStatus` 移至 `app_link_cmd.hpp`，上行模式字节也在该文件显式映射为 `Task`，不再借用业务枚举的数值范围。
- `app_state.h` 只保留业务词汇、纯判定与 `Snapshot` 类型；空掉的 `app_state.cpp` 删除，并移出 EIDE 工程文件列表。生成的 `compile_commands.json` 需由 EIDE 下次构建刷新。
- 本次迁移不改变当前协议编码，但协议不是不可更改的约束；未来变更应集中在 Link 适配与编解码文件，并同步对端。

以下内容保留为第一轮裁剪的历史记录，不能视为当前符号清单。

> **状态**：已实施。
> **边界（用户 2026-10-13 指定）**：**只删，不加**；状态切换**先不迁入控制环**；
> 删出来的连锁问题**如实保留**，不写替代实现。
> **动机**：`Guards` / `Step` / 三张规则表 / `ArmStickLimits` 这些结构，存在的理由是
> "保证状态转移不出错"—— 而 `state` 的实际写者**从头到尾只有控制环**（`startPhase` 与
> `stepStateMachine`，同一线程）。既然只有一个写者，这套防御可以去掉，切换将来直接长在控制环里。

---

## 1. 删了什么

### 1.1 结构体（用户点名的六个）

| 结构 | 原位置 | 原职责 |
|---|---|---|
| `Guards` | `app_state.h` | 状态机的 14 项输入（健康位/杆量/链路稳/驻留时长/失效保护等级…） |
| `Step` | `app_state.h` | 一步的结果 `{next, effects, stateChanged, matched}` |
| `StateRule` + `stateRules[24]` + `findRule` + `ruleFlagsOk` + 三个 flags 位 | `app_state.cpp` | 迁移表（24 行，其中 7 行不可达/无触发者） |
| `ModeRule` + `modeRules[4]` + `modeStateAllowed` | `app_state.cpp` | 模式准入表 |
| `FailsafeRule` + `failsafeRules[8]` | `app_state.cpp` | 失效保护规则表（顺序即优先级） |
| `ArmStickLimits` + `armStickLimits()` + `applyStickFacts()` | `app_guards.hpp` | 解锁前置的两个**量**判据（阈值 → counts） |

### 1.2 随之失去载体的函数（它们的全部意义就是消费上面那些结构）

`stepState` · `admitTask` · `taskAdmitted` · `checkArmPreconditions` · `internalEvent` ·
`cutDemanded` · `failsafeAction` + `FailsafeAction` · `evaluateFailsafe` ·
`failsafeConditionsFromHealth` + `fsCond*` 八个条件位 · `modeAdmitted` · `modeSwitchAllowed` ·
`degradeMode` · `linkGuards`（链路侧 Guards 构造）· `makeGuards`（控制环侧）· `startPhase`。

控制环 `stepStateMachine()` 里被删掉的四段：① 失效保护评估与动作分层、
② 挑位 + 碰撞检测、③ 准入 → 提交、④ 自动降级。`run()` 里 `startPhase()` 的三次调用
（上电三段）一并删除。

### 1.3 没有读者的死符号（与压扁无关，本来就能删）

`motorArmDelayMs` · `minValidUplink` · `hoverLandMs`（三个常量在删除后**零读者**，
且是 `cfg::` 同值常量的第二份真相）· `isStartup` · `linkTimeoutMs` ·
`health::logOk`（**连写者都没有**）· `health::heightAny` ·
`LinkState` + `linkStateOf` + `linkStateName` · `wireOpOf` · `commandIsUnwired` ·
`FlightStack::rangeObsValid` / `velHoldActive` / `armOrigin` / `stateEnteredMs` /
`sh.rt.lastArmStatus` · `kFlowFlagFlowValid` ·
八张名字表（`stateName` / `taskName` / `modeName` / `operationName` / `failsafeLevelName` /
`failsafeReasonName` / `groundStatusName` / `linkStateName`）。

### 1.4 文件级改动

- **删除文件**：`app_guards.hpp`（4 个符号全部是本次删除项，无残留引用）。
- `app_state.h` 620 → 约 400 行；`app_state.cpp` 622 → 约 150 行（只剩请求通道 + 快照 + `deriveWire`）。
- `test/state_step_test.cpp` 836 → 约 330 行（用例随被测对象删）。

### 1.5 保留没动的

`State` / `Task` / `Mode` / `Operation` / `FailsafeLevel` / `FailsafeReason` / `GroundStatus` /
`CommandOp` 这套**词汇**；请求通道（`postTask` / `takeTasks` / `estopLatched` / `clearEstop` /
`TaskPick` / `pickTask`）；`Snapshot` + `publishSnapshot` / `snapshot`；`WireState` + `deriveWire`；
`effect*` 四个效果位与 `FlightStack::applyEffects()` / `applyInPlace()`；`app_task.hpp` 整个
（操作状态机，纯逻辑，未受影响）。

---

## 2. 连锁问题（**刻意保留**，本轮不做替代实现）

| # | 现象 | 位置 | 严重度 |
|---|---|---|---|
| ① | **`state` 恒为 `Boot`**：上电三段没有迁移载体，`run()` 里那三次 `startPhase` 已删 ⇒ 永不进入 `Standby`/`Maintenance` ⇒ 永不 `Armed`、电机永不使能、`DL_STATE.lifecycle` 恒 0 | `app_flight_stack.hpp::run()` | ★★★ |
| ② | **`taskWord` 有人置、没人取**：`takeTasks()` 仍被 `stepSampleShared()` 调用，取出来即丢 ⇒ 链路的 ARM / TAKEOFF / 模式请求不产生任何动作 | `stepSampleShared()` | ★★★ |
| ③ | **DL_ACK 不再承载裁决**：`admitTask` + `linkGuards` 删除后，回执只剩 `Ok`(认得并已发出) 与 `BadCommand`(不认得/没接线)；`RejectedState(6)`/`RejectedArming(7)`/`RejectedHealth(8)`/`RejectedSafety(9)`/`Busy(5)` **没有产生者**。"回 Ok 而飞机不动 ⇒ 操作手去查电机/电调/电池"就是 M1 那条教训 | `app_link_thread.hpp` UpCmd 分支 | ★★★ |
| ④ | **失效保护不再评估**：`fsLevel/fsReason` 恒 `None` ⇒ `failsafeLatch` 永不置位；`hoverLandActive` 恒 false ⇒ `stepAltitudeTarget()` 收到的 `pctx.hoverLand` 恒 false ⇒ **自动缓降消失** | `stepStateMachine()` | ★★★ |
| ⑤ | **`Cut` 没有产生者** ⇒ `Arming` 期间链路丢失**不再断动力**（原判据：`canOutput(from) && (fsLevel==Cut \|\| linkTimedOut && from==Arming)`） | 已随 `cutDemanded`/`internalEvent` 删除 | ★★★ |
| ⑥ | **`applyEffects()` 不再被调用** ⇒ 时基 / MOE / DShot 永不启动（函数本身保留，语义未变） | `app_flight_stack.hpp` | ★★★ |
| ⑦ | **ESTOP 锁存的唯一清除路径消失**：`clearEstop()` 原来的唯一调用点是"一次走通完整前置的 ARM"那一拍，现在没有调用者 ⇒ 锁存一旦置上就再也清不掉 | `stepStateMachine()` | ★★ |
| ⑧ | **模式不再受准入/自动降级保护**：`mode` 可被写成任何档；`degradeMode()` 消失后，`stepControl` 里的"光流不可用 ⇒ 退回角度环 + 高度环"成了**唯一**一道回退 | `app_state.h` `Mode` 注释 / `stepControl()` | ★★ |
| ⑨ | **`health::flow` / `gnss` / `power` 只剩写者**（读者随 `modeRules` / `failsafeConditionsFromHealth` 消失），`gnss`/`power` 本来就是只写不读（低压分级保护未接线） | `app_sensors.hpp` 写 | ★ |
| ⑩ | **`FailsafeReason` 整体无产生者**（含保留下来的 `LogFull` 数值），`GroundStatus` 六个拒绝码无产生者 | `app_state.h` | ★ |
| ⑪ | **主机回归缩水**：`state_step_test.cpp` 从 **786 项**（迁移表逐行 / 非法迁移 / ESTOP 锁存三规则 / 准入表 / 降级链 / 杆量判据 / 链路状态）缩到覆盖剩余部分；被删的那一半**没有替代验证**。而文档记载「上板回归未做」⇒ 状态机目前**没有任何自动化验证** | `test/state_step_test.cpp` | ★★★ |
| ⑫ | **`checkTaskPreconditions()` / `hoverLandDemanded()` 变成零调用者**（纯函数保留，未删） | `app_task.hpp` | ★ |

### 已确认的历史洞（本轮**没动**，别误当成新引入的）

- `stepPilotThrottle()` 的 `modePrev` 顺序问题（[notes §13.5](2026-10-11_flight_stack_refactor_notes.md)）：
  `modePrev = mode` 写在 `stepAltitudeTarget()` 末尾，紧接着 `stepPilotThrottle()` 读它 ⇒
  `fellToStab` 恒 false ⇒ "从定高/定速降级回自稳时先把油门钉在悬停油门"这条兜底**从未生效**。
  本轮**只删不加**，一行未动 —— 它与"恢复切换"属于同一批要一起处理的安全项。
- `app_task.hpp::stepOperation()` 里的 `hoverLand` 分支现在收不到 `true`（见 ④）。

---

## 3. 恢复顺序（下一轮的建议，不在本轮范围）

1. **先修 `modePrev` 顺序**（安全问题，最后 10cm 与降级掉高都在这条路径上）。
2. 决定 **DL_ACK 由谁回答**：要么把准入收进一个新的、宿主可测的纯函数（控制环与链路共用一份），
   要么明确放弃裁决、只回"收到"，让地面端看 `DL_STATE` 状态反馈。**不要**在链路层重写一份判据。
3. 恢复 `state` 迁移时，**把判据直接写在控制环里**（本轮的决定），不要再造 `Guards` 结构；
   迁移表那 7 行死行（6 行上电幂等自环 + `ExitMaint`）也随之不要恢复。
   ⚠ 但"DISARM/ESTOP 在任何状态都有效"这条不变量要重新落实 —— 它原来正是靠那 6 行自环成立的
   （见 [notes §1.2](2026-10-11_flight_stack_refactor_notes.md)：**散文里的不变量，表里少一行就不成立**）。
4. 三个时限常量恢复时**直接取 `cfg::`**（`cfg::control::kMotorArmDelayMs` /
   `cfg::link::arm::kMinValidUplink` / `cfg::link::failsafe::kHoverLandMs`），不要再在 app 层复制。
5. 恢复 `clearEstop()` 的调用点（⑦）与 `Cut` 的产生者（⑤）—— 这两条是安全出口，优先级高于功能。

---

## 4. 验证（本轮实际跑的）

| 检查 | 命令 | 结果 |
|---|---|---|
| 主机回归（裁剪后） | `g++ -std=c++11 -Wall -Wextra -I test/stub -I DL_LIB -I . test/state_step_test.cpp app_state.cpp app_health.cpp` | **456 项检查 / 0 失败**，`-Wall -Wextra` **零告警**（裁剪前 786 项；含 §5 的 health 改动后新增位序回归 21 项） |
| 真编译器逐 TU | `tools/build_check.ps1` | **BUILD CHECK PASS (36/36)**（含 `release.cpp` ⇒ 完整类型检查 `app_flight_stack.hpp` / `app_link_thread.hpp` / `app_sensors.hpp` / `app_task.hpp` / `app_link_cmd.hpp`） |
| 真编译器 + 真 armlink | `tools/build_check.ps1 -Link` | **78/78 + `LINK CHECK PASS`** |
| 残留引用普查 | grep 全部删除符号（`Guards`/`Step`/`stateRules`/`stepState`/`admitTask`/`fsCond*`/`health::logOk`/`linkIsSteady` …） | 源码中**零活引用**，只剩说明性注释 |

---

## 5. 追加（同一天，用户改动）：健康项改为「一项一个 bool + `enum class`」

`app_state.h` 里 `app::health::*` 七个常量删除；存储改成 **bool 数组**，用
`enum class SensorHealth` 取具名索引（`app_health.hpp` / `app_health.cpp`），四个访问器改为
收 `SensorHealth`：

| 旧写法 | 新写法 |
|---|---|
| `healthSet(app::health::baro)` | `healthSet(SensorHealth::BARO)` |
| `healthClear(app::health::link)` | `healthClear(SensorHealth::LINK)` |
| `healthOk(app::health::imu)` | `healthOk(SensorHealth::IMU)` |
| `healthBits()`（读一个打包字节） | **不变** —— 仍是上报用的打包（bit N = 第 N 项） |

- **位序 = 枚举顺序**（bit0=LINK … bit6=POWER）。往中间插一项 = 改空口
  （`Snapshot::health` / `DL_SYS.health`）；`test/state_step_test.cpp` 的 `testHealth()`
  用 0x01→0x7F 的逐项累加把这条钉住。
- `TOTAL_SIZE`（原 `TOTOL_SIZE` 拼写已修正）**≤ 8** 由 `app_health.cpp` 的 `static_assert` 保证
  —— 它要装进一个字节。
- 顺带解掉一个真实风险：位图是"读-改-写同一份共享字节"（不同位也会互相丢更新，所以必须上临界区）；
  现在每个数据源写**自己那一个字节**。临界区保留，作用是让 `healthBits()` 的整体读拿到一致快照。
- `app_health.hpp` 不再 include `app_state.h`（枚举搬过来之后它成了只依赖 `<stdint.h>` 的叶子头）。
- 调用点：`app_sensors.hpp`(10) / `app_flight_stack.hpp`(6) / `app_link_thread.hpp`(2)，
  以及注释里的引用（`app_sensors.hpp` / `app_task.hpp` / `dlx_link_downlink.hpp`）。
