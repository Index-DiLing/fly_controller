# 2026-10-11 flight_stack 状态机重构 —— 工作记录 / 经验与注意事项

> 配套文档：[实施计划](2026-10-11_flight_stack_state_machine_refactor.md)（规格与决策表）
> 本文只记"做的时候才知道的事"：方案里写错的地方、踩到的坑、以及下次别再犯的规矩。
> 代码注释里只留结论，来龙去脉在这里。

---

## 1. 落地方案里三个"写着对、做起来会坏"的洞

这三处都是**在写代码/写测试的过程中**才暴露的，而且两处会让安全动作失效、一处会让合法操作被拒。

### 1.1 ESTOP"跳过消费位"会让单按急停什么都不发生

- **方案原文（§3.2）**：ESTOP 是字里的一位，控制环"只跳过它、不算进待清除位"，唯一清除路径是一次走通完整前置的 ARM。
- **坏在哪**：把 ESTOP 排除在"这一圈要执行的请求"之外以后，**单按一次急停就只剩一个锁存、没有任何停机动作**。
  今天能停，是因为链路线程同时置了 `disarmRequest` —— 也就是说，方案照抄了今天的结果，却删掉了产生它的那个机制。
  更糟的是若按"今天就行的写法"让链路同时置 `Disarm | EStop` 两位，位图会把它判成**碰撞**并抛异常。
- **修法**：把"**位**"与"**锁存**"拆开。
  - 位 = 这一拍要执行的动作：`postTask(EStop)` 置位 → `takeTasks()` **照常取走** → 这一拍就是 `Task::EStop` → 停机；
  - 锁存 = 守卫：`postTask(EStop)` 同时把 `estopLatch` 置上，它**不**随位被消费而消失，只有走通完整前置的 ARM 才清。
  - 附带好处：ESTOP 之后再按 ARM，字里只有 ARM 一位，不会和锁存撞成"碰撞"。
- **教训**：凡是"读出来但不消费"的字段，先问一句"那它的动作由谁产生"。

### 1.2 `DISARM` 在启动三段被拒（两张表自相矛盾）

- **方案 §3.8** 说"DISARM / ESTOP 任何状态永远有效 —— 人上锁是最后出口"。
- **方案 §3.3** 的迁移表里却没有 `Boot/InitLow/InitHigh` 那三行，而 §8 又把"状态级准入"定义成"表里有没有这一行"。
  于是 `taskAdmitted(Boot, Disarm)` 为假 ⇒ 回 `RejectedState` ⇒ **"永远有效"当场不成立**。
- **修法**：三个上电状态各补 `Disarm` / `EStop` 两行幂等自环（共 6 行），效果位用 `None`
  —— 那三个状态硬件根本没起来，不需要也不该去碰定时器。
- **教训**：写"永远有效"这种话时，必须回到数据表里数行数。散文里的不变量，表里少一行就不成立。

### 1.3 "链路丢了该做什么"不能暴露成两个谓词让调用方拼

- **方案 §3.2 / §3.8** 分别给了 `cutDemanded()` 与 `hoverLandDemanded()` 两句话，各自都对。
- **坏在哪**：`Arming` 的失效保护**等级**同样是 `HoverLand`，但动作必须是 `Cut`（地面上"停"永远比"降"安全，
  而且那一刻姿态环还没接管，没有"降"这回事）。调用方自己拼两个谓词，迟早拼出"ARMING 期间开始缓降"。
  测试里我第一版就是照方案拼的，断言 `!hoverLandDemanded(...)` 直接红了 —— 那一红就是证据。
- **修法**：收敛成一个判定点 `app::failsafeAction(state, guards) -> {None, HoverLand, Cut}`，分层条件只在那里一份。
- **教训**：两个各自正确的谓词，组合起来不一定正确。**分层条件必须住在同一个函数里。**

---

## 2. `Operation` 不能把 `HoverLand` 折进去（两个正交的轴）

方案 §3.1 把 `Operation` 写成 `{None, Takeoff, Land, HoverLand}`，理由是"飞机在干什么应该只有一个答案"。
**这条是错的**，落地时会引入真 bug：

- `Land` 触地之后 `stage == landStageTouchdown`，飞机在软地板上等人上锁。
- 此时链路超时 ⇒ 失效保护要求"缓降"。
- 若把缓降折进 `Operation`，走"换操作就复位阶段"的自然写法 ⇒ `stage` 被复位成"下降"
  ⇒ **已经落地的飞机重新开始下降**。
- 原实现正是靠"`in.operation == Land` 时不归一化阶段"挡住这一支的；折进去之后这个判据就没有了。

正确的划分是**两个轴**：

| 轴 | 回答 | 载体 | 谁写 |
|---|---|---|---|
| `Operation` | **做什么**(Takeoff / Land / None) | 提交的状态, 也上线(`DL_STATE.task`) | 准入通过后提交 |
| `hoverLand` | **为什么**(选哪个下降率、拒哪些命令) | 每圈推导的输入 | 失效保护 |

顺带一条：`Operation` 的线上值 3 原本是 `Calibrate`(从未接线)。折进 `HoverLand` 会让遥控器把"缓降"读成"标定"。
**保留旧编号 0/1/2，线上零风险。**

---

## 3. 影子量：为什么必须删干净

原来 `en` / `starting` / `motorStart` / `stop` / `hardStop` 五个成员是**上一圈状态的镜像**：
控制环每圈把 `Armed`/`Arming` 写进共享区(`sh.rt.enabledMotor`)，下一圈再读回成员变量。

- 它让"为什么电机在转"有两个答案（`state` 与 `en`），而两者只是一次回环的延迟关系；
- 它让副作用变成"每圈判一次"（`if (state == Armed && !en) { en = true; ... }`），
  而不是"迁移那一拍执行一次"。

现在：`en`/`starting` 派生自 `state`（`isTakeover()` / `state == Arming`），
`stop`/`hardStop` 是**效果位**（`effectStopPower`）与 **ESTOP 锁存**。

**判断标准**：如果一个量能由别的量算出来，它就不该是成员；如果它是"某一拍的一次性动作"，它就该是效果位而不是标志。

---

## 4. 只写不读的字段："加字段前先找到读者"

这次顺手删掉的：

| 字段 | 真相 |
|---|---|
| `taskProgress` | 只在赋值处写 `0`，**从来没有第二个写者、也没有读者** —— 恒 0 |
| `rangeObsValid` | 只写不读（方案 §7.2 记录） |
| `diffHealth` / `HealthDelta` | 零调用者 |
| `lastRejectStatus()` / `lastCommandOp()` / `lastCommandArg()` | 零调用者 —— 而且注释说"按它回 DL_ACK"，**从未实现**（见 §5） |

于是 `Snapshot` 里我也没有放 `stage` / `progress`：链路侧不需要它们，日志侧没有读者。
**加字段的时候先写下读者是谁；写不出读者就先别加。**

---

## 5. DL_ACK 的依据必须与控制环同源 —— 连阈值一起

`DL_ACK` 是给**操作手**看的。回 `Ok` 而飞机不动，他会去查电机、查电调、查电池（M1 那条教训）。

这次发现的是一个**实锤 bug**：`dlx_link_sched.hpp` 的采信前置里混进了"命令现在能不能执行"
（`admitCommand(m.op, true, *st)`），于是带锁按 SET_MODE、未解锁按 TAKEOFF 这类命令被判 `StateNotAllowed`，
调用方 `++linkAdmitRejects; return;` —— **一个 ACK 都不回**。而注释说的
"按 `lastRejectStatus()` 回拒绝码"从未实现（那三个 getter 零调用者）。遥测现象是"遥控器上什么都没有"。

修法（方案甲）之后，"准入"只剩 app 层一处，而那一处本来就会回 ACK。但**光搬位置还不够**：

- 解锁真正的前置是"链路稳 + 油门低位 + 杆量中位 + IMU 正常 + 上电门槛"；
- 其中有两条是**量**的判断，阈值来自配置（`kThrottleLowPct` × `kThrMax` 之类）；
- 如果链路层按自己的写法算一遍、控制环再算一遍，两边一定会漂。

所以阈值、比较、以及"链路稳不稳"的判据全部收到了 `app_guards.hpp` 一处，
两个调用点各自只填**自己拥有的**那几项（控制环填全部；链路层填它手里的杆量/连续帧数/快照高度）。

**规矩**：一个判据要么只写一处，要么把它拆成"纯数据 + 一个函数"。两边各写一份"看起来一样"的表达式不算一处。

---

## 6. 单核上的互斥：关中断，不是自旋锁，也不是 `rt_enter_critical()`

| 手段 | 单核上为什么不行 / 行 |
|---|---|
| 自旋锁 | 是 **SMP** 原语。持有者可能是优先级更低的线程（链路 24 / 日志 26），自旋者是控制环（10）⇒ **必然死锁**，不是"概率问题" |
| `rt_enter_critical()` | 只挡**调度**、不挡中断（返回时中断已经开了）；互斥范围是"线程 vs 线程" |
| `dlx::InterruptLock` | PRIMASK 保存/恢复 ⇒ 可嵌套、不误开外层屏蔽。**本工程共享状态一律用它** |

用法上的一致性也很重要：**共享量的读写协议必须封装在访问器里**，不要暴露"裸变量 + 调用方自己加锁"。
`postTask` / `takeTasks` / `estopLatched` / `publishSnapshot` / `snapshot` / `healthSet` 都是这个形状。

顺带纠正一个写法：头文件里
```cpp
inline volatile uint8_t &appHealth() { static volatile uint8_t bits = 0u; return bits; }
```
这种"返回函数内静态变量的引用"把存储和接口混在一起，每个使用者都成了"这个静态量到底定义在哪"的猜测者。
按规矩改成：**存储在 .cpp 的匿名命名空间里，头文件只暴露 setter/getter**（`app_health.hpp` + `app_health.cpp`）。

---

## 7. 主机测试要能编过，决定了文件怎么切

`app_state.cpp` 需要 `dlx::InterruptLock` ⇒ `#include "stm32f4xx.h"`。而主机测试（方案 §6.1：一条 `g++` 命令）
不能带真 CMSIS 头。仓库里**已经有** `test/stub/stm32f4xx.h`（三个原语的单线程替身），
所以只要 `-I test/stub` 放在最前面就能编。

因此文件切分是**按依赖**而不是按"概念大小"：

| 文件 | 依赖 | 能被主机编译 |
|---|---|---|
| `app_state.h` | 只有 `<stdint.h>`（叶子头） | ✅ |
| `app_state.cpp` | + `dlx_interrupt_lock.hpp`（只为了那两个访问器） | ✅（配 stub） |
| `app_task.hpp` / `app_link_cmd.hpp` / `app_guards.hpp` | + `flight_config.hpp` / 帧头 | ✅ |
| `app_health.cpp` | + `dlx_interrupt_lock.hpp` | ✅（配 stub） |

**规矩**：新建的纯逻辑文件不要顺手 include 硬件头；一个 `#include` 就能决定它还能不能被主机测试覆盖。

---

## 8. 构建真相：EIDE + AC6，语言标准是 **C++11**

`.eide/eide.yml` 里 `language-cpp: c++11`。以下几件事在 C++11 里**不成立**，别写：

- `constexpr` 函数里**不能有循环、不能有局部变量**（只能一条 `return`）⇒ 查表的函数写成 `inline` 而不是 `constexpr`；
- 没有 `inline` 变量（C++17）⇒ 头文件里不能定义全局量，必须有 .cpp；
- 带默认成员初始化的结构体**不是聚合**（C++11 的聚合初始化限制）⇒ 数据表的行结构体不要写 NSDMI，
  否则 `{State::Boot, Task::PowerOn, ...}` 这种初始化编不过。

另外三条工程事实：

1. **新 .cpp 必须登记进 `.eide/eide.yml` 的 USER 文件列表**，否则 EIDE 不会编它。
   这次加了 `app_state.cpp` / `app_health.cpp`。
2. 仓库根目录的 `dl_fly.uvprojx`（Keil 工程）**已经过期**：它引用一个不存在的 `main.cpp`，
   也没有列任何 app 层 .cpp。构建以 `.eide/eide.yml` 为准。
3. **`app_exception_handler.cpp` 不在 `.eide/eide.yml` 里** ⇒ 它里面的 `DLX_HANDLE_EXCEPTION`
   等函数（还是在 .cpp 里写的 `inline`）不会被编进镜像 ⇒ 链接期 undefined reference。
   （对照：`release_header.cpp` **在**列表里，所以那几个全局句柄没问题。）
   要么补进构建，要么把定义搬回头文件。

### 验证方法（不依赖 Keil IDE）

`tools/build_check.ps1` 把 EIDE 生成的 `build/FLY/compile_commands.json` **原样重放**：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\build_check.ps1              # 工程源文件
powershell -ExecutionPolicy Bypass -File .\tools\build_check.ps1 -Filter release
powershell -ExecutionPolicy Bypass -File .\tools\build_check.ps1 -Link        # + 真 armlink(查重复/未定义符号)
```

`-Link` 那条不是可选项：**编译通过看不见重复符号/未定义符号**（`HardFault_Handler` 那次就是这么漏的）。

主机测试：

```
g++ -std=c++11 -Wall -Wextra -I test/stub -I DL_LIB -I . \
    test/state_step_test.cpp app_state.cpp app_health.cpp -o state_step_test
```

---

## 9. 空口语义变化：`lifecycle` 的 Standby(3) vs Active(4)

今天 `DL_STATE.lifecycle` 是"链路稳但未解锁 ⇒ **Active(4)**"；新模型里 `Standby` 就是"未解锁"，
纯映射一律发 **3(STBY)**。

方案 §3.6 原本建议"wire 层保留旧口径（链路稳也发 4）"，但那要求把"链路稳"塞进 `deriveWire()`。
**本次决定纠正为 3**（用户确认遥控器侧会同步）——于是 `deriveWire(snapshot)` 保持**纯函数**、可主机测。
这条是**与遥控器约定的同步点**，不是可以随手改的兼容层，写在这里以免下次被当成 bug "修回去"。

另外 `Operation` 的线上值 3 历史上是 `Calibrate`（从未接线）。**不要复用 3。**

---

## 10. 验证覆盖到哪儿（以及没覆盖到哪儿）

**已覆盖**：

- 主机测试 `test/state_step_test.cpp`：786 项检查，0 失败。
  24 行迁移表逐行、16 条非法迁移、`Arming` 驻留边界（差 1 ms 不行）、
  ESTOP 锁存三规则（置位即停机 / 锁存期间只放行 Arm+Disarm / 只有完整前置的 ARM 能清）、
  碰撞检测与优先级（含模式组内部序）、链路丢失三态分层、`armOrigin`、准入表与"准入⟹状态级准入"结构不变量、
  降级链、`deriveWire` 逐值、命令映射穷举 0..255、杆量判据边界。
- armclang（AC6，真编译器）编译：`app_state.cpp` / `app_health.cpp` 0 错误；
  `app_flight_stack.hpp` + `app_link_thread.hpp` 经一次性空壳探针完整类型检查 0 错误。
- 全工程 33 个 TU：32 通过，唯一失败的 `release.cpp` 只错在它自己那 3 行（见下）。

**未覆盖（本轮做不到）**：

1. **`release.cpp` 编译不过 —— 与本次重构无关**。它引用了工作树里**根本没有定义**的 3 个符号：
   `DLX_SET_EXCEPTION_TEXT` / `DLX_SET_EXCEPTION_REPORT`（第 133-134 行）、`isOkCode`（第 235 行）；
   `app_flight_stack.hpp` 里还有 4 处：`applyMotorDirect`、`DLX_SET_EXCEPTION_IMMEDIATE` /
   `_LED` / `_PHASE`（含未定义的 `ExceptionPhase`）。这些引用行**本次一行都没动**。
2. **上板回归没做**（方案 §6.2 那一整张清单）：解锁全流程、ARMING 启动、DISARM 停机、
   ESTOP 锁存→重解锁、失效保护缓降、链路恢复交还、Manual 直控、模式切换与自动降级、
   `LogManage→SerialDebug/Flight` 交接。
3. **`Arming` 驻留期间"桨在转但控制未接管"必须与今天逐字一致** —— 这是最该盯的一条，
   它只在目标机上可见（`timer1.start/enableOutputs/dshot.start` 的顺序与时机）。

---

## 11. 本次的"变量分配"决定（一次说清）

| 量 | 放哪 | 为什么 |
|---|---|---|
| 请求位图 | `app_state.cpp` 匿名命名空间 + `postTask`/`takeTasks` | 跨线程 + 必须关中断保护 ⇒ 存储与协议封在一起 |
| ESTOP 锁存 | 同上（独立变量，不是字里的一位） | 见 §1.1 |
| 健康位 | `app_health.cpp` 匿名命名空间 + 四个访问器 | 同上；顺带替掉"返回静态变量的引用"那种写法 |
| 状态快照 | `app_state.cpp` 匿名命名空间 + `publishSnapshot`/`snapshot` | 读写要一致 ⇒ 一个临界区取一份 |
| `state`/`mode`/`operation`/`fsLevel`/`armOrigin`/`stateEnteredMs` | `FlightStack` 成员 | 单写者 = 控制环；写入点必须与副作用在同一上下文 |
| `operation` 的 `opStage` | `FlightStack` 成员 | 只驱动斜坡，其他人不需要 ⇒ 不进快照 |
| `Snapshot` 里的 `stage`/`progress` | **不放** | 没有读者（§4） |

---

## 12. 链接期才会露头的两个坑（编译全绿也不代表能链）

`build_check.ps1` 不加 `-Link` 只看编译，而**编译通过看不见重复符号/未定义符号**。这次两个都是链接期才发现：

### 12.1 在 `.cpp` 里把函数定义成 `inline`，而头文件里是非 inline 声明

现象：`app_exception_handler.cpp`（已进构建目标）里的
```cpp
inline void ENABLE_EXCEPTION_TEXT() { ... }      // .cpp
inline void DLX_HANDLE_EXCEPTION(ExceptionCode code) { ... }
```
而 `app_exception_handler.h` 里是
```cpp
inline void ENABLE_EXCEPTION_TEXT();             // 头文件(注意: 也写着 inline)
void DLX_HANDLE_EXCEPTION(ExceptionCode code);
```
链接期：`L6218E: Undefined symbol ENABLE_EXCEPTION_TEXT() (referred from release.o)`。

三条规则叠在一起才是根因：

1. 声明为 `inline` 的函数，**定义必须在每一个 odr-use 它的翻译单元里可见**；
   这里 `release.cpp` 只看到声明、看不到定义 ⇒ 连内联的机会都没有；
2. 编译器**不保证**为"只在本 TU 定义、且本 TU 没有 out-of-line 调用"的 inline 函数发射外部定义；
3. 声明与定义对 `inline` 的处理不一致属于"无需诊断的 ill-formed"，所以**编译期一声不响**。

**修法**：头文件与 .cpp **两处都去掉 `inline`**（只去 .cpp 那处没用 —— 第一条声明已经把它定成 inline 了）。
本来这个文件的状态住在匿名命名空间里，**必须是单实例**，所以定义就该是非 inline 的。

> 推广：`.cpp` 里定义、头文件里声明的函数，**不要写 `inline`**。`inline` 只用于"每个 TU 都能看到定义"的头文件函数。

### 12.2 新 .cpp 忘了登记进 `.eide/eide.yml`

`srcDirs` 只 glob `DL_LIB` / `rt-thread`；根目录的 app 层文件是靠 USER 分组**逐个列出**的。
漏一个 .cpp 不会报编译错（头文件的 `inline` 函数照样能编），而是**链接期一次性冒出成片的未定义符号**。

这次实测：把 `app_state.cpp` / `app_health.cpp` 从对象列表里去掉再链，报 **20 个** `L6218E`
（`app::stepState/admitTask/postTask/takeTasks/estopLatched/clearEstop/snapshot/publishSnapshot/deriveWire/…`
+ `dlx::healthSet/healthClear/healthOk/healthBits`）。

**规矩**：新建 .cpp 的同一步就把它加进 `.eide/eide.yml` 的 USER 分组，然后**跑一次 `-Link`**。
另外：`tools/build_check.ps1` 读的是 EIDE 生成的 `compile_commands.json`，
所以**加了文件必须先在 EIDE 里构建一次**，那个 JSON 才会包含新目标。

---

## 13. "编译通、接口也对得上，但逻辑不对" 的清查清单

这一类错误编译器一句话都不会说，只能靠"沿数据流走一遍 + 数一数谁写了、谁读了"。下面是这次
实际使用的检查项（每一条都真的抓到了东西，或确认了没问题）。

| 检查 | 怎么查 | 本次结果 |
|---|---|---|
| **每个字段的写者与读者** | 对每个新/改的字段 grep 名字，数调用点 | 抓到 3 处"只写不读"（见 §4 与 §13.3） |
| **每个枚举/位域有没有人来置位** | grep 定义名 + 写侧调用 | 抓到 `health::logOk` **零写者**（§13.1） |
| **同一件事是不是有两份数字** | 对新引入的 constexpr，grep 它的配置对应项 | 抓到 4 对"两处真相"（§13.2） |
| **"取齐"的几项是不是真在同一个临界区里** | 看两次取数之间有没有释放锁 | 抓到 1 处（§13.4） |
| **边沿判据用的"上一圈的值"是不是真的还是上一圈的** | 看赋值点与读取点的先后顺序 | 抓到 1 处**既有**失效保护（§13.5） |
| **准入/状态表的"被拒"路径会不会每帧刷异常** | 对每个 Task 想一遍"操作手连按会怎样" | 抓到"已解锁时按 ARM"（§13.6） |
| **一段代码里的 `if` 顺序是否等价于重构前** | 逐分支对照旧实现 | 抓到 1 处（§13.7） |

### 13.1 `health::logOk` 从来没有写者

`failsafeConditionsFromHealth()` 会读 `health::logOk`，但全仓**没有任何地方 healthSet 它**。
后果：`fsCondLogFull` 恒真 ⇒ `evaluateFailsafe()` 永远至少命中最后一行 ⇒ **遥测里的
`failsafeLvl` 恒 ≥ Warn、`failsafeRsn` 恒 = LogFull(8)** —— 地面端以为"日志一直满"。
（旧代码的 `kHealthLog` 同样没人写，所以这是**既有**缺口，不是重构引入的。）

### 13.2 四对"两处真相"

状态机按"暂时与 setting 脱节"就地 `constexpr`，但别处仍用 `cfg::` 的同一个数 —— 值现在**恰好相同**，
所以编译、测试、上板都不会报，改配置时才会翻脸：

| 状态机用 | 别处仍在用 | 不一致时的现象 |
|---|---|---|
| `app::motorArmDelayMs` | `cfg::control::kMotorArmDelayMs`（`app_boot_report.hpp` 打印） | 报告说"启动序列 3200ms"，实际按另一值进 Armed |
| `app::minValidUplink` | `cfg::link::arm::kMinValidUplink` | — |
| `app::hoverLandMs` | `cfg::link::failsafe::kHoverLandMs` | — |
| `app::linkTimeoutMs` | `cfg::link::failsafe::kLinkTimeoutMs`（`gatherFacts`/`app_log_thread` 判 `linkOk`） | **遥测说"链路 OK"而状态机已经判定超时缓降** |

**建议**：在 `app_guards.hpp`（它同时看得到 `app_state.h` 与 `flight_config.hpp`）加 4 条
`static_assert`，把"脱节但必须相等"变成编译期强制 —— 这既满足"暂时脱节"，又不可能悄悄漂。

### 13.3 只写不读 / 零调用者

- `release_header.h` 的 `lastArmStatus`：控制环写、**全仓零读者**（注释说"给状态行/日志看"，但状态行没打印它）。
- 名字表 `linkStateName` / `modeName` / `operationName` / `failsafeLevelName` / `failsafeReasonName` /
  `groundStatusName`：**零调用者**（`stateName`/`taskName` 只有主机测试在 `printf` 里用）。

### 13.4 "同一时刻取齐"的两项被拆到了两个临界区

`linkFillDownlinkSource()` 原来是"姿态 + 系统状态"在**同一个** `rt_enter_critical()` 里拷贝，
注释明确写着"避免新姿态 + 旧状态"。改成 `snapshot()` 之后变成了两个独立临界区 ⇒ 撕裂又回来了。
修法：外层套一个 `dlx::InterruptLock`（**可嵌套**，内层 `snapshot()` 自己的锁不会把外层打开）。

### 13.5 `modePrev` 在同一拍就被更新 ⇒ 降级油门兜底从没生效（既有）

```cpp
stepAltitudeTarget();   // 函数**末尾**: modePrev = mode;
stepPilotThrottle();    // 紧接着比较 modePrev ⇒ 拿到的其实是**本圈**的 mode
```
于是 `fellToStab = (mode == Stabilize) && (modePrev == AltHold || modePrev == VelHold)` **恒为 false**：
"从定高/定速降级回自稳时先把油门钉在悬停油门"这条兜底**从来没执行过**（那一支里的
`STATE_MODE_DEGRADED` 异常也从不触发）。危险点在**从定高降级**那一瞬间：遥控器在定高档发的油门
可能是 0，直接交还就是当场掉高 —— 这正是这段代码想防的事。

**修法**：把 `modePrev = mode;` 从 `stepAltitudeTarget()` 末尾移到 `stepStateMachine()` 里
`stepPilotThrottle()` **之后**（`stepAltitudeTarget` 的"进入边沿"判据仍然看到上一圈的值）。

### 13.6 已解锁时按 ARM：从"静默忽略"变成了"每帧一条异常"

重构前：控制环 `else if (armReq && st.arming == Disarmed)` ⇒ 已解锁时**整支不进**（不报异常、不改状态），
而链路层的 DL_ACK 回 `RejectedArming(7)`。
重构后：`taskAdmitted(Armed, Arm)` 为假 ⇒ `RejectedState(6)` ⇒ 控制环**每收到一帧 ARM 就抛一次
`STATE_COMMAND_REJECTED`**。操作手连按几次 ARM（很常见）就会刷异常、并改掉 DL_ACK 的理由码。

### 13.7 模式请求被拒的那一拍会**叠加**一次自动降级

`modeRequested` 若只在"准入通过"时才置真，那么**被拒**的模式请求会让同一拍继续跑
`degradeMode()`。重构前是 `modeReq != 0xFF` 就走请求分支，被拒只报异常、不跑降级。修法：
`modeRequested` 在**准入之前**按 `isModeTask(on)` 定下来。

### 13.8 异常出口：不是"开关没开"，是**渠道对象根本不存在**（已修）

`app_flight_stack.hpp` 里那段旧残骸（`DLX_SET_EXCEPTION_IMMEDIATE/_LED/_PHASE` + `ExceptionPhase`，
全仓无定义）已按新 handler 改成一句 `ENABLE_EXCEPTION_DLX(app.protocol)`。

当时发现那个开关**仍然不生效**，原因比"忘记开开关"深一层：

| 环节 | 当时的状态 |
|---|---|
| `LogChannel` 实例 | **全仓一个都没有**（唯一一处 `TextMailbox` 构造在主机测试里） |
| `ReleaseContext::channel` | 聚合初始化只给了 8/9 个值 ⇒ **永远是 `nullptr`**，之后也没有任何赋值点 |
| `logLine()` | `channel == nullptr` 早退 ⇒ **所有文本日志被丢弃**（`main` 里那些 `[Boot]` 检查点、`[Manage]` 提示都看不见） |
| `DLX_EXCEPTION_HANDLER()` | `ctx->channel == nullptr` 早退 ⇒ **异常被丢弃** |
| `ENABLE_EXCEPTION_DLX(pr)` | 零调用者 |
| `ENABLE_EXCEPTION_RADIO/FLASH` | 零调用者，且连 `.h` 都没声明 |

**修法（已落地）**：

1. `main()` 栈上创建 `TextMailbox`(2×512B 池) + `ExceptionMailbox` + `LogChannel`，
   并作为 `ReleaseContext` 聚合初始化的**第 9 个值**传进去。
   拥有者只能是 `main` 的栈或全局：**渠道必须在所有模式下都在**（`LogManage` 也用 `logLine`，
   而它不进 FlightStack）。预算：`RT_MAIN_THREAD_STACK_SIZE = 24576`，新增 ~1.07 KB。
2. `app_exception_handler.h/.cpp` 里的 `inline` **全部去掉**（头 10 处 + .cpp 18 处）。

第 2 条为什么是必须的 —— 实测证据：`fromelf --text -s 47_app_exception_handler.o`
筛 `ENABLE_EXCEPTION|DLX_HANDLE_EXCEPTION` **一个符号都没有**，即那个目标文件**是空的**。
三条规则叠起来才是根因（见 §12.1），而它**编译期一声不响**，只在 armlink 报
`L6218E: Undefined symbol ENABLE_EXCEPTION_TEXT() (referred from release.o)`。

修完的实测结果：**78/78 编译 + `LINK CHECK PASS`**。

**还留着一个**：`ENABLE_EXCEPTION_DLX(DLX_ProtocolBuffer *pr)` 的 `pr` **存而不用** ——
`DLX_EXCEPTION_HANDLER` 只往 `exceptionMailbox` 里 push，根本不看 `protocol`。
要么把它用起来，要么把参数去掉。

### 13.9 接上信箱 ≠ 有人排空：`LogManage` 模式的文本仍然发不出去

`logLine()` **只投递**（`textMailbox->post`），`flushSerial()` 只把 **txBuf** 里的东西发完 ——
它**不排空信箱**。而 `textMailbox` 的消费者全仓**只有 `app_log_thread.hpp`**（FlightStack 的日志线程）。

`release_log_manage.hpp` 的主循环只做 `drainDlx(...)`（解 DLX 帧）+ `flushSerial(txBuf)`，
**从不排空 `textMailbox`** ⇒ 它在 16+ 个地方调的 `logLine("[Manage] ...")` 永远到不了串口：
接线前是"立即丢弃"，接线后是"积在池里，满了按计数丢"。两种都不显示。

（对比：同一文件里的 `replyGround()` 是**直接写 txBuf + flushSerial**，所以应答能到 ——
这也是为什么这个缺口很容易被忽略：**一半的消息能到，另一半静默消失**。）

**修法**：`runLogManage` 的主循环里加一次"排空文本信箱 → 成帧 → 写 txBuf"
（即把 `app_log_thread.hpp` 那段文本批处理复用一次），或者让 LogManage 也走 `LogThreadCtx` 的
`logcfg` 批处理。**在修之前，`main`/`LogManage` 里的 `logLine` 输出都只能靠 `replyGround` 传递。**

### 13.10 声明与定义的签名不一致 = 重载 ⇒ 编译通过、链接才报未定义

实录：`app_exception_handler.cpp` 里把 `ENABLE_EXCEPTION_DLX(DLX_ProtocolBuffer* pr)` 的参数去掉了
（那个 `protocol` 成员谁也不读），但 `app_exception_handler.h` 还声明着带参数的版本。结果：

```
47_app_exception_handler.o 里定义的是  _Z20ENABLE_EXCEPTION_DLXv          (无参)
48_release.o       里引用的是           _Z20ENABLE_EXCEPTION_DLX**P18DLX_ProtocolBuffer**
```

两个**不同的重载**。编译期一声不响（`.cpp` 里那句定义在**重载**意义上是完全合法的），
链接期才报 `L6218E`。用一句话就能自查：

```powershell
fromelf --text -s <对象> | Select-String '符号名'    # 对比 .o 里定义/引用的**mangled 名**是否一致
```

**规矩**：改了函数签名就同时 grep 一遍声明与定义；**签名不一致是"无需诊断"的**，
只有 `-Link` 或者直接从 `.o` 里读 mangled 名才看得见。

---

## 14. UP_MOTOR：由"单路 4B"改成"四路一次发全 8B"（协议破坏性变更）

### 为什么改

改造前的 `UP_MOTOR` 是 `index(3b) + throttle(11b)`（4B 帧），一次只发一路。它有三个后果：

1. **四路不是同一时刻的**：遥控器要连发 4 帧才把四路写全，期间飞控缓存里可能是"新三路 + 旧一路"；
2. **"哪一路刚被寻址"要额外记账**（`FlightRuntimeState::motorCmdIndex`），而这个账本是纯协议副产物；
3. **协议未定义的 index 4..6 会混叠到真实通道上**（`motorCmdValue[index & 0x3]`），于是必须有一条
   专门的拒绝路径（`applyMotorDirect` 的返回值 + `motorDirectRejects` 计数）—— 而那条路径的
   语义只能靠调用点注释反推，谁也不敢改。

一帧就是**完整状态**之后，上面 2 与 3 **一起消失**：没有 index、没有半更新、没有未定义码点。
代价是帧长 4B → 8B（上行 33Hz 下多 132B/s，相对 57600bps 可忽略）。

### 新格式

```
0xD  UP_MOTOR   44b + 4 空位 = 6B 负载 → 8B 帧
    motor[0] u11 @bit 0   | motor[1] u11 @bit 11 | motor[2] u11 @bit 22 | motor[3] u11 @bit 33
    bit 44..47 恒 0(准入门用它挡"不是本工程编出来的帧")
    值 = DShot 计数原样 0..1999(2000..2047 是特殊命令区, 编码时 clampDshot 夹回)
    **饱和而不是回绕**: 回绕会把 2047 变成 0(满油变停车)
```

### 连带删掉的东西（都是"单路模型"的产物）

| 删掉 | 原用途 |
|---|---|
| `upMotor::kIndexBits` / `kThrottleBits` / `kIndexAll` | 单路位表 |
| `UpMotorPayload::index` | "哪一路" |
| `FlightRuntimeState::motorCmdIndex` | 记"最近一次寻址的是哪一路" |
| `FlightRuntimeState::motorDirectRejects` | 4..6 未定义码点的计数（**而且它原本零读者**） |
| `applyMotorDirect(index, counts, motor)` | 唯一用途就是识别 4..6 —— 于是**函数本身不再需要存在**，那个"编译缺口"随协议变更一起消失 |

控制环里"逻辑电机 → 物理通道"改为直接用混控同一张表：

```cpp
for (uint8_t i = 0u; i < link::upMotor::kMotorCount; ++i) {
    motor[cfg::hw::motor::kMap[i]] = counts[i];
}
```
（遥控器发的是"1 号电机"，与混控算出的"1 号电机"必须是同一个物理通道；`kMap = {0,2,3,1}`。）

准入门那条量级检查也**从查一路改成查四路** —— 漏一路就等于放行一路"电机唱歌"。

### 怎么验的

新增 `test/up_motor_frame_test.cpp`（48 项检查，0 失败），跑法：

```
g++ -std=c++11 -Wall -Wextra -I test/stub -I DL_LIB -I . test/up_motor_frame_test.cpp -o up_motor_frame_test
```

它做的是**真编码 → 真解码**的往返，而不是对着常量打勾：

- 四路取**互不相同**的值（0/1/1024/1999）往返 —— 只有这样才能发现"通道串位"；
- **逐路独立性**：每次只给一路 1999、其余 0，验证值确实落在那一路上（位表差一位就会露馅）；
- **饱和**：2047/2000/1999 都解成 1999（不是回绕成 0）；
- **保留位**：正常帧的 4 个空位为 0，手工置 1 必须被认出来；
- **CRC**：先确认"没改就能解出来"，再动负载中间一个 bit，必须 `kTryReject` 且 `consumed == 1`。

**规矩**：改空口格式必须配一个"编出来的字节 → 解出来的值"的往返测试。
两侧各自的自测都证明不了"位表一致"，只有真的对拍才能。

---

## 15. 当前收口状态（2026-10-12）

| 检查 | 结果 |
|---|---|
| 真实 armclang 编译（工程 34 TU，**零桩**） | **34/34 PASS** |
| 全量 78 TU + armlink（含尚未进 EIDE 的两个 .cpp） | **编译 78/78 + `LINK CHECK PASS`** |
| `test/state_step_test.cpp` | 786 项 / 0 失败 |
| `test/up_motor_frame_test.cpp` | 48 项 / 0 失败 |
| `test/dlx_mailbox_test.cpp` | 60 项 / 0 失败 |

**⚠ 仍需在 EIDE 里重构建一次**：`.eide/eide.yml` 已登记 `app_state.cpp` / `app_health.cpp`，
但 `build/FLY/compile_commands.json` 还没有它们（那是 EIDE 生成的）。

**未做**：上板回归（方案 §6.2 整张清单）。最该盯的是 `Arming` 驻留期间"桨在转但控制未接管"必须与
改造前逐字一致（`timer1.start` → `enableOutputs` → `dshot.start` 的顺序与时机）。


---




