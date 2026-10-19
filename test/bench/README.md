# test/bench —— 链路 v2 P0 台架固件

不属于主线固件，也不参与主线构建（EIDE `srcDirs` = `app`/`DL_LIB`/`rt-thread`）。目的、命令、
接线、测量步骤与结果记录表见 [`docs/链路层v2_P0台架.md`](../../docs/链路层v2_P0台架.md)。

| 内容 | 说明 |
|---|---|
| `link_bench_main.cpp` | 飞控端独立 `main`：USART1 文本命令、USART3（E220）DMA+IDLE 收发、DWT 时间戳、AUX EXTI |
| `_rc_stage/link_bench/` | 遥控器端独立 ESP-IDF 工程（**回同步副本**；权威版本在 RC 仓库，见 `_rc_stage/SYNC.md`） |
| `_out/` | 沙箱内构建产物（`build/` 不可写时的自动回退目标；不提交） |

## 测量口径备忘（2026-10-18 修正，读数据前先看）

1. **版本标识**：上电 banner 第一行是 `[ver] p0 bench r3 2026-10-18`；交互敲 `ver` 也能看到。
   脚本先比对版本再解析数据——**旧 bench 没有 `offAux`/`口径`/`d0AuxUs`，数据一律作废**。
2. **`stat` 的空样本显示 `n=0 (--)`**：`min` 的初值是 `0xFFFFFFFF` 并在上电时 `resetAllStats()`，
   不再把"还没测过"打印成 `min=0`。`min=0` 现在只在真的测到 0µs 时出现。
3. **`round` 测的是"4 个上行 burst + 1 个 D0"的周转**，不是 v2 的 120ms 轮次；
   `roundUs` 落在 4×s 或 5×s 都正常（21B D0 ≈25ms > 22ms 槽 ⇒ 必然吃掉一帧上行）。
   结束行字段顺序固定：`rounds= dlSent= bursts= bursts/round= txtSkip=`。
4. **`txtSkip` = 被过滤掉的可打印 ASCII burst**（遥控器 5s 心跳 `#STAT`、`#ACK/#PONG`）。
   它们不参与 idx/轮次统计——否则会把"4 上行 + 1 下行"切开（旧固件里 `roundUs` 的
   131/176ms 与 4.89s 离群值就是它）。测量载荷首字节 0xA0..，不受影响。
5. **`auxDrop` 不是丢帧判据**（那是 AUX 边沿环形缓冲 128 深的溢出计数）。`rx`/`echo`/`round`
   与空闲主循环都会 `drainAuxEdges()`；只有 `round` 结束时打印的 **`rxDrop` 增量**才是链路判据。
6. **`d0OffUs` 是软件口径**（从 IDLE 戳算，恒等于 guard 只证明软件等待）；
   **`d0AuxUs` 是物理口径**（从 U3 的 AUX 上升沿算）——判断 guard 够不够看它。

## 构建（飞控端）

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\bench\build_fc_bench.ps1 -Rebuild
```

产物默认在 `build/bench/`（`dl_fly_bench.axf/.hex/.bin`）。若当前会话写不了 `build/`，
脚本自动退到 `test/bench/_out/` 并打印用完全权限重跑的命令（最近一次实测：
`_out/dl_fly_bench.hex` = 159,261 B，总 ROM 56,608 B，版本 `p0 bench r3 2026-10-18`）。

## 构建（遥控器端）

见 `_rc_stage/link_bench/README.md`；工程需先同步到
`E:\ESP_PROJECT\controller\bench\link_bench\`（命令见 `_rc_stage/SYNC.md`）。

## 自动化

`python tools\bench\run_p0.py --port COM11 --soak 60`（依赖 pyserial；先核对 [ver] p0 bench r3 2026-10-18）。

---

## baro bench —— 气压计偶发 0x404 定位（2026-10-18）

`baro_bench_main.cpp`：飞控端独立 `main`，用 USART1 文本命令把运行期偶发 `0x404`
（`ExceptionCode::BARO_READ_FAILED`，`DL_LIB/dlx_exception.h:59`）拆成**可判别**的来源。
目标、假设清单（H1~H7）、判据表与结果记录表见
[`docs/气压计0x404定位_台架.md`](../../docs/气压计0x404定位_台架.md)；`0x404` 的四个来源里只有两个与
I2C 事务有关，本台架就是把它们分开。

**版本行（banner 第一行，`ver` 也打它）**：`[ver] baro bench r3 2026-10-18`
（`tools/bench/run_baro.py` 的版本门禁按这一行工作；改口径/命令/输出字段必须 +1 并改日期。）
**旧版本一律作废**：r2 相对 r1 在 `[fail]` 行尾多了 `err=` 与 `scl=/sda=` 并有 `kind=other`；
r3 相对 r2 **输出字段一个没变**，修的是 soak 的绝对计时（见下一条）—— r2 的 `soak <ms>` 只要
`ms > 25566` 就永远不自己结束，所以拿 r2 跑长 soak 得到的数据不可用。**门禁只认 r3。**
**soak 时长口径（r3）**：`soak <ms>` 的 `ms` 与 `[soak]`/`[soak.end]` 的 `t` 都真的可用到
`kSoakMaxMs = 3600000`（1h），`period_ms ∈ [1, 1000]`；原因是绝对计时改成了逐轮累加（`uint64`），
不再受 DWT `CYCCNT` 的 **25.566s**（= 2³²/168MHz）回绕影响。推导（单轮差值上界 ≈1.1s、累加器单调、
1h = 6.05e11 周期 < 2⁶⁴）写在 `baro_bench_main.cpp` 的 `cmdSoak()` 里，可据此复核。

### 前置条件与接线

| 项 | 要求 |
|---|---|
| 控制台 | USART1 PA9/PA10 → USB-TTL，**115200 8N1**（台架固定值，不是主线的 2Mbaud；只影响打印） |
| 被测 | BMP388 在 I2C1 PB6/PB7，地址 0x76，400kHz（`cfg::hw::baro::*`） |
| `stress` 可选负载 | SPI1 BMI088（CS=PB12/PB13, `cfg::hw::imu::kSpi`）、I2C2 VL53L1X（PF1/PF0, XSHUT=PD2）、ADC1 PA0/PA1 |
| 烧录 | ST-LINK/J-Link + 你惯用的下载工具；**这个镜像替换了 `release.cpp` 的 `main`，不要在装机状态飞它** |
| 桨 | 建议拆桨（台架只读传感器、不驱动电机，但 `stress` 会跑 SPI/I2C/ADC 负载） |
| 主机构建前提 | `build/FLY/compile_commands.json` 与 `build/FLY/dl_fly.sct` 必须已由 EIDE 生成（脚本靠它们重放编译参数，不自己编一套） |

### 命令（大小写不敏感，行式，提示符 `ok>`；输出字段顺序冻结）

```
ver | help | cfg | sr | bus
init [hz]                          驱动的分阶段 init（按 kInitRetry 重试），stage=reset|chipid|calib|write|readback
soak <ms> [period_ms]              持续读；每 1s 一行 [soak]，结束一行 [soak.end]；period 默认 8ms
                                   ms ∈ [1, 3600000]（1h），period ∈ [1, 1000]ms；t 单调不减（r3 修）
inject <n>                         对不存在的地址 0x33 写 ⇒ 必然 NACK ⇒ 记录要几次尝试才恢复
speed <hz>                         重启 I2C1（I2C_DeInit）并重新 init 器件（H2：100k/400k 对比）
stress <on|off>                    SPI1 BMI088 读 + I2C2 VL53L1X 读 + ADC 采样（H6：并发负载）
irqblock <us> <period_ms> <on|off> TIM7 更新中断里忙等 us µs（H5/H7：事务中被 ISR 打断）
recover | stat | reset
```

参数不合法（含未知命令）只回一行 `[err] bad-arg <原文>`；`soak` 运行中敲任意键即中止并照常收尾。
`soak`/`inject` 在器件未初始化时**拒绝执行**并回 `[err] not-initialized`（见下面口径 2）。
自动化：`python tools\bench\run_baro.py --port COMx --soak 60000 --inject 20`。

### 读数据前必须知道的口径

1. **`kind` 是判出来的，不是猜的，而且先按错误码分流**：只有 `err == 0x0404`（`BARO_READ_FAILED`）
   才进 tx/comp 二选一——
   `eventTimeouts()` 有增量**或** `baro.failCount()>0` ⇒ `tx`（突发读事务失败，来源 A；init 阶段同类 = 来源 C）；
   两者都没动 ⇒ `comp`（事务成功了、是补偿结果被否掉，来源 B）。
   其它错误码（尤其**未初始化**时的 `0x0403 BARO_CALIB_INVALID`，那次调用一个寄存器都没读）单列
   **`kind=other`**，行尾的 `err=0x....` 给出原因。
   证据：`failStreak` 只在 `readBurst` 失败时 ++、成功时清零（`dlx_bmp388.hpp:328-334`）。
   **不变量：`fail == tx + comp + other`**（`[soak]`/`[soak.end]` 仍只印 `fail/tx/comp`——契约字段没改，
   所以 `other = fail − tx − comp`；有 `other` 时说明失败不在 0x404 的判别范围内，别拿它当 comp 读）。
   这条门禁是必须的：否则"忘了 `init`"会让健康板子报 `fail≈att`、`comp≈att`，正好命中判据表里
   "comp 占大头 ⇒ 与 I2C 无关"，签名与真结论相反。
2. **上电会自动初始化一次**，并打一行 `[boot] baro init=<OK|FAIL> err=0x....`（**不是** `[init]`，避免与
   `init` 命令的解析撞车）。它只做**一次** `tryInit`（健康器件 ~3ms；不用 `init()` 的 20×100ms 重试，
   否则会把 `run_baro.py` 探测串口的 2s 超时拖爆（`_probe_port` 的 `expect(..., 2.0)`）。`soak`/`inject` 在器件未初始化时**拒绝执行**并回
   `[err] not-initialized`（此时脚本等不到 `[soak.end]`，会自己报 FAIL，不会静默产出假数据）。
   要完整重试请用 `init` 命令。
3. **`[fail]` 行的 `sr1/sr2/scl/sda/lastEv` 是失败当刻的快照**，取在任何 `recover()`／下一次事务之前
   （`IICDevice::read()` 下一次会先 `bus.recover()`，那 9 个 SCL 脉冲会毁掉现场）。
   H7（事务途中被 ISR 打断 ⇒ OVR）的判据就是 `sr1` 的 **bit11 = 0x0800**；
   **H4（总线被从机拉住）的判据是行尾的 `scl=0` / `sda=0`**（r2 起打这两个字段，就在 `err=` 之后；
   它们与 `sr1/sr2` 同一个快照块，是失败**当刻**的电平，而不是事后敲 `bus` 看到的电平）。
   行尾字段顺序：`... streak= err= scl= sda=`（既有字段名/顺序未动）。
4. **`status`/`raw` 只有 `kind=comp` 时是真数据**：comp 意味着突发读成功过，而驱动不导出那个缓冲，
   于是台架在快照之后补一次**诊断性**突发读，取"失败之后那一刻 0x03~0x09 的现场"。
   `kind=tx` 时驱动已把缓冲清零且不导出 ⇒ 固定打 `status=0x00 raw=000000000000`；
   tx 的诊断靠 `sr1/sr2/lastEv/evD/durUs/gapUs`。`kind=other` 两者都不做（没有可比的事务）。
   （tx 为什么不补诊断读：失败后 `busError` 置位，再发事务必须先 `clearError`，那会绕过下一次尝试里的
   自动 `recover()`，把 H1/H4 要观测的恢复行为改掉。宁可少一个字段，不改被测对象。）
5. **`[soak]`/`[soak.end]` 的计数都是"本次 soak"的**（从 0 起算）；`evTimeouts` 是
   `IICBus::eventTimeouts()` 的**累计值**（该 getter 无法清零）。`maxgapUs`/`maxdurUs` 是本次 soak 的最大值；
   `stat` 打的是自 `reset` 以来的累计值。`failrate` 是 `fail/att`（`%.1e` 形状，如 `3.2e-04`；`att=0` 时为 `0.0e+00`）。
6. `gapUs` 是**两次尝试起点**之差（DWT，start-to-start），首次为 0；周期超期时它大于 `periodMs`，
   这正是"本次事务把节拍吃掉了"的证据。`[recover]`/`[inject]` 的 `next_ok`/`attempts_to_ok` 只认
   "I2C 事务通了"（comp 类失败算通；未初始化的 0x0403 没有事务，不算通）。
7. `[stress]` 行的 `spi/i2c2/adc` 是**累计健康位**：自这次 `stress on` 以来该路是否一直成功。
   某一路首次失败后，台架会**停掉该路的后续步进**（器件不在时每次事务要在 `waitEvent` 的 20 万次轮询
   上限上空转几十毫秒，那种空转是台架自己的伪影，不是被测现象），所以 `stress off` 时打印的仍是
   这一次的结论。
8. **`irqblock` 的机制是"TIM7 更新中断里忙等"，不是"主循环关中断"**。H7 说的是**事务进行途中**
   被打断（400kHz 下一个字节只有 22.5µs，软件晚一步读 DR 就 OVR），而主循环里 `__disable_irq()`
   只会推后中断、推后的是它自己的下一轮轮询，插不进正在进行的事务中间 —— 复现不了 H7。
   代价：**"关中断"这一项没做**（`__disable_irq` 路径没实现）。另外 TIM7 是
   `stm32f4xx_it.cpp:288` 唯一留了转发的定时器，台架借它注册回调，不覆盖 `IRQHandler`。
   参数约束：`us < period_ms×1000` 且 **`period_ms ≤ 65`**（见下面的局限一条），超限直接 `[err] bad-arg`。
   另有**时钟约束**：TIM7 的计数时钟必须能整除到精确 1MHz（本板 84MHz/84 = 1.000000MHz 整除）。
   若换 HCLK 后不整除，`irqblock ... on` **不启动也不打 `[irqblock]` 行**，只回
   `[err] bad-arg irqblock-clock timClock=<n>` —— 宁可拒绝，也不让"打印的 periodMs"与"实际周期"差 ~1%。
   （`off` 不受此限：停止不依赖任何时钟，任何时候都必须能关掉。）
9. banner 除版本行外还会顺带打 `[cfg]`/`[sr]`/`[bus]`（与 link bench 的 banner 同款，便于上电即读现场）；
   它们只是上电快照，命令后再打的就是命令的答复。上电 DWT 时基按 `SystemCoreClock = 168MHz` 换算。
10. **`delay_ms` 在 SysTick 未使能时走 NOP 近似分支**（`dlx_delay.hpp:19-26`）；本台架不启动
    RT-Thread，所以 `init` 的 100ms 重试间隔与 BMP388 的 2ms 软复位等待都是这个近似量级（够用），
    而所有**微秒级测量**走 DWT，不受影响。
11. **DWT 时基有硬回绕点 25.566s**（= 2³²/168MHz）。凡是"单次差值"的用法（`durUs`/`gapUs`/5ms 节拍/
    微秒忙等）都在代码里写明了上界 ≪ 25.566s；**跨回绕点的绝对经过时间只有 soak 的 `t`**，
    r3 起用逐轮累加的 `uint64` 累加器（`cmdSoak`），所以 `[soak] t`/`[soak.end] t` 单调不减、
    长 soak 正常结束。**r2 及以前的 `soak <ms>` 在 ms > 25566 时静默归零、永不结束**（上板实测：
    `t` 从 24999 跳回 3）—— 见到 `[soak] t` 非单调或 soak 超时，先查镜像版本是不是 r3。

### 构建

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\bench\build_baro_bench.ps1 -Rebuild
```

产物 `build/bench_baro/dl_fly_baro_bench.axf/.hex/.bin`（`build/` 在本会话不可写时脚本自动退到
`test/bench/_out_baro/`，并打印用完全权限重跑的命令）。2026-10-18 实测（`-Rebuild`，77 个 TU，
`release.cpp` 被剔除、由 `baro_bench_main.cpp` 提供 `main`）：

```
test/bench/_out_baro/dl_fly_baro_bench.axf  126,552 B
test/bench/_out_baro/dl_fly_baro_bench.hex  163,176 B
test/bench/_out_baro/dl_fly_baro_bench.bin   58,000 B   (Total ROM 58,000 B / RW+ZI 93,040 B)
```

> 版本 `r3`（2026-10-18）。历史：`r2` = 126,124 / 162,591 / 57,792，`r1` = 125,964 / 162,215 / 57,656
> —— 各版**不可混用**（r2 起有了 `[fail]` 行尾的 `err/scl/sda`、`kind=other` 与 `irqblock` 时钟拒绝；
> r3 修了 soak 的 25.566s 回绕，输出字段未变但 r2 的长 soak 不可用）。

### 台架覆盖不到的部分

- 台架是**裸机 + 阻塞式命令**：没有复现 `sns_baro(22) 空转 → link_tel(24) 饿死` 这条 RT-Thread 抢占链
  （见文档 §10）；`[soak] maxdurUs` 只回答"单次事务最坏阻塞多久"。
- **来源 D**（`app_sensors.hpp:546` 的 `baroReference.apply()` 失败后硬编码报 0x404）只在主线应用里出现，
  台架不加载 app 层，所以台架日志里不会出现它。
- 器件自身的 `ERR_REG` 只在 `kind=comp` 的诊断读里顺带看到 `STATUS`；没有专门的器件自检命令。
- **`irqblock` 的周期上限是 65ms，且 `us` 必须小于一个周期**：TIM7 是 16 位定时器（`ARR` 只有 16 位，
  RM0090 的 TIM6/TIM7 时基单元），本台架把它的计数时钟配成 1MHz ⇒ `ARR = period_ms×1000 − 1 ≤ 65535`
  ⇒ `period_ms ≤ 65`。超限、或 `us ≥ period_ms×1000`（中断会吃掉几乎全部 CPU，`soak` 期间连按键都
  中止不了），都直接回 `[err] bad-arg` —— 不再出现"打印的是请求值、硬件实际被 ARR 静默截断"的错配
  （D3：旧上限 10000 时 `period_ms=100` 实际 34.5ms、`period_ms=66` 实际 0.46ms）。
  H4/H5 的短停顿实验用 1~10ms 就够，这个上限不影响任何实验。
  另外 TIM7 的计数时钟**必须整除到精确 1MHz**（本板 84MHz/84 整除）：不整除时 `on` 会被拒绝
  （`[err] bad-arg irqblock-clock timClock=<n>`，且不启动、不打 `[irqblock]` 行），
  这样"打印值 = 实际周期"在任何 HCLK 配置下都成立；代价是换非整数 MHz 的 HCLK 时 `irqblock` 直接不可用
  （比给一个差 ~1% 的周期好）。
