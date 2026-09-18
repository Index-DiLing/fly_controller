# KMP 上位机(E:\kmpFly):DLX 报文 / 地面命令 / 回传数据处理

一句话:上位机是 KMP 桌面应用(Compose for Desktop),报文类写在 `Protocol.kt`,KSP 同时生成
**Kotlin 侧**的 `DLXController` 与 **C++ 侧**的 `DL_LIB/dlx.hpp` —— 两边字段/类型号/字节序都由生成器保证一致。

| 位置 | 作用 |
| --- | --- |
| `E:\kmpFly\shared\src\jvmMain\kotlin\kmpfly\Protocol.kt` | 报文类定义(Kotlin 侧"唯一真相"),文件头写着 `@file:CppProtocolTarget("E:\\STM32Code\\fly_controller\\DL_LIB\\dlx.hpp")` |
| `shared\build\generated\ksp\jvm\jvmMain\kotlin\DLXController.kt` | KSP 生成的收发代码(读写函数 + 每种报文一个 `xxxListener`) |
| `shared\...\kmpfly\connection\Controller.kt` | 后端控制器:报文回调 → 曲线数据 / 日志文件 |
| `shared\...\kmpfly\connection\ConnectionManager.kt` | 连接页 UI(串口 + 地面命令按钮),**GroundReply 的回调在这里** |
| `shared\...\kmpfly\components\Table.kt` | `DataEntry`/`DataTable`:键值 + 曲线采样(还留了 `UpdateLogSink` 接口,目前是 DISCARD) |

编译验证(离线即可,依赖在本机 Gradle 缓存里):

```powershell
E:\kmpFly\gradlew.bat -p E:\kmpFly --offline :shared:compileKotlinJvm
```

---

## 1. 地面命令 op 表(必须与 MCU 的 `GroundOp` 一致)

| op | 名称 | 参数 | 上位机按钮 |
| --- | --- | --- | --- |
| 1 | `Unlock` | `mode`:0 飞行 / 1 串口调试 / 2 日志管理 | 解锁MCU(调试模式)= `GroundCmd(1,1,0,0)`;解锁MCU(日志模式)= `GroundCmd(1,2,0,0)` |
| 2 | `ListSessions` | — | 回传会话信息 |
| 3 | `DeleteOldest` | `count` = 会话个数 | 删除一条会话 = `GroundCmd(3,0,0,1)` |
| 4 | `DumpSession` | `sessionId` | 回传最新会话(用 Controller 里 `lastSessionId`) |
| 5 | `FormatFlash` | — | 格式化 |
| 6 | `Ping` | — | Ping |

> 早期按钮把"解锁"写成了 `op=0`,与 MCU 的 `Unlock=1` 对不上(MCU 会回 BadCommand);
> 另外解锁按钮原先 `enabled = ... && !isSerialOpened`,串口一打开就点不动了 —— 两处已修。

## 2. 回传数据谁处理

| 报文(类型) | 回调位置 | 处理 |
| --- | --- | --- |
| `FlightCoreStatus`(8) | Controller.kt | 四元数→roll/pitch/yaw;`error` 变化时打日志;气压正常(`error & 0x04 == 0`)时 `height` 字段是**垂速** `teleVz`,否则是高度 `teleH` |
| `FlightLog`(15) | Controller.kt | **写文件**(见 §3)+ 同步喂 `Height`/`VH`/四路电机曲线 + `logRows` 计数 |
| `PerfMonitor`(14) | Controller.kt | DWT 周期 → us(168 周期/us),填 `loopAvg/loopMax/ekf/ctrl/log/io`;**冷启动第一窗忽略**;`loopMax>1600us` 打一条预警,最多每 10s 一条 |
| `SessionInfo`(13) | Controller.kt | 打一行 `[SESSION] id/槽/条/<本次上电>`;记录 `lastSessionId`、累加 `sessions` |
| `GroundReply`(12) | ConnectionManager.kt | 按钮侧判断 `status`;列表/回传/格式化的结果打日志 |
| `SimpleLog`(0) | Controller.kt | 转成 UI 日志行(`[MCU] ...`) |

## 3. 日志落盘(固定位置)

`kmpfly.connection.FlightLogWriter`(Controller.kt 同目录):

- 默认目录 **`H:\flight_logs`**;目录建不出来(比如没插 H 盘)自动退到 `<用户目录>\flight_logs`
- 文件名 `session_<会话号>_<yyyyMMdd_HHmmss>.csv`;串口调试流 `sessionId=0` → `debug_<时间戳>.csv`
- 一个会话一个文件,首行表头(39 列),浮点 `%.6g` 且强制 `Locale.US`(保证点号小数点)
- 每 64 行 flush,断开串口时 `close()` 并打一行"已保存 N 条到 <路径>"
- 每帧约 300B,50Hz ≈15KB/s(1 分钟 <1MB),可直接 Excel / pandas 打开

想要二进制存档(省空间/方便回放)时,把 `row()` 换成直接写 `FlightLog` 的 140B 负载即可,字段顺序与
C++/Kotlin 的 `FlightLog` 定义一致。

## 4. 踩过的坑(下次注意)

1. **PowerShell 5.1 读写文件必须显式指定编码**:`Get-Content -Raw` 默认按 ANSI 解码,
   再 `Set-Content -Encoding UTF8` 写回会把中文注释变成乱码(甚至吃掉换行导致语法错误)。
   改文件请用 `[System.IO.File]::ReadAllText/WriteAllText(..., UTF8Encoding($false))`,或直接整文件拷贝。
   同理:**补丁脚本自身的非 ASCII 字面量也会被按 ANSI 读坏**,所以脚本里的注释/字符串一律用 ASCII,
   需要写中文注释时用 ASCII 占位或改用 `pwsh`(PowerShell 7)执行。
2. 改 `Protocol.kt` 后必须让 KSP 重新生成:`compileKotlinJvm` 会跑 `kspKotlinJvm`,
   C++ 侧 `DL_LIB/dlx.hpp` 会一起刷新(注意它没有 include guard 的老版本已加 `#pragma once`)。
3. 报文类型号按类声明顺序分配,C++ 与 Kotlin 必须来自**同一次生成**;
  只改一边会出现"类型号对不上 → 静默丢帧"。

## 4.1 约定:生成的代码与生成器都不动

`processors/.../KotlinCodeGenerator.kt` 与它产出的 `DLXController.kt` **保持原样**,
生成代码里那几个 `println("CRC校验出错" / "Header错误" / "流已结束" / "读取流出错")` 走控制台,
不进屏幕日志(屏幕日志只收应用层回调:`[MCU] / [SESSION] / [PERF] / [DLX 不参与]`)。
应用侧的 `println` 已全部改成 `logger` 并经 `ScrollLogger` 上屏:

- `[THROTTLE]` 只在四路值变化时打
- `[DEBUG]` 调试帧限到 2Hz(原始 20Hz)
- `[EKF]` 帧间隔统计限到 1Hz(原来每帧都打,会把列表刷爆)

## 4.2 症状:控制台刷 `Header错误5b42` / `未知Header,可能没有对齐6f6f`

**原因**:MCU 把人类可读文本和 DLX 二进制帧混在同一路串口上。旧版 `logLine()` 用 `usart1.send()`
直接发 ASCII/UTF-8 文本(`"[Boot] ..."`、`"[Manage] ..."`),上位机按帧解析这些字节,于是:

| 打印出来的"帧头" | 实际内容 |
| --- | --- |
| `5b42` | `'[' 'B'`(来自 `[Boot]`) |
| `6f6f` | `'o' 'o'`(来自 `Boot`) |
| `745d` | `'t' ']'`(来自 `[Boot]`) |
| `e588` / `b0e8` / `86e6` … | 后面中文注释的 UTF-8 字节 |

顺带还会偶发一条 `CRC校验出错`(刚好凑出一个"看着合法"的帧头)。

**修法(MCU 侧,已完成)**:`flight_config_struct.hpp` 的 `logLine()` 现在把文本打成
`SimpleLog`(类型 0)帧:长度补齐成偶数(保证帧长偶数)→ `protocol.SimpleLogW()` → 阻塞发完。
上位机的 `simpleLogListener` 收到后 `logger.info("[MCU]", ...)` 显示在屏幕上(UTF-8 解码, 中文正常)。
发布版里已不存在任何直接 `usart1.send()` 的原始文本。

**使用建议**:先打开上位机串口、再发"解锁"命令。MCU 是从收到解锁那一刻才开始发流的,
这样上位机的字节对齐一定落在帧边界上;如果是"MCU 已经在发流、中途才打开串口",
生成代码按 2 字节步进、错位后不会自恢复(生成代码按约定不改),重连或复位一次即可。

## 4.3 症状:"超预算"警告只在开机时出现过一次, 之后再没响过

**原因**:`Controller.kt` 里那句 `perfWarned` 是**一次性 latch**(报过就置位, 只有 `loopMax<=1200us`
才解锁)。它第一次被触发的那一窗恰好是**冷启动窗**:

| 窗口 | 现象 | 真实含义 |
| --- | --- | --- |
| 上电第 1 条 | `n=1 work=5000us fifo=4477us` | 控制线程刚起来: BMI088 FIFO 积压 + EKF 首次整定, **不是稳态负载** |
| 第 2 条 | `n=100 work=564/4480us period=2000/5004us` | 稳态里真卡了一圈(~4.5ms), 但 latch 已被第 1 条吃掉, **没报警** |
| 第 3 条起 | `work=505/1271us` | 正常 |

**修法(两处, 已完成)**:

1. MCU: `kConfig.perfWarmupLoops`(默认 5)圈不计入统计窗口 —— 热身圈的 `loopMax` 丢弃并重新对表,
   于是第一个 `PerfMonitor` 窗口从稳态开始算;
2. 上位机: `perfWarned` 换成**冷却式提醒** —— 上电第一窗忽略, 之后 `loopMax>1600us` 每 10s 最多提醒一条。

### 4.3.1 当时的判读口径(供下次排查复用)

曾经在控制环里插过 TEMP 分段计时, 打 `[SEG1]/[SEG2]/[SEG3]` 三行文本。**现已全部清理**
(`PerfSegments`、`g_segAcc`、`perfLogSegments`、`[SEG*]` 打印、各段 DWT 埋点都不在了)。
当时的口径记在这里, 需要时照着临时加回去:

- `work` = 单圈控制环耗时, `period` = 两次唤醒之间的间隔, `imu` = FIFO 读取 + 配对 + EKF 快路径,
  `ekfF/ekfS` = EKF 快/慢路径, `baro` = 气压计读取 + 更新, `ctrl` = 角度环 + 混控 + DShot, `misc` = 余项;
- 判读: `period` 远大于 `work`(实测出现过 period=5004us / work=4480us)⇒ 卡顿不是控制环自己算出来的,
  而是**被别的线程/中断挤掉**了, 补算积压的 FIFO 才把 `work` 顶上去;`period ≈ work` ⇒ 是控制环内部某段变慢;
- 实测稳态负载(未解锁): `work≈505us/1271us(均/最大)`, `period=2000/2006us`, 主要构成是
  `ekfS≈598us/次`(250Hz)与 `imu≈446us`(含 4 个样本的快路径), 气压计 508us/20ms —— 2ms 周期下约 25% 占用。

### 4.3.2 现在还留着的"正式"负载通道

调试文本删掉后, 负载信息仍有一条非调试的正式通道, 不占额外带宽:

- `PerfMonitor`(14):5Hz, `loopAvg/loopMax(~周期数)/ekf/ctrl/log/io` + 窗口样本数 + 丢包数 → 上位机曲线页;
- `kConfig.perfWarmupLoops`(默认 5):起跑热身圈不计入该窗口(冷启动那几圈含 FIFO 积压 + EKF 首次整定,
  实测单圈能到 5ms, 计进去会让"超预算"提醒变假警报);
- `kConfig.logStackWatermark`(默认 false):打开后每 10s 用 `SimpleLog` 打一行三个线程的栈水位线。

## 5. 还可以做

- 会话选择界面:现在只有"回传最新会话"(用最近一次列会话的 `lastSessionId`);
  想做完整列表:把 `[SESSION]` 行收进一个 `mutableStateListOf` 再渲染成表格 + 每条一个"回传"按钮。
- `Table.kt` 里的 `UpdateLogSink` 目前是 DISCARD,可以接成"把勾选的条目同步写 CSV"。
- 上位机侧的自动化回归:Kotlin 侧写一个"喂字节 → 断言回调"的测试,与 MCU 的 `test/release_check` 对应。
