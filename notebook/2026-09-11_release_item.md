# 发布版: release.cpp + flight_config_struct.hpp

一句话：把首飞用的 `main_final_test.cpp` 重写成正式发布项 —— 上电模式解耦成独立函数、
日志换成 FlashManager 文件系统(记录更多信息)、姿态估计换成已测过的 16 维 EKF 并加气压计高度，
其余(500Hz 控制环 / NRF 遥控 / DShot / 线程结构)与首飞版保持一致，便于对比。

发布项由 4 个文件组成，职责单一、互不缠绕：

> 总览图（模块结构 / 上电模式 / 线程时序 / 控制环 / 数据结构 / 日志链路 / 报文一张图看全）：
> `docs/release_overview.png`（矢量版 `docs/release_overview.svg`，生成脚本 `docs/make_release_overview.py`，
> 改完代码重跑一次即可刷新：`python docs/make_release_overview.py`）
>
> 上板测试清单（带勾选项与实测记录表）：`docs/release_test_checklist.md`

| 文件 | 职责 |
| --- | --- |
| `flight_config_struct.hpp` | **发布项唯一的公共头文件**：编译期配置 `kConfig`（含 `kConfig.hw` 全部引脚/SPI/地址/缓冲）、上电模式枚举、运行时状态 `FlightRuntimeState`、地面命令字段约定、日志状态位，以及共用上下文 `ReleaseContext` + 小工具 `logLine/flushSerial/replyGround/drainDlx/onFlashBusy/writeLogEntry` |
| `release_log_manage.hpp` | **日志管理模式**（与飞行完全分离）：列会话/删最老 N 个/整会话回传/整片格式化/Ping/解锁切模式；不初始化 IMU/NRF/定时器，物理上不可能输出电机 |
| `release.cpp` | `main` + 上电模式判定 `decideBootMode()` + 飞行/串口调试主流程 `runFlightStack()` |
| `DL_LIB/W25Q128/dlx_flash_manager_config.h` | `LogEntry`(飞行日志字段) 与文件系统几何(默认单次预留 32 槽) |

报文不再手写编解码：地面模式的 4 条报文用 KSP 生成的 `GroundCmdW/GroundReplyW/SessionInfoW/PerfMonitorW`
与回调，日志回传用生成的 `FlightLogW`（详见 [2026-09-11_dlx_protocol.md](./2026-09-11_dlx_protocol.md) §5/§6）。
配置项也全部收进 `kConfig`，`release.cpp` 顶部只剩"一次排空日志队列最多取几条"这类局部常量。

---

## 1. 上电模式(跳线帽 PB8 & PB9)

`decideBootMode()` 是唯一决定"上电后干什么"的地方，跟飞行逻辑完全解耦：

```
跳线帽没接 ──► Flight: 直接飞, 完全不看串口(g_debug = nullptr, 一声不响)
                │
跳线帽接上 ──► 等串口"解锁"命令(LED 120ms 闪烁) ──┬─ Unlock(mode=1) / UnBlock(param=1) ──► SerialDebug
                │                                  ├─ Unlock(mode=2) / UnBlock(param=2) ──► LogManage
                │                                  └─ Unlock(mode=0) ─────────────────────► Flight
                └─ 超时(kConfig.groundWaitTimeoutMs, 默认 0 = 一直等) ──► LogManage(安全档)
```

- 跳线检测用两相法：先 `PB8` 输出低 + `PB9` 输入上拉读一次，再反过来读一次，
  两相里**任一脚被拉低**就算"接上" —— 兼容"一个跳线帽短接两脚"和"两脚各自接地"两种做法；
  检测完两脚都恢复成输入上拉，不驱动。
- 三个模式的差别：

| 模式 | 日志 | 电机 | 串口 |
| --- | --- | --- | --- |
| Flight | 写 flash(50Hz) | 正常 | 不处理 |
| SerialDebug | 原始日志帧直接发串口(100Hz) + `PerfMonitor`(5Hz) | 可解锁(台架) | 遥控指令 + 负载统计 |
| LogManage | 只读/删/回传/格式化 | **绝不输出** | 命令/应答/日志流 |

---

## 2. 日志

### 2.1 条目字段(`LogEntry`, 共 144B = 12B 管理器 + 132B 业务)

管理器的 12B：`type / reserved / crc / sessionId / seq`(写入时自动填)。

| 业务字段 | 含义 |
| --- | --- |
| `tickMs` | 上电以来的毫秒 |
| `flags` / `events` | 状态位(电平) / 事件位(边沿)，含义见 `flight_config_struct.hpp` 的 `kLogFlag*` / `kLogEvent*` |
| `linkAgeMs` / `loopPeriodUs` | 遥控链路延迟 / 本周期控制循环实测周期 |
| `quat[4]` | EKF 姿态(机体->世界) |
| `bodyRate[3]` / `accelG[3]` | 机体角速度 / 比力 |
| `rateSetpoint[3]` / `torque[3]` | 期望角速度 / 期望力矩 |
| `targetPitchDeg` / `targetRollDeg` / `targetHeightM` | 遥控期望值(高度为定高预留) |
| `throttle` / `manualThrottle` | 控制输出油门 / 遥控手动油门 |
| `heightM` / `vertVelMps` | EKF 高度 / 垂向速度 |
| `baroRelM` / `baroAbsM` | 气压高度(相对起飞点) / (绝对) |
| `gyroBias[3]` | EKF 陀螺零偏(每 `ekfLogDivider` 条才更新一次，中间几条为 0) |
| `motor[4]` | 四路 DShot 输出(物理通道顺序) |

### 2.2 容量与频率

| 项 | 值 |
| --- | --- |
| 条目步长 | 148B(帧头+结构体+帧尾，对齐到 4) |
| 每槽条目 | 885 条(槽 128KB) |
| 单次日志(一个会话) | 32 槽 = 4MB = **28320 条 ≈ 50Hz 下 9.4 分钟** |
| 上电擦除 | 只擦上次真正用掉的槽(典型 1~2 槽 ≈ 0.3~0.6s)；写满即停(`Halt`)，**飞行中绝不擦除** |

想飞更久：地面先用管理模式 `clearHistory()`(删历史、槽并给当前会话)或整片格式化。

### 2.2.1 布局指纹不符(`FLASH_GEOMETRY_MISMATCH` = 0x0102)怎么办

改了 `LogEntry` 字段/大小之后，板上旧数据的布局指纹与新固件对不上，`fs.init()` 会返回
`err = 0x0102` 且**不自动重建**（怕误删旧日志）。此时：

| 模式 | 行为 |
| --- | --- |
| 飞行模式 | 照飞，只是不写日志（置 `kLogFlagLogError`）；想立刻记录就开 `kConfig.autoFormatOnMismatch` |
| 日志管理模式 | **仍然可以进入**（只要 W25Q128 认得到就行），发 `op=5` 整片格式化即可恢复；格式化会自动重新 `init`（预留 1 槽） |
| 格式化后切飞行模式 | 允许（会按 `flightLogSlots`=32 槽重开会话）——注意这里判断的是 `fs.isReady()` 实时状态，不是上电时那次 init 的结果 |

> 早期版本在"布局不符"时把日志管理模式也挡在门外，等于唯一的恢复入口没了（表现为
> `[Manage] flash 不可用…复位重试` 死循环），已修。

### 2.3 日志流格式(flash 回传与串口调试共用)

回传/调试流都是 DLX 的 `FlightLog` 报文(类型 15，负载 140B，帧 142B)：

```
[head 2B][sessionId][seq][tickMs][flags][events][linkAgeMs][loopPeriodUs]
         [quat×4][bodyRate×3][accelG×3][rateSetpoint×3][torque×3]
         [targetPitchDeg][targetRollDeg][targetHeightM][throttle][manualThrottle]
         [heightM][vertVelMps][baroRelM][baroAbsM][gyroBias×3][motor×4]
```

- flash 里存的还是 `dlx::LogEntry`(POD，CRC16，掉电安全)；回传时现场转成 `FlightLog` 帧，
  上位机用生成代码解析，不必手算字节偏移；
- 调试模式同一路串口还夹着 `PerfMonitor`(类型 14)，按类型号分流即可；
- 报文定义/字节序见 [2026-09-11_dlx_protocol.md](./2026-09-11_dlx_protocol.md) §5/§6。

---

## 3. 日志管理模式：上位机操作流程

```text
1) 接上跳线帽, 上电, 收串口 -> GroundReply(op=1, status=0)            // 已在地面模式
2) GroundCmd(op=1, mode=2)                                            // 选日志管理
3) GroundCmd(op=2)  -> 若干 SessionInfo(会话号/槽数/条目数/是否当前会话) + GroundReply(value=会话数)
4) GroundCmd(op=4, sessionId=X)                                       // 回传整个会话(原始日志帧流) 
   收到 GroundReply(op=4, status=0, value=条数) 表示这个会话发完了
5) GroundCmd(op=3, count=N)                                           // 丢掉最老的 N 个会话
6) GroundCmd(op=5)                                                    // 整片格式化(几十秒, 丢日志+参数)
7) GroundCmd(op=1, mode=1)                                            // 转台架调试; mode=0 转飞行
```

注意：

- **删除/回传都以"会话"为单位**(一次上电一个会话)，符合文件系统的环形队列语义；
- `ListSessions` 的条目数是**扫一遍数出来的**(槽头里没存)，历史会话多的时候要几秒到几十秒；
- 管理模式启动时也会 `init()`(开一个"当前会话"，占 1 个槽)，它本身会出现在会话列表里(标记 `active`)；
- 管理模式下 `runLogManage()` 是单线程直接收发，日志线程/控制线程都没有启动。

---

## 4. 传感器融合(EKF + 气压计)

### 4.0 遥控回传(FlightCoreStatus)里 Height 字段的语义

回传遥控器的结构体**没有变**，但那个 float（名字仍叫 `height`）现在按气压计状态区分语义：

| 情况 | 回传内容 |
| --- | --- |
| `telem.baroValid == true`（BME280 正常） | **EKF 估计的垂向速度** `vertical_vel_mps` [m/s] |
| 气压计不可用 | 退回 EKF 高度 `height_m` [m] |

实现：控制线程把 `baroValid` 一起写进 `FlightTelemetry`，`nrf_tel` 线程按
`(status & kTelStatusBaroOk) ? vertical_vel_mps : height_m` 填 `FlightCoreStatusW(...)`。

字段名不改（`height` 就是垂速，名字不符是已知且接受的设计）。区分"这个 float 装的是哪一个"
**看 `status` 的 bit7（`kTelStatusBaroOk`）**；`error` 的 bit2 是同一事实的粘滞版，
一旦气压计出过错就再也不归零，不能拿来判断。

回传的 `status` / `error` 各 8 位，**两个都是实时值**：`status` 报此刻状态，`error` 只放
真故障且这一刻没问题就回 0（"一闪而过别漏看"是显示端的事：遥控器/上位机自己按位或锁存）。
完整位定义 + 遥控器侧需要同步的改动见
`notebook/2026-09-13_remote_compat.md`。

---

## 4.1 融合流程

与 `main_att_ekf.cpp` 同一套做法：

```
每个陀螺样本(2kHz):  ekf.integrateNominal(gyro, accel, 0.5ms)      // 快路径: 只用 IMU 数据积分
每 8 个样本(250Hz):  ekf.propagateCovariance(...) + updateAccelerometer()
每 20ms:             ekf.updateBarometer(baroAbs - baroRef)        // BME280, I2C1 100kHz
解锁瞬间:            开始累积气压基准(见下)
控制接管(en=true):   baroRef = 启动序列内的平均气压高度; ekf.reset(当前姿态, 0, 0)
```

- 控制器吃的是 `ekf.getQuaternion()` / `getHeight()` / `getVerticalVelocity()`(定高就绪)；
- 整定：纯 IMU 不估加速度零偏(`estimate_accel_bias=false`)，倾角门限 2.0 m/s²、气压噪声 1.0m(与实测一致)；
- 气压计初始化失败不影响飞：只记 `kLogFlagBaroOk=0`，垂速退回不可信。

### 4.1.1 气压基准怎么取的(两处平均)

| 时机 | 做法 | 代价 |
| --- | --- | --- |
| 上电(控制环起来之前) | 连读 16 次、每次间隔 50ms 求平均当基准 | 多花 800ms 启动时间(不影响控制环) |
| 解锁 → 控制接管 | **复用启动序列那 3.2s 里每 20ms 那次气压读数**累积，`en=true` 时提交平均值并归零 EKF 高度 | 零额外 I2C 读、零阻塞 |

第二个是刻意这么设计的：`motorArmDelayMs`(默认 3200ms) 本来就是"电机 0 油门、飞机停在地面上"
的窗口，正好拿来攒样本 —— 3200ms ÷ 20ms ≈ **160 个样本**，比上电那次的 16 个还稳。

> 千万别在解锁分支里写 `rt_thread_mdelay` 求平均(曾经踩过)：那是几百 ms 的阻塞，
> IMU FIFO 会溢出、EKF 丢数据、`loopMax` 直接爆表。要多个样本就用这个"顺手攒"的写法。

中途停机(`stop`)会丢弃未提交的累积值；`motorArmDelayMs` 改成 0 时样本数为 0，
此时不提交新基准(沿用上电那次的)，不会用平均值去除以 0。

### 4.2 控制链路与首飞版(main_final_test.cpp)的一致性(2026-09-13 核对)

逐项对比过, **控制相关的东西完全一致**, 没有隐藏的差异:

| 项目 | 首飞版 | 发布版 | |
| --- | --- | --- | --- |
| 控制频率 / 周期 | 500Hz / 2ms | 同 | ✓ |
| 控制器与混控 | `FlightController::updateAngle` + `mixMotors` | 同 | ✓ |
| 参数表 | `kFlightControlParamDefaults` | 同(两版都不覆盖) | ✓ |
| **电机映射** | `{0,2,3,1}` (逻辑M0→物理通道0, M1→2, M2→3, M3→1) | `kConfig.motorMap{0,2,3,1}` | ✓ |
| DShot 换算 / 饱和判据 | `×1900+50`, `≤55 或 ≥1945` | `dshotUnit=1900`/`dshotOffset=50`, 同判据 | ✓ |
| 油门地板 | `max(manThr, 0.01)` | `kConfig.minThrottle=0.01` | ✓ |
| 解锁等待 / 失控保护 | 3200ms / 5000ms | 同 | ✓ |
| 遥控指令语义 | 0 自稳 / 1 直通 / 2 解锁 / 3 停机 / >3 硬停机 | 同 | ✓ |
| 期望姿态 | `quatFromEulerZYX(0, pitch, roll)` | 同 | ✓ |
| IMU 轴重映射 | `kSensorYawDeg = 0` | `imuYawDeg = 0` | ✓ |
| DShot 模式 | Dshot300 | 同 | ✓ |
| 姿态源 | Madgwick AHRS | **EKF** | 有意变更 |
| 垂速有效性 | 固定 `false` | `baroHealthy`(气压计可用才 true) | 新增 |

**HOVER_THROTTLE: 0.30 → 0.25**(首飞实测悬停约在 25% 杆量)。它有两处作用, 改它等于同时改两件事:

1. **定高模式的基准油门**: `throttle = hover × (1 + acc_z/g)`(现在还没启用高度环);
2. **混控比例** `k = m·g/(4·hover)`: hover 调小 ⇒ k 变大 ⇒ 同样力矩需要的油门差动变小 ⇒
   **姿态环的等效增益按 1/hover 变化**。0.30→0.25 等于把等效增益降到原来的 **0.83 倍**,
   手感会比首飞软一档(如果首飞那 0.30 本来就偏高, 这次是"修正成物理正确")。
   嫌软就把 `RATE_ROLL/PITCH_P` 从 8.0 提到 ~9.5(=×1.2)补回来。

> 注意 `updateAngle()` 不动 `out.throttle` —— 自稳模式下油门完全由**手动摇杆**决定,
> `HOVER_THROTTLE` 目前只影响混控比例; 只有启用 `updateAngleHeight()` 后它才决定基准油门。

**顺手修的一处过时断言**: `flight_control_test.cpp` 里"大油门+大力矩必须饱和"那条写死了
`torque = 0.3 N·m`, 按现在的参数(臂长/质量/hover)已经不会让混控饱和了 —— 也就是说
**这条断言在改 hover 之前就是失败的**(已用 0.30 复现确认)。改成按参数反推阈值
(`Ty > (1-T)·4·ARM_FORWARD_M·k`, 再乘 1.2 余量), 以后改质量/臂长/hover 都不会悄悄失效。

### 4.3 Madgwick 对比:试过, 已撤销(2026-09-14 加 / 09-15 撤)

曾在发布版里加过"调试模式额外跑一套 Madgwick + 用 `QuatW`(类型 2)发送"来做和 EKF 的对比。
实测发现 **Quat 和 FlightLog 两路打印频率对不齐**(Quat 只在"这一轮真发了日志"时才发,
和 FlightLog 不是严格一一对应), 曲线对不上时间轴, 对比起来不方便, 所以**已整体撤销**
(`release.cpp` / `flight_config_struct.hpp` / `Controller.kt` 里的相关代码与注释都删掉了)。

算法对比改到 **`main_att_ekf.cpp`** 那份工程里做 —— 那边本来就是专门跑滤波对比的, 节拍自己说了算。

---

## 5. 上电时序

```
上电 ─ DWT 计时使能 ─ 串口(2Mbaud) ─ 判定模式
   ├ Flight     : flash.init() + fs.init()(擦上次用掉的槽, ~0.3s/槽) ─ 传感器/EKF/NRF ─ 线程 ─ 500Hz 控制环
   ├ SerialDebug: 不碰 flash ─ 传感器/EKF/NRF ─ 线程 ─ 500Hz 控制环(日志走串口)
   └ LogManage  : flash.init() + fs.init()(预留 1 槽) ─ 命令循环(单线程)
```

LED：`PF3` = 心跳(飞行/调试每 500ms 翻一次；管理模式慢闪；等命令时 120ms 闪)，
`PF2` = flash 擦除进度(低电平点亮，每擦完一个 64KB 块翻一次)。

---

## 6. EIDE 里的切换

发布项在 `.eide/eide.yml` 的 `USER` 组里是 `release.cpp`、`flight_config_struct.hpp`、
`release_log_manage.hpp`，并且把 `main_flash_fs.cpp` 设成了"排除编译"，所以工程默认构建发布版。

**只需要 `release.cpp` 一个编译单元**：`release_log_manage.hpp` 是头文件实现（inline 函数），
被 `release.cpp` include 就一定编得进去 —— 这样就不会再出现"漏加 .cpp → 链接期报
`L6218E: Undefined symbol dlx::runLogManage(dlx::ReleaseContext&)`"这类问题。
自检方法（不依赖 EIDE）：编译 `release.cpp` 后用 `arm-none-eabi-nm -C release.o | grep runLogManage`
应当能看到 `W dlx::runLogManage(dlx::ReleaseContext&)`。

想回到 flash 文件系统自测：把 `release.cpp` 设为排除、`main_flash_fs.cpp` 取消排除即可
(右键 → Exclude/Include；EIDE 若开着工程，改完 yml 需要重新加载工程)。

---

## 7. 主机端自检(不用板子)

| 脚本 | 检查内容 |
| --- | --- |
| `test/release_check/run.ps1` | 日志结构体布局、4 条地面报文 + `FlightLog` 的打包(帧长/类型号/CRC4/字段回读)、`check()`+回调、`DLXCheckResult` 五种返回(成功/半帧/非法帧头/未知类型/CRC 错)、旧版 UnBlock 兼容(39 项) |
| `test/flashfs_sim/run.ps1` | FlashManager 全部逻辑(9 千多项) |
| `test/flashfs_sim/main_check/run.ps1` | `main_flash_fs.cpp` 的干跑(48 项) |

硬件相关的(SPI/I2C 速度、DShot 时序、线程优先级、跳线检测的电气行为)**只能上板验证**。

---

## 8. 控制环压力预估与栈/堆预算

### 8.1 控制环(每 2ms)压力

参考: `main_att_ekf.cpp` 实测那个 2ms 循环平均 **466us**(含 IMU FIFO 读取 + Madgwick + EKF 快/慢路径
+ 20ms 一次的气压计读数, 不含控制/通信/日志与末尾 DMA)。

发布版在这个基础上的变化:

| 项 | 变化 | 量级 |
| --- | --- | --- |
| Madgwick | 去掉(只留 EKF) | −5us / 2ms |
| 控制器 + 混控 + DShot 预置 | 新增 | +4~8us / 2ms |
| 日志打包 + 入队(每 20ms 一次, 摊到每 2ms) | 新增 | +0.6us / 2ms |
| 气压计 I2C 100kHz(每 20ms 一次, 摊到每 2ms) | 与参考的 400kHz 不同 | +100us / 2ms |

结论(预估, 以 168MHz 计):

- **平均占用 ≈ 550~600us / 2ms ≈ 28~30%**;
- **最坏那个 slot(气压计读数落在本周期)≈ 1.4~1.5ms / 2ms ≈ 73%** —— I2C 走 100kHz 时一次
  BME280 读(寄存器地址 + 8 字节)约 1ms, 是单周期里最大的一块;
- 若把 `kConfig.hw.bmeSpeed` 改成 **400000**(与 main_att_ekf 实测一致), 最坏 slot 降到 **≈0.7ms**,
  平均 ≈470us; 前提是接线短、上拉够, 读数稳定;
  ⚠️ **2026-09-13 的排障结论(重要)**: 当时"飞行模式下 BME280 必失败、调试模式永远正常"
  并不是速率问题 —— 真因是 **BME280 的 SDO(地址选择)悬空**: 地址在 0x76/0x77 之间翻,
  而 flash 开机那段擦写的干扰正好会把它激翻。详见 §8.4。
  **默认值 100000 是"稳妥值"不是"必需值"**: 把 SDO 钉死到 GND(或 MCU 拉低)之后,
  400k 也可以再试(调试模式下 400k 一直是稳定的)。
- 需要更多余量时: 把 `kConfig.ekfDecim` 8 → 12/16(协方差传播 250Hz → 167/125Hz),
  EKF 慢路径是耗时大头, 线性见效。

实测手段已经内建: 串口调试模式每 200ms 一条 `PerfMonitor`(主循环 avg/max、EKF、控制、日志、IO),
飞行日志里每条也带 `loopPeriodUs`, 写满/异常另有状态位。

### 8.2 线程栈(静态分析, GCC -O3 + `-fstack-usage -fcallgraph-info=su`)

最坏调用链(静态, 每层帧大小):

| 线程 | 最坏链 | 需求 | 配置 | 判断 |
| --- | --- | --- | --- | --- |
| main(控制环) | `main` 3144 + `runFlightStack` 8832 + `kalmanUpdate` 6872 + `injectError` 136 + `quatMul` 48 | **≈19.0KB** | 24,576B | 24KB 够用(留 5KB 余量); 13,312B(旧值)**不够** |
| nrf_tel | `nrfTelThreadEntry` 152 + `check()` 48 + 读缓冲 24 | ≈224B + 中断嵌套 | 3,072B | 很宽裕(可降到 1.5~2KB) |
| logger | `logThreadEntry` 448 + `delay_ticks` 24 | ≈472B + 中断嵌套 | 3,072B | 够用 |
| 中断/MSP | 启动文件 `Stack_Size = 0x8000` | 中断帧(含 FPU 惰性栈 ~104B) + 处理函数几百字节 | 32KB | 非常宽裕 |

> 静态分析偏保守(它把编译器可能并存的临时量都算上), **必须上板用栈水位线量一次**:
> 发布版已在 main/nrf_tel/logger 三个线程里打了 0xA5 水位线, 串口调试模式下每 10s 打一行
> `stack used: main=x/24576 nrf=y/3072 logger=z/3072`。

### 8.3 RT-Thread 堆(重要)

`rtconfig.h` 里定义了 `RT_USING_HEAP`, 所以 `rt_application_init()` 是用
`rt_thread_create()` 建 main 线程的 —— **三个线程栈都从 RT-Thread 堆里分配**:

```
main 24KB + nrf_tel 3KB + logger 3KB + 信号量/定时器对象 ≈1KB ≈ 31KB
```

`board.c` 里 `RT_HEAP_SIZE` 原是 **16KB**, 连原来的 13,312B main 栈都放不下
(13,312 + 3,072 + 3,072 = 19,456 > 16,384), 后建的线程会返回 `RT_NULL`。
现已改为 **40KB**, 并在 `release.cpp` 里加了保护: 线程建不起来就**拒绝解锁**
(只允许地面观察), 同时把失败原因打到串口。

片内 RAM 128KB 的账: 中断栈 32KB + 启动堆 8KB(未使用, 可缩到 1KB) + RT-Thread 堆 40KB
+ 数据/bss ≈4KB ≈ **84KB**, 还有约 44KB 余量。

### 8.4 「飞行模式没信号 / 开机卡死」的排障记录(2026-09-13)

**现象**: 飞行模式下心跳灯定住(常亮或常灭)、NRF 无遥测、串口几乎没有输出; 同一份固件
切到串口调试模式一切正常, 连 BME280 开 400k 都稳。

**真因(三层叠加)**:

1. **硬件 strap 悬空**: BME280 的 `SDO`(地址选择)接在 PD2 上但没接地/没驱动, 电平会漂。
   BME280 的从机地址是**每次 I2C 事务实时采样 SDO** 的(不是上电锁存), 所以地址会在
   0x76/0x77 之间**中途翻转**;
2. **flash 开机初始化是触发器**: 飞行/管理模式要先 `fs.init()`, 那段片内擦写的干扰一来,
   SDO 就翻 ⇒ 原地址不 ACK。调试模式不碰 flash, 所以永远正常;
3. **老 I2C 驱动无超时死等**: `waitEvent()` 是 `while (I2C_CheckEvent(...) != SUCCESS);`,
   从机不应答就**永久卡死**在开机里 —— 于是一个"气压计地址翻了"的小问题升级成"整机起不来"。

**定位手法(值得复用)**:

- **时间点探针**: 在 flash init **之前/之后**各裸读一次 ChipID ⇒ 一眼看出是"被哪一段弄掉的";
- **寄存器 dump**: 超时时打 `SR1/SR2` 并解成 `BUSY/AF/BERR/ARLO/PE`。这次是
  `AF=1`(从机不 ACK) + `BUSY=0`(总线没被拉死) ⇒ 直接排除"总线卡死", 指向"器件不在";
- **双地址探针**: 原地址失败时自动试 0x77 —— 一击命中(`0x77` 答 `0x60`), 证明地址翻了;
- **心跳 + 计数器**: 控制环每 200ms 打一行"圈数/队列/写 flash 条数/NRF 收发/当前段",
  用来分清"哪个线程停了、停在哪一步"。

**修复(全部保留在发布版里)**:

| 位置 | 修复 |
| --- | --- |
| `main()` | PD2(SDO) 开漏输出拉低 ⇒ 地址固定 0x76(硬件再接地更保险) |
| `dlx_iic` | `waitEvent()` 加轮询上限, 超时不再死等; 出错后本次传输直接作废 |
| `dlx_iic` | `IICDevice::write/read` 返回 `bool`; 上次出错则先做**总线恢复**(9 个 SCL 脉冲 + 手工 STOP) |
| `dlx_bme280` | `init()`/`readRegs` 传播失败; 读失败时输出清零(避免栈上旧值被当成有效数据) |
| `release.cpp` | 气压计初始化**自适应重试**(健康机器零等待); 运行时连续读失败 ⇒ `baroHealthy=false` ⇒ 遥测 `height` 字段语义切回高度、`error` 置 `kTelErrBaro`, 读成功即恢复 |
| `release.cpp` | `getRaw(&ok)`: 读失败就跳过这次观测, 不把 0Pa(→44330m)喂进 EKF |

**经验**: 等硬件标志的循环一律要有上限; 硬件 strap 脚必须钉死电平; "只有某个模式出问题"
时先找两种模式**真正不同的那几行代码**, 再用时间点探针把窗口夹出来, 别猜玄学。

### 8.5 一键禁用气压计: `DLX_BARO_ENABLED`(2026-09-13)

排障或对比试验时(例如怀疑"气压计把高度估计带偏"), 不必再去 `release.cpp` 里注释代码 ——
翻一个开关就行:

| 位置 | 开关 | 效果 |
| --- | --- | --- |
| `flight_config_struct.hpp` 顶部「零、编译期总开关」 | `DLX_BARO_ENABLED`(默认 **1**) | 置 **0** = 编译期裁掉整个气压计 |

置 0 之后发生的事(**全是编译期**, 不是运行时跳过, 引脚和 EKF 里都不留残渣):

- **不初始化**: 不碰 I2C1(PB6/PB7)、不驱动 SDO 地址脚(PD2)、不做初始化重试
  ⇒ 顺带省掉开机最多 2s 的重试等待(见 §8.4);
- **不喂 EKF**: 高度/垂速只剩纯 IMU 积分(会缓慢漂移), `vertical_velocity_valid` 恒 `false` ⇒ 定高不可用;
- **遥测/日志**: 遥控回传的 `height` 字段退回"**高度**"语义(`kTelStatusBaroOk` 恒 0), `kLogFlagBaroOk` 恒 0;
- **不报故障**: 主动关掉 ⇒ 不再置 `kTelErrBaro`, 免得上位机把"关了"显示成"气压计坏了";
- 开机那行 `[Boot] BMI088=OK BME280=OFF ...` 一眼能看出这份固件是关着气压计跑的。

想不动代码就做 A/B(开着 / 关着各飞一次), 在编译器全局宏里加 `DLX_BARO_ENABLED=0` 覆盖本文件
(Keil: Options for Target → C/C++ → Define; EIDE: `.eide/eide.yml` 的 `defineList`)。

对应的上板勾选项见 `docs/release_test_checklist.md` §B。
