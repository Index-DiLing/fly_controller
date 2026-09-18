# 发布版上板测试清单(release.cpp + flight_config_struct.hpp + release_log_manage.hpp)

配套: 总览图 `docs/release_overview.png` ｜ 说明 `notebook/2026-09-11_release_item.md` ｜ 协议 `notebook/2026-09-11_dlx_protocol.md`

用法: 打印或直接在编辑器里勾选;每组末尾的"实测"栏用来填数,跑完一轮把表头三行(负载/栈/会话)抄下来就是这次发布的记录。

## 0. 前置条件与安全

- [ ] **拆桨**(除了 C 组的电机方向测试,任何一项都不允许带桨)
- [ ] 电机测试用**限流电源**(例如 12V/2A)或电池 + 串联限流灯,先只上电不转
- [ ] 跳线帽跳线( PB8–PB9 短接,或两脚各自接地)、USART1 接上位机(2Mbaud)、Flash/BME280/BMI088/NRF 都在位
- [ ] 上位机工具就绪:能发 `GroundCmd`/`UnBlock`、能解析 `FlightLog`/`SessionInfo`/`PerfMonitor`/`GroundReply`
- [ ] 升级前**先把机上的旧日志导出**(改过 `LogEntry` 布局时会要求整片格式化,数据会丢)
- [ ] 记录本轮固件版本(`FLIGHT_CONFIG_VERSION`)、Git commit、电池电压

---

## A. 上电与模式判定(跳线帽 PB8 & PB9)

- [ ] 不接跳线帽上电 → 串口**全程静音**、PF3 每 500ms 心跳;不解锁时电机不出任何信号 → 期望: 无输出、无命令
- [ ] 接跳线帽上电 → PF3 120ms 闪、串口打印 `[Boot] 检测到地面模式跳线…等待上位机解锁命令`
- [ ] 发 `UnBlock(param=2)` 或 `GroundCmd(op=1,mode=2)` → 回 `GroundReply(op=1,status=0)`,进日志管理(PF3 慢闪)
- [ ] 管理模式发 `GroundCmd(op=1,mode=1)` → 回 OK,切到串口调试(出现 `FlightLog` + `PerfMonitor` 流)
- [ ] 调试模式发 `GroundCmd(op=1,mode=0)` → 按 32 槽重开会话后进飞行模式(日志转写 flash、串口静音)
- [ ] 接跳线帽上电后**不**发命令 → 一直等(默认超时 0),不会自己飞起来
- [ ] 拔跳线帽复位 → 直接进飞行模式(不依赖串口) → 期望: 上电即待命,可解锁

## B. 传感器与 EKF(姿态 / 气压计)

- [ ] 静止台架跑 1 分钟 → `PerfMonitor` 的 `ekfCycle` 稳定;`loopMaxCycle` 不出现异常尖峰(门槛见 F 组)
- [ ] 手动抬机头(绕横轴) → 日志里 pitch 变化方向正确;左右滚 → roll 方向正确(否则改 `kConfig.imuYawDeg` 试 90/270)
- [ ] 静止时 `euler` 漂移 → 1 分钟内 <2°;陀螺零偏 `gyroBias` 收敛不发散
- [ ] 举高 1m 再放下 → `heightM` 跟随(±0.3m),`vertVelMps` 符号正确(升正降负)
- [ ] 捂住/拔掉 BME280 → `kLogFlagBaroOk` 变 0,飞控不崩;遥控回传的 Height 字段退回高度语义
- [ ] 编一版 `DLX_BARO_ENABLED=0`(见 notebook §8.5) → 开机日志 `BME280=OFF`、能飞、`kLogFlagBaroOk=0`、
      Height 退回高度语义、`error` **不**置 `kTelErrBaro`(主动关掉不算故障)
- [ ] 解锁瞬间 → `kLogEventEkfRezero` 置位,`baroRef` 与 EKF 高度归零,之后高度从 0 起算
- [ ] 拔掉 BMI088 复位 → 能进地面模式,状态位不带 `kLogFlagImuOk`,不允许进入控制

## C. 控制与执行器(台架,必须拆桨;方向测试才可短时带桨并限流)

- [ ] `command=1` 直接给四路电调值 → 电机编号符合 `kConfig.motorMap{0,2,3,1}`(逻辑 M0 左前 → 通道 0)
- [ ] `command=0` 自稳 + 手动油门 → 机身对抗方向正确(不反向);油门最小时输出不为 0(混控需要非零集体油门)
- [ ] `command=2` 解锁 → 3200ms 启动序列后 `enabledMotor` 置位;`command=3` 停机
- [ ] `command>3` 硬停机 → DShot/定时器立即停,且是**控制线程**执行的(遥控线程只置标志)
- [ ] 关掉发射机 >5s → `kLogEventFailsafe` 出现、电机停;`kLogFlagFailsafe` 之后保持置位
- [ ] 满舵/满油门 → `kLogFlagMotorSat` 出现,电机计数始终在 55~1945 内
- [ ] 打俯仰/横滚/偏航各一次 → 四路电机差动方向正确(对照 `release_overview.png` 的电机映射)

## D. 日志(飞行模式写 flash)

- [ ] 上电打印 `[Flash] init=… 预留 32 槽 上限=28320 条`,用时 ≈0.3s × 上次用掉的槽数
- [ ] 50Hz 跑 1 分钟 → 条目数 ≈3000(误差 <5%),`logEntries` 单调递增
- [ ] 把预留临时改成 1 槽(或长飞)写满 → `kLogFlagLogFull` 置位、`FLASH_LOG_FULL`,**飞行中零擦除**
- [ ] 掉电测试(飞行中拔电) → 下次上电该会话最后一条之前的记录完好;最后一条要么完整要么不存在
- [ ] 每条日志 `loopPeriodUs` 2000±50us;`linkAgeMs` 与实际遥控延迟相符
- [ ] `flags/events` 逐位对得上实际事件(解锁/停机/失控/饱和…);`gyroBias` 每 10 条更新一次

## E. 日志管理模式

- [ ] `op=2` 列会话 → `SessionInfo` 逐条出 + 末尾 `GroundReply(value=会话数)`;条目数与实际一致(大会话要等几秒)
- [ ] `op=4` 回传整会话 → `FlightLog`(142B/条)连续流出,末尾 `GroundReply(value=条数)`;上位机无坏帧
- [ ] `op=3` 删最老 N 个 → 只剩期望会话,当前会话不被重开,`value` 回报并入的槽数
- [ ] `op=5` 整片格式化 → PF2 每 64KB 翻一次(几十秒),结束后会话号 +1、预留 1 槽
- [ ] `op=6` Ping 秒回;拔电重启后管理模式仍可用(会话号继续递增)
- [ ] 管理模式下**电机全程无输出**(示波器/电调无声)

## F. 性能与栈(关键实测,决定要不要调参)

- [ ] 调试模式跑 5 分钟 → `PerfMonitor`: **loopAvg < 600us**、**loopMax < 1600us**
      - 超了先看 `ekfCycle`(EKF 慢路径);仍高就把 `kConfig.ekfDecim` 8 → 12/16
      - 若超出的尖峰只出现在气压计那个 slot: 把 `kConfig.hw.bmeSpeed` 100000 → 400000(需接线短、上拉足)
- [ ] 看每 10s 的 `stack used:` → **main < 20KB**(配 24KB)、**nrf/logger < 1.5KB**(配 3KB)
      - main 接近 24KB: 把 `release.cpp` 里的大缓冲挪成 static,再复测
- [ ] 开机日志无 `线程创建失败(nrf=… logger=…)`(有就是 RT-Thread 堆不够,查 `board.c` 的 `RT_HEAP_SIZE`)
- [ ] 连续跑 ≥10 分钟无 HardFault/看门狗复位(栈溢出会先砸相邻堆内存,表现是随机跑飞)

## G. 异常与恢复

- [ ] 拔掉 W25Q128 → 仍能解锁飞行;`kLogFlagLogError` 置位,遥控 `error` 低位带 1
- [ ] 拔掉 BME280 → 能飞,`kLogFlagBaroOk=0`,Height 字段退回高度
- [ ] 拔掉 BMI088 → 不进入控制(状态位不带 `kLogFlagImuOk`)
- [ ] 改一次 `LogEntry` 字段后烧录 → 报 `FLASH_GEOMETRY_MISMATCH`,格式化后恢复(或 `kConfig.autoFormatOnMismatch=true` 自动重建)
- [ ] 每种异常恢复后复位一次都能回到正常流程(不留半初始化状态)

---

## 实测记录表(填完随发布一起存档)

| 记录项 | 期望/门槛 | 实测 | 结论 |
| --- | --- | --- | --- |
| 固件版本 / commit | `FLIGHT_CONFIG_VERSION=1` / ______ | | |
| 上电擦除耗时 | ≈0.3s/槽(默认 32 槽预留) | | |
| 控制环 loopAvg | < 600us | | |
| 控制环 loopMax | < 1600us | | |
| 其中 EKF(ekfCycle) | ______ | | |
| 日志写入(logCycle) | ≈0.23ms/条 | | |
| main 栈水位 | < 20KB / 24KB | | |
| nrf / logger 栈水位 | < 1.5KB / 3KB | | |
| 50Hz 1 分钟条数 | ≈3000 | | |
| 单会话上限 | 28320 条 ≈9.4 分钟 | | |
| 姿态静止漂移(1min) | < 2° | | |
| 高度跟随(举高 1m) | ±0.3m | | |
| 失控保护 | 5s 后停机 | | |
| 会话列表/回传/删除/格式化 | 全部 OK,无坏帧 | | |

## 失败时的常见处理

| 现象 | 处理 |
| --- | --- |
| `loopMax` 逼近/超过 2ms | `kConfig.hw.bmeSpeed` 改 400000;还不行 `ekfDecim` 8→12/16 |
| main 栈水位逼近 24KB | 大缓冲改 static;或 `RT_MAIN_THREAD_STACK_SIZE` 再加大并同步加大 `RT_HEAP_SIZE` |
| 线程创建失败 / 随机跑飞 | `board.c` 的 `RT_HEAP_SIZE` 加大(当前 40KB);确认不是别的 malloc 吃光 |
| 报 `FLASH_GEOMETRY_MISMATCH` | 接跳线帽 → 日志管理模式 → `op=5` 整片格式化 |
| 日志写满 | 地面 `op=3` 删最老会话 / `clearHistory`,或把 `kConfig.flightLogSlots` 调大 |
| 姿态方向不对 | `kConfig.imuYawDeg` 试 90/270;电机方向对照 `motorMap` |
