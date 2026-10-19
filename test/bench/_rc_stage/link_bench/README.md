# 遥控器端链路的 P0 台架固件（暂存目录）

这是**独立 ESP-IDF 工程**，不属于主工程：不加进 `controller/main/CMakeLists.txt`，
`EXTRA_COMPONENT_DIRS` 指回主工程的 `components/`，因此 E220 驱动与引脚定义只有一份。

> 由于本会话的沙箱不允许写 `E:\ESP_PROJECT\`，工程先写在这里；
> 同步清单与复制命令见同级目录的 [`SYNC.md`](../SYNC.md)。

## 它做什么

- 以 `components/radio` 的原样驱动初始化 E220（**只读寄存器**，不调 `radio_e220_boot_ensure()` /
  `radio_e220_apply_profile()`，因此**不会写模块 flash**）。
- 没有 USB 日志（1W 模块与 USB 共用供电有风险）：所有结果用 ASCII 行经 E220 回传，
  由飞控 bench 打印。
- 三种被测模式 + 回传统计：

| 模式 | 触发 | 行为 |
|---|---|---|
| `SILENT` | `@MODE SILENT` | 不发测量帧，只响应命令行 |
| `RESPOND` | `@MODE RESPOND len=8 guard=0` | 收到一帧后等 `guard` ms 回发 `len` 字节 |
| `PATTERN` | `@MODE PATTERN len=8 gap=22` | 每 `gap` ms 主动发 `len` 字节，记录定拍抖动 |

其它命令：`@STAT`（回一行统计）、`@PING`（回 `#PONG`）、`@RESET`（清统计）。

回传行（以 `#` 开头）：

```
#BOOT fw=2 role=rc_bench baud=57600 rate=7 ch=23 aux=1 reg=...   ← 上电一行（寄存器只读快照）
#ACK  mode=.. len=.. gap=.. guard=..
#STAT mode=.. len=.. gap=.. guard=.. tx=.. rx=.. txbusy=.. jmin=../javg=../jmax=../jp99=.. auxbusy=.. uptime_ms=..
#PONG
```

`jmin/javg/jmax/jp99` 是 `PATTERN` 模式下"实际发送时刻 − 计划时刻"（µs）。

## 构建 / 烧录

```powershell
# 1) 加载 ESP-IDF 环境（主工程自带脚本）
. E:\ESP_PROJECT\controller\tools\idf_env.ps1
# 2) 在本目录（同步后是 controller\bench\link_bench）
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

## 与飞控 bench 的约定

- 串口/空速/信道/分包长度**不由本工程设置**：模块 flash 里是主工程烧进去的
  （57600 8N1 / 62.5k / CH23 / 32B 分包 / UAUX=1 / URXT=3）。
  ⇒ 全新模块先烧一次主工程固件，再烧这个 bench。
- 命令与回传共用同一条 E220 链路；测量时两者会混在飞控 bench 的 `rx` 打印里，靠首字符区分：
  `@` = 命令、`#` = 回传、其它 = 测量载荷（0xA0+i 台阶字节）。
- 引脚以 `components/rc_core/include/app_config.h` 为准（M0=GPIO46、M1=GPIO21、AUX=GPIO39、
  TX=GPIO17、RX=GPIO16）。若 M0 那一路被外部拽住，先按该文件 `E220_M0M1_EXTERNAL` 的说明排查。
