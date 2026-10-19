# 遥控器 bench 工程：暂存副本（与 RC 仓库同步）

> **状态（2026-10-18）：RC 仓库版本为准，已由 rc-link-dev 同步进 RC 仓库并 `idf.py build` 通过**
> （`link_bench.bin` = 0x35760 字节）。本目录是**回同步后的副本**，用于留档与审阅；
> 内容与 RC 仓库 `bench/link_bench/` 一致。

## 为什么会有这个暂存目录

写遥控器 bench 的会话（radio-phy-recon）权限固定在 FC 工作区（workspace-write），
无法写 `E:\ESP_PROJECT\controller\`。因此工程先完整写在这里，由 Lead / rc-link-dev 同步进 RC 仓库，
再把 RC 仓库里"编过之后的最终版"回同步到这里。

**因此：不要再从本目录覆盖 RC 仓库。** 要改遥控器 bench，先改 staging（或直接在 RC 仓库改并回同步），
两边必须逐字节一致；以 RC 仓库版本为准（它能被 `idf.py build` 验证）。

## 1. 路径映射（源 → 目标）

| 源（本仓库，回同步副本） | 目标（遥控器仓库，权威版本） |
|---|---|
| `test/bench/_rc_stage/link_bench/CMakeLists.txt` | `E:\ESP_PROJECT\controller\bench\link_bench\CMakeLists.txt` |
| `test/bench/_rc_stage/link_bench/sdkconfig.defaults` | `E:\ESP_PROJECT\controller\bench\link_bench\sdkconfig.defaults` |
| `test/bench/_rc_stage/link_bench/README.md` | `E:\ESP_PROJECT\controller\bench\link_bench\README.md` |
| `test/bench/_rc_stage/link_bench/main/CMakeLists.txt` | `E:\ESP_PROJECT\controller\bench\link_bench\main\CMakeLists.txt` |
| `test/bench/_rc_stage/link_bench/main/link_bench_main.c` | `E:\ESP_PROJECT\controller\bench\link_bench\main\link_bench_main.c` |

不需要同步（RC 侧本地构建产物）：`build/`、`sdkconfig`、`sdkconfig.old`、`managed_components/`、
`sdkconfig.ci`。

## 2. 让它编过所必需的**两处修改**（已包含在两边版本里，别改回去）

### 2.1 `EXTRA_COMPONENT_DIRS` 只列 `radio` + `rc_core`

**不能**写成 `../../components`（整棵组件树）。原因：整棵树会把 `ui`/`bsp` 拉进构建，而它们依赖
主工程的 `managed_components`（`espressif__esp_lvgl_port` / `lvgl__lvgl`，位于
`controller/managed_components`，bench 工程里不存在），cmake 直接以
`component 'espressif__esp_lvgl_port' could not be found` 失败。

bench 只需要两个组件：

```cmake
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components/radio"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/rc_core")
```

### 2.2 `qsort` 比较器返回 `int`（不是 `int32_t`）

ESP-IDF v6.1 用 `-std=gnu23` + picolibc，`qsort` 的比较器签名要求 `int (*)(const void*, const void*)`。
返回 `int32_t` 在 gnu23 下会因函数指针类型不兼容而报错（实测：`-Wincompatible-pointer-types` 升为错误）。
现在：

```c
static int cmp_i32(const void *a, const void *b)
{
    const int32_t x = *(const int32_t *)a;
    const int32_t y = *(const int32_t *)b;
    return (x < y) ? -1 : (x > y) ? 1 : 0;
}
```

## 3. 构建 / 烧录（RC 侧，权威流程）

```powershell
. E:\ESP_PROJECT\controller\tools\idf_env.ps1
cd E:\ESP_PROJECT\controller\bench\link_bench
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash
```

已确认结果：构建成功，`link_bench.bin` = **0x35760** 字节（≈214 KB）。

## 4. 已知不确定点（要 owner / RC 队员确认）

- **M0/M1 接线与极性**：`app_config.h` 的 `E220_M0M1_EXTERNAL` 默认 0（固件驱动 M0=46/M1=21）。
  现场若 M0 被外部拽低，表现为"AT 全无响应"（该文件有专门说明）。bench 不改这些宏。
- **AUX 极性**：按驱动合同"高=空闲/低=忙"（`radio_e220.h`、模块手册 §5.2.4）。若 AUX 悬空/接反，
  `#BOOT ... aux=` 会恒 0，`txbusy` 会涨；`RESPOND/PATTERN` 仍能发但测量值不可信。
- **供电**：T30S 发射瞬时 >600mA，必须单独供电 + 大电容；否则"AT 通、一发射就复位"
  （`app_config.h` 已记录该现场）。
- **不要同时插 USB 与外供电**：所以本 bench 不依赖 USB 日志，结果全部经 E220 回传。
