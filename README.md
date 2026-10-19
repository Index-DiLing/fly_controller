# Fly Controller

STM32F407 飞控固件。当前构建入口为 **EIDE / FLY / ARMCLANG 6**，应用入口为 `release.cpp`。

## 目录

| 路径 | 职责 |
| --- | --- |
| `app/boot/` | 启动模式、设备初始化和诊断报告 |
| `app/control/` | 控制主流程、系统状态及起降操作逻辑 |
| `app/channels/` | 请求、故障、日志渠道及跨线程发布对象 |
| `app/link/` | 无线线程、命令映射及状态编码 |
| `app/log/` | 运行期日志线程及启动期日志管理模式 |
| `app/sensors/` | 采集线程、样本及带时间戳的健康状态 |
| `app/runtime/` | 异常入口、公共上下文、运行常量及栈诊断 |
| `DL_LIB/` | 设备驱动、控制算法和通用容器 |
| `rt-thread/` | RT-Thread 内核及板级启动代码 |
| `test/` | 主机侧回归测试与 CMSIS 替身 |
| `tools/` | 构建核对工具 |
| `docs/` | 当前运行期契约 |
| `archive/` | 旧 Keil 工程、历史报告和源码快照，不参与当前构建 |

硬件接线断言保留在根目录 `hardware_claims.hpp`；中断与芯片初始化文件也保留在根目录。
应用文件保留原文件名，便于检索历史；头文件统一使用相对工程根目录的 include 路径。
未将可复用但当前未启用的 DL_LIB 驱动当成死文件清理。

## 构建

1. 打开 `dl_fly.code-workspace`，在 EIDE 中选择 `FLY` 目标并执行 Rebuild。
2. EIDE 扫描 `app/`、`DL_LIB/` 和 `rt-thread/`；根目录入口及芯片文件由虚拟文件组引用。
3. 编译数据库在 `build/FLY/compile_commands.json`，固件输出为 `build/FLY/dl_fly.hex`。

`.eide/`、`.pack/`、`.cmsis/`、`RTE/` 及 `RTE_Components.h` 是当前本地 EIDE 环境的一部分，不因旧 Keil 工程归档而移动。

需要核对编译数据库中的参数及链接时，可在工程根目录执行：

```powershell
powershell -ExecutionPolicy Bypass -File tools/build_check.ps1 -Link
```

核对脚本会检查 `app/` 下的 `.cpp` 是否全部出现在编译数据库中；若提示漏编，先确认本地 `.eide/eide.yml` 的 `srcDirs` 包含 `app`，再从 EIDE 执行 Rebuild。仅重放旧编译数据库不会自动纳入迁移后的源码。

在编辑器外修改 `.eide/eide.yml` 后，应先在 VS Code 执行 `Developer: Reload Window` 重新加载 EIDE 工程，避免内存中的旧配置覆盖磁盘改动。

## 主机测试

在工程根目录执行，要求 C++11 编译器；下例使用 PowerShell 和 PATH 中的 g++，测试程序只输出到临时目录。

```powershell
$tests = @('state_step_test', 'runtime_channel_test', 'dlx_queue_test',
           'dlx_mpsc_queue_test', 'dlx_mailbox_test', 'dlx_locked_cell_test',
           'up_motor_frame_test', 'sensor_measurement_test',
           'log_mailbox_test', 'bmp388_read_test',
           'bmp388_error_provenance_test', 'iic_error_recovery_test')
foreach ($test in $tests) {
    $extra = @()
    $flags = @()
    $includes = @('-I', 'test/stub', '-I', 'DL_LIB', '-I', '.')
    if ($test -eq 'state_step_test' -or $test -eq 'bmp388_read_test') {
        $extra = @('app/sensors/app_health.cpp')
    }
    if ($test -eq 'bmp388_read_test' -or $test -eq 'bmp388_error_provenance_test') {
        $includes = @('-I', 'test/bmp388_stub') + $includes
    }
    if ($test -eq 'iic_error_recovery_test') {
        # 真实的 DL_LIB/dlx_iic.hpp + 寄存器级替身 test/iic_stub;
        # -Wno-cpp 只用于屏蔽 DL_LIB 里既有的两个 `#warning [Experimental]`
        # (DL_LIB/dlx_gpio.hpp 与 DL_LIB/dlx_iic.hpp 里的 `#warning [Experimental]`), 测试自身代码零警告。
        $includes = @('-I', 'test/iic_stub', '-I', 'DL_LIB', '-I', '.')
        $flags    = @('-Wno-cpp')
    }
    $output = Join-Path $env:TEMP ($test + '.exe')
    g++ -std=c++11 -Wall -Wextra @flags @includes "test/$test.cpp" @extra -o $output
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $test" }
    & $output
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $test" }
}

# 链路 v2 帧层 / 轮次从机(协议规格 §10 黄金向量 + RoundSlave 状态机; 路径较深, 单独跑)
foreach ($test in @('test/link_v2/fc/frame_test', 'test/link_v2/fc/round_test')) {
    $output = Join-Path $env:TEMP ((Split-Path $test -Leaf) + '.exe')
    g++ -std=c++11 -Wall -Wextra -I test/stub -I DL_LIB -I . "$test.cpp" -o $output
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $test" }
    & $output
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $test" }
}
```

`iic_error_recovery_test` 是**硬回归门**: 它用真实的 `DL_LIB/dlx_iic.hpp` 配寄存器级替身 `test/iic_stub/`, 断言
(1) 错误标志(AF/BERR/ARLO/OVR)置位时 `waitEvent` 立即放弃 —— 一次事务的 `I2C_CheckEvent` 调用必须 <200(不是 ~200000);
(2) 下一次事务开头的 `IICBus::recover()` 恰好重跑一次 `I2C_Init`、恰好清一次错误标志, 之后 SR1 错误位为 0 且同一次读成功。
删掉 `recover()` 里的 `I2C_Init`/清标志、或删掉 `waitEvent` 的提前放弃, 该测试都会以 exit 1 失败。
`bmp388_error_provenance_test` 用假 IIC 驱动真实 `DL_LIB/BMP388/dlx_bmp388.hpp`,
证明 0x404 既可来自 I2C 事务失败(`failCount()==1`), 也可来自事务成功但补偿结果非法(`failCount()==0`)。

## 阅读入口

- [运行期通道与所有权契约](docs/runtime_channels.md)
- [归档索引](archive/README.md)

历史架构图、符号表和旧重构计划已经归档，不应视为当前源码的精确描述。
