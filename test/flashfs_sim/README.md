# FlashManager 主机端逻辑测试

在电脑上用一个 **模拟的 W25Q128**(16MB, 支持"写一半掉电"注入)跑 `DL_LIB/W25Q128/dlx_flash_manager.hpp`
的全部逻辑, 不需要板子、不需要 EIDE。改完 FlashManager 或 `LogEntry` 结构体后建议重跑一次。

## 怎么跑

```powershell
powershell -ExecutionPolicy Bypass -File .\test\flashfs_sim\run.ps1
```

需要 PATH 里有桌面版 `g++`(MinGW-w64, 本机在 `E:\MINGW\bin`)。脚本会把
`DL_LIB` 下的 4 个源文件复制到 `_build\`, 和这里的模拟驱动一起编译运行, 输出形如:

```
===== 检查 22523 项, 失败 0 项 =====
```

## 文件

| 文件 | 说明 |
| --- | --- |
| `dlx_w25q128.hpp` | 模拟 Flash 驱动(只实现 FlashManager 用到的接口), **编译时才替换真实驱动** |
| `test_main.cpp` | 测试用例 |
| `run.ps1` | 复制源码 -> 编译 -> 运行的脚本 |
| `_build\` | 生成物(可随时删) |

## 覆盖到的场景

1. 空芯片首次初始化: 整片擦除 + 新建文件系统(返回 `FLASH_SYSTEM_CREATED`)、进度回调次数
2. 多次上电 = 多个会话, 会话号递增
3. 写日志 + 按会话回读(整条/按序号/遍历/计数)
4. 持久化参数: 保存、读回、写满一个扇区后擦除轮转、`peekParams()` 免 init 探测
5. Halt 策略写满预留区 -> `FLASH_LOG_FULL`; 切 OverwriteOldest 后继续写(跨多个槽)
6. 掉电写一半: 半条记录被丢弃, 之前的数据完整
7. 数据损坏: 翻转 1 个 bit -> 那一条之后不再返回, 之前仍可读
8. 布局指纹不匹配(改了 `LogEntry` 大小等) -> `FLASH_GEOMETRY_MISMATCH`
9. 130 次上电把日志槽环绕一遍: 只保留最近 127 个槽的数据
10. 显式 `format()`
11. 上电擦除量: 上次用 1 个槽 -> 本次只擦 1 个槽(2 个 64KB 块), 不是整块预留区
12. `dropSessions(n)` 丢掉**最老的** n 个会话(环形队列的队头)、`clearHistory()` 清空历史:
    写指针不动、当前会话不被重开, 腾出的槽并入当前会话预留区 -> 上限变大还能接着写
13. 丢掉一个跨多个槽的最老会话(按它实际占用的槽数擦除), 之后继续写 + 下一次上电起点仍算得对

## 改了 `LogEntry` 之后

`FLASH_LOG_ENTRIES_PER_SLOT` / 单会话上限会自动跟着变; 如果结构体大小变了, 芯片上的旧数据会被
判为 `FLASH_GEOMETRY_MISMATCH`, 需要 `format()` 重建(测试里第 8 条就是验证这个行为)。
