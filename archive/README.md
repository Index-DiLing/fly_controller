# 归档索引

2026-10-16 工作区整理：以下内容仅移动归档，没有删除，也没有改写归档文件内容。

| 目录 | 内容 | 当前状态 |
| --- | --- | --- |
| `legacy_keil/` | `dl_fly.uvprojx`、`dl_fly.uvoptx`、`dl_fly.uvguix.30407`、`EventRecorderStub.scvd` 及旧 `Objects/` | 旧 Keil 工程与输出，不参与 EIDE 构建 |
| `snapshots/` | `code.zip` | 历史源码快照，不作为活动源码解压或编译 |
| `notes/` | `update-v2.2.txt` | 历史更新记录 |
| `docs/` | 2026-10-11、2026-10-13 的重构计划、架构 HTML、符号表 HTML 及 `_symbols/` 原始表和生成器 | 保留当时的架构描述，不再跟随当前代码同步 |

## 使用边界

- 当前构建入口是工程根目录 `.eide/eide.yml`，不是旧 `.uvprojx`。
- 历史资料中的文件名、行号及根目录相对路径属于归档时的旧结构，不能当作当前代码索引。
- 旧 Keil 工程本身已过时，包含失效的历史入口；归档不代表该工程现在可独立构建。
- `legacy_keil/Objects/` 和 ZIP 快照保持本地归档，不作为新构建产物纳入版本控制。
- 当前仍使用的 `.pack/`、`.cmsis/`、`RTE/`、`.eide/` 没有归档。
- 应用层新位置见根目录 [README](../README.md)，当前通信契约见 [runtime_channels](../docs/runtime_channels.md)。
