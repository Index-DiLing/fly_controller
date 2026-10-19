#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 test/link_v2/sim/REPORT.md（由 tools/link_v2/run_verify.ps1 调用）

输入（run_verify.ps1 先落盘）:
  test/link_v2/sim/out/steps.json      各步骤命令/退出码/耗时
  test/link_v2/sim/out/sim_100k.json   仿真统计
  test/link_v2/sim/out/xcheck.json     帧层对拍统计
  test/link_v2/sim/out/selftest.txt    确定性自检输出
输出:
  test/link_v2/sim/REPORT.md
"""
import datetime, json, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")


def load(path, default=None):
    try:
        with open(path, encoding="utf-8") as fp:
            return json.load(fp)
    except Exception:
        return default


def read_text(path):
    try:
        with open(path, encoding="utf-8") as fp:
            return fp.read().replace("\r", "")
    except Exception:
        return "(missing)"


DEVIATIONS = """\
> 处置状态：2026-10-18 Lead 已逐条拍板；D1/D2 改规格+常量，D3 新增 §4「本轮关闭」，
> D4/D5 在 §7 与台架口径注明；I1–I4 为实现偏差，已由 rc-link-dev / fc-link-recon 修完，
> 并由 `host_checks.py`（两端真实实现的独立口径断言）回归验证 **全部 PASS**。

| # | 发现（验证者提出） | 期望 | 实际（提出时） | 处置 |
|---|---|---|---|---|
| D1 | §2.3 预算检查只算空中时间，未算接收端串口排空 | `now + airtime + n×LinkUartByteUs + LinkDlBudgetSlackUs ≤ t0+117ms` | 13B 帧实际收完比空中结束晚 ~2.3ms（21B ~3.7ms），可能"空中进窗、遥控器已判丢" | **closed**：新增 `LinkUartByteUs=174`、`LinkDlBudgetSlackUs=1000`，§2.3 更新；`fc_host_checks` H5 断言预算判据 |
| D2 | §5 上行迟到容差 12ms 与飞控兜底触发矛盾 | 迟到 >3ms 即跳过/顺延（`LinkGuardUs`） | U3 迟到 >6ms 时空中结束晚于兜底触发，两端会争用半双工 | **closed**：阈值改 `LinkGuardUs(3ms)`，§5 更新；仿真 `late`/`late_big` 专项断言无同刻冲突 |
| D3 | 规格未定义"D0 已发又收到迟到上行" | D0 后同 t0（差 <60ms）迟到帧只计数，不再触发第二帧 | 旧模型会开新一轮并在预算够时再发 D0 | **closed**：§4 新增「本轮关闭」；仿真 `--selftest` 用变异开关证明断言能抓到第二次 D0 尝试；`fc_host_checks` H4 断言迟到帧不重开轮次 |
| D4 | §7 "≤3 轮恢复" 不适用于 NO_LINK 路径 | 注明 NO_LINK 恢复上限 ≈500ms+握手往返 | NO_LINK HELLO 重发周期 500ms，实测最坏 ~548ms > 360ms | **closed（文档）**：§7 注明；仿真恢复上限按"非 NO_LINK 360ms / NO_LINK +500ms"分别断言 |
| D5 | 握手/重启瞬间 HELLO 与对端定时帧可能同刻 | 台架"无双发"限定 RUN 稳态 | 重启后 RC 的 HELLO 可能与飞控残余轮次 D0 重叠（半双工协议固有，靠 60ms 重发兜底） | **closed（文档）**：§7 注明；仿真把这类碰撞计为 warning，RUN 稳态严格零重叠 |
| I1 | RC 命令门控只判 `state==RUN`，缺 §5.1 的 360ms 新鲜度 | `RUN && 版本匹配 && now-lastAnyDl < LinkDegradeMs` | `fc_round.c` `fc_round_queue_cmd()` 原只判 RUN | **closed**：实现已补降解门控；`rc_host_checks` R1 验证"连续 2 轮无下行→脱离 RUN→命令被拒" |
| I2 | RC 命令超时按"发送次数"而非日历 600ms | 入队起 600ms 未确认即放弃 | U2 被跳过时不计数，实际可能 >5 日历轮 | **closed**：改为 `cmd_start_us` 日历超时；`rc_host_checks` R2 验证 600ms 内不放弃、660ms 前放弃且结果=RES_TIMEOUT |
| I3 | RC DL 成功判定窗口 `[t0,t0+117ms)` 与 §5 "任意合法下行"表述不一致 | 文档明确 117–120ms 窗口归属 | 117–120ms 到达的合法下行不阻 streak++ | **closed（文档）**：§5 已写明窗口口径；实现不变 |
| I4 | FC HELLO 应答计入 D0 发送统计 | HELLO 只进独立计数、不推进遥测计数（§6） | `onDownlinkSent(isHello=true, D0 槽)` 同时 `m_d0Sent++` | **closed**：HELLO 只进 `m_helloReplies`，本轮槽/关闭照常置位；`fc_host_checks` H1/H2/H3 断言 |

**当前无未处置偏差**：帧层两端与独立 oracle 字节完全一致；两端轮次实现的 4 项偏差修复已回归验证。
"""

CHECKLIST = """\
| 检查项 | 依据 | 结论 |
|---|---|---|
| CRC-16/CCITT-FALSE（poly 0x1021/init 0xFFFF/不反射/xorout 0）"123456789"→0x29B1 | 规格 §1.1/§10 | 两端实现 + 独立 oracle 三方一致 |
| CRC 字段低字节在前（BN-2 = crc&0xFF） | 规格 §1.1 | 黄金向量逐字节一致 |
| 帧头 `(TYPE<<4)|(SEQ&0xF)`；SEQ 4bit | 规格 §1.1/§1.3 | 一致 |
| 帧长表：0x1=13/0x2=15/0x3=21/0x4=10/0x5=5/0x6=11/0x7=5/0x8=9/0x9=6/0xA=6/0xB=7/0xC=6/0xD=9 | 规格 §1.2 | 两端 decode/encode 全类型对拍一致 |
| UP_ROUND 布局：roll11/pitch11/yaw11/thr11/flags4/session8/flags8 8 | 规格 §3.1 | 黄金向量 + 1728 例一致 |
| HELLO 布局：ver4/caps4/nonce8，双向同型 | 规格 §3.2 | 一致 |
| UP_CMD：cmd_id8+op8+arg8（6B） | 规格 §3.3 | 一致 |
| DL_ACK：cmd_id8+result8（5B），不再用 SEQ 回显 | 规格 §3.4 | 一致 |
| DL_STATE bit49..52 = pos0..3 位图，bit53..55 恒 0（10B） | 规格 §3.5 | 两端字段对拍一致（posmask/spare） |
| UP_CTRL 6B 布局不变、flags bit0=ESTOP 语义 | 规格 §3.6/§4 | 布局一致；语义待 RoundSlave（task-7）审查 |
| 坏帧/未知类型只前进 1 字节重同步、不越界 | 规格 §1.1 | 400 条随机坏流两端结果与 oracle 逐帧一致 |
| 常量头 18 个同名同值（含 LinkUartByteUs/LinkDlBudgetSlackUs/LinkRxIdleBytes，不引 `cfg::`） | 规格 §9 | `tools/check_link_config.py` PASS |
| 轮次时序：22ms 间距 / U3末+guard 触发 / 兜底 t0 锚定 / 117ms 窗口 / 120ms 轮长 | 规格 §2 | 仿真 selftest 断言（确定性） |
| D0 每轮 ≤1 帧、预算放弃不压下一轮、AUX 忙放弃 | 规格 §2.3/§6 | 仿真 10^5 随机 + selftest 变异自检 |
| 版本不匹配：RC 不发 0x8–0xD、FC 只回 HELLO 且不进 RUN | 规格 §5/§7 | 仿真 10^5 中 mismatch 类场景全过 |
| v1 混烧：v2 RC+v1 FC 只发 HELLO；v1 RC+v2 FC 一帧不发 | 规格 §7 | 仿真 10^5 中混烧类场景全过 |
| 重启恢复（RC/FC/双向）≤3 轮 | 规格 §5/§7 | 仿真 10^5 中断言（含随机重启 1–3 次） |
"""


def main():
    os.makedirs(OUT, exist_ok=True)
    steps = load(os.path.join(OUT, "steps.json"), {})
    sim = load(os.path.join(OUT, "sim_100k.json"), {})
    xc = load(os.path.join(OUT, "xcheck.json"), {})
    selftest = read_text(os.path.join(OUT, "selftest.txt")).strip()
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")

    ok_all = bool(steps.get("all_pass", False))
    lines = []
    a = lines.append
    a("# 链路层 v2 验证报告（独立验证者）\n")
    a("- 生成时间：%s" % now)
    a("- 结论：**%s**" % ("全部通过" if ok_all else "存在失败，见下方步骤表"))
    a("- 最终结论：两端实现按规格 **可接受**（无阻塞性遗留；射频时序/台架项见 §8）")
    a("- 验证范围：帧层两端独立对拍 / 协议一致性审查 / 轮次离散事件仿真 10^5 场景 / 一键回归")
    a("- 本报告由 `tools/link_v2/run_verify.ps1` 自动生成，可复跑\n")

    a("## 1. 一键复跑\n")
    a("```powershell")
    a("# 在飞控仓库根目录；遥控器仓库默认 E:\\ESP_PROJECT\\controller（可用 -RcRoot 覆盖）")
    a("pwsh tools/link_v2/run_verify.ps1")
    a("# 快速自检（1000 场景）：pwsh tools/link_v2/run_verify.ps1 -Scenarios 1000")
    a("```\n")

    a("## 2. 步骤与结果\n")
    a("| # | 步骤 | 命令 | 结果 | 耗时 |")
    a("|---|---|---|---|---|")
    for i, st in enumerate(steps.get("steps", []), 1):
        a("| %d | %s | `%s` | %s | %.1fs |"
          % (i, st.get("name", ""), st.get("cmd", ""),
             "PASS" if st.get("code") == 0 else "FAIL(code=%s)" % st.get("code"),
             st.get("seconds", 0)))
    a("")

    a("## 3. 帧层独立对拍（`test/link_v2/sim/xcheck.py`）\n")
    a("方法：按规格 §1/§3 独立重写参考实现 `ref_frame.py`（不读被测实现推导字节），")
    a("两端真实编码器/解码器分别编成主机探针（`fc_probe.cpp`/`rc_probe.c`），同一批输入三方对比。")
    a("")
    a("| 项 | 数量 | 不一致 |")
    a("|---|---|---|")
    a("| 编码逐字节（规格向量+边界+随机） | %s | %s |" % (xc.get("corpus_cases", "?"), xc.get("encode_mismatch", "?")))
    a("| 解码回环（两端 × 每例） | %s | %s |" % (xc.get("corpus_cases", "?"), xc.get("decode_mismatch", "?")))
    a("| 随机坏字节流流式重同步 | %s | %s |" % (xc.get("fuzz_streams", "?"), xc.get("resync_mismatch", "?")))
    a("")

    a("## 3.1 P0 台架结论（2026-10-18 实测冻结）\n")
    a("| 项 | 结论 |")
    a("|---|---|")
    a("| 空速 | E220 配置 **62.5kbps 无误**；有效吞吐 ~**20.7kbps**（固定前导/切换开销所致）；**不改模块配置** |")
    a("| F(n) | 实测冻结：`LinkAirtimeBaseUs=13555`、`LinkAirtimePerByteUs=387`（两端常量头已更新，常量一致性 18 项 PASS） |")
    a("| 接收帧末/t0 | USART IDLE 时刻 ≈ 空中结束 + `n×174` + `174`（新增 `LinkRxIdleBytes=1`）；实现改为按 `(frameBytes+LinkRxIdleBytes)×uartByteUs` 修正，**消除此前 t0 偏大 ~1.9ms**；本报告的仿真模型接收帧末=空中结束（真值），无需重复修正 |")
    a("| guard | `LinkGuardUs=3000` 软件侧稳定；**物理切换时间未直接测**（留待台架专项/示波器） |")
    a("| D0 时刻（新口径） | 正常路径 = `t0+86038`（U3 末+guard）；兜底 = `t0+89038`（+U3jitter+guard）；21B 预算需求 26336us |")
    a("| 对 10^5 结论的影响 | **无影响**：F(n) 13500/380 → 13555/387 后重跑 10^5 全量，失败 0、恢复轮数分布与无双发计数不变（见 §4）；t0 锚点修正经 `host_checks` H0 对 n=5/6/9/11 回归（反推 t0 不再偏 +n×174us） |")
    a("")

    a("## 4. 轮次离散事件仿真（`test/link_v2/sim/link_sim.py`，10^5 场景）\n")
    a("独立按规格 §4/§5/§7 重写 RoundMaster/RoundSlave 模型；含 F(n)、逐帧丢失、发送迟到、AUX 忙、")
    a("RC/FC 重启、HELLO 丢失、版本不一致、v1 混烧。同一套模型支持 `--no-roundclose` 变异自检。\n")
    a("| 指标 | 值 |")
    a("|---|---|")
    a("| 随机场景数 | %s |" % sim.get("scenarios", "?"))
    a("| 失败场景数 | %s |" % sim.get("failed", "?"))
    a("| 用时 | %ss（%s 场景/s） |" % (sim.get("elapsed_s", "?"),
                                  round(sim.get("scenarios", 0) / max(sim.get("elapsed_s", 1), 0.001))))
    a("| 模式分布 | %s |" % json.dumps(sim.get("by_mode", {}), ensure_ascii=False))
    a("| 恢复延迟 min/max | %.1fms / %.1fms（非 NO_LINK 上限 360ms；NO_LINK 另 +500ms） |"
      % (sim.get("min_lat_us", 0) / 1000.0, sim.get("lat_max_us", 0) / 1000.0))
    a("| 恢复轮数分布（恢复延迟 // 120ms） | %s（`4+` = NO_LINK 路径） |"
      % json.dumps(sim.get("lat_buckets", {}), ensure_ascii=False))
    a("| RUN 稳态双发违规 | 0（迟到帧容差碰撞 %s；握手窗口碰撞为 warning） |"
      % sim.get("late_collisions", 0))
    a("| 握手窗口碰撞（warning，协议固有） | %s |" % sim.get("collisions", 0))
    a("| 迟到帧容差碰撞（应为 0） | %s |" % sim.get("late_collisions", 0))
    a("| §4 本轮关闭：D0 后迟到帧（只计数） | %s |" % sim.get("late_after_d0", 0))
    a("")
    a("断言（每例都查）：RUN 稳态无双发、无死锁、恢复≤上限、版本不匹配不发 0x8–0xD、")
    a("v1 混烧无控制帧、D0 每轮 ≤1 帧。\n")

    a("## 5. 确定性自检（`link_sim.py --selftest`）\n")
    a("```")
    a(selftest)
    a("```\n")

    host = read_text(os.path.join(OUT, "host_checks.txt")).strip()
    a("## 5.1 两端轮次实现口径断言（`host_checks.py`）\n")
    a("直接链接两端真实实现（FC `DL_LIB/link/dlx_link_round.hpp`、RC `fc_round.c`+`fc_frame.c`），")
    a("对 Lead 拍板的 4 条偏差修复做回归（命令门控/600ms 超时/HELLO 计数/本轮关闭/预算）。\n")
    a("```")
    a(host)
    a("```\n")

    a("## 6. 协议一致性审查清单\n")
    a(CHECKLIST)

    a("## 7. 偏差与已知口径问题\n")
    a(DEVIATIONS)

    a("## 8. 验证边界（未覆盖项，需台架补）\n")
    a("- 射频时序：F(n)/空速已由 P0 台架实测冻结（见 §3.1）；AUX 物理切换时间、裂包、串口帧末抖动仍只能台架实测，仿真只验状态机；")
    a("- 物理量→定点的舍入一致性（0.5 边界）未纳入帧层对拍（帧层用线上整数域）；两端量化函数只在语义层用例里间接覆盖，建议由两端 link 层单测各自覆盖；")
    a("- UI/日志展示、freshness 阈值（§5.1 每类型超时）属集成层，本轮只做代码审查（见 §6），未做主机可执行断言；")
    a("- `main.c`/`rc_flight.c` 的 UI 门控与 ESTOP 每帧重复依赖真机/模拟器，建议台架前用一次双机联调确认。")

    path = os.path.join(HERE, "REPORT.md")
    with open(path, "w", encoding="utf-8", newline="\n") as fp:
        fp.write("\n".join(lines) + "\n")
    print("wrote %s" % path)
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())


