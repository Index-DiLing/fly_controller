# -*- coding: utf-8 -*-
"""
生成发布版总览图(一张图看全: 模块结构 / 上电模式 / 线程时序 / 控制环 / 数据结构 / 日志链路 / 报文)。

    python docs/make_release_overview.py

输出 docs/release_overview.png(位图, 便于查看/打印)与 docs/release_overview.svg(矢量, 可放大)。
图中数字都取自 flight_config_struct.hpp / release.cpp / release_log_manage.hpp /
dlx_flash_manager_config.h 的实际配置, 改代码后重跑本脚本即可刷新。
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DengXian"]
plt.rcParams["axes.unicode_minus"] = False

# ---- 配色(按子系统分类) ----
C_FLIGHT, C_E_FLIGHT = "#DCEBFA", "#2F6DA8"    # 飞行/控制/EKF
C_LOG, C_E_LOG = "#E2F0DC", "#4C7F3A"          # 日志/Flash
C_GROUND, C_E_GROUND = "#FBEBD5", "#B2762A"    # 地面模式/串口
C_MSG, C_E_MSG = "#EFE3F5", "#6D4B8E"          # DLX 报文
C_STATE, C_E_STATE = "#F2E7D5", "#8A6A34"      # 数据结构
C_NEUTRAL, C_E_NEUTRAL = "#F2F3F5", "#7A7F87"  # 其它
INK = "#1D2126"

HERE = os.path.dirname(os.path.abspath(__file__))
FIG_W_IN, FIG_H_IN = 25.0, 17.0
fig = plt.figure(figsize=(FIG_W_IN, FIG_H_IN), dpi=200)
fig.patch.set_facecolor("white")


def panel(rect, title, subtitle=None):
    ax = fig.add_axes(rect)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis("off")
    # 记录本面板里"1 个坐标单位 = 多少磅", 供自动折行/缩字号用
    ax.ppu_x = rect[2] * FIG_W_IN * 72.0 / 100.0
    ax.ppu_y = rect[3] * FIG_H_IN * 72.0 / 100.0
    ax.text(0.4, 99.5, title, ha="left", va="top", fontsize=15.5, weight="bold", color=INK)
    if subtitle:
        ax.text(0.4, 93.4, subtitle, ha="left", va="top", fontsize=9.6, color="#5A6169")
    return ax


def _char_w(ch):
    return 1.0 if ord(ch) > 0x2000 else 0.55


def _wrap(text, max_w):
    """按"视觉宽度"折行: 中文算 1, 英文/数字算 0.55"""
    out, cur, w = [], "", 0.0
    for ch in text:
        cw = _char_w(ch)
        if cur and w + cw > max_w:
            out.append(cur)
            cur, w = "", 0.0
        cur += ch
        w += cw
    if cur:
        out.append(cur)
    return out or [""]


def _fit(ax, w, h, title, lines, ts, bs):
    """逐步缩小字号, 直到"标题 + 折行后的正文"能放进框里, 保证永不溢出"""
    for ts_try in (ts, ts - 0.4, ts - 0.8, ts - 1.2):
        for bs_try in (bs, bs - 0.4, bs - 0.8, bs - 1.2, bs - 1.6, bs - 2.0, bs - 2.4, bs - 2.8):
            tw = (w - 2.6) * ax.ppu_x / max(ts_try, 1e-6)
            bw = (w - 3.6) * ax.ppu_x / max(bs_try, 1e-6)
            wrapped = [_wrap(l, bw) for l in lines]
            n = sum(len(x) for x in wrapped)
            need = 3.4
            if title:
                need += (ts_try * 1.30 + 1.6) / ax.ppu_y
            need += n * (bs_try * 1.42) / ax.ppu_y
            title_ok = (not title) or _wrap(title, tw) == [title]
            if need <= h and title_ok:
                return ts_try, bs_try, wrapped
    return 6.0, 5.6, [_wrap(l, (w - 3.6) * ax.ppu_x / 5.6) for l in lines]


def box(ax, x, y, w, h, title=None, lines=(), fc=C_NEUTRAL, ec=C_E_NEUTRAL, ts=10.6, bs=9.0,
        lh=None):
    ts_use, bs_use, wrapped = _fit(ax, w, h, title, list(lines), ts, bs)
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.4,rounding_size=1.6",
                                linewidth=1.3, edgecolor=ec, facecolor=fc, zorder=2))
    ty = y + h - 2.6
    if title:
        ax.text(x + w / 2.0, ty, title, ha="center", va="top", fontsize=ts_use, weight="bold",
                color=INK, zorder=3)
        ty -= (ts_use * 1.30 + 1.6) / ax.ppu_y
    step = (bs_use * 1.42) / ax.ppu_y
    for ln in [l for group in wrapped for l in group]:
        ax.text(x + 1.8, ty, ln, ha="left", va="top", fontsize=bs_use, color=INK, zorder=3)
        ty -= step


def arrow(ax, p1, p2, label=None, color="#4A5058", ls="-", rad=0.0, fs=8.4, lw=1.3,
          label_dx=0.0, label_dy=1.4):
    ax.add_patch(FancyArrowPatch(p1, p2, arrowstyle="-|>", mutation_scale=11, linewidth=lw,
                                 color=color, linestyle=ls, zorder=1, shrinkA=0.8, shrinkB=0.8,
                                 connectionstyle="arc3,rad=%.2f" % rad))
    if label:
        ax.text((p1[0] + p2[0]) / 2.0 + label_dx, (p1[1] + p2[1]) / 2.0 + label_dy, label,
                ha="center", va="bottom", fontsize=fs, color="#3A4048", zorder=4)


def table(ax, x, y, w, col_frac, rows, header, row_h=4.2, fs=8.4, head_fs=8.8):
    xs = [x + w * sum(col_frac[:i]) for i in range(len(col_frac) + 1)]
    ax.add_patch(Rectangle((x, y - row_h), w, row_h, facecolor=C_MSG, edgecolor=C_E_MSG,
                           linewidth=1.0, zorder=2))
    def _vis_len(s):
        return sum(_char_w(ch) for ch in s)

    cell_fs = fs
    for i in range(len(header)):
        cw = (xs[i + 1] - xs[i] - 1.6) * ax.ppu_x
        longest = max([_vis_len(r[i]) for r in [header] + list(rows)] or [1.0])
        cell_fs = min(cell_fs, cw / max(longest, 1e-6))
    cell_fs = max(min(cell_fs, fs), 5.8)
    head_fs = max(min(head_fs, cell_fs + 0.4), 5.8)
    for i, htxt in enumerate(header):
        ax.text(xs[i] + 0.8, y - row_h / 2.0, htxt, ha="left", va="center", fontsize=head_fs,
                weight="bold", color=INK, zorder=3)
    yy = y - row_h
    for r in rows:
        ax.add_patch(Rectangle((x, yy - row_h), w, row_h, facecolor="white", edgecolor="#C9CED6",
                               linewidth=0.8, zorder=2))
        for i, cell in enumerate(r):
            ax.text(xs[i] + 0.8, yy - row_h / 2.0, cell, ha="left", va="center", fontsize=cell_fs,
                    color=INK, zorder=3)
        yy -= row_h
    return yy


# ==================================================================================================
# 标题 + 图例
# ==================================================================================================
top = panel([0.008, 0.936, 0.984, 0.058], "DLX 飞控发布版 · 总览图",
            "release.cpp + flight_config_struct.hpp + release_log_manage.hpp ｜ 500Hz 控制环 + EKF/气压计 ｜ 一次上电 = 一个日志会话")
for i, (t, fc, ec) in enumerate([("飞行 / 控制 / EKF", C_FLIGHT, C_E_FLIGHT),
                                 ("日志 / Flash", C_LOG, C_E_LOG),
                                 ("地面模式 / 串口", C_GROUND, C_E_GROUND),
                                 ("DLX 报文", C_MSG, C_E_MSG),
                                 ("数据结构", C_STATE, C_E_STATE)]):
    lx = 1.2 + i * 19.2
    top.add_patch(Rectangle((lx, 6.5), 2.6, 8.5, facecolor=fc, edgecolor=ec, linewidth=1.1))
    top.text(lx + 3.5, 10.8, t, ha="left", va="center", fontsize=9.4, color=INK)
top.text(99.0, 10.8, "数字均为当前配置实际取值", ha="right", va="center", fontsize=9.0,
         color="#5A6169")

# ==================================================================================================
# 1 上电与模式判定
# ==================================================================================================
p1 = panel([0.008, 0.618, 0.474, 0.310], "1  上电 → 模式判定(decideBootMode)",
           "跳线帽没接: 直接飞且串口静音; 接上: 等上位机解锁命令")
box(p1, 33, 86, 34, 11, "上电 / 复位", ["DWT 计时使能 → USART1 2Mbaud → 协议对象就绪"], bs=8.6)
box(p1, 20, 66, 60, 16, "跳线帽 PB8 & PB9 两相检测(groundJumpersPresent)",
    ["相位1: PB8 输出低 + PB9 上拉 → 读 PB9",
     "相位2: PB9 输出低 + PB8 上拉 → 读 PB8",
     "任一相为低 = 接上; 之后两脚恢复输入上拉"], fc=C_GROUND, ec=C_E_GROUND, bs=8.7)
arrow(p1, (50, 86), (50, 82.3))
arrow(p1, (20, 74), (11, 74), "否(未接)", label_dy=1.6)
arrow(p1, (11, 74), (11, 62.6))
arrow(p1, (80, 74), (89, 74), "是(接上)", label_dy=1.6)
arrow(p1, (89, 74), (89, 62.6))
box(p1, 1, 44, 20, 18, "Flight 飞行模式",
    ["debugPort = nullptr(串口静音)", "flash: 预留 32 槽, Halt 策略", "日志 50Hz 写 flash",
     "上电即能飞"], fc=C_FLIGHT, ec=C_E_FLIGHT, bs=8.7)
box(p1, 74, 44, 25, 18, "地面模式: 等解锁命令",
    ["LED 120ms 闪; 不输出电机", "接受 GroundCmd(op=1, mode)", "或旧版 UnBlock(param)"],
    fc=C_GROUND, ec=C_E_GROUND, bs=8.7)
arrow(p1, (86.5, 44), (86.5, 40.6))
box(p1, 74, 31, 25, 10, "mode = 0 / 1 / 2", ["0 飞行 · 1 串口调试 · 2 日志管理"], fc=C_GROUND,
    ec=C_E_GROUND, bs=8.7)
arrow(p1, (74, 36), (18, 28), "mode=0", rad=0.06, fs=8.2)
arrow(p1, (84, 31), (62, 28), "mode=1", fs=8.2)
arrow(p1, (96, 31), (84, 28), "mode=2", fs=8.2)
box(p1, 1, 17, 18, 12, "Flight", ["日志写 flash"], fc=C_FLIGHT, ec=C_E_FLIGHT, bs=8.5, ts=9.8)
box(p1, 52, 17, 20, 12, "SerialDebug", ["日志走串口 100Hz"], fc=C_GROUND, ec=C_E_GROUND, bs=8.5,
    ts=9.8)
box(p1, 74, 17, 25, 12, "LogManage", ["查 / 删 / 回传 / 格式化"], fc=C_LOG, ec=C_E_LOG, bs=8.5,
    ts=9.8)
arrow(p1, (11, 44), (11, 29.6))
box(p1, 1, 1, 90, 13, None,
    ["超时(kConfig.groundWaitTimeoutMs = 0 → 一直等) → 落到 LogManage(安全档, 不动电机)",
     "管理模式里再收到 Unlock 就返回 release.cpp 重新分派; 转飞行会按 32 槽重开会话",
     "SerialDebug 不进 flash; LogManage 与 Flight 上电先 fs.init()"], fc=C_NEUTRAL,
    ec=C_E_NEUTRAL, bs=8.4)

# ==================================================================================================
# 2 模块结构
# ==================================================================================================
p2 = panel([0.518, 0.618, 0.474, 0.310], "2  模块与数据流总览",
           "4 个发布项文件 + 依赖库; 数据只沿箭头方向流动")
box(p2, 30, 84, 40, 11, "发布项(4 个文件)", ["release.cpp / flight_config_struct.hpp / ",
                                             "release_log_manage.hpp(头文件实现)"], bs=8.6)
box(p2, 1, 60, 30, 20, "release.cpp",
    ["main()", "decideBootMode() 上电模式判定", "runFlightStack() 飞行/调试",
     " ├ nrf_tel 线程 50Hz", " ├ logger 线程 5ms", " └ 控制环 500Hz"], fc=C_FLIGHT,
    ec=C_E_FLIGHT, bs=8.5, ts=10.0)
box(p2, 35, 60, 30, 20, "flight_config_struct.hpp",
    ["kConfig + kConfig.hw(引脚/SPI/地址)", "BootMode / BootDecision",
     "FlightRuntimeState / Telemetry", "PerfStats / ReleaseContext", "工具: logLine/flushSerial/replyGround",
     "drainDlx/onFlashBusy/writeLogEntry"], fc=C_STATE, ec=C_E_STATE, bs=8.4, ts=10.0)
box(p2, 69, 60, 30, 20, "release_log_manage.hpp",
    ["runLogManage() 单线程命令循环", "列会话 / 删最老 N 个 / 整会话回传",
     "整片格式化 / Ping / 解锁切模式", "不初始化 IMU/NRF/TIM1", "→ 物理上不可能输出电机"], fc=C_LOG,
    ec=C_E_LOG, bs=8.4, ts=10.0)
box(p2, 1, 34, 47, 21, "DL_LIB/dlx.hpp(KSP 从 Kotlin 类生成)",
    ["报文视图 + XxxW() 写函数 + check() 解析 + 回调", "1 UnBlock / 7 ControlToFlight(NRF 收)",
     "8 FlightCoreStatus(NRF 发)", "11 GroundCmd(收)",
     "12 GroundReply / 13 SessionInfo", "14 PerfMonitor / 15 FlightLog(发)"], fc=C_MSG,
    ec=C_E_MSG, bs=8.3, ts=10.0)
box(p2, 52, 34, 47, 21, "DL_LIB 其它库",
    ["W25Q128/dlx_flash_manager: 会话·槽·条目, CRC16 掉电安全",
     "flight_control: FlightControlFilter(16 维 EKF)", "             FlightController(角度+角速度环) / mixMotors",
     "驱动: BMI088 / BME280 / NRF24L01 / W25Q128", "      TIM1+DShot300 / USART+DMA"], fc=C_NEUTRAL,
    ec=C_E_NEUTRAL, bs=8.3, ts=10.0)
box(p2, 1, 5, 98, 24, "每 2ms 一次的数据主干",
    ["BMI088 FIFO(陀螺 2000Hz / 加速度计 1600Hz) → 配对 → EKF(快路径每样本 + 慢路径每 8 样本)",
     "BME280(20ms, 100kHz I2C) → 气压高度 → EKF 的高度/垂速观测",
     "EKF → FlightController(角度环 + 角速度环) → mixMotors → TIM1 + DShot300(PE9/PE11/PE13/PE14)",
     "同周期: 回写遥测 → NRF24L01 50Hz 发遥控器 ｜ 日志按 20ms(飞行)/10ms(调试) 入队 → logger 落盘或发串口"],
    fc=C_FLIGHT, ec=C_E_FLIGHT, bs=8.5, ts=10.2)
arrow(p2, (50, 84), (50, 80.6))
for x0 in (16.0, 50.0, 84.0):
    arrow(p2, (50, 80.2), (x0, 80.2), lw=1.0)
    arrow(p2, (x0, 80.2), (x0, 76.3), lw=1.0)

# ==================================================================================================
# 3 三线程时序
# ==================================================================================================
p3 = panel([0.008, 0.318, 0.474, 0.290], "3  三线程时序(同一时间轴, 0~60ms)",
           "主线程 2ms / nrf_tel 20ms / logger 5ms; 三边只通过共享块交换数据")
AX_L, AX_R = 20.0, 98.0


def tx(ms):
    return AX_L + (AX_R - AX_L) * (ms / 60.0)


for name, y, fc, ec in [("主线程 500Hz(2ms)", 66.0, C_FLIGHT, C_E_FLIGHT),
                        ("nrf_tel 50Hz(20ms)", 42.0, C_MSG, C_E_MSG),
                        ("logger 200Hz(5ms)", 18.0, C_LOG, C_E_LOG)]:
    p3.add_patch(Rectangle((2, y), 96, 15, facecolor=fc, edgecolor=ec, linewidth=1.1, alpha=0.55,
                           zorder=1))
    p3.text(3.2, y + 12.7, name, ha="left", va="top", fontsize=9.6, weight="bold", color=INK,
            zorder=3)
p3.add_patch(Rectangle((AX_L, 4.5), AX_R - AX_L, 0.9, facecolor="#9AA2AC", zorder=2))
for ms in range(0, 61, 10):
    p3.add_patch(Rectangle((tx(ms), 3.0), 0.25, 2.4, facecolor="#9AA2AC", zorder=2))
    p3.text(tx(ms), 1.4, "%dms" % ms, ha="center", va="center", fontsize=8.4, color="#5A6169")
for ms in range(0, 61, 2):
    p3.add_patch(Rectangle((tx(ms), 66.0), 0.16, 15.0, facecolor=C_E_FLIGHT, alpha=0.35, zorder=2))
for ms in range(0, 61, 5):
    p3.add_patch(Rectangle((tx(ms), 18.0), 0.16, 15.0, facecolor=C_E_LOG, alpha=0.35, zorder=2))
for ms in range(0, 61, 20):
    p3.add_patch(Rectangle((tx(ms), 42.0), 0.24, 15.0, facecolor=C_E_MSG, alpha=0.5, zorder=2))
p3.text(tx(2), 74.4, "每个 2ms: 取共享状态 → IMU FIFO → EKF 快路径 → 控制/混控 → DShot → 回写遥测 → 日志入队",
        ha="left", va="center", fontsize=8.5, color=INK)
p3.text(tx(2), 62.4, "每 8 样本(4ms): 协方差+倾角 ｜ 每 20ms: 气压计 ｜ 每 20ms(飞行)/10ms(调试): 日志 ｜ 每 200ms: 负载统计",
        ha="left", va="center", fontsize=8.2, color="#3A4048")
p3.text(tx(0.4) + 0.6, 53.0, "Tx: FlightCoreStatus(四元数/四电机/高度字段) ｜ Rx: ControlToFlight → 指令 + lastValidCtrl",
        ha="left", va="center", fontsize=8.3, color=INK)
p3.text(tx(0.4) + 0.6, 38.4, "气压计有效时 Height 字段改发 EKF 估计的垂速(baroValid), 无效才发高度",
        ha="left", va="center", fontsize=8.2, color="#3A4048")
p3.text(tx(0.4) + 0.6, 29.2, "排空 LogQueue(≤24 条/次) → 飞行: appendLog ｜ 调试: FlightLogW + 每 200ms PerfMonitorW → DMA",
        ha="left", va="center", fontsize=8.3, color=INK)
for i, (t, s) in enumerate([("g_rt(临界区保护)", "指令 / 期望值 / 遥测 / 负载"),
                            ("LogQueue 16 条", "控制线程入队, logger 出队"),
                            ("g_rt.perf", "每 200ms 汇总 DWT 负载")]):
    box(p3, 2 + i * 32.6, 5.2, 31.0, 10.8, t, [s], fc=C_STATE, ec=C_E_STATE, bs=8.1, ts=9.2)
arrow(p3, (17.5, 73.2), (17.5, 16.2), "写遥测", lw=1.0, fs=8.0, label_dy=0.0, label_dx=-5.0)
arrow(p3, (17.6, 16.4), (17.6, 44.0), "读指令", lw=1.0, fs=8.0, label_dy=0.0, label_dx=5.0)

# ==================================================================================================
# 4 控制环内部
# ==================================================================================================
p4 = panel([0.518, 0.318, 0.474, 0.290], "4  控制环内部(每个 2ms 周期的顺序)",
           "电机/定时器操作全部在控制线程; 遥控线程只改共享状态")
steps = [
    ("1  rt_sem_take(ctrlSem) —— 2ms 硬定时器唤醒", C_FLIGHT, C_E_FLIGHT),
    ("2  取共享状态(临界区): command / 手动油门 / 期望姿态 / start·stop·hardStop", C_FLIGHT, C_E_FLIGHT),
    ("3  bmi.fifoRead() → 陀螺与加速度按比例就近配对", C_FLIGHT, C_E_FLIGHT),
    ("4  每陀螺样本: ekf.integrateNominal(gyro, accel, 0.5ms) —— 2000Hz 快路径", C_FLIGHT, C_E_FLIGHT),
    ("5  每 8 样本: propagateCovariance + updateAccelerometer —— 250Hz 慢路径", C_FLIGHT, C_E_FLIGHT),
    ("6  每 20ms: BME280 → getAltitude → updateBarometer(baroAbs - baroRef)", C_FLIGHT, C_E_FLIGHT),
    ("7  组装 FlightControlState: 姿态/高度/垂速 ← EKF, 机体角速度 ← 陀螺", C_FLIGHT, C_E_FLIGHT),
    ("8  失控保护: now - lastValidCtrl > 5000ms 且已使能 → 停机", C_GROUND, C_E_GROUND),
    ("9  解锁: DShot 预置 0 → TIM1 时基 → 开 MOE → DMA; baroRef 与 EKF 高度归零", C_GROUND, C_E_GROUND),
    ("10 停机: TIM1 停 → 关 MOE → DMA 停; 启动序列 3200ms 后 en=1", C_GROUND, C_E_GROUND),
    ("11 角度环 updateAngle(sp, fcs, 2ms) → 期望力矩 / 角速度设定", C_FLIGHT, C_E_FLIGHT),
    ("12 mixMotors → 电机映射 {0,2,3,1} → DShot 计数(~50..1950) + 饱和判断", C_FLIGHT, C_E_FLIGHT),
    ("13 dshot.preloadThrottle(motor) —— 下一帧 DShot 生效", C_FLIGHT, C_E_FLIGHT),
    ("14 回写遥测(临界区): 姿态/四电机/高度/垂速/baroValid/error", C_STATE, C_E_STATE),
    ("15 事件位(解锁/停机/失控/饱和/日志故障…); 按周期打包 LogEntry → logQueuePush", C_LOG, C_E_LOG),
    ("16 负载统计累加(主循环/EKF/控制/IO); 每 200ms 汇总到 g_rt.perf", C_LOG, C_E_LOG),
]
y = 87.5
for i, (t, fc, ec) in enumerate(steps):
    box(p4, 1.5, y - 4.4, 97.0, 5.0, None, [t], fc=fc, ec=ec, bs=8.4)
    if i:
        arrow(p4, (50, y + 0.9), (50, y + 1.5), lw=0.9)
    y -= 4.85

# ==================================================================================================
# 5 数据结构
# ==================================================================================================
p5 = panel([0.008, 0.012, 0.316, 0.290], "5  数据结构(跨线程共享)",
           "谁写哪一块在 flight_config_struct.hpp 里有约定")
box(p5, 1, 60, 98, 28, "FlightRuntimeState(飞行状态)",
    ["指令区(遥控线程写): command / manualThrottle / setpoint / motorValue[4]",
     "                   startMotor / stopMotor / hardStop / lastValidCtrl",
     "控制区: enabledMotor / startingMotor / motorStartTime",
     "遥测区(控制→NRF): telem{ quat / motor[4] / height_m / vertical_vel_mps /",
     "                        baroValid / error / enabled }",
     "负载区(控制→logger): perf{ 各段周期数 / samples / drops }",
     "日志区(logger→控制): logError / logFull / logEntries"],
    fc=C_STATE, ec=C_E_STATE, bs=8.3, ts=10.0, lh=4.1)
box(p5, 1, 41, 98, 16, "ReleaseContext(两个源文件共享)",
    ["usart / serialDma / txBuf(2048B) / uartRx(256B)",
     "protocol(DLX 收发) / fs(FlashManager*) / ledRun / mode"], fc=C_STATE, ec=C_E_STATE, bs=8.4,
    ts=10.0)
box(p5, 1, 23, 48, 15, "LogQueue(16 × 144B)",
    ["head / tail / count / dropped", "满则丢最旧 + 计数, 不阻塞控制环"], fc=C_LOG, ec=C_E_LOG,
    bs=8.2, ts=9.6)
box(p5, 51, 23, 48, 15, "PerfAccum / BootDecision / ManageCommand",
    ["累加器 / 模式判定结论 / 命令暂存", "都在文件作用域, 不跨模块暴露"], fc=C_LOG, ec=C_E_LOG,
    bs=8.2, ts=9.6)
box(p5, 1, 1, 98, 19, "LogEntry = 144B(管理器 12B + 业务 132B)",
    ["管理器: type / reserved / crc16 / sessionId / seq",
     "业务: tickMs / flags / events / linkAgeMs / loopPeriodUs",
     "quat[4] / bodyRate[3] / accelG[3] / rateSetpoint[3] / torque[3]",
     "targetPitch·Roll·Height / throttle / manualThrottle / heightM / vertVelMps",
     "baroRelM / baroAbsM / gyroBias[3] / motor[4]"], fc=C_LOG, ec=C_E_LOG, bs=8.2, ts=9.8,
    lh=3.8)

# ==================================================================================================
# 6 日志与 Flash
# ==================================================================================================
p6 = panel([0.334, 0.012, 0.316, 0.290], "6  日志链路与 Flash",
           "flash 存 POD(CRC16), 串口发 FlightLog 报文(CRC4)")
box(p6, 1, 81, 98, 9, "W25Q128 16MB 三段布局",
    ["元数据 120KB(30 扇区) ｜ 参数 8KB(2 扇区) ｜ 日志段 127 槽 × 128KB ≈ 15.87MB(环形)"],
    fc=C_LOG, ec=C_E_LOG, bs=8.3, ts=9.8)
box(p6, 1, 64, 98, 15, "一次上电 = 一个会话(默认预留 32 槽)",
    ["32 槽 × 885 条/槽 = 28320 条 ≈ 50Hz 下 9.4 分钟", "上电只擦上次真正用掉的槽(典型 ≈0.3s/槽)",
     "写满即停(Halt): 飞行中零擦除; 地面 dropSessions/clearHistory 扩容"], fc=C_LOG, ec=C_E_LOG,
    bs=8.2, ts=9.8)
box(p6, 1, 47, 98, 15, "写入(飞行模式)",
    ["控制环每 20ms 打包 LogEntry → LogQueue → logger(5ms) appendLog()",
     "记录 = [0xAA][LogEntry 144B][0x00] + 补齐 = 148B/条, CRC16 掉电安全",
     "写入 ≈0.23ms/条(SPI2 5.25MHz), 只在 logger 线程发生"], fc=C_FLIGHT, ec=C_E_FLIGHT, bs=8.2,
    ts=9.8)
box(p6, 1, 30, 98, 15, "回传 / 串口调试(上位机用生成代码解析)",
    ["POD → FlightLogW() → [head 2B][负载 140B] = 142B/条(类型 15, CRC4)",
     "调试流: 100Hz FlightLog + 5Hz PerfMonitor(类型 14, DWT 负载)",
     "管理回传: 按会话逐条读 flash → 转 FlightLog 帧分批发(DMA 阻塞)"], fc=C_GROUND,
    ec=C_E_GROUND, bs=8.2, ts=9.8)
box(p6, 1, 1, 98, 27, "日志管理命令(单线程, 不动电机)",
    ["2 列会话 → 每会话一条 SessionInfo + 汇总 GroundReply(N 条)",
     "      (先扫槽头, 再逐会话数条目: 0.2~7s/会话)",
     "3 删最老 N 个会话 → 擦掉它们占的槽, 腾出的槽并入当前会话",
     "4 回传会话 → 整会话 FlightLog 帧流, 结尾回 GroundReply(条数)",
     "5 整片格式化 → 2048 个 64KB 块(几十秒) → 重新 init(预留 1 槽)",
     "1 解锁切模式(0/1/2) · 6 Ping; 擦除时 PF2 每 64KB 翻一次"], fc=C_GROUND, ec=C_E_GROUND,
    bs=8.1, ts=9.8, lh=3.95)

# ==================================================================================================
# 7 报文 / 频率 / 状态位
# ==================================================================================================
p7 = panel([0.658, 0.012, 0.334, 0.290], "7  报文清单 / 频率 / 状态位",
           "报文由 KSP 从 Kotlin 类生成: 标量大端, 定长数组原始小端")
table(p7, 1, 88, 98, (0.10, 0.40, 0.15, 0.35),
      [("1", "UnBlock", "1B", "上位→下位(旧解锁)"),
       ("7", "ControlToFlight", "30B", "上位→下位(NRF 遥控)"),
       ("8", "FlightCoreStatus", "30B", "下位→上位(NRF 遥测)"),
       ("11", "GroundCmd", "12B", "上位→下位(地面命令)"),
       ("12", "GroundReply", "12B", "下位→上位(命令应答)"),
       ("13", "SessionInfo", "16B", "下位→上位(会话信息)"),
       ("14", "PerfMonitor", "28B", "下位→上位(负载统计)"),
       ("15", "FlightLog", "140B", "下位→上位(一条日志)")],
      ("类型", "名称", "负载", "方向 / 用途"), row_h=4.0, fs=8.1, head_fs=8.6)
box(p7, 1, 45, 98, 16, "频率与耗时(实际配置)",
    ["陀螺 2000Hz / 加速度计 1600Hz ｜ 控制环 500Hz(2ms) ｜ EKF 慢路径 250Hz(每 8 样本)",
     "气压计 50Hz ｜ NRF 遥测 50Hz ｜ 飞行日志 50Hz ｜ 调试日志 100Hz ｜ 负载统计 5Hz",
     "logger 200Hz(5ms) ｜ 串口 2Mbaud ｜ flash 写 ≈0.23ms/条 ｜ 擦 ≈0.3s/槽 ｜ 整片 ≈几十秒"],
    fc=C_NEUTRAL, ec=C_E_NEUTRAL, bs=8.1, ts=9.8)
box(p7, 1, 23, 48, 20, "状态位 flags(电平型)",
    ["0x0001 链路在线    0x0002 IMU OK", "0x0004 气压 OK      0x0008 角度环工作",
     "0x0010 电机使能    0x0020 启动序列", "0x0040 硬停机      0x0080 flash 可用",
     "0x0100 日志写满    0x0200 日志出错", "0x0400 电机饱和    0x0800 失控过",
     "0x1000 地面模式"], fc=C_STATE, ec=C_E_STATE, bs=7.9, ts=9.6, lh=3.6)
box(p7, 51, 23, 48, 20, "事件位 events(边沿型, 随日志清零)",
    ["0x0001 指令变化    0x0002 解锁", "0x0004 停机        0x0008 失控保护",
     "0x0010 电机饱和    0x0020 日志故障", "0x0040 气压更新    0x0080 EKF 归零",
     "", "同一条日志里汇总, 归档后清零"], fc=C_STATE, ec=C_E_STATE, bs=7.9, ts=9.6, lh=3.6)
box(p7, 1, 1, 98, 20, "运行约定(易踩的点)",
    ["• 跳线帽没接: 不看串口; 接上: 等解锁(LED 120ms 闪), 超时落到日志管理",
     "• 飞行模式串口完全静音(debugPort=nullptr); 调试模式不碰 flash",
     "• 遥控线程只置标志(cmd>3 → hardStop), 关定时器/DMA 由控制线程做",
     "• 日志写满/出错只置状态位, 不影响飞行; 失控保护 5s 超时停机",
     "• 改 LogEntry 字段会让布局指纹不符 → 需整片格式化一次(或开 autoFormatOnMismatch)"],
    fc=C_NEUTRAL, ec=C_E_NEUTRAL, bs=8.1, ts=9.8)

def save_robust(path, **kwargs):
    """先写临时文件再替换: 目标被图片查看器占用时也不会整脚本失败"""
    tmp = path + ".tmp"
    fmt = os.path.splitext(path)[1].lstrip(".").lower() or "png"
    fig.savefig(tmp, format=fmt, **kwargs)
    try:
        os.replace(tmp, path)
        print("written:", path)
    except OSError as exc:  # 目标被别的程序打开着
        print("warning: %s 被占用(%s), 已写为 %s" % (path, exc, tmp))


out_png = os.path.join(HERE, "release_overview.png")
out_svg = os.path.join(HERE, "release_overview.svg")
save_robust(out_png, dpi=200, facecolor="white")
save_robust(out_svg, facecolor="white")
