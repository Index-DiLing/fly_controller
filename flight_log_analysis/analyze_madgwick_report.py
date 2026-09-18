"""从 flight_log_analysis.html(2026-09-08 的 Madgwick 飞行报告)里取出嵌入数据, 做同样的检验。

该 HTML 里有一行 `const D={...};` —— 是当时上位机/脚本导出的序列(已按 0.2s 抽样):
  t, roll, pitch, yaw(deg), bx,by,bz(机体角速度 rad/s), acc(|a| g), thr, mA(电机均值),
  armed/pid/sat/timeout(状态位), link(ms), eps/eps20/peaks(异常 pitch 区间)

能做的对比(与 EKF 那批同样的口径):
  1) 解锁段里姿态/角速度/加速度/油门/电机的范围 —— 飞得到底稳不稳
  2) 姿态角变化率 vs 机体角速度(运动学一致性): 若一致, 说明姿态基本是陀螺推出来的,
     没有被加速度计拖走; 这正是 EKF 那批做不到的地方
  3) 姿态低频漂移量(滤波后的峰峰)
"""
from __future__ import annotations

import io
import json
import os
import re

import numpy as np

from load_logs import out_path

HTML = r"E:\STM32Code\fly_controller\flight_log_analysis.html"
OUT = io.StringIO()


def out(*a):
    print(*a)
    print(*a, file=OUT)


src = open(HTML, encoding="utf-8").read()
m = re.search(r"const D=(\{.*?\});", src, re.S)
if not m:
    raise SystemExit("未找到嵌入数据")
D = json.loads(m.group(1))
t = np.array(D["t"])
roll = np.array(D["roll"])
pitch = np.array(D["pitch"])
yaw = np.array(D["yaw"])
bx, by, bz = np.array(D["bx"]), np.array(D["by"]), np.array(D["bz"])
acc = np.array(D["acc"])
thr = np.array(D["thr"])
mA = np.array(D["mA"])
armed = np.array(D["armed"], dtype=bool)

out("=" * 104)
out("一、Madgwick 那次飞行(2026-09-08, flight_log_analysis.html)总览")
out("=" * 104)
out(f"样本 {len(t)} 点, 采样间隔 {t[1]-t[0]:.2f}s(≈{1/(t[1]-t[0]):.0f}Hz), 时长 {t[-1]:.1f}s, "
    f"解锁帧 {int(armed.sum())}({100*armed.mean():.0f}%), link 最大 {int(np.max(D['link']))} ms")
idx = np.flatnonzero(np.diff(armed.astype(int)) != 0) + 1
b = [0, *idx, len(armed)]
segs = [(a, z) for a, z in zip(b[:-1], b[1:]) if armed[a]]
out(f"解锁段 {len(segs)} 段: " + "; ".join(f"{t[a]:.1f}~{t[z-1]:.1f}s({t[z-1]-t[a]:.1f}s)" for a, z in segs))
for i, (a, z) in enumerate(segs):
    w = np.stack([bx[a:z], by[a:z], bz[a:z]], axis=1)
    out(f"  段{i+1}: 油门 {thr[a:z].min():.3f}~{thr[a:z].max():.3f} 电机均值 {mA[a:z].mean():.1f} | "
        f"|ω| 中位 {np.median(np.linalg.norm(w,axis=1)):.2f} max {np.linalg.norm(w,axis=1).max():.2f} rad/s | "
        f"|a| {acc[a:z].min():.2f}~{acc[a:z].max():.2f} g | "
        f"roll {roll[a:z].min():+.1f}~{roll[a:z].max():+.1f}° pitch {pitch[a:z].min():+.1f}~{pitch[a:z].max():+.1f}°")

out("")
out("=" * 104)
out("二、关键检验: 姿态角变化率 与 机体角速度 是否一致(= 姿态是不是陀螺推出来的)")
out("   运动学: dpitch/dt = wy*cos(roll) - wz*sin(roll);  droll/dt = wx + tan(pitch)*(wy sin roll + wz cos roll)")
out("   斜率≈+1 => 姿态基本跟着陀螺(加速度计只做缓慢修正); 斜率明显偏离/相关差 => 被加速度计拖走")
out("=" * 104)
out(f"{'数据':>10} {'段':>4} {'dpitch/dt 斜率':>14} {'相关':>7} {'droll/dt 斜率':>14} {'相关':>7} {'R²(合成)':>10}")


def consistency(tag, mask, i0=0):
    if mask.sum() < 50:
        return
    r = np.radians(roll[mask])
    p = np.radians(pitch[mask])
    dt = t[1] - t[0]
    dp = np.gradient(pitch[mask], dt)
    dr = np.gradient(roll[mask], dt)
    wp = by[mask] * np.cos(r) - bz[mask] * np.sin(r)
    wr = bx[mask] + np.tan(p) * (by[mask] * np.sin(r) + bz[mask] * np.cos(r))
    k = (np.abs(dp) < 400) & (np.abs(dr) < 400) & (np.abs(p) < np.radians(60))
    sp = np.sum(wp[k] * dp[k]) / np.sum(wp[k] ** 2)
    sr = np.sum(wr[k] * dr[k]) / np.sum(wr[k] ** 2)
    cp = np.corrcoef(wp[k], dp[k])[0, 1]
    cr = np.corrcoef(wr[k], dr[k])[0, 1]
    out(f"{tag:>10} {i0:>4} {sp:>14.3f} {cp:>7.3f} {sr:>14.3f} {cr:>7.3f} {0.5*(cp**2+cr**2):>10.3f}")


for i, (a, z) in enumerate(segs):
    m = np.zeros(len(t), bool)
    m[a:z] = True
    consistency("Madgwick", m, i + 1)
consistency("Madgwick", ~armed, 0)

out("")
out("=" * 104)
out("三、解锁段内的姿态行为(0.5s 抽样): 是否出现'稳定倾斜不修正'")
out("=" * 104)
for i, (a, z) in enumerate(segs):
    out(f"--- 段{i+1} {t[a]:.1f}~{t[z-1]:.1f}s")
    for j in range(a, z, max(1, int(0.5 / (t[1] - t[0])))):
        out(f"  t={t[j]:6.1f} thr={thr[j]:.3f} roll={roll[j]:+7.2f} pitch={pitch[j]:+7.2f} "
            f"|ω|={np.hypot(np.hypot(bx[j],by[j]),bz[j]):5.2f} |a|={acc[j]:.2f} mot={mA[j]:5.1f}")

out("")
out("=" * 104)
out("四、低频姿态漂移(0.5s 滑动平均后的峰峰) —— 与 EKF 那批同口径")
out("=" * 104)
fs = 1.0 / (t[1] - t[0])
k = max(int(0.5 * fs) | 1, 3)
ker = np.ones(k) / k
for nm, arr in (("roll", roll), ("pitch", pitch)):
    lf = np.convolve(arr, ker, "same")
    out(f"  {nm}: 解锁段低频 5~95% {np.percentile(lf[armed],5):+.2f}~{np.percentile(lf[armed],95):+.2f}°, "
        f"峰峰 {lf[armed].max()-lf[armed].min():.2f}°, 高频抖动(去低频)σ {np.std(arr[armed]-lf[armed]):.2f}°")

with open(out_path("madgwick_flight"), "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())
