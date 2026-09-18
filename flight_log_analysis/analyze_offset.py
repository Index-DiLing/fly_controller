"""量测"静止时的姿态读数偏移"。约定(用户已确认 + 代码一致): 抬头 pitch<0, 低头 pitch>0。

静止在地面、桨不转时, 加速度计方向就是重力方向, 所以此刻的读数 = 该姿态估计的零点偏移
(加速度计零偏 / 安装偏角 / EKF 起始参考)。控制器会把"读数=0"当成水平,
因此这个偏移会原封不动地变成飞行时的真实倾角, 进而变成固定方向的水平加速度。
"""
from __future__ import annotations

import io
import os

import numpy as np

from load_logs import load_all

OUT = io.StringIO()


def out(*a):
    print(*a)
    print(*a, file=OUT)


out("静止段(未解锁、角速度<0.05、|a-1g|<0.05)的姿态读数 —— 单位: 度")
out("pitch<0 = 抬头, pitch>0 = 低头; 读数即该估计的零点偏移(按理应为 0)")
for s in load_all():
    t = s.t
    stat = (~s.armed) & (s.gyro_mag < 0.05) & (np.abs(s.acc_mag - 1.0) < 0.05)
    idx = np.flatnonzero(stat)
    if len(idx) < 100:
        continue
    # 分成若干连续窗口, 每个 >= 2s
    br = np.flatnonzero(np.diff(idx) > 25)
    groups = [g for g in np.split(idx, br + 1) if len(g) >= 100]
    out(f"\ns{s.session_id} (共 {len(groups)} 个静止窗):")
    for g in groups:
        e = s.euler_deg[g]
        accx, accy = s["acc_x"][g], s["acc_y"][g]
        pA = -np.degrees(np.arcsin(np.clip(accx, -1, 1)))
        rA = np.degrees(np.arcsin(np.clip(accy / np.sqrt(np.maximum(1 - accx ** 2, 1e-9)), -1, 1)))
        out(f"   t={t[g[0]]:7.1f}~{t[g[-1]]:7.1f}s ({len(g):5d} 帧): "
            f"pitch {np.median(e[:,1]):+6.2f}° (由加速度计 {np.median(pA):+6.2f}°)  "
            f"roll {np.median(e[:,0]):+6.2f}° (由加速度计 {np.median(rA):+6.2f}°)  "
            f"| 等效水平加速度 = 低头 {np.tan(np.radians(np.median(e[:,1])))*9.81:+.2f} m/s²(向机头) / "
            f"右倾 {np.tan(np.radians(np.median(e[:,0])))*9.81:+.2f} m/s²(向右)")

out("")
out("解读: 若静止读数长期偏 negative(抬头/左倾), 控制器飞起来会把飞机压成 positive(低头/右倾),")
out("      也就是持续性'向机头 + 向右'的水平加速度 —— 与实飞观察到的右上漂一致。")

with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "offset_check.txt"),
          "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())
