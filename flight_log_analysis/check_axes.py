"""完全与角度约定无关的传感器互检: 加速度计方向 u 与陀螺 ω 是否属于同一套机体轴。

刚体运动学: u(静止时=重力方向, 世界系固定) 在机体系里的时间导数满足
    du/dt = -ω × u
把 8 种加速度计符号组合 / 若干轴置换代入, 看哪种组合使残差 |du/dt + ω×u| 最小。
用桨停车的"手搬动"数据(只有重力+手, 没有推力/振动干扰)。
"""
from __future__ import annotations

import io
import os
from itertools import product, permutations

import numpy as np

from load_logs import load_all

OUT = io.StringIO()


def out(*a):
    print(*a)
    print(*a, file=OUT)


def residual(u, w):
    """u,w: (n,3) 同一采样率; 返回残差 |du/dt + w×u| 的 RMS(用相邻差分)。"""
    du = np.diff(u, axis=0)
    wm = 0.5 * (w[1:] + w[:-1])
    um = 0.5 * (u[1:] + u[:-1])
    cross = np.cross(wm, um)
    r = du + cross * 0.02            # du/dt*dt 与 -(w×u)*dt 应抵消
    return float(np.sqrt((r ** 2).sum(axis=1).mean()))


data = []
for s in load_all():
    m = (~s.armed) & (np.abs(s.acc_mag - 1.0) < 0.15) & (s.gyro_mag < 1.5)
    idx = np.flatnonzero(m)
    if len(idx) < 400:
        continue
    cont = np.diff(idx) == 1
    a = np.stack([s["acc_x"], s["acc_y"], s["acc_z"]], axis=1)[idx][:-1][cont]
    w = np.stack([s["gyro_x"], s["gyro_y"], s["gyro_z"]], axis=1)[idx][:-1][cont]
    u = a / np.linalg.norm(a, axis=1)[:, None]
    data.append((s.session_id, u, w))
    out(f"s{s.session_id}: 手搬动/静止帧 {len(u)}")

out("")
out("候选组合的残差 RMS(越小越自洽); 期望: 原始顺序(+x+y+z, 恒等置换)最小")
results = []
for perm in permutations(range(3)):
    for sgn in product((1, -1), repeat=3):
        tot, n = 0.0, 0
        for _, u, w in data:
            uu = u[:, list(perm)] * np.array(sgn)
            tot += residual(uu, w) * len(uu)
            n += len(uu)
        results.append((tot / n, perm, sgn))
results.sort()
for r, perm, sgn in results[:6]:
    name = "".join(("-" if sgn[i] < 0 else "+") + "xyz"[perm[i]] for i in range(3))
    out(f"   残差 {r:.5f}  ({name} 顺序; 置换 {perm}, 符号 {sgn})")
out("   ...")
for r, perm, sgn in results[-3:]:
    name = "".join(("-" if sgn[i] < 0 else "+") + "xyz"[perm[i]] for i in range(3))
    out(f"   残差 {r:.5f}  ({name} 顺序; 置换 {perm}, 符号 {sgn})")

base = [x for x in results if x[1] == (0, 1, 2) and x[2] == (1, 1, 1)]
out("")
if base:
    r0 = base[0][0]
    out(f"原始顺序(+x,+y,+z)的残差 = {r0:.5f}, 最优组合 = {results[0][0]:.5f} "
        f"(比值 {results[0][0]/r0 if r0 else 0:.2f})")

with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "axes_check.txt"),
          "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())
