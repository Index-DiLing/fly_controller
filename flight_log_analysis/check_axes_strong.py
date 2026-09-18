"""更强的轴向互检: 用"陀螺积分出来的转动"去预测加速度计方向的变化。

对每个窗口: 由陀螺样本积分出机体转动 ΔR(机体增量右乘):
    ΔR = Π exp([ω_k]·dt)
若加速度计与陀螺同轴同号, 则(世界固定的)g 方向在机体系里应满足
    u(t2) ≈ ΔR^T · u(t1)
比较实测 u(t2) 与预测的夹角(deg), 越小越自洽。逐一试 8 种符号/若干置换。
用桨停车的"手搬动"段(只有重力, 无推力/振动)。
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


def rotvec_to_mat(v):
    ang = np.linalg.norm(v)
    if ang < 1e-12:
        return np.eye(3)
    k = v / ang
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


def window_error(u, w, dt=0.02):
    """u,w: 同一窗口 (n,3); 返回实测 u_end 与陀螺预测的夹角(deg)。"""
    R = np.eye(3)
    for k in range(len(w) - 1):
        R = R @ rotvec_to_mat(w[k] * dt)
    pred = R.T @ u[0]
    pred /= np.linalg.norm(pred)
    meas = u[-1] / np.linalg.norm(u[-1])
    return np.degrees(np.arccos(np.clip(pred @ meas, -1, 1)))


N = 25  # 0.5s
data = []
segs = {}
for s in load_all():
    m = (~s.armed) & (np.abs(s.acc_mag - 1.0) < 0.15) & (s.gyro_mag < 1.0)
    idx = np.flatnonzero(m)
    if len(idx) < 400:
        continue
    a = np.stack([s["acc_x"], s["acc_y"], s["acc_z"]], axis=1)
    w = np.stack([s["gyro_x"], s["gyro_y"], s["gyro_z"]], axis=1)
    u = a / np.linalg.norm(a, axis=1)[:, None]
    wins = []
    i = 0
    while i + N < len(idx):
        if np.all(np.diff(idx[i:i + N + 1]) == 1):
            wins.append((u[idx[i]:idx[i] + N + 1], w[idx[i]:idx[i] + N + 1]))
            i += N
        else:
            i += 1
    if len(wins) >= 10:
        data.append((s.session_id, wins))
        out(f"s{s.session_id}: 0.5s 窗口 {len(wins)} 个")

out("")
out("各种组合下的平均夹角误差(deg) —— 最小者即真实轴向")
results = []
for perm in permutations(range(3)):
    for sgn in product((1, -1), repeat=3):
        errs = []
        for _, wins in data:
            for u, w in wins:
                errs.append(window_error(u[:, list(perm)] * np.array(sgn), w))
        results.append((float(np.mean(errs)), perm, sgn, float(np.median(errs))))
results.sort()
for r, perm, sgn, md in results[:6]:
    name = "".join(("-" if sgn[i] < 0 else "+") + "xyz"[perm[i]] for i in range(3))
    out(f"   平均 {r:6.2f}°  中位 {md:6.2f}°   ({name})")
out("   ...")
for r, perm, sgn, md in results[-2:]:
    name = "".join(("-" if sgn[i] < 0 else "+") + "xyz"[perm[i]] for i in range(3))
    out(f"   平均 {r:6.2f}°  中位 {md:6.2f}°   ({name})")

base = [x for x in results if x[1] == (0, 1, 2) and x[2] == (1, 1, 1)][0]
out("")
out(f"原始顺序(+x,+y,+z): 平均 {base[0]:.2f}° 中位 {base[3]:.2f}°; "
    f"最优: {results[0][0]:.2f}° ({''.join(('-' if results[0][2][i]<0 else '+')+ 'xyz'[results[0][1][i]] for i in range(3))})")

with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "axes_check_strong.txt"),
          "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())
