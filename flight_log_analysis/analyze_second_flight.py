"""session 4 第二次飞行 216.6~232s(离地前、bx/by≈0、气压≈0)的姿态到底错在哪。

对照三样东西:
  (1) EKF 姿态(日志四元数)   (2) 加速度计给出的倾角(= 地面静置时的真值)
  (3) 只积分陀螺得到的倾角(相对解锁时刻) —— 与 (2) 的差就是"加速度计方向自身的偏差"
并按油门分箱, 看加速度计方向的偏差是否随转速增大(振动整流/机架受力)。
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


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def qaxis(v, ang):
    n = np.linalg.norm(v)
    if n < 1e-12:
        return np.array([1.0, 0, 0, 0])
    s = np.sin(ang / 2) / n
    return np.array([np.cos(ang / 2), v[0] * s, v[1] * s, v[2] * s])


def euler(q):
    w, x, y, z = q / np.linalg.norm(q)
    return (np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
            np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))))


def lp(x, tau, fs=50.0):
    a = np.exp(-1 / (tau * fs))
    y = np.empty_like(x)
    acc = x[0]
    for i, v in enumerate(x):
        acc = a * acc + (1 - a) * v
        y[i] = acc
    return y


s = [x for x in load_all() if x.session_id == 4][0]
t = s.t
q = s.q
g = np.stack([s["gyro_x"], s["gyro_y"], s["gyro_z"]], axis=1)

out("session 4 第二次飞行 216.6~232s: 每 0.5s 对照")
out("  时间  油门 | EKF roll/pitch | 加速度计 roll/pitch | 陀螺积分 roll/pitch(自解锁) | |a|  | 门开 | baroRel")
arm = np.searchsorted(t, 216.6)
qn = q[arm] / np.linalg.norm(q[arm])
qg = qn.copy()
prev = None
for i in range(np.searchsorted(t, 216.0), np.searchsorted(t, 232.0), 25):
    j = min(i + 25, len(t) - 1)
    # 陀螺积分到 j
    qq = qn.copy()
    k0 = max(arm, i)
    for k in range(k0, j):
        w = g[k]
        if np.linalg.norm(w) > 1e-9:
            qq = qmul(qq, qaxis(w, np.linalg.norm(w) * 0.02))
    er, ep = euler(q[i:j].mean(axis=0))
    gr, gp = euler(qq)
    ax, ay = lp(s["acc_x"], 0.5)[i:j].mean(), lp(s["acc_y"], 0.5)[i:j].mean()
    ar = np.degrees(np.arctan2(ay, 1.0))
    ap = -np.degrees(np.arcsin(np.clip(ax, -1, 1)))
    gate = (np.abs(s.acc_mag[i:j] - 1.0) < (2.0 / 9.80665)).mean()
    out(f"  {t[i]:6.1f} {s['throttle'][i:j].mean():5.3f} | {er:+6.2f} {ep:+6.2f} | "
        f"{ar:+6.2f} {ap:+6.2f} | {gr:+6.2f} {gp:+6.2f} | {s.acc_mag[i:j].mean():4.2f} | "
        f"{100*gate:3.0f}% | {s['baro_rel_m'][i:j].mean():+6.2f}")

out("")
out("按油门分箱: 加速度计方向相对'陀螺积分姿态'的偏差(即加速度计方向的固定偏差)")
out("  油门区间  帧数  由加速度计的 roll/pitch   陀螺积分的 roll/pitch    差(acc - gyro)")
for lo, hi in [(0.0, 0.02), (0.05, 0.09), (0.09, 0.13), (0.13, 0.17), (0.17, 0.25)]:
    m = s.armed & (s["throttle"] >= lo) & (s["throttle"] < hi)
    if m.sum() < 200:
        continue
    idx = np.flatnonzero(m)
    # 用窗口起点作为四元数基准, 积分陀螺
    a0, b0 = idx[0], idx[-1]
    qq = q[a0] / np.linalg.norm(q[a0])
    grs, gps = [], []
    for k in range(a0, b0):
        w = g[k]
        if np.linalg.norm(w) > 1e-9:
            qq = qmul(qq, qaxis(w, np.linalg.norm(w) * 0.02))
        if m[k]:
            r, p = euler(qq)
            grs.append(r)
            gps.append(p)
    gr, gp = np.mean(grs), np.mean(gps)
    ax, ay = lp(s["acc_x"], 0.5)[m].mean(), lp(s["acc_y"], 0.5)[m].mean()
    ar = np.degrees(np.arctan2(ay, np.sqrt(np.maximum(1 - ax ** 2, 1e-6))))
    ap = -np.degrees(np.arcsin(np.clip(ax, -1, 1)))
    out(f"  {lo:.2f}~{hi:.2f}  {m.sum():5d}   {ar:+7.2f} {ap:+7.2f}      {gr:+7.2f} {gp:+7.2f}      "
        f"{ar-gr:+7.2f} {ap-gp:+7.2f}")

with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "second_flight.txt"),
          "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())
