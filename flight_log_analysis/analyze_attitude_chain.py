"""把"姿态角为什么和真实相反/不合理"的链条逐条量化。

C1 姿态的转动 = (陀螺 - 零偏估计) 吗?  —— 逐秒比较四元数隐含角速度 与 (g - b)
C2 零偏估计跑了多少? 真实零偏是多少(用桨停车的静止段量)
C3 加速度计倾角修正在飞行段"开门"的比例, 与它可能注入的姿态偏差
C4 两次飞行的对照(是否同一模式)
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


def implied(qa, qb, dt):
    a = qa / np.linalg.norm(qa)
    b = qb / np.linalg.norm(qb)
    d = qmul(np.array([a[0], -a[1], -a[2], -a[3]]), b)
    if d[0] < 0:
        d = -d
    v = d[1:4]
    n = np.linalg.norm(v)
    ang = 2.0 * np.arctan2(n, d[0])
    return v / n * ang / dt if n > 1e-9 else np.zeros(3)


s = [x for x in load_all() if x.session_id == 4][0]
t = s.t
q = s.q
g = np.stack([s["gyro_x"], s["gyro_y"], s["gyro_z"]], axis=1)
bi = np.flatnonzero(np.abs(s["gyro_bias_x"]) + np.abs(s["gyro_bias_y"]) + np.abs(s["gyro_bias_z"]) > 0)
bx = np.interp(np.arange(len(t)), bi, s["gyro_bias_x"][bi])
by = np.interp(np.arange(len(t)), bi, s["gyro_bias_y"][bi])
bz = np.interp(np.arange(len(t)), bi, s["gyro_bias_z"][bi])

out("C1  session4 161~190s(第一次飞行): 每秒 四元数隐含角速度  vs  (陀螺 - 零偏估计)")
out("     时间  油门 |  隐含 wx     wy     wz  |  (g-b) wx    wy     wz  |  差(加速度计修正+其他)")
W = 50
for i in range(np.searchsorted(t, 161.0), np.searchsorted(t, 190.0), W):
    if i + W >= len(t):
        break
    r = implied(q[i], q[i + W], W * 0.02)
    gg = np.array([g[i:i + W, 0].mean() - bx[i:i + W].mean(),
                   g[i:i + W, 1].mean() - by[i:i + W].mean(),
                   g[i:i + W, 2].mean() - bz[i:i + W].mean()])
    out(f"   {t[i]:6.1f} {s['throttle'][i]:5.3f} | {r[0]:+7.3f} {r[1]:+7.3f} {r[2]:+7.3f} | "
        f"{gg[0]:+7.3f} {gg[1]:+7.3f} {gg[2]:+7.3f} | "
        f"{r[0]-gg[0]:+6.3f} {r[1]-gg[1]:+6.3f} {r[2]-gg[2]:+6.3f}")

out("")
out("C2 零偏估计 vs 真实零偏")
for sid in (4, 5, 6, 8):
    s2 = [x for x in load_all() if x.session_id == sid][0]
    t2 = s2.t
    idx = np.flatnonzero(np.abs(s2["gyro_bias_x"]) + np.abs(s2["gyro_bias_y"]) + np.abs(s2["gyro_bias_z"]) > 0)
    # 真实零偏: 桨停车的静止段(未解锁、角速度很小)里陀螺读数均值
    stat = (~s2.armed) & (s2.gyro_mag < 0.05) & (np.abs(s2.acc_mag - 1) < 0.05)
    true_b = [float(np.degrees(np.median(s2[k][stat]))) if stat.sum() > 200 else float("nan")
              for k in ("gyro_x", "gyro_y", "gyro_z")]
    out(f"  s{sid}: 真实零偏(桨停车静止段) = {true_b[0]:+.2f}, {true_b[1]:+.2f}, {true_b[2]:+.2f} deg/s"
        f"  |  EKF 估计末值 = {np.degrees(s2['gyro_bias_x'][idx[-1]]):+.2f}, "
        f"{np.degrees(s2['gyro_bias_y'][idx[-1]]):+.2f}, {np.degrees(s2['gyro_bias_z'][idx[-1]]):+.2f} deg/s"
        f"  |  最大 = {np.degrees(np.abs(s2['gyro_bias_x'][idx]).max()):.2f}, "
        f"{np.degrees(np.abs(s2['gyro_bias_y'][idx]).max()):.2f}, {np.degrees(np.abs(s2['gyro_bias_z'][idx]).max()):.2f}")

out("")
out("C3 加速度计倾角修正门限(||a|-g|<2.0 m/s²)在飞行段的开门比例")
for sid in (4, 6, 8):
    s2 = [x for x in load_all() if x.session_id == sid][0]
    a = s2.acc_mag
    gate = np.abs(a - 1.0) < (2.0 / 9.80665)
    armed = s2.armed
    segs = []
    idx = np.flatnonzero(np.diff(armed.astype(int)) != 0) + 1
    b = [0, *idx, len(armed)]
    for x, z in zip(b[:-1], b[1:]):
        if armed[x]:
            segs.append((s2.t[x], s2.t[z - 1], 100 * gate[x:z].mean(), a[x:z].mean()))
    out(f"  s{sid}:")
    for x, z, gp, am in segs:
        out(f"     解锁段 {x:6.1f}~{z:6.1f}s: 门开 {gp:.0f}%, |a| 均值 {am:.2f} g")

out("")
out("C4 四元数隐含角速度的峰值 vs 陀螺峰值(检验标定/采样是否一致)")
for sid, t0, t1, lbl in [(4, 161.8, 190.0, "s4 第一次飞行"), (4, 216.6, 245.0, "s4 第二次飞行")]:
    s2 = [x for x in load_all() if x.session_id == sid][0]
    t2 = s2.t
    a, b = np.searchsorted(t2, t0), np.searchsorted(t2, t1)
    gg = np.stack([s2["gyro_x"], s2["gyro_y"], s2["gyro_z"]], axis=1)[a:b]
    rr = np.array([implied(s2.q[i - 1], s2.q[i + 1], 0.04) for i in range(a + 1, b - 1)])
    gm = np.linalg.norm(gg[1:-1], axis=1)
    rm = np.linalg.norm(rr, axis=1)
    k = int(np.argmax(gm))
    out(f"  {lbl}: 陀螺峰值 {gm.max():.2f} rad/s @ t={t2[a+1+k]:.2f}s, 同刻四元数隐含 {rm[k]:.2f} rad/s; "
        f"帧间相关 corr(|隐含|,|陀螺|) = {np.corrcoef(rm, gm)[0,1]:+.2f}; "
        f"中位比 {np.median(rm/gm):.2f}")

with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "attitude_chain.txt"),
          "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())
