"""逐帧检验: 四元数本身隐含的角速度(相邻两帧的姿态差/dt) 与 同一条日志里的陀螺是否一致。

这条检验与任何欧拉角约定无关: q(t+dt)=q(t)*exp(ω dt/2), 于是
  Δq = conj(q_i) * q_{i+1}  ->  旋转轴*角/dt 就是"四元数自己认为的机体角速度"。
如果它与同一时刻记录的 bodyRate 不符, 说明姿态估计没有跟着陀螺走(被别的观测拖走)。
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


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def implied_rate(q, dt=0.02):
    """q: (n,4) -> (n-1,3) 角速度 [rad/s]"""
    r = np.empty((len(q) - 1, 3))
    for i in range(len(q) - 1):
        a = q[i] / np.linalg.norm(q[i])
        b = q[i + 1] / np.linalg.norm(q[i + 1])
        d = quat_mul(np.array([a[0], -a[1], -a[2], -a[3]]), b)   # conj(a)*b, 机体系
        if d[0] < 0:
            d = -d
        v = d[1:4]
        n = np.linalg.norm(v)
        ang = 2.0 * np.arctan2(n, d[0])
        r[i] = (v / n * ang / dt) if n > 1e-9 else 0.0
    return r


out("逐帧: 四元数隐含角速度 vs 日志里的陀螺 (rad/s)。两者应当几乎相等")
out("(若某项差很多, 说明姿态估计在那一帧没有跟着陀螺走)")
for sid, t0, t1, lbl in [(4, 161.8, 190.0, "s4 第一次飞行(含 166~176 加速段)"),
                         (4, 216.6, 245.0, "s4 第二次飞行"),
                         (6, 43.8, 66.3, "s6 飞行"),
                         (8, 294.6, 313.0, "s8 最后一次"),
                         (5, 53.4, 80.8, "s5 台架(对照)"),
                         (8, 170.1, 250.0, "s8 台架(对照)")]:
    s = [x for x in load_all() if x.session_id == sid][0]
    t = s.t
    a, b = np.searchsorted(t, t0), np.searchsorted(t, t1)
    q = s.q[a:b]
    r = implied_rate(q)
    g = np.stack([s["gyro_x"], s["gyro_y"], s["gyro_z"]], axis=1)[a:b - 1]
    err = np.abs(r - g)
    out(f"\n{lbl}: {t[a]:.1f}~{t[b-1]:.1f}s ({len(r)} 帧)")
    for i, nm in enumerate(("x", "y", "z")):
        c = np.corrcoef(r[:, i], g[:, i])[0, 1]
        out(f"   轴 {nm}: corr(四元数隐含, 陀螺) = {c:+0.3f};  |差| 中位 {np.median(err[:,i]):6.3f} "
            f"p95 {np.percentile(err[:,i],95):6.3f} rad/s;  四元数隐含 |ω| 中位 {np.median(np.abs(r[:,i])):.3f} "
            f"vs 陀螺 {np.median(np.abs(g[:,i])):.3f}")

out("\n" + "=" * 90)
out("s4 160~180s 逐 0.5s: 四元数隐含角速度(平均) vs 陀螺(平均)")
s = [x for x in load_all() if x.session_id == 4][0]
t = s.t
q = s.q
r = implied_rate(q)
g = np.stack([s["gyro_x"], s["gyro_y"], s["gyro_z"]], axis=1)
out("   时间   油门   | 四元数隐含 wx,wy,wz        | 陀螺 wx,wy,wz             | acc_x  acc_z")
a = np.searchsorted(t, 161.0)
for i in range(a, np.searchsorted(t, 180.0), 25):
    j = min(i + 25, len(r) - 1)
    out(f"  {t[i]:6.1f} {s['throttle'][i:j].mean():5.3f}  | "
        f"{r[i:j,0].mean():+6.3f} {r[i:j,1].mean():+6.3f} {r[i:j,2].mean():+6.3f}  | "
        f"{g[i:j,0].mean():+6.3f} {g[i:j,1].mean():+6.3f} {g[i:j,2].mean():+6.3f}  | "
        f"{s['acc_x'][i:j].mean():+6.2f} {s['acc_z'][i:j].mean():+6.2f}")

with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "quat_vs_gyro.txt"),
          "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())
