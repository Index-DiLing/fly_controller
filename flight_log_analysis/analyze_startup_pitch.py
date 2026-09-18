"""专门看 session 4 第一次飞行"电机刚启动、还没离地"那几秒(162~169.5s)的 pitch 行为。

问题: 那会儿油门 0.08~0.12(悬停≈0.20), 推力只是部分抵消重力, 几乎还没有线加速度,
为什么 pitch 会开始变负(按约定 = 抬头)?
输出: 静态段(154~161.8) 与 162~169.5、169.5~171 的均值对比, 以及陀螺/加速度计/零偏在同一窗口的行为。
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
bi = np.flatnonzero(np.abs(s["gyro_bias_x"]) + np.abs(s["gyro_bias_y"]) + np.abs(s["gyro_bias_z"]) > 0)
bz = np.interp(np.arange(len(t)), bi, s["gyro_bias_z"][bi])


def stat(a0, a1, lbl):
    m = (t >= a0) & (t < a1)
    e = s.euler_deg[m]
    out(f"{lbl} ({a0:.1f}~{a1:.1f}s, {m.sum()} 帧):")
    out(f"   pitch 均值 {e[:,1].mean():+.3f}° (中位 {np.median(e[:,1]):+.3f}°, 首/末 {e[0,1]:+.3f}/{e[-1,1]:+.3f})")
    out(f"   roll  均值 {e[:,0].mean():+.3f}°    yaw 首/末 {e[0,2]:+.2f}/{e[-1,2]:+.2f}°")
    out(f"   油门均值 {s['throttle'][m].mean():.3f}  |a| 均值 {s.acc_mag[m].mean():.3f} g "
        f"(5~95% {np.percentile(s.acc_mag[m],5):.2f}~{np.percentile(s.acc_mag[m],95):.2f})")
    out(f"   陀螺均值 wx {s['gyro_x'][m].mean():+.4f} wy {s['gyro_y'][m].mean():+.4f} "
        f"wz {s['gyro_z'][m].mean():+.4f} rad/s   (= {np.degrees(s['gyro_y'][m].mean()):+.3f} °/s 俯仰)")
    out(f"   加速度均值 ax {s['acc_x'][m].mean():+.4f} ay {s['acc_y'][m].mean():+.4f} az {s['acc_z'][m].mean():+.4f} g"
        f"   -> 由加速度计的俯仰 = {-np.degrees(np.arcsin(np.clip(s['acc_x'][m].mean(),-1,1))):+.2f}°")
    out(f"   EKF 零偏均值 bz {np.degrees(bz[m].mean()):+.3f} °/s")
    # 四元数隐含角速度
    i0, i1 = np.searchsorted(t, a0), np.searchsorted(t, a1)
    if i1 - i0 > 25:
        rs = [implied(s.q[i], s.q[i + 25], 0.5) for i in range(i0, i1 - 25, 25)]
        rs = np.array(rs)
        out(f"   四元数隐含角速度均值 wx {rs[:,0].mean():+.4f} wy {rs[:,1].mean():+.4f} wz {rs[:,2].mean():+.4f} rad/s")
    out("")


stat(154.0, 161.8, "A 静态(解锁前, 桨已停/未启动)")
stat(161.8, 163.0, "B 电机启动最初 1.2s")
stat(163.0, 165.0, "C")
stat(165.0, 167.0, "D")
stat(167.0, 169.5, "E")
stat(169.5, 171.0, "F 开始离地")
stat(171.0, 173.0, "G")

out("逐 0.5s 细看 161.5~171.5s:")
out("   时间  油门  |a|    ax      ay     az   |  gyro wy   wz  | pitch   roll    yaw  | 隐含wy 隐含wz | bz(°/s)")
i0 = np.searchsorted(t, 161.5)
for i in range(i0, np.searchsorted(t, 171.5), 25):
    j = min(i + 25, len(t) - 1)
    im = implied(s.q[i], s.q[j], (j - i) * 0.02)
    out(f"  {t[i]:6.1f} {s['throttle'][i:j].mean():5.3f} {s.acc_mag[i:j].mean():5.2f} "
        f"{s['acc_x'][i:j].mean():+7.3f} {s['acc_y'][i:j].mean():+7.3f} {s['acc_z'][i:j].mean():+7.3f} | "
        f"{s['gyro_y'][i:j].mean():+8.4f} {s['gyro_z'][i:j].mean():+7.4f} | "
        f"{s.euler_deg[i:j,1].mean():+6.2f} {s.euler_deg[i:j,0].mean():+6.2f} {s.euler_deg[i:j,2].mean():+7.1f} | "
        f"{im[1]:+7.4f} {im[2]:+7.4f} | {np.degrees(bz[i:j].mean()):+6.2f}")

with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "startup_pitch.txt"),
          "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())
