"""把"真实姿态(陀螺积分)、EKF 姿态、加速度计指向"三条线摆在一起。

做法:
  - 锚点: 解锁前的静止段(最后 2s)里, 陀螺零偏可由实测均值估计, 且此刻
    "EKF 姿态 == 真实姿态"(静止时加速度计是有效重力计)。
  - 从锚点起用(去偏)陀螺积分得到真实姿态 θ_gyro —— 陀螺不会骗人(短时间)。
  - 加速度计指向 θ_acc: 把实测比力方向当成重力, 反推出的姿态角(这正是 EKF 的观测量来源)。
  - 比较三者: 若 θ_acc 与 θ_gyro-θ_ekf 的符号相反, 说明"加速度计修正"把估计往真值反方向拉,
    控制器于是越修越歪(正反馈), 飞机表现为"稳定倾斜、电机不修正"。
"""
from __future__ import annotations

import io

import numpy as np

from analyze import segments
from load_logs import load_all, out_path

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


def integrate_gyro(q0, w, dt=0.02):
    """用机体角速度积分四元数(右乘), 返回每个时刻的四元数。"""
    out_q = np.empty((len(w), 4))
    q = q0.copy()
    out_q[0] = q
    for i in range(1, len(w)):
        om = w[i - 1]
        n = np.linalg.norm(om)
        if n > 1e-9:
            axis = om / n
            ang = n * dt
            dq = np.array([np.cos(ang / 2), *(axis * np.sin(ang / 2))])
            q = quat_mul(q, dq)
            q = q / np.linalg.norm(q)
        out_q[i] = q
    return out_q


def euler_deg(q):
    w, x, y, z = q
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    return roll, pitch


for s in load_all():
    segs = segments(s.armed)
    if not segs:
        continue
    a0, z0 = max(segs, key=lambda p: p[1] - p[0])
    t = s.t
    # 锚点: 解锁前 2s 的静止段
    pre = (t > t[a0] - 2.5) & (t < t[a0])
    if pre.sum() < 50:
        continue
    bias = np.array([np.median(s["gyro_x"][pre]), np.median(s["gyro_y"][pre]), np.median(s["gyro_z"][pre])])
    q0 = s.q[a0]
    w = np.stack([s["gyro_x"], s["gyro_y"], s["gyro_z"]], axis=1)[a0:z0] - bias
    qg = integrate_gyro(q0, w)
    rk, pk = euler_deg(qg.T)
    r_e, p_e = s.euler_deg[a0:z0, 0], s.euler_deg[a0:z0, 1]
    f = np.stack([s["acc_x"], s["acc_y"], s["acc_z"]], axis=1)[a0:z0]
    # 加速度计指向反推的姿态(把比力当重力)
    p_a = -np.degrees(np.arctan2(f[:, 0], np.sqrt(f[:, 1] ** 2 + f[:, 2] ** 2)))
    r_a = np.degrees(np.arctan2(f[:, 1], f[:, 2]))
    out("=" * 108)
    out(f"session {s.session_id} 解锁段 {t[a0]:.1f}~{t[z0-1]:.1f}s (锚点陀螺零偏 "
        f"{np.degrees(bias[0]):+.2f}/{np.degrees(bias[1]):+.2f}/{np.degrees(bias[2]):+.2f} °/s)")
    out("  t     油门   pitch: 陀螺积分  EKF   加速度计 |  roll: 陀螺积分  EKF   加速度计 | "
        "EKF-真实(pitch/roll)  ACC-真实(pitch/roll)")
    for i in range(0, len(rk), 50):
        out(f"{t[a0]+i*0.02:7.1f} {s['throttle'][a0+i]:5.3f}   {pk[i]:8.2f} {p_e[i]:7.2f} {p_a[i]:8.2f} | "
            f"{rk[i]:8.2f} {r_e[i]:7.2f} {r_a[i]:8.2f} | {p_e[i]-pk[i]:+8.2f}/{r_e[i]-rk[i]:+8.2f}  "
            f"{p_a[i]-pk[i]:+8.2f}/{r_a[i]-rk[i]:+8.2f}")
    out("")

with open(out_path("truth_check"), "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())
