"""三条"倾角"来源并排, 判断到底是谁在漂:
   acc_tilt   : 比力与机体 z 的夹角 —— 飞机静止(或匀速)时它=真实倾角
   gyro_tilt  : 用解锁前静止段零偏去偏后, 陀螺积分得到的姿态的倾角 —— 短时可信的"惯性真值"
   ekf_tilt   : 日志四元数(滤波器输出)的倾角
再把这些差值与 时间/油门 做相关, 用来分辨:
   - 若 (gyro − acc) 随时间线性增长 ⇒ 陀螺零偏在漂(温漂/电磁)
   - 若 (gyro − acc) 随油门增长       ⇒ 加速度计被推力/振动带偏
"""
from __future__ import annotations

import io

import numpy as np

from analyze import segments
from analyze_truth import euler_deg, integrate_gyro
from load_logs import load_all, out_path

OUT = io.StringIO()


def out(*a):
    print(*a)
    print(*a, file=OUT)


def tilt_of_q(q):
    q = q / np.linalg.norm(q, axis=1)[:, None]
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    upx = 2 * (x * z - y * w)
    upy = 2 * (y * z + x * w)
    upz = 1 - 2 * (x * x + y * y)
    return np.degrees(np.arccos(np.clip(upz, -1, 1))), upx, upy, upz


for s in load_all():
    segs = segments(s.armed)
    if not segs:
        continue
    a0, z0 = max(segs, key=lambda p: p[1] - p[0])
    t = s.t
    pre = (t > t[a0] - 2.5) & (t < t[a0])
    if pre.sum() < 30:
        continue
    w = np.stack([s["gyro_x"], s["gyro_y"], s["gyro_z"]], axis=1)
    bias = np.array([np.median(w[pre, 0]), np.median(w[pre, 1]), np.median(w[pre, 2])])
    qg = integrate_gyro(s.q[a0], w[a0:z0] - bias)
    g_tilt = tilt_of_q(qg)[0]
    e_tilt = tilt_of_q(s.q[a0:z0])[0]
    f = np.stack([s["acc_x"], s["acc_y"], s["acc_z"]], axis=1)[a0:z0]
    a_tilt = np.degrees(np.arctan2(np.hypot(f[:, 0], f[:, 1]), f[:, 2]))
    thr = s["throttle"][a0:z0]
    amag = np.linalg.norm(f, axis=1)
    tt = t[a0:z0] - t[a0]
    out("=" * 100)
    out(f"session {s.session_id} 解锁段 {t[a0]:.1f}~{t[z0-1]:.1f}s")
    out("   t    油门  |a|   acc_tilt  gyro_tilt  ekf_tilt |  gyro-acc   ekf-acc")
    for i in range(0, len(tt), 50):
        out(f"{tt[i]:6.1f} {thr[i]:6.3f} {amag[i]:5.2f} {a_tilt[i]:8.2f} {g_tilt[i]:10.2f} {e_tilt[i]:9.2f} | "
            f"{g_tilt[i]-a_tilt[i]:+9.2f} {e_tilt[i]-a_tilt[i]:+8.2f}")
    # 相关性: 差值与时间 / 油门
    d_ga = g_tilt - a_tilt
    d_ea = e_tilt - a_tilt
    out(f"   corr(gyro_tilt-acc_tilt, t)      = {np.corrcoef(d_ga, tt)[0,1]:+.2f}   "
        f"corr(..., 油门) = {np.corrcoef(d_ga, thr)[0,1]:+.2f}")
    out(f"   corr(ekf_tilt -acc_tilt, t)      = {np.corrcoef(d_ea, tt)[0,1]:+.2f}   "
        f"corr(..., 油门) = {np.corrcoef(d_ea, thr)[0,1]:+.2f}")
    out(f"   末值: acc {a_tilt[-1]:.1f}°  gyro {g_tilt[-1]:.1f}°  ekf {e_tilt[-1]:.1f}°")

with open(out_path("three_ways"), "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())
