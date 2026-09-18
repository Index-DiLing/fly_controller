"""量化: 日志里的姿态变化有多少是陀螺积分出来的, 有多少是 EKF 用加速度计"拧"过去的。

欧拉运动学(与 flight_controller 无关, 纯刚体运动学):
  droll/dt  = wx + tan(pitch)*(wy*sin(roll) + wz*cos(roll))
  dpitch/dt = wy*cos(roll) - wz*sin(roll)
把等式右边用实测陀螺积分得到"陀螺该给的角度变化", 再与四元数解出的角度变化比较,
差值就是"非陀螺来源"(EKF 加速度计修正 + 数值) —— 正常情况下这部分应当很小(滤波器已收敛)。
"""
from __future__ import annotations

import io
import os

import numpy as np

from analyze import segments
from load_logs import load_all, out_path

OUT = io.StringIO()


def out(*a):
    print(*a)
    print(*a, file=OUT)


def gyro_expected(s):
    e = np.radians(s.euler_deg)
    r, p = e[:, 0], e[:, 1]
    wx, wy, wz = s["gyro_x"], s["gyro_y"], s["gyro_z"]
    droll = wx + np.tan(p) * (wy * np.sin(r) + wz * np.cos(r))
    dpitch = wy * np.cos(r) - wz * np.sin(r)
    dt = 0.02
    return np.cumsum(droll) * dt * 57.29578, np.cumsum(dpitch) * dt * 57.29578


out("窗口内 姿态角变化 vs 陀螺积分(单位: 度)。 差值 = EKF 用加速度计把姿态拧过去的量")
out("(正负号按日志自身约定, 只看量级; 差值大 = 姿态估计不是靠陀螺推出来的)")
# 自动挑选窗口: 每个会话取"最长的解锁段"的前 7s 与后 7s; 另外取最长的一段做对照
WINDOWS = []
_sess = load_all()
for s in _sess:
    segs = segments(s.armed)
    if not segs:
        continue
    a, z = max(segs, key=lambda p: p[1] - p[0])
    t0, t1 = s.t[a], s.t[z - 1]
    WINDOWS.append((s.session_id, t0, min(t0 + 7.0, t1), f"s{s.session_id} 解锁段前 7s"))
    if t1 - t0 > 10.0:
        WINDOWS.append((s.session_id, max(t1 - 7.0, t0), t1, f"s{s.session_id} 解锁段后 7s"))
for s in _sess:
    segs = segments(s.armed)
    if segs:
        a, z = max(segs, key=lambda p: p[1] - p[0])
        WINDOWS.append((s.session_id, s.t[a], s.t[z - 1], f"s{s.session_id} 最长解锁段(对照)"))
for sid, t0, t1, lbl in WINDOWS:
    s = [x for x in load_all() if x.session_id == sid][0]
    t = s.t
    a, b = np.searchsorted(t, t0), np.searchsorted(t, t1)
    if b - a < 20:
        continue
    eg, ep = gyro_expected(s)
    droll_q = s.euler_deg[b - 1, 0] - s.euler_deg[a, 0]
    dpitch_q = s.euler_deg[b - 1, 1] - s.euler_deg[a, 1]
    droll_g = eg[b - 1] - eg[a]
    dpitch_g = ep[b - 1] - ep[a]
    out(f"{lbl:26s} s{sid} {t0:6.1f}~{t1:6.1f}s: "
        f"roll 四元数 {droll_q:+7.2f}° vs 陀螺 {droll_g:+7.2f}° (差 {droll_q-droll_g:+7.2f}°) | "
        f"pitch 四元数 {dpitch_q:+7.2f}° vs 陀螺 {dpitch_g:+7.2f}° (差 {dpitch_q-dpitch_g:+7.2f}°)")

out("")
out("逐 0.5s 的差值序列(每会话最长解锁段的前 8s), 看加速度计修正在什么时候把姿态拧走")
for s in _sess:
    segs = segments(s.armed)
    if not segs:
        continue
    a0, z0 = max(segs, key=lambda p: p[1] - p[0])
    sid, t0, t1 = s.session_id, s.t[a0], min(s.t[a0] + 8.0, s.t[z0 - 1])
    s = [x for x in load_all() if x.session_id == sid][0]
    t = s.t
    eg, ep = gyro_expected(s)
    out(f"\ns{sid}:  时间   Δpitch(四元数)  Δpitch(陀螺)  差值    |a|均值  | 油门")
    a = np.searchsorted(t, t0)
    for i in range(a, np.searchsorted(t, t1), 25):
        j = min(i + 25, len(t) - 1)
        out(f"      {t[i]:6.1f}   {s.euler_deg[j,1]-s.euler_deg[i,1]:+12.2f}  "
            f"{ep[j]-ep[i]:+12.2f}  {(s.euler_deg[j,1]-s.euler_deg[i,1])-(ep[j]-ep[i]):+7.2f}   "
            f"{s.acc_mag[i:j].mean():6.2f}  {s['throttle'][i:j].mean():5.3f}")

out("")
out("倾角修正门限(||a|-g|<2.0 m/s^2, 即 0.204g)的开门比例, 与高油门段非陀螺姿态变化速率")
for s in load_all():
    a = s.acc_mag
    ar = s.armed
    gate = np.abs(a - 1.0) < (2.0 / 9.80665)
    if ar.sum() < 200:
        out(f"  s{s.session_id} 未解锁: 开门 {100*gate.mean():.0f}%")
        continue
    _eg, ep = gyro_expected(s)
    res = np.abs(np.diff(s.euler_deg[:, 1]) - np.diff(ep)) * 50.0      # deg/s
    hi = ar[1:] & (s["throttle"][1:] > 0.12)
    if hi.any():
        out(f"  s{s.session_id}: 解锁段开门 {100*gate[ar].mean():.0f}% (|a| 中位 {np.median(a[ar]):.2f} g); "
            f"高油门段非陀螺 pitch 变化速率 中位 {np.median(res[hi]):.1f} °/s, "
            f"p95 {np.percentile(res[hi],95):.1f} °/s")

with open(out_path("ekf_drag_check"),
          "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())

