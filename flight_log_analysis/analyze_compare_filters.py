"""同一段日志上离线重放: Madgwick(IMU) vs EKF(日志里的四元数) vs 陀螺积分(真值参考)。

为什么能做这个对比:
  - 日志里同时有 陀螺/加速度计(50Hz 采样) 和 EKF 输出的四元数;
  - Madgwick 的修正量是 beta*s(|s|=1), 对应"姿态修正角速度上限 = 2*beta", 与采样率无关,
    所以用 50Hz 重放(dt=0.02)能还原它真实的修正速率;
  - "真值参考" = 用解锁前静止段测得的陀螺零偏去偏后积分(40s 内误差 <1°)。

对比指标: 各滤波器相对真值参考的姿态误差(倾角), 以及"加速度计修正把真实转动抵消掉多少"。
"""
from __future__ import annotations

import io

import numpy as np

from analyze import segments
from analyze_truth import euler_deg, integrate_gyro, quat_mul
from load_logs import load_all, out_path

OUT = io.StringIO()


def out(*a):
    print(*a)
    print(*a, file=OUT)


def madgwick_imu(gyro, accel, dt, beta, q0=None):
    """Madgwick 的 IMU-only 版本(DL_LIB/DL_AHRS/MadgwickAHRS.hpp 的 1:1 移植)。"""
    q = np.array([1.0, 0.0, 0.0, 0.0]) if q0 is None else np.array(q0, dtype=float)
    q = q / np.linalg.norm(q)
    hist = np.empty((len(gyro), 4))
    for i in range(len(gyro)):
        gx, gy, gz = gyro[i]
        ax, ay, az = accel[i]
        q0_, q1, q2, q3 = q
        qDot = np.array([0.5 * (-q1 * gx - q2 * gy - q3 * gz),
                         0.5 * (q0_ * gx + q2 * gz - q3 * gy),
                         0.5 * (q0_ * gy - q1 * gz + q3 * gx),
                         0.5 * (q0_ * gz + q1 * gy - q2 * gx)])
        n = np.linalg.norm(accel[i])
        if n > 1e-6:
            ax, ay, az = ax / n, ay / n, az / n
            _2q0, _2q1, _2q2, _2q3 = 2 * q0_, 2 * q1, 2 * q2, 2 * q3
            _4q0, _4q1, _4q2 = 4 * q0_, 4 * q1, 4 * q2
            _8q1, _8q2 = 8 * q1, 8 * q2
            q0q0, q1q1, q2q2, q3q3 = q0_ * q0_, q1 * q1, q2 * q2, q3 * q3
            s = np.array([
                _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay,
                _4q1 * q3q3 - _2q3 * ax + 4 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az,
                4 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az,
                4 * q1q1 * q3 - _2q1 * ax + 4 * q2q2 * q3 - _2q2 * ay])
            ns = np.linalg.norm(s)
            if ns > 1e-9:
                qDot = qDot - beta * (s / ns)
        q = q + qDot * dt
        q = q / np.linalg.norm(q)
        hist[i] = q
    return hist


out("=" * 112)
out("同一段日志上的离线重放对比(以去偏陀螺积分为真值参考)")
out("=" * 112)
for s in load_all():
    segs = segments(s.armed)
    if not segs:
        continue
    a0, z0 = max(segs, key=lambda p: p[1] - p[0])
    t = s.t
    pre = (t > t[a0] - 2.5) & (t < t[a0])
    if pre.sum() < 50:
        continue
    w = np.stack([s["gyro_x"], s["gyro_y"], s["gyro_z"]], axis=1)
    acc = np.stack([s["acc_x"], s["acc_y"], s["acc_z"]], axis=1)
    bias_true = np.array([np.median(w[pre, 0]), np.median(w[pre, 1]), np.median(w[pre, 2])])
    # 真值参考: 去偏陀螺积分
    q_truth = integrate_gyro(s.q[a0], w[a0:z0] - bias_true)
    # EKF: 日志里的四元数
    q_ekf = s.q[a0:z0]
    # Madgwick: 用原始陀螺(它不做零偏估计, 靠 beta 反馈)
    q_mad = madgwick_imu(w[a0:z0], acc[a0:z0], 0.02, 0.078, q0=s.q[a0])
    q_mad_low = madgwick_imu(w[a0:z0], acc[a0:z0], 0.02, 0.02, q0=s.q[a0])
    _, p_truth = euler_deg(q_truth.T)
    _, p_ekf = euler_deg(q_ekf.T)
    _, p_mad = euler_deg(q_mad.T)
    _, p_mad_low = euler_deg(q_mad_low.T)
    r_truth, _ = euler_deg(q_truth.T)
    r_ekf, _ = euler_deg(q_ekf.T)
    r_mad, _ = euler_deg(q_mad.T)
    r_mad_low, _ = euler_deg(q_mad_low.T)
    dt = t[a0:z0]
    out(f"\nsession {s.session_id}: 解锁段 {t[a0]:.1f}~{t[z0-1]:.1f}s ({t[z0-1]-t[a0]:.1f}s), "
        f"锚点零偏 {np.degrees(bias_true[0]):+.2f}/{np.degrees(bias_true[1]):+.2f}/{np.degrees(bias_true[2]):+.2f} °/s")
    out("   滤波器          结束时 pitch(相对真值) 结束时 roll(相对真值)  姿态误差 RMS(pitch)  姿态误差 RMS(roll)")
    for name, p, r in (("陀螺积分(真值参考)", p_truth, r_truth),
                       ("EKF(日志)", p_ekf, r_ekf),
                       ("Madgwick β=0.078", p_mad, r_mad),
                       ("Madgwick β=0.020", p_mad_low, r_mad_low)):
        ep = np.array(p) - np.array(p_truth)
        er = np.array(r) - np.array(r_truth)
        out(f"   {name:20s} {p[-1]-p_truth[-1]:+10.2f}° {r[-1]-r_truth[-1]:+18.2f}° "
            f"{np.sqrt((ep**2).mean()):16.2f}° {np.sqrt((er**2).mean()):18.2f}°")
    out("   逐 4s 的 pitch: 真值 / EKF / Madgwick(0.078) / Madgwick(0.02)")
    line = []
    for i in range(0, len(p_truth), 200):
        line.append(f"   t={dt[i]:6.1f}  {p_truth[i]:+7.2f} / {p_ekf[i]:+7.2f} / {p_mad[i]:+7.2f} / {p_mad_low[i]:+7.2f}")
    out("\n".join(line))

with open(out_path("filter_compare"), "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())
