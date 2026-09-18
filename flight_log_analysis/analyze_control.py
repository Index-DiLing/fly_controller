"""控制链路的符号/轴向一致性检查(针对"姿态不修正、起步就漂"的怀疑)。

用日志能直接做的三个测试:
  T1 电机模式 -> 实测角加速度: 纯物理测试, 完全不经控制器公式。
     按混控几何把四路电机换算成三轴"物理力矩"(M0左前/M1右前/M2右后/M3左后),
     再看它与实测角加速度 dω/dt 的相关性。符号一致 = 电机排序/旋向/陀螺轴向自洽。
  T2 控制器链路复算: 用日志的四元数 + 目标姿态, 按 flight_controller.hpp 的
     倾转分离算法复算 rate_setpoint, 与日志里的 rate_setpoint 对比(验证理解无误),
     并检查"误差方向 -> 期望力矩"是否真的指向缩小误差。
  T3 姿态估计偏差: 静/准静态(加速度≈1g、角速度小)时用加速度计方向当重力基准,
     与 EKF 姿态比较, 得到"估计偏差向量"(机体轴), 这就是飞机被控到倾斜多少。
"""
from __future__ import annotations

import io
import os
import sys

import numpy as np

from analyze import segments
from load_logs import load_all, out_path

OUT = io.StringIO()


def out(*a):
    try:
        print(*a)
    except UnicodeEncodeError:
        print(*a, file=sys.stderr, errors="replace")
    print(*a, file=OUT)

MASS, HOVER = 1.566, 0.25
ARM_F, ARM_L, YAW_ARM = 0.12933, 0.18106, 0.03
K = (MASS * 9.80665 / HOVER) / 4.0


def phys_torque(motor_count: np.ndarray) -> np.ndarray:
    """四路 DShot -> 物理力矩(相对量, N*m)。

    列顺序: motor0=M0左前, motor1=M3左后, motor2=M1右前, motor3=M2右后
    (kConfig.motorMap={0,2,3,1} 的后果)。
    """
    frac = (motor_count - 50.0) / 1900.0
    m0, m3, m1, m2 = frac[:, 0], frac[:, 1], frac[:, 2], frac[:, 3]
    tx = 4 * ARM_L * K * (m0 + m3 - m1 - m2) / 4      # 左 - 右
    ty = 4 * ARM_F * K * (m2 + m3 - m0 - m1) / 4      # 后 - 前
    tz = 4 * YAW_ARM * K * (m0 + m2 - m1 - m3) / 4    # 顺 - 逆
    return np.stack([tx, ty, tz], axis=1)


def smooth(x, n):
    if n <= 1:
        return x
    ker = np.ones(n) / n
    if x.ndim == 1:
        return np.convolve(x, ker, "same")
    return np.stack([np.convolve(x[:, i], ker, "same") for i in range(x.shape[1])], axis=1)


out("=" * 100)
out("T1 电机模式 -> 实测角加速度 的符号一致性 (机体三轴)")
out("=" * 100)
for s in load_all():
    ar = s.armed
    if ar.sum() < 100:
        continue
    dt = 0.02
    tp = phys_torque(s.motor)
    w = np.stack([s["gyro_x"], s["gyro_y"], s["gyro_z"]], axis=1)
    # 角加速度: 20ms 相邻差分噪声大, 用 5 点(100ms)平滑后的差分
    acc_meas = np.gradient(smooth(w, 5), dt, axis=0)
    out(f"\n--- session {s.session_id} (解锁 {ar.sum()} 帧)")
    names = ["x 绕前后轴(roll)", "y 绕左右轴(pitch)", "z 绕垂直轴(yaw)"]
    for ax in range(3):
        m = ar & (np.abs(tp[:, ax]) > np.percentile(np.abs(tp[ar, ax]), 50))
        if m.sum() < 30:
            out(f"   {names[ax]}: 样本不足")
            continue
        c_all = np.corrcoef(tp[ar, ax], acc_meas[ar, ax])[0, 1]
        c_hi = np.corrcoef(tp[m, ax], acc_meas[m, ax])[0, 1]
        out(f"   {names[ax]}: 全解锁帧相关 {c_all:+.3f}; 大力矩帧 {c_hi:+.3f}  "
              f"(力矩>0 时实测角加速度均值 {acc_meas[m, ax].mean():+.2f} rad/s², "
              f"力矩<0 时 {acc_meas[ar & (tp[:, ax] < 0), ax].mean():+.2f} rad/s²)")

out()
out("=" * 100)
out("T3 姿态估计偏差(加速度计当基准) —— 静态/准静态窗口")
out("=" * 100)
for s in load_all():
    w = s.gyro_mag
    a = s.acc_mag
    # 准静态: 角速度小、合加速度接近 1g
    qs = (w < 0.25) & (np.abs(a - 1.0) < 0.15)
    idx = np.flatnonzero(qs)
    if len(idx) < 200:
        continue
    # 用四元数把实测比力转到世界系: 静态时它应当只指向 +z(即 1g 朝上)
    q = s.q / np.linalg.norm(s.q, axis=1)[:, None]
    w_, x_, y_, z_ = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    # R^T * a_body  (机体->世界的转置作用在比力上)
    R = np.stack([
        np.stack([1 - 2 * (y_ ** 2 + z_ ** 2), 2 * (x_ * y_ - z_ * w_), 2 * (x_ * z_ + y_ * w_)], axis=1),
        np.stack([2 * (x_ * y_ + z_ * w_), 1 - 2 * (x_ ** 2 + z_ ** 2), 2 * (y_ * z_ - x_ * w_)], axis=1),
        np.stack([2 * (x_ * z_ - y_ * w_), 2 * (y_ * z_ + x_ * w_), 1 - 2 * (x_ ** 2 + y_ ** 2)], axis=1),
    ], axis=1)
    a_body = np.stack([s["acc_x"], s["acc_y"], s["acc_z"]], axis=1)
    a_world = np.einsum("nij,nj->ni", np.transpose(R, (0, 2, 1)), a_body)
    sel = a_world[idx]
    tilt_x = np.degrees(np.arctan2(sel[:, 0], sel[:, 2]))
    tilt_y = np.degrees(np.arctan2(sel[:, 1], sel[:, 2]))
    out(f"session {s.session_id}: 准静态帧 {len(idx)} ({100*qs.mean():.0f}%), "
          f"实测重力在机体轴上的倾斜 = ({np.median(tilt_x):+.2f}°, {np.median(tilt_y):+.2f}°), "
          f"5~95 分位 x[{np.percentile(tilt_x,5):+.1f},{np.percentile(tilt_x,95):+.1f}] "
          f"y[{np.percentile(tilt_y,5):+.1f},{np.percentile(tilt_y,95):+.1f}]")

print()
print("=" * 100)
print("T4 由加速度计推出的真实姿态 vs EKF 姿态  (Δ = 加速度计 - EKF; 残差倾斜会让飞机持续平移)")
print("    约定: roll>0 = 右侧下沉(往右漂), pitch>0 = 机头下沉(往前漂)")
print("=" * 100)
for s in load_all():
    t = s.t
    f_body = np.stack([s["acc_x"], s["acc_y"], s["acc_z"]], axis=1)
    amag = np.linalg.norm(f_body, axis=1)
    roll_a = np.degrees(np.arctan2(f_body[:, 1], f_body[:, 2]))
    pitch_a = np.degrees(-np.arctan2(f_body[:, 0], np.sqrt(f_body[:, 1] ** 2 + f_body[:, 2] ** 2)))
    eul = s.euler_deg
    droll, dpitch = roll_a - eul[:, 0], pitch_a - eul[:, 1]
    # 准静态才把加速度计当重力: |a|≈1g 且角速度小
    qs = (np.abs(amag - 1.0) < 0.12) & (s.gyro_mag < 0.3)
    armed = s.armed
    def med(mask, arr):
        return f"{np.median(arr[mask]):+.2f}" if mask.sum() > 20 else "  n/a"
    out(f"session {s.session_id}:")
    out(f"   全段准静态 {qs.sum():5d} 帧: Δroll {med(qs, droll)}°, Δpitch {med(qs, dpitch)}°")
    out(f"   未解锁准静态 {int((qs&~armed).sum()):5d} 帧: Δroll {med(qs&~armed, droll)}°, Δpitch {med(qs&~armed, dpitch)}°")
    out(f"   解锁中准静态 {int((qs&armed).sum()):5d} 帧: Δroll {med(qs&armed, droll)}°, Δpitch {med(qs&armed, dpitch)}°")
    for a, z in [(a, z) for a, z in zip([0] + list(np.flatnonzero(np.diff(armed.astype(int)) != 0) + 1),
                                        list(np.flatnonzero(np.diff(armed.astype(int)) != 0) + 1) + [len(armed)]) if armed[a]]:
        m = np.zeros(len(armed), bool)
        m[a:z] = qs[a:z]
        out(f"      解锁段 {t[a]:6.1f}~{t[z-1]:6.1f}s: 准静态 {m.sum():4d} 帧  "
            f"Δroll {med(m, droll)}°  Δpitch {med(m, dpitch)}°  "
            f"(该段电机均值 {s.motor_avg[a:z].mean():.0f}, 油门均值 {s['throttle'][a:z].mean():.3f})")

print()
print("=" * 100)
print("T5 控制器链路复算 (session 4 t=171 用户举例附近)")
print("=" * 100)


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_from_two_vectors(a, b):
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    d = float(np.dot(a, b))
    cr = np.cross(a, b)
    if np.linalg.norm(cr) < 1e-6:
        if d > 0:
            return np.array([1.0, 0, 0, 0])
        ref = np.array([1.0, 0, 0]) if abs(a[2]) > 0.9 else np.array([0, 0, 1.0])
        ax = np.cross(a, ref)
        ax = ax / np.linalg.norm(ax)
        return np.array([0.0, *ax])
    q = np.array([1.0 + d, *cr])
    return q / np.linalg.norm(q)


def quat_dcm_z(q):
    w, x, y, z = q
    return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), w * w - x * x - y * y + z * z])


def euler_zyx_deg(q):
    w, x, y, z = q / np.linalg.norm(q)
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    yaw = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw


ATT_P = 3.0   # ATT_ROLL_P / ATT_PITCH_P / ATT_YAW_P
_T5 = []
for _s in load_all():
    _segs = segments(_s.armed)
    if not _segs:
        continue
    _a, _z = max(_segs, key=lambda p: p[1] - p[0])
    _t0, _t1 = _s.t[_a], _s.t[_z - 1]
    _mid = 0.5 * (_t0 + _t1)
    _T5.append((_s.session_id, max(_mid - 3.5, _t0), min(_mid + 3.5, _t1)))
for sid, t0, t1 in _T5:
    s = [x for x in load_all() if x.session_id == sid][0]
    tt = s.t
    out(f"\n--- session {sid} {t0:.0f}~{t1:.0f}s  (目标 roll/pitch = RC 给定期望, 基本为 0)")
    out("   t     roll   pitch | 目标R 目标P | 误差轴(ex,ey) 误差角 | 期望角速度 spx,spy | 实测 wx,wy | "
        "力矩 tx,ty | 复算 spx,spy | 逻辑电机 M0左前/M1右前/M2右后/M3左后")
    for i in range(np.searchsorted(tt, t0), np.searchsorted(tt, t1), 6):
        q = s.q[i]
        r, p, y = euler_zyx_deg(q)
        tr = np.radians(s["target_roll_deg"][i])
        tp = np.radians(s["target_pitch_deg"][i])
        # 期望姿态: 与飞控 release.cpp 的 quatFromEulerZYX(0, targetPitch, targetRoll) 一致(yaw 给 0)
        q_yaw = np.array([1.0, 0, 0, 0])
        cr, sr = np.cos(tr / 2), np.sin(tr / 2)
        cp, sp_ = np.cos(tp / 2), np.sin(tp / 2)
        q_roll = np.array([cr, sr, 0, 0])
        q_pitch = np.array([cp, 0, sp_, 0])
        q_sp = quat_mul(quat_mul(q_yaw, q_pitch), q_roll)
        e_z = quat_dcm_z(q)
        e_zd = quat_dcm_z(q_sp)
        qd_red = quat_from_two_vectors(e_z, e_zd)
        qd_red = quat_mul(qd_red, q)
        qe = quat_mul(quat_conj(q), qd_red)
        if qe[0] < 0:
            qe = -qe
        eq = 2 * qe[1:4]
        ang = np.degrees(2 * np.arccos(np.clip(qe[0], -1, 1)))
        spx, spy = eq[0] * ATT_P, eq[1] * ATT_P
        out(f" {tt[i]:6.2f} {r:6.2f} {p:6.2f} | {s['target_roll_deg'][i]:5.2f} {s['target_pitch_deg'][i]:5.2f} | "
            f"{eq[0]:+.3f},{eq[1]:+.3f} {ang:5.1f}° | {s['rate_sp_x'][i]:+.2f},{s['rate_sp_y'][i]:+.2f} | "
            f"{s['gyro_x'][i]:+.2f},{s['gyro_y'][i]:+.2f} | {s['torque_x'][i]:+.3f},{s['torque_y'][i]:+.3f} | "
            f"{spx:+.2f},{spy:+.2f} | {s.motor[i]}  (a={s.acc_mag[i]:.2f}g)")

with open(out_path("control_check"),
          "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())



