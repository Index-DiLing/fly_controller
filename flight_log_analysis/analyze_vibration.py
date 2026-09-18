"""机架/桨振动 -> 角速度噪声 -> 电机指令抖动 的链路检查。

V1 陀螺与电机指令的帧间高频分量(20ms 差分)统计: 解锁 vs 未解锁
V2 电机力矩指令与实测角速度的关系(控制器是否在"追着噪声打")
V3 低频姿态误差 vs 高频抖动: 说明"看起来没修正"是噪声盖过了修正
"""
from __future__ import annotations

import io
import os

import numpy as np

from analyze import ARM_FORWARD_M, ARM_LATERAL_M, K_NEWTON_PER_UNIT, YAW_TORQUE_ARM_M, mix_scale
from analyze import segments
from load_logs import load_all, out_path


def phys_torque(motor_count: np.ndarray) -> np.ndarray:
    """四路 DShot -> 物理力矩相对量(motor0=M0左前, 1=M3左后, 2=M1右前, 3=M2右后)。"""
    frac = (motor_count - 50.0) / 1900.0
    m0, m3, m1, m2 = frac[:, 0], frac[:, 1], frac[:, 2], frac[:, 3]
    tx = 4 * ARM_LATERAL_M * K_NEWTON_PER_UNIT * (m0 + m3 - m1 - m2) / 4
    ty = 4 * ARM_FORWARD_M * K_NEWTON_PER_UNIT * (m2 + m3 - m0 - m1) / 4
    tz = 4 * YAW_TORQUE_ARM_M * K_NEWTON_PER_UNIT * (m0 + m2 - m1 - m3) / 4
    return np.stack([tx, ty, tz], axis=1)

OUT = io.StringIO()


def out(*a):
    print(*a)
    print(*a, file=OUT)


print("=" * 104)
print("V1 帧间高频分量(20ms 差分) —— 陀螺 vs 电机指令")
print("=" * 104)
out(f"{'会话':>6} {'状态':>8} {'|Δω| 中位':>10} {'|Δω| p95':>9} {'|Δmotor| 中位':>13} "
    f"{'|Δt_phys|roll':>14} {'|Δt_phys|pitch':>15} {'油门中位':>9}")
for s in load_all():
    m = s.motor.astype(float)
    tp = phys_torque(s.motor)
    dw = np.abs(np.diff(np.stack([s["gyro_x"], s["gyro_y"], s["gyro_z"]], axis=1), axis=0)).max(axis=1)
    dm = np.abs(np.diff(m, axis=0)).max(axis=1)
    dt = np.abs(np.diff(tp, axis=0))
    ar = s.armed[1:]
    for lbl, mask in (("解锁", ar), ("未解锁", ~ar)):
        if mask.sum() < 50:
            continue
        out(f"{s.session_id:>6} {lbl:>8} {np.median(dw[mask]):>10.3f} {np.percentile(dw[mask],95):>9.3f} "
            f"{np.median(dm[mask]):>13.1f} {np.median(dt[mask,0]):>14.4f} {np.median(dt[mask,1]):>15.4f} "
            f"{np.median(s['throttle'][1:][mask]):>9.3f}")

print()
print("=" * 104)
print("V2 力矩指令 与 实测角速度 的逐帧关系 (控制器在追噪声?)")
print("=" * 104)
for s in load_all():
    ar = s.armed
    if ar.sum() < 200:
        continue
    tp = phys_torque(s.motor)
    w = np.stack([s["gyro_x"], s["gyro_y"], s["gyro_z"]], axis=1)
    out(f"session {s.session_id}:")
    for ax, nm in ((0, "roll/x"), (1, "pitch/y"), (2, "yaw/z")):
        a, b = tp[ar, ax], w[ar, ax]
        out(f"   {nm}: corr(电机力矩指令, 实测角速度) = {np.corrcoef(a,b)[0,1]:+.3f}  "
            f"(阻尼环应当为负: 角速度越大越往反方向打)")
    # 高频(帧间)相关性: 力矩指令变化 与 角速度变化
    d_tp = np.diff(tp, axis=0)[ar[1:]]
    d_w = np.diff(w, axis=0)[ar[1:]]
    for ax, nm in ((0, "roll/x"), (1, "pitch/y"), (2, "yaw/z")):
        out(f"   {nm} 帧间: corr(Δ力矩, Δ角速度) = {np.corrcoef(d_tp[:,ax], d_w[:,ax])[0,1]:+.3f}")

print()
print("=" * 104)
print("V3 低频姿态(0.5s 滑动平均) 与 高频抖动 的幅度对比")
print("=" * 104)
for s in load_all():
    ar = s.armed
    if ar.sum() < 200:
        continue
    eul = s.euler_deg
    k = 25  # 0.5s @50Hz
    ker = np.ones(k) / k
    for ax, nm in ((0, "roll"), (1, "pitch")):
        lf = np.convolve(eul[:, ax], ker, "same")
        hf = eul[:, ax] - lf
        out(f"session {s.session_id} {nm}: 解锁段低频姿态 5~95 分位 "
            f"{np.percentile(lf[ar],5):+6.2f}~{np.percentile(lf[ar],95):+6.2f}°, "
            f"帧间抖动(去低频) 标准差 {hf[ar].std():5.2f}°, 峰值 {np.abs(hf[ar]).max():5.2f}°; "
            f"陀螺帧间差分 标准差 {np.diff(eul[:,ax])[ar[1:]].std():5.3f}°/帧")

print()
print("=" * 104)
print("V4 振荡幅度是否在失事前逐渐增长(1s 滑窗 RMS)")
print("=" * 104)
_DYN = []
for _s in load_all():
    _segs = segments(_s.armed)
    if _segs:
        _a, _z = max(_segs, key=lambda p: p[1] - p[0])
        _DYN.append((_s.session_id, _s.t[_a], _s.t[_z - 1]))
for sid, t0, t1 in _DYN:
    s = [x for x in load_all() if x.session_id == sid][0]
    t = s.t
    w = np.stack([s["gyro_x"], s["gyro_y"], s["gyro_z"]], axis=1)
    spread = s.motor.max(axis=1) - s.motor.min(axis=1)
    out(f"\nsession {sid} {t0:.0f}~{t1:.0f}s:")
    out("   时间    |ω| 1s RMS  电机差动 RMS  油门   姿态度(roll峰峰/pitch峰峰)  状态")
    a = np.searchsorted(t, t0)
    b = np.searchsorted(t, t1)
    for i in range(a, b, 25):          # 每 0.5s
        w2 = slice(i, min(i + 25, b))
        out(f"   {t[i]:6.1f}  {np.sqrt((w[w2]**2).sum(axis=1).mean()):9.2f}  "
            f"{spread[w2].mean():11.0f}   {np.median(s['throttle'][w2]):5.3f}   "
            f"{np.ptp(s.euler_deg[w2,0]):5.1f}/{-np.ptp(s.euler_deg[w2,1]):5.1f}  "
            f"{'解锁' if s.armed[i] else '停机'}{'(失控保护)' if s.flags_set['Failsafe'][i] else ''}")

with open(out_path("vibration_check"),
          "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())

