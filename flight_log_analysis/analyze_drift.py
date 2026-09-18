"""针对"为什么刚离地就往右前漂、姿态像没在修正"的进一步检查。

D1 混控裁剪发生在什么油门/哪个轴上(哪一路吃掉了权限)
D2 离地瞬间的水平加速度方向(机体系 x 前 / y 左) = 漂移方向
D3 姿态误差随时间是收敛还是发散
D4 悬停点 0.25 vs 实测 0.20 对等效增益的影响
"""
from __future__ import annotations

import io
import os

import numpy as np

from analyze import ARM_FORWARD_M, ARM_LATERAL_M, K_NEWTON_PER_UNIT, YAW_TORQUE_ARM_M, mix_scale
from load_logs import load_all, out_path

OUT = io.StringIO()


def out(*a, **kw):
    print(*a, **kw)
    print(*a, **kw, file=OUT)


def rot_batch(q: np.ndarray) -> np.ndarray:
    """四元数数组 -> 旋转矩阵数组 (n,3,3), 机体系->世界系。"""
    q = q / np.linalg.norm(q, axis=1)[:, None]
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], axis=1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], axis=1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], axis=1),
    ], axis=1)


def tilt_angle(s) -> np.ndarray:
    """机体 z 轴与世界 z 轴夹角 [deg] = 相对水平面的倾角。"""
    R = rot_batch(s.q)
    up_body = np.einsum("nji,nj->ni", R, np.tile(np.array([0.0, 0.0, 1.0]), (len(s.q), 1)))
    return np.degrees(np.arccos(np.clip(up_body[:, 2], -1, 1)))


print("=" * 104)
print("D1 混控裁剪分解: 什么油门、哪一路先顶到边界")
print("=" * 104)
for s in load_all():
    ar = s.armed
    if ar.sum() < 100:
        continue
    th = s["throttle"].copy()
    torque = np.stack([s["torque_x"], s["torque_y"], s["torque_z"]], axis=1)
    scl, d = mix_scale(np.maximum(th, 1e-9), torque)
    clip = ar & (scl < 0.999)
    roll_d = torque[:, 0] / (4 * ARM_LATERAL_M * K_NEWTON_PER_UNIT)
    pitch_d = torque[:, 1] / (4 * ARM_FORWARD_M * K_NEWTON_PER_UNIT)
    yaw_d = torque[:, 2] / (4 * YAW_TORQUE_ARM_M * K_NEWTON_PER_UNIT)
    out(f"\nsession {s.session_id}: 解锁帧 {ar.sum()}, 差动被裁剪帧 {clip.sum()} ({100*clip.sum()/ar.sum():.0f}%)")
    if clip.sum() == 0:
        continue
    out(f"   被裁剪帧的油门: 中位 {np.median(th[clip]):.3f} (5~95 分位 "
        f"{np.percentile(th[clip],5):.3f}~{np.percentile(th[clip],95):.3f}); "
        f"未被裁剪帧油门中位 {np.median(th[ar&~clip]):.3f}")
    for nm, arr in (("roll 差动", roll_d), ("pitch 差动", pitch_d), ("yaw 差动", yaw_d)):
        out(f"   {nm}: 被裁剪帧 |d| 中位 {np.median(np.abs(arr[clip])):.3f} "
            f"= 油门的 {np.median(np.abs(arr[clip])/np.maximum(th[clip],1e-3)):.2f} 倍"
            f"; 符号 {100*(arr[clip]>0).mean():.0f}% 正 / {100*(arr[clip]<0).mean():.0f}% 负")
    need = np.abs(d).max(axis=1)
    out(f"   总差动需求 |d|max: 被裁剪帧中位 {np.median(need[clip]):.3f} vs 可用油门 "
        f"{np.median(th[clip]):.3f} → 需要 {np.median(need[clip]/np.maximum(th[clip],1e-3)):.2f} 倍")

print()
print("=" * 104)
print("D2 接管瞬间的水平加速度方向 (机体系: x 前 / y 左; +x=往前漂, -y=往右漂)")
print("=" * 104)
for s in load_all():
    t = s.t
    R = rot_batch(s.q)
    up_body = np.einsum("nji,nj->ni", R, np.tile(np.array([0.0, 0.0, 1.0]), (len(s.q), 1)))
    f_body = np.stack([s["acc_x"], s["acc_y"], s["acc_z"]], axis=1)
    a_body = f_body - up_body          # 单位 g
    for i in np.flatnonzero(s.events_set["EkfRezero"]):
        w = (t >= t[i]) & (t <= t[i] + 1.5)
        if w.sum() < 30:
            continue
        ax_med, ay_med = float(np.median(a_body[w, 0])), float(np.median(a_body[w, 1]))
        dirs = []
        if ax_med > 0.02:
            dirs.append("前")
        elif ax_med < -0.02:
            dirs.append("后")
        if ay_med < -0.02:
            dirs.append("右")
        elif ay_med > 0.02:
            dirs.append("左")
        eul = s.euler_deg[w]
        out(f"session {s.session_id} 接管后 1.5s (t={t[i]:6.1f}s, 油门中位 {np.median(s['throttle'][w]):.3f}): "
            f"水平加速度 {ax_med:+.3f}g 前向 / {ay_med:+.3f}g 左向 → 往 {'+'.join(dirs) if dirs else '原地'} "
            f"({np.hypot(ax_med, ay_med)*9.81:.2f} m/s²); 同期 roll {np.median(eul[:,0]):+.1f}° "
            f"pitch {np.median(eul[:,1]):+.1f}°")

print()
print("=" * 104)
print("D3 接管后倾角误差随时间变化 (每格 0.5s; 目标 = 0°)")
print("=" * 104)
for s in load_all():
    t = s.t
    tilt = tilt_angle(s)
    for i in np.flatnonzero(s.events_set["EkfRezero"]):
        if t[i] + 4.0 > t[-1]:
            continue
        out(f"session {s.session_id} t={t[i]:6.1f}s: ", end="")
        for k in range(8):
            w = (t >= t[i] + 0.5 * k) & (t < t[i] + 0.5 * (k + 1))
            out(f"{np.median(tilt[w]):5.1f}° " if w.sum() > 5 else "  n/a ", end="")
        out("")

print()
print("=" * 104)
print("D4 悬停点偏差对等效增益的影响")
print("=" * 104)
mass = 1.566
for hover in (0.25, 0.20):
    k = (mass * 9.80665 / hover) / 4.0
    out(f"若真实悬停油门 = {hover:.2f}: 单电机单位推力 k = {k:.2f} N, "
        f"参数表按 0.25 换算 → 实际力矩 = 记录的差动 × k(实际) / k(0.25) "
        f"= {k/((mass*9.80665/0.25)/4.0):.3f} 倍")
out("也就是说: 油门标定偏低 20% ⇒ 同样差动产生的真实力矩大 25% ⇒ 姿态环等效增益 ×1.25,"
    "而 ANGACC_MAX/限幅是按标称值算的, 环路会更硬、更容易振荡。")

with open(out_path("drift_check"),
          "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())

