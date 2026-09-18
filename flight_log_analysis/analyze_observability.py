"""关键检验: 有推力时, 加速度计还能不能"看见"倾斜?

物理事实(刚体运动 + 牛顿第二定律):
  加速度计测的是比力 f = a - g。稳态推力飞行时(推力沿机体系 z),
  推导可得 f_body = (0, 0, g/cosφ) —— 水平分量为 0, 与倾斜角 φ 无关:
  **只要飞机在"推力支撑自身"的状态, 加速度计就看不见倾斜**。
  只有"准静态"(a≈0, 例如落地静止/被手扶住不动)时, f_body = R^T(0,0,g), 才带倾斜信息。

本脚本把这件事从日志里量出来:
  A1 比力与机体系 z 轴的夹角  vs  姿态估计的倾角(相对世界竖直)
     - 若夹角远小于倾角 => 加速度计测的是推力方向, 对倾斜"失明"
  A2 比力幅值(应≈1/cosφ 倍 g)
  A3 把"比力方向当成重力"会推出什么姿态, 与真实姿态估计差多少
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


def rot_batch(q):
    q = q / np.linalg.norm(q, axis=1)[:, None]
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], axis=1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], axis=1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], axis=1),
    ], axis=1)


out("=" * 100)
out("A1 解锁飞行 vs 地面静止: 比力方向 与 姿态倾角 的关系")
out("   acc_tilt = 比力与机体系 z 轴的夹角(加速度计能看到的倾斜)")
out("   est_tilt = 姿态估计的机体 z 相对世界竖直的夹角(四元数解出)")
out("   若推力飞行时 acc_tilt ≈ 0 而 est_tilt 明显更大 => 加速度计对倾斜失明")
out("=" * 100)
for s in load_all():
    t = s.t
    R = rot_batch(s.q)
    up_body = np.einsum("nji,nj->ni", R, np.tile(np.array([0.0, 0.0, 1.0]), (len(s.q), 1)))
    est_tilt = np.degrees(np.arccos(np.clip(up_body[:, 2], -1, 1)))
    f = np.stack([s["acc_x"], s["acc_y"], s["acc_z"]], axis=1)
    fmag = np.linalg.norm(f, axis=1)
    acc_tilt = np.degrees(np.arctan2(np.hypot(f[:, 0], f[:, 1]), f[:, 2]))
    # 静止(未解锁且角速度小) vs 推力(解锁且油门>0.12)
    stat = (~s.armed) & (s.gyro_mag < 0.1)
    thr = s.armed & (s["throttle"] > 0.12)
    out(f"\nsession {s.session_id}:")
    for lbl, m in (("地面静止", stat), ("推力飞行", thr)):
        if m.sum() < 100:
            out(f"   {lbl}: 样本不足")
            continue
        out(f"   {lbl} ({m.sum():5d} 帧): acc_tilt 中位 {np.median(acc_tilt[m]):5.2f}° "
            f"p95 {np.percentile(acc_tilt[m],95):5.2f}° | est_tilt 中位 {np.median(est_tilt[m]):5.2f}° "
            f"p95 {np.percentile(est_tilt[m],95):5.2f}° | |f| 中位 {np.median(fmag[m]):.3f} g "
            f"(p95 {np.percentile(fmag[m],95):.3f}) | 油门中位 {np.median(s['throttle'][m]):.3f}")

out("")
out("=" * 100)
out("A2 解锁段内: 比力夹角(acc_tilt) 随油门变化 —— 该值应≈0(推力方向), 不含倾斜信息")
out("=" * 100)
out(f"{'会话':>4} {'油门':>6} {'acc_tilt 中位':>14} {'acc_tilt p95':>13} {'|f| 中位':>10} {'帧数':>6}")
for s in load_all():
    t = s.t
    f = np.stack([s["acc_x"], s["acc_y"], s["acc_z"]], axis=1)
    fmag = np.linalg.norm(f, axis=1)
    acc_tilt = np.degrees(np.arctan2(np.hypot(f[:, 0], f[:, 1]), f[:, 2]))
    for lo in np.arange(0.0, 0.36, 0.02):
        m = s.armed & (s["throttle"] >= lo) & (s["throttle"] < lo + 0.02)
        if m.sum() < 150:
            continue
        out(f"{s.session_id:>4} {lo+0.01:>6.2f} {np.median(acc_tilt[m]):>14.2f} "
            f"{np.percentile(acc_tilt[m],95):>13.2f} {np.median(fmag[m]):>10.3f} {m.sum():>6}")

out("")
out("=" * 100)
out("A3 若把比力方向当重力(R^T z ≈ f/|f|), 推出的倾角 vs 估计倾角 —— 推力飞行时应≈0")
out("=" * 100)
for s in load_all():
    m = s.armed & (s["throttle"] > 0.12)
    if m.sum() < 100:
        continue
    f = np.stack([s["acc_x"], s["acc_y"], s["acc_z"]], axis=1)
    fn = f / np.maximum(np.linalg.norm(f, axis=1), 1e-9)[:, None]
    # 估计的"世界上方向在机体里的表示": R^T * z
    R = rot_batch(s.q)
    up_body = np.einsum("nji,nj->ni", R, np.tile(np.array([0.0, 0.0, 1.0]), (len(s.q), 1)))
    # 二者夹角 = 滤波器"新息"大小(它正是被当作倾斜误差去修正的量)
    dot = np.clip((fn * up_body).sum(axis=1), -1, 1)
    innov = np.degrees(np.arccos(dot))
    out(f"session {s.session_id}: 推力段 新息夹角(比力方向 vs 预测重力方向) 中位 {np.median(innov[m]):5.2f}° "
        f"p95 {np.percentile(innov[m],95):5.2f}°  ← 中位越小, 说明滤波器把比力当成重力, 于是把倾斜判成水平")

with open(out_path("observability_check"), "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())

