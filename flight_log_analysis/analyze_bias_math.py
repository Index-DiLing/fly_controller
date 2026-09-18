"""核对三件事:
  B1 零偏列为什么"大部分是 0、隔几帧才有一个值"(日志写入策略)
  B2 零偏估计在解锁段里到底有多大、对姿态贡献了多少度(和"总姿态误差"分开算)
  B3 姿态环看见的误差 vs 真实误差 —— 相当于环的有效增益被压低了多少倍
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


out("=" * 104)
out("B1 零偏列为什么隔几帧才出现一次")
out("=" * 104)
out("release.cpp:  if ((logCounter % kConfig.ekfLogDivider) == 0u) { 写入 e.gyroBias }")
out("kConfig.ekfLogDivider 默认 = 10 ⇒ 只有每第 10 条日志(50Hz 下 = 每 200ms)才写零偏,")
out("其余条目的这三个字段是结构体零初始化留下的 0 —— 是没记, 不是估出来就是 0。")
out("所以读这一列必须只取非 0 样本、按 0.2s 间隔理解, 或者直接看每 10 行。")

out("")
out("=" * 104)
out("B2 解锁段内: 零偏估计值 / 真实零偏 / 零偏误差造成的姿态累积")
out("=" * 104)
for s in load_all():
    segs = segments(s.armed)
    if not segs:
        continue
    a0, z0 = max(segs, key=lambda p: p[1] - p[0])
    t = s.t
    dur = t[z0 - 1] - t[a0]
    pre = (t > t[a0] - 2.5) & (t < t[a0])
    true_b = np.array([np.median(s[k][pre]) for k in ("gyro_x", "gyro_y", "gyro_z")])   # rad/s
    bidx = np.flatnonzero(np.abs(s["gyro_bias_x"]) + np.abs(s["gyro_bias_y"]) + np.abs(s["gyro_bias_z"]) > 0)
    ina = bidx[(bidx >= a0) & (bidx < z0)]
    if len(ina) < 5:
        continue
    est = np.stack([s["gyro_bias_x"][ina], s["gyro_bias_y"][ina], s["gyro_bias_z"][ina]], axis=1)
    err = est - true_b[None, :]
    # 姿态累积 = 平均零偏误差 × 时长
    acc = np.degrees(err.mean(axis=0)) * dur
    out(f"\nsession {s.session_id} 解锁 {t[a0]:.1f}~{t[z0-1]:.1f}s ({dur:.1f}s), 零偏样本 {len(ina)} 个(每 0.2s)")
    out(f"   真实零偏 x/y/z = {np.degrees(true_b[0]):+.3f} / {np.degrees(true_b[1]):+.3f} / {np.degrees(true_b[2]):+.3f} °/s")
    out(f"   EKF 估计(段内均值) = {np.degrees(est[:,0].mean()):+.3f} / {np.degrees(est[:,1].mean()):+.3f} / "
        f"{np.degrees(est[:,2].mean()):+.3f} °/s; (段内最大) = {np.degrees(est[:,0].max()):+.3f} / "
        f"{np.degrees(est[:,1].max()):+.3f} / {np.degrees(est[:,2].max()):+.3f} °/s")
    out(f"   零偏误差(均值) = {np.degrees(err[:,0].mean()):+.3f} / {np.degrees(err[:,1].mean()):+.3f} / "
        f"{np.degrees(err[:,2].mean()):+.3f} °/s")
    out(f"   ⇒ 零偏误差在这段飞行里累积的姿态 = roll {acc[0]:+.1f}° / pitch {acc[1]:+.1f}° / yaw {acc[2]:+.1f}°"
        f"   (yaw 不参与控制)")

out("")
out("=" * 104)
out("B3 姿态环看见的误差 vs 真实误差(用去偏陀螺积分当真值)")
out("=" * 104)
out(f"{'会话':>4} {'时长':>6} {'真实 pitch 变化':>15} {'EKF pitch 变化':>15} {'比值':>6} "
    f"{'真实 roll 变化':>14} {'EKF roll 变化':>14} {'比值':>6}")
for s in load_all():
    segs = segments(s.armed)
    if not segs:
        continue
    a0, z0 = max(segs, key=lambda p: p[1] - p[0])
    t = s.t
    pre = (t > t[a0] - 2.5) & (t < t[a0])
    w = np.stack([s["gyro_x"], s["gyro_y"], s["gyro_z"]], axis=1)
    bias_true = np.array([np.median(w[pre, 0]), np.median(w[pre, 1]), np.median(w[pre, 2])])
    qg = integrate_gyro(s.q[a0], w[a0:z0] - bias_true)
    rg, pg = euler_deg(qg.T)
    r_e, p_e = s.euler_deg[a0:z0, 0], s.euler_deg[a0:z0, 1]
    dp_t, dp_e = pg[-1] - pg[0], p_e[-1] - p_e[0]
    dr_t, dr_e = rg[-1] - rg[0], r_e[-1] - r_e[0]
    ratio_p = dp_e / dp_t if abs(dp_t) > 1e-6 else float("nan")
    ratio_r = dr_e / dr_t if abs(dr_t) > 1e-6 else float("nan")
    out(f"{s.session_id:>4} {t[z0-1]-t[a0]:>6.1f} {dp_t:>+15.2f} {dp_e:>+15.2f} {ratio_p:>6.2f} "
        f"{dr_t:>+14.2f} {dr_e:>+14.2f} {ratio_r:>6.2f}")
out("比值 = 姿态环看到的倾角变化 / 真实倾角变化。比值越小, 环对真实倾斜越迟钝;")
out("若是 0.2, 则同样的扰动力矩下, 稳态倾斜会是设计值的 1/0.2 = 5 倍。")

with open(out_path("bias_math"), "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())

