"""区分"控制环的低频振荡"与"桨/机架的高频振动", 并定位最剧烈的时段。

O1 按油门分箱: 低频(0.15s 低通, 控制环能响应的频段) vs 高频(残差) 的 |ω| RMS, 以及姿态峰峰
O2 每个会话解锁段前半/后半的振荡主频与峰峰
O3 角速度最大那一段的 20ms 明细(链路/油门/姿态/电机)
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


def lpf(x, tau_s, fs=50.0):
    a = np.exp(-1.0 / (tau_s * fs))
    y = np.empty_like(x)
    acc = x[0]
    for i, v in enumerate(x):
        acc = a * acc + (1 - a) * v
        y[i] = acc
    return y


out("=" * 100)
out("O1 低频(控制环, 0.15s 低通) vs 高频(残差) 的角速度幅值随油门变化")
out("=" * 100)
out(f"{'会话':>4} {'油门':>6} {'低频 |ω| RMS':>12} {'高频 |ω| RMS':>12} {'姿态峰峰(roll+pitch)':>22} {'帧数':>6}")
for s in load_all():
    ar = s.armed
    if ar.sum() < 200:
        continue
    w = np.stack([s["gyro_x"], s["gyro_y"], s["gyro_z"]], axis=1)
    wmag = np.linalg.norm(w, axis=1)
    wlf = lpf(wmag, 0.15)
    whf = wmag - wlf
    eul = s.euler_deg
    for lo in np.arange(0.0, 0.36, 0.02):
        m = ar & (s["throttle"] >= lo) & (s["throttle"] < lo + 0.02)
        if m.sum() < 150:
            continue
        pp = (np.percentile(eul[m, 0], 99) - np.percentile(eul[m, 0], 1) +
              np.percentile(eul[m, 1], 99) - np.percentile(eul[m, 1], 1))
        out(f"{s.session_id:>4} {lo+0.01:>6.2f} {np.sqrt((wlf[m]**2).mean()):>12.3f} "
            f"{np.sqrt((whf[m]**2).mean()):>12.3f} {pp:>22.1f} {m.sum():>6}")
out("注: 低频 = 控制环能响应的频段(姿态真的在摆); 高频 = 桨/机架振动(控制器只能追着它打)")

out("")
out("=" * 100)
out("O2 每个会话解锁段前半/后半的振荡主频与姿态峰峰")
out("=" * 100)
for s in load_all():
    segs = segments(s.armed)
    if not segs:
        continue
    a0, z0 = max(segs, key=lambda p: p[1] - p[0])
    t = s.t
    tm = 0.5 * (t[a0] + t[z0 - 1])
    for a, b, lbl in ((t[a0], tm, f"s{s.session_id} 前半段"), (tm, t[z0 - 1], f"s{s.session_id} 后半段")):
        m = (t >= a) & (t <= b)
        if m.sum() < 100:
            continue
        x = s.euler_deg[m, 1] - s.euler_deg[m, 1].mean()
        fs = 50.0
        X = np.fft.rfft(x * np.hanning(len(x)))
        f = np.fft.rfftfreq(len(x), 1 / fs)
        k = int(np.argmax(np.abs(X[1:])) + 1)
        out(f"{lbl} ({a:6.1f}~{b:6.1f}s): pitch 主频 {f[k]:.2f} Hz, "
            f"pitch 峰峰 {np.ptp(s.euler_deg[m,1]):5.1f}°, roll 峰峰 {np.ptp(s.euler_deg[m,0]):5.1f}°, "
            f"油门 {np.median(s['throttle'][m]):.3f}")

out("")
out("=" * 100)
out("O3 角速度峰值附近 20ms 明细(链路/油门/姿态/电机)")
out("=" * 100)
for s in load_all():
    ar = s.armed
    if ar.sum() < 200:
        continue
    i0 = int(np.argmax(np.where(ar, s.gyro_mag, 0)))
    t = s.t
    m = (t >= t[i0] - 0.5) & (t <= t[i0] + 0.5)
    out(f"\ns{s.session_id} 峰值 |ω|={s.gyro_mag[i0]:.2f} rad/s @ t={t[i0]:.2f}s")
    out("   时刻   linkAge  |ω|   roll   pitch  油门   |a|   电机(M0左前/M1右前/M2右后/M3左后)")
    for i in np.flatnonzero(m)[::5]:
        out(f"  {t[i]:7.2f} {s['linkAgeMs'][i]:>8} {s.gyro_mag[i]:>5.2f} "
            f"{s.euler_deg[i,0]:>7.2f} {s.euler_deg[i,1]:>7.2f} {s['throttle'][i]:>5.3f} "
            f"{s.acc_mag[i]:>5.2f}   {s.motor[i]}")

with open(out_path("oscillation_check"), "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())
