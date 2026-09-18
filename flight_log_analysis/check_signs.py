"""用加速度计(静止时测的是世界"上"方向)判定姿态角的正负约定。

IMU 轴向: x = 机头(用户已确认), z = 上, y = 左手系下的左。
静止时:
  机头下沉(低头)  -> 世界"上"在机体系里偏向机尾 -> acc_x < 0
  右侧下沉(右倾)  -> 世界"上"在机体系里偏向左侧 -> acc_y > 0
所以只要看 pitch_log/roll_log 与 acc_x/acc_y 是同号还是反号, 就知道日志里的
正负约定对应"低头/抬头"、"右倾/左倾"。
"""
from __future__ import annotations

import io
import os

import numpy as np

from load_logs import load_all, out_path

OUT = io.StringIO()


def out(*a):
    print(*a)
    print(*a, file=OUT)


out("准静态帧(未解锁或 |a|≈1g、角速度小): 姿态角 vs 加速度计分量")
for s in load_all():
    e = s.euler_deg
    a = s.acc_mag
    m = (np.abs(a - 1.0) < 0.12) & (s.gyro_mag < 0.2)
    if m.sum() < 200:
        continue
    px, axx = e[m, 1], s["acc_x"][m]
    rl, ayy = e[m, 0], s["acc_y"][m]
    sp = float(np.sum(px * axx) / np.sum(axx ** 2))
    sr = float(np.sum(rl * ayy) / np.sum(ayy ** 2))
    out(f"s{s.session_id} n={m.sum():6d}: "
        f"pitch_log≈{sp:+.2f}·acc_x (corr {np.corrcoef(px, axx)[0,1]:+.2f})  "
        f"roll_log≈{sr:+.2f}·acc_y (corr {np.corrcoef(rl, ayy)[0,1]:+.2f})")
out("")
out("解读: 若 slope(pitch_log vs acc_x) < 0 → pitch_log>0 对应 acc_x<0 = 低头(前倾),")
out("      即日志里 pitch 为负 = 抬头; 反之 pitch_log>0 = 抬头。")

# 第一次飞行(166~176s)里: 姿态、期望、力矩、电机差动 的时间线(0.2s 平均)
s = [x for x in load_all() if x.session_id == 4][0]
t = s.t
e = s.euler_deg
k = 10
out("")
out("session 4 首次飞行 165~178s (0.2s 平均; 电机差动 = (后侧-前侧), 正=压机头)")
out("   t   油门  pitch_log  roll_log | sp_y(期望俯仰速率)  ty(俯仰力矩)  后-前电机差  | wx     wy    | acc_x  acc_y")
for i in range(np.searchsorted(t, 165.0), np.searchsorted(t, 178.0), k):
    sl = slice(i, i + k)
    back = (s.motor[sl, 3] + s.motor[sl, 1]).mean()      # M2右后 + M3左后
    front = (s.motor[sl, 0] + s.motor[sl, 2]).mean()     # M0左前 + M1右前
    out(f" {t[i]:6.1f} {s['throttle'][sl].mean():5.3f} {e[sl,1].mean():+9.2f} {e[sl,0].mean():+8.2f} | "
        f"{s['rate_sp_y'][sl].mean():+8.2f} {s['torque_y'][sl].mean():+11.3f} {back-front:+11.0f} | "
        f"{s['gyro_x'][sl].mean():+6.2f} {s['gyro_y'][sl].mean():+6.2f} | "
        f"{s['acc_x'][sl].mean():+6.2f} {s['acc_y'][sl].mean():+6.2f}")

with open(out_path("sign_check"),
          "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())

