"""快速侦查 debug_*.csv(串口调试流): 判断是哪版固件、有没有真的飞起来。

判据:
  - gyro_bias 三列是否出现过非 0: 只有 EKF 版(release.cpp)才会写零偏估计;
    Madgwick 版(main_final_test)这三列恒为 0。
  - GroundMode 位(0x1000)是否置位: 调试流/台架模式。
  - 是否有 MotorEnabled/AngleLoop 的解锁段, 以及油门/电机到多少。
  - 记录周期 10ms(100Hz, debugLogPeriodMs) 与 vs 20ms(50Hz, 飞行模式)。
"""
from __future__ import annotations

import csv
import io
import os

import numpy as np

from load_logs import LOG_DIR, out_path

OUT = io.StringIO()

COLS = ["seq", "tickMs", "flags", "events", "linkAgeMs", "loopPeriodUs",
        "quat_w", "quat_x", "quat_y", "quat_z",
        "gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z",
        "rate_sp_x", "rate_sp_y", "rate_sp_z", "torque_x", "torque_y", "torque_z",
        "target_pitch_deg", "target_roll_deg", "throttle", "manual_throttle",
        "height_m", "vert_vel_mps", "baro_rel_m", "baro_abs_m",
        "gyro_bias_x", "gyro_bias_y", "gyro_bias_z",
        "motor0", "motor1", "motor2", "motor3"]


def out(*a):
    print(*a)
    print(*a, file=OUT)


FILES = ["debug_20260913_154856.csv", "debug_20260913_202311.csv",
         "debug_20260913_221150.csv", "debug_20260914_124056.csv"]


def read(path, step=1):
    with open(path, encoding="utf-8", newline="") as fh:
        rdr = csv.reader(fh)
        header = next(rdr)
        idx = {k: header.index(k) for k in COLS}
        data = {k: [] for k in COLS}
        for i, row in enumerate(rdr):
            if i % step or len(row) != len(header):
                continue
            for k, j in idx.items():
                data[k].append(float(row[j]))
    return {k: np.array(v) for k, v in data.items()}


for fn in FILES:
    p = os.path.join(LOG_DIR, fn)
    if not os.path.exists(p):
        continue
    d = read(p, step=1)
    n = len(d["tickMs"])
    t = (d["tickMs"] - d["tickMs"][0]) / 1000.0
    flags = d["flags"].astype(int)
    armed = (flags & 0x0010).astype(bool)
    angle = (flags & 0x0008).astype(bool)
    ground = (flags & 0x1000).astype(bool)
    bias_nonzero = (np.abs(d["gyro_bias_x"]) + np.abs(d["gyro_bias_y"]) + np.abs(d["gyro_bias_z"])) > 0
    dt = np.diff(d["tickMs"])
    out(f"=== {fn}: {n} 行, 时长 {t[-1]:.1f}s, tick 步长中位 {np.median(dt):.0f}ms "
        f"(≈{1000/np.median(dt):.0f}Hz), 模式: 地面模式 {100*ground.mean():.0f}%")
    out(f"    零偏列出现过非 0 的条数: {int(bias_nonzero.sum())} "
        f"→ {'EKF 版(release)' if bias_nonzero.any() else '**Madgwick 版(main_final_test)**'}")
    out(f"    解锁帧 {int(armed.sum())}, 角度环帧 {int(angle.sum())}, "
        f"油门 {d['throttle'].min():.3f}~{d['throttle'].max():.3f}, "
        f"电机 {int(min(d['motor0'].min(), d['motor1'].min(), d['motor2'].min(), d['motor3'].min()))}"
        f"~{int(max(d['motor0'].max(), d['motor1'].max(), d['motor2'].max(), d['motor3'].max()))}")
    if armed.any():
        a0 = int(np.flatnonzero(armed)[0])
        out(f"    第一次解锁 t={t[a0]:.1f}s, 该段油门 {d['throttle'][armed].min():.3f}~"
            f"{d['throttle'][armed].max():.3f}, |ω| max {np.linalg.norm(np.stack([d['gyro_x'],d['gyro_y'],d['gyro_z']],1),axis=1)[armed].max():.2f} rad/s")

with open(out_path("debug_profile"), "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())
