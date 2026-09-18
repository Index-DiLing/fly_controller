# -*- coding: utf-8 -*-
"""
calibrate_from_flight_log.py -- 用真实飞行日志反推动力配置

仿真里最不确定的一项不是控制参数, 而是"动力配置": 桨多大、电机多少 KV、
几 S 电池, 直接决定"多少油门能悬停"。 这个脚本从实飞日志里把悬停油门量出来,
用来定标 / 判断 JSBSim 模型的动力配置对不对。

日志来源: E:\kmpFly\desktopApp\flight_logs\*.csv  (表头见 flight_log_analysis/load_logs.py)
用法:  python sim\calibrate_from_flight_log.py [--dir <日志目录>] [--min-hover-s 1.0]

判据 (只用日志自己的量):
  - 已解锁 + 角度环工作 (flags: MotorEnabled 0x10, AngleLoop 0x08)
  - 离地: height_m > 0.5 m (EKF 高度, 起飞点为零)
  - 稳定: |vert_vel_mps| <= 0.30 m/s 且 |roll|,|pitch| <= 20 deg
  - 四路电机都没有贴到 DShot 上下限
满足的连续片段 >= min-hover-s 才算一段"悬停", 取其 manual_throttle / 四路电机均值的中位数。
"""

import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

FLAG_MOTOR_ENABLED = 0x0010
FLAG_ANGLE_LOOP = 0x0008
DSHOT_UNIT = 1900.0
DSHOT_OFFSET = 50.0

COL = dict(
    flags=3, quat_w=7, quat_x=8, quat_y=9, quat_z=10,
    manual_throttle=27, height_m=28, vert_vel_mps=29,
    motor0=35, motor1=36, motor2=37, motor3=38,
)
DEFAULT_DIR = r"E:\kmpFly\desktopApp\flight_logs"


def read_session(path):
    with open(path, "r", encoding="utf-8", errors="replace", newline="") as f:
        rd = csv.reader(f)
        header = next(rd)
        idx = {name: header.index(name) for name in COL}
        need = max(idx.values())
        data = {name: [] for name in COL}
        for row in rd:
            if len(row) <= need:
                continue
            for name, i in idx.items():
                data[name].append(float(row[i]))
    for name in data:
        data[name] = np.asarray(data[name], dtype=float)
    return data


def euler_deg(w, x, y, z):
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    sp = np.clip(2 * (w * y - z * x), -1, 1)
    pitch = np.degrees(np.arcsin(sp))
    return roll, pitch


def find_hover_windows(d, min_len_s, dt_s=0.02):
    flags = d["flags"].astype(int)
    armed = ((flags & FLAG_MOTOR_ENABLED) != 0) & ((flags & FLAG_ANGLE_LOOP) != 0)
    roll, pitch = euler_deg(d["quat_w"], d["quat_x"], d["quat_y"], d["quat_z"])
    motors = np.stack([d["motor%d" % i] for i in range(4)], axis=1)
    ok = (armed
          & (d["height_m"] > 0.5)
          & (np.abs(d["vert_vel_mps"]) <= 0.30)
          & (np.abs(roll) <= 20.0) & (np.abs(pitch) <= 20.0)
          & np.all((motors > 55.0) & (motors < 1945.0), axis=1))
    min_len = max(1, int(min_len_s / dt_s))
    windows = []
    i, n = 0, len(ok)
    while i < n:
        if ok[i]:
            j = i
            while j + 1 < n and ok[j + 1]:
                j += 1
            if (j - i + 1) >= min_len:
                windows.append((i, j))
            i = j + 1
        else:
            i += 1
    return windows, motors


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=DEFAULT_DIR)
    ap.add_argument("--min-hover-s", type=float, default=1.0)
    ap.add_argument("--pattern", default="*.csv")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.dir, args.pattern)))
    if not files:
        raise SystemExit("找不到日志: %s" % os.path.join(args.dir, args.pattern))

    print("日志目录: %s   (%d 个文件)" % (args.dir, len(files)))
    print()
    print("%-34s %-8s %-9s %-9s %-24s %-9s %s" %
          ("session", "hover[s]", "stick", "coll(mot)", "每路电机(归一化)", "alt[m]", "|roll|/|pitch|"))

    all_stick, all_coll = [], []
    for path in files:
        try:
            d = read_session(path)
        except Exception as e:
            print("%-34s  读取失败: %s" % (os.path.basename(path), e))
            continue
        if len(d["flags"]) < 50:
            continue
        wins, motors = find_hover_windows(d, args.min_hover_s)
        if not wins:
            continue
        dur = sum(w[1] - w[0] + 1 for w in wins) * 0.02
        idx = np.concatenate([np.arange(a, b + 1) for a, b in wins])
        stick = d["manual_throttle"][idx]
        coll = ((motors[idx] - DSHOT_OFFSET) / DSHOT_UNIT).mean(axis=1)
        roll, pitch = euler_deg(d["quat_w"][idx], d["quat_x"][idx], d["quat_y"][idx], d["quat_z"][idx])
        all_stick.append(stick)
        all_coll.append(coll)
        print("%-34s %-8.1f %-9.3f %-9.3f %-24s %-9s %.1f/%.1f" % (
            os.path.basename(path), dur, float(np.median(stick)), float(np.median(coll)),
            "/".join("%.3f" % ((float(np.median(motors[idx][:, i])) - DSHOT_OFFSET) / DSHOT_UNIT)
                     for i in range(4)),
            "%.2f" % float(np.median(d["height_m"][idx])),
            float(np.median(np.abs(roll))), float(np.median(np.abs(pitch)))))

    if not all_stick:
        raise SystemExit("没有找到符合条件的悬停片段")

    stick = np.concatenate(all_stick)
    coll = np.concatenate(all_coll)
    print()
    print("汇总 (%d 个悬停采样点, %.1f s):" % (stick.size, stick.size * 0.02))
    for name, v in (("manual_throttle (摇杆)", stick), ("coll = mean(motor)", coll)):
        q = np.percentile(v, [5, 25, 50, 75, 95])
        print("  %-24s p5=%.3f p25=%.3f 中位=%.3f p75=%.3f p95=%.3f 均值=%.3f" %
              (name, q[0], q[1], q[2], q[3], q[4], float(v.mean())))

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "flight_hover.json"), "w") as f:
        json.dump({"stick_median": float(np.median(stick)),
                   "stick_mean": float(stick.mean()),
                   "stick_p95": float(np.percentile(stick, 95)),
                   "collective_median": float(np.median(coll)),
                   "samples": int(stick.size),
                   "source": args.dir}, f, indent=2)
    print("\n写出 %s" % os.path.join(out, "flight_hover.json"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
