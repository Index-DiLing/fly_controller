# -*- coding: utf-8 -*-
"""
compare_filters.py -- 同一条指令、同一组传感器噪声, 对比三套滤波/参数

三套链路:
  1) EKF (as flown)  : release.cpp 现在在用的参数 (sigma .05 / gate 2.0 / 不限幅)
  2) EKF (tuned)     : main_att_ekf.cpp / notebook 2026-09-15_ekf_tilt_fix.md 的优化参数
                       (sigma .08 / gate .20 / inno .15 / freeze_gyro_bias)
  3) Madgwick + VzEst: Madgwick AHRS (beta=0.1) + VerticalVelocityEstimator

两个测试用例:
  takeoff  (默认) : 平地起飞 —— 停机停在地面, 解锁, 推油门离地爬升, 起飞后做一次 -6 度滚转修正再回平
  attitude        : 空中直接给持续倾角指令 (滚转 -10 / 俯仰 +10 各保持 4 s)

公平性: 三个跑的是同一段杆量/指令 + 同一组随机噪声 (强行用同一个场景名 -> 同一个噪声种子),
所以差异只来自滤波器和它的参数。

用法:  python sim\compare_filters.py [--case takeoff|attitude] [--clean]
输出:  sim\out\compare_<case>.png / .txt / 三份 csv
"""

import argparse
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dlx_conf as C
import jsbsim_drone_sim as S

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

CONFIGS = [
    dict(label="EKF (as flown)", filter="ekf",
         ekf=dict(sigma=0.05, gate=2.0, inno=0.0, freeze=0), color="tab:red"),
    dict(label="EKF (tuned)", filter="ekf",
         ekf=dict(sigma=0.08, gate=0.20, inno=0.15, freeze=1), color="tab:orange"),
    dict(label="Madgwick + VzEst", filter="madgwick", ekf=dict(), color="tab:blue"),
]


def build_spec(cfg, case, cg_mm=0.0):
    if case == "takeoff":
        # 与 jsbsim_drone_sim.py 里的 takeoff 场景完全一致
        return dict(hover="measured", pilot_kv=0.15, filter=cfg["filter"], ekf=cfg["ekf"],
                    start="ground", start_h=0.0, arm_t=0.5, cg_mm=cg_mm,
                    thr_profile=[(0.0, 0.0), (2.0, 0.34), (3.0, 0.30)],
                    phases=[S.ph("ground hold", 2.0), S.ph("liftoff", 3.0), S.ph("climb", 3.0),
                            S.ph("roll -6deg", 3.0, roll=math.radians(-6.0)),
                            S.ph("level", 5.0)])
    phases = [S.ph("level", 3.0),
              S.ph("roll -10", 4.0, roll=math.radians(-10.0)),
              S.ph("level", 3.0),
              S.ph("pitch +10", 4.0, pitch=math.radians(10.0)),
              S.ph("level", 4.0)]
    return dict(hover="measured", pilot_kv=0.05, filter=cfg["filter"], ekf=cfg["ekf"],
                start="air", start_h=100.0, arm_t=0.05, phases=phases)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", default="takeoff", choices=["takeoff", "attitude"])
    ap.add_argument("--clean", action="store_true")
    ap.add_argument("--seed", type=int, default=20260916)
    ap.add_argument("--cg-mm", type=float, default=5.0,
                    help="重心沿机体 x 偏移 [mm], 正=偏前 (默认 5mm = 真实装配量级; 0 = 理想情况)")
    args = ap.parse_args()

    os.makedirs(OUT, exist_ok=True)

    class A:
        clean = args.clean
        seed = args.seed
        tilt_sigma = 0.0
        tilt_gate = 0.0
        tilt_inno_limit = 0.0
    sim = S.DroneSim(A)

    print("对比三套滤波/参数   case = %s   (同一段指令 + 同一组噪声)" % args.case)
    print("  悬停油门 %.4f (%.1f in 桨 / 6S), 传感器噪声: %s"
          % (S.HOVER_MEASURED, S.PROP_DIAM_IN, "关" if args.clean else "开"))
    if args.case == "takeoff":
        print("  起飞场景重心沿机体 x 偏移: %+.1f mm (正=偏前; 用 --cg-mm 改)" % args.cg_mm)
    print()

    results = []
    for cfg in CONFIGS:
        print("[%s]" % cfg["label"])
        rows, hover_th, fm = sim.run_scenario(args.case, build_spec(cfg, args.case, args.cg_mm))
        results.append((cfg, rows))

    # ---- 图 ----
    fig, ax = plt.subplots(4, 1, figsize=(11, 11.5), sharex=True)
    ref = results[0][1]
    t = np.array([r["t"] for r in ref])
    col = lambda rows, k: np.array([r[k] for r in rows], dtype=float)

    ax[0].plot(t, np.degrees(col(ref, "roll_sp")), "k--", lw=1.2, label="setpoint")
    for cfg, rows in results:
        ax[0].plot(t, np.degrees(col(rows, "roll")), lw=1.6, color=cfg["color"],
                   label=cfg["label"] + " (truth)")
    ax[0].set_ylabel("roll [deg]"); ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8, ncol=2)

    ax[1].plot(t, np.degrees(col(ref, "pitch_sp")), "k--", lw=1.0)
    for cfg, rows in results:
        ax[1].plot(t, np.degrees(col(rows, "pitch")), lw=1.6, color=cfg["color"])
    ax[1].set_ylabel("pitch [deg]"); ax[1].grid(alpha=0.3)

    for cfg, rows in results:
        ax[2].plot(t, col(rows, "alt"), lw=1.4, color=cfg["color"], label=cfg["label"])
    ax[2].plot(t, col(ref, "thr") * 2.0, "k:", lw=1.0, label="throttle x2")
    ax[2].set_ylabel("height above start [m]"); ax[2].grid(alpha=0.3); ax[2].legend(fontsize=8)

    for cfg, rows in results:
        ax[3].plot(t, col(rows, "tilt_err"), lw=1.4, color=cfg["color"], label=cfg["label"])
    ax[3].set_ylabel("attitude estimate error [deg]\n(true vs estimated body-z)")
    ax[3].set_xlabel("time [s]"); ax[3].grid(alpha=0.3); ax[3].legend(fontsize=8)

    title = ("DLX450 ground takeoff" if args.case == "takeoff"
             else "DLX450 in-air attitude command")
    fig.suptitle(title + ":  EKF(as flown) vs EKF(tuned) vs Madgwick"
                 "  --  same stick/noise,  %.1f in prop, 6S" % S.PROP_DIAM_IN)
    fig.tight_layout()
    png = os.path.join(OUT, "compare_%s.png" % args.case)
    fig.savefig(png, dpi=110)
    plt.close(fig)

    # ---- 指标 ----
    def steadies(c, ph):
        out = []
        i = 0
        while i < len(ph):
            j = i
            while j + 1 < len(ph) and ph[j + 1] == ph[i]:
                j += 1
            k0 = i + int(0.7 * (j - i + 1))
            if j >= k0:
                out.append(c[k0:j + 1])
            i = j + 1
        return out

    lines = ["%-20s %-10s %-10s %-10s %-10s %-9s %-9s %-8s" %
             ("config", "rollErrMax", "pitchErrMax", "tiltErrRMS", "tiltErrMax",
              "liftoff[s]", "altMax[m]", "sat%")]
    for cfg, rows in results:
        c = lambda k: np.array([r[k] for r in rows], dtype=float)
        tt = c("t")
        sel = tt > 0.5
        roll_e = np.degrees(c("roll") - c("roll_sp"))
        pitch_e = np.degrees(c("pitch") - c("pitch_sp"))
        air = np.where(c("alt") > 0.3)[0]
        # 只有地面起步才有"离地时刻"; 空中起步这一列没意义
        liftoff = (float(tt[air[0]]) if air.size else float("nan")) if args.case == "takeoff" \
            else float("nan")
        lines.append("%-20s %-10.3f %-10.3f %-10.3f %-10.3f %-9.2f %-9.2f %-8.1f" % (
            cfg["label"], float(np.max(np.abs(roll_e[sel]))),
            float(np.max(np.abs(pitch_e[sel]))),
            float(np.sqrt(np.mean(c("tilt_err")[sel] ** 2))),
            float(np.max(c("tilt_err")[sel])),
            liftoff, float(np.max(c("alt")) - (0.0 if args.case == "takeoff" else np.min(c("alt")))),
            100.0 * float(np.mean(c("sat")[sel]))))

    print()
    for l in lines:
        print(l)
    with open(os.path.join(OUT, "compare_%s.txt" % args.case), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    for i, (cfg, rows) in enumerate(results):
        S.save_csv(rows, os.path.join(OUT, "compare_%s_%d_%s.csv" % (args.case, i + 1, cfg["filter"])))
    print("\n输出: %s / compare_%s.txt" % (png, args.case))
    return 0


if __name__ == "__main__":
    sys.exit(main())
