# -*- coding: utf-8 -*-
"""
cg_trim_test.py -- 起飞时"重心偏前"能容忍多少? 三套滤波参数各扫一遍

背景: 实飞时用当前 EKF 参数起飞, 飞机前倾且无法自行修正。
      这个脚本把"电池装偏导致重心前移"这个最常见的扰动源加进平地起飞场景, 看仿真能不能复现,
      以及三套滤波/参数各自能容忍多少。

做法: 把 Mass.xml 的 CG 沿机体 x 前移 cg_mm 毫米 (JSBSim 结构系 x 向后为正, 所以取负),
      跑平地起飞 (解锁 -> 推油门离地 -> 爬升 -> -6 度滚转修正), 记录:
        末态俯仰 / 最大俯仰 / 姿态估计误差 / 是否发散
      CG 前移 -> 电机落在重心后面 -> 恒定"低头"力矩; 控制器必须靠角速度环积分去抵消,
      而积分项被 RATE_INT_LIMIT 限住了, 所以能抵消的力矩有上限。

用法:  python sim\cg_trim_test.py                 # 全部
       python sim\cg_trim_test.py --cg 0 6 8 10   # 指定几个偏移
输出:  sim\out\cg_trim.txt (+ 每个组合的 csv)
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dlx_conf as C
import jsbsim_drone_sim as S

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

CONFIGS = [
    dict(label="EKF as flown", filter="ekf",
         ekf=dict(sigma=0.05, gate=2.0, inno=0.0, freeze=0)),
    dict(label="EKF tuned   ", filter="ekf",
         ekf=dict(sigma=0.08, gate=0.20, inno=0.15, freeze=1)),
    dict(label="Madgwick    ", filter="madgwick", ekf=dict()),
]

CG_MM = [0.0, 4.0, 6.0, 8.0, 10.0, 14.0, 20.0]


def takeoff_spec(cfg, cg_mm):
    return dict(hover="measured", pilot_kv=0.15, filter=cfg["filter"], ekf=cfg["ekf"],
                start="ground", start_h=0.0, arm_t=0.5, cg_mm=cg_mm,
                thr_profile=[(0.0, 0.0), (2.0, 0.34), (3.0, 0.30)],
                phases=[S.ph("ground hold", 2.0), S.ph("liftoff", 3.0), S.ph("climb", 3.0),
                        S.ph("roll -6deg", 3.0, roll=math.radians(-6.0)),
                        S.ph("level", 5.0)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cg", nargs="*", type=float, default=CG_MM)
    ap.add_argument("--clean", action="store_true")
    ap.add_argument("--seed", type=int, default=20260916)
    args = ap.parse_args()

    class A:
        clean = args.clean
        seed = args.seed
        tilt_sigma = 0.0
        tilt_gate = 0.0
        tilt_inno_limit = 0.0
    sim = S.DroneSim(A)

    print("平地起飞 x 重心偏前: 三套滤波/参数     (悬停油门 %.4f, %s)"
          % (S.HOVER_MEASURED, "无噪声" if args.clean else "带噪声"))
    print()
    print("%-16s %-7s %-10s %-10s %-10s %-10s %-9s %s" %
          ("config", "cg[mm]", "pitchMax", "pitchEnd", "rollEnd", "tiltErrMax", "altMax", "结果"))
    lines = []
    for cfg in CONFIGS:
        for cg in args.cg:
            rows, hover_th, fm = sim.run_scenario(
                "cgtest", takeoff_spec(cfg, float(cg)))
            c = lambda k: np.array([r[k] for r in rows], dtype=float)
            pitch = np.degrees(c("pitch"))
            ok = bool(np.isfinite(pitch).all())
            verdict = "正常" if ok and np.max(np.abs(np.degrees(c("pitch")))) < 20 else \
                      ("翻掉/发散" if not ok else "前倾失控")
            line = "%-16s %-7.1f %-10.1f %-10.1f %-10.1f %-10.1f %-9.2f %s" % (
                cfg["label"], cg, float(np.max(np.abs(pitch))) if ok else float("nan"),
                float(pitch[-1]) if ok else float("nan"),
                float(np.degrees(c("roll"))[-1]) if ok else float("nan"),
                float(np.max(c("tilt_err"))) if ok else float("nan"),
                float(np.max(c("alt"))) if ok else float("nan"), verdict)
            print(line)
            lines.append(line)
            S.save_csv(rows, os.path.join(OUT, "cg_%s_%dmm.csv"
                                          % (cfg["filter"], int(cg))))
        print()

    with open(os.path.join(OUT, "cg_trim.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("输出: out/cg_trim.txt")
    return 0


if __name__ == "__main__":
    sys.exit(main())
