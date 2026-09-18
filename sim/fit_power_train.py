# -*- coding: utf-8 -*-
"""
fit_power_train.py -- 用 JSBSim 标定动力配置 (桨直径 / KV / 电池节数), 让悬停油门对上实测

为什么需要:
    实飞日志显示 1.566 kg 的飞机在集合油门 ~0.16~0.30 就能悬停
    (sim/calibrate_from_flight_log.py 量出来的), 但当前模型要 0.437。
    "多少油门悬停" 由 KV、桨直径 D、电池电压这三样一起决定, 这里把它们扫一遍,
    同时算出"满油门推重比", 用来判断哪个组合物理上说得通。

原理:
    T_motor = Ct * rho * n^2 * D^4,  n = KV * V(throttle) / 60
    悬停时 4*T = m*g;  推重比上限 = T_max / (m*g)

用法:
    python sim\fit_power_train.py            # 扫一遍并打印表
    python sim\fit_power_train.py --apply    # 把选中的组合写进 engine/DLX_{MOTOR,PROP}.xml

注意: 扫描时会临时改写 E:\JSBSim\engine\DLX_MOTOR.xml / DLX_PROP.xml。
"""

import argparse
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dlx_conf as C

import jsbsim

_jsb_logger = jsbsim.DefaultLogger()
_jsb_logger.set_min_level(jsbsim.LogLevel.FATAL)
_jsb_logger.set_level(jsbsim.LogLevel.FATAL)
jsbsim.set_logger(_jsb_logger)

JSBSIM_ROOT = r"E:\JSBSim"
MOTOR_XML = os.path.join(JSBSIM_ROOT, "engine", "DLX_MOTOR.xml")
PROP_XML = os.path.join(JSBSIM_ROOT, "engine", "DLX_PROP.xml")
AIRCRAFT = "DLX450"
DT = 0.002

# 电池: 4S 满电 16.8V / 带载 ~14.6V ; 6S 满电 25.2V / 带载 ~22.2V
CELL_VOLT = {4: 14.63, 6: 22.20}

# (桨直径[in], KV, 电池节数) 候选
CANDIDATES = [
    (9.4, 960, 4),
    (10.0, 960, 4),
    (10.0, 1100, 4),
    (11.0, 960, 4),
    (12.0, 960, 4),
    (10.0, 960, 6),
    (10.0, 700, 6),
    (9.4, 960, 6),
]


def write_motor(kv, volts, r=0.117, i0=0.45):
    with open(MOTOR_XML, encoding="utf-8") as f:
        s = f.read()
    s = re.sub(r"<velocityconstant>[^<]*</velocityconstant>", "<velocityconstant> %.0f </velocityconstant>" % kv, s)
    s = re.sub(r"<coilresistance>[^<]*</coilresistance>", "<coilresistance>%.3f</coilresistance>" % r, s)
    s = re.sub(r"<noloadcurrent>[^<]*</noloadcurrent>", "<noloadcurrent>%.2f</noloadcurrent>" % i0, s)
    s = re.sub(r"<maxvolts>[^<]*</maxvolts>", "<maxvolts> %.2f </maxvolts>" % volts, s)
    with open(MOTOR_XML, "w", encoding="utf-8") as f:
        f.write(s)


def write_prop(diam_in):
    with open(PROP_XML, encoding="utf-8") as f:
        s = f.read()
    s = re.sub(r'<diameter unit="IN">[^<]*</diameter>', '<diameter unit="IN"> %.2f </diameter>' % diam_in, s)
    with open(PROP_XML, "w", encoding="utf-8") as f:
        f.write(s)


def new_fdm():
    fdm = jsbsim.FGFDMExec(JSBSIM_ROOT)
    fdm.set_debug_level(0)
    if not fdm.load_model(AIRCRAFT):
        raise SystemExit("cannot load " + AIRCRAFT)
    fdm.set_dt(DT)
    for k in range(4):
        fdm["propulsion/engine[%d]/set-running" % k] = 1
    fdm.load_ic("initAir", True)
    fdm["ic/h-sl-ft"] = (10.0 + 200.0) / 0.3048
    fdm.run_ic()
    return fdm


def set_motors(fdm, logical):
    phys = [0.0] * 4
    for i in range(4):
        phys[C.logical_to_engine(i)] = logical[i]
    for k in range(4):
        fdm["fcs/dlx/motor%d-nd" % k] = phys[k]


def measure(th):
    """固定油门跑 1s 建立转速 + 2s 测平均垂向加速度; 返回 (总推力[N], az, rpm)"""
    fdm = new_fdm()
    set_motors(fdm, [th] * 4)
    for _ in range(int(1.0 / DT)):
        fdm.run()
    vz0 = -fdm["velocities/v-down-fps"] * 0.3048
    for _ in range(int(2.0 / DT)):
        fdm.run()
    vz1 = -fdm["velocities/v-down-fps"] * 0.3048
    T = sum(fdm["propulsion/engine[%d]/thrust-lbs" % k] * 4.448222 for k in range(4))
    return T, (vz1 - vz0) / 2.0, fdm["propulsion/engine[0]/propeller-rpm"]


def hover_throttle():
    lo, hi = 0.05, 1.0
    for _ in range(14):
        mid = 0.5 * (lo + hi)
        T, az, _ = measure(mid)
        if az < 0.0:
            lo = mid
        else:
            hi = mid
    h = 0.5 * (lo + hi)
    T, az, rpm = measure(h)
    Tmax, _, rpmax = measure(1.0)
    return h, T, rpm, Tmax, rpmax


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true", help="把选中的组合写进 engine/DLX_*.xml")
    ap.add_argument("--pick", default="10.0/1100/4", help="要套用的组合 直径/KV/节数")
    args = ap.parse_args()

    need = C.MASS_KG * C.GRAVITY
    print("质量 %.3f kg -> 悬停需要总推力 %.2f N" % (C.MASS_KG, need))
    print()
    print("%-14s %-8s %-8s %-10s %-9s %-9s" %
          ("桨[in]/KV/S", "悬停油门", "悬停rpm", "满油门推力", "满油门rpm", "满油门T/W"))

    best = None
    for diam, kv, cells in CANDIDATES:
        write_motor(kv, CELL_VOLT[cells])
        write_prop(diam)
        try:
            h, T, rpm, Tmax, rpmax = hover_throttle()
        except Exception as e:
            print("%-14s  失败: %s" % ("%.1f/%d/%dS" % (diam, kv, cells), e))
            continue
        tw = Tmax / need
        print("%-14s %-8.3f %-8.0f %-10.2f %-9.0f %-9.2f" %
              ("%.1f/%d/%dS" % (diam, kv, cells), h, rpm, Tmax, rpmax, tw))
        if best is None or abs(h - 0.27) < abs(best[0] - 0.27):
            best = (h, diam, kv, cells)

    # 扫描完恢复成"选中的组合"
    pick = args.pick
    pd, pkv, pcells = pick.split("/")
    pd, pkv, pcells = float(pd), float(pkv), int(pcells.rstrip("S"))
    write_motor(pkv, CELL_VOLT[pcells])
    write_prop(pd)
    print()
    print("当前写入 engine/DLX_*.xml 的组合: 桨 %.1f in / %d KV / %dS (maxvolts %.2f V)"
          % (pd, pkv, pcells, CELL_VOLT[pcells]))
    print("最接近悬停 0.27 的候选: 桨 %.1f in / %d KV / %dS (悬停 %.3f)" %
          (best[1], best[2], best[3], best[0]))
    if not args.apply:
        print("(没有 --apply, 参数已经按 --pick 写好了; 加 --apply 只是把选中组合再写一遍)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
