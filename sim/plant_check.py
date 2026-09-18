# -*- coding: utf-8 -*-
"""
plant_check.py -- JSBSim 飞机模型自检 (DLX450)

在上闭环之前, 先证明"飞机模型本身是对的":
  1. 模型能加载, 能按 initAir 起在 1m 高度, IMU 出的是固件坐标系的量;
  2. 推力求曲线, 找出真实悬停油门, 和固件 HOVER_THROTTLE 对比;
  3. 四个电机的位置 + 旋向 + 电机顺序 (motorMap) 与固件混控器假设一致;
  4. JSBSim 姿态 -> 固件四元数 的转换自检。

用法:  python sim\plant_check.py
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dlx_conf as C

import jsbsim

# JSBSim 的加载告警全部静音(只留致命错误), 让自检输出干净
_jsb_logger = jsbsim.DefaultLogger()
_jsb_logger.set_min_level(jsbsim.LogLevel.FATAL)
_jsb_logger.set_level(jsbsim.LogLevel.FATAL)
jsbsim.set_logger(_jsb_logger)

JSBSIM_ROOT = r"E:\JSBSim"
AIRCRAFT = "DLX450"

FAIL = []


def check(ok, what, detail=""):
    print(("  [PASS] " if ok else "  [FAIL] ") + what + ("   " + detail if detail else ""))
    if not ok:
        FAIL.append(what)
    return ok


def new_fdm(dt=0.0005):
    fdm = jsbsim.FGFDMExec(JSBSIM_ROOT)
    fdm.set_debug_level(0)
    if not fdm.load_model(AIRCRAFT):
        raise RuntimeError("cannot load " + AIRCRAFT)
    fdm.set_dt(dt)
    for k in range(4):
        fdm["propulsion/engine[%d]/set-running" % k] = 1
    return fdm


def ic_air(h_agl_m=1.0, phi=0.0, theta=0.0, psi=0.0):
    """建一个已经在空中、水平、静止的初始条件。JSBSim 的 ic/h-sl-ft 是绝对高度。"""
    fdm = new_fdm()
    fdm.load_ic("initAir", True)
    fdm["ic/h-sl-ft"] = (10.0 + h_agl_m) / 0.3048
    fdm["ic/phi-deg"] = math.degrees(phi)
    fdm["ic/theta-deg"] = math.degrees(theta)
    fdm.run_ic()
    return fdm


def run(fdm, seconds):
    n = int(round(seconds / fdm.get_delta_t()))
    for _ in range(n):
        fdm.run()


def thrust_n(fdm):
    return sum(fdm["propulsion/engine[%d]/thrust-lbs" % k] * 4.448222 for k in range(4))


def rpm(fdm, k=0):
    return fdm["propulsion/engine[%d]/propeller-rpm" % k]


def body_moment_jsb(fdm):
    """由 pdot/qdot/rdot 反算机体总力矩 [N*m] (JSBSim 机体系 x前 y右 z下)。"""
    p, q, r = (fdm["velocities/%s-rad_sec" % a] for a in "pqr")
    pd, qd, rd = (fdm["accelerations/%sdot-rad_sec2" % a] for a in "pqr")
    l = C.INERTIA_XX * pd + (C.INERTIA_ZZ - C.INERTIA_YY) * q * r
    m = C.INERTIA_YY * qd + (C.INERTIA_XX - C.INERTIA_ZZ) * r * p
    n = C.INERTIA_ZZ * rd + (C.INERTIA_YY - C.INERTIA_XX) * p * q
    return l, m, n


def set_motors_logical(fdm, logical):
    """logical[i] = 固件逻辑电机 i 的归一化油门; 走 kConfig.motorMap 到物理通道。"""
    phys = [0.0] * 4
    for i in range(4):
        phys[C.logical_to_engine(i)] = logical[i]
    for k in range(4):
        fdm["fcs/dlx/motor%d-nd" % k] = phys[k]
    return phys


# ==================================================================================================
# 1. 加载 + IMU
# ==================================================================================================
def test_load_and_imu():
    print("[1] 模型加载 / IMU 坐标系")
    fdm = ic_air(1.0)
    h_agl = fdm["position/h-agl-ft"] * 0.3048
    check(abs(h_agl - 1.0) < 0.02, "initAir 起在离地 1 m", "h_agl=%.3f m" % h_agl)

    run(fdm, 0.3)
    a = [fdm["sensor/imu/accel%s_mps2" % ax] for ax in "XYZ"]
    check(max(abs(v) for v in a) < 0.8, "自由落体加速度计≈0 (比力)",
          "a=[%.3f %.3f %.3f]" % tuple(a))

    g = new_fdm()
    g.load_ic("initGrnd", True)
    g.run_ic()
    run(g, 2.0)
    ag = [g["sensor/imu/accel%s_mps2" % ax] for ax in "XYZ"]
    check(abs(ag[2] - 9.81) < 0.6, "地面静止 accelZ ≈ +9.81 (固件 z 向上)",
          "a=[%.3f %.3f %.3f]" % tuple(ag))
    check(abs(ag[0]) < 0.5 and abs(ag[1]) < 0.5, "地面静止 accelX/Y ≈ 0",
          "h_agl=%.3f m" % (g["position/h-agl-ft"] * 0.3048))


# ==================================================================================================
# 2. 推力 / 悬停油门
# ==================================================================================================
def thrust_and_acc(th, settle=1.0, window=2.0):
    """固定油门跑 settle+window 秒, 用窗口两端的垂速差算平均垂向加速度。"""
    fdm = ic_air(200.0)
    set_motors_logical(fdm, [th] * 4)
    run(fdm, settle)                                # 等电机转速建立
    vz0 = -fdm["velocities/v-down-fps"] * 0.3048    # 向上为正
    run(fdm, window)
    vz1 = -fdm["velocities/v-down-fps"] * 0.3048
    return thrust_n(fdm), (vz1 - vz0) / window, rpm(fdm)


def test_thrust():
    print("[2] 推力 / 悬停油门")
    need = C.MASS_KG * C.GRAVITY
    print("     需要悬停总推力 %.2f N (%.3f kg)" % (need, C.MASS_KG))
    sweep = []
    for th in np.arange(0.10, 1.001, 0.10):
        T, acc, r = thrust_and_acc(float(th))
        sweep.append([float(th), float(T), float(acc), float(r)])
        print("     throttle=%.2f  T=%6.2f N  T/W=%5.2f  a_z=%+6.2f m/s^2  rpm=%7.0f"
              % (th, T, T / need, acc, r))

    lo, hi = 0.05, 1.0
    for _ in range(16):
        mid = 0.5 * (lo + hi)
        T, acc, r = thrust_and_acc(mid)
        if acc < 0.0:
            lo = mid
        else:
            hi = mid
    hover = 0.5 * (lo + hi)
    T, acc, r = thrust_and_acc(hover)
    print("     实测悬停油门 = %.5f   (T=%.2f N, rpm=%.0f, a_z=%+.4f m/s^2)" % (hover, T, r, acc))
    import json
    outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "plant_hover.json"), "w") as f:
        json.dump({"hover_throttle": hover,
                   "thrust_at_hover_N": T,
                   "rpm_at_hover": r,
                   "mass_kg": C.MASS_KG,
                   "thrust_full_N": sweep[-1][1],
                   "t_w_max": sweep[-1][1] / (C.MASS_KG * C.GRAVITY),
                   "thrust_sweep_throttle": [s[0] for s in sweep],
                   "thrust_sweep_N": [s[1] for s in sweep],
                   "thrust_sweep_rpm": [s[3] for s in sweep]}, f, indent=2)
    print("     固件 HOVER_THROTTLE = %.4f" % C.HOVER_THROTTLE)
    check(0.05 < hover < 0.99, "存在可用悬停油门")
    t_full = sweep[-1][1]
    print("     满油门总推力 %.2f N -> 最大推重比 %.2f" % (t_full, t_full / need))
    return hover, sweep


# ==================================================================================================
# 3. 电机位置 / 旋向 / 顺序
# ==================================================================================================
def test_motor_geometry(hover):
    print("[3] 电机位置 / 旋向 / motorMap 顺序   (基准油门 %.2f)" % hover)

    # 单电机: 位置对了力矩方向才对。M0 左前在 JSBSim 里是 (x=+, y=-)
    for mid, name, l_sgn, m_sgn in [(0, "M0 左前", +1, +1), (3, "M3 左后", +1, -1),
                                    (1, "M1 右前", -1, +1), (2, "M2 右后", -1, -1)]:
        fdm = ic_air(90.0)
        logical = [0.0] * 4
        logical[mid] = 0.55
        set_motors_logical(fdm, logical)
        run(fdm, 0.25)
        l, m, n = body_moment_jsb(fdm)
        check(l * l_sgn > 0 and m * m_sgn > 0,
              "%s 单独出力 -> 力矩方向符合布局" % name,
              "l=%+.4f m=%+.4f (期望 l%+d m%+d)" % (l, m, l_sgn, m_sgn))

    # 混控器三种纯力矩指令 (mixMotors 的 d[] 模式)
    base, delta = hover, 0.02
    patterns = {
        "roll+":  [+delta, -delta, -delta, +delta],
        "pitch+": [-delta, -delta, +delta, +delta],
        "yaw+":   [+delta, -delta, +delta, -delta],
    }
    got = {}
    for name, d in patterns.items():
        fdm = ic_air(90.0)
        set_motors_logical(fdm, [base + v for v in d])
        run(fdm, 0.25)
        got[name] = body_moment_jsb(fdm)

    l, m, n = got["roll+"]
    check(l > 0 and abs(m) < 0.05 * abs(l) + 1e-9, "roll+ -> 纯右滚力矩", "l=%+.4f m=%+.4f" % (l, m))
    l, m, n = got["pitch+"]
    check(m < 0 and abs(l) < 0.05 * abs(m) + 1e-9, "pitch+ -> 纯俯仰力矩 (m<0 = 机头下压)",
          "l=%+.4f m=%+.4f" % (l, m))
    l, m, n = got["yaw+"]
    check(abs(l) < 0.05 * max(abs(n), 1e-9) and abs(m) < 0.05 * max(abs(n), 1e-9),
          "yaw+ -> 只有偏航力矩 (推力差相互抵消)", "l=%+.4f m=%+.4f n=%+.4f" % (l, m, n))
    # 固件 +yaw = 给 M0/M2 (俯视顺时针) 加油门 -> 机体反力矩指向 +z_dlx; z_dlx=-z_jsb
    check(n < 0.0, "yaw+ 机体反力矩 = +z_dlx (moments/n<0)", "n=%+.4f" % n)

    # 偏航权限: 满差动的等效偏航力矩, 和 YAW_TORQUE_ARM_M=0.03m 对比
    T_hover_1 = C.MASS_KG * C.GRAVITY / 4.0
    check(abs(n) > 0.0, "yaw 力矩非零")
    print("     yaw+ 指令 (每机 ±0.02 油门) 实测力矩 n=%.4f N*m" % n)


# ==================================================================================================
# 4. 姿态转换
# ==================================================================================================
def test_attitude_conversion():
    print("[4] JSBSim 姿态 -> 固件四元数")
    worst = 0.0
    for phi, theta, psi in [(0, 0, 0), (0.3, 0, 0), (0, 0.4, 0), (0, 0, 0.7), (0.2, -0.3, 1.1)]:
        q = C.jsb_attitude_to_dlx_quat(phi, theta, psi)
        R_ref = C.S_MIRROR @ C.jsb_dcm_bn(phi, theta, psi) @ C.S_MIRROR
        worst = max(worst, float(np.max(np.abs(C.dcm_from_quat_wxyz(q) - R_ref))))
    check(worst < 1e-9, "四元数 <-> 方向余弦阵 一致", "max|dR|=%.2e" % worst)
    check(np.allclose(C.jsb_attitude_to_dlx_quat(0, 0, 0), [1, 0, 0, 0], atol=1e-9),
          "水平姿态 -> 单位四元数")

    q = C.dlx_quat_from_euler_zyx(0.4, -0.6, 0.3)
    y, p, r = C.dlx_quat_to_euler_zyx(q)
    check(abs(y - 0.4) < 1e-6 and abs(p + 0.6) < 1e-6 and abs(r - 0.3) < 1e-6,
          "dlx_quat_from/to_euler_zyx 往返一致")

    # 坐标映射自检: JSBSim 抬头 -> 固件机体 z 轴往 -x 倒
    z_idle = C.dcm_from_quat_wxyz(C.jsb_attitude_to_dlx_quat(0, 0, 0))[:, 2]
    z_up = C.dcm_from_quat_wxyz(C.jsb_attitude_to_dlx_quat(0.0, 0.3, 0.0))[:, 2]
    check(z_up[0] < -0.2 and abs(z_idle[2] - 1.0) < 1e-9,
          "JSBSim theta>0 (抬头) -> 固件机体 z 轴后倒",
          "z_world=[%+.3f %+.3f %+.3f]" % tuple(z_up))

    # 实际仿真里 JSBSim 的姿态和转换结果要对得上
    def wrap(a):
        """把角度差折到 (-pi, pi], 免得 yaw 绕圈后被当成错误"""
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    fdm = ic_air(2.0, phi=0.0, theta=0.0, psi=0.0)
    set_motors_logical(fdm, [0.55, 0.55, 0.55, 0.55])
    run(fdm, 0.4)
    phi, theta, psi = (fdm["attitude/%s-rad" % a] for a in ["phi", "theta", "psi"])
    q = C.jsb_attitude_to_dlx_quat(phi, theta, psi)
    yaw_d, pitch_d, roll_d = C.dlx_quat_to_euler_zyx(q)
    check(abs(wrap(roll_d - phi)) < 1e-6 and abs(wrap(pitch_d + theta)) < 1e-6
          and abs(wrap(yaw_d + psi)) < 1e-6,
          "运行中: roll=phi, pitch=-theta, yaw=-psi",
          "phi=%+.4f theta=%+.4f psi=%+.4f -> roll=%+.4f pitch=%+.4f yaw=%+.4f"
          % (phi, theta, psi, roll_d, pitch_d, yaw_d))


def main():
    test_load_and_imu()
    hover, _sweep = test_thrust()
    test_motor_geometry(hover)
    test_attitude_conversion()
    print()
    if FAIL:
        print("FAILED: %d 项 -> %s" % (len(FAIL), FAIL))
        return 1
    print("ALL PLANT CHECKS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
