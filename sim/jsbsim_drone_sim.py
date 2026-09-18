# -*- coding: utf-8 -*-
"""
jsbsim_drone_sim.py -- JSBSim 飞机 + 固件控制/滤波算法 的闭环仿真

结构:
    JSBSim (aircraft/DLX450, 2000 Hz)  <-- 刚体动力学 + 电机/螺旋桨 + IMU/气压计/起落架
        |  sensor/imu/*, sensor/baro/*            ^ fcs/dlx/motorN-nd
        v                                         |
    传感器误差模型 (BMI088 噪声/零偏/振动 + BME280 高度噪声, 可关)
        |                                         |
        v                                         |
    fc_sim.dll  (sim/fc_sim.cpp, 直接 include 固件源码)
        EKF 或 Madgwick -> FlightController(角度环+角速度环) -> mixMotors -> motorMap
        |  motor_phys[4]
        +-----------------------------------------+

用法:
    python sim\\jsbsim_drone_sim.py                 # 跑全部场景
    python sim\\jsbsim_drone_sim.py -s hover gust    # 只跑指定场景
    python sim\\jsbsim_drone_sim.py --clean          # 关掉传感器噪声
输出:
    sim\\out\\<场景>.csv   逐 100Hz 采样
    sim\\out\\<场景>.png   曲线
    sim\\out\\summary.txt  指标汇总
"""

import argparse
import ctypes
import json
import math
import os
import sys
import time
import zlib

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dlx_conf as C

import jsbsim

# JSBSim 的加载告警(告警级别)全部静音, 只留致命错误
_jsb_logger = jsbsim.DefaultLogger()
_jsb_logger.set_min_level(jsbsim.LogLevel.FATAL)
_jsb_logger.set_level(jsbsim.LogLevel.FATAL)
jsbsim.set_logger(_jsb_logger)

HERE = os.path.dirname(os.path.abspath(__file__))
JSBSIM_ROOT = r"E:\JSBSim"
AIRCRAFT = "DLX450"
MASS_XML = os.path.join(JSBSIM_ROOT, "aircraft", AIRCRAFT, "Mass.xml")
MASS_XML_ORIG = MASS_XML + ".orig"   # 首次使用时留一份原件, 保证随时能还原


def set_cg_offset_mm(mm_x):
    """把重心沿机体 x 前后挪 (正 = 往前/机头方向), 单位 mm。

    JSBSim 的 <location name="CG"> 用"结构系"坐标 (x 向后为正), 而电机/起落架的
    位置都是相对同一个原点的, 所以把 CG 的 x 改成负值 = 重心往前 = 电机落在重心后面
    = 恒定抬头/低头力矩。 真实飞机上最常见的原因就是电池装偏了。
    """
    import re as _re
    want = "%.6f" % (-mm_x / 1000.0)
    cur = open(MASS_XML, encoding="utf-8").read()
    if ('<x>' + want + '</x>') in cur:
        return          # 已经是目标值: 不碰文件 (并发/重复调用都安全)
    if not os.path.exists(MASS_XML_ORIG):
        import shutil
        shutil.copyfile(MASS_XML, MASS_XML_ORIG)
    s = open(MASS_XML_ORIG, encoding="utf-8").read()
    s = _re.sub(r'(<location name="CG"[^>]*>\s*<x>)[^<]*(</x>)',
                lambda m: m.group(1) + ("%.6f" % (-mm_x / 1000.0)) + m.group(2), s, count=1)
    with open(MASS_XML, "w", encoding="utf-8") as f:
        f.write(s)


def restore_cg():
    """还原 Mass.xml (仿真脚本退出时自动调用); 设 DLX_KEEP_CG=1 可交给调用方统一还原"""
    if os.environ.get("DLX_KEEP_CG") == "1":
        return
    if os.path.exists(MASS_XML_ORIG):
        import shutil
        shutil.copyfile(MASS_XML_ORIG, MASS_XML)
        os.remove(MASS_XML_ORIG)


import atexit as _atexit
_atexit.register(restore_cg)

SIM_HZ = 2000.0            # = BMI088 ODR = JSBSim 步长
SIM_DT = 1.0 / SIM_HZ
LOG_HZ = 100.0
LOG_EVERY = int(round(SIM_HZ / LOG_HZ))
BARO_PERIOD_S = 0.02       # kConfig.baroPeriodMs
GRAVITY = 9.80665
P_SEA = 101325.0


def pa_to_alt(p):
    return (1.0 - (p / P_SEA) ** 0.190284) * 44330.77


# 悬停油门: plant_check.py 实测值 (固件里写的是 0.25, 差很多 -> 见 README)
PROP_DIAM_IN = 10.0     # 当前模型用的桨直径 [in] (engine/DLX_PROP.xml)
HOVER_FALLBACK = 0.26205
try:
    with open(os.path.join(HERE, "out", "plant_hover.json"), encoding="utf-8") as _f:
        HOVER_MEASURED = float(json.load(_f)["hover_throttle"])
except Exception:
    HOVER_MEASURED = HOVER_FALLBACK


# ==================================================================================================
# 传感器误差模型 (BMI088 / BME280)
#   BMI088: 陀螺噪声密度 0.014 dps/sqrt(Hz), 加速度计 175 ug/sqrt(Hz)
#           ODR 2000Hz + 片上低通, 等效噪声带宽按 650Hz 估 -> sigma 见下
# ==================================================================================================
class SensorModel:
    def __init__(self, rng, enable=True):
        self.rng = rng
        self.enable = enable
        self.gyro_sigma = 0.0060      # [rad/s]  ~0.34 dps
        self.accel_sigma = 0.045      # [m/s^2]  ~4.6 mg
        self.gyro_bias = np.array([0.0050, -0.0035, 0.0020])   # [rad/s] ~0.3 dps
        self.accel_bias = np.array([0.020, -0.015, 0.030])     # [m/s^2]
        self.baro_sigma = 0.5         # [m]
        self.vib_accel = 0.25         # [m/s^2]
        self.vib_gyro = 0.010         # [rad/s]

    def imu(self, gyro, accel, t, rpm_norm):
        """返回 (gyro_meas [rad/s], accel_meas [m/s^2]), 顺序与 FlightControlState 一致。"""
        if not self.enable:
            return np.array(gyro, float), np.array(accel, float)
        w = self.rng.normal(0.0, self.gyro_sigma, 3)
        a = self.rng.normal(0.0, self.accel_sigma, 3)
        f = 60.0 + 120.0 * rpm_norm          # 电机基频附近的机体振动
        vib, vg = self.vib_accel * rpm_norm, self.vib_gyro * rpm_norm
        ph = 2 * math.pi * f * t
        a = a + vib * np.array([math.sin(ph), math.sin(ph + 1.1), 0.6 * math.sin(2 * ph + 0.4)])
        w = w + vg * np.array([math.sin(ph + 2.0), math.sin(ph + 0.7), 0.5 * math.sin(ph + 3.0)])
        return np.array(gyro, float) + w + self.gyro_bias, \
               np.array(accel, float) + a + self.accel_bias

    def baro(self, alt_true):
        return alt_true if not self.enable else alt_true + self.rng.normal(0.0, self.baro_sigma)


# ==================================================================================================
# fc_sim.dll 接口
# ==================================================================================================
class FcSimInput(ctypes.Structure):
    _fields_ = [
        ("gyro", ctypes.c_float * 3),
        ("accel", ctypes.c_float * 3),
        ("baro_abs_m", ctypes.c_float),
        ("baro_valid", ctypes.c_int),
        ("target_yaw_rad", ctypes.c_float),
        ("target_pitch_rad", ctypes.c_float),
        ("target_roll_rad", ctypes.c_float),
        ("target_height_m", ctypes.c_float),
        ("manual_throttle", ctypes.c_float),
        ("height_mode", ctypes.c_int),
        ("armed", ctypes.c_int),
        ("reset", ctypes.c_int),
        ("filter_mode", ctypes.c_int),
        ("hover_throttle", ctypes.c_float),
        ("init_height_m", ctypes.c_float),
        ("tilt_sigma", ctypes.c_float),
        ("tilt_gate_mss", ctypes.c_float),
        ("tilt_inno_limit_rad", ctypes.c_float),
        ("freeze_gyro_bias", ctypes.c_int),
        ("int_limit", ctypes.c_float),
        ("rate_i_scale", ctypes.c_float),
        ("tilt_angle_gate_deg", ctypes.c_float),
        ("accel_hold_enable", ctypes.c_int),
        ("accel_hold_gate_mss", ctypes.c_float),
        ("accel_hold_rate_dps", ctypes.c_float),
        ("accel_hold_time_s", ctypes.c_float),
        ("tilt_air_weak", ctypes.c_int),
    ]


class FcSimOutput(ctypes.Structure):
    _fields_ = [
        ("motor_phys", ctypes.c_float * 4),
        ("motor_logical", ctypes.c_float * 4),
        ("quat", ctypes.c_float * 4),
        ("height_m", ctypes.c_float),
        ("vz_mps", ctypes.c_float),
        ("torque", ctypes.c_float * 3),
        ("rate_setpoint", ctypes.c_float * 3),
        ("att_err_angle", ctypes.c_float),
        ("mix_scale", ctypes.c_float),
        ("gyro_bias", ctypes.c_float * 3),
        ("accel_norm", ctypes.c_float),
        ("tilt_angle_deg", ctypes.c_float),
        ("rate_int", ctypes.c_float * 3),
        ("ang_acc", ctypes.c_float * 3),
        ("motor_saturated", ctypes.c_int),
        ("filter_mode", ctypes.c_int),
    ]


class FcSim:
    def __init__(self, dll_path=None):
        dll_path = dll_path or os.path.join(HERE, "fc_sim.dll")
        if not os.path.exists(dll_path):
            raise SystemExit("缺少 %s -- 先跑 sim\\build_fc_sim.ps1 编译" % dll_path)
        self.lib = ctypes.CDLL(dll_path)
        self.lib.fc_sim_step.argtypes = [ctypes.POINTER(FcSimInput), ctypes.POINTER(FcSimOutput)]
        self.lib.fc_sim_step.restype = None
        if self.lib.fc_sim_sizeof_input() != ctypes.sizeof(FcSimInput) or \
           self.lib.fc_sim_sizeof_output() != ctypes.sizeof(FcSimOutput):
            raise SystemExit("fc_sim.dll 与 Python 结构体布局不一致, 重新编译 DLL")
        self.inp = FcSimInput()
        self.out = FcSimOutput()

    def step(self, **kw):
        for k, v in kw.items():
            cur = getattr(self.inp, k)
            if isinstance(cur, ctypes.Array):
                for i, x in enumerate(v):
                    cur[i] = float(x)
            else:
                setattr(self.inp, k, v)
        self.lib.fc_sim_step(ctypes.byref(self.inp), ctypes.byref(self.out))
        return self.out


# ==================================================================================================
# 场景定义
#   mode   : "angle" = 仅角度环 + 遥控手动油门 (release.cpp 实际在用的模式)
#            "height"= 角度环 + 定高 (updateAngleHeight)
#   start_h: 起始离地高度 [m]
#   motor_bias: 施加到物理电机通道上的固定偏置 (= 电调/桨不平衡)
# ==================================================================================================
def ph(name, dur, roll=0.0, pitch=0.0, yaw=0.0, throttle=None, height=2.0,
       mode="angle", gust=0.0, motor_bias=(0.0, 0.0, 0.0, 0.0), profile=None, sine=None):
    """profile: [(绝对时间 s, 归一化油门), ...] 线性插值 · sine: dict(f=<Hz>, amp=<deg>) 正弦滚转指令"""
    return dict(name=name, dur=dur, roll=roll, pitch=pitch, yaw=yaw, throttle=throttle,
                height=height, mode=mode, gust=gust, motor_bias=motor_bias, profile=profile,
                sine=sine)


SCENARIOS = {
    "hover": dict(
        pilot_kv=0.05,   # 驾驶员补油门的增益 [1/(m/s)]: 0 = 完全固定油门
        hover="measured", filter="ekf", start_h=100.0, arm_t=0.05,
        phases=[ph("hover", 12.0)]),

    "attitude": dict(
        pilot_kv=0.05,   # 驾驶员补油门的增益 [1/(m/s)]: 0 = 完全固定油门
        hover="measured", filter="ekf", start_h=100.0, arm_t=0.05,
        phases=[ph("level", 4.0),
                ph("roll -10deg", 3.0, roll=math.radians(-10.0)),
                ph("level", 3.0),
                ph("pitch +10deg", 3.0, pitch=math.radians(10.0)),
                ph("level", 3.0)]),

    "gust": dict(
        pilot_kv=0.05,   # 驾驶员补油门的增益 [1/(m/s)]: 0 = 完全固定油门
        hover="measured", filter="ekf", start_h=100.0, arm_t=0.05,
        phases=[ph("hover", 4.0), ph("push 2.5N fwd", 0.4, gust=2.5), ph("recover", 6.0)]),

    "imbalance": dict(
        pilot_kv=0.05,   # 驾驶员补油门的增益 [1/(m/s)]: 0 = 完全固定油门
        hover="measured", filter="ekf", start_h=100.0, arm_t=0.05,
        phases=[ph("hover", 5.0),
                ph("M1 +3% bias", 7.0, motor_bias=(0.0, 0.0, 0.0, 0.03))]),

    "height": dict(
        hover="measured", filter="ekf", start_h=2.0, arm_t=0.05,
        phases=[ph("hold 2.0m", 4.0, mode="height", height=2.0),
                ph("climb 3.5m", 5.0, mode="height", height=3.5),
                ph("descend 2.5m", 5.0, mode="height", height=2.5)]),

    "height_fw_hover": dict(
        hover=0.25, filter="ekf", start_h=2.0, arm_t=0.05,
        phases=[ph("hold 2.0m", 6.0, mode="height", height=2.0),
                ph("climb 3.5m", 6.0, mode="height", height=3.5)]),

    # 频率扫描: 8° 正弦滚转指令 (f 由 --sine-f 指定)
    "chirp": dict(
        pilot_kv=0.05, hover="measured", filter="ekf", start_h=100.0, arm_t=0.05, cg_mm=0.0,
        phases=[ph("level", 2.0), ph("sine", 14.0, sine={"f": 1.0, "amp": 8.0})]),

    # 动态工况 1: 快速杆量 (roll ±8° 方波, 0.7s 半周期 x4) —— 看积分会不会造成超调/振荡
    "doublet": dict(
        pilot_kv=0.05, hover="measured", filter="ekf", start_h=100.0, arm_t=0.05, cg_mm=0.0,
        phases=[ph("level", 3.0),
                ph("roll +8", 0.7, roll=math.radians(8.0)),
                ph("roll -8", 0.7, roll=math.radians(-8.0)),
                ph("roll +8", 0.7, roll=math.radians(8.0)),
                ph("roll -8", 0.7, roll=math.radians(-8.0)),
                ph("level", 5.0)]),

    # 动态工况 2: 常值力矩扰动加上又撤掉 (物理通道 3 加 +4% 偏置 4s) —— 看撤掉后的回正会不会冲过头
    "imbal_step": dict(
        pilot_kv=0.05, hover="measured", filter="ekf", start_h=100.0, arm_t=0.05, cg_mm=0.0,
        phases=[ph("hover", 4.0),
                ph("M1 +4% bias", 5.0, motor_bias=(0.0, 0.0, 0.0, 0.04)),
                ph("recover", 7.0)]),

    # 平地起飞: 停机停在地面上 -> 解锁 -> 推油门离地 -> 爬升 -> 起飞后轻微滚转修正 -> 回平
    #   thr_profile 是"杆量曲线": 0.5s 解锁后 1.5s 推到 0.34 (离地), 再收到 0.30 (略高于悬停 0.262)
    #   pilot_kv=0.15 让"人"在爬升后把油门收住, 不至于一直加速上升
    "takeoff": dict(
        pilot_kv=0.15, hover="measured", filter="ekf", start="ground", start_h=0.0, arm_t=0.5,
        # 重心沿机体 x 的偏移 [mm], 正 = 偏前 (电池装偏)。
        # ⚠ 设成 0 会得到"过于干净"的起飞: 真实飞机不可能重心严格落在几何中心,
        #   716g 的电池偏 1cm 就是 4.6mm 的重心偏移。 5mm 是"装配正常但不算完美"的量级。
        cg_mm=5.0,
        thr_profile=[(0.0, 0.0), (2.0, 0.34), (3.0, 0.30)],
        phases=[ph("ground hold", 2.0), ph("liftoff", 3.0), ph("climb", 3.0),
                ph("roll -6deg", 3.0, roll=math.radians(-6.0)),
                ph("level", 5.0)]),

    # 同一段指令, 换成 notebook/2026-09-15_ekf_tilt_fix.md 给 main_att_ekf.cpp 的那组参数
    "attitude_tuned": dict(
        pilot_kv=0.05,   # 驾驶员补油门的增益 [1/(m/s)]: 0 = 完全固定油门
        hover="measured", filter="ekf", start_h=100.0, arm_t=0.05,
        ekf=dict(sigma=0.08, gate=0.20, inno=0.15, freeze=1),
        phases=[ph("level", 4.0),
                ph("roll -10deg", 3.0, roll=math.radians(-10.0)),
                ph("level", 3.0),
                ph("pitch +10deg", 3.0, pitch=math.radians(10.0)),
                ph("level", 3.0)]),

    # 把倾角修正压到很弱 (sigma 大 + 门限紧 + innovation 限幅), 持续倾角指令才跟得住
    "attitude_weak": dict(
        pilot_kv=0.05,   # 驾驶员补油门的增益 [1/(m/s)]: 0 = 完全固定油门
        hover="measured", filter="ekf", start_h=100.0, arm_t=0.05,
        ekf=dict(sigma=0.50, gate=0.03, inno=0.05),
        phases=[ph("level", 4.0),
                ph("roll -10deg", 3.0, roll=math.radians(-10.0)),
                ph("level", 3.0),
                ph("pitch +10deg", 3.0, pitch=math.radians(10.0)),
                ph("level", 3.0)]),

    "madgwick": dict(
        pilot_kv=0.05,   # 驾驶员补油门的增益 [1/(m/s)]: 0 = 完全固定油门
        hover="measured", filter="madgwick", start_h=100.0, arm_t=0.05,
        phases=[ph("hover", 5.0),
                ph("roll -12deg", 3.0, roll=math.radians(-12.0)),
                ph("level", 5.0)]),
}


# ==================================================================================================
# 仿真主循环
# ==================================================================================================
class DroneSim:
    def __init__(self, args):
        self.args = args
        self.fc = FcSim()

    def new_fdm(self, h_agl, start="air"):
        """start="ground" -> 停机停在地面上 (平地起飞);  start="air" -> 直接放在空中"""
        fdm = jsbsim.FGFDMExec(JSBSIM_ROOT)
        fdm.set_debug_level(0)
        if not fdm.load_model(AIRCRAFT):
            raise SystemExit("加载 DLX450 失败")
        fdm.set_dt(SIM_DT)
        for k in range(4):
            fdm["propulsion/engine[%d]/set-running" % k] = 1
        # 先在"坐在地面上"的状态读一次气压, 作为升空前的气压基准 (机头起飞后高度以此为零点)
        fdm.load_ic("initGrnd", True)
        fdm.run_ic()
        p_ground = fdm["sensor/baro/presStatic_Pa"]
        if start == "ground":
            fdm.load_ic("initGrnd", True)
        else:
            fdm.load_ic("initAir", True)
            fdm["ic/h-sl-ft"] = (10.0 + h_agl) / 0.3048
        h_sit = fdm["position/h-agl-ft"] * 0.3048   # 停机时重心离地高度 (起落架高度)
        if start == "ground":
            h0 = h_sit
        else:
            h0 = h_agl
        # 高度零点统一成"停机时重心所在高度" —— 气压计/固件给的就是这个原点
        fdm.load_ic("initGrnd", True)
        fdm.run_ic()
        p_sit = fdm["sensor/baro/presStatic_Pa"]
        if start == "ground":
            fdm.load_ic("initGrnd", True)
        else:
            fdm.load_ic("initAir", True)
            fdm["ic/h-sl-ft"] = (10.0 + h_agl) / 0.3048
        fdm.run_ic()
        p0 = fdm["sensor/baro/presStatic_Pa"]
        return fdm, p_sit, h0, p0, h_sit

    def run_scenario(self, name, spec):
        hover_th = HOVER_MEASURED if spec["hover"] == "measured" else float(spec["hover"])
        filter_mode = 1 if spec["filter"] == "madgwick" else 0
        phases = spec["phases"]
        start_h = spec["start_h"]
        arm_t = spec["arm_t"]
        ekfp = spec.get("ekf", {})
        total_t = sum(p["dur"] for p in phases)

        # 用 crc32 而不是 hash(): hash() 每个进程都不一样, 会让仿真不可复现
        self.rng = np.random.default_rng(self.args.seed + zlib.crc32(name.encode("utf-8")) % 100000)
        # 只有场景显式写了 cg_mm 才动 Mass.xml; 没写的场景沿用当前值
        # (这样"启动方预设一次重心 -> 所有场景共用"是安全的, 并发也不会互相踩)
        if "cg_mm" in spec:
            set_cg_offset_mm(float(spec["cg_mm"]))
        sensor = SensorModel(self.rng, enable=not self.args.clean)
        start = spec.get("start", "air")
        fdm, p_ground, h0, p0, h_sit = self.new_fdm(start_h, start)
        # 气压/高度零点 = 停机时重心所在的高度 (起落架高度): 固件的气压计就是这个原点
        baro_zero = pa_to_alt(p0) - pa_to_alt(p_ground)

        phase_t0 = 0.0
        rows = []
        n_steps = int(round(total_t * SIM_HZ))
        baro_hold = baro_zero
        baro_cd = 0
        motors = [0.0] * 4
        fc_reset = 1
        t_wall = time.time()

        def phase_at(t):
            acc = 0.0
            for p in phases:
                if t < acc + p["dur"]:
                    return p, t - acc, acc
                acc += p["dur"]
            return phases[-1], phases[-1]["dur"], acc - phases[-1]["dur"]

        for i in range(n_steps):
            t = i * SIM_DT
            phase, t_in, phase_t0 = phase_at(t)

            # ---- 读传感器 ----
            acc = np.array([fdm["sensor/imu/accel%s_mps2" % a] for a in "XYZ"])
            gyr = np.array([fdm["sensor/imu/gyro%s_rps" % a] for a in "XYZ"])
            baro_valid = 0
            if baro_cd <= 0:
                baro_hold = pa_to_alt(fdm["sensor/baro/presStatic_Pa"]) - pa_to_alt(p_ground)
                baro_valid = 1
                baro_cd = int(round(BARO_PERIOD_S * SIM_HZ))
            baro_cd -= 1
            w_meas, a_meas = sensor.imu(gyr, acc, t, float(np.clip(np.mean(motors), 0, 1)))
            baro_meas = sensor.baro(baro_hold)

            # ---- 遥控油门 ----
            # 角度环模式下油门是"人给的": 真人会一边看高度一边补油门。
            # 固定油门时因为"下沉 -> 桨进气角变小 -> 推力下降 -> 继续下沉"是发散的,
            # 不管它一定会慢慢沉到地上, 撞地后的数据没意义。这里用一个最简单的驾驶员模型:
            #     thr = 基准油门 - pilot_kv * vz      (vz 向上为正)
            # pilot_kv = 0 就是纯固定油门 (最原始的行为)。
            if phase["mode"] == "height":
                thr_cmd = 0.0
            else:
                prof = phase.get("profile") or spec.get("thr_profile")
                if prof:
                    pts = prof
                    thr_cmd = float(np.interp(t, [q[0] for q in pts], [q[1] for q in pts]))
                else:
                    thr_cmd = phase["throttle"] if phase["throttle"] is not None else hover_th
                if t < arm_t:
                    thr_cmd = 0.0
                else:
                    vz_truth = -fdm["velocities/v-down-fps"] * 0.3048
                    thr_cmd = float(np.clip(thr_cmd - float(spec.get("pilot_kv", 0.05)) * vz_truth,
                                            0.0, 1.0))

            # ---- 固件控制链路 ----
            out = self.fc.step(
                gyro=[float(w_meas[0]), float(w_meas[1]), float(w_meas[2])],
                accel=[float(a_meas[0]), float(a_meas[1]), float(a_meas[2])],
                baro_abs_m=float(baro_meas), baro_valid=baro_valid,
                target_yaw_rad=float(phase["yaw"]),
                target_pitch_rad=float(phase["pitch"]),
                target_roll_rad=float(phase["roll"]) + (
                    math.radians(phase["sine"]["amp"]) * math.sin(2.0 * math.pi * phase["sine"]["f"] * t_in)
                    if phase.get("sine") else 0.0),
                target_height_m=float(phase["height"]),
                manual_throttle=float(thr_cmd),
                height_mode=1 if phase["mode"] == "height" else 0,
                armed=1 if t >= arm_t else 0,
                reset=fc_reset,
                filter_mode=filter_mode,
                hover_throttle=float(hover_th),
                init_height_m=float(baro_zero),
                tilt_sigma=float(ekfp.get("sigma", self.args.tilt_sigma)),
                tilt_gate_mss=float(ekfp.get("gate", self.args.tilt_gate)),
                tilt_inno_limit_rad=float(ekfp.get("inno", self.args.tilt_inno_limit)),
                freeze_gyro_bias=int(ekfp.get("freeze", 0)),
                int_limit=float(spec.get("int_limit", 0.0)),
                rate_i_scale=float(spec.get("rate_i_scale", 0.0)),
                tilt_angle_gate_deg=float(spec.get("tilt_angle_gate_deg", 0.0)),
                tilt_air_weak=int(spec.get("tilt_air_weak", 0)),
                accel_hold_enable=int(ekfp.get("hold", 0)),
                accel_hold_gate_mss=float(ekfp.get("hold_gate", 0.0)),
                accel_hold_rate_dps=float(ekfp.get("hold_rate", 0.0)),
                accel_hold_time_s=float(ekfp.get("hold_time", 0.0)))
            fc_reset = 0

            # ---- 写电机指令 (含人为注入的通道偏置; 解锁前强制 0) ----
            motors = [max(0.0, min(1.0, out.motor_phys[k] + phase["motor_bias"][k])) for k in range(4)]
            if t < arm_t:
                motors = [0.0] * 4
            for k in range(4):
                fdm["fcs/dlx/motor%d-nd" % k] = float(motors[k])
            fdm["external_reactions/dlx_gust/magnitude"] = float(phase["gust"])

            fdm.run()

            # ---- 记录 (100 Hz) ----
            if i % LOG_EVERY == 0:
                phi, theta, psi = (fdm["attitude/%s-rad" % a] for a in ["phi", "theta", "psi"])
                _, pitch_t, roll_t = C.dlx_quat_to_euler_zyx(C.jsb_attitude_to_dlx_quat(phi, theta, psi))
                _, pitch_e, roll_e = C.dlx_quat_to_euler_zyx(list(out.quat))
                rows.append(dict(
                    t=t, phase=phase["name"],
                    roll=roll_t, pitch=pitch_t,
                    roll_est=roll_e, pitch_est=pitch_e,
                    roll_sp=float(phase["roll"]) + (
                        math.radians(phase["sine"]["amp"]) * math.sin(2.0 * math.pi * phase["sine"]["f"] * t_in)
                        if phase.get("sine") else 0.0),
                    pitch_sp=phase["pitch"],
                    p=fdm["velocities/p-rad_sec"], q=fdm["velocities/q-rad_sec"],
                    r=fdm["velocities/r-rad_sec"],
                    alt=fdm["position/h-agl-ft"] * 0.3048 - h_sit,
                    alt_abs=fdm["position/h-agl-ft"] * 0.3048,
                    alt_est=out.height_m, vz=out.vz_mps,
                    height_sp=(phase["height"] if phase["mode"] == "height" else float("nan")),
                    thr=thr_cmd,
                    m0=out.motor_logical[0], m1=out.motor_logical[1],
                    m2=out.motor_logical[2], m3=out.motor_logical[3],
                    mp0=motors[0], mp1=motors[1], mp2=motors[2], mp3=motors[3],
                    mix_scale=out.mix_scale, sat=out.motor_saturated,
                    att_err=out.att_err_angle, gust=phase["gust"],
                    torque_x=out.torque[0], torque_y=out.torque[1], torque_z=out.torque[2],
                    rate_sp_x=out.rate_setpoint[0], rate_sp_y=out.rate_setpoint[1],
                    bg_x=out.gyro_bias[0], bg_y=out.gyro_bias[1], bg_z=out.gyro_bias[2],
                    gx=w_meas[0], gy=w_meas[1], gz=w_meas[2],
                    ax=a_meas[0], ay=a_meas[1], az=a_meas[2],
                    acc_n=out.accel_norm,
                    tilt_angle=out.tilt_angle_deg,
                    ri_x=out.rate_int[0], ri_y=out.rate_int[1], ri_z=out.rate_int[2],
                    aa_x=out.ang_acc[0], aa_y=out.ang_acc[1], aa_z=out.ang_acc[2],
                    tilt_err=float(np.degrees(np.arccos(np.clip(
                        np.dot(C.dcm_from_quat_wxyz(C.jsb_attitude_to_dlx_quat(phi, theta, psi))[:, 2],
                               C.dcm_from_quat_wxyz(list(out.quat))[:, 2]), -1.0, 1.0))))))

        print("   %-16s %5.1f s 仿真 (%d 步, %s起步) 用时 %.1f s"
              % (name, total_t, n_steps, "地面" if start == "ground" else "空中", time.time() - t_wall))
        return rows, hover_th, filter_mode


# ==================================================================================================
# 输出
# ==================================================================================================
def save_csv(rows, path):
    keys = list(rows[0].keys())
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(",".join(keys) + "\n")
        for r in rows:
            f.write(",".join(("%.6g" % r[k]) if isinstance(r[k], (int, float)) else str(r[k])
                             for k in keys) + "\n")


def plot_scenario(name, rows, outdir):
    t = np.array([r["t"] for r in rows])
    col = lambda k: np.array([r[k] for r in rows], dtype=float)
    fig, ax = plt.subplots(4, 1, figsize=(11, 11.5), sharex=True)

    ax[0].plot(t, np.degrees(col("roll_sp")), "k--", lw=1.0, label="setpoint")
    ax[0].plot(t, np.degrees(col("roll")), lw=1.8, label="JSBSim truth")
    ax[0].plot(t, np.degrees(col("roll_est")), lw=1.0, label="filter estimate")
    ax[0].set_ylabel("roll [deg]"); ax[0].legend(fontsize=8, loc="best"); ax[0].grid(alpha=0.3)

    ax[1].plot(t, np.degrees(col("pitch_sp")), "k--", lw=1.0)
    ax[1].plot(t, np.degrees(col("pitch")), lw=1.8)
    ax[1].plot(t, np.degrees(col("pitch_est")), lw=1.0)
    ax[1].set_ylabel("pitch [deg]"); ax[1].grid(alpha=0.3)

    ax[2].plot(t, col("height_sp"), "k--", lw=1.0, label="height setpoint")
    ax[2].plot(t, col("alt"), lw=1.8, label="truth (rel. to start)")
    ax[2].plot(t, col("alt_est"), lw=1.0, label="filter height")
    ax[2].set_ylabel("height [m]"); ax[2].legend(fontsize=8, loc="best"); ax[2].grid(alpha=0.3)

    for k, lab in [("m0", "M0 LF"), ("m1", "M1 RF"), ("m2", "M2 RR"), ("m3", "M3 LR")]:
        ax[3].plot(t, col(k), lw=1.0, label=lab)
    ax[3].plot(t, col("thr"), "k--", lw=1.2, label="collective cmd")
    ax[3].set_ylabel("motor cmd"); ax[3].set_xlabel("time [s]")
    ax[3].legend(fontsize=8, loc="best", ncol=5); ax[3].grid(alpha=0.3)

    fig.suptitle("DLX450 + firmware control/filter chain  --  scenario '%s'" % name)
    fig.tight_layout()
    path = os.path.join(outdir, "%s.png" % name)
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def metrics(name, rows, hover_th, filter_mode):
    col = lambda k: np.array([r[k] for r in rows], dtype=float)
    t = col("t")
    sel = t > 3.0
    if not np.any(sel):
        sel = t > t.max() * 0.5
    m = dict(scenario=name, filter=("Madgwick+VzEst" if filter_mode else "EKF"),
             hover_th=hover_th)
    def nanmax(a):
        a = a[np.isfinite(a)]
        return float(np.max(a)) if a.size else float("nan")

    def nanmean(a):
        a = a[np.isfinite(a)]
        return float(np.mean(a)) if a.size else float("nan")

    m["roll_err_deg_max"] = nanmax(np.abs(np.degrees(col("roll") - col("roll_sp")))[sel])
    m["pitch_err_deg_max"] = nanmax(np.abs(np.degrees(col("pitch") - col("pitch_sp")))[sel])
    est = col("tilt_err")   # 真值/估计 机体 z 轴夹角, 与偏航无关
    m["est_err_deg_max"] = nanmax(est[sel])
    m["est_err_deg_rms"] = float(np.sqrt(nanmean(est[sel] ** 2)))
    m["sat_frac"] = nanmean(col("sat")[sel])
    m["mix_scale_min"] = nanmax(-col("mix_scale")[sel]) * -1.0
    m["alt_err_m_max"] = nanmax(np.abs((col("alt") - col("alt")[0]) - col("height_sp"))[sel])
    m["alt_drift_m"] = float(abs(col("alt")[-1] - col("alt")[0]))
    m["vz_max"] = nanmax(np.abs(col("vz")[sel]))
    m["diverged"] = int(not np.isfinite(col("roll")[-1]))

    # 每个阶段最后 30% 的高度误差 (定高模式的稳态指标)
    alt_err = col("alt") - col("height_sp")
    ph = [r["phase"] for r in rows]
    steadies = []
    i = 0
    while i < len(ph):
        j = i
        while j + 1 < len(ph) and ph[j + 1] == ph[i]:
            j += 1
        k0 = i + int(0.7 * (j - i + 1))
        seg = np.abs(alt_err[k0:j + 1])
        seg = seg[np.isfinite(seg)]
        if seg.size:
            steadies.append(float(np.max(seg)))
        i = j + 1
    m["alt_ss_err_m"] = float(max(steadies)) if steadies else float("nan")
    m["alt_drift_m"] = float(abs(col("alt")[-1] - col("alt")[0]))

    # 每个阶段最后 30% 的姿态误差 (稳态跟踪指标, 不含阶跃超调)
    roll_e = np.degrees(col("roll") - col("roll_sp"))
    pitch_e = np.degrees(col("pitch") - col("pitch_sp"))
    rs, ps = [], []
    i = 0
    while i < len(ph):
        j = i
        while j + 1 < len(ph) and ph[j + 1] == ph[i]:
            j += 1
        k0 = i + int(0.7 * (j - i + 1))
        if j >= k0:
            rs.append(nanmax(np.abs(roll_e[k0:j + 1])))
            ps.append(nanmax(np.abs(pitch_e[k0:j + 1])))
        i = j + 1
    m["roll_ss_err_deg"] = float(max(rs)) if rs else float("nan")
    m["pitch_ss_err_deg"] = float(max(ps)) if ps else float("nan")
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--scenarios", nargs="*", default=None)
    ap.add_argument("--clean", action="store_true", help="关闭传感器噪声")
    ap.add_argument("--seed", type=int, default=20260916)
    ap.add_argument("--tilt-sigma", type=float, default=0.0,
                    help="覆盖 EKF accel_tilt_sigma (固件默认 0.05; 代码注释建议 0.2~0.5)")
    ap.add_argument("--tilt-gate", type=float, default=0.0,
                    help="覆盖 EKF accel_tilt_gate_mss (固件默认 2.0 m/s^2)")
    ap.add_argument("--tilt-inno-limit", type=float, default=0.0,
                    help="覆盖 EKF accel_tilt_inno_limit_rad (固件默认 0 = 不限幅)")
    ap.add_argument("--tag", type=str, default="", help="输出文件名后缀")
    ap.add_argument("--sine-f", type=float, default=None, help="chirp 场景的正弦频率 [Hz]")
    ap.add_argument("--cg-mm", type=float, default=None,
                    help="重心沿机体 x 偏移 [mm], 正=偏前 (覆盖场景里的 cg_mm)")
    ap.add_argument("--int-limit", type=float, default=None,
                    help="覆盖角速度环积分限幅 RATE_INT_LIMIT (固件默认 5 rad/s^2)")
    ap.add_argument("--rate-i-scale", type=float, default=None,
                    help="角速度环积分增益倍数 (RATE_*_I 同乘, 固件默认 1)")
    ap.add_argument("--tilt-angle-gate", type=float, default=None,
                    help="倾角修正角度门控 [deg]: 只在估计倾角小于该值时才做倾角修正 (0/缺省 = 不门控)")
    ap.add_argument("--freeze-bias", type=int, default=None,
                    help="1 = 只在通过倾角判定的那次更新里改陀螺零偏 (main_att_ekf 的做法)")
    ap.add_argument("--accel-hold", type=int, default=None,
                    help="1 = 打开准静止判定 (要连续满足幅值/角速度条件才允许倾角修正)")
    ap.add_argument("--tilt-air-weak", type=int, default=None,
                    help="1 = 地面上用固件默认倾角修正(快速收敛零偏), 解锁后换成 tilt-sigma/gate/inno 那一组(更弱)")
    ap.add_argument("--filter", choices=["ekf", "madgwick"], default=None,
                    help="覆盖场景的滤波链路")
    args = ap.parse_args()

    outdir = os.path.join(HERE, "out")
    os.makedirs(outdir, exist_ok=True)
    names = args.scenarios if args.scenarios else list(SCENARIOS.keys())
    for n in names:
        if n not in SCENARIOS:
            raise SystemExit("未知场景 %s (可选: %s)" % (n, ", ".join(SCENARIOS)))

    print("DLX450 x 固件控制链路 闭环仿真")
    print("  飞机模型: %s\\aircraft\\%s  质量 %.3f kg, 对角轴距 %.1f mm"
          % (JSBSIM_ROOT, AIRCRAFT, C.MASS_KG, 2000.0 * math.hypot(C.ARM_FORWARD_M, C.ARM_LATERAL_M)))
    print("  控制器  : sim/fc_sim.dll (固件源码直接编译)  EKF/慢路径 %.0fHz, 控制环 %.0fHz, IMU %.0fHz"
          % (C.IMU_HZ / C.EKF_DECIM, C.LOOP_HZ, C.IMU_HZ))
    print("  传感器  : %s   (悬停油门 %.5f)" % ("BMI088/BME280 噪声+零偏+振动" if not args.clean
                                                else "理想无噪声", HOVER_MEASURED))
    print()

    sim = DroneSim(args)
    all_metrics = []
    for n in names:
        print("[%s]" % n)
        spec = dict(SCENARIOS[n])
        if args.sine_f is not None:
            for p in spec["phases"]:
                if p.get("sine"):
                    p["sine"] = dict(p["sine"]); p["sine"]["f"] = float(args.sine_f)
        if args.cg_mm is not None:
            spec["cg_mm"] = float(args.cg_mm)
        if args.int_limit is not None:
            spec["int_limit"] = float(args.int_limit)
        if args.rate_i_scale is not None:
            spec["rate_i_scale"] = float(args.rate_i_scale)
        if args.freeze_bias is not None:
            spec.setdefault("ekf", {})["freeze"] = int(args.freeze_bias)
        if args.accel_hold is not None:
            e = spec.setdefault("ekf", {})
            e["hold"] = int(args.accel_hold)
            if args.accel_hold:
                e.setdefault("hold_gate", 0.15)
                e.setdefault("hold_rate", 150.0)
                e.setdefault("hold_time", 0.05)
        if args.tilt_air_weak is not None:
            spec["tilt_air_weak"] = int(args.tilt_air_weak)
        if args.tilt_angle_gate is not None:
            spec["tilt_angle_gate_deg"] = float(args.tilt_angle_gate)
        if args.filter is not None:
            spec["filter"] = args.filter
        rows, hover_th, fm = sim.run_scenario(n, spec)
        label = n + args.tag
        save_csv(rows, os.path.join(outdir, "%s.csv" % label))
        plot_scenario(label, rows, outdir)
        m = metrics(label, rows, hover_th, fm)
        m["run"] = n
        ekfp = SCENARIOS[n].get("ekf", {})
        m["tilt_sigma"] = ekfp.get("sigma", args.tilt_sigma if args.tilt_sigma > 0 else 0.05)
        m["tilt_gate"] = ekfp.get("gate", args.tilt_gate if args.tilt_gate > 0 else 2.0)
        m["tilt_inno"] = ekfp.get("inno", args.tilt_inno_limit)
        all_metrics.append(m)

    lines = ["%-16s %-13s %-7s %-6s %-6s %-6s | %-9s %-10s %-9s %-10s | %-9s %-9s" %
             ("scenario", "filter", "hover", "tSig", "tGate", "tInno",
              "rollErrMax", "rollSSerr", "pitchSSerr", "tiltErrRMS", "altSSerr", "satFrac")]
    for m in all_metrics:
        lines.append("%-16s %-13s %-7.4f %-6.3f %-6.2f %-6.3f | %-9.3f %-10.3f %-9.3f %-10.3f | %-9.3f %-9.3f" %
                     (m["scenario"], m["filter"], m["hover_th"], m["tilt_sigma"], m["tilt_gate"],
                      m["tilt_inno"], m["roll_err_deg_max"], m["roll_ss_err_deg"],
                      m["pitch_ss_err_deg"], m["est_err_deg_rms"], m["alt_ss_err_m"], m["sat_frac"]))
    text = "\n".join(lines)
    print()
    print(text)
    with open(os.path.join(outdir, "summary.txt"), "w", encoding="utf-8") as f:
        f.write(text + "\n")
    print("\n输出目录: %s" % outdir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
