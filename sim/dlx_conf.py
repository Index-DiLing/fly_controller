# -*- coding: utf-8 -*-
"""
DLX450 仿真参数 / 坐标系转换工具

本文件把固件里的参数"抄"到 PC 端, 供 JSBSim 仿真脚本 import。
参数来源(唯一真值仍是固件源码):
    DL_LIB/flight_control/flight_control_params.hpp
    flight_config_struct.hpp  (kConfig.motorMap / loopHz / imuRateHz ...)

坐标系
    固件(DLX):  机体系 x 前, y 左, z 上;       世界系 ENU, z 向上
    JSBSim   :  机体系 x 前, y 右, z 下;       世界系 NED, z 向下
    两者相差一个绕 x 轴的 180 度旋转  S = diag(1,-1,-1), 且 S 是正交阵(det=+1),
    所以同一个物理姿态在两边写出来的旋转矩阵满足 R_dlx = S * R_jsb * S。
"""

import math
import numpy as np

# ==================================================================================================
# 一、机体 / 控制参数 (flight_control_params.hpp)
# ==================================================================================================
ARM_FORWARD_M    = 0.12933   # 每个电机到中心的纵向(前后)偏移 [m]
ARM_LATERAL_M    = 0.18106   # 每个电机到中心的横向(左右)偏移 [m]
MASS_KG          = 1.566     # 总质量 [kg]
INERTIA_XX       = 0.0234    # 绕机体 x 轴(滚转)转动惯量 [kg*m^2]
INERTIA_YY       = 0.0124    # 绕机体 y 轴(俯仰)
INERTIA_ZZ       = 0.0338    # 绕机体 z 轴(偏航)
YAW_TORQUE_ARM_M = 0.03      # 旋翼偏航反力矩等效力臂 [m] (力矩/推力)

HOVER_THROTTLE   = 0.25      # 固件里假定的悬停油门 (归一化)
THROTTLE_MIN     = 0.05
THROTTLE_MAX     = 1.0
TILT_MAX_RAD     = 0.35
GRAVITY          = 9.80665

ATTR_ROLL_P      = 3.0
ATTR_PITCH_P     = 3.0
ATTR_YAW_P       = 3.0
ATT_YAW_WEIGHT   = 0.0
RATE_ROLL_MAX    = 2.0
RATE_PITCH_MAX   = 2.0
RATE_YAW_MAX     = 1.0
RATE_ROLL_P      = 9.0
RATE_ROLL_I      = 2.0
RATE_ROLL_D      = 0.015
RATE_PITCH_P     = 9.0
RATE_PITCH_I     = 2.0
RATE_PITCH_D     = 0.015
RATE_YAW_P       = 5.0
RATE_YAW_I       = 1.0
RATE_YAW_D       = 0.01
ANGACC_MAX       = 40.0
RATE_INT_LIMIT   = 5.0
RATE_D_TAU_S     = 0.01
POS_Z_P          = 2.0
VEL_Z_P          = 3.5
VEL_Z_I          = 0.6
VZ_MAX_UP        = 1.5
VZ_MAX_DOWN      = 0.8

# ==================================================================================================
# 二、电机布局 / 顺序 (flight_control_params.hpp + flight_config_struct.hpp)
# ==================================================================================================
# 逻辑电机编号 -> 机体系(前, 左)平面内的位置 [m]
MOTOR_POS_DLX = {
    0: (+ARM_FORWARD_M, +ARM_LATERAL_M),  # M0 左前
    1: (+ARM_FORWARD_M, -ARM_LATERAL_M),  # M1 右前
    2: (-ARM_FORWARD_M, -ARM_LATERAL_M),  # M2 右后
    3: (-ARM_FORWARD_M, +ARM_LATERAL_M),  # M3 左后
}
MOTOR_NAME = {0: "M0 left-front", 1: "M1 right-front", 2: "M2 right-rear", 3: "M3 left-rear"}
# 从上方俯视的旋向 (True = 顺时针 CW)
MOTOR_CW_FROM_ABOVE = {0: True, 1: False, 2: True, 3: False}

# kConfig.motorMap: 逻辑电机 -> 物理 DShot 通道
MOTOR_MAP = (0, 2, 3, 1)

# ==================================================================================================
# 三、时序 (flight_config_struct.hpp)
# ==================================================================================================
LOOP_HZ  = 500.0            # 控制环频率
IMU_HZ   = 2000.0           # BMI088 ODR
BARO_HZ  = 50.0             # baroPeriodMs = 20
EKF_DECIM = 8               # 每 N 个 IMU 样本做一次协方差传播 + 倾角更新

# ==================================================================================================
# 四、坐标系转换
# ==================================================================================================
# diag(1,-1,-1): 把 JSBSim 的 (前,右,下) 分量换成 DLX 的 (前,左,上) 分量
S_MIRROR = np.diag([1.0, -1.0, -1.0])


def quat_wxyz_from_dcm(R):
    """3x3 方向余弦阵 -> 四元数 (w,x,y,z), 与 dlx::matFromQuaternion 互逆。"""
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=float)
    q /= np.linalg.norm(q)
    if q[0] < 0.0:      # 与 dlx::quatCanonicalize 一致
        q = -q
    return q


def dcm_from_quat_wxyz(q):
    """四元数 (w,x,y,z, 机体->世界) -> 3x3 矩阵, 与 dlx::matFromQuaternion 完全一致。"""
    w, x, y, z = q
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),       2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y),       2.0 * (y * z + w * x),       1.0 - 2.0 * (x * x + y * y)],
    ])


def jsb_dcm_bn(phi, theta, psi):
    """JSBSim 的机体->NED 方向余弦阵, 由 phi/theta/psi (ZYX) 构造。"""
    cph, sph = math.cos(phi), math.sin(phi)
    cth, sth = math.cos(theta), math.sin(theta)
    cps, sps = math.cos(psi), math.sin(psi)
    return np.array([
        [cth * cps, sph * sth * cps - cph * sps, cph * sth * cps + sph * sps],
        [cth * sps, sph * sth * sps + cph * cps, cph * sth * sps - sph * cps],
        [-sth,      sph * cth,                   cph * cth],
    ])


def jsb_attitude_to_dlx_quat(phi, theta, psi):
    """JSBSim 姿态角 -> 固件用的"机体->世界"四元数 (w,x,y,z, DLX 世界系 ENU)。"""
    R_dlx = S_MIRROR @ jsb_dcm_bn(phi, theta, psi) @ S_MIRROR
    return quat_wxyz_from_dcm(R_dlx)


def dlx_quat_to_euler_zyx(q):
    """DLX 四元数 -> (yaw, pitch, roll), 与 dlx::quatToEuler 逐行一致。"""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sp)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    return yaw, pitch, roll


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def dlx_quat_from_euler_zyx(yaw, pitch, roll):
    """(yaw, pitch, roll) -> DLX 四元数, 与 dlx::quatFromEulerZYX 一致 (Rz*Ry*Rx)。"""
    qz = np.array([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)])
    qy = np.array([math.cos(0.5 * pitch), 0.0, math.sin(0.5 * pitch), 0.0])
    qx = np.array([math.cos(0.5 * roll), math.sin(0.5 * roll), 0.0, 0.0])
    return _quat_mul(qz, _quat_mul(qy, qx))


def logical_to_engine(logical_idx):
    """固件逻辑电机编号 -> JSBSim engine 下标 (= 物理 DShot 通道, 见 Propulsion.xml)。"""
    return MOTOR_MAP[logical_idx]
