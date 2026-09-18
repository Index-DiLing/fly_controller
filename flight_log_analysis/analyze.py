"""把回传的会话日志算成指标 + 降采样曲线, 输出 analysis.json。

指标口径全部对应飞控代码:
  - LogEntry / flags / events  -> DL_LIB/W25Q128/dlx_flash_manager_config.h
                                   flight_config_struct.hpp (kLogFlag* / kLogEvent*)
  - 电机:DShot 计数 = 油门(0~1)*dshotUnit(1900) + dshotOffset(50), 逻辑电机->物理通道
         用 kConfig.motorMap = {0,2,3,1}
  - 混控:flight_control_mixer.hpp, 参数取 flight_control_params.hpp 默认值
  - 饱和门槛:release.cpp 的 pidSat = (motor<=55 || motor>=1945)
  - 控制环 500Hz / 日志 50Hz; 失控保护 linkTimeoutMs=5000
"""
from __future__ import annotations

import json
import math
import os

import numpy as np

from load_logs import BATCH, EVENT_BITS, FLAG_BITS, LOG_DIR, load_all

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
SUFFIX = f"_{BATCH}" if BATCH else ""

# ---- 控制参数(flight_control_params.hpp 默认值) ----
MASS_KG = 1.566
HOVER_THROTTLE = 0.25
ARM_FORWARD_M = 0.12933
ARM_LATERAL_M = 0.18106
YAW_TORQUE_ARM_M = 0.03
THROTTLE_MIN = 0.01

# ---- 硬件/协议常数 ----
DSHOT_UNIT = 1900.0
DSHOT_OFFSET = 50.0
MOTOR_MAP = [0, 2, 3, 1]          # 逻辑 i -> 物理通道
LINK_TIMEOUT_MS = 5000
LOOP_TARGET_US = 2000.0
GYRO_BIAS_LIMIT = 0.3

K_NEWTON_PER_UNIT = (MASS_KG * 9.80665 / HOVER_THROTTLE) / 4.0


def mix_scale(throttle: np.ndarray, torque: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """按 flight_control_mixer.hpp 复算差动量与推力优先缩放系数 s。"""
    tx, ty, tz = torque[:, 0], torque[:, 1], torque[:, 2]
    roll_d = tx / (4.0 * ARM_LATERAL_M * K_NEWTON_PER_UNIT)
    pitch_d = ty / (4.0 * ARM_FORWARD_M * K_NEWTON_PER_UNIT)
    yaw_d = tz / (4.0 * YAW_TORQUE_ARM_M * K_NEWTON_PER_UNIT)
    d = np.stack(
        [roll_d - pitch_d + yaw_d, -roll_d - pitch_d - yaw_d,
         -roll_d + pitch_d + yaw_d, roll_d + pitch_d - yaw_d],
        axis=1,
    )
    dmax, dmin = d.max(axis=1), d.min(axis=1)
    s = np.ones(len(throttle))
    up = dmax > 0
    s[up] = np.minimum(s[up], (1.0 - throttle[up]) / dmax[up])
    dn = dmin < 0
    s[dn] = np.minimum(s[dn], throttle[dn] / (-dmin[dn]))
    s = np.clip(s, 0.0, 1.0)
    return s, d


def gravity_deviation(s) -> np.ndarray:
    """姿态四元数预测的重力方向 vs 加速度计实测方向, 夹角 [deg]。

    静态时该角应≈0; 机动/受冲击时加速度计量的不是重力, 这个角会变大,
    此时"用加速度判断姿态"就不可信(参考 flight_log_analysis.html 的说明)。
    """
    q = s.q
    q = q / np.linalg.norm(q, axis=1)[:, None]
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    # R = 机体系->世界系, 取转置作用在 [0,0,1] 上得到机体坐标下的重力方向
    rows = np.stack([
        2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
    ], axis=1)
    a = np.stack([s["acc_x"], s["acc_y"], s["acc_z"]], axis=1)
    an = a / np.maximum(np.linalg.norm(a, axis=1)[:, None], 1e-6)
    dot = np.clip((an * rows).sum(axis=1), -1.0, 1.0)
    return np.degrees(np.arccos(dot))


def ekf_drag(s, armed, throttle_limit: float = 0.12) -> dict:
    """量化"姿态估计被加速度计拧走"的程度。

    欧拉运动学(纯刚体运动学, 与控制器无关):
      dpitch/dt = wy*cos(roll) - wz*sin(roll)
    把实测陀螺积分得到"姿态该有的变化", 与四元数解出的变化相减, 差值只能来自
    EKF 的加速度计倾角修正(以及数值误差)。飞行段该值很大 = 姿态不是陀螺推出来的。
    """
    e = np.radians(s.euler_deg)
    roll, pitch = e[:, 0], e[:, 1]
    dpitch = s["gyro_y"] * np.cos(roll) - s["gyro_z"] * np.sin(roll)
    gyro_int = np.cumsum(dpitch) * 0.02 * 57.29578            # deg
    quat = s.euler_deg[:, 1]
    resid = (quat - gyro_int)
    rate = np.abs(np.diff(quat) - np.diff(gyro_int)) * 50.0    # deg/s
    gate = np.abs(s.acc_mag - 1.0) < (2.0 / 9.80665)           # 门限 2.0 m/s^2
    hi = armed[1:] & (s["throttle"][1:] > throttle_limit)
    out = {
        "gyro_int": [round(float(v), 2) for v in gyro_int[::250]],
        "gyro_int_dt": 5.0,
        "samples": [{"t": round(float(s.t[i]), 1),
                     "dq": round(float(quat[i] - quat[max(i - 25, 0)]), 2),
                     "dg": round(float(gyro_int[i] - gyro_int[max(i - 25, 0)]), 2)}
                    for i in range(250, s.n, 250)],
        "gate_open_armed": round(float(100 * gate[armed].mean()), 1) if armed.any() else None,
        "gate_open_all": round(float(100 * gate.mean()), 1),
        "thr_min_for_armed": throttle_limit,
    }
    if hi.any():
        out["rate_median"] = round(float(np.median(rate[hi])), 1)
        out["rate_p95"] = round(float(np.percentile(rate[hi], 95)), 1)
    return out


def mixer_check(s, scl: np.ndarray, d: np.ndarray, armed: np.ndarray) -> dict:
    """用日志里的 torque+throttle 反算四路 DShot, 与记录值对账。

    对得上 => 控制器/混控/记录链路自洽, 且能反推物理通道顺序(motorMap)。
    """
    th = np.maximum(s["throttle"], 1e-9)
    frac = np.clip(th[:, None] + scl[:, None] * d, 0.0, 1.0)
    pred = np.round(frac * DSHOT_UNIT + DSHOT_OFFSET)
    pred_phys = pred[:, [0, 3, 1, 2]]          # 逻辑 M0,M1,M2,M3 -> 物理通道
    err = np.abs(pred_phys - s.motor).max(axis=1)
    a = armed & (s["throttle"] > 0.05)
    if not a.any():
        return {"n": 0, "match_le1": 0, "match_pct": None, "max_err": None}
    return {
        "n": int(a.sum()),
        "match_le1": int((err[a] <= 1).sum()),
        "match_pct": round(float(100 * (err[a] <= 1).mean()), 2),
        "max_err": int(err[a].max()),
    }


def gyro_vs_ekf(s, armed: np.ndarray) -> dict:
    """解锁段内: 陀螺积分出来的姿态变化(用 EKF 自己的零偏估计) vs EKF 实际姿态变化。

    差值 = 加速度计倾角修正把姿态"拧"回去的量。飞行中加速度计测的是推力+运动加速度,
    不是重力, 所以这个修正会把真实的转动从估计里抵消掉 —— 控制器于是"看不到"飞机在倾斜。
    """
    segs = segments(armed)
    if not segs:
        return {}
    a, z = max(segs, key=lambda p: p[1] - p[0])
    bidx = np.flatnonzero(np.abs(s["gyro_bias_x"]) + np.abs(s["gyro_bias_y"]) + np.abs(s["gyro_bias_z"]) > 0)
    if len(bidx) < 5:
        return {}
    bgx = np.interp(np.arange(s.n), bidx, s["gyro_bias_x"][bidx])
    bgy = np.interp(np.arange(s.n), bidx, s["gyro_bias_y"][bidx])
    bgz = np.interp(np.arange(s.n), bidx, s["gyro_bias_z"][bidx])
    e = np.radians(s.euler_deg)
    roll, pitch = e[:, 0], e[:, 1]
    dpitch = (s["gyro_y"] - bgy) * np.cos(roll) - (s["gyro_z"] - bgz) * np.sin(roll)
    droll = (s["gyro_x"] - bgx) + np.tan(pitch) * ((s["gyro_y"] - bgy) * np.sin(roll) +
                                                  (s["gyro_z"] - bgz) * np.cos(roll))
    gp = float(np.sum(dpitch[a:z]) * 0.02 * 57.29578)
    gr = float(np.sum(droll[a:z]) * 0.02 * 57.29578)
    ep = float(s.euler_deg[z - 1, 1] - s.euler_deg[a, 1])
    er = float(s.euler_deg[z - 1, 0] - s.euler_deg[a, 0])
    # 真实陀螺零偏(解锁前 2.5s 静止段实测)
    pre = (s.t > s.t[a] - 2.5) & (s.t < s.t[a])
    return {
        "t0": round(float(s.t[a]), 1), "t1": round(float(s.t[z - 1]), 1),
        "pitch_gyro": round(gp, 2), "pitch_ekf": round(ep, 2), "pitch_accel_contrib": round(ep - gp, 2),
        "roll_gyro": round(gr, 2), "roll_ekf": round(er, 2), "roll_accel_contrib": round(er - gr, 2),
        "bias_static_dps": [round(float(np.degrees(np.median(s[k][pre]))), 2)
                            for k in ("gyro_x", "gyro_y", "gyro_z")] if pre.sum() > 20 else None,
    }


def segments(mask: np.ndarray) -> list[tuple[int, int]]:
    """把布尔序列切成连续 True 段的 [起, 止) 下标。"""
    if not mask.any():
        return []
    idx = np.flatnonzero(np.diff(mask.astype(np.int8)) != 0) + 1
    b = [0, *idx, len(mask)]
    return [(a, z) for a, z in zip(b[:-1], b[1:]) if mask[a]]


def group_events(mask: np.ndarray, gap: int = 2) -> list[tuple[int, int]]:
    """把散点/短段并成事件窗(中间间隔 <= gap 个采样点算同一个事件)。"""
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return []
    out = []
    start = prev = idx[0]
    for i in idx[1:]:
        if i - prev > gap:
            out.append((start, prev + 1))
            start = i
        prev = i
    out.append((start, prev + 1))
    return out


PRECISION = {
    "m0": 0, "m1": 0, "m2": 0, "m3": 0, "mavg": 0, "mspread": 0, "link": 0,
    "roll": 2, "pitch": 2, "yaw": 2, "gdev": 1, "hgt": 2, "vv": 2, "brel": 2,
}


def envelope(t: np.ndarray, series: dict[str, np.ndarray], buckets: int = 400):
    """min/max 包络降采样: 保峰值, 用于画图。"""
    n = len(t)
    if n <= buckets * 3:
        keep = np.arange(n)
    else:
        edges = np.linspace(0, n, buckets + 1).astype(int)
        keep = set()
        for a, b in zip(edges[:-1], edges[1:]):
            if b <= a:
                continue
            keep.add(a)
            keep.add(b - 1)
            for arr in series.values():
                seg = arr[a:b]
                keep.add(a + int(np.argmax(seg)))
                keep.add(a + int(np.argmin(seg)))
        keep = sorted(keep)
    ts = t[keep]
    out = {"t": [round(float(v), 2) for v in ts]}
    for k, arr in series.items():
        vals = arr[keep]
        p = PRECISION.get(k, 3)
        out[k] = [round(float(v), p) for v in vals]
    return out


def event_rows(s, name: str, mask: np.ndarray, gap: int = 3) -> list[dict]:
    rows = []
    t = s.t
    for a, z in group_events(mask, gap=gap):
        if z - a < 1:
            continue
        rows.append({
            "kind": name,
            "t0": round(float(t[a]), 2),
            "t1": round(float(t[z - 1]), 2),
            "n": int(z - a),
        })
    return rows


def analyze_session(s) -> dict:
    t = s.t
    eul = s.euler_deg
    armed = s.armed
    angle = s.flags_set["AngleLoop"]
    motor = s.motor
    motor_frac = (motor - DSHOT_OFFSET) / DSHOT_UNIT
    th, man = s["throttle"], s["manual_throttle"]
    torque = np.stack([s["torque_x"], s["torque_y"], s["torque_z"]], axis=1)
    scl, _d = mix_scale(np.maximum(th, 1e-9), torque)
    acc = s.acc_mag
    gyro = s.gyro_mag
    gdev = gravity_deviation(s)
    link = s["linkAgeMs"]
    has_baro = bool(s.flags_set["BaroOk"].any())

    # ---- 完整性 ----
    dseq = np.diff(s["seq"])
    dtick = np.diff(s["tickMs"])
    gap_ok = np.abs(dtick - 20 * dseq) <= 2
    lost = int(np.maximum(dseq - 1, 0).sum())
    lp = s["loopPeriodUs"]

    # ---- 解锁段 ----
    segs = []
    for a, z in segments(armed):
        height_vs_baro = round(float(np.abs(s["height_m"][a:z] - s["baro_rel_m"][a:z]).max()), 2) if has_baro else None
        seg = {
            "t0": round(float(t[a]), 2), "t1": round(float(t[z - 1]), 2),
            "dur": round(float(t[z - 1] - t[a]), 2),
            "man_max": round(float(man[a:z].max()), 3),
            "man_mean": round(float(man[a:z].mean()), 3),
            "thr_min": round(float(th[a:z].min()), 3),
            "thr_max": round(float(th[a:z].max()), 3),
            "tgt_max_deg": round(float(max(np.abs(s["target_roll_deg"][a:z]).max(),
                                            np.abs(s["target_pitch_deg"][a:z]).max())), 2),
            "roll_min": round(float(eul[a:z, 0].min()), 2), "roll_max": round(float(eul[a:z, 0].max()), 2),
            "pitch_min": round(float(eul[a:z, 1].min()), 2), "pitch_max": round(float(eul[a:z, 1].max()), 2),
            "roll_absmax": round(float(np.abs(eul[a:z, 0]).max()), 2),
            "pitch_absmax": round(float(np.abs(eul[a:z, 1]).max()), 2),
            "gyro_max": round(float(gyro[a:z].max()), 2),
            "acc_min": round(float(acc[a:z].min()), 2), "acc_max": round(float(acc[a:z].max()), 2),
            "motor_min": int(motor[a:z].min()), "motor_max": int(motor[a:z].max()),
            "motor_mean": round(float(motor[a:z].mean()), 1),
            "spread_max": int(s.motor_spread[a:z].max()),
            "sat_fw": int(s.flags_set["MotorSat"][a:z].sum()),
            "sat_fw_low": int((s.flags_set["MotorSat"][a:z] & (motor[a:z].min(axis=1) <= 55)).sum()),
            "sat_fw_high": int((s.flags_set["MotorSat"][a:z] & (motor[a:z].max(axis=1) >= 1945)).sum()),
            "sat_frac": round(float(100 * s.flags_set["MotorSat"][a:z].mean()), 1),
            "mix_clip_pct": round(float(100 * (scl[a:z] < 0.98).mean()), 1),
            "mix_min": round(float(scl[a:z].min()), 3),
            "gdev_median": round(float(np.median(gdev[a:z])), 1),
            "gdev_p95": round(float(np.percentile(gdev[a:z], 95)), 1),
            "acc_dev_pct": round(float(100 * (np.abs(acc[a:z] - 1.0) > 0.2).mean()), 1),
            "vib": round(float(np.abs(np.diff(acc[a:z])).mean()), 3),
            "baro_min": round(float(s["baro_rel_m"][a:z].min()), 2) if has_baro else None,
            "baro_max": round(float(s["baro_rel_m"][a:z].max()), 2) if has_baro else None,
            "hgt_min": round(float(s["height_m"][a:z].min()), 2),
            "hgt_max": round(float(s["height_m"][a:z].max()), 2),
            "vv_min": round(float(s["vert_vel_mps"][a:z].min()), 2),
            "vv_max": round(float(s["vert_vel_mps"][a:z].max()), 2),
            "link_lost": int((~s.flags_set["LinkOk"][a:z]).sum()),
            "yaw_drift": round(float(eul[z - 1, 2] - eul[a, 2] + 360.0 * round(
                ((eul[a, 2] - eul[z - 1, 2]) / 360.0))), 1),
        }
        # 高度估计与气压的偏离(解锁后两者基准相同)
        seg["h_vs_baro_max"] = height_vs_baro
        segs.append(seg)

    # ---- 事件/异常 ----
    ev = []
    ev += event_rows(s, "大姿态(飞行中 |roll|或|pitch|>30°)",
                     armed & ((np.abs(eul[:, 0]) > 30) | (np.abs(eul[:, 1]) > 30)), gap=10)
    ev += event_rows(s, "大姿态(未解锁, 地面搬运)",
                     (~armed) & ((np.abs(eul[:, 0]) > 60) | (np.abs(eul[:, 1]) > 30)), gap=10)
    ev += event_rows(s, "高角速度(>8 rad/s)", gyro > 8.0, gap=10)
    ev += event_rows(s, "冲击/失重(|加速度-1g|>1g)", np.abs(acc - 1.0) > 1.0, gap=10)
    ev += event_rows(s, "控制饱和(差动被收缩 >10%)", armed & (scl < 0.9), gap=25)
    ev += event_rows(s, "遥控超时(linkAge>5s)", link > LINK_TIMEOUT_MS, gap=10)
    if has_baro:
        ev += event_rows(s, "高度估计与气压偏离 >2m", armed & (np.abs(s["height_m"] - s["baro_rel_m"]) > 2.0), gap=25)
    ev += event_rows(s, "零偏估计到钳位(0.3 rad/s)",
                     (np.abs(s["gyro_bias_x"]) > 0.299) | (np.abs(s["gyro_bias_y"]) > 0.299) |
                     (np.abs(s["gyro_bias_z"]) > 0.299), gap=50)
    ev += event_rows(s, "电机低侧饱和(motor<=55)", (s["flags"] & FLAG_BITS["MotorSat"]).astype(bool) & (motor.max(axis=1) <= 55), gap=3)
    for k, v in s.events_set.items():
        idx = np.flatnonzero(v)
        if k in ("BaroUpdate",):
            continue
        if len(idx) and k in ("Arm", "Disarm", "Failsafe", "CommandChanged", "EkfRezero", "LogFault"):
            # 同一次动作会在连续几帧重复(50Hz 采样到同一次边沿), 取首帧
            first = [int(idx[0])]
            for i in idx[1:]:
                if i - first[-1] > 3:
                    first.append(int(i))
            ev.append({"kind": f"事件 {k}", "t0": round(float(t[first[0]]), 2),
                       "t1": round(float(t[first[-1]]), 2), "n": int(v.sum()),
                       "count": len(first),
                       "times": [round(float(t[i]), 2) for i in first]})
    ev.sort(key=lambda r: r["t0"])

    # ---- 零偏时间线 ----
    bidx = np.flatnonzero((np.abs(s["gyro_bias_x"]) + np.abs(s["gyro_bias_y"]) + np.abs(s["gyro_bias_z"])) > 0)
    bias = {
        "t": [round(float(t[i]), 2) for i in bidx],
        "x": [round(float(s["gyro_bias_x"][i]), 5) for i in bidx],
        "y": [round(float(s["gyro_bias_y"][i]), 5) for i in bidx],
        "z": [round(float(s["gyro_bias_z"][i]), 5) for i in bidx],
    }

    # ---- 静态段(未解锁 + 陀螺很小): 量测真实零偏与姿态漂移 ----
    stat = (~armed) & (gyro < 0.08) & (acc > 0.9) & (acc < 1.1)
    static_rows = []
    for a, z in segments(stat):
        if t[z - 1] - t[a] < 2.0:
            continue
        y = np.unwrap(np.radians(eul[a:z, 2]))
        tt = t[a:z] - t[a]
        yaw_rate = float(np.degrees(np.polyfit(tt, y, 1)[0]))
        static_rows.append({
            "t0": round(float(t[a]), 1), "t1": round(float(t[z - 1]), 1),
            "yaw_rate": round(yaw_rate, 3),
            "gyro_z_dps": round(float(np.degrees(s["gyro_z"][a:z].mean())), 3),
            "bias_z_inferred": round(float(s["gyro_bias_z"][a:z].mean() + 0 * 0), 5),
            "bias_z_logged": round(float(np.interp(np.arange(a, z), bidx, s["gyro_bias_z"][bidx]).mean()), 5) if len(bidx) else 0.0,
        })

    # ---- flags 时间线(压缩成 bitmask 序列) ----
    flags_keep = ["LinkOk", "ImuOk", "BaroOk", "AngleLoop", "MotorEnabled", "MotorStarting",
                  "HardStop", "FlashOk", "LogFull", "LogError", "MotorSat", "Failsafe", "GroundMode"]
    flags_ts = {}
    for k in flags_keep:
        flags_ts[k] = [int(v) for v in s.flags_set[k][::5]]      # 10Hz 足够画状态条

    # ---- 振荡幅值随油门: 低频(控制环能响应的频段) vs 姿态峰峰 ----
    dgyro = np.abs(np.diff(np.stack([s["gyro_x"], s["gyro_y"], s["gyro_z"]], axis=1), axis=0)).max(axis=1)
    dmot = np.abs(np.diff(s.motor.astype(float), axis=0)).max(axis=1)
    armed_next = armed[1:]
    vib = {
        "dgyro_armed": round(float(np.median(dgyro[armed_next])), 3) if armed_next.any() else None,
        "dgyro_idle": round(float(np.median(dgyro[~armed_next])), 3),
        "dmotor_armed": round(float(np.median(dmot[armed_next])), 1) if armed_next.any() else None,
    }
    alpha = math.exp(-1.0 / (0.15 * 50.0))
    wlf = np.empty_like(gyro)
    _acc = float(gyro[0])
    for _i, _v in enumerate(gyro):
        _acc = alpha * _acc + (1.0 - alpha) * float(_v)
        wlf[_i] = _acc
    osc = []
    if armed.sum() > 200:
        for lo in np.arange(0.0, 0.30, 0.02):
            m = armed & (th >= lo) & (th < lo + 0.02)
            if m.sum() < 150:
                continue
            osc.append({
                "thr": round(float(0.5 * (2 * lo + 0.02)), 2),
                "wlf": round(float(np.sqrt((wlf[m] ** 2).mean())), 3),
                "attpp": round(float(np.percentile(eul[m, 0], 99) - np.percentile(eul[m, 0], 1) +
                                     np.percentile(eul[m, 1], 99) - np.percentile(eul[m, 1], 1)), 1),
                "n": int(m.sum()),
            })

    series = envelope(t, {
        "roll": eul[:, 0], "pitch": eul[:, 1], "yaw": eul[:, 2],
        "wx": s["gyro_x"], "wy": s["gyro_y"], "wz": s["gyro_z"],
        "spx": s["rate_sp_x"], "spy": s["rate_sp_y"], "spz": s["rate_sp_z"],
        "tx": s["torque_x"], "ty": s["torque_y"], "tz": s["torque_z"],
        "thr": th, "man": man,
        "m0": motor[:, 0], "m1": motor[:, 1], "m2": motor[:, 2], "m3": motor[:, 3],
        "mavg": s.motor_avg, "mspread": s.motor_spread,
        "scale": scl,
        "acc": acc, "gdev": gdev, "gyroabs": gyro,
        "hgt": s["height_m"], "vv": s["vert_vel_mps"],
        "brel": s["baro_rel_m"], "babs": s["baro_abs_m"],
        "link": link.astype(float),
        "tgtroll": s["target_roll_deg"], "tgtpitch": s["target_pitch_deg"],
        "wrms": np.sqrt(np.convolve(gyro ** 2, np.ones(50) / 50.0, "same")),
        "attpp": np.maximum(
            np.convolve(np.abs(eul[:, 0]), np.ones(50) / 50.0, "same"),
            np.convolve(np.abs(eul[:, 1]), np.ones(50) / 50.0, "same"),
        ),
    })

    armed_segs = [(round(float(t[a]), 2), round(float(t[z - 1]), 2)) for a, z in segments(armed)]

    flags_used = {k: int(v.sum()) for k, v in s.flags_set.items() if v.any()}
    events_used = {k: int(v.sum()) for k, v in s.events_set.items() if v.any()}

    return {
        "id": s.session_id,
        "file": s.name,
        "rows": s.n,
        "tickMs": [int(s["tickMs"][0]), int(s["tickMs"][-1])],
        "duration": round(float(t[-1]), 2),
        "rate": round(float(s.n / max(t[-1], 1e-9)), 2),
        "lost_frames": lost,
        "lost_pct": round(float(100 * lost / max(s.n + lost, 1)), 2),
        "gap_explained_pct": round(float(100 * gap_ok.mean()), 2),
        "loop_us": {"p50": float(np.median(lp)), "p95": float(np.percentile(lp, 95)),
                    "max": float(lp.max()), "over_pct": round(float(100 * (lp > 2500).mean()), 2)},
        "flags_used": flags_used,
        "events_used": events_used,
        "segments": segs,
        "armed_segs": armed_segs,
        "armed_frames": int(armed.sum()),
        "angle_frames": int(angle.sum()),
        "failsafe_frames": int(s.flags_set["Failsafe"].sum()),
        "link_lost_frames": int((~s.flags_set["LinkOk"]).sum()),
        "link_max": int(link.max()),
        "quat_norm": [round(float(np.linalg.norm(s.q, axis=1).min()), 5),
                      round(float(np.linalg.norm(s.q, axis=1).max()), 5)],
        "att_range": {"roll": [round(float(eul[:, 0].min()), 2), round(float(eul[:, 0].max()), 2)],
                      "pitch": [round(float(eul[:, 1].min()), 2), round(float(eul[:, 1].max()), 2)],
                      "yaw": [round(float(eul[:, 2].min()), 2), round(float(eul[:, 2].max()), 2)]},
        "gyro_max": round(float(gyro.max()), 2),
        "gyro_peak": {"v": round(float(gyro.max()), 2), "t": round(float(t[int(np.argmax(gyro))]), 2),
                      "armed": bool(armed[int(np.argmax(gyro))])},
        "gyro_max_armed": round(float(gyro[armed].max()), 2) if armed.any() else None,
        "acc_min": round(float(acc.min()), 2),
        "acc_max": round(float(acc.max()), 2),
        "thr_max": round(float(th.max()), 3),
        "man_max": round(float(man.max()), 3),
        "motor_max": int(motor.max()),
        "motor_frac_max": round(float(motor_frac.max()), 3),
        "baro_abs_range": [round(float(s["baro_abs_m"].min()), 2), round(float(s["baro_abs_m"].max()), 2)],
        "hgt_raw_range": [round(float(s["height_m"].min()), 2), round(float(s["height_m"].max()), 2)],
        "bias_last": [round(float(s["gyro_bias_x"][bidx[-1]]), 5) if len(bidx) else 0.0,
                      round(float(s["gyro_bias_y"][bidx[-1]]), 5) if len(bidx) else 0.0,
                      round(float(s["gyro_bias_z"][bidx[-1]]), 5) if len(bidx) else 0.0],
        "bias_range_z": [round(float(s["gyro_bias_z"][bidx].min()), 5) if len(bidx) else 0.0,
                         round(float(s["gyro_bias_z"][bidx].max()), 5) if len(bidx) else 0.0],
        "bias": bias,
        "static": static_rows,
        "osc": osc,
        "vib": vib,
        "has_baro": has_baro,
        "ekf_drag": ekf_drag(s, armed),
        "mixer_check": mixer_check(s, scl, _d, armed),
        "gyro_vs_ekf": gyro_vs_ekf(s, armed),
        "events": ev,
        "flags_ts": flags_ts,
        "flags_dt": 0.1,
        "series": series,
    }


def main():
    sessions = load_all()
    out = {
        "source_dir": LOG_DIR,
        "batch": BATCH or "all",
        "params": {
            "loop_hz": 500.0, "log_hz": 50.0, "hover_throttle": HOVER_THROTTLE,
            "mass_kg": MASS_KG, "dshot_unit": DSHOT_UNIT, "dshot_offset": DSHOT_OFFSET,
            "motor_map": MOTOR_MAP, "link_timeout_ms": LINK_TIMEOUT_MS,
            "gyro_bias_limit": GYRO_BIAS_LIMIT,
        },
        "sessions": [analyze_session(s) for s in sessions],
    }
    path = os.path.join(OUT_DIR, f"analysis{SUFFIX}.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(out, fh, ensure_ascii=False, separators=(",", ":"))
    print("wrote", path, os.path.getsize(path), "bytes")
    for s in out["sessions"]:
        ev = {}
        for r in s["events"]:
            ev[r["kind"]] = ev.get(r["kind"], 0) + 1
        print(f"session {s['id']}: {s['rows']} rows, {s['duration']}s, lost {s['lost_frames']} "
              f"({s['lost_pct']}%), armed segs {len(s['segments'])}, "
              f"events {len(s['events'])}")


if __name__ == "__main__":
    main()
