"""读取 desktopApp/flight_logs 下回传的会话 CSV, 解析成 numpy 数组。

CSV 表头与 FlightLogWriter.HEADER 严格对应, 字段含义见
DL_LIB/W25Q128/dlx_flash_manager_config.h 的 LogEntry。
"""
from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass, field

import numpy as np

LOG_DIR = r"E:\kmpFly\desktopApp\flight_logs"

"""批次过滤: 设环境变量 FLIGHT_LOG_BATCH=20260914 就只读这一天的会话。
所有分析脚本都通过 load_all() 取数, 因此这一个开关就能切换整批数据。"""
BATCH = os.environ.get("FLIGHT_LOG_BATCH")


def out_path(stem: str) -> str:
    """检查脚本文本输出路径(带批次后缀, 避免覆盖上一批结果)。"""
    suffix = f"_{BATCH}" if BATCH else ""
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), f"{stem}{suffix}.txt")

# ---------------------------------------------------------------- flags / events
FLAG_BITS = {
    "LinkOk": 0x0001,
    "ImuOk": 0x0002,
    "BaroOk": 0x0004,
    "AngleLoop": 0x0008,
    "MotorEnabled": 0x0010,
    "MotorStarting": 0x0020,
    "HardStop": 0x0040,
    "FlashOk": 0x0080,
    "LogFull": 0x0100,
    "LogError": 0x0200,
    "MotorSat": 0x0400,
    "Failsafe": 0x0800,
    "GroundMode": 0x1000,
}

EVENT_BITS = {
    "CommandChanged": 0x0001,
    "Arm": 0x0002,
    "Disarm": 0x0004,
    "Failsafe": 0x0008,
    "MotorSat": 0x0010,
    "LogFault": 0x0020,
    "BaroUpdate": 0x0040,
    "EkfRezero": 0x0080,
}

FLOAT_COLS = [
    "quat_w", "quat_x", "quat_y", "quat_z",
    "gyro_x", "gyro_y", "gyro_z",
    "acc_x", "acc_y", "acc_z",
    "rate_sp_x", "rate_sp_y", "rate_sp_z",
    "torque_x", "torque_y", "torque_z",
    "target_pitch_deg", "target_roll_deg", "target_height_m",
    "throttle", "manual_throttle", "height_m", "vert_vel_mps",
    "baro_rel_m", "baro_abs_m",
    "gyro_bias_x", "gyro_bias_y", "gyro_bias_z",
]

INT_COLS = [
    "sessionId", "seq", "tickMs", "flags", "events",
    "linkAgeMs", "loopPeriodUs",
    "motor0", "motor1", "motor2", "motor3",
]


@dataclass
class Session:
    name: str
    path: str
    data: dict[str, np.ndarray] = field(default_factory=dict)

    def __getitem__(self, k: str) -> np.ndarray:
        return self.data[k]

    def __contains__(self, k: str) -> bool:
        return k in self.data

    @property
    def n(self) -> int:
        return len(self.data["tickMs"])

    @property
    def session_id(self) -> int:
        return int(self.data["sessionId"][0])

    # ---- 派生量 ----
    @property
    def t(self) -> np.ndarray:
        """相对会话起始的时刻 [s]"""
        return (self.data["tickMs"] - self.data["tickMs"][0]) / 1000.0

    @property
    def q(self) -> np.ndarray:
        d = self.data
        return np.stack([d["quat_w"], d["quat_x"], d["quat_y"], d["quat_z"]], axis=1)

    @property
    def euler_deg(self) -> np.ndarray:
        """(roll, pitch, yaw) [deg], ZYX 顺序, 与 flight_log_analysis.html 一致"""
        q = self.q
        w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
        roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
        pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0)))
        yaw = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        return np.stack([roll, pitch, yaw], axis=1)

    @property
    def acc_mag(self) -> np.ndarray:
        d = self.data
        return np.sqrt(d["acc_x"] ** 2 + d["acc_y"] ** 2 + d["acc_z"] ** 2)

    @property
    def gyro_mag(self) -> np.ndarray:
        d = self.data
        return np.sqrt(d["gyro_x"] ** 2 + d["gyro_y"] ** 2 + d["gyro_z"] ** 2)

    @property
    def motor(self) -> np.ndarray:
        d = self.data
        return np.stack([d["motor0"], d["motor1"], d["motor2"], d["motor3"]], axis=1)

    @property
    def motor_avg(self) -> np.ndarray:
        return self.motor.mean(axis=1)

    @property
    def motor_spread(self) -> np.ndarray:
        """四路电机最大-最小, 反映混控差动量"""
        m = self.motor
        return m.max(axis=1) - m.min(axis=1)

    @property
    def armed(self) -> np.ndarray:
        return (self.data["flags"] & FLAG_BITS["MotorEnabled"]).astype(bool)

    @property
    def flags_set(self) -> dict[str, np.ndarray]:
        f = self.data["flags"]
        return {k: (f & v).astype(bool) for k, v in FLAG_BITS.items()}

    @property
    def events_set(self) -> dict[str, np.ndarray]:
        e = self.data["events"]
        return {k: (e & v).astype(bool) for k, v in EVENT_BITS.items()}

    @property
    def loop_hz(self) -> float:
        return 1e6 / float(np.median(self.data["loopPeriodUs"]))


def load(path: str) -> Session:
    with open(path, "r", encoding="utf-8", newline="") as fh:
        rdr = csv.reader(fh)
        header = next(rdr)
        cols = {h: [] for h in header}
        for row in rdr:
            if not row or len(row) != len(header):
                continue
            for h, v in zip(header, row):
                cols[h].append(v)

    data: dict[str, np.ndarray] = {}
    for k, vals in cols.items():
        if k in INT_COLS:
            data[k] = np.array([int(float(v)) for v in vals], dtype=np.int64)
        else:
            data[k] = np.array([float(v) for v in vals], dtype=np.float64)

    return Session(name=os.path.basename(path), path=path, data=data)


def load_all(names: list[str] | None = None) -> list[Session]:
    if names is None and BATCH:
        names = [BATCH]
    out: list[Session] = []
    for fn in sorted(os.listdir(LOG_DIR)):
        if not fn.startswith("session_") or not fn.endswith(".csv"):
            continue
        if names and not any(n in fn for n in names):
            continue
        out.append(load(os.path.join(LOG_DIR, fn)))
    out.sort(key=lambda s: s.session_id)
    return out


def bit_summary(bits: dict[str, np.ndarray]) -> str:
    return ", ".join(f"{k}={int(v.sum())}" for k, v in bits.items() if v.any())


if __name__ == "__main__":
    for s in load_all():
        eul = s.euler_deg
        print(f"=== session {s.session_id}  ({s.name})")
        print(f"  条数 {s.n}  时长 {s.t[-1]:.2f}s  实际频率 {s.n / max(s.t[-1], 1e-9):.2f}Hz  "
              f"循环中位 {s.loop_hz:.1f}Hz  tick {s['tickMs'][0]}~{s['tickMs'][-1]}ms")
        print(f"  flags used: {bit_summary(s.flags_set)}")
        print(f"  events     : {bit_summary(s.events_set)}")
        print(f"  linkAgeMs  max={s['linkAgeMs'].max()} p99={np.percentile(s['linkAgeMs'], 99):.0f}")
        print(f"  roll  {eul[:, 0].min():7.2f}~{eul[:, 0].max():7.2f} deg  "
              f"pitch {eul[:, 1].min():7.2f}~{eul[:, 1].max():7.2f}  "
              f"yaw {eul[:, 2].min():8.2f}~{eul[:, 2].max():8.2f}")
        print(f"  acc   {s.acc_mag.min():.2f}~{s.acc_mag.max():.2f} g   "
              f"vertVel {s['vert_vel_mps'].min():.2f}~{s['vert_vel_mps'].max():.2f} m/s  "
              f"height {s['height_m'].min():.1f}~{s['height_m'].max():.1f} m")
        print(f"  baroRel {s['baro_rel_m'].min():.2f}~{s['baro_rel_m'].max():.2f} m  "
              f"baroAbs {s['baro_abs_m'].min():.1f}~{s['baro_abs_m'].max():.1f} m")
        print(f"  throttle {s['throttle'].min():.3f}~{s['throttle'].max():.3f}  "
              f"manual {s['manual_throttle'].min():.3f}~{s['manual_throttle'].max():.3f}")
        print(f"  motor {s.motor.min()}~{s.motor.max()}  avg {s.motor_avg.mean():.1f}  "
              f"spread max {s.motor_spread.max()}")
        print(f"  gyro  {np.abs(np.stack([s['gyro_x'], s['gyro_y'], s['gyro_z']])).max():.2f} rad/s")
        print(f"  torque max abs "
              f"{np.abs(np.stack([s['torque_x'], s['torque_y'], s['torque_z']])).max():.4f} N*m")
        print(f"  rate_sp max abs "
              f"{np.abs(np.stack([s['rate_sp_x'], s['rate_sp_y'], s['rate_sp_z']])).max():.3f} rad/s")
        print()
