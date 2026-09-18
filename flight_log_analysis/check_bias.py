"""验证"姿态 yaw 的额外旋转 = -EKF 零偏估计"这件事。

EKF 的名义积分是 q <- q * exp((gyro - bg) dt)(flight_control_fliter.hpp:225),
所以姿态解出来的 yaw 角速度应当等于陀螺贡献 + (-bg_z 贡献)。
如果两者吻合, 说明零偏估计错了多少, 姿态就跟着多转多少。
"""
import numpy as np

from load_logs import load_all


def decompose(s, win_s=2.0):
    t = s.t
    e = s.euler_deg
    yu = np.unwrap(np.radians(e[:, 2]))
    ru, pu = np.radians(e[:, 0]), np.radians(e[:, 1])
    k = max(int(win_s / 0.02), 1)
    ker = np.ones(k) / k
    dyaw = np.convolve(np.gradient(yu, t), ker, "same")
    gyro_part = (s["gyro_z"] * np.cos(ru) + s["gyro_y"] * np.sin(ru)) / np.cos(pu)
    extra = dyaw - gyro_part
    return t, dyaw, gyro_part, extra


for s in load_all():
    t, dyaw, gyro_part, extra = decompose(s)
    bidx = np.flatnonzero(np.abs(s["gyro_bias_z"]) > 0)
    bz = np.interp(np.arange(s.n), bidx, s["gyro_bias_z"][bidx]) if len(bidx) else np.zeros(s.n)
    m = np.abs(s.euler_deg[:, 1]) < 25
    print("s%d  相关(多出来的yaw角速度, -零偏估计) = %.3f, 中位残差 %+.3f deg/s"
          % (s.session_id, np.corrcoef(extra[m], -np.degrees(bz[m]))[0, 1],
             np.degrees(np.median(extra[m] + bz[m]))))
    for frac in (0.05, 0.25, 0.5, 0.75, 0.95):
        i = int(frac * (s.n - 1))
        print("    t=%6.1fs  yaw角速度 %+7.2f  陀螺贡献 %+7.2f  多出来 %+7.2f  "
              "(-零偏估计) %+7.2f  deg/s"
              % (t[i], np.degrees(dyaw[i]), np.degrees(gyro_part[i]),
                 np.degrees(extra[i]), -np.degrees(bz[i])))
