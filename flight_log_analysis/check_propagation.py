"""逐帧检验 EKF 的姿态传播到底用的是哪种四元数乘法:
    P_body  = q ⊗ exp((ω−b)Δt)     (机体系增量, 正确写法)
    P_world = exp((ω−b)Δt) ⊗ q     (世界系增量, 错误写法)
用日志里相邻两帧的四元数 + 同步陀螺/零偏 预测下一帧, 看哪个预测更接近 ==> 反推实际实现。
再比较"只用陀螺"与"实测", 差值就是这一拍里加速度计修正的贡献。
"""
from __future__ import annotations

import io
import os

import numpy as np

from load_logs import load_all

OUT = io.StringIO()


def out(*a):
    print(*a)
    print(*a, file=OUT)


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qaxis(v, ang):
    n = np.linalg.norm(v)
    if n < 1e-12:
        return np.array([1.0, 0, 0, 0])
    a = ang / 2.0
    s = np.sin(a) / n
    return np.array([np.cos(a), v[0] * s, v[1] * s, v[2] * s])


def ang_between(qa, qb):
    d = qmul(qconj(qa), qb)
    return np.degrees(2 * np.arccos(np.clip(abs(d[0]), -1, 1)))


def test(sid, t0, t1, lbl, dt=0.02):
    s = [x for x in load_all() if x.session_id == sid][0]
    t = s.t
    bi = np.flatnonzero(np.abs(s["gyro_bias_x"]) + np.abs(s["gyro_bias_y"]) + np.abs(s["gyro_bias_z"]) > 0)
    bg = np.stack([np.interp(np.arange(len(t)), bi, s[k][bi]) for k in
                   ("gyro_bias_x", "gyro_bias_y", "gyro_bias_z")], axis=1)
    a, b = np.searchsorted(t, t0), np.searchsorted(t, t1)
    errs = {"body_bias": [], "world_bias": [], "body_raw": []}
    for i in range(a, b - 1):
        q0, q1 = s.q[i] / np.linalg.norm(s.q[i]), s.q[i + 1] / np.linalg.norm(s.q[i + 1])
        w = np.array([s["gyro_x"][i], s["gyro_y"][i], s["gyro_z"][i]])
        dd = qaxis(w - bg[i], dt)
        errs["body_bias"].append(ang_between(qmul(q0, dd), q1))
        errs["world_bias"].append(ang_between(qmul(dd, q0), q1))
        errs["body_raw"].append(ang_between(qmul(q0, qaxis(w, dt)), q1))
    out(f"{lbl} ({t0:.1f}~{t1:.1f}s, {b-a-1} 帧):")
    for k, v in errs.items():
        v = np.array(v)
        out(f"   {k:11s}: 每帧预测误差 中位 {np.median(v):6.3f}°  均值 {v.mean():6.3f}°  p95 {np.percentile(v,95):6.3f}°")
    out("")


out("逐帧姿态传播检验(误差越小说明越接近实际实现)")
test(4, 154.0, 161.8, "s4 静止(桨停, yaw≈−52°)")
test(4, 162.0, 169.0, "s4 桨转未离地(yaw −50→+64°)")
test(4, 169.5, 171.5, "s4 离地瞬间(pitch 跑偏那 2s)")
test(4, 178.0, 180.0, "s4 撞击 |ω|≈6.9 rad/s")
test(4, 236.0, 237.0, "s4 第二次飞行翻滚 |ω|≈15.7 rad/s")
test(8, 170.0, 200.0, "s8 台架 30s(对照)")

with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "propagation_check.txt"),
          "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())
