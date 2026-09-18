"""检验: 气压计的虚假高度(桨流)是否通过"位置->速度->姿态/零偏"的协方差耦合,
把姿态估计和零偏估计一起带走 —— 这能解释"为什么一启动电机 by 就开始涨"。

气压量测在代码里只挂位置: H[0][FF_E_DP2]=1。但卡尔曼增益 K = P·Hᵀ/S 会取 P 的
"pz 那一列", 所以只要 P 里 pz 与速度/姿态/零偏有交叉协方差(由 F 的耦合累积),
气压的大新息就会同时改动速度、姿态与零偏。
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


def segs(armed):
    idx = np.flatnonzero(np.diff(armed.astype(np.int8)) != 0) + 1
    b = [0, *idx, len(armed)]
    return [(x, z) for x, z in zip(b[:-1], b[1:]) if armed[x]]


out("① 各解锁段: 气压计虚假高度 vs 姿态/零偏 的幅度对比(全部相对解锁时刻)")
out(f"{'会话':>4} {'起止s':>16} {'时长':>6} {'baroRel 范围':>16} {'height 范围':>16} "
    f"{'baro-h 差异 最大':>15} {'pitch 范围':>15} {'by 变化':>9} {'bz 变化':>9}")
for s in load_all():
    t = s.t
    bi = np.flatnonzero(np.abs(s["gyro_bias_x"]) + np.abs(s["gyro_bias_y"]) + np.abs(s["gyro_bias_z"]) > 0)
    for a, z in segs(s.armed):
        br, hg = s["baro_rel_m"][a:z], s["height_m"][a:z]
        dif = br - hg
        e = s.euler_deg[a:z]
        by = np.interp(np.arange(a, z), bi, s["gyro_bias_y"][bi]) if len(bi) else np.zeros(1)
        bzs = np.interp(np.arange(a, z), bi, s["gyro_bias_z"][bi]) if len(bi) else np.zeros(1)
        out(f"{s.session_id:>4} {t[a]:7.1f}~{t[z-1]:7.1f} {t[z-1]-t[a]:6.1f} "
            f"{br.min():+7.2f}~{br.max():+6.2f} {hg.min():+7.2f}~{hg.max():+6.2f} "
            f"{np.abs(dif).max():15.2f} {e[:,1].min():+6.1f}~{e[:,1].max():+5.1f} "
            f"{np.degrees(by[-1]-by[0]):+8.2f}° {np.degrees(bzs[-1]-bzs[0]):+8.2f}°")

out("")
out("② 每次解锁后 6s 内: 气压(相对起飞基准)的变化 与 by/姿态 的变化 的对应关系")
for s in load_all():
    t = s.t
    bi = np.flatnonzero(np.abs(s["gyro_bias_x"]) + np.abs(s["gyro_bias_y"]) + np.abs(s["gyro_bias_z"]) > 0)
    for a, z in segs(s.armed):
        b = min(a + 300, z)          # 6s
        if b - a < 100:
            continue
        br = s["baro_rel_m"][a:b]
        pitch = s.euler_deg[a:b, 1]
        by = np.interp(np.arange(a, b), bi, s["gyro_bias_y"][bi]) if len(bi) else np.zeros(b - a)
        bz = np.interp(np.arange(a, b), bi, s["gyro_bias_z"][bi]) if len(bi) else np.zeros(b - a)
        hg = s["height_m"][a:b]
        out(f"  s{s.session_id} {t[a]:6.1f}s 起 6s: baroRel {br[0]:+.2f}→{br[-1]:+.2f} m "
            f"(EKF 高度 {hg[0]:+.2f}→{hg[-1]:+.2f}), pitch {pitch[0]:+.2f}→{pitch[-1]:+.2f}°, "
            f"by {np.degrees(by[0]):+.2f}→{np.degrees(by[-1]):+.2f} °/s, "
            f"bz {np.degrees(bz[0]):+.2f}→{np.degrees(bz[-1]):+.2f} °/s")

out("")
out("③ 逐条相关: d(baroRel)/dt 与 by 变化率(200ms 采样)的相关性")
for s in load_all():
    t = s.t
    bi = np.flatnonzero(np.abs(s["gyro_bias_x"]) + np.abs(s["gyro_bias_y"]) + np.abs(s["gyro_bias_z"]) > 0)
    if len(bi) < 20:
        continue
    br = s["baro_rel_m"][bi]
    dbr = np.gradient(br, t[bi])
    dby = np.gradient(np.degrees(s["gyro_bias_y"][bi]), t[bi])
    dbz = np.gradient(np.degrees(s["gyro_bias_z"][bi]), t[bi])
    m = np.abs(dbr) < 5
    out(f"  s{s.session_id}: n={m.sum():4d}  corr(d baro/dt, d by/dt) = {np.corrcoef(dbr[m], dby[m])[0,1]:+.3f}  "
        f"corr(d baro/dt, d bz/dt) = {np.corrcoef(dbr[m], dbz[m])[0,1]:+.3f}")

with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "baro_coupling.txt"),
          "w", encoding="utf-8") as fh:
    fh.write(OUT.getvalue())
