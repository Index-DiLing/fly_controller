#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""run_baro.py —— 气压计(BMP388/I2C1)台架自动化（baro bench 串口控制台）

目标：把运行期偶发的 0x404(BARO_READ_FAILED) 从"读寄存器失败"这个笼统说法拆成可判别的来源：
      kind=tx（I2C 事务失败） vs kind=comp（突发读成功但补偿结果非法）、
      失败是否成簇（一次 NACK/AF 粘连后连续失败） vs 单次偶发、
      inject NACK 后要几次尝试才恢复（AF 未清假设的决定性实验）。

owner 只需接线/上电；脚本按固定顺序跑测量，把原始日志 + 解析摘要 + 结构化 CSV 落到
test-data/baro_bench/<YYYYMMDD-HHMMSS>/：
    raw.log     串口原始行 + 本脚本发出的命令（`        >>> cmd`）
    summary.md  人工判读摘要（含结论）
    fails.csv   所有 [fail] 行结构化
    inject.csv  所有 [inject] 行结构化

用法
    python tools/bench/run_baro.py --selftest          # 不开串口，跑内嵌合成日志验收解析/汇总
    python tools/bench/run_baro.py --list              # 列出串口
    python tools/bench/run_baro.py                     # 自动找口，跑默认流程
    python tools/bench/run_baro.py --port COM11 --soak 60000 --inject 20

默认实验流程（脚本实际发出的命令，见 summary.md 的"命令序列"一节）
    ver → cfg → sr → bus → reset → init（必须 result=OK，否则数据不可信）
    → soak <soak>ms @400k（器件初始化后的速度）
    → inject <n> → recover@post-inject（趁 SR1 还带 AF 采 sr1_before）
    → speed 100000 + soak <side>ms
    → speed 400000 + soak <side>ms
    → stress on + soak <side>ms → stress off
    → recover → stat

退出码：0 正常 / 1 有步骤判据失败 / 2 串口或环境问题 / 3 版本门禁不过 / 4 器件未初始化。
`init` 的超时固定 5s —— **未**实现 `--init-timeout` 开关（需要再加）。

版本门禁：只接受含 `[ver] baro bench r3 2026-10-18` 的镜像；不匹配立即报错退出（退出码 3），
旧的 baro bench 镜像数据一律作废。

设计要点
    - 解析层（analyze/summary/csv）与串口层完全解耦：--selftest 把内嵌合成日志喂给同一套
      解析+汇总代码，因此本机没有 ST-LINK/真板子时也能真验收（本任务唯一可跑的验收）。
    - 命令行回声（`>>> cmd`）也进 raw.log 与分析输入，soak 段才能被可靠地按时序切开。
    - 失败聚类用固件的 `streak` 字段（=1 说明上一条尝试成功，属于新簇）为主判据；
      streak 缺失时退回"相邻失败 t 间隙 > 30ms 即新簇"。
"""

import argparse
import csv
import datetime as _dt
import queue
import re
import sys
import tempfile
import threading
import time
from collections import Counter
from pathlib import Path

# pyserial 只有真串口路径需要：--selftest 不依赖它（本机验收必须能离线跑）。
try:
    import serial
    from serial.tools import list_ports
    HAVE_SERIAL = True
except ImportError:                                   # pragma: no cover - 本机已装 3.5
    serial = None
    list_ports = None
    HAVE_SERIAL = False

REPO = Path(__file__).resolve().parent.parent.parent
OUT_ROOT = REPO / "test-data" / "baro_bench"

# Windows 控制台默认 GBK：结论里的箭头/符号可能编不出来，宁可变成 "?" 也不要崩在 print 上
# （summary.md 本身按 UTF-8 落盘，不受影响）。
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(errors="replace")       # type: ignore[union-attr]
    except Exception:
        pass

# 冻结的版本契约（T1 定义，不得改动）
BARO_VER_LINE = "[ver] baro bench r3 2026-10-18"
BARO_VER_TOKEN = "baro bench r3 2026-10-18"

DEFAULT_SOAK_MS = 60000          # 主 soak
DEFAULT_SIDE_SOAK_MS = 20000     # 变速/带负载 soak
DEFAULT_INJECT = 20
DEFAULT_SPEED = 400000           # 被测主速度（cfg 里 hw.baro.kSpeed）
LOW_SPEED = 100000               # 变速对照点
CLUSTER_GAP_US = 30000           # streak 缺失时的聚类间隙阈值

# --------------------------------------------------------------------------------------------------
# 行解析：正则 + 数值类型表（严格按 T1 冻结的字段名/顺序；截断行必须解析失败）
# --------------------------------------------------------------------------------------------------
_RE_VER = re.compile(r"\[ver\]\s*(?P<text>.+)$")
_RE_CFG = re.compile(
    r"\[cfg\]\s+i2c=(?P<i2c>\S+)\s+scl=(?P<scl>\S+)\s+sda=(?P<sda>\S+)\s+addr=(?P<addr>\S+)"
    r"\s+speed=(?P<speed>\d+)\s+osr=(?P<osr>\S+)\s+odr=(?P<odr>\S+)\s+cfg=(?P<cfg>\S+)"
    r"\s+int=(?P<intc>\S+)\s+pwr=(?P<pwr>\S+)")
_RE_SR = re.compile(
    r"\[sr\]\s+sr1=(?P<sr1>0x[0-9A-Fa-f]+)\s+sr2=(?P<sr2>0x[0-9A-Fa-f]+)\s+af=(?P<af>\d+)"
    r"\s+berr=(?P<berr>\d+)\s+arlo=(?P<arlo>\d+)\s+busy=(?P<busy>\d+)\s+msL=(?P<msl>\d+)"
    r"\s+tra=(?P<tra>\d+)\s+txE=(?P<txe>\d+)\s+rxne=(?P<rxne>\d+)\s+btf=(?P<btf>\d+)"
    r"\s+ccr=(?P<ccr>\d+)\s+trise=(?P<trise>\d+)\s+ackCfg=(?P<ack>\d+)")
_RE_BUS = re.compile(
    r"\[bus\]\s+scl=(?P<scl>\d+)\s+sda=(?P<sda>\d+)\s+cpu=(?P<cpu>\d+)\s+cycPerUs=(?P<cyc>\d+)")
_RE_INIT = re.compile(
    r"\[init\]\s+speed=(?P<speed>\d+)\s+stage=(?P<stage>\w+)\s+result=(?P<result>\w+)"
    r"\s+err=(?P<err>0x[0-9A-Fa-f]+)\s+chipId=(?P<chipid>0x[0-9A-Fa-f]+)\s+attempts=(?P<attempts>\d+)")
_RE_SOAK_END = re.compile(
    r"\[soak\.end\]\s+t=(?P<t>\d+)\s+att=(?P<att>\d+)\s+new=(?P<new>\d+)\s+notready=(?P<notready>\d+)"
    r"\s+fail=(?P<fail>\d+)\s+tx=(?P<tx>\d+)\s+comp=(?P<comp>\d+)\s+failrate=(?P<failrate>\S+)"
    r"\s+maxgapUs=(?P<maxgap>\d+)\s+maxdurUs=(?P<maxdur>\d+)\s+evTimeouts=(?P<evtimeouts>\d+)"
    r"\s+recovered=(?P<recovered>\d+)")
_RE_SOAK_TICK = re.compile(
    r"\[soak\]\s+t=(?P<t>\d+)\s+att=(?P<att>\d+)\s+new=(?P<new>\d+)\s+notready=(?P<notready>\d+)"
    r"\s+fail=(?P<fail>\d+)\s+tx=(?P<tx>\d+)\s+comp=(?P<comp>\d+)\s+maxgapUs=(?P<maxgap>\d+)"
    r"\s+maxdurUs=(?P<maxdur>\d+)")
# [fail] 行不用一条长正则：T1 会新增 kind=other err=0x....，字段可能多一个/顺序微调。
# 这里"头 + 逐字段提取"：头必须匹配，且 9 个核心字段一个都不能少（截断行仍然必须解析失败），
# 但允许出现额外的 err= 字段。strictness 由每个字段自己的模式保证。
_RE_FAIL_HEAD = re.compile(
    r"\[fail\]\s+i=(?P<i>\d+)\s+t=(?P<t>\d+)\s+kind=(?P<kind>tx|comp|other)(?:\s|$)")
_FAIL_FIELDS = (
    # (record key, 行内字段名, 值的模式, 是否 16 进制, 是否必填)
    # 必填 9 项缺一个就整行不解析（截断/畸形行）；scl/sda 是 r2 新增的**可选**字段，
    # 缺失时留空即可 —— 这样 r1 风格的旧行、以及不带电平的行照旧能解析。
    ("sr1", "sr1", r"0x[0-9A-Fa-f]{1,4}", True, True),
    ("sr2", "sr2", r"0x[0-9A-Fa-f]{1,4}", True, True),
    ("lastev", "lastEv", r"0x[0-9A-Fa-f]{1,8}", True, True),
    ("evd", "evD", r"\d+", False, True),
    ("status", "status", r"0x[0-9A-Fa-f]{1,2}", True, True),
    ("raw", "raw", r"[0-9A-Fa-f]{12}", True, True),   # raw 是裸 12 位十六进制，没有 0x 前缀
    ("dur", "durUs", r"\d+", False, True),
    ("gap", "gapUs", r"\d+", False, True),
    ("streak", "streak", r"\d+", False, True),
    ("scl", "scl", r"[01]", False, False),          # A21: 失败瞬间 PB6(SCL) 电平
    ("sda", "sda", r"[01]", False, False),          # A21: 失败瞬间 PB7(SDA) 电平
)
_RE_FAIL_ERR = re.compile(r"(?:^|\s)err=(?P<err>0x[0-9A-Fa-f]{1,4})(?:\s|$)")
_RE_INJECT = re.compile(
    r"\[inject\]\s+i=(?P<i>\d+)\s+pre_sr1=(?P<pre_sr1>0x[0-9A-Fa-f]+)\s+af=(?P<af>\d+)"
    r"\s+nack_ok=(?P<nack_ok>\d+)\s+post_sr1=(?P<post_sr1>0x[0-9A-Fa-f]+)\s+post_af=(?P<post_af>\d+)"
    r"\s+next_ok=(?P<next_ok>\d+)\s+attempts_to_ok=(?P<attempts>-?\d+)")
_RE_SPEED = re.compile(r"\[speed\]\s+hz=(?P<hz>\d+)\s+result=(?P<result>\w+)")
_RE_STRESS = re.compile(
    r"\[stress\]\s+state=(?P<state>\w+)\s+spi=(?P<spi>\w+)\s+i2c2=(?P<i2c2>\w+)\s+adc=(?P<adc>\w+)")
_RE_IRQBLOCK = re.compile(
    r"\[irqblock\]\s+us=(?P<us>\d+)\s+periodMs=(?P<period>\d+)\s+state=(?P<state>\w+)")
_RE_RECOVER = re.compile(
    r"\[recover\]\s+sr1_before=(?P<sr1b>0x[0-9A-Fa-f]+)\s+sr2_before=(?P<sr2b>0x[0-9A-Fa-f]+)"
    r"\s+sr1_after=(?P<sr1a>0x[0-9A-Fa-f]+)\s+sr2_after=(?P<sr2a>0x[0-9A-Fa-f]+)"
    r"\s+next_ok=(?P<next_ok>\d+)")
_RE_STAT = re.compile(
    r"\[stat\]\s+att=(?P<att>\d+)\s+new=(?P<new>\d+)\s+notready=(?P<notready>\d+)\s+fail=(?P<fail>\d+)"
    r"\s+tx=(?P<tx>\d+)\s+comp=(?P<comp>\d+)\s+evTimeouts=(?P<evtimeouts>\d+)"
    r"\s+recovered=(?P<recovered>\d+)\s+maxgapUs=(?P<maxgap>\d+)\s+maxdurUs=(?P<maxdur>\d+)")
_RE_ERR = re.compile(r"\[err\]\s*(?P<text>.+)$")

_SOAK_INTS = ("t", "att", "new", "notready", "fail", "tx", "comp", "maxgap", "maxdur")
_STAT_INTS = ("att", "new", "notready", "fail", "tx", "comp", "evtimeouts", "recovered",
              "maxgap", "maxdur")

# (kind, 正则, 需要转 int 的字段, 需要按 16 进制转 int 的字段)
_RULES = (
    ("ver", _RE_VER, (), ()),
    ("soak_end", _RE_SOAK_END, _SOAK_INTS + ("evtimeouts", "recovered"), ()),
    ("soak_tick", _RE_SOAK_TICK, _SOAK_INTS, ()),
    ("inject", _RE_INJECT, ("i", "af", "nack_ok", "post_af", "next_ok", "attempts"),
     ("pre_sr1", "post_sr1")),
    ("cfg", _RE_CFG, ("speed",), ()),
    ("sr", _RE_SR,
     ("af", "berr", "arlo", "busy", "msl", "tra", "txe", "rxne", "btf", "ccr", "trise", "ack"),
     ("sr1", "sr2")),
    ("bus", _RE_BUS, ("scl", "sda", "cpu", "cyc"), ()),
    ("init", _RE_INIT, ("speed", "attempts"), ("err", "chipid")),
    ("speed", _RE_SPEED, ("hz",), ()),
    ("stress", _RE_STRESS, (), ()),
    ("irqblock", _RE_IRQBLOCK, ("us", "period"), ()),
    ("recover", _RE_RECOVER, ("next_ok",), ("sr1b", "sr2b", "sr1a", "sr2a")),
    ("stat", _RE_STAT, _STAT_INTS, ()),
    ("err", _RE_ERR, (), ()),
)


def _parse_fail(s):
    """解析 [fail] 行：头 + 9 个必填字段；可选 err= 字段（kind=other 时 T1 会带）。"""
    head = _RE_FAIL_HEAD.match(s)
    if not head:
        return None
    rec = head.groupdict()
    rec["i"] = int(rec["i"])
    rec["t"] = int(rec["t"])
    for key, token, pattern, is_hex, required in _FAIL_FIELDS:
        fm = re.search(r"(?:^|\s)" + re.escape(token) + r"=(?P<v>" + pattern + r")(?:\s|$)", s)
        if not fm:
            if required:
                return None                 # 必填字段缺失 = 截断/畸形行：宁可不解析，也不要错数据
            continue                        # 可选字段（scl/sda）缺失：留空
        value = fm.group("v")
        rec[key] = int(value, 16) if is_hex else int(value)
    em = _RE_FAIL_ERR.search(s)
    if em:
        rec["err"] = int(em.group("err"), 16)
    return rec


def parse_line(line):
    """把一行串口文本解析成 (kind, record)；不认识的/字段不全的行返回 None。"""
    s = (line or "").strip()
    if not s:
        return None
    if s.startswith("[fail]"):
        rec = _parse_fail(s)
        if rec is None:
            return None
        rec["_raw"] = s
        return "fail", rec
    for kind, rx, ints, hexs in _RULES:
        m = rx.search(s)
        if not m:
            continue
        rec = m.groupdict()
        for key in ints:
            if rec.get(key) is not None:
                rec[key] = int(rec[key])
        for key in hexs:
            if rec.get(key) is not None:
                rec[key] = int(rec[key], 16)
        rec["_raw"] = s
        return kind, rec
    return None


# --------------------------------------------------------------------------------------------------
# 汇总模型
# --------------------------------------------------------------------------------------------------
class Analysis:
    """一次运行（或一段合成日志）的全部结构化记录。"""

    def __init__(self):
        self.commands = []       # 脚本实际发出的命令（按序）
        self.ver = []            # 所有 [ver] 行
        self.boots = []          # 所有 [boot] 行（只记录，绝不参与 init 判定）
        self.segments = []       # soak 段（按 soak 命令切）
        self.fails = []          # 所有 [fail]（附 seg 标签）
        self.injects = []
        self.speeds = []
        self.stress = []
        self.irqblocks = []
        self.recovers = []
        self.stats = []
        self.inits = []
        self.errs = []
        self.cfg = None
        self.sr = None
        self.bus = None


def analyze(lines):
    """从"逻辑行"（含 `>>> cmd` 命令回声）重建一次运行的结构。

    串口层与分析层共用本函数：raw.log 里既有命令回声也有设备输出，所以同一份日志可以离线重放。
    """
    an = Analysis()
    seg = None
    cur_speed = None
    stress_on = False

    for raw in lines:
        s = (raw or "").strip()
        if not s:
            continue

        if s.startswith(">>>"):
            cmd = s[3:].strip()
            an.commands.append(cmd)
            parts = cmd.split()
            head = parts[0].lower() if parts else ""
            if head == "soak":
                seg = {
                    "cmd": cmd,
                    "n": len(an.segments) + 1,
                    "speed": cur_speed,
                    "stress": stress_on,
                    "ticks": [],
                    "fails": [],
                    "end": None,
                    "closed": False,
                }
                seg["label"] = "soak{0}@{1}{2}".format(
                    seg["n"],
                    "{0}Hz".format(cur_speed) if cur_speed else "boot",
                    "+stress" if stress_on else "")
                an.segments.append(seg)
            elif head == "stress" and len(parts) > 1:
                stress_on = parts[1].lower() == "on"
            continue

        if s.startswith("[boot]"):
            # 开机行只记录：它写着 init=<OK|FAIL>，但绝不能当成 [init] 参与初始化判定。
            an.boots.append(s)
            continue

        parsed = parse_line(s)
        if parsed is None:
            continue
        kind, rec = parsed

        if kind == "ver":
            an.ver.append(s)
        elif kind == "cfg":
            an.cfg = rec
        elif kind == "sr":
            an.sr = rec
        elif kind == "bus":
            an.bus = rec
        elif kind == "init":
            an.inits.append(rec)
        elif kind == "soak_tick":
            if seg is not None and not seg["closed"]:
                seg["ticks"].append(rec)
        elif kind == "soak_end":
            if seg is not None and not seg["closed"]:
                seg["end"] = rec
                seg["closed"] = True
            else:
                # 没有对应 soak 命令的结束行（例如日志从中间开始）：单独成段，不丢数据。
                an.segments.append({
                    "cmd": "(no soak cmd)", "n": len(an.segments) + 1, "speed": cur_speed,
                    "stress": stress_on, "ticks": [], "fails": [], "end": rec, "closed": True,
                    "label": "soak?@unmatched",
                })
        elif kind == "fail":
            open_seg = seg is not None and not seg["closed"]
            rec["seg"] = seg["label"] if open_seg else "(post-soak)"
            if open_seg:
                seg["fails"].append(rec)
            an.fails.append(rec)
        elif kind == "inject":
            an.injects.append(rec)
        elif kind == "speed":
            an.speeds.append(rec)
            cur_speed = rec.get("hz", cur_speed)
        elif kind == "stress":
            an.stress.append(rec)
        elif kind == "irqblock":
            an.irqblocks.append(rec)
        elif kind == "recover":
            an.recovers.append(rec)
        elif kind == "stat":
            an.stats.append(rec)
        elif kind == "err":
            an.errs.append(rec)

    return an


def check_version(an):
    """版本门禁：返回 (是否接受, 用于报错的版本行)。"""
    for line in an.ver:
        if BARO_VER_TOKEN in line:
            return True, line
    return False, (an.ver[-1] if an.ver else "(未收到 [ver] 行)")


def _hx(value, width):
    try:
        return "0x{0:0{1}X}".format(int(value), width)
    except (TypeError, ValueError):
        return ""


def _raw_hex(value):
    """[fail] 行的 raw= 是裸 12 位十六进制：内部按 int 存，落 CSV 时还原成十六进制字符串。"""
    try:
        return "{0:012X}".format(int(value))
    except (TypeError, ValueError):
        return ""


def cluster_fails(fails, gap_us=CLUSTER_GAP_US):
    """把同一 soak 段内的失败按 t 排序后切成"连续失败簇"。

    主判据是固件的 streak（连续失败计数）：streak==1 说明上一条尝试是成功的，必是新簇起点；
    只有在 streak 缺失/为 0 时才退回"相邻失败 t 间隙 > gap_us 即新簇"。
    """
    ordered = sorted(fails, key=lambda r: r.get("t", 0))
    clusters = []
    cur = []
    prev = None
    for rec in ordered:
        start_new = True
        if prev is not None:
            st, pst = rec.get("streak"), prev.get("streak")
            if st and pst:
                start_new = not (st > 1 and st == pst + 1)
            else:
                start_new = (rec.get("t", 0) - prev.get("t", 0)) > gap_us
        if start_new and cur:
            clusters.append(cur)
            cur = []
        cur.append(rec)
        prev = rec
    if cur:
        clusters.append(cur)
    return clusters


def fail_clusters_by_segment(an, gap_us=CLUSTER_GAP_US):
    """按段分组聚类，返回 {段标签: [簇, ...]}（保持出现顺序）。"""
    groups = {}
    for rec in an.fails:
        groups.setdefault(rec.get("seg", ""), []).append(rec)
    return {label: cluster_fails(recs, gap_us) for label, recs in groups.items()}


def _hist(values):
    return dict(sorted(Counter(values).items()))


def _hx_hist(records, key, width):
    return dict(sorted(Counter(_hx(r.get(key), width) for r in records).items()))


def _pct(part, whole):
    return "{0:.1f}%".format(100.0 * part / whole) if whole else "-"


# --------------------------------------------------------------------------------------------------
# summary.md / CSV
# --------------------------------------------------------------------------------------------------
FAIL_COLS = ["seg", "i", "t", "kind", "sr1", "sr2", "lastEv", "evD", "status", "raw",
             "durUs", "gapUs", "streak", "err", "scl", "sda"]
INJECT_COLS = ["i", "pre_sr1", "af", "nack_ok", "post_sr1", "post_af", "next_ok", "attempts_to_ok"]


def fails_csv_rows(an):
    rows = []
    for rec in sorted(an.fails, key=lambda r: (r.get("seg", ""), r.get("i", 0))):
        rows.append({
            "seg": rec.get("seg", ""),
            "i": rec.get("i", ""),
            "t": rec.get("t", ""),
            "kind": rec.get("kind", ""),
            "sr1": _hx(rec.get("sr1"), 4),
            "sr2": _hx(rec.get("sr2"), 4),
            "lastEv": _hx(rec.get("lastev"), 8),
            "evD": rec.get("evd", ""),
            "status": _hx(rec.get("status"), 2),
            "raw": _raw_hex(rec.get("raw")),
            "durUs": rec.get("dur", ""),
            "gapUs": rec.get("gap", ""),
            "streak": rec.get("streak", ""),
            "err": _hx(rec.get("err"), 4) if rec.get("err") is not None else "",
            "scl": rec.get("scl") if rec.get("scl") is not None else "",
            "sda": rec.get("sda") if rec.get("sda") is not None else "",
        })
    return rows


def injects_csv_rows(an):
    rows = []
    for rec in sorted(an.injects, key=lambda r: r.get("i", 0)):
        rows.append({
            "i": rec.get("i", ""),
            "pre_sr1": _hx(rec.get("pre_sr1"), 4),
            "af": rec.get("af", ""),
            "nack_ok": rec.get("nack_ok", ""),
            "post_sr1": _hx(rec.get("post_sr1"), 4),
            "post_af": rec.get("post_af", ""),
            "next_ok": rec.get("next_ok", ""),
            "attempts_to_ok": rec.get("attempts", ""),
        })
    return rows


def init_state_from(an):
    """从日志判定器件是否初始化成功：返回 (True/False/None, 对应的 [init] 记录)。

    只看 [init] 行；[boot] 行（`[boot] baro init=OK`）绝不参与判定。
    """
    ok = [r for r in an.inits if (r.get("result") or "").upper() == "OK"]
    bad = [r for r in an.inits if (r.get("result") or "").upper() != "OK"]
    if ok:
        return True, ok[-1]
    if bad:
        return False, bad[-1]
    return None, None


def uninit_fingerprints(an):
    """命中"器件没起来"指纹的 soak 段：全部尝试都没产生真实事务（maxdurUs≈0）。

    两种口径都要认：
      - 旧口径：未初始化时 baro.read() 直接返回 0x0403 且被记成 comp ⇒ comp==att；
      - 新口径（T1 D1）：err != 0x0404 记成 kind=other，而 `[soak.end]` 里没有 other 字段，
        只能由不变量 fail == tx + comp + other 推算 ⇒ other == att。
    共同点是 durUs=0，必须与"事务成功但补偿结果非法"（maxdurUs 是正常事务耗时）区分开。
    """
    hits = []
    for seg in an.segments:
        end = seg.get("end") or {}
        att = end.get("att") or 0
        if att <= 0:
            continue
        tx = end.get("tx") or 0
        comp = end.get("comp") or 0
        fail = end.get("fail") or 0
        other = max(0, fail - tx - comp)          # soak.end 无 other 字段：按不变量推算
        if (comp == att or other == att) and (end.get("maxdur") or 0) <= 1:
            hits.append(seg)
    return hits


def build_warnings(an, version_ok_flag=None):
    """summary 顶部的醒目警告：数据可用性优先于任何结论。

    version_ok_flag 由 summary_text 传入（已知的门禁结果）；直接调用时自己算。
    """
    out = []
    if version_ok_flag is None:
        version_ok_flag = check_version(an)[0]
    state, rec = init_state_from(an)
    if state is False:
        out.append("**后续 soak/inject 数据不可信（器件未初始化）** —— `init` 返回 `{0}`"
                   "（result={1} err={2} chipId={3}）。未初始化时 `baro.read()` 直接返回 "
                   "0x0403(BARO_CALIB_INVALID) 且不动 failStreak ⇒ 每个失败都被记成 kind=comp，"
                   "基线 soak 会呈现 comp==att / tx=0 / failrate≈1.0 —— 这**不是**“与 I2C 无关”的"
                   "证据，而是“器件没起来”。".format(
                       rec.get("_raw", "?"), rec.get("result"), _hx(rec.get("err"), 4),
                       _hx(rec.get("chipid"), 2)))
        boot_ok = [b for b in an.boots if "init=OK" in b]
        if boot_ok:
            out.append("注意：开机行 `{0}` 报 init=OK，但显式 `init` 没确认 —— 以显式结果为准，"
                       "先重发一次 `init` 复核。".format(boot_ok[-1]))
    elif state is None and an.segments:
        out.append("日志里没有 `[init] ... result=OK`：无法确认器件已初始化，下面的"
                   " soak/inject 数字都要先打问号（旧镜像，或流程没跑到 init）。")
    refused = [e for e in an.errs if "not-initialized" in (e.get("_raw") or "")]
    if refused:
        out.append("固件拒绝了 soak/inject：`{0}` —— 器件未就绪，**这几段没有任何数据**"
                   "（不是“没复现”，也不是“没有失败”）。".format(refused[0].get("_raw", "")))
        for seg in an.segments:
            if not seg.get("end"):
                out.append("段 `{0}`（命令 `{1}`）没有 `[soak.end]`：被固件拒绝或没跑到，无数据。".format(
                    seg["label"], seg["cmd"]))
    for seg in uninit_fingerprints(an):
        end = seg.get("end") or {}
        out.append("段 `{0}` 命中未初始化指纹：comp==att={1}、tx={2}、maxdurUs={3} ⇒ "
                   "**器件可能未初始化/没起来，该段结论作废**（这不是“数据被顶掉”）。".format(
                       seg["label"], end.get("att"), end.get("tx"), end.get("maxdur")))
    for seg in an.segments:
        end = seg.get("end") or {}
        if end and end.get("fail") != len(seg["fails"]):
            out.append("段 `{0}`：`[soak.end] fail={1}` 与捕获到的 [fail] 行数 {2} 不一致"
                       "（可能串口丢行，结论按保守方向读）。".format(
                           seg["label"], end.get("fail"), len(seg["fails"])))
    # 门禁通过 ⇒ 镜像就是 r2，而 r2 的每行 [fail] 都该带 scl/sda（代码结构保证）。
    # 此时"缺字段"只可能是串口尾部截断 ⇒ H4 的带电平样本数会静默偏少，必须显式说出来。
    no_lvl = [r for r in an.fails if r.get("scl") is None or r.get("sda") is None]
    if version_ok_flag and no_lvl:
        out.append("**{0}/{1}** 行 `[fail]` 缺少 scl/sda，而本日志已通过 r2 门禁 —— r2 的每行 "
                   "`[fail]` 都应带这两个字段，缺行多半是**串口尾部截断** ⇒ H4 的"
                   "“带电平样本数”会偏少，缺行**不能**当成“该行电平没有拉低”，"
                   "结论按保守方向读。".format(len(no_lvl), len(an.fails)))
    for err in an.errs:
        if "not-initialized" in (err.get("_raw") or ""):
            continue                       # 已在上面单独说明，不重复刷屏
        out.append("固件报错行：`{0}`".format(err.get("_raw", "")))
    return out


def build_conclusions(an, clusters, hist):
    """数据驱动的结论段（不硬编码 PASS/FAIL，只给判读方向）。"""
    out = []
    total = len(an.fails)

    if total == 0:
        end_fails = sum(((s.get("end") or {}).get("fail") or 0) for s in an.segments)
        sig = uninit_fingerprints(an)
        if end_fails:
            out.append("- `[soak.end]` 累计记了 **{0}** 次失败，却一行 `[fail]` 都没捕获到："
                       "要么串口丢行，要么器件根本没起来（读失败的明细没打出来）。"
                       "**不能**据此写“未复现 0x404”。".format(end_fails))
            if sig:
                out.append("  - ⇒ 且命中未初始化指纹（{0}）：按“器件没起来”处理，"
                           "这段不能当基线。".format(", ".join(s["label"] for s in sig)))
        else:
            out.append("- 本次未复现任何 `[fail]`（0x404 未出现）：不能据此否定问题，但可确认"
                       "当前速度/负载组合下没有触发。")
    else:
        longest = max(len(c) for c in clusters) if clusters else 0
        singles = sum(1 for c in clusters if len(c) == 1)
        out.append("- 失败总数 **{0}**，簇 **{1}** 个（单次簇 {2} 个，最长连续 **{3}** 次）。".format(
            total, len(clusters), singles, longest))
        if longest >= 2:
            out.append("  - ⇒ 存在连续失败簇（与“单次孤立扰动”不符）；但连续失败**同样兼容** "
                       "H4（从机拉住 SDA / 信号余量不足 / 器件自身卡住），**不能仅凭聚类断定是 "
                       "AF 未清**——必须回到失败行的 `sr1` AF 位（bit10=0x0400）与 `[recover]` 的 "
                       "`sr1_before`/`sr1_after` 才能定性。")
        else:
            out.append("  - ⇒ 全部是孤立单次失败：不支持“一次 NACK 后连续失败”，"
                       "更像单次总线/时序扰动（看 maxgapUs 是否与失败同窗）。")

    if an.fails:
        af_rows = sum(1 for r in an.fails if (r.get("sr1") or 0) & 0x0400)
        out.append("- AF 直接证据：失败行里 `sr1` AF 位置位 {0}/{1} 行；`[recover]` 记录 {2} 条"
                   "（看 `sr1_before` 是否带 AF、`sr1_after` 是否被清掉）。".format(
                       af_rows, len(an.fails), len(an.recovers)))
        if af_rows == 0:
            out.append("  - ⇒ 失败瞬间 SR1 的 AF 位全都没置位：这些失败**不是** AF/NACK 造成的，"
                       "更像事件超时（TXE/RXNE/BTF）或从机拉死总线。")

    tx = sum(1 for r in an.fails if r.get("kind") == "tx")
    comp = sum(1 for r in an.fails if r.get("kind") == "comp")
    other = sum(1 for r in an.fails if r.get("kind") == "other")
    if total:
        out.append("- 来源分解：kind=tx **{0}** 次（{1}） / kind=comp **{2}** 次（{3}） / "
                   "kind=other **{4}** 次（{5}，不计入 tx/comp 比例）".format(
                       tx, _pct(tx, total), comp, _pct(comp, total), other, _pct(other, total)))
        if tx and comp:
            out.append("  - ⇒ 两类都在：0x404 里既有真 I2C 事务失败，也有“突发读成功但补偿结果非法”，"
                       "整改不能只按 I2C 处理。")
        elif comp:
            out.append("  - ⇒ 全部是 comp：与 I2C 事务无关，问题在读回数据/补偿链路"
                       "（器件数据或换算结果非有限/压力<=0）。")
        elif tx:
            out.append("  - ⇒ 全部是 tx：失败来自 I2C 事务本身（事件超时/NACK），与补偿无关。")
        else:
            out.append("  - ⇒ 全部是 other：都是非 0x404 的错误类别（见 err 列），"
                       "先解决这些，tx/comp 判据本轮不适用。")

    attempts = [r["attempts"] for r in an.injects if r.get("attempts") is not None]
    if attempts:
        first_try = sum(1 for a in attempts if a == 1)
        slow = sum(1 for a in attempts if a > 1)
        never = sum(1 for a in attempts if a < 0)        # 台架约定 -1 = 尝试上限内从未恢复
        out.append("- NACK 注入 **{0}** 次：attempts_to_ok 分布 {1}"
                   "（==1 有 {2} 次，>1 有 {3} 次，<0 有 {4} 次）。".format(
                       len(attempts), _hist(attempts), first_try, slow, never))
        if never:
            out.append("  - ⇒ **有 {0} 次注入 NACK 后在尝试上限内始终没恢复**：这是“AF 未清/粘连”"
                       "最强的台架证据（连 recover 都没立刻救回来）。".format(never))
        if slow:
            out.append("  - ⇒ 有 {0} 次需要 >1 次尝试才恢复：与 AF 未清相容，但**同样兼容 H4**"
                       "（从机拉 SDA/信号余量不足），要结合失败行 `sr1` 的 AF 位与 `[recover]` 的 "
                       "`sr1_before`/`sr1_after` 才能定性。".format(slow))
        if first_try:
            out.append("  - ⇒ 有 {0} 次一次即恢复：`attempts_to_ok==1` **两种情形不可分** ——"
                       "(a) recover 的 PE 关开确实清了 AF，(b) AF 根本没挡住后续事务；"
                       "因此 ==1 不能单独证明“无 AF 粘连”。".format(first_try))
    else:
        out.append("- 未捕获 `[inject]` 行（inject 段没跑或没回显），无法判定 AF 粘连。")

    if an.fails:
        ev = _hx_hist(an.fails, "lastev", 8)
        top_ev = max(ev.items(), key=lambda kv: kv[1])
        out.append("- 失败行 lastEv 分布 {0}；最多的是 `{1}`（{2} 次）——对照驱动里该事件的超时源"
                   "即可定位主要失败路径。".format(ev, top_ev[0], top_ev[1]))

    # 注：段级 fail 计数不一致、未初始化指纹、[err] 行统一放在 summary 顶部的
    # build_warnings() 里（数据可用性问题比结论更该先看到）。
    return out


def summary_text(an, meta, version_ok_flag, ver_line, seg_clusters, hist, all_clusters):
    L = []
    add = L.append

    add("# 气压计 baro 台架自动化摘要")
    add("")
    add("- 时间：{0}".format(meta.get("time", "")))
    add("- 原始日志：`{0}`".format(meta.get("raw_log", "")))
    add("- 命令：`{0}`".format(meta.get("cmdline", "")))
    if meta.get("selftest"):
        add("- 数据来源：**`--selftest` 内嵌合成日志**（未接真板子，数字仅供参考解析逻辑）")
    add("- 版本门禁：**{0}** `{1}`（要求包含 `{2}`）".format(
        "PASS" if version_ok_flag else "FAIL", ver_line, BARO_VER_TOKEN))
    add("")

    # 数据可用性警告放最前面：未初始化/丢行这类问题会让下面所有数字失去意义。
    warn = build_warnings(an, version_ok_flag)
    if warn:
        add("## ⚠ 警告（先看这里）")
        add("")
        for line in warn:
            add("- " + line)
        add("")

    steps = meta.get("steps") or []
    if steps:
        add("## 流程判据（本次实际执行）")
        add("")
        add("| 结果 | 步骤 |")
        add("|---|---|")
        for ok, msg in steps:
            add("| {0} | {1} |".format("PASS" if ok else "**FAIL**",
                                       str(msg).replace("|", "\\|")))
        add("")

    add("## 器件/总线快照")
    add("")
    init_state, init_rec = init_state_from(an)
    if init_rec:
        add("- init：`{0}` ⇒ **{1}**（err={2} chipId={3}）".format(
            init_rec.get("_raw", ""), {True: "OK", False: "FAIL"}.get(init_state, "?"),
            _hx(init_rec.get("err"), 4), _hx(init_rec.get("chipid"), 2)))
    else:
        add("- init：(无 `[init]` 行 —— 无法确认器件已初始化)")
    for boot in an.boots:
        add("- boot：`{0}`".format(boot))
    if an.cfg:
        add("- cfg：`{0}`".format(an.cfg.get("_raw", "")))
    else:
        add("- cfg：(无)")
    if an.sr:
        add("- sr：`{0}`".format(an.sr.get("_raw", "")))
    else:
        add("- sr：(无)")
    if an.bus:
        add("- bus：`{0}`".format(an.bus.get("_raw", "")))
    else:
        add("- bus：(无)")
    add("")

    add("## soak 段（按时序；speed 由段前的 `speed` 命令推得，boot = 上电默认）")
    add("")
    if not an.segments:
        add("(无 soak 段)")
    else:
        add("| 段 | 命令 | att | new | notready | fail | tx | comp | other* | failrate | "
            "maxgapUs | maxdurUs | evTimeouts | recovered |")
        add("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
        for seg in an.segments:
            end = seg.get("end")
            if end:
                other = max(0, (end.get("fail") or 0) - (end.get("tx") or 0) - (end.get("comp") or 0))
                add("| {0} | `{1}` | {2} | {3} | {4} | {5} | {6} | {7} | {8} | {9} | {10} | {11} | {12} | {13} |".format(
                    seg["label"], seg["cmd"], end.get("att"), end.get("new"), end.get("notready"),
                    end.get("fail"), end.get("tx"), end.get("comp"), other, end.get("failrate"),
                    end.get("maxgap"), end.get("maxdur"), end.get("evtimeouts"), end.get("recovered")))
            else:
                add("| {0} | `{1}` | - | - | - | - | - | - | - | - | - | - | - | - |".format(
                    seg["label"], seg["cmd"]))
        add("")
        add("- `other* = fail - tx - comp`：`[soak.end]` 契约里没有 other 字段（T1 D1），按 "
            "`fail == tx + comp + other` 不变量推算；逐行统计仍以捕获到的 `[fail]` 行为准。")
        add("")
        add("| 段 | 1s tick 数 | tick 末 t(ms) | 该段捕获的 [fail] 行 |")
        add("|---|---|---|---|")
        for seg in an.segments:
            ticks = seg["ticks"]
            add("| {0} | {1} | {2} | {3} |".format(
                seg["label"], len(ticks), ticks[-1].get("t") if ticks else "-", len(seg["fails"])))
    add("")

    add("## 失败聚类（单次偶发 vs 一次 NACK 后连续失败）")
    add("")
    total = len(an.fails)
    add("- 失败总数：{0}；簇：{1} 个".format(total, len(all_clusters)))
    add("- 连续失败长度直方图（簇长度 -> 簇数）：{0}".format(hist))
    if all_clusters:
        add("")
        add("| 段 | 簇长度序列 | 最长 |")
        add("|---|---|---|")
        for label, clusters in seg_clusters.items():
            add("| {0} | {1} | {2} |".format(
                label, [len(c) for c in clusters], max(len(c) for c in clusters)))
        add("")
        add("| 段 | 簇# | 长度 | t 范围 | kind 序列 |")
        add("|---|---|---|---|---|")
        for label, clusters in seg_clusters.items():
            for idx, cluster in enumerate(clusters, 1):
                add("| {0} | {1} | {2} | {3}..{4} | {5} |".format(
                    label, idx, len(cluster), cluster[0].get("t"), cluster[-1].get("t"),
                    "/".join(r.get("kind", "?") for r in cluster)))
    add("")

    add("## inject：注入 NACK 后恢复到正常读需要几次尝试（AF 粘连判据）")
    add("")
    attempts = [r["attempts"] for r in an.injects if r.get("attempts") is not None]
    if attempts:
        first_try = sum(1 for a in attempts if a == 1)
        add("| attempts_to_ok | 次数 | 占比 |")
        add("|---|---|---|")
        for value, count in _hist(attempts).items():
            add("| {0} | {1} | {2} |".format(value, count, _pct(count, len(attempts))))
        add("")
        add("- 一次即恢复（attempts_to_ok==1）：**{0}/{1}**（{2}）".format(
            first_try, len(attempts), _pct(first_try, len(attempts))))
        add("- nack_ok 分布：{0}；post_af 分布：{1}".format(
            _hist([r.get("nack_ok") for r in an.injects]),
            _hist([r.get("post_af") for r in an.injects])))
        add("- 逐条：")
        for rec in an.injects:
            add("  - i={0} pre_sr1={1} af={2} nack_ok={3} post_sr1={4} post_af={5} next_ok={6} "
                "attempts_to_ok={7}".format(
                    rec.get("i"), _hx(rec.get("pre_sr1"), 4), rec.get("af"), rec.get("nack_ok"),
                    _hx(rec.get("post_sr1"), 4), rec.get("post_af"), rec.get("next_ok"),
                    rec.get("attempts")))
    else:
        add("(无 [inject] 行)")
    add("")

    add("## 失败来源 kind（tx=I2C 事务失败 / comp=补偿结果非法）")
    add("")
    if total:
        tx = sum(1 for r in an.fails if r.get("kind") == "tx")
        comp = sum(1 for r in an.fails if r.get("kind") == "comp")
        add("| kind | 次数 | 占全部失败 |")
        add("|---|---|---|")
        for kind, count in _hist([r.get("kind") for r in an.fails]).items():
            add("| {0} | {1} | {2} |".format(kind, count, _pct(count, total)))
        add("")
        other = sum(1 for r in an.fails if r.get("kind") == "other")
        add("- tx:comp（**不含 kind=other**）= **{0}:{1}**".format(tx, comp))
        if other:
            add("- kind=other **单列 {0} 次**，不计入 tx/comp 比例；err 分布 {1}"
                "（只有 err==0x0404 才分 tx/comp，其它 err 属于别的故障类别）。".format(
                    other, _hx_hist([r for r in an.fails if r.get("kind") == "other"], "err", 4)))
    else:
        add("(无 [fail] 行)")
    add("")

    add("## 失败行 sr1 / sr2 / lastEv / status 分布（哪个事件超时最多）")
    add("")
    if total:
        add("| 字段 | 值 | 次数 |")
        add("|---|---|---|")
        for name, width in (("sr1", 4), ("sr2", 4), ("lastev", 8), ("status", 2)):
            for value, count in _hx_hist(an.fails, name, width).items():
                add("| {0} | `{1}` | {2} |".format(
                    {"lastev": "lastEv"}.get(name, name), value, count))
        add("")
        add("- durUs：max {0}；gapUs：max {1}；streak：max {2}".format(
            max((r.get("dur", 0) or 0) for r in an.fails),
            max((r.get("gap", 0) or 0) for r in an.fails),
            max((r.get("streak", 0) or 0) for r in an.fails)))
        add("- 全部失败行见 `fails.csv`。")
    else:
        add("(无 [fail] 行)")
    add("")

    add("## 失败瞬间 PB6(SCL)/PB7(SDA) 电平分布（H4 总线被拉住）")
    add("")
    lvl = [r for r in an.fails if r.get("scl") is not None and r.get("sda") is not None]
    miss = len(an.fails) - len(lvl)
    # 门禁通过 ⇒ 镜像必是 r2，而 r2 每行 [fail] 都带 scl/sda：缺字段只能是尾部截断。
    # 不能把这种情况写成"旧固件"，否则会把"丢行"误读成"该行电平没拉低"。
    miss_reason = ("**疑似串口尾部截断**（r2 的每行 [fail] 都应带 scl/sda）"
                   if (version_ok_flag and miss) else "旧固件或该输出未启用")
    if lvl:
        combos = Counter((r.get("scl"), r.get("sda")) for r in lvl)
        add("| (scl,sda) | 次数 | 占带电平样本 |")
        add("|---|---|---|")
        for (s, d), cnt in sorted(combos.items()):
            add("| ({0},{1}) | {2} | {3} |".format(s, d, cnt, _pct(cnt, len(lvl))))
        add("")
        held = sum(1 for r in lvl if r.get("scl") == 0 or r.get("sda") == 0)
        both_low = sum(1 for r in lvl if r.get("scl") == 0 and r.get("sda") == 0)
        add("- 带电平样本 **{0}/{1}** 行（其余 {2} 行没有 scl/sda 字段：{3}）。".format(
            len(lvl), len(an.fails), miss, miss_reason))
        add("- 其中 scl=0 或 sda=0 共 **{0}** 行（{1}），两条同时为 0 的 {2} 行。".format(
            held, _pct(held, len(lvl)), both_low))
        add("- 判读提示：失败瞬间**反复**出现 scl=0 或 sda=0 ⇒ 总线可能被从机拉住（H4）；"
            "单次出现不足以定性，需结合 `[recover]` 的 `next_ok`（恢复后是否立刻能通）一起看。")
    else:
        add("(本次 `[fail]` 行都没有 scl/sda 字段 —— {0})".format(miss_reason))
    add("")

    add("## 其它记录")
    add("")
    if an.speeds:
        add("- speed：" + "；".join(
            "hz={0} result={1}".format(r.get("hz"), r.get("result")) for r in an.speeds))
    if an.stress:
        add("- stress：" + "；".join(
            "state={0}(spi={1},i2c2={2},adc={3})".format(
                r.get("state"), r.get("spi"), r.get("i2c2"), r.get("adc")) for r in an.stress))
    if an.irqblocks:
        add("- irqblock：" + "；".join(
            "us={0} periodMs={1} state={2}".format(r.get("us"), r.get("period"), r.get("state"))
            for r in an.irqblocks))
    if an.inits:
        add("- init：" + "；".join(
            "speed={0} stage={1} result={2} err={3} chipId={4} attempts={5}".format(
                r.get("speed"), r.get("stage"), r.get("result"), _hx(r.get("err"), 4),
                _hx(r.get("chipid"), 2), r.get("attempts")) for r in an.inits))
    if an.recovers:
        for rec in an.recovers:
            add("- recover：sr1_before={0} sr2_before={1} sr1_after={2} sr2_after={3} next_ok={4}".format(
                _hx(rec.get("sr1b"), 4), _hx(rec.get("sr2b"), 4), _hx(rec.get("sr1a"), 4),
                _hx(rec.get("sr2a"), 4), rec.get("next_ok")))
    if an.stats:
        add("- stat（最后一次）：`{0}`".format(an.stats[-1].get("_raw", "")))
    if an.errs:
        add("- [err] 行：" + "；".join("`{0}`".format(r.get("_raw", "")) for r in an.errs))
    if not any((an.speeds, an.stress, an.irqblocks, an.inits, an.recovers, an.stats, an.errs)):
        add("(无)")
    add("")

    add("## 脚本实际发出的命令序列（完整清单）")
    add("")
    if an.commands:
        for cmd in an.commands:
            add("- `{0}`".format(cmd))
    else:
        add("(无命令回声)")
    add("")

    add("## 判读结论")
    add("")
    for line in build_conclusions(an, all_clusters, hist):
        add(line)
    add("")

    add("## 前置条件与口径")
    add("")
    add("- 固件：`test/bench/baro_bench_main.cpp`，版本行 `{0}`；USART1 PA9/PA10 115200 8N1。".format(
        BARO_VER_LINE))
    add("- 版本不匹配时脚本**不跑流程**并退出码 3，旧镜像数据作废。")
    add("- 版本口径：只认 **r3**。r2 的 `soak <ms>` 在 `ms > 25566` 时因 DWT 25.566s 回绕"
        "（2^32 / 168MHz）使 `[soak] t` 归零、**永不自己结束**，该配置下的 soak 数据不可用"
        "（r2 后被判为旧镜像正是为此）；r3 改成逐轮累加（uint64），`[soak] t` 单调不减 —— "
        "这就是 r3 存在的理由。")
    add("- `[fail]` 行的 `seg` 由 raw.log 里 `>>> soak ...` 命令回声切分；段外失败记为 `(post-soak)`。")
    add("- 聚类主判据是固件 `streak`（==1 即新簇）；streak 缺失时用相邻失败 t 间隙 > {0}us。".format(
        CLUSTER_GAP_US))
    add("")

    return "\n".join(L) + "\n"


def write_csv(path, columns, rows):
    with open(path, "w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_outputs(lines, out_dir, meta):
    """解析 + 汇总 + 落盘（summary.md / fails.csv / inject.csv）；--selftest 复用同一条路径。"""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    an = analyze(lines)
    version_ok_flag, ver_line = check_version(an)
    seg_clusters = fail_clusters_by_segment(an)
    all_clusters = [c for clusters in seg_clusters.values() for c in clusters]
    hist = _hist([len(c) for c in all_clusters])

    text = summary_text(an, meta, version_ok_flag, ver_line, seg_clusters, hist, all_clusters)
    fails_rows = fails_csv_rows(an)
    inject_rows = injects_csv_rows(an)

    summary_path = out_dir / "summary.md"
    summary_path.write_text(text, encoding="utf-8")
    write_csv(out_dir / "fails.csv", FAIL_COLS, fails_rows)
    write_csv(out_dir / "inject.csv", INJECT_COLS, inject_rows)

    return {
        "analysis": an,
        "version_ok": version_ok_flag,
        "ver_line": ver_line,
        "summary": text,
        "summary_path": summary_path,
        "fails_rows": fails_rows,
        "inject_rows": inject_rows,
        "hist": hist,
        "clusters": all_clusters,
        "seg_clusters": seg_clusters,
    }


# --------------------------------------------------------------------------------------------------
# 串口层（风格对齐 run_p0.py）
# --------------------------------------------------------------------------------------------------
class Bench:
    """baro bench 串口控制台：读线程把每行落 raw.log，同时进 self.lines 供离线解析复用。"""

    def __init__(self, port, baud, log_path=None):
        # write_timeout 必须有：本机蓝牙虚拟口能 open 但 write() 会一直阻塞（无 write_timeout
        # 时脚本会永久卡死），探测阶段就靠它放弃死口。
        self.ser = serial.Serial(port, baud, timeout=0.05, write_timeout=1.0)
        self.log_path = None
        self._logf = None
        self.q = queue.Queue()
        self.lines = []          # 逻辑行：设备输出 + `>>> cmd` 命令回声
        self._stop = False
        threading.Thread(target=self._reader, daemon=True).start()
        if log_path is not None:
            self.attach_log(log_path)

    def attach_log(self, log_path):
        """挂上 raw.log。探测阶段先不挂：开不了口就不该留空目录和半截日志。"""
        self.log_path = Path(log_path)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._logf = open(self.log_path, "a", encoding="utf-8", errors="replace")

    def _log_write(self, text):
        if self._logf is not None:
            self._logf.write(text)
            self._logf.flush()

    def _reader(self):
        buf = b""
        while not self._stop:
            try:
                chunk = self.ser.read(1024)
            except Exception:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode("utf-8", errors="replace").rstrip("\r")
                stamp = _dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                self._log_write("[{0}] {1}\n".format(stamp, text))
                self.q.put(text)
                self.lines.append(text)

    def close(self):
        self._stop = True
        try:
            self.ser.close()
        finally:
            if self._logf is not None:
                self._logf.close()

    def send(self, text):
        # raw.log 里命令回声写成 run_p0.py 的 `        >>> cmd` 风格；self.lines 里去掉缩进，
        # 因为 analyze() 用它切 soak 段。
        self._log_write("        >>> {0}\n".format(text))
        self.lines.append(">>> {0}".format(text))
        self.ser.write((text + "\r\n").encode("ascii", errors="replace"))
        self.ser.flush()

    def expect(self, pattern, timeout):
        """等到第一行匹配 pattern，返回该行；超时返回 ""。"""
        rx = re.compile(pattern)
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                line = self.q.get(timeout=min(0.2, max(0.01, deadline - time.time())))
            except queue.Empty:
                continue
            if rx.search(line):
                return line
        return ""

    def wait_lines(self, pattern, count, timeout):
        """收集匹配行直到够 count 条或超时，返回已收列表。"""
        rx = re.compile(pattern)
        got = []
        deadline = time.time() + timeout
        while len(got) < count and time.time() < deadline:
            try:
                line = self.q.get(timeout=min(0.2, max(0.01, deadline - time.time())))
            except queue.Empty:
                continue
            if rx.search(line):
                got.append(line)
        return got

    def drain(self, seconds=0.3):
        deadline = time.time() + seconds
        while time.time() < deadline:
            try:
                self.q.get(timeout=0.05)
            except queue.Empty:
                pass


def is_bluetooth(port):
    """判断是否是蓝牙虚拟口。

    不能只匹配英文 description：中文 Windows 上蓝牙口的 description 是“蓝牙链接上的标准串行
    (COM8)”，而这种口能 open、write 却会永久阻塞，必须主动跳过。hwid 里的 BTHENUM 是兜底判据。
    """
    text = "{0} {1} {2}".format(port.device or "", port.description or "",
                                getattr(port, "hwid", "") or "")
    low = text.lower()
    return ("bluetooth" in low) or ("蓝牙" in text) or ("bthenum" in low)


def _probe_port(port, baud, box):
    """在一个可被放弃的线程里探测单个串口（探测结果写进 box）；探测不落 raw.log。"""
    try:
        bench = Bench(port.device, baud)
    except Exception as exc:
        box["error"] = repr(exc)
        return
    try:
        bench.send("help")
        if bench.expect(r"ok>|\[ver\]|\bhelp\b", 2.0):
            box["bench"] = bench
            return
    except Exception as exc:
        box["error"] = repr(exc)
    bench.close()


def open_auto(baud, probe_timeout=8.0):
    """自动找口：跳过 description 含 bluetooth/蓝牙、hwid 含 BTHENUM 的口；单口探测有硬超时。"""
    for port in list_ports.comports():
        if is_bluetooth(port):
            print("[auto] 跳过 {0}（{1}）：蓝牙虚拟口".format(port.device, port.description))
            continue
        box = {}
        th = threading.Thread(target=_probe_port, args=(port, baud, box), daemon=True)
        th.start()
        th.join(probe_timeout)
        if th.is_alive():
            print("[auto] 跳过 {0}：探测超过 {1:.0f}s（端口卡死，放弃）".format(port.device, probe_timeout))
            continue
        if box.get("bench"):
            print("[auto] baro bench 控制台: {0} ({1})".format(port.device, port.description))
            return box["bench"]
        if box.get("error"):
            print("[auto] 跳过 {0}：{1}".format(port.device, box["error"]))
    raise RuntimeError("没有找到响应 help 的 bench 控制台串口，请用 --port 指定")


# --------------------------------------------------------------------------------------------------
# 默认实验流程
# --------------------------------------------------------------------------------------------------
class Flow:
    def __init__(self, quiet=False):
        self.steps = []
        self.failed = 0
        self.quiet = quiet          # --selftest 里跑假控制台时静音，保持验收输出干净

    def note(self, ok, msg):
        self.steps.append((bool(ok), msg))
        if not ok:
            self.failed += 1
        if not self.quiet:
            print(("[PASS] " if ok else "[FAIL] ") + msg, flush=True)


def do_soak(b, ms, flow, tag):
    """发 soak 并等 [soak.end]（soak 期间固件每 1s 打一行 [soak]）。

    同时盯着 `[err] not-initialized`：新固件在器件未就绪时会**拒绝执行** soak，只回这一行；
    这时要立刻判失败，不能白等满整个 soak（默认 60s + 15s 余量）。
    """
    b.send("soak {0}".format(ms))
    deadline = time.time() + ms / 1000.0 + 15.0
    got = ""
    while time.time() < deadline:
        line = b.expect(r"\[soak\.end\]|\[err\]\s*not-initialized",
                        min(1.0, max(0.05, deadline - time.time())))
        if line:
            got = line
            break
    ok = "[soak.end]" in got
    flow.note(ok, "{0}: soak {1}ms -> {2}".format(tag, ms, got or "(无 [soak.end])"))
    return ok


def do_speed(b, hz, flow):
    b.send("speed {0}".format(hz))
    got = b.expect(r"\[speed\]\s+hz=\d+\s+result=\w+", 8.0)
    ok = bool(got) and ("hz={0}".format(hz) in got) and ("result=OK" in got)
    flow.note(ok, "speed {0} -> {1}".format(hz, got or "(无 [speed] 行)"))
    return ok


def do_init(b, flow, hz=None, timeout=5.0):
    """发 init 并等到 result=OK。

    必须放在第一个 soak/inject 之前：台架固件的**开机路径不初始化器件**，未初始化时
    baro.read() 直接返回 0x0403(BARO_CALIB_INVALID) 且不动 failStreak，于是每个失败都会被记成
    kind=comp —— 那正好是"与 I2C 无关"的假签名，会把结论带反；inject 的 attempts_to_ok 也会恒为 1。
    契约是每个 stage 一行 [init]（stage=reset|chipid|calib|write|readback），失败可能打多行，
    所以这里循环收 [init]：见到 result=OK 即成功；已收到 FAIL 行后只再等 1.5s，避免白等一整轮。
    """
    b.send("init" if hz is None else "init {0}".format(hz))
    deadline = time.time() + timeout
    last = ""
    while time.time() < deadline:
        wait = (deadline - time.time()) if not last else min(1.5, max(0.1, deadline - time.time()))
        got = b.expect(r"\[init\]", wait)
        if not got:
            break
        last = got
        if "result=OK" in got:
            flow.note(True, "init -> {0}".format(got))
            return got
    flow.note(False, "init -> {0}（未拿到 result=OK：后续 soak/inject 数据不可信！）".format(
        last or "(无 [init] 行)"))
    return None


def run_flow(b, args, flow):
    """默认流程；返回 {"version_ok": bool, "init_ok": bool|None}（步骤判据在 flow.steps 里）。"""
    # 0) 控制台 / 版本门禁（版本不对就不浪费 60s soak）
    b.send("help")
    if not b.expect(r"ok>|\[ver\]|\bhelp\b", 3.0):
        flow.note(False, "控制台无响应（help 没有回显 ok>）")
        return {"version_ok": False, "init_ok": None}
    b.send("ver")
    b.expect(r"\[ver\]", 2.0)
    ver_lines = [l for l in b.lines if "[ver]" in l]
    ver = ver_lines[-1].strip() if ver_lines else "(无 [ver] 行)"
    if BARO_VER_TOKEN not in ver:
        flow.note(False, "固件版本不匹配：{0}（需要 {1}；旧镜像数据作废，请重烧 baro bench）".format(
            ver, BARO_VER_LINE))
        return {"version_ok": False, "init_ok": None}
    flow.note(True, "固件版本：{0}".format(ver))

    # 1) 静态快照 + 复位
    for cmd, pattern, timeout in (("cfg", r"\[cfg\]", 3.0),
                                  ("sr", r"\[sr\]", 3.0),
                                  ("bus", r"\[bus\]", 3.0),
                                  ("reset", r"\[reset\]\s+ok", 5.0)):
        b.send(cmd)
        got = b.expect(pattern, timeout)
        flow.note(bool(got), "{0} -> {1}".format(cmd, got or "(无回显)"))

    # 1b) 器件初始化：硬要求，必须在第一个 soak/inject 之前（见 do_init 注释）
    init_line = do_init(b, flow)
    init_ok = bool(init_line and "result=OK" in init_line)

    # 2) 主 soak @默认速度（上电 400k）
    do_soak(b, args.soak, flow, "soak@{0}Hz".format(args.speed))

    # 3) NACK 注入（AF 粘连判据）；未初始化时固件会回 [err] not-initialized 并拒绝执行
    b.send("inject {0}".format(args.inject))
    got = []
    refused = ""
    deadline = time.time() + max(10.0, args.inject * 3.0 + 5.0)
    while len(got) < args.inject and time.time() < deadline:
        line = b.expect(r"\[inject\]|\[err\]\s*not-initialized",
                        min(1.0, max(0.05, deadline - time.time())))
        if not line:
            continue
        if "[inject]" in line:
            got.append(line)
        else:
            refused = line
            break
    flow.note(len(got) >= args.inject,
              "inject {0} -> 收到 {1} 行 [inject]{2}".format(
                  args.inject, len(got),
                  "（被拒绝：{0}）".format(refused) if refused else ""))

    # 3b) 注入后立刻 recover：只有这时 SR1 才可能还带着 AF=1，sr1_before 才有判读价值
    #     （收尾那次 recover 在 stress 之后，SR1 大概率已经干净，采不到 AF）
    b.send("recover")
    got = b.expect(r"\[recover\]", 8.0)
    flow.note(bool(got), "recover@post-inject -> {0}".format(got or "(无回显)"))

    # 4) 变速对照
    if do_speed(b, LOW_SPEED, flow):
        do_soak(b, args.side_soak, flow, "soak@{0}Hz".format(LOW_SPEED))
    if do_speed(b, args.speed, flow):
        do_soak(b, args.side_soak, flow, "soak@{0}Hz".format(args.speed))

    # 5) 并发负载（可选：默认跑，--no-stress 跳过）
    if not args.no_stress:
        b.send("stress on")
        got = b.expect(r"\[stress\]\s+state=\w+", 5.0)
        flow.note(bool(got) and "state=on" in got, "stress on -> {0}".format(got or "(无回显)"))
        do_soak(b, args.side_soak, flow, "soak@stress")
        b.send("stress off")
        got = b.expect(r"\[stress\]\s+state=\w+", 5.0)
        flow.note(bool(got) and "state=off" in got, "stress off -> {0}".format(got or "(无回显)"))
        flow.note(True, "stress 段结束（--no-stress 可跳过该段）")
    else:
        flow.note(True, "stress 段被 --no-stress 跳过")

    # 6) 收尾：recover + stat
    b.send("recover")
    got = b.expect(r"\[recover\]", 8.0)
    flow.note(bool(got), "recover -> {0}".format(got or "(无回显)"))
    b.send("stat")
    got = b.expect(r"\[stat\]", 5.0)
    flow.note(bool(got), "stat -> {0}".format(got or "(无回显)"))
    return {"version_ok": True, "init_ok": init_ok}


# --------------------------------------------------------------------------------------------------
# --selftest：内嵌合成日志 + 假控制台
#   覆盖: 正常 soak / tx+comp 失败 / inject(1 与 5 与 -1) / 版本不匹配 /
#         未初始化签名(comp==att + maxdurUs=0) / init=FAIL / kind=other / [boot] 行 /
#         默认流程命令序列(含 init 与 recover@post-inject)
# --------------------------------------------------------------------------------------------------
_SYN_NORMAL = """\
>>> ver
[ver] baro bench r3 2026-10-18
>>> cfg
[cfg] i2c=I2C1 scl=PB6 sda=PB7 addr=0x76 speed=400000 osr=0x03 odr=0x02 cfg=0x00 int=0x00 pwr=0x33
>>> sr
[sr] sr1=0x0000 sr2=0x0000 af=0 berr=0 arlo=0 busy=0 msL=0 tra=0 txE=1 rxne=0 btf=0 ccr=0 trise=0 ackCfg=0
>>> bus
[bus] scl=1 sda=1 cpu=168 cycPerUs=168
>>> reset
[reset] ok
>>> soak 60000
[soak] t=1000 att=125 new=125 notready=0 fail=0 tx=0 comp=0 maxgapUs=8100 maxdurUs=260
[soak] t=2000 att=250 new=250 notready=0 fail=0 tx=0 comp=0 maxgapUs=8100 maxdurUs=265
[soak.end] t=60000 att=7500 new=7500 notready=0 fail=0 tx=0 comp=0 failrate=0.0e+00 maxgapUs=8100 maxdurUs=270 evTimeouts=0 recovered=0
"""

_SYN_FAIL = """\
>>> speed 100000
[speed] hz=100000 result=OK
>>> soak 20000
[fail] i=1 t=1500 kind=tx sr1=0x0400 sr2=0x0000 lastEv=0x00000100 evD=1 status=0x02 raw=AA55AA55AA55 durUs=260 gapUs=8100 streak=1
[fail] i=2 t=1516 kind=tx sr1=0x0400 sr2=0x0000 lastEv=0x00000100 evD=2 status=0x02 raw=AA55AA55AA55 durUs=258 gapUs=8000 streak=2
[fail] i=3 t=1532 kind=tx sr1=0x0400 sr2=0x0000 lastEv=0x00000100 evD=3 status=0x02 raw=AA55AA55AA55 durUs=255 gapUs=8000 streak=3
[fail] i=4 t=9000 kind=comp sr1=0x0000 sr2=0x0000 lastEv=0x00000000 evD=0 status=0x00 raw=000000000000 durUs=300 gapUs=8000 streak=1
[fail] i=5 t=18000 kind=comp sr1=0x0000 sr2=0x0000 lastEv=0x00000000 evD=0 status=0x00 raw=000000000000 durUs=305 gapUs=8000 streak=1
[soak.end] t=20000 att=2500 new=2495 notready=0 fail=5 tx=3 comp=2 failrate=2.0e-03 maxgapUs=18000 maxdurUs=305 evTimeouts=3 recovered=1
"""

_SYN_INJECT = """\
>>> inject 6
[inject] i=1 pre_sr1=0x0000 af=0 nack_ok=1 post_sr1=0x0400 post_af=1 next_ok=1 attempts_to_ok=1
[inject] i=2 pre_sr1=0x0000 af=0 nack_ok=1 post_sr1=0x0400 post_af=1 next_ok=1 attempts_to_ok=1
[inject] i=3 pre_sr1=0x0000 af=0 nack_ok=1 post_sr1=0x0400 post_af=0 next_ok=1 attempts_to_ok=5
[inject] i=4 pre_sr1=0x0000 af=0 nack_ok=1 post_sr1=0x0400 post_af=1 next_ok=1 attempts_to_ok=1
[inject] i=5 pre_sr1=0x0000 af=0 nack_ok=1 post_sr1=0x0400 post_af=0 next_ok=1 attempts_to_ok=5
[inject] i=6 pre_sr1=0x0000 af=0 nack_ok=1 post_sr1=0x0400 post_af=1 next_ok=1 attempts_to_ok=1
>>> stat
[stat] att=100000 new=99990 notready=0 fail=5 tx=3 comp=2 evTimeouts=3 recovered=1 maxgapUs=20000 maxdurUs=400
"""

_SYN_OLD_VERSION = """\
>>> ver
[ver] baro bench r2 2026-10-18
"""

# 器件没起来：read() 不做任何 I2C 事务就返回 0x0403 ⇒ 每个失败都记成 comp、durUs=0。
_SYN_NOT_INITIALIZED = """\
>>> ver
[ver] baro bench r3 2026-10-18
>>> soak 60000
[soak.end] t=60000 att=7500 new=0 notready=7500 fail=7500 tx=0 comp=7500 failrate=1.0e+00 maxgapUs=0 maxdurUs=0 evTimeouts=0 recovered=0
"""

# init 失败 + 随后的未初始化签名：summary 顶部必须出现"数据不可信"。
_SYN_INIT_FAIL = """\
>>> ver
[ver] baro bench r3 2026-10-18
>>> init
[init] speed=400000 stage=calib result=FAIL err=0x0403 chipId=0x50 attempts=3
>>> soak 60000
[soak.end] t=60000 att=7500 new=0 notready=7500 fail=7500 tx=0 comp=7500 failrate=1.0e+00 maxgapUs=0 maxdurUs=0 evTimeouts=0 recovered=0
"""

# 新契约：kind=other（带 err=）必须单列、不进 tx/comp 比例；[boot] 行不能被当成 [init]。
_SYN_OTHER_KIND = """\
>>> ver
[ver] baro bench r3 2026-10-18
[boot] baro init=OK
>>> init
[init] speed=400000 stage=readback result=OK err=0x0000 chipId=0x50 attempts=1
>>> soak 20000
[fail] i=1 t=1000 kind=tx sr1=0x0400 sr2=0x0000 lastEv=0x00000100 evD=1 status=0x02 raw=AA55AA55AA55 durUs=260 gapUs=8100 streak=1
[fail] i=2 t=2000 kind=other err=0x0403 sr1=0x0000 sr2=0x0000 lastEv=0x00000000 evD=0 status=0x00 raw=000000000000 durUs=0 gapUs=8000 streak=1
[fail] i=3 t=3000 kind=comp sr1=0x0000 sr2=0x0000 lastEv=0x00000000 evD=0 status=0x00 raw=000000000000 durUs=300 gapUs=8000 streak=1
[soak.end] t=20000 att=2500 new=2495 notready=0 fail=3 tx=1 comp=1 failrate=1.2e-03 maxgapUs=8000 maxdurUs=300 evTimeouts=1 recovered=1
"""

# attempts_to_ok = -1（尝试上限内从未恢复）才是 AF 粘连的强证据，必须与 >1 分开写。
_SYN_INJECT_NEVER = """\
>>> ver
[ver] baro bench r3 2026-10-18
>>> inject 3
[inject] i=1 pre_sr1=0x0000 af=0 nack_ok=1 post_sr1=0x0400 post_af=1 next_ok=1 attempts_to_ok=1
[inject] i=2 pre_sr1=0x0000 af=0 nack_ok=1 post_sr1=0x0400 post_af=1 next_ok=0 attempts_to_ok=-1
[inject] i=3 pre_sr1=0x0400 af=1 nack_ok=1 post_sr1=0x0400 post_af=1 next_ok=0 attempts_to_ok=-1
"""

# 新固件：器件未就绪时 soak/inject 被拒绝，只回 [err] not-initialized（不是 bad-arg）。
_SYN_REFUSED = """\
>>> ver
[ver] baro bench r3 2026-10-18
>>> soak 60000
[err] not-initialized
>>> inject 20
[err] not-initialized
>>> stat
[stat] att=0 new=0 notready=0 fail=0 tx=0 comp=0 evTimeouts=0 recovered=0 maxgapUs=0 maxdurUs=0
"""

# D1：`[fail]` 行尾追加 err=0x%04X，且 err != 0x0404 记成 kind=other（soak.end 里 comp=0）
_SYN_TRAILING_ERR = """\
>>> ver
[ver] baro bench r3 2026-10-18
>>> soak 20000
[fail] i=1 t=1000 kind=other sr1=0x0000 sr2=0x0000 lastEv=0x00000000 evD=0 status=0x00 raw=000000000000 durUs=0 gapUs=8000 streak=1 err=0x0403
[soak.end] t=20000 att=2500 new=0 notready=2500 fail=2500 tx=0 comp=0 failrate=1.0e+00 maxgapUs=0 maxdurUs=0 evTimeouts=0 recovered=0
"""

# A21: r2 的 [fail] 行尾追加 scl=%u sda=%u（失败瞬间 PB6/PB7 电平），用于 H4"总线被拉住"自动判读。
# 5 行样本的 (scl,sda) 故意取成 (0,0)x2 / (0,1) / (1,0) / (1,1)，让四个派生计数两两不等：
#   held     = scl==0 or  sda==0 → (0,0),(0,0),(0,1),(1,0) = 4
#   both_low = scl==0 and sda==0 → (0,0),(0,0)             = 2
#   count(scl==0) = 3 ; count(sda==0) = 3
# 所以把 `or` 改 `and`(→2)、只算 scl(→3)、只算 sda(→3)、both_low 放宽(→3) 都会被 F8 抓住。
# （早先 3 行样本里 held=2 与"只算 scl"=2 撞值，verifier 的 held 变异才漏网。）
_SYN_LEVELS = """\
>>> ver
[ver] baro bench r3 2026-10-18
>>> soak 20000
[fail] i=1 t=1000 kind=tx sr1=0x0400 sr2=0x0000 lastEv=0x00000100 evD=1 status=0x02 raw=AA55AA55AA55 durUs=260 gapUs=8100 streak=1 err=0x0404 scl=0 sda=0
[fail] i=2 t=2000 kind=tx sr1=0x0400 sr2=0x0000 lastEv=0x00000100 evD=2 status=0x02 raw=AA55AA55AA55 durUs=255 gapUs=8000 streak=2 err=0x0404 scl=0 sda=0
[fail] i=3 t=3000 kind=other err=0x0403 sr1=0x0000 sr2=0x0000 lastEv=0x00000000 evD=0 status=0x00 raw=000000000000 durUs=0 gapUs=8000 streak=1 scl=0 sda=1
[fail] i=4 t=4000 kind=tx sr1=0x0400 sr2=0x0000 lastEv=0x00000100 evD=4 status=0x02 raw=AA55AA55AA55 durUs=258 gapUs=8000 streak=1 err=0x0404 scl=1 sda=0
[fail] i=5 t=5000 kind=comp sr1=0x0000 sr2=0x0000 lastEv=0x00000000 evD=0 status=0x00 raw=000000000000 durUs=300 gapUs=8000 streak=1 err=0x0404 scl=1 sda=1
[soak.end] t=20000 att=2500 new=2495 notready=0 fail=5 tx=3 comp=1 failrate=2.0e-03 maxgapUs=8000 maxdurUs=300 evTimeouts=3 recovered=1
"""

# r2 日志里被**串口尾部截断**的 [fail] 行：9 个必填字段与 err= 都在，只剩尾部 scl/sda 丢了。
# 这行仍会被解析（scl/sda 是可选的），但 H4 的"带电平样本数"会静默偏少 ⇒ 必须给出截断警告，
# 而不是把它写成"旧固件"（门禁通过就意味着镜像必是 r2）。
_SYN_TRUNCATED_TAIL = """\
>>> ver
[ver] baro bench r3 2026-10-18
>>> soak 20000
[fail] i=1 t=1000 kind=tx sr1=0x0400 sr2=0x0000 lastEv=0x00000100 evD=1 status=0x02 raw=AA55AA55AA55 durUs=260 gapUs=8100 streak=1 err=0x0404 scl=0 sda=0
[fail] i=2 t=2000 kind=tx sr1=0x0400 sr2=0x0000 lastEv=0x00000100 evD=2 status=0x02 raw=AA55AA55AA55 durUs=255 gapUs=8000 streak=2 err=0x0404
[soak.end] t=20000 att=2500 new=2498 notready=0 fail=2 tx=2 comp=0 failrate=8.0e-04 maxgapUs=8100 maxdurUs=260 evTimeouts=2 recovered=1
"""


class _FakeBench:
    """--selftest 专用假控制台：按 T1 契约即时回显，离线验证 run_flow 的命令序列。"""

    def __init__(self, ver=BARO_VER_LINE, init_ok=True):
        self.lines = []
        self._q = []
        self._ver = ver
        self._init_ok = init_ok

    # --- 串口 API 的替身 ---
    def attach_log(self, path):
        pass

    def close(self):
        pass

    def drain(self, seconds=0.3):
        pass

    def _emit(self, line):
        self.lines.append(line)
        self._q.append(line)

    def send(self, text):
        self.lines.append(">>> " + text)
        parts = text.split()
        head = parts[0].lower() if parts else ""
        if head == "help":
            self._emit("ok> ")
        elif head == "ver":
            self._emit(self._ver)
        elif head == "cfg":
            self._emit("[cfg] i2c=I2C1 scl=PB6 sda=PB7 addr=0x76 speed=400000 osr=0x03 odr=0x02 cfg=0x00 int=0x00 pwr=0x33")
        elif head == "sr":
            self._emit("[sr] sr1=0x0000 sr2=0x0000 af=0 berr=0 arlo=0 busy=0 msL=0 tra=0 txE=1 rxne=0 btf=0 ccr=0 trise=0 ackCfg=0")
        elif head == "bus":
            self._emit("[bus] scl=1 sda=1 cpu=168 cycPerUs=168")
        elif head == "reset":
            self._emit("[reset] ok")
        elif head == "init":
            hz = parts[1] if len(parts) > 1 else "400000"
            if self._init_ok:
                self._emit("[init] speed={0} stage=readback result=OK err=0x0000 chipId=0x50 attempts=1".format(hz))
            else:
                self._emit("[init] speed={0} stage=calib result=FAIL err=0x0403 chipId=0x50 attempts=3".format(hz))
        elif head == "soak":
            ms = int(parts[1])
            self._emit("[soak.end] t={0} att={1} new={1} notready=0 fail=0 tx=0 comp=0 "
                       "failrate=0.0e+00 maxgapUs=8100 maxdurUs=270 evTimeouts=0 recovered=0".format(
                           ms, ms // 8))
        elif head == "inject":
            for i in range(1, int(parts[1]) + 1):
                self._emit("[inject] i={0} pre_sr1=0x0000 af=0 nack_ok=1 post_sr1=0x0400 "
                           "post_af=1 next_ok=1 attempts_to_ok=1".format(i))
        elif head == "speed":
            self._emit("[speed] hz={0} result=OK".format(parts[1]))
        elif head == "stress":
            self._emit("[stress] state={0} spi=ok i2c2=ok adc=ok".format(parts[1]))
        elif head == "recover":
            self._emit("[recover] sr1_before=0x0400 sr2_before=0x0000 sr1_after=0x0000 "
                       "sr2_after=0x0000 next_ok=1")
        elif head == "stat":
            self._emit("[stat] att=1000 new=1000 notready=0 fail=0 tx=0 comp=0 evTimeouts=0 "
                       "recovered=0 maxgapUs=8100 maxdurUs=270")

    def expect(self, pattern, timeout):
        rx = re.compile(pattern)
        for i, line in enumerate(self._q):
            if rx.search(line):
                return self._q.pop(i)
        return ""

    def wait_lines(self, pattern, count, timeout):
        rx = re.compile(pattern)
        got = []
        i = 0
        while i < len(self._q) and len(got) < count:
            if rx.search(self._q[i]):
                got.append(self._q.pop(i))
            else:
                i += 1
        return got


FAKE_FLOW_CMDS = ["help", "ver", "cfg", "sr", "bus", "reset", "init", "soak 60000", "inject 20",
                  "recover", "speed 100000", "soak 20000", "speed 400000", "soak 20000",
                  "stress on", "soak 20000", "stress off", "recover", "stat"]


def _flog(text):
    return [line for line in text.splitlines() if line.strip()]


def selftest():
    """不开串口，用合成日志跑解析+汇总+CSV，并断言关键统计值。

    这是本机（无 ST-LINK / 无真板子串口）唯一能真跑的验收；任何解析/聚类/汇总回归都会让它失败。
    """
    checks = []

    def check(name, ok, detail=""):
        checks.append((name, bool(ok), detail))
        line = ("[ok]   " if ok else "[FAIL] ") + name
        if not ok and detail:
            line += "   <" + str(detail) + ">"
        print(line, flush=True)

    # ---- F1 正常 soak --------------------------------------------------------
    an1 = analyze(_flog(_SYN_NORMAL))
    vok1, vline1 = check_version(an1)
    check("F1 版本门禁接受 r2", vok1, vline1)
    check("F1 soak 段数=1", len(an1.segments) == 1, len(an1.segments))
    seg1 = an1.segments[0] if an1.segments else {}
    end1 = seg1.get("end") or {}
    check("F1 段标签 soak1@boot", seg1.get("label") == "soak1@boot", seg1.get("label"))
    check("F1 [soak.end] 统计值", end1.get("att") == 7500 and end1.get("new") == 7500
          and end1.get("notready") == 0 and end1.get("fail") == 0 and end1.get("tx") == 0
          and end1.get("comp") == 0 and end1.get("maxgap") == 8100 and end1.get("maxdur") == 270
          and end1.get("evtimeouts") == 0 and end1.get("recovered") == 0, end1)
    try:
        check("F1 failrate=0", float(end1.get("failrate", "nan")) == 0.0, end1.get("failrate"))
    except ValueError:
        check("F1 failrate=0", False, end1.get("failrate"))
    check("F1 无 [fail] 行", len(an1.fails) == 0, len(an1.fails))
    check("F1 1s tick 解析=2 条", len(seg1.get("ticks", [])) == 2
          and seg1["ticks"][1].get("t") == 2000 and seg1["ticks"][1].get("att") == 250,
          [t.get("t") for t in seg1.get("ticks", [])])
    check("F1 cfg/sr/bus 解析", an1.cfg is not None and an1.cfg.get("speed") == 400000
          and an1.sr is not None and an1.sr.get("sr1") == 0x0000
          and an1.bus is not None and an1.bus.get("cyc") == 168,
          (an1.cfg, an1.sr, an1.bus))

    # ---- F2 tx + comp 两类失败 / 聚类 ---------------------------------------
    an2 = analyze(_flog(_SYN_FAIL))
    check("F2 失败总数=5", len(an2.fails) == 5, len(an2.fails))
    check("F2 kind 分布 tx=3 comp=2",
          Counter(r.get("kind") for r in an2.fails) == Counter({"tx": 3, "comp": 2}),
          Counter(r.get("kind") for r in an2.fails))
    seg2 = an2.segments[0] if an2.segments else {}
    check("F2 段标签含 100000Hz", "100000Hz" in (seg2.get("label") or ""), seg2.get("label"))
    check("F2 段内 [fail] 行数=5", len(seg2.get("fails", [])) == 5, len(seg2.get("fails", [])))
    clusters2 = cluster_fails(an2.fails)
    check("F2 簇长度序列 [3,1,1]", [len(c) for c in clusters2] == [3, 1, 1],
          [len(c) for c in clusters2])
    check("F2 连续失败长度直方图 {1:2, 3:1}", _hist([len(c) for c in clusters2]) == {1: 2, 3: 1},
          _hist([len(c) for c in clusters2]))
    check("F2 单次簇数=2 最长=3",
          sum(1 for c in clusters2 if len(c) == 1) == 2 and max(len(c) for c in clusters2) == 3)
    check("F2 sr1 分布 {0x0400:3, 0x0000:2}",
          _hx_hist(an2.fails, "sr1", 4) == {"0x0400": 3, "0x0000": 2},
          _hx_hist(an2.fails, "sr1", 4))
    check("F2 lastEv 分布 {0x00000100:3, 0x00000000:2}",
          _hx_hist(an2.fails, "lastev", 8) == {"0x00000100": 3, "0x00000000": 2},
          _hx_hist(an2.fails, "lastev", 8))
    check("F2 failrate=2e-3",
          (seg2.get("end") or {}).get("failrate") == "2.0e-03", (seg2.get("end") or {}).get("failrate"))
    check("F2 evTimeouts/recovered", (seg2.get("end") or {}).get("evtimeouts") == 3
          and (seg2.get("end") or {}).get("recovered") == 1, seg2.get("end"))

    # ---- F3 inject attempts_to_ok 1 与 5 ------------------------------------
    an3 = analyze(_flog(_SYN_INJECT))
    check("F3 inject 行数=6", len(an3.injects) == 6, len(an3.injects))
    check("F3 attempts_to_ok 分布 {1:4, 5:2}",
          _hist([r.get("attempts") for r in an3.injects]) == {1: 4, 5: 2},
          _hist([r.get("attempts") for r in an3.injects]))
    check("F3 attempts_to_ok==1 计数=4",
          sum(1 for r in an3.injects if r.get("attempts") == 1) == 4)
    check("F3 post_af 分布 {1:4, 0:2}",
          _hist([r.get("post_af") for r in an3.injects]) == {1: 4, 0: 2},
          _hist([r.get("post_af") for r in an3.injects]))
    check("F3 stat 解析 fail=5 tx=3 comp=2",
          an3.stats and an3.stats[-1].get("fail") == 5 and an3.stats[-1].get("tx") == 3
          and an3.stats[-1].get("comp") == 2 and an3.stats[-1].get("maxgap") == 20000,
          an3.stats)

    # ---- F4 版本不匹配 / 严格性 ---------------------------------------------
    an4 = analyze(_flog(_SYN_OLD_VERSION))
    vok4, vline4 = check_version(an4)
    check("F4 旧版本 r2 被门禁拒绝（只认 r3）", (not vok4) and "r2" in vline4, vline4)
    an5 = analyze([])
    vok5, vline5 = check_version(an5)
    check("F4 无 [ver] 行被拒绝", not vok5, vline5)
    check("F4 截断 [fail] 行不匹配",
          parse_line("[fail] i=1 t=1500 kind=tx sr1=0x0400 sr2=0x0000") is None)
    check("F4 截断 [soak.end] 行不匹配",
          parse_line("[soak.end] t=20000 att=2500 new=2495 fail=5") is None)
    check("F4 非法 kind 不匹配",
          parse_line("[fail] i=1 t=1500 kind=xyz sr1=0x0400 sr2=0x0000 lastEv=0x00000001 "
                     "evD=1 status=0x02 raw=AA55AA55AA55 durUs=1 gapUs=1 streak=1") is None)

    # ---- F5 汇总 + CSV 落盘（与真跑走同一条 write_outputs 路径）--------------
    combo = _flog(_SYN_NORMAL) + _flog(_SYN_FAIL) + _flog(_SYN_INJECT)
    tmp = Path(tempfile.mkdtemp(prefix="baro_selftest_"))
    meta = {"time": "selftest", "raw_log": "(合成日志)", "cmdline": "run_baro.py --selftest",
            "selftest": True}
    out = write_outputs(combo, tmp, meta)
    check("F5 汇总段数=2", len(out["analysis"].segments) == 2, len(out["analysis"].segments))
    check("F5 fails.csv 5 行数据", len(out["fails_rows"]) == 5, len(out["fails_rows"]))
    check("F5 inject.csv 6 行数据", len(out["inject_rows"]) == 6, len(out["inject_rows"]))
    check("F5 fails.csv/inject.csv/summary.md 落盘",
          (tmp / "fails.csv").is_file() and (tmp / "inject.csv").is_file()
          and (tmp / "summary.md").is_file())
    csv_lines = (tmp / "fails.csv").read_text(encoding="utf-8").strip().splitlines()
    check("F5 fails.csv 行数=表头+5", len(csv_lines) == 6, len(csv_lines))
    check("F5 fails.csv 表头字段顺序",
          csv_lines[0].split(",") == FAIL_COLS, csv_lines[0])
    inject_lines = (tmp / "inject.csv").read_text(encoding="utf-8").strip().splitlines()
    check("F5 inject.csv 行数=表头+6", len(inject_lines) == 7, len(inject_lines))
    text = out["summary"]
    check("F5 summary 含簇长度直方图 {1: 2, 3: 1}", "{1: 2, 3: 1}" in text)
    check("F5 summary 含 soak 段表 att=7500", "| 7500 |" in text)
    check("F5 summary 含 attempts_to_ok 分布", "attempts_to_ok" in text and "| 5 | 2 |" in text)
    check("F5 summary 含 kind 比例 tx:comp = 3:2（不含 other）",
          "tx:comp（**不含 kind=other**）= **3:2**" in text)
    check("F5 summary 含 lastEv 分布", "| lastEv | `0x00000100` | 3 |" in text)
    check("F5 summary 含版本门禁 PASS", "版本门禁：**PASS**" in text)
    check("F5 summary 含命令序列清单",
          "- `soak 60000`" in text and "- `inject 6`" in text and "- `stat`" in text)
    check("F5 结论：tx+comp 两类并存",
          any("两类都在" in line for line in build_conclusions(out["analysis"], out["clusters"],
                                                              out["hist"])),
          out["hist"])
    check("F5 结论：注入 >1 次尝试只算“与 AF 未清相容”，且点明兼容 H4",
          any(("同样兼容" in line and "H4" in line)
              for line in build_conclusions(out["analysis"], out["clusters"], out["hist"])))
    check("F5 结论：attempts_to_ok==1 明确写成“两种情形不可分”",
          any("两种情形不可分" in line
              for line in build_conclusions(out["analysis"], out["clusters"], out["hist"])))
    old_out = write_outputs(_flog(_SYN_OLD_VERSION), tmp / "old", meta)
    check("F5 版本不匹配的汇总标记 FAIL", "版本门禁：**FAIL**" in old_out["summary"])

    # ---- F6 默认流程命令序列（假控制台离线跑 run_flow）----------------------
    def _pos(seq, item):
        return seq.index(item) if item in seq else -1

    flat = argparse.Namespace(soak=60000, side_soak=20000, inject=20, speed=400000,
                              no_stress=False)
    fb = _FakeBench()
    f1 = Flow(quiet=True)
    res1 = run_flow(fb, flat, f1)
    cmds1 = [l[4:] for l in fb.lines if l.startswith(">>> ")]
    check("F6 命令序列 = 默认流程（含 init 与 recover@post-inject）",
          cmds1 == FAKE_FLOW_CMDS, cmds1)
    check("F6 init 在 reset 之后、第一个 soak 之前",
          _pos(cmds1, "init") >= 0 and _pos(cmds1, "init") == _pos(cmds1, "reset") + 1
          and _pos(cmds1, "init") < _pos(cmds1, "soak 60000"), cmds1)
    check("F6 inject 之后紧接一次 recover（趁 SR1 还带 AF 采 sr1_before）",
          _pos(cmds1, "inject 20") >= 0 and _pos(cmds1, "inject 20") + 1 < len(cmds1)
          and cmds1[_pos(cmds1, "inject 20") + 1] == "recover", cmds1)
    check("F6 init=OK ⇒ init_ok=True 且无失败步骤",
          res1["version_ok"] is True and res1["init_ok"] is True and f1.failed == 0,
          (res1, f1.failed))
    fb2 = _FakeBench(init_ok=False)
    f2 = Flow(quiet=True)
    res2 = run_flow(fb2, flat, f2)
    check("F6 init=FAIL ⇒ init_ok=False 且计为失败步骤（退出码非 0）",
          res2["init_ok"] is False and f2.failed >= 1, (res2, f2.failed))
    fb3 = _FakeBench(ver="[ver] baro bench r2 2026-10-18")
    f3 = Flow(quiet=True)
    res3 = run_flow(fb3, flat, f3)
    cmds3 = [l[4:] for l in fb3.lines if l.startswith(">>> ")]
    check("F6 版本不匹配只发 help/ver 且不放行",
          res3["version_ok"] is False and cmds3 == ["help", "ver"], (res3, cmds3))

    # ---- F7 新契约：未初始化指纹 / kind=other / [boot] ----------------------
    an_ni = analyze(_flog(_SYN_NOT_INITIALIZED))
    warn_ni = build_warnings(an_ni)
    check("F7 未初始化指纹(comp==att + maxdurUs=0)触发警告",
          any("器件可能未初始化" in w for w in warn_ni), warn_ni)
    check("F7 未初始化指纹段被识别且结论作废", len(uninit_fingerprints(an_ni)) == 1,
          len(uninit_fingerprints(an_ni)))
    ni_out = write_outputs(_flog(_SYN_INIT_FAIL), tmp / "initfail", meta)
    check("F7 init=FAIL ⇒ summary 顶部含“数据不可信（器件未初始化）”",
          "数据不可信（器件未初始化）" in ni_out["summary"]
          and "器件可能未初始化" in ni_out["summary"])
    check("F7 init=FAIL ⇒ init_state=False", init_state_from(ni_out["analysis"])[0] is False)
    an_ot = analyze(_flog(_SYN_OTHER_KIND))
    check("F7 kind=other 解析成功且 err=0x0403",
          any(r.get("kind") == "other" and r.get("err") == 0x0403 for r in an_ot.fails),
          [(r.get("kind"), r.get("err")) for r in an_ot.fails])
    check("F7 [boot] 行不被当成 [init]（inits 只有 1 条且是 OK）",
          len(an_ot.boots) == 1 and len(an_ot.inits) == 1
          and an_ot.inits[0].get("result") == "OK", (an_ot.boots, an_ot.inits))
    ot_out = write_outputs(_flog(_SYN_OTHER_KIND), tmp / "other", meta)
    check("F7 kind=other 进 fails.csv 且 err 列有值",
          any(r["kind"] == "other" and r["err"] == "0x0403" for r in ot_out["fails_rows"]),
          ot_out["fails_rows"])
    check("F7 kind 比例不含 other（= 1:1 且 other 单列）",
          "tx:comp（**不含 kind=other**）= **1:1**" in ot_out["summary"]
          and "kind=other **单列 1 次**" in ot_out["summary"])
    an_nv = analyze(_flog(_SYN_INJECT_NEVER))
    nv_conc = build_conclusions(an_nv, [], {})
    check("F7 attempts_to_ok=-1 单列为 AF 粘连最强证据",
          any("尝试上限内始终没恢复" in line for line in nv_conc), nv_conc)
    check("F7 attempts_to_ok==1 不再被写成“支持无 AF 粘连”",
          not any("不支持" in line and "AF" in line for line in nv_conc), nv_conc)
    check("F7 fails.csv 的 raw 列是 12 位十六进制（不是十进制）",
          {r["raw"] for r in ot_out["fails_rows"]} == {"AA55AA55AA55", "000000000000"},
          {r["raw"] for r in ot_out["fails_rows"]})
    check("F7 有 end.fail 但没有 [fail] 行时不写“未复现”",
          any(("不能" in line) and ("未复现" in line)
              for line in build_conclusions(an_ni, [], {})),
          build_conclusions(an_ni, [], {}))
    an_rf = analyze(_flog(_SYN_REFUSED))
    warn_rf = build_warnings(an_rf)
    check("F7 [err] not-initialized 被收下并给出“拒绝执行/无数据”警告",
          any(("拒绝" in w) and ("not-initialized" in w) for w in warn_rf)
          and any("没有 `[soak.end]`" in w for w in warn_rf), warn_rf)
    check("F7 [err] not-initialized 不会被当成 bad-arg 丢掉",
          len(an_rf.errs) == 2, [e.get("_raw") for e in an_rf.errs])
    an_te = analyze(_flog(_SYN_TRAILING_ERR))
    check("F7 行尾 err=0x0403 被解析且 kind=other",
          len(an_te.fails) == 1 and an_te.fails[0].get("kind") == "other"
          and an_te.fails[0].get("err") == 0x0403, an_te.fails)
    te_seg = an_te.segments[0] if an_te.segments else {}
    te_out = write_outputs(_flog(_SYN_TRAILING_ERR), tmp / "trailing", meta)
    check("F7 新口径 other==att 也算未初始化指纹（fail-tx-comp 推算）",
          len(uninit_fingerprints(an_te)) == 1, (te_seg.get("end"),))
    check("F7 soak 表含 other* 推算列且值正确",
          "| 2500 | 0 | 0 | 2500 |" in te_out["summary"], None)

    # ---- F8 A21: 失败瞬间 PB6(SCL)/PB7(SDA) 电平 ---------------------------
    an_lv = analyze(_flog(_SYN_LEVELS))
    lv_out = write_outputs(_flog(_SYN_LEVELS), tmp / "levels", meta)
    check("F8 带 scl/sda 的 [fail] 行解析出电平",
          [r.get("scl") for r in an_lv.fails] == [0, 0, 0, 1, 1]
          and [r.get("sda") for r in an_lv.fails] == [0, 0, 1, 0, 1],
          [(r.get("scl"), r.get("sda")) for r in an_lv.fails])
    check("F8 fails.csv 的 scl/sda 列有值",
          [r["scl"] for r in lv_out["fails_rows"]] == [0, 0, 0, 1, 1]
          and [r["sda"] for r in lv_out["fails_rows"]] == [0, 0, 1, 0, 1],
          lv_out["fails_rows"])
    check("F8 summary (scl,sda) 组合表四项计数",
          all(tok in lv_out["summary"]
              for tok in ("| (0,0) | 2 |", "| (0,1) | 1 |", "| (1,0) | 1 |", "| (1,1) | 1 |")),
          lv_out["summary"][lv_out["summary"].index("## 失败瞬间"):][:320])
    # fixture 期望值自算（独立于产品代码）：5 行样本 (0,0)x2/(0,1)/(1,0)/(1,1)
    exp_held = sum(1 for r in an_lv.fails if r.get("scl") == 0 or r.get("sda") == 0)
    exp_both = sum(1 for r in an_lv.fails if r.get("scl") == 0 and r.get("sda") == 0)
    check("F8 fixture 期望自算 held=4 / both_low=2",
          exp_held == 4 and exp_both == 2, (exp_held, exp_both))
    # 派生计数钉在渲染文本上：held 与 both_low 各自出现在同一句里，改坏任一个都会失败
    check("F8 summary 钉住派生计数 held=4 与 both_low=2（样本 5/5）",
          "带电平样本 **5/5** 行" in lv_out["summary"]
          and "scl=0 或 sda=0 共 **4** 行（80.0%）" in lv_out["summary"]
          and "两条同时为 0 的 2 行" in lv_out["summary"],
          [l for l in lv_out["summary"].splitlines() if "scl=0 或 sda=0" in l])
    check("F8 电平判读提示是“可能/不足以定性”，不是因果断言",
          "可能被从机拉住（H4）" in lv_out["summary"]
          and "单次出现不足以定性" in lv_out["summary"]
          and "结合 `[recover]` 的 `next_ok`" in lv_out["summary"])
    old_rows = fails_csv_rows(an2)
    check("F8 缺 scl/sda 的旧式行仍解析成功、CSV 对应列为空",
          len(an2.fails) == 5 and all(r["scl"] == "" and r["sda"] == "" for r in old_rows),
          old_rows[:1])
    check("F8 版本门禁：r2 被拒、r3 被接受",
          check_version(an4)[0] is False and check_version(an_lv)[0] is True)
    # 尾部截断的 r2 行：数据保留 + 显式截断警告（不能误写成"旧固件"，也不能静默偏少）
    an_tr = analyze(_flog(_SYN_TRUNCATED_TAIL))
    tr_out = write_outputs(_flog(_SYN_TRUNCATED_TAIL), tmp / "trunc", meta)
    check("F8 r2 日志缺 scl/sda 触发“尾部截断”警告（而不是“旧固件”）",
          any(("尾部截断" in w) and ("带电平样本数" in w) for w in build_warnings(an_tr)),
          build_warnings(an_tr))
    check("F8 截断行数据仍保留：进 fails.csv 且 scl/sda 留空",
          len(tr_out["fails_rows"]) == 2 and tr_out["fails_rows"][1]["scl"] == ""
          and tr_out["fails_rows"][1]["sda"] == "", tr_out["fails_rows"])
    tr_lines = [l for l in tr_out["summary"].splitlines() if l.startswith("- 带电平样本")]
    check("F8 summary 带电平样本 **1/2** 且原因是截断",
          len(tr_lines) == 1 and "带电平样本 **1/2** 行" in tr_lines[0]
          and "疑似串口尾部截断" in tr_lines[0] and "旧固件" not in tr_lines[0],
          tr_lines)

    total = len(checks)
    passed = sum(1 for _, ok, _ in checks if ok)
    print("", flush=True)
    print("run_baro.py selftest: {0}/{1} checks ok".format(passed, total), flush=True)
    if passed == total:
        print("run_baro.py selftest: ok", flush=True)
        return 0
    print("run_baro.py selftest: FAILED ({0} check(s))".format(total - passed), flush=True)
    for name, ok, detail in checks:
        if not ok:
            print("  - {0}  <{1}>".format(name, detail), flush=True)
    return 1


# --------------------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------------------
FLOW_HELP = """\
默认流程：
  ver -> cfg -> sr -> bus -> reset -> init（必须 OK，否则数据不可信）
  -> soak <--soak>ms @400k（上电默认）
  -> inject <--inject> -> recover@post-inject（趁 SR1 还带 AF 采 sr1_before）
  -> speed 100000 + soak <--side-soak>ms
  -> speed <--speed> + soak <--side-soak>ms
  -> stress on + soak <--side-soak>ms -> stress off
  -> recover -> stat
退出码：0 正常 / 1 有步骤判据失败 / 2 串口或环境问题 / 3 版本门禁不过 / 4 器件未初始化"""


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="气压计(BMP388/I2C1)台架自动化：把偶发 0x404 拆成 kind/聚类/AF 粘连三类判据",
        epilog=FLOW_HELP,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="列出串口后退出")
    ap.add_argument("--port", default="", help="串口（默认自动找口，跳过 bluetooth）")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--soak", type=int, default=DEFAULT_SOAK_MS, help="主 soak 毫秒（默认 60000）")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED, help="被测主速度 Hz（默认 400000）")
    ap.add_argument("--inject", type=int, default=DEFAULT_INJECT, help="NACK 注入次数（默认 20）")
    ap.add_argument("--stress", action="store_true",
                    help="含 stress 并发负载段（默认已含；显式声明等价）")
    ap.add_argument("--no-stress", action="store_true", help="跳过 stress 段")
    ap.add_argument("--side-soak", type=int, default=DEFAULT_SIDE_SOAK_MS,
                    help="变速/带负载 soak 毫秒（默认 20000）")
    ap.add_argument("--out", default="", help="输出目录（默认 test-data/baro_bench/<时间戳>）")
    ap.add_argument("--selftest", action="store_true",
                    help="不打开串口，用内嵌合成日志验收解析/汇总（退出码 0/1）")
    args = ap.parse_args(argv)

    if args.selftest:
        return selftest()
    if args.list:
        if not HAVE_SERIAL:
            print("FAIL: 需要 pyserial（python -m pip install pyserial）")
            return 2
        for port in list_ports.comports():
            tag = "   [自动找口会跳过：蓝牙]" if is_bluetooth(port) else ""
            print("{0}\t{1}{2}".format(port.device, port.description, tag))
        return 0
    if not HAVE_SERIAL:
        print("FAIL: 需要 pyserial（python -m pip install pyserial）")
        return 2

    out_dir = Path(args.out) if args.out else OUT_ROOT / _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    log_path = out_dir / "raw.log"
    meta = {
        "time": _dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "raw_log": str(log_path),
        "cmdline": "python " + " ".join(["tools/bench/run_baro.py"] + (argv if argv is not None
                                                                     else sys.argv[1:])),
        "selftest": False,
    }

    print("[out] {0}".format(out_dir), flush=True)
    print(FLOW_HELP, flush=True)

    try:
        bench = Bench(args.port, args.baud) if args.port else open_auto(args.baud)
    except Exception as exc:
        print("FAIL: 打开串口失败: {0}".format(exc), flush=True)
        print("提示：本机只有蓝牙虚拟口，真板子串口不在时用 --list 核对；无硬件请跑 --selftest。",
              flush=True)
        return 2
    bench.attach_log(log_path)      # 控制台确认后才落 raw.log（开不了口不留空目录）

    flow = Flow()
    try:
        result = run_flow(bench, args, flow)
    finally:
        bench.close()
    meta["steps"] = list(flow.steps)        # 步骤判据写进 summary（含 init 的 result/err/chipId）
    meta["init_ok"] = result.get("init_ok")

    out = write_outputs(bench.lines, out_dir, meta)
    print("", flush=True)
    print("[raw]     {0}".format(log_path), flush=True)
    print("[summary] {0}".format(out["summary_path"]), flush=True)
    print("[csv]     {0}".format(out_dir / "fails.csv"), flush=True)
    print("[csv]     {0}".format(out_dir / "inject.csv"), flush=True)

    if not result.get("version_ok"):
        print("FAIL: 固件版本门禁不通过，数据作废（见 summary.md）", flush=True)
        return 3
    if result.get("init_ok") is False:
        print("FAIL: 器件未初始化（init 未返回 result=OK）⇒ 后续 soak/inject 数据不可信，"
              "退出码 4；见 summary.md 顶部警告", flush=True)
        return 4
    if flow.failed:
        print("FAIL: {0} 个步骤判据失败（见 summary.md）".format(flow.failed), flush=True)
        return 1
    print("PASS: 流程跑完且无步骤失败；判定见 summary.md 的\"判读结论\"", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
