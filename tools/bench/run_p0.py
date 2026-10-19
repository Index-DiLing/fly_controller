#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""run_p0.py —— 链路 v2 P0 台架自动化（飞控 bench 串口控制台）

owner 只需接线/上电；脚本按 P0 文档顺序跑测量，原始日志 + 解析摘要写到 test-data/。
串口 = 飞控 bench 的 USART1 控制台（115200）。

用法
    python tools/bench/run_p0.py                 # 自动找口
    python tools/bench/run_p0.py --port COM11 --soak 60
    python tools/bench/run_p0.py --list

设计要点（都是台架实测踩出来的）
    - RC bench 每 5s 主动回 "#STAT"（ASCII），在串口上是 `[rx] ... [hex]` 的文本帧；
      新版 FC bench 的 round 会过滤 ASCII burst，不会污染轮次统计。
    - PATTERN 占空 ~75%，用命令从 PATTERN 切回 SILENT 大概率撞包收不到；所以 PATTERN
      支持 `dur=ms` 自动停（RC bench 本次改动）。脚本每次测 round 都重新起一段带 dur 的 PATTERN。
    - 串口提示符 "ok> " 不一定带换行，脚本不依赖它，改用"静默/流量"判定。
"""

import argparse
import datetime as _dt
import queue
import re
import sys
import threading
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("FAIL: 需要 pyserial（python -m pip install pyserial）")
    sys.exit(2)

REPO = Path(__file__).resolve().parent.parent.parent
OUT_DIR = REPO / "test-data"
BENCH_VER = "p0 bench r3 2026-10-18"

RX_RE = re.compile(r"\[rx\] len=(\d+) t=(\d+)us .*\[((?:[0-9A-Fa-f]{2} )*)\]")
PAYLOAD_RE = re.compile(r"\[rx\] len=8 .*A0 A1 A2 A3")


class Bench:
    def __init__(self, port: str, baud: int, log_path: Path):
        self.log_path = log_path
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self._logf = open(log_path, "a", encoding="utf-8", errors="replace")
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.q: "queue.Queue[str]" = queue.Queue()
        self.lines: list[str] = []
        self._stop = False
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self) -> None:
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
                self._logf.write(f"[{stamp}] {text}\n")
                self._logf.flush()
                self.q.put(text)
                self.lines.append(text)

    def close(self) -> None:
        self._stop = True
        try:
            self.ser.close()
        finally:
            self._logf.close()

    def send(self, text: str) -> None:
        self._logf.write(f"        >>> {text}\n")
        self._logf.flush()
        self.ser.write((text + "\r\n").encode("ascii", errors="replace"))
        self.ser.flush()

    def mark(self) -> int:
        return len(self.lines)

    def since(self, mark: int) -> list[str]:
        return self.lines[mark:]

    def expect(self, pattern: str, timeout: float) -> list[str]:
        rx = re.compile(pattern)
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                line = self.q.get(timeout=min(0.2, max(0.01, deadline - time.time())))
            except queue.Empty:
                continue
            if rx.search(line):
                return [line]
        return []

    def drain(self, seconds: float = 0.3) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline:
            try:
                self.q.get(timeout=0.05)
            except queue.Empty:
                pass


def open_auto(baud: int, log_path: Path) -> Bench:
    for p in list_ports.comports():
        if "bluetooth" in p.description.lower():
            continue
        try:
            b = Bench(p.device, baud, log_path)
        except Exception:
            continue
        b.send("help")
        if b.expect(r"ok>|sweep", 2.0):
            print(f"[auto] bench 控制台: {p.device} ({p.description})")
            return b
        b.close()
    raise RuntimeError("没有找到响应 help 的 bench 控制台串口，请用 --port 指定")


# --------------------------------------------------------------------------------------------------
# RC 模式控制
# --------------------------------------------------------------------------------------------------
def rc_send(b: Bench, text: str, wait_ok: float = 2.0) -> bool:
    # 注意：FC bench 的 rc 命令一次连续写，文本（含 @ 与 \n）必须 ≤32B —— 超了会打
    # "[rc] FAILED sent=37 uart=0"。所以 RC 侧命令一律用默认 len/gap，只传必要参数。
    if len(text) > 30:
        print(f"[warn] rc 文本过长({len(text)}): {text}", flush=True)
    b.send(f"rc {text}")
    got = b.expect(r"\[rc\] (ok|FAILED)", wait_ok)
    return bool(got) and "ok" in got[0]


def payload_count(lines: list[str]) -> int:
    return sum(1 for l in lines if PAYLOAD_RE.search(l))


def try_silent(b: Bench, tries: int = 8) -> bool:
    """把 RC 拉回 SILENT：命令可能撞包，重试；以'不再有 0xA0 载荷'为成功。"""
    for _ in range(tries):
        rc_send(b, "MODE SILENT")
        b.drain(0.4)
        m = b.mark()
        time.sleep(0.8)
        if payload_count(b.since(m)) == 0:
            return True
    return False


def start_pattern(b: Bench, dur_ms: int, tries: int = 4) -> bool:
    # 默认 len=8 gap=22，只传 dur（命令必须短，见 rc_send 注释）
    for _ in range(tries):
        if not rc_send(b, f"MODE PATTERN dur={dur_ms}"):
            b.drain(0.3)
            continue
        b.drain(0.3)
        m = b.mark()
        time.sleep(1.0)
        if payload_count(b.since(m)) >= 3:
            return True
    return False


def wait_pattern_end(b: Bench, timeout: float) -> None:
    """PATTERN 到 dur 会自动停；等到连续 0.8s 没有载荷。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        m = b.mark()
        time.sleep(0.8)
        if payload_count(b.since(m)) == 0:
            return


def hex_to_text(lines: list[str]) -> str:
    out = bytearray()
    for l in lines:
        m = RX_RE.search(l)
        if not m:
            continue
        for h in m.group(3).split():
            out.append(int(h, 16))
    return out.decode("latin-1", errors="ignore")


def extract_rc_lines(lines: list[str]) -> dict:
    text = hex_to_text(lines)
    got = {}
    for m in re.finditer(r"#(BOOT|ACK|STAT|PONG|ERR)[^\r\n]*", text):
        got[m.group(1)] = m.group(0).strip()
    return got


# --------------------------------------------------------------------------------------------------
# 主流程
# --------------------------------------------------------------------------------------------------
def run(b: Bench, args) -> dict:
    res = {"steps": [], "fails": []}

    def note(ok: bool, msg: str) -> None:
        res["steps"].append((ok, msg))
        if not ok:
            res["fails"].append(msg)
        print(("[PASS] " if ok else "[FAIL] ") + msg, flush=True)

    # 0) 控制台 / 版本 / 接线
    b.send("")
    b.send("help")
    if not b.expect(r"sweep", 3.0):
        note(False, "控制台无响应")
        return res
    m0 = b.mark()
    b.send("ver")
    b.expect(r"\[ver\]", 2.0)
    ver = [l for l in b.since(m0) if "[ver]" in l]
    res["ver"] = ver[-1].strip() if ver else "(无 ver 命令)"
    ver_ok = BENCH_VER in res["ver"]
    note(ver_ok, f"bench 固件: {res['ver']}" + ("" if ver_ok else f"（期望 {BENCH_VER}，请重烧 bench）"))

    b.send("cfg")
    b.expect(r"\[cfg\]", 2.0)
    b.send("aux")
    b.expect(r"\[aux\]", 2.0)
    aux = [l for l in b.lines[-30:] if "[aux]" in l]
    res["aux"] = aux[-1].strip() if aux else ""

    # 1) 先把 RC 拉回 SILENT
    if not try_silent(b):
        note(False, "RC 不静默（PATTERN 停不下来）——请复位遥控器后重跑")
        return res
    note(True, "RC 已静默")

    if args.boot_wait > 0:
        print(f"[boot] 请给遥控器重新上电（等 #BOOT，最多 {args.boot_wait:.0f}s）...", flush=True)
        m0 = b.mark()
        time.sleep(args.boot_wait)
        rc_lines = extract_rc_lines(b.since(m0))
        if "BOOT" in rc_lines:
            res["boot"] = rc_lines["BOOT"]
            note(True, f"#BOOT: {res['boot'][:110]}")
        else:
            note(False, "未捕获 #BOOT（非关键：只缺寄存器快照）")
        try_silent(b)

    # 2) F(n)
    m0 = b.mark()
    b.send("sweep 3")
    b.expect(r"LinkAirtimeBaseUs", 150.0)
    b.drain(0.5)
    for line in b.since(m0):
        m = re.search(r"F\(n\)=([\d.]+)us \+ ([\d.]+)us/B", line)
        if m:
            res["fit_base"], res["fit_per"] = float(m.group(1)), float(m.group(2))
    base_ok = "fit_base" in res and abs(res["fit_base"] - 13555) <= 200
    per_ok = "fit_per" in res and abs(res["fit_per"] - 387) <= 5
    note(base_ok and per_ok,
         f"F(n): {res.get('fit_base')} + {res.get('fit_per')}*n（期望 13555±200 / 387±5）")

    # 3) 裂包
    for n in (8, 21, 32):
        b.send(f"tx {n} 50 20")
        got = b.expect(r"\[tx\] ok=(\d+) miss=(\d+)", 20.0)
        b.drain(0.3)
        ok = bool(got) and "miss=0" in got[0]
        note(ok, f"tx {n}B×50: {got[0] if got else '(no result)'}")

    # 4) round：guard 0/1/3 (8B) + soak(21B)
    rounds = []
    for guard, dl_len, dur in ((0, 8, args.round_s), (1, 8, args.round_s),
                               (3, 8, args.round_s), (3, 21, args.soak)):
        pat_dur = int(dur * 1000) + 5000
        if not start_pattern(b, pat_dur):
            note(False, "RC 起 PATTERN 失败")
            break
        b.send(f"round 22 {guard} {dl_len}")
        if not b.expect(r"\[round\] s=(\d+)ms guard=(\d+)ms dlLen=(\d+)", 5.0):
            note(False, f"round 未启动 (guard={guard}, dl={dl_len})")
            wait_pattern_end(b, pat_dur / 1000 + 3)
            continue
        time.sleep(dur)
        b.send("stop")
        end = b.expect(r"\[round\] end rounds=(\d+) dlSent=(\d+) bursts=(\d+)", 10.0)
        drop = b.expect(r"rxDrop=(\d+) \(\+(\d+)\)", 5.0)
        wait_pattern_end(b, pat_dur / 1000 + 3)   # 等 PATTERN 自动停，再进下一项
        if not end:
            note(False, f"round (guard={guard}, dl={dl_len}) 没拿到结束行")
            continue
        m = re.search(r"rounds=(\d+) dlSent=(\d+) bursts=(\d+)", end[0])
        r, d, bu = (int(x) for x in m.groups())
        inc = drop[0].split("(+")[1].split(")")[0] if drop else "?"
        tag = "soak-21B" if dl_len == 21 else f"guard={guard}ms-8B"
        note(inc == "0", f"round {tag}: rounds={r} dlSent={d} bursts={bu} "
                         f"bursts/round={bu/max(r,1):.2f} rxDrop+={inc}")
        rounds.append((tag, r, d, bu))
        if dl_len == 21:
            res["new_fw"] = any("5*s=" in l for l in b.lines)
    res["rounds"] = rounds
    try_silent(b)

    # 5) stat + RC 统计
    m_stat = b.mark()
    b.send("stat")
    b.drain(1.0)
    stats = {}
    for line in b.since(m_stat):
        m = re.match(r"^\s*(.+?)\s+n=(\d+) min=(\d+) avg=(\d+) max=(\d+) p99=(\d+)\s*$", line)
        if m:
            stats[m.group(1).strip()] = dict(n=int(m.group(2)), min=int(m.group(3)),
                                             avg=int(m.group(4)), max=int(m.group(5)),
                                             p99=int(m.group(6)))
    res["stats"] = stats

    m_rc = b.mark()
    rc_send(b, "STAT")
    time.sleep(1.2)
    b.drain(0.5)
    rc_lines = extract_rc_lines(b.since(m_rc))
    if "STAT" in rc_lines:
        res["rc_stat"] = rc_lines["STAT"]
    m_ping = b.mark()
    rc_send(b, "PING")
    time.sleep(1.0)
    b.drain(0.3)
    res["pong"] = "PONG" in extract_rc_lines(b.since(m_ping))
    note(res["pong"], "遥控器 PING/PONG")
    return res


def write_summary(res: dict, log_path: Path, summary_path: Path) -> None:
    L = [
        "# P0 台架自动化摘要",
        "",
        f"- 时间：{_dt.datetime.now():%Y-%m-%d %H:%M:%S}",
        f"- 原始日志：`{log_path}`",
        f"- 飞控 bench：`{res.get('ver', '?')}`",
        f"- aux：`{res.get('aux', '')}`",
        "",
        "## 判据",
        "",
        "| 结果 | 项 |",
        "|---|---|",
    ]
    for ok, msg in res.get("steps", []):
        L.append(f"| {'PASS' if ok else 'FAIL'} | {msg} |")
    L += ["", "## F(n)", ""]
    if "fit_base" in res:
        L.append(f"- `F(n) = {res['fit_base']:.2f} + {res['fit_per']:.2f}·n` us（冻结 13555 + 387·n）")
    L += ["", "## round", "", "| 场景 | rounds | dlSent | bursts | bursts/round |", "|---|---|---|---|---|"]
    for tag, r, d, bu in res.get("rounds", []):
        L.append(f"| {tag} | {r} | {d} | {bu} | {bu/max(r,1):.2f} |")
    L += ["", "## stat", "", "| 统计 | n | min | avg | max | p99 |", "|---|---|---|---|---|---|"]
    for name, s in res.get("stats", {}).items():
        L.append(f"| {name} | {s['n']} | {s['min']} | {s['avg']} | {s['max']} | {s['p99']} |")
    if res.get("rc_stat"):
        L += ["", f"- 遥控器 #STAT：`{res['rc_stat']}`"]
    if res.get("boot"):
        L += ["", f"- #BOOT：`{res['boot']}`"]
    L += ["", "## 结论", ""]
    if res.get("fails"):
        L.append(f"**{len(res['fails'])} 项判据失败**，见上表。")
    else:
        L.append("台架可自动化项全部通过；物理收↔发切换（d0AuxUs 分布）与双机协议仍需 P4。")
    summary_path.write_text("\n".join(L) + "\n", encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--round-s", type=float, default=6.0)
    ap.add_argument("--soak", type=float, default=60.0)
    ap.add_argument("--boot-wait", type=float, default=0.0)
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    if args.list:
        for p in list_ports.comports():
            print(f"{p.device}\t{p.description}")
        return 0

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    ts = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = OUT_DIR / f"p0_auto_{ts}.log"
    summary_path = OUT_DIR / f"p0_auto_{ts}.summary.md"

    try:
        b = Bench(args.port, args.baud, log_path) if args.port else open_auto(args.baud, log_path)
    except Exception as exc:
        print(f"FAIL: 打开串口失败: {exc}")
        return 2
    print(f"[log] {log_path}", flush=True)
    try:
        res = run(b, args)
        write_summary(res, log_path, summary_path)
        print(f"[summary] {summary_path}", flush=True)
        return 0 if not res["fails"] else 1
    finally:
        b.close()


if __name__ == "__main__":
    sys.exit(main())
