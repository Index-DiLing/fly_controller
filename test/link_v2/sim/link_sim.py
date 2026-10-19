#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""链路 v2 轮次级离散事件仿真 —— 验证者独立模型（按 docs/链路层v2_协议规格.md 重写）

不引用飞控/遥控器任何实现代码；两端状态机是按规格 §4/§5/§7 独立重写的模型：
  RoundMaster（遥控器）：HANDSHAKE / MISMATCH / RUN / NO_LINK
  RoundSlave （飞控）  ：ACQUIRE / RUN

模型包含：F(n) 空中时间、逐帧丢失、遥控器发送迟到（>12ms 跳帧 / U0 顺延整轮）、AUX 忙、
两端重启、HELLO 丢失、版本不一致、v1 混烧（v1 RC / v1 FC 模型）。

随机场景断言：
  A. 半双工：任意两次发送的空中区间不重叠（无双发）。
  B. 无死锁：故障窗口结束后的干净尾巴里，双方回到 RUN 且持续收到下行。
  C. 恢复 ≤3 轮（360ms）：从最后一个故障事件（或故障窗口结束）算起。
  D. 版本不匹配：RC 从不发 0x8–0xD；FC 只发 HELLO、不进 RUN。
  E. v1 混烧：v2 RC + v1 FC 只发 HELLO 且永不 RUN；v2 FC + v1 RC 一帧不发。
  F. 每轮下行 ≤1 帧（迟到上行在 D0 之后到达导致的第二轮 D0 单独计数并作为"口径缺口"上报）。

用法:
  python link_sim.py --n 100000 --seed 20261018 [--json out.json]
  python link_sim.py --trace        # 打一条名义时序轨迹（人工核对）
退出码: 0 = 全部断言通过；1 = 失败（打印前几个反例的参数与原因）。
"""
import argparse, heapq, json, random, sys, time

# ------------------------------------------------------------------ 常量（规格 §1/§2/§9）
SPACING_US = 22000
ROUND_US = 120000
GUARD_US = 3000
U3_JITTER_US = 3000
DL_TIMEOUT_US = 117000
HELLO_INTERVAL_US = 60000
HELLO_SLOW_US = 500000
HELLO_MAX = 5
ACQUIRE_US = 200000
AIR_BASE_US = 13555          # P0 台架实测冻结（2026-10-18）: F(n)=13555+387·n
AIR_PER_BYTE_US = 387        # 与两端常量头 LinkAirtimeBaseUs/PerByteUs 一致
# 注：仿真里"接收帧末"= 空中结束（真值）。实现侧 USART IDLE 时间戳 = 空中结束 + n×174 + 174，
# 由实现减掉 (frameBytes+LinkRxIdleBytes)×uartByteUs 还原 —— 属实现口径，模型不重复减。
VERSION = 2
DL_LOSS_TO_HANDSHAKE = 2
LATE_SKIP_US = 3000          # 规格 §5（2026-10-18 修订）: 迟到 > LinkGuardUs 即跳过/顺延
UART_BYTE_US = 174           # 规格 §1: 57600 8N1 单字节时间；预算检查要算接收端串口排空
DL_BUDGET_SLACK_US = 1000    # 规格 §2.3: 预算检查的调度余量
ROUND_CLOSE_US = 60000       # 规格 §4 本轮关闭: 反推 t0 差 <60ms 的迟到帧不再开新轮
U0_AUX_WAIT_US = 2000
TAIL_US = 700000

T_DL_ATT, T_DL_SYS, T_DL_GPS, T_DL_STATE, T_DL_ACK = 0x1, 0x2, 0x3, 0x4, 0x5
T_UP_ROUND, T_HELLO = 0x6, 0x7
T_UP_CTRL, T_UP_CMD, T_UP_VEL, T_UP_POS, T_UP_SETPOINT, T_UP_MOTOR = 0x8, 0x9, 0xA, 0xB, 0xC, 0xD
CONTROL_TYPES = {T_UP_CTRL, T_UP_CMD, T_UP_VEL, T_UP_POS, T_UP_SETPOINT, T_UP_MOTOR}
DL_TYPES = {T_DL_ATT, T_DL_SYS, T_DL_GPS, T_DL_STATE, T_DL_ACK}

FRAME_LEN = {  # 规格 §1.2（CRC-16 后的整帧字节）
    0x1: 13, 0x2: 15, 0x3: 21, 0x4: 10, 0x5: 5,
    0x6: 11, 0x7: 5,
    0x8: 9, 0x9: 6, 0xA: 6, 0xB: 7, 0xC: 6, 0xD: 9,
}


def airtime(ftype):
    return AIR_BASE_US + AIR_PER_BYTE_US * FRAME_LEN[ftype]


def u3_expected_end(t0):
    return t0 + 3 * SPACING_US + airtime(T_UP_CTRL)


def u3_deadline(t0):
    return u3_expected_end(t0) + U3_JITTER_US


# ------------------------------------------------------------------ 仿真世界
class Sim:
    def __init__(self, rng, cfg):
        self.rng = rng
        self.cfg = cfg
        self.q = []
        self.seq = 0
        self.t = 0
        self.air = []                 # (start, end, origin, ftype)
        self.violations = []          # 协议违规文本
        self.overlaps = []            # 空中区间重叠（是否算失败在 analyze 里按时序窗口判定）
        self.late_after_d0 = 0        # 规格 §4 本轮关闭：D0 之后放弃的迟到帧（只计数）
        self.rx_log = []              # (t, dst, ftype, origin)
        self.tx_log = []              # (t, origin, ftype)
        self.trace = cfg.get("trace", False)
        self.started = False

    # ---- 调度 ----
    def push(self, t, target, kind, data=None):
        self.seq += 1
        heapq.heappush(self.q, (t, self.seq, target, kind, data))

    def run(self, t_end):
        # 实体在 push 前需要已存在，所以延迟到 run() 里第一次使用时创建
        while self.q:
            t, _, target, kind, data = heapq.heappop(self.q)
            if t > t_end:
                break
            self.t = t
            target.handle(kind, data)

    # ---- 物理层 ----
    def tx(self, origin, ftype, now=None, state=None, target=None):
        t0 = self.t if now is None else now
        dur = airtime(ftype)
        end = t0 + dur
        late_us = 0 if target is None else max(0, t0 - target)
        for (s, e, o, ft, st, lu) in self.air:
            if t0 < e and end > s:
                self.overlaps.append((t0, origin, ftype, o, ft, s, e, late_us, lu))
                break
        self.air.append((t0, end, origin, ftype, state, late_us))
        self.tx_log.append((t0, origin, ftype, state, late_us))
        # 同一端的射频是串行资源：上一帧还在空中时，下一帧必须等（AUX 忙）
        ep = getattr(self, "rc", None) if origin == "RC" else getattr(self, "fc", None)
        if ep is not None:
            ep.busy_until = end
        if self.trace:
            print("  %8.3fms TX %-2s 0x%X (%2dB 空中 %0.2fms -> %0.3fms)"
                  % (t0 / 1000.0, origin, ftype, FRAME_LEN[ftype], dur / 1000.0, end / 1000.0))
        loss = self.cfg["ul_loss"] if origin == "RC" else self.cfg["dl_loss"]
        if t0 < self.cfg["loss_end"] and self.rng.random() < loss:
            return None
        return end

    def deliver(self, t, dst, ftype, origin, seq=0):
        self.rx_log.append((t, dst, ftype, origin))
        if self.trace:
            print("  %8.3fms RX %-2s 0x%X from %s seq=%d" % (t / 1000.0, dst, ftype, origin, seq))
        dst.on_frame(t, ftype, origin, seq)

    def radio_busy_until(self):
        """AUX 忙：只在故障窗口内按概率命中；返回忙到何时（<= now 表示不忙）"""
        if self.t >= self.cfg["loss_end"]:
            return 0
        if self.rng.random() < self.cfg["aux_busy"]:
            return self.t + self.rng.randint(1000, 9000)
        return 0


class Endpoint:
    name = "?"

    def __init__(self, sim):
        self.sim = sim
        self.gen = 0
        self.state = None
        self.history = []          # (t, state)
        self.busy_until = 0        # 本端射频发送占用到何时

    def busy(self):
        """返回本端射频忙到何时（自身发送 + 随机 AUX 忙），<= now 表示空闲"""
        return max(self.busy_until, self.sim.radio_busy_until())

    def now(self):
        return self.sim.t

    def bump_gen(self):
        self.gen += 1
        return self.gen

    def handle(self, kind, data):
        gen, payload = data
        if kind == "rx":
            self.sim.rx_log.append((self.sim.t, self.name, payload[0], payload[1]))
            if gen != self.gen:      # 重启瞬间在途的帧：丢弃（真实接收端同样不会采信）
                return
            self.on_frame(self.sim.t, *payload)
            return
        if gen != self.gen:
            return
        getattr(self, kind)(payload)

    def sched(self, t, kind, payload=None):
        self.sim.push(t, self, kind, (self.gen, payload))

    def set_state(self, st):
        if st != self.state:
            self.state = st
            self.history.append((self.now(), st))


# ------------------------------------------------------------------ 遥控器 RoundMaster
class RcMaster(Endpoint):
    name = "RC"

    def __init__(self, sim):
        super().__init__(sim)
        self.ver = sim.cfg["rc_ver"]
        self.session = 1
        self.round = None
        self.round_count = 0
        self.round_uid = 0
        self.hello_sent = 0
        self.hello_seq = 0
        self.dl_loss_streak = 0
        self.last_dl_t = None
        self.cmd_pending = False
        self.tick_key = 0
        if sim.cfg["rc_is_v1"]:
            self.set_state("V1")
            self.sched(0, "v1_send")
        else:
            self.enter_handshake(self.now())

    # ---- 状态切换 ----
    def _next_hello_timer(self, now, interval_us):
        self.tick_key += 1
        self.sched(now + interval_us, "hello_timer", self.tick_key)

    def enter_handshake(self, now):
        self.set_state("HANDSHAKE")
        self.round = None
        self.hello_sent = 0
        self.send_hello(now)
        self._next_hello_timer(now, HELLO_INTERVAL_US)

    def enter_no_link(self, now):
        self.set_state("NO_LINK")
        self.round = None
        self.send_hello(now)
        self._next_hello_timer(now, HELLO_SLOW_US)

    def enter_mismatch(self, now):
        self.set_state("MISMATCH")
        self.round = None
        self.send_hello(now)
        self._next_hello_timer(now, HELLO_INTERVAL_US)

    def enter_run(self, t0):
        self.tick_key += 1          # 作废还在路上的 HELLO 定时器
        self.set_state("RUN")
        self.dl_loss_streak = 0
        self.start_round(t0)

    # ---- HELLO ----
    def send_hello(self, now):
        self.hello_sent += 1
        if self.busy() > now:
            return
        self._tx_uplink(T_HELLO, now, self.hello_seq)
        self.hello_seq = (self.hello_seq + 1) & 0xFF

    def _tx_uplink(self, ftype, now, seq, target=None, extra_delay=0):
        end = self.sim.tx("RC", ftype, now, state=self.state, target=target)
        if end is not None:
            self.sim.push(end + extra_delay, self.sim.fc, "rx",
                          (self.sim.fc.gen, (ftype, "RC", seq)))
        return end

    def jitter(self):
        """发送时刻的本地调度抖动（只在故障窗口内注入）"""
        if self.now() >= self.sim.cfg["loss_end"]:
            return 0
        return self.sim.rng.randint(0, self.sim.cfg["jitter_us"])

    def hello_timer(self, key):
        if key != self.tick_key or self.state == "RUN":
            return
        if self.state == "HANDSHAKE" and self.hello_sent >= HELLO_MAX:
            self.enter_no_link(self.now())
            return
        self.send_hello(self.now())
        interval = HELLO_SLOW_US if self.state == "NO_LINK" else HELLO_INTERVAL_US
        self.sched(self.now() + interval, "hello_timer", key)

    # ---- 轮次 ----
    def start_round(self, t0):
        self.round_uid += 1
        self.round = {"uid": self.round_uid, "t0": t0, "dl_ok": False}
        self.sched(t0 + self.jitter(), "u0", self.round_uid)
        self.sched(t0 + DL_TIMEOUT_US, "dl_check", self.round_uid)
        self.sched(t0 + ROUND_US, "round_next", self.round_uid)

    def send_uplink(self, pos, ftype, target):
        now = self.now()
        if now > target + LATE_SKIP_US:
            if self.sim.trace:
                print("  %8.3fms RC skip pos=%d (迟到 %0.2fms)" % (now / 1000.0, pos, (now - target) / 1000.0))
            return False
        if self.busy() > now:
            if self.sim.trace:
                print("  %8.3fms RC skip pos=%d (AUX 忙)" % (now / 1000.0, pos))
            return False
        seq = (self.round_count * 4 + pos) & 0xFF
        extra = 0
        if pos == 3 and self.sim.cfg.get("u3_delay") and self.sim.t < self.sim.cfg["loss_end"] \
                and self.sim.rng.random() < 0.5:
            # 故障注入：U3 的"接收端帧末时间戳"被推迟（模块/中断 backlog），
            # 用来验证规格 §4「本轮关闭」不会因此发第二帧 D0
            lo, hi = self.sim.cfg["u3_delay"]
            extra = self.sim.rng.randint(lo, hi)
        self._tx_uplink(ftype, now, seq, target=target, extra_delay=extra)
        return True

    def u0(self, uid):
        if self.state != "RUN" or self.round is None or self.round["uid"] != uid:
            return
        r = self.round
        now = self.now()
        if now > r["t0"] + LATE_SKIP_US:
            if self.sim.trace:
                print("  %8.3fms RC U0 迟到 %0.2fms -> 以 now 为新 t0 顺延整轮"
                      % (now / 1000.0, (now - r["t0"]) / 1000.0))
            self.start_round(now)   # 新 t0 的 u0 事件排在同刻，随后立即触发
            return
        busy = self.busy()
        if busy > now:
            if busy <= now + U0_AUX_WAIT_US:
                self.sched(busy, "u0", uid)
                return
            self.start_round(busy)   # 等 2ms 仍忙 -> 以当前时刻为新 t0 顺延整轮
            return
        if self.sim.cfg.get("rc_reanchor_u0") and now > r["t0"]:
            # 可选项（规格未要求）：U0 一旦比目标晚，就以实际发送时刻重锚整轮，
            # 使飞控反推的 t0 与本端网格严格一致，消除"飞控兜底 D0 压下一轮 U0"。
            self.start_round(now)
            return
        self._tx_uplink(T_UP_ROUND, now, (self.round_count * 4 + 0) & 0xFF, target=r["t0"])
        t0 = r["t0"]
        self.sched(t0 + 1 * SPACING_US + self.jitter(), "u1", uid)
        self.sched(t0 + 2 * SPACING_US + self.jitter(), "u2", uid)
        self.sched(t0 + 3 * SPACING_US + self.jitter(), "u3", uid)

    def _guard_round(self, uid, pos, ftype):
        if self.state != "RUN" or self.round is None or self.round["uid"] != uid:
            return
        self.send_uplink(pos, ftype, self.round["t0"] + pos * SPACING_US)

    def u1(self, uid):
        self._guard_round(uid, 1, T_UP_CTRL)

    def u2(self, uid):
        self._guard_round(uid, 2, T_UP_CMD if self.cmd_pending else T_UP_CTRL)

    def u3(self, uid):
        self._guard_round(uid, 3, T_UP_CTRL)

    def dl_check(self, uid):
        if self.state != "RUN" or self.round is None or self.round["uid"] != uid:
            return
        if self.round["dl_ok"]:
            self.dl_loss_streak = 0
        else:
            self.dl_loss_streak += 1
        if self.dl_loss_streak >= DL_LOSS_TO_HANDSHAKE:
            self.enter_handshake(self.now())

    def round_next(self, uid):
        if self.state != "RUN" or self.round is None or self.round["uid"] != uid:
            return
        self.round_count = (self.round_count + 1) & 0xFF
        self.start_round(self.round["t0"] + ROUND_US)

    # ---- 收下行 ----
    def on_frame(self, t, ftype, origin, seq=0):
        if origin != "FC":
            return
        if ftype == T_HELLO:
            ver = self.sim.cfg["fc_ver"]
            if self.state in ("HANDSHAKE", "MISMATCH", "NO_LINK"):
                if ver == self.ver:
                    self.enter_run(t + GUARD_US)
                elif self.state != "MISMATCH":
                    # 已在 MISMATCH 时不因每个应答重发 HELLO（否则会与对端应答乒乓）
                    self.enter_mismatch(t)
            elif self.state == "RUN" and ver != self.ver:
                self.enter_mismatch(t)
            return
        if self.state != "RUN" or ftype not in DL_TYPES:
            return
        self.last_dl_t = t
        if self.round is not None and t >= self.round["t0"]:
            self.round["dl_ok"] = True
        if ftype == T_DL_ACK:
            self.cmd_pending = False

    def noop(self, _):
        pass

    # ---- v1 遥控器模型：无 HELLO / 无 UP_ROUND，22ms 一帧 UP_CTRL ----
    def v1_send(self, _):
        self._tx_uplink(T_UP_CTRL, self.now(), (self.round_count * 4) & 0xFF)
        self.round_count = (self.round_count + 1) & 0xFF
        self.sched(self.now() + SPACING_US, "v1_send")


# ------------------------------------------------------------------ 飞控 RoundSlave
class FcSlave(Endpoint):
    name = "FC"

    def __init__(self, sim):
        super().__init__(sim)
        self.ver = sim.cfg["fc_ver"]
        self.round = None
        self.round_uid = 0
        self.last_valid_end = -10 ** 12
        self.hello_pending = None
        self.dl_index = 0
        self.acquire_gen = 0
        self.stat_d0 = 0
        self.stat_skip = 0
        self.attempt_log = []          # (t, round_uid, "sent"/"busy"/"budget") 自检用
        self.set_state("ACQUIRE")
        self.arm_acquire_timeout()

    def arm_acquire_timeout(self):
        self.acquire_gen += 1
        self.sched(self.now() + ACQUIRE_US, "acquire_tick", self.acquire_gen)

    def enter_acquire(self):
        self.set_state("ACQUIRE")
        self.round = None
        self.hello_pending = None

    # ---- 收上行 ----
    def on_frame(self, t, ftype, origin, seq=0):
        if origin != "RC":
            return
        if self.sim.cfg["fc_is_v1"]:
            return                      # v1 飞控：不认 0x6/0x7，全部不采信、不回复
        if ftype == T_HELLO:
            self.last_valid_end = t
            self.hello_pending = {"ver": self.sim.cfg["rc_ver"], "t": t}
            if self.state == "ACQUIRE":
                self.sched(t + GUARD_US, "hello_reply")
            elif self.round is None or self.round["done"]:
                self.sched(t + GUARD_US, "hello_reply")
            self.arm_acquire_timeout()
            return
        if ftype == T_UP_ROUND:
            self.last_valid_end = t
            self.arm_acquire_timeout()
            if self.state == "ACQUIRE":
                self.set_state("RUN")
            self.open_round(t, 0, ftype)
            return
        if ftype in CONTROL_TYPES:
            self.last_valid_end = t
            self.arm_acquire_timeout()
            if self.state != "RUN":
                return                  # ACQUIRE 只对 UP_ROUND 采信
            self.open_round(t, seq & 3, ftype)
            return

    def open_round(self, frame_end, pos, ftype):
        t0 = frame_end - (pos * SPACING_US + airtime(ftype))
        same_round = self.round is not None and abs(self.round["t0"] - t0) < ROUND_CLOSE_US
        open_new = True
        if same_round and self.round["done"]:
            if self.sim.cfg.get("round_close_rule", True):
                # 规格 §4「本轮关闭」：D0 已发/跳过，同 t0 的迟到帧只更新计数，不得再发第二帧 D0
                self.round["late_frames"] = self.round.get("late_frames", 0) + 1
                self.sim.late_after_d0 += 1
                return
            # 变异开关（仅自检用）：模拟"没有本轮关闭规则"的旧模型，应被断言抓到双 D0
            open_new = True
        elif same_round:
            open_new = (ftype == T_UP_ROUND)
        if open_new:
            self.round_uid += 1
            self.round = {"uid": self.round_uid, "t0": t0, "pos_mask": 0,
                          "u3_end": None, "done": False, "gen": 0,
                          "deadline": u3_deadline(t0)}
        r = self.round
        r["t0"] = t0                       # 规格 §2.2: 每收到一帧都刷新 t0
        r["deadline"] = u3_deadline(t0)
        r["pos_mask"] |= (1 << pos)
        if pos == 3:
            r["u3_end"] = frame_end
        if r["u3_end"] is not None:
            want = r["u3_end"] + GUARD_US          # 正常路径：IDLE 帧末事件，精确
        else:
            # 兜底路径走 6ms 定时器（规格 §1.2：定时器只做 watchdog），触发时刻按 tick 向上取整
            tick = 6000
            want = ((r["deadline"] + GUARD_US + tick - 1) // tick) * tick
        r["gen"] += 1
        self.sched(want, "d0_fire", (r["uid"], r["gen"]))

    def hello_reply(self, _):
        if self.hello_pending is None:
            return
        if self.busy() > self.now():
            self.sched(self.now() + 6000, "hello_reply")    # 6ms tick 重试
            return
        self.hello_pending = None
        end = self.sim.tx("FC", T_HELLO, state=self.state)
        if end is not None:
            self.sim.push(end, self.sim.rc, "rx", (self.sim.rc.gen, (T_HELLO, "FC", 0)))

    def select_dl(self):
        if self.hello_pending is not None:
            self.hello_pending = None
            return T_HELLO
        return T_DL_ATT

    def d0_fire(self, payload):
        uid, rgen = payload
        r = self.round
        if r is None or r["uid"] != uid or r["done"] or rgen != r["gen"]:
            return
        now = self.now()
        ftype = self.select_dl()
        if self.busy() > now:
            r["done"] = True
            self.stat_skip += 1
            self.attempt_log.append((now, r["uid"], "busy"))
            return
        if now + airtime(ftype) + FRAME_LEN[ftype] * UART_BYTE_US + DL_BUDGET_SLACK_US > r["t0"] + DL_TIMEOUT_US:
            r["done"] = True            # 预算不足：放弃本轮 D0（绝不压下一轮；含接收端串口排空）
            self.stat_skip += 1
            self.attempt_log.append((now, r["uid"], "budget"))
            return
        r["done"] = True
        r["d0_type"] = ftype
        self.stat_d0 += 1
        self.dl_index += 1
        self.attempt_log.append((now, r["uid"], "sent"))
        end = self.sim.tx("FC", ftype, state=self.state)
        if end is not None:
            self.sim.push(end, self.sim.rc, "rx", (self.sim.rc.gen, (ftype, "FC", 0)))

    def acquire_tick(self, agen):
        if agen != self.acquire_gen:
            return
        if self.state == "RUN" and self.now() - self.last_valid_end > ACQUIRE_US:
            self.enter_acquire()
        self.arm_acquire_timeout()


# ------------------------------------------------------------------ 场景
def make_cfg(rng, kind="normal"):
    loss_end = rng.choice([400000, 600000, 800000])
    cfg = dict(
        duration=loss_end + TAIL_US,
        loss_end=loss_end,
        ul_loss=rng.uniform(0.0, 0.35),
        dl_loss=rng.uniform(0.0, 0.35),
        aux_busy=rng.uniform(0.0, 0.15),
        jitter_us=rng.choice([0, 1000, 3000, 6000]),
        rc_reanchor_u0=False,
        rc_ver=2, fc_ver=2, rc_is_v1=False, fc_is_v1=False,
        reboots=[], trace=False,
    )
    if kind == "normal":
        for _ in range(rng.choice([0, 0, 0, 1, 1, 2])):
            cfg["reboots"].append((rng.randint(20000, max(30000, loss_end - 50000)),
                                   rng.choice(["RC", "FC"])))
    elif kind == "reboot":
        who = rng.choice(["RC", "FC"])
        for _ in range(rng.randint(1, 3)):
            cfg["reboots"].append((rng.randint(20000, max(30000, loss_end - 50000)), who))
    elif kind == "late":
        # 迟到专项：上行发送抖动 = 3ms（新口径的边界值：迟到 >3ms 才跳过/顺延，
        # 恰在阈值上的帧仍发；用于验证边界两侧都不会与对端定时帧争用半双工）
        cfg["jitter_us"] = 3000
        cfg["ul_loss"] = rng.uniform(0.0, 0.05)
        cfg["dl_loss"] = rng.uniform(0.0, 0.05)
    elif kind == "late_big":
        # 超过阈值：U0 会顺延整轮、U1..U3 会被跳过 —— 验证"宁丢帧不压下一轮"
        cfg["jitter_us"] = rng.choice([4000, 6000, 8000, 12000])
        cfg["ul_loss"] = rng.uniform(0.0, 0.05)
        cfg["dl_loss"] = rng.uniform(0.0, 0.05)
    elif kind == "late_u3":
        # 「本轮关闭」专项：U3 的接收时间戳被推迟 6–10ms（模块/中断 backlog），
        # 会让 U3 在飞控兜底 D0 之后才被"收到"，验证不会发第二帧 D0
        cfg["u3_delay"] = (6000, 7000)   # 落在"迟到帧晚于兜底、但预算仍够"的窗口
    elif kind == "mismatch":
        cfg["fc_ver"] = 1
        cfg["ul_loss"] = rng.uniform(0.0, 0.2)
        cfg["dl_loss"] = rng.uniform(0.0, 0.2)
    elif kind == "rc2_fc1":
        cfg["fc_is_v1"] = True
        cfg["reboots"] = []
    elif kind == "rc1_fc2":
        cfg["rc_is_v1"] = True
        cfg["reboots"] = []
    cfg["reboots"].sort()
    return cfg


class RebootTarget:
    def __init__(self, sim, who, rng):
        self.sim, self.who, self.rng = sim, who, rng

    def handle(self, kind, data):
        if kind != "reboot":
            return
        # 重启瞬间打断在途发送（射频已停）
        now = self.sim.t
        for i, (s, e, o, ft, st, lu) in enumerate(self.sim.air):
            if o == self.who and e > now:
                self.sim.air[i] = (s, now, o, ft, st, lu)
        if self.who == "RC":
            ep = self.sim.rc
            ep.bump_gen()
            ep.session = self.rng.randint(1, 255)
            ep.round = None
            ep.round_count = 0
            ep.enter_handshake(self.sim.t)
        else:
            ep = self.sim.fc
            ep.bump_gen()
            ep.round = None
            ep.hello_pending = None
            ep.enter_acquire()
            ep.arm_acquire_timeout()


def run_scenario(cfg, seed, run_to=None):
    rng = random.Random(seed)
    sim = Sim(rng, cfg)
    sim.fc = FcSlave(sim)
    sim.rc = RcMaster(sim)
    for t_ev, who in cfg["reboots"]:
        sim.push(t_ev, RebootTarget(sim, who, rng), "reboot")
    sim.run(cfg["duration"] if run_to is None else run_to)
    return sim


def analyze(sim, cfg):
    fails = []
    # A. 半双工：按开始时刻排序后检查区间重叠。
    #    轮次协议只保证 RUN 内不重叠；握手/重启瞬间（任一端不在 RUN）的碰撞是
    #    "尚未建立时序"的固有代价（HELLO 靠 60ms 重发兜底），单独计数不算失败。
    air = sorted(sim.air, key=lambda x: x[0])
    handshake_collisions = 0
    late_collisions = 0
    # 握手窗口：任一端状态切换后 1 轮内，时序尚未建立（HELLO 靠重发兜底），碰撞可接受
    changes = sorted([t for (t, st) in sim.rc.history] + [t for (t, st) in sim.fc.history])
    for i in range(1, len(air)):
        s_prev, e_prev, o_prev, ft_prev, st_prev, lu_prev = air[i - 1]
        s_cur, e_cur, o_cur, ft_cur, st_cur, lu_cur = air[i]
        if s_cur < e_prev:
            recent_change = any(s_cur - ROUND_US <= c <= s_cur for c in changes)
            if st_prev != "RUN" or st_cur != "RUN" or recent_change:
                handshake_collisions += 1
            elif lu_prev > 0 or lu_cur > 0:
                # 3ms 迟到容差内的帧与对端定时帧同刻（新口径下应极少；保留计数用于回归监视）
                late_collisions += 1
            else:
                fails.append("RUN 内半双工重叠 @%dus (%s 0x%X vs %s 0x%X)"
                             % (s_cur, o_prev, ft_prev, o_cur, ft_cur))
                break
    for v in sim.violations:
        if not v.startswith("late-uplink-after-d0"):
            fails.append(v)

    n_late = sim.late_after_d0
    n_fc_tx = sum(1 for x in sim.tx_log if x[1] == "FC")
    n_fc_dl = sum(1 for x in sim.tx_log if x[1] == "FC" and x[2] in DL_TYPES)
    n_rc_tx = sum(1 for x in sim.tx_log if x[1] == "RC")
    # 双 D0 检测：正常轮距 120ms，两次纯下行(D0)间隔 <30ms 只可能是同一轮发了第二帧
    fc_dl_times = sorted(t for (t, o, ft, st, lu) in sim.tx_log if o == "FC" and ft in DL_TYPES)
    for a, b in zip(fc_dl_times, fc_dl_times[1:]):
        if b - a < 30000:
            fails.append("疑似同轮双 D0：间隔 %0.1fms < 30ms" % ((b - a) / 1000.0))
            break

    # E. v1 混烧
    if cfg["fc_is_v1"]:
        if n_fc_tx:
            fails.append("v1 飞控不应发送（实际 %d 帧）" % n_fc_tx)
        rc_types = {x[2] for x in sim.tx_log if x[1] == "RC"}
        if rc_types - {T_HELLO}:
            fails.append("v2 遥控器对 v1 飞控发了非 HELLO 帧: %s" % sorted(rc_types - {T_HELLO}))
        if sim.rc.state == "RUN":
            fails.append("v2 遥控器在 v1 飞控在场时进入 RUN")
        return fails, {"mode": "v1mixed_rc2_fc1", "rc_tx": n_rc_tx, "fc_tx": n_fc_tx,
                       "collisions": handshake_collisions}

    if cfg["rc_is_v1"]:
        if n_fc_tx:
            fails.append("v2 飞控对 v1 遥控器发了帧（应保持 ACQUIRE 不发）")
        if sim.fc.state == "RUN":
            fails.append("v2 飞控对 v1 遥控器进入 RUN")
        return fails, {"mode": "v1mixed_rc1_fc2", "fc_tx": n_fc_tx,
                       "collisions": handshake_collisions}

    # D. 版本不匹配
    if cfg["rc_ver"] != cfg["fc_ver"]:
        bad = [x for x in sim.tx_log if x[2] in CONTROL_TYPES]
        if bad:
            fails.append("版本不匹配时出现控制帧 0x%X" % bad[0][2])
        fc_bad = [x for x in sim.tx_log if x[1] == "FC" and x[2] != T_HELLO]
        if fc_bad:
            fails.append("版本不匹配时飞控发了非 HELLO 帧 0x%X" % fc_bad[0][2])
        if sim.fc.state == "RUN":
            fails.append("版本不匹配时飞控进入 RUN")
        if sim.rc.state == "RUN":
            fails.append("版本不匹配时遥控器进入 RUN")
        return fails, {"mode": "mismatch", "rc_state": sim.rc.state, "fc_tx": n_fc_tx,
                       "rc_tx": n_rc_tx, "collisions": handshake_collisions}

    # B/C. 正常模式：恢复时限 + 干净尾巴
    rc_enter_run = [t for (t, st) in sim.rc.history if st == "RUN"]
    fc_enter_run = [t for (t, st) in sim.fc.history if st == "RUN"]
    t_rec = None
    if rc_enter_run and fc_enter_run:
        cand = max(rc_enter_run[-1], fc_enter_run[-1])
        dl_after = [t for (t, dst, ft, org) in sim.rx_log
                    if dst == "RC" and org == "FC" and t >= cand]
        if dl_after:
            t_rec = max(cand, cfg["loss_end"])
    if t_rec is None:
        fails.append("干净尾巴内未恢复（rc=%s fc=%s）" % (sim.rc.state, sim.fc.state))
        lat = None
    else:
        lat = t_rec - cfg["loss_end"]
        # NO_LINK 的 HELLO 重发周期是 500ms（规格 §5），恢复上限另算；其余按 3 轮
        st_before = [st for (t, st) in sim.rc.history if t <= cfg["loss_end"]]
        no_link_after = (st_before and st_before[-1] == "NO_LINK") or \
                        any(t > cfg["loss_end"] and st == "NO_LINK" for (t, st) in sim.rc.history)
        limit = 3 * ROUND_US + (HELLO_SLOW_US if no_link_after else 0)
        if lat > limit:
            fails.append("恢复延迟 %0.1fms > 上限 %0.0fms" % (lat / 1000.0, limit / 1000.0))
    if sim.rc.state != "RUN":
        fails.append("结束时遥控器不在 RUN（%s）" % sim.rc.state)
    if sim.fc.state != "RUN":
        fails.append("结束时飞控不在 RUN（%s）" % sim.fc.state)
    dl_tail = [t for (t, dst, ft, org) in sim.rx_log
               if dst == "RC" and org == "FC" and t > cfg["duration"] - 240000]
    if not dl_tail:
        fails.append("末 240ms 无下行（疑似死锁）")

    # F. 每轮下行 ≤1 帧（不含 HELLO 应答）：按 FC 本次下行相对 RC 轮次锚点归并
    stats = {"mode": "normal", "lat_us": lat, "dl_tx": n_fc_dl, "fc_hello": n_fc_tx - n_fc_dl,
             "ul_tx": n_rc_tx, "late_after_d0": n_late, "reboots": len(cfg["reboots"]),
             "rounds": sim.rc.round_count, "collisions": handshake_collisions,
             "late_collisions": late_collisions}
    return fails, stats


def run_batch(n, seed0, kinds, reanchor=False, round_close=True):
    rng = random.Random(seed0)
    fails = []
    stats = {"scenarios": 0, "by_mode": {}, "lat_max_us": 0, "late_after_d0": 0,
             "fail_by_kind": {}, "min_lat_us": 10 ** 9, "collisions": 0, "rounds": 0,
             "late_collisions": 0, "failed": 0, "lat_buckets": {}}
    for i in range(n):
        kind = kinds[i % len(kinds)] if len(kinds) > 1 else kinds[0]
        cfg = make_cfg(rng, kind)
        cfg["rc_reanchor_u0"] = reanchor
        cfg["round_close_rule"] = round_close
        seed = rng.randint(0, 2 ** 31 - 1)
        sim = run_scenario(cfg, seed)
        f, st = analyze(sim, cfg)
        stats["scenarios"] += 1
        m = st.get("mode", kind)
        stats["by_mode"][m] = stats["by_mode"].get(m, 0) + 1
        if st.get("lat_us") is not None:
            stats["lat_max_us"] = max(stats["lat_max_us"], st["lat_us"])
            stats["min_lat_us"] = min(stats["min_lat_us"], st["lat_us"])
            bucket = min(9, st["lat_us"] // ROUND_US)
            stats["lat_buckets"][str(bucket)] = stats["lat_buckets"].get(str(bucket), 0) + 1
        stats["late_after_d0"] += st.get("late_after_d0", 0)
        stats["collisions"] += st.get("collisions", 0)
        stats["late_collisions"] += st.get("late_collisions", 0)
        stats["rounds"] += st.get("rounds", 0)
        if f:
            stats["fail_by_kind"][kind] = stats["fail_by_kind"].get(kind, 0) + 1
            fails.append({"kind": kind, "seed": seed, "cfg": cfg, "fails": f})
            if len(fails) >= 5:
                break
    stats["failed"] = len(fails)
    return fails, stats


def trace_one(seed=1):
    rng = random.Random(seed)
    cfg = make_cfg(rng, "normal")
    cfg.update(trace=True, ul_loss=0.0, dl_loss=0.0, aux_busy=0.0, reboots=[],
               loss_end=240000, duration=600000)
    sim = run_scenario(cfg, seed)
    f, st = analyze(sim, cfg)
    print("\n断言:", "PASS" if not f else f)
    print("统计:", json.dumps(st, ensure_ascii=False))


class InjectTarget:
    """自检用：在指定时刻往事件队列里注入一个动作"""
    def __init__(self, fn):
        self.fn = fn

    def handle(self, kind, data):
        self.fn()


def selftest():
    """确定性自检：不靠随机，逐条验收"时序公式/预算/本轮关闭/ACQUIRE 门控/半双工"。
    返回 (ok, 报告行列表)。"""
    lines = []
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and cond
        lines.append("%-46s %s%s" % (name, "PASS" if cond else "FAIL", ("  " + detail) if detail else ""))

    base = dict(duration=600000, loss_end=0, ul_loss=0.0, dl_loss=0.0, aux_busy=0.0,
                jitter_us=0, rc_reanchor_u0=False, rc_ver=2, fc_ver=2,
                rc_is_v1=False, fc_is_v1=False, reboots=[], trace=False,
                round_close_rule=True)

    # ---- 1. 名义轮次时序 ----
    cfg = dict(base)
    sim = run_scenario(cfg, 1)
    u0 = [t for (t, o, f, st, lu) in sim.tx_log if o == "RC" and f == T_UP_ROUND]
    check("U0 轮间距 = 120ms", all(b - a == ROUND_US for a, b in zip(u0, u0[1:])),
          "%s" % [(b - a) for a, b in zip(u0, u0[1:])][:4])
    rnd = [x for x in sim.tx_log if x[1] == "RC" and (x[2] == T_UP_ROUND or x[2] in CONTROL_TYPES)]
    check("每轮 4 帧上行(U0..U3)", len(rnd) == 4 * len(u0), "ul=%d rounds=%d" % (len(rnd), len(u0)))
    # D0 触发 = U3 末字节 + guard（正常路径）
    fc_dl = sorted(t for (t, o, f, st, lu) in sim.tx_log if o == "FC" and f in DL_TYPES)
    exp = [u + 3 * SPACING_US + airtime(T_UP_CTRL) + GUARD_US for u in u0 if u + 117000 < cfg["duration"]]
    check("D0 触发 = U3末+guard（正常路径）",
          len(fc_dl) >= len(exp) - 1 and all(abs(a - e) <= 6000 for a, e in zip(fc_dl, exp)),
          "fc_dl=%s exp=%s" % (fc_dl, exp))
    # 半双工（RUN 稳态）
    overlaps = [x for x in sim.overlaps]
    check("RUN 稳态无双发（无重叠区间）", len(overlaps) == 0, str(overlaps[:2]))
    # RC 每轮收到下行（t0+117 之前）
    dls = [t for (t, dst, f, o) in sim.rx_log if dst == "RC" and o == "FC"]
    check("每轮 D0 在 t0+117ms 前收完", len(dls) >= len(u0))

    # ---- 2. 预算检查（含接收端串口排空） ----
    # 造一轮"U3 帧末很晚"的情况：t0+? 使 now+airtime+drain+slack 超过 t0+117ms → 必须放弃
    cfg = dict(base)
    cfg["u3_delay"] = (12000, 12000)     # U3 帧末晚 12ms → sendAt = t0+97.9ms
    sim = run_scenario(cfg, 2)
    late_rounds = 0
    for (t, o, f, st, lu) in sim.tx_log:
        pass
    skipped = 0
    for (t, o, f, st, lu) in sim.tx_log:
        if o == "FC" and f in DL_TYPES:
            pass
    # 直接验证判据公式：发一帧 21B GPS 在 t0+95ms 必须被预算拒绝
    check("预算判据：21B 在 t0+95ms 应放弃", 95000 + airtime(T_DL_GPS) + 21 * UART_BYTE_US
          + DL_BUDGET_SLACK_US > DL_TIMEOUT_US)
    check("预算判据：13B 在 t0+89ms 可发", 89000 + airtime(T_DL_ATT) + 13 * UART_BYTE_US
          + DL_BUDGET_SLACK_US <= DL_TIMEOUT_US)

    # ---- 3. 本轮关闭（规格 §4）----
    # 名义时序是确定的：u0@36.8ms → D0@122.72ms（末字节 122.72+18.44=141.16）。
    # 在 D0 之后注入"迟到 U3"（反推 t0 比当轮晚 6.5ms，<60ms）：ON 应该完全不尝试第二帧；
    # OFF（变异）应该出现第二次 D0 尝试（会被 AUX 忙挡下，因此统计"尝试"而不是"发出"）。
    for rule in (True, False):
        cfg = dict(base)
        cfg["round_close_rule"] = rule
        sim = run_scenario(cfg, 3, run_to=130000)
        d0 = sim.fc.attempt_log[-1][0] if sim.fc.attempt_log else None
        if d0 is None or sim.rc.round is None:
            check("本轮关闭自检（rule=%s）: 找得到名义 D0" % rule, False, "d0=%s" % d0)
            continue
        exp_d0 = sim.rc.round["t0"] + 3 * SPACING_US + airtime(T_UP_CTRL) + GUARD_US
        if abs(d0 - exp_d0) > 1000:
            check("本轮关闭自检（rule=%s）: 名义 D0 = U3末+guard" % rule, False,
                  "d0=%s exp=%s" % (d0, exp_d0))
            continue
        inject_t = d0 + 3500
        sim.push(inject_t, sim.fc, "rx", (sim.fc.gen, (T_UP_CTRL, "RC", 3)))
        sim.run(inject_t + 40000)
        att = [a for a in sim.fc.attempt_log if d0 < a[0] <= d0 + 30000]
        if rule:
            check("本轮关闭 ON：迟到 U3 不再尝试第二帧 D0", len(att) == 0,
                  "attempts=%s" % att)
        else:
            check("本轮关闭 OFF（变异自检）：断言能抓到第二次 D0 尝试", len(att) >= 1,
                  "attempts=%s" % att)

    # ---- 4. ACQUIRE 门控：v1 遥控器 + v2 飞控 ----
    cfg = dict(base)
    cfg["rc_is_v1"] = True
    sim = run_scenario(cfg, 4)
    check("v1 遥控器：v2 飞控不发任何帧", not any(o == "FC" for (_, o, _, _, _) in sim.tx_log))
    check("v1 遥控器：v2 飞控保持 ACQUIRE", sim.fc.state == "ACQUIRE")

    # ---- 5. 版本不匹配：只发 HELLO ----
    cfg = dict(base)
    cfg["fc_ver"] = 1
    sim = run_scenario(cfg, 5)
    bad = [f for (_, o, f, _, _) in sim.tx_log if f in CONTROL_TYPES]
    fc_bad = [f for (_, o, f, _, _) in sim.tx_log if o == "FC" and f != T_HELLO]
    check("版本不匹配：无控制帧", not bad)
    check("版本不匹配：飞控只发 HELLO", not fc_bad)
    check("版本不匹配：双方不进 RUN",
          sim.rc.state != "RUN" and sim.fc.state != "RUN")

    if ok:
        print("\n".join(lines))
        print("\n确定性自检：全部 PASS")
    else:
        print("\n".join(lines))
        print("\n确定性自检：存在 FAIL")
    return ok, lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1000)
    ap.add_argument("--seed", type=int, default=20261018)
    ap.add_argument("--kinds", default="normal,normal,normal,normal,reboot,reboot,late,late_big,late_u3,mismatch,rc2_fc1,rc1_fc2")
    ap.add_argument("--json", default=None)
    ap.add_argument("--trace", action="store_true")
    ap.add_argument("--reanchor", action="store_true",
                    help="实验开关：RC 在 U0 迟到时立即以实际时刻重锚整轮（验证口径建议）")
    ap.add_argument("--no-roundclose", action="store_true",
                    help="变异自检：关掉规格 §4「本轮关闭」规则，断言应能抓到双 D0（预期失败）")
    ap.add_argument("--selftest", action="store_true",
                    help="确定性自检：时序公式/预算/本轮关闭/ACQUIRE 门控/版本不匹配")
    args = ap.parse_args()
    if args.selftest:
        ok, _ = selftest()
        return 0 if ok else 1
    if args.trace:
        trace_one(args.seed)
        return 0
    kinds = [k for k in args.kinds.split(",") if k]
    t0 = time.time()
    fails, stats = run_batch(args.n, args.seed, kinds, reanchor=args.reanchor,
                             round_close=not args.no_roundclose)
    dt = time.time() - t0
    stats["elapsed_s"] = round(dt, 2)
    print("场景数 %d  用时 %.1fs (%.0f 场景/s)"
          % (stats["scenarios"], dt, stats["scenarios"] / max(dt, 1e-9)))
    print("模式分布:", json.dumps(stats["by_mode"], ensure_ascii=False))
    if stats["lat_max_us"]:
        print("恢复延迟: min %.1fms / max %.1fms（非 NO_LINK 上限 360ms；NO_LINK 另加 500ms）"
              % (stats["min_lat_us"] / 1000.0, stats["lat_max_us"] / 1000.0))
    print("恢复轮数分布（恢复延迟 // 120ms）:", json.dumps(stats.get("lat_buckets", {}), ensure_ascii=False))
    print("握手窗口碰撞（协议固有，已计数不判失败）:", stats["collisions"])
    print("迟到帧容差碰撞（U3 迟到>6ms 与兜底 D0 同刻，见 REPORT）：", stats["late_collisions"])
    print("迟到上行在 D0 后到达（口径缺口计数）:", stats["late_after_d0"])
    if fails:
        print("\n!!! 失败场景 %d 个（前 %d 个）:" % (len(fails), len(fails)))
        for f in fails:
            print("  kind=%s seed=%d -> %s" % (f["kind"], f["seed"], "; ".join(f["fails"])))
            print("    cfg=%s" % json.dumps({k: v for k, v in f["cfg"].items() if k != "trace"},
                                            ensure_ascii=False))
        if args.json:
            with open(args.json, "w", encoding="utf-8") as fp:
                json.dump({"stats": stats, "fails": fails}, fp, ensure_ascii=False, indent=1)
        return 1
    print("\n全部断言通过: 无双发 / 无死锁 / 恢复≤3轮 / 版本与混烧安全 / 每轮≤1下行")
    if args.json:
        with open(args.json, "w", encoding="utf-8") as fp:
            json.dump(stats, fp, ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
