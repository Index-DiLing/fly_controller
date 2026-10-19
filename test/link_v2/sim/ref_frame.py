#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""链路 v2 帧层 —— 独立参考实现 / 黄金向量 oracle（验证者视角，按文档重写）

依据 docs/链路层v2_协议规格.md
  §1.1 帧结构   B0=(TYPE<<4)|(SEQ&0xF)，负载 LSB-first 位打包小端，尾部 CRC-16 低字节在前
  §1.2 帧类型/长度表      §3.1 UP_ROUND     §3.2 HELLO    §3.3 UP_CMD   §3.4 DL_ACK
  §3.5 DL_STATE 空位      §3.6 flags        §10 黄金向量
"布局同 v1"的帧（0x1/0x2/0x3/0x4/0x8/0xA/0xB/0xC/0xD）位偏移取 v1 冻结布局
（规格 §1.2 明文：位宽/位偏移/定点换算沿用现有实现）。该布局在本文件固化为常量，
不随被测实现变化。

语料一律用**线上整数域(raw)**：float 字段也给量化后的整数，避免两端"物理量->定点"的
舍入差异污染帧层对拍（那属于语义层，另有单独断言）。

用法:
  python ref_frame.py selfcheck              # 校验 §10 黄金向量
  python ref_frame.py emit-corpus OUT.txt    # 生成对拍语料（规格向量+边界+随机）
  python ref_frame.py emit-expected OUT.txt  # 生成每例参考字节
  python ref_frame.py emit-fuzz OUT.txt N    # 生成 N 条随机坏字节流（含嵌入合法帧）
"""
import sys, random

# ---------------------------------------------------------------- CRC
def crc16_ccitt_false(data):
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else ((crc << 1) & 0xFFFF)
    return crc

assert crc16_ccitt_false(b"123456789") == 0x29B1

# ---------------------------------------------------------------- 位打包
def _put(buf, val, bit, width):
    v = val & ((1 << width) - 1) if width < 64 else val
    byte, off, shift = bit // 8, bit % 8, 0
    while shift < width:
        take = min(8 - off, width - shift)
        buf[byte] |= ((v >> shift) & ((1 << take) - 1)) << off
        shift += take
        byte += 1
        off = 0


def _get(buf, bit, width):
    byte, off, shift, val = bit // 8, bit % 8, 0, 0
    while shift < width:
        take = min(8 - off, width - shift)
        val |= ((buf[byte] >> off) & ((1 << take) - 1)) << shift
        shift += take
        byte += 1
        off = 0
    return val


def _sx(v, width):
    v &= (1 << width) - 1
    return v - (1 << width) if (v & (1 << (width - 1))) else v


def _f(name, bit, width, rmin, rmax, scale=None):
    return {"name": name, "bit": bit, "width": width, "kind": "f" if scale else "s",
            "rmin": rmin, "rmax": rmax, "scale": scale}


def _u(name, bit, width, rmin, rmax):
    return {"name": name, "bit": bit, "width": width, "kind": "u", "rmin": rmin, "rmax": rmax}


def _s(name, bit, width, rmin, rmax):
    return {"name": name, "bit": bit, "width": width, "kind": "s", "rmin": rmin, "rmax": rmax}


# 字段：rmin/rmax = 本字段**合法输入域**（线上整数，语义范围内；越界饱和不在对拍输入里）
FRAMES = {
    0x1: dict(name="DL_ATT", frame_bytes=13, payload_bits=80, fields=[
        _f("q1", 0, 12, -2047, 2047, 2047), _f("q2", 12, 12, -2047, 2047, 2047),
        _f("q3", 24, 12, -2047, 2047, 2047),
        _f("vx", 36, 8, -128, 127, 10), _f("vy", 44, 8, -128, 127, 10),
        _f("vz", 52, 8, -128, 127, 10),
        _u("status", 60, 8, 0, 255), _u("error", 68, 8, 0, 255),
        _u("arming", 76, 2, 0, 3), _u("mode", 78, 2, 0, 3),
    ]),
    0x2: dict(name="DL_SYS", frame_bytes=15, payload_bits=96, fields=[
        _f("x", 0, 12, -2048, 2047, 10), _f("y", 12, 12, -2048, 2047, 10),
        _f("z", 24, 12, -2048, 2047, 10),
        _f("current", 36, 8, 0, 255, 10), _f("voltage", 44, 8, 0, 255, 10),
        _u("motor0", 52, 11, 0, 1999), _u("motor1", 63, 11, 0, 1999),
        _u("motor2", 74, 11, 0, 1999), _u("motor3", 85, 11, 0, 1999),
    ]),
    0x3: dict(name="DL_GPS", frame_bytes=21, payload_bits=144, fields=[
        _s("lat", 0, 32, -2 ** 31, 2 ** 31 - 1), _s("lon", 32, 32, -2 ** 31, 2 ** 31 - 1),
        _s("alt", 64, 32, -2 ** 31, 2 ** 31 - 1),
        _s("gspeed", 96, 16, -32768, 32767), _u("course", 112, 16, 0, 35999),
        _u("sats", 128, 6, 0, 63), _u("fix", 134, 2, 0, 3),
        _f("hdop", 136, 8, 0, 255, 10),
    ]),
    0x4: dict(name="DL_STATE", frame_bytes=10, payload_bits=56, fields=[
        _u("lifecycle", 0, 3, 0, 7), _u("arming", 3, 2, 0, 3), _u("mode", 5, 3, 0, 7),
        _u("task", 8, 2, 0, 3), _u("failsafe_lvl", 10, 3, 0, 7),
        _u("failsafe_rsn", 13, 4, 0, 15), _u("health", 17, 8, 0, 255),
        _u("ul_loss", 25, 8, 0, 100), _u("dl_loss", 33, 8, 0, 100),
        _s("rssi", 41, 8, -128, 127),
        _u("posmask", 49, 4, 0, 15),          # 规格 §3.5: bit49..52 = 收到 pos0..3
        _u("spare", 53, 3, 0, 0),             # bit53..55 保留恒 0
    ]),
    0x5: dict(name="DL_ACK", frame_bytes=5, payload_bits=16, fields=[
        _u("cmd_id", 0, 8, 1, 255), _u("result", 8, 8, 0, 11),
    ]),
    0x6: dict(name="UP_ROUND", frame_bytes=11, payload_bits=64, fields=[
        _s("roll", 0, 11, -1023, 1023), _s("pitch", 11, 11, -1023, 1023),
        _s("yaw", 22, 11, -1023, 1023), _u("thr", 33, 11, 0, 2047),
        _u("flags", 44, 4, 0, 15), _u("session", 48, 8, 1, 255),
        _u("flags8", 56, 8, 0, 7),
    ]),
    0x7: dict(name="HELLO", frame_bytes=5, payload_bits=16, fields=[
        _u("ver", 0, 4, 1, 15), _u("caps", 4, 4, 0, 7), _u("nonce", 8, 8, 0, 255),
    ]),
    0x8: dict(name="UP_CTRL", frame_bytes=9, payload_bits=48, fields=[
        _s("roll", 0, 11, -1023, 1023), _s("pitch", 11, 11, -1023, 1023),
        _s("yaw", 22, 11, -1023, 1023), _u("thr", 33, 11, 0, 2047),
        _u("flags", 44, 4, 0, 15),
    ]),
    0x9: dict(name="UP_CMD", frame_bytes=6, payload_bits=24, fields=[
        _u("cmd_id", 0, 8, 1, 255), _u("op", 8, 8, 0, 255), _u("arg", 16, 8, 0, 255),
    ]),
    0xA: dict(name="UP_VEL", frame_bytes=6, payload_bits=24, fields=[
        _s("vx", 0, 8, -128, 127), _s("vy", 8, 8, -128, 127), _s("vz", 16, 8, -128, 127),
    ]),
    0xB: dict(name="UP_POS", frame_bytes=7, payload_bits=32, fields=[
        _s("x", 0, 16, -32768, 32767), _s("y", 16, 16, -32768, 32767),
    ]),
    0xC: dict(name="UP_SETPOINT", frame_bytes=6, payload_bits=24, fields=[
        _s("alt_cm", 0, 16, -32767, 32767), _u("flags", 16, 8, 0, 1),
    ]),
    0xD: dict(name="UP_MOTOR", frame_bytes=9, payload_bits=48, fields=[
        _u("motor0", 0, 11, 0, 1999), _u("motor1", 11, 11, 0, 1999),
        _u("motor2", 22, 11, 0, 1999), _u("motor3", 33, 11, 0, 1999),
        _u("spare", 44, 4, 0, 0),
    ]),
}

LEN_TABLE = [0] * 16
for _t, _m in FRAMES.items():
    LEN_TABLE[_t] = _m["frame_bytes"]


def encode_raw(ftype, seq, raw):
    """raw: {字段名: 线上整数}（float 字段也给量化后的整数）。缺省 0。"""
    m = FRAMES[ftype]
    buf = bytearray(m["payload_bits"] // 8)
    for f in m["fields"]:
        _put(buf, int(raw.get(f["name"], 0)), f["bit"], f["width"])
    body = bytes([((ftype << 4) | (seq & 0x0F)) & 0xFF]) + bytes(buf)
    crc = crc16_ccitt_false(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def decode_raw(frame):
    """返回 (type, seq, {字段: 线上整数})；非法长度/CRC 抛 ValueError。"""
    if len(frame) < 5:
        raise ValueError("too short")
    ftype, seq = frame[0] >> 4, frame[0] & 0x0F
    exp = LEN_TABLE[ftype]
    if exp == 0:
        raise ValueError("illegal type")
    if len(frame) != exp:
        raise ValueError("bad length")
    if crc16_ccitt_false(frame[:-2]) != (frame[-2] | (frame[-1] << 8)):
        raise ValueError("bad crc")
    payload = frame[1:-2]
    out = {}
    for f in FRAMES[ftype]["fields"]:
        v = _get(payload, f["bit"], f["width"])
        out[f["name"]] = _sx(v, f["width"]) if f["kind"] in ("s", "f") else v
    return ftype, seq, out


def scan_stream(buf):
    """规格 §1.1：坏帧/未知类型只前进 1 字节重同步。返回 [(offset, type, seq)]（CRC 正确）。"""
    i, out = 0, []
    while i < len(buf):
        exp = LEN_TABLE[buf[i] >> 4]
        if exp and i + exp <= len(buf):
            try:
                t, s, _ = decode_raw(bytes(buf[i:i + exp]))
                out.append((i, t, s))
                i += exp
                continue
            except ValueError:
                pass
        i += 1
    return out


def scan_streaming(buf):
    """接收端逐字节喂入模型（与两端 tryDecode/fc_frame_next 的流式语义一致）：
    每来一个字节，循环"解出一帧就前进整帧 / 坏就前进 1 字节 / 字节不够就等"。
    返回 [(offset, type, seq)]。"""
    acc = bytearray()
    base = 0
    out = []
    for b in buf:
        acc.append(b)
        while acc:
            exp = LEN_TABLE[acc[0] >> 4]
            if exp == 0:
                acc.pop(0)
                base += 1
                continue
            if len(acc) < exp:
                break                      # 字节不够：等后续字节
            try:
                t, s, _ = decode_raw(bytes(acc[:exp]))
                out.append((base, t, s))
                del acc[:exp]
                base += exp
            except ValueError:
                acc.pop(0)
                base += 1
    return out


# ---------------------------------------------------------------- §10 黄金向量（原样抄自规格）
SPEC_VECTORS = [
    ("HELLO seq=0 ver=2 caps=0 nonce=0x5A", 0x7, 0x0,
     dict(ver=2, caps=0, nonce=0x5A), "70 02 5A 49 89"),
    ("UP_ROUND seq=0 杆量0 thr=1000 flags=1 session=0x3C flags8=1", 0x6, 0x0,
     dict(roll=0, pitch=0, yaw=0, thr=1000, flags=1, session=0x3C, flags8=1),
     "60 00 00 00 00 D0 17 3C 01 7E DE"),
    ("UP_CMD seq=1 cmd_id=7 op=0x01 arg=0", 0x9, 0x1,
     dict(cmd_id=7, op=0x01, arg=0), "91 07 01 00 4A 82"),
    ("DL_ACK seq=2 cmd_id=7 result=0", 0x5, 0x2,
     dict(cmd_id=7, result=0), "52 07 00 A5 65"),
    ("UP_CTRL seq=2 全0", 0x8, 0x2, dict(), "82 00 00 00 00 00 00 D4 3A"),
    ("UP_CTRL seq=1 roll=-1023 pitch=1023 thr=2047 flags=3", 0x8, 0x1,
     dict(roll=-1023, pitch=1023, thr=2047, flags=3), "81 01 FC 1F 00 FE 3F B0 2B"),
    ("DL_ATT seq=0 全0负载", 0x1, 0x0, dict(), "10 00 00 00 00 00 00 00 00 00 00 32 81"),
    ("DL_STATE seq=3 全0负载", 0x4, 0x3, dict(), "43 00 00 00 00 00 00 00 9B 0F"),
]


def selfcheck(verbose=True):
    ok = True
    lines = []
    c = crc16_ccitt_false(b"123456789")
    lines.append("CRC-16/CCITT-FALSE(\"123456789\")=0x%04X 期望0x29B1 %s" % (c, "OK" if c == 0x29B1 else "FAIL"))
    ok = ok and c == 0x29B1
    for name, ftype, seq, values, exp_hex in SPEC_VECTORS:
        got = encode_raw(ftype, seq, values)
        exp = bytes(int(x, 16) for x in exp_hex.split())
        good = got == exp
        ok = ok and good
        try:
            t2, s2, back = decode_raw(got)
            loop_ok = (t2 == ftype and s2 == seq and
                       all(back[f["name"]] == int(values.get(f["name"], 0)) for f in FRAMES[ftype]["fields"]))
        except ValueError:
            loop_ok = False
        ok = ok and loop_ok
        lines.append("%-46s exp=%s got=%s %s%s"
                     % (name, exp_hex, " ".join("%02X" % b for b in got),
                        "OK" if good else "MISMATCH", "" if loop_ok else " (decode-loop FAIL)"))
    if verbose:
        print("\n".join(lines))
        print("\n§10 黄金向量 vs 独立参考实现: %s" % ("全部一致" if ok else "存在不一致"))
    return ok, lines


# ---------------------------------------------------------------- 语料
def _boundary_values(f):
    lo, hi = f["rmin"], f["rmax"]
    vals = {lo, hi}
    for d in (-2, -1, 0, 1, 2):
        for base in (lo, hi, 0):
            v = base + d
            if lo <= v <= hi:
                vals.add(v)
    if f["kind"] == "u" and hi > 3:
        vals.update({x for x in (1, 2, 3, 7, 15, 63, 127, 255, 1023, 1999) if lo <= x <= hi})
    return sorted(vals)


def build_corpus(seed=20261018, nrandom=1000):
    rng = random.Random(seed)
    cases = []   # (name, type, seq, raw_values)

    for name, ftype, seq, values, _ in SPEC_VECTORS:
        cases.append(("spec:" + name, ftype, seq, dict(values)))

    for ftype, m in sorted(FRAMES.items()):
        for f in m["fields"]:
            for v in _boundary_values(f):
                cases.append(("bound:%s.%s=%d" % (m["name"], f["name"], v), ftype, 0, {f["name"]: v}))

    for ftype, m in sorted(FRAMES.items()):
        for tag in ("min", "max", "zero"):
            raw = {}
            for f in m["fields"]:
                raw[f["name"]] = {"min": f["rmin"], "max": f["rmax"], "zero": 0}[tag]
            cases.append(("edge:%s.%s" % (m["name"], tag), ftype, 0, raw))

    types = sorted(FRAMES.keys())
    for i in range(nrandom):
        ftype = types[i % len(types)]
        m = FRAMES[ftype]
        raw = {}
        for f in m["fields"]:
            lo, hi = f["rmin"], f["rmax"]
            raw[f["name"]] = rng.randint(lo, hi)
        cases.append(("rand:%d" % i, ftype, rng.randint(0, 15), raw))
    return cases


def case_line(ftype, seq, raw):
    parts = ["%02X" % ftype, "%02X" % seq]
    for f in FRAMES[ftype]["fields"]:
        parts.append("%s=%d" % (f["name"], int(raw.get(f["name"], 0))))
    return " ".join(parts)


def write_corpus(path, cases):
    with open(path, "w", encoding="utf-8", newline="\n") as fp:
        fp.write("# 链路 v2 帧层对拍语料（线上整数域）%d 例\n" % len(cases))
        for name, ftype, seq, raw in cases:
            fp.write("# " + name + "\n")
            fp.write(case_line(ftype, seq, raw) + "\n")


def write_expected(path, cases):
    with open(path, "w", encoding="utf-8", newline="\n") as fp:
        for name, ftype, seq, raw in cases:
            fp.write("%s %s\n" % (name, " ".join("%02X" % b for b in encode_raw(ftype, seq, raw))))


def build_fuzz_streams(seed=777, n=200, max_len=96):
    """随机坏字节流：随机噪声 + 随机嵌入合法帧（含截断帧/翻位帧）。返回 [(seed, bytes)]"""
    rng = random.Random(seed)
    types = sorted(FRAMES.keys())
    out = []
    for i in range(n):
        srng = random.Random(seed * 100003 + i)
        buf = bytearray()
        while len(buf) < srng.randint(16, max_len):
            kind = srng.random()
            if kind < 0.45:
                # 噪声字节（含大量"像帧头"的高半字节）
                buf.append(srng.randrange(256))
            elif kind < 0.75:
                ftype = srng.choice(types)
                raw = {f["name"]: srng.randint(f["rmin"], max(f["rmin"], min(f["rmax"], 200)))
                       for f in FRAMES[ftype]["fields"]}
                fr = encode_raw(ftype, srng.randrange(16), raw)
                # 30% 翻 1 bit（CRC 坏帧），10% 截断
                if srng.random() < 0.3:
                    pos = srng.randrange(len(fr))
                    fr = bytearray(fr)
                    fr[pos] ^= 1 << srng.randrange(8)
                    fr = bytes(fr)
                if srng.random() < 0.1:
                    fr = fr[:srng.randrange(1, len(fr))]
                buf.extend(fr)
            else:
                # 全 0xFF / 全 0x00 段
                buf.extend([srng.choice((0x00, 0xFF))] * srng.randint(1, 8))
        out.append((seed * 100003 + i, bytes(buf)))
    return out


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "selfcheck"
    if cmd == "selfcheck":
        ok, _ = selfcheck()
        sys.exit(0 if ok else 1)
    elif cmd == "emit-corpus":
        out = sys.argv[2] if len(sys.argv) > 2 else "corpus.txt"
        cases = build_corpus()
        write_corpus(out, cases)
        print("wrote %s (%d cases)" % (out, len(cases)))
    elif cmd == "emit-expected":
        out = sys.argv[2] if len(sys.argv) > 2 else "expected.txt"
        cases = build_corpus()
        write_expected(out, cases)
        print("wrote %s (%d cases)" % (out, len(cases)))
    elif cmd == "emit-fuzz":
        out = sys.argv[2] if len(sys.argv) > 2 else "fuzz.txt"
        n = int(sys.argv[3]) if len(sys.argv) > 3 else 200
        streams = build_fuzz_streams(n=n)
        with open(out, "w", encoding="utf-8", newline="\n") as fp:
            for _seed, buf in streams:
                fp.write(buf.hex().upper() + "\n")
        print("wrote %s (%d streams)" % (out, len(streams)))
    else:
        print(__doc__)
        sys.exit(2)
