"""Flash JEDEC bench toolkit: ST-Link (pyOCD) reset / peek + COM11 raw capture.

Usage:
    python tools/flash_bench/bench.py ping
    python tools/flash_bench/bench.py reset
    python tools/flash_bench/bench.py boot --n 5        # reset+capture N boots, decode text
"""
import argparse
import os
import struct
import sys
import threading
import time

import serial
from pyocd.core.helpers import ConnectHelper

COM = "COM11"
BAUD = 2000000
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")


# ---------------------------------------------------------------- DLX framing
def crc4_itu(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x0C) if (crc & 1) else (crc >> 1)
    return crc & 0xF


def unblock_frame(param: int = 1) -> bytes:
    payload = bytes([param & 0xFF])
    head = (1 << 15) | (1 << 5) | (crc4_itu(payload) << 1)
    return struct.pack(">HH", head, 0) [:2] + payload  # header BE + payload


def ground_cmd_frame(op: int, mode: int, session_id: int = 0, count: int = 0) -> bytes:
    payload = struct.pack(">HHII", op, mode, session_id, count)
    head = (1 << 15) | (3 << 5) | (crc4_itu(payload) << 1)
    return struct.pack(">H", head) + payload


# Type ids and payload sizes from DL_LIB/dlx.hpp (MCU side) / generated host reader.
#   0 SimpleLog(var) 1 UnBlock(1) 2 Quat(16) 3 GroundCmd(12) 4 GroundReply(12)
#   5 SessionInfo(16) 6 PerfMonitor(28) 7 FlightLog(140) 8 ExceptionReport(13)
#   9 DLXException(2) 10 DebugSensor(48)
FIXED = {1: 1, 2: 16, 3: 12, 4: 12, 5: 16, 6: 28, 7: 140, 8: 13, 9: 2, 10: 48}


def decode_stream(raw: bytes):
    """Yield ('text', str) / ('ex', code) / ('reply', ...) / ('other', type, bytes) in wire order."""
    i = 0
    n = len(raw)
    while i + 2 <= n:
        head = (raw[i] << 8) | raw[i + 1]
        if not (head & 0x8000) or (head & 1):
            i += 1
            continue
        typ = (head >> 5) & 0x3FF
        if typ == 0:  # SimpleLog: [head u16][len u16][payload]
            if i + 4 > n:
                break
            ln = (raw[i + 2] << 8) | raw[i + 3]
            if i + 4 + ln > n:
                break
            payload = raw[i + 4:i + 4 + ln]
            yield ("text", payload.decode("utf-8", "replace"))
            i += 4 + ln
        elif typ == 9:
            if i + 4 > n:
                break
            yield ("ex", (raw[i + 2] << 8) | raw[i + 3])
            i += 4
        else:
            ln = FIXED.get(typ)
            if ln is None:
                i += 1
                continue
            if i + 2 + ln > n:
                break
            payload = raw[i + 2:i + 2 + ln]
            if typ == 4:
                op, status, sid, val = struct.unpack(">HHII", payload)
                yield ("reply", (op, status, sid, val))
            else:
                yield ("other", typ, payload)
            i += 2 + ln


# ---------------------------------------------------------------- ST-Link
def session():
    return ConnectHelper.session_with_chosen_probe(
        options={"target_override": "cortex_m", "frequency": 4000000}
    )


def ping():
    s = session()
    with s:
        t = s.target
        t.halt()
        print("halted:", t.get_state())
        print("cpuid   : 0x%08X" % t.read32(0xE000ED00))
        print("RCC_CSR : 0x%08X" % t.read32(0x40023874))
        print("SPI2_CR1: 0x%04X" % t.read16(0x40003800))
        print("SPI2_SR : 0x%04X" % t.read16(0x40003808))
        t.resume()


def reset_mcu():
    s = session()
    with s:
        s.target.reset()
    time.sleep(0.02)


class Capture(threading.Thread):
    def __init__(self, port=COM, baud=BAUD):
        super().__init__(daemon=True)
        self.buf = bytearray()
        self.port = port
        self.baud = baud
        self.stop = False
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.02)

    def run(self):
        while not self.stop:
            n = self.ser.in_waiting
            if n:
                self.buf += self.ser.read(n)
            else:
                time.sleep(0.002)

    def close(self):
        self.stop = True
        self.join(1.0)
        if self.ser:
            self.ser.close()


def send_unlock(ser, seconds=1.2, use_groundcmd=True):
    """Keep sending unlock while MCU boots/waites for it."""
    frame = ground_cmd_frame(1, 1) if use_groundcmd else unblock_frame(1)
    end = time.time() + seconds
    while time.time() < end:
        ser.write(frame)
        time.sleep(0.15)


def boot_once(secs=6.0):
    cap = Capture()
    cap.open()
    cap.start()
    # start sending unlock before reset so it lands while the MCU waits in decideBootMode
    t = threading.Thread(target=send_unlock, args=(cap.ser, secs), daemon=True)
    t.start()
    time.sleep(0.05)
    reset_mcu()
    t.join()
    time.sleep(max(0.0, secs - 1.3))
    cap.close()
    return bytes(cap.buf)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["ping", "reset", "boot", "listen"])
    ap.add_argument("--n", type=int, default=1)
    ap.add_argument("--secs", type=float, default=6.0)
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)

    if args.cmd == "ping":
        ping()
    elif args.cmd == "reset":
        reset_mcu()
        print("reset issued")
    elif args.cmd == "listen":
        cap = Capture()
        cap.open()
        cap.start()
        time.sleep(args.secs)
        cap.close()
        raw = bytes(cap.buf)
        open(os.path.join(OUT, "listen.raw"), "wb").write(raw)
        for kind, v in decode_stream(raw):
            if kind == "text":
                sys.stdout.write(v)
            elif kind == "ex":
                print("<EX 0x%04X>" % v)
        print("\n[%d bytes]" % len(raw))
    elif args.cmd == "boot":
        os.makedirs(os.path.join(OUT, "boots"), exist_ok=True)
        stamp = time.strftime("%H%M%S")
        results = []
        for k in range(args.n):
            raw = boot_once(args.secs)
            fn = os.path.join(OUT, "boots", "boot_%s_%02d.raw" % (stamp, k))
            open(fn, "wb").write(raw)
            txt = "".join(e[1] for e in decode_stream(raw) if e[0] == "text")
            open(fn.replace(".raw", ".txt"), "w", encoding="utf-8").write(txt)
            exs = [e[1] for e in decode_stream(raw) if e[0] == "ex"]
            verdict = "?"
            jedec = "-"
            for line in txt.splitlines():
                if "JEDEC=" in line:
                    jedec = line.split("JEDEC=")[1].split()[0].strip()
                if "init=OK" in line:
                    verdict = "OK"
                elif "init=FAILED" in line:
                    verdict = "FAILED"
                elif "识别失败" in line:
                    verdict = "ID_FAIL"
            results.append((k, verdict, jedec, len(raw), exs))
            # ASCII-only summary (console is GBK): keep the verdict and the raw JEDEC value
            print("[boot %02d] verdict=%-6s jedec=%-8s bytes=%-5d ex=%s %s" % (
                k, verdict, jedec, len(raw), ["0x%04X" % e for e in exs],
                os.path.basename(fn)))
            sys.stdout.flush()
            time.sleep(0.5)
        ok = sum(1 for r in results if r[1] == "OK")
        bad = len(results) - ok
        print("\n=== summary: %d boots, OK=%d BAD=%d ===" % (len(results), ok, bad))
        for r in results:
            print("  boot %02d: %-6s jedec=%s" % (r[0], r[1], r[2]))



if __name__ == "__main__":
    main()
