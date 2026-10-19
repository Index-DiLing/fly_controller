"""Generic build+run of a RAM bench program via pyOCD, printing tagged results.

Usage: python tools/flash_bench/run_ram.py fw_flash_recover.c [--noreset]
"""
import argparse
import os
import struct
import subprocess
import time

from pyocd.core.helpers import ConnectHelper

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
CODE_ADDR = 0x2001C000
RESULT_ADDR = 0x2001B000
STACK_TOP = 0x2001F000


def build(src):
    srcpath = os.path.join(HERE, src)
    base = os.path.join(OUT, os.path.splitext(src)[0])
    elf, binp = base + ".elf", base + ".bin"
    cc = ["arm-none-eabi-gcc", "-mcpu=cortex-m4", "-mthumb", "-O2", "-ffreestanding",
          "-nostdlib", "-fno-builtin", "-Wall", "-Wextra",
          "-Wl,-Ttext=0x2001C000", "-Wl,-e,ram_main", "-o", elf, srcpath]
    r = subprocess.run(cc, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout, r.stderr)
        raise SystemExit("compile failed")
    if r.stderr.strip():
        print("warnings:", r.stderr.strip()[:600])
    subprocess.run(["arm-none-eabi-objcopy", "-O", "binary", elf, binp], check=True)
    nm = subprocess.run(["arm-none-eabi-nm", elf], capture_output=True, text=True).stdout
    entry = None
    for ln in nm.splitlines():
        p = ln.split()
        if len(p) == 3 and p[2] == "ram_main":
            entry = int(p[0], 16) & ~1
    print("entry=0x%08X image=%d bytes" % (entry, os.path.getsize(binp)))
    return entry, binp


def run(entry, binp, do_reset=True):
    img = open(binp, "rb").read()
    img += b"\x00" * ((4 - len(img) % 4) % 4)
    s = ConnectHelper.session_with_chosen_probe(
        options={"target_override": "cortex_m", "frequency": 4000000})
    with s:
        t = s.target
        if do_reset:
            t.reset_and_halt()
        else:
            t.halt()
        t.write_memory_block32(CODE_ADDR, list(struct.unpack("<%dI" % (len(img) // 4), img)))
        t.write_memory_block32(RESULT_ADDR, [0] * (0x1000 // 4))
        t.write_core_register("primask", 1)
        t.write_core_register("sp", STACK_TOP)
        t.write_core_register("pc", entry)
        t.resume()
        deadline = time.time() + 40
        while time.time() < deadline:
            if t.is_halted():
                print("!! core halted (fault?)")
                break
            if t.read32(RESULT_ADDR) == 0x74696E67:
                for i in range(0x1000 // 4):
                    if t.read32(RESULT_ADDR + 4 * i) == 0xDEADBEEF:
                        words = [t.read32(RESULT_ADDR + 4 * j) for j in range(i + 1)]
                        t.halt()
                        t.reset()
                        return words
            time.sleep(0.03)
        words = [t.read32(RESULT_ADDR + 4 * j) for j in range(0x400 // 4)]
        t.halt()
        t.reset()
        return words


def show(words):
    for w in words:
        print("0x%08X" % w)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("--noreset", action="store_true")
    a = ap.parse_args()
    entry, binp = build(a.src)
    show(run(entry, binp, do_reset=not a.noreset))
