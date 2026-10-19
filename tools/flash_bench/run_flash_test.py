"""Build the RAM flash-test program, run it on the target via pyOCD, dump results.

The flight firmware is halted, the test image is copied into free RAM
(0x2001C000), PC/SP are redirected, and the tagged result words written at
RESULT_ADDR are read back over SWD.  The target is reset afterwards.
"""
import os
import struct
import subprocess
import sys
import time

from pyocd.core.helpers import ConnectHelper

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
import sys as _sys
_SRC = "fw_flash_sr.c"
for _a in _sys.argv[1:]:
    if _a.startswith("--src="):
        _SRC = _a.split("=", 1)[1]
SRC = os.path.join(HERE, _SRC)
ELF = os.path.join(OUT, _SRC.replace(".c", ".elf"))
BIN = os.path.join(OUT, _SRC.replace(".c", ".bin"))

CODE_ADDR = 0x2001C000
RESULT_ADDR = 0x2001B000
STACK_TOP = 0x2001F000
TAGS = {
    0x74696E67: "START",
    0xA0000000: "JEDEC",
    0xA1000000: "SR1",
    0xA2000000: "NOCS",
    0xA4000000: "RACE",
    0xA5000000: "RACEN",
    0xB0000000: "BAD",
    0xB1000000: "TOTAL",
    0xB2000000: "FIRSTBAD",
    0xC0000001: "BATCH /8 plain",
    0xC0000002: "BATCH /8 cfg-rewrite",
    0xC0000003: "BATCH /16",
    0xC0000004: "BATCH /32",
    0xC0000005: "BATCH /8 3000x",
    0xDEADBEEF: "DONE",
}


def build():
    os.makedirs(OUT, exist_ok=True)
    cc = ["arm-none-eabi-gcc", "-mcpu=cortex-m4", "-mthumb", "-O2", "-ffreestanding",
          "-nostdlib", "-fno-builtin", "-Wall", "-Wextra",
          "-Wl,-Ttext=0x2001C000", "-Wl,-e,ram_main", "-o", ELF, SRC]
    r = subprocess.run(cc, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout, r.stderr)
        raise SystemExit("compile failed")
    if r.stderr.strip():
        print("warnings:", r.stderr.strip()[:800])
    subprocess.run(["arm-none-eabi-objcopy", "-O", "binary", ELF, BIN], check=True)
    nm = subprocess.run(["arm-none-eabi-nm", ELF], capture_output=True, text=True).stdout
    entry = None
    for ln in nm.splitlines():
        p = ln.split()
        if len(p) == 3 and p[2] in ("ram_main",):
            entry = int(p[0], 16) & ~1
    print("entry ram_main @ 0x%08X, image %d bytes" % (entry, os.path.getsize(BIN)))
    return entry


def run(entry):
    img = open(BIN, "rb").read()
    words = img + b"\x00" * ((4 - len(img) % 4) % 4)
    s = ConnectHelper.session_with_chosen_probe(
        options={"target_override": "cortex_m", "frequency": 4000000})
    with s:
        t = s.target
        t.reset_and_halt()
        t.write_memory_block32(CODE_ADDR, list(struct.unpack("<%dI" % (len(words) // 4), words)))
        t.write_memory_block32(RESULT_ADDR, [0] * (0x1000 // 4))
        t.write_core_register("primask", 1)
        t.write_core_register("sp", STACK_TOP)
        t.write_core_register("pc", entry)
        print("running...")
        t.resume()
        done = False
        deadline = time.time() + 30
        while time.time() < deadline:
            if t.is_halted():
                print("!! core halted unexpectedly")
                break
            head = t.read32(RESULT_ADDR)
            if head == 0x74696E67:
                for i in range(0, 0x1000 // 4):
                    if t.read32(RESULT_ADDR + 4 * i) == 0xDEADBEEF:
                        done = True
                        break
            if done:
                break
            time.sleep(0.05)
        words = [t.read32(RESULT_ADDR + 4 * i) for i in range(0x1000 // 4)]
        t.halt()
        t.reset()
    if not done:
        print("!! DONE marker not found; dumping what is there")
    return words


def dump(words):
    i = 0
    while i < len(words):
        w = words[i]
        if w == 0xDEADBEEF:
            print("DONE")
            break
        if w == 0x74696E67:
            print("START version=%d" % words[i + 1])
            i += 2
            continue
        hi = w & 0xFF000000
        lo = w & 0x00FFFFFF
        if w == 0xDEADBEEF:
            print("DONE")
            break
        if hi in (0xA0000000, 0xB0000000, 0xC0000000, 0xD0000000):
            print("  read[%d] JEDEC=0x%06X %s" % ((lo >> 16) & 0xF, lo & 0xFFFF,
                                                  "OK" if (lo & 0xFFFF) == 0x4018 else ""))
            i += 1
            continue
        if hi == 0xA1000000:
            print("  SR1=0x%02X" % (lo & 0xFF))
            i += 1
            continue
        if hi == 0xE0000000:
            print("  -- WRSR race --")
            i += 1
            continue
        if hi == 0xE1000000:
            print("  busy-poll iterations until idle = %d" % lo)
            i += 1
            continue
        if hi == 0xF1000000:
            print("  RCC_CFGR = 0x%06X" % lo)
        elif hi == 0xF2000000:
            print("  RCC_AHB1ENR = 0x%06X" % lo)
        elif hi == 0xF3000000:
            print("  SPI2_CR1 = 0x%04X" % (lo & 0xFFFF))
        else:
            print("  0x%08X ?" % w)
        i += 1
        continue


if __name__ == "__main__":
    e = build()
    dump(run(e))
