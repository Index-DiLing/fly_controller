"""Send unlock frames and check whether the MCU's boot-mode command callback is reached.

Breakpoint: dlx::onBootGroundCmd (from dl_fly.map) -- if hit, the frame was parsed OK
and the remaining problem is in decideBootMode()/reply path; if never hit, our bytes
never make it into a valid DLX frame (framing/CRC/ring buffer).
"""
import struct
import time

import serial
from pyocd.core.helpers import ConnectHelper

COM, BAUD = "COM11", 2000000
BP_ONBOOT_GROUNDCMD = 0x0800E6E1
BP_ONBOOT_UNBLOCK = 0x0800E6E1  # same stub area; use groundcmd only

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench import crc4_itu, ground_cmd_frame, unblock_frame  # noqa: E402


def run(frames, labels, bp):
    s = ConnectHelper.session_with_chosen_probe(
        options={"target_override": "cortex_m", "frequency": 4000000})
    with s:
        t = s.target
        t.halt()
        t.set_breakpoint(bp)
        ser = serial.Serial(COM, BAUD, timeout=0.05)
        t.resume()
        hit = None
        for label, fr in zip(labels, frames):
            ser.write(fr)
            ser.flush()
            for _ in range(40):
                if t.is_halted():
                    hit = label
                    break
                time.sleep(0.005)
            if hit:
                break
        pc = t.read_core_register("pc") if t.is_halted() else None
        print("halted=%s pc=%s hit=%s" % (t.is_halted(), "0x%08X" % pc if pc else "-", hit))
        if t.is_halted():
            print("  r0=0x%08X r1=0x%08X r2=0x%08X" % (
                t.read_core_register("r0"), t.read_core_register("r1"), t.read_core_register("r2")))
        t.remove_breakpoint(bp)
        t.resume()
        ser.close()
    return hit


if __name__ == "__main__":
    gc = ground_cmd_frame(1, 1)
    ub = unblock_frame(1)
    print("GroundCmd frame:", gc.hex(" "))
    print("UnBlock  frame:", ub.hex(" "))
    print("--- send GroundCmd(1,1) ---")
    run([gc], ["groundcmd"], BP_ONBOOT_GROUNDCMD)
    print("--- send UnBlock(1) ---")
    run([ub], ["unblock"], BP_ONBOOT_GROUNDCMD)
    print("--- send UnBlock(1) x5 with gaps ---")
    run([ub] * 5, ["ub0", "ub1", "ub2", "ub3", "ub4"], BP_ONBOOT_GROUNDCMD)
