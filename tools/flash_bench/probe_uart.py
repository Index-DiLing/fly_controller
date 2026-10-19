"""Serial<->MCU connectivity test: halt core, send bytes on COM11, inspect USART1 SR/DR.

If the CH343 on COM11 is really wired to USART1 (PA9/PA10), sending bytes while the
core is halted must set RXNE (or ORE) in USART1->SR.  This distinguishes
"wrong COM port / no wire" from "firmware ignores the command".
"""
import sys
import time

import serial
from pyocd.core.helpers import ConnectHelper

HERE = __file__.rsplit("\\", 1)[0] if "\\" in __file__ else "."
COM = "COM11"
USART1 = 0x40011000
SR, DR = USART1 + 0x00, USART1 + 0x04
CR1 = USART1 + 0x0C
BAUD_BITS = {0x0000: "9600?", 0x10: "?"}
BRR_MAP = {0x2A: 2000000, 0xD0: 115200}


def main(port=COM, baud=2000000, nbytes=4):
    s = ConnectHelper.session_with_chosen_probe(
        options={"target_override": "cortex_m", "frequency": 4000000})
    with s:
        t = s.target
        t.halt()
        cr1 = t.read16(CR1)
        brr = t.read16(USART1 + 0x08)
        print("USART1 CR1=0x%04X BRR=0x%04X (=> %s baud)" % (cr1, brr, BRR_MAP.get(brr, "?")))
        print("SR before: 0x%04X" % t.read16(SR))
        # drain any stale byte
        if t.read16(SR) & 0x20:
            t.read16(DR)
            print("drained stale RXNE, SR now 0x%04X" % t.read16(SR))
        ser = serial.Serial(port, baud, timeout=0.1)
        ser.write(bytes([0x55, 0xAA, 0x5A, 0xA5])[:nbytes])
        ser.flush()
        time.sleep(0.15)
        sr = t.read16(SR)
        print("SR after  : 0x%04X  (RXNE=%d ORE=%d FE=%d NE=%d TC=%d TXE=%d)"
              % (sr, (sr >> 5) & 1, (sr >> 3) & 1, (sr >> 1) & 1, sr & 1, (sr >> 6) & 1, (sr >> 7) & 1))
        if sr & 0x20:
            print("DR        : 0x%04X" % t.read16(DR))
            sr = t.read16(SR)
            print("SR after DR read: 0x%04X" % sr)
        # TX side: is the pin actually driving?  read GPIOA MODER/ODR for PA9/PA10
        print("GPIOA MODER=0x%08X ODR=0x%08X" % (t.read32(0x40020000), t.read32(0x40020014)))
        ser.close()
        t.resume()
        print("resumed")


if __name__ == "__main__":
    main()
