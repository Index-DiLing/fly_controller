"""Live check: does the USART1 RX interrupt actually consume bytes while running?"""
import time

import serial
from pyocd.core.helpers import ConnectHelper

COM, BAUD = "COM11", 2000000
USART1 = 0x40011000
NVIC_ISER1 = 0xE000E104


def main():
    s = ConnectHelper.session_with_chosen_probe(
        options={"target_override": "cortex_m", "frequency": 4000000})
    with s:
        t = s.target
        t.halt()
        iser1 = t.read32(NVIC_ISER1)
        print("NVIC_ISER1 = 0x%08X  USART1(IRQ37) enabled=%d" % (iser1, (iser1 >> 5) & 1))
        print("USART1 CR1 = 0x%04X SR=0x%04X" % (t.read16(USART1 + 0x0C), t.read16(USART1)))
        pc = t.read_core_register("pc")
        print("PC before = 0x%08X" % pc)
        t.resume()

        ser = serial.Serial(COM, BAUD, timeout=0.1)
        ser.write(bytes([0x55]))
        ser.flush()
        time.sleep(0.05)
        t.halt()
        sr = t.read16(USART1)
        print("after 1 byte: SR=0x%04X (RXNE=%d ORE=%d)" % (sr, (sr >> 5) & 1, (sr >> 3) & 1))
        if sr & 0x20:
            print("  DR=0x%04X  <-- ISR did NOT consume it" % t.read16(USART1 + 0x04))
        else:
            print("  ISR consumed the byte OK")
        print("PC after  = 0x%08X" % t.read_core_register("pc"))
        t.resume()
        ser.close()
    print("done")


if __name__ == "__main__":
    main()
