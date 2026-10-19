"""Check the board's default (un-driven) levels of the SPI2 flash pins during boot.

Reset the MCU and halt it while it is still in the "ground mode wait" loop, i.e.
before any SPI2 pin is configured.  The GPIO IDR then shows whether the board has
external pull-up/down resistors on /CS, SCK, MISO, MOSI.  A floating /CS during
reset/power-up lets the W25Q128 latch spurious commands from noise.
"""
import time

from pyocd.core.helpers import ConnectHelper

GPIOB, GPIOC, GPIOD = 0x40020400, 0x40020800, 0x40020C00
IDR, ODR, PUPDR, MODER = 0x10, 0x14, 0x0C, 0x00


def main():
    s = ConnectHelper.session_with_chosen_probe(
        options={"target_override": "cortex_m", "frequency": 4000000})
    with s:
        t = s.target
        t.reset()
        time.sleep(0.25)  # let the firmware reach decideBootMode() (about 5 ms after reset)
        t.halt()
        print("state:", t.get_state())
        print("PC=0x%08X" % t.read_core_register("pc"))
        print("GPIOB MODER=0x%08X PUPDR=0x%08X IDR=0x%08X" %
              (t.read32(GPIOB + MODER), t.read32(GPIOB + PUPDR), t.read32(GPIOB + IDR)))
        print("GPIOC MODER=0x%08X PUPDR=0x%08X IDR=0x%08X" %
              (t.read32(GPIOC + MODER), t.read32(GPIOC + PUPDR), t.read32(GPIOC + IDR)))
        print("GPIOD MODER=0x%08X PUPDR=0x%08X IDR=0x%08X" %
              (t.read32(GPIOD + MODER), t.read32(GPIOD + PUPDR), t.read32(GPIOD + IDR)))
        print()
        print("flash pins: PD1=/CS PB10=SCK PC2=MISO PC3=MOSI")
        idd = t.read32(GPIOD + IDR)
        idb = t.read32(GPIOB + IDR)
        idc = t.read32(GPIOC + IDR)
        print("  /CS  PD1 = %d" % ((idd >> 1) & 1))
        print("  SCK  PB10= %d" % ((idb >> 10) & 1))
        print("  MISO PC2 = %d" % ((idc >> 2) & 1))
        print("  MOSI PC3 = %d" % ((idc >> 3) & 1))
        t.resume()


if __name__ == "__main__":
    main()
