"""One-shot MCU state dump over ST-Link (pyOCD)."""
import sys
from pyocd.core.helpers import ConnectHelper

REGS = ["r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
        "r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc", "xpsr"]

RCC = 0x40023800
GPIOB, GPIOC, GPIOD = 0x40020400, 0x40020800, 0x40020C00
SPI2 = 0x40003800
USART1 = 0x40011000


def main():
    s = ConnectHelper.session_with_chosen_probe(
        options={"target_override": "cortex_m", "frequency": 4000000})
    with s:
        t = s.target
        t.halt()
        print("state:", t.get_state())
        for r in REGS:
            try:
                print("%4s = 0x%08X" % (r, t.read_core_register(r) & 0xFFFFFFFF))
            except Exception as e:
                print(r, "?", e)
        def w(name, addr, size=32):
            v = t.read32(addr) if size == 32 else t.read16(addr)
            print("%-16s @0x%08X = 0x%0*X" % (name, addr, size // 4, v))
        w("RCC_CR", RCC + 0x00)
        w("RCC_CFGR", RCC + 0x08)
        w("RCC_AHB1ENR", RCC + 0x30)
        w("RCC_APB1ENR", RCC + 0x40)
        w("RCC_APB2ENR", RCC + 0x44)
        w("RCC_CSR", RCC + 0x74)
        for g, n in ((GPIOB, "B"), (GPIOC, "C"), (GPIOD, "D")):
            w("GPIO%s_MODER" % n, g + 0x00)
            w("GPIO%s_ODR" % n, g + 0x14)
        for i in range(0, 0x0C, 2):
            w("SPI2_%02X" % (i), SPI2 + i, 16)
        w("SPI2_CR2", SPI2 + 0x04, 16)
        w("SPI2_SR", SPI2 + 0x08, 16)
        w("USART1_CR1", USART1 + 0x0C, 16)
        w("USART1_BRR", USART1 + 0x08, 16)
        t.resume()
        print("resumed")


if __name__ == "__main__":
    main()
