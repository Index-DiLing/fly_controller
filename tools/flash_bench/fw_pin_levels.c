/* RAM probe: board default levels of the W25Q128 control pins.
 *
 * PD1=/CS  PB10=SCK  PC2=MISO  PC3=MOSI.  All are left as inputs; the IDR value
 * shows the external pull resistor (if any) that defines their level while the
 * MCU is in reset / before the driver configures them.  A /CS that floats low
 * lets the flash latch random commands from noise on SCK during reset/power-up.
 */
#include <stdint.h>

#define REG32(a) (*(volatile uint32_t *)(a))
#define RCC_BASE 0x40023800u
#define GPIOB 0x40020400u
#define GPIOC 0x40020800u
#define GPIOD 0x40020C00u
#define MODER 0x00u
#define PUPDR 0x0Cu
#define IDR 0x10u

#define RESULT_ADDR 0x2001B000u

static volatile uint32_t *out;
static uint32_t oi;
static void put(uint32_t v) { out[oi++] = v; }

void *memcpy(void *d, const void *s, unsigned n)
{
    uint8_t *dd = (uint8_t *)d;
    const uint8_t *ss = (const uint8_t *)s;
    while (n--) {
        *dd++ = *ss++;
    }
    return d;
}
void *memset(void *d, int c, unsigned n)
{
    uint8_t *dd = (uint8_t *)d;
    while (n--) {
        *dd++ = (uint8_t)c;
    }
    return d;
}

static void delay(volatile uint32_t c)
{
    while (c--) {
        __asm volatile("nop");
    }
}

static void set_pupd(uint32_t base, uint32_t pin, uint32_t mode)
{
    REG32(base + PUPDR) = (REG32(base + PUPDR) & ~(3u << (2u * pin))) | (mode << (2u * pin));
}

void ram_main(void)
{
    __asm volatile("cpsid i" ::: "memory");
    out = (uint32_t *)RESULT_ADDR;
    oi = 0;
    put(0x74696E67u);
    put(0x00000007u);

    /* enable GPIOB/C/D clocks, keep every flash pin an input */
    REG32(RCC_BASE + 0x30u) |= (1u << 1) | (1u << 2) | (1u << 3);
    (void)REG32(RCC_BASE + 0x30u);
    REG32(GPIOB + MODER) &= ~(3u << 20);  /* PB10 input */
    REG32(GPIOC + MODER) &= ~((3u << 4) | (3u << 6)); /* PC2, PC3 input */
    REG32(GPIOD + MODER) &= ~(3u << 2);   /* PD1 input */
    set_pupd(GPIOB, 10, 0);
    set_pupd(GPIOC, 2, 0);
    set_pupd(GPIOC, 3, 0);
    set_pupd(GPIOD, 1, 0);
    delay(200000);

    put(0xA0000000u | (REG32(GPIOD + IDR) & 0xFFu) << 8 | (REG32(GPIOD + IDR) & 0xFFu));
    put(0xA1000000u | ((REG32(GPIOD + IDR) >> 1) & 1u));   /* /CS no pull */
    put(0xA2000000u | ((REG32(GPIOB + IDR) >> 10) & 1u));  /* SCK no pull */
    put(0xA3000000u | ((REG32(GPIOC + IDR) >> 2) & 1u));   /* MISO no pull */
    put(0xA4000000u | ((REG32(GPIOC + IDR) >> 3) & 1u));   /* MOSI no pull */

    /* PD1 with internal pull-down, then pull-up: reveals external resistors */
    set_pupd(GPIOD, 1, 2);
    delay(200000);
    put(0xA5000000u | ((REG32(GPIOD + IDR) >> 1) & 1u)); /* /CS pulldown */
    set_pupd(GPIOD, 1, 1);
    delay(200000);
    put(0xA6000000u | ((REG32(GPIOD + IDR) >> 1) & 1u)); /* /CS pullup */

    /* SCK with pull-down/up */
    set_pupd(GPIOB, 10, 2);
    delay(200000);
    put(0xA7000000u | ((REG32(GPIOB + IDR) >> 10) & 1u));
    set_pupd(GPIOB, 10, 1);
    delay(200000);
    put(0xA8000000u | ((REG32(GPIOB + IDR) >> 10) & 1u));

    put(0xDEADBEEFu);
    while (1) {
        __asm volatile("nop");
    }
}
