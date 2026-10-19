/* RAM bench program for probing the W25Q128 on SPI2 (PB10/PC2/PC3, CS=PD1).
 *
 * Started by pyOCD while the flight firmware is halted.  Results are written as
 * tagged u32 words to RAM at RESULT_ADDR and read back over the debug probe.
 * Build/load: tools/flash_bench/run_flash_test.py
 */
#include <stdint.h>

#define REG32(a) (*(volatile uint32_t *)(a))
#define R16(a) (*(volatile uint16_t *)(a))
#define R8(a) (*(volatile uint8_t *)(a))

#define RCC_BASE 0x40023800u
#define RCC_AHB1ENR (RCC_BASE + 0x30u)
#define RCC_APB1ENR (RCC_BASE + 0x40u)

#define GPIOB 0x40020400u
#define GPIOC 0x40020800u
#define GPIOD 0x40020C00u
#define MODER 0x00u
#define OSPEEDR 0x08u
#define PUPDR 0x0Cu
#define IDR 0x10u
#define ODR 0x14u
#define AFRL 0x20u
#define AFRH 0x24u

#define SPI2 0x40003800u
#define SPI2_CR1 (SPI2 + 0x00u)
#define SPI2_SR (SPI2 + 0x08u)
#define SPI2_DR (SPI2 + 0x0Cu)

#define RESULT_ADDR 0x2001B000u

#define TAG_START 0x74696E67u
#define TAG_JEDEC 0xA0000000u
#define TAG_SR1 0xA1000000u
#define TAG_DONE 0xDEADBEEFu

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

static void delay_cycles(volatile uint32_t c)
{
    while (c--) {
        __asm volatile("nop");
    }
}

static void cs(uint32_t low)
{
    if (low) {
        REG32(GPIOD + ODR) &= ~(1u << 1);
    } else {
        REG32(GPIOD + ODR) |= (1u << 1);
    }
}

static void spi_hw_init(uint32_t br_code)
{
    REG32(RCC_AHB1ENR) |= (1u << 1) | (1u << 2) | (1u << 3);
    REG32(RCC_APB1ENR) |= (1u << 14);
    (void)REG32(RCC_APB1ENR);

    /* PB10 = SPI2_SCK, AF5 */
    REG32(GPIOB + MODER) = (REG32(GPIOB + MODER) & ~(3u << 20)) | (2u << 20);
    REG32(GPIOB + AFRH) = (REG32(GPIOB + AFRH) & ~(0xFu << 8)) | (5u << 8);
    REG32(GPIOB + OSPEEDR) |= (3u << 20);
    /* PC2 = SPI2_MISO, AF5 */
    REG32(GPIOC + MODER) = (REG32(GPIOC + MODER) & ~(3u << 4)) | (2u << 4);
    REG32(GPIOC + AFRL) = (REG32(GPIOC + AFRL) & ~(0xFu << 8)) | (5u << 8);
    REG32(GPIOC + OSPEEDR) |= (3u << 4);
    /* PC3 = SPI2_MOSI, AF5 */
    REG32(GPIOC + MODER) = (REG32(GPIOC + MODER) & ~(3u << 6)) | (2u << 6);
    REG32(GPIOC + AFRL) = (REG32(GPIOC + AFRL) & ~(0xFu << 12)) | (5u << 12);
    REG32(GPIOC + OSPEEDR) |= (3u << 6);
    /* PD1 = /CS, output push-pull, high */
    REG32(GPIOD + MODER) = (REG32(GPIOD + MODER) & ~(3u << 2)) | (1u << 2);
    REG32(GPIOD + PUPDR) = (REG32(GPIOD + PUPDR) & ~(3u << 2)) | (1u << 2);
    REG32(GPIOD + OSPEEDR) |= (3u << 2);
    cs(0);

    R16(SPI2_CR1) = 0;
    R16(SPI2_CR1) = (uint16_t)(0x0004u | 0x0200u | 0x0100u | ((br_code & 7u) << 3));
    R16(SPI2_CR1) |= 0x0040u;
    (void)R16(SPI2_SR);
    (void)R16(SPI2_DR);
}

static uint8_t spi_swap(uint8_t d)
{
    volatile uint32_t guard = 200000u;
    while (!(R16(SPI2_SR) & 0x02u) && --guard) {
    }
    R8(SPI2_DR) = d;
    guard = 200000u;
    while (!(R16(SPI2_SR) & 0x01u) && --guard) {
    }
    return R8(SPI2_DR);
}

static uint32_t jedec_raw(void)
{
    (void)spi_swap(0x9Fu);
    uint32_t a = spi_swap(0x00), b = spi_swap(0x00), c = spi_swap(0x00);
    return (a << 16) | (b << 8) | c;
}
static uint32_t sr1_raw(void)
{
    (void)spi_swap(0x05u);
    return spi_swap(0x00);
}
static uint32_t jedec(void)
{
    cs(1);
    uint32_t v = jedec_raw();
    cs(0);
    return v;
}
static uint32_t sr1(void)
{
    cs(1);
    uint32_t v = sr1_raw();
    cs(0);
    return v;
}
static void flash_soft_reset(void)
{
    cs(1);
    (void)spi_swap(0x66u);
    cs(0);
    cs(1);
    (void)spi_swap(0x99u);
    cs(0);
    delay_cycles(2000);
}
static void release_pd(void)
{
    cs(1);
    (void)spi_swap(0xABu);
    (void)spi_swap(0x00);
    (void)spi_swap(0x00);
    (void)spi_swap(0x00);
    cs(0);
    delay_cycles(2000);
}

/* plain reads, optionally with a gap / CS retoggles before each */
static void read_seq(uint32_t tag, uint32_t n, uint32_t cycles_gap, uint32_t cs_toggles)
{
    for (uint32_t i = 0; i < n; ++i) {
        for (uint32_t k = 0; k < cs_toggles; ++k) {
            cs(1);
            cs(0);
        }
        put(tag | (i << 16) | jedec());
        if (cycles_gap) {
            delay_cycles(cycles_gap);
        }
    }
}

void ram_main(void)
{
    __asm volatile("cpsid i" ::: "memory");
    out = (uint32_t *)RESULT_ADDR;
    oi = 0;
    put(TAG_START);
    put(0x00000003u);

    spi_hw_init(2);

    put(0xF1000000u | (REG32(RCC_BASE + 0x08u) & 0x00FFFFFFu));
    put(0xF2000000u | (REG32(RCC_BASE + 0x30u) & 0x00FFFFFFu));
    put(0xF3000000u | R16(SPI2_CR1));

    /* A: baseline - 8 plain reads */
    read_seq(0xA0000000u, 8, 0, 0);

    /* B: long gap (0.5 ms @16MHz = 8000 cycles) between reads */
    read_seq(0xB0000000u, 4, 8000u, 0);

    /* C: 10 CS pulses (~100 ns each) before each read */
    read_seq(0xC0000000u, 4, 0, 10);

    /* D: flash software reset (66h/99h) + release power-down, then reads */
    flash_soft_reset();
    release_pd();
    read_seq(0xD0000000u, 4, 0, 0);

    /* E: SR1 reads only (is the chip still answering 0x05?) */
    for (uint32_t i = 0; i < 4; ++i) {
        put(TAG_SR1 | (i << 16) | sr1());
        delay_cycles(1000);
    }

    /* F: WRSR -> immediate SR1/JEDEC (the tW race) */
    {
        cs(1);
        (void)spi_swap(0x06u); /* write enable */
        cs(0);
        cs(1);
        (void)spi_swap(0x01u); /* write status 1 */
        (void)spi_swap(0x00u);
        cs(0);
        put(0xE0000000u);
        put(TAG_SR1 | sr1());
        put(TAG_JEDEC | jedec());
        put(TAG_SR1 | sr1());
        put(TAG_JEDEC | jedec());
        uint32_t n = 0;
        while ((sr1() & 1u) && n < 100000u) {
            ++n;
        }
        put(0xE1000000u | n);
        put(TAG_JEDEC | jedec());
    }

    put(TAG_DONE);
    while (1) {
        __asm volatile("nop");
    }
}
