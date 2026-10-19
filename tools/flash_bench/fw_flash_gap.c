/* RAM probe: is the "deaf" state caused by back-to-back transactions?
 * Phase 1: 5 JEDEC reads with a 1 ms idle gap   -> expect all OK
 * Phase 2: 5 JEDEC reads back-to-back           -> watch for the deaf state
 * Phase 3: 5 JEDEC reads with 100 us gap
 * Phase 4: 5 JEDEC reads with 10 ms gap
 */
#include <stdint.h>

#define REG32(a) (*(volatile uint32_t *)(a))
#define R16(a) (*(volatile uint16_t *)(a))
#define R8(a) (*(volatile uint8_t *)(a))

#define RCC_BASE 0x40023800u
#define GPIOB 0x40020400u
#define GPIOC 0x40020800u
#define GPIOD 0x40020C00u
#define MODER 0x00u
#define OSPEEDR 0x08u
#define PUPDR 0x0Cu
#define ODR 0x14u
#define AFRL 0x20u
#define AFRH 0x24u
#define IDR 0x10u

#define SPI2 0x40003800u
#define SPI2_CR1 (SPI2 + 0x00u)
#define SPI2_SR (SPI2 + 0x08u)
#define SPI2_DR (SPI2 + 0x0Cu)

#define RESULT_ADDR 0x2001B000u
/* CYCCNT @16MHz HSI: 1ms = 16000 cycles */

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

static void spi_hw_init(void)
{
    REG32(RCC_BASE + 0x30u) |= (1u << 1) | (1u << 2) | (1u << 3);
    REG32(RCC_BASE + 0x40u) |= (1u << 14);
    (void)REG32(RCC_BASE + 0x40u);

    REG32(GPIOB + MODER) = (REG32(GPIOB + MODER) & ~(3u << 20)) | (2u << 20);
    REG32(GPIOB + AFRH) = (REG32(GPIOB + AFRH) & ~(0xFu << 8)) | (5u << 8);
    REG32(GPIOB + OSPEEDR) |= (3u << 20);
    REG32(GPIOC + MODER) = (REG32(GPIOC + MODER) & ~(3u << 4)) | (2u << 4);
    REG32(GPIOC + AFRL) = (REG32(GPIOC + AFRL) & ~(0xFu << 8)) | (5u << 8);
    REG32(GPIOC + OSPEEDR) |= (3u << 4);
    REG32(GPIOC + MODER) = (REG32(GPIOC + MODER) & ~(3u << 6)) | (2u << 6);
    REG32(GPIOC + AFRL) = (REG32(GPIOC + AFRL) & ~(0xFu << 12)) | (5u << 12);
    REG32(GPIOC + OSPEEDR) |= (3u << 6);
    REG32(GPIOD + MODER) = (REG32(GPIOD + MODER) & ~(3u << 2)) | (1u << 2);
    REG32(GPIOD + PUPDR) = (REG32(GPIOD + PUPDR) & ~(3u << 2)) | (1u << 2);
    REG32(GPIOD + OSPEEDR) |= (3u << 2);
    cs(0);

    R16(SPI2_CR1) = 0;
    R16(SPI2_CR1) = (uint16_t)(0x0004u | 0x0200u | 0x0100u | (2u << 3));
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

static uint32_t jedec(void)
{
    cs(1);
    (void)spi_swap(0x9Fu);
    uint32_t a = spi_swap(0), b = spi_swap(0), c = spi_swap(0);
    cs(0);
    return (a << 16) | (b << 8) | c;
}

static void phase(uint32_t phase_id, uint32_t n, uint32_t gap)
{
    for (uint32_t i = 0; i < n; ++i) {
        uint32_t v = jedec();
        put(0xA0000000u | (phase_id << 20) | (i << 16) | v);
        put(0x50000000u | (phase_id << 20) | (i << 16) | (R16(SPI2_SR) & 0xFFu));
        if (gap) {
            delay_cycles(gap);
        }
    }
}

void ram_main(void)
{
    __asm volatile("cpsid i" ::: "memory");
    out = (uint32_t *)RESULT_ADDR;
    oi = 0;
    put(0x74696E67u);
    put(0x00000006u);
    spi_hw_init();

    phase(1, 5, 16000u);  /* 1 ms gaps */
    phase(2, 5, 0u);      /* back-to-back */
    phase(3, 5, 1600u);   /* 100 us gaps */
    phase(4, 5, 160000u); /* 10 ms gaps */

    put(0xDEADBEEFu);
    while (1) {
        __asm volatile("nop");
    }
}
