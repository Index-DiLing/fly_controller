/* Minimal RAM probe: read JEDEC + SR1/SR2/SR3 (SR2 bit1 = QE, decides whether
 * /HOLD and /WP pins are functionally active -> floating-pin failure mode).
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

#define SPI2 0x40003800u
#define SPI2_CR1 (SPI2 + 0x00u)
#define SPI2_SR (SPI2 + 0x08u)
#define SPI2_DR (SPI2 + 0x0Cu)

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

static uint32_t cmd_read24(uint8_t cmd)
{
    cs(1);
    (void)spi_swap(cmd);
    uint32_t a = spi_swap(0), b = spi_swap(0), c = spi_swap(0);
    cs(0);
    return (a << 16) | (b << 8) | c;
}
static uint32_t cmd_read8(uint8_t cmd)
{
    cs(1);
    (void)spi_swap(cmd);
    uint32_t v = spi_swap(0);
    cs(0);
    return v;
}

void ram_main(void)
{
    __asm volatile("cpsid i" ::: "memory");
    out = (uint32_t *)RESULT_ADDR;
    oi = 0;
    put(0x74696E67u);
    put(0x00000004u);
    spi_hw_init();

    put(0xA0000000u | cmd_read24(0x9F)); /* JEDEC */
    put(0xA1000000u | cmd_read8(0x05));  /* SR1 */
    put(0xA2000000u | cmd_read8(0x35));  /* SR2: bit1=QE, bit6=CMP, bit0=SRL */
    put(0xA3000000u | cmd_read8(0x15));  /* SR3 */
    put(0xA4000000u | cmd_read24(0x9F)); /* JEDEC again */
    put(0xA5000000u | cmd_read24(0x90)); /* device ID (90h) */
    put(0xA6000000u | cmd_read24(0x9F)); /* JEDEC again */
    put(0xDEADBEEFu);
    while (1) {
        __asm volatile("nop");
    }
}
