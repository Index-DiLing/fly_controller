#pragma once
// 语法检查 + 主机跑测用的模拟 W25Q128(内存里 16MB, 支持按字节编程/擦除)
#include <stdint.h>
#include <string.h>
#include "dlx_spi.hpp"
#include "dlx_w25q128_config.h"

namespace dlx
{
    class W25Q128
    {
    public:
        explicit W25Q128(SPI &) {}

        bool init() { return true; }
        uint32_t readJEDEC() { return 0xEF4018u; }

        bool read(uint32_t addr, uint8_t *data, uint32_t len)
        {
            memcpy(data, mem() + addr, len);
            return true;
        }

        bool write(uint32_t addr, const uint8_t *data, uint32_t len)
        {
            for (uint32_t i = 0; i < len; ++i) {
                mem()[addr + i] &= data[i];
            }
            return true;
        }

        bool eraseSector(uint32_t addr)
        {
            memset(mem() + addr, 0xFF, W25Q128_SECTOR_SIZE);
            return true;
        }

        bool eraseBlock64K(uint32_t addr)
        {
            memset(mem() + addr, 0xFF, W25Q128_BLOCK64K_SIZE);
            return true;
        }

        bool eraseChip(uint32_t = 0u)
        {
            memset(mem(), 0xFF, W25Q128_CAPACITY);
            return true;
        }

    private:
        static uint8_t *mem()
        {
            static uint8_t storage[W25Q128_CAPACITY];
            static bool first = true;
            if (first) {
                memset(storage, 0xFF, sizeof(storage));
                first = false;
            }
            return storage;
        }
    };
} // namespace dlx
