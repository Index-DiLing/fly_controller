#pragma once
//=============================================================================
// 主机测试用的 W25Q128 模拟实现: 只实现 FlashManager 用到的接口。
// 语义尽量贴近真实芯片: 编程只能把 1 写成 0, 擦除整扇区/整块为 0xFF,
// 并且可以注入"写一半掉电"。
//=============================================================================
#include <stdint.h>
#include <string.h>
#include "dlx_w25q128_config.h"

namespace sim
{
    struct Chip
    {
        static const uint32_t kSize = 16u * 1024u * 1024u;
        uint8_t mem[kSize];
        long killBudget = -1;   // >=0: 还能编程多少字节, 用完就模拟掉电(部分写入后中断)
        bool killed = false;
        long programCalls = 0;
        long eraseCalls = 0;
        long readCalls = 0;
        uint32_t jedec = 0xEF4018u;

        void eraseAll()
        {
            memset(mem, 0xFF, kSize);
        }

        bool program(uint32_t addr, const uint8_t *data, uint32_t len)
        {
            ++programCalls;
            for (uint32_t i = 0; i < len; ++i) {
                if (killBudget == 0) {
                    killed = true;   // 掉电: 后面的字节没写进去, 函数也不会返回
                    return false;
                }
                if (killBudget > 0) {
                    --killBudget;
                }
                mem[(addr + i) % kSize] &= data[i];
            }
            return true;
        }

        void eraseRange(uint32_t addr, uint32_t len)
        {
            ++eraseCalls;
            memset(mem + addr, 0xFF, len);
        }
    };
}

namespace dlx
{
    class W25Q128
    {
    public:
        W25Q128(sim::Chip &chipRef) : chip(chipRef) {}

        bool init() { return true; }
        uint32_t readJEDEC() { return chip.jedec; }

        bool read(uint32_t addr, uint8_t *data, uint32_t len)
        {
            if (data == 0 || len == 0) {
                return false;
            }
            ++chip.readCalls;
            memcpy(data, chip.mem + (addr & W25Q128_ADDR_MASK), len);
            return true;
        }

        bool write(uint32_t addr, const uint8_t *data, uint32_t len)
        {
            if (data == 0 || len == 0) {
                return false;
            }
            return chip.program(addr & W25Q128_ADDR_MASK, data, len);
        }

        bool eraseSector(uint32_t addr)
        {
            chip.eraseRange(addr & W25Q128_ADDR_MASK, W25Q128_SECTOR_SIZE);
            return true;
        }

        bool eraseBlock64K(uint32_t addr)
        {
            chip.eraseRange(addr & W25Q128_ADDR_MASK, W25Q128_BLOCK64K_SIZE);
            return true;
        }

        bool eraseChip(uint32_t = 0)
        {
            chip.eraseAll();
            return true;
        }

    private:
        sim::Chip &chip;
    };
} // namespace dlx
