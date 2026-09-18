#pragma once
#include <stdint.h>

// 主机端自检用的 DMA 桩
namespace dlx
{
    class DMA
    {
    public:
        void wait() {}
        void reset(uint16_t = 0) {}
        void start() {}
    };
} // namespace dlx
