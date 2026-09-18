#pragma once
#include <stdint.h>
#include <stdio.h>
#include "dlx_bytebuffer.hpp"
#include "dlx_dma.hpp"

// 主机端自检用的 USART 桩: 文本日志直接打到终端
namespace dlx
{
    enum class USARTModeProfile
    {
        WL8_SB1_PN_RXTX_FCN,
    };

    class USART
    {
    public:
        static USART USART1_TA9_RAA() { return USART(); }
        void         init(USARTModeProfile, uint32_t, RingByteBuffer &) {}
        void         send(const uint8_t *data, uint16_t len)
        {
            fwrite(data, 1u, len, stdout);
            fflush(stdout);
        }
        DMA setDMASend(ByteBuffer &) { return DMA(); }
    };
} // namespace dlx
