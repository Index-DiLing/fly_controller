#pragma once
#include <stdint.h>
#include <stdio.h>
#include "dlx_bytebuffer.hpp"
#include "dlx_nvic_it.h"
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
        void init(USARTModeProfile, uint32_t, RingByteBuffer &) {}
        void send(const uint8_t *data, uint16_t len)
        {
            fwrite(data, 1u, len, stdout); // 主机跑测: 直接打到终端
            fflush(stdout);
        }
    };
}
