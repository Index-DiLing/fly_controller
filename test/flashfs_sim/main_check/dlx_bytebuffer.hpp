#pragma once
#include <stdint.h>
namespace dlx
{
    class ByteBuffer
    {
    public:
        ByteBuffer(uint8_t *, uint16_t) {}
    };
    class RingByteBuffer
    {
    public:
        explicit RingByteBuffer(ByteBuffer &) {}
        uint16_t available() const { return 0u; }
    };
}
