#pragma once
#include "dlx_bytebuffer.hpp"

namespace dlx
{
    struct IICBus {
        uint8_t registers[256]{};
        bool failRead{false};
        bool failWrite{false};
    };

    class IICDevice
    {
        IICBus &bus;

    public:
        IICDevice(IICBus &source, uint16_t)
            : bus(source)
        {
        }

        bool write(ByteBuffer &buffer)
        {
            if (bus.failWrite || buffer.used() < 2u) return false;
            const uint8_t address = buffer.src[0];
            for (uint16_t index = 1u; index < buffer.used(); ++index) {
                bus.registers[static_cast<uint8_t>(address + index - 1u)] = buffer.src[index];
            }
            return true;
        }

        bool read(ByteBuffer &output, ByteBuffer &command, uint16_t count)
        {
            if (bus.failRead || command.used() != 1u || output.remaining() < count) return false;
            const uint8_t address = command.src[0];
            for (uint16_t index = 0u; index < count; ++index) {
                output.write<uint8_t>(bus.registers[static_cast<uint8_t>(address + index)]);
            }
            return true;
        }
    };
}
