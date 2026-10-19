#pragma once
#include "app/channels/app_log_channel.hpp"

namespace dlx
{
    template <typename Protocol>
    void appendLogMailbox(LogChannel *channel, Protocol &protocol, ByteBuffer &buffer)
    {
        if (channel == nullptr) return;
        if (channel->exceptionMailbox != nullptr) {
            const uint16_t room    = static_cast<uint16_t>(buffer.remaining() / 4u);
            const uint8_t capacity = static_cast<uint8_t>(room < logchannel::EXCEPTION_SLOTS ? room : logchannel::EXCEPTION_SLOTS);
            ExceptionCode codes[logchannel::EXCEPTION_SLOTS];
            const uint8_t count = capacity == 0u ? 0u : channel->exceptionMailbox->take(codes, capacity);
            for (uint8_t index = 0u; index < count; ++index) protocol.DLXExceptionW(static_cast<uint16_t>(codes[index]));
            if (buffer.remaining() >= 4u && channel->exceptionMailbox->takeOverflow()) {
                protocol.DLXExceptionW(static_cast<uint16_t>(ExceptionCode::EXCEPTION_OVERFLOW));
            }
        }
        if (channel->textMailbox == nullptr || buffer.remaining() < textmbox::POOL_BYTES + 4u) return;
        const StringPool *batch = channel->textMailbox->acquire();
        if (batch == nullptr) return;
        const uint16_t size = batch->used();
        if (size > 0u && buffer.remaining() >= size + 4u) {
            protocol.SimpleLogW(reinterpret_cast<int8_t *>(const_cast<uint8_t *>(batch->data())), size);
        } else {
            channel->textMailbox->noteDropped(size);
        }
        channel->textMailbox->release();
    }
}
