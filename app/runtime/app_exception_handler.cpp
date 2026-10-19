#include "app/runtime/app_exception_handler.h"
#include "app/channels/app_fault_channel.hpp"

namespace
{
    app::FaultChannel *controlFaultChannel = nullptr;
    app::ControlChannels *controlChannels = nullptr;
}

void app::bindFaultChannel(FaultChannel *channel) { controlFaultChannel = channel; }
void app::bindControlChannels(ControlChannels *channels) { controlChannels = channels; }
bool app::postRequest(const Request &request)
{
    return controlChannels != nullptr && controlChannels->post(request);
}

namespace
{
    volatile bool PRIVATE_TEXT_EXCEPTION_ENABLE = false;

    volatile bool PRIVATE_DLX_EXCEPTION_ENABLE = false;

    volatile bool PRIVATE_RADIO_EXCEPTION_ENABLE = false;

    volatile bool PRIVATE_FLASH_EXCEPTION_ENABLE = false;
} // namespace

void ENABLE_EXCEPTION_TEXT()
{
    PRIVATE_TEXT_EXCEPTION_ENABLE = true;
}

void DISABLE_EXCEPTION_TEXT()
{
    PRIVATE_TEXT_EXCEPTION_ENABLE = false;
}

bool IS_EXCEPTION_TEXT()
{
    return PRIVATE_TEXT_EXCEPTION_ENABLE;
}

void TEXT_EXCEPTION_HANDLER(ExceptionCode code)
{
    logLine("[Exp ] ExceptionCode: 0x%04X\r\n", static_cast<unsigned>(code)); //不关心LogLine是否真的有效
}

void ENABLE_EXCEPTION_DLX()
{
    PRIVATE_DLX_EXCEPTION_ENABLE = true;
}

void DISABLE_EXCEPTION_DLX()
{
    PRIVATE_DLX_EXCEPTION_ENABLE = false;
}

bool IS_EXCEPTION_DLX()
{
    return PRIVATE_DLX_EXCEPTION_ENABLE;
}

void DLX_EXCEPTION_HANDLER(ExceptionCode code)
{
    ReleaseContext *ctx = logContext;
    if (ctx == nullptr || ctx->channel == nullptr || ctx->channel->exceptionMailbox == nullptr) {
        return; // 还没有人接这条渠道: 直接丢弃
    }
    ctx->channel->exceptionMailbox->push(code);
}

void ENABLE_EXCEPTION_RADIO()
{
    PRIVATE_RADIO_EXCEPTION_ENABLE = true;
}

void DISABLE_EXCEPTION_RADIO()
{
    PRIVATE_RADIO_EXCEPTION_ENABLE = false;
}

bool IS_EXCEPTION_RADIO()
{
    return PRIVATE_RADIO_EXCEPTION_ENABLE;
}

void RADIO_EXCEPTION_HANDLER(ExceptionCode code)
{
}

void ENABLE_EXCEPTION_FLASH()
{
    PRIVATE_FLASH_EXCEPTION_ENABLE = true;
}

void DISABLE_EXCEPTION_FLASH()
{
    PRIVATE_FLASH_EXCEPTION_ENABLE = false;
}

bool IS_EXCEPTION_FLASH()
{
    return PRIVATE_FLASH_EXCEPTION_ENABLE;
}

void FLASH_EXCEPTION_HANDLER(ExceptionCode code)
{
}

static void dispatchException(ExceptionCode code)
{
    if (controlFaultChannel != nullptr) controlFaultChannel->report(code);
    if (PRIVATE_TEXT_EXCEPTION_ENABLE) {
        TEXT_EXCEPTION_HANDLER(code);
    }
    if (PRIVATE_DLX_EXCEPTION_ENABLE) {
        DLX_EXCEPTION_HANDLER(code);
    }
    if (PRIVATE_FLASH_EXCEPTION_ENABLE) {
        FLASH_EXCEPTION_HANDLER(code);
    }
    if (PRIVATE_RADIO_EXCEPTION_ENABLE) {
        RADIO_EXCEPTION_HANDLER(code);
    }
}

ExceptionHandler EXCEPTION_HANDLE = dispatchException;

void DLX_HANDLE_EXCEPTION(ExceptionCode code)
{
    if (EXCEPTION_HANDLE != nullptr) EXCEPTION_HANDLE(code);
}

void DLX_HANDLE_EXCEPTION(ExceptionRecorder &rec)
{
    for (uint8_t i = 0; i < rec.size(); i++)
    {
        DLX_HANDLE_EXCEPTION(rec.at(i));
    }
}
