#pragma once
#include "app/runtime/app_runtime_limits.hpp"
#include "app/channels/app_control_channel.hpp"
#include "dlx_exception.h"
#include "dlx_queue.hpp"
#include "dlx_locked_cell.hpp"

namespace app
{
enum : uint32_t { RefreshHealth = 1u, StopOutput = 2u };

inline uint32_t controlFaultAction(dlx::ExceptionCode code)
{
    using dlx::ExceptionCode;
    switch (code) {
    case ExceptionCode::POWER_VOLTAGE_CUT: return StopOutput;
    case ExceptionCode::IMU_SPI_ERROR:
    case ExceptionCode::IMU_DATA_STALE:
    case ExceptionCode::IMU_FIFO_OVERRUN:
    case ExceptionCode::BARO_READ_FAILED:
    case ExceptionCode::BARO_DATA_STALE:
    case ExceptionCode::RANGE_DATA_STALE:
    case ExceptionCode::FLOW_NOT_FOUND:
    case ExceptionCode::FLOW_SPI_GARBAGE:
    case ExceptionCode::GNSS_NO_FIX:
    case ExceptionCode::POWER_SAMPLE_FAILED:
    case ExceptionCode::LINK_DESYNC:
    case ExceptionCode::LINK_RESYNCED: return RefreshHealth;
    default: return 0u;
    }
}

class FaultChannel
{
    uint8_t storage[FaultQueueCapacity * sizeof(dlx::ExceptionCode)]{};
    dlx::ByteBuffer buffer{storage, static_cast<uint16_t>(sizeof(storage))};
    dlx::MPSCQueue<dlx::ExceptionCode> events{buffer};
    dlx::LockedCell<uint32_t> pendingActions{0u};
public:
    void report(dlx::ExceptionCode code)
    {
        const uint32_t action = controlFaultAction(code);
        if (action == 0u) return;
        pendingActions.update([&](uint32_t &pending) { pending |= action; });
        events.tryPush(code);
    }
    uint32_t takeActions() { return pendingActions.exchange(0u); }
    bool take(dlx::ExceptionCode &code) { return events.tryPop(code); }
    uint32_t dropped() const { return events.dropped(); }
};

void bindFaultChannel(FaultChannel *channel);
void bindControlChannels(ControlChannels *channels);
bool postRequest(const Request &request);
}
