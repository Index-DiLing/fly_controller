#pragma once
#include <stdint.h>
#include "app/control/app_state.h"
#include "app/runtime/app_runtime_limits.hpp"
#include "dlx_queue.hpp"
#include "dlx_locked_cell.hpp"
#include "app/link/app_link_cmd.hpp"
#include "flight_control/flight_controller.hpp"

namespace app
{
enum class RequestSource : uint8_t { Local, Link, Console };
enum class ResultKind : uint8_t { Request, EmergencyStop };

struct Request
{
    Task task{Task::None};
    RequestSource source{RequestSource::Local};
    uint32_t id{0u};
    uint32_t issuedMs{0u};
    uint32_t safetyEpoch{0u};
    uint8_t wireSequence{0u};
    uint8_t wireOperation{0u};
};

struct RequestResult
{
    ResultKind kind{ResultKind::Request};
    uint32_t id{0u};
    GroundStatus status{GroundStatus::Ok};
    uint8_t wireSequence{0u};
    uint8_t wireOperation{0u};
};

struct EmergencyStopHandshake
{
    uint32_t requestedId{0u};
    bool pending{false};

    uint32_t request()
    {
        if (++requestedId == 0u) ++requestedId;
        pending = true;
        return requestedId;
    }

    bool acknowledge(uint32_t id)
    {
        if (!pending || id != requestedId) return false;
        pending = false;
        return true;
    }
};

struct ControlInput
{
    dlx::FlightControlSetpoint setpoint{};
    float manualThrottle{0.0f};
    float altitudeTargetM{0.0f};
    float velocityTargetMps[3]{};
    uint16_t motorValue[4]{};
    int16_t rollCounts{0};
    int16_t pitchCounts{0};
    uint16_t throttleCounts{0u};
    uint16_t command{9u};
    uint32_t receivedMs{0u};
    uint32_t sequence{0u};
    uint32_t motorSequence{0u};
    uint32_t motorReceivedMs{0u};
    uint32_t stickReceivedMs{0u};
    uint32_t velocityReceivedMs{0u};
    uint32_t emergencyStopId{0u};
    bool emergencyStopPending{false};
    bool valid{false};
    bool stickValid{false};
    bool velocityValid{false};
    bool altitudeValid{false};
};
inline bool controlInputFresh(const ControlInput &input, Mode mode, uint32_t nowMs)
{
    if (!input.valid) return false;
    if (mode == Mode::Manual) {
        return input.motorSequence != 0u && static_cast<uint32_t>(nowMs - input.motorReceivedMs) <= ControlFreshnessMs;
    }
    if (mode == Mode::VelHold && input.velocityValid &&
        static_cast<uint32_t>(nowMs - input.velocityReceivedMs) <= ControlFreshnessMs) return true;
    return input.stickValid && static_cast<uint32_t>(nowMs - input.stickReceivedMs) <= ControlFreshnessMs;
}

struct ControlChannels
{
    uint8_t requestStorage[RequestQueueCapacity * sizeof(Request)]{};
    uint8_t resultStorage[ResultQueueCapacity * sizeof(RequestResult)]{};
    dlx::ByteBuffer requestBuffer{requestStorage, static_cast<uint16_t>(sizeof(requestStorage))};
    dlx::ByteBuffer resultBuffer{resultStorage, static_cast<uint16_t>(sizeof(resultStorage))};
    dlx::MPSCQueue<Request> requests{requestBuffer};
    dlx::SPSCQueue<RequestResult> results{resultBuffer};
    dlx::LockedCell<ControlInput> input{};
    dlx::LockedCell<uint32_t> safetyEpoch{0u};

    bool post(const Request &request)
    {
        return isRequest(request.task) && request.task != Task::RequestEStop && requests.tryPush(request);
    }
};
}
