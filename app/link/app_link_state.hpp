#pragma once

#include "app/control/app_state.h"

namespace app
{

struct WireState
{
    uint8_t lifecycle{0};
    uint8_t arming{0};
    uint8_t mode{0};
    uint8_t task{0};
};

inline WireState deriveWire(const Snapshot &snapshot)
{
    WireState wire;
    switch (snapshot.state) {
    case SystemState::Boot:
        wire.lifecycle = 0u;
        wire.arming    = 0u;
        break;
    case SystemState::InitLow:
        wire.lifecycle = 1u;
        wire.arming    = 0u;
        break;
    case SystemState::InitHigh:
        wire.lifecycle = 2u;
        wire.arming    = 0u;
        break;
    case SystemState::Standby:
        wire.lifecycle = 3u;
        wire.arming    = 0u;
        break;
    case SystemState::Maintenance:
        wire.lifecycle = 5u;
        wire.arming    = 0u;
        break;
    case SystemState::Arming:
        wire.lifecycle = 4u;
        wire.arming    = 1u;
        break;
    case SystemState::Armed:
        wire.lifecycle = 4u;
        wire.arming    = 2u;
        break;
    }
    switch (snapshot.mode) {
    case Mode::Manual:    wire.mode = 0u; break;
    case Mode::Stabilize: wire.mode = 1u; break;
    case Mode::AltHold:   wire.mode = 2u; break;
    case Mode::VelHold:   wire.mode = 3u; break;
    }
    switch (snapshot.operation) {
    case Operation::None:    wire.task = 0u; break;
    case Operation::Takeoff: wire.task = 1u; break;
    case Operation::Land:    wire.task = 2u; break;
    }
    return wire;
}

}
