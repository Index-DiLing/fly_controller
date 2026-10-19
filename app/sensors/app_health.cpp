#include "app/sensors/app_health.hpp"

namespace dlx
{
bool sampleFresh(uint32_t nowMs, uint32_t sampleMs, uint32_t maxAgeMs)
{
    return static_cast<uint32_t>(nowMs - sampleMs) <= maxAgeMs;
}

uint32_t healthMaxAgeMs(SensorHealth source)
{
    switch (source) {
    case SensorHealth::LINK: return app::ControlFreshnessMs;
    case SensorHealth::IMU: return app::ImuFreshnessMs;
    case SensorHealth::BARO: return app::BaroFreshnessMs;
    case SensorHealth::FLOW: return app::FlowFreshnessMs;
    case SensorHealth::RANGE: return app::RangeFreshnessMs;
    case SensorHealth::GNSS: return app::GnssFreshnessMs;
    case SensorHealth::POWER: return app::PowerFreshnessMs;
    default: return 0u;
    }
}

bool HealthSnapshot::ok(SensorHealth source, uint32_t nowMs) const
{
    const uint8_t index = static_cast<uint8_t>(source);
    if (index >= static_cast<uint8_t>(SensorHealth::TOTAL_SIZE)) return false;
    const HealthState &state = sources[index];
    return state.hasSample && state.available && sampleFresh(nowMs, state.lastSuccessMs, healthMaxAgeMs(source));
}

uint8_t HealthSnapshot::bits(uint32_t nowMs) const
{
    uint8_t result = 0u;
    for (uint8_t index = 0u; index < static_cast<uint8_t>(SensorHealth::TOTAL_SIZE); ++index) {
        if (ok(static_cast<SensorHealth>(index), nowMs)) result |= static_cast<uint8_t>(1u << index);
    }
    return result;
}

bool Health::success(SensorHealth source, uint32_t sampleMs)
{
    const uint8_t index = static_cast<uint8_t>(source);
    if (index >= static_cast<uint8_t>(SensorHealth::TOTAL_SIZE)) return false;
    bool changed = false;
    sources[index].update([&](HealthState &state) {
        changed = !state.available;
        state.lastSuccessMs = sampleMs;
        state.hasSample = true;
        state.available = true;
    });
    return changed;
}

bool Health::failure(SensorHealth source)
{
    const uint8_t index = static_cast<uint8_t>(source);
    if (index >= static_cast<uint8_t>(SensorHealth::TOTAL_SIZE)) return false;
    bool changed = false;
    sources[index].update([&](HealthState &state) { changed = state.available; state.available = false; });
    return changed;
}

HealthSnapshot Health::snapshot() const
{
    HealthSnapshot result;
    for (uint8_t index = 0u; index < static_cast<uint8_t>(SensorHealth::TOTAL_SIZE); ++index) result.sources[index] = sources[index].load();
    return result;
}
}
