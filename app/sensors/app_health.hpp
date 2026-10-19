#pragma once
#include <stdint.h>
#include "dlx_locked_cell.hpp"
#include "app/runtime/app_runtime_limits.hpp"

namespace dlx
{
enum class SensorHealth : uint8_t
{
    LINK = 0, IMU, BARO, FLOW, RANGE, GNSS, POWER, TOTAL_SIZE
};

struct HealthState
{
    uint32_t lastSuccessMs{0u};
    bool hasSample{false};
    bool available{false};
};

struct HealthSnapshot
{
    HealthState sources[static_cast<uint8_t>(SensorHealth::TOTAL_SIZE)]{};
    bool ok(SensorHealth source, uint32_t nowMs) const;
    uint8_t bits(uint32_t nowMs) const;
};

class Health
{
    LockedCell<HealthState> sources[static_cast<uint8_t>(SensorHealth::TOTAL_SIZE)]{};
public:
    bool success(SensorHealth source, uint32_t sampleMs);
    bool failure(SensorHealth source);
    HealthSnapshot snapshot() const;
};

bool sampleFresh(uint32_t nowMs, uint32_t sampleMs, uint32_t maxAgeMs);
uint32_t healthMaxAgeMs(SensorHealth source);

}
