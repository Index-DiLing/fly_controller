#pragma once
#include <stdint.h>
#include "dlx_sensor_data.hpp"

namespace dlx
{
    struct ImuSample {
        AccelerometerG acceleration{{0.0f, 0.0f, 1.0f}};
        GyroscopeRads angularVelocity{{0.0f, 0.0f, 0.0f}};
        uint32_t tickMs{0u};
        uint32_t sequence{0u};
    };

    struct BaroSample {
        float pressurePa{0.0f};
        float temperatureC{0.0f};
        float altitudeM{0.0f};
        float relativeAltitudeM{0.0f};
        uint32_t referenceSequence{0u};
        uint32_t tickMs{0u};
    };

    struct RangeSample {
        float rangeM{0.0f};
        uint8_t status{0u};
        uint16_t rawMm{0u};
        uint16_t peakSignal{0u};
        uint16_t ambient{0u};
        uint32_t tickMs{0u};
    };

    struct FlowSample {
        int16_t dx{0};
        int16_t dy{0};
        uint8_t squal{0u};
        uint16_t dtMs{0u};
        uint32_t tickMs{0u};
    };

    struct GnssSample {
        int32_t latE7{0};
        int32_t lonE7{0};
        int32_t altMslMm{0};
        int16_t gspeedCms{0};
        uint16_t courseCdeg{0u};
        uint8_t sats{0u};
        uint8_t fix{0u};
        uint8_t hdopX10{0u};
        bool posValid{false};
        bool altValid{false};
        bool speedValid{false};
        bool courseValid{false};
        bool rmcSeen{false};
        bool rmcValid{false};
        float eastM{0.0f};
        float northM{0.0f};
        float upM{0.0f};
        float velocityEastMps{0.0f};
        float velocityNorthMps{0.0f};
        bool localPositionValid{false};
        bool velocityValid{false};
        uint32_t positionMs{0u};
        uint32_t velocityMs{0u};
        uint32_t velocitySequence{0u};
        uint32_t tickMs{0u};
        uint32_t solSeq{0u};
    };

    struct PowerSample {
        float packV{0.0f};
        float amps{0.0f};
        uint32_t tickMs{0u};
        bool valid{false};
    };
}
