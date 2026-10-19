#pragma once
#include <atomic>
#include <cmath>
#include "app/sensors/app_sensor_samples.hpp"
#include "GNSS/dlx_geo.hpp"
#include "GNSS/dlx_gnss_nmea.hpp"

namespace dlx
{
    static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "Barometer request flag must be lock-free");

    class BaroReference
    {
        std::atomic<bool> requested{true};
        float referenceM{0.0f};
        float sumM{0.0f};
        uint16_t count{0u};
        uint32_t sequence{0u};
        bool referenceChanged{false};

    public:
        static constexpr uint16_t SamplesPerReference = 16u;

        bool request()
        {
            bool expected = false;
            return requested.compare_exchange_strong(expected, true, std::memory_order_relaxed);
        }
        bool pending() const
        {
            return requested.load(std::memory_order_acquire);
        }

        bool apply(BaroSample &sample)
        {
            if (!std::isfinite(sample.altitudeM)) return false;
            if (pending()) {
                sumM += sample.altitudeM;
                if (++count >= SamplesPerReference) {
                    referenceM = sumM / static_cast<float>(count);
                    if (++sequence == 0u) ++sequence;
                    sumM             = 0.0f;
                    count            = 0u;
                    referenceChanged = true;
                }
            }
            sample.relativeAltitudeM = sequence == 0u ? 0.0f : sample.altitudeM - referenceM;
            sample.referenceSequence = sequence;
            return true;
        }

        void published()
        {
            if (!referenceChanged) return;
            referenceChanged = false;
            requested.store(false, std::memory_order_release);
        }
    };

    class GnssReference
    {
        GeoOrigin origin{};
        uint32_t speedMs{0u};
        uint32_t courseMs{0u};
        uint32_t velocitySequence{0u};

        static uint32_t older(uint32_t first, uint32_t second)
        {
            return static_cast<int32_t>(first - second) <= 0 ? first : second;
        }

    public:
        GnssSample convert(const GnssSolution &solution, uint8_t minSats, uint16_t maxHdopX10)
        {
            GnssSample sample;
            sample.latE7       = solution.latE7;
            sample.lonE7       = solution.lonE7;
            sample.altMslMm    = solution.altMslMm;
            sample.gspeedCms   = solution.gspeedCms;
            sample.courseCdeg  = solution.courseCdeg;
            sample.sats        = solution.sats;
            sample.fix         = solution.fix;
            sample.hdopX10     = solution.hdopX10;
            sample.posValid    = solution.posValid;
            sample.altValid    = solution.altValid;
            sample.speedValid  = solution.speedValid;
            sample.courseValid = solution.courseValid;
            sample.rmcSeen     = solution.rmcSeen;
            sample.rmcValid    = solution.rmcValid;
            sample.tickMs      = solution.latestMs;
            sample.solSeq      = solution.seq;
            sample.positionMs  = older(solution.fieldMs[kNfLat], solution.fieldMs[kNfLon]);
            sample.velocityMs  = older(solution.fieldMs[kNfSpeed], solution.fieldMs[kNfCourse]);
            if (!origin.set) setOriginFromFix(solution, origin, minSats, maxHdopX10);
            if (sample.posValid && sample.altValid && origin.set) {
                sample.localPositionValid = geoToEnu(static_cast<double>(sample.latE7) * 1e-7,
                                                     static_cast<double>(sample.lonE7) * 1e-7, static_cast<double>(sample.altMslMm) * 0.001,
                                                     origin, sample.eastM, sample.northM, sample.upM);
            }
            sample.velocityValid = sample.speedValid && sample.courseValid;
            if (sample.velocityValid) {
                if (velocitySequence == 0u || speedMs != solution.fieldMs[kNfSpeed] || courseMs != solution.fieldMs[kNfCourse]) {
                    speedMs  = solution.fieldMs[kNfSpeed];
                    courseMs = solution.fieldMs[kNfCourse];
                    if (++velocitySequence == 0u) ++velocitySequence;
                }
                trackSpeedToEnu(static_cast<float>(sample.courseCdeg) * 0.01f,
                                static_cast<float>(sample.gspeedCms) * 0.01f, sample.velocityEastMps, sample.velocityNorthMps);
            }
            sample.velocitySequence = velocitySequence;
            return sample;
        }
    };
}
