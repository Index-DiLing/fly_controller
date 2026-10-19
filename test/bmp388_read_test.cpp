#include <stdio.h>
#include "BMP388/dlx_bmp388.hpp"
#include "app/sensors/app_health.hpp"
#include "app/sensors/app_sensor_samples.hpp"
#include "dlx_sample_channel.hpp"

static unsigned checks   = 0u;
static unsigned failures = 0u;
#define CHECK(condition)                                        \
    do {                                                        \
        ++checks;                                               \
        if (!(condition)) {                                     \
            ++failures;                                         \
            printf("FAIL line %d: %s\n", __LINE__, #condition); \
        }                                                       \
    } while (false)

static void prepare(dlx::IICBus &bus)
{
    bus.registers[0x00] = dlx::BMP388_CHIP_ID_VALUE;
    bus.registers[0x31] = 1u;
    bus.registers[0x33] = 1u;
    bus.registers[0x37] = 0x40u;
    bus.registers[0x39] = 0x40u;
    bus.registers[0x3c] = 0xd4u;
    bus.registers[0x3d] = 0x30u;
}

int main()
{
    dlx::IICBus bus;
    prepare(bus);
    dlx::BMP388 barometer(bus);
    dlx::ExceptionCode error = dlx::ExceptionCode::GNSS_NO_FIX;
    float pressure           = 12.0f;
    float temperature        = 34.0f;
    CHECK(barometer.read(pressure, temperature, error) == dlx::BMP388::ReadResult::Failed);
    CHECK(error == dlx::ExceptionCode::BARO_CALIB_INVALID);
    CHECK(pressure == 12.0f && temperature == 34.0f);
    CHECK(barometer.tryInit(error) == dlx::FunctionResult::SUCCESS);
    error = dlx::ExceptionCode::GNSS_NO_FIX;
    for (unsigned index = 0u; index < 32u; ++index) {
        CHECK(barometer.read(pressure, temperature, error) == dlx::BMP388::ReadResult::NotReady);
        CHECK(pressure == 12.0f && temperature == 34.0f);
        CHECK(error == dlx::ExceptionCode::GNSS_NO_FIX);
    }
    bus.registers[0x03] = dlx::BMP388_STATUS_DRDY_PRESS;
    CHECK(barometer.read(pressure, temperature, error) == dlx::BMP388::ReadResult::NewData);
    CHECK(pressure == 100000.0f);
    CHECK(error == dlx::ExceptionCode::GNSS_NO_FIX);
    dlx::Health health;
    dlx::SampleSlot<dlx::BaroSample> samples;
    dlx::BaroSample sample;
    sample.pressurePa   = pressure;
    sample.temperatureC = temperature;
    sample.tickMs       = 100u;
    samples.publish(sample);
    health.success(dlx::SensorHealth::BARO, sample.tickMs);
    uint32_t seen = 0u;
    CHECK(samples.readNew(sample, seen));
    bus.registers[0x03] = 0u;
    CHECK(barometer.read(pressure, temperature, error) == dlx::BMP388::ReadResult::NotReady);
    CHECK(!samples.readNew(sample, seen));
    CHECK(sample.tickMs == 100u);
    CHECK(health.snapshot().ok(dlx::SensorHealth::BARO, 100u));
    CHECK(!health.snapshot().ok(dlx::SensorHealth::BARO, 101u + dlx::healthMaxAgeMs(dlx::SensorHealth::BARO)));
    uint32_t rawPressure    = 123u;
    uint32_t rawTemperature = 456u;
    CHECK(barometer.readRaw(rawPressure, rawTemperature, error) == dlx::BMP388::ReadResult::NotReady);
    CHECK(rawPressure == 123u && rawTemperature == 456u);
    bus.failRead = true;
    CHECK(barometer.read(pressure, temperature, error) == dlx::BMP388::ReadResult::Failed);
    CHECK(error == dlx::ExceptionCode::BARO_READ_FAILED);
    CHECK(pressure == 100000.0f);
    CHECK(barometer.failCount() == 1u);
    bus.failRead        = false;
    bus.registers[0x03] = dlx::BMP388_STATUS_DRDY_PRESS;
    CHECK(barometer.read(pressure, temperature, error) == dlx::BMP388::ReadResult::NewData);
    CHECK(barometer.failCount() == 0u);
    printf("bmp388_read_test: %u checks, %u failures\n", checks, failures);
    return failures == 0u ? 0 : 1;
}
