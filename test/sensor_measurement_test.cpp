#include <stdio.h>
#include <cmath>
#include <limits>
#include "app/sensors/app_measurement_reference.hpp"
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

static bool near(float actual, float expected)
{
    return std::fabs(actual - expected) < 0.001f;
}

static void testBaroReference()
{
    dlx::BaroReference reference;
    dlx::SampleSlot<dlx::BaroSample> samples;
    dlx::BaroSample sample;
    uint32_t seen = 0u;
    CHECK(reference.pending());
    CHECK(!reference.request());
    sample.altitudeM = std::numeric_limits<float>::quiet_NaN();
    CHECK(!reference.apply(sample));
    for (uint16_t index = 0u; index < dlx::BaroReference::SamplesPerReference; ++index) {
        sample.altitudeM = 100.0f + index;
        sample.tickMs    = index * 20u;
        CHECK(reference.apply(sample));
        CHECK(reference.pending());
        if (index + 1u < dlx::BaroReference::SamplesPerReference) CHECK(sample.referenceSequence == 0u);
        samples.publish(sample);
        reference.published();
    }
    CHECK(!reference.pending());
    CHECK(sample.referenceSequence == 1u);
    CHECK(near(sample.relativeAltitudeM, 7.5f));
    CHECK(samples.readNew(sample, seen));
    CHECK(sample.referenceSequence == 1u);
    CHECK(!samples.readNew(sample, seen));
    sample.altitudeM = 120.0f;
    CHECK(reference.apply(sample));
    reference.published();
    CHECK(sample.referenceSequence == 1u);
    CHECK(near(sample.relativeAltitudeM, 12.5f));
    CHECK(reference.request());
    for (uint16_t index = 0u; index < dlx::BaroReference::SamplesPerReference; ++index) {
        CHECK(!reference.request());
        sample.altitudeM = 200.0f + index;
        CHECK(reference.apply(sample));
        CHECK(reference.pending());
        if (index + 1u < dlx::BaroReference::SamplesPerReference) CHECK(sample.referenceSequence == 1u);
        samples.publish(sample);
        reference.published();
    }
    CHECK(!reference.pending());
    CHECK(samples.readNew(sample, seen));
    CHECK(sample.referenceSequence == 2u);
    CHECK(near(sample.relativeAltitudeM, 7.5f));
    CHECK(reference.request());
    sample.altitudeM = std::numeric_limits<float>::infinity();
    CHECK(!reference.apply(sample));
    CHECK(reference.pending());
    CHECK(samples.read(sample));
    CHECK(sample.referenceSequence == 2u);
}

static dlx::GnssSolution solutionAt(uint32_t stamp)
{
    dlx::GnssSolution solution;
    solution.clear();
    solution.latE7       = 480000000;
    solution.lonE7       = 110000000;
    solution.altMslMm    = 100000;
    solution.gspeedCms   = 200;
    solution.courseCdeg  = 9000u;
    solution.sats        = 8u;
    solution.fix         = 1u;
    solution.hdopX10     = 9u;
    solution.speedValid  = true;
    solution.courseValid = true;
    solution.latestMs    = stamp;
    solution.seq         = 1u;
    for (uint8_t index = 0u; index < dlx::kNfCount; ++index) {
        solution.fieldMs[index]  = stamp;
        solution.fieldSrc[index] = static_cast<uint8_t>(dlx::NmeaGroup::Rmc);
    }
    return solution;
}

static void testGnssReference()
{
    dlx::GnssReference reference;
    dlx::GnssSolution solution = solutionAt(100u);
    dlx::GnssSample sample     = reference.convert(solution, 6u, 20u);
    CHECK(!sample.localPositionValid);
    CHECK(sample.velocityValid);
    CHECK(near(sample.velocityEastMps, 2.0f));
    CHECK(near(sample.velocityNorthMps, 0.0f));
    CHECK(sample.velocitySequence == 1u);
    CHECK(sample.velocityMs == 100u);
    solution.seq                  = 2u;
    solution.latestMs             = 200u;
    solution.fieldMs[dlx::kNfFix] = 200u;
    sample                        = reference.convert(solution, 6u, 20u);
    CHECK(sample.solSeq == 2u);
    CHECK(sample.velocitySequence == 1u);
    CHECK(sample.velocityMs == 100u);
    solution.courseValid = false;
    sample               = reference.convert(solution, 6u, 20u);
    CHECK(!sample.velocityValid);
    CHECK(sample.velocitySequence == 1u);
    solution.courseValid             = true;
    solution.fieldMs[dlx::kNfSpeed]  = 300u;
    solution.fieldMs[dlx::kNfCourse] = 290u;
    sample                           = reference.convert(solution, 6u, 20u);
    CHECK(sample.velocitySequence == 2u);
    CHECK(sample.velocityMs == 290u);
    solution.posValid = true;
    solution.altValid = true;
    sample            = reference.convert(solution, 6u, 20u);
    CHECK(sample.localPositionValid);
    CHECK(near(sample.eastM, 0.0f));
    CHECK(near(sample.northM, 0.0f));
    solution.latE7 += 10000;
    sample = reference.convert(solution, 6u, 20u);
    CHECK(sample.localPositionValid);
    CHECK(sample.northM > 100.0f);
    solution.posValid = false;
    sample            = reference.convert(solution, 6u, 20u);
    CHECK(!sample.localPositionValid);
    CHECK(sample.velocityValid);
    solution.fieldMs[dlx::kNfSpeed]  = 0xfffffff0u;
    solution.fieldMs[dlx::kNfCourse] = 10u;
    sample                           = reference.convert(solution, 6u, 20u);
    CHECK(sample.velocityMs == 0xfffffff0u);
}

static void feed(dlx::GnssNmeaParser &parser, const char *body, uint32_t stamp)
{
    uint8_t checksum = 0u;
    for (const char *character = body; *character != '\0'; ++character) checksum ^= static_cast<uint8_t>(*character);
    char sentence[160];
    snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", body, static_cast<unsigned>(checksum));
    for (const char *character = sentence; *character != '\0'; ++character) parser.feed(static_cast<uint8_t>(*character), stamp);
}

static void testCourseFreshness()
{
    dlx::GnssNmeaParser parser(dlx::kNmeaMaskRmc | dlx::kNmeaMaskGga | dlx::kNmeaMaskVtg);
    const char *gga = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
    feed(parser, gga, 100u);
    feed(parser, "GPRMC,123519,A,4807.038,N,01131.000,E,1.0,90.0,230394,,,A", 100u);
    CHECK(parser.solution().speedValid);
    CHECK(parser.solution().courseValid);
    const uint32_t later = 101u + dlx::kGnssStaleMs;
    feed(parser, gga, later);
    feed(parser, "GPVTG,,T,,M,1.0,N,1.9,K,A", later);
    CHECK(parser.solution().speedValid);
    CHECK(!parser.solution().courseValid);
}

int main()
{
    testBaroReference();
    testGnssReference();
    testCourseFreshness();
    printf("sensor_measurement_test: %u checks, %u failures\n", checks, failures);
    return failures == 0u ? 0 : 1;
}
