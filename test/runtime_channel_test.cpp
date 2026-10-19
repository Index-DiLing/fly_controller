#include <stdio.h>
#include "app/channels/app_control_channel.hpp"
#include "app/channels/app_fault_channel.hpp"
#include "dlx_sample_channel.hpp"

static unsigned checks = 0u;
static unsigned failures = 0u;

#define CHECK(condition) do { ++checks; if (!(condition)) { ++failures; printf("FAIL line %d: %s\n", __LINE__, #condition); } } while (false)

static void testRequests()
{
    app::ControlChannels channels;
    app::Request request;
    CHECK(!channels.post(request));
    request.task = app::Task::RequestEStop;
    CHECK(!channels.post(request));
    request.task = app::Task::RequestArm;
    request.source = app::RequestSource::Link;
    request.safetyEpoch = 7u;
    for (uint32_t index = 0u; index < app::RequestQueueCapacity; ++index) {
        request.id = index + 1u;
        CHECK(channels.post(request));
    }
    CHECK(!channels.post(request));
    CHECK(channels.requests.dropped() == 1u);
    for (uint32_t index = 0u; index < app::RequestQueueCapacity; ++index) {
        app::Request received;
        CHECK(channels.requests.tryPop(received));
        CHECK(received.id == index + 1u);
        CHECK(received.task == app::Task::RequestArm);
        CHECK(received.source == app::RequestSource::Link);
        CHECK(received.safetyEpoch == 7u);
    }
    CHECK(!channels.requests.tryPop(request));
    app::RequestResult result;
    result.kind = app::ResultKind::EmergencyStop;
    result.id = 42u;
    for (uint16_t index = 0u; index < app::ResultQueueCapacity; ++index) CHECK(channels.results.tryPush(result));
    CHECK(!channels.results.tryPush(result));
    CHECK(channels.results.tryPop(result));
    CHECK(result.kind == app::ResultKind::EmergencyStop && result.id == 42u);
    CHECK(channels.results.tryPush(result));
    channels.safetyEpoch.store(7u);
    channels.safetyEpoch.update([](uint32_t &epoch) { ++epoch; });
    CHECK(channels.safetyEpoch.load() != request.safetyEpoch);
}

static void testFaultOverflow()
{
    app::FaultChannel faults;
    dlx::ExceptionCode code;
    faults.report(dlx::ExceptionCode::STATE_MODE_DEGRADED);
    CHECK(!faults.take(code));
    CHECK(faults.takeActions() == 0u);
    for (uint16_t index = 0u; index < app::FaultQueueCapacity; ++index) faults.report(dlx::ExceptionCode::IMU_DATA_STALE);
    faults.report(dlx::ExceptionCode::POWER_VOLTAGE_CUT);
    CHECK(faults.dropped() == 1u);
    const uint32_t actions = faults.takeActions();
    CHECK((actions & app::RefreshHealth) != 0u);
    CHECK((actions & app::StopOutput) != 0u);
    CHECK(faults.takeActions() == 0u);
    for (uint16_t index = 0u; index < app::FaultQueueCapacity; ++index) {
        CHECK(faults.take(code));
        CHECK(code == dlx::ExceptionCode::IMU_DATA_STALE);
    }
    CHECK(!faults.take(code));
}

static void testSamples()
{
    dlx::SampleSlot<uint32_t> latest;
    uint32_t sample = 0u;
    uint32_t seen = 0u;
    CHECK(!latest.read(sample));
    CHECK(!latest.readNew(sample, seen));
    latest.publish(10u);
    latest.publish(20u);
    CHECK(latest.readNew(sample, seen));
    CHECK(sample == 20u);
    CHECK(!latest.readNew(sample, seen));
    CHECK(latest.read(sample));
    CHECK(sample == 20u);
    latest.publish(20u);
    CHECK(latest.readNew(sample, seen));
    dlx::SampleStream<uint32_t, 2u> stream;
    CHECK(stream.publish(1u));
    CHECK(stream.publish(2u));
    CHECK(!stream.publish(3u));
    CHECK(stream.dropped() == 1u);
    CHECK(stream.size() == 2u);
    CHECK(stream.read(sample) && sample == 1u);
    CHECK(stream.publish(4u));
    CHECK(stream.read(sample) && sample == 2u);
    CHECK(stream.read(sample) && sample == 4u);
    CHECK(!stream.read(sample));
}

static void testControlFreshness()
{
    app::ControlInput input;
    CHECK(!app::controlInputFresh(input, app::Mode::Stabilize, 100u));
    input.valid = true;
    input.stickValid = true;
    input.stickReceivedMs = 100u;
    CHECK(app::controlInputFresh(input, app::Mode::Stabilize, 100u));
    const uint32_t expired = 101u + app::ControlFreshnessMs;
    input.receivedMs = expired;
    CHECK(!app::controlInputFresh(input, app::Mode::Stabilize, expired));
    CHECK(!app::controlInputFresh(input, app::Mode::Manual, 100u));
    input.motorSequence = 1u;
    input.motorReceivedMs = 100u;
    CHECK(app::controlInputFresh(input, app::Mode::Manual, 100u));
    CHECK(!app::controlInputFresh(input, app::Mode::Manual, expired));
    input.velocityValid = true;
    input.velocityReceivedMs = expired;
    CHECK(app::controlInputFresh(input, app::Mode::VelHold, expired));
    CHECK(!app::controlInputFresh(input, app::Mode::AltHold, expired));
    CHECK(!app::controlInputFresh(input, app::Mode::VelHold, expired + app::ControlFreshnessMs + 1u));
}

static void testStickyStop()
{
    app::ControlChannels channels;
    app::EmergencyStopHandshake handshake;
    app::ControlInput input;
    input.emergencyStopId = handshake.request();
    input.emergencyStopPending = handshake.pending;
    channels.input.store(input);
    input.sequence = 8u;
    channels.input.store(input);
    const app::ControlInput received = channels.input.load();
    CHECK(received.emergencyStopPending);
    CHECK(received.emergencyStopId == handshake.requestedId);
    const uint32_t first = received.emergencyStopId;
    const uint32_t second = handshake.request();
    CHECK(!handshake.acknowledge(first));
    CHECK(handshake.pending);
    CHECK(handshake.acknowledge(second));
    CHECK(!handshake.pending);
    CHECK(!handshake.acknowledge(second));
    handshake.requestedId = UINT32_MAX;
    CHECK(handshake.request() == 1u);
}

int main()
{
    testRequests();
    testFaultOverflow();
    testSamples();
    testControlFreshness();
    testStickyStop();
    printf("Runtime channels: %u checks, %u failures\n", checks, failures);
    return failures == 0u ? 0 : 1;
}
