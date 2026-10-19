#pragma once
#include <stdint.h>

namespace app
{
constexpr uint16_t RequestQueueCapacity = 16u;
constexpr uint16_t ResultQueueCapacity = 16u;
constexpr uint16_t FaultQueueCapacity = 16u;
constexpr uint8_t RequestsPerCycle = 8u;
constexpr uint8_t FaultsPerCycle = 8u;
constexpr uint16_t ImuQueueCapacity = 32u;
constexpr uint16_t FlowQueueCapacity = 8u;
constexpr uint8_t ImuSamplesPerCycle = 16u;
constexpr uint8_t FlowSamplesPerCycle = 4u;
constexpr uint32_t ControlPeriodMs = 2u;
constexpr uint32_t MotorArmDelayMs = 3200u;
constexpr float ArmThrottleLowFraction = 0.05f;
constexpr float ArmStickCenterFraction = 0.05f;
constexpr uint8_t FlowMinimumQuality = 25u;
constexpr uint16_t ImuThreadStackBytes = 2048u;
constexpr uint16_t SensorThreadStackBytes = 1024u;
constexpr uint8_t ImuThreadPriority = 21u;
constexpr uint8_t SensorThreadPriority = 22u;
constexpr uint8_t LinkAckCapacity = 4u;
constexpr uint32_t ImuPollMs = 2u;
constexpr uint32_t ImuFreshnessMs = 20u;
constexpr uint32_t BaroFreshnessMs = 200u;
constexpr uint32_t RangeFreshnessMs = 200u;
constexpr uint32_t FlowFreshnessMs = 100u;
constexpr uint32_t GnssFreshnessMs = 2000u;
constexpr uint32_t PowerFreshnessMs = 200u;
constexpr uint32_t ControlFreshnessMs = 2000u;
constexpr uint32_t RequestFreshnessMs = 2000u;
constexpr uint32_t SerialRxBytesPerRun = 256u;
}
