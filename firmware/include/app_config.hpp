#pragma once

#include <cstdint>

namespace config {
constexpr uint32_t kCanBitrate = 500000;
constexpr uint32_t kListenOnlyMs = 3000;
constexpr uint32_t kCanRestartRetryMs = 1000;
constexpr uint32_t kObdResponseTimeoutMs = 120;
constexpr uint32_t kFastStaleMs = 500;
constexpr uint32_t kSlowStaleMs = 5000;
constexpr uint32_t kTpmsStaleMs = 60000;
constexpr uint32_t kSleepAfterCanIdleMs = 60000;
constexpr uint32_t kUiPeriodMs = 50;
constexpr uint16_t kRpmYellowStart = 5900;
constexpr uint16_t kRpmRedStart = 6900;
constexpr float kBoostMinBar = -1.0f;
constexpr float kBoostMaxBar = 2.0f;
constexpr float kBoostRedStartBar = 1.6f;
// A lone CAN error can be normal electrical noise. These conservative limits
// stop use after repeated loss, well before the controller's 128-count passive
// warning region; any FIFO overrun means received data is already incomplete.
constexpr uint32_t kTwaiBusErrorCountLimit = 8;
constexpr uint32_t kTwaiRxErrorCounterLimit = 96;
constexpr uint32_t kTwaiRxMissedCountLimit = 8;
}
