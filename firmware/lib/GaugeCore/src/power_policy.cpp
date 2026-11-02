#include "power_policy.hpp"

#include <app_config.hpp>

namespace {
constexpr uint32_t kLightSleepAfterCanIdleMs =
    config::kSleepAfterCanIdleMs + 500U;
}

PowerAction PowerPolicy::onCanActivity(uint32_t nowMs) {
  const bool shouldWake = backlightOff_ || sleepRequested_;
  lastActivityMs_ = nowMs;
  initialized_ = true;
  backlightOff_ = false;
  sleepRequested_ = false;
  return shouldWake ? PowerAction::Wake : PowerAction::None;
}

PowerAction PowerPolicy::tick(uint32_t nowMs) {
  if (!initialized_) {
    return PowerAction::None;
  }

  const uint32_t idleMs = nowMs - lastActivityMs_;
  if (!backlightOff_ && idleMs >= config::kSleepAfterCanIdleMs) {
    backlightOff_ = true;
    return PowerAction::BacklightOff;
  }
  if (!sleepRequested_ && idleMs >= kLightSleepAfterCanIdleMs) {
    sleepRequested_ = true;
    return PowerAction::EnterLightSleep;
  }
  return PowerAction::None;
}
