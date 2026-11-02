#include "safety_state.hpp"

#include <app_config.hpp>

void CanSafetyState::start(uint32_t nowMs) {
  phase_ = CanPhase::ListenOnly;
  startedAtMs_ = nowMs;
  sawValidTraffic_ = false;
}

void CanSafetyState::onValidFrame(uint32_t nowMs) {
  if (phase_ == CanPhase::ListenOnly &&
      nowMs - startedAtMs_ <= config::kListenOnlyMs) {
    sawValidTraffic_ = true;
  }
}

void CanSafetyState::onRxError() { transitionOffline(); }

void CanSafetyState::onBusOff() { phase_ = CanPhase::BusOffLatched; }

void CanSafetyState::onPortUnavailable() { transitionOffline(); }

void CanSafetyState::transitionOffline() {
  if (phase_ != CanPhase::BusOffLatched) {
    phase_ = CanPhase::Offline;
  }
}

CanDecision CanSafetyState::tick(uint32_t nowMs) {
  if (phase_ == CanPhase::ListenOnly &&
      nowMs - startedAtMs_ >= config::kListenOnlyMs) {
    phase_ = sawValidTraffic_ ? CanPhase::Active : CanPhase::Offline;
  }

  return {phase_, phase_ == CanPhase::Active};
}

CanPhase CanSafetyState::phase() const { return phase_; }
