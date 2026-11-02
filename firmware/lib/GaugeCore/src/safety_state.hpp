#pragma once

#include <cstdint>

enum class CanPhase : uint8_t {
  Boot,
  ListenOnly,
  Active,
  Offline,
  BusOffLatched,
};

struct CanDecision {
  CanPhase phase;
  bool mayTransmit;
};

class CanSafetyState {
 public:
  void start(uint32_t nowMs);
  void onValidFrame(uint32_t nowMs);
  void onRxError();
  void onBusOff();
  void onPortUnavailable();
  CanDecision tick(uint32_t nowMs);
  CanPhase phase() const;

 private:
  void transitionOffline();

  CanPhase phase_{CanPhase::Boot};
  uint32_t startedAtMs_{0};
  bool sawValidTraffic_{false};
};
