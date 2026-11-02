#pragma once

#include <cstdint>

enum class PowerAction : uint8_t {
  None,
  BacklightOff,
  EnterLightSleep,
  Wake,
};

class PowerPolicy {
 public:
  PowerAction onCanActivity(uint32_t nowMs);
  PowerAction tick(uint32_t nowMs);

 private:
  uint32_t lastActivityMs_{0};
  bool initialized_{false};
  bool backlightOff_{false};
  bool sleepRequested_{false};
};
