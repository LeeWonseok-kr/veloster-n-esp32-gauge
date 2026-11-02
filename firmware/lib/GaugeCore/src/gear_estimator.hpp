#pragma once

#include <array>
#include <cstdint>
#include <optional>

struct RatioBand {
  uint8_t gear;
  float minRpmPerKph;
  float maxRpmPerKph;
};

class GearEstimator {
 public:
  void configure(std::array<RatioBand, 6> bands);
  std::optional<uint8_t> update(float rpm, float kph, uint32_t nowMs);

 private:
  void resetCandidate();

  std::array<RatioBand, 6> bands_{};
  std::optional<uint8_t> candidate_;
  uint32_t candidateSinceMs_ = 0;
  bool configured_ = false;
};
