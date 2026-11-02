#include "gear_estimator.hpp"

#include <utility>

namespace {

constexpr float kMinimumSpeedKph = 5.0f;
constexpr uint32_t kStabilityMs = 200;

}  // namespace

void GearEstimator::configure(std::array<RatioBand, 6> bands) {
  bands_ = std::move(bands);
  configured_ = true;
  resetCandidate();
}

std::optional<uint8_t> GearEstimator::update(float rpm, float kph,
                                              uint32_t nowMs) {
  if (!configured_ || kph < kMinimumSpeedKph) {
    resetCandidate();
    return std::nullopt;
  }

  const float rpmPerKph = rpm / kph;
  std::optional<uint8_t> match;
  for (const RatioBand& band : bands_) {
    if (rpmPerKph < band.minRpmPerKph || rpmPerKph > band.maxRpmPerKph) {
      continue;
    }
    if (match.has_value()) {
      resetCandidate();
      return std::nullopt;
    }
    match = band.gear;
  }

  if (!match.has_value()) {
    resetCandidate();
    return std::nullopt;
  }

  if (candidate_ != match) {
    candidate_ = match;
    candidateSinceMs_ = nowMs;
    return std::nullopt;
  }

  if (nowMs - candidateSinceMs_ < kStabilityMs) {
    return std::nullopt;
  }
  return candidate_;
}

void GearEstimator::resetCandidate() {
  candidate_.reset();
  candidateSinceMs_ = 0;
}
