#pragma once

#include <array>
#include <cstdint>

#include <app_config.hpp>

#include "signal_store.hpp"

enum class StatusCellState : uint8_t { Unknown, Inactive, Active };

namespace layout {
constexpr uint16_t kScreenWidth = 480;

constexpr uint16_t boostPixels(float boostBar) {
  const float clamped = boostBar < config::kBoostMinBar
                            ? config::kBoostMinBar
                            : (boostBar > config::kBoostMaxBar
                                   ? config::kBoostMaxBar
                                   : boostBar);
  return static_cast<uint16_t>(
      ((clamped - config::kBoostMinBar) * kScreenWidth /
       (config::kBoostMaxBar - config::kBoostMinBar)) +
      0.5f);
}

constexpr uint16_t kBoostZeroPixels = boostPixels(0.0f);
constexpr uint16_t kBoostRedStartPixels =
    boostPixels(config::kBoostRedStartBar);
}  // namespace layout

struct Rect {
  uint16_t x;
  uint16_t y;
  uint16_t width;
  uint16_t height;

  constexpr uint16_t right() const { return static_cast<uint16_t>(x + width); }
  constexpr uint16_t bottom() const { return static_cast<uint16_t>(y + height); }
};

struct DashboardFrame {
  std::array<char, 8> rpmText{};
  std::array<char, 8> boostText{};
  std::array<char, 8> speedText{};
  std::array<char, 8> voltageText{};
  std::array<char, 8> coolantText{};
  std::array<char, 8> oilText{};
  std::array<char, 8> intakeText{};
  std::array<char, 8> gearText{};
  std::array<char, 8> modeText{};
  std::array<char, 8> clutchText{};
  std::array<char, 8> brakeText{};
  std::array<char, 8> accelText{};
  std::array<char, 8> tpmsFlPsi{};
  std::array<char, 8> tpmsFrPsi{};
  std::array<char, 8> tpmsRlPsi{};
  std::array<char, 8> tpmsRrPsi{};
  std::array<char, 8> tpmsFlTemp{};
  std::array<char, 8> tpmsFrTemp{};
  std::array<char, 8> tpmsRlTemp{};
  std::array<char, 8> tpmsRrTemp{};
  std::array<StatusCellState, 7> nStatus{};
  uint16_t rpmBarPixels{0};
  uint16_t boostZeroPixels{layout::kBoostZeroPixels};
  uint16_t boostEndpointPixels{layout::kBoostZeroPixels};
  uint16_t boostRedStartPixels{layout::kBoostRedStartPixels};
  bool boostValid{false};
};

namespace layout {
constexpr uint16_t kScreenHeight = 320;
constexpr uint16_t kRpmScaleMaximum = 8000;
constexpr uint16_t kRpmYellowStartPixels = static_cast<uint16_t>(
    static_cast<uint32_t>(config::kRpmYellowStart) * kScreenWidth /
    kRpmScaleMaximum);
constexpr uint16_t kRpmRedStartPixels = static_cast<uint16_t>(
    static_cast<uint32_t>(config::kRpmRedStart) * kScreenWidth /
    kRpmScaleMaximum);

constexpr Rect kRpm{0, 0, 480, 56};
constexpr Rect kBoost{0, 56, 480, 32};
constexpr Rect kTpms{0, 88, 149, 162};
constexpr Rect kGear{149, 88, 139, 162};
constexpr Rect kTemps{288, 88, 106, 162};
constexpr Rect kClutch{394, 88, 29, 162};
constexpr Rect kBrake{423, 88, 29, 162};
constexpr Rect kAccel{452, 88, 28, 162};
constexpr Rect kMode{0, 250, 115, 42};
constexpr Rect kVoltage{115, 250, 96, 42};
constexpr Rect kSpeed{211, 250, 130, 42};
constexpr Rect kRpmNumber{341, 250, 139, 42};
constexpr Rect kRevStatus{0, 292, 69, 28};
constexpr Rect kEngStatus{69, 292, 69, 28};
constexpr Rect kLsdStatus{138, 292, 68, 28};
constexpr Rect kExstStatus{206, 292, 69, 28};
constexpr Rect kEcsStatus{275, 292, 68, 28};
constexpr Rect kMdpsStatus{343, 292, 69, 28};
constexpr Rect kVdcStatus{412, 292, 68, 28};
}  // namespace layout

class DashboardModel {
 public:
  static DashboardFrame build(const SignalStore& store, uint32_t nowMs);
};
