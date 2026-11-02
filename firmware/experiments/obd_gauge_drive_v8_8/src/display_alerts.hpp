#pragma once

#include <cmath>
#include <cstdint>

// 0 = deliberately disabled. A nonzero setting is user policy, NOT an OEM limit.
#ifndef GAUGE_SHIFT_RPM
#define GAUGE_SHIFT_RPM 0
#endif
static_assert(GAUGE_SHIFT_RPM >= 0 && GAUGE_SHIFT_RPM <= 8000,
              "Shift threshold must be disabled (0) or within the displayed RPM range");

namespace gaugev4 {
namespace alerts {
enum class DataState : uint8_t { Waiting, Receiving, Stale, Off };
enum class ShiftState : uint8_t { Disabled, Waiting, Ready, Shift };

inline ShiftState shiftState(float rpm, bool fresh, unsigned threshold=GAUGE_SHIFT_RPM) {
  if (!threshold || threshold>8000) return ShiftState::Disabled;
  if (!fresh || !std::isfinite(rpm) || rpm<0 || rpm>8000) return ShiftState::Waiting;
  return rpm>=threshold ? ShiftState::Shift : ShiftState::Ready;
}

inline const char* dataLabel(DataState state) {
  switch(state) {
    case DataState::Receiving:return "DATA RX";
    case DataState::Stale:return "DATA STALE";
    case DataState::Off:return "DATA OFF";
    default:return "DATA WAIT";
  }
}
inline const char* shiftLabel(ShiftState state) {
  switch(state) {
    case ShiftState::Waiting:return "SHIFT WAIT";
    case ShiftState::Ready:return "SHIFT READY";
    case ShiftState::Shift:return "SHIFT";
    default:return "SHIFT OFF";
  }
}
} // namespace alerts
} // namespace gaugev4
