#pragma once

#include <can_frame.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

class ObdScheduler {
 public:
  std::optional<CanFrame> next(uint32_t nowMs);
  bool cancel(const CanFrame& request);
  void reset();
  void onResponse(uint8_t pid, uint32_t nowMs);
  void setSupported(uint8_t basePid, uint32_t bitmap);
  void finishDiscovery();
  std::optional<uint8_t> inFlightPid() const;

 private:
  static constexpr size_t kPollCount = 10;

  bool isSupported(uint8_t pid) const;
  bool isPollEnabled(size_t index) const;
  void clearInFlight();
  void completeInFlight();
  CanFrame beginRequest(uint8_t pid, uint32_t nowMs);

  std::array<uint32_t, 4> supported_{};
  std::array<uint32_t, kPollCount> lastRequestMs_{};
  std::array<bool, kPollCount> requested_{};
  std::optional<uint8_t> inFlightPid_;
  std::optional<size_t> inFlightPollIndex_;
  bool inFlightPollWasRequested_ = false;
  uint32_t inFlightPollLastRequestMs_ = 0;
  uint32_t inFlightSinceMs_ = 0;
  size_t discoveryIndex_ = 0;
  bool discovering_ = true;
};
