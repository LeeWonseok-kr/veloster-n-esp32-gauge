#include <obd_scheduler.hpp>

#include <app_config.hpp>

#include <array>
#include <cstddef>

namespace {

struct PollItem {
  uint8_t pid;
  uint32_t cadenceMs;
};

constexpr std::array<PollItem, 10> kPolls = {{{0x0C, 50},
                                               {0x0B, 50},
                                               {0x49, 50},
                                               {0x11, 50},
                                               {0x0D, 100},
                                               {0x05, 500},
                                               {0x5C, 500},
                                               {0x0F, 500},
                                               {0x42, 500},
                                               {0x33, 500}}};
constexpr std::array<uint8_t, 4> kDiscoveryPids = {0x00, 0x20, 0x40,
                                                   0x60};
}  // namespace

std::optional<CanFrame> ObdScheduler::next(uint32_t nowMs) {
  if (inFlightPid_.has_value()) {
    if (nowMs - inFlightSinceMs_ < config::kObdResponseTimeoutMs) {
      return std::nullopt;
    }
    completeInFlight();
  }

  if (discovering_) {
    if (discoveryIndex_ >= kDiscoveryPids.size()) {
      return std::nullopt;
    }
    return beginRequest(kDiscoveryPids[discoveryIndex_], nowMs);
  }

  size_t selected = kPollCount;
  uint32_t greatestOverdueMs = 0;
  bool selectedNeverRequested = false;

  for (size_t index = 0; index < kPolls.size(); ++index) {
    if (!isPollEnabled(index)) {
      continue;
    }

    if (!requested_[index]) {
      if (!selectedNeverRequested) {
        selected = index;
        selectedNeverRequested = true;
      }
      continue;
    }
    if (selectedNeverRequested) {
      continue;
    }

    const uint32_t elapsedMs = nowMs - lastRequestMs_[index];
    if (elapsedMs < kPolls[index].cadenceMs) {
      continue;
    }
    const uint32_t overdueMs = elapsedMs - kPolls[index].cadenceMs;
    if (selected == kPollCount || overdueMs > greatestOverdueMs) {
      selected = index;
      greatestOverdueMs = overdueMs;
    }
  }

  if (selected == kPollCount) {
    return std::nullopt;
  }

  inFlightPollIndex_ = selected;
  inFlightPollWasRequested_ = requested_[selected];
  inFlightPollLastRequestMs_ = lastRequestMs_[selected];
  requested_[selected] = true;
  lastRequestMs_[selected] = nowMs;
  return beginRequest(kPolls[selected].pid, nowMs);
}

bool ObdScheduler::cancel(const CanFrame& request) {
  if (!inFlightPid_.has_value()) {
    return false;
  }

  const CanFrame currentRequest = {
      0x7DF, 8, {0x02, 0x01, *inFlightPid_, 0, 0, 0, 0, 0}};
  if (request.id != currentRequest.id || request.dlc != currentRequest.dlc ||
      request.data != currentRequest.data) {
    return false;
  }

  if (inFlightPollIndex_.has_value()) {
    const size_t index = *inFlightPollIndex_;
    requested_[index] = inFlightPollWasRequested_;
    lastRequestMs_[index] = inFlightPollLastRequestMs_;
  }
  clearInFlight();
  return true;
}

void ObdScheduler::reset() {
  supported_.fill(0);
  lastRequestMs_.fill(0);
  requested_.fill(false);
  inFlightPid_.reset();
  inFlightPollIndex_.reset();
  inFlightPollWasRequested_ = false;
  inFlightPollLastRequestMs_ = 0;
  inFlightSinceMs_ = 0;
  discoveryIndex_ = 0;
  discovering_ = true;
}

void ObdScheduler::onResponse(uint8_t pid, uint32_t) {
  if (!inFlightPid_.has_value() || *inFlightPid_ != pid) {
    return;
  }
  completeInFlight();
}

void ObdScheduler::setSupported(uint8_t basePid, uint32_t bitmap) {
  if (basePid > 0x60 || basePid % 0x20 != 0) {
    return;
  }
  supported_[basePid / 0x20] |= bitmap;
}

void ObdScheduler::finishDiscovery() {
  discovering_ = false;
  clearInFlight();
}

std::optional<uint8_t> ObdScheduler::inFlightPid() const {
  return inFlightPid_;
}

bool ObdScheduler::isSupported(uint8_t pid) const {
  if (pid == 0) {
    return false;
  }
  const uint8_t basePid = static_cast<uint8_t>((pid - 1u) & 0xE0u);
  if (basePid > 0x60) {
    return false;
  }
  const uint32_t offset = static_cast<uint32_t>(pid - basePid - 1u);
  return (supported_[basePid / 0x20u] & (uint32_t{1} << (31u - offset))) !=
         0;
}

bool ObdScheduler::isPollEnabled(size_t index) const {
  const uint8_t pid = kPolls[index].pid;
  if (!isSupported(pid)) {
    return false;
  }
  return pid != 0x11 || !isSupported(0x49);
}

void ObdScheduler::completeInFlight() {
  if (discovering_) {
    ++discoveryIndex_;
    if (discoveryIndex_ >= kDiscoveryPids.size()) {
      discovering_ = false;
    }
  }
  clearInFlight();
}

void ObdScheduler::clearInFlight() {
  inFlightPid_.reset();
  inFlightPollIndex_.reset();
}

CanFrame ObdScheduler::beginRequest(uint8_t pid, uint32_t nowMs) {
  inFlightPid_ = pid;
  inFlightSinceMs_ = nowMs;
  return {0x7DF, 8, {0x02, 0x01, pid, 0, 0, 0, 0, 0}};
}
