#pragma once

#include "types.hpp"

#include <cstring>

namespace gaugev4 {
enum class IsoTpState : uint8_t { Idle, Pending, Complete, Fault };

struct IsoTpRecord {
  Frame frame{};
  uint32_t at = 0;
};

class IsoTpReceiver {
 public:
  explicit IsoTpReceiver(Port& port, uint32_t (*clock)() = nullptr)
      : port_(port), clock_(clock) {}

  bool setTrialWindow(uint32_t start, uint32_t limit) {
    if (state_ != IsoTpState::Idle || trialLimit_ || limit != 60000) return false;
    trialStart_ = start;
    trialLimit_ = limit;
    return true;
  }

  bool start(Profile profile, uint32_t transactionId, uint32_t requestAt) {
    if ((state_ != IsoTpState::Idle && state_ != IsoTpState::Complete) ||
        !diagnosticProfile(profile) || !transactionId) return false;
    state_ = IsoTpState::Pending;
    reason_ = "PENDING";
    profile_ = profile;
    transactionId_ = transactionId;
    requestAt_ = requestAt;
    progressAt_ = requestAt;
    completionAt_ = 0;
    length_ = 0;
    received_ = 0;
    recordCount_ = 0;
    rawRx_ = 0;
    flowControls_ = 0;
    sequence_ = 1;
    assembling_ = false;
    transactionEnded_ = false;
    std::memset(payload_, 0, sizeof(payload_));
    return true;
  }

  void tick(uint32_t now) {
    if (state_ != IsoTpState::Pending) return;
    if (!healthyAndTimely(now)) return;
    Frame frame{};
    bool empty = false;
    for (unsigned drained = 0; drained < 128; ++drained) {
      if (!healthyAndTimely(now)) return;
      const bool receivedFrame = port_.receive(frame);
      if (!healthyAndTimely(now)) return;
      if (!receivedFrame) { empty = true; break; }
      ++rawRx_;
      accept(frame, current(now));
      if (state_ != IsoTpState::Pending) return;
    }
    if (!empty) fail("RX_BACKLOG");
  }

  IsoTpState state() const { return state_; }
  const char* reason() const { return reason_; }
  Profile profile() const { return profile_; }
  unsigned length() const { return length_; }
  unsigned received() const { return received_; }
  const uint8_t* payload() const { return payload_; }
  uint32_t completionAt() const { return completionAt_; }
  uint32_t flowControls() const { return flowControls_; }
  uint32_t rawRx() const { return rawRx_; }
  unsigned recordCount() const { return recordCount_; }
  const IsoTpRecord* record(unsigned index) const {
    return index < recordCount_ ? &records_[index] : nullptr;
  }

 private:
  uint32_t current(uint32_t fallback) const { return clock_ ? clock_() : fallback; }

  bool timely(uint32_t fallback) {
    const uint32_t now = current(fallback);
    if (!kContinuous && trialLimit_ && now - trialStart_ >= trialLimit_) {
      fail("TRIAL_LIMIT"); return false;
    }
    if (now - requestAt_ >= 3000) { fail("ISO_TOTAL_TIMEOUT"); return false; }
    if (!assembling_ && now - requestAt_ >= 500) { fail("NO_RESPONSE"); return false; }
    if (assembling_ && now - progressAt_ >= 500) { fail("CF_TIMEOUT"); return false; }
    return true;
  }

  bool healthyAndTimely(uint32_t fallback) {
    if (!timely(fallback)) return false;
    if (!port_.good()) { fail("CAN_ERROR"); return false; }
    return timely(fallback);
  }

  uint32_t responseId() const { return profileResponseId(profile_); }

  bool matchingHeader(const uint8_t* bytes) const {
    return profilePrefixMatches(profile_, bytes);
  }

  void endPortTransaction() {
    if (!transactionEnded_) {
      port_.endTransaction();
      transactionEnded_ = true;
    }
  }

  void fail(const char* reason) {
    state_ = IsoTpState::Fault;
    reason_ = reason;
    assembling_ = false;
    endPortTransaction();
  }

  void complete(uint32_t now) {
    if (!healthyAndTimely(now)) return;
    completionAt_ = current(now);
    state_ = IsoTpState::Complete;
    reason_ = "COMPLETE";
    assembling_ = false;
    endPortTransaction();
  }

  void accept(const Frame& frame, uint32_t now) {
    if (frame.id != responseId()) return;
    if (recordCount_ >= 64) { fail("RECORD_LIMIT"); return; }
    records_[recordCount_++] = {frame, now};
    if (frame.extended || frame.rtr || frame.dlc == 0 || frame.dlc > 8) {
      fail("BAD_FRAME"); return;
    }

    const uint8_t type = frame.data[0] >> 4;
    if (type == 0) {
      const unsigned size = frame.data[0];
      if (frame.dlc >= 4 && frame.data[1] == 0x7f && frame.data[2] == 0x22) {
        if (size == 3 && size + 1 <= frame.dlc) {
          length_ = received_ = 3;
          std::memcpy(payload_, frame.data + 1, 3);
          fail("NEGATIVE_RESPONSE");
        }
        else fail("BAD_LENGTH");
        return;
      }
      if (frame.dlc < 4 || !matchingHeader(frame.data + 1)) return;
      if (size < 3 || size > 7 || size + 1 > frame.dlc) { fail("BAD_LENGTH"); return; }
      if (assembling_) { fail("RESTARTED_RESPONSE"); return; }
      length_ = size;
      received_ = size;
      std::memcpy(payload_, frame.data + 1, size);
      complete(now);
      return;
    }

    if (type == 1) {
      if (frame.dlc < 5 || !matchingHeader(frame.data + 2)) return;
      const unsigned size = ((frame.data[0] & 15u) << 8) | frame.data[1];
      if (frame.dlc != 8 || size <= 7 || size > sizeof(payload_)) {
        fail("BAD_LENGTH"); return;
      }
      if (assembling_) { fail("RESTARTED_RESPONSE"); return; }
      if (!healthyAndTimely(now)) return;
      length_ = size;
      received_ = 6;
      std::memcpy(payload_, frame.data + 2, 6);
      assembling_ = true;
      sequence_ = 1;
      progressAt_ = current(now);
      if (!port_.permitCts(transactionId_, progressAt_)) { fail("CTS_NOT_PERMITTED"); return; }
      Frame flow{};
      flow.id = profileRequestId(profile_);
      flow.dlc = 8;
      flow.data[0] = 0x30;
      flow.data[2] = 5;
      if (!healthyAndTimely(now)) return;
      // Count the CTS attempt at the physical send boundary, including send failure.
      ++flowControls_;
      if (!port_.send(flow)) { fail("FC_FAILED"); return; }
      if (!healthyAndTimely(now)) return;
      return;
    }

    if (type != 2 || !assembling_) return;
    if ((frame.data[0] & 15u) != sequence_) { fail("SEQUENCE"); return; }
    const unsigned remaining = length_ - received_;
    const unsigned take = remaining < 7 ? remaining : 7;
    if (frame.dlc < take + 1) { fail("BAD_LENGTH"); return; }
    if (!healthyAndTimely(now)) return;
    std::memcpy(payload_ + received_, frame.data + 1, take);
    received_ += take;
    sequence_ = static_cast<uint8_t>((sequence_ + 1) & 15u);
    progressAt_ = current(now);
    if (received_ == length_) complete(progressAt_);
  }

  Port& port_;
  uint32_t (*clock_)();
  IsoTpState state_ = IsoTpState::Idle;
  Profile profile_ = Profile::Oil;
  const char* reason_ = "IDLE";
  uint32_t transactionId_ = 0;
  uint32_t requestAt_ = 0;
  uint32_t trialStart_ = 0;
  uint32_t trialLimit_ = 0;
  uint32_t progressAt_ = 0;
  uint32_t completionAt_ = 0;
  uint32_t flowControls_ = 0;
  uint32_t rawRx_ = 0;
  unsigned length_ = 0;
  unsigned received_ = 0;
  unsigned recordCount_ = 0;
  uint8_t payload_[256]{};
  uint8_t sequence_ = 1;
  IsoTpRecord records_[64]{};
  bool assembling_ = false;
  bool transactionEnded_ = false;
};
}
