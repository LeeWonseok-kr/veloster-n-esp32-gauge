#pragma once

#include "types.hpp"

#include <cstring>
#include <initializer_list>

namespace gaugev4 {
inline bool allowedPairRequest(const Frame& frame) {
  const uint8_t bytes[8]={3,1,0x0c,0x0d,0,0,0,0};
  return kPairEnabled && frame.id==0x7df && frame.dlc==8 && !frame.extended && !frame.rtr &&
         std::memcmp(frame.data,bytes,8)==0;
}
inline bool allowedStandardRequest(const Frame& frame) {
  if(allowedPairRequest(frame))return true;
  if (frame.id != 0x7df || frame.dlc != 8 || frame.extended || frame.rtr ||
      frame.data[0] != 2 || frame.data[1] != 1) return false;
  bool pidAllowed = false;
  for (uint8_t pid : {0, 0x20, 0x40, 0x0c, 5, 0x0d, 0x0f, 0x42, 0x0b,
                      0x33, 0x49, 0x4a})
    if (frame.data[2] == pid) pidAllowed = true;
  if (!pidAllowed) return false;
  if(kPairDrive && (frame.data[2]==0x0c || frame.data[2]==0x0d))return false;
  if(kPairProbe && frame.data[2]!=0 && frame.data[2]!=0x20 && frame.data[2]!=0x40)return false;
  for (unsigned i = 3; i < 8; ++i) if (frame.data[i] != 0) return false;
  return true;
}

inline bool allowedOilRequest(const Frame& frame) {
  const uint8_t bytes[8] = {3, 0x22, 0xe0, 1, 0, 0, 0, 0};
  return frame.id == 0x7e0 && frame.dlc == 8 && !frame.extended && !frame.rtr &&
         std::memcmp(frame.data, bytes, sizeof(bytes)) == 0;
}

inline bool allowedTpmsRequest(const Frame& frame) {
  const uint8_t bytes[8] = {3, 0x22, 0xc0, 0x0b, 0, 0, 0, 0};
  return frame.id == 0x7a0 && frame.dlc == 8 && !frame.extended && !frame.rtr &&
         std::memcmp(frame.data, bytes, sizeof(bytes)) == 0;
}

inline bool allowedFlow(const Frame& frame, Profile profile) {
  if (kSwitchOnly && profile == Profile::BrakePressure) return false;
  const uint8_t bytes[8] = {0x30, 0, 5, 0, 0, 0, 0, 0};
  const uint32_t id = profileRequestId(profile);
  return diagnosticProfile(profile) && frame.id == id && frame.dlc == 8 &&
         !frame.extended && !frame.rtr && std::memcmp(frame.data, bytes, sizeof(bytes)) == 0;
}

inline bool allowedBrakePressureRequest(const Frame& frame) {
  if (kSwitchOnly) return false;
  const uint8_t bytes[8] = {3, 0x22, 1, 4, 0, 0, 0, 0};
  return frame.id == 0x7d1 && frame.dlc == 8 && !frame.extended && !frame.rtr &&
         std::memcmp(frame.data, bytes, sizeof(bytes)) == 0;
}

inline bool allowedBrakeSwitchRequest(const Frame& frame) {
  const uint8_t bytes[8] = {3, 0x22, 0xbc, 6, 0, 0, 0, 0};
  return frame.id == 0x770 && frame.dlc == 8 && !frame.extended && !frame.rtr &&
         std::memcmp(frame.data, bytes, sizeof(bytes)) == 0;
}

inline bool requestProfile(const Frame& frame, Profile& profile) {
  if(kPairProbe) {
    profile=Profile::Standard;return allowedStandardRequest(frame);
  }
  const uint8_t reverse[8]={3,0x22,0xbc,4,0,0,0,0};
  if(kDriveUi && frame.id==0x770 && frame.dlc==8 && !frame.extended && !frame.rtr &&
     std::memcmp(frame.data,reverse,8)==0) {profile=Profile::Reverse;return true;}
  if (allowedStandardRequest(frame)) { profile = Profile::Standard; return true; }
  if (allowedOilRequest(frame)) { profile = Profile::Oil; return true; }
  if (allowedTpmsRequest(frame)) { profile = Profile::Tpms; return true; }
  if (allowedBrakePressureRequest(frame)) { profile = Profile::BrakePressure; return true; }
  if (allowedBrakeSwitchRequest(frame)) { profile = Profile::BrakeSwitch; return true; }
  return false;
}

class TxGuard {
 public:
  bool selectRate(unsigned rate) {
    if (intervalMs_ || rate != kRequestHz) return false;
    rate_ = rate;
    intervalMs_ = 1000 / rate;
    requestLimit_ = kContinuous ? UINT32_MAX - 1u : 60 * rate;
    return true;
  }

  bool beginWindow(uint32_t now) {
    if (!intervalMs_ || windowStarted_) return false;
    windowStarted_ = true;
    windowStart_ = now;
    return true;
  }

  bool allow(const Frame& frame, uint32_t transactionId, uint32_t now) {
    if (!windowStarted_ || (!kContinuous && now - windowStart_ >= 60000)) return false;

    if (allowedFlow(frame, Profile::Oil) || allowedFlow(frame, Profile::Tpms) ||
        allowedFlow(frame, Profile::BrakePressure) || allowedFlow(frame, Profile::BrakeSwitch)) {
      if (!pending_ || pendingProfile_ == Profile::Standard || transactionId != pendingId_ ||
          !ctsPermitted_ || ctsSent_ || now - pendingAt_ >= 500 ||
          !allowedFlow(frame, pendingProfile_)) return false;
      ctsPermitted_ = false;
      ctsSent_ = true;
      ++flowControlCount_;
      return true;
    }

    Profile profile = Profile::Standard;
    if (!requestProfile(frame, profile) || !transactionId || pending_ ||
        transactionId != requestCount_ + 1 || requestCount_ >= requestLimit_) return false;
    if(kPairProbe && pairSent_)return false;
    if (haveRequest_ && now - lastRequestAt_ < intervalMs_) return false;
    if (profile == Profile::Tpms) {
      if ((!haveTpms_ && now - windowStart_ < 5000) || (!kContinuous && tpmsCount_ >= 12) ||
          (haveTpms_ && now - lastTpmsAt_ < kTpmsIntervalMs)) return false;
    }
    if(profile==Profile::Reverse && ((!haveReverse_ && now-windowStart_<2000) ||
       (haveReverse_ && now-lastReverseAt_<2000)))return false;
    if (profile == Profile::BrakePressure || profile == Profile::BrakeSwitch) {
      const unsigned index = static_cast<unsigned>(profile) - 3;
      const uint32_t first = index == 0 ? 15000 : 2000;
      const uint32_t limit = index == 0 ? 4 : 116;
      const uint32_t gap = index == 0 ? 10000 : 500;
      if ((!brakeCount_[index] && now - windowStart_ < first) ||
          (!kContinuous && brakeCount_[index] >= limit) ||
          (brakeCount_[index] && now - lastBrakeAt_[index] < gap)) return false;
    }

    ++requestCount_;
    if(allowedPairRequest(frame))pairSent_=true;
    if(profile==Profile::Reverse){haveReverse_=true;lastReverseAt_=now;}
    if (profile == Profile::BrakePressure || profile == Profile::BrakeSwitch) {
      const unsigned index = static_cast<unsigned>(profile) - 3;
      ++brakeCount_[index];
      lastBrakeAt_[index] = now;
    }
    if (profile == Profile::Tpms) {
      ++tpmsCount_;
      lastTpmsAt_ = now;
      haveTpms_ = true;
    }
    lastRequestAt_ = now;
    haveRequest_ = true;
    pending_ = true;
    pendingProfile_ = profile;
    pendingId_ = transactionId;
    pendingAt_ = now;
    ctsPermitted_ = false;
    ctsSent_ = false;
    return true;
  }

  bool permitCts(uint32_t transactionId, uint32_t now) {
    if (!windowStarted_ || (!kContinuous && now - windowStart_ >= 60000) || !pending_ ||
        pendingProfile_ == Profile::Standard || pendingId_ != transactionId ||
        ctsPermitted_ || ctsSent_ || now - pendingAt_ >= 500) return false;
    ctsPermitted_ = true;
    return true;
  }

  void endTransaction() {
    pending_ = false;
    pendingId_ = 0;
    ctsPermitted_ = false;
    ctsSent_ = false;
  }

  unsigned rateHz() const { return rate_; }
  uint32_t requestCount() const { return requestCount_; }
  uint32_t tpmsCount() const { return tpmsCount_; }
  uint32_t flowControlCount() const { return flowControlCount_; }
  bool pending() const { return pending_; }

 private:
  unsigned rate_ = 0;
  uint32_t intervalMs_ = 0;
  uint32_t requestLimit_ = 0;
  uint32_t windowStart_ = 0;
  uint32_t lastRequestAt_ = 0;
  uint32_t lastTpmsAt_ = 0;
  uint32_t pendingAt_ = 0;
  uint32_t pendingId_ = 0;
  uint32_t requestCount_ = 0;
  uint32_t tpmsCount_ = 0;
  uint32_t flowControlCount_ = 0;
  uint32_t brakeCount_[2]{};
  uint32_t lastBrakeAt_[2]{};
  Profile pendingProfile_ = Profile::Standard;
  bool windowStarted_ = false;
  bool haveRequest_ = false;
  bool haveTpms_ = false;
  bool pending_ = false;
  bool ctsPermitted_ = false;
  bool ctsSent_ = false;
  bool haveReverse_ = false;
  bool pairSent_ = false;
  uint32_t lastReverseAt_ = 0;
};
}
