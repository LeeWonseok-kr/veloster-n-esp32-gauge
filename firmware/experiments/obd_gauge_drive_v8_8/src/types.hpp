#pragma once

#include "../../obd_probe/src/probe.hpp"

#include <cstdint>

#ifndef GAUGE_DRIVE_UI
#define GAUGE_DRIVE_UI 0
#endif

#ifndef GAUGE_CONTINUOUS
#define GAUGE_CONTINUOUS 0
#endif
#ifndef GAUGE_REQUEST_HZ
#define GAUGE_REQUEST_HZ 10
#endif
#ifndef GAUGE_PAIR_PROBE
#define GAUGE_PAIR_PROBE 0
#endif
#ifndef GAUGE_PAIR_TRIAL
#define GAUGE_PAIR_TRIAL 0
#endif
#ifndef GAUGE_PAIR_CONTINUOUS
#define GAUGE_PAIR_CONTINUOUS 0
#endif
static_assert(GAUGE_PAIR_CONTINUOUS==0 || (GAUGE_PAIR_CONTINUOUS==1 && GAUGE_CONTINUOUS==1 && GAUGE_PAIR_TRIAL==0 && GAUGE_PAIR_PROBE==0 && GAUGE_DRIVE_UI==1 && GAUGE_REQUEST_HZ==20), "Continuous pair requires its own explicit 20Hz auto build");
static_assert(GAUGE_PAIR_TRIAL==0 || (GAUGE_PAIR_TRIAL==1 && GAUGE_PAIR_PROBE==0 && GAUGE_DRIVE_UI==1 && GAUGE_REQUEST_HZ==20), "Repeated pair is a separate manual 20Hz build");
static_assert(GAUGE_PAIR_PROBE==0 || (GAUGE_PAIR_PROBE==1 && GAUGE_DRIVE_UI==1 && GAUGE_REQUEST_HZ==10), "Pair probe is a separate manual 10Hz build");
static_assert(GAUGE_REQUEST_HZ==10 || GAUGE_REQUEST_HZ==20, "Only prepared 10/20Hz builds");
static_assert(GAUGE_REQUEST_HZ!=20 || GAUGE_DRIVE_UI==1, "20Hz requires the drive UI build");
static_assert(GAUGE_CONTINUOUS == 0 || GAUGE_CONTINUOUS == 1,
              "Continuous operation must be explicitly selected");

#ifndef GAUGE_COLLECT_SWITCH_ONLY
#define GAUGE_COLLECT_SWITCH_ONLY 1
#endif
static_assert(GAUGE_COLLECT_SWITCH_ONLY == 1,
              "Brake display trial must block the rejected pressure request");

namespace gaugev4 {
using obdprobe::Frame;
inline constexpr unsigned kRequestHz=GAUGE_REQUEST_HZ;
inline constexpr bool kPairContinuous=GAUGE_PAIR_CONTINUOUS==1;
inline constexpr bool kFastTrial=kRequestHz==20 && !kPairContinuous;
inline constexpr bool kPairProbe=GAUGE_PAIR_PROBE==1;
inline constexpr bool kPairTrial=GAUGE_PAIR_TRIAL==1;
inline constexpr bool kPairDrive=kPairTrial || kPairContinuous;
inline constexpr bool kPairEnabled=kPairProbe || kPairDrive;
// Existing 20Hz trials stay bounded; only the explicit pair-continuous build
// opts into auto start and operation beyond 60s. Fault/STOP remain latched.
inline constexpr bool kContinuous = GAUGE_CONTINUOUS == 1 && !kFastTrial && !kPairProbe;
inline constexpr bool kDriveUi = GAUGE_DRIVE_UI == 1;
inline constexpr uint32_t kTpmsIntervalMs=kDriveUi?10000u:5000u;
inline constexpr uint32_t kTpmsVisibleMs=kDriveUi?20000u:10000u;
inline constexpr bool kSwitchOnly = GAUGE_COLLECT_SWITCH_ONLY == 1;

enum class Profile : uint8_t { Standard, Oil, Tpms, BrakePressure, BrakeSwitch, Reverse };
inline bool diagnosticProfile(Profile profile) {
  return profile >= Profile::Oil && profile <= (kDriveUi ? Profile::Reverse : Profile::BrakeSwitch);
}
inline uint32_t profileRequestId(Profile profile) {
  switch (profile) {
    case Profile::Standard: return 0x7df;
    case Profile::Oil: return 0x7e0;
    case Profile::Tpms: return 0x7a0;
    case Profile::BrakePressure: return 0x7d1;
    case Profile::BrakeSwitch: return 0x770;
    case Profile::Reverse: return 0x770;
  }
  return 0;
}
inline uint32_t profileResponseId(Profile profile) {
  return profile == Profile::Standard ? 0x7e8 : diagnosticProfile(profile) ? profileRequestId(profile) + 8 : 0;
}
inline uint16_t profileDid(Profile profile) {
  switch (profile) {
    case Profile::Oil: return 0xe001;
    case Profile::Tpms: return 0xc00b;
    case Profile::BrakePressure: return 0x0104;
    case Profile::BrakeSwitch: return 0xbc06;
    case Profile::Reverse: return 0xbc04;
    default: return 0;
  }
}
inline bool profilePrefixMatches(Profile profile, const uint8_t* bytes) {
  return diagnosticProfile(profile) && bytes[0] == 0x62 &&
         bytes[1] == static_cast<uint8_t>(profileDid(profile) >> 8) &&
         bytes[2] == static_cast<uint8_t>(profileDid(profile));
}
enum class State : uint8_t { BootWait, Active, Stopped, Fault, Lockout };
enum class Exclusion : uint8_t { None, Schema, Pressure, Temperature, Status, Trailing };

struct Port : obdprobe::Port {
  virtual bool selectRate(unsigned rate) = 0;
  // Exact clock epoch passed to TxGuard::beginWindow() by a successful open().
  virtual uint32_t trialStartedAt() const = 0;
  // transactionId is the 1-based combined diagnostic-request ordinal.
  virtual bool permitCts(uint32_t transactionId, uint32_t now) = 0;
  virtual void endTransaction() = 0;
};

struct Sample {
  bool present = false;
  float value = 0;
  uint32_t at = 0;
  uint32_t requestAt = 0;
  uint32_t requestOrdinal = 0;
};

struct Observation {
  uint32_t generation = 0;
  uint32_t requestOrdinal = 0;
  uint32_t requestAt = 0;
  uint32_t receivedAt = 0;
  Profile profile = Profile::Standard;
  uint8_t key = 0;
  bool complete = false;
  unsigned length = 0;
  uint8_t payload[256]{};
};

struct Counters {
  uint32_t requests = 0;
  uint32_t responses = 0;
  uint32_t flowControls = 0;
};

struct Timing {
  uint32_t count = 0;
  uint32_t meanRttMs = 0;
  uint32_t maxRttMs = 0;
  uint32_t maxCompletionGapMs = 0;
  uint32_t lastCompletionAt = 0;
  uint64_t sumRttMs = 0;
  bool hasCompletion = false;

  void note(uint32_t requestAt, uint32_t completionAt) {
    const uint32_t rtt = completionAt - requestAt;
    if (rtt > maxRttMs) maxRttMs = rtt;
    if (hasCompletion) {
      const uint32_t gap = completionAt - lastCompletionAt;
      if (gap > maxCompletionGapMs) maxCompletionGapMs = gap;
    }
    lastCompletionAt = completionAt;
    hasCompletion = true;
    sumRttMs += rtt;
    ++count;
    meanRttMs = static_cast<uint32_t>(sumRttMs / count);
  }
};

struct TpmsSlot {
  uint8_t pressureRaw = 0;
  uint8_t temperatureRaw = 0;
  uint8_t statusRaw = 0;
  uint8_t trailingRaw = 0;
  Exclusion exclusion = Exclusion::Schema;

  float pressurePsi() const { return pressureRaw / 5.0f; }
  int temperatureC() const { return static_cast<int>(temperatureRaw) - 50; }
};

struct TpmsSample {
  uint8_t raw[256]{};
  unsigned length = 0;
  uint32_t receivedAt = 0;
  bool schemaAccepted = false;
  TpmsSlot slots[4]{};  // FL, FR, RR, RL candidate order; mapping remains unverified.

  bool visible(uint8_t slot, uint32_t now, bool active) const {
    return active && schemaAccepted && slot < 4 &&
           slots[slot].exclusion == Exclusion::None && now - receivedAt < kTpmsVisibleMs;
  }
  const char* sensorAge() const { return "UNKNOWN"; }
};
}
