#pragma once

#include "isotp.hpp"
#include "tpms.hpp"
#include "tx_guard.hpp"
#include "drive_gear.hpp"

#include <cstring>

namespace gaugev4 {
inline unsigned commandRate(const char* line) {
  return !kContinuous && std::strcmp(line, kPairTrial ? "START PAIR DRIVE 20" : kPairProbe ? "START PAIR 10" : kFastTrial ? "START COLLECT 20" : "START COLLECT 10") == 0 ? kRequestHz : 0u;
}

class Gauge {
 public:
  explicit Gauge(Port& port, uint32_t (*clock)() = nullptr)
      : port_(port), receiver_(port, clock), clock_(clock) {}
  Gauge(const Gauge&) = delete;
  Gauge& operator=(const Gauge&) = delete;

  void begin(uint32_t now, bool locked, uint32_t startupDelayMs=3000) {
    if (begun_) return;
    begun_ = true;
    fallbackNow_ = now;
    bootAt_ = now;
    startupDelayMs_ = startupDelayMs;
    if (locked) finish(State::Lockout, "LOCKOUT");
    else if (kContinuous && state_ == State::BootWait) {
      if (!port_.selectRate(kRequestHz)) { finish(State::Fault, "RATE_SELECT"); return; }
      rate_ = kRequestHz;
      reason_ = startupDelayMs_ ? "AUTO_WAIT_3S" : "READINESS_CONFIRMED";
    }
  }

  bool command(const char* line, uint32_t now = 0) {
    fallbackNow_ = now;
    const unsigned rate = commandRate(line);
    if (rate) {
      if (!begun_ || state_ != State::BootWait || rate_ != 0) return false;
      if (!port_.selectRate(rate)) { finish(State::Fault, "RATE_SELECT"); return false; }
      rate_ = rate;
      bootAt_ = now;
      reason_ = "ARMED_3S";
      return true;
    }
    if (std::strcmp(line, "STOP") != 0) return false;
    begun_ = true;
    if (state_ != State::Fault && state_ != State::Lockout)
      finish(State::Stopped, "STOPPED");
    return true;
  }

  void tick(uint32_t now) {
    fallbackNow_ = now;
    if (!begun_) return;
    if (state_ == State::BootWait) {
      if (!rate_ || current(now) - bootAt_ < startupDelayMs_) return;
      if (!port_.open()) { finish(State::Fault, "CAN_INIT"); return; }
      if (!port_.good()) { finish(State::Fault, "CAN_ERROR"); return; }
      state_ = State::Active;
      reason_ = "DISCOVER";
      activeAt_ = port_.trialStartedAt();
      if (!receiver_.setTrialWindow(activeAt_, 60000)) {
        finish(State::Fault, "RX_TRIAL_INIT");
        return;
      }
    }
    if (state_ != State::Active || !activeAndHealthy(now)) return;

    if (pending_ && pendingProfile_ != Profile::Standard) {
      receiver_.tick(now);
      collectReceiverCounts();
      // Receiver operations can advance the actual clock; the enclosing trial wins.
      if (!activeAndHealthy(now)) return;
      if (receiver_.state() == IsoTpState::Fault) {
        finish(State::Fault, receiver_.reason());
        return;
      }
      if (receiver_.state() == IsoTpState::Complete) {
        completeIso();
      }
      return;
    }

    if (uiPending_) return;

    Frame frame{};
    bool empty = false;
    for (unsigned drained = 0; drained < 128; ++drained) {
      if (!activeAndHealthy(now)) return;
      const bool receivedFrame = port_.receive(frame);
      if (!activeAndHealthy(now)) return;
      if (!receivedFrame) { empty = true; break; }
      if (pending_) acceptStandard(frame, current(now));
      if (state_ != State::Active || !pending_) {
        if (uiPending_) return;
      }
    }
    if (!empty) { finish(State::Fault, "RX_BACKLOG"); return; }
    if (!activeAndHealthy(now) || pending_) return;

    const uint32_t actual = current(now);
    if (requests_ && actual - lastTx_ < 1000u / rate_) return;
    if (!kContinuous && requests_ >= 60u * rate_) return;
    // Never wrap request identities or timing counters into a new transaction.
    if (kContinuous && requests_ >= UINT32_MAX - 1u) {
      finish(State::Stopped, "COUNT_LIMIT"); return;
    }
    if(kPairDrive && !discovering_ && (!supports(0x0c) || !supports(0x0d))) {
      finish(State::Stopped,"PAIR_UNSUPPORTED");return;
    }

    Profile profile = Profile::Standard;
    uint8_t key = 0;
    Frame request{};
    request.dlc = 8;
    const bool scheduledPair=kPairDrive && !discovering_;
    const uint8_t scheduledKey=scheduledPair?nextPairPid(actual):0;
    if(kPairProbe) {
      request.id=0x7df;request.data[1]=1;
      if(discovering_) {key=page_;request.data[0]=2;request.data[2]=key;}
      else {
        if(!supports(0x0c) || !supports(0x0d)){finish(State::Stopped,"PAIR_UNSUPPORTED");return;}
        key=0x85;request.data[0]=3;request.data[2]=0x0c;request.data[3]=0x0d;
      }
    } else if (scheduledPair ? scheduledKey==0x83 :
               (brakeDue(Profile::BrakePressure, actual) || brakeDue(Profile::BrakeSwitch, actual))) {
      profile = brakeDue(Profile::BrakePressure, actual) ? Profile::BrakePressure : Profile::BrakeSwitch;
      key = profile == Profile::BrakePressure ? 0x82 : 0x83;
      request.id = profileRequestId(profile);
      request.data[0] = 3;
      request.data[1] = 0x22;
      request.data[2] = static_cast<uint8_t>(profileDid(profile) >> 8);
      request.data[3] = static_cast<uint8_t>(profileDid(profile));
    } else if (scheduledPair ? scheduledKey==0x84 : (kDriveUi && !discovering_ &&
               (haveReverseTx_ ? actual-lastReverseTx_>=2000 : actual-activeAt_>=2000))) {
      profile=Profile::Reverse;key=0x84;request.id=0x770;
      request.data[0]=3;request.data[1]=0x22;request.data[2]=0xbc;request.data[3]=4;
    } else if (scheduledPair ? scheduledKey==0x81 : tpmsDue(actual)) {
      profile = Profile::Tpms;
      key = 0x81;
      request.id = 0x7a0;
      request.data[0] = 3;
      request.data[1] = 0x22;
      request.data[2] = 0xc0;
      request.data[3] = 0x0b;
    } else {
      key = scheduledPair ? scheduledKey : discovering_ ? page_ : nextPid(actual);
      if(key==0xfd)return; // No due/supported read: do not invent a fallback TX.
      if(kPairDrive && key==0x85) {
        request.id=0x7df;request.data[0]=3;request.data[1]=1;
        request.data[2]=0x0c;request.data[3]=0x0d;
      } else if (key == 0x80) {
        profile = Profile::Oil;
        request.id = 0x7e0;
        request.data[0] = 3;
        request.data[1] = 0x22;
        request.data[2] = 0xe0;
        request.data[3] = 1;
      } else {
        request.id = 0x7df;
        request.data[0] = 2;
        request.data[1] = 1;
        request.data[2] = key;
      }
    }

    Profile verified = Profile::Standard;
    if (!requestProfile(request, verified) || verified != profile) {
      finish(State::Fault, "TX_WHITELIST");
      return;
    }

    // This is the actual attempt boundary used for response deadlines and RTT.
    const uint32_t requestAttemptAt = current(now);
    const uint32_t transactionId = requests_ + 1;
    ++requests_;
    ++profileCounters_[profileIndex(profile)].requests;
    if (!port_.send(request)) {
      // A send rejected at the enclosing deadline is normal trial completion.
      if (!kContinuous && current(now) - activeAt_ >= 60000)
        finish(State::Stopped, "TRIAL_COMPLETE");
      else
        finish(State::Fault, "TX_FAILED");
      return;
    }
    // Return time is intentionally used only for conservative next-request spacing.
    lastTx_ = current(now);
    pendingAt_ = requestAttemptAt;
    pendingOrdinal_ = transactionId;
    pending_ = true;
    receiverActive_ = false;
    standardPartialLength_ = 0;
    pendingProfile_ = profile;
    pendingKey_ = key;
    if(profile==Profile::Reverse){haveReverseTx_=true;lastReverseTx_=lastTx_;}
    if (profile == Profile::BrakePressure || profile == Profile::BrakeSwitch)
      lastBrakeTx_[profileIndex(profile) - 3] = lastTx_;
    if (profile == Profile::Tpms) {
      // Conservative return time cannot precede the HardwarePort guard's actual check.
      lastTpmsTx_ = lastTx_;
      haveTpmsTx_ = true;
    }
    // send() can advance the real clock or expose a health/deadline transition.
    if (!activeAndHealthy(now)) return;
    if (current(now) - pendingAt_ >= 500) {
      finish(State::Fault, "NO_RESPONSE");
      return;
    }
    if (profile != Profile::Standard) {
      receiverFlowSeen_ = 0;
      if (!receiver_.start(profile, transactionId, pendingAt_)) {
        finish(State::Fault, "RX_INIT");
      } else receiverActive_ = true;
    }
  }

  void uiServiced() { if (!pending_) uiPending_ = false; }
  bool pending() const { return pending_; }
  unsigned rateHz() const { return rate_; }
  State state() const { return state_; }
  const char* reason() const { return reason_; }
  uint32_t requests() const { return requests_; }
  uint32_t responses() const { return responses_; }
  uint32_t flowControls() const { return flowControls_; }
  const Observation& observation() const { return observation_; }

  Sample sample(uint8_t pid) const {
    const int index = sampleIndex(pid);
    return index < 0 ? Sample{} : samples_[index];
  }

  bool fresh(uint8_t pid, uint32_t now) const {
    const Sample value = sample(pid);
    const bool fast = pid == 0x0c || pid == 0x0b || pid == 0x0d ||
                      pid == 0x49 || pid == 0x4a;
    // Display expiry follows the existing 10Hz mixed schedule, not per-query RTT.
    // Alternating pedal channels can arrive 2.2s apart; speed can exceed 1s.
    const uint32_t ttl = (pid == 0x49 || pid == 0x4a || pid == 0x84) ? 2500u :
                         (pid == 0x0d || pid == 0x83) ? 1500u :
                         fast ? (rate_ ? 10000u / rate_ : 0u) : 20000u;
    return state_ == State::Active && value.present && now - value.at < ttl;
  }

  bool boostFresh(uint32_t now) const {
    return fresh(0x0b, now) && fresh(0x33, now) && now - sample(0x33).at < 10000;
  }

  uint8_t pedalPid() const {
    if(kDriveUi)return supports(0x49)?0x49:0; // Only D has observed endpoints.
    return supports(0x49) ? 0x49 : supports(0x4a) ? 0x4a : 0;
  }

  unsigned estimatedGear(uint32_t now) const {
    return kDriveUi && state_==State::Active ? gear_.value(sample(0x0c),sample(0x0d),sample(0x84),now) : 0;
  }
  GearPresentation gearPresentation(uint32_t now) const {
    return kDriveUi && state_==State::Active ? gear_.presentation(sample(0x0c),sample(0x0d),sample(0x84),now) : GearPresentation{};
  }
  const char* gearReason(uint32_t now) const {
    return state_==State::Active?gear_.reason(sample(0x0c),sample(0x0d),sample(0x84),now):"INACTIVE";
  }

  unsigned oilLength() const { return oilLength_; }
  const uint8_t* oilPayload() const { return oilPayload_; }
  const TpmsSample& tpms() const { return tpms_; }
  const Counters& counters(Profile profile) const { return profileCounters_[profileIndex(profile)]; }
  const Timing& timing(uint8_t pid) const { return timings_[pid]; }
  const Timing& totalTiming() const { return totalTiming_; }
  uint32_t meanRttMs() const { return totalTiming_.meanRttMs; }
  uint32_t maxRttMs() const { return totalTiming_.maxRttMs; }
  uint32_t maxCompletionGapMs() const { return totalTiming_.maxCompletionGapMs; }

 private:
  static unsigned profileIndex(Profile profile) { return static_cast<unsigned>(profile); }
  uint32_t current(uint32_t fallback) const { return clock_ ? clock_() : fallback; }

  bool activeAndHealthy(uint32_t fallback) {
    const uint32_t now = current(fallback);
    if (!kContinuous && now - activeAt_ >= 60000) {
      finish(State::Stopped, "TRIAL_COMPLETE");
      return false;
    }
    if (!port_.good()) {
      finish(State::Fault, "CAN_ERROR");
      return false;
    }
    if (pending_ && pendingProfile_ == Profile::Standard && now - pendingAt_ >= 500) {
      finish(State::Fault, "NO_RESPONSE");
      return false;
    }
    return true;
  }

  bool tpmsDue(uint32_t now) const {
    return !discovering_ && !tpmsDisabled_ &&
           (kContinuous || profileCounters_[profileIndex(Profile::Tpms)].requests < 12) &&
           (haveTpmsTx_ ? now - lastTpmsTx_ >= kTpmsIntervalMs : now - activeAt_ >= 5000);
  }

  bool brakeDue(Profile profile, uint32_t now) const {
    if (kSwitchOnly && profile == Profile::BrakePressure) return false;
    const unsigned index = profileIndex(profile) - 3;
    const uint32_t count = profileCounters_[profileIndex(profile)].requests;
    return profile == Profile::BrakeSwitch && !discovering_ && (kContinuous || count < 116) &&
           (count ? now - lastBrakeTx_[index] >= 500u : now - activeAt_ >= 2000u);
  }

  bool supports(uint8_t pid) const {
    if (!pid || pid > 0x60) return false;
    const unsigned page = (pid - 1) / 32;
    for (unsigned index = 0; index <= page; ++index)
      if (!known_[index] || (index < page && !(maps_[index] & 1u))) return false;
    return (maps_[page] & (uint32_t(1) << (32 - (pid - page * 32)))) != 0;
  }

  bool slowDue(uint8_t pid,uint32_t now) const {
    const Timing& previous=timings_[pid];
    return !previous.hasCompletion || now-previous.lastCompletionAt >= (pid==0x33?4000u:10000u);
  }

  uint8_t nextPairPid(uint32_t now) {
    // Ten globally rate-limited requests, with two separated background slots.
    // All due diagnostics share those slots instead of preempting fast reads.
    constexpr uint8_t order[]={0x85,0x49,0x0b,0x85,0xff,0x85,0x49,0x0b,0x85,0xff};
    const uint8_t key=order[pairSlot_];pairSlot_=(pairSlot_+1)%10;
    if(key!=0xff)return key==0x85 || supports(key)?key:0x85;
    uint8_t selected=0xfd;uint32_t oldest=0;
    auto consider=[&](uint8_t candidate,bool due,uint32_t late){
      if(due && (selected==0xfd || late>oldest)){selected=candidate;oldest=late;}
    };
    const uint32_t reverseAge=now-(haveReverseTx_?lastReverseTx_:activeAt_);
    const uint32_t tpmsAge=now-(haveTpmsTx_?lastTpmsTx_:activeAt_);
    consider(0x81,tpmsDue(now),tpmsAge-(haveTpmsTx_?kTpmsIntervalMs:5000u));
    for(uint8_t candidate:{uint8_t(0x33),uint8_t(5),uint8_t(0x0f),uint8_t(0x42),uint8_t(0x80)}){
      const Timing& t=timings_[candidate];
      const uint32_t late=t.hasCompletion?now-t.lastCompletionAt-(candidate==0x33?4000u:10000u):now-activeAt_;
      consider(candidate,(candidate==0x80 || supports(candidate)) && slowDue(candidate,now),late);
    }
    // Under overload a background slot can take longer than a brake period.
    // Age out that priority after 5s of lateness so temperatures cannot starve.
    // Existing freshness gates still hide any overdue safety-related value.
    if(selected!=0xfd && oldest>=5000u)return selected;
    if(reverseAge>=2000u)return 0x84;
    if(brakeDue(Profile::BrakeSwitch,now))return 0x83;
    // Spare background time improves pedal cadence; never invent an unsupported PID.
    return selected!=0xfd?selected:supports(0x49)?0x49:0x85;
  }

  uint8_t nextPid(uint32_t now) {
    const uint8_t schedule[10]={0x0c,0x0d,0xfe,0x0b,0x0c,0x0d,0xfe,0x0b,0x0c,0xff};
    // Manual 20Hz only: three near-time RPM/speed pairs, two pedal/MAP reads,
    // and one slow opportunity. Keep the proven 10Hz comparison unchanged.
    const uint8_t fastSchedule[11]={0x0c,0x0d,0xfe,0x0b,0x0c,0x0d,0xfe,0x0b,0x0c,0x0d,0xff};
    // Same total request cap; one transaction replaces each individual pair.
    const uint8_t pairSchedule[8]={0x85,0xfe,0x0b,0x85,0xfe,0x0b,0x85,0xff};
    const uint8_t legacy[8]={0x0c,0x0b,0x0c,0xfe,0x0c,0x0b,0x0d,0xff};
    const unsigned slots=kPairDrive?8u:kDriveUi?(kFastTrial?11u:10u):8u;
    const uint8_t slow[5] = {0x33, 5, 0x0f, 0x42, 0x80};
    for (unsigned tries = 0; tries < slots; ++tries) {
      uint8_t pid = kPairDrive?pairSchedule[slot_]:kDriveUi?(kFastTrial?fastSchedule[slot_]:schedule[slot_]):legacy[slot_];
      slot_ = static_cast<uint8_t>((slot_ + 1) % slots);
      if (pid == 0xfe) {
        pid = pedalPid();
        if (!kDriveUi && supports(0x49) && supports(0x4a)) {
          pid = pedalAlternate_ ? 0x4a : 0x49;
          pedalAlternate_ = !pedalAlternate_;
        }
      }
      if (pid == 0xff) {
        // Most overdue deadline wins, rather than BARO taking every slow slot
        // under load. Unsampled supported channels are due from activeAt_.
        // Age subtraction remains valid across millis() rollover. Existing
        // freshness rules still hide values if hardware cannot keep up.
        if(kDriveUi) {
          uint8_t selected=0xfd;uint32_t oldest=0;
          for(unsigned i=0;i<5;++i) {
            const uint8_t candidate=slow[(slowSlot_+i)%5];
            if((candidate!=0x80 && !supports(candidate)) || !slowDue(candidate,now))continue;
            const Timing& previous=timings_[candidate];
            const uint32_t late=previous.hasCompletion
                ? now-previous.lastCompletionAt-(candidate==0x33?4000u:10000u)
                : now-activeAt_;
            if(selected==0xfd || late>oldest){selected=candidate;oldest=late;}
          }
          if(selected!=0xfd) {
            for(unsigned i=0;i<5;++i)if(slow[i]==selected)slowSlot_=static_cast<uint8_t>((i+1)%5);
            return selected;
          }
        }
        bool due=false;
        for (unsigned slowTries = 0; slowTries < 5; ++slowTries) {
          pid = slow[slowSlot_];
          slowSlot_ = static_cast<uint8_t>((slowSlot_ + 1) % 5);
          if ((pid == 0x80 || supports(pid)) &&
              (!kDriveUi || slowDue(pid,now))) {due=true;break;}
        }
        if(kDriveUi && !due) {
          sparePedal_=!sparePedal_;
          pid=sparePedal_ && pedalPid()?pedalPid():(kPairDrive?0x85:0x0c);
        }
      }
      if (pid == 0x80 || (kPairDrive && pid==0x85) || supports(pid)) return pid;
    }
    return !kDriveUi || slowDue(0x80,now)?0x80:0xfd;
  }

  static int sampleIndex(uint8_t pid) {
    const uint8_t ids[12] = {0x0c, 5, 0x0d, 0x0f, 0x42, 0x0b, 0x33, 0x80, 0x49, 0x4a, 0x83, 0x84};
    for (unsigned index = 0; index < 12; ++index) if (ids[index] == pid) return index;
    return -1;
  }

  void noteResponse(uint8_t key, uint32_t completionAt) {
    if(kDriveUi)gear_.update(sample(0x0c),sample(0x0d),sample(0x84),completionAt,sample(0x49));
    timings_[key].note(pendingAt_, completionAt);
    totalTiming_.note(pendingAt_, completionAt);
    ++responses_;
    ++profileCounters_[profileIndex(pendingProfile_)].responses;
  }

  void collectReceiverCounts() {
    const uint32_t count = receiver_.flowControls();
    if (count > receiverFlowSeen_) {
      const uint32_t added = count - receiverFlowSeen_;
      flowControls_ += added;
      profileCounters_[profileIndex(pendingProfile_)].flowControls += added;
      receiverFlowSeen_ = count;
    }
  }

  void completeIso() {
    const uint32_t at = receiver_.completionAt();
    if (pendingProfile_ == Profile::Oil) {
      oilLength_ = receiver_.length();
      std::memcpy(oilPayload_, receiver_.payload(), oilLength_);
      const int index = sampleIndex(0x80);
      if (index >= 0) {
        samples_[index] = {};
        if (oilLength_ >= 38)
          samples_[index] = {true, oilPayload_[37] * 0.7435f - 49.2f, at, pendingAt_, pendingOrdinal_};
      }
    } else if (pendingProfile_ == Profile::Tpms) {
      if (!decodeTpms(receiver_.payload(), receiver_.length(), at, tpms_)) tpmsDisabled_ = true;
    } else if (pendingProfile_ == Profile::BrakeSwitch || pendingProfile_ == Profile::Reverse) {
      Sample& brake = samples_[sampleIndex(pendingProfile_==Profile::Reverse?0x84:0x83)];
      brake = {};
      // Only the observed complete schema; P8 is known to vary independently.
      if (receiver_.length() == 11 && profilePrefixMatches(pendingProfile_, receiver_.payload()))
        brake = {true, static_cast<float>(receiver_.payload()[7]), at, pendingAt_, pendingOrdinal_};
    }
    observe(true, receiver_.payload(), receiver_.length(), at);
    noteResponse(pendingKey_, at);
    pending_ = false;
    uiPending_ = true;
  }

  void acceptStandard(const Frame& frame, uint32_t now) {
    if (frame.id != 0x7e8) return;
    if (frame.extended || frame.rtr || frame.dlc < 3 || frame.dlc > 8) {
      finish(State::Fault, "BAD_FRAME"); return;
    }
    const unsigned size = frame.data[0];
    if (frame.dlc >= 4 && size == 3 && frame.data[1] == 0x7f && frame.data[2] == 1) {
      standardPartialLength_ = 3;
      std::memcpy(standardPartial_, frame.data + 1, 3);
      finish(State::Fault, "NEGATIVE_RESPONSE"); return;
    }
    if(kPairEnabled && pendingKey_==0x85) {
      if(frame.data[1]!=0x41)return;
      standardPartialLength_=size<static_cast<unsigned>(frame.dlc-1)?size:frame.dlc-1;
      std::memcpy(standardPartial_,frame.data+1,standardPartialLength_);
      // Accept only a complete SF with both exact PIDs, in either order.
      bool rpmSeen=false,speedSeen=false;float rpm=0,speed=0;
      if(size!=6 || frame.dlc<7){finish(State::Fault,"BAD_PAIR_LENGTH");return;}
      unsigned pos=2;
      while(pos<=size) {
        const uint8_t pid=frame.data[pos++];
        if(pid==0x0c && !rpmSeen && pos+1<=size) {
          rpm=(uint32_t(frame.data[pos])*256+frame.data[pos+1])/4.0f;
          pos+=2;rpmSeen=true;
        } else if(pid==0x0d && !speedSeen && pos<=size) {speed=frame.data[pos++];speedSeen=true;}
        else {finish(State::Fault,"BAD_PAIR_SCHEMA");return;}
      }
      if(!rpmSeen || !speedSeen){finish(State::Fault,"BAD_PAIR_SCHEMA");return;}
      if(!activeAndHealthy(now))return;
      now=current(now);
      samples_[sampleIndex(0x0c)]={true,rpm,now,pendingAt_,pendingOrdinal_};
      samples_[sampleIndex(0x0d)]={true,speed,now,pendingAt_,pendingOrdinal_};
      timings_[0x0c].note(pendingAt_,now);timings_[0x0d].note(pendingAt_,now);
      observe(true,frame.data+1,size,now);noteResponse(0x85,now);
      pending_=false;
      if(kPairProbe)finish(State::Stopped,"PAIR_PROBE_COMPLETE");
      else {port_.endTransaction();uiPending_=true;}
      return;
    }
    if (frame.data[1] != 0x41 || frame.data[2] != pendingKey_) return;
    standardPartialLength_ = size < static_cast<unsigned>(frame.dlc - 1) ? size : frame.dlc - 1;
    std::memcpy(standardPartial_, frame.data + 1, standardPartialLength_);
    const unsigned expected = discovering_ ? 6u
                            : (pendingKey_ == 0x0c || pendingKey_ == 0x42) ? 4u : 3u;
    if (size != expected || size + 1 > frame.dlc) {
      finish(State::Fault, "BAD_LENGTH"); return;
    }
    if (!activeAndHealthy(now)) return;
    now = current(now);
    if (discovering_) {
      const uint32_t bits = (uint32_t(frame.data[3]) << 24) |
                            (uint32_t(frame.data[4]) << 16) |
                            (uint32_t(frame.data[5]) << 8) | frame.data[6];
      maps_[page_ / 32] = bits;
      known_[page_ / 32] = true;
      if (page_ < 0x40 && (bits & 1u)) page_ = static_cast<uint8_t>(page_ + 0x20);
      else { discovering_ = false; reason_ = kContinuous ? "ACTIVE_CONTINUOUS" : "ACTIVE_TRIAL"; }
    } else {
      float value = frame.data[3];
      if (pendingKey_ == 0x0c)
        value = (uint32_t(frame.data[3]) * 256 + frame.data[4]) / 4.0f;
      else if (pendingKey_ == 0x42)
        value = (uint32_t(frame.data[3]) * 256 + frame.data[4]) / 1000.0f;
      else if (pendingKey_ == 5 || pendingKey_ == 0x0f)
        value = static_cast<float>(static_cast<int>(frame.data[3]) - 40);
      else if (pendingKey_ == 0x49 || pendingKey_ == 0x4a)
        value = frame.data[3] * 100.0f / 255.0f;
      const int index = sampleIndex(pendingKey_);
      if (index >= 0) samples_[index] = {true, value, now, pendingAt_, requests_};
    }
    observe(true, frame.data + 1, size, now);
    noteResponse(pendingKey_, now);
    port_.endTransaction();
    pending_ = false;
    uiPending_ = true;
  }

  void observe(bool complete, const uint8_t* payload, unsigned length, uint32_t at) {
    const uint32_t generation = observation_.generation + 1;
    observation_ = {};
    observation_.generation = generation;
    observation_.requestOrdinal = pendingOrdinal_;
    observation_.requestAt = pendingAt_;
    observation_.receivedAt = at;
    observation_.profile = pendingProfile_;
    observation_.key = pendingKey_;
    observation_.complete = complete;
    observation_.length = length < sizeof(observation_.payload) ? length : sizeof(observation_.payload);
    if (observation_.length) std::memcpy(observation_.payload, payload, observation_.length);
  }

  void finish(State state, const char* reason) {
    if (pending_) {
      const bool iso = pendingProfile_ != Profile::Standard && receiverActive_;
      const unsigned length = iso ? receiver_.received() : pendingProfile_ == Profile::Standard ? standardPartialLength_ : 0;
      observe(false, iso ? receiver_.payload() : standardPartial_, length, current(fallbackNow_));
    }
    port_.endTransaction();
    port_.stop();
    state_ = state;
    reason_ = reason;
    pending_ = false;
    uiPending_ = true;
  }

  Port& port_;
  IsoTpReceiver receiver_;
  uint32_t (*clock_)();
  uint32_t startupDelayMs_=3000; // Only the sleep controller supplies 0 after a valid readiness reply.
  State state_ = State::BootWait;
  const char* reason_ = "WAIT_COLLECT";
  bool begun_ = false;
  bool pending_ = false;
  bool discovering_ = true;
  bool uiPending_ = false;
  bool tpmsDisabled_ = false;
  bool haveTpmsTx_ = false;
  bool receiverActive_ = false;
  unsigned rate_ = 0;
  uint32_t bootAt_ = 0;
  uint32_t activeAt_ = 0;
  uint32_t lastTx_ = 0;
  uint32_t lastTpmsTx_ = 0;
  uint32_t lastBrakeTx_[2]{};
  uint32_t pendingAt_ = 0;
  uint32_t pendingOrdinal_ = 0;
  uint32_t fallbackNow_ = 0;
  uint32_t requests_ = 0;
  uint32_t responses_ = 0;
  uint32_t flowControls_ = 0;
  uint32_t receiverFlowSeen_ = 0;
  Profile pendingProfile_ = Profile::Standard;
  uint8_t pendingKey_ = 0;
  uint8_t page_ = 0;
  uint8_t slot_ = 0;
  uint8_t pairSlot_ = 0;
  uint8_t slowSlot_ = 0;
  bool pedalAlternate_ = false;
  bool sparePedal_ = false;
  uint32_t maps_[3]{};
  bool known_[3]{};
  Sample samples_[12]{};
  Counters profileCounters_[6]{};
  DriveGear gear_{};
  bool haveReverseTx_=false;
  uint32_t lastReverseTx_=0;
  Timing timings_[256]{};
  Timing totalTiming_{};
  unsigned oilLength_ = 0;
  uint8_t oilPayload_[256]{};
  TpmsSample tpms_{};
  Observation observation_{};
  uint8_t standardPartial_[7]{};
  unsigned standardPartialLength_ = 0;
};
}
