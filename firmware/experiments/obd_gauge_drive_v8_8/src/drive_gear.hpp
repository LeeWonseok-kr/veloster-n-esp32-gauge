#pragma once
#include "types.hpp"
#include <cmath>

namespace gaugev4 {
// Exact observed BC04 P7 states only. 0x04's meaning is not established.
// 2026-09-16: 01/03; 2026-09-27 engine-running parked comparison: 05/07.
// NotReverse must NEVER be interpreted as neutral.
inline bool observedReverse(float raw) {return raw==3 || raw==7;}
inline bool observedNotReverse(float raw) {return raw==1 || raw==5;}
// 2026-09-27 moving capture: six exact 0D observations during road travel.
// Permit ratio estimation only; this does not decode bit08, neutral or 0F.
inline bool forwardEstimationAllowed(float raw) {return observedNotReverse(raw) || raw==13;}
enum class GearConfidence { None, Candidate, Held, Confirmed };
struct GearPresentation { unsigned gear=0; GearConfidence confidence=GearConfidence::None; };
inline const char* gearConfidenceName(GearConfidence c) {
  switch(c) {
    case GearConfidence::Candidate:return "CANDIDATE";
    case GearConfidence::Held:return "HELD";
    case GearConfidence::Confirmed:return "CONFIRMED_EST";
    default:return "NONE";
  }
}
// Experimental related-model 6MT Performance ratios; NOT a confirmed ECU gear.
// Source and vehicle limitations: README.md. Nominal 235/35R19 circumference.
class DriveGear {
 public:
  void update(const Sample& rpm, const Sample& speed, const Sample& reverse, uint32_t now,
              const Sample& pedal = Sample{}) {
    notePedal(pedal,now);
    if (!inputsFresh(rpm,speed,reverse,now) || !forwardEstimationAllowed(reverse.value) || speed.value<10 || rpm.value<700) {
      clear();reason_="INPUT_GATE";return;
    }
    // An asynchronous pair cannot confirm OR contradict a ratio. Wait for a
    // near-time partner without refreshing lastPairAt_: value() still expires
    // the prior estimate after 1500ms, and input/state gates above still clear.
    const uint32_t gapA=rpm.at-speed.at,gapB=speed.at-rpm.at;
    if(gapA>350 && gapB>350){
      reason_="PAIR_SKEW";
      if(heldGear_ && std::fabs(rpm.value/speed.value-lastRatio_)>lastRatio_*0.35f){clear();reason_="RPM_DEPARTURE";}
      return;
    }
    // A repeated UI tick/RPM packet cannot confirm a second speed observation.
    if(speed.requestOrdinal==speedSeen_ || rpm.requestOrdinal==rpmSeen_) {
      // A small asynchronous ratio change is not a second observation.
      // Still reject a gross RPM departure; never renew lastPairAt_ here.
      reason_="WAIT_PAIR";
      if((havePair_ || heldGear_) && std::fabs(rpm.value/speed.value-lastRatio_)>lastRatio_*0.35f){clear();reason_="RPM_DEPARTURE";}
      return;
    }
    speedSeen_=speed.requestOrdinal;
    rpmSeen_=rpm.requestOrdinal;
    const unsigned gear=classify(rpm.value/speed.value);
    if(!gear){
      // Presentation-only grace for a small ratio disturbance under power.
      // Never extend its deadline with repeated invalid pairs or UI ticks.
      const bool near=std::fabs(rpm.value/speed.value-lastRatio_)<=lastRatio_*0.15f;
      // A displayed candidate is already visible to the driver. Give it the
      // same bounded grace as a confirmed estimate; do not promote confidence.
      // Only a new valid pair may re-arm this deadline, never invalid repeats.
      const bool currentVisiblePair=candidate_ && havePair_ && now-lastPairAt_<1500;
      if(near && pedalPressed(now) && (currentVisiblePair || heldGear_)) {
        if(currentVisiblePair){heldGear_=candidate_;holdAt_=now;}
        candidate_=confirmed_=0;havePair_=false;reason_="RATIO_HOLD";
      } else {clear();reason_="OUT_OF_RATIO";}
      return;
    }
    heldGear_=0; // A real calculated candidate always supersedes a held digit.
    if(gear!=candidate_ || !havePair_ || now-lastPairAt_>=1500) {
      candidate_=gear;confirmed_=0;candidateAt_=now;
    } else if(now-candidateAt_>=200 && !(haveLift_ && now-liftAt_<400 && !confirmed_)) confirmed_=gear;
    reason_=confirmed_?"CONFIRMED":haveLift_ && now-liftAt_<400?"LIFT_SETTLE":"CONFIRMING";
    lastPairAt_=now;lastRatio_=rpm.value/speed.value;havePair_=true;
  }
  unsigned value(const Sample& rpm,const Sample& speed,const Sample& reverse,uint32_t now) const {
    if(!inputsFresh(rpm,speed,reverse,now) || !forwardEstimationAllowed(reverse.value) || speed.value<10 ||
       rpm.value<700 || !havePair_ || now-lastPairAt_>=1500)return 0;
    return confirmed_;
  }
  GearPresentation presentation(const Sample& rpm,const Sample& speed,const Sample& reverse,uint32_t now) const {
    if(!inputsFresh(rpm,speed,reverse,now) || !forwardEstimationAllowed(reverse.value) ||
       speed.value<10 || rpm.value<700)return {};
    if(havePair_ && now-lastPairAt_<1500 && candidate_)
      return {confirmed_?confirmed_:candidate_,confirmed_?GearConfidence::Confirmed:GearConfidence::Candidate};
    if(heldGear_ && now-holdAt_<400 && pedalPressed(now))return {heldGear_,GearConfidence::Held};
    return {};
  }
  const char* reason(const Sample& r,const Sample& s,const Sample& reverse,uint32_t now) const {
    if(!inputsFresh(r,s,reverse,now))return "STALE_INPUT";
    if(!forwardEstimationAllowed(reverse.value) || s.value<10 || r.value<700)return "INPUT_GATE";
    if(havePair_ && now-lastPairAt_>=1500)return "PAIR_EXPIRED";
    return reason_;
  }
 private:
  bool pedalPressed(uint32_t now) const {return havePedal_ && now-pedalAt_<800 && pedalRaw_>43;}
  void notePedal(const Sample& p,uint32_t now) {
    // PID49 is the accelerator sensor, NOT throttle-plate opening. The raw
    // release endpoint is observed 38; +5 counts is experimental noise margin.
    if(!p.present || !p.requestOrdinal || !std::isfinite(p.value) || p.value<0 || p.value>100 ||
       now-p.at>=800 || p.requestOrdinal==pedalSeen_)return;
    const int raw=static_cast<int>(std::lround(p.value*2.55f));
    if(havePedal_ && p.at-pedalAt_<1500 && pedalRaw_-raw>=20 && raw<=43) {
      liftAt_=p.at;haveLift_=true;
    }
    havePedal_=true;pedalSeen_=p.requestOrdinal;pedalAt_=p.at;pedalRaw_=raw;
  }
  static bool inputsFresh(const Sample& r,const Sample& s,const Sample& reverse,uint32_t now) {
    return r.present && s.present && reverse.present && r.requestOrdinal && s.requestOrdinal &&
      std::isfinite(r.value) && std::isfinite(s.value) && now-r.at<1000 &&
      now-s.at<1500 && now-reverse.at<2500;
  }
  static unsigned classify(float ratio) {
    constexpr float centers[]={109.52f,68.60f,45.19f,34.00f,27.36f,22.76f};
    unsigned result=0;
    for(unsigned i=0;i<6;++i)if(std::fabs(ratio-centers[i])<=centers[i]*0.07f) {
      if(result)return 0;
      result=i+1;
    }
    return result;
  }
  void clear(){candidate_=confirmed_=heldGear_=0;havePair_=false;}
  unsigned candidate_=0,confirmed_=0;
  unsigned heldGear_=0;
  uint32_t holdAt_=0;
  uint32_t speedSeen_=0,candidateAt_=0,lastPairAt_=0;
  uint32_t rpmSeen_=0;
  float lastRatio_=0;
  const char* reason_="CONFIRMING";
  uint32_t pedalSeen_=0,pedalAt_=0,liftAt_=0;
  int pedalRaw_=0;
  bool havePedal_=false,haveLift_=false;
  bool havePair_=false;
};
}
