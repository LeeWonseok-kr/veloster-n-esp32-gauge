#pragma once

#include "gauge.hpp"
#include "display_alerts.hpp"

#include <dashboard_model.hpp>

#include <algorithm>
#include <array>
#include <cstdio>

namespace gaugev4 {
namespace model {

// User-selected display origin only. REL is NOT physical/OEM boost pressure.
// Keep the signed MAP-BARO source and raw observations unchanged.
inline constexpr float kBoostDisplayOffsetBar = 0.60f;

struct ReferenceMetadata {
  std::array<bool, 4> tpmsCandidateVisible{};
  bool tpmsReferenceOnly = true;
  bool oilReferenceOnly = true;
  bool sensorAgeUnknown = true;
};

struct Frame {
  DashboardFrame dashboard{};
  GearConfidence gearConfidence=GearConfidence::None;
  ReferenceMetadata reference{};
  alerts::DataState dataState = alerts::DataState::Waiting;
  alerts::ShiftState shift = alerts::ShiftState::Disabled;
};

inline Frame buildFrame(const Gauge& gauge, uint32_t now) {
  Frame frame{};
  DashboardFrame& dashboard = frame.dashboard;
  frame.shift = alerts::shiftState(gauge.sample(0x0c).value,gauge.fresh(0x0c,now));
  if (gauge.state()==State::Stopped || gauge.state()==State::Fault || gauge.state()==State::Lockout) {
    frame.dataState=alerts::DataState::Off;
  } else if (gauge.state()==State::Active) {
    bool seen=false,stale=false;
    // Only sampled display channels participate; unsupported/missing channels
    // stay '--'. DATA RX means recent observed data, not every sensor verified.
    const uint8_t pids[]={0x0c,5,0x0d,0x0f,0x42,0x0b,0x33,0x80,0x83,gauge.pedalPid()};
    for(uint8_t pid:pids) if(pid && gauge.sample(pid).present) {
      seen=true; stale=stale || !gauge.fresh(pid,now);
    }
    frame.dataState=stale?alerts::DataState::Stale:seen?alerts::DataState::Receiving:alerts::DataState::Waiting;
  }
  for (auto* text : {&dashboard.rpmText, &dashboard.boostText,
                     &dashboard.speedText, &dashboard.voltageText,
                     &dashboard.coolantText, &dashboard.oilText,
                     &dashboard.intakeText, &dashboard.gearText,
                     &dashboard.modeText, &dashboard.clutchText,
                     &dashboard.brakeText, &dashboard.accelText,
                     &dashboard.tpmsFlPsi, &dashboard.tpmsFrPsi,
                     &dashboard.tpmsRlPsi, &dashboard.tpmsRrPsi,
                     &dashboard.tpmsFlTemp, &dashboard.tpmsFrTemp,
                     &dashboard.tpmsRlTemp, &dashboard.tpmsRrTemp}) {
    std::snprintf(text->data(), text->size(), "--");
  }

  const uint8_t ids[] = {0x0c, 5, 0x0d, 0x0f, 0x42, 0x80};
  std::array<char, 8>* fields[] = {
      &dashboard.rpmText, &dashboard.coolantText, &dashboard.speedText,
      &dashboard.intakeText, &dashboard.voltageText, &dashboard.oilText};
  for (unsigned index = 0; index < 6; ++index) {
    if (!gauge.fresh(ids[index], now)) continue;
    std::snprintf(fields[index]->data(), fields[index]->size(),
                  ids[index] == 0x42 ? "%.1f" : "%.0f",
                  static_cast<double>(gauge.sample(ids[index]).value));
  }

  if (gauge.fresh(0x0c, now)) {
    const float rpm = std::min(8000.0f, std::max(0.0f, gauge.sample(0x0c).value));
    dashboard.rpmBarPixels = static_cast<uint16_t>(rpm * 480 / 8000);
  }
  if (gauge.pedalPid() && gauge.fresh(gauge.pedalPid(), now)) {
    const float rawPercent=gauge.sample(gauge.pedalPid()).value;
    const float displayed=kDriveUi ? std::clamp((rawPercent*2.55f-38.0f)*100.0f/175.0f,0.0f,100.0f) : rawPercent;
    std::snprintf(dashboard.accelText.data(), dashboard.accelText.size(), "%.0f",
                  static_cast<double>(displayed));
  }
  if (gauge.boostFresh(now)) {
    const float signedBoost = (gauge.sample(0x0b).value - gauge.sample(0x33).value) / 100.0f;
    const float boost=signedBoost + (kDriveUi ? kBoostDisplayOffsetBar : 0.0f);
    std::snprintf(dashboard.boostText.data(), dashboard.boostText.size(), "%.2f",
                  static_cast<double>(boost));
    dashboard.boostValid = true;
    dashboard.boostEndpointPixels = layout::boostPixels(boost);
  }
  if(kDriveUi && gauge.fresh(0x84,now) && observedReverse(gauge.sample(0x84).value))
    std::snprintf(dashboard.gearText.data(),dashboard.gearText.size(),"R");
  else {
    const auto gear=gauge.gearPresentation(now);
    frame.gearConfidence=gear.confidence;
    if(gear.gear)std::snprintf(dashboard.gearText.data(),dashboard.gearText.size(),"%u",gear.gear);
  }

  if (gauge.fresh(0x83, now)) {
    const float raw = gauge.sample(0x83).value;
    // Empirical reference states, not a calibrated pressure or a universal bit mask.
    const char* value = raw == 0 ? "OFF" : raw == 0x2d ? "ON" : "--";
    std::snprintf(dashboard.brakeText.data(), dashboard.brakeText.size(), "%s", value);
  }

  const TpmsSample& tpms = gauge.tpms();
  std::array<char, 8>* pressures[] = {
      &dashboard.tpmsFlPsi, &dashboard.tpmsFrPsi,
      &dashboard.tpmsRrPsi, &dashboard.tpmsRlPsi};
  std::array<char, 8>* temperatures[] = {
      &dashboard.tpmsFlTemp, &dashboard.tpmsFrTemp,
      &dashboard.tpmsRrTemp, &dashboard.tpmsRlTemp};
  for (uint8_t slot = 0; slot < 4; ++slot) {
    const bool visible = tpms.visible(slot, now, gauge.state() == State::Active);
    frame.reference.tpmsCandidateVisible[slot] = visible;
    if (!visible) continue;
    std::snprintf(pressures[slot]->data(), pressures[slot]->size(), "%.1f",
                  static_cast<double>(tpms.slots[slot].pressurePsi()));
    std::snprintf(temperatures[slot]->data(), temperatures[slot]->size(), "%d",
                  tpms.slots[slot].temperatureC());
  }
  return frame;
}

}  // namespace model
}  // namespace gaugev4
