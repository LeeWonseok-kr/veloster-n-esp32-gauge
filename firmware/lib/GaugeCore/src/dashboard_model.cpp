#include "dashboard_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

using Text = std::array<char, 8>;

void setUnavailable(Text& text) {
  text[0] = '-';
  text[1] = '-';
  text[2] = '\0';
}

void formatInteger(Text& text, FreshValue value) {
  if (!value.valid) {
    setUnavailable(text);
    return;
  }
  std::snprintf(text.data(), text.size(), "%ld",
                static_cast<long>(std::lround(value.value)));
}

void formatOneDecimal(Text& text, FreshValue value) {
  if (!value.valid) {
    setUnavailable(text);
    return;
  }
  std::snprintf(text.data(), text.size(), "%.1f",
                static_cast<double>(value.value));
}

StatusCellState statusFrom(FreshValue value) {
  if (!value.valid) {
    return StatusCellState::Unknown;
  }
  return value.value == 0.0f ? StatusCellState::Inactive
                            : StatusCellState::Active;
}

uint16_t rpmPixels(FreshValue rpm) {
  if (!rpm.valid) {
    return 0;
  }
  const float clamped =
      std::clamp(rpm.value, 0.0f, static_cast<float>(layout::kRpmScaleMaximum));
  return static_cast<uint16_t>(
      clamped * static_cast<float>(layout::kScreenWidth) /
      static_cast<float>(layout::kRpmScaleMaximum));
}

}  // namespace

DashboardFrame DashboardModel::build(const SignalStore& store, uint32_t nowMs) {
  DashboardFrame frame;

  const FreshValue rpm = store.read(SignalId::Rpm, nowMs);
  formatInteger(frame.rpmText, rpm);
  frame.rpmBarPixels = rpmPixels(rpm);

  formatInteger(frame.speedText, store.read(SignalId::Speed, nowMs));
  formatOneDecimal(frame.voltageText,
                   store.read(SignalId::BatteryVoltage, nowMs));
  formatInteger(frame.coolantText,
                store.read(SignalId::CoolantTemp, nowMs));
  formatInteger(frame.oilText, store.read(SignalId::OilTemp, nowMs));
  formatInteger(frame.intakeText, store.read(SignalId::IntakeTemp, nowMs));
  formatInteger(frame.gearText, store.read(SignalId::Gear, nowMs));
  formatInteger(frame.modeText, store.read(SignalId::DriveMode, nowMs));
  formatInteger(frame.clutchText, store.read(SignalId::Clutch, nowMs));
  formatInteger(frame.brakeText, store.read(SignalId::Brake, nowMs));
  formatInteger(frame.accelText, store.read(SignalId::Accelerator, nowMs));

  formatInteger(frame.tpmsFlPsi,
                store.read(SignalId::TpmsFlPressure, nowMs));
  formatInteger(frame.tpmsFrPsi,
                store.read(SignalId::TpmsFrPressure, nowMs));
  formatInteger(frame.tpmsRlPsi,
                store.read(SignalId::TpmsRlPressure, nowMs));
  formatInteger(frame.tpmsRrPsi,
                store.read(SignalId::TpmsRrPressure, nowMs));
  formatInteger(frame.tpmsFlTemp, store.read(SignalId::TpmsFlTemp, nowMs));
  formatInteger(frame.tpmsFrTemp, store.read(SignalId::TpmsFrTemp, nowMs));
  formatInteger(frame.tpmsRlTemp, store.read(SignalId::TpmsRlTemp, nowMs));
  formatInteger(frame.tpmsRrTemp, store.read(SignalId::TpmsRrTemp, nowMs));

  const FreshValue map = store.read(SignalId::ManifoldPressureKpa, nowMs);
  const FreshValue baro = store.read(SignalId::BarometricPressureKpa, nowMs);
  if (map.valid && baro.valid) {
    float boostBar = (map.value - baro.value) / 100.0f;
    if (std::fabs(boostBar) < 0.0005f) {
      boostBar = 0.0f;
    }
    frame.boostValid = true;
    std::snprintf(frame.boostText.data(), frame.boostText.size(), "%+.1f",
                  static_cast<double>(boostBar));
    frame.boostEndpointPixels = layout::boostPixels(boostBar);
  } else {
    setUnavailable(frame.boostText);
  }

  constexpr std::array<SignalId, 7> kStatusSignals = {
      SignalId::RevMatchStatus,       SignalId::EngineModeStatus,
      SignalId::LsdModeStatus,        SignalId::ExhaustModeStatus,
      SignalId::SuspensionModeStatus, SignalId::SteeringModeStatus,
      SignalId::VdcModeStatus};
  for (size_t index = 0; index < kStatusSignals.size(); ++index) {
    frame.nStatus[index] = statusFrom(store.read(kStatusSignals[index], nowMs));
  }

  return frame;
}
