#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

enum class SignalId : uint8_t {
  Rpm, Speed, CoolantTemp, OilTemp, IntakeTemp, Accelerator,
  BatteryVoltage, ManifoldPressureKpa, BarometricPressureKpa,
  Gear, DriveMode, Clutch, Brake,
  TpmsFlPressure, TpmsFrPressure, TpmsRlPressure, TpmsRrPressure,
  TpmsFlTemp, TpmsFrTemp, TpmsRlTemp, TpmsRrTemp,
  RevMatchStatus, EngineModeStatus, LsdModeStatus, ExhaustModeStatus,
  SuspensionModeStatus, SteeringModeStatus, VdcModeStatus,
  Count
};

struct FreshValue { bool valid; float value; };

class SignalStore {
 public:
  void update(SignalId id, float value, uint32_t nowMs);
  void invalidate(SignalId id);
  FreshValue read(SignalId id, uint32_t nowMs) const;

 private:
  struct StoredValue {
    float value = 0.0f;
    uint32_t updatedMs = 0;
    bool valid = false;
  };

  static constexpr size_t kSignalCount = static_cast<size_t>(SignalId::Count);
  static size_t indexOf(SignalId id);
  static uint32_t ttlFor(SignalId id);

  std::array<StoredValue, kSignalCount> values_{};
};
