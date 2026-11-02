#include "signal_store.hpp"

#include <app_config.hpp>

void SignalStore::update(SignalId id, float value, uint32_t nowMs) {
  values_[indexOf(id)] = {value, nowMs, true};
}

void SignalStore::invalidate(SignalId id) {
  values_[indexOf(id)].valid = false;
}

FreshValue SignalStore::read(SignalId id, uint32_t nowMs) const {
  const StoredValue& stored = values_[indexOf(id)];
  const uint32_t ageMs = nowMs - stored.updatedMs;
  if (!stored.valid || ageMs > ttlFor(id)) {
    return {false, 0.0f};
  }
  return {true, stored.value};
}

size_t SignalStore::indexOf(SignalId id) {
  return static_cast<size_t>(id);
}

uint32_t SignalStore::ttlFor(SignalId id) {
  switch (id) {
    case SignalId::Rpm:
    case SignalId::Speed:
    case SignalId::Accelerator:
    case SignalId::ManifoldPressureKpa:
    case SignalId::Gear:
    case SignalId::Clutch:
    case SignalId::Brake:
      return config::kFastStaleMs;

    case SignalId::TpmsFlPressure:
    case SignalId::TpmsFrPressure:
    case SignalId::TpmsRlPressure:
    case SignalId::TpmsRrPressure:
    case SignalId::TpmsFlTemp:
    case SignalId::TpmsFrTemp:
    case SignalId::TpmsRlTemp:
    case SignalId::TpmsRrTemp:
      return config::kTpmsStaleMs;

    default:
      return config::kSlowStaleMs;
  }
}
