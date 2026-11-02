#pragma once

#include "types.hpp"

#include <cstring>

namespace gaugev4 {
inline bool decodeTpms(const uint8_t* payload, unsigned length, uint32_t receivedAt,
                       TpmsSample& destination) {
  TpmsSample next{};
  next.length = length;
  next.receivedAt = receivedAt;
  const unsigned copy = length < sizeof(next.raw) ? length : sizeof(next.raw);
  if (payload && copy) std::memcpy(next.raw, payload, copy);
  for (auto& slot : next.slots) slot.exclusion = Exclusion::Schema;

  const uint8_t prefix[7] = {0x62, 0xc0, 0x0b, 0xff, 0xff, 0x00, 0x00};
  if (!payload || length != 23 || std::memcmp(payload, prefix, sizeof(prefix)) != 0) {
    destination = next;
    return false;
  }

  next.schemaAccepted = true;
  const unsigned offsets[4] = {7, 11, 15, 19};
  for (unsigned index = 0; index < 4; ++index) {
    TpmsSlot& slot = next.slots[index];
    const unsigned offset = offsets[index];
    slot.pressureRaw = payload[offset];
    slot.temperatureRaw = payload[offset + 1];
    slot.statusRaw = payload[offset + 2];
    slot.trailingRaw = payload[offset + 3];
    if (slot.statusRaw != 1 && slot.statusRaw != 2) slot.exclusion = Exclusion::Status;
    else if (slot.trailingRaw != 0) slot.exclusion = Exclusion::Trailing;
    else if (slot.pressureRaw == 0 || slot.pressureRaw == 0xff) slot.exclusion = Exclusion::Pressure;
    else if (slot.temperatureRaw == 0xff) slot.exclusion = Exclusion::Temperature;
    else slot.exclusion = Exclusion::None;
  }
  destination = next;
  return true;
}
}
