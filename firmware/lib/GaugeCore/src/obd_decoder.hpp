#pragma once

#include <cstdint>
#include <optional>

#include "can_frame.hpp"
#include "signal_store.hpp"

struct DecodedSignal {
  SignalId id;
  float value;
};

struct SupportedBitmap {
  uint8_t basePid;
  uint32_t bitmap;
};

std::optional<DecodedSignal> decodeMode01(const CanFrame& frame);
std::optional<SupportedBitmap> decodeSupportedBitmap(const CanFrame& frame);
