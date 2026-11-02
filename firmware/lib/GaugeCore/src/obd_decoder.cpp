#include "obd_decoder.hpp"

namespace {

bool isMode01Response(const CanFrame& frame, uint8_t payloadBytes) {
  if (frame.id < 0x7E8 || frame.id > 0x7EF || frame.dlc > frame.data.size()) {
    return false;
  }

  const uint8_t requiredDlc = static_cast<uint8_t>(payloadBytes + 1);
  if (frame.dlc < requiredDlc || frame.data[0] < payloadBytes ||
      frame.data[0] > frame.dlc - 1) {
    return false;
  }

  return frame.data[1] == 0x41;
}

bool usesTwoDataBytes(uint8_t pid) {
  return pid == 0x0C || pid == 0x42;
}

}  // namespace

std::optional<DecodedSignal> decodeMode01(const CanFrame& frame) {
  if (!isMode01Response(frame, 3)) {
    return std::nullopt;
  }

  const uint8_t pid = frame.data[2];
  if (usesTwoDataBytes(pid) && !isMode01Response(frame, 4)) {
    return std::nullopt;
  }

  const uint8_t a = frame.data[3];
  const uint8_t b = frame.data[4];
  switch (pid) {
    case 0x0C:
      return DecodedSignal{SignalId::Rpm,
                           ((a * 256.0f) + b) / 4.0f};
    case 0x0D:
      return DecodedSignal{SignalId::Speed, static_cast<float>(a)};
    case 0x05:
      return DecodedSignal{SignalId::CoolantTemp,
                           static_cast<float>(a) - 40.0f};
    case 0x0F:
      return DecodedSignal{SignalId::IntakeTemp,
                           static_cast<float>(a) - 40.0f};
    case 0x0B:
      return DecodedSignal{SignalId::ManifoldPressureKpa,
                           static_cast<float>(a)};
    case 0x33:
      return DecodedSignal{SignalId::BarometricPressureKpa,
                           static_cast<float>(a)};
    case 0x5C:
      return DecodedSignal{SignalId::OilTemp,
                           static_cast<float>(a) - 40.0f};
    case 0x11:
    case 0x49:
      return DecodedSignal{SignalId::Accelerator, a * 100.0f / 255.0f};
    case 0x42:
      return DecodedSignal{SignalId::BatteryVoltage,
                           ((a * 256.0f) + b) / 1000.0f};
    default:
      return std::nullopt;
  }
}

std::optional<SupportedBitmap> decodeSupportedBitmap(const CanFrame& frame) {
  if (!isMode01Response(frame, 6)) {
    return std::nullopt;
  }

  const uint8_t basePid = frame.data[2];
  if (basePid != 0x00 && basePid != 0x20 && basePid != 0x40 &&
      basePid != 0x60) {
    return std::nullopt;
  }

  const uint32_t bitmap = (static_cast<uint32_t>(frame.data[3]) << 24) |
                          (static_cast<uint32_t>(frame.data[4]) << 16) |
                          (static_cast<uint32_t>(frame.data[5]) << 8) |
                          static_cast<uint32_t>(frame.data[6]);
  return SupportedBitmap{basePid, bitmap};
}
