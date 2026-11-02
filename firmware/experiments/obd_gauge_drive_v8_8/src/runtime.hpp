#pragma once

#include "model.hpp"

#include <safety_state.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace gaugev4 {

struct CandidateRecord {
  Frame frame{};
  uint32_t at = 0;
  uint32_t count = 0;
};

class CandidateSightings {
 public:
  static constexpr unsigned capacity = 8;

  static uint32_t idAt(unsigned index) {
    switch (index) {
      case 0: return 0x492;
      case 1: return 0x329;
      case 2: return 0x541;
      case 3: return 0x4a0;
      case 4: return 0x50e;
      case 5: return 0x50c;
      case 6: return 0x593;
      case 7: return 0x501;
      default: return 0;
    }
  }

  void observe(const Frame& frame, uint32_t at) {
    if (frame.extended || frame.rtr || frame.dlc > 8) return;
    for (unsigned index = 0; index < capacity; ++index) {
      if (frame.id != idAt(index)) continue;
      CandidateRecord& destination = records_[index];
      destination.frame = frame;
      destination.at = at;
      ++destination.count;
      return;
    }
  }

  const CandidateRecord* record(unsigned index) const {
    return index < capacity ? &records_[index] : nullptr;
  }

 private:
  CandidateRecord records_[capacity]{};
};

struct PortDiagnostics {
  uint32_t errors = 0;
  uint32_t failed = 0;
  uint32_t missed = 0;
  uint32_t overruns = 0;
  uint32_t txErrors = 0;
  uint32_t rxErrors = 0;
};

struct OutputPlan {
  bool render = false;
  bool status = false;
  bool terminal = false;
  CanPhase phase = CanPhase::Boot;
};

inline bool terminalState(State state) {
  return state == State::Stopped || state == State::Fault || state == State::Lockout;
}

inline OutputPlan planOutputs(const Gauge& gauge, uint32_t now,
                              uint32_t lastUi, uint32_t lastStatus,
                              bool terminalReported) {
  OutputPlan result{};
  if (gauge.state() == State::Active) {
    result.phase = std::strcmp(gauge.reason(), "DISCOVER") == 0
                       ? CanPhase::ListenOnly : CanPhase::Active;
  } else if (terminalState(gauge.state())) {
    result.phase = CanPhase::Offline;
  }
  if (gauge.pending()) return result;
  result.render = now - lastUi >= 50;
  result.status = now - lastStatus >= 1000;
  result.terminal = terminalState(gauge.state()) && !terminalReported;
  return result;
}

template <typename Display, typename Clock>
inline uint32_t serviceDisplay(Display& display, const model::Frame& frame,
                               unsigned rate, CanPhase phase, Clock clock) {
  const uint32_t started = clock();
  display.setTrialRate(rate);
  display.setCanStatus(phase);
  display.render(frame);
  return clock() - started;
}

inline void noteUiDuration(State state, uint32_t elapsed,
                           uint32_t& activeMaximum,
                           uint32_t& terminalMaximum) {
  if (state == State::Active) {
    if (elapsed > activeMaximum) activeMaximum = elapsed;
  } else if (terminalState(state) && elapsed > terminalMaximum) {
    terminalMaximum = elapsed;
  }
}

class BoundedLogger {
 public:
  uint32_t dropped() const { return dropped_; }

  // Called between transactions, before another request can replace a sample.
  // Records identify accepted samples. A shared grouped response does not
  // establish simultaneous sensor acquisition inside the ECU.
  template <typename Sink>
  bool gearInputs(Sink& sink, const Gauge& gauge) {
    if (gauge.pending() || gauge.state() != State::Active) return true;
    const uint8_t pids[2] = {0x0c, 0x0d};
    bool complete = true;
    for (unsigned index=0; index<2; ++index) {
      const Sample sample = gauge.sample(pids[index]);
      if (!sample.present || !sample.requestOrdinal ||
          sample.requestOrdinal == gearInputSeen_[index]) continue;
      // Consume even on a full USB buffer; never stall CAN or replay old data.
      gearInputSeen_[index] = sample.requestOrdinal;
      char line[192];
      const int length = std::snprintf(line,sizeof(line),
          "GEAR_INPUT pid=%02X seq=%lu value=%.2f request_at_ms=%lu received_at_ms=%lu rtt_ms=%lu gear=UNKNOWN\n",
          static_cast<unsigned>(pids[index]),
          static_cast<unsigned long>(sample.requestOrdinal),
          static_cast<double>(sample.value),
          static_cast<unsigned long>(sample.requestAt),
          static_cast<unsigned long>(sample.at),
          static_cast<unsigned long>(sample.at-sample.requestAt));
      complete = writeLine(sink,line,
          length>0 && static_cast<size_t>(length)<sizeof(line) ? length : -1) && complete;
    }
    return complete;
  }

  template <typename Sink>
  bool status(Sink& sink, const Gauge& gauge, const model::Frame& frame,
              uint32_t now, const PortDiagnostics& port,
              uint32_t activeUiMaxUs, uint32_t terminalUiMaxUs) {
    bool complete = true;
    char line[768];
    int length = std::snprintf(
        line, sizeof(line),
        "STATUS v=%u state=%s req=%lu resp=%lu fc=%lu rpm=%s coolant=%s speed=%s intake=%s ecu_v=%s boost=%s oil_ref=%s oil_verified=0 pedal=%s errors=%lu failed=%lu missed=%lu overrun=%lu tx_err=%lu rx_err=%lu"
#if GAUGE_NO_SLEEP_CEREMONY
        " variant=%s pressure_disabled=1 brake_ref=%s brake_verified=0 build=20261102_r9_nosleep\n",
#elif GAUGE_SLEEP_VARIANT
        " variant=%s pressure_disabled=1 brake_ref=%s brake_verified=0 build=20261027_r8_sleep\n",
#else
        " variant=%s pressure_disabled=1 brake_ref=%s brake_verified=0 build=20261023_r7\n",
#endif
        kDriveUi ? 8u : kContinuous ? 7u : 6u, gauge.reason(), static_cast<unsigned long>(gauge.requests()),
        static_cast<unsigned long>(gauge.responses()),
        static_cast<unsigned long>(gauge.flowControls()),
        frame.dashboard.rpmText.data(), frame.dashboard.coolantText.data(),
        frame.dashboard.speedText.data(), frame.dashboard.intakeText.data(),
        frame.dashboard.voltageText.data(), frame.dashboard.boostText.data(),
        frame.dashboard.oilText.data(), frame.dashboard.accelText.data(),
        static_cast<unsigned long>(port.errors), static_cast<unsigned long>(port.failed),
        static_cast<unsigned long>(port.missed), static_cast<unsigned long>(port.overruns),
        static_cast<unsigned long>(port.txErrors), static_cast<unsigned long>(port.rxErrors),
        kDriveUi ? (kPairContinuous?"drive_v8_8_pair_20hz_continuous":kPairTrial?"drive_v8_8_pair_20hz_trial":kPairProbe?"drive_v8_8_pair_probe":kFastTrial?"drive_v8_8_20hz_trial":"drive_v8_8_10hz") : kContinuous ? "continuous_10hz" : "brake_display_2hz", frame.dashboard.brakeText.data());
    complete = writeLine(sink, line, length) && complete;

    if(kDriveUi) {
      char pedalRaw[16]="--",pedalByte[16]="--",signedBoost[16]="--";
      if(gauge.fresh(0x49,now)) {
        std::snprintf(pedalRaw,sizeof(pedalRaw),"%.3f",static_cast<double>(gauge.sample(0x49).value));
        std::snprintf(pedalByte,sizeof(pedalByte),"%.0f",static_cast<double>(gauge.sample(0x49).value*2.55f));
      }
      if(gauge.boostFresh(now))std::snprintf(signedBoost,sizeof(signedBoost),"%.3f",static_cast<double>((gauge.sample(0x0b).value-gauge.sample(0x33).value)/100.0f));
      length=std::snprintf(line,sizeof(line),"DRIVE gear=%s gear_source=%s gear_verified=0 pedal_raw_pct=%s pedal_endpoints=38:213 boost_signed_bar=%s reverse_age_ms=%lu pedal_raw=%s pedal_display_pct=%s boost_display_bar=%s boost_offset_bar=%.3f boost_kind=REL gear_reason=%s gear_display_state=%s\n",
        frame.dashboard.gearText.data(),frame.dashboard.gearText[0]=='R'?"REF":"EST",pedalRaw,signedBoost,
        static_cast<unsigned long>(gauge.sample(0x84).present?now-gauge.sample(0x84).at:UINT32_MAX),
        pedalByte,frame.dashboard.accelText.data(),frame.dashboard.boostText.data(),
        static_cast<double>(model::kBoostDisplayOffsetBar),gauge.gearReason(now),
        frame.dashboard.gearText[0]=='R'?"REVERSE_REF":gearConfidenceName(frame.gearConfidence));
      complete=writeLine(sink,line,length)&&complete;
    }

    const Counters& standard = gauge.counters(Profile::Standard);
    const Counters& oil = gauge.counters(Profile::Oil);
    const Counters& tpms = gauge.counters(Profile::Tpms);
    length = std::snprintf(
        line, sizeof(line),
        "PROFILE std_req=%lu std_resp=%lu std_fc=%lu oil_req=%lu oil_resp=%lu oil_fc=%lu tpms_req=%lu tpms_resp=%lu tpms_fc=%lu\n",
        static_cast<unsigned long>(standard.requests), static_cast<unsigned long>(standard.responses),
        static_cast<unsigned long>(standard.flowControls), static_cast<unsigned long>(oil.requests),
        static_cast<unsigned long>(oil.responses), static_cast<unsigned long>(oil.flowControls),
        static_cast<unsigned long>(tpms.requests), static_cast<unsigned long>(tpms.responses),
        static_cast<unsigned long>(tpms.flowControls));
    complete = writeLine(sink, line, length) && complete;

    length = std::snprintf(
        line, sizeof(line),
        "METRICS rate_cap_hz=%u pedal_pid=%02X rtt_mean_ms=%lu rtt_max_ms=%lu completion_gap_max_ms=%lu active_ui_max_us=%lu terminal_ui_max_us=%lu dropped=%lu\n",
        gauge.rateHz(), gauge.pedalPid(), static_cast<unsigned long>(gauge.meanRttMs()),
        static_cast<unsigned long>(gauge.maxRttMs()),
        static_cast<unsigned long>(gauge.maxCompletionGapMs()),
        static_cast<unsigned long>(activeUiMaxUs), static_cast<unsigned long>(terminalUiMaxUs),
        static_cast<unsigned long>(dropped_));
    complete = writeLine(sink, line, length) && complete;

    const TpmsSample& sample = gauge.tpms();
    const uint32_t age = sample.length ? now - sample.receivedAt : UINT32_MAX;
    length = std::snprintf(
        line, sizeof(line),
        "TPMS schema=%u length=%u received_at_ms=%lu ecu_receive_age_ms=%lu active=%u visible=%u%u%u%u slot_order=A/B/C/D wheel_candidates=FL?/FR?/RR?/RL? mapping_verified=0 conversion_verified=0 sensor_age=UNKNOWN status_semantics=UNKNOWN tpms_raw=",
        sample.schemaAccepted ? 1u : 0u, sample.length,
        static_cast<unsigned long>(sample.receivedAt), static_cast<unsigned long>(age),
        gauge.state() == State::Active ? 1u : 0u,
        frame.reference.tpmsCandidateVisible[0] ? 1u : 0u,
        frame.reference.tpmsCandidateVisible[1] ? 1u : 0u,
        frame.reference.tpmsCandidateVisible[2] ? 1u : 0u,
        frame.reference.tpmsCandidateVisible[3] ? 1u : 0u);
    if (length < 0 || static_cast<size_t>(length) >= sizeof(line)) {
      complete = rejectLine() && complete;
    } else {
      size_t used = static_cast<size_t>(length);
      appendHex(line, sizeof(line), used, sample.raw, sample.length);
      if (used + 1 < sizeof(line)) { line[used++]='\n'; line[used]='\0'; }
      else { used=sizeof(line); }
      complete = writeLine(sink, line, used < sizeof(line) ? static_cast<int>(used) : -1) && complete;
    }

    for (unsigned emitted = 0; emitted < 2; ++emitted) {
      const uint8_t key = timingKeyAt(timingCursor_);
      timingCursor_ = static_cast<uint8_t>((timingCursor_ + 1) % timingKeyCount());
      complete = writeTimingRecord(sink, key, gauge.timing(key)) && complete;
    }
    const uint8_t slot = slotCursor_;
    slotCursor_ = static_cast<uint8_t>((slotCursor_ + 1) % 4);
    complete = writeSlotRecord(sink, slot, sample, frame, now) && complete;
    return complete;
  }

  template <typename Sink>
  bool terminal(Sink& sink, const Gauge& gauge, const model::Frame& frame,
                uint32_t now, const PortDiagnostics& port,
                const CandidateSightings& candidates,
                uint32_t activeUiMaxUs, uint32_t terminalUiMaxUs) {
    bool complete = status(sink, gauge, frame, now, port, activeUiMaxUs, terminalUiMaxUs);
    char line[192];
    int length = std::snprintf(
        line, sizeof(line), "TERMINAL_SNAPSHOT state=%s at_ms=%lu raw_retained=1 values_hidden=1\n",
        gauge.reason(), static_cast<unsigned long>(now));
    complete = writeLine(sink, line, length) && complete;
    for (uint8_t slot = 0; slot < 4; ++slot)
      complete = writeSlotRecord(sink, slot, gauge.tpms(), frame, now) && complete;
    complete = writeLiteral(sink,"CANDIDATE_SIGHTINGS verified=0 snapshot=1 no_extra_requests=1\n") && complete;
    for (unsigned index=0; index<CandidateSightings::capacity; ++index) {
      const CandidateRecord* record=candidates.record(index);
      if (!record || !record->count) continue;
      length=std::snprintf(line,sizeof(line),"CANDIDATE_RAW id=%03lX count=%lu at_ms=%lu data=",
          static_cast<unsigned long>(CandidateSightings::idAt(index)),
          static_cast<unsigned long>(record->count),static_cast<unsigned long>(record->at));
      if (length<0 || static_cast<size_t>(length)>=sizeof(line)) {
        complete=rejectLine() && complete; continue;
      }
      size_t used=static_cast<size_t>(length);
      appendHex(line,sizeof(line),used,record->frame.data,record->frame.dlc);
      if(used+1<sizeof(line)) {line[used++]='\n';line[used]='\0';}
      else used=sizeof(line);
      complete=writeLine(sink,line,used<sizeof(line)?static_cast<int>(used):-1) && complete;
    }
    complete=writeLiteral(sink,"CANDIDATE_SIGHTINGS_END\n") && complete;
    return complete;
  }

 private:
  static unsigned timingKeyCount() { return 14; }

  static uint8_t timingKeyAt(unsigned index) {
    switch (index % timingKeyCount()) {
      case 0: return 0x00;
      case 1: return 0x20;
      case 2: return 0x40;
      case 3: return 0x0c;
      case 4: return 0x05;
      case 5: return 0x0d;
      case 6: return 0x0f;
      case 7: return 0x42;
      case 8: return 0x0b;
      case 9: return 0x33;
      case 10: return 0x49;
      case 11: return 0x4a;
      case 12: return 0x80;
      default: return 0x81;
    }
  }

  static const char* exclusionName(Exclusion exclusion) {
    switch (exclusion) {
      case Exclusion::None: return "NONE";
      case Exclusion::Schema: return "SCHEMA";
      case Exclusion::Pressure: return "PRESSURE";
      case Exclusion::Temperature: return "TEMPERATURE";
      case Exclusion::Status: return "STATUS";
      case Exclusion::Trailing: return "TRAILING";
    }
    return "UNKNOWN";
  }

  template <typename Sink>
  bool writeTimingRecord(Sink& sink, uint8_t key, const Timing& timing) {
    char line[192];
    const int length = std::snprintf(
        line, sizeof(line),
        "TIMING key=%02X count=%lu mean_rtt_ms=%lu max_rtt_ms=%lu max_completion_gap_ms=%lu last_completion_at_ms=%lu\n",
        static_cast<unsigned>(key), static_cast<unsigned long>(timing.count),
        static_cast<unsigned long>(timing.meanRttMs),
        static_cast<unsigned long>(timing.maxRttMs),
        static_cast<unsigned long>(timing.maxCompletionGapMs),
        static_cast<unsigned long>(timing.lastCompletionAt));
    return writeLine(sink, line, length);
  }

  template <typename Sink>
  bool writeSlotRecord(Sink& sink, uint8_t slot, const TpmsSample& sample,
                       const model::Frame& frame, uint32_t now) {
    static const char* wheels[4] = {"FL?", "FR?", "RR?", "RL?"};
    const TpmsSlot& value = sample.slots[slot];
    const uint32_t age = sample.length ? now - sample.receivedAt : UINT32_MAX;
    char line[256];
    const int length = std::snprintf(
        line, sizeof(line),
        "TPMS_SLOT slot=%c wheel_candidate=%s pressure_raw=%02X temperature_raw=%02X status_raw=%02X trailing_raw=%02X exclusion=%s ecu_receive_age_ms=%lu visible=%u mapping_verified=0 conversion_verified=0\n",
        static_cast<char>('A' + slot), wheels[slot],
        static_cast<unsigned>(value.pressureRaw),
        static_cast<unsigned>(value.temperatureRaw),
        static_cast<unsigned>(value.statusRaw),
        static_cast<unsigned>(value.trailingRaw), exclusionName(value.exclusion),
        static_cast<unsigned long>(age),
        frame.reference.tpmsCandidateVisible[slot] ? 1u : 0u);
    return writeLine(sink, line, length);
  }

  bool rejectLine() { ++dropped_; return false; }

  static void appendHex(char* destination, size_t capacity, size_t& used,
                        const uint8_t* bytes, unsigned length) {
    static const char digits[]="0123456789ABCDEF";
    for(unsigned index=0;index<length;++index) {
      if(used+2>=capacity) {used=capacity;return;}
      destination[used++]=digits[bytes[index]>>4];
      destination[used++]=digits[bytes[index]&15];
    }
    if(used<capacity) destination[used]='\0';
  }

  template <typename Sink>
  bool writeLiteral(Sink& sink,const char* text) {
    return writeLine(sink,text,static_cast<int>(std::strlen(text)));
  }

  template <typename Sink>
  bool writeLine(Sink& sink,const char* text,int length) {
    if(length<=0 || sink.availableForWrite()<length) return rejectLine();
    if(sink.write(reinterpret_cast<const uint8_t*>(text),static_cast<size_t>(length)) !=
       static_cast<size_t>(length)) return rejectLine();
    return true;
  }

  uint32_t dropped_ = 0;
  uint32_t gearInputSeen_[2]{};
  uint8_t timingCursor_ = 0;
  uint8_t slotCursor_ = 0;
};

template <typename Sink>
inline bool emitPlannedLogs(const OutputPlan& plan, BoundedLogger& logger,
                            Sink& sink, const Gauge& gauge,
                            const model::Frame& frame, uint32_t now,
                            const PortDiagnostics& port,
                            const CandidateSightings& candidates,
                            uint32_t activeUiMaxUs,
                            uint32_t terminalUiMaxUs) {
  if (gauge.pending()) return true;
  bool complete = logger.gearInputs(sink,gauge);
  if (plan.terminal)
    complete = logger.terminal(sink, gauge, frame, now, port, candidates,
                               activeUiMaxUs, terminalUiMaxUs) && complete;
  else if (plan.status)
    complete = logger.status(sink, gauge, frame, now, port,
                             activeUiMaxUs, terminalUiMaxUs) && complete;
  return complete;
}

}  // namespace gaugev4
