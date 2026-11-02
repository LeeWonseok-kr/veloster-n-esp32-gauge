#include <Arduino.h>
#include <esp_system.h>

#include "../../obd_gauge_drive_v8_8/src/hardware_port.hpp"
#include "../../obd_gauge_drive_v8_8/src/model.hpp"
#include "../../obd_gauge_drive_v8_8/src/runtime.hpp"
#include "../../obd_gauge_drive_v8_8/src/tft_dashboard.hpp"

#include "../../obd_gauge/src/reset_policy.hpp"
#include "../../obd_probe/src/command_line.hpp"

HardwarePort port;
uint32_t gaugeClock() { return millis(); }
gaugev4::Gauge gauge(port, gaugeClock);
TftDashboard display;
obdprobe::CommandLine commandLine;
gaugev4::BoundedLogger logger;
gaugev4::CollectLog collectLog;
gaugev4::MarkerQueue markers;

RTC_NOINIT_ATTR uint32_t faultMarker;
constexpr uint32_t kFaultMagic = 0x47433631;
uint32_t lastUi = 0;
uint32_t lastStatus = 0;
uint32_t activeUiMaxUs = 0;
uint32_t terminalUiMaxUs = 0;
bool terminalReported = false;

gaugev4::PortDiagnostics portDiagnostics() {
  gaugev4::PortDiagnostics result{};
  result.errors = port.errors;
  result.failed = port.failed;
  result.missed = port.missed;
  result.overruns = port.overruns;
  result.txErrors = port.txErrors;
  result.rxErrors = port.rxErrors;
  return result;
}

void setup() {
  HardwarePort::quietTx();
  digitalWrite(pins::kTftBacklight,LOW);pinMode(pins::kTftBacklight,OUTPUT);
  pinMode(pins::kCanStandby, OUTPUT);
  digitalWrite(pins::kCanStandby, LOW);
  // Covers the bounded worst-case terminal snapshot without a fixed-order tail loss.
  Serial.setTxBufferSize(4096);
  Serial.begin(115200);

  const esp_reset_reason_t reset = esp_reset_reason();
  bool locked = obdgauge::bootLocked(faultMarker == kFaultMagic,
                                     reset == ESP_RST_POWERON,
                                     reset == ESP_RST_EXT);
  if (!display.begin()) locked = true;

  faultMarker = locked ? kFaultMagic : 0;
  gauge.begin(millis(), locked);
  if(!locked)display.startBootSweep();
  display.render(gaugev4::model::buildFrame(gauge, millis()));
  display.setBacklight(true);
  Serial.printf("DISPLAY_GAUGE shift_rpm=%u oil_pressure=UNAVAILABLE\n",
                static_cast<unsigned>(GAUGE_SHIFT_RPM));
  Serial.printf("OBD_DISPLAY reset=%u lockout=%u ALL_CANDIDATES_UNVERIFIED GPIO13_CONTROL NO_AUTO_SLEEP\n",
                static_cast<unsigned>(reset), locked ? 1u : 0u);
  Serial.println(gaugev4::kPairContinuous
      ? "NO_SLEEP_CEREMONY build=20261102_r9_nosleep AUTO_START_3S rate=20 CONTINUOUS NO_AUTO_RETRY STOP_LATCHES_OFF"
      : gaugev4::kPairTrial
      ? "DRIVE_V8_8_PAIR_R7 MANUAL START PAIR DRIVE 20 ONE_60S_TRIAL NO_AUTO_RETRY"
      : gaugev4::kPairProbe
      ? "DRIVE_V8_8_PAIR MANUAL START PAIR 10 ONE_GROUPED_READ NO_AUTO_RETRY"
      : gaugev4::kFastTrial
      ? "DRIVE_V8_8_R7_20HZ RPM_SPEED_PRIORITY MANUAL START COLLECT 20 ONE_60S_TRIAL NO_AUTO_RETRY"
      : gaugev4::kDriveUi
      ? "DRIVE_V8_8 AUTO_START_3S rate=10 REL_PRESSURE_OFFSET_0.60 GEAR_EST EARLY_CANDIDATE BOUNDED_HOLD NO_AUTO_RETRY"
      : gaugev4::kContinuous
      ? "CONTINUOUS_V7 AUTO_START_3S rate=10 NO_AUTO_RETRY STOP_LATCHES_OFF"
      : "MANUAL_V6 START COLLECT 10: one 60-second trial per boot; STOP ends CAN");
}

void loop() {
  gauge.tick(millis());

  // USB input, output, and TFT work are excluded from every pending transaction.
  if (!gauge.pending()) {
    for (unsigned index = 0; index < 128 && Serial.available(); ++index) {
      if (commandLine.feed(static_cast<char>(Serial.read())) && commandLine.valid()) {
        const char* text = commandLine.text();
        if (std::strncmp(text,"MARK ",5)==0)
          markers.command(text,millis(),gauge.state()==gaugev4::State::Active);
        else
          gauge.command(text, millis());
      }
    }
  }

  gauge.tick(millis());
  if (gaugev4::terminalState(gauge.state())) faultMarker = kFaultMagic;

  const uint32_t now = millis();
  const gaugev4::OutputPlan plan =
      gaugev4::planOutputs(gauge, now, lastUi, lastStatus, terminalReported);
  if (!gauge.pending()) {
    collectLog.observation(Serial,gauge.observation(),false);
    collectLog.markers(Serial,markers,false);
    collectLog.bus(Serial,port.busJournal,false);
    // Record each accepted RPM/speed response before TFT work. No extra CAN TX.
    logger.gearInputs(Serial, gauge);
    if (plan.render) {
      lastUi = now;
      const uint32_t elapsed = gaugev4::serviceDisplay(
          display, gaugev4::model::buildFrame(gauge, now), gauge.rateHz(),
          plan.phase, micros);
      gaugev4::noteUiDuration(gauge.state(), elapsed,
                              activeUiMaxUs, terminalUiMaxUs);
    }
    if (plan.status || plan.terminal) {
      if (plan.status) lastStatus = now;
      if (plan.terminal) terminalReported = true;
      gaugev4::emitPlannedLogs(
          plan, logger, Serial, gauge, gaugev4::model::buildFrame(gauge, now),
          now, portDiagnostics(), port.candidates,
          activeUiMaxUs, terminalUiMaxUs);
      collectLog.metrics(Serial,port.busJournal,markers,port.trialStartedAt());
    }
    gauge.uiServiced();
  }
  delay(1);
}
