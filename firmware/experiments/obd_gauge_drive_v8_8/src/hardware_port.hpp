#pragma once

#include <Arduino.h>
#include <board_pins.hpp>
#include <driver/gpio.h>
#include <driver/twai.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>

#include "gauge.hpp"
#include "runtime.hpp"
#include "collect_log.hpp"

class HardwarePort final : public gaugev4::Port {
 public:
  bool selectRate(unsigned hz) override {
    return !openedOnce_ && guard_.selectRate(hz);
  }

  bool open() override {
    if (openedOnce_ || guard_.rateHz() == 0) return false;
    openedOnce_ = true;
    quietTx();
    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
        static_cast<gpio_num_t>(pins::kCanTx),
        static_cast<gpio_num_t>(pins::kCanRx), TWAI_MODE_NORMAL);
    general.tx_queue_len = 0;
    general.rx_queue_len = 1024;
    const twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
    const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&general, &timing, &filter) != ESP_OK) {
      quietTx();
      return false;
    }
    installed_ = true;
    if (twai_start() != ESP_OK) {
      stop();
      return false;
    }
    running_ = true;
    const uint32_t epoch = millis();
    if (!guard_.beginWindow(epoch)) {
      stop();
      return false;
    }
    trialStartedAt_ = epoch;
    return true;
  }

  void stop() override {
    endTransaction();
    quietTx();
    if (installed_) {
      if (running_) twai_stop();
      twai_driver_uninstall();
    }
    installed_ = false;
    running_ = false;
    quietTx();
  }

  bool good() override {
    twai_status_info_t status{};
    if (!running_ || twai_get_status_info(&status) != ESP_OK) return false;
    errors = status.bus_error_count;
    failed = status.tx_failed_count;
    missed = status.rx_missed_count;
    overruns = status.rx_overrun_count;
    txErrors = status.tx_error_counter;
    rxErrors = status.rx_error_counter;
    return status.state == TWAI_STATE_RUNNING && errors == 0 && failed == 0 &&
           missed == 0 && overruns == 0 && txErrors == 0 && rxErrors == 0;
  }

  bool receive(gaugev4::Frame& frame) override {
    twai_message_t message{};
    if (!running_ || twai_receive(&message, 0) != ESP_OK) return false;
    frame = {};
    frame.id = message.identifier;
    frame.dlc = message.data_length_code;
    frame.extended = message.extd;
    frame.rtr = message.rtr;
    for (unsigned index = 0; index < 8; ++index) frame.data[index] = message.data[index];
    candidates.observe(frame, millis());
    busJournal.observe(frame, millis());
    return true;
  }

  bool send(const gaugev4::Frame& frame) override {
    if (!good()) return false;
    const uint32_t now = millis();
    gaugev4::Profile profile = gaugev4::Profile::Standard;
    if (gaugev4::requestProfile(frame, profile)) {
      const uint32_t ordinal = guard_.requestCount() + 1;
      if (!guard_.allow(frame, ordinal, now)) return false;
      currentTransactionId_ = ordinal;
      currentProfile_ = profile;
    } else {
      if (!currentTransactionId_ ||
          !gaugev4::allowedFlow(frame, currentProfile_) ||
          !guard_.allow(frame, currentTransactionId_, now)) return false;
    }

    twai_message_t message{};
    message.identifier = frame.id;
    message.data_length_code = frame.dlc;
    message.extd = frame.extended;
    message.rtr = frame.rtr;
    message.ss = 1;
    for (unsigned index = 0; index < 8; ++index) message.data[index] = frame.data[index];
    // TxGuard has consumed the attempt before the one physical single-shot call.
    return twai_transmit(&message, 0) == ESP_OK;
  }

  uint32_t trialStartedAt() const override { return trialStartedAt_; }

  bool permitCts(uint32_t transactionId, uint32_t) override {
    if (!currentTransactionId_ || transactionId != currentTransactionId_) return false;
    return guard_.permitCts(transactionId, millis());
  }

  void endTransaction() override {
    guard_.endTransaction();
    currentTransactionId_ = 0;
    currentProfile_ = gaugev4::Profile::Standard;
  }

  uint32_t guardRequestCount() const { return guard_.requestCount(); }
  uint32_t guardFlowControlCount() const { return guard_.flowControlCount(); }

  static void quietTx() {
    // The installed external 10 kOhm TXD pull-up holds the transceiver recessive.
    gpio_set_direction(static_cast<gpio_num_t>(pins::kCanTx), GPIO_MODE_INPUT);
    gpio_set_level(static_cast<gpio_num_t>(pins::kCanTx), 1);
    esp_rom_gpio_connect_out_signal(pins::kCanTx, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction(static_cast<gpio_num_t>(pins::kCanTx), GPIO_MODE_INPUT);
  }

  uint32_t errors = 0;
  uint32_t failed = 0;
  uint32_t missed = 0;
  uint32_t overruns = 0;
  uint32_t txErrors = 0;
  uint32_t rxErrors = 0;
  gaugev4::CandidateSightings candidates{};
  gaugev4::BusJournal busJournal{};

 private:
  gaugev4::TxGuard guard_{};
  gaugev4::Profile currentProfile_ = gaugev4::Profile::Standard;
  uint32_t currentTransactionId_ = 0;
  uint32_t trialStartedAt_ = 0;
  bool installed_ = false;
  bool running_ = false;
  bool openedOnce_ = false;
};
