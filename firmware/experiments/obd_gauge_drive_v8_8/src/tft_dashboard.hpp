#pragma once
#include "model.hpp"
#include <TFT_eSPI.h>
#include <safety_state.hpp>
#include "startup_sweep.hpp"

class TftDashboard final {
 public:
  bool begin();
  void startBootSweep();
  bool bootSweepActive() const;
  void render(const gaugev4::model::Frame& frame);
  void setCanStatus(CanPhase phase);
  void setBacklight(bool on);
  void setSleeping(bool asleep);
  void setTrialRate(unsigned hz);
 private:
  void text(const char* value,int x,int y,int font,uint16_t color,int width,uint16_t background=TFT_BLACK);
  void rpm(const DashboardFrame& frame);
  void wheel(int x,int y,const std::array<char,8>& psi,const std::array<char,8>& temp,bool valid,bool pressureChanged,bool temperatureChanged);
  void temperature(int y,const char* label,const std::array<char,8>& value);
  void pedal(const std::array<char,8>& value);
  void boost(const DashboardFrame& frame);
  void bottom(int x,const char* label,const char* value,const char* unit);
  void footer(const gaugev4::model::Frame& frame);
  void push(int x,int y,int w,int h);
  gaugev4::model::Frame nextSlice(const gaugev4::model::Frame& frame);
  TFT_eSPI panel_{};
  TFT_eSprite tft_{&panel_};
  gaugev4::model::Frame previous_{};
  CanPhase canPhase_{CanPhase::Boot};
  bool hasPrevious_{false},begun_{false},statusDirty_{true};
  unsigned trialRate_{0};
  unsigned slowCell_{0};
  gaugev4::startup::Sweep sweep_;
  bool drawingSweep_{false};
};
