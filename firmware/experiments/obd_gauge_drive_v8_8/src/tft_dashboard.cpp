#include "tft_dashboard.hpp"
#include "hal/tft_init_gate.hpp"
#include <algorithm>
#include <cstdlib>
#include <cstring>
#if GAUGE_LED_CONTROL
#include <Arduino.h>
#include <board_pins.hpp>
#endif

namespace {
constexpr uint16_t gray=0x7bef,dim=0x4208,orange=0xfd20;
bool missing(const std::array<char,8>& v) {return v[0]=='-' && v[1]=='-';}
}

bool TftDashboard::begin() {
  begun_=false;hasPrevious_=false;statusDirty_=true;slowCell_=0;
#if GAUGE_PCB_REVA
  setBacklight(false);
  digitalWrite(7,HIGH);pinMode(7,OUTPUT);delay(20);
#endif
  panel_.init();panel_.setRotation(1);
  if(!tft_init::hasExpectedGeometry(panel_.width(),panel_.height()))return false;
  // Compose erase + redraw in RAM; send only completed dirty rectangles.
  // 8-bit canvas uses ~154 KB without changing PSRAM/pin/clock configuration.
  tft_.setColorDepth(8);
  if(!tft_.createSprite(480,320))return false; // setup locks CAN on allocation failure.
  tft_.setTextWrap(false,false);tft_.fillScreen(TFT_BLACK);begun_=true;
  for(int y:{71,255,302})tft_.drawFastHLine(0,y,480,dim);
  for(int x:{154,302,396,438})tft_.drawFastVLine(x,72,183,dim);
  for(int y:{132,193})tft_.drawFastHLine(303,y,93,dim);
  for(int x:{160,320})tft_.drawFastVLine(x,256,46,dim);
  text("RPM x1000",48,1,1,gray,94);
  for(int i=1;i<=8;++i){char label[2]={static_cast<char>('0'+i),0};
    const int x=4+i*59;text(label,std::min(470,x),14,2,TFT_WHITE,20);
    tft_.drawFastVLine(std::min(475,x),32,5,gray);}
  text("TPMS",64,75,2,TFT_WHITE,92);text("REF",135,79,1,gray,26);
  text("psi / C",77,241,1,gray,146);
  text("RPM",228,195,1,gray,96);
  tft_.pushSprite(0,0);
  return true;
}

void TftDashboard::push(int x,int y,int w,int h) {tft_.pushSprite(x,y,x,y,w,h);}

void TftDashboard::setBacklight(bool on) {
#if GAUGE_LED_CONTROL
  digitalWrite(pins::kTftBacklight,on?HIGH:LOW);
  pinMode(pins::kTftBacklight,OUTPUT);
#else
  (void)on; // r7: LED tied to 3.3V, never drive GPIO13.
#endif
}
void TftDashboard::startBootSweep() {
#if GAUGE_LED_CONTROL
  sweep_.start(millis());hasPrevious_=false;statusDirty_=true;
#endif
}
bool TftDashboard::bootSweepActive() const {
#if GAUGE_LED_CONTROL
  return sweep_.active(millis());
#else
  return false;
#endif
}
void TftDashboard::setSleeping(bool asleep) {
  if(asleep){sweep_.cancel();drawingSweep_=false;}

#if GAUGE_LED_CONTROL
  setBacklight(false);
#if GAUGE_PCB_REVA
  if(asleep){
    if(begun_){panel_.writecommand(0x28);panel_.writecommand(0x10);delay(120);}
    digitalWrite(7,LOW);pinMode(7,OUTPUT);
    return;
  }
  if(!begun_)return;
  digitalWrite(7,HIGH);delay(20);
  // RevA removes panel VCC. Sleep-out alone cannot restore its registers.
  panel_.init();panel_.setRotation(1);tft_.pushSprite(0,0);
  hasPrevious_=false;statusDirty_=true;
#else
  if(!begun_)return;
  if(asleep){panel_.writecommand(0x28);panel_.writecommand(0x10);delay(120);}
  else {panel_.writecommand(0x11);delay(120);panel_.writecommand(0x29);hasPrevious_=false;statusDirty_=true;}
#endif
#else
  (void)asleep;
#endif
}
void TftDashboard::setCanStatus(CanPhase phase) {
  if(canPhase_!=phase){canPhase_=phase;statusDirty_=true;}
}
void TftDashboard::setTrialRate(unsigned rate) {
  if(trialRate_!=rate){trialRate_=rate;statusDirty_=true;}
}
void TftDashboard::text(const char* value,int x,int y,int font,uint16_t color,int width,uint16_t background) {
  // Use real font metrics on hardware, falling back for long/negative values.
  while(tft_.textWidth(value,font)>width && font>1)font=font==8?4:font==4?2:1;
  tft_.setTextDatum(TC_DATUM);tft_.setTextColor(color,background);
  tft_.drawString(value,x,y,font);
}

void TftDashboard::rpm(const DashboardFrame& d) {
  tft_.fillRect(4,39,472,25,TFT_BLACK);
  tft_.fillRect(4,39,472,25,dim);
  const int filled=std::min<unsigned>(480,d.rpmBarPixels)*472/480;
  const int yellow=config::kRpmYellowStart*472/8000,red=config::kRpmRedStart*472/8000;
  if(filled>0)tft_.fillRect(4,39,std::min(filled,yellow),25,TFT_CYAN);
  if(filled>yellow)tft_.fillRect(4+yellow,39,std::min(filled,red)-yellow,25,TFT_YELLOW);
  if(filled>red)tft_.fillRect(4+red,39,filled-red,25,TFT_RED);
  // Static red-zone reference is distinct from the received-RPM fill.
  tft_.fillRect(4+red,65,472-red,2,TFT_RED);
  if(!hasPrevious_ || !gaugev4::kDriveUi)push(4,39,472,28);
  else {
    const int old=std::min<unsigned>(480,previous_.dashboard.rpmBarPixels)*472/480;
    if(old!=filled)push(4+std::min(old,filled),39,std::abs(old-filled),25);
  }
  if(!hasPrevious_ || d.rpmText!=previous_.dashboard.rpmText) {
    // Numeric RPM is independent of the gear glyph and its reference label.
    tft_.fillRect(180,205,96,26,TFT_BLACK);
    text(d.rpmText.data(),228,205,4,missing(d.rpmText)?gray:TFT_WHITE,94);
    push(180,205,96,26);
  }
}
void TftDashboard::wheel(int x,int y,const std::array<char,8>& psi,
                          const std::array<char,8>& temp,bool valid,bool pressureChanged,bool temperatureChanged) {
  tft_.fillRect(x-36,y,72,70,TFT_BLACK);
  tft_.fillRoundRect(x-31,y+2,62,38,12,valid?TFT_CYAN:dim);
  // Font4 has heavier 26px glyphs; all values fit the unchanged 72x70 cell.
  text(psi.data(),x,y+8,4,valid?TFT_BLACK:gray,58,valid?TFT_CYAN:dim);
  text(temp.data(),x,y+43,4,valid?TFT_WHITE:gray,68);
  if(!hasPrevious_ || !gaugev4::kDriveUi || (pressureChanged && temperatureChanged))push(x-36,y,72,70);
  else if(pressureChanged)push(x-36,y+2,72,38);
  else if(temperatureChanged)push(x-36,y+43,72,27);
}
void TftDashboard::temperature(int y,const char* label,const std::array<char,8>& value) {
  tft_.fillRect(303,y,93,59,TFT_BLACK);
  text(label,349,y+2,2,gray,86);
  text(value.data(),339,y+25,4,missing(value)?gray:TFT_WHITE,62);
  text("C",382,y+35,1,gray,14);
  if(!hasPrevious_ || !gaugev4::kDriveUi)push(303,y,93,59);
  else push(303,y+25,93,34);
}
void TftDashboard::pedal(const std::array<char,8>& value) {
  tft_.fillRect(397,73,41,181,TFT_BLACK);
  text("ACC",417,78,2,TFT_WHITE,38);
  tft_.drawRect(405,99,24,120,gray);
  if(!missing(value)) {
    const int h=std::clamp(std::atoi(value.data()),0,100)*118/100;
    if(h)tft_.fillRect(406,218-h,22,h,TFT_CYAN);
  }
  text(value.data(),417,227,2,missing(value)?gray:TFT_WHITE,38);
  text("%",417,246,1,gray,38);
  if(!hasPrevious_ || !gaugev4::kDriveUi)push(397,73,41,181);
  else {
    const auto& before=previous_.dashboard.accelText;
    const int oldH=missing(before)?0:std::clamp(std::atoi(before.data()),0,100)*118/100;
    const int newH=missing(value)?0:std::clamp(std::atoi(value.data()),0,100)*118/100;
    if(oldH!=newH)push(406,218-std::max(oldH,newH),22,std::abs(oldH-newH));
    push(397,227,41,18);
  }
}
void TftDashboard::boost(const DashboardFrame& d) {
  tft_.fillRect(439,73,41,181,TFT_BLACK);
  text(gaugev4::kDriveUi?"REL":"BST",459,78,2,TFT_WHITE,38);
  tft_.drawRect(443,99,17,120,gray);
  const int zero=218-static_cast<int>(d.boostZeroPixels)*118/480;
  if(d.boostValid) {
    const int end=218-std::min<unsigned>(480,d.boostEndpointPixels)*118/480;
    const int top=std::min(end,zero),h=std::abs(end-zero);
    if(h)tft_.fillRect(444,top,15,h,TFT_CYAN);
  }
  tft_.drawFastHLine(441,zero,21,TFT_WHITE);
  text("2",470,98,1,gray,15);text("0",470,zero-4,1,gray,15);
  text("-1",470,211,1,gray,18);
  text(d.boostText.data(),459,229,1,d.boostValid?TFT_WHITE:gray,40);
  text("bar",459,246,1,gray,38);
  if(!hasPrevious_ || !gaugev4::kDriveUi)push(439,73,41,181);
  else {
    const auto& before=previous_.dashboard;
    const int oldZero=static_cast<int>(before.boostZeroPixels)*118/480;
    const int newZero=static_cast<int>(d.boostZeroPixels)*118/480;
    const int oldH=before.boostValid?std::min<unsigned>(480,before.boostEndpointPixels)*118/480:oldZero;
    const int newH=d.boostValid?std::min<unsigned>(480,d.boostEndpointPixels)*118/480:newZero;
    // Missing means no fill at the zero reference, including below-zero bars.
    if(oldZero!=newZero)push(441,99,21,120);
    else if(oldH!=newH)push(444,218-std::max(oldH,newH),15,std::abs(oldH-newH));
    push(439,229,41,9);
  }
}
void TftDashboard::bottom(int x,const char* label,const char* value,const char* unit) {
  tft_.fillRect(x+1,256,159,46,TFT_BLACK);
  text(label,x+80,258,1,gray,152);
  text(value,x+72,273,4,TFT_WHITE,108);
  text(unit,x+139,286,1,gray,36);
  if(!hasPrevious_ || !gaugev4::kDriveUi)push(x+1,256,159,46);
  // Refresh the full old/new value cell, but not the unchanged label/unit.
  // The centered text is bounded to 108px and font4 is 26px high.
  else push(x+18,273,108,26);
}
void TftDashboard::footer(const gaugev4::model::Frame& f) {
  using namespace gaugev4::alerts;
  tft_.fillRect(0,303,480,17,TFT_BLACK);
  if(drawingSweep_){
    text("STARTUP DEMO",240,307,1,TFT_CYAN,470);
    tft_.fillRect(4,68,472,2,TFT_BLACK);
    push(0,303,480,17);push(4,68,472,2);statusDirty_=false;return;
  }
  const char* can=canPhase_==CanPhase::Active?"CAN ACTIVE":
    canPhase_==CanPhase::ListenOnly?"DISCOVER":
    canPhase_==CanPhase::Boot?(trialRate_?"ARMED 3S":"WAIT START"):"CAN OFF";
  text(can,46,307,1,gray,90);
  text(dataLabel(f.dataState),156,307,1,f.dataState==DataState::Stale?TFT_YELLOW:gray,116);
  text(shiftLabel(f.shift),290,307,1,f.shift==ShiftState::Shift?TFT_WHITE:gray,136);
  text("OIL P N/A",421,307,1,dim,112);
  tft_.fillRect(4,68,472,2,f.shift==ShiftState::Shift?TFT_WHITE:TFT_BLACK);
  push(0,303,480,17);push(4,68,472,2);
  statusDirty_=false;
}
gaugev4::model::Frame TftDashboard::nextSlice(const gaugev4::model::Frame& incoming) {
  auto f=incoming;
  if(drawingSweep_ || !gaugev4::kPairDrive || canPhase_!=CanPhase::Active || !hasPrevious_)return f;
  // Keep fast fields current. Cache only values actually sent to the panel:
  // deferred slow fields remain dirty until their own round-robin turn.
  auto& d=f.dashboard;const auto& old=previous_.dashboard;const auto& latest=incoming.dashboard;
  std::array<char,8>* dest[]={&d.tpmsFlPsi,&d.tpmsFrPsi,&d.tpmsRlPsi,&d.tpmsRrPsi,
    &d.tpmsFlTemp,&d.tpmsFrTemp,&d.tpmsRlTemp,&d.tpmsRrTemp,&d.coolantText,&d.oilText,&d.intakeText,&d.voltageText};
  const std::array<char,8>* before[]={&old.tpmsFlPsi,&old.tpmsFrPsi,&old.tpmsRlPsi,&old.tpmsRrPsi,
    &old.tpmsFlTemp,&old.tpmsFrTemp,&old.tpmsRlTemp,&old.tpmsRrTemp,&old.coolantText,&old.oilText,&old.intakeText,&old.voltageText};
  const std::array<char,8>* after[]={&latest.tpmsFlPsi,&latest.tpmsFrPsi,&latest.tpmsRlPsi,&latest.tpmsRrPsi,
    &latest.tpmsFlTemp,&latest.tpmsFrTemp,&latest.tpmsRlTemp,&latest.tpmsRrTemp,&latest.coolantText,&latest.oilText,&latest.intakeText,&latest.voltageText};
  for(unsigned i=0;i<12;++i)*dest[i]=*before[i];
  f.reference.tpmsCandidateVisible=previous_.reference.tpmsCandidateVisible;
  constexpr unsigned slots[]={0,1,3,2};
  for(unsigned tries=0;tries<8;++tries){
    const unsigned cell=slowCell_;slowCell_=(slowCell_+1)%8;
    if(cell<4){
      const unsigned slot=slots[cell];
      if(*before[cell]==*after[cell] && *before[cell+4]==*after[cell+4] &&
         f.reference.tpmsCandidateVisible[slot]==incoming.reference.tpmsCandidateVisible[slot])continue;
      *dest[cell]=*after[cell];*dest[cell+4]=*after[cell+4];
      f.reference.tpmsCandidateVisible[slot]=incoming.reference.tpmsCandidateVisible[slot];
    }else{
      const unsigned field=cell+4;
      if(*before[field]==*after[field])continue;
      *dest[field]=*after[field];
    }
    break;
  }
  return f;
}
void TftDashboard::render(const gaugev4::model::Frame& incoming) {
  if(!begun_)return;
  auto source=incoming;
#if GAUGE_LED_CONTROL
  if(incoming.dataState==gaugev4::alerts::DataState::Off)sweep_.cancel();
  const bool demo=bootSweepActive();
  if(drawingSweep_!=demo){hasPrevious_=false;statusDirty_=true;}
  drawingSweep_=demo;
  if(demo)source=sweep_.frame(millis());
#endif
  const auto f=nextSlice(source);
  const auto& d=f.dashboard;const auto& p=previous_.dashboard;
  if(!hasPrevious_ || d.rpmBarPixels!=p.rpmBarPixels || d.rpmText!=p.rpmText)rpm(d);
  // Confidence remains in diagnostics, not in digit brightness. Repainting an
  // unchanged digit on Candidate/Held/Confirmed transitions caused pulsing.
  if(!hasPrevious_ || d.gearText!=p.gearText) {
    tft_.fillRect(155,116,147,78,TFT_BLACK);
    const bool reverse=d.gearText[0]=='R';
    // Font8 only contains digits/punctuation. Use enlarged alphabetic font4 for R.
    tft_.setTextSize(reverse?3:1);
    text(d.gearText.data(),228,116,reverse?4:8,missing(d.gearText)?gray:orange,140);
    tft_.setTextSize(1);
    push(155,116,147,78);
    if(!hasPrevious_ || reverse!=(p.gearText[0]=='R')) {
      tft_.fillRect(155,235,147,16,TFT_BLACK);
      text(gaugev4::kDriveUi ? (reverse?"R REF":"GEAR EST") : "GEAR",228,235,2,gray,138);
      push(155,235,147,16);
    }
  }
  if(!hasPrevious_ || d.accelText!=p.accelText)pedal(d.accelText);
  if(!hasPrevious_ || d.boostText!=p.boostText || d.boostValid!=p.boostValid ||
      d.boostEndpointPixels!=p.boostEndpointPixels || d.boostZeroPixels!=p.boostZeroPixels)boost(d);
  if(!hasPrevious_ || d.brakeText!=p.brakeText)bottom(0,"BRK REF",d.brakeText.data(),"");
  if(!hasPrevious_ || d.speedText!=p.speedText)bottom(160,"SPEED",d.speedText.data(),"km/h");
  if(!hasPrevious_ || statusDirty_ || f.dataState!=previous_.dataState || f.shift!=previous_.shift)footer(f);
  // Fast digits/bars and status reach the panel before any slow cell.
  const std::array<char,8>* ps[]={&d.tpmsFlPsi,&d.tpmsFrPsi,&d.tpmsRlPsi,&d.tpmsRrPsi};
  const std::array<char,8>* ts[]={&d.tpmsFlTemp,&d.tpmsFrTemp,&d.tpmsRlTemp,&d.tpmsRrTemp};
  const std::array<char,8>* pp[]={&p.tpmsFlPsi,&p.tpmsFrPsi,&p.tpmsRlPsi,&p.tpmsRrPsi};
  const std::array<char,8>* pt[]={&p.tpmsFlTemp,&p.tpmsFrTemp,&p.tpmsRlTemp,&p.tpmsRrTemp};
  const unsigned slots[]={0,1,3,2};
  for(unsigned i=0;i<4;++i)if(!hasPrevious_ || *ps[i]!=*pp[i] || *ts[i]!=*pt[i] ||
     f.reference.tpmsCandidateVisible[slots[i]]!=previous_.reference.tpmsCandidateVisible[slots[i]])
    wheel(i%2?115:38,i<2?98:171,*ps[i],*ts[i],f.reference.tpmsCandidateVisible[slots[i]],
      !hasPrevious_ || *ps[i]!=*pp[i] || f.reference.tpmsCandidateVisible[slots[i]]!=previous_.reference.tpmsCandidateVisible[slots[i]],
      !hasPrevious_ || *ts[i]!=*pt[i] || f.reference.tpmsCandidateVisible[slots[i]]!=previous_.reference.tpmsCandidateVisible[slots[i]]);
  if(!hasPrevious_ || d.coolantText!=p.coolantText)temperature(73,"WTR",d.coolantText);
  if(!hasPrevious_ || d.oilText!=p.oilText)temperature(134,"OIL~",d.oilText);
  if(!hasPrevious_ || d.intakeText!=p.intakeText)temperature(195,"ITK",d.intakeText);
  if(!hasPrevious_ || d.voltageText!=p.voltageText)bottom(320,"ECU V",d.voltageText.data(),"V");
  previous_=f;hasPrevious_=true;
}
