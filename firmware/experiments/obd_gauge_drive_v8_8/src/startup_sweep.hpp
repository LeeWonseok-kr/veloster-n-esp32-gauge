#pragma once
#include "model.hpp"
namespace gaugev4::startup {
// Display demonstration ranges, never sensor limits or calibration.
class Sweep {
 public:
 void start(uint32_t now){start_=now;enabled_=true;}
 void cancel(){enabled_=false;}
 bool active(uint32_t now)const{return enabled_ && uint32_t(now-start_)<2200;}
 model::Frame frame(uint32_t now)const{
   const uint32_t t=uint32_t(now-start_);
   float q=t<100?0:t<1000?(t-100)/900.0f:t<1200?1:t<2100?(2100-t)/900.0f:0;
   model::Frame f{}; auto& d=f.dashboard;
   auto number=[q](auto& field,float low,float high,const char* fmt="%.0f"){
     std::snprintf(field.data(),field.size(),fmt,static_cast<double>(low+(high-low)*q));
   };
   number(d.rpmText,0,8000); d.rpmBarPixels=static_cast<uint16_t>(480*q);
   number(d.speedText,0,300); number(d.accelText,0,100);
   number(d.boostText,-1,2,"%.2f"); d.boostValid=true;
   d.boostEndpointPixels=static_cast<uint16_t>(480*q);
   d.boostZeroPixels=160;
   number(d.coolantText,0,150); number(d.oilText,0,150); number(d.intakeText,-40,100);
   number(d.voltageText,0,16,"%.1f"); number(d.gearText,1,6);
   std::snprintf(d.brakeText.data(),d.brakeText.size(),"%s",q>=0.5f?"ON":"OFF");
   for(auto* p:{&d.tpmsFlPsi,&d.tpmsFrPsi,&d.tpmsRlPsi,&d.tpmsRrPsi})number(*p,0,60,"%.1f");
   for(auto* p:{&d.tpmsFlTemp,&d.tpmsFrTemp,&d.tpmsRlTemp,&d.tpmsRrTemp})number(*p,-40,125);
   f.reference.tpmsCandidateVisible.fill(true); return f;
 }
 private:uint32_t start_=0;bool enabled_=false;
};
}
