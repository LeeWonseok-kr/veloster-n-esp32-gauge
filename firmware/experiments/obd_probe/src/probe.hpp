#pragma once
#include <cstdint>
#include <cstring>
namespace obdprobe {
struct Frame { uint32_t id=0; uint8_t dlc=0; uint8_t data[8]={}; bool extended=false,rtr=false; };
struct Port {
  virtual ~Port() = default;
  virtual bool open()=0;
  virtual void stop()=0;
  virtual bool good()=0;
  virtual bool receive(Frame&)=0;
  virtual bool send(const Frame&)=0;
};
enum class State { Idle, Active, Done, Fault };
inline bool allowedRequest(const Frame& f) {
  if(f.id!=0x7df || f.dlc!=8 || f.extended || f.rtr ||
     f.data[0]!=2 || f.data[1]!=1 ||
     (f.data[2]!=0 && f.data[2]!=5 && f.data[2]!=12)) return false;
  for(unsigned i=3;i<8;++i) if(f.data[i]) return false;
  return true;
}
class Probe {
 public:
  explicit Probe(Port& port): port_(port) {}
  bool start(uint32_t now) {
    if(state_!=State::Idle) return false;
    if(!port_.open()) { finish(State::Fault,"CAN_INIT"); return false; }
    state_=State::Active; reason_="ACTIVE"; started_=now; return true;
  }
  bool command(const char* line,uint32_t now) {
    if(std::strcmp(line,"START OBD TEST")==0) return start(now);
    if(std::strcmp(line,"STOP")==0) {
      finish(State::Done,"USER_STOP"); return true;
    }
    return false;
  }
  void tick(uint32_t now) {
    if(state_!=State::Active) return;
    if(now-started_>=15000) { finish(State::Done,"TIME_LIMIT"); return; }
    if(!port_.good()) { finish(State::Fault,"CAN_ERROR"); return; }
    if(pending_ && now-lastTx_>=500) { finish(State::Fault,"NO_RESPONSE"); return; }
    Frame f{};
    // Bounded drain prevents flood traffic from starving deadlines/STOP handling.
    for(unsigned i=0;i<128 && port_.receive(f);++i) {
      ++rawRx_;
      accept(f,now);
      if(state_!=State::Active) return;
    }
    if(!port_.good()) { finish(State::Fault,"CAN_ERROR"); return; }
    if(pending_ || (requests_ && now-lastTx_<1000)) return;
    if(requests_>=10) { finish(State::Done,"REQUEST_LIMIT"); return; }
    if(discovered_) {
      if(!supportsRpm_ && !supportsCoolant_) {
        finish(State::Done,"NO_SUPPORTED_PID"); return;
      }
      pid_ = supportsRpm_ && (!supportsCoolant_ || lastPid_!=12) ? 12 : 5;
    } else pid_=0;
    Frame request{}; request.id=0x7df; request.dlc=8;
    request.data[0]=2; request.data[1]=1; request.data[2]=pid_;
    if(!allowedRequest(request) || !port_.send(request)) {
      finish(State::Fault,"TX_FAILED"); return;
    }
    ++requests_; lastTx_=now; lastPid_=pid_; pending_=true;
  }
  State state() const { return state_; }
  const char* reason() const { return reason_; }
  uint32_t responses() const { return responses_; }
  uint32_t requests() const { return requests_; }
  uint32_t rawRx() const { return rawRx_; }
  uint32_t ecu() const { return ecu_; }
  float rpm() const { return rpm_; }
  int coolant() const { return coolant_; }
  bool rpmFresh(uint32_t now) const { return state_==State::Active && haveRpm_ && now-rpmAt_<3000; }
  bool coolantFresh(uint32_t now) const { return state_==State::Active && haveCoolant_ && now-coolantAt_<3000; }
  bool valuesFresh(uint32_t now) const { return rpmFresh(now)||coolantFresh(now); }
 private:
  void finish(State state,const char* reason) {
    port_.stop(); state_=state; reason_=reason; pending_=false;
  }
  void accept(const Frame& f,uint32_t now) {
    if(!pending_ || f.extended || f.rtr || f.id<0x7e8 || f.id>0x7ef ||
       (ecu_ && f.id!=ecu_) || f.dlc>8 || f.dlc<3) return;
    const uint8_t length=f.data[0];
    if(length>7 || length+1>f.dlc) return; // Single Frame only; no VIN/ISO-TP flow control.
    if(length==3 && f.data[1]==0x7f && f.data[2]==1) {
      finish(State::Fault,"NEGATIVE_RESPONSE"); return;
    }
    const uint8_t expected=pid_==0?6:(pid_==12?4:3);
    if(length!=expected || f.data[1]!=0x41 || f.data[2]!=pid_) return;
    if(pid_==0) {
      const uint32_t bitmap=(uint32_t(f.data[3])<<24)|(uint32_t(f.data[4])<<16)|
                            (uint32_t(f.data[5])<<8)|f.data[6];
      supportsRpm_=(bitmap & 0x00100000)!=0;
      supportsCoolant_=(bitmap & 0x08000000)!=0;
      ecu_=f.id; discovered_=true;
    } else if(pid_==12) {
      rpm_=(uint32_t(f.data[3])*256+f.data[4])/4.0f; haveRpm_=true; rpmAt_=now;
    } else {
      coolant_=int(f.data[3])-40; haveCoolant_=true; coolantAt_=now;
    }
    ++responses_; pending_=false;
    if(discovered_ && !supportsRpm_ && !supportsCoolant_) finish(State::Done,"NO_SUPPORTED_PID");
    else if(requests_>=10) finish(State::Done,"REQUEST_LIMIT");
  }
  Port& port_;
  State state_=State::Idle;
  const char* reason_="WAIT_START";
  uint32_t started_=0,lastTx_=0,requests_=0,responses_=0,rawRx_=0,ecu_=0;
  uint32_t rpmAt_=0,coolantAt_=0;
  uint8_t pid_=0,lastPid_=0;
  bool pending_=false,discovered_=false,supportsRpm_=false,supportsCoolant_=false;
  bool haveRpm_=false,haveCoolant_=false;
  float rpm_=0;
  int coolant_=0;
};
}
