#pragma once
#include "runtime.hpp"

namespace gaugev4 {
struct MarkEvent { uint32_t n=0, at=0; const char* tag=nullptr; };
class MarkerQueue {
 public:
  bool command(const char* line,uint32_t at,bool active) {
    if(!active || std::strncmp(line,"MARK ",5)!=0) return false;
    static const char* tags[]={"BASELINE","CLUTCH_RELEASED","CLUTCH_PRESSED","BRAKE_RELEASED","BRAKE_PRESSED","MODE_NORMAL","MODE_SPORT","MODE_N","MODE_CUSTOM","OEM_NOTE"};
    const char* tag=nullptr;
    for(const char* candidate:tags) if(std::strcmp(line+5,candidate)==0) tag=candidate;
    if(!tag) return false;
    if(count_>=32){++dropped_;return false;}
    events_[count_]={count_+1,at,tag};++count_;return true;
  }
  bool pop(MarkEvent& event){if(read_==count_)return false;event=events_[read_++];return true;}
  uint32_t count() const{return count_;}
  uint32_t queued() const{return count_-read_;}
  uint32_t dropped() const{return dropped_;}
 private:
  MarkEvent events_[32]{};
  uint32_t count_=0,read_=0,dropped_=0;
};
struct BusEvent { Frame frame{};uint32_t n=0,at=0; };
class BusJournal {
 public:
  void observe(const Frame& frame,uint32_t at){
    if(frame.extended||frame.rtr||frame.dlc>8)return;
    unsigned i=0;for(;i<CandidateSightings::capacity;++i)if(frame.id==CandidateSightings::idAt(i))break;
    if(i==CandidateSightings::capacity)return;
    if(seen_[i]&&last_[i].dlc==frame.dlc&&std::memcmp(last_[i].data,frame.data,frame.dlc)==0)return;
    seen_[i]=true;last_[i]=frame;++ordinal_;
    if(size_==128){++dropped_;return;}
    events_[(head_+size_)%128]={frame,ordinal_,at};++size_;
  }
  bool pop(BusEvent& event){if(!size_)return false;event=events_[head_];head_=(head_+1)%128;--size_;return true;}
  uint32_t queued() const{return size_;}
  uint32_t dropped() const{return dropped_;}
 private:
  BusEvent events_[128]{};
  Frame last_[8]{};bool seen_[8]{};
  uint32_t head_=0,size_=0,ordinal_=0,dropped_=0;
};
class CollectLog {
 public:
  template<typename Sink,typename ObservationType>
  bool observation(Sink& sink,const ObservationType& obs,bool pending){
    if(pending||!obs.generation||obs.generation==seen_)return true;
    seen_=obs.generation;
    const unsigned profile=static_cast<unsigned>(obs.profile);
    static const char* names[]={"STD","OIL","TPMS","BRAKE_PRESSURE","BRAKE_SWITCH","REVERSE"};
    if(profile>=6||obs.length>256){++obsDropped_;return false;}
    char line[800];
    int n=std::snprintf(line,sizeof(line),"OBS seq=%lu profile=%s key=%02X request_at_ms=%lu received_at_ms=%lu complete=%u bytes=%u data=",
      static_cast<unsigned long>(obs.requestOrdinal),names[profile],static_cast<unsigned>(obs.key),static_cast<unsigned long>(obs.requestAt),static_cast<unsigned long>(obs.receivedAt),obs.complete?1u:0u,obs.length);
    if(n<=0||static_cast<size_t>(n)>=sizeof(line)){++obsDropped_;return false;}
    size_t used=static_cast<size_t>(n);
    if(!hex(line,sizeof(line),used,obs.payload,obs.length)||!write(sink,line,used)){++obsDropped_;return false;}return true;
  }
  template<typename Sink>bool markers(Sink& sink,MarkerQueue& queue,bool pending){
    if(pending)return true;
    bool ok=true;MarkEvent event;
    for(unsigned i=0;i<4&&queue.pop(event);++i){
      char line[128];int n=std::snprintf(line,sizeof(line),"MARK n=%lu at_ms=%lu tag=%s\n",static_cast<unsigned long>(event.n),static_cast<unsigned long>(event.at),event.tag);
      if(n<=0||static_cast<size_t>(n)>=sizeof(line)||!write(sink,line,static_cast<size_t>(n))){++markDropped_;ok=false;}
    }return ok;
  }
  template<typename Sink>bool bus(Sink& sink,BusJournal& queue,bool pending){
    if(pending)return true;
    bool ok=true;BusEvent event;
    for(unsigned i=0;i<4&&queue.pop(event);++i){
      char line[160];int n=std::snprintf(line,sizeof(line),"BUS n=%lu at_ms=%lu id=%03lX dlc=%u ext=0 rtr=0 data=",static_cast<unsigned long>(event.n),static_cast<unsigned long>(event.at),static_cast<unsigned long>(event.frame.id),static_cast<unsigned>(event.frame.dlc));
      if(n<=0||static_cast<size_t>(n)>=sizeof(line)){++busDropped_;ok=false;continue;}
      size_t used=static_cast<size_t>(n);
      if(!hex(line,sizeof(line),used,event.frame.data,event.frame.dlc)||!write(sink,line,used)){++busDropped_;ok=false;}
    }return ok;
  }
  template<typename Sink>bool metrics(Sink& sink,const BusJournal& bus,const MarkerQueue& marks,uint32_t epoch=0){
    char line[224];int n=std::snprintf(line,sizeof(line),"COLLECT_METRICS obs_dropped=%lu bus_dropped=%lu mark_dropped=%lu bus_queued=%lu marks=%lu trial_at_ms=%lu\n",static_cast<unsigned long>(obsDropped_),static_cast<unsigned long>(bus.dropped()+busDropped_),static_cast<unsigned long>(marks.dropped()+markDropped_),static_cast<unsigned long>(bus.queued()),static_cast<unsigned long>(marks.count()),static_cast<unsigned long>(epoch));
    if(n<=0||static_cast<size_t>(n)>=sizeof(line)||!write(sink,line,static_cast<size_t>(n))){++obsDropped_;return false;}return true;
  }
  uint32_t obsDropped() const{return obsDropped_;}
  uint32_t busDropped() const{return busDropped_;}
 private:
  static bool hex(char* line,size_t size,size_t& used,const uint8_t* data,unsigned count){
    if(used+count*2+2>size)return false;
    static const char digits[]="0123456789ABCDEF";
    for(unsigned i=0;i<count;++i){line[used++]=digits[data[i]>>4];line[used++]=digits[data[i]&15];}
    line[used++]='\n';line[used]='\0';return true;
  }
  template<typename Sink>static bool write(Sink& sink,const char* text,size_t n){
    return sink.availableForWrite()>=static_cast<int>(n)&&sink.write(reinterpret_cast<const uint8_t*>(text),n)==n;
  }
  uint32_t seen_=0,obsDropped_=0,busDropped_=0,markDropped_=0;
};
}
