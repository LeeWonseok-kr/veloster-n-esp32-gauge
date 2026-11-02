#pragma once
namespace obdprobe {
class CommandLine {
 public:
  bool feed(char c) {
    if(c=='\n') { text_[size_]='\0'; readyValid_=!invalid_; size_=0; invalid_=false; cr_=false; return true; }
    if(cr_) invalid_=true; // CR is allowed only immediately before the final LF.
    if(c=='\r') cr_=true;
    else {
      if(c=='\0' || size_>=sizeof(text_)-1) invalid_=true;
      else if(!invalid_) text_[size_++]=c;
    }
    return false;
  }
  bool valid() const { return readyValid_; }
  const char* text() const { return text_; }
 private:
  char text_[40]{};
  unsigned size_=0;
  bool invalid_=false,readyValid_=false,cr_=false;
};
}
