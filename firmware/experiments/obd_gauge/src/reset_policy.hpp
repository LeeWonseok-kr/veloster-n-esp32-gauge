#pragma once
namespace obdgauge {
inline bool bootLocked(bool latched, bool powerOn, bool externalReset) {
  if(powerOn) return false;
  return latched || !externalReset;
}
}
