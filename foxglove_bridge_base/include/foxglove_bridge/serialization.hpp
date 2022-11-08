#pragma once

#include <stdint.h>

namespace foxglove {

inline void WriteUint64LE(uint8_t* buf, uint64_t val) {
#ifdef ARCH_IS_BIG_ENDIAN
  buf[0] = val & 0xff;
  buf[1] = (val >> 8) & 0xff;
  buf[2] = (val >> 16) & 0xff;
  buf[3] = (val >> 24) & 0xff;
  buf[4] = (val >> 32) & 0xff;
  buf[5] = (val >> 40) & 0xff;
  buf[6] = (val >> 48) & 0xff;
  buf[7] = (val >> 56) & 0xff;
#else
  reinterpret_cast<uint64_t*>(buf)[0] = val;
#endif
}

inline void WriteUint32LE(uint8_t* buf, uint32_t val) {
#ifdef ARCH_IS_BIG_ENDIAN
  buf[0] = val & 0xff;
  buf[1] = (val >> 8) & 0xff;
  buf[2] = (val >> 16) & 0xff;
  buf[3] = (val >> 24) & 0xff;
#else
  reinterpret_cast<uint32_t*>(buf)[0] = val;
#endif
}

}  // namespace foxglove
