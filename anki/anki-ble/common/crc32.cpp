/**
 * File: crc32.cpp
 *
 * Author: seichert
 * Created: 2/28/2018
 *
 * Description: Routines for calculating crc32 values
 *
 * Copyright: Anki, Inc. 2018
 *
 * Based on various public domain implementations of the crc32 algorithm
 *
 **/

#include "crc32.h"
#include <cstddef>

namespace Anki {

static uint32_t CrcForByte(uint32_t r)
{
  for(int j = 0; j < 8; ++j) {
    r = (r & 1? 0: (uint32_t)0xEDB88320L) ^ r >> 1;
  }
  return r ^ (uint32_t)0xFF000000L;
}

uint32_t Crc32(const std::vector<uint8_t>& data)
{
  uint32_t crc = 0;
  static uint32_t table[0x100];
  if (!table[0]) {
    for (size_t i = 0; i < 0x100; ++i) {
      table[i] = CrcForByte(i);
    }
  }
  for (size_t i = 0 ; i < data.size(); i++) {
    crc = table[(uint8_t)crc ^ data[i]] ^ crc >> 8;
  }

  return crc;
}

} // namespace Anki
