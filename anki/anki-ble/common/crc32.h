/**
 * File: crc32.h
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


#pragma once

#include <cstdint>
#include <vector>

namespace Anki {
uint32_t Crc32(const std::vector<uint8_t>& data);
} // namespace Anki
