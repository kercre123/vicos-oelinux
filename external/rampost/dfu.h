#ifndef DFU_H_
#define DFU_H_

#include "rampost.h"

int dfu_if_needed(const char* dfu_file, const uint64_t timeout);

RampostErr dfu_sequence(const char* dfu_file, const uint64_t timeout);

#endif//DFU_H_
