#ifndef DFU_H_
#define DFU_H_

#include "rampost.h"

RampostErr dfu_sequence(const char* dfu_file, bool force_update);

#endif//DFU_H_
