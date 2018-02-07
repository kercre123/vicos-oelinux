/*
* Copyright (c) 2016, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <utils/List.h>
#include <utils/Mutex.h>

#include "qmmf-sdk/qmmf_display_params.h"
#include "common/qmmf_log.h"

namespace qmmf {

namespace display {

/*
* Define LOG_LEVEL1 & 2 enable more debug logs.
*/
//#define LOG_LEVEL1
//#define LOG_LEVEL2

#ifdef LOG_LEVEL1
#define QMMF_LEVEL1(fmt, args...)  ALOGD(fmt, ##args)
#else
#define QMMF_LEVEL1(...) ((void)0)
#endif

#ifdef LOG_LEVEL2
#define QMMF_LEVEL2(fmt, args...)  ALOGD(fmt, ##args)
#else
#define QMMF_LEVEL2(...) ((void)0)
#endif

/* handle to a specific display client/service connection */
typedef int32_t DisplayHandle;

#define GRALLOC_MODULE_PATH    "/usr/lib/hw/gralloc.default.so"

}; //namespace display.

}; //namespace qmmf.
