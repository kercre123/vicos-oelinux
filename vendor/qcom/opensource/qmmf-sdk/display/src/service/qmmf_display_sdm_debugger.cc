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

#define TAG "DisplayDebugHandler"

#include "sdm/include/utils/constants.h"
#include <cutils/properties.h>

#include "display/src/service/qmmf_display_sdm_debugger.h"

namespace qmmf {

namespace display {

  #define MAX_NAME_SIZE 256

DisplayDebugHandler DisplayDebugHandler::debug_handler_;
std::bitset<32> DisplayDebugHandler::debug_flags_ = 0x7FFFFFFF;
int32_t DisplayDebugHandler::verbose_level_ = 0x0;

void DisplayDebugHandler::DebugAll(bool enable, int verbose_level) {
  if (enable) {
    debug_flags_ = 0x7FFFFFFF;
    verbose_level_ = verbose_level;
  } else {
    debug_flags_ = 0x1;   // kTagNone should always be printed.
    verbose_level_ = 0;
  }
}

void DisplayDebugHandler::DebugResources(bool enable, int verbose_level) {
  if (enable) {
    debug_flags_[kTagResources] = 1;
    verbose_level_ = verbose_level;
  } else {
    debug_flags_[kTagResources] = 0;
    verbose_level_ = 0;
  }
}

void DisplayDebugHandler::DebugStrategy(bool enable, int verbose_level) {
  if (enable) {
    debug_flags_[kTagStrategy] = 1;
    verbose_level_ = verbose_level;
  } else {
    debug_flags_[kTagStrategy] = 0;
    verbose_level_ = 0;
  }
}

void DisplayDebugHandler::DebugCompManager(bool enable, int verbose_level) {
  if (enable) {
    debug_flags_[kTagCompManager] = 1;
    verbose_level_ = verbose_level;
  } else {
    debug_flags_[kTagCompManager] = 0;
    verbose_level_ = 0;
  }
}

void DisplayDebugHandler::DebugDriverConfig(bool enable, int verbose_level) {
  if (enable) {
    debug_flags_[kTagDriverConfig] = 1;
    verbose_level_ = verbose_level;
  } else {
    debug_flags_[kTagDriverConfig] = 0;
    verbose_level_ = 0;
  }
}

void DisplayDebugHandler::DebugRotator(bool enable, int verbose_level) {
  if (enable) {
    debug_flags_[kTagRotator] = 1;
    verbose_level_ = verbose_level;
  } else {
    debug_flags_[kTagRotator] = 0;
    verbose_level_ = 0;
  }
}

void DisplayDebugHandler::DebugQdcm(bool enable, int verbose_level) {
  if (enable) {
    debug_flags_[kTagQDCM] = 1;
    verbose_level_ = verbose_level;
  } else {
    debug_flags_[kTagQDCM] = 0;
    verbose_level_ = 0;
  }
}

void DisplayDebugHandler::Error(DebugTag tag, const char *format, ...) {
  va_list list;
  va_start(list, format);
  __android_log_vprint(ANDROID_LOG_ERROR, LOG_TAG, format, list);
}

void DisplayDebugHandler::Warning(DebugTag tag, const char *format, ...) {
  va_list list;
  va_start(list, format);
  __android_log_vprint(ANDROID_LOG_WARN, LOG_TAG, format, list);
}

void DisplayDebugHandler::Info(DebugTag tag, const char *format, ...) {
  if (debug_flags_[tag]) {
    va_list list;
    va_start(list, format);
    __android_log_vprint(ANDROID_LOG_INFO, LOG_TAG, format, list);
  }
}

void DisplayDebugHandler::Debug(DebugTag tag, const char *format, ...) {
  if (debug_flags_[tag]) {
    va_list list;
    va_start(list, format);
    __android_log_vprint(ANDROID_LOG_DEBUG, LOG_TAG, format, list);
  }
}

void DisplayDebugHandler::Verbose(DebugTag tag, const char *format, ...) {
  if (debug_flags_[tag] && verbose_level_) {
    va_list list;
    va_start(list, format);
    __android_log_vprint(ANDROID_LOG_VERBOSE, LOG_TAG, format, list);
  }
}

void DisplayDebugHandler::BeginTrace(const char *class_name,
    const char *function_name, const char *custom_string) {
  char name[MAX_NAME_SIZE] = {0};
  snprintf(name, sizeof(name), "%s::%s::%s", class_name, function_name,
      custom_string);
  //TBD
}

void DisplayDebugHandler::EndTrace() {
  //TBD
}

int  DisplayDebugHandler::GetIdleTimeoutMs() {
  int value = IDLE_TIMEOUT_DEFAULT_MS;
  debug_handler_.GetProperty("sdm.idle_time", &value);

  return value;
}

DisplayError DisplayDebugHandler::GetProperty(const char *property_name,
    int *value) {
  char property[PROPERTY_VALUE_MAX];

  if (property_get(property_name, property, NULL) > 0) {
    *value = atoi(property);
    return kErrorNone;
  }

  return kErrorNotSupported;
}

DisplayError DisplayDebugHandler::GetProperty(const char *property_name,
    char *value) {
  if (property_get(property_name, value, NULL) > 0) {
    return kErrorNone;
  }

  return kErrorNotSupported;
}

DisplayError DisplayDebugHandler::SetProperty(const char *property_name,
    const char *value) {
  if (property_set(property_name, value) == 0) {
    return kErrorNone;
  }

  return kErrorNotSupported;
}

}; // namespace display

}; //namespace qmmf

