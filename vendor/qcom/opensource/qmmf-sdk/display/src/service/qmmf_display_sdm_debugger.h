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

#include "sdm/include/core/sdm_types.h"
#include "sdm/include/core/debug_interface.h"
#include "cutils/log.h"
#include "utils/Trace.h"
#include "bitset"

using namespace sdm;

namespace qmmf {

namespace display {

class DisplayDebugHandler : public DebugHandler {
 public:
  static inline DebugHandler* Get() { return &debug_handler_; }

  static void DebugAll(bool enable, int verbose_level);
  static void DebugResources(bool enable, int verbose_level);
  static void DebugStrategy(bool enable, int verbose_level);
  static void DebugCompManager(bool enable, int verbose_level);
  static void DebugDriverConfig(bool enable, int verbose_level);
  static void DebugRotator(bool enable, int verbose_level);
  static void DebugQdcm(bool enable, int verbose_level);
  static int  GetIdleTimeoutMs();

  virtual void Error(DebugTag tag, const char *format, ...) override;
  virtual void Warning(DebugTag tag, const char *format, ...) override;
  virtual void Info(DebugTag tag, const char *format, ...) override;
  virtual void Debug(DebugTag tag, const char *format, ...) override;
  virtual void Verbose(DebugTag tag, const char *format, ...) override;
  virtual void BeginTrace(const char *class_name, const char *function_name,
                          const char *custom_string) override;
  virtual void EndTrace() override;
  virtual DisplayError GetProperty(const char *property_name, int *value) override;
  virtual DisplayError GetProperty(const char *property_name, char *value) override;
  virtual DisplayError SetProperty(const char *property_name,
      const char *value) override;

 private:
  static DisplayDebugHandler debug_handler_;
  static std::bitset<32> debug_flags_;
  static int32_t verbose_level_;
};

}; // namespace display

}; //namespace qmmf
