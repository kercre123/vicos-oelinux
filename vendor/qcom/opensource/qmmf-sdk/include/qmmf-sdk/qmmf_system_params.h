/*
 * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#include <cstdint>
#include <functional>
#include <sstream>
#include <string>

#include <sys/types.h>

#include "qmmf-sdk/qmmf_codec.h"
#include "qmmf-sdk/qmmf_device.h"

namespace qmmf {
namespace system {

typedef int32_t status_t;

// System Class specific callbacks

// System errors
typedef ::std::function<void(const int32_t error)> SystemCb;

// SoundTrigger recognized utterance, or error occurred
typedef ::std::function<void(const int32_t error)> TriggerCb;

// device was unplugged or plugged
typedef ::std::function<void(const DeviceInfo& device)> DeviceCb;

// tone has finished playing, with possible error
typedef ::std::function<void(const int32_t error)> ToneCb;

struct SoundModel {
  DeviceId device;
  uint32_t keywords;
  uint32_t size;
  void*    data;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "device[" << device << "] ";
    stream << "keywords[" << keywords << "] ";
    stream << "size[" << size << "] ";
    stream << "data[" << data << "]";
    return stream.str();
  }
};

struct Tone {
  uint32_t delay;  // milliseconds
  uint32_t loop_num;
  uint32_t size;
  void*    buffer;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "delay[" << delay << "] ";
    stream << "loop_num[" << loop_num << "] ";
    stream << "size[" << size << "] ";
    stream << "buffer[" << buffer << "]";
    return stream.str();
  }
};

}; // namespace system
}; // namespace qmmf
