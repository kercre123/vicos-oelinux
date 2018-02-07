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

#include <vector>

#include "common/audio/inc/qmmf_audio_definitions.h"

namespace qmmf {
namespace common {
namespace audio {

class IAudioBackend {
 public:
  virtual ~IAudioBackend() {}

  virtual int32_t Open(const ::std::vector<DeviceId>& devices,
                       const AudioMetadata& metadata) = 0;
  virtual int32_t Close() = 0;

  virtual int32_t Start() = 0;
  virtual int32_t Stop(const bool flush) = 0;
  virtual int32_t Pause() = 0;
  virtual int32_t Resume() = 0;

  virtual int32_t SendBuffers(const ::std::vector<AudioBuffer>& buffers) = 0;

  virtual int32_t GetLatency(int32_t* latency) = 0;
  virtual int32_t GetBufferSize(int32_t* buffer_size) = 0;
  virtual int32_t SetParam(const AudioParamType type,
                           const AudioParamData& data) = 0;
  virtual int32_t GetRenderedPosition(uint32_t* frames,
                                      uint64_t* time) = 0;
};

}; // namespace audio
}; // namespace common
}; // namespace qmmf
