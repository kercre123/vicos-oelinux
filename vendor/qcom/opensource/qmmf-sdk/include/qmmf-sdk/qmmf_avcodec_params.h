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

#include <cstdint>
#include <iomanip>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "qmmf-sdk/qmmf_buffer.h"
#include "qmmf-sdk/qmmf_codec.h"
#include "qmmf-sdk/qmmf_player_params.h"
#include "qmmf-sdk/qmmf_recorder_params.h"

#define INPUT_MAX_COUNT         (11)
#define OUTPUT_MAX_COUNT        (6)
#define CMD_BUF_MAX_COUNT       (10)

namespace qmmf {
namespace avcodec {

typedef int32_t status_t;

static const uint32_t kPortIndexInput = 0;
static const uint32_t kPortIndexOutput = 1;
static const uint32_t kPortALL = 0xFFFFFFFF;
#define PORT_NAME(port)                       \
  (port == kPortIndexInput ? "IN_PORT" :      \
  (port == kPortIndexOutput ? "OUT_PORT" :    \
  "ALL_PORT"))

typedef struct PortreconfigData {

  enum class PortReconfigType {
    kCropParametersChanged,
    kBufferRequirementsChanged,
  }reconfig_type;

  typedef struct CropData{
    uint32_t left;
    uint32_t top;
    uint32_t width;
    uint32_t height;
  }CropData;
  CropData rect;

  typedef struct BufferRequirements {
    uint32_t buf_count;
    uint32_t buf_size;
  }BufferRequirements;
  BufferRequirements buf_reqs;

}PortreconfigData;

// AVCodec will notify input port status to Codec source
enum class CodecPortStatus {
  kPortStart,
  // notify when codec receives EOS from track source
  kPortStop,
  // notify when codec returns all buffers to track source
  kPortIdle,
};

enum class PortEventType {
  // AVcodec will notify port status
  kPortStatus,
  // notify when there is a Port Reconfig Event
  kPortSettingsChanged,
};

union CodecParam {
  ::qmmf::recorder::VideoTrackCreateParam video_enc_param;
  ::qmmf::recorder::AudioTrackCreateParam audio_enc_param;
  ::qmmf::player::AudioTrackCreateParam   audio_dec_param;
  ::qmmf::player::VideoTrackCreateParam   video_dec_param;

  // needed for unions with non-trivial members
  CodecParam() : video_enc_param() {}
  CodecParam(const ::qmmf::recorder::VideoTrackCreateParam& _param)
      : video_enc_param(_param) {}
  CodecParam(const ::qmmf::recorder::AudioTrackCreateParam& _param)
      : audio_enc_param(_param) {}
  CodecParam(const ::qmmf::player::AudioTrackCreateParam& _param)
      : audio_dec_param(_param) {}
  CodecParam(const ::qmmf::player::VideoTrackCreateParam& _param)
      : video_dec_param(_param) {}
  ~CodecParam() {}
};

}; // namespace avcodec
}; // namespace qmmf
