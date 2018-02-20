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

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <mm-audio/qahw_api/inc/qahw_api.h>
#include <mm-audio/qahw_api/inc/qahw_defs.h>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/src/service/qmmf_audio_backend.h"
#include "common/audio/src/service/qmmf_audio_common.h"

namespace qmmf {
namespace common {
namespace audio {

class AudioBackendSink : public IAudioBackend {
 public:
  AudioBackendSink(const AudioHandle audio_handle,
                   const AudioErrorHandler& error_handler,
                   const AudioBufferHandler& buffer_handler,
                   const AudioStoppedHandler& stopped_handler);
  ~AudioBackendSink();

  int32_t Open(const qahw_module_handle_t * const modules[],
               const ::std::vector<DeviceId>& devices,
               const AudioMetadata& metadata);
  int32_t Close();

  int32_t Start();
  int32_t Stop();
  int32_t Pause();
  int32_t Resume();

  int32_t SendBuffers(const ::std::vector<AudioBuffer>& buffers);

  int32_t GetLatency(int32_t* latency);
  int32_t GetBufferSize(int32_t* buffer_size);
  int32_t SetParam(const AudioParamType type,
                   const AudioParamData& data);
  int32_t GetRenderedPosition(uint32_t* frames,
                              uint64_t* time);

 private:
  enum class AudioMessageType {
    kMessageStop,
    kMessagePause,
    kMessageResume,
    kMessageBuffer,
    kMessageWriteDone,
    kMessageFlushDone,
  };

  struct AudioMessage {
    AudioMessageType type;
    ::std::vector<AudioBuffer> buffers;
  };

  static const audio_io_handle_t kIOHandleMin;
  static const audio_io_handle_t kIOHandleMax;

  static void ThreadEntry(AudioBackendSink* backend);
  void Thread();

  static int CallbackEntry(qahw_stream_callback_event_t event,
                           void* param,
                           void* cookie);
  int Callback(qahw_stream_callback_event_t event, void* param);

  AudioHandle audio_handle_;
  AudioState state_;

  AudioErrorHandler error_handler_;
  AudioBufferHandler buffer_handler_;
  AudioStoppedHandler stopped_handler_;

  ::std::thread* thread_;
  ::std::mutex message_lock_;
  ::std::queue<AudioMessage> messages_;
  ::std::condition_variable signal_;

  bool using_offload_;

  qahw_module_handle_t* qahw_module_;
  qahw_stream_handle_t* qahw_stream_;
  audio_io_handle_t current_io_handle_;

  // disable default, copy, assignment, and move
  AudioBackendSink() = delete;
  AudioBackendSink(const AudioBackendSink&) = delete;
  AudioBackendSink(AudioBackendSink&&) = delete;
  AudioBackendSink& operator=(const AudioBackendSink&) = delete;
  AudioBackendSink& operator=(const AudioBackendSink&&) = delete;
};

}; // namespace audio
}; // namespace common
}; // namespace qmmf
