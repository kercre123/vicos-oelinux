/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#include <unistd.h>

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/inc/qmmf_audio_endpoint.h"
#include "player/src/service/qmmf_player_common.h"
#include "player/src/service/qmmf_player_ion.h"

namespace qmmf {
namespace player {

using namespace android;

class AudioRawTrackSink;

class AudioRawSink {
 public:
  static AudioRawSink* CreateAudioRawSink();

  virtual ~AudioRawSink();

  status_t CreateTrackSink(uint32_t track_id,
                           AudioTrackParams& param,
                           TrackCb& callback);
  status_t DeleteTrackSink(uint32_t track_id);

  status_t StartTrackSink(uint32_t track_id);
  status_t StopTrackSink(uint32_t track_id);
  status_t PauseTrackSink(uint32_t track_id);
  status_t ResumeTrackSink(uint32_t track_id);

  status_t SetAudioTrackSinkParams(uint32_t track_id,
                                   CodecParamType param_type,
                                   void* param,
                                   uint32_t param_size);

  status_t DequeueTrackInputBuffer(uint32_t track_id,
                                   ::std::vector<AVCodecBuffer>& buffers);
  status_t QueueTrackInputBuffer(uint32_t track_id,
                                 ::std::vector<AVCodecBuffer>& buffers);

 private:
  // map of track_id and TrackSink
  typedef ::std::map<uint32_t, ::std::shared_ptr<AudioRawTrackSink>>
          AudioTrackSinkMap;

  static AudioRawSink* instance_;

  AudioTrackSinkMap track_sink_map_;

  // disable default, copy, assignment, and move
  AudioRawSink();
  AudioRawSink(const AudioRawSink&) = delete;
  AudioRawSink(AudioRawSink&&) = delete;
  AudioRawSink& operator=(const AudioRawSink&) = delete;
  AudioRawSink& operator=(const AudioRawSink&&) = delete;
};

class AudioRawTrackSink {
 public:
  AudioRawTrackSink();
  virtual ~AudioRawTrackSink();

  status_t Init(const AudioTrackParams& params, TrackCb& callback);
  status_t DeInit();

  status_t StartSink();
  status_t StopSink();
  status_t PauseSink();
  status_t ResumeSink();

  status_t SetAudioSinkParams(CodecParamType param_type,
                              void* param,
                              uint32_t param_size);

  status_t DequeueInputBuffer(::std::vector<AVCodecBuffer>& buffers);
  status_t QueueInputBuffer(::std::vector<AVCodecBuffer>& buffers);

 private:
  enum class AudioMessageType {
    kMessageStop,
    kMessagePause,
    kMessageResume,
    kMessageBuffer,
    kMessageAVBuffer,
  };

  struct AudioMessage {
    AudioMessageType type;
    union {
      ::qmmf::common::audio::AudioBuffer buffer;
      AVCodecBuffer av_buffer;
    };
  };

  static void ThreadEntry(AudioRawTrackSink* sink);
  void Thread();

  static void PtsThreadEntry(AudioRawTrackSink* sink);
  void PtsThread();

  void ErrorHandler(const int32_t error);
  void BufferHandler(const ::qmmf::common::audio::AudioBuffer& buffer);
  void StoppedHandler();

  AudioTrackParams track_params_;
  TrackCb callback_;
  ::std::mutex av_buffers_lock_;
  ::std::condition_variable buffer_signal_;
  ::std::queue<AVCodecBuffer> av_buffers_;
  ::qmmf::common::audio::AudioEndPoint* end_point_;
  PlayerIon ion_;

  ::std::thread* thread_;
  ::std::mutex message_lock_;
  ::std::queue<AudioMessage> messages_;
  ::std::condition_variable signal_;

  ::std::thread* pts_thread_;
  ::std::mutex pts_message_lock_;
  ::std::queue<AudioMessage> pts_messages_;

  // disable copy, assignment, and move
  AudioRawTrackSink(const AudioRawTrackSink&) = delete;
  AudioRawTrackSink(AudioRawTrackSink&&) = delete;
  AudioRawTrackSink& operator=(const AudioRawTrackSink&) = delete;
  AudioRawTrackSink& operator=(const AudioRawTrackSink&&) = delete;
};

};  // namespace player
};  // namespace qmmf
