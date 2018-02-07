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

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/inc/qmmf_audio_endpoint.h"
#include "common/codecadaptor/src/qmmf_avcodec.h"
#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_recorder_ion.h"

namespace qmmf {
namespace recorder {

class IAudioTrackSource {
 public:
  virtual ~IAudioTrackSource() {}

  virtual status_t Init() = 0;
  virtual status_t DeInit() = 0;

  virtual status_t StartTrack() = 0;
  virtual status_t StopTrack() = 0;
  virtual status_t PauseTrack() = 0;
  virtual status_t ResumeTrack() = 0;

  virtual status_t ReturnTrackBuffer(const std::vector<BnBuffer>& buffers) = 0;
};

class AudioRawTrackSource : public IAudioTrackSource {
 public:
  AudioRawTrackSource(const AudioTrackParams& params);
  virtual ~AudioRawTrackSource();

  // methods of IAudioTrackSource
  status_t Init() override;
  status_t DeInit() override;
  status_t StartTrack() override;
  status_t StopTrack() override;
  status_t PauseTrack() override;
  status_t ResumeTrack() override;
  status_t ReturnTrackBuffer(const std::vector<BnBuffer>& buffers) override;

 private:
  enum class AudioMessageType {
    kMessageStop,
    kMessagePause,
    kMessageResume,
    kMessageBuffer,
    kMessageBnBuffer,
  };

  struct AudioMessage {
    AudioMessageType type;
    union {
      ::qmmf::common::audio::AudioBuffer buffer;
      BnBuffer bn_buffer;
    };
  };

  static void ThreadEntry(AudioRawTrackSource* source);
  void Thread();

  void ErrorHandler(const int32_t error);
  void BufferHandler(const ::qmmf::common::audio::AudioBuffer& buffer);

  AudioTrackParams track_params_;
  ::qmmf::common::audio::AudioEndPoint* end_point_;
  RecorderIon ion_;

  ::std::thread* thread_;
  ::std::mutex message_lock_;
  ::std::queue<AudioMessage> messages_;
  ::std::condition_variable signal_;

  // disable copy, assignment, and move
  AudioRawTrackSource(const AudioRawTrackSource&) = delete;
  AudioRawTrackSource(AudioRawTrackSource&&) = delete;
  AudioRawTrackSource& operator=(const AudioRawTrackSource&) = delete;
  AudioRawTrackSource& operator=(const AudioRawTrackSource&&) = delete;
};

// This class is behaves as both producer and consumer. At one end, it takes
// PCM buffers from the audio endpoint; and on the other end, it provides those
// PCM buffers to Encoder. It also manages buffer circulation, skip, etc.
class AudioEncodedTrackSource : public ::qmmf::avcodec::ICodecSource,
                                public IAudioTrackSource {
 public:
  AudioEncodedTrackSource(const AudioTrackParams& params);
  virtual ~AudioEncodedTrackSource();

  // methods of IAudioTrackSource
  status_t Init() override;
  status_t DeInit() override;
  status_t StartTrack() override;
  status_t StopTrack() override;
  status_t PauseTrack() override;
  status_t ResumeTrack() override;
  status_t ReturnTrackBuffer(const std::vector<BnBuffer>& buffers) override;

  // methods of IInputCodecSource
  status_t GetBuffer(BufferDescriptor& buffer, void* client_data) override;
  status_t ReturnBuffer(BufferDescriptor& buffer, void* client_data) override;
  status_t NotifyPortEvent(::qmmf::avcodec::PortEventType event_type,
                           void* event_data) override;

  status_t GetBufferSize(int32_t* buffer_size);
  status_t SetBufferSize(const int32_t buffer_size);

 private:
  void ErrorHandler(const int32_t error);
  void BufferHandler(const ::qmmf::common::audio::AudioBuffer& buffer);

  AudioTrackParams track_params_;
  ::qmmf::common::audio::AudioEndPoint* end_point_;
  RecorderIon ion_;
  ::std::queue<BufferDescriptor> buffers_;
  int32_t buffer_size_;
  bool stop_called_;
  bool stop_notify_received_;

  ::std::mutex mutex_;
  ::std::condition_variable signal_;

  // disable copy, assignment, and move
  AudioEncodedTrackSource(const AudioEncodedTrackSource&) = delete;
  AudioEncodedTrackSource(AudioEncodedTrackSource&&) = delete;
  AudioEncodedTrackSource& operator=(const AudioEncodedTrackSource&) = delete;
  AudioEncodedTrackSource& operator=(const AudioEncodedTrackSource&&) = delete;
};

}; // namespace recorder
}; // namespace qmmf
