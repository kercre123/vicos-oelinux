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
#include <mutex>
#include <vector>

#include <binder/IBinder.h>
#include <utils/RefBase.h>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/src/service/qmmf_audio_service_interface.h"
#include "common/audio/src/service/qmmf_audio_common.h"
#include "common/qmmf_log.h"

namespace qmmf {
namespace common {
namespace audio {

class AudioEndPointClient {
 public:
  AudioEndPointClient();
  ~AudioEndPointClient();

  int32_t Connect(const AudioEventHandler& handler);
  int32_t Disconnect();
  int32_t Configure(const AudioEndPointType type,
                    const ::std::vector<DeviceId>& devices,
                    const AudioMetadata& metadata);

  int32_t Start();
  int32_t Stop(const bool flush);
  int32_t Pause();
  int32_t Resume();

  int32_t SendBuffers(const ::std::vector<AudioBuffer>& buffers);

  int32_t GetLatency(int32_t* latency);
  int32_t GetBufferSize(int32_t* buffer_size);
  int32_t SetParam(const AudioParamType type, const AudioParamData& data);
  int32_t GetRenderedPosition(uint32_t* frames, uint64_t* time);

  // callbacks from service
  void NotifyErrorEvent(const int32_t error);
  void NotifyBufferEvent(const AudioBuffer& buffer);

 private:
  class DeathNotifier : public ::android::IBinder::DeathRecipient {
   public:
    DeathNotifier(AudioEndPointClient* parent) : parent_(parent) {}

    void binderDied(const ::android::wp<::android::IBinder>&) override {
      QMMF_WARN("%s() audio service died", __func__);

      ::std::lock_guard<::std::mutex> lock(parent_->lock_);
      parent_->audio_service_.clear();
      parent_->audio_service_ = nullptr;
    }

    AudioEndPointClient* parent_;
  };
  friend class DeathNotifier;

  AudioState state_;
  ::std::mutex lock_;
  ::android::sp<IAudioService> audio_service_;
  ::android::sp<DeathNotifier> death_notifier_;
  AudioEventHandler event_handler_;
  AudioHandle audio_handle_;

  // Disable copy, assignment, and move
  AudioEndPointClient(const AudioEndPointClient&) = delete;
  AudioEndPointClient(AudioEndPointClient&&) = delete;
  AudioEndPointClient& operator=(const AudioEndPointClient&) = delete;
  AudioEndPointClient& operator=(const AudioEndPointClient&&) = delete;
};

class ServiceCallbackHandler : public BnAudioServiceCallback {
 public:
  ServiceCallbackHandler(AudioEndPointClient* client);
  ~ServiceCallbackHandler();

 private:
  // methods of BnAudioServiceCallback
  void NotifyErrorEvent(const int32_t error);
  void NotifyBufferEvent(const AudioBuffer& buffer);

  AudioEndPointClient *client_;
};

}; // namespace audio
}; // namespace common
}; // namespace qmmf
