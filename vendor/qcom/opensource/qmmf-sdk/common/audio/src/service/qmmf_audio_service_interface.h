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
#include <vector>

#include <binder/IBinder.h>
#include <binder/IInterface.h>
#include <binder/Parcel.h>
#include <utils/Errors.h>
#include <utils/RefBase.h>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/src/service/qmmf_audio_common.h"

namespace qmmf {
namespace common {
namespace audio {

enum class AudioServiceCommand {
  kAudioConnect = ::android::IBinder::FIRST_CALL_TRANSACTION,
  kAudioDisconnect,
  kAudioConfigure,
  kAudioStart,
  kAudioStop,
  kAudioPause,
  kAudioResume,
  kAudioSendBuffers,
  kAudioGetLatency,
  kAudioGetBufferSize,
  kAudioSetParam,
  kGetRenderedPosition,
};

enum class AudioServiceCallbackCommand {
  kAudioNotifyError = ::android::IBinder::FIRST_CALL_TRANSACTION,
  kAudioNotifyBuffer,
};

#define QMMF_AUDIO_SERVICE_NAME "qmmf_audio.service"

// Binder interface for callbacks from AudioService to AudioEndPointClient
class IAudioServiceCallback : public ::android::IInterface {
 public:
  DECLARE_META_INTERFACE(AudioServiceCallback);

  virtual void NotifyErrorEvent(const int32_t error) = 0;
  virtual void NotifyBufferEvent(const AudioBuffer& buffer) = 0;
};

class IAudioService : public ::android::IInterface {
 public:
  DECLARE_META_INTERFACE(AudioService);

  virtual int32_t Connect(
      const ::android::sp<IAudioServiceCallback>& client_handler,
      AudioHandle* audio_handle) = 0;
  virtual int32_t Disconnect(const AudioHandle audio_handle) = 0;
  virtual int32_t Configure(const AudioHandle audio_handle,
                            const AudioEndPointType type,
                            const ::std::vector<DeviceId>& devices,
                            const AudioMetadata& metadata) = 0;

  virtual int32_t Start(const AudioHandle audio_handle) = 0;
  virtual int32_t Stop(const AudioHandle audio_handle, const bool flush) = 0;
  virtual int32_t Pause(const AudioHandle audio_handle) = 0;
  virtual int32_t Resume(const AudioHandle audio_handle) = 0;

  virtual int32_t SendBuffers(const AudioHandle audio_handle,
                              const ::std::vector<AudioBuffer>& buffers) = 0;

  virtual int32_t GetLatency(const AudioHandle audio_handle,
                             int32_t* latency) = 0;
  virtual int32_t GetBufferSize(const AudioHandle audio_handle,
                                int32_t* buffer_size) = 0;
  virtual int32_t SetParam(const AudioHandle audio_handle,
                           const AudioParamType type,
                           const AudioParamData& data) = 0;
  virtual int32_t GetRenderedPosition(const AudioHandle audio_handle,
                                      uint32_t* frames, uint64_t* time) = 0;
};

// this class is responsible to provide callbacks from audio service
class BnAudioServiceCallback
    : public ::android::BnInterface<IAudioServiceCallback> {
 public:
  virtual int32_t onTransact(uint32_t code, const ::android::Parcel &data,
                             ::android::Parcel *reply, uint32_t flags = 0);
};

}; // namespace audio
}; // namespace common
}; // namespace qmmf
