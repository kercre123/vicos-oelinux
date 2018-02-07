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

#define TAG "AudioService"

#include "common/audio/src/service/qmmf_audio_service.h"

#include <cerrno>
#include <cstdint>
#include <map>
#include <mutex>
#include <vector>

#include <binder/IInterface.h>
#include <binder/Parcel.h>
#include <utils/RefBase.h>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/src/service/qmmf_audio_common.h"
#include "common/audio/src/service/qmmf_audio_frontend.h"
#include "common/audio/src/service/qmmf_audio_ion.h"
#include "common/qmmf_log.h"

namespace qmmf {
namespace common {
namespace audio {

using ::android::IInterface;
using ::android::interface_cast;
using ::android::Parcel;
using ::android::sp;
using ::std::lock_guard;
using ::std::map;
using ::std::mutex;
using ::std::vector;

AudioService::AudioService() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  AudioErrorHandler error_handler =
    [this](const AudioHandle audio_handle, const int32_t error) -> void {
      ClientHandlerMap::iterator client_handler_iterator =
          client_handlers_.find(audio_handle);
      if (client_handler_iterator == client_handlers_.end()) {
        QMMF_ERROR("%s: %s() no client handler for key[%d]", TAG, __func__,
                   audio_handle);
        return;
      }

      client_handler_iterator->second->NotifyErrorEvent(error);
    };

  AudioBufferHandler buffer_handler =
    [this](const AudioHandle audio_handle, const AudioBuffer& buffer) -> void {
      ClientHandlerMap::iterator client_handler_iterator =
          client_handlers_.find(audio_handle);
      if (client_handler_iterator == client_handlers_.end()) {
        QMMF_ERROR("%s: %s() no client handler for key[%d]", TAG, __func__,
                   audio_handle);
        return;
      }

      client_handler_iterator->second->NotifyBufferEvent(buffer);
    };

  audio_frontend_.RegisterErrorHandler(error_handler);
  audio_frontend_.RegisterBufferHandler(buffer_handler);

  QMMF_INFO("%s: %s() service instantiated", TAG, __func__);
}

AudioService::~AudioService()
{
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  death_notifiers_.clear();
  client_handlers_.clear();
  QMMF_INFO("%s: %s: service destroyed", TAG, __func__);
}

int32_t AudioService::Connect(const sp<IAudioServiceCallback>& client_handler,
                              AudioHandle* audio_handle) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  lock_guard<mutex> lock(lock_);

  int32_t result = audio_frontend_.Connect(audio_handle);
  if (result < 0) {
    QMMF_ERROR("%s: %s() frontend->Connect failed: %d", TAG, __func__, result);
    return result;
  }

  sp<DeathNotifier> death_notifier = new DeathNotifier(this, *audio_handle);
  if (death_notifier.get() == nullptr) {
    QMMF_ERROR("%s: %s() unable to allocate death notifier", TAG, __func__);
    return -ENOMEM;
  }
  IInterface::asBinder(client_handler)->linkToDeath(death_notifier);

  death_notifiers_.insert({*audio_handle, death_notifier});
  client_handlers_.insert({*audio_handle, client_handler});

  QMMF_VERBOSE("%s: %s() OUTPARAM: audio_handle[%d]", TAG, __func__,
               *audio_handle);
  return result;
}

int32_t AudioService::Disconnect(const AudioHandle audio_handle) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
               audio_handle);
  lock_guard<mutex> lock(lock_);

  int32_t result = audio_frontend_.Disconnect(audio_handle);
  if (result < 0) {
    QMMF_ERROR("%s: %s() frontend->Disconnect failed: %d", TAG, __func__,
               result);
    return result;
  }

  ClientHandlerMap::iterator client_handler_iterator =
      client_handlers_.find(audio_handle);
  if (client_handler_iterator == client_handlers_.end()) {
    QMMF_ERROR("%s: %s() no client handler for key[%d]", TAG, __func__,
               audio_handle);
    return -EINVAL;
  }

  DeathNotifierMap::iterator death_notifier_iterator =
      death_notifiers_.find(audio_handle);
  if (death_notifier_iterator == death_notifiers_.end()) {
    QMMF_ERROR("%s: %s() no death notifier for key[%d]", TAG, __func__,
               audio_handle);
    return -EINVAL;
  }

  IInterface::asBinder(client_handler_iterator->second)->
      unlinkToDeath(death_notifier_iterator->second);
  client_handler_iterator->second.clear();
  death_notifier_iterator->second.clear();
  client_handlers_.erase(client_handler_iterator);
  death_notifiers_.erase(death_notifier_iterator);

  ion_.Release(audio_handle);

  return 0;
}

int32_t AudioService::Configure(const AudioHandle audio_handle,
                                const AudioEndPointType type,
                                const vector<DeviceId>& devices,
                                const AudioMetadata& metadata) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
               audio_handle);
  QMMF_VERBOSE("%s: %s() INPARAM: type[%d]", TAG, __func__,
               static_cast<int>(type));
  for (const DeviceId device : devices)
    QMMF_VERBOSE("%s: %s() INPARAM: device[%d]", TAG, __func__, device);
  QMMF_VERBOSE("%s: %s() INPARAM: metadata[%s]", TAG, __func__,
               metadata.ToString().c_str());
  lock_guard<mutex> lock(lock_);

  int32_t result = audio_frontend_.Configure(audio_handle, type, devices,
                                             metadata);
  if (result < 0)
    QMMF_ERROR("%s: %s() frontend->Configure failed: %d", TAG, __func__,
               result);

  return result;
}

int32_t AudioService::Start(const AudioHandle audio_handle) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
               audio_handle);
  lock_guard<mutex> lock(lock_);

  int32_t result = audio_frontend_.Start(audio_handle);
  if (result < 0)
    QMMF_ERROR("%s: %s() frontend->Start failed: %d", TAG, __func__, result);

  return result;
}

int32_t AudioService::Stop(const AudioHandle audio_handle, const bool flush) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
               audio_handle);
  QMMF_VERBOSE("%s: %s() INPARAM: flush[%s]", TAG, __func__,
               flush ? "true" : "false");
  lock_guard<mutex> lock(lock_);

  int32_t result = audio_frontend_.Stop(audio_handle, flush);
  if (result < 0)
    QMMF_ERROR("%s: %s() frontend->Stop failed: %d", TAG, __func__, result);

  return result;
}

int32_t AudioService::Pause(const AudioHandle audio_handle) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
               audio_handle);
  lock_guard<mutex> lock(lock_);

  int result = audio_frontend_.Pause(audio_handle);
  if (result < 0)
    QMMF_ERROR("%s: %s() frontend->Pause failed: %d", TAG, __func__, result);

  return result;
}

int32_t AudioService::Resume(const AudioHandle audio_handle) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
               audio_handle);
  lock_guard<mutex> lock(lock_);

  int32_t result = audio_frontend_.Resume(audio_handle);
  if (result < 0)
    QMMF_ERROR("%s: %s() frontend->Resume failed: %d", TAG, __func__, result);

  return result;
}

int32_t AudioService::SendBuffers(const AudioHandle audio_handle,
                                  const vector<AudioBuffer>& buffers) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
               audio_handle);
  for (const AudioBuffer& buffer : buffers)
    QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
                 buffer.ToString().c_str());

  int32_t result = audio_frontend_.SendBuffers(audio_handle, buffers);
  if (result < 0)
    QMMF_ERROR("%s: %s() frontend->SendBuffers failed: %d", TAG, __func__,
               result);

  return result;
}

int32_t AudioService::GetLatency(const AudioHandle audio_handle,
                                 int32_t* latency) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
               audio_handle);
  lock_guard<mutex> lock(lock_);

  int32_t result = audio_frontend_.GetLatency(audio_handle, latency);
  if (result < 0)
    QMMF_ERROR("%s: %s() frontend->GetLatency failed: %d", TAG, __func__,
               result);

  QMMF_VERBOSE("%s: %s() OUTPARAM: latency[%d]", TAG, __func__, *latency);
  return result;
}

int32_t AudioService::GetBufferSize(const AudioHandle audio_handle,
                                    int32_t* buffer_size) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
               audio_handle);
  lock_guard<mutex> lock(lock_);

  int32_t result = audio_frontend_.GetBufferSize(audio_handle, buffer_size);
  if (result < 0)
    QMMF_ERROR("%s: %s() frontend->GetBufferSize failed: %d", TAG, __func__,
               result);

  QMMF_VERBOSE("%s: %s() OUTPARAM: buffer_size[%d]", TAG, __func__,
               *buffer_size);
  return result;
}

int32_t AudioService::SetParam(const AudioHandle audio_handle,
                               const AudioParamType type,
                               const AudioParamData& data) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
               audio_handle);
  QMMF_VERBOSE("%s: %s() INPARAM: type[%d]", TAG, __func__,
               static_cast<int>(type));
  QMMF_VERBOSE("%s: %s() INPARAM: data[%s]", TAG, __func__,
               data.ToString(type).c_str());
  lock_guard<mutex> lock(lock_);

  int32_t result = audio_frontend_.SetParam(audio_handle, type, data);
  if (result < 0)
    QMMF_ERROR("%s: %s() frontend->SetParam failed: %d", TAG, __func__, result);

  return result;
}

int32_t AudioService::GetRenderedPosition(const AudioHandle audio_handle,
                                          uint32_t* frames, uint64_t* time) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
               audio_handle);
  lock_guard<mutex> lock(lock_);

  int32_t result = audio_frontend_.GetRenderedPosition(audio_handle, frames, time);
  if (result < 0) {
    QMMF_ERROR("%s: %s() frontend->GetRenderedPosition failed: %d", TAG, __func__,
        result);
  }

  QMMF_VERBOSE("%s: %s() OUTPARAM: Frames[%u] Time[%llu]", TAG, __func__,
      *frames, *time);
  return result;
}


int32_t AudioService::onTransact(uint32_t code, const Parcel& input,
                                 Parcel* output, uint32_t flags) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: code[%u]", TAG, __func__, code);
  QMMF_VERBOSE("%s: %s() INPARAM: flags[%u]", TAG, __func__, flags);

  if (!input.checkInterface(this))
    return -EPERM;

  switch (static_cast<AudioServiceCommand>(code)) {
    case AudioServiceCommand::kAudioConnect: {
      sp<IAudioServiceCallback> client_handler =
          interface_cast<IAudioServiceCallback>(input.readStrongBinder());

      QMMF_DEBUG("%s: %s-AudioConnect() TRACE", TAG, __func__);
      AudioHandle audio_handle;
      int32_t result = Connect(client_handler, &audio_handle);
      QMMF_VERBOSE("%s: %s-AudioConnect() OUTPARAM: audio_handle[%d]", TAG,
                   __func__, audio_handle);

      output->writeInt32(static_cast<int32_t>(audio_handle));
      output->writeInt32(result);
      break;
    }

    case AudioServiceCommand::kAudioDisconnect: {
      AudioHandle audio_handle = static_cast<AudioHandle>(input.readInt32());

      QMMF_DEBUG("%s: %s-AudioDisconnect() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-AudioDisconnect() INPARAM: audio_handle[%d]", TAG,
                   __func__, audio_handle);
      int32_t result = Disconnect(audio_handle);

      output->writeInt32(result);
      break;
    }

    case AudioServiceCommand::kAudioConfigure: {
      AudioHandle audio_handle = static_cast<AudioHandle>(input.readInt32());
      AudioEndPointType type =
          static_cast<AudioEndPointType>(input.readInt32());

      vector<DeviceId> devices;
      size_t number_of_devices = static_cast<size_t>(input.readUint32());
      for (size_t index = 0; index < number_of_devices; ++index)
        devices.push_back(static_cast<DeviceId>(input.readInt32()));

      AudioMetadata metadata;
      metadata.FromParcel(input);

      QMMF_DEBUG("%s: %s-AudioConfigure() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-AudioConfigure() INPARAM: audio_handle[%d]", TAG,
                   __func__, audio_handle);
      QMMF_VERBOSE("%s: %s-AudioConfigure() INPARAM: type[%d]", TAG, __func__,
                   static_cast<int>(type));
      for (const DeviceId device : devices)
        QMMF_VERBOSE("%s: %s() INPARAM: device[%d]", TAG, __func__, device);
      QMMF_VERBOSE("%s: %s-AudioConfigure() INPARAM: metadata[%s]", TAG,
                   __func__, metadata.ToString().c_str());
      int32_t result = Configure(audio_handle, type, devices, metadata);

      output->writeInt32(result);
      break;
    }

    case AudioServiceCommand::kAudioStart: {
      AudioHandle audio_handle = static_cast<AudioHandle>(input.readInt32());

      QMMF_DEBUG("%s: %s-AudioStart() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-AudioStart() INPARAM: audio_handle[%d]", TAG,
                   __func__, audio_handle);
      int32_t result = Start(audio_handle);

      output->writeInt32(result);
      break;
    }

    case AudioServiceCommand::kAudioStop: {
      AudioHandle audio_handle = static_cast<AudioHandle>(input.readInt32());
      bool flush = static_cast<bool>(input.readInt32());

      QMMF_DEBUG("%s: %s-AudioStop() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-AudioStop() INPARAM: audio_handle[%d]", TAG,
                   __func__, audio_handle);
      QMMF_VERBOSE("%s: %s-AudioStop() INPARAM: flush[%s]", TAG, __func__,
                   flush ? "true" : "false");
      int32_t result = Stop(audio_handle, flush);

      output->writeInt32(result);
      break;
    }

    case AudioServiceCommand::kAudioPause: {
      AudioHandle audio_handle = static_cast<AudioHandle>(input.readInt32());

      QMMF_DEBUG("%s: %s-AudioPause() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-AudioPause() INPARAM: audio_handle[%d]", TAG,
                   __func__, audio_handle);
      int32_t result = Pause(audio_handle);

      output->writeInt32(result);
      break;
    }

    case AudioServiceCommand::kAudioResume: {
      AudioHandle audio_handle = static_cast<AudioHandle>(input.readInt32());

      QMMF_DEBUG("%s: %s-AudioResume() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-AudioResume() INPARAM: audio_handle[%d]", TAG,
                   __func__, audio_handle);
      int32_t result = Resume(audio_handle);

      output->writeInt32(result);
      break;
    }

    case AudioServiceCommand::kAudioSendBuffers: {
      AudioHandle audio_handle = static_cast<AudioHandle>(input.readInt32());

      vector<AudioBuffer> buffers;
      size_t number_of_buffers = static_cast<size_t>(input.readUint32());
      for (size_t index = 0; index < number_of_buffers; ++index) {
        AudioBuffer buffer;
        buffer.FromParcel(input, true);
        buffers.push_back(buffer);
      }

      for (AudioBuffer& buffer : buffers) {
        if (buffer.ion_fd != -1)
          ion_.Associate(audio_handle, &buffer);
      }

      QMMF_DEBUG("%s: %s-AudioSendBuffers() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-AudioSendBuffers() INPARAM: audio_handle[%d]", TAG,
                   __func__, audio_handle);
      for (const AudioBuffer& buffer : buffers)
        QMMF_VERBOSE("%s: %s-AudioSendBuffers() INPARAM: buffer[%s]", TAG,
                     __func__, buffer.ToString().c_str());
      int32_t result = SendBuffers(audio_handle, buffers);

      output->writeInt32(result);
      break;
    }

    case AudioServiceCommand::kAudioGetLatency: {
      AudioHandle audio_handle = static_cast<AudioHandle>(input.readInt32());

      QMMF_DEBUG("%s: %s-AudioGetLatency() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-AudioGetLatency() INPARAM: audio_handle[%d]", TAG,
                   __func__, audio_handle);
      int32_t latency;
      int32_t result = GetLatency(audio_handle, &latency);
      QMMF_VERBOSE("%s: %s-AudioGetLatency() OUTPARAM: latency[%d]", TAG,
                   __func__, latency);

      output->writeInt32(latency);
      output->writeInt32(result);
      break;
    }

    case AudioServiceCommand::kAudioGetBufferSize: {
      AudioHandle audio_handle = static_cast<AudioHandle>(input.readInt32());

      QMMF_DEBUG("%s: %s-AudioGetBufferSize() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-AudioGetBufferSize() INPARAM: audio_handle[%d]", TAG,
                   __func__, audio_handle);
      int32_t buffer_size;
      int32_t result = GetBufferSize(audio_handle, &buffer_size);
      QMMF_VERBOSE("%s: %s-AudioGetBufferSize() OUTPARAM: buffer_size[%d]", TAG,
                   __func__, buffer_size);

      output->writeInt32(buffer_size);
      output->writeInt32(result);
      break;
    }

    case AudioServiceCommand::kAudioSetParam: {
      AudioHandle audio_handle = static_cast<AudioHandle>(input.readInt32());

      AudioParamType type = static_cast<AudioParamType>(input.readInt32());

      AudioParamData data;
      data.FromParcel(input);

      QMMF_DEBUG("%s: %s-AudioSetParam() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-AudioSetParam() INPARAM: audio_handle[%d]", TAG,
                   __func__, audio_handle);
      QMMF_VERBOSE("%s: %s-AudioSetParam() INPARAM: type[%d]", TAG, __func__,
                   static_cast<int>(type));
      QMMF_VERBOSE("%s: %s-AudioSetParam() INPARAM: data[%s]", TAG, __func__,
                   data.ToString(type).c_str());
      int32_t result = SetParam(audio_handle, type, data);

      output->writeInt32(result);
      break;
    }

    case AudioServiceCommand::kGetRenderedPosition: {
      AudioHandle audio_handle = static_cast<AudioHandle>(input.readInt32());

      QMMF_DEBUG("%s: %s-AudioGetRenderedPosition() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-AudioGetRenderedPosition() INPARAM: audio_handle[%d]",
          TAG,__func__, audio_handle);
      uint32_t frames;
      uint64_t time;
      int32_t result = GetRenderedPosition(audio_handle, &frames, &time);
      QMMF_VERBOSE("%s: %s-GetRenderedPosition() OUTPARAM: frames[%u] time[%llu]",
          TAG,__func__, frames, time);

      output->writeUint32(frames);
      output->writeUint64(time);
      output->writeInt32(result);
      break;
    }

    default:
      QMMF_ERROR("%s: %s() code %u not supported ", TAG, __func__, code);
      output->writeInt32(static_cast<int32_t>(-EINVAL));
      break;
  }
  return 0;
}

}; // namespace audio
}; // namespace common
}; // namespace qmmf
