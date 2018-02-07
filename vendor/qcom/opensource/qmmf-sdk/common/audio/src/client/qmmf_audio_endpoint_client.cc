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

//
// This file has implementation of following classes:
//
// - AudioEndPointClient    : Delegation to binder proxy <IAudioService>
//                            and implementation of binder CB.
// - BpAudioService         : Binder proxy implementation.
// - BpAudioServiceCallback : Binder CB proxy implementation.
// - BnAudioServiceCallback : Binder CB stub implementation.
//

#define TAG "AudioEndPointClient"

#include "common/audio/src/client/qmmf_audio_endpoint_client.h"

#include <cerrno>
#include <cstdint>
#include <mutex>
#include <vector>

#include <binder/IBinder.h>
#include <binder/IInterface.h>
#include <binder/IServiceManager.h>
#include <binder/Parcel.h>
#include <binder/ProcessState.h>
#include <utils/RefBase.h>
#include <utils/String16.h>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/qmmf_device_internal.h"
#include "common/qmmf_log.h"

namespace qmmf {
namespace common {
namespace audio {

using ::android::BpInterface;
using ::android::defaultServiceManager;
using ::android::IBinder;
using ::android::IInterface;
using ::android::interface_cast;
using ::android::IServiceManager;
using ::android::Parcel;
using ::android::ProcessState;
using ::android::sp;
using ::android::String16;
using ::std::lock_guard;
using ::std::mutex;
using ::std::vector;

AudioEndPointClient::AudioEndPointClient()
    : state_(AudioState::kNew),
      audio_service_(nullptr),
      death_notifier_(nullptr),
      audio_handle_(-1) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
             static_cast<int>(state_));

  sp<ProcessState> proc(ProcessState::self());
  proc->startThreadPool();

  QMMF_INFO("%s: %s() client instantiated", TAG, __func__);
}

AudioEndPointClient::~AudioEndPointClient() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  if (audio_service_ != nullptr) {
    audio_service_.clear();
    audio_service_ = nullptr;
  }

  QMMF_INFO("%s: %s() client destroyed", TAG, __func__);
}

int32_t AudioEndPointClient::Connect(const AudioEventHandler& handler) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  lock_guard<mutex> lock(lock_);

  switch (state_) {
    case AudioState::kNew:
      // proceed
      break;
    case AudioState::kConnect:
    case AudioState::kIdle:
    case AudioState::kRunning:
    case AudioState::kPaused:
      QMMF_ERROR("%s: %s() invalid operation for current state: %d", TAG,
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s: %s() unknown state: %d", TAG, __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  if (audio_service_.get() != nullptr) {
    QMMF_ERROR("%s: %s() already connected to service", TAG, __func__);
    return -ENOSYS;
  }

  event_handler_ = handler;

  death_notifier_ = new DeathNotifier(this);
  if (death_notifier_.get() == nullptr) {
    QMMF_ERROR("%s: %s() unable to allocate death notifier", TAG, __func__);
    return -ENOMEM;
  }

  sp<IBinder> service_handle;
  sp<IServiceManager> service_manager = defaultServiceManager();

  service_handle = service_manager->getService(String16(QMMF_AUDIO_SERVICE_NAME));
  if (service_handle.get() == nullptr) {
    QMMF_ERROR("%s: %s() can't get service %s", TAG, __func__,
               QMMF_AUDIO_SERVICE_NAME);
    return -ENODEV;
  }

  audio_service_ = interface_cast<IAudioService>(service_handle);
  IInterface::asBinder(audio_service_)->linkToDeath(death_notifier_);

  sp<ServiceCallbackHandler> cb_handler = new ServiceCallbackHandler(this);
  int32_t result = audio_service_->Connect(cb_handler, &audio_handle_);
  if (result < 0) {
    QMMF_ERROR("%s: %s() can't connect to service %s: %d", TAG, __func__,
               QMMF_AUDIO_SERVICE_NAME, result);
  } else {
    state_ = AudioState::kConnect;
    QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
               static_cast<int>(state_));
  }

  return result;
}

int32_t AudioEndPointClient::Disconnect() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  lock_guard<mutex> lock(lock_);

  switch (state_) {
    case AudioState::kNew:
      QMMF_WARN("%s: %s() nothing to do, state is: %d", TAG, __func__,
                static_cast<int>(state_));
      return 0;
      break;
    case AudioState::kConnect:
    case AudioState::kIdle:
      // proceed
      break;
    case AudioState::kRunning:
    case AudioState::kPaused:
      QMMF_ERROR("%s: %s() invalid operation for current state: %d", TAG,
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s: %s() unknown state: %d", TAG, __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  if (audio_service_.get() == nullptr) {
    QMMF_ERROR("%s: %s() not connected to service", TAG, __func__);
    return -ENOSYS;
  }

  int32_t result = audio_service_->Disconnect(audio_handle_);
  if (result < 0)
    QMMF_ERROR("%s: %s() service->Disconnect failed: %d", TAG, __func__,
               result);

  audio_service_->asBinder(audio_service_)->unlinkToDeath(death_notifier_);
  audio_service_.clear();
  audio_service_ = nullptr;

  death_notifier_.clear();
  death_notifier_ = nullptr;

  audio_handle_ = -1;

  state_ = AudioState::kNew;
  QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
             static_cast<int>(state_));

  return result;
}

int32_t AudioEndPointClient::Configure(const AudioEndPointType type,
                                       const vector<DeviceId>& devices,
                                       const AudioMetadata& metadata) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: type[%d]", TAG, __func__,
               static_cast<int>(type));
  for (const DeviceId device : devices)
    QMMF_VERBOSE("%s: %s() INPARAM: device[%d]", TAG, __func__, device);
  QMMF_VERBOSE("%s: %s() INPARAM: metadata[%s]", TAG, __func__,
               metadata.ToString().c_str());
  lock_guard<mutex> lock(lock_);

  switch (state_) {
    case AudioState::kConnect:
      // proceed
      break;
    case AudioState::kNew:
    case AudioState::kIdle:
    case AudioState::kRunning:
    case AudioState::kPaused:
      QMMF_ERROR("%s: %s() invalid operation for current state: %d", TAG,
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s: %s() unknown state: %d", TAG, __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  if (audio_service_.get() == nullptr) {
    QMMF_ERROR("%s: %s() not connected to service", TAG, __func__);
    return -ENOSYS;
  }

  int32_t result = audio_service_->Configure(audio_handle_, type, devices,
                                             metadata);
  if (result < 0) {
    QMMF_ERROR("%s: %s() service->Configure failed: %d", TAG, __func__, result);
  } else {
    state_ = AudioState::kIdle;
    QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
               static_cast<int>(state_));
  }

  return result;
}

int32_t AudioEndPointClient::Start() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  lock_guard<mutex> lock(lock_);

  switch (state_) {
    case AudioState::kIdle:
      // proceed
      break;
    case AudioState::kNew:
    case AudioState::kConnect:
    case AudioState::kRunning:
    case AudioState::kPaused:
      QMMF_ERROR("%s: %s() invalid operation for current state: %d", TAG,
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s: %s() unknown state: %d", TAG, __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  if (audio_service_.get() == nullptr) {
    QMMF_ERROR("%s: %s() not connected to service", TAG, __func__);
    return -ENOSYS;
  }

  int32_t result = audio_service_->Start(audio_handle_);
  if (result < 0) {
    QMMF_ERROR("%s: %s() service->Start failed: %d", TAG, __func__, result);
  } else {
    state_ = AudioState::kRunning;
    QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
               static_cast<int>(state_));
  }

  return result;
}

int32_t AudioEndPointClient::Stop(const bool flush) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: flush[%s]", TAG, __func__,
               flush ? "true" : "false");
  lock_guard<mutex> lock(lock_);

  switch (state_) {
    case AudioState::kNew:
    case AudioState::kConnect:
    case AudioState::kIdle:
      QMMF_WARN("%s: %s() nothing to do, state is: %d", TAG, __func__,
                static_cast<int>(state_));
      return 0;
      break;
    case AudioState::kRunning:
    case AudioState::kPaused:
      // proceed
      break;
    default:
      QMMF_ERROR("%s: %s() unknown state: %d", TAG, __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  if (audio_service_.get() == nullptr) {
    QMMF_ERROR("%s: %s() not connected to service", TAG, __func__);
    return -ENOSYS;
  }

  int32_t result = audio_service_->Stop(audio_handle_, flush);
  if (result < 0) {
    QMMF_ERROR("%s: %s() service->Stop failed: %d", TAG, __func__, result);
  } else {
    state_ = AudioState::kIdle;
    QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
               static_cast<int>(state_));
  }

  return result;
}

int32_t AudioEndPointClient::Pause() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  lock_guard<mutex> lock(lock_);

  switch (state_) {
    case AudioState::kRunning:
      // proceed
      break;
    case AudioState::kNew:
    case AudioState::kConnect:
    case AudioState::kIdle:
    case AudioState::kPaused:
      QMMF_ERROR("%s: %s() invalid operation for current state: %d", TAG,
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s: %s() unknown state: %d", TAG, __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  if (audio_service_.get() == nullptr) {
    QMMF_ERROR("%s: %s() not connected to service", TAG, __func__);
    return -ENOSYS;
  }

  int32_t result = audio_service_->Pause(audio_handle_);
  if (result < 0) {
    QMMF_ERROR("%s: %s() service->Pause failed: %d", TAG, __func__, result);
  } else {
    state_ = AudioState::kPaused;
    QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
               static_cast<int>(state_));
  }

  return result;
}

int32_t AudioEndPointClient::Resume() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  lock_guard<mutex> lock(lock_);

  switch (state_) {
    case AudioState::kPaused:
      // proceed
      break;
    case AudioState::kNew:
    case AudioState::kConnect:
    case AudioState::kIdle:
    case AudioState::kRunning:
      QMMF_ERROR("%s: %s() invalid operation for current state: %d", TAG,
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s: %s() unknown state: %d", TAG, __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  if (audio_service_.get() == nullptr) {
    QMMF_ERROR("%s: %s() not connected to service", TAG, __func__);
    return -ENOSYS;
  }

  int32_t result = audio_service_->Resume(audio_handle_);
  if (result < 0) {
    QMMF_ERROR("%s: %s() service->Resume failed: %d", TAG, __func__, result);
  } else {
    state_ = AudioState::kRunning;
    QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
               static_cast<int>(state_));
  }

  return result;
}

int32_t AudioEndPointClient::SendBuffers(const vector<AudioBuffer>& buffers) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  for (const AudioBuffer& buffer : buffers)
    QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
                 buffer.ToString().c_str());

  switch (state_) {
    case AudioState::kIdle:
    case AudioState::kRunning:
      // proceed
      break;
    case AudioState::kNew:
    case AudioState::kConnect:
    case AudioState::kPaused:
      QMMF_ERROR("%s: %s() invalid operation for current state: %d", TAG,
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s: %s() unknown state: %d", TAG, __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  if (audio_service_.get() == nullptr) {
    QMMF_ERROR("%s: %s() not connected to service", TAG, __func__);
    return -ENOSYS;
  }

  int32_t result = audio_service_->SendBuffers(audio_handle_, buffers);
  if (result < 0)
    QMMF_ERROR("%s: %s() service->SendBuffers failed: %d", TAG, __func__,
               result);

  return result;
}

int32_t AudioEndPointClient::GetLatency(int32_t* latency) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  lock_guard<mutex> lock(lock_);

  switch (state_) {
    case AudioState::kIdle:
      // proceed
      break;
    case AudioState::kNew:
    case AudioState::kConnect:
    case AudioState::kRunning:
    case AudioState::kPaused:
      QMMF_ERROR("%s: %s() invalid operation for current state: %d", TAG,
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s: %s() unknown state: %d", TAG, __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  if (audio_service_.get() == nullptr) {
    QMMF_ERROR("%s: %s() not connected to service", TAG, __func__);
    return -ENOSYS;
  }

  int32_t result = audio_service_->GetLatency(audio_handle_, latency);
  if (result < 0)
    QMMF_ERROR("%s: %s() service->GetLatency failed: %d", TAG, __func__,
               result);

  QMMF_VERBOSE("%s: %s() OUTPARAM: latency[%d]", TAG, __func__, *latency);
  return result;
}

int32_t AudioEndPointClient::GetBufferSize(int32_t* buffer_size) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  lock_guard<mutex> lock(lock_);

  switch (state_) {
    case AudioState::kIdle:
      // proceed
      break;
    case AudioState::kNew:
    case AudioState::kConnect:
    case AudioState::kRunning:
    case AudioState::kPaused:
      QMMF_ERROR("%s: %s() invalid operation for current state: %d", TAG,
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s: %s() unknown state: %d", TAG, __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  if (audio_service_.get() == nullptr) {
    QMMF_ERROR("%s: %s() not connected to service", TAG, __func__);
    return -ENOSYS;
  }

  int32_t result = audio_service_->GetBufferSize(audio_handle_, buffer_size);
  if (result < 0)
    QMMF_ERROR("%s: %s() service->GetBufferSize failed: %d", TAG, __func__,
               result);

  QMMF_VERBOSE("%s: %s() OUTPARAM: buffer_size[%d]", TAG, __func__,
               *buffer_size);
  return result;
}

int32_t AudioEndPointClient::SetParam(const AudioParamType type,
                                      const AudioParamData& data) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: type[%d]", TAG, __func__,
               static_cast<int>(type));
  QMMF_VERBOSE("%s: %s() INPARAM: data[%s]", TAG, __func__,
               data.ToString(type).c_str());
  lock_guard<mutex> lock(lock_);

  switch (state_) {
    case AudioState::kIdle:
    case AudioState::kRunning:
    case AudioState::kPaused:
      // proceed
      break;
    case AudioState::kNew:
    case AudioState::kConnect:
      QMMF_ERROR("%s: %s() invalid operation for current state: %d", TAG,
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s: %s() unknown state: %d", TAG, __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  if (audio_service_.get() == nullptr) {
    QMMF_ERROR("%s: %s() not connected to service", TAG, __func__);
    return -ENOSYS;
  }

  int32_t result = audio_service_->SetParam(audio_handle_, type, data);
  if (result < 0)
    QMMF_ERROR("%s: %s() service->SetParam failed: %d", TAG, __func__, result);

  return result;
}

int32_t AudioEndPointClient::GetRenderedPosition(uint32_t* frames,
                                                  uint64_t* time) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  lock_guard<mutex> lock(lock_);

  switch (state_) {
    case AudioState::kIdle:
    case AudioState::kNew:
    case AudioState::kConnect:
      QMMF_ERROR("%s: %s() invalid operation for current state: %d", TAG,
          __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    case AudioState::kRunning:
    case AudioState::kPaused:
      // proceed
      break;
    default:
      QMMF_ERROR("%s: %s() unknown state: %d", TAG, __func__,
          static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  if (audio_service_.get() == nullptr) {
    QMMF_ERROR("%s: %s() not connected to service", TAG, __func__);
    return -ENOSYS;
  }

  int32_t result = audio_service_->GetRenderedPosition(audio_handle_, frames,
      time);
  if (result < 0) {
    QMMF_ERROR("%s: %s() service->GetRenderedPosition failed: %d", TAG, __func__,
        result);
  }

  QMMF_VERBOSE("%s: %s() OUTPARAM: frames[%u] time[%llu]", TAG, __func__,
      *frames, *time);

  return result;
}

void AudioEndPointClient::NotifyErrorEvent(const int32_t error) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: error[%d]", TAG, __func__, error);

  event_handler_(AudioEventType::kError, AudioEventData(error));
}

void AudioEndPointClient::NotifyBufferEvent(const AudioBuffer& buffer) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
               buffer.ToString().c_str());

  event_handler_(AudioEventType::kBuffer, AudioEventData(buffer));
}

// Binder proxy implementation of IAudioService
class BpAudioService: public BpInterface<IAudioService> {
 public:
  BpAudioService(const sp<IBinder>& impl) : BpInterface<IAudioService>(impl) {}

  int32_t Connect(const sp<IAudioServiceCallback>& client_handler,
                  AudioHandle* audio_handle) {
    QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
    Parcel input, output;

    // register service callback to get callbacks from audio service
    input.writeInterfaceToken(IAudioService::getInterfaceDescriptor());
    input.writeStrongBinder(IInterface::asBinder(client_handler));

    remote()->transact(static_cast<uint32_t>
                                  (AudioServiceCommand::kAudioConnect),
                       input, &output);

    *audio_handle = static_cast<AudioHandle>(output.readInt32());
    QMMF_VERBOSE("%s: %s() OUTPARAM: audio_handle[%d]", TAG, __func__,
                 *audio_handle);

    return output.readInt32();
  }

  int32_t Disconnect(const AudioHandle audio_handle) {
    QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
    QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
                 audio_handle);
    Parcel input, output;

    input.writeInterfaceToken(IAudioService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(audio_handle));

    remote()->transact(static_cast<uint32_t>
                                  (AudioServiceCommand::kAudioDisconnect),
                       input, &output);

    return output.readInt32();
  }

  int32_t Configure(const AudioHandle audio_handle,
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
    Parcel input, output;

    input.writeInterfaceToken(IAudioService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(audio_handle));
    input.writeInt32(static_cast<int32_t>(type));
    input.writeUint32(static_cast<uint32_t>(devices.size()));
    for (const DeviceId device : devices)
      input.writeInt32(static_cast<int32_t>(device));
    metadata.ToParcel(&input);

    remote()->transact(static_cast<uint32_t>
                                  (AudioServiceCommand::kAudioConfigure),
                       input, &output);

    return output.readInt32();
  }

  int32_t Start(const AudioHandle audio_handle) {
    QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
    QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
                 audio_handle);
    Parcel input, output;

    input.writeInterfaceToken(IAudioService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(audio_handle));

    remote()->transact(static_cast<uint32_t>(AudioServiceCommand::kAudioStart),
                       input, &output);

    return output.readInt32();
  }

  int32_t Stop(const AudioHandle audio_handle, const bool flush) {
    QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
    QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
                 audio_handle);
    QMMF_VERBOSE("%s: %s() INPARAM: flush[%s]", TAG, __func__,
                 flush ? "true" : "false");
    Parcel input, output;

    input.writeInterfaceToken(IAudioService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(audio_handle));
    input.writeInt32(static_cast<int32_t>(flush));

    remote()->transact(static_cast<uint32_t>(AudioServiceCommand::kAudioStop),
                       input, &output);

    return output.readInt32();
  }

  int32_t Pause(const AudioHandle audio_handle) {
    QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
    QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
                 audio_handle);
    Parcel input, output;

    input.writeInterfaceToken(IAudioService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(audio_handle));

    remote()->transact(static_cast<uint32_t>(AudioServiceCommand::kAudioPause),
                       input, &output);

    return output.readInt32();
  }

  int32_t Resume(const AudioHandle audio_handle) {
    QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
    QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
                 audio_handle);
    Parcel input, output;

    input.writeInterfaceToken(IAudioService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(audio_handle));

    remote()->transact(static_cast<uint32_t>(AudioServiceCommand::kAudioResume),
                       input, &output);

    return output.readInt32();
  }

  int32_t SendBuffers(const AudioHandle audio_handle,
                      const vector<AudioBuffer>& buffers) {
    QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
    QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
                 audio_handle);
    for (const AudioBuffer& buffer : buffers)
      QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
                   buffer.ToString().c_str());
    Parcel input, output;

    input.writeInterfaceToken(IAudioService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(audio_handle));
    input.writeUint32(static_cast<uint32_t>(buffers.size()));
    for (const AudioBuffer& buffer : buffers)
      buffer.ToParcel(&input, true);

    remote()->transact(static_cast<uint32_t>
                                  (AudioServiceCommand::kAudioSendBuffers),
                       input, &output);

    return output.readInt32();
  }

  int32_t GetLatency(const AudioHandle audio_handle, int32_t* latency) {
    QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
    QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
                 audio_handle);
    Parcel input, output;

    input.writeInterfaceToken(IAudioService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(audio_handle));

    remote()->transact(static_cast<uint32_t>
                                  (AudioServiceCommand::kAudioGetLatency),
                       input, &output);

    *latency = output.readInt32();
    QMMF_VERBOSE("%s: %s() OUTPARAM: latency[%d]", TAG, __func__, *latency);

    return output.readInt32();
  }

  int32_t GetBufferSize(const AudioHandle audio_handle, int32_t* buffer_size) {
    QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
    QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
                 audio_handle);
    Parcel input, output;

    input.writeInterfaceToken(IAudioService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(audio_handle));

    remote()->transact(static_cast<uint32_t>
                                  (AudioServiceCommand::kAudioGetBufferSize),
                       input, &output);

    *buffer_size = output.readInt32();
    QMMF_VERBOSE("%s: %s() OUTPARAM: buffer_size[%d]", TAG, __func__,
                 *buffer_size);

    return output.readInt32();
  }

  int32_t SetParam(const AudioHandle audio_handle, const AudioParamType type,
                   const AudioParamData& data) {
    QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
    QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
                 audio_handle);
    QMMF_VERBOSE("%s: %s() INPARAM: type[%d]", TAG, __func__,
                 static_cast<int>(type));
    QMMF_VERBOSE("%s: %s() INPARAM: data[%s]", TAG, __func__,
                 data.ToString(type).c_str());
    Parcel input, output;

    input.writeInterfaceToken(IAudioService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(audio_handle));
    input.writeInt32(static_cast<int32_t>(type));
    data.ToParcel(type, &input);

    remote()->transact(static_cast<uint32_t>
                                  (AudioServiceCommand::kAudioSetParam),
                       input, &output);

    return output.readInt32();
  }

  int32_t GetRenderedPosition(const AudioHandle audio_handle,
                             uint32_t* frames, uint64_t* time) {
    QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
    QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
                 audio_handle);
    Parcel input, output;

    input.writeInterfaceToken(IAudioService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(audio_handle));

    remote()->transact(static_cast<uint32_t>
                                  (AudioServiceCommand::kGetRenderedPosition),
                       input, &output);

    *frames = output.readUint32();
    *time = output.readUint64();
    QMMF_VERBOSE("%s: %s() OUTPARAM: Frames[%u] Time[%llu]", TAG, __func__,
        *frames, *time);

    return output.readInt32();
  }
};

IMPLEMENT_META_INTERFACE(AudioService, QMMF_AUDIO_SERVICE_NAME);

ServiceCallbackHandler::ServiceCallbackHandler(AudioEndPointClient* client)
    : client_(client) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
}

ServiceCallbackHandler::~ServiceCallbackHandler() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
}

void ServiceCallbackHandler::NotifyErrorEvent(const int32_t error) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: error[%d]", TAG, __func__, error);

  if (client_ == nullptr)
    QMMF_ERROR("%s: %s() no client to send notification to", TAG, __func__);
  else
    client_->NotifyErrorEvent(error);
}

void ServiceCallbackHandler::NotifyBufferEvent(const AudioBuffer& buffer) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
               buffer.ToString().c_str());

  if (client_ == nullptr)
    QMMF_ERROR("%s: %s() no client to send notification to", TAG, __func__);
  else
    client_->NotifyBufferEvent(buffer);
}

class BpAudioServiceCallback: public BpInterface<IAudioServiceCallback> {
 public:
  BpAudioServiceCallback(const sp<IBinder>& impl)
      : BpInterface<IAudioServiceCallback>(impl) {}

  void NotifyErrorEvent(const int32_t error) {
    QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
    QMMF_VERBOSE("%s: %s() INPARAM: error[%d]", TAG, __func__, error);
    Parcel input, output;

    input.writeInterfaceToken(IAudioServiceCallback::getInterfaceDescriptor());
    input.writeInt32(error);

    remote()->transact(static_cast<uint32_t>
        (AudioServiceCallbackCommand::kAudioNotifyError), input, &output);
  }

  void NotifyBufferEvent(const AudioBuffer& buffer) {
    QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
    QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
                 buffer.ToString().c_str());
    Parcel input, output;

    input.writeInterfaceToken(IAudioServiceCallback::getInterfaceDescriptor());
    buffer.ToParcel(&input, false);

    remote()->transact(static_cast<uint32_t>
        (AudioServiceCallbackCommand::kAudioNotifyBuffer), input, &output);
  }
};

IMPLEMENT_META_INTERFACE(AudioServiceCallback,
                         "audio.service.IAudioServiceCallback");

int32_t BnAudioServiceCallback::onTransact(uint32_t code, const Parcel& input,
                                           Parcel* output, uint32_t flags) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: code[%u]", TAG, __func__, code);
  QMMF_VERBOSE("%s: %s() INPARAM: flags[%u]", TAG, __func__, flags);

  if (!input.checkInterface(this))
    return -EPERM;

  switch (static_cast<AudioServiceCallbackCommand>(code)) {
    case AudioServiceCallbackCommand::kAudioNotifyError: {
      int32_t error = input.readInt32();

      QMMF_DEBUG("%s: %s-AudioNotifyError() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-AudioNotifyError() INPARAM: error[%d]", TAG,
                   __func__, error);
      NotifyErrorEvent(error);
      break;
    }

    case AudioServiceCallbackCommand::kAudioNotifyBuffer: {
      AudioBuffer buffer;
      buffer.FromParcel(input, false);

      QMMF_DEBUG("%s: %s-AudioNotifyBuffer() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-AudioNotifyBuffer() INPARAM: buffer[%s]", TAG,
                   __func__, buffer.ToString().c_str());
      NotifyBufferEvent(buffer);
      break;
    }

    default:
      QMMF_ERROR("%s: %s() code %u not supported ", TAG, __func__, code);
      break;
  }
  return 0;
}

}; // namespace audio
}; // namespace common
}; // namespace qmmf
