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

#define LOG_TAG "AudioFrontend"

#include "common/audio/src/service/qmmf_audio_frontend.h"

#include <functional>
#include <map>
#include <vector>
#include <type_traits>

#include <mm-audio/qahw_api/inc/qahw_api.h>
#include <mm-audio/qahw_api/inc/qahw_defs.h>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/src/service/qmmf_audio_backend.h"
#include "common/audio/src/service/qmmf_audio_backend_sink.h"
#include "common/audio/src/service/qmmf_audio_backend_source.h"
#include "common/audio/src/service/qmmf_audio_common.h"
#include "common/utils/qmmf_log.h"

namespace qmmf {
namespace common {
namespace audio {

using ::std::vector;

const AudioHandle AudioFrontend::kAudioHandleMax = 100;

AudioFrontend::AudioFrontend() : current_handle_(0) {
  int result;

  result = qahw_get_version();
  if (result < QAHW_MODULE_API_VERSION_MIN) {
    QMMF_ERROR("%s() incorrect QAHW module version[%d]", __func__,
               result);
    for (int idx = 0; idx < AudioHAL::kNum; ++idx)
      modules_[idx] = nullptr;
    return;
  }
  QMMF_INFO("%s() QAHW module version[%d]", __func__, result);

  modules_[AudioHAL::kPrimary] = qahw_load_module(QAHW_MODULE_ID_PRIMARY);
  if (modules_[AudioHAL::kPrimary] == nullptr)
    QMMF_ERROR("%s() failed to load QAHW module[%s]", __func__,
               QAHW_MODULE_ID_PRIMARY);

  result = qahw_init_check(modules_[AudioHAL::kPrimary]);
  if (result != 0) {
    QMMF_ERROR("%s() QAHW module[%s] initialization failed: %d[%s]",
               __func__, QAHW_MODULE_ID_PRIMARY, result, strerror(result));
    modules_[AudioHAL::kPrimary] = nullptr;
  }

  modules_[AudioHAL::kA2DP] = qahw_load_module(QAHW_MODULE_ID_A2DP);
  if (modules_[AudioHAL::kA2DP] == nullptr)
    QMMF_ERROR("%s() failed to load QAHW module[%s]", __func__,
               QAHW_MODULE_ID_A2DP);

  result = qahw_init_check(modules_[AudioHAL::kA2DP]);
  if (result != 0) {
    QMMF_ERROR("%s() QAHW module[%s] initialization failed: %d[%s]",
               __func__, QAHW_MODULE_ID_A2DP, result, strerror(result));
    modules_[AudioHAL::kA2DP] = nullptr;
  }

  modules_[AudioHAL::kUSB] = qahw_load_module(QAHW_MODULE_ID_USB);
  if (modules_[AudioHAL::kUSB] == nullptr)
    QMMF_ERROR("%s() failed to load QAHW module[%s]", __func__,
               QAHW_MODULE_ID_USB);

  result = qahw_init_check(modules_[AudioHAL::kUSB]);
  if (result != 0) {
    QMMF_ERROR("%s() QAHW module[%s] initialization failed: %d[%s]",
               __func__, QAHW_MODULE_ID_USB, result, strerror(result));
    modules_[AudioHAL::kUSB] = nullptr;
  }
}

AudioFrontend::~AudioFrontend() {
  int result;

  result = qahw_unload_module(modules_[AudioHAL::kPrimary]);
  if (result != 0)
    QMMF_ERROR("%s() failed to unload QAHW module[%s]: %d[%s]",
               __func__, QAHW_MODULE_ID_PRIMARY, result, strerror(result));

  result = qahw_unload_module(modules_[AudioHAL::kA2DP]);
  if (result != 0)
    QMMF_ERROR("%s() failed to unload QAHW module[%s]: %d[%s]",
               __func__, QAHW_MODULE_ID_A2DP, result, strerror(result));

  result = qahw_unload_module(modules_[AudioHAL::kUSB]);
  if (result != 0)
    QMMF_ERROR("%s() failed to unload QAHW module[%s]: %d[%s]",
               __func__, QAHW_MODULE_ID_USB, result, strerror(result));
}

void AudioFrontend::RegisterErrorHandler(const AudioErrorHandler& handler) {
  QMMF_DEBUG("%s() TRACE", __func__);

  error_handler_ = handler;
}

void AudioFrontend::RegisterBufferHandler(const AudioBufferHandler& handler) {
  QMMF_DEBUG("%s() TRACE", __func__);

  buffer_handler_ = handler;
}

void AudioFrontend::RegisterStoppedHandler(const AudioStoppedHandler& handler) {
  QMMF_DEBUG("%s() TRACE", __func__);

  stopped_handler_ = handler;
}

int32_t AudioFrontend::Connect(AudioHandle* audio_handle) {
  QMMF_DEBUG("%s() TRACE", __func__);

  // find an available AudioHandle
  if (current_handle_ + 1 > kAudioHandleMax)
    current_handle_ = 0;
  ++current_handle_;
  while (backends_.find(current_handle_) != backends_.end())
    ++current_handle_;

  backends_.insert({current_handle_, nullptr});

  *audio_handle = current_handle_;
  QMMF_VERBOSE("%s() OUTPARAM: audio_handle[%d]", __func__,
               *audio_handle);

  return 0;
}

int32_t AudioFrontend::Disconnect(const AudioHandle audio_handle) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: audio_handle[%d]", __func__,
               audio_handle);

  AudioBackendMap::iterator backend_iterator = backends_.find(audio_handle);
  if (backend_iterator == backends_.end()) {
    QMMF_ERROR("%s() no backend for key[%d]", __func__, audio_handle);
    return -EINVAL;
  }

  int32_t result = 0;
  if (backend_iterator->second != nullptr) {
    result = backend_iterator->second->Close();
    if (result < 0)
      QMMF_ERROR("%s() backend->Close failed: %d", __func__, result);

    delete backend_iterator->second;
    backend_iterator->second = nullptr;
  }

  backends_.erase(backend_iterator);

  return result;
}

int32_t AudioFrontend::Configure(const AudioHandle audio_handle,
                                 const AudioEndPointType type,
                                 const vector<DeviceId>& devices,
                                 const AudioMetadata& metadata) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: audio_handle[%d]", __func__,
               audio_handle);
  QMMF_VERBOSE("%s() INPARAM: type[%d]", __func__,
               static_cast<int>(type));
  for (const DeviceId device : devices)
    QMMF_VERBOSE("%s() INPARAM: device[%d]", __func__, device);
  QMMF_VERBOSE("%s() INPARAM: metadata[%s]", __func__,
               metadata.ToString().c_str());

  AudioBackendMap::iterator backend_iterator = backends_.find(audio_handle);
  if (backend_iterator == backends_.end()) {
    QMMF_ERROR("%s() no backend for key[%d]", __func__, audio_handle);
    return -EINVAL;
  }

  IAudioBackend* backend;
  if (type == AudioEndPointType::kSource) {
    backend = new AudioBackendSource(audio_handle, error_handler_,
                                     buffer_handler_);
  } else if (type == AudioEndPointType::kSink) {
    backend = new AudioBackendSink(audio_handle, error_handler_,
                                   buffer_handler_, stopped_handler_);
  } else {
    QMMF_ERROR("%s() invalid type given: %d", __func__,
               static_cast<int>(type));
    return -EINVAL;
  }
  if (backend == nullptr) {
    QMMF_ERROR("%s() unable to allocate backend", __func__);
    return -ENOMEM;
  }
  backend_iterator->second = backend;

  int32_t result = backend_iterator->second->Open(modules_, devices, metadata);
  if (result < 0)
    QMMF_ERROR("%s() backend->Open failed: %d", __func__, result);

  return result;
}

int32_t AudioFrontend::Start(const AudioHandle audio_handle) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: audio_handle[%d]", __func__,
               audio_handle);

  AudioBackendMap::iterator backend_iterator = backends_.find(audio_handle);
  if (backend_iterator == backends_.end()) {
    QMMF_ERROR("%s() no backend for key[%d]", __func__, audio_handle);
    return -EINVAL;
  }

  if (backend_iterator->second == nullptr) {
    QMMF_ERROR("%s() backend[%d] has a null object pointer", __func__,
               audio_handle);
    return -ENOSYS;
  }

  int32_t result = backend_iterator->second->Start();
  if (result < 0)
    QMMF_ERROR("%s() backend->Start failed: %d", __func__, result);

  return result;
}

int32_t AudioFrontend::Stop(const AudioHandle audio_handle) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: audio_handle[%d]", __func__,
               audio_handle);

  AudioBackendMap::iterator backend_iterator = backends_.find(audio_handle);
  if (backend_iterator == backends_.end()) {
    QMMF_ERROR("%s() no backend for key[%d]", __func__, audio_handle);
    return -EINVAL;
  }

  if (backend_iterator->second == nullptr) {
    QMMF_ERROR("%s() backend[%d] has a null object pointer", __func__,
               audio_handle);
    return -ENOSYS;
  }

  int32_t result = backend_iterator->second->Stop();
  if (result < 0)
    QMMF_ERROR("%s() backend->Stop failed: %d", __func__, result);

  return result;
}

int32_t AudioFrontend::Pause(const AudioHandle audio_handle) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: audio_handle[%d]", __func__,
               audio_handle);

  AudioBackendMap::iterator backend_iterator = backends_.find(audio_handle);
  if (backend_iterator == backends_.end()) {
    QMMF_ERROR("%s() no backend for key[%d]", __func__, audio_handle);
    return -EINVAL;
  }

  if (backend_iterator->second == nullptr) {
    QMMF_ERROR("%s() backend[%d] has a null object pointer", __func__,
               audio_handle);
    return -ENOSYS;
  }

  int32_t result = backend_iterator->second->Pause();
  if (result < 0)
    QMMF_ERROR("%s() backend->Pause failed: %d", __func__, result);

  return result;
}

int32_t AudioFrontend::Resume(const AudioHandle audio_handle) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: audio_handle[%d]", __func__,
               audio_handle);

  AudioBackendMap::iterator backend_iterator = backends_.find(audio_handle);
  if (backend_iterator == backends_.end()) {
    QMMF_ERROR("%s() no backend for key[%d]", __func__, audio_handle);
    return -EINVAL;
  }

  if (backend_iterator->second == nullptr) {
    QMMF_ERROR("%s() backend[%d] has a null object pointer", __func__,
               audio_handle);
    return -ENOSYS;
  }

  int32_t result = backend_iterator->second->Resume();
  if (result < 0)
    QMMF_ERROR("%s() backend->Resume failed: %d", __func__, result);

  return result;
}

int32_t AudioFrontend::SendBuffers(const AudioHandle audio_handle,
                                   const vector<AudioBuffer>& buffers) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: audio_handle[%d]", __func__,
               audio_handle);
  for (const AudioBuffer& buffer : buffers)
    QMMF_VERBOSE("%s() INPARAM: buffer[%s]", __func__,
                 buffer.ToString().c_str());

  AudioBackendMap::iterator backend_iterator = backends_.find(audio_handle);
  if (backend_iterator == backends_.end()) {
    QMMF_ERROR("%s() no backend for key[%d]", __func__, audio_handle);
    return -EINVAL;
  }

  if (backend_iterator->second == nullptr) {
    QMMF_ERROR("%s() backend[%d] has a null object pointer", __func__,
               audio_handle);
    return -ENOSYS;
  }

  int32_t result = backend_iterator->second->SendBuffers(buffers);
  if (result < 0)
    QMMF_ERROR("%s() backend->SendBuffers failed: %d", __func__,
               result);

  return result;
}

int32_t AudioFrontend::GetLatency(const AudioHandle audio_handle,
                                  int32_t* latency) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: audio_handle[%d]", __func__,
               audio_handle);

  AudioBackendMap::iterator backend_iterator = backends_.find(audio_handle);
  if (backend_iterator == backends_.end()) {
    QMMF_ERROR("%s() no backend for key[%d]", __func__, audio_handle);
    return -EINVAL;
  }

  if (backend_iterator->second == nullptr) {
    QMMF_ERROR("%s() backend[%d] has a null object pointer", __func__,
               audio_handle);
    return -ENOSYS;
  }

  int32_t result = backend_iterator->second->GetLatency(latency);
  if (result < 0)
    QMMF_ERROR("%s() backend->GetLatency failed: %d", __func__,
               result);

  QMMF_VERBOSE("%s() OUTPARAM: latency[%d]", __func__, *latency);
  return result;
}

int32_t AudioFrontend::GetBufferSize(const AudioHandle audio_handle,
                                     int32_t* buffer_size) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: audio_handle[%d]", __func__,
               audio_handle);

  AudioBackendMap::iterator backend_iterator = backends_.find(audio_handle);
  if (backend_iterator == backends_.end()) {
    QMMF_ERROR("%s() no backend for key[%d]", __func__, audio_handle);
    return -EINVAL;
  }

  if (backend_iterator->second == nullptr) {
    QMMF_ERROR("%s() backend[%d] has a null object pointer", __func__,
               audio_handle);
    return -ENOSYS;
  }

  int32_t result = backend_iterator->second->GetBufferSize(buffer_size);
  if (result < 0)
    QMMF_ERROR("%s() backend->GetBufferSize failed: %d", __func__,
               result);

  QMMF_VERBOSE("%s() OUTPARAM: buffer_size[%d]", __func__,
               *buffer_size);
  return result;
}

int32_t AudioFrontend::SetParam(const AudioHandle audio_handle,
                            const AudioParamType type,
                            const AudioParamData& data) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: audio_handle[%d]", __func__,
               audio_handle);
  QMMF_VERBOSE("%s() INPARAM: type[%d]", __func__,
               static_cast<int>(type));
  QMMF_VERBOSE("%s() INPARAM: data[%s]", __func__,
               data.ToString(type).c_str());

  AudioBackendMap::iterator backend_iterator = backends_.find(audio_handle);
  if (backend_iterator == backends_.end()) {
    QMMF_ERROR("%s() no backend for key[%d]", __func__, audio_handle);
    return -EINVAL;
  }

  if (backend_iterator->second == nullptr) {
    QMMF_ERROR("%s() backend[%d] has a null object pointer", __func__,
               audio_handle);
    return -ENOSYS;
  }

  int32_t result = backend_iterator->second->SetParam(type, data);
  if (result < 0)
    QMMF_ERROR("%s() backend->SetParam failed: %d", __func__, result);

  return result;
}

int32_t AudioFrontend::GetRenderedPosition(const AudioHandle audio_handle,
                                           uint32_t* frames, uint64_t* time) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: audio_handle[%d]", __func__,
               audio_handle);

  AudioBackendMap::iterator backend_iterator = backends_.find(audio_handle);
  if (backend_iterator == backends_.end()) {
    QMMF_ERROR("%s() no backend for key[%d]", __func__, audio_handle);
    return -EINVAL;
  }

  if (backend_iterator->second == nullptr) {
    QMMF_ERROR("%s() backend[%d] has a null object pointer", __func__,
               audio_handle);
    return -ENOSYS;
  }

  int32_t result = backend_iterator->second->GetRenderedPosition(frames, time);
  if (result < 0) {
    QMMF_ERROR("%s() backend->GetRenderedPosition failed: %d", __func__,
        result);
  }

  QMMF_VERBOSE("%s() OUTPARAM: Frames[%u] Time[%llu]", __func__,
      *frames, *time);
  return result;
}

}; // namespace audio
}; // namespace common
}; // namespace qmmf
