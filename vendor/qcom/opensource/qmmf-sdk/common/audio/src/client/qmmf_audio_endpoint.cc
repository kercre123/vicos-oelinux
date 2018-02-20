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

#define LOG_TAG "AudioEndPoint"

#include <vector>

#include "common/audio/inc/qmmf_audio_endpoint.h"

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/src/client/qmmf_audio_endpoint_client.h"
#include "common/utils/qmmf_log.h"

namespace qmmf {
namespace common {
namespace audio {

AudioEndPoint::AudioEndPoint()
    : audio_endpoint_client_(nullptr) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_INFO("%s() endpoint instantiated", __func__);
}

AudioEndPoint::~AudioEndPoint() {
  QMMF_DEBUG("%s() TRACE", __func__);

  if (audio_endpoint_client_ != nullptr) {
    delete audio_endpoint_client_;
    audio_endpoint_client_ = nullptr;
  }

  QMMF_INFO("%s() endpoint destroyed", __func__);
}

int32_t AudioEndPoint::Connect(const AudioEventHandler& handler) {
  QMMF_DEBUG("%s() TRACE", __func__);

  audio_endpoint_client_ = new AudioEndPointClient();
  if (audio_endpoint_client_ == nullptr)
    return -ENOMEM;

  int32_t result = audio_endpoint_client_->Connect(handler);
  if (result < 0)
    QMMF_ERROR("%s() client->Connect failed: %d", __func__, result);

  return result;
}

int32_t AudioEndPoint::Disconnect() {
  QMMF_DEBUG("%s() TRACE", __func__);

  if (audio_endpoint_client_ == nullptr) {
    QMMF_WARN("%s() end point already disconnected", __func__);
    return 0;
  }

  int32_t result = audio_endpoint_client_->Disconnect();
  if (result < 0)
    QMMF_ERROR("%s() client->Disconnect failed: %d", __func__, result);

  delete audio_endpoint_client_;
  audio_endpoint_client_ = nullptr;

  return result;
}

int32_t AudioEndPoint::Configure(const AudioEndPointType type,
                                 const ::std::vector<DeviceId>& devices,
                                 const AudioMetadata& metadata) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: type[%d]", __func__,
               static_cast<int>(type));
  for (const DeviceId device : devices)
    QMMF_VERBOSE("%s() INPARAM: device[%d]", __func__, device);
  QMMF_VERBOSE("%s() INPARAM: metadata[%s]", __func__,
               metadata.ToString().c_str());
  assert(audio_endpoint_client_ != nullptr);

  int32_t result = audio_endpoint_client_->Configure(type, devices, metadata);
  if (result < 0)
    QMMF_ERROR("%s() client->Configure failed: %d", __func__, result);

  return result;
}

int32_t AudioEndPoint::Start() {
  QMMF_DEBUG("%s() TRACE", __func__);
  assert(audio_endpoint_client_ != nullptr);

  int32_t result = audio_endpoint_client_->Start();
  if (result < 0)
    QMMF_ERROR("%s() client->Start failed: %d", __func__, result);

  return result;
}

int32_t AudioEndPoint::Stop() {
  QMMF_DEBUG("%s() TRACE", __func__);

  if (audio_endpoint_client_ == nullptr) {
    QMMF_WARN("%s() stopping an unconnected end point", __func__);
    return 0;
  }

  int32_t result = audio_endpoint_client_->Stop();
  if (result < 0)
    QMMF_ERROR("%s() client->Stop failed: %d", __func__, result);

  return result;
}

int32_t AudioEndPoint::Pause() {
  QMMF_DEBUG("%s() TRACE", __func__);
  assert(audio_endpoint_client_ != nullptr);

  int32_t result = audio_endpoint_client_->Pause();
  if (result < 0)
    QMMF_ERROR("%s() client->Pause failed: %d", __func__, result);

  return result;
}

int32_t AudioEndPoint::Resume() {
  QMMF_DEBUG("%s() TRACE", __func__);
  assert(audio_endpoint_client_ != nullptr);

  int32_t result = audio_endpoint_client_->Resume();
  if (result < 0)
    QMMF_ERROR("%s() client->Resume failed: %d", __func__, result);

  return result;
}

int32_t AudioEndPoint::SendBuffers(const ::std::vector<AudioBuffer>& buffers) {
  QMMF_DEBUG("%s() TRACE", __func__);
  for (const AudioBuffer& buffer : buffers)
    QMMF_VERBOSE("%s() INPARAM: buffer[%s]", __func__,
                 buffer.ToString().c_str());
  assert(audio_endpoint_client_ != nullptr);

  int32_t result = audio_endpoint_client_->SendBuffers(buffers);
  if (result < 0)
    QMMF_ERROR("%s() client->SendBuffers failed: %d", __func__, result);

  return result;
}

int32_t AudioEndPoint::GetLatency(int32_t* latency) {
  QMMF_DEBUG("%s() TRACE", __func__);
  assert(audio_endpoint_client_ != nullptr);
  assert(latency != nullptr);

  int32_t result = audio_endpoint_client_->GetLatency(latency);
  if (result < 0)
    QMMF_ERROR("%s() client->GetLatency failed: %d", __func__, result);

  QMMF_VERBOSE("%s() OUTPARAM: latency[%d]", __func__, *latency);
  return result;
}

int32_t AudioEndPoint::GetBufferSize(int32_t* buffer_size) {
  QMMF_DEBUG("%s() TRACE", __func__);
  assert(audio_endpoint_client_ != nullptr);
  assert(buffer_size != nullptr);

  int32_t result = audio_endpoint_client_->GetBufferSize(buffer_size);
  if (result < 0)
    QMMF_ERROR("%s() client->GetBufferSize failed: %d", __func__,
               result);

  QMMF_VERBOSE("%s() OUTPARAM: buffer_size[%d]", __func__,
               *buffer_size);
  return result;
}

int32_t AudioEndPoint::SetParam(const AudioParamType type,
                                const AudioParamData& data) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: type[%d]", __func__,
               static_cast<int>(type));
  QMMF_VERBOSE("%s() INPARAM: data[%s]", __func__,
               data.ToString(type).c_str());
  assert(audio_endpoint_client_ != nullptr);

  int32_t result = audio_endpoint_client_->SetParam(type, data);
  if (result < 0)
    QMMF_ERROR("%s() client->SetParam failed: %d", __func__, result);

  return result;
}

int32_t AudioEndPoint::GetRenderedPosition(uint32_t* frames,
                                           uint64_t* time) {
  QMMF_DEBUG("%s() TRACE", __func__);
  assert(audio_endpoint_client_ != nullptr);
  assert(frames != nullptr);
  assert(time != nullptr);

  int32_t result = audio_endpoint_client_->GetRenderedPosition(frames, time);
  if (result < 0)
    QMMF_ERROR("%s() client->GetRenderedPosition failed: %d", __func__,
        result);

  QMMF_VERBOSE("%s() OUTPARAM: [%u] [%llu]", __func__, *frames, *time);
  return result;
}

}; // namespace audio
}; // namespace common
}; // namespace qmmf
