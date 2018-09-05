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

#define LOG_TAG "RecorderAudioRawTrackSource"

#include "recorder/src/service/qmmf_audio_track_source.h"

#include <cstdint>
#include <cstring>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/inc/qmmf_audio_endpoint.h"
#include "common/utils/qmmf_log.h"
#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_recorder_ion.h"

namespace qmmf {
namespace recorder {

using ::qmmf::AudioFormat;
using ::qmmf::DeviceId;
using ::qmmf::common::audio::AudioBuffer;
using ::qmmf::common::audio::AudioEndPoint;
using ::qmmf::common::audio::AudioEndPointType;
using ::qmmf::common::audio::AudioEventHandler;
using ::qmmf::common::audio::AudioEventType;
using ::qmmf::common::audio::AudioEventData;
using ::qmmf::common::audio::AudioMetadata;
using ::qmmf::common::audio::AudioParamCustomData;
using ::qmmf::common::audio::AudioParamType;
using ::std::mutex;
using ::std::queue;
using ::std::string;
using ::std::thread;
using ::std::unique_lock;
using ::std::vector;

static const int kNumberOfBuffers = 4;

AudioRawTrackSource::AudioRawTrackSource(const AudioTrackParams& params)
    : track_params_(params),
      end_point_(nullptr),
      thread_(nullptr) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: params[%s]", __func__,
               params.ToString().c_str());
}

AudioRawTrackSource::~AudioRawTrackSource() {
  QMMF_DEBUG("%s() TRACE", __func__);
}

status_t AudioRawTrackSource::Init() {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  int32_t result;
  AudioMetadata metadata{};

  if (end_point_ != nullptr) {
    QMMF_ERROR("%s() endpoint already exists", __func__);
    return ::android::ALREADY_EXISTS;
  }

  end_point_ = new AudioEndPoint;
  if (end_point_ == nullptr) {
    QMMF_ERROR("%s() could not instantiate endpoint", __func__);
    return ::android::NO_MEMORY;
  }

  AudioEventHandler audio_handler =
    [this] (AudioEventType event_type, const AudioEventData& event_data)
           -> void {
      switch (event_type) {
        case AudioEventType::kError:
          ErrorHandler(event_data.error);
          break;
        case AudioEventType::kBuffer:
          BufferHandler(event_data.buffer);
          break;
        case AudioEventType::kStopped:
          // TODO
          break;
      }
    };

  result = end_point_->Connect(audio_handler);
  if (result < 0) {
    QMMF_ERROR("%s() endpoint->Connect failed: %d[%s]", __func__,
               result, strerror(result));
    goto error_free;
  }

  metadata.format = AudioFormat::kPCM;
  metadata.num_channels = track_params_.params.channels;
  metadata.sample_rate = track_params_.params.sample_rate;
  metadata.sample_size = track_params_.params.bit_depth;\
  {
    vector<DeviceId> devices;
    for (uint32_t i = 0; i < track_params_.params.in_devices_num; i++)
      devices.push_back(track_params_.params.in_devices[i]);

    result = end_point_->Configure(AudioEndPointType::kSource, devices,
                                   metadata);
  }
  if (result < 0) {
    QMMF_ERROR("%s() endpoint->Configure failed: %d[%s]", __func__,
               result, strerror(result));
    goto error_disconnect;
  }

  if (strlen(track_params_.params.profile) > 0) {
    auto ret = SetParameter("audio_stream_profile",
                            track_params_.params.profile);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: Failed to enable profile: %d", __func__, ret);
      return ret;
    }
  }

  int32_t buffer_size;
  result = end_point_->GetBufferSize(&buffer_size);
  if (result < 0) {
    QMMF_ERROR("%s() endpoint->GetBufferSize failed: %d[%s]", __func__,
               result, strerror(result));
    goto error_disconnect;
  }
  QMMF_INFO("%s() buffer_size is %d", __func__, buffer_size);

  result = ion_.Allocate(kNumberOfBuffers, buffer_size);
  if (result < 0) {
    QMMF_ERROR("%s() ion->Allocate failed: %d[%s]", __func__, result,
               strerror(result));
    goto error_deallocate;
  }

  return ::android::NO_ERROR;

error_deallocate:
  ion_.Deallocate();

error_disconnect:
  end_point_->Disconnect();

error_free:
  delete end_point_;
  end_point_ = nullptr;

  return ::android::FAILED_TRANSACTION;
}

status_t AudioRawTrackSource::DeInit() {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  int32_t result;

  result = ion_.Deallocate();
  if (result < 0)
    QMMF_ERROR("%s() ion->Deallocate failed: %d[%s]", __func__, result,
               strerror(result));

  result = end_point_->Disconnect();
  if (result < 0)
    QMMF_ERROR("%s() endpoint->Disconnect failed: %d[%s]", __func__,
               result, strerror(result));

  delete end_point_;
  end_point_ = nullptr;

  return ::android::NO_ERROR;
}

status_t AudioRawTrackSource::StartTrack() {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);

  if (thread_ != nullptr) {
    QMMF_ERROR("%s() track already started", __func__);
    return ::android::ALREADY_EXISTS;
  }

  int32_t result = end_point_->Start();
  if (result < 0) {
    QMMF_ERROR("%s() endpoint->Start failed: %d[%s]", __func__,
               result, strerror(result));
    return ::android::FAILED_TRANSACTION;
  }

  while (!messages_.empty())
    messages_.pop();

  thread_ = new thread(AudioRawTrackSource::ThreadEntry, this);
  if (thread_ == nullptr) {
    QMMF_ERROR("%s() could not instantiate thread", __func__);
    end_point_->Stop();
    return ::android::NO_MEMORY;
  }

  return ::android::NO_ERROR;
}

status_t AudioRawTrackSource::StopTrack() {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);

  AudioMessage message;
  message.type = AudioMessageType::kMessageStop;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.Signal();

  int32_t result = end_point_->Stop();
  if (result < 0) {
    QMMF_ERROR("%s() endpoint->Stop failed: %d[%s]", __func__,
               result, strerror(result));
    return ::android::FAILED_TRANSACTION;
  }

  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
    thread_ = nullptr;
  }

  while (!messages_.empty())
    messages_.pop();

  return ::android::NO_ERROR;
}

status_t AudioRawTrackSource::PauseTrack() {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);

  AudioMessage message;
  message.type = AudioMessageType::kMessagePause;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.Signal();

  int32_t result = end_point_->Pause();
  if (result < 0) {
    QMMF_ERROR("%s() endpoint->Pause failed: %d[%s]", __func__,
               result, strerror(result));
    return ::android::FAILED_TRANSACTION;
  }

  return ::android::NO_ERROR;
}

status_t AudioRawTrackSource::ResumeTrack() {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);

  int32_t result = end_point_->Resume();
  if (result < 0) {
    QMMF_ERROR("%s() endpoint->Resume failed: %d[%s]", __func__,
               result, strerror(result));
    return ::android::FAILED_TRANSACTION;
  }

  AudioMessage message;
  message.type = AudioMessageType::kMessageResume;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.Signal();

  return ::android::NO_ERROR;
}

status_t AudioRawTrackSource::SetParameter(const string& key,
                                           const string& value) {
  QMMF_VERBOSE("%s() INPARAM: key[%s]", __func__, key.c_str());
  QMMF_VERBOSE("%s() INPARAM: value[%s]", __func__, value.c_str());

  AudioParamCustomData data;
  data.key = key;
  data.value = value;

  int32_t result = end_point_->SetParam(AudioParamType::kCustom, data);
  if (result < 0) {
    QMMF_ERROR("%s() endpoint->SetParam failed: %d[%s]", __func__,
               result, strerror(result));
    return ::android::FAILED_TRANSACTION;
  }

  return ::android::NO_ERROR;
}

status_t AudioRawTrackSource::ReturnTrackBuffer(
    const std::vector<BnBuffer> &buffers) {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  for (const BnBuffer& buffer : buffers)
    QMMF_VERBOSE("%s() INPARAM: bn_buffer[%s]", __func__,
                 buffer.ToString().c_str());

  for (const BnBuffer& buffer : buffers) {
    AudioMessage message;
    message.type = AudioMessageType::kMessageBnBuffer;
    message.bn_buffer = buffer;

    message_lock_.lock();
    messages_.push(message);
    message_lock_.unlock();
    signal_.Signal();
  }

  return ::android::NO_ERROR;
}

void AudioRawTrackSource::ErrorHandler(const int32_t error) {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  QMMF_VERBOSE("%s() INPARAM: type[%d]", __func__, error);

  QMMF_ERROR("%s() received error from endpoint: %d[%s]", __func__,
               error, strerror(error));
  assert(false);
  // TODO(kwestfie@codeaurora.org): send notification to application instead
}

void AudioRawTrackSource::BufferHandler(const AudioBuffer& buffer) {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  QMMF_VERBOSE("%s() INPARAM: buffer[%s]", __func__,
               buffer.ToString().c_str());

  AudioMessage message;
  message.type = AudioMessageType::kMessageBuffer;
  message.buffer = buffer;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.Signal();
}

void AudioRawTrackSource::ThreadEntry(AudioRawTrackSource* source) {
  QMMF_DEBUG("%s() TRACE: track_id", __func__);

  source->Thread();
}

void AudioRawTrackSource::Thread() {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  queue<AudioBuffer> buffers;
  queue<BnBuffer> bn_buffers;
  int32_t result;

  // send the initial list of buffers
  vector<AudioBuffer> initial_buffers;
  ion_.GetList(&initial_buffers);
  result = end_point_->SendBuffers(initial_buffers);
  if (result < 0) {
    QMMF_ERROR("%s() endpoint->SendBuffers failed: %d[%s]", __func__,
               result, strerror(result));
    assert(false);
    // TODO(kwestfie@codeaurora.org): send notification to application instead
  }
  initial_buffers.clear();

  bool keep_running = true;
  bool stop_received = false;
  bool paused = false;
  while (keep_running) {
    // wait until there is something to do
    if (bn_buffers.empty() && buffers.empty() && messages_.empty()) {
      unique_lock<mutex> lk(message_lock_);
      signal_.Wait(lk);
    }

    // process the next pending message
    message_lock_.lock();
    if (!messages_.empty()) {
      AudioMessage message = messages_.front();

      switch (message.type) {
        case AudioMessageType::kMessagePause:
          QMMF_DEBUG("%s-MessagePause() TRACE", __func__);
          paused = true;
          break;

        case AudioMessageType::kMessageResume:
          QMMF_DEBUG("%s-MessageResume() TRACE", __func__);
          paused = false;
          break;

        case AudioMessageType::kMessageStop:
          QMMF_DEBUG("%s-MessageStop() TRACE", __func__);
          paused = false;
          stop_received = true;
          break;

        case AudioMessageType::kMessageBuffer:
          QMMF_DEBUG("%s-MessageBuffer() TRACE", __func__);
          QMMF_VERBOSE("%s() INPARAM: buffer[%s] to queue[%u]",
                       __func__, message.buffer.ToString().c_str(),
                       buffers.size());
          buffers.push(message.buffer);
          QMMF_VERBOSE("%s() buffers queue is now %u deep",
                       __func__, buffers.size());
          break;

        case AudioMessageType::kMessageBnBuffer:
          QMMF_DEBUG("%s-MessageBnBuffer() TRACE", __func__);
          QMMF_VERBOSE("%s() INPARAM: bn_buffer[%s] to queue[%u]",
                       __func__, message.bn_buffer.ToString().c_str(),
                       bn_buffers.size());
          bn_buffers.push(message.bn_buffer);
          QMMF_VERBOSE("%s() bn_buffers queue is now %u deep",
                       __func__, bn_buffers.size());
          break;
      }
      messages_.pop();
    }
    message_lock_.unlock();

    // process buffers from endpoint
    if (!buffers.empty() && !paused && keep_running) {
      AudioBuffer buffer = buffers.front();
      QMMF_VERBOSE("%s() track[%u] processing next buffer[%s] from queue[%u]",
                   __func__, track_params_.track_id,
                   buffer.ToString().c_str(), buffers.size());

      BnBuffer bn_buffer;
      ion_.Export(buffer, &bn_buffer);
      std::vector<BnBuffer> bn_buffers;
      bn_buffers.push_back(bn_buffer);

      MetaData meta_data{};
      meta_data.meta_flag = static_cast<uint32_t>(MetaParamType::kNone);
      std::vector<MetaData> meta_buffers;
      meta_buffers.push_back(meta_data);
      track_params_.data_cb(bn_buffers, meta_buffers);

      buffers.pop();
      QMMF_VERBOSE("%s() buffers queue is now %u deep",
                   __func__, buffers.size());
    }

    // process buffers from client
    if (!bn_buffers.empty() && !paused && keep_running) {
      BnBuffer bn_buffer = bn_buffers.front();
      QMMF_VERBOSE("%s() track[%u] processing next bn_buffer[%s] from queue[%u]",
                   __func__, track_params_.track_id,
                   bn_buffer.ToString().c_str(), bn_buffers.size());

      if (stop_received &&
          bn_buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS)) {
        keep_running = false;
      } else {
        AudioBuffer buffer;
        ion_.Import(bn_buffer, &buffer);

        memset(buffer.data, 0x00, buffer.capacity);
        buffer.size = 0;
        buffer.timestamp = 0;

        int32_t result = end_point_->SendBuffers({buffer});
        if (result < 0) {
          QMMF_ERROR("%s() endpoint->SendBuffers failed: %d[%s]",
                     __func__, result, strerror(result));
          assert(false);
          // TODO(kwestfie@codeaurora.org): send notification to application
        }
      }

      bn_buffers.pop();
      QMMF_VERBOSE("%s() bn_buffers queue is now %u deep",
                   __func__, bn_buffers.size());
    }
  }
}

}; // namespace recorder
}; // namespace qmmf
