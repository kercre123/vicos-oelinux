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

#define TAG "RecorderAudioRawTrackSource"

#include "recorder/src/service/qmmf_audio_track_source.h"

#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/inc/qmmf_audio_endpoint.h"
#include "common/qmmf_log.h"
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
using ::qmmf::common::audio::AudioMetadata;
using ::qmmf::common::audio::AudioEventType;
using ::qmmf::common::audio::AudioEventData;
using ::std::condition_variable;
using ::std::mutex;
using ::std::queue;
using ::std::thread;
using ::std::unique_lock;
using ::std::vector;

static const int kNumberOfBuffers = 4;

AudioRawTrackSource::AudioRawTrackSource(const AudioTrackParams& params)
    : track_params_(params),
      end_point_(nullptr),
      thread_(nullptr) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: params[%s]", TAG, __func__,
               params.ToString().c_str());
}

AudioRawTrackSource::~AudioRawTrackSource() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
}

status_t AudioRawTrackSource::Init() {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);
  int32_t result;

  if (end_point_ != nullptr) {
    QMMF_ERROR("%s: %s() endpoint already exists", TAG, __func__);
    return ::android::ALREADY_EXISTS;
  }

  end_point_ = new AudioEndPoint;
  if (end_point_ == nullptr) {
    QMMF_ERROR("%s: %s() could not instantiate endpoint", TAG, __func__);
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
      }
    };

  result = end_point_->Connect(audio_handler);
  if (result < 0) {
    QMMF_ERROR("%s: %s() endpoint->Connect failed: %d[%s]", TAG, __func__,
               result, strerror(result));
    goto error_free;
  }

  AudioMetadata metadata;
  memset(&metadata, 0x0, sizeof metadata);
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
    QMMF_ERROR("%s: %s() endpoint->Configure failed: %d[%s]", TAG, __func__,
               result, strerror(result));
    goto error_disconnect;
  }

  int32_t buffer_size;
  result = end_point_->GetBufferSize(&buffer_size);
  if (result < 0) {
    QMMF_ERROR("%s: %s() endpoint->GetBufferSize failed: %d[%s]", TAG, __func__,
               result, strerror(result));
    goto error_disconnect;
  }
  QMMF_INFO("%s: %s() buffer_size is %d", TAG, __func__, buffer_size);

  result = ion_.Allocate(kNumberOfBuffers, buffer_size);
  if (result < 0) {
    QMMF_ERROR("%s: %s() ion->Allocate failed: %d[%s]", TAG, __func__, result,
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
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);
  int32_t result;

  result = ion_.Deallocate();
  if (result < 0)
    QMMF_ERROR("%s: %s() ion->Deallocate failed: %d[%s]", TAG, __func__, result,
               strerror(result));

  result = end_point_->Disconnect();
  if (result < 0)
    QMMF_ERROR("%s: %s() endpoint->Disconnect failed: %d[%s]", TAG, __func__,
               result, strerror(result));

  delete end_point_;
  end_point_ = nullptr;

  return ::android::NO_ERROR;
}

status_t AudioRawTrackSource::StartTrack() {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);

  if (thread_ != nullptr) {
    QMMF_ERROR("%s: %s() track already started", TAG, __func__);
    return ::android::ALREADY_EXISTS;
  }

  int32_t result = end_point_->Start();
  if (result < 0) {
    QMMF_ERROR("%s: %s() endpoint->Start failed: %d[%s]", TAG, __func__,
               result, strerror(result));
    return ::android::FAILED_TRANSACTION;
  }

  while (!messages_.empty())
    messages_.pop();

  thread_ = new thread(AudioRawTrackSource::ThreadEntry, this);
  if (thread_ == nullptr) {
    QMMF_ERROR("%s: %s() could not instantiate thread", TAG, __func__);
    end_point_->Stop(false);
    return ::android::NO_MEMORY;
  }

  return ::android::NO_ERROR;
}

status_t AudioRawTrackSource::StopTrack() {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);

  AudioMessage message;
  message.type = AudioMessageType::kMessageStop;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();

  int32_t result = end_point_->Stop(false);
  if (result < 0) {
    QMMF_ERROR("%s: %s() endpoint->Stop failed: %d[%s]", TAG, __func__,
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
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);

  AudioMessage message;
  message.type = AudioMessageType::kMessagePause;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();

  int32_t result = end_point_->Pause();
  if (result < 0) {
    QMMF_ERROR("%s: %s() endpoint->Pause failed: %d[%s]", TAG, __func__,
               result, strerror(result));
    return ::android::FAILED_TRANSACTION;
  }

  return ::android::NO_ERROR;
}

status_t AudioRawTrackSource::ResumeTrack() {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);

  int32_t result = end_point_->Resume();
  if (result < 0) {
    QMMF_ERROR("%s: %s() endpoint->Resume failed: %d[%s]", TAG, __func__,
               result, strerror(result));
    return ::android::FAILED_TRANSACTION;
  }

  AudioMessage message;
  message.type = AudioMessageType::kMessageResume;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();

  return ::android::NO_ERROR;
}

status_t AudioRawTrackSource::ReturnTrackBuffer(
    const std::vector<BnBuffer> &buffers) {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);
  for (const BnBuffer& buffer : buffers)
    QMMF_VERBOSE("%s: %s() INPARAM: bn_buffer[%s]", TAG, __func__,
                 buffer.ToString().c_str());

  for (const BnBuffer& buffer : buffers) {
    AudioMessage message;
    message.type = AudioMessageType::kMessageBnBuffer;
    message.bn_buffer = buffer;

    message_lock_.lock();
    messages_.push(message);
    message_lock_.unlock();
    signal_.notify_one();
  }

  return ::android::NO_ERROR;
}

void AudioRawTrackSource::ErrorHandler(const int32_t error) {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);
  QMMF_VERBOSE("%s: %s() INPARAM: type[%d]", TAG, __func__, error);

  QMMF_ERROR("%s: %s() received error from endpoint: %d[%s]", TAG, __func__,
               error, strerror(error));
  assert(false);
  // TODO(kwestfie@codeaurora.org): send notification to application instead
}

void AudioRawTrackSource::BufferHandler(const AudioBuffer& buffer) {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);
  QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
               buffer.ToString().c_str());

  AudioMessage message;
  message.type = AudioMessageType::kMessageBuffer;
  message.buffer = buffer;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();
}

void AudioRawTrackSource::ThreadEntry(AudioRawTrackSource* source) {
  QMMF_DEBUG("%s: %s() TRACE: track_id", TAG, __func__);

  source->Thread();
}

void AudioRawTrackSource::Thread() {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);
  queue<AudioBuffer> buffers;
  queue<BnBuffer> bn_buffers;
  int32_t result;

  // send the initial list of buffers
  vector<AudioBuffer> initial_buffers;
  ion_.GetList(&initial_buffers);
  result = end_point_->SendBuffers(initial_buffers);
  if (result < 0) {
    QMMF_ERROR("%s: %s() endpoint->SendBuffers failed: %d[%s]", TAG, __func__,
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
      signal_.wait(lk);
    }

    // process the next pending message
    message_lock_.lock();
    if (!messages_.empty()) {
      AudioMessage message = messages_.front();

      switch (message.type) {
        case AudioMessageType::kMessagePause:
          QMMF_DEBUG("%s: %s-MessagePause() TRACE", TAG, __func__);
          paused = true;
          break;

        case AudioMessageType::kMessageResume:
          QMMF_DEBUG("%s: %s-MessageResume() TRACE", TAG, __func__);
          paused = false;
          break;

        case AudioMessageType::kMessageStop:
          QMMF_DEBUG("%s: %s-MessageStop() TRACE", TAG, __func__);
          paused = false;
          stop_received = true;
          break;

        case AudioMessageType::kMessageBuffer:
          QMMF_DEBUG("%s: %s-MessageBuffer() TRACE", TAG, __func__);
          QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s] to queue[%u]",
                       TAG, __func__, message.buffer.ToString().c_str(),
                       buffers.size());
          buffers.push(message.buffer);
          QMMF_VERBOSE("%s: %s() buffers queue is now %u deep",
                       TAG, __func__, buffers.size());
          break;

        case AudioMessageType::kMessageBnBuffer:
          QMMF_DEBUG("%s: %s-MessageBnBuffer() TRACE", TAG, __func__);
          QMMF_VERBOSE("%s: %s() INPARAM: bn_buffer[%s] to queue[%u]",
                       TAG, __func__, message.bn_buffer.ToString().c_str(),
                       bn_buffers.size());
          bn_buffers.push(message.bn_buffer);
          QMMF_VERBOSE("%s: %s() bn_buffers queue is now %u deep",
                       TAG, __func__, bn_buffers.size());
          break;
      }
      messages_.pop();
    }
    message_lock_.unlock();

    // process buffers from endpoint
    if (!buffers.empty() && !paused && keep_running) {
      AudioBuffer buffer = buffers.front();
      QMMF_VERBOSE("%s: %s() track[%u] processing next buffer[%s] from queue[%u]",
                   TAG, __func__, track_params_.track_id,
                   buffer.ToString().c_str(), buffers.size());

      BnBuffer bn_buffer;
      ion_.Export(buffer, &bn_buffer);
      std::vector<BnBuffer> bn_buffers;
      bn_buffers.push_back(bn_buffer);

      MetaData meta_data;
      memset(&meta_data, 0x0, sizeof meta_data);
      meta_data.meta_flag = static_cast<uint32_t>(MetaParamType::kNone);
      std::vector<MetaData> meta_buffers;
      meta_buffers.push_back(meta_data);
      track_params_.data_cb(bn_buffers, meta_buffers);

      if (stop_received &&
          buffer.flags & static_cast<uint32_t>(BufferFlags::kFlagEOS))
        keep_running = false;

      buffers.pop();
      QMMF_VERBOSE("%s: %s() buffers queue is now %u deep",
                   TAG, __func__, buffers.size());
    }

    // process buffers from client
    if (!bn_buffers.empty() && !paused && keep_running) {
      BnBuffer bn_buffer = bn_buffers.front();
      QMMF_VERBOSE("%s: %s() track[%u] processing next bn_buffer[%s] from queue[%u]",
                   TAG, __func__, track_params_.track_id,
                   bn_buffer.ToString().c_str(), bn_buffers.size());

      AudioBuffer buffer;
      ion_.Import(bn_buffer, &buffer);

      memset(buffer.data, 0x00, buffer.capacity);
      buffer.size = 0;
      buffer.timestamp = 0;

      int32_t result = end_point_->SendBuffers({buffer});
      if (result < 0) {
        QMMF_ERROR("%s: %s() endpoint->SendBuffers failed: %d[%s]", TAG,
                   __func__, result, strerror(result));
        assert(false);
        // TODO(kwestfie@codeaurora.org): send notification to application
      }

      bn_buffers.pop();
      QMMF_VERBOSE("%s: %s() bn_buffers queue is now %u deep",
                   TAG, __func__, bn_buffers.size());
    }
  }
}

}; // namespace recorder
}; // namespace qmmf
