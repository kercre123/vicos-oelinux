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

#define LOG_TAG "AudioRawSink"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "common/utils/qmmf_log.h"
#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/inc/qmmf_audio_endpoint.h"
#include "player/src/service/qmmf_player_common.h"
#include "player/src/service/qmmf_player_ion.h"
#include "player/src/service/qmmf_player_audio_raw_sink.h"

#define NUMBER_OF_SINK_BUFFERS (4)

namespace qmmf {
namespace player {

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
using ::std::chrono::milliseconds;
using ::std::chrono::seconds;
using ::std::condition_variable;
using ::std::cv_status;
using ::std::make_shared;
using ::std::map;
using ::std::mutex;
using ::std::queue;
using ::std::shared_ptr;
using ::std::string;
using ::std::thread;
using ::std::this_thread::sleep_for;
using ::std::unique_lock;
using ::std::vector;

AudioRawSink* AudioRawSink::instance_ = nullptr;

AudioRawSink* AudioRawSink::CreateAudioRawSink() {
  QMMF_DEBUG("%s() TRACE", __func__);

  if (instance_ == nullptr) {
    instance_ = new AudioRawSink();
    if (instance_ == nullptr)
      QMMF_ERROR("%s() can't instantiate AudioRawSink", __func__);
  }
  QMMF_INFO("%s: AudioRawSink successfully retrieved", __func__);

  return instance_;
}

AudioRawSink::AudioRawSink() {
  QMMF_DEBUG("%s() TRACE", __func__);
}

AudioRawSink::~AudioRawSink() {
  QMMF_DEBUG("%s() TRACE", __func__);

  if (!track_sink_map_.empty())
    track_sink_map_.clear();

  instance_ = nullptr;
}

status_t AudioRawSink::CreateTrackSink(uint32_t track_id,
                                       AudioTrackParams& param,
                                       TrackCb& callback) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);
  QMMF_VERBOSE("%s() INPARAM: param[%s]", __func__,
               param.ToString().c_str());

  AudioTrackSinkMap::iterator track_sink_iterator =
      track_sink_map_.find(track_id);
  if (track_sink_iterator != track_sink_map_.end()) {
    QMMF_ERROR("%s() track already exists for track_id[%u]",
               __func__, track_id);
    return ::android::BAD_VALUE;
  }

  shared_ptr<AudioRawTrackSink> track_sink = make_shared<AudioRawTrackSink>();
  if (track_sink == nullptr) {
    QMMF_ERROR("%s() could not instantiate track_sink[%u]",
               __func__, track_id);
    return ::android::NO_MEMORY;
  }

  status_t result = track_sink->Init(param, callback);
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() track_sink[%u]->Init failed: %d",
               __func__, track_id, result);
    return result;
  }

  track_sink_map_.insert({track_id, track_sink});

  return ::android::NO_ERROR;
}

status_t AudioRawSink::DeleteTrackSink(uint32_t track_id) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);

  AudioTrackSinkMap::iterator track_sink_iterator =
      track_sink_map_.find(track_id);
  if (track_sink_iterator == track_sink_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  status_t result = track_sink_iterator->second->DeInit();
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() track_sink[%u]->DeInit failed: %d",
               __func__, track_id, result);
    return result;
  }

  track_sink_iterator->second = nullptr;
  track_sink_map_.erase(track_sink_iterator->first);

  return ::android::NO_ERROR;
}

status_t AudioRawSink::StartTrackSink(uint32_t track_id) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);

  AudioTrackSinkMap::iterator track_sink_iterator =
      track_sink_map_.find(track_id);
  if (track_sink_iterator == track_sink_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  status_t result = track_sink_iterator->second->StartSink();
  if (result != NO_ERROR) {
    QMMF_ERROR("%s() track_sink[%u]->StartSink failed: %d",
               __func__, track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioRawSink::StopTrackSink(uint32_t track_id) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);

  AudioTrackSinkMap::iterator track_sink_iterator =
      track_sink_map_.find(track_id);
  if (track_sink_iterator == track_sink_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  status_t result = track_sink_iterator->second->StopSink();
  if (result != NO_ERROR) {
    QMMF_ERROR("%s() track_sink[%u]->StopSink failed: %d",
               __func__, track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioRawSink::PauseTrackSink(uint32_t track_id) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);

  AudioTrackSinkMap::iterator track_sink_iterator =
      track_sink_map_.find(track_id);
  if (track_sink_iterator == track_sink_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  status_t result = track_sink_iterator->second->PauseSink();
  if (result != NO_ERROR) {
    QMMF_ERROR("%s() track_sink[%u]->PauseSink failed: %d",
               __func__, track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioRawSink::ResumeTrackSink(uint32_t track_id) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);

  AudioTrackSinkMap::iterator track_sink_iterator =
      track_sink_map_.find(track_id);
  if (track_sink_iterator == track_sink_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  status_t result = track_sink_iterator->second->ResumeSink();
  if (result != NO_ERROR) {
    QMMF_ERROR("%s() track_sink[%u]->ResumeSink failed: %d",
               __func__, track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioRawSink::SetAudioTrackSinkParams(uint32_t track_id,
                                               CodecParamType param_type,
                                               void* param,
                                               uint32_t param_size) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);

  AudioTrackSinkMap::iterator track_sink_iterator =
      track_sink_map_.find(track_id);
  if (track_sink_iterator == track_sink_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  status_t result = track_sink_iterator->second->SetAudioSinkParams(param_type,
                                                                    param,
                                                                    param_size);
  if (result != NO_ERROR) {
    QMMF_ERROR("%s() track_sink[%u]->SetAudioSinkParams failed: %d",
               __func__, track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioRawSink::DequeueTrackInputBuffer(uint32_t track_id,
                                               vector<AVCodecBuffer>& buffers) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);
  for (const AVCodecBuffer& buffer : buffers)
    QMMF_VERBOSE("%s() INPARAM: buffer[%s]", __func__,
                 buffer.ToString().c_str());

  AudioTrackSinkMap::iterator track_sink_iterator =
      track_sink_map_.find(track_id);
  if (track_sink_iterator == track_sink_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  status_t result = track_sink_iterator->second->DequeueInputBuffer(buffers);
  if (result != NO_ERROR) {
    QMMF_ERROR("%s() track_sink[%u]->DequeueInputBuffer failed: %d",
               __func__, track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioRawSink::QueueTrackInputBuffer(uint32_t track_id,
                                             vector<AVCodecBuffer>& buffers) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);
  for (const AVCodecBuffer& buffer : buffers)
    QMMF_VERBOSE("%s() INPARAM: buffer[%s]", __func__,
                 buffer.ToString().c_str());

  AudioTrackSinkMap::iterator track_sink_iterator =
      track_sink_map_.find(track_id);
  if (track_sink_iterator == track_sink_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  status_t result = track_sink_iterator->second->QueueInputBuffer(buffers);
  if (result != NO_ERROR) {
    QMMF_ERROR("%s() track_sink[%u]->QueueInputBuffer failed: %d",
               __func__, track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

AudioRawTrackSink::AudioRawTrackSink()
    : end_point_(nullptr),
      thread_(nullptr),
      pts_thread_(nullptr) {
  QMMF_DEBUG("%s() TRACE", __func__);
}

AudioRawTrackSink::~AudioRawTrackSink() {
  QMMF_DEBUG("%s() TRACE", __func__);
}

status_t AudioRawTrackSink::Init(const AudioTrackParams& params,
                                 TrackCb& callback) {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__, params.track_id);
  int32_t result;
  vector<DeviceId> devices;

  if (end_point_ != nullptr) {
    QMMF_ERROR("%s() endpoint already exists", __func__);
    return ::android::ALREADY_EXISTS;
  }

  callback_ = callback;
  track_params_ = params;

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
          StoppedHandler();
          break;
      }
    };

  result = end_point_->Connect(audio_handler);
  if (result < 0) {
    QMMF_ERROR("%s() endpoint->Connect failed: %d[%s]", __func__,
               result, strerror(result));
    goto error_free;
  }

  AudioMetadata metadata;
  memset(&metadata, 0x0, sizeof metadata);
  if (track_params_.params.codec == AudioFormat::kPCM) {
    metadata.format = AudioFormat::kPCM;
  } else if (track_params_.params.codec == AudioFormat::kMP3) {
    metadata.format = AudioFormat::kMP3;
  } else if (track_params_.params.codec == AudioFormat::kAAC) {
    metadata.format = AudioFormat::kAAC;
  } else {
    QMMF_ERROR("%s() invalid codec given %d", __func__,
               static_cast<int32_t>(track_params_.params.codec));
    goto error_disconnect;
  }
  metadata.num_channels = track_params_.params.channels;
  metadata.sample_rate = track_params_.params.sample_rate;
  metadata.sample_size = track_params_.params.bit_depth;

  devices.push_back(static_cast<DeviceId>(track_params_.params.out_device));

  result = end_point_->Configure(AudioEndPointType::kSink, devices, metadata);
  if (result < 0) {
    QMMF_ERROR("%s() endpoint->Configure failed: %d[%s]", __func__,
               result, strerror(result));
    goto error_disconnect;
  }

  int32_t latency;
  result = end_point_->GetLatency(&latency);
  if (result < 0) {
    QMMF_ERROR("%s() endpoint->GetLatency failed: %d[%s]", __func__,
               result, strerror(result));
    goto error_disconnect;
  }
  QMMF_INFO("%s() latency is %d", __func__, latency);

  int32_t buffer_size;
  result = end_point_->GetBufferSize(&buffer_size);
  if (result < 0) {
    QMMF_ERROR("%s() endpoint->GetBufferSize failed: %d[%s]", __func__,
               result, strerror(result));
    goto error_disconnect;
  }
  QMMF_INFO("%s() buffer_size is %d", __func__, buffer_size);

  result = ion_.Allocate(NUMBER_OF_SINK_BUFFERS, buffer_size);
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

status_t AudioRawTrackSink::DeInit() {
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

status_t AudioRawTrackSink::StartSink() {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);

  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
    thread_ = nullptr;
  }

  int32_t result = end_point_->Start();
  if (result < 0) {
    QMMF_ERROR("%s() endpoint->Start failed: %d[%s]", __func__,
               result, strerror(result));
    return ::android::FAILED_TRANSACTION;
  }

  while (!messages_.empty())
    messages_.pop();

  thread_ = new thread(AudioRawTrackSink::ThreadEntry, this);
  if (thread_ == nullptr) {
    QMMF_ERROR("%s() could not instantiate thread", __func__);
    end_point_->Stop();
    return ::android::NO_MEMORY;
  }

  if (track_params_.params.pts_callback_interval != 0) {
    while (!pts_messages_.empty())
      pts_messages_.pop();

    pts_thread_ = new thread(AudioRawTrackSink::PtsThreadEntry, this);
    if (pts_thread_ == nullptr) {
      QMMF_ERROR("%s() could not instantiate PTS thread", __func__);
      end_point_->Stop();

      AudioMessage message;
      message.type = AudioMessageType::kMessageStop;

      message_lock_.lock();
      messages_.push(message);
      message_lock_.unlock();
      signal_.notify_one();

      thread_->join();
      delete thread_;
      thread_ = nullptr;

      return ::android::NO_MEMORY;
    }
  }

  return ::android::NO_ERROR;
}

status_t AudioRawTrackSink::StopSink() {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);

  AudioMessage message;
  message.type = AudioMessageType::kMessageStop;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();

  if (track_params_.params.pts_callback_interval != 0) {
    pts_message_lock_.lock();
    pts_messages_.push(message);
    pts_message_lock_.unlock();
  }

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

  if (track_params_.params.pts_callback_interval != 0) {
    if (pts_thread_ != nullptr) {
      pts_thread_->join();
      delete pts_thread_;
      pts_thread_ = nullptr;
    }

    while (!pts_messages_.empty())
      pts_messages_.pop();
  }

  return ::android::NO_ERROR;
}

status_t AudioRawTrackSink::PauseSink() {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);

  AudioMessage message;
  message.type = AudioMessageType::kMessagePause;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();

  if (track_params_.params.pts_callback_interval != 0) {
    pts_message_lock_.lock();
    pts_messages_.push(message);
    pts_message_lock_.unlock();
  }

  int32_t result = end_point_->Pause();
  if (result < 0) {
    QMMF_ERROR("%s() endpoint->Pause failed: %d[%s]", __func__,
               result, strerror(result));
    return ::android::FAILED_TRANSACTION;
  }

  return ::android::NO_ERROR;
}

status_t AudioRawTrackSink::ResumeSink() {
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
  signal_.notify_one();

  if (track_params_.params.pts_callback_interval != 0) {
    pts_message_lock_.lock();
    pts_messages_.push(message);
    pts_message_lock_.unlock();
  }

  return ::android::NO_ERROR;
}

status_t AudioRawTrackSink::SetAudioSinkParams(CodecParamType param_type,
                                               void* param,
                                               uint32_t param_size) {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  auto ret = 0;

  if (param_type == CodecParamType::kAudioVolumeParamType) {
    uint32_t* volume_ptr = reinterpret_cast<uint32_t*>(param);
    ret = end_point_->SetParam(AudioParamType::kVolume, *volume_ptr);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: track_id(%d) SetAudioSinkParams failed!",
                 __func__, track_params_.track_id);
      return ret;
    }
  }

  return ::android::NO_ERROR;
}

status_t AudioRawTrackSink::DequeueInputBuffer(vector<AVCodecBuffer>& buffers) {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  QMMF_VERBOSE("%s() INPARAM: buffer size[%u]", __func__,
               buffers.size());

  uint32_t number_of_buffers = buffers.size();
  buffers.clear();

  // wait until there is data
  while (av_buffers_.size() < number_of_buffers && thread_ != nullptr) {
    unique_lock<mutex> lk(av_buffers_lock_);
    if (buffer_signal_.wait_for(lk, seconds(1)) == cv_status::timeout)
      QMMF_WARN("%s() timed out on wait for buffers", __func__);
  }
  if (thread_ == nullptr) return ::android::NO_ERROR;

  av_buffers_lock_.lock();

  for (uint32_t idx = 0; idx < number_of_buffers; ++idx) {
    AVCodecBuffer av_buffer = av_buffers_.front();
    QMMF_VERBOSE("%s() track[%u] processing next av_buffer[%s] from queue[%u]",
                 __func__, track_params_.track_id,
                 av_buffer.ToString().c_str(), av_buffers_.size());
    buffers.push_back(av_buffer);
    av_buffers_.pop();
  }

  av_buffers_lock_.unlock();

  for (const AVCodecBuffer& buffer : buffers)
    QMMF_VERBOSE("%s() OUTPARAM: buffer[%s]", __func__,
                 buffer.ToString().c_str());
  return ::android::NO_ERROR;
}

status_t AudioRawTrackSink::QueueInputBuffer(vector<AVCodecBuffer>& buffers) {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  for (const AVCodecBuffer& buffer : buffers)
    QMMF_VERBOSE("%s() INPARAM: buffer[%s]", __func__,
                 buffer.ToString().c_str());

  for (const AVCodecBuffer& buffer : buffers) {
    AudioMessage message;
    message.type = AudioMessageType::kMessageAVBuffer;
    message.av_buffer = buffer;

    message_lock_.lock();
    messages_.push(message);
    message_lock_.unlock();
    signal_.notify_one();
  }

  return ::android::NO_ERROR;
}

void AudioRawTrackSink::ErrorHandler(const int32_t error) {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  QMMF_VERBOSE("%s() INPARAM: type[%d]", __func__, error);

  QMMF_ERROR("%s() received error from endpoint: %d[%s]", __func__,
               error, strerror(error));
  assert(false);
}

void AudioRawTrackSink::BufferHandler(const AudioBuffer& buffer) {
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
  signal_.notify_one();
}

void AudioRawTrackSink::StoppedHandler() {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);

  if (track_params_.params.pts_callback_interval != 0) {
    AudioMessage message;
    message.type = AudioMessageType::kMessageStop;

    pts_message_lock_.lock();
    pts_messages_.push(message);
    pts_message_lock_.unlock();

    if (pts_thread_ != nullptr) {
      pts_thread_->join();
      delete pts_thread_;
      pts_thread_ = nullptr;
    }

    while (!pts_messages_.empty())
      pts_messages_.pop();
  }

  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
    thread_ = nullptr;
  }

  while (!messages_.empty())
    messages_.pop();

  callback_.event_cb(track_params_.track_id, EventType::kEOSRendered,
                     nullptr, 0);
}

void AudioRawTrackSink::ThreadEntry(AudioRawTrackSink* sink) {
  QMMF_DEBUG("%s() TRACE", __func__);

  sink->Thread();
}

void AudioRawTrackSink::Thread() {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  queue<AudioBuffer> buffers;
  queue<AVCodecBuffer> av_buffers;

  // get the initial list of buffers
  ion_.GetList(&av_buffers_);

  bool stop_received = false;
  bool eof_received = false;
  bool paused = false;
  bool keep_running = true;
  while (keep_running) {
    // wait until there is something to do
    if (av_buffers.empty() && buffers.empty()) {
      unique_lock<mutex> lk(message_lock_);
      if (!signal_.wait_for(lk, seconds(1), [this]{return !messages_.empty();}))
        QMMF_WARN("%s() timed out on wait", __func__);
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

        case AudioMessageType::kMessageAVBuffer:
          QMMF_DEBUG("%s-MessageAVBuffer() TRACE", __func__);
          QMMF_VERBOSE("%s() INPARAM: av_buffer[%s] to queue[%u]",
                       __func__, message.av_buffer.ToString().c_str(),
                       av_buffers.size());
          av_buffers.push(message.av_buffer);
          QMMF_VERBOSE("%s() av_buffers queue is now %u deep",
                       __func__, av_buffers.size());
          break;
      }
      messages_.pop();
    }
    message_lock_.unlock();

    // process buffers from endpoint
    if (!buffers.empty() && !paused && !stop_received && keep_running) {
      AudioBuffer buffer = buffers.front();
      QMMF_VERBOSE("%s() track[%u] processing next buffer[%s] from queue[%u]",
                   __func__, track_params_.track_id,
                   buffer.ToString().c_str(), buffers.size());


      AVCodecBuffer av_buffer;
      ion_.Export(buffer, &av_buffer);

      memset(av_buffer.data, 0x00, av_buffer.frame_length);
      av_buffer.filled_length = 0;
      av_buffer.time_stamp = 0;

      av_buffers_lock_.lock();
      av_buffers_.push(av_buffer);
      av_buffers_lock_.unlock();
      buffer_signal_.notify_one();

      buffers.pop();
      QMMF_VERBOSE("%s() buffers queue is now %u deep",
                   __func__, buffers.size());
    }

    // process buffers from client
    if (!av_buffers.empty() && !paused && !stop_received && keep_running) {
      AVCodecBuffer av_buffer = av_buffers.front();
      QMMF_VERBOSE("%s() track[%u] processing next av_buffer[%s] from queue[%u]",
                   __func__, track_params_.track_id,
                   av_buffer.ToString().c_str(), av_buffers.size());

      AudioBuffer buffer;
      ion_.Import(av_buffer, &buffer);

      int32_t result = end_point_->SendBuffers({buffer});
      if (result < 0) {
        QMMF_ERROR("%s() endpoint->SendBuffers failed: %d[%s]",
                   __func__, result, strerror(result));
        assert(false);
      }

      if (buffer.flags & static_cast<uint32_t>(BufferFlags::kFlagEOS))
        eof_received = true;

      av_buffers.pop();
      QMMF_VERBOSE("%s() av_buffers queue is now %u deep",
                   __func__, av_buffers.size());
    }

    // stop conditions
    if (stop_received || eof_received)
      keep_running = false;
  }
  QMMF_DEBUG("%s() exiting", __func__);
}

void AudioRawTrackSink::PtsThreadEntry(AudioRawTrackSink* sink) {
  QMMF_DEBUG("%s() TRACE", __func__);

  sink->PtsThread();
}

void AudioRawTrackSink::PtsThread() {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  uint64_t previous_timestamp = 0UL;

  bool paused = false;
  bool keep_running = true;
  while (keep_running) {
    sleep_for(milliseconds(track_params_.params.pts_callback_interval));

    // check for messages
    {
      unique_lock<mutex> lk(pts_message_lock_);
      while (!pts_messages_.empty()) {
        AudioMessage message = pts_messages_.front();

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
            keep_running = false;
            break;

          case AudioMessageType::kMessageBuffer:
          case AudioMessageType::kMessageAVBuffer:
            QMMF_WARN("%s-MessageBuffer() ignoring message", __func__);
            break;
        }
        pts_messages_.pop();
      }
    }

    if (!paused) {
      uint32_t frames;
      uint64_t notused;

      int32_t result = end_point_->GetRenderedPosition(&frames, &notused);
      if (result < 0) {
        QMMF_WARN("%s() endpoint->GetRenderedPosition failed: %d[%s]",
                  __func__, result, strerror(result));
      } else {
        uint64_t timestamp = frames / (track_params_.params.sample_rate / 1000);
        if (timestamp != previous_timestamp) {
          QMMF_DEBUG("%s() sending timestamp[%llu] for track[%u]", __func__, timestamp,
                     track_params_.track_id);
          callback_.event_cb(track_params_.track_id,
                             EventType::kPresentationTimestamp, &timestamp,
                             sizeof(timestamp));
        }
        previous_timestamp = timestamp;
      }
    }
  }

  QMMF_DEBUG("%s() exiting", __func__);
}

};  // namespace player
};  // namespace qmmf
