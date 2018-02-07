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

#define TAG "RecorderAudioEncodedTrackSource"

#include "recorder/src/service/qmmf_audio_track_source.h"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/inc/qmmf_audio_endpoint.h"
#include "common/codecadaptor/src/qmmf_avcodec.h"
#include "common/qmmf_log.h"
#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_recorder_ion.h"

namespace qmmf {
namespace recorder {

using ::qmmf::common::audio::AudioBuffer;
using ::qmmf::common::audio::AudioEndPoint;
using ::qmmf::common::audio::AudioEndPointType;
using ::qmmf::common::audio::AudioEventHandler;
using ::qmmf::common::audio::AudioMetadata;
using ::qmmf::common::audio::AudioEventType;
using ::qmmf::common::audio::AudioEventData;
using ::qmmf::avcodec::CodecPortStatus;
using ::qmmf::avcodec::PortEventType;
using ::qmmf::avcodec::PortreconfigData;
using ::std::chrono::seconds;
using ::std::condition_variable;
using ::std::cv_status;
using ::std::mutex;
using ::std::queue;
using ::std::thread;
using ::std::unique_lock;
using ::std::vector;

static const int kNumberOfBuffers = INPUT_MAX_COUNT;

AudioEncodedTrackSource::AudioEncodedTrackSource(const AudioTrackParams& params)
    : track_params_(params),
      end_point_(nullptr),
      buffer_size_(0),
      stop_called_(true),
      stop_notify_received_(true) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: params[%s]", TAG, __func__,
               params.ToString().c_str());
}

AudioEncodedTrackSource::~AudioEncodedTrackSource() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
}

status_t AudioEncodedTrackSource::Init() {
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
    goto error_init_free;
  }

  AudioMetadata metadata;
  memset(&metadata, 0x0, sizeof metadata);
  metadata.format = AudioFormat::kPCM;
  metadata.num_channels = track_params_.params.channels;
  metadata.sample_rate = track_params_.params.sample_rate;
  metadata.sample_size = track_params_.params.bit_depth;
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
    goto error_init_disconnect;
  }

  int32_t buffer_size;
  result = end_point_->GetBufferSize(&buffer_size);
  if (result < 0) {
    QMMF_ERROR("%s: %s() endpoint->GetBufferSize failed: %d[%s]", TAG, __func__,
               result, strerror(result));
    goto error_init_disconnect;
  }
  QMMF_INFO("%s: %s() buffer_size is %d", TAG, __func__, buffer_size);
  buffer_size_ = buffer_size;

  result = ion_.Allocate(kNumberOfBuffers, buffer_size_);
  if (result < 0) {
    QMMF_ERROR("%s: %s() ion->Allocate failed: %d[%s]", TAG, __func__, result,
               strerror(result));
    goto error_init_deallocate;
  }

  return ::android::NO_ERROR;

error_init_deallocate:
  ion_.Deallocate();

error_init_disconnect:
  end_point_->Disconnect();

error_init_free:
  delete end_point_;
  end_point_ = nullptr;

  return ::android::FAILED_TRANSACTION;
}

status_t AudioEncodedTrackSource::DeInit() {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);
  int32_t result;

  result = ion_.Deallocate();
  if (result < 0)
    QMMF_ERROR("%s: %s() ion->Deallocate failed: %d[%s]", TAG, __func__,
               result, strerror(result));

  result = end_point_->Disconnect();
  if (result < 0)
    QMMF_ERROR("%s: %s() endpoint->Disconnect failed: %d[%s]", TAG, __func__,
               result, strerror(result));

  delete end_point_;
  end_point_ = nullptr;

  return ::android::NO_ERROR;
}

status_t AudioEncodedTrackSource::StartTrack() {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);
  vector<AudioBuffer> initial_buffers;
  int32_t result;

  result = end_point_->Start();
  if (result < 0) {
    QMMF_ERROR("%s: %s() endpoint->Start failed: %d[%s]", TAG, __func__,
               result, strerror(result));
    goto error_start_stop;
  }

  // send the initial list of buffers
  result = ion_.GetList(&initial_buffers);
  if (result < 0) {
    QMMF_ERROR("%s: %s() ion->GetList failed: %d[%s]", TAG, __func__, result,
               strerror(result));
    goto error_start_stop;
  }

  result = end_point_->SendBuffers(initial_buffers);
  if (result < 0) {
    QMMF_ERROR("%s: %s() endpoint->SendBuffers failed: %d[%s]", TAG, __func__,
               result, strerror(result));
    goto error_start_stop;
  }

  stop_called_ = false;
  stop_notify_received_ = false;

  return ::android::NO_ERROR;

error_start_stop:
  StopTrack();

  return ::android::FAILED_TRANSACTION;
}

status_t AudioEncodedTrackSource::StopTrack() {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);

  int32_t result = end_point_->Stop(true);
  if (result < 0) {
    QMMF_ERROR("%s: %s() endpoint->Stop failed: %d[%s]", TAG, __func__,
               result, strerror(result));
    return ::android::FAILED_TRANSACTION;
  }

  stop_called_ = true;

  return ::android::NO_ERROR;
}

status_t AudioEncodedTrackSource::PauseTrack() {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);

  int32_t result = end_point_->Pause();
  if (result < 0) {
    QMMF_ERROR("%s: %s() endpoint->Pause failed: %d[%s]", TAG, __func__,
               result, strerror(result));
    return ::android::FAILED_TRANSACTION;
  }

  return ::android::NO_ERROR;
}

status_t AudioEncodedTrackSource::ResumeTrack() {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);

  int32_t result = end_point_->Resume();
  if (result < 0) {
    QMMF_ERROR("%s: %s() endpoint->Resume failed: %d[%s]", TAG, __func__,
               result, strerror(result));
    return ::android::FAILED_TRANSACTION;
  }

  return ::android::NO_ERROR;
}

status_t AudioEncodedTrackSource::ReturnTrackBuffer(
    const std::vector<BnBuffer> &buffers) {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);
  for (const BnBuffer& buffer : buffers)
    QMMF_VERBOSE("%s: %s() INPARAM: bn_buffer[%s]", TAG, __func__,
                 buffer.ToString().c_str());

  QMMF_ERROR("%s: %s() unsupported operation", TAG, __func__);
  assert(0);

  return ::android::INVALID_OPERATION;
}

status_t AudioEncodedTrackSource::GetBuffer(BufferDescriptor& buffer,
                                            void* client_data) {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);

  while (buffers_.empty() && !stop_notify_received_) {
    unique_lock<mutex> lk(mutex_);
    if (signal_.wait_for(lk, seconds(1)) == cv_status::timeout)
      QMMF_WARN("%s: %s() timed out on wait", TAG, __func__);
  }

  if (stop_notify_received_) {
    QMMF_WARN("%s: %s() request for buffer after stop_notify", TAG, __func__);
    return ::android::NO_ERROR;
  }

  mutex_.lock();
  buffer = buffers_.front();
  buffers_.pop();
  mutex_.unlock();

  QMMF_VERBOSE("%s: %s() OUTPARAM: buffer[%s]", TAG, __func__,
               buffer.ToString().c_str());
  if (buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS))
    return -1;
  else
    return ::android::NO_ERROR;
}

status_t AudioEncodedTrackSource::ReturnBuffer(BufferDescriptor& buffer,
                                               void* client_data) {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);
  QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
               buffer.ToString().c_str());
  int32_t result;

  if (stop_called_ == true) {
    QMMF_VERBOSE("%s: %s() throwning away the buffer", TAG, __func__);
    return ::android::NO_ERROR;
  }

  AudioBuffer audio_buffer;
  memset(&audio_buffer, 0x00, sizeof audio_buffer);
  result = ion_.Import(buffer, &audio_buffer);
  if (result < 0) {
    QMMF_ERROR("%s: %s() ion->Import failed: %d[%s]", TAG, __func__, result,
               strerror(result));
    return ::android::FAILED_TRANSACTION;
  }

  memset(audio_buffer.data, 0x00, audio_buffer.capacity);
  audio_buffer.size = 0;
  audio_buffer.timestamp = 0;

  result = end_point_->SendBuffers({audio_buffer});
  if (result < 0 && !stop_called_) {
    QMMF_ERROR("%s: %s() endpoint->SendBuffers failed: %d[%s]", TAG,
               __func__, result, strerror(result));
    return ::android::FAILED_TRANSACTION;
  }

  return ::android::NO_ERROR;
}

status_t AudioEncodedTrackSource::NotifyPortEvent(PortEventType event_type,
                                                  void* event_data) {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);
  if (event_type == PortEventType::kPortStatus) {
    CodecPortStatus status = *(static_cast<CodecPortStatus*>(event_data));
    switch (status) {
      case CodecPortStatus::kPortStop:
        if (stop_called_)
          stop_notify_received_ = true;
        break;
      case CodecPortStatus::kPortIdle:
        if (stop_notify_received_) {
          mutex_.lock();
          while (!buffers_.empty())
            buffers_.pop();
          mutex_.unlock();
          signal_.notify_one();
          QMMF_VERBOSE("%s: %s() emptied the buffer queue", TAG, __func__);
        }
        break;
      case CodecPortStatus::kPortStart:
        break;
    }
  }

  return ::android::NO_ERROR;
}

status_t AudioEncodedTrackSource::GetBufferSize(int32_t* buffer_size) {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);

  *buffer_size = buffer_size_;
  QMMF_VERBOSE("%s: %s() OUTPARAM: buffer_size[%d]", TAG, __func__,
               *buffer_size);

  return ::android::NO_ERROR;
}

status_t AudioEncodedTrackSource::SetBufferSize(const int32_t buffer_size) {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);
  QMMF_VERBOSE("%s: %s() INPARAM: buffer_size[%d]", TAG, __func__, buffer_size);

  buffer_size_ = buffer_size;

  return ::android::NO_ERROR;
}

void AudioEncodedTrackSource::ErrorHandler(const int32_t error) {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);
  QMMF_VERBOSE("%s: %s() INPARAM: type[%d]", TAG, __func__, error);

  QMMF_ERROR("%s: %s() received error from endpoint: %d[%s]", TAG, __func__,
               error, strerror(error));
  assert(false);
  // TODO(kwestfie@codeaurora.org): send notification to application instead
}

void AudioEncodedTrackSource::BufferHandler(const AudioBuffer& buffer) {
  QMMF_DEBUG("%s: %s() TRACE: track_id[%u]", TAG, __func__,
             track_params_.track_id);
  QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
               buffer.ToString().c_str());

  BufferDescriptor stream_buffer;
  memset(&stream_buffer, 0x00, sizeof stream_buffer);
  int32_t result = ion_.Export(buffer, &stream_buffer);
  if (result < 0)
    QMMF_ERROR("%s: %s() ion->Export failed: %d[%s]", TAG, __func__, result,
               strerror(result));

  mutex_.lock();
  buffers_.push(stream_buffer);
  mutex_.unlock();
  signal_.notify_one();
}

}; // namespace recorder
}; // namespace qmmf
