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

#define TAG "SystemKeytone"

#include "system/src/service/qmmf_system_keytone.h"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/inc/qmmf_audio_endpoint.h"
#include "common/qmmf_log.h"
#include "qmmf-sdk/qmmf_system_params.h"
#include "system/src/service/qmmf_system_common.h"

namespace qmmf {
namespace system {

using ::qmmf::DeviceId;
using ::qmmf::common::audio::AudioBuffer;
using ::qmmf::common::audio::AudioEndPoint;
using ::qmmf::common::audio::AudioEndPointType;
using ::qmmf::common::audio::AudioEventHandler;
using ::qmmf::common::audio::AudioFlag;
using ::qmmf::common::audio::AudioMetadata;
using ::qmmf::common::audio::AudioEventType;
using ::qmmf::common::audio::AudioEventData;
using ::std::chrono::milliseconds;
using ::std::mutex;
using ::std::thread;
using ::std::unique_lock;
using ::std::vector;

SystemKeytone::SystemKeytone()
    : end_point_(nullptr), thread_(nullptr), current_handle_(0) {}

SystemKeytone::~SystemKeytone() {}

status_t SystemKeytone::PlayTone(const SystemHandle system_handle,
                                 const vector<DeviceId>& devices,
                                 const Tone& tone,
                                 const SystemToneHandler& handler) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);
  for (const DeviceId& device : devices)
    QMMF_VERBOSE("%s: %s() INPARAM: device[%d]", TAG, __func__, device);
  QMMF_VERBOSE("%s: %s() INPARAM: tone[%s]", TAG, __func__,
               tone.ToString().c_str());
  int32_t result;
  int32_t buffer_size;
  int32_t number_of_buffers;

  if (current_handle_ != 0 || end_point_ != nullptr) {
    QMMF_ERROR("%s: %s() endpoint already exists", TAG, __func__);
    return ::android::ALREADY_EXISTS;
  }

  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
    thread_ = nullptr;
  }

  current_handle_ = system_handle;
  tone_handler_ = handler;

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
  metadata.num_channels = 1;
  metadata.sample_rate = 48000;
  metadata.sample_size = 16;
  metadata.flags = static_cast<uint32_t>(AudioFlag::kFlagLowLatency);

  result = end_point_->Configure(AudioEndPointType::kSink, devices, metadata);
  if (result < 0) {
    QMMF_ERROR("%s: %s() endpoint->Configure failed: %d[%s]", TAG, __func__,
               result, strerror(result));
    goto error_disconnect;
  }

  result = end_point_->GetBufferSize(&buffer_size);
  if (result < 0) {
    QMMF_ERROR("%s: %s() endpoint->GetBufferSize failed: %d[%s]", TAG, __func__,
               result, strerror(result));
    goto error_disconnect;
  }
  if (buffer_size == 0) {
    QMMF_ERROR("%s: %s() endpoint->GetBufferSize returned 0", TAG, __func__);
    goto error_disconnect;
  }
  QMMF_INFO("%s: %s() buffer_size is %d", TAG, __func__, buffer_size);

  number_of_buffers = tone.size / buffer_size;
  if (tone.size % buffer_size != 0) ++number_of_buffers;
  result = ion_.Allocate(number_of_buffers, buffer_size);
  if (result < 0) {
    QMMF_ERROR("%s: %s() ion->Allocate failed: %d[%s]", TAG, __func__, result,
               strerror(result));
    goto error_deallocate;
  }

  memset(&tone_, 0x0, sizeof tone_);
  tone_.buffer = calloc(1, tone.size);
  if (tone_.buffer == NULL) {
    QMMF_ERROR("%s: %s() could not allocate memory", TAG, __func__);
    goto error_malloc;
  }
  memcpy(tone_.buffer, tone.buffer, tone.size);
  tone_.size = tone.size;
  tone_.loop_num = tone.loop_num;
  tone_.delay = tone.delay;

  while (!messages_.empty())
    messages_.pop();

  thread_ = new thread(SystemKeytone::ThreadEntry, this);
  if (thread_ == nullptr) {
    QMMF_ERROR("%s: %s() could not instantiate thread", TAG, __func__);
    goto error_malloc;
  }

  return ::android::NO_ERROR;

error_malloc:
  free(tone_.buffer);

error_deallocate:
  result = ion_.Deallocate();
  if (result < 0)
    QMMF_ERROR("%s: %s() ion->Deallocate failed: %d[%s]", TAG, __func__, result,
               strerror(result));

error_disconnect:
  result = end_point_->Disconnect();
  if (result < 0)
    QMMF_ERROR("%s: %s() endpoint->Disconnect failed: %d[%s]", TAG, __func__,
               result, strerror(result));

error_free:
  delete end_point_;
  end_point_ = nullptr;

  current_handle_ = 0;
  tone_handler_ = nullptr;

  return ::android::FAILED_TRANSACTION;
}

void SystemKeytone::ErrorHandler(const int32_t error) {
  QMMF_DEBUG("%s: %s() TRACE: current_handle[%d]", TAG, __func__,
             current_handle_);
  QMMF_VERBOSE("%s: %s() INPARAM: error[%d]", TAG, __func__, error);

  QMMF_ERROR("%s: %s() received error from endpoint: %d[%s]", TAG, __func__,
               error, strerror(error));

  SystemMessage message;
  message.type = SystemMessageType::kMessageError;
  message.error = error;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();
}

void SystemKeytone::BufferHandler(const AudioBuffer& buffer) {
  QMMF_DEBUG("%s: %s() TRACE: current_handle[%d]", TAG, __func__,
             current_handle_);
  QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
               buffer.ToString().c_str());

  SystemMessage message;
  message.type = SystemMessageType::kMessageBuffer;
  message.buffer = buffer;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();
}

void SystemKeytone::ThreadEntry(SystemKeytone* source) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  source->Thread();
}

void SystemKeytone::Thread() {
  QMMF_DEBUG("%s: %s() TRACE: current_handle[%d]", TAG, __func__,
             current_handle_);
  vector<AudioBuffer> buffers;
  int32_t result;

  ion_.GetList(&buffers);
  uint32_t idx = 0;
  for (auto&& buffer : buffers) {
    uint8_t* local_pointer = reinterpret_cast<uint8_t*>(tone_.buffer);
    if (idx == buffers.size() - 1) {
      memset(buffer.data, 0x0, buffer.capacity);
      memcpy(buffer.data, local_pointer + (idx * buffer.capacity),
             tone_.size % buffer.capacity);
      buffer.size = tone_.size % buffer.capacity;
    } else {
      memcpy(buffer.data, local_pointer + (idx * buffer.capacity),
             buffer.capacity);
      buffer.size = buffer.capacity;
    }
    ++idx;
  }
  free(tone_.buffer);
  size_t number_of_buffers = idx;

  bool error_detected = false;
  for (uint32_t loop_idx = 0; loop_idx < tone_.loop_num; ++loop_idx) {
    if (tone_.delay != 0)
      ::std::this_thread::sleep_for(::std::chrono::milliseconds(tone_.delay));

    int32_t result = end_point_->Start();
    if (result < 0) {
      QMMF_ERROR("%s: %s() endpoint->Start failed: %d[%s]", TAG, __func__,
                 result, strerror(result));
      tone_handler_(current_handle_, result);
      error_detected = true;
      break;
    }

    result = end_point_->SendBuffers(buffers);
    if (result < 0) {
      QMMF_ERROR("%s: %s() endpoint->SendBuffers failed: %d[%s]", TAG, __func__,
                 result, strerror(result));
      tone_handler_(current_handle_, result);
      error_detected = true;
      break;
    }
    buffers.clear();

    bool keep_running = true;
    while (keep_running) {
      // wait until there is something to do
      if (messages_.empty()) {
        unique_lock<mutex> lk(message_lock_);
        signal_.wait(lk);
      }

      // process the next pending message
      message_lock_.lock();
      if (!messages_.empty()) {
        SystemMessage message = messages_.front();

        switch (message.type) {
          case SystemMessageType::kMessageError:
            QMMF_DEBUG("%s: %s-MessageError() TRACE", TAG, __func__);
            QMMF_VERBOSE("%s: %s() INPARAM: error[%d]", TAG, __func__,
                         message.error);
            tone_handler_(current_handle_, message.error);
            keep_running = false;
            error_detected = true;
            break;

          case SystemMessageType::kMessageBuffer:
            QMMF_DEBUG("%s: %s-MessageBuffer() TRACE", TAG, __func__);
            QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s] to vector[%u]",
                         TAG, __func__, message.buffer.ToString().c_str(),
                         buffers.size());
            message.buffer.size = message.buffer.capacity;
            buffers.push_back(message.buffer);
            QMMF_VERBOSE("%s: %s() buffers vector is now %u deep",
                         TAG, __func__, buffers.size());
            break;
        }
        messages_.pop();
      }
      message_lock_.unlock();

      if (buffers.size() == number_of_buffers)
        keep_running = false;
    }

    if (error_detected)
      break;

    result = end_point_->Stop(false);
    if (result < 0) {
      QMMF_ERROR("%s: %s() endpoint->Stop failed: %d[%s]", TAG, __func__,
                 result, strerror(result));
      tone_handler_(current_handle_, result);
      error_detected = true;
      break;
    }
  }

  result = end_point_->Stop(false);
  if (result < 0)
    QMMF_ERROR("%s: %s() endpoint->Stop failed: %d[%s]", TAG, __func__,
               result, strerror(result));

  result = end_point_->Disconnect();
  if (result < 0)
    QMMF_ERROR("%s: %s() endpoint->Disconnect failed: %d[%s]", TAG, __func__,
               result, strerror(result));

  result = ion_.Deallocate();
  if (result < 0)
    QMMF_ERROR("%s: %s() ion->Deallocate failed: %d[%s]", TAG, __func__, result,
               strerror(result));

  delete end_point_;
  end_point_ = nullptr;

  if (!error_detected)
    tone_handler_(current_handle_, 0);

  current_handle_ = 0;
  tone_handler_ = nullptr;
}

}; // namespace system
}; // namespace qmmf
