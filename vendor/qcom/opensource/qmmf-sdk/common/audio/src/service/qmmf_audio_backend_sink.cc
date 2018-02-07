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

#define TAG "AudioBackendSink"

#include "common/audio/src/service/qmmf_audio_backend_sink.h"

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <functional>
#include <map>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <mm-audio/qahw_api/inc/qahw_api.h>
#include <mm-audio/qahw_api/inc/qahw_defs.h>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/src/service/qmmf_audio_common.h"
#include "common/qmmf_log.h"

// remove comment marker to mimic the AHAL instead of using it
//#define AUDIO_BACKEND_PRIMARY_DEBUG_DATAFLOW

namespace qmmf {
namespace common {
namespace audio {

using ::std::chrono::seconds;
using ::std::condition_variable;
using ::std::cv_status;
using ::std::function;
using ::std::map;
using ::std::mutex;
using ::std::queue;
using ::std::thread;
using ::std::unique_lock;
using ::std::vector;

const audio_io_handle_t AudioBackendSink::kIOHandleMin = 800;
const audio_io_handle_t AudioBackendSink::kIOHandleMax = 899;

AudioBackendSink::AudioBackendSink(const AudioHandle audio_handle,
                                   const AudioErrorHandler& error_handler,
                                   const AudioBufferHandler& buffer_handler)
    : audio_handle_(audio_handle),
      state_(AudioState::kNew),
      error_handler_(error_handler),
      buffer_handler_(buffer_handler),
      current_io_handle_(kIOHandleMin) {
  QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
             static_cast<int>(state_));
}

AudioBackendSink::~AudioBackendSink() {}

int32_t AudioBackendSink::Open(const vector<DeviceId>& devices,
                               const AudioMetadata& metadata) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  for (const DeviceId device : devices)
    QMMF_VERBOSE("%s: %s() INPARAM: device[%d]", TAG, __func__, device);
  QMMF_VERBOSE("%s: %s() INPARAM: metadata[%s]", TAG, __func__,
               metadata.ToString().c_str());
  int32_t result = 0;

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

#ifndef AUDIO_BACKEND_PRIMARY_DEBUG_DATAFLOW
  int qahw_version = qahw_get_version();
  if (qahw_version < QAHW_MODULE_API_VERSION_MIN) {
    QMMF_ERROR("%s: %s() incorrect QAHW module version[%d]", TAG, __func__,
               qahw_version);
    return -EPERM;
  }
  QMMF_INFO("%s: %s() QAHW module version[%d]", TAG, __func__, qahw_version);

  qahw_module_ = qahw_load_module(QAHW_MODULE_ID_PRIMARY);
  if (qahw_module_ == nullptr) {
    QMMF_ERROR("%s: %s() failed to load QAHW module[%s]", TAG, __func__,
               QAHW_MODULE_ID_PRIMARY);
    return -ENOMEM;
  }

  result = qahw_init_check(qahw_module_);
  if (result != 0) {
    QMMF_ERROR("%s: %s() QAHW module initialization failed: %d[%s]",
               TAG, __func__, result, strerror(result));
    return result;
  }

  audio_config_t config = AUDIO_CONFIG_INITIALIZER;
  switch (metadata.sample_size) {
    case 16:
      config.format = AUDIO_FORMAT_PCM_16_BIT;
      break;
    case 32:
      config.format = AUDIO_FORMAT_PCM_32_BIT;
      break;
    default:
      QMMF_ERROR("%s: %s() invalid sample size: %d", TAG, __func__,
                 metadata.sample_size);
      return -EINVAL;
  }
  config.sample_rate = metadata.sample_rate;
  config.channel_mask = audio_channel_out_mask_from_count(metadata.num_channels);
  config.frame_count = 0;

  if ((metadata.flags & static_cast<uint32_t>(AudioFlag::kFlagLowLatency)) == 0) {
    config.offload_info.version = AUDIO_OFFLOAD_INFO_VERSION_CURRENT;
    config.offload_info.size = sizeof(audio_offload_info_t);
    config.offload_info.sample_rate = config.sample_rate;
    config.offload_info.channel_mask = config.channel_mask;
    config.offload_info.format = config.format;
  }

  // use the next available io_handle
  if (current_io_handle_ + 1 > kIOHandleMax)
    current_io_handle_ = kIOHandleMin;
  ++current_io_handle_;

  audio_output_flags_t flags;
  if ((metadata.flags & static_cast<uint32_t>(AudioFlag::kFlagLowLatency)) != 0)
    flags = AUDIO_OUTPUT_FLAG_FAST;
  else
    flags = AUDIO_OUTPUT_FLAG_COMPRESS_OFFLOAD;

  result = qahw_open_output_stream(qahw_module_, current_io_handle_,
                                   AUDIO_DEVICE_OUT_SPEAKER, flags, &config,
                                   &qahw_stream_, "output_stream");
  if (result != 0) {
    QMMF_ERROR("%s: %s() failed to open output stream: %d[%s]", TAG, __func__,
               result, strerror(result));
    return result;
  }
#endif

  state_ = AudioState::kIdle;
  QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
             static_cast<int>(state_));

  return result;
}

int32_t AudioBackendSink::Close() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  int32_t result = 0;

  switch (state_) {
    case AudioState::kNew:
      QMMF_WARN("%s: %s() nothing to do, state is: %d", TAG, __func__,
                static_cast<int>(state_));
      return 0;
      break;
    case AudioState::kIdle:
      // proceed
      break;
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

#ifndef AUDIO_BACKEND_PRIMARY_DEBUG_DATAFLOW
  result = qahw_close_output_stream(qahw_stream_);
  if (result != 0) {
    QMMF_ERROR("%s: %s() failed to close output stream: %d[%s]",
               TAG, __func__, result, strerror(result));
    return result;
  }

  result = qahw_unload_module(qahw_module_);
  if (result != 0) {
    QMMF_ERROR("%s: %s() failed to unload QAHW module: %d[%s]",
               TAG, __func__, result, strerror(result));
    return result;
  }
#endif

  state_ = AudioState::kNew;
  QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
             static_cast<int>(state_));

  return result;
}

int32_t AudioBackendSink::Start() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

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

  while (!messages_.empty())
    messages_.pop();

  thread_ = new thread(AudioBackendSink::ThreadEntry, this);
  if (thread_ == nullptr) {
    QMMF_ERROR("%s: %s() unable to allocate thread", TAG, __func__);
    return -ENOMEM;
  }

  state_ = AudioState::kRunning;
  QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
             static_cast<int>(state_));

  return 0;
}

int32_t AudioBackendSink::Stop(const bool flush) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: flush[%s]", TAG, __func__,
               flush ? "true" : "false");

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

  AudioMessage message;
  message.type = AudioMessageType::kMessageStop;
  message.flush = flush;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();

  thread_->join();
  delete thread_;

  while (!messages_.empty())
    messages_.pop();

  state_ = AudioState::kIdle;
  QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
             static_cast<int>(state_));

  return 0;
}

int32_t AudioBackendSink::Pause() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

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

  AudioMessage message;
  message.type = AudioMessageType::kMessagePause;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();

  state_ = AudioState::kPaused;
  QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
             static_cast<int>(state_));

  return 0;
}

int32_t AudioBackendSink::Resume() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

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

  AudioMessage message;
  message.type = AudioMessageType::kMessageResume;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();

  state_ = AudioState::kRunning;
  QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
             static_cast<int>(state_));

  return 0;
}

int32_t AudioBackendSink::SendBuffers(const vector<AudioBuffer>& buffers) {
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

  AudioMessage message;
  message.type = AudioMessageType::kMessageBuffer;
  for (const AudioBuffer& buffer : buffers)
    message.buffers.push_back(buffer);

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();

  return 0;
}

int32_t AudioBackendSink::GetLatency(int32_t* latency) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

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

#ifndef AUDIO_BACKEND_PRIMARY_DEBUG_DATAFLOW
  *latency = qahw_out_get_latency(qahw_stream_);
#else
  *latency = 11;
#endif

  QMMF_VERBOSE("%s: %s() OUTPARAM: latency[%d]", TAG, __func__, *latency);
  return 0;
}

int32_t AudioBackendSink::GetBufferSize(int32_t* buffer_size) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

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

#ifndef AUDIO_BACKEND_PRIMARY_DEBUG_DATAFLOW
  *buffer_size = qahw_out_get_buffer_size(qahw_stream_);
#else
  *buffer_size = 16;
#endif

  QMMF_VERBOSE("%s: %s() OUTPARAM: buffer_size[%d]", TAG, __func__,
               *buffer_size);
  return 0;
}

int32_t AudioBackendSink::SetParam(const AudioParamType type,
                                   const AudioParamData& data) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: type[%d]", TAG, __func__,
               static_cast<int>(type));
  QMMF_VERBOSE("%s: %s() INPARAM: data[%s]", TAG, __func__,
               data.ToString(type).c_str());

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

  return 0;
}

int32_t AudioBackendSink::GetRenderedPosition(uint32_t* frames,
                                              uint64_t* time)
{
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

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

#ifndef AUDIO_BACKEND_PRIMARY_DEBUG_DATAFLOW
  int result = qahw_out_get_render_position(qahw_stream_, frames);
  if (result < 0) {
    QMMF_ERROR("%s: %s() Failed to get render position : %d", TAG,
        __func__, result);
  }

  QMMF_VERBOSE("%s: %s() Total Frames Rendered : %u", TAG, __func__, *frames);

  uint64_t frame;
  struct timespec tv;

  result = qahw_out_get_presentation_position(qahw_stream_, &frame, &tv);
  if (result < 0) {
    QMMF_ERROR("%s: %s() Failed to get presentation position: %d", TAG,
        __func__, result);
  }

  *time = (uint64_t)(tv.tv_sec) * 1000000 + (uint64_t)(tv.tv_nsec) / 1000;

  QMMF_VERBOSE("%s: %s() Total Frames Rendered (%llu) Time (%llu)", TAG,
      __func__, frame, *time);
#else
  *frames = 16;
  *time = 10000000;
#endif

  QMMF_VERBOSE("%s: %s() OUTPARAM: frames[%u] time[%llu]", TAG, __func__,
      *frames, *time);
  return 0;
}

void AudioBackendSink::ThreadEntry(AudioBackendSink* backend) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  backend->Thread();
}

void AudioBackendSink::Thread() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  queue<AudioBuffer> buffers;
  bool paused = false;
  bool flushing = false;
  int result;

  bool keep_running = true;
  while (keep_running) {
    // wait until there is something to do
    if (buffers.empty() && messages_.empty()) {
      unique_lock<mutex> lk(message_lock_);
      if (signal_.wait_for(lk, seconds(1)) == cv_status::timeout)
        QMMF_WARN("%s: %s() timed out on wait", TAG, __func__);
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
          flushing = message.flush;
          keep_running = false;
          break;

        case AudioMessageType::kMessageBuffer:
          QMMF_DEBUG("%s: %s-MessageBuffer() TRACE", TAG, __func__);
          for (const AudioBuffer& buffer : message.buffers) {
            QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s] to queue[%u]",
                         TAG, __func__, buffer.ToString().c_str(),
                         buffers.size());
            buffers.push(buffer);
            QMMF_VERBOSE("%s: %s() buffers queue is now %u deep",
                         TAG, __func__, buffers.size());
          }
          break;
      }

      messages_.pop();
    }
    message_lock_.unlock();

    // process the next pending buffer
    do {
      if (!buffers.empty() && !paused) {
        AudioBuffer& buffer = buffers.front();
        QMMF_VERBOSE("%s: %s() processing next buffer[%s] from queue[%u]",
                     TAG, __func__, buffer.ToString().c_str(), buffers.size());

#ifndef AUDIO_BACKEND_PRIMARY_DEBUG_DATAFLOW
        qahw_out_buffer_t qahw_buffer;
        memset(&qahw_buffer, 0, sizeof(qahw_out_buffer_t));
        qahw_buffer.buffer = buffer.data;
        qahw_buffer.bytes = buffer.size;

        result = qahw_out_write(qahw_stream_, &qahw_buffer);
        if (result < 0) {
          QMMF_ERROR("%s: %s() failed to write output stream: %d[%s]", TAG,
                     __func__, result, strerror(result));
          error_handler_(audio_handle_, result);
        } else {
          buffer.size = 0;
          buffer.timestamp = 0;
        }
#else
        for (size_t index = 0; index < buffer.size; ++index) {
          unsigned char* charbuf = static_cast<unsigned char*>(buffer.data);
          QMMF_INFO("%s: %s() consumed buffer[%d][0x%x]", TAG, __func__, index,
                    charbuf[index]);
        }
        buffer.size = 0;
        buffer.timestamp = 0;
        ::std::this_thread::sleep_for(::std::chrono::seconds(1));
#endif

        if (buffer.size == 0) {
          // return empty buffer to client
          buffer_handler_(audio_handle_, buffer);
          buffers.pop();
          QMMF_VERBOSE("%s: %s() buffers queue is now %u deep",
                       TAG, __func__, buffers.size());
        }
      }

      if (buffers.empty() || paused)
        flushing = false;
    } while (flushing == true);
  }

#ifndef AUDIO_BACKEND_PRIMARY_DEBUG_DATAFLOW
  result = qahw_out_standby(qahw_stream_);
  if (result != 0) {
    QMMF_ERROR("%s: %s() failed to put output stream in standby: %d[%s]",
               TAG, __func__, result, strerror(result));
    error_handler_(audio_handle_, result);
  }
#endif
}

}; // namespace audio
}; // namespace common
}; // namespace qmmf
