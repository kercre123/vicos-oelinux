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

#define TAG "AudioBackendSource"

#include "common/audio/src/service/qmmf_audio_backend_source.h"

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <map>
#include <mutex>
#include <queue>
#include <thread>
#include <time.h>
#include <vector>

#include <cutils/properties.h>
#include <mm-audio/qahw_api/inc/qahw_api.h>
#include <mm-audio/qahw_api/inc/qahw_defs.h>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/src/service/qmmf_audio_common.h"
#include "common/qmmf_log.h"

// remove comment marker to mimic the AHAL instead of using it
//#define AUDIO_BACKEND_PRIMARY_DEBUG_DATAFLOW

#define AUDIO_TIMESTAMP_ADJUST_PROPERTY   "persist.qmmf.timestamp.adjust"

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

const audio_io_handle_t AudioBackendSource::kIOHandleMin = 900;
const audio_io_handle_t AudioBackendSource::kIOHandleMax = 999;

AudioBackendSource::AudioBackendSource(const AudioHandle audio_handle,
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

AudioBackendSource::~AudioBackendSource() {}

int32_t AudioBackendSource::Open(const vector<DeviceId>& devices,
                                 const AudioMetadata& metadata) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  for (const DeviceId device : devices)
    QMMF_VERBOSE("%s: %s() INPARAM: device[%d]", TAG, __func__, device);
  QMMF_VERBOSE("%s: %s() INPARAM: metadata[%s]", TAG, __func__,
               metadata.ToString().c_str());
  int result = 0;

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

  audio_devices_t audio_devices = 0;
  for (const DeviceId device : devices) {
    switch (device) {
      case static_cast<int32_t>(AudioDeviceId::kDefault):
        audio_devices |= AUDIO_DEVICE_IN_DEFAULT;
        break;
      case static_cast<int32_t>(AudioDeviceId::kCommunication):
        audio_devices |= AUDIO_DEVICE_IN_COMMUNICATION;
        break;
      case static_cast<int32_t>(AudioDeviceId::kAmbient):
        audio_devices |= AUDIO_DEVICE_IN_AMBIENT;
        break;
      case static_cast<int32_t>(AudioDeviceId::kBuiltIn):
        audio_devices |= AUDIO_DEVICE_IN_BUILTIN_MIC;
        break;
      case static_cast<int32_t>(AudioDeviceId::kHeadSet):
        audio_devices |= AUDIO_DEVICE_IN_WIRED_HEADSET;
        break;
      case static_cast<int32_t>(AudioDeviceId::kBlueToothSCO):
        audio_devices |= AUDIO_DEVICE_IN_BLUETOOTH_SCO_HEADSET;
        break;
      case static_cast<int32_t>(AudioDeviceId::kBlueToothA2DP):
        audio_devices |= AUDIO_DEVICE_IN_BLUETOOTH_A2DP;
        break;
    }
  }
  if (audio_devices == 0) {
    QMMF_ERROR("%s: %s() no valid device IDs specified", TAG, __func__);
    return -EINVAL;
  }

  if ((audio_devices & AUDIO_DEVICE_IN_BLUETOOTH_A2DP) ==
      AUDIO_DEVICE_IN_BLUETOOTH_A2DP) {
    audio_devices = AUDIO_DEVICE_IN_BLUETOOTH_A2DP;
    QMMF_INFO("%s: %s() constraining input devices to BT-A2DP only",
              TAG, __func__);

    qahw_module_ = qahw_load_module(QAHW_MODULE_ID_A2DP);
    if (qahw_module_ == nullptr) {
      QMMF_ERROR("%s: %s() failed to load QAHW module[%s]", TAG, __func__,
                 QAHW_MODULE_ID_A2DP);
      return -ENOMEM;
    }
  } else {
    qahw_module_ = qahw_load_module(QAHW_MODULE_ID_PRIMARY);
    if (qahw_module_ == nullptr) {
      QMMF_ERROR("%s: %s() failed to load QAHW module[%s]", TAG, __func__,
                 QAHW_MODULE_ID_PRIMARY);
      return -ENOMEM;
    }
  }

  result = qahw_init_check(qahw_module_);
  if (result != 0) {
    QMMF_ERROR("%s: %s() QAHW module initialization failed: %d[%s]",
               TAG, __func__, result, strerror(result));
    return result;
  }

  audio_config_t config = AUDIO_CONFIG_INITIALIZER;
  switch (metadata.num_channels) {
    case 1:
      config.channel_mask = AUDIO_CHANNEL_IN_MONO;
      break;
    case 2:
      config.channel_mask = AUDIO_CHANNEL_IN_STEREO;
      break;
    case 4:
      config.channel_mask = AUDIO_CHANNEL_INDEX_MASK_4;
      break;
    default:
      QMMF_ERROR("%s: %s() invalid number of channels: %d", TAG, __func__,
                 metadata.num_channels);
      return -EINVAL;
  }

  switch (metadata.sample_size) {
    case 16:
      config.format = AUDIO_FORMAT_PCM_16_BIT;
      break;
    case 24:
      config.format = AUDIO_FORMAT_PCM_24_BIT_PACKED;
      break;
    default:
      QMMF_ERROR("%s: %s() invalid sample size: %d", TAG, __func__,
                 metadata.sample_size);
      return -EINVAL;
  }

  config.sample_rate = metadata.sample_rate;
  config.frame_count = 0;

  // use the next available io_handle
  if (current_io_handle_ + 1 > kIOHandleMax)
    current_io_handle_ = kIOHandleMin;
  ++current_io_handle_;

  result = qahw_open_input_stream(qahw_module_, current_io_handle_,
                                  audio_devices, &config, &qahw_stream_,
                                  static_cast<audio_input_flags_t>
                                  (QAHW_INPUT_FLAG_COMPRESS),
                                  "input_stream",
                                  AUDIO_SOURCE_DEFAULT);
  if (result != 0) {
    QMMF_ERROR("%s: %s() failed to open input stream: %d[%s]", TAG, __func__,
               result, strerror(result));
    return result;
  }

  // set the input source to AUDIO_SOURCE_MIC
  result = qahw_in_set_parameters(qahw_stream_, "input_source=1");
  if (result != 0) {
    QMMF_ERROR("%s: %s() failed to set input source: %d[%s]", TAG, __func__,
               result, strerror(result));
    return result;
  }

  if ((audio_devices & AUDIO_DEVICE_IN_BLUETOOTH_SCO_HEADSET) ==
      AUDIO_DEVICE_IN_BLUETOOTH_SCO_HEADSET) {
    result = qahw_in_set_parameters(qahw_stream_, "bt_wbs=no");
    if (result != 0) {
      QMMF_ERROR("%s: %s() failed to set BT-wbs: %d[%s]", TAG, __func__,
                 result, strerror(result));
      return result;
    }
  }

#endif

  state_ = AudioState::kIdle;
  QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
             static_cast<int>(state_));

  return 0;
}

int32_t AudioBackendSource::Close() {
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
  result = qahw_in_standby(qahw_stream_);
  if (result != 0) {
    QMMF_ERROR("%s: %s() failed to put input stream in standby: %d[%s]",
               TAG, __func__, result, strerror(result));
    return result;
  }

  result = qahw_close_input_stream(qahw_stream_);
  if (result != 0) {
    QMMF_ERROR("%s: %s() failed to close input stream: %d[%s]",
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

int32_t AudioBackendSource::Start() {
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

  thread_ = new thread(AudioBackendSource::ThreadEntry, this);
  if (thread_ == nullptr) {
    QMMF_ERROR("%s: %s() unable to allocate thread", TAG, __func__);
    return -ENOMEM;
  }

  state_ = AudioState::kRunning;
  QMMF_DEBUG("%s: %s() state is now %d", TAG, __func__,
             static_cast<int>(state_));

  return 0;
}

int32_t AudioBackendSource::Stop(const bool flush) {
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

int32_t AudioBackendSource::Pause() {
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

int32_t AudioBackendSource::Resume() {
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

int32_t AudioBackendSource::SendBuffers(const vector<AudioBuffer>& buffers) {
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

int32_t AudioBackendSource::GetLatency(int32_t* latency) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  QMMF_WARN("%s: %s() invalid operation", TAG, __func__);
  return 0;
}

int32_t AudioBackendSource::GetBufferSize(int32_t* buffer_size) {
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
  *buffer_size = qahw_in_get_buffer_size(qahw_stream_);
#else
  *buffer_size = 16;
#endif

  QMMF_VERBOSE("%s: %s() OUTPARAM: buffer_size[%d]", TAG, __func__,
               *buffer_size);
  return 0;
}

int32_t AudioBackendSource::SetParam(const AudioParamType type,
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

int32_t AudioBackendSource::GetRenderedPosition(uint32_t* frames,
                                                uint64_t* time)
{
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  QMMF_WARN("%s: %s() invalid operation", TAG, __func__);
  return 0;
}

void AudioBackendSource::ThreadEntry(AudioBackendSource* backend) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  backend->Thread();
}

void AudioBackendSource::Thread() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  queue<AudioBuffer> buffers;
  bool paused = false;

  bool keep_running = true;
  bool stop_received = false;
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
          stop_received = true;
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
    if (!buffers.empty() && !paused && keep_running) {
      AudioBuffer& buffer = buffers.front();
      QMMF_VERBOSE("%s: %s() processing next buffer[%s] from queue[%u]",
                   TAG, __func__, buffer.ToString().c_str(), buffers.size());

#ifndef AUDIO_BACKEND_PRIMARY_DEBUG_DATAFLOW
      qahw_in_buffer_t qahw_buffer;
      memset(&qahw_buffer, 0, sizeof(qahw_in_buffer_t));
      qahw_buffer.buffer = buffer.data;
      qahw_buffer.bytes = buffer.capacity;

      int result = qahw_in_read(qahw_stream_, &qahw_buffer);
      if (result < 0) {
        QMMF_ERROR("%s: %s() failed to read input stream: %d[%s]", TAG,
                   __func__, result, strerror(result));
        error_handler_(audio_handle_, result);
        buffer.size = 0;
      } else {
        buffer.size = result;
      }
#else
      memset(buffer.data, 0xFF, buffer.capacity);
      memset(buffer.data, 0x11, 1);
      buffer.size = buffer.capacity;
      ::std::this_thread::sleep_for(::std::chrono::seconds(1));
#endif

      // if filled, return timestamped buffer to client
      if (buffer.size > 0) {
        struct timespec tv;
        clock_gettime(CLOCK_MONOTONIC, &tv);
        buffer.timestamp = (int64_t)(tv.tv_sec) * 1000000 +
                           (int64_t)(tv.tv_nsec) / 1000;

        char adjust_string[PROPERTY_VALUE_MAX];
        property_get(AUDIO_TIMESTAMP_ADJUST_PROPERTY, adjust_string, "0");
        buffer.timestamp += atoi(adjust_string);
        QMMF_VERBOSE("%s: %s() generated timestamp[%lld] with adjust[%d]",
                     TAG, __func__, buffer.timestamp, atoi(adjust_string));

        if (stop_received) {
          QMMF_DEBUG("%s: %s() setting EOS flag", TAG, __func__);
          buffer.flags |= static_cast<uint32_t>(BufferFlags::kFlagEOS);
          keep_running = false;
        } else {
          buffer.flags = 0;
        }

        buffer_handler_(audio_handle_, buffer);
        buffers.pop();
        QMMF_VERBOSE("%s: %s() buffers queue is now %u deep",
                     TAG, __func__, buffers.size());
      }
    }
  }
}

}; // namespace audio
}; // namespace common
}; // namespace qmmf
