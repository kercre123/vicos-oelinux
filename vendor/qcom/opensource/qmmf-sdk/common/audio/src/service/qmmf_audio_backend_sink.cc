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

#define LOG_TAG "AudioBackendSink"

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
#include "common/utils/qmmf_log.h"

#ifdef ANDROID_O_OR_ABOVE
// TODO: Resolve header dependencies once available
#define AUDIO_FORMAT_AAC_LATM       (0x23000000UL)
#define AUDIO_FORMAT_AAC_LATM_LC    (AUDIO_FORMAT_AAC_LATM |\
                                      AUDIO_FORMAT_AAC_SUB_LC)
#define AUDIO_FORMAT_AAC_LATM_HE_V1 (AUDIO_FORMAT_AAC_LATM |\
                                      AUDIO_FORMAT_AAC_SUB_HE_V1)
#define AUDIO_FORMAT_AAC_LATM_HE_V2 (AUDIO_FORMAT_AAC_LATM |\
                                      AUDIO_FORMAT_AAC_SUB_HE_V2)
#endif

namespace qmmf {
namespace common {
namespace audio {

using ::std::chrono::milliseconds;
using ::std::chrono::seconds;
using ::std::condition_variable;
using ::std::cv_status;
using ::std::function;
using ::std::map;
using ::std::mutex;
using ::std::queue;
using ::std::string;
using ::std::thread;
using ::std::unique_lock;
using ::std::vector;

const audio_io_handle_t AudioBackendSink::kIOHandleMin = 800;
const audio_io_handle_t AudioBackendSink::kIOHandleMax = 899;

AudioBackendSink::AudioBackendSink(const AudioHandle audio_handle,
                                   const AudioErrorHandler& error_handler,
                                   const AudioBufferHandler& buffer_handler,
                                   const AudioStoppedHandler& stopped_handler)
    : audio_handle_(audio_handle),
      state_(AudioState::kNew),
      error_handler_(error_handler),
      buffer_handler_(buffer_handler),
      stopped_handler_(stopped_handler),
      thread_(nullptr),
      using_offload_(false),
      current_io_handle_(kIOHandleMin) {
  QMMF_DEBUG("%s() state is now %d", __func__,
             static_cast<int>(state_));
}

AudioBackendSink::~AudioBackendSink() {}

int32_t AudioBackendSink::Open(const qahw_module_handle_t * const modules[],
                               const vector<DeviceId>& devices,
                               const AudioMetadata& metadata) {
  QMMF_DEBUG("%s() TRACE", __func__);
  for (const DeviceId device : devices)
    QMMF_VERBOSE("%s() INPARAM: device[%d]", __func__, device);
  QMMF_VERBOSE("%s() INPARAM: metadata[%s]", __func__,
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
      QMMF_ERROR("%s() invalid operation for current state: %d",
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s() unknown state: %d", __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  qahw_module_ = const_cast<qahw_module_handle_t *>
                           (modules[AudioHAL::kPrimary]);
  if (qahw_module_ == nullptr) {
    QMMF_ERROR("%s() QAHW module[%s] is not currently loaded",
               __func__, QAHW_MODULE_ID_PRIMARY);
    return -ENOMEM;
  }

  if ((metadata.flags & static_cast<uint32_t>(AudioFlag::kFlagLowLatency)) == 0)
    using_offload_ = true;
  else
    using_offload_ = false;

  audio_config_t config = AUDIO_CONFIG_INITIALIZER;
  switch (metadata.format) {
    case AudioFormat::kPCM:
      switch (metadata.sample_size) {
        case 8:
          config.format = AUDIO_FORMAT_PCM_8_BIT;
          break;
        case 16:
          config.format = AUDIO_FORMAT_PCM_16_BIT;
          break;
        case 24:
          config.format = AUDIO_FORMAT_PCM_24_BIT_PACKED;
          break;
        case 243:
          config.format = AUDIO_FORMAT_PCM_24_BIT_PACKED;
          break;
        case 244:
          config.format = AUDIO_FORMAT_PCM_8_24_BIT;
          break;
        case 32:
          config.format = AUDIO_FORMAT_PCM_32_BIT;
          break;
        default:
          QMMF_ERROR("%s() invalid sample size: %d", __func__,
                     metadata.sample_size);
          return -EINVAL;
      }
      break;

    case AudioFormat::kMP3:
      config.format = AUDIO_FORMAT_MP3;
      break;

    case AudioFormat::kAAC:
      switch (metadata.codec_params.aac.format) {
        case AACFormat::kADTS:
          switch (metadata.codec_params.aac.mode) {
            case AACMode::kAALC:
              config.format = AUDIO_FORMAT_AAC_ADTS_LC;
              break;
            case AACMode::kHEVC_v1:
              config.format = AUDIO_FORMAT_AAC_ADTS_HE_V1;
              break;
            case AACMode::kHEVC_v2:
              config.format = AUDIO_FORMAT_AAC_ADTS_HE_V2;
              break;
            default:
              break;
          }
          break;

        case AACFormat::kRaw:
          switch (metadata.codec_params.aac.mode) {
            case AACMode::kAALC:
              config.format = AUDIO_FORMAT_AAC_LC;
              break;
            case AACMode::kHEVC_v1:
              config.format = AUDIO_FORMAT_AAC_HE_V1;
              break;
            case AACMode::kHEVC_v2:
              config.format = AUDIO_FORMAT_AAC_HE_V2;
              break;
            default:
              break;
          }
          break;

        case AACFormat::kMP4FF:
          switch (metadata.codec_params.aac.mode) {
            case AACMode::kAALC:
#ifdef ANDROID_O_OR_ABOVE
              config.format = static_cast<audio_format_t>(AUDIO_FORMAT_AAC_LATM_LC);
#else
              config.format = AUDIO_FORMAT_AAC_LATM_LC;
#endif
              break;
            case AACMode::kHEVC_v1:
#ifdef ANDROID_O_OR_ABOVE
              config.format = static_cast<audio_format_t>(AUDIO_FORMAT_AAC_LATM_HE_V1);
#else
              config.format = AUDIO_FORMAT_AAC_LATM_HE_V1;
#endif
              break;
            case AACMode::kHEVC_v2:
#ifdef ANDROID_O_OR_ABOVE
              config.format = static_cast<audio_format_t>(AUDIO_FORMAT_AAC_LATM_HE_V2);
#else
              config.format = AUDIO_FORMAT_AAC_LATM_HE_V2;
#endif
              break;
            default:
              break;
          }
          break;

        default:
          QMMF_ERROR("%s() invalid format: %d", __func__,
                     static_cast<int32_t>(metadata.format));
          return -EINVAL;
      }
      break;

    default:
      QMMF_ERROR("%s() invalid format: %d", __func__,
                 static_cast<int32_t>(metadata.format));
      return -EINVAL;
  }

  config.sample_rate = metadata.sample_rate;
  config.channel_mask = audio_channel_out_mask_from_count(metadata.num_channels);
  config.frame_count = 0;

  if (using_offload_) {
    config.offload_info = AUDIO_INFO_INITIALIZER;
    config.offload_info.sample_rate = config.sample_rate;
    config.offload_info.channel_mask = config.channel_mask;
    config.offload_info.format = config.format;
    config.offload_info.stream_type = AUDIO_STREAM_MUSIC;
    switch (metadata.sample_size) {
      case 8:   config.offload_info.bit_width = 8; break;
      case 16:  config.offload_info.bit_width = 16; break;
      case 24:  config.offload_info.bit_width = 24; break;
      case 243: config.offload_info.bit_width = 24; break;
      case 244: config.offload_info.bit_width = 24; break;
      case 32:  config.offload_info.bit_width = 32; break;
      default:
        QMMF_ERROR("%s() invalid sample size: %d", __func__,
                   metadata.sample_size);
        return -EINVAL;
    }
    config.offload_info.usage = AUDIO_USAGE_MEDIA;
    config.offload_info.is_streaming = true;
  }

  // use the next available io_handle
  if (current_io_handle_ + 1 > kIOHandleMax)
    current_io_handle_ = kIOHandleMin;
  ++current_io_handle_;

  audio_output_flags_t flags;
  if (using_offload_)
    flags = static_cast<audio_output_flags_t>(AUDIO_OUTPUT_FLAG_DIRECT |
                                              AUDIO_OUTPUT_FLAG_COMPRESS_OFFLOAD |
                                              AUDIO_OUTPUT_FLAG_NON_BLOCKING);
  else
    flags = static_cast<audio_output_flags_t>(AUDIO_OUTPUT_FLAG_DIRECT |
                                              AUDIO_OUTPUT_FLAG_FAST);

  result = qahw_open_output_stream(qahw_module_, current_io_handle_,
                                   AUDIO_DEVICE_OUT_SPEAKER, flags, &config,
                                   &qahw_stream_, "output_stream");
  if (result != 0) {
    QMMF_ERROR("%s() failed to open output stream: %d[%s]", __func__,
               result, strerror(result));
    return result;
  }

  if (using_offload_) {
    result = qahw_out_set_callback(qahw_stream_,
                                   AudioBackendSink::CallbackEntry, this);
    if (result != 0) {
      QMMF_ERROR("%s() failed to set callback: %d[%s]", __func__,
                 result, strerror(result));
      return result;
    }
  }

  state_ = AudioState::kIdle;
  QMMF_DEBUG("%s() state is now %d", __func__,
             static_cast<int>(state_));

  return result;
}

int32_t AudioBackendSink::Close() {
  QMMF_DEBUG("%s() TRACE", __func__);
  int32_t result = 0;

  switch (state_) {
    case AudioState::kNew:
      QMMF_WARN("%s() nothing to do, state is: %d", __func__,
                static_cast<int>(state_));
      return 0;
      break;
    case AudioState::kIdle:
      // proceed
      break;
    case AudioState::kConnect:
    case AudioState::kRunning:
    case AudioState::kPaused:
      QMMF_ERROR("%s() invalid operation for current state: %d",
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s() unknown state: %d", __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
    thread_ = nullptr;
  }

  result = qahw_close_output_stream(qahw_stream_);
  if (result != 0) {
    QMMF_ERROR("%s() failed to close output stream: %d[%s]",
               __func__, result, strerror(result));
    return result;
  }

  state_ = AudioState::kNew;
  QMMF_DEBUG("%s() state is now %d", __func__,
             static_cast<int>(state_));

  return result;
}

int32_t AudioBackendSink::Start() {
  QMMF_DEBUG("%s() TRACE", __func__);

  switch (state_) {
    case AudioState::kIdle:
      // proceed
      break;
    case AudioState::kNew:
    case AudioState::kConnect:
    case AudioState::kRunning:
    case AudioState::kPaused:
      QMMF_ERROR("%s() invalid operation for current state: %d",
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s() unknown state: %d", __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
    thread_ = nullptr;
  }

  while (!messages_.empty())
    messages_.pop();

  thread_ = new thread(AudioBackendSink::ThreadEntry, this);
  if (thread_ == nullptr) {
    QMMF_ERROR("%s() unable to allocate thread", __func__);
    return -ENOMEM;
  }

  state_ = AudioState::kRunning;
  QMMF_DEBUG("%s() state is now %d", __func__,
             static_cast<int>(state_));

  return 0;
}

int32_t AudioBackendSink::Stop() {
  QMMF_DEBUG("%s() TRACE", __func__);

  switch (state_) {
    case AudioState::kNew:
    case AudioState::kConnect:
    case AudioState::kIdle:
      QMMF_WARN("%s() nothing to do, state is: %d", __func__,
                static_cast<int>(state_));
      return 0;
      break;
    case AudioState::kRunning:
    case AudioState::kPaused:
      // proceed
      break;
    default:
      QMMF_ERROR("%s() unknown state: %d", __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  AudioMessage message;
  message.type = AudioMessageType::kMessageStop;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();

  thread_->join();
  delete thread_;
  thread_ = nullptr;

  while (!messages_.empty())
    messages_.pop();

  state_ = AudioState::kIdle;
  QMMF_DEBUG("%s() state is now %d", __func__,
             static_cast<int>(state_));

  return 0;
}

int32_t AudioBackendSink::Pause() {
  QMMF_DEBUG("%s() TRACE", __func__);

  switch (state_) {
    case AudioState::kRunning:
      // proceed
      break;
    case AudioState::kNew:
    case AudioState::kConnect:
    case AudioState::kIdle:
    case AudioState::kPaused:
      QMMF_ERROR("%s() invalid operation for current state: %d",
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s() unknown state: %d", __func__,
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
  QMMF_DEBUG("%s() state is now %d", __func__,
             static_cast<int>(state_));

  return 0;
}

int32_t AudioBackendSink::Resume() {
  QMMF_DEBUG("%s() TRACE", __func__);

  switch (state_) {
    case AudioState::kPaused:
      // proceed
      break;
    case AudioState::kNew:
    case AudioState::kConnect:
    case AudioState::kIdle:
    case AudioState::kRunning:
      QMMF_ERROR("%s() invalid operation for current state: %d",
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s() unknown state: %d", __func__,
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
  QMMF_DEBUG("%s() state is now %d", __func__,
             static_cast<int>(state_));

  return 0;
}

int32_t AudioBackendSink::SendBuffers(const vector<AudioBuffer>& buffers) {
  QMMF_DEBUG("%s() TRACE", __func__);
  for (const AudioBuffer& buffer : buffers)
    QMMF_VERBOSE("%s() INPARAM: buffer[%s]", __func__,
                 buffer.ToString().c_str());

  switch (state_) {
    case AudioState::kIdle:
    case AudioState::kRunning:
      // proceed
      break;
    case AudioState::kNew:
    case AudioState::kConnect:
    case AudioState::kPaused:
      QMMF_ERROR("%s() invalid operation for current state: %d",
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s() unknown state: %d", __func__,
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
  QMMF_DEBUG("%s() TRACE", __func__);

  switch (state_) {
    case AudioState::kIdle:
      // proceed
      break;
    case AudioState::kNew:
    case AudioState::kConnect:
    case AudioState::kRunning:
    case AudioState::kPaused:
      QMMF_ERROR("%s() invalid operation for current state: %d",
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s() unknown state: %d", __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  *latency = qahw_out_get_latency(qahw_stream_);

  QMMF_VERBOSE("%s() OUTPARAM: latency[%d]", __func__, *latency);
  return 0;
}

int32_t AudioBackendSink::GetBufferSize(int32_t* buffer_size) {
  QMMF_DEBUG("%s() TRACE", __func__);

  switch (state_) {
    case AudioState::kIdle:
      // proceed
      break;
    case AudioState::kNew:
    case AudioState::kConnect:
    case AudioState::kRunning:
    case AudioState::kPaused:
      QMMF_ERROR("%s() invalid operation for current state: %d",
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s() unknown state: %d", __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  *buffer_size = qahw_out_get_buffer_size(qahw_stream_);

  QMMF_VERBOSE("%s() OUTPARAM: buffer_size[%d]", __func__,
               *buffer_size);
  return 0;
}

int32_t AudioBackendSink::SetParam(const AudioParamType type,
                                   const AudioParamData& data) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: type[%d]", __func__,
               static_cast<int>(type));
  QMMF_VERBOSE("%s() INPARAM: data[%s]", __func__,
               data.ToString(type).c_str());

  switch (state_) {
    case AudioState::kIdle:
    case AudioState::kRunning:
    case AudioState::kPaused:
      // proceed
      break;
    case AudioState::kNew:
    case AudioState::kConnect:
      QMMF_ERROR("%s() invalid operation for current state: %d",
                 __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    default:
      QMMF_ERROR("%s() unknown state: %d", __func__,
                 static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  switch (type) {
    case AudioParamType::kVolume:
      {
        float volume = static_cast<float>(data.volume) / 100.0;

        int result = qahw_out_set_volume(qahw_stream_, volume, volume);
        if (result != 0) {
          QMMF_ERROR("%s() failed to set volume[%f]: %d[%s]",
                     __func__, volume, result, strerror(result));
          return result;
        }
      }
      break;
    case AudioParamType::kDevice:
      QMMF_WARN("%s() invalid operation", __func__);
      break;
    case AudioParamType::kCustom:
      {
        string keyvalue = data.custom.key;
        keyvalue.append("=");
        keyvalue.append(data.custom.value);

        int result = qahw_out_set_parameters(qahw_stream_, keyvalue.c_str());
        if (result != 0) {
          QMMF_ERROR("%s() failed to set custom parameter[%s]: %d[%s]",
                     __func__, keyvalue.c_str(), result,
                     strerror(result));
          return result;
        }
      }
      break;
    default:
      QMMF_ERROR("%s() unknown parameter: %d", __func__,
                 static_cast<int>(type));
      return -ENOSYS;
      break;
  }

  return 0;
}

int32_t AudioBackendSink::GetRenderedPosition(uint32_t* frames,
                                              uint64_t* time)
{
  QMMF_DEBUG("%s() TRACE", __func__);

  switch (state_) {
    case AudioState::kIdle:
    case AudioState::kNew:
    case AudioState::kConnect:
      QMMF_ERROR("%s() invalid operation for current state: %d",
          __func__, static_cast<int>(state_));
      return -ENOSYS;
      break;
    case AudioState::kRunning:
    case AudioState::kPaused:
      // proceed
      break;
    default:
      QMMF_ERROR("%s() unknown state: %d", __func__,
          static_cast<int>(state_));
      return -ENOSYS;
      break;
  }

  int result = qahw_out_get_render_position(qahw_stream_, frames);
  if (result < 0) {
    QMMF_ERROR("%s() Failed to get render position : %d",
        __func__, result);
  }

  QMMF_VERBOSE("%s() Total Frames Rendered : %u", __func__, *frames);

  uint64_t frame;
  struct timespec tv;

  result = qahw_out_get_presentation_position(qahw_stream_, &frame, &tv);
  if (result < 0) {
    QMMF_ERROR("%s() Failed to get presentation position: %d",
               __func__, result);
  }

  *time = (uint64_t)(tv.tv_sec) * 1000000 + (uint64_t)(tv.tv_nsec) / 1000;

  QMMF_VERBOSE("%s() Total Frames Rendered (%llu) Time (%llu)",
      __func__, frame, *time);

  QMMF_VERBOSE("%s() OUTPARAM: frames[%u] time[%llu]", __func__,
      *frames, *time);
  return 0;
}

int AudioBackendSink::CallbackEntry(qahw_stream_callback_event_t event,
                                    void* param,
                                    void* cookie) {
  QMMF_DEBUG("%s() TRACE", __func__);

  if (cookie == nullptr) {
    QMMF_ERROR("%s() invalid cookie given", __func__);
    return 0;
  }

  AudioBackendSink* backend = reinterpret_cast<AudioBackendSink*>(cookie);
  return backend->Callback(event, param);
}

int AudioBackendSink::Callback(qahw_stream_callback_event_t event,
                               void* param) {
  QMMF_DEBUG("%s() TRACE", __func__);

  switch (event) {
    case QAHW_STREAM_CBK_EVENT_WRITE_READY:
      QMMF_DEBUG("%s() received WRITE_READY event", __func__);
      {
        AudioMessage message;
        message.type = AudioMessageType::kMessageWriteDone;

        message_lock_.lock();
        messages_.push(message);
        message_lock_.unlock();
        signal_.notify_one();
      }
      break;

    case QAHW_STREAM_CBK_EVENT_DRAIN_READY:
      QMMF_DEBUG("%s() received DRAIN_READY event", __func__);
      {
        AudioMessage message;
        message.type = AudioMessageType::kMessageFlushDone;

        message_lock_.lock();
        messages_.push(message);
        message_lock_.unlock();
        signal_.notify_one();
      }
      break;

    case QAHW_STREAM_CBK_EVENT_ADSP:
      QMMF_DEBUG("%s() received ADSP event", __func__);
      break;

    case QAHW_STREAM_CBK_EVENT_ERROR:
      QMMF_ERROR("%s() received ERROR event", __func__);
      Stop();
      break;

    default:
      QMMF_ERROR("%s() invalid event[%d]", __func__, event);
      break;
  }

  return 0;
}

void AudioBackendSink::ThreadEntry(AudioBackendSink* backend) {
  QMMF_DEBUG("%s() TRACE", __func__);

  backend->Thread();
}

void AudioBackendSink::Thread() {
  QMMF_DEBUG("%s() TRACE", __func__);
  queue<AudioBuffer> buffers;
  int result;

  size_t bytes_written = 0;
  bool pending_write = false;
  bool stop_received = false;
  bool eof_received = false;
  bool paused = false;
  bool flushing = false;
  bool pending_flush = false;
  bool keep_running = true;
  while (keep_running) {
    // wait until there is something to do
    while (buffers.empty() && messages_.empty()) {
      unique_lock<mutex> lk(message_lock_);
      if (signal_.wait_for(lk, seconds(1)) == cv_status::timeout)
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

          result = qahw_out_pause(qahw_stream_);
          if (result < 0) {
            QMMF_ERROR("%s() failed to pause output stream: %d[%s]",
                       __func__, result, strerror(result));
            error_handler_(audio_handle_, result);
          }
          break;

        case AudioMessageType::kMessageResume:
          QMMF_DEBUG("%s-MessageResume() TRACE", __func__);
          paused = false;

          result = qahw_out_resume(qahw_stream_);
          if (result < 0) {
            QMMF_ERROR("%s() failed to resume output stream: %d[%s]",
                       __func__, result, strerror(result));
            error_handler_(audio_handle_, result);
          }
          break;

        case AudioMessageType::kMessageStop:
          QMMF_DEBUG("%s-MessageStop() TRACE", __func__);
          paused = false;
          eof_received = false;
          stop_received = true;
          break;

        case AudioMessageType::kMessageBuffer:
          QMMF_DEBUG("%s-MessageBuffer() TRACE", __func__);
          for (const AudioBuffer& buffer : message.buffers) {
            QMMF_VERBOSE("%s() INPARAM: buffer[%s] to queue[%u]",
                         __func__, buffer.ToString().c_str(),
                         buffers.size());
            buffers.push(buffer);
            QMMF_VERBOSE("%s() buffers queue is now %u deep",
                         __func__, buffers.size());
          }
          break;

        case AudioMessageType::kMessageWriteDone:
          QMMF_DEBUG("%s-MessageWriteDone() TRACE", __func__);
          pending_write = false;
          break;

        case AudioMessageType::kMessageFlushDone:
          QMMF_DEBUG("%s-MessageFlushDone() TRACE", __func__);
          keep_running = false;
          break;
      }

      messages_.pop();
    }
    message_lock_.unlock();

    // process the next pending buffer
    if (!buffers.empty() && !paused && !flushing && !pending_write &&
        !stop_received && keep_running) {
      AudioBuffer& buffer = buffers.front();
      QMMF_VERBOSE("%s() processing next buffer[%s] from queue[%u]",
                   __func__, buffer.ToString().c_str(), buffers.size());

      qahw_out_buffer_t qahw_buffer;
      memset(&qahw_buffer, 0, sizeof(qahw_out_buffer_t));
      qahw_buffer.buffer = reinterpret_cast<uint8_t*>(buffer.data) +
                           bytes_written;
      qahw_buffer.bytes = buffer.size - bytes_written;

      result = qahw_out_write(qahw_stream_, &qahw_buffer);
      if (result < 0) {
        QMMF_ERROR("%s() failed to write output stream: %d[%s]",
                   __func__, result, strerror(result));
        error_handler_(audio_handle_, result);
      } else if (static_cast<size_t>(result) != qahw_buffer.bytes &&
                 using_offload_) {
        pending_write = true;
        bytes_written += result;
      } else if (static_cast<size_t>(result) == qahw_buffer.bytes) {
        bytes_written += result;
      } else {
        QMMF_ERROR("%s() incomplete write to output stream for non-offload stream",
                   __func__);
        error_handler_(audio_handle_, -1);
      }

      if (bytes_written == static_cast<size_t>(buffer.size)) {
        bytes_written = 0;

        if (buffer.flags & static_cast<uint32_t>(BufferFlags::kFlagEOS))
          eof_received = true;

        buffer.size = 0;
        buffer.timestamp = 0;

        // return empty buffer to client
        buffer_handler_(audio_handle_, buffer);
        buffers.pop();
        QMMF_VERBOSE("%s() buffers queue is now %u deep",
                     __func__, buffers.size());
      }
    }

    // stop conditions
    if (stop_received) keep_running = false;
    else if (eof_received) flushing = true;

    if (flushing && !pending_flush) {
      if (using_offload_) {
        result = qahw_out_drain(qahw_stream_, QAHW_DRAIN_ALL);
        if (result != 0) {
          QMMF_ERROR("%s() failed to drain the output stream: %d[%s]",
                     __func__, result, strerror(result));
          error_handler_(audio_handle_, result);
        }
        pending_flush = true;
      } else {
        keep_running = false;
      }
    }
  }

  result = qahw_out_standby(qahw_stream_);
  if (result != 0) {
    QMMF_ERROR("%s() failed to put output stream in standby: %d[%s]",
               __func__, result, strerror(result));
    error_handler_(audio_handle_, result);
  }

  if (eof_received) {
    while (!messages_.empty())
      messages_.pop();

    state_ = AudioState::kIdle;
    QMMF_DEBUG("%s() state is now %d", __func__,
               static_cast<int>(state_));

    // notify client that endpoint has stopped
    stopped_handler_(audio_handle_);
  }
}

}; // namespace audio
}; // namespace common
}; // namespace qmmf
