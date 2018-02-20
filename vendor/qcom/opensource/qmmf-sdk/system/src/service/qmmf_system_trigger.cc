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

#define LOG_TAG "SystemTrigger"

#include "system/src/service/qmmf_system_trigger.h"

#include <functional>
#include <thread>

#include <mm-audio/qsthw_api/qsthw_defs.h>
#include <mm-audio/qsthw_api/qsthw_api.h>
#include <utils/Errors.h>

#include "common/utils/qmmf_log.h"
#include "qmmf-sdk/qmmf_system_params.h"
#include "system/src/service/qmmf_system_common.h"

namespace qmmf {
namespace system {

using ::std::thread;

const sound_trigger_uuid_t SystemTrigger::kQcUuid =
  { 0x68ab2d40, 0xe860, 0x11e3, 0x95ef, { 0x00, 0x02, 0xa5, 0xd5, 0xc5, 0x1b }};

SystemTrigger::SystemTrigger()
    : thread_(nullptr),
      current_handle_(0),
      module_(nullptr),
      sound_model_(nullptr),
      sm_handle_(0),
      rc_config_(nullptr),
      capture_duration_(0) {}

SystemTrigger::~SystemTrigger() {}

status_t SystemTrigger::LoadSoundModel(const SystemHandle system_handle,
                                       const SoundModel& soundmodel) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);
  QMMF_VERBOSE("%s() INPARAM: soundmodel[%s]", __func__,
               soundmodel.ToString().c_str());
  int32_t result = 0;
  size_t size;

  if (current_handle_ != 0) {
    QMMF_ERROR("%s() already in use", __func__);
    return ::android::ALREADY_EXISTS;
  }

  current_handle_ = system_handle;

  module_ = qsthw_load_module(QSTHW_MODULE_ID_PRIMARY);
  if (module_ == NULL) {
    QMMF_ERROR("%s() failed to load qsthw module[%s]",
               __func__, QSTHW_MODULE_ID_PRIMARY);
    goto error_handle;
  }

  size = sizeof(sound_trigger_phrase_sound_model) + soundmodel.size;
  sound_model_ = (sound_trigger_phrase_sound_model*)calloc(1, size);
  if (sound_model_ == nullptr) {
    QMMF_ERROR("%s() failed to allocate memory for sound model",
               __func__);
    goto error_close;
  }

  sound_model_->common.type = SOUND_MODEL_TYPE_KEYPHRASE;
  sound_model_->common.data_size = soundmodel.size;
  sound_model_->common.data_offset = sizeof(*sound_model_);
  sound_model_->num_phrases = soundmodel.keywords;
  for (uint32_t idx = 0; idx < sound_model_->num_phrases; ++idx)
    sound_model_->phrases[idx].recognition_mode = RECOGNITION_MODE_VOICE_TRIGGER;

  memcpy(&sound_model_->common.vendor_uuid, &kQcUuid,
         sizeof(sound_trigger_uuid_t));

  memcpy((char*)sound_model_ + sound_model_->common.data_offset,
         soundmodel.data, soundmodel.size);

  result = qsthw_load_sound_model(module_, &sound_model_->common, NULL, NULL,
                                  &sm_handle_);
  if (result != 0) {
    QMMF_ERROR("%s() failed to load sound model: result[%d]", __func__,
               result);
    goto error_free;
  }

  return ::android::NO_ERROR;

error_free:
  free(sound_model_);
  sound_model_ = nullptr;

error_close:
  result = qsthw_unload_module(module_);
  if (result != 0)
    QMMF_ERROR("%s() failed to close qsthw module: result[%d]",
               __func__, result);
  module_ = nullptr;

error_handle:
  current_handle_ = 0;

  return ::android::FAILED_TRANSACTION;
}

status_t SystemTrigger::UnloadSoundModel(const SystemHandle system_handle) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);
  int32_t result = 0;

  if (current_handle_ != system_handle) {
    QMMF_ERROR("%s() invalid system handle", __func__);
    return ::android::INVALID_OPERATION;
  }

  if (trigger_handler_ != nullptr) {
    result = qsthw_stop_recognition(module_, sm_handle_);
    if (result != 0)
      QMMF_ERROR("%s() failed to stop recognition: result[%d]",
                 __func__, result);
  }

  free(sound_model_);
  sound_model_ = nullptr;

  result = qsthw_unload_sound_model(module_, sm_handle_);
  if (result != 0)
    QMMF_ERROR("%s() failed to unload sound model: result[%d]",
               __func__, result);
  sm_handle_ = 0;

  result = qsthw_unload_module(module_);
  if (result != 0)
    QMMF_ERROR("%s() failed to close qsthw module: result[%d]",
               __func__, result);
  module_ = nullptr;

  current_handle_ = 0;

  return ::android::NO_ERROR;
}

status_t SystemTrigger::EnableSoundTrigger(const SystemHandle system_handle,
                                           const TriggerConfig& trigger_config,
                                           const SystemTriggerHandler& handler) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);
  int32_t result;

  if (current_handle_ != system_handle) {
    QMMF_ERROR("%s() invalid system handle", __func__);
    return ::android::INVALID_OPERATION;
  }
  if (trigger_handler_ != nullptr) {
    QMMF_ERROR("%s() trigger already enabled", __func__);
    return ::android::INVALID_OPERATION;
  }
  if (trigger_config.with_keyword) {
    QMMF_ERROR("%s() keyword capture not supported", __func__);
    return ::android::INVALID_OPERATION;
  }

  trigger_handler_ = handler;

  if (trigger_config.request_capture) {
    if (trigger_config.with_keyword) {
      capture_duration_ = trigger_config.capture_duration +
                          trigger_config.keyword_duration;
    } else {
      capture_duration_ = trigger_config.capture_duration;
    }
  } else {
    capture_duration_ = 0;
  }

  int32_t opaque_data_size = 0;
  if (trigger_config.with_keyword)
    opaque_data_size = sizeof(struct keyword_buffer_config);

  size_t rc_config_size = sizeof(struct sound_trigger_recognition_config)
                          + opaque_data_size;
  rc_config_ =
      (struct sound_trigger_recognition_config*)calloc(1, rc_config_size);
  if (rc_config_ == nullptr) {
    QMMF_ERROR("%s() failed to allocate memory for recognition config",
               __func__);
    goto error_handler;
  }

  rc_config_->capture_handle = AUDIO_IO_HANDLE_NONE;
  rc_config_->capture_device = AUDIO_DEVICE_NONE;
  rc_config_->capture_requested = trigger_config.request_capture;
  rc_config_->num_phrases = sound_model_->num_phrases;
  for (uint32_t idx = 0; idx < rc_config_->num_phrases; ++idx) {
    rc_config_->phrases[idx].id = idx;
    rc_config_->phrases[idx].recognition_modes = RECOGNITION_MODE_VOICE_TRIGGER;
    rc_config_->phrases[idx].confidence_level = 60;
    rc_config_->phrases[idx].num_levels = 1;
    rc_config_->phrases[idx].levels[0].level = 60;
    rc_config_->phrases[idx].levels[0].user_id = rc_config_->num_phrases;
  }

  if (trigger_config.with_keyword) {
    struct keyword_buffer_config kb_config;

    kb_config.version = 1;
    kb_config.kb_duration = trigger_config.keyword_duration;
    rc_config_->data_size = opaque_data_size;
    rc_config_->data_offset = sizeof(struct sound_trigger_recognition_config);

    memcpy((char*)rc_config_ + rc_config_->data_offset, &kb_config,
           opaque_data_size);
  }

  memset(&qsthw_event_, 0, sizeof(struct qsthw_phrase_recognition_event));

  result = qsthw_start_recognition(module_, sm_handle_, rc_config_,
                                   SystemTrigger::EventCallbackEntry, this);
  if (result != 0) {
    QMMF_ERROR("%s() failed to start recognition: result[%d]",
               __func__, result);
    goto error_free;
  }

  return ::android::NO_ERROR;

error_free:
  free(rc_config_);
  rc_config_ = nullptr;

error_handler:
  trigger_handler_ = nullptr;

  return ::android::FAILED_TRANSACTION;
}

status_t SystemTrigger::DisableSoundTrigger(const SystemHandle system_handle) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);

  if (current_handle_ != system_handle) {
    QMMF_ERROR("%s() invalid system handle", __func__);
    return ::android::INVALID_OPERATION;
  }
  if (trigger_handler_ == nullptr) {
    QMMF_ERROR("%s() trigger already disabled", __func__);
    return ::android::INVALID_OPERATION;
  }

  int32_t result = qsthw_stop_recognition(module_, sm_handle_);
  if (result != 0)
    QMMF_ERROR("%s() failed to stop recognition: result[%d]",
               __func__, result);

  trigger_handler_ = nullptr;

  if (rc_config_ != nullptr) {
    free(rc_config_);
    rc_config_ = nullptr;
  }

  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
    thread_ = nullptr;
  }

  return ::android::NO_ERROR;
}

void SystemTrigger::EventCallbackEntry(
    struct sound_trigger_recognition_event* event, void* object) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: status[%d]", __func__, event->status);

  SystemTrigger* systrigger = reinterpret_cast<SystemTrigger*>(object);
  systrigger->EventCallback(event);
}

void SystemTrigger::EventCallback(
    struct sound_trigger_recognition_event* event) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: status[%d]", __func__, event->status);

  struct qsthw_phrase_recognition_event* qsthw_event =
      reinterpret_cast<struct qsthw_phrase_recognition_event*>(event);
  memcpy(&qsthw_event_, qsthw_event,
         sizeof(struct qsthw_phrase_recognition_event));

  if (qsthw_event_.phrase_event.common.capture_available) {
    if (thread_ != nullptr) {
      thread_->join();
      delete thread_;
      thread_ = nullptr;
    }
    thread_ = new thread(SystemTrigger::CaptureThreadEntry, this);
    if (thread_ == nullptr)
      QMMF_ERROR("%s() unable to allocate capture thread", __func__);
  } else {
    memset(&qsthw_event_, 0, sizeof(struct qsthw_phrase_recognition_event));

    BufferDescriptor null_buffer = { nullptr, -1, 0, 0, 0, 0, 0, 0 };
    trigger_handler_(current_handle_, event->status, null_buffer);

    int32_t result = qsthw_start_recognition(module_, sm_handle_, rc_config_,
                                             EventCallbackEntry, this);
    if (result != 0)
      QMMF_ERROR("%s() failed to start recognition: result[%d]",
                 __func__, result);
  }
}

void SystemTrigger::CaptureThreadEntry(SystemTrigger* system) {
  QMMF_DEBUG("%s() TRACE", __func__);

  system->CaptureThread();
}

void SystemTrigger::CaptureThread() {
  QMMF_DEBUG("%s() TRACE", __func__);
  struct sound_trigger_phrase_recognition_event phrase_event =
      qsthw_event_.phrase_event;
  audio_config_t* audio_config = &phrase_event.common.audio_config;
  uint32_t sample_rate = audio_config->sample_rate;
  uint32_t channels =
      audio_channel_count_from_in_mask(audio_config->channel_mask);
  size_t sample_size = audio_bytes_per_sample(audio_config->format);

  size_t read_size = qsthw_get_buffer_size(module_, sm_handle_);
  if (read_size <= 0) {
    QMMF_ERROR("%s() invalid buffer size returned: result[%d]",
               __func__, read_size);
    return;
  }

  size_t buffer_size = ((sample_rate * channels * sample_size) *
                       capture_duration_) / 1000;
  QMMF_DEBUG("%s() sample_rate[%d] channels[%d] sample_size[%d] capture_duration_[%d] buffer_size[%d]",
             __func__, sample_rate, channels, sample_size,
             capture_duration_, buffer_size);

  uint8_t* buffer = (uint8_t*)calloc(1, buffer_size);
  if (buffer == nullptr) {
    QMMF_ERROR("%s() could not allocate memory for buffer", __func__);
    return;
  }

  size_t current_bytes_read = 0;
  while (current_bytes_read <= buffer_size - read_size) {
    qsthw_read_buffer(module_, sm_handle_, buffer + current_bytes_read,
                      read_size);
    current_bytes_read += read_size;
  }
  QMMF_DEBUG("%s() total bytes read[%d]", __func__,
             current_bytes_read);

  qsthw_stop_buffering(module_, sm_handle_);

  BufferDescriptor buffer_descriptor;
  buffer_descriptor.data = buffer;
  buffer_descriptor.fd = -1;
  buffer_descriptor.buf_id = -1;
  buffer_descriptor.size = current_bytes_read;
  buffer_descriptor.capacity = buffer_size;
  buffer_descriptor.offset = 0;
  buffer_descriptor.timestamp = qsthw_event_.timestamp;
  buffer_descriptor.flag = 0;

  trigger_handler_(current_handle_, 0, buffer_descriptor);
  free(buffer);

  int32_t result = qsthw_start_recognition(module_, sm_handle_, rc_config_,
                                           EventCallbackEntry, this);
  if (result != 0)
    QMMF_ERROR("%s() failed to start recognition: result[%d]",
               __func__, result);
}

}; // namespace system
}; // namespace qmmf
