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

#define TAG "SystemTrigger"

#include "system/src/service/qmmf_system_trigger.h"

#include <functional>

#include <hardware/sound_trigger.h>
#include <utils/Errors.h>

#include "common/qmmf_log.h"
#include "qmmf-sdk/qmmf_system_params.h"
#include "system/src/service/qmmf_system_common.h"

namespace qmmf {
namespace system {

SystemTrigger::SystemTrigger()
    : current_handle_(0),
      hw_module_(nullptr),
      hw_device_(nullptr),
      sound_model_(nullptr),
      sm_handle_(0) {}

SystemTrigger::~SystemTrigger() {}

status_t SystemTrigger::LoadSoundModel(const SystemHandle system_handle,
                                       const SoundModel& soundmodel) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);
  QMMF_VERBOSE("%s: %s() INPARAM: soundmodel[%s]", TAG, __func__,
               soundmodel.ToString().c_str());
  int32_t result = 0;

  if (current_handle_ != 0) {
    QMMF_ERROR("%s: %s() already in use", TAG, __func__);
    return ::android::ALREADY_EXISTS;
  }

  current_handle_ = system_handle;

  result = hw_get_module_by_class(SOUND_TRIGGER_HARDWARE_MODULE_ID,
                                  SOUND_TRIGGER_HARDWARE_MODULE_ID_PRIMARY,
                                  &hw_module_);
  if (result != 0) {
    QMMF_ERROR("%s: %s() failed to load STHW module[%s]: result[%d]",
               TAG, __func__, SOUND_TRIGGER_HARDWARE_MODULE_ID_PRIMARY, result);
    return ::android::NO_INIT;
  }

  result = sound_trigger_hw_device_open(hw_module_, &hw_device_);
  if (result != 0 || hw_device_ == nullptr) {
    QMMF_ERROR("%s: %s() failed to open STHW device: result[%d]",
               TAG, __func__, result);
    return ::android::NO_INIT;
  }

  size_t size = sizeof(sound_trigger_phrase_sound_model) + soundmodel.size;
  sound_model_ = (sound_trigger_phrase_sound_model*)calloc(1, size);
  if (sound_model_ == nullptr) {
    QMMF_ERROR("%s: %s() failed to allocate memory for sound model",
               TAG, __func__);
    goto error_close;
  }
  sound_model_->common.type = SOUND_MODEL_TYPE_KEYPHRASE;
  sound_model_->common.data_size = soundmodel.size;
  sound_model_->common.data_offset = sizeof(*sound_model_);
  sound_model_->num_phrases = soundmodel.keywords;
  for (uint32_t idx = 0; idx < sound_model_->num_phrases; ++idx)
    sound_model_->phrases[idx].recognition_mode = RECOGNITION_MODE_VOICE_TRIGGER;

  memcpy((char*)sound_model_ + sound_model_->common.data_offset,
         soundmodel.data, soundmodel.size);

  result = hw_device_->load_sound_model(hw_device_, &sound_model_->common,
                                        NULL, NULL, &sm_handle_);
  if (result != 0) {
    QMMF_ERROR("%s: %s() failed to load sound model: result[%d]", TAG, __func__,
               result);
    goto error_free;
  }

  return ::android::NO_ERROR;

error_free:
  free(sound_model_);
  sound_model_ = nullptr;

error_close:
  result = sound_trigger_hw_device_close(hw_device_);
  if (result != 0)
    QMMF_ERROR("%s: %s() failed to close STHW device: result[%d]",
               TAG, __func__, result);
  hw_device_ = nullptr;
  hw_module_ = nullptr;

  current_handle_ = 0;

  return ::android::FAILED_TRANSACTION;
}

status_t SystemTrigger::UnloadSoundModel(const SystemHandle system_handle) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);
  int32_t result = 0;

  if (current_handle_ == 0) {
    QMMF_ERROR("%s: %s() not currently in use", TAG, __func__);
    return ::android::INVALID_OPERATION;
  }

  result = hw_device_->unload_sound_model(hw_device_, sm_handle_);
  if (result != 0)
    QMMF_ERROR("%s: %s() failed to unload sound model: result[%d]",
               TAG, __func__, result);
  sm_handle_ = 0;

  free(sound_model_);
  sound_model_ = nullptr;

  result = sound_trigger_hw_device_close(hw_device_);
  if (result != 0)
    QMMF_ERROR("%s: %s() failed to close STHW device: result[%d]",
               TAG, __func__, result);
  hw_device_ = nullptr;
  hw_module_ = nullptr;

  current_handle_ = 0;

  return ::android::NO_ERROR;
}

static void StaticEventCallback(sound_trigger_recognition_event* event,
                                void* object) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: status[%d]", TAG, __func__, event->status);

  SystemTrigger* systrigger = reinterpret_cast<SystemTrigger*>(object);
  systrigger->EventCallback(event->status);
}

void SystemTrigger::EventCallback(const int32_t error) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: error[%d]", TAG, __func__, error);

  trigger_handler_(current_handle_, error);
}

status_t SystemTrigger::EnableSoundTrigger(const SystemHandle system_handle,
                                           const SystemTriggerHandler& handler) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);

  if (current_handle_ == 0) {
    QMMF_ERROR("%s: %s() not currently in use", TAG, __func__);
    return ::android::INVALID_OPERATION;
  }

  trigger_handler_ = handler;

  memset(&rc_config_, 0, sizeof(rc_config_));
  rc_config_.capture_handle = AUDIO_IO_HANDLE_NONE;
  rc_config_.capture_device = AUDIO_DEVICE_NONE;
  rc_config_.capture_requested = 0;
  rc_config_.num_phrases = sound_model_->num_phrases;
  for (uint32_t idx = 0; idx < rc_config_.num_phrases; ++idx) {
    rc_config_.phrases[idx].id = idx;
    rc_config_.phrases[idx].recognition_modes = RECOGNITION_MODE_VOICE_TRIGGER;
    rc_config_.phrases[idx].confidence_level = 60;
  }

  int32_t result = hw_device_->start_recognition(hw_device_, sm_handle_,
                                                 &rc_config_,
                                                 StaticEventCallback, this);
  if (result != 0) {
    QMMF_ERROR("%s: %s() failed to start recognition: result[%d]",
               TAG, __func__, result);
    trigger_handler_ = nullptr;
    return ::android::FAILED_TRANSACTION;
  }

  return ::android::NO_ERROR;
}

status_t SystemTrigger::DisableSoundTrigger(const SystemHandle system_handle) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);

  if (current_handle_ == 0) {
    QMMF_ERROR("%s: %s() not currently in use", TAG, __func__);
    return ::android::INVALID_OPERATION;
  }

  int32_t result = hw_device_->stop_recognition(hw_device_, sm_handle_);
  if (result != 0)
    QMMF_ERROR("%s: %s() failed to stop recognition: result[%d]",
               TAG, __func__, result);

  trigger_handler_ = nullptr;

  return ::android::NO_ERROR;
}

}; // namespace system
}; // namespace qmmf
