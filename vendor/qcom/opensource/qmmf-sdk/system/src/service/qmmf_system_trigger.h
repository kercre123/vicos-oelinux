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

#pragma once

#include <thread>

#include <mm-audio/qsthw_api/qsthw_defs.h>
#include <mm-audio/qsthw_api/qsthw_api.h>

#include "qmmf-sdk/qmmf_system_params.h"
#include "system/src/service/qmmf_system_common.h"

namespace qmmf {
namespace system {

class SystemTrigger {
 public:
  SystemTrigger();
  ~SystemTrigger();

  status_t LoadSoundModel(const SystemHandle system_handle,
                          const SoundModel& soundmodel);
  status_t UnloadSoundModel(const SystemHandle system_handle);
  status_t EnableSoundTrigger(const SystemHandle system_handle,
                              const TriggerConfig& trigger_config,
                              const SystemTriggerHandler& handler);
  status_t DisableSoundTrigger(const SystemHandle system_handle);

 private:
  struct keyword_buffer_config {
    int32_t  version;
    uint32_t kb_duration;
  } __packed;

  static const sound_trigger_uuid_t kQcUuid;

  static void EventCallbackEntry(struct sound_trigger_recognition_event* event,
                                 void* object);
  void EventCallback(struct sound_trigger_recognition_event* event);
  static void CaptureThreadEntry(SystemTrigger* backend);
  void CaptureThread();

  ::std::thread* thread_;

  SystemHandle current_handle_;
  SystemTriggerHandler trigger_handler_;

  const qsthw_module_handle_t* module_;
  sound_trigger_phrase_sound_model* sound_model_;
  sound_model_handle_t sm_handle_;
  sound_trigger_recognition_config* rc_config_;
  struct qsthw_phrase_recognition_event qsthw_event_;
  uint32_t capture_duration_;

  // disable copy, assignment, and move
  SystemTrigger(const SystemTrigger&) = delete;
  SystemTrigger(SystemTrigger&&) = delete;
  SystemTrigger& operator=(const SystemTrigger&) = delete;
  SystemTrigger& operator=(const SystemTrigger&&) = delete;
};

}; // namespace system
}; // namespace qmmf
