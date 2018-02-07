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

#include <map>
#include <vector>

#include "qmmf-sdk/qmmf_system_params.h"
#include "system/src/service/qmmf_system_common.h"
#include "system/src/service/qmmf_system_devices.h"
#include "system/src/service/qmmf_system_keytone.h"
#include "system/src/service/qmmf_system_trigger.h"

namespace qmmf {
namespace system {

class SystemImplementation {
 public:
  SystemImplementation();
  ~SystemImplementation();

  void RegisterErrorHandler(const SystemErrorHandler& handler);
  void RegisterTriggerHandler(const SystemTriggerHandler& handler);
  void RegisterDeviceHandler(const SystemDeviceHandler& handler);
  void RegisterToneHandler(const SystemToneHandler& handler);

  status_t Connect(SystemHandle* system_handle);
  status_t Disconnect(const SystemHandle system_handle);

  status_t LoadSoundModel(const SystemHandle system_handle,
                          const SoundModel& soundmodel);
  status_t UnloadSoundModel(const SystemHandle system_handle);
  status_t EnableSoundTrigger(const SystemHandle system_handle);
  status_t DisableSoundTrigger(const SystemHandle system_handle);

  status_t RegisterForDeviceEvents(const SystemHandle system_handle);
  status_t QueryDeviceInfo(const SystemHandle system_handle,
                           ::std::vector<DeviceInfo>* devices);
  status_t QueryDeviceCapabilities(const SystemHandle system_handle,
                                   const DeviceId device,
                                   DeviceCaps* caps);

  status_t QueryCodecInfo(const SystemHandle system_handle,
                          ::std::vector<CodecInfo>* codecs);

  status_t PlayTone(const SystemHandle system_handle,
                    const ::std::vector<DeviceId>& devices,
                    const Tone& tone);

 private:
  static const SystemHandle kSystemHandleMax;

  SystemHandle current_handle_;
  SystemErrorHandler error_handler_;
  SystemTriggerHandler trigger_handler_;
  SystemDeviceHandler device_handler_;
  SystemToneHandler tone_handler_;

  SystemDevices devices_;
  SystemKeytone keytone_;
  SystemTrigger trigger_;

  // disable copy, assignment, and move
  SystemImplementation(const SystemImplementation&) = delete;
  SystemImplementation(SystemImplementation&&) = delete;
  SystemImplementation& operator=(const SystemImplementation&) = delete;
  SystemImplementation& operator=(const SystemImplementation&&) = delete;
};

}; // namespace system
}; // namespace qmmf
