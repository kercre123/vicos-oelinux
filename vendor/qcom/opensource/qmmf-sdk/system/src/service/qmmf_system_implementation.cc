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

#define LOG_TAG "SystemImplementation"

#include "system/src/service/qmmf_system_implementation.h"

#include <functional>
#include <map>
#include <vector>
#include <type_traits>

#include "qmmf-sdk/qmmf_system_params.h"
#include "system/src/service/qmmf_system_common.h"
#include "system/src/service/qmmf_system_devices.h"
#include "system/src/service/qmmf_system_keytone.h"
#include "common/utils/qmmf_log.h"

namespace qmmf {
namespace system {

using ::std::vector;

const SystemHandle SystemImplementation::kSystemHandleMax = 1000;

SystemImplementation::SystemImplementation() : current_handle_(0) {}

SystemImplementation::~SystemImplementation() {}

void SystemImplementation::RegisterErrorHandler(
    const SystemErrorHandler& handler) {
  QMMF_DEBUG("%s() TRACE", __func__);

  error_handler_ = handler;
}

void SystemImplementation::RegisterTriggerHandler(
    const SystemTriggerHandler& handler) {
  QMMF_DEBUG("%s() TRACE", __func__);

  trigger_handler_ = handler;
}

void SystemImplementation::RegisterDeviceHandler(
    const SystemDeviceHandler& handler) {
  QMMF_DEBUG("%s() TRACE", __func__);

  device_handler_ = handler;
}

void SystemImplementation::RegisterToneHandler(
    const SystemToneHandler& handler) {
  QMMF_DEBUG("%s() TRACE", __func__);

  tone_handler_ = handler;
}

status_t SystemImplementation::Connect(SystemHandle* system_handle) {
  QMMF_DEBUG("%s() TRACE", __func__);

  // find an available SystemHandle
  if (current_handle_ + 1 > kSystemHandleMax)
    current_handle_ = 0;
  ++current_handle_;

  *system_handle = current_handle_;
  QMMF_VERBOSE("%s() OUTPARAM: system_handle[%d]", __func__,
               *system_handle);

  return 0;
}

status_t SystemImplementation::Disconnect(const SystemHandle system_handle) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);

  status_t result = devices_.UnregisterFromDeviceEvents(system_handle);
  if (result < 0)
    QMMF_ERROR("%s() failed to unregister from device events: %d",
               __func__, result);

  return result;
}

status_t SystemImplementation::LoadSoundModel(const SystemHandle system_handle,
                                              const SoundModel& soundmodel) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);
  QMMF_VERBOSE("%s() INPARAM: soundmodel[%s]", __func__,
               soundmodel.ToString().c_str());

  status_t result = trigger_.LoadSoundModel(system_handle, soundmodel);
  if (result < 0)
    QMMF_ERROR("%s() failed to load SoundModel: %d", __func__, result);

  return result;
}

status_t SystemImplementation::UnloadSoundModel(
    const SystemHandle system_handle) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);

  status_t result = trigger_.UnloadSoundModel(system_handle);
  if (result < 0)
    QMMF_ERROR("%s() failed to unload SoundModel: %d",
               __func__, result);

  return result;
}

status_t SystemImplementation::EnableSoundTrigger(
    const SystemHandle system_handle, const TriggerConfig& config) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);
  QMMF_VERBOSE("%s() INPARAM: config[%s]", __func__,
               config.ToString().c_str());

  status_t result = trigger_.EnableSoundTrigger(system_handle, config,
                                                trigger_handler_);
  if (result < 0)
    QMMF_ERROR("%s() failed to enable SoundTrigger: %d",
               __func__, result);

  return result;
}

status_t SystemImplementation::DisableSoundTrigger(
    const SystemHandle system_handle) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);

  status_t result = trigger_.DisableSoundTrigger(system_handle);
  if (result < 0)
    QMMF_ERROR("%s() failed to disable SoundTrigger: %d",
               __func__, result);

  return result;
}

status_t SystemImplementation::RegisterForDeviceEvents(
    const SystemHandle system_handle) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);

  status_t result = devices_.RegisterForDeviceEvents(system_handle,
                                                     device_handler_);
  if (result < 0)
    QMMF_ERROR("%s() failed to register for device events: %d",
               __func__, result);

  return result;
}

status_t SystemImplementation::QueryDeviceInfo(const SystemHandle system_handle,
                                               vector<DeviceInfo>* devices) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);

  status_t result = devices_.QueryDeviceInfo(system_handle, devices);
  if (result < 0)
    QMMF_ERROR("%s() failed to retrieve device info: %d",
               __func__, result);

  for (const DeviceInfo& device : *devices)
    QMMF_VERBOSE("%s() OUTPARAM: device[%s]", __func__,
                 device.ToString().c_str());
  return result;
}

status_t SystemImplementation::QueryDeviceCapabilities(
    const SystemHandle system_handle, const DeviceId device, DeviceCaps* caps) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);
  QMMF_VERBOSE("%s() INPARAM: device[%d]", __func__, device);

  status_t result = devices_.QueryDeviceCapabilities(system_handle, device,
                                                     caps);
  if (result < 0)
    QMMF_ERROR("%s() failed to retrieve device capabilities: %d",
               __func__, result);

  QMMF_VERBOSE("%s() OUTPARAM: caps[%s]", __func__,
               caps->ToString().c_str());
  return result;
}

status_t SystemImplementation::QueryCodecInfo(const SystemHandle system_handle,
                                              vector<CodecInfo>* codecs) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);

  for (const CodecInfo& codec : *codecs)
    QMMF_VERBOSE("%s() OUTPARAM: codec[%s]", __func__,
                 codec.ToString().c_str());
  return 0;
}

status_t SystemImplementation::PlayTone(const SystemHandle system_handle,
                                        const vector<DeviceId>& devices,
                                        const Tone& tone) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);
  for (const DeviceId& device : devices)
    QMMF_VERBOSE("%s() INPARAM: device[%d]", __func__, device);
  QMMF_VERBOSE("%s() INPARAM: tone[%s]", __func__,
               tone.ToString().c_str());

  status_t result = keytone_.PlayTone(system_handle, devices, tone,
                                      tone_handler_);
  if (result < 0)
    QMMF_ERROR("%s() failed to play tone: %d", __func__, result);

  return result;
}

status_t SystemImplementation::Mute(const SystemHandle system_handle,
                                        const DeviceId device,
                                        const bool mute) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);
  QMMF_VERBOSE("%s() INPARAM: device[%d]", __func__, device);
  QMMF_VERBOSE("%s() INPARAM: mute[%s]", __func__,
               mute ? "true" : "false");

  status_t result = devices_.Mute(system_handle, device, mute);
  if (result < 0)
    QMMF_ERROR("%s() failed to apply mute: %d", __func__, result);

  return result;
}

}; // namespace system
}; // namespace qmmf
