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

#define TAG "SystemDevices"

#include "system/src/service/qmmf_system_devices.h"

#include <functional>
#include <vector>

#include "common/qmmf_log.h"
#include "qmmf-sdk/qmmf_device.h"
#include "qmmf-sdk/qmmf_system_params.h"
#include "system/src/service/qmmf_system_common.h"

namespace qmmf {
namespace system {

using ::std::vector;

SystemDevices::SystemDevice SystemDevices::hard_wired_devices[] =
{
  { // built-in microphone
    {
      DeviceType::kAudioIn,
      {
        .audio_in = AudioInSubtype::kBuiltIn
      },
      static_cast<DeviceId>(0)
    },
    {
      DeviceType::kAudioIn,
      {
        { AudioFormat::kPCM },
        { 44100, 48000 },
        { 1, 2 },
        { 16 }
      },
      {
        {},
        {},
        {}
      }
    }
  },
  { // built-in speaker
    {
      DeviceType::kAudioOut,
      {
        .audio_out = AudioOutSubtype::kBuiltIn
      },
      static_cast<DeviceId>(0)
    },
    {
      DeviceType::kAudioOut,
      {
        { AudioFormat::kPCM },
        { 44100, 48000 },
        { 1, 2 },
        { 16 }
      },
      {
        {},
        {},
        {}
      }
    }
  },
  { // terminal
    {
      DeviceType::kVideoIn,
      {
        .video_in = VideoInSubtype::kNone
      },
      static_cast<DeviceId>(-1)
    },
    {
      DeviceType::kVideoIn,
      {
        {},
        {},
        {},
        {}
      },
      {
        {},
        {},
        {}
      }
    }
  }
};

SystemDevices::SystemDevices() : current_id_(0) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  uint32_t idx = 0;
  while (hard_wired_devices[idx].info.id != static_cast<DeviceId>(-1)) {
    hard_wired_devices[idx].info.id = idx;
    devices_.push_back(hard_wired_devices[idx]);
    ++idx;
  }

  current_id_ = idx;
}

SystemDevices::~SystemDevices() {}

status_t SystemDevices::RegisterForDeviceEvents(
    const SystemHandle system_handle,
    const SystemDeviceHandler& handler) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);

  if (!device_handler_)
    device_handler_ = handler;

  for (SystemHandle handle : registered_handles_) {
    if (handle == system_handle) {
      QMMF_WARN("%s: %s() system_handle[%d] is already registered",
                TAG, __func__, system_handle);
      return 0;
    }
  }

  registered_handles_.push_back(system_handle);
  QMMF_DEBUG("%s: %s() registered system_handle[%d] for device events",
            TAG, __func__, system_handle);

  return 0;
}

status_t SystemDevices::UnregisterFromDeviceEvents(
    const SystemHandle system_handle) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);

  for (uint32_t idx = 0; idx < registered_handles_.size(); ++idx) {
    if (registered_handles_[idx] == system_handle) {
      registered_handles_.erase(registered_handles_.begin() + idx);
      QMMF_DEBUG("%s: %s() unregistered system_handle[%d] for device events",
                TAG, __func__, system_handle);
      break;
    }
  }

  if (registered_handles_.size() == 0)
    device_handler_ = nullptr;

  return 0;
}

status_t SystemDevices::QueryDeviceInfo(const SystemHandle system_handle,
                                        vector<DeviceInfo>* devices) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);

  for (const SystemDevice& device : devices_)
    devices->push_back(device.info);

  for (const DeviceInfo& device : *devices)
    QMMF_VERBOSE("%s: %s() OUTPARAM: device[%s]", TAG, __func__,
                 device.ToString().c_str());
  return 0;
}

status_t SystemDevices::QueryDeviceCapabilities(
    const SystemHandle system_handle,
    const DeviceId device,
    DeviceCaps* caps) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);
  QMMF_VERBOSE("%s: %s() INPARAM: device[%d]", TAG, __func__, device);

  for (const SystemDevice& system_device : devices_) {
    if (system_device.info.id == device)
      *caps = system_device.caps;
  }

  QMMF_VERBOSE("%s: %s() OUTPARAM: caps[%s]", TAG, __func__,
               caps->ToString().c_str());
  return 0;
}

}; // namespace system
}; // namespace qmmf
