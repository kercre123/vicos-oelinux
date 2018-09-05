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

#define LOG_TAG "SystemDevices"

#include "system/src/service/qmmf_system_devices.h"

#include <cstdlib>
#include <functional>
#include <vector>

#include <qmmf-sdk/qmmf_device.h>
#include <qmmf-sdk/qmmf_system_params.h>

#include "common/utils/qmmf_log.h"
#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/inc/qmmf_audio_endpoint.h"
#include "system/src/service/qmmf_system_common.h"

namespace qmmf {
namespace system {

using ::qmmf::common::audio::AudioEndPoint;
using ::qmmf::common::audio::AudioEventData;
using ::qmmf::common::audio::AudioEventHandler;
using ::qmmf::common::audio::AudioEventType;
using ::qmmf::common::audio::AudioParamDeviceData;
using ::qmmf::common::audio::AudioParamType;
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
  QMMF_DEBUG("%s() TRACE", __func__);

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
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);

  if (!device_handler_)
    device_handler_ = handler;

  for (SystemHandle handle : registered_handles_) {
    if (handle == system_handle) {
      QMMF_WARN("%s() system_handle[%d] is already registered",
                __func__, system_handle);
      return 0;
    }
  }

  registered_handles_.push_back(system_handle);
  QMMF_DEBUG("%s() registered system_handle[%d] for device events",
            __func__, system_handle);

  return 0;
}

status_t SystemDevices::UnregisterFromDeviceEvents(
    const SystemHandle system_handle) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);

  for (uint32_t idx = 0; idx < registered_handles_.size(); ++idx) {
    if (registered_handles_[idx] == system_handle) {
      registered_handles_.erase(registered_handles_.begin() + idx);
      QMMF_DEBUG("%s() unregistered system_handle[%d] for device events",
                __func__, system_handle);
      break;
    }
  }

  if (registered_handles_.size() == 0)
    device_handler_ = nullptr;

  return 0;
}

status_t SystemDevices::QueryDeviceInfo(const SystemHandle system_handle,
                                        vector<DeviceInfo>* devices) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);

  for (const SystemDevice& device : devices_)
    devices->push_back(device.info);

  for (const DeviceInfo& device : *devices)
    QMMF_VERBOSE("%s() OUTPARAM: device[%s]", __func__,
                 device.ToString().c_str());
  return 0;
}

status_t SystemDevices::QueryDeviceCapabilities(
    const SystemHandle system_handle,
    const DeviceId device,
    DeviceCaps* caps) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);
  QMMF_VERBOSE("%s() INPARAM: device[%d]", __func__, device);

  for (const SystemDevice& system_device : devices_) {
    if (system_device.info.id == device)
      *caps = system_device.caps;
  }

  QMMF_VERBOSE("%s() OUTPARAM: caps[%s]", __func__,
               caps->ToString().c_str());
  return 0;
}

status_t SystemDevices::Mute(const SystemHandle system_handle,
                             const DeviceId device,
                             const bool mute) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
               system_handle);
  QMMF_VERBOSE("%s() INPARAM: device[%d]", __func__, device);
  QMMF_VERBOSE("%s() INPARAM: mute[%s]", __func__,
               mute ? "true" : "false");
  int32_t result;
  AudioEndPoint end_point;

  AudioEventHandler audio_handler =
    [] (AudioEventType event_type, const AudioEventData& event_data)
           -> void {
      switch (event_type) {
        case AudioEventType::kError:
        case AudioEventType::kBuffer:
        case AudioEventType::kStopped:
          // do nothing
          break;
      }
    };

  result = end_point.Connect(audio_handler);
  if (result < 0) {
    QMMF_ERROR("%s() endpoint->Connect failed: %d[%s]", __func__,
               result, strerror(result));
    return ::android::FAILED_TRANSACTION;
  }

  AudioParamDeviceData device_data;
  device_data.enable = mute;
  device_data.id = device;

  result = end_point.SetParam(AudioParamType::kMute, {device_data});
  if (result < 0)
    QMMF_ERROR("%s() endpoint->SetParam failed: %d[%s]", __func__,
               result, strerror(result));

  result = end_point.Disconnect();
  if (result < 0)
    QMMF_ERROR("%s() endpoint->Disconnect failed: %d[%s]", __func__,
               result, strerror(result));

  return ::android::NO_ERROR;
}

}; // namespace system
}; // namespace qmmf
