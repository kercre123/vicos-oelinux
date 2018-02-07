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

#include <cstdint>
#include <vector>

#include <binder/IBinder.h>
#include <binder/IInterface.h>
#include <binder/Parcel.h>
#include <utils/Errors.h>
#include <utils/RefBase.h>

#include "qmmf-sdk/qmmf_system_params.h"
#include "system/src/service/qmmf_system_common.h"

namespace qmmf {
namespace system {

enum class SystemServiceCommand {
  kSystemConnect = ::android::IBinder::FIRST_CALL_TRANSACTION,
  kSystemDisconnect,
  kSystemLoadSoundModel,
  kSystemUnloadSoundModel,
  kSystemEnableSoundTrigger,
  kSystemDisableSoundTrigger,
  kSystemRegisterForDeviceEvents,
  kSystemQueryDeviceInfo,
  kSystemQueryDeviceCapabilities,
  kSystemQueryCodecInfo,
  kSystemPlayTone,
};

enum class SystemServiceCallbackCommand {
  kSystemNotifySystem = ::android::IBinder::FIRST_CALL_TRANSACTION,
  kSystemNotifyTrigger,
  kSystemNotifyDevice,
  kSystemNotifyTone,
};

#define QMMF_SYSTEM_SERVICE_NAME "qmmf_system.service"

// Binder interface for callbacks from SystemService to SystemClient
class ISystemServiceCallback : public ::android::IInterface {
 public:
  DECLARE_META_INTERFACE(SystemServiceCallback);

  virtual void NotifySystemEvent(const int32_t error) = 0;
  virtual void NotifyTriggerEvent(const int32_t error) = 0;
  virtual void NotifyDeviceEvent(const DeviceInfo& device) = 0;
  virtual void NotifyToneEvent(const int32_t error) = 0;
};

class ISystemService : public ::android::IInterface {
 public:
  DECLARE_META_INTERFACE(SystemService);

  virtual status_t Connect(
      const ::android::sp<ISystemServiceCallback>& client_handler,
      SystemHandle* system_handle) = 0;
  virtual status_t Disconnect(const SystemHandle system_handle) = 0;

  virtual status_t LoadSoundModel(const SystemHandle system_handle,
                                  const SoundModel& soundmodel) = 0;
  virtual status_t UnloadSoundModel(const SystemHandle system_handle) = 0;
  virtual status_t EnableSoundTrigger(const SystemHandle system_handle) = 0;
  virtual status_t DisableSoundTrigger(const SystemHandle system_handle) = 0;

  virtual status_t RegisterForDeviceEvents(const SystemHandle system_handle) = 0;
  virtual status_t QueryDeviceInfo(const SystemHandle system_handle,
                                   ::std::vector<DeviceInfo>* devices) = 0;
  virtual status_t QueryDeviceCapabilities(const SystemHandle system_handle,
                                           const DeviceId device,
                                           DeviceCaps* caps) = 0;

  virtual status_t QueryCodecInfo(const SystemHandle system_handle,
                                  ::std::vector<CodecInfo>* codecs) = 0;

  virtual status_t PlayTone(const SystemHandle system_handle,
                            const ::std::vector<DeviceId>& devices,
                            const Tone& tone) = 0;
};

// this class is responsible to provide callbacks from system service
class BnSystemServiceCallback
    : public ::android::BnInterface<ISystemServiceCallback> {
 public:
  virtual status_t onTransact(uint32_t code, const ::android::Parcel &data,
                              ::android::Parcel *reply, uint32_t flags = 0);
};

}; // namespace system
}; // namespace qmmf
