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
#include <mutex>
#include <vector>

#include <binder/IBinder.h>
#include <utils/RefBase.h>

#include "qmmf-sdk/qmmf_system_params.h"
#include "system/src/service/qmmf_system_service_interface.h"
#include "system/src/service/qmmf_system_common.h"
#include "common/qmmf_log.h"

namespace qmmf {
namespace system {

class SystemClient {
 public:
  SystemClient();
  ~SystemClient();

  status_t Connect(const SystemCb& callback);
  status_t Disconnect();

  status_t LoadSoundModel(const SoundModel& soundmodel);
  status_t UnloadSoundModel();
  status_t EnableSoundTrigger(const TriggerCb& callback);
  status_t DisableSoundTrigger();

  status_t RegisterForDeviceEvents(const DeviceCb& callback);
  status_t QueryDeviceInfo(::std::vector<DeviceInfo>* devices);
  status_t QueryDeviceCapabilities(const DeviceId device,
                                   DeviceCaps* caps);

  status_t QueryCodecInfo(::std::vector<CodecInfo>* codecs);

  status_t PlayTone(const ::std::vector<DeviceId>& devices,
                    const Tone& tone,
                    const ToneCb& callback);

  // callbacks from service
  void NotifySystemEvent(const int32_t error);
  void NotifyTriggerEvent(const int32_t error);
  void NotifyDeviceEvent(const DeviceInfo& device);
  void NotifyToneEvent(const int32_t error);

 private:
  class DeathNotifier : public ::android::IBinder::DeathRecipient {
   public:
    DeathNotifier(SystemClient* parent) : parent_(parent) {}

    void binderDied(const ::android::wp<::android::IBinder>&) override {
      QMMF_WARN("%s() system service died", __func__);

      ::std::lock_guard<::std::mutex> lock(parent_->lock_);
      parent_->system_service_.clear();
      parent_->system_service_ = nullptr;
    }

    SystemClient* parent_;
  };
  friend class DeathNotifier;

  ::std::mutex lock_;
  ::android::sp<ISystemService> system_service_;
  ::android::sp<DeathNotifier> death_notifier_;
  SystemHandle system_handle_;
  SystemCb system_callback_;
  TriggerCb trigger_callback_;
  DeviceCb device_callback_;
  ToneCb tone_callback_;

  // Disable copy, assignment, and move
  SystemClient(const SystemClient&) = delete;
  SystemClient(SystemClient&&) = delete;
  SystemClient& operator=(const SystemClient&) = delete;
  SystemClient& operator=(const SystemClient&&) = delete;
};

class ServiceCallbackHandler : public BnSystemServiceCallback {
 public:
  ServiceCallbackHandler(SystemClient* client);
  ~ServiceCallbackHandler();

 private:
  // methods of BnSystemServiceCallback
  void NotifySystemEvent(const int32_t error);
  void NotifyTriggerEvent(const int32_t error);
  void NotifyDeviceEvent(const DeviceInfo& device);
  void NotifyToneEvent(const int32_t error);

  SystemClient *client_;
};

}; // namespace system
}; // namespace qmmf
