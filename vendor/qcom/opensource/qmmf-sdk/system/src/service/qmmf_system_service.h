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
#include <map>
#include <mutex>
#include <vector>

#include <binder/Parcel.h>
#include <utils/RefBase.h>

#include "qmmf-sdk/qmmf_system_params.h"
#include "system/src/service/qmmf_system_service_interface.h"
#include "system/src/service/qmmf_system_common.h"
#include "system/src/service/qmmf_system_implementation.h"
#include "common/qmmf_log.h"

namespace qmmf {
namespace system {

class SystemService : public ::android::BnInterface<ISystemService>
{
 public:
  SystemService();
  ~SystemService();

 private:
  status_t Connect(const ::android::sp<ISystemServiceCallback>& client_handler,
                   SystemHandle* system_handle) override;
  status_t Disconnect(const SystemHandle system_handle) override;

  status_t LoadSoundModel(const SystemHandle system_handle,
                          const SoundModel& soundmodel) override;
  status_t UnloadSoundModel(const SystemHandle system_handle) override;
  status_t EnableSoundTrigger(const SystemHandle system_handle) override;
  status_t DisableSoundTrigger(const SystemHandle system_handle) override;

  status_t RegisterForDeviceEvents(const SystemHandle system_handle) override;
  status_t QueryDeviceInfo(const SystemHandle system_handle,
                           ::std::vector<DeviceInfo>* devices) override;
  status_t QueryDeviceCapabilities(const SystemHandle system_handle,
                                   const DeviceId device,
                                   DeviceCaps* caps) override;

  status_t QueryCodecInfo(const SystemHandle system_handle,
                          ::std::vector<CodecInfo>* codecs) override;

  status_t PlayTone(const SystemHandle system_handle,
                    const ::std::vector<DeviceId>& devices,
                    const Tone& tone) override;

  // methods of BnInterface<ISystemService>
  int32_t onTransact(uint32_t code, const ::android::Parcel& data,
                     ::android::Parcel* reply, uint32_t flags = 0) override;

  class DeathNotifier : public ::android::IBinder::DeathRecipient {
   public:
    DeathNotifier(SystemService* parent, const SystemHandle system_handle)
        : parent_(parent), system_handle_(system_handle) {}

    void binderDied(const ::android::wp<::android::IBinder>&) override {
      QMMF_WARN("%s() system client died", __func__);
      ::std::lock_guard<::std::mutex> lock(parent_->lock_);

      parent_->client_handlers_.find(system_handle_)->second.clear();
      parent_->client_handlers_.erase(system_handle_);
    }

    SystemService* parent_;
    SystemHandle system_handle_;
  };
  friend class DeathNotifier;

  typedef ::std::map<SystemHandle,
                     ::android::sp<DeathNotifier>> DeathNotifierMap;
  typedef ::std::map<SystemHandle,
                     ::android::sp<ISystemServiceCallback>> ClientHandlerMap;

  ::std::mutex lock_;
  SystemImplementation system_impl_;
  DeathNotifierMap death_notifiers_;
  ClientHandlerMap client_handlers_;

  // disable copy, assignment, and move
  SystemService(const SystemService&) = delete;
  SystemService(SystemService&&) = delete;
  SystemService& operator=(const SystemService&) = delete;
  SystemService& operator=(const SystemService&&) = delete;
};

}; // namespace system
}; // namespace qmmf
