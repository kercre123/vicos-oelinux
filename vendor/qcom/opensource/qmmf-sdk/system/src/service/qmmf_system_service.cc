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

#define TAG "SystemService"

#include "system/src/service/qmmf_system_service.h"

#include <cerrno>
#include <cstdint>
#include <map>
#include <mutex>
#include <vector>

#include <binder/IInterface.h>
#include <binder/Parcel.h>
#include <utils/RefBase.h>

#include "common/qmmf_codec_internal.h"
#include "common/qmmf_device_internal.h"
#include "common/qmmf_log.h"
#include "qmmf-sdk/qmmf_system_params.h"
#include "system/src/client/qmmf_system_params_internal.h"
#include "system/src/service/qmmf_system_common.h"
#include "system/src/service/qmmf_system_implementation.h"

namespace qmmf {
namespace system {

using ::android::IInterface;
using ::android::interface_cast;
using ::android::Parcel;
using ::android::sp;
using ::std::lock_guard;
using ::std::map;
using ::std::mutex;
using ::std::vector;

SystemService::SystemService() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  SystemErrorHandler error_handler =
    [this](const SystemHandle system_handle, const int32_t error) -> void {
      ClientHandlerMap::iterator client_handler_iterator =
          client_handlers_.find(system_handle);
      if (client_handler_iterator == client_handlers_.end()) {
        QMMF_ERROR("%s: %s() no client handler for key[%d]", TAG, __func__,
                   system_handle);
        return;
      }

      client_handler_iterator->second->NotifySystemEvent(error);
    };

  SystemTriggerHandler trigger_handler =
    [this](const SystemHandle system_handle, const int32_t error) -> void {
      ClientHandlerMap::iterator client_handler_iterator =
          client_handlers_.find(system_handle);
      if (client_handler_iterator == client_handlers_.end()) {
        QMMF_ERROR("%s: %s() no client handler for key[%d]", TAG, __func__,
                   system_handle);
        return;
      }

      client_handler_iterator->second->NotifyTriggerEvent(error);
    };

  SystemDeviceHandler device_handler =
    [this](const SystemHandle system_handle, const DeviceInfo& device) -> void {
      ClientHandlerMap::iterator client_handler_iterator =
          client_handlers_.find(system_handle);
      if (client_handler_iterator == client_handlers_.end()) {
        QMMF_ERROR("%s: %s() no client handler for key[%d]", TAG, __func__,
                   system_handle);
        return;
      }

      client_handler_iterator->second->NotifyDeviceEvent(device);
    };

  SystemToneHandler tone_handler =
    [this](const SystemHandle system_handle, const int32_t error) -> void {
      ClientHandlerMap::iterator client_handler_iterator =
          client_handlers_.find(system_handle);
      if (client_handler_iterator == client_handlers_.end()) {
        QMMF_ERROR("%s: %s() no client handler for key[%d]", TAG, __func__,
                   system_handle);
        return;
      }

      client_handler_iterator->second->NotifyToneEvent(error);
    };

  system_impl_.RegisterErrorHandler(error_handler);
  system_impl_.RegisterTriggerHandler(trigger_handler);
  system_impl_.RegisterDeviceHandler(device_handler);
  system_impl_.RegisterToneHandler(tone_handler);

  QMMF_INFO("%s: %s() service instantiated", TAG, __func__);
}

SystemService::~SystemService()
{
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  death_notifiers_.clear();
  client_handlers_.clear();
  QMMF_INFO("%s: %s: service destroyed", TAG, __func__);
}

status_t SystemService::Connect(const sp<ISystemServiceCallback>& client_handler,
                                SystemHandle* system_handle) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  lock_guard<mutex> lock(lock_);

  status_t result = system_impl_.Connect(system_handle);
  if (result < 0) {
    QMMF_ERROR("%s: %s() impl->Connect failed: %d", TAG, __func__, result);
    return result;
  }

  sp<DeathNotifier> death_notifier = new DeathNotifier(this, *system_handle);
  if (death_notifier.get() == nullptr) {
    QMMF_ERROR("%s: %s() unable to allocate death notifier", TAG, __func__);
    return -ENOMEM;
  }
  IInterface::asBinder(client_handler)->linkToDeath(death_notifier);

  death_notifiers_.insert({*system_handle, death_notifier});
  client_handlers_.insert({*system_handle, client_handler});

  QMMF_VERBOSE("%s: %s() OUTPARAM: system_handle[%d]", TAG, __func__,
               *system_handle);
  return result;
}

status_t SystemService::Disconnect(const SystemHandle system_handle) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);
  lock_guard<mutex> lock(lock_);

  status_t result = system_impl_.Disconnect(system_handle);
  if (result < 0) {
    QMMF_ERROR("%s: %s() impl->Disconnect failed: %d", TAG, __func__,
               result);
    return result;
  }

  ClientHandlerMap::iterator client_handler_iterator =
      client_handlers_.find(system_handle);
  if (client_handler_iterator == client_handlers_.end()) {
    QMMF_ERROR("%s: %s() no client handler for key[%d]", TAG, __func__,
               system_handle);
    return -EINVAL;
  }

  DeathNotifierMap::iterator death_notifier_iterator =
      death_notifiers_.find(system_handle);
  if (death_notifier_iterator == death_notifiers_.end()) {
    QMMF_ERROR("%s: %s() no death notifier for key[%d]", TAG, __func__,
               system_handle);
    return -EINVAL;
  }

  IInterface::asBinder(client_handler_iterator->second)->
      unlinkToDeath(death_notifier_iterator->second);
  client_handler_iterator->second.clear();
  death_notifier_iterator->second.clear();
  client_handlers_.erase(client_handler_iterator);
  death_notifiers_.erase(death_notifier_iterator);

  return 0;
}

status_t SystemService::LoadSoundModel(const SystemHandle system_handle,
                                       const SoundModel& soundmodel) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);
  QMMF_VERBOSE("%s: %s() INPARAM: soundmodel[%s]", TAG, __func__,
               soundmodel.ToString().c_str());

  status_t result = system_impl_.LoadSoundModel(system_handle, soundmodel);
  if (result < 0)
    QMMF_ERROR("%s: %s() impl->LoadSoundModel failed: %d", TAG, __func__,
               result);

  return result;
}

status_t SystemService::UnloadSoundModel(const SystemHandle system_handle) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);

  status_t result = system_impl_.UnloadSoundModel(system_handle);
  if (result < 0)
    QMMF_ERROR("%s: %s() impl->UnloadSoundModel failed: %d", TAG, __func__,
               result);

  return result;
}

status_t SystemService::EnableSoundTrigger(const SystemHandle system_handle) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);

  status_t result = system_impl_.EnableSoundTrigger(system_handle);
  if (result < 0)
    QMMF_ERROR("%s: %s() impl->EnableSoundTrigger failed: %d", TAG, __func__,
               result);

  return result;
}

status_t SystemService::DisableSoundTrigger(const SystemHandle system_handle) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);

  status_t result = system_impl_.DisableSoundTrigger(system_handle);
  if (result < 0)
    QMMF_ERROR("%s: %s() impl->DisableSoundTrigger failed: %d", TAG, __func__,
               result);

  return result;
}

status_t SystemService::RegisterForDeviceEvents(
    const SystemHandle system_handle) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);

  status_t result = system_impl_.RegisterForDeviceEvents(system_handle);
  if (result < 0)
    QMMF_ERROR("%s: %s() impl->RegisterForDeviceEvents failed: %d",
               TAG, __func__, result);

  return result;
}

status_t SystemService::QueryDeviceInfo(const SystemHandle system_handle,
                                        vector<DeviceInfo>* devices) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);

  status_t result = system_impl_.QueryDeviceInfo(system_handle, devices);
  if (result < 0)
    QMMF_ERROR("%s: %s() impl->QueryDeviceInfo failed: %d", TAG, __func__,
               result);

  for (const DeviceInfo& device : *devices)
    QMMF_VERBOSE("%s: %s() OUTPARAM: device[%s]", TAG, __func__,
                 device.ToString().c_str());
  return result;
}

status_t SystemService::QueryDeviceCapabilities(
    const SystemHandle system_handle, const DeviceId device, DeviceCaps* caps) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);
  QMMF_VERBOSE("%s: %s() INPARAM: device[%d]", TAG, __func__, device);

  status_t result = system_impl_.QueryDeviceCapabilities(system_handle, device,
                                                         caps);
  if (result < 0)
    QMMF_ERROR("%s: %s() impl->QueryDeviceCapabilities failed: %d",
               TAG, __func__, result);

  QMMF_VERBOSE("%s: %s() OUTPARAM: caps[%s]", TAG, __func__,
               caps->ToString().c_str());
  return result;
}

status_t SystemService::QueryCodecInfo(const SystemHandle system_handle,
                                       vector<CodecInfo>* codecs) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);

  status_t result = system_impl_.QueryCodecInfo(system_handle, codecs);
  if (result < 0)
    QMMF_ERROR("%s: %s() impl->QueryCodecInfo failed: %d", TAG, __func__,
               result);

  for (const CodecInfo& codec : *codecs)
    QMMF_VERBOSE("%s: %s() OUTPARAM: codec[%s]", TAG, __func__,
                 codec.ToString().c_str());
  return result;
}

status_t SystemService::PlayTone(const SystemHandle system_handle,
                                 const vector<DeviceId>& devices,
                                 const Tone& tone) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: system_handle[%d]", TAG, __func__,
               system_handle);
  for (const DeviceId& device : devices)
    QMMF_VERBOSE("%s: %s() INPARAM: device[%d]", TAG, __func__, device);
  QMMF_VERBOSE("%s: %s() INPARAM: tone[%s]", TAG, __func__,
               tone.ToString().c_str());

  status_t result = system_impl_.PlayTone(system_handle, devices, tone);
  if (result < 0)
    QMMF_ERROR("%s: %s() impl->PlayTone failed: %d", TAG, __func__,
               result);

  return result;
}

status_t SystemService::onTransact(uint32_t code, const Parcel& input,
                                   Parcel* output, uint32_t flags) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: code[%u]", TAG, __func__, code);
  QMMF_VERBOSE("%s: %s() INPARAM: flags[%u]", TAG, __func__, flags);

  if (!input.checkInterface(this))
    return -EPERM;

  switch (static_cast<SystemServiceCommand>(code)) {
    case SystemServiceCommand::kSystemConnect: {
      sp<ISystemServiceCallback> client_handler =
          interface_cast<ISystemServiceCallback>(input.readStrongBinder());

      QMMF_DEBUG("%s: %s-SystemConnect() TRACE", TAG, __func__);
      SystemHandle system_handle;
      status_t result = Connect(client_handler, &system_handle);
      QMMF_VERBOSE("%s: %s-SystemConnect() OUTPARAM: system_handle[%d]", TAG,
                   __func__, system_handle);

      output->writeInt32(static_cast<int32_t>(system_handle));
      output->writeInt32(result);
      break;
    }

    case SystemServiceCommand::kSystemDisconnect: {
      SystemHandle system_handle = static_cast<SystemHandle>(input.readInt32());

      QMMF_DEBUG("%s: %s-SystemDisconnect() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-SystemDisconnect() INPARAM: system_handle[%d]", TAG,
                   __func__, system_handle);
      status_t result = Disconnect(system_handle);

      output->writeInt32(result);
      break;
    }

    case SystemServiceCommand::kSystemLoadSoundModel: {
      SystemHandle system_handle = static_cast<SystemHandle>(input.readInt32());
      SoundModelInternal soundmodel;
      Parcel::ReadableBlob blob;
      soundmodel.FromParcel(input, &blob);

      QMMF_DEBUG("%s: %s-SystemLoadSoundModel() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-SystemLoadSoundModel() INPARAM: system_handle[%d]",
                   TAG, __func__, system_handle);
      QMMF_VERBOSE("%s: %s-SystemLoadSoundModel() INPARAM: soundmodel[%s]",
                   TAG, __func__, soundmodel.ToString().c_str());
      status_t result = LoadSoundModel(system_handle, soundmodel);

      blob.release();
      output->writeInt32(result);
      break;
    }

    case SystemServiceCommand::kSystemUnloadSoundModel: {
      SystemHandle system_handle = static_cast<SystemHandle>(input.readInt32());

      QMMF_DEBUG("%s: %s-SystemUnloadSoundModel() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-SystemUnloadSoundModel() INPARAM: system_handle[%d]",
                   TAG, __func__, system_handle);
      status_t result = UnloadSoundModel(system_handle);

      output->writeInt32(result);
      break;
    }

    case SystemServiceCommand::kSystemEnableSoundTrigger: {
      SystemHandle system_handle = static_cast<SystemHandle>(input.readInt32());

      QMMF_DEBUG("%s: %s-SystemEnableSoundTrigger() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-SystemEnableSoundTrigger() INPARAM: system_handle[%d]",
                   TAG, __func__, system_handle);
      status_t result = EnableSoundTrigger(system_handle);

      output->writeInt32(result);
      break;
    }

    case SystemServiceCommand::kSystemDisableSoundTrigger: {
      SystemHandle system_handle = static_cast<SystemHandle>(input.readInt32());

      QMMF_DEBUG("%s: %s-SystemDisableSoundTrigger() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-SystemDisableSoundTrigger() INPARAM: system_handle[%d]",
                   TAG, __func__, system_handle);
      status_t result = DisableSoundTrigger(system_handle);

      output->writeInt32(result);
      break;
    }

    case SystemServiceCommand::kSystemRegisterForDeviceEvents: {
      SystemHandle system_handle = static_cast<SystemHandle>(input.readInt32());

      QMMF_DEBUG("%s: %s-SystemRegisterForDeviceEvents() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-SystemRegisterForDeviceEvents() INPARAM: system_handle[%d]",
                   TAG, __func__, system_handle);
      status_t result = RegisterForDeviceEvents(system_handle);

      output->writeInt32(result);
      break;
    }

    case SystemServiceCommand::kSystemQueryDeviceInfo: {
      SystemHandle system_handle = static_cast<SystemHandle>(input.readInt32());

      QMMF_DEBUG("%s: %s-SystemQueryDeviceInfo() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-SystemQueryDeviceInfo() INPARAM: system_handle[%d]",
                   TAG, __func__, system_handle);
      vector<DeviceInfo> devices;
      status_t result = QueryDeviceInfo(system_handle, &devices);
      for (const DeviceInfo& device : devices)
        QMMF_VERBOSE("%s: %s-SystemQueryDeviceInfo() OUTPARAM: device[%s]",
                     TAG, __func__, device.ToString().c_str());

      output->writeUint32(devices.size());
      for (const DeviceInfo& device : devices)
        DeviceInfoInternal(device).ToParcel(output);
      output->writeInt32(result);
      break;
    }

    case SystemServiceCommand::kSystemQueryDeviceCapabilities: {
      SystemHandle system_handle = static_cast<SystemHandle>(input.readInt32());
      DeviceId device = static_cast<DeviceId>(input.readInt32());

      QMMF_DEBUG("%s: %s-SystemQueryDeviceCapabilities() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-SystemQueryDeviceCapabilities() INPARAM: system_handle[%d]",
                   TAG, __func__, system_handle);
      QMMF_VERBOSE("%s: %s-SystemQueryDeviceCapabilities() INPARAM: device[%d]",
                   TAG, __func__, device);
      DeviceCaps caps;
      status_t result = QueryDeviceCapabilities(system_handle, device, &caps);
      QMMF_VERBOSE("%s: %s-SystemQueryDeviceCapabilities() OUTPARAM: caps[%s]",
                   TAG, __func__, caps.ToString().c_str());

      DeviceCapsInternal(caps).ToParcel(output);
      output->writeInt32(result);
      break;
    }

    case SystemServiceCommand::kSystemQueryCodecInfo: {
      SystemHandle system_handle = static_cast<SystemHandle>(input.readInt32());

      QMMF_DEBUG("%s: %s-SystemQueryCodecInfo() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-SystemQueryCodecInfo() INPARAM: system_handle[%d]",
                   TAG, __func__, system_handle);
      vector<CodecInfo> codecs;
      status_t result = QueryCodecInfo(system_handle, &codecs);
      for (const CodecInfo& codec : codecs)
        QMMF_VERBOSE("%s: %s-SystemQueryCodecInfo() OUTPARAM: codec[%s]",
                     TAG, __func__, codec.ToString().c_str());

      output->writeUint32(codecs.size());
      for (const CodecInfo& codec : codecs)
        CodecInfoInternal(codec).ToParcel(output);
      output->writeInt32(result);
      break;
    }

    case SystemServiceCommand::kSystemPlayTone: {
      SystemHandle system_handle = static_cast<SystemHandle>(input.readInt32());

      vector<DeviceId> devices;
      size_t number_of_devices = static_cast<size_t>(input.readUint32());
      for (size_t index = 0; index < number_of_devices; ++index)
        devices.push_back(input.readInt32());

      ToneInternal tone;
      Parcel::ReadableBlob blob;
      tone.FromParcel(input, &blob);

      QMMF_DEBUG("%s: %s-SystemPlayTone() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s: %s-SystemPlayTone() INPARAM: system_handle[%d]",
                   TAG, __func__, system_handle);
      for (const DeviceId device : devices)
        QMMF_VERBOSE("%s: %s-SystemPlayTone() INPARAM: device[%d]",
                     TAG, __func__, device);
      QMMF_VERBOSE("%s: %s-SystemPlayTone() INPARAM: tone[%s]", TAG, __func__,
                   tone.ToString().c_str());
      status_t result = PlayTone(system_handle, devices, tone);

      blob.release();
      output->writeInt32(result);
      break;
    }

    default:
      QMMF_ERROR("%s: %s() code %u not supported ", TAG, __func__, code);
      output->writeInt32(static_cast<int32_t>(-EINVAL));
      break;
  }
  return 0;
}

}; // namespace system
}; // namespace qmmf
