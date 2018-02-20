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

//
// This file has implementation of following classes:
//
// - SystemClient            : Delegation to binder proxy <ISystemService>
//                             and implementation of binder CB.
// - BpSystemService         : Binder proxy implementation.
// - BpSystemServiceCallback : Binder CB proxy implementation.
// - BnSystemServiceCallback : Binder CB stub implementation.
//

#define LOG_TAG "SystemClient"

#include "system/src/client/qmmf_system_client.h"

#include <cerrno>
#include <cstdint>
#include <mutex>
#include <vector>

#include <binder/IBinder.h>
#include <binder/IInterface.h>
#include <binder/IServiceManager.h>
#include <binder/Parcel.h>
#include <binder/ProcessState.h>
#include <utils/RefBase.h>
#include <utils/String16.h>

#include "common/utils/qmmf_codec_internal.h"
#include "common/utils/qmmf_device_internal.h"
#include "common/utils/qmmf_log.h"
#include "qmmf-sdk/qmmf_system_params.h"
#include "system/src/client/qmmf_system_params_internal.h"

uint32_t qmmf_log_level;

namespace qmmf {
namespace system {

using ::android::BpInterface;
using ::android::defaultServiceManager;
using ::android::IBinder;
using ::android::IInterface;
using ::android::interface_cast;
using ::android::IServiceManager;
using ::android::Parcel;
using ::android::ProcessState;
using ::android::sp;
using ::android::String16;
using ::std::lock_guard;
using ::std::mutex;
using ::std::vector;

SystemClient::SystemClient()
    : system_service_(nullptr),
      death_notifier_(nullptr),
      system_handle_(-1) {
  QMMF_GET_LOG_LEVEL();
  QMMF_DEBUG("%s() TRACE", __func__);

  sp<ProcessState> proc(ProcessState::self());
  proc->startThreadPool();

  QMMF_INFO("%s() client instantiated", __func__);
}

SystemClient::~SystemClient() {
  QMMF_DEBUG("%s() TRACE", __func__);

  if (system_service_ != nullptr) {
    system_service_.clear();
    system_service_ = nullptr;
  }

  QMMF_INFO("%s() client destroyed", __func__);
}

status_t SystemClient::Connect(const SystemCb& callback) {
  QMMF_DEBUG("%s() TRACE", __func__);
  lock_guard<mutex> lock(lock_);

  if (system_service_.get() != nullptr) {
    QMMF_ERROR("%s() already connected to service", __func__);
    return -ENOSYS;
  }

  system_callback_ = callback;

  death_notifier_ = new DeathNotifier(this);
  if (death_notifier_.get() == nullptr) {
    QMMF_ERROR("%s() unable to allocate death notifier", __func__);
    return -ENOMEM;
  }

  sp<IBinder> service_handle;
  sp<IServiceManager> service_manager = defaultServiceManager();

  service_handle = service_manager->getService(String16(QMMF_SYSTEM_SERVICE_NAME));
  if (service_handle.get() == nullptr) {
    QMMF_ERROR("%s() can't get service %s", __func__,
               QMMF_SYSTEM_SERVICE_NAME);
    return -ENODEV;
  }

  system_service_ = interface_cast<ISystemService>(service_handle);
  IInterface::asBinder(system_service_)->linkToDeath(death_notifier_);

  sp<ServiceCallbackHandler> cb_handler = new ServiceCallbackHandler(this);
  status_t result = system_service_->Connect(cb_handler, &system_handle_);
  if (result < 0)
    QMMF_ERROR("%s() can't connect to service %s: %d", __func__,
               QMMF_SYSTEM_SERVICE_NAME, result);

  return result;
}

status_t SystemClient::Disconnect() {
  QMMF_DEBUG("%s() TRACE", __func__);
  lock_guard<mutex> lock(lock_);

  if (system_service_.get() == nullptr) {
    QMMF_ERROR("%s() not connected to service", __func__);
    return -ENOSYS;
  }

  status_t result = system_service_->Disconnect(system_handle_);
  if (result < 0)
    QMMF_ERROR("%s() service->Disconnect failed: %d", __func__,
               result);

  system_service_->asBinder(system_service_)->unlinkToDeath(death_notifier_);
  system_service_.clear();
  system_service_ = nullptr;

  death_notifier_.clear();
  death_notifier_ = nullptr;

  system_handle_ = -1;

  return result;
}

status_t SystemClient::LoadSoundModel(const SoundModel& soundmodel) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: soundmodel[%s]", __func__,
               soundmodel.ToString().c_str());
  lock_guard<mutex> lock(lock_);

  if (system_service_.get() == nullptr) {
    QMMF_ERROR("%s() not connected to service", __func__);
    return -ENOSYS;
  }

  status_t result = system_service_->LoadSoundModel(system_handle_, soundmodel);
  if (result < 0)
    QMMF_ERROR("%s() service->LoadSoundModel failed: %d", __func__,
               result);

  return result;
}

status_t SystemClient::UnloadSoundModel() {
  QMMF_DEBUG("%s() TRACE", __func__);
  lock_guard<mutex> lock(lock_);

  if (system_service_.get() == nullptr) {
    QMMF_ERROR("%s() not connected to service", __func__);
    return -ENOSYS;
  }

  status_t result = system_service_->UnloadSoundModel(system_handle_);
  if (result < 0)
    QMMF_ERROR("%s() service->UnloadSoundModel failed: %d", __func__,
               result);

  return result;
}

status_t SystemClient::EnableSoundTrigger(const TriggerConfig& config,
                                          const TriggerCb& callback) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: config[%s]", __func__,
               config.ToString().c_str());
  lock_guard<mutex> lock(lock_);

  if (system_service_.get() == nullptr) {
    QMMF_ERROR("%s() not connected to service", __func__);
    return -ENOSYS;
  }

  trigger_callback_ = callback;

  status_t result = system_service_->EnableSoundTrigger(system_handle_, config);
  if (result < 0)
    QMMF_ERROR("%s() service->EnableSoundTrigger failed: %d", __func__,
               result);

  return result;
}

status_t SystemClient::DisableSoundTrigger() {
  QMMF_DEBUG("%s() TRACE", __func__);
  lock_guard<mutex> lock(lock_);

  if (system_service_.get() == nullptr) {
    QMMF_ERROR("%s() not connected to service", __func__);
    return -ENOSYS;
  }

  status_t result = system_service_->DisableSoundTrigger(system_handle_);
  if (result < 0)
    QMMF_ERROR("%s() service->DisableSoundTrigger failed: %d",
               __func__, result);

  return result;
}

status_t SystemClient::RegisterForDeviceEvents(const DeviceCb& callback) {
  QMMF_DEBUG("%s() TRACE", __func__);
  lock_guard<mutex> lock(lock_);

  if (system_service_.get() == nullptr) {
    QMMF_ERROR("%s() not connected to service", __func__);
    return -ENOSYS;
  }

  device_callback_ = callback;

  status_t result = system_service_->RegisterForDeviceEvents(system_handle_);
  if (result < 0)
    QMMF_ERROR("%s() service->RegisterForDeviceEvents failed: %d",
               __func__, result);

  return result;
}

status_t SystemClient::QueryDeviceInfo(vector<DeviceInfo>* devices) {
  QMMF_DEBUG("%s() TRACE", __func__);
  lock_guard<mutex> lock(lock_);

  if (system_service_.get() == nullptr) {
    QMMF_ERROR("%s() not connected to service", __func__);
    return -ENOSYS;
  }

  status_t result = system_service_->QueryDeviceInfo(system_handle_, devices);
  if (result < 0)
    QMMF_ERROR("%s() service->QueryDeviceInfo failed: %d", __func__,
               result);

  for (const DeviceInfo& device : *devices)
    QMMF_VERBOSE("%s() OUTPARAM: device[%s]", __func__,
                 device.ToString().c_str());
  return result;
}

status_t SystemClient::QueryDeviceCapabilities(const DeviceId device,
                                               DeviceCaps* caps) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: device[%d]", __func__, device);
  lock_guard<mutex> lock(lock_);

  if (system_service_.get() == nullptr) {
    QMMF_ERROR("%s() not connected to service", __func__);
    return -ENOSYS;
  }

  status_t result = system_service_->QueryDeviceCapabilities(system_handle_,
                                                             device, caps);
  if (result < 0)
    QMMF_ERROR("%s() service->QueryDeviceCapabilities failed: %d",
               __func__, result);

  QMMF_VERBOSE("%s() OUTPARAM: caps[%s]", __func__,
               caps->ToString().c_str());
  return result;
}

status_t SystemClient::QueryCodecInfo(vector<CodecInfo>* codecs) {
  QMMF_DEBUG("%s() TRACE", __func__);
  lock_guard<mutex> lock(lock_);

  if (system_service_.get() == nullptr) {
    QMMF_ERROR("%s() not connected to service", __func__);
    return -ENOSYS;
  }

  status_t result = system_service_->QueryCodecInfo(system_handle_, codecs);
  if (result < 0)
    QMMF_ERROR("%s() service->QueryCodecInfo failed: %d", __func__,
               result);

  for (const CodecInfo& codec : *codecs)
    QMMF_VERBOSE("%s() OUTPARAM: codec[%s]", __func__,
                 codec.ToString().c_str());
  return result;
}

status_t SystemClient::PlayTone(const vector<DeviceId>& devices,
                                const Tone& tone,
                                const ToneCb& callback) {
  QMMF_DEBUG("%s() TRACE", __func__);
  for (const DeviceId& device : devices)
    QMMF_VERBOSE("%s() INPARAM: device[%d]", __func__, device);
  QMMF_VERBOSE("%s() INPARAM: tone[%s]", __func__,
               tone.ToString().c_str());
  lock_guard<mutex> lock(lock_);

  if (system_service_.get() == nullptr) {
    QMMF_ERROR("%s() not connected to service", __func__);
    return -ENOSYS;
  }

  tone_callback_ = callback;

  status_t result = system_service_->PlayTone(system_handle_, devices, tone);
  if (result < 0)
    QMMF_ERROR("%s() service->PlayTone failed: %d", __func__, result);

  return result;
}

status_t SystemClient::Mute(const DeviceId device, const bool mute) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: device[%d]", __func__, device);
  QMMF_VERBOSE("%s() INPARAM: mute[%s]", __func__,
               mute ? "true" : "false");
  lock_guard<mutex> lock(lock_);

  if (system_service_.get() == nullptr) {
    QMMF_ERROR("%s() not connected to service", __func__);
    return -ENOSYS;
  }

  status_t result = system_service_->Mute(system_handle_, device, mute);
  if (result < 0)
    QMMF_ERROR("%s() service->Mute failed: %d", __func__, result);

  return result;
}

void SystemClient::NotifySystemEvent(const int32_t error) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: error[%d]", __func__, error);

  system_callback_(error);
}

void SystemClient::NotifyTriggerEvent(const int32_t error,
                                      const BufferDescriptor& buffer) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: error[%d]", __func__, error);
  QMMF_VERBOSE("%s() INPARAM: buffer[%s]", __func__,
               buffer.ToString().c_str());

  if (trigger_callback_)
    trigger_callback_(error, buffer);
  else
    QMMF_ERROR("%s() no callback registered", __func__);
}

void SystemClient::NotifyDeviceEvent(const DeviceInfo& device) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: device[%s]", __func__,
               device.ToString().c_str());

  if (device_callback_)
    device_callback_(device);
  else
    QMMF_ERROR("%s() no callback registered", __func__);
}

void SystemClient::NotifyToneEvent(const int32_t error) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: error[%d]", __func__, error);

  if (tone_callback_)
    tone_callback_(error);
  else
    QMMF_ERROR("%s() no callback registered", __func__);
}

// Binder proxy implementation of ISystemService
class BpSystemService: public BpInterface<ISystemService> {
 public:
  BpSystemService(const sp<IBinder>& impl) : BpInterface<ISystemService>(impl) {}

  status_t Connect(const sp<ISystemServiceCallback>& client_handler,
                   SystemHandle* system_handle) {
    QMMF_DEBUG("%s() TRACE", __func__);
    Parcel input, output;

    // register service callback to get callbacks from system service
    input.writeInterfaceToken(ISystemService::getInterfaceDescriptor());
    input.writeStrongBinder(IInterface::asBinder(client_handler));

    remote()->transact(static_cast<uint32_t>
                                  (SystemServiceCommand::kSystemConnect),
                       input, &output);

    *system_handle = static_cast<SystemHandle>(output.readInt32());
    QMMF_VERBOSE("%s() OUTPARAM: system_handle[%d]", __func__,
                 *system_handle);

    return output.readInt32();
  }

  status_t Disconnect(const SystemHandle system_handle) {
    QMMF_DEBUG("%s() TRACE", __func__);
    QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
                 system_handle);
    Parcel input, output;

    input.writeInterfaceToken(ISystemService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(system_handle));

    remote()->transact(static_cast<uint32_t>
                                  (SystemServiceCommand::kSystemDisconnect),
                       input, &output);

    return output.readInt32();
  }

  status_t LoadSoundModel(const SystemHandle system_handle,
                          const SoundModel& soundmodel) {
    QMMF_DEBUG("%s() TRACE", __func__);
    QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
                 system_handle);
    QMMF_VERBOSE("%s() INPARAM: soundmodel[%s]", __func__,
                 soundmodel.ToString().c_str());
    Parcel input, output;
    Parcel::WritableBlob blob;

    input.writeInterfaceToken(ISystemService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(system_handle));
    SoundModelInternal(soundmodel).ToParcel(&input, &blob);

    remote()->transact(static_cast<uint32_t>
                                  (SystemServiceCommand::kSystemLoadSoundModel),
                       input, &output);

    blob.release();
    return output.readInt32();
  }

  status_t UnloadSoundModel(const SystemHandle system_handle) {
    QMMF_DEBUG("%s() TRACE", __func__);
    QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
                 system_handle);
    Parcel input, output;

    input.writeInterfaceToken(ISystemService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(system_handle));

    remote()->transact(static_cast<uint32_t>
        (SystemServiceCommand::kSystemUnloadSoundModel), input, &output);

    return output.readInt32();
  }

  status_t EnableSoundTrigger(const SystemHandle system_handle,
                              const TriggerConfig& config) {
    QMMF_DEBUG("%s() TRACE", __func__);
    QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
                 system_handle);
    QMMF_VERBOSE("%s() INPARAM: config[%s]", __func__,
                 config.ToString().c_str());
    Parcel input, output;

    input.writeInterfaceToken(ISystemService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(system_handle));
    TriggerConfigInternal(config).ToParcel(&input);

    remote()->transact(static_cast<uint32_t>
        (SystemServiceCommand::kSystemEnableSoundTrigger), input, &output);

    return output.readInt32();
  }

  status_t DisableSoundTrigger(const SystemHandle system_handle) {
    QMMF_DEBUG("%s() TRACE", __func__);
    QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
                 system_handle);
    Parcel input, output;

    input.writeInterfaceToken(ISystemService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(system_handle));

    remote()->transact(static_cast<uint32_t>
        (SystemServiceCommand::kSystemDisableSoundTrigger), input, &output);

    return output.readInt32();
  }

  status_t RegisterForDeviceEvents(const SystemHandle system_handle) {
    QMMF_DEBUG("%s() TRACE", __func__);
    QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
                 system_handle);
    Parcel input, output;

    input.writeInterfaceToken(ISystemService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(system_handle));

    remote()->transact(static_cast<uint32_t>
        (SystemServiceCommand::kSystemRegisterForDeviceEvents), input, &output);

    return output.readInt32();
  }

  status_t QueryDeviceInfo(const SystemHandle system_handle,
                           vector<DeviceInfo>* devices) {
    QMMF_DEBUG("%s() TRACE", __func__);
    QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
                 system_handle);
    Parcel input, output;

    input.writeInterfaceToken(ISystemService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(system_handle));

    remote()->transact(static_cast<uint32_t>
        (SystemServiceCommand::kSystemQueryDeviceInfo), input, &output);

    uint32_t size = output.readUint32();
    for (uint32_t idx = 0; idx < size; ++idx) {
      DeviceInfoInternal device;
      device.FromParcel(output);
      devices->push_back(device);
    }

    for (const DeviceInfo& device : *devices)
      QMMF_VERBOSE("%s() OUTPARAM: device[%s]", __func__,
                   device.ToString().c_str());
    return output.readInt32();
  }

  status_t QueryDeviceCapabilities(const SystemHandle system_handle,
                                   const DeviceId device,
                                   DeviceCaps* caps) {
    QMMF_DEBUG("%s() TRACE", __func__);
    QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
                 system_handle);
    QMMF_VERBOSE("%s() INPARAM: device[%d]", __func__, device);
    Parcel input, output;

    input.writeInterfaceToken(ISystemService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(system_handle));
    input.writeInt32(static_cast<int32_t>(device));

    remote()->transact(static_cast<uint32_t>
        (SystemServiceCommand::kSystemQueryDeviceCapabilities), input, &output);

    DeviceCapsInternal capsInternal;
    capsInternal.FromParcel(output);
    *caps = capsInternal;

    QMMF_VERBOSE("%s() OUTPARAM: caps[%s]", __func__,
                 caps->ToString().c_str());
    return output.readInt32();
  }

  status_t QueryCodecInfo(const SystemHandle system_handle,
                          vector<CodecInfo>* codecs) {
    QMMF_DEBUG("%s() TRACE", __func__);
    QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
                 system_handle);
    Parcel input, output;

    input.writeInterfaceToken(ISystemService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(system_handle));

    remote()->transact(static_cast<uint32_t>
                                  (SystemServiceCommand::kSystemQueryCodecInfo),
                       input, &output);

    uint32_t size = output.readUint32();
    for (uint32_t idx = 0; idx < size; ++idx) {
      CodecInfoInternal codec;
      codec.FromParcel(output);
      codecs->push_back(codec);
    }

    for (const CodecInfo& codec : *codecs)
      QMMF_VERBOSE("%s() OUTPARAM: codec[%s]", __func__,
                   codec.ToString().c_str());
    return output.readInt32();
  }

  status_t PlayTone(const SystemHandle system_handle,
                    const vector<DeviceId>& devices,
                    const Tone& tone) {
    QMMF_DEBUG("%s() TRACE", __func__);
    QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
                 system_handle);
    for (const DeviceId device : devices)
      QMMF_VERBOSE("%s() INPARAM: device[%d]", __func__, device);
    QMMF_VERBOSE("%s() INPARAM: tone[%s]", __func__,
                 tone.ToString().c_str());
    Parcel input, output;
    Parcel::WritableBlob blob;

    input.writeInterfaceToken(ISystemService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(system_handle));
    input.writeUint32(static_cast<uint32_t>(devices.size()));
    for (const DeviceId& device : devices)
      input.writeInt32(static_cast<int32_t>(device));
    ToneInternal(tone).ToParcel(&input, &blob);

    remote()->transact(static_cast<uint32_t>
                                  (SystemServiceCommand::kSystemPlayTone),
                       input, &output);

    blob.release();
    return output.readInt32();
  }

  status_t Mute(const SystemHandle system_handle,
                const DeviceId device,
                const bool mute) {
    QMMF_DEBUG("%s() TRACE", __func__);
    QMMF_VERBOSE("%s() INPARAM: system_handle[%d]", __func__,
                 system_handle);
    QMMF_VERBOSE("%s() INPARAM: device[%d]", __func__, device);
    QMMF_VERBOSE("%s() INPARAM: mute[%s]", __func__,
                 mute ? "true" : "false");
    Parcel input, output;

    input.writeInterfaceToken(ISystemService::getInterfaceDescriptor());
    input.writeInt32(static_cast<int32_t>(system_handle));
    input.writeInt32(static_cast<int32_t>(device));
    input.writeInt32(static_cast<int32_t>(mute));

    remote()->transact(static_cast<uint32_t>(SystemServiceCommand::kSystemMute),
                       input, &output);

    return output.readInt32();
  }
};

IMPLEMENT_META_INTERFACE(SystemService, QMMF_SYSTEM_SERVICE_NAME);

ServiceCallbackHandler::ServiceCallbackHandler(SystemClient* client)
    : client_(client) {
  QMMF_DEBUG("%s() TRACE", __func__);
}

ServiceCallbackHandler::~ServiceCallbackHandler() {
  QMMF_DEBUG("%s() TRACE", __func__);
}

void ServiceCallbackHandler::NotifySystemEvent(const int32_t error) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: error[%d]", __func__, error);

  if (client_ == nullptr)
    QMMF_ERROR("%s() no client to send notification to", __func__);
  else
    client_->NotifySystemEvent(error);
}

void ServiceCallbackHandler::NotifyTriggerEvent(const int32_t error,
                                                const BufferDescriptor& buffer) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: error[%d]", __func__, error);
  QMMF_VERBOSE("%s() INPARAM: buffer[%s]", __func__,
               buffer.ToString().c_str());

  if (client_ == nullptr)
    QMMF_ERROR("%s() no client to send notification to", __func__);
  else
    client_->NotifyTriggerEvent(error, buffer);
}

void ServiceCallbackHandler::NotifyDeviceEvent(const DeviceInfo& device) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: device[%s]", __func__,
               device.ToString().c_str());

  if (client_ == nullptr)
    QMMF_ERROR("%s() no client to send notification to", __func__);
  else
    client_->NotifyDeviceEvent(device);
}

void ServiceCallbackHandler::NotifyToneEvent(const int32_t error) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: error[%d]", __func__, error);

  if (client_ == nullptr)
    QMMF_ERROR("%s() no client to send notification to", __func__);
  else
    client_->NotifyToneEvent(error);
}

class BpSystemServiceCallback: public BpInterface<ISystemServiceCallback> {
 public:
  BpSystemServiceCallback(const sp<IBinder>& impl)
      : BpInterface<ISystemServiceCallback>(impl) {}

  void NotifySystemEvent(const int32_t error) {
    QMMF_DEBUG("%s() TRACE", __func__);
    QMMF_VERBOSE("%s() INPARAM: error[%d]", __func__, error);
    Parcel input, output;

    input.writeInterfaceToken(ISystemServiceCallback::getInterfaceDescriptor());
    input.writeInt32(error);

    remote()->transact(static_cast<uint32_t>
        (SystemServiceCallbackCommand::kSystemNotifySystem), input, &output);
  }

  void NotifyTriggerEvent(const int32_t error,
                          const BufferDescriptor& buffer) {
    QMMF_DEBUG("%s() TRACE", __func__);
    QMMF_VERBOSE("%s() INPARAM: error[%d]", __func__, error);
    QMMF_VERBOSE("%s() INPARAM: buffer[%s]", __func__,
                 buffer.ToString().c_str());
    Parcel input, output;
    Parcel::WritableBlob blob;

    input.writeInterfaceToken(ISystemServiceCallback::getInterfaceDescriptor());
    input.writeInt32(error);

    if (buffer.data == nullptr) {
      input.writeInt32(0);
    } else {
      input.writeInt32(1);
      input.writeUint32(buffer.size);
      input.writeUint32(buffer.capacity);
      input.writeUint64(buffer.timestamp);
      input.writeBlob(buffer.capacity, false, &blob);
      memcpy(blob.data(), buffer.data, buffer.capacity);
    }

    remote()->transact(static_cast<uint32_t>
        (SystemServiceCallbackCommand::kSystemNotifyTrigger), input, &output);

    blob.release();
  }

  void NotifyDeviceEvent(const DeviceInfo& device) {
    QMMF_DEBUG("%s() TRACE", __func__);
    QMMF_VERBOSE("%s() INPARAM: device[%s]", __func__,
                 device.ToString().c_str());
    Parcel input, output;

    input.writeInterfaceToken(ISystemServiceCallback::getInterfaceDescriptor());
    DeviceInfoInternal(device).ToParcel(&input);

    remote()->transact(static_cast<uint32_t>
        (SystemServiceCallbackCommand::kSystemNotifyDevice), input, &output);
  }

  void NotifyToneEvent(const int32_t error) {
    QMMF_DEBUG("%s() TRACE", __func__);
    QMMF_VERBOSE("%s() INPARAM: error[%d]", __func__, error);
    Parcel input, output;

    input.writeInterfaceToken(ISystemServiceCallback::getInterfaceDescriptor());
    input.writeInt32(error);

    remote()->transact(static_cast<uint32_t>
        (SystemServiceCallbackCommand::kSystemNotifyTone), input, &output);
  }
};

IMPLEMENT_META_INTERFACE(SystemServiceCallback,
                         "system.service.ISystemServiceCallback");

int32_t BnSystemServiceCallback::onTransact(uint32_t code, const Parcel& input,
                                            Parcel* output, uint32_t flags) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: code[%u]", __func__, code);
  QMMF_VERBOSE("%s() INPARAM: flags[%u]", __func__, flags);

  if (!input.checkInterface(this))
    return -EPERM;

  switch (static_cast<SystemServiceCallbackCommand>(code)) {
    case SystemServiceCallbackCommand::kSystemNotifySystem: {
      int32_t error = input.readInt32();

      QMMF_DEBUG("%s-SystemNotifyError() TRACE", __func__);
      QMMF_VERBOSE("%s-SystemNotifyError() INPARAM: error[%d]",
                   __func__, error);
      NotifySystemEvent(error);
      break;
    }

    case SystemServiceCallbackCommand::kSystemNotifyTrigger: {
      int32_t error = input.readInt32();

      Parcel::ReadableBlob blob;
      BufferDescriptor buffer = { nullptr, -1, 0, 0, 0, 0, 0, 0 };
      int32_t buffer_present = input.readInt32();

      if (buffer_present) {
        buffer.size = input.readUint32();
        buffer.capacity = input.readUint32();
        buffer.timestamp = input.readUint64();
        input.readBlob(buffer.capacity, &blob);
        buffer.data = const_cast<void*>(blob.data());
      }

      QMMF_DEBUG("%s-SystemNotifyTrigger() TRACE", __func__);
      QMMF_VERBOSE("%s-SystemNotifyTrigger() INPARAM: error[%d]",
                   __func__, error);
      QMMF_VERBOSE("%s()-SystemNotifyTrigger() INPARAM: buffer[%s]",
                   __func__, buffer.ToString().c_str());
      NotifyTriggerEvent(error, buffer);

      blob.release();
      break;
    }

    case SystemServiceCallbackCommand::kSystemNotifyDevice: {
      DeviceInfoInternal device;
      device.FromParcel(input);

      QMMF_DEBUG("%s-SystemNotifyDevice() TRACE", __func__);
      QMMF_VERBOSE("%s-SystemNotifyDevice() INPARAM: device[%s]",
                   __func__, device.ToString().c_str());
      NotifyDeviceEvent(device);
      break;
    }

    case SystemServiceCallbackCommand::kSystemNotifyTone: {
      int32_t error = input.readInt32();

      QMMF_DEBUG("%s-SystemNotifyTone() TRACE", __func__);
      QMMF_VERBOSE("%s-SystemNotifyTone() INPARAM: error[%d]",
                   __func__, error);
      NotifyToneEvent(error);
      break;
    }

    default:
      QMMF_ERROR("%s() code %u not supported ", __func__, code);
      break;
  }
  return 0;
}

}; // namespace system
}; // namespace qmmf
