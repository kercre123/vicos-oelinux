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

#define TAG "System"

#include "qmmf-sdk/qmmf_system.h"

#include <vector>

#include "qmmf-sdk/qmmf_system_params.h"
#include "system/src/client/qmmf_system_client.h"
#include "common/qmmf_log.h"

namespace qmmf {
namespace system {

using ::std::vector;

System::System()
    : system_client_(nullptr) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_INFO("%s: %s() system instantiated", TAG, __func__);
}

System::~System() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  if (system_client_ != nullptr) {
    delete system_client_;
    system_client_ = nullptr;
  }

  QMMF_INFO("%s: %s() system destroyed", TAG, __func__);
}

status_t System::Connect(const SystemCb& callback) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  system_client_ = new SystemClient();
  if (system_client_ == nullptr)
    return -ENOMEM;

  int32_t result = system_client_->Connect(callback);
  if (result < 0)
    QMMF_ERROR("%s: %s() client->Connect failed: %d", TAG, __func__, result);

  return result;
}

status_t System::Disconnect() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  if (system_client_ == nullptr) {
    QMMF_WARN("%s: %s() system already disconnected", TAG, __func__);
    return 0;
  }

  status_t result = system_client_->Disconnect();
  if (result < 0)
    QMMF_ERROR("%s: %s() client->Disconnect failed: %d", TAG, __func__, result);

  delete system_client_;
  system_client_ = nullptr;

  return result;
}

status_t System::LoadSoundModel(const SoundModel& soundmodel) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: soundmodel[%s]", TAG, __func__,
               soundmodel.ToString().c_str());
  assert(system_client_ != nullptr);
  assert(soundmodel.data != nullptr);

  status_t result = system_client_->LoadSoundModel(soundmodel);
  if (result < 0)
    QMMF_ERROR("%s: %s() client->LoadSoundModel failed: %d", TAG, __func__,
               result);

  return result;
}

status_t System::UnloadSoundModel() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  assert(system_client_ != nullptr);

  status_t result = system_client_->UnloadSoundModel();
  if (result < 0)
    QMMF_ERROR("%s: %s() client->UnloadSoundModel failed: %d", TAG, __func__,
               result);

  return result;
}

status_t System::EnableSoundTrigger(const TriggerCb& callback) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  assert(system_client_ != nullptr);

  status_t result = system_client_->EnableSoundTrigger(callback);
  if (result < 0)
    QMMF_ERROR("%s: %s() client->EnableSoundTrigger failed: %d", TAG, __func__,
               result);

  return result;
}

status_t System::DisableSoundTrigger() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  assert(system_client_ != nullptr);

  status_t result = system_client_->DisableSoundTrigger();
  if (result < 0)
    QMMF_ERROR("%s: %s() client->DisableSoundTrigger failed: %d", TAG, __func__,
               result);

  return result;
}

status_t System::RegisterForDeviceEvents(const DeviceCb& callback) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  assert(system_client_ != nullptr);

  status_t result = system_client_->RegisterForDeviceEvents(callback);
  if (result < 0)
    QMMF_ERROR("%s: %s() client->RegisterForDeviceEvents failed: %d",
               TAG, __func__, result);

  return result;
}

status_t System::QueryDeviceInfo(vector<DeviceInfo>* devices) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  assert(system_client_ != nullptr);
  assert(devices != nullptr);

  status_t result = system_client_->QueryDeviceInfo(devices);
  if (result < 0)
    QMMF_ERROR("%s: %s() client->QueryDeviceInfo failed: %d", TAG, __func__,
               result);

  for (const DeviceInfo& device : *devices)
    QMMF_VERBOSE("%s: %s() OUTPARAM: device[%s]", TAG, __func__,
                 device.ToString().c_str());
  return result;
}

status_t System::QueryDeviceCapabilities(const DeviceId device,
                                         DeviceCaps* caps) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: device[%d]", TAG, __func__, device);
  assert(system_client_ != nullptr);
  assert(caps != nullptr);

  status_t result = system_client_->QueryDeviceCapabilities(device, caps);
  if (result < 0)
    QMMF_ERROR("%s: %s() client->QueryDeviceCapabilities failed: %d",
               TAG, __func__, result);

  QMMF_VERBOSE("%s: %s() OUTPARAM: caps[%s]", TAG, __func__,
               caps->ToString().c_str());
  return result;
}

status_t System::QueryCodecInfo(vector<CodecInfo>* codecs) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  assert(system_client_ != nullptr);
  assert(codecs != nullptr);

  status_t result = system_client_->QueryCodecInfo(codecs);
  if (result < 0)
    QMMF_ERROR("%s: %s() client->QueryCodecInfo failed: %d", TAG, __func__,
               result);

  for (const CodecInfo& codec : *codecs)
    QMMF_VERBOSE("%s: %s() OUTPARAM: codec[%s]", TAG, __func__,
                 codec.ToString().c_str());
  return result;
}

status_t System::PlayTone(const vector<DeviceId>& devices,
                          const Tone& tone,
                          const ToneCb& callback) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  for (const DeviceId& device : devices)
    QMMF_VERBOSE("%s: %s() INPARAM: device[%d]", TAG, __func__, device);
  QMMF_VERBOSE("%s: %s() INPARAM: tone[%s]", TAG, __func__,
               tone.ToString().c_str());
  assert(system_client_ != nullptr);
  assert(tone.buffer != nullptr);
  assert(tone.size <= 192000);

  status_t result = system_client_->PlayTone(devices, tone, callback);
  if (result < 0)
    QMMF_ERROR("%s: %s() client->PlayTone failed: %d", TAG, __func__, result);

  return result;
}

}; // namespace system
}; // namespace qmmf
