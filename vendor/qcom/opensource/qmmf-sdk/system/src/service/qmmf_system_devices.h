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

#include <vector>

#include "qmmf-sdk/qmmf_device.h"
#include "qmmf-sdk/qmmf_system_params.h"
#include "system/src/service/qmmf_system_common.h"

namespace qmmf {
namespace system {

class SystemDevices {
 public:
  SystemDevices();
  ~SystemDevices();

  status_t RegisterForDeviceEvents(const SystemHandle system_handle,
                                   const SystemDeviceHandler& handler);
  status_t UnregisterFromDeviceEvents(const SystemHandle system_handle);
  status_t QueryDeviceInfo(const SystemHandle system_handle,
                           ::std::vector<DeviceInfo>* devices);
  status_t QueryDeviceCapabilities(const SystemHandle system_handle,
                                   const DeviceId device,
                                   DeviceCaps* caps);

 private:
  struct SystemDevice {
    DeviceInfo info;
    DeviceCaps caps;
  };

  static SystemDevice hard_wired_devices[];

  DeviceId current_id_;
  SystemDeviceHandler device_handler_;

  ::std::vector<SystemHandle> registered_handles_;
  ::std::vector<SystemDevice> devices_;

  // disable copy, assignment, and move
  SystemDevices(const SystemDevices&) = delete;
  SystemDevices(SystemDevices&&) = delete;
  SystemDevices& operator=(const SystemDevices&) = delete;
  SystemDevices& operator=(const SystemDevices&&) = delete;
};

}; // namespace system
}; // namespace qmmf
