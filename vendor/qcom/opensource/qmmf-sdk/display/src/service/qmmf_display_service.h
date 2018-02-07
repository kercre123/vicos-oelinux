/*
* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#include <map>

#include "display/src/client/qmmf_display_service_intf.h"
#include "display/src/service/qmmf_display_common.h"
#include "display/src/service/qmmf_display_impl.h"

namespace qmmf {

namespace display {

using namespace android;

class DisplayService : public BnInterface<IDisplayService> {
 public:
  DisplayService();

  ~DisplayService();

 private:

  class DeathNotifier : public IBinder::DeathRecipient {
  public:
    DeathNotifier(sp<DisplayService> parent) :
        parent_(parent) {}

    void binderDied(const wp<IBinder>&) override {
      QMMF_WARN("DisplaySerive:%s: Client Exited or died!", __func__);
      assert(parent_.get() != nullptr);
      parent_->Disconnect();
    }
  sp<DisplayService> parent_;
  DisplayHandle display_handle_;
  };

  friend class DeathNotifier;

  // Method of BnInterface<IDisplayService>.
  // This method would get call to handle incoming messages from clients.
  status_t onTransact(uint32_t code, const Parcel& data,
      Parcel* reply, uint32_t flags = 0) override;

  status_t Connect() override;

  status_t Disconnect() override;

  status_t CreateDisplay(const sp<IDisplayServiceCallback>&
    service_cb, DisplayType display_type, DisplayHandle* display_handle)
    override;

  status_t DestroyDisplay(DisplayHandle display_handle) override;

  status_t CreateSurface(DisplayHandle display_handle,
      SurfaceConfig &surface_config, uint32_t* surface_id) override;

  status_t DestroySurface(DisplayHandle display_handle,
      const uint32_t surface_id) override;

  status_t DequeueSurfaceBuffer(DisplayHandle display_handle,
      const uint32_t surface_id, SurfaceBuffer &surface_buffer) override;

  status_t QueueSurfaceBuffer(DisplayHandle display_handle,
      const uint32_t surface_id, SurfaceBuffer &surface_buffer,
      SurfaceParam &surface_param) override;

  status_t GetDisplayParam(DisplayHandle display_handle,
      DisplayParamType param_type, void *param, size_t param_size) override;

  status_t SetDisplayParam(DisplayHandle display_handle,
      DisplayParamType param_type, void *param, size_t param_size) override;

  status_t DequeueWBSurfaceBuffer(DisplayHandle display_handle,
      const uint32_t surface_id, SurfaceBuffer &surface_buffer) override;

  status_t QueueWBSurfaceBuffer(DisplayHandle display_handle,
      const uint32_t surface_id, const SurfaceBuffer &surface_buffer)
      override;


  DisplayImpl*                 display_;
  std::map<DisplayHandle, sp<RemoteCallBack>> remote_callback_;
  sp<DeathNotifier> death_notifier_;
  std::map<DisplayHandle, sp<IDisplayServiceCallback>> client_handlers_;
  bool                         connected_;
  ion_fd_map ion_fd_mapping;
  use_buffer_map use_buffer_mapping;
  int32_t              ion_device_;
};

}; //namespace display

}; //namespace qmmf
