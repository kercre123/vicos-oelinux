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

#include <utils/Errors.h>
#include <utils/Mutex.h>
#include <utils/RefBase.h>
#include <utils/KeyedVector.h>
#include <map>

#include "qmmf-sdk/qmmf_display_params.h"
#include "display/src/client/qmmf_display_service_intf.h"

namespace qmmf {

namespace display {

class DisplayClient
{
public:
  DisplayClient();

  ~DisplayClient();

  status_t Connect();

  status_t Disconnect();

  status_t CreateDisplay(DisplayType type, DisplayCb& cb);

  status_t DestroyDisplay(DisplayType type);

  status_t CreateSurface(SurfaceConfig &surface_config,
      uint32_t* surface_id);

  status_t DestroySurface(const uint32_t surface_id);

  status_t DequeueSurfaceBuffer(const uint32_t surface_id,
      SurfaceBuffer &surface_buffer);

  status_t QueueSurfaceBuffer(const uint32_t surface_id,
      SurfaceBuffer &surface_buffer, SurfaceParam &surface_param);

  status_t GetDisplayParam(DisplayParamType param_type, void *param,
      size_t param_size);

  status_t SetDisplayParam(DisplayParamType param_type, void *param,
      size_t param_size);

  status_t DequeueWBSurfaceBuffer(const uint32_t surface_id,
      SurfaceBuffer &surface_buffer);

  status_t QueueWBSurfaceBuffer(const uint32_t surface_id,
      const SurfaceBuffer &surface_buffer);

  //Callbacks from service.
  void notifyDisplayEvent(DisplayEventType event_type, void *event_data,
      size_t event_data_size);

  void notifySessionEvent(DisplayEventType event_type, void *event_data,
      size_t event_data_size);

  //VSync Callback from service.
  void notifyVSyncEvent(int64_t time_stamp);

 private:

  bool checkServiceStatus();

  class DeathNotifier : public IBinder::DeathRecipient
  {
   public:
    DeathNotifier(DisplayClient* parent) : parent_(parent) {}

    void binderDied(const wp<IBinder>&) override {
        ALOGD("DisplayClient:%s: Display service died", __func__);

          Mutex::Autolock l(parent_->lock_);
          parent_->display_service_.clear();
          parent_->display_service_ = NULL;
    }
    DisplayClient* parent_;
  };
  friend class DeathNotifier;

  Mutex                lock_;
  sp<IDisplayService>  display_service_;
  sp<DeathNotifier>    death_notifier_;
  DisplayCb            display_cb_;
  int32_t              ion_device_;
  DisplayHandle        display_handle_;
  DisplayType          display_type_;
  // List of session callbacks.
  DefaultKeyedVector<uint32_t, DisplaySessionCb > session_cb_list_;

  typedef struct BufInfo {
    // Transferred ION Id.
    int32_t ion_fd;
    // Memory mapped buffer.
    void    *pointer;
    // Size
    size_t  frame_len;
  } BufInfo;

  // map <buffer index, buffer_info>
  typedef std::map<int32_t, BufInfo*> buf_info_map;
  buf_info_map buf_info_map_;

  // map <session id, vector<track id> >
  DefaultKeyedVector<uint32_t, Vector<uint32_t> >  sessions_;

  bool context_;
};

class ServiceCallbackHandler : public BnDisplayServiceCallback {
 public:

  ServiceCallbackHandler(DisplayClient* client);

  ~ServiceCallbackHandler();

 private:
  //Methods of BnDisplayServiceCallback.
  void notifyDisplayEvent(DisplayEventType event_type, void *event_data,
      size_t event_data_size) override;

  void notifySessionEvent(DisplayEventType event_type, void *event_data,
      size_t event_data_size) override;

  void notifyVSyncEvent(int64_t time_stamp) override;

  DisplayClient *client_;
};


}; // namespace qmmf

}; // namespace display.
