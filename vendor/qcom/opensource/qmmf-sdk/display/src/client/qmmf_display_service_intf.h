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

#include <binder/IBinder.h>
#include <binder/IServiceManager.h>
#include <binder/Parcel.h>
#include <map>

#include "qmmf-sdk/qmmf_display_params.h"
#include "display/src/service/qmmf_display_common.h"

namespace qmmf {

namespace display {

using namespace android;

#define QMMF_DISPLAY_SERVICE_NAME "qmmf_display.service"

enum QMMF_DISPLAY_SERVICE_CMDS {
  DISPLAY_CONNECT = IBinder::FIRST_CALL_TRANSACTION,
  DISPLAY_DISCONNECT,
  DISPLAY_CREATE_DISPLAY,
  DISPLAY_DESTROY_DISPLAY,
  DISPLAY_CREATE_SURFACE,
  DISPLAY_DESTROY_SURFACE,
  DISPLAY_DEQUEUE_SURFACE_BUFFER,
  DISPLAY_QUEUE_SURFACE_BUFFER,
  DISPLAY_GET_DISPLAY_PARAM,
  DISPLAY_SET_DISPLAY_PARAM,
  DISPLAY_DEQUEUE_WBSURFACE_BUFFER,
  DISPLAY_QUEUE_WBSURFACE_BUFFER,
};

//mapping of service ion_fd and client ion_fd
typedef std::map<int32_t, int32_t> ion_fd_map;

//mapping of surface id and use_buffer
typedef std::map<uint32_t, bool> use_buffer_map;

class IDisplayServiceCallback;
class IDisplayService : public IInterface {
public:
  DECLARE_META_INTERFACE(DisplayService);
  virtual status_t Connect() = 0;

  virtual status_t Disconnect() = 0;

  virtual status_t CreateDisplay(const sp<IDisplayServiceCallback>&service_cb,
      DisplayType type, DisplayHandle* display_handle) = 0;

  virtual status_t DestroyDisplay(DisplayHandle display_handle) = 0;

  virtual status_t CreateSurface(DisplayHandle display_handle,
      SurfaceConfig &surface_config, uint32_t* surface_id) = 0;

  virtual status_t DestroySurface(DisplayHandle display_handle,
      const uint32_t surface_id) = 0;

  virtual status_t DequeueSurfaceBuffer(DisplayHandle display_handle,
      const uint32_t surface_id, SurfaceBuffer &surface_buffer) = 0;

  virtual status_t QueueSurfaceBuffer(DisplayHandle display_handle,
      const uint32_t surface_id, SurfaceBuffer &surface_buffer,
      SurfaceParam &surface_param) = 0;

  virtual status_t GetDisplayParam(DisplayHandle display_handle,
      DisplayParamType param_type, void *param, size_t param_size) = 0;

  virtual status_t SetDisplayParam(DisplayHandle display_handle,
      DisplayParamType param_type, void *param, size_t param_size) = 0;

  virtual status_t DequeueWBSurfaceBuffer(DisplayHandle display_handle,
      const uint32_t surface_id, SurfaceBuffer &surface_buffer) = 0;
  virtual status_t QueueWBSurfaceBuffer(DisplayHandle display_handle,
      const uint32_t surface_id, const SurfaceBuffer &surface_buffer) = 0;

};

enum DISPLAY_SERVICE_CB_CMDS{
    DISPLAY_NOTIFY_EVENT=IBinder::FIRST_CALL_TRANSACTION,
    DISPLAY_NOTIFY_SESSION_EVENT,
    DISPLAY_NOTIFY_VSYNC_EVENT,
};

//Binder interface for callbacks from DisplayService to DisplayClient.
class IDisplayServiceCallback : public IInterface {
public:
  DECLARE_META_INTERFACE(DisplayServiceCallback);

  virtual void notifyDisplayEvent(DisplayEventType event_type, void *event_data,
      size_t event_data_size) = 0;

  virtual void notifySessionEvent(DisplayEventType event_type, void *event_data,
      size_t event_data_size) = 0;

  virtual void notifyVSyncEvent(int64_t time_stamp) = 0;

};

//This class is responsible to provide callbacks from display service.
class BnDisplayServiceCallback : public BnInterface<IDisplayServiceCallback>
{
public:
  virtual status_t onTransact(uint32_t code, const Parcel& data,
      Parcel* reply, uint32_t flags = 0);
};

}; //namespace display

}; //namespace qmmf
