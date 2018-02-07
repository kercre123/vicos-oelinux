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

#include <utils/KeyedVector.h>

#include "qmmf-sdk/qmmf_display_params.h"
#include "common/qmmf_log.h"
#include "display/src/service/qmmf_display_common.h"
#include "display/src/service/qmmf_remote_cb.h"
#include "display/src/service/qmmf_display_sdm_buffer_allocator.h"
#include "display/src/service/qmmf_display_sdm_buffer_sync_handler.h"
#include "display/src/service/qmmf_display_sdm_debugger.h"
#include "sdm/include/core/core_interface.h"
#include "sdm/include/utils/locker.h"
#include <map>
#include <vector>
#include <hardware/gralloc.h>

namespace qmmf {

namespace display {

using ::sdm::DisplayEventHandler;
using ::sdm::DisplayError;
using ::sdm::LayerRect;
using ::sdm::CoreInterface;
using ::sdm::Layer;
using ::sdm::LayerBuffer;
using ::sdm::DisplayInterface;
using ::sdm::LayerStack;
using ::sdm::DisplayEventVSync;
using ::sdm::Locker;
using ::sdm::BufferInfo;
using ::sdm::LayerBlending;
using ::sdm::LayerBufferFormat;


#define NUM_DISPLAY_ALLOWED 3
#define FLOAT(exp) static_cast<float>(exp)

class DisplayImpl : public DisplayEventHandler
{
 public:

  static DisplayImpl* CreateDisplayCore();

  ~DisplayImpl();

  status_t Connect();

  status_t Disconnect();

  status_t CreateDisplay(sp<RemoteCallBack>& remote_cb,
      DisplayType display_type, DisplayHandle* display_handle);

  status_t DestroyDisplay(DisplayHandle display_handle);

  status_t CreateSurface(DisplayHandle display_handle,
      SurfaceConfig &surface_config, uint32_t* surface_id);

  status_t DestroySurface(DisplayHandle display_handle,
      const uint32_t surface_id);

  status_t DequeueSurfaceBuffer(DisplayHandle display_handle,
      const uint32_t surface_id, SurfaceBuffer &surface_buffer);

  status_t QueueSurfaceBuffer(DisplayHandle display_handle,
      const uint32_t surface_id, SurfaceBuffer &surface_buffer,
      SurfaceParam &surface_param);

  status_t GetDisplayParam(DisplayHandle display_handle,
      DisplayParamType param_type, void *param, size_t param_size);

  status_t SetDisplayParam(DisplayHandle display_handle,
      DisplayParamType param_type, void *param, size_t param_size);

  status_t DequeueWBSurfaceBuffer(DisplayHandle display_handle,
      const uint32_t surface_id, SurfaceBuffer &surface_buffer);

  status_t QueueWBSurfaceBuffer(DisplayHandle display_handle,
      const uint32_t surface_id, const SurfaceBuffer &surface_buffer);

 protected:
  inline void SetRect(const SurfaceRect &source, LayerRect *target);
  Layer* AllocateLayer(DisplayHandle display_handle, uint32_t* surface_id);
  void FreeLayer(DisplayHandle display_handle, const uint32_t surface_id);
  Layer* GetLayer(DisplayHandle display_handle, const uint32_t surface_id);
  LayerStack* GetLayerStack(DisplayHandle display_handle,
      bool queued_buffers_only);

  // DisplayEventHandler methods
  virtual DisplayError VSync(const DisplayEventVSync &vsync);
  virtual DisplayError VSync(int fd, unsigned int sequence,
                             unsigned int tv_sec, unsigned int tv_usec,
                             void *data);
  virtual DisplayError PFlip(int fd, unsigned int sequence,
                             unsigned int tv_sec, unsigned int tv_usec,
                             void *data);
  virtual DisplayError Refresh();
  virtual DisplayError CECMessage(char *message);

 private:

  Mutex                        mLock;
  pthread_mutex_t thread_lock_;
  pthread_t                    pid_;
  Locker     vsync_callback_locker_;

  /**Not allowed */
  DisplayImpl();
  DisplayImpl(const DisplayImpl&);
  DisplayImpl& operator=(const DisplayImpl&);
  bool running_;
  static void* HandleVSync(void *ptr);
  static DisplayImpl* instance_;
  DisplayBufferAllocator buffer_allocator_;
  DisplayBufferSyncHandler buffer_sync_handler_;
  static CoreInterface* core_intf_;
  alloc_device_t *gralloc_device_;

  typedef struct Buff_Info {
    bool  queued;
    bool  dequed;
    bool  commited;
  }Buff_Info;

  typedef struct SurfaceInfo {
    void* mmapbuf;
    Layer* layer;
    std::map<int32_t, BufferInfo*> buffer_info;
    std::map<int32_t, Buff_Info*> buf_id_use;
    bool buffer_internal;
  }SurfaceInfo;

  typedef std::map<uint32_t, SurfaceInfo*> SurfaceinfoMap;
  typedef struct DisplayInfo {
    DisplayType       display_type;
    DisplayInterface* display_intf;
    uint32_t          layer_count;
    uint32_t          num_of_clients;
    sp<RemoteCallBack>           remote_cb_;
    SurfaceinfoMap surfaceinfo_;
  }DisplayInfo;

  DisplayHandle current_handle_;
  bool vsync_state_;
  std::map<DisplayHandle, DisplayInfo*> displayinfo_;
};

}; // namespace display

}; //namespace qmmf
