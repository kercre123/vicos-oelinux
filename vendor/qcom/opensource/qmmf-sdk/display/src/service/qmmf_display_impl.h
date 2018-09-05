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
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <set>

#include <hardware/gralloc.h>
#include <utils/KeyedVector.h>
#include <sdm/core/core_interface.h>
#include <sdm/utils/locker.h>

#include "qmmf-sdk/qmmf_display_params.h"
#include "common/utils/qmmf_log.h"
#include "display/src/service/qmmf_display_common.h"
#include "display/src/service/qmmf_remote_cb.h"
#include "display/src/service/qmmf_display_sdm_buffer_allocator.h"
#include "display/src/service/qmmf_display_sdm_buffer_sync_handler.h"
#include "display/src/service/qmmf_display_sdm_debugger.h"

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
  Layer* AllocateLayer(DisplayHandle display_handle, uint32_t* surface_id,
      uint32_t z_order);
  status_t FreeLayer(DisplayHandle display_handle, const uint32_t surface_id);
  Layer* GetLayer(DisplayHandle display_handle, const uint32_t surface_id);
  LayerStack* GetLayerStack(DisplayType display_type,
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

  std::mutex       thread_lock_;
  ::std::thread*   handle_vsync_thread_;
  Locker           vsync_callback_locker_;
  bool             running_;

  /**Not allowed */
  DisplayImpl();
  DisplayImpl(const DisplayImpl&);
  DisplayImpl& operator=(const DisplayImpl&);

  static void HandleVSyncThreadEntry(DisplayImpl* display_impl);
  void HandleVSync();
  static DisplayImpl* instance_;
  DisplayBufferAllocator buffer_allocator_;
  DisplayBufferSyncHandler buffer_sync_handler_;
  static CoreInterface* core_intf_;
  alloc_device_t *gralloc_device_;

  enum class BufferStates {
    kStateFree      = 1, // x1 = 0, x2 = 0, x3 = 0
    kStateDequeued  = 2, // x1 = 1, x2 = 0, x3 = 0
    kStateQueued    = 3, // x1 = 0, x2 = 1, x3 = 0
    kStateCommitted = 4, // x1 = 0, x2 = 1, x3 = 1
    kInvalid        = 0x7FFFFFFF, // (!x1 & x2) | (!x2 & !x3) = 1
  };

  class BufferState {
   public:
    BufferState(const BufferStates state) {
      current_state_ = state;
      dequeued_  = GetDequeuedState(state);
      queued_    = GetQueuedState(state);
      committed_ = GetCommitedState(state);
    }

    inline BufferStates GetState() {return current_state_;}
    BufferStates SetState(const BufferStates state) {
      if (IsStateTransitionValid(current_state_, state)) {
        dequeued_  = GetDequeuedState(state);
        queued_    = GetQueuedState(state);
        committed_ = GetCommitedState(state);
        current_state_ = state;
      }
      return current_state_;
    }
   private:
    bool  dequeued_;   // x1
    bool  queued_;     // x2
    bool  committed_;  // x3
    BufferStates current_state_;
    inline bool GetDequeuedState(const BufferStates state) {
      if (state == BufferStates::kStateDequeued)
        return true;
      else
        return false;
    }

    inline bool GetQueuedState(const BufferStates state) {
      if (state == BufferStates::kStateQueued ||
          state == BufferStates::kStateCommitted)
        return true;
      else
        return false;
    }

    inline bool GetCommitedState(const BufferStates state) {
      if (state == BufferStates::kStateCommitted)
        return true;
      else
        return false;
    }

    bool IsStateTransitionValid(const BufferStates old_state,
                                const BufferStates new_state) {
      if (old_state == BufferStates::kInvalid)
        return false;
      else if (old_state == BufferStates::kStateDequeued
               && new_state == BufferStates::kStateQueued)
        return true;
      else if (old_state == BufferStates::kStateQueued
               && new_state == BufferStates::kStateCommitted)
        return true;
      else if (old_state == BufferStates::kStateCommitted
               && new_state == BufferStates::kStateFree)
        return true;
      else if (old_state == BufferStates::kStateFree
               && new_state == BufferStates::kStateDequeued)
        return true;
      else
        return false;
    }
  };

  typedef struct SurfaceInfo {
    Layer*                           layer;
    // map of buffer id and buffer info
    std::map<int32_t, BufferInfo*>   buffer_info;
    // map of buffer id and buffer state
    std::map<int32_t, BufferState*>  buffer_state;
    bool                             allocate_buffer_mode;
  } SurfaceInfo;

  typedef struct DisplayClientInfo {
    DisplayType                      display_type;
    uint32_t                         num_of_client_layers;
    sp<RemoteCallBack>               remote_cb;
  } DisplayClientInfo;

  typedef struct DisplayTypeInfo {
    std::map<uint32_t, uint32_t>     z_order_surface_id_map;
    uint32_t                         num_of_clients;
    DisplayInterface*                display_intf;
  } DisplayTypeInfo;

  bool                                         vsync_state_;
  DisplayHandle                                current_handle_;
  uint32_t                                     unique_surface_id_;
  std::mutex                                   api_lock_;
  std::mutex                                   layer_lock_;
  std::map<DisplayHandle, DisplayClientInfo*>  display_client_info_map_;
  std::map<DisplayType, DisplayTypeInfo*>      display_type_info_map_;
  std::map<uint32_t, SurfaceInfo*>             surface_info_map_;
};

}; // namespace display

}; //namespace qmmf
