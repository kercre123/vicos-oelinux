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

#define TAG "DisplayImpl"

#include "display/src/service/qmmf_display_impl.h"
#include "sdm/include/core/dump_interface.h"
#include "display/src/service/qmmf_display_sdm_buffer_sync_handler.h"
#include <dlfcn.h>
#include <utils/KeyedVector.h>
#include <utils/List.h>
#include <utils/RefBase.h>
#include <hardware/hardware.h>

namespace qmmf {

namespace display {

DisplayImpl* DisplayImpl::instance_ = nullptr;
CoreInterface* DisplayImpl::core_intf_ = nullptr;

DisplayImpl* DisplayImpl::CreateDisplayCore() {
    QMMF_INFO("%s:%s: Enter", TAG, __func__);

  if(!instance_) {

    int32_t res;
    void *handle;

    instance_ = new DisplayImpl;
    if(!instance_) {
      QMMF_ERROR("%s:%s: Can't Create Display Instance!", TAG, __func__);
      return NULL;
    }

    struct hw_module_t *hmi;
    handle = dlopen(GRALLOC_MODULE_PATH, RTLD_NOW);
    if (handle == NULL) {
      char const *err_str = dlerror();
      QMMF_ERROR("load: module=%s\n%s \n", GRALLOC_MODULE_PATH,
          err_str ? err_str : "unknown");
      res = -EINVAL;
    }

    hmi = (struct hw_module_t *)dlsym(handle, HAL_MODULE_INFO_SYM_AS_STR);
    if (hmi == NULL) {
      QMMF_ERROR("load: couldn't find symbol %s\n", HAL_MODULE_INFO_SYM_AS_STR);
      res = -EINVAL;
    }

    if (strcmp(GRALLOC_HARDWARE_MODULE_ID, hmi->id) != 0) {
      QMMF_ERROR("load: id=%s != hmi->id=%s\n", GRALLOC_HARDWARE_MODULE_ID,
          hmi->id);
      res = -EINVAL;
    }

    hmi->dso = handle;
    res = 0;

    hmi->methods->open(hmi, GRALLOC_HARDWARE_GPU0,
                          (struct hw_device_t **)&instance_->gralloc_device_);
    if (0 != res) {
      QMMF_ERROR("%s: Could not open Gralloc module: %s (%d) \n", __func__,
                 strerror(-res), res);
    }

    QMMF_INFO("%s: Gralloc Module author: %s, version: %d name: %s\n", __func__,
        instance_->gralloc_device_->common.module->author,
        instance_->gralloc_device_->common.module->hal_api_version,
        instance_->gralloc_device_->common.module->name);

  }

  QMMF_INFO("%s:%s: Display Instance Created Successfully(0x%p)", TAG,
      __func__, instance_);
  return instance_;
}
DisplayImpl::DisplayImpl()
  : current_handle_(0),
    vsync_state_(false) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  if(!core_intf_) {
    DisplayError error = CoreInterface::CreateCore(DisplayDebugHandler::Get(),
        &buffer_allocator_, &buffer_sync_handler_, &core_intf_);
    if (!core_intf_) {
      QMMF_ERROR("%s:%s: Display Core Initialization Failed. Error = %d",
          TAG, __func__, error);
    }
    QMMF_INFO("%s:%s: Display Core Initialized Successfully!", TAG, __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

DisplayImpl::~DisplayImpl() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  if (displayinfo_.size() != 0)
    return ;

  DisplayError error = CoreInterface::DestroyCore();
  if (error != kErrorNone) {
    QMMF_ERROR("%s: %s() Display core de-initialization failed. Error = %d",
        TAG, __func__, error);
  }
  instance_->displayinfo_.clear();
  instance_ = nullptr;
  core_intf_ = nullptr;
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

status_t DisplayImpl::Connect() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  int32_t ret = NO_ERROR;

  pthread_mutex_init(&thread_lock_, NULL);

  ret = pthread_create(&pid_, NULL, HandleVSync, this);
  if (0 != ret) {
    QMMF_ERROR("%s: Unable to create HandleVSync thread\n", __func__);
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t DisplayImpl::Disconnect() {

  int32_t ret = NO_ERROR;

  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  if (0 != pid_) {
    pthread_join(pid_, NULL);

    pthread_mutex_lock(&thread_lock_);
    pid_ = 0;
    pthread_mutex_unlock(&thread_lock_);
    pthread_mutex_destroy(&thread_lock_);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t DisplayImpl::CreateDisplay(sp<RemoteCallBack>& remote_cb,
    DisplayType display_type, DisplayHandle* display_handle) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  int32_t ret = NO_ERROR;

  pthread_mutex_lock(&thread_lock_);
  assert(remote_cb.get() != nullptr);

  // check if the display type has already been created.If yes,
  // provide a new handle mapped to the existing display.
  for (std::map<DisplayHandle, DisplayInfo*>::iterator it=displayinfo_.begin();
      it != displayinfo_.end(); ++it) {
    if (it->second->display_type == display_type)
      {
        *display_handle = ++current_handle_;
        QMMF_INFO("%s:%s: Display handle already created:%d",
            TAG, __func__, *display_handle);
        it->second->num_of_clients++;
        displayinfo_.insert({*display_handle, it->second});
        pthread_mutex_unlock(&thread_lock_);
        return ret;
      }
  }
  // Create a new handle value.
  if (current_handle_ + 1 > NUM_DISPLAY_ALLOWED)
    current_handle_ = 0;
  ++current_handle_;
  while (displayinfo_.find(current_handle_) != displayinfo_.end())
    ++current_handle_;

  displayinfo_.insert({current_handle_, nullptr});

  *display_handle = current_handle_;
  vsync_state_= true;
  auto displayinfo = displayinfo_.find(*display_handle);

  displayinfo->second = new DisplayInfo();
  assert(displayinfo->second != NULL);
  DisplayInfo* display_info = displayinfo->second;

  DisplayInterface* displayintf = nullptr;
  DisplayError error = core_intf_->CreateDisplay((sdm::DisplayType)display_type,
      (DisplayEventHandler*)this, &displayintf);
  if (error != kErrorNone) {
    QMMF_ERROR("%s:%s: Display Create Failed. Error = %d", TAG, __func__,
        error);
    pthread_mutex_unlock(&thread_lock_);
    return error;
  }

  assert(displayintf != NULL);
  display_info->display_intf = displayintf;
  display_info->display_type = display_type;
  display_info->num_of_clients = 1;

  assert(remote_cb.get() != nullptr);
  display_info->remote_cb_ = remote_cb;

  running_ = 1;

  error = displayintf->SetCompositionState((sdm::LayerComposition)
      kCompositionGPU, false);
  if (error != kErrorNone) {
    QMMF_ERROR("%s:%s: SetCompositionState Failed. Error = %d", TAG, __func__,
        error);
  }

  error = displayintf->SetDisplayState(kStateOn);
  if (error != kErrorNone) {
    QMMF_ERROR("%s:%s: SetDisplayState Failed. Error = %d", TAG, __func__,
        error);
  }

  displayintf->SetIdleTimeoutMs(0);
  error = displayintf->SetVSyncState(true);
  if (error != kErrorNone) {
    QMMF_ERROR("%s:%s: SetVSyncState Failed. Error = %d", TAG, __func__, error);
  }
  QMMF_INFO("%s:%s: Display Created Successfully! *display_handle:%d",
      TAG, __func__, *display_handle);

  pthread_mutex_unlock(&thread_lock_);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t DisplayImpl::DestroyDisplay(DisplayHandle display_handle) {

  int32_t ret = NO_ERROR;

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  pthread_mutex_lock(&thread_lock_);

  auto displayinfo = displayinfo_.find(display_handle);
  if (displayinfo == displayinfo_.end()) {
    QMMF_ERROR("%s: %s() no displayinfo %u", TAG, __func__, display_handle);
    pthread_mutex_unlock(&thread_lock_);
    return -EINVAL;
  }

  if(displayinfo->second->num_of_clients>1) {
    QMMF_ERROR("%s: %s() Cant destroy.There are more users of this display. %u",
        TAG, __func__, display_handle);
    displayinfo->second->num_of_clients--;
    displayinfo_.erase(displayinfo);
    pthread_mutex_unlock(&thread_lock_);
    return ret;
  }

  if (0 != pid_) {
    running_ = 0;
  }

  for (SurfaceinfoMap::iterator it=displayinfo->second->surfaceinfo_.begin();
      it != displayinfo->second->surfaceinfo_.end(); ++it) {
    status_t error = DestroySurface(display_handle,it->first);
    if (error != kErrorNone){
      QMMF_ERROR("%s: %s() no displayinfo %u", TAG, __func__, display_handle);
      pthread_mutex_unlock(&thread_lock_);
      return -EINVAL;
    }
  }

  DisplayInterface* displayintf = displayinfo->second->display_intf;
  assert(displayintf != NULL);

  DisplayError error = displayintf->SetVSyncState(false);
  if (error != kErrorNone) {
    QMMF_ERROR("%s:%s: SetVSyncState Failed. Error = %d", TAG, __func__,
        error);
  }

  error = displayintf->SetDisplayState(kStateOff);
  if (error != kErrorNone) {
    QMMF_ERROR("%s:%s: SetDisplayState Failed. Error = %d", TAG, __func__,
        error);
  }

  if(displayintf && core_intf_) {
    DisplayError error = core_intf_->DestroyDisplay(displayintf);
    if (error != kErrorNone) {
      QMMF_ERROR("%s:%s: Display Disconnet Failed. Error = %d", TAG, __func__,
          error);
      pthread_mutex_unlock(&thread_lock_);
      return error;
    }

    displayintf = nullptr;
    displayinfo->second->surfaceinfo_.clear();
    displayinfo->second->num_of_clients = 0;
    delete displayinfo->second;
    displayinfo->second = nullptr;
    displayinfo_.erase(displayinfo);
  }
  pthread_mutex_unlock(&thread_lock_);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t DisplayImpl::CreateSurface(DisplayHandle display_handle,
    SurfaceConfig &surface_config, uint32_t* surface_id) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  pthread_mutex_lock(&thread_lock_);

  int32_t ret = NO_ERROR;
  SurfaceParam surface_param;
  DisplayError error;

  surface_param.src_rect = { 0.0, 0.0, (float)surface_config.width,
      (float)surface_config.height };
  surface_param.dst_rect = { 0.0, 0.0, (float)surface_config.width,
      (float)surface_config.height };
  surface_param.surface_blending = SurfaceBlending::kBlendingCoverage;
  surface_param.surface_flags.cursor=0;
  surface_param.frame_rate=30;
  surface_param.z_order = 0;
  surface_param.solid_fill_color=0;

  auto displayinfo = displayinfo_.find(display_handle);
  if (displayinfo == displayinfo_.end()) {
    QMMF_ERROR("%s: %s() no displayinfo %u", TAG, __func__, display_handle);
    pthread_mutex_unlock(&thread_lock_);
    return -EINVAL;
  }
  DisplayInterface* displayintf = displayinfo->second->display_intf;
  assert(displayintf != NULL);

  SurfaceInfo* surfaceinfo = new SurfaceInfo();
  assert(surfaceinfo != NULL);

  Layer* layer = AllocateLayer(display_handle, surface_id);
  assert(layer != NULL);

  surfaceinfo->layer=layer;
  displayinfo->second->surfaceinfo_.insert({*surface_id, surfaceinfo});

  assert(layer != NULL);

  BufferInfo buffer_info;
  int32_t aligned_width, aligned_height;

  aligned_width = surface_config.width;
  aligned_height = surface_config.height;
  buffer_info.buffer_config.width = surface_config.width;
  buffer_info.buffer_config.height = surface_config.height;
  buffer_info.buffer_config.format = (LayerBufferFormat)surface_config.format;
  buffer_info.buffer_config.buffer_count = 1;
  buffer_info.buffer_config.cache = surface_config.cache;
  buffer_info.alloc_buffer_info.fd = -1;
  buffer_info.alloc_buffer_info.stride = 0;
  buffer_info.alloc_buffer_info.size = 0;
  error = buffer_allocator_.GetBufferInfo(&buffer_info, aligned_width, aligned_height);
  if (error != kErrorNone) {
    QMMF_ERROR("%s:%s: GetBufferInfo Failed. Error = %d", TAG,
        __func__, error);
  }

  layer->input_buffer.width = aligned_width;
  layer->input_buffer.height = aligned_height;
  layer->input_buffer.unaligned_width = surface_config.width;
  layer->input_buffer.unaligned_height = surface_config.height;
  layer->input_buffer.format = (LayerBufferFormat)surface_config.format;
  SetRect(surface_param.dst_rect, &layer->dst_rect);
  SetRect(surface_param.src_rect, &layer->src_rect);
  layer->frame_rate = surface_param.frame_rate;
  layer->solid_fill_color = surface_param.solid_fill_color;
  layer->flags.solid_fill = 0;
  layer->flags.cursor = 0;

  LayerStack* layer_stack = GetLayerStack(display_handle, 0);
  layer_stack->flags.flags=0;
  error = displayintf->Prepare(layer_stack);
  if (error != kErrorNone) {
    if (error == kErrorShutDown) {
    } else if (error != kErrorPermission) {
      QMMF_ERROR("%s:%s: Prepare failed. Error = %d", TAG, __func__, error);
    }
    delete surfaceinfo;
    FreeLayer(display_handle, *surface_id);
    pthread_mutex_unlock(&thread_lock_);
    return error;
  }
  delete layer_stack;
  layer_stack = nullptr;
  if(!surface_config.use_buffer) {
    for(uint32_t i=0; i<surface_config.buffer_count;i++) {
      BufferInfo *bufferinfo = new BufferInfo();
      Buff_Info  *bufiduse = new Buff_Info();

      bufferinfo->buffer_config.width = surface_config.width;
      bufferinfo->buffer_config.height = surface_config.height;
      bufferinfo->buffer_config.format = (LayerBufferFormat)surface_config.format;
      bufferinfo->buffer_config.buffer_count = 1;
      bufferinfo->buffer_config.cache = surface_config.cache;
      bufferinfo->alloc_buffer_info.fd = -1;
      bufferinfo->alloc_buffer_info.stride = 0;
      bufferinfo->alloc_buffer_info.size = 0;
      surfaceinfo->buffer_internal = !surface_config.use_buffer;
      surfaceinfo->buffer_info.insert({i,bufferinfo});
      bufiduse->commited = 0;
      bufiduse->queued = 0;
      bufiduse->dequed = 0;
      surfaceinfo->buf_id_use.insert({i,bufiduse});
      surfaceinfo->mmapbuf = nullptr;
      error = buffer_allocator_.AllocateBuffer(bufferinfo);
      if (error != kErrorNone) {
        QMMF_ERROR("%s:%s: AllocateBuffer Failed. Error = %d", TAG,
            __func__, error);
        pthread_mutex_unlock(&thread_lock_);
        return -ENOMEM;
      }
    }
  }

  pthread_mutex_unlock(&thread_lock_);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t DisplayImpl::DestroySurface(DisplayHandle display_handle,
    const uint32_t surface_id) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  pthread_mutex_lock(&thread_lock_);

  int32_t ret = NO_ERROR;
  auto displayinfo = displayinfo_.find(display_handle);
  if (displayinfo == displayinfo_.end()) {
    QMMF_ERROR("%s: %s() no displayinfo %u", TAG, __func__, display_handle);
    pthread_mutex_unlock(&thread_lock_);
    return -EINVAL;
  }

  auto surfaceinfo = displayinfo->second->surfaceinfo_.find(surface_id);
  assert(surfaceinfo->second != NULL);

  for (std::map<int32_t, BufferInfo*>::iterator it =
      surfaceinfo->second->buffer_info.begin() ;
      it != surfaceinfo->second->buffer_info.end(); ++it) {
    if (surfaceinfo->second->buffer_internal) {
      buffer_allocator_.FreeBuffer(it->second);
    }
    delete it->second;
    it->second = nullptr;
    surfaceinfo->second->buffer_info.erase(it);
  }

  for (std::map<int32_t, Buff_Info*>::iterator it =
      surfaceinfo->second->buf_id_use.begin() ;
      it != surfaceinfo->second->buf_id_use.end(); ++it) {
    surfaceinfo->second->buf_id_use.erase(it);
  }

  FreeLayer(display_handle, surface_id);
  surfaceinfo->second->buffer_info.clear();
  surfaceinfo->second->buf_id_use.clear();
  delete surfaceinfo->second;
  displayinfo->second->surfaceinfo_.erase(surfaceinfo);
  pthread_mutex_unlock(&thread_lock_);

  return ret;
}
status_t DisplayImpl::DequeueSurfaceBuffer(DisplayHandle display_handle,
    const uint32_t surface_id, SurfaceBuffer &surface_buffer) {
    QMMF_INFO("%s:%s: Enter", TAG, __func__);

  int32_t ret = NO_ERROR;
  pthread_mutex_lock(&thread_lock_);

  auto displayinfo = displayinfo_.find(display_handle);
  if (displayinfo == displayinfo_.end()) {
    QMMF_ERROR("%s: %s() no displayinfo %u", TAG, __func__, display_handle);
    pthread_mutex_unlock(&thread_lock_);
    return -EINVAL;
  }
  auto surfaceinfo = displayinfo->second->surfaceinfo_.find(surface_id);
  assert(surfaceinfo->second != NULL);
  surface_buffer.buf_id = -1;


  for (std::map<int, Buff_Info*>::iterator it =
          surfaceinfo->second->buf_id_use.begin();
          it != surfaceinfo->second->buf_id_use.end(); ++it) {
      if (!it->second->dequed) {
        it->second->dequed =1;
        it->second->queued = 0;
        it->second->commited = 0;
      auto buffer_info = surfaceinfo->second->buffer_info.find(it->first);
      if (buffer_info != surfaceinfo->second->buffer_info.end()) {
        BufferInfo* bufferinfo = buffer_info->second;
        assert(bufferinfo != NULL);
        //TBD for plane_info other than 0.
        surface_buffer.plane_info[0].width = bufferinfo->buffer_config.width;
        surface_buffer.plane_info[0].height = bufferinfo->buffer_config.height;
        surface_buffer.format = (SurfaceFormat)bufferinfo->buffer_config.format;
        surface_buffer.plane_info[0].ion_fd = bufferinfo->alloc_buffer_info.fd;
        QMMF_DEBUG("%s:%s State of Buffer Ion_Fd::%u queued::%u dequed::%u"
            "commited::%u", TAG, __func__, surface_buffer.plane_info[0].ion_fd,
            it->second->queued, it->second->dequed, it->second->commited);
        surface_buffer.buf_id = it->first;
        surface_buffer.plane_info[0].offset = 0;
        surface_buffer.plane_info[0].stride = bufferinfo->alloc_buffer_info.stride;
        surface_buffer.plane_info[0].size = bufferinfo->alloc_buffer_info.size/
            bufferinfo->buffer_config.buffer_count;
        surface_buffer.capacity = bufferinfo->alloc_buffer_info.size;
        break;
      }
    }
  }
  for (std::map<int, Buff_Info*>::iterator it =
       surfaceinfo->second->buf_id_use.begin();
       it != surfaceinfo->second->buf_id_use.end(); ++it) {
       if(it->second->commited == 1)
        it->second->dequed =0;
       it->second->commited = 0;
   }

  pthread_mutex_unlock(&thread_lock_);

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t DisplayImpl::QueueSurfaceBuffer(DisplayHandle display_handle,
    const uint32_t surface_id, SurfaceBuffer &surface_buffer,
    SurfaceParam &surface_param) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  pthread_mutex_lock(&thread_lock_);

  int32_t ret = NO_ERROR;
  auto displayinfo = displayinfo_.find(display_handle);
  if (displayinfo == displayinfo_.end()) {
    QMMF_ERROR("%s: %s() no displayinfo %u", TAG, __func__, display_handle);
    pthread_mutex_unlock(&thread_lock_);
    return -EINVAL;
  }
  auto surfaceinfo = displayinfo->second->surfaceinfo_.find(surface_id);
  assert(surfaceinfo->second != NULL);

  Layer* layer = GetLayer(display_handle, surface_id);
  assert(layer != NULL);
  if(layer->input_buffer.release_fence_fd>0) {
    close(layer->input_buffer.release_fence_fd);
  }
  BufferInfo buffer_info;
  int32_t aligned_width, aligned_height;
  aligned_width = surface_buffer.plane_info[0].width;
  aligned_height = surface_buffer.plane_info[0].height;
  buffer_info.buffer_config.width = surface_buffer.plane_info[0].width;
  buffer_info.buffer_config.height = surface_buffer.plane_info[0].height;
  buffer_info.buffer_config.format = (LayerBufferFormat)surface_buffer.format;
  buffer_info.buffer_config.buffer_count = 1;
  buffer_info.buffer_config.cache = 0;
  buffer_info.alloc_buffer_info.fd = -1;
  buffer_info.alloc_buffer_info.stride = 0;
  buffer_info.alloc_buffer_info.size = 0;
  ret = buffer_allocator_.GetBufferInfo(&buffer_info, aligned_width, aligned_height);
  if (ret != kErrorNone) {
      QMMF_ERROR("%s:%s: GetBufferInfo Failed. Error = %d", TAG,
          __func__, ret);
  }
  layer->input_buffer.width = aligned_width;
  layer->input_buffer.height = aligned_height;
  layer->input_buffer.unaligned_width = surface_buffer.plane_info[0].width;
  layer->input_buffer.unaligned_height = surface_buffer.plane_info[0].height;
  layer->input_buffer.size = surface_buffer.plane_info[0].size;
  layer->input_buffer.planes[0].offset = surface_buffer.plane_info[0].offset;
  layer->input_buffer.planes[0].stride = surface_buffer.plane_info[0].stride;
  layer->input_buffer.color_metadata.colorPrimaries = ColorPrimaries_BT601_6_525;
  layer->input_buffer.color_metadata.range = Range_Limited;
  SetRect(surface_param.dst_rect, &layer->dst_rect);
  SetRect(surface_param.src_rect, &layer->src_rect);
  layer->blending = (LayerBlending)surface_param.surface_blending;
  layer->transform.flip_horizontal =
      surface_param.surface_transform.flip_horizontal;
  layer->transform.flip_vertical =
      surface_param.surface_transform.flip_vertical;
  layer->transform.rotation = surface_param.surface_transform.rotation;
  layer->plane_alpha = 0xFF;
  layer->frame_rate = surface_param.frame_rate;
  layer->solid_fill_color = surface_param.solid_fill_color;
  layer->flags.solid_fill = surface_param.surface_flags.solid_fill;
  layer->flags.cursor = surface_param.surface_flags.cursor;
  layer->input_buffer.planes[0].fd = surface_buffer.plane_info[0].ion_fd;
  QMMF_DEBUG("%s:%s Value of Buffer Ion_Fd", TAG,
                __func__, surface_buffer.plane_info[0].ion_fd);
  layer->input_buffer.buffer_id = surface_buffer.buf_id;
  layer->flags.updating = true;
  if (surfaceinfo->second->buffer_internal) {
    auto buf_id_use = surfaceinfo->second->buf_id_use.find(surface_buffer.buf_id);
    buf_id_use->second->queued = 1;
    buf_id_use->second->commited = 0;
  } else {
    std::map<int, Buff_Info*>::iterator it;
    for (it = surfaceinfo->second->buf_id_use.begin() ;
          it != surfaceinfo->second->buf_id_use.end(); ++it) {
      if (!it->second->queued && it->first == surface_buffer.buf_id) {
        auto buffer_info = surfaceinfo->second->buffer_info.find(it->first);
        if (buffer_info != surfaceinfo->second->buffer_info.end()) {
          BufferInfo* bufferinfo = buffer_info->second;
          if (bufferinfo) {
            bufferinfo->buffer_config.width =
                surface_buffer.plane_info[0].width;
            bufferinfo->buffer_config.height =
                surface_buffer.plane_info[0].height;
            bufferinfo->buffer_config.format =
                (LayerBufferFormat)surface_buffer.format;
            bufferinfo->alloc_buffer_info.stride =
                surface_buffer.plane_info[0].stride;
            bufferinfo->alloc_buffer_info.size =
                surface_buffer.plane_info[0].size;
            bufferinfo->alloc_buffer_info.fd =
                surface_buffer.plane_info[0].ion_fd;

            it->second->queued = 1;
            it->second->commited = 0;
            QMMF_DEBUG("%s:%s State of Buffer Ion_Fd::%u queued::%u dequed::%u "
                "commited::%u", TAG, __func__, bufferinfo->alloc_buffer_info.fd,
                it->second->queued, it->second->dequed, it->second->commited);
            break;
          }
        }
      }
    }
    if (it == surfaceinfo->second->buf_id_use.end()) {
      BufferInfo *bufferinfo = new BufferInfo();
      Buff_Info  *bufiduse = new Buff_Info();
      bufiduse->queued = 1;
      bufiduse->commited = 0;
      int32_t buf_id = surface_buffer.buf_id;
      bufferinfo->buffer_config.width =surface_buffer.plane_info[0].width;
      bufferinfo->buffer_config.height = surface_buffer.plane_info[0].height;
      bufferinfo->buffer_config.format =
          (LayerBufferFormat)surface_buffer.format;
      bufferinfo->buffer_config.buffer_count = 1;
      bufferinfo->alloc_buffer_info.fd = surface_buffer.plane_info[0].ion_fd;
      QMMF_DEBUG("%s:%s State of Buffer Ion_Fd::%u queued::%u dequed::%u "
          "commited::%u", TAG, __func__, bufferinfo->alloc_buffer_info.fd,
          it->second->queued, it->second->dequed, it->second->commited);
      bufferinfo->alloc_buffer_info.stride =
          surface_buffer.plane_info[0].stride;
      bufferinfo->alloc_buffer_info.size = surface_buffer.plane_info[0].size;
      surfaceinfo->second->buffer_info.insert({buf_id,bufferinfo});
      surfaceinfo->second->buf_id_use.insert({buf_id,bufiduse});
    }
  }
  pthread_mutex_unlock(&thread_lock_);

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t DisplayImpl::GetDisplayParam(DisplayHandle display_handle,
    DisplayParamType param_type, void *param, size_t param_size) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  uint32_t ret = NO_ERROR;

  auto displayinfo = displayinfo_.find(display_handle);
  if (displayinfo == displayinfo_.end()) {
    QMMF_ERROR("%s: %s() no displayinfo %d", TAG, __func__, display_handle);
    return -EINVAL;
  }

  DisplayInterface* displayintf = displayinfo->second->display_intf;
  assert(displayintf != NULL);
  if (param_type == DisplayParamType::kBrightness){
    DisplayError error = displayintf->GetPanelBrightness((int*)param);
    if (error != kErrorNone) {
      QMMF_ERROR("%s:%s: GetPanelBrightness Failed. Error = %d", TAG, __func__,
          error);
      return error;
    }
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t DisplayImpl::SetDisplayParam(DisplayHandle display_handle,
    DisplayParamType param_type, void *param, size_t param_size) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  uint32_t ret = NO_ERROR;

  auto displayinfo = displayinfo_.find(display_handle);
  if (displayinfo == displayinfo_.end()) {
    QMMF_ERROR("%s: %s() no displayinfo %u", TAG, __func__, display_handle);
    return -EINVAL;
  }

  DisplayInterface* displayintf = displayinfo->second->display_intf;
  assert(displayintf != NULL);
  if (param_type == DisplayParamType::kBrightness){
    int *level = (int*)param;
    DisplayError error = displayintf->SetPanelBrightness(*level);
    if (error != kErrorNone) {
      QMMF_ERROR("%s:%s: SetPanelBrightness Failed. Error = %d", TAG, __func__,
          error);
      return error;
    }
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t DisplayImpl::DequeueWBSurfaceBuffer(DisplayHandle display_handle,
    const uint32_t surface_id, SurfaceBuffer &surface_buffer) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  //TBD
  int32_t ret = NO_ERROR;

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t DisplayImpl::QueueWBSurfaceBuffer(DisplayHandle display_handle,
    const uint32_t surface_id, const SurfaceBuffer &surface_buffer) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  //TBD
  int32_t ret = NO_ERROR;
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

void* DisplayImpl::HandleVSync(void *userdata) {
  bool run = true;
  DisplayInterface* displayintf;

  DisplayImpl *displayimpl = reinterpret_cast<DisplayImpl *>(userdata);
  if (NULL == displayimpl) {
    return NULL;
  }

  SCOPE_LOCK(displayimpl->vsync_callback_locker_);
  displayimpl->vsync_callback_locker_.Wait();
  pthread_mutex_lock(&displayimpl->thread_lock_);
  //Need to disable Hardware VSYnc since its power intensive.
  for (std::map<DisplayHandle, DisplayInfo*>::iterator it =
      displayimpl->displayinfo_.begin();
      it != displayimpl->displayinfo_.end(); ++it) {
    displayintf = it->second->display_intf;
    assert(displayintf != NULL);

    DisplayError error = displayintf->SetVSyncState(false);
    if (error != kErrorNone) {
      QMMF_ERROR("%s:%s: SetVSyncState Failed. Error = %d", TAG, __func__,
          error);
    }
    pthread_mutex_unlock(&displayimpl->thread_lock_);
    //To be checked later.
    break;
  }
  while (run) {
    pthread_mutex_lock(&displayimpl->thread_lock_);
    run = displayimpl->running_;
    for (std::map<DisplayHandle, DisplayInfo*>::iterator it =
        displayimpl->displayinfo_.begin();
        it != displayimpl->displayinfo_.end(); ++it) {
      int64_t timestamp = 0;
      displayintf = it->second->display_intf;
      if(displayintf == NULL) {
        pthread_mutex_unlock(&displayimpl->thread_lock_);
        continue;
      }
      if(!it->second->layer_count) {
        pthread_mutex_unlock(&displayimpl->thread_lock_);
        continue;
      }
      LayerStack* layer_stack = displayimpl->GetLayerStack(it->first, 1);
      if(layer_stack->layers.size()) {
        layer_stack->flags.flags=0;
        DisplayError error = displayintf->Prepare(layer_stack);
        if (error != kErrorNone) {
          QMMF_WARN("%s:%s: Prepare failed. Error = %d", TAG, __func__, error);
        } else {
          error = displayintf->Commit(layer_stack);
          if (error != kErrorNone) {
            QMMF_WARN("%s:%s: Commit failed. Error = %d", TAG, __func__,
                  error);
          }
        }
      }
      if(layer_stack->retire_fence_fd>0) {
        close(layer_stack->retire_fence_fd);
      }
      delete layer_stack;
      layer_stack = nullptr;
      assert(it->second->remote_cb_.get() != nullptr);
      it->second->remote_cb_->notifyVSyncEvent(timestamp);
    }
    pthread_mutex_unlock(&displayimpl->thread_lock_);
    //sleep time in microseconds for display refresh rate.
    usleep(16666);
  }

  return nullptr;
}

void DisplayImpl::SetRect(const SurfaceRect &source, LayerRect *target) {
  target->left = FLOAT(source.left);
  target->top = FLOAT(source.top);
  target->right = FLOAT(source.right);
  target->bottom = FLOAT(source.bottom);
}

Layer* DisplayImpl::AllocateLayer(DisplayHandle display_handle,
    uint32_t* surface_id) {
    QMMF_INFO("%s:%s: Enter", TAG, __func__);

  auto displayinfo = displayinfo_.find(display_handle);
  if (displayinfo == displayinfo_.end()) {
    QMMF_ERROR("%s: %s() no displayinfo %u", TAG, __func__, display_handle);
    return NULL;
  }
  Layer* layer = new Layer();
  assert(layer != NULL);

  displayinfo->second->layer_count++;
  *surface_id = displayinfo->second->layer_count;
  QMMF_INFO("%s:%s: Exit", TAG, __func__);

  return layer;
}

void DisplayImpl::FreeLayer(DisplayHandle display_handle,
    const uint32_t surface_id) {
    QMMF_INFO("%s:%s: Enter", TAG, __func__);

  auto displayinfo = displayinfo_.find(display_handle);
  if (displayinfo == displayinfo_.end()) {
    QMMF_ERROR("%s: %s() no displayinfo %u", TAG, __func__, display_handle);
    return;
  }

  auto surfaceinfo = displayinfo->second->surfaceinfo_.find(surface_id);
  if (surfaceinfo == displayinfo->second->surfaceinfo_.end()) {
    QMMF_ERROR("%s: %s() no displayinfo %u", TAG, __func__, display_handle);
    return;
  }

  Layer *layer = surfaceinfo->second->layer;
  assert(layer != NULL);
  delete layer;
  surfaceinfo->second->layer = nullptr;
  displayinfo->second->layer_count--;

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

Layer* DisplayImpl::GetLayer(DisplayHandle display_handle,
    const uint32_t surface_id) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  auto displayinfo = displayinfo_.find(display_handle);
  if (displayinfo == displayinfo_.end()) {
    QMMF_ERROR("%s: %s() no displayinfo %u", TAG, __func__, display_handle);
    return NULL;
  }

  auto surfaceinfo = displayinfo->second->surfaceinfo_.find(surface_id);
  if (surfaceinfo == displayinfo->second->surfaceinfo_.end()) {
    QMMF_ERROR("%s: %s() no displayinfo %u", TAG, __func__, display_handle);
    return NULL;
  }
  assert(surfaceinfo->second->layer);

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return surfaceinfo->second->layer;
}

LayerStack* DisplayImpl::GetLayerStack(DisplayHandle display_handle,
    bool queued_buffers_only) {

  QMMF_VERBOSE("%s:%s: Enter", TAG, __func__);
  auto displayinfo = displayinfo_.find(display_handle);
  if (displayinfo == displayinfo_.end()) {
    QMMF_ERROR("%s: %s() no displayinfo %u", TAG, __func__, display_handle);
    return NULL;
  }
  LayerStack* layer_stack = new LayerStack();

  for(uint32_t i=1;i<=displayinfo->second->layer_count;i++) {
    auto surfaceinfo = displayinfo->second->surfaceinfo_.find(i);
    if (surfaceinfo == displayinfo->second->surfaceinfo_.end()) {
      QMMF_ERROR("%s: %s() no displayinfo:%d i: %d", TAG, __func__,
          display_handle, i);
      continue;
    }
    if(surfaceinfo->second->layer != NULL) {
      bool push_layer=0;
      if (!queued_buffers_only)
        push_layer =1;
      else {
        for (std::map<int, Buff_Info*>::iterator it =
            surfaceinfo->second->buf_id_use.begin() ;
            it != surfaceinfo->second->buf_id_use.end(); ++it) {
            QMMF_DEBUG("%s:%s State of Buffer is queued::%u dequed::%u"
                "commited::%u", TAG, __func__, it->second->queued,
                it->second->dequed, it->second->commited);
            if(it->second->queued) {
              push_layer = 1;
              it->second->queued = 0;
              it->second->commited =1;
            }
        }
      }
      if(push_layer) {
        layer_stack->layers.push_back(surfaceinfo->second->layer);
      }
    }
  }

  QMMF_VERBOSE("%s:%s: Exit", TAG, __func__);
  return layer_stack;
}

DisplayError DisplayImpl::VSync(const DisplayEventVSync &vsync) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  if (vsync_state_) {
    SCOPE_LOCK(vsync_callback_locker_);
    vsync_callback_locker_.Signal();
    vsync_state_ = false;
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return kErrorNone;
}

DisplayError DisplayImpl::VSync(int fd, unsigned int sequence,
                                unsigned int tv_sec, unsigned int tv_usec,
                                void *data) {
  return kErrorNotSupported;
}

DisplayError DisplayImpl::PFlip(int fd, unsigned int sequence,
                                unsigned int tv_sec, unsigned int tv_usec,
                                void *data) {
  return kErrorNotSupported;
}

DisplayError DisplayImpl::Refresh() {
  return kErrorNotSupported;
}

DisplayError DisplayImpl::CECMessage(char *message) {
  return kErrorNotSupported;
}

}; // namespace display

}; //namespace qmmf
