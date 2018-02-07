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

#define TAG "DisplayService"

#include "display/src/service/qmmf_display_service.h"
#include <sys/time.h>
#include <sys/resource.h>
#include <errno.h>
#include <linux/msm_ion.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

namespace qmmf {

namespace display {

DisplayService::DisplayService()
  :connected_(false) {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  QMMF_INFO("%s:%s: DisplayService Instantiated! ", TAG, __func__);
  QMMF_INFO("%s:%s: Exit ", TAG, __func__);
}

DisplayService::~DisplayService()
{
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  death_notifier_.clear();
  client_handlers_.clear();
  QMMF_INFO("%s:%s: Exit ", TAG, __func__);
}

status_t DisplayService::onTransact(uint32_t code, const Parcel& data,
    Parcel* reply, uint32_t flag)
{
  QMMF_LEVEL2("%s:%s: Enter ", TAG, __func__);
  CHECK_INTERFACE(IDisplayService, data, reply);
  int32_t ret = 0;

  switch (code) {
    case DISPLAY_CONNECT: {
      ret = Connect();
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;
    case DISPLAY_DISCONNECT:
    {
        ret = Disconnect();
        reply->writeInt32(ret);
        return NO_ERROR;
    }
    break;
    case DISPLAY_CREATE_DISPLAY: {
      sp<IDisplayServiceCallback> client_cb_handle = interface_cast
          <IDisplayServiceCallback>(data.readStrongBinder());
      DisplayType display_type = static_cast<DisplayType>(data.readInt32());
      DisplayHandle display_handle;
      ret = CreateDisplay(client_cb_handle, display_type, &display_handle);
      reply->writeInt32(static_cast<int32_t>(display_handle));
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;
    case DISPLAY_DESTROY_DISPLAY:
    {
        DisplayHandle display_handle = static_cast<DisplayHandle>
            (data.readInt32());
        ret = DestroyDisplay(display_handle);
        ion_fd_map::iterator it_fd;
        for (it_fd=ion_fd_mapping.begin();
            it_fd!=ion_fd_mapping.end(); ++it_fd) {
          ion_fd_mapping.erase(it_fd);
        }
        reply->writeInt32(ret);
        return NO_ERROR;
    }
    break;
    case DISPLAY_CREATE_SURFACE:
    {
      DisplayHandle display_handle = static_cast<DisplayHandle>
          (data.readInt32());
      uint32_t blob_size;
      data.readUint32(&blob_size);
      android::Parcel::ReadableBlob blob;
      data.readBlob(blob_size, &blob);
      void* params = const_cast<void*>(blob.data());
      SurfaceConfig surface_config;
      memset(&surface_config, 0x0, sizeof surface_config);
      memcpy(&surface_config, params, blob_size);

      uint32_t surface_id;
      blob.release();
      ret = CreateSurface(display_handle, surface_config, &surface_id);
      use_buffer_mapping.insert({surface_id, surface_config.use_buffer});
      reply->writeUint32(surface_id);
      reply->writeInt32(ret);

      return NO_ERROR;
    }
    break;
    case DISPLAY_DESTROY_SURFACE:
    {
      DisplayHandle display_handle = static_cast<DisplayHandle>
          (data.readInt32());
      uint32_t surface_id;
      data.readUint32(&surface_id);
      ret = DestroySurface(display_handle, surface_id);
      for (use_buffer_map::iterator it = use_buffer_mapping.begin();
          it != use_buffer_mapping.end(); ++it) {
        if (it->first == surface_id) {
          use_buffer_mapping.erase(surface_id);
          break;
        }
      }
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;
    case DISPLAY_DEQUEUE_SURFACE_BUFFER:
    {
      DisplayHandle display_handle = static_cast<DisplayHandle>
          (data.readInt32());
      uint32_t surface_id;
      data.readUint32(&surface_id);
      SurfaceBuffer surface_buffer;
      uint32_t blob_size = sizeof surface_buffer;
      memset(&surface_buffer, 0x0, blob_size);
      ret = DequeueSurfaceBuffer(display_handle, surface_id, surface_buffer);
      reply->writeInt32(ret);
      reply->writeInt32(blob_size);
      android::Parcel::WritableBlob blob;
      reply->writeBlob(blob_size, false, &blob);
      memset(blob.data(), 0x0, blob_size);
      memcpy(blob.data(), reinterpret_cast<void*>(&surface_buffer), blob_size);
      ion_fd_map::iterator it_fd;
      if(surface_buffer.buf_id != -1) {
        for (it_fd = ion_fd_mapping.begin(); it_fd != ion_fd_mapping.end();
            ++it_fd) {
          if(it_fd->first == surface_buffer.plane_info[0].ion_fd) {
            for (use_buffer_map::iterator it = use_buffer_mapping.begin();
                it != use_buffer_mapping.end(); ++it) {
              if (it->first == surface_id) {
                if (it->second == 1)
                  reply->writeInt32(it_fd->second);
                else
                  reply->writeInt32(it_fd->first);
                break;
              }
            }
          }
        }
        if(it_fd == ion_fd_mapping.end()) {
          ion_fd_mapping.insert({surface_buffer.plane_info[0].ion_fd, 1});
          reply->writeInt32(0);
          reply->writeInt32(surface_buffer.plane_info[0].ion_fd);
          reply->writeFileDescriptor(surface_buffer.plane_info[0].ion_fd);
        }
      }
      return NO_ERROR;
    }
    break;
    case DISPLAY_QUEUE_SURFACE_BUFFER:
    {
      DisplayHandle display_handle = static_cast<DisplayHandle>
          (data.readInt32());
      uint32_t surface_id;
      data.readUint32(&surface_id);

      uint32_t blob_size;
      SurfaceBuffer surface_buffer;
      SurfaceParam surface_param;

      data.readUint32(&blob_size);
      android::Parcel::ReadableBlob blob;

      data.readBlob(blob_size, &blob);
      void* buffer = const_cast<void*>(blob.data());
      memset(&surface_buffer, 0x0, sizeof surface_buffer);
      memcpy(&surface_buffer, buffer, blob_size);
      blob.release();

      data.readUint32(&blob_size);
      data.readBlob(blob_size, &blob);

      void* params = const_cast<void*>(blob.data());
      memset(&surface_param, 0x0, sizeof surface_param);
      memcpy(&surface_param, params, blob_size);
      blob.release();
      for (use_buffer_map::iterator it = use_buffer_mapping.begin();
          it != use_buffer_mapping.end(); ++it) {
        if (it->first == surface_id && it->second == 1) {
          ion_fd_map::iterator it_fd;
          for (it_fd = ion_fd_mapping.begin(); it_fd != ion_fd_mapping.end();
              ++it_fd) {
            if (it_fd->second == surface_buffer.plane_info[0].ion_fd) {
              surface_buffer.plane_info[0].ion_fd = it_fd->first;
              break;
            }
          }
          if(it_fd == ion_fd_mapping.end()) {
            int32_t ion_fd;
            ion_fd = surface_buffer.plane_info[0].ion_fd;
            surface_buffer.plane_info[0].ion_fd =
                dup(data.readFileDescriptor());
            struct ion_fd_data ion_info_fd;
            memset(&ion_info_fd, 0x0, sizeof(ion_info_fd));
            ion_info_fd.fd = surface_buffer.plane_info[0].ion_fd;
            ret = ioctl(ion_device_, ION_IOC_IMPORT, &ion_info_fd);
            if(ret != NO_ERROR) {
              QMMF_ERROR("%s:%s: ION_IOC_IMPORT failed for fd(%d) ret:%d errno:%d",
                  TAG, __func__, ion_info_fd.fd, ret, errno);
            }
            ion_fd_mapping.insert({surface_buffer.plane_info[0].ion_fd,
                ion_fd});
          }
          break;
        }
      }
      ret = QueueSurfaceBuffer(display_handle, surface_id, surface_buffer,
          surface_param);
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;
    case DISPLAY_GET_DISPLAY_PARAM:
    {
      DisplayHandle display_handle = static_cast<DisplayHandle>
          (data.readInt32());
      uint32_t param_type;
      int32_t  blob_size;
      data.readUint32(&param_type);

      data.readInt32(&blob_size);
      void* params=operator new(blob_size);
      memset(params, 0x0, blob_size);
      ret = GetDisplayParam(display_handle, (DisplayParamType)param_type,
          params, blob_size);

      reply->writeInt32(ret);
      reply->writeInt32(blob_size);
      android::Parcel::WritableBlob blob;
      reply->writeBlob(blob_size, false, &blob);
      memset(blob.data(), 0x0, blob_size);
      memcpy(blob.data(), reinterpret_cast<void*>(params), blob_size);
      operator delete(params);
      return NO_ERROR;
    }
    break;
    case DISPLAY_SET_DISPLAY_PARAM:
    {
      DisplayHandle display_handle = static_cast<DisplayHandle>
          (data.readInt32());
      uint32_t param_type;
      int32_t  blob_size;
      data.readUint32(&param_type);

      data.readInt32(&blob_size);
      android::Parcel::ReadableBlob blob;
      data.readBlob(blob_size, &blob);
      void* params = const_cast<void*>(blob.data());
      blob.release();
      ret = SetDisplayParam(display_handle, (DisplayParamType)param_type,
          params, blob_size);
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;
    case DISPLAY_DEQUEUE_WBSURFACE_BUFFER:
    {
      DisplayHandle display_handle = static_cast<DisplayHandle>
          (data.readInt32());
      uint32_t surface_id;
      data.readUint32(&surface_id);
      SurfaceBuffer surface_buffer;
      int32_t blob_size = sizeof surface_buffer;
      memset(&surface_buffer, 0x0, blob_size);
      ret = DequeueWBSurfaceBuffer(display_handle, surface_id, surface_buffer);
      reply->writeInt32(ret);
      reply->writeInt32(blob_size);
      android::Parcel::WritableBlob blob;
      reply->writeBlob(blob_size, false, &blob);
      memset(blob.data(), 0x0, blob_size);
      memcpy(blob.data(), reinterpret_cast<void*>(&surface_buffer), blob_size);
      return NO_ERROR;
    }
    break;
    case DISPLAY_QUEUE_WBSURFACE_BUFFER:
    {
      DisplayHandle display_handle = static_cast<DisplayHandle>
          (data.readInt32());
     uint32_t surface_id;
     data.readUint32(&surface_id);

     uint32_t blob_size;
     data.readUint32(&blob_size);
     android::Parcel::ReadableBlob blob;
     data.readBlob(blob_size, &blob);
     void* params = const_cast<void*>(blob.data());
     SurfaceBuffer surface_buffer;
     memset(&surface_buffer, 0x0, sizeof surface_buffer);
     memcpy(&surface_buffer, params, blob_size);
     blob.release();
     ret = QueueWBSurfaceBuffer(display_handle, surface_id, surface_buffer);
     reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;
    default:
    {
        QMMF_ERROR("QIPCamService: %s: Method not supported ",__func__);
        reply->writeInt32(-1);
    }
    break;
  }

  QMMF_LEVEL2("%s: DisplayService::Exit ",__func__);
  return 0;
}

status_t DisplayService::Connect() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  ion_device_ = open("/dev/ion", O_RDONLY);
  if (ion_device_ < 0) {
    QMMF_ERROR("%s:%s: Can't open Ion device!", TAG, __func__);
    return NO_INIT;
  }

  display_ = DisplayImpl::CreateDisplayCore();
  if (!display_) {
    QMMF_ERROR("%s:%s: Can't create Display Instance!!", TAG, __func__);
    return NO_MEMORY;
  }

  auto ret = display_->Connect();
  assert(ret == NO_ERROR);

  death_notifier_ = new DeathNotifier(this);
  if (NULL == death_notifier_.get()) {
    QMMF_ERROR("%s:%s: Unable to allocate death notifier!", TAG, __func__);
    return NO_MEMORY;
  }

  connected_ = true;
  QMMF_INFO("%s:%s: EXIT ", TAG, __func__);
  return ret;
}

status_t DisplayService::Disconnect() {

  QMMF_LEVEL1("%s:%s: Enter ", TAG, __func__);
  int32_t ret = NO_ERROR;
  if (!connected_)
    return NO_INIT;

  close(ion_device_);

  if (death_notifier_.get() != nullptr) {
    death_notifier_.clear();
    death_notifier_ = nullptr;
  }

  if (display_)
  {
    ret = display_->Disconnect();
    delete display_;
  }


  QMMF_LEVEL1("%s:%s: Exit ", TAG, __func__);

  return ret;
}

status_t DisplayService::CreateDisplay(const sp<IDisplayServiceCallback>&
    service_cb, DisplayType display_type, DisplayHandle* display_handle) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  sp<RemoteCallBack>           remote_callback;

  remote_callback = new RemoteCallBack(service_cb);
  auto ret = display_->CreateDisplay(remote_callback, display_type,
      display_handle);
  assert(ret == NO_ERROR);

  IInterface::asBinder(remote_callback->getRemoteClient())
      ->linkToDeath(death_notifier_);
  client_handlers_.insert({*display_handle,
      remote_callback->getRemoteClient()});
  remote_callback_.insert({*display_handle, remote_callback});

  connected_ = true;
  QMMF_INFO("%s:%s: EXIT ", TAG, __func__);
  return ret;
}

status_t DisplayService::DestroyDisplay(DisplayHandle display_handle) {

  QMMF_LEVEL1("%s:%s: Enter ", TAG, __func__);
  int32_t ret = NO_ERROR;
  if (!connected_)
    return NO_INIT;

  auto remote_callback = remote_callback_.find(display_handle);
  if (remote_callback == remote_callback_.end()) {
    QMMF_ERROR("%s: %s() no remote_callback for handle[%d]", TAG, __func__,
               display_handle);
    return -EINVAL;
  }

  IInterface::asBinder(remote_callback->second->getRemoteClient())
      ->unlinkToDeath(death_notifier_);

  if (display_)
  {
    ret = display_->DestroyDisplay(display_handle);
  }
  assert(ret == NO_ERROR);

  QMMF_LEVEL1("%s:%s: Exit ", TAG, __func__);

  return ret;
}

status_t DisplayService::CreateSurface(DisplayHandle display_handle,
    SurfaceConfig &surface_config,
    uint32_t* surface_id) {
  QMMF_LEVEL1("%s:%s: Enter ", TAG, __func__);

  if(!connected_) {
    QMMF_WARN("%s:%s: Connect Should be called, before calling CreateSurface!!",
        TAG, __func__);
    return NO_INIT;
  }
  assert(display_ != NULL);

  auto ret = display_->CreateSurface(display_handle, surface_config,
      surface_id);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: Can't create surface!!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t DisplayService::DestroySurface(DisplayHandle display_handle,
    const uint32_t surface_id) {

  QMMF_LEVEL1("%s:%s: Enter ", TAG, __func__);

  assert(display_ != NULL);

  QMMF_INFO("%s:%s: surface_id:%d", TAG, __func__, surface_id);
  auto ret = display_->DestroySurface(display_handle, surface_id);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: Can't destroy surface!!", TAG, __func__);
    return ret;
  }
  QMMF_LEVEL1("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t DisplayService::DequeueSurfaceBuffer(DisplayHandle display_handle,
    const uint32_t surface_id, SurfaceBuffer &surface_buffer) {

  QMMF_LEVEL1("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(display_ != NULL);
  auto ret = display_->DequeueSurfaceBuffer(display_handle, surface_id,
      surface_buffer);
  assert(ret == NO_ERROR);
  QMMF_LEVEL1("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t DisplayService::QueueSurfaceBuffer(DisplayHandle display_handle,
    const uint32_t surface_id, SurfaceBuffer &surface_buffer,
    SurfaceParam &surface_param) {

  QMMF_LEVEL1("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(display_ != NULL);

  auto ret = display_->QueueSurfaceBuffer(display_handle, surface_id,
      surface_buffer, surface_param);
  assert(ret == NO_ERROR);
  QMMF_LEVEL1("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t DisplayService::GetDisplayParam(DisplayHandle display_handle,
    DisplayParamType param_type, void *param, size_t param_size) {

  QMMF_LEVEL1("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(display_ != NULL);

  QMMF_INFO("%s:%s: param_type:%d", TAG, __func__, param_type);
  auto ret = display_->GetDisplayParam(display_handle, param_type, param,
      param_size);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: GetDisplayParam failed!", TAG, __func__);
  }
  QMMF_LEVEL1("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t DisplayService::SetDisplayParam(DisplayHandle display_handle,
    DisplayParamType param_type, void *param, size_t param_size) {
  QMMF_LEVEL1("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(display_ != NULL);
  auto ret = display_->SetDisplayParam(display_handle, param_type, param,
      param_size);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: SetDisplayParam failed!", TAG, __func__);
  }
  QMMF_LEVEL1("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t DisplayService::DequeueWBSurfaceBuffer(DisplayHandle display_handle,
    const uint32_t surface_id, SurfaceBuffer &surface_buffer) {

  QMMF_LEVEL1("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(display_ != NULL);

  auto ret = display_->DequeueWBSurfaceBuffer(display_handle, surface_id,
      surface_buffer);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: DequeueWBSurfaceBuffer failed!", TAG, __func__);
  }
  QMMF_LEVEL1("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t DisplayService::QueueWBSurfaceBuffer(DisplayHandle display_handle,
    const uint32_t surface_id, const SurfaceBuffer &surface_buffer) {

  QMMF_LEVEL1("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(display_ != NULL);

  auto ret = display_->QueueWBSurfaceBuffer(display_handle, surface_id,
      surface_buffer);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: QueueWBSurfaceBuffer failed!", TAG, __func__);
  }
  QMMF_LEVEL1("%s:%s: Exit ", TAG, __func__);
  return ret;
}

}; //namespace display

}; //namespace qmmf
