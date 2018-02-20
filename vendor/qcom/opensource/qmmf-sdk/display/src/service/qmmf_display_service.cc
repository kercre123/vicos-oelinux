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

#define LOG_TAG "DisplayService"

#include "display/src/service/qmmf_display_service.h"
#include <sys/time.h>
#include <sys/resource.h>
#include <errno.h>
#include <linux/msm_ion.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <vector>

namespace qmmf {

namespace display {

using std::vector;

DisplayService::DisplayService()
  :connected_(false) {
  QMMF_INFO("%s: Enter ", __func__);
  QMMF_INFO("%s: DisplayService Instantiated! ", __func__);
  QMMF_INFO("%s: Exit ", __func__);
}

DisplayService::~DisplayService()
{
  QMMF_INFO("%s: Enter ", __func__);
  death_notifier_.clear();
  client_handlers_.clear();
  QMMF_INFO("%s: Exit ", __func__);
}

status_t DisplayService::onTransact(uint32_t code, const Parcel& data,
    Parcel* reply, uint32_t flag)
{
  QMMF_DEBUG("%s: Enter ", __func__);
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
        {
          std::lock_guard<std::mutex> lock(fd_map_lock_);
          ion_fd_mapping_.clear();
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
      {
        std::lock_guard<std::mutex> lock(use_buffer_map_lock_);
        use_buffer_mapping_.insert({surface_id, surface_config.use_buffer});
      }
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
      vector<int32_t> remove_fds;
      auto use_buffer = use_buffer_mapping_.find(surface_id);
      if (use_buffer != use_buffer_mapping_.end()) {
        if (use_buffer->second == true) {
          for (auto& it : buf_info_map_) {
            if (it.second) {
              if (it.second->surface_id == surface_id) {
                struct ion_handle_data ion_handle;
                memset(&ion_handle, 0, sizeof(ion_handle));
                ion_handle.handle = it.second->ion_handle;
                if (ioctl(ion_device_, ION_IOC_FREE, &ion_handle) < 0) {
                  QMMF_ERROR("%s ION free failed: %d[%s]", __func__, -errno,
                      strerror(errno));
                }

                if (it.second->ion_fd > 0) {
                  auto stat = close(it.second->ion_fd);
                  if (0 != stat) {
                    QMMF_ERROR("%s Failed to close ION fd: %d : %d[%s]", __func__,
                        it.second->ion_fd, -errno, strerror(errno));
                    return -errno;
                  }
                }
                if (it.second != nullptr) {
                  delete (it.second);
                  it.second = nullptr;
                }
                remove_fds.push_back(it.first);
              }
            }
          }
        }
      }
      for (auto fd : remove_fds) {
        buf_info_map_.erase(fd);
      }
      use_buffer = use_buffer_mapping_.find(surface_id);
      if (use_buffer != use_buffer_mapping_.end()) {
        use_buffer_mapping_.erase(surface_id);
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
      {
        std::lock_guard<std::mutex> lock(fd_map_lock_);
        if (surface_buffer.buf_id != -1) {
          for (it_fd = ion_fd_mapping_.begin(); it_fd != ion_fd_mapping_.end();
              ++it_fd) {
            if(it_fd->first == surface_buffer.plane_info[0].ion_fd) {
              for (use_buffer_map::iterator it = use_buffer_mapping_.begin();
                  it != use_buffer_mapping_.end(); ++it) {
                if (it->first == surface_id) {
                  if (it->second == true)
                    reply->writeInt32(it_fd->second);
                  else
                    reply->writeInt32(it_fd->first);
                  break;
                }
              }
            }
          }
          if (it_fd == ion_fd_mapping_.end()) {
            ion_fd_mapping_.insert({surface_buffer.plane_info[0].ion_fd, 1});
            reply->writeInt32(0);
            reply->writeInt32(surface_buffer.plane_info[0].ion_fd);
            reply->writeFileDescriptor(surface_buffer.plane_info[0].ion_fd);
          }
        }

        for (auto& it : ion_fd_mapping_) {
          QMMF_DEBUG("%s ion_fd_mapping_ service_ion_fd::%d "
              "client_ion_fd::%d ", __func__, it.first, it.second);
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
      {
        std::lock_guard<std::mutex> lock(fd_map_lock_);
        for (use_buffer_map::iterator it = use_buffer_mapping_.begin();
            it != use_buffer_mapping_.end(); ++it) {
          if (it->first == surface_id && it->second == true) {
            ion_fd_map::iterator it_fd;
            for (it_fd = ion_fd_mapping_.begin(); it_fd != ion_fd_mapping_.end();
                ++it_fd) {
              if (it_fd->second == surface_buffer.plane_info[0].ion_fd) {
                surface_buffer.plane_info[0].ion_fd = it_fd->first;
                break;
              }
            }
            if(it_fd == ion_fd_mapping_.end()) {
              int32_t ion_fd;
              ion_fd = surface_buffer.plane_info[0].ion_fd;
              surface_buffer.plane_info[0].ion_fd =
                  dup(data.readFileDescriptor());
              QMMF_DEBUG("%s OnTransact client ion_fd::%d ", __func__,
                  surface_buffer.plane_info[0].ion_fd);
              struct ion_fd_data ion_info_fd;
              memset(&ion_info_fd, 0x0, sizeof(ion_info_fd));
              ion_info_fd.fd = surface_buffer.plane_info[0].ion_fd;
              ret = ioctl(ion_device_, ION_IOC_IMPORT, &ion_info_fd);
              if(ret != NO_ERROR) {
                QMMF_ERROR("%s: ION_IOC_IMPORT failed for fd(%d) ret:%d errno:%d",
                    __func__, ion_info_fd.fd, ret, errno);
              }
              ion_fd_mapping_.insert({surface_buffer.plane_info[0].ion_fd,
                  ion_fd});
              BufInfo* bufinfo = new BufInfo();
              bufinfo->ion_fd = surface_buffer.plane_info[0].ion_fd;
              bufinfo->pointer = nullptr;
              bufinfo->frame_len =  surface_buffer.plane_info[0].size;
              bufinfo->ion_handle = ion_info_fd.handle;
              bufinfo->surface_id = surface_id;
              buf_info_map_.insert({surface_buffer.plane_info[0].ion_fd, bufinfo});
            }
            break;
          }
        }
        for (auto& it : ion_fd_mapping_) {
          QMMF_DEBUG("%s ion_fd_mapping_ service_ion_fd::%d "
              "client_ion_fd::%d ", __func__, it.first, it.second);
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

  QMMF_DEBUG("%s: DisplayService::Exit ",__func__);
  return 0;
}

status_t DisplayService::Connect() {

  QMMF_INFO("%s: Enter", __func__);

  std::unique_lock<std::mutex> lock(client_handlers_lock_);
  if (client_handlers_.size() == 0) {
    ion_device_ = open("/dev/ion", O_RDONLY);
    if (ion_device_ < 0) {
      QMMF_ERROR("%s: Can't open Ion device!", __func__);
      return NO_INIT;
    }
  }

  display_ = DisplayImpl::CreateDisplayCore();
  if (!display_) {
    QMMF_ERROR("%s: Can't create Display Instance!!", __func__);
    return NO_MEMORY;
  }

  auto ret = display_->Connect();
  assert(ret == NO_ERROR);

  death_notifier_ = new DeathNotifier(this);
  if (NULL == death_notifier_.get()) {
    QMMF_ERROR("%s: Unable to allocate death notifier!", __func__);
    return NO_MEMORY;
  }

  connected_ = true;
  QMMF_INFO("%s: EXIT ", __func__);
  return ret;
}

status_t DisplayService::Disconnect() {

  QMMF_INFO("%s: Enter ", __func__);
  int32_t ret = NO_ERROR;
  if (!connected_)
    return NO_INIT;

  std::unique_lock<std::mutex> lock(client_handlers_lock_);
  if (client_handlers_.size() == 0) {
    close(ion_device_);

    if (death_notifier_.get() != nullptr) {
      death_notifier_.clear();
      death_notifier_ = nullptr;
    }
  }

  if (display_)
  {
    ret = display_->Disconnect();
    if (client_handlers_.size() == 0) {
        delete display_;
        display_ = nullptr;
    } else {
      QMMF_INFO("%s() Cant destroy.display as there are more client of "
          "display::%u",__func__, client_handlers_.size());
    }
  }

  QMMF_INFO("%s: Exit ", __func__);

  return ret;
}

status_t DisplayService::CreateDisplay(const sp<IDisplayServiceCallback>&
    service_cb, DisplayType display_type, DisplayHandle* display_handle) {

  QMMF_INFO("%s: Enter", __func__);
  sp<RemoteCallBack>           remote_callback;

  remote_callback = new RemoteCallBack(service_cb);
  auto ret = display_->CreateDisplay(remote_callback, display_type,
      display_handle);
  assert(ret == NO_ERROR);

  IInterface::asBinder(remote_callback->getRemoteClient())
      ->linkToDeath(death_notifier_);

  {
    std::lock_guard<std::mutex> lock(remote_callback_lock_);
    remote_callback_.insert({*display_handle, remote_callback});
    for (std::map<DisplayHandle, sp<RemoteCallBack>>::iterator it =
        remote_callback_.begin(); it!=remote_callback_.end(); ++it) {
      QMMF_DEBUG("%s remote_callback DisplayHandle::%u ", __func__, it->first);
    }
  }

  {
    std::lock_guard<std::mutex> lock(client_handlers_lock_);
    client_handlers_.insert({*display_handle,
        remote_callback->getRemoteClient()});
    QMMF_INFO("%s: Added display handle::%d Number of display handle::%d",
        __func__, *display_handle, client_handlers_.size());
  }
  connected_ = true;
  QMMF_INFO("%s: EXIT ", __func__);
  return ret;
}

status_t DisplayService::DestroyDisplay(DisplayHandle display_handle) {

  QMMF_INFO("%s: Enter ", __func__);
  int32_t ret = NO_ERROR;
  if (!connected_)
    return NO_INIT;

  {
    std::lock_guard<std::mutex> lock(remote_callback_lock_);
    auto remote_callback = remote_callback_.find(display_handle);
    if (remote_callback == remote_callback_.end()) {
      QMMF_ERROR("%s() no remote_callback for handle[%d]", __func__,
                 display_handle);
      return -EINVAL;
    }

    for (std::map<DisplayHandle, sp<RemoteCallBack>>::iterator it =
        remote_callback_.begin(); it!=remote_callback_.end(); ++it) {
      QMMF_DEBUG("%s remote_callback DisplayHandle::%u ", __func__, it->first);
    }

    IInterface::asBinder(remote_callback->second->getRemoteClient())
        ->unlinkToDeath(death_notifier_);

     remote_callback_.erase(display_handle);

    if (display_)
    {
      ret = display_->DestroyDisplay(display_handle);
    }
    assert(ret == NO_ERROR);
  }

  {
    std::lock_guard<std::mutex> lock(client_handlers_lock_);
    client_handlers_.erase(display_handle);
    QMMF_INFO("%s: Removed display handle::%d Number of display handle::%d",
        __func__, display_handle, client_handlers_.size());
  }

  QMMF_INFO("%s: Exit ", __func__);

  return ret;
}

status_t DisplayService::CreateSurface(DisplayHandle display_handle,
    SurfaceConfig &surface_config,
    uint32_t* surface_id) {
  QMMF_INFO("%s: Enter ", __func__);

  if(!connected_) {
    QMMF_WARN("%s: Connect Should be called, before calling CreateSurface!!",
        __func__);
    return NO_INIT;
  }
  assert(display_ != NULL);

  auto ret = display_->CreateSurface(display_handle, surface_config,
      surface_id);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s: Can't create surface!!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit", __func__);
  return NO_ERROR;
}

status_t DisplayService::DestroySurface(DisplayHandle display_handle,
    const uint32_t surface_id) {

  QMMF_INFO("%s: Enter ", __func__);

  assert(display_ != NULL);

  QMMF_INFO("%s: surface_id:%d", __func__, surface_id);
  auto ret = display_->DestroySurface(display_handle, surface_id);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s: Can't destroy surface!!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit", __func__);
  return NO_ERROR;
}

status_t DisplayService::DequeueSurfaceBuffer(DisplayHandle display_handle,
    const uint32_t surface_id, SurfaceBuffer &surface_buffer) {

  QMMF_INFO("%s: Enter ", __func__);
  if (!connected_)
    return NO_INIT;

  assert(display_ != NULL);
  auto ret = display_->DequeueSurfaceBuffer(display_handle, surface_id,
      surface_buffer);
  assert(ret == NO_ERROR);
  QMMF_INFO("%s: Exit ", __func__);
  return ret;
}

status_t DisplayService::QueueSurfaceBuffer(DisplayHandle display_handle,
    const uint32_t surface_id, SurfaceBuffer &surface_buffer,
    SurfaceParam &surface_param) {

  QMMF_INFO("%s: Enter ", __func__);
  if (!connected_)
    return NO_INIT;

  assert(display_ != NULL);

  auto ret = display_->QueueSurfaceBuffer(display_handle, surface_id,
      surface_buffer, surface_param);
  assert(ret == NO_ERROR);
  QMMF_INFO("%s: Exit ", __func__);
  return ret;
}

status_t DisplayService::GetDisplayParam(DisplayHandle display_handle,
    DisplayParamType param_type, void *param, size_t param_size) {

  QMMF_INFO("%s: Enter ", __func__);
  if (!connected_)
    return NO_INIT;

  assert(display_ != NULL);

  QMMF_INFO("%s: param_type:%d", __func__, param_type);
  auto ret = display_->GetDisplayParam(display_handle, param_type, param,
      param_size);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: GetDisplayParam failed!", __func__);
  }
  QMMF_INFO("%s: Exit ", __func__);
  return ret;
}

status_t DisplayService::SetDisplayParam(DisplayHandle display_handle,
    DisplayParamType param_type, void *param, size_t param_size) {
  QMMF_INFO("%s: Enter ", __func__);
  if (!connected_)
    return NO_INIT;

  assert(display_ != NULL);
  auto ret = display_->SetDisplayParam(display_handle, param_type, param,
      param_size);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: SetDisplayParam failed!", __func__);
  }
  QMMF_INFO("%s: Exit ", __func__);
  return ret;
}

status_t DisplayService::DequeueWBSurfaceBuffer(DisplayHandle display_handle,
    const uint32_t surface_id, SurfaceBuffer &surface_buffer) {

  QMMF_INFO("%s: Enter ", __func__);
  if (!connected_)
    return NO_INIT;

  assert(display_ != NULL);

  auto ret = display_->DequeueWBSurfaceBuffer(display_handle, surface_id,
      surface_buffer);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: DequeueWBSurfaceBuffer failed!", __func__);
  }
  QMMF_INFO("%s: Exit ", __func__);
  return ret;
}

status_t DisplayService::QueueWBSurfaceBuffer(DisplayHandle display_handle,
    const uint32_t surface_id, const SurfaceBuffer &surface_buffer) {

  QMMF_INFO("%s: Enter ", __func__);
  if (!connected_)
    return NO_INIT;

  assert(display_ != NULL);

  auto ret = display_->QueueWBSurfaceBuffer(display_handle, surface_id,
      surface_buffer);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: QueueWBSurfaceBuffer failed!", __func__);
  }
  QMMF_INFO("%s: Exit ", __func__);
  return ret;
}

}; //namespace display

}; //namespace qmmf
