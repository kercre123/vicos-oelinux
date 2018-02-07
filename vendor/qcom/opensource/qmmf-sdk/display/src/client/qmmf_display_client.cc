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

#define TAG "DisplayClient"
#define BINDERTAG "DisplayServiceBinder"

#include <binder/Parcel.h>
#include <binder/ProcessState.h>
#include <binder/IPCThreadState.h>
#include <linux/msm_ion.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <map>
#include <errno.h>

#include "display/src/client/qmmf_display_client.h"
#include "display/src/service/qmmf_display_common.h"

namespace qmmf {

namespace display {

/**
This file has implementation of following classes:

- DisplayClient    : Delegation to binder proxy <IDisplayService>
                    and implementation of binder CB.
- BpDisplayService : Binder proxy implementation.
- BpDisplayServiceCallback : Binder CB proxy implementation.
- BnDisplayServiceCallback : Binder CB stub implementation.
*/

using namespace android;

DisplayClient::DisplayClient()
    : display_service_(nullptr)
    , death_notifier_(nullptr)
    , ion_device_(-1)
    , display_handle_(-1)
{
  QMMF_INFO("%s:%s Enter ", TAG, __func__);
  sp<ProcessState> proc(ProcessState::self());
  proc->startThreadPool();
  QMMF_INFO("%s:%s Exit (0x%p)", TAG, __func__, this);
}

DisplayClient::~DisplayClient()
{
  QMMF_INFO("%s:%s Enter ", TAG, __func__);

  if (!session_cb_list_.isEmpty()) {
    session_cb_list_.clear();
  }

  if(display_handle_ && display_service_!= nullptr) {
    auto ret = display_service_->DestroyDisplay(display_handle_);
    if(NO_ERROR != ret) {
      QMMF_ERROR("%s:%s DestroyDisplay failed!", TAG, __func__);
    }

    if (context_ == 0) {
      buf_info_map::iterator it;
      for (it=buf_info_map_.begin(); it!=buf_info_map_.end();
          ++it) {
        if (it->second) {
          ret = munmap( it->second->pointer, it->second->frame_len);
          if(ret != 0) {
            QMMF_ERROR("%s:%s munmap Failed!!", TAG, __func__);
          }
          delete (it->second);
        }
        buf_info_map_.erase(it);
      }
      buf_info_map_.clear();
    }
    Disconnect();
  }
  if (display_service_ != nullptr) {
    display_service_.clear();
    display_service_ = nullptr;
  }

  QMMF_INFO("%s:%s Exit 0x%p", TAG, __func__, this);
}

status_t DisplayClient::Connect()
{
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);

  if (checkServiceStatus()) {
    QMMF_WARN("%s:%s Client is already connected to service!", TAG, __func__);
    return NO_ERROR;
  }

  //TODO: close in disconnect.
  ion_device_ = open("/dev/ion", O_RDONLY);
  if (ion_device_ < 0) {
    QMMF_ERROR("%s:%s: Can't open Ion device!", TAG, __func__);
    return NO_INIT;
  }

  death_notifier_ = new DeathNotifier(this);
  if (nullptr == death_notifier_.get()) {
    QMMF_ERROR("%s:%s Unable to allocate death notifier!", TAG, __func__);
    return NO_MEMORY;
  }

  sp<IBinder> service_handle;
  sp<IServiceManager> service_manager = defaultServiceManager();

  service_handle = service_manager->getService
      (String16(QMMF_DISPLAY_SERVICE_NAME));
  if(service_handle.get() == nullptr) {
    QMMF_ERROR("%s:%s Can't get (%s) service", __func__, TAG,
        QMMF_DISPLAY_SERVICE_NAME);
    return NO_INIT;
  }

  display_service_ = interface_cast<IDisplayService>(service_handle);
  IInterface::asBinder(display_service_)->linkToDeath(death_notifier_);

  auto ret = display_service_->Connect();
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s:%s Can't connect to (%s) service", __func__, TAG,
        QMMF_DISPLAY_SERVICE_NAME);
  }

  if (!session_cb_list_.isEmpty()) {
    session_cb_list_.clear();
  }

  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t DisplayClient::Disconnect()
{
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);

  if (!checkServiceStatus()) {
    return NO_INIT;
  }

  auto ret = display_service_->Disconnect();
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s:%s Disconnect failed!", TAG, __func__);
  }

  close(ion_device_);
  display_service_->asBinder(display_service_)->
      unlinkToDeath(death_notifier_);

  display_service_.clear();
  display_service_ = NULL;

  death_notifier_.clear();
  death_notifier_ = NULL;

  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t DisplayClient::CreateDisplay(DisplayType type, DisplayCb& cb)
{
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);

  if (!checkServiceStatus()) {
    return NO_INIT;
  }

  display_cb_ = cb;
  display_type_ = type;

  sp<ServiceCallbackHandler> handler = new ServiceCallbackHandler(this);
  auto ret = display_service_->CreateDisplay(handler, type, &display_handle_);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s:%s CreateDisplay failed!", TAG, __func__);
  }
  display_type_ = type;
  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t DisplayClient::DestroyDisplay(DisplayType type)
{
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);
  int32_t ret = 0;

  if (!checkServiceStatus()) {
    return NO_INIT;
  }

  if(type == display_type_) {
    ret = display_service_->DestroyDisplay(display_handle_);
    if(NO_ERROR != ret) {
      QMMF_ERROR("%s:%s DestroyDisplay failed!", TAG, __func__);
    }
  }

  if (context_ == 0) {
    buf_info_map::iterator it;
    for (it=buf_info_map_.begin(); it!=buf_info_map_.end();
        ++it) {
      if (it->second) {
        ret = munmap( it->second->pointer, it->second->frame_len);
        if(ret != 0) {
          QMMF_ERROR("%s:%s munmap Failed!!", TAG, __func__);
        }
        delete (it->second);
      }
      buf_info_map_.erase(it);
    }
    buf_info_map_.clear();
  }
  display_handle_=-1;
  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t DisplayClient::CreateSurface(SurfaceConfig &surface_config,
    uint32_t* surface_id)
{
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);

  if (!checkServiceStatus()) {
    return NO_INIT;
  }

  auto ret = display_service_->CreateSurface(display_handle_, surface_config,
      surface_id);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s:%s CreateSurface failed!", TAG, __func__);
  }

  context_ = surface_config.context;

  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t DisplayClient::DestroySurface(const uint32_t surface_id)
{
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);

  if (!checkServiceStatus()) {
    return NO_INIT;
  }

  auto ret = display_service_->DestroySurface(display_handle_, surface_id);
  if(NO_ERROR != ret) {
      QMMF_ERROR("%s:%s DestroySurface failed!", TAG, __func__);
  }
  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t DisplayClient::DequeueSurfaceBuffer(const uint32_t surface_id,
    SurfaceBuffer &surface_buffer)
{
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);

  if (!checkServiceStatus()) {
    return NO_INIT;
  }

  auto ret = display_service_->DequeueSurfaceBuffer(display_handle_,
      surface_id, surface_buffer);
  if(NO_ERROR != ret) {
      QMMF_ERROR("%s:%s DequeueSurfaceBuffer failed!", TAG, __func__);
  }

  if (context_ == 0) {
    surface_buffer.plane_info[0].buf = NULL;
    if(surface_buffer.buf_id != -1) {
      auto buf_info_map = buf_info_map_.find(surface_buffer.plane_info[0].ion_fd);
      if (buf_info_map == buf_info_map_.end()) {
        BufInfo *buf_info = new BufInfo();
        buf_info->pointer = nullptr;
        buf_info->frame_len = surface_buffer.plane_info[0].size;
        buf_info_map_.insert({surface_buffer.plane_info[0].ion_fd, buf_info});
      }

      buf_info_map = buf_info_map_.find(surface_buffer.plane_info[0].ion_fd);
      if (buf_info_map != buf_info_map_.end()) {
        if (buf_info_map->second->pointer == NULL) {
          struct ion_fd_data ion_info_fd;
          memset(&ion_info_fd, 0x0, sizeof(ion_info_fd));
          ion_info_fd.fd = surface_buffer.plane_info[0].ion_fd;
          ret = ioctl(ion_device_, ION_IOC_IMPORT, &ion_info_fd);
          if(ret != NO_ERROR) {
            QMMF_ERROR("%s:%s: ION_IOC_IMPORT failed for fd(%d) ret:%d errno:%d",
                TAG, __func__, ion_info_fd.fd, ret, errno);
          }
          void* vaddr = mmap(NULL, (size_t)surface_buffer.capacity,
              PROT_READ | PROT_WRITE, MAP_SHARED, ion_info_fd.fd, 0);
          assert(vaddr != NULL);
          buf_info_map->second->pointer = vaddr;
        }
      }
      surface_buffer.plane_info[0].buf = buf_info_map->second->pointer;
    }
  }
  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t DisplayClient::QueueSurfaceBuffer(const uint32_t surface_id,
    SurfaceBuffer &surface_buffer, SurfaceParam &surface_param) {

  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);
  if (!checkServiceStatus()) {
    return NO_INIT;
  }

  if (context_ == 0) {
    buf_info_map::iterator it;
    for (it=buf_info_map_.begin(); it!=buf_info_map_.end();
        ++it) {
      if (it->second->pointer == surface_buffer.plane_info[0].buf) {
        surface_buffer.plane_info[0].ion_fd = it->first;
        break;
      }
    }

    if (it == buf_info_map_.end()) {
      BufInfo *buf_info = new BufInfo();
      buf_info->pointer = surface_buffer.plane_info[0].buf;
      buf_info->frame_len = surface_buffer.plane_info[0].size;
      buf_info_map_.insert({surface_buffer.plane_info[0].ion_fd, buf_info});
    }
  }

  auto ret = display_service_->QueueSurfaceBuffer(display_handle_, surface_id,
      surface_buffer, surface_param);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s:%s QueueSurfaceBuffer failed!", TAG, __func__);
  }

  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t DisplayClient::GetDisplayParam(DisplayParamType param_type,
    void *param, size_t param_size)
{
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);

  if (!checkServiceStatus()) {
    return NO_INIT;
  }

  auto ret = display_service_->GetDisplayParam(display_handle_, param_type,
      param, param_size);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s:%s GetDisplayParam failed!", TAG, __func__);
  }
  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t DisplayClient::SetDisplayParam(DisplayParamType param_type,
    void *param, size_t param_size)
{
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);

  if (!checkServiceStatus()) {
    return NO_INIT;
  }

  auto ret = display_service_->SetDisplayParam(display_handle_, param_type,
      param, param_size);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s:%s SetDisplayParam failed!", TAG, __func__);
  }
  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t DisplayClient::DequeueWBSurfaceBuffer(const uint32_t surface_id,
        SurfaceBuffer &surface_buffer)
{
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);

  if (!checkServiceStatus()) {
    return NO_INIT;
  }

  auto ret = display_service_->DequeueWBSurfaceBuffer(display_handle_,
      surface_id, surface_buffer);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s:%s DequeueWBSurfaceBuffer failed!", TAG, __func__);
  }
  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
  return ret;
}

status_t DisplayClient::QueueWBSurfaceBuffer(const uint32_t surface_id,
        const SurfaceBuffer &surface_buffer)
{
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  Mutex::Autolock lock(lock_);

  if (!checkServiceStatus()) {
    return NO_INIT;
  }

  auto ret = display_service_->QueueWBSurfaceBuffer(display_handle_,
      surface_id, surface_buffer);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s:%s QueueWBSurfaceBuffer failed!", TAG, __func__);
  }
  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
  return ret;
}

bool DisplayClient::checkServiceStatus() {

  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  bool connected = true;
  if (NULL == display_service_.get()) {
    QMMF_WARN("%s:%s Not connected to display service!", TAG, __func__);
    connected = false;
  }
  return connected;
}
void DisplayClient::notifyDisplayEvent(DisplayEventType event_type,
    void *event_data, size_t event_data_size) {
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
}

void DisplayClient::notifySessionEvent(DisplayEventType event_type,
    void *event_data, size_t event_data_size) {
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
}

void DisplayClient::notifyVSyncEvent(int64_t time_stamp) {
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  display_cb_.VSyncCb(time_stamp);
  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
}


//Binder Proxy implementation of IDisplayService.
class BpDisplayService: public BpInterface<IDisplayService>
{
public:
  BpDisplayService(const sp<IBinder>& impl)
  : BpInterface<IDisplayService>(impl) {}

  status_t Connect()
  {
    Parcel data, reply;
    QMMF_LEVEL1("%s:%s Enter ", BINDERTAG, __func__);
    data.writeInterfaceToken(IDisplayService::getInterfaceDescriptor());
    remote()->transact(uint32_t(QMMF_DISPLAY_SERVICE_CMDS::
                            DISPLAY_CONNECT), data, &reply);

    QMMF_LEVEL1("%s:%s Exit ", BINDERTAG, __func__);
    return reply.readInt32();
  }

  status_t Disconnect()
  {
    Parcel data, reply;
    QMMF_LEVEL1("%s:%s Enter ", BINDERTAG, __func__);
    data.writeInterfaceToken(IDisplayService::getInterfaceDescriptor());
    remote()->transact(uint32_t(QMMF_DISPLAY_SERVICE_CMDS::
        DISPLAY_DISCONNECT), data, &reply);

    QMMF_LEVEL1("%s:%s Exit ", BINDERTAG, __func__);
    return reply.readInt32();
  }

  status_t CreateDisplay(const sp<IDisplayServiceCallback>& service_cb,
      DisplayType display_type, DisplayHandle* display_handle)
  {
    Parcel data, reply;
    QMMF_LEVEL1("%s:%s Enter ", BINDERTAG, __func__);
    data.writeInterfaceToken(IDisplayService::getInterfaceDescriptor());
    data.writeStrongBinder(IInterface::asBinder(service_cb));
    data.writeInt32(static_cast<int32_t>(display_type));
    remote()->transact(uint32_t(QMMF_DISPLAY_SERVICE_CMDS::
                            DISPLAY_CREATE_DISPLAY), data, &reply);
    *display_handle = static_cast<DisplayHandle>(reply.readInt32());

    QMMF_LEVEL1("%s:%s Exit ", BINDERTAG, __func__);
    return reply.readInt32();
  }

  status_t DestroyDisplay(DisplayHandle display_handle)
  {
    Parcel data, reply;
    QMMF_LEVEL1("%s:%s Enter ", BINDERTAG, __func__);
    data.writeInterfaceToken(IDisplayService::getInterfaceDescriptor());
    data.writeInt32(static_cast<int32_t>(display_handle));
    remote()->transact(uint32_t(QMMF_DISPLAY_SERVICE_CMDS::
        DISPLAY_DESTROY_DISPLAY), data, &reply);

    ion_fd_map::iterator it_fd;
    for (it_fd=ion_fd_mapping.begin();
        it_fd!=ion_fd_mapping.end(); ++it_fd) {
      ion_fd_mapping.erase(it_fd);
    }
    ion_fd_mapping.clear();
    use_buffer_mapping.clear();

    QMMF_LEVEL1("%s:%s Exit ", BINDERTAG, __func__);
    return reply.readInt32();
  }

  status_t CreateSurface(DisplayHandle display_handle,
      SurfaceConfig &surface_config, uint32_t* surface_id)
  {
    Parcel data, reply;
    data.writeInterfaceToken(IDisplayService::getInterfaceDescriptor());
    data.writeInt32(static_cast<int32_t>(display_handle));

    uint32_t param_size = sizeof surface_config;
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memset(blob.data(), 0x0, param_size);
    memcpy(blob.data(), reinterpret_cast<void*>(&surface_config), param_size);
    remote()->transact(uint32_t(QMMF_DISPLAY_SERVICE_CMDS::
        DISPLAY_CREATE_SURFACE), data, &reply);

    uint32_t id;
    reply.readUint32(&id);
    *surface_id = id;
    use_buffer_mapping.insert({*surface_id, surface_config.use_buffer});

    assert(id != 0);
    return reply.readInt32();
  }

  status_t DestroySurface(DisplayHandle display_handle,
      const uint32_t surface_id)
  {
    Parcel data, reply;
    data.writeInterfaceToken(IDisplayService::getInterfaceDescriptor());
    data.writeInt32(static_cast<int32_t>(display_handle));

    assert(surface_id != 0);
    data.writeUint32(surface_id);
    remote()->transact(uint32_t(QMMF_DISPLAY_SERVICE_CMDS::
        DISPLAY_DESTROY_SURFACE), data, &reply);
    for (use_buffer_map::iterator it = use_buffer_mapping.begin();
        it != use_buffer_mapping.end(); ++it) {
      if (it->first == surface_id) {
        use_buffer_mapping.erase(surface_id);
        break;
      }
    }
    return reply.readInt32();
  }
  status_t DequeueSurfaceBuffer(DisplayHandle display_handle,
      const uint32_t surface_id, SurfaceBuffer &surface_buffer)
  {
    Parcel data, reply;
    data.writeInterfaceToken(IDisplayService::getInterfaceDescriptor());
    data.writeInt32(static_cast<int32_t>(display_handle));
    assert(surface_id != 0);
    data.writeUint32(surface_id);

    remote()->transact(uint32_t(QMMF_DISPLAY_SERVICE_CMDS::
        DISPLAY_DEQUEUE_SURFACE_BUFFER), data, &reply);
    auto ret = reply.readInt32();
    if (NO_ERROR == ret) {
      uint32_t param_size;
      int32_t fd, ion_fd;
      reply.readUint32(&param_size);
      android::Parcel::ReadableBlob blob;
      reply.readBlob(param_size, &blob);
      memcpy((void *)&surface_buffer, blob.data(), param_size);
      if(surface_buffer.buf_id != -1) {
        reply.readInt32(&fd);
        if (fd != 0) {
          for (use_buffer_map::iterator it = use_buffer_mapping.begin();
              it != use_buffer_mapping.end(); ++it) {
            if (it->first == surface_id) {
              if (it->second == 1) {
                surface_buffer.plane_info[0].ion_fd = fd;
              }
              else {
                auto ion_fd_map_ = ion_fd_mapping.find(fd);
                if (ion_fd_map_->second == -1) {
                  surface_buffer.plane_info[0].ion_fd =
                      dup(reply.readFileDescriptor());
                }
                else {
                  surface_buffer.plane_info[0].ion_fd = ion_fd_map_->second;
                }
              }
              break;
            }
          }
        }
        else {
          reply.readInt32(&ion_fd);
          surface_buffer.plane_info[0].ion_fd = dup(reply.readFileDescriptor());
          ion_fd_mapping.insert({ion_fd, surface_buffer.plane_info[0].ion_fd});
        }
      }
      blob.release();
    }
    return reply.readInt32();
  }

  status_t QueueSurfaceBuffer(DisplayHandle display_handle,
      const uint32_t surface_id, SurfaceBuffer &surface_buffer,
      SurfaceParam &surface_param)
  {
    Parcel data, reply;
    data.writeInterfaceToken(IDisplayService::getInterfaceDescriptor());
    data.writeInt32(static_cast<int32_t>(display_handle));
    assert(surface_id != 0);
    data.writeUint32(surface_id);

    ion_fd_map::iterator it_fd;
    for (it_fd=ion_fd_mapping.begin();
    it_fd!=ion_fd_mapping.end(); ++it_fd) {
      if (it_fd->second == surface_buffer.plane_info[0].ion_fd) {
        surface_buffer.plane_info[0].ion_fd = it_fd->first;
        break;
      }
    }
    uint32_t buffer_size = sizeof surface_buffer;
    uint32_t param_size = sizeof surface_param;

    data.writeUint32(buffer_size);
    android::Parcel::WritableBlob blob1;
    data.writeBlob(buffer_size, false, &blob1);
    memset(blob1.data(), 0x0, buffer_size);
    memcpy(blob1.data(), reinterpret_cast<void*>(&surface_buffer), buffer_size);
    android::Parcel::WritableBlob blob2;

    data.writeUint32(param_size);
    data.writeBlob(param_size, false, &blob2);
    memset(blob2.data(), 0x0, param_size);

    memcpy(blob2.data(), reinterpret_cast<void*>(&surface_param), param_size);
    if (it_fd==ion_fd_mapping.end()) {
      for (use_buffer_map::iterator it = use_buffer_mapping.begin();
          it != use_buffer_mapping.end(); ++it) {
        if (it->first == surface_id && it->second == 1) {
          data.writeFileDescriptor(surface_buffer.plane_info[0].ion_fd);
        }
        break;
      }
    }
    remote()->transact(uint32_t(QMMF_DISPLAY_SERVICE_CMDS::
        DISPLAY_QUEUE_SURFACE_BUFFER), data, &reply);

    return reply.readInt32();
  }

  status_t GetDisplayParam(DisplayHandle display_handle,
      DisplayParamType param_type, void *param, size_t param_size)
  {
    Parcel data, reply;
    data.writeInterfaceToken(IDisplayService::getInterfaceDescriptor());
    data.writeInt32(static_cast<int32_t>(display_handle));
    data.writeUint32((uint32_t)param_type);
    data.writeInt32(param_size);

    remote()->transact(uint32_t(QMMF_DISPLAY_SERVICE_CMDS::
        DISPLAY_GET_DISPLAY_PARAM), data, &reply);
    auto ret = reply.readInt32();
    if (NO_ERROR == ret) {
      uint32_t param_size;
      reply.readUint32(&param_size);
      android::Parcel::ReadableBlob blob;
      reply.readBlob(param_size, &blob);
      memcpy(param, blob.data(), param_size);
      blob.release();
    }
    return ret;
  }

  status_t SetDisplayParam(DisplayHandle display_handle,
      DisplayParamType param_type, void *param, size_t param_size)
  {
    Parcel data, reply;
    data.writeInterfaceToken(IDisplayService::getInterfaceDescriptor());
    data.writeInt32(static_cast<int32_t>(display_handle));
    data.writeUint32((uint32_t)param_type);
    data.writeInt32(param_size);

    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memset(blob.data(), 0x0, param_size);
    memcpy(blob.data(), reinterpret_cast<void*>(param), param_size);
    remote()->transact(uint32_t(QMMF_DISPLAY_SERVICE_CMDS::
        DISPLAY_SET_DISPLAY_PARAM), data, &reply);
    return reply.readInt32();
  }

  status_t DequeueWBSurfaceBuffer(DisplayHandle display_handle,
      const uint32_t surface_id, SurfaceBuffer &surface_buffer)
  {
    Parcel data, reply;
    data.writeInterfaceToken(IDisplayService::getInterfaceDescriptor());
    data.writeInt32(static_cast<int32_t>(display_handle));
    assert(surface_id != 0);
    data.writeUint32(surface_id);
    remote()->transact(uint32_t(QMMF_DISPLAY_SERVICE_CMDS::
        DISPLAY_DEQUEUE_WBSURFACE_BUFFER), data, &reply);
    auto ret = reply.readInt32();
    if (NO_ERROR == ret) {
      uint32_t param_size;
      reply.readUint32(&param_size);
      android::Parcel::ReadableBlob blob;
      reply.readBlob(param_size, &blob);
      memcpy((void *)&surface_buffer, blob.data(), param_size);
      blob.release();
    }
    return reply.readInt32();
  }

  status_t QueueWBSurfaceBuffer(DisplayHandle display_handle,
      const uint32_t surface_id, const SurfaceBuffer &surface_buffer)
  {
    Parcel data, reply;
    data.writeInterfaceToken(IDisplayService::getInterfaceDescriptor());
    data.writeInt32(static_cast<int32_t>(display_handle));
    assert(surface_id != 0);
    data.writeUint32(surface_id);

    uint32_t param_size = sizeof surface_buffer;
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memset(blob.data(), 0x0, param_size);
    memcpy(blob.data(), reinterpret_cast<const void*>(&surface_buffer),
       param_size);
    remote()->transact(uint32_t(QMMF_DISPLAY_SERVICE_CMDS::
        DISPLAY_QUEUE_WBSURFACE_BUFFER), data, &reply);
    return reply.readInt32();
  }

  ion_fd_map ion_fd_mapping;
  use_buffer_map use_buffer_mapping;

};

IMPLEMENT_META_INTERFACE(DisplayService, QMMF_DISPLAY_SERVICE_NAME);

ServiceCallbackHandler::ServiceCallbackHandler(DisplayClient* client)
    : client_(client) {
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
}

ServiceCallbackHandler::~ServiceCallbackHandler() {
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
}

void ServiceCallbackHandler::notifyDisplayEvent(DisplayEventType event_type,
    void *event_data, size_t event_data_size) {
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
}

void ServiceCallbackHandler::notifySessionEvent(DisplayEventType event_type,
    void *event_data, size_t event_data_size) {
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
}

void ServiceCallbackHandler::notifyVSyncEvent(int64_t time_stamp) {
  QMMF_LEVEL1("%s:%s Enter ", TAG, __func__);
  assert(client_ != NULL);
  client_->notifyVSyncEvent(time_stamp);
  QMMF_LEVEL1("%s:%s Exit ", TAG, __func__);
}


class BpDisplayServiceCallback: public BpInterface<IDisplayServiceCallback> {
 public:
  BpDisplayServiceCallback(const sp<IBinder>& impl)
      : BpInterface<IDisplayServiceCallback>(impl) {}

  void notifyDisplayEvent(DisplayEventType event_type, void *event_data,
      size_t event_data_size) {

  }

  void notifySessionEvent(DisplayEventType event_type, void *event_data,
      size_t event_data_size) {

  }

  void notifyVSyncEvent(int64_t time_stamp) {

    Parcel data, reply;
    data.writeInterfaceToken(IDisplayServiceCallback::getInterfaceDescriptor());
    data.writeInt64(time_stamp);
    remote()->transact(uint32_t(DISPLAY_SERVICE_CB_CMDS::
        DISPLAY_NOTIFY_VSYNC_EVENT), data, &reply);
  }

 private:
  std::map<uint32_t , std::map<uint32_t , int32_t> > track_buf_map_;
  std::map<uint32_t , std::map<uint32_t , int32_t> >::iterator track_buf_iter_;
};

IMPLEMENT_META_INTERFACE(DisplayServiceCallback,
    "display.service.IDisplayServiceCallback");

status_t BnDisplayServiceCallback::onTransact(uint32_t code,
                                               const Parcel& data,
                                               Parcel* reply,
                                               uint32_t flags) {

  QMMF_LEVEL2("%s:%s: Enter:(BnDisplayServiceCallback::onTransact)", TAG,
      __func__);
  CHECK_INTERFACE(IDisplayServiceCallback, data, reply);

  switch(code) {
    case DISPLAY_SERVICE_CB_CMDS::DISPLAY_NOTIFY_EVENT: {
      //TODO:
      return NO_ERROR;
    }
    break;
    case DISPLAY_SERVICE_CB_CMDS::DISPLAY_NOTIFY_SESSION_EVENT: {
      //TODO:
      return NO_ERROR;
    }

    case DISPLAY_SERVICE_CB_CMDS::DISPLAY_NOTIFY_VSYNC_EVENT: {
      int64_t time_stamp;
      data.readInt64(&time_stamp);
      notifyVSyncEvent(time_stamp);
      return NO_ERROR;
    }
    break;
    default: {
      QMMF_ERROR("%s:%s Method not supported ", TAG, __func__);
    }
    break;
  }
  return NO_ERROR;
}

}; //namespace qmmf

}; //namespace display
