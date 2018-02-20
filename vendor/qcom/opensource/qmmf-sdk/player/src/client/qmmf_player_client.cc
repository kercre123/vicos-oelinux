/*
* Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "PlayerClient"

#include <binder/Parcel.h>
#include <binder/ProcessState.h>
#include <binder/IPCThreadState.h>
#include <linux/msm_ion.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <map>
#include <algorithm>


#include "player/src/client/qmmf_player_client.h"
#include "player/src/client/qmmf_player_service_intf.h"

uint32_t qmmf_log_level;

namespace qmmf {
namespace player {

/**
This file has implementation of following classes:

- PlayerClient    : Delegation to binder proxy <IPlayerService>
                    and implementation of binder CB.
- BpPlayerService : Binder proxy implementation.
- BpPlayerServiceCallback : Binder CB proxy implementation.
- BnPlayerServiceCallback : Binder CB stub implementation.
*/

using namespace android;


PlayerClient::PlayerClient()
    : player_service_(nullptr), death_notifier_(nullptr),
      ion_device_(-1) {
  QMMF_GET_LOG_LEVEL();
  QMMF_INFO("%s Enter ", __func__);
  sp<ProcessState> proc(ProcessState::self());
  proc->startThreadPool();
  QMMF_INFO("%s Exit (0x%p)", __func__, this);
}

PlayerClient::~PlayerClient() {
  QMMF_DEBUG("%s Enter ", __func__);
  if (player_service_ != nullptr) {
    player_service_.clear();
    player_service_ = nullptr;
  }
  QMMF_DEBUG("%s Exit 0x%p", __func__, this);
}

status_t PlayerClient::Connect(PlayerCb& cb) {
  QMMF_DEBUG("%s Enter ", __func__);
  Mutex::Autolock lock(lock_);

  if (CheckServiceStatus()) {
    QMMF_WARN("%s Client is already connected to service!", __func__);
    return NO_ERROR;
  }

  //TODO: close in disconnect.
  ion_device_ = open("/dev/ion", O_RDONLY);
  if (ion_device_ < 0) {
    QMMF_ERROR("%s: Can't open Ion device!", __func__);
    return NO_INIT;
  }

  player_cb_= cb;

  death_notifier_ = new DeathNotifier(this);
  if (nullptr == death_notifier_.get()) {
    QMMF_ERROR("%s Unable to allocate death notifier!", __func__);
    return NO_MEMORY;
  }

  sp<IBinder> service_handle;
  sp<IServiceManager> service_manager = defaultServiceManager();

  service_handle = service_manager->getService(String16(QMMF_PLAYER_SERVICE_NAME));
  if(service_handle.get() == nullptr) {
    QMMF_ERROR("%s Can't get (%s) service", __func__,
        QMMF_PLAYER_SERVICE_NAME);
   return NO_INIT;
  }

  player_service_ = interface_cast<IPlayerService>(service_handle);
  IInterface::asBinder(player_service_)->linkToDeath(death_notifier_);

  sp<ServiceCallbackHandler> handler = new ServiceCallbackHandler(this);
  auto ret = player_service_->Connect(handler);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s Can't connect to (%s) service", __func__,
        QMMF_PLAYER_SERVICE_NAME);
  }

  if (!track_cb_list_.isEmpty())
    track_cb_list_.clear();

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t PlayerClient::Disconnect() {
  QMMF_DEBUG("%s Enter ", __func__);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  auto ret = player_service_->Disconnect();
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s Disconnect failed!", __func__);
  }

  player_service_->asBinder(player_service_)->unlinkToDeath(death_notifier_);

  player_service_.clear();
  player_service_ = nullptr;

  death_notifier_.clear();
  death_notifier_ = nullptr;

  if (!track_cb_list_.isEmpty()) {
    track_cb_list_.clear();
  }

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t PlayerClient::CreateAudioTrack(uint32_t track_id,
                                        AudioTrackCreateParam& param,
                                        TrackCb& cb) {
  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_VERBOSE("%s INPARAM: track_id[%u]", __func__, track_id);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  assert(track_id != 0);

  auto result = player_service_->CreateAudioTrack(track_id, param);
  if (result != NO_ERROR)
    QMMF_ERROR("%s CreateAudioTrack failed: %d", __func__, result);

  track_cb_list_.add(track_id, cb);

  QMMF_DEBUG("%s Exit ", __func__);
  return result;
}

status_t PlayerClient::CreateVideoTrack(uint32_t track_id,
                                        VideoTrackCreateParam& param,
                                        TrackCb& cb) {
  QMMF_DEBUG("%s Enter ", __func__);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  assert(track_id != 0);

  auto result = player_service_->CreateVideoTrack(track_id, param);
  if (result != NO_ERROR)
    QMMF_ERROR("%s CreateVideoTrack failed: %d", __func__, result);

  track_cb_list_.add(track_id, cb);

  QMMF_DEBUG("%s Exit ", __func__);
  return result;
}

status_t PlayerClient::DeleteAudioTrack(uint32_t track_id) {
  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_VERBOSE("%s INPARAM: track_id[%u]", __func__, track_id);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  if (track_buf_map_.indexOfKey(track_id) >= 0) {
    buf_info_map info_map = track_buf_map_.valueFor(track_id);
    for (size_t j = 0; j < info_map.size(); j++) {
      BufInfo buf_info = info_map.valueAt(j);
      QMMF_INFO("%s: track_id(%d):buf_info.client_fd(%d) to close",
          __func__, track_id, buf_info.client_fd);
      if (buf_info.vaddr != nullptr) {
        struct ion_handle_data ion_handle;
        memset(&ion_handle, 0, sizeof(ion_handle));
        ion_handle.handle = buf_info.ion_handle;
        if (ioctl(ion_device_, ION_IOC_FREE, &ion_handle) < 0) {
          QMMF_ERROR("%s ION free failed: %d", __func__, -errno);
        }

        QMMF_INFO("%s: track_id(%d):buf_info.vaddr=0x%p and frame_len=%d",
            __func__, track_id, buf_info.vaddr, buf_info.frame_len);
        if (buf_info.vaddr != nullptr) {
          munmap(buf_info.vaddr, buf_info.frame_len);
          buf_info.vaddr = nullptr;
        }
      }

      if (buf_info.client_fd > 0) {
        auto stat = close(buf_info.client_fd);
        if (0 != stat) {
          QMMF_ERROR("%s Failed to close ION fd: %d : %d", __func__,
              buf_info.client_fd, -errno);
        }
      }
    }
    track_buf_map_.removeItem(track_id);
  }

  auto ret = player_service_->DeleteAudioTrack(track_id);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s DeleteAudioTrack failed: %d", __func__, ret);
  }

  if (track_cb_list_.indexOfKey(track_id) >= 0) {
    track_cb_list_.removeItem(track_id);
  }

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t PlayerClient::DeleteVideoTrack(uint32_t track_id) {
  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_VERBOSE("%s INPARAM: track_id[%u]", __func__, track_id);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  if (track_buf_map_.indexOfKey(track_id) >= 0) {
    buf_info_map info_map = track_buf_map_.valueFor(track_id);
    for (size_t j = 0; j < info_map.size(); j++) {
      BufInfo buf_info = info_map.valueAt(j);
      QMMF_INFO("%s: track_id(%d):buf_info.client_fd(%d) to close",
          __func__, track_id, buf_info.client_fd);

      if (buf_info.vaddr != nullptr) {
        struct ion_handle_data ion_handle;
        memset(&ion_handle, 0, sizeof(ion_handle));
        ion_handle.handle = buf_info.ion_handle;
        if (ioctl(ion_device_, ION_IOC_FREE, &ion_handle) < 0) {
          QMMF_ERROR("%s ION free failed: %d", __func__, -errno);
        }

        QMMF_INFO("%s: track_id(%d):buf_info.vaddr=0x%p and frame_len=%d",
            __func__, track_id, buf_info.vaddr, buf_info.frame_len);
        if (buf_info.vaddr != nullptr) {
          munmap(buf_info.vaddr, buf_info.frame_len);
          buf_info.vaddr = nullptr;
        }
      }

      if (buf_info.client_fd > 0) {
        auto stat = close(buf_info.client_fd);
        if (0 != stat) {
          QMMF_ERROR("%s Failed to close ION fd: %d : %d", __func__,
              buf_info.client_fd, -errno);
        }
      }
    }
    track_buf_map_.removeItem(track_id);
  }

  auto ret = player_service_->DeleteVideoTrack(track_id);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s DeleteVideoTrack failed: %d", __func__, ret);
  }

  if (track_cb_list_.indexOfKey(track_id) >= 0) {
    track_cb_list_.removeItem(track_id);
  }

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t PlayerClient::Prepare() {
  QMMF_DEBUG("%s Enter ", __func__);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  auto ret = player_service_->Prepare();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s Prepare failed: %d", __func__, ret);
  }

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t PlayerClient::DequeueInputBuffer(
    uint32_t track_id,
    std::vector<TrackBuffer>& buffers) {
  QMMF_DEBUG("%s Enter ", __func__);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  int32_t size = buffers.size();
  void* vaddr = nullptr;

  std::vector<AVCodecBuffer> codecbuffer;
  AVCodecBuffer cb;

  QMMF_DEBUG("%s size is %d", __func__,size);

  for (int32_t i = 0; i < size; i++) {
    memset(&cb,0x0,sizeof(cb));
    codecbuffer.push_back(cb);
  }

  auto ret = player_service_->DequeueInputBuffer(track_id, codecbuffer);

  if (codecbuffer.size() == 0) {
    buffers.clear();
  } else {
    for (size_t i = 0; i < codecbuffer.size(); i++) {

    BufInfo bufinfo;
    memset(&bufinfo, 0x0, sizeof bufinfo);
    bool is_mapped = false;

    if (!track_buf_map_.isEmpty()) {
      DefaultKeyedVector<uint32_t, BufInfo> buf_map;
      int32_t map_idx = track_buf_map_.indexOfKey(track_id);

      if (map_idx >= 0) {
        buf_map = track_buf_map_.valueFor(track_id);
        int32_t buf_idx = buf_map.indexOfKey(codecbuffer[i].buf_id);

        if (buf_idx >= 0) {
          bufinfo = buf_map.valueFor(codecbuffer[i].buf_id);
          assert(bufinfo.buf_id > 0);
          buffers[i].data   = bufinfo.vaddr;
          buffers[i].size   = codecbuffer[i].frame_length;
          buffers[i].buf_id = codecbuffer[i].buf_id;
          is_mapped = true;
          QMMF_VERBOSE("%s: Buf is already mapped! ion_fd(%d):"
            "vaddr(0x%p)",  __func__, bufinfo.buf_id, bufinfo.vaddr);
        }
      }
    }

    if (!is_mapped) {
      // Map Ion Fd to client address space.
      assert(ion_device_ > 0);
      struct ion_fd_data ion_info_fd;
      memset(&ion_info_fd, 0x0, sizeof(ion_info_fd));

      assert(codecbuffer[i].fd > 0);

      ion_info_fd.fd = codecbuffer[i].fd;
      ret = ioctl(ion_device_, ION_IOC_IMPORT, &ion_info_fd);
      if(ret != NO_ERROR) {
        QMMF_ERROR("%s: ION_IOC_IMPORT failed for fd(%d)", __func__,
            ion_info_fd.fd);
      }
      QMMF_VERBOSE("%s: ion_info_fd.fd =%d", __func__, ion_info_fd.fd);
      vaddr = mmap(nullptr, codecbuffer[i].frame_length, PROT_READ | PROT_WRITE,
                   MAP_SHARED, ion_info_fd.fd, 0);
      assert(vaddr != nullptr);

      bufinfo.vaddr      = vaddr;
      bufinfo.buf_id     = codecbuffer[i].buf_id;
      bufinfo.client_fd  = codecbuffer[i].fd;
      bufinfo.frame_len  = codecbuffer[i].frame_length;
      bufinfo.ion_handle = ion_info_fd.handle;

       DefaultKeyedVector<uint32_t, BufInfo> buffer_map;
       if (track_buf_map_.isEmpty()) {
         buffer_map.add(codecbuffer[i].buf_id, bufinfo);
       } else {
         buffer_map = track_buf_map_.valueFor(track_id);
         buffer_map.add(codecbuffer[i].buf_id, bufinfo);
       }

       track_buf_map_.replaceValueFor(track_id, buffer_map);

       QMMF_VERBOSE("%s: track_buf_map_.size = %d", __func__,
           track_buf_map_.size());

       for (uint32_t i = 0; i < track_buf_map_.size(); i++) {
         buffer_map = track_buf_map_[i];
         QMMF_VERBOSE("%s: track_id : %d buffer_map.size=%d", __func__,
             track_buf_map_.keyAt(i), buffer_map.size());

         for(uint32_t j = 0; j < buffer_map.size(); j++) {
           QMMF_VERBOSE("%s: buffer_map:idx(%d) :key(%d) :fd:%d :data:"
               "0x%p",  __func__, j, buffer_map.keyAt(j), buffer_map[j].buf_id,
               buffer_map[j].vaddr);
         }
       }
         buffers[i].data   = vaddr;
         buffers[i].size   = codecbuffer[i].frame_length;
         buffers[i].buf_id = codecbuffer[i].buf_id;
       }

       QMMF_DEBUG("%s vaddr 0x%p", __func__,buffers[i].data);
       QMMF_DEBUG("%s size %d", __func__,buffers[i].size);
       QMMF_DEBUG("%s buf_id %d", __func__,buffers[i].buf_id);
    }

    if (NO_ERROR != ret) {
      QMMF_ERROR("%s DequeueInputBuffer failed: %d", __func__, ret);
    }

    for (int32_t i = 0; i < size; i++) {
      codecbuffer.clear();
    }
  }

  QMMF_VERBOSE("%s: buffers.size()=%d", __func__, buffers.size());
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t PlayerClient::QueueInputBuffer(uint32_t track_id,
                                        std::vector<TrackBuffer>& buffers,
                                        void *meta_param,
                                        size_t meta_size,
                                        TrackMetaBufferType meta_type) {
  QMMF_DEBUG("%s Enter ", __func__);

  auto ret = 0;

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  int32_t size = buffers.size();

  for (auto i =0; i< size; i++) {

    std::vector<AVCodecBuffer> codecbuffer;
    AVCodecBuffer cb;

    BufInfo bufinfo;
    memset(&bufinfo, 0x0, sizeof bufinfo);

    DefaultKeyedVector<uint32_t, BufInfo> buf_map;

    buf_map = track_buf_map_.valueFor(track_id);

    QMMF_DEBUG("%s track_id : %d", __func__, track_id);

    bufinfo = buf_map.valueFor(buffers[i].buf_id);

    QMMF_DEBUG("%s vaddr : 0x%p", __func__,bufinfo.vaddr);
    QMMF_DEBUG("%s data : 0x%p", __func__, buffers[i].data);

    if (bufinfo.vaddr == buffers[i].data) {

      memset(&cb,0x0,sizeof(cb));

      cb.time_stamp     = buffers[i].time_stamp;
      cb.flag           = buffers[i].flag;
      cb.frame_length   = buffers[i].size;
      cb.filled_length  = buffers[i].filled_size;
      cb.buf_id         = bufinfo.buf_id;

      codecbuffer.push_back(cb);

     ret = player_service_->QueueInputBuffer(track_id, codecbuffer,
         meta_param,meta_size, meta_type);

    if (NO_ERROR != ret) {
      QMMF_ERROR("%s QueueInputBuffer failed: %d", __func__, ret);
    }

    codecbuffer.clear();
  }
 }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t PlayerClient::Start() {
  QMMF_DEBUG("%s Enter ", __func__);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  picture_cb_.event_cb = nullptr;
  picture_cb_.data_cb = nullptr;

  auto ret = player_service_->Start();
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s Start failed: %d", __func__, ret);
  }

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t PlayerClient::Stop(const PictureCallback& handler,
                            const PictureParam& params) {
  QMMF_DEBUG("%s Enter ", __func__);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  if (params.enable)
    picture_cb_ = handler;

  auto ret = player_service_->Stop(params);
  if(NO_ERROR != ret) {
      QMMF_ERROR("%s Stop failed: %d", __func__, ret);
  }

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t PlayerClient::Pause(const PictureCallback& handler,
                             const PictureParam& params) {
  QMMF_DEBUG("%s Enter ", __func__);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  if (params.enable)
    picture_cb_ = handler;

  auto ret = player_service_->Pause(params);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s Pause failed: %d", __func__, ret);
  }

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t PlayerClient::Resume() {
  QMMF_DEBUG("%s Enter ", __func__);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  picture_cb_.event_cb = nullptr;
  picture_cb_.data_cb = nullptr;

  auto ret = player_service_->Resume();
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s Resume failed: %d", __func__, ret);
  }

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t PlayerClient::SetPosition(int64_t seek_time) {
  QMMF_DEBUG("%s Enter ", __func__);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  auto ret = player_service_->SetPosition(seek_time);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s Start failed: %d", __func__, ret);
  }

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t PlayerClient::SetTrickMode(TrickModeSpeed speed,
                                    TrickModeDirection dir) {
  QMMF_DEBUG("%s Enter ", __func__);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  auto ret = player_service_->SetTrickMode(speed, dir);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s SetTrickMode failed: %d", __func__, ret);
  }

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t PlayerClient::SetAudioTrackParam(uint32_t track_id,
                                          CodecParamType type,
                                          void *param,
                                          size_t param_size) {
  QMMF_DEBUG("%s Enter ", __func__);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  auto ret = player_service_->SetAudioTrackParam(track_id,
                                                 type, param, param_size);

  if (NO_ERROR != ret) {
    QMMF_ERROR("%s SetAudioTrackParam failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t PlayerClient::SetVideoTrackParam(uint32_t track_id,
                                          CodecParamType type,
                                          void *param,
                                          size_t param_size) {
  QMMF_DEBUG("%s Enter ", __func__);
  Mutex::Autolock lock(lock_);

  if (!CheckServiceStatus()) {
    return NO_INIT;
  }

  auto ret = player_service_->SetVideoTrackParam(track_id,
                                                 type, param, param_size);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s SetAudioTrackParam failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}


bool PlayerClient::CheckServiceStatus() {
  QMMF_DEBUG("%s Enter ", __func__);
  bool connected = true;
  if (nullptr == player_service_.get()) {
    QMMF_WARN("%s Not connected to Player service!", __func__);
    connected = false;
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return connected;
}

void PlayerClient::NotifyPlayerEvent(EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) {
  QMMF_DEBUG("%s Enter ", __func__);
  player_cb_.event_cb(event_type,event_data,event_data_size);
  QMMF_DEBUG("%s Exit ", __func__);
}

void PlayerClient::NotifyVideoTrackEvent(uint32_t track_id,
                                         EventType event_type,
                                         void *event_data,
                                         size_t event_data_size) {
  QMMF_DEBUG("%s Enter ", __func__);
  if (track_cb_list_.indexOfKey(track_id) >= 0)
    track_cb_list_.valueFor(track_id).event_cb(track_id, event_type, event_data,
                                               event_data_size);
  QMMF_DEBUG("%s Exit ", __func__);
}

void PlayerClient::NotifyAudioTrackEvent(uint32_t track_id,
                                         EventType event_type,
                                         void *event_data,
                                         size_t event_data_size) {
  QMMF_DEBUG("%s Enter ", __func__);
  if (track_cb_list_.indexOfKey(track_id) >= 0)
    track_cb_list_.valueFor(track_id).event_cb(track_id, event_type, event_data,
                                               event_data_size);
  QMMF_DEBUG("%s Exit ", __func__);
}

void PlayerClient::NotifyGrabPictureData(uint32_t track_id,
                                         BufferDescriptor& buffer) {
  QMMF_DEBUG("%s Enter ", __func__);

  void* vaddr = nullptr;
  struct ion_fd_data ion_info_fd;
  memset(&ion_info_fd, 0x0, sizeof(ion_info_fd));

  ion_info_fd.fd = buffer.fd;
  auto ret = ioctl(ion_device_, ION_IOC_IMPORT, &ion_info_fd);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s: ION_IOC_IMPORT failed for fd(%d)", __func__,
        ion_info_fd.fd);
  }

  QMMF_VERBOSE("%s: ion_info_fd.fd =%d", __func__, ion_info_fd.fd);
  vaddr = mmap(nullptr, buffer.capacity, PROT_READ | PROT_WRITE,
               MAP_SHARED, ion_info_fd.fd, 0);
  assert(vaddr != nullptr);

  buffer.data = vaddr;
  buffer.fd = ion_info_fd.fd;

  picture_cb_.data_cb(track_id, buffer);

  // free ion fd
  struct ion_handle_data ion_handle;
  memset(&ion_handle, 0, sizeof(ion_handle));
  ion_handle.handle = ion_info_fd.handle;
  if (ioctl(ion_device_, ION_IOC_FREE, &ion_handle) < 0) {
    QMMF_ERROR("%s ION free failed: %d", __func__, -errno);
  }

  // unmap memory
  if (vaddr != nullptr) {
    munmap(vaddr, buffer.capacity);
    vaddr = nullptr;
  }

  // close fd
  if (ion_info_fd.fd > 0) {
    close(ion_info_fd.fd);
  }

  QMMF_DEBUG("%s Exit ", __func__);
}

//Binder Proxy implementation of IPlayerService.
class BpPlayerService : public BpInterface<IPlayerService>
{
 public:
  BpPlayerService(const sp<IBinder>& impl)
  : BpInterface<IPlayerService>(impl) {}

  status_t Connect(const sp<IPlayerServiceCallback>& service_cb) {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());
    data.writeStrongBinder(IInterface::asBinder(service_cb));
    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_CONNECT), data, &reply);
    return reply.readInt32();
  }

  status_t Disconnect() {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());
    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_DISCONNECT), data, &reply);
    return reply.readInt32();
  }

  status_t CreateAudioTrack(uint32_t track_id,
                            AudioTrackCreateParam& param) {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());
    data.writeUint32(track_id);

    uint32_t param_size = sizeof param;
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memset(blob.data(), 0x0, param_size);
    memcpy(blob.data(), reinterpret_cast<void*>(&param), param_size);
    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_CREATE_AUDIOTRACK), data, &reply);
    blob.release();
    return reply.readInt32();
  }

  status_t CreateVideoTrack(uint32_t track_id,
                            VideoTrackCreateParam& param) {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());
    data.writeUint32(track_id);

    uint32_t param_size = sizeof param;
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memset(blob.data(), 0x0, param_size);
    memcpy(blob.data(), reinterpret_cast<void*>(&param), param_size);
    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_CREATE_VIDEOTRACK), data, &reply);
    blob.release();
    return reply.readInt32();
  }

  status_t DeleteAudioTrack(uint32_t track_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());
    data.writeUint32(track_id);
    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_DELETE_AUDIOTRACK), data, &reply);

    if (track_fd_map_.isEmpty()) {
      return NO_ERROR;
    }

    if (track_fd_map_.indexOfKey(track_id) >= 0) {
      track_fd_map_.removeItem(track_id);
    }
    return reply.readInt32();
  }

  status_t DeleteVideoTrack(uint32_t track_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());
    data.writeUint32(track_id);
    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_DELETE_VIDEOTRACK), data, &reply);

    if (track_fd_map_.isEmpty()) {
      return NO_ERROR;
    }

    if (track_fd_map_.indexOfKey(track_id) >= 0) {
      track_fd_map_.removeItem(track_id);
    }
    return reply.readInt32();
  }

  status_t SetAudioTrackParam(uint32_t track_id,
                              CodecParamType type,
                              void *param,
                              size_t param_size) {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());
    data.writeUint32(track_id);
    data.writeUint32(static_cast<uint32_t>(type));
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memcpy(blob.data(), reinterpret_cast<void*>(param), param_size);
    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_SET_AUDIOTRACK_PARAMS), data, &reply);
    return reply.readInt32();
  }

  status_t SetVideoTrackParam(uint32_t track_id,
                              CodecParamType type,
                              void *param,
                              size_t param_size) {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());
    data.writeUint32(track_id);
    data.writeUint32(static_cast<uint32_t>(type));
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memcpy(blob.data(), reinterpret_cast<void*>(param), param_size);

    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_SET_VIDEOTRACK_PARAMS), data, &reply);
    return reply.readInt32();
  }

  status_t DequeueInputBuffer(uint32_t track_id,
                              std::vector<AVCodecBuffer>& buffers) {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());
    data.writeUint32(track_id);

    size_t size = buffers.size();
    assert(size > 0);
    data.writeUint32(size);
    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_DEQUEUE_INPUT_BUFFER), data, &reply);
    int32_t ret = reply.readInt32();
    size = reply.readUint32();

    if (size == 0) {
      buffers.clear();
    } else {
      Mutex::Autolock lock(lock_);

      for (size_t i = 0; i < size; i++)
      {
        uint32_t param_size;
        reply.readUint32(&param_size);
        android::Parcel::ReadableBlob blob;
        reply.readBlob(param_size, &blob);
        void* buffer = const_cast<void*>(blob.data());
        AVCodecBuffer track_buffer;
        assert(param_size == sizeof(track_buffer));
        memset(&track_buffer, 0x0, sizeof track_buffer);
        memcpy(&track_buffer, buffer, param_size);

        int32_t mapped;
        reply.readInt32(&mapped);

        if (!mapped) {
          buffers[i].frame_length = track_buffer.frame_length;
          buffers[i].fd         = dup(reply.readFileDescriptor());
          buffers[i].buf_id     = track_buffer.buf_id;
          buffers[i].data       = track_buffer.data;

          ion_fd_map_ fd_map;
          if (track_fd_map_.isEmpty()) {
            fd_map.add(buffers[i].buf_id, buffers[i].fd);
          } else {
            fd_map = track_fd_map_.valueFor(track_id);
            fd_map.add(buffers[i].buf_id, buffers[i].fd);
          }

          track_fd_map_.replaceValueFor(track_id, fd_map);
        } else {

          ion_fd_map_ fd_map = track_fd_map_.valueFor(track_id);
          uint32_t client_fd = fd_map.valueFor(track_buffer.buf_id);

          buffers[i].fd           = client_fd;
          buffers[i].frame_length = track_buffer.frame_length;
          buffers[i].buf_id       = track_buffer.buf_id;
          buffers[i].data         = track_buffer.data;
        }

        QMMF_DEBUG("%s client fd : %d", __func__, buffers[i].fd);
        QMMF_DEBUG("%s service fd : %d", __func__, buffers[i].buf_id);
        QMMF_DEBUG("%s data : 0x%p", __func__, buffers[i].data);

        int32_t fd;
        reply.readInt32(&fd);
        blob.release();
      }
    }

    QMMF_VERBOSE("%s: buffers.size()=%d", __func__, buffers.size());
    return ret;
  }

  status_t QueueInputBuffer(uint32_t track_id,
                            std::vector<AVCodecBuffer>& buffers,
                            void *meta_param,
                            size_t meta_size,
                            TrackMetaBufferType meta_type) {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());
    data.writeUint32(track_id);

    size_t size = buffers.size();

    assert(size > 0);
    data.writeUint32(size);
    for (size_t i = 0; i < size; i++) {
      uint32_t param_size = sizeof (AVCodecBuffer);
      data.writeUint32(param_size);
      android::Parcel::WritableBlob blob;
      data.writeBlob(param_size, false, &blob);
      memset(blob.data(), 0x0, param_size);
      memcpy(blob.data(), reinterpret_cast<void*>(&buffers[i]), param_size);
    }

    data.writeUint32(meta_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(meta_size, false, &blob);
    memcpy(blob.data(), reinterpret_cast<void*>(meta_param), meta_size);
    data.writeUint32(static_cast<uint32_t>(meta_type));

    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_QUEUE_INPUT_BUFFER), data, &reply);
    return reply.readInt32();
  }

  status_t Prepare() {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());
    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_PREPARE), data, &reply);
    return reply.readInt32();
  }

  status_t Start() {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());
    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_START), data, &reply);
    return reply.readInt32();
  }

  status_t Stop(const PictureParam& params) {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());

    uint32_t param_size = sizeof params;
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memset(blob.data(), 0x0, param_size);
    memcpy(blob.data(),
           reinterpret_cast<void*>(const_cast<PictureParam*>(&params)),
           param_size);

    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_STOP), data, &reply);

    blob.release();
    return reply.readInt32();
  }

  status_t Pause(const PictureParam& params) {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());

    uint32_t param_size = sizeof params;
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memset(blob.data(), 0x0, param_size);
    memcpy(blob.data(),
           reinterpret_cast<void*>(const_cast<PictureParam*>(&params)),
           param_size);

    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_PAUSE), data, &reply);

    blob.release();
    return reply.readInt32();
  }

  status_t Resume() {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());
    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_RESUME), data, &reply);
    return reply.readInt32();
  }

  status_t SetPosition(int64_t seek_time) {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());
    data.writeInt64(seek_time);
    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_SET_POSITION), data, &reply);
    return reply.readInt32();
  }

  status_t SetTrickMode(TrickModeSpeed speed, TrickModeDirection dir) {
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerService::getInterfaceDescriptor());
    data.writeUint32(static_cast<uint32_t>(speed));
    data.writeUint32(static_cast<uint32_t>(dir));
    remote()->transact(uint32_t(QMMF_PLAYER_SERVICE_CMDS::
        PLAYER_SET_TRICKMODE), data, &reply);
    return reply.readInt32();
  }

 private:
  // mapping of service ion_fd and client ion_fd
  typedef DefaultKeyedVector<uint32_t, uint32_t> ion_fd_map_;

  // mapping of <track_id, map <service_fd, client_fd>>
  DefaultKeyedVector<int32_t, ion_fd_map_> track_fd_map_;

  Mutex lock_;
};


IMPLEMENT_META_INTERFACE(PlayerService, QMMF_PLAYER_SERVICE_NAME);

ServiceCallbackHandler::ServiceCallbackHandler(PlayerClient* client)
    : client_(client) {
  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_DEBUG("%s Exit ", __func__);
}

ServiceCallbackHandler::~ServiceCallbackHandler() {
  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_DEBUG("%s Exit ", __func__);
}

void ServiceCallbackHandler::NotifyPlayerEvent(EventType event_type,
                                               void *event_data,
                                               size_t event_data_size) {
  QMMF_DEBUG("%s Enter ", __func__);
  assert(client_ != nullptr);
  client_->NotifyPlayerEvent(event_type,event_data,event_data_size);
  QMMF_DEBUG("%s Exit ", __func__);
}

void ServiceCallbackHandler::NotifyVideoTrackEvent(uint32_t track_id,
                                                   EventType event_type,
                                                   void *event_data,
                                                   size_t event_data_size) {
  QMMF_DEBUG("%s Enter ", __func__);
  assert(client_ != nullptr);
  client_->NotifyVideoTrackEvent(track_id, event_type, event_data,
                                 event_data_size);
  QMMF_DEBUG("%s Exit ", __func__);
}

void ServiceCallbackHandler::NotifyAudioTrackEvent(uint32_t track_id,
                                                   EventType event_type,
                                                   void *event_data,
                                                   size_t event_data_size) {
  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_VERBOSE("%s INPARAM: track_id[%u]", __func__, track_id);
  QMMF_VERBOSE("%s INPARAM: event_type[%d]", __func__,
             static_cast<int>(event_type));
  QMMF_VERBOSE("%s INPARAM: event_data_size[%u]", __func__,
               event_data_size);
  assert(client_ != nullptr);
  client_->NotifyAudioTrackEvent(track_id, event_type, event_data,
                                 event_data_size);
  QMMF_DEBUG("%s Exit ", __func__);
}

void ServiceCallbackHandler::NotifyGrabPictureData(uint32_t track_id,
                                                   BufferDescriptor& buffer){
  QMMF_DEBUG("%s Enter", __func__);
  QMMF_DEBUG("%s track_id[%u]", __func__, track_id);
  assert(client_ != nullptr);
  client_->NotifyGrabPictureData(track_id, buffer);
  QMMF_DEBUG("%s Exit ", __func__);
}

class BpPlayerServiceCallback: public BpInterface<IPlayerServiceCallback> {
 public:
  BpPlayerServiceCallback(const sp<IBinder>& impl)
     : BpInterface<IPlayerServiceCallback>(impl) {}

  ~BpPlayerServiceCallback() {
    if (!track_buf_map_.isEmpty()) {
      track_buf_map_.clear();
    }
    //TODO: Expose DeleteTrack Api from Binder proxy and call it from service.
  }

  void NotifyPlayerEvent(EventType event_type, void *event_data,
                         size_t event_data_size)
  {
    QMMF_DEBUG("%s Enter ", __func__);
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerServiceCallback::getInterfaceDescriptor());
    data.writeInt32(static_cast<int32_t>(event_type));
    uint32_t eventSize = event_data_size;
    data.writeUint32(eventSize);
    android::Parcel::WritableBlob blob;
    data.writeBlob(eventSize, false, &blob);
    memcpy(blob.data(), reinterpret_cast<void*>(event_data), eventSize);
    remote()->transact(uint32_t(PLAYER_SERVICE_CB_CMDS::PLAYER_NOTIFY_EVENT),
       data, &reply, IBinder::FLAG_ONEWAY);
    blob.release();
    QMMF_DEBUG("%s Exit ", __func__);
  }

  void NotifyVideoTrackEvent(uint32_t track_id, EventType event_type,
                             void *event_data, size_t event_data_size) {
    QMMF_DEBUG("%s Enter ", __func__);
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerServiceCallback::getInterfaceDescriptor());
    data.writeUint32(static_cast<uint32_t>(track_id));
    data.writeInt32(static_cast<int32_t>(event_type));
    uint32_t eventSize = event_data_size;
    data.writeUint32(eventSize);
    android::Parcel::WritableBlob blob;
    data.writeBlob(eventSize, false, &blob);
    memcpy(blob.data(), reinterpret_cast<void*>(event_data), eventSize);
    remote()->transact(
        uint32_t(PLAYER_SERVICE_CB_CMDS::PLAYER_NOTIFY_VIDEO_TRACK_EVENT),
        data, &reply, IBinder::FLAG_ONEWAY);
    blob.release();
    QMMF_DEBUG("%s Exit ", __func__);
  }

  void NotifyAudioTrackEvent(uint32_t track_id, EventType event_type,
                             void *event_data, size_t event_data_size) {
    QMMF_DEBUG("%s Enter ", __func__);
    Parcel data, reply;
    data.writeInterfaceToken(IPlayerServiceCallback::getInterfaceDescriptor());
    data.writeUint32(static_cast<uint32_t>(track_id));
    data.writeInt32(static_cast<int32_t>(event_type));
    uint32_t eventSize = event_data_size;
    data.writeUint32(eventSize);
    android::Parcel::WritableBlob blob;
    data.writeBlob(eventSize, false, &blob);
    memcpy(blob.data(), reinterpret_cast<void*>(event_data), eventSize);
    remote()->transact(
        uint32_t(PLAYER_SERVICE_CB_CMDS::PLAYER_NOTIFY_AUDIO_TRACK_EVENT),
        data, &reply, IBinder::FLAG_ONEWAY);
    blob.release();
    QMMF_DEBUG("%s Exit ", __func__);
  }

  void NotifyGrabPictureData(uint32_t track_id, BufferDescriptor& buffer) {
    QMMF_DEBUG("Bp%s: Enter", __func__);

    Parcel data, reply;
    data.writeInterfaceToken(IPlayerServiceCallback::getInterfaceDescriptor());
    data.writeUint32(track_id);
    data.writeFileDescriptor(buffer.fd);
    uint32_t size = sizeof buffer;
    data.writeUint32(size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(size, false, &blob);
    memset(blob.data(), 0x0, size);
    memcpy(blob.data(), reinterpret_cast<void*>(&buffer), size);

    remote()->transact(
        uint32_t(PLAYER_SERVICE_CB_CMDS::PLAYER_NOTIFY_GRAB_PICTURE_DATA),
        data, &reply, IBinder::FLAG_ONEWAY);

    blob.release();
    QMMF_DEBUG("Bp%s: Exit", __func__);
  }

 private:
  // vector <ion_fd, bool>
  typedef DefaultKeyedVector <uint32_t, bool> buffer_map;
  // vector <track_id , buffer_map>
  DefaultKeyedVector<uint32_t,  buffer_map > track_buf_map_;
};

IMPLEMENT_META_INTERFACE(PlayerServiceCallback,
                            "player.service.IPlayerServiceCallback");

status_t BnPlayerServiceCallback::onTransact(uint32_t code,
                                             const Parcel& data,
                                             Parcel* reply,
                                             uint32_t flags) {
  QMMF_DEBUG("%s: Enter:(BnPlayerServiceCallback::onTransact)",
      __func__);
  CHECK_INTERFACE(BnPlayerServiceCallback, data, reply);

  switch(code) {
    case PLAYER_SERVICE_CB_CMDS::PLAYER_NOTIFY_EVENT: {
      EventType event_type = static_cast<EventType>(data.readInt32());
      uint32_t blobSize;
      data.readUint32(&blobSize);
      android::Parcel::ReadableBlob blob;
      data.readBlob(blobSize, &blob);
      void* event = const_cast<void*>(blob.data());
      NotifyPlayerEvent(event_type,event,blobSize);
      blob.release();
      return NO_ERROR;
    }
    break;
    case PLAYER_SERVICE_CB_CMDS::PLAYER_NOTIFY_VIDEO_TRACK_EVENT: {
      uint32_t track_id = data.readUint32();
      EventType event_type = static_cast<EventType>(data.readInt32());

      uint32_t event_size;
      data.readUint32(&event_size);
      android::Parcel::ReadableBlob blob;
      data.readBlob(event_size, &blob);
      void* event = const_cast<void*>(blob.data());

      QMMF_DEBUG("%s-NotifyVideoTrackEvent() TRACE", __func__);
      QMMF_VERBOSE("%s-NotifyVideoTrackEvent() INPARAM: track_id[%u]",
                   __func__, track_id);
      QMMF_VERBOSE("%s-NotifyVideoTrackEvent() INPARAM: event_type[%d]",
                   __func__, static_cast<int>(event_type));
      QMMF_VERBOSE("%s-NotifyVideoTrackEvent() INPARAM: event_size[%u]",
                   __func__, event_size);
      NotifyVideoTrackEvent(track_id, event_type, event, event_size);
      blob.release();

      return NO_ERROR;
    }
    break;
    case PLAYER_SERVICE_CB_CMDS::PLAYER_NOTIFY_AUDIO_TRACK_EVENT: {
      uint32_t track_id = data.readUint32();
      EventType event_type = static_cast<EventType>(data.readInt32());

      uint32_t event_size;
      data.readUint32(&event_size);
      android::Parcel::ReadableBlob blob;
      data.readBlob(event_size, &blob);
      void* event = const_cast<void*>(blob.data());

      QMMF_DEBUG("%s-NotifyAudioTrackEvent() TRACE", __func__);
      QMMF_VERBOSE("%s-NotifyAudioTrackEvent() INPARAM: track_id[%u]",
                   __func__, track_id);
      QMMF_VERBOSE("%s-NotifyAudioTrackEvent() INPARAM: event_type[%d]",
                   __func__, static_cast<int>(event_type));
      QMMF_VERBOSE("%s-NotifyAudioTrackEvent() INPARAM: event_size[%u]",
                   __func__, event_size);
      NotifyAudioTrackEvent(track_id, event_type, event, event_size);
      blob.release();

      return NO_ERROR;
    }
    break;
    case PLAYER_SERVICE_CB_CMDS::PLAYER_NOTIFY_GRAB_PICTURE_DATA: {
      QMMF_DEBUG("%s-NotifyGrabPictureData() TRACE", __func__);
      uint32_t track_id = data.readUint32();
      uint32_t ion_fd = dup(data.readFileDescriptor());

      uint32_t size;
      data.readUint32(&size);
      android::Parcel::ReadableBlob blob;
      data.readBlob(size, &blob);
      void* buf = const_cast<void*>(blob.data());

      BufferDescriptor buffer;
      memset(&buffer, 0x0, sizeof buffer);
      memcpy(&buffer, buf, size);
      buffer.fd = ion_fd;

      QMMF_DEBUG("%s-NotifyGrabPictureData() TRACE", __func__);
      QMMF_VERBOSE("%s-NotifyGrabPictureData() INPARAM: track_id[%u]",
                   __func__, track_id);
      QMMF_VERBOSE("%s-NotifyGrabPictureData() INPARAM: buffer[%s]",
                   __func__, buffer.ToString().c_str());
      NotifyGrabPictureData(track_id, buffer);
      blob.release();

      return NO_ERROR;
    }
    break;
    default: {
      QMMF_ERROR("%s Method not supported ", __func__);
    }
    break;
  }
  return NO_ERROR;
}

};  // namespace player
};  // namespace qmmf
