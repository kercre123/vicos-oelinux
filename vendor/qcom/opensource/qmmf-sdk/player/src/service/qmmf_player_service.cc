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

#define TAG "PlayerService"

#include "player/src/service/qmmf_player_service.h"
#include "player/src/client/qmmf_player_service_intf.h"

namespace qmmf {
namespace player {

 // Method of BnInterface<IPlayerService>.
 // This method would get call to handle incoming messages from clients.
 status_t PlayerService::onTransact(uint32_t code, const Parcel& data,
                                    Parcel* reply, uint32_t flags) {
  QMMF_DEBUG("%s:%s: Enter:(BnPlayerService::onTransact)", TAG, __func__);
  CHECK_INTERFACE(IPlayerService, data, reply);
  int32_t ret = 0;

  switch (code) {
    case PLAYER_CONNECT: {
      sp<IPlayerServiceCallback> client_cb_handle = interface_cast
          <IPlayerServiceCallback>(data.readStrongBinder());
      ret = Connect(client_cb_handle);
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;

    case PLAYER_DISCONNECT: {
      ret = Disconnect();
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;

    case PLAYER_CREATE_AUDIOTRACK:
    {
      uint32_t track_id = data.readUint32();
      uint32_t blob_size;
      data.readUint32(&blob_size);
      android::Parcel::ReadableBlob blob;
      data.readBlob(blob_size, &blob);
      void* params = const_cast<void*>(blob.data());
      AudioTrackCreateParam audio_track_params;
      memset(&audio_track_params, 0x0, sizeof audio_track_params);
      memcpy(&audio_track_params, params, blob_size);
      QMMF_DEBUG("%s:%s-CreateAudioTrack() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s:%s-CreateAudioTrack() INPARAM: track_id[%u]",
          TAG, __func__, track_id);
      ret = CreateAudioTrack(track_id, audio_track_params);
      blob.release();
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;

    case PLAYER_CREATE_VIDEOTRACK:
    {
      uint32_t track_id = data.readUint32();
      uint32_t blob_size;
      data.readUint32(&blob_size);
      android::Parcel::ReadableBlob blob;
      data.readBlob(blob_size, &blob);
      void* params = const_cast<void*>(blob.data());
      VideoTrackCreateParam video_track_param;
      memset(&video_track_param, 0x0, sizeof video_track_param);
      memcpy(&video_track_param, params, blob_size);
      QMMF_DEBUG("%s:%s-CreateVideoTrack() TRACE", TAG, __func__);
      QMMF_VERBOSE("%s:%s-CreateVideoTrack() INPARAM: track_id[%u]",
          TAG, __func__, track_id);
      ret = CreateVideoTrack(track_id, video_track_param);
      blob.release();
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;

    case PLAYER_DELETE_AUDIOTRACK:
    {
      uint32_t track_id = data.readUint32();
      ret = DeleteAudioTrack(track_id);
      if (track_fd_map_.isEmpty()) {
        return NO_ERROR;
      }

      if (track_fd_map_.indexOfKey(track_id) >= 0) {
        track_fd_map_.removeItem(track_id);
      }
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;

    case PLAYER_DELETE_VIDEOTRACK:
    {
      uint32_t track_id = data.readUint32();
      ret = DeleteVideoTrack(track_id);
      if (track_fd_map_.isEmpty()) {
        return NO_ERROR;
      }

      if (track_fd_map_.indexOfKey(track_id) >= 0) {
        track_fd_map_.removeItem(track_id);
      }
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;

    case PLAYER_SET_AUDIOTRACK_PARAMS:
    {
      uint32_t track_id = data.readUint32();
      uint32_t param_type = data.readUint32();
      uint32_t blob_size;
      data.readUint32(&blob_size);
      android::Parcel::ReadableBlob blob;
      data.readBlob(blob_size, &blob);
      void* params = const_cast<void*>(blob.data());
      ret = SetAudioTrackParam(track_id,
          static_cast<CodecParamType>(param_type), params, blob_size);
      blob.release();
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;

    case PLAYER_SET_VIDEOTRACK_PARAMS:
    {
      uint32_t track_id = data.readUint32();
      uint32_t param_type = data.readUint32();
      uint32_t blob_size;
      data.readUint32(&blob_size);
      android::Parcel::ReadableBlob blob;
      data.readBlob(blob_size, &blob);
      void* params = const_cast<void*>(blob.data());
      ret = SetVideoTrackParam(track_id,
          static_cast<CodecParamType>(param_type), params, blob_size);
      blob.release();
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;

    case PLAYER_DEQUEUE_INPUT_BUFFER:
    {
      uint32_t track_id = data.readUint32();
      std::vector<AVCodecBuffer> buffers;

      uint32_t vector_size;
      data.readUint32(&vector_size);

      for (uint32_t i = 0; i < vector_size; i++)  {
        AVCodecBuffer track_buffer;
        memset(&track_buffer, 0x0, sizeof track_buffer);
        buffers.push_back(track_buffer);
      }

      ret = DequeueInputBuffer(track_id, buffers);

      reply->writeInt32(ret);

      {
        Mutex::Autolock lock(lock_);

        for (size_t i = 0; i < vector_size; i++)
        {
          uint32_t param_size = sizeof (AVCodecBuffer);
          reply->writeUint32(param_size);
          android::Parcel::WritableBlob blob;
          reply->writeBlob(param_size, false, &blob);
          memset(blob.data(), 0x0, param_size);
          memcpy(blob.data(), reinterpret_cast<void*>(&buffers[i]),param_size);

          QMMF_DEBUG("%s:%s service fd : %d", TAG, __func__, buffers[i].fd);
          QMMF_DEBUG("%s:%s size %d", TAG,__func__, buffers[i].frame_length);
          QMMF_DEBUG("%s:%s data 0x%p", TAG, __func__, buffers[i].data);

          bool mapped = false;

          if (!(track_fd_map_.isEmpty())) {

            int32_t map_idx = track_fd_map_.indexOfKey(track_id);

            if (map_idx >= 0) {
              ion_fd_map_ fd_map = track_fd_map_.valueFor(track_id);
              int32_t fd_idx = fd_map.indexOfKey(buffers[i].fd);

              if (fd_idx >= 0) {
                reply->writeInt32(1);
                reply->writeInt32(buffers[i].fd);
                mapped = true;
              }
            }
          }

          if (!mapped) {
            reply->writeInt32(0);
            reply->writeFileDescriptor(buffers[i].fd);
            reply->writeInt32(buffers[i].fd);

            ion_fd_map_ fd_map;

            if (!track_fd_map_.isEmpty()) {
              fd_map = track_fd_map_.valueFor(track_id);
            }
            fd_map.add(buffers[i].fd, 1);

            track_fd_map_.replaceValueFor(track_id, fd_map);
          }

          blob.release();
        }
      }
      return NO_ERROR;
    }
    break;

    case PLAYER_QUEUE_INPUT_BUFFER:
    {
      uint32_t track_id = data.readUint32();

      std::vector<AVCodecBuffer> buffers;
      uint32_t vector_size;
      data.readUint32(&vector_size);

      for (uint32_t i = 0; i < vector_size; i++)  {
        uint32_t size;
        data.readUint32(&size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(size, &blob);
        void* buffer = const_cast<void*>(blob.data());
        AVCodecBuffer track_buffer;
        assert(size == sizeof(track_buffer));
        memset(&track_buffer, 0x0, sizeof track_buffer);
        memcpy(&track_buffer, buffer, size);
        buffers.push_back(track_buffer);
        blob.release();
      }

      uint32_t meta_size, meta_type;
      meta_size = data.readUint32();
      android::Parcel::ReadableBlob blob;
      data.readBlob(meta_size, &blob);
      void* meta_param = const_cast<void*>(blob.data());
      meta_type = data.readUint32();
      ret =  QueueInputBuffer(track_id, buffers, meta_param, meta_size,
      static_cast<TrackMetaBufferType>(meta_type));
      blob.release();
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;

    case PLAYER_PREPARE:
    {
      ret = Prepare();
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;

    case PLAYER_START:
    {
      ret = Start();
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;

    case PLAYER_STOP:
    {
      int32_t do_flush;
      data.readInt32(&do_flush);
      ret = Stop(do_flush);
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;

    case PLAYER_PAUSE:
    {
      ret = Pause();
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;

    case PLAYER_RESUME:
    {
      ret = Resume();
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;

    case PLAYER_SET_POSITION:
    {
      int64_t seek_time;
      seek_time = data.readInt64();
      ret = SetPosition(seek_time);
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;

    case PLAYER_SET_TRICKMODE:
    {
      uint32_t speed, dir;
      speed = data.readUint32();
      dir = data.readUint32();
      ret = SetTrickMode(static_cast<TrickModeSpeed>(speed),
          static_cast<TrickModeDirection>(dir));
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;

    case PLAYER_GRAB_PICTURE:
    {
      uint32_t blob_size;
      data.readUint32(&blob_size);
      android::Parcel::ReadableBlob blob;
      data.readBlob(blob_size, &blob);
      void* params = const_cast<void*>(blob.data());
      PictureParam picture_params;
      assert(blob_size == sizeof(picture_params));
      memcpy(&picture_params, params, blob_size);
      ret = GrabPicture(static_cast<PictureParam>(picture_params));
      blob.release();
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
  return NO_ERROR;
}

PlayerService::PlayerService()
    : connected_(false), player_(nullptr) {
  QMMF_INFO("%s:%s: PlayerService Instantiated! ", TAG, __func__);
}

PlayerService::~PlayerService() {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  QMMF_INFO("%s:%s: Exit ", TAG, __func__);
}

status_t PlayerService::Connect(const sp<IPlayerServiceCallback>& service_cb) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);

  player_ = PlayerImpl::CreatePlayer();
  if (!player_) {
      QMMF_ERROR("%s:%s: Can't create Player Instance!!", TAG, __func__);
      return NO_MEMORY;
  }
  remote_callback_ = new RemoteCallBack(service_cb);

  auto ret = player_->Connect(remote_callback_);
  assert(ret == NO_ERROR);

  death_notifier_ = new DeathNotifier(this);
  if (NULL == death_notifier_.get()) {
      QMMF_ERROR("%s:%s: Unable to allocate death notifier!", TAG, __func__);
      return NO_MEMORY;
  }

  IInterface::asBinder(remote_callback_->getRemoteClient())
      ->linkToDeath(death_notifier_);

  connected_ = true;
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t PlayerService::Disconnect() {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);

  int32_t ret = NO_ERROR;
  if (!connected_)
    return NO_INIT;

  IInterface::asBinder(remote_callback_->getRemoteClient())
    ->unlinkToDeath(death_notifier_);

  if (death_notifier_.get() != nullptr) {
  death_notifier_.clear();
  death_notifier_ = nullptr;
  }

  if (player_ != nullptr) {
  ret = player_->Disconnect();
  delete player_;
  player_ = nullptr;
  }

  if (remote_callback_.get() != nullptr) {
  remote_callback_.clear();
  remote_callback_ = nullptr;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t PlayerService::CreateAudioTrack(
    uint32_t track_id,
    AudioTrackCreateParam& param) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);
  assert(player_ != NULL);

  auto ret = player_->CreateAudioTrack(track_id, param);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: CreateAudioTrack failed: %d", TAG, __func__, ret);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return NO_ERROR;
}

status_t PlayerService::CreateVideoTrack(
    uint32_t track_id,
    VideoTrackCreateParam& param) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);
  assert(player_ != NULL);

  auto ret = player_->CreateVideoTrack(track_id, param);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: CreateVideoTrack failed: %d", TAG, __func__, ret);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return NO_ERROR;
}

status_t PlayerService::DeleteAudioTrack(uint32_t track_id) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  QMMF_VERBOSE("%s:%s INPARAM: track_id[%u]", TAG, __func__, track_id);
  assert(player_ != NULL);

  auto ret = player_->DeleteAudioTrack(track_id);
  if (ret != NO_ERROR) {
  QMMF_INFO("%s:%s: DeleteAudioTrack failed!", TAG, __func__);
  return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return NO_ERROR;
}

status_t PlayerService::DeleteVideoTrack(uint32_t track_id) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(player_ != NULL);

  auto ret = player_->DeleteVideoTrack(track_id);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: DeleteVideoTrack failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t PlayerService::DequeueInputBuffer(
    uint32_t track_id,
    std::vector<AVCodecBuffer>& buffers) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(player_ != NULL);

  auto ret = player_->DequeueInputBuffer(track_id,buffers);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: DequeueInputBuffer failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t PlayerService::QueueInputBuffer(
    uint32_t track_id,
    std::vector<AVCodecBuffer>& buffers,
    void *meta_param,
    size_t meta_size,
    TrackMetaBufferType meta_type) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(player_ != NULL);

  auto ret = player_->QueueInputBuffer(track_id, buffers, meta_param,
  meta_size, meta_type);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: QueueInputBuffer failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t PlayerService::Prepare() {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(player_ != NULL);

  auto ret = player_->Prepare();
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: Prepare failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret; //TODO
}

status_t PlayerService::Start() {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(player_ != NULL);

  auto ret = player_->Start();
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: Start failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t PlayerService::Stop(bool do_flush) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(player_ != NULL);

  auto ret = player_->Stop(do_flush);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: Stop failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t PlayerService::Pause() {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(player_ != NULL);

  auto ret = player_->Pause();
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: Pause failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t PlayerService::Resume() {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(player_ != NULL);

  auto ret = player_->Resume();
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: Resume failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t PlayerService::SetPosition(int64_t seek_time) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(player_ != NULL);

  auto ret = player_->SetPosition(seek_time);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: SetPosition failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t PlayerService::SetTrickMode(TrickModeSpeed speed, TrickModeDirection dir) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(player_ != NULL);

  auto ret = player_->SetTrickMode(speed, dir);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: SetTrickMode failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t PlayerService::GrabPicture(PictureParam param) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(player_ != NULL);

  auto ret = player_->GrabPicture(param);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: GrabPicture failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t PlayerService::SetAudioTrackParam(uint32_t track_id,
                                           CodecParamType type,
                                           void *param,
                                           size_t param_size) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(player_ != NULL);

  auto ret = player_->SetAudioTrackParam(track_id, type, param, param_size);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: SetAudioTrackParam failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}

status_t PlayerService::SetVideoTrackParam(uint32_t track_id,
                                           CodecParamType type,
                                           void *param,
                                           size_t param_size) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  if (!connected_)
    return NO_INIT;

  assert(player_ != NULL);

  auto ret = player_->SetVideoTrackParam(track_id, type, param, param_size);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: SetVideoTrackParam failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit ", TAG, __func__);
  return ret;
}


};  // namespace player
};  // namespace qmmf
