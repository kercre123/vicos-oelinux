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
#include <linux/msm_ion.h>

#include "qmmf-sdk/qmmf_player_params.h"
#include "player/src/client/qmmf_player_service_intf.h"
#include "player/src/service/qmmf_player_common.h"
#include "common/qmmf_log.h"

namespace qmmf {
namespace player {

using namespace android;

class PlayerClient {
 public:
  PlayerClient();

  ~PlayerClient();

  status_t Connect(PlayerCb& cb);

  status_t Disconnect();

  status_t CreateAudioTrack(uint32_t track_id,
                            AudioTrackCreateParam& param,
                            TrackCb& cb);

  status_t CreateVideoTrack(uint32_t track_id,
                            VideoTrackCreateParam& param,
                            TrackCb& cb);

  status_t DeleteAudioTrack(uint32_t track_id);

  status_t DeleteVideoTrack(uint32_t track_id);

  status_t Prepare();

  status_t DequeueInputBuffer(uint32_t track_id,
                              std::vector<TrackBuffer>& buffers);

  status_t QueueInputBuffer(uint32_t track_id,
                            std::vector<TrackBuffer>& buffers,
                            void *meta_param,
                            size_t meta_size,
                            TrackMetaBufferType meta_type);

  status_t Start();

  status_t Stop(bool do_flush);

  status_t Pause();

  status_t Resume();

  status_t SetPosition(int64_t seek_time);

  status_t SetTrickMode(TrickModeSpeed speed, TrickModeDirection dir);

  status_t GrabPicture(PictureParam param, PictureCallback& cb);

  status_t SetAudioTrackParam(uint32_t track_id,
                              CodecParamType type,
                              void *param,
                              size_t param_size);

  status_t SetVideoTrackParam(uint32_t track_id,
                              CodecParamType type,
                              void *param,
                              size_t param_size);

  // callback from service.
  void NotifyPlayerEvent(EventType event_type, void *event_data,
                         size_t event_data_size);

  void NotifyVideoTrackData(uint32_t track_id,
                            std::vector<BnTrackBuffer> &buffers,
                            void *meta_param,
                            TrackMetaBufferType meta_type,
                            size_t meta_size);

  void NotifyVideoTrackEvent(uint32_t track_id, EventType event_type,
                             void *event_data,
                             size_t event_data_size);

  void NotifyAudioTrackData(uint32_t track_id,
                            const std::vector<BnTrackBuffer> &buffers,
                            void *meta_param,
                            TrackMetaBufferType meta_type,
                            size_t meta_size);

  void NotifyAudioTrackEvent(uint32_t track_id, EventType event_type,
                             void *event_data,
                             size_t event_data_size);

  void NotifyDeleteAudioTrack(uint32_t track_id);

  void NotifyDeleteVideoTrack(uint32_t track_id);

  void NotifyGrabPictureData(BufferDescriptor& buffer);

 private:

  bool CheckServiceStatus();

  class DeathNotifier : public IBinder::DeathRecipient
  {
   public:
    DeathNotifier(PlayerClient* parent) : parent_(parent) {}

    void binderDied(const wp<IBinder>&) override {
          ALOGD("PlayerClient:%s: Player service died", __func__);
          Mutex::Autolock l(parent_->lock_);
          parent_->player_service_.clear();
          parent_->player_service_ = nullptr;
          assert(0);
    }
    PlayerClient* parent_;
  };
  friend class DeathNotifier;

  Mutex                                     lock_;
  sp<IPlayerService>                        player_service_;
  sp<DeathNotifier>                         death_notifier_;
  int32_t                                   ion_device_;
  PlayerCb                                  player_cb_;
  PictureCallback                           picture_cb_;
  DefaultKeyedVector<uint32_t, TrackCb >    track_cb_list_;

  typedef struct BufInfo {
    // fd at service
    uint32_t buf_id;

    uint32_t client_fd;

    // memory mapped buffer
    void*    vaddr;

    size_t   frame_len;

    // ION handle
    ion_user_handle_t ion_handle;
  } BufInfo;

  // map<fd , buf_info>
  typedef DefaultKeyedVector<uint32_t, BufInfo> buf_info_map;

  // map <track id , map<fd , buf info>>
  DefaultKeyedVector<uint32_t,  buf_info_map> track_buf_map_;
};


class ServiceCallbackHandler : public BnPlayerServiceCallback {
 public:

  ServiceCallbackHandler(PlayerClient * client);

  ~ServiceCallbackHandler();

 private:
  // Methods of BnPlayerServiceCallback.

  void NotifyPlayerEvent(EventType event_type, void *event_data,
                         size_t event_data_size) override;

  void NotifyVideoTrackData(uint32_t track_id,
                            std::vector<BnTrackBuffer> &buffers,
                            void *meta_param,
                            TrackMetaBufferType meta_type,
                            size_t meta_size) override;

  void NotifyVideoTrackEvent(uint32_t track_id, EventType event_type,
                             void *event_data,
                             size_t event_data_size) override;

  void NotifyAudioTrackData(uint32_t track_id,
                            const std::vector<BnTrackBuffer> &buffers,
                            void *meta_param,
                            TrackMetaBufferType meta_type,
                            size_t meta_size) override;

  void NotifyAudioTrackEvent(uint32_t track_id, EventType event_type,
                             void *event_data,
                             size_t event_data_size) override;

  void NotifyGrabPictureData(BufferDescriptor& buffer) override;

  PlayerClient *client_;
};


};  // namespace player
};  // namespace qmmf
