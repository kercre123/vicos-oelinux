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
#include <algorithm>

#include "player/src/client/qmmf_player_service_intf.h"
#include "player/src/service/qmmf_player_impl.h"
#include "player/src/service/qmmf_player_remote_cb.h"
#include "player/src/service/qmmf_player_common.h"

namespace qmmf {
namespace player {

using namespace android;

class PlayerService: public BnInterface<IPlayerService> {
 public:

  PlayerService();

  ~PlayerService();

 private:
  class DeathNotifier : public IBinder::DeathRecipient {
   public:
    DeathNotifier(sp<PlayerService> parent) : parent_(parent) {}

    void binderDied(const wp<IBinder>&) override {
       QMMF_WARN("PlayerService:%s: Client Exited or Died!", __func__);
       assert(parent_.get() != nullptr);
       parent_->Disconnect();
     }
     sp<PlayerService> parent_;
  };

  friend class DeathNotifier;

  // Method of BnInterface<IPlayerService>.
  // This method would get call to handle incoming messages from clients.
  status_t onTransact(uint32_t code, const Parcel& data,
                               Parcel* reply, uint32_t flags = 0) override;

  status_t Connect(const sp<IPlayerServiceCallback>& service_cb) override;

  status_t Disconnect() override;

  status_t CreateAudioTrack(uint32_t track_id,
                          AudioTrackCreateParam& param) override;

  status_t CreateVideoTrack(uint32_t track_id,
                          VideoTrackCreateParam& param) override;

  status_t DeleteAudioTrack(uint32_t track_id) override;
  status_t DeleteVideoTrack(uint32_t track_id) override;

  status_t Prepare() override;

  status_t DequeueInputBuffer(uint32_t track_id,
                            std::vector<AVCodecBuffer>& buffers) override;

  status_t QueueInputBuffer(uint32_t track_id,
                          std::vector<AVCodecBuffer>& buffers,
                          void *meta_param,
                          size_t meta_size,
                          TrackMetaBufferType meta_type) override;

  status_t Start() override;

  status_t Stop(bool do_flush) override;

  status_t Pause() override;

  status_t Resume() override;

  status_t SetPosition(int64_t seek_time) override;

  status_t SetTrickMode(TrickModeSpeed speed, TrickModeDirection dir) override;

  status_t GrabPicture(PictureParam param) override;

  status_t SetAudioTrackParam(uint32_t track_id,
                            CodecParamType type,
                            void *param,
                            size_t param_size) override;

  status_t SetVideoTrackParam(uint32_t track_id,
                            CodecParamType type,
                            void *param,
                            size_t param_size) override;

 private:
  bool                  connected_;
  PlayerImpl*           player_;
  sp<DeathNotifier>     death_notifier_;
  sp<RemoteCallBack>    remote_callback_;
  Mutex                 lock_;


  // mapping of service ion_fd and mapped status
  typedef DefaultKeyedVector<uint32_t, uint32_t> ion_fd_map_;

  // mapping of <track_id, map <service_fd , mapped_status>>
  DefaultKeyedVector<int32_t, ion_fd_map_> track_fd_map_;

};

};  // namespace player
};  // namespce qmmf
