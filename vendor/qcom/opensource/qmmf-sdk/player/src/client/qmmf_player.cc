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

#define TAG "Player"

#include <binder/IPCThreadState.h>

#include "qmmf-sdk/qmmf_player.h"
#include "qmmf-sdk/qmmf_player_params.h"
#include "player/src/client/qmmf_player_client.h"
#include "common/qmmf_common_utils.h"

namespace qmmf {
namespace player {

Player::Player()
    : player_client_(nullptr) {
  player_client_ = new PlayerClient();
  assert( player_client_ != NULL);
}

Player::~Player() {
  if (player_client_) {
    delete player_client_;
    player_client_ = nullptr;
  }
}

status_t Player::Connect(PlayerCb& cb) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->Connect(cb);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Connect failed!", __func__);
  }
  return ret;
}

status_t Player::Disconnect() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->Disconnect();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Disconnect failed!", __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Player::CreateAudioTrack(
    uint32_t track_id,
    AudioTrackCreateParam& param,
    TrackCb& cb) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->CreateAudioTrack(track_id, param, cb);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: CreateAudioTrack failed!", __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Player::CreateVideoTrack(
    uint32_t track_id,
    VideoTrackCreateParam& param,
    TrackCb& cb) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->CreateVideoTrack(track_id, param, cb);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: CreateVideoTrack failed!", __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Player::DeleteAudioTrack(uint32_t track_id) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->DeleteAudioTrack(track_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: DeleteAudioTrack failed!", __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Player::DeleteVideoTrack(uint32_t track_id) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->DeleteVideoTrack(track_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: DeleteVideoTrack failed!", __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Player::DequeueInputBuffer(
    uint32_t track_id,
    std::vector<TrackBuffer>& buffers) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->DequeueInputBuffer(track_id, buffers);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: DequeueInputBuffer failed!", __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Player::QueueInputBuffer(
    uint32_t track_id,
    std::vector<TrackBuffer>& buffers,
    void *meta_param,
    size_t meta_size,
    TrackMetaBufferType meta_type) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->QueueInputBuffer(track_id, buffers, meta_param,
    meta_size, meta_type);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: QueueInputBuffer failed!", __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Player::Prepare() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->Prepare();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Prepare failed!", __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Player::Start() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->Start();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Start failed!", __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Player::Stop(bool do_flush) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->Stop(do_flush);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s: Stop failed!", __func__);
  }
  return ret;
}

status_t Player::Pause() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->Pause();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Pause failed!", __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Player::Resume() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->Resume();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Resume failed!", __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Player::SetPosition(int64_t seek_time) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->SetPosition(seek_time);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: SetPosition failed!", __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Player::SetTrickMode(TrickModeSpeed speed, TrickModeDirection dir) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->SetTrickMode(speed, dir);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: SetTrickMode failed!", __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Player::GrabPicture(PictureParam param, PictureCallback& cb) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->GrabPicture(param, cb);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: GrabPicture failed!", __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Player::SetAudioTrackParam(uint32_t track_id,
                                    CodecParamType type,
                                    void *param,
                                    size_t param_size) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->SetAudioTrackParam(track_id, type, param,
             param_size);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: SetAudioTrackParam failed!", __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Player::SetVideoTrackParam(uint32_t track_id,
                                    CodecParamType type,
                                    void *param,
                                    size_t param_size) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->SetVideoTrackParam(track_id, type, param,
             param_size);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: SetVideoTrackParam failed!", __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

};  // namespace player
};  // namespace qmmf
