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

#define LOG_TAG "Player"

#include <binder/IPCThreadState.h>
#include <qmmf-sdk/qmmf_player.h>
#include <qmmf-sdk/qmmf_player_params.h>

#include "common/utils/qmmf_common_utils.h"
#include "player/src/client/qmmf_player_client.h"

namespace qmmf {
namespace player {

Player::Player()
    : player_client_(nullptr) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_INFO("%s() player instantiated", __func__);
}

Player::~Player() {
  QMMF_DEBUG("%s() TRACE", __func__);

  if (player_client_ != nullptr) {
    delete player_client_;
    player_client_ = nullptr;
  }

  QMMF_INFO("%s() player destroyed", __func__);
}

status_t Player::Connect(PlayerCb& cb) {
  QMMF_DEBUG("%s() TRACE", __func__);

  player_client_ = new PlayerClient();
  if (player_client_ == nullptr)
    return -ENOMEM;

  status_t result = player_client_->Connect(cb);
  if (result != NO_ERROR)
    QMMF_ERROR("%s() client->Connect failed: %d", __func__, result);

  return result;
}

status_t Player::Disconnect() {
  QMMF_DEBUG("%s() TRACE", __func__);

  if (player_client_ == nullptr) {
    QMMF_WARN("%s() player already disconnected", __func__);
    return NO_ERROR;
  }

  auto ret = player_client_->Disconnect();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Disconnect failed!", __func__);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Player::CreateAudioTrack(
    uint32_t track_id,
    AudioTrackCreateParam& param,
    TrackCb& cb) {
  QMMF_INFO("%s: Enter", __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->CreateAudioTrack(track_id, param, cb);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: CreateAudioTrack failed!", __func__);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Player::CreateVideoTrack(
    uint32_t track_id,
    VideoTrackCreateParam& param,
    TrackCb& cb) {
  QMMF_INFO("%s: Enter", __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->CreateVideoTrack(track_id, param, cb);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: CreateVideoTrack failed!", __func__);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Player::DeleteAudioTrack(uint32_t track_id) {
  QMMF_INFO("%s: Enter", __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->DeleteAudioTrack(track_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: DeleteAudioTrack failed!", __func__);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Player::DeleteVideoTrack(uint32_t track_id) {
  QMMF_INFO("%s: Enter", __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->DeleteVideoTrack(track_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: DeleteVideoTrack failed!", __func__);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Player::DequeueInputBuffer(uint32_t track_id,
                                    std::vector<TrackBuffer>& buffers) {
  QMMF_INFO("%s: Enter", __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->DequeueInputBuffer(track_id, buffers);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: DequeueInputBuffer failed!", __func__);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Player::QueueInputBuffer(
    uint32_t track_id,
    std::vector<TrackBuffer>& buffers,
    void *meta_param,
    size_t meta_size,
    TrackMetaBufferType meta_type) {
  QMMF_INFO("%s: Enter", __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->QueueInputBuffer(track_id, buffers, meta_param,
    meta_size, meta_type);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: QueueInputBuffer failed!", __func__);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Player::Prepare() {
  QMMF_INFO("%s: Enter", __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->Prepare();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Prepare failed!", __func__);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Player::Start() {
  QMMF_INFO("%s: Enter", __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->Start();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Start failed!", __func__);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Player::Stop(const PictureCallback& handler,
                      const PictureParam& params) {
  QMMF_INFO("%s: Enter", __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->Stop(handler, params);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s: Stop failed!", __func__);
  }
  return ret;
}

status_t Player::Pause(const PictureCallback& handler,
                       const PictureParam& params) {
  QMMF_INFO("%s: Enter", __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->Pause(handler, params);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Pause failed!", __func__);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Player::Resume() {
  QMMF_INFO("%s: Enter", __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->Resume();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Resume failed!", __func__);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Player::SetPosition(int64_t seek_time) {
  QMMF_INFO("%s: Enter", __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->SetPosition(seek_time);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: SetPosition failed!", __func__);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Player::SetTrickMode(TrickModeSpeed speed, TrickModeDirection dir) {
  QMMF_INFO("%s: Enter", __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->SetTrickMode(speed, dir);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: SetTrickMode failed!", __func__);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Player::SetAudioTrackParam(uint32_t track_id,
                                    CodecParamType type,
                                    void *param,
                                    size_t param_size) {
  QMMF_INFO("%s: Enter", __func__);
  QMMF_VERBOSE("%s() INPARAM: type[%d]", __func__,
               static_cast<int>(type));
  QMMF_VERBOSE("%s() INPARAM: param_size[%zu]", __func__, param_size);
  assert(player_client_ != nullptr);

  auto ret = player_client_->SetAudioTrackParam(track_id, type, param,
             param_size);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: SetAudioTrackParam failed!", __func__);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Player::SetVideoTrackParam(uint32_t track_id,
                                    CodecParamType type,
                                    void *param,
                                    size_t param_size) {
  QMMF_INFO("%s: Enter", __func__);
  assert(player_client_ != nullptr);

  auto ret = player_client_->SetVideoTrackParam(track_id, type, param,
             param_size);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: SetVideoTrackParam failed!", __func__);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

}; // namespace player
}; // namespace qmmf
