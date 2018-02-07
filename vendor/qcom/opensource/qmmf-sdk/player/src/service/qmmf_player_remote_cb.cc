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

#define TAG "PlayerRemoteCallBack"

#include "player/src/service/qmmf_player_remote_cb.h"

namespace qmmf {
namespace player {

RemoteCallBack::RemoteCallBack(const sp<IPlayerServiceCallback>& remote_cb)
    : client_cb_handle_(remote_cb) {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

RemoteCallBack::~RemoteCallBack() {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

void RemoteCallBack::NotifyPlayerEvent(EventType event_type,
                                       void *event_data,
                                       size_t event_data_size) {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  assert(client_cb_handle_.get() != nullptr);
  client_cb_handle_->NotifyPlayerEvent(event_type, event_data, event_data_size);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

void RemoteCallBack::NotifyVideoTrackData(
    uint32_t track_id,
    std::vector<BnTrackBuffer> &buffers,
    void *meta_param,
    TrackMetaBufferType meta_type,
    size_t meta_size) {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  assert(client_cb_handle_.get() != nullptr);
  client_cb_handle_->NotifyVideoTrackData(track_id, buffers, meta_param,
      meta_type, meta_size);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

void RemoteCallBack::NotifyVideoTrackEvent(
    uint32_t track_id,
    EventType event_type,
    void *event_data,
    size_t event_data_size) {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  assert(client_cb_handle_.get() != nullptr);
  client_cb_handle_->NotifyVideoTrackEvent(track_id, event_type,
      event_data, event_data_size);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

void RemoteCallBack::NotifyAudioTrackData(
    uint32_t track_id,
    std::vector<BnTrackBuffer> &buffers,
    void *meta_param,
    TrackMetaBufferType meta_type,
    size_t meta_size) {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  assert(client_cb_handle_.get() != nullptr);
  client_cb_handle_->NotifyAudioTrackData(track_id, buffers,
      meta_param, meta_type, meta_size);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

void RemoteCallBack::NotifyAudioTrackEvent(
    uint32_t track_id,
    EventType event_type,
    void *event_data,
    size_t event_data_size) {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  assert(client_cb_handle_.get() != nullptr);
  client_cb_handle_->NotifyAudioTrackEvent(track_id,event_type,
      event_data,event_data_size);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

void RemoteCallBack::NotifyDeleteAudioTrack(uint32_t track_id) {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  assert(client_cb_handle_.get() != nullptr);
  client_cb_handle_->NotifyDeleteAudioTrack(track_id);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

void RemoteCallBack::NotifyDeleteVideoTrack(uint32_t track_id) {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  assert(client_cb_handle_.get() != nullptr);
  client_cb_handle_->NotifyDeleteVideoTrack(track_id);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

void RemoteCallBack::NotifyGrabPictureData(BufferDescriptor& buffer) {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  assert(client_cb_handle_.get() != nullptr);
  client_cb_handle_->NotifyGrabPictureData(buffer);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

};  // namespace player
};  // namespace qmmf
