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

#define TAG "RecorderRemoteCallBack"

#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_remote_cb.h"

namespace qmmf {

namespace recorder {

RemoteCallBack::RemoteCallBack(const uint32_t client_id,
                               const sp<IRecorderServiceCallback>&
                                  remote_client)
    : client_cb_handle_(remote_client)
    , client_id_(client_id) {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  QMMF_INFO("%s:%s: Exit client_id(%d) (0x%p)", TAG, __func__, client_id, this);
}

RemoteCallBack::~RemoteCallBack() {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

void RemoteCallBack::NotifyRecorderEvent(EventType event_type, void *event_data,
                                         size_t event_data_size) {

  assert(client_cb_handle_.get() != nullptr);
  client_cb_handle_->NotifyRecorderEvent(event_type, event_data,
                                         event_data_size);
}

void RemoteCallBack::NotifySessionEvent(EventType event_type, void *event_data,
                                        size_t event_data_size) {

  assert(client_cb_handle_.get() != NULL);
  client_cb_handle_->NotifySessionEvent(event_type, event_data,
                                        event_data_size);
}

void RemoteCallBack::NotifySnapshotData(uint32_t camera_id,
                                        uint32_t sequence_count,
                                        BnBuffer& buffer, MetaData& meta_data) {

  assert(client_cb_handle_.get() != NULL);
  client_cb_handle_->NotifySnapshotData(camera_id, sequence_count, buffer,
                                        meta_data);
}

void RemoteCallBack::NotifyVideoTrackData(uint32_t track_id,
                                          std::vector<BnBuffer> &buffers,
                                          std::vector<MetaData>& meta_buffers) {

  assert(client_cb_handle_.get() != nullptr);
  client_cb_handle_->NotifyVideoTrackData(track_id, buffers, meta_buffers);
}

void RemoteCallBack::NotifyVideoTrackEvent(uint32_t track_id,
                                           EventType event_type,
                                           void *event_data,
                                           size_t event_data_size) {

  assert(client_cb_handle_.get() != nullptr);
  client_cb_handle_->NotifyVideoTrackEvent(track_id, event_type, event_data,
                                           event_data_size);
}

void RemoteCallBack::NotifyAudioTrackData(uint32_t track_id,
                                          std::vector<BnBuffer> &buffers,
                                          std::vector<MetaData>& meta_buffers) {

  assert(client_cb_handle_.get() != nullptr);
  client_cb_handle_->NotifyAudioTrackData(track_id, buffers, meta_buffers);
}

void RemoteCallBack::NotifyAudioTrackEvent(uint32_t track_id,
                                           EventType event_type,
                                           void *event_data,
                                           size_t event_data_size) {

  assert(client_cb_handle_.get() != nullptr);
  client_cb_handle_->NotifyAudioTrackEvent(track_id, event_type, event_data,
                                           event_data_size);
}

void RemoteCallBack::NotifyCameraResult(uint32_t camera_id,
                                        const CameraMetadata &result) {
  assert(client_cb_handle_.get() != nullptr);
  client_cb_handle_->NotifyCameraResult(camera_id, result);
}

void RemoteCallBack::NotifyDeleteVideoTrack(uint32_t track_id) {

  assert(client_cb_handle_.get() != nullptr);
  client_cb_handle_->NotifyDeleteVideoTrack(track_id);
}

}; // namespace recorder

}; // namespace qmmf
