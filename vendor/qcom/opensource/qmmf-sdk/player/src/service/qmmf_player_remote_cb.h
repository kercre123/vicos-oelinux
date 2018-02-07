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

#include "player/src/client/qmmf_player_service_intf.h"
#include "common/qmmf_common_utils.h"
#include "common/qmmf_log.h"

namespace qmmf {
namespace player {

class RemoteCallBack : public RefBase {
 public:
  RemoteCallBack(const sp<IPlayerServiceCallback>& remote_cb);

  ~RemoteCallBack();

  sp<IPlayerServiceCallback>& getRemoteClient() {
     return client_cb_handle_;
  }

  void NotifyPlayerEvent(EventType event_type, void *event_data,
                           size_t event_data_size);

  void NotifyVideoTrackData(uint32_t track_id,
                            std::vector<BnTrackBuffer> &buffers,
                            void *meta_param, TrackMetaBufferType meta_type,
                            size_t meta_size);

  void NotifyVideoTrackEvent(uint32_t track_id, EventType event_type,
                             void *event_data, size_t event_data_size);

  void NotifyAudioTrackData(uint32_t track_id,
                            std::vector<BnTrackBuffer> &buffers,
                            void *meta_param, TrackMetaBufferType meta_type,
                            size_t meta_size);

  void NotifyAudioTrackEvent(uint32_t track_id, EventType event_type,
                             void *event_data, size_t event_data_size);


  void NotifyDeleteAudioTrack(uint32_t track_id);

  void NotifyDeleteVideoTrack(uint32_t track_id);

  void NotifyGrabPictureData(BufferDescriptor& buffer);

 private:
  sp<IPlayerServiceCallback> client_cb_handle_;
};

};  // namespace player
};  // namespace qmmf
