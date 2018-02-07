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

#include "recorder/src/client/qmmf_recorder_service_intf.h"

namespace qmmf {

namespace recorder {

class RemoteCallBack : public RefBase {
   public:
    RemoteCallBack(const uint32_t client_id,
                   const sp<IRecorderServiceCallback>& remote_cb);

    ~RemoteCallBack();

    sp<IRecorderServiceCallback>& getRemoteClient() {
       return client_cb_handle_;
    }

    void NotifyRecorderEvent(EventType event_type, void *event_data,
                             size_t event_data_size);

    void NotifySessionEvent(EventType event_type, void *event_data,
                            size_t event_data_size);

    void NotifySnapshotData(uint32_t camera_id, uint32_t image_sequence_count,
                            BnBuffer& buffer, MetaData& meta_data);

    void NotifyVideoTrackData(uint32_t track_id,
                              std::vector<BnBuffer>& buffers,
                              std::vector<MetaData>& meta_buffers);

    void NotifyVideoTrackEvent(uint32_t track_id, EventType event_type,
                               void *event_data, size_t event_data_size);

    void NotifyAudioTrackData(uint32_t track_id,
                              std::vector<BnBuffer>& buffers,
                              std::vector<MetaData>& meta_buffers);

    void NotifyAudioTrackEvent(uint32_t track_id, EventType event_type,
                               void *event_data, size_t event_data_size);

    void NotifyCameraResult(uint32_t camera_id, const CameraMetadata &result);

    void NotifyDeleteVideoTrack(uint32_t track_id);

    uint32_t GetClientId() { return client_id_; }

   private:
    sp<IRecorderServiceCallback> client_cb_handle_;
    uint32_t client_id_;
  };

}; // namespace recorder

}; //namespace qmmf
