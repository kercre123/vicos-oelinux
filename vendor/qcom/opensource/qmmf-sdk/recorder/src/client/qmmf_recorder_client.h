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

#pragma once

#include <utils/Mutex.h>
#include <utils/RefBase.h>
#include <utils/KeyedVector.h>
#include <camera/CameraMetadata.h>
#include <vector>
#include <map>

#include "common/cameraadaptor/qmmf_camera3_device_client.h"
#include "qmmf-sdk/qmmf_recorder_params.h"
#include "qmmf-sdk/qmmf_recorder_extra_param.h"
#include "recorder/src/client/qmmf_recorder_client_ion.h"
#include "recorder/src/client/qmmf_recorder_service_intf.h"

namespace qmmf {

namespace recorder {

using namespace android;
using namespace cameraadaptor;

class RecorderClient {
 public:
  RecorderClient();

  ~RecorderClient();

  status_t Connect(const RecorderCb& cb);

  status_t Disconnect();

  status_t StartCamera(const uint32_t camera_id,
                       const CameraStartParam &param,
                       const CameraResultCb &result_cb = nullptr);

  status_t StopCamera(const uint32_t camera_id);

  status_t CreateSession(const SessionCb& cb, uint32_t* session_id);

  status_t DeleteSession(const uint32_t session_id);

  status_t StartSession(const uint32_t session_id);

  status_t StopSession(const uint32_t session_id, bool do_flush);

  status_t PauseSession(const uint32_t session_id);

  status_t ResumeSession(const uint32_t session_id);

  status_t GetSupportedPlugins(SupportedPlugins *plugins);

  status_t CreatePlugin(uint32_t *uid, const PluginInfo &plugin);

  status_t DeletePlugin(const uint32_t &uid);

  status_t ConfigPlugin(const uint32_t &uid, const std::string &json_config);

  status_t CreateAudioTrack(const uint32_t session_id, const uint32_t track_id,
                            const AudioTrackCreateParam& param,
                            const TrackCb& cb);

  status_t CreateVideoTrack(const uint32_t session_id, const uint32_t track_id,
                            const VideoTrackCreateParam& param,
                            const TrackCb& cb);

  status_t CreateVideoTrack(const uint32_t session_id, const uint32_t track_id,
                            const VideoTrackCreateParam& param,
                            const VideoExtraParam& extra_param,
                            const TrackCb& cb);

  status_t ReturnTrackBuffer(const uint32_t session_id,
                             const uint32_t track_id,
                             std::vector<BufferDescriptor> &buffers);

  status_t SetAudioTrackParam(const uint32_t session_id,
                              const uint32_t track_id,
                              CodecParamType type, const void *param,
                              size_t param_size);

  status_t SetVideoTrackParam(const uint32_t session_id,
                              const uint32_t track_id,
                              CodecParamType type, const void *param,
                              size_t param_size);

  status_t DeleteAudioTrack(const uint32_t session_id,
                            const uint32_t track_id);

  status_t DeleteVideoTrack(const uint32_t session_id,
                            const uint32_t track_id);

  status_t CaptureImage(const uint32_t camera_id,
                        const ImageParam &param,
                        const uint32_t num_images,
                        const std::vector<CameraMetadata> &meta,
                        const ImageCaptureCb &cb);

  status_t ConfigImageCapture(const uint32_t camera_id,
                              const ImageConfigParam &config);

  status_t CancelCaptureImage(const uint32_t camera_id);

  status_t ReturnImageCaptureBuffer(const uint32_t camera_id,
                                    const BufferDescriptor &buffer);

  status_t SetCameraParam(const uint32_t camera_id, const CameraMetadata &meta);

  status_t GetCameraParam(const uint32_t camera_id, CameraMetadata &meta);

  status_t GetDefaultCaptureParam(const uint32_t camera_id,
                                  CameraMetadata &meta);

  status_t CreateOverlayObject(const uint32_t track_id,
                               const OverlayParam &param,
                               uint32_t *overlay_id);

  status_t DeleteOverlayObject(const uint32_t track_id,
                               const uint32_t overlay_id);

  status_t GetOverlayObjectParams(const uint32_t track_id,
                                  const uint32_t overlay_id,
                                  OverlayParam &param);

  status_t UpdateOverlayObjectParams(const uint32_t track_id,
                                     const uint32_t overlay_id,
                                     const OverlayParam &param);

  status_t SetOverlay(const uint32_t track_id, const uint32_t overlay_id);

  status_t RemoveOverlay(const uint32_t track_id, const uint32_t overlay_id);

  status_t CreateMultiCamera(const std::vector<uint32_t> camera_ids,
                             uint32_t *virtual_camera_id);

  status_t ConfigureMultiCamera(const uint32_t virtual_camera_id,
                                const MultiCameraConfigType type,
                                const void *param,
                                const uint32_t param_size);

  // Callback handlers from service.ap
  void NotifyRecorderEvent(EventType event_type, void *event_data,
                           size_t event_data_size);

  void NotifySessionEvent(EventType event_type, void *event_data,
                          size_t event_data_size);

  void NotifySnapshotData(uint32_t camera_id, uint32_t image_sequence_count,
                          BnBuffer& buffer, MetaData& meta_data);

  void NotifyVideoTrackData(uint32_t track_id,
                            std::vector<BnBuffer>& bn_buffers,
                            std::vector<MetaData>& meta_buffers);

  void NotifyVideoTrackEvent(uint32_t track_id,
                             EventType event_type,
                             void *event_data,
                             size_t event_data_size);

  void NotifyAudioTrackData(uint32_t track_id,
                            const std::vector<BnBuffer>& buffers,
                            const std::vector<MetaData>& meta_buffers);

  void NotifyAudioTrackEvent(uint32_t track_id,
                             EventType event_type,
                             void *event_data,
                             size_t event_data_size);

  void NotifyCameraResult(uint32_t camera_id,
                          const CameraMetadata &result);

 private:

  void UpdateSessionTopology(const uint32_t session_id, const uint32_t track_id,
                             bool /*Add or Delete*/);

  bool CheckServiceStatus();

  void ServiceDeathHandler();

  typedef std::function <void(void)> NotifyServerDeathCB;
  class DeathNotifier : public IBinder::DeathRecipient {
   public:
    DeathNotifier(NotifyServerDeathCB& cb) : notify_server_death_(cb) {}

    void binderDied(const wp<IBinder>&) override {
      ALOGD("RecorderClient:%s: Recorder service died", __func__);
      notify_server_death_();
    }
    NotifyServerDeathCB notify_server_death_;
  };

  vendor_tag_ops_t     vendor_tag_ops_;
  camera_module_t      *camera_module_;
  Mutex                lock_;
  sp<IRecorderService> recorder_service_;
  sp<DeathNotifier>    death_notifier_;
  RecorderCb           recorder_cb_;
  int32_t              ion_device_;
  RecorderClientIon    buffer_ion_;
  uint32_t             client_id_;

  // List of session callbacks.
  DefaultKeyedVector<uint32_t, SessionCb > session_cb_list_;
  // List of Track callbacks.
  DefaultKeyedVector<uint32_t, TrackCb >   track_cb_list_;
  // Capture callback.
  ImageCaptureCb                           image_capture_cb_;
  // Camera result callback
  CameraResultCb                           metadata_cb_;

  typedef struct BufInfo {
    // Transferred ION Id.
    uint32_t ion_fd;
    // Memory mapped buffer.
    void    *pointer;
    // Size
    size_t  frame_len;
    // ION handle
    ion_user_handle_t ion_handle;
  } BufInfo;

  // map <session id, vector<track id> >
  DefaultKeyedVector<uint32_t, Vector<uint32_t> >  sessions_;
  // map <buffer index, buffer_info>
  typedef DefaultKeyedVector<uint32_t, BufInfo> buf_info_map;
  // map <track_id, map <buffer index, buffer_info> >
  DefaultKeyedVector<uint32_t,  buf_info_map> track_buf_map_;

  DefaultKeyedVector<uint32_t, BufInfo> snapshot_buffers_;
};

class ServiceCallbackHandler : public BnRecorderServiceCallback {
 public:

  ServiceCallbackHandler(RecorderClient* client);

  ~ServiceCallbackHandler();

 private:
  //Methods of BnRecorderServiceCallback.
  void NotifyRecorderEvent(EventType event_type, void *event_data,
                           size_t event_data_size) override;

  void NotifySessionEvent(EventType event_type, void *event_data,
                          size_t event_data_size) override;

  void NotifySnapshotData(uint32_t camera_id, uint32_t image_sequence_count,
                          BnBuffer& buffer, MetaData& meta_data) override;

  void NotifyVideoTrackData(uint32_t track_id,
                            std::vector<BnBuffer>& buffers,
                            std::vector<MetaData>& meta_buffers) override;

  void NotifyVideoTrackEvent(uint32_t track_id, EventType event_type,
                             void *event_data,
                             size_t event_data_size) override;

  void NotifyAudioTrackData(uint32_t track_id,
                            const std::vector<BnBuffer>& buffers,
                            const std::vector<MetaData>& meta_buffers) override;

  void NotifyAudioTrackEvent(uint32_t track_id, EventType event_type,
                             void *event_data,
                             size_t event_data_size) override;

  void NotifyCameraResult(uint32_t camera_id,
                          const CameraMetadata &result) override;

  RecorderClient *client_;
};


}; // namespace qmmf

}; // namespace recorder.
