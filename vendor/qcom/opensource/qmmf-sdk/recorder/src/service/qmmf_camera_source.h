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

#include <memory>
#include <mutex>
#include <condition_variable>

#include <camera/CameraMetadata.h>
#include <utils/KeyedVector.h>

#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_camera_interface.h"
#include "recorder/src/service/qmmf_camera_context.h"
#include "recorder/src/service/qmmf_camera_rescaler.h"
#include "common/cameraadaptor/qmmf_camera3_device_client.h"
#include "common/codecadaptor/src/qmmf_avcodec.h"

#include <qmmf-sdk/qmmf_recorder_extra_param_tags.h>

namespace qmmf {

using namespace cameraadaptor;
using namespace android;
using namespace overlay;
using namespace avcodec;

namespace recorder {

#define FPS_CHANGE_THRESHOLD  (0.005)
#define FRAME_SKIP_THRESHOLD_PERCENT (0.05)

class TrackSource;

class CameraSource {
 public:

  static CameraSource* CreateCameraSource();

  ~CameraSource();

  status_t StartCamera(const uint32_t camera_id, const CameraStartParam &param,
                       const ResultCb &cb = nullptr,
                       const ErrorCb &errcb = nullptr);

  status_t StopCamera(const uint32_t camera_id);

  status_t CreateMultiCamera(const std::vector<uint32_t> camera_ids,
                             uint32_t *virtual_camera_id);

  status_t ConfigureMultiCamera(const uint32_t virtual_camera_id,
                                const MultiCameraConfigType type,
                                const void *param,
                                const uint32_t param_size);

  status_t GetSupportedPlugins(SupportedPlugins *plugins);

  status_t CreatePlugin(uint32_t *uid, const PluginInfo &plugin);

  status_t DeletePlugin(const uint32_t &uid);

  status_t ConfigPlugin(const uint32_t &uid, const std::string &json_config);

  status_t CaptureImage(const uint32_t camera_id,
                        const ImageParam &param,
                        const uint32_t num_images,
                        const std::vector<CameraMetadata> &meta,
                        const SnapshotCb &cb);

  status_t ConfigImageCapture(const uint32_t camera_id,
                              const ImageConfigParam &config);

  status_t CancelCaptureImage(const uint32_t camera_id);

  status_t ReturnImageCaptureBuffer(const uint32_t camera_id,
                           const int32_t buffer_id);

  status_t CreateTrackSource(const uint32_t track_id,
                             const VideoTrackParams& param);

  status_t DeleteTrackSource(const uint32_t track_id);

  status_t StartTrackSource(const uint32_t track_id);

  status_t StopTrackSource(const uint32_t track_id,
                           bool is_force_cleanup = false);

  status_t PauseTrackSource(const uint32_t track_id);

  status_t ResumeTrackSource(const uint32_t track_id);

  status_t ReturnTrackBuffer(const uint32_t track_id,
                             std::vector<BnBuffer> &buffers);

  status_t SetCameraParam(const uint32_t camera_id, const CameraMetadata &meta);

  status_t GetCameraParam(const uint32_t camera_id, CameraMetadata &meta);

  status_t GetDefaultCaptureParam(const uint32_t camera_id,
                                  CameraMetadata &meta);

  status_t UpdateTrackFrameRate(const uint32_t track_id,
                                const float frame_rate);

  status_t EnableFrameRepeat(const uint32_t track_id,
                             const bool enable_frame_repeat);

  status_t CreateOverlayObject(const uint32_t track_id,
                               OverlayParam *param,
                               uint32_t *overlay_id);

  status_t DeleteOverlayObject(const uint32_t track_id,
                               const uint32_t overlay_id);

  status_t GetOverlayObjectParams(const uint32_t track_id,
                                  const uint32_t overlay_id,
                                  OverlayParam &param);

  status_t UpdateOverlayObjectParams(const uint32_t track_id,
                                     const uint32_t overlay_id,
                                     OverlayParam *param);

  status_t SetOverlayObject(const uint32_t track_id,
                            const uint32_t overlay_id);

  status_t RemoveOverlayObject(const uint32_t track_id,
                               const uint32_t overlay_id);

  const ::std::shared_ptr<TrackSource>& GetTrackSource(uint32_t track_id);

 private:

  bool IsTrackIdValid(const uint32_t track_id);
  void SnapshotCallback(uint32_t count, StreamBuffer& buffer);
  uint32_t GetJpegSize(uint8_t *blobBuffer, uint32_t width);

  bool ValidateSlaveTrackParam(
    const VideoTrackParams& slave_track,
    const VideoTrackParams& master_track);

  bool CheckLinkedStream(
    const VideoTrackParams& slave_track,
    const VideoTrackParams& master_track);

  status_t GetSlaveStreamMasterTrackId(const VideoTrackParams& params,
                                      int32_t& track_id_master_);

  status_t GetSourceTrackParam(const VideoTrackParams& params,
                               SourceVideoTrack& surface_video_copy);

  bool IsCopyStream(const VideoTrackParams& params);

  // Map of camera id and CameraContext.
  DefaultKeyedVector<uint32_t, sp<CameraInterface>> camera_map_;

  // Map of track it and TrackSources.
  DefaultKeyedVector<uint32_t, ::std::shared_ptr<TrackSource>> track_sources_;

  SnapshotCb client_snapshot_cb_;

  sp<PostProcFactory> factory_;

  // Not allowed
  CameraSource();
  CameraSource(const CameraSource&);
  CameraSource& operator=(const CameraSource&);
  static CameraSource* instance_;
  std::map<int32_t, sp<CameraRescaler> > rescalers_;

};

// This class is behaves as producer and consumer both, at one end it takes
// YUV buffers from camera stream and another end it provides buffers to
// Encoder, and manages buffer circulation, skip etc.
class TrackSource : public ICodecSource {
 public:
  TrackSource(const VideoTrackParams& params,
              const sp<CameraInterface>& camera_intf);

  ~TrackSource();

  status_t Init();

  status_t DeInit();

  status_t StartTrack();

  status_t StopTrack(bool is_force_cleanup = false);

  // Methods of IInputCodecSource
  // This method to provide input buffer to Encoder.
  status_t GetBuffer(BufferDescriptor& buffer, void* client_data) override;

  // This method is used by Encoder to provide buffer back after encoding.
  status_t ReturnBuffer(BufferDescriptor& buffer, void* client_data) override;

  // This method is used by Encoder to notify stop.
  status_t NotifyPortEvent(PortEventType event_type,
                           void* event_data) override;

  // Global track specific params can be query from TrackSource during its life
  // cycle.
  VideoTrackParams& getParams() { return track_params_; }

  // This method to handle incoming buffers from producer, producer can be
  // anyone, Camera context's port or rescaler.
  void OnFrameAvailable(StreamBuffer& buffer);

  status_t ReturnTrackBuffer(std::vector<BnBuffer>& buffers);

  bool IsStop();

  void ClearInputQueue();

  // Overlay Apis. TrackSource has instance of Overlay to deal with static
  // and dynamic types of overlay.
  status_t CreateOverlayObject(OverlayParam *param, uint32_t *overlay_id);

  status_t DeleteOverlayObject(const uint32_t overlay_id);

  status_t GetOverlayObjectParams(const uint32_t overlay_id,
                                  OverlayParam &param);

  status_t UpdateOverlayObjectParams(const uint32_t overlay_id,
                                     OverlayParam *param);

  status_t SetOverlayObject(const uint32_t overlay_id);

  status_t RemoveOverlayObject(const uint32_t overlay_id);

  void UpdateFrameRate(const float frame_rate);

  void EnableFrameRepeat(const bool enable_frame_repeat);

  void NotifyBufferReturned(StreamBuffer& buffer);

  //status_t GetStreamParam(CameraStreamParam& stream_param);

  status_t InitCopy(std::shared_ptr<TrackSource> track_source,
                    const sp<CameraRescaler>& rescaler,
                    int32_t port_track_id,
                    int32_t track_id_master);

  bool IsConnectedToCameraPort() { return connected_tocamera_port_;};

  bool IsSlaveTrack() {return slave_track_source_; };

  int32_t GetMasterTrackId();

  int32_t GetCameraPortId();

  status_t AddConsumer(const sp<IBufferConsumer>& consumer);

  status_t RemoveConsumer(sp<IBufferConsumer>& consumer);

 private:

  // Method to provide consumer interface, it would be used by producer to
  // post buffers.
  sp<IBufferConsumer>& GetConsumerIntf() { return buffer_consumer_impl_; }

  void PushFrameToQueue(StreamBuffer& buffer);

  uint32_t TrackId() { return track_params_.track_id; }

  bool IsEnableFrameSkip();

  bool IsFrameSkip();

  uint32_t CalculateEncodesPerFrame();

#ifdef ENABLE_FRAME_DUMP
  status_t DumpYUV(StreamBuffer& buffer);
#endif

  void ReturnBufferToProducer(StreamBuffer& buffer);

  bool IsNeedScaler(const VideoTrackParams& slave_track,
                    const VideoTrackParams& master_track);

  VideoTrackParams         track_params_;
  sp<IBufferConsumer>      buffer_consumer_impl_;
  bool                     is_stop_;
  Mutex                    stop_lock_;
  bool                     eos_acked_;
  Mutex                    eos_lock_;

  std::mutex               lock_;
  std::condition_variable  wait_for_frame_;

  // will be used till we make stop api as async.
  std::mutex               idle_lock_;
  std::condition_variable  wait_for_idle_;

  // Maps of Unique buffer Id and Buffer.
  DefaultKeyedVector<uint32_t, StreamBuffer> buffer_list_;

  Mutex buffer_list_lock_;

  // Input buffer list, to feed buffers to encoder.
  TSQueue<StreamBuffer> frames_received_;

  // List of buffers held by encoder.
  TSQueue<StreamBuffer> frames_being_encoded_;

  sp<CameraInterface>   camera_interface_;

  Overlay  overlay_;
  uint32_t active_overlays_;

  float   source_frame_rate_;
  float   input_frame_rate_;
  double  input_frame_interval_;
  double  output_frame_interval_;
  double  remaining_frame_skip_time_;
  Mutex   frame_skip_lock_;

  uint32_t debug_fps_;
  struct timeval input_prevtv_;
  uint32_t input_count_;
  struct timeval prevtv_;
  uint32_t count_;

  float      pending_encodes_per_frame_ratio_;
  uint64_t   frame_repeat_ts_prev_;
  uint64_t   frame_repeat_ts_curr_;
  bool       enable_frame_repeat_;
  std::mutex frame_repeat_lock_;
  sp<CameraRescaler>  rescaler_;
  CameraStreamParam stream_param_;

  bool  connected_tocamera_port_;
  bool  slave_track_source_;

  int32_t track_id_master_;
  int32_t port_track_id_;

  sp<IBufferProducer>    buffer_producer_impl_;
  std::mutex             consumer_lock_;
  std::shared_ptr<TrackSource> master_track_;

  std::map<buffer_handle_t, uint32_t >  buffer_map_;
  std::map<buffer_handle_t, StreamBuffer > stream_buffer_map_;

};

}; //namespace recorder

}; //namespace qmmf
