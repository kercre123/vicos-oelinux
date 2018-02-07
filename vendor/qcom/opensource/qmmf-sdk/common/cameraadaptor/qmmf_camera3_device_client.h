/*
 * Copyright (c) 2016 The Linux Foundation. All rights reserved.
 * Not a Contribution.
 */

/*
 * Copyright (C) 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CAMERA3DEVICE_H_
#define CAMERA3DEVICE_H_

#include <pthread.h>
#include <hardware/hardware.h>
#include <camera/CameraMetadata.h>
#include <utils/KeyedVector.h>
#include <utils/List.h>
#include <utils/RefBase.h>

#include "qmmf_camera3_types.h"
#include "qmmf_camera3_internal_types.h"
#include "qmmf_camera3_stream.h"
#include "qmmf_camera3_request_handler.h"
#include "qmmf_camera3_monitor.h"
#include "qmmf_camera3_prepare_handler.h"

extern "C" {
typedef void(callbacks_process_capture_result_t)(
    const struct camera3_callback_ops *, const camera3_capture_result_t *);

typedef void(callbacks_notify_t)(const struct camera3_callback_ops *,
                                 const camera3_notify_msg_t *);

typedef void(camera_device_status_change_t)(
    const struct camera_module_callbacks *, int camera_id, int new_status);

typedef void(torch_mode_status_change_t)(const struct camera_module_callbacks *,
                                         const char *camera_id, int new_status);
}

using namespace android;

namespace qmmf {

namespace cameraadaptor {

class Camera3DeviceClient : public camera3_callback_ops,
                            public camera_module_callbacks_t,
                            public RefBase {
 public:
  Camera3DeviceClient(CameraClientCallbacks clientCb);
  virtual ~Camera3DeviceClient();

  int32_t Initialize();

  int32_t OpenCamera(uint32_t idx);
  int32_t BeginConfigure() { return 0; }
  int32_t EndConfigure(bool isConstrainedHighSpeed = false,
                       bool isRawOnly = false, uint32_t batch_size = 1,
                       bool is_pp_enabled = true);

  int32_t DeleteStream(int streamId, bool cache);
  int32_t CreateStream(const CameraStreamParameters &outputConfiguration);
  int32_t CreateInputStream(
      const CameraInputStreamParameters &inputConfiguration);

  int32_t CreateDefaultRequest(int templateId, CameraMetadata *request);
  int32_t SubmitRequest(Camera3Request request, bool streaming = false,
                        int64_t *lastFrameNumber = NULL);
  int32_t SubmitRequestList(List<Camera3Request> requests,
                            bool streaming = false,
                            int64_t *lastFrameNumber = NULL);
  int32_t ReturnStreamBuffer(StreamBuffer buffer);
  int32_t CancelRequest(int requestId, int64_t *lastFrameNumber = NULL);

  int32_t GetCameraInfo(uint32_t idx, CameraMetadata *info);
  int32_t GetNumberOfCameras() { return number_of_cameras_; }
  const Vector<int32_t> GetRequestIds(){ return current_request_ids_; }
  int32_t WaitUntilIdle();

  int32_t Flush(int64_t *lastFrameNumber = NULL);
  int32_t Prepare(int streamId);
  int32_t TearDown(int streamId);

  static int32_t LoadHWModule(const char *moduleId,
                              const struct hw_module_t **pHmi);

 private:
  Vector<int32_t> current_request_ids_;
  typedef enum State_t {
    STATE_ERROR,
    STATE_NOT_INITIALIZED,
    STATE_CLOSED,
    STATE_NOT_CONFIGURED,
    STATE_CONFIGURED,
    STATE_RUNNING
  } State;

  friend class Camera3PrepareHandler;
  friend class Camera3RequestHandler;
  friend class Camera3Monitor;
  friend class Camera3Gtest;
  friend class DualCamera3Gtest;

  int32_t AddRequestListLocked(const List<const CameraMetadata> &requests,
                               bool streaming, int64_t *lastFrameNumber = NULL);

  void HandleCaptureResult(const camera3_capture_result *result);
  void Notify(const camera3_notify_msg *msg);
  void NotifyError(const camera3_error_msg_t &msg);
  void NotifyShutter(const camera3_shutter_msg_t &msg);
  void RemovePendingRequestLocked(int idx);
  void ReturnOutputBuffers(const camera3_stream_buffer_t *outputBuffers,
                           size_t numBuffers, int64_t timestamp,
                           int64_t frame_number);
  void SendCaptureResult(CameraMetadata &pendingMetadata,
                         CaptureResultExtras &resultExtras,
                         CameraMetadata &collectedPartialResult,
                         uint32_t frameNumber);

  void NotifyStatus(bool idle);
  int32_t WaitUntilDrainedLocked();
  void InternalUpdateStatusLocked(State state);
  int32_t InternalPauseAndWaitLocked();
  int32_t InternalResumeLocked();
  int32_t WaitUntilStateThenRelock(bool active, int64_t timeout);

  int32_t CaclulateBlobSize(int32_t width, int32_t height);
  int32_t QueryMaxBlobSize(int32_t &maxJpegSizeWidth,
                           int32_t &maxJpegSizeHeight);

  int32_t ConfigureStreams(bool isConstrainedHighSpeed = false,
                           bool isRawOnly = false,
                           uint32_t batch_size = 1,bool is_pp_enabled = true);
  int32_t ConfigureStreamsLocked(bool is_pp_enabled = true);

  void SetErrorState(const char *fmt, ...);
  void SetErrorStateV(const char *fmt, va_list args);
  void SetErrorStateLocked(const char *fmt, ...);
  void SetErrorStateLockedV(const char *fmt, va_list args);

  static callbacks_process_capture_result_t processCaptureResult;
  static callbacks_notify_t notifyFromHal;
  static camera_device_status_change_t deviceStatusChange;
  static torch_mode_status_change_t torchModeStatusChange;

  int32_t MarkPendingRequest(uint32_t frameNumber, int32_t numBuffers,
                             CaptureResultExtras resultExtras);

  bool HandlePartialResult(uint32_t frameNumber, const CameraMetadata &partial,
                           const CaptureResultExtras &resultExtras);

  alloc_device_t *GetGrallocDevice() { return gralloc_device_; }

  /**Not allowed */
  Camera3DeviceClient(const Camera3DeviceClient &);
  Camera3DeviceClient &operator=(const Camera3DeviceClient &);

  template <typename T>
  bool QueryPartialTag(const CameraMetadata &result, int32_t tag, T *value,
                       uint32_t frameNumber);
  template <typename T>
  bool UpdatePartialTag(CameraMetadata &result, int32_t tag, const T *value,
                        uint32_t frameNumber);

  int32_t GetRequestListLocked(const List<const CameraMetadata> &metadataList,
                               RequestList *requestList);
  int32_t GenerateCaptureRequestLocked(const CameraMetadata &request,
                                       CaptureRequest &captureRequest);

  pthread_mutex_t pending_requests_lock_;
  PendingRequestVector pending_requests_vector_;

  pthread_mutex_t lock_;
  CameraClientCallbacks client_cb_;

  String8 last_error_;
  uint32_t id_;

  State state_;

  KeyedVector<int, Camera3Stream *> streams_;
  Vector<Camera3Stream *> deleted_streams_;

  int next_stream_id_;
  bool reconfig_;

  CameraMetadata request_templates_[CAMERA3_TEMPLATE_COUNT];
  static const int32_t JPEG_BUFFER_SIZE_MIN =
      256 * 1024 + sizeof(camera3_jpeg_blob);

  camera_module_t *camera_module_;
  camera3_device_t *device_;
  uint32_t number_of_cameras_;
  struct camera_info static_info_;
  CameraMetadata device_info_;
  alloc_device_t *gralloc_device_;

  Vector<int32_t> repeating_requests_;
  int32_t next_request_id_;
  uint32_t frame_number_;
  uint32_t next_shutter_frame_number_;
  uint32_t next_shutter_input_frame_number_;
  uint32_t partial_result_count_;
  bool is_partial_result_supported_;
  uint32_t next_result_frame_number_;
  uint32_t next_result_input_frame_number_;
  vendor_tag_ops_t vendor_tag_ops_;
  Camera3Monitor monitor_;
  Camera3RequestHandler request_handler_;

  bool pause_state_notify_;
  Vector<State> current_state_updates_;
  int state_listeners_;
  pthread_cond_t state_updated_;
  static const int64_t WAIT_FOR_SHUTDOWN = 5e9;  // 5 sec.
  static const int64_t WAIT_FOR_RUNNING = 1e9;   // 1 sec.

  bool is_hfr_supported_;
  bool is_raw_only_;
  bool hfr_mode_enabled_;
  Camera3PrepareHandler prepare_handler_;
  Camera3InputStream input_stream_;
  uint32_t batch_size_;
};

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here

#endif /* CAMERA3DEVICE_H_ */
