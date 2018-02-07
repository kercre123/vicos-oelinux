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
#ifndef CAMERA3REQUESTHANDLER_H_
#define CAMERA3REQUESTHANDLER_H_

#include <pthread.h>
#include <hardware/hardware.h>
#include <camera/CameraMetadata.h>
#include <utils/KeyedVector.h>
#include <utils/List.h>

#include "common/cameraadaptor/qmmf_camera3_types.h"
#include "common/cameraadaptor/qmmf_camera3_internal_types.h"
#include "common/cameraadaptor/qmmf_camera3_thread.h"

using namespace android;

namespace qmmf {

namespace cameraadaptor {

typedef std::function<void(const char *fmt, va_list args)> SetError;
typedef std::function<int32_t(uint32_t frameNumber, int32_t numBuffers,
                              CaptureResultExtras resultExtras)> MarkRequest;

class Camera3Monitor;

class Camera3RequestHandler : public Camera3Thread {
 public:
  Camera3RequestHandler(Camera3Monitor &monitor);
  virtual ~Camera3RequestHandler();

  int32_t Initialize(camera3_device_t *device, ErrorCallback error_cb,
                     MarkRequest mark_cb, SetError set_error);

  int32_t SetRepeatingRequests(const RequestList &requests,
                               int64_t *lastFrameNumber = NULL);
  int32_t ClearRepeatingRequests(int64_t *lastFrameNumber = NULL);

  int32_t QueueRequestList(List<CaptureRequest> &requests,
                           int64_t *lastFrameNumber = NULL);

  int32_t Clear(int64_t *lastFrameNumber = NULL);

  void TogglePause(bool pause);

  bool IsStreamActive(Camera3Stream &stream);
  void FinishConfiguration(uint32_t batch_size);

  void RequestExit() override;
  void RequestExitAndWait() override;

 protected:
  bool ThreadLoop() override;

 private:
  int32_t GetRequest(CaptureRequest &request);
  void ClearCaptureRequest(CaptureRequest &request);
  void HandleErrorRequest(camera3_capture_request_t &request,
                          CaptureRequest &nextRequest,
                          Vector<camera3_stream_buffer_t> &outputBuffers);

  bool WaitOnPause();
  void Resume();

  void SignalError(const char *fmt, ...);

  /**Not allowed */
  Camera3RequestHandler(const Camera3RequestHandler &);
  Camera3RequestHandler &operator=(const Camera3RequestHandler &);

  static const int64_t WAIT_TIMEOUT = 50e6;  // 50 ms

  ErrorCallback error_cb_;
  MarkRequest mark_cb_;
  SetError set_error_;
  camera3_device_t *hal3_device_;

  pthread_mutex_t lock_;
  pthread_cond_t requests_signal_;
  RequestList requests_;
  RequestList streaming_requests_;

  CaptureRequest current_request_;
  CaptureRequest old_request_;

  bool configuration_update_;

  bool toggle_pause_state_;
  bool paused_state_;
  pthread_mutex_t pause_lock_;
  pthread_cond_t toggle_pause_signal_;
  pthread_cond_t pause_state_signal_;

  uint32_t current_frame_number_;
  int64_t streaming_last_frame_number_;

  Camera3Monitor &monitor_;
  int32_t monitor_id_;
  uint32_t batch_size_;
};

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here

#endif /* CAMERA3REQUESTHANDLER_H_ */
