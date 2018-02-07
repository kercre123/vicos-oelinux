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

#ifndef CAMERA3TYPES_H_
#define CAMERA3TYPES_H_
#include <functional>
#include <hardware/camera_common.h>
#include <hardware/camera3.h>
#include <hardware/gralloc.h>
#include <camera/CameraMetadata.h>

#include "common/qmmf_common_utils.h"

#define MAX_PLANE 3

using namespace android;

namespace qmmf {

namespace cameraadaptor {

// Please note that you can call all "Camera3DeviceClient" API methods
// from the same context of this callback.
typedef std::function<void(StreamBuffer buffer)> StreamCallback;

typedef struct {
  uint32_t width;
  uint32_t height;
  int32_t format;
  android_dataspace data_space;
  camera3_stream_rotation_t rotation;
  int32_t grallocFlags;
  uint32_t bufferCount;
  StreamCallback cb;
} CameraStreamParameters;

typedef struct Camera3Request_t {
  CameraMetadata metadata;
  Vector<int32_t> streamIds;
} Camera3Request;

typedef struct {
  int32_t  requestId;
  int32_t  burstId;
  uint32_t frameNumber;
  int32_t  partialResultCount;
  bool     input;
} CaptureResultExtras;

typedef struct {
  CameraMetadata metadata;
  CaptureResultExtras resultExtras;
} CaptureResult;

enum CameraErrorCode {
  ERROR_CAMERA_INVALID_ERROR = -1,  // To indicate all invalid error codes
  ERROR_CAMERA_DISCONNECTED = 0,    // Indicates that the camera disconnected
  ERROR_CAMERA_DEVICE = 1,          // Indicates un-recoverable camera error
  ERROR_CAMERA_SERVICE = 2,         // Indicates issue with device client
  ERROR_CAMERA_REQUEST = 3,         // Indicates error during request processing
  ERROR_CAMERA_RESULT = 4,  // Indicates an error when generating request result
  ERROR_CAMERA_BUFFER = 5,  // Indicates an error during buffer processing
};

// Notifies about all sorts of errors that can happen during camera operation
typedef std::function<
    void(CameraErrorCode errorCode, const CaptureResultExtras &resultExtras)>
    ErrorCallback;
// Notifies the client that camera is idle with no pending requests
typedef std::function<void()> IdleCallback;
// Notifies about a shutter event
typedef std::function<void(const CaptureResultExtras &resultExtras,
                           int64_t timestamp)> ShutterCallback;
// Notifies when stream buffers got allocated
typedef std::function<void(int streamId)> PreparedCallback;
// Notifies about a new capture result
typedef std::function<void(const CaptureResult &result)> ResultCallback;

// Please note that these callbacks shouldn't get blocked for long durations.
// Also very important is to not to try and call "Camera3DeviceClient" API
// methods
// from the same context of these callbacks. This can lead to deadlocks!
typedef struct {
  ErrorCallback errorCb;
  IdleCallback idleCb;
  ShutterCallback shutterCb;
  PreparedCallback peparedCb;
  ResultCallback resultCb;
} CameraClientCallbacks;

//Please note that this callbacks need to return as fast as possible
//otherwise the camera framerate can be affected.
typedef std::function<void(StreamBuffer &buffer)> GetInputBuffer;
typedef std::function<void(StreamBuffer &buffer)> ReturnInputBuffer;

typedef struct {
  uint32_t width;
  uint32_t height;
  int32_t format;
  GetInputBuffer get_input_buffer;
  ReturnInputBuffer return_input_buffer;
} CameraInputStreamParameters;

enum {
  NO_IN_FLIGHT_REPEATING_FRAMES = -1,
};

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here

#endif /* CAMERA3TYPES_H_ */
