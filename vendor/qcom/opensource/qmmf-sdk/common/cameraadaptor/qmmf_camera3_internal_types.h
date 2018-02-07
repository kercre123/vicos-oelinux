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

#ifndef CAMERA3INTERNALTYPES_H_
#define CAMERA3INTERNALTYPES_H_
#include <hardware/camera_common.h>
#include <hardware/camera3.h>
#include <utils/KeyedVector.h>
#include <camera/CameraMetadata.h>

using namespace android;

namespace qmmf {

namespace cameraadaptor {

class Camera3Stream;

typedef struct : public camera3_stream_t {
  int32_t stream_id;
  GetInputBuffer get_input_buffer;
  ReturnInputBuffer return_input_buffer;
} Camera3InputStream;

typedef struct CaptureRequest_t {
  CameraMetadata metadata;
  Vector<Camera3Stream *> streams;
  CaptureResultExtras resultExtras;
  Camera3InputStream *input;
} CaptureRequest;

typedef List<CaptureRequest> RequestList;

struct PendingRequest {
  int64_t shutterTS;
  int64_t sensorTS;
  CaptureResultExtras resultExtras;
  int status;
  bool isMetaPresent;
  int buffersRemaining;
  CameraMetadata pendingMetadata;
  Vector<camera3_stream_buffer_t> pendingBuffers;

  struct PartialResult {
    bool partial3AReceived;
    CameraMetadata composedResult;

    PartialResult() : partial3AReceived(false) {}
  } partialResult;

  PendingRequest()
      : shutterTS(0),
        sensorTS(0),
        status(OK),
        isMetaPresent(false),
        buffersRemaining(0) {}

  PendingRequest(int numBuffers, CaptureResultExtras extras)
      : shutterTS(0),
        sensorTS(0),
        resultExtras(extras),
        status(OK),
        isMetaPresent(false),
        buffersRemaining(numBuffers) {}
};

typedef KeyedVector<uint32_t, PendingRequest> PendingRequestVector;

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here

#endif /* CAMERA3INTERNALTYPES_H_ */
