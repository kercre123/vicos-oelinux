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

#include "recorder/src/service/qmmf_recorder_common.h"
#include "qmmf_camera3_utils.h"
#include "qmmf_camera3_prepare_handler.h"
#include "qmmf_camera3_stream.h"

namespace qmmf {

namespace cameraadaptor {

Camera3PrepareHandler::Camera3PrepareHandler() :
    prepare_cb_(nullptr),
    prepare_running_(false),
    abort_prepare_(false),
    prepared_stream_(NULL) {
  pthread_mutex_init(&lock_, NULL);
}

Camera3PrepareHandler::~Camera3PrepareHandler() {
  RequestExitAndWait();
  if (NULL != prepared_stream_) {
    prepared_stream_->EndPrepare();
    prepared_stream_ = NULL;
  }
  Clear();

  pthread_mutex_destroy(&lock_);
}

int32_t Camera3PrepareHandler::Prepare(Camera3Stream *stream) {
  int32_t res;

  if (NULL == stream) {
    return -EINVAL;
  }

  pthread_mutex_lock(&lock_);

  res = stream->BeginPrepare();
  if (0 == res) {
    if (nullptr != prepare_cb_) {
      prepare_cb_(stream->GetId());
    }
    res = 0;
    goto exit;
  } else if (-ENODATA != res) {
    goto exit;
  }

  if (!prepare_running_) {
      RequestExitAndWait();
      res = Run("Prepare-Handler");
      if (res != OK) {
          QMMF_ERROR("%s: Failed to start prepare handler: %d (%s)\n",
                     __func__, res, strerror(-res));
          if (nullptr != prepare_cb_) {
            prepare_cb_(stream->GetId());
          }
          goto exit;
      }
      abort_prepare_ = false;
      prepare_running_ = true;
  }

  streams_.push_back(stream);

  res = 0;

exit:

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3PrepareHandler::Clear() {
  pthread_mutex_lock(&lock_);

  for (const auto& stream : streams_) {
      stream->EndPrepare();
  }
  streams_.clear();
  abort_prepare_ = true;

  pthread_mutex_unlock(&lock_);

  return 0;
}

bool Camera3PrepareHandler::ThreadLoop() {
  int32_t res;

  pthread_mutex_lock(&lock_);
  if (NULL == prepared_stream_) {
      if (streams_.empty()) {
          prepare_running_ = false;
          res = false;
          goto exit;
      }
      auto it = streams_.begin();
      prepared_stream_ = *it;
      streams_.erase(it);
  } else if (abort_prepare_) {
    prepared_stream_->EndPrepare();
    prepared_stream_ = NULL;
    abort_prepare_ = false;
    res = true;
    goto exit;
  }
  pthread_mutex_unlock(&lock_);

  res = prepared_stream_->PrepareBuffer();
  if (-ENODATA == res) {
    //This is expected in case we didn't allocate
    //all buffers.
    return true;
  }

  if (0 != res) {
    QMMF_ERROR("%s: Prepare failed: %d (%s) on Stream: %d\n", __func__,
               res, strerror(-res), prepared_stream_->GetId());
    prepared_stream_->EndPrepare();
  }

  pthread_mutex_lock(&lock_);
  if (nullptr != prepare_cb_) {
    prepare_cb_(prepared_stream_->GetId());
  }

  prepared_stream_ = NULL;
  res = true;

exit:

  pthread_mutex_unlock(&lock_);

  return res;
}

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here
