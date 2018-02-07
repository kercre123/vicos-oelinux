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
#ifndef CAMERA3PREPAREHANDLER_H_
#define CAMERA3PREPAREHANDLER_H_

#include <pthread.h>
#include <utils/List.h>

#include "qmmf_camera3_types.h"
#include "qmmf_camera3_internal_types.h"
#include "qmmf_camera3_thread.h"

using namespace android;

namespace qmmf {

namespace cameraadaptor {

class Camera3Stream;

class Camera3PrepareHandler : public Camera3Thread {
 public:
  Camera3PrepareHandler();
  virtual ~Camera3PrepareHandler();

  void SetPrepareCb(PreparedCallback prepare_cb) {prepare_cb_ = prepare_cb;};
  int32_t Prepare(Camera3Stream *stream);
  int32_t Clear();

 protected:
  bool ThreadLoop() override;

 private:

  /**Not allowed */
  Camera3PrepareHandler(const Camera3PrepareHandler &);
  Camera3PrepareHandler &operator=(const Camera3PrepareHandler &);

  PreparedCallback prepare_cb_;
  pthread_mutex_t lock_;
  List<Camera3Stream *> streams_;
  bool prepare_running_;
  bool abort_prepare_;
  Camera3Stream *prepared_stream_;
};

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here

#endif /* CAMERA3PREPAREHANDLER_H_ */
