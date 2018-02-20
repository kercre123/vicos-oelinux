/*
* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#ifndef CAMERA3SMOOTHZOOM_H_
#define CAMERA3SMOOTHZOOM_H_

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

typedef struct CropData{
  int32_t left;
  int32_t top;
  int32_t width;
  int32_t height;
}CropData;

class Camera3SmoothZoom {
 public:
  Camera3SmoothZoom();
  virtual ~Camera3SmoothZoom();

  void Initialize();
  void Update(CaptureRequest &request);
  bool IsEnable();
  bool IsGoing();
  void Enable();
  void Disable();
 protected:

 private:
  bool enable_;
  int32_t max_width_;
  int32_t max_height_;
  float zoom_factor_;
  float zoom_step_size_;
  float zoom_step_;
  int32_t zoom_steps_;

  float left_offset_;
  float top_offset_;
  float left_step_ ;
  float top_step_ ;

  CropData crop_current_;
  CropData crop_target_;
};

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here

#endif /* CAMERA3THREAD_H_ */
