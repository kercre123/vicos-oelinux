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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <cutils/properties.h>
#include <qmmf_camera3_utils.h>
#include <qmmf_camera3_device_client.h>
#include "recorder/src/service/qmmf_recorder_common.h"
#include "qmmf_camera3_smooth_zoom.h"

namespace qmmf {

namespace cameraadaptor {

Camera3SmoothZoom::Camera3SmoothZoom()
    : enable_(false),
      max_width_(0),
      max_height_(0),
      zoom_factor_(1.0f),
      zoom_step_size_(0.1f),
      zoom_step_(0.0f),
      zoom_steps_(0),
      left_offset_(0.0f),
      top_offset_(0.0f),
      left_step_(0.0f),
      top_step_(0.0f){
  crop_current_ = {};

  char prop_val[PROPERTY_VALUE_MAX];
  property_get("persist.qmmf.zoom.step", prop_val, "0.1");
  zoom_step_size_ = atof(prop_val);
}

Camera3SmoothZoom::~Camera3SmoothZoom() {

}

void Camera3SmoothZoom::Update(CaptureRequest &request) {
  if(!enable_)
    return;

  if (max_width_ == 0 || max_height_ == 0) {
    if (request.metadata.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
      max_width_ =
        request.metadata.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES).data.i32[0];
      max_height_ =
        request.metadata.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES).data.i32[1];
      QMMF_INFO("%s: max_width_ = %d, max_height_ = %d\n", __func__,
        max_width_, max_height_);
    }
    if (request.metadata.exists(ANDROID_SCALER_CROP_REGION)) {
      camera_metadata_entry_t entry;
      entry = request.metadata.find(ANDROID_SCALER_CROP_REGION);
      crop_current_.left = crop_target_.left = entry.data.i32[0];
      crop_current_.top = crop_target_.top = entry.data.i32[1];
      crop_current_.width = crop_target_.width = entry.data.i32[2];
      crop_current_.height = crop_target_.height = entry.data.i32[3];
    }
  }

  if (max_width_ <= 0 ||
     max_height_ <= 0 ||
     crop_current_.width <= 0 ||
     crop_current_.height <= 0 ||
     crop_target_.width <= 0 ||
     crop_target_.height <= 0 ||
     zoom_step_size_ <= 0) {
    return;
  }

  if (request.metadata.exists(ANDROID_SCALER_CROP_REGION)) {
    camera_metadata_entry_t entry;
    entry = request.metadata.find(ANDROID_SCALER_CROP_REGION);

    if ((entry.data.i32[0] != crop_current_.left ||
        entry.data.i32[1] != crop_current_.top ||
        entry.data.i32[2] != crop_current_.width ||
        entry.data.i32[3] != crop_current_.height) &&
        zoom_steps_ < 0) {

      crop_target_.left = entry.data.i32[0];
      crop_target_.top = entry.data.i32[1];
      crop_target_.width = entry.data.i32[2];
      crop_target_.height = entry.data.i32[3];

      if (crop_target_.width > max_width_) {
        crop_target_.width = max_width_;
      }

      if (crop_target_.height > max_height_) {
        crop_target_.height = max_height_;
      }

      if (crop_target_.left + crop_target_.width > max_width_) {
        crop_target_.left = max_width_ - crop_target_.width;
      }

      if (crop_target_.top + crop_target_.height > max_height_) {
        crop_target_.top = max_height_ - crop_target_.height;
      }

      QMMF_INFO("%s: Smooth zoom config: crop_target_: %d %d %d %d\n",
        __func__,
        crop_target_.left,
        crop_target_.top,
        crop_target_.width,
        crop_target_.height);

      float zoom_target = static_cast<float>(crop_target_.width) / max_width_;
      zoom_factor_ = static_cast<float>(crop_current_.width) / max_width_;
      zoom_steps_ = static_cast<int32_t>
        (fabs((zoom_target - zoom_factor_) / zoom_step_size_));

      if (zoom_steps_ != 0) {
        zoom_step_ = (zoom_target - zoom_factor_) / zoom_steps_;
        left_offset_ = static_cast<float>(crop_current_.left);
        top_offset_ = static_cast<float>(crop_current_.top);

        left_step_ = static_cast<float>(crop_target_.left - crop_current_.left);
        left_step_ /= zoom_steps_;

        top_step_ = static_cast<float>(crop_target_.top - crop_current_.top);
        top_step_ /= zoom_steps_;
      } else {
        zoom_step_ = 0;
        left_step_ = 0;
        top_step_ = 0;
      }

      QMMF_INFO("%s: Smooth zoom config: zoom_target_=%f zoom_factor_=%f, "
        "zoom_step_=%f, zoom_steps_ = %d\n",
        __func__,
        zoom_target,
        zoom_factor_,
        zoom_step_,
        zoom_steps_);
    }
  }

  if (zoom_steps_ <= 0) {
    crop_current_.left = crop_target_.left;
    crop_current_.top = crop_target_.top;
    crop_current_.width = crop_target_.width;
    crop_current_.height = crop_target_.height;
  } else {
    zoom_factor_ += zoom_step_;
    left_offset_ += left_step_;
    top_offset_ += top_step_;
    crop_current_.width = static_cast<int32_t>(max_width_ * zoom_factor_);
    crop_current_.height =
      (crop_current_.width * crop_target_.height / crop_target_.width);
    crop_current_.left = static_cast<int32_t>(left_offset_);
    crop_current_.top = static_cast<int32_t>(top_offset_);
  }

  QMMF_DEBUG("%s: crop top:%d left:%d width:%d height:%d, zoom: %f\n",
    __func__,
    crop_current_.top,
    crop_current_.left,
    crop_current_.width,
    crop_current_.height,
    zoom_factor_);

  int32_t * crop = reinterpret_cast<int32_t *>(&crop_current_);
  request.metadata.update(ANDROID_SCALER_CROP_REGION, crop, 4);

  if (zoom_steps_ >= 0) {
    zoom_steps_--;
  }
}

bool Camera3SmoothZoom::IsEnable() {
  return enable_;
}

bool Camera3SmoothZoom::IsGoing() {
  return zoom_steps_ >= 0;
}

void Camera3SmoothZoom::Enable() {
  enable_ = true;
}

void Camera3SmoothZoom::Disable() {
  enable_ = false;
}

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here
