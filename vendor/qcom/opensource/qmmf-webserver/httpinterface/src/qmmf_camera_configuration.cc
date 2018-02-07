/*
* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#include <QCamera3VendorTags.h>
#include "qmmf_camera_configuration.h"

namespace qmmf {

namespace httpinterface {

using namespace android;
using namespace qcamera;

const char CameraConfiguration::kModeOff[] = "off";
const char CameraConfiguration::kModeOn[] = "on";
const char CameraConfiguration::kNRModeFast[] = "fast";
const char CameraConfiguration::kNRModeHighQuality[] = "high-quality";
const char CameraConfiguration::kNRModeMinimal[] = "minimal";
const char CameraConfiguration::kNRModeZSL[] = "zsl";

CameraConfiguration::CameraConfiguration(CameraMetadata &static_info) :
    nr_modes_supported_(NULL),
    hdr_modes_supported_(NULL),
    ir_modes_supported_(NULL),
    static_info_(static_info) {

  InitSupportedNRModes();
  InitSupportedHDRModes();
  InitSupportedIRModes();
}

CameraConfiguration::~CameraConfiguration() {
  if (NULL != nr_modes_supported_) {
    free(nr_modes_supported_);
    nr_modes_supported_ = NULL;
  }
  if (NULL != hdr_modes_supported_) {
    free(hdr_modes_supported_);
    hdr_modes_supported_ = NULL;
  }
  if (NULL != ir_modes_supported_) {
    free(ir_modes_supported_);
    ir_modes_supported_ = NULL;
  }
}

void CameraConfiguration::InitSupportedNRModes() {
  camera_metadata_entry_t entry;

  String8 nr_modes_supported;
  nr_parms_to_mode_.clear();
  if (static_info_.exists(
      ANDROID_NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES)) {
    entry = static_info_.find(
        ANDROID_NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES);
    for (uint32_t i = 0 ; i < entry.count; i++) {
      switch(entry.data.u8[i]) {
        case ANDROID_NOISE_REDUCTION_MODE_OFF:
          nr_modes_supported.appendFormat(" %s", kModeOff);
          nr_parms_to_mode_.add(String8(kModeOff),
                                ANDROID_NOISE_REDUCTION_MODE_OFF);
          break;
        case ANDROID_NOISE_REDUCTION_MODE_FAST:
          nr_modes_supported.appendFormat(" %s", kNRModeFast);
          nr_parms_to_mode_.add(String8(kNRModeFast),
                                ANDROID_NOISE_REDUCTION_MODE_FAST);
          break;
        case ANDROID_NOISE_REDUCTION_MODE_HIGH_QUALITY:
          nr_modes_supported.appendFormat(" %s", kNRModeHighQuality);
          nr_parms_to_mode_.add(String8(kNRModeHighQuality),
                                ANDROID_NOISE_REDUCTION_MODE_HIGH_QUALITY);
          break;
        case ANDROID_NOISE_REDUCTION_MODE_MINIMAL:
          nr_modes_supported.appendFormat(" %s", kNRModeMinimal);
          nr_parms_to_mode_.add(String8(kNRModeMinimal),
                                ANDROID_NOISE_REDUCTION_MODE_MINIMAL);
          break;
        case ANDROID_NOISE_REDUCTION_MODE_ZERO_SHUTTER_LAG:
          nr_modes_supported.appendFormat(" %s", kNRModeZSL);
          nr_parms_to_mode_.add(String8(kNRModeZSL),
                                ANDROID_NOISE_REDUCTION_MODE_ZERO_SHUTTER_LAG);
          break;
        default:
          ALOGE("%s Invalid NR mode: %d\n", __func__, entry.data.u8[i]);
      }
    }

    if (!nr_modes_supported.isEmpty()) {
      nr_modes_supported_ = (char *) malloc(nr_modes_supported.size() + 1);
      if (NULL != nr_modes_supported_) {
        memset(nr_modes_supported_, '\0', nr_modes_supported.size() + 1);
        strncpy(nr_modes_supported_, nr_modes_supported.string(),
                nr_modes_supported.size());
      }
    }
  }
}

int32_t CameraConfiguration::SetNRMode(const char *mode, CameraMetadata &meta) {
  // If platform does not support NR, return gracefully.
  if (NULL == nr_modes_supported_) {
    ALOGW("%s: NR modes not available in the platform.\n", __func__);
    return NO_ERROR;
  }

  if (NULL == mode) {
    ALOGE("%s: Invalid NR mode parameter!\n", __func__);
    return BAD_VALUE;
  }

  ssize_t idx = nr_parms_to_mode_.indexOfKey(String8(mode));
  if (NAME_NOT_FOUND != idx) {
    uint8_t mode = nr_parms_to_mode_.valueAt(idx);
    meta.update(ANDROID_NOISE_REDUCTION_MODE, &mode, 1);
  } else {
    ALOGE("%s: NR mode: %s not found in supported modes!\n", __func__, mode);
    return NAME_NOT_FOUND;
  }

  return NO_ERROR;
}

void CameraConfiguration::InitSupportedHDRModes() {
  camera_metadata_entry_t entry;

  String8 hdr_modes_supported;
  hdr_parms_to_mode_.clear();
  if (static_info_.exists(QCAMERA3_AVAILABLE_VIDEO_HDR_MODES)) {
    entry = static_info_.find(QCAMERA3_AVAILABLE_VIDEO_HDR_MODES);
    for (uint32_t i = 0 ; i < entry.count; i++) {
      switch(entry.data.i32[i]) {
        case QCAMERA3_VIDEO_HDR_MODE_OFF:
          hdr_parms_to_mode_.add(String8(kModeOff), entry.data.i32[i]);
          hdr_modes_supported.appendFormat(" %s", kModeOff);
          break;
        case QCAMERA3_VIDEO_HDR_MODE_ON:
          hdr_parms_to_mode_.add(String8(kModeOn), entry.data.i32[i]);
          hdr_modes_supported.appendFormat(" %s", kModeOn);
          break;
        default:
          ALOGE("%s Invalid VHDR mode: %d\n", __func__, entry.data.i32[i]);
      }
    }

    if (!hdr_modes_supported.isEmpty()) {
      hdr_modes_supported_ = (char *) malloc(hdr_modes_supported.size() + 1);
      if (NULL != hdr_modes_supported_) {
        memset(hdr_modes_supported_, '\0', hdr_modes_supported.size() + 1);
        strncpy(hdr_modes_supported_, hdr_modes_supported.string(),
                hdr_modes_supported.size());
      }
    }
  }
}

int32_t CameraConfiguration::SetHDRMode(const char *mode, CameraMetadata &meta) {
  // If platform/camera does not support SHDR, return gracefully.
  if (NULL == hdr_modes_supported_) {
    ALOGW("%s: HDR modes not available in the platform.\n", __func__);
    return NO_ERROR;
  }

  if (NULL == mode) {
    ALOGE("%s: Invalid HDR mode parameter!\n", __func__);
    return BAD_VALUE;
  }

  ssize_t idx = hdr_parms_to_mode_.indexOfKey(String8(mode));
  if (NAME_NOT_FOUND != idx) {
    int32_t mode = hdr_parms_to_mode_.valueAt(idx);
    meta.update(QCAMERA3_VIDEO_HDR_MODE, &mode, 1);
  } else {
    ALOGE("%s: HDR mode: %s not found in supported modes!\n", __func__, mode);
    return NAME_NOT_FOUND;
  }

  return NO_ERROR;
}

void CameraConfiguration::InitSupportedIRModes() {
  camera_metadata_entry_t entry;

  String8 ir_modes_supported;
  ir_parms_to_mode_.clear();
  if (static_info_.exists(QCAMERA3_IR_AVAILABLE_MODES)) {
    entry = static_info_.find(QCAMERA3_IR_AVAILABLE_MODES);
    for (uint32_t i = 0 ; i < entry.count; i++) {
      switch(entry.data.i32[i]) {
        case QCAMERA3_IR_MODE_OFF:
          ir_parms_to_mode_.add(String8(kModeOff), entry.data.i32[i]);
          ir_modes_supported.appendFormat(" %s", kModeOff);
          break;
        case QCAMERA3_IR_MODE_ON:
          ir_parms_to_mode_.add(String8(kModeOn), entry.data.i32[i]);
          ir_modes_supported.appendFormat(" %s", kModeOn);
          break;
        default:
          ALOGE("%s Invalid IR mode: %d\n", __func__, entry.data.i32[i]);
      }
    }

    if (!ir_modes_supported.isEmpty()) {
      ir_modes_supported_ = (char *) malloc(ir_modes_supported.size() + 1);
      if (NULL != ir_modes_supported_) {
        memset(ir_modes_supported_, '\0', ir_modes_supported.size() + 1);
        strncpy(ir_modes_supported_, ir_modes_supported.string(),
                ir_modes_supported.size());
      }
    }
  }
}

int32_t CameraConfiguration::SetIRMode(const char *mode, CameraMetadata &meta) {
  // If platform does not support IR, return gracefully.
  if (NULL == ir_modes_supported_) {
    ALOGW("%s: IR modes not available in the platform.\n", __func__);
    return NO_ERROR;
  }

  if (NULL == mode) {
    ALOGE("%s: Invalid IR mode parameter!\n", __func__);
    return BAD_VALUE;
  }

  ssize_t idx = ir_parms_to_mode_.indexOfKey(String8(mode));
  if (NAME_NOT_FOUND != idx) {
    int32_t mode = ir_parms_to_mode_.valueAt(idx);
    meta.update(QCAMERA3_IR_MODE, &mode, 1);
  } else {
    ALOGE("%s: IR mode: %s not found in supported modes!\n", __func__, mode);
    return NAME_NOT_FOUND;
  }

  return NO_ERROR;
}

} //namespace httpinterface ends here
} //namespace qmmf ends here
