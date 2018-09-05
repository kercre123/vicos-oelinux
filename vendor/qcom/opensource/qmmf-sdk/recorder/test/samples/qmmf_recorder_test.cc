/*
* Copyright (c) 2016-2018, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "RecorderTest"

#include <string>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <utils/Log.h>
#include <assert.h>
#include <system/graphics.h>
#ifdef ANDROID_O_OR_ABOVE
#include "common/utils/qmmf_common_utils.h"
#else
#include <QCamera3VendorTags.h>
#endif

#include "recorder/test/samples/qmmf_recorder_test.h"
#include "recorder/test/samples/qmmf_recorder_test_wav.h"
#include "recorder/test/samples/qmmf_recorder_test_amr.h"
#include "common/utils/qmmf_log.h"

volatile uint32_t kpi_debug_mask = KPI_DISABLE;
//#define DEBUG
#define TEST_INFO(fmt, args...)  ALOGD(fmt, ##args)
#define TEST_ERROR(fmt, args...) ALOGE(fmt, ##args)
#define TEST_WARN(fmt,args...)   ALOGW(fmt, ##args)
#ifdef DEBUG
#define TEST_DBG  TEST_INFO
#else
#define TEST_DBG(...) ((void)0)
#endif

using namespace qcamera;

using ::std::mutex;
using ::std::unique_lock;
using ::std::vector;


static const char* kDefaultAudioFilenamePrefix =
    "/data/misc/qmmf/recorder_test_audio";

static const char* kDefaultHistogramStatsFilename =
    "/data/histogram_stats.txt";

static const char* kDefaultAECAWBStatsFilename =
    "/data/AEC_AWB_stats.txt";

const char kAutoOrWarmBootModeArgs[] = {
    AutoOrWarmBootModeOptions::kWidth, ':',
    AutoOrWarmBootModeOptions::kHeight, ':',
    AutoOrWarmBootModeOptions::kFps, ':',
    AutoOrWarmBootModeOptions::kTrackType, ':',
    '\n'
};

// Number of histogram color channels.
// Currently 4: R, GR, GB, B
static const int32_t kHistogramColorChannels = 4;
// As per Venus supported range [0-4]
static const int32_t kMinLTRCount = 0;
static const int32_t kMaxLTRCount = 4;

#if USE_SKIA
static const uint32_t kColorLightGray  = 0xFFCCCCCC;
static const uint32_t kColorRed        = 0xFFFF0000;
static const uint32_t kColorLightGreen = 0xFF33CC00;
static const uint32_t kColorLightBlue  = 0xFF189BF2;
#elif USE_CAIRO
static const uint32_t kColorLightGray  = 0xCCCCCCFF;
static const uint32_t kColorRed        = 0xFF0000FF;
static const uint32_t kColorLightGreen = 0x33CC00FF;
static const uint32_t kColorLightBlue  = 0x189BF2FF;
#endif

RecorderTest::RecorderTest() :
            camera_id_(0),
            session_enabled_(false),
            preview_session_id_(-1),
            snapshot_choice_(SnapshotType::kNone),
            dump_aec_awb_stats_(false),
            dump_histogram_stats_(false),
            num_images_(0),
            in_suspend_(true),
            ltr_count_(0),
            camera_error_(false) {
  TEST_INFO("%s: Enter", __func__);
  static_info_.clear();
  use_display = 0;
#ifdef ANDROID_O_OR_ABOVE
  vendor_tag_desc_ = nullptr;
#endif
  TEST_KPI_GET_MASK();
  TEST_INFO("%s: Exit kpi_debug_mask=%d", __func__, kpi_debug_mask);

  char prop_val[PROPERTY_VALUE_MAX];
  property_get(PROP_DUMP_BITSTREAM, prop_val, "1");
  is_dump_bitstream_enabled_ = (atoi(prop_val) == 0) ? false : true;
  property_get(PROP_DUMP_YUV, prop_val, "1");
  is_dump_yuv_enabled_ = (atoi(prop_val) == 0) ? false : true;
  property_get(PROP_DUMP_RAW, prop_val, "1");
  is_dump_raw_enabled_ = (atoi(prop_val) == 0) ? false : true;
  property_get(PROP_DUMP_JPEG, prop_val, "1");
  is_dump_jpg_enabled_ = (atoi(prop_val) == 0) ? false : true;
  property_get(PROP_DUMP_FRAME_FREQ, prop_val, DEFAULT_DUMP_FRAME_FREQ);
  dump_frame_freq_ = atoi(prop_val);

  printf("%s: is_dump_bitstream_enabled_ = %d\n",
           __func__, is_dump_bitstream_enabled_);
  printf("%s: is_dump_yuv_enabled_ = %d\n",
           __func__, is_dump_yuv_enabled_);
  printf("%s: is_dump_raw_enabled_ = %d\n",
           __func__, is_dump_raw_enabled_);
  printf("%s: is_dump_jpg_enabled_ = %d\n",
           __func__, is_dump_jpg_enabled_);
  printf("%s: dump_frame_freq_ = %d\n",
           __func__, dump_frame_freq_);

  TEST_INFO("%s: Exit", __func__);
}

RecorderTest::~RecorderTest() {
  TEST_INFO("%s: Enter", __func__);
  TEST_INFO("%s: Exit", __func__);
}

status_t RecorderTest::Connect() {

  TEST_INFO("%s: Enter", __func__);

  RecorderCb recorder_status_cb;
  recorder_status_cb.event_cb = [&] (EventType event_type, void *event_data,
      size_t event_data_size) { RecorderEventCallbackHandler(event_type,
      event_data, event_data_size); };

  auto ret = recorder_.Connect(recorder_status_cb);
  TEST_INFO("%s: Exit", __func__);

  return ret;
}

status_t RecorderTest::Disconnect() {

  TEST_INFO("%s: Enter", __func__);
  auto ret = recorder_.Disconnect();
  TEST_INFO("%s: Exit", __func__);
  return ret;
}

void RecorderTest::PreviewTrackHandler(uint32_t session_id, uint32_t track_id,
                                       vector<BufferDescriptor> buffers,
                                       vector<MetaData> meta_buffers) {
  TEST_DBG("%s: Enter", __func__);
  recorder_.ReturnTrackBuffer(session_id, track_id, buffers);
  TEST_DBG("%s: Exit", __func__);
}

status_t RecorderTest::AddPreviewTrack() {
  TEST_DBG("%s: Enter", __func__);
  SessionCb session_status_cb;
  uint32_t session_id;
  uint32_t track_id = 1;

  if (preview_session_id_ > -1) {
    TEST_INFO("%s: Preview track is already added.", __func__);
    return NO_ERROR;
  }

  session_status_cb.event_cb = [] (EventType event_type, void *event_data,
    size_t event_data_size) {};
  status_t ret = recorder_.CreateSession(session_status_cb, &session_id);
  if (ret != 0) {
    TEST_ERROR("%s:Error in Create Session", __func__);
    return ret;
  }
  preview_session_id_ = session_id;
  VideoTrackCreateParam video_track_param{};
  video_track_param.camera_id = camera_id_;
  video_track_param.width = 640;
  video_track_param.height = 480;
  video_track_param.frame_rate = 30;
  video_track_param.format_type = VideoFormat::kYUV;
  video_track_param.low_power_mode = 1;
  TrackCb video_track_cb;
  video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
    vector <BufferDescriptor> buffers,
    vector <MetaData> meta_buffers) {
    PreviewTrackHandler(session_id,track_id, buffers, meta_buffers);};
  video_track_cb.event_cb = [] (uint32_t track_id, EventType event_type,
    void *event_data, size_t event_data_size) {};
  ret = recorder_.CreateVideoTrack(preview_session_id_, track_id,
                                   video_track_param, video_track_cb);
  if (ret != 0) {
    TEST_ERROR("%s:Error in Create Video Track", __func__);
    return ret;
  }
  ret = recorder_.StartSession(preview_session_id_);
  if (ret != 0) {
    TEST_ERROR("%s:Error in Start Session", __func__);
    return ret;
  }
  TEST_DBG("%s:%d : Exit", __func__, preview_session_id_);
  return ret;
}

status_t RecorderTest::RemovePreviewTrack() {
  TEST_DBG("%s: %d :Enter", __func__, preview_session_id_);

  if (preview_session_id_ == -1) {
    TEST_INFO("%s: Preview Track is already removed!", __func__);
    return NO_ERROR;
  }

  status_t ret = recorder_.StopSession(preview_session_id_, true);
  if (ret != 0) {
    TEST_ERROR("%s: Failed in stopping the session : ", __func__);
    return ret;
  }
  ret = recorder_.DeleteVideoTrack(preview_session_id_, 1);
  if (ret != 0) {
    TEST_ERROR("%s: Failed in deleting the video track : ", __func__);
    return ret;
  }
  ret = recorder_.DeleteSession(preview_session_id_);
  if (ret != 0) {
    TEST_ERROR("%s: Failed in deleting the session : ", __func__);
    return ret;
  }
  preview_session_id_ = -1;
  TEST_DBG("%s: Exit", __func__);
  return ret;
}

#ifdef ANDROID_O_OR_ABOVE
/**
 * This function can be called only after StartCamera. It tries to fetch
 * tag_id, on success, returns true and fills vendor tag_id. On failure,
 * returns false.
 */
bool RecorderTest::VendorTagSupported(const String8& name,
                                      const String8& section,
                                      uint32_t* tag_id) {
  TEST_DBG("%s: Enter", __func__);
  bool is_available = false;
  status_t result = 0;

  if (nullptr == tag_id) {
    TEST_ERROR("%s: tag_id is not allocated, returning", __func__);
    return false;
  }

  if (nullptr == vendor_tag_desc_.get()) {
    vendor_tag_desc_ = VendorTagDescriptor::getGlobalVendorTagDescriptor();
    if (nullptr == vendor_tag_desc_.get()) {
      TEST_ERROR("%s: Failed in fetching vendor tag descriptor", __func__);
      return false;
    }
  }

  result = vendor_tag_desc_->lookupTag(name, section, tag_id);
  if (0 != result) {
    TEST_ERROR("%s: TagId lookup failed with error: %d", __func__, result);
    return false;
  } else {
    TEST_INFO("%s: name = %s, section = %s, tag_id = 0x%x",
              __func__, name.string(), section.string(), *tag_id);
    is_available = true;
  }

  TEST_DBG("%s: Exit", __func__);
  return is_available;
}

/**
 * This function can be called only after StartCamera. It checks whether
 * tag_id is present in given meta, on success, returns true and fills
 * vendor tag_id. On failure, returns false.
 */
bool RecorderTest::VendorTagExistsInMeta(const CameraMetadata& meta,
                                         const String8& name,
                                         const String8& section,
                                         uint32_t* tag_id) {
  TEST_DBG("%s: Enter", __func__);
  bool is_available = false;

  if (VendorTagSupported(name, section, tag_id)) {
    if (meta.exists(*tag_id)) {
      is_available = true;
    } else {
      TEST_ERROR("%s: TagId does not exist in given meta", __func__);
      return false;
    }
  }

  TEST_DBG("%s: Exit", __func__);
  return is_available;
}
#endif

status_t RecorderTest::GetCurrentAFMode(int32_t camera_id, int32_t& mode) {
  CameraMetadata meta;

  auto status = recorder_.GetCameraParam(camera_id, meta);
  if (NO_ERROR == status) {
     if (meta.exists(ANDROID_CONTROL_AF_MODE)) {
        mode = meta.find(ANDROID_CONTROL_AF_MODE).data.i32[0];
        TEST_DBG("current af mode(%d)",mode);
     } else {
        TEST_ERROR("auto focus not support\n");
        return NAME_NOT_FOUND;
     }
  } else {
    TEST_ERROR("Fail to get camera param, %s, %d\n", strerror(errno), status);
    return BAD_VALUE;
  }

  return NO_ERROR;
}

status_t RecorderTest::ToggleAFMode(int32_t camera_id, const AfMode& af_mode) {
  CameraMetadata meta;
  uint8_t mode;
  status_t ret = NO_ERROR;
  int32_t current_mode = 0;

  switch (af_mode){
    case AfMode::kOff:
      mode = ANDROID_CONTROL_AF_MODE_OFF;
      break;
    case AfMode::kAuto:
      mode = ANDROID_CONTROL_AF_MODE_AUTO;
      break;
    case AfMode::kMacro:
      mode = ANDROID_CONTROL_AF_MODE_MACRO;
      break;
    case AfMode::kContinousVideo:
      mode = ANDROID_CONTROL_AF_MODE_CONTINUOUS_VIDEO;
      break;
    case AfMode::kContinuousPicture:
      mode = ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE;
      break;
    case AfMode::kEdof:
      mode = ANDROID_CONTROL_AF_MODE_EDOF;
      break;
    default:
      TEST_ERROR("Focuse -> Focus mode(%d) not supported! \n",af_mode);
      return BAD_VALUE;
  }

  ret = GetCurrentAFMode(camera_id, current_mode);
  if(NO_ERROR != ret) {
     TEST_ERROR("Fail to get current focus mode\n");
     return ret;
  } else {
    if (current_mode != static_cast<int32_t>(mode)) {
      TEST_ERROR("current focus mode (%d),update focus mode to (%d)now!",
                current_mode,mode);
      meta.update(ANDROID_CONTROL_AF_MODE, &mode, 1);
      auto status = recorder_.SetCameraParam(camera_id, meta);
      if (NO_ERROR != status) {
         TEST_ERROR("Fail to set focus mode param\n");
         ret = status;
      }
    }
  }

 return ret;
}

int32_t RecorderTest::ToggleNR() {
  CameraMetadata meta;
  auto status = recorder_.GetCameraParam(camera_id_, meta);
  if (NO_ERROR == status) {
    if (meta.exists(ANDROID_NOISE_REDUCTION_MODE)) {
      uint8_t mode = meta.find(ANDROID_NOISE_REDUCTION_MODE).data.u8[0];
      nr_modes_iter it = supported_nr_modes_.begin();
      nr_modes_iter next;
      while (it != supported_nr_modes_.end()) {
        if ((*it).first == mode) {
          it++;
          if (it == supported_nr_modes_.end()) {
            next = supported_nr_modes_.begin();
          } else {
            next = it;
          }
          meta.update(ANDROID_NOISE_REDUCTION_MODE, &next->first, 1);
          status = recorder_.SetCameraParam(camera_id_, meta);
          if (NO_ERROR != status) {
            ALOGE("%s Failed to apply: %s\n",
                  __func__, next->second.c_str());
          } else {
              uint8_t current_mode = meta.find(ANDROID_NOISE_REDUCTION_MODE)
                                               .data.u8[0];
              if (current_mode == ANDROID_NOISE_REDUCTION_MODE_OFF ||
                  current_mode == ANDROID_NOISE_REDUCTION_MODE_HIGH_QUALITY) {
                  TEST_KPI_ASYNC_BEGIN("TnrToggle",
                                       static_cast<int32_t>(current_mode));
              }
          }
          break;
        } else {
          it++;
        }
      }
    }
  }

  return status;
}

std::string RecorderTest::GetCurrentNRMode() {
  CameraMetadata meta;
  std::string ret(FEATURE_NOT_AVAILABLE);
  auto status = recorder_.GetCameraParam(camera_id_, meta);
  if (NO_ERROR == status) {
    if (meta.exists(ANDROID_NOISE_REDUCTION_MODE)) {
      uint8_t mode = meta.find(ANDROID_NOISE_REDUCTION_MODE).data.u8[0];
      for (auto it : supported_nr_modes_) {
        if ((it).first == mode) {
          ret = (it).second;
          break;
        }
      }
    }
  }

  return ret;
}

void RecorderTest::InitSupportedNRModes() {
  camera_metadata_entry_t entry;

  if (static_info_.exists(
      ANDROID_NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES)) {
    entry = static_info_.find(
        ANDROID_NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES);
    for (uint32_t i = 0 ; i < entry.count; i++) {
      switch (entry.data.u8[i]) {
        case ANDROID_NOISE_REDUCTION_MODE_OFF:
          supported_nr_modes_.insert(std::make_pair(entry.data.u8[i], "Off"));
          break;
        case ANDROID_NOISE_REDUCTION_MODE_FAST:
          supported_nr_modes_.insert(std::make_pair(entry.data.u8[i], "Fast"));
          break;
        case ANDROID_NOISE_REDUCTION_MODE_HIGH_QUALITY:
          supported_nr_modes_.insert(std::make_pair(entry.data.u8[i],
                                                    "High quality"));
          break;
        case ANDROID_NOISE_REDUCTION_MODE_MINIMAL:
          supported_nr_modes_.insert(std::make_pair(entry.data.u8[i],
                                                    "Minimal"));
          break;
        case ANDROID_NOISE_REDUCTION_MODE_ZERO_SHUTTER_LAG:
          supported_nr_modes_.insert(std::make_pair(entry.data.u8[i], "ZSL"));
          break;
        default:
          ALOGE("%s Invalid NR mode: %d\n", __func__,
                entry.data.u8[i]);
      }
    }
  }
}

int32_t RecorderTest::ToggleVHDR() {
  CameraMetadata meta;
  auto status = recorder_.GetCameraParam(camera_id_, meta);
  if (NO_ERROR == status) {
    uint32_t hdr_mode_vtag;
#ifdef ANDROID_O_OR_ABOVE
    if (VendorTagExistsInMeta(meta, String8("vhdr_mode"),
        String8("org.codeaurora.qcamera3.video_hdr_mode"),
        &hdr_mode_vtag)) {
#else
    hdr_mode_vtag = QCAMERA3_VIDEO_HDR_MODE;
    if (meta.exists(hdr_mode_vtag)) {
#endif
      int32_t mode = meta.find(hdr_mode_vtag).data.i32[0];
      vhdr_modes_iter it = supported_hdr_modes_.begin();
      vhdr_modes_iter next;
      while (it != supported_hdr_modes_.end()) {
        if ((*it).first == mode) {
          it++;
          if (it == supported_hdr_modes_.end()) {
            next = supported_hdr_modes_.begin();
          } else {
            next = it;
          }
          meta.update(hdr_mode_vtag, &next->first, 1);
          status = recorder_.SetCameraParam(camera_id_, meta);
          if (NO_ERROR != status) {
            ALOGE("%s Failed to apply: %s\n",
                  __func__, next->second.c_str());
          } else {
              uint8_t current_mode = meta.find(hdr_mode_vtag).data.u8[0];
              TEST_KPI_ASYNC_BEGIN("ShdrToggle",
                                   static_cast<int32_t>(current_mode));
          }
          break;
        } else {
          it++;
        }
      }
    }
  }

  return status;
}

std::string RecorderTest::GetCurrentVHDRMode() {
  CameraMetadata meta;
  std::string ret(FEATURE_NOT_AVAILABLE);
  uint32_t hdr_mode_vtag;

  auto status = recorder_.GetCameraParam(camera_id_, meta);
  if (NO_ERROR == status) {
#ifdef ANDROID_O_OR_ABOVE
    if (VendorTagExistsInMeta(meta, String8("vhdr_mode"),
        String8("org.codeaurora.qcamera3.video_hdr_mode"),
        &hdr_mode_vtag)) {
#else
    hdr_mode_vtag = QCAMERA3_VIDEO_HDR_MODE;
    if (meta.exists(hdr_mode_vtag)) {
#endif
      int32_t mode = meta.find(hdr_mode_vtag).data.i32[0];
      for (auto it : supported_hdr_modes_) {
        if ((it).first == mode) {
          ret = (it).second;
          break;
        }
      }
    } else {
      //In case camera didn't set default check if there are any
      //modes supported. In case there are set the first available.
      if (!supported_hdr_modes_.empty()) {
        vhdr_modes_iter start = supported_hdr_modes_.begin();
        meta.update(hdr_mode_vtag, &start->first, 1);
        status = recorder_.SetCameraParam(camera_id_, meta);
        if (NO_ERROR != status) {
          ALOGE("%s Failed to apply: %s\n",
                __func__, start->second.c_str());
        }
        ret = start->second;
      }
    }
  }

  return ret;
}

void RecorderTest::InitSupportedVHDRModes() {
  camera_metadata_entry_t entry;
  uint32_t hdr_supported_modes_vtag;

#ifdef ANDROID_O_OR_ABOVE
  if (!VendorTagSupported(String8("vhdr_supported_modes"),
      String8("org.codeaurora.qcamera3.video_hdr_mode"),
      &hdr_supported_modes_vtag)) {
    TEST_WARN("%s: vhdr_supported_modes is not supported", __func__);
    return;
  }
  if (static_info_.exists(hdr_supported_modes_vtag)) {
    entry = static_info_.find(hdr_supported_modes_vtag);
    for (uint32_t i = 0 ; i < entry.count; i++) {
      switch (entry.data.i32[i]) {
        case VideoHDRAvailableModes::kVideoHdrOff:
          supported_hdr_modes_.insert(std::make_pair(entry.data.i32[i], "Off"));
          break;
        case VideoHDRAvailableModes::kVideoHdrOn:
          supported_hdr_modes_.insert(std::make_pair(entry.data.i32[i], "On"));
          break;
        default:
          ALOGE("%s Invalid VHDR mode: %d\n", __func__, entry.data.i32[i]);
      }
    }
  }
#else
  hdr_supported_modes_vtag = QCAMERA3_AVAILABLE_VIDEO_HDR_MODES;
  if (static_info_.exists(hdr_supported_modes_vtag)) {
    entry = static_info_.find(hdr_supported_modes_vtag);
    for (uint32_t i = 0 ; i < entry.count; i++) {
      switch (entry.data.i32[i]) {
        case QCAMERA3_VIDEO_HDR_MODE_OFF:
          supported_hdr_modes_.insert(std::make_pair(entry.data.i32[i], "Off"));
          break;
        case QCAMERA3_VIDEO_HDR_MODE_ON:
          supported_hdr_modes_.insert(std::make_pair(entry.data.i32[i], "On"));
          break;
        default:
          ALOGE("%s Invalid VHDR mode: %d\n", __func__, entry.data.i32[i]);
      }
    }
  }
#endif
}

int32_t RecorderTest::ToggleIR() {
  CameraMetadata meta;
  auto status = recorder_.GetCameraParam(camera_id_, meta);
  if (NO_ERROR == status) {
    if (meta.exists(QCAMERA3_IR_MODE)) {
      int32_t mode = meta.find(QCAMERA3_IR_MODE).data.i32[0];
      ir_modes_iter it = supported_ir_modes_.begin();
      ir_modes_iter next;
      while (it != supported_ir_modes_.end()) {
        if ((*it).first == mode) {
          it++;
          if (it == supported_ir_modes_.end()) {
            next = supported_ir_modes_.begin();
          } else {
            next = it;
          }
          meta.update(QCAMERA3_IR_MODE, &next->first, 1);
          status = recorder_.SetCameraParam(camera_id_, meta);
          if (NO_ERROR != status) {
            ALOGE("%s Failed to apply: %s\n",
                  __func__, next->second.c_str());
          } else {
              uint8_t current_mode = meta.find(QCAMERA3_IR_MODE).data.u8[0];
              TEST_KPI_ASYNC_BEGIN("IrToggle",
                                   static_cast<int32_t>(current_mode));
          }
          break;
        } else {
          it++;
        }
      }
    }
  }

  return status;
}

void RecorderTest::InitSupportedBinningCorrectionModes() {
  camera_metadata_entry_t entry;

  if (static_info_.exists(QCAMERA3_AVAILABLE_BINNING_CORRECTION_MODES)) {
    entry = static_info_.find(QCAMERA3_AVAILABLE_BINNING_CORRECTION_MODES);
    for (uint32_t i = 0 ; i < entry.count; i++) {
      switch (entry.data.i32[i]) {
        case QCAMERA3_BINNING_CORRECTION_MODE_OFF:
          supported_bc_modes_.insert(std::make_pair(entry.data.i32[i], "Off"));
          break;
        case QCAMERA3_BINNING_CORRECTION_MODE_ON:
          supported_bc_modes_.insert(std::make_pair(entry.data.i32[i], "On"));
          break;
        default:
          ALOGW("%s: Invalid binning correction mode: %d,continuing anyway\n"
              ,  __func__, entry.data.i32[i]);
      }
    }
  }
}

int32_t RecorderTest::ToggleBinningCorrectionMode() {
  CameraMetadata meta;
  auto status = recorder_.GetCameraParam(camera_id_, meta);
  if (NO_ERROR == status) {
    if (meta.exists(QCAMERA3_BINNING_CORRECTION_MODE)) {
      int32_t mode = meta.find(QCAMERA3_BINNING_CORRECTION_MODE).data.i32[0];
      bc_modes_iter it = supported_bc_modes_.begin();
      bc_modes_iter next;
      while (it != supported_bc_modes_.end()) {
        if ((*it).first == mode) {
          it++;
          if (it == supported_bc_modes_.end()) {
            next = supported_bc_modes_.begin();
          } else {
            next = it;
          }
          meta.update(QCAMERA3_BINNING_CORRECTION_MODE, &next->first, 1);
          status = recorder_.SetCameraParam(camera_id_, meta);
          if (NO_ERROR != status) {
            ALOGE("%s Failed to apply: %s\n",
                  __func__, next->second.c_str());
          }
          break;
        } else {
          it++;
        }
      }
    }
  }

  return status;
}

std::string RecorderTest::GetCurrentBinningCorrectionMode(int32_t camera_id) {
  CameraMetadata meta;
  std::string ret(FEATURE_NOT_AVAILABLE);
  auto status = recorder_.GetCameraParam(camera_id, meta);
  if (NO_ERROR == status) {
    if (meta.exists(QCAMERA3_BINNING_CORRECTION_MODE)) {
      int32_t mode = meta.find(QCAMERA3_BINNING_CORRECTION_MODE).data.i32[0];
      for (auto it : supported_bc_modes_) {
        if ((it).first == mode) {
          ret = (it).second;
          break;
        }
      }
    } else {
      //In case camera didn't set default check if there are any
      //modes supported. In case there are set the first available.
      if (!supported_bc_modes_.empty()) {
        bc_modes_iter start = supported_bc_modes_.begin();
        meta.update(QCAMERA3_BINNING_CORRECTION_MODE, &start->first, 1);
        status = recorder_.SetCameraParam(camera_id, meta);
        if (NO_ERROR != status) {
          ALOGE("%s Failed to apply: %s\n",
                __func__, start->second.c_str());
        }
        ret = start->second;
      }
    }
  }

  return ret;
}

int32_t RecorderTest::SetBinningCorrectionMode(int32_t camera_id,
                                               const bool& mode) {
  CameraMetadata meta;
  auto ret = recorder_.GetCameraParam(camera_id, meta);
  assert(ret == NO_ERROR);

  int32_t binning_correction_mode = QCAMERA3_BINNING_CORRECTION_MODE_OFF;
  if(mode) { //Enabled
    binning_correction_mode = QCAMERA3_BINNING_CORRECTION_MODE_ON;
  }
  meta.update(QCAMERA3_BINNING_CORRECTION_MODE, &binning_correction_mode, 1);
  ret = recorder_.SetCameraParam(camera_id, meta);

  return ret;
}

void RecorderTest::InitSupportedVideoStabilizationModes() {
  TEST_DBG("%s: Enter", __func__);
  camera_metadata_entry_t entry;
  entry =
      static_info_.find(ANDROID_CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES);
  if (entry.count > 0) {
    for (uint32_t i = 0 ; i < entry.count; i++) {
      switch (entry.data.u8[i]) {
        case ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_OFF:
          supported_vs_modes_.insert(std::make_pair(entry.data.u8[i], "Off"));
          break;
        case ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_ON:
          supported_vs_modes_.insert(std::make_pair(entry.data.u8[i], "On"));
          break;
        default:
          ALOGW(
              "%s Invalid video stabilization mode: %d, continuing anyway\n",
              __func__, entry.data.i32[i]);
      }
    }
  }
  TEST_DBG("%s: Exit", __func__);
}

int32_t RecorderTest::ToggleVideoStabilizationMode() {
  CameraMetadata meta;
  auto status = recorder_.GetCameraParam(camera_id_, meta);
  if (NO_ERROR == status) {
    if (meta.exists(ANDROID_CONTROL_VIDEO_STABILIZATION_MODE)) {
      uint8_t mode =
          meta.find(ANDROID_CONTROL_VIDEO_STABILIZATION_MODE).data.u8[0];
      vs_modes_iter it = supported_vs_modes_.begin();
      vs_modes_iter next;
      while (it != supported_vs_modes_.end()) {
        if ((*it).first == mode) {
          it++;
          if (it == supported_vs_modes_.end()) {
            next = supported_vs_modes_.begin();
          } else {
            next = it;
          }
          meta.update(ANDROID_CONTROL_VIDEO_STABILIZATION_MODE,
                      &next->first, 1);
          status = recorder_.SetCameraParam(camera_id_, meta);
          if (0 != status) {
            ALOGE("%s Failed to apply: %s\n",
                  __func__, next->second.c_str());
            return status;
          }
          break;
        } else {
          it++;
        }
      }
    }
  }

  return status;
}

std::string RecorderTest::GetCurrentVideoStabilizationMode(const int32_t&
                                                           camera_id) {
  CameraMetadata meta;
  std::string ret(FEATURE_NOT_AVAILABLE);
  auto status = recorder_.GetCameraParam(camera_id, meta);
  if (NO_ERROR == status) {
    if (meta.exists(ANDROID_CONTROL_VIDEO_STABILIZATION_MODE)) {
      int32_t mode =
          meta.find(ANDROID_CONTROL_VIDEO_STABILIZATION_MODE).data.i32[0];
      for (auto it : supported_vs_modes_) {
        if ((it).first == mode) {
          ret = (it).second;
          break;
        }
      }
    } else {
      //In case camera didn't set default check if there are any
      //modes supported. In case there are set the first available.
      if (!supported_vs_modes_.empty()) {
        vs_modes_iter start = supported_vs_modes_.begin();
        meta.update(ANDROID_CONTROL_VIDEO_STABILIZATION_MODE,
                    &start->first, 1);
        status = recorder_.SetCameraParam(camera_id, meta);
        if (NO_ERROR != status) {
          ALOGE("%s Failed to apply: %s\n",
                __func__, start->second.c_str());
        }
        ret = start->second;
      }
    }
  }

  return ret;
}

status_t RecorderTest::SetVideoStabilization(const int32_t& camera_id,
                                             const bool& config) {
  TEST_INFO("%s: Enter", __func__);
  CameraMetadata meta;
  auto ret = recorder_.GetDefaultCaptureParam(camera_id_, meta);
  if (ret != 0) {
    TEST_ERROR("%s: Failed to get camera params", __func__);
    return ret;
  }

  if (meta.exists(ANDROID_CONTROL_VIDEO_STABILIZATION_MODE)) {
    uint8_t vstab_mode = ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_OFF;
    if (config) { //Enable Video stabilization
      vstab_mode = ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_ON;
    }
    meta.update(ANDROID_CONTROL_VIDEO_STABILIZATION_MODE, &vstab_mode, 1);
    ret = recorder_.SetCameraParam(camera_id, meta);
    if (ret != 0) {
      TEST_ERROR("%s Failed to set video stabilization\n",
                 __func__);
      return ret;
    }
  }
  TEST_INFO("%s: Exit", __func__);
  return ret;
}

int32_t RecorderTest::ChooseCamera() {

  // TODO - propagate num cameras
  int num_cameras = 3;
  int input;

  do {
    printf("\n");
    printf("****** Current Camera is 0 - %d *******\n", camera_id_);
    printf("    Choose Camera: 0 - %d\n", num_cameras-1);
    printf("\n");
    scanf("%d", &input);
  } while ((input > num_cameras - 1) || (input < 0));
  camera_id_ = input;

  return 0;
}

int32_t RecorderTest::SetAntibandingMode() {

  int input;
  uint8_t mode;
  CameraMetadata meta;

  printf("\n");
  printf("****** Antibanding mode *******\n" );
  printf("  1. Off\n" );
  printf("  2. 50Hz\n" );
  printf("  3. 60Hz\n" );
  printf("  4. Auto\n" );
  printf("  0. exit\n");
  scanf("%d", &input);
  auto status = recorder_.GetCameraParam(camera_id_, meta);

  switch (input) {
    case 1:
      mode = ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF;
      break;
    case 2:
      mode = ANDROID_CONTROL_AE_ANTIBANDING_MODE_50HZ;
      break;
    case 3:
      mode = ANDROID_CONTROL_AE_ANTIBANDING_MODE_60HZ;
      break;
    case 4:
      mode = ANDROID_CONTROL_AE_ANTIBANDING_MODE_AUTO;
      break;
    default:
      return 0;
      break;
  }

  meta.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &mode, 1);
  status = recorder_.SetCameraParam(camera_id_, meta);
  if (NO_ERROR != status) {
    TEST_ERROR("%s Failed to apply: ANDROID_CONTROL_AE_ANTIBANDING_MODE\n",
                __func__);
    return status;
  }

  return 0;
}

std::string RecorderTest::GetCurrentIRMode() {
  CameraMetadata meta;
  std::string ret(FEATURE_NOT_AVAILABLE);
  auto status = recorder_.GetCameraParam(camera_id_, meta);
  if (NO_ERROR == status) {
    if (meta.exists(QCAMERA3_IR_MODE)) {
      int32_t mode = meta.find(QCAMERA3_IR_MODE).data.i32[0];
      for (auto it : supported_ir_modes_) {
        if ((it).first == mode) {
          ret = (it).second;
          break;
        }
      }
    }  else {
      //In case camera didn't set default check if there are any
      //modes supported. In case there are set the first available.
      if (!supported_ir_modes_.empty()) {
        ir_modes_iter start = supported_ir_modes_.begin();
        meta.update(QCAMERA3_IR_MODE, &start->first, 1);
        status = recorder_.SetCameraParam(camera_id_, meta);
        if (NO_ERROR != status) {
          ALOGE("%s Failed to apply: %s\n",
                __func__, start->second.c_str());
        }
        ret = start->second;
      }
    }
  }

  return ret;
}

void RecorderTest::InitSupportedIRModes() {
  camera_metadata_entry_t entry;

  if (static_info_.exists(QCAMERA3_IR_AVAILABLE_MODES)) {
    entry = static_info_.find(QCAMERA3_IR_AVAILABLE_MODES);
    for (uint32_t i = 0 ; i < entry.count; i++) {
      switch (entry.data.i32[i]) {
        case QCAMERA3_IR_MODE_OFF:
          supported_ir_modes_.insert(std::make_pair(entry.data.i32[i], "Off"));
          break;
        case QCAMERA3_IR_MODE_ON:
          supported_ir_modes_.insert(std::make_pair(entry.data.i32[i], "On"));
          break;
        default:
          ALOGE("%s Invalid IR mode: %d\n", __func__,
                entry.data.i32[i]);
      }
    }
  }
}

status_t RecorderTest::GetSharpnessStrength(int32_t& strength) {
  TEST_INFO("%s: Enter", __func__);
  CameraMetadata meta;

  status_t ret = recorder_.GetCameraParam(camera_id_, meta);
  if (ret != 0) {
    TEST_ERROR("%s: Failed to get camera params", __func__);
    return ret;
  }

  // Check if edge mode is OFF.
  // If not, set it to OFF for sharpness value to take effect.
  if (meta.find(ANDROID_EDGE_MODE).data.u8[0] != ANDROID_EDGE_MODE_OFF) {
    const uint8_t edge_mode = ANDROID_EDGE_MODE_OFF;
    meta.update(ANDROID_EDGE_MODE, &edge_mode, 1);
    ret = recorder_.SetCameraParam(camera_id_, meta);
    if (ret != 0) {
      TEST_ERROR("%s Failed to set edge to OFF\n", __func__);
      return ret;
    }
    ret = recorder_.GetCameraParam(camera_id_, meta);
    if (ret != 0) {
      TEST_ERROR("%s: Failed to get camera params", __func__);
      return ret;
    }
  }

  uint32_t sharpness_strength_vtag;
#ifdef ANDROID_O_OR_ABOVE
  if (VendorTagExistsInMeta(meta, String8("strength"),
      String8("org.codeaurora.qcamera3.sharpness"),
      &sharpness_strength_vtag)) {
#else
  sharpness_strength_vtag = QCAMERA3_SHARPNESS_STRENGTH;
  if (meta.exists(sharpness_strength_vtag)) {
#endif
    strength = meta.find(sharpness_strength_vtag).data.i32[0];
  } else {
    TEST_DBG("%s sharpness_strength_vtag does not exist!\n", __func__);

    // In case camera didn't set default.
    // Setting the value to MIN possible by default.
    uint32_t sharpness_range_vtag;
#ifdef ANDROID_O_OR_ABOVE
    if (!VendorTagSupported(String8("range"),
        String8("org.codeaurora.qcamera3.sharpness"),
        &sharpness_range_vtag)) {
      TEST_ERROR("%s: sharpness range is not supported", __func__);
      return -1;
    }
#else
    sharpness_range_vtag = QCAMERA3_SHARPNESS_RANGE;
#endif
    strength = static_info_.find(sharpness_range_vtag).data.i32[0];
    ret = SetSharpnessStrength(strength);
    if (ret != 0) {
      TEST_ERROR("%s Failed to apply sharpness strength:%d\n",
                 __func__, strength);
      return ret;
    }
  }

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::SetSharpnessStrength(const int32_t& val) {
  TEST_INFO("%s: Enter", __func__);
  CameraMetadata meta;

  status_t ret = recorder_.GetCameraParam(camera_id_, meta);
  if (ret != 0) {
    TEST_ERROR("%s: Failed to get camera params", __func__);
    return ret;
  }

  uint32_t sharpness_strength_vtag;
#ifdef ANDROID_O_OR_ABOVE
  if (!VendorTagSupported(String8("strength"),
      String8("org.codeaurora.qcamera3.sharpness"),
      &sharpness_strength_vtag)) {
    TEST_ERROR("%s: sharpness strength is not supported", __func__);
    return -1;
  }
#else
  sharpness_strength_vtag = QCAMERA3_SHARPNESS_STRENGTH;
#endif
  meta.update(sharpness_strength_vtag, &val, 1);
  ret = recorder_.SetCameraParam(camera_id_, meta);
  if (ret != 0) {
    TEST_ERROR("%s Failed to apply sharpness value: %d\n",
               __func__, val);
    return ret;
  }

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::GetSensorSensitivity(int32_t *sensitivity) {
  TEST_INFO("%s: Enter", __func__);
  CameraMetadata meta;

  status_t ret = recorder_.GetCameraParam(camera_id_, meta);
  if (ret != 0) {
    TEST_ERROR("%s: Failed to get camera params", __func__);
    return ret;
  }

  if (meta.exists(ANDROID_SENSOR_SENSITIVITY)) {
    *sensitivity = meta.find(ANDROID_SENSOR_SENSITIVITY).data.i32[0];
  } else {
    TEST_ERROR("%s Meta tag does not exists\n", __func__);
    return -ENOENT;
  }

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::SetSensorSensitivity(const int32_t& val) {
  TEST_INFO("%s: Enter", __func__);
  CameraMetadata meta;

  status_t ret = recorder_.GetCameraParam(camera_id_, meta);
  if (ret != 0) {
    TEST_ERROR("%s: Failed to get camera params", __func__);
    return ret;
  }

  // Check if AE control mode is OFF.
  // If not, set it to OFF for sensitivity value to take effect.
  if (meta.find(ANDROID_CONTROL_AE_MODE).data.u8[0]
      != ANDROID_CONTROL_AE_MODE_OFF) {
    const uint8_t ae_mode = ANDROID_CONTROL_AE_MODE_OFF;
    meta.update(ANDROID_CONTROL_AE_MODE, &ae_mode, 1);
    ret = recorder_.SetCameraParam(camera_id_, meta);
    if (ret != 0) {
      TEST_ERROR("%s Failed to set AE control mode to OFF\n",
                 __func__);
      return ret;
    }
    ret = recorder_.GetCameraParam(camera_id_, meta);
    if (ret != 0) {
      TEST_ERROR("%s: Failed to get camera params", __func__);
      return ret;
    }
  }

  if (meta.exists(ANDROID_SENSOR_SENSITIVITY)) {
    meta.update(ANDROID_SENSOR_SENSITIVITY, &val, 1);
    ret = recorder_.SetCameraParam(camera_id_, meta);
    if (ret != 0) {
      TEST_ERROR("%s Failed to apply sensitivity value: %d\n",
                 __func__, val);
      return ret;
    }
  } else {
    TEST_ERROR("%s Meta tag does not exists\n", __func__);
    return -ENOENT;
  }

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::GetExposureTime(int64_t *time_ns) {
  TEST_INFO("%s: Enter", __func__);
  CameraMetadata meta;

  status_t ret = recorder_.GetCameraParam(camera_id_, meta);
  if (ret != 0) {
    TEST_ERROR("%s: Failed to get camera params", __func__);
    return ret;
  }

  if (meta.exists(ANDROID_SENSOR_EXPOSURE_TIME)) {
    TEST_INFO("%s: Exit", __func__);
    *time_ns = meta.find(ANDROID_SENSOR_EXPOSURE_TIME).data.i64[0];
  } else {
    TEST_ERROR("%s Meta tag does not exists\n", __func__);
    return -ENOENT;
  }

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::SetExposureTime(const int64_t& val) {
  TEST_INFO("%s: Enter", __func__);
  CameraMetadata meta;

  status_t ret = recorder_.GetCameraParam(camera_id_, meta);
  if (ret != 0) {
    TEST_ERROR("%s: Failed to get camera params", __func__);
    return ret;
  }

  if (meta.exists(ANDROID_SENSOR_EXPOSURE_TIME)) {
    meta.update(ANDROID_SENSOR_EXPOSURE_TIME, &val, 1);
    ret = recorder_.SetCameraParam(camera_id_, meta);
    if (ret != 0) {
      TEST_ERROR("%s Failed to apply exposure time value: %lld\n",
                 __func__, val);
      return ret;
    }
  } else {
    TEST_ERROR("%s Meta tag does not exists\n", __func__);
    return -ENOENT;
  }

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::GetWNRStrength(int32_t *wnr_strength) {
  TEST_INFO("%s: Enter", __func__);
  CameraMetadata meta;
  camera_metadata_entry_t entry;

  status_t ret = recorder_.GetCameraParam(camera_id_, meta);
  if (ret != 0) {
    TEST_ERROR("%s: Failed to get camera params", __func__);
    return ret;
  }

  // Check if NR mode is not OFF.
  // If not, set it to FAST for enabling WNR.
  entry = meta.find(ANDROID_NOISE_REDUCTION_MODE);
  if (entry.data.u8[0] != ANDROID_NOISE_REDUCTION_MODE_OFF) {
    const uint8_t nr_mode = ANDROID_NOISE_REDUCTION_MODE_FAST;
    meta.update(ANDROID_NOISE_REDUCTION_MODE,
                &nr_mode, 1);
    ret = recorder_.SetCameraParam(camera_id_, meta);
    if (ret != 0) {
      TEST_ERROR("%s Failed to set AE control mode to OFF\n",
                 __func__);
      return ret;
    }
    ret = recorder_.GetCameraParam(camera_id_, meta);
    if (ret != 0) {
      TEST_ERROR("%s: Failed to get camera params", __func__);
      return ret;
    }
  }

  if (meta.exists(ANDROID_NOISE_REDUCTION_STRENGTH)) {
    entry = meta.find(ANDROID_NOISE_REDUCTION_STRENGTH);
    *wnr_strength = entry.data.u8[0];
  } else {
    // In case camera didn't set default.
    // Setting the value to MIN possible by default.
    uint8_t strength = static_info_.find(QCAMERA3_WNR_RANGE).data.u8[0];
    ret = SetWNRStrength(static_cast<int32_t>(strength));
    if (ret != 0) {
      TEST_ERROR("%s Failed to apply WNR strength: %d\n",
                 __func__, strength);
      *wnr_strength = -1;
      return ret;
    }
    *wnr_strength =  int32_t{strength};
  }

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::SetWNRStrength(const int32_t& val) {
  TEST_INFO("%s: Enter", __func__);
  CameraMetadata meta;

  status_t ret = recorder_.GetCameraParam(camera_id_, meta);
  if (ret != 0) {
    TEST_ERROR("%s: Failed to get camera params", __func__);
    return ret;
  }

  uint8_t strength = static_cast<uint8_t>(val);
  meta.update(ANDROID_NOISE_REDUCTION_STRENGTH, &strength, 1);
  ret = recorder_.SetCameraParam(camera_id_, meta);
  if (ret != 0) {
    TEST_ERROR("%s Failed to apply WNR strength : %u\n",
               __func__, (uint32_t)val);
    return ret;
  }

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::GetTNRIntensity(float *intensity) {
  TEST_INFO("%s: Enter", __func__);
  CameraMetadata meta;

  status_t ret = recorder_.GetCameraParam(camera_id_, meta);
  if (ret != 0) {
    TEST_ERROR("%s: Failed to get camera params", __func__);
    return ret;
  }

  if (meta.exists(QCAMERA3_TNR_INTENSITY)) {
    *intensity = meta.find(QCAMERA3_TNR_INTENSITY).data.f[0];
  } else {
    TEST_ERROR("%s Meta tag does not exists\n", __func__);
    return -ENOENT;
  }

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::SetTNRIntensity(const float& intensity) {
  TEST_INFO("%s: Enter", __func__);
  CameraMetadata meta;

  status_t ret = recorder_.GetCameraParam(camera_id_, meta);
  if (ret != 0) {
    TEST_ERROR("%s: Failed to get camera params", __func__);
    return ret;
  }

  if (meta.exists(QCAMERA3_TNR_INTENSITY)) {
    meta.update(QCAMERA3_TNR_INTENSITY, &intensity, 1);
    ret = recorder_.SetCameraParam(camera_id_, meta);
    if (ret != 0) {
      TEST_ERROR("%s Failed to apply TNR intensity value: %f\n",
                 __func__, intensity);
      return ret;
    }
  } else {
    TEST_ERROR("%s Meta tag does not exists\n", __func__);
    ret = -ENOENT;
  }

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::GetTNRMotionDetectionSensitivity(float *sensitivity) {
  TEST_INFO("%s: Enter", __func__);
  CameraMetadata meta;

  status_t ret = recorder_.GetCameraParam(camera_id_, meta);
  if (ret != 0) {
    TEST_ERROR("%s: Failed to get camera params", __func__);
    return ret;
  }

  if (meta.exists(QCAMERA3_TNR_MOTION_DETECTION_SENSITIVITY)) {
    *sensitivity =
        meta.find(QCAMERA3_TNR_MOTION_DETECTION_SENSITIVITY).data.f[0];
  } else {
    TEST_ERROR("%s Meta tag does not exists\n", __func__);
    return -ENOENT;
  }

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::SetTNRMotionDetectionSensitivity(const float&
                                                            sensitivity) {
  TEST_INFO("%s: Enter", __func__);
  CameraMetadata meta;

  status_t ret = recorder_.GetCameraParam(camera_id_, meta);
  if (ret != 0) {
    TEST_ERROR("%s: Failed to get camera params", __func__);
    return ret;
  }

  if (meta.exists(QCAMERA3_TNR_MOTION_DETECTION_SENSITIVITY)) {
    meta.update(QCAMERA3_TNR_MOTION_DETECTION_SENSITIVITY, &sensitivity, 1);
    ret = recorder_.SetCameraParam(camera_id_, meta);
    if (ret != 0) {
      TEST_ERROR("%s Failed to apply sensitivity value: %f\n",
                 __func__, sensitivity);
      return ret;
    }
  } else {
    TEST_ERROR("%s Meta tag does not exists\n", __func__);
    return -ENOENT;
  }

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::SetTNRLevel() {
  TEST_INFO("%s: Enter", __func__);
  status_t ret = 0;

  char input;
  float tnr_tuning_min, tnr_tuning_max;
  CameraMetadata meta;
  camera_metadata_entry_t entry;

  if (static_info_.exists(QCAMERA3_TNR_TUNING_RANGE)) {
    entry = static_info_.find(QCAMERA3_TNR_TUNING_RANGE);
    tnr_tuning_min = entry.data.f[0];
    tnr_tuning_max = entry.data.f[1];
  } else {
    TEST_ERROR("%s Meta tag does not exists\n", __func__);
    return -ENOENT;
  }

  do {
      std::cout << std::endl;
      std::cout << "****** Set TNR level *******" << std::endl;
      std::cout << "  1. TNR Intensity" << std::endl;
      std::cout << "  2. TNR Motion Detection Sensitivity" << std::endl;
      std::cout << "  X. exit" << std::endl;
      std::cout << std::endl;
      std::cout << "Enter option" << std::endl;
      std::cin >> input;

    ret = recorder_.GetCameraParam(camera_id_, meta);
    if (ret != 0) {
      TEST_ERROR("%s: Failed to get camera params", __func__);
      return ret;
    }

    std::cout << std::fixed;
    std::cout.precision(2);
    switch (static_cast<TNRTuningCmd>(input)) {
      case TNRTuningCmd::kTNRIntensity: {
        float intensity;
        ret = GetTNRIntensity(&intensity);
        std::cout << "Enter TNR Intensity (Current:" << intensity;
        std::cout << ")[" << tnr_tuning_min << '-';
        std::cout << tnr_tuning_max <<"]: ";
        std::cin >> intensity;
        ret = SetTNRIntensity(intensity);
        break;
      }
      case TNRTuningCmd::kMotionDetectionSensitivity: {
        float sensitivity;
        ret = GetTNRMotionDetectionSensitivity(&sensitivity);
        std::cout << "Enter TNR Motion Detection Sensitivity (Current:";
        std::cout << sensitivity << ")[" << tnr_tuning_min << '-';
        std::cout << tnr_tuning_max <<"]: ";
        std::cin >> sensitivity;
        ret = SetTNRMotionDetectionSensitivity(sensitivity);
        break;
      }
      case TNRTuningCmd::kExit:
        break;
      default:
        std::cout << "Wrong value entered:" << input << std::endl;
    }
    if (input == static_cast<char>(TNRTuningCmd::kExit)
        || (ret != 0)) {
      break;
    }
  } while (input);

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::GetRawHistogramStatistic(const CameraMetadata& meta) {
  TEST_INFO("%s: Enter", __func__);

  std::ofstream hist_stats_file;
  hist_stats_file.open(kDefaultHistogramStatsFilename, std::ofstream::app);
  TEST_INFO("Dumping raw histogram stats to %s\n",
      kDefaultHistogramStatsFilename);

  uint32_t histogram_stats_vtag;
  uint32_t histogram_buckets_vtag;
#ifdef ANDROID_O_OR_ABOVE
  if (VendorTagExistsInMeta(meta, String8("stats"),
      String8("org.codeaurora.qcamera3.histogram"),
      &histogram_stats_vtag)) {

    if (!VendorTagSupported(String8("buckets"),
        String8("org.codeaurora.qcamera3.histogram"),
        &histogram_buckets_vtag)) {
      return -EINVAL;
    }
#else
  histogram_stats_vtag = QCAMERA3_HISTOGRAM_STATS;
  if (meta.exists(histogram_stats_vtag)) {
    histogram_buckets_vtag = QCAMERA3_HISTOGRAM_BUCKETS;
#endif
    uint32_t buckets = static_cast<uint32_t>(
        static_info_.find(histogram_buckets_vtag).data.i32[0]);
    hist_stats_file << "Buckets=" << buckets << std::endl;

    camera_metadata_ro_entry entry;
    entry = meta.find(histogram_stats_vtag);

    const char* channels[] = {"R", "GR", "GB", "B"};
    const int32_t *channel_stats_ptr[4];
    uint32_t i,j;
    for (i=0; i < kHistogramColorChannels; ++i) {
      channel_stats_ptr[i] = entry.data.i32 + i;
      hist_stats_file << std::endl << channels[i] << "channel:" << std::endl;
      for (j = 0; j < buckets; ++j) {
        if (j % 16 == 0) {
          hist_stats_file << std::endl;
        }
        hist_stats_file << static_cast<uint32_t>(channel_stats_ptr[i][j]);
        hist_stats_file << ' ';
      }
      hist_stats_file << std::endl;
    }
    hist_stats_file.close();
  } else {
    TEST_ERROR("%s Meta tag does not exists\n", __func__);
    hist_stats_file.close();
    return -ENOENT;
  }

  CameraMetadata temp_meta;
  auto ret = recorder_.GetCameraParam(camera_id_, temp_meta);
  if (ret != 0) {
    TEST_ERROR("%s: Failed to get camera params", __func__);
    return ret;
  }

  // Setting mode to OFF after dumping
  uint32_t histogram_mode_vtag;
#ifdef ANDROID_O_OR_ABOVE
  if (VendorTagExistsInMeta(temp_meta, String8("enable"),
      String8("org.codeaurora.qcamera3.histogram"),
      &histogram_mode_vtag)) {
    const uint8_t hist_mode = StatisticsHistogramModeValues
        ::kStatisticsHistogramModeOff;
#else
  histogram_mode_vtag = QCAMERA3_HISTOGRAM_MODE;
  if (temp_meta.exists(histogram_mode_vtag)) {
    const uint8_t hist_mode = QCAMERA3_HISTOGRAM_MODE_OFF;
#endif
    temp_meta.update(histogram_mode_vtag, &hist_mode, 1);
    ret = recorder_.SetCameraParam(camera_id_, temp_meta);
    if (ret != 0) {
      TEST_ERROR("%s Failed to set Histogram mode to OFF\n", __func__);
      return ret;
    } else {
      TEST_INFO("Histogram mode set to off\n");
    }
  }

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::GetRawAECAWBStatistic(const CameraMetadata& meta) {
  TEST_INFO("%s: Enter", __func__);

  std::ofstream raw_stats_file;
  raw_stats_file.open(kDefaultAECAWBStatsFilename, std::ofstream::app);

  TEST_INFO("Dumping raw AEC AWB stats to %s\n",
            kDefaultAECAWBStatsFilename);
  camera_metadata_ro_entry  h_num, v_num, pixel_cnt, height, width;
  camera_metadata_ro_entry r_sum, b_sum, gr_sum, gb_sum;
  camera_metadata_ro_entry r_num, b_num, gr_num, gb_num;


  h_num = meta.find(QCAMERA3_EXPOSURE_DATA_REGION_H_NUM);
  v_num = meta.find(QCAMERA3_EXPOSURE_DATA_REGION_V_NUM);
  pixel_cnt = meta.find(QCAMERA3_EXPOSURE_DATA_REGION_PIXEL_CNT);
  height = meta.find(QCAMERA3_EXPOSURE_DATA_REGION_HEIGHT);
  width = meta.find(QCAMERA3_EXPOSURE_DATA_REGION_WIDTH);

  r_sum = meta.find(QCAMERA3_EXPOSURE_DATA_R_SUM);
  b_sum = meta.find(QCAMERA3_EXPOSURE_DATA_B_SUM);
  gr_sum = meta.find(QCAMERA3_EXPOSURE_DATA_GR_SUM);
  gb_sum = meta.find(QCAMERA3_EXPOSURE_DATA_GB_SUM);

  r_num = meta.find(QCAMERA3_EXPOSURE_DATA_R_NUM);
  b_num = meta.find(QCAMERA3_EXPOSURE_DATA_B_NUM);
  gr_num = meta.find(QCAMERA3_EXPOSURE_DATA_GR_NUM);
  gb_num = meta.find(QCAMERA3_EXPOSURE_DATA_GB_NUM);


  if (h_num.count && v_num.count && pixel_cnt.count && height.count &&
      width.count && r_sum.count && b_sum.count && gr_sum.count &&
      gb_sum.count && r_num.count && b_num.count && gr_num.count &&
      gb_num.count) {
    raw_stats_file << "Region h_num= " << std::hex << h_num.data.i32[0];
    raw_stats_file << ", Region v_num= " << std::hex << v_num.data.i32[0];
    raw_stats_file << ", Pixel count= " << std::hex << pixel_cnt.data.i32[0];
    raw_stats_file << ", Region height= " << std::hex << height.data.i32[0];
    raw_stats_file << ", Region width= " << std::hex << width.data.i32[0];
    raw_stats_file << std::endl;

    int32_t num_regions = h_num.data.i32[0] * v_num.data.i32[0];

    raw_stats_file << "     r_sum     b_sum    gr_sum    gb_sum";
    raw_stats_file << "     r_num     b_num    gr_num    gb_num" << std::endl;
    int32_t i;
    for (i = 0; i < num_regions; ++i) {
      raw_stats_file << std::setw (10) << std::hex << r_sum.data.i32[i];
      raw_stats_file << std::setw (10) << std::hex << b_sum.data.i32[i];
      raw_stats_file << std::setw (10) << std::hex << gr_sum.data.i32[i];
      raw_stats_file << std::setw (10) << std::hex << gb_sum.data.i32[i];
      raw_stats_file << std::setw (10) << std::hex << r_num.data.i32[i];
      raw_stats_file << std::setw (10) << std::hex << b_num.data.i32[i];
      raw_stats_file << std::setw (10) << std::hex << gr_num.data.i32[i];
      raw_stats_file << std::setw (10) << std::hex << gb_num.data.i32[i];
      raw_stats_file << std::endl;
    }
    raw_stats_file.close();
  } else {
    TEST_ERROR("%s Meta tag does not exists\n", __func__);
    raw_stats_file.close();
    return -ENOENT;
  }

  // Disabling data after dumping.
  CameraMetadata temp_meta;
  status_t ret = recorder_.GetCameraParam(camera_id_, temp_meta);
  if (ret != 0) {
    TEST_ERROR("%s: Failed to get camera params", __func__);
    return ret;
  }
  const int32_t mode = QCAMERA3_EXPOSURE_DATA_OFF;
  temp_meta.update(QCAMERA3_EXPOSURE_DATA_ENABLE, &mode, 1);
  ret = recorder_.SetCameraParam(camera_id_, temp_meta);
  if (ret != 0) {
    TEST_ERROR("%s Failed to set edge to OFF\n",
               __func__);
    return ret;
  } else {
    TEST_INFO("AEC AWB stats mode set to off\n");
  }

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::StartCamera() {

  TEST_INFO("%s: Enter", __func__);

  CameraStartParam camera_params{};
  camera_params.zsl_mode            = false;
  camera_params.zsl_queue_depth     = 10;
  camera_params.zsl_width           = 3840;
  camera_params.zsl_height          = 2160;
  camera_params.frame_rate          = 30;
  camera_params.flags               = 0x0;
  camera_params.enable_partial_metadata  = false;
  CameraResultCb result_cb = [&] (uint32_t camera_id,
            const CameraMetadata &result) {
            CameraResultCallbackHandler(camera_id, result); };
  if (kpi_debug_mask) {
    kpi_marker_.SetUp();
  }
  auto ret = recorder_.StartCamera(camera_id_, camera_params, result_cb);
  if(ret != 0) {
      ALOGE("%s StartCamera Failed!!", __func__);
  }

  ret = recorder_.GetDefaultCaptureParam(camera_id_, static_info_);
  if (NO_ERROR != ret) {
    ALOGE("%s Unable to query default capture parameters!\n",
          __func__);
  } else {
    InitSupportedNRModes();
    InitSupportedVHDRModes();
    InitSupportedIRModes();
    InitSupportedBinningCorrectionModes();
    InitSupportedVideoStabilizationModes();
  }
  TEST_INFO("%s: Exit", __func__);
  return 0;
}

status_t RecorderTest::StopCamera() {

  TEST_INFO("%s: Enter", __func__);

  auto ret = recorder_.StopCamera(camera_id_);
  if(ret != 0) {
    ALOGE("%s StopCamera Failed!!", __func__);
  }

  static_info_.clear();
  TEST_INFO("%s: Exit", __func__);
  return 0;
}

status_t RecorderTest::TakeSnapshotWithConfig(const SnapshotInfo&
                                          snapshot_info) {

  TEST_INFO("%s: Enter", __func__);
  int32_t ret = 0;
  camera_metadata_entry_t entry;
  CameraMetadata meta;
  ret = recorder_.GetDefaultCaptureParam(snapshot_info.camera_id, meta);
  assert(ret == 0);

  ImageParam image_param{};

  image_param.width = snapshot_info.width;
  image_param.height = snapshot_info.height;

  switch (snapshot_info.type) {
    case SnapshotType::kNone:
      TEST_INFO("Snapshot format(%d) is not correct\n",snapshot_info.type);
      break;
    case SnapshotType::kJpeg:
      // Check Supported JPEG snapshot resolutions.
      if (meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
        entry = meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
        for (uint32_t i = 0 ; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == entry.data.i32[i]) {
           if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
               entry.data.i32[i+3]) {
             if ( (0 == snapshot_info.width)
               && (0 == snapshot_info.height) ){
               image_param.width = (uint32_t)entry.data.i32[i+1];
               image_param.height = (uint32_t)entry.data.i32[i+2];
               TEST_INFO("Snapshot JPEG size %dx%d is support\n",
                       image_param.width,image_param.height);
               break;
             }
             if ( (image_param.width == (uint32_t)entry.data.i32[i+1])
               && (image_param.height == (uint32_t)entry.data.i32[i+2]) ){
                TEST_INFO("Snapshot size %dx%d is support\n",
                        image_param.width,image_param.height);
                break;
             }
           }
         }
       }
     }

     image_param.image_format  = ImageFormat::kJPEG;
     image_param.image_quality = 95;
     break;
   case SnapshotType::kRawYuv:
     // Check available raw YUV resolutions.
     if (meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
       entry = meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
       for (uint32_t i = 0 ; i < entry.count; i += 4) {
         if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == entry.data.i32[i]) {
           if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
             entry.data.i32[i+3]) {
             TEST_INFO("%s:(%d) Supported Raw YUV:(%d)x(%d)",
                    __func__, i, entry.data.i32[i+1], entry.data.i32[i+2]);
             if ( (0 == snapshot_info.width)
               && (0 == snapshot_info.height) ){
               image_param.width = (uint32_t)entry.data.i32[i+1];
               image_param.height = (uint32_t)entry.data.i32[i+2];
               TEST_INFO("RAW YUV size %dx%d\n is support\n",
                 image_param.width,image_param.height);
               break;
              }
              if ( (image_param.width == (uint32_t)entry.data.i32[i+1])
                 && (image_param.height == (uint32_t)entry.data.i32[i+2]) ){
                 TEST_INFO("RAW YUV size %dx%d is support\n",
                          image_param.width,image_param.height);
              }
            }
          }
        }
      }

      image_param.image_format = ImageFormat::kNV12;
      break;
    case SnapshotType::kRawRdi:
      if (meta.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
          entry = meta.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES);
        for (uint32_t i = 0 ; i < entry.count; i += 2) {
          TEST_INFO("%s: (%d) Supported RAW RDI W(%d):H(%d)\n",
                __func__, i, entry.data.i32[i+0], entry.data.i32[i+1]);
          image_param.width = (uint32_t)entry.data.i32[i+0];
          image_param.height = (uint32_t)entry.data.i32[i+1];
          break;
        }
      }

      image_param.image_format = ImageFormat::kBayerRDI10BIT;
      break;
    default:
      TEST_INFO("Snapshot format(%d) is not correct\n",snapshot_info.type);
      return BAD_VALUE;
  }

  if (snapshot_info.type  != SnapshotType::kNone) {
    ImageCaptureCb cb = [&] (uint32_t camera_id_, uint32_t image_count,
                             BufferDescriptor buffer, MetaData meta_data)
          { SnapshotCb(camera_id_, image_count, buffer, meta_data); };

    assert(ret == NO_ERROR);

    std::vector<CameraMetadata> meta_array;
    for (uint32_t i = 0; i < snapshot_info.count; i++) {
      meta_array.push_back(meta);
    }
    int32_t repeat = snapshot_info.count;
    bool is_cancel_snapshot = false;
    is_cancel_snapshot = is_test_cancel_snapshot();

    do {
      {
        std::lock_guard<std::mutex> lk(error_lock_);
        camera_error_ = false;
      }
      TEST_INFO("CaptureImage size %dx%d images count %d\n",
                image_param.width,image_param.height,snapshot_info.count);
      ret = recorder_.CaptureImage(snapshot_info.camera_id, image_param,
                                   snapshot_info.count,
                                   meta_array, cb);
      if(ret != 0) {
        ALOGE("%s CaptureImage Failed", __func__);
      }
      std::unique_lock<std::mutex> lock(snapshot_wait_lock_);
      burst_snapshot_count_ = snapshot_info.count;
      std::chrono::milliseconds wait_time(get_snapshot_cb_wait_time() * 1000);

      if (snapshot_wait_signal_.WaitFor(lock, wait_time) != 0) {
           TEST_ERROR("%s Capture Image Timed out", __func__);
      }
      {
        std::lock_guard<std::mutex> lk(error_lock_);
        if (!camera_error_) {
          TEST_ERROR("%s Capture Image Done", __func__);
          break;
        }
      }
      TEST_ERROR("%s Restart TakeSnapshot", __func__);
    } while (repeat-- > 0);

    if (is_cancel_snapshot) {
      ret = CancelTakeSnapshot();
      if (ret != 0) {
        TEST_ERROR("%s CancelTakeSnapshot Failed", __func__);
      }
    }
  }

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CancelTakeSnapshot() {

   TEST_INFO("%s: Enter", __func__);
   int32_t ret = 0;

   ret = recorder_.CancelCaptureImage(camera_id_);
   if (ret != 0) {
      TEST_ERROR("%s CancelTakeSnapshot Failed", __func__);
   }

   TEST_INFO("%s: Exit", __func__);
   return ret;
}

status_t RecorderTest::TakeSnapshot() {
  TEST_INFO("%s: Enter", __func__);
  int32_t ret = 0;
  char input;
  camera_metadata_entry_t entry;
  bool flag_exit = false;
  CameraMetadata meta;
  ret = recorder_.GetDefaultCaptureParam(camera_id_, meta);
  assert(ret == 0);
  snapshot_choice_ = SnapshotType::kNone;
  do {
    num_images_ = 1;
    std::cout << std::endl;
    std::cout << "****** Take Snapshot *******" << std::endl;
    std::cout << "  1. JPEG - 4K " << std::endl;
    std::cout << "  2. RAW:YUV - 1080p " << std::endl;
    std::cout << "  3. RAW:BAYER RDI 10 bit" << std::endl;
    std::cout << "  4. JPEG Burst (30 frames) - 1080p " << std::endl;
    std::cout << "  0. exit " << std::endl;
    std::cout << "  Enter option:" << std::endl;
    std::cin >> input;
    uint32_t w, h;
    ImageParam image_param{};
    switch (input) {
      case '0':
        flag_exit = true;
        break;
      case '1':
        snapshot_choice_ = SnapshotType::kJpeg;
        if (session_enabled_ == false) {
          ret = AddPreviewTrack();
          if (ret != 0) {
            TEST_ERROR("%s:Error in AddPreview Track", __func__);
            return ret;
          }
        }
        image_param.width = 3840;
        image_param.height = 2160;
        image_param.image_format = ImageFormat::kJPEG;
        image_param.image_quality = 95;
        break;
      case '2':
        snapshot_choice_ = SnapshotType::kRawYuv;
        if (session_enabled_ == false) {
          ret = AddPreviewTrack();
          if (ret != 0) {
            TEST_ERROR("%s:Error in AddPreview Track", __func__);
            return ret;
          }
        }
        if (meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
          entry = meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
          for (uint32_t i = 0; i < entry.count; i += 4) {
            if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == entry.data.i32[i]) {
              if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT
                  == entry.data.i32[i + 3]) {
                TEST_INFO("%s:(%d) Supported Raw YUV:(%d)x(%d)",
                    __func__, i, entry.data.i32[i+1], entry.data.i32[i+2]);
              }
            }
          }
        }
        image_param.width = 1920;
        image_param.height = 1080;
        image_param.image_format = ImageFormat::kNV12;
        break;
      case '3':
        snapshot_choice_ = SnapshotType::kRawRdi;
        if (meta.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
          entry = meta.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES);
          if (entry.count < 2) {
            printf("ANDROID_SCALER_AVAILABLE_RAW_SIZES count is wrong\n");
            flag_exit = true;
            break;
          }
          for (uint32_t i = 0; i < entry.count; i += 2) {
            w = entry.data.i32[i + 0];
            h = entry.data.i32[i + 1];
            TEST_INFO("%s: (%d) Supported RAW RDI W(%d):H(%d)",
                __func__, i, w, h);
          }
        } else {
          printf("ANDROID_SCALER_AVAILABLE_RAW_SIZES not found\n");
          flag_exit = true;
          break;
        }
        image_param.width = w; // 5344
        image_param.height = h; // 4016
        image_param.image_format = ImageFormat::kBayerRDI10BIT;
        break;
      case '4':
        snapshot_choice_ = SnapshotType::kJpegBurst;
        if (session_enabled_ == false) {
          ret = AddPreviewTrack();
          if (ret != 0) {
            TEST_ERROR("%s:Error in AddPreview Track", __func__);
            return ret;
          }
        }
        // Check available raw YUV resolutions.
        if (meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
          entry = meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
          for (uint32_t i = 0; i < entry.count; i += 4) {
            if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == entry.data.i32[i]) {
              if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT
                  == entry.data.i32[i + 3]) {
                TEST_INFO("%s:(%d) Supported Raw YUV:(%d)x(%d)",
                    __func__, i, entry.data.i32[i+1], entry.data.i32[i+2]);
              }
            }
          }
        }
        image_param.width = 1920;
        image_param.height = 1080;
        image_param.image_format = ImageFormat::kJPEG;
        num_images_ = 30;
        break;
      default:
        printf("Wrong value entered(%c)\n", input);
        flag_exit = true;
    }
    if (flag_exit != true) {
      ImageCaptureCb cb = [&] (uint32_t camera_id_, uint32_t image_count,
          BufferDescriptor buffer, MetaData meta_data)
      { SnapshotCb(camera_id_, image_count, buffer, meta_data);};
      assert(ret == NO_ERROR);
      uint8_t awb_mode = ANDROID_CONTROL_AWB_MODE_AUTO;
      ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
      assert(ret == NO_ERROR);
      if (!sessions_.size()) {
        /* we have only capture stream which will by default disable WB and lead
         to broken picture */
        uint8_t intent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
        ret = meta.update(ANDROID_CONTROL_CAPTURE_INTENT, &intent, 1);
        assert(ret == NO_ERROR);
      }
      std::vector < CameraMetadata > meta_array;
      for (uint32_t i = 0; i < num_images_; i++) {
        meta_array.push_back(meta);
      }

      int32_t repeat = num_images_;
      do {
        {
          std::lock_guard<std::mutex> lock(error_lock_);
          camera_error_ = false;
        }
        ret = recorder_.CaptureImage(camera_id_, image_param, num_images_,
                                     meta_array, cb);
        if (ret != 0) {
          ALOGE("%s CaptureImage Failed!!", __func__);
        }
        std::unique_lock<std::mutex> lock(snapshot_wait_lock_);
        burst_snapshot_count_ = num_images_;
        std::chrono::milliseconds wait_time(get_snapshot_cb_wait_time() * 1000);

        if (snapshot_wait_signal_.WaitFor(lock, wait_time) != 0) {
             TEST_ERROR("%s Capture Image Timed out", __func__);
        }
        {
          std::lock_guard<std::mutex> lock(error_lock_);
          if (!camera_error_) {
            TEST_ERROR("%s Capture Image Done", __func__);
            break;
          }
        }
      } while(repeat-- > 0);
    }
  } while ((input != '0'));

  RemovePreviewTrack();
  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::StartMultiCameraMode() {

  TEST_INFO("%s: Enter", __func__);

  CameraStartParam camera_params{};
  camera_params.zsl_mode            = false;
  camera_params.zsl_queue_depth     = 10;
  camera_params.zsl_width           = 3840;
  camera_params.zsl_height          = 2160;
  camera_params.frame_rate          = 30;
  camera_params.flags               = 0x0;

  camera_id_ = 1;

  auto ret = recorder_.StartCamera(camera_id_, camera_params);
  if(ret != 0) {
      ALOGE("%s StartCamera Failed!!", __func__);
  }

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id, raw_width = 0, raw_height = 0;

  CameraMetadata meta;
  camera_metadata_entry_t entry;
  recorder_.GetDefaultCaptureParam(camera_id_, meta);

  if (meta.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
    entry = meta.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES);
    for (uint32_t i = 0 ; i < entry.count; i += 2) {
      raw_width = entry.data.i32[i+0];
      raw_height = entry.data.i32[i+1];
      break;
    }
  }
  assert(raw_width != 0 && raw_height != 0);

  ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> session0_tracks;

  TestTrack *rdi_track = new TestTrack(this);
  TrackInfo info{};
  info.width      = raw_width;
  info.height     = raw_height;
  info.track_id   = 1;
  info.track_type = TrackType::kVideoRDI;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.low_power_mode = false;

  ret = rdi_track->SetUp(info);
  assert(ret == 0);
  session0_tracks.push_back(rdi_track);

  sessions_.insert(std::make_pair(session_id, session0_tracks));

  rdi_track->Prepare();
  auto result = recorder_.StartSession(session_id);
  assert(result == NO_ERROR);

  sleep(1);
  camera_id_ = 0;

  ret = recorder_.StartCamera(camera_id_, camera_params);
  if(ret != 0) {
      ALOGE("%s StartCamera Failed!!", __func__);
  }

  ret = recorder_.GetDefaultCaptureParam(camera_id_, static_info_);
  if (NO_ERROR != ret) {
    ALOGE("%s Unable to query default capture parameters!\n",
          __func__);
  } else {
    InitSupportedNRModes();
    InitSupportedVHDRModes();
    InitSupportedIRModes();
  }
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> session2_tracks;

  TestTrack *yuv_1080p_track = new TestTrack(this);
  info = {};
  info.width      = 3840;
  info.height     = 2160;
  info.track_id   = 2;
  info.track_type = TrackType::kVideoAVC;
  info.session_id = session_id;
  info.camera_id = camera_id_;

  ret = yuv_1080p_track->SetUp(info);
  assert(ret == 0);
  session2_tracks.push_back(yuv_1080p_track);

  sessions_.insert(std::make_pair(session_id, session2_tracks));

  yuv_1080p_track->Prepare();
  result = recorder_.StartSession(session_id);
  assert(result == NO_ERROR);
  session_enabled_ = true;

  sleep(1);
  camera_id_ = 2;

  ret = recorder_.StartCamera(camera_id_, camera_params);
  if(ret != 0) {
      ALOGE("%s StartCamera Failed!!", __func__);
  }

  ret = recorder_.GetDefaultCaptureParam(camera_id_, static_info_);
  if (NO_ERROR != ret) {
    ALOGE("%s Unable to query default capture parameters!\n",
          __func__);
  }

  ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> session3_tracks;

  TestTrack *yuv_stereo_track = new TestTrack(this);
  info = {};
  info.width      = 1280;
  info.height     = 480;
  info.track_id   = 3;
  info.track_type = TrackType::kVideoYUV;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.low_power_mode = true;

  ret = yuv_stereo_track->SetUp(info);
  assert(ret == 0);
  session3_tracks.push_back(yuv_stereo_track);

  sessions_.insert(std::make_pair(session_id, session3_tracks));

  yuv_stereo_track->Prepare();
  result = recorder_.StartSession(session_id);
  assert(result == NO_ERROR);

  camera_id_ = 0;
  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::StopMultiCameraMode() {
  TEST_INFO("%s: Enter", __func__);

  for (session_iter_ it = sessions_.begin(); it != sessions_.end(); ++it) {

    uint32_t session_id = it->first;
    auto result = recorder_.StopSession(session_id, true /*flush buffers*/);
    assert(result == NO_ERROR);

    for (auto track : it->second) {
      track->CleanUp();
    }
  }
  session_enabled_ = false;

  camera_id_ = 1;

  auto ret = recorder_.StopCamera(camera_id_);
  if(ret != 0) {
    ALOGE("%s StopCamera 1 Failed!!", __func__);
  }

  camera_id_ = 2;

  ret = recorder_.StopCamera(camera_id_);
  if(ret != 0) {
    ALOGE("%s StopCamera 2 Failed!!", __func__);
  }

  camera_id_ = 0;

  ret = recorder_.StopCamera(camera_id_);
  if(ret != 0) {
    ALOGE("%s StopCamera 0 Failed!!", __func__);
  }

  static_info_.clear();
  TEST_INFO("%s: Exit", __func__);
  return 0;

}

// This session has two YUV video tracks 4K and 1080p.
status_t RecorderTest::Session4KAnd1080pYUVTracks() {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *yuv_4k_track = new TestTrack(this);
  TrackInfo info{};
  info.width      = 3840;
  info.height     = 2160;
  info.track_id   = 1;
  info.track_type = TrackType::kVideoYUV;
  info.session_id = session_id;
  info.ltr_count  = ltr_count_;
  info.camera_id = camera_id_;

  ret = yuv_4k_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(yuv_4k_track);

  TestTrack *yuv_1080p_track = new TestTrack(this);
  info = {};
  info.width      = 1920;
  info.height     = 1080;
  info.track_id   = 2;
  info.track_type = TrackType::kVideoYUV;
  info.session_id = session_id;
  info.camera_id = camera_id_;

  ret = yuv_1080p_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(yuv_1080p_track);

  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

// This session has one 4K video encode track
status_t RecorderTest::Session4KEncTrack(const TrackType& track_type) {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  TestTrack *video_track = new TestTrack(this);
  TrackInfo info{};
  info.width      = 3840;
  info.height     = 2160;
  info.track_id   = 1;
  info.track_type = track_type;
  info.ltr_count  = ltr_count_;
  info.session_id = session_id;
  info.camera_id = camera_id_;

  ret = video_track->SetUp(info);
  assert(ret == 0);

  std::vector<TestTrack*> tracks;
  tracks.push_back(video_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

// This session has one 1080p video encode and one AAC Audio track.
status_t RecorderTest::Session1080pEncAndAudioAACTrack(const TrackType& vid_track_type) {

  TEST_INFO("%s: Enter", __func__);
  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *enc_1080p_track = new TestTrack(this);
  TrackInfo info{};
  info.width      = 1920;
  info.height     = 1080;
  info.track_id   = 1;
  info.track_type = vid_track_type;
  info.ltr_count  = ltr_count_;
  info.session_id = session_id;
  info.camera_id = camera_id_;

  ret = enc_1080p_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(enc_1080p_track);

  TestTrack *audio_aac_track = new TestTrack(this);
  info = {};
  info.track_id   = 101;
  info.track_type = TrackType::kAudioAAC;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBuiltIn);

  ret = audio_aac_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_aac_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

// In this test case session has one 1080p video encode and one 1080p YUV track.
status_t RecorderTest::Session1080pEnc1080YUV(const TrackType& track_type) {

  TEST_INFO("%s: Enter", __func__);
  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *enc_1080p_track = new TestTrack(this);
  TrackInfo info{};
  info.width      = 1920;
  info.height     = 1080;
  info.track_id   = 1;
  info.track_type = track_type;
  info.ltr_count  = ltr_count_;
  info.session_id = session_id;
  info.camera_id = camera_id_;

  ret = enc_1080p_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(enc_1080p_track);

  TestTrack *yuv_1080p_track = new TestTrack(this);
  info = {};
  info.width      = 1920;
  info.height     = 1080;
  info.track_id   = 2;
  info.track_type = TrackType::kVideoYUV;
  info.session_id = session_id;
  info.camera_id = camera_id_;

  ret = yuv_1080p_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(yuv_1080p_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

// In this test case session has one 4K video HEVC and one 1080p YUV track.
status_t RecorderTest::Session4KEncAnd1080pYUVTracks(const TrackType&
                                                      track_type) {

  TEST_INFO("%s: Enter", __func__);
  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *enc_1080p_track = new TestTrack(this);
  TrackInfo info{};
  info.width      = 3840;
  info.height     = 2160;
  info.track_id   = 1;
  info.track_type = track_type;
  info.ltr_count  = ltr_count_;
  info.session_id = session_id;
  info.camera_id = camera_id_;

  ret = enc_1080p_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(enc_1080p_track);

  TestTrack *yuv_1080p_track = new TestTrack(this);
  info = {};
  info.width      = 1920;
  info.height     = 1080;
  info.track_id   = 2;
  info.track_type = TrackType::kVideoYUV;
  info.session_id = session_id;
  info.camera_id = camera_id_;

  ret = yuv_1080p_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(yuv_1080p_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

// This session has one 4K YUV and one 1080p video encode track
status_t RecorderTest::Session4KYUVAnd1080pEncTracks(const TrackType&
                                                        track_type) {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *yuv_4k_track = new TestTrack(this);
  TrackInfo info{};
  info.width      = 3840;
  info.height     = 2160;
  info.track_id   = 1;
  info.track_type = TrackType::kVideoYUV;
  info.session_id = session_id;
  info.camera_id = camera_id_;

  ret = yuv_4k_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(yuv_4k_track);

  TestTrack *enc_1080p_track = new TestTrack(this);
  info = {};
  info.width      = 1920;
  info.height     = 1080;
  info.track_id   = 2;
  info.track_type = track_type;
  info.ltr_count  = ltr_count_;
  info.session_id = session_id;
  info.camera_id = camera_id_;

  ret = enc_1080p_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(enc_1080p_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

// This session has two 1080p video encode tracks.
status_t RecorderTest::SessionTwo1080pEncTracks(const TrackType& track_type) {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *enc_1080p_track1 = new TestTrack(this);
  TrackInfo info{};
  info.width      = 1920;
  info.height     = 1080;
  info.track_id   = 1;
  info.track_type = track_type;
  info.ltr_count  = ltr_count_;
  info.session_id = session_id;
  info.camera_id = camera_id_;

  ret = enc_1080p_track1->SetUp(info);
  assert(ret == 0);
  tracks.push_back(enc_1080p_track1);

  TestTrack *enc_1080p_track2 = new TestTrack(this);
  info = {};
  info.width      = 1920;
  info.height     = 1080;
  info.track_id   = 2;
  info.track_type = track_type;
  info.ltr_count  = ltr_count_;
  info.session_id = session_id;
  info.camera_id = camera_id_;

  ret = enc_1080p_track2->SetUp(info);
  assert(ret == 0);
  tracks.push_back(enc_1080p_track2);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

// This session has one 720P LPM track
status_t RecorderTest::Session720pLPMTrack(const TrackType& track_type) {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *yuv_720p_track = new TestTrack(this);
  TrackInfo info{};
  info.width      = 1280;
  info.height     = 720;
  info.track_id   = 1;
  info.track_type = track_type;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.low_power_mode = true;

  ret = yuv_720p_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(yuv_720p_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

// This session has one 1080p Encode and one 1080p LPM tracks
status_t RecorderTest::Session1080pEnc1080pLPMTracks(const TrackType& track_type) {

  TEST_INFO("%s: Enter", __func__);
  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *enc_1080p_track = new TestTrack(this);
  TrackInfo info{};
  info.width      = 1920;
  info.height     = 1080;
  info.track_id   = 1;
  info.track_type = track_type;
  info.ltr_count  = ltr_count_;
  info.session_id = session_id;
  info.low_power_mode = false;

  ret = enc_1080p_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(enc_1080p_track);

  TestTrack *yuv_1080p_track = new TestTrack(this);
  info = {};
  info.width          = 1920;
  info.height         = 1080;
  info.track_id       = 2;
  info.track_type     = TrackType::kVideoYUV;
  info.session_id     = session_id;
  info.low_power_mode = true;
  info.camera_id = camera_id_;

  ret = yuv_1080p_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(yuv_1080p_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CreateAudioPCMTrack() {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *audio_pcm_track = new TestTrack(this);
  TrackInfo info{};
  info.track_id   = 101;
  info.track_type = TrackType::kAudioPCM;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBuiltIn);

  ret = audio_pcm_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_pcm_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CreateAudio2PCMTrack() {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *audio_pcm_track1 = new TestTrack(this);
  TrackInfo info{};
  info.track_id   = 101;
  info.track_type = TrackType::kAudioPCM;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBuiltIn);

  ret = audio_pcm_track1->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_pcm_track1);

  TestTrack *audio_pcm_track2 = new TestTrack(this);
  info.track_id   = 102;

  ret = audio_pcm_track2->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_pcm_track2);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CreateAudioSCOTrack() {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *audio_sco_track = new TestTrack(this);
  TrackInfo info{};
  info.track_id   = 101;
  info.track_type = TrackType::kAudioPCM;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBlueToothSCO);

  ret = audio_sco_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_sco_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CreateAudioPCMSCOTrack() {
  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *audio_pcm_track = new TestTrack(this);
  TrackInfo info{};
  info.track_id   = 101;
  info.track_type = TrackType::kAudioPCM;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBuiltIn);

  ret = audio_pcm_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_pcm_track);

  TestTrack *audio_sco_track = new TestTrack(this);
  info.track_id   = 102;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBlueToothSCO);

  ret = audio_sco_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_sco_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CreateAudioA2DPTrack() {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *audio_a2dp_track = new TestTrack(this);
  TrackInfo info{};
  info.track_id   = 101;
  info.track_type = TrackType::kAudioPCM;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBlueToothA2DP);

  ret = audio_a2dp_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_a2dp_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CreateAudioPCMA2DPTrack() {
  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *audio_pcm_track = new TestTrack(this);
  TrackInfo info{};
  info.track_id   = 101;
  info.track_type = TrackType::kAudioPCM;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBuiltIn);

  ret = audio_pcm_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_pcm_track);

  TestTrack *audio_a2dp_track = new TestTrack(this);
  info.track_id   = 102;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBlueToothA2DP);

  ret = audio_a2dp_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_a2dp_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CreateAudioAACTrack() {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *audio_aac_track = new TestTrack(this);
  TrackInfo info{};
  info.track_id   = 101;
  info.track_type = TrackType::kAudioAAC;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBuiltIn);

  ret = audio_aac_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_aac_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CreateAudio2AACTrack() {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *audio_aac_track1 = new TestTrack(this);
  TrackInfo info{};
  info.track_id   = 101;
  info.track_type = TrackType::kAudioAAC;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBuiltIn);

  ret = audio_aac_track1->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_aac_track1);

  TestTrack *audio_aac_track2 = new TestTrack(this);
  info.track_id   = 102;

  ret = audio_aac_track2->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_aac_track2);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CreateAudioPCMAACTrack() {
  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *audio_pcm_track = new TestTrack(this);
  TrackInfo info{};
  info.track_id   = 101;
  info.track_type = TrackType::kAudioPCM;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBuiltIn);

  ret = audio_pcm_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_pcm_track);

  TestTrack *audio_aac_track = new TestTrack(this);
  info.track_id   = 102;
  info.track_type = TrackType::kAudioAAC;

  ret = audio_aac_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_aac_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CreateAudioAMRTrack() {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *audio_amr_track = new TestTrack(this);
  TrackInfo info{};
  info.track_id   = 101;
  info.track_type = TrackType::kAudioAMR;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBuiltIn);

  ret = audio_amr_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_amr_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CreateAudio2AMRTrack() {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *audio_amr_track1 = new TestTrack(this);
  TrackInfo info{};
  info.track_id   = 101;
  info.track_type = TrackType::kAudioAMR;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBuiltIn);

  ret = audio_amr_track1->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_amr_track1);

  TestTrack *audio_amr_track2 = new TestTrack(this);
  info.track_id   = 102;

  ret = audio_amr_track2->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_amr_track2);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CreateAudioPCMAMRTrack() {
  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *audio_pcm_track = new TestTrack(this);
  TrackInfo info{};
  info.track_id   = 101;
  info.track_type = TrackType::kAudioPCM;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBuiltIn);

  ret = audio_pcm_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_pcm_track);

  TestTrack *audio_amr_track = new TestTrack(this);
  info.track_id   = 102;
  info.track_type = TrackType::kAudioAMR;

  ret = audio_amr_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_amr_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CreateAudioG711Track() {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *audio_g711_track = new TestTrack(this);
  TrackInfo info{};
  info.track_id   = 101;
  info.track_type = TrackType::kAudioG711;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBuiltIn);

  ret = audio_g711_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_g711_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CreateAudio2G711Track() {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *audio_g711_track1 = new TestTrack(this);
  TrackInfo info{};
  info.track_id   = 101;
  info.track_type = TrackType::kAudioG711;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBuiltIn);

  ret = audio_g711_track1->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_g711_track1);

  TestTrack *audio_g711_track2 = new TestTrack(this);
  info.track_id   = 102;

  ret = audio_g711_track2->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_g711_track2);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CreateAudioPCMG711Track() {
  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *audio_pcm_track = new TestTrack(this);
  TrackInfo info{};
  info.track_id   = 101;
  info.track_type = TrackType::kAudioPCM;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBuiltIn);

  ret = audio_pcm_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_pcm_track);

  TestTrack *audio_g711_track = new TestTrack(this);
  info.track_id   = 102;
  info.track_type = TrackType::kAudioG711;

  ret = audio_g711_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_g711_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::CreateAudioPCMFluenceTrack() {
  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *audio_pcm_track = new TestTrack(this);
  TrackInfo info;
  memset(&info, 0x0, sizeof info);
  info.track_id   = 101;
  info.track_type = TrackType::kAudioPCMFP;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.device_id = static_cast<DeviceId>(AudioDeviceId::kBuiltIn);

  ret = audio_pcm_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(audio_pcm_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

// This session has one RDI track with sensor resolution.
status_t RecorderTest::SessionRDITrack() {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id, raw_width = 0, raw_height = 0;

  CameraMetadata meta;
  camera_metadata_entry_t entry;
  recorder_.GetDefaultCaptureParam(camera_id_, meta);

  if (meta.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
    entry = meta.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES);
    for (uint32_t i = 0 ; i < entry.count; i += 2) {
      raw_width = entry.data.i32[i+0];
      raw_height = entry.data.i32[i+1];
      break;
    }
  }
  assert(raw_width != 0 && raw_height != 0);

  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;

  TestTrack *rdi_track = new TestTrack(this);
  TrackInfo info{};
  info.width      = raw_width;
  info.height     = raw_height;
  info.track_id   = 1;
  info.track_type = TrackType::kVideoRDI;
  info.session_id = session_id;
  info.camera_id = camera_id_;
  info.low_power_mode = false;

  ret = rdi_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(rdi_track);

  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

// This session has 1080P YUV video track with Display Enabled.
status_t RecorderTest::Session1080pYUVTrackWithDisplay() {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;
  TrackInfo info{};

  TestTrack *yuv_1080p_track = new TestTrack(this);
  info.width      = 1920;
  info.height     = 1080;
  info.track_id   = 1;
  info.track_type = TrackType::kVideoYUV;
  info.session_id = session_id;
  info.fps = 30;

  ret = yuv_1080p_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(yuv_1080p_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  use_display = 1;
  TEST_INFO("%s: Exit", __func__);
  return ret;
}

// 1080P YUV video track with Display Enabled in recorder service.
status_t RecorderTest::Session1080pYUVTrackWithPreview() {

  TEST_INFO("%s: Enter", __func__);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  auto ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);

  std::vector<TestTrack*> tracks;
  TrackInfo info{};

  TestTrack *yuv_1080p_track = new TestTrack(this);
  info.width      = 1920;
  info.height     = 1080;
  info.track_id   = 1;
  info.track_type = TrackType::kVideoPreview;
  info.session_id = session_id;

  ret = yuv_1080p_track->SetUp(info);
  assert(ret == 0);
  tracks.push_back(yuv_1080p_track);
  sessions_.insert(std::make_pair(session_id, tracks));

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::StartSession() {

  TEST_INFO("%s: Enter", __func__);
  session_iter_ it = sessions_.begin();

  // Prepare tracks: setup files to dump track data, event etc.
  for (auto track : it->second) {
    track->Prepare();
    TrackType type = track->GetTrackType();
    if ( (type == TrackType::kVideoYUV)
        || (type == TrackType::kVideoRDI)
        || (type == TrackType::kVideoAVC)
        || (type == TrackType::kVideoHEVC)
        || (type == TrackType::kVideoPreview) ) {
      session_enabled_ = true;
#ifndef DISABLE_DISPLAY
      if (use_display == 1) {
        auto ret = track->StartDisplay(DisplayType::kPrimary);
        if(ret != 0) {
          ALOGE("%s StartDisplay Failed!!", __func__);
        }
      }
#endif
    }
  }
  uint32_t session_id = it->first;
  auto result = recorder_.StartSession(session_id);
  assert(result == NO_ERROR);

  TEST_INFO("%s: Enter", __func__);
  return NO_ERROR;
}

status_t RecorderTest::StopSession() {

  TEST_INFO("%s: Enter", __func__);
  session_iter_ it = sessions_.begin();

  uint32_t session_id = it->first;
  auto result = recorder_.StopSession(session_id, true /*flush buffers*/);
  assert(result == NO_ERROR);

  for (auto track : it->second) {
#ifndef DISABLE_DISPLAY
    if (use_display == 1) {
      auto ret = track->StopDisplay(DisplayType::kPrimary);
      if(ret != 0) {
        ALOGE("%s StopDisplay Failed!!", __func__);
      }
    }
#endif
    track->CleanUp();
    TrackType type = track->GetTrackType();
    if ( (type == TrackType::kVideoYUV)
        || (type == TrackType::kVideoRDI)
        || (type == TrackType::kVideoAVC)
        || (type == TrackType::kVideoHEVC)
        || (type == TrackType::kVideoPreview) ) {
      session_enabled_ = false;
    }
  }
  TEST_INFO("%s: Exit", __func__);
  return NO_ERROR;
}

status_t RecorderTest::PauseSession() {

  TEST_INFO("%s: Enter", __func__);
  session_iter_ it = sessions_.begin();
  uint32_t session_id = it->first;
  auto ret = recorder_.PauseSession(session_id);
  assert(ret == 0);
  TEST_INFO("%s: Exit", __func__);

  return NO_ERROR;
}

status_t RecorderTest::ResumeSession() {

  TEST_INFO("%s: Enter", __func__);
  session_iter_ it = sessions_.begin();
  uint32_t session_id = it->first;
  auto ret = recorder_.ResumeSession(session_id);
  assert(ret == 0);
  TEST_INFO("%s: Exit", __func__);

  return NO_ERROR;
}

status_t RecorderTest::SetParams() {

  TEST_INFO("%s: Enter", __func__);
  int32_t ret = 0;
  session_iter_ it = sessions_.begin();
  uint32_t session_id = it->first;
  CodecParamType param_type;

  uint32_t input;
  uint32_t value;
  float fps;

  do {
    printf("\n");
    printf("****** Set Dynamic Codec Param *******\n" );
    printf("  1. bitrate \n" );
    printf("  2. framerate \n" );
    printf("  3. insert-idr \n" );
    printf("  4. idr interval \n" );
    printf("  5. ltr count (This has to be set before creating session)\n");
    if (ltr_count_ > 0) {
      printf("  6. ltr mark \n" );
      printf("  7. ltr use \n" );
    }
    printf("  0. exit \n");
    printf("\n");
    printf("Enter set param option\n");
    scanf("%d", &input);

    switch (input) {
      case 0:
        break;
      case 1:
        printf("Enter bitrate value\n");
        scanf("%d", &value);
        param_type = CodecParamType::kBitRateType;
        ret = recorder_.SetVideoTrackParam(session_id, 1, param_type, &value,
                                              sizeof(value));
        break;
      case 2:
        printf("Enter fps value\n");
        scanf("%f", &fps);
        param_type = CodecParamType::kFrameRateType;
        ret = recorder_.SetVideoTrackParam(session_id, 1, param_type, &fps,
                                              sizeof(fps));
        break;
      case 3:
        printf("Insert idr frame\n");
        param_type = CodecParamType::kInsertIDRType;
        ret = recorder_.SetVideoTrackParam(session_id, 1, param_type, &value,
                                              sizeof(value));
        break;
      case 4:
        printf("Enter number of P frame value\n");
        scanf("%d", &value);
        param_type = CodecParamType::kIDRIntervalType;
        VideoEncIdrInterval idr_interval;
        idr_interval.num_pframes = value;
        idr_interval.num_bframes = 0;
        idr_interval.idr_period = 0;
        ret = recorder_.SetVideoTrackParam(session_id, 1, param_type,
                                           &idr_interval, sizeof(idr_interval));
        break;
      case 5:
        printf("Enter ltr count value [1-4]\n");
        scanf("%d", &ltr_count_);
        ltr_count_= CLIP(ltr_count_, kMinLTRCount, kMaxLTRCount);
        break;
      case 6:
        printf("Enter ltr mark id value\n");
        scanf("%d", &value);
        param_type = CodecParamType::kMarkLtrType;
        ret = recorder_.SetVideoTrackParam(session_id, 1, param_type, &value,
                                              sizeof(value));
        break;
      case 7:
        printf("Enter ltr use id value\n");
        scanf("%d", &value);
        param_type = CodecParamType::kUseLtrType;
        VideoEncLtrUse ltr_use;
        ltr_use.id = value;
        ltr_use.frame = 5;
        ret = recorder_.SetVideoTrackParam(session_id, 1, param_type, &ltr_use,
                                              sizeof(ltr_use));
        break;
      default:
         printf("Wrong value entered(%d)\n", input);
         input = 0;
    }
    if(input) {
      assert(ret == 0);
    }
  } while(input);

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::SetDynamicCameraParam() {
  TEST_INFO("%s: Enter", __func__);

  status_t ret = 0;
  char input;
  CameraMetadata meta;
  camera_metadata_entry_t entry;
  int64_t range_min, range_max;

  do {
    std::cout << std::endl;
    std::cout << "****** Set Dynamic Camera Param *******" << std::endl;
    std::cout << "  1. Sharpness" << std::endl;
    std::cout << "  2. AE Gain" << std::endl;
    std::cout << "  3. Manual Exposure Time" << std::endl;
    std::cout << "  4. WNR Strength" << std::endl;
    std::cout << "  5. TNR Tuning" << std::endl;
    std::cout << "  6. Dump raw histogram stats" << std::endl;
    std::cout << "  7. Dump raw AEC AWB stats" << std::endl;
    std::cout << "  X. exit" << std::endl;
    std::cout << std::endl;
    std::cout << "Enter set camera param option" << std::endl;
    std::cin >> input;

    ret = recorder_.GetCameraParam(camera_id_, meta);
    if (ret != 0) {
      TEST_ERROR("%s: Failed to get camera params", __func__);
      TEST_INFO("%s: Exit", __func__);
      return ret;
    }

    switch (static_cast<DynamicCameraParamsCmd>(input)) {
      case DynamicCameraParamsCmd::kSharpness: {
        uint32_t sharpness_range_vtag;
#ifdef ANDROID_O_OR_ABOVE
        if (!VendorTagSupported(String8("range"),
            String8("org.codeaurora.qcamera3.sharpness"),
            &sharpness_range_vtag)) {
          TEST_WARN("%s: sharpness range is not supported", __func__);
          break;
        }
#else
        sharpness_range_vtag = QCAMERA3_SHARPNESS_RANGE;
#endif
        if (static_info_.exists(sharpness_range_vtag)) {
          entry = static_info_.find(sharpness_range_vtag);
          range_min = entry.data.i32[0];
          range_max = entry.data.i32[1];
          int32_t sharpness_strength;
          ret = GetSharpnessStrength(sharpness_strength);
          if (ret != 0) {
            TEST_ERROR("%s: failed to get sharpness strength", __func__);
            break;
          }
          std::cout << "Enter Sharpness Value (Current:" << sharpness_strength;
          std::cout << ")[" << range_min << '-' << range_max <<"]: ";
          std::cin >> sharpness_strength;
          ret = SetSharpnessStrength(sharpness_strength);
          if (ret != 0) {
            TEST_ERROR("%s: failed to set sharpness strength", __func__);
            break;
          }
        }
        break;
      }
      case DynamicCameraParamsCmd::kAEGain: {
        if (static_info_.exists(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE)) {
          entry = static_info_.find(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE);
          range_min = entry.data.i32[0];
          range_max = entry.data.i32[1];
          int32_t sensitivity;
          ret = GetSensorSensitivity(&sensitivity);
          if (ret != 0) {
            TEST_ERROR("%s: failed to get sensor sensitivity",
                       __func__);
            break;
          }
          std::cout << "Enter Sensitivity Value (Current:" << sensitivity;
          std::cout << ")[" << range_min << '-' << range_max <<"]: ";
          // AE algorithm will calculate gain based on this value
          std::cin >> sensitivity;
          ret = SetSensorSensitivity(sensitivity);
          if (ret != 0) {
            TEST_ERROR("%s: failed to set sensor sensitivity",
                       __func__);
            break;
          }
        }
        break;
      }
      case DynamicCameraParamsCmd::kExposureTime: {
        if (static_info_.exists(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE)) {
          entry = static_info_.find(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE);
          range_min = entry.data.i64[0];
          range_max = entry.data.i64[1];
          int64_t exposure_time_ns;
          ret = GetExposureTime(&exposure_time_ns);
          if (ret != 0) {
            TEST_ERROR("%s: failed to get exposure time",
                       __func__);
            break;
          }
          std::cout << "Enter Exposure Time Value in ns (Current:";
          std::cout << exposure_time_ns << ")[" << range_min << '-';
          std::cout << range_max <<"]: ";
          std::cin >> exposure_time_ns;
          ret = SetExposureTime(exposure_time_ns);
          if (ret != 0) {
            TEST_ERROR("%s: failed to set exposure time",
                       __func__);
            break;
          }
        }
        break;
      }
      case DynamicCameraParamsCmd::kWNRStrength: {
        if (static_info_.exists(QCAMERA3_WNR_RANGE)) {
          entry = static_info_.find(QCAMERA3_WNR_RANGE);
          range_min = static_cast<int32_t>(entry.data.u8[0]);
          range_max = static_cast<int32_t>(entry.data.u8[1]);
          int32_t wnr_strength;
          ret = GetWNRStrength(&wnr_strength);
          if (ret != 0) {
            TEST_ERROR("%s: failed to get WNR strength",
                       __func__);
            break;
          }
          std::cout << "Enter WNR strength value (Current:" << wnr_strength;
          std::cout << ")[" << range_min << '-';
          std::cout << range_max <<"]: ";
          std::cin >> wnr_strength;
          ret = SetWNRStrength(wnr_strength);
          if (ret != 0) {
            TEST_ERROR("%s: failed to set WNR strength",
                       __func__);
            break;
          }
        }
        break;
      }
      case DynamicCameraParamsCmd::kTNRTuning: {
        // Check if TNR is enabled.
        // If not,NR mode to HIGH_QAULITY to enable TNR.
        entry = meta.find(ANDROID_NOISE_REDUCTION_MODE);
        if (entry.data.u8[0] != ANDROID_NOISE_REDUCTION_MODE_HIGH_QUALITY) {
          const uint8_t nr_mode = ANDROID_NOISE_REDUCTION_MODE_HIGH_QUALITY;
          meta.update(ANDROID_NOISE_REDUCTION_MODE, &nr_mode, 1);
          ret = recorder_.SetCameraParam(camera_id_, meta);
          if (ret != 0) {
            TEST_ERROR("%s Failed to enable TNR\n",
                       __func__);
            break;
          }
        }
        ret = SetTNRLevel();
        if (ret != 0) {
          TEST_ERROR("%s Failed to set TNR level\n",
                     __func__);
          break;
        }
        break;
      }
      case DynamicCameraParamsCmd::kDumpHistogramStats: {
        // Enabling Histogram stats in Metadata
        uint32_t histogram_mode_vtag;
#ifdef ANDROID_O_OR_ABOVE
        if (VendorTagExistsInMeta(meta, String8("enable"),
            String8("org.codeaurora.qcamera3.histogram"),
            &histogram_mode_vtag)) {
          const uint8_t hist_mode = StatisticsHistogramModeValues
              ::kStatisticsHistogramModeOn;
#else
        histogram_mode_vtag = QCAMERA3_HISTOGRAM_MODE;
        if (meta.exists(histogram_mode_vtag)) {
          const uint8_t hist_mode = QCAMERA3_HISTOGRAM_MODE_ON;
#endif
          meta.update(histogram_mode_vtag, &hist_mode, 1);
          ret = recorder_.SetCameraParam(camera_id_, meta);
          if (ret != 0) {
            TEST_ERROR("%s Failed to set Histogram mode to ON\n", __func__);
            break;
          }
          dump_histogram_stats_ = true;
        }
        break;
      }
      case DynamicCameraParamsCmd::kDumpAECAWBStats: {
        // Enabling AEC AWC stats in Metadata
        const int32_t aec_awc_mode = QCAMERA3_EXPOSURE_DATA_ON;
        meta.update(QCAMERA3_EXPOSURE_DATA_ENABLE, &aec_awc_mode, 1);
        ret = recorder_.SetCameraParam(camera_id_, meta);
        if (ret != 0) {
          TEST_ERROR("%s Failed to set edge to OFF\n",
                     __func__);
          break;
        }
        dump_aec_awb_stats_ = true;
        break;
      }
      case DynamicCameraParamsCmd::kExit:
        break;
      default:
         std::cout << "Wrong value entered:" << input << std::endl;
    }
    if (input == static_cast<char>(DynamicCameraParamsCmd::kExit)
        || ret != 0) {
      break;
    }
  } while (input);

  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::EnableOverlay() {

  TEST_INFO("%s: Enter", __func__);
  int32_t ret = 0;
  // Enable overlay on all existing video tracks.
  session_iter_ it = sessions_.begin();
  for (auto track : it->second) {
    TrackType type = track->GetTrackType();
    if ( (type == TrackType::kVideoYUV)
        || (type == TrackType::kVideoAVC)
        || (type == TrackType::kVideoHEVC)
        || (type == TrackType::kVideoPreview) ) {
      track->EnableOverlay();
      assert(ret == 0);
    }
  }
  TEST_INFO("%s: Exit", __func__);
  return ret;
}

status_t RecorderTest::DisableOverlay() {

  TEST_INFO("%s: Enter", __func__);
  session_iter_ it = sessions_.begin();
  for (auto track : it->second) {
    TrackType type = track->GetTrackType();
    if ( (type == TrackType::kVideoYUV)
        || (type == TrackType::kVideoAVC)
        || (type == TrackType::kVideoHEVC)
        || (type == TrackType::kVideoPreview) ) {
      track->DisableOverlay();
    }
  }
  TEST_INFO("%s: Exit", __func__);
  return NO_ERROR;
}

status_t RecorderTest::DeleteSession() {

  TEST_INFO("%s: Enter", __func__);
  session_iter_ it = sessions_.begin();
  uint32_t session_id = it->first;
  // Delete all the tracks associated to session.
  status_t ret;
  for (auto track : it->second) {
      assert(track != nullptr);
      if (track->GetTrackType() == TrackType::kAudioPCM ||
          track->GetTrackType() == TrackType::kAudioPCMFP ||
          track->GetTrackType() == TrackType::kAudioAAC ||
          track->GetTrackType() == TrackType::kAudioAMR ||
          track->GetTrackType() == TrackType::kAudioG711) {
        ret = recorder_.DeleteAudioTrack(session_id, track->GetTrackId());
      } else {
        ret = recorder_.DeleteVideoTrack(session_id, track->GetTrackId());
      }
      assert(ret == 0);
      delete track;
      track = nullptr;
  }
  // Once all tracks are deleted successfully delete session.
  ret = recorder_.DeleteSession(session_id);
  sessions_.erase(it);
  use_display = 0;
  ltr_count_ = 0;

  TEST_INFO("%s: Exit", __func__);
  return 0;
}

void RecorderTest::GetMaxResolutionTrack(TrackInfo &result) {
  if (sessions_.size() <= 0) {
    TEST_ERROR("%s: Application does not have the active session ",
               __func__);
  }
  auto it_sessions = sessions_.begin();
  auto it_testtrack = it_sessions->second;
  uint32_t max_width = ((*it_testtrack.begin())->GetTrackHandle()).width;
  result = (*it_testtrack.begin())->GetTrackHandle();
  for (auto it = sessions_.begin(); it != sessions_.end(); ++it) {
    auto it_test_track = it->second;
    for (auto it_trackinfo = it_test_track.begin();
         it_trackinfo != it_test_track.end(); ++it_trackinfo) {
      TEST_DBG("%s: Width:%u Height:%u ", __func__,
                (*it_trackinfo)->GetTrackHandle().width,
                (*it_trackinfo)->GetTrackHandle().height);
      if (max_width < ((*it_trackinfo)->GetTrackHandle().width)) {
        max_width = (*it_trackinfo)->GetTrackHandle().width;
        result = (*it_trackinfo)->GetTrackHandle();
      }
    }
  }
}

void RecorderTest::PrintAWBROIHelp() {
  std::cout << " \nNote :" << std::endl;
  std::cout << " Target color is color of ROI that user expect." << std::endl;
  std::cout << " ROI should be selected such that there are no two colors in "
               "that ROI."
            << std::endl;
  std::cout << " In ROI there should be only one color, and target color "
               "specified should be the color which is expected color for user "
               "in ROI."
            << std::endl;
  std::cout << " Procedure to test :" << std::endl;
  std::cout << " - Enable the session in recorder_test" << std::endl;
  std::cout << "   e.g. 1->3->6->A" << std::endl;
  std::cout << " - Press '$' and get below option :" << std::endl;
  std::cout << "   1. Enable ROI" << std::endl;
  std::cout << "   2. Disable ROI" << std::endl;
  std::cout << "   3. Help" << std::endl;
  std::cout << "   X. Exit" << std::endl;
  std::cout << " - Press '1 to enable ROI. below is the sample text to show "
               "how to enter values :"
            << std::endl;
  std::cout << "  Enter Top Left Coordinate Values [x y] : 100 100"
            << std::endl;
  std::cout << "  Enter Bottom Right Coordinate Values [x y] : 200 200"
            << std::endl;
  std::cout << "  Enter RGB values [R G B] :255 1 1" << std::endl;
  std::cout << " - Press '2' to disable ROI." << std::endl;
  std::cout << " - Press 'X'/'x' to exit the ROI window.\n" << std::endl;
}

status_t RecorderTest::HandleAWBROIRequest() {
  char input;
  status_t status = -1;
  int32_t full_fov_width, full_fov_height;
  bool coordinate_correct = false;
  int32_t top_left[2] = {0};
  int32_t bottom_right[2] = {0};
  bool color_correct = false;
  CameraMetadata meta;
  if (session_enabled_) {
    if (static_info_.exists(ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE)) {
      auto active_array_size =
          static_info_.find(ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE);
      if (!active_array_size.count) {
        TEST_ERROR("%s: Active sensor array size is missing!",
                   __func__);
        return status;
      }
      full_fov_width = active_array_size.data.i32[2];
      full_fov_height = active_array_size.data.i32[3];
    }
    TrackInfo result{};
    GetMaxResolutionTrack(result);
    int32_t max_roi_width_ = result.width;
    int32_t max_roi_height_ = result.height;
    do {
      coordinate_correct = false;
      color_correct = false;
      std::cout << "  1. Enable ROI " << std::endl;
      std::cout << "  2. Disable ROI " << std::endl;
      std::cout << "  3. Help " << std::endl;
      std::cout << "  X. Exit" << std::endl;
      std::cin >> input;

      switch (input) {
        case '1':
          std::cout << "  Max Width and Height supported is :["
                    << max_roi_width_ << "][" << max_roi_height_ << "]"
                    << "\n";

          std::cout << "  Enter Top Left Coordinate Values [x y] : ";
          std::cin >> top_left[0];
          std::cin >> top_left[1];

          std::cout << "\n  Enter Bottom Right Coordinate Values [x y] : ";
          std::cin >> bottom_right[0];
          std::cin >> bottom_right[1];

          if ((top_left[0] >= 0 && top_left[0] <= max_roi_width_) &&
              (top_left[1] >= 0 && top_left[1] <= max_roi_height_) &&
              (bottom_right[0] > 0 && bottom_right[0] <= max_roi_width_) &&
              (bottom_right[1] > 0 && bottom_right[1] <= max_roi_height_)) {
            coordinate_correct = true;
            roi_region_.roi_coordinates[0] =
                top_left[0] * (full_fov_width / max_roi_width_);
            roi_region_.roi_coordinates[1] =
                top_left[1] * (full_fov_height / max_roi_height_);
            roi_region_.roi_coordinates[2] =
                bottom_right[0] * (full_fov_width / max_roi_width_);
            roi_region_.roi_coordinates[3] =
                bottom_right[1] * (full_fov_height / max_roi_height_);
            roi_region_.roi_coordinates[4] = 1;  // Enable ROI
          } else {
            std::cout << "\n  Wrong Coordinates - Please enter ROI coordinates "
                         "in range of :["
                      << max_roi_width_ << "][" << max_roi_height_ << "]"
                      << "\n";
            break;
          }

          std::cout << "\n  Enter RGB values [R G B] :";
          std::cin >> roi_region_.rgb_color[0];
          std::cin >> roi_region_.rgb_color[1];
          std::cin >> roi_region_.rgb_color[2];
          std::cout << std::endl;
          if ((roi_region_.rgb_color[0] >= 1 &&
               roi_region_.rgb_color[0] <= 255) &&
              (roi_region_.rgb_color[1] >= 1 &&
               roi_region_.rgb_color[1] <= 255) &&
              (roi_region_.rgb_color[2] >= 1 &&
               roi_region_.rgb_color[2] <= 255)) {
            color_correct = true;
          } else {
            std::cout << "\n Wrong Color Values - Please enter ROI color in "
                         "range of :[1-255] \n";
            break;
          }
          if (coordinate_correct && color_correct) {
            meta.update(QCAMERA3_AWB_ROI_COLOR, roi_region_.rgb_color, 3);
            meta.update(ANDROID_CONTROL_AWB_REGIONS,
                        roi_region_.roi_coordinates, 5);
            status = recorder_.SetCameraParam(camera_id_, meta);
            if (status != 0) {
              TEST_ERROR("%s: Failed to set camera params", __func__);
              return status;
            }
          }
          break;
        case '2':
          status = recorder_.GetCameraParam(camera_id_, meta);
          if (NO_ERROR == status) {
            if (meta.exists(ANDROID_CONTROL_AWB_REGIONS)) {
              TEST_DBG("%s ANDROID_CONTROL_AWB_REGIONS Exists ",
                       __func__);
              roi_region_.roi_coordinates[4] = 0;  // Disable ROI
              meta.update(ANDROID_CONTROL_AWB_REGIONS,
                          roi_region_.roi_coordinates, 5);
              status = recorder_.SetCameraParam(camera_id_, meta);
              if (status != 0) {
                TEST_ERROR("%s: Failed to set camera params", __func__);
                return status;
              }
            } else {
              TEST_DBG("%s ANDROID_CONTROL_AWB_REGIONS Does not Exists ",
                       __func__);
              return status;
            }
          } else {
            TEST_ERROR("%s: Failed to Get camera params", __func__);
            return status;
          }
          break;
        case '3':
          PrintAWBROIHelp();
          break;
        case 'X':
        case 'x':
          break;
        default:
          std::cout
              << " \n Wrong option selected - Please Enter value between 1-3 "
              << std::endl;
      }
    } while (input != 'X' && input != 'x');
  } else {
    TEST_ERROR("%s: No session found for ROI to apply", __func__);
  }
  return status;
}

void RecorderTest::SnapshotCb(uint32_t camera_id,
                              uint32_t image_sequence_count,
                              BufferDescriptor buffer, MetaData meta_data) {

  TEST_INFO("%s Enter ", __func__);
  const char* ext_str;
  if (meta_data.meta_flag
      & static_cast<uint32_t> (MetaParamType::kCamBufMetaData)) {
    CameraBufferMetaData cam_buf_meta = meta_data.cam_buffer_meta_data;
    TEST_DBG("%s: format(0x%x)", __func__, cam_buf_meta.format);
    TEST_DBG("%s: num_planes=%d", __func__, cam_buf_meta.num_planes);
    for (uint8_t i = 0; i < cam_buf_meta.num_planes; ++i) {
      TEST_DBG("plane[%d]:stride(%d)", __func__, i,
          cam_buf_meta.plane_info[i].stride);
      TEST_DBG("plane[%d]:scanline(%d)", __func__, i,
          cam_buf_meta.plane_info[i].scanline);
      TEST_DBG("plane[%d]:width(%d)", __func__, i,
          cam_buf_meta.plane_info[i].width);
      TEST_DBG("plane[%d]:height(%d)", __func__, i,
          cam_buf_meta.plane_info[i].height);
    }

    if ( (is_dump_jpg_enabled_ && cam_buf_meta.format == BufferFormat::kBLOB)
      || (is_dump_raw_enabled_ && (cam_buf_meta.format == BufferFormat::kRAW10
          || cam_buf_meta.format == BufferFormat::kRAW16
          || cam_buf_meta.format == BufferFormat::kRAW8))
      || (is_dump_yuv_enabled_ && (cam_buf_meta.format == BufferFormat::kNV12
          || cam_buf_meta.format == BufferFormat::kNV21))) {
      switch (cam_buf_meta.format) {
        case BufferFormat::kNV12:
        ext_str = "nv12";
        break;
        case BufferFormat::kNV21:
        ext_str = "nv21";
        break;
        case BufferFormat::kBLOB:
        ext_str = "jpg";
        break;
        case BufferFormat::kRAW8:
        ext_str = "raw8";
        break;
        case BufferFormat::kRAW10:
        ext_str = "raw10";
        break;
        case BufferFormat::kRAW16:
        ext_str = "raw16";
        break;
        default:
        assert(0);
        break;
      }
      std::string file_path("/data/misc/qmmf/snapshot_");
      file_path += std::to_string(image_sequence_count) + "_";
      file_path += std::to_string(
          cam_buf_meta.plane_info[0].width);
      file_path += "x" + std::to_string(
          cam_buf_meta.plane_info[0].height) + "_";
      file_path += std::to_string(buffer.timestamp) + ".";
      file_path += ext_str;
      DumpFrameToFile(buffer, cam_buf_meta, file_path);
    }
  }
  // Return buffer back to recorder service.
  recorder_.ReturnImageCaptureBuffer(camera_id, buffer);
  std::unique_lock<std::mutex> lock(snapshot_wait_lock_);
  if (image_sequence_count == burst_snapshot_count_ - 1) {
    snapshot_wait_signal_.Signal();
  }

  TEST_INFO("%s Exit", __func__);
}

void RecorderTest::RecorderEventCallbackHandler(EventType event_type,
                                                void *event_data,
                                                size_t event_data_size) {
  TEST_INFO("%s: Enter", __func__);
  if (event_type == EventType::kServerDied) {
    // qmmf-server died, reason could be non recoverable FATAL error,
    // qmmf-server runs as a daemon and gets restarted automatically, on death
    // event application can cleanup all its resources and connect again.
    TEST_WARN("%s: Recorder Service died!", __func__);
  } else if (event_type == EventType::kCameraError) {
    TEST_INFO("%s: Found CameraError", __func__);
    std::lock_guard<std::mutex> lock(error_lock_);
    camera_error_ = true;
  }
  TEST_INFO("%s: Exit", __func__);
}

void RecorderTest::SessionCallbackHandler(EventType event_type,
                                          void *event_data,
                                          size_t event_data_size) {
  TEST_INFO("%s: Enter", __func__);
  TEST_INFO("%s: Exit", __func__);
}

void RecorderTest::CameraResultCallbackHandler(uint32_t camera_id,
                                               const CameraMetadata &result) {
  TEST_DBG("%s: Enter", __func__);
  status_t ret;

  if(kpi_debug_mask) {
     kpi_marker_.CheckSwicthTime(result);
  }
  camera_metadata_ro_entry aec_awb_stat_enable =
      result.find(QCAMERA3_EXPOSURE_DATA_ENABLE);
  camera_metadata_ro_entry histogram_stats =
      result.find(QCAMERA3_HISTOGRAM_STATS);

  if (dump_histogram_stats_ && histogram_stats.count > 0) {
    ret = GetRawHistogramStatistic(result);
    if (ret != 0) {
      TEST_WARN("%s: Dumping raw histogram stats failed!!!",
                __func__);
    } else {
      TEST_INFO("%s: Successfully dumped raw histogram stats!!!",
                __func__);
      dump_histogram_stats_ = false;
    }
  }

  if (dump_aec_awb_stats_ &&
      (aec_awb_stat_enable.count > 0) &&
      aec_awb_stat_enable.data.u8[0] == QCAMERA3_EXPOSURE_DATA_ON) {
    ret = GetRawAECAWBStatistic(result);
    if (ret != 0) {
      TEST_WARN("%s: Dumping raw AEC/AWB stats failed!!!",
                __func__);
    } else {
      TEST_INFO("%s: Successfully dumped raw AEC/AWB stats!!!",
                __func__);
      dump_aec_awb_stats_ = false;
    }
  }

  TEST_DBG("%s: Exit", __func__);
}

// This function dumps YUV, JPEG and RAW frames to file.
status_t RecorderTest::DumpFrameToFile(BufferDescriptor& buffer,
                                       CameraBufferMetaData& meta_data,
                                       std::string& file_path) {
  size_t written_len = 0;
  FILE *file = fopen(file_path.c_str(), "w+");
  if (!file) {
    ALOGE("%s: Unable to open file(%s)", __func__,
        file_path.c_str());
    return -1;
  }
  // JPEG, RAW & NV12UBWC
  if ( (meta_data.format == BufferFormat::kBLOB)
      || (meta_data.format == BufferFormat::kRAW10)
      || (meta_data.format == BufferFormat::kRAW8)
      || (meta_data.format == BufferFormat::kRAW12)
      || (meta_data.format == BufferFormat::kNV12UBWC) ){
    written_len = fwrite(buffer.data, sizeof(uint8_t), buffer.size, file);
  } else {
    // YUV
    uint32_t offset = 0;
    for (uint32_t i = 0; i < meta_data.num_planes; ++i) {
      void* data = static_cast<void*>((static_cast<uint8_t*>(buffer.data)
                      + offset));
      written_len += fwrite(data, sizeof(uint8_t),
          meta_data.plane_info[i].width *
          meta_data.plane_info[i].height, file);
      TEST_DBG("plane(%d) written_len = %d", __func__, i,
          written_len);
      offset += meta_data.plane_info[i].stride
                    * meta_data.plane_info[i].scanline;
    }
  }
  TEST_DBG("%s: total written_len = %d", __func__, written_len);
  TEST_INFO("%s: Buffer(0x%p) Size(%u) Stored@(%s)\n", __func__,
      buffer.data, written_len, file_path.c_str());

  fclose(file);

  return NO_ERROR;
}

int32_t RecorderTest::RunFromConfig(int32_t argc, char *argv[])
{
  ALOGD("%s: Enter ",__func__);

  CameraInitInfo *current_camera_info;
  int32_t current_camera_id;
  int32_t ret;

  if(strcmp(argv[1], "-c")) {
    ALOGD("Usage: %s -c config.txt",argv[0]);
    return -1;
  }

  TestInitParams params;
  std::vector<TrackInfo> infos;
  uint32_t session_id;
  ret = ParseConfig(argv[2], &params, &infos);
  if(ret != 0) {
    return ret;
  }
  printf("%s Connect - Start\n",__func__);

  // Connect - Start
  RecorderCb recorder_status_cb;
  recorder_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { RecorderEventCallbackHandler(event_type,
      event_data, event_data_size); };

  ret = recorder_.Connect(recorder_status_cb);

  if (NO_ERROR  != ret) {
    ALOGE("%s Connect Failed!!", __func__);
    return ret;
  }
  // Connect - End

  // StartCamera - Begin
  // TODO: this parameters to be configured from config file
  // once the proper lower layer support for zsl is added
  CameraStartParam camera_params{};
  camera_params.zsl_mode            = false;
  camera_params.zsl_queue_depth     = 10;
  camera_params.zsl_width           = 3840;
  camera_params.zsl_height          = 2160;
  camera_params.flags               = 0x0;

  camera_id_ = 0;

  for (std::vector<uint32_t>::size_type camera_index = 0;
       camera_index < params.cam_init_infos.size();
       camera_index++) {
    std::vector<TestTrack*> tracks;
    current_camera_info = params.cam_init_infos.at(camera_index);
    current_camera_id = current_camera_info->camera_id;
    if (current_camera_id == -1)
        current_camera_id = camera_id_;
    camera_params.frame_rate = current_camera_info->camera_fps;
    CameraResultCb result_cb = [&] (uint32_t camera_id,
            const CameraMetadata &result) {
            CameraResultCallbackHandler(camera_id, result); };
    printf("%s StartCamera (%d)\n",__func__, current_camera_id);
    ret = recorder_.StartCamera(current_camera_id, camera_params, result_cb);
    if(ret != 0) {
      ALOGE("%s StartCamera (%d) Failed!", __func__, current_camera_id);
      return ret;
    }

    // StartCamera - End
    printf("%s Create session and add track\n",__func__);

    // Session for encoder tracks
    SessionCb session_status_cb;
    session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
        size_t event_data_size) { SessionCallbackHandler(event_type,
        event_data, event_data_size); };

    ret = recorder_.CreateSession(session_status_cb, &session_id);
    if(ret != 0) {
      ALOGE("%s CreateSession Failed!!", __func__);
      return ret;
    }
    TEST_INFO("%s: sessions_id = %d", __func__, session_id);

    for(uint32_t i=1; i <= infos.size(); i++) {
      TrackInfo track_info = infos[i-1];
      if (track_info.camera_id == current_camera_id) {
        TestTrack *video_track = new TestTrack(this);
        track_info.track_id = session_id << 4 | i;
        track_info.session_id = session_id;
        ret = video_track->SetUp(track_info);
        assert(ret == 0);
        tracks.push_back(video_track);
        printf("%s, Create track id %d, camera_id %d\n",
            __func__, track_info.track_id, track_info.camera_id);

        if(track_info.track_type == TrackType::kAudioAAC) {
           //Test audio AAC track
           //TODO: To be removed when support added in config file
           TestTrack *audio_aac_track = new TestTrack(this);
           TrackInfo info{};
           info.track_id   = 101;
           info.track_type = TrackType::kAudioAAC;
           info.session_id = session_id;
           info.camera_id = current_camera_id;

           ret = audio_aac_track->SetUp(info);
           assert(ret == 0);
           tracks.push_back(audio_aac_track);
        }
      }
    }
    sessions_.insert(std::make_pair(session_id, tracks));
     printf("%s StartSession\n",__func__);
    // StartSession - Begin
    // Prepare tracks: setup files to dump track data, event etc.
    for (uint32_t i=0;i < tracks.size();i++) {
      tracks[i]->Prepare();
      TrackType type = tracks[i]->GetTrackType();
      if ( (type == TrackType::kVideoYUV)
        || (type == TrackType::kVideoRDI)
        || (type == TrackType::kVideoAVC)
        || (type == TrackType::kVideoHEVC)
        || (type == TrackType::kVideoPreview) ) {
        session_enabled_ = true;
      }
    }

    // Video Stabilization
    if (current_camera_info->video_stabilize) {
      ret = SetVideoStabilization(current_camera_id, true);
      if(ret != 0) {
        TEST_ERROR("%s SetVideoStabilization to On Failed!!", __func__);
        return ret;
      }
    }

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);
    struct timeval tv;
    gettimeofday(&tv, NULL);
    printf("%s: Start session at %ld:%ld for camera_id %d\n",
        __func__, tv.tv_sec, tv.tv_usec, current_camera_id);
    // StartSession - End

    // Setting binning correction off by default
    if (GetCurrentBinningCorrectionMode(current_camera_id) !=
        FEATURE_NOT_AVAILABLE) {
      ret = SetBinningCorrectionMode(current_camera_id,false);
      assert (ret == NO_ERROR);
    }

    // TNR & SHDR - Start
    CameraMetadata meta;
    auto status = recorder_.GetCameraParam(current_camera_id, meta);
    if (NO_ERROR == status) {
      if (meta.exists(ANDROID_NOISE_REDUCTION_MODE)) {
        if (current_camera_info->tnr) {
          const uint8_t tnrMode = ANDROID_NOISE_REDUCTION_MODE_HIGH_QUALITY;
          ALOGI("%s Selecting TNR mode to %s \n",
            __func__,"High quality");
          meta.update(ANDROID_NOISE_REDUCTION_MODE, &tnrMode, 1);
        } else {
          const uint8_t tnrMode = ANDROID_NOISE_REDUCTION_MODE_OFF;
          ALOGI("%s Selecting TNR mode to %s \n",__func__,"Off");
          meta.update(ANDROID_NOISE_REDUCTION_MODE, &tnrMode, 1);
        }
        status = recorder_.SetCameraParam(current_camera_id, meta);
        if (NO_ERROR != status) {
          ALOGE("%s Failed to apply: TNR/VHDR\n",__func__);
          return status;
        }
      }
    }

    if (current_camera_info->lcac_yuv) {
      ALOGI("%s Enable LCAC YUV \n", __func__);
      uint8_t enable_lcac = 1;
      status = meta.update(QCAMERA3_LCAC_PROCESSING_ENABLE, &enable_lcac, 1);
      status = recorder_.SetCameraParam(current_camera_id, meta);
      if (NO_ERROR != status) {
        ALOGE("%s Failed to Enable LCAC YUV \n", __func__);
        return status;
      }
    }
    // TODO: This value is still under discussion and verification
    PARAMETER_SETTLE_INTERVAL(2);

    status = recorder_.GetCameraParam(current_camera_id, meta);
    if (NO_ERROR == status) {
#ifdef ANDROID_O_OR_ABOVE
      uint32_t hdr_mode_vtag;
      if (VendorTagExistsInMeta(meta, String8("vhdr_mode"),
          String8("org.codeaurora.qcamera3.video_hdr_mode"),
          &hdr_mode_vtag)) {
        if (current_camera_info->vhdr) {
          const int32_t vhdrMode = VideoHDRAvailableModes::kVideoHdrOn;
          ALOGI("%s Selecting sHDR mode to %s \n",__func__,"On");
          meta.update(hdr_mode_vtag, &vhdrMode, 1);
        } else {
          const int32_t vhdrMode = VideoHDRAvailableModes::kVideoHdrOff;
          ALOGI("%s Selecting sHDR mode to %s \n",__func__,"Off");
          meta.update(hdr_mode_vtag, &vhdrMode, 1);
        }
        status = recorder_.SetCameraParam(current_camera_id, meta);
        if (NO_ERROR != status) {
          ALOGE("%s Failed to apply: TNR/VHDR\n",__func__);
          return status;
        }
      } else {
        // In case camera didn't set default turn on HDR if user requested
        if (current_camera_info->vhdr) {
          const int32_t vhdrMode = VideoHDRAvailableModes::kVideoHdrOn;
          ALOGI("%s Selecting sHDR mode to %s \n",__func__,"On");
          meta.update(hdr_mode_vtag, &vhdrMode, 1);
          status = recorder_.SetCameraParam(camera_id_, meta);
          if (NO_ERROR != status) {
            ALOGE("%s Failed to apply SHDR\n", __func__);
            return status;
          }
        }
      }
#else
      if (meta.exists(QCAMERA3_VIDEO_HDR_MODE)) {
        if (current_camera_info->vhdr) {
          const int32_t vhdrMode = QCAMERA3_VIDEO_HDR_MODE_ON;
          ALOGI("%s Selecting sHDR mode to %s \n",__func__,"On");
          meta.update(QCAMERA3_VIDEO_HDR_MODE, &vhdrMode, 1);
        } else {
          const int32_t vhdrMode = QCAMERA3_VIDEO_HDR_MODE_OFF;
          ALOGI("%s Selecting sHDR mode to %s \n",__func__,"Off");
          meta.update(QCAMERA3_VIDEO_HDR_MODE, &vhdrMode, 1);
        }
        status = recorder_.SetCameraParam(current_camera_id, meta);
        if (NO_ERROR != status) {
          ALOGE("%s Failed to apply: TNR/VHDR\n",__func__);
          return status;
        }
      } else {
        // In case camera didn't set default turn on HDR if user requested
        if (current_camera_info->vhdr) {
          const int32_t vhdrMode = QCAMERA3_VIDEO_HDR_MODE_ON;
          ALOGI("%s Selecting sHDR mode to %s \n",__func__,"On");
          meta.update(QCAMERA3_VIDEO_HDR_MODE, &vhdrMode, 1);
          status = recorder_.SetCameraParam(camera_id_, meta);
          if (NO_ERROR != status) {
            ALOGE("%s Failed to apply SHDR\n", __func__);
            return status;
          }
        }
      }
#endif

      // TODO: This value is still under discussion and verification
      PARAMETER_SETTLE_INTERVAL(2);
    }
    // TNR/SHDR - End

    if (current_camera_info->binning_correct) {
      ret = SetBinningCorrectionMode(current_camera_id, true);
      assert (ret == NO_ERROR);
    }

    if (current_camera_info->af_mode != AfMode::kNone) {
       printf("%s toggle auto focus mode\n",__func__);
       status = ToggleAFMode(current_camera_id, current_camera_info->af_mode);
       if (NO_ERROR != status) {
         TEST_ERROR("failed to toggle focus mode, status = %d\n", status);
         if (BAD_VALUE == status) {
           return BAD_VALUE;
         }
       }
    }
  }

  if (params.snapshot_info.type != SnapshotType::kNone) {

     printf("%s waiting for 2 seconds for exposure to settle\n",__func__);
     // sleep required to settle the exposure before taking snapshot.
     // This app does not provide interactive feedback to user
     // about the exposure
     PARAMETER_SETTLE_INTERVAL(AEC_SETTLE_INTERVAL);
     TEST_INFO("start to take snapshot\n");
     ret = TakeSnapshotWithConfig(params.snapshot_info);
     if (NO_ERROR != ret) {
        printf("failed to take snapshot with config\n");
        assert(ret == NO_ERROR);
     }
  }

  printf("%s Keep recording for %ds time\n",__func__,params.recordTime);
  // Keep recording for the given time
  sleep(params.recordTime);
  int32_t camera_id[3] = { -1, -1, -1};
  uint32_t camera_idx = 0;
  printf("%s StopSession\n",__func__);
  for (session_iter_ it = sessions_.begin(); it != sessions_.end(); it++) {
    session_id = it->first;
    std::vector<TestTrack*> tracks = it->second;
    // StopSession - Begin
    ret = recorder_.StopSession(session_id, true );
    assert(ret == NO_ERROR);

    for (uint32_t i=0;i < tracks.size();i++) {
      tracks[i]->CleanUp();
      TrackType type = tracks[i]->GetTrackType();
      if ( (type == TrackType::kVideoYUV)
           || (type == TrackType::kVideoRDI)
           || (type == TrackType::kVideoAVC)
           || (type == TrackType::kVideoHEVC)
           || (type == TrackType::kVideoPreview) ) {
        session_enabled_ = false;
      }
    }

    // StopSession - End
    printf("%s DeleteSession\n",__func__);

    // DeleteSession - Begin
    // Delete all the tracks associated to session.
    for (uint32_t i=0;i < tracks.size();i++) {
      if (tracks[i]->GetTrackType() == TrackType::kAudioPCM ||
            tracks[i]->GetTrackType() == TrackType::kAudioPCMFP ||
            tracks[i]->GetTrackType() == TrackType::kAudioAAC ||
            tracks[i]->GetTrackType() == TrackType::kAudioAMR ||
            tracks[i]->GetTrackType() == TrackType::kAudioG711) {
        ret = recorder_.DeleteAudioTrack(session_id, tracks[i]->GetTrackId());
      } else {
        ret = recorder_.DeleteVideoTrack(session_id, tracks[i]->GetTrackId());
      }
      assert(ret == 0);
      if(camera_id[camera_idx] == -1) {
        camera_id[camera_idx] = tracks[i]->GetCameraId();
      }
      delete tracks[i];
      tracks[i] = nullptr;
    }
    // Turn off Video stabilization if On
    if (GetCurrentVideoStabilizationMode(camera_id[camera_idx]) !=
            FEATURE_NOT_AVAILABLE) {
      ret = SetVideoStabilization(camera_id[camera_idx],false);
      if(ret != 0) {
        TEST_ERROR("%s SetVideoStabilization to OFF Failed!!", __func__);
        return ret;
      }
    }
    // Once all tracks are deleted successfully delete session.
    ret = recorder_.DeleteSession(session_id);

    // DeleteSession - End

    ++camera_idx;
    sleep(1);
  }

  for (int i = 0; i < 3; i++) {
    // StopCamera - Begin
    if (camera_id[i] > -1) {
      printf("%s StopCamera (%d)\n",__func__, camera_id[i]);
      ret = recorder_.StopCamera(camera_id[i]);
      if(ret != 0) {
        ALOGE("%s StopCamera (%d) Failed!!", __func__, camera_id[i]);
        return ret;
      }
    }
  }
  // StopCamera - End

  printf("%s Disconnect\n",__func__);

  // Disconnect - Begin
  ret = recorder_.Disconnect();
  if(ret != 0) {
    ALOGE("%s Disconnect Failed!!", __func__);
    return ret;
  }
  // Disconnect - End

  ALOGD("%s: Exit ",__func__);
  return ret;
}

void RecorderTest::printInitParamAndTtrackInfo(
    const TestInitParams &params, const std::vector<TrackInfo> &infos) {
  CameraInitInfo *cameraInfo;
  const TrackInfo *track_info;
  printf("\ninitParams.snapshot_info.camera_id = %d\n",
         params.snapshot_info.camera_id);
  printf("initParams.snapshot_info.type = %d\n", params.snapshot_info.type);
  printf("initParams.snapshot_info.width = %d\n", params.snapshot_info.width);
  printf("initParams.snapshot_info.height = %d\n", params.snapshot_info.height);
  printf("initParams.snapshot_info.count = %d\n", params.snapshot_info.count);

  for (std::vector<uint32_t>::size_type i = 0; i < params.cam_init_infos.size();
       i++) {
    cameraInfo = params.cam_init_infos.at(i);
    printf("\ncamera(%d).numStreams = %d\n", cameraInfo->camera_id,
           cameraInfo->numStream);
    printf("camera(%d).camera_fps = %d\n", cameraInfo->camera_id,
           cameraInfo->camera_fps);
    printf("camera(%d).af_mode = %d\n", cameraInfo->camera_id,
           cameraInfo->af_mode);
    printf("camera(%d).tnr = %d\n", cameraInfo->camera_id, cameraInfo->tnr);
    printf("camera(%d).vhdr = %d\n", cameraInfo->camera_id, cameraInfo->vhdr);
    printf("camera(%d).binning_correct = %d\n", cameraInfo->camera_id,
           cameraInfo->binning_correct);
    printf("camera(%d).video_stabilize = %d\n", cameraInfo->camera_id,
           cameraInfo->video_stabilize);

    for (uint32_t j = 0; j < infos.size(); j++) {
      track_info = &infos[j];
      if (track_info->camera_id == cameraInfo->camera_id) {
        printf("\n\tTrackInfo.track_type = %d\n", track_info->track_type);
        printf("\tTrackInfo.camera_id = %d\n", track_info->camera_id);
        printf("\tTrackInfo.fps = %5.2f\n", track_info->fps);
        printf("\tTrackInfo.width = %d\n", track_info->width);
        printf("\tTrackInfo.height = %d\n\n", track_info->height);
        if (track_info->track_type == TrackType::kVideoAVC) {
          printf("\tTrackInfo.bitrate = %d\n", track_info->avcparams.bitrate);
          printf("\tTrackInfo.profile = %d\n", track_info->avcparams.profile);
          printf("\tTrackInfo.level = %d\n", track_info->avcparams.level);
          printf("\tTrackInfo.ratecontrol = %d\n",
                 track_info->avcparams.ratecontrol_type);
          printf("\tTrackInfo.enable_init_qp = %d\n",
                 track_info->avcparams.qp_params.enable_init_qp);
          printf("\tTrackInfo.init_IQP = %d\n",
                 track_info->avcparams.qp_params.init_qp.init_IQP);
          printf("\tTrackInfo.init_PQP = %d\n",
                 track_info->avcparams.qp_params.init_qp.init_PQP);
          printf("\tTrackInfo.init_BQP = %d\n",
                 track_info->avcparams.qp_params.init_qp.init_BQP);
          printf("\tTrackInfo.init_QP_mode = %d\n",
                 track_info->avcparams.qp_params.init_qp.init_QP_mode);
          printf("\tTrackInfo.enable_qp_range = %d\n",
                 track_info->avcparams.qp_params.enable_qp_range);
          printf("\tTrackInfo.min_QP = %d\n",
                 track_info->avcparams.qp_params.qp_range.min_QP);
          printf("\tTrackInfo.max_QP = %d\n",
                 track_info->avcparams.qp_params.qp_range.max_QP);
          printf("\tTrackInfo.enable_qp_IBP_range = %d\n",
                 track_info->avcparams.qp_params.enable_qp_IBP_range);
          printf("\tTrackInfo.min_IQP = %d\n",
                 track_info->avcparams.qp_params.qp_IBP_range.min_IQP);
          printf("\tTrackInfo.max_IQP = %d\n",
                 track_info->avcparams.qp_params.qp_IBP_range.max_IQP);
          printf("\tTrackInfo.min_PQP = %d\n",
                 track_info->avcparams.qp_params.qp_IBP_range.min_PQP);
          printf("\tTrackInfo.max_PQP = %d\n",
                 track_info->avcparams.qp_params.qp_IBP_range.max_PQP);
          printf("\tTrackInfo.min_BQP = %d\n",
                 track_info->avcparams.qp_params.qp_IBP_range.min_BQP);
          printf("\tTrackInfo.max_BQP = %d\n",
                 track_info->avcparams.qp_params.qp_IBP_range.max_BQP);
          printf("\tTrackInfo.ltr_count = %d\n",
                 track_info->avcparams.ltr_count);
          printf("\tTrackInfo.hier_layer = %d\n",
                 track_info->avcparams.hier_layer);
          printf("\tTrackInfo.insert_aud_delimiter = %d\n",
                 track_info->avcparams.insert_aud_delimiter);
          printf("\tTrackInfo.prepend_sps_pps_to_idr = %d\n",
                 track_info->avcparams.prepend_sps_pps_to_idr);
          printf("\tTrackInfo.slice_enabled = %d\n",
                 track_info->avcparams.slice_enabled);
          printf("\tTrackInfo.slice_header_spacing = %d\n",
                 track_info->avcparams.slice_header_spacing);
        } else if (track_info->track_type == TrackType::kVideoHEVC) {
          printf("\tTrackInfo.bitrate = %d\n", track_info->hevcparams.bitrate);
          printf("\tTrackInfo.profile = %d\n", track_info->hevcparams.profile);
          printf("\tTrackInfo.level = %d\n", track_info->hevcparams.level);
          printf("\tTrackInfo.ratecontrol = %d\n",
                 track_info->hevcparams.ratecontrol_type);
          printf("\tTrackInfo.enable_init_qp = %d\n",
                 track_info->hevcparams.qp_params.enable_init_qp);
          printf("\tTrackInfo.init_IQP = %d\n",
                 track_info->hevcparams.qp_params.init_qp.init_IQP);
          printf("\tTrackInfo.init_PQP = %d\n",
                 track_info->hevcparams.qp_params.init_qp.init_PQP);
          printf("\tTrackInfo.init_BQP = %d\n",
                 track_info->hevcparams.qp_params.init_qp.init_BQP);
          printf("\tTrackInfo.init_QP_mode = %d\n",
                 track_info->hevcparams.qp_params.init_qp.init_QP_mode);
          printf("\tTrackInfo.enable_qp_range = %d\n",
                 track_info->hevcparams.qp_params.enable_qp_range);
          printf("\tTrackInfo.min_QP = %d\n",
                 track_info->hevcparams.qp_params.qp_range.min_QP);
          printf("\tTrackInfo.max_QP = %d\n",
                 track_info->hevcparams.qp_params.qp_range.max_QP);
          printf("\tTrackInfo.enable_qp_IBP_range = %d\n",
                 track_info->hevcparams.qp_params.enable_qp_IBP_range);
          printf("\tTrackInfo.min_IQP = %d\n",
                 track_info->hevcparams.qp_params.qp_IBP_range.min_IQP);
          printf("\tTrackInfo.max_IQP = %d\n",
                 track_info->hevcparams.qp_params.qp_IBP_range.max_IQP);
          printf("\tTrackInfo.min_PQP = %d\n",
                 track_info->hevcparams.qp_params.qp_IBP_range.min_PQP);
          printf("\tTrackInfo.max_PQP = %d\n",
                 track_info->hevcparams.qp_params.qp_IBP_range.max_PQP);
          printf("\tTrackInfo.min_BQP = %d\n",
                 track_info->hevcparams.qp_params.qp_IBP_range.min_BQP);
          printf("\tTrackInfo.max_BQP = %d\n",
                 track_info->hevcparams.qp_params.qp_IBP_range.max_BQP);
          printf("\tTrackInfo.ltr_count = %d\n",
                 track_info->hevcparams.ltr_count);
          printf("\tTrackInfo.hier_layer = %d\n",
                 track_info->hevcparams.hier_layer);
          printf("\tTrackInfo.insert_aud_delimiter = %d\n",
                 track_info->hevcparams.insert_aud_delimiter);
          printf("\tTrackInfo.prepend_sps_pps_to_idr = %d\n",
                 track_info->hevcparams.prepend_sps_pps_to_idr);
        }
      }
    }
  }
}

int32_t RecorderTest::ParseConfig(char *fileName, TestInitParams *initParams,
                                  std::vector<TrackInfo> *infos) {
  FILE *fp;
  TrackInfo track_info{};
  bool isStreamReadCompleted = false;
  const int MAX_LINE = 128;
  char line[MAX_LINE];
  char value[50];
  char key[25];
  uint32_t id = 0;
  int32_t camera_index = -1;
  CameraInitInfo *current_camera_info = NULL;

  if (!(fp = fopen(fileName, "r"))) {
    ALOGE("failed to open config file: %s", fileName);
    return -1;
  }

  while (fgets(line, MAX_LINE - 1, fp)) {
    if ((line[0] == '\n') || (line[0] == '/') || line[0] == ' ') continue;
    strtok(line, "\n");
    memset(value, 0x0, sizeof(value));
    memset(key, 0x0, sizeof(key));
    if (isStreamReadCompleted) {
      track_info = {};
      isStreamReadCompleted = false;
    }
    int len = strlen(line);
    int i, j = 0;

    // This assumes new stream params always start with #
    if (!strcspn(line, "#")) {
      id++;
      continue;
    }

    if (current_camera_info) {
      if ((id > 0) && (id > current_camera_info->numStream)) {
        break;
      }
    }

    int pos = strcspn(line, ":");
    for (i = 0; i < pos; i++) {
      if (line[i] != ' ') {
        key[j] = line[i];
        j++;
      }
    }

    key[j] = '\0';
    j = 0;
    for (i = pos + 1; i < len; i++) {
      if (line[i] != ' ') {
        value[j] = line[i];
        j++;
      }
    }
    value[j] = '\0';

    if (!strncmp("NumCameras", key, strlen("NumCameras"))) {
      initParams->num_cameras = atoi(value);
      printf("%s: Num cameras read %d\n", __func__, initParams->num_cameras);
      if (initParams->num_cameras > MAX_NUM_CAMERAS) {
        ALOGE("%s: Unsupported number of Cameras %d", __func__,
              initParams->num_cameras);
        goto READ_FAILED;
      }
    } else if (!strncmp("CameraID", key, strlen("CameraID"))) {
      if (++camera_index < initParams->num_cameras) {
        if (current_camera_info != NULL) {
          initParams->cam_init_infos.push_back(current_camera_info);
          current_camera_info = NULL;
        }
        current_camera_info = new CameraInitInfo();
        assert(current_camera_info != nullptr);
        current_camera_info->camera_id = atoi(value);
        id = 0;
        printf("%s: Camera ID %d\n", __func__, current_camera_info->camera_id);
      } else {
        ALOGE("%s: Number requested cameras %d are more than declared  %d",
              __func__, camera_index + 1, initParams->num_cameras);
        goto READ_FAILED;
      }
    } else if (!strncmp("CameraFPS", key, strlen("CameraFPS"))) {
      current_camera_info->camera_fps = atof(value);
    } else if (!strncmp("SnapshotType", key, strlen("SnapshotType"))) {
      if (!strncmp("None", value, strlen("None"))) {
        initParams->snapshot_info.type = SnapshotType::kNone;
      } else if (!strncmp("JPEG", value, strlen("JPEG"))) {
        initParams->snapshot_info.type = SnapshotType::kJpeg;
      } else if (!strncmp("RAWYUV", value, strlen("RAWYUV"))) {
        initParams->snapshot_info.type = SnapshotType::kRawYuv;
      } else if (!strncmp("RAWRDI", value, strlen("RAWRDI"))) {
        initParams->snapshot_info.type = SnapshotType::kRawRdi;
      } else {
        ALOGE("%s: Unknown SnapshotType(%s)", __func__, value);
        goto READ_FAILED;
      }
    } else if (!strncmp("SnapshotCameraID", key, strlen("SnapshotCameraID"))) {
      initParams->snapshot_info.camera_id = atoi(value);
    } else if (!strncmp("SnapshotWidth", key, strlen("SnapshotWidth"))) {
      initParams->snapshot_info.width = atoi(value);
    } else if (!strncmp("SnapshotHeight", key, strlen("SnapshotHeight"))) {
      initParams->snapshot_info.height = atoi(value);
    } else if (!strncmp("SnapshotCount", key, strlen("SnapshotCount"))) {
      initParams->snapshot_info.count = atoi(value);
    } else if (!strncmp("AFMode", key, strlen("AFMode"))) {
      if (!strncmp("None", value, strlen("None"))) {
        current_camera_info->af_mode = AfMode::kNone;
      } else if (!strncmp("Off", value, strlen("Off"))) {
        current_camera_info->af_mode = AfMode::kOff;
      } else if (!strncmp("AUTO", value, strlen("AUTO"))) {
        current_camera_info->af_mode = AfMode::kAuto;
      } else if (!strncmp("MACRO", value, strlen("MACRO"))) {
        current_camera_info->af_mode = AfMode::kMacro;
      } else if (!strncmp("CVAF", value, strlen("CVAF"))) {
        current_camera_info->af_mode = AfMode::kContinousVideo;
      } else if (!strncmp("CPAF", value, strlen("CPAF"))) {
        current_camera_info->af_mode = AfMode::kContinuousPicture;
      } else {
        ALOGE("%s: Unknown AFMode(%s)", __func__, value);
        goto READ_FAILED;
      }
    } else if (!strncmp("RecordingTime", key, strlen("RecordingTime"))) {
      initParams->recordTime = atoi(value);
    } else if (!strncmp("NumStream", key, strlen("NumStream"))) {
      if (atoi(value) <= 0) {
        ALOGE("%s Number of stream can not be %d", __func__, atoi(value));
        goto READ_FAILED;
      }
      current_camera_info->numStream = atoi(value);
    } else if (!strncmp("VHDR", key, strlen("VHDR"))) {
      current_camera_info->vhdr = atoi(value) ? true : false;
    } else if (!strncmp("TNR", key, strlen("TNR"))) {
      current_camera_info->tnr = atoi(value) ? true : false;
    } else if (!strncmp("BinningCorrect", key, strlen("BinningCorrect"))) {
      current_camera_info->binning_correct = atoi(value) ? true : false;
    } else if (!strncmp("VideoStabilize", key, strlen("VideoStablize"))) {
      current_camera_info->video_stabilize = atoi(value) ? true : false;
    } else if (!strncmp("LCACYUV", key, strlen("LCACYUV"))) {
      current_camera_info->lcac_yuv = atoi(value) ? true : false;
    } else if (!strncmp("Width", key, strlen("Width"))) {
      track_info.width = atoi(value);
    } else if (!strncmp("Height", key, strlen("Height"))) {
      track_info.height = atoi(value);
    } else if (!strncmp("FPS", key, strlen("FPS"))) {
      track_info.fps = atof(value);
    } else if (!strncmp("TrackType", key, strlen("TrackType"))) {
      if (!strncmp("AVC", value, strlen("AVC"))) {
        track_info.track_type = TrackType::kVideoAVC;
      } else if (!strncmp("HEVC", value, strlen("HEVC"))) {
        track_info.track_type = TrackType::kVideoHEVC;
      } else if (!strncmp("YUV", value, strlen("YUV"))) {
        track_info.track_type = TrackType::kVideoYUV;
      } else if (!strncmp("RAW", value, strlen("RAW"))) {
        track_info.track_type = TrackType::kVideoRDI;
      } else if (!strncmp("Preview", value, strlen("Preview"))) {
        track_info.track_type = TrackType::kVideoPreview;
      } else {
        ALOGE("%s: Unknown Video CodecType(%s)", __func__, value);
        goto READ_FAILED;
      }
    } else if (!strncmp("idr_interval", key, strlen("idr_interval"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.idr_interval = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.idr_interval = atoi(value);
      }
    } else if (!strncmp("Bitrate", key, strlen("Bitrate"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.bitrate = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.bitrate = atoi(value);
      }
    } else if (!strncmp("profile", key, strlen("profile"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        if (!strncmp("kBaseline", value, strlen("kBaseline"))) {
          track_info.avcparams.profile = AVCProfileType::kBaseline;
        } else if (!strncmp("kMain", value, strlen("kMain"))) {
          track_info.avcparams.profile = AVCProfileType::kMain;
        } else if (!strncmp("kHigh", value, strlen("kHigh"))) {
          track_info.avcparams.profile = AVCProfileType::kHigh;
        } else {
          ALOGE("%s: Unknown AVC Profile(%s)", __func__, value);
          goto READ_FAILED;
        }
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        if (!strncmp("kMain", value, strlen("kMain"))) {
          track_info.hevcparams.profile = HEVCProfileType::kMain;
        } else {
          ALOGE("%s: Unknown HEVC Profile(%s)", __func__, value);
          goto READ_FAILED;
        }
      }
    } else if (!strncmp("level", key, strlen("level"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        if (!strncmp("kLevel1", value, strlen("kLevel1"))) {
          track_info.avcparams.level = AVCLevelType::kLevel1;
        } else if (!strncmp("kLevel1_3", value, strlen("kLevel1_3"))) {
          track_info.avcparams.level = AVCLevelType::kLevel1_3;
        } else if (!strncmp("kLevel2", value, strlen("kLevel2"))) {
          track_info.avcparams.level = AVCLevelType::kLevel2;
        } else if (!strncmp("kLevel2_1", value, strlen("kLevel2_1"))) {
          track_info.avcparams.level = AVCLevelType::kLevel2_1;
        } else if (!strncmp("kLevel2_2", value, strlen("kLevel2_2"))) {
          track_info.avcparams.level = AVCLevelType::kLevel2_2;
        } else if (!strncmp("kLevel3", value, strlen("kLevel3"))) {
          track_info.avcparams.level = AVCLevelType::kLevel3;
        } else if (!strncmp("kLevel3_1", value, strlen("kLevel3_1"))) {
          track_info.avcparams.level = AVCLevelType::kLevel3_1;
        } else if (!strncmp("kLevel3_2", value, strlen("kLevel3_2"))) {
          track_info.avcparams.level = AVCLevelType::kLevel3_2;
        } else if (!strncmp("kLevel4", value, strlen("kLevel4"))) {
          track_info.avcparams.level = AVCLevelType::kLevel4;
        } else if (!strncmp("kLevel4_1", value, strlen("kLevel4_1"))) {
          track_info.avcparams.level = AVCLevelType::kLevel4_1;
        } else if (!strncmp("kLevel4_2", value, strlen("kLevel4_2"))) {
          track_info.avcparams.level = AVCLevelType::kLevel4_2;
        } else if (!strncmp("kLevel5", value, strlen("kLevel5"))) {
          track_info.avcparams.level = AVCLevelType::kLevel5;
        } else if (!strncmp("kLevel5_1", value, strlen("kLevel5_1"))) {
          track_info.avcparams.level = AVCLevelType::kLevel5_1;
        } else if (!strncmp("kLevel5_2", value, strlen("kLevel5_2"))) {
          track_info.avcparams.level = AVCLevelType::kLevel5_2;
        } else {
          ALOGE("%s: Unknown AVC level(%s)", __func__, value);
          goto READ_FAILED;
        }
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        if (!strncmp("kLevel3", value, strlen("kLevel3"))) {
          track_info.hevcparams.level = HEVCLevelType::kLevel3;
        } else if (!strncmp("kLevel4", value, strlen("kLevel4"))) {
          track_info.hevcparams.level = HEVCLevelType::kLevel4;
        } else if (!strncmp("kLevel5", value, strlen("kLevel5"))) {
          track_info.hevcparams.level = HEVCLevelType::kLevel5;
        } else if (!strncmp("kLevel5_1", value, strlen("kLevel5_1"))) {
          track_info.hevcparams.level = HEVCLevelType::kLevel5_1;
        } else if (!strncmp("kLevel5_2", value, strlen("kLevel5_2"))) {
          track_info.hevcparams.level = HEVCLevelType::kLevel5_2;
        }
      }
    } else if (!strncmp("ratecontrol_type", key, strlen("ratecontrol_type"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        if (!strncmp("kDisable", value, strlen("kDisable"))) {
          track_info.avcparams.ratecontrol_type =
              VideoRateControlType::kDisable;
        } else if (!strncmp("kVariableSkipFrames", value,
                            strlen("kVariableSkipFrames"))) {
          track_info.avcparams.ratecontrol_type =
              VideoRateControlType::kVariableSkipFrames;
        } else if (!strncmp("kVariable", value, strlen("kVariable"))) {
          track_info.avcparams.ratecontrol_type =
              VideoRateControlType::kVariable;
        } else if (!strncmp("kConstantSkipFrames", value,
                            strlen("kConstantSkipFrames"))) {
          track_info.avcparams.ratecontrol_type =
              VideoRateControlType::kConstantSkipFrames;
        } else if (!strncmp("kConstant", value, strlen("kConstant"))) {
          track_info.avcparams.ratecontrol_type =
              VideoRateControlType::kConstant;
        } else if (!strncmp("kMaxBitrate", value, strlen("kMaxBitrate"))) {
          track_info.avcparams.ratecontrol_type =
              VideoRateControlType::kMaxBitrate;
        } else if (!strncmp("kMaxBitrateSkipFrames", value,
                            strlen("kMaxBitrateSkipFrames"))) {
          track_info.avcparams.ratecontrol_type =
              VideoRateControlType::kMaxBitrateSkipFrames;
        } else {
          ALOGE("%s: Unknown AVC Rate Control Type(%s)", __func__, value);
          goto READ_FAILED;
        }
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        if (!strncmp("kDisable", value, strlen("kDisable"))) {
          track_info.hevcparams.ratecontrol_type =
              VideoRateControlType::kDisable;
        } else if (!strncmp("kVariableSkipFrames", value,
                            strlen("kVariableSkipFrames"))) {
          track_info.hevcparams.ratecontrol_type =
              VideoRateControlType::kVariableSkipFrames;
        } else if (!strncmp("kVariable", value, strlen("kVariable"))) {
          track_info.hevcparams.ratecontrol_type =
              VideoRateControlType::kVariable;
        } else if (!strncmp("kConstantSkipFrames", value,
                            strlen("kConstantSkipFrames"))) {
          track_info.hevcparams.ratecontrol_type =
              VideoRateControlType::kConstantSkipFrames;
        } else if (!strncmp("kConstant", value, strlen("kConstant"))) {
          track_info.hevcparams.ratecontrol_type =
              VideoRateControlType::kConstant;
        } else if (!strncmp("kMaxBitrate", value, strlen("kMaxBitrate"))) {
          track_info.hevcparams.ratecontrol_type =
              VideoRateControlType::kMaxBitrate;
        } else if (!strncmp("kMaxBitrateSkipFrames", value,
                            strlen("kMaxBitrateSkipFrames"))) {
          track_info.hevcparams.ratecontrol_type =
              VideoRateControlType::kMaxBitrateSkipFrames;
        } else {
          ALOGE("%s: Unknown HEVC Rate Control Type(%s)", __func__, value);
          goto READ_FAILED;
        }
      }
    } else if (!strncmp("enable_init_qp", key, strlen("enable_init_qp"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.qp_params.enable_init_qp = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.qp_params.enable_init_qp = atoi(value);
      }
    } else if (!strncmp("init_IQP", key, strlen("init_IQP"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.qp_params.init_qp.init_IQP = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.qp_params.init_qp.init_IQP = atoi(value);
      }
    } else if (!strncmp("init_PQP", key, strlen("init_PQP"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.qp_params.init_qp.init_PQP = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.qp_params.init_qp.init_PQP = atoi(value);
      }
    } else if (!strncmp("init_BQP", key, strlen("init_BQP"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.qp_params.init_qp.init_BQP = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.qp_params.init_qp.init_BQP = atoi(value);
      }
    } else if (!strncmp("init_QP_mode", key, strlen("init_QP_mode"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.qp_params.init_qp.init_QP_mode = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.qp_params.init_qp.init_QP_mode = atoi(value);
      }
    } else if (!strncmp("enable_qp_range", key, strlen("enable_qp_range"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.qp_params.enable_qp_range = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.qp_params.enable_qp_range = atoi(value);
      }
    } else if (!strncmp("min_QP", key, strlen("min_QP"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.qp_params.qp_range.min_QP = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.qp_params.qp_range.min_QP = atoi(value);
      }
    } else if (!strncmp("max_QP", key, strlen("max_QP"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.qp_params.qp_range.max_QP = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.qp_params.qp_range.max_QP = atoi(value);
      }
    } else if (!strncmp("enable_qp_IBP_range", key,
                        strlen("enable_qp_IBP_range"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.qp_params.enable_qp_IBP_range = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.qp_params.enable_qp_IBP_range = atoi(value);
      }
    } else if (!strncmp("min_IQP", key, strlen("min_IQP"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.qp_params.qp_IBP_range.min_IQP = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.qp_params.qp_IBP_range.min_IQP = atoi(value);
      }
    } else if (!strncmp("max_IQP", key, strlen("max_IQP"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.qp_params.qp_range.max_QP = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.qp_params.qp_range.max_QP = atoi(value);
      }
    } else if (!strncmp("min_PQP", key, strlen("min_PQP"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.qp_params.qp_IBP_range.min_PQP = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.qp_params.qp_IBP_range.min_PQP = atoi(value);
      }
    } else if (!strncmp("max_PQP", key, strlen("max_PQP"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.qp_params.qp_IBP_range.max_PQP = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.qp_params.qp_IBP_range.max_PQP = atoi(value);
      }
    } else if (!strncmp("min_BQP", key, strlen("min_BQP"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.qp_params.qp_IBP_range.min_BQP = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.qp_params.qp_IBP_range.min_BQP = atoi(value);
      }
    } else if (!strncmp("max_BQP", key, strlen("max_BQP"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.qp_params.qp_IBP_range.max_BQP = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.qp_params.qp_IBP_range.max_BQP = atoi(value);
      }
    } else if (!strncmp("Ltr_Count", key, strlen("Ltr_Count"))) {
      int32_t val = atoi(value);
      track_info.ltr_count = CLIP(val, kMinLTRCount, kMaxLTRCount);
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.ltr_count = track_info.ltr_count;
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.ltr_count = track_info.ltr_count;
      }
    } else if (!strncmp("Hier_Layer", key, strlen("Hier_Layer"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.hier_layer = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.hier_layer = atoi(value);
      }
    } else if (!strncmp("insert_aud_delimiter", key,
                        strlen("insert_aud_delimiter"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.insert_aud_delimiter = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.insert_aud_delimiter = atoi(value);
      }
    } else if (!strncmp("prepend_sps_pps_to_idr", key,
                        strlen("prepend_sps_pps_to_idr"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.prepend_sps_pps_to_idr = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        track_info.hevcparams.prepend_sps_pps_to_idr = atoi(value);
      }
    } else if (!strncmp("slice_enabled", key, strlen("slice_enabled"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.slice_enabled = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        // No support from Venus as of now
      }
    } else if (!strncmp("slice_header_spacing", key,
                        strlen("slice_header_spacing"))) {
      if (track_info.track_type == TrackType::kVideoAVC) {
        track_info.avcparams.slice_header_spacing = atoi(value);
      } else if (track_info.track_type == TrackType::kVideoHEVC) {
        // No support from Venus as of now
      }
    } else if (!strncmp("CamLowPowerMode", key, strlen("CamLowPowerMode"))) {
      track_info.low_power_mode = atoi(value) ? true : false;
      isStreamReadCompleted = true;
    } else {
      ALOGE("Unknown Key %s found in %s", key, fileName);
      goto READ_FAILED;
    }
    if (isStreamReadCompleted) {
      track_info.camera_id = current_camera_info->camera_id;
      infos->push_back(track_info);
    }
  }
  if (current_camera_info != NULL) {
    initParams->cam_init_infos.push_back(current_camera_info);
    current_camera_info = NULL;
  }

  printInitParamAndTtrackInfo(*initParams, *infos);

  fclose(fp);
  return 0;
READ_FAILED:
  fclose(fp);
  return -1;
}

int32_t RecorderTest::ParseAutoOrWarmBootModeParams(int32_t argc,
                                          char *argv[], TrackInfo& track_info) {
  ALOGD("%s: Enter ",__func__);

  if (argc != 10) {
    return -EINVAL;
  }

  int32_t val, opt;
  optind = 2;
  while ((opt = getopt(argc, argv, kAutoOrWarmBootModeArgs)) != -1) {
    switch (opt) {
      case AutoOrWarmBootModeOptions::kWidth:
        val = atoi(optarg);
        if (val < 0) {
          TEST_ERROR("%s: Invalid width = %d", __func__, val);
          return -EINVAL;
        }
        track_info.width = val;
        break;
      case AutoOrWarmBootModeOptions::kHeight:
        val = atoi(optarg);
        if (val < 0) {
          TEST_ERROR("%s: Invalid height = %d", __func__, val);
          return -EINVAL;
        }
        track_info.height = val;
        break;
      case AutoOrWarmBootModeOptions::kFps:
        val = atoi(optarg);
        if (val < 0) {
          TEST_ERROR("%s: Invalid FPS = %d", __func__, val);
          return -EINVAL;
        }
        track_info.fps = val;
        break;
      case AutoOrWarmBootModeOptions::kTrackType:
        if (!strcmp(optarg, "AVC")) {
          track_info.track_type = TrackType::kVideoAVC;
        } else if (!strcmp(optarg, "HEVC")) {
          track_info.track_type = TrackType::kVideoHEVC;
        } else {
          TEST_ERROR("%s: Invalid TrackType = %s", __func__, optarg);
          return -EINVAL;
        }
        break;
      default:
        return -EINVAL;
    }
  }

  return 0;
  ALOGD("%s: Exit ",__func__);
}

int32_t RecorderTest::StartRecording(
    const VideoTrackCreateParam &video_track_param) {

  TEST_INFO("%s: Enter ", __func__);

  int32_t ret;
  TrackCb video_track_cb;
  SessionCb session_status_cb;
  CameraStartParam camera_params;

  ret = recorder_.StartCamera(camera_id_, camera_params);
  if (ret != 0) {
    TEST_ERROR("%s StartCamera Failed!!", __func__);
    return ret;
  }

  session_status_cb.event_cb = [&](EventType event_type, void *event_data,
                                   size_t event_data_size) {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  ret = recorder_.CreateSession(session_status_cb, &current_session_id_);
  TEST_INFO("%s: sessions_id = %d", __func__, current_session_id_);
  if (ret != 0) {
    TEST_ERROR("%s CreateSession failed!!", __func__);
    recorder_.StopCamera(camera_id_);
    return ret;
  }

  video_track_cb.data_cb = [&](uint32_t track_id,
                               std::vector<BufferDescriptor> buffers,
                               std::vector<MetaData> meta_buffers) {
    recorder_.ReturnTrackBuffer(current_session_id_, 1, buffers);
  };

  video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                void *event_data, size_t data_size) {};

  ret = recorder_.CreateVideoTrack(current_session_id_, 1, video_track_param,
                                   video_track_cb);
  if (ret != 0) {
    TEST_ERROR("%s CreateVideoTrack failed!!", __func__);
    recorder_.StopSession(current_session_id_, true);
    recorder_.StopCamera(camera_id_);
    return ret;
  }

  ret = recorder_.StartSession(current_session_id_);
  if (ret != 0) {
    TEST_ERROR("%s StartSession failed!!", __func__);
    recorder_.DeleteVideoTrack(current_session_id_, 1);
    recorder_.StopSession(current_session_id_, true /*flush buffers*/);
    recorder_.StopCamera(camera_id_);
    return ret;
  }

  in_suspend_ = false;
  TEST_INFO("%s: Exit ", __func__);
  return 0;
}

int32_t RecorderTest::StopRecording() {

  TEST_INFO("%s: Enter ", __func__);

  int32_t ret;

  ret = recorder_.StopSession(current_session_id_, true /*flush buffers*/);
  if (ret != 0) {
    TEST_ERROR("%s StopSession failed!!", __func__);
    return ret;
  }

  ret = recorder_.DeleteVideoTrack(current_session_id_, 1);
  if (ret != 0) {
    TEST_ERROR("%s DeleteVideoTrack Failed!!", __func__);
    return ret;
  }

  ret = recorder_.DeleteSession(current_session_id_);
  if (ret != 0) {
    TEST_ERROR("%s DeleteSession Failed!!", __func__);
    return ret;
  }

  ret = recorder_.StopCamera(camera_id_);
  if (ret != 0) {
    TEST_ERROR("%s StopCamera Failed!!", __func__);
    return ret;
  }

  in_suspend_ = true;
  TEST_INFO("%s: Exit ", __func__);
  return 0;
}

int32_t RecorderTest::RunWarmBootMode(int32_t argc, char *argv[]) {

  TEST_INFO("%s: Enter ", __func__);

  int32_t ret;
  struct input_event ev;
  unsigned int n = 0;
  const char *input_device = "/dev/input/event0";
  const char *wake_lock_node = "/sys/power/wake_lock";
  const char *wake_unlock_node = "/sys/power/wake_unlock";
  const char *buf = "qmmf_wakelock";


  in_suspend_ = true;
  int resume_fd = open(wake_lock_node, O_WRONLY);
  if (resume_fd == -1) {
    TEST_ERROR("%s Error in opening WAKE_LOCK_NODE node", __func__);
    exit(2);
  }
  int suspend_fd = open(wake_unlock_node, O_WRONLY);
  if (suspend_fd == -1) {
    TEST_ERROR("%s Error in opening WAKE_UNLOCK_NODE node", __func__);
    exit(2);
  }
  int input_fd = open(input_device, O_RDONLY);
  if (input_fd == -1) {
    TEST_ERROR("%s Error in opening input key device node ... Exiting",
          __func__);
    exit(2);
  }
  errno = 0;
  if (write(resume_fd, buf, strlen(buf)) == -1) {
    TEST_ERROR("First Write to resume_fd failed %d (%s)\n", errno, strerror(errno));
    exit(2);
  }

  TrackInfo track_info;

  if (argc > 2) {
    ret = ParseAutoOrWarmBootModeParams(argc, argv, track_info);
    if (ret != 0) {
      TEST_ERROR(
          "%s:Usage: recorder_test testwarmboot -w <width> -h <height>"
          " -f <fps> -t <AVC/HEVC>",
          __func__);
      TEST_INFO("%s: Switching to default param", __func__);
    }
  }

  ret = Connect();
  if (NO_ERROR != ret) {
    TEST_INFO("%s Connect Failed!!", __func__);
    return ret;
  }

  VideoFormat videoformat = (track_info.track_type == TrackType::kVideoAVC)
                                ? VideoFormat::kAVC
                                : VideoFormat::kHEVC;
  VideoTrackCreateParam video_track_param{camera_id_, videoformat,
                                          track_info.width, track_info.height,
                                          track_info.fps};

  ret = StartRecording(video_track_param);
  if (NO_ERROR != ret) {
    TEST_ERROR("%s StartRecording Failed!!", __func__);
    goto exit;
  }

  while ((n = read(input_fd, &ev, sizeof(struct input_event))) > 0) {
    if (n < sizeof(struct input_event)) {
      TEST_ERROR("%s Error reading input event", __func__);
      exit(2);
    }
    if (ev.type == EV_KEY && ev.code == KEY_POWER && ev.value == 1) {

        if (in_suspend_ == false) {
          printf("PowerKey Press detected, going to suspend\n");
          ret = StopRecording();
          if (NO_ERROR != ret) {
            TEST_ERROR("%s StopRecording Failed!!", __func__);
            exit(2);
          }
          errno = 0;
          if (write(suspend_fd, buf, strlen(buf)) == -1) {
            printf("Failed to write suspend_fd %d (%s)\n", errno, strerror(errno));
            exit(2);
          }
        } else {
            printf("PowerKey Press detected, going to resume\n");
            errno = 0;
            if (write(resume_fd, buf, strlen(buf)) == -1) {
              printf("Failed to write resume_fd %d (%s)\n", errno,
                     strerror(errno));
              exit(2);
            }
            ret = StartRecording(video_track_param);
            if (NO_ERROR != ret) {
              TEST_ERROR("%s StartRecording Failed!!", __func__);
              goto exit;
            }
          }
      }
  }

  if (in_suspend_ == false) {
    ret = StopRecording();
    if (NO_ERROR != ret) {
      TEST_ERROR("%s StopRecording Failed!!", __func__);
    }
  }

exit:

  ret = Disconnect();
  if (NO_ERROR != ret) {
    TEST_ERROR("%s Disconnect Failed!!", __func__);
  }

  close(resume_fd);
  close(suspend_fd);
  close(input_fd);
  TEST_INFO("%s: Exit ", __func__);

  return 0;
}

int32_t RecorderTest::RunAutoMode(int32_t argc, char *argv[]) {
  ALOGD("%s: Enter ",__func__);

  CameraStartParam camera_params{};
  TrackCb video_track_cb;
  SessionCb session_status_cb;
  uint32_t session_id;
  TrackInfo track_info;

  auto ret = ParseAutoOrWarmBootModeParams(argc, argv, track_info);
  if (ret != 0) {
    TEST_ERROR("%s:Usage: recorder_test --auto -w <width> -h <height>"
               " -f <fps> -t <AVC/HEVC>",  __func__);
    return ret;
  }

  ret = Connect();
  if (NO_ERROR  != ret) {
    ALOGE("%s Connect Failed!!", __func__);
    return ret;
  }

  VideoFormat videoformat = (track_info.track_type == TrackType::kVideoAVC)
                                ? VideoFormat::kAVC
                                : VideoFormat::kHEVC;
  VideoTrackCreateParam video_track_param{camera_id_, videoformat,
                                          track_info.width, track_info.height,
                                          track_info.fps};

  camera_params.zsl_mode            = false;
  camera_params.zsl_queue_depth     = 10;
  camera_params.zsl_width           = 3840;
  camera_params.zsl_height          = 2160;
  camera_params.frame_rate          = track_info.fps;
  camera_params.flags               = 0x0;

  ret = recorder_.StartCamera(camera_id_, camera_params);
  if(ret != 0) {
      ALOGE("%s StartCamera Failed!!", __func__);
      goto disconnect;
  }

  session_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
      size_t event_data_size) { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  ret = recorder_.CreateSession(session_status_cb, &session_id);
  TEST_INFO("%s: sessions_id = %d", __func__, session_id);
  if(ret != 0) {
    ALOGE("%s CreateSession failed!!", __func__);
    goto stop_camera;
  }

  video_track_cb.data_cb = [&] (uint32_t track_id,
      std::vector<BufferDescriptor> buffers, std::vector<MetaData>
      meta_buffers) { recorder_.ReturnTrackBuffer(session_id, 1, buffers); };

  video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
      void *event_data, size_t data_size) { };

  ret = recorder_.CreateVideoTrack(session_id,
            1, video_track_param, video_track_cb);

  if(ret != 0) {
    ALOGE("%s CreateVideoTrack failed!!", __func__);
    goto delete_session;
  }

  ret = recorder_.StartSession(session_id);
  if(ret != 0) {
    ALOGE("%s StartSession failed!!", __func__);
    goto delete_track;
  }

  // Record video for 2 sec
  sleep(2);

  ret = recorder_.StopSession(session_id, true /*flush buffers*/);
  if(ret != 0) {
    ALOGE("%s StopSession failed!!", __func__);
  }

delete_track:
  ret = recorder_.DeleteVideoTrack(session_id, 1);    //info.track_id = 1;
  if (ret != 0) {
    ALOGE("%s DeleteVideoTrack Failed!!", __func__);
  }

delete_session:
  ret = recorder_.DeleteSession(session_id);
  if (ret != 0) {
    ALOGE("%s DeleteSession Failed!!", __func__);
  }

stop_camera:
  ret = recorder_.StopCamera(camera_id_);
  if(ret != 0) {
    ALOGE("%s StopCamera Failed!!", __func__);
  }

disconnect:
  ret = Disconnect();
  if (NO_ERROR  != ret) {
    ALOGE("%s Disconnect Failed!!", __func__);
  }

  ALOGD("%s: Exit ",__func__);
  return ret;
}

CameraMetaDataParser::CameraMetaDataParser() {
  TEST_DBG("%s: Enter", __func__);
  TEST_DBG("%s: Exit", __func__);
}

CameraMetaDataParser::~CameraMetaDataParser() {
  TEST_DBG("%s: Enter", __func__);
  TEST_DBG("%s: Exit", __func__);
}

bool CameraMetaDataParser::IsIREnabled(const CameraMetadata& metadata) {
  TEST_DBG("%s: Enter", __func__);
  bool ret = false;
  if (metadata.exists(QCAMERA3_IR_MODE)) {
    TEST_DBG("%s: Meta Exists!", __func__);
    camera_metadata_ro_entry entry = metadata.find(QCAMERA3_IR_MODE);
    uint8_t ir_camera_mode = entry.data.u8[0];
    TEST_DBG("%s: ir_camera_mode = %d", __func__,
             static_cast<uint32_t>(ir_camera_mode));
    if (QCAMERA3_IR_MODE_ON == ir_camera_mode) {
      ret = true;
    }
  }
  TEST_DBG("%s: Exit ret=%d", __func__, static_cast<uint32_t>(ret));
  return ret;
}

bool CameraMetaDataParser::IsTNREnabled(const CameraMetadata& metadata) {
  TEST_DBG("%s: Enter", __func__);
  bool ret = false;
  if (metadata.exists(ANDROID_NOISE_REDUCTION_MODE)) {
    TEST_DBG("%s: Meta Exists!", __func__);
    camera_metadata_ro_entry entry = metadata.find(
       ANDROID_NOISE_REDUCTION_MODE);
    uint8_t nr_camera_mode = entry.data.u8[0];
    TEST_DBG("%s: nr_camera_mode = %d", __func__,
             static_cast<uint32_t>(nr_camera_mode));
    if (ANDROID_NOISE_REDUCTION_MODE_HIGH_QUALITY == nr_camera_mode) {
      ret = true;
    }
  }
  TEST_DBG("%s: Exit ret=%d", __func__, static_cast<uint32_t>(ret));
  return ret;
}

bool CameraMetaDataParser::IsSVHDREnabled(const CameraMetadata& metadata) {
  TEST_DBG("%s: Enter", __func__);
  bool ret = false;

#ifdef ANDROID_O_OR_ABOVE
  uint32_t hdr_mode_vtag;
  sp<VendorTagDescriptor> vendor_tag_desc = VendorTagDescriptor
      ::getGlobalVendorTagDescriptor();
  status_t result = vendor_tag_desc->lookupTag(String8("vhdr_mode"),
      String8("org.codeaurora.qcamera3.video_hdr_mode"), &hdr_mode_vtag);
  if (0 != result) {
    TEST_ERROR("%s: TagId lookup failed with error: %d", __func__, result);
    return false;
  }
  if (metadata.exists(hdr_mode_vtag)) {
    TEST_DBG("%s: Meta Exists!", __func__);
    camera_metadata_ro_entry entry = metadata.find(hdr_mode_vtag);
    uint8_t vhdr_camera_mode = entry.data.u8[0];
    TEST_DBG("%s: vhdr_camera_mode = %d", __func__,
             static_cast<uint32_t>(vhdr_camera_mode));
    if (VideoHDRAvailableModes::kVideoHdrOn == vhdr_camera_mode) {
      ret = true;
    }
  }
#else
  if (metadata.exists(QCAMERA3_VIDEO_HDR_MODE)) {
    TEST_DBG("%s: Meta Exists!", __func__);
    camera_metadata_ro_entry entry = metadata.find(QCAMERA3_VIDEO_HDR_MODE);
    uint8_t vhdr_camera_mode = entry.data.u8[0];
    TEST_DBG("%s: vhdr_camera_mode = %d", __func__,
             static_cast<uint32_t>(vhdr_camera_mode));
    if (QCAMERA3_VIDEO_HDR_MODE_ON == vhdr_camera_mode) {
      ret = true;
    }
  }
#endif
  TEST_DBG("%s: Exit ret=%d", __func__, static_cast<uint32_t>(ret));
  return ret;
}

int64_t CameraMetaDataParser::ParseFrameTime(const CameraMetadata& metadata) {
  TEST_DBG("%s: Enter", __func__);
  int64_t ret = -1;
  if (metadata.exists(ANDROID_SENSOR_TIMESTAMP)) {
    camera_metadata_ro_entry entry = metadata.find(ANDROID_SENSOR_TIMESTAMP);
    ret = entry.data.i64[0];
  }
  TEST_DBG("%s: Exit ret=%d", __func__, static_cast<uint32_t>(ret));
  return ret;
}

CheckKPITime::CheckKPITime() :
             prev_nr_mode_(false),
             prev_ir_mode_(false),
             prev_svhdr_mode_(false),
             new_nr_mode_(false),
             new_ir_mode_(false),
             new_svhdr_mode_(false),
             last_frame_time_(0),
             new_frame_time_(0),
             mark_first_frame_time_(0) {
  TEST_DBG("%s: Enter", __func__);
  TEST_DBG("%s: Exit", __func__);
}

CheckKPITime::~CheckKPITime() {
  TEST_DBG("%s: Enter", __func__);
  TEST_DBG("%s: Exit", __func__);
}

void CheckKPITime::SetUp() {
  TEST_DBG("%s: Enter", __func__);
  mark_first_frame_time_ = true;
  last_frame_time_ = 0;
  prev_nr_mode_ = false;
  prev_ir_mode_ = false;
  prev_svhdr_mode_ = false;
  TEST_DBG("%s: Exit", __func__);
}

void CheckKPITime::CheckSwicthTime(const CameraMetadata& metadata) {
  TEST_DBG("%s: Enter", __func__);
  int64_t time_diff;
  ParseCameraMetaData(metadata);

  if (mark_first_frame_time_) {
    TEST_DBG("%s: First frame time = %lld us", __func__,
        new_frame_time_ / 1000);
    last_frame_time_ = 0;
    mark_first_frame_time_ = false;
  }

  time_diff = (new_frame_time_ - last_frame_time_) / 1000;

  if (new_nr_mode_ != prev_nr_mode_) {
    TEST_DBG("%s: NR Mode changed!", __func__);
    if (last_frame_time_) {
      TEST_KPI_ASYNC_END("TnrToggle", static_cast<int32_t>(time_diff));
    }
    prev_nr_mode_ = new_nr_mode_;
  }

  if (new_ir_mode_ != prev_ir_mode_) {
    if (last_frame_time_) {
      TEST_KPI_ASYNC_END("IrToggle", static_cast<int32_t>(time_diff));
    }
    prev_ir_mode_ = new_ir_mode_;
  }

  if (new_svhdr_mode_ != prev_svhdr_mode_) {
    TEST_DBG("%s: SHDR Mode changed!", __func__);
    if (last_frame_time_) {
      TEST_KPI_ASYNC_END("ShdrToggle", static_cast<int32_t>(time_diff));
    }
    prev_svhdr_mode_ = new_svhdr_mode_;
  }

  last_frame_time_ = new_frame_time_;
  TEST_DBG("%s: Exit", __func__);
}

void CheckKPITime::ParseCameraMetaData(const CameraMetadata& metadata) {
  TEST_DBG("%s: Enter", __func__);
  new_frame_time_ = cam_metadata_parser_.ParseFrameTime(metadata);
  new_nr_mode_ = cam_metadata_parser_.IsTNREnabled(metadata);
  new_ir_mode_ = cam_metadata_parser_.IsIREnabled(metadata);
  new_svhdr_mode_ = cam_metadata_parser_.IsSVHDREnabled(metadata);
  TEST_DBG("%s: Exit", __func__);
}

TestTrack::TestTrack(RecorderTest* recorder_test)
    : recorder_test_(recorder_test),
      num_yuv_frames_(0) {
#ifndef DISABLE_DISPLAY
  display_started_ = false;
#endif
  TEST_DBG("%s: Enter", __func__);
  track_info_ = {};
  TEST_DBG("%s: Exit", __func__);
}

TestTrack::~TestTrack() {
  TEST_DBG("%s: Enter", __func__);

  TEST_DBG("%s: Exit", __func__);
}

status_t TestTrack::SetUp(TrackInfo& track_info) {

  TEST_DBG("%s: Enter", __func__);
  int32_t ret = NO_ERROR;
  assert(recorder_test_ != nullptr);

  if ((track_info.track_type == TrackType::kVideoAVC) ||
      (track_info.track_type == TrackType::kVideoHEVC) ||
      (track_info.track_type == TrackType::kVideoRDI) ||
      (track_info.track_type == TrackType::kVideoYUV) ||
      (track_info.track_type == TrackType::kVideoPreview)) {
    float fps = track_info.fps;
    // Create Video Track.
    VideoTrackCreateParam video_track_param{};
    video_track_param.camera_id = track_info.camera_id;
    video_track_param.width = track_info.width;
    video_track_param.height = track_info.height;

    if (fps != 0)
      video_track_param.frame_rate  = fps;
    else
      video_track_param.frame_rate  = 30;
    video_track_param.low_power_mode  = track_info.low_power_mode;

    switch (track_info.track_type) {
      case TrackType::kVideoAVC:
        video_track_param.format_type = VideoFormat::kAVC;
        video_track_param.codec_param.avc.idr_interval =
            track_info.avcparams.idr_interval;
        video_track_param.codec_param.avc.bitrate =
            track_info.avcparams.bitrate;
        video_track_param.codec_param.avc.profile =
            track_info.avcparams.profile;
        video_track_param.codec_param.avc.level = track_info.avcparams.level;
        video_track_param.codec_param.avc.ratecontrol_type =
            track_info.avcparams.ratecontrol_type;
        video_track_param.codec_param.avc.qp_params.enable_init_qp =
            track_info.avcparams.qp_params.enable_init_qp;
        video_track_param.codec_param.avc.qp_params.init_qp.init_IQP =
            track_info.avcparams.qp_params.init_qp.init_IQP;
        video_track_param.codec_param.avc.qp_params.init_qp.init_PQP =
            track_info.avcparams.qp_params.init_qp.init_PQP;
        video_track_param.codec_param.avc.qp_params.init_qp.init_BQP =
            track_info.avcparams.qp_params.init_qp.init_BQP;
        video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode =
            track_info.avcparams.qp_params.init_qp.init_QP_mode;
        video_track_param.codec_param.avc.qp_params.enable_qp_range =
            track_info.avcparams.qp_params.enable_qp_range;
        video_track_param.codec_param.avc.qp_params.qp_range.min_QP =
            track_info.avcparams.qp_params.qp_range.min_QP;
        video_track_param.codec_param.avc.qp_params.qp_range.max_QP =
            track_info.avcparams.qp_params.qp_range.max_QP;
        video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range =
            track_info.avcparams.qp_params.enable_qp_IBP_range;
        video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP =
            track_info.avcparams.qp_params.qp_IBP_range.min_IQP;
        video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP =
            track_info.avcparams.qp_params.qp_IBP_range.max_IQP;
        video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP =
            track_info.avcparams.qp_params.qp_IBP_range.min_PQP;
        video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP =
            track_info.avcparams.qp_params.qp_IBP_range.max_PQP;
        video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP =
            track_info.avcparams.qp_params.qp_IBP_range.min_BQP;
        video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP =
            track_info.avcparams.qp_params.qp_IBP_range.max_BQP;
        video_track_param.codec_param.avc.ltr_count =
            track_info.avcparams.ltr_count;
        video_track_param.codec_param.avc.hier_layer =
            track_info.avcparams.hier_layer;
        video_track_param.codec_param.avc.insert_aud_delimiter =
            track_info.avcparams.insert_aud_delimiter;
        video_track_param.codec_param.avc.prepend_sps_pps_to_idr =
            track_info.avcparams.prepend_sps_pps_to_idr;
        video_track_param.codec_param.avc.slice_enabled =
            track_info.avcparams.slice_enabled;
        video_track_param.codec_param.avc.slice_header_spacing =
            track_info.avcparams.slice_header_spacing;
        break;
      case TrackType::kVideoHEVC:
        video_track_param.format_type = VideoFormat::kHEVC;
        video_track_param.codec_param.hevc.idr_interval =
            track_info.hevcparams.idr_interval;
        video_track_param.codec_param.hevc.bitrate =
            track_info.avcparams.bitrate;
        video_track_param.codec_param.hevc.profile =
            track_info.hevcparams.profile;
        video_track_param.codec_param.hevc.level = track_info.hevcparams.level;
        video_track_param.codec_param.hevc.ratecontrol_type =
            track_info.hevcparams.ratecontrol_type;
        video_track_param.codec_param.hevc.qp_params.enable_init_qp =
            track_info.hevcparams.qp_params.enable_init_qp;
        video_track_param.codec_param.hevc.qp_params.init_qp.init_IQP =
            track_info.hevcparams.qp_params.init_qp.init_IQP;
        video_track_param.codec_param.hevc.qp_params.init_qp.init_PQP =
            track_info.hevcparams.qp_params.init_qp.init_PQP;
        video_track_param.codec_param.hevc.qp_params.init_qp.init_BQP =
            track_info.hevcparams.qp_params.init_qp.init_BQP;
        video_track_param.codec_param.hevc.qp_params.init_qp.init_QP_mode =
            track_info.hevcparams.qp_params.init_qp.init_QP_mode;
        video_track_param.codec_param.hevc.qp_params.enable_qp_range =
            track_info.hevcparams.qp_params.enable_qp_range;
        video_track_param.codec_param.hevc.qp_params.qp_range.min_QP =
            track_info.hevcparams.qp_params.qp_range.min_QP;
        video_track_param.codec_param.hevc.qp_params.qp_range.max_QP =
            track_info.hevcparams.qp_params.qp_range.max_QP;
        video_track_param.codec_param.hevc.qp_params.enable_qp_IBP_range =
            track_info.hevcparams.qp_params.enable_qp_IBP_range;
        video_track_param.codec_param.hevc.qp_params.qp_IBP_range.min_IQP =
            track_info.hevcparams.qp_params.qp_IBP_range.min_IQP;
        video_track_param.codec_param.hevc.qp_params.qp_IBP_range.max_IQP =
            track_info.hevcparams.qp_params.qp_IBP_range.max_IQP;
        video_track_param.codec_param.hevc.qp_params.qp_IBP_range.min_PQP =
            track_info.hevcparams.qp_params.qp_IBP_range.min_PQP;
        video_track_param.codec_param.hevc.qp_params.qp_IBP_range.max_PQP =
            track_info.hevcparams.qp_params.qp_IBP_range.max_PQP;
        video_track_param.codec_param.hevc.qp_params.qp_IBP_range.min_BQP =
            track_info.hevcparams.qp_params.qp_IBP_range.min_BQP;
        video_track_param.codec_param.hevc.qp_params.qp_IBP_range.max_BQP =
            track_info.hevcparams.qp_params.qp_IBP_range.max_BQP;
        video_track_param.codec_param.hevc.ltr_count =
            track_info.hevcparams.ltr_count;
        video_track_param.codec_param.hevc.hier_layer =
            track_info.hevcparams.hier_layer;
        video_track_param.codec_param.hevc.insert_aud_delimiter =
            track_info.hevcparams.insert_aud_delimiter;
        video_track_param.codec_param.hevc.prepend_sps_pps_to_idr =
            track_info.hevcparams.prepend_sps_pps_to_idr;
        break;
      case TrackType::kVideoYUV:
      case TrackType::kVideoPreview:
      video_track_param.format_type = VideoFormat::kYUV;
      break;
      case TrackType::kVideoRDI:
      video_track_param.format_type = VideoFormat::kBayerRDI10BIT;
      break;
      default:
      break;
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers, std::vector<MetaData>
        meta_buffers) { TrackDataCB(track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t data_size) { TrackEventCB(track_id,
        event_type, event_data, data_size); };

    ret = recorder_test_->GetRecorder().CreateVideoTrack(track_info.session_id,
              track_info.track_id, video_track_param, video_track_cb);
    assert(ret == 0);
  } else {
    // Create AudioTrack
    AudioTrackCreateParam audio_track_params{};
    audio_track_params.in_devices_num = 0;
    audio_track_params.in_devices[audio_track_params.in_devices_num++] =
        track_info.device_id;
    audio_track_params.sample_rate = 48000;
    audio_track_params.channels    = 1;
    audio_track_params.bit_depth   = 16;
    audio_track_params.flags       = 0;

    switch (track_info.track_type) {
      case TrackType::kAudioPCM:
        audio_track_params.format = AudioFormat::kPCM;
        break;
      case TrackType::kAudioPCMFP:
        audio_track_params.format = AudioFormat::kPCM;
        ::std::string("record_fluence").copy(audio_track_params.profile,
                      strlen("record_fluence"));
        audio_track_params.sample_rate = 16000;
        break;
      case TrackType::kAudioAAC:
        audio_track_params.format = AudioFormat::kAAC;
        audio_track_params.codec_params.aac.format = AACFormat::kADTS;
        audio_track_params.codec_params.aac.mode = AACMode::kAALC;
        break;
      case TrackType::kAudioAMR:
        audio_track_params.format = AudioFormat::kAMR;
        audio_track_params.codec_params.amr.isWAMR = false;
        audio_track_params.sample_rate = 8000;
        break;
      case TrackType::kAudioG711:
        audio_track_params.format = AudioFormat::kG711;
        audio_track_params.codec_params.g711.mode = G711Mode::kALaw;
        audio_track_params.sample_rate = 8000;
        break;
      default:
        assert(0);
        break;
    }
    TrackCb audio_track_cb;
    audio_track_cb.data_cb =
        [this] (uint32_t track_id, std::vector<BufferDescriptor> buffers,
                std::vector<MetaData> meta_buffers)
                -> void {
          TrackDataCB(track_id, buffers, meta_buffers);
        };

    audio_track_cb.event_cb =
        [this] (uint32_t track_id, EventType event_type, void *event_data,
                size_t event_data_size) -> void {
          TrackEventCB(track_id, event_type, event_data, event_data_size);
        };

    ret = recorder_test_->GetRecorder().CreateAudioTrack(track_info.session_id,
              track_info.track_id, audio_track_params, audio_track_cb);
    assert(ret == NO_ERROR);

    switch (track_info.track_type) {
      case TrackType::kAudioPCM:
      case TrackType::kAudioPCMFP:
      case TrackType::kAudioG711:
        // Configure .wav output.
        ret = wav_output_.Configure(kDefaultAudioFilenamePrefix,
                                    track_info.track_id, audio_track_params);
        assert(ret == NO_ERROR);
        break;
      case TrackType::kAudioAAC:
        // Configure .aac output.
        ret = aac_output_.Configure(kDefaultAudioFilenamePrefix,
                                    track_info.track_id, audio_track_params);
        assert(ret == NO_ERROR);
        break;
      case TrackType::kAudioAMR:
        // Configure .amr output.
        ret = amr_output_.Configure(kDefaultAudioFilenamePrefix,
                                    track_info.track_id, audio_track_params);
        assert(ret == NO_ERROR);
        break;
      default:
        assert(0);
        break;
    }
  }
  track_info_ = track_info;

  TEST_DBG("%s: Exit", __func__);
  return ret;
}

// Set up file to dump track data.
status_t TestTrack::Prepare() {

  TEST_DBG("%s: Enter", __func__);
  int32_t ret = NO_ERROR;

  if (track_info_.track_type == TrackType::kVideoAVC ||
    track_info_.track_type == TrackType::kVideoHEVC) {

    VideoFormat videoformat = (track_info_.track_type == TrackType::kVideoAVC)
                   ? VideoFormat::kAVC : VideoFormat::kHEVC;
    if (recorder_test_->is_dump_bitstream_enabled_) {
      StreamDumpInfo dumpinfo = {
        videoformat, // format
        track_info_.track_id, // track_id
        (int32_t)track_info_.width, // width
        (int32_t)track_info_.height // height
      };

      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }
  }

  if (track_info_.track_type == TrackType::kAudioPCM ||
      track_info_.track_type == TrackType::kAudioPCMFP ||
      track_info_.track_type == TrackType::kAudioG711) {
    ret = wav_output_.Open();
    assert(ret == NO_ERROR);
  } else if (track_info_.track_type == TrackType::kAudioAAC) {
    ret = aac_output_.Open();
    assert(ret == NO_ERROR);
  } else if (track_info_.track_type == TrackType::kAudioAMR) {
    ret = amr_output_.Open();
    assert(ret == NO_ERROR);
  }
  TEST_DBG("%s: Exit", __func__);
  return ret;
}

// Clean up file.
status_t TestTrack::CleanUp() {

  TEST_DBG("%s: Enter", __func__);
  int32_t ret = NO_ERROR;
  switch (track_info_.track_type) {
    case TrackType::kVideoAVC:
    case TrackType::kVideoHEVC:
    dump_bitstream_.Close();
    break;
    case TrackType::kAudioPCM:
    case TrackType::kAudioPCMFP:
    case TrackType::kAudioG711:
    wav_output_.Close();
    break;
    case TrackType::kAudioAAC:
    aac_output_.Close();
    break;
    case TrackType::kAudioAMR:
    amr_output_.Close();
    break;
    default:
    break;
  }
  TEST_DBG("%s: Exit", __func__);
  return ret;
}

status_t TestTrack::EnableOverlay() {

  TEST_DBG("%s: Enter", __func__);
  int32_t ret = 0;
  OverlayParam object_params{};
  // Create Image buffer blob type overlay.
  object_params.type = OverlayType::kStaticImage;
  object_params.location = OverlayLocationType::kRandom;
  object_params.image_info.image_type = OverlayImageType::kBlobType;
  object_params.dst_rect.start_x = 1200;
  object_params.dst_rect.start_y = 580;
  object_params.dst_rect.width   = 451;
  object_params.dst_rect.height  = 109;

  object_params.image_info.source_rect.start_x = 0;
  object_params.image_info.source_rect.start_y = 0;
  object_params.image_info.source_rect.width  = 451;
  object_params.image_info.source_rect.height = 109;
  object_params.image_info.buffer_updated = false;

  FILE *image;
  image = fopen("/etc/overlay_test.rgba", "r");
  if (!image) {
   TEST_ERROR("%s: Unable to open file", __func__);
   return -1;
  }

  object_params.image_info.image_size =
      (object_params.image_info.source_rect.width *
      object_params.image_info.source_rect.height * 4);
  object_params.image_info.image_buffer =
      reinterpret_cast<char *>(malloc(sizeof(char) * object_params.image_info.image_size));

  fread(object_params.image_info.image_buffer, sizeof(char),
     object_params.image_info.image_size, image);

  fclose(image);

  uint32_t image_id;
  assert(recorder_test_ != nullptr);
  ret = recorder_test_->GetRecorder().CreateOverlayObject(track_info_.track_id,
                                                          object_params,
                                                          &image_id);
  assert(ret == 0);

  ret = recorder_test_->GetRecorder().SetOverlay(track_info_.track_id,
                                                 image_id);
  assert(ret == 0);
  // One track can have multiple types of overlay.
  overlay_ids_.push_back(image_id);

  // Create user text buffer blob type overlay.
  object_params = {};
  object_params.type = OverlayType::kStaticImage;
  object_params.location = OverlayLocationType::kRandom;
  object_params.image_info.image_type = OverlayImageType::kBlobType;
  object_params.dst_rect.start_x = 1400;
  object_params.dst_rect.start_y = 850;
  object_params.dst_rect.width   = 480;
  object_params.dst_rect.height  = 60;

  object_params.image_info.source_rect.start_x = 0;
  object_params.image_info.source_rect.start_y = 0;
  object_params.image_info.source_rect.width = 480;
  object_params.image_info.source_rect.height = 60;
  object_params.image_info.buffer_updated = false;

  object_params.image_info.image_size =
      (object_params.image_info.source_rect.width *
      object_params.image_info.source_rect.height * 4);
  object_params.image_info.image_buffer =
      reinterpret_cast<char *>(malloc(sizeof(char) * object_params.image_info.image_size));

  DrawOverlay(object_params.image_info.image_buffer,
      object_params.dst_rect.width, object_params.dst_rect.height);

  uint32_t usertxt_blob_id;
  assert(recorder_test_ != nullptr);
  ret = recorder_test_->GetRecorder().CreateOverlayObject(track_info_.track_id,
                                                          object_params,
                                                          &usertxt_blob_id);
  assert(ret == 0);

  ret = recorder_test_->GetRecorder().SetOverlay(track_info_.track_id,
                                                 usertxt_blob_id);
  assert(ret == 0);
  // One track can have multiple types of overlay.
  overlay_ids_.push_back(usertxt_blob_id);

  // Create Static Image type overlay.
  object_params = {};
  object_params.type = OverlayType::kStaticImage;
  object_params.location = OverlayLocationType::kRandom;
  object_params.image_info.image_type = OverlayImageType::kFilePath;
  object_params.dst_rect.start_x = 1400;
  object_params.dst_rect.start_y = 100;
  object_params.dst_rect.width   = 451;
  object_params.dst_rect.height  = 109;

  std::string str("/etc/overlay_test.rgba");
  str.copy(object_params.image_info.image_location, str.length());

  uint32_t object_id;
  assert(recorder_test_ != nullptr);
  ret = recorder_test_->GetRecorder().CreateOverlayObject(track_info_.track_id,
                                                          object_params,
                                                          &object_id);
  assert(ret == 0);

  ret = recorder_test_->GetRecorder().SetOverlay(track_info_.track_id,
                                                 object_id);
  assert(ret == 0);
  // One track can have multiple types of overlay.
  overlay_ids_.push_back(object_id);
  // Create Date & Time type overlay.
  object_params = {};
  object_params.type = OverlayType::kDateType;
  object_params.location = OverlayLocationType::kRandom;
  object_params.dst_rect.start_x = 1100;
  object_params.dst_rect.start_y = 50;
  object_params.dst_rect.width   = 192;
  object_params.dst_rect.height  = 108;
  object_params.color            = kColorRed;
  object_params.date_time.time_format = OverlayTimeFormatType::kHHMMSS_AMPM;
  object_params.date_time.date_format = OverlayDateFormatType::kMMDDYYYY;

  uint32_t date_time_id;
  ret = recorder_test_->GetRecorder().CreateOverlayObject(track_info_.track_id,
                                                          object_params,
                                                          &date_time_id);
  assert(ret == 0);

  ret = recorder_test_->GetRecorder().SetOverlay(track_info_.track_id,
                                                 date_time_id);
  assert(ret == 0);
  // One track can have multiple types of overlay.
  overlay_ids_.push_back(date_time_id);

  // Create BoundingBox type overlay.
  object_params = {};
  object_params.type  = OverlayType::kBoundingBox;
  object_params.color = kColorLightGreen;
  // Dummy coordinates for test purpose.
  object_params.dst_rect.start_x = 100;
  object_params.dst_rect.start_y = 200;
  object_params.dst_rect.width   = 400;
  object_params.dst_rect.height  = 400;
  std::string bb_text("Test BBox..");
  bb_text.copy(object_params.bounding_box.box_name, bb_text.length());

  uint32_t bbox_id;
  ret = recorder_test_->GetRecorder().CreateOverlayObject(track_info_.track_id,
                                                          object_params,
                                                          &bbox_id);
  assert(ret == 0);
  ret = recorder_test_->GetRecorder().SetOverlay(track_info_.track_id, bbox_id);
  assert(ret == 0);
  overlay_ids_.push_back(bbox_id);

  // Create UserText type overlay.
  object_params = {};
  object_params.type = OverlayType::kUserText;
  object_params.location = OverlayLocationType::kRandom;
  object_params.color = kColorLightBlue;
  object_params.dst_rect.start_x = 200;
  object_params.dst_rect.start_y = 800;
  object_params.dst_rect.width   = 480;
  object_params.dst_rect.height  = 60;
  std::string user_text("Simple User Text For Testing!!");
  user_text.copy(object_params.user_text, user_text.length());

  uint32_t user_text_id;
  ret = recorder_test_->GetRecorder().CreateOverlayObject(track_info_.track_id,
                                                          object_params,
                                                          &user_text_id);
  assert(ret == 0);
  ret = recorder_test_->GetRecorder().SetOverlay(track_info_.track_id,
                                                 user_text_id);
  assert(ret == 0);
  overlay_ids_.push_back(user_text_id);

  // Create PrivacyMask type overlay.
  object_params = {};
  object_params.type = OverlayType::kPrivacyMask;
  object_params.color = kColorLightGray; //light gray
  // Dummy coordinates for test purpose.
  object_params.dst_rect.start_x = 800;
  object_params.dst_rect.start_y = 250;
  object_params.dst_rect.width   = 1920/4;
  object_params.dst_rect.height  = 1080/4;

  uint32_t privacy_mask_id;
  ret = recorder_test_->GetRecorder().CreateOverlayObject(track_info_.track_id,
                                                          object_params,
                                                          &privacy_mask_id);
  assert(ret == 0);
  ret = recorder_test_->GetRecorder().SetOverlay(track_info_.track_id,
                                                 privacy_mask_id);
  assert(ret == 0);
  overlay_ids_.push_back(privacy_mask_id);
  TEST_DBG("%s: Exit", __func__);
  return ret;
}

status_t TestTrack::DisableOverlay() {

  TEST_DBG("%s: Enter", __func__);
  int32_t ret = 0;
  assert(recorder_test_ != nullptr);
  for (auto overlay_id : overlay_ids_) {
    ret = recorder_test_->GetRecorder().RemoveOverlay(GetTrackId(), overlay_id);
    assert(ret == 0);
    ret = recorder_test_->GetRecorder().DeleteOverlayObject(GetTrackId(),
                                                            overlay_id);
    assert(ret == 0);
  }
  overlay_ids_.clear();
  TEST_DBG("%s: Exit", __func__);
  return ret;
}

status_t TestTrack::DrawOverlay(void *data, int32_t width, int32_t height) {

  TEST_DBG("%s: Enter", __func__);
  status_t ret = 0;
  std::string text("User Text Bolb Test");
#if USE_SKIA
  int32_t text_size = 40;
  //Create Skia canvas outof ION memory.
  SkImageInfo imageInfo = SkImageInfo::Make(width, height,
      kRGBA_8888_SkColorType, kPremul_SkAlphaType);

#ifdef ANDROID_O_OR_ABOVE
  canvas_ = (SkCanvas::MakeRasterDirect(imageInfo,
      static_cast<unsigned char*>(data), width * 4)).release();
#else
  canvas_ = SkCanvas::NewRasterDirect(imageInfo,
      static_cast<unsigned char*>(data), width * 4);
#endif

  canvas_->clear(SK_AlphaOPAQUE);
  SkPaint paint;
  paint.setColor(kColorRed);
  paint.setTextSize(SkIntToScalar(text_size));
  paint.setAntiAlias(true);

  int32_t x = 0;
  int32_t y = width - text_size/2;
  SkString sk_text(text.c_str(), text.length());
  canvas_->drawText(sk_text.c_str(), sk_text.size(), x, y, paint);
  canvas_->flush();

#elif USE_CAIRO
  cr_surface_ = cairo_image_surface_create_for_data(static_cast<unsigned char*>
                                                    (data),
                                                    CAIRO_FORMAT_ARGB32, width,
                                                    height, width * 4);
  assert (cr_surface_ != nullptr);

  cr_context_ = cairo_create (cr_surface_);
  assert (cr_context_ != nullptr);

  cairo_select_font_face(cr_context_, "@cairo:Georgia", CAIRO_FONT_SLANT_NORMAL,
                          CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size (cr_context_, TEXT_SIZE);
  cairo_set_antialias (cr_context_, CAIRO_ANTIALIAS_BEST);
  assert(CAIRO_STATUS_SUCCESS == cairo_status(cr_context_));

  cairo_font_extents_t font_extent;
  cairo_font_extents (cr_context_, &font_extent);
  TEST_DBG("%s: ascent=%f, descent=%f, height=%f, max_x_advance=%f,"
      " max_y_advance = %f", __func__, font_extent.ascent, font_extent.descent,
       font_extent.height, font_extent.max_x_advance,
       font_extent.max_y_advance);

  cairo_text_extents_t text_extents;
  cairo_text_extents (cr_context_, text.c_str(), &text_extents);

  TEST_DBG("%s: Custom text: te.x_bearing=%f, te.y_bearing=%f,"
      " te.width=%f, te.height=%f, te.x_advance=%f, te.y_advance=%f", __func__,
      text_extents.x_bearing, text_extents.y_bearing,
      text_extents.width, text_extents.height,
      text_extents.x_advance, text_extents.y_advance);

  cairo_font_options_t *options;
  options = cairo_font_options_create ();
  cairo_font_options_set_antialias (options, CAIRO_ANTIALIAS_DEFAULT);
  cairo_set_font_options (cr_context_, options);
  cairo_font_options_destroy (options);

  //(0,0) is at topleft corner of draw buffer.
  double x_text = 0.0;
  double y_text = text_extents.height - (font_extent.descent/2.0);
  TEST_DBG("%s: x_text=%f, y_text=%f", __func__, x_text, y_text);
  cairo_move_to (cr_context_, x_text, y_text);

  // Draw Text.
  RGBAValues text_color{};
  ExtractColorValues(kColorRed, &text_color);
  cairo_set_source_rgba (cr_context_, text_color.red, text_color.green,
                         text_color.blue, text_color.alpha);

  cairo_show_text (cr_context_, text.c_str());
  assert(CAIRO_STATUS_SUCCESS == cairo_status(cr_context_));
  cairo_surface_flush(cr_surface_);

  if (cr_surface_) {
    cairo_surface_destroy(cr_surface_);
  }
  if (cr_context_) {
    cairo_destroy(cr_context_);
  }
#endif

  TEST_DBG("%s: Exit", __func__);
  return ret;
}

void TestTrack::ExtractColorValues(uint32_t hex_color, RGBAValues* color) {

  color->red   = ((hex_color >> 24) & 0xff) / 255.0;
  color->green = ((hex_color >> 16) & 0xff) / 255.0;
  color->blue  = ((hex_color >> 8) & 0xff) / 255.0;
  color->alpha = ((hex_color) & 0xff) / 255.0;
}

void TestTrack::TrackEventCB(uint32_t track_id, EventType event_type,
                             void *event_data, size_t event_data_size) {

  TEST_DBG("%s: Enter", __func__);
  TEST_DBG("%s: Exit", __func__);
}

void TestTrack::TrackDataCB(uint32_t track_id, std::vector<BufferDescriptor>
                            buffers, std::vector<MetaData> meta_buffers) {

  TEST_DBG("%s: Enter track_id(%dd)", __func__, track_id);
  assert (recorder_test_ != nullptr);
  int32_t ret = 0;

  switch (track_info_.track_type) {
    case TrackType::kAudioPCM:
    case TrackType::kAudioPCMFP:
    case TrackType::kAudioG711:
      for (const BufferDescriptor& buffer : buffers) {
        ret = wav_output_.Write(buffer);
        assert(ret == 0);
      }
    break;
    case TrackType::kAudioAAC:
      for (const BufferDescriptor& buffer : buffers) {
        if (buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS))
          break;
        ret = aac_output_.Write(buffer);
        assert(ret == 0);
      }
    break;
    case TrackType::kAudioAMR:
      for (const BufferDescriptor& buffer : buffers) {
        if (buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS))
          break;
        ret = amr_output_.Write(buffer);
        assert(ret == 0);
      }
    break;
    case TrackType::kVideoYUV:
    case TrackType::kVideoRDI:
      for (uint32_t i = 0; i < meta_buffers.size(); ++i) {
        MetaData meta_data = meta_buffers[i];
        if (meta_data.meta_flag &
            static_cast<uint32_t>(MetaParamType::kCamBufMetaData)) {
          CameraBufferMetaData cam_buf_meta = meta_data.cam_buffer_meta_data;
          TEST_DBG("%s: format=%d", __func__, cam_buf_meta.format);
          TEST_DBG("%s: num_planes=%d", __func__,
              cam_buf_meta.num_planes);
          for (uint8_t i = 0; i < cam_buf_meta.num_planes; ++i) {
            TEST_DBG("plane[%d]:stride(%d)", __func__, i,
                cam_buf_meta.plane_info[i].stride);
            TEST_DBG("plane[%d]:scanline(%d)", __func__, i,
                cam_buf_meta.plane_info[i].scanline);
            TEST_DBG("plane[%d]:width(%d)", __func__, i,
                cam_buf_meta.plane_info[i].width);
            TEST_DBG("plane[%d]:height(%d)", __func__, i,
                cam_buf_meta.plane_info[i].height);
          }

          if ((recorder_test_->is_dump_yuv_enabled_ &&
               track_info_.track_type == TrackType::kVideoYUV) ||
              (recorder_test_->is_dump_raw_enabled_ &&
               track_info_.track_type == TrackType::kVideoRDI)) {
            ++num_yuv_frames_;
            if (num_yuv_frames_ == recorder_test_->dump_frame_freq_) {
              const char *ext = track_info_.track_type ==  TrackType::kVideoRDI ?
                  "raw" : "yuv";
              std::string file_path("/data/misc/qmmf/track_");
              file_path += std::to_string(track_info_.track_id) + "_";
              file_path += std::to_string(
                  cam_buf_meta.plane_info[0].width);
              file_path += "x" + std::to_string(
                  cam_buf_meta.plane_info[0].height) + "_";
              file_path += std::to_string(buffers[i].timestamp) + ".";
              file_path += ext;
              recorder_test_->DumpFrameToFile(buffers[i], cam_buf_meta,
                                            file_path);
              num_yuv_frames_ = 0;
            }
          }
#ifndef DISABLE_DISPLAY
          PushFrameToDisplay(buffers[i], cam_buf_meta);
#endif
        }
      }
    break;
    case TrackType::kVideoAVC:
    case TrackType::kVideoHEVC:
      for (uint32_t i = 0; i < meta_buffers.size(); ++i) {
        MetaData meta_data = meta_buffers[i];
        if (meta_data.meta_flag &
            static_cast<uint32_t>(MetaParamType::kVideoFrameType)) {
          TEST_DBG("%s: frame_type=%d", __func__,
                   meta_data.video_frame_type_info);
        }
      }

      if (recorder_test_->is_dump_bitstream_enabled_) {
        dump_bitstream_.Dump(buffers);
      }
    break;
    default:
    break;
  }
  // Return buffers back to service.
  ret = recorder_test_->GetRecorder().ReturnTrackBuffer(track_info_.session_id,
                                                        track_id, buffers);
  assert(ret == 0);
  TEST_DBG("%s: Exit", __func__);
}

#ifndef DISABLE_DISPLAY
void TestTrack::DisplayCallbackHandler(DisplayEventType event_type,
    void *event_data, size_t event_data_size) {
  TEST_DBG("%s Enter ", __func__);
  TEST_DBG("%s Exit ", __func__);
}

void TestTrack::DisplayVSyncHandler(int64_t time_stamp) {
  TEST_DBG("%s: Enter", __func__);
  TEST_DBG("%s: Exit", __func__);
}

status_t TestTrack::StartDisplay(DisplayType display_type) {
  TEST_INFO("%s: Enter", __func__);
  int32_t res = 0;
  SurfaceConfig surface_config{};
  DisplayCb  display_status_cb;

  display_= new Display();
  assert(display_ != nullptr);

  res = display_->Connect();
  assert(res == 0);

  display_status_cb.EventCb = [&] ( DisplayEventType event_type,
      void *event_data, size_t event_data_size) { DisplayCallbackHandler
      (event_type, event_data, event_data_size); };

  display_status_cb.VSyncCb = [&] ( int64_t time_stamp)
      { DisplayVSyncHandler(time_stamp); };

  res = display_->CreateDisplay(display_type, display_status_cb);
  assert(res == 0);

  surface_config.width = track_info_.width;
  surface_config.height = track_info_.height;
  surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
  surface_config.buffer_count = 1;
  surface_config.cache = 0;
  surface_config.use_buffer = 1;
  surface_config.z_order = 1;
  res = display_->CreateSurface(surface_config, &surface_id_);
  assert(res == 0);

  display_started_ = 1;

  surface_param_.src_rect = { 0.0, 0.0, (float)track_info_.width,
      (float)track_info_.height };
  surface_param_.dst_rect = { 0.0, 0.0, (float)track_info_.width,
      (float)track_info_.height };
  surface_param_.surface_blending =
      SurfaceBlending::kBlendingCoverage;
  surface_param_.surface_flags.cursor = 0;
  surface_param_.frame_rate = track_info_.fps;
  surface_param_.solid_fill_color = 0;
  surface_param_.surface_transform.rotation = 0.0f;
  surface_param_.surface_transform.flip_horizontal = 0;
  surface_param_.surface_transform.flip_vertical = 0;

  TEST_INFO("%s: Exit", __func__);
  return res;
}

status_t TestTrack::StopDisplay(DisplayType display_type) {
  TEST_INFO("%s: Enter", __func__);
  int32_t res = 0;

  if (display_started_ == 1) {
    display_started_ = 0;
    res = display_->DestroySurface(surface_id_);
    if (res != 0) {
      TEST_ERROR("%s DestroySurface Failed!!", __func__);
    }

    res = display_->DestroyDisplay(display_type);
    if (res != 0) {
      TEST_ERROR("%s DestroyDisplay Failed!!", __func__);
    }
    res = display_->Disconnect();

    if (display_ != nullptr) {
      TEST_INFO("%s: DELETE display_:%p", __func__, display_);
      delete display_;
      display_ = nullptr;
    }
  }
  TEST_INFO("%s: Exit", __func__);
  return res;
}

status_t TestTrack::PushFrameToDisplay(BufferDescriptor& buffer,
    CameraBufferMetaData& meta_data) {
  if (display_started_ == 1) {
    int32_t ret;
    surface_buffer_.plane_info[0].ion_fd = buffer.fd;
    surface_buffer_.buf_id = buffer.fd;
    surface_buffer_.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
    surface_buffer_.plane_info[0].stride = meta_data.plane_info[0].stride;
    surface_buffer_.plane_info[0].size = buffer.size;
    surface_buffer_.plane_info[0].width = meta_data.plane_info[0].width;
    surface_buffer_.plane_info[0].height = meta_data.plane_info[0].height;
    surface_buffer_.plane_info[0].offset = 0;
    surface_buffer_.plane_info[0].buf = buffer.data;

    ret = display_->QueueSurfaceBuffer(surface_id_, surface_buffer_,
        surface_param_);
    if (ret != 0) {
      TEST_ERROR("%s QueueSurfaceBuffer Failed!!", __func__);
      return ret;
    }

    ret = display_->DequeueSurfaceBuffer(surface_id_, surface_buffer_);
    if (ret != 0) {
      TEST_ERROR("%s DequeueSurfaceBuffer Failed!!", __func__);
    }
  }
  return NO_ERROR;
}
#endif

status_t DumpBitStream::SetUp(const StreamDumpInfo& dumpinfo) {

  TEST_DBG("%s: Enter", __func__);
  assert(dumpinfo.width > 0);
  assert(dumpinfo.height > 0);

  Close();

  const char* type_string;
  switch (dumpinfo.format) {
    case VideoFormat::kAVC:
      type_string = "h264";
      break;
    case VideoFormat::kHEVC:
      type_string = "h265";
      break;
    default:
      type_string = "bin";
      break;
  }
  struct timeval tv;
  gettimeofday(&tv, NULL);
  std::string extn(type_string);
  std::string bitstream_filepath("/data/misc/qmmf/test_track_");
  bitstream_filepath += std::to_string(dumpinfo.track_id) + "_";
  bitstream_filepath += std::to_string(dumpinfo.width) + "x";
  bitstream_filepath += std::to_string(dumpinfo.height) + "_";
  bitstream_filepath += std::to_string(tv.tv_sec) + ".";
  bitstream_filepath += extn;
  file_fd_ = open(bitstream_filepath.c_str(),
                          O_CREAT | O_WRONLY | O_TRUNC, 0655);
  if (file_fd_ <= 0) {
    TEST_ERROR("%s File open failed!", __func__);
    return BAD_VALUE;
  }

  TEST_DBG("%s: Exit", __func__);
  return NO_ERROR;
}

void DumpBitStream::Close() {
  TEST_DBG("%s: Enter", __func__);
  if (file_fd_ > 0) {
    close(file_fd_);
    file_fd_ = -1;
  }
  TEST_DBG("%s: Exit", __func__);
}

status_t DumpBitStream::Dump(const std::vector<BufferDescriptor>& buffers) {

  TEST_DBG("%s: Enter", __func__);
  assert(file_fd_ > 0);

  for (auto& iter : buffers) {
    uint32_t exp_size = iter.size;
    TEST_DBG("%s: BitStream buffer data(0x%x):size(%d):ts(%lld):flag(0x%x)"
      ":buf_id(%d):capacity(%d)",  __func__, iter.data, iter.size,
       iter.timestamp, iter.flag, iter.buf_id, iter.capacity);

    uint32_t written_length = write(file_fd_, iter.data, iter.size);
    TEST_DBG("%s: written_length(%d)", __func__, written_length);
    if (written_length != exp_size) {
      TEST_ERROR("%s: Bad Write error (%d) %s", __func__, errno,
      strerror(errno));
      return BAD_VALUE;
    }

    if (iter.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS)) {
      TEST_INFO("%s EOS Last buffer!", __func__);
      Close();
    }
  }

  TEST_DBG("%s: Exit", __func__);
  return NO_ERROR;
}

void CmdMenu::PrintMenu() {
  printf("\n\n=========== QMMF RECORDER TEST MENU ===================\n\n");

  printf(" Recorder Test Application commands \n");
  printf(" ---------------------------------\n");
  printf("   %c. Connect\n", CmdMenu::CONNECT_CMD);
  printf("   %c. Disconnect\n", CmdMenu::DISCONNECT_CMD);
  printf("   %c. Choose camera\n", CmdMenu::CHOOSE_CAMERA_CMD);
  printf("   %c. Start Camera\n", CmdMenu::START_CAMERA_CMD);
  printf("   %c. Stop Camera\n", CmdMenu::STOP_CAMERA_CMD);
  printf("   %c. Start Multi Camera Mode \n", CmdMenu::START_MULTICAMERA_CMD);
  printf("   %c. Stop Multi Camera Mode \n", CmdMenu::STOP_MULTICAMERA_CMD);
  printf("   %c. Create Session: (4K YUV + 1080 YUV)\n",
      CmdMenu::CREATE_YUV_SESSION_CMD);
  printf("   %c. Create Session: (4K Enc AVC)\n",
      CmdMenu::CREATE_4KENC_AVC_SESSION_CMD);
  printf("   %c. Create Session: (4K Enc HEVC)\n",
      CmdMenu::CREATE_4KENC_HEVC_SESSION_CMD);
  printf("   %c. Create Session: (1080p AVC + AAC)\n",
      CmdMenu::CREATE_1080pAVC_AAC_AUD_SESSION_CMD);
  printf("   %c. Create Session: (1080p HEVC + AAC)\n",
      CmdMenu::CREATE_1080pHEVC_AAC_AUD_SESSION_CMD);
  printf("   %c. Create Session: (4K YUV + 1080p Enc AVC)\n",
    CmdMenu::CREATE_4KYUV_1080pENC_SESSION_CMD);
  printf("   %c. Create Session: (Two 1080p Enc AVC)\n",
    CmdMenu::CREATE_TWO_1080pENC_SESSION_CMD);
  printf("   %c. Create Session: (1080p Enc AVC + 1080 YUV)\n",
    CmdMenu::CREATE_1080pENC_AVC_1080YUV_SESSION_CMD);
  printf("   %c. Create Session: (4K Enc HEVC + 1080 YUV)\n",
    CmdMenu::CREATE_4KENC_HEVC_1080YUV_SESSION_CMD);
  printf("   %c. Create Session: (720p LPM YUV)\n",
    CmdMenu::CREATE_720pLPM_SESSION_CMD);
  printf("   %c. Create Session: (1080p Enc AVC + 1080 LPM YUV)\n",
      CmdMenu::CREATE_1080pENC_AVC_1080LPM_SESSION_CMD);
  printf("   %c. Create Session: (RDI)\n",
      CmdMenu::CREATE_RDI_SESSION_CMD);
  printf("   %c. Create Session: (PCM mono,16,48KHz)\n",
      CmdMenu::CREATE_PCM_AUD_SESSION_CMD);
  printf("   %c. Create Session: (PCM mono,16,48KHz + PCM mono,16,48KHz)\n",
      CmdMenu::CREATE_2PCM_AUD_SESSION_CMD);
  printf("   %c. Create Session: (SCO mono,16,48KHz)\n",
      CmdMenu::CREATE_SCO_AUD_SESSION_CMD);
  printf("   %c. Create Session: (PCM mono,16,48KHz + SCO mono,16,48KHz)\n",
      CmdMenu::CREATE_PCM_SCO_AUD_SESSION_CMD);
#ifdef ENABLE_A2DP_USECASE
  printf("   %c. Create Session: (A2DP mono,16,48KHz)\n",
      CmdMenu::CREATE_A2DP_AUD_SESSION_CMD);
  printf("   %c. Create Session: (PCM mono,16,48KHz + A2DP mono,16,48KHz)\n",
      CmdMenu::CREATE_PCM_A2DP_AUD_SESSION_CMD);
#endif
  printf("   %c. Create Session: (AAC mono)\n",
      CmdMenu::CREATE_AAC_AUD_SESSION_CMD);
  printf("   %c. Create Session: (AAC mono + AAC mono)\n",
      CmdMenu::CREATE_2AAC_AUD_SESSION_CMD);
  printf("   %c. Create Session: (PCM mono,16,48KHz + AAC mono)\n",
    CmdMenu::CREATE_PCM_AAC_AUD_SESSION_CMD);
  printf("   %c. Create Session: (AMR mono)\n",
      CmdMenu::CREATE_AMR_AUD_SESSION_CMD);
  printf("   %c. Create Session: (AMR mono + AMR mono)\n",
      CmdMenu::CREATE_2AMR_AUD_SESSION_CMD);
  printf("   %c. Create Session: (PCM mono,16,8KHz + AMR mono)\n",
      CmdMenu::CREATE_PCM_AMR_AUD_SESSION_CMD);
  printf("   %c. Create Session: (G711 mono)\n",
      CmdMenu::CREATE_G7ll_AUD_SESSION_CMD);
  printf("   %c. Create Session: (G711 mono + G711 mono)\n",
      CmdMenu::CREATE_2G7ll_AUD_SESSION_CMD);
  printf("   %c. Create Session: (PCM mono,16,8KHz + G711 mono)\n",
      CmdMenu::CREATE_PCM_G7ll_AUD_SESSION_CMD);
  printf("   %c. Create Session: (PCM mono,16,16KHz,FluencePro)\n",
      CmdMenu::CREATE_PCMFL_AUD_SESSION_CMD);
  printf("   %c. Create Session: (1080p YUV with Display)\n",
      CmdMenu::CREATE_YUV_SESSION_DISPLAY_CMD);
  printf("   %c. Create Session: (1080p YUV with Preview)\n",
      CmdMenu::CREATE_YUV_SESSION_PREVIEW_CMD);
  printf("   %c. Start Session\n", CmdMenu::START_SESSION_CMD);
  printf("   %c. Stop Session\n", CmdMenu::STOP_SESSION_CMD);
  printf("   %c. Take Snapshot\n", CmdMenu::TAKE_SNAPSHOT_CMD);
  printf("   %c. Set Dynamic Codec Param \n", CmdMenu::SET_PARAM_CMD);
  printf("   %c. Change Camera Param \n",
      CmdMenu::SET_DYNAMIC_CAMERA_PARAM_CMD);
  printf("   %c. Pause Session\n", CmdMenu::PAUSE_SESSION_CMD);
  printf("   %c. Resume Session\n", CmdMenu::RESUME_SESSION_CMD);
  printf("   %c. Enable Overlay\n", CmdMenu::ENABLE_OVERLAY_CMD);
  printf("   %c. Disable Overlay\n", CmdMenu::DISABLE_OVERLAY_CMD);
  printf("   %c. Delete Session\n", CmdMenu::DELETE_SESSION_CMD);
  if (ctx_.session_enabled_) {
    printf("   %c. NR mode: %s\n", CmdMenu::NOISE_REDUCTION_CMD,
           ctx_.GetCurrentNRMode().c_str());
    printf("   %c. VHDR: %s\n", CmdMenu::VIDEO_HDR_CMD,
           ctx_.GetCurrentVHDRMode().c_str());

    printf("   %c. IR: %s\n", CmdMenu::IR_MODE_CMD,
           ctx_.GetCurrentIRMode().c_str());
    printf("   %c. Set AWB ROI\n", CmdMenu::AWB_ROI_CMD);
  } else {
    printf("   %c. Toggle Video stabilization "
           "[Use after create session (1080p ONLY!)]: %s\n",
               CmdMenu::VIDEO_STABILZATION_CMD,
               ctx_.GetCurrentVideoStabilizationMode(ctx_.camera_id_).c_str());
  }
  printf("   %c. Set Antibanding mode\n", CmdMenu::SET_ANTIBANDING_MODE_CMD);
  printf("   %c. Exit\n", CmdMenu::EXIT_CMD);
  printf("\n   Choice: ");
}

CmdMenu::Command CmdMenu::GetCommand(bool& is_print_menu) {
  if (is_print_menu) {
    PrintMenu();
    is_print_menu = false;
  }
  return CmdMenu::Command(static_cast<CmdMenu::CommandType>(getchar()));
}

int main(int argc,char *argv[]) {
  QMMF_GET_LOG_LEVEL();
  TEST_INFO("%s: Enter", __func__);

  RecorderTest test_context;

  if (argc > 1) {
    if (strcmp(argv[1], "-a") == 0) {
      return test_context.RunAutoMode(argc, argv);
    } else if (strcmp(argv[1], "--testwarmboot") == 0) {
      return test_context.RunWarmBootMode(argc, argv);
    } else
      return test_context.RunFromConfig(argc, argv);
  }

  CmdMenu cmd_menu(test_context);
  bool is_print_menu = true;
  int32_t exit_test = false;

  while (!exit_test) {

    CmdMenu::Command command = cmd_menu.GetCommand(is_print_menu);
    switch (command.cmd) {

      case CmdMenu::CONNECT_CMD: {
        test_context.Connect();
      }
      break;
      case CmdMenu::DISCONNECT_CMD: {
        test_context.Disconnect();
      }
      break;
      case CmdMenu::START_CAMERA_CMD: {
        test_context.StartCamera();
      }
      break;
      case CmdMenu::STOP_CAMERA_CMD: {
        test_context.StopCamera();
      }
      break;
      case CmdMenu::START_MULTICAMERA_CMD: {
        test_context.StartMultiCameraMode();
      }
      break;
      case CmdMenu::STOP_MULTICAMERA_CMD: {
        test_context.StopMultiCameraMode();
      }
      break;
      case CmdMenu::CREATE_YUV_SESSION_CMD: {
        test_context.Session4KAnd1080pYUVTracks();
      }
      break;
      case CmdMenu::CREATE_4KENC_AVC_SESSION_CMD: {
        test_context.Session4KEncTrack(TrackType::kVideoAVC);
      }
      break;
      case CmdMenu::CREATE_4KENC_HEVC_SESSION_CMD: {
        test_context.Session4KEncTrack(TrackType::kVideoHEVC);
      }
      break;
      case CmdMenu::CREATE_1080pAVC_AAC_AUD_SESSION_CMD: {
        test_context.Session1080pEncAndAudioAACTrack(TrackType::kVideoAVC);
      }
      break;
      case CmdMenu::CREATE_1080pHEVC_AAC_AUD_SESSION_CMD: {
        test_context.Session1080pEncAndAudioAACTrack(TrackType::kVideoHEVC);
      }
      break;
      case CmdMenu::CREATE_1080pENC_AVC_1080YUV_SESSION_CMD: {
        test_context.Session1080pEnc1080YUV(TrackType::kVideoAVC);
      }
      break;
      case CmdMenu::CREATE_4KENC_HEVC_1080YUV_SESSION_CMD: {
        test_context.Session4KEncAnd1080pYUVTracks(TrackType::kVideoHEVC);
      }
      break;
      case CmdMenu::CREATE_4KYUV_1080pENC_SESSION_CMD: {
        test_context.Session4KYUVAnd1080pEncTracks(TrackType::kVideoAVC);
      }
      break;
      case CmdMenu::CREATE_TWO_1080pENC_SESSION_CMD: {
        test_context.SessionTwo1080pEncTracks(TrackType::kVideoAVC);
      }
      break;
      case CmdMenu::CREATE_720pLPM_SESSION_CMD: {
        test_context.Session720pLPMTrack(TrackType::kVideoYUV);
      }
      break;
      case CmdMenu::CREATE_1080pENC_AVC_1080LPM_SESSION_CMD: {
        test_context.Session1080pEnc1080pLPMTracks(TrackType::kVideoAVC);
      }
      break;
      case CmdMenu::CREATE_PCM_AUD_SESSION_CMD: {
          test_context.CreateAudioPCMTrack();
      }
      break;
      case CmdMenu::CREATE_2PCM_AUD_SESSION_CMD: {
          test_context.CreateAudio2PCMTrack();
      }
      break;
      case CmdMenu::CREATE_SCO_AUD_SESSION_CMD: {
          test_context.CreateAudioSCOTrack();
      }
      break;
      case CmdMenu::CREATE_PCM_SCO_AUD_SESSION_CMD: {
          test_context.CreateAudioPCMSCOTrack();
      }
      break;
#ifdef ENABLE_A2DP_USECASE
      case CmdMenu::CREATE_A2DP_AUD_SESSION_CMD: {
          test_context.CreateAudioA2DPTrack();
      }
      break;
      case CmdMenu::CREATE_PCM_A2DP_AUD_SESSION_CMD: {
          test_context.CreateAudioPCMA2DPTrack();
      }
      break;
#endif
      case CmdMenu::CREATE_AAC_AUD_SESSION_CMD: {
          test_context.CreateAudioAACTrack();
      }
      break;
      case CmdMenu::CREATE_2AAC_AUD_SESSION_CMD: {
          test_context.CreateAudio2AACTrack();
      }
      break;
      case CmdMenu::CREATE_PCM_AAC_AUD_SESSION_CMD: {
          test_context.CreateAudioPCMAACTrack();
      }
      break;
      case CmdMenu::CREATE_AMR_AUD_SESSION_CMD: {
          test_context.CreateAudioAMRTrack();
      }
      break;
      case CmdMenu::CREATE_2AMR_AUD_SESSION_CMD: {
          test_context.CreateAudio2AMRTrack();
      }
      break;
      case CmdMenu::CREATE_PCM_AMR_AUD_SESSION_CMD: {
          test_context.CreateAudioPCMAMRTrack();
      }
      break;
      case CmdMenu::CREATE_G7ll_AUD_SESSION_CMD: {
          test_context.CreateAudioG711Track();
      }
      break;
      case CmdMenu::CREATE_2G7ll_AUD_SESSION_CMD: {
          test_context.CreateAudio2G711Track();
      }
      break;
      case CmdMenu::CREATE_PCM_G7ll_AUD_SESSION_CMD: {
          test_context.CreateAudioPCMG711Track();
      }
      break;
      case CmdMenu::CREATE_PCMFL_AUD_SESSION_CMD: {
          test_context.CreateAudioPCMFluenceTrack();
      }
      break;
      case CmdMenu::CREATE_RDI_SESSION_CMD: {
          test_context.SessionRDITrack();
      }
      break;
      case CmdMenu::CREATE_YUV_SESSION_DISPLAY_CMD: {
        test_context.Session1080pYUVTrackWithDisplay();
      }
      break;
      case CmdMenu::CREATE_YUV_SESSION_PREVIEW_CMD: {
        test_context.Session1080pYUVTrackWithPreview();
      }
      break;
      case CmdMenu::START_SESSION_CMD: {
        test_context.StartSession();
      }
      break;
      case CmdMenu::STOP_SESSION_CMD: {
        test_context.StopSession();
      }
      break;
      case CmdMenu::TAKE_SNAPSHOT_CMD: {
        test_context.TakeSnapshot();
      }
      break;
      case CmdMenu::SET_PARAM_CMD: {
        test_context.SetParams();
      }
      break;
      case CmdMenu::SET_DYNAMIC_CAMERA_PARAM_CMD: {
        test_context.SetDynamicCameraParam();
      }
      break;
      case CmdMenu::PAUSE_SESSION_CMD: {
        test_context.PauseSession();
      }
      break;
      case CmdMenu::RESUME_SESSION_CMD: {
        test_context.ResumeSession();
      }
      break;
      case CmdMenu::ENABLE_OVERLAY_CMD: {
        test_context.EnableOverlay();
      }
      break;
      case CmdMenu::DISABLE_OVERLAY_CMD: {
        test_context.DisableOverlay();
      }
      break;
      case CmdMenu::DELETE_SESSION_CMD: {
        test_context.DeleteSession();
      }
      break;
      case CmdMenu::NOISE_REDUCTION_CMD: {
        test_context.ToggleNR();
      }
      break;
      case CmdMenu::VIDEO_HDR_CMD: {
        test_context.ToggleVHDR();
      }
      break;
      case CmdMenu::IR_MODE_CMD: {
        test_context.ToggleIR();
      }
      break;
      case CmdMenu::CHOOSE_CAMERA_CMD: {
        test_context.ChooseCamera();
      }
      break;
      case CmdMenu::SET_ANTIBANDING_MODE_CMD: {
        test_context.SetAntibandingMode();
      }
      break;
      // TODO: To be exposed to end user once
      // 1080p@90FPS usecase is enabled through Menu
      case CmdMenu::BINNING_CORRECTION_CMD: {
        test_context.ToggleBinningCorrectionMode();
      }
      break;
      case CmdMenu::AWB_ROI_CMD: {
        test_context.HandleAWBROIRequest();
      }
      break;
      case CmdMenu::VIDEO_STABILZATION_CMD: {
        test_context.ToggleVideoStabilizationMode();
      }
      break;
      case CmdMenu::EXIT_CMD: {
        exit_test = true;
      }
      break;
      case CmdMenu::NEXT_CMD: {
        is_print_menu = true;
      }
      break;
      default:
        break;
    }
  }
  return 0;
}
