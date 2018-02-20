/*
 * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "RecorderCameraContext"

#include <algorithm>
#include <chrono>
#include <fcntl.h>
#include <math.h>
#include <sys/mman.h>
#include <json/json.h>
#ifdef ANDROID_O_OR_ABOVE
#include "common/utils/qmmf_common_utils.h"
#else
#include <QCamera3VendorTags.h>
#endif

#include "recorder/src/service/qmmf_camera_context.h"
#include "recorder/src/service/qmmf_recorder_utils.h"

namespace qmmf {

namespace recorder {

//Framerate after which we need to run in constrained mode.
float CameraContext::kConstrainedModeThreshold = 30.0f;
//Framerate at which batch requests are needed.

#ifdef _DRONE_
float CameraContext::kHFRBatchModeThreshold = 90.0f;
#else
float CameraContext::kHFRBatchModeThreshold = 120.0f;
#endif

CameraContext::CameraContext()
    : PostProcPlugin<CameraContext>(this),
      camera_id_(-1),
      streaming_request_id_(-1),
      last_frame_number_(-1),
      sequence_cnt_(1),
      last_snapshot_id_(-1),
      curr_snapshot_id_(-1),
      capture_cnt_(0),
      capture_done_(true),
      postproc_enable_(false),
      result_cb_(nullptr),
      error_cb_(nullptr),
      hfr_supported_(false),
      batch_size_(1),
      batch_stream_id_(-1),
      partial_metadata_required_(false),
      partial_result_count_(0),
      snapshot_param_{0, 0, 0, ImageFormat::kJPEG},
      snapshot_type_(SnapshotMode::kStill),
      new_snapshot_type_(SnapshotMode::kStill),
      postproc_frame_skip_(false) {
  camera_start_params_ = {};
}

CameraContext::~CameraContext() {

  QMMF_INFO("%s: Enter", __func__);
  if(camera_device_.get()) {
    camera_device_.clear();
    camera_device_ = nullptr;
  }
  //TODO: check all active ports
  QMMF_INFO("%s: Exit", __func__);
}

void CameraContext::InitSupportedFPS() {
  if (static_meta_.exists(ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)) {
    camera_metadata_entry_t entry = static_meta_.find(
        ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
    for (size_t i = 0 ; i < entry.count; i += 2) {
      if (entry.data.i32[i] == entry.data.i32[i+1]) {
        supported_fps_.push_back(entry.data.i32[i]);
      }
    }
  } else {
    QMMF_INFO("%s: Tag ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES "
        " doesn't exist in static metadata",  __func__);
  }
}

bool CameraContext::IsInputSupported() {
  if (static_meta_.exists(ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS)) {
    camera_metadata_entry entry = static_meta_.find(
        ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS);
    if (0 < entry.data.i32[0]) {
      return true;
    }
  }

  return false;
}

status_t CameraContext::CreateSnapshotStream(const ImageParam &param) {

  QMMF_INFO("%s: Enter", __func__);
  int32_t stream_id = -1;
  status_t ret = NO_ERROR;

  if (!snapshot_request_.streamIds.isEmpty()) {
    if (1 < snapshot_request_.streamIds.size()) {
      QMMF_ERROR("%s: Several non-zsl snapshot streams present!\n",
                 __func__);
      return BAD_VALUE;
    }
    QMMF_INFO("%s: Deleting Existing Snapshot Stream!!", __func__);
    ret = DeleteDeviceStream(snapshot_request_.streamIds[0], true);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Failed to delete non-zsl snapshot stream: %d\n",
                 __func__, ret);
      return ret;
    }
    snapshot_request_.streamIds.clear();

    PostProcDelete();
  }

  CameraStreamParameters stream_param{};

  ret = ValidateResolution(param.image_format, param.width, param.height);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: format(0x%x),width(%d):height(%d) Not supported!",
               __func__, param.image_format, param.width, param.height);
    return ret;
  }

  stream_param.format       = ImageToHalFormat(param.image_format);
  stream_param.width        = param.width;
  stream_param.height       = param.height;
  stream_param.grallocFlags = GRALLOC_USAGE_SW_WRITE_OFTEN |
                                GRALLOC_USAGE_SW_READ_OFTEN;
  stream_param.cb           = GetStreamCb(param);
  if (snapshot_type_ == SnapshotMode::kContinuous) {
    stream_param.bufferCount = PREVIEW_STREAM_BUFFER_COUNT;
  } else {
    stream_param.bufferCount = sequence_cnt_;
  }

  if (postproc_enable_) {

    // Stops active streaming to prevent multiple camera restarts
    for (auto port : active_ports_) {
      ret = port->Stop();
      assert(ret == NO_ERROR);
    }

    ret = PostProcCreatePipeAndUpdateStreams(stream_param, param.image_quality,
                            camera_start_params_.frame_rate, capture_plugins_);
    assert(ret == NO_ERROR);
  }

  QMMF_INFO("%s: W(%d) & H(%d) Fmt(0x%x)", __func__, stream_param.width,
            stream_param.height, stream_param.format);

  ret = CreateDeviceStream(stream_param, camera_start_params_.frame_rate,
                           &stream_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Failed creating snapshot stream: %d!",
               __func__, ret);
    return ret;
  }
  QMMF_INFO("%s Snapshot stream_id(%d)", __func__, stream_id);
  snapshot_param_ = param;
  snapshot_request_.streamIds.add(stream_id);

  if (postproc_enable_) {
    ret = PostProcStart(stream_id);
    assert(ret == NO_ERROR);

    // Resumes stopped streaming
    for (auto port : active_ports_) {
      ret = port->Start();
      assert(ret == NO_ERROR);
    }
  }
  return ret;
}

status_t CameraContext::DeleteSnapshotStream() {
  QMMF_INFO("%s: Enter", __func__);

  PostProcDelete();

  bool cache = streaming_request_id_ == -1;
  if (!snapshot_request_.streamIds.empty()) {
    auto ret = DeleteDeviceStream(snapshot_request_.streamIds[0], cache);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Failed to delete snapshot stream: %d",
          __func__, ret);
      return ret;
    }
    snapshot_request_.streamIds.clear();
  }
  QMMF_INFO("%s Exit", __func__);
  return NO_ERROR;
}

status_t CameraContext::OpenCamera(const uint32_t camera_id,
                                   const CameraStartParam &param,
                                   const ResultCb &cb,
                                   const ErrorCb &errcb) {

  uint32_t ret = NO_ERROR;
  bool match_camera_id = false;
  uint32_t num_camera = 0;

  //Setup Camera3DeviceClient callbacks.
  camera_callbacks_.errorCb = [&] (CameraErrorCode error_code,
      const CaptureResultExtras &extras) { CameraErrorCb(error_code, extras);};

  camera_callbacks_.idleCb = [&] () { CameraIdleCb(); };

  camera_callbacks_.peparedCb = [&] (int32_t id) { CameraPreparedCb(id); };

  camera_callbacks_.shutterCb = [&] (const CaptureResultExtras &extras,
      int64_t ts) { CameraShutterCb(extras, ts); };

  camera_callbacks_.resultCb = [&] (const CaptureResult &result)
      { CameraResultCb(result); };

  if (param.enable_partial_metadata) {
    partial_metadata_required_ = true;
  }

  camera_device_ = new Camera3DeviceClient(camera_callbacks_);
  if(!camera_device_.get()) {
    QMMF_ERROR("%s: Can't Instantiate Camera3DeviceClient", __func__);
    return NO_MEMORY;
  }
  camera_start_params_ = param;

  ret = camera_device_->Initialize();
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s Unable to Initialize Camera3DeviceClient %d",
               __func__, ret);
    goto FAIL;
  }

  num_camera = camera_device_->GetNumberOfCameras();
  for(uint32_t i = 0; i < num_camera; i++) {
    if(i == camera_id) {
      match_camera_id = true;
      break;
    }
  }
  if(!match_camera_id) {
    QMMF_ERROR("%s: Invalid Camera Id (%d)", __func__, camera_id);
    ret = BAD_VALUE;
    goto FAIL;
  }

  ret = camera_device_->OpenCamera(camera_id);
  assert(ret == NO_ERROR);
  camera_id_ = camera_id;

  ret = camera_device_->GetCameraInfo(camera_id, &static_meta_);
  assert(ret == NO_ERROR);
  InitSupportedFPS();
  assert(!supported_fps_.empty());
  InitHFRModes();

  {
    camera_metadata_entry partial_result_count =
        static_meta_.find(ANDROID_REQUEST_PARTIAL_RESULT_COUNT);
    if (partial_result_count.count > 0) {
      partial_result_count_ = partial_result_count.data.i32[0];
    }
  }

  ret = CreateCaptureRequest(snapshot_request_,
                             CAMERA3_TEMPLATE_STILL_CAPTURE);
  assert(ret == NO_ERROR);
  QMMF_INFO("%s: Non-zsl snapshot capture request created successfully!",
      __func__);

  if (param.zsl_mode) {

    if (!IsInputSupported()) {
      QMMF_ERROR("%s: Camera doesn't support input streams!",
                 __func__);
      ret = BAD_VALUE;
      goto FAIL;
    }

    //The snapshot stream is fixed and matches the ZSL stream
    //size. We cannot re-configure streams dynamically during
    //re-processing as this could have impact on the already
    //cached ZSL buffers and they may fail re-process.
    ImageParam image_param{};
    image_param.width = param.zsl_width;
    image_param.height = param.zsl_height;
    image_param.image_format = ImageFormat::kJPEG;

    ret = CreateSnapshotStream(image_param);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s Failed during snapshot stream setup",
                 __func__);
      return ret;
    }

    if (streaming_active_requests_.empty()) {
      streaming_active_requests_.emplace_back();
    }

    ret = CreateCaptureRequest(streaming_active_requests_[0],
                               CAMERA3_TEMPLATE_ZERO_SHUTTER_LAG);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Capture request for ZSL failed!", __func__);
      return ret;
    }

    int32_t fps_range[2];
    fps_range[0] = param.frame_rate;
    fps_range[1] = param.frame_rate;

    streaming_active_requests_[0].metadata.update(
        ANDROID_CONTROL_AE_TARGET_FPS_RANGE, fps_range, 2);

    CameraStreamParam zsl_param = {};
    zsl_param.cam_stream_dim.width  = param.zsl_width;
    zsl_param.cam_stream_dim.height = param.zsl_height;
    zsl_param.frame_rate            = param.frame_rate;
    zsl_param.low_power_mode        = false;
    zsl_port_ =
        std::make_shared<ZslPort>(const_cast<CameraStreamParam&>(zsl_param), 1,
        CameraPortType::kZSL, this);
    assert(zsl_port_.get() != nullptr);

    auto ret = zsl_port_->Init();
    if(ret != NO_ERROR) {
      QMMF_ERROR("%s: CameraPort is not initialized in ZSL mode!",
          __func__);
      zsl_port_ = nullptr;
      return BAD_VALUE;
    }

    active_ports_.push_back(zsl_port_);

    ret = zsl_port_->Start();
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: zsl port start failed!", __func__);
      return ret;
    }
    QMMF_INFO("%s: Number of Active ports=%d", __func__,
        active_ports_.size());
  }

  result_cb_ = cb;
  error_cb_ = errcb;

  return ret;
FAIL:
  camera_device_.clear();
  camera_device_= nullptr;
  return ret;
}

void CameraContext::InitHFRModes() {
  uint32_t width_offset = 0;
  uint32_t height_offset = 1;
  uint32_t min_fps_offset = 2;
  uint32_t max_fps_offset = 3;
  uint32_t batch_size_offset = 4;
  uint32_t hfr_size = 5;

  camera_metadata_entry meta_entry =
      static_meta_.find(ANDROID_REQUEST_AVAILABLE_CAPABILITIES);
  for (uint32_t i = 0; i < meta_entry.count; ++i) {
    uint8_t caps = meta_entry.data.u8[i];
    if (ANDROID_REQUEST_AVAILABLE_CAPABILITIES_CONSTRAINED_HIGH_SPEED_VIDEO ==
        caps) {
      hfr_supported_ = true;
      break;
    }
  }
  if (!hfr_supported_) {
    return;
  }

  meta_entry = static_meta_.find(
      ANDROID_CONTROL_AVAILABLE_HIGH_SPEED_VIDEO_CONFIGURATIONS);
  for (uint32_t i = 0; i < meta_entry.count; i += hfr_size) {
    uint32_t width = meta_entry.data.i32[i + width_offset];
    uint32_t height = meta_entry.data.i32[i + height_offset];
    uint32_t min_fps = meta_entry.data.i32[i + min_fps_offset];
    uint32_t max_fps = meta_entry.data.i32[i + max_fps_offset];
    uint32_t batch = meta_entry.data.i32[i + batch_size_offset];
    if (min_fps == max_fps) { //Only constant framerates are supported
      HFRMode_t mode = {width, height, batch, min_fps};
      hfr_batch_modes_list_.push_back(mode);
    }
  }
}

status_t CameraContext::CloseCamera(const uint32_t camera_id) {

  QMMF_INFO("%s: Enter", __func__);
  int32_t ret = NO_ERROR;
  assert(camera_id_ == camera_id);
  assert(camera_device_.get() != nullptr);

  if (camera_start_params_.zsl_mode && zsl_port_.get() != nullptr) {
    ZslPort* port = static_cast<ZslPort*>(zsl_port_.get());
    auto ret = port->PauseAndFlushZSLQueue();
    if (ret != NO_ERROR) {
      QMMF_WARN("%s: ZSL queue is not flashed!", __func__);
      // Even it is not flushed still give a try to Stop it.
    }
    ret = zsl_port_->Stop();
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s ZSL port stop failed!", __func__);
      return ret;
    }
  }
  DeleteSnapshotStream();

  if (streaming_request_id_ > 0) {
    QMMF_ERROR("%s: Streaming Request still running! delete all tracks "
    "before closing camera",  __func__);
    return INVALID_OPERATION;
  }

  ret = camera_device_->WaitUntilIdle();
  assert(ret == NO_ERROR);

  camera_device_.clear();
  camera_device_ = nullptr;

  QMMF_INFO("%s: Camera Closed Succussfully!", __func__);
  return ret;
}

int32_t CameraContext::ImageToHalFormat(ImageFormat image_format) {
      int32_t format;
  switch (image_format) {
    case ImageFormat::kJPEG:
      format = HAL_PIXEL_FORMAT_BLOB;
      break;
    case ImageFormat::kNV12:
      format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
      break;
    case ImageFormat::kBayerRDI10BIT:
      format = HAL_PIXEL_FORMAT_RAW10;
      break;
    case ImageFormat::kBayerRDI12BIT:
      format = HAL_PIXEL_FORMAT_RAW12;
      break;
    case ImageFormat::kBayerIdeal:
      // Not supported.
      QMMF_ERROR("%s ImageFormat::kBayerIdeal is Not supported!",
          __func__);
      return BAD_VALUE;
      break;
    default:
      format = HAL_PIXEL_FORMAT_BLOB;
      break;
  }
  return format;
}

bool CameraContext::IsPostProcNeeded(const ImageParam &param,
                                     const uint32_t sequence_cnt) {
  if (((sequence_cnt > 1) && (param.image_format == ImageFormat::kJPEG)) ||
      !capture_plugins_.empty()) {
    return true;
  } else {
    return false;
  }
}

void CameraContext::ReprocessCaptureCallback(StreamBuffer buffer) {

  QMMF_INFO("%s: StreamBuffer(0x%p) fd: %d stream_id: %d ts: %lld",
      __func__, buffer.handle, buffer.fd, buffer.stream_id,
      buffer.timestamp);

  if (GetNumConsumer() > 0) {
    NotifyBuffer(buffer);
  } else {
    // Return the buffer back to the camera.
    ReturnStreamBuffer(buffer);
  }
}

std::function<void(StreamBuffer buffer)>
    CameraContext::GetStreamCb(const ImageParam &param) {
  if (postproc_enable_) {
    return [=](StreamBuffer buffer) { ReprocessCaptureCallback (buffer); };
  } else {
    return [=](StreamBuffer buffer) { SnapshotCaptureCallback (buffer); };
  }
}

status_t CameraContext::WaitAecToConverge(const uint32_t timeout) {

  if (streaming_request_id_ == -1) {
    QMMF_DEBUG("%s: No active streams, skip wait!", __func__);
    return NO_ERROR;
  }

  std::unique_lock<std::mutex> lock(aec_lock_);
  std::chrono::nanoseconds wait_time(timeout);

  while ((aec_.state != ANDROID_CONTROL_AE_STATE_LOCKED) &&
         (aec_.state != ANDROID_CONTROL_AE_STATE_CONVERGED)) {
    if (aec_state_updated_.WaitFor(lock, wait_time) != 0) {
      QMMF_ERROR("%s Timed out on AEC converge Wait", __func__);
      return TIMED_OUT;
    }
  }
  return NO_ERROR;
}

status_t CameraContext::ValideteCaptureParams(const ImageParam &image_param) {
  if (snapshot_request_.metadata.isEmpty()) {
    QMMF_ERROR("%s Camera is not started Or it is started in zsl mode!!",
               __func__);
    return BAD_VALUE;
  }

  //Validate in params
  bool res_supported = false;
  camera_metadata_entry_t entry;
  CameraMetadata& meta = snapshot_request_.metadata;
  if (image_param.image_format == ImageFormat::kBayerRDI10BIT ||
      image_param.image_format == ImageFormat::kBayerRDI12BIT) {
    if (meta.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
      entry = meta.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES);
      for (uint32_t i = 0 ; i < entry.count; i += 2) {
        if (image_param.width == static_cast<uint32_t>(entry.data.i32[i+0]) &&
            image_param.height == static_cast<uint32_t>(entry.data.i32[i+1])) {
          res_supported = true;
          break;
        }
      }
    }
  } else if (image_param.image_format == ImageFormat::kNV12) {
    // Check Supported snapshot resolutions.
    if (meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
      entry = meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
      for (uint32_t i = 0 ; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == entry.data.i32[i]) {
          if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
              entry.data.i32[i+3]) {
            if (image_param.width == static_cast<uint32_t>(entry.data.i32[i+1])
                && image_param.height ==
                    static_cast<uint32_t>(entry.data.i32[i+2])) {
              res_supported = true;
              break;
            }
          }
        }
      }
    }
  } else if (image_param.image_format == ImageFormat::kJPEG) {
    if (meta.exists(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES)) {
      entry = meta.find(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES);
      for (uint32_t i = 0 ; i < entry.count; i += 2) {
        if(image_param.width == static_cast<uint32_t>(entry.data.i32[i+0]) &&
           image_param.height == static_cast<uint32_t>(entry.data.i32[i+1])) {
          res_supported = true;
          break;
        }
      }
    }
  }

  if (res_supported != true) {
    QMMF_ERROR("%s Unsuported Snapshot resolution %d x %d!",
               __func__, image_param.width, image_param.height);
    return BAD_VALUE;
  }

  return NO_ERROR;
}

status_t CameraContext::SetUpCapture(const ImageParam &param,
                                     const uint32_t num_images) {

  if (!camera_start_params_.zsl_mode) {
    bool reconfigure_needed = false;
    {
      std::unique_lock<std::mutex> lock(capture_lock_);

      bool new_postproc_enable = IsPostProcNeeded(param, num_images);
      reconfigure_needed = snapshot_request_.streamIds.empty() ||
                           (snapshot_param_.width != param.width) ||
                           (snapshot_param_.height != param.height) ||
                           (sequence_cnt_ != num_images) ||
                           (postproc_enable_ != new_postproc_enable) ||
                           (snapshot_type_ != new_snapshot_type_);
      snapshot_param_ = param;
      postproc_enable_ = new_postproc_enable;
      snapshot_type_ = new_snapshot_type_;

      if (snapshot_type_ == SnapshotMode::kContinuous) {
        sequence_cnt_ = 1;
      } else {
        sequence_cnt_ = num_images;
      }
    }

    if (reconfigure_needed) {

      auto ret = ValideteCaptureParams(param);
      if (NO_ERROR != ret) {
        QMMF_ERROR("%s Failed during snapshot validation", __func__);
        return ret;
      }

      QMMF_INFO("%s: Snapshot stream reconfigure required", __func__);
      ret = CreateSnapshotStream(param);
      if (NO_ERROR != ret) {
        QMMF_ERROR("%s Failed during snapshot re-configure", __func__);
        return ret;
      }
      // Wait AE to converge after reconfiguration if there are active streams.
      WaitAecToConverge(kWaitAecTimeout);
    }
  } else {
    if (ImageFormat::kJPEG != param.image_format) {
      QMMF_ERROR("%s ZSL capture supports only Jpeg as output!",
                 __func__);
      return BAD_VALUE;
    }

    if ((param.width != camera_start_params_.zsl_width) ||
        (param.height != camera_start_params_.zsl_height)) {
      QMMF_ERROR("%s ZSL stream size %dx%d doesn't match image size %dx%d!",
                 __func__, camera_start_params_.zsl_width,
                 camera_start_params_.zsl_height, param.width, param.height);
      return BAD_VALUE;
    }
  }
  return NO_ERROR;
}

status_t CameraContext::CaptureImage(const std::vector<CameraMetadata> &meta,
                                     const StreamSnapshotCb& cb) {

  QMMF_INFO("%s: Enter", __func__);
  int32_t ret = NO_ERROR;
  client_snapshot_cb_ = cb;
  capture_cnt_ = 0;
  if (!camera_start_params_.zsl_mode) {
    std::lock_guard<std::mutex> lock(device_access_lock_);
    int64_t last_frame_number;
    uint8_t jpeg_quality = snapshot_param_.image_quality;
    std::list<Camera3Request> requests;
    std::vector<CameraMetadata>::const_iterator it = meta.begin();
    for (uint32_t i = 0; i < sequence_cnt_; i++) {
      if (it != meta.end()) {
        snapshot_request_.metadata.clear();
        snapshot_request_.metadata.append(*it++);
      }
      snapshot_request_.metadata.update(ANDROID_JPEG_QUALITY, &jpeg_quality, 1);
      uint32_t active_streamid_count = 0;

      if (snapshot_type_ == SnapshotMode::kVideo ||
          snapshot_type_ == SnapshotMode::kContinuous) {
        if (streaming_active_requests_.size() == 1) {
          auto request = streaming_active_requests_[0];
          for (auto stream_id : request.streamIds) {
            snapshot_request_.streamIds.add(stream_id);
            active_streamid_count++;
          }
        } else {
          QMMF_INFO("%s: No other active video streams!", __func__);
        }
      }
      requests.push_back(snapshot_request_);
      snapshot_request_.streamIds.
        resize(snapshot_request_.streamIds.size() - active_streamid_count);
    }

    bool streaming = false;
    if (snapshot_type_ == SnapshotMode::kContinuous) {
      streaming = true;
    }

    {
      std::unique_lock<std::mutex> lock(capture_lock_);
      auto request_id = camera_device_->SubmitRequestList(requests,
                                                          streaming,
                                                          &last_frame_number);
      assert(request_id >= 0);
      snapshot_request_id_ = camera_device_->GetRequestIds();
      capture_done_ = false;
      if (snapshot_type_ == SnapshotMode::kContinuous) {
        last_snapshot_id_ = -1;
      } else {
        last_snapshot_id_ = last_frame_number;
      }
    }

    QMMF_INFO("%s: Request for non-zsl submitted successfully",
        __func__);
  } else {
    ret = CaptureZSLImage();
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: CaptureImage Failed in ZSL mode!", __func__);
      return ret;
    }
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

std::string CameraContext::GetSnapshotJsonConfig() {
  Json::Value root(Json::objectValue);

  for (size_t i = 0; i < thumbnails_.size(); i++) {
    root["thumbnail"][i]["width"] = thumbnails_[i].width;
    root["thumbnail"][i]["height"] = thumbnails_[i].height;
    root["thumbnail"][i]["quality"] = thumbnails_[i].quality;
  }

  root["frameskip"] = postproc_frame_skip_;
  root["jpeg quality"] = snapshot_param_.image_quality;

  Json::FastWriter fastWriter;
  auto config = fastWriter.write(root);

  QMMF_INFO("%s: Snapshot configuration: %s", __func__, config.c_str());

  return config;
}

status_t CameraContext::ConfigImageCapture(const ImageConfigParam &config) {
  capture_plugins_.clear();

  if (config.Exists(QMMF_POSTPROCESS_PLUGIN)) {
    for (size_t i = 0; i < config.EntryCount(QMMF_POSTPROCESS_PLUGIN); ++i) {
      PostprocPlugin plugin;
      config.Fetch(QMMF_POSTPROCESS_PLUGIN, plugin, i);
      if (plugin.uid == 0) {
        QMMF_ERROR("%s: Invalid plugin parameters!", __func__);
        return BAD_VALUE;
      }
      capture_plugins_.push_back(plugin.uid);
    }
  }

  if (config.Exists(QMMF_SNAPSHOT_TYPE)) {
    SnapshotType type;
    config.Fetch(QMMF_SNAPSHOT_TYPE, type);

    std::unique_lock<std::mutex> lock(capture_lock_);
    if (capture_done_ == false && snapshot_type_ != type.type) {
      QMMF_ERROR("%s: %d capture is ongoing", __func__, snapshot_type_);
      return INVALID_OPERATION;
    }
    new_snapshot_type_ = type.type;
  }

  if (config.Exists(QMMF_IMAGE_THUMBNAIL)) {
    thumbnails_.clear();
    for (size_t i = 0; i < config.EntryCount(QMMF_IMAGE_THUMBNAIL); i++) {
      thumbnails_.push_back(ImageThumbnail());
      config.Fetch(QMMF_IMAGE_THUMBNAIL, thumbnails_[i], i);
    }
  }

  if (config.Exists(QMMF_POSTPROCESS_FRAME_SKIP)) {
    PostprocFrameSkip frame_skip;
    config.Fetch(QMMF_POSTPROCESS_FRAME_SKIP, frame_skip, 0);
    postproc_frame_skip_ = frame_skip.frame_skip > 0 ? true : false;
  }

  return NO_ERROR;
}

status_t CameraContext::CancelCaptureImage() {

  QMMF_INFO("%s: Enter", __func__);

  if (!snapshot_request_.streamIds.empty() && !snapshot_request_id_.empty()) {
    if (snapshot_type_ == SnapshotMode::kContinuous) {
      // Update request check all active ports and  prepare request only
      // for active streams which uses port. This discards snapshot streaming.
      UpdateRequest(true);
      last_snapshot_id_ = last_frame_number_;
    }

    if (postproc_enable_) {
      auto ret = postproc_pipe_->Abort();
      if (ret != NO_ERROR) {
        QMMF_ERROR("%s: Post process abort failed", __func__);
        return ret;
      }
    }

    {
      std::unique_lock<std::mutex> lock(capture_lock_);
      std::chrono::nanoseconds wait_time(kWaitPendingFramesTimeout);
      cancel_capture_ = true;

      while (capture_done_ == false) {
        // Capture is not complete yet, wait till pending buffers are returned.
        if (postproc_enable_) {
          QMMF_INFO("%s: Cancel request is waiting for frame %lld",
              __func__, last_snapshot_id_);
        } else {
          QMMF_INFO("%s: Cancel request with pending buffer(%d)!",
              __func__, sequence_cnt_ - capture_cnt_);
        }

        auto ret = capture_signal_.WaitFor(lock, wait_time);
        if (ret != 0) {
          QMMF_ERROR("%s: Timed out on Wait", __func__);
          return TIMED_OUT;
        }
      }
      cancel_capture_ = false;
      QMMF_INFO("%s: Capture is complete!", __func__);
    }

    DeleteSnapshotStream();
  }

  QMMF_INFO("%s: Exit", __func__);
  return NO_ERROR;
}

void CameraContext::RestoreBatchStreamId(CameraPort* port) {
  if (!port) {
    QMMF_ERROR("%s: Invalid port", __func__);
    return;
  }

  if (batch_stream_id_ == port->GetCameraStreamId()) {
    batch_stream_id_ = -1;
    batch_size_ = 1;
  }
}

void CameraContext::StoreBatchStreamId(std::shared_ptr<CameraPort>& port) {
  assert(port.get() != nullptr);
  if (port->GetPortBatchSize() > 1) {
    if (batch_stream_id_ > -1) {
      QMMF_WARN("%s:The Batch stream is already configuried", __func__);
    } else {
      batch_stream_id_ = port->GetCameraStreamId();
    }
  }
}

status_t CameraContext::GetBatchSize(const CameraStreamParam& param,
                                     uint32_t& batch_size) {

  /* only one batch stream is supported */
  if (batch_size_ > 1) {
    /* set batch size to default */
    batch_size = 1;
    return NO_ERROR;
  }

  if ((kConstrainedModeThreshold < param.frame_rate) && (!hfr_supported_)) {
    QMMF_ERROR("%s: Stream tries to enable HFR which is not supported!",
               __func__);
    return BAD_VALUE;
  }

  if ((kConstrainedModeThreshold < param.frame_rate) &&
      (camera_start_params_.zsl_mode)) {
    QMMF_ERROR("%s: HFR and ZSL are mutually exclusive!",
               __func__);
    return BAD_VALUE;
  }

  size_t batch = 1;
  if (kHFRBatchModeThreshold <= param.frame_rate) {
    bool supported = false;
    for (size_t i = 0; i < hfr_batch_modes_list_.size(); i++) {
      if ((param.cam_stream_dim.width == hfr_batch_modes_list_[i].width) &&
          (param.cam_stream_dim.height == hfr_batch_modes_list_[i].height) &&
          fabs(param.frame_rate - hfr_batch_modes_list_[i].framerate) < 0.1f) {
        batch = hfr_batch_modes_list_[i].batch_size;
        supported = true;
        break;
      }
    }

    if (!supported) {
      QMMF_ERROR("%s: HFR stream with size %dx%d fps: %5.2f is not supported!",
                 __func__, param.cam_stream_dim.width,
                 param.cam_stream_dim.height,
                 param.frame_rate);
      return BAD_VALUE;
    }
  }

  batch_size = batch;
  batch_size_ = batch_size;

  return NO_ERROR;
}

status_t CameraContext::CreateStream(const CameraStreamParam& param,
                                     const VideoExtraParam& extra_param) {

  QMMF_VERBOSE("%s: Enter", __func__);
  // 1. Check if streaming request already is going on, if yes then cancel it
  //    and reconfigure it with adding new request.
  // 2. Check for available port where consumer can be attached, if not then
  //    Create new one.
  // 3. Create camera adaptor stream.
  // 4. Create port and link it with adaptor stream.
  // 5. Create producer interface in port and link consumer.

  assert(camera_device_.get() != nullptr);
  assert(param.id != 0);

  size_t batch;
  if (NO_ERROR != GetBatchSize(param, batch)) {
    return BAD_VALUE;
  }

  std::shared_ptr<CameraPort> port =
      std::make_shared<CameraPort>(param, batch, CameraPortType::kVideo, this);
  assert(port.get() != nullptr);

  if (extra_param.Exists(QMMF_POSTPROCESS_PLUGIN)) {
    size_t entry_count = extra_param.EntryCount(QMMF_POSTPROCESS_PLUGIN);
    std::vector<uint32_t> plugin_uids;

    for (size_t i = 0; i < entry_count; ++i) {
      PostprocPlugin plugin;
      extra_param.Fetch(QMMF_POSTPROCESS_PLUGIN, plugin, i);
      if (plugin.uid == 0) {
        QMMF_ERROR("%s: Invalid plugin parameters!", __func__);
        return BAD_VALUE;
      }
      plugin_uids.push_back(plugin.uid);
    }
    video_plugins_.emplace(port->GetPortId(), plugin_uids);
  }

  auto ret = port->Init();
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s: CameraPort Can't be Created!", __func__);
    return BAD_VALUE;
  }

  StoreBatchStreamId(port);

  // Create global streaming capture request, this capture request would be
  // Common to all video/preview and zsl snapshot stream. non zsl snapshot
  // will have separate capture request.
  if (streaming_active_requests_.empty()) {
    streaming_active_requests_.emplace_back();
    ret = CreateCaptureRequest(streaming_active_requests_[0],
                               CAMERA3_TEMPLATE_VIDEO_RECORD);
    assert(ret == NO_ERROR);
    QMMF_INFO("%s: Global Streaming Capture request created successfully!",
        __func__);
  }

  // Add port to list of active ports.
  active_ports_.push_back(port);

  QMMF_INFO("%s: Number of Active ports=%d", __func__,
      active_ports_.size());

  QMMF_VERBOSE("%s: Exit", __func__);
  return ret;
}

status_t CameraContext::DeleteStream(const uint32_t track_id) {

  auto port = GetPort(track_id);
  if (!port) {
    QMMF_ERROR("%s: Invalid track_id(%x)", __func__, track_id);
    return BAD_VALUE;
  }
  assert(port != nullptr);
  if (port->GetNumConsumers() > 0) {
    // Port still being used by another consumer, eventually this port would be
    // deleted once consumers count would become zero.
    return NO_ERROR;
  }

  // Remove the cached plugins for this track
  video_plugins_.erase(track_id);

  auto ret = port->DeInit();
  assert(ret == NO_ERROR);

  RestoreBatchStreamId(port);

  DeletePort(track_id);

  QMMF_INFO("%s: Camera Port for track_id(%x) deleted", __func__,
      track_id);

  return ret;
}

status_t CameraContext::AddConsumer(const uint32_t& track_id,
                                    sp<IBufferConsumer>& consumer) {

  auto port = GetPort(track_id);
  if (!port) {
    QMMF_ERROR("%s: Invalid track_id(%x)", __func__, track_id);
    return BAD_VALUE;
  }
  assert(port != nullptr);

  auto ret = port->AddConsumer(consumer);
  assert(ret == NO_ERROR);
  QMMF_INFO("%s: Consumer(%p) added to track_id(%d)", __func__,
      consumer.get(), track_id);
  return NO_ERROR;
}

status_t CameraContext::RemoveConsumer(const uint32_t& track_id,
                                       sp<IBufferConsumer>& consumer) {

  auto port = GetPort(track_id);
  if (!port) {
    QMMF_ERROR("%s: Invalid track_id(%x)", __func__, track_id);
    return BAD_VALUE;
  }
  assert(port != nullptr);

  auto ret = port->RemoveConsumer(consumer);
  assert(ret == NO_ERROR);
  return NO_ERROR;
}

status_t CameraContext::StartStream(const uint32_t track_id) {

  auto port = GetPort(track_id);
  if (!port) {
    QMMF_ERROR("%s: Invalid track_id(%x)", __func__, track_id);
    return BAD_VALUE;
  }
  assert(port != nullptr);

  auto ret = port->Start();
  assert(ret == NO_ERROR);
  QMMF_INFO("%s: track_id(%d) started on port(0x%p)", __func__,
      track_id, port);
  return NO_ERROR;
}

status_t CameraContext::StopStream(const uint32_t track_id) {

  auto port = GetPort(track_id);
  if (!port) {
    QMMF_ERROR("%s: Invalid track_id(%x)", __func__, track_id);
    return BAD_VALUE;
  }
  assert(port != nullptr);

  auto ret = port->Stop();
  assert(ret == NO_ERROR);
  return NO_ERROR;
}

status_t CameraContext::SetCameraParam(const CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter", __func__);

  std::lock_guard<std::mutex> lock(device_access_lock_);
  if ((!streaming_active_requests_.empty()) &&
      (!streaming_active_requests_[0].metadata.isEmpty())) {
    int64_t last_frame_mumber;
    std::list<Camera3Request> request_list;
    for (size_t i = 0; i < streaming_active_requests_.size(); i++) {
      Camera3Request &req = streaming_active_requests_[i];
      req.metadata.clear();
      req.metadata.append(meta);
      request_list.push_back(req);
    }
    // Submit request with updated camera meta data only if streaming is
    // started, if not then just update default meta data and leave it to
    // startSession -> startStream to submit request.
    if (streaming_request_id_ >= 0) {
      auto ret = camera_device_->SubmitRequestList(request_list, true,
                                                   &last_frame_mumber);
      assert(ret >= 0);
      streaming_request_id_ = ret;
    }
  } else {
    QMMF_ERROR("%s: No active requests present!\n", __func__);
    return NO_INIT;
  }
  QMMF_DEBUG("%s: Exit", __func__);
  return NO_ERROR;
}

status_t CameraContext::GetCameraParam(CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter", __func__);
  meta.clear();
  int32_t ret = NO_ERROR;
  if ((!streaming_active_requests_.empty()) &&
      (!streaming_active_requests_[0].metadata.isEmpty())) {
    meta.append(streaming_active_requests_[0].metadata);
  } else {
    QMMF_ERROR("%s No active requests present!\n", __func__);
    return NO_INIT;
  }
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t CameraContext::GetDefaultCaptureParam(CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter", __func__);
  auto ret = NO_ERROR;
  if (!snapshot_request_.metadata.isEmpty()) {
    meta.clear();
    // Append default snapshot meta data.
    meta.append(snapshot_request_.metadata);
  } else {
    QMMF_WARN("%s Camera is not started Or it is started in zsl mode!\n",
        __func__);
    ret = NO_INIT;
  }
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t CameraContext::ReturnImageCaptureBuffer(const uint32_t camera_id,
                                                 const int32_t buffer_id) {

  QMMF_DEBUG("%s: Enter", __func__);
  if (snapshot_buffer_list_.find(buffer_id) == snapshot_buffer_list_.end()) {
    QMMF_ERROR("%s: buffer_id(%u) is not valid!!", __func__, buffer_id);
    return BAD_VALUE;
  }

  StreamBuffer buffer = snapshot_buffer_list_.find(buffer_id)->second;
  assert(buffer.fd == buffer_id);


  if (snapshot_buffer_stream_list_.find(buffer_id) ==
      snapshot_buffer_stream_list_.end()) {
    QMMF_ERROR("%s: buffer_id(%u) is not valid!!", __func__, buffer_id);
    return BAD_VALUE;
  }
  int32_t stream_id = snapshot_buffer_stream_list_.find(buffer_id)->second;

  QMMF_DEBUG("%s: stream_id(%d):stream_buffer(0x%p):ion_fd(%d)"
      " returned back!",  __func__, stream_id, buffer.handle, buffer_id);

  status_t ret = NO_ERROR;
  if (postproc_enable_) {
    postproc_pipe_->PipeNotifyBufferReturn(buffer);
  } else {
    ret = camera_device_->ReturnStreamBuffer(buffer);
  }

  QMMF_DEBUG("%s: ret %d", __func__, ret);
  assert(ret == NO_ERROR);

  snapshot_buffer_list_.erase(buffer_id);
  snapshot_buffer_stream_list_.erase(buffer_id);

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

CameraStartParam& CameraContext::GetCameraStartParam() {

  return camera_start_params_;
}

std::vector<int32_t>& CameraContext::GetSupportedFps() {

  return supported_fps_;
}

status_t CameraContext::CreateDeviceStream(CameraStreamParameters& params,
                                           uint32_t frame_rate,
                                           int32_t* stream_id) {

  std::lock_guard<std::mutex> lock(device_access_lock_);
  QMMF_VERBOSE("%s: Enter", __func__);

  int32_t ret = NO_ERROR;
  assert(camera_device_.get() != nullptr);

  if (camera_start_params_.zsl_mode && zsl_port_.get() != nullptr) {
    ZslPort* zsl_port = static_cast<ZslPort*>(zsl_port_.get());
    if (zsl_port->IsRunning()) {
      QMMF_INFO("%s: ZSL is running, pause and flush queue!",
          __func__);
      ret = zsl_port->PauseAndFlushZSLQueue();
      if (ret != NO_ERROR) {
        QMMF_ERROR("%s: zsl queue is not flashed!", __func__);
        return ret;
      }
    }
  }

  // Configure is required only once, if streaming request is already submitted
  // then BeginConfigure is not required to be called, stream can be created
  // without calling it.
  if (streaming_request_id_ < 0) {
    ret = camera_device_->BeginConfigure();
    assert(ret == NO_ERROR);
  }

  int32_t id;
  id = camera_device_->CreateStream(params);
  if (id < 0) {
    QMMF_INFO("%s: createStream failed!!", __func__);
    return BAD_VALUE;
  }
  *stream_id = id;

  // At this point stream is created but it is not added to request, it will be
  // added once corresponding port will get the start cmd from it's consumer.
  if (streaming_request_id_ < 0) {
    bool is_constrained_mode = false;
    if (hfr_supported_) {
      for (auto iter : active_ports_) {
        if (kConstrainedModeThreshold < iter->GetPortFramerate()) {
          is_constrained_mode = true;
          break;
        }
      }
      if (!is_constrained_mode && (kConstrainedModeThreshold < frame_rate)) {
        is_constrained_mode = true;
      }
    }
    QMMF_VERBOSE("%s: is_constrained_mode(%d)", __func__,
        is_constrained_mode);
    bool is_raw_only = false;
    if (params.format == HAL_PIXEL_FORMAT_RAW10) {
      is_raw_only = true;
    }

    ret = camera_device_->EndConfigure(is_constrained_mode, is_raw_only,
                                       batch_size_, params.is_pp_enabled);
    assert(ret == NO_ERROR);
  }

  if (camera_start_params_.zsl_mode && zsl_port_.get() != nullptr) {
    ZslPort* zsl_port = static_cast<ZslPort*>(zsl_port_.get());
    QMMF_INFO("%s: Resume ZSL!", __func__);
    zsl_port->ResumeZSL();
  }

  QMMF_VERBOSE("%s: Exit", __func__);
  return ret;
}

status_t CameraContext::CreateDeviceInputStream(
    CameraInputStreamParameters& params, int32_t* stream_id) {
  std::lock_guard<std::mutex> lock(device_access_lock_);
  QMMF_INFO("%s: Enter", __func__);

  int32_t ret = NO_ERROR;
  assert(camera_device_.get() != nullptr);

  // Configure is required only once, if streaming request is already submitted
  // then BeginConfigure is not required to be called, stream can be created
  // without calling it.
  if (streaming_request_id_ < 0) {
    ret = camera_device_->BeginConfigure();
    assert(ret == NO_ERROR);
  }

  int32_t id;
  id = camera_device_->CreateInputStream(params);
  if (id < 0) {
    QMMF_INFO("%s: createStream failed!!", __func__);
    return BAD_VALUE;
  }
  *stream_id = id;

  // At this point stream is created but it is not added to request, it will be
  // added once corresponding port will get the start cmd from it's consumer.
  if (streaming_request_id_ < 0) {
    ret = camera_device_->EndConfigure();
    assert(ret == NO_ERROR);
  }

  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t CameraContext::DeleteDeviceStream(int32_t stream_id, bool cache) {

  QMMF_VERBOSE("%s: Enter", __func__);
  status_t ret = NO_ERROR;
  int64_t last_frame_mumber;
  assert(camera_device_.get() != nullptr);

  bool resume_streaming = false;
  if (camera_start_params_.zsl_mode && zsl_port_.get() != nullptr
      && (0 <= streaming_request_id_)) {

    ZslPort* zsl_port = static_cast<ZslPort*>(zsl_port_.get());
    if (zsl_port->IsRunning()) {
      QMMF_INFO("%s: ZSL is running, pause and flush queue!",
        __func__);
      auto ret = zsl_port->PauseAndFlushZSLQueue();
      if (ret != NO_ERROR) {
        QMMF_ERROR("%s: zsl queue is not flashed!", __func__);
        return ret;
      }
      QMMF_INFO("%s: Cancelling Request!!", __func__);
      ret = CancelRequest();
      if (NO_ERROR != ret) {
        QMMF_ERROR("%s Cancel request failed:%d", __func__, ret);
        return ret;
      }
      resume_streaming = true;
    }
  }

  std::lock_guard<std::mutex> lock(device_access_lock_);

  if (camera_start_params_.zsl_mode && zsl_port_.get() != nullptr) {
    ret = camera_device_->BeginConfigure();
    assert(ret == NO_ERROR);
  }

  ret = camera_device_->DeleteStream(stream_id, cache);
  assert(ret == NO_ERROR);
  QMMF_INFO("%s: Camera Device Stream(%d) deleted successfully!",
      __func__, stream_id);

  if (camera_start_params_.zsl_mode && zsl_port_.get() != nullptr) {
    ret = camera_device_->EndConfigure();
    assert(ret == NO_ERROR);

    ZslPort* zsl_port = static_cast<ZslPort*>(zsl_port_.get());
    zsl_port->ResumeZSL();

    if (resume_streaming) {
      ret = camera_device_->SubmitRequest(streaming_active_requests_[0], true,
                                          &last_frame_mumber);
      assert(ret >= 0);
      streaming_request_id_ = ret;
      ret = NO_ERROR;
    }
  }

  QMMF_VERBOSE("%s: Exit", __func__);
  return ret;
}

status_t CameraContext::CreateCaptureRequest(Camera3Request& request,
                                             camera3_request_template_t
                                             template_type) {

  std::lock_guard<std::mutex> lock(device_access_lock_);

  auto ret = camera_device_->CreateDefaultRequest(template_type,
      &request.metadata);
  assert(ret == NO_ERROR);

  // Append static meta data.
  request.metadata.append(static_meta_);
  return ret;
}

CameraMetadata CameraContext::GetCameraStaticMeta() {
  return static_meta_;
}

status_t CameraContext::UpdateRequest(bool is_streaming) {

  QMMF_DEBUG("%s: Enter", __func__);
  float max_fps = 0;
  std::set<int32_t> stream_ids;
  std::set<int32_t> removed_streams;

  //Get all camera stream ids from all active ports which are ready to start.
  size_t size = active_ports_.size();
  QMMF_INFO("%s: Number of active_ports(%d)", __func__, size);

  for (size_t i = 0; i < size; i++) {
    std::shared_ptr<CameraPort> port = active_ports_[i];
    assert(port != nullptr);

    int32_t cam_stream_id = port->GetCameraStreamId();
    size_t batch_size = port->GetPortBatchSize();
    QMMF_INFO("%s: cam_stream_id(%d)", __func__, cam_stream_id);
    if (port->getPortState() == PortState::PORT_READYTOSTART) {

      QMMF_INFO("%s: CameraPort(0x%p):camera_stream_id(%d) is ready to"
          " start!",  __func__, port.get(), cam_stream_id);
      if (max_fps < port->GetPortFramerate()) {
        max_fps = port->GetPortFramerate();
      }
      if (batch_size > streaming_active_requests_.size()) {
        streaming_active_requests_.resize(batch_size);
      }
      for (size_t i = 0; i < batch_size; i++) {
        streaming_active_requests_[i].streamIds.add(cam_stream_id);
        if ((1 < i) && (streaming_active_requests_[i].metadata.isEmpty())) {
          assert(!streaming_active_requests_[0].metadata.isEmpty());
          streaming_active_requests_[i].metadata.append(
              streaming_active_requests_[0].metadata);
        }
      }
      stream_ids.emplace(cam_stream_id);
    } else if (port->getPortState() == PortState::PORT_READYTOSTOP) {

      QMMF_INFO("%s: CameraPort(0x%p):camera_stream_id(%d) is stopped ",
          __func__, port.get(), cam_stream_id);
      // Check if camera stream is already part of request, if yes then remove
      // it from request list. if not then it means stream is created but its
      // corresponding port is not started yet.
      QMMF_INFO("%s: streaming_active_requests_.size(%d)", __func__,
          streaming_active_requests_.size());
      for (size_t j = 0; j < streaming_active_requests_.size(); j++) {
        Camera3Request &req = streaming_active_requests_[j];
        bool match = false;
        size_t idx = -1;
        QMMF_INFO("%s: req.streamIds.size(%d)", __func__,
            req.streamIds.size());
        for (size_t i = 0; i < req.streamIds.size(); i++) {
          if (cam_stream_id == req.streamIds[i]) {
            match = true;
            idx = i;
            break;
          }
        }
        if(match == true) {
          req.streamIds.removeAt(idx);
          QMMF_INFO("%s: cam_stream_id(%d) removed from Request!",
                      __func__, cam_stream_id);
          removed_streams.emplace(cam_stream_id);
          stream_ids.emplace(cam_stream_id);
          QMMF_INFO("%s: removed_streams.size(%d)", __func__,
              removed_streams.size());
        }
      }
    } else if (port->getPortState() == PortState::PORT_STARTED) {
      if (max_fps < port->GetPortFramerate()) {
        max_fps = port->GetPortFramerate();
      }
      stream_ids.emplace(cam_stream_id);
    }
  }

  bool stale_batches_present = false;
  ssize_t stale_idx = -1;
  size_t stale_count = 0;
  //Check for any stale batch requests and remove if present
  for (size_t i = 1; i < streaming_active_requests_.size(); i++) {
    if(streaming_active_requests_[i].streamIds.isEmpty()) {
      if (!stale_batches_present) {
        stale_batches_present = true;
        stale_idx = i;
      }
      stale_count++;
    } else {
      assert(!stale_batches_present);
    }
  }

  if (stale_batches_present) {
    streaming_active_requests_.erase(streaming_active_requests_.begin()
        + stale_idx,
        streaming_active_requests_.begin()
        + stale_idx + stale_count);
  }
  size = streaming_active_requests_[0].streamIds.size();
  QMMF_INFO("%s: Number of streams(%d) to start", __func__, size);
  if (size == 0) {
    QMMF_INFO("%s:Cancelling the request, no pending stream!", __func__);
    return CancelRequest();
  }

  {
    std::lock_guard<std::mutex> lock(device_access_lock_);
    if (0 < max_fps) {
      int32_t fpsRange[2];
      fpsRange[0] = ceil(max_fps);
      fpsRange[1] = ceil(max_fps);

      for (size_t i = 0; i < streaming_active_requests_.size(); ++i) {
        streaming_active_requests_[i].metadata.update(
            ANDROID_CONTROL_AE_TARGET_FPS_RANGE, fpsRange, 2);
      }
    }
    std::list<Camera3Request> request_list;
    for (ssize_t i = (streaming_active_requests_.size() - 1); i >= 0; --i) {
      request_list.push_back(streaming_active_requests_[i]);
      assert(!streaming_active_requests_[i].metadata.isEmpty());
    }
    std::unique_lock<std::mutex> pending_frames_lock(pending_frames_lock_);
    auto req_id = camera_device_->SubmitRequestList(request_list, is_streaming,
                                                    &last_frame_number_);
    QMMF_INFO("%s: last_frame_number=%lld", __func__,
        last_frame_number_);
    assert(req_id >= 0);
    streaming_request_id_ = req_id;

    for (auto const& stream_id : stream_ids) {
      // Update the last submitted frame number for each stream id.
      if (last_frame_number_map_.count(stream_id) != 0 &&
          last_frame_number_ != NO_IN_FLIGHT_REPEATING_FRAMES) {
        // Request was submitted successfully since previous call, update.
        last_frame_number_map_[stream_id] = last_frame_number_;

      } else if (last_frame_number_map_.count(stream_id) == 0) {
        // Newly initiated stream, request hasn't yet been submitted to HAL.
        last_frame_number_map_[stream_id] = NO_IN_FLIGHT_REPEATING_FRAMES;
      }
      QMMF_INFO("%s: last_frame_number_map_[%d]=%lld", __func__,
          stream_id, last_frame_number_map_[stream_id]);

      // Update the removed stream ids that need to wait for frames to return.
      if (removed_streams.count(stream_id) != 0 &&
          last_frame_number_map_[stream_id] != NO_IN_FLIGHT_REPEATING_FRAMES) {
        removed_stream_ids_.emplace(stream_id);
      }
    }

    std::chrono::nanoseconds wait_time(kWaitPendingFramesTimeout);
    while (!removed_stream_ids_.empty()) {
      auto ret = pending_frames_.WaitFor(pending_frames_lock, wait_time);
      if (ret != 0) {
        QMMF_WARN("%s: Waiting for submitted frames to return, timed out!",
            __func__);
        break;
      }
    }
  }
  QMMF_INFO("%s: SubmitRequest for Num streams(%d) is successfull"
      " request_id(%d) batches: %d",  __func__, size, streaming_request_id_,
      streaming_active_requests_.size());

  return NO_ERROR;
}

int32_t CameraContext::SubmitRequest(Camera3Request request,
                                     bool is_streaming,
                                     int64_t *lastFrameNumber) {
  std::lock_guard<std::mutex> lock(device_access_lock_);

  int32_t ret = NO_ERROR;
  ret = camera_device_->SubmitRequest(request, is_streaming,
                                      lastFrameNumber);
  assert(ret >= 0);
  return ret;
}

status_t CameraContext::CancelRequest() {

  std::lock_guard<std::mutex> lock(device_access_lock_);

  int64_t last_frame_mumber;
  assert(streaming_request_id_ >= 0);

  QMMF_INFO("%s: Issuing CancelRequest!", __func__);
  auto ret = camera_device_->CancelRequest(streaming_request_id_,
                                           &last_frame_mumber);
  assert(ret == NO_ERROR);
  QMMF_INFO("%s: last_frame_mumber(%lld) after CancelRequest", __func__,
      last_frame_mumber);

  ret = camera_device_->WaitUntilIdle();
  assert(ret == NO_ERROR);
  {
    std::lock_guard<std::mutex> aec_lock(aec_lock_);
    aec_.Reset();
  }
  streaming_request_id_ = -1;
  QMMF_INFO("%s: Request cancelled last frame number: %lld\n",
      __func__, last_frame_mumber);
  return ret;
}

status_t CameraContext::ReturnStreamBuffer(StreamBuffer buffer) {
  QMMF_DEBUG("%s: camera_stream_id: %d, buffer: 0x%p ts: %lld\n",
      __func__, buffer.stream_id, buffer.handle, buffer.timestamp);

  auto ret = camera_device_->ReturnStreamBuffer(buffer);
  assert(ret == NO_ERROR);

  std::lock_guard<std::mutex> lock(pending_frames_lock_);
  if (removed_stream_ids_.count(buffer.stream_id) != 0) {
    QMMF_DEBUG("%s: removed_stream_ids_.size(%d)", __func__,
        removed_stream_ids_.size());
    QMMF_DEBUG("%s: last_frame_number_map_[%d]=%lld, "
        "buffer.frame_number: %u", __func__,
        buffer.stream_id, last_frame_number_map_[buffer.stream_id],
        buffer.frame_number);
    if (last_frame_number_map_[buffer.stream_id] == buffer.frame_number) {
      removed_stream_ids_.erase(buffer.stream_id);
      last_frame_number_map_.erase(buffer.stream_id);
      pending_frames_.Signal();
    }
  }

  return ret;
}

void CameraContext::SnapshotCaptureCallback(StreamBuffer buffer) {

  QMMF_DEBUG("%s Enter ", __func__);

  QMMF_DEBUG("%s format(0x%x):num_planes(%d) ", __func__,
      buffer.info.format, buffer.info.num_planes);
  for (uint32_t i = 0; i < buffer.info.num_planes; ++i) {
    QMMF_DEBUG("%s plane_info[%d].stride=%d", __func__, i,
        buffer.info.plane_info[i].stride);
    QMMF_DEBUG("%s plane_info[%d].scanline=%d", __func__, i,
        buffer.info.plane_info[i].scanline);
    QMMF_DEBUG("%s plane_info[%d].width=%d", __func__, i,
        buffer.info.plane_info[i].width);
    QMMF_DEBUG("%s plane_info[%d].height=%d", __func__, i,
        buffer.info.plane_info[i].height);
  }
  QMMF_DEBUG("%s fd(0x%x):size(%d) ", __func__, buffer.fd, buffer.size);

  uint32_t frame_number = 0;
  {
    std::lock_guard<std::mutex> lock(capture_lock_);
    frame_number = capture_cnt_;
    capture_cnt_++;

    // return buffer if cancel capture
    if (cancel_capture_) {
      if (postproc_enable_) {
        postproc_pipe_->PipeNotifyBufferReturn(buffer);
      } else {
        auto ret = camera_device_->ReturnStreamBuffer(buffer);
        assert(ret == NO_ERROR);
      }
    }

    // Check if this is the last frame. If post processing is enabled
    // we have to wait post processing pipe to return the buffers.
    if (!postproc_enable_ && capture_cnt_ == sequence_cnt_) {
      QMMF_INFO("%s: Capture done.", __func__);
      capture_done_ = true;
      capture_signal_.Signal();
    }

    // return if cancel capture because we already returned the buffer
    if (cancel_capture_) {
      return;
    }
  }

  buffer.camera_id = camera_id_;
  snapshot_buffer_list_.insert(std::make_pair(buffer.fd, buffer));
  snapshot_buffer_stream_list_.insert(std::make_pair(buffer.fd, buffer.stream_id));

  assert(client_snapshot_cb_ != nullptr);
  client_snapshot_cb_(frame_number, buffer);

  QMMF_DEBUG("%s Exit ", __func__);
}

status_t CameraContext::ValidateResolution(const ImageFormat format,
                                           const uint32_t width,
                                           const uint32_t height) {

  QMMF_VERBOSE("%s Enter ", __func__);

  camera_metadata_entry_t entry;
  bool supported = false;
  uint32_t w, h;

  switch (format) {
    case ImageFormat::kJPEG:
    //TODO: ANDROID_SCALER_AVAILABLE_JPEG_SIZES tag is not available in static
    // meta.
    if (static_meta_.exists(ANDROID_SCALER_AVAILABLE_JPEG_SIZES)) {
      entry = static_meta_.find(ANDROID_SCALER_AVAILABLE_JPEG_SIZES);
      for (uint32_t i = 0 ; i < entry.count; i += 2) {
        w = entry.data.i32[i+0];
        h = entry.data.i32[i+1];
        QMMF_INFO("%s:(%d) Supported Jpeg:(%d)x(%d)",__func__, i, w, h);
        if(w == width && h == height) {
          supported = true;
          break;
        }
      }
    }
    supported = true;
    break;
    case ImageFormat::kNV12:
    if (static_meta_.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
      entry = static_meta_.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
      for (uint32_t i = 0 ; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == entry.data.i32[i]) {
          if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
              entry.data.i32[i+3]) {
            w = entry.data.i32[i+1];
            h = entry.data.i32[i+2];
            QMMF_DEBUG("%s: (%d) Supported Raw YUV:(%d)x(%d)",__func__,
                i, w, h);
            if(w == width && h == height) {
              supported = true;
              break;
            }
          }
        }
      }
    }
    break;
    case ImageFormat::kBayerRDI10BIT:
    case ImageFormat::kBayerRDI12BIT:
    if (static_meta_.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
      entry = static_meta_.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES);
      for (uint32_t i = 0 ; i < entry.count; i += 2) {
        w = entry.data.i32[i+0];
        h = entry.data.i32[i+1];
        QMMF_INFO("%s: (%d) Supported RAW RDI W(%d):H(%d)", __func__, i,
            width, height);
        if(w == width && h == height) {
          supported = true;
          break;
        }
      }
    }
    break;
    default:
    break;
  }
  if (!supported) {
    QMMF_ERROR("%s: format(0x%x):width(%d):height(%d) not supported!",
        __func__, format, width, height);
    return BAD_VALUE;
  }
  QMMF_VERBOSE("%s Exit ", __func__);
  return NO_ERROR;
}

status_t CameraContext::CaptureZSLImage() {

  QMMF_INFO("%s: Enter", __func__);
  status_t ret = NO_ERROR;

  bool regular_snapshot = false;
  assert(!snapshot_request_.streamIds.isEmpty());

  assert(zsl_port_.get() != nullptr);
  ZslPort* zsl_port = static_cast<ZslPort*>(zsl_port_.get());
  auto stat = zsl_port->PickZSLBuffer();
  if (NO_ERROR != stat) {
    QMMF_ERROR("%s Failed to find a good ZSL input buffer: %d",
        __func__, stat);
    QMMF_ERROR("%s Switching to regular snapshot!", __func__);
    regular_snapshot = true;
  }

  std::lock_guard<std::mutex> lock(device_access_lock_);
  uint8_t jpeg_quality = snapshot_param_.image_quality;
  int64_t last_frame_mumber;

  if (!regular_snapshot) {
    Camera3Request reprocess_request;
    reprocess_request.streamIds.add(zsl_port->GetInputStreamId());
    reprocess_request.streamIds.add(snapshot_request_.streamIds[0]);
    reprocess_request.metadata = zsl_port->GetInputBuffer().result;
    reprocess_request.metadata.update(ANDROID_JPEG_QUALITY, &jpeg_quality,
                                      1);
    if (snapshot_type_ == SnapshotMode::kVideo) {
      if (streaming_active_requests_.size() == 1) {
        auto request = streaming_active_requests_[0];
        for (auto stream_id : request.streamIds) {
          reprocess_request.streamIds.add(stream_id);
        }
      } else {
        QMMF_INFO("%s: No other active video streams!", __func__);
      }
    }
    QMMF_INFO("%s: Submit ZSL reprocess request!!", __func__);
    auto id = camera_device_->SubmitRequest(reprocess_request, false,
                                            &last_frame_mumber);
    if (0 > id) {
      QMMF_ERROR("%s Failed to submit ZSL reprocess request: %d",
                 __func__, id);
      ret = UNKNOWN_ERROR;
    }
  } else {
    QMMF_INFO("%s: Submit Reguar snapshot request!", __func__);

    uint32_t active_streamid_count = 0;
    if (snapshot_type_ == SnapshotMode::kVideo) {
      if (streaming_active_requests_.size() == 1) {
        auto request = streaming_active_requests_[0];
        for (auto stream_id : request.streamIds) {
          snapshot_request_.streamIds.add(stream_id);
          active_streamid_count++;
        }
      } else {
        QMMF_INFO("%s: No other active video streams!", __func__);
      }
    }
    snapshot_request_.metadata.update(ANDROID_JPEG_QUALITY, &jpeg_quality, 1);
    auto id = camera_device_->SubmitRequest(snapshot_request_,
                                            false,
                                            &last_frame_mumber);
    if (0 > id) {
      QMMF_ERROR("%s Failed to submit reguar snapshot request: %d",
                 __func__, id);
      ret = UNKNOWN_ERROR;
    }

    snapshot_request_.streamIds.
      resize(snapshot_request_.streamIds.size() - active_streamid_count);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

//Camera device callbacks
void CameraContext::CameraErrorCb(CameraErrorCode error_code,
                                  const CaptureResultExtras &result) {

  QMMF_WARN("%s: Camera Client: error_code:%d RequestId:%d FrameNumber:%d\n",
      __func__, error_code, result.requestId, result.frameNumber);

  if (nullptr != error_cb_) {
    RecorderErrorData error_data {};
    error_data.camera_id = camera_id_;
    error_data.error_code = error_code;
    error_cb_(error_data);
  }

  std::unique_lock<std::mutex> pending_frames_lock(pending_frames_lock_);
  if (last_frame_number_map_.size() > 0 && removed_stream_ids_.size() > 0) {
    QMMF_DEBUG("%s: last_frame_mumber.size(%d) & emoved_stream_ids_.size(%d)"
        "& last_frame_number_(%lld)", __func__,
        last_frame_number_map_.size(), removed_stream_ids_.size(),
        last_frame_number_);
    if (result.frameNumber == last_frame_number_) {
      QMMF_WARN("%s: Request corresponds to last_frame_number(%lld) is"
       "missed! Notify pending frame wait!!", __func__,
       last_frame_number_);
      pending_frames_.Signal();
      removed_stream_ids_.clear();
      last_frame_number_map_.clear();
    }
  }
}

void CameraContext::CameraIdleCb() {
  QMMF_WARN("%s: Camera is in Idle State!!", __func__);
}

void CameraContext::CameraShutterCb(const CaptureResultExtras &result,
                                    int64_t time_stamp) {

}

void CameraContext::CameraPreparedCb(int32_t) {

}

template <typename T>
bool CameraContext::UpdatePartialTag(CameraMetadata &result, int32_t tag,
                                     const T *value,
                                     uint32_t frame_number) {
  if (0 != result.update(tag, value, 1)) {
    return false;
  }
  return true;
}

template <typename T>
bool CameraContext::QueryPartialTag(const CameraMetadata &result,
                                    int32_t tag, T *value,
                                    uint32_t frame_number) {
  (void)frame_number;

  camera_metadata_ro_entry_t entry;

  entry = result.find(tag);
  if (entry.count == 0) {
    return false;
  }

  if (sizeof(T) == sizeof(uint8_t)) {
    *value = entry.data.u8[0];
  } else if (sizeof(T) == sizeof(int32_t)) {
    *value = entry.data.i32[0];
  } else {
    return false;
  }
  return true;
}

void CameraContext::HandleFinalResult(const CaptureResult &result) {

  if (result.metadata.exists(ANDROID_CONTROL_AE_STATE) &&
      result.metadata.exists(ANDROID_SENSOR_TIMESTAMP)) {
    auto new_state = result.metadata.find(ANDROID_CONTROL_AE_STATE).data.u8[0];
    auto timestamp = result.metadata.find(ANDROID_SENSOR_TIMESTAMP).data.i64[0];

    std::lock_guard<std::mutex> lock(aec_lock_);
    if (aec_.state != new_state) {
      QMMF_VERBOSE("%s: Camera: %d, Frame: %d, Ts: %lld AE state changed:"
          " %d --> %d", __func__, camera_id_,
          result.resultExtras.frameNumber, timestamp, aec_.state, new_state);

      aec_.state     = new_state;
      aec_.timestamp = timestamp;
      aec_state_updated_.Signal();
    }
  }

  if (camera_start_params_.zsl_mode) {
    assert(zsl_port_.get() != nullptr);
    ZslPort* zsl_port = static_cast<ZslPort*>(zsl_port_.get());
    zsl_port->HandleZSLCaptureResult(result);
  }
  if (nullptr != result_cb_) {
    result_cb_(camera_id_, result.metadata);
  }
  if (postproc_pipe_.get() != nullptr) {
    postproc_pipe_->AddResult(&result);
  }
  return;
}

void CameraContext::CameraResultCb(const CaptureResult &result) {

  std::lock_guard<std::mutex> lock(partial_result_lock_);
  if (result.resultExtras.partialResultCount < partial_result_count_) {

    bool complete_result = true;

    uint8_t afMode, afState, aeState, awbState, awbMode;
    auto frame_number  = result.resultExtras.frameNumber;

    complete_result &=
        QueryPartialTag(result.metadata, ANDROID_CONTROL_AWB_MODE,
                        &awbMode, frame_number);
    complete_result &=
        QueryPartialTag(result.metadata, ANDROID_CONTROL_AF_MODE,
                        &afMode, frame_number);
    complete_result &=
        QueryPartialTag(result.metadata, ANDROID_CONTROL_AE_STATE,
                        &aeState, frame_number);
    complete_result &= QueryPartialTag(result.metadata,
                                       ANDROID_CONTROL_AWB_STATE,
                                       &awbState, frame_number);
    complete_result &=
        QueryPartialTag(result.metadata, ANDROID_CONTROL_AF_STATE, &afState,
                        frame_number);

    if (!complete_result && partial_metadata_required_) {
      if (nullptr != result_cb_) {
        result_cb_(camera_id_, result.metadata);
      }
      return;
    }

    if (complete_result) {
      CaptureResult captureResult;
      captureResult.resultExtras = result.resultExtras;
      captureResult.metadata = CameraMetadata(10, 0);

      if (!UpdatePartialTag(captureResult.metadata, ANDROID_REQUEST_FRAME_COUNT,
                            reinterpret_cast<int32_t *>(&frame_number),
                            frame_number)) {
        return;
      }

      int32_t requestId = result.resultExtras.requestId;
      if (!UpdatePartialTag(captureResult.metadata, ANDROID_REQUEST_ID,
                            &requestId, frame_number)) {
        return;
      }

      if (!UpdatePartialTag(captureResult.metadata, ANDROID_CONTROL_AWB_STATE,
                            &awbState, frame_number)) {
        return;
      }
      if (!UpdatePartialTag(captureResult.metadata, ANDROID_CONTROL_AF_MODE,
                            &afMode, frame_number)) {
        return;
      }
      if (!UpdatePartialTag(captureResult.metadata, ANDROID_CONTROL_AWB_MODE,
                            &awbMode, frame_number)) {
        return;
      }
      if (!UpdatePartialTag(captureResult.metadata, ANDROID_CONTROL_AF_STATE,
                            &afState, frame_number)) {
        return;
      }
      if (!UpdatePartialTag(captureResult.metadata, ANDROID_CONTROL_AE_STATE,
                            &aeState, frame_number)) {
        return;
      }

      HandleFinalResult(captureResult);
    }
  } else {
    HandleFinalResult(result);
  }
}

CameraPort* CameraContext::GetPort(const uint32_t track_id) {

  CameraPort* port = nullptr;
  for (auto iter : active_ports_) {
    auto type = iter->GetPortType();
    if (track_id == iter->GetPortId() && (type != CameraPortType::kZSL)) {
      QMMF_INFO("%s: Found the port for id(0%x)", __func__, track_id);
      port = static_cast<CameraPort*>(iter.get());
      break;
    }
  }
  if (!port) {
    QMMF_ERROR("%s: No port belongs to consumer(%d)", __func__,
        track_id);
  }
  return port;
}

void CameraContext::DeletePort(const uint32_t track_id) {

  QMMF_DEBUG("%s: Enter track_id(%x)", __func__, track_id);
  auto iter = active_ports_.begin();
  while (iter != active_ports_.end()) {
    auto type = (*iter)->GetPortType();
    if (track_id == (*iter)->GetPortId()
        && (type != CameraPortType::kZSL)) {
      QMMF_INFO("%s: Found the port for id(0%x)", __func__, track_id);
      iter = active_ports_.erase(iter);
      break;
    } else {
      ++iter;
    }
  }
  QMMF_DEBUG("%s: Exit track_id(0%x)", __func__, track_id);
}

void CameraContext::OnFrameAvailable(StreamBuffer& buffer) {

  QMMF_DEBUG("%s: StreamBuffer(0x%p) fd: %d stream_id: %d ts: %lld",
      __func__, buffer.handle, buffer.fd, buffer.stream_id,
      buffer.timestamp);
  SnapshotCaptureCallback(buffer);
}

void CameraContext::NotifyBufferReturned(StreamBuffer& buffer) {

  QMMF_DEBUG("%s: StreamBuffer(0x%p) fd: %d stream_id: %d ts: %lld",
      __func__, buffer.handle, buffer.fd, buffer.stream_id,
      buffer.timestamp);
  ReturnStreamBuffer(buffer);

  // If snapshot post processing is enabled we have to wait
  // post process pipe to return all buffers
  if (!snapshot_request_.streamIds.empty() &&
      !snapshot_request_id_.empty() && postproc_enable_) {
    std::unique_lock<std::mutex> lock(capture_lock_);
    curr_snapshot_id_ = buffer.frame_number;

    if (last_snapshot_id_ == curr_snapshot_id_) {
      QMMF_INFO("%s: Capture done.", __func__);
      capture_done_ = true;
      capture_signal_.Signal();
    }
  }
}

status_t CameraContext::PostProcDelete() {

  QMMF_INFO("%s: Enter", __func__);
  if (postproc_pipe_.get() != nullptr) {
    postproc_pipe_->Stop();
    postproc_pipe_->RemoveConsumer(GetConsumerIntf());
    DetachConsumer(postproc_pipe_->GetConsumerIntf());
    postproc_pipe_->DeletePipe();
  }

  postproc_pipe_ = nullptr;
  QMMF_INFO("%s: Exit", __func__);
  return NO_ERROR;
}

status_t CameraContext::PostProcCreatePipeAndUpdateStreams(
                                        CameraStreamParameters& stream_param,
                                        uint32_t image_quality,
                                        uint32_t frame_rate,
                                        const std::vector<uint32_t> &plugins) {

  postproc_pipe_ = std::make_shared<PostProcPipe>(this);
  assert(postproc_pipe_.get() != nullptr);

  PipeIOParam out_param;
  out_param.width = stream_param.width;
  out_param.height = stream_param.height;
  out_param.format = stream_param.format;
  out_param.frame_rate = frame_rate;
  out_param.image_quality = image_quality;
  out_param.gralloc_flags = stream_param.grallocFlags;
  out_param.buffer_count = REPROC_STREAM_BUFFER_COUNT;
  out_param.max_internal_buffers = 0; // unlimited
  out_param.frame_skip = postproc_frame_skip_;

  PipeIOParam in_param;
  auto ret = postproc_pipe_->CreatePipe(out_param, plugins, in_param);
  if (ret != NO_ERROR) {
    return ret;
  }

  ret = postproc_pipe_->Configure(GetSnapshotJsonConfig());
  if (ret != NO_ERROR) {
    return ret;
  }

  stream_param.format = in_param.format;
  stream_param.width  = in_param.width;
  stream_param.height = in_param.height;
  stream_param.grallocFlags |= in_param.gralloc_flags;

  QMMF_INFO("%s: input dim %dx%d format %x ", __func__,
      stream_param.width, stream_param.height, stream_param.format);

  return NO_ERROR;
}

int32_t CameraContext::PostProcStart(int32_t stream_id) {
  postproc_pipe_->AddConsumer(GetConsumerIntf());
  AttachConsumer(postproc_pipe_->GetConsumerIntf());
  postproc_pipe_->Start(stream_id);

  return NO_ERROR;
}

AECData CameraContext::GetAECData() {

  std::lock_guard<std::mutex> lock(aec_lock_);
  return aec_;
}

CameraPort::CameraPort(const CameraStreamParam& param, size_t batch,
                       CameraPortType port_type, CameraContext* context)
    : port_type_(port_type),
      context_(context),
      camera_stream_id_(-1),
      params_(param),
      ready_to_start_(false),
      batch_size_(batch),
      port_id_(param.id) {

  QMMF_INFO("%s: Enter", __func__);

  BufferProducerImpl<CameraPort> *producer_impl;
  producer_impl = new BufferProducerImpl<CameraPort>(this);
  buffer_producer_impl_ = producer_impl;

  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

CameraPort::~CameraPort() {

  QMMF_INFO("%s: Enter ", __func__);
  buffer_producer_impl_.clear();
  buffer_producer_impl_ = nullptr;
  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

status_t CameraPort::Init() {

  cam_stream_params_ = {};
  if (params_.cam_stream_format == CameraStreamFormat::kRAW10) {
    cam_stream_params_.format       = HAL_PIXEL_FORMAT_RAW10;
  } else if (params_.cam_stream_format == CameraStreamFormat::kRAW12) {
    cam_stream_params_.format       = HAL_PIXEL_FORMAT_RAW12;
  } else {
    cam_stream_params_.format       = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  }
  cam_stream_params_.width        = params_.cam_stream_dim.width;
  cam_stream_params_.height       = params_.cam_stream_dim.height;
  cam_stream_params_.grallocFlags =
      GRALLOC_USAGE_SW_READ_OFTEN | GRALLOC_USAGE_SW_WRITE_OFTEN;
  cam_stream_params_.rotation     = static_cast<camera3_stream_rotation_t> (params_.rotation);
  bool is_lpm_use_preview = false;
  char prop[PROPERTY_VALUE_MAX];
  memset(prop, 0, sizeof(prop));
  property_get("persist.camera.lpm.preview", prop, "0");
  is_lpm_use_preview = atoi(prop);
  if (params_.low_power_mode) {
    cam_stream_params_.bufferCount = PREVIEW_STREAM_BUFFER_COUNT;
    if (!is_lpm_use_preview) {
      cam_stream_params_.format = HAL_PIXEL_FORMAT_YCbCr_420_888;
      cam_stream_params_.is_pp_enabled = false;
    }
  } else {
    cam_stream_params_.grallocFlags |= private_handle_t::
        PRIV_FLAGS_VIDEO_ENCODER;
    cam_stream_params_.bufferCount = VIDEO_STREAM_BUFFER_COUNT;
    if (params_.cam_stream_dim.width == 3840
        && params_.cam_stream_dim.height == 2160) {
      cam_stream_params_.bufferCount += EXTRA_DCVS_BUFFERS;
    }
  }
  cam_stream_params_.cb = [&] (StreamBuffer buffer) { StreamCallback(buffer); };

  assert(context_ != nullptr);

  if (context_->video_plugins_.count(port_id_) != 0) {
    auto port_plugins = context_->video_plugins_.at(port_id_);
    postproc_pipe_ = std::make_shared<PostProcPipe>(context_);
    assert(postproc_pipe_.get() != nullptr);

    PipeIOParam out_param;
    out_param.width = cam_stream_params_.width;
    out_param.height = cam_stream_params_.height;
    out_param.format = cam_stream_params_.format;
    out_param.frame_rate = static_cast<uint32_t>(params_.frame_rate);
    out_param.image_quality = 100;
    out_param.gralloc_flags = cam_stream_params_.grallocFlags;
    out_param.buffer_count = cam_stream_params_.bufferCount;

    PipeIOParam in_param;
    auto ret = postproc_pipe_->CreatePipe(out_param, port_plugins, in_param);
    if (ret != NO_ERROR) {
      return ret;
    }

    ret = postproc_pipe_->Configure("");
    if (ret != NO_ERROR) {
      return ret;
    }
    // Update stream parameters
    cam_stream_params_.format = in_param.format;
    cam_stream_params_.width  = in_param.width;
    cam_stream_params_.height = in_param.height;
  }

  int32_t stream_id;
  auto ret = context_->CreateDeviceStream(cam_stream_params_,
                                          params_.frame_rate, &stream_id);
  if (ret != NO_ERROR || stream_id < 0) {
    QMMF_ERROR("%s: CreateDeviceStream failed!!", __func__);
    return BAD_VALUE;
  }
  camera_stream_id_ = stream_id;

  if (context_->video_plugins_.count(port_id_) != 0) {
    sp<IBufferConsumer>& consumer = postproc_pipe_->GetConsumerIntf();
    buffer_producer_impl_->AddConsumer(consumer);
    consumer->SetProducerHandle(buffer_producer_impl_);
  }
  port_state_ = PortState::PORT_CREATED;

  QMMF_INFO("%s: Camera Device Stream(%d) is created Succussfully!",
      __func__, camera_stream_id_);
  QMMF_INFO("%s: track_id(0%x) is mapped to camera stream_id(%d)",
      __func__, params_.id, camera_stream_id_);
  return NO_ERROR;
}

status_t CameraPort::DeInit() {

  QMMF_INFO("%s: Enter", __func__);
  assert(ready_to_start_ == false);
  assert(context_ != nullptr);

  if (postproc_pipe_.get() != nullptr) {
    buffer_producer_impl_->RemoveConsumer(postproc_pipe_->GetConsumerIntf());
    postproc_pipe_->DeletePipe();
    postproc_pipe_ = nullptr;
  }

  auto ret = context_->DeleteDeviceStream(camera_stream_id_, true);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s: DeleteDeviceStream failed!!", __func__);
    return BAD_VALUE;
  }
  consumers_.clear();
  QMMF_DEBUG("%s: CameraPort(0x%p) deinitialized successfully! ",
      __func__, this);
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t CameraPort::Start() {

  if (port_state_ == PortState::PORT_STARTED){
    // Port is already in started state.
    return NO_ERROR;
  }

  if (postproc_pipe_.get() != nullptr) {
    postproc_pipe_->Start(camera_stream_id_);
  }

  //TODO: protect it with lock.
  ready_to_start_ = true;
  aec_converged_  = false;
  port_state_ = PortState::PORT_READYTOSTART;

  QMMF_INFO("%s: track_id(%x):camera stream(%d) to start!", __func__,
      port_id_, camera_stream_id_);

  auto ret = context_->UpdateRequest(true);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: UpdateRequest failed! for track_id = %d",
        __func__, port_id_);
    return ret;
  }
  QMMF_INFO("%s: track_id(%x):Port(%p) Started Succussfully!",
      __func__, port_id_, this);

  port_state_ = PortState::PORT_STARTED;
  return NO_ERROR;
}

status_t CameraPort::Stop() {

  if (port_state_ == PortState::PORT_CREATED ||
      port_state_ == PortState::PORT_STOPPED){
    // Port is already in stopped state.
    return NO_ERROR;
  }

  //TODO: protect it with lock.
  ready_to_start_ = false;
  aec_converged_  = false;
  port_state_ = PortState::PORT_READYTOSTOP;

  // Stop basically removes the stream from current running capture request,
  // it doen't delete the stream.
  auto ret = context_->UpdateRequest(true);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: CameraPort:Start:UpdateRequest failed! for track_id = %d"
        ,  __func__, port_id_);
    return ret;
  }

  if (postproc_pipe_.get() != nullptr) {
    postproc_pipe_->Stop();
  }
  QMMF_INFO("%s: track_id(%x):Port(%p) Stopped Succussfully!",
      __func__, port_id_, this);

  port_state_ = PortState::PORT_STOPPED;
  return NO_ERROR;
}

status_t CameraPort::AddConsumer(sp<IBufferConsumer>& consumer) {

  std::lock_guard<std::mutex> lock(consumer_lock_);
  assert(consumer.get() != nullptr);

  if (IsConsumerConnected(consumer)) {
    QMMF_ERROR("%s: consumer(%p) already added to the producer!",
        __func__, consumer.get());
    return ALREADY_EXISTS;
  }

  if (postproc_pipe_.get() != nullptr) {
    postproc_pipe_->AddConsumer(consumer);
  } else {
    // Add consumer to port's producer interface.
    assert(buffer_producer_impl_.get() != nullptr);
    buffer_producer_impl_->AddConsumer(consumer);
    consumer->SetProducerHandle(buffer_producer_impl_);
    QMMF_DEBUG("%s: Consumer(%p) has been added to CameraPort(%p)."
        "Total number of consumer = %d",  __func__, consumer.get()
        , this, buffer_producer_impl_->GetNumConsumer());
  }
  consumers_.emplace(reinterpret_cast<uintptr_t>(consumer.get()), consumer);
  return NO_ERROR;
}

status_t CameraPort::RemoveConsumer(sp<IBufferConsumer>& consumer) {
  std::lock_guard<std::mutex> lock(consumer_lock_);
  assert(consumer.get() != nullptr);
  if (!IsConsumerConnected(consumer)) {
    QMMF_ERROR("%s: consumer(%p) is not connected to this port(%p)!",
        __func__, consumer.get(), this);
    return BAD_VALUE;
  }

  if (postproc_pipe_.get() != nullptr) {
    postproc_pipe_->RemoveConsumer(consumer);
  } else {
    // Remove consumer from port's producer interface.
    assert(buffer_producer_impl_.get() != nullptr);
    buffer_producer_impl_->RemoveConsumer(consumer);
    QMMF_DEBUG("%s: Consumer(%p) has been removed from CameraPort(%p)."
        "Total number of consumer = %d",  __func__, consumer.get()
        , this, buffer_producer_impl_->GetNumConsumer());
  }
  consumers_.erase(reinterpret_cast<uintptr_t>(consumer.get()));
  return NO_ERROR;
}

void CameraPort::NotifyBufferReturned(const StreamBuffer& buffer) {

  QMMF_VERBOSE("%s: StreamBuffer(0x%p) Cameback to CameraPort",
       __func__, buffer.handle);
  //TODO: protect this with lock, would be required once multiple camera ports
  // are enabled.
  context_->ReturnStreamBuffer(buffer);
}

int32_t CameraPort::GetNumConsumers() {

  std::lock_guard<std::mutex> lock(consumer_lock_);
  return consumers_.size();
}

bool CameraPort::IsReadyToStart() {
  //TODO: protect it with lock.
  return ready_to_start_;
}

PortState& CameraPort::getPortState() {
  return port_state_;
}

bool CameraPort::IsConsumerConnected(sp<IBufferConsumer>& consumer) {

  uintptr_t key = reinterpret_cast<uintptr_t>(consumer.get());
  return (consumers_.count(key) != 0) ? true : false;
}

void CameraPort::StreamCallback(StreamBuffer buffer) {

  QMMF_VERBOSE("%s: Enter stream_id(%d)", __func__, buffer.stream_id);
  assert(buffer.stream_id == camera_stream_id_);
  assert(buffer_producer_impl_.get() != nullptr);

  // Assign camera id to the stream buffer.
  buffer.camera_id = context_->camera_id_;

  QMMF_VERBOSE("%s: camera_id: %d, stream_id: %d, buffer: %p ts: %lld "
      "frame_number: %d", __func__, buffer.camera_id, buffer.stream_id,
      buffer.handle, buffer.timestamp, buffer.frame_number);

  bool skip_frame = false;

  if (params_.wait_aec_mode) {
    // Get auto exposure data and check if initial AE has converged.
    std::lock_guard<std::mutex> lock(aec_lock_);
    if (!aec_converged_) {
      auto aec = context_->GetAECData();
      aec_converged_ = (aec.state == ANDROID_CONTROL_AE_STATE_LOCKED) ||
                       (aec.state == ANDROID_CONTROL_AE_STATE_CONVERGED);
      aec_timestamp_ = aec.timestamp;
    }
    // Raise the skip flag if AE did not converged for this buffer.
    skip_frame = !(aec_converged_ && (buffer.timestamp >= aec_timestamp_));
  }

  std::lock_guard<std::mutex> lock(consumer_lock_);
  if (buffer_producer_impl_->GetNumConsumer() > 0 && !skip_frame) {
    buffer_producer_impl_->NotifyBuffer(buffer);
  } else {
    QMMF_VERBOSE("%s: Return buffer back to camera!", __func__);
    context_->ReturnStreamBuffer(buffer);
  }

  QMMF_VERBOSE("%s: Exit ", __func__);
}

ZslPort::ZslPort(const CameraStreamParam& param, size_t batch_size,
                 CameraPortType port_type, CameraContext *context)
    : CameraPort(param, batch_size, port_type, context) {

  QMMF_INFO("%s: Enter", __func__);
  zsl_input_buffer_.timestamp = -1;
  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

ZslPort::~ZslPort() {

  QMMF_INFO("%s: Enter ", __func__);
  if (!zsl_queue_.empty()) {
    PauseAndFlushZSLQueue();
  }
  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

status_t ZslPort::Init() {

  assert(port_type_ == CameraPortType::kZSL);
  auto ret = SetUpZSL();
  if (ret != NO_ERROR) {
    QMMF_INFO("%s SetUpZSL failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: ZslPort Initialized!", __func__);
  return ret;
}

status_t ZslPort::PauseAndFlushZSLQueue() {

  QMMF_DEBUG("%s: Enter", __func__);
  int32_t ret = NO_ERROR;
  std::lock_guard<std::mutex> l(zsl_queue_lock_);
  zsl_running_ = false;

  if (!zsl_queue_.empty()) {
    std::list<ZSLEntry>::iterator it = zsl_queue_.begin();
    std::list<ZSLEntry>::iterator end = zsl_queue_.end();
    while (it != end) {
      if (it->timestamp == it->buffer.timestamp) {
        assert(context_ != nullptr);
        auto stat = context_->ReturnStreamBuffer(it->buffer);
        if (NO_ERROR != ret) {
          QMMF_ERROR("%s Failed to flush ZSL buffer: %d",
                     __func__, ret);
          ret = stat;
        }
      }
      it++;
    }
    zsl_queue_.clear();
  }
  QMMF_INFO("%s: Zsl queue flush is: %s", __func__,
      ret == NO_ERROR ? "Successful!" : "Failed!");

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

void ZslPort::ResumeZSL() {
  std::lock_guard<std::mutex> l(zsl_queue_lock_);
  zsl_running_ = true;
}

bool ZslPort::IsRunning() {
  std::lock_guard<std::mutex> l(zsl_queue_lock_);
  return zsl_running_ && (camera_stream_id_ > 0);
}

status_t ZslPort::PickZSLBuffer() {

  QMMF_DEBUG("%s Enter ", __func__);
  std::lock_guard<std::mutex> l(zsl_queue_lock_);
  auto ret = NO_ERROR;

  if (zsl_queue_.empty()) {
    QMMF_ERROR("%s ZSL queue is empty!\n", __func__);
    return NO_INIT;
  }

  if (0 <= zsl_input_buffer_.timestamp) {
    QMMF_ERROR("%s Previous ZSL input still processing!", __func__);
    return -EBUSY;
  }

  std::list<ZSLEntry>::iterator good_entry;
  std::list<ZSLEntry>::iterator it = zsl_queue_.begin();
  std::list<ZSLEntry>::iterator end = zsl_queue_.end();
  bool found = false;
  while (it != end) {
    if ((it->timestamp == it->buffer.timestamp) && (!it->result.isEmpty())) {
      camera_metadata_entry_t entry;
      entry = it->result.find(ANDROID_CONTROL_AE_STATE);
      if (0 < entry.count) {
        if ((entry.data.u8[0] == ANDROID_CONTROL_AE_STATE_CONVERGED) ||
            (entry.data.u8[0] == ANDROID_CONTROL_AE_STATE_LOCKED)) {
          good_entry = it;
          found = true;
        }
      }
    }
    it++;
  }

  if (found) {
    zsl_input_buffer_ = *good_entry;
    zsl_queue_.erase(good_entry);
    QMMF_INFO("%s: Found Good ZSL buffer!!", __func__);
  } else {
    QMMF_ERROR("%s: No appropriate ZSL buffer found!", __func__);
    ret = NAME_NOT_FOUND;
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

void ZslPort::HandleZSLCaptureResult(const CaptureResult &result) {

  QMMF_VERBOSE("%s Enter ", __func__);

  if (0 <= camera_stream_id_) {
    int64_t timestamp;
    if (result.metadata.exists(ANDROID_SENSOR_TIMESTAMP)) {
      timestamp = result.metadata.find(ANDROID_SENSOR_TIMESTAMP).data.i64[0];
    } else {
      QMMF_ERROR("%s Sensor timestamp tag missing in result!\n",
        __func__);
      return;
    }

    {
      ZSLEntry entry{};
      entry.timestamp = -1;

      std::lock_guard<std::mutex> l(zsl_queue_lock_);
      if (zsl_running_) {
        bool append = true;
        if (!zsl_queue_.empty()) {
          std::list<ZSLEntry>::iterator it = zsl_queue_.begin();
          std::list<ZSLEntry>::iterator end = zsl_queue_.end();
          while (it != end) {
            if (it->timestamp == timestamp) {
              it->result.append(result.metadata);
              append = false;
              break;
            }
            it++;
          }
        }

        if (append) {
          //Buffer is missing append to queue directly
          ZSLEntry new_entry{};
          new_entry.result.append(result.metadata);
          new_entry.timestamp = timestamp;
          zsl_queue_.push_back(new_entry);
        }

        if (zsl_queue_.size() >= (zsl_queue_depth_ + 1)) {
          entry = *zsl_queue_.begin(); //return oldest buffer
          zsl_queue_.erase(zsl_queue_.begin());
        }
      }

      if (entry.timestamp == entry.buffer.timestamp) {
        QMMF_DEBUG("%s Return Buffer from ZSl Queue back to camera!",
            __func__);
        assert (context_ != nullptr);
        auto ret = context_->ReturnStreamBuffer(entry.buffer);
        if (NO_ERROR != ret) {
          QMMF_ERROR("%s Failed to return ZSL buffer to camera: %d",
                     __func__, ret);
        }
      }
    }
  }
  QMMF_VERBOSE("%s Exit ", __func__);
}

status_t ZslPort::SetUpZSL() {

  QMMF_DEBUG("%s: Enter", __func__);
  bool is_fps_supported = false;

  CameraStartParam cam_start_param = context_->GetCameraStartParam();

  zsl_queue_depth_ = cam_start_param.zsl_queue_depth;
  if (0 == zsl_queue_depth_) {
    QMMF_ERROR("%s: Invalid ZSL queue depth size!", __func__);
    return BAD_VALUE;
  }

  for (auto &iter : context_->GetSupportedFps()) {
    if (iter == static_cast<int32_t>(cam_start_param.frame_rate)) {
      is_fps_supported = true;
      break;
    }
  }
  if (!is_fps_supported) {
    QMMF_ERROR("%s: Framerate: %d not supported by camera!",
        __func__, cam_start_param.frame_rate);
    return BAD_VALUE;
  }
  QMMF_INFO("%s zsl width(%d):height(%d), queue_depth=%d", __func__,
    cam_start_param.zsl_width, cam_start_param.zsl_height,
    cam_start_param.zsl_queue_depth);

  auto ret = context_->ValidateResolution(ImageFormat::kNV12,
                                          cam_start_param.zsl_width,
                                          cam_start_param.zsl_height);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: ZSL width(%d):height(%d) Not supported!",
        __func__, cam_start_param.zsl_width, cam_start_param.zsl_height);
    return ret;
  }

  CameraStreamParameters zsl_stream_params{};
  zsl_stream_params.bufferCount = cam_start_param.zsl_queue_depth +
                                  VIDEO_STREAM_BUFFER_COUNT;
  zsl_stream_params.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  zsl_stream_params.width  = cam_start_param.zsl_width;
  zsl_stream_params.height = cam_start_param.zsl_height;
  zsl_stream_params.grallocFlags = GRALLOC_USAGE_HW_FB
                                   |GRALLOC_USAGE_HW_CAMERA_ZSL;
  zsl_stream_params.cb = [&](StreamBuffer buffer)
      { ZSLCaptureCallback(buffer); };

  ret = context_->CreateDeviceStream(zsl_stream_params,
                                     cam_start_param.frame_rate,
                                     &camera_stream_id_);
  if (NO_ERROR != ret || camera_stream_id_ < 0) {
    QMMF_ERROR("%s: CreateDeviceStream failed!", __func__);
    return ret;
  }

  // Create Input stream for reprocess.
  CameraInputStreamParameters input_stream_params{};
  input_stream_params.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  input_stream_params.width  = cam_start_param.zsl_width;
  input_stream_params.height = cam_start_param.zsl_height;
  input_stream_params.get_input_buffer = [&] (StreamBuffer& buffer)
      { GetZSLInputBuffer(buffer); };
  input_stream_params.return_input_buffer  = [&] (StreamBuffer& buffer)
      { ReturnZSLInputBuffer(buffer); };

  int32_t stream_id;
  ret = context_->CreateDeviceInputStream(input_stream_params, &stream_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s Failed to create input reprocess stream: %d",
               __func__, ret);
    return ret;
  }
  assert(stream_id >= 0);
  input_stream_id_ = stream_id;
  QMMF_INFO("%s: zsl input_stream_id_(%d)", __func__, input_stream_id_);

  zsl_running_ = true;
  QMMF_INFO("%s: zsl port configured with stream id(%d)", __func__,
      camera_stream_id_);

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

void ZslPort::ZSLCaptureCallback(StreamBuffer buffer) {

  QMMF_DEBUG("%s Enter ", __func__);
  ZSLEntry entry;
  entry.timestamp = -1;
  {
    std::lock_guard<std::mutex> l(zsl_queue_lock_);
    if (zsl_running_) {
      bool append = true;
      if (!zsl_queue_.empty()) {
        std::list<ZSLEntry>::iterator it = zsl_queue_.begin();
        std::list<ZSLEntry>::iterator end = zsl_queue_.end();
        while (it != end) {
          if (it->timestamp == buffer.timestamp) {
            it->buffer = buffer;
            append = false;
            break;
          }
          it++;
        }
      }

      if (append) {
        //Result is missing append to queue directly
        ZSLEntry new_entry;
        new_entry.buffer = buffer;
        new_entry.timestamp = buffer.timestamp;
        new_entry.result.clear();
        zsl_queue_.push_back(new_entry);
      }

      if (zsl_queue_.size() >= (zsl_queue_depth_ + 1)) {
        entry = *zsl_queue_.begin(); //return oldest buffer
        zsl_queue_.erase(zsl_queue_.begin());
      }
    } else {
      entry.buffer = buffer;
      entry.timestamp = buffer.timestamp;
    }
  }

  if (entry.timestamp == entry.buffer.timestamp) {
    QMMF_DEBUG("%s Return Buffer from ZSl Queue back to camera!",
        __func__);
    assert (context_ != nullptr);
    auto ret = context_->ReturnStreamBuffer(entry.buffer);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s Failed to return ZSL buffer to camera: %d",
                 __func__, ret);
    }
  }
  QMMF_DEBUG("%s Exit ", __func__);
}

void ZslPort::GetZSLInputBuffer(StreamBuffer& buffer) {
  buffer = zsl_input_buffer_.buffer;
  QMMF_INFO("%s buffer(%d) submitted for reprocess!", __func__,
    buffer.fd);
}

void ZslPort::ReturnZSLInputBuffer(StreamBuffer& buffer) {

  QMMF_DEBUG("%s Enter ", __func__);
  if (buffer.handle == zsl_input_buffer_.buffer.handle) {
    assert (context_ != nullptr);
    QMMF_INFO("%s buffer(%d) returned from reprocess!", __func__,
        buffer.fd);
    auto ret = context_->ReturnStreamBuffer(zsl_input_buffer_.buffer);
    if (NO_ERROR == ret) {
      zsl_input_buffer_.timestamp = -1;
    } else {
      QMMF_ERROR("%s Failed to return input buffer: %d\n", __func__,
          ret);
    }
  } else {
    QMMF_ERROR("%s: Buffer handle of returned buffer: %p doesn't match with"
        "expected handle: %p\n",  __func__, buffer.handle,
        zsl_input_buffer_.buffer.handle);
  }
  QMMF_DEBUG("%s Exit ", __func__);
}

}; // namespace recoder

}; // namespace qmmf
