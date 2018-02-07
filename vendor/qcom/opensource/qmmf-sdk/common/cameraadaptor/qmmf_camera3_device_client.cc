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

#define TAG "CameraAdaptor"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <dlfcn.h>
#include <string.h>
#include <utils/String8.h>

#include "recorder/src/service/qmmf_recorder_common.h"
#include "qmmf_camera3_utils.h"
#include "qmmf_camera3_device_client.h"
#include <QCamera3VendorTags.h>

// Convenience macros for transitioning to the error state
#define SET_ERR(fmt, ...) \
  SetErrorState("%s: " fmt, __FUNCTION__, ##__VA_ARGS__)
#define SET_ERR_L(fmt, ...) \
  SetErrorStateLocked("%s: " fmt, __FUNCTION__, ##__VA_ARGS__)
using namespace qcamera;
extern "C" {
extern int set_camera_metadata_vendor_ops(const vendor_tag_ops_t *query_ops);
}

namespace qmmf {

namespace cameraadaptor {

Camera3DeviceClient::Camera3DeviceClient(CameraClientCallbacks clientCb)
    : client_cb_(clientCb),
      id_(0),
      state_(STATE_NOT_INITIALIZED),
      next_stream_id_(0),
      reconfig_(false),
      camera_module_(NULL),
      device_(NULL),
      number_of_cameras_(0),
      gralloc_device_(NULL),
      next_request_id_(0),
      frame_number_(0),
      next_shutter_frame_number_(0),
      next_shutter_input_frame_number_(0),
      partial_result_count_(0),
      is_partial_result_supported_(false),
      next_result_frame_number_(0),
      next_result_input_frame_number_(0),
      monitor_(),
      request_handler_(monitor_),
      pause_state_notify_(false),
      state_listeners_(0),
      is_hfr_supported_(false),
      is_raw_only_(false),
      hfr_mode_enabled_(false),
      prepare_handler_() {
  camera3_callback_ops::notify = &notifyFromHal;
  camera3_callback_ops::process_capture_result = &processCaptureResult;
  camera_module_callbacks_t::camera_device_status_change = &deviceStatusChange;
  camera_module_callbacks_t::torch_mode_status_change = &torchModeStatusChange;
  pthread_mutex_init(&lock_, NULL);
  pthread_mutex_init(&pending_requests_lock_, NULL);
  pthread_cond_init(&state_updated_, NULL);
  memset(&input_stream_, 0, sizeof(input_stream_));
  input_stream_.stream_id = -1;
  prepare_handler_.SetPrepareCb(clientCb.peparedCb);
}

Camera3DeviceClient::~Camera3DeviceClient() {
  if (!request_handler_.ExitPending()) {
    request_handler_.RequestExitAndWait();
  }

  if (NULL != device_) {
    device_->common.close(&device_->common);
  }

  prepare_handler_.Clear();
  if (!prepare_handler_.ExitPending()) {
    prepare_handler_.RequestExit();
  }

  for (uint32_t i = 0; i < streams_.size(); i++) {
    Camera3Stream *stream = streams_.editValueAt(i);
    delete stream;
  }
  streams_.clear();

  Vector<Camera3Stream* >::iterator it = deleted_streams_.begin();
  while (it != deleted_streams_.end()) {
    Camera3Stream *stream = *it;
    it = deleted_streams_.erase(it);
    delete stream;

  }
  deleted_streams_.clear();

  if (!monitor_.ExitPending()) {
    monitor_.RequestExit();
  }

  if (NULL != gralloc_device_) {
    gralloc_device_->common.close(&gralloc_device_->common);
  }
  if (camera_module_->get_vendor_tag_ops) {
    set_camera_metadata_vendor_ops(nullptr);
  }

  pthread_mutex_destroy(&lock_);
  pthread_mutex_destroy(&pending_requests_lock_);
  pthread_cond_destroy(&state_updated_);
}

int32_t Camera3DeviceClient::Initialize() {
  int32_t res = 0;
  hw_module_t const *module = NULL;

  pthread_mutex_lock(&lock_);

  if (state_ != STATE_NOT_INITIALIZED) {
    QMMF_ERROR("%s: Already initialized! \n", __func__);
    res = -ENOSYS;
    goto exit;
  }

  res = LoadHWModule(CAMERA_HARDWARE_MODULE_ID,
                     (const hw_module_t **)&camera_module_);

  if ((0 != res) || (NULL == camera_module_)) {
    QMMF_ERROR("%s: Unable to load Hal module: %d\n", __func__, res);
    goto exit;
  }

  QMMF_INFO("%s: Camera Module author: %s, version: %d name: %s\n", __func__,
            camera_module_->common.author, camera_module_->common.hal_api_version,
            camera_module_->common.name);

  number_of_cameras_ = camera_module_->get_number_of_cameras();
  QMMF_INFO("%s: Number of cameras: %d\n", __func__, number_of_cameras_);

  if (NULL != camera_module_->init) {
    res = camera_module_->init();
    if (0 != res) {
      QMMF_ERROR("%s: Failed to initialize Camera Hal module!", __func__);
      goto exit;
    }
  }

  if (camera_module_->get_vendor_tag_ops) {
    vendor_tag_ops_ = vendor_tag_ops_t();
    camera_module_->get_vendor_tag_ops(&vendor_tag_ops_);

    res = set_camera_metadata_vendor_ops(&vendor_tag_ops_);
    if (0 != res) {
      QMMF_ERROR(
          "%s: Could not set vendor tag descriptor, "
          "received error %s (%d). \n",
          __func__, strerror(-res), res);
      goto exit;
    }
  }

  camera_module_->set_callbacks(this);

  res = LoadHWModule(GRALLOC_HARDWARE_MODULE_ID, &module);
  if ((0 != res) || (NULL == module)) {
    QMMF_ERROR("%s: Unable to load GrallocHal module: %d\n", __func__, res);
    goto exit;
  }

  module->methods->open(module, GRALLOC_HARDWARE_GPU0,
                        (struct hw_device_t **)&gralloc_device_);
  if (0 != res) {
    QMMF_ERROR("%s: Could not open Gralloc module: %s (%d) \n", __func__,
               strerror(-res), res);
    goto exit;
  }

  QMMF_INFO("%s: Gralloc Module author: %s, version: %d name: %s\n", __func__,
            gralloc_device_->common.module->author,
            gralloc_device_->common.module->hal_api_version,
            gralloc_device_->common.module->name);

  state_ = STATE_CLOSED;
  next_stream_id_ = 0;
  reconfig_ = true;

  pthread_mutex_unlock(&lock_);

  return res;

exit:

  if (NULL != camera_module_) {
    dlclose(camera_module_->common.dso);
  }
  device_ = NULL;
  camera_module_ = NULL;

  if (NULL != module) {
    dlclose(module->dso);
  }

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::OpenCamera(uint32_t idx) {
  int32_t res = 0;
  String8 Id;
  camera_metadata_entry_t capsEntry;
  MarkRequest mark_cb = [&] (uint32_t frameNumber, int32_t numBuffers,
                                 CaptureResultExtras resultExtras) {
    return MarkPendingRequest(frameNumber,numBuffers, resultExtras); };
  SetError set_error = [&] (const char *fmt, va_list args) {
    SetErrorStateV(fmt, args);
  };
  if (idx >= number_of_cameras_) {
    QMMF_ERROR("%s: Invalid camera idx: %d\n", __func__, idx);
    return -EINVAL;
  }

  if (NULL == camera_module_) {
    QMMF_ERROR("%s: Hal module not initialized yet!\n", __func__);
    return -ENODEV;
  }

  if (NULL != device_) {
    QMMF_ERROR("%s: Camera device is already open!\n", __func__);
    return -EINVAL;
  }

  pthread_mutex_lock(&lock_);

  if (state_ != STATE_CLOSED) {
    QMMF_ERROR("%s: Invalid state: %d! \n", __func__, state_);
    res = -ENOSYS;
    goto exit;
  }

  res = camera_module_->get_camera_info(idx, &static_info_);
  if (0 != res) {
    QMMF_ERROR("%s: Error during camera static info query: %s!\n", __func__,
               strerror(res));
    goto exit;
  }
  device_info_ = static_info_.static_camera_characteristics;

  Id.appendFormat("%d", idx);
  res = camera_module_->common.methods->open(&camera_module_->common, Id.string(),
                                            (hw_device_t **)(&device_));
  if (0 != res) {
    QMMF_ERROR("Could not open camera: %s (%d) \n", strerror(-res), res);
    goto exit;
  }

  if (device_->common.version < CAMERA_DEVICE_API_VERSION_3_2) {
    QMMF_ERROR(
        "Could not open camera: "
        "Camera device should be at least %x, reports %x instead",
        CAMERA_DEVICE_API_VERSION_3_2, device_->common.version);
    res = -EINVAL;
    goto exit;
  }

  res = device_->ops->initialize(device_, this);
  if (0 != res) {
    QMMF_ERROR("Could not initialize camera: %s (%d) \n", strerror(-res), res);
    goto exit;
  }

  {
    camera_metadata_entry partialResultsCount =
        device_info_.find(ANDROID_REQUEST_PARTIAL_RESULT_COUNT);
    if (partialResultsCount.count > 0) {
      partial_result_count_ = partialResultsCount.data.i32[0];
      is_partial_result_supported_ = (partial_result_count_ > 1);
    }
  }

  capsEntry = device_info_.find(ANDROID_REQUEST_AVAILABLE_CAPABILITIES);
  for (uint32_t i = 0; i < capsEntry.count; ++i) {
    uint8_t caps = capsEntry.data.u8[i];
    if (ANDROID_REQUEST_AVAILABLE_CAPABILITIES_CONSTRAINED_HIGH_SPEED_VIDEO ==
        caps) {
      is_hfr_supported_ = true;
      break;
    }
  }

  id_ = idx;
  state_ = STATE_NOT_CONFIGURED;

  monitor_.SetIdleNotifyCb([&] (bool idle) {NotifyStatus(idle);});
  monitor_.Run(String8::format("C3-%d-Monitor", id_).string());
  if (0 != res) {
    SET_ERR_L("Unable to start monitor: %s (%d)", strerror(-res), res);
    goto exit;
  }

  request_handler_.Initialize(device_, client_cb_.errorCb, mark_cb, set_error);
  res = request_handler_.Run(String8::format("C3-%d-Handler", id_).string());
  if (0 > res) {
    SET_ERR_L("Unable to start request handler: %s (%d)", strerror(-res), res);
    goto exit;
  }

  pthread_mutex_unlock(&lock_);

  return res;

exit:

  if (device_) {
    device_->common.close(&device_->common);
  }
  device_ = NULL;
  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::EndConfigure(bool isConstrainedHighSpeed,
                                          bool isRawOnly, uint32_t batch_size,
                                          bool is_pp_enabled) {
  if (NULL == camera_module_) {
    return -ENODEV;
  }

  if (isConstrainedHighSpeed && !is_hfr_supported_) {
    QMMF_ERROR("%s: HFR mode is not supported by this camera!\n", __func__);
    return -EINVAL;
  }

  return ConfigureStreams(isConstrainedHighSpeed, isRawOnly, batch_size, is_pp_enabled);
}

int32_t Camera3DeviceClient::ConfigureStreams(bool isConstrainedHighSpeed,
                                              bool isRawOnly,
                                              uint32_t batch_size,
                                              bool is_pp_enabled) {
  pthread_mutex_lock(&lock_);

  hfr_mode_enabled_ = isConstrainedHighSpeed;
  is_raw_only_ = isRawOnly;
  batch_size_ = batch_size;

  bool res = ConfigureStreamsLocked(is_pp_enabled);

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::ConfigureStreamsLocked(bool is_pp_enabled) {
  status_t res;

  if (state_ != STATE_NOT_CONFIGURED && state_ != STATE_CONFIGURED) {
    QMMF_ERROR("%s: Not idle\n", __func__);
    return -ENOSYS;
  }

  if (!reconfig_) {
    QMMF_ERROR("%s: Skipping config, no stream changes\n", __func__);
    return 0;
  }

  camera3_stream_configuration config;
  memset(&config, 0, sizeof(config));
  if (hfr_mode_enabled_) {
    config.operation_mode =
        CAMERA3_STREAM_CONFIGURATION_CONSTRAINED_HIGH_SPEED_MODE;
  } else if (is_raw_only_) {
    config.operation_mode = QCAMERA3_VENDOR_STREAM_CONFIGURATION_RAW_ONLY_MODE;
  } else if (!is_pp_enabled) {
    config.operation_mode =
        QCAMERA3_VENDOR_STREAM_CONFIGURATION_PP_DISABLED_MODE;
  } else {
    config.operation_mode = CAMERA3_STREAM_CONFIGURATION_NORMAL_MODE;
  }
  Vector<camera3_stream_t *> streams;
  for (size_t i = 0; i < streams_.size(); i++) {
    camera3_stream_t *outputStream;
    outputStream = streams_.editValueAt(i)->BeginConfigure();
    if (outputStream == NULL) {
      QMMF_ERROR("%s: Can't start stream configuration\n", __func__);
      return -ENOSYS;
    }
    streams.add(outputStream);
  }

  if (0 <= input_stream_.stream_id) {
    input_stream_.usage = 0; //Reset any previously set usage flags from Hal
    streams.add(&input_stream_);
  }

  config.streams = streams.editArray();
  config.num_streams = streams.size();

  res = device_->ops->configure_streams(device_, &config);

  if (res == -EINVAL) {
    for (uint32_t i = 0; i < streams_.size(); i++) {
      Camera3Stream *stream = streams_.editValueAt(i);
      if (stream->IsConfigureActive()) {
        res = stream->AbortConfigure();
        if (0 != res) {
          QMMF_ERROR("Can't abort stream %d configure: %s (%d)\n",
                     stream->GetId(), strerror(-res), res);
          return res;
        }
      }
    }

    InternalUpdateStatusLocked(STATE_NOT_CONFIGURED);
    reconfig_ = true;

    return -EINVAL;
  } else if (0 != res) {
    QMMF_ERROR("%s: Unable to configure streams with HAL: %s (%d)\n", __func__,
               strerror(-res), res);
    return res;
  }

  for (uint32_t i = 0; i < streams_.size(); i++) {
    Camera3Stream *outputStream = streams_.editValueAt(i);
    if (outputStream->IsConfigureActive()) {
      res = outputStream->EndConfigure();
      if (0 != res) {
        QMMF_ERROR(
            "%s: Unable to complete stream configuration"
            "%d: %s (%d)\n",
            __func__, outputStream->GetId(), strerror(-res), res);
        return res;
      }
    }
  }

  request_handler_.FinishConfiguration(batch_size_);
  reconfig_ = false;
  frame_number_ = 0;
  InternalUpdateStatusLocked(STATE_CONFIGURED);

  Vector<Camera3Stream* >::iterator it = deleted_streams_.begin();
  while (it != deleted_streams_.end()) {
    Camera3Stream *stream = *it;
    it = deleted_streams_.erase(it);
    delete stream;

  }
  deleted_streams_.clear();

  return 0;
}

int32_t Camera3DeviceClient::DeleteStream(int streamId, bool cache) {
  int32_t res = 0;
  Camera3Stream *stream;
  int32_t outputStreamIdx;
  pthread_mutex_lock(&lock_);

  switch (state_) {
    case STATE_ERROR:
      QMMF_ERROR("%s: Device has encountered a serious error\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_INITIALIZED:
    case STATE_CLOSED:
      QMMF_ERROR("%s: Device not initialized\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_CONFIGURED:
    case STATE_CONFIGURED:
    case STATE_RUNNING:
      if (!cache) {
        QMMF_INFO("%s:%s: Stream is not cached, Issue internal reconfig!", TAG,
            __func__);
        res = InternalPauseAndWaitLocked();
        if (0 != res) {
          SET_ERR_L("Can't pause captures to reconfigure streams!");
          goto exit;
        }
      }
      break;
    default:
      QMMF_ERROR("%s: Unknown state: %d\n", __func__, state_);
      res = -ENOSYS;
      goto exit;
  }

  if (streamId == input_stream_.stream_id) {
    input_stream_.stream_id = -1;
  } else {
    outputStreamIdx = streams_.indexOfKey(streamId);
    if (outputStreamIdx == -ENOENT) {
      QMMF_ERROR("%s: Stream %d does not exist\n", __func__, streamId);
      res = -EINVAL;
      goto exit;
    }

    stream = streams_.editValueAt(outputStreamIdx);
    if (request_handler_.IsStreamActive(*stream)) {
      QMMF_ERROR("%s: Stream %d still has pending requests\n", __func__,
                 streamId);
      res = -ENOSYS;
      goto exit;
    }

    streams_.removeItem(streamId);

    res = stream->Close();
    if (0 != res) {
      QMMF_ERROR("%s: Can't close deleted stream %d\n", __func__, streamId);
    }
    if (!cache && !streams_.isEmpty()) {
      reconfig_ = true;
      res = ConfigureStreamsLocked();
      if (0 != res) {
        QMMF_ERROR("%s:Can't reconfigure device for new stream %d: %s (%d)",
                 __func__, next_stream_id_, strerror(-res), res);
        goto exit;
      }
      InternalResumeLocked();
    } else {
      // In this scenario stream will be cached and will not trigger stream
      // reconfiguration. reconfiguration will be triggered in next round of
      // updating streaming capture request - creating a brand new stream or
      // deleting an existing stream without caching.
      if (state_ != STATE_RUNNING) {
        // Avoid reconfiguration if any existing stream is running, otherwise
        // updating capture request for setting parameters will try to
        // reconfigure it.
        reconfig_ = true;
      }
      deleted_streams_.push_back(stream);
    }
  }

exit:

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::CreateInputStream(
    const CameraInputStreamParameters &inputConfiguration) {
  int32_t res = 0;
  bool wasActive = false;
  pthread_mutex_lock(&lock_);

  if ((nullptr == inputConfiguration.get_input_buffer) ||
      (nullptr == inputConfiguration.return_input_buffer)) {
    QMMF_ERROR("%s: Input stream callbacks are invalid!\n", __func__);
    res = -EINVAL;
    goto exit;
  }

  if (0 <= input_stream_.stream_id) {
    QMMF_ERROR("%s: Only one input stream can be created at any time!\n",
               __func__);
    res = -ENOSYS;
    goto exit;
  }

  switch (state_) {
    case STATE_ERROR:
      QMMF_ERROR("%s: Device has encountered a serious error\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_INITIALIZED:
    case STATE_CLOSED:
      QMMF_ERROR("%s: Device not initialized\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_CONFIGURED:
    case STATE_CONFIGURED:
      break;
    case STATE_RUNNING:
      res = InternalPauseAndWaitLocked();
      if (0 != res) {
        SET_ERR_L("Can't pause captures to reconfigure streams!");
        goto exit;
      }
      wasActive = true;
      break;
    default:
      QMMF_ERROR("%s: Unknown state: %d\n", __func__, state_);
      res = -ENOSYS;
      goto exit;
  }
  assert(state_ != STATE_RUNNING);

  reconfig_ = true;

  memset(&input_stream_, 0, sizeof(input_stream_));
  input_stream_.width = inputConfiguration.width;
  input_stream_.height = inputConfiguration.height;
  input_stream_.format = inputConfiguration.format;
  input_stream_.get_input_buffer = inputConfiguration.get_input_buffer;
  input_stream_.return_input_buffer = inputConfiguration.return_input_buffer;
  input_stream_.stream_id = next_stream_id_++;
  input_stream_.stream_type = CAMERA3_STREAM_INPUT;

  // Continue captures if active at start
  if (wasActive) {
    res = ConfigureStreamsLocked();
    if (0 != res) {
      QMMF_ERROR("%s:Can't reconfigure device for new stream %d: %s (%d)",
                 __func__, next_stream_id_, strerror(-res), res);
      goto exit;
    }
    InternalResumeLocked();
  }

  res = input_stream_.stream_id;

exit:

    pthread_mutex_unlock(&lock_);

    return res;
}

int32_t Camera3DeviceClient::CreateStream(
    const CameraStreamParameters &outputConfiguration) {
  int32_t res = 0;
  Camera3Stream *newStream = NULL;
  int32_t blobBufferSize = 0;
  bool wasActive = false;
  pthread_mutex_lock(&lock_);

  if (nullptr == outputConfiguration.cb) {
    QMMF_ERROR("%s: Stream callback invalid!\n", __func__);
    res = -EINVAL;
    goto exit;
  }

  switch (state_) {
    case STATE_ERROR:
      QMMF_ERROR("%s: Device has encountered a serious error\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_INITIALIZED:
    case STATE_CLOSED:
      QMMF_ERROR("%s: Device not initialized\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_CONFIGURED:
    case STATE_CONFIGURED:
      break;
    case STATE_RUNNING:
      res = InternalPauseAndWaitLocked();
      if (0 != res) {
        SET_ERR_L("Can't pause captures to reconfigure streams!");
        goto exit;
      }
      wasActive = true;
      break;
    default:
      QMMF_ERROR("%s: Unknown state: %d\n", __func__, state_);
      res = -ENOSYS;
      goto exit;
  }
  assert(state_ != STATE_RUNNING);

  if (outputConfiguration.format == HAL_PIXEL_FORMAT_BLOB) {
    blobBufferSize = CaclulateBlobSize(outputConfiguration.width,
                                       outputConfiguration.height);
    if (blobBufferSize <= 0) {
      QMMF_ERROR("%s: Invalid jpeg buffer size %zd\n", __func__,
                 blobBufferSize);
      res = -EINVAL;
      goto exit;
    }
  }
  newStream = new Camera3Stream(next_stream_id_, blobBufferSize,
                                outputConfiguration, gralloc_device_, monitor_);
  if (NULL == newStream) {
    res = -ENOMEM;
    goto exit;
  }

  res = streams_.add(next_stream_id_, newStream);
  if (res < 0) {
    QMMF_ERROR("%s: Can't add new stream to set: %s (%d)\n", __func__,
               strerror(-res), res);
    goto exit;
  }

  reconfig_ = true;

  // Continue captures if active at start
  if (wasActive) {
    res = ConfigureStreamsLocked();
    if (0 != res) {
      QMMF_ERROR("%s:Can't reconfigure device for new stream %d: %s (%d)",
                 __func__, next_stream_id_, strerror(-res), res);
      goto exit;
    }
    InternalResumeLocked();
  }

  res = next_stream_id_++;

exit:

  pthread_mutex_unlock(&lock_);
  return res;
}

int32_t Camera3DeviceClient::QueryMaxBlobSize(int32_t &maxBlobWidth,
                                              int32_t &maxBlobHeight) {
  maxBlobWidth = 0;
  maxBlobHeight = 0;
  camera_metadata_entry_t availableStreamConfigs =
      device_info_.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
  if (availableStreamConfigs.count == 0 ||
      availableStreamConfigs.count % 4 != 0) {
    return 0;
  }

  for (uint32_t i = 0; i < availableStreamConfigs.count; i += 4) {
    int32_t format = availableStreamConfigs.data.i32[i];
    int32_t width = availableStreamConfigs.data.i32[i + 1];
    int32_t height = availableStreamConfigs.data.i32[i + 2];
    int32_t isInput = availableStreamConfigs.data.i32[i + 3];
    if (isInput == ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT &&
        format == HAL_PIXEL_FORMAT_BLOB &&
        (width * height > maxBlobWidth * maxBlobHeight)) {
      maxBlobWidth = width;
      maxBlobHeight = height;
    }
  }

  return 0;
}

int32_t Camera3DeviceClient::CaclulateBlobSize(int32_t width, int32_t height) {
  // Get max jpeg size (area-wise).
  int32_t maxJpegSizeWidth = 0;
  int32_t maxJpegSizeHeight = 0;
  QueryMaxBlobSize(maxJpegSizeWidth, maxJpegSizeHeight);
  if (maxJpegSizeWidth == 0) {
    QMMF_ERROR(
        "%s: Camera %d: Can't find valid available jpeg sizes in "
        "static metadata!\n",
        __func__, id_);
    return -EINVAL;
  }

  // Get max jpeg buffer size
  int32_t maxJpegBufferSize = 0;
  camera_metadata_entry jpegBufMaxSize =
      device_info_.find(ANDROID_JPEG_MAX_SIZE);
  if (jpegBufMaxSize.count == 0) {
    QMMF_ERROR(
        "%s: Camera %d: Can't find maximum JPEG size in static"
        " metadata!\n",
        __func__, id_);
    return -EINVAL;
  }
  maxJpegBufferSize = jpegBufMaxSize.data.i32[0];
  assert(JPEG_BUFFER_SIZE_MIN < maxJpegBufferSize);

  // Calculate final jpeg buffer size for the given resolution.
  float scaleFactor =
      ((float)(width * height)) / (maxJpegSizeWidth * maxJpegSizeHeight);
  ssize_t jpegBufferSize =
      scaleFactor * (maxJpegBufferSize - JPEG_BUFFER_SIZE_MIN) +
      JPEG_BUFFER_SIZE_MIN;
  if (jpegBufferSize > maxJpegBufferSize) {
    jpegBufferSize = maxJpegBufferSize;
  }

  return jpegBufferSize;
}

int32_t Camera3DeviceClient::CreateDefaultRequest(int templateId,
                                                  CameraMetadata *request) {
  int32_t res = 0;
  pthread_mutex_lock(&lock_);

  switch (state_) {
    case STATE_ERROR:
      QMMF_ERROR("%s: Device has encountered a serious error\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_INITIALIZED:
    case STATE_CLOSED:
      QMMF_ERROR("%s: Device is not initialized!\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_CONFIGURED:
    case STATE_CONFIGURED:
    case STATE_RUNNING:
      break;
    default:
      QMMF_ERROR("%s: Unexpected status: %d", __func__, state_);
      res = -ENOSYS;
      goto exit;
  }

  if (!request_templates_[templateId].isEmpty()) {
    *request = request_templates_[templateId];
    goto exit;
  }

  const camera_metadata_t *rawRequest;
  rawRequest =
      device_->ops->construct_default_request_settings(device_, templateId);
  if (rawRequest == NULL) {
    QMMF_ERROR("%s: template %d is not supported on this camera device\n",
               __func__, templateId);
    res = -EINVAL;
    goto exit;
  }
  *request = rawRequest;
  request_templates_[templateId] = rawRequest;

exit:

  pthread_mutex_unlock(&lock_);
  return res;
}

int32_t Camera3DeviceClient::MarkPendingRequest(
    uint32_t frameNumber, int32_t numBuffers,
    CaptureResultExtras resultExtras) {
  pthread_mutex_lock(&pending_requests_lock_);

  int32_t res;
  res = pending_requests_vector_.add(frameNumber,
                                   PendingRequest(numBuffers, resultExtras));

  pthread_mutex_unlock(&pending_requests_lock_);

  return res;
}

bool Camera3DeviceClient::HandlePartialResult(
    uint32_t frameNumber, const CameraMetadata &partial,
    const CaptureResultExtras &resultExtras) {

  bool completeResult = true;

  uint8_t afMode, afState, aeState, awbState, awbMode;

  completeResult &=
      QueryPartialTag(partial, ANDROID_CONTROL_AWB_MODE, &awbMode, frameNumber);
  completeResult &=
      QueryPartialTag(partial, ANDROID_CONTROL_AF_MODE, &afMode, frameNumber);
  completeResult &=
      QueryPartialTag(partial, ANDROID_CONTROL_AE_STATE, &aeState, frameNumber);
  completeResult &= QueryPartialTag(partial, ANDROID_CONTROL_AWB_STATE,
                                    &awbState, frameNumber);
  completeResult &=
      QueryPartialTag(partial, ANDROID_CONTROL_AF_STATE, &afState, frameNumber);

  if (!completeResult) {
    return false;
  }

  if (nullptr != client_cb_.resultCb) {
    CaptureResult captureResult;
    captureResult.resultExtras = resultExtras;
    captureResult.metadata = CameraMetadata(10, 0);

    if (!UpdatePartialTag(captureResult.metadata, ANDROID_REQUEST_FRAME_COUNT,
                          reinterpret_cast<int32_t *>(&frameNumber),
                          frameNumber)) {
      return false;
    }

    int32_t requestId = resultExtras.requestId;
    if (!UpdatePartialTag(captureResult.metadata, ANDROID_REQUEST_ID,
                          &requestId, frameNumber)) {
      return false;
    }

    if (device_->common.version < CAMERA_DEVICE_API_VERSION_3_2) {
      static const uint8_t partialResult =
          ANDROID_QUIRKS_PARTIAL_RESULT_PARTIAL;
      if (!UpdatePartialTag(captureResult.metadata,
                            ANDROID_QUIRKS_PARTIAL_RESULT, &partialResult,
                            frameNumber)) {
        return false;
      }
    }

    if (!UpdatePartialTag(captureResult.metadata, ANDROID_CONTROL_AWB_STATE,
                          &awbState, frameNumber)) {
      return false;
    }
    if (!UpdatePartialTag(captureResult.metadata, ANDROID_CONTROL_AF_MODE,
                          &afMode, frameNumber)) {
      return false;
    }
    if (!UpdatePartialTag(captureResult.metadata, ANDROID_CONTROL_AWB_MODE,
                          &awbMode, frameNumber)) {
      return false;
    }
    if (!UpdatePartialTag(captureResult.metadata, ANDROID_CONTROL_AF_STATE,
                          &afState, frameNumber)) {
      return false;
    }
    if (!UpdatePartialTag(captureResult.metadata, ANDROID_CONTROL_AE_STATE,
                          &aeState, frameNumber)) {
      return false;
    }

    client_cb_.resultCb(captureResult);
  }

  return true;
}

template <typename T>
bool Camera3DeviceClient::QueryPartialTag(const CameraMetadata &result,
                                          int32_t tag, T *value,
                                          uint32_t frameNumber) {
  (void)frameNumber;

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

template <typename T>
bool Camera3DeviceClient::UpdatePartialTag(CameraMetadata &result, int32_t tag,
                                           const T *value,
                                           uint32_t frameNumber) {
  if (0 != result.update(tag, value, 1)) {
    return false;
  }
  return true;
}

int32_t Camera3DeviceClient::CancelRequest(int requestId,
                                           int64_t *lastFrameNumber) {
  Vector<int32_t>::iterator it, end;
  int32_t res = 0;

  pthread_mutex_lock(&lock_);

  switch (state_) {
    case STATE_ERROR:
      QMMF_ERROR("%s: Device has encountered a serious error\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_INITIALIZED:
    case STATE_CLOSED:
      QMMF_ERROR("%s: Device not initialized\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_CONFIGURED:
    case STATE_CONFIGURED:
    case STATE_RUNNING:
      break;
    default:
      SET_ERR_L("Unknown state: %d", state_);
      res = -ENOSYS;
      goto exit;
  }

  for (it = repeating_requests_.begin(), end = repeating_requests_.end();
       it != end; ++it) {
    if (*it == requestId) {
      break;
    }
  }

  if (it == end) {
    QMMF_ERROR(
        "%s: Camera%d: Did not find request id %d in list of"
        " streaming requests",
        __FUNCTION__, id_, requestId);
    res = -EINVAL;
    goto exit;
  }

  res = request_handler_.ClearRepeatingRequests(lastFrameNumber);
  if (0 == res) {
    repeating_requests_.clear();
  }

exit:

  pthread_mutex_unlock(&lock_);
  return res;
}

void Camera3DeviceClient::HandleCaptureResult(
    const camera3_capture_result *result) {
  int32_t res;

  uint32_t frameNumber = result->frame_number;
  if (result->result == NULL && result->num_output_buffers == 0 &&
      result->input_buffer == NULL) {
    SET_ERR("No result data provided by HAL for frame %d", frameNumber);
    return;
  }

  if (!is_partial_result_supported_ && result->result != NULL &&
      result->partial_result != 1) {
    SET_ERR(
        "Result is invalid for frame %d: partial_result %u should be 1"
        " when partial result are not supported\n",
        frameNumber, result->partial_result);
    return;
  }

  bool isPartialResult = false;
  CameraMetadata collectedPartialResult;
  camera_metadata_ro_entry_t entry;
  uint32_t numBuffersReturned;

  int64_t shutterTimestamp = 0;

  pthread_mutex_lock(&pending_requests_lock_);
  ssize_t idx = pending_requests_vector_.indexOfKey(frameNumber);
  if (-ENOENT == idx) {
    SET_ERR("Invalid frame number in capture result: %d", frameNumber);
    pthread_mutex_unlock(&pending_requests_lock_);
    return;
  }
  PendingRequest &request = pending_requests_vector_.editValueAt(idx);
  QMMF_DEBUG(
      "%s: Received PendingRequest requestId = %d, frameNumber = %d,"
      "burstId = %d, partialResultCount = %d\n",
      __func__, request.resultExtras.requestId,
      request.resultExtras.frameNumber, request.resultExtras.burstId,
      result->partial_result);
  if (result->partial_result != 0)
    request.resultExtras.partialResultCount = result->partial_result;

  if (is_partial_result_supported_ && result->result != NULL) {
    if (result->partial_result > partial_result_count_ ||
        result->partial_result < 1) {
      SET_ERR(
          "Result is invalid for frame %d: partial_result %u"
          "should be in the range of [1, %d] when meta gets included"
          "in the result",
          frameNumber, result->partial_result, partial_result_count_);
      goto exit;
    }
    isPartialResult = (result->partial_result < partial_result_count_);
    if (isPartialResult) {
      request.partialResult.composedResult.append(result->result);
    }

    if (isPartialResult) {
      if (!request.partialResult.partial3AReceived) {
        request.partialResult.partial3AReceived = HandlePartialResult(
            frameNumber, request.partialResult.composedResult,
            request.resultExtras);
      }
    }
  }

  shutterTimestamp = request.shutterTS;

  if (result->result != NULL && !isPartialResult) {
    if (request.isMetaPresent) {
      SET_ERR("Called several times with meta for frame %d", frameNumber);
      goto exit;
    }
    if (is_partial_result_supported_ &&
        !request.partialResult.composedResult.isEmpty()) {
      collectedPartialResult.acquire(request.partialResult.composedResult);
    }
    request.isMetaPresent = true;
  }

  numBuffersReturned = result->num_output_buffers;
  request.buffersRemaining -= numBuffersReturned;
  if (NULL != result->input_buffer) {
    request.buffersRemaining--;
  }
  if (request.buffersRemaining < 0) {
    SET_ERR("Too many buffers returned for frame %d", frameNumber);
    goto exit;
  }

  res = find_camera_metadata_ro_entry(result->result, ANDROID_SENSOR_TIMESTAMP,
                                      &entry);
  if ((0 == res) && (entry.count == 1)) {
    request.sensorTS = entry.data.i64[0];
  }

  if ((shutterTimestamp == 0) && (NULL != result->output_buffers) &&
      (0 < result->num_output_buffers)) {
    request.pendingBuffers.appendArray(result->output_buffers,
                                       result->num_output_buffers);
  }

  if (result->result != NULL && !isPartialResult) {
    if (shutterTimestamp == 0) {
      request.pendingMetadata = result->result;
      request.partialResult.composedResult = collectedPartialResult;
    } else {
      CameraMetadata metadata;
      metadata = result->result;
      SendCaptureResult(metadata, request.resultExtras, collectedPartialResult,
                        frameNumber);
    }
  }

  RemovePendingRequestLocked(idx);

  pthread_mutex_unlock(&pending_requests_lock_);

  if (0 < shutterTimestamp) {
    ReturnOutputBuffers(result->output_buffers, result->num_output_buffers,
                        shutterTimestamp, result->frame_number);
  }

  if (NULL != result->input_buffer) {
    StreamBuffer input_buffer;
    memset(&input_buffer, 0, sizeof(input_buffer));
    Camera3InputStream *input_stream =
        static_cast<Camera3InputStream *>(result->input_buffer->stream);
    input_buffer.stream_id = input_stream->stream_id;
    input_buffer.data_space = input_stream->data_space;
    input_buffer.handle = *result->input_buffer->buffer;
    input_stream->return_input_buffer(input_buffer);
  }

  return;

exit:
  pthread_mutex_unlock(&pending_requests_lock_);
}

void Camera3DeviceClient::Notify(const camera3_notify_msg *msg) {
  if (msg == NULL) {
    SET_ERR("HAL sent NULL notify message!");
    return;
  }

  switch (msg->type) {
    case CAMERA3_MSG_SHUTTER: {
      NotifyShutter(msg->message.shutter);
      break;
    }
    case CAMERA3_MSG_ERROR: {
      NotifyError(msg->message.error);
      break;
    }
    default:
      SET_ERR("Unknown notify message from HAL: %d", msg->type);
  }
}

void Camera3DeviceClient::NotifyError(const camera3_error_msg_t &msg) {
  int32_t idx;

  static const CameraErrorCode halErrorMap[CAMERA3_MSG_NUM_ERRORS] = {
      ERROR_CAMERA_INVALID_ERROR, ERROR_CAMERA_DEVICE, ERROR_CAMERA_REQUEST,
      ERROR_CAMERA_RESULT,        ERROR_CAMERA_BUFFER};

  CameraErrorCode errorCode =
      ((msg.error_code >= 0) && (msg.error_code < CAMERA3_MSG_NUM_ERRORS))
          ? halErrorMap[msg.error_code]
          : ERROR_CAMERA_INVALID_ERROR;

  CaptureResultExtras resultExtras;
  switch (errorCode) {
    case ERROR_CAMERA_DEVICE:
      SET_ERR("Camera HAL reported serious device error");
      break;
    case ERROR_CAMERA_REQUEST:
    case ERROR_CAMERA_RESULT:
    case ERROR_CAMERA_BUFFER:
      pthread_mutex_lock(&pending_requests_lock_);
      idx = pending_requests_vector_.indexOfKey(msg.frame_number);
      if (idx >= 0) {
        PendingRequest &r = pending_requests_vector_.editValueAt(idx);
        r.status = msg.error_code;
        resultExtras = r.resultExtras;
      } else {
        resultExtras.frameNumber = msg.frame_number;
        QMMF_ERROR(
            "%s: Camera %d: cannot find pending request for "
            "frame %u error\n",
            __func__, id_, resultExtras.frameNumber);
      }
      pthread_mutex_unlock(&pending_requests_lock_);
      if (nullptr != client_cb_.errorCb) {
        client_cb_.errorCb(errorCode, resultExtras);
      } else {
        QMMF_ERROR("%s: Camera %d: no listener available\n", __func__, id_);
      }
      break;
    default:
      SET_ERR("Unknown error message from HAL: %d", msg.error_code);
      break;
  }
}

void Camera3DeviceClient::NotifyShutter(const camera3_shutter_msg_t &msg) {
  int64_t idx;

  pthread_mutex_lock(&pending_requests_lock_);
  idx = pending_requests_vector_.indexOfKey(msg.frame_number);
  if (idx >= 0) {
    PendingRequest &r = pending_requests_vector_.editValueAt(idx);

    if (nullptr != client_cb_.shutterCb) {
      client_cb_.shutterCb(r.resultExtras, msg.timestamp);
    }

    if (r.resultExtras.input) {
      if (msg.frame_number < next_shutter_input_frame_number_) {
        SET_ERR(
            "Shutter notification out-of-order. Expected "
            "notification for frame %d, got frame %d",
            next_shutter_input_frame_number_, msg.frame_number);
        pthread_mutex_unlock(&pending_requests_lock_);
        return;
      }
      next_shutter_input_frame_number_ = msg.frame_number + 1;
    } else {
      if (msg.frame_number < next_shutter_frame_number_) {
        SET_ERR(
            "Shutter notification out-of-order. Expected "
            "notification for frame %d, got frame %d",
            next_shutter_frame_number_, msg.frame_number);
        pthread_mutex_unlock(&pending_requests_lock_);
        return;
      }
      next_shutter_frame_number_ = msg.frame_number + 1;
    }

    r.shutterTS = msg.timestamp;

    SendCaptureResult(r.pendingMetadata, r.resultExtras,
                      r.partialResult.composedResult, msg.frame_number);
    ReturnOutputBuffers(r.pendingBuffers.array(), r.pendingBuffers.size(),
                        r.shutterTS, msg.frame_number);
    r.pendingBuffers.clear();

    RemovePendingRequestLocked(idx);
  }
  pthread_mutex_unlock(&pending_requests_lock_);

  if (idx < 0) {
    SET_ERR("Shutter notification with invalid frame number %d",
            msg.frame_number);
  }
}

void Camera3DeviceClient::SendCaptureResult(
    CameraMetadata &pendingMetadata, CaptureResultExtras &resultExtras,
    CameraMetadata &collectedPartialResult, uint32_t frameNumber) {
  if (pendingMetadata.isEmpty()) return;

  if (nullptr == client_cb_.resultCb) {
    return;
  }

  if (resultExtras.input) {
    if (frameNumber < next_result_input_frame_number_) {
      SET_ERR(
          "Out-of-order result received! "
          "(arriving frame number %d, expecting %d)",
          frameNumber, next_result_input_frame_number_);
      return;
    }
    next_result_input_frame_number_ = frameNumber + 1;
  } else {
    if (frameNumber < next_result_frame_number_) {
      SET_ERR(
          "Out-of-order result received! "
          "(arriving frame number %d, expecting %d)",
          frameNumber, next_result_frame_number_);
      return;
    }
    next_result_frame_number_ = frameNumber + 1;
  }

  CaptureResult captureResult;
  captureResult.resultExtras = resultExtras;
  captureResult.metadata = pendingMetadata;

  if (captureResult.metadata.update(ANDROID_REQUEST_FRAME_COUNT,
                                    (int32_t *)&frameNumber, 1) != 0) {
    SET_ERR("Unable to update frame number (%d)", frameNumber);
    return;
  }

  if (is_partial_result_supported_ && !collectedPartialResult.isEmpty()) {
    captureResult.metadata.append(collectedPartialResult);
  }

  captureResult.metadata.sort();

  camera_metadata_entry entry =
      captureResult.metadata.find(ANDROID_SENSOR_TIMESTAMP);
  if (entry.count == 0) {
    SET_ERR("No timestamp from Hal for frame %d!", frameNumber);
    return;
  }

  client_cb_.resultCb(captureResult);
}

void Camera3DeviceClient::ReturnOutputBuffers(
    const camera3_stream_buffer_t *outputBuffers, size_t numBuffers,
    int64_t timestamp, int64_t frame_number) {
  for (size_t i = 0; i < numBuffers; i++) {
    Camera3Stream *stream = Camera3Stream::CastTo(outputBuffers[i].stream);
    stream->ReturnBufferToClient(outputBuffers[i], timestamp, frame_number);
  }
}

int32_t Camera3DeviceClient::ReturnStreamBuffer(StreamBuffer buffer) {
  Camera3Stream *stream;
  int32_t streamIdx;
  int32_t res = 0;
  pthread_mutex_lock(&lock_);

  switch (state_) {
    case STATE_ERROR:
      QMMF_ERROR("%s: Device has encountered a serious error\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_INITIALIZED:
    case STATE_CLOSED:
    case STATE_NOT_CONFIGURED:
      QMMF_ERROR("%s: Device is not initialized/configured!\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_CONFIGURED:
    case STATE_RUNNING:
      break;
    default:
      QMMF_ERROR("%s: Unknown state: %d", __func__, state_);
      res = -ENOSYS;
      goto exit;
  }

  streamIdx = streams_.indexOfKey(buffer.stream_id);
  if (streamIdx == -ENOENT) {
    QMMF_ERROR("%s: Stream %d does not exist\n", __func__, buffer.stream_id);
    res = -EINVAL;
    goto exit;
  }

  stream = streams_.editValueAt(streamIdx);
  if (0 != res) {
    QMMF_ERROR("%s: Can't return buffer to its stream: %s (%d)\n", __func__,
               strerror(-res), res);
  }

  res = stream->ReturnBuffer(buffer);
exit:

  pthread_mutex_unlock(&lock_);
  return res;
}

void Camera3DeviceClient::RemovePendingRequestLocked(int idx) {
  const PendingRequest &request = pending_requests_vector_.valueAt(idx);
  const uint32_t frameNumber = pending_requests_vector_.keyAt(idx);

  int64_t sensorTS = request.sensorTS;
  int64_t shutterTS = request.shutterTS;

  if (request.buffersRemaining == 0 &&
      (0 != request.status || (request.isMetaPresent && shutterTS != 0))) {

    if (0 == request.status && sensorTS != shutterTS) {
      SET_ERR(
          "sensor timestamp (%ld) for frame %d doesn't match shutter"
          " timestamp (%ld)\n",
          sensorTS, frameNumber, shutterTS);
    }

    ReturnOutputBuffers(request.pendingBuffers.array(),
                        request.pendingBuffers.size(), 0,
                        frameNumber);

    pending_requests_vector_.removeItemsAt(idx, 1);
  }
}

int32_t Camera3DeviceClient::LoadHWModule(const char *moduleId,
                                          const struct hw_module_t **pHmi) {

  int32_t status;

  if (NULL == moduleId) {
    QMMF_ERROR("%s: Invalid module id! \n", __func__);
    return -EINVAL;
  }

  status = hw_get_module(moduleId, pHmi);

  return status;
}

int32_t Camera3DeviceClient::GetCameraInfo(uint32_t idx, CameraMetadata *info) {
  if (NULL == info) {
    return -EINVAL;
  }

  if (idx >= number_of_cameras_) {
    return -EINVAL;
  }

  if (NULL == camera_module_) {
    return -ENODEV;
  }

  camera_info cam_info;
  int32_t res = camera_module_->get_camera_info(idx, &cam_info);
  if (0 != res) {
    QMMF_ERROR("%s: Error during camera static info query: %s!\n", __func__,
               strerror(res));
    return res;
  }
  *info = cam_info.static_camera_characteristics;

  return res;
}

int32_t Camera3DeviceClient::SubmitRequest(Camera3Request request,
                                           bool streaming,
                                           int64_t *lastFrameNumber) {
  List<Camera3Request> requestList;
  requestList.push_back(request);
  return SubmitRequestList(requestList, streaming, lastFrameNumber);
}

int32_t Camera3DeviceClient::SubmitRequestList(List<Camera3Request> requests,
                                               bool streaming,
                                               int64_t *lastFrameNumber) {
  int32_t res = 0;
  if (requests.empty()) {
    QMMF_ERROR("%s: Camera %d: Received empty!\n", __func__, id_);
    return -EINVAL;
  }

  List<const CameraMetadata> metadataRequestList;
  int32_t requestId = next_request_id_;
  int32_t temp_request_id = requestId;

  pthread_mutex_lock(&lock_);
  current_request_ids_.clear();

  switch (state_) {
    case STATE_ERROR:
      QMMF_ERROR("%s: Device has encountered a serious error\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_INITIALIZED:
    case STATE_CLOSED:
      QMMF_ERROR("%s: Device not initialized\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_CONFIGURED:
    case STATE_CONFIGURED:
    case STATE_RUNNING:
      break;
    default:
      QMMF_ERROR("%s: Unknown state: %d", __func__, state_);
      res = -ENOSYS;
      goto exit;
  }

  for (List<Camera3Request>::iterator it = requests.begin();
       it != requests.end(); ++it) {
    Camera3Request request = *it;
    CameraMetadata metadata(request.metadata);
    if (metadata.isEmpty()) {
      QMMF_ERROR("%s: Camera %d: Received invalid meta.\n", __func__, id_);
      res = -EINVAL;
      goto exit;
    } else if (request.streamIds.isEmpty()) {
      QMMF_ERROR(
          "%s: Camera %d: Requests must have at least one"
          " stream.\n",
          __func__, id_);
      res = -EINVAL;
      goto exit;
    }

    Vector<int32_t> request_stream_id;
    request_stream_id.appendVector(request.streamIds);
    request_stream_id.sort(compare);
    int32_t prev_id = -1;
    int32_t input_stream_idx = -1;
    for (uint32_t i = 0; i < request_stream_id.size(); ++i) {
      if (input_stream_.stream_id == request_stream_id[i]) {
        metadata.update(ANDROID_REQUEST_INPUT_STREAMS,
                        &input_stream_.stream_id, 1);
        input_stream_idx = i;
        continue;
      }

      Camera3Stream *stream = streams_.valueFor(request_stream_id[i]);

      if (NULL == stream) {
        QMMF_ERROR("%s: Camera %d: Request contains invalid stream!\n",
                   __func__, id_);
        res = -EINVAL;
        goto exit;
      }

      if (prev_id == request_stream_id[i]) {
        QMMF_ERROR("%s: Camera %d: Stream with id: %d appears several times in "
            "request!\n", __func__, id_, prev_id);
        res = -EINVAL;
        goto exit;
      } else {
        prev_id = request_stream_id[i];
      }
    }

    if (0 <= input_stream_idx) {
      request_stream_id.removeAt(input_stream_idx);
    }
    metadata.update(ANDROID_REQUEST_OUTPUT_STREAMS, &request_stream_id[0],
                    request_stream_id.size());

    metadata.update(ANDROID_REQUEST_ID, &temp_request_id, 1);
    metadataRequestList.push_back(metadata);
    current_request_ids_.push_back(temp_request_id);
    temp_request_id++;
  }
  next_request_id_ = temp_request_id;

  res = AddRequestListLocked(metadataRequestList, streaming, lastFrameNumber);
  if (0 != res) {
    QMMF_ERROR("%s: Camera %d: Got error %d after trying to set capture\n",
               __func__, id_, res);
  }

  if (0 == res) {
    res = requestId;

    if (streaming) {
      repeating_requests_.push_back(requestId);
    }
  }

exit:
  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::AddRequestListLocked(
    const List<const CameraMetadata> &requests, bool streaming,
    int64_t *lastFrameNumber) {
  RequestList requestList;

  int32_t res = GetRequestListLocked(requests, &requestList);
  if (0 != res) {
    return res;
  }

  if (streaming) {
    res = request_handler_.SetRepeatingRequests(requestList, lastFrameNumber);
  } else {
    res = request_handler_.QueueRequestList(requestList, lastFrameNumber);
  }

  if (0 == res) {
    WaitUntilStateThenRelock(true, WAIT_FOR_RUNNING);
    if (0 != res) {
      SET_ERR_L("Unable to change to running in %f seconds!",
                WAIT_FOR_RUNNING / 1e9);
    }
  } else {
    QMMF_ERROR("%s: Request queue failed: %d\n", __func__, res);
  }

  return res;
}

int32_t Camera3DeviceClient::GetRequestListLocked(
    const List<const CameraMetadata> &metadataList, RequestList *requestList) {
  if (requestList == NULL) {
    QMMF_ERROR("%s: Invalid requestList\n", __func__);
    return -EINVAL;
  }

  int32_t burstId = 0;
  for (List<const CameraMetadata>::const_iterator it = metadataList.begin();
       it != metadataList.end(); ++it) {
    CaptureRequest newRequest;
    int32_t res = GenerateCaptureRequestLocked(*it, newRequest);
    if (0 != res) {
      QMMF_ERROR("%s: Can't create capture request\n", __func__);
      return -EINVAL;
    }

    // Setup burst Id and request Id
    newRequest.resultExtras.burstId = burstId++;
    if (it->exists(ANDROID_REQUEST_ID)) {
      if (it->find(ANDROID_REQUEST_ID).count == 0) {
        QMMF_ERROR("%s: Empty RequestID\n", __func__);
        return -EINVAL;
      }
      newRequest.resultExtras.requestId =
          it->find(ANDROID_REQUEST_ID).data.i32[0];
    } else {
      QMMF_ERROR("%s: RequestID missing\n", __func__);
      return -EINVAL;
    }

    requestList->push_back(newRequest);
  }

  return 0;
}

int32_t Camera3DeviceClient::GenerateCaptureRequestLocked(
    const CameraMetadata &request, CaptureRequest &captureRequest) {
  int32_t res;

  if (state_ == STATE_NOT_CONFIGURED || reconfig_) {
    res = ConfigureStreamsLocked();
    if (res == BAD_VALUE && state_ == STATE_NOT_CONFIGURED) {
      QMMF_ERROR("%s: No streams configured\n", __func__);
      return -EINVAL;
    }
    if (0 != res) {
      QMMF_ERROR("%s: Can't set up streams: %s (%d)\n", __func__,
                 strerror(-res), res);
      return res;
    }
    if (state_ == STATE_NOT_CONFIGURED) {
      QMMF_ERROR("%s: No streams configured\n", __func__);
      return -ENODEV;
    }
  }

  captureRequest.metadata = request;

  camera_metadata_entry_t streams =
      captureRequest.metadata.find(ANDROID_REQUEST_OUTPUT_STREAMS);
  if (streams.count == 0) {
    QMMF_ERROR("%s: Zero output streams specified!\n", __func__);
    return -EINVAL;
  }

  for (uint32_t i = 0; i < streams.count; i++) {
    int idx = streams_.indexOfKey(streams.data.i32[i]);
    if (-ENOENT == idx) {
      QMMF_ERROR("%s: Request references unknown stream %d\n", __func__,
                 streams.data.u8[i]);
      return -EINVAL;
    }
    Camera3Stream *stream = streams_.editValueAt(idx);

    if (stream->IsConfigureActive()) {
      res = stream->EndConfigure();
      if (0 != res) {
        QMMF_ERROR("%s: Stream configuration failed %d: %s (%d)\n", __func__,
                   stream->GetId(), strerror(-res), res);
        return -ENODEV;
      }
    }

    if (stream->IsPrepareActive()) {
      QMMF_ERROR("%s: Request contains a stream that is currently being"
          "prepared!\n", __func__);
        return -ENOSYS;
    }

    captureRequest.streams.push(stream);
  }
  captureRequest.metadata.erase(ANDROID_REQUEST_OUTPUT_STREAMS);

  captureRequest.input = NULL;
  captureRequest.resultExtras.input = false;
  if (captureRequest.metadata.exists(ANDROID_REQUEST_INPUT_STREAMS)) {
    streams =
          captureRequest.metadata.find(ANDROID_REQUEST_INPUT_STREAMS);
    if (1 == streams.count) {
      if (input_stream_.stream_id == streams.data.i32[0]) {
        captureRequest.input = &input_stream_;
        captureRequest.resultExtras.input = true;
      } else {
        QMMF_ERROR("%s: Request contains input stream with id: %d that"
            "doesn't match the registered one: %d\n", __func__,
            streams.data.i32[0], input_stream_.stream_id);
          return -ENOSYS;
      }
    } else {
      QMMF_ERROR("%s: Request contains multiple input streams: %d\n", __func__,
                 streams.count);
        return -ENOSYS;
    }
    captureRequest.metadata.erase(ANDROID_REQUEST_INPUT_STREAMS);
  }

  return 0;
}

void Camera3DeviceClient::SetErrorState(const char *fmt, ...) {
  pthread_mutex_lock(&lock_);
  va_list args;
  va_start(args, fmt);

  SetErrorStateLockedV(fmt, args);

  va_end(args);
  pthread_mutex_unlock(&lock_);
}

void Camera3DeviceClient::SetErrorStateV(const char *fmt, va_list args) {
  pthread_mutex_lock(&lock_);
  SetErrorStateLockedV(fmt, args);
  pthread_mutex_unlock(&lock_);
}

void Camera3DeviceClient::SetErrorStateLocked(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);

  SetErrorStateLockedV(fmt, args);

  va_end(args);
}

void Camera3DeviceClient::SetErrorStateLockedV(const char *fmt, va_list args) {
  String8 errorCause = String8::formatV(fmt, args);
  QMMF_ERROR("%s: Camera %d: %s\n", __func__, id_, errorCause.string());

  if (state_ == STATE_ERROR || state_ == STATE_NOT_INITIALIZED ||
      state_ == STATE_CLOSED)
    return;

  last_error_ = errorCause;

  request_handler_.TogglePause(true);
  InternalUpdateStatusLocked(STATE_ERROR);

  if (nullptr != client_cb_.errorCb) {
    client_cb_.errorCb(ERROR_CAMERA_DEVICE, CaptureResultExtras());
  }
}

void Camera3DeviceClient::NotifyStatus(bool idle) {
  pthread_mutex_lock(&lock_);
  if (state_ != STATE_RUNNING && state_ != STATE_CONFIGURED) {
    pthread_mutex_unlock(&lock_);
    return;
  }
  InternalUpdateStatusLocked(idle ? STATE_CONFIGURED : STATE_RUNNING);

  if (pause_state_notify_) {
    pthread_mutex_unlock(&lock_);
    return;
  }

  pthread_mutex_unlock(&lock_);

  if (idle && nullptr != client_cb_.idleCb) {
    client_cb_.idleCb();
  }
}

int32_t Camera3DeviceClient::Flush(int64_t *lastFrameNumber) {
  int32_t res;
  pthread_mutex_lock(&lock_);

  res = request_handler_.Clear(lastFrameNumber);
  if (0 != res) {
    QMMF_ERROR("%s: Couldn't reset request handler!\n", __func__);
    goto exit;
  }
  // We can't hold locks during Hal call to flush.
  // Some implementations will return buffers to client
  // from the same context and this can cause deadlock
  // if client tries to return them.
  pthread_mutex_unlock(&lock_);

  res = device_->ops->flush(device_);

  pthread_mutex_lock(&lock_);

  if (0 == res) {
    repeating_requests_.clear();
  }

exit:

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::WaitUntilIdle() {
  pthread_mutex_lock(&lock_);
  int32_t res = WaitUntilDrainedLocked();
  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::WaitUntilDrainedLocked() {
  switch (state_) {
    case STATE_NOT_INITIALIZED:
    case STATE_CLOSED:
    case STATE_NOT_CONFIGURED:
      return 0;
    case STATE_CONFIGURED:
    // To avoid race conditions, check with tracker to be sure
    case STATE_ERROR:
    case STATE_RUNNING:
      // Need to verify shut down
      break;
    default:
      SET_ERR_L("Unexpected status: %d", state_);
      return -ENOSYS;
  }

  int32_t res = WaitUntilStateThenRelock(false, WAIT_FOR_SHUTDOWN);
  if (0 != res) {
    SET_ERR_L("Error waiting for HAL to drain: %s (%d)", strerror(-res), res);
  }
  return res;
}

void Camera3DeviceClient::InternalUpdateStatusLocked(State state) {
  state_ = state;
  current_state_updates_.add(state_);
  pthread_cond_broadcast(&state_updated_);
}

int32_t Camera3DeviceClient::InternalPauseAndWaitLocked() {
  request_handler_.TogglePause(true);
  pause_state_notify_ = true;

  int32_t res = WaitUntilStateThenRelock(false, WAIT_FOR_SHUTDOWN);
  if (0 != res) {
    SET_ERR_L("Can't idle device in %f seconds!", WAIT_FOR_SHUTDOWN / 1e9);
  }

  return res;
}

int32_t Camera3DeviceClient::InternalResumeLocked() {
  int32_t res;

  request_handler_.TogglePause(false);

  res = WaitUntilStateThenRelock(true, WAIT_FOR_RUNNING);
  if (0 != res) {
    SET_ERR_L("Can't transition to active in %f seconds!",
              WAIT_FOR_RUNNING / 1e9);
  }

  pause_state_notify_ = false;
  return res;
}

int32_t Camera3DeviceClient::WaitUntilStateThenRelock(bool active,
                                                      int64_t timeout) {
  int32_t res = 0;

  uint32_t startIndex = 0;
  if (state_listeners_ == 0) {
    current_state_updates_.clear();
  } else {
    startIndex = current_state_updates_.size();
  }

  state_listeners_++;

  bool stateSeen = false;
  do {
    if (active == (state_ == STATE_RUNNING)) {
      break;
    }

    res = cond_wait_relative(&state_updated_, &lock_, timeout);
    if (0 != res) {
      break;
    }

    for (uint32_t i = startIndex; i < current_state_updates_.size(); i++) {
      if (active == (current_state_updates_[i] == STATE_RUNNING)) {
        stateSeen = true;
        break;
      }
    }
  } while (!stateSeen);

  state_listeners_--;

  return res;
}

int32_t Camera3DeviceClient::Prepare(int streamId) {
  int32_t res = 0;
  pthread_mutex_lock(&lock_);

  Camera3Stream *stream;
  int32_t outputStreamIdx = streams_.indexOfKey(streamId);
  if (-ENOENT == outputStreamIdx) {
      QMMF_ERROR("%s: Stream %d is invalid!\n", __func__, streamId);
      res = -EINVAL;
  }

  stream = streams_.editValueAt(outputStreamIdx);
  if (stream->IsStreamActive()) {
    QMMF_ERROR("%s: Stream %d has already received requests\n", __func__,
               streamId);
    res = -EINVAL;
    goto exit;
  }

  if (request_handler_.IsStreamActive(*stream)) {
    QMMF_ERROR("%s: Stream %d already has pending requests\n", __func__,
               streamId);
    res = -EINVAL;
    goto exit;
  }

  res = prepare_handler_.Prepare(stream);

exit:

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::TearDown(int streamId) {
  int32_t res = 0;
  pthread_mutex_lock(&lock_);

  Camera3Stream *stream;
  int32_t outputStreamIdx = streams_.indexOfKey(streamId);
  if (-ENOENT == outputStreamIdx) {
      QMMF_ERROR("%s: Stream %d is invalid!\n", __func__, streamId);
      res = -EINVAL;
  }

  stream = streams_.editValueAt(outputStreamIdx);
  if (request_handler_.IsStreamActive(*stream)) {
    QMMF_ERROR("%s: Stream %d already has pending requests\n", __func__,
               streamId);
    res = -EINVAL;
    goto exit;
  }

  res = stream->TearDown();

exit:

  pthread_mutex_unlock(&lock_);

  return res;
}

void Camera3DeviceClient::processCaptureResult(
    const camera3_callback_ops *cb, const camera3_capture_result *result) {
  Camera3DeviceClient *ctx = const_cast<Camera3DeviceClient *>(
      static_cast<const Camera3DeviceClient *>(cb));
  ctx->HandleCaptureResult(result);
}

void Camera3DeviceClient::notifyFromHal(const camera3_callback_ops *cb,
                                        const camera3_notify_msg *msg) {
  Camera3DeviceClient *ctx = const_cast<Camera3DeviceClient *>(
      static_cast<const Camera3DeviceClient *>(cb));
  ctx->Notify(msg);
}

void Camera3DeviceClient::deviceStatusChange(
    const struct camera_module_callbacks *, int camera_id, int new_status) {
  // TODO: No implementation yet
}

void Camera3DeviceClient::torchModeStatusChange(
    const struct camera_module_callbacks *, const char *camera_id,
    int new_status) {
  // TODO: No implementation yet
}

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here
