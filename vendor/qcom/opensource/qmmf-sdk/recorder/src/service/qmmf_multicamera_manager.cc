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

#define LOG_TAG "RecorderMultiCameraManager"

#include <algorithm>
#include <functional>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <cinttypes>
#include <fcntl.h>
#include <dlfcn.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <hardware/hardware.h>
#ifdef ANDROID_O_OR_ABOVE
#include "common/utils/qmmf_common_utils.h"
#else
#include <QCamera3VendorTags.h>
#endif
#include <cutils/properties.h>

#include "recorder/src/service/qmmf_multicamera_manager.h"
#include "recorder/src/service/qmmf_camera_context.h"
#include "recorder/src/service/qmmf_recorder_utils.h"

namespace qmmf {

namespace recorder {

static const char *kSideBySideLib = "libqmmf_alg_side_by_side.so";
static const char *k360StitchLib = "libqmmf_alg_polaris_stitch.so";

using namespace qcamera;

MultiCameraManager::MultiCameraManager()
  : virtual_camera_id_(kVirtualCameraIdOffset),
    start_params_{},
    multicam_type_(MultiCameraConfigType::k360Stitch),
    result_cb_(nullptr),
    error_cb_(nullptr),
    snapshot_param_{0, 0, 0, ImageFormat::kJPEG},
    sequence_cnt_(1),
    jpeg_encoding_enabled_(false),
    snapshot_configured_(false),
    client_snapshot_cb_(nullptr) {}

MultiCameraManager::~MultiCameraManager() {

  QMMF_INFO("%s: Enter", __func__);

  stream_stitch_algos_.clear();
  snapshot_stitch_algo_ = nullptr;

  // Close cameras backwards since first camera is master camera.
  while (!camera_contexts_.empty()) {
    uint32_t cam_id = camera_contexts_.rbegin()->first;
    std::shared_ptr<CameraContext> camera_context = camera_contexts_.rbegin()->second;

    auto ret = camera_context->CloseCamera(cam_id);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: CloseCamera(%d) failed!", __func__, cam_id);
    } else {
      QMMF_INFO("%s Camera %d closed successfully!", __func__, cam_id);
    }
    camera_contexts_.erase(cam_id);
  }

  QMMF_INFO("%s: Exit", __func__);
}

status_t
MultiCameraManager::CreateMultiCamera(const std::vector<uint32_t> camera_ids,
                                      uint32_t* virtual_camera_id) {

  QMMF_INFO("%s: Enter", __func__);
  if (camera_ids.size() < 2) {
    return BAD_VALUE;
  }
  QMMF_INFO("%s Number of camera to be used(%d)", __func__,
      camera_ids.size());

  virtual_camera_map_.emplace(++virtual_camera_id_, camera_ids);
  *virtual_camera_id = virtual_camera_id_;

  QMMF_INFO("%s: Exit", __func__);
  return NO_ERROR;
}

status_t MultiCameraManager::ConfigureMultiCamera(
    const uint32_t virtual_camera_id, const MultiCameraConfigType type,
    const void *param, const size_t param_size) {

  multicam_type_ = type;
  return NO_ERROR;
}

status_t MultiCameraManager::OpenCamera(const uint32_t virtual_camera_id,
                                        const CameraStartParam &param,
                                        const ResultCb &cb,
                                        const ErrorCb &errcb) {

  QMMF_INFO("%s: Enter", __func__);
  status_t ret = NO_ERROR;

  if (virtual_camera_map_.count(virtual_camera_id) == 0) {
    QMMF_ERROR("%s: Invalid virtual camera ID!", __func__);
    return BAD_VALUE;
  }
  auto camera_ids = virtual_camera_map_[virtual_camera_id];

  QMMF_INFO("%s: Total Number of cameras to be open(%d)", __func__,
      camera_ids.size());

  // Status(std::future) from asynchronous tasks for each camera(uint32_t).
  std::vector<std::tuple<uint32_t, std::future<status_t>>> results;

  // Open cameras in separate asynchronous tasks.
  for (auto const& cam_id : camera_ids) {
    std::shared_ptr<CameraContext> context = std::make_shared<CameraContext>();
    camera_contexts_.emplace(cam_id, context);

    ResultCb result_cb = [this] (uint32_t camera_id,
      const CameraMetadata &meta) { ResultCallback(camera_id, meta); };

    auto future = std::async(std::launch::async, &CameraContext::OpenCamera,
                             context.get(), cam_id, param, result_cb, nullptr);
    results.push_back(std::make_tuple(cam_id, std::move(future)));
  }

  start_params_ = param;
  result_cb_    = cb;
  error_cb_     = errcb;

  StitchingBase::InitParams algo_param {};
  algo_param.multicam_id = virtual_camera_id_;
  algo_param.camera_ids  = virtual_camera_map_[virtual_camera_id_];
  algo_param.stitch_mode = multicam_type_;
  algo_param.frame_rate  = 1;

  snapshot_stitch_algo_ =
      std::make_shared<SnapshotStitching>(algo_param, camera_contexts_);
  ret = snapshot_stitch_algo_->Initialize();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to initialize stitching algo!", __func__);
    return ret;
  }

#ifndef DISABLE_PP_JPEG
  jpeg_encoder_ = std::make_shared<CameraJpeg>();
  jpeg_memory_pool_ = std::make_shared<GrallocMemory>();
  ret = jpeg_memory_pool_->Initialize();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Jpeg encoder's memory pool initialization failed!",
        __func__);
    jpeg_memory_pool_ = nullptr;
    jpeg_encoder_ = nullptr;
    return NO_INIT;
  }
#else
  QMMF_WARN("%s: JPEG Postproc not supported.", __func__);
#endif

  // Wait for all asynchronous tasks to complete and return status.
  for (auto& result : results) {
    auto camera_id = std::get<0>(result);
    auto future = std::move(std::get<1>(result));

    if (future.get() != NO_ERROR) {
      QMMF_ERROR("%s: Failed to open camera(%d)!", __func__, camera_id);
      ret |= NO_INIT;
    }
  }

  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t MultiCameraManager::CloseCamera(const uint32_t virtual_camera_id) {

  QMMF_INFO("%s: Enter", __func__);
  status_t ret = NO_ERROR;
  bool closing_failed = false;

  if (virtual_camera_map_.count(virtual_camera_id) == 0) {
    QMMF_ERROR("%s: Invalid virtual camera ID!", __func__);
    return BAD_VALUE;
  }
  QMMF_INFO("%s: Total Number of cameras to be closed(%d)", __func__,
      camera_contexts_.size());

  snapshot_stitch_algo_->RequestExitAndWait();
  snapshot_stitch_algo_ = nullptr;

  // Close cameras backwards since first camera is master camera.
  while (!camera_contexts_.empty()) {
    uint32_t cam_id = camera_contexts_.rbegin()->first;
    std::shared_ptr<CameraContext> camera_context = camera_contexts_.rbegin()->second;

    ret = camera_context->CloseCamera(cam_id);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: CloseCamera(%d) failed!", __func__, cam_id);
      closing_failed = true;
      camera_context = nullptr;
    }
    camera_contexts_.erase(cam_id);
  }

  if (jpeg_encoding_enabled_) {
    jpeg_encoder_->Delete();
    jpeg_encoding_enabled_ = false;
  }
  jpeg_memory_pool_ = nullptr;
  jpeg_encoder_ = nullptr;;

  QMMF_INFO("%s: Exit", __func__);
  return closing_failed ? UNKNOWN_ERROR : NO_ERROR;
}

status_t MultiCameraManager::WaitAecToConverge(const uint32_t timeout) {

  // Since both cameras are in sync we need to wait Aec
  // to converge only on main camera
  status_t ret = camera_contexts_.at(0)->WaitAecToConverge(timeout);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: WaitAecToConverge Failed!", __func__);
    return ret;
  }
  return NO_ERROR;
}

status_t MultiCameraManager::SetUpCapture(const ImageParam &param,
                                          const uint32_t num_images) {

  std::lock_guard<std::mutex> lock(lock_);
  status_t ret = NO_ERROR;

  sequence_cnt_ = num_images;

  bool reconfigure_needed = (snapshot_param_.width != param.width) ||
                            (snapshot_param_.height != param.height) ||
                            !snapshot_configured_;

  ImageParam capture_param = snapshot_param_ = param;
  capture_param.image_format = (param.image_format == ImageFormat::kJPEG) ?
                               ImageFormat::kNV12 : param.image_format;
  if (reconfigure_needed) {
    snapshot_stitch_algo_->RequestExitAndWait();
    jpeg_encoding_enabled_ = (param.image_format == ImageFormat::kJPEG);

    if (jpeg_encoding_enabled_) {
      jpeg_encoder_->Delete();
      ret = CreateJpegEncoder(capture_param);
      if (ret != NO_ERROR) {
        QMMF_ERROR("%s: Failed to create JPEG encoder!", __func__);
        return ret;
      }
    }

    // Set buffer params for stitching.
    GrallocMemory::BufferParams buffer_param {};
    buffer_param.format        = ImageToHalFormat(capture_param.image_format);
    buffer_param.width         = param.width;
    buffer_param.height        = param.height;
    buffer_param.gralloc_flags = GRALLOC_USAGE_SW_WRITE_OFTEN;
    buffer_param.max_buffer_count = SNAPSHOT_STREAM_BUFFER_COUNT;

    QMMF_INFO("%s W(%d) & H(%d)", __func__, buffer_param.width,
        buffer_param.height);
    ret = snapshot_stitch_algo_->Configure(buffer_param);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Failed to configure buffer params!", __func__);
      return ret;
    }
    snapshot_stitch_algo_->Run();
  }

  auto streams = active_streams_;
  if (reconfigure_needed) {
    // Stop all active streams.
    for (auto const& track_id : streams) {
      StopStream(track_id);
    }
  }
  SetDefaultSurfaceDim(capture_param.width, capture_param.height);

  for (auto& camera : camera_contexts_) {
    uint32_t camera_id = camera.first;
    std::shared_ptr<CameraContext> context = camera.second;

    ret = context->SetUpCapture(capture_param, num_images);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: Camera %d: SetUpCapture Failed!", __func__,
          camera_id);
      return ret;
    }
  }

  if (reconfigure_needed) {
    // Resume all previously active streams.
    for (auto const& track_id : streams) {
      StartStream(track_id);
    }
    // Wait avoid capturing black frames
    ret = WaitAecToConverge(kAecConvergeTimeout);
    if (ret != NO_ERROR) {
      QMMF_WARN("%s: AE failed to converge!", __func__);
    }
  }
  snapshot_configured_ = true;

  return NO_ERROR;
}

status_t MultiCameraManager::CaptureImage(const std::vector<CameraMetadata>
                                          &meta, const StreamSnapshotCb& cb) {

  std::lock_guard<std::mutex> lock(lock_);
  status_t ret = NO_ERROR;

  if (start_params_.zsl_mode) {
    QMMF_ERROR("%s: ZSL not supported!", __func__);
    return BAD_VALUE;
  }

  if (jpeg_encoding_enabled_) {
    StreamSnapshotCb encoder_cb = [&] (uint32_t count, StreamBuffer& buffer) {
      OnStitchedFrameAvailable(buffer);
    };
    snapshot_stitch_algo_->SetClientCallback(encoder_cb);
    client_snapshot_cb_ = cb;
  } else {
    snapshot_stitch_algo_->SetClientCallback(cb);
  }

  StreamSnapshotCb stream_cb = [&] (uint32_t count, StreamBuffer& buf) {
    snapshot_stitch_algo_->FrameAvailableCb(count, buf);
  };

  // Always use synchronized request for capture.
  std::vector<CameraMetadata> capture_meta = meta;

  const uint8_t sync_req = 1;
  capture_meta[0].update(QCAMERA3_DUALCAM_SYNCHRONIZED_REQUEST,
                         &sync_req, 1);

  auto camera_ids = virtual_camera_map_[virtual_camera_id_];
  for (size_t idx = 0; idx < camera_ids.size(); ++idx) {
    auto camera_id = camera_ids[idx];

    // Dual camera meta should only be sent only on first capture in burst.
    ret = FillDualCamMetadata(capture_meta.at(0), idx);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: Camera %d: FillDualCamMetadata failed!",
          __func__, camera_id);
      return ret;
    }
    ret = camera_contexts_[camera_id]->CaptureImage(capture_meta, stream_cb);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: Camera %d: CaptureImage failed!", __func__,
          camera_id);
      return ret;
    }
  }
  return NO_ERROR;
}

status_t MultiCameraManager::ConfigImageCapture(const ImageConfigParam &config) {

  thumbnails_.clear();

  for (size_t i = 0; i < config.EntryCount(QMMF_IMAGE_THUMBNAIL); i++) {
    ImageThumbnail thumbnail;
    config.Fetch(QMMF_IMAGE_THUMBNAIL, thumbnail, i);
    thumbnails_.push_back(thumbnail);
  }
  return NO_ERROR;
}

status_t MultiCameraManager::CancelCaptureImage() {

  snapshot_stitch_algo_->RequestExitAndWait();

  {
    // Wait for all currently processed buffers to return.
    std::unique_lock<std::mutex> lock(jpeg_lock_);
    std::chrono::nanoseconds wait_time(kWaitJPEGTimeout);

    while (!jpeg_buffers_map_.empty()) {
      auto ret = wait_for_jpeg_.WaitFor(lock, wait_time);
      if (ret != 0) {
        QMMF_ERROR("%s: Wait for jpeg buffers timed out!", __func__);
        return TIMED_OUT;
      }
    }
  }

  if (snapshot_configured_) {
    auto streams = active_streams_;
    status_t ret = NO_ERROR;

    for (auto const& track_id : streams) {
      ret = StopStream(track_id);
      if (ret != NO_ERROR) {
        QMMF_ERROR("%s: StopStream %d Failed!", __func__, track_id);
        return ret;
      }
    }
  auto camera_ids = virtual_camera_map_[virtual_camera_id_];
  for (auto const& camera_id : camera_ids) {
    auto ret = camera_contexts_[camera_id]->CancelCaptureImage();
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: Camera %d: CancelCaptureImage failed!", __func__,
          camera_id);
      return ret;
    }
  }
    for (auto const& track_id : streams) {
      ret = StartStream(track_id);
      if (ret != NO_ERROR) {
        QMMF_ERROR("%s: StartStream %d Failed!", __func__, track_id);
        return ret;
      }
    }
    snapshot_configured_ = false;
  }
  return NO_ERROR;
}

status_t MultiCameraManager::CreateStream(const CameraStreamParam& param,
                                          const VideoExtraParam& extra_param) {

  SourceSurfaceDesc surface;
  source_surface_.clear();
  surface_crop_.clear();

  auto camera_ids = virtual_camera_map_[virtual_camera_id_];

  if (extra_param.Exists(QMMF_SOURCE_SURFACE_DESCRIPTOR)) {
    // Source surface entry count should be equal to the number of cameras.
    size_t entry_count = extra_param.EntryCount(QMMF_SOURCE_SURFACE_DESCRIPTOR);
    if (entry_count < camera_contexts_.size()) {
      QMMF_ERROR("%s: Not enough QMMF_SOURCE_SURFACE_PARAM entries! "
          "Required entries: %d!",  __func__, camera_contexts_.size());
      return NOT_ENOUGH_DATA;
    } else if (entry_count > camera_contexts_.size()) {
      QMMF_ERROR("%s: QMMF_SOURCE_SURFACE_PARAM entries count exceeds "
          "camera count (%d)!",  __func__, camera_contexts_.size());
      return BAD_INDEX;
    }
    // Fetch source surface dimensions data from the container.
    for (size_t i = 0; i < entry_count; ++i) {
      extra_param.Fetch(QMMF_SOURCE_SURFACE_DESCRIPTOR, surface, i);
      if (source_surface_.find(surface.camera_id) != source_surface_.end()) {
        QMMF_ERROR("%s: Found more than one QMMF_SOURCE_SURFACE_PARAM "
            "entry for camera %d!",  __func__, surface.camera_id);
        return ALREADY_EXISTS;
      }
      source_surface_.emplace(surface.camera_id, surface);
    }
    // Verify that surface dimensions are set for every camera.
    auto camera_ids = virtual_camera_map_[virtual_camera_id_];
    for (auto const& cam_id : camera_ids) {
      if (source_surface_.find(cam_id) == source_surface_.end()) {
        QMMF_ERROR("%s: QMMF_SOURCE_SURFACE_PARAM for camera %d missing!",
            __func__, cam_id);
        return NAME_NOT_FOUND;
      }
    }
  } else {
    surface.width = param.cam_stream_dim.width;
    surface.height = param.cam_stream_dim.height;
    // Fill the source camera surfaces with default values.
    SetDefaultSurfaceDim(surface.width, surface.height);
    for (auto const& cam_id : camera_ids) {
      surface.camera_id = cam_id;
      source_surface_.emplace(cam_id, surface);
    }
  }

  if (extra_param.Exists(QMMF_SURFACE_CROP)) {
    SurfaceCrop crop;
    // Fetch crop rectangle data from the container.
    for (size_t i = 0; i < extra_param.EntryCount(QMMF_SURFACE_CROP); ++i) {
      extra_param.Fetch(QMMF_SURFACE_CROP, crop, i);
      if (surface_crop_.find(crop.camera_id) != surface_crop_.end()) {
        QMMF_ERROR("%s: Found more than one QMMF_SURFACE_CROP entry "
            "for camera %d!",  __func__, crop.camera_id);
        return ALREADY_EXISTS;
      }
      // Verify the camera ID.
      if (camera_contexts_.count(crop.camera_id) == 0) {
        QMMF_ERROR("%s: Camera ID %d for QMMF_SURFACE_CROP entry %d "
            "does not exist!",  __func__, crop.camera_id, i);
        return NAME_NOT_FOUND;
      }
      auto surface = source_surface_.at(crop.camera_id);
      if (crop.width == 0 && crop.height == 0) {
        auto camera_ids = virtual_camera_map_[virtual_camera_id_];
        for (auto const& cam_id : camera_ids) {
          source_surface_.at(cam_id).width = param.cam_stream_dim.width;
          source_surface_.at(cam_id).height = param.cam_stream_dim.height;
        }
      } else if (surface.width < crop.width || surface.height < crop.height) {
        QMMF_ERROR("%s: Invalid QMMF_SURFACE_CROP entry dimensions for "
            "camera %d!",  __func__, crop.camera_id);
        return BAD_VALUE;
      }
      surface_crop_.emplace(crop.camera_id, crop);
    }
  }


  // Stop all active streams.
  auto streams = active_streams_;
  for (auto const& track_id : streams) {
    StopStream(track_id);
  }

  auto ret = CreateStreamStitching(param);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: CreateStreamStitching Failed!", __func__);
    return ret;
  }

  // Start streams in reverse order. This is needed because camera
  // context is caching our streams and streams will be destroyed only
  // when new stream is created, and not on delete stream as expected.
  for (ssize_t ctx_idx = camera_contexts_.size() - 1; ctx_idx >= 0; --ctx_idx) {
    CameraStreamParam stream_param(param);
    auto &camera_surface = source_surface_.at(camera_ids[ctx_idx]);
    stream_param.cam_stream_dim.width = camera_surface.width;
    stream_param.cam_stream_dim.height = camera_surface.height;

    stream_param.wait_aec_mode &= (ctx_idx == 0) ? true : false;

    ret = CreateCameraStream(ctx_idx, stream_param, extra_param);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: Camera %d: CreateCameraStream failed!", __func__,
          camera_ids[ctx_idx]);
      for (size_t idx = ctx_idx + 1; idx < camera_contexts_.size(); ++idx) {
        DeleteCameraStream(idx, param.id);
      }
      DeleteStreamStitching(param.id);
      return ret;
    }
  }

  std::shared_ptr<StreamStitching> stitching_algo = stream_stitch_algos_[param.id];
  assert(stitching_algo.get() != nullptr);

  for (auto& camera_id : camera_ids) {
    sp<IBufferConsumer> consumer = stitching_algo->GetConsumerIntf(camera_id);
    assert(consumer.get() != nullptr);

    ret = camera_contexts_[camera_id]->AddConsumer(param.id, consumer);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: AddConsumer Failed!", __func__);
      return ret;
    }
  }

  // Resume all previously active streams.
  for (auto const& track_id : streams) {
    StartStream(track_id);
  }
  return NO_ERROR;
}

status_t MultiCameraManager::DeleteStream(const uint32_t track_id) {

  status_t ret = NO_ERROR;

  std::shared_ptr<StreamStitching> stitching_algo = stream_stitch_algos_.at(track_id);
  assert(stitching_algo.get() != nullptr);

  for (auto& camera : camera_contexts_) {
    uint32_t camera_id = camera.first;
    std::shared_ptr<CameraContext> context = camera.second;

    sp<IBufferConsumer> consumer = stitching_algo->GetConsumerIntf(camera_id);
    assert(consumer.get() != nullptr);

    ret = context->RemoveConsumer(track_id, consumer);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: RemoveConsumer Failed!", __func__);
      return ret;
    }
  }

  // Delete the streams backwards since first camera is master camera
  // and need to be stopped last.
  for (ssize_t idx = camera_contexts_.size() - 1; idx >= 0; --idx) {
    ret = DeleteCameraStream(idx, track_id);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: DeleteCameraStream Failed!", __func__);
      return ret;
    }
  }

  ret = DeleteStreamStitching(track_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: DeleteStreamStitching failed %d!", __func__, ret);
  }
  return NO_ERROR;
}

status_t MultiCameraManager::AddConsumer(const uint32_t& track_id,
                                         sp<IBufferConsumer>& consumer) {

  std::shared_ptr<StreamStitching> stitching_algo = stream_stitch_algos_.at(track_id);
  assert(stitching_algo.get() != nullptr);

  auto ret = stitching_algo->AddConsumer(consumer);
  assert(ret == NO_ERROR);
  QMMF_INFO("%s: Consumer(%p) added to track_id(%d)", __func__,
      consumer.get(), track_id);

  return NO_ERROR;
}

status_t MultiCameraManager::RemoveConsumer(const uint32_t& track_id,
                                            sp<IBufferConsumer>& consumer) {

  std::shared_ptr<StreamStitching> stitching_algo = stream_stitch_algos_.at(track_id);
  assert(stitching_algo.get() != nullptr);

  auto ret = stitching_algo->RemoveConsumer(consumer);
  assert(ret == NO_ERROR);

  return NO_ERROR;
}

status_t MultiCameraManager::StartStream(const uint32_t track_id) {

  status_t ret = NO_ERROR;

  std::shared_ptr<StreamStitching> stitching_algo = stream_stitch_algos_.at(track_id);
  assert(stitching_algo.get() != nullptr);

  stitching_algo->Run();

  for (auto& camera : camera_contexts_) {
    std::shared_ptr<CameraContext> context = camera.second;

    ret = context->StartStream(track_id);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: StartStream Failed!", __func__);
      return ret;
    }
  }
  if (active_streams_.count(track_id) == 0) {
    active_streams_.emplace(track_id);
  }
  return ret;
}

status_t MultiCameraManager::StopStream(const uint32_t track_id) {

  status_t ret = NO_ERROR;
  std::shared_ptr<StreamStitching> stitching_algo = stream_stitch_algos_.at(track_id);
  assert(stitching_algo.get() != nullptr);

  stitching_algo->RequestExitAndWait();

  // Stop the streams backwards since first camera is master camera
  // and need to be stopped last.
  for (auto i = camera_contexts_.rbegin(); i != camera_contexts_.rend(); ++i) {
    std::shared_ptr<CameraContext> camera_context = i->second;
    assert(camera_context.get() != nullptr);

    ret = camera_context->StopStream(track_id);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: StopStream Failed!", __func__);
      return ret;
    }
  }
  if (active_streams_.count(track_id) != 0) {
    active_streams_.erase(track_id);
  }
  return ret;
}

status_t MultiCameraManager::SetCameraParam(const CameraMetadata &meta) {

  size_t ctx_idx = 0;
  for (auto ctx_it = camera_contexts_.begin();
    ctx_it != camera_contexts_.end(); ++ctx_it, ++ctx_idx) {
    std::shared_ptr<CameraContext> camera_context = ctx_it->second;
    int32_t camera_id = ctx_it->first;

    auto ret = FillDualCamMetadata(const_cast<CameraMetadata&>(meta), ctx_idx);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: Camera %d: FillDualCamMetadata Failed!",
          __func__, camera_id);
      return ret;
    }

    ret = camera_context->SetCameraParam(meta);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: Camera %d: SetCameraParam Failed!", __func__,
          camera_id);
      return ret;
    }
  }
  return NO_ERROR;
}

status_t MultiCameraManager::GetCameraParam(CameraMetadata &meta) {

  std::shared_ptr<CameraContext> camera_context = camera_contexts_.at(0);
  assert(camera_context.get() != nullptr);
  status_t ret = camera_context->GetCameraParam(meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: GetCameraParam Failed!", __func__);
  }
  return ret;
}

status_t MultiCameraManager::GetDefaultCaptureParam(CameraMetadata &meta) {

  std::shared_ptr<CameraContext> camera_context = camera_contexts_.at(0);
  assert(camera_context.get() != nullptr);
  status_t ret = camera_context->GetDefaultCaptureParam(meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: GetDefaultCaptureParam Failed!", __func__);
  }
  return ret;
}

status_t MultiCameraManager::ReturnImageCaptureBuffer(const uint32_t camera_id,
                                                      const int32_t buffer_id) {

  if (virtual_camera_map_.count(camera_id) == 0) {
    QMMF_ERROR("%s: Invalid virtual camera ID!", __func__);
    return BAD_VALUE;
  }

  // Check if the returned buffer is one of the JPEG buffers. If it's not,
  // it's most likely a YUV buffer; return it to the SnapshotStitching class.
  // If buffer_id is invalid, the SnapshotStitching class will take care of it.

  if (jpeg_buffers_map_.find(buffer_id) == jpeg_buffers_map_.end()) {
    status_t ret = snapshot_stitch_algo_->ImageBufferReturned(buffer_id);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Unable to return stitched buffer!", __func__);
      return ret;
    }
  } else {
    // Return the JPEG buffer back to its memory pool.
    ReturnJpegBuffer(buffer_id);
  }
  return NO_ERROR;
}

CameraStartParam& MultiCameraManager::GetCameraStartParam() {

  return start_params_;
}

std::vector<int32_t>& MultiCameraManager::GetSupportedFps() {

  return camera_contexts_.at(0)->GetSupportedFps();
}

void MultiCameraManager::ResultCallback(uint32_t camera_id,
                                        const CameraMetadata &meta) {

  if (camera_id == 0) {
    if (jpeg_encoding_enabled_) {
      jpeg_encoder_->AddResult(&meta);
    }
    if (nullptr != result_cb_) {
      result_cb_(virtual_camera_id_, meta);
    }
  }
}

status_t MultiCameraManager::SetDefaultSurfaceDim(uint32_t& w, uint32_t& h) {

  switch (multicam_type_) {
    case MultiCameraConfigType::k360Stitch:
    case MultiCameraConfigType::kSideBySide:
      // Divide the width of the stitched output on the number of cameras.
      w /= camera_contexts_.size();
      break;
    default:
      QMMF_ERROR("%s: Unsupported MultiCamera mode: 0x%x", __func__,
          multicam_type_);
      return NAME_NOT_FOUND;
  }
  return NO_ERROR;
}

int32_t MultiCameraManager::ImageToHalFormat(const ImageFormat &image) {

  int32_t format;
  switch (image) {
    case ImageFormat::kJPEG:
      format = HAL_PIXEL_FORMAT_BLOB;
      break;
    case ImageFormat::kNV12:
      format = HAL_PIXEL_FORMAT_YCbCr_420_888;
      break;
    case ImageFormat::kBayerRDI8BIT:
      format = HAL_PIXEL_FORMAT_RAW8;
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

status_t MultiCameraManager::CreateJpegEncoder(const ImageParam &param) {

  PostProcParam in {}, out {};
  PostProcCb jpeg_cb =
      [this] (StreamBuffer in_buffer, StreamBuffer out_buffer) -> void
      { OnJpegImageAvailable(in_buffer, out_buffer);};

  in.width = param.width;
  in.height = param.height;
  in.format = ImageToHalFormat(param.image_format);
  out.width = param.width;
  out.height = param.height;
  out.format = ImageToHalFormat(ImageFormat::kJPEG);

  status_t ret = jpeg_encoder_->Create(0, in, out,
                                       start_params_.frame_rate, 1,
                                       param.image_quality, nullptr, jpeg_cb,
                                       nullptr);
  if (ret < NO_ERROR) {
    QMMF_ERROR("%s: Error with creating jpeg encoder: %d\n", __func__, ret);
    return ret;
  }
#ifndef DISABLE_PP_JPEG
  jpeg_encoder_->Configure(thumbnails_);
#endif
  jpeg_encoder_->Start();

  // Set buffer params for jpeg encoding.
  GrallocMemory::BufferParams buffer_param{};
  buffer_param.format           = HAL_PIXEL_FORMAT_BLOB;
  buffer_param.width            = param.width;
  buffer_param.height           = param.height;
  buffer_param.gralloc_flags    = GRALLOC_USAGE_SW_WRITE_OFTEN;
  // TODO: Need to revisit the calculation of max_size.
  //       Width and height need to be extracted from metadata.
  buffer_param.max_size         = (param.width * param.height) * 2;
  buffer_param.max_buffer_count = SNAPSHOT_STREAM_BUFFER_COUNT;
  ret = jpeg_memory_pool_->Configure(buffer_param);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to configure buffer params!", __func__);
    return ret;
  }
  return NO_ERROR;
}

void MultiCameraManager::EncodeJpegImage(const StreamBuffer &buffer) {

  StreamBuffer output_buffer {};

  status_t ret = jpeg_memory_pool_->GetBuffer(output_buffer.handle);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Unable to retrieve gralloc buffer", __func__);
    status_t ret = snapshot_stitch_algo_->ImageBufferReturned(buffer.fd);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Unable to return stitched buffer!", __func__);
    }
    return;
  }

  ret = jpeg_memory_pool_->PopulateMetaInfo(output_buffer.info,
                                            output_buffer.handle);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to populate buffer meta info", __func__);
    status_t ret = snapshot_stitch_algo_->ImageBufferReturned(buffer.fd);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Unable to return stitched buffer!", __func__);
    }
    return;
  }
  const struct private_handle_t *priv_handle =
      static_cast<const private_handle_t *>(output_buffer.handle);
  output_buffer.fd           = priv_handle->fd;
  output_buffer.size         = priv_handle->size;
  output_buffer.frame_number = buffer.frame_number;
  output_buffer.timestamp    = buffer.timestamp;
  output_buffer.camera_id    = buffer.camera_id;

  jpeg_encoder_->AddBuff(buffer, output_buffer);
}

void MultiCameraManager::OnStitchedFrameAvailable(StreamBuffer buffer) {

  EncodeJpegImage(buffer);
}

void MultiCameraManager::OnJpegImageAvailable(StreamBuffer in_buffer,
                                              StreamBuffer out_buffer) {

  // Return input(stitched) buffer back to SnapshotStitching.
  status_t ret = snapshot_stitch_algo_->ImageBufferReturned(in_buffer.fd);
  if (NO_ERROR != ret) {
    // Only send an error notification. Don't return, since there
    // is a valid jpeg image that needs to be returned to client.
    QMMF_ERROR("%s: Unable to return stitch image buffer!", __func__);
  }
  // Map output(encoded) buffer's fd to StreamBuffer. This is needed to
  // return encoded buffers to their owners on ReturnImageCaptureBuffer.
  {
    std::lock_guard<std::mutex> lock(jpeg_lock_);
    jpeg_buffers_map_.emplace(out_buffer.fd, out_buffer);
  }

  // Send the encoded buffer to the client.
  if (client_snapshot_cb_ == nullptr) {
    QMMF_ERROR("%s: Unable to send encoded image buffer to client!",
        __func__);
    ReturnJpegBuffer(out_buffer.fd);
    return;
  }
  client_snapshot_cb_(1, out_buffer);
}

status_t MultiCameraManager::ReturnJpegBuffer(const int32_t buffer_id) {

  StreamBuffer buffer = jpeg_buffers_map_.at(buffer_id);
  QMMF_DEBUG("%s: Post processed buffer(handle %p, fd %d) returned",
      __func__, buffer.handle, buffer.fd);
  status_t ret = jpeg_memory_pool_->ReturnBuffer(buffer.handle);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Unable to return encoded buffer!",
               __func__);
    return ret;
  }
  {
    std::lock_guard<std::mutex> lock(jpeg_lock_);
    jpeg_buffers_map_.erase(buffer_id);
    wait_for_jpeg_.Signal();
  }
  return NO_ERROR;
}

status_t MultiCameraManager::CreateStreamStitching(const CameraStreamParam&
                                                   param) {

  StitchingBase::InitParams algo_param {};
  algo_param.multicam_id  = virtual_camera_id_;
  algo_param.camera_ids   = virtual_camera_map_.at(virtual_camera_id_);
  algo_param.stitch_mode  = multicam_type_;
  algo_param.surface_crop = surface_crop_;
  algo_param.frame_rate   = param.frame_rate;

  GrallocMemory::BufferParams buffer_param {};
  if (param.cam_stream_format != CameraStreamFormat::kRAW10) {
    buffer_param.format      = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  } else {
    buffer_param.format      = HAL_PIXEL_FORMAT_RAW10;
  }
  buffer_param.width         = param.cam_stream_dim.width;
  buffer_param.height        = param.cam_stream_dim.height;
  buffer_param.gralloc_flags = GRALLOC_USAGE_SW_WRITE_OFTEN;
  buffer_param.max_size      = 0;

  buffer_param.max_buffer_count = VIDEO_STREAM_BUFFER_COUNT;
  if (param.cam_stream_dim.width == kWidth4K &&
      param.cam_stream_dim.height == kHeight4K) {
    buffer_param.max_buffer_count += EXTRA_DCVS_BUFFERS;
  }
  buffer_param.gralloc_flags |= private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;

  std::shared_ptr<StreamStitching> stitching_algo =
      std::make_shared<StreamStitching>(algo_param);
  auto ret = stitching_algo->Initialize();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to initialize stitching algo!", __func__);
    return ret;
  }
  ret = stitching_algo->Configure(buffer_param);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to configure stitching algo!", __func__);
    return ret;
  }
  stream_stitch_algos_.emplace(param.id, stitching_algo);

  return NO_ERROR;
}

status_t MultiCameraManager::DeleteStreamStitching(const uint32_t id) {

  if (stream_stitch_algos_.count(id) == 0) {
    QMMF_ERROR("%s: Stitching algo not present for track id %d",
               __func__, id);
    return BAD_VALUE ;
  }
  stream_stitch_algos_.erase(id);

  return NO_ERROR;
}

status_t MultiCameraManager::CreateCameraStream(const uint32_t& cam_idx, const
                                                CameraStreamParam& param, const
                                                VideoExtraParam& extra_param) {

  auto camera_id = virtual_camera_map_[virtual_camera_id_].at(cam_idx);
  std::shared_ptr<CameraContext> camera_context = camera_contexts_[camera_id];

  // On CreateStream, camera context most probably will
  // reconfigure the camera. So make sure that every time.
  // after reconfiguration we are linking the related cameras.
  status_t ret = camera_context->CreateStream(param, extra_param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Camera %d: CreateStream Failed!", __func__,
        camera_id);
    return ret;
  }

  CameraMetadata meta;
  ret = camera_context->GetCameraParam(meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Camera %d: GetCameraParam Failed!", __func__,
        camera_id);
    return ret;
  }
  ret = FillDualCamMetadata(meta, cam_idx);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Camera %d: FillDualCamMetadata Failed!",
        __func__, camera_id);
    return ret;
  }

  ret = camera_context->SetCameraParam(meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Camera %d: SetCameraParam Failed!", __func__,
        camera_id);
    return ret;
  }

  return NO_ERROR;
}

status_t MultiCameraManager::DeleteCameraStream(const uint32_t& cam_idx,
                                                const uint32_t& track_id) {

  auto camera_id = virtual_camera_map_[virtual_camera_id_].at(cam_idx);
  status_t ret = camera_contexts_[camera_id]->DeleteStream(track_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Camera %d: DeleteStream Failed!", __func__,
        camera_id);
    return ret;
  }
  return NO_ERROR;
}

status_t MultiCameraManager::FillDualCamMetadata(CameraMetadata& meta,
                                                 const uint32_t& cam_idx) {

  if (cam_idx >= camera_contexts_.size()) {
    QMMF_ERROR("%s: Invalid camera index %d number of cameras %d!",
        __func__, cam_idx, camera_contexts_.size());
    return BAD_VALUE;
  }

  if (camera_contexts_.size() < 2) {
      QMMF_INFO("%s: No need to link one camera skip!", __func__);
      return NO_ERROR;
  }

  // If we don't have even cameras to link don't link the last camera
  if ((cam_idx == camera_contexts_.size() - 1) &&
      (camera_contexts_.size() & 1)) {
    QMMF_WARN("%s: Last camera id %d will not be linked, No pair!",
        __func__, camera_contexts_.end()->first);
    return NO_ERROR;
  }

  int32_t related_id;
  uint8_t is_main;

  // Link First with Second, Second with first etc...
  if (cam_idx & 1) {
    related_id = virtual_camera_map_[virtual_camera_id_].at(cam_idx - 1);
    is_main = 0;
  } else {
    related_id = virtual_camera_map_[virtual_camera_id_].at(cam_idx + 1);
    is_main = 1;
  }

  meta.update(QCAMERA3_DUALCAM_LINK_IS_MAIN, &is_main, 1);
  meta.update(QCAMERA3_DUALCAM_LINK_RELATED_CAMERA_ID, &related_id, 1);

  uint8_t sync = 1;
  meta.update(QCAMERA3_DUALCAM_LINK_ENABLE, &sync, 1);

  uint8_t role = QCAMERA3_DUALCAM_LINK_CAMERA_ROLE_BAYER;
  meta.update(QCAMERA3_DUALCAM_LINK_CAMERA_ROLE, &role, 1);

  uint8_t sync_mode = QCAMERA3_DUALCAM_LINK_3A_360_CAMERA;
  meta.update(QCAMERA3_DUALCAM_LINK_3A_SYNC_MODE, &sync_mode, 1);

  return NO_ERROR;
}

SnapshotStitching::SnapshotStitching(
    InitParams &param,
    std::map<uint32_t, std::shared_ptr<CameraContext> > &contexts)
    : StitchingBase(param),
      client_snapshot_cb_(nullptr) {

  QMMF_INFO("%s: Enter", __func__);
  work_thread_name_ = "SnapshotStitching";
  camera_contexts_ = contexts;
  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

SnapshotStitching::~SnapshotStitching() {

  QMMF_INFO("%s: Enter", __func__);
  camera_contexts_.clear();
  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

void SnapshotStitching::FrameAvailableCb(uint32_t count,
                                         StreamBuffer &buffer) {

  std::lock_guard<std::mutex> lock(frame_lock_);
  QMMF_DEBUG("%s: Camera %u: Snapshot Frame %d is available",
      __func__, buffer.camera_id, buffer.frame_number);

  // Handling input buffers from camera contexts.
  if (stop_frame_sync_) {
    ReturnBufferToCamera(buffer);
  } else {
    FrameSync(buffer);
  }
}

status_t SnapshotStitching::ImageBufferReturned(const int32_t buffer_id) {

  std::lock_guard<std::mutex> lock(snapshot_lock_);
  if (snapshot_buffer_list_.count(buffer_id) == 0) {
    QMMF_ERROR("%s: buffer_id(%u) is not valid!", __func__, buffer_id);
    return BAD_VALUE;
  }

  StreamBuffer buffer = snapshot_buffer_list_[buffer_id];
  QMMF_DEBUG("%s: Image capture buffer(handle %p, fd %d) returned",
      __func__, buffer.handle, buffer.fd);

  // Handling return buffer from camera source.
  status_t ret = ReturnBufferToBufferPool(buffer);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Unable to return image buffer!", __func__);
    return ret;
  }
  snapshot_buffer_list_.erase(buffer_id);
  return ret;
}

status_t SnapshotStitching::NotifyBufferToClient(StreamBuffer &buffer) {

  status_t ret = NO_ERROR;
  if(nullptr != client_snapshot_cb_) {
    {
      std::lock_guard<std::mutex> lock(snapshot_lock_);
      snapshot_buffer_list_.emplace(buffer.fd, buffer);
    }
    client_snapshot_cb_(1, buffer);
  } else {
    QMMF_VERBOSE("%s: No client callback, simply return buffer back to"
        " memory pool!",  __func__);
    ret = ReturnBufferToBufferPool(buffer);
  }
  return ret;
}

status_t SnapshotStitching::ReturnBufferToCamera(StreamBuffer &buffer) {

  status_t ret = NO_ERROR;
  if (camera_contexts_.count(buffer.camera_id) == 0) {
    QMMF_ERROR("%s: Invalid camera ID(%d)", __func__, buffer.camera_id);
    return BAD_VALUE;
  }

  std::shared_ptr<CameraContext> camera = camera_contexts_[buffer.camera_id];

  ret = camera->ReturnImageCaptureBuffer(buffer.camera_id, buffer.fd);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to return buffer to camera(%d)",
        __func__, buffer.camera_id);
  }
  camera = nullptr;
  return ret;
}

StreamStitching::StreamStitching(InitParams &param)
    : StitchingBase(param) {

  QMMF_INFO("%s: Enter", __func__);

  use_frame_sync_timeout = true;
  work_thread_name_ = "StreamStitching";

  // Create consumers for the physical cameras.
  for (auto const& camera_id : params_.camera_ids) {
    BufferConsumerImpl<StreamStitching> *impl;
    impl = new BufferConsumerImpl<StreamStitching>(this);
    camera_consumers_map_.emplace(camera_id, impl);
  }

  BufferProducerImpl<StreamStitching> *producer_impl;
  producer_impl = new BufferProducerImpl<StreamStitching>(this);
  buffer_producer_impl_ = producer_impl;

  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

StreamStitching::~StreamStitching() {

  QMMF_INFO("%s: Enter", __func__);
  buffer_producer_impl_.clear();
  camera_consumers_map_.clear();
  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

status_t StreamStitching::AddConsumer(const sp<IBufferConsumer>& consumer) {

  std::lock_guard<std::mutex> lock(consumer_lock_);
  if (consumer.get() == nullptr) {
    QMMF_ERROR("%s: Input consumer is NULL", __func__);
    return BAD_VALUE;
  }

  if (IsConnected(consumer)) {
    QMMF_ERROR("%s: consumer(%p) already added to the producer!",
        __func__, consumer.get());
    return ALREADY_EXISTS;
  }

  buffer_producer_impl_->AddConsumer(consumer);
  consumer->SetProducerHandle(buffer_producer_impl_);
  QMMF_DEBUG("%s: Consumer(%p) has been added."
      " Total number of consumers = %d",  __func__, consumer.get(),
      buffer_producer_impl_->GetNumConsumer());

  stitching_consumers_.emplace(reinterpret_cast<uintptr_t>(consumer.get()),
                               consumer);
  return NO_ERROR;
}

status_t StreamStitching::RemoveConsumer(sp<IBufferConsumer>& consumer) {

  std::lock_guard<std::mutex> lock(consumer_lock_);
  if (consumer.get() == nullptr) {
    QMMF_ERROR("%s: Input consumer is NULL", __func__);
    return BAD_VALUE;
  }

  if(buffer_producer_impl_->GetNumConsumer() == 0) {
    QMMF_ERROR("%s: There are no connected consumers!", __func__);
    return INVALID_OPERATION;
  }

  if (!IsConnected(consumer)) {
    QMMF_ERROR("%s: consumer(%p) is not connected to this port(%p)!",
        __func__, consumer.get(), this);
    return BAD_VALUE;
  }

  buffer_producer_impl_->RemoveConsumer(consumer);
  QMMF_DEBUG("%s: Consumer(%p) has been removed."
      "Total number of consumer = %d",  __func__, consumer.get(),
      buffer_producer_impl_->GetNumConsumer());

  stitching_consumers_.erase(reinterpret_cast<uintptr_t>(consumer.get()));
  return NO_ERROR;
}

sp<IBufferConsumer>& StreamStitching::GetConsumerIntf(uint32_t camera_id) {

  return camera_consumers_map_.at(camera_id);
}

void StreamStitching::OnFrameAvailable(StreamBuffer& buffer) {

  std::lock_guard<std::mutex> lock(frame_lock_);
  QMMF_VERBOSE("%s: Camera %u: Frame %u is available",
      __func__, buffer.camera_id, buffer.frame_number);

  if (stop_frame_sync_ || (single_camera_mode_ &&
      buffer.camera_id == skip_camera_id_)) {
    ReturnBufferToCamera(buffer);
  } else if (single_camera_mode_ && (buffer.camera_id != skip_camera_id_)) {
    NotifyBufferToClient(buffer);
  } else {
    FrameSync(buffer);
  }
}

void StreamStitching::NotifyBufferReturned(const StreamBuffer& buffer) {

  QMMF_VERBOSE("%s: Stream buffer(handle %p) returned", __func__,
      buffer.handle);
  if (buffer.camera_id == params_.multicam_id) {
    ReturnBufferToBufferPool(buffer);
  } else {
    ReturnBufferToCamera(const_cast<StreamBuffer&>(buffer));
  }
}

status_t StreamStitching::NotifyBufferToClient(StreamBuffer &buffer) {

  status_t ret = NO_ERROR;
  if(buffer_producer_impl_->GetNumConsumer() > 0) {
    buffer_producer_impl_->NotifyBuffer(buffer);
  } else {
    QMMF_VERBOSE("%s: No consumer, simply return buffer back to"
        " memory pool!",  __func__);
    ret = ReturnBufferToBufferPool(buffer);
  }
  return ret;
}

status_t StreamStitching::ReturnBufferToCamera(StreamBuffer &buffer) {

  const sp<IBufferConsumer> consumer = GetConsumerIntf(buffer.camera_id);
  if (consumer.get() == nullptr) {
    QMMF_ERROR("%s: Failed to retrieve buffer consumer for camera(%d)!",
               __func__, buffer.camera_id);
    return BAD_VALUE;
  }
  consumer->GetProducerHandle()->NotifyBufferReturned(buffer);
  return NO_ERROR;
}

bool StreamStitching::IsConnected(const sp<IBufferConsumer>& consumer) {

  uintptr_t key = reinterpret_cast<uintptr_t>(consumer.get());
  if (stitching_consumers_.count(key) != 0) {
    return true;
  }
  return false;
}

StitchingBase::StitchingBase(InitParams &param)
    : params_(param),
      stop_frame_sync_(false),
      use_frame_sync_timeout(false),
      skip_camera_id_ (0),
      single_camera_mode_(false) {

  QMMF_INFO("%s: Enter", __func__);
  stitch_lib_ = {};

  // Initialize the buffer map with unsynchronized buffers.
  for (auto const& camera_id : params_.camera_ids) {
    std::vector<StreamBuffer> empty_buffers;
    unsynced_buffer_map_.emplace(camera_id, empty_buffers);
  }

  for (auto const& cam_id : params_.camera_ids) {
    if (params_.surface_crop.find(cam_id) != params_.surface_crop.end()) {
      auto crop = params_.surface_crop.at(cam_id);
      if (crop.width == 0 && crop.height == 0) {
        skip_camera_id_ = cam_id;
        single_camera_mode_ = true;
      }
    }
  }

  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

StitchingBase::~StitchingBase() {

  QMMF_INFO("%s: Enter", __func__);

  RequestExitAndWait();
  DeInitLibrary();

  unsynced_buffer_map_.clear();
  process_buffers_map_.clear();
  registered_buffers_.clear();
  memory_pool_ = nullptr;

  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}


status_t StitchingBase::Initialize() {

  if (nullptr != memory_pool_.get()) {
    QMMF_WARN("%s: Memory pool already initialized", __func__);
    return NO_ERROR;
  }
  memory_pool_ = std::make_shared<GrallocMemory>();

  auto ret = memory_pool_->Initialize();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Unable to create memory pool!", __func__);
    return ret;
  }

  init_library_status_ = std::async(
      std::launch::async, &StitchingBase::InitLibrary, this
  );

  return NO_ERROR;
}

status_t StitchingBase::Configure(GrallocMemory::BufferParams &param) {

  return memory_pool_->Configure(param);
}

int32_t StitchingBase::Run() {

  std::lock_guard<std::mutex> lock(frame_lock_);
  stop_frame_sync_ = false;
  return Camera3Thread::Run(work_thread_name_.c_str());
}

void StitchingBase::RequestExitAndWait() {

  std::lock_guard<std::mutex> lock(frame_lock_);
  status_t ret = StopFrameSync();
  assert(NO_ERROR == ret);
}

bool StitchingBase::ThreadLoop() {

  std::vector<StreamBuffer> input_buffers, output_buffers;
  {
    // If there aren't any pending synchronized buffers waiting to go through
    // stitch processing, wait until such buffer becomes available.
    std::unique_lock<std::mutex> lock(sync_lock_);
    std::chrono::nanoseconds wait_time(kFrameSyncTimeout);

    while (synced_buffer_queue_.empty() && !stop_frame_sync_) {
      if (use_frame_sync_timeout) {
        auto ret = wait_for_sync_frames_.WaitFor(lock, wait_time);
        if (ret != 0) {
          QMMF_DEBUG("%s: Wait for frame available timed out", __func__);
        }
      } else {
        wait_for_sync_frames_.Wait(lock);
      }
    }
    // Exit from thread loop if frame sync is stopped
    if (stop_frame_sync_) return false;

    for (auto const& id : params_.camera_ids) {
      input_buffers.push_back(synced_buffer_queue_.front().at(id));
    }
    synced_buffer_queue_.pop();
  }

  if (!stitch_lib_.initialized) {
    if (init_library_status_.get() != NO_ERROR) {
      QMMF_ERROR("%s: Failed to load algorithm library!", __func__);
      return false;
    }
    stitch_lib_.initialized = true;
  }

  // TODO: add some logic for more than 1 output buffer
  StreamBuffer b {};

  auto ret = memory_pool_->GetBuffer(b.handle);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Unable to retrieve gralloc buffer", __func__);
    return true;
  }
  ret = memory_pool_->PopulateMetaInfo(b.info, b.handle);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to populate buffer meta info", __func__);
    return ret;
  }
  const struct private_handle_t *priv_handle =
      static_cast<const private_handle_t *>(b.handle);
  b.fd           = priv_handle->fd;
  b.size         = priv_handle->size;
  b.frame_number = input_buffers[0].frame_number;
  b.timestamp    = input_buffers[0].timestamp;
  b.camera_id    = params_.multicam_id;
  output_buffers.push_back(b);

  if (!stitch_lib_.configured) {
    ret = Configlibrary(input_buffers, output_buffers);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Failed to configure library", __func__);
      DeInitLibrary();
      return true;
    }
    stitch_lib_.configured = true;
  }

  {
    std::lock_guard<std::mutex> lock(buffers_lock_);
    for (auto const& buffer : input_buffers) {
      std::pair<buffer_handle_t, StreamBuffer> pair (buffer.handle, buffer);
      process_buffers_map_.insert(pair);
    }
    for (auto const& buffer : output_buffers) {
      std::pair<buffer_handle_t, StreamBuffer> pair (buffer.handle, buffer);
      process_buffers_map_.insert(pair);
    }
  }

  ret = ProcessBuffers(input_buffers, output_buffers);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to process images", __func__);
  }

  return true;
}

status_t StitchingBase::FrameSync(StreamBuffer& buffer) {

  bool match_found;
  int32_t timestamp_delta;
  uint32_t num_matched_frames = 1;
  std::vector<StreamBuffer> *unsynced_buffers;
  // Map of camera id and index of the matched buffer from
  // the unsynced_buffers queue for that camera id.
  std::map<uint32_t, uint32_t> matched_buffers;

  // Each matched buffer for given camera will be added to the
  // synced_frames vector and identified by it's camera id.
  std::map<uint32_t, StreamBuffer> synced_frames;
  synced_frames.emplace(buffer.camera_id, buffer);

  // Iterate through the unsynced buffers for each camera, except current one.
  for (auto const& camera_id : params_.camera_ids) {
    if (camera_id == buffer.camera_id) {
      continue;
    }
    match_found = false;

    // Retrieve a list with unsynced buffers for each of the other cameras.
    unsynced_buffers = &unsynced_buffer_map_.at(camera_id);

    // Backward search, as the latest buffers are at the back.
    for (int32_t idx = (unsynced_buffers->size() - 1); idx >= 0; --idx) {
      const StreamBuffer &unsynced_frame = unsynced_buffers->at(idx);
      timestamp_delta = buffer.timestamp - unsynced_frame.timestamp;

      if (std::abs(timestamp_delta) < kMaxTimestampDelta) {
        synced_frames.emplace(camera_id, unsynced_frame);
        matched_buffers.emplace(camera_id, idx);
        ++num_matched_frames;
        match_found = true;
        break;
      } else if (timestamp_delta > 0) {
        // No need to check the rest of the buffers in the queue for
        // this camera_id, as they will be with a lower timestamp.
        break;
      }
    }
    // If a matched frame wasn't found there is no need to check
    // all other remaining cameras (if any).
    if (!match_found) {
      break;
    }
  }

  // Matched number of frames is not the same as the number of cameras.
  if (num_matched_frames != params_.camera_ids.size()) {
    QMMF_DEBUG("%s: Camera %u: No matching buffers found", __func__,
        buffer.camera_id);

    // Push the buffer in the unsynced buffer queue for its camera id.
    unsynced_buffer_map_.at(buffer.camera_id).push_back(buffer);

    // Check if the queue of current buffer camera_id has reached max size.
    unsynced_buffers = &unsynced_buffer_map_.at(buffer.camera_id);
    int32_t excess_buffers = unsynced_buffers->size() - kUnsyncedQueueMaxSize;

    if (excess_buffers > 0) {
      QMMF_DEBUG("%s: Camera %u: Unsynced buffer queue reached max "
          "size: %d",  __func__, buffer.camera_id, kUnsyncedQueueMaxSize);

      // Remove older excess buffers from the queue.
      for (int32_t i = 0; i < excess_buffers; ++i) {
        StreamBuffer &buf = unsynced_buffers->at(i);
        ReturnBufferToCamera(buf);
      }
      unsynced_buffers->erase(unsynced_buffers->begin(),
          unsynced_buffers->begin() + excess_buffers);
    }
    return FAILED_TRANSACTION;
  }

  // A matched frame(s) have been found, return all unsynced buffers and clear
  // the queue of the camera_id from which the synchronization buffer came.
  ReturnUnsyncedBuffers(buffer.camera_id);

  // Clear the obsolete unsynced buffers from queue of the matched cameras,
  // starting from beginning to the latest matched buffer and return them
  // back to their corresponding producers.
  for (auto& it : matched_buffers) {
    uint32_t camera_id = it.first;
    uint32_t match_idx = it.second;

    unsynced_buffers = &unsynced_buffer_map_.at(camera_id);
    unsynced_buffers->erase(unsynced_buffers->begin() + match_idx);
    for (uint32_t i = 0; i < match_idx; ++i) {
      StreamBuffer &buf = unsynced_buffers->at(i);
      ReturnBufferToCamera(buf);
    }
    unsynced_buffers->erase(unsynced_buffers->begin(),
        unsynced_buffers->begin() + match_idx);
  }

  std::lock_guard<std::mutex> lock(sync_lock_);
  synced_buffer_queue_.push(synced_frames);
  wait_for_sync_frames_.Signal();

  return NO_ERROR;
}

status_t StitchingBase::ReturnBufferToBufferPool(const StreamBuffer &buffer) {

  status_t ret = memory_pool_->ReturnBuffer(buffer.handle);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to return buffer to memory pool", __func__);
  }
  return ret;
}

status_t StitchingBase::StopFrameSync() {

  status_t ret = NO_ERROR;
  {
    //First signal the thread to not wait on frames
    std::lock_guard<std::mutex> lock(sync_lock_);
    if (stop_frame_sync_ == true) {
      QMMF_DEBUG("%s: Worker thread is already in stopped state",
          __func__);
      return ret;
    }
    stop_frame_sync_ = true;
    wait_for_sync_frames_.Signal();
  }
  // We need to wait thread to exit to avoid ace between
  // flush and ongoing processing in the thread
  Camera3Thread::RequestExitAndWait();

  // Return all unsynced buffers back to the camera contexts.
  for (auto const& camera_id : params_.camera_ids) {
    ret = ReturnUnsyncedBuffers(camera_id);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: Failed to return some of the unsynchronized buffers"
          " for camera %d!",  __func__, camera_id);
    }
  }
  {
    // Return all synced but unconsumed buffers back to the camera contexts.
    std::lock_guard<std::mutex> lock(sync_lock_);

    while (!synced_buffer_queue_.empty()) {
      for (auto const& id : params_.camera_ids) {
        StreamBuffer &buffer = synced_buffer_queue_.front().at(id);
        if (ReturnBufferToCamera(buffer) != NO_ERROR) {
          QMMF_ERROR("%s: Failed to return buffer %p for camera %d",
              __func__, buffer.handle, id);
        }
      }
      synced_buffer_queue_.pop();
    }
  }
  // Flush all pending buffers from the library.
  FlushLibrary();

  // Wait for all currently processed buffers to return.
  std::unique_lock<std::mutex> lock(buffers_lock_);
  std::chrono::nanoseconds wait_time(kWaitBuffersTimeout);

  while (!process_buffers_map_.empty()) {
    auto ret = wait_for_buffers_.WaitFor(lock, wait_time);
    if (ret != 0) {
      QMMF_ERROR("%s: Wait for processed buffers timed out", __func__);
      return TIMED_OUT;
    }
  }
  return NO_ERROR;
}

status_t StitchingBase::ReturnProcessedBuffer(buffer_handle_t &handle,
                                              qmmf_alg_status_t status) {

  status_t ret = NO_ERROR;
  std::lock_guard<std::mutex> lock(buffers_lock_);
  if (process_buffers_map_.find(handle) == process_buffers_map_.end()) {
    QMMF_ERROR("%s: Buffer %p not registered", __func__, handle);
    return BAD_VALUE;
  }

  StreamBuffer &buffer = process_buffers_map_.at(handle);
  QMMF_DEBUG("%s: Got buffer(%p), camera id %d", __func__, handle,
      buffer.camera_id);

  if (buffer.camera_id == params_.multicam_id) {
    if (QMMF_ALG_SUCCESS == status) {
      ret = NotifyBufferToClient(buffer);
    } else {
      ret = ReturnBufferToBufferPool(buffer);
    }
  } else {
    ret = ReturnBufferToCamera(buffer);
  }
  process_buffers_map_.erase(handle);
  wait_for_buffers_.Signal();

  return ret;
}

status_t StitchingBase::ReturnUnsyncedBuffers(uint32_t camera_id) {

  std::vector<StreamBuffer> &buffers =
      unsynced_buffer_map_.at(camera_id);

  status_t ret = NO_ERROR;
  while (!buffers.empty()) {
    StreamBuffer &buf = buffers.back();
    ret = ReturnBufferToCamera(buf);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Failed to return buffer %p for camera %d",
          __func__, buf.handle, camera_id);
      return ret;
    }
    buffers.pop_back();
  }
  return ret;
}

status_t StitchingBase::InitLibrary() {

  status_t ret = NO_ERROR;

  if (nullptr != stitch_lib_.handle) {
    QMMF_WARN("%s: Stitch library already initialized", __func__);
    return ret;
  }

  std::string lib_name;
  switch (params_.stitch_mode) {
    case MultiCameraConfigType::k360Stitch:
      lib_name.append(k360StitchLib);
      break;
    case MultiCameraConfigType::kSideBySide:
      lib_name.append(kSideBySideLib);
      break;
    default:
      QMMF_ERROR("%s MultiCamera type (%d) is not supported!",
          __func__, params_.stitch_mode);
      return BAD_VALUE;
  }

  void* handle = dlopen(lib_name.c_str(), RTLD_NOW);
  if (nullptr == handle) {
    QMMF_ERROR("%s: Failed to open %s, error: %s", __func__,
        lib_name.c_str(), dlerror());
    return BAD_VALUE;
  }

  stitch_lib_.handle = handle;

  *(void **) &stitch_lib_.init       = dlsym(handle, "qmmf_alg_init");
  *(void **) &stitch_lib_.deinit     = dlsym(handle, "qmmf_alg_deinit");
  *(void **) &stitch_lib_.get_caps   = dlsym(handle, "qmmf_alg_get_caps");
  *(void **) &stitch_lib_.set_tuning = dlsym(handle, "qmmf_alg_set_tuning");
  *(void **) &stitch_lib_.config     = dlsym(handle, "qmmf_alg_config");
  *(void **) &stitch_lib_.flush      = dlsym(handle, "qmmf_alg_flush");
  *(void **) &stitch_lib_.process    = dlsym(handle, "qmmf_alg_process");
  *(void **) &stitch_lib_.register_bufs =
      dlsym(handle, "qmmf_alg_register_bufs");
  *(void **) &stitch_lib_.unregister_bufs =
      dlsym(handle, "qmmf_alg_unregister_bufs");
  *(void **) &stitch_lib_.get_debug_info_log =
      dlsym(handle, "qmmf_alg_get_debug_info_log");

  if (!stitch_lib_.init || !stitch_lib_.deinit || !stitch_lib_.get_caps ||
      !stitch_lib_.set_tuning || !stitch_lib_.get_debug_info_log ||
      !stitch_lib_.register_bufs || !stitch_lib_.unregister_bufs ||
      !stitch_lib_.flush || !stitch_lib_.process || !stitch_lib_.config) {
    QMMF_ERROR("%s: Unable to link all symbols", __func__);
    QMMF_ERROR("%s: qmmf_alg_init %p", __func__, stitch_lib_.init);
    QMMF_ERROR("%s: qmmf_alg_deinit %p", __func__, stitch_lib_.deinit);
    QMMF_ERROR("%s: qmmf_alg_get_caps %p", __func__,
        stitch_lib_.get_caps);
    QMMF_ERROR("%s: qmmf_alg_set_tuning %p", __func__,
        stitch_lib_.set_tuning);
    QMMF_ERROR("%s: qmmf_alg_config %p", __func__, stitch_lib_.config);
    QMMF_ERROR("%s: qmmf_alg_register_bufs %p", __func__,
        stitch_lib_.register_bufs);
    QMMF_ERROR("%s: qmmf_alg_unregister_bufs %p", __func__,
        stitch_lib_.unregister_bufs);
    QMMF_ERROR("%s: qmmf_alg_flush %p", __func__, stitch_lib_.flush);
    QMMF_ERROR("%s: qmmf_alg_process %p", __func__,
        stitch_lib_.process);
    QMMF_ERROR("%s: qmmf_alg_get_debug_info_log %p", __func__,
        stitch_lib_.get_debug_info_log);
    ret = NAME_NOT_FOUND;
    goto FAIL;
  }

  ret = stitch_lib_.init(&stitch_lib_.context, nullptr);
  if (QMMF_ALG_SUCCESS != ret) {
    QMMF_ERROR("%s: Failed to initialize library, ret(%d)",
        __func__, ret);
    goto FAIL;
  }
  return ret;

FAIL:
  dlclose(handle);
  stitch_lib_ = {};
  return ret;
}

status_t StitchingBase::DeInitLibrary() {

  status_t ret = NO_ERROR;

  if (nullptr != stitch_lib_.handle) {
    stitch_lib_.deinit(stitch_lib_.context);
    ret = dlclose(stitch_lib_.handle);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Failed to close library, error: %s", __func__,
          dlerror());
    }
    stitch_lib_ = {};
  }
  return ret;
}

status_t StitchingBase::FlushLibrary() {

  if (nullptr == stitch_lib_.handle) {
    QMMF_ERROR("%s: Invalid library handle!", __func__);
    return BAD_VALUE;
  }

  if (!stitch_lib_.initialized) {
    // Library hasn't been initialized yet, wait and flush afterwards.
    if (init_library_status_.get() != NO_ERROR) {
      QMMF_ERROR("%s: Failed to load algorithm library!", __func__);
      return NO_INIT;
    }
    stitch_lib_.initialized = true;
  }
  stitch_lib_.flush(stitch_lib_.context);
  return NO_ERROR;
}

status_t StitchingBase::Configlibrary(std::vector<StreamBuffer> &input_buffers,
                                      std::vector<StreamBuffer> &output_buffers) {

  status_t ret = NO_ERROR;

  if (nullptr == stitch_lib_.handle) {
    QMMF_ERROR("%s: Invalid library handle!", __func__);
    return BAD_VALUE;
  }

  qmmf_alg_config_t config {};

  config.input.cnt  = input_buffers.size();
  config.output.cnt = output_buffers.size();

  config.input.fmts = static_cast<qmmf_alg_format_t*>(
      calloc(config.input.cnt, sizeof(*config.input.fmts)));

  if (nullptr == config.input.fmts) {
    QMMF_ERROR("%s: Failed to allocate memory for input format list",
        __func__);
    ret = NO_MEMORY;
    goto EXIT;
  }

  config.output.fmts = static_cast<qmmf_alg_format_t*>(
      calloc(config.output.cnt, sizeof(*config.output.fmts)));

  if (nullptr == config.output.fmts) {
    QMMF_ERROR("%s: Failed to allocate memory for output format list",
        __func__);
    ret = NO_MEMORY;
    goto EXIT;
  }

  for (uint32_t idx = 0; idx < config.input.cnt; ++idx) {
    const StreamBuffer *buf = &input_buffers.at(idx);
    ret = PopulateImageFormat(config.input.fmts[idx], buf);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Failed to set input image format", __func__);
      goto EXIT;
    }
  }

  for (uint32_t idx = 0; idx < config.output.cnt; ++idx) {
    const StreamBuffer *buf = &output_buffers.at(idx);
    ret = PopulateImageFormat(config.output.fmts[idx], buf);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Failed to set output image format", __func__);
      goto EXIT;
    }
  }

  ret = stitch_lib_.config(stitch_lib_.context, &config);
  if (QMMF_ALG_SUCCESS != ret) {
    QMMF_ERROR("%s: Failed to configure algo library, error(%d)",
        __func__, ret);
  }

EXIT:
  free(config.input.fmts);
  free(config.output.fmts);
  return ret;
}

status_t StitchingBase::ProcessBuffers(std::vector<StreamBuffer> &input_buffers,
                                       std::vector<StreamBuffer> &output_buffers) {

  status_t ret = NO_ERROR;
  const StreamBuffer *buffer = nullptr;

  if (nullptr == stitch_lib_.handle) {
    QMMF_ERROR("%s: Invalid library handle!", __func__);
    return BAD_VALUE;
  }

  qmmf_alg_process_data_t proc_data {};
  qmmf_alg_buf_list_t reg_buf_list {};

  proc_data.input.cnt  = input_buffers.size();
  proc_data.output.cnt = output_buffers.size();
  proc_data.user_data  = this;

  qmmf_alg_buffer_t input_buffer_list[proc_data.input.cnt];
  qmmf_alg_buffer_t output_buffer_list[proc_data.output.cnt];

  proc_data.input.bufs = input_buffer_list;
  proc_data.output.bufs = output_buffer_list;

  if (!params_.surface_crop.empty() && !single_camera_mode_) {
    for (auto const& cam_id : params_.camera_ids) {
      if (params_.surface_crop.find(cam_id) != params_.surface_crop.end()) {
        proc_data.input.crop_cnt++;
      }
    }
    if (params_.surface_crop.find(params_.multicam_id) !=
        params_.surface_crop.end()) {
      proc_data.output.crop_cnt++;
    }
  }
  qmmf_alg_crop_t input_crop[proc_data.input.crop_cnt];
  qmmf_alg_crop_t output_crop[proc_data.output.crop_cnt];

  proc_data.input.crop = input_crop;
  proc_data.output.crop = output_crop;

  for (uint32_t idx = 0; idx < proc_data.input.cnt; ++idx) {
    buffer = &input_buffers.at(idx);
    memset(&proc_data.input.bufs[idx], 0x0, sizeof(proc_data.input.bufs[idx]));
    ret = PrepareBuffer(reg_buf_list, proc_data.input.bufs[idx], buffer);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Failed to prepare input buffer", __func__);
      goto EXIT;
    }
    auto it = params_.surface_crop.find(buffer->camera_id);
    if (it != params_.surface_crop.end()) {
      proc_data.input.crop[idx].x = it->second.x;
      proc_data.input.crop[idx].y = it->second.y;
      proc_data.input.crop[idx].width = it->second.width;
      proc_data.input.crop[idx].height = it->second.height;
    }
  }

  for (uint32_t idx = 0; idx < proc_data.output.cnt; ++idx) {
    buffer = &output_buffers.at(idx);
    memset(&proc_data.output.bufs[idx], 0x0, sizeof(proc_data.output.bufs[idx]));
    ret = PrepareBuffer(reg_buf_list, proc_data.output.bufs[idx], buffer);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Failed to prepare output buffer", __func__);
      goto EXIT;
    }
    auto it = params_.surface_crop.find(buffer->camera_id);
    if (it != params_.surface_crop.end()) {
      proc_data.output.crop[idx].x = it->second.x;
      proc_data.output.crop[idx].y = it->second.y;
      proc_data.output.crop[idx].width = it->second.width;
      proc_data.output.crop[idx].height = it->second.height;
    }
  }

  if (reg_buf_list.cnt > 0) {
    ret = stitch_lib_.register_bufs(stitch_lib_.context, reg_buf_list);
    if (QMMF_ALG_SUCCESS != ret) {
      // Remove the failed buffers from the list with registered buffers.
      std::lock_guard<std::mutex> lock(register_buffer_lock_);
      for (uint32_t idx = 0; idx < reg_buf_list.cnt; ++idx) {
        registered_buffers_.erase(reg_buf_list.bufs[idx].fd);
      }
      QMMF_ERROR("%s: Register buffers failed, err(%d)", __func__, ret);
      goto EXIT;
    }
  }

  proc_data.complete = &StitchingBase::ProcessCallback;
  ret = stitch_lib_.process(stitch_lib_.context, &proc_data);
  if (QMMF_ALG_SUCCESS != ret) {
    QMMF_ERROR("%s: Failed to process images, err(%d)", __func__, ret);
  }

EXIT:
  free(reg_buf_list.bufs);
  return ret;
}

status_t StitchingBase::PopulateImageFormat(qmmf_alg_format_t &fmt,
                                            const StreamBuffer *buffer) {

  struct private_handle_t *priv_handle = (struct private_handle_t *)
      buffer->handle;
  if (nullptr == priv_handle) {
    QMMF_ERROR("%s: Invalid private handle!", __func__);
    return BAD_VALUE;
  }

  switch (priv_handle->format) {
    case HAL_PIXEL_FORMAT_BLOB:
      fmt.pix_fmt = QMMF_ALG_PIXFMT_JPEG;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
      fmt.pix_fmt = QMMF_ALG_PIXFMT_NV12;
      break;
    case HAL_PIXEL_FORMAT_NV21_ZSL:
      fmt.pix_fmt = QMMF_ALG_PIXFMT_NV21;
      break;
    case HAL_PIXEL_FORMAT_RAW10:
      fmt.pix_fmt = QMMF_ALG_PIXFMT_RAW_RGGB10;
      break;
    default:
      QMMF_ERROR("%s: Unsupported format: 0x%x", __func__,
          priv_handle->format);
      return NAME_NOT_FOUND;
  }
  fmt.width      = priv_handle->unaligned_width;
  fmt.height     = priv_handle->unaligned_height;
  fmt.num_planes = buffer->info.num_planes;

  for (uint32_t i = 0; i < buffer->info.num_planes; ++i) {
    fmt.plane[i].stride = buffer->info.plane_info[i].stride;
    fmt.plane[i].offset = 0;
    fmt.plane[i].length = buffer->info.plane_info[i].scanline *
      buffer->info.plane_info[i].stride;
  }

  return NO_ERROR;
}

status_t StitchingBase::PrepareBuffer(qmmf_alg_buf_list_t &reg_buf_list,
                                      qmmf_alg_buffer_t &img_buffer,
                                      const StreamBuffer *buffer) {

  status_t ret = PopulateImageFormat(img_buffer.fmt, buffer);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to set output image format", __func__);
    return ret;
  }

  img_buffer.vaddr  = 0;
  img_buffer.fd     = buffer->fd;
  img_buffer.size   = buffer->size;
  img_buffer.handle = buffer->handle;

  std::lock_guard<std::mutex> lock(register_buffer_lock_);
  if (registered_buffers_.find(buffer->fd) == registered_buffers_.end()) {
    // Add buffer to the list, later it will be removed in case register fails.
    registered_buffers_.insert(buffer->fd);

    // Increment the count of the buffers that need to be registered.
    ++reg_buf_list.cnt;

    // reallocate memory of the new qmmf_alg_buffer_t structure
    uint64_t new_size = reg_buf_list.cnt * sizeof(*reg_buf_list.bufs);
    qmmf_alg_buffer_t *new_buffer_ptr =
        static_cast<qmmf_alg_buffer_t*>(realloc(reg_buf_list.bufs, new_size));
    if (nullptr == new_buffer_ptr) {
      QMMF_ERROR("%s: Failed to realloc buffer memory", __func__);
      return NO_MEMORY;
    }
    reg_buf_list.bufs = new_buffer_ptr;

    // Get a pointer to the last buffer in the newly allocated structure
    // and copy the data from the previously filled image buffer.
    qmmf_alg_buffer_t *buf = &reg_buf_list.bufs[reg_buf_list.cnt - 1];
    memcpy(buf, &img_buffer, sizeof(img_buffer));
  }
  return NO_ERROR;
}

status_t StitchingBase::UnregisterBuffers(std::set<int32_t> buffer_fds) {

  std::lock_guard<std::mutex> lock(register_buffer_lock_);

  if (nullptr == stitch_lib_.handle) {
    QMMF_ERROR("%s: Invalid library handle!", __func__);
    return BAD_VALUE;
  }

  qmmf_alg_buf_list_t reg_buf_list {};
  reg_buf_list.cnt = buffer_fds.size();
  reg_buf_list.bufs = static_cast<qmmf_alg_buffer_t*>(
      calloc(reg_buf_list.cnt, sizeof(*reg_buf_list.bufs)));
  if (nullptr == reg_buf_list.bufs) {
    QMMF_ERROR("%s: Failed to allocate buffer memory", __func__);
    return NO_MEMORY;
  }

  uint32_t idx = 0;
  for (auto const& buffer_fd : buffer_fds) {
    reg_buf_list.bufs[idx++].fd = buffer_fd;
  }

  stitch_lib_.unregister_bufs(stitch_lib_.context, reg_buf_list);
  free(reg_buf_list.bufs);

  return NO_ERROR;
}

void StitchingBase::ProcessCallback(qmmf_alg_cb_t *cb_data) {

  QMMF_DEBUG("%s: Return status (%d)", __func__, cb_data->status);

  StitchingBase *algo = static_cast<StitchingBase *> (cb_data->user_data);
  if (algo->work_thread_name_ == "SnapshotStitching") {
    std::set<int32_t> buffer_fds = { cb_data->buf->fd };
    auto ret = algo->UnregisterBuffers(buffer_fds);
    if (NO_ERROR == ret) {
      std::lock_guard<std::mutex> lock(algo->register_buffer_lock_);
      algo->registered_buffers_.erase(cb_data->buf->fd);
    }
  }
  algo->ReturnProcessedBuffer(cb_data->buf->handle, cb_data->status);
}

GrallocMemory::GrallocMemory(alloc_device_t *gralloc_device)
    : gralloc_device_(gralloc_device),
      gralloc_slots_(nullptr),
      buffers_allocated_(0),
      pending_buffer_count_(0) {

  QMMF_INFO("%s: Enter", __func__);

  if (nullptr != gralloc_device_) {
    QMMF_INFO("%s: Gralloc Module author: %s, version: %d name: %s", __func__,
        gralloc_device_->common.module->author,
        gralloc_device_->common.module->hal_api_version,
        gralloc_device_->common.module->name);
  }
  QMMF_INFO("%s: Exit (%p)", __func__, this);
}

GrallocMemory::~GrallocMemory() {

  QMMF_INFO("%s: Enter", __func__);

  delete[] gralloc_slots_;
  gralloc_slots_ = nullptr;

  for (auto& it : gralloc_buffers_) {
    FreeGrallocBuffer(it.first);
  }
  gralloc_buffers_.clear();

  if (nullptr != gralloc_device_) {
    gralloc_device_->common.close(&gralloc_device_->common);
  }
  QMMF_INFO("%s: Exit (%p)", __func__, this);
}

status_t GrallocMemory::Initialize() {

  status_t ret = NO_ERROR;
  hw_module_t const *module = nullptr;

  if (nullptr != gralloc_device_) {
    QMMF_WARN("%s: Gralloc allocator already created", __func__);
    return ret;
  }

  ret = hw_get_module(GRALLOC_HARDWARE_MODULE_ID, &module);
  if ((NO_ERROR != ret) || (nullptr == module)) {
    QMMF_ERROR("%s: Unable to load Gralloc module: %d", __func__, ret);
    return ret;
  }

  ret = module->methods->open(module, GRALLOC_HARDWARE_GPU0,
                              (struct hw_device_t **)&gralloc_device_);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Could not open Gralloc module: %s (%d)", __func__,
        strerror(-ret), ret);
    dlclose(module->dso);
    return ret;
  }

  QMMF_INFO("%s: Gralloc Module author: %s, version: %d name: %s", __func__,
      gralloc_device_->common.module->author,
      gralloc_device_->common.module->hal_api_version,
      gralloc_device_->common.module->name);

  return ret;
}

status_t GrallocMemory::Configure(BufferParams &params) {

  if (gralloc_device_ == nullptr) {
    QMMF_ERROR("%s: Gralloc allocator not created", __func__);
    return INVALID_OPERATION;
  }

  delete[] gralloc_slots_;
  gralloc_slots_ = nullptr;

  for (auto& it : gralloc_buffers_) {
    FreeGrallocBuffer(it.first);
  }
  gralloc_buffers_.clear();
  buffers_allocated_ = 0;

  params_ = params;

  gralloc_slots_ = new buffer_handle_t[params_.max_buffer_count];
  if (nullptr == gralloc_slots_) {
    QMMF_ERROR("%s: Unable to allocate buffer handles!\n", __func__);
    return NO_MEMORY;
  }

  return NO_ERROR;
}

status_t GrallocMemory::GetBuffer(buffer_handle_t &buffer) {

  std::unique_lock<std::mutex> lock(buffer_lock_);
  std::chrono::nanoseconds wait_time(kBufferWaitTimeout);

  while (pending_buffer_count_ == params_.max_buffer_count) {
    QMMF_VERBOSE("%s: Already retrieved maximum buffers (%d), waiting"
        " on a free one", __func__, params_.max_buffer_count);

    auto ret = wait_for_buffer_.WaitFor(lock, wait_time);
    if (ret != 0) {
      QMMF_ERROR("%s: Wait for output buffer return timed out", __func__);
      return TIMED_OUT;
    }
  }
  auto ret = GetBufferLocked(buffer);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to retrieve output buffer", __func__);
    return ret;
  }

  return NO_ERROR;
}

status_t GrallocMemory::ReturnBuffer(const buffer_handle_t &buffer) {

  std::lock_guard<std::mutex> lock(buffer_lock_);
  QMMF_DEBUG("%s: Buffer(%p) returned to memory pool", __func__, buffer);

  status_t ret = ReturnBufferLocked(buffer);
  if (ret == NO_ERROR) {
    wait_for_buffer_.Signal();
  }
  return ret;
}

status_t GrallocMemory::GetBufferLocked(buffer_handle_t &buffer) {

  status_t ret = NO_ERROR;
  int32_t idx = -1;
  buffer_handle_t handle = nullptr;

  //Only pre-allocate buffers in case no valid streamBuffer
  //is passed as an argument.

  for (auto& buffer : gralloc_buffers_) {
    if (buffer.second == true) {
      handle = buffer.first;
      buffer.second = false;
      break;
    }
  }
  // Find the slot of the available gralloc buffer.
  if (nullptr != handle) {
    for (uint32_t i = 0; i < buffers_allocated_; ++i) {
      if (gralloc_slots_[i] == handle) {
        idx = i;
        break;
      }
    }
  } else if ((nullptr == handle) &&
             (buffers_allocated_ < params_.max_buffer_count)) {
    ret = AllocGrallocBuffer(&handle);
    if (NO_ERROR != ret) {
      return ret;
    }
    idx = buffers_allocated_;
    gralloc_slots_[idx] = handle;
    gralloc_buffers_.emplace(gralloc_slots_[idx], false);
    ++buffers_allocated_;
  }

  if ((nullptr == handle) || (0 > idx)) {
    QMMF_ERROR("%s: Unable to allocate or find a free buffer!", __func__);
    return INVALID_OPERATION;
  }

  buffer = gralloc_slots_[idx];
  ++pending_buffer_count_;

  return ret;
}


status_t GrallocMemory::ReturnBufferLocked(const buffer_handle_t &buffer) {

  if (pending_buffer_count_ == 0) {
    QMMF_ERROR("%s: Not expecting any buffers!", __func__);
    return INVALID_OPERATION;
  }

  if (gralloc_buffers_.count(buffer) == 0) {
    QMMF_ERROR("%s: Buffer %p returned that wasn't allocated by this"
        " Memory Pool!", __func__, buffer);
    return BAD_VALUE;
  }

  gralloc_buffers_[buffer] = true;
  --pending_buffer_count_;

  return NO_ERROR;
}

status_t GrallocMemory::PopulateMetaInfo(CameraBufferMetaData &info,
                                             buffer_handle_t &buffer) {

  if (nullptr == buffer) {
    QMMF_ERROR("%s: Invalid buffer handle!\n", __func__);
    return BAD_VALUE;
  }

  {
    std::lock_guard<std::mutex> lock(buffer_lock_);
    bool is_valid_handle = false;
    for (uint32_t i = 0; i < buffers_allocated_; ++i) {
      if (gralloc_slots_[i] == buffer) {
        is_valid_handle = true;
        break;
      }
    }
    if (!is_valid_handle) {
      QMMF_ERROR("%s: Buffer handle wasn't allocated by this Gralloc"
          " Memory Pool!", __func__);
      return BAD_VALUE;
    }
  }

  struct private_handle_t *priv_handle = (struct private_handle_t *) buffer;

  int aligned_width, aligned_height;
  gralloc_module_t const *mapper = reinterpret_cast<gralloc_module_t const *>(
          gralloc_device_->common.module);
  status_t ret = mapper->perform(
      mapper,
      GRALLOC_MODULE_PERFORM_GET_CUSTOM_STRIDE_AND_HEIGHT_FROM_HANDLE,
      priv_handle, &aligned_width, &aligned_height);
  if (0 != ret) {
    QMMF_ERROR("%s: Unable to query stride&scanline: %d\n", __func__, ret);
    return ret;
  }

  switch (priv_handle->format) {
    case HAL_PIXEL_FORMAT_BLOB:
      info.format = BufferFormat::kBLOB;
      info.num_planes = 1;
      info.plane_info[0].width = params_.max_size;
      info.plane_info[0].height = 1;
      info.plane_info[0].stride = aligned_width;
      info.plane_info[0].scanline = aligned_height;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
      info.format = BufferFormat::kNV12;
      info.num_planes = 2;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = aligned_width;
      info.plane_info[0].scanline = aligned_height;
      info.plane_info[1].width = params_.width;
      info.plane_info[1].height = params_.height/2;
      info.plane_info[1].stride = aligned_width;
      info.plane_info[1].scanline = aligned_height/2;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
      info.format = BufferFormat::kNV12UBWC;
      info.num_planes = 2;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = aligned_width;
      info.plane_info[0].scanline = aligned_height;
      info.plane_info[1].width = params_.width;
      info.plane_info[1].height = params_.height/2;
      info.plane_info[1].stride = aligned_width;
      info.plane_info[1].scanline = aligned_height/2;
      break;
    case HAL_PIXEL_FORMAT_NV21_ZSL:
      info.format = BufferFormat::kNV21;
      info.num_planes = 2;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = aligned_width;
      info.plane_info[0].scanline = aligned_height;
      info.plane_info[1].width = params_.width;
      info.plane_info[1].height = params_.height/2;
      info.plane_info[1].stride = aligned_width;
      info.plane_info[1].scanline = aligned_height/2;
      break;
    case HAL_PIXEL_FORMAT_RAW8:
      info.format = BufferFormat::kRAW8;
      info.num_planes = 1;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = aligned_width;
      info.plane_info[0].scanline = aligned_height;
      break;
    case HAL_PIXEL_FORMAT_RAW10:
      info.format = BufferFormat::kRAW10;
      info.num_planes = 1;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = aligned_width;
      info.plane_info[0].scanline = aligned_height;
      break;
    case HAL_PIXEL_FORMAT_RAW16:
      info.format = BufferFormat::kRAW16;
      info.num_planes = 1;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = aligned_width;
      info.plane_info[0].scanline = aligned_height;
      break;
    default:
      QMMF_ERROR("%s: Unsupported format: %d", __func__,
          priv_handle->format);
      return NAME_NOT_FOUND;
  }

  return NO_ERROR;
}

status_t GrallocMemory::AllocGrallocBuffer(buffer_handle_t *buf) {

  if (gralloc_device_ == nullptr) {
    QMMF_ERROR("%s: Gralloc allocator not created", __func__);
    return INVALID_OPERATION;
  }

  status_t ret      = NO_ERROR;
  uint32_t width    = params_.width;
  uint32_t height   = params_.height;
  int32_t  format   = params_.format;
  int32_t  usage    = params_.gralloc_flags;
  uint32_t max_size = params_.max_size;

  // Filter out any usage bits that shouldn't be passed to the gralloc module.
  usage &= GRALLOC_USAGE_ALLOC_MASK;

  if (!width || !height) {
    width = height = 1;
  }

  int stride = 0;
  if (0 < max_size) {
    // Blob buffers are expected to get allocated with width equal to blob
    // max size and height equal to 1.
    ret = gralloc_device_->alloc(gralloc_device_, static_cast<int>(max_size),
                                 static_cast<int>(1), format,
                                 static_cast<int>(usage), buf, &stride);
  } else {
    ret = gralloc_device_->alloc(gralloc_device_, static_cast<int>(width),
                                 static_cast<int>(height), format,
                                 static_cast<int>(usage), buf, &stride);
  }
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to allocate gralloc buffer", __func__);
  }

  return ret;
}

status_t GrallocMemory::FreeGrallocBuffer(buffer_handle_t buf) {

  if (gralloc_device_ == nullptr) {
    QMMF_ERROR("%s: Gralloc allocator not created", __func__);
    return INVALID_OPERATION;
  }
  return gralloc_device_->free(gralloc_device_, buf);
}

}; //namespace recorder.

}; //namespace qmmf.
