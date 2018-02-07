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

#define TAG "RecorderCameraSource"

#include <cmath>
#include <fcntl.h>
#include <dirent.h>
#include <sys/mman.h>
#include <sys/time.h>

#ifdef ENABLE_360
#include "recorder/src/service/qmmf_multicamera_manager.h"
#endif
#include "recorder/src/service/qmmf_camera_source.h"
#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_recorder_utils.h"
#include "recorder/src/service/post-process/factory/qmmf_postproc_factory.h"

namespace qmmf {

namespace recorder {

using ::std::make_shared;
using ::std::shared_ptr;

static const nsecs_t kWaitDuration = 2000000000; // 2 s.
static const int32_t kDebugTrackFps = 1<<0;
static const int32_t kDebugSourceTrackFps = 1<<1;
static const int32_t kDebugFrameSkip = 1<<2;
static const uint64_t kTsFactor = 10000000; // 10 ms.

CameraSource* CameraSource::instance_ = nullptr;

CameraSource* CameraSource::CreateCameraSource() {

  if (!instance_) {
    instance_ = new CameraSource;
    if (!instance_) {
      QMMF_ERROR("%s:%s: Can't Create CameraSource Instance", TAG, __func__);
      //return nullptr;
    }
  }
  QMMF_INFO("%s:%s: CameraSource Instance Created Successfully(0x%p)", TAG,
      __func__, instance_);
  return instance_;
}

CameraSource::CameraSource() {

  QMMF_KPI_GET_MASK();
  QMMF_KPI_DETAIL();
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  factory_ = PostProcFactory::getInstance();
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

CameraSource::~CameraSource() {

  QMMF_KPI_DETAIL();
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  if (!camera_map_.isEmpty()) {
    camera_map_.clear();
  }
  PostProcFactory::releaseInstance();
  factory_ = nullptr;
  instance_ = nullptr;
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

status_t CameraSource::StartCamera(const uint32_t camera_id,
                                   const CameraStartParam &param,
                                   const ResultCb &cb,
                                   const ErrorCb &errcb) {

  QMMF_INFO("%s:%s: Camera Id(%u) to open!", TAG, __func__, camera_id);
  QMMF_KPI_DETAIL();
  bool is_virtual_camera_id = false;

#ifdef ENABLE_360
  is_virtual_camera_id = (kVirtualCameraIdOffset <= camera_id);
#endif

  sp<CameraInterface> camera;

  if (is_virtual_camera_id) {
    if (NAME_NOT_FOUND == camera_map_.indexOfKey(camera_id)) {
      QMMF_ERROR("%s:%s: Invalid Virtual Camera Id(%u)!", TAG, __func__,
                 camera_id);
      return BAD_VALUE;
    }
    camera = camera_map_.valueFor(camera_id);
  } else {
    if (camera_map_.indexOfKey(camera_id) >= 0) {
      QMMF_ERROR("%s:%s: Camera Id(%u) is already open!", TAG, __func__,
          camera_id);
      return BAD_VALUE;
    }
    camera = new CameraContext();
    if (!camera.get()) {
      QMMF_ERROR("%s:%s: Can't Instantiate CameraDevice(%d)!!", TAG,
          __func__, camera_id);
      return NO_MEMORY;
    }
    // Add contexts to map when in regular camera case.
    camera_map_.add(camera_id, camera);
  }

  auto ret = camera->OpenCamera(camera_id, param, cb, errcb);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CameraDevice:OpenCamera(%d)failed!", TAG, __func__,
        camera_id);
    if (!is_virtual_camera_id) {
      camera.clear();
      camera_map_.removeItem(camera_id);
    }
    return ret;
  }
  QMMF_INFO("%s:%s: Camera(%d) Open is Successfull!", TAG, __func__, camera_id);
  return ret;
}

status_t CameraSource::StopCamera(const uint32_t camera_id) {

  QMMF_KPI_DETAIL();
  int32_t ret = NO_ERROR;
  QMMF_INFO("%s:%s: CameraId(%u) to close!", TAG, __func__, camera_id);

  //TODO: check if streams are still active, flush them before closing camera.

  bool match = false;
  for (uint32_t i = 0; i < camera_map_.size(); ++i) {
    if (camera_id == camera_map_.keyAt(i)) {
      match = true;
      sp<CameraInterface> camera = camera_map_.valueFor(camera_id);
      ret = camera->CloseCamera(camera_id);
      assert(ret == NO_ERROR);
      camera_map_.removeItem(camera_id);
      QMMF_INFO("%s:%s: Camera(%d) is Closed Successfull!", TAG, __func__,
          camera_id);
      break;
    }
  }
  if (!match) {
    QMMF_ERROR("%s:%s: Invalid Camera Id(%d)", TAG, __func__, camera_id);
    return BAD_VALUE;
  }
  return ret;
}

status_t CameraSource::CreateMultiCamera(const std::vector<uint32_t> camera_ids,
                                         uint32_t *virtual_camera_id) {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  QMMF_KPI_DETAIL();
#ifdef ENABLE_360
  sp<CameraInterface> multi_camera = new MultiCameraManager();
  if (!multi_camera.get()) {
    QMMF_ERROR("%s:%s: Can't Instantiate MultiCameraDevice!!", TAG, __func__);
    return NO_MEMORY;
  }

  MultiCameraManager *camera_mgr =
      static_cast<MultiCameraManager*>(multi_camera.get());

  auto ret = camera_mgr->CreateMultiCamera(camera_ids, virtual_camera_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreateMultiCamera Failed!", TAG, __func__);
    multi_camera.clear();
    return NO_INIT;
  }
  // Adds only virtual cameras. Virtual camera is a camera used
  // for 360 camera case.
  camera_map_.add(*virtual_camera_id, multi_camera);
#endif
  QMMF_INFO("%s:%s: Exit ", TAG, __func__);
  return NO_ERROR;
}

status_t CameraSource::ConfigureMultiCamera(const uint32_t virtual_camera_id,
                                            const MultiCameraConfigType type,
                                            const void *param,
                                            const uint32_t param_size) {

  status_t ret = NO_ERROR;
#ifdef ENABLE_360
  if ((kVirtualCameraIdOffset > virtual_camera_id) ||
      (NAME_NOT_FOUND == camera_map_.indexOfKey(virtual_camera_id))) {
    QMMF_ERROR("%s:%s: Invalid Virtual Camera Id(%u)!", TAG, __func__,
        virtual_camera_id);
    return BAD_VALUE;
  }

  sp<CameraInterface> multi_camera = camera_map_.valueFor(virtual_camera_id);
  MultiCameraManager *camera_mgr =
      static_cast<MultiCameraManager*>(multi_camera.get());

  ret = camera_mgr->ConfigureMultiCamera(virtual_camera_id, type,
                                         param, param_size);
#endif
  return ret;
}

status_t CameraSource::GetSupportedPlugins(SupportedPlugins *plugins) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);

  auto ret = factory_->GetSupportedPlugins(plugins);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: GetSupportedPlugins Failed!", TAG, __func__);
    return ret;
  }

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t CameraSource::CreatePlugin(uint32_t *uid, const PluginInfo &plugin) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);

  auto ret = factory_->CreatePlugin(*uid, plugin);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreatePlugin Failed!", TAG, __func__);
    return ret;
  }

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t CameraSource::DeletePlugin(const uint32_t &uid) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);

  auto ret = factory_->DeletePlugin(uid);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: DeletePlugin Failed!", TAG, __func__);
    return ret;
  }

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t CameraSource::ConfigPlugin(const uint32_t &uid,
                                    const std::string &json_config) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);

  auto ret = factory_->ConfigPlugin(uid, json_config);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: ConfigPlugin Failed!", TAG, __func__);
    return ret;
  }

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t CameraSource::CaptureImage(const uint32_t camera_id,
                                    const ImageParam &param,
                                    const uint32_t num_images,
                                    const std::vector<CameraMetadata> &meta,
                                    const SnapshotCb& cb) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  QMMF_KPI_DETAIL();

  bool match = false;
  sp<CameraInterface> camera;
  for (uint8_t i = 0; i < camera_map_.size(); i++) {
    if (camera_id == camera_map_.keyAt(i)) {
        match = true;
        camera = camera_map_.valueAt(i);
        break;
    }
  }
  if (!match) {
    QMMF_ERROR("%s:%s: Invalid Camera Id, It is different then camera is open"
        "with", TAG, __func__);
    return BAD_VALUE;
  }
  assert(camera.get() != nullptr);

  auto ret = camera->SetUpCapture(param, num_images);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: SetUpCapture Failed!", TAG, __func__);
    return ret;
  }
  client_snapshot_cb_ = cb;
  StreamSnapshotCb stream_cb = [&] (uint32_t count, StreamBuffer& buf) {
    SnapshotCallback(count, buf);
  };
  ret = camera->CaptureImage(meta, stream_cb);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CaptureImage Failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t CameraSource::ConfigImageCapture(const uint32_t camera_id,
                                          const ImageConfigParam &config) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);

  bool match = false;
  sp<CameraInterface> camera;
  for (uint8_t i = 0; i < camera_map_.size(); i++) {
    if (camera_id == camera_map_.keyAt(i)) {
        match = true;
        camera = camera_map_.valueAt(i);
        break;
    }
  }
  if (!match) {
    QMMF_ERROR("%s:%s: Invalid Camera Id, It is different then camera is open"
        "with", TAG, __func__);
    return BAD_VALUE;
  }
  assert(camera.get() != nullptr);

  auto ret = camera->ConfigImageCapture(config);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: ConfigImageCapture Failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t CameraSource::CancelCaptureImage(const uint32_t camera_id) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  QMMF_KPI_DETAIL();

  bool match = false;
  sp<CameraInterface> camera;
  for (uint8_t i = 0; i < camera_map_.size(); i++) {
    if (camera_id == camera_map_.keyAt(i)) {
      match = true;
      camera = camera_map_.valueAt(i);
    }
  }
  if (!match) {
    QMMF_ERROR("%s:%s: Invalid Camera Id(%d)!", TAG, __func__, camera_id);
    return BAD_VALUE;
  }
  assert(camera.get() != nullptr);

  auto ret = camera->CancelCaptureImage();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CancelCaptureImage Failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t CameraSource::ReturnImageCaptureBuffer(const uint32_t camera_id,
                                                const int32_t buffer_id) {
  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);

  bool match = false;
  sp<CameraInterface> camera;
  for (uint8_t i = 0; i < camera_map_.size(); i++) {
    if (camera_id == camera_map_.keyAt(i)) {
      match = true;
      camera = camera_map_.valueAt(i);
      break;
    }
  }
  if (!match) {
    QMMF_ERROR("%s:%s: Invalid Camera Id!", TAG, __func__);
    return BAD_VALUE;
  }
  assert(camera.get() != nullptr);
  auto ret = camera->ReturnImageCaptureBuffer(camera_id, buffer_id);

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
}

bool CameraSource::IsCopyStream(const VideoTrackParams& params) {
  return params.extra_param.Exists(QMMF_SOURCE_VIDEO_TRACK_ID);
}

status_t CameraSource::GetSourceTrackParam(
    const VideoTrackParams& params, SourceVideoTrack& surface_video_copy) {
  status_t ret = BAD_VALUE;
  if (IsCopyStream(params)) {
    params.extra_param.Fetch(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy);
    ret = NO_ERROR;
  }
  return ret;
}

status_t CameraSource::GetSlaveStreamMasterTrackId(
    const VideoTrackParams& params, int32_t& track_id_master) {
  SourceVideoTrack surface_video_copy;
  if (NO_ERROR == GetSourceTrackParam(params, surface_video_copy)) {
    track_id_master = params.track_id&0xffff0000;
    track_id_master |= surface_video_copy.source_track_id;
    return NO_ERROR;
  }
  return BAD_VALUE;
}

bool CameraSource::ValidateSlaveTrackParam(
    const VideoTrackParams& slave_track,
    const VideoTrackParams& master_track) {

  QMMF_DEBUG("%s:%s %d x %d -> %d x %d fmt 0x%x -> 0x%x", TAG, __func__,
      master_track.params.width,
      master_track.params.height,
      slave_track.params.width,
      slave_track.params.height,
      master_track.params.format_type,
      slave_track.params.format_type);

  if ((slave_track.params.format_type != VideoFormat::kHEVC) &&
      (slave_track.params.format_type != VideoFormat::kAVC) &&
      (slave_track.params.format_type != VideoFormat::kYUV) &&
      (master_track.params.format_type != VideoFormat::kHEVC) &&
      (master_track.params.format_type != VideoFormat::kAVC) &&
      (master_track.params.format_type != VideoFormat::kYUV)) {
    QMMF_ERROR("%s:%s Invalid format:", TAG, __func__);
    return false;
  }

  if((slave_track.params.width >  master_track.params.width) ||
      (slave_track.params.height > master_track.params.height)) {
    QMMF_ERROR("%s:%s Invalid size:", TAG, __func__);
    return false;
  }
  return true;
}

bool CameraSource::CheckLinkedStream(
    const VideoTrackParams& slave_track,
    const VideoTrackParams& master_track) {

  QMMF_DEBUG("%s:%s %d x %d -> %d x %d fmt 0x%x -> 0x%x", TAG, __func__,
    master_track.params.width,
    master_track.params.height,
    slave_track.params.width,
    slave_track.params.height,
    master_track.params.format_type,
    slave_track.params.format_type);

  if ((slave_track.params.format_type != VideoFormat::kHEVC) &&
      (slave_track.params.format_type != VideoFormat::kAVC) &&
      (slave_track.params.format_type != VideoFormat::kYUV) &&
      (master_track.params.format_type != VideoFormat::kHEVC) &&
      (master_track.params.format_type != VideoFormat::kAVC) &&
      (master_track.params.format_type != VideoFormat::kYUV)) {
    QMMF_ERROR("%s:%s Invalid format:", TAG, __func__);
    return false;
  }

  if((slave_track.params.width ==  master_track.params.width) ||
      (slave_track.params.height == master_track.params.height)) {
    QMMF_ERROR("%s:%s Same size:", TAG, __func__);
    return true;
  }
  return false;
}

status_t CameraSource::CreateTrackSource(const uint32_t track_id,
                                         const VideoTrackParams& track_params) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  QMMF_KPI_DETAIL();

  // Find out the camera context corresponding to camera id where track has to
  // be created.
  bool match = false;
  sp<CameraInterface> camera;
  for (uint8_t i = 0; i < camera_map_.size(); i++) {
    if (track_params.params.camera_id == camera_map_.keyAt(i)) {
      match = true;
      camera = camera_map_.valueAt(i);
      break;
    }
  }
  if (!match) {
    QMMF_ERROR("%s:%s: Invalid Camera Id, It is different then camera is open"
        "with", TAG, __func__);
    return BAD_VALUE;
  }

  status_t ret;
  int32_t track_id_master = -1;
  int32_t port_track_id = -1;
  bool copy_stream_mode = false;
  bool linked_mode = false;
  ret = GetSlaveStreamMasterTrackId(track_params, track_id_master);
  if (ret == NO_ERROR && track_id_master != -1) {
    shared_ptr<TrackSource> track = track_sources_.valueFor(track_id_master);
    QMMF_INFO("%s:%s: Master->slave 0x%x->0x%x", TAG, __func__,
        track_id_master, track_id);
    assert(track.get() != nullptr);
    if (ValidateSlaveTrackParam(track_params, track->getParams())) {
      linked_mode = CheckLinkedStream(track_params, track->getParams());
      if (track->IsSlaveTrack()) {
        int32_t master_track_id = track->GetMasterTrackId();
        for (size_t i = 0; i < track_sources_.size(); i++) {
          shared_ptr<TrackSource> track =
              track_sources_.valueFor(master_track_id);
          if (track.get() != nullptr) {
            if (track->IsSlaveTrack()) {
              master_track_id = track->GetMasterTrackId();
              continue;
            } else {
                port_track_id = track->GetCameraPortId();
                break;
            }
          }
        }
      } else {
        port_track_id = track_id_master;
      }
      if (port_track_id != -1) {
        copy_stream_mode = true;
        QMMF_INFO("%s:%s: Copy stream should be create.", TAG, __func__);
      }
    } else {
      QMMF_ERROR("%s:%s: Copy stream validation failed.", TAG, __func__);
    }
  } else {
    QMMF_INFO("%s:%s: Normal stream should be create.", TAG, __func__);
  }

  // Create TrackSource and give it to CameraInterface, CameraConext in turn would
  // Map it to its one of port.
  shared_ptr<TrackSource> track_source = make_shared<TrackSource>(track_params,
                                                                  camera);
  if (!track_source.get()) {
    QMMF_ERROR("%s:%s: Can't create TrackSource Instance", TAG, __func__);
    return NO_MEMORY;
  }

  shared_ptr<TrackSource> master_track;
  if (copy_stream_mode) {
    sp<CameraRescaler> rescaler;
    if (linked_mode == false) {
      if (rescalers_.count(track_id_master) == 0 ||
         (port_track_id == track_id_master)) {
        rescaler = new CameraRescaler();
        ret = rescaler->Init(track_params);
        if (ret != NO_ERROR) {
          rescaler = nullptr;
          QMMF_ERROR("%s:%s: Rescaler Init Failed", TAG, __func__);
          return BAD_VALUE;
        }
        rescalers_.emplace(track_id, rescaler);
      } else {
        QMMF_ERROR("%s:%s: GET Copy TrackSource Instance trackId: %x",
            TAG, __func__, track_id);
        rescaler = rescalers_.at(track_id_master);
      }
      master_track = track_sources_.valueFor(port_track_id);
    } else {
      master_track = track_sources_.valueFor(track_id_master);
    }

    assert(master_track.get() != nullptr);
    ret = track_source->InitCopy(
        master_track, rescaler, port_track_id, track_id_master);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: track_id(%x) CopyTrackSource Init failed!",
          TAG, __func__, track_id);
      rescalers_.erase(track_id);
    }
  } else {
    ret = track_source->Init();
  }

  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: track_id(%x) TrackSource Init failed!", TAG, __func__,
        track_id);
    goto FAIL;
  }
  track_sources_.add(track_id, track_source);

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
FAIL:
  track_source = nullptr;
  return ret;
}

status_t CameraSource::DeleteTrackSource(const uint32_t track_id) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->DeInit();
  assert(ret == NO_ERROR);

  track_sources_.removeItem(track_id);
  rescalers_.erase(track_id);

  QMMF_INFO("%s:%s: track_id(%x) Deleted Successfully!", TAG, __func__,
      track_id);
  return ret;
}

status_t CameraSource::StartTrackSource(const uint32_t track_id) {

  QMMF_KPI_DETAIL();
  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->StartTrack();
  assert(ret == NO_ERROR);

  QMMF_VERBOSE("%s:%s: TrackSource id(%x) Started Succesffuly!", TAG, __func__,
      track_id);
  return ret;
}

status_t CameraSource::StopTrackSource(const uint32_t track_id,
                                       bool is_force_cleanup) {

  QMMF_KPI_DETAIL();
  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->StopTrack(is_force_cleanup);
  assert(ret == NO_ERROR);

  QMMF_VERBOSE("%s:%s: TrackSource id(%x) Stopped Succesffuly!", TAG, __func__,
      track_id);
  return ret;
}

status_t CameraSource::PauseTrackSource(const uint32_t track_id) {
  // Not Implemented
  return NO_ERROR;
}

status_t CameraSource::ResumeTrackSource(const uint32_t track_id) {
  // Not Implemented
  return NO_ERROR;
}

status_t CameraSource::ReturnTrackBuffer(const uint32_t track_id,
                                         std::vector<BnBuffer> &buffers) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }

  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);
  auto ret = track->ReturnTrackBuffer(buffers);
  assert(ret == NO_ERROR);
  return ret;
}

status_t CameraSource::SetCameraParam(const uint32_t camera_id,
                                      const CameraMetadata &meta) {

  sp<CameraInterface> camera = camera_map_.valueFor(camera_id);
  assert(camera.get() != nullptr);

  return camera->SetCameraParam(meta);
}

status_t CameraSource::GetCameraParam(const uint32_t camera_id,
                                      CameraMetadata &meta) {

  sp<CameraInterface> camera = camera_map_.valueFor(camera_id);
  assert(camera.get() != nullptr);

  return camera->GetCameraParam(meta);
}

status_t CameraSource::GetDefaultCaptureParam(const uint32_t camera_id,
                                              CameraMetadata &meta) {

  sp<CameraInterface> camera = camera_map_.valueFor(camera_id);
  assert(camera.get() != nullptr);

  return camera->GetDefaultCaptureParam(meta);
}

status_t CameraSource::UpdateTrackFrameRate(const uint32_t track_id,
                                            const float frame_rate) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  track->UpdateFrameRate(frame_rate);

  return NO_ERROR;
}

status_t CameraSource::EnableFrameRepeat(const uint32_t track_id,
                                         const bool enable_frame_repeat) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  track->EnableFrameRepeat(enable_frame_repeat);
  return NO_ERROR;
}

status_t CameraSource::CreateOverlayObject(const uint32_t track_id,
                                           OverlayParam *param,
                                           uint32_t *overlay_id) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->CreateOverlayObject(param, overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreateOverlayObject failed!", TAG, __func__);
    return BAD_VALUE;
  }
  return ret;
}

status_t CameraSource::DeleteOverlayObject(const uint32_t track_id,
                                           const uint32_t overlay_id) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->DeleteOverlayObject(overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: DeleteOverlayObject failed!", TAG, __func__);
    return BAD_VALUE;
  }
  return ret;
}

status_t CameraSource::GetOverlayObjectParams(const uint32_t track_id,
                                              const uint32_t overlay_id,
                                              OverlayParam &param) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->GetOverlayObjectParams(overlay_id, param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: GetOverlayObjectParams failed!", TAG, __func__);
    return BAD_VALUE;
  }
  return ret;
}

status_t CameraSource::UpdateOverlayObjectParams(const uint32_t track_id,
                                                 const uint32_t overlay_id,
                                                 OverlayParam *param) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->UpdateOverlayObjectParams(overlay_id, param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: UpdateOverlayObjectParams failed!", TAG, __func__);
    return BAD_VALUE;
  }
  return ret;
}

status_t CameraSource::SetOverlayObject(const uint32_t track_id,
                                        const uint32_t overlay_id) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->SetOverlayObject(overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: SetOverlayObject failed!", TAG, __func__);
    return BAD_VALUE;
  }
  return ret;
}

status_t CameraSource::RemoveOverlayObject(const uint32_t track_id,
                                           const uint32_t overlay_id) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s:%s: track_id is not valid !!", TAG, __func__);
    return BAD_VALUE;
  }
  shared_ptr<TrackSource> track = track_sources_.valueFor(track_id);
  assert(track.get() != nullptr);

  auto ret = track->RemoveOverlayObject(overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: RemoveOverlayObject failed!", TAG, __func__);
    return BAD_VALUE;
  }
  return ret;
}

const shared_ptr<TrackSource>& CameraSource::GetTrackSource(uint32_t track_id) {

  int32_t idx = track_sources_.indexOfKey(track_id);
  assert(idx >= 0);
  return track_sources_.valueFor(track_id);
}

bool CameraSource::IsTrackIdValid(const uint32_t track_id) {

  bool valid = false;
  size_t size = track_sources_.size();
  QMMF_DEBUG("%s: Number of Tracks exist = %d",__func__, size);
  for (size_t i = 0; i < size; i++) {
    if (track_id == track_sources_.keyAt(i)) {
        valid = true;
        break;
    }
  }
  return valid;
}

uint32_t CameraSource::GetJpegSize(uint8_t *blobBuffer, uint32_t width) {

  uint32_t ret = width;
  uint32_t blob_size = sizeof(struct camera3_jpeg_blob);

  if (width > blob_size) {
    size_t offset = width - blob_size - 1;
    uint8_t *footer = blobBuffer + offset;
    struct camera3_jpeg_blob *jpegBlob = (struct camera3_jpeg_blob *)footer;

    if (CAMERA3_JPEG_BLOB_ID == jpegBlob->jpeg_blob_id) {
      ret = jpegBlob->jpeg_size;
    } else {
      QMMF_ERROR("%s:%s Jpeg Blob structure missing!\n", TAG, __func__);
    }
  } else {
    QMMF_ERROR("%s:%s Buffer width: %u equal or smaller than Blob size: %u\n",
        TAG, __func__, width, blob_size);
  }
  return ret;
}

void CameraSource::SnapshotCallback(uint32_t count, StreamBuffer& buffer) {

  uint32_t content_size = 0;
  int32_t width = -1, height = -1;
  void* vaddr = nullptr;
  switch (buffer.info.format) {
    case BufferFormat::kNV12:
    case BufferFormat::kNV21:
    case BufferFormat::kRAW10:
    case BufferFormat::kRAW12:
    case BufferFormat::kRAW16:
      width  = buffer.info.plane_info[0].width;
      height = buffer.info.plane_info[0].height;
      content_size = buffer.size;
      break;
    case BufferFormat::kBLOB:
      vaddr = mmap(nullptr, buffer.size, PROT_READ | PROT_WRITE, MAP_SHARED,
          buffer.fd, 0);
      assert(vaddr != nullptr);
      assert(0 < buffer.info.num_planes);
      content_size = GetJpegSize((uint8_t*) vaddr,
                                 buffer.info.plane_info[0].width);
      QMMF_INFO("%s:%s: jpeg buffer size(%d)", TAG, __func__, content_size);
      assert(0 < content_size);
      if (vaddr) {
        munmap(vaddr, buffer.size);
        vaddr = nullptr;
      }
      width  = -1;
      height = -1;
    break;
    default:
      QMMF_ERROR("%s:%s format(%d) not supported", TAG, __func__,
          buffer.info.format);
      assert(0);
    break;
  }

  BnBuffer bn_buffer;
  memset(&bn_buffer, 0x0, sizeof bn_buffer);
  bn_buffer.ion_fd    = buffer.fd;
  bn_buffer.size      = content_size;
  bn_buffer.timestamp = buffer.timestamp;
  bn_buffer.width     = width;
  bn_buffer.height    = height;
  bn_buffer.buffer_id = buffer.fd;
  bn_buffer.capacity  = buffer.size;

  MetaData meta_data;
  memset(&meta_data, 0x0, sizeof meta_data);
  meta_data.meta_flag = static_cast<uint32_t>(MetaParamType::kCamBufMetaData);
  meta_data.cam_buffer_meta_data = buffer.info;
  client_snapshot_cb_(buffer.camera_id, count, bn_buffer, meta_data);
}

TrackSource::TrackSource(const VideoTrackParams& params,
                         const sp<CameraInterface>& camera_intf)
    : track_params_(params),
      is_stop_(false),
      eos_acked_(false),
      active_overlays_(0),
      input_count_(0),
      count_(0),
      pending_encodes_per_frame_ratio_(0.0),
      frame_repeat_ts_prev_(0),
      frame_repeat_ts_curr_(0),
      enable_frame_repeat_(0),
      rescaler_(nullptr),
      connected_tocamera_port_(true),
      slave_track_source_(false) {

  BufferConsumerImpl<TrackSource> *impl;
  impl = new BufferConsumerImpl<TrackSource>(this);
  buffer_consumer_impl_ = impl;

  BufferProducerImpl<TrackSource> *producer_impl;
  producer_impl = new BufferProducerImpl<TrackSource>(this);
  buffer_producer_impl_ = producer_impl;

  assert(camera_intf.get() != nullptr);
  camera_interface_ = camera_intf;

  source_frame_rate_ = camera_intf->GetCameraStartParam().frame_rate;
  QMMF_INFO("%s:%s camera_frame_rate =%f", TAG, __func__, source_frame_rate_);
  input_frame_rate_ = source_frame_rate_;
  input_frame_interval_  = 1000000.0 / input_frame_rate_;
  output_frame_interval_ = 1000000.0 / track_params_.params.frame_rate;
  remaining_frame_skip_time_ = output_frame_interval_;
  QMMF_INFO("%s:%s: input_frame_interval_(%f) & output_frame_interval_(%f) & "
      "remaining_frame_skip_time_(%f)", TAG, __func__, input_frame_interval_,
      output_frame_interval_, remaining_frame_skip_time_);

  // TODO: There are issues related to how recorder service
  // treats the adb properties at runtime. Once it gets resolved,
  // the following lines for prop querying may be moved to
  // OnFrameAvailable.
  char prop_val[PROPERTY_VALUE_MAX];
  property_get(PROP_DEBUG_FPS, prop_val, "1");
  debug_fps_ = atoi(prop_val);

  QMMF_INFO("%s:%s: TrackSource (0x%p)", TAG, __func__, this);
}

TrackSource::~TrackSource() {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);

  QMMF_INFO("%s:%s: Exit(0x%p) ", TAG, __func__, this);
}

status_t TrackSource::AddConsumer(const sp<IBufferConsumer>& consumer) {
  std::lock_guard<std::mutex> lock(consumer_lock_);
  if (consumer.get() == nullptr) {
    QMMF_ERROR("%s:%s: Input consumer is nullptr", TAG, __func__);
    return BAD_VALUE;
  }

  buffer_producer_impl_->AddConsumer(consumer);
  consumer->SetProducerHandle(buffer_producer_impl_);

  QMMF_VERBOSE("%s:%s: Consumer(%p) has been added.", TAG, __func__,
      consumer.get());
  return NO_ERROR;
}

status_t TrackSource::RemoveConsumer(sp<IBufferConsumer>& consumer) {
  std::lock_guard<std::mutex> lock(consumer_lock_);

  if(buffer_producer_impl_->GetNumConsumer() == 0) {
    QMMF_ERROR("%s:%s: There are no connected consumers!", TAG, __func__);
    return INVALID_OPERATION;
  }

  buffer_producer_impl_->RemoveConsumer(consumer);

  return NO_ERROR;
}

int32_t TrackSource::GetMasterTrackId() {
  if (slave_track_source_) {
    return track_id_master_;
  } else {
    return TrackId();
  }
}

int32_t TrackSource::GetCameraPortId() {
  if (slave_track_source_) {
    return port_track_id_;
  } else {
    return TrackId();
  }
}

status_t TrackSource::InitCopy(shared_ptr<TrackSource> master_track_source,
                               const sp<CameraRescaler>& rescaler,
                               int32_t port_track_id,
                               int32_t track_id_master) {

  assert(master_track_source.get() != nullptr);
  master_track_ = master_track_source;

  connected_tocamera_port_ = (port_track_id == track_id_master);
  track_id_master_ = track_id_master;
  port_track_id_ = port_track_id;
  slave_track_source_ = true;

  if (rescaler.get() == nullptr) {
    QMMF_INFO("%s:%s Linked stream", TAG, __func__);
  } else {
    rescaler_ = rescaler;
  }

  return NO_ERROR;
}

status_t TrackSource::Init() {

  QMMF_DEBUG("%s:%s Enter track_id(%x)", TAG, __func__, TrackId());

  connected_tocamera_port_ = true;
  slave_track_source_ = false;
  rescaler_ = nullptr;
  master_track_ = nullptr;

  CameraStreamParam stream_param;
  memset(&stream_param, 0x0, sizeof stream_param);
  stream_param.cam_stream_dim.width  = track_params_.params.width;
  stream_param.cam_stream_dim.height = track_params_.params.height;
  if (track_params_.params.format_type == VideoFormat::kBayerRDI10BIT) {
    stream_param.cam_stream_format     = CameraStreamFormat::kRAW10;
  } else if (track_params_.params.format_type == VideoFormat::kBayerRDI12BIT) {
    stream_param.cam_stream_format     = CameraStreamFormat::kRAW12;
  } else {
    stream_param.cam_stream_format     = CameraStreamFormat::kNV21;
  }
  stream_param.frame_rate       = track_params_.params.frame_rate;
  stream_param.id               = track_params_.track_id;
  stream_param.low_power_mode   = track_params_.params.low_power_mode;

  assert(camera_interface_.get() != nullptr);
  auto ret = camera_interface_->CreateStream(stream_param,
                                             track_params_.extra_param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreateStream failed!!", TAG, __func__);
    return BAD_VALUE;
  }
  stream_param_ = stream_param;

  QMMF_INFO("%s:%s: TrackSource(0x%p)(%dx%d) and Camera Device Stream "
      " Created Succesffuly for track_id(%x)", TAG, __func__, this,
      track_params_.params.width, track_params_.params.height, TrackId());
  //TODO: Add mechanism to query the stream format from adaptor.
  ret = overlay_.Init(TargetBufferFormat::kYUVNV12);
  assert(ret == NO_ERROR);

  QMMF_DEBUG("%s:%s Exit track_id(%x)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackSource::DeInit() {

  QMMF_DEBUG("%s:%s Enter track_id(%x)", TAG, __func__, TrackId());
  assert(camera_interface_.get() != nullptr);
  status_t ret = NO_ERROR;

  if (slave_track_source_ == false) {
    ret = camera_interface_->DeleteStream(TrackId());
  }
  assert(ret == NO_ERROR);

  rescaler_ = nullptr;

  QMMF_DEBUG("%s:%s Exit track_id(%x)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackSource::StartTrack() {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());

  assert(camera_interface_.get() != nullptr);

  Mutex::Autolock lock(stop_lock_);
  is_stop_ = false;
  eos_acked_ = false;

  sp<IBufferConsumer> consumer;
  consumer = GetConsumerIntf();
  assert(consumer.get() != nullptr);

  status_t ret;
  if (rescaler_.get() != nullptr) {
    ret = master_track_->AddConsumer(rescaler_->GetCopyConsumerIntf());
    assert(ret == NO_ERROR);
    ret = rescaler_->AddConsumer(consumer);
    assert(ret == NO_ERROR);
  } else if (slave_track_source_ == true) {
    ret = master_track_->AddConsumer(consumer);
    assert(ret == NO_ERROR);
  }

  if (slave_track_source_ == false) {
    ret = camera_interface_->AddConsumer(TrackId(), consumer);
    assert(ret == NO_ERROR);
    ret = camera_interface_->StartStream(TrackId());
    assert(ret == NO_ERROR);
  }

  if (rescaler_.get() != nullptr) {
    ret = rescaler_->Start();
    assert(ret == NO_ERROR);
  }

  QMMF_DEBUG("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return NO_ERROR;
}

status_t TrackSource::StopTrack(bool is_force_cleanup) {
  status_t ret;

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  {
    Mutex::Autolock lock(stop_lock_);
    is_stop_ = true;
  }
  // Stop sequence when encoder is involved.
  // 1. Send EOS to encoder with last valid buffer. If frames_received_ queue is
  //    empty and read thread is waiting for buffers then wait till next buffer
  //    is available then send EOS to encoder. once EOS is notified encoder will
  //    stop calling read method.
  // 2. Once EOS is acknowledged by encoder stop the camera port, which in turn
  //    will break port's connection with TrackSource.
  // 3. Return all the buffers back to camera port from frames_received_ queue
  //    if any. there are very less chances frames_received_ list wil have
  //    buffers after we send EOS to encoder and before we break the connection
  //    between CameraPort and TrackSource, it is very important to check
  //    otherwise camera adaptor will never go in idle state and as a side
  //    effect delete camera stream would fail.
  // 4. Once all buffers are returned at input port of encoder it will notify
  //    the status:kPortIdle, and at this point client's stop method can be
  //    returned.

  bool wait = true;
  if (track_params_.params.format_type == VideoFormat::kYUV ||
      track_params_.params.format_type == VideoFormat::kBayerRDI10BIT ||
      track_params_.params.format_type == VideoFormat::kBayerRDI12BIT ||
      track_params_.params.format_type == VideoFormat::kBayerIdeal) {

    if (is_force_cleanup) {
      QMMF_INFO("%s:%s: track_id(%x) stopping in force mode!", TAG, __func__,
          TrackId());
      Mutex::Autolock autoLock(buffer_list_lock_);
      for (uint32_t i = 0; i < buffer_list_.size(); ++i) {
        StreamBuffer buffer = buffer_list_.valueAt(i);
        ReturnBufferToProducer(buffer);
      }
      buffer_list_.clear();
      wait = false;
    }
    // Encoder is not involved in this case.
    assert(camera_interface_.get() != nullptr);


    if (slave_track_source_ == false) {
      ret = camera_interface_->StopStream(TrackId());
      assert(ret == NO_ERROR);
    }

    sp<IBufferConsumer> consumer = GetConsumerIntf();
    if (rescaler_.get() != nullptr) {
      ret = rescaler_->Stop();
      assert(ret == NO_ERROR);
    }
    if (slave_track_source_ == false) {
      ret = camera_interface_->RemoveConsumer(TrackId(), consumer);
      assert(ret == NO_ERROR);
    }

    if (rescaler_.get() != nullptr) {
      ret = rescaler_->RemoveConsumer(consumer);
      assert(ret == NO_ERROR);
      ret = master_track_->RemoveConsumer(rescaler_->GetCopyConsumerIntf());
      assert(ret == NO_ERROR);
    } else if (slave_track_source_ == true) {
      ret = master_track_->RemoveConsumer(consumer);
      assert(ret == NO_ERROR);
    }
    QMMF_INFO("%s:%s: Pipe stop done(%x)", TAG, __func__, TrackId());
    {
      Mutex::Autolock autoLock(buffer_list_lock_);
      QMMF_DEBUG("%s:%s: track_id(%x) buffer_list_.size(%d)", TAG, __func__,
          TrackId(), buffer_list_.size());
      if (buffer_list_.size() == 0) {
        wait = false;
      }
    }
  } else {
      QMMF_DEBUG("%s:%s: track_id(%x), Wait for Encoder to return being encoded"
          " buffers!", TAG, __func__, TrackId());
  }
  std::unique_lock<std::mutex> lock(idle_lock_);
  std::chrono::nanoseconds wait_time(kWaitDuration);

  while (wait) {
    auto ret = wait_for_idle_.wait_for(lock, wait_time);
    if (ret == std::cv_status::timeout) {
        QMMF_ERROR("%s:%s: track_id(%x) StopTrack Timed out happend! Encoder"
        "failed to go in Idle state!", TAG, __func__, TrackId());
      return TIMED_OUT;
    }
    wait = false;
  }
  QMMF_DEBUG("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return NO_ERROR;
}

status_t TrackSource::NotifyPortEvent(PortEventType event_type,
                                      void* event_data) {

  QMMF_DEBUG("%s:%s Enter track_id(%x)", TAG, __func__, TrackId());
  if (event_type == PortEventType::kPortStatus) {
    CodecPortStatus status = *(static_cast<CodecPortStatus*>(event_data));
    if (status == CodecPortStatus::kPortStop) {
      // Encoder Received the EOS with valid last buffer successfully, stop the
      // camera stream and clear the received buffer queue.
      QMMF_INFO("%s:%s: track_id(%x) EOS acknowledged by Encoder!!", TAG,
          __func__, TrackId());
      ClearInputQueue();
      Mutex::Autolock lock(eos_lock_);
      eos_acked_ = true;
    } else if (status == CodecPortStatus::kPortIdle) {
      QMMF_INFO("%s:%s: track_id(%x) PortIdle acknowledged by Encoder!!", TAG,
          __func__, TrackId());
      ClearInputQueue();
      assert(camera_interface_.get() != nullptr);
      {
        std::unique_lock<std::mutex> lock(lock_);
        for(auto it : buffer_map_) {
          if (stream_buffer_map_.find(it.first) !=  stream_buffer_map_.end()) {
            QMMF_INFO("%s:%s: track_id(%x) fd: %d stream_id: %x", TAG, __func__,
                TrackId(), stream_buffer_map_[it.first].fd,
                stream_buffer_map_[it.first].stream_id);
            buffer_consumer_impl_->GetProducerHandle()->NotifyBufferReturned(
                stream_buffer_map_[it.first]);
          }
        }
      }
      status_t ret = NO_ERROR;
      if (slave_track_source_ == false) {
       ret = camera_interface_->StopStream(TrackId());
        assert(ret == NO_ERROR);
      }

      sp<IBufferConsumer> consumer = GetConsumerIntf();
      if (rescaler_.get() != nullptr) {
        ret = rescaler_->Stop();
        assert(ret == NO_ERROR);
      }
      if (slave_track_source_ == false) {
        ret = camera_interface_->RemoveConsumer(TrackId(), consumer);
        assert(ret == NO_ERROR);
      }

      if (rescaler_.get() != nullptr) {
        ret = rescaler_->RemoveConsumer(consumer);
        assert(ret == NO_ERROR);
        ret = master_track_->RemoveConsumer(rescaler_->GetCopyConsumerIntf());
        assert(ret == NO_ERROR);
      } else if (slave_track_source_ == true) {
        ret = master_track_->RemoveConsumer(consumer);
        assert(ret == NO_ERROR);
      }
      // All input port buffers from encoder are returned, Being encoded queue
      // should be zero at this point.
      assert(frames_being_encoded_.Size() == 0);
      QMMF_INFO("%s:%s: track_id(%x) All queued buffers are returned from"
          " encoder!!", TAG, __func__, TrackId());
      // wait_for_idle_ will not be needed once we make stop api as async.
      std::lock_guard<std::mutex> lock(idle_lock_);
      wait_for_idle_.notify_one();
    }
  }

  QMMF_DEBUG("%s:%s Exit track_id(%x)", TAG, __func__, TrackId());
  return NO_ERROR;
}

status_t TrackSource::GetBuffer(BufferDescriptor& buffer,
                                void* client_data) {

  QMMF_DEBUG("%s:%s Enter track_id(%x)", TAG, __func__, TrackId());
  bool timeout = false;

  {
    std::unique_lock<std::mutex> lock(lock_);
    std::chrono::nanoseconds wait_time(kWaitDuration);
    while (frames_received_.Size() == 0) {
      QMMF_DEBUG("%s:%s: track_id(%x) Wait for bufferr!!", TAG, __func__,
          TrackId());
      auto ret = wait_for_frame_.wait_for(lock, wait_time);
      if (ret == std::cv_status::timeout) {
          QMMF_ERROR("%s:%s: track_id(%x) Buffer Timed out happend! No buffers"
              "from Camera", TAG, __func__, TrackId());
        timeout = true;
        break;
      }
    }
    assert(timeout == false);

    QMMF_VERBOSE("%s:%s: track_id(%x) frames_received_.size(%d)", TAG, __func__,
        TrackId(), frames_received_.Size());

    auto stream_buffer = frames_received_.Begin();

    buffer.data =
        const_cast<void*>(reinterpret_cast<const void*>(stream_buffer->handle));
    buffer.fd = stream_buffer->fd;
    buffer.capacity = stream_buffer->frame_length;
    buffer.size = stream_buffer->filled_length;
    buffer.timestamp = stream_buffer->timestamp;
    buffer.flag = stream_buffer->flags;

    uint64_t ts_factor = (stream_buffer->timestamp - frame_repeat_ts_prev_) /
        stream_buffer->encodes_per_frame_count - kTsFactor;
    if (stream_buffer->encodes_per_frame_count !=
        stream_buffer->pending_encodes_per_frame) {
      buffer.timestamp = frame_repeat_ts_curr_ + ts_factor;
    }
    frame_repeat_ts_curr_ = buffer.timestamp;

    stream_buffer->pending_encodes_per_frame--;
    if (!stream_buffer->pending_encodes_per_frame) {
      stream_buffer->needs_return = true;
      frame_repeat_ts_prev_ = stream_buffer->timestamp;
    }
    frames_being_encoded_.PushBack((*stream_buffer));

    if (!stream_buffer->pending_encodes_per_frame) {
      frames_received_.Erase(frames_received_.Begin());
    }
  }

  if (IsStop()) {
    QMMF_DEBUG("%s:%s: track_id(%x) Send EOS to Encoder!", TAG, __func__,
        TrackId());
    // TODO defile EOS flag in AVCodec to delete EOS.
    return -1;
  }
  QMMF_DEBUG("%s:%s Exit track_id(%x)", TAG, __func__, TrackId());
  return NO_ERROR;
}

status_t TrackSource::ReturnBuffer(BufferDescriptor& buffer,
                                   void* client_data) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());

  QMMF_VERBOSE("%s:%s: track_id(%x) frames_being_encoded_.size(%d)", TAG,
      __func__, TrackId(), frames_being_encoded_.Size());

  bool found = false;

  std::unique_lock<std::mutex> lock(lock_);
  auto iter = frames_being_encoded_.Begin();
  for (; iter != frames_being_encoded_.End(); ++iter) {
    if ((*iter).handle ==  buffer.data) {
      QMMF_VERBOSE("%s:%s: Buffer found in frames_being_encoded_ list!", TAG,
          __func__);

      if (iter->needs_return) {
        ReturnBufferToProducer((*iter));
      }

      frames_being_encoded_.Erase(iter);
      found = true;
      break;
    }
  }
  assert(found == true);
  QMMF_VERBOSE("%s:%s: frames_being_encoded_.Size(%d)", TAG, __func__,
      frames_being_encoded_.Size());

  QMMF_DEBUG("%s:%s Exit track_id(%x)", TAG, __func__, TrackId());
  return NO_ERROR;
}

void TrackSource::OnFrameAvailable(StreamBuffer& buffer) {

  QMMF_VERBOSE("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  {
    std::unique_lock<std::mutex> lock(lock_);
    buffer_map_.insert(std::make_pair(buffer.handle, 1));
    stream_buffer_map_.emplace(buffer.handle, buffer);
  }

#ifdef NO_FRAME_PROCESS
  ReturnBufferToProducer(buffer);
  return;
#endif

  {
    Mutex::Autolock lock(eos_lock_);
    if (eos_acked_ && IsStop()) {
      auto track_format = track_params_.params.format_type;
      if (track_format == VideoFormat::kAVC
          || track_format == VideoFormat::kHEVC) {
        // Return buffer if track is stoped and EOS is acknowledged by AVCodec.
        QMMF_INFO("%s:%s: Track(%x) Stoped and eos is acked!", TAG, __func__,
          TrackId());
        std::unique_lock<std::mutex> lock(lock_);
        ReturnBufferToProducer(buffer);
        return;
      }
    }
  }

  // Dynamic FPS measurement of source (Camera)
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  uint64_t time_diff = (uint64_t)((tv.tv_sec * 1000000 + tv.tv_usec) -
                       (input_prevtv_.tv_sec * 1000000 +
                       input_prevtv_.tv_usec));
  input_count_++;
  if (time_diff >= FPS_TIME_INTERVAL) {
    float framerate = (input_count_ * 1000000) / (float)time_diff;
    bool is_first_time = (framerate <= 1.0);

    // Re-calculate input and output frame intervals if input framerate
    // is different from its previous value
    if (!(is_first_time) &&
        (fabs(input_frame_rate_ - framerate) >= FPS_CHANGE_THRESHOLD)) {
      Mutex::Autolock autoLock(frame_skip_lock_);
      QMMF_INFO("%s:%s: track_id(%x) adjusting fps from (%0.2f) to (%0.2f)",
                TAG, __func__, TrackId(), input_frame_rate_, framerate);
      input_frame_rate_ = framerate;
      input_frame_interval_  = 1000000.0 / input_frame_rate_;

      // Track frame rate can be higher than input frame rate only if
      // frame_repeat is enabled
      if (input_frame_rate_ < track_params_.params.frame_rate) {
        if (enable_frame_repeat_) {
          output_frame_interval_ = 1000000.0 / track_params_.params.frame_rate;
        } else {
          output_frame_interval_ = 1000000.0 / input_frame_rate_;
        }
      } else {
        output_frame_interval_ = 1000000.0 / track_params_.params.frame_rate;
      }
    }
    if (debug_fps_ & kDebugSourceTrackFps) {
      QMMF_INFO("%s:%s: camera id %d, track_id(%x): source fps: = %0.2f", TAG,
                 __func__,track_params_.params.camera_id, TrackId(), framerate);
    }
    input_prevtv_ = tv;
    input_count_ = 0;
  }

  // Return buffer back to camera if frameskip is enabled and valid.
  if ((IsEnableFrameSkip()) && IsFrameSkip()) {
    // Skip frame to adjust fps.
    if (debug_fps_ & kDebugFrameSkip) {
      QMMF_INFO("%s:%s:cam_id(%d),track_id(%x),skip frame %u,fps %0.2f",
                TAG, __func__,track_params_.params.camera_id,
                TrackId(),buffer.frame_number,input_frame_rate_);
    }
    std::unique_lock<std::mutex> lock(lock_);
    ReturnBufferToProducer(buffer);
    return;
  }

  buffer.needs_return = false;
  buffer.pending_encodes_per_frame = CalculateEncodesPerFrame();
  buffer.encodes_per_frame_count = buffer.pending_encodes_per_frame;

  QMMF_VERBOSE("%s:%s: track_id(%d) numInts = %d", TAG, __func__, TrackId(),
      buffer.handle->numInts);
  for (int32_t i = 0; i < buffer.handle->numInts; i++) {
    QMMF_VERBOSE("%s:%s: track_id(%x) data[%d] =%d", TAG, __func__, TrackId(),
        i , buffer.handle->data[i]);
  }

  QMMF_VERBOSE("%s:%s: track_id(%x) ion_fd = %d", TAG, __func__, TrackId(),
      buffer.fd);
  QMMF_VERBOSE("%s:%s: track_id(%x) size = %d", TAG, __func__, TrackId(),
      buffer.size);

  if (active_overlays_ > 0) {
    OverlayTargetBuffer overlay_buf;
    //TODO: get format from streamBuffer.
    overlay_buf.format    = TargetBufferFormat::kYUVNV12;
    overlay_buf.width     = buffer.info.plane_info[0].width;
    overlay_buf.height    = buffer.info.plane_info[0].height;
    overlay_buf.ion_fd    = buffer.fd;
    overlay_buf.frame_len = buffer.size;
    overlay_.ApplyOverlay(overlay_buf);
  }
#ifdef ENABLE_FRAME_DUMP
  DumpYUV(buffer);
#endif

  {
    std::unique_lock<std::mutex> lock(lock_);
    auto val = buffer_map_.at(buffer.handle);
    buffer_map_[buffer.handle] = buffer_producer_impl_->GetNumConsumer() + val;
    if (buffer_producer_impl_->GetNumConsumer() > 0) {
      buffer_producer_impl_->NotifyBuffer(buffer);
    }
  }

  // If format type is YUV or BAYER then give callback from this point, do not
  // feed buffer to Encoder.
  if (track_params_.params.format_type == VideoFormat::kYUV ||
      track_params_.params.format_type == VideoFormat::kBayerRDI10BIT ||
      track_params_.params.format_type == VideoFormat::kBayerRDI12BIT ||
      track_params_.params.format_type == VideoFormat::kBayerIdeal) {

    if (IsStop()) {
      QMMF_DEBUG("%s:%s: track_id(%x) Stop is triggred, Stop giving raw buffer"
          " to client!", TAG, __func__, TrackId());
      std::unique_lock<std::mutex> lock(lock_);
      ReturnBufferToProducer(buffer);
      return;
    }

    BnBuffer bn_buffer;
    memset(&bn_buffer, 0x0, sizeof bn_buffer);
    bn_buffer.ion_fd            = buffer.fd;
    bn_buffer.size              = buffer.size;
    bn_buffer.timestamp         = buffer.timestamp;
    bn_buffer.width             = buffer.info.plane_info[0].width;
    bn_buffer.height            = buffer.info.plane_info[0].height;
    bn_buffer.buffer_id         = buffer.fd;
    bn_buffer.flag              = 0x10;
    bn_buffer.capacity          = buffer.size;

    // Buffers from this list used for YUV callback.
    {
      Mutex::Autolock autoLock(buffer_list_lock_);
      buffer_list_.add(buffer.fd, buffer);
    }
    std::vector<BnBuffer> bn_buffers;
    bn_buffers.push_back(bn_buffer);

    MetaData meta_data {};
    meta_data.meta_flag = static_cast<uint32_t>(MetaParamType::kCamBufMetaData);
    meta_data.meta_flag |= static_cast<uint32_t>
        (MetaParamType::kCamMetaFrameNumber);
    meta_data.cam_buffer_meta_data  = buffer.info;
    meta_data.cam_meta_frame_number = buffer.frame_number;

    std::vector<MetaData> meta_buffers;
    meta_buffers.push_back(meta_data);

    track_params_.data_cb(bn_buffers, meta_buffers);
  } else {
    // Push buffers into encoder queue.
    PushFrameToQueue(buffer);
  }
}

status_t TrackSource::ReturnTrackBuffer(std::vector<BnBuffer>& bn_buffers) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  assert(bn_buffers.size() > 0);
  assert(buffer_consumer_impl_ != nullptr);

  std::unique_lock<std::mutex> lock(lock_);
  for (size_t i = 0; i < bn_buffers.size(); ++i) {
    QMMF_VERBOSE("%s:%s: track_id(%x) bn_buffers[%d].ion_fd=%d", TAG, __func__,
        TrackId(), i, bn_buffers[i].ion_fd);
    {
      Mutex::Autolock autoLock(buffer_list_lock_);
      int32_t idx = buffer_list_.indexOfKey(bn_buffers[i].ion_fd);
      QMMF_DEBUG("%s:%s: track_id(%x) Buffer fd(%d) found in list", TAG,
          __func__, TrackId(), bn_buffers[i].ion_fd);
      assert(idx >= 0);
      StreamBuffer buffer = buffer_list_.valueFor(bn_buffers[i].ion_fd);
      ReturnBufferToProducer(buffer);
      buffer_list_.removeItem(bn_buffers[i].ion_fd);
    }
  }
  if (IsStop()) {
    if (buffer_list_.size() > 0) {
      QMMF_INFO("%s:%s: track_id(%x) Stop is triggered, but still num raw "
          "buffers(%d) are with client!", TAG, __func__, TrackId(),
          buffer_list_.size());
    } else {
      // wait_for_idle_ will not be needed once we make stop api as async.
      QMMF_INFO("%s:%s: track_id(%x) Stop is triggered, all raw buffers are"
          " returned from client!", TAG, __func__, TrackId());
      std::lock_guard<std::mutex> lock(idle_lock_);
      wait_for_idle_.notify_one();
    }
  }
  QMMF_VERBOSE("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return NO_ERROR;
}

void TrackSource::PushFrameToQueue(StreamBuffer& buffer) {

  QMMF_VERBOSE("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());

  // Dynamic FPS measurement of Track
  if (debug_fps_ & kDebugTrackFps) {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    uint64_t time_diff = (uint64_t)((tv.tv_sec * 1000000 + tv.tv_usec) -
                         (prevtv_.tv_sec * 1000000 + prevtv_.tv_usec));
    count_++;
    if (time_diff >= FPS_TIME_INTERVAL) {
      float framerate = (count_ * 1000000) / (float)time_diff;
      QMMF_INFO("%s:%s: track_id(%x):track fps: = %0.2f", TAG, __func__,
                TrackId(), framerate);
      prevtv_ = tv;
      count_ = 0;
    }
  }
  frames_received_.PushBack(buffer);
  QMMF_DEBUG("%s:%s: track_id(%x) frames_received.size(%d)", TAG, __func__,
      TrackId(), frames_received_.Size());
  wait_for_frame_.notify_one();

  QMMF_VERBOSE("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
}

bool TrackSource::IsStop() {

  QMMF_VERBOSE("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  Mutex::Autolock lock(stop_lock_);
  QMMF_VERBOSE("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return is_stop_;
}

void TrackSource::ClearInputQueue() {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  std::unique_lock<std::mutex> lock(lock_);
  // Once connection is broken b/w port and trackSoure there is no chance to
  // get new buffers in frames_received_ queue.
  uint32_t size = frames_received_.Size();
  QMMF_INFO("%s:%s: track_id(%x):(%d) buffers to return from frames_received_",
      TAG, __func__, TrackId(), size);
  assert(buffer_consumer_impl_->GetProducerHandle().get() != nullptr);
  auto iter = frames_received_.Begin();
  for (; iter != frames_received_.End(); ++iter) {
    ReturnBufferToProducer((*iter));
  }
  frames_received_.Clear();
  QMMF_DEBUG("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
}

status_t TrackSource::CreateOverlayObject(OverlayParam *param,
                                          uint32_t *overlay_id) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  uint32_t id;
  auto ret = overlay_.CreateOverlayItem(*param, &id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: createOverlayItem failed!", TAG, __func__);
    return BAD_VALUE;
  }
  *overlay_id = id;
  QMMF_INFO("%s:%s: OverlayItem of type(%d) created! id(%d)", TAG, __func__,
      param->type, id);
  QMMF_DEBUG("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackSource::DeleteOverlayObject(const uint32_t overlay_id) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  auto ret = overlay_.DeleteOverlayItem(overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: deleteOverlayItem failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackSource::GetOverlayObjectParams(const uint32_t overlay_id,
                                             OverlayParam &param) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  auto ret = overlay_.GetOverlayParams(overlay_id, param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: getOverlayItemParams failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackSource::UpdateOverlayObjectParams(const uint32_t overlay_id,
                                                OverlayParam *param) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  auto ret = overlay_.UpdateOverlayParams(overlay_id, *param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: updateOverlayParams failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_DEBUG("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackSource::SetOverlayObject(const uint32_t overlay_id) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  auto ret = overlay_.EnableOverlayItem(overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: enableOverlayItem failed!", TAG, __func__);
    return BAD_VALUE;
  }
  ++active_overlays_;
  QMMF_DEBUG("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackSource::RemoveOverlayObject(const uint32_t overlay_id) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  auto ret = overlay_.DisableOverlayItem(overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: disableOverlayItem failed!", TAG, __func__);
    return BAD_VALUE;
  }
  --active_overlays_;
  QMMF_DEBUG("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return ret;
}

void TrackSource::UpdateFrameRate(const float frame_rate) {

  Mutex::Autolock autoLock(frame_skip_lock_);
  assert(frame_rate > 0.0f);

  if (fabs(track_params_.params.frame_rate - frame_rate) > 0.1f) {
      QMMF_INFO("%s:%s: track_id(%x) Track fps changed from (%5.2f) to (%5.2f)",
          TAG, __func__, TrackId(), track_params_.params.frame_rate, frame_rate);
    track_params_.params.frame_rate = frame_rate;
    output_frame_interval_ = 1000000.0 / frame_rate;
  }
}

void TrackSource::EnableFrameRepeat(const bool enable_frame_repeat) {

  std::lock_guard<std::mutex> lock(frame_repeat_lock_);
  enable_frame_repeat_ = enable_frame_repeat;
}

// Enable frameskip only when dynamic fps is FRAME_SKIP_THRESHOLD_PERCENT
// higher than required and this frame is NOT a stop condition. In STOP
// condition,frame skip logic is bypassed as the buffer consumer may wait
// for the last bufferas part of stop processing. Skipping frames may result
// in timeouts in the consumer.
bool TrackSource::IsEnableFrameSkip() {
  bool is_enable = false;
  if ((!IsStop()) &&
     ((input_frame_rate_ - track_params_.params.frame_rate) >
     (track_params_.params.frame_rate * FRAME_SKIP_THRESHOLD_PERCENT))) {
      is_enable = true;
  }
  return is_enable;
}

bool TrackSource::IsFrameSkip() {

  Mutex::Autolock autoLock(frame_skip_lock_);
  bool skip;
  remaining_frame_skip_time_ -= input_frame_interval_;
  if (0 >= remaining_frame_skip_time_) {
    skip = false;
    remaining_frame_skip_time_ += output_frame_interval_;
  } else {
    skip = true;
  }
  return skip;
}

uint32_t TrackSource::CalculateEncodesPerFrame() {

  std::lock_guard<std::mutex> lock(frame_repeat_lock_);
  uint32_t encodes_per_frame;

  if ((!enable_frame_repeat_) ||
      (input_frame_rate_ >= track_params_.params.frame_rate)) {
    return 1;
  }

  encodes_per_frame = static_cast<uint32_t>(input_frame_interval_) /
      output_frame_interval_;
  pending_encodes_per_frame_ratio_ +=
      (input_frame_interval_ / output_frame_interval_) - encodes_per_frame;

  if (pending_encodes_per_frame_ratio_ >= 1.0) {
    pending_encodes_per_frame_ratio_ -= 1.0;
    ++encodes_per_frame;
  }

  return encodes_per_frame;
}

void TrackSource::ReturnBufferToProducer(StreamBuffer& buffer) {
  QMMF_DEBUG("%s:%s: Enter track_id(%x) fd: %d ts: %lld", TAG, __func__,
      TrackId(), buffer.fd, buffer.timestamp);

  if (buffer_map_.find(buffer.handle) == buffer_map_.end()) {
    QMMF_INFO("%s:%s: Error track_id(%x) fd: %d ts: %lld", TAG, __func__,
         TrackId(), buffer.fd, buffer.timestamp);

  } else {
    QMMF_DEBUG("%s:%s: Buffer is back to Producer Intf,buffer(0x%p) RefCount=%d",
        TAG, __func__, buffer.handle, buffer_map_.at(buffer.handle));
    if(buffer_map_.at(buffer.handle) == 1) {
      buffer_map_.erase(buffer.handle);
      // Return buffer back to actual owner.
      if (stream_buffer_map_.find(buffer.handle) != stream_buffer_map_.end()) {
        stream_buffer_map_.erase(buffer.handle);
      }
      buffer_consumer_impl_->GetProducerHandle()->NotifyBufferReturned(buffer);
    } else {
      // Hold this buffer, do not return until its ref count is 1.
      uint32_t value = buffer_map_.at(buffer.handle);
      buffer_map_[buffer.handle] = --value;
      if (buffer_producer_impl_->GetNumConsumer() == 0) {
        stream_buffer_map_.erase(buffer.handle);
        buffer_consumer_impl_->GetProducerHandle()->NotifyBufferReturned(buffer);
      }
    }
  }
}

void TrackSource::NotifyBufferReturned(StreamBuffer& buffer) {
  QMMF_DEBUG("%s:%s: Enter track_id(%x) fd: %d ts: %lld", TAG, __func__,
      TrackId(), buffer.fd, buffer.timestamp);
  std::unique_lock<std::mutex> lock(lock_);
  ReturnBufferToProducer(buffer);
}

#ifdef ENABLE_FRAME_DUMP
status_t TrackSource::DumpYUV(StreamBuffer& buffer) {

  static uint32_t id;
  ++id;
  // Dump every 100th frame.
  if (id == 100) {

    void *buf_vaaddr = mmap(nullptr, buffer.size, PROT_READ  | PROT_WRITE,
                            MAP_SHARED, buffer.fd, 0);
    assert(buf_vaaddr != nullptr);

    String8 file_path;
    size_t written_len;
    file_path.appendFormat(FRAME_DUMP_PATH"/track_%x_%lld.yuv",
        TrackId(), buffer.timestamp);

    FILE *file = fopen(file_path.string(), "w+");
    if (!file) {
      QMMF_ERROR("%s:%s: Unable to open file(%s)", TAG, __func__,
          file_path.string());
      goto FAIL;
    }
    written_len = fwrite(buf_vaaddr, sizeof(uint8_t), buffer.size,
        file);
    QMMF_INFO("%s:%s: written_len =%d", TAG, __func__, written_len);

    if (buffer.size != written_len) {
      QMMF_ERROR("%s:%s: Bad Write error (%d):(%s)\n", TAG, __func__, errno,
          strerror(errno));
        goto FAIL;
    }
    QMMF_INFO("%s:%s: Buffer(0x%p) Size(%u) Stored(%s)\n",__func__,
        buf_vaaddr, written_len, file_path.string());

FAIL:
    if (file != nullptr) {
      fclose(file);
    }
    if (buf_vaaddr != nullptr) {
      munmap(buf_vaaddr, buffer.size);
      buf_vaaddr = nullptr;
    }
    id = 0;
  }
}
#endif

}; //namespace recorder

}; //namespace qmmf
