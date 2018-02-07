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

#define LOG_TAG "TimeLapseTest"

#include "qmmf_video_time_lapse.h"
#include <camera/CameraMetadata.h>
#include <cutils/properties.h>
#include <qmmf-sdk/qmmf_avcodec.h>

using std::vector;
using std::shared_ptr;
using std::make_shared;
using std::thread;
using std::mutex;
using std::lock_guard;
using std::unique_lock;
using std::ofstream;
using std::streampos;
using std::string;
using std::to_string;
using std::this_thread::sleep_for;

using namespace qmmf;
using namespace qmmf::recorder;
using namespace qmmf::avcodec;
using namespace android;

#define TIMELAPSE_DEBUG 0

const uint32_t TimeLapse::kLPMTrackId = 1;
const uint32_t TimeLapse::kLPMTrackWidth = 640;
const uint32_t TimeLapse::kLPMTrackHeight = 480;
const uint32_t TimeLapse::kThresholdTime = 3000;
const uint32_t TimeLapse::kSanpShotBufferReturnedWaitTimeOut = 2;  // 2 sec

const uint32_t EncoderSource::kEOSFlag = 1;

TimeLapse::TimeLapse(const TimeLapseParams &params)
    : params_(params),
      video_time_lapse_mode_(VideoTimeLapseMode::kModeOne),
      session_id_(0),
      ion_device_(-1),
      snapshot_count_(0),
      atomic_stop_(false) {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter ", __func__);

  ion_device_ = open("/dev/ion", O_RDONLY);
  if (ion_device_ <= 0) {
    ALOGE("%s: Ion dev open failed %s", __func__, strerror(errno));
    ion_device_ = -1;
    assert(ion_device_ > 0);
  }

  char prop_val[PROPERTY_VALUE_MAX];
  property_get(PROP_DUMP_YUV_SNAPSHOT, prop_val, "0");
  is_dump_yuv_snapshot_enabled_ = (atoi(prop_val) == 0) ? false : true;
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
}

TimeLapse::~TimeLapse() {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter ", __func__);

  if (avcodec_) delete avcodec_;

  close(ion_device_);
  ion_device_ = -1;
  ALOGD_IF(TIMELAPSE_DEBUG, "%s:: Exit ", __func__);
}

int32_t TimeLapse::Start() {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);
  RecorderCb recorder_status = {
      [](EventType event_type, void *event_data, size_t event_data_size) {
        ALOGI("%s: Rececived Event Type =%d ", __func__,
              static_cast<int32_t>(event_type));
      }};

  int32_t ret = recorder_.Connect(recorder_status);
  if (NO_ERROR != ret) {
    ALOGE("%s: Connect Failed", __func__);
    return ret;
  }

  ret = StartAVCodec();
  if (NO_ERROR != ret) {
    ALOGE("%s: StartAVCodec Failed", __func__);
    return ret;
  }

  if (static_cast<TimeLapseMode>(params_.time_lapse_mode) ==
      TimeLapseMode::kVideoTimeLapse) {
    if (params_.time_lapse_interval <= kThresholdTime) {
      video_time_lapse_mode_ = VideoTimeLapseMode::kModeOne;
    } else {
      video_time_lapse_mode_ = VideoTimeLapseMode::kModeTwo;
    }
    time_lapse_thread_ = thread(TimeLapse::TimeLapseModeThread, this);
  } else if (static_cast<TimeLapseMode>(params_.time_lapse_mode) ==
             TimeLapseMode::kPhotoTimeLapse) {
    ALOGE("%s: Currently JPEG Mode is not supported", __func__);
    return INVALID_OPERATION;
  }

  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
  return ret;
}

void TimeLapse::TimeLapseModeThread(TimeLapse *timelapse) {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);

  if (timelapse->video_time_lapse_mode_ == VideoTimeLapseMode::kModeOne) {
    timelapse->StartTimeLapseModeOne();
  } else if (timelapse->video_time_lapse_mode_ ==
             VideoTimeLapseMode::kModeTwo) {
    timelapse->StartTimeLapseModeTwo();
  }

  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
}

int32_t TimeLapse::StartTimeLapseModeOne() {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);
  int32_t ret;

  ret = StartCamera();
  if (NO_ERROR != ret) {
    ALOGE("%s: StartCamera Failed", __func__);
    return ret;
  }

  ret = CreateSession();
  if (NO_ERROR != ret) {
    ALOGE("%s: CreateSession Failed", __func__);
    return ret;
  }

  ret = CreateLPMTrack();
  if (NO_ERROR != ret) {
    ALOGE("%s: CreateLPMTrack Failed", __func__);
    return ret;
  }

  ret = StartSession();
  if (NO_ERROR != ret) {
    ALOGE("%s: StartSession Failed", __func__);
    return ret;
  }

  while (!atomic_stop_) {
    ret = TakeYUVSnapshotandEnqueuetoEncoder();
    if (NO_ERROR != ret) {
      ALOGE("%s: Take YUV Snapshtot Failed", __func__);
      break;
    }

    sleep_for(std::chrono::milliseconds(params_.time_lapse_interval));
  }

  ret = StopSession();
  if (NO_ERROR != ret) {
    ALOGE("%s: StopSession Failed", __func__);
    return ret;
  }

  ret = DeleteLPMTrack();
  if (NO_ERROR != ret) {
    ALOGE("%s: DeleteLPMTrack Failed", __func__);
    return ret;
  }

  ret = DeleteSession();
  if (NO_ERROR != ret) {
    ALOGE("%s: DeleteSession Failed", __func__);
    return ret;
  }

  ret = StopCamera();
  if (NO_ERROR != ret) {
    ALOGE("%s: StopCamera Failed", __func__);
    return ret;
  }

  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
  return ret;
}

int32_t TimeLapse::StartTimeLapseModeTwo() {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);
  int32_t ret;

  while (!atomic_stop_) {
    auto pipeline_creation_start = std::chrono::high_resolution_clock::now();

    ret = StartCamera();
    if (NO_ERROR != ret) {
      ALOGE("%s: StartCamera Failed", __func__);
      return ret;
    }

    ret = CreateSession();
    if (NO_ERROR != ret) {
      ALOGE("%s: CreateSession Failed", __func__);
      return ret;
    }

    ret = CreateLPMTrack();
    if (NO_ERROR != ret) {
      ALOGE("%s: CreateLPMTrack Failed", __func__);
      return ret;
    }

    ret = StartSession();
    if (NO_ERROR != ret) {
      ALOGE("%s: StartSession Failed", __func__);
      return ret;
    }

    snapshot_buffer_returnerd_future_ =
        snapshot_buffer_returned_promise_.get_future();

    ret = TakeYUVSnapshotandEnqueuetoEncoder();
    if (NO_ERROR != ret) {
      ALOGE("%s: Take YUV Snapshtot Failed", __func__);
    }

    auto status = snapshot_buffer_returnerd_future_.wait_for(
        std::chrono::seconds(kSanpShotBufferReturnedWaitTimeOut));
    if (status == std::future_status::timeout) {
      ALOGE("%s: Sanpshot buffer not returned to recorder service", __func__);
    }
    snapshot_buffer_returned_promise_ = std::promise<int32_t>();

    ret = StopSession();
    if (NO_ERROR != ret) {
      ALOGE("%s: StopSession Failed", __func__);
      return ret;
    }

    ret = DeleteLPMTrack();
    if (NO_ERROR != ret) {
      ALOGE("%s: DeleteLPMTrack Failed", __func__);
      return ret;
    }

    ret = DeleteSession();
    if (NO_ERROR != ret) {
      ALOGE("%s: DeleteSession Failed", __func__);
      return ret;
    }

    ret = StopCamera();
    if (NO_ERROR != ret) {
      ALOGE("%s: StopCamera Failed", __func__);
      return ret;
    }

    auto pipeline_creation_end = std::chrono::high_resolution_clock::now();
    auto diff = std::chrono::duration_cast<::std::chrono::milliseconds>(
                    pipeline_creation_end - pipeline_creation_start)
                    .count();
    ALOGD_IF(TIMELAPSE_DEBUG, "%s: Pipeline reconfiguration time =%lld ms",
             __func__, diff);

    sleep_for(std::chrono::milliseconds(params_.time_lapse_interval - diff));
  }

  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
  return ret;
}

int32_t TimeLapse::StartCamera() {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);
  int32_t ret;
  CameraStartParam camera_start_params{false, 0, 0, 0, 30, 0};

  ret = recorder_.StartCamera(params_.camera_id, camera_start_params);
  if (NO_ERROR != ret) {
    ALOGE("%s: StartCamera Failed", __func__);
    return ret;
  }

  if (!ResolutionSupported(params_.width, params_.height)) {
    ALOGE("%s: resolution %dx%d not supported!\n", __func__, params_.width,
          params_.height);
  }

  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
  return ret;
}

bool TimeLapse::ResolutionSupported(uint32_t width, uint32_t height) {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);
  int32_t ret;

  std::vector<android::CameraMetadata> meta_array;
  camera_metadata_entry_t entry;
  android::CameraMetadata meta;

  ret = recorder_.GetDefaultCaptureParam(params_.camera_id, meta);
  if (NO_ERROR != ret) {
    ALOGE("%s: Unable to query default capture parameters!\n", __func__);
    return false;
  }

  bool res_supported = false;
  // Check Supported Raw YUV snapshot resolutions.
  if (meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
    entry = meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
    for (uint32_t i = 0; i < entry.count; i += 4) {
      if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == entry.data.i32[i]) {
        if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
            entry.data.i32[i + 3]) {
          if (width == static_cast<uint32_t>(entry.data.i32[i + 1]) &&
              height == static_cast<uint32_t>(entry.data.i32[i + 2])) {
            res_supported = true;
            break;
          }
        }
      }
    }
  }

  return res_supported;
}

int32_t TimeLapse::StopCamera() {
  return recorder_.StopCamera(params_.camera_id);
}

int32_t TimeLapse::CreateSession() {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);
  int32_t ret;
  SessionCb session_status_cb;
  session_status_cb.event_cb = {
      [](EventType event_type, void *event_data, size_t event_data_size) {
        ALOGI("%s: Rececived Event Type =%d ", __func__,
              static_cast<int32_t>(event_type));
      }};

  ret = recorder_.CreateSession(session_status_cb, &session_id_);
  assert(session_id_ > 0);
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
  return ret;
}

int32_t TimeLapse::DeleteSession() {
  int32_t ret = recorder_.DeleteSession(session_id_);
  session_id_ = 0;
  return ret;
}

int32_t TimeLapse::StartSession() {
  return recorder_.StartSession(session_id_);
}

int32_t TimeLapse::StopSession() {
  return recorder_.StopSession(session_id_, true);
}

int32_t TimeLapse::StartAVCodec() {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);
  int32_t ret;

  ret = SetupAVCodec(params_);
  if (NO_ERROR != ret) {
    ALOGE("%s: ConfigureAVCodec Failed", __func__);
  }

  ret = avcodec_->StartCodec();
  if (NO_ERROR != ret) {
    ALOGE("%s: StartCodec Failed", __func__);
  }

  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
  return ret;
}

int32_t TimeLapse::StopAVCodec() {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);
  int32_t ret;

  encoder_source_->SetEOS();

  ret = avcodec_->StopCodec();
  if (NO_ERROR != ret) {
    ALOGE("%s: StopCodec Failed", __func__);
    return ret;
  }

  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
  return ret;
}

int32_t TimeLapse::Stop() {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);
  int32_t ret;

  ret = StopAVCodec();
  if (NO_ERROR != ret) {
    ALOGE("%s: StopAVCodec Failed", __func__);
  }

  atomic_stop_ = true;

  ret = ReleaseBuffer();
  if (NO_ERROR != ret) {
    ALOGE("%s: failed to release allocated buffers", __func__);
  }

  time_lapse_thread_.join();

  ret = recorder_.Disconnect();
  if (NO_ERROR != ret) {
    ALOGE("%s: Disconnect Failed", __func__);
    return ret;
  }
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
  return ret;
}

int32_t TimeLapse::CreateLPMTrack() {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);

  if (!ResolutionSupported(kLPMTrackWidth, kLPMTrackHeight)) {
    ALOGE("%s: resolution %dx%d not supported!\n", __func__, kLPMTrackWidth,
          kLPMTrackHeight);
  }

  VideoTrackCreateParam video_track_param{params_.camera_id, VideoFormat::kYUV,
                                          kLPMTrackWidth, kLPMTrackHeight, 30};

  TrackCb video_track_cb;
  video_track_cb.data_cb = {[&](uint32_t track_id,
                                std::vector<BufferDescriptor> buffers,
                                std::vector<MetaData> meta_buffers) {
    recorder_.ReturnTrackBuffer(session_id_, kLPMTrackId, buffers);
  }};

  video_track_cb.event_cb = {[](uint32_t track_id, EventType event_type,
                                void *event_data, size_t event_data_size) {}};

  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
  return recorder_.CreateVideoTrack(session_id_, kLPMTrackId, video_track_param,
                                    video_track_cb);
}

int32_t TimeLapse::DeleteLPMTrack() {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);
  return recorder_.DeleteVideoTrack(session_id_, kLPMTrackId);
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
}

int32_t TimeLapse::TakeYUVSnapshotandEnqueuetoEncoder() {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);

  ImageParam image_param{params_.width, params_.height, 0, ImageFormat::kNV12};

  std::vector<android::CameraMetadata> meta_array;
  ImageCaptureCb cb;
  cb = {[&](uint32_t camera_id, uint32_t image_count, BufferDescriptor buffer,
            MetaData meta_data) {
    YUVSnapshotCb(camera_id, image_count, buffer, meta_data);
  }};

  int32_t ret =
      recorder_.CaptureImage(params_.camera_id, image_param, 1, meta_array, cb);
  if (NO_ERROR != ret) {
    ALOGE("%s: CaptureImage Failed", __func__);
  }

  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
  return ret;
}

void TimeLapse::YUVSnapshotCb(uint32_t camera_id, uint32_t image_sequence_count,
                              BufferDescriptor buffer, MetaData meta_data) {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);

  if (atomic_stop_) {
    int32_t ret = 0;
    // Return buffer back to recorder service.
    ret = recorder_.ReturnImageCaptureBuffer(params_.camera_id, buffer);
    if (NO_ERROR != ret) {
      ALOGE("%s: ReturnImageCaptureBuffer failed", __func__);
    }
  }

  if (is_dump_yuv_snapshot_enabled_) {
    DumpYUVSnapShot(buffer);
  }

  encoder_source_->ConsumeBuffer(buffer);
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
}

void TimeLapse::ReturnYUVSnapshotBuffer(BufferDescriptor &buffer) {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);
  int32_t ret = 0;

  // Return buffer back to recorder service.
  ret = recorder_.ReturnImageCaptureBuffer(params_.camera_id, buffer);
  if (NO_ERROR != ret) {
    ALOGE("%s: ReturnImageCaptureBuffer failed", __func__);
  }

  if (video_time_lapse_mode_ == VideoTimeLapseMode::kModeTwo) {
    snapshot_buffer_returned_promise_.set_value(1);
  }

  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
}

int32_t TimeLapse::SetupAVCodec(const TimeLapseParams &params) {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter ", __func__);
  int32_t ret = 0;

  VideoFormat format = VideoFormat::kAVC;

  switch (params.video_encode_format) {
    case VideoEncodeFormat::kAVC:
      format = VideoFormat::kAVC;
      break;
    case VideoEncodeFormat::kHEVC:
      format = VideoFormat::kHEVC;
      break;
    case VideoEncodeFormat::kMJPEG:
    default:
      ALOGE("%s: Invalid/not supported codec format ", __func__);
      break;
  }

  VideoTrackCreateParam video_track_params{params.camera_id, format,
                                           params.width, params.height,
                                           static_cast<float>(params.fps)};

  CodecParam codec_param;
  codec_param.video_enc_param = video_track_params;

  avcodec_ = IAVCodec::CreateAVCodec();
  if (avcodec_ == nullptr) {
    ALOGE("%s: avcodec creation failed", __func__);
    return NO_MEMORY;
  }

  ret = avcodec_->ConfigureCodec(CodecMimeType::kMimeTypeVideoEncAVC,
                                 codec_param);
  if (ret != OK) {
    ALOGE("%s: Failed to configure Codec", __func__);
    return ret;
  }

  ReturnBufferCB ret_buf_cb = [&](BufferDescriptor &buffer) {
    ReturnYUVSnapshotBuffer(buffer);
  };

  uint32_t count, size;
  ret = avcodec_->GetBufferRequirements(kPortIndexInput, &count, &size);
  if (ret != OK) {
    ALOGE("%s: Failed to get Buffer Requirements on %s", __func__,
          PORT_NAME(kPortIndexInput));
    return ret;
  }

  encoder_source_ = make_shared<EncoderSource>(ret_buf_cb, params_.fps, size);
  if (encoder_source_.get() == nullptr) {
    ALOGE("%s: failed to create input source", __func__);
    return NO_MEMORY;
  }

  ret = avcodec_->AllocateBuffer(kPortIndexInput, 0, 0,
                                 shared_ptr<ICodecSource>(encoder_source_),
                                 input_buffer_list_);
  if (ret != OK) {
    ALOGE("%s: Failed to Call Allocate buffer on PORT_NAME(%d)", __func__,
          kPortIndexInput);
    ReleaseBuffer();
    return ret;
  }

  ret = AllocateBuffer(kPortIndexOutput);
  if (ret != OK) {
    ALOGE("%s: Failed to allocate buffer on PORT_NAME(%d)", __func__,
          kPortIndexOutput);
    ReleaseBuffer();
    return ret;
  }

  ret = avcodec_->RegisterOutputBuffers(output_buffer_list_);
  if (ret != NO_ERROR) {
    ALOGE("%s: output buffers failed to register to AVCodec", __func__);
    ReleaseBuffer();
    return ret;
  }

  std::string type_string;
  switch (params.video_encode_format) {
    case VideoEncodeFormat::kAVC:
      type_string = "h264";
      break;
    case VideoEncodeFormat::kHEVC:
      type_string = "h265";
      break;
    case VideoEncodeFormat::kMJPEG:
      type_string = "mjpeg";
      break;
  }

  auto time_now = std::chrono::system_clock::now();
  auto time_us = std::chrono::duration_cast<std::chrono::microseconds>(
      time_now.time_since_epoch());

  std::ostringstream bitstream_filepath;
  bitstream_filepath << video_time_lapse_file_name_preamble;
  bitstream_filepath << params.width << "x" << params.height << "_";
  bitstream_filepath << time_us.count() << "." << type_string.c_str();

  encoder_sink_ = make_shared<EncoderSink>(bitstream_filepath.str());
  if (encoder_sink_.get() == nullptr) {
    ALOGE("%s: failed to create output source", __func__);
    return NO_MEMORY;
  }

  ret = avcodec_->AllocateBuffer(kPortIndexOutput, 0, 0,
                                 shared_ptr<ICodecSource>(encoder_sink_),
                                 output_buffer_list_);
  if (ret != OK) {
    ALOGE("%s: Failed to Call Allocate buffer on PORT_NAME(%d)", __func__,
          kPortIndexOutput);
    ReleaseBuffer();
    return ret;
  }

  encoder_sink_->AddBufferList(output_buffer_list_);
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
  return ret;
}

int32_t TimeLapse::AllocateBuffer(uint32_t index) {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);
  int32_t ret = 0;

  assert(ion_device_ > 0);
  int32_t ionType = ION_HEAP(ION_IOMMU_HEAP_ID);

  uint32_t count, size;
  ret = avcodec_->GetBufferRequirements(index, &count, &size);
  if (ret != OK) {
    ALOGE("%s: Failed to get Buffer Requirements on %s", __func__,
          PORT_NAME(index));
    return ret;
  }

  struct ion_allocation_data alloc;
  struct ion_fd_data ionFdData;

  void *vaddr = nullptr;

  for (uint32_t i = 0; i < count; i++) {
    BufferDescriptor buffer;
    vaddr = nullptr;

    memset(&buffer, 0x0, sizeof(buffer));
    memset(&alloc, 0x0, sizeof(ion_allocation_data));
    memset(&ionFdData, 0x0, sizeof(ion_fd_data));

    alloc.len = size;
    alloc.len = (alloc.len + 4095) & (~4095);
    alloc.align = 4096;
    alloc.flags = ION_FLAG_CACHED;
    alloc.heap_id_mask = ionType;

    ret = ioctl(ion_device_, ION_IOC_ALLOC, &alloc);
    if (ret < 0) {
      ALOGE("%s: ION allocation failed", __func__);
      goto ION_ALLOC_FAILED;
    }

    ionFdData.handle = alloc.handle;
    ret = ioctl(ion_device_, ION_IOC_SHARE, &ionFdData);
    if (ret < 0) {
      ALOGE("%s: ION map failed %s", __func__, strerror(errno));
      goto ION_MAP_FAILED;
    }

    vaddr = mmap(nullptr, alloc.len, PROT_READ | PROT_WRITE, MAP_SHARED,
                 ionFdData.fd, 0);
    if (vaddr == MAP_FAILED) {
      ALOGE("%s: ION mmap failed: %s (%d)", __func__, strerror(errno), errno);
      goto ION_MAP_FAILED;
    }

    output_ion_handle_data_.push_back(alloc);

    buffer.fd = ionFdData.fd;
    buffer.capacity = alloc.len;
    buffer.size = alloc.len;
    buffer.data = vaddr;

    ALOGD_IF(TIMELAPSE_DEBUG, "%s: buffer.Fd(%d)", __func__, buffer.fd);
    ALOGD_IF(TIMELAPSE_DEBUG, "%s: buffer.capacity(%d)", __func__,
             buffer.capacity);
    ALOGD_IF(TIMELAPSE_DEBUG, "%s: buffer.vaddr(%p)", __func__, buffer.data);
    output_buffer_list_.push_back(buffer);
  }

  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
  return ret;

ION_MAP_FAILED:
  struct ion_handle_data ionHandleData;
  memset(&ionHandleData, 0x0, sizeof(ionHandleData));
  ionHandleData.handle = ionFdData.handle;
  ioctl(ion_device_, ION_IOC_FREE, &ionHandleData);
ION_ALLOC_FAILED:
  close(ion_device_);
  ion_device_ = -1;
  ALOGE("%s: ION Buffer allocation failed!", __func__);
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
  return -1;
}

int32_t TimeLapse::ReleaseBuffer() {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter ", __func__);
  int32_t ret;

  ret = avcodec_->ReleaseBuffer();
  if (NO_ERROR != ret) {
    ALOGE("%s: ReleaseBuffer Failed", __func__);
  }

  int i = 0;
  for (auto &iter : output_buffer_list_) {
    if ((iter).data) {
      munmap((iter).data, (iter).capacity);
      (iter).data = nullptr;
    }
    if ((iter).fd) {
      ioctl(ion_device_, ION_IOC_FREE, &(output_ion_handle_data_[i]));
      close((iter).fd);
      (iter).fd = -1;
    }
    ++i;
  }

  output_buffer_list_.clear();
  output_ion_handle_data_.clear();

  ret = encoder_source_->BufferStatus();
  if (NO_ERROR != ret) {
    ALOGE("%s: EncoderSource BufferStatus Failed", __func__);
  }
  ret = encoder_sink_->BufferStatus();
  if (NO_ERROR != ret) {
    ALOGE("%s: EncoderSink BufferStatus Failed", __func__);
  }

  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: Exit", __func__);
  return ret;
}

EncoderSource::EncoderSource(ReturnBufferCB &return_buffer_cb,
                             const uint32_t fps, uint32_t buffer_size)
    : atomic_eos_(false), time_stamp_(0) {
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: Enter ", __func__);

  buffer_size_ = buffer_size;
  encode_fps_ = fps;
  return_buffer_cb_ = return_buffer_cb;
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: Exit", __func__);
}

EncoderSource::~EncoderSource() {
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: Enter", __func__);
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: Exit", __func__);
}

int32_t EncoderSource::NotifyPortEvent(PortEventType event_type,
                                       void *event_data) {
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: Enter", __func__);
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: Exit", __func__);
  return 0;
}

void EncoderSource::ConsumeBuffer(BufferDescriptor &buffer) {
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: Enter ", __func__);

  std::unique_lock<mutex> lock(wait_for_frame_lock_);
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: codec_buffer.data(0X%p)",
           __func__, buffer.data);
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: codec_buffer.fd(%d)", __func__,
           buffer.fd);
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: buffer.capacity(%d)", __func__,
           buffer.capacity);
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: codec_buffer.size(%d)", __func__,
           buffer.size);

  input_free_buffer_list_.push_back(buffer);
  wait_for_frame_.notify_one();

  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: Exit", __func__);
}

int32_t EncoderSource::GetBuffer(BufferDescriptor &codec_buffer,
                                 void *client_data) {
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: Enter ", __func__);
  int32_t ret = 0;

  std::unique_lock<mutex> lock(wait_for_frame_lock_);
  while (input_free_buffer_list_.size() <= 0) {
    ALOGW("EncoderSource:%s: No sanpshot available wait for snapshot",
          __func__);
    wait_for_frame_.wait(lock);
  }

  BufferDescriptor &buffer = *input_free_buffer_list_.begin();
  lock.unlock();

  int num_fds = 1;
  int num_ints = 3;

  // Allocate buffer for MetaData Handle
  native_handle_t *meta_handle = (native_handle_create(1, 16));
  if (meta_handle == nullptr) {
    ALOGE("EncoderSource:%s: failed to allocated metabuffer handle", __func__);
    return NO_MEMORY;
  }

  meta_handle->version = sizeof(native_handle_t);
  meta_handle->numFds = num_fds;
  meta_handle->numInts = num_ints;
  meta_handle->data[0] = buffer.fd;
  meta_handle->data[1] = 0;  // offset
  meta_handle->data[4] = buffer_size_;

  codec_buffer.data =
      const_cast<void *>(reinterpret_cast<const void *>(meta_handle));
  codec_buffer.fd = buffer.fd;
  codec_buffer.capacity = buffer.capacity;
  codec_buffer.size = buffer.size;

  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: codec_buffer.data(0x%p)",
           __func__, codec_buffer.data);
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: codec_buffer.fd(%d)", __func__,
           codec_buffer.fd);
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: buffer.capacity(%d)", __func__,
           codec_buffer.capacity);
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: codec_buffer.size(%d)", __func__,
           codec_buffer.size);
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: required buffer size(%d)",
           __func__, buffer_size_);

  if (atomic_eos_) {
    codec_buffer.flag = kEOSFlag;
    ret = -1;
  } else {
    codec_buffer.flag = buffer.flag;
  }

  codec_buffer.timestamp = time_stamp_;
  time_stamp_ = time_stamp_ + (uint64_t)(1000000000 / encode_fps_);

  {
    std::unique_lock<mutex> lock(wait_for_frame_lock_);
    input_occupy_buffer_list_.push_back(*input_free_buffer_list_.begin());
    input_free_buffer_list_.erase(input_free_buffer_list_.begin());
  }
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: Exit ", __func__);
  return ret;
}

int32_t EncoderSource::ReturnBuffer(BufferDescriptor &codec_buffer,
                                    void *client_data) {
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: Enter ", __func__);

  std::lock_guard<mutex> lock(wait_for_frame_lock_);
  BufferDescriptor &buffer = *input_occupy_buffer_list_.begin();

  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: buffer.data(0x%p)", __func__,
           buffer.data);
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: buffer.fd(%d)", __func__,
           buffer.fd);
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: codec_buffer.data(0x%p)",
           __func__, codec_buffer.data);

  return_buffer_cb_(buffer);
  input_occupy_buffer_list_.erase(input_occupy_buffer_list_.begin());

  native_handle_t *meta_handle =
      reinterpret_cast<native_handle_t *>(codec_buffer.data);
  native_handle_delete(meta_handle);
  meta_handle = nullptr;

  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: Exit ", __func__);
  return NO_ERROR;
}

int32_t EncoderSource::BufferStatus() {
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: Enter ", __func__);
  int32_t ret = 0;

  ALOGD_IF(TIMELAPSE_DEBUG,
           "EncoderSource:%s: free buffer(%d), occupy buffer(%d)", __func__,
           input_free_buffer_list_.size(), input_occupy_buffer_list_.size());
  assert(input_occupy_buffer_list_.size() == 0);

  ret = (input_occupy_buffer_list_.size() == 0) ? NO_ERROR : BAD_VALUE;
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSource:%s: Exit ", __func__);
  return ret;
}

void EncoderSource::SetEOS() { atomic_eos_ = true; }

EncoderSink::EncoderSink(string file_name) {
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSink:%s: Enter ", __func__);

  output_file_.open(file_name.c_str(),
                    std::ios::out | std::ios::binary | std::ios::trunc);
  if (!output_file_.is_open()) {
    ALOGE("EncoderSink:%s:  error opening file[%s]", __func__,
          file_name.c_str());
  }

  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSink:%s: Exit", __func__);
}

EncoderSink::~EncoderSink() {
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSink:%s: Enter", __func__);
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSink:%s: Exit", __func__);
}

void EncoderSink::AddBufferList(vector<BufferDescriptor> &list) {
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSink:%s: Enter ", __func__);

  output_list_ = list;

  output_free_buffer_list_.clear();

  output_occupy_buffer_list_.clear();

  for (auto &iter : output_list_) {
    output_free_buffer_list_.push_back(iter);
  }

  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSink:%s: Exit", __func__);
}

int32_t EncoderSink::NotifyPortEvent(PortEventType event_type,
                                     void *event_data) {
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSink:%s: Enter", __func__);
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSink:%s: Exit", __func__);
  return 0;
}

int32_t EncoderSink::GetBuffer(BufferDescriptor &codec_buffer,
                               void *client_data) {
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSink:%s: Enter ", __func__);
  int32_t ret = 0;

  std::unique_lock<mutex> lock(wait_for_frame_lock_);
  while (output_free_buffer_list_.size() <= 0) {
    ALOGW("%s: No buffer available to notify. Wait for new buffer", __func__);
    wait_for_frame_.wait(lock);
  }

  BufferDescriptor &buffer = *output_free_buffer_list_.begin();
  codec_buffer.fd = buffer.fd;
  codec_buffer.data = buffer.data;
  output_occupy_buffer_list_.push_back(buffer);
  output_free_buffer_list_.erase(output_free_buffer_list_.begin());

  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSink:%s: Exit ", __func__);
  return ret;
}

int32_t EncoderSink::ReturnBuffer(BufferDescriptor &codec_buffer,
                                  void *client_data) {
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSink:%s: Enter ", __func__);
  int32_t ret = 0;

  assert(codec_buffer.data != nullptr);
  streampos before;
  streampos after;

  if (output_file_.is_open()) {
    before = output_file_.tellp();
    output_file_.write(reinterpret_cast<const char *>(codec_buffer.data),
                       codec_buffer.size);
    after = output_file_.tellp();
    if (after - before != codec_buffer.size) {
      ALOGE("%s: Bad Write error (%d):(%s)\n", __func__, errno,
            strerror(errno));
    }
  } else {
    ALOGE("%s: File is not open to write", __func__);
  }

  if (codec_buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS)) {
    ALOGD_IF(TIMELAPSE_DEBUG, "%s: This is last buffer from encoder.Close file",
             __func__);
    if (output_file_.is_open()) output_file_.close();
  }

  {
    std::unique_lock<mutex> lock(wait_for_frame_lock_);
    std::list<BufferDescriptor>::iterator it =
        output_occupy_buffer_list_.begin();
    bool found = false;
    for (; it != output_occupy_buffer_list_.end(); ++it) {
      if (((*it).data) == (codec_buffer.data)) {
        output_free_buffer_list_.push_back(*it);
        output_occupy_buffer_list_.erase(it);
        wait_for_frame_.notify_one();
        found = true;
        break;
      }
    }

    assert(found == true);
  }

  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSink:%s Exit ", __func__);
  return ret;
}

int32_t EncoderSink::BufferStatus() {
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSink:%s: Enter ", __func__);
  int32_t ret = NO_ERROR;
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Total Buffer(%d), free(%d), occupy(%d)",
           __func__, output_list_.size(), output_free_buffer_list_.size(),
           output_occupy_buffer_list_.size());
  assert(output_occupy_buffer_list_.size() == 0);

  ret = (output_occupy_buffer_list_.size() == 0) ? NO_ERROR : BAD_VALUE;
  ALOGD_IF(TIMELAPSE_DEBUG, "EncoderSink:%s: Exit ", __func__);
  return ret;
}

void TimeLapse::DumpYUVSnapShot(const BufferDescriptor &buffer) {
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Enter", __func__);
  string filename;
  ofstream snapshot_file;
  streampos before;
  streampos after;

  filename.append(yuv_snapshot_file_name_preamble);
  filename.append(to_string(++snapshot_count_));
  filename.append(".yuv");

  snapshot_file.open(filename.c_str(),
                     std::ios::out | std::ios::binary | std::ios::trunc);
  if (!snapshot_file.is_open()) {
    ALOGE("%s: error opening file[%s]", __func__, filename.c_str());
    goto FAIL;
  }

  before = snapshot_file.tellp();
  snapshot_file.write(reinterpret_cast<const char *>(buffer.data), buffer.size);
  after = snapshot_file.tellp();
  if (after - before != buffer.size) {
    ALOGE("%s: Bad Write error (%d):(%s)\n", __func__, errno, strerror(errno));
    goto FAIL;
  }

FAIL:
  if (snapshot_file.is_open()) snapshot_file.close();
  ALOGD_IF(TIMELAPSE_DEBUG, "%s: Exit", __func__);
}
