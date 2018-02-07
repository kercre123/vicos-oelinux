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

#include <inttypes.h>
#include <sys/time.h>
#include <math.h>
#include <log/log.h>
#include <libgralloc/gralloc_priv.h>
#include <QCamera3VendorTags.h>
#include "qmmf_camera3_utils.h"
#include "qmmf_camera_adaptor_gtest.h"

#define PREVIEW_WIDTH 1920
#define PREVIEW_HEIGHT 1080
#define STREAM_BUFFER_COUNT 4
#define HFR_BUFFER_COUNT 48
#define FPS_TIME_INTERVAL 3000000  // Measure avg. FPS once per 3 sec.
#define FPS_ALLOWED_DEV 0.01f  // 1% avg. allowed deviation from FPS
#define ITERATION_COUNT 50
#define ZOOM_STEPS 5
#define EV_STEPS 7

//FIXME: This is temporary change until necessary vendor mode changes are merged
// in HAL3.
#define QCAMERA3_VENDOR_SENSOR_MODE 1

namespace qmmf {

namespace cameraadaptor {

using namespace qcamera;

Camera3Gtest::Camera3Gtest()
    : camera_idx_(0),
      number_of_cameras_(0),
      device_client_(NULL),
      camera_error_(false),
      fps_count_(0),
      dump_yuv_(false),
      yuv_idx_(0),
      raw_idx_(0),
      jpeg_idx_(0),
      prepare_flag_(false),
      input_buffer_flag_(false),
      input_stream_id_(-1),
      input_last_frame_number_(-1),
      cache_last_meta_(false),
      reprocess_flag_(false),
      aec_lock_(false),
      awb_lock_(false) {
  memset(&fps_old_ts_, 0, sizeof(fps_old_ts_));
  memset(&client_cb_, 0, sizeof(client_cb_));
  client_cb_.errorCb = [&](
      CameraErrorCode errorCode,
      const CaptureResultExtras &extras) { ErrorCb(errorCode, extras); };
  client_cb_.idleCb = [&]() { IdleCb(); };
  client_cb_.peparedCb = [&](int id) { PreparedCb(id); };
  client_cb_.shutterCb = [&](const CaptureResultExtras &extras,
                            int64_t ts) { ShutterCb(extras, ts); };
  client_cb_.resultCb = [&](const CaptureResult &result) { ResultCb(result); };
  pthread_mutex_init(&prepare_lock_, NULL);
  pthread_cond_init(&prepare_cond_, NULL);
  pthread_mutex_init(&input_lock_, NULL);
  pthread_cond_init(&input_cond_, NULL);
  pthread_mutex_init(&reprocess_lock_, NULL);
  pthread_cond_init(&reprocess_cond_, NULL);
  pthread_mutex_init(&meta_lock_, NULL);
  pthread_cond_init(&meta_cond_, NULL);
}

Camera3Gtest::~Camera3Gtest() {
  pthread_mutex_destroy(&prepare_lock_);
  pthread_cond_destroy(&prepare_cond_);
  pthread_mutex_destroy(&input_lock_);
  pthread_cond_destroy(&input_cond_);
  pthread_mutex_destroy(&reprocess_lock_);
  pthread_cond_destroy(&reprocess_cond_);
  pthread_mutex_destroy(&meta_lock_);
  pthread_cond_destroy(&meta_cond_);
}

void Camera3Gtest::SetUp() {
  dump_yuv_ = false;
  fps_count_ = yuv_idx_ = raw_idx_ = jpeg_idx_ = 0;
  prepare_flag_ = false;
  input_buffer_flag_ = false;
  input_stream_id_ = -1;
  input_last_frame_number_ = -1;
  cache_last_meta_ = false;
  reprocess_flag_ = false;
  aec_lock_ = false;
  awb_lock_ = false;
  memset(&fps_old_ts_, 0, sizeof(fps_old_ts_));
  avg_fps_ = 0.0f;
  device_client_ = new Camera3DeviceClient(client_cb_);
  ASSERT_TRUE(NULL != device_client_.get());

  auto ret = device_client_->Initialize();
  ASSERT_EQ(0, ret);

  number_of_cameras_ = device_client_->GetNumberOfCameras();
  ASSERT_GT(number_of_cameras_, 0U);

  ret = device_client_->OpenCamera(camera_idx_);
  ASSERT_EQ(0, ret);
}

void Camera3Gtest::TearDown() {
  if (NULL != device_client_.get()) {
    device_client_.clear();
    device_client_ = NULL;
  }
}

void Camera3Gtest::StreamCbAvgFPS(StreamBuffer buffer) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  uint64_t timeDiff =
      (uint64_t)((tv.tv_sec * 1000000 + tv.tv_usec) -
                 (fps_old_ts_.tv_sec * 1000000 + fps_old_ts_.tv_usec));
  fps_count_++;
  if (FPS_TIME_INTERVAL <= timeDiff) {
    avg_fps_ = (fps_count_ * 1000000) / (float)timeDiff;
    printf("%s: Average stream FPS = %5.2f\n", __func__, avg_fps_);
    fps_old_ts_ = tv;
    fps_count_ = 0;
  }

  device_client_->ReturnStreamBuffer(buffer);
}

void Camera3Gtest::ErrorCb(CameraErrorCode errorCode,
                           const CaptureResultExtras &extras) {
  printf("%s: ErrorCode: %d frameNumber %d requestId %d\n", __func__,
         errorCode, extras.frameNumber, extras.requestId);
  if (ERROR_CAMERA_SERVICE >= errorCode) {
    camera_error_ = true;  // Unrecoverable error
  }
}

void Camera3Gtest::IdleCb() {
  printf("%s: Idle state notification\n", __func__);
}

void Camera3Gtest::ShutterCb(const CaptureResultExtras &, int64_t) {}

void Camera3Gtest::PreparedCb(int stream_id) {
  printf("%s: Stream with id: %d prepared\n", __func__, stream_id);
  pthread_mutex_lock(&prepare_lock_);
  prepare_flag_ = true;
  pthread_cond_broadcast(&prepare_cond_);
  pthread_mutex_unlock(&prepare_lock_);
}

void Camera3Gtest::ResultCb(const CaptureResult &result) {
  printf("%s: Result requestId: %d partial count: %d\n", __func__,
         result.resultExtras.requestId, result.resultExtras.partialResultCount);
  if (result.metadata.exists(ANDROID_SENSOR_TIMESTAMP)) {
    int64_t timestamp;
    timestamp = result.metadata.find(ANDROID_SENSOR_TIMESTAMP).data.i64[0];
    printf("%s: Result ts: %" PRId64 "\n", __func__, timestamp);
  }

  if (result.metadata.exists(ANDROID_SENSOR_EXPOSURE_TIME)) {
    int64_t sensorExpTime =
        result.metadata.find(ANDROID_SENSOR_EXPOSURE_TIME).data.i64[0];
    printf("%s: Result sensorExpTime %" PRId64 "\n", __func__, sensorExpTime);
  }

  if (result.metadata.exists(ANDROID_CONTROL_AF_MODE)) {
    uint8_t focus_mode =
        result.metadata.find(ANDROID_CONTROL_AF_MODE).data.u8[0];
    printf("%s: Focus mode active: %u\n", __func__, focus_mode);
  }

  if (result.metadata.exists(ANDROID_CONTROL_AF_STATE)) {
    uint8_t focus_state =
        result.metadata.find(ANDROID_CONTROL_AF_STATE).data.u8[0];
    printf("%s: Focus mode state: %u\n", __func__, focus_state);
  }

  pthread_mutex_lock(&meta_lock_);
  if (cache_last_meta_) {
    last_meta_ = result.metadata;
    cache_last_meta_ = false;
    pthread_cond_broadcast(&meta_cond_);
  }
  pthread_mutex_unlock(&meta_lock_);
}

void Camera3Gtest::SnapshotCb(StreamBuffer buffer) {
  printf("%s: E streamId: %d buffer: %p ts: %" PRId64 "\n", __func__,
         buffer.stream_id, buffer.handle, buffer.timestamp);

  CalcSize sizeFunc = [&](
      uint8_t *mappedBuffer, uint32_t width, uint32_t height,
      uint32_t stride) { return GetJpegSize(mappedBuffer, width); };

  String8 path;
  path.appendFormat("/usr/stream_%d_%" PRIo64, buffer.stream_id, jpeg_idx_);
  path.appendFormat(".jpg");
  StoreBuffer(path, jpeg_idx_, buffer, sizeFunc);

  device_client_->ReturnStreamBuffer(buffer);
}

bool Camera3Gtest::IsInputSupported() {
  CameraMetadata staticInfo;
  auto res = device_client_->GetCameraInfo(camera_idx_, &staticInfo);
  if (0 != res) {
    return false;
  }

  if (staticInfo.exists(ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS)) {
    camera_metadata_entry_t entry = staticInfo.find(
        ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS);
    if (0 < entry.data.i32[0]) {
      return true;
    }
  }

  return false;
}

int32_t Camera3Gtest::GetMaxYUVSize(int32_t &width, int32_t &height) {
  CameraMetadata static_info;
  camera_metadata_entry_t entry;
  auto res = device_client_->GetCameraInfo(camera_idx_, &static_info);
  if (0 != res) {
    return res;
  }

  if (static_info.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
    entry = static_info.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
    int32_t w, h;
    for (uint32_t i = 0 ; i < entry.count; i += 4) {
      if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == entry.data.i32[i]) {
        if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
            entry.data.i32[i+3]) {
          w = entry.data.i32[i+1];
          h = entry.data.i32[i+2];
          if ((width * height) < (w * h)) {
            width = w;
            height = h;
          }
        }
      }
    }
  }

  if (0 >= (width * height)) {
    res = -ENOENT;
  }

  return res;
}

int32_t Camera3Gtest::GetMaxRAWSize(int32_t &width, int32_t &height) {
  CameraMetadata static_info;
  auto res = device_client_->GetCameraInfo(camera_idx_, &static_info);
  if (0 != res) {
    return res;
  }

  if (static_info.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
    width = static_info.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES).data.i32[0];
    height = static_info.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES).data.i32[1];
  }

  if (0 >= (width * height)) {
    res = -ENOENT;
  }

  return res;
}

uint64_t Camera3Gtest::GetJpegSize(uint8_t *blobBuffer, uint32_t width) {
  uint32_t ret = width;
  uint32_t blobSize = sizeof(struct camera3_jpeg_blob);

  if (width > blobSize) {
    size_t offset = width - blobSize - 1;
    uint8_t *footer = blobBuffer + offset;
    struct camera3_jpeg_blob *jpegBlob = (struct camera3_jpeg_blob *)footer;

    if (CAMERA3_JPEG_BLOB_ID == jpegBlob->jpeg_blob_id) {
      ret = jpegBlob->jpeg_size;
    } else {
      printf("%s: Jpeg Blob structure missing!\n", __func__);
    }
  } else {
    printf("%s: Buffer width: %u equal or smaller than Blob size: %u\n",
           __func__, width, blobSize);
  }

  return ret;
}

int32_t Camera3Gtest::StartSreaming(int32_t usage, uint32_t width,
                                    uint32_t height, int templateId,
                                    int32_t &streamId, int32_t &requestId) {
  CameraStreamParameters streamParams;
  Camera3Request streamingRequest;
  int64_t lastFrameNumber;

  auto ret = device_client_->BeginConfigure();
  if (0 != ret) {
    return ret;
  }

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = width;
  streamParams.height = height;
  streamParams.grallocFlags = usage;;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  ret = device_client_->CreateStream(streamParams);
  if (0 > ret) {
    return ret;
  }
  streamId = ret;
  streamingRequest.streamIds.add(streamId);

  ret = device_client_->EndConfigure();
  if (0 != ret) {
    return ret;
  }

  ret = device_client_->CreateDefaultRequest(templateId,
                                             &streamingRequest.metadata);
  if (0 != ret) {
    return ret;
  }

  ret = device_client_->SubmitRequest(streamingRequest, true, &lastFrameNumber);
  if (0 > ret) {
    return ret;
  }
  requestId = ret;

  return 0;
}

int32_t Camera3Gtest::StopDeleteStream(int32_t streamId, int32_t requestId) {
  int64_t lastFrameNumber;

  auto ret = device_client_->CancelRequest(requestId, &lastFrameNumber);
  if (0 != ret) {
    return ret;
  }

  printf("%s: Streaming request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  if (0 != ret) {
    return ret;
  }

  ret = device_client_->DeleteStream(streamId, true);
  if (0 != ret) {
    return ret;
  }

  return ret;
}

void Camera3Gtest::StreamCb(StreamBuffer buffer) {
  printf("%s: streamId: %d buffer: %p ts: %" PRId64 "\n", __func__,
         buffer.stream_id, buffer.handle, buffer.timestamp);
  device_client_->ReturnStreamBuffer(buffer);
}

void Camera3Gtest::StreamCbSignalOnFrame(StreamBuffer buffer) {
  printf("%s: streamId: %d buffer with frame number: %d arrived\n",
         __func__, buffer.stream_id, buffer.frame_number);
  device_client_->ReturnStreamBuffer(buffer);
  pthread_mutex_lock(&input_lock_);
  if (input_buffer_flag_ &&
      (buffer.frame_number == input_last_frame_number_)) {
    input_buffer_flag_ = false;
    pthread_cond_broadcast(&input_cond_);
  }
  pthread_mutex_unlock(&input_lock_);
}

void Camera3Gtest::StreamCbDumpNVXX(StreamBuffer buffer) {
  printf("%s: streamId: %d buffer: %p ts: %" PRId64 "\n", __func__,
         buffer.stream_id, buffer.handle, buffer.timestamp);

  if (dump_yuv_) {
    String8 path;
    dump_yuv_ = false;
    CalcSize sizeFunc = [&](uint8_t *mappedBuffer, uint32_t width,
        uint32_t height, uint32_t stride) { return 0; };

    path.appendFormat("/usr/stream_%d_%" PRIo64, buffer.stream_id, yuv_idx_);
    path.appendFormat(".yuv");
    StoreBuffer(path, yuv_idx_, buffer, sizeFunc);
  }

  device_client_->ReturnStreamBuffer(buffer);

  pthread_mutex_lock(&reprocess_lock_);
  if (reprocess_flag_) {
    reprocess_flag_ = false;
    pthread_cond_broadcast(&reprocess_cond_);
  }
  pthread_mutex_unlock(&reprocess_lock_);
}

void Camera3Gtest::StreamCbAecLock(StreamBuffer buffer) {
  printf("%s: streamId: %d buffer: %p ts: %" PRId64 "\n", __func__,
         buffer.stream_id, buffer.handle, buffer.timestamp);

  CalcSize sizeFunc = [&](uint8_t *mappedBuffer, uint32_t width,
        uint32_t height, uint32_t stride) { return 0; };

  if(buffer.frame_number % 5 == 0) {
    String8 path;
    path.appendFormat("/data/misc/qmmf/aec_lock/stream_%d_%d_%d.yuv",
                      buffer.stream_id, buffer.frame_number, aec_lock_);
    mkdir("/data/misc/qmmf/aec_lock", S_IRWXU);
    StoreBuffer(path, yuv_idx_, buffer, sizeFunc);
  }
  device_client_->ReturnStreamBuffer(buffer);
}

void Camera3Gtest::StreamCbAwbLock(StreamBuffer buffer) {
  printf("%s: streamId: %d buffer: %p ts: %" PRId64 "\n", __func__,
         buffer.stream_id, buffer.handle, buffer.timestamp);

  CalcSize sizeFunc = [&](uint8_t *mappedBuffer, uint32_t width,
        uint32_t height, uint32_t stride) { return 0; };

  if(buffer.frame_number % 5 == 0) {
    String8 path;
    path.appendFormat("/data/misc/qmmf/awb_lock/stream_%d_%d_%d.yuv",
                      buffer.stream_id, buffer.frame_number, awb_lock_);
    mkdir("/data/misc/qmmf/awb_lock", S_IRWXU);
    StoreBuffer(path, yuv_idx_, buffer, sizeFunc);
  }
  device_client_->ReturnStreamBuffer(buffer);
}

void Camera3Gtest::Raw16Cb(StreamBuffer buffer) {
  printf("%s: E streamId: %d buffer: %p ts: %" PRId64 "\n", __func__,
         buffer.stream_id, buffer.handle, buffer.timestamp);

  CalcSize sizeFunc = [&](
      uint8_t *mappedBuffer, uint32_t width, uint32_t height,
      uint32_t stride) { return stride*height*2; };

  String8 path;
  path.appendFormat("/usr/stream_%d_%" PRIo64, buffer.stream_id, raw_idx_);
  path.appendFormat(".raw");
  StoreBuffer(path, raw_idx_, buffer, sizeFunc);

  device_client_->ReturnStreamBuffer(buffer);
}

void Camera3Gtest::InputCb(StreamBuffer buffer) {
  bool return_buffer = true;
  pthread_mutex_lock(&input_lock_);
  if (input_buffer_flag_) {
    input_buffer_flag_ = false;
    input_buffer_ = buffer;
    return_buffer = false;
    pthread_cond_broadcast(&input_cond_);
  }
  pthread_mutex_unlock(&input_lock_);

  if (return_buffer) {
    device_client_->ReturnStreamBuffer(buffer);
  }
}

void Camera3Gtest::GetInputBuffer(StreamBuffer &buffer) {
  buffer = input_buffer_;
}

void Camera3Gtest::ReturnInputBuffer(StreamBuffer &buffer) {
  if (buffer.handle == input_buffer_.handle) {
    auto ret = device_client_->ReturnStreamBuffer(input_buffer_);
    if (0 != ret) {
      printf("%s: Failed to return input buffer: %d\n", __func__, ret);
      camera_error_ = true;
    }
  } else {
    printf("%s: Buffer handle of returned buffer: %p doesn't match with"
        "expected handle: %p\n", __func__, buffer.handle,
        input_buffer_.handle);
    camera_error_ = true;
  }
}

int32_t Camera3Gtest::StoreBuffer(String8 path, uint64_t &idx,
                                  StreamBuffer &buffer, CalcSize &calcSize) {
  int32_t ret = 0;

  alloc_device_t *grallocDevice = device_client_->GetGrallocDevice();

  if (NULL != grallocDevice) {
    FILE *f = fopen(path.string(), "w+");
    if (NULL == f) {
      printf("%s:Unable to open file(%s) \n", __func__, strerror(errno));
      return -errno;
    }

    gralloc_module_t const *mapper = reinterpret_cast<gralloc_module_t const *>(
        grallocDevice->common.module);
    struct android_ycbcr grallocBuffer;
    if ((BufferFormat::kNV12 == buffer.info.format) ||
        (BufferFormat::kNV21 == buffer.info.format)) {
      if (2 != buffer.info.num_planes) {
        printf("%s: Unexpected number of planes: %d for NVXX format!\n",
               __func__, buffer.info.num_planes);
        return -EINVAL;
      }
      ret = mapper->lock_ycbcr(mapper, buffer.handle,
                               GRALLOC_USAGE_SW_READ_OFTEN, 0, 0,
                               buffer.info.plane_info[0].width,
                               buffer.info.plane_info[0].height,
                               &grallocBuffer);
      //Chroma step should be 2 for interleaved data
      if ((0 != ret) || (2 != grallocBuffer.chroma_step)) {
        printf("%s: Unable to map gralloc buffer res: %d \n", __func__, ret);
        return ret;
      }
      void *mappedYBuffer = grallocBuffer.y;
      void *mappedCbCrBuffer = (grallocBuffer.cb < grallocBuffer.cr) ?
          grallocBuffer.cb : grallocBuffer.cr;

      uint64_t sizeY = buffer.info.plane_info[0].stride *
          buffer.info.plane_info[0].scanline;
      uint64_t sizeCbCr = buffer.info.plane_info[1].stride *
          buffer.info.plane_info[1].scanline;

      if (sizeY != fwrite(mappedYBuffer, 1, sizeY, f)) {
        ret = ferror(f);
        printf("%s: Bad Write error (%d) %s\n", __func__, -ret, strerror(ret));
        goto exit;
      }
      if (sizeCbCr != fwrite(mappedCbCrBuffer, 1, sizeCbCr, f)) {
        ret = ferror(f);
        printf("%s: Bad Write error (%d) %s\n", __func__, -ret, strerror(ret));
        goto exit;
      }
      idx++;

      printf("%s: %s Size=%" PRIo64 " Stored\n", __func__, path.string(),
             sizeY + sizeCbCr);
    } else {
      if (0 == buffer.info.num_planes) {
        printf("%s: Unexpected number of planes: %d!\n",
               __func__, buffer.info.num_planes);
        return -EINVAL;
      }
      uint8_t *mappedBuffer = NULL;
      ret = mapper->lock(mapper, buffer.handle, GRALLOC_USAGE_SW_READ_OFTEN, 0,
                         0, buffer.info.plane_info[0].width,
                         buffer.info.plane_info[0].height,
                         (void **)&mappedBuffer);
      if ((0 != ret) || (NULL == mappedBuffer)) {
        printf("%s: Unable to map gralloc buffer: %p res: %d\n", __func__,
               mappedBuffer, ret);
        return ret;
      }

      uint64_t size;
      if (BufferFormat::kNV12UBWC == buffer.info.format) {
        size = buffer.size;
      } else {
        size = calcSize(mappedBuffer, buffer.info.plane_info[0].width,
                   buffer.info.plane_info[0].height,
                   buffer.info.plane_info[0].stride);
      }

      if (size != fwrite(mappedBuffer, sizeof(uint8_t), size, f)) {
        ret = ferror(f);
        printf("%s: Bad Write error (%d) %s\n", __func__, -ret, strerror(ret));
        goto exit;
      }
      idx++;

      printf("%s: %s Buffer=%p, Size=%" PRIo64 " Stored\n", __func__,
             path.string(), mappedBuffer, size);
    }

  exit:
    mapper->unlock(mapper, buffer.handle);

    if (NULL != f) {
      fclose(f);
    }
  }

  return ret;
}

TEST_F(Camera3Gtest, Video1080pManualExposure) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest;
  CameraMetadata staticInfo;
  int64_t lastFrameNumber;
  int32_t repeatingStreamId, videoRequestId;
  int64_t exposureTime = 10000000;
  int32_t sensitivity = 100;

  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 1920;
  streamParams.height = 1080;
  streamParams.grallocFlags =
    GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCbDumpNVXX(buffer); };

  // 1080p Stream1
  repeatingStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(repeatingStreamId, 0);
  videoRequest.streamIds.add(repeatingStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                      &videoRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                      &videoRequest.metadata);
  ASSERT_EQ(0, ret);

  uint8_t aeMode =  ANDROID_CONTROL_AE_MODE_OFF;
  videoRequest.metadata.update(ANDROID_CONTROL_AE_MODE, &aeMode, 1);

  printf("Manual Exposure Time: %lld\n", exposureTime);
  videoRequest.metadata.update(ANDROID_SENSOR_EXPOSURE_TIME, &exposureTime, 1);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  dump_yuv_ = true;

  exposureTime = 30000000;
  printf("Manual Exposure Time: %lld\n", exposureTime);
  videoRequest.metadata.update(ANDROID_SENSOR_EXPOSURE_TIME, &exposureTime, 1);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  dump_yuv_ = true;

  ret = device_client_->GetCameraInfo(camera_idx_, &staticInfo);
  ASSERT_EQ(0, ret);

  ASSERT_TRUE(staticInfo.exists(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE));

  camera_metadata_entry metaEntry =
      staticInfo.find(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE);
  int32_t minSens = metaEntry.data.i32[0];
  int32_t maxSens = metaEntry.data.i32[1];

//  ANDROID_SENSOR_SENSITIVITY
  printf("%s: sensor sensitivity range: min[%" PRId32 "], max[%" PRId32 "]\n",
     __func__, minSens, maxSens);

  sensitivity = minSens;

  videoRequest.metadata.update(ANDROID_SENSOR_SENSITIVITY, &sensitivity, 1);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  dump_yuv_ = true;

  sensitivity = maxSens;

  videoRequest.metadata.update(ANDROID_SENSOR_SENSITIVITY, &sensitivity, 1);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  dump_yuv_ = true;

  ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
     __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);

  ret = device_client_->DeleteStream(repeatingStreamId, true);
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, Video1080pSceneControl) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest;
  int64_t lastFrameNumber;
  int32_t repeatingStreamId, videoRequestId;
  CameraMetadata staticInfo;

  auto ret = device_client_->GetCameraInfo(camera_idx_, &staticInfo);
  ASSERT_EQ(0, ret);

  if (staticInfo.exists(ANDROID_CONTROL_AVAILABLE_SCENE_MODES)) {
    camera_metadata_entry metaEntry =
        staticInfo.find(ANDROID_CONTROL_AVAILABLE_SCENE_MODES);

    ret = device_client_->BeginConfigure();
    ASSERT_EQ(0, ret);

    memset(&streamParams, 0, sizeof(streamParams));
    streamParams.bufferCount = STREAM_BUFFER_COUNT;
    streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
    streamParams.width = 1920;
    streamParams.height = 1080;
    streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
    streamParams.cb = [&](StreamBuffer buffer) { StreamCbDumpNVXX(buffer); };

    // 1080p Stream1
    repeatingStreamId = device_client_->CreateStream(streamParams);
    ASSERT_GE(repeatingStreamId, 0);
    videoRequest.streamIds.add(repeatingStreamId);

    ret = device_client_->EndConfigure();
    ASSERT_EQ(0, ret);

    ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                        &videoRequest.metadata);
    ASSERT_EQ(0, ret);

    ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
    ASSERT_GE(ret, 0);
    videoRequestId = ret;

    while (metaEntry.count) {
      --metaEntry.count;

      printf("Scene Mode: %d\n", metaEntry.data.u8[metaEntry.count]);

      ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                          &videoRequest.metadata);
      ASSERT_EQ(0, ret);

      uint8_t sceneMode = ANDROID_CONTROL_MODE_USE_SCENE_MODE;
      videoRequest.metadata.update(ANDROID_CONTROL_MODE, &sceneMode, 1);
      videoRequest.metadata.update(ANDROID_CONTROL_SCENE_MODE,
                                    &metaEntry.data.u8[metaEntry.count],
                                    1);

      ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
      ASSERT_GE(ret, 0);
      videoRequestId = ret;

      // Run video for some time
      sleep(5);

      dump_yuv_ = true;
    }

    ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
    ASSERT_EQ(0, ret);

    printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
       __func__, lastFrameNumber);

    ret = device_client_->WaitUntilIdle();
    ASSERT_EQ(0, ret);

    ret = device_client_->DeleteStream(repeatingStreamId, true);
    ASSERT_EQ(0, ret);
    ASSERT_FALSE(camera_error_);
  }
}

TEST_F(Camera3Gtest, Video1080pEVcontrol) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest;
  int64_t lastFrameNumber;
  int32_t repeatingStreamId, videoRequestId;
  CameraMetadata staticInfo;

  auto ret = device_client_->GetCameraInfo(camera_idx_, &staticInfo);
  ASSERT_EQ(0, ret);

  if (staticInfo.exists(ANDROID_CONTROL_AE_COMPENSATION_RANGE)) {
    camera_metadata_entry metaEntry =
        staticInfo.find(ANDROID_CONTROL_AE_COMPENSATION_RANGE);

    printf("EV Compensation 0\n");
    ret = device_client_->BeginConfigure();
    ASSERT_EQ(0, ret);

    memset(&streamParams, 0, sizeof(streamParams));
    streamParams.bufferCount = STREAM_BUFFER_COUNT;
    streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
    streamParams.width = 1920;
    streamParams.height = 1080;
    streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
    streamParams.cb = [&](StreamBuffer buffer) { StreamCbDumpNVXX(buffer); };

    // 1080p Stream1
    repeatingStreamId = device_client_->CreateStream(streamParams);
    ASSERT_GE(repeatingStreamId, 0);
    videoRequest.streamIds.add(repeatingStreamId);

    ret = device_client_->EndConfigure();
    ASSERT_EQ(0, ret);

    ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                        &videoRequest.metadata);
    ASSERT_EQ(0, ret);

    int32_t evCompensation = 0;
    videoRequest.metadata.update(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION,
                                  &evCompensation,
                                  1);

    ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
    ASSERT_GE(ret, 0);
    videoRequestId = ret;

    // Run video for some time
    sleep(5);

    dump_yuv_ = true;

    while (metaEntry.count) {
      --metaEntry.count;
      printf("EV Compensation: %d\n", metaEntry.data.i32[metaEntry.count]);

      ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                          &videoRequest.metadata);
      ASSERT_EQ(0, ret);

      videoRequest.metadata.update(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION,
                                    &metaEntry.data.i32[metaEntry.count],
                                    1);

      ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
      ASSERT_GE(ret, 0);
      videoRequestId = ret;

      // Run video for some time
      sleep(5);

      dump_yuv_ = true;
    }

    ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
    ASSERT_EQ(0, ret);

    printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
       __func__, lastFrameNumber);

    ret = device_client_->WaitUntilIdle();
    ASSERT_EQ(0, ret);

    ret = device_client_->DeleteStream(repeatingStreamId, true);
    ASSERT_EQ(0, ret);
    ASSERT_FALSE(camera_error_);
  }
}

TEST_F(Camera3Gtest, Video1080pExposureModes) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest;
  int64_t lastFrameNumber;
  int32_t repeatingStreamId, videoRequestId;
  CameraMetadata staticInfo;

  auto ret = device_client_->GetCameraInfo(camera_idx_, &staticInfo);
  ASSERT_EQ(0, ret);

  if (staticInfo.exists(ANDROID_CONTROL_AE_AVAILABLE_MODES)) {
    camera_metadata_entry metaEntry =
        staticInfo.find(ANDROID_CONTROL_AE_AVAILABLE_MODES);

    ret = device_client_->BeginConfigure();
    ASSERT_EQ(0, ret);

    memset(&streamParams, 0, sizeof(streamParams));
    streamParams.bufferCount = STREAM_BUFFER_COUNT;
    streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
    streamParams.width = 1920;
    streamParams.height = 1080;
    streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
    streamParams.cb = [&](StreamBuffer buffer) { StreamCbDumpNVXX(buffer); };

    // 1080p Stream1
    repeatingStreamId = device_client_->CreateStream(streamParams);
    ASSERT_GE(repeatingStreamId, 0);
    videoRequest.streamIds.add(repeatingStreamId);

    ret = device_client_->EndConfigure();
    ASSERT_EQ(0, ret);

    ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                        &videoRequest.metadata);
    ASSERT_EQ(0, ret);

    ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
    ASSERT_GE(ret, 0);
    videoRequestId = ret;

    while (metaEntry.count) {
      --metaEntry.count;
      printf("Exposure mode: %d\n", metaEntry.data.u8[metaEntry.count]);

      ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                          &videoRequest.metadata);
      ASSERT_EQ(0, ret);

      videoRequest.metadata.update(ANDROID_CONTROL_AE_MODE,
                                    &metaEntry.data.u8[metaEntry.count],
                                    1);

      ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
      ASSERT_GE(ret, 0);
      videoRequestId = ret;

      // Run video for some time
      sleep(5);

      dump_yuv_ = true;
    }

    ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
    ASSERT_EQ(0, ret);

    printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
       __func__, lastFrameNumber);

    ret = device_client_->WaitUntilIdle();
    ASSERT_EQ(0, ret);

    ret = device_client_->DeleteStream(repeatingStreamId, true);
    ASSERT_EQ(0, ret);
    ASSERT_FALSE(camera_error_);
  }
}

TEST_F(Camera3Gtest, Video1080pExposureMeteringModes) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest;
  int64_t lastFrameNumber;
  int32_t repeatingStreamId, videoRequestId;
  CameraMetadata staticInfo;

  auto ret = device_client_->GetCameraInfo(camera_idx_, &staticInfo);
  ASSERT_EQ(0, ret);

  if (staticInfo.exists(QCAMERA3_EXPOSURE_METER_AVAILABLE_MODES)) {
    camera_metadata_entry metaEntry =
        staticInfo.find(QCAMERA3_EXPOSURE_METER_AVAILABLE_MODES);

    ret = device_client_->BeginConfigure();
    ASSERT_EQ(0, ret);

    memset(&streamParams, 0, sizeof(streamParams));
    streamParams.bufferCount = STREAM_BUFFER_COUNT;
    streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
    streamParams.width = 1920;
    streamParams.height = 1080;
    streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
    streamParams.cb = [&](StreamBuffer buffer) { StreamCbDumpNVXX(buffer); };

    // 1080p Stream1
    repeatingStreamId = device_client_->CreateStream(streamParams);
    ASSERT_GE(repeatingStreamId, 0);
    videoRequest.streamIds.add(repeatingStreamId);

    ret = device_client_->EndConfigure();
    ASSERT_EQ(0, ret);

    ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                        &videoRequest.metadata);
    ASSERT_EQ(0, ret);

    ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
    ASSERT_GE(ret, 0);
    videoRequestId = ret;

    while (metaEntry.count) {
      --metaEntry.count;
      printf("Exposure metering mode: %d\n", metaEntry.data.i32[metaEntry.count]);

      ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                          &videoRequest.metadata);
      ASSERT_EQ(0, ret);

      videoRequest.metadata.update(QCAMERA3_EXPOSURE_METER,
                                    &metaEntry.data.i32[metaEntry.count],
                                    1);

      ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
      ASSERT_GE(ret, 0);
      videoRequestId = ret;

      // Run video for some time
      sleep(5);

      dump_yuv_ = true;
    }

    ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
    ASSERT_EQ(0, ret);

    printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
       __func__, lastFrameNumber);

    ret = device_client_->WaitUntilIdle();
    ASSERT_EQ(0, ret);

    ret = device_client_->DeleteStream(repeatingStreamId, true);
    ASSERT_EQ(0, ret);
    ASSERT_FALSE(camera_error_);
  }
}

TEST_F(Camera3Gtest, Video1080pSnapshotHDR) {
  CameraStreamParameters streamParams;
  Camera3Request previewRequest, snapshotRequest;
  int64_t lastFrameNumber;
  int32_t previewStreamId, snapshotStreamId;
  int32_t previewRequestId;
  CameraMetadata staticInfo;
  uint8_t sceneMode = ANDROID_CONTROL_MODE_USE_SCENE_MODE;

  auto ret = device_client_->GetCameraInfo(camera_idx_, &staticInfo);
  ASSERT_EQ(0, ret);

  if (staticInfo.exists(ANDROID_CONTROL_AVAILABLE_SCENE_MODES)) {
    camera_metadata_entry metaEntry =
        staticInfo.find(ANDROID_CONTROL_AVAILABLE_SCENE_MODES);

    ret = device_client_->BeginConfigure();
    ASSERT_EQ(0, ret);

    memset(&streamParams, 0, sizeof(streamParams));
    streamParams.bufferCount = STREAM_BUFFER_COUNT;
    streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
    streamParams.width = 1920;
    streamParams.height = 1080;
    streamParams.grallocFlags = GRALLOC_USAGE_HW_FB;
    streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

    // 1080p Stream1
    previewStreamId = device_client_->CreateStream(streamParams);
    ASSERT_GE(previewStreamId, 0);
    previewRequest.streamIds.add(previewStreamId);

    memset(&streamParams, 0, sizeof(streamParams));
    streamParams.bufferCount = 3;
    streamParams.format = HAL_PIXEL_FORMAT_BLOB;
    streamParams.width = 4000;
    streamParams.height = 3000;
    streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
    streamParams.cb = [&](StreamBuffer buffer) { SnapshotCb(buffer); };

    snapshotStreamId = device_client_->CreateStream(streamParams);
    ASSERT_GE(snapshotStreamId, 0);
    snapshotRequest.streamIds.add(snapshotStreamId);

    ret = device_client_->EndConfigure();
    ASSERT_EQ(0, ret);

    ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_PREVIEW,
                        &previewRequest.metadata);
    ASSERT_EQ(0, ret);

    ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_STILL_CAPTURE,
                        &snapshotRequest.metadata);
    ASSERT_EQ(0, ret);

    ret = device_client_->SubmitRequest(previewRequest, true, &lastFrameNumber);
    ASSERT_GE(ret, 0);
    previewRequestId = ret;

    sleep(2);

    for ( uint8_t i = 0; i < metaEntry.count; i++) {
        if (metaEntry.data.u8[i] == ANDROID_CONTROL_SCENE_MODE_HDR) {
            snapshotRequest.metadata.update(ANDROID_CONTROL_MODE, &sceneMode, 1);
            snapshotRequest.metadata.update(ANDROID_CONTROL_SCENE_MODE,
                                    &metaEntry.data.u8[i],
                                    1);
            ret = device_client_->SubmitRequest(snapshotRequest, false, &lastFrameNumber);
            ASSERT_GE(ret, 0);
            break;
        }
    }

    // Run preview for some time
    sleep(2);

    ret = device_client_->CancelRequest(previewRequestId, &lastFrameNumber);
    ASSERT_EQ(0, ret);

    printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
       __func__, lastFrameNumber);

    ret = device_client_->WaitUntilIdle();
    ASSERT_EQ(0, ret);

    ret = device_client_->DeleteStream(snapshotStreamId, true);
    ASSERT_EQ(0, ret);

    ret = device_client_->DeleteStream(previewRequestId, true);
    ASSERT_EQ(0, ret);
    ASSERT_FALSE(camera_error_);
  } else {
    printf("%s: Snapshot HDR is not supported by the Camera.\n",
       __func__);
   ret = 0;
   ASSERT_EQ(0, ret);
  }
}

TEST_F(Camera3Gtest, ZSLStream12Mp) {
  CameraStreamParameters streamParams;
  Camera3Request zslRequest;
  int64_t lastFrameNumber;
  int32_t zslStreamId, zslRequestId;
  int32_t streamFPS = 30;
  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 4;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 4000;
  streamParams.height = 3000;
  streamParams.grallocFlags = GRALLOC_USAGE_HW_FB|GRALLOC_USAGE_HW_CAMERA_ZSL;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCbAvgFPS(buffer); };

  zslStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(zslStreamId, 0);
  zslRequest.streamIds.add(zslStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_ZERO_SHUTTER_LAG,
                                            &zslRequest.metadata);
  ASSERT_EQ(0, ret);

  int32_t fpsRange[2];
  fpsRange[0] = streamFPS;
  fpsRange[1] = streamFPS;

  zslRequest.metadata.update(ANDROID_CONTROL_AE_TARGET_FPS_RANGE, fpsRange, 2);

  ret = device_client_->SubmitRequest(zslRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  zslRequestId = ret;

  // Run ZSL for some time
  sleep(5);

  ret = device_client_->CancelRequest(zslRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: ZSL request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
  float allowedDeviation = streamFPS * FPS_ALLOWED_DEV;
  float measuredDeviation = fabs(streamFPS - avg_fps_);
  ASSERT_GE(allowedDeviation, measuredDeviation);
  printf("%s: Measured deviation: %5.2f, allowed deviation: %5.2f\n", __func__,
         measuredDeviation, allowedDeviation);
}

TEST_F(Camera3Gtest, FlushZSL) {
  CameraStreamParameters streamParams;
  Camera3Request zslRequest;
  int64_t lastFrameNumber;
  int32_t zslStreamId;
  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 4;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 4000;
  streamParams.height = 3000;
  streamParams.grallocFlags = GRALLOC_USAGE_HW_FB;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  zslStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(zslStreamId, 0);
  zslRequest.streamIds.add(zslStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_ZERO_SHUTTER_LAG,
                                            &zslRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(zslRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);

  // Run ZSL for some time
  sleep(5);

  ret = device_client_->Flush(&lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: ZSL request flushed last frame number: %" PRId64 "\n", __func__,
         lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, Preview1080pSnapshot12Mp) {
  CameraStreamParameters streamParams;
  Camera3Request previewRequest, snapshotRequest;
  int64_t lastFrameNumber;
  int32_t previewStreamId, previewRequestId;
  int32_t snapshotStreamId;
  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = PREVIEW_WIDTH;
  streamParams.height = PREVIEW_HEIGHT;
  streamParams.grallocFlags = GRALLOC_USAGE_HW_FB;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  previewStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(previewStreamId, 0);
  previewRequest.streamIds.add(previewStreamId);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 1;
  streamParams.format = HAL_PIXEL_FORMAT_BLOB;
  streamParams.width = 4000;
  streamParams.height = 3000;
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](StreamBuffer buffer) { SnapshotCb(buffer); };

  snapshotStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(snapshotStreamId, 0);
  snapshotRequest.streamIds.add(snapshotStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_PREVIEW,
                                            &previewRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_STILL_CAPTURE,
                                            &snapshotRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(previewRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  previewRequestId = ret;

  // Run Preview for some time
  sleep(5);

  // Take a snapshot
  ret = device_client_->SubmitRequest(snapshotRequest, false, &lastFrameNumber);
  ASSERT_GE(ret, 0);

  // Run Preview for some time
  sleep(5);

  ret = device_client_->CancelRequest(previewRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Preview request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, UpdateExposureDuringPreviewVGA) {
  CameraStreamParameters streamParams;
  Camera3Request previewRequest;
  CameraMetadata staticInfo;
  int64_t lastFrameNumber;
  int32_t previewStreamId, previewRequestId;

  auto ret = device_client_->GetCameraInfo(camera_idx_, &staticInfo);
  ASSERT_EQ(0, ret);

  ASSERT_TRUE(staticInfo.exists(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE));

  camera_metadata_entry metaEntry =
      staticInfo.find(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE);
  int64_t maxExposureTime = metaEntry.data.i64[1];
  ASSERT_GT(maxExposureTime, 0);

  ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 640;
  streamParams.height = 480;
  streamParams.grallocFlags = GRALLOC_USAGE_HW_FB;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCbDumpNVXX(buffer); };

  previewStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(previewStreamId, 0);
  previewRequest.streamIds.add(previewStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_PREVIEW,
                                            &previewRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(previewRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  previewRequestId = ret;

  // Run preview for some time
  sleep(5);

  // Set exposure time to the maximum supported
  ret = previewRequest.metadata.update(ANDROID_SENSOR_EXPOSURE_TIME,
                                       &maxExposureTime, 1);
  ASSERT_EQ(0, ret);
  // Switch AE mode to off in order for the exposure
  // to get applied.
  uint8_t aeMode = ANDROID_CONTROL_AE_MODE_OFF;
  ret = previewRequest.metadata.update(ANDROID_CONTROL_AE_MODE, &aeMode, 1);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(previewRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  previewRequestId = ret;
  printf("%s: Exposure time set to: %" PRId64 "\n", __func__, maxExposureTime);

  // Run preview for some time
  sleep(5);
  dump_yuv_ = true;

  ret = device_client_->CancelRequest(previewRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Preview request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, Video1080pSnapshot4kSaturation) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest, snapshotRequest;
  int64_t lastFrameNumber;
  int32_t videoStreamId, videoRequestId;
  int32_t snapshotStreamId;
  int32_t sat;

  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));

  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 1920;
  streamParams.height = 1080;
  streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  videoStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(videoStreamId, 0);
  videoRequest.streamIds.add(videoStreamId);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 1;
  streamParams.format = HAL_PIXEL_FORMAT_BLOB;
  streamParams.width = 3840;
  streamParams.height = 2160;
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](StreamBuffer buffer) { SnapshotCb(buffer); };

  snapshotStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(snapshotStreamId, 0);
  snapshotRequest.streamIds.add(snapshotStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                                            &videoRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_SNAPSHOT,
                                            &snapshotRequest.metadata);
  ASSERT_EQ(0, ret);

  sat = 0;
  ret = videoRequest.metadata.update(QCAMERA3_USE_SATURATION, &sat, 1);
  ASSERT_EQ(ret, 0);
  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  // Take a live snapshot
  ret = device_client_->SubmitRequest(snapshotRequest, false, &lastFrameNumber);
  ASSERT_GE(ret, 0);

  sat = 10;
  ret = videoRequest.metadata.update(QCAMERA3_USE_SATURATION, &sat, 1);
  ASSERT_EQ(ret, 0);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  // Take a live snapshot
  ret = device_client_->SubmitRequest(snapshotRequest, false, &lastFrameNumber);
  ASSERT_GE(ret, 0);

  ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, Video1080pSnapshot4kISO) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest, snapshotRequest;
  int64_t lastFrameNumber;
  int32_t videoStreamId, videoRequestId;
  int32_t snapshotStreamId;
  int32_t mode;
  int64_t value;

  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));

  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 1920;
  streamParams.height = 1080;
  streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  videoStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(videoStreamId, 0);
  videoRequest.streamIds.add(videoStreamId);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 1;
  streamParams.format = HAL_PIXEL_FORMAT_BLOB;
  streamParams.width = 3840;
  streamParams.height = 2160;
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](StreamBuffer buffer) { SnapshotCb(buffer); };

  snapshotStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(snapshotStreamId, 0);
  snapshotRequest.streamIds.add(snapshotStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                                            &videoRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_SNAPSHOT,
                                            &snapshotRequest.metadata);
  ASSERT_EQ(0, ret);
  mode = 0;
  value = 2;
  ret = videoRequest.metadata.update(QCAMERA3_SELECT_PRIORITY, &mode, 1);
  ASSERT_EQ(ret, 0);
  ret = videoRequest.metadata.update(QCAMERA3_USE_ISO_EXP_PRIORITY, &value, 1);
  ASSERT_EQ(ret, 0);
  ret = snapshotRequest.metadata.update(QCAMERA3_SELECT_PRIORITY, &mode, 1);
  ASSERT_EQ(ret, 0);
  ret = snapshotRequest.metadata.update(QCAMERA3_USE_ISO_EXP_PRIORITY,
          &value, 1);
  ASSERT_EQ(ret, 0);
  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(10);

  // Take a live snapshot
  ret = device_client_->SubmitRequest(snapshotRequest, false, &lastFrameNumber);
  ASSERT_GE(ret, 0);

  mode = 0;
  value = 5;
  ret = videoRequest.metadata.update(QCAMERA3_SELECT_PRIORITY, &mode, 1);
  ASSERT_EQ(ret, 0);
  ret = videoRequest.metadata.update(QCAMERA3_USE_ISO_EXP_PRIORITY, &value, 1);
  ASSERT_EQ(ret, 0);
  ret = snapshotRequest.metadata.update(QCAMERA3_SELECT_PRIORITY, &mode, 1);
  ASSERT_EQ(ret, 0);
  ret = snapshotRequest.metadata.update(QCAMERA3_USE_ISO_EXP_PRIORITY,
         &value, 1);
  ASSERT_EQ(ret, 0);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(10);

  // Take a live snapshot
  ret = device_client_->SubmitRequest(snapshotRequest, false, &lastFrameNumber);
  ASSERT_GE(ret, 0);

  ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, Video1080pSnapshot4kWNR) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest, snapshotRequest;
  int64_t lastFrameNumber;
  int32_t videoStreamId, videoRequestId;
  int32_t snapshotStreamId;
  uint8_t mode;

  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));

  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 1920;
  streamParams.height = 1080;
  streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  videoStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(videoStreamId, 0);
  videoRequest.streamIds.add(videoStreamId);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 1;
  streamParams.format = HAL_PIXEL_FORMAT_BLOB;
  streamParams.width = 3840;
  streamParams.height = 2160;
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](StreamBuffer buffer) { SnapshotCb(buffer); };

  snapshotStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(snapshotStreamId, 0);
  snapshotRequest.streamIds.add(snapshotStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                                            &videoRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_SNAPSHOT,
                                            &snapshotRequest.metadata);
  ASSERT_EQ(0, ret);
  mode = 0;
  ret = snapshotRequest.metadata.update(ANDROID_NOISE_REDUCTION_MODE,
        &mode, 1);
  ASSERT_EQ(ret, 0);
  ret = videoRequest.metadata.update(ANDROID_NOISE_REDUCTION_MODE, &mode, 1);
  ASSERT_EQ(ret, 0);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  // Take a live snapshot
  ret = device_client_->SubmitRequest(snapshotRequest, false,
        &lastFrameNumber);
  ASSERT_GE(ret, 0);

  mode = 1;
  ret = snapshotRequest.metadata.update(ANDROID_NOISE_REDUCTION_MODE,
        &mode, 1);
  ASSERT_EQ(ret, 0);
  ret = videoRequest.metadata.update(ANDROID_NOISE_REDUCTION_MODE, &mode, 1);
  ASSERT_EQ(ret, 0);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  // Take a live snapshot
  ret = device_client_->SubmitRequest(snapshotRequest, false,
        &lastFrameNumber);
  ASSERT_GE(ret, 0);

  ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, Video4KLiveSnapshot4K) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest, snapshotRequest;
  int64_t lastFrameNumber;
  int32_t videoStreamId, videoRequestId;
  int32_t snapshotStreamId;
  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 3840;
  streamParams.height = 2160;
  streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  videoStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(videoStreamId, 0);
  videoRequest.streamIds.add(videoStreamId);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 1;
  streamParams.format = HAL_PIXEL_FORMAT_BLOB;
  streamParams.width = 3840;
  streamParams.height = 2160;
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](StreamBuffer buffer) { SnapshotCb(buffer); };

  snapshotStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(snapshotStreamId, 0);
  snapshotRequest.streamIds.add(snapshotStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                                            &videoRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_SNAPSHOT,
                                            &snapshotRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  // Take a live snapshot
  ret = device_client_->SubmitRequest(snapshotRequest, false, &lastFrameNumber);
  ASSERT_GE(ret, 0);

  // Run video for some time
  sleep(5);

  ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, Video4KPlus180pLiveSnapshot4KYUVPreview1080p) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest, snapshotRequest;
  int64_t lastFrameNumber;
  int32_t repeatingStreamId, videoRequestId;
  int32_t snapshotStreamId;
  int32_t streamFPS = 30;
  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  // 4K video stream
  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 3840;
  streamParams.height = 2160;
  streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCbAvgFPS(buffer); };

  repeatingStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(repeatingStreamId, 0);
  videoRequest.streamIds.add(repeatingStreamId);

  // 180p video stream
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 320;
  streamParams.height = 180;
  streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };
  repeatingStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(repeatingStreamId, 0);
  videoRequest.streamIds.add(repeatingStreamId);

  // 1080p preview stream
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 1920;
  streamParams.height = 1080;
  streamParams.grallocFlags = GRALLOC_USAGE_HW_FB;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };
  repeatingStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(repeatingStreamId, 0);
  videoRequest.streamIds.add(repeatingStreamId);

  // 4K YUV snapshot
  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 1;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 3840;
  streamParams.height = 2160;
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  snapshotStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(snapshotStreamId, 0);
  snapshotRequest.streamIds.add(snapshotStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                                            &videoRequest.metadata);
  ASSERT_EQ(0, ret);

  int32_t fpsRange[2];
  fpsRange[0] = streamFPS;
  fpsRange[1] = streamFPS;
  videoRequest.metadata.update(ANDROID_CONTROL_AE_TARGET_FPS_RANGE, fpsRange,
                               2);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_SNAPSHOT,
                                            &snapshotRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  // Take a live snapshot
  ret = device_client_->SubmitRequest(snapshotRequest, false, &lastFrameNumber);
  ASSERT_GE(ret, 0);

  // Run video for some time
  sleep(5);

  ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);

  float allowedDeviation = streamFPS * FPS_ALLOWED_DEV;
  float measuredDeviation = fabs(streamFPS - avg_fps_);
  ASSERT_GE(allowedDeviation, measuredDeviation);
  printf("%s: Measured deviation: %5.2f, allowed deviation: %5.2f\n", __func__,
         measuredDeviation, allowedDeviation);
}

TEST_F(Camera3Gtest, Video1080pAFR) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest;
  int64_t lastFrameNumber;
  int32_t repeatingStreamId, videoRequestId;
  CameraMetadata staticInfo;
  int32_t fpsRange[2] = {0, 0};

  auto ret = device_client_->GetCameraInfo(camera_idx_, &staticInfo);
  ASSERT_EQ(0, ret);


  if (staticInfo.exists(ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)) {
    camera_metadata_entry metaEntry =
        staticInfo.find(ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
    for (uint32_t i = 0; i < metaEntry.count; i += 2) {
      if(metaEntry.data.i32[i+1] - metaEntry.data.i32[i+0] > fpsRange[1] - fpsRange[0]) {
        fpsRange[0] = metaEntry.data.i32[i+0];
        fpsRange[1] = metaEntry.data.i32[i+1];
      }
    }
  }
  printf("Chosen range %d - %d\n", fpsRange[0], fpsRange[1]);



  ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 1920;
  streamParams.height = 1080;
  streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCbAvgFPS(buffer); };

  // 1080p Stream1
  repeatingStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(repeatingStreamId, 0);
  videoRequest.streamIds.add(repeatingStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                                            &videoRequest.metadata);
  ASSERT_EQ(0, ret);
  videoRequest.metadata.update(ANDROID_CONTROL_AE_TARGET_FPS_RANGE, fpsRange,
                               2);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, Video1080pSharpness) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest;
  int64_t lastFrameNumber;
  int32_t repeatingStreamId, videoRequestId;
  CameraMetadata staticInfo;
  unsigned char edge_mode = ANDROID_EDGE_MODE_OFF;
  unsigned char strength = 100;

  auto ret = device_client_->GetCameraInfo(camera_idx_, &staticInfo);
  ASSERT_EQ(0, ret);


  if (staticInfo.exists(ANDROID_EDGE_AVAILABLE_EDGE_MODES)) {
    camera_metadata_entry metaEntry =
        staticInfo.find(ANDROID_EDGE_AVAILABLE_EDGE_MODES);
    for (uint32_t i = 0; i < metaEntry.count; i++) {
      // Prefer high quality mode
      if(metaEntry.data.u8[i] == ANDROID_EDGE_MODE_HIGH_QUALITY) {
        edge_mode = ANDROID_EDGE_MODE_HIGH_QUALITY;
        break;
      }
      if(metaEntry.data.u8[i] == ANDROID_EDGE_MODE_FAST) {
        edge_mode = ANDROID_EDGE_MODE_FAST;
      }
    }
  }
  printf("Chosen mode %d\n", edge_mode);


  ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 1920;
  streamParams.height = 1080;
  streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCbDumpNVXX(buffer); };

  // 1080p Stream1
  repeatingStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(repeatingStreamId, 0);
  videoRequest.streamIds.add(repeatingStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                                            &videoRequest.metadata);
  ASSERT_EQ(0, ret);
  videoRequest.metadata.update(ANDROID_EDGE_MODE,     &edge_mode, 1);
  videoRequest.metadata.update(ANDROID_EDGE_STRENGTH, &strength, 1);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  dump_yuv_ = true;

  ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);


  ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 1920;
  streamParams.height = 1080;
  streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCbDumpNVXX(buffer); };

  // 1080p Stream1
  repeatingStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(repeatingStreamId, 0);
  videoRequest.streamIds.add(repeatingStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                                            &videoRequest.metadata);
  ASSERT_EQ(0, ret);
  edge_mode = ANDROID_EDGE_MODE_OFF;
  strength = 00;
  videoRequest.metadata.update(ANDROID_EDGE_MODE,     &edge_mode, 1);
  videoRequest.metadata.update(ANDROID_EDGE_STRENGTH, &strength, 1);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  dump_yuv_ = true;

  ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);

}

TEST_F(Camera3Gtest, Video1080pZoom) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest;
  int64_t lastFrameNumber;
  int32_t repeatingStreamId, videoRequestId = -1;
  CameraMetadata staticInfo;
  int32_t width, height;
  int32_t crop_rgn[4];
  int i;

  auto ret = device_client_->GetCameraInfo(camera_idx_, &staticInfo);
  ASSERT_EQ(0, ret);

  ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 1920;
  streamParams.height = 1080;
  streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCbDumpNVXX(buffer); };

  // 1080p Stream1
  repeatingStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(repeatingStreamId, 0);
  videoRequest.streamIds.add(repeatingStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                                            &videoRequest.metadata);
  ASSERT_EQ(0, ret);

  GetMaxRAWSize(width, height);

  for (i = width; i >= width/4;
        i -= width*3/(4*(ZOOM_STEPS-1))) {
    crop_rgn[2] = i & ~(0xFu);
    crop_rgn[3] = (crop_rgn[2]*height/width) & ~(0xFu);
    crop_rgn[0] = (width - crop_rgn[2])/2;
    crop_rgn[1] = (height - crop_rgn[3])/2;

    videoRequest.metadata.update(ANDROID_SCALER_CROP_REGION, crop_rgn, 4);

    ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
    ASSERT_GE(ret, 0);
    videoRequestId = ret;

    // Run video for some time
    sleep(5);

    dump_yuv_ = true;
    sleep(5);
  }


  if (videoRequestId >= 0) {
    ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
    ASSERT_EQ(0, ret);
  }

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, Video1080pThreeStreams) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest;
  int64_t lastFrameNumber;
  int32_t repeatingStreamId, videoRequestId;
  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 1920;
  streamParams.height = 1080;
  streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCbAvgFPS(buffer); };

  // 1080p Stream1
  repeatingStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(repeatingStreamId, 0);
  videoRequest.streamIds.add(repeatingStreamId);

  // 1080p Stream2
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };
  repeatingStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(repeatingStreamId, 0);
  videoRequest.streamIds.add(repeatingStreamId);

  // 1080p Stream3
  repeatingStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(repeatingStreamId, 0);
  videoRequest.streamIds.add(repeatingStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                                            &videoRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, ThreeVideo1080Plus180pPreview1080pLiveSnapshot4KYUV) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest, snapshotRequest;
  int64_t lastFrameNumber;
  int32_t repeatingStreamId, videoRequestId;
  int32_t snapshotStreamId;
  int32_t streamFPS = 30;
  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 1920;
  streamParams.height = 1080;
  streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  // 1080p Stream1
  repeatingStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(repeatingStreamId, 0);
  videoRequest.streamIds.add(repeatingStreamId);

  // 1080p Stream2
  repeatingStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(repeatingStreamId, 0);
  videoRequest.streamIds.add(repeatingStreamId);

  // 1080p Stream3
  repeatingStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(repeatingStreamId, 0);
  videoRequest.streamIds.add(repeatingStreamId);

  // 1080p Preview
  streamParams.grallocFlags = GRALLOC_USAGE_HW_FB;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCbAvgFPS(buffer); };
  repeatingStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(repeatingStreamId, 0);
  videoRequest.streamIds.add(repeatingStreamId);

  // 180p Video
  streamParams.width = 320;
  streamParams.height = 180;
  streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };
  repeatingStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(repeatingStreamId, 0);
  videoRequest.streamIds.add(repeatingStreamId);

  // 4K YUV snapshot
  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 1;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 3840;
  streamParams.height = 2160;
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  snapshotStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(snapshotStreamId, 0);
  snapshotRequest.streamIds.add(snapshotStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                                            &videoRequest.metadata);
  ASSERT_EQ(0, ret);

  int32_t fpsRange[2];
  fpsRange[0] = streamFPS;
  fpsRange[1] = streamFPS;
  videoRequest.metadata.update(ANDROID_CONTROL_AE_TARGET_FPS_RANGE, fpsRange,
                               2);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_SNAPSHOT,
                                            &snapshotRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  // Take a live snapshot
  ret = device_client_->SubmitRequest(snapshotRequest, false, &lastFrameNumber);
  ASSERT_GE(ret, 0);

  // Run video for some time
  sleep(5);

  ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);

  float allowedDeviation = streamFPS * FPS_ALLOWED_DEV;
  float measuredDeviation = fabs(streamFPS - avg_fps_);
  ASSERT_GE(allowedDeviation, measuredDeviation);
  printf("%s: Measured deviation: %5.2f, allowed deviation: %5.2f\n", __func__,
         measuredDeviation, allowedDeviation);
}

TEST_F(Camera3Gtest, DynamicDeleteVideo1080p) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest;
  int64_t lastFrameNumber;
  int32_t videoStreamId, videoRequestId, videoStreamId1;
  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 1920;
  streamParams.height = 1080;
  streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  // Stream1
  videoStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(videoStreamId, 0);
  videoRequest.streamIds.add(videoStreamId);

  // Add a second 1080p video stream
  streamParams.cb = [&](StreamBuffer buffer) { StreamCbSignalOnFrame(buffer); };
  videoStreamId1 = device_client_->CreateStream(streamParams);
  ASSERT_GE(videoStreamId1, 0);
  videoRequest.streamIds.add(videoStreamId1);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                                            &videoRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  //Update request and remove second stream;
  input_buffer_flag_ = true;
  videoRequest.streamIds.clear();
  videoRequest.streamIds.add(videoStreamId);
  ret = device_client_->SubmitRequest(videoRequest, true,
                                      &input_last_frame_number_);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  printf("%s: Video request updated last frame number: %" PRId64 "\n",
         __func__, input_last_frame_number_);

  pthread_mutex_lock(&input_lock_);
  while(input_buffer_flag_) {
    ret = cond_wait_relative(&input_cond_, &input_lock_, PROCESS_TIMEOUT);
    ASSERT_EQ(0, ret);
  }
  pthread_mutex_unlock(&input_lock_);

  ret = device_client_->DeleteStream(videoStreamId1, true);
  ASSERT_EQ(0, ret);

  //Continue streaming with just one stream
  sleep(5);

  ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, DynamicReconfigureVideo1080p) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest;
  int64_t lastFrameNumber;
  int32_t videoStreamId, videoRequestId, videoStreamId1;
  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 1920;
  streamParams.height = 1080;
  streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  // Stream1
  videoStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(videoStreamId, 0);
  videoRequest.streamIds.add(videoStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                                            &videoRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  // Add a second 1080p video stream dynamically
  videoStreamId1 = device_client_->CreateStream(streamParams);
  ASSERT_GE(videoStreamId1, 0);

  ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);

  videoRequest.streamIds.add(videoStreamId1);
  ret = device_client_->SubmitRequest(videoRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run with both streams for some time
  sleep(5);

  ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, SwitchPreview1080pVideo4K) {
  int32_t streamId, requestId;
  int32_t videoUsage = GRALLOC_USAGE_HW_FB |
      private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  int32_t previewUsage = GRALLOC_USAGE_HW_FB;
  int32_t ret = 0;

  for (uint32_t i = 0; i < ITERATION_COUNT; i++) {
    printf("%s: Iteration: %d\n", __func__, i);
    ret = StartSreaming(previewUsage, 1920, 1080,
                        CAMERA3_TEMPLATE_PREVIEW, streamId,
                        requestId);
    ASSERT_EQ(0, ret);

    // Run video for some time
    sleep(5);

    ret = StopDeleteStream(streamId, requestId);
    ASSERT_EQ(0, ret);

    ret = StartSreaming(videoUsage, 3840, 2160,
                        CAMERA3_TEMPLATE_VIDEO_RECORD, streamId,
                        requestId);
    ASSERT_EQ(0, ret);

    // Run preview for some time
    sleep(5);

    ret = StopDeleteStream(streamId, requestId);
    ASSERT_EQ(0, ret);
  }

  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, InvalidRequest) {
  CameraStreamParameters streamParams;
  Camera3Request previewRequest;
  int64_t lastFrameNumber;
  int32_t previewStreamId;

  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = PREVIEW_WIDTH;
  streamParams.height = PREVIEW_HEIGHT;
  streamParams.grallocFlags = GRALLOC_USAGE_HW_FB;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  previewStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(previewStreamId, 0);
  previewRequest.streamIds.add(previewStreamId);
  //Try to include one stream twice in the same request
  previewRequest.streamIds.add(previewStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_PREVIEW,
                                            &previewRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(previewRequest, true, &lastFrameNumber);
  ASSERT_LT(ret, 0);
}

TEST_F(Camera3Gtest, PrepareTeardownPreview) {
  CameraStreamParameters streamParams;
  Camera3Request previewRequest;
  int64_t lastFrameNumber;
  int32_t previewStreamId, previewRequestId;

  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = PREVIEW_WIDTH;
  streamParams.height = PREVIEW_HEIGHT;
  streamParams.grallocFlags = GRALLOC_USAGE_HW_FB;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  previewStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(previewStreamId, 0);
  previewRequest.streamIds.add(previewStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_PREVIEW,
                                            &previewRequest.metadata);
  ASSERT_EQ(0, ret);

  pthread_mutex_lock(&prepare_lock_);
  prepare_flag_ = false;
  ret = device_client_->Prepare(previewStreamId);
  ASSERT_EQ(0, ret);

  while(!prepare_flag_) {
    ret = cond_wait_relative(&prepare_cond_, &prepare_lock_, PREPARE_TIMEOUT);
    ASSERT_EQ(0, ret);
  }
  pthread_mutex_unlock(&prepare_lock_);

  ret = device_client_->SubmitRequest(previewRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  previewRequestId = ret;

  // Run preview for some time
  sleep(5);

  ret = device_client_->TearDown(previewStreamId);
  ASSERT_LT(ret, 0);

  ret = device_client_->CancelRequest(previewRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Preview request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);

  ret = device_client_->TearDown(previewStreamId);
  ASSERT_EQ(ret, 0);

  pthread_mutex_lock(&prepare_lock_);
  prepare_flag_ = false;
  ret = device_client_->Prepare(previewStreamId);
  ASSERT_EQ(0, ret);

  while(!prepare_flag_) {
    ret = cond_wait_relative(&prepare_cond_, &prepare_lock_, PREPARE_TIMEOUT);
    ASSERT_EQ(0, ret);
  }
  pthread_mutex_unlock(&prepare_lock_);

  ret = device_client_->SubmitRequest(previewRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  previewRequestId = ret;

  // Run preview for some time
  sleep(5);

  ret = device_client_->CancelRequest(previewRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Preview request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);

  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, HFRVideo1080p60FPS) {
  CameraStreamParameters stream_params;
  Camera3Request video_request;
  int64_t last_frame_number;
  int32_t video_stream_id, video_request_id;
  int32_t stream_width = 1920;
  int32_t stream_height = 1080;
  int32_t stream_fps = 60;

  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&stream_params, 0, sizeof(stream_params));
  stream_params.bufferCount = HFR_BUFFER_COUNT;
  stream_params.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  stream_params.width = stream_width;
  stream_params.height = stream_height;
  stream_params.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  stream_params.cb = [&](StreamBuffer buffer) { StreamCbAvgFPS(buffer); };

  video_stream_id = device_client_->CreateStream(stream_params);
  ASSERT_GE(video_stream_id, 0);
  video_request.streamIds.add(video_stream_id);

  ret = device_client_->EndConfigure(true);
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                                            &video_request.metadata);
  ASSERT_EQ(0, ret);

  int32_t fps_range[2];
  fps_range[0] = stream_fps;
  fps_range[1] = stream_fps;

  video_request.metadata.update(ANDROID_CONTROL_AE_TARGET_FPS_RANGE, fps_range,
                              2);

  int32_t sensor_vendor_mode = 6;
  video_request.metadata.update(QCAMERA3_VENDOR_SENSOR_MODE,
                                &sensor_vendor_mode, 1);

  List<Camera3Request> requests;
  requests.push_back(video_request);

  ret = device_client_->SubmitRequestList(requests, true, &last_frame_number);
  ASSERT_GE(ret, 0);
  video_request_id = ret;

  // Run video for some time
  sleep(5);

  ret = device_client_->CancelRequest(video_request_id, &last_frame_number);
  ASSERT_EQ(0, ret);

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
         __func__, last_frame_number);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
  float allowedDeviation = stream_fps * FPS_ALLOWED_DEV;
  float measuredDeviation = fabs(stream_fps - avg_fps_);
  ASSERT_GE(allowedDeviation, measuredDeviation);
  printf("%s: Measured deviation: %5.2f, allowed deviation: %5.2f\n", __func__,
         measuredDeviation, allowedDeviation);
}

TEST_F(Camera3Gtest, HFRVideo720p120FPS) {
  CameraStreamParameters streamParams;
  Camera3Request videoRequest;
  int64_t lastFrameNumber;
  int32_t videoStreamId, videoRequestId;
  CameraMetadata staticInfo;
  bool isHFRSupported = false;
  uint32_t widthOffset = 0;
  uint32_t heightOffset = 1;
  uint32_t minFPSOffset = 2;
  uint32_t maxFPSOffset = 3;
  uint32_t batchSizeOffset = 4;
  uint32_t HFRSize = 5;
  int32_t streamWidth = 1280;
  int32_t streamHeight = 720;
  int32_t streamFPS = 120;
  int32_t batchSize = -1;

  auto ret = device_client_->GetCameraInfo(camera_idx_, &staticInfo);
  ASSERT_EQ(0, ret);

  camera_metadata_entry metaEntry =
      staticInfo.find(ANDROID_REQUEST_AVAILABLE_CAPABILITIES);
  for (uint32_t i = 0; i < metaEntry.count; ++i) {
    uint8_t caps = metaEntry.data.u8[i];
    if (ANDROID_REQUEST_AVAILABLE_CAPABILITIES_CONSTRAINED_HIGH_SPEED_VIDEO ==
        caps) {
      isHFRSupported = true;
      break;
    }
  }
  ASSERT_TRUE(isHFRSupported);

  metaEntry = staticInfo.find(
      ANDROID_CONTROL_AVAILABLE_HIGH_SPEED_VIDEO_CONFIGURATIONS);
  for (uint32_t i = 0; i < metaEntry.count; i += HFRSize) {
    int32_t width = metaEntry.data.i32[i + widthOffset];
    int32_t height = metaEntry.data.i32[i + heightOffset];
    int32_t minFPS = metaEntry.data.i32[i + minFPSOffset];
    int32_t maxFPS = metaEntry.data.i32[i + maxFPSOffset];
    int32_t batch = metaEntry.data.i32[i + batchSizeOffset];
    if ((streamWidth == width) && (streamHeight == height) &&
        (streamFPS == minFPS) && (streamFPS == maxFPS)) {
      batchSize = batch;
      break;
    }
  }
  ASSERT_GT(batchSize, 0);

  ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = HFR_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = streamWidth;
  streamParams.height = streamHeight;
  streamParams.grallocFlags =
      GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCbAvgFPS(buffer); };

  videoStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(videoStreamId, 0);
  videoRequest.streamIds.add(videoStreamId);

  ret = device_client_->EndConfigure(true);
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_VIDEO_RECORD,
                                            &videoRequest.metadata);
  ASSERT_EQ(0, ret);

  int32_t fpsRange[2];
  fpsRange[0] = streamFPS;
  fpsRange[1] = streamFPS;

  videoRequest.metadata.update(ANDROID_CONTROL_AE_TARGET_FPS_RANGE, fpsRange,
                               2);
  List<Camera3Request> requests;
  for (int32_t i = 0; i < batchSize; i++) {
    requests.push_back(videoRequest);
  }

  ret = device_client_->SubmitRequestList(requests, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  videoRequestId = ret;

  // Run video for some time
  sleep(5);

  ret = device_client_->CancelRequest(videoRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Video request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
  float allowedDeviation = streamFPS * FPS_ALLOWED_DEV;
  float measuredDeviation = fabs(streamFPS - avg_fps_);
  ASSERT_GE(allowedDeviation, measuredDeviation);
  printf("%s: Measured deviation: %5.2f, allowed deviation: %5.2f\n", __func__,
         measuredDeviation, allowedDeviation);
}

TEST_F(Camera3Gtest, ReprocessYUVToYUV) {
  CameraStreamParameters streamParams;
  CameraInputStreamParameters inputStreamParams;
  Camera3Request previewRequest, yuvRequest, reprocessRequest;
  int64_t lastFrameNumber;
  int32_t previewRequestId;
  int32_t previewStreamId, yuvStreamId, inputStreamId, yuvOutputStreamId;
  int32_t yuvSize[2] = {0, 0};

  ASSERT_TRUE(IsInputSupported());
  ASSERT_EQ(0, GetMaxYUVSize(yuvSize[0], yuvSize[1]));

  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = PREVIEW_WIDTH;
  streamParams.height = PREVIEW_HEIGHT;
  streamParams.grallocFlags = GRALLOC_USAGE_HW_FB;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  previewStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(previewStreamId, 0);
  previewRequest.streamIds.add(previewStreamId);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 1;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = yuvSize[0];
  streamParams.height = yuvSize[1];
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](StreamBuffer buffer) { InputCb(buffer); };

  yuvStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(yuvStreamId, 0);
  yuvRequest.streamIds.add(yuvStreamId);
  input_stream_id_ = yuvStreamId;

  memset(&inputStreamParams, 0, sizeof(inputStreamParams));
  inputStreamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  inputStreamParams.width = yuvSize[0];
  inputStreamParams.height = yuvSize[1];
  inputStreamParams.get_input_buffer = [&] (StreamBuffer &buffer)
      { GetInputBuffer(buffer); };
  inputStreamParams.return_input_buffer  = [&] (StreamBuffer &buffer)
      { ReturnInputBuffer(buffer); };

  inputStreamId = device_client_->CreateInputStream(inputStreamParams);
  ASSERT_GE(inputStreamId, 0);
  reprocessRequest.streamIds.add(inputStreamId);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 1;
  streamParams.format = HAL_PIXEL_FORMAT_YCbCr_420_888;
  streamParams.width = yuvSize[0];
  streamParams.height = yuvSize[1];
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCbDumpNVXX(buffer); };

  yuvOutputStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(yuvOutputStreamId, 0);
  reprocessRequest.streamIds.add(yuvOutputStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_PREVIEW,
                                            &previewRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_STILL_CAPTURE,
                                            &yuvRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(previewRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  previewRequestId = ret;

  // Run preview so 3A can converge
  sleep(2);

  ret = device_client_->CancelRequest(previewRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);

  input_buffer_flag_ = true;
  cache_last_meta_ = true;
  // Take a YUV input
  ret = device_client_->SubmitRequest(yuvRequest, false, &lastFrameNumber);
  ASSERT_GE(ret, 0);

  pthread_mutex_lock(&input_lock_);
  while(input_buffer_flag_) {
    ret = cond_wait_relative(&input_cond_, &input_lock_, PROCESS_TIMEOUT);
    ASSERT_EQ(0, ret);
  }
  pthread_mutex_unlock(&input_lock_);

  ASSERT_FALSE(cache_last_meta_);
  reprocessRequest.metadata = last_meta_;

  dump_yuv_ = true;
  reprocess_flag_ = true;
  ret = device_client_->SubmitRequest(reprocessRequest, false,
                                      &lastFrameNumber);
  ASSERT_GE(ret, 0);

  pthread_mutex_lock(&reprocess_lock_);
  while(reprocess_flag_) {
    ret = cond_wait_relative(&reprocess_cond_, &reprocess_lock_,
                             PROCESS_TIMEOUT);
    ASSERT_EQ(0, ret);
  }
  pthread_mutex_unlock(&reprocess_lock_);

  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, ReprocessRAWToYUV1080p) {
  CameraStreamParameters streamParams;
  CameraInputStreamParameters inputStreamParams;
  Camera3Request previewRequest, rawRequest, reprocessRequest;
  int64_t lastFrameNumber;
  int32_t previewRequestId;
  int32_t previewStreamId, rawStreamId, inputStreamId, yuvOutputStreamId;
  int32_t rawSize[2] = {0, 0};

  ASSERT_TRUE(IsInputSupported());
  ASSERT_EQ(0, GetMaxRAWSize(rawSize[0], rawSize[1]));

  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = PREVIEW_WIDTH;
  streamParams.height = PREVIEW_HEIGHT;
  streamParams.grallocFlags = GRALLOC_USAGE_HW_FB;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  previewStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(previewStreamId, 0);
  previewRequest.streamIds.add(previewStreamId);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 1;
  streamParams.format = HAL_PIXEL_FORMAT_RAW10;
  streamParams.width = rawSize[0];
  streamParams.height = rawSize[1];
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](StreamBuffer buffer) { InputCb(buffer); };

  rawStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(rawStreamId, 0);
  rawRequest.streamIds.add(rawStreamId);
  input_stream_id_ = rawStreamId;

  memset(&inputStreamParams, 0, sizeof(inputStreamParams));
  inputStreamParams.format = HAL_PIXEL_FORMAT_RAW10;
  inputStreamParams.width = rawSize[0];
  inputStreamParams.height = rawSize[1];
  inputStreamParams.get_input_buffer = [&] (StreamBuffer &buffer)
      { GetInputBuffer(buffer); };
  inputStreamParams.return_input_buffer  = [&] (StreamBuffer &buffer)
      { ReturnInputBuffer(buffer); };

  inputStreamId = device_client_->CreateInputStream(inputStreamParams);
  ASSERT_GE(inputStreamId, 0);
  reprocessRequest.streamIds.add(inputStreamId);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 1;
  streamParams.format = HAL_PIXEL_FORMAT_YCbCr_420_888;
  streamParams.width = 1920;
  streamParams.height = 1080;
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCbDumpNVXX(buffer); };

  yuvOutputStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(yuvOutputStreamId, 0);
  reprocessRequest.streamIds.add(yuvOutputStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_PREVIEW,
                                            &previewRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_STILL_CAPTURE,
                                            &rawRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(previewRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  previewRequestId = ret;

  // Run preview so 3A can converge
  sleep(2);

  ret = device_client_->CancelRequest(previewRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);

  input_buffer_flag_ = true;
  cache_last_meta_ = true;
  // Take a RAW input
  ret = device_client_->SubmitRequest(rawRequest, false, &lastFrameNumber);
  ASSERT_GE(ret, 0);

  pthread_mutex_lock(&input_lock_);
  while(input_buffer_flag_) {
    ret = cond_wait_relative(&input_cond_, &input_lock_, PROCESS_TIMEOUT);
    ASSERT_EQ(0, ret);
  }
  pthread_mutex_unlock(&input_lock_);

  ASSERT_FALSE(cache_last_meta_);
  reprocessRequest.metadata = last_meta_;

  dump_yuv_ = true;
  reprocess_flag_ = true;
  ret = device_client_->SubmitRequest(reprocessRequest, false,
                                      &lastFrameNumber);
  ASSERT_GE(ret, 0);

  pthread_mutex_lock(&reprocess_lock_);
  while(reprocess_flag_) {
    ret = cond_wait_relative(&reprocess_cond_, &reprocess_lock_,
                             PROCESS_TIMEOUT);
    ASSERT_EQ(0, ret);
  }
  pthread_mutex_unlock(&reprocess_lock_);

  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, ReprocessZSL12MpToYUV4K) {
  CameraStreamParameters stream_params;
  CameraInputStreamParameters input_stream_params;
  Camera3Request zsl_request, reprocess_request;
  int64_t last_frame_number;
  int32_t zsl_width = 4000;
  int32_t zsl_height = 3000;
  int32_t yuv_width = 3840;
  int32_t yuv_height = 2160;
  int32_t zsl_stream_id, input_stream_id, yuv_output_stream_id;
  int32_t zsl_request_id;

  ASSERT_TRUE(IsInputSupported());

  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&stream_params, 0, sizeof(stream_params));
  stream_params.bufferCount = STREAM_BUFFER_COUNT;
  stream_params.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  stream_params.width = zsl_width;
  stream_params.height = zsl_height;
  stream_params.grallocFlags = GRALLOC_USAGE_HW_FB|GRALLOC_USAGE_HW_CAMERA_ZSL;
  stream_params.cb = [&](StreamBuffer buffer) { InputCb(buffer); };

  zsl_stream_id = device_client_->CreateStream(stream_params);
  ASSERT_GE(zsl_stream_id, 0);
  zsl_request.streamIds.add(zsl_stream_id);
  input_stream_id_ = zsl_stream_id;

  memset(&input_stream_params, 0, sizeof(input_stream_params));
  input_stream_params.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  input_stream_params.width = zsl_width;
  input_stream_params.height = zsl_height;
  input_stream_params.get_input_buffer = [&] (StreamBuffer &buffer)
      { GetInputBuffer(buffer); };
  input_stream_params.return_input_buffer  = [&] (StreamBuffer &buffer)
      { ReturnInputBuffer(buffer); };

  input_stream_id = device_client_->CreateInputStream(input_stream_params);
  ASSERT_GE(input_stream_id, 0);
  reprocess_request.streamIds.add(input_stream_id);

  memset(&stream_params, 0, sizeof(stream_params));
  stream_params.bufferCount = 1;
  stream_params.format = HAL_PIXEL_FORMAT_YCbCr_420_888;
  stream_params.width = yuv_width;
  stream_params.height = yuv_height;
  stream_params.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  stream_params.cb = [&](StreamBuffer buffer) { StreamCbDumpNVXX(buffer); };

  yuv_output_stream_id = device_client_->CreateStream(stream_params);
  ASSERT_GE(yuv_output_stream_id, 0);
  reprocess_request.streamIds.add(yuv_output_stream_id);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_ZERO_SHUTTER_LAG,
                                            &zsl_request.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(zsl_request, true, &last_frame_number);
  ASSERT_GE(ret, 0);
  zsl_request_id = ret;

  // Run ZSL so 3A can converge
  sleep(2);

  input_buffer_flag_ = true;
  cache_last_meta_ = true;
  pthread_mutex_lock(&input_lock_);
  while(input_buffer_flag_) {
    ret = cond_wait_relative(&input_cond_, &input_lock_, PROCESS_TIMEOUT);
    ASSERT_EQ(0, ret);
  }
  pthread_mutex_unlock(&input_lock_);

  //Ideally the user should keep its own buffer queue
  //with matched zsl buffers and metadata based on their
  //timestamps. For testing purposes this primitive method
  //should suffice though.
  pthread_mutex_lock(&meta_lock_);
  while(cache_last_meta_) {
    ret = cond_wait_relative(&meta_cond_, &meta_lock_, PROCESS_TIMEOUT);
    ASSERT_EQ(0, ret);
  }
  pthread_mutex_unlock(&meta_lock_);
  reprocess_request.metadata = last_meta_;

  dump_yuv_ = true;
  reprocess_flag_ = true;
  ret = device_client_->SubmitRequest(reprocess_request, false,
                                      &last_frame_number);
  ASSERT_GE(ret, 0);

  pthread_mutex_lock(&reprocess_lock_);
  while(reprocess_flag_) {
    ret = cond_wait_relative(&reprocess_cond_, &reprocess_lock_,
                             PROCESS_TIMEOUT);
    ASSERT_EQ(0, ret);
  }
  pthread_mutex_unlock(&reprocess_lock_);

  // Run ZSL for some time afterwards
  sleep(2);

  ret = device_client_->CancelRequest(zsl_request_id, &last_frame_number);
  ASSERT_EQ(0, ret);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);

  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, RAW16Bit) {
  CameraStreamParameters streamParams;
  Camera3Request previewRequest, rawRequest;
  int64_t lastFrameNumber;
  int32_t previewStreamId, previewRequestId;
  int32_t rawStreamId;
  int32_t rawSize[2];

  ASSERT_EQ(0, GetMaxRAWSize(rawSize[0], rawSize[1]));

  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = PREVIEW_WIDTH;
  streamParams.height = PREVIEW_HEIGHT;
  streamParams.grallocFlags = GRALLOC_USAGE_HW_FB;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  previewStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(previewStreamId, 0);
  previewRequest.streamIds.add(previewStreamId);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 1;
  streamParams.format = HAL_PIXEL_FORMAT_RAW16;
  streamParams.width = rawSize[0];
  streamParams.height = rawSize[1];
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](StreamBuffer buffer) { Raw16Cb(buffer); };

  rawStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(rawStreamId, 0);
  rawRequest.streamIds.add(rawStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_PREVIEW,
                                            &previewRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_STILL_CAPTURE,
                                            &rawRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(previewRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  previewRequestId = ret;

  // Run Preview for some time
  sleep(5);

  // Take a RAW
  ret = device_client_->SubmitRequest(rawRequest, false, &lastFrameNumber);
  ASSERT_GE(ret, 0);

  // Run Preview for some time
  sleep(5);

  ret = device_client_->CancelRequest(previewRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Preview request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}


TEST_F(Camera3Gtest, SnapshotBurstBracketing) {
  CameraStreamParameters streamParams;

  Camera3Request previewRequest;
  Camera3Request captureRequest;
  List<Camera3Request> burstRequests;
  int64_t lastFrameNumber;
  int32_t previewStreamId, previewRequestId;
  int32_t snapshotStreamId;

  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = PREVIEW_WIDTH;
  streamParams.height = PREVIEW_HEIGHT;
  streamParams.grallocFlags = GRALLOC_USAGE_HW_FB;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  previewStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(previewStreamId, 0);
  previewRequest.streamIds.add(previewStreamId);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 1;
  streamParams.format = HAL_PIXEL_FORMAT_BLOB;
  streamParams.width = PREVIEW_WIDTH;
  streamParams.height = PREVIEW_HEIGHT;
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](StreamBuffer buffer) { SnapshotCb(buffer); };

  snapshotStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(snapshotStreamId, 0);
  captureRequest.streamIds.add(snapshotStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_PREVIEW,
                                            &previewRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_STILL_CAPTURE,
                                            &captureRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(previewRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  previewRequestId = ret;
  int32_t evCompensation = 0;
  int32_t evCompensationStep = 0;
  CameraMetadata static_info;
  ret = device_client_->GetCameraInfo(camera_idx_, &static_info);
  ASSERT_EQ(0, ret);
  if (static_info.exists(ANDROID_CONTROL_AE_COMPENSATION_RANGE)) {
    camera_metadata_entry entry =
        static_info.find(ANDROID_CONTROL_AE_COMPENSATION_RANGE);
    evCompensation = entry.data.i32[1];
    evCompensationStep = (entry.data.i32[0] - entry.data.i32[1]) / (EV_STEPS - 1);
    printf ("Bracketing from %d to %d step %d\n", entry.data.i32[1], entry.data.i32[0], evCompensationStep);
  } else {
    printf ("EV Compensation not supported \n");
  }
  ASSERT_TRUE(evCompensationStep != 0);

  for (int i = 0; i < EV_STEPS; i++) {
        captureRequest.metadata.update(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION, &evCompensation, 1);
        evCompensation += evCompensationStep;
        burstRequests.push_back(captureRequest);
  }

  // Run Preview for some time
  sleep(5);

  // Take a Burst JPEG
  ret = device_client_->SubmitRequestList(burstRequests, false, &lastFrameNumber);
  ASSERT_GE(ret, 0);

  // Run Preview for some time
  sleep(5);

  ret = device_client_->CancelRequest(previewRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Preview request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}


TEST_F(Camera3Gtest, SnapshotAndRAW16Bit) {
  CameraStreamParameters streamParams;
  Camera3Request previewRequest, rawRequest;
  int64_t lastFrameNumber;
  int32_t previewStreamId, previewRequestId;
  int32_t rawStreamId;
  int32_t snapshotStreamId;
  int32_t rawSize[2];

  ASSERT_EQ(0, GetMaxRAWSize(rawSize[0], rawSize[1]));

  auto ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = PREVIEW_WIDTH;
  streamParams.height = PREVIEW_HEIGHT;
  streamParams.grallocFlags = GRALLOC_USAGE_HW_FB;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCb(buffer); };

  previewStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(previewStreamId, 0);
  previewRequest.streamIds.add(previewStreamId);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 1;
  streamParams.format = HAL_PIXEL_FORMAT_RAW16;
  streamParams.width = rawSize[0];
  streamParams.height = rawSize[1];
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](StreamBuffer buffer) { Raw16Cb(buffer); };

  rawStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(rawStreamId, 0);
  rawRequest.streamIds.add(rawStreamId);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = 1;
  streamParams.format = HAL_PIXEL_FORMAT_BLOB;
  streamParams.width = PREVIEW_WIDTH;
  streamParams.height = PREVIEW_HEIGHT;
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](StreamBuffer buffer) { SnapshotCb(buffer); };

  snapshotStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(snapshotStreamId, 0);
  rawRequest.streamIds.add(snapshotStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_PREVIEW,
                                            &previewRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_STILL_CAPTURE,
                                            &rawRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(previewRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  previewRequestId = ret;

  // Run Preview for some time
  sleep(5);

  // Take a RAW + JPEG
  ret = device_client_->SubmitRequest(rawRequest, false, &lastFrameNumber);
  ASSERT_GE(ret, 0);

  // Run Preview for some time
  sleep(5);

  ret = device_client_->CancelRequest(previewRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Preview request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, ExposureLockVGA) {
  CameraStreamParameters streamParams;
  Camera3Request previewRequest;
  CameraMetadata staticInfo;
  int64_t lastFrameNumber;
  int32_t previewStreamId, previewRequestId;

  auto ret = device_client_->GetCameraInfo(camera_idx_, &staticInfo);
  ASSERT_EQ(0, ret);

  ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 640;
  streamParams.height = 480;
  streamParams.grallocFlags = GRALLOC_USAGE_HW_FB;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCbAecLock(buffer); };

  previewStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(previewStreamId, 0);
  previewRequest.streamIds.add(previewStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  aec_lock_ = false;

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_PREVIEW,
                                             &previewRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(previewRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  previewRequestId = ret;

  sleep(5);

  uint8_t aeLock = ANDROID_CONTROL_AE_LOCK_ON;
  ret = previewRequest.metadata.update(ANDROID_CONTROL_AE_LOCK,
                                       &aeLock, 1);
  ASSERT_EQ(0, ret);
  aec_lock_ = true;
  printf("%s: AE Lock: %d\n", __func__, aeLock);

  ret = device_client_->SubmitRequest(previewRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  previewRequestId = ret;

  // Run preview for some time
  sleep(5);

  ret = device_client_->CancelRequest(previewRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Preview request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  aec_lock_ = false;
  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(Camera3Gtest, AwbLockVGA) {
  CameraStreamParameters streamParams;
  Camera3Request previewRequest;
  CameraMetadata staticInfo;
  int64_t lastFrameNumber;
  int32_t previewStreamId, previewRequestId;

  auto ret = device_client_->GetCameraInfo(camera_idx_, &staticInfo);
  ASSERT_EQ(0, ret);

  ret = device_client_->BeginConfigure();
  ASSERT_EQ(0, ret);

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = 640;
  streamParams.height = 480;
  streamParams.grallocFlags = GRALLOC_USAGE_HW_FB;
  streamParams.cb = [&](StreamBuffer buffer) { StreamCbAwbLock(buffer); };

  previewStreamId = device_client_->CreateStream(streamParams);
  ASSERT_GE(previewStreamId, 0);
  previewRequest.streamIds.add(previewStreamId);

  ret = device_client_->EndConfigure();
  ASSERT_EQ(0, ret);

  awb_lock_ = false;

  ret = device_client_->CreateDefaultRequest(CAMERA3_TEMPLATE_PREVIEW,
                                             &previewRequest.metadata);
  ASSERT_EQ(0, ret);

  ret = device_client_->SubmitRequest(previewRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  previewRequestId = ret;

  sleep(5);

  uint8_t awbLock = ANDROID_CONTROL_AWB_LOCK_ON;
  ret = previewRequest.metadata.update(ANDROID_CONTROL_AWB_LOCK,
                                       &awbLock, 1);
  ASSERT_EQ(0, ret);
  awb_lock_ = true;
  printf("%s: Awb Lock: %d\n", __func__, awbLock);


  ret = device_client_->SubmitRequest(previewRequest, true, &lastFrameNumber);
  ASSERT_GE(ret, 0);
  previewRequestId = ret;

  // Run preview for some time
  sleep(5);

  ret = device_client_->CancelRequest(previewRequestId, &lastFrameNumber);
  ASSERT_EQ(0, ret);

  printf("%s: Preview request cancelled last frame number: %" PRId64 "\n",
         __func__, lastFrameNumber);

  awb_lock_ = false;
  ret = device_client_->WaitUntilIdle();
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here
