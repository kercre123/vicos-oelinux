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
#include <libgralloc/gralloc_priv.h>
#include "qmmf_dual_camera_adaptor_gtest.h"

#define BUFFER_COUNT 4

namespace qmmf {

namespace cameraadaptor {

DualCamera3Gtest::DualCamera3Gtest()
    : number_of_cameras_(0), camera_error_(false) {}

void DualCamera3Gtest::SetUp() {
  memset(&client_cb_, 0, sizeof(client_cb_));
  client_cb_.errorCb = [&](
      CameraErrorCode errorCode,
      const CaptureResultExtras &extras) { ErrorCb(errorCode, extras); };
  client_cb_.idleCb = [&]() { IdleCb(); };
  client_cb_.peparedCb = [&](int id) { PreparedCb(id); };
  client_cb_.shutterCb = [&](const CaptureResultExtras &extras,
                            int64_t ts) { ShutterCb(extras, ts); };
  client_cb_.resultCb = [&](const CaptureResult &result) { ResultCb(result); };

  device_client_= new Camera3DeviceClient(client_cb_);
  ASSERT_TRUE(NULL != device_client_.get());

  auto ret = device_client_->Initialize();
  ASSERT_EQ(0, ret);

  number_of_cameras_ = device_client_->GetNumberOfCameras();
  ASSERT_GE(number_of_cameras_, 2U);
  memset(&ctx1_, 0, sizeof(ctx1_));
  memset(&ctx2_, 0, sizeof(ctx2_));
  memset(&ctx3_, 0, sizeof(ctx3_));
  ctx1_.cameraIdx = 0;
  ctx2_.cameraIdx = 1;
  ctx3_.cameraIdx = 2;
}

void DualCamera3Gtest::StreamCb(StreamBuffer buffer) {
  String8 path;
  alloc_device_t *grallocDevice = device_client_->GetGrallocDevice();
  gralloc_module_t const *mapper = reinterpret_cast<gralloc_module_t const *>(
        grallocDevice->common.module);

  printf("%s: E streamId: %d buffer: %p size %d ts: %" PRId64 "\n", __func__,
         buffer.stream_id, buffer.handle, buffer.size, buffer.timestamp);

  if (!(buffer.frame_number % 10)) {
    if (buffer.info.format < BufferFormat::kBLOB ) {
       path.appendFormat("/data/misc/qmmf/frame_%d_dim_%dx%d.yuv", buffer.frame_number,
          buffer.info.plane_info[0].width, buffer.info.plane_info[0].height);
    } else if (buffer.info.format > BufferFormat::kBLOB ) {
       path.appendFormat("/data/misc/qmmf/frame_%d_dim_%dx%d.raw", buffer.frame_number,
          buffer.info.plane_info[0].width, buffer.info.plane_info[0].height);
    }
    FILE *f = fopen(path.string(), "w+");
    uint8_t *mappedBuffer = NULL;
    auto ret = mapper->lock(mapper, buffer.handle, GRALLOC_USAGE_SW_READ_OFTEN, 0,
                      0, buffer.info.plane_info[0].width,
                      buffer.info.plane_info[0].height,
                      (void **)&mappedBuffer);
    if ((0 != ret) || (NULL == mappedBuffer)) {
       printf("%s: Unable to map gralloc buffer: %p res: %d\n", __func__,
             mappedBuffer, ret);
    }
    uint64_t size = buffer.size;
    if (size != fwrite(mappedBuffer, sizeof(uint8_t), size, f)) {
       ret = ferror(f);
       printf("%s: Bad Write error (%d) %s\n", __func__, -ret, strerror(ret));
    }
  }

}

void DualCamera3Gtest::ErrorCb(CameraErrorCode errorCode,
                               const CaptureResultExtras &) {
  printf("%s: ErrorCode: %d\n", __func__, errorCode);
  if (ERROR_CAMERA_SERVICE >= errorCode) {
    camera_error_ = true;  // Unrecoverable error
  }
}

void DualCamera3Gtest::IdleCb() {
  printf("%s: Idle state notification\n", __func__);
}

void DualCamera3Gtest::ShutterCb(const CaptureResultExtras &, int64_t) {}

void DualCamera3Gtest::PreparedCb(int) {}

void DualCamera3Gtest::ResultCb(const CaptureResult &result) {
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
}

int32_t DualCamera3Gtest::StartStreaming(CameraContext &ctx, uint32_t width,
                                         uint32_t height, uint32_t format) {
  CameraStreamParameters streamParams;
  CameraClientCallbacks clientCb;
  Camera3Request request;
  int64_t lastFrameNumber;

  uint32_t numberOfCameras = 0;

  memset(&clientCb, 0, sizeof(clientCb));
  clientCb.errorCb = [&](
      CameraErrorCode errorCode,
      const CaptureResultExtras &extras) { ErrorCb(errorCode, extras); };
  clientCb.idleCb = [&]() { IdleCb(); };
  clientCb.peparedCb = [&](int id) { PreparedCb(id); };
  clientCb.shutterCb = [&](const CaptureResultExtras &extras,
                           int64_t ts) { ShutterCb(extras, ts); };
  clientCb.resultCb = [&](const CaptureResult &result) { ResultCb(result); };
  ctx.device = new Camera3DeviceClient(clientCb);
  if (NULL == ctx.device.get()) {
    return -ENOMEM;
  }
  auto ret = ctx.device->Initialize();
  if (0 != ret) {
    printf("%s: Unable to initialize camera client: %d\n", __func__, ret);
    goto exit;
  }

  numberOfCameras = ctx.device->GetNumberOfCameras();
  if (ctx.cameraIdx < numberOfCameras) {
    ret = ctx.device->OpenCamera(ctx.cameraIdx);
    if (0 != ret) {
      printf("%s: Unable to open camera: %d %s", __func__, ret, strerror(-ret));
      goto exit;
    }

    ret = ctx.device->BeginConfigure();
    if (0 != ret) {
      printf("%s: Unable to start camera configuration: %d %s", __func__, ret,
             strerror(-ret));
      goto exit;
    }

    memset(&streamParams, 0, sizeof(streamParams));
    streamParams.bufferCount = BUFFER_COUNT;
    streamParams.format = format;
    streamParams.width = width;
    streamParams.height = height;
    streamParams.grallocFlags = GRALLOC_USAGE_HW_FB;

    streamParams.cb = [&](StreamBuffer buffer) {
      printf("%s: Received buffer from camera Id: %d\n", __func__,
             ctx.cameraIdx);
      StreamCb(buffer);
      ctx.device->ReturnStreamBuffer(buffer);
    };
    ret = ctx.device->CreateStream(streamParams);
    if (0 > ret) {
      printf("%s: Unable to create camera stream: %d %s", __func__, ret,
             strerror(-ret));
      goto exit;
    }
    request.streamIds.add(ret);
    /* during multi camera we need to use Raw only mode to avoid
     * allocating unneeded resources */
    bool isRawOnly = (format == HAL_PIXEL_FORMAT_RAW10);
    ret = ctx.device->EndConfigure(false, isRawOnly);
    if (0 != ret) {
      printf("%s: Unable to complete camera configuration: %d %s\n", __func__,
             ret, strerror(-ret));
      goto exit;
    }

    ret = ctx.device->CreateDefaultRequest(CAMERA3_TEMPLATE_PREVIEW,
                                           &request.metadata);
    if (0 != ret) {
      printf("%s: Unable to create default camera request: %d %s", __func__,
             ret, strerror(-ret));
      goto exit;
    }

    ret = ctx.device->SubmitRequest(request, true, &lastFrameNumber);
    if (0 > ret) {
      printf("%s: Unable to submit camera request: %d\n", __func__, ret);
      goto exit;
    }
    ctx.requestId = ret;
  } else {
    printf("%s: Number of cameras detected:%d camera idx:%d\n", __func__,
           numberOfCameras, ctx.cameraIdx);
    ret = -EINVAL;
    goto exit;
  }

  return ret;

exit:

  if (NULL != ctx.device.get()) {
    ctx.device.clear();
    ctx.device = NULL;
  }

  return ret;
}

int32_t DualCamera3Gtest::StopStreamingAndClose(CameraContext &ctx) {
  int64_t lastFrameNumber;

  if (NULL == ctx.device.get()) {
    return -EINVAL;
  }

  auto ret = ctx.device->CancelRequest(ctx.requestId, &lastFrameNumber);
  if (0 > ret) {
    printf("%s: Unable to cancel camera request: %d\n", __func__, ret);
    goto exit;
  }

  printf("%s: Request cancelled last frame number: %" PRId64 "\n", __func__,
         lastFrameNumber);

  ret = ctx.device->WaitUntilIdle();
  if (0 > ret) {
    printf("%s: Wait until Idle failed: %d\n", __func__, ret);
    goto exit;
  }

exit:

  ctx.device.clear();
  ctx.device = NULL;

  return ret;
}

TEST_F(DualCamera3Gtest, DualPreview1080p) {
  auto ret = StartStreaming(ctx2_, 1920, 1080, HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED);
  ASSERT_EQ(0, ret);

  ret = StartStreaming(ctx1_, 1920, 1080, HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED);
  ASSERT_EQ(0, ret);

  // Let streaming run for a while
  sleep(5);

  ret = StopStreamingAndClose(ctx2_);
  ASSERT_EQ(0, ret);

  ret = StopStreamingAndClose(ctx1_);
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(DualCamera3Gtest, DualPreviewVGA) {
  auto ret = StartStreaming(ctx2_, 640, 480, HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED);
  ASSERT_EQ(0, ret);

  ret = StartStreaming(ctx1_, 640, 480, HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED);
  ASSERT_EQ(0, ret);

  // Let streaming run for a while
  sleep(5);

  ret = StopStreamingAndClose(ctx1_);
  ASSERT_EQ(0, ret);

  ret = StopStreamingAndClose(ctx2_);
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(DualCamera3Gtest, ThreeCamerasPreviewVGA) {
  auto ret = StartStreaming(ctx2_, 640, 480, HAL_PIXEL_FORMAT_RAW10);
  ASSERT_EQ(0, ret);

  sleep(5);

  ret = StartStreaming(ctx1_, 640, 480, HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED);
  ASSERT_EQ(0, ret);

  sleep(5);

  ret = StartStreaming(ctx3_, 1280, 480, HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED);
  ASSERT_EQ(0, ret);

  // Let streaming run for a while
  sleep(10);

  ret = StopStreamingAndClose(ctx2_);
  ASSERT_EQ(0, ret);
  sleep(2);
  ret = StopStreamingAndClose(ctx1_);
  ASSERT_EQ(0, ret);
  sleep(2);
  ret = StopStreamingAndClose(ctx3_);
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

TEST_F(DualCamera3Gtest, StereoPreviewVGA) {

  CameraContext StereoCtx;
  memset(&StereoCtx, 0, sizeof(StereoCtx));
  StereoCtx.cameraIdx = 2;

  auto ret = StartStreaming(StereoCtx, 1280, 480, HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED);
  ASSERT_EQ(0, ret);

  // Let streaming run for a while
  sleep(5);

  ret = StopStreamingAndClose(StereoCtx);
  ASSERT_EQ(0, ret);

  ASSERT_FALSE(camera_error_);
}

TEST_F(DualCamera3Gtest, DualCamerasPreviewVGA) {
  auto ret = StartStreaming(ctx2_, 640, 480, HAL_PIXEL_FORMAT_RAW10);
  ASSERT_EQ(0, ret);

  sleep(5);

  ret = StartStreaming(ctx3_, 1280, 480, HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED);
  ASSERT_EQ(0, ret);

  // Let streaming run for a while
  sleep(100);

  ret = StopStreamingAndClose(ctx2_);
  ASSERT_EQ(0, ret);
  ret = StopStreamingAndClose(ctx3_);
  ASSERT_EQ(0, ret);
  ASSERT_FALSE(camera_error_);
}

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here
