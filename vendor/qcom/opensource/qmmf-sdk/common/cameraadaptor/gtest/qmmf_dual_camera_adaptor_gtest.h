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

#ifndef CAMERA3GTEST_H_
#define CAMERA3GTEST_H_

#include <fcntl.h>
#include <dirent.h>
#include <gtest/gtest.h>

#include <qmmf_camera3_device_client.h>

namespace qmmf {

namespace cameraadaptor {

class DualCamera3Gtest : public ::testing::Test {
 protected:
  DualCamera3Gtest();
  virtual ~DualCamera3Gtest() {};

  virtual void SetUp();

  void StreamCb(StreamBuffer buffer);

  void ErrorCb(CameraErrorCode errorCode, const CaptureResultExtras &);
  void IdleCb();
  void ShutterCb(const CaptureResultExtras &, int64_t);
  void PreparedCb(int);
  void ResultCb(const CaptureResult &result);

  typedef struct CameraContext_t {
    sp<Camera3DeviceClient> device;
    int32_t requestId;
    uint32_t cameraIdx;
  } CameraContext;

  int32_t StartStreaming(CameraContext &ctx, uint32_t width, uint32_t height, uint32_t format);
  int32_t StopStreamingAndClose(CameraContext &ctx);

  uint32_t number_of_cameras_;
  CameraClientCallbacks client_cb_;
  sp<Camera3DeviceClient> device_client_;
  CameraContext ctx1_, ctx2_, ctx3_;
  bool camera_error_;
};

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here

#endif
