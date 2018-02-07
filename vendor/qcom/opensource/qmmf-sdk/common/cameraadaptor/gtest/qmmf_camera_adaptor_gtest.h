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
#include <functional>
#include <gtest/gtest.h>
#include <qmmf_camera3_device_client.h>

namespace qmmf {

namespace cameraadaptor {

using namespace android;

typedef std::function<uint64_t(uint8_t *mappedBuffer, uint32_t width,
                               uint32_t height, uint32_t stride)> CalcSize;

class Camera3Gtest : public ::testing::Test {
 protected:
  Camera3Gtest();
  ~Camera3Gtest() override;

  void SetUp() override;

  void TearDown() override;

  void StreamCb(StreamBuffer buffer);
  void StreamCbSignalOnFrame(StreamBuffer buffer);
  void StreamCbAvgFPS(StreamBuffer buffer);
  void SnapshotCb(StreamBuffer buffer);
  void Raw16Cb(StreamBuffer buffer);
  void StreamCbDumpNVXX(StreamBuffer buffer);
  void StreamCbAecLock(StreamBuffer buffer);
  void StreamCbAwbLock(StreamBuffer buffer);
  int32_t StoreBuffer(String8 extension, uint64_t &idx, StreamBuffer &buffer,
                      CalcSize &calcSize);

  void InputCb(StreamBuffer buffer);
  void GetInputBuffer(StreamBuffer &buffer);
  void ReturnInputBuffer(StreamBuffer &buffer);

  void ErrorCb(CameraErrorCode errorCode, const CaptureResultExtras &);
  void IdleCb();
  void ShutterCb(const CaptureResultExtras &, int64_t);
  void PreparedCb(int);
  void ResultCb(const CaptureResult &result);

  uint64_t GetJpegSize(uint8_t *blobBuffer, uint32_t width);
  bool IsInputSupported();
  int32_t GetMaxYUVSize(int32_t &width, int32_t &height);
  int32_t GetMaxRAWSize(int32_t &width, int32_t &height);

  int32_t StartSreaming(int32_t usage, uint32_t width, uint32_t height,
                        int templateId, int32_t &streamId, int32_t &requestId);
  int32_t StopDeleteStream(int32_t streamId, int32_t requestId);

  uint32_t camera_idx_;
  uint32_t number_of_cameras_;
  CameraClientCallbacks client_cb_;
  sp<Camera3DeviceClient> device_client_;
  bool camera_error_;

  struct timeval fps_old_ts_;
  uint64_t fps_count_;
  float avg_fps_;

  bool dump_yuv_;
  uint64_t yuv_idx_, raw_idx_, jpeg_idx_;

  pthread_mutex_t prepare_lock_;
  pthread_cond_t prepare_cond_;
  bool prepare_flag_;

  pthread_mutex_t input_lock_;
  pthread_cond_t input_cond_;
  bool input_buffer_flag_;
  StreamBuffer input_buffer_;
  int32_t input_stream_id_;
  int64_t input_last_frame_number_;

  pthread_mutex_t meta_lock_;
  pthread_cond_t meta_cond_;
  CameraMetadata last_meta_;
  bool cache_last_meta_;

  pthread_mutex_t reprocess_lock_;
  pthread_cond_t reprocess_cond_;
  bool reprocess_flag_;

  bool aec_lock_;
  bool awb_lock_;

  static const int64_t PREPARE_TIMEOUT = 1e9;  // 1 sec.
  static const int64_t PROCESS_TIMEOUT = 10e9;  // 10 sec.
};

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here

#endif
