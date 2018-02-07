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

#define TAG "Recorder360GTest"

#include <utils/Log.h>
#include <utils/String8.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <math.h>
#include <assert.h>
#include <camera/CameraMetadata.h>
#include <system/graphics.h>
#include <QCamera3VendorTags.h>

#include <qmmf-sdk/qmmf_queue.h>
#include <qmmf-sdk/qmmf_recorder_extra_param.h>
#include <qmmf-sdk/qmmf_recorder_extra_param_tags.h>
#include "recorder/test/gtest/qmmf_recorder_360cam_gtest.h"

//#define DEBUG
#define TEST_INFO(fmt, args...)  ALOGD(fmt, ##args)
#define TEST_ERROR(fmt, args...) ALOGE(fmt, ##args)
#define TEST_WARN(fmt, args...) ALOGW(fmt, ##args)
#ifdef DEBUG
#define TEST_DBG  TEST_INFO
#else
#define TEST_DBG(...) ((void)0)
#endif

static const int32_t kDelayAfterSnapshot = 5;
static const uint32_t kZslWidth          = 1920;
static const uint32_t kZslHeight         = 960;
static const uint32_t kZslQDepth         = 10;

using namespace qcamera;

void Recorder360Gtest::SetUp() {

  TEST_INFO("%s:%s Enter ", TAG, __func__);

  test_info_ = ::testing::UnitTest::GetInstance()->current_test_info();

  recorder_status_cb_.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
      { RecorderCallbackHandler(event_type, event_data, event_data_size); };

  char prop_val[PROPERTY_VALUE_MAX];
  property_get(PROP_DUMP_360_BITSTREAM, prop_val, "0");
  if (atoi(prop_val) == 0) {
    dump_bitstream_.Enable(false);
  } else {
    dump_bitstream_.Enable(true);
  }
  property_get(PROP_DUMP_360_JPEG, prop_val, "0");
  is_dump_jpeg_enabled_ = (atoi(prop_val) == 0) ? false : true;
  property_get(PROP_DUMP_360_RAW, prop_val, "0");
  is_dump_raw_enabled_ = (atoi(prop_val) == 0) ? false : true;
  property_get(PROP_DUMP_360_YUV_FRAMES, prop_val, "0");
  is_dump_yuv_enabled_ = (atoi(prop_val) == 0) ? false : true;
  property_get(PROP_DUMP_360_YUV_FREQ, prop_val, DEFAULT_360_YUV_DUMP_FREQ);
  dump_yuv_freq_ = atoi(prop_val);
  property_get(PROP_360_N_ITERATIONS, prop_val, DEFAULT_360_ITERATIONS_COUNT);
  iteration_count_ = atoi(prop_val);
  property_get(PROP_360_RECORD_DURATION, prop_val, DEFAULT_360_RECORD_DURATION);
  record_duration_ = atoi(prop_val);

  camera_ids_.push_back(0);
  camera_ids_.push_back(1);

  multicam_id_ = 0;
  multicam_type_ = MultiCameraConfigType::k360Stitch;

  memset(&multicam_start_params_, 0x0, sizeof multicam_start_params_);
  multicam_start_params_.zsl_mode         = false;
  multicam_start_params_.zsl_queue_depth  = 10;
  multicam_start_params_.zsl_width        = kZslWidth;
  multicam_start_params_.zsl_height       = kZslHeight;
  multicam_start_params_.frame_rate       = kZslQDepth;
  multicam_start_params_.flags            = 0x0;

  TEST_INFO("%s:%s Exit ", TAG, __func__);
}

void Recorder360Gtest::TearDown() {

  TEST_INFO("%s:%s Enter ", TAG, __func__);
  TEST_INFO("%s:%s Exit ", TAG, __func__);
}

int32_t Recorder360Gtest::Init() {

  auto ret = recorder_.Connect(recorder_status_cb_);
  assert(ret == NO_ERROR);
  return ret;
}

int32_t Recorder360Gtest::DeInit() {

  auto ret = recorder_.Disconnect();
  assert(ret == NO_ERROR);
  return ret;
}

/*
* CreateDeleteSession: This case will test Create & Delete Session Api using
*                      MultiCamnera instance.
* Api test sequence:
*   loop Start {
*   ------------------
*   - CreateMultiCamera
*   - ConfigureMultiCamera
*   - StartCamera
*   - CreateSession
*   - DeleteSession
*   - StopCamera
*   ------------------
*   } loop End
*/
TEST_F(Recorder360Gtest, CreateDeleteSession) {

  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);
  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    sleep(2);
    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);
  }
  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched6KSnapshot: This case will test a MultiCamera capture for stitched
*                     6K JPEG snapshot.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CaptureImage - JPEG
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched6KSnapshot) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  ImageParam image_param;
  memset(&image_param, 0x0, sizeof image_param);
  image_param.width         = 6080;
  image_param.height        = 3040;
  image_param.image_format  = ImageFormat::kJPEG;
  image_param.image_quality = 95;

  std::vector<CameraMetadata> meta_array;
  CameraMetadata meta;

  ret = recorder_.GetDefaultCaptureParam(multicam_id_, meta);
  assert(ret == NO_ERROR);

  meta_array.push_back(meta);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    ImageCaptureCb cb = [this] (uint32_t camera_id, uint32_t image_count,
                                BufferDescriptor buffer,
                                MetaData meta_data) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta_data); };

    ret = recorder_.CaptureImage(multicam_id_, image_param, 1, meta_array, cb);
    assert(ret == NO_ERROR);
    // Take snapshot after every 5 sec.
    sleep(kDelayAfterSnapshot);
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* Stitched4KSnapshot: This case will test a MultiCamera capture for stitched
*                     4K JPEG snapshot.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CaptureImage - JPEG
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KSnapshot) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  ImageParam image_param;
  memset(&image_param, 0x0, sizeof image_param);
  image_param.width         = 3840;
  image_param.height        = 1920;
  image_param.image_format  = ImageFormat::kJPEG;
  image_param.image_quality = 95;

  std::vector<CameraMetadata> meta_array;
  CameraMetadata meta;

  ret = recorder_.GetDefaultCaptureParam(multicam_id_, meta);
  assert(ret == NO_ERROR);

  meta_array.push_back(meta);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    ImageCaptureCb cb = [this] (uint32_t camera_id, uint32_t image_count,
                                BufferDescriptor buffer,
                                MetaData meta_data) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta_data); };

    ret = recorder_.CaptureImage(multicam_id_, image_param, 1, meta_array, cb);
    assert(ret == NO_ERROR);
    // Take snapshot after every 5 sec.
    sleep(kDelayAfterSnapshot);
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* StitchedHDSnapshot: This case will test a MultiCamera capture for stitched
*                     Full HD JPEG snapshot.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CaptureImage - JPEG
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, StitchedHDSnapshot) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  ImageParam image_param;
  memset(&image_param, 0x0, sizeof image_param);
  image_param.width         = 1920;
  image_param.height        = 960;
  image_param.image_format  = ImageFormat::kJPEG;
  image_param.image_quality = 95;

  std::vector<CameraMetadata> meta_array;
  CameraMetadata meta;

  ret = recorder_.GetDefaultCaptureParam(multicam_id_, meta);
  assert(ret == NO_ERROR);

  meta_array.push_back(meta);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    ImageCaptureCb cb = [this] (uint32_t camera_id, uint32_t image_count,
                                BufferDescriptor buffer,
                                MetaData meta_data) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta_data); };

    ret = recorder_.CaptureImage(multicam_id_, image_param, 1, meta_array, cb);
    assert(ret == NO_ERROR);
    // Take snapshot after every 5 sec.
    sleep(kDelayAfterSnapshot);
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* Stitched720pSnapshot: This case will test a MultiCamera capture for stitched
*                       720p JPEG snapshot.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CaptureImage - JPEG
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched720pSnapshot) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  ImageParam image_param;
  memset(&image_param, 0x0, sizeof image_param);
  image_param.width         = 1440;
  image_param.height        = 720;
  image_param.image_format  = ImageFormat::kJPEG;
  image_param.image_quality = 95;

  std::vector<CameraMetadata> meta_array;
  CameraMetadata meta;

  ret = recorder_.GetDefaultCaptureParam(multicam_id_, meta);
  assert(ret == NO_ERROR);

  meta_array.push_back(meta);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    ImageCaptureCb cb = [this] (uint32_t camera_id, uint32_t image_count,
                                BufferDescriptor buffer,
                                MetaData meta_data) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta_data); };

    ret = recorder_.CaptureImage(multicam_id_, image_param, 1, meta_array, cb);
    assert(ret == NO_ERROR);
    // Take snapshot after every 5 sec.
    sleep(kDelayAfterSnapshot);
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* Stitched4KYUVTrack: This case will test a MultiCamera session with one
*                     4K YUV track and configured to produce stitched
*                     frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KYUVTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                            3840, /* Width */
                                            1920, /* Height */
                                            30 /* FPS */};
    uint32_t video_track_id = 1;

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for time record_duration_, during this time buffer with
    // valid data would be received in track callback (VideoTrackYUVDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* StitchedHDYUVTrack: This case will test a MultiCamera session with one
*                     Full HD YUV track and configured to produce stitched
*                     frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, StitchedHDYUVTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                            1920, /* Width */
                                            960,  /* Height */
                                            30 /* FPS */};
    uint32_t video_track_id = 1;

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for time record_duration_, during this time buffer with
    // valid data would be received in track callback (VideoTrackYUVDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* Stitched720pYUVTrack: This case will test a MultiCamera session with one
*                       720p YUV track and configured to produce stitched
*                       frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched720pYUVTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                            1440, /* Width */
                                            720,  /* Height */
                                            30 /* FPS */};
    uint32_t video_track_id = 1;
    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for time record_duration_, during this time buffer with
    // valid data would be received in track callback (VideoTrackYUVDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* Stitched4KAndFullHDYUVTrack: This case will test a MultiCamera session with
*                              one 4K and one Full HD YUV tracks, configured
*                              to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   ------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   ------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KAndFullHDYUVTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);
  uint32_t track_4k_id = 1;
  uint32_t track_fullhd_id = 2;

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                            3840, /* Width */
                                            1920, /* Height */
                                            30 /* FPS */};

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, track_4k_id,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    video_track_param.width  = 1920;
    video_track_param.height = 960;

    ret = recorder_.CreateVideoTrack(session_id, track_fullhd_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for time record_duration_, during this time buffer with
    // valid data would be received in track callback (VideoTrackYUVDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, track_4k_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, track_fullhd_id);
    assert(ret == NO_ERROR);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* Stitched4KEncTrack: This case will test a MultiCamera session with 3840x1920
*                     h264 encodded track and configured to produce stitched
*                     frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30 /* FPS */};
     // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for record_duration_, during this time buffer with valid
    // data would be received in track callback (VideoTrackDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }
  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* StitchedHDEncTrack: This case will test a MultiCamera session with 1920x960
*                     h264 encodded track and configured to produce stitched
*                     frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, StitchedHDEncTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 1920;
  uint32_t stream_height = 960;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30 /* FPS */};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel4;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for record_duration_, during this time buffer with valid
    // data would be received in track callback (VideoTrackDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }
  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* Stitched720pEncTrack: This case will test a MultiCamera session with 1440x720
*                       h264 encodded track and configured to produce stitched
*                       frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched720pEncTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 1440;
  uint32_t stream_height = 720;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);

    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30 /* FPS */};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel4;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for record_duration_, during this time buffer with valid
    // data would be received in track callback (VideoTrackDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }
  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* Stitched720p120fpsEncTrack: This case will test a MultiCamera session with
*                       1440x720 h264 encoded track at 120 fps and configured
*                       to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched720p120fpsEncTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 1440;
  uint32_t stream_height = 720;
  float fps = 120;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);

    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            fps /* FPS */};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel4;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for record_duration_, during this time buffer with valid
    // data would be received in track callback (VideoTrackDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }
  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* Stitched4KAnd720pEncTrack: This case will test a MultiCamera session with one
*                            3840x1920 and one 1440x720 h264 encoded tracks,
*                            configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KAnd720pEncTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;
  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  uint32_t video_track_id_720p = 2;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb =
      [this] (EventType event_type, void *event_data,
              size_t event_data_size) -> void { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);
  VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                          3840,
                                          1920,
                                          stream_fps /* FPS */};
  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    // Set parameters for and create 3840x1920 h264 encodded track.
    stream_width  = 3840;
    stream_height = 1920;
    video_track_param.width       = stream_width;
    video_track_param.height      = stream_height;
      // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id_4k,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb =
        [this] (uint32_t track_id, EventType event_type,
                void *event_data, size_t event_data_size) -> void
        { VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    // Set parameters for and create 1440x720 h264 encodded track.
    stream_width  = 1440;
    stream_height = 720;
    video_track_param.width       = stream_width;
    video_track_param.height      = stream_height;

    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel4;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id_720p,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb =
        [this] (uint32_t track_id, EventType event_type,
                void *event_data, size_t event_data_size) -> void
        { VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_720p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_720p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());

}

/*
* Stitched4KAnd480pEncTrack: This case will test a MultiCamera session with one
*                            3840x1920 and one 960x480 h264 encoded tracks,
*                            configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KAnd480pEncTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width = 3840;
  uint32_t stream_height = 1920;
  float fps = 30;
  uint32_t video_track_id_4k = 1;
  uint32_t video_track_id_480p = 2;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb =
      [this] (EventType event_type, void *event_data,
              size_t event_data_size) -> void { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);
  VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                          stream_width,
                                          stream_height,
                                          fps /* FPS */};
  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    // Set parameters for and create 3840x1920 h264 encodded track.
    stream_width  = 3840;
    stream_height = 1920;
    video_track_param.width       = stream_width;
    video_track_param.height      = stream_height;
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id_4k,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb =
        [this] (uint32_t track_id, EventType event_type,
                void *event_data, size_t event_data_size) -> void
        { VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    // Set parameters for and create 960x480 h264 encodded track.
    stream_width  = 960;
    stream_height = 480;
    video_track_param.width       = stream_width;
    video_track_param.height      = stream_height;
       // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel4;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id_480p,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb =
        [this] (uint32_t track_id, EventType event_type,
                void *event_data, size_t event_data_size) -> void
        { VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* StitchedHDAnd480pEncTrack: This case will test a MultiCamera session with one
*                            1920x960 and one 960x480 h264 encoded tracks,
*                            configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, StitchedHDAnd480pEncTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width = 1920;
  uint32_t stream_height = 960;
  float fps = 30;
  uint32_t video_track_id_HD = 1;
  uint32_t video_track_id_480p = 2;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb =
      [this] (EventType event_type, void *event_data,
              size_t event_data_size) -> void { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);
  VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                          stream_width,
                                          stream_height,
                                          fps };
  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    // Set parameters for and create 1920x960 h264 encodded track.
    stream_width  = 1920;
    stream_height = 960;
    video_track_param.width       = stream_width;
    video_track_param.height      = stream_height;
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel4;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id_HD,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb =
        [this] (uint32_t track_id, EventType event_type,
                void *event_data, size_t event_data_size) -> void
        { VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_HD,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    // Set parameters for and create 960x480 h264 encodded track.
    stream_width  = 960;
    stream_height = 480;
    video_track_param.width       = stream_width;
    video_track_param.height      = stream_height;
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel4;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id_480p,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb =
        [this] (uint32_t track_id, EventType event_type,
                void *event_data, size_t event_data_size) -> void
        { VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_HD);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* Stitched4KEncTrackWithTNR: This case will test a MultiCamera
*                            session with 3840x1920 stitched h264 encoded track
*                            with SW TNR enabled.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack
*   - StartSession
*   - Enable TNR
*   - StopSession
*   - DeleteVideoTrack
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithTNR) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width = 3840;
  uint32_t stream_height = 1920;
  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb =
      [this] (EventType event_type, void *event_data,
              size_t event_data_size) -> void { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);
  VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                          stream_width,
                                          stream_height,
                                          stream_fps };
  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    // Set parameters for and create 3840x1920 h264 encoded track.
    video_track_param.low_power_mode = false;
    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate    = 4000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id_4k,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb =
        [this] (uint32_t track_id, EventType event_type,
                void *event_data, size_t event_data_size) -> void
        { VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithTNRAnd1080pYUVTrack: This case will test a MultiCamera
*                            session with one 3840x1920 h264 encoded and one
*                            2160x1080 YUV track with SW TNR enabled and both
*                            tracks configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithTNRAnd1080pYUVTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width = 3840;
  uint32_t stream_height = 1920;
  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  uint32_t video_track_id_1080p = 2;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb =
      [this] (EventType event_type, void *event_data,
              size_t event_data_size) -> void { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kAVC,
                                            stream_width,
                                            stream_height,
                                            stream_fps };
    // Set parameters for and create 3840x1920 h264 encoded track.
    stream_width  = 3840;
    stream_height = 1920;
    format_type = VideoFormat::kAVC;
    video_track_param.width       = stream_width;
    video_track_param.height      = stream_height;
    video_track_param.low_power_mode = false;
    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate    = 4000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id_4k,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb =
        [this] (uint32_t track_id, EventType event_type,
                void *event_data, size_t event_data_size) -> void
        { VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    // Set parameters for and create 2160x1080 YUV track.
    stream_width  = 2160;
    stream_height = 1080;
    format_type = VideoFormat::kYUV;
    video_track_param.width       = stream_width;
    video_track_param.height      = stream_height;
    video_track_param.frame_rate  = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = true;

    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_1080p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_1080p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());

}

/*
* Stitched4KEncTrackWithTNRAnd480pYUVTrack: This case will test a MultiCamera
*                            session with one 3840x1920 h264 encoded with TNR
*                            enabled; and one 960x480 YUV track and both tracks
*                            configured to produce stitched frames. TNR will be
*                            applied only on 4k stream.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithTNRAnd480pYUVTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;
  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  uint32_t video_track_id_1080p = 2;
  VideoFormat format_type = VideoFormat::kAVC;
  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb =
      [this] (EventType event_type, void *event_data,
              size_t event_data_size) -> void { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kAVC,
                                            3840,
                                            1920,
                                            stream_fps};
    // Set parameters for and create 3840x1920 h264 encoded track.
    stream_width  = 3840;
    stream_height = 1920;
    format_type = VideoFormat::kAVC;
    video_track_param.width       = stream_width;
    video_track_param.height      = stream_height;
    video_track_param.frame_rate  = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = false;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate    = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id_4k,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb =
        [this] (uint32_t track_id, EventType event_type,
                void *event_data, size_t event_data_size) -> void
        { VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    // Set parameters for and create 960x480 YUV track.
    stream_width  = 960;
    stream_height = 480;
    format_type = VideoFormat::kYUV;
    video_track_param.width       = stream_width;
    video_track_param.height      = stream_height;
    video_track_param.frame_rate  = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = true;

    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_1080p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_1080p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());

}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRAnd480pYUVTrack: This case will test a
*                            MultiCamera session with one 3840x1920 h264 encoded
*                            with Source Surface Downscaling with SW TNR enabled.
*                            Also one 960x480 YUV track ; both tracks configured
*                            to produce stitched frames. TNR will only be applied
*                            on the 4k stream.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRAnd480pYUVTrack) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;
  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  uint32_t video_track_id_480p = 2;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this](EventType event_type, void *event_data,
                                      size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
              test_info_->name(), i);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kAVC,
                                            3840,
                                            1920,
                                            stream_fps};
    // Set parameters for and create 3840x1920 h264 encoded track.
    stream_width = 3840;
    stream_height = 1920;
    format_type = VideoFormat::kAVC;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = false;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {video_track_param.format_type,
                                    video_track_id_4k, stream_width,
                                    stream_height};
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [this](uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) -> void {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     video_track_param, extra_param,
                                     video_track_cb);

    assert(ret == NO_ERROR);

    stream_width = 960;
    stream_height = 480;
    format_type = VideoFormat::kYUV;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = true;

    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);
    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();

  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}


/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRAnd960pYUVTrack: This case will test a
*                            MultiCamera session with one 3840x1920 h264 encoded
*                            with Source Surface Downscaling with SW TNR enabled.
*                            Also one 1920x960 YUV track ; both tracks configured
*                            to produce stitched frames. TNR will only be applied
*                            on the 4k stream.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRAnd960pYUVTrack) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;

  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  uint32_t video_track_id_480p = 2;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this](EventType event_type, void *event_data,
                                      size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
              test_info_->name(), i);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kAVC,
                                            3840,
                                            1920,
                                            stream_fps};
    // Set parameters for and create 3840x1920 h264 encoded track.
    stream_width = 3840;
    stream_height = 1920;
    format_type = VideoFormat::kAVC;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = false;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {video_track_param.format_type,
                                    video_track_id_4k, stream_width,
                                    stream_height};
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [this](uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) -> void {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     video_track_param, extra_param,
                                     video_track_cb);

    assert(ret == NO_ERROR);

    stream_width = 1920;
    stream_height = 960;
    format_type = VideoFormat::kYUV;

    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = true;

    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);
    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();

  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithTNRWithOverlayMix: This case will test a
*                                       MultiCamera session with
*                                       3840x1920 stitched h264
*                                       encoded track with SW TNR
*                                       enabled with ten Overlays.
*                                       Includes Privacy Mask, User
*                                       Text and Static Image Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*  - CreateVideoTrack
*  - StartSession
*  - Enable TNR
*  - CreateOverlayObjects
*  - SetOverlays
*  - RemoveOverlays
*  - DeleteOverlayObjects
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithTNRWithOverlayMix) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;

  std::vector<uint32_t> overlay_ids_;

  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this](EventType event_type, void *event_data,
                                      size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);
  VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                          3840,
                                          1920,
                                          stream_fps};
  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    // Set parameters for and create 3840x1920 h264 encoded track.
    stream_width = 3840;
    stream_height = 1920;

    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = false;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {video_track_param.format_type,
                                    video_track_id_4k, stream_width,
                                    stream_height};
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [this](uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) -> void {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    // 1. Create PrivacyMask type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kPrivacyMask;
    object_params.color = 0x4C4C4CFF; //Fill mask with color.
    // Dummy coordinates for test purpose.
    object_params.dst_rect.start_x = 2800;
    object_params.dst_rect.start_y = 900;
    object_params.dst_rect.width   = 1920/4;
    object_params.dst_rect.height  = 1080/4;

    uint32_t privacy_mask_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &privacy_mask_id_1);
    assert(ret == 0);
    ret = recorder_.SetOverlay(video_track_id_4k, privacy_mask_id_1);
    assert(ret == 0);
    overlay_ids_.push_back(privacy_mask_id_1);

    // 2. Create PrivacyMask type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kPrivacyMask;
    object_params.color = 0x4C4C4CFF; //Fill mask with color.
    // Dummy coordinates for test purpose.
    object_params.dst_rect.start_x = 1100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width   = 1920/4;
    object_params.dst_rect.height  = 1080/4;

    uint32_t privacy_mask_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &privacy_mask_id_2);
    assert(ret == 0);
    ret = recorder_.SetOverlay(video_track_id_4k, privacy_mask_id_2);
    assert(ret == 0);
    overlay_ids_.push_back(privacy_mask_id_2);

    // 3. Create PrivacyMask type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kPrivacyMask;
    object_params.color = 0x4C4C4CFF; //Fill mask with color.
    // Dummy coordinates for test purpose.
    object_params.dst_rect.start_x = 1300;
    object_params.dst_rect.start_y = 1000;
    object_params.dst_rect.width   = 1920/2;
    object_params.dst_rect.height  = 1080/2;

    uint32_t privacy_mask_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &privacy_mask_id_3);
    assert(ret == 0);
    ret = recorder_.SetOverlay(video_track_id_4k, privacy_mask_id_3);
    assert(ret == 0);
    overlay_ids_.push_back(privacy_mask_id_3);

    // 4. Create UserText type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kUserText;
    object_params.location = OverlayLocationType::kRandom;
    object_params.color = 0x660066FF; //Purple
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1000;
    object_params.dst_rect.width   = 480;
    object_params.dst_rect.height  = 60;
    std::string user_text_0("Channel Title Testing");
    user_text_0.copy(object_params.user_text, user_text_0.length());

    uint32_t user_text_id_0;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &user_text_id_0);
    assert(ret == 0);
    ret = recorder_.SetOverlay(video_track_id_4k, user_text_id_0);
    assert(ret == 0);
    overlay_ids_.push_back(user_text_id_0);

    // 5. Create UserText type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kUserText;
    object_params.location = OverlayLocationType::kRandom;
    object_params.color = 0x33CC00FF; //Green
    object_params.dst_rect.start_x = 2400;
    object_params.dst_rect.start_y = 200;
    object_params.dst_rect.width   = 480;
    object_params.dst_rect.height  = 60;
    std::string user_text_1("Location text");
    user_text_1.copy(object_params.user_text, user_text_1.length());

    uint32_t user_text_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &user_text_id_1);
    assert(ret == 0);
    ret = recorder_.SetOverlay(video_track_id_4k, user_text_id_1);
    assert(ret == 0);
    overlay_ids_.push_back(user_text_id_1);

    // 6. Create UserText type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kUserText;
    object_params.location = OverlayLocationType::kRandom;
    object_params.color = 0x33CC00FF; //Green
    object_params.dst_rect.start_x = 2400;
    object_params.dst_rect.start_y = 300;
    object_params.dst_rect.width   = 480;
    object_params.dst_rect.height  = 60;
    std::string user_text_2("Location text");
    user_text_2.copy(object_params.user_text, user_text_2.length());

    uint32_t user_text_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &user_text_id_2);
    assert(ret == 0);
    ret = recorder_.SetOverlay(video_track_id_4k, user_text_id_2);
    assert(ret == 0);
    overlay_ids_.push_back(user_text_id_2);

    // 7. Create UserText type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kUserText;
    object_params.location = OverlayLocationType::kRandom;
    object_params.color = 0x33CC00FF; //Green
    object_params.dst_rect.start_x = 2400;
    object_params.dst_rect.start_y = 400;
    object_params.dst_rect.width   = 480;
    object_params.dst_rect.height  = 60;
    std::string user_text_3("Location text");
    user_text_3.copy(object_params.user_text, user_text_3.length());

    uint32_t user_text_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &user_text_id_3);
    assert(ret == 0);
    ret = recorder_.SetOverlay(video_track_id_4k, user_text_id_3);
    assert(ret == 0);
    overlay_ids_.push_back(user_text_id_3);

    // 8. Create Static Image type overlay.
    uint32_t static_img_id;
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kBottomRight;
    std::string str("/etc/overlay_test.rgba");
    str.copy(object_params.image_info.image_location, str.length());
    object_params.dst_rect.width = 451;
    object_params.dst_rect.height = 109;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &static_img_id);
    assert(ret == 0);
    // Apply overlay object on video track.
    ret = recorder_.SetOverlay(video_track_id_4k, static_img_id);
    assert(ret == 0);
    overlay_ids_.push_back(static_img_id);

    // 9. Create UserText type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kUserText;
    object_params.location = OverlayLocationType::kRandom;
    object_params.color = 0x189BF2FF; //Light Green
    object_params.dst_rect.start_x = 200;
    object_params.dst_rect.start_y = 400;
    object_params.dst_rect.width   = 480;
    object_params.dst_rect.height  = 60;
    std::string user_text_4("Va event text");
    user_text_4.copy(object_params.user_text, user_text_4.length());

    uint32_t user_text_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &user_text_id_4);
    assert(ret == 0);
    ret = recorder_.SetOverlay(video_track_id_4k, user_text_id_4);
    assert(ret == 0);
    overlay_ids_.push_back(user_text_id_4);

    // 10. Create UserText type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kUserText;
    object_params.location = OverlayLocationType::kRandom;
    object_params.color = 0x189BF2FF; //Light Green
    object_params.dst_rect.start_x = 200;
    object_params.dst_rect.start_y = 500;
    object_params.dst_rect.width   = 480;
    object_params.dst_rect.height  = 60;
    std::string user_text_5("Va event text");
    user_text_5.copy(object_params.user_text, user_text_5.length());

    uint32_t user_text_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &user_text_id_5);
    assert(ret == 0);
    ret = recorder_.SetOverlay(video_track_id_4k, user_text_id_5);
    assert(ret == 0);
    overlay_ids_.push_back(user_text_id_5);

    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}


/*
* Stitched4KEncTrackWithTNRWithOverlayBlob: This case will test a
*                                           MultiCamera session with
*                                           3840x1920 stitched h264
*                                           encoded track with SW TNR
*                                           enabled with five Blob type
*                                           Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*  - CreateVideoTrack
*  - StartSession
*  - Enable TNR
*  - CreateOverlayObjects
*  - SetOverlays
*  - RemoveOverlays
*  - DeleteOverlayObjects
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithTNRWithOverlayBlob) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;

  std::vector<uint32_t> overlay_ids_;

  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this](EventType event_type, void *event_data,
                                      size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);
  VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                          3840,
                                          1920,
                                          stream_fps};
  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    // Set parameters for and create 3840x1920 h264 encoded track.
    stream_width = 3840;
    stream_height = 1920;
    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = false;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {video_track_param.format_type,
                                    video_track_id_4k, stream_width,
                                    stream_height};
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [this](uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) -> void {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

   char * image_buffer1;
   char * image_buffer2;
   char * image_buffer3;
   char * image_buffer4;
   char * image_buffer5;

    // 1. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width   = 1334;
    object_params.dst_rect.height  = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
        object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer =
        reinterpret_cast<char *>(malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer1 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
        object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_1);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_1);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_1);


    // 2. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 600;
    object_params.dst_rect.start_y = 600;
    object_params.dst_rect.width   = 1334;
    object_params.dst_rect.height  = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
        object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer =
        reinterpret_cast<char *>(malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer2 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
        object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_2);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_2);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_2);


    // 3. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1250;
    object_params.dst_rect.width   = 960;
    object_params.dst_rect.height  = 320;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 320;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
        object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer =
        reinterpret_cast<char *>(malloc(sizeof(char) * object_params.image_info.image_size));

    DrawOverlay(object_params.image_info.image_buffer,
        object_params.dst_rect.width, object_params.dst_rect.height);
    image_buffer3 = object_params.image_info.image_buffer;

    uint32_t usertxt_blob_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_3);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_3);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_3);

    // 4. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 3000;
    object_params.dst_rect.start_y = 1200;
    object_params.dst_rect.width   = 128;
    object_params.dst_rect.height  = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 128;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
        object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer =
        reinterpret_cast<char *>(malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer4 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
        object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_4);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_4);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_4);

    // 5. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 2200;
    object_params.dst_rect.start_y = 1600;
    object_params.dst_rect.width   = 960;
    object_params.dst_rect.height  = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
        object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer =
        reinterpret_cast<char *>(malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer5 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
        object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_5);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_5);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_5);

    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();

    free(image_buffer1);
    free(image_buffer2);
    free(image_buffer3);
    free(image_buffer4);
    free(image_buffer5);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRAndOverlayMix: This will test
*                                                    a MultiCamera session with
*                                                    3840x1920 stitched h264
*                                                    encoded track and source
*                                                    surface downscale with
*                                                    SW TNR enabled with
*                                                    seven Overlays. Includes
*                                                    Privacy Mask, User Text and
*                                                    Static Image type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*  - CreateVideoTrack
*  - StartSession
*  - Enable TNR
*  - CreateOverlayObjects
*  - SetOverlays
*  - RemoveOverlays
*  - DeleteOverlayObjects
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRAndOverlayMix) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;

  std::vector<uint32_t> overlay_ids_;

  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this](EventType event_type, void *event_data,
                                      size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kAVC,
                                            3840,
                                            1920,
                                            stream_fps};
    // Set parameters for and create 3840x1920 h264 encoded track.
    stream_width = 3840;
    stream_height = 1920;
    format_type = VideoFormat::kAVC;
    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = false;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {video_track_param.format_type,
                                    video_track_id_4k, stream_width,
                                    stream_height};
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [this](uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) -> void {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     video_track_param, extra_param,
                                     video_track_cb);

    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    // 1. Create PrivacyMask type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kPrivacyMask;
    object_params.color = 0x4C4C4CFF; //Fill mask with color.
    // Dummy coordinates for test purpose.
    object_params.dst_rect.start_x = 2800;
    object_params.dst_rect.start_y = 900;
    object_params.dst_rect.width   = 1920/4;
    object_params.dst_rect.height  = 1080/4;

    uint32_t privacy_mask_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &privacy_mask_id_1);
    assert(ret == 0);
    ret = recorder_.SetOverlay(video_track_id_4k, privacy_mask_id_1);
    assert(ret == 0);
    overlay_ids_.push_back(privacy_mask_id_1);

    // 2. Create PrivacyMask type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kPrivacyMask;
    object_params.color = 0x4C4C4CFF; //Fill mask with color.
    // Dummy coordinates for test purpose.
    object_params.dst_rect.start_x = 1100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width   = 1920/4;
    object_params.dst_rect.height  = 1080/4;

    uint32_t privacy_mask_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &privacy_mask_id_2);
    assert(ret == 0);
    ret = recorder_.SetOverlay(video_track_id_4k, privacy_mask_id_2);
    assert(ret == 0);
    overlay_ids_.push_back(privacy_mask_id_2);

    // 3. Create PrivacyMask type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kPrivacyMask;
    object_params.color = 0x4C4C4CFF; //Fill mask with color.
    // Dummy coordinates for test purpose.
    object_params.dst_rect.start_x = 1300;
    object_params.dst_rect.start_y = 1000;
    object_params.dst_rect.width   = 1920/2;
    object_params.dst_rect.height  = 1080/2;

    uint32_t privacy_mask_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &privacy_mask_id_3);
    assert(ret == 0);
    ret = recorder_.SetOverlay(video_track_id_4k, privacy_mask_id_3);
    assert(ret == 0);
    overlay_ids_.push_back(privacy_mask_id_3);

    // 4. Create UserText type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kUserText;
    object_params.location = OverlayLocationType::kRandom;
    object_params.color = 0x660066FF; //Purple
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1000;
    object_params.dst_rect.width   = 480;
    object_params.dst_rect.height  = 60;
    std::string user_text_0("Channel Title Testing");
    user_text_0.copy(object_params.user_text, user_text_0.length());

    uint32_t user_text_id_0;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &user_text_id_0);
    assert(ret == 0);
    ret = recorder_.SetOverlay(video_track_id_4k, user_text_id_0);
    assert(ret == 0);
    overlay_ids_.push_back(user_text_id_0);


    // 5. Create UserText type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kUserText;
    object_params.location = OverlayLocationType::kRandom;
    object_params.color = 0x33CC00FF; //Green
    object_params.dst_rect.start_x = 2400;
    object_params.dst_rect.start_y = 200;
    object_params.dst_rect.width   = 480;
    object_params.dst_rect.height  = 60;
    std::string user_text_1("Location text");
    user_text_1.copy(object_params.user_text, user_text_1.length());

    uint32_t user_text_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &user_text_id_1);
    assert(ret == 0);
    ret = recorder_.SetOverlay(video_track_id_4k, user_text_id_1);
    assert(ret == 0);
    overlay_ids_.push_back(user_text_id_1);


    // 6. Create UserText type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kUserText;
    object_params.location = OverlayLocationType::kRandom;
    object_params.color = 0x33CC00FF; //Green
    object_params.dst_rect.start_x = 2400;
    object_params.dst_rect.start_y = 300;
    object_params.dst_rect.width   = 480;
    object_params.dst_rect.height  = 60;
    std::string user_text_2("Location text");
    user_text_2.copy(object_params.user_text, user_text_2.length());

    uint32_t user_text_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &user_text_id_2);
    assert(ret == 0);
    ret = recorder_.SetOverlay(video_track_id_4k, user_text_id_2);
    assert(ret == 0);
    overlay_ids_.push_back(user_text_id_2);

    // 7. Create Static Image type overlay.
    uint32_t static_img_id;
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kBottomRight;
    std::string str("/etc/overlay_test.rgba");
    str.copy(object_params.image_info.image_location, str.length());
    object_params.dst_rect.width = 451;
    object_params.dst_rect.height = 109;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &static_img_id);
    assert(ret == 0);
    // Apply overlay object on video track.
    ret = recorder_.SetOverlay(video_track_id_4k, static_img_id);
    assert(ret == 0);
    overlay_ids_.push_back(static_img_id);

    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRAndOverlayBlob: This will test
*                                                 a MultiCamera session with
*                                                 3840x1920 stitched h264
*                                                 encoded track and surface
*                                                 downscale with SW TNR enabled
*                                                 with five Blob type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*  - CreateVideoTrack
*  - StartSession
*  - Enable TNR
*  - CreateOverlayObjects
*  - SetOverlays
*  - RemoveOverlays
*  - DeleteOverlayObjects
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRAndOverlayBlob) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;
  std::vector<uint32_t> overlay_ids_;

  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this](EventType event_type, void *event_data,
                                      size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kAVC,
                                            3840,
                                            1920,
                                            stream_fps};
    // Set parameters for and create 3840x1920 h264 encoded track.
    stream_width = 3840;
    stream_height = 1920;
    format_type = VideoFormat::kAVC;
    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = false;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {video_track_param.format_type,
                                    video_track_id_4k, stream_width,
                                    stream_height};
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [this](uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) -> void {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     video_track_param, extra_param,
                                     video_track_cb);

    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    char * image_buffer1;
    char * image_buffer2;
    char * image_buffer3;
    char * image_buffer4;
    char * image_buffer5;

    // 1. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width   = 1334;
    object_params.dst_rect.height  = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
        object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer =
        reinterpret_cast<char *>(malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer1 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
        object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_1);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_1);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_1);

    // 2. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 600;
    object_params.dst_rect.start_y = 600;
    object_params.dst_rect.width   = 1334;
    object_params.dst_rect.height  = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
        object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer =
        reinterpret_cast<char *>(malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer2 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
        object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_2);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_2);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_2);


    // 3. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1250;
    object_params.dst_rect.width   = 960;
    object_params.dst_rect.height  = 320;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 320;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
        object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer =
        reinterpret_cast<char *>(malloc(sizeof(char) * object_params.image_info.image_size));

    DrawOverlay(object_params.image_info.image_buffer,
        object_params.dst_rect.width, object_params.dst_rect.height);
    image_buffer3 = object_params.image_info.image_buffer;

    uint32_t usertxt_blob_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_3);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_3);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_3);

    // 4. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 3000;
    object_params.dst_rect.start_y = 1200;
    object_params.dst_rect.width   = 128;
    object_params.dst_rect.height  = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 128;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
        object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer =
        reinterpret_cast<char *>(malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer4 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
        object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_4);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_4);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_4);

    // 5. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 2200;
    object_params.dst_rect.start_y = 1600;
    object_params.dst_rect.width   = 960;
    object_params.dst_rect.height  = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
        object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer =
        reinterpret_cast<char *>(malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer5 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
        object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_5);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_5);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_5);

    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();

    free(image_buffer1);
    free(image_buffer2);
    free(image_buffer3);
    free(image_buffer4);
    free(image_buffer5);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithTNRWithOverlayBlobAnd480pYUVTrack: This case will test a
*                            MultiCamera session with one 3840x1920 h264 encoded
*                            with with SW TNR enabled; and one 960x480 YUV
*                            track ; both tracks configured to produce stitched
*                            frames. TNR will only be applied on the 4k stream.
*                            Usecase also includes five Blob type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*     CreateOverlayObjects
*  -  SetOverlays
*  -  RemoveOverlays
*  -  DeleteOverlayObjects
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithTNRWithOverlayBlobAnd480pYUVTrack) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;

  std::vector<uint32_t> overlay_ids_;

  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  uint32_t video_track_id_480p = 2;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this](EventType event_type, void *event_data,
                                      size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
              test_info_->name(), i);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kAVC,
                                            3840,
                                            1920,
                                            stream_fps};
    // Set parameters for and create 3840x1920 h264 encoded track.
    stream_width = 3840;
    stream_height = 1920;
    format_type = VideoFormat::kAVC;
    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = false;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {video_track_param.format_type,
                                    video_track_id_4k, stream_width,
                                    stream_height};
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [this](uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) -> void {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     video_track_param,
                                     video_track_cb);

    assert(ret == NO_ERROR);

    stream_width = 960;
    stream_height = 480;
    format_type = VideoFormat::kYUV;

    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = true;

    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    char *image_buffer1;
    char *image_buffer2;
    char *image_buffer3;
    char *image_buffer4;
    char *image_buffer5;

    // 1. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer1 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_1);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_1);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_1);

    // 2. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 600;
    object_params.dst_rect.start_y = 600;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer2 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_2);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_2);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_2);

    // 3. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1250;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 320;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 320;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);
    image_buffer3 = object_params.image_info.image_buffer;

    uint32_t usertxt_blob_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_3);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_3);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_3);

    // 4. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 3000;
    object_params.dst_rect.start_y = 1200;
    object_params.dst_rect.width = 128;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 128;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer4 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_4);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_4);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_4);

    // 5. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 2200;
    object_params.dst_rect.start_y = 1600;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer5 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_5);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_5);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_5);

    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p);
    assert(ret == NO_ERROR);
    dump_bitstream_.CloseAll();

    free(image_buffer1);
    free(image_buffer2);
    free(image_buffer3);
    free(image_buffer4);
    free(image_buffer5);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlobAnd480pYUVTrack:
*                            This case will test a MultiCamera session with
*                            one 3840x1920 h264 encoded with source surface
*                            downscaled with SW TNR enabled track; and one
*                            960x480 YUV track ; both tracks configured to
*                            produce stitched frames. TNR will only be applied
*                            on the 4k stream. Usecase also includes five
*                            Blob type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*     CreateOverlayObjects
*  -  SetOverlays
*  -  RemoveOverlays
*  -  DeleteOverlayObjects
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlobAnd480pYUVTrack) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;
  std::vector<uint32_t> overlay_ids_;

  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  uint32_t video_track_id_480p = 2;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this](EventType event_type, void *event_data,
                                      size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
              test_info_->name(), i);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kAVC,
                                            3840,
                                            1920,
                                            stream_fps};
    // Set parameters for and create 3840x1920 h264 encoded track.
    stream_width = 3840;
    stream_height = 1920;
    format_type = VideoFormat::kAVC;
    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = false;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {video_track_param.format_type,
                                    video_track_id_4k, stream_width,
                                    stream_height};
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [this](uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) -> void {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     video_track_param, extra_param,
                                     video_track_cb);

    assert(ret == NO_ERROR);

    stream_width = 960;
    stream_height = 480;
    format_type = VideoFormat::kYUV;

    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = true;

    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    char *image_buffer1;
    char *image_buffer2;
    char *image_buffer3;
    char *image_buffer4;
    char *image_buffer5;

    // 1. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer1 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_1);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_1);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_1);

    // 2. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 600;
    object_params.dst_rect.start_y = 600;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer2 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_2);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_2);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_2);

    // 3. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1250;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 320;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 320;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);
    image_buffer3 = object_params.image_info.image_buffer;

    uint32_t usertxt_blob_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_3);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_3);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_3);

    // 4. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 3000;
    object_params.dst_rect.start_y = 1200;
    object_params.dst_rect.width = 128;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 128;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer4 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_4);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_4);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_4);

    // 5. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 2200;
    object_params.dst_rect.start_y = 1600;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer5 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_5);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_5);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_5);

    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p);
    assert(ret == NO_ERROR);
    dump_bitstream_.CloseAll();

    free(image_buffer1);
    free(image_buffer2);
    free(image_buffer3);
    free(image_buffer4);
    free(image_buffer5);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlobAnd960pYUVTrack:
*                            This case will test a MultiCamera session with
*                            one 3840x1920 h264 encoded with source surface
*                            downscaled with SW TNR enabled track; and one
*                            1920x960 YUV track ; both tracks configured to
*                            produce stitched frames. TNR will only be applied
*                            on the 4k stream. Usecase also includes five
*                            Blob type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*     CreateOverlayObjects
*  -  SetOverlays
*  -  RemoveOverlays
*  -  DeleteOverlayObjects
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlobAnd960pYUVTrack) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;
  std::vector<uint32_t> overlay_ids_;

  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  uint32_t video_track_id_960p = 2;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this](EventType event_type, void *event_data,
                                      size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
              test_info_->name(), i);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kAVC,
                                            3840,
                                            1920,
                                            stream_fps};
    // Set parameters for and create 3840x1920 h264 encoded track.
    stream_width = 3840;
    stream_height = 1920;
    format_type = VideoFormat::kAVC;
    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = false;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {video_track_param.format_type,
                                    video_track_id_4k, stream_width,
                                    stream_height};
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [this](uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) -> void {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     video_track_param, extra_param,
                                     video_track_cb);

    assert(ret == NO_ERROR);

    stream_width = 1920;
    stream_height = 960;
    format_type = VideoFormat::kYUV;

    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = true;

    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_960p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    char *image_buffer1;
    char *image_buffer2;
    char *image_buffer3;
    char *image_buffer4;
    char *image_buffer5;

    // 1. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer1 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_1);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_1);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_1);

    // 2. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 600;
    object_params.dst_rect.start_y = 600;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer2 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_2);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_2);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_2);

    // 3. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1250;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 320;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 320;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);
    image_buffer3 = object_params.image_info.image_buffer;

    uint32_t usertxt_blob_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_3);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_3);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_3);

    // 4. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 3000;
    object_params.dst_rect.start_y = 1200;
    object_params.dst_rect.width = 128;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 128;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer4 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_4);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_4);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_4);

    // 5. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 2200;
    object_params.dst_rect.start_y = 1600;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer5 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_5);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_5);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_5);

    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_960p);
    assert(ret == NO_ERROR);
    dump_bitstream_.CloseAll();

    free(image_buffer1);
    free(image_buffer2);
    free(image_buffer3);
    free(image_buffer4);
    free(image_buffer5);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNR480pPreviewTrack960pYUVTrack: This case will test a
*                            MultiCamera session with one 3840x1920 h264 encoded
*                            with Source Surface Downscaling with SW TNR enabled.
*                            Also one 1920x960 YUV track ; both tracks configured
*                            to produce stitched frames. TNR will only be applied
*                            on the 4k stream.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNR480pPreviewTrack960pYUVTrack) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;
  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  uint32_t video_track_id_480p = 2;
  uint32_t video_track_id_960p = 3;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this](EventType event_type, void *event_data,
                                      size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
              test_info_->name(), i);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kAVC,
                                            3840,
                                            1920,
                                            stream_fps};
    // Set parameters for and create 3840x1920 h264 encoded track.
    stream_width = 3840;
    stream_height = 1920;
    format_type = VideoFormat::kAVC;
    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = false;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {video_track_param.format_type,
                                    video_track_id_4k, stream_width,
                                    stream_height};
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [this](uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) -> void {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     video_track_param, extra_param,
                                     video_track_cb);

    assert(ret == NO_ERROR);

    stream_width = 960;
    stream_height = 480;
    format_type = VideoFormat::kYUV;

    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;

    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    stream_width = 1920;
    stream_height = 960;
    format_type = VideoFormat::kYUV;

    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = true;

    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_960p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);
    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p);
    assert(ret == NO_ERROR);
    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_960p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNR480pPreviewEncTrack960pYUVTrack: This case will test a
*                            MultiCamera session with one 3840x1920 h264 encoded
*                            with Source Surface Downscaling with SW TNR enabled.
*                            Also one 1920x960 YUV track ; both tracks configured
*                            to produce stitched frames. TNR will only be applied
*                            on the 4k stream.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNR480pPreviewEncTrack960pYUVTrack) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;

  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  uint32_t video_track_id_480p = 2;
  uint32_t video_track_id_960p = 3;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this](EventType event_type, void *event_data,
                                      size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
              test_info_->name(), i);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kAVC,
                                            3840,
                                            1920,
                                            stream_fps};
    // Set parameters for and create 3840x1920 h264 encoded track.
    stream_width = 3840;
    stream_height = 1920;
    format_type = VideoFormat::kAVC;
    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = false;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {video_track_param.format_type,
                                    video_track_id_4k, stream_width,
                                    stream_height};
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [this](uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) -> void {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     video_track_param, extra_param,
                                     video_track_cb);

    assert(ret == NO_ERROR);

    stream_width = 960;
    stream_height = 480;
    format_type = VideoFormat::kAVC;

    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {video_track_param.format_type,
                                    video_track_id_480p, stream_width,
                                    stream_height};
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }
    video_track_cb.data_cb = [&, session_id](
            uint32_t track_id, std::vector<BufferDescriptor> buffers,
            std::vector<MetaData> meta_buffers) {
            VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers);
        };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);


    stream_width = 1920;
    stream_height = 960;
    format_type = VideoFormat::kYUV;

    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = true;

    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_960p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);
    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p);
    assert(ret == NO_ERROR);
    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_960p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob480pPreviewEncTrack960pYUVTrack:
*                            This case will test a MultiCamera session with
*                            one 3840x1920 h264 encoded with source surface
*                            downscaled with SW TNR enabled track; one 960X480
*                            h264 encode track; and one 1920x960 YUV track ; 
*                            all tracks  configured to produce stitched frames.
*                            TNR will only be applied on the 4k stream. Usecase also 
*                            includes five Blob type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*     CreateOverlayObjects
*  -  SetOverlays
*  -  RemoveOverlays
*  -  DeleteOverlayObjects
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob480pPreviewEncTrack960pYUVTrack) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;
  std::vector<uint32_t> overlay_ids_;

  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  uint32_t video_track_id_480p = 2;
  uint32_t video_track_id_960p = 3;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this](EventType event_type, void *event_data,
                                      size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
              test_info_->name(), i);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kAVC,
                                            3840,
                                            1920,
                                            stream_fps};
    // Set parameters for and create 3840x1920 h264 encoded track.
    stream_width = 3840;
    stream_height = 1920;
    format_type = VideoFormat::kAVC;
    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = false;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {video_track_param.format_type,
                                    video_track_id_4k, stream_width,
                                    stream_height};
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [this](uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) -> void {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     video_track_param, extra_param,
                                     video_track_cb);

    assert(ret == NO_ERROR);

    stream_width = 960;
    stream_height = 480;
    format_type = VideoFormat::kAVC;

    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {video_track_param.format_type,
                                    video_track_id_480p, stream_width,
                                    stream_height};
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
        VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    stream_width = 1920;
    stream_height = 960;
    format_type = VideoFormat::kYUV;

    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = true;

    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
        VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_960p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    char *image_buffer1;
    char *image_buffer2;
    char *image_buffer3;
    char *image_buffer4;
    char *image_buffer5;

    // 1. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer1 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_1);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_1);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_1);

    // 2. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 600;
    object_params.dst_rect.start_y = 600;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer2 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_2);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_2);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_2);

    // 3. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1250;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 320;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 320;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);
    image_buffer3 = object_params.image_info.image_buffer;

    uint32_t usertxt_blob_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_3);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_3);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_3);

    // 4. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 3000;
    object_params.dst_rect.start_y = 1200;
    object_params.dst_rect.width = 128;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 128;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer4 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_4);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_4);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_4);

    // 5. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 2200;
    object_params.dst_rect.start_y = 1600;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer5 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_5);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_5);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_5);

    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_960p);
    assert(ret == NO_ERROR);
    dump_bitstream_.CloseAll();

    free(image_buffer1);
    free(image_buffer2);
    free(image_buffer3);
    free(image_buffer4);
    free(image_buffer5);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}


/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob480pPreviewEncTrack960pYUVTrack24FPS:
*                            This case will test a MultiCamera session with
*                            one 3840x1920 h264 encoded with source surface
*                            downscaled with SW TNR enabled track; and one
*                            1920x960 YUV track ; both tracks configured to
*                            produce stitched frames. TNR will only be applied
*                            on the 4k stream. Usecase also includes five
*                            Blob type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*     CreateOverlayObjects
*  -  SetOverlays
*  -  RemoveOverlays
*  -  DeleteOverlayObjects
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob480pPreviewEncTrack960pYUVTrack24FPS) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;
  std::vector<uint32_t> overlay_ids_;

  float stream_fps = 24;
  uint32_t video_track_id_4k = 1;
  uint32_t video_track_id_480p = 2;
  uint32_t video_track_id_960p = 3;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this](EventType event_type, void *event_data,
                                      size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
              test_info_->name(), i);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kAVC,
                                            3840,
                                            1920,
                                            stream_fps};
    // Set parameters for and create 3840x1920 h264 encoded track.
    stream_width = 3840;
    stream_height = 1920;
    format_type = VideoFormat::kAVC;
    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = false;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {video_track_param.format_type,
                                    video_track_id_4k, stream_width,
                                    stream_height};
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [this](uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) -> void {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     video_track_param, extra_param,
                                     video_track_cb);

    assert(ret == NO_ERROR);

    stream_width = 960;
    stream_height = 480;
    format_type = VideoFormat::kAVC;

    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;

    video_track_param.codec_param.avc.idr_interval = 1;
    video_track_param.codec_param.avc.bitrate = 12000000;
    video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    video_track_param.codec_param.avc.ltr_count = 0;
    video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {video_track_param.format_type,
                                    video_track_id_480p, stream_width,
                                    stream_height};
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
        VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    stream_width = 1920;
    stream_height = 960;
    format_type = VideoFormat::kYUV;

    video_track_param.camera_id = multicam_id_;
    video_track_param.width = stream_width;
    video_track_param.height = stream_height;
    video_track_param.frame_rate = stream_fps;
    video_track_param.format_type = format_type;
    video_track_param.low_power_mode = true;

    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_960p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    char *image_buffer1;
    char *image_buffer2;
    char *image_buffer3;
    char *image_buffer4;
    char *image_buffer5;

    // 1. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer1 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_1);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_1);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_1);

    // 2. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 600;
    object_params.dst_rect.start_y = 600;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer2 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_2);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_2);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_2);

    // 3. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1250;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 320;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 320;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);
    image_buffer3 = object_params.image_info.image_buffer;

    uint32_t usertxt_blob_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_3);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_3);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_3);

    // 4. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 3000;
    object_params.dst_rect.start_y = 1200;
    object_params.dst_rect.width = 128;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 128;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer4 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_4);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_4);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_4);

    // 5. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 2200;
    object_params.dst_rect.start_y = 1600;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer5 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_5);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_5);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_5);

    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_960p);
    assert(ret == NO_ERROR);
    dump_bitstream_.CloseAll();

    free(image_buffer1);
    free(image_buffer2);
    free(image_buffer3);
    free(image_buffer4);
    free(image_buffer5);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNR960pEncTrack960pYUVTrackWithRescaler:
*                            This case will test a MultiCamera session with
*                            one 3840x1920 h264 encoded with source surface
*                            downscaled with SW TNR enabled track; and one
*                            1920x960 YUV track ; both tracks configured to
*                            produce stitched frames. TNR will only be applied
*                            on the 4k stream. Usecase also includes five
*                            Blob type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*     CreateOverlayObjects
*  -  SetOverlays
*  -  RemoveOverlays
*  -  DeleteOverlayObjects
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNR960pEncTrack960pYUVTrackWithRescaler) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = 30;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    VideoTrackCreateParam master_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   3840,
                                                   1920,
                                                   30};

    master_video_track_param.codec_param.avc.idr_interval = 1;
    master_video_track_param.codec_param.avc.bitrate = 12000000;
    master_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    master_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    master_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    master_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    master_video_track_param.codec_param.avc.ltr_count = 0;
    master_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    uint32_t video_track_id_4k  = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        master_video_track_param.format_type,
        video_track_id_4k,
        master_video_track_param.width,
        master_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
          VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
        };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }
    
    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     master_video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    // Second Track
    uint32_t video_track_id_960p = 2;
    VideoTrackCreateParam second_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   1920,
                                                   960,
                                                   30};
    second_video_track_param.low_power_mode = true;
    second_video_track_param.codec_param.avc.idr_interval = 1;
    second_video_track_param.codec_param.avc.bitrate = 12000000;
    second_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    second_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    second_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    second_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    second_video_track_param.codec_param.avc.ltr_count = 0;
    second_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        second_video_track_param.format_type,
        video_track_id_960p,
        second_video_track_param.width,
        second_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb2;
    video_track_cb2.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb2.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_960p,
                                     second_video_track_param, video_track_cb2);
    assert(ret == NO_ERROR);

    //Third Track
    uint32_t yuv_track_id_960p  = 3;
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                                   1920,
                                                   960,
                                                   30};
    video_track_param.low_power_mode = false;

    TrackCb yuv_track_cb3;
    yuv_track_cb3.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    yuv_track_cb3.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param_2;
    SourceVideoTrack surface_video_copy_2;
    surface_video_copy_2.source_track_id = video_track_id_960p;
    extra_param_2.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy_2);

    ret = recorder_.CreateVideoTrack(session_id, yuv_track_id_960p,
                                    video_track_param, extra_param_2,
                                    yuv_track_cb3);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Let session run for time record_duration_, during this time buffer with
    // valid data would be received in track callback (VideoTrackYUVDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_960p);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, yuv_track_id_960p);
    assert(ret == NO_ERROR);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob960pEncTrack960pYUVTrackWithRescaler:
*                            This case will test a MultiCamera session with
*                            one 3840x1920 h264 encoded with source surface
*                            downscaled with SW TNR enabled track; and one
*                            1920x960 YUV track ; both tracks configured to
*                            produce stitched frames. TNR will only be applied
*                            on the 4k stream. Usecase also includes five
*                            Blob type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*     CreateOverlayObjects
*  -  SetOverlays
*  -  RemoveOverlays
*  -  DeleteOverlayObjects
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob960pEncTrack960pYUVTrackWithRescaler) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  std::vector<uint32_t> overlay_ids_;
  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = 30;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    VideoTrackCreateParam master_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   3840,
                                                   1920,
                                                   30};

    master_video_track_param.codec_param.avc.idr_interval = 1;
    master_video_track_param.codec_param.avc.bitrate = 12000000;
    master_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    master_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    master_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    master_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    master_video_track_param.codec_param.avc.ltr_count = 0;
    master_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    uint32_t video_track_id_4k  = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        master_video_track_param.format_type,
        video_track_id_4k,
        master_video_track_param.width,
        master_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
          VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
        };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     master_video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    // Second Track
    uint32_t video_track_id_960p = 2;
    VideoTrackCreateParam second_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   1920,
                                                   960,
                                                   30};
    second_video_track_param.low_power_mode = true;

    second_video_track_param.codec_param.avc.idr_interval = 1;
    second_video_track_param.codec_param.avc.bitrate = 12000000;
    second_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    second_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    second_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    second_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    second_video_track_param.codec_param.avc.ltr_count = 0;
    second_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        second_video_track_param.format_type,
        video_track_id_960p,
        second_video_track_param.width,
        second_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb2;
    video_track_cb2.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb2.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_960p,
                                     second_video_track_param, video_track_cb2);
    assert(ret == NO_ERROR);

    //Third Track
    uint32_t yuv_track_id_960p  = 3;

    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                            1920,
                                            960,
                                            30};

    video_track_param.low_power_mode = false;

    TrackCb yuv_track_cb3;
    yuv_track_cb3.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    yuv_track_cb3.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param_2;
    SourceVideoTrack surface_video_copy_2;
    surface_video_copy_2.source_track_id = video_track_id_960p;
    extra_param_2.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy_2);

    ret = recorder_.CreateVideoTrack(session_id, yuv_track_id_960p,
                                    video_track_param, extra_param_2,
                                    yuv_track_cb3);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    char *image_buffer1;
    char *image_buffer2;
    char *image_buffer3;
    char *image_buffer4;
    char *image_buffer5;

    // 1. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer1 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_1);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_1);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_1);

    // 2. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 600;
    object_params.dst_rect.start_y = 600;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer2 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_2);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_2);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_2);

    // 3. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1250;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 320;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 320;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);
    image_buffer3 = object_params.image_info.image_buffer;

    uint32_t usertxt_blob_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_3);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_3);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_3);

    // 4. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 3000;
    object_params.dst_rect.start_y = 1200;
    object_params.dst_rect.width = 128;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 128;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer4 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_4);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_4);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_4);

    // 5. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 2200;
    object_params.dst_rect.start_y = 1600;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer5 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_5);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_5);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_5);

    // Let session run for time record_duration_.
    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_960p);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, yuv_track_id_960p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();

    free(image_buffer1);
    free(image_buffer2);
    free(image_buffer3);
    free(image_buffer4);
    free(image_buffer5);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob960pEncTrack960pYUVTrackWithRescaler24FPS:
*                            This case will test a MultiCamera session with
*                            one 3840x1920 h264 encoded with source surface
*                            downscaled with SW TNR enabled track; and one
*                            1920x960 YUV track ; both tracks configured to
*                            produce stitched frames. TNR will only be applied
*                            on the 4k stream. Usecase also includes five
*                            Blob type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*     CreateOverlayObjects
*  -  SetOverlays
*  -  RemoveOverlays
*  -  DeleteOverlayObjects
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob960pEncTrack960pYUVTrackWithRescaler24FPS) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

 std::vector<uint32_t> overlay_ids_;
  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = 24;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    VideoTrackCreateParam master_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   3840,
                                                   1920,
                                                   24};
    master_video_track_param.codec_param.avc.idr_interval = 1;
    master_video_track_param.codec_param.avc.bitrate = 12000000;
    master_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    master_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    master_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    master_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    master_video_track_param.codec_param.avc.ltr_count = 0;
    master_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    uint32_t video_track_id_4k  = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        master_video_track_param.format_type,
        video_track_id_4k,
        master_video_track_param.width,
        master_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
          VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
        };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     master_video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    // Second Track
    uint32_t video_track_id_960p = 2;
    VideoTrackCreateParam second_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   1920,
                                                   960,
                                                   24};
    second_video_track_param.low_power_mode = true;

    second_video_track_param.codec_param.avc.idr_interval = 1;
    second_video_track_param.codec_param.avc.bitrate = 12000000;
    second_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    second_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    second_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    second_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    second_video_track_param.codec_param.avc.ltr_count = 0;
    second_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        second_video_track_param.format_type,
        video_track_id_960p,
        second_video_track_param.width,
        second_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb2;
    video_track_cb2.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb2.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_960p,
                                     second_video_track_param, video_track_cb2);
    assert(ret == NO_ERROR);

    //Third Track
    uint32_t yuv_track_id_960p  = 3;
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                            1920,
                                            960,
                                            24};
    video_track_param.low_power_mode = false;

    TrackCb yuv_track_cb3;
    yuv_track_cb3.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    yuv_track_cb3.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param_2;
    SourceVideoTrack surface_video_copy_2;
    surface_video_copy_2.source_track_id = video_track_id_960p;
    extra_param_2.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy_2);

    ret = recorder_.CreateVideoTrack(session_id, yuv_track_id_960p,
                                    video_track_param, extra_param_2,
                                    yuv_track_cb3);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    char *image_buffer1;
    char *image_buffer2;
    char *image_buffer3;
    char *image_buffer4;
    char *image_buffer5;

    // 1. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer1 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_1);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_1);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_1);

    // 2. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 600;
    object_params.dst_rect.start_y = 600;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer2 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_2);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_2);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_2);

    // 3. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1250;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 320;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 320;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);
    image_buffer3 = object_params.image_info.image_buffer;

    uint32_t usertxt_blob_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_3);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_3);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_3);

    // 4. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 3000;
    object_params.dst_rect.start_y = 1200;
    object_params.dst_rect.width = 128;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 128;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer4 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_4);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_4);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_4);

    // 5. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 2200;
    object_params.dst_rect.start_y = 1600;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer5 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_5);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_5);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_5);

    // Let session run for time record_duration_.
    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_960p);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, yuv_track_id_960p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();

    free(image_buffer1);
    free(image_buffer2);
    free(image_buffer3);
    free(image_buffer4);
    free(image_buffer5);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob960pEncTrackWithTNR960pYUVTrackWithRescaler:
*                            This case will test a MultiCamera session with
*                            one 3840x1920 h264 encoded with source surface
*                            downscaled with SW TNR enabled track; and one
*                            1920x960 YUV track ; both tracks configured to
*                            produce stitched frames. TNR will only be applied
*                            on the 4k stream. Usecase also includes five
*                            Blob type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*     CreateOverlayObjects
*  -  SetOverlays
*  -  RemoveOverlays
*  -  DeleteOverlayObjects
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob960pEncTrackWithTNR960pYUVTrackWithRescaler) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  std::vector<uint32_t> overlay_ids_;
  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = 30;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    VideoTrackCreateParam master_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   3840,
                                                   1920,
                                                   30};

    master_video_track_param.codec_param.avc.idr_interval = 1;
    master_video_track_param.codec_param.avc.bitrate = 12000000;
    master_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    master_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    master_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    master_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    master_video_track_param.codec_param.avc.ltr_count = 0;
    master_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    uint32_t video_track_id_4k  = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        master_video_track_param.format_type,
        video_track_id_4k,
        master_video_track_param.width,
        master_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
          VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
        };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     master_video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    // Second Track
    uint32_t video_track_id_960p = 2;
    VideoTrackCreateParam second_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   1920,
                                                   960,
                                                   30};

    second_video_track_param.codec_param.avc.idr_interval = 1;
    second_video_track_param.codec_param.avc.bitrate = 12000000;
    second_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    second_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    second_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    second_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    second_video_track_param.codec_param.avc.ltr_count = 0;
    second_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        second_video_track_param.format_type,
        video_track_id_960p,
        second_video_track_param.width,
        second_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb2;
    video_track_cb2.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb2.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_960p,
                                     second_video_track_param, video_track_cb2);
    assert(ret == NO_ERROR);

    //Third Track
    uint32_t yuv_track_id_960p  = 3;
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                            1920,
                                            960,
                                            30};

    video_track_param.low_power_mode = false;

    TrackCb yuv_track_cb3;
    yuv_track_cb3.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    yuv_track_cb3.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param_2;
    SourceVideoTrack surface_video_copy_2;
    surface_video_copy_2.source_track_id = video_track_id_960p;
    extra_param_2.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy_2);

    ret = recorder_.CreateVideoTrack(session_id, yuv_track_id_960p,
                                    video_track_param, extra_param_2,
                                    yuv_track_cb3);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    char *image_buffer1;
    char *image_buffer2;
    char *image_buffer3;
    char *image_buffer4;
    char *image_buffer5;

    // 1. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer1 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_1);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_1);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_1);

    // 2. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 600;
    object_params.dst_rect.start_y = 600;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer2 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_2);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_2);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_2);

    // 3. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1250;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 320;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 320;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);
    image_buffer3 = object_params.image_info.image_buffer;

    uint32_t usertxt_blob_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_3);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_3);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_3);

    // 4. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 3000;
    object_params.dst_rect.start_y = 1200;
    object_params.dst_rect.width = 128;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 128;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer4 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_4);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_4);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_4);

    // 5. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 2200;
    object_params.dst_rect.start_y = 1600;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer5 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_5);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_5);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_5);

    // Let session run for time record_duration_.
    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_960p);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, yuv_track_id_960p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();

    free(image_buffer1);
    free(image_buffer2);
    free(image_buffer3);
    free(image_buffer4);
    free(image_buffer5);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob960pEncTrackWithTNR960pYUVTrackWithRescaler24FPS:
*                            This case will test a MultiCamera session with
*                            one 3840x1920 h264 encoded with source surface
*                            downscaled with SW TNR enabled track; and one
*                            1920x960 YUV track ; both tracks configured to
*                            produce stitched frames. TNR will only be applied
*                            on the 4k stream. Usecase also includes five
*                            Blob type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*     CreateOverlayObjects
*  -  SetOverlays
*  -  RemoveOverlays
*  -  DeleteOverlayObjects
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob960pEncTrackWithTNR960pYUVTrackWithRescaler24FPS) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

 std::vector<uint32_t> overlay_ids_;
  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = 24;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    VideoTrackCreateParam master_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   3840,
                                                   1920,
                                                   24};

    master_video_track_param.codec_param.avc.idr_interval = 1;
    master_video_track_param.codec_param.avc.bitrate = 12000000;
    master_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    master_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    master_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    master_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    master_video_track_param.codec_param.avc.ltr_count = 0;
    master_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    uint32_t video_track_id_4k  = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        master_video_track_param.format_type,
        video_track_id_4k,
        master_video_track_param.width,
        master_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
          VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
        };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     master_video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    // Second Track
    uint32_t video_track_id_960p = 2;
    VideoTrackCreateParam second_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   1920,
                                                   960,
                                                   24};

    second_video_track_param.codec_param.avc.idr_interval = 1;
    second_video_track_param.codec_param.avc.bitrate = 12000000;
    second_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    second_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    second_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    second_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    second_video_track_param.codec_param.avc.ltr_count = 0;
    second_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        second_video_track_param.format_type,
        video_track_id_960p,
       second_video_track_param.width,
       second_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb2;
    video_track_cb2.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb2.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_960p,
                                     second_video_track_param, video_track_cb2);
    assert(ret == NO_ERROR);

    //Third Track
    uint32_t yuv_track_id_960p  = 3;
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                            1920,
                                            960,
                                            24};

    video_track_param.low_power_mode = false;

    TrackCb yuv_track_cb3;
    yuv_track_cb3.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    yuv_track_cb3.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param_2;
    SourceVideoTrack surface_video_copy_2;
    surface_video_copy_2.source_track_id = video_track_id_960p;
    extra_param_2.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy_2);

    ret = recorder_.CreateVideoTrack(session_id, yuv_track_id_960p,
                                    video_track_param, extra_param_2,
                                    yuv_track_cb3);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    char *image_buffer1;
    char *image_buffer2;
    char *image_buffer3;
    char *image_buffer4;
    char *image_buffer5;

    // 1. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer1 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_1);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_1);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_1);

    // 2. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 600;
    object_params.dst_rect.start_y = 600;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer2 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_2);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_2);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_2);

    // 3. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1250;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 320;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 320;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);
    image_buffer3 = object_params.image_info.image_buffer;

    uint32_t usertxt_blob_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_3);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_3);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_3);

    // 4. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 3000;
    object_params.dst_rect.start_y = 1200;
    object_params.dst_rect.width = 128;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 128;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer4 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_4);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_4);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_4);

    // 5. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 2200;
    object_params.dst_rect.start_y = 1600;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer5 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_5);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_5);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_5);

    // Let session run for time record_duration_.
    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_960p);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, yuv_track_id_960p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();

    free(image_buffer1);
    free(image_buffer2);
    free(image_buffer3);
    free(image_buffer4);
    free(image_buffer5);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob720pEncTrackWithTNR720pYUVTrackWithRescaler:
*                            This case will test a MultiCamera session with
*                            one 3840x1920 h264 encoded with source surface
*                            downscaled with SW TNR enabled track; and one
*                            1920x960 YUV track ; both tracks configured to
*                            produce stitched frames. TNR will only be applied
*                            on the 4k stream. Usecase also includes five
*                            Blob type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*     CreateOverlayObjects
*  -  SetOverlays
*  -  RemoveOverlays
*  -  DeleteOverlayObjects
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob720pEncTrackWithTNR720pYUVTrackWithRescaler) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  std::vector<uint32_t> overlay_ids_;
  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = 30;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    VideoTrackCreateParam master_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   3840,
                                                   1920,
                                                   30};

    master_video_track_param.codec_param.avc.idr_interval = 1;
    master_video_track_param.codec_param.avc.bitrate = 12000000;
    master_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    master_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    master_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    master_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    master_video_track_param.codec_param.avc.ltr_count = 0;
    master_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    uint32_t video_track_id_4k  = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        master_video_track_param.format_type,
        video_track_id_4k,
        master_video_track_param.width,
        master_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
          VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
        };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     master_video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    // Second Track
    uint32_t video_track_id_720p = 2;
    VideoTrackCreateParam second_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   1440,
                                                   720,
                                                   30};
    second_video_track_param.codec_param.avc.idr_interval = 1;
    second_video_track_param.codec_param.avc.bitrate = 12000000;
    second_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    second_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    second_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    second_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    second_video_track_param.codec_param.avc.ltr_count = 0;
    second_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        second_video_track_param.format_type,
        video_track_id_720p,
        second_video_track_param.width,
        second_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb2;
    video_track_cb2.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb2.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_720p,
                                     second_video_track_param, video_track_cb2);
    assert(ret == NO_ERROR);

    //Third Track
    uint32_t yuv_track_id_720p  = 3;
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                            1440,
                                            720,
                                            30};

    TrackCb yuv_track_cb3;
    yuv_track_cb3.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    yuv_track_cb3.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param_2;
    SourceVideoTrack surface_video_copy_2;
    surface_video_copy_2.source_track_id = video_track_id_720p;
    extra_param_2.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy_2);

    ret = recorder_.CreateVideoTrack(session_id, yuv_track_id_720p,
                                    video_track_param, extra_param_2,
                                    yuv_track_cb3);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    char *image_buffer1;
    char *image_buffer2;
    char *image_buffer3;
    char *image_buffer4;
    char *image_buffer5;

    // 1. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer1 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_1);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_1);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_1);

    // 2. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 600;
    object_params.dst_rect.start_y = 600;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer2 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_2);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_2);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_2);

    // 3. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1250;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 320;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 320;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);
    image_buffer3 = object_params.image_info.image_buffer;

    uint32_t usertxt_blob_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_3);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_3);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_3);

    // 4. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 3000;
    object_params.dst_rect.start_y = 1200;
    object_params.dst_rect.width = 128;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 128;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer4 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_4);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_4);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_4);

    // 5. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 2200;
    object_params.dst_rect.start_y = 1600;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer5 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_5);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_5);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_5);

    // Let session run for time record_duration_.
    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_720p);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, yuv_track_id_720p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();

    free(image_buffer1);
    free(image_buffer2);
    free(image_buffer3);
    free(image_buffer4);
    free(image_buffer5);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob720pEncCBTrack720pYUVTrackWithRescaler:
*                            This case will test a MultiCamera session with
*                            one 3840x1920 h264 encoded with source surface
*                            downscaled with SW TNR enabled track; and one
*                            1920x960 YUV track ; both tracks configured to
*                            produce stitched frames. TNR will only be applied
*                            on the 4k stream. Usecase also includes five
*                            Blob type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*     CreateOverlayObjects
*  -  SetOverlays
*  -  RemoveOverlays
*  -  DeleteOverlayObjects
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob720pEncCBTrack720pYUVTrackWithRescaler) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  std::vector<uint32_t> overlay_ids_;
  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = 30;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    VideoTrackCreateParam master_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   3840,
                                                   1920,
                                                   30};

    master_video_track_param.codec_param.avc.idr_interval = 1;
    master_video_track_param.codec_param.avc.bitrate = 12000000;
    master_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    master_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    master_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    master_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    master_video_track_param.codec_param.avc.ltr_count = 0;
    master_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    uint32_t video_track_id_4k  = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        master_video_track_param.format_type,
        video_track_id_4k,
        master_video_track_param.width,
        master_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
          VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
        };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     master_video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    // Second Track
    uint32_t video_track_id_720p = 2;
    VideoTrackCreateParam second_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   1440,
                                                   720,
                                                   30};
    second_video_track_param.low_power_mode = true;

    second_video_track_param.codec_param.avc.idr_interval = 1;
    second_video_track_param.codec_param.avc.bitrate = 12000000;
    second_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    second_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    second_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    second_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    second_video_track_param.codec_param.avc.ltr_count = 0;
    second_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        second_video_track_param.format_type,
        video_track_id_720p,
        second_video_track_param.width,
        second_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb2;
    video_track_cb2.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb2.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_720p,
                                     second_video_track_param, video_track_cb2);
    assert(ret == NO_ERROR);

    //Third Track
    uint32_t yuv_track_id_720p  = 3;
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                            1440,
                                            720,
                                            30};

    TrackCb yuv_track_cb3;
    yuv_track_cb3.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    yuv_track_cb3.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param_2;
    SourceVideoTrack surface_video_copy_2;
    surface_video_copy_2.source_track_id = video_track_id_720p;
    extra_param_2.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy_2);

    ret = recorder_.CreateVideoTrack(session_id, yuv_track_id_720p,
                                    video_track_param, extra_param_2,
                                    yuv_track_cb3);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    char *image_buffer1;
    char *image_buffer2;
    char *image_buffer3;
    char *image_buffer4;
    char *image_buffer5;

    // 1. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer1 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_1);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_1);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_1);

    // 2. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 600;
    object_params.dst_rect.start_y = 600;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer2 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_2);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_2);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_2);

    // 3. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1250;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 320;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 320;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);
    image_buffer3 = object_params.image_info.image_buffer;

    uint32_t usertxt_blob_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_3);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_3);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_3);

    // 4. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 3000;
    object_params.dst_rect.start_y = 1200;
    object_params.dst_rect.width = 128;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 128;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer4 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_4);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_4);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_4);

    // 5. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 2200;
    object_params.dst_rect.start_y = 1600;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer5 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_5);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_5);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_5);

    // Let session run for time record_duration_.
    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_720p);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, yuv_track_id_720p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();

    free(image_buffer1);
    free(image_buffer2);
    free(image_buffer3);
    free(image_buffer4);
    free(image_buffer5);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob720pYUVCBTrack720pYUVTrackWithRescaler:
*                            This case will test a MultiCamera session with
*                            one 3840x1920 h264 encoded with source surface
*                            downscaled with SW TNR enabled track; and one
*                            1920x960 YUV track ; both tracks configured to
*                            produce stitched frames. TNR will only be applied
*                            on the 4k stream. Usecase also includes five
*                            Blob type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*     CreateOverlayObjects
*  -  SetOverlays
*  -  RemoveOverlays
*  -  DeleteOverlayObjects
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob720pYUVCBTrack720pYUVTrackWithRescaler) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  std::vector<uint32_t> overlay_ids_;
  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = 30;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    VideoTrackCreateParam master_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   3840,
                                                   1920,
                                                   30};

    master_video_track_param.codec_param.avc.idr_interval = 1;
    master_video_track_param.codec_param.avc.bitrate = 12000000;
    master_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    master_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    master_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    master_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    master_video_track_param.codec_param.avc.ltr_count = 0;
    master_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    uint32_t video_track_id_4k  = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        master_video_track_param.format_type,
        video_track_id_4k,
        master_video_track_param.width,
        master_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
          VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
        };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     master_video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    // Second Track
    uint32_t yuv_track_id_720p_src = 2;
    VideoTrackCreateParam second_video_track_param{multicam_id_, VideoFormat::kYUV,
                                                   1440,
                                                   720,
                                                   30};

    second_video_track_param.low_power_mode = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        second_video_track_param.format_type,
        yuv_track_id_720p_src,
        second_video_track_param.width,
        second_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb2;
    video_track_cb2.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb2.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, yuv_track_id_720p_src,
                                     second_video_track_param, video_track_cb2);
    assert(ret == NO_ERROR);

    //Third Track
    uint32_t yuv_track_id_720p  = 3;
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                                   1440,
                                                   720,
                                                   30};

    TrackCb yuv_track_cb3;
    yuv_track_cb3.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    yuv_track_cb3.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param_2;
    SourceVideoTrack surface_video_copy_2;
    surface_video_copy_2.source_track_id = yuv_track_id_720p_src;
    extra_param_2.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy_2);

    ret = recorder_.CreateVideoTrack(session_id, yuv_track_id_720p,
                                    video_track_param, extra_param_2,
                                    yuv_track_cb3);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    char *image_buffer1;
    char *image_buffer2;
    char *image_buffer3;
    char *image_buffer4;
    char *image_buffer5;

    // 1. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer1 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_1);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_1);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_1);

    // 2. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 600;
    object_params.dst_rect.start_y = 600;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer2 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_2);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_2);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_2);

    // 3. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1250;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 320;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 320;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);
    image_buffer3 = object_params.image_info.image_buffer;

    uint32_t usertxt_blob_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_3);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_3);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_3);

    // 4. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 3000;
    object_params.dst_rect.start_y = 1200;
    object_params.dst_rect.width = 128;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 128;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer4 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_4);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_4);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_4);

    // 5. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 2200;
    object_params.dst_rect.start_y = 1600;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer5 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_5);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_5);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_5);

    // Let session run for time record_duration_.
    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, yuv_track_id_720p_src);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, yuv_track_id_720p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();

    free(image_buffer1);
    free(image_buffer2);
    free(image_buffer3);
    free(image_buffer4);
    free(image_buffer5);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob720pEncTrackWithTNR720pYUVTrackWithRescaler24FPS:
*                            This case will test a MultiCamera session with
*                            one 3840x1920 h264 encoded with source surface
*                            downscaled with SW TNR enabled track; and one
*                            1920x960 YUV track ; both tracks configured to
*                            produce stitched frames. TNR will only be applied
*                            on the 4k stream. Usecase also includes five
*                            Blob type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*     CreateOverlayObjects
*  -  SetOverlays
*  -  RemoveOverlays
*  -  DeleteOverlayObjects
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob720pEncTrackWithTNR720pYUVTrackWithRescaler24FPS) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

 std::vector<uint32_t> overlay_ids_;
  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = 24;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    VideoTrackCreateParam master_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   3840,
                                                   1920,
                                                   24};

    master_video_track_param.codec_param.avc.idr_interval = 1;
    master_video_track_param.codec_param.avc.bitrate = 12000000;
    master_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    master_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    master_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    master_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    master_video_track_param.codec_param.avc.ltr_count = 0;
    master_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    uint32_t video_track_id_4k  = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        master_video_track_param.format_type,
        video_track_id_4k,
        master_video_track_param.width,
        master_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
          VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
        };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     master_video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    // Second Track
    uint32_t video_track_id_720p = 2;
    VideoTrackCreateParam second_video_track_param{multicam_id_, VideoFormat::kAVC,
                                                   1440,
                                                   720,
                                                   24};

    second_video_track_param.codec_param.avc.idr_interval = 1;
    second_video_track_param.codec_param.avc.bitrate = 12000000;
    second_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    second_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    second_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    second_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    second_video_track_param.codec_param.avc.ltr_count = 0;
    second_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        second_video_track_param.format_type,
        video_track_id_720p,
        second_video_track_param.width,
        second_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb2;
    video_track_cb2.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb2.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_720p,
                                     second_video_track_param, video_track_cb2);
    assert(ret == NO_ERROR);

    //Third Track
    uint32_t yuv_track_id_720p  = 3;
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                            1440,
                                            720,
                                            24};

    TrackCb yuv_track_cb3;
    yuv_track_cb3.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    yuv_track_cb3.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param_2;
    SourceVideoTrack surface_video_copy_2;
    surface_video_copy_2.source_track_id = video_track_id_720p;
    extra_param_2.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy_2);

    ret = recorder_.CreateVideoTrack(session_id, yuv_track_id_720p,
                                    video_track_param, extra_param_2,
                                    yuv_track_cb3);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    char *image_buffer1;
    char *image_buffer2;
    char *image_buffer3;
    char *image_buffer4;
    char *image_buffer5;

    // 1. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer1 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_1);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_1);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_1);

    // 2. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 600;
    object_params.dst_rect.start_y = 600;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer2 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_2);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_2);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_2);

    // 3. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1250;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 320;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 320;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);
    image_buffer3 = object_params.image_info.image_buffer;

    uint32_t usertxt_blob_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_3);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_3);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_3);

    // 4. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 3000;
    object_params.dst_rect.start_y = 1200;
    object_params.dst_rect.width = 128;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 128;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer4 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_4);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_4);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_4);

    // 5. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 2200;
    object_params.dst_rect.start_y = 1600;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer5 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_5);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_5);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_5);

    // Let session run for time record_duration_.
    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_720p);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, yuv_track_id_720p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();

    free(image_buffer1);
    free(image_buffer2);
    free(image_buffer3);
    free(image_buffer4);
    free(image_buffer5);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob480pEncTrackFromRescaler720pYUVTrack24FPS:
*                            This case will test a MultiCamera session with
*                            one 3840x1920 h264 encoded with source surface
*                            downscaled with SW TNR enabled track; and one
*                            1920x960 YUV track ; both tracks configured to
*                            produce stitched frames. TNR will only be applied
*                            on the 4k stream. Usecase also includes five
*                            Blob type Overlays.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - CreateVideoTrack 3
*   - StartVideoTrack
*     CreateOverlayObjects
*  -  SetOverlays
*  -  RemoveOverlays
*  -  DeleteOverlayObjects
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   - DeleteVideoTrack 3
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackWithSrcSurfDSWithTNRWithOverlayBlob480pEncTrackFromRescaler720pYUVTrack24FPS) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

 std::vector<uint32_t> overlay_ids_;
  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = 24;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    VideoTrackCreateParam master_video_track_param;
    memset(&master_video_track_param, 0x0, sizeof master_video_track_param);

    master_video_track_param.camera_id   = multicam_id_;
    master_video_track_param.width       = 3840;
    master_video_track_param.height      = 1920;
    master_video_track_param.frame_rate  = 24;
    master_video_track_param.format_type = VideoFormat::kAVC;

    master_video_track_param.codec_param.avc.idr_interval = 1;
    master_video_track_param.codec_param.avc.bitrate = 12000000;
    master_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    master_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    master_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    master_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    master_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    master_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    master_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    master_video_track_param.codec_param.avc.ltr_count = 0;
    master_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    uint32_t video_track_id_4k  = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        master_video_track_param.format_type,
        video_track_id_4k,
        master_video_track_param.width,
        master_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
          VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
        };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1600;
      source_surface.height = 1600;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     master_video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    // Second Track
    uint32_t video_track_id_480p = 2;

    VideoTrackCreateParam second_video_track_param;
    memset(&second_video_track_param, 0x0, sizeof second_video_track_param);
    second_video_track_param.camera_id = multicam_id_;
    second_video_track_param.width = 960;
    second_video_track_param.height = 480;
    second_video_track_param.frame_rate = 24;
    second_video_track_param.format_type = VideoFormat::kAVC;

    second_video_track_param.codec_param.avc.idr_interval = 1;
    second_video_track_param.codec_param.avc.bitrate = 12000000;
    second_video_track_param.codec_param.avc.profile = AVCProfileType::kBaseline;
    second_video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
    second_video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kMaxBitrate;
    second_video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 51;
    second_video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
    second_video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 26;
    second_video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    second_video_track_param.codec_param.avc.ltr_count = 0;
    second_video_track_param.codec_param.avc.insert_aud_delimiter = true;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        second_video_track_param.format_type,
        video_track_id_480p,
        second_video_track_param.width,
        second_video_track_param.height
      };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb2;
    video_track_cb2.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers);
    };

    video_track_cb2.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam extra_param_2;
    SourceVideoTrack surface_video_copy_2;
    surface_video_copy_2.source_track_id = video_track_id_4k;
    extra_param_2.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy_2);

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p,
                                    second_video_track_param, extra_param_2,
                                    video_track_cb2);
    assert(ret == NO_ERROR);

    //Third Track
    uint32_t yuv_track_id_720p  = 3;

    VideoTrackCreateParam video_track_param;
    memset(&video_track_param, 0x0, sizeof video_track_param);
    video_track_param.camera_id = multicam_id_;
    video_track_param.width = 1440;
    video_track_param.height = 720;
    video_track_param.frame_rate = 24;
    video_track_param.format_type = VideoFormat::kYUV;
    video_track_param.low_power_mode = true;

    TrackCb yuv_track_cb3;
    yuv_track_cb3.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<MetaData> meta_buffers) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers);
    };

    yuv_track_cb3.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, yuv_track_id_720p,
                                     video_track_param, yuv_track_cb3);
    assert(ret == NO_ERROR);

    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    // Turn TNR On (High Quality)
    uint8_t swtnr_enable = 2;
    ret = meta.update(ANDROID_NOISE_REDUCTION_MODE, &swtnr_enable, 1);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    OverlayParam object_params;

    char *image_buffer1;
    char *image_buffer2;
    char *image_buffer3;
    char *image_buffer4;
    char *image_buffer5;

    // 1. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 100;
    object_params.dst_rect.start_y = 100;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer1 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_1;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_1);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_1);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_1);

    // 2. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 600;
    object_params.dst_rect.start_y = 600;
    object_params.dst_rect.width = 1334;
    object_params.dst_rect.height = 64;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 1334;
    object_params.image_info.source_rect.height = 64;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer2 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_2;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_2);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_2);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_2);

    // 3. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 400;
    object_params.dst_rect.start_y = 1250;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 320;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 320;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);
    image_buffer3 = object_params.image_info.image_buffer;

    uint32_t usertxt_blob_id_3;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_3);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_3);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_3);

    // 4. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 3000;
    object_params.dst_rect.start_y = 1200;
    object_params.dst_rect.width = 128;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 128;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer4 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_4;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_4);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_4);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_4);

    // 5. Create buffer blob type overlay.
    memset(&object_params, 0x0, sizeof object_params);
    object_params.type = OverlayType::kStaticImage;
    object_params.location = OverlayLocationType::kRandom;
    object_params.image_info.image_type = OverlayImageType::kBlobType;
    object_params.dst_rect.start_x = 2200;
    object_params.dst_rect.start_y = 1600;
    object_params.dst_rect.width = 960;
    object_params.dst_rect.height = 128;

    object_params.image_info.source_rect.start_x = 0;
    object_params.image_info.source_rect.start_y = 0;
    object_params.image_info.source_rect.width = 960;
    object_params.image_info.source_rect.height = 128;
    object_params.image_info.buffer_updated = false;

    object_params.image_info.image_size =
        (object_params.image_info.source_rect.width *
         object_params.image_info.source_rect.height * 4);
    object_params.image_info.image_buffer = reinterpret_cast<char *>(
        malloc(sizeof(char) * object_params.image_info.image_size));
    image_buffer5 = object_params.image_info.image_buffer;

    DrawOverlay(object_params.image_info.image_buffer,
                object_params.dst_rect.width, object_params.dst_rect.height);

    uint32_t usertxt_blob_id_5;
    ret = recorder_.CreateOverlayObject(video_track_id_4k, object_params,
                                        &usertxt_blob_id_5);
    assert(ret == 0);

    ret = recorder_.SetOverlay(video_track_id_4k, usertxt_blob_id_5);
    assert(ret == 0);
    // One track can have multiple types of overlay.
    overlay_ids_.push_back(usertxt_blob_id_5);

    // Let session run for time record_duration_.
    sleep(record_duration_);

    // Remove all overlays
    for (auto overlay_id : overlay_ids_) {
      ret = recorder_.RemoveOverlay(video_track_id_4k, overlay_id);
      assert(ret == 0);
      ret = recorder_.DeleteOverlayObject(video_track_id_4k, overlay_id);
      assert(ret == 0);
    }
    overlay_ids_.clear();

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, yuv_track_id_720p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();

    free(image_buffer1);
    free(image_buffer2);
    free(image_buffer3);
    free(image_buffer4);
    free(image_buffer5);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched720pYUVSessionAnd4KEncSession:
*     This case will test a MultiCamera with 2 sessions, one 1440x720 YUV track
*     and one 3840x1920 h264 encoded track, both at 30fps and configured to
*     produce stitched frames.
*     Both sessions are created at the beginning but the second is Start/Stop
*     while the first is running.
*
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   - CreateSession 1
*   - CreateVideoTrack 1
*   - CreateSession 2
*   - CreateVideoTrack 2
*   - StartSession 1
*   loop Start {
*   ------------------
*   - StartSession 2
*   - StopSession 2
*   ------------------
*   } loop End
*   - StopSession 1
*   - DeleteVideoTrack 2
*   - DeleteSession 2
*   - DeleteVideoTrack 1
*   - DeleteSession 1
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched720pYUVSessionAnd4KEncSession) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width = 3840;
  uint32_t stream_height = 1920;
  float stream_fps = 30;
  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t video_track_id_720p = 1;
  uint32_t video_track_id_4k = 2;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id_720p;
  ret = recorder_.CreateSession(session_status_cb, &session_id_720p);
  assert(session_id_720p > 0);
  assert(ret == NO_ERROR);
  VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                          stream_width,
                                          stream_height,
                                          stream_fps };

  stream_width  = 1440;
  stream_height = 720;

  video_track_param.width         = stream_width;
  video_track_param.height        = stream_height;
  video_track_param.frame_rate    = stream_fps;
  video_track_param.format_type   = VideoFormat::kYUV;

  TrackCb video_track_cb;
  video_track_cb.data_cb = [&, session_id_720p] (uint32_t track_id,
                            std::vector<BufferDescriptor> buffers,
                            std::vector<MetaData> meta_buffers) {
    VideoTrackYUVDataCb(session_id_720p, track_id, buffers, meta_buffers); };

  video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
      void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
      event_type, event_data, event_data_size); };

  ret = recorder_.CreateVideoTrack(session_id_720p, video_track_id_720p,
                                   video_track_param, video_track_cb);
  assert(ret == NO_ERROR);

  uint32_t session_id_4k;
  ret = recorder_.CreateSession(session_status_cb, &session_id_4k);
  assert(session_id_4k > 0);
  assert(ret == NO_ERROR);

  // Set parameters for and create 3840x1920 h264 encodded track.
  stream_width  = 3840;
  stream_height = 1920;

  video_track_param.width         = stream_width;
  video_track_param.height        = stream_height;
  video_track_param.format_type   = VideoFormat::kAVC;
  // Set media profiles
  video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
  video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

  if (dump_bitstream_.IsEnabled()) {
    Stream360DumpInfo dumpinfo = {
      video_track_param.format_type,
      video_track_id_4k,
      stream_width,
      stream_height };
    ret = dump_bitstream_.SetUp(dumpinfo);
    assert(ret == NO_ERROR);
  }

  video_track_cb.data_cb = [&, session_id_4k] (uint32_t track_id,
                            std::vector<BufferDescriptor> buffers,
                            std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id_4k, track_id, buffers, meta_buffers); };

  video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
      void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
      event_type, event_data, event_data_size); };

  ret = recorder_.CreateVideoTrack(session_id_4k, video_track_id_4k,
                                   video_track_param, video_track_cb);
  assert(ret == NO_ERROR);

  ret = recorder_.StartSession(session_id_720p);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    ret = recorder_.StartSession(session_id_4k);
    assert(ret == NO_ERROR);

    // Let session run for record_duration_, during this time buffer with valid
    // data would be received in track callback (VideoTrackDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id_4k, false);
    assert(ret == NO_ERROR);
  }
  dump_bitstream_.CloseAll();

  sleep(record_duration_);

  ret = recorder_.StopSession(session_id_720p, false);
  assert(ret == NO_ERROR);

  ret = recorder_.DeleteVideoTrack(session_id_4k, video_track_id_4k);
  assert(ret == NO_ERROR);

  ret = recorder_.DeleteSession(session_id_4k);
  assert(ret == NO_ERROR);

  ret = recorder_.DeleteVideoTrack(session_id_720p, video_track_id_720p);
  assert(ret == NO_ERROR);

  ret = recorder_.DeleteSession(session_id_720p);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* Stitched720pYUVSessionAnd4KEncSessionAtRunTime:
*     This case will test a MultiCamera with 2 sessions, one 1440x720 YUV track
*     and one 3840x1920 h264 encoded track, both at 30fps and configured to
*     produce stitched frames.
*     The second session is created while the first is still running.
*
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   - CreateSession 1
*   - CreateVideoTrack 1
*   - StartSession 1
*   loop Start {
*   ------------------
*   - CreateSession 2
*   - CreateVideoTrack 2
*   - StartSession 2
*   - StopSession 2
*   - DeleteVideoTrack 2
*   - DeleteSession 2
*   ------------------
*   } loop End
*   - StopSession 1
*   - DeleteVideoTrack 1
*   - DeleteSession 1
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched720pYUVSessionAnd4KEncSessionAtRunTime) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;
  float stream_fps = 30;
  uint32_t video_track_id_720p = 1;
  uint32_t video_track_id_4k = 2;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id_720p;
  ret = recorder_.CreateSession(session_status_cb, &session_id_720p);
  assert(session_id_720p > 0);
  assert(ret == NO_ERROR);


  stream_width  = 1440;
  stream_height = 720;
  VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kAVC,
                                          stream_width,
                                          stream_height,
                                          stream_fps };
  video_track_param.format_type   = VideoFormat::kYUV;

  TrackCb video_track_cb;
  video_track_cb.data_cb = [&, session_id_720p] (uint32_t track_id,
                            std::vector<BufferDescriptor> buffers,
                            std::vector<MetaData> meta_buffers) {
    VideoTrackYUVDataCb(session_id_720p, track_id, buffers, meta_buffers); };

  video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
      void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
      event_type, event_data, event_data_size); };

  ret = recorder_.CreateVideoTrack(session_id_720p, video_track_id_720p,
                                   video_track_param, video_track_cb);
  assert(ret == NO_ERROR);

  ret = recorder_.StartSession(session_id_720p);
  assert(ret == NO_ERROR);

  // Set parameters for and create 3840x1920 h264 encodded track.
  stream_width  = 3840;
  stream_height = 1920;

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    uint32_t session_id_4k;
    ret = recorder_.CreateSession(session_status_cb, &session_id_4k);
    assert(session_id_4k > 0);
    assert(ret == NO_ERROR);

    video_track_param.width         = stream_width;
    video_track_param.height        = stream_height;
    video_track_param.format_type   = VideoFormat::kAVC;
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id_4k,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id_4k] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
      VideoTrackOneEncDataCb(session_id_4k, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id_4k, video_track_id_4k,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id_4k);
    assert(ret == NO_ERROR);

    // Let session run for record_duration_, during this time buffer with valid
    // data would be received in track callback (VideoTrackDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id_4k, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id_4k, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id_4k);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  sleep(record_duration_);

  ret = recorder_.StopSession(session_id_720p, false);
  assert(ret == NO_ERROR);

  ret = recorder_.DeleteVideoTrack(session_id_720p, video_track_id_720p);
  assert(ret == NO_ERROR);

  ret = recorder_.DeleteSession(session_id_720p);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySide6KSnapshot: This case will test a MultiCamera capture for
*                       side-by-side 6K JPEG snapshot.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CaptureImage - JPEG
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySide6KSnapshot) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  ImageParam image_param;
  memset(&image_param, 0x0, sizeof image_param);
  image_param.width         = 6080;
  image_param.height        = 3040;
  image_param.image_format  = ImageFormat::kJPEG;
  image_param.image_quality = 95;

  std::vector<CameraMetadata> meta_array;
  CameraMetadata meta;

  ret = recorder_.GetDefaultCaptureParam(multicam_id_, meta);
  assert(ret == NO_ERROR);

  meta_array.push_back(meta);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    ImageCaptureCb cb = [this] (uint32_t camera_id, uint32_t image_count,
                                BufferDescriptor buffer,
                                MetaData meta_data) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta_data); };

    ret = recorder_.CaptureImage(multicam_id_, image_param, 1, meta_array, cb);
    assert(ret == NO_ERROR);
    // Take snapshot after every 5 sec.
    sleep(kDelayAfterSnapshot);
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySide4KSnapshot: This case will test a MultiCamera capture for
*                       side-by-side 4K JPEG snapshot.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CaptureImage - JPEG
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySide4KSnapshot) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  ImageParam image_param;
  memset(&image_param, 0x0, sizeof image_param);
  image_param.width         = 3840;
  image_param.height        = 1920;
  image_param.image_format  = ImageFormat::kJPEG;
  image_param.image_quality = 95;

  std::vector<CameraMetadata> meta_array;
  CameraMetadata meta;

  ret = recorder_.GetDefaultCaptureParam(multicam_id_, meta);
  assert(ret == NO_ERROR);

  meta_array.push_back(meta);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    ImageCaptureCb cb = [this] (uint32_t camera_id, uint32_t image_count,
                                BufferDescriptor buffer,
                                MetaData meta_data) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta_data); };

    ret = recorder_.CaptureImage(multicam_id_, image_param, 1, meta_array, cb);
    assert(ret == NO_ERROR);
    // Take snapshot after every 5 sec.
    sleep(kDelayAfterSnapshot);
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySideHDSnapshot: This case will test a MultiCamera capture for
*                       side-by-side HD JPEG snapshot.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CaptureImage - JPEG
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySideHDSnapshot) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  ImageParam image_param;
  memset(&image_param, 0x0, sizeof image_param);
  image_param.width         = 1920;
  image_param.height        = 960;
  image_param.image_format  = ImageFormat::kJPEG;
  image_param.image_quality = 95;

  std::vector<CameraMetadata> meta_array;
  CameraMetadata meta;

  ret = recorder_.GetDefaultCaptureParam(multicam_id_, meta);
  assert(ret == NO_ERROR);

  meta_array.push_back(meta);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    ImageCaptureCb cb = [this] (uint32_t camera_id, uint32_t image_count,
                                BufferDescriptor buffer,
                                MetaData meta_data) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta_data); };

    ret = recorder_.CaptureImage(multicam_id_, image_param, 1, meta_array, cb);
    assert(ret == NO_ERROR);
    // Take snapshot after every 5 sec.
    sleep(kDelayAfterSnapshot);
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySide720pSnapshot: This case will test a MultiCamera capture for
*                         side-by-side 720p JPEG snapshot.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CaptureImage - JPEG
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySide720pSnapshot) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  ImageParam image_param;
  memset(&image_param, 0x0, sizeof image_param);
  image_param.width         = 1440;
  image_param.height        = 720;
  image_param.image_format  = ImageFormat::kJPEG;
  image_param.image_quality = 95;

  std::vector<CameraMetadata> meta_array;
  CameraMetadata meta;

  ret = recorder_.GetDefaultCaptureParam(multicam_id_, meta);
  assert(ret == NO_ERROR);

  meta_array.push_back(meta);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    ImageCaptureCb cb = [this] (uint32_t camera_id, uint32_t image_count,
                                BufferDescriptor buffer,
                                MetaData meta_data) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta_data); };

    ret = recorder_.CaptureImage(multicam_id_, image_param, 1, meta_array, cb);
    assert(ret == NO_ERROR);
    // Take snapshot after every 5 sec.
    sleep(kDelayAfterSnapshot);
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySide4KYUVTrack:  This case will test a MultiCamera session with one
*                        4K YUV track and configured to produce side-by-side
*                        frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySide4KYUVTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);

    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                            3840,
                                            1920,
                                            30};

    uint32_t video_track_id = 1;

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for time record_duration_, during this time buffer with
    // valid data would be received in track callback (VideoTrackYUVDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySideHDYUVTrack:  This case will test a MultiCamera session with one
*                        Full HD YUV track and configured to produce side-by-side
*                        frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySideHDYUVTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                            1920,
                                            960,
                                            30};
    uint32_t video_track_id = 1;

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for time record_duration_, during this time buffer with
    // valid data would be received in track callback (VideoTrackYUVDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySide720pYUVTrack: This case will test a MultiCamera session with one
*                         720p YUV track and configured to produce side-by-side
*                         frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySide720pYUVTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                            1440,
                                            720,
                                            30};

    uint32_t video_track_id = 1;

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for time record_duration_, during this time buffer with
    // valid data would be received in track callback (VideoTrackYUVDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySide4KAndFullHDYUVTrack: This case will test a MultiCamera session with
*                                one 4K and one Full HD YUV tracks, configured
*                                to produce side-by-side frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   ------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   ------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySide4KAndFullHDYUVTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);
  uint32_t track_4k_id = 1;
  uint32_t track_fullhd_id = 2;

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                            3840,
                                            1920,
                                            30};

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, track_4k_id,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    video_track_param.width  = 1920;
    video_track_param.height = 960;

    ret = recorder_.CreateVideoTrack(session_id, track_fullhd_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for time record_duration_, during this time buffer with
    // valid data would be received in track callback (VideoTrackYUVDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, track_4k_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, track_fullhd_id);
    assert(ret == NO_ERROR);
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySide4KEncTrack: This test will test session with 3840x1920 h264 encoded
*                       track and configured to produce side-by-side frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySide4KEncTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
        // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for record_duration_, during this time buffer with valid
    // data would be received in track callback (VideoTrackDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }
  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySideHDEncTrack: This test will test session with 1920x960 h264 encoded
*                       track and configured to produce side-by-side frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySideHDEncTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 1920;
  uint32_t stream_height = 960;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel4;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for record_duration_, during this time buffer with valid
    // data would be received in track callback (VideoTrackDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }
  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySide720pEncTrack: This test will test session with 1440x720 h264 encoded
*                         track and configured to produce side-by-side frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySide720pEncTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 1440;
  uint32_t stream_height = 720;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel4;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for record_duration_, during this time buffer with valid
    // data would be received in track callback (VideoTrackDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }
  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySide720p120fpsEncTrack: This test will test session with 1440x720 h264
*                         encoded track at 120 fps and configured to produce
*                         side-by-side frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySide720p120fpsEncTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 1440;
  uint32_t stream_height = 720;
  float fps = 120;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;
  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            fps};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel4;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for record_duration_, during this time buffer with valid
    // data would be received in track callback (VideoTrackDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }
  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySide4KAnd720pEncTrack: This case will test a MultiCamera session with
*                              one 3840x1920 and one 1440x720 h264 encoded
*                              tracks, configured to produce side-by-side frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*   loop Start {
*   --------------------
*   - CreateVideoTrack 1
*   - CreateVideoTrack 2
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack 1
*   - DeleteVideoTrack 2
*   --------------------
*   } loop End
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySide4KAnd720pEncTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;
  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  uint32_t video_track_id_720p = 2;
  VideoFormat format_type = VideoFormat::kAVC;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;
  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  multicam_start_params_.frame_rate = stream_fps;
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb =
      [this] (EventType event_type, void *event_data,
              size_t event_data_size) -> void { SessionCallbackHandler(event_type,
      event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);
  VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                          3840,
                                          1920,
                                          stream_fps};
  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    // Set parameters for and create 3840x1920 h264 encodded track.
    stream_width  = 3840;
    stream_height = 1920;

    video_track_param.width       = stream_width;
    video_track_param.height      = stream_height;
     // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id_4k,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb =
        [this] (uint32_t track_id, EventType event_type,
                void *event_data, size_t event_data_size) -> void
        { VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    // Set parameters for and create 1440x720 h264 encodded track.
    stream_width  = 1440;
    stream_height = 720;

    video_track_param.width       = stream_width;
    video_track_param.height      = stream_height;
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel4;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id_720p,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackTwoEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb =
        [this] (uint32_t track_id, EventType event_type,
                void *event_data, size_t event_data_size) -> void
        { VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_720p,
                                     video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_720p);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySide4KUHDYUVTrackWithMaxFOV:
*     This case will test a MultiCamera session with one 4K Ultra HD YUV track
*     and configured to produce side-by-side frames.
*
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySide4KUHDEncTrackWithMaxFOV) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);
  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 2160;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);

    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 2160;
      source_surface.height = 2160;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                     video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for record_duration_, during this time buffer with valid
    // data would be received in track callback (VideoTrackDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }
  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySide4KUHDYUVTrackWithMaxPPD:
*     This case will test a MultiCamera session with one 4K Ultra HD YUV track
*     and configured to produce side-by-side frames.
*
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySide4KUHDYUVTrackWithMaxPPD) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);

    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kYUV,
                                            3840,
                                            2160,
                                            30};
    uint32_t video_track_id = 1;
    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 2160;
      source_surface.height = 2160;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                     video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for record_duration_, during this time buffer with valid
    // data would be received in track callback (VideoTrackDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }
  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySide4KUHDEncTrackWithMaxPPD:
*     This case will test a MultiCamera session with one 4K Ultra HD YUV track
*     and configured to produce side-by-side frames.
*
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySide4KUHDEncTrackWithMaxPPD) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 2160;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);

    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1920;
      source_surface.height = 2160;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                     video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    int32_t sensor_mode_width = 3040, sensor_mode_height = 3040;
    int32_t crop_x = 560, crop_y = 440, crop_w = 1920, crop_h = 2160;

    ret = FillCropMetadata(meta, sensor_mode_width, sensor_mode_height,
                           crop_x, crop_y, crop_w, crop_h);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for record_duration_, during this time buffer with valid
    // data would be received in track callback (VideoTrackDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }
  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySide4KUHDEncMaxFOVAndSingleWXGAYUVTrack:
*     This case will test a MultiCamera session with one 4K Ultra HD h264
*     encoded track configured to produce side-by-side frames and one 1280x640
*     YUV track configured to output the frames from only one of the cameras.
*
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySide4KUHDEncMaxFOVAndSingleWXGAYUVTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;
  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  uint32_t video_track_id_wxga = 2;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);

    stream_width  = 3840;
    stream_height = 2160;

    VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kAVC,
                                            stream_width,
                                            stream_height,
                                            stream_fps};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id_4k,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb =
        [this] (uint32_t track_id, EventType event_type,
                void *event_data, size_t event_data_size) -> void
        { VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 2160;
      source_surface.height = 2160;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    stream_width  = 1280;
    stream_height = 640;

    memset(&video_track_param, 0x0, sizeof video_track_param);
    video_track_param.camera_id     = multicam_id_;
    video_track_param.width         = stream_width;
    video_track_param.height        = stream_height;
    video_track_param.frame_rate    = stream_fps;
    video_track_param.format_type   = VideoFormat::kYUV;
    // Set media profiles
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    extra_param.Clear();
    SurfaceCrop surface_crop;
    surface_crop.camera_id = camera_ids_.at(1);
    extra_param.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_crop);

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_wxga,
                                     video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for time record_duration_, during this time buffer with
    // valid data would be received in track callback (VideoTrackYUVDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_wxga);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* SideBySide4KUHDEncMaxPPDAndSingleWXGAYUVTrack:
*     This case will test a MultiCamera session with one 4K Ultra HD h264
*     encoded track configured to produce side-by-side frames and one 1280x640
*     YUV track configured to output the frames from only one of the cameras.
*
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, SideBySide4KUHDEncMaxPPDAndSingleWXGAYUVTrack) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  uint32_t stream_width;
  uint32_t stream_height;
  float stream_fps = 30;
  uint32_t video_track_id_4k = 1;
  uint32_t video_track_id_wxga = 2;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  multicam_type_ = MultiCameraConfigType::kSideBySide;

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  VideoTrackCreateParam video_track_param{multicam_id_, VideoFormat::kAVC,
                                          3840,
                                          2160,
                                          stream_fps};

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);

    stream_width  = 3840;
    stream_height = 2160;
    video_track_param.width       = stream_width;
    video_track_param.height      = stream_height;
    video_track_param.format_type = VideoFormat::kAVC;
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id_4k,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb =
        [this] (uint32_t track_id, EventType event_type,
                void *event_data, size_t event_data_size) -> void
        { VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    VideoExtraParam extra_param;
    for (size_t i = 0; i < camera_ids_.size(); ++i) {
      SourceSurfaceDesc source_surface;
      source_surface.camera_id = camera_ids_.at(i);
      source_surface.width = 1920;
      source_surface.height = 2160;
      source_surface.flags = TransformFlags::kNone;
      extra_param.Update(QMMF_SOURCE_SURFACE_DESCRIPTOR, source_surface, i);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_4k,
                                     video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    stream_width  = 1280;
    stream_height = 640;

    video_track_param.width         = stream_width;
    video_track_param.height        = stream_height;
    video_track_param.format_type   = VideoFormat::kYUV;

    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                                  std::vector<BufferDescriptor> buffers,
                                  std::vector<MetaData> meta_buffers) {
        VideoTrackYUVDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    extra_param.Clear();
    SurfaceCrop surface_crop;
    surface_crop.camera_id = camera_ids_.at(1);
    extra_param.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_crop);

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_wxga,
                                     video_track_param, extra_param,
                                     video_track_cb);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    int32_t sensor_mode_width = 3040, sensor_mode_height = 3040;
    int32_t crop_x = 560, crop_y = 440, crop_w = 1920, crop_h = 2160;

    ret = FillCropMetadata(meta, sensor_mode_width, sensor_mode_height,
                           crop_x, crop_y, crop_w, crop_h);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    // Let session run for time record_duration_, during this time buffer with
    // valid data would be received in track callback (VideoTrackYUVDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_wxga);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_4k);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());

}

/*
* Stitched4KEncAllAWBModes: This case will test a MultiCamera session with 3840x1920
*                 h264 encoded track, during which in 10 sec interval time AWB modes
*                 will change. This test is configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set AWB mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncAllAWBModes) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  uint8_t WbModes[] = {
    ANDROID_CONTROL_AWB_MODE_AUTO,
    ANDROID_CONTROL_AWB_MODE_INCANDESCENT,
    ANDROID_CONTROL_AWB_MODE_FLUORESCENT,
    ANDROID_CONTROL_AWB_MODE_WARM_FLUORESCENT,
    ANDROID_CONTROL_AWB_MODE_DAYLIGHT,
    ANDROID_CONTROL_AWB_MODE_CLOUDY_DAYLIGHT,
    ANDROID_CONTROL_AWB_MODE_TWILIGHT,
    ANDROID_CONTROL_AWB_MODE_SHADE,
  };
  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);

    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    for (auto p : WbModes) {
      uint8_t awb_mode;
      switch (p) {
        case ANDROID_CONTROL_AWB_MODE_AUTO:
          awb_mode = ANDROID_CONTROL_AWB_MODE_AUTO;
          ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
          assert(ret == NO_ERROR);
          fprintf(stderr, "WB mode switched to "
              "ANDROID_CONTROL_AWB_MODE_AUTO\n");
          break;
        case ANDROID_CONTROL_AWB_MODE_INCANDESCENT:
          awb_mode = ANDROID_CONTROL_AWB_MODE_INCANDESCENT;
          ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
          assert(ret == NO_ERROR);
          fprintf(stderr, "WB mode switched to "
              "ANDROID_CONTROL_AWB_MODE_INCANDESCENT\n");
          break;
        case ANDROID_CONTROL_AWB_MODE_FLUORESCENT:
          awb_mode = ANDROID_CONTROL_AWB_MODE_FLUORESCENT;
          ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
          assert(ret == NO_ERROR);
          fprintf(stderr, "WB mode switched to "
              "ANDROID_CONTROL_AWB_MODE_FLUORESCENT\n");
          break;
        case ANDROID_CONTROL_AWB_MODE_WARM_FLUORESCENT:
          awb_mode = ANDROID_CONTROL_AWB_MODE_WARM_FLUORESCENT;
          ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
          assert(ret == NO_ERROR);
          fprintf(stderr, "WB mode switched to "
              "ANDROID_CONTROL_AWB_MODE_WARM_FLUORESCENT\n");
          break;
        case ANDROID_CONTROL_AWB_MODE_DAYLIGHT:
          awb_mode = ANDROID_CONTROL_AWB_MODE_DAYLIGHT;
          ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
          assert(ret == NO_ERROR);
          fprintf(stderr, "WB mode switched to "
              "ANDROID_CONTROL_AWB_MODE_DAYLIGHT\n");
          break;
        case ANDROID_CONTROL_AWB_MODE_CLOUDY_DAYLIGHT:
          awb_mode = ANDROID_CONTROL_AWB_MODE_CLOUDY_DAYLIGHT;
          ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
          assert(ret == NO_ERROR);
          fprintf(stderr, "WB mode switched to "
              "ANDROID_CONTROL_AWB_MODE_CLOUDY_DAYLIGHT\n");
          break;
        case ANDROID_CONTROL_AWB_MODE_TWILIGHT:
          awb_mode = ANDROID_CONTROL_AWB_MODE_TWILIGHT;
          ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
          assert(ret == NO_ERROR);
          fprintf(stderr, "WB mode switched to "
              "ANDROID_CONTROL_AWB_MODE_TWILIGHT\n");
          break;
        case ANDROID_CONTROL_AWB_MODE_SHADE:
          awb_mode = ANDROID_CONTROL_AWB_MODE_SHADE;
          ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
          assert(ret == NO_ERROR);
          fprintf(stderr, "WB mode switched to "
              "ANDROID_CONTROL_AWB_MODE_SHADE\n");
          break;
        default:
          assert(0);
      }

      ret = recorder_.SetCameraParam(multicam_id_, meta);
      assert(ret == NO_ERROR);
      sleep(kDelayAfterSnapshot);
    }
    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncAWBModeAuto: This case will test a MultiCamera session with
*                  3840x1920 h264 encoded track with AWB_MODE_AUTO. This test is
*                  configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set AWB mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncAWBModeAuto) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);

    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};

    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    uint8_t awb_mode;
    awb_mode = ANDROID_CONTROL_AWB_MODE_AUTO;
    ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
    assert(ret == NO_ERROR);
    fprintf(stderr, "WB mode switched to "
        "ANDROID_CONTROL_AWB_MODE_AUTO\n");

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncAWBModeIncandescent: This case will test a MultiCamera session with
*                          3840x1920 h264 encoded track with AWB_MODE_INCANDESCENT.
*                          This test is configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set AWB mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncAWBModeIncandescent) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    uint8_t awb_mode;
    awb_mode = ANDROID_CONTROL_AWB_MODE_INCANDESCENT;
    ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
    assert(ret == NO_ERROR);
    fprintf(stderr, "WB mode switched to "
        "ANDROID_CONTROL_AWB_MODE_INCANDESCENT\n");

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncAWBModeFluorescent: This case will test a MultiCamera session with
*                         3840x1920 h264 encoded track with AWB_MODE_FLUORESCENT.
*                         This test is configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set AWB mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncAWBModeFluorescent) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    uint8_t awb_mode;
    awb_mode = ANDROID_CONTROL_AWB_MODE_FLUORESCENT;
    ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
    assert(ret == NO_ERROR);
    fprintf(stderr, "WB mode switched to "
        "ANDROID_CONTROL_AWB_MODE_FLUORESCENT\n");

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncAWBModeWarmFluorescent: This case will test a MultiCamera session with
*                             3840x1920 h264 encoded track with AWB_MODE_WARM_FLUORESCENT.
*                             This test is configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set AWB mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncAWBModeWarmFluorescent) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);

    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    uint8_t awb_mode;
    awb_mode = ANDROID_CONTROL_AWB_MODE_WARM_FLUORESCENT;
    ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
    assert(ret == NO_ERROR);
    fprintf(stderr, "WB mode switched to "
        "ANDROID_CONTROL_AWB_MODE_WARM_FLUORESCENT\n");

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncAWBModeDaylight: This case will test a MultiCamera session with
*                      3840x1920 h264 encoded track with AWB_MODE_DAYLIGHT. This
*                      test is configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set AWB mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncAWBModeDaylight) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);

    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    uint8_t awb_mode;
    awb_mode = ANDROID_CONTROL_AWB_MODE_DAYLIGHT;
    ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
    assert(ret == NO_ERROR);
    fprintf(stderr, "WB mode switched to "
        "ANDROID_CONTROL_AWB_MODE_DAYLIGHT\n");

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncAWBModeCloudyDaylight: This case will test a MultiCamera session with
*                            3840x1920 h264 encoded track with AWB_MODE_CLOUDY_DAYLIGHT.
*                            This test is configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set AWB mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncAWBModeCloudyDaylight) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    uint8_t awb_mode;
    awb_mode = ANDROID_CONTROL_AWB_MODE_CLOUDY_DAYLIGHT;
    ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
    assert(ret == NO_ERROR);
    fprintf(stderr, "WB mode switched to "
        "ANDROID_CONTROL_AWB_MODE_CLOUDY_DAYLIGHT\n");

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncAWBModeTwilight: This case will test a MultiCamera session with
*                      3840x1920 h264 encoded track with AWB_MODE_TWILIGHT. This
*                      test is configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set AWB mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncAWBModeTwilight) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    uint8_t awb_mode;
    awb_mode = ANDROID_CONTROL_AWB_MODE_TWILIGHT;
    ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
    assert(ret == NO_ERROR);
    fprintf(stderr, "WB mode switched to "
        "ANDROID_CONTROL_AWB_MODE_TWILIGHT\n");

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncAWBModeShade: This case will test a MultiCamera session with
*                   3840x1920 h264 encoded track with AWB_MODE_SHADE. This test
*                   is configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set AWB mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncAWBModeShade) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    uint8_t awb_mode;
    awb_mode = ANDROID_CONTROL_AWB_MODE_SHADE;
    ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
    assert(ret == NO_ERROR);
    fprintf(stderr, "WB mode switched to "
        "ANDROID_CONTROL_AWB_MODE_SHADE\n");

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncAllAEAntiBandingModes: This case will test a MultiCamera session
*                           with 3840x1920 h264 encoded track, during which in 10
*                           sec interval time AE anti-banding modes will change.
*                           This test is configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   --------------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set AE anti-banding mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   --------------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncAllAEAntiBandingModes) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  uint8_t AEAntiBandingModes[] = {
      ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF,
      ANDROID_CONTROL_AE_ANTIBANDING_MODE_50HZ,
      ANDROID_CONTROL_AE_ANTIBANDING_MODE_60HZ,
      ANDROID_CONTROL_AE_ANTIBANDING_MODE_AUTO,
  };

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    for (auto p : AEAntiBandingModes) {
      uint8_t ae_mode;
      switch (p) {
        case ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF:
          ae_mode = ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF;
          ret = meta.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &ae_mode, 1);
          assert(ret == NO_ERROR);
          fprintf(stderr, "AE mode switched to "
              "ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF\n");
          break;
        case ANDROID_CONTROL_AE_ANTIBANDING_MODE_50HZ:
          ae_mode = ANDROID_CONTROL_AE_ANTIBANDING_MODE_50HZ;
          ret = meta.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &ae_mode, 1);
          assert(ret == NO_ERROR);
          fprintf(stderr, "AE mode switched to "
              "ANDROID_CONTROL_AE_ANTIBANDING_MODE_50HZ\n");
          break;
        case ANDROID_CONTROL_AE_ANTIBANDING_MODE_60HZ:
          ae_mode = ANDROID_CONTROL_AE_ANTIBANDING_MODE_60HZ;
          ret = meta.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &ae_mode, 1);
          assert(ret == NO_ERROR);
          fprintf(stderr, "AE mode switched to "
              "ANDROID_CONTROL_AE_ANTIBANDING_MODE_60HZ\n");
          break;
        case ANDROID_CONTROL_AE_ANTIBANDING_MODE_AUTO:
          ae_mode = ANDROID_CONTROL_AE_ANTIBANDING_MODE_AUTO;
          ret = meta.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &ae_mode, 1);
          assert(ret == NO_ERROR);
          fprintf(stderr, "AE mode switched to "
              "ANDROID_CONTROL_AE_ANTIBANDING_MODE_AUTO\n");
          break;
        default:
          assert(0);
      }

      ret = recorder_.SetCameraParam(multicam_id_, meta);
      assert(ret == NO_ERROR);
      sleep(kDelayAfterSnapshot);
    }
    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncAEAntiBandingModeOff: This case will test a MultiCamera session with
*                           3840x1920 h264 encoded track with AE_ANTIBANDING_MODE_OFF.
*                           This test is configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set AE anti-banding mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncAEAntiBandingModeOff) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    uint8_t ae_mode;
    ae_mode = ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF;
    ret = meta.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &ae_mode, 1);
    assert(ret == NO_ERROR);
    fprintf(stderr, "AE mode switched to "
        "ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF\n");

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncAEAntiBandingMode50Hz: This case will test a MultiCamera session with
*                            3840x1920 h264 encoded track withAE_ANTIBANDING_MODE_50HZ.
*                            This test is configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set AE anti-banding mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncAEAntiBandingMode50Hz) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

 ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    uint8_t ae_mode;
    ae_mode = ANDROID_CONTROL_AE_ANTIBANDING_MODE_50HZ;
    ret = meta.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &ae_mode, 1);
    assert(ret == NO_ERROR);
    fprintf(stderr, "AE mode switched to "
        "ANDROID_CONTROL_AE_ANTIBANDING_MODE_50HZ\n");

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncAEAntiBandingMode60Hz: This case will test a MultiCamera session with
*                            3840x1920 h264 encoded track with AE_ANTIBANDING_MODE_60HZ.
*                            This test is configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set AE anti-banding mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncAEAntiBandingMode60Hz) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    uint8_t ae_mode;
    ae_mode = ANDROID_CONTROL_AE_ANTIBANDING_MODE_60HZ;
    ret = meta.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &ae_mode, 1);
    assert(ret == NO_ERROR);
    fprintf(stderr, "AE mode switched to "
        "ANDROID_CONTROL_AE_ANTIBANDING_MODE_60HZ\n");

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncAEAntiBandingModeAuto: This case will test a MultiCamera session with
*                            3840x1920 h264 encoded track with AE_ANTIBANDING_MODE_AUTO.
*                            This test is configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set AE anti-banding mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncAEAntiBandingModeAuto) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    uint8_t ae_mode;
    ae_mode = ANDROID_CONTROL_AE_ANTIBANDING_MODE_AUTO;
    ret = meta.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &ae_mode, 1);
    assert(ret == NO_ERROR);
    fprintf(stderr, "AE mode switched to "
        "ANDROID_CONTROL_AE_ANTIBANDING_MODE_AUTO\n");

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncAllISOModes: This case will test a MultiCamera session with 3840x1920
*                 h264 encoded track, during which in 10 sec interval time ISO modes
*                 will change. This test is configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   --------------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set ISO mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   --------------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncAllISOModes) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  uint8_t Qcam3ISOModes[] = {
      QCAMERA3_ISO_MODE_AUTO,
      QCAMERA3_ISO_MODE_100,
      QCAMERA3_ISO_MODE_200,
      QCAMERA3_ISO_MODE_400,
      QCAMERA3_ISO_MODE_800,
      QCAMERA3_ISO_MODE_1600,
      QCAMERA3_ISO_MODE_3200,
  };

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);

    int32_t use_exp_priority = 0;
    ret = meta.update(QCAMERA3_SELECT_PRIORITY, &use_exp_priority, 4);

    for (auto p : Qcam3ISOModes) {
      int64_t iso_mode;
      switch (p) {
        case QCAMERA3_ISO_MODE_AUTO:
          iso_mode = QCAMERA3_ISO_MODE_AUTO;
          ret = meta.update(QCAMERA3_USE_ISO_EXP_PRIORITY, &iso_mode, 8);
          assert(ret == NO_ERROR);
          fprintf(stderr, "ISO mode switched to QCAMERA3_ISO_MODE_AUTO\n");
          break;
        case QCAMERA3_ISO_MODE_100:
          iso_mode = QCAMERA3_ISO_MODE_100;
          ret = meta.update(QCAMERA3_USE_ISO_EXP_PRIORITY, &iso_mode, 8);
          assert(ret == NO_ERROR);
          fprintf(stderr, "ISO mode switched to QCAMERA3_ISO_MODE_100\n");
          break;
        case QCAMERA3_ISO_MODE_200:
          iso_mode = QCAMERA3_ISO_MODE_200;
          ret = meta.update(QCAMERA3_USE_ISO_EXP_PRIORITY, &iso_mode, 8);
          assert(ret == NO_ERROR);
          fprintf(stderr, "ISO mode switched to QCAMERA3_ISO_MODE_200\n");
          break;
        case QCAMERA3_ISO_MODE_400:
          iso_mode = QCAMERA3_ISO_MODE_400;
          ret = meta.update(QCAMERA3_USE_ISO_EXP_PRIORITY, &iso_mode, 8);
          assert(ret == NO_ERROR);
          fprintf(stderr, "ISO mode switched to QCAMERA3_ISO_MODE_400\n");
          break;
        case QCAMERA3_ISO_MODE_800:
          iso_mode = QCAMERA3_ISO_MODE_800;
          ret = meta.update(QCAMERA3_USE_ISO_EXP_PRIORITY, &iso_mode, 8);
          assert(ret == NO_ERROR);
          fprintf(stderr, "ISO mode switched to QCAMERA3_ISO_MODE_800\n");
          break;
        case QCAMERA3_ISO_MODE_1600:
          iso_mode = QCAMERA3_ISO_MODE_1600;
          ret = meta.update(QCAMERA3_USE_ISO_EXP_PRIORITY, &iso_mode, 8);
          assert(ret == NO_ERROR);
          fprintf(stderr, "ISO mode switched to QCAMERA3_ISO_MODE_1600\n");
          break;
        case QCAMERA3_ISO_MODE_3200:
          iso_mode = QCAMERA3_ISO_MODE_3200;
          ret = meta.update(QCAMERA3_USE_ISO_EXP_PRIORITY, &iso_mode, 8);
          assert(ret == NO_ERROR);
          fprintf(stderr, "ISO mode switched to QCAMERA3_ISO_MODE_3200\n");
          break;
        default:
          assert(0);
      }

      ret = recorder_.SetCameraParam(multicam_id_, meta);
      assert(ret == NO_ERROR);
      sleep(kDelayAfterSnapshot);
    }
    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* TestISOModeAuto: This case will test a MultiCamera session with 3840x1920 h264
*                  encoded track with ISO_MODE_AUTO. This test is configured to
*                  produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set ISO mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, TestISOModeAuto) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);

    int32_t use_exp_priority = 0;
    ret = meta.update(QCAMERA3_SELECT_PRIORITY, &use_exp_priority, 4);

    int64_t iso_mode;
    iso_mode = QCAMERA3_ISO_MODE_AUTO;
    ret = meta.update(QCAMERA3_USE_ISO_EXP_PRIORITY, &iso_mode, 8);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);
    fprintf(stderr, "ISO mode switched to QCAMERA3_ISO_MODE_AUTO\n");

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* TestISOMode100: This case will test a MultiCamera session with 3840x1920 h264
*                 encoded track with ISO_MODE_100. This test is configured to
*                 produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set ISO mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, TestISOMode100) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);

    int32_t use_exp_priority = 0;
    ret = meta.update(QCAMERA3_SELECT_PRIORITY, &use_exp_priority, 4);

    int64_t iso_mode;
    iso_mode = QCAMERA3_ISO_MODE_100;
    ret = meta.update(QCAMERA3_USE_ISO_EXP_PRIORITY, &iso_mode, 8);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);
    fprintf(stderr, "ISO mode switched to QCAMERA3_ISO_MODE_100\n");

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* TestISOMode200: This case will test a MultiCamera session with 3840x1920 h264
*                 encoded track with ISO_MODE_200. This test is configured to
*                 produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set ISO mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, TestISOMode200) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);

    int32_t use_exp_priority = 0;
    ret = meta.update(QCAMERA3_SELECT_PRIORITY, &use_exp_priority, 4);

    int64_t iso_mode;
    iso_mode = QCAMERA3_ISO_MODE_200;
    ret = meta.update(QCAMERA3_USE_ISO_EXP_PRIORITY, &iso_mode, 8);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);
    fprintf(stderr, "ISO mode switched to QCAMERA3_ISO_MODE_200\n");

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* TestISOMode400: This case will test a MultiCamera session with 3840x1920 h264
*                 encoded track with ISO_MODE_400. This test is configured to
*                 produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set ISO mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, TestISOMode400) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);

    int32_t use_exp_priority = 0;
    ret = meta.update(QCAMERA3_SELECT_PRIORITY, &use_exp_priority, 4);

    int64_t iso_mode;
    iso_mode = QCAMERA3_ISO_MODE_400;
    ret = meta.update(QCAMERA3_USE_ISO_EXP_PRIORITY, &iso_mode, 8);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);
    fprintf(stderr, "ISO mode switched to QCAMERA3_ISO_MODE_400\n");

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* TestISOMode800: This case will test a MultiCamera session with 3840x1920 h264
*                 encoded track with ISO_MODE_800. This test is configured to
*                 produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set ISO mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, TestISOMode800) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);

    int32_t use_exp_priority = 0;
    ret = meta.update(QCAMERA3_SELECT_PRIORITY, &use_exp_priority, 4);

    int64_t iso_mode;
    iso_mode = QCAMERA3_ISO_MODE_800;
    ret = meta.update(QCAMERA3_USE_ISO_EXP_PRIORITY, &iso_mode, 8);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);
    fprintf(stderr, "ISO mode switched to QCAMERA3_ISO_MODE_800\n");

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* TestISOMode1600: This case will test a MultiCamera session with 3840x1920 h264
*                 encoded track with ISO_MODE_1600. This test is configured to
*                 produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set ISO mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, TestISOMode1600) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);
    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};

    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);

    int32_t use_exp_priority = 0;
    ret = meta.update(QCAMERA3_SELECT_PRIORITY, &use_exp_priority, 4);

    int64_t iso_mode;
    iso_mode = QCAMERA3_ISO_MODE_1600;
    ret = meta.update(QCAMERA3_USE_ISO_EXP_PRIORITY, &iso_mode, 8);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);
    fprintf(stderr, "ISO mode switched to QCAMERA3_ISO_MODE_1600\n");

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* TestISOMode3200: This case will test a MultiCamera session with 3840x1920 h264
*                 encoded track with ISO_MODE_3200. This test is configured to
*                 produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - Set ISO mode
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(Recorder360Gtest, TestISOMode3200) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    SessionCb session_status_cb;
    session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                         size_t event_data_size) -> void
        { SessionCallbackHandler(event_type, event_data, event_data_size); };

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    assert(session_id > 0);
    assert(ret == NO_ERROR);

    VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                            stream_width,
                                            stream_height,
                                            30};
    // Set media profiles
    video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
    video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

    uint32_t video_track_id = 1;

    if (dump_bitstream_.IsEnabled()) {
      Stream360DumpInfo dumpinfo = {
        video_track_param.format_type,
        video_track_id,
        stream_width,
        stream_height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      assert(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

    video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
        void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                      video_track_param, video_track_cb);
    assert(ret == NO_ERROR);

    ret = recorder_.StartSession(session_id);
    assert(ret == NO_ERROR);

    CameraMetadata meta;
    ret = recorder_.GetCameraParam(multicam_id_, meta);

    int32_t use_exp_priority = 0;
    ret = meta.update(QCAMERA3_SELECT_PRIORITY, &use_exp_priority, 4);

    int64_t iso_mode;
    iso_mode = QCAMERA3_ISO_MODE_3200;
    ret = meta.update(QCAMERA3_USE_ISO_EXP_PRIORITY, &iso_mode, 8);
    assert(ret == NO_ERROR);

    ret = recorder_.SetCameraParam(multicam_id_, meta);
    assert(ret == NO_ERROR);
    fprintf(stderr, "ISO mode switched to QCAMERA3_ISO_MODE_3200\n");

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    assert(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    assert(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched4KEncTrackAnd6KSnapshot: This case will test a MultiCamera session with one
*                            3840x1920 h264 encodded track and 6k snapshot,
*                            configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*  - CreateVideoTrack
*  - StartVideoTrack
*   loop Start {
*   --------------------
*   - Capture Snapshot
*   --------------------
*   } loop End
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched4KEncTrackAnd6KSnapshot) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 3840;
  uint32_t stream_height = 1920;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

 SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                          stream_width,
                                          stream_height,
                                          30};
  // Set media profiles
  video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
  video_track_param.codec_param.avc.level   = AVCLevelType::kLevel5_1;

  uint32_t video_track_id = 1;

  if (dump_bitstream_.IsEnabled()) {
    Stream360DumpInfo dumpinfo = {
      video_track_param.format_type,
      video_track_id,
      stream_width,
      stream_height };
    ret = dump_bitstream_.SetUp(dumpinfo);
    assert(ret == NO_ERROR);
  }

  TrackCb video_track_cb;
  video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                            std::vector<BufferDescriptor> buffers,
                            std::vector<MetaData> meta_buffers) {
  VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

  video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
      void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
      event_type, event_data, event_data_size); };

  ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                    video_track_param, video_track_cb);
  assert(ret == NO_ERROR);

  ret = recorder_.StartSession(session_id);
  assert(ret == NO_ERROR);

  ImageParam image_param;
  memset(&image_param, 0x0, sizeof image_param);
  image_param.width         = 6080;
  image_param.height        = 3040;
  image_param.image_format  = ImageFormat::kJPEG;
  image_param.image_quality = 95;

  std::vector<CameraMetadata> meta_array;
  CameraMetadata meta;

  ret = recorder_.GetDefaultCaptureParam(multicam_id_, meta);
  assert(ret == NO_ERROR);

  meta_array.push_back(meta);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    ImageCaptureCb cb = [this] (uint32_t camera_id, uint32_t image_count,
                                BufferDescriptor buffer,
                                MetaData meta_data) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta_data); };

    ret = recorder_.CaptureImage(multicam_id_, image_param, 1, meta_array, cb);
    assert(ret == NO_ERROR);
    // Take snapshot after every 5 sec.
    sleep(kDelayAfterSnapshot);
  }

  ret = recorder_.StopSession(session_id, false);
  assert(ret == NO_ERROR);

  ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
  assert(ret == NO_ERROR);

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  dump_bitstream_.CloseAll();
  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* StitchedHDEncTrackAnd6KSnapshot: This case will test a MultiCamera session with one
*                            1920x960 h264 encodded track and 6k snapshot,
*                            configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*  - CreateVideoTrack
*  - StartVideoTrack
*   loop Start {
*   --------------------
*   - Capture Snapshot
*   --------------------
*   } loop End
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, StitchedHDEncTrackAnd6KSnapshot) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 1920;
  uint32_t stream_height = 960;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

 SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                          stream_width,
                                          stream_height,
                                          30};
  // Set media profiles
  video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
  video_track_param.codec_param.avc.level   = AVCLevelType::kLevel4;

  uint32_t video_track_id = 1;

  if (dump_bitstream_.IsEnabled()) {
    Stream360DumpInfo dumpinfo = {
      video_track_param.format_type,
      video_track_id,
      stream_width,
      stream_height };
    ret = dump_bitstream_.SetUp(dumpinfo);
    assert(ret == NO_ERROR);
  }

  TrackCb video_track_cb;
  video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                            std::vector<BufferDescriptor> buffers,
                            std::vector<MetaData> meta_buffers) {
  VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

  video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
      void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
      event_type, event_data, event_data_size); };

  ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                    video_track_param, video_track_cb);
  assert(ret == NO_ERROR);

  ret = recorder_.StartSession(session_id);
  assert(ret == NO_ERROR);

  ImageParam image_param;
  memset(&image_param, 0x0, sizeof image_param);
  image_param.width         = 6080;
  image_param.height        = 3040;
  image_param.image_format  = ImageFormat::kJPEG;
  image_param.image_quality = 95;

  std::vector<CameraMetadata> meta_array;
  CameraMetadata meta;

  ret = recorder_.GetDefaultCaptureParam(multicam_id_, meta);
  assert(ret == NO_ERROR);

  meta_array.push_back(meta);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    ImageCaptureCb cb = [this] (uint32_t camera_id, uint32_t image_count,
                                BufferDescriptor buffer,
                                MetaData meta_data) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta_data); };

    ret = recorder_.CaptureImage(multicam_id_, image_param, 1, meta_array, cb);
    assert(ret == NO_ERROR);
    // Take snapshot after every 5 sec.
    sleep(kDelayAfterSnapshot);
  }

  ret = recorder_.StopSession(session_id, false);
  assert(ret == NO_ERROR);

  ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
  assert(ret == NO_ERROR);

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  dump_bitstream_.CloseAll();
  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched720pEncTrackAnd6KSnapshot: This case will test a MultiCamera session with one
*                            1440x720 h264 encodded track and 6k snapshot,
*                            configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*  - CreateVideoTrack
*  - StartVideoTrack
*   loop Start {
*   --------------------
*   - Capture Snapshot
*   --------------------
*   } loop End
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched720pEncTrackAnd6KSnapshot) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 1440;
  uint32_t stream_height = 720;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

 SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);
  VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                          stream_width,
                                          stream_height,
                                          30};
  // Set media profiles
  video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
  video_track_param.codec_param.avc.level   = AVCLevelType::kLevel4;

  uint32_t video_track_id = 1;

  if (dump_bitstream_.IsEnabled()) {
    Stream360DumpInfo dumpinfo = {
      video_track_param.format_type,
      video_track_id,
      stream_width,
      stream_height };
    ret = dump_bitstream_.SetUp(dumpinfo);
    assert(ret == NO_ERROR);
  }

  TrackCb video_track_cb;
  video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                            std::vector<BufferDescriptor> buffers,
                            std::vector<MetaData> meta_buffers) {
  VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

  video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
      void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
      event_type, event_data, event_data_size); };

  ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                    video_track_param, video_track_cb);
  assert(ret == NO_ERROR);

  ret = recorder_.StartSession(session_id);
  assert(ret == NO_ERROR);

  ImageParam image_param;
  memset(&image_param, 0x0, sizeof image_param);
  image_param.width         = 6080;
  image_param.height        = 3040;
  image_param.image_format  = ImageFormat::kJPEG;
  image_param.image_quality = 95;

  std::vector<CameraMetadata> meta_array;
  CameraMetadata meta;

  ret = recorder_.GetDefaultCaptureParam(multicam_id_, meta);
  assert(ret == NO_ERROR);

  meta_array.push_back(meta);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    ImageCaptureCb cb = [this] (uint32_t camera_id, uint32_t image_count,
                                BufferDescriptor buffer,
                                MetaData meta_data) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta_data); };

    ret = recorder_.CaptureImage(multicam_id_, image_param, 1, meta_array, cb);
    assert(ret == NO_ERROR);
    // Take snapshot after every 5 sec.
    sleep(kDelayAfterSnapshot);
  }

  ret = recorder_.StopSession(session_id, false);
  assert(ret == NO_ERROR);

  ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
  assert(ret == NO_ERROR);

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  dump_bitstream_.CloseAll();
  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched480pEncTrackAnd6KSnapshot: This case will test a MultiCamera session with one
*                            960x480 h264 encodded track and 6k snapshot,
*                            configured to produce stitched frames.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*  - CreateVideoTrack
*  - StartVideoTrack
*   loop Start {
*   --------------------
*   - Capture Snapshot
*   --------------------
*   } loop End
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched480pEncTrackAnd6KSnapshot) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width  = 960;
  uint32_t stream_height = 480;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret = recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_,
                                       nullptr, 0);
  assert(ret == NO_ERROR);

  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_);
  assert(ret == NO_ERROR);

 SessionCb session_status_cb;
  session_status_cb.event_cb = [this] (EventType event_type, void *event_data,
                                       size_t event_data_size) -> void
      { SessionCallbackHandler(event_type, event_data, event_data_size); };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                          stream_width,
                                          stream_height,
                                          30};
  // Set media profiles
  video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
  video_track_param.codec_param.avc.level   = AVCLevelType::kLevel4;

  uint32_t video_track_id = 1;

  if (dump_bitstream_.IsEnabled()) {
    Stream360DumpInfo dumpinfo = {
      video_track_param.format_type,
      video_track_id,
      stream_width,
      stream_height };
    ret = dump_bitstream_.SetUp(dumpinfo);
    assert(ret == NO_ERROR);
  }

  TrackCb video_track_cb;
  video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
                            std::vector<BufferDescriptor> buffers,
                            std::vector<MetaData> meta_buffers) {
  VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers); };

  video_track_cb.event_cb = [&] (uint32_t track_id, EventType event_type,
      void *event_data, size_t event_data_size) { VideoTrackEventCb(track_id,
      event_type, event_data, event_data_size); };

  ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                    video_track_param, video_track_cb);
  assert(ret == NO_ERROR);

  ret = recorder_.StartSession(session_id);
  assert(ret == NO_ERROR);

  ImageParam image_param;
  memset(&image_param, 0x0, sizeof image_param);
  image_param.width         = 6080;
  image_param.height        = 3040;
  image_param.image_format  = ImageFormat::kJPEG;
  image_param.image_quality = 95;

  std::vector<CameraMetadata> meta_array;
  CameraMetadata meta;

  ret = recorder_.GetDefaultCaptureParam(multicam_id_, meta);
  assert(ret == NO_ERROR);

  meta_array.push_back(meta);

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    ImageCaptureCb cb = [this] (uint32_t camera_id, uint32_t image_count,
                                BufferDescriptor buffer,
                                MetaData meta_data) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta_data); };

    ret = recorder_.CaptureImage(multicam_id_, image_param, 1, meta_array, cb);
    assert(ret == NO_ERROR);
    // Take snapshot after every 5 sec.
    sleep(kDelayAfterSnapshot);
  }

  ret = recorder_.StopSession(session_id, false);
  assert(ret == NO_ERROR);

  ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
  assert(ret == NO_ERROR);

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  dump_bitstream_.CloseAll();
  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Stitched480pEncTrackAndPrintLumaValues: This case will test a MultiCamera
*                                         session with one 960x480 h264
*                                         encodded track and print luma values.
* Api test sequence:
*  - CreateMultiCamera
*  - ConfigureMultiCamera
*  - StartCamera
*  - CreateSession
*  - CreateVideoTrack
*  - StartVideoTrack
*  - Print Luma Values
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/
TEST_F(Recorder360Gtest, Stitched480pEncTrackAndPrintLumaValues) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());
  auto ret = Init();
  assert(ret == NO_ERROR);

  VideoFormat format_type = VideoFormat::kAVC;
  uint32_t stream_width = 960;
  uint32_t stream_height = 480;

  ret = recorder_.CreateMultiCamera(camera_ids_, &multicam_id_);
  assert(ret == NO_ERROR);

  ret =
      recorder_.ConfigureMultiCamera(multicam_id_, multicam_type_, nullptr, 0);
  assert(ret == NO_ERROR);
  CameraResultCb result_cb = [&](uint32_t camera_id,
                                 const CameraMetadata &result) {
    CameraResultCallbackHandler(camera_id, result);
  };
  ret = recorder_.StartCamera(multicam_id_, multicam_start_params_, result_cb);
  assert(ret == NO_ERROR);

  SessionCb session_status_cb;
  session_status_cb.event_cb = [this](EventType event_type, void *event_data,
                                      size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  assert(session_id > 0);
  assert(ret == NO_ERROR);

  VideoTrackCreateParam video_track_param{multicam_id_, format_type,
                                          stream_width,
                                          stream_height,
                                          30};
    // Set media profiles
  video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
  video_track_param.codec_param.avc.level = AVCLevelType::kLevel4;

  uint32_t video_track_id = 1;

  if (dump_bitstream_.IsEnabled()) {
    Stream360DumpInfo dumpinfo = {video_track_param.format_type, video_track_id,
                                  stream_width, stream_height};
    ret = dump_bitstream_.SetUp(dumpinfo);
    assert(ret == NO_ERROR);
  }

  TrackCb video_track_cb;
  video_track_cb.data_cb = [&, session_id](
      uint32_t track_id, std::vector<BufferDescriptor> buffers,
      std::vector<MetaData> meta_buffers) {
    VideoTrackOneEncDataCb(session_id, track_id, buffers, meta_buffers);
  };

  video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                void *event_data, size_t event_data_size) {
    VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
  };

  ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                   video_track_param, video_track_cb);
  assert(ret == NO_ERROR);

  ret = recorder_.StartSession(session_id);
  assert(ret == NO_ERROR);

  sleep(10);

  ret = recorder_.StopSession(session_id, false);
  assert(ret == NO_ERROR);

  ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
  assert(ret == NO_ERROR);

  ret = recorder_.DeleteSession(session_id);
  assert(ret == NO_ERROR);

  ret = recorder_.StopCamera(multicam_id_);
  assert(ret == NO_ERROR);

  ret = DeInit();
  assert(ret == NO_ERROR);

  dump_bitstream_.CloseAll();
  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

void Recorder360Gtest::RecorderCallbackHandler(EventType event_type,
                                            void *event_data,
                                            size_t event_data_size) {
  TEST_INFO("%s:%s Enter ", TAG, __func__);
  TEST_INFO("%s:%s Exit ", TAG, __func__);
}

void Recorder360Gtest::SessionCallbackHandler(EventType event_type,
                                          void *event_data,
                                          size_t event_data_size) {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

void Recorder360Gtest::CameraResultCallbackHandler(
    uint32_t camera_id, const CameraMetadata &result) {
  {
    static uint32_t count = 1;
    if (count%30 == 0) {
      if (result.exists(QCAMERA3_TARGET_LUMA)) {
        auto entry = result.find(QCAMERA3_TARGET_LUMA);
        TEST_INFO("%s:%s: Target Luma Value: %f", TAG, __func__,
                  entry.data.f[0]);
      } else {
        TEST_DBG("%s:%s QCAMERA3_TARGET_LUMA does not exists", TAG, __func__);
      }
      if (result.exists(QCAMERA3_CURRENT_LUMA)) {
        auto entry = result.find(QCAMERA3_CURRENT_LUMA);
        TEST_INFO("%s:%s: Current Luma Value: %f", TAG, __func__,
                  entry.data.f[0]);
      } else {
        TEST_DBG("%s:%s QCAMERA3_CURRENT_LUMA does not exists", TAG, __func__);
      }
      if (result.exists(QCAMERA3_LUMA_RANGE)) {
        auto entry = result.find(QCAMERA3_LUMA_RANGE);
        TEST_INFO("%s:%s: Target Luma Range: [%f - %f]", TAG, __func__,
                  entry.data.f[0], entry.data.f[1]);

      } else {
        TEST_DBG("%s:%s QCAMERA3_LUMA_RANGE does not exists", TAG, __func__);
      }
    }
    ++count;
  }
}

void Recorder360Gtest::VideoTrackYUVDataCb(uint32_t session_id,
                                        uint32_t track_id,
                                        std::vector<BufferDescriptor> buffers,
                                        std::vector<MetaData> meta_buffers) {

  TEST_DBG("%s:%s: Enter", TAG, __func__);

  if (is_dump_yuv_enabled_) {
    static uint32_t id = 0;
    ++id;
    if (id == dump_yuv_freq_) {
      String8 file_path;
      size_t written_len;
      file_path.appendFormat("/data/misc/qmmf/gtest_360_track_%d_%lld.yuv",
          track_id, buffers[0].timestamp);

      FILE *file = fopen(file_path.string(), "w+");
      if (!file) {
        TEST_ERROR("%s:%s: Unable to open file(%s)", TAG, __func__,
            file_path.string());
        goto FAIL;
      }

      written_len = fwrite(buffers[0].data, sizeof(uint8_t),
                           buffers[0].size, file);
      TEST_DBG("%s:%s: written_len =%d", TAG, __func__, written_len);
      if (buffers[0].size != written_len) {
        TEST_ERROR("%s:%s: Bad Write error (%d):(%s)\n", TAG, __func__, errno,
            strerror(errno));
        goto FAIL;
      }
      TEST_INFO("%s:%s: Buffer(0x%p) Size(%u) Stored@(%s)\n", TAG, __func__,
          buffers[0].data, written_len, file_path.string());

  FAIL:
      if (file != NULL) {
        fclose(file);
      }
      id = 0;
    }
  }

  // Return buffers back to service.
  auto ret = recorder_.ReturnTrackBuffer(session_id, track_id, buffers);
  assert(ret == NO_ERROR);
  TEST_DBG("%s:%s: Exit", TAG, __func__);
}

void Recorder360Gtest::VideoTrackOneEncDataCb(uint32_t session_id,
                                         uint32_t track_id,
                                         std::vector<BufferDescriptor> buffers,
                                         std::vector<MetaData> meta_buffers) {

  TEST_DBG("%s:%s: Enter", TAG, __func__);
  if (dump_bitstream_.IsUsed()) {
    int32_t file_fd = dump_bitstream_.GetFileFd(1);
    dump_bitstream_.Dump(buffers, file_fd);
  }
  // Return buffers back to service.
  auto ret = recorder_.ReturnTrackBuffer(session_id, track_id, buffers);
  assert(ret == NO_ERROR);

  TEST_DBG("%s:%s: Exit", TAG, __func__);
}

void Recorder360Gtest::VideoTrackTwoEncDataCb(uint32_t session_id,
                                         uint32_t track_id,
                                         std::vector<BufferDescriptor> buffers,
                                         std::vector<MetaData> meta_buffers) {

  TEST_DBG("%s:%s: Enter", TAG, __func__);
  if (dump_bitstream_.IsUsed()) {
    int32_t file_fd = dump_bitstream_.GetFileFd(2);
    dump_bitstream_.Dump(buffers, file_fd);
  }
  // Return buffers back to service.
  auto ret = recorder_.ReturnTrackBuffer(session_id, track_id, buffers);
  assert(ret == NO_ERROR);

  TEST_DBG("%s:%s: Exit", TAG, __func__);
}

void Recorder360Gtest::VideoTrackThreeEncDataCb(uint32_t session_id,
                                         uint32_t track_id,
                                         std::vector<BufferDescriptor> buffers,
                                         std::vector<MetaData> meta_buffers) {

  TEST_DBG("%s:%s: Enter", TAG, __func__);
  if (dump_bitstream_.IsUsed()) {
    int32_t file_fd = dump_bitstream_.GetFileFd(3);
    dump_bitstream_.Dump(buffers, file_fd);
  }
  // Return buffers back to service.
  auto ret = recorder_.ReturnTrackBuffer(session_id, track_id, buffers);
  assert(ret == NO_ERROR);

  TEST_DBG("%s:%s: Exit", TAG, __func__);
}

void Recorder360Gtest::VideoTrackEventCb(uint32_t track_id, EventType event_type,
                                      void *event_data, size_t data_size) {
    TEST_DBG("%s:%s: Enter", TAG, __func__);
    TEST_DBG("%s:%s: Exit", TAG, __func__);
}

void Recorder360Gtest::SnapshotCb(uint32_t camera_id,
                               uint32_t image_sequence_count,
                               BufferDescriptor buffer, MetaData meta_data) {

  TEST_INFO("%s:%s Enter", TAG, __func__);
  String8 file_path;
  size_t written_len;
  const char* ext_str;

  if (meta_data.meta_flag  &
      static_cast<uint32_t>(MetaParamType::kCamBufMetaData)) {
    CameraBufferMetaData cam_buf_meta = meta_data.cam_buffer_meta_data;
    TEST_DBG("%s:%s: format(0x%x)", TAG, __func__, cam_buf_meta.format);
    TEST_DBG("%s:%s: num_planes=%d", TAG, __func__, cam_buf_meta.num_planes);
    for (uint8_t i = 0; i < cam_buf_meta.num_planes; ++i) {
      TEST_DBG("%s:%s: plane[%d]:stride(%d)", TAG, __func__, i,
          cam_buf_meta.plane_info[i].stride);
      TEST_DBG("%s:%s: plane[%d]:scanline(%d)", TAG, __func__, i,
          cam_buf_meta.plane_info[i].scanline);
      TEST_DBG("%s:%s: plane[%d]:width(%d)", TAG, __func__, i,
          cam_buf_meta.plane_info[i].width);
      TEST_DBG("%s:%s: plane[%d]:height(%d)", TAG, __func__, i,
          cam_buf_meta.plane_info[i].height);
    }

    bool dump_file = true;
    if (cam_buf_meta.format == BufferFormat::kBLOB) {
      dump_file = (is_dump_jpeg_enabled_) ? true : false;
    } else {
      dump_file = (is_dump_raw_enabled_) ? true : false;
      fprintf(stderr, "\nRaw snapshot dumping enabled; "
              "keep track of free storage space.\n");
    }

    if (dump_file) {
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
        case BufferFormat::kRAW10:
        ext_str = "raw10";
        break;
        case BufferFormat::kRAW16:
        ext_str = "raw16";
        break;
        default:
        ext_str = "bin";
        break;
      }

      struct timeval tv;
      gettimeofday(&tv, NULL);
      uint64_t tv_ms = (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
      file_path.appendFormat("/data/misc/qmmf/snapshot_360_%u_%llu.%s",
          image_sequence_count, tv_ms, ext_str);
      FILE *file = fopen(file_path.string(), "w+");
      if (!file) {
        TEST_ERROR("%s:%s: Unable to open file(%s)", TAG, __func__,
            file_path.string());
        goto FAIL;
      }

      written_len = fwrite(buffer.data, sizeof(uint8_t), buffer.size, file);
      TEST_INFO("%s:%s: written_len =%d", TAG, __func__, written_len);
      if (buffer.size != written_len) {
        TEST_ERROR("%s:%s: Bad Write error (%d):(%s)\n", TAG, __func__, errno,
            strerror(errno));
        goto FAIL;
      }
      TEST_INFO("%s:%s: Buffer(0x%p) Size(%u) Stored@(%s)\n", TAG, __func__,
                buffer.data, written_len, file_path.string());

    FAIL:
      if (file != nullptr) {
        fclose(file);
      }
    }
  }
  // Return buffer back to recorder service.
  recorder_.ReturnImageCaptureBuffer(camera_id, buffer);
  TEST_INFO("%s:%s Exit", TAG, __func__);
}

status_t Recorder360Gtest::FillCropMetadata(CameraMetadata& meta,
                                            int32_t sensor_mode_w,
                                            int32_t sensor_mode_h,
                                            int32_t crop_x, int32_t crop_y,
                                            int32_t crop_w, int32_t crop_h) {

  auto active_array_size = meta.find(ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE);
  if (!active_array_size.count) {
    TEST_ERROR("%s:%s: Active sensor array size is missing!", TAG, __func__);
    return NAME_NOT_FOUND;
  }
  // Take the active pixel array width and height as base on which to
  // recalculate the actual crop region dimensions.
  float x = active_array_size.data.i32[2];
  float y = active_array_size.data.i32[3];
  float width = active_array_size.data.i32[2];
  float height = active_array_size.data.i32[3];

  // Get the crop region scale ratios and recalculate them against the base.
  x *= (static_cast<float>(crop_x) / sensor_mode_w);
  y *= (static_cast<float>(crop_y) / sensor_mode_h);
  width *= (static_cast<float>(crop_w) / sensor_mode_w);
  height *= (static_cast<float>(crop_h) / sensor_mode_h);

  int32_t crop_region[] = {
      static_cast<int32_t>(round(x)),
      static_cast<int32_t>(round(y)),
      static_cast<int32_t>(round(width)),
      static_cast<int32_t>(round(height)),
  };
  auto ret = meta.update(ANDROID_SCALER_CROP_REGION, crop_region, 4);
  if (NO_ERROR != ret) {
    TEST_ERROR("%s:%s: Failed to set crop region metadata!", TAG, __func__);
    return ret;
  }

  return NO_ERROR;
}

status_t Recorder360Gtest::DrawOverlay(void *data, int32_t width, int32_t height) {
  TEST_DBG("%s: Enter", __func__);
  status_t ret = 0;

#if USE_SKIA

#elif USE_CAIRO

  cr_surface_ = cairo_image_surface_create_for_data(static_cast<unsigned char*>
                                                    (data),
                                                    CAIRO_FORMAT_ARGB32, width,
                                                    height, width * 4);
  assert (cr_surface_ != nullptr);

  cr_context_ = cairo_create (cr_surface_);
  assert (cr_context_ != nullptr);

  uint32_t mask_color_ = 0x189BF2FF;
  RGBAValues mask_color;
  ExtractColorValues(mask_color_, &mask_color);

  // Paint entire rectangle with color.
  cairo_set_source_rgba (cr_context_, mask_color.red, mask_color.green,
                         mask_color.blue, mask_color.alpha);
  cairo_paint(cr_context_);
  assert(CAIRO_STATUS_SUCCESS == cairo_status(cr_context_));
  cairo_surface_flush (cr_surface_);

#endif
  TEST_DBG("%s: Exit", __func__);
  return ret;
}

void Recorder360Gtest::ExtractColorValues(uint32_t hex_color, RGBAValues* color) {

  color->red   = ((hex_color >> 24) & 0xff) / 255.0;
  color->green = ((hex_color >> 16) & 0xff) / 255.0;
  color->blue  = ((hex_color >> 8) & 0xff) / 255.0;
  color->alpha = ((hex_color) & 0xff) / 255.0;
}

status_t Dump360BitStream::SetUp(const Stream360DumpInfo& dumpinfo) {

  TEST_DBG("%s:%s: Enter", TAG, __func__);
  assert(dumpinfo.width > 0);
  assert(dumpinfo.height > 0);

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
  String8 extn(type_string);
  String8 bitstream_filepath;
  bitstream_filepath.appendFormat("/data/misc/qmmf/gtest_360_track_%d_%dx%d.%s",
                                  dumpinfo.track_id, dumpinfo.width,
                                  dumpinfo.height, extn.string());
  int32_t file_fd = open(bitstream_filepath.string(),
                          O_CREAT | O_WRONLY | O_TRUNC, 0655);
  if (file_fd <= 0) {
    TEST_ERROR("%s:%s File open failed!", TAG, __func__);
    return BAD_VALUE;
  }

  file_fds_.push_back(file_fd);

  TEST_DBG("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t Dump360BitStream::Dump(const std::vector<BufferDescriptor>& buffers,
                                const int32_t file_fd) {

  TEST_DBG("%s:%s: Enter", TAG, __func__);
  assert(file_fd > 0);

  for (auto& iter : buffers) {
    uint32_t exp_size = iter.size;
    TEST_DBG("%s:%s BitStream buffer data(%p):size(%d):ts(%lld):flag(0x%x)"
      ":buf_id(%d):capacity(%d)", TAG, __func__, iter.data, iter.size,
       iter.timestamp, iter.flag, iter.buf_id, iter.capacity);

    uint32_t written_length = write(file_fd, iter.data, iter.size);
    TEST_DBG("%s:%s: written_length(%d)", TAG, __func__, written_length);
    if (written_length != exp_size) {
      TEST_ERROR("%s:%s: Bad Write error (%d) %s", TAG, __func__, errno,
      strerror(errno));
      return BAD_VALUE;
    }

    if(iter.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS)) {
      TEST_INFO("%s:%s EOS Last buffer!", TAG, __func__);
      break;
    }
  }

  TEST_DBG("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

void Dump360BitStream::Close(int32_t file_fd) {
  TEST_DBG("%s:%s: Enter", TAG, __func__);
  if (file_fd > 0) {
    auto iter = std::find(file_fds_.begin(), file_fds_.end(), file_fd);
    if(iter != file_fds_.end()) {
      close(file_fd);
      file_fds_.erase(iter);
    } else {
      TEST_WARN("%s:%s: file_fd does not exist!", TAG, __func__);
    }
  }
  TEST_DBG("%s:%s: Exit", TAG, __func__);
}

void Dump360BitStream::CloseAll() {
  TEST_DBG("%s:%s: Enter", TAG, __func__);
  for (auto& iter : file_fds_) {
    if (iter > 0) {
      close(iter);
    }
  }
  file_fds_.clear();
  TEST_DBG("%s:%s: Exit", TAG, __func__);
}
