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

#pragma once

#include <fcntl.h>
#include <dirent.h>
#include <functional>
#include <gtest/gtest.h>
#include <vector>
#include <map>
#include <cutils/properties.h>
#include <qmmf-sdk/qmmf_recorder.h>
#include <qmmf-sdk/qmmf_recorder_params.h>

#if USE_SKIA
#include <SkCanvas.h>
#elif USE_CAIRO
#include <cairo/cairo.h>
#endif

using namespace qmmf;
using namespace recorder;
using namespace android;

#define DEFAULT_360_YUV_DUMP_FREQ       "200"
#define DEFAULT_360_ITERATIONS_COUNT    "50"
// Default recording duration is 2 minutes i.e. 2 * 60 seconds
#define DEFAULT_360_RECORD_DURATION     "120"

// Prop to enable YUV data dumping from YUV track
#define PROP_DUMP_360_YUV_FRAMES        "persist.qmmf.360.gtest.dump.yuv"
// Prop to enable encoded bitstream data dumping
#define PROP_DUMP_360_BITSTREAM         "persist.qmmf.360.gtest.dump.stm"
// Prop to enable JPEG (BLOB) dumping
#define PROP_DUMP_360_JPEG              "persist.qmmf.360.gtest.dump.jpg"
// Prop to enable RAW Snapshot dumping
#define PROP_DUMP_360_RAW               "persist.qmmf.360.gtest.dump.raw"
// Prop to set frequency of YUV data dumping
#define PROP_DUMP_360_YUV_FREQ          "persist.qmmf.360.gtest.dump.frq"
// Prop to set no of iterations
#define PROP_360_N_ITERATIONS           "persist.qmmf.360.gtest.iter"
// Prop to set recording duration in seconds
#define PROP_360_RECORD_DURATION        "persist.qmmf.360.gtest.rec.dur"

typedef struct Stream360DumpInfo {
  VideoFormat           format;
  uint32_t              track_id;
  uint32_t               width;
  uint32_t               height;
} Stream360DumpInfo;

struct RGBAValues {
  double red;
  double green;
  double blue;
  double alpha;
};

class Dump360BitStream {
 public:
  Dump360BitStream() : is_enabled_(false) {};

  ~Dump360BitStream() {file_fds_.clear();}

  bool IsEnabled() {return is_enabled_;}

  bool IsUsed() {return (is_enabled_ && file_fds_.size());}

  int32_t GetFileFd(const uint32_t count)
                   {assert(count > 0);
                    assert(count <= file_fds_.size());
                    return file_fds_[count-1];}

  void Enable(const bool enable) {is_enabled_ = enable;}

  status_t SetUp(const Stream360DumpInfo& dumpinfo);

  status_t Dump(const std::vector<BufferDescriptor>& buffers,
                const int32_t file_fd);

  void Close(int32_t file_fd);

  void CloseAll();
 private:
  bool is_enabled_;
  std::vector<int32_t> file_fds_;
};

class Recorder360Gtest : public ::testing::Test {
 public:
  Recorder360Gtest() : recorder_() {};

  ~Recorder360Gtest() {};

 protected:
  const ::testing::TestInfo* test_info_;

  void SetUp() override;

  void TearDown() override;

  int32_t Init();

  int32_t DeInit();

  void RecorderCallbackHandler(EventType event_type, void *event_data,
                               size_t event_data_size);

  void SessionCallbackHandler(EventType event_type,
                              void *event_data,
                              size_t event_data_size);

  void CameraResultCallbackHandler(uint32_t camera_id,
                                   const CameraMetadata &result);

  void VideoTrackYUVDataCb(uint32_t session_id, uint32_t track_id,
                           std::vector<BufferDescriptor> buffers,
                           std::vector<MetaData> meta_buffers);

  void VideoTrackOneEncDataCb(uint32_t session_id, uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers);

  void VideoTrackTwoEncDataCb(uint32_t session_id, uint32_t track_id,
                              std::vector<BufferDescriptor> buffers,
                              std::vector<MetaData> meta_buffers);

  void VideoTrackThreeEncDataCb(uint32_t session_id, uint32_t track_id,
                                std::vector<BufferDescriptor> buffers,
                                std::vector<MetaData> meta_buffers);

  void VideoTrackEventCb(uint32_t track_id, EventType event_type,
                         void *event_data, size_t event_data_size);

  void SnapshotCb(uint32_t camera_id, uint32_t image_sequence_count,
                  BufferDescriptor buffer, MetaData meta_data);

  status_t FillCropMetadata(CameraMetadata& meta,
                            int32_t sensor_mode_w, int32_t sensor_mode_h,
                            int32_t crop_x, int32_t crop_y,
                            int32_t crop_w, int32_t crop_h);
  status_t DrawOverlay(void *data, int32_t width, int32_t height);

  void ExtractColorValues(uint32_t hex_color, RGBAValues* color);

  Recorder              recorder_;
  uint32_t              multicam_id_;
  MultiCameraConfigType multicam_type_;
  uint32_t              iteration_count_;
  std::vector<uint32_t> camera_ids_;
  CameraStartParam      multicam_start_params_;
  RecorderCb            recorder_status_cb_;

  Dump360BitStream      dump_bitstream_;
  bool                  is_dump_jpeg_enabled_;
  bool                  is_dump_raw_enabled_;
  bool                  is_dump_yuv_enabled_;
  uint32_t              dump_yuv_freq_;
  uint32_t              record_duration_;
#if USE_SKIA
  SkCanvas*            canvas_;
#elif USE_CAIRO
  cairo_surface_t*     cr_surface_;
  cairo_t*             cr_context_;
#endif
};

