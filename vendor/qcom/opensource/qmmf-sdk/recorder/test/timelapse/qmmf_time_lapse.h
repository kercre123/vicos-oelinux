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

#ifndef QMMF_TIME_LAPSE_H_
#define QMMF_TIME_LAPSE_H_

#include <utils/Condition.h>
#include <qmmf-sdk/qmmf_recorder.h>
#include <qmmf-sdk/qmmf_recorder_params.h>
#include <qmmf-sdk/qmmf_display.h>
#include <qmmf-sdk/qmmf_display_params.h>

namespace qmmf {
namespace timelapse {

using namespace qmmf::recorder;
using namespace android;
using ::qmmf::display::DisplayEventType;
using ::qmmf::display::DisplayType;
using ::qmmf::display::Display;
using ::qmmf::display::DisplayCb;
using ::qmmf::display::SurfaceBuffer;
using ::qmmf::display::SurfaceParam;
using ::qmmf::display::SurfaceConfig;
using ::qmmf::display::SurfaceBlending;
using ::qmmf::display::SurfaceFormat;

struct TimeLapseParams {
  uint32_t              camera_id;
  uint32_t              preview_width;
  uint32_t              preview_height;
  uint32_t              snapshot_width;
  uint32_t              snapshot_height;
  uint32_t              period; // [ms.]
  uint32_t              count;
};

class TimeLapse {
 public:
  TimeLapse(TimeLapseParams params) : params_(params), session_id_(0),
  last_capture_ts_(0), snapshot_count_(0) {};

  int32_t Run();

  TimeLapse(const TimeLapse &) = delete;
  TimeLapse &operator=(const TimeLapse &) = delete;

 private:

  int32_t Init();
  int32_t DeInit();
  void PreviewTrackHandler(uint32_t track_id,
                           std::vector<BufferDescriptor> buffers,
                           std::vector<MetaData> meta_buffers);

  int32_t CreateSession();
  int32_t DeleteSession();

  int32_t StartSession();
  int32_t StopSession();

  int32_t AddPreviewTrack();
  int32_t DeletePreviewTrack();

  int32_t CaptureImage(bool store = true);
  void SnapshotCb(uint32_t camera_id,
                  uint32_t image_sequence_count,
                  BufferDescriptor buffer, MetaData meta_data);

  void DisplayCallbackHandler(DisplayEventType event_type,
                              void *event_data, size_t event_data_size);

  void DisplayVSyncHandler(int64_t time_stamp);

  status_t StartDisplay(DisplayType display_type);

  status_t StopDisplay(DisplayType display_type);

  status_t PushFrameToDisplay(BufferDescriptor& buffer,
                              CameraBufferMetaData& meta_data);

  Recorder              recorder_;
  CameraMetadata        static_info_;
  TimeLapseParams       params_;
  uint32_t              session_id_;
  uint64_t              last_capture_ts_;
  uint64_t              snapshot_count_;
  Condition             lapse_cond_;
  Mutex                 lapse_lock_;
  Condition             snapshot_cond_;
  Mutex                 snapshot_lock_;


  static const uint32_t kPreviewTrackId;

  Display*   display_;
  uint32_t   surface_id_;
  SurfaceParam surface_param_;
  SurfaceBuffer surface_buffer_;
  bool display_started_;
};

} //namespace timelapse ends here
} //namespace qmmf ends here

#endif /* QMMF_TIME_LAPSE_H_ */
