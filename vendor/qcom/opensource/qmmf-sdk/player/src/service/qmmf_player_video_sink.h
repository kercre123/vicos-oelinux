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

#pragma once

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <memory>
#include <mutex>
#include <thread>

#include <cutils/properties.h>
#include <fastcv/fastcv.h>
#include <linux/msm_ion.h>
#include <media/msm_media_info.h>
#include <utils/KeyedVector.h>

#include "common/codecadaptor/src/qmmf_avcodec.h"
#include "player/src/service/qmmf_player_common.h"
#include "player/src/service/qmmf_player_video_decoder_core.h"
#include "qmmf-sdk/qmmf_display.h"
#include "qmmf-sdk/qmmf_display_params.h"



using ::qmmf::display::DisplayEventType;
using ::qmmf::display::DisplayType;
using ::qmmf::display::Display;
using ::qmmf::display::DisplayCb;
using ::qmmf::display::SurfaceBuffer;
using ::qmmf::display::SurfaceParam;
using ::qmmf::display::SurfaceConfig;
using ::qmmf::display::SurfaceBlending;
using ::qmmf::display::SurfaceFormat;

#define DISPLAY_WIDTH 1920
#define DISPLAY_HEIGHT 1080

namespace qmmf {
namespace player {

class VideoTrackSink;
class VideoTrackDecoder;

class VideoSink {
 public:

  static VideoSink* CreateVideoSink();

  ~VideoSink();

  status_t CreateTrackSink(uint32_t track_id, VideoTrackParams& track_param);

  const ::std::shared_ptr<VideoTrackSink>& GetTrackSink(uint32_t track_id);

  status_t StartTrackSink(uint32_t track_id);

  status_t StopTrackSink(uint32_t track_id);

  status_t DeleteTrackSink(uint32_t track_id);

 private:
  VideoSink();

  static VideoSink* instance_;

  // Map of track it and TrackSinks.
  ::android::DefaultKeyedVector<uint32_t, ::std::shared_ptr<VideoTrackSink>>
      video_track_sinks;
};


class VideoTrackSink : public ::qmmf::avcodec::ICodecSource {
 public:

  VideoTrackSink();

  ~VideoTrackSink();

  status_t Init(VideoTrackParams& param);

  status_t StartSink();

  status_t StopSink();

  status_t PauseSink();

  status_t ResumeSink();

  status_t DeleteSink();

  status_t SetTrickMode(TrickModeSpeed speed, TrickModeDirection dir);

  void AddBufferList(::android::Vector<::qmmf::avcodec::CodecBuffer>& list);

  void PassTrackDecoder(
      const ::std::shared_ptr<VideoTrackDecoder>& video_track_decoder);

  status_t GetBuffer(BufferDescriptor& codec_buffer,
                     void* client_data) override;

  status_t ReturnBuffer(BufferDescriptor& codec_buffer,
                        void* client_data) override;

  status_t NotifyPortEvent(::qmmf::avcodec::PortEventType event_type,
                           void* event_data) override;

  status_t CreateDisplay(display::DisplayType display_type,
      VideoTrackParams& track_param);

  status_t DeleteDisplay(display::DisplayType display_type);

  void DisplayCallbackHandler(display::DisplayEventType event_type,
      void *event_data, size_t event_data_size);

  void DisplayVSyncHandler(int64_t time_stamp);

  status_t UpdateCropParameters(void* arg);

  status_t GrabPicture(PictureParam param, BufferDescriptor& buffer);

  status_t Dispatcher(const BufferDescriptor& codec_buffer);

  static void RendererThread(VideoTrackSink* video_sink);

  void Renderer();

 private:

  int32_t TrackId() { return track_params_.track_id; }

  VideoTrackParams        track_params_;
  ::qmmf::avcodec::PortreconfigData::CropData                crop_data_;
  uint32_t                current_width;
  uint32_t                current_height;
  ::std::shared_ptr<VideoTrackDecoder> video_track_decoder_;

  ::android::Vector<::qmmf::avcodec::CodecBuffer>  output_buffer_list_;
  TSQueue<::qmmf::avcodec::CodecBuffer>            output_free_buffer_queue_;
  TSQueue<::qmmf::avcodec::CodecBuffer>            output_occupy_buffer_queue_;

  ::android::Mutex        wait_for_frame_lock_;
  ::android::Condition    wait_for_frame_;
  ::android::Mutex        queue_lock_;
  bool                    stopplayback_;
  bool                    paused_;
  uint32_t                decoded_frame_number_;

  Display*   display_;
  uint32_t   surface_id_;
  SurfaceParam surface_param_;
  SurfaceBuffer surface_buffer_;
  SurfaceConfig surface_config;
  bool display_started_;

  typedef struct BufInfo {
    // FD at service
    uint32_t buf_id;

    // Memory mapped buffer.
    void*    vaddr;
  } BufInfo;

  //map<fd , buf_info>
  ::android::DefaultKeyedVector<int32_t, BufInfo> buf_info_map;

#ifdef DUMP_YUV_FRAMES
  int32_t               file_fd_;
  void DumpYUVData(BufferDescriptor& codec_buffer);
#endif

  status_t PushFrameToDisplay(BufferDescriptor& codec_buffer);

  status_t SkipFrame();

  status_t ReturnBufferToCodec(BufferDescriptor& codec_buffer);

  static void DisplayedBufferThread(VideoTrackSink* video_sink);

  void DisplayedBuffer();

  int32_t AllocateGrabPictureBuffer(const uint32_t size);

  status_t CopyGrabPictureBuffer(SurfaceBuffer& buffer, uint32_t size);

  void GetSnapShotDumpsProperty();

  TrickModeSpeed                         playback_speed_;
  TrickModeDirection                     playback_dir_;
  uint32_t                               displayed_frames_;
  std::mutex                             state_change_lock_;
  bool                                   grab_picture_;
  int32_t                                ion_device_;

  int32_t                                grabpicture_file_fd_;
  BufferDescriptor                       grab_picture_buffer_;
  struct ion_handle_data                 grab_picture_ion_handle_;
  ::android::Mutex                       grab_picture_buffer_copy_lock_;
  ::android::Condition                   wait_for_grab_picture_buffer_copy_;
  uint32_t                               snapshot_dumps_;
  std::mutex                             grab_picture_lock;

  time_point<high_resolution_clock>      prev_time_;
  uint32_t                               player_decode_profile_;

  ::std::thread*                         renderer_thread_;
  TSQueue<BufferDescriptor>              decoded_buffer_queue_;
  ::std::thread*                         displayed_buffer_thread_;
  TSQueue<BufferDescriptor>              displayed_buffer_queue_;
};

};  // namespace player
};  // namespace qmmf
