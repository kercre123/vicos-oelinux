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

#include <memory>

#include <utils/KeyedVector.h>

#include "include/qmmf-sdk/qmmf_player_params.h"
#include "include/qmmf-sdk/qmmf_codec.h"
#include "common/codecadaptor/src/qmmf_avcodec.h"
#include "player/src/service/qmmf_player_common.h"
#include "player/src/service/qmmf_player_video_sink.h"

namespace qmmf {
namespace player {

using namespace android;

class VideoTrackDecoder;
class VideoTrackSink;

class VideoDecoderCore {
 public:
  ~VideoDecoderCore();

  static VideoDecoderCore* CreateVideoDecoderCore();

  status_t CreateVideoTrack(VideoTrackParams& params);

  status_t DequeueTrackInputBuffer(uint32_t track_id,
      std::vector<AVCodecBuffer>& buffers);

  status_t QueueTrackInputBuffer(uint32_t track_id,
      std::vector<AVCodecBuffer>& buffers);

  status_t PrepareTrackPipeline(uint32_t track_id,
      const ::std::shared_ptr<VideoTrackSink>& video_track_sink);

  status_t StartTrackDecoder(uint32_t track_id);

  status_t StopTrackDecoder(uint32_t track_id, bool do_flush);

  status_t PauseTrackDecoder(uint32_t track_id);

  status_t ResumeTrackDecoder(uint32_t track_id);

  status_t SetVideoTrackDecoderParams(uint32_t track_id,
                                      CodecParamType param_type,
                                      void* param,
                                      uint32_t param_size);

  status_t DeleteTrackDecoder(uint32_t track_id);

  status_t SetTrackTrickMode(uint32_t track_id, TrickModeSpeed speed,
                             TrickModeDirection dir);

 private:

  bool isTrackValid(uint32_t track_id);

  VideoDecoderCore();

  VideoDecoderCore(const VideoDecoderCore&);

  VideoDecoderCore& operator=(const VideoDecoderCore&);

  //Map of track id and video decoder
  DefaultKeyedVector<uint32_t, ::std::shared_ptr<VideoTrackDecoder>>video_track_decoders_;

  static VideoDecoderCore* instance_;
  int32_t ion_device_;
};

class VideoTrackDecoder : public ::qmmf::avcodec::ICodecSource {
 public:
  VideoTrackDecoder(int32_t ion_device);

  ~VideoTrackDecoder();

  status_t ConfigureTrackDecoder(VideoTrackParams& track_params);

  status_t DequeueInputBuffer(std::vector<AVCodecBuffer>& buffers);

  status_t QueueInputBuffer(std::vector<AVCodecBuffer>& buffers);

  status_t PreparePipeline(const ::std::shared_ptr<VideoTrackSink>& video_track_sink,
                           const ::std::shared_ptr<VideoTrackDecoder>& video_track_decoder);

  status_t StartDecoder();

  status_t StopDecoder(bool do_flush);

  status_t PauseDecoder();

  status_t ResumeDecoder();

  status_t SetVideoDecoderParams(CodecParamType param_type, void* param,
                                 uint32_t param_size);

  status_t DeleteDecoder();

  status_t SetTrickMode(TrickModeSpeed speed, TrickModeDirection dir);

  status_t GetBuffer(BufferDescriptor& stream_buffer,
                     void* client_data) override;
  status_t ReturnBuffer(BufferDescriptor& stream_buffer,
                        void* client_data) override;
  status_t NotifyPortEvent(::qmmf::avcodec::PortEventType event_type,
                           void* event_data) override;

  status_t ReconfigOutputPort(void *arg);

 private:

  status_t AllocInputPortBufs();

  status_t AllocOutputPortBufs();

  status_t ReleaseOutputBuffers();

  uint32_t TrackId() { return video_track_params_.track_id; }

  typedef struct BufInfo {
    // FD at service
    uint32_t buf_id;

    // Memory mapped buffer.
    void*    vaddr;
  } BufInfo;

  //map<fd , buf_info>
  DefaultKeyedVector<uint32_t, BufInfo> buf_info_map;

  ::std::shared_ptr<VideoTrackSink> video_track_sink_;
  VideoTrackParams                  video_track_params_;
  ::qmmf::avcodec::AVCodec*         avcodec_;

  //For input port
  Vector<StreamBuffer>    input_buffer_list_;
  TSQueue<StreamBuffer>   unfilled_frame_queue_;
  TSQueue<StreamBuffer>   filled_frame_queue_;
  TSQueue<StreamBuffer>   frames_to_decode_;
  TSQueue<StreamBuffer>   frames_being_decoded_;

  typedef  struct ion_allocation_data IonHandleData;
  Vector<IonHandleData>   ion_handle_data;


  Vector<::qmmf::avcodec::CodecBuffer> output_buffer_list_;
  uint32_t                             output_buffer_count_;
  uint32_t                             output_buffer_size_;

  Mutex                   wait_for_empty_frame_lock_;
  Condition               wait_for_empty_frame_;
  Mutex                   wait_for_frame_lock_;
  Condition               wait_for_frame_;
  int32_t                 ion_device_;
  Mutex                   queue_lock_;

#ifdef DUMP_VIDEO_BITSTREAM
  int32_t                 file_fd_video_;
#endif
  time_point<high_resolution_clock>   prev_time_;
  uint32_t                            player_decode_profile_;
};

};  // namespace player
};  // namespace qmmf
