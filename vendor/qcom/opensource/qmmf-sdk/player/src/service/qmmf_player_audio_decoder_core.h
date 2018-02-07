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
#include "player/src/service/qmmf_player_common.h"
#include "player/src/service/qmmf_player_audio_sink.h"
#include "include/qmmf-sdk/qmmf_avcodec_params.h"
#include "include/qmmf-sdk/qmmf_avcodec.h"
#include "include/qmmf-sdk/qmmf_buffer.h"


namespace qmmf {
namespace player {

using namespace android;

class AudioTrackSink;
class AudioTrackDecoder;

class AudioDecoderCore {
 public:
  ~AudioDecoderCore();

  static AudioDecoderCore* CreateAudioDecoderCore();

  status_t CreateAudioTrack(AudioTrackParams& params);

  status_t DequeueTrackInputBuffer(uint32_t track_id,
                         std::vector<AVCodecBuffer>& buffers);

  status_t QueueTrackInputBuffer(uint32_t track_id,
                         std::vector<AVCodecBuffer>& buffers);

  status_t PrepareTrackPipeline(uint32_t track_id,
      const ::std::shared_ptr<AudioTrackSink>& audio_track_sink);

  status_t StartTrackDecoder(uint32_t track_id);

  status_t StopTrackDecoder(uint32_t track_id, bool do_flush);

  status_t PauseTrackDecoder(uint32_t track_id);

  status_t ResumeTrackDecoder(uint32_t track_id);

  status_t SetAudioTrackDecoderParams(uint32_t track_id,
                               CodecParamType param_type, void* param,
                               uint32_t param_size);

  status_t DeleteTrackDecoder(uint32_t track_id);

 private:

  bool isTrackValid(uint32_t track_id);

  AudioDecoderCore();
  AudioDecoderCore(const AudioDecoderCore&);
  AudioDecoderCore& operator=(const AudioDecoderCore&);

  // Map of track id and audio decoder
  DefaultKeyedVector<uint32_t, ::std::shared_ptr<AudioTrackDecoder>> audio_track_decoders_;

  static AudioDecoderCore* instance_;
  int32_t ion_device_;
};

// This class behaves as both producer and consumer. At one end, it takes
// encoded data from the demuxer; and on the other end, it provides those
// encoded data to Decoder. It also manages buffer circulation, skip, etc.
class AudioTrackDecoder : public ::qmmf::avcodec::ICodecSource {
 public:
  AudioTrackDecoder(int32_t ion_device);

  ~AudioTrackDecoder();

  status_t ConfigureTrackDecoder(AudioTrackParams& track_params);

  status_t DequeueInputBuffer(std::vector<AVCodecBuffer>& buffers);

  status_t QueueInputBuffer(std::vector<AVCodecBuffer>& buffers);

  status_t PreparePipeline(const ::std::shared_ptr<AudioTrackSink>& audio_track_sink,
                           const ::std::shared_ptr<AudioTrackDecoder>& audio_track_decoder);

  status_t StartDecoder();

  status_t StopDecoder(bool do_flush);

  status_t PauseDecoder();

  status_t ResumeDecoder();

  status_t SetAudioDecoderParams(CodecParamType param_type, void* param,
                                  uint32_t param_size);

  status_t DeleteDecoder();

  status_t GetBuffer(BufferDescriptor& stream_buffer,
                     void* client_data) override;
  status_t ReturnBuffer(BufferDescriptor& stream_buffer,
                        void* client_data) override;
  status_t NotifyPortEvent(::qmmf::avcodec::PortEventType event_type,
                           void* event_data) override;

 private:

  status_t AllocInputPortBufs();

  status_t AllocOutputPortBufs();

  uint32_t TrackId() { return audio_track_params_.track_id; }

  typedef struct BufInfo {
    // FD at service
    uint32_t buf_id;

    // Memory mapped buffer.
    void*    vaddr;
  } BufInfo;

  // map<fd , buf_info>
  DefaultKeyedVector<uint32_t, BufInfo> buf_info_map;

  ::std::shared_ptr<AudioTrackSink> audio_track_sink_;
  AudioTrackParams                  audio_track_params_;
  ::qmmf::avcodec::AVCodec*         avcodec_;

  // For bitstream
  Vector<StreamBuffer>      input_buffer_list_;
  TSQueue<StreamBuffer>     unfilled_frame_queue_;
  TSQueue<StreamBuffer>     filled_frame_queue_;
  TSQueue<StreamBuffer>     frames_to_decode_;
  TSQueue<StreamBuffer>     frames_being_decoded_;

  typedef  struct ion_allocation_data IonHandleData;
  Vector<IonHandleData>     ion_handle_data;


  Vector<::qmmf::avcodec::CodecBuffer> output_buffer_list_;

  Mutex                     wait_for_empty_frame_lock_;
  Condition                 wait_for_empty_frame_;

  Mutex                     wait_for_frame_lock_;
  Condition                 wait_for_frame_;
  int32_t                   ion_device_;
  Mutex                     queue_lock_;

#ifdef DUMP_PCM_DATA
  int32_t                   file_fd_audio_;
#endif
};

};  // namepsse player
};  // namespace qmmf
