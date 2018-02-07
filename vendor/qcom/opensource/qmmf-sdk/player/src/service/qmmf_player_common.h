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

#include <chrono>
#include "qmmf-sdk/qmmf_player_params.h"

// Enable this define to dump audio bitstream from demuxer
//#define DUMP_AUDIO_BITSTREAM

// Enable this define to dump audio PCM from decoder
//#define DUMP_PCM_DATA

// Enable this define to dump video bitstream from demuxer
//#define DUMP_VIDEO_BITSTREAM

// Enable this define to dump video YUV frames from decoder
//#define DUMP_YUV_FRAMES


namespace qmmf {
namespace player {

using ::std::chrono::high_resolution_clock;
using ::std::chrono::microseconds;
using ::std::chrono::time_point;
using ::std::chrono::duration_cast;

enum class TrackType {
  kVideo,
  kAudio
};

enum PlayerState
{
  QPLAYER_STATE_ERROR = 0,
  QPLAYER_STATE_IDLE = 1 << 0,
  QPLAYER_STATE_PREPARED = 1 << 1,
  QPLAYER_STATE_STARTED = 1 << 2,
  QPLAYER_STATE_PAUSED = 1 << 3,
  QPLAYER_STATE_STOPPED =  1 << 4,
  QPLAYER_STATE_PLAYBACK_COMPLETED = 1<< 5,
};

/*
typedef std::function<void(uint32_t track_id, std::vector<BnBuffer> buffers,
    void *meta_param, MetaParamType meta_type, size_t meta_size)>
    buffer_callback;
*/

struct AudioTrackParams {
  AudioTrackCreateParam    params;
  uint32_t                 track_id;
  //buffer_callback        data_cb;
};

struct VideoTrackParams {
  VideoTrackCreateParam   params;
  uint32_t                track_id;
  //buffer_callback        data_cb;
};

struct AVCodecBuffer {
  void *data;
  size_t frame_length;
  size_t filled_length;
  uint64_t time_stamp;
  uint32_t flag;
  uint32_t fd;
  uint32_t buf_id;
};

struct Event{
  PlayerState state;
};

extern "C" void DebugAudioTrackCreateParam (const char* _func_,
                                            AudioTrackCreateParam& track_params);

extern "C" void DebugVideoTrackCreateParam (const char* _func_,
                                            VideoTrackCreateParam& track_params);

extern "C" void DebugAudioSinkParam (const char* _func_,
                                     AudioTrackParams& track_params);

extern "C" void DebugQueueInputBuffer(const char* _func_,
                                      std::vector<AVCodecBuffer>& buffers);

extern "C" uint32_t GetPlayerDecodeProfileProperty();

};  // namespace player
};  // namespace qmmf
