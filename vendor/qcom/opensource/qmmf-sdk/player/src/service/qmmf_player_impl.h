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

#include <utils/KeyedVector.h>
#include <pthread.h>
#include <vector>
#include <algorithm>


#include "player/src/client/qmmf_player_service_intf.h"
#include "player/src/service/qmmf_player_remote_cb.h"

#include "player/src/service/qmmf_player_common.h"
#include "player/src/service/qmmf_player_audio_decoder_core.h"
#include "player/src/service/qmmf_player_video_decoder_core.h"
#include "player/src/service/qmmf_player_audio_sink.h"
#include "player/src/service/qmmf_player_video_sink.h"
#include "common/qmmf_log.h"

namespace qmmf {
namespace player {

using namespace android;

class AudioTrackSink;

class PlayerImpl {
 public:

  static PlayerImpl* CreatePlayer();

  ~PlayerImpl();

  status_t Connect(sp<RemoteCallBack>& remote_cb);

  status_t Disconnect();

  status_t CreateAudioTrack(uint32_t track_id,
                          AudioTrackCreateParam& param);

  status_t CreateVideoTrack(uint32_t track_id,
                          VideoTrackCreateParam& param);

  status_t DeleteAudioTrack(uint32_t track_id);

  status_t DeleteVideoTrack(uint32_t track_id);

  status_t Prepare();

  status_t DequeueInputBuffer(uint32_t track_id,
                            std::vector<AVCodecBuffer>& buffers);

  status_t QueueInputBuffer(uint32_t track_id,
                          std::vector<AVCodecBuffer>& buffers,
                          void *meta_param,
                          size_t meta_size ,
                          TrackMetaBufferType meta_type);

  status_t Start();

  status_t Stop(bool do_flush);

  status_t Pause();

  status_t Resume();

  status_t SetPosition(int64_t seek_time);

  status_t SetTrickMode(TrickModeSpeed speed, TrickModeDirection dir);

  status_t GrabPicture(PictureParam param);

  status_t SetAudioTrackParam(uint32_t track_id,
                            CodecParamType type,
                            void *param,
                            size_t param_size);

  status_t SetVideoTrackParam(uint32_t track_id,
                            CodecParamType type,
                            void *param,
                            size_t param_size);

  void setCurrentState(PlayerState state);

  void NotifyPlayerEventCallback(EventType event_type, void *event_data,
                          size_t event_data_size);

  void NotifyVideoTrackDataCallback(uint32_t track_id,
                           std::vector<BnTrackBuffer> &buffers,
                           void *meta_param, TrackMetaBufferType meta_type,
                           size_t meta_size);

  void NotifyVideoTrackEventCallback(uint32_t track_id, EventType event_type,
                            void *event_data, size_t event_data_size);

  void NotifyAudioTrackDataCallback(uint32_t track_id,
                           std::vector<BnTrackBuffer> &buffers,
                           void *meta_param, TrackMetaBufferType meta_type,
                           size_t meta_size);

  void NotifyAudioTrackEventCallback(uint32_t track_id, EventType event_type,
                            void *event_data, size_t event_data_size);

  void NotifyDeleteAudioTrackCallback(uint32_t track_id);

  void NotifyDeleteVideoTrackCallback(uint32_t track_id);

  void NotifyGrabPictureDataCallback(BufferDescriptor& buffer);

 private:

  bool IsTrackValid(const uint32_t track_id);

  bool IsTrickModeEnabled();

  typedef struct TrackInfo {
    uint32_t         track_id;
    TrackType        type;
  } TrackInfo;

  uint32_t            unique_id_;
  sp<RemoteCallBack>  remote_cb_;

  AudioDecoderCore*   audio_decoder_core_;
  VideoDecoderCore*   video_decoder_core_;
  AudioSink*          audio_sink_;
  VideoSink*          video_sink_;

  PlayerState         current_state_;
  pthread_t           prepare_th;
  Mutex               state_lock_;
  TrickModeSpeed      trick_mode_speed_;
  TrickModeDirection  trick_mode_dir_;
  Mutex               trick_mode_change_lock_;

  std::vector<TrackInfo> tracks_;
  DefaultKeyedVector<uint32_t, TrackInfo> track_map_;


  /**Not allowed */
  PlayerImpl();
  PlayerImpl(const PlayerImpl&);
  PlayerImpl& operator=(const PlayerImpl&);
  static PlayerImpl* instance_;
};

};  // namespace player
};  // namespace qmmf
