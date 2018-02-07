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

#define TAG "PlayerImpl"

#include "player/src/service/qmmf_player_impl.h"

 namespace qmmf {
 namespace player {

PlayerImpl* PlayerImpl::instance_ = nullptr;


PlayerImpl* PlayerImpl::CreatePlayer() {
  if(!instance_) {
    instance_ = new PlayerImpl;
    if(!instance_) {
      QMMF_ERROR("%s:%s: Can't Create Player Instance!", TAG, __func__);
      return nullptr;
    }
  }
  QMMF_INFO("%s:%s: Player Instance Created Successfully(0x%p)", TAG,
    __func__, instance_);
  return instance_;
}

PlayerImpl::PlayerImpl()
    : audio_decoder_core_(nullptr), video_decoder_core_(nullptr),
      audio_sink_(nullptr), video_sink_(nullptr),
      current_state_(PlayerState::QPLAYER_STATE_IDLE),
      trick_mode_speed_(TrickModeSpeed::kSpeed_1x),
      trick_mode_dir_(TrickModeDirection::kNormalForward)
{
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

PlayerImpl::~PlayerImpl() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  if (audio_decoder_core_) {
    delete audio_decoder_core_;
    audio_decoder_core_ = nullptr;
  }

  if (video_decoder_core_) {
    delete video_decoder_core_;
    video_decoder_core_ = nullptr;
  }

  if (audio_sink_) {
    delete audio_sink_;
    audio_sink_ = nullptr;
  }

  if (video_sink_) {
    delete video_sink_;
    video_sink_ = nullptr;
  }

  instance_ = nullptr;
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

status_t PlayerImpl::Connect(sp<RemoteCallBack>& remote_cb) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  assert(remote_cb.get() != nullptr);
  remote_cb_ = remote_cb;

  audio_decoder_core_ = AudioDecoderCore::CreateAudioDecoderCore();
  if (!audio_decoder_core_) {
    QMMF_ERROR("%s:%s: Can't Create AudioDecoderCore Instance!", TAG, __func__);
    return NO_MEMORY;
  }
  QMMF_INFO("%s:%s: AudioDecoderCore Instance Created Successfully!", TAG,
       __func__);

  video_decoder_core_ = VideoDecoderCore::CreateVideoDecoderCore();
  if (!video_decoder_core_) {
    QMMF_ERROR("%s:%s: Can't Create VideoDecoderCore Instance!", TAG, __func__);
    return NO_MEMORY;
  }
  QMMF_INFO("%s:%s: VideoDecoderCore Instance Created Successfully!", TAG,
       __func__);

  audio_sink_ = AudioSink::CreateAudioSink();
  if (!audio_sink_) {
    QMMF_ERROR("%s:%s: Can't Create AudioSink Instance!", TAG, __func__);
    return NO_MEMORY;
  }
  QMMF_INFO("%s:%s: AudioSink Instance Created Successfully!", TAG,
       __func__);

  video_sink_ = VideoSink::CreateVideoSink();
  if (!video_sink_) {
    QMMF_ERROR("%s:%s: Can't Create VideoSink Instance!", TAG, __func__);
    return NO_MEMORY;
  }
   QMMF_INFO("%s:%s: VideoSink Instance Created Successfully!", TAG,
        __func__);

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t PlayerImpl::Disconnect() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  if (audio_decoder_core_) {
    delete audio_decoder_core_;
    audio_decoder_core_ = nullptr;
  }

  if (video_decoder_core_) {
    delete video_decoder_core_;
    video_decoder_core_ = nullptr;
  }

  if (audio_sink_) {
    delete audio_sink_;
    audio_sink_ = nullptr;
  }

  if (video_sink_) {
    delete video_sink_;
    video_sink_ = nullptr;
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);

  return NO_ERROR;
}

status_t PlayerImpl::CreateAudioTrack(uint32_t track_id,
                                      AudioTrackCreateParam& param) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  status_t result;

  AudioTrackParams audio_track_param;
  memset(&audio_track_param,0x0, sizeof audio_track_param);

  audio_track_param.track_id    = track_id;
  audio_track_param.params      = param;

  DebugAudioTrackCreateParam(__func__, param);

  result = audio_decoder_core_->CreateAudioTrack(audio_track_param);
  if(result != NO_ERROR) {
      QMMF_ERROR("%s:%s: CreateAudioTrack failed!", TAG, __func__);
      return BAD_VALUE;
  }

  assert(audio_sink_ != nullptr);
  audio_sink_->CreateTrackSink(track_id, audio_track_param);
  if(result != NO_ERROR) {
    QMMF_ERROR("%s:%s: Audio CreateTrackSink id(%d) failed!", TAG, __func__,
              track_id);
    return BAD_VALUE;
  }
  QMMF_INFO("%s:%s: AudioTrackSink for track_id(%d) Added Successfully in"
    "AudioSink", TAG, __func__, track_id);

  TrackInfo track_info;
  memset(&track_info, 0x0, sizeof track_info);
  track_info.track_id     = track_id;
  track_info.type         = TrackType::kAudio;
  tracks_.push_back(track_info);
  track_map_.add(track_id, track_info);

  QMMF_INFO("%s:%s: Exit", TAG, __func__);

  return NO_ERROR;
}

status_t PlayerImpl::CreateVideoTrack(uint32_t track_id,
                                      VideoTrackCreateParam& param) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  status_t result;

  VideoTrackParams video_track_param;

  video_track_param.track_id    = track_id;
  video_track_param.params      = param;

  DebugVideoTrackCreateParam(__func__, param);

  result = video_decoder_core_->CreateVideoTrack(video_track_param);
  if(result != NO_ERROR) {
      QMMF_ERROR("%s:%s: CreateVideoTrack failed!", TAG, __func__);
      return BAD_VALUE;
    }

  assert(video_sink_ != nullptr);
  result = video_sink_->CreateTrackSink(track_id, video_track_param);
  if(result != NO_ERROR) {
     QMMF_ERROR("%s:%s: Video CreateTrackSink id(%d) failed!", TAG, __func__,
               track_id);
     return BAD_VALUE;
  }
  QMMF_INFO("%s:%s: VideoTrackSink for track_id(%d) Added Successfully in"
    "VideoSink",TAG, __func__, track_id);

  TrackInfo track_info;
  memset(&track_info, 0x0, sizeof track_info);
  track_info.track_id     = track_id;
  track_info.type         = TrackType::kVideo;
  tracks_.push_back(track_info);
  track_map_.add(track_id, track_info);

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t PlayerImpl::DeleteAudioTrack(uint32_t track_id) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  status_t result;

  assert(audio_sink_ != nullptr);
  result = audio_sink_->DeleteTrackSink(track_id);
  if (result != NO_ERROR) {
    QMMF_ERROR("%s:%s: track_id(%d) DeleteTrackSink failed: %d", TAG,
               __func__, track_id, result);
    return result;
  }

  audio_decoder_core_->DeleteTrackDecoder(track_id);
  track_map_.removeItem(track_id);

  auto it = std::find_if(tracks_.begin(), tracks_.end(),
     [track_id](TrackInfo& track){ return track.track_id == track_id; });

  tracks_.erase(it);

  QMMF_INFO("%s:%s: Exit", TAG, __func__);

  return NO_ERROR;
}

status_t PlayerImpl::DeleteVideoTrack(uint32_t track_id) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  status_t result;

  assert(video_sink_ != nullptr);
  result = video_sink_->DeleteTrackSink(track_id);
  if (result != NO_ERROR) {
    QMMF_ERROR("%s:%s: track_id(%d) DeleteTrackSource failed: %d", TAG,
               __func__, track_id, result);
    return result;
  }

  video_decoder_core_->DeleteTrackDecoder(track_id);
  track_map_.removeItem(track_id);

  auto it = std::find_if(tracks_.begin(), tracks_.end(),
     [track_id](TrackInfo& track){ return track.track_id == track_id; });

  tracks_.erase(it);

  QMMF_INFO("%s:%s: Exit", TAG, __func__);

  return NO_ERROR;
}

status_t PlayerImpl::DequeueInputBuffer(uint32_t track_id,
                                        std::vector<AVCodecBuffer>& buffers) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;

  size_t num_tracks = tracks_.size();

  for(size_t i = 0; i < num_tracks; i++) {

    if ((tracks_[i].track_id == track_id) &&
        (tracks_[i].type == TrackType::kVideo)) {
      ret = video_decoder_core_->DequeueTrackInputBuffer(
          tracks_[i].track_id, buffers);

    } else if ((tracks_[i].track_id == track_id) &&
        (tracks_[i].type == TrackType::kAudio)) {
      ret = audio_decoder_core_->DequeueTrackInputBuffer(
          tracks_[i].track_id, buffers);
    }
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t PlayerImpl::QueueInputBuffer(uint32_t track_id,
                                      std::vector<AVCodecBuffer>& buffers,
                                      void *meta_param,
                                      size_t meta_size,
                                      TrackMetaBufferType meta_type) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;

  size_t num_tracks = tracks_.size();

  for(size_t i = 0; i < num_tracks; i++) {

    if ((tracks_[i].track_id == track_id) &&
        (tracks_[i].type == TrackType::kVideo)) {
      ret = video_decoder_core_->QueueTrackInputBuffer(
          tracks_[i].track_id, buffers);

    } else if ((tracks_[i].track_id == track_id) &&
        (tracks_[i].type == TrackType::kAudio)) {
      ret = audio_decoder_core_->QueueTrackInputBuffer(
         tracks_[i].track_id, buffers);
    }
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);

  return ret;
}

status_t PlayerImpl::Prepare() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;

  Mutex::Autolock lock(state_lock_);

  Event event;
  memset(&event,0x0,sizeof(Event));


  if (current_state_ & (PlayerState::QPLAYER_STATE_PREPARED))
    return NO_ERROR;

  if (current_state_ & (PlayerState::QPLAYER_STATE_IDLE |
      PlayerState::QPLAYER_STATE_STOPPED)) {
    QMMF_INFO("%s:%s: PrepareTrackPipeline !", TAG, __func__);

      size_t num_tracks = tracks_.size();

      for(size_t i = 0; i < num_tracks; i++) {
        if (tracks_[i].type == TrackType::kVideo) {
          ret = video_decoder_core_->PrepareTrackPipeline(tracks_[i].track_id,
             video_sink_->GetTrackSink(tracks_[i].track_id));
        } else if (tracks_[i].type == TrackType::kAudio) {
          ret = audio_decoder_core_->PrepareTrackPipeline(tracks_[i].track_id,
             audio_sink_->GetTrackSink(tracks_[i].track_id));
        }
      }

      if (ret != NO_ERROR) {
           QMMF_INFO("%s:%s: Prepare failed!", TAG, __func__);
           setCurrentState(PlayerState::QPLAYER_STATE_ERROR);
           event.state = PlayerState::QPLAYER_STATE_ERROR;
           QMMF_INFO("%s:%s: EventType: %d state: %d", TAG,__func__,
               EventType::kStateChanged, event.state);
           NotifyPlayerEventCallback(EventType::kStateChanged,
               &event,sizeof(Event));
           return BAD_VALUE;
       } else {
           QMMF_INFO("%s:%s: Prepare successs!", TAG, __func__);
           setCurrentState(PlayerState::QPLAYER_STATE_PREPARED);
           event.state = PlayerState::QPLAYER_STATE_PREPARED;
           QMMF_INFO("%s:%s: EventType: %d state: %d", TAG,__func__,
               EventType::kStateChanged, event.state);
           NotifyPlayerEventCallback(EventType::kStateChanged,&event,
               sizeof(Event));
       }
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t PlayerImpl::Start() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;

  Mutex::Autolock lock(state_lock_);

  Event event;
  memset(&event,0x0,sizeof(Event));

  if (current_state_ & PlayerState::QPLAYER_STATE_STARTED)
    return NO_ERROR;

  if(current_state_ & (PlayerState::QPLAYER_STATE_PREPARED |
      PlayerState::QPLAYER_STATE_PAUSED |
      PlayerState::QPLAYER_STATE_PLAYBACK_COMPLETED |
      PlayerState::QPLAYER_STATE_STOPPED)) {

    size_t num_tracks = tracks_.size();

    for(size_t i = 0; i < num_tracks; i++) {
      if (tracks_[i].type == TrackType::kVideo) {
        ret = video_decoder_core_->StartTrackDecoder(tracks_[i].track_id);
      } else if ((tracks_[i].type == TrackType::kAudio) && (!IsTrickModeEnabled())) {
        ret = audio_decoder_core_->StartTrackDecoder(tracks_[i].track_id);
      }
    }

   if (ret != NO_ERROR) {
     QMMF_INFO("%s:%s: Start failed!", TAG, __func__);
     setCurrentState(PlayerState::QPLAYER_STATE_ERROR);
     event.state = PlayerState::QPLAYER_STATE_ERROR;
     QMMF_INFO("%s:%s: EventType: %d state: %d", TAG,__func__,
         EventType::kStateChanged, event.state);
     NotifyPlayerEventCallback(EventType::kStateChanged,&event,sizeof(Event));
     return BAD_VALUE;
   } else {
       QMMF_INFO("%s:%s: Start successs!", TAG, __func__);
       setCurrentState(PlayerState::QPLAYER_STATE_STARTED);
       event.state = PlayerState::QPLAYER_STATE_STARTED;
       QMMF_INFO("%s:%s: EventType: %d state: %d", TAG,__func__,
           EventType::kStateChanged, event.state);
       NotifyPlayerEventCallback(EventType::kStateChanged,&event,sizeof(Event));
     if (current_state_ & PlayerState::QPLAYER_STATE_PLAYBACK_COMPLETED)
       QMMF_INFO("%s:%s: PlayBack completed", TAG, __func__);
   }
  }

  QMMF_DEBUG("%s:%s: Start Called in %d", TAG, __func__, current_state_);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t PlayerImpl::Stop(bool do_flush) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  Mutex::Autolock lock(state_lock_);

  status_t ret = NO_ERROR;

  Event event;
  memset(&event,0x0,sizeof(Event));

  if (current_state_ & (PlayerState::QPLAYER_STATE_STOPPED))
  return NO_ERROR;

  if(current_state_ & (PlayerState::QPLAYER_STATE_PREPARED |
      PlayerState::QPLAYER_STATE_STARTED |
      PlayerState::QPLAYER_STATE_PAUSED |
      PlayerState::QPLAYER_STATE_PLAYBACK_COMPLETED)) {

    size_t num_tracks = tracks_.size();

    for(size_t i = 0; i < num_tracks; i++) {
      if (tracks_[i].type == TrackType::kVideo) {
        ret = video_decoder_core_->StopTrackDecoder(tracks_[i].track_id,
           do_flush);
      } else if ((tracks_[i].type == TrackType::kAudio) && (!IsTrickModeEnabled())) {
        ret = audio_decoder_core_->StopTrackDecoder(tracks_[i].track_id,
           do_flush);
      }
    }

    if (ret != NO_ERROR) {
      QMMF_INFO("%s:%s: Stop failed!", TAG, __func__);
      setCurrentState(PlayerState::QPLAYER_STATE_ERROR);
      event.state = PlayerState::QPLAYER_STATE_ERROR;
      QMMF_INFO("%s:%s: EventType: %d state: %d", TAG,__func__,
          EventType::kStateChanged, event.state);
      NotifyPlayerEventCallback(EventType::kStateChanged,&event,sizeof(Event));
      return BAD_VALUE;
    } else {
      QMMF_INFO("%s:%s: Stop successs!", TAG, __func__);
      setCurrentState(PlayerState::QPLAYER_STATE_STOPPED);
      event.state = PlayerState::QPLAYER_STATE_STOPPED;
      QMMF_INFO("%s:%s: EventType: %d state: %d", TAG,__func__,
          EventType::kStateChanged, event.state);
      NotifyPlayerEventCallback(EventType::kStateChanged,&event,sizeof(Event));
     }
  }

  QMMF_DEBUG("%s:%s: Stop Called in %d", TAG, __func__, current_state_);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t PlayerImpl::Pause() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  Mutex::Autolock lock(state_lock_);

  status_t ret = NO_ERROR;

  Event event;
  memset(&event,0x0,sizeof(Event));

  if (current_state_ & (PlayerState::QPLAYER_STATE_PAUSED |
      PlayerState::QPLAYER_STATE_STOPPED |
      PlayerState::QPLAYER_STATE_PLAYBACK_COMPLETED))
   return NO_ERROR;

  if (current_state_ & (PlayerState::QPLAYER_STATE_STARTED)) {

    size_t num_tracks = tracks_.size();

    for(size_t i = 0; i < num_tracks; i++) {
      if (tracks_[i].type == TrackType::kVideo) {
        ret = video_decoder_core_->PauseTrackDecoder(tracks_[i].track_id);
      } else if ((tracks_[i].type == TrackType::kAudio) && (!IsTrickModeEnabled())) {
        ret = audio_decoder_core_->PauseTrackDecoder(tracks_[i].track_id);
      }
    }

    if (ret != NO_ERROR) {
      QMMF_INFO("%s:%s: Pause failed!", TAG, __func__);
      setCurrentState(PlayerState::QPLAYER_STATE_ERROR);
      event.state = PlayerState::QPLAYER_STATE_ERROR;
      QMMF_INFO("%s:%s: EventType: %d state: %d", TAG,__func__,
          EventType::kStateChanged, event.state);
      NotifyPlayerEventCallback(EventType::kStateChanged,&event,sizeof(Event));
      return BAD_VALUE;
    } else {
      QMMF_INFO("%s:%s: Pause successs!", TAG, __func__);
      setCurrentState(PlayerState::QPLAYER_STATE_PAUSED);
      event.state = PlayerState::QPLAYER_STATE_PAUSED;
      QMMF_INFO("%s:%s: EventType: %d state: %d", TAG,__func__,
          EventType::kStateChanged, event.state);
      NotifyPlayerEventCallback(EventType::kStateChanged,&event,sizeof(Event));
    }
  }

  QMMF_DEBUG("%s:%s: Pause Called in %d", TAG, __func__, current_state_);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t PlayerImpl::Resume() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  Mutex::Autolock lock(state_lock_);
  status_t ret = NO_ERROR;

  Event event;
  memset(&event,0x0,sizeof(Event));

  if (current_state_ & (PlayerState::QPLAYER_STATE_STARTED |
      PlayerState::QPLAYER_STATE_STOPPED |
      PlayerState::QPLAYER_STATE_PLAYBACK_COMPLETED))
    return NO_ERROR;

  if (current_state_ & (PlayerState::QPLAYER_STATE_PAUSED)) {

    size_t num_tracks = tracks_.size();

    for(size_t i = 0; i < num_tracks; i++) {
      if (tracks_[i].type == TrackType::kVideo) {
        ret = video_decoder_core_->ResumeTrackDecoder(tracks_[i].track_id);
      } else if ((tracks_[i].type == TrackType::kAudio) && (!IsTrickModeEnabled())) {
        ret = audio_decoder_core_->ResumeTrackDecoder(tracks_[i].track_id);
      }
    }

    if (ret != NO_ERROR) {
      QMMF_INFO("%s:%s: Resume failed!", TAG, __func__);
      setCurrentState(PlayerState::QPLAYER_STATE_ERROR);
      event.state = PlayerState::QPLAYER_STATE_ERROR;
      QMMF_INFO("%s:%s: EventType: %d state: %d", TAG,__func__,
          EventType::kStateChanged, event.state);
      NotifyPlayerEventCallback(EventType::kStateChanged,&event,sizeof(Event));
      return BAD_VALUE;
    } else {
      QMMF_INFO("%s:%s: Resume successs!", TAG, __func__);
      setCurrentState(PlayerState::QPLAYER_STATE_STARTED);
      event.state = PlayerState::QPLAYER_STATE_STARTED;
      QMMF_INFO("%s:%s: EventType: %d state: %d", TAG,__func__,
          EventType::kStateChanged, event.state);
      NotifyPlayerEventCallback(EventType::kStateChanged,&event,sizeof(Event));
    }
  }

  QMMF_DEBUG("%s:%s: Resume Called in %d", TAG, __func__, current_state_);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t PlayerImpl::SetPosition(int64_t seek_time) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  Mutex::Autolock lock(state_lock_);
  status_t ret = NO_ERROR;
  if (current_state_ & (PlayerState::QPLAYER_STATE_PREPARED |
    PlayerState::QPLAYER_STATE_STARTED |
    PlayerState::QPLAYER_STATE_PAUSED |
    PlayerState::QPLAYER_STATE_PLAYBACK_COMPLETED)) {

    if (ret != NO_ERROR) {
      QMMF_INFO("%s:%s: SetPosition failed!", TAG, __func__);
      return BAD_VALUE;
    }
    return ret;
  }

  QMMF_DEBUG("%s:%s: SetPosition Called in %d", TAG, __func__, current_state_);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t PlayerImpl::SetTrickMode(TrickModeSpeed speed, TrickModeDirection dir) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  Mutex::Autolock lock(trick_mode_change_lock_);

  status_t ret = NO_ERROR;

  trick_mode_speed_ = speed;
  trick_mode_dir_ = dir;

  size_t num_tracks = tracks_.size();
  assert(num_tracks != 0);

  for(size_t i = 0; i < num_tracks; i++) {
    if (tracks_[i].type == TrackType::kVideo) {
      ret = video_decoder_core_->SetTrackTrickMode(tracks_[i].track_id,
          speed, dir);
    }
  }

  // normal playback
  if ((speed == TrickModeSpeed::kSpeed_1x) &&
      (dir == TrickModeDirection::kNormalForward)) {
    for(size_t i = 0; i < num_tracks; i++) {
      if (tracks_[i].type == TrackType::kAudio)
        ret = audio_decoder_core_->StartTrackDecoder(tracks_[i].track_id);
    }
  } else { // other than normal playback audio will always be stopped
    for(size_t i = 0; i < num_tracks; i++) {
      if (tracks_[i].type == TrackType::kAudio)
        ret = audio_decoder_core_->StopTrackDecoder(tracks_[i].track_id, false);
    }
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t PlayerImpl::GrabPicture(PictureParam param) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;

  BufferDescriptor pbuffer;
  memset(&pbuffer, 0x0, sizeof pbuffer);

  for (const TrackInfo& track : tracks_) {
    if (track.type == TrackType::kVideo) {
      ret = (video_sink_->GetTrackSink(track.track_id))->GrabPicture(param,
          pbuffer);
      NotifyGrabPictureDataCallback(pbuffer);
      break;
    }
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

//Audio post processing
status_t PlayerImpl::SetAudioTrackParam(uint32_t track_id,
                                        CodecParamType type,
                                        void *param,
                                        size_t param_size) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;

  size_t num_tracks = tracks_.size();

  for(size_t i = 0; i < num_tracks; i++) {
    if (tracks_[i].type == TrackType::kAudio) {
      ret = audio_decoder_core_->SetAudioTrackDecoderParams(
          tracks_[i].track_id,type, param, param_size);
    }
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

//Video Post processing
status_t PlayerImpl::SetVideoTrackParam(uint32_t track_id,
                                        CodecParamType type,
                                        void *param,
                                        size_t param_size) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;

  size_t num_tracks = tracks_.size();

  for(size_t i = 0; i < num_tracks; i++) {
     if (tracks_[i].type == TrackType::kVideo) {
       ret = video_decoder_core_->SetVideoTrackDecoderParams(
           tracks_[i].track_id,type, param, param_size);
    }
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

bool PlayerImpl::IsTrackValid(const uint32_t track_id) {
  return true;
}

void PlayerImpl::setCurrentState(PlayerState state) {
  current_state_ = state;
}

bool PlayerImpl::IsTrickModeEnabled() {
  Mutex::Autolock lock(trick_mode_change_lock_);
  if ((trick_mode_speed_ == TrickModeSpeed::kSpeed_1x) &&
      (trick_mode_dir_ == TrickModeDirection::kNormalForward)) {
    return false;
  } else {
    return true;
  }
}

void PlayerImpl::NotifyPlayerEventCallback(EventType event_type,
                                           void *event_data,
                                           size_t event_data_size) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  remote_cb_->NotifyPlayerEvent(event_type,event_data,event_data_size);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

void PlayerImpl::NotifyVideoTrackDataCallback(
    uint32_t track_id,
    std::vector<BnTrackBuffer> &buffers,
    void *meta_param, TrackMetaBufferType meta_type,
    size_t meta_size) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  remote_cb_->NotifyVideoTrackData(track_id,buffers,meta_param,meta_type,
      meta_size);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

void PlayerImpl::NotifyVideoTrackEventCallback(uint32_t track_id,
                                               EventType event_type,
                                               void *event_data,
                                               size_t event_data_size) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  remote_cb_->NotifyVideoTrackEvent(track_id,event_type,event_data,
      event_data_size);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

void PlayerImpl::NotifyAudioTrackDataCallback(
    uint32_t track_id,
    std::vector<BnTrackBuffer> &buffers,
    void *meta_param, TrackMetaBufferType meta_type,
    size_t meta_size) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  remote_cb_->NotifyAudioTrackData(track_id,buffers,meta_param,meta_type,
      meta_size);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

void PlayerImpl::NotifyAudioTrackEventCallback(uint32_t track_id,
                                               EventType event_type,
                                               void *event_data,
                                               size_t event_data_size) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  remote_cb_->NotifyAudioTrackEvent(track_id,event_type,event_data,
      event_data_size);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

void PlayerImpl::NotifyDeleteAudioTrackCallback(uint32_t track_id) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  remote_cb_->NotifyDeleteAudioTrack(track_id);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

void PlayerImpl::NotifyDeleteVideoTrackCallback(uint32_t track_id) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  remote_cb_->NotifyDeleteVideoTrack(track_id);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

void PlayerImpl::NotifyGrabPictureDataCallback(BufferDescriptor& buffer){
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  remote_cb_->NotifyGrabPictureData(buffer);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

};  // namespace player
};  // namespace qmmf
