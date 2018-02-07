/*
 * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#define TAG "RecorderImpl"

#include "recorder/src/client/qmmf_recorder_params_internal.h"
#include "recorder/src/service/qmmf_recorder_impl.h"

#ifdef LOG_LEVEL_KPI
volatile uint32_t kpi_debug_level = BASE_KPI_FLAG;
#endif

namespace qmmf {

namespace recorder {

using ::std::shared_ptr;

RecorderImpl* RecorderImpl::instance_ = nullptr;

RecorderImpl* RecorderImpl::CreateRecorder() {

  if(!instance_) {
    instance_ = new RecorderImpl;
    if(!instance_) {
      QMMF_ERROR("%s:%s: Can't Create Recorder Instance!", TAG, __func__);
      return NULL;
    }
  }
  QMMF_INFO("%s:%s: Recorder Instance Created Successfully(0x%p)", TAG,
      __func__, instance_);
  return instance_;
}

RecorderImpl::RecorderImpl()
  : unique_session_id_(0),
    camera_source_(nullptr),
    encoder_core_(nullptr),
    audio_source_(nullptr),
    audio_encoder_core_(nullptr),
    client_died_(false) {

    QMMF_KPI_GET_MASK();
    QMMF_KPI_DETAIL();
    QMMF_INFO("%s:%s: Enter", TAG, __func__);
    QMMF_INFO("%s:%s: Exit", TAG, __func__);
  }

RecorderImpl::~RecorderImpl() {

  QMMF_KPI_DETAIL();
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  if (camera_source_) {
    delete camera_source_;
    camera_source_ = nullptr;
  }
  if (encoder_core_) {
    delete encoder_core_;
    encoder_core_ = nullptr;
  }
  if (audio_source_) {
    delete audio_source_;
    audio_source_ = nullptr;
  }
  if (audio_encoder_core_) {
    delete audio_encoder_core_;
    audio_encoder_core_ = nullptr;
  }
  instance_ = nullptr;
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

status_t RecorderImpl::Init(const RemoteCallbackHandle& remote_cb_handle) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  QMMF_KPI_DETAIL();

  assert(remote_cb_handle != nullptr);
  remote_cb_handle_ = remote_cb_handle;

  camera_source_ = CameraSource::CreateCameraSource();
  if (!camera_source_) {
    QMMF_ERROR("%s:%s: Can't Create CameraSource Instance!", TAG, __func__);
    return NO_MEMORY;
  }
  QMMF_INFO("%s:%s: CameraSource Instance Created Successfully!", TAG,
      __func__);

  encoder_core_ = EncoderCore::CreateEncoderCore();
  if (!encoder_core_) {
    QMMF_ERROR("%s:%s: Can't Create EncoderCore Instance!", TAG, __func__);
    return NO_MEMORY;
  }
  QMMF_INFO("%s:%s: EncoderCore Instance Created Successfully!", TAG,
      __func__);

  audio_source_ = AudioSource::CreateAudioSource();
  if (!audio_source_) {
    QMMF_ERROR("%s:%s: Can't Create AudioSource Instance!", TAG, __func__);
    return NO_MEMORY;
  }
  QMMF_INFO("%s:%s: AudioSource Instance Created Successfully!", TAG,
      __func__);

  audio_encoder_core_ = AudioEncoderCore::CreateAudioEncoderCore();
  if (!audio_encoder_core_) {
    QMMF_ERROR("%s:%s: Can't Create AudioEncoderCore Instance!", TAG, __func__);
    return NO_MEMORY;
  }
  QMMF_INFO("%s:%s: AudioEncoderCore Instance Created Successfully!", TAG,
      __func__);

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t RecorderImpl::DeInit() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  QMMF_KPI_DETAIL();

  if (camera_source_) {
    delete camera_source_;
    camera_source_ = nullptr;
  }
  if (encoder_core_) {
    delete encoder_core_;
    encoder_core_ = nullptr;
  }
  if (audio_source_) {
    delete audio_source_;
    audio_source_ = nullptr;
  }
  if (audio_encoder_core_) {
    delete audio_encoder_core_;
    audio_encoder_core_ = nullptr;
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t RecorderImpl::RegisterClient(const uint32_t client_id) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto iter = client_session_map_.find(client_id);
  if (iter != client_session_map_.end()) {
    QMMF_WARN("%s:%s: Client is already connected !!", TAG,
        __func__);
    return NO_ERROR;
  }
  client_session_map_.insert( {client_id, SessionTrackMap()} );
  client_cameraid_map_.insert( {client_id, std::vector<uint32_t>()} );

  QMMF_INFO("%s:%s:client_session_map_.size(%d)", TAG, __func__,
      client_session_map_.size());
  SessionTrackMap session_track_map = client_session_map_[client_id];
  QMMF_INFO("%s:%s:session_track_map.size(%d)", TAG, __func__,
      session_track_map.size());

  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return NO_ERROR;
}

status_t RecorderImpl::DeRegisterClient(const uint32_t client_id,
                                        bool force_cleanup) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  status_t ret = NO_ERROR;
  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s client_id(%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  client_session_lock_.lock();
  auto session_track_map = client_session_map_[client_id];
  client_session_lock_.unlock();
  if (session_track_map.size() > 0) {
    if (!force_cleanup) {
      QMMF_WARN("%s:%s Resource belogs to client(%d) are not released!", TAG,
          __func__, client_id);
      return INVALID_OPERATION;
    } else {
      // This is the case when client is dead before releasing its acquired
      // resources, service is trying to free up his resources to avoid
      // affecting other connected clients, worst case if this doesn't help
      // then micro restart is the only option left.
      QMMF_INFO("%s:%s: triggering force cleanup for dead client(%d)", TAG,
        __func__, client_id);
      {
        // Raise the client_died_ flag in order to signal the audio/video
        // track callbacks to return the buffers from where they originated.
        std::lock_guard<std::mutex> lock(client_died_lock_);
        client_died_ = true;
      }

      for (auto session : session_track_map) {
        auto session_id = session.first;
        ret = StopSession(client_id, session_id, false, true);
        if (ret != NO_ERROR) {
          QMMF_WARN("%s:%s: Internal stop session is failed!,"
            " client_id(%d):session_id(%d)", TAG, __func__,
            client_id, session_id);
          // Carry-on even stop session fails.
        }
       std::vector<TrackTuple>::iterator track = session.second.end();
        while (track != session.second.begin()) {
          --track;
          uint32_t client_track_id  = std::get<0>(*track);
          uint32_t service_track_id  = std::get<1>(*track);
          TrackInfo track_info      = std::get<2>(*track);
          QMMF_INFO("%s:%s: Track to Delete, client_id(%d):session_id(%d), "
              "client_track_id(%d):service_track_id(%x)", TAG, __func__,
              client_id, session_id, client_track_id, service_track_id);
          if (track_info.type == TrackType::kVideo) {
            ret = DeleteVideoTrack(client_id, session_id, client_track_id);
          } else {
            ret = DeleteAudioTrack(client_id, session_id, client_track_id);
          }
          // Carry-on even delete track fails.
        }
        ret = DeleteSession(client_id, session_id);
        if (ret != NO_ERROR) {
          QMMF_WARN("%s:%s: Internal delete session is failed!,"
            " client_id(%d):session_id(%d)", TAG, __func__,
            client_id, session_id);
        }
      }
      QMMF_INFO("%s:%s: Number of sessions(%d) left after cleanup for"
        " client(%d)!", TAG, __func__, client_session_map_[client_id].size(),
        client_id);
      // Close ownded cameras.
      for (auto iter : client_cameraid_map_) {
        std::vector<uint32_t> camera_ids = iter.second;
        QMMF_INFO("%s:%s: client(%d) owning num cameras(%d)", TAG, __func__,
            client_id, camera_ids.size());
        for (auto camera : camera_ids) {
          ret = StopCamera(client_id, camera);
          if (ret != NO_ERROR) {
            QMMF_INFO("%s:%s: client_id(%d) camera id(%d) close failed!", TAG,
                __func__, client_id, camera);
            // Go ahead with removing camera id from map.
          }
        }
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(camera_map_lock_);
    client_session_map_.erase(client_id);
    client_cameraid_map_.erase(client_id);
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return NO_ERROR;
}

status_t RecorderImpl::StartCamera(const uint32_t client_id,
                                   const uint32_t camera_id,
                                   const CameraStartParam &param,
                                   bool enable_result_cb) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  QMMF_KPI_DETAIL();
  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client, Not in connected client list!", TAG,
        __func__);
    return BAD_VALUE;
  }
  {
    std::lock_guard<std::mutex> lock(camera_map_lock_);
    // Check if camera is owned by other client.
    for (auto iter : client_cameraid_map_) {
      std::vector<uint32_t> camera_ids = iter.second;
      for (auto idx : camera_ids) {
        if (camera_id == idx) {
          QMMF_ERROR("%s:%s client_id(%d) Camera (%d) is already "
                "owned by other client, Not allowed!", TAG, __func__, client_id,
                camera_id);
          return INVALID_OPERATION;
        }
      }
    }
  }
  assert(camera_source_ != nullptr);
  ResultCb cb = [ this, client_id ] (uint32_t camera_id,
      const CameraMetadata &result) {
        CameraResultCallback(client_id, camera_id, result);
      };

  ErrorCb errcb = [ this, client_id ] (RecorderErrorData &error) {
        CameraErrorCallback(client_id, error);
      };

  auto ret = camera_source_->StartCamera(camera_id, param,
                                         enable_result_cb ? cb : nullptr,
                                         errcb);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: StartCamera Failed!!", TAG, __func__);
    return BAD_VALUE;
  }
  std::lock_guard<std::mutex> lock(camera_map_lock_);
  auto& camera_ids = client_cameraid_map_[client_id];
  camera_ids.push_back(camera_id);

  QMMF_INFO("%s:%s: number of clients connected(%d)", TAG, __func__,
      client_cameraid_map_.size());
  for (auto iter : client_cameraid_map_) {
    auto camera_ids = iter.second;
    QMMF_INFO("%s:%s client_id(%d): number of cameras(%d) owned!", TAG,
        __func__, client_id, camera_ids.size());
    for (auto idx : camera_ids) {
      QMMF_INFO("%s:%s \t client_id(%d): camera_id(%d)", TAG, __func__,
          client_id, idx);
    }
  }
  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t RecorderImpl::StopCamera(const uint32_t client_id,
                                  const uint32_t camera_id) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  QMMF_KPI_DETAIL();
  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client, Not in connected client list!", TAG,
        __func__);
    return BAD_VALUE;
  }
  if (!IsCameraOwned(client_id, camera_id)) {
    QMMF_ERROR("%s:%s client_id(%d) Camera (%d) is already "
        "owned by other client, operation not allowed!", TAG, __func__,
        client_id, camera_id);
    return INVALID_OPERATION;
  }
  assert(camera_source_ != nullptr);

  auto ret = camera_source_->StopCamera(camera_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: StopCamera Failed!!", TAG, __func__);
    return BAD_VALUE;
  }
  std::lock_guard<std::mutex> lock(camera_map_lock_);
  auto& camera_id_vector = client_cameraid_map_[client_id];
  auto camera_id_iter = std::find(camera_id_vector.begin(),
      camera_id_vector.end(), camera_id);
  camera_id_vector.erase(camera_id_iter);
  QMMF_INFO("%s:%s client_id(%d): number of cameras(%d)", TAG, __func__,
      client_id, camera_id_vector.size());

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t RecorderImpl::CreateSession(const uint32_t client_id,
                                     uint32_t *session_id) {
  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  QMMF_KPI_DETAIL();

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client, Not in connected client list!", TAG,
        __func__);
    return BAD_VALUE;
  }
  if(!camera_source_) {
    QMMF_ERROR("%s:%s: Can't Create Session! Connect Should be called before"
        " Calling CreateSession", TAG, __func__);
    return NO_INIT;
  }
  std::lock_guard<std::mutex> lock(client_session_lock_);
  ++unique_session_id_;
  *session_id = unique_session_id_;

  auto& session_track_map = client_session_map_[client_id];
  session_track_map.insert( {*session_id, std::vector<TrackTuple>()} );

  sessions_state_.insert( {*session_id, false} );
  QMMF_INFO("%s:%s: client_id(%d):session_id(%d) ", TAG, __func__, client_id,
      *session_id);

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t RecorderImpl::DeleteSession(const uint32_t client_id,
                                     const uint32_t session_id) {
  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  QMMF_KPI_DETAIL();

  int32_t ret = NO_ERROR;
  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client, Not in connected client list!", TAG,
        __func__);
    return BAD_VALUE;
  }
  if (!IsSessionIdValid(client_id, session_id)) {
    QMMF_ERROR("%s:%s: session_id is not valid!", TAG, __func__);
    return BAD_VALUE;
  }
  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto& session_track_map = client_session_map_[client_id];
  std::vector<TrackTuple>& tracks = session_track_map[session_id];
  if (tracks.size() > 0) {
    QMMF_ERROR("%s:%s: ClientId(%d) Session(%d) Can't be deleted until all"
        " tracks(%d) within this session are stopped & deleted!", TAG, __func__,
        client_id, session_id, tracks.size());
    return INVALID_OPERATION;
  }
  session_track_map.erase(session_id);
  QMMF_INFO("%s:%s: Number of sessions(%d) left in client_id(%d)", TAG,
      __func__, session_track_map.size(), client_id);
  sessions_state_.erase(session_id);

  if (session_track_map.size() == 0) {
    uint32_t num_sessions = 0;
    for (auto client_iter : client_session_map_) {
        QMMF_INFO("%s:%s: client_id(%d):num_sessions(%d)", TAG, __func__,
            client_iter.first, client_iter.second.size());
        num_sessions += client_iter.second.size();
    }
    if (num_sessions == 0) {
      QMMF_INFO("%s:%s: Reseting unique session id to 0!", TAG, __func__);
      unique_session_id_ = 0;
    }
  }
  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t RecorderImpl::StartSession(const uint32_t client_id,
                                    const uint32_t session_id) {
  QMMF_DEBUG("%s:%s: Enter client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  QMMF_KPI_DETAIL();

  uint32_t ret = NO_ERROR;
  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client_id(%d), Not in connected client list!",
        TAG, __func__, client_id);
    return BAD_VALUE;
  }
  if (!IsSessionIdValid(client_id, session_id)) {
    QMMF_ERROR("%s:%s: session_id(%d) is not valid!", TAG, __func__, session_id);
    return BAD_VALUE;
  }
  if (!IsSessionValid(client_id, session_id)) {
    QMMF_ERROR("%s:%s: session_id(%d) is not valid Or No track is associated"
        " with it", TAG, __func__, session_id);
    return BAD_VALUE;
  }
  if (IsSessionStarted(session_id)) {
    QMMF_INFO("%s:%s: session_id(%d) is already started!", TAG, __func__,
        session_id);
    return NO_ERROR;
  }
  client_session_lock_.lock();
  auto session_track_map = client_session_map_[client_id];
  auto tracks_in_session = session_track_map[session_id];
  client_session_lock_.unlock();

  QMMF_INFO("%s:%s: client_id(%d):session_id(%d) number of tracks(%d) to start",
      TAG, __func__, client_id, session_id, tracks_in_session.size());
  // All the tracks associated to one session starts together.
  for(auto track : tracks_in_session) {

    uint32_t client_track_id  = std::get<0>(track);
    uint32_t service_track_id = std::get<1>(track);
    TrackInfo track_info      = std::get<2>(track);
    QMMF_INFO("%s:%s: Track to Start, client_id(%d):session_id(%d), "
        "client_track_id(%d):service_track_id(%x)", TAG, __func__, client_id,
        session_id, client_track_id, service_track_id);
    if (track_info.type == TrackType::kVideo) {

      assert(camera_source_ != nullptr);
      ret = camera_source_->StartTrackSource(service_track_id);
      if (ret != NO_ERROR) {
        ret = BAD_VALUE;
        QMMF_ERROR("%s:%s: client_id(%d):session_id(%d), StartTrackSource"
            " failed for client_track_id(%d):service_track_id(%x)", TAG,
            __func__, client_id, session_id, client_track_id, service_track_id);
        break;
      }
      VideoFormat fmt_type = track_info.video_params.params.format_type;
      if ( (fmt_type == VideoFormat::kHEVC) ||
           (fmt_type == VideoFormat::kAVC) ) {
        assert(encoder_core_ != nullptr);
        ret = encoder_core_->StartTrackEncoder(service_track_id);
        // Initial debug purpose.
        assert(ret == NO_ERROR);
        if (ret != NO_ERROR) {
          ret = BAD_VALUE;
          QMMF_ERROR("%s:%s: client_id(%d):session_id(%d), StartTrackEncoder"
              " failed for client_track_id(%d):service_track_id(%x)", TAG,
              __func__, client_id, session_id, client_track_id,
              service_track_id);
          break;
        }
      }
    }
    else if (track_info.type == TrackType::kAudio) {

      assert(audio_source_ != nullptr);
      ret = audio_source_->StartTrackSource(service_track_id);
      if (ret != NO_ERROR) {
          QMMF_ERROR("%s:%s: client_id(%d):session_id(%d), "
              "audio->StartTrackSource failed for "
              " client_track_id(%d):service_track_id(%x)", TAG, __func__,
              client_id, session_id, client_track_id, service_track_id);
        ret = BAD_VALUE;
        break;
      }
      if (track_info.audio_params.params.format != AudioFormat::kPCM) {

        assert(audio_encoder_core_ != nullptr);
        shared_ptr<IAudioTrackSource> track_source;
        ret = audio_source_->getTrackSource(service_track_id, &track_source);
        if (ret != NO_ERROR || track_source == nullptr) {
          QMMF_ERROR("%s:%s: client_id(%d):session_id(%d), "
              "audio->getTrackSource failed for "
              " client_track_id(%d):service_track_id(%x)", TAG, __func__,
              client_id, session_id, client_track_id, service_track_id);
          return BAD_VALUE;
        }
        ret = audio_encoder_core_->StartTrackEncoder(service_track_id,
                                                     track_source);
        if (ret != NO_ERROR) {
          QMMF_ERROR("%s:%s: client_id(%d):session_id(%d), "
              "audio->StartTrackEncoder failed for "
              "client_track_id(%d):service_track_id(%x)", TAG, __func__,
              client_id, session_id, client_track_id, service_track_id);
          ret = BAD_VALUE;
          break;
        }
      }
    }
    QMMF_INFO("%s:%s: client_id(%d):session_id(%d), "
        "client_track_id(%d):service_track_id(%x) Started Successfully!", TAG,
        __func__, client_id, session_id, client_track_id, service_track_id);
  } // tracks loop ends.

  if(ret == NO_ERROR) {
    QMMF_INFO("%s:%s: client_id(%d):session_id(%d) with num tracks(%d) Started"
        " Successfully!", TAG, __func__, client_id, session_id,
        tracks_in_session.size());

    auto& started = sessions_state_[session_id];
    started = true;

    QMMF_INFO("%s:%s: session status=%d", TAG, __func__,
        sessions_state_[session_id]);
  }
  QMMF_DEBUG("%s:%s: Exit client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  return ret;
}

status_t RecorderImpl::StopSession(const uint32_t client_id,
                                   const uint32_t session_id, bool do_flush,
                                   bool is_force_cleanup) {

  QMMF_DEBUG("%s:%s: Enter client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  QMMF_KPI_DETAIL();

  uint32_t ret = NO_ERROR;
  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client_id(%d), Not in connected client list!",
        TAG, __func__, client_id);
    return BAD_VALUE;
  }
  if (!IsSessionIdValid(client_id, session_id)) {
    QMMF_ERROR("%s:%s: session_id(%d) is not valid!", TAG, __func__,
        session_id);
    return BAD_VALUE;
  }
  if (!IsSessionValid(client_id, session_id)) {
    QMMF_ERROR("%s:%s: session_id(%d) is not valid Or No track is associated"
        " with it", TAG, __func__, session_id);
    return BAD_VALUE;
  }
  if (!IsSessionStarted(session_id)) {
    QMMF_INFO("%s:%s: session_id(%d) is not started yet!", TAG, __func__,
              session_id);
    return NO_ERROR;
  }
  client_session_lock_.lock();
  auto session_track_map = client_session_map_[client_id];
  auto tracks_in_session = session_track_map[session_id];
  client_session_lock_.unlock();

  QMMF_INFO("%s:%s: client_id(%d):session_id(%d), number of tracks(%d) to stop",
      TAG, __func__, client_id, session_id, tracks_in_session.size());
  // All the tracks associated to one session starts together.

  std::vector<TrackTuple>::iterator track = tracks_in_session.end();
  while (track != tracks_in_session.begin()) {
    --track;
    uint32_t client_track_id  = std::get<0>(*track);
    uint32_t service_track_id = std::get<1>(*track);
    TrackInfo track_info      = std::get<2>(*track);
    QMMF_INFO("%s:%s: Track to Stop, client_id(%d):session_id(%d), "
        "client_track_id(%d):service_track_id(%x)", TAG, __func__, client_id,
        session_id, client_track_id, service_track_id);

    if (track_info.type == TrackType::kVideo) {
      // Stop TrackSource
      assert(camera_source_ != nullptr);
      ret = camera_source_->StopTrackSource(service_track_id, is_force_cleanup);
      if (ret != NO_ERROR) {
        ret = BAD_VALUE;
        QMMF_ERROR("%s:%s: client_id(%d):session_id(%d), StopTrackSource"
            " failed for client_track_id(%d):service_track_id(%x)", TAG,
            __func__, client_id, session_id, client_track_id,
            service_track_id);
        break;
      }
      // Stop TrackEncoder
      VideoFormat fmt_type = track_info.video_params.params.format_type;
      if ((fmt_type == VideoFormat::kHEVC) || (fmt_type == VideoFormat::kAVC)) {
        assert(encoder_core_ != nullptr);
        ret = encoder_core_->StopTrackEncoder(service_track_id,
                                              is_force_cleanup);
        if (ret != NO_ERROR) {
          ret = BAD_VALUE;
          QMMF_ERROR("%s:%s: client_id(%d):session_id(%d), StopTrackEncoder"
            " failed for client_track_id(%d):service_track_id(%x)", TAG,
            __func__, client_id, session_id, client_track_id,
            service_track_id);
          break;
        }
      }
    } else if (track_info.type == TrackType::kAudio) {

      assert(audio_source_ != nullptr);
      ret = audio_source_->StopTrackSource(service_track_id);
      if (ret != NO_ERROR) {
        ret = BAD_VALUE;
        QMMF_ERROR("%s:%s: client_id(%d):session_id(%d), audio->StopTrackSource"
            " failed for client_track_id(%d):service_track_id(%x)", TAG,
            __func__, client_id, session_id, client_track_id,
            service_track_id);
        break;
      }
      if (track_info.audio_params.params.format != AudioFormat::kPCM) {
        assert(audio_encoder_core_ != nullptr);
        ret = audio_encoder_core_->StopTrackEncoder(service_track_id);
        if (ret != NO_ERROR) {
          ret = BAD_VALUE;
          QMMF_ERROR("%s:%s:client_id(%d):session_id(%d), StopTrackEncoder"
            " failed for client_track_id(%d):service_track_id(%x)", TAG,
            __func__, client_id, session_id, client_track_id, service_track_id);
          break;
        }
      }
    }
    QMMF_INFO("%s:%s: client_id(%d):session_id(%d), "
        "client_track_id(%d):service_track_id(%x) Stoped Successfully!", TAG,
        __func__, client_id, session_id, client_track_id, service_track_id);
  } // tracks loop ends.

   if(ret == NO_ERROR) {
    QMMF_INFO("%s:%s: client_id(%d):session_id(%d) with num tracks(%d) Stoped"
        " Successfully!", TAG, __func__, client_id, session_id,
        tracks_in_session.size());

    auto& started = sessions_state_[session_id];
    started = false;

    QMMF_INFO("%s:%s: session status=%d", TAG, __func__,
        sessions_state_[session_id]);
  }
  QMMF_DEBUG("%s:%s: Exit client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  return ret;
}

status_t RecorderImpl::PauseSession(const uint32_t client_id,
                                    const uint32_t session_id) {
  QMMF_DEBUG("%s:%s: Enter client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  QMMF_KPI_DETAIL();

  uint32_t ret = NO_ERROR;
  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client_id(%d), Not in connected client list!",
        TAG, __func__, client_id);
    return BAD_VALUE;
  }
  if (!IsSessionIdValid(client_id, session_id)) {
    QMMF_ERROR("%s:%s: session_id(%d) is not valid!", TAG, __func__,
        session_id);
    return BAD_VALUE;
  }
  if (!IsSessionValid(client_id, session_id)) {
    QMMF_ERROR("%s:%s: session_id(%d) is not valid Or No track is associated"
        " with it", TAG, __func__, session_id);
    return BAD_VALUE;
  }
  if (!IsSessionStarted(session_id)) {
    QMMF_INFO("%s:%s: session_id(%d) is not started yet!", TAG, __func__,
              session_id);
    return INVALID_OPERATION;
  }
  client_session_lock_.lock();
  auto session_track_map = client_session_map_[client_id];
  auto tracks_in_session = session_track_map[session_id];
  client_session_lock_.unlock();

  QMMF_INFO("%s:%s: client_id(%d):session_id(%d),number of tracks(%d) to Pause",
      TAG, __func__, client_id, session_id, tracks_in_session.size());
  // All the tracks associated to one session starts together.
  for(auto track : tracks_in_session) {

    uint32_t client_track_id  = std::get<0>(track);
    uint32_t service_track_id = std::get<1>(track);
    TrackInfo track_info      = std::get<2>(track);
    QMMF_INFO("%s:%s: Track to Pause, client_id(%d):session_id(%d), "
        "client_track_id(%d):service_track_id(%x)", TAG, __func__, client_id,
        session_id, client_track_id, service_track_id);
    if (track_info.type == TrackType::kVideo) {
      assert(camera_source_ != nullptr);
      ret = camera_source_->PauseTrackSource(service_track_id);
      if (ret != NO_ERROR) {
        ret = BAD_VALUE;
        QMMF_ERROR("%s:%s: client_id(%d):session_id(%d), PauseTrackSource"
            " failed for client_track_id(%d):service_track_id(%x)", TAG,
            __func__, client_id, session_id, client_track_id,
            service_track_id);
        break;
      }
      VideoFormat fmt_type = track_info.video_params.params.format_type;
      if ( (fmt_type == VideoFormat::kHEVC) ||
           (fmt_type == VideoFormat::kAVC) ) {
        //TODO: Add logic to stop TrackEncoder
      }

    } else if (track_info.type == TrackType::kAudio) {

      assert(audio_source_ != nullptr);
      ret = audio_source_->PauseTrackSource(service_track_id);
      if (ret != NO_ERROR) {
        ret = BAD_VALUE;
        QMMF_ERROR("%s:%s:client_id(%d):session_id(%d), audio->PauseTrackSource"
            " failed for client_track_id(%d):service_track_id(%x)", TAG,
            __func__, client_id, session_id, client_track_id,
            service_track_id);
        break;
      }
      if (track_info.audio_params.params.format != AudioFormat::kPCM) {

        assert(audio_encoder_core_ != nullptr);
        ret = audio_encoder_core_->PauseTrackEncoder(track_info.track_id);
        if (ret != NO_ERROR) {
          ret = BAD_VALUE;
          QMMF_ERROR("%s:%s: client_id(%d):session_id(%d), "
            "audio->PauseTrackEncoder failed for client_track_id(%d):"
            "service_track_id(%x)", TAG, __func__, client_id, session_id,
            client_track_id, service_track_id);
          break;
        }
      }
    }
  }
  if(ret == NO_ERROR) {
    QMMF_INFO("%s:%s: client_id(%d):session_id(%d) with num tracks(%d) Paused"
        " Successfully!", TAG, __func__, client_id, session_id,
        tracks_in_session.size());
  }
  QMMF_DEBUG("%s:%s: Exit client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  return ret;
}

status_t RecorderImpl::ResumeSession(const uint32_t client_id,
                                     const uint32_t session_id) {
  QMMF_DEBUG("%s:%s: Enter client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  QMMF_KPI_DETAIL();

  uint32_t ret = NO_ERROR;
  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client_id(%d), Not in connected client list!",
        TAG, __func__, client_id);
    return BAD_VALUE;
  }
  if (!IsSessionIdValid(client_id, session_id)) {
    QMMF_ERROR("%s:%s: session_id(%d) is not valid!", TAG, __func__,
        session_id);
    return BAD_VALUE;
  }
  if (!IsSessionValid(client_id, session_id)) {
    QMMF_ERROR("%s:%s: session_id(%d) is not valid Or No track is associated"
        " with it", TAG, __func__, session_id);
    return BAD_VALUE;
  }
  client_session_lock_.lock();
  auto session_track_map = client_session_map_[client_id];
  auto tracks_in_session = session_track_map[session_id];
  client_session_lock_.unlock();

  QMMF_INFO("%s:%s:client_id(%d):session_id(%d),number of tracks(%d) to Resume",
      TAG, __func__, client_id, session_id, tracks_in_session.size());
  // All the tracks associated to one session starts together.
  for(auto track : tracks_in_session) {

    uint32_t client_track_id  = std::get<0>(track);
    uint32_t service_track_id = std::get<1>(track);
    TrackInfo track_info      = std::get<2>(track);
    QMMF_INFO("%s:%s: Track to Resume, client_id(%d):session_id(%d), "
        "client_track_id(%d):service_track_id(%x)", TAG, __func__, client_id,
        session_id, client_track_id, service_track_id);
    if (track_info.type == TrackType::kVideo) {
      assert(camera_source_ != nullptr);
      ret = camera_source_->ResumeTrackSource(service_track_id);
      if (ret != NO_ERROR) {
        ret = BAD_VALUE;
        QMMF_ERROR("%s:%s: client_id(%d):session_id(%d), ResumeTrackSource"
            " failed for client_track_id(%d):service_track_id(%x)", TAG,
            __func__, client_id, session_id, client_track_id,
            service_track_id);
        break;
      }
      VideoFormat fmt_type = track_info.video_params.params.format_type;
      if ( (fmt_type == VideoFormat::kHEVC) ||
           (fmt_type == VideoFormat::kAVC) ) {
        //TODO: Add logic to resume TrackEncoder
      }

    } else if (track_info.type == TrackType::kAudio) {

      assert(audio_source_ != nullptr);
      ret = audio_source_->ResumeTrackSource(service_track_id);
      if (ret != NO_ERROR) {
        ret = BAD_VALUE;
        QMMF_ERROR("%s:%s:client_id(%d):session_id(%d),audio->ResumeTrackSource"
            " failed for client_track_id(%d):service_track_id(%x)", TAG,
            __func__, client_id, session_id, client_track_id,
            service_track_id);
        break;
      }
      if (track_info.audio_params.params.format != AudioFormat::kPCM) {

        assert(audio_encoder_core_ != nullptr);
        ret = audio_encoder_core_->ResumeTrackEncoder(track_info.track_id);
        if (ret != NO_ERROR) {
          ret = BAD_VALUE;
          QMMF_ERROR("%s:%s: client_id(%d):session_id(%d), "
            "audio->ResumeTrackEncoder failed for client_track_id(%d):"
            "service_track_id(%x)", TAG, __func__, client_id, session_id,
            client_track_id, service_track_id);
          break;
        }
      }
    }
  }
  if(ret == NO_ERROR) {
    QMMF_INFO("%s:%s: client_id(%d):session_id(%d) with num tracks(%d) Resumed"
        " Successfully!", TAG, __func__, client_id, session_id,
        tracks_in_session.size());
  }
  QMMF_DEBUG("%s:%s: Exit client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  return ret;
}

status_t RecorderImpl::GetSupportedPlugins(const uint32_t client_id,
                                           SupportedPlugins *plugins) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client_id(%d), Not in connected client list!",
        TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(camera_source_ != nullptr);
  auto ret = camera_source_->GetSupportedPlugins(plugins);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: client_id(%d): GetSupportedPlugins failed!", TAG,
        __func__, client_id);
    return ret;
  }

  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return NO_ERROR;
}

status_t RecorderImpl::CreatePlugin(const uint32_t client_id, uint32_t *uid,
                                    const PluginInfo &plugin) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client_id(%d), Not in connected client list!",
        TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(camera_source_ != nullptr);
  auto ret = camera_source_->CreatePlugin(uid, plugin);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: client_id(%d): CreatePlugin %s failed!", TAG, __func__,
        client_id, plugin.name.c_str());
    return ret;
  }
  QMMF_INFO("%s:%s: client_id(%d): Plugin %s created uid(%d)", TAG, __func__,
      client_id, plugin.name.c_str(), *uid);

  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return NO_ERROR;
}

status_t RecorderImpl::DeletePlugin(const uint32_t client_id,
                                    const uint32_t &uid) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client_id(%d), Not in connected client list!",
        TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(camera_source_ != nullptr);
  auto ret = camera_source_->DeletePlugin(uid);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: client_id(%d):DeletePlugin uid(%d) failed!", TAG, __func__,
        client_id, uid);
    return ret;
  }

  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return NO_ERROR;
}

status_t RecorderImpl::ConfigPlugin(const uint32_t client_id,
                                    const uint32_t &uid,
                                    const std::string &json_config) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client_id(%d), Not in connected client list!",
        TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(camera_source_ != nullptr);
  auto ret = camera_source_->ConfigPlugin(uid, json_config);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: client_id(%d): ConfigPlugin uid(%d) failed!", TAG,
        __func__, client_id, uid);
    return ret;
  }

  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return NO_ERROR;
}

status_t RecorderImpl::CreateAudioTrack(const uint32_t client_id,
                                        const uint32_t session_id,
                                        const uint32_t track_id,
                                        const AudioTrackCreateParam& param) {
  QMMF_DEBUG("%s:%s: Enter client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  QMMF_KPI_DETAIL();

  uint32_t ret = NO_ERROR;
  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client_id(%d), Not in connected client list!",
        TAG, __func__, client_id);
    return BAD_VALUE;
  }
  if (!IsSessionIdValid(client_id, session_id)) {
    QMMF_ERROR("%s:%s: session_id(%d) is not valid!", TAG, __func__,
        session_id);
    return BAD_VALUE;
  }
  uint32_t service_track_id = GetUniqueServiceTrackId(client_id, session_id,
                                                      track_id);
  QMMF_INFO("%s:%s: client_id(%d):session_id(%d) client_track_id(%d):"
      "service_track_id(%x)", TAG, __func__, client_id, session_id, track_id,
      service_track_id);

  AudioTrackParams audio_track_params;
  memset(&audio_track_params, 0x00, sizeof audio_track_params);
  audio_track_params.track_id = service_track_id;
  audio_track_params.params   = param;
  audio_track_params.data_cb  = [this, client_id, session_id, track_id]
      (std::vector<BnBuffer>& buffers, std::vector<MetaData>& meta_buffers) {
          AudioTrackBufferCallback(client_id, session_id, track_id, buffers,
                                   meta_buffers);
      };

  assert(audio_source_ != nullptr);
  ret = audio_source_->CreateTrackSource(service_track_id,
                                            audio_track_params);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreateTrackSource track_id(%d):service_track_id(%x) "
        " failed!", TAG, __func__, track_id, service_track_id);
    return BAD_VALUE;
  }
  QMMF_INFO("%s:%s: client_id(%d):session_id(%d), TrackSource for"
      "client_track_id(%d):service_track_id(%x) Added Successfully in "
      "CameraSource", TAG, __func__, client_id, session_id, track_id,
      service_track_id);

  if (param.format != AudioFormat::kPCM) {
    assert(audio_encoder_core_ != NULL);

    ret = audio_encoder_core_->AddSource(audio_track_params);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: track_id(%d):service_track_id(%x) AddSource"
        " failed!", TAG, __func__, track_id, service_track_id);
      return BAD_VALUE;
    }
  }
  // Assosiate track to session.
  TrackInfo track_info;
  memset(&track_info, 0x0, sizeof track_info);
  track_info.track_id     = service_track_id;
  track_info.type         = TrackType::kAudio;
  track_info.audio_params = audio_track_params;

  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto& session_track_map = client_session_map_[client_id];
  auto& tracks_in_session = session_track_map[session_id];
  auto track_tuple = std::make_tuple(track_id, service_track_id, track_info);
  tracks_in_session.push_back(track_tuple);

  QMMF_DEBUG("%s:%s: Exit client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  return NO_ERROR;
}

status_t RecorderImpl::DeleteAudioTrack(const uint32_t client_id,
                                        const uint32_t session_id,
                                        const uint32_t track_id) {
  QMMF_DEBUG("%s:%s: Enter client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  QMMF_KPI_DETAIL();

  uint32_t ret = NO_ERROR;
  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client_id(%d), Not in connected client list!",
        TAG, __func__, client_id);
    return BAD_VALUE;
  }
  if (!IsTrackValid(client_id, session_id, track_id)) {
    QMMF_ERROR("%s:%s: client_id(%d):session_id(%d) track_id(%d) is not valid!",
        TAG, __func__, client_id, session_id, track_id);
    return BAD_VALUE;
  }
  client_session_lock_.lock();
  auto& session_track_map = client_session_map_[client_id];
  auto& tracks_in_session = session_track_map[session_id];
  uint32_t service_track_id = 0;
  TrackInfo track_info {};
  std::vector<TrackTuple>::iterator track_iter = tracks_in_session.begin();
  std::vector<TrackTuple>::iterator track_to_remove = tracks_in_session.end();
  for (; track_iter != tracks_in_session.end(); ++track_iter) {
    if (track_id == std::get<0>(*track_iter)) {
      service_track_id = std::get<1>(*track_iter);
      track_info       = std::get<2>(*track_iter);
      track_to_remove  = track_iter;
      QMMF_INFO("%s:%s: client_id(%d):session_id(%d), "
          "track_id(%d):service_track_id(%x)", TAG, __func__, client_id,
          session_id, track_id, service_track_id);
      break;
    }
  }
  client_session_lock_.unlock();

  assert(track_info.type == TrackType::kAudio);
  assert(audio_source_ != nullptr);
  assert(service_track_id > 0);
  ret = audio_source_->DeleteTrackSource(service_track_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: service_track_id(%x) DeleteTrackSource failed!", TAG,
        __func__, service_track_id);
    return ret;
  }
  if (track_info.audio_params.params.format != AudioFormat::kPCM) {
    assert(audio_encoder_core_ != NULL);

    ret = audio_encoder_core_->DeleteTrackEncoder(service_track_id);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: DeleteTrackEncoder failed for "
          "client_track_id(%d):service_track_id(%x)", TAG, __func__,
           track_id, service_track_id);
      return ret;
    }
  }
  std::lock_guard<std::mutex> lock(client_session_lock_);
  assert(track_to_remove != tracks_in_session.end());
  tracks_in_session.erase(track_to_remove);
  QMMF_INFO("%s:%s: client_track_id(%d):service_track_id(%x) Deleted "
      "Successfully", TAG, __func__, track_id, service_track_id);
  QMMF_INFO("%s:%s: Number of tracks(%d) left in session(%d)", TAG, __func__,
      tracks_in_session.size(), session_id);

  QMMF_DEBUG("%s:%s: Enter client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  return NO_ERROR;
}

status_t RecorderImpl::CreateVideoTrack(const uint32_t client_id,
                                        const uint32_t session_id,
                                        const uint32_t track_id,
                                        const VideoTrackCreateParam& params) {

  QMMF_DEBUG("%s:%s: Enter client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  QMMF_KPI_DETAIL();

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client_id(%d), Not in connected client list!",
        TAG, __func__, client_id);
    return BAD_VALUE;
  }
  if (!IsSessionIdValid(client_id, session_id)) {
    QMMF_ERROR("%s:%s: session_id(%d) is not valid!", TAG, __func__,
        session_id);
    return BAD_VALUE;
  }
  uint32_t service_track_id = GetUniqueServiceTrackId(client_id, session_id,
                                                      track_id);
  QMMF_INFO("%s:%s: client_id(%d):session_id(%d) client_track_id(%d):"
      "service_track_id(%x)", TAG, __func__, client_id, session_id, track_id,
      service_track_id);
  VideoExtraParam empty_extra_params;

  VideoTrackParams video_track_params;
  memset(&video_track_params, 0x0, sizeof video_track_params);
  video_track_params.track_id    = service_track_id;
  video_track_params.params      = params;
  video_track_params.extra_param = empty_extra_params;
  video_track_params.data_cb     = [this, client_id, session_id, track_id]
      (std::vector<BnBuffer>& buffers, std::vector<MetaData>& meta_buffers) {
          VideoTrackBufferCallback(client_id, session_id, track_id,
                                   buffers, meta_buffers);
      };
  // Create Camera track first.
  assert(camera_source_ != nullptr);
  auto ret = camera_source_->CreateTrackSource(service_track_id,
                                               video_track_params);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreateTrackSource track_id(%d):service_track_id(%x) "
        " failed!", TAG, __func__, track_id, service_track_id);
    return BAD_VALUE;
  }
  QMMF_INFO("%s:%s: client_id(%d):session_id(%d), TrackSource for "
      "client_track_id(%d):service_track_id(%x) Added Successfully in "
      "CameraSource!", TAG, __func__, client_id, session_id, track_id,
      service_track_id);

  // If video codec type is set to YUV then no need to create Encoder instance.
  // direct YUV frame will go to client.
  if ( (params.format_type == VideoFormat::kHEVC)
      || (params.format_type == VideoFormat::kAVC) ) {

    // Create Encoder track and add TrackSource as a source to iit.
    // Track pipeline: TrackSource <--> TrackEncoder
    assert(encoder_core_ != nullptr);
    ret = encoder_core_->AddSource(camera_source_->
        GetTrackSource(service_track_id), video_track_params);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: track_id(%d):service_track_id(%x) AddSource"
        " failed!", TAG, __func__, track_id, service_track_id);
      return BAD_VALUE;
    }
  }

  // Assosiate track to session.
  TrackInfo track_info;
  memset(&track_info, 0x0, sizeof track_info);
  track_info.track_id     = service_track_id;
  track_info.type         = TrackType::kVideo;
  track_info.video_params = video_track_params;

  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto& session_track_map = client_session_map_[client_id];
  auto& tracks_in_session = session_track_map[session_id];
  auto track_tuple = std::make_tuple(track_id, service_track_id, track_info);
  tracks_in_session.push_back(track_tuple);

  QMMF_INFO("%s:%s: client_id(%d), session_id(%d), num sessions=%d", TAG,
      __func__, client_id, session_id, session_track_map.size());
  QMMF_INFO("%s:%s: num of tracks=%d", TAG, __func__,
      tracks_in_session.size());
  for (auto test : tracks_in_session) {
    QMMF_INFO("%s:%s: client_track_id(%d):service_track_id(%x):track_type(%d)",
        TAG, __func__, std::get<0>(test), std::get<1>(test),
        std::get<2>(test).type);
  }
  QMMF_DEBUG("%s:%s: Exit client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  return ret;
}

status_t RecorderImpl::CreateVideoTrack(const uint32_t client_id,
                                        const uint32_t session_id,
                                        const uint32_t track_id,
                                        const VideoTrackCreateParam& params,
                                        const VideoExtraParam& extra_param) {

  QMMF_DEBUG("%s:%s: Enter client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client_id(%d), Not in connected client list!",
        TAG, __func__, client_id);
    return BAD_VALUE;
  }
  if (!IsSessionIdValid(client_id, session_id)) {
    QMMF_ERROR("%s:%s: session_id(%d) is not valid!", TAG, __func__,
        session_id);
    return BAD_VALUE;
  }
  uint32_t service_track_id = GetUniqueServiceTrackId(client_id, session_id,
                                                      track_id);
  QMMF_INFO("%s:%s: client_id(%d):session_id(%d) client_track_id(%d):"
      "service_track_id(%x)", TAG, __func__, client_id, session_id, track_id,
      service_track_id);

  VideoTrackParams video_track_params;
  memset(&video_track_params, 0x0, sizeof video_track_params);
  video_track_params.track_id    = service_track_id;
  video_track_params.params      = params;
  video_track_params.extra_param = extra_param;
  video_track_params.data_cb     = [this, client_id, session_id, track_id]
      (std::vector<BnBuffer>& buffers, std::vector<MetaData>& meta_buffers) {
          VideoTrackBufferCallback(client_id, session_id, track_id,
                                   buffers, meta_buffers);
      };
  // Create Camera track first.
  assert(camera_source_ != nullptr);
  auto ret = camera_source_->CreateTrackSource(service_track_id,
                                               video_track_params);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreateTrackSource track_id(%d):service_track_id(%x) "
        " failed!", TAG, __func__, track_id, service_track_id);
    return BAD_VALUE;
  }
  QMMF_INFO("%s:%s: client_id(%d):session_id(%d), TrackSource for "
      "client_track_id(%d):service_track_id(%x) Added Successfully in "
      "CameraSource!", TAG, __func__, client_id, session_id, track_id,
      service_track_id);

  // If video codec type is set to YUV then no need to create Encoder instance.
  // direct YUV frame will go to client.
  if ( (params.format_type == VideoFormat::kHEVC)
      || (params.format_type == VideoFormat::kAVC) ) {

    // Create Encoder track and add TrackSource as a source to iit.
    // Track pipeline: TrackSource <--> TrackEncoder
    assert(encoder_core_ != nullptr);
    ret = encoder_core_->AddSource(camera_source_->
        GetTrackSource(service_track_id), video_track_params);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: track_id(%d):service_track_id(%x) AddSource"
        " failed!", TAG, __func__, track_id, service_track_id);
      return BAD_VALUE;
    }
  }

  // Assosiate track to session.
  TrackInfo track_info;
  memset(&track_info, 0x0, sizeof track_info);
  track_info.track_id     = service_track_id;
  track_info.type         = TrackType::kVideo;
  track_info.video_params = video_track_params;

  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto& session_track_map = client_session_map_[client_id];
  auto& tracks_in_session = session_track_map[session_id];
  auto track_tuple = std::make_tuple(track_id, service_track_id, track_info);
  tracks_in_session.push_back(track_tuple);

  QMMF_INFO("%s:%s: client_id(%d), session_id(%d), num sessions=%d", TAG,
      __func__, client_id, session_id, session_track_map.size());
  QMMF_INFO("%s:%s: num of tracks=%d", TAG, __func__,
      tracks_in_session.size());
  for (auto test : tracks_in_session) {
    QMMF_INFO("%s:%s: client_track_id(%d):service_track_id(%x):track_type(%d)",
        TAG, __func__, std::get<0>(test), std::get<1>(test),
        std::get<2>(test).type);
  }
  QMMF_DEBUG("%s:%s: Exit client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  return NO_ERROR;
}

status_t RecorderImpl::DeleteVideoTrack(const uint32_t client_id,
                                        const uint32_t session_id,
                                        const uint32_t track_id) {
  QMMF_DEBUG("%s:%s: Enter client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  QMMF_KPI_DETAIL();

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Invalid client_id(%d), Not in connected client list!",
        TAG, __func__, client_id);
    return BAD_VALUE;
  }
  if (!IsTrackValid(client_id, session_id, track_id)) {
    QMMF_ERROR("%s:%s: client_id(%d):session_id(%d) track_id(%d) is not valid!",
        TAG, __func__, client_id, session_id, track_id);
    return BAD_VALUE;
  }
  client_session_lock_.lock();
  auto& session_track_map = client_session_map_[client_id];
  auto& tracks_in_session = session_track_map[session_id];
  uint32_t service_track_id = 0;
  TrackInfo track_info {};
  std::vector<TrackTuple>::iterator track_iter = tracks_in_session.begin();
  std::vector<TrackTuple>::iterator track_to_remove = tracks_in_session.end();
  for (; track_iter != tracks_in_session.end(); ++track_iter) {
    if (track_id == std::get<0>(*track_iter)) {
      service_track_id = std::get<1>(*track_iter);
      track_info       = std::get<2>(*track_iter);
      track_to_remove  = track_iter;
      QMMF_INFO("%s:%s: client_id(%d):session_id(%d), "
          "track_id(%d):service_track_id(%x)", TAG, __func__, client_id,
          session_id, track_id, service_track_id);
      break;
    }
  }
  client_session_lock_.unlock();

  assert(track_info.type == TrackType::kVideo);
  assert(camera_source_ != nullptr);
  assert(service_track_id > 0);
  auto ret = camera_source_->DeleteTrackSource(service_track_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: service_track_id(%x) DeleteTrackSource failed!", TAG,
        __func__, service_track_id);
    return ret;
  }

  VideoFormat fmt_type = track_info.video_params.params.format_type;
  if ((fmt_type == VideoFormat::kHEVC) || (fmt_type == VideoFormat::kAVC)) {

    assert(encoder_core_ != nullptr);
    ret = encoder_core_->DeleteTrackEncoder(service_track_id);
    if (ret != NO_ERROR) {
      ret = BAD_VALUE;
      QMMF_ERROR("%s:%s: DeleteTrackEncoder failed for "
          "client_track_id(%d):service_track_id(%x)", TAG, __func__,
           track_id, service_track_id);
      return ret;
    }
  }
  {
    std::lock_guard<std::mutex> lock(client_session_lock_);
    assert(track_to_remove != tracks_in_session.end());
    tracks_in_session.erase(track_to_remove);
  }

  QMMF_INFO("%s:%s: client_track_id(%d):service_track_id(%x) Deleted "
      "Successfully", TAG, __func__, track_id, service_track_id);
  QMMF_INFO("%s:%s: Number of tracks(%d) left in session(%d)", TAG, __func__,
      tracks_in_session.size(), session_id);

  // This method doesn't go up to client as a callback, it is just to update
  // Internal data structure used for buffer mapping.
  remote_cb_handle_(client_id)->NotifyDeleteVideoTrack(track_id);

  QMMF_DEBUG("%s:%s: Enter client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  return ret;
}

status_t RecorderImpl::ReturnTrackBuffer(const uint32_t client_id,
                                         const uint32_t session_id,
                                         const uint32_t track_id,
                                         std::vector<BnBuffer> &buffers) {

  QMMF_VERBOSE("%s:%s: Enter client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  for (const BnBuffer& buffer : buffers)
    QMMF_VERBOSE("%s:%s INPARAM: buffers[%s]", TAG, __func__,
                 buffer.ToString().c_str());

  uint32_t ret = NO_ERROR;
  if (!IsTrackValid(client_id, session_id, track_id)) {
    QMMF_ERROR("%s:%s: client_id(%d):session_id(%d),track_id(%d) is not valid!",
        TAG, __func__, client_id, session_id, track_id);
    return BAD_VALUE;
  }
  TrackInfo track_info {};
  GetServiceTrackInfo(client_id, session_id, track_id, &track_info);
  assert(track_info.track_id > 0);

  if (track_info.type == TrackType::kVideo) {

    VideoFormat fmt_type = track_info.video_params.params.format_type;
    if ( (fmt_type == VideoFormat::kAVC)
         || (fmt_type == VideoFormat::kHEVC) ) {
      assert(encoder_core_ != nullptr);
      ret = encoder_core_->ReturnTrackBuffer(track_info.track_id, buffers);
      assert(ret == NO_ERROR);

    } else {
      // These are the RAW frames, return them back to camera source directly.
      assert(camera_source_ != nullptr);
      ret = camera_source_->ReturnTrackBuffer(track_info.track_id, buffers);
      assert(ret == NO_ERROR);
    }

  } else {
    if (track_info.audio_params.params.format != AudioFormat::kPCM) {
      assert(audio_encoder_core_ != nullptr);
      ret = audio_encoder_core_->ReturnTrackBuffer(track_info.track_id,
                                                   buffers);
      assert(ret == NO_ERROR);
    } else {
      assert(audio_source_ != nullptr);
      ret = audio_source_->ReturnTrackBuffer(track_info.track_id, buffers);
      assert(ret == NO_ERROR);
    }
  }
  QMMF_VERBOSE("%s:%s: Exit client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  return ret;
}

status_t RecorderImpl::SetAudioTrackParam(const uint32_t client_id,
                                          const uint32_t session_id,
                                          const uint32_t track_id,
                                          CodecParamType type,
                                          void *param,
                                          size_t param_size) {
  return NO_ERROR;
}

status_t RecorderImpl::SetVideoTrackParam(const uint32_t client_id,
                                          const uint32_t session_id,
                                          const uint32_t track_id,
                                          CodecParamType type,
                                          void *param,
                                          size_t param_size) {
  QMMF_DEBUG("%s:%s: Enter client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);

  if (!IsTrackValid(client_id, session_id, track_id)) {
    QMMF_ERROR("%s:%s: client_id(%d):session_id(%d),track_id(%d) is not valid!",
        TAG, __func__, client_id, session_id, track_id);
    return BAD_VALUE;
  }
  TrackInfo track_info {};
  GetServiceTrackInfo(client_id, session_id, track_id, &track_info);
  assert(track_info.track_id > 0);

  assert(encoder_core_ != nullptr);
  auto ret = encoder_core_->SetTrackEncoderParams(track_info.track_id, type,
                                                  param, param_size);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: client_id(%d) Failed to set video encode params!", TAG,
        __func__, client_id);
    return ret;
  }
  if (ret == NO_ERROR && type == CodecParamType::kFrameRateType) {
    float* fps = static_cast<float*>(param);
    ret = camera_source_->UpdateTrackFrameRate(track_info.track_id, *fps);
    if(ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: client_id(%d) Failed to set FrameRate to TrackSource",
          TAG, __func__, client_id);
      return ret;
    }
  }

  if (type == CodecParamType::kEnableFrameRepeat) {
    bool* enable_frame_repeat = static_cast<bool*>(param);
    ret = camera_source_->EnableFrameRepeat(track_info.track_id,
                                            *enable_frame_repeat);
    if(ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: client_id(%d) Failed to set "
          "FrameRepeat to TrackSource", TAG, __func__, client_id);
      return ret;
    }
  }
  QMMF_DEBUG("%s:%s: Exit client_id(%d):session_id(%d)", TAG, __func__,
      client_id, session_id);
  return ret;
}

status_t RecorderImpl::CaptureImage(const uint32_t client_id,
                                    const uint32_t camera_id,
                                    const ImageParam &param,
                                    const uint32_t num_images,
                                    const std::vector<CameraMetadata> &meta) {

  QMMF_DEBUG("%s:%s: Enter client_id(%d):camera_id(%d)", TAG, __func__,
      client_id, camera_id);

  assert(camera_source_ != nullptr);
  SnapshotCb cb = [ this, client_id ] (uint32_t camera_id,
      uint32_t count, BnBuffer& buf, MetaData& meta_data) {
          SnapshotCallback(client_id, camera_id, count, buf, meta_data);
      };
  auto ret = camera_source_->CaptureImage(camera_id, param, num_images,
                                          meta, cb);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: client_id(%d):camera_id(%d) CaptureImage failed!", TAG,
        __func__, client_id, camera_id);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit client_id(%d):camera_id(%d)", TAG, __func__,
      client_id, camera_id);;
  return ret;
}

status_t RecorderImpl::ConfigImageCapture(const uint32_t client_id,
                                          const uint32_t camera_id,
                                          const ImageConfigParam &config) {

  QMMF_DEBUG("%s:%s: Enter client_id(%d):camera_id(%d)", TAG, __func__,
      client_id, camera_id);

  assert(camera_source_ != nullptr);
  auto ret = camera_source_->ConfigImageCapture(camera_id, config);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: client_id(%d):camera_id(%d) ConfigImageCapture failed!",
        TAG, __func__, client_id, camera_id);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit client_id(%d):camera_id(%d)", TAG, __func__,
      client_id, camera_id);;
  return ret;
}


status_t RecorderImpl::CancelCaptureImage(const uint32_t client_id,
                                          const uint32_t camera_id) {

  QMMF_DEBUG("%s:%s: Enter client_id(%d):camera_id(%d)", TAG, __func__,
      client_id, camera_id);
  assert(camera_source_ != nullptr);
  auto ret = camera_source_->CancelCaptureImage(camera_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CancelCaptureImage failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit client_id(%d):camera_id(%d)", TAG, __func__,
      client_id, camera_id);
  return NO_ERROR;
}

status_t RecorderImpl::ReturnImageCaptureBuffer(const uint32_t client_id,
                                                const uint32_t camera_id,
                                                const int32_t buffer_id) {

  QMMF_DEBUG("%s:%s: Enter client_id(%d):camera_id(%d)", TAG, __func__,
      client_id, camera_id);
  assert(camera_source_ != nullptr);
  auto ret = camera_source_->ReturnImageCaptureBuffer(camera_id, buffer_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: ReturnImageCaptureBuffer failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit client_id(%d):camera_id(%d)", TAG, __func__,
      client_id, camera_id);
  return ret;
}


status_t RecorderImpl::SetCameraParam(const uint32_t client_id,
                                      const uint32_t camera_id,
                                      const CameraMetadata &meta) {
  QMMF_DEBUG("%s:%s: Enter client_id(%d):camera_id(%d)", TAG, __func__,
      client_id, camera_id);
  if (!IsCameraOwned(client_id, camera_id)) {
    QMMF_ERROR("%s:%s client_id(%d) Not allowed! Camera (%d) is already "
        "owned by other client", TAG, __func__, client_id, camera_id);
    return INVALID_OPERATION;
  }
  assert(camera_source_ != nullptr);
  auto ret = camera_source_->SetCameraParam(camera_id, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: SetCameraParam failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Enter client_id(%d):camera_id(%d)", TAG, __func__,
      client_id, camera_id);
  return ret;
}

status_t RecorderImpl::GetCameraParam(const uint32_t client_id,
                                      const uint32_t camera_id,
                                      CameraMetadata &meta) {
  QMMF_DEBUG("%s:%s: Enter client_id(%d):camera_id(%d)", TAG, __func__,
      client_id, camera_id);
  if (!IsCameraOwned(client_id, camera_id)) {
    QMMF_ERROR("%s:%s client_id(%d) Not allowed! Camera (%d) is already "
        "owned by other client", TAG, __func__, client_id, camera_id);
    return INVALID_OPERATION;
  }
  assert(camera_source_ != nullptr);
  auto ret = camera_source_->GetCameraParam(camera_id, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: GetCameraParam failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit client_id(%d):camera_id(%d)", TAG, __func__,
      client_id, camera_id);
  return ret;
}

status_t RecorderImpl::GetDefaultCaptureParam(const uint32_t client_id,
                                              const uint32_t camera_id,
                                              CameraMetadata &meta) {
  QMMF_DEBUG("%s:%s: Enter client_id(%d):camera_id(%d)", TAG, __func__,
      client_id, camera_id);
  if (!IsCameraOwned(client_id, camera_id)) {
    QMMF_ERROR("%s:%s client_id(%d) Not allowed! Camera (%d) is already "
        "owned by other client", TAG, __func__, client_id, camera_id);
    return INVALID_OPERATION;
  }
  assert(camera_source_ != nullptr);
  auto ret = camera_source_->GetDefaultCaptureParam(camera_id, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: GetDefaultCaptureParam failed!", TAG, __func__);
    return ret;
  }
  QMMF_DEBUG("%s:%s: Exit client_id(%d):camera_id(%d)", TAG, __func__,
      client_id, camera_id);
  return ret;
}

status_t RecorderImpl::CreateOverlayObject(const uint32_t client_id,
                                           const uint32_t track_id,
                                           OverlayParam *param,
                                           uint32_t *overlay_id) {
  QMMF_VERBOSE("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;

  if (!IsTrackValid(client_id, track_id)) {
    QMMF_ERROR("%s:%s: client_id(%d):track_id(%d) is not valid!", TAG, __func__,
        client_id, track_id);
    return BAD_VALUE;
  }
  TrackInfo track_info {};
  GetServiceTrackInfo(client_id, track_id, &track_info);
  assert(track_info.track_id > 0);

  assert(camera_source_ != NULL);
  ret = camera_source_->CreateOverlayObject(track_info.track_id,
                                            param, overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreateOverlayObject failed!", TAG, __func__);
    return ret;
  }
  QMMF_VERBOSE("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t RecorderImpl::DeleteOverlayObject(const uint32_t client_id,
                                           const uint32_t track_id,
                                           const uint32_t overlay_id) {
  QMMF_VERBOSE("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;

  if (!IsTrackValid(client_id, track_id)) {
    QMMF_ERROR("%s:%s: client_id(%d):track_id(%d) is not valid!", TAG, __func__,
        client_id, track_id);
    return BAD_VALUE;
  }
  TrackInfo track_info {};
  GetServiceTrackInfo(client_id, track_id, &track_info);
  assert(track_info.track_id > 0);

  assert(camera_source_ != NULL);
  ret = camera_source_->DeleteOverlayObject(track_info.track_id, overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: DeleteOverlayObject failed!", TAG, __func__);
    return ret;
  }
  QMMF_VERBOSE("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t RecorderImpl::GetOverlayObjectParams(const uint32_t client_id,
                                              const uint32_t track_id,
                                              const uint32_t overlay_id,
                                              OverlayParam &param) {
  QMMF_VERBOSE("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;

  if (!IsTrackValid(client_id, track_id)) {
    QMMF_ERROR("%s:%s: client_id(%d):track_id(%d) is not valid!",
        TAG, __func__, client_id, track_id);
    return BAD_VALUE;
  }
  TrackInfo track_info {};
  GetServiceTrackInfo(client_id, track_id, &track_info);
  assert(track_info.track_id > 0);

  assert(camera_source_ != NULL);
  ret = camera_source_->GetOverlayObjectParams(track_info.track_id, overlay_id,
                                               param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: GetOverlayObjectParams failed!", TAG, __func__);
    return ret;
  }
  QMMF_VERBOSE("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t RecorderImpl::UpdateOverlayObjectParams(const uint32_t client_id,
                                                 const uint32_t track_id,
                                                 const uint32_t overlay_id,
                                                 OverlayParam *param) {
  QMMF_VERBOSE("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;

  if (!IsTrackValid(client_id, track_id)) {
    QMMF_ERROR("%s:%s: client_id(%d):track_id(%d) is not valid!",
        TAG, __func__, client_id, track_id);
    return BAD_VALUE;
  }
  TrackInfo track_info {};
  GetServiceTrackInfo(client_id, track_id, &track_info);
  assert(track_info.track_id > 0);

  assert(camera_source_ != NULL);
  ret = camera_source_->UpdateOverlayObjectParams(track_info.track_id,
                                                  overlay_id, param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: UpdateOverlayObjectParams failed!", TAG, __func__);
    return ret;
  }
  QMMF_VERBOSE("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t RecorderImpl::SetOverlayObject(const uint32_t client_id,
                                        const uint32_t track_id,
                                        const uint32_t overlay_id) {
  QMMF_VERBOSE("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;

  if (!IsTrackValid(client_id, track_id)) {
    QMMF_ERROR("%s:%s: client_id(%d):track_id(%d) is not valid!",
        TAG, __func__, client_id, track_id);
    return BAD_VALUE;
  }
  TrackInfo track_info {};
  GetServiceTrackInfo(client_id, track_id, &track_info);
  assert(track_info.track_id > 0);

  assert(camera_source_ != NULL);
  ret = camera_source_->SetOverlayObject(track_info.track_id, overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: SetOverlayObject failed!", TAG, __func__);
    return ret;
  }
  QMMF_VERBOSE("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t RecorderImpl::RemoveOverlayObject(const uint32_t client_id,
                                           const uint32_t track_id,
                                           const uint32_t overlay_id) {
  QMMF_VERBOSE("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;

  if (!IsTrackValid(client_id, track_id)) {
    QMMF_ERROR("%s:%s: client_id(%d):track_id(%d) is not valid!",
        TAG, __func__, client_id, track_id);
    return BAD_VALUE;
  }
  TrackInfo track_info {};
  GetServiceTrackInfo(client_id, track_id, &track_info);
  assert(track_info.track_id > 0);

  assert(camera_source_ != NULL);
  ret = camera_source_->RemoveOverlayObject(track_info.track_id, overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: RemoveOverlayObject failed!", TAG, __func__);
    return ret;
  }
  QMMF_VERBOSE("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t RecorderImpl::CreateMultiCamera(const uint32_t client_id,
                                         const std::vector<uint32_t> camera_ids,
                                         uint32_t *virtual_camera_id) {

  QMMF_VERBOSE("%s:%s: Enter", TAG, __func__);
  assert(camera_source_ != NULL);
  auto ret = camera_source_->CreateMultiCamera(camera_ids, virtual_camera_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreateMultiCamera failed!", TAG, __func__);
    return ret;
  }
  QMMF_VERBOSE("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t RecorderImpl::ConfigureMultiCamera(const uint32_t client_id,
                                            const uint32_t virtual_camera_id,
                                            const MultiCameraConfigType type,
                                            const void *param,
                                            const uint32_t param_size) {

  QMMF_VERBOSE("%s:%s: Enter", TAG, __func__);
  assert(camera_source_ != NULL);
  auto ret = camera_source_->ConfigureMultiCamera(virtual_camera_id, type,
                                                  param, param_size);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: ConfigureMultiCamera failed!", TAG, __func__);
    return ret;
  }
  QMMF_VERBOSE("%s:%s: Exit", TAG, __func__);
  return ret;
}

// Data callback handlers.
void RecorderImpl::VideoTrackBufferCallback(uint32_t remote_client_id,
                                            uint32_t session_id,
                                            uint32_t client_track_id,
                                            std::vector<BnBuffer>& buffers,
                                            std::vector<MetaData>&
                                            meta_buffers) {

  assert(remote_cb_handle_ != nullptr);
  assert(remote_client_id > 0);

  std::lock_guard<std::mutex> lock(client_died_lock_);
  if (client_died_) {
    ReturnTrackBuffer(remote_client_id, session_id, client_track_id, buffers);
  } else {
    remote_cb_handle_(remote_client_id)->NotifyVideoTrackData(client_track_id,
        buffers, meta_buffers);
  }
}

void RecorderImpl::AudioTrackBufferCallback(uint32_t remote_client_id,
                                            uint32_t session_id,
                                            uint32_t client_track_id,
                                            std::vector<BnBuffer>& buffers,
                                            std::vector<MetaData>&
                                            meta_buffers) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  for (const BnBuffer& buffer : buffers)
    QMMF_VERBOSE("%s:%s INPARAM: buffer[%s]", TAG, __func__,
                 buffer.ToString().c_str());
  assert(remote_cb_handle_ != nullptr);
  assert(remote_client_id > 0);

  std::lock_guard<std::mutex> lock(client_died_lock_);
  if (client_died_) {
    ReturnTrackBuffer(remote_client_id, session_id, client_track_id, buffers);
  } else {
    remote_cb_handle_(remote_client_id)->NotifyAudioTrackData(client_track_id,
        buffers, meta_buffers);
  }
}

void RecorderImpl::SnapshotCallback(uint32_t remote_client_id,
                                    uint32_t camera_id, uint32_t count,
                                    BnBuffer& buffer, MetaData& meta_data) {

  assert(remote_cb_handle_ != nullptr);
  assert(remote_client_id > 0);
  remote_cb_handle_(remote_client_id)->NotifySnapshotData(camera_id, count,
                                                          buffer, meta_data);
}

void RecorderImpl::CameraResultCallback(uint32_t remote_client_id,
                                        uint32_t camera_id,
                                        const CameraMetadata &result) {
  assert(remote_cb_handle_ != nullptr);
  assert(remote_client_id > 0);
  remote_cb_handle_(remote_client_id)->NotifyCameraResult(camera_id, result);
}

void RecorderImpl::CameraErrorCallback(uint32_t remote_client_id,
                                       RecorderErrorData &error) {
  assert(remote_cb_handle_ != nullptr);
  assert(remote_client_id > 0);
  remote_cb_handle_(remote_client_id)->NotifyRecorderEvent(
      EventType::kCameraError, reinterpret_cast<void*>(&error),
      sizeof(RecorderErrorData));
}

bool RecorderImpl::IsClientValid(const uint32_t client_id) {
  std::lock_guard<std::mutex> lock(client_session_lock_);
  bool valid = false;
  auto iter = client_session_map_.find(client_id);
  if (iter != client_session_map_.end()) {
    valid = true;
  }
  return valid;
}

bool RecorderImpl::IsSessionIdValid(const uint32_t client_id,
                                    const uint32_t session_id) {
  std::lock_guard<std::mutex> lock(client_session_lock_);
  bool valid = false;
  auto session_track_map = client_session_map_[client_id];
  auto iter = session_track_map.find(session_id);
  if (iter != session_track_map.end()) {
    valid = true;
  }
  return valid;
}

bool RecorderImpl::IsSessionValid(const uint32_t client_id,
                                  const uint32_t session_id) {

  // There could be a case where application can try to start session without
  // Adding valid track in session.
  std::lock_guard<std::mutex> lock(client_session_lock_);
  bool valid = false;
  auto session_track_map = client_session_map_[client_id];
  auto iter = session_track_map.find(session_id);
  if (iter != session_track_map.end()) {
    auto tracks = session_track_map[session_id];
    if (tracks.size() > 0) {
      valid = true;
    }
  }
  return valid;
}

bool RecorderImpl::IsSessionStarted(const uint32_t session_id) {
  std::lock_guard<std::mutex> lock(client_session_lock_);
  bool started = false;
  auto iter = sessions_state_.find(session_id);
  if (iter != sessions_state_.end()) {
    started = sessions_state_[session_id];
  }
  return started;
}

bool RecorderImpl::IsTrackValid(const uint32_t client_id,
                                const uint32_t session_id,
                                const uint32_t client_track_id) {
  std::lock_guard<std::mutex> lock(client_session_lock_);
  bool valid = false;
  auto session_track_map = client_session_map_[client_id];
  auto iter = session_track_map.find(session_id);
  if (iter != session_track_map.end()) {
    auto tracks = session_track_map[session_id];
    for (auto track : tracks) {
      if (client_track_id == std::get<0>(track)) {
        valid = true;
        break;
      }
    }
  }
  return valid;
}

bool RecorderImpl::IsTrackValid(const uint32_t client_id,
                                const uint32_t client_track_id) {
  std::lock_guard<std::mutex> lock(client_session_lock_);
  bool valid = false;
  auto sessions = client_session_map_[client_id];
  for (auto session : sessions) {
    auto tracks_in_session = session.second;
    for (auto track : tracks_in_session) {
      if (client_track_id == std::get<0>(track)) {
        valid = true;
        return valid;
      }
    }
  }
  return valid;
}

bool RecorderImpl::IsCameraOwned(const uint32_t client_id,
                                 const uint32_t camera_id) {
  std::lock_guard<std::mutex> lock(camera_map_lock_);
  bool valid = false;
  for (auto iter : client_cameraid_map_) {
    std::vector<uint32_t> camera_ids = iter.second;
    for (auto idx : camera_ids) {
      if (camera_id == idx) {
        if (iter.first == client_id) {
          valid = true;
          return valid;
        }
        break;
      }
    }
  }
  return false;
}

uint32_t RecorderImpl::GetUniqueServiceTrackId(const uint32_t client_id,
                                               const uint32_t session_id,
                                               const uint32_t track_id) {
  uint32_t service_track_id = client_id << 24;
  service_track_id |= session_id << 16;
  service_track_id |= track_id;
  return service_track_id;
}

status_t RecorderImpl::GetServiceTrackInfo(const uint32_t client_id,
                                           const uint32_t session_id,
                                           const uint32_t client_track_id,
                                           TrackInfo* track_info) {
  status_t ret = BAD_VALUE;
  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto session_track_map = client_session_map_[client_id];
  auto tracks_in_session = session_track_map[session_id];
  for (auto track : tracks_in_session) {
    if (client_track_id == std::get<0>(track)) {
      *track_info      = std::get<2>(track);
      QMMF_VERBOSE("%s:%s: client_id(%d):session_id(%d), "
          "track_id(%d):service_track_id(%x)", TAG, __func__, client_id,
          session_id, client_track_id, track_info->track_id);
      ret = NO_ERROR;
      return ret;
    }
  }
  return ret;
}

status_t RecorderImpl::GetServiceTrackInfo(const uint32_t client_id,
                                           const uint32_t client_track_id,
                                           TrackInfo* track_info) {
  status_t ret = BAD_VALUE;
  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto sessions = client_session_map_[client_id];
  for (auto session : sessions) {
    auto tracks_in_session = session.second;
    for (auto track : tracks_in_session) {
      if (client_track_id == std::get<0>(track)) {
        *track_info = std::get<2>(track);
        QMMF_VERBOSE("%s:%s: client_id(%d):track_id(%d):service_track_id(%x)",
            TAG, __func__, client_id, client_track_id, track_info->track_id);
        ret = NO_ERROR;
        return ret;
      }
    }
  }
  return ret;
}

}; // namespace recorder

}; //namespace qmmf
