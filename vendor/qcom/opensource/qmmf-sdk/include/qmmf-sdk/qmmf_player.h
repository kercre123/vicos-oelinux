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

#pragma once

#include <cstddef>
#include <cstdlib>
#include <vector>
#include <string>

#include "qmmf-sdk/qmmf_player_params.h"

namespace qmmf {
namespace player {

class PlayerClient;

class Player
{
 public:
  Player();
  ~Player();

  status_t Connect(PlayerCb& cb);

  status_t Disconnect();


  status_t CreateAudioTrack(uint32_t track_id,
                            AudioTrackCreateParam& param,
                            TrackCb& cb);

  status_t CreateVideoTrack(uint32_t track_id,
                            VideoTrackCreateParam& param,
                            TrackCb& cb);

  status_t DeleteAudioTrack(uint32_t track_id);
  status_t DeleteVideoTrack(uint32_t track_id);

  // Prepares the player for playback which involves creating instances of
  // Audio and video decoder, connects the data source with the data sinks
  // and starts the data source to queue the media data for playback
  // Prepare must be called after creating all tracks
  status_t Prepare();

  status_t DequeueInputBuffer(uint32_t track_id,
                              std::vector<TrackBuffer>& buffers);

  status_t QueueInputBuffer(uint32_t track_id,
                            std::vector<TrackBuffer>& buffers,
                            void *meta_param,
                            size_t meta_size,
                            TrackMetaBufferType meta_type);

  // Starts track playback
  status_t Start();

  // Stops track playback. Optionally grabs the last video frame rendered to
  // display and returns the buffer
  status_t Stop(const PictureCallback& handler = {nullptr, nullptr},
                const PictureParam& params = {false, VideoCodecType::kYUV,
                                              0, 0, 0});

  // Pauses track playback. Optionally grabs the last video frame rendered to
  // display and returns the buffer
  status_t Pause(const PictureCallback& handler = {nullptr, nullptr},
                 const PictureParam& params = {false, VideoCodecType::kYUV,
                                               0, 0, 0});

  // Resumes the currently paused playback
  status_t Resume();

  // seek to a specified time position, the seek time is in microseconds
  // from the start to seek to
  status_t SetPosition(int64_t seek_time);

  // set playback rate and direction of playback.
  status_t SetTrickMode(TrickModeSpeed speed, TrickModeDirection dir);

  status_t SetAudioTrackParam(uint32_t track_id,
                              CodecParamType type,
                              void *param,
                              size_t param_size);

  status_t SetVideoTrackParam(uint32_t track_id,
                              CodecParamType type,
                              void *param,
                              size_t param_size);

 private:
  PlayerClient* player_client_;
};

}; // namespace player
}; // namespace qmmf
