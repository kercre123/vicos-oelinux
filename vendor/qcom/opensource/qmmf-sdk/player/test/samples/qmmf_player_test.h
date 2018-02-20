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

#include <cutils/properties.h>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include <qmmf-sdk/qmmf_buffer.h>
#include <qmmf-sdk/qmmf_display.h>
#include <qmmf-sdk/qmmf_display_params.h>
#include <qmmf-sdk/qmmf_player.h>
#include <qmmf-sdk/qmmf_player_params.h>

#include "player/test/demuxer/qmmf_demuxer_intf.h"
#include "player/test/demuxer/qmmf_demuxer_mediadata_def.h"
#include "player/test/demuxer/qmmf_demuxer_sourceport.h"

using namespace qmmf;
using namespace player;
using namespace android;

enum class AudioFileType{
  kPCM,
  kAAC,
  kAMR,
  kG711,
  kMP3,
};

enum class TrackTypes{
  kAudioVideo,
  kAudioOnly,
  kVideoOnly,
  kInvalid,
};

class PlayerTest {
 public:
  PlayerTest(char* filename);
  ~PlayerTest();

  void Connect();
  void Disconnect();

  void Prepare(bool with_pts);
  void Delete();
  void Start();
  void Stop(bool with_grab);
  void Pause(bool with_grab);
  void Resume();

  void SetPosition();
  void SetTrickMode();

  void PlayerHandler(EventType event_type,
                     void* event_data,
                     size_t event_data_size);

  void AudioTrackHandler(uint32_t track_id,
                         EventType event_type,
                         void* event_data,
                         size_t event_data_size);

  void VideoTrackHandler(uint32_t track_id,
                         EventType event_type,
                         void* event_data,
                         size_t event_data_size);

  void GrabPictureDataCB(uint32_t track_id, BufferDescriptor& buffer);

  bool IsStopPlaying();
  int32_t StopPlayback();

  uint32_t CreateDataSource();
  uint32_t ReadMediaInfo();
  uint32_t ReadAudioTrackMediaInfo(uint32 ulTkId,
                                   FileSourceMnMediaType eMnType);
  uint32_t ReadVideoTrackMediaInfo(uint32 ulTkId,
                                   FileSourceMnMediaType eMnType);

  TrackTypes config_track_type_;

 private:
  enum class State {
    kStopped,
    kRunning,
    kPaused,
  };

  int32_t ParseFile(AudioTrackCreateParam& audio_track_param,
                    VideoTrackCreateParam& video_track_param);

  uint32_t UpdateCurrentPlaybackTime(uint64_t current_time);
  uint32_t GetCurrentPlaybackTime();

  bool IsTrickModeEnabled();

  static void AudioThreadEntry(PlayerTest* player_test);
  void AudioThread();
  static void VideoThreadEntry(PlayerTest* player_test);
  void VideoThread();

#ifndef DISABLE_DISPLAY
  void DisplayCallbackHandler(display::DisplayEventType event_type,
                              void *event_data, size_t event_data_size);
  void DisplayVSyncHandler(int64_t time_stamp);
  status_t StartDisplay(display::DisplayType display_type);
  status_t StopDisplay(display::DisplayType display_type);
  int32_t DequeueSurfaceBuffer();
  int32_t QueueSurfaceBuffer();
  static void DisplayThreadEntry(PlayerTest* player_test);
  void DisplayThread();
#endif

  Player player_;

  std::mutex                      lock_;
  State                           audio_state_;
  State                           video_state_;
  bool                            start_again_;
  ::std::thread*                  audio_thread_;
  ::std::thread*                  video_thread_;

  char*                           filename_;
  MM_TRACK_INFOTYPE               m_sTrackInfo_;
  CMM_MediaSourcePort*            m_pIStreamPort_;
  CMM_MediaDemuxInt*              m_pDemux_;
  int                             fileCount_audio_;
  int                             fileCount_video_;
  std::ofstream                   srcFile_audio_;
  std::ofstream                   srcFile_video_;

  uint32_t                        audio_track_id_;
  uint32_t                        video_track_id_;
  bool                            audioFirstFrame_;
  bool                            videoFirstFrame_;
  bool                            audioLastFrame_;
  bool                            videoLastFrame_;

  TrackTypes                      track_type_;
  TrickModeSpeed                  playback_speed_;
  TrickModeDirection              playback_dir_;
  int32_t                         grabpicture_file_fd_;
  std::mutex                      time_lock_;
  bool                            trick_mode_enabled_;
  uint64_t                        current_playback_time_;
  bool                            enable_gfx_;
  bool                            display_started_;
  uint32_t                        gfx_plane_update_rate_;
  uint32_t                        gfx_plane_frame_count_;
  bool                            push_gfx_content_to_display_;

#ifndef DISABLE_DISPLAY
  std::thread*                    display_thread_;
  display::Display*               display_;
  uint32_t                        surface_id_;
  display::SurfaceParam           surface_param_;
  display::SurfaceBuffer          surface_buffer_;
  std::ifstream                   gfx_frame_;
#endif

  PlayerTest();
};

class CmdMenu {
 public:
  enum CommandType {
      CONNECT_CMD                       = '1',
      DISCONNECT_CMD                    = '2',
      PREPARE_CMD                       = '3',
      PREPARE_PTS_CMD                   = '4',
      START_CMD                         = '5',
      STOP_CMD                          = '6',
      STOP_WITH_GRAB_CMD                = 'a',
      PAUSE_CMD                         = '7',
      PAUSE_WITH_GRAB_CMD               = 'b',
      RESUME_CMD                        = '8',
      DELETE_CMD                        = '9',
      TRICK_MODE_CMD                    = 'T',
      SEEK_CMD                          = 'S',
      EXIT_CMD                          = 'X',
      NEXT_CMD                          = '\n',
      INVALID_CMD                       = '0'
  };

  struct Command {
      Command( CommandType cmd)
      : cmd(cmd) {}
      Command()
      : cmd(INVALID_CMD) {}
      CommandType cmd;
  };

  CmdMenu(PlayerTest &ctx) :  ctx_(ctx) {};

  ~CmdMenu() {};

  Command GetCommand(bool& is_print_menu);

  void PrintMenu();

  void HelpMenu(const char* test_name);

  PlayerTest &ctx_;
};
