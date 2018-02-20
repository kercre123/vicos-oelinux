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

#include <map>
#include <mutex>
#include <string>
#include <thread>

#include <qmmf-sdk/qmmf_player.h>
#include <qmmf-sdk/qmmf_player_params.h>

#include "player/test/samples/qmmf_player_parser.h"

using namespace qmmf;
using namespace player;
using namespace android;

enum class AudioFileType {
  kPCM,
  kAAC,
  kAMR,
  kG711,
  kMP3,
};

class PlayerTest {
 public:
  PlayerTest();
  ~PlayerTest();

  void Connect();
  void Disconnect();

  void Prepare();
  void Delete();
  void Start();
  void Stop();
  void Pause();
  void Resume();

  void SetPosition();
  void SetTrickMode();

  void GrabPicture();

  void AdjustVolume(const int32_t adjustment);

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

  char*             filename_;
  AudioFileType     filetype_;

 private:
  enum class State {
    kStopped,
    kRunning,
    kPaused,
  };

  int32_t ParseFile(AudioTrackCreateParam& audio_track_param_);

  static void ThreadEntry(PlayerTest* player_test);
  void Thread();

  Player player_;

  ::std::thread*    thread_;
  ::std::mutex      lock_;
  State             state_;
  bool              start_again_;
  int32_t           volume_;

  PCMfileIO*        pcm_file_io_;
  AACfileIO*        aac_file_io_;
  G711fileIO*       g711_file_io_;
  AMRfileIO*        amr_file_io_;
  MP3fileIO*        mp3_file_io_;
};

class CmdMenu {
 public:
  enum CommandType {
      CONNECT_CMD                       = '1',
      DISCONNECT_CMD                    = '2',
      PREPARE_CMD                       = '3',
      START_CMD                         = '4',
      STOP_CMD                          = '5',
      PAUSE_CMD                         = '6',
      RESUME_CMD                        = '7',
      DELETE_CMD                        = '8',
      VOLUME_UP_CMD                     = 'U',
      VOLUME_DOWN_CMD                   = 'D',
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

  PlayerTest &ctx_;
};
