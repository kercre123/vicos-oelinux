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

#include <map>
#include <string>
#include <vector>
#include <fcntl.h>
#include <dirent.h>
#include <functional>
#include <gtest/gtest.h>
#include <pthread.h>
#include <mutex>

#include <cutils/properties.h>
#include <qmmf-sdk/qmmf_player.h>
#include <qmmf-sdk/qmmf_player_params.h>
#include "player/test/gtest/qmmf_player_parser.h"

#define DEFAULT_ITERATION 1

using namespace qmmf;
using namespace player;
using namespace android;

enum class AudioFileType {
  kAAC,
  kAMR,
  kG711
};

enum class PlayerState {
  kError = 0,
  kIdle = 1 << 0,
  kPrepared = 1 << 1,
  kStarted = 1 << 2,
  kPaused = 1 << 3,
  kStopped =  1 << 4,
  kCompleted = 1<< 5,
};

struct Event {
  PlayerState state;
};

class PlayerGtest : public ::testing::Test {
 public:
  PlayerGtest() : player_() {};

  ~PlayerGtest() {};

 protected:
  const ::testing::TestInfo* test_info_;

  void GetGTestParams();

  void SetUp() override;

  void TearDown() override;

  int32_t Init();

  int32_t DeInit();

  void playercb(EventType event_type, void *event_data,
                size_t event_data_size);

  void audiotrackcb(EventType event_type, void *event_data,
                    size_t event_data_size);

  void videotrackcb(EventType event_type, void *event_data,
                    size_t event_data_size);

  int32_t ParseFile(AudioTrackCreateParam& audio_track_param_);

  int32_t Prepare();

  int32_t Start();

  static void* StartPlaying(void* ptr);

  int32_t Stop();

  int32_t StopPlaying();

  int32_t Pause();

  int32_t Resume();

  int32_t Delete();

  char *            filename_;
  AudioFileType     filetype_;

  Player            player_;
  uint32_t          iteration_count_;
  PlayerCb          player_cb_;

  std::mutex        state_change_lock_;

  bool              stopped_;
  bool              start_again_;
  bool              paused_;
  pthread_t         start_thread_id;

  AACfileIO*        aac_file_io_;
  G711fileIO*       g711_file_io_;
  AMRfileIO*        amr_file_io_;

  std::map<uint32_t, const char *>  statemap_;
  const char*                       player_test_event_[2];
  const char *                      current_state_;
  std::vector<std::string>          codec_type_;
};
