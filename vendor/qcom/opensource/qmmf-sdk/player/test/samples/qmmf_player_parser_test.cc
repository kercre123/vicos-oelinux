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

#define TAG "PlayerTest"

#include <fcntl.h>
#include <sys/mman.h>
#include <utils/Log.h>
#include <utils/String8.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>

#include "common/qmmf_common_utils.h"
#include "player/test/samples/qmmf_player_parser_test.h"


//#define DEBUG
#define TEST_INFO(fmt, args...)  ALOGD(fmt, ##args)
#define TEST_ERROR(fmt, args...) ALOGE(fmt, ##args)
#ifdef DEBUG
#define TEST_DBG  TEST_INFO
#else
#define TEST_DBG(...) ((void)0)
#endif

// Enable this define to dump audio bitstream from demuxer
//#define DUMP_AUDIO_BITSTREAM

// Enable this define to dump PCM from decoder
//#define DUMP_PCM_DATA

// Enable this define to dump video bitstream from demuxer
//#define DUMP_VIDEO_BITSTREAM

// Enable this define to dump YUV from decoder
//#define DUMP_YUV_FRAMES

void PlayerTest::playercb(EventType event_type,
                    void *event_data,
                    size_t event_data_size)
{
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  Event* ev = (Event *)event_data;

  TEST_INFO("%s:%s Event type is %s", TAG, __func__,
      player_test_event_[((int)event_type)]);

  TEST_INFO("%s:%s Player is in %s state", TAG,__func__,
      statemap_[static_cast<uint32_t>(ev->state)]);

  std::map<uint32_t, const char*>::iterator it;

  it = statemap_.find(static_cast<uint32_t>(ev->state));
  if (it != statemap_.end()) {
    current_state_ = statemap_[static_cast<uint32_t>(ev->state)];
    TEST_INFO("%s:%s current_state_ is %s", TAG, __func__, current_state_);
  } else {
    TEST_ERROR("%s:%s key %u not found", TAG,__func__,
        static_cast<uint32_t>(ev->state));
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

void PlayerTest::audiotrackcb(EventType event_type,
    void *event_data, size_t event_data_size) {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

void PlayerTest::videotrackcb(EventType event_type,
    void *event_data, size_t event_data_size) {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

PlayerTest::PlayerTest()
    : filename_(nullptr), stopped_(false),
      start_again_(false),paused_(false),
      current_state_("Idle") {
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  player_test_event_[0] = "Error";
  player_test_event_[1] = "State Changed";

  statemap_.insert(std::pair<uint32_t, const char *> (0,"Error"));
  statemap_.insert(std::pair<uint32_t, const char *> (1,"Idle"));
  statemap_.insert(std::pair<uint32_t, const char *> (2,"Prepared"));
  statemap_.insert(std::pair<uint32_t, const char *> (4,"Started"));
  statemap_.insert(std::pair<uint32_t, const char *> (8,"Paused"));
  statemap_.insert(std::pair<uint32_t, const char *> (16,"Stopped"));
  statemap_.insert(std::pair<uint32_t, const char *> (32,"Playback Completed"));

  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

PlayerTest::~PlayerTest() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  statemap_.clear();
  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

int32_t PlayerTest::Connect() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  PlayerCb player_cb_;

  player_cb_.event_cb = [&] (EventType event_type, void *event_data,
                  size_t event_data_size) {playercb(event_type,event_data,
                  event_data_size); };
  auto ret = player_.Connect(player_cb_);
  if (ret != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to Connect", TAG, __func__);
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

int32_t PlayerTest::Disconnect() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  auto ret = player_.Disconnect();
  if (ret != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to Disconnect", TAG, __func__);
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

int32_t PlayerTest::Prepare() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  std::lock_guard<std::mutex> lock(state_change_lock_);

  auto result = 0;

  // Create Audio Track
  AudioTrackCreateParam audio_track_param_;
  memset(&audio_track_param_, 0x0, sizeof audio_track_param_);

  result = ParseFile(audio_track_param_);
  if (result != NO_ERROR) {
    TEST_INFO("%s:%s unable to parse file", TAG, __func__);
  }

  uint32_t track_id_1 =1;
  TrackCb audio_track_cb_;

  audio_track_param_.out_device  = AudioOutSubtype::kBuiltIn;

  audio_track_cb_.event_cb = [&] (EventType event_type, void *event_data,
      size_t event_data_size) {audiotrackcb(event_type, event_data,
      event_data_size);};

  result = player_.CreateAudioTrack(track_id_1,audio_track_param_,
      audio_track_cb_);

  result = player_.Prepare();
  if (result != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to Prepare", TAG, __func__);
  }

  start_again_ = false;
  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return result;
}

int32_t PlayerTest::ParseFile(AudioTrackCreateParam& audio_track_param_) {
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  auto result = 0;

  switch(filetype_)
  {
    case AudioFileType::kAAC:
      aac_file_io_ = new AACfileIO(filename_);
      result = aac_file_io_->Fillparams(&audio_track_param_);
      if (result != NO_ERROR) {
        TEST_INFO("%s:%s Could not fill the AAC params", TAG, __func__);
      }
      break;

    case AudioFileType::kG711:
      g711_file_io_ = new G711fileIO(filename_);
      result = g711_file_io_->Fillparams(&audio_track_param_);
      if (result != NO_ERROR) {
        TEST_INFO("%s:%s Could not fill the G711 params", TAG, __func__);
      }
      break;

    case AudioFileType::kAMR:
      amr_file_io_ = new AMRfileIO(filename_);
      result = amr_file_io_->Fillparams(&audio_track_param_);
      if (result != NO_ERROR) {
        TEST_INFO("%s:%s Could not fill the AMR params", TAG, __func__);
      }
      break;

    default:
      break;
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return result;
}

int32_t PlayerTest::Start() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  std::lock_guard<std::mutex> lock(state_change_lock_);
  auto ret = 0;

  if(start_again_)
  {
    // Create Audio Track
    AudioTrackCreateParam audio_track_param_;
    memset(&audio_track_param_, 0x0, sizeof audio_track_param_);
    ret = ParseFile(audio_track_param_);
    if (ret != NO_ERROR) {
      TEST_INFO("%s:%s unable to parse file", TAG, __func__);
    }
  }

  ret = player_.Start();
  if (ret != NO_ERROR) {
    TEST_INFO("%s:%s unable to start playabck", TAG, __func__);
  }
  stopped_ = false;
  paused_ = false;

  ret = pthread_create(&start_thread_id, NULL, PlayerTest::StartPlaying,
      (void*)this);
  if (ret != NO_ERROR) {
    TEST_INFO("%s:%s unable to create StartPlaying thread", TAG, __func__);
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

void * PlayerTest::StartPlaying(void *ptr) {
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  auto ret = 0;

  uint32_t track_id_1 = 1;
  uint32_t result;

  PlayerTest* playertest = static_cast<PlayerTest *>(ptr);
  std::vector<TrackBuffer> buffers;
  TrackBuffer tb;
  const char *current_state;

  while (1)
  {

    {
      std::lock_guard<std::mutex> lock(playertest->state_change_lock_);
      current_state = playertest->current_state_;
    }

    if (playertest->paused_ || (strcmp(current_state,"Paused") == 0)||
        !(strcmp(current_state,"Started") == 0)) {
      continue;
    }

    memset(&tb,0x0,sizeof(tb));
    buffers.push_back(tb);
    uint32_t val = 1;

    ret = playertest->player_.DequeueInputBuffer(track_id_1,buffers);
    assert(NO_ERROR == ret);

    int32_t num_frames_read;
    uint32_t bytes_read;

    switch(playertest->filetype_)
    {
      case AudioFileType::kAAC:
        //For AAC
        result = playertest->aac_file_io_->GetFrames(
            (void*)buffers[0].data,buffers[0].size,&num_frames_read,&bytes_read);
        break;

      case AudioFileType::kG711:
        //For G711
        result = playertest->g711_file_io_->GetFrames(
            (void*)buffers[0].data,buffers[0].size,&bytes_read);
        break;

      case AudioFileType::kAMR:
        //For AMR
        result = playertest->amr_file_io_->GetFrames(
            (void*)buffers[0].data,buffers[0].size,&num_frames_read,&bytes_read);
        break;

       default:
         break;
    }

    buffers[0].filled_size = bytes_read;

    if (result != 0 || playertest->stopped_) {
      //EOF reached or Stopped
      TEST_INFO("%s:%s:File read completed result is %d", TAG, __func__, result);
      buffers[0].flag = 1;
      ret = playertest->player_.QueueInputBuffer(track_id_1,buffers,(void*)&val,
          sizeof (uint32_t),TrackMetaBufferType::kNone);
      assert(NO_ERROR == ret);
      buffers.clear();

      {
        std::lock_guard<std::mutex> lock(playertest->state_change_lock_);
        playertest->stopped_ = true;
      }

      playertest->StopPlaying();
      break;
    }

    TEST_DBG("%s:%s: filled_size %d", TAG, __func__, buffers[0].filled_size);
    TEST_DBG("%s:%s: buffer size %d", TAG, __func__, buffers[0].size);
    TEST_DBG("%s:%s: vaddr 0x%p", TAG, __func__, buffers[0].data);

    ret = playertest->player_.QueueInputBuffer(track_id_1,buffers,(void*)&val,
        sizeof (uint32_t),TrackMetaBufferType::kNone);
    assert(NO_ERROR == ret);

    buffers.clear();

  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return NULL;
}

int32_t PlayerTest::Stop() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  std::lock_guard<std::mutex> lock(state_change_lock_);
  stopped_ = true;

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return 0;
}

int32_t PlayerTest::StopPlaying() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  std::lock_guard<std::mutex> lock(state_change_lock_);
  auto ret = 0;

  ret = player_.Stop(false);
  if (ret != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to Stop", TAG, __func__);
  }

  switch(filetype_)
  {
    case AudioFileType::kAAC:
      delete aac_file_io_;
      break;
    case AudioFileType::kG711:
      delete g711_file_io_;
      break;
    case AudioFileType::kAMR:
      delete amr_file_io_;
      break;
  }
  start_again_ = true;
  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

int32_t PlayerTest::Pause() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  std::lock_guard<std::mutex> lock(state_change_lock_);

  paused_ = true;
  auto ret = player_.Pause();
  if (ret != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to Pause", TAG, __func__);
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

int32_t PlayerTest::Resume()
{
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  std::lock_guard<std::mutex> lock(state_change_lock_);

  paused_ = false;
  auto ret = player_.Resume();
  if (ret != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to Resume", TAG, __func__);
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

int32_t PlayerTest::SetPosition() {
  auto ret = 0;
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

int32_t PlayerTest::SetTrickMode() {
  auto ret = 0;
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

int32_t PlayerTest::GrabPicture() {
  auto ret = 0;
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

int32_t PlayerTest::Delete() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  std::lock_guard<std::mutex> lock(state_change_lock_);

  auto ret = 0;
  uint32_t track_id_1 =1;
  ret = player_.DeleteAudioTrack(track_id_1);
  if (ret != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to DeleteAudioTrack", TAG, __func__);
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

void CmdMenu::PrintMenu() {

  printf("\n\n=========== PLAYER TEST MENU ===================\n\n");

  printf(" \n\nPlayer Test Application commands \n");
  printf(" -----------------------------\n");
  printf("   %c. Connect\n", CmdMenu::CONNECT_CMD);
  printf("   %c. Disconnect\n", CmdMenu::DISCONNECT_CMD);
  printf("   %c. Prepare\n", CmdMenu::PREPARE_CMD);
  printf("   %c. Start\n", CmdMenu::START_CMD);
  printf("   %c. Stop\n", CmdMenu::STOP_CMD);
  printf("   %c. Pause\n", CmdMenu::PAUSE_CMD);
  printf("   %c. Resume\n", CmdMenu::RESUME_CMD);
  printf("   %c. Delete\n", CmdMenu::DELETE_CMD);
  printf("   %c. Exit\n", CmdMenu::EXIT_CMD);
  printf("\n   Choice: ");
}

CmdMenu::Command CmdMenu::GetCommand(bool& is_print_menu) {
  if (is_print_menu) {
    PrintMenu();
    is_print_menu = false;
  }
  return CmdMenu::Command(static_cast<CmdMenu::CommandType>(getchar()));
}

int main(int argc,char *argv[]) {

  TEST_INFO("%s:%s: Enter", TAG, __func__);

  PlayerTest test_context;

  CmdMenu cmd_menu(test_context);

  bool is_print_menu = true;
  int32_t exit_test = false;

  if (argc == 2) {
    test_context.filename_ = argv[1];
    char *extn = strrchr(argv[1], '.');

    TEST_INFO("%s: exten is: %s", TAG, extn);

    if (strcmp(extn, ".aac") == 0)
      test_context.filetype_ = AudioFileType::kAAC;

    else if (strcmp(extn, ".g711") ==0)
      test_context.filetype_ = AudioFileType::kG711;

    else if (strcmp(extn, ".amr") == 0)
      test_context.filetype_ = AudioFileType::kAMR;

    else {
      TEST_ERROR("%s:%s %s extn not supported, supported extn are"
          ".aac, .amr, .g711", TAG,__func__,extn);
      exit_test = true;
        }
  } else {
    TEST_INFO("%s:%s Give some file to play, supported extn"
        "are .aac, .amr, .g711 ", TAG,__func__);
    exit_test = true;
  }

  while (!exit_test) {

    CmdMenu::Command command = cmd_menu.GetCommand(is_print_menu);
    switch (command.cmd) {

      case CmdMenu::CONNECT_CMD: {
        test_context.Connect();
      }
      break;
      case CmdMenu::DISCONNECT_CMD: {
        test_context.Disconnect();
      }
      break;
      case CmdMenu::PREPARE_CMD: {
        test_context.Prepare();
      }
      break;
      case CmdMenu::START_CMD: {
        test_context.Start();
      }
      break;
      case CmdMenu::STOP_CMD: {
        test_context.Stop();
      }
      break;
      case CmdMenu::PAUSE_CMD: {
        test_context.Pause();
      }
      break;
      case CmdMenu::RESUME_CMD: {
        test_context.Resume();
      }
      break;
      case CmdMenu::DELETE_CMD: {
        test_context.Delete();
      }
      break;
      case CmdMenu::NEXT_CMD: {
        is_print_menu = true;
      }
      break;
      case CmdMenu::EXIT_CMD: {
        exit_test = true;
      }
      break;
      default:
        break;
    }
  }
  return 0;
}
