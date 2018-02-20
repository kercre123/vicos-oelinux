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

#define LOG_TAG "PlayerParserTest"

#include <fcntl.h>
#include <sys/mman.h>
#include <utils/String8.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>

#include "common/utils/qmmf_common_utils.h"
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

void PlayerTest::PlayerHandler(EventType event_type,
                               void* event_data,
                               size_t event_data_size)
{
  TEST_INFO("%s: Enter", __func__);
  TEST_INFO("%s event_type[%d]", __func__,
            static_cast<int32_t>(event_type));

  if (event_type == EventType::kStopped) {
    if (thread_ != nullptr) {
      thread_->join();
      delete thread_;
      thread_ = nullptr;
    }

    printf("\nPlayback has finished.\n");
  }

  TEST_INFO("%s: Exit", __func__);
}

void PlayerTest::AudioTrackHandler(uint32_t track_id,
                                   EventType event_type,
                                   void* event_data,
                                   size_t event_data_size) {
  TEST_INFO("%s: Enter", __func__);
  TEST_INFO("%s event_type[%d]", __func__,
            static_cast<int32_t>(event_type));
  TEST_INFO("%s track_id[%u]", __func__, track_id);
  TEST_INFO("%s: Exit", __func__);
}

void PlayerTest::VideoTrackHandler(uint32_t track_id,
                                   EventType event_type,
                                   void* event_data,
                                   size_t event_data_size) {
  TEST_INFO("%s: Enter", __func__);
  TEST_INFO("%s event_type[%d]", __func__,
            static_cast<int32_t>(event_type));
  TEST_INFO("%s track_id[%u]", __func__, track_id);
  TEST_INFO("%s: Exit", __func__);
}

PlayerTest::PlayerTest()
    : filename_(nullptr),
      thread_(nullptr),
      state_(State::kStopped),
      start_again_(false),
      volume_(20),
      pcm_file_io_(nullptr),
      aac_file_io_(nullptr),
      g711_file_io_(nullptr),
      amr_file_io_(nullptr),
      mp3_file_io_(nullptr) {
  TEST_INFO("%s: Enter", __func__);
  TEST_INFO("%s: Exit", __func__);
}

PlayerTest::~PlayerTest() {
  TEST_INFO("%s: Enter", __func__);
  TEST_INFO("%s: Exit", __func__);
}

void PlayerTest::Connect() {
  TEST_INFO("%s: Enter", __func__);

  PlayerCb callback;
  callback.event_cb = [this](EventType event_type,
                             void *event_data,
                             size_t event_data_size) {
    PlayerHandler(event_type, event_data, event_data_size);
  };

  auto result = player_.Connect(callback);
  assert(result == NO_ERROR);

  TEST_INFO("%s: Exit", __func__);
}

void PlayerTest::Disconnect() {
  TEST_INFO("%s: Enter", __func__);

  auto result = player_.Disconnect();
  assert(result == NO_ERROR);

  TEST_INFO("%s: Exit", __func__);
}

void PlayerTest::Prepare() {
  TEST_INFO("%s: Enter", __func__);
  std::lock_guard<std::mutex> lock(lock_);
  auto result = 0;

  AudioTrackCreateParam audio_track_param;
  memset(&audio_track_param, 0x0, sizeof audio_track_param);

  result = ParseFile(audio_track_param);
  assert(result == NO_ERROR);

  uint32_t track_id_1 = 1;

  audio_track_param.out_device = AudioOutSubtype::kBuiltIn;

  TrackCb callback;
  callback.event_cb = [this] (uint32_t track_id,
                              EventType event_type,
                              void *event_data,
                              size_t event_data_size) {
    AudioTrackHandler(track_id, event_type, event_data, event_data_size);
  };

  result = player_.CreateAudioTrack(track_id_1, audio_track_param, callback);
  assert(result == NO_ERROR);

  result = player_.Prepare();
  assert(result == NO_ERROR);

  start_again_ = false;

  TEST_INFO("%s: Exit", __func__);
}

int32_t PlayerTest::ParseFile(AudioTrackCreateParam& audio_track_param) {
  TEST_INFO("%s: Enter", __func__);
  auto result = 0;

  switch(filetype_)
  {
    case AudioFileType::kPCM:
      pcm_file_io_ = new PCMfileIO(filename_);
      result = pcm_file_io_->Fillparams(&audio_track_param);
      if (result != NO_ERROR)
        TEST_INFO("%s Could not fill the PCM params", __func__);
      break;

    case AudioFileType::kAAC:
      aac_file_io_ = new AACfileIO(filename_);
      result = aac_file_io_->Fillparams(&audio_track_param);
      if (result != NO_ERROR)
        TEST_INFO("%s Could not fill the AAC params", __func__);
      break;

    case AudioFileType::kG711:
      g711_file_io_ = new G711fileIO(filename_);
      result = g711_file_io_->Fillparams(&audio_track_param);
      if (result != NO_ERROR)
        TEST_INFO("%s Could not fill the G711 params", __func__);
      break;

    case AudioFileType::kAMR:
      amr_file_io_ = new AMRfileIO(filename_);
      result = amr_file_io_->Fillparams(&audio_track_param);
      if (result != NO_ERROR)
        TEST_INFO("%s Could not fill the AMR params", __func__);
      break;

    case AudioFileType::kMP3:
      mp3_file_io_ = new MP3fileIO(filename_);
      result = mp3_file_io_->Fillparams(&audio_track_param);
      if (result != NO_ERROR)
        TEST_INFO("%s Could not fill the MP3 params", __func__);
      break;

    default:
      break;
  }

  TEST_INFO("%s: Exit", __func__);
  return result;
}

void PlayerTest::Start() {
  TEST_INFO("%s: Enter", __func__);
  std::lock_guard<std::mutex> lock(lock_);
  auto result = 0;

  if(start_again_)
  {
    AudioTrackCreateParam audio_track_param;
    memset(&audio_track_param, 0x0, sizeof audio_track_param);

    result = ParseFile(audio_track_param);
    assert(result == NO_ERROR);
  }

  result = player_.Start();
  assert(result == NO_ERROR);

  uint32_t track_id_1 = 1;
  result = player_.SetAudioTrackParam(track_id_1,
                                      CodecParamType::kAudioVolumeParamType,
                                      &volume_, sizeof(volume_));
  assert(result == NO_ERROR);

  state_ = State::kRunning;
  thread_ = new thread(PlayerTest::ThreadEntry, this);
  assert(thread_ != nullptr);

  TEST_INFO("%s: Exit", __func__);
}

void PlayerTest::ThreadEntry(PlayerTest* player_test) {
  QMMF_DEBUG("%s() TRACE", __func__);

  player_test->Thread();
}

void PlayerTest::Thread() {
  TEST_INFO("%s: Enter", __func__);
  auto ret = 0;
  uint32_t track_id_1 = 1;
  uint32_t result = 0;

  while (state_ != State::kStopped) {
    {
      std::lock_guard<std::mutex> lock(lock_);
      if (state_ == State::kPaused) continue;
    }

    std::vector<TrackBuffer> buffers;
    TrackBuffer tb;
    memset(&tb, 0x0, sizeof tb);
    buffers.push_back(tb);

    ret = player_.DequeueInputBuffer(track_id_1, buffers);
    assert(ret == NO_ERROR);

    for (TrackBuffer& buffer : buffers) {
      int32_t num_frames_read;
      uint32_t bytes_read;

      switch (filetype_)
      {
        case AudioFileType::kPCM:
          result = pcm_file_io_->GetFrames(buffer.data, buffer.size,
                                           &bytes_read);
          break;
        case AudioFileType::kAAC:
          result = aac_file_io_->GetFrames(buffer.data, buffer.size,
                                           &num_frames_read, &bytes_read);
          break;
        case AudioFileType::kG711:
          result = g711_file_io_->GetFrames(buffer.data, buffer.size,
                                            &bytes_read);
          break;
        case AudioFileType::kAMR:
          result = amr_file_io_->GetFrames(buffer.data, buffer.size,
                                           &num_frames_read, &bytes_read);
          break;
        case AudioFileType::kMP3:
          result = mp3_file_io_->GetFrames(buffer.data, buffer.size,
                                           &bytes_read);
          break;
        default:
          break;
      }
      buffer.filled_size = bytes_read;

      if (result != 0) {
        // EOF reached
        TEST_INFO("%s: File read completed result is %d",
                  __func__, result);
        buffer.flag |= static_cast<uint32_t>(BufferFlags::kFlagEOS);

        {
          std::lock_guard<std::mutex> lock(lock_);
          state_ = State::kStopped;
        }
      }

      TEST_DBG("%s: filled_size %d", __func__, buffer.filled_size);
      TEST_DBG("%s: buffer size %d", __func__, buffer.size);
      TEST_DBG("%s: vaddr 0x%p", __func__, buffer.data);

      ret = player_.QueueInputBuffer(track_id_1, buffers, nullptr, 0,
                                     TrackMetaBufferType::kNone);
      assert(ret == NO_ERROR);
    }
    buffers.clear();
  }

  switch (filetype_) {
    case AudioFileType::kPCM: delete pcm_file_io_; break;
    case AudioFileType::kAAC: delete aac_file_io_; break;
    case AudioFileType::kG711: delete g711_file_io_; break;
    case AudioFileType::kAMR: delete amr_file_io_; break;
    case AudioFileType::kMP3: delete mp3_file_io_; break;
  }

  start_again_ = true;

  TEST_INFO("%s: Exit", __func__);
}

void PlayerTest::Stop() {
  TEST_INFO("%s: Enter", __func__);

  {
    std::lock_guard<std::mutex> lock(lock_);
    state_ = State::kStopped;
  }

  auto result = player_.Stop();
  assert(result == NO_ERROR);

  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
    thread_ = nullptr;
  }

  TEST_INFO("%s: Exit", __func__);
}

void PlayerTest::Pause() {
  TEST_INFO("%s: Enter", __func__);
  std::lock_guard<std::mutex> lock(lock_);

  if (state_ == State::kRunning) state_ = State::kPaused;

  auto result = player_.Pause();
  assert(result == NO_ERROR);

  TEST_INFO("%s: Exit", __func__);
}

void PlayerTest::Resume()
{
  TEST_INFO("%s: Enter", __func__);
  std::lock_guard<std::mutex> lock(lock_);

  if (state_ == State::kPaused) state_ = State::kRunning;

  auto result = player_.Resume();
  assert(result == NO_ERROR);

  TEST_INFO("%s: Exit", __func__);
}

void PlayerTest::SetPosition() {
  TEST_ERROR("%s: not implemented", __func__);
  assert(false);
}

void PlayerTest::SetTrickMode() {
  TEST_ERROR("%s: not implemented", __func__);
  assert(false);
}

void PlayerTest::GrabPicture() {
  TEST_ERROR("%s: not implemented", __func__);
  assert(false);
}

void PlayerTest::Delete() {
  TEST_INFO("%s: Enter", __func__);

  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
    thread_ = nullptr;
  }

  uint32_t track_id_1 =1;
  auto result = player_.DeleteAudioTrack(track_id_1);
  assert(result == NO_ERROR);

  TEST_INFO("%s: Exit", __func__);
}

void PlayerTest::AdjustVolume(const int32_t adjustment) {
  TEST_INFO("%s: Enter", __func__);
  std::lock_guard<std::mutex> lock(lock_);

  volume_ += adjustment;

  // clip volume
  if (volume_ > 100)
    volume_ = 100;
  else if (volume_ < 0)
    volume_ = 0;

  printf("\n\nTone Volume is %d\n\n", volume_);

  if (state_ != State::kStopped) {
    uint32_t track_id_1 = 1;
    auto result = player_.SetAudioTrackParam(track_id_1,
        CodecParamType::kAudioVolumeParamType, &volume_, sizeof(volume_));
    assert(result == NO_ERROR);
  }

  TEST_INFO("%s: Exit", __func__);
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
  printf("   %c. Volume Up\n", CmdMenu::VOLUME_UP_CMD);
  printf("   %c. Volume Down\n", CmdMenu::VOLUME_DOWN_CMD);
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

int main(int argc, char* argv[]) {
  QMMF_GET_LOG_LEVEL();

  TEST_INFO("%s: Enter", __func__);

  PlayerTest test_context;

  CmdMenu cmd_menu(test_context);

  bool is_print_menu = true;
  int32_t exit_test = false;

  if (argc == 2) {
    test_context.filename_ = argv[1];
    char *extn = strrchr(argv[1], '.');

    TEST_INFO("exten is: %s", extn);

    if (strcmp(extn, ".wav") == 0)
      test_context.filetype_ = AudioFileType::kPCM;

    else if (strcmp(extn, ".aac") == 0)
      test_context.filetype_ = AudioFileType::kAAC;

    else if (strcmp(extn, ".g711") ==0)
      test_context.filetype_ = AudioFileType::kG711;

    else if (strcmp(extn, ".amr") == 0)
      test_context.filetype_ = AudioFileType::kAMR;

    else if (strcmp(extn, ".mp3") == 0)
      test_context.filetype_ = AudioFileType::kMP3;

    else {
      TEST_ERROR("%s %s extn not supported, supported extn are"
          ".wav, .aac, .amr, .g711, .mp3", __func__,extn);
      exit_test = true;
        }
  } else {
    TEST_INFO("%s Give some file to play, supported extn"
        "are .wav, .aac, .amr, .g711, .mp3", __func__);
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
      case CmdMenu::VOLUME_UP_CMD: {
        test_context.AdjustVolume(1);
      }
      break;
      case CmdMenu::VOLUME_DOWN_CMD: {
        test_context.AdjustVolume(-1);
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
