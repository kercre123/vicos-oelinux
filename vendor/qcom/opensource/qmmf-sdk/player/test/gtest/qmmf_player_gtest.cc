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

#define TAG "PlayerGTest"

#include <fcntl.h>
#include <sys/mman.h>
#include <utils/Log.h>
#include <utils/String8.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "common/qmmf_common_utils.h"
#include "player/test/gtest/qmmf_player_gtest.h"

//#define DEBUG
#define TEST_INFO(fmt, args...)  ALOGD(fmt, ##args)
#define TEST_ERROR(fmt, args...) ALOGE(fmt, ##args)
#ifdef DEBUG
#define TEST_DBG  TEST_INFO
#else
#define TEST_DBG(...) ((void)0)
#endif

void PlayerGtest::GetGTestParams()
{
  char prop[PROPERTY_VALUE_MAX];
  memset(prop, 0, sizeof(prop));
  property_get("persist.player.gtest.iteration", prop, "0");
  iteration_count_ = (uint32_t) atoi(prop);
  if(iteration_count_ == 0)
      iteration_count_ = DEFAULT_ITERATION;
}

void PlayerGtest::playercb(EventType event_type, void *event_data,
                          size_t event_data_size) {
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

void PlayerGtest::audiotrackcb(EventType event_type, void *event_data,
                              size_t event_data_size) {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

void PlayerGtest::videotrackcb(EventType event_type, void *event_data,
                              size_t event_data_size) {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

void PlayerGtest::SetUp() {

  TEST_INFO("%s:%s Enter ", TAG, __func__);

  test_info_ = ::testing::UnitTest::GetInstance()->current_test_info();

  player_cb_.event_cb = [this] (EventType event_type, void *event_data,
                                size_t event_data_size) -> void
      { playercb(event_type, event_data, event_data_size); };

  GetGTestParams();
  filetype_ = AudioFileType::kAAC;
  stopped_ = false;
  paused_ = false;
  start_again_ = false;
  current_state_ = "Idle";

  player_test_event_[0] = "Error";
  player_test_event_[1] = "State Changed";

  statemap_.insert(std::pair<uint32_t, const char*> (0,"Error"));
  statemap_.insert(std::pair<uint32_t, const char*> (1,"Idle"));
  statemap_.insert(std::pair<uint32_t, const char*> (2,"Prepared"));
  statemap_.insert(std::pair<uint32_t, const char*> (4,"Started"));
  statemap_.insert(std::pair<uint32_t, const char*> (8,"Paused"));
  statemap_.insert(std::pair<uint32_t, const char*> (16,"Stopped"));
  statemap_.insert(std::pair<uint32_t, const char*> (32,"Playback Completed"));

  codec_type_.push_back("aac");
  codec_type_.push_back("amr");
  codec_type_.push_back("g711");

  TEST_INFO("%s:%s Exit ", TAG, __func__);
}

void PlayerGtest::TearDown() {
  TEST_INFO("%s:%s Enter ", TAG, __func__);
  TEST_INFO("%s:%s Exit ", TAG, __func__);
}

int32_t PlayerGtest::Init() {

  auto ret = player_.Connect(player_cb_);
  assert(ret == NO_ERROR);
  return ret;
}

int32_t PlayerGtest::DeInit() {

  auto ret = player_.Disconnect();
  assert(ret == NO_ERROR);
  return ret;
}

/*
* ConnectToService: This test case will test Connect/Disconnect Api.
* Api test sequence:
*  - Connect
*  - Disconnect
*/
TEST_F(PlayerGtest, ConnectToService) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

   auto ret = player_.Connect(player_cb_);
   assert(ret == NO_ERROR);
   sleep(3);

    ret = player_.Disconnect();
    assert(ret == NO_ERROR);
  }
  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* AllocDeallocBufAudioPlayback: This test will test start, stop, pause, resume Api.
* This will allocate and deallocated all buffers in every iteration allocated for
* particular codec.
* Api test sequence:
*  loop Start {
*  ------------------
*  - CreateAudioTrack
*  - Prepare
*  - Start
*
*  loop Start {
*  - Puase
*  - Resume
*  } loop End
*
*  - Stop
*  - DeleteAudioTrack
*   ------------------
*   } loop End
*/
TEST_F(PlayerGtest, AllocDeallocBufAudioPlayback) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  for(uint32_t k = 0; k <= 2; k++) {

    if (k == 0)
      filetype_ = AudioFileType::kAAC;
    else if (k == 1)
      filetype_ = AudioFileType::kAMR;
    else if (k == 2)
      filetype_ = AudioFileType::kG711;

    fprintf(stderr,"\n-----Iterations are runnning for %s codec type ------\n",
        codec_type_[k].c_str());

    for(uint32_t i = 1; i <= iteration_count_; i++) {
      fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
      TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
          test_info_->name(), i);

      ret = Prepare();
      assert(ret == NO_ERROR);
      sleep(2);

      ret = Start();
      assert(ret == NO_ERROR);
      sleep(5);

      for (uint32_t j = 1; j<= 2; j++) {
        ret = Pause();
        assert(ret == NO_ERROR);
        sleep(4);
        ret = Resume();
        assert(ret == NO_ERROR);
        sleep(4);
      }

      sleep(5);
      ret = Stop();
      assert(ret == NO_ERROR);
      sleep(2);

      ret = Delete();
      assert(ret == NO_ERROR);
      sleep(2);
    }
  }
  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* StartStopAudioPlayback: This test will test start, stop, pause, resume Api.
* This will allocate and deallocated all buffers once and will use the same for
* in all iterations for particular codec.
* Api test sequence:
*  ------------------
*  - CreateAudioTrack
*  - Prepare
*  loop Start {
*  - Start
*
*  loop Start {
*  - Puase
*  - Resume
*  } loop End
*
*  - Stop
*  } loop End
*  - DeleteAudioTrack
*   ------------------
*/
TEST_F(PlayerGtest, StartStopAudioPlayback) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  for(uint32_t k = 0; k <= 2; k++) {

    if (k == 0)
      filetype_ = AudioFileType::kAAC;
    else if (k == 1)
      filetype_ = AudioFileType::kAMR;
    else if (k == 2)
      filetype_ = AudioFileType::kG711;

    fprintf(stderr,"\n-----Iterations are runnning for %s codec type ------\n",
        codec_type_[k].c_str());

    ret = Prepare();
    assert(ret == NO_ERROR);
    sleep(2);

    for(uint32_t i = 1; i <= iteration_count_; i++) {
      fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
      TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
          test_info_->name(), i);

      ret = Start();
      assert(ret == NO_ERROR);
      sleep(5);

      for (uint32_t j = 1; j<= 2; j++) {
        ret = Pause();
        assert(ret == NO_ERROR);
        sleep(4);
        ret = Resume();
        assert(ret == NO_ERROR);
        sleep(4);
      }

      sleep(5);
      ret = Stop();
      assert(ret == NO_ERROR);
      sleep(2);
    }

    ret = Delete();
    assert(ret == NO_ERROR);
    sleep(2);

  }
  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* RepeatedAudioPlayback: This test will test start, stop, pause, resume Api.
* This will be the same as StartStopAudioPlayback test but instead of stopped by
* stop command it will be stopped at the EOF and will restart playback again.
* Api test sequence:
*  ------------------
*  - CreateAudioTrack
*  - Prepare
*  loop Start {
*  - Start
*
*  loop Start {
*  - Puase
*  - Resume
*  } loop End
*
*  - Stop AT EOF
*  } loop End
*  - DeleteAudioTrack
*   ------------------
*/
TEST_F(PlayerGtest, RepeatedAudioPlayback) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  assert(ret == NO_ERROR);

  for(uint32_t k = 0; k <= 2; k++) {

    if (k == 0)
      filetype_ = AudioFileType::kAAC;
    else if (k == 1)
      filetype_ = AudioFileType::kAMR;
    else if (k == 2)
      filetype_ = AudioFileType::kG711;

    fprintf(stderr,"\n-----Iterations are runnning for %s codec type ------\n",
        codec_type_[k].c_str());

    ret = Prepare();
    assert(ret == NO_ERROR);
    sleep(2);

    for(uint32_t i = 1; i <= iteration_count_; i++) {
      fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
      TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
          test_info_->name(), i);

      ret = Start();
      assert(ret == NO_ERROR);
      sleep(5);

      for (uint32_t j = 1; j<= 2; j++) {
        ret = Pause();
        assert(ret == NO_ERROR);
        sleep(4);
        ret = Resume();
        assert(ret == NO_ERROR);
        sleep(4);
      }

      while (!(strcmp(current_state_, "Stopped") == 0)){
        sleep(1);
        continue;
      }
    }

    ret = Delete();
    assert(ret == NO_ERROR);
    sleep(2);
  }
  ret = DeInit();
  assert(ret == NO_ERROR);

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

int32_t PlayerGtest::ParseFile(AudioTrackCreateParam& audio_track_param_) {
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  auto result = 0;

  switch(filetype_)
  {
    case AudioFileType::kAAC:
      aac_file_io_ = new AACfileIO("/data/misc/qmmf/test.aac");
      result = aac_file_io_->Fillparams(&audio_track_param_);
      if (result != NO_ERROR) {
        TEST_INFO("%s:%s Could not fill the AAC params", TAG, __func__);
      }
      break;

    case AudioFileType::kG711:
      g711_file_io_ = new G711fileIO("/data/misc/qmmf/test.g711");
      result = g711_file_io_->Fillparams(&audio_track_param_);
      if (result != NO_ERROR) {
        TEST_INFO("%s:%s Could not fill the G711 params", TAG, __func__);
      }
      break;

    case AudioFileType::kAMR:
      amr_file_io_ = new AMRfileIO("/data/misc/qmmf/test.amr");
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

int32_t PlayerGtest::Prepare() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  std::lock_guard<std::mutex> lock(state_change_lock_);

  auto result = 0;

  // Create Audio Track
  AudioTrackCreateParam audio_track_param_;
  memset(&audio_track_param_, 0x0, sizeof audio_track_param_);

  result = ParseFile(audio_track_param_);
  if (result != NO_ERROR) {
    TEST_ERROR("%s:%s unable to parse file", TAG, __func__);
    return result;
  }

  uint32_t track_id_1 =1;
  TrackCb audio_track_cb_;

  audio_track_param_.out_device  = AudioOutSubtype::kBuiltIn;

  audio_track_cb_.event_cb = [&] (EventType event_type, void *event_data,
      size_t event_data_size) {audiotrackcb(event_type, event_data,
      event_data_size);};

  result = player_.CreateAudioTrack(track_id_1,audio_track_param_,
      audio_track_cb_);
  if (result != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to CreateAudioTrack", TAG, __func__);
    return result;
  }

  result = player_.Prepare();
  if (result != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to Prepare", TAG, __func__);
    player_.DeleteAudioTrack(track_id_1);
    return result;
  }

  start_again_ = false;

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return result;
}

int32_t PlayerGtest::Start() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  std::lock_guard<std::mutex> lock(state_change_lock_);

  auto ret = 0;

  if (start_again_) {
    // Create Audio Track
    AudioTrackCreateParam audio_track_param_;
    memset(&audio_track_param_, 0x0, sizeof audio_track_param_);
    ret = ParseFile(audio_track_param_);
    if (ret != NO_ERROR) {
      TEST_ERROR("%s:%s unable to parse file", TAG, __func__);
      return ret;
    }
  }

  ret = player_.Start();
  if (ret != NO_ERROR) {
    TEST_ERROR("%s:%s unable to start playabck", TAG, __func__);
    return ret;
  }

  stopped_ = false;
  paused_ = false;

  ret = pthread_create(&start_thread_id, NULL, PlayerGtest::StartPlaying,
                       (void*)this);
  if (ret != NO_ERROR) {
    TEST_ERROR("%s:%s unable to create StartPlaying thread", TAG, __func__);
    return ret;
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

void * PlayerGtest::StartPlaying(void *ptr) {
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  auto ret = 0;

  uint32_t track_id_1 = 1;
  uint32_t result;
  const char *current_state;

  PlayerGtest* playergtest = static_cast<PlayerGtest *>(ptr);
  std::vector<TrackBuffer> buffers;
  TrackBuffer tb;

  while (1) {

    {
      std::lock_guard<std::mutex> lock(playergtest->state_change_lock_);
      current_state = playergtest->current_state_;
    }

    if (playergtest->paused_ || (strcmp(current_state, "Paused") == 0)||
        !(strcmp(current_state, "Started") == 0)) {
      continue;
    }

    memset(&tb,0x0,sizeof(tb));
    buffers.push_back(tb);
    uint32_t val = 1;

    ret = playergtest->player_.DequeueInputBuffer(track_id_1,buffers);
    assert(NO_ERROR == ret);

    int32_t num_frames_read;
    uint32_t bytes_read;

    switch (playergtest->filetype_) {
      case AudioFileType::kAAC:
        //For AAC
        result = playergtest->aac_file_io_->GetFrames(
            (void*)buffers[0].data,buffers[0].size,&num_frames_read,&bytes_read);
        break;

      case AudioFileType::kG711:
        //For G711
        result = playergtest->g711_file_io_->GetFrames(
            (void*)buffers[0].data,buffers[0].size,&bytes_read);
        break;

      case AudioFileType::kAMR:
        //For AMR
        result = playergtest->amr_file_io_->GetFrames(
            (void*)buffers[0].data,buffers[0].size,&num_frames_read,&bytes_read);
        break;

       default:
         break;
    }

    buffers[0].filled_size = bytes_read;

    if (result != 0 || playergtest->stopped_) {
      //EOF reached or Stopped
      TEST_INFO("%s:%s:File read completed result is %d", TAG, __func__, result);
      buffers[0].flag = 1;
      ret = playergtest->player_.QueueInputBuffer(track_id_1,buffers,(void*)&val,
          sizeof (uint32_t),TrackMetaBufferType::kNone);
      assert(NO_ERROR == ret);
      buffers.clear();

      {
        std::lock_guard<std::mutex> lock(playergtest->state_change_lock_);
        playergtest->stopped_ = true;
      }

      playergtest->StopPlaying();
      break;
    }

    TEST_DBG("%s:%s: filled_size %d", TAG, __func__, buffers[0].filled_size);
    TEST_DBG("%s:%s: buffer size %d", TAG, __func__, buffers[0].size);
    TEST_DBG("%s:%s: vaddr 0x%p", TAG, __func__, buffers[0].data);

    ret = playergtest->player_.QueueInputBuffer(track_id_1,buffers,(void*)&val,
        sizeof (uint32_t),TrackMetaBufferType::kNone);
    assert(NO_ERROR == ret);

    buffers.clear();

  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return NULL;
}

int32_t PlayerGtest::Stop() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  std::lock_guard<std::mutex> lock(state_change_lock_);
  stopped_ = true;

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return 0;
}

int32_t PlayerGtest::StopPlaying() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  std::lock_guard<std::mutex> lock(state_change_lock_);
  auto ret = -1;

  pthread_join(start_thread_id, NULL);

  ret = player_.Stop(false);
  if (ret != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to Stop", TAG, __func__);
  }

  switch (filetype_) {
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

int32_t PlayerGtest::Pause() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  std::lock_guard<std::mutex> lock(state_change_lock_);

  paused_ = true;

  auto ret = player_.Pause();
  if (ret != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to Pause", TAG, __func__);
    return ret;
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

int32_t PlayerGtest::Resume() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  std::lock_guard<std::mutex> lock(state_change_lock_);
  paused_ = false;

  auto ret = player_.Resume();
  if (ret != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to Resume", TAG, __func__);
    return ret;
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

int32_t PlayerGtest::Delete() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  std::lock_guard<std::mutex> lock(state_change_lock_);

  auto ret = 0;
  uint32_t track_id_1 =1;
  ret = player_.DeleteAudioTrack(track_id_1);
  if (ret != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to DeleteAudioTrack", TAG, __func__);
    return ret;
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}
