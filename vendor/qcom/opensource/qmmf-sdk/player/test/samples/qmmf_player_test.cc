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
#include <sys/time.h>
#include <utils/Log.h>
#include <utils/String8.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <fstream>

#include "common/qmmf_common_utils.h"
#include "player/test/samples/qmmf_player_test.h"


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

void PlayerTest::playercb(EventType event_type, void *event_data,
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

void PlayerTest::audiotrackcb(EventType event_type, void *event_data,
                              size_t event_data_size) {

  TEST_INFO("%s:%s: Enter", TAG, __func__);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

void PlayerTest::videotrackcb(EventType event_type, void *event_data,
                              size_t event_data_size) {

  TEST_INFO("%s:%s: Enter", TAG, __func__);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

void PlayerTest::GrabPictureDataCB(BufferDescriptor& buffer) {
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  String8 snapshot_filepath;
  uint32_t size;
  struct timeval tv;
  gettimeofday(&tv, NULL);

  snapshot_filepath.appendFormat("/data/misc/qmmf/player_snapshot_%dx%d_%lu.%s",
      m_sTrackInfo_.sVideo.ulWidth, m_sTrackInfo_.sVideo.ulHeight,
      tv.tv_sec, "yuv");

  grabpicture_file_fd_ = open(snapshot_filepath.string(), O_CREAT |
      O_WRONLY | O_TRUNC, 0655);
  assert(grabpicture_file_fd_ >= 0);

  size = (m_sTrackInfo_.sVideo.ulWidth * m_sTrackInfo_.sVideo.ulHeight*3)/2;
  TEST_DBG("%s:%s: vaddr 0x%p", TAG, __func__,buffer.data);
  TEST_DBG("%s:%s: size %u", TAG, __func__, size);

  uint32_t bytes_written;
  bytes_written  = write(grabpicture_file_fd_, buffer.data, size);
  if (bytes_written !=  size) {
    QMMF_ERROR("Bytes written != %d and written = %u",
        size, bytes_written);
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

PlayerTest::PlayerTest()
    : filename_(nullptr), stopped_(false), stop_playing_(false),
      start_again_(false), audioFirstFrame_(true), videoFirstFrame_(true),
      audioLastFrame_(false), videoLastFrame_(false),
      paused_(false), current_state_("Idle"),
      playback_speed_(TrickModeSpeed::kSpeed_1x),
      playback_dir_(TrickModeDirection::kNormalForward),
      grabpicture_file_fd_(-1), trick_mode_enabled_(false),
      current_playback_time_(0), intermediate_stop_(false) {

  TEST_INFO("%s:%s: Enter", TAG, __func__);

  if(filename_ != nullptr)
    m_pIStreamPort_ = new CMM_MediaSourcePort(filename_);

  player_test_event_[0] = "Error";
  player_test_event_[1] = "State Changed";

  statemap_.insert(std::pair<uint32_t, const char*> (0,"Error"));
  statemap_.insert(std::pair<uint32_t, const char*> (1,"Idle"));
  statemap_.insert(std::pair<uint32_t, const char*> (2,"Prepared"));
  statemap_.insert(std::pair<uint32_t, const char*> (4,"Started"));
  statemap_.insert(std::pair<uint32_t, const char*> (8,"Paused"));
  statemap_.insert(std::pair<uint32_t, const char*> (16,"Stopped"));
  statemap_.insert(std::pair<uint32_t, const char*> (32,"Playback Completed"));

  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

PlayerTest::PlayerTest(char* filename_)
    : filename_(nullptr), stopped_(false), stop_playing_(false),
      start_again_(false), audioFirstFrame_(true), videoFirstFrame_(true),
      audioLastFrame_(false), videoLastFrame_(false),
      paused_(false), current_state_("Idle"),
      playback_speed_(TrickModeSpeed::kSpeed_1x),
      playback_dir_(TrickModeDirection::kNormalForward),
      grabpicture_file_fd_(-1), trick_mode_enabled_(false),
      current_playback_time_(0), intermediate_stop_(false) {

  TEST_INFO("%s:%s: Enter", TAG, __func__);
  if (filename_ != nullptr)
    m_pIStreamPort_ = new CMM_MediaSourcePort(filename_);

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
  if (grabpicture_file_fd_ > 0) {
    close(grabpicture_file_fd_);
  }
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

  if(m_pIStreamPort_ == nullptr && filename_ != nullptr)
    m_pIStreamPort_ = new CMM_MediaSourcePort(filename_);

  auto result = 0;

#ifdef DUMP_AUDIO_BITSTREAM
  srcFile_audio_.open("/data/misc/qmmf/dump_audio.bin", std::ios::binary | std::ios::out);
#endif

#ifdef DUMP_VIDEO_BITSTREAM
  srcFile_video_.open("/data/misc/qmmf/dump_video.bin", std::ios::binary | std::ios::out);
#endif

  AudioTrackCreateParam audio_track_param_;
  memset(&audio_track_param_, 0x0, sizeof audio_track_param_);
  fileCount_audio_ = 0;

  VideoTrackCreateParam video_track_param_;
  memset(&video_track_param_, 0x0, sizeof video_track_param_);
  fileCount_video_ = 0;

  result = ParseFile(audio_track_param_,video_track_param_);
  if (result != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to ParseFile", TAG, __func__);
  }

  if (track_type_ == TrackTypes::kAudioVideo ||
      track_type_ == TrackTypes::kAudioOnly) {

    TrackCb audio_track_cb_;

    audio_track_cb_.event_cb = [&] (EventType event_type, void *event_data,
        size_t event_data_size) {audiotrackcb(event_type, event_data,
        event_data_size);};

    result = player_.CreateAudioTrack(audio_track_id_,audio_track_param_,
                                      audio_track_cb_);
    if (result != NO_ERROR) {
      TEST_ERROR("%s:%s Failed to CreateAudioTrack", TAG, __func__);
    }
  }

  if (track_type_ == TrackTypes::kAudioVideo ||
      track_type_ == TrackTypes::kVideoOnly) {

    TrackCb video_track_cb_;

    video_track_cb_.event_cb = [&] (EventType event_type, void *event_data,
        size_t event_data_size) {videotrackcb(event_type, event_data,
        event_data_size);};

    result = player_.CreateVideoTrack(video_track_id_,video_track_param_,
                                      video_track_cb_);
    if (result != NO_ERROR) {
      TEST_ERROR("%s:%s Failed to CreateVideoTrack", TAG, __func__);
    }
  }

  result = player_.Prepare();
  if (result != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to Prepare", TAG, __func__);
  }

  start_again_ = false;

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return result;
}

int32_t PlayerTest::ParseFile(AudioTrackCreateParam& audio_track_param_,
                              VideoTrackCreateParam& video_track_param_) {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  auto ret = 0;
  AacCodecData           aac_codec_data;

  CreateDataSource();

  if (track_type_ == TrackTypes::kAudioVideo ||
      track_type_ == TrackTypes::kAudioOnly) {

    audio_track_param_.sample_rate = m_sTrackInfo_.sAudio.ulSampleRate;
    audio_track_param_.channels    = m_sTrackInfo_.sAudio.ulChCount;
    audio_track_param_.bit_depth   = 16; //TODO m_sTrackInfo_.sAudio.ulBitDepth;

    if (m_sTrackInfo_.sAudio.ulCodecType == 3) {
      ret = m_pDemux_->GetAACCodecData(audio_track_id_, &aac_codec_data);
      if (ret == false) {
        TEST_ERROR("%s:%s: Failed to get codec info", TAG, __func__);
    }
      TEST_DBG("%s:%s aac codec profile : %u format : %d ", TAG, __func__,
          aac_codec_data.ucAACProfile,
          static_cast<uint32_t>(aac_codec_data.eAACStreamFormat));
      audio_track_param_.codec       = (AudioCodecType)AudioFormat::kAAC;
      audio_track_param_.codec_params.aac.bit_rate =
          m_sTrackInfo_.sAudio.ulBitRate;
      audio_track_param_.codec_params.aac.format   = AACFormat::kRaw;
      audio_track_param_.codec_params.aac.mode     = AACMode::kAALC;
      if (m_sTrackInfo_.sAudio.ulSampleRate == 22050 ||
          m_sTrackInfo_.sAudio.ulSampleRate == 24000) {
        audio_track_param_.sample_rate  = m_sTrackInfo_.sAudio.ulSampleRate * 2;
      }
    } else if (m_sTrackInfo_.sAudio.ulCodecType == 7) {
      audio_track_param_.codec       = (AudioCodecType)AudioFormat::kAMR;
      audio_track_param_.sample_rate = 16000;
      audio_track_param_.channels    = 1;
      audio_track_param_.codec_params.amr.isWAMR   = 0;
    } else if (m_sTrackInfo_.sAudio.ulCodecType == 45) {
      audio_track_param_.codec       = (AudioCodecType)AudioFormat::kAMR;
      audio_track_param_.sample_rate = 16000;
      audio_track_param_.channels    = 1;
      audio_track_param_.codec_params.amr.isWAMR   = 1;
    }
    audio_track_param_.out_device                = AudioOutSubtype::kBuiltIn;
    TEST_INFO("%s:%s audio_track_id_ : %d ", TAG, __func__, audio_track_id_);

    TEST_INFO("%s:%s sample rate : %d channel %d bitdepth %d, bitrate %d ", TAG
        , __func__, audio_track_param_.sample_rate, audio_track_param_.channels,
        audio_track_param_.bit_depth, m_sTrackInfo_.sAudio.ulBitRate);
  }

  if (track_type_ == TrackTypes::kAudioVideo ||
      track_type_ == TrackTypes::kVideoOnly) {

    if (m_sTrackInfo_.sVideo.ulCodecType == 11) {
      video_track_param_.codec       = VideoCodecType::kAVC;
    } else if (m_sTrackInfo_.sVideo.ulCodecType == 12) {
      video_track_param_.codec       = VideoCodecType::kHEVC;
    }

    video_track_param_.frame_rate  = m_sTrackInfo_.sVideo.fFrameRate;
    video_track_param_.height      = m_sTrackInfo_.sVideo.ulHeight;
    video_track_param_.width       = m_sTrackInfo_.sVideo.ulWidth;
    video_track_param_.bitrate     = m_sTrackInfo_.sVideo.ulBitRate;
    video_track_param_.num_buffers = 1;
    video_track_param_.out_device  = VideoOutSubtype::kHDMI;

    TEST_INFO("%s:%s video_track_id_ : %d ", TAG, __func__, video_track_id_);

    TEST_INFO("%s:%s height : %d width %d frame_rate %d, bitrate %d ", TAG
        , __func__, video_track_param_.height, video_track_param_.width,
        video_track_param_.frame_rate, video_track_param_.bitrate);
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);

  return 0;
}

int32_t PlayerTest::Start() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  auto ret = 0;
  FileSourceStatus mFSStatus = FILE_SOURCE_FAIL;

  if(start_again_ && !intermediate_stop_)
  {
    TEST_INFO("%s:%s: Playback Speed(%u) Playback Direction(%u)", TAG, __func__,
        static_cast<uint32_t>(playback_speed_),
        static_cast<uint32_t>(playback_dir_));

    mFSStatus = m_pDemux_->SeekAbsolutePosition(0, true,
        static_cast<int64>(current_playback_time_/1000));

    if (mFSStatus == FILE_SOURCE_FAIL) {
      TEST_INFO("%s:%s: Failed to seek %d", TAG, __func__,
          static_cast<uint32_t>(mFSStatus));
    }
  }

  {
    std::lock_guard<std::mutex> lock(state_change_lock_);
    videoLastFrame_ = false;
    audioLastFrame_ = false;
    stopped_ = false;
    paused_ = false;
    stop_playing_ = false;
  }

  ret = player_.Start();
  if (ret != 0) {
    TEST_ERROR("%s:%s Failed to Start", TAG, __func__);
  }

  if ((track_type_ == TrackTypes::kAudioVideo ||
      track_type_ == TrackTypes::kAudioOnly) && (!IsTrickModeEnabled())) {

    ret = pthread_create(&audio_thread_id_, nullptr, PlayerTest::StartPlayingAudio,
                         (void*)this);
    if (ret != NO_ERROR) {
      TEST_ERROR("%s:%s Failed to create StartPlayingAudio Thread", TAG, __func__);
    }
  }

  if (track_type_ == TrackTypes::kAudioVideo ||
      track_type_ == TrackTypes::kVideoOnly) {

    ret = pthread_create(&video_thread_id_, nullptr, PlayerTest::StartPlayingVideo,
                         (void*)this);
    if (ret != NO_ERROR) {
      TEST_ERROR("%s:%s Failed to create StartPlayingVideo Thread", TAG, __func__);
    }
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

void * PlayerTest::StartPlayingAudio(void *ptr) {
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  auto ret = 0;

  PlayerTest* playertest = static_cast<PlayerTest *>(ptr);
  std::vector<TrackBuffer> buffers;
  TrackBuffer tb;
  const char *current_state;

  while (!(playertest->audioLastFrame_))
  {

    {
      std::lock_guard<std::mutex> lock(playertest->state_change_lock_);
      current_state = playertest->current_state_;
    }

    if (playertest->paused_ || (strcmp(current_state,"Paused") == 0) ||
        !(strcmp(current_state,"Started") == 0) || (playertest->IsTrickModeEnabled())) {
      continue;
    }

    memset(&tb,0x0,sizeof(tb));
    buffers.push_back(tb);
    uint32_t val = 1;

    ret = playertest->player_.DequeueInputBuffer(playertest->audio_track_id_,
        buffers);
    assert(NO_ERROR == ret);

    FileSourceSampleInfo sSampleInfo;
    FileSourceMediaStatus eMediaStatus = FILE_SOURCE_DATA_ERROR;
    memset(&sSampleInfo, 0, sizeof(FileSourceSampleInfo));

    playertest->m_sTrackInfo_.sAudio.sSampleBuf.pucData1 =
        static_cast<uint8*>(buffers[0].data);

    playertest->m_sTrackInfo_.sAudio.sSampleBuf.ulLen = buffers[0].size;

    playertest->m_sTrackInfo_.sAudio.sSampleBuf.ulLen =
        playertest->m_sTrackInfo_.sAudio.sSampleBuf.ulMaxLen;

    uint32_t nFormatBlockSize =0;

    if (playertest->audioFirstFrame_) {
      uint32_t status = playertest->m_pDemux_->m_pFileSource->GetFormatBlock(
          playertest->m_sTrackInfo_.sAudio.ulTkId, nullptr, &nFormatBlockSize);
      TEST_DBG("%s:%s: Audio getFormatBlock size = %lu", TAG, __func__,
          nFormatBlockSize);
      assert(FILE_SOURCE_SUCCESS == status);

      uint8_t *buffer = new uint8_t[nFormatBlockSize];
      if (buffer != nullptr) {
        status = playertest->m_pDemux_->m_pFileSource->GetFormatBlock(
            playertest->m_sTrackInfo_.sAudio.ulTkId, buffer, &nFormatBlockSize);
       assert(FILE_SOURCE_SUCCESS == status);
      }

      memcpy(buffers[0].data , buffer, nFormatBlockSize );
      delete[] buffer;
      playertest->audioFirstFrame_ = false;
    }

    eMediaStatus = playertest->m_pDemux_->GetNextMediaSample(
        playertest->m_sTrackInfo_.sAudio.ulTkId,
        playertest->m_sTrackInfo_.sAudio.sSampleBuf.pucData1+nFormatBlockSize,
        &(playertest->m_sTrackInfo_.sAudio.sSampleBuf.ulLen), sSampleInfo);

#ifdef DUMP_AUDIO_BITSTREAM
    playertest->srcFile_audio_.write((const char*) buffers[0].data,
        playertest->m_sTrackInfo_.sAudio.sSampleBuf.ulLen);
    playertest->fileCount_audio_++;
#endif

    buffers[0].filled_size = playertest->m_sTrackInfo_.sAudio.sSampleBuf.ulLen +
        nFormatBlockSize ;
    buffers[0].time_stamp = sSampleInfo.startTime;

    playertest->UpdateCurrentPlaybackTime(sSampleInfo.startTime);

    if (FILE_SOURCE_DATA_END == eMediaStatus || playertest->IsPlayerStopped()) {
      //EOF reached or Stopped
      TEST_INFO("%s:%s:File read completed", TAG, __func__);
      buffers[0].flag = EOS_FLAG;
      buffers[0].filled_size = EOS_BUFFER_SIZE;

      TEST_DBG("%s:%s: audio_filled_size %d", TAG, __func__,
          buffers[0].filled_size);
      TEST_DBG("%s:%s: audio_buffer size %d", TAG, __func__, buffers[0].size);
      TEST_DBG("%s:%s: audio_vaddr 0x%p", TAG, __func__, buffers[0].data);
      TEST_DBG("%s:%s: audio_frame timestamp %llu", TAG, __func__,
          buffers[0].time_stamp);

      playertest->player_.QueueInputBuffer(playertest->audio_track_id_, buffers,
          (void*)&val, sizeof (uint32_t), TrackMetaBufferType::kNone);
      buffers.clear();
      {
        std::lock_guard<std::mutex> lock(playertest->state_change_lock_);
        playertest->stopped_ = true;
      }

      playertest->audioLastFrame_ = true;
      if (!playertest->IsStopPlaying()) {
        pthread_create(&(playertest->stop_thread_), nullptr, PlayerTest::StopPlaying,
                       (void*)playertest);
      }
      break;
    }

    TEST_DBG("%s:%s: audio_filled_size %d", TAG, __func__,
        buffers[0].filled_size);
    TEST_DBG("%s:%s: audio_buffer size %d", TAG, __func__, buffers[0].size);
    TEST_DBG("%s:%s: audio_vaddr 0x%p", TAG, __func__, buffers[0].data);
    TEST_DBG("%s:%s: audio_frame timestamp %llu", TAG, __func__,
         buffers[0].time_stamp);

    playertest->player_.QueueInputBuffer(playertest->audio_track_id_, buffers,
        (void*)&val, sizeof (uint32_t), TrackMetaBufferType::kNone);
    assert(NO_ERROR == ret);
    buffers.clear();

  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return NULL;
}

void * PlayerTest::StartPlayingVideo(void *ptr) {
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  auto ret = 0;

  PlayerTest* playertest = static_cast<PlayerTest *>(ptr);
  std::vector<TrackBuffer> buffers;
  TrackBuffer tb;
  const char *current_state;
  FileSourceStatus mFSStatus = FILE_SOURCE_FAIL;

  while (!(playertest->videoLastFrame_))
  {

    {
      std::lock_guard<std::mutex> lock(playertest->state_change_lock_);
      current_state = playertest->current_state_;
    }

    if (playertest->paused_ || (strcmp(current_state,"Paused") == 0) ||
        !(strcmp(current_state,"Started") == 0)) {
      continue;
    }

    memset(&tb,0x0,sizeof(tb));
    buffers.push_back(tb);
    uint32_t val = 1;

    ret =playertest->player_.DequeueInputBuffer(playertest->video_track_id_,
        buffers);
    assert(NO_ERROR == ret);

    //Video
    FileSourceSampleInfo sSampleInfo;
    FileSourceMediaStatus eMediaStatus = FILE_SOURCE_DATA_ERROR;
    memset(&sSampleInfo, 0, sizeof(FileSourceSampleInfo));

    playertest->m_sTrackInfo_.sVideo.sSampleBuf.pucData1 =
        static_cast<uint8*>(buffers[0].data);

    playertest->m_sTrackInfo_.sVideo.sSampleBuf.ulLen = buffers[0].size;

    playertest->m_sTrackInfo_.sVideo.sSampleBuf.ulLen =
        playertest->m_sTrackInfo_.sVideo.sSampleBuf.ulMaxLen;

    uint32_t nFormatBlockSize =0;

    if (playertest->videoFirstFrame_) {
      uint32_t status = playertest->m_pDemux_->m_pFileSource->GetFormatBlock(
          playertest->m_sTrackInfo_.sVideo.ulTkId, nullptr, &nFormatBlockSize);
      TEST_DBG("%s:%s: Video getFormatBlock size = %lu", TAG, __func__,
          nFormatBlockSize);
      assert(FILE_SOURCE_SUCCESS == status);

      uint8_t *buffer = new uint8_t[nFormatBlockSize];
      if (buffer != nullptr) {
        status = playertest->m_pDemux_->m_pFileSource->GetFormatBlock(
            playertest->m_sTrackInfo_.sVideo.ulTkId, buffer, &nFormatBlockSize);
        assert(FILE_SOURCE_SUCCESS == status);
      }

      memcpy(buffers[0].data , buffer, nFormatBlockSize );
      delete[] buffer;
      playertest->videoFirstFrame_ = false;
    }

    eMediaStatus = playertest->m_pDemux_->GetNextMediaSample(
        playertest->m_sTrackInfo_.sVideo.ulTkId,
        playertest->m_sTrackInfo_.sVideo.sSampleBuf.pucData1 + nFormatBlockSize,
        &(playertest->m_sTrackInfo_.sVideo.sSampleBuf.ulLen), sSampleInfo);

    if (playertest->playback_dir_ == TrickModeDirection::kNormalRewind) {
      if (sSampleInfo.startTime == 0) {
        eMediaStatus = FILE_SOURCE_DATA_END;
      } else {
        mFSStatus = playertest->m_pDemux_->SeekRelativeSyncPoint(
            static_cast<int>(sSampleInfo.startTime/1000), -2);
        TEST_INFO("%s:%s: REW %u", TAG, __func__, static_cast<uint32_t>(mFSStatus));
      }
    }

#ifdef DUMP_VIDEO_BITSTREAM
    playertest->srcFile_video_.write((const char*) buffers[0].data,
        playertest->m_sTrackInfo_.sVideo.sSampleBuf.ulLen);
    playertest->fileCount_video_++;
#endif

    buffers[0].filled_size = playertest->m_sTrackInfo_.sVideo.sSampleBuf.ulLen +
        nFormatBlockSize;
    buffers[0].time_stamp = sSampleInfo.startTime;

    if (playertest->track_type_ == TrackTypes::kVideoOnly ||
           (playertest->IsTrickModeEnabled())) {
      playertest->UpdateCurrentPlaybackTime(sSampleInfo.startTime);
    }

    if (FILE_SOURCE_DATA_END == eMediaStatus || playertest->IsPlayerStopped()) {
      //EOF reached or Stopped
      TEST_INFO("%s:%s:File read completed", TAG, __func__);
      buffers[0].flag = EOS_FLAG;
      buffers[0].filled_size = EOS_BUFFER_SIZE;

      TEST_DBG("%s:%s: video_filled_size %d", TAG, __func__,
          buffers[0].filled_size);
      TEST_DBG("%s:%s: video_buffer size %d", TAG, __func__, buffers[0].size);
      TEST_DBG("%s:%s: video_vaddr 0x%p", TAG, __func__, buffers[0].data);
      TEST_DBG("%s:%s: video_frame timestamp %llu", TAG, __func__,
          buffers[0].time_stamp);

      playertest->player_.QueueInputBuffer(playertest->video_track_id_, buffers,
          (void*)&val, sizeof (uint32_t), TrackMetaBufferType::kNone);
      buffers.clear();
      {
        std::lock_guard<std::mutex> lock(playertest->state_change_lock_);
        playertest->stopped_ = true;
      }

      playertest->videoLastFrame_ = true;
      if (FILE_SOURCE_DATA_END == eMediaStatus ||
        ((playertest->audioLastFrame_) && (!playertest->IsTrickModeEnabled()))) {
        playertest->intermediate_stop_ = false;
      }

      if (!playertest->IsStopPlaying()) {
        if (playertest->IsTrickModeEnabled()) {
          playertest->intermediate_stop_ = false;
        }
        pthread_create(&(playertest->stop_thread_), nullptr, PlayerTest::StopPlaying,
                       (void*)playertest);
      }
      break;
    }

    TEST_DBG("%s:%s: video_filled_size %d", TAG, __func__,
        buffers[0].filled_size);
    TEST_DBG("%s:%s: video_buffer size %d", TAG, __func__, buffers[0].size);
    TEST_DBG("%s:%s: video_vaddr 0x%p", TAG, __func__, buffers[0].data);
    TEST_DBG("%s:%s: video_frame timestamp %llu", TAG, __func__,
        buffers[0].time_stamp);

    ret = playertest->player_.QueueInputBuffer(playertest->video_track_id_, buffers,
        (void*)&val, sizeof (uint32_t), TrackMetaBufferType::kNone);
    assert(NO_ERROR == ret);
    buffers.clear();

  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return NULL;
}

int32_t PlayerTest::Stop() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  auto ret = 0;

  std::lock_guard<std::mutex> lock(state_change_lock_);
  stopped_ = true;
  intermediate_stop_ = false;

  if (paused_) {
    ret = player_.Resume();
    if (ret != NO_ERROR) {
      TEST_ERROR("%s:%s Failed to Resume", TAG, __func__);
    }
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return 0;
}

bool PlayerTest::IsPlayerStopped() {
  std::lock_guard<std::mutex> lock(state_change_lock_);
  return stopped_;
}

void* PlayerTest::StopPlaying(void* ptr) {
  TEST_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;

  PlayerTest* playertest = static_cast<PlayerTest*>(ptr);

  {
    std::lock_guard<std::mutex> lock(playertest->state_change_lock_);
    playertest->stop_playing_ = true;
  }

  ret = playertest->StopPlayback();

  assert(ret == 0);

  TEST_INFO("%s:%s Exit", TAG, __func__);
  return nullptr;
}

bool PlayerTest::IsStopPlaying() {
  std::lock_guard<std::mutex> lock(state_change_lock_);
  return stop_playing_;
}

int32_t PlayerTest::StopPlayback() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  auto ret = 0;

  if ((track_type_ == TrackTypes::kAudioVideo ||
      track_type_ == TrackTypes::kAudioOnly) && (!IsTrickModeEnabled())) {

    ret = pthread_join(audio_thread_id_, NULL);
    if (ret != NO_ERROR) {
      TEST_ERROR("%s:%s Failed to join StartPlayingAudio Thread", TAG, __func__);
    }
  }

  if (track_type_ == TrackTypes::kAudioVideo ||
      track_type_ == TrackTypes::kVideoOnly) {

    ret = pthread_join(video_thread_id_, NULL);
    if (ret != NO_ERROR) {
      TEST_ERROR("%s:%s Failed to join StartPlayingVideo Thread", TAG, __func__);
    }
  }

  ret = player_.Stop(false);
  if (ret != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to Stop", TAG, __func__);
  }

#ifdef DUMP_AUDIO_BITSTREAM
  if(srcFile_audio_.is_open())
    srcFile_audio_.close();
#endif

#ifdef DUMP_VIDEO_BITSTREAM
  if(srcFile_video_.is_open())
    srcFile_video_.close();
#endif

  {
    std::lock_guard<std::mutex> lock(state_change_lock_);
    start_again_ = true;
  }

  wait_for_state_change_.notify_one();

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

int32_t PlayerTest::Resume() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  std::lock_guard<std::mutex> lock(state_change_lock_);
  auto ret = 0;
  paused_ = false;

   ret = player_.Resume();
  if (ret != NO_ERROR) {
    TEST_ERROR("%s:%s Failed to Resume", TAG, __func__);
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

int32_t PlayerTest::SetPosition() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  auto ret = 0;
  uint64_t time;
  FileSourceStatus mFSStatus = FILE_SOURCE_FAIL;

  uint64_t current_time = GetCurrentPlaybackTime();
  uint64_t clip_duration = m_pDemux_->GetClipDuration();

  printf("\n");
  printf("****** Seek *******\n" );
  printf("Enter time between [0 to %llu sec] :: ", clip_duration/(1000000));
  scanf("%llu", &time);

  mFSStatus = m_pDemux_->SeekAbsolutePosition(time*1000, true,
      static_cast<int64_t>(current_time/1000));
  if (mFSStatus == FILE_SOURCE_FAIL) {
    TEST_INFO("%s:%s: Failed to seek %u to %llu sec", TAG, __func__,
        static_cast<uint32_t>(mFSStatus), time);
  }

  TEST_INFO("%s:%s: Seek to %llu sec", TAG, __func__, time);

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

int32_t PlayerTest::SetTrickMode() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  auto ret = 0;
  FileSourceStatus mFSStatus = FILE_SOURCE_FAIL;

  if (track_type_ == TrackTypes::kAudioVideo ||
      track_type_ == TrackTypes::kVideoOnly) {
    uint32_t dir, speed;

    printf("\n");
    printf("****** Set Trick Mode *******\n" );
    printf(" Enter Trick Mode Type [Normal Playback->1, FF->2, SF->3, REW->4]): ");
    scanf("%d", &dir);
    printf(" Enter Trick Mode Speed/Factor of (supported "
        "[Normal Playback or REW->1 :::: FF,SF-> 2, 4, 8]): ");
    scanf("%d", &speed);

    if ((speed >= 1 && speed <= 8 && (!(speed & (speed-1))))
        && (dir >=1  && dir <= 4)) {

      if (dir == 1 && speed == 1 && (!IsTrickModeEnabled())) {
        std::lock_guard<std::mutex> lock(state_change_lock_);
        trick_mode_enabled_ = false;
      } else {

        {
          std::lock_guard<std::mutex> lock(state_change_lock_);
          trick_mode_enabled_ = true;
          intermediate_stop_ = true;
        }

        uint64_t current_time = GetCurrentPlaybackTime();

        // seek audio tracks to current playback time when normal playback
        if (dir == 1 && speed == 1) {
          mFSStatus = m_pDemux_->SeekAbsolutePosition(audio_track_id_,
              static_cast<int>(current_time/1000), false, -1,
              FS_SEEK_MODE::FS_SEEK_DEFAULT);

          if (mFSStatus == FILE_SOURCE_FAIL) {
            TEST_INFO("%s:%s: Failed to seek %u", TAG, __func__,
                static_cast<uint32_t>(mFSStatus));
          }

          {
            std::lock_guard<std::mutex> lock(state_change_lock_);
            trick_mode_enabled_ = false;
          }
        }

        // set video playback speed and direction
        playback_dir_ = static_cast<TrickModeDirection>(dir);
        playback_speed_ = static_cast<TrickModeSpeed>(speed);
        ret = player_.SetTrickMode(playback_speed_, playback_dir_);

        if (ret != NO_ERROR) {
          TEST_ERROR("%s:%s Failed to SetTrickMode", TAG, __func__);
        }
      }
    } else {
      TEST_INFO("%s:%s:Wrong trick mode type or speed, supported values are "
          "trick mode type [Normal Playback->1, FF->2, SF->3] "
          "speed [Normal Playback->1, 2, 4, 8]", TAG, __func__);
    }
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

bool PlayerTest::IsTrickModeEnabled() {
  std::lock_guard<std::mutex> lock(state_change_lock_);
  return trick_mode_enabled_;
}

int32_t PlayerTest::GrabPicture() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  auto ret = 0;

  PictureParam param_;
  PictureCallback picture_cb_;

  memset(&param_, 0x0, sizeof param_);
  param_.height = m_sTrackInfo_.sVideo.ulHeight;
  param_.width = m_sTrackInfo_.sVideo.ulWidth;
  param_.quality = 1;

  picture_cb_.data_cb = [&] (BufferDescriptor& buffer)
      {GrabPictureDataCB(buffer);};

  player_.GrabPicture(param_, picture_cb_);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

int32_t PlayerTest::Delete() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  std::lock_guard<std::mutex> lock(state_change_lock_);

  auto ret = 0;

  if (track_type_ == TrackTypes::kAudioVideo ||
      track_type_ == TrackTypes::kAudioOnly) {

    ret = player_.DeleteAudioTrack(audio_track_id_);
    if (ret != NO_ERROR) {
      TEST_ERROR("%s:%s Failed to DeleteAudioTrack", TAG, __func__);
    }
  }

  if (track_type_ == TrackTypes::kAudioVideo ||
      track_type_ == TrackTypes::kVideoOnly) {

    ret = player_.DeleteVideoTrack(video_track_id_);
    if (ret != NO_ERROR) {
      TEST_ERROR("%s:%s Failed to DeleteVideoTrack", TAG, __func__);
    }
  }

  delete m_pIStreamPort_;
  m_pIStreamPort_ = NULL;
  audioFirstFrame_ = true;
  videoFirstFrame_ = true;

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

uint32_t PlayerTest::UpdateCurrentPlaybackTime(uint64_t current_time) {
  TEST_DBG("%s:%s: Enter", TAG, __func__);
  std::lock_guard<std::mutex> lock(time_lock_);

  current_playback_time_ = current_time;

  TEST_DBG("%s:%s: Current video playback time %lld", TAG, __func__,
    static_cast<int64_t>(current_playback_time_/1000));
  TEST_DBG("%s:%s: Exit", TAG, __func__);
  return 0;
}

uint32_t PlayerTest::GetCurrentPlaybackTime() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  std::lock_guard<std::mutex> lock(time_lock_);

  TEST_INFO("%s:%s: Current video playback time %lld", TAG, __func__,
      static_cast<int64_t>(current_playback_time_/1000));

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return current_playback_time_;
}

uint32_t PlayerTest::CreateDataSource() {

  TEST_INFO("%s:%s: Enter", TAG, __func__);
  uint32_t eErr = MM_STATUS_ErrorNone;

  m_pDemux_ = CMM_MediaDemuxInt::New(*m_pIStreamPort_, FILE_SOURCE_MPEG4);

  if (!m_pDemux_) {
    TEST_ERROR("%s %s DataSource CreationFAILURE!!", TAG, __func__);
    BAIL_ON_ERROR(MM_STATUS_ErrorDefault);
  }

  TEST_INFO("%s:%s: DataSource Creation SUCCESS!!", TAG, __func__);

  //Read file meta-data
  eErr = ReadMediaInfo();
  BAIL_ON_ERROR(eErr);

  TEST_INFO("%s:%s: Exit", TAG, __func__);

ERROR_BAIL:
  return eErr;
}

uint32_t PlayerTest::ReadMediaInfo() {

  TEST_INFO("%s:%s: Enter", TAG, __func__);

  uint32_t eErr = 0;
  FileSourceTrackIdInfoType aTrackList[MM_SOURCE_MAX_TRACKS];
  FileSourceMjMediaType eMjType = FILE_SOURCE_MJ_TYPE_UNKNOWN;
  FileSourceMnMediaType eMnType = FILE_SOURCE_MN_TYPE_UNKNOWN;
  FileSourceStatus eFS_Status = FILE_SOURCE_FAIL;
  track_type_ = TrackTypes::kAudioVideo;

  // Get total number of tracks available.
  m_sTrackInfo_.ulNumTracks = m_pDemux_->GetWholeTracksIDList(aTrackList);
  TEST_INFO("%s:%s: NumTracks = %u", TAG, __func__, m_sTrackInfo_.ulNumTracks);

  for (uint32 ulIdx = 0; ulIdx < m_sTrackInfo_.ulNumTracks; ulIdx++) {
    FileSourceTrackIdInfoType sTrackInfo = aTrackList[ulIdx];

    // Get MimeType
    eFS_Status = m_pDemux_->GetMimeType(sTrackInfo.id, eMjType, eMnType);
    if(FILE_SOURCE_SUCCESS != eFS_Status) {
      TEST_INFO("%s:%s: Unable to get MIME_TYPE = %u", TAG, __func__,
          eFS_Status);
      continue;
    }

    if (FILE_SOURCE_SUCCESS == eFS_Status ) {
      if ( FILE_SOURCE_MJ_TYPE_AUDIO == eMjType ) {
        TEST_INFO("%s:%s: TRACK_AUDIO @MIME_TYPE = %u", TAG, __func__,
          eMnType);
        m_sTrackInfo_.sAudio.bTrackSelected = sTrackInfo.selected;

       TEST_INFO("%s : %s id:%d ", __func__, TAG, sTrackInfo.id);

        eErr = ReadAudioTrackMediaInfo(sTrackInfo.id, eMnType);
        audio_track_id_ = sTrackInfo.id;
        if (m_sTrackInfo_.ulNumTracks  == 1 ||
            config_track_type_ == TrackTypes::kAudioOnly) {
          track_type_ = TrackTypes::kAudioOnly;
        }

      } else if (FILE_SOURCE_MJ_TYPE_VIDEO == eMjType) {
        TEST_INFO("%s:%s: TRACK_VIDEO @MIME_TYPE = %u", TAG, __func__,
          eMnType);

        m_sTrackInfo_.sVideo.bTrackSelected = sTrackInfo.selected;

        TEST_INFO("%s : %s id:%d ", __func__, TAG, sTrackInfo.id);

        eErr = ReadVideoTrackMediaInfo(sTrackInfo.id, eMnType);
        video_track_id_ = sTrackInfo.id;
        if (m_sTrackInfo_.ulNumTracks  == 1 ||
            config_track_type_ == TrackTypes::kVideoOnly) {
          track_type_ = TrackTypes::kVideoOnly;
        }
      }
    } else {
      eErr = MM_STATUS_ErrorStreamCorrupt;
      TEST_ERROR("%s %sFailed to identify Tracks Error= %u", TAG, __func__,
      eFS_Status);
      BAIL_ON_ERROR(eErr);
    }
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);

ERROR_BAIL:
  return eErr;
}

uint32_t PlayerTest::ReadAudioTrackMediaInfo(
    uint32 ulTkId,
    FileSourceMnMediaType eTkMnType) {

  TEST_INFO("%s:%s: Enter", TAG, __func__);

  MM_STATUS_TYPE eErr = MM_STATUS_ErrorNone;
  FileSourceStatus eFS_Status = FILE_SOURCE_FAIL;
  MediaTrackInfo sMediaInfo;
  memset(&sMediaInfo, 0, sizeof(MediaTrackInfo));

  // Get max buffer size
  m_sTrackInfo_.sAudio.sSampleBuf.ulMaxLen = \
    m_pDemux_->GetTrackMaxFrameBufferSize(ulTkId);

  // Get track media information
  eFS_Status = m_pDemux_->GetMediaTrackInfo(ulTkId, &sMediaInfo);

  // Update track media information
  if (FILE_SOURCE_SUCCESS == eFS_Status) {
    m_sTrackInfo_.sAudio.ulTkId = ulTkId;
    m_sTrackInfo_.sAudio.ulCodecType = eTkMnType;
    m_sTrackInfo_.sAudio.ulChCount = \
                                sMediaInfo.audioTrackInfo.numChannels;
    m_sTrackInfo_.sAudio.ulBitRate = \
                                sMediaInfo.audioTrackInfo.bitRate;
    m_sTrackInfo_.sAudio.ulSampleRate = \
                                sMediaInfo.audioTrackInfo.samplingRate;
    m_sTrackInfo_.sAudio.ulBitDepth = \
                                sMediaInfo.audioTrackInfo.nBitsPerSample;
    m_sTrackInfo_.sAudio.ullDuration = \
                                sMediaInfo.audioTrackInfo.duration;
    m_sTrackInfo_.sAudio.ulTimeScale = \
                                sMediaInfo.audioTrackInfo.timeScale;

    TEST_INFO("%s:%s:Audio CodecType is = %u ", TAG, __func__,
        m_sTrackInfo_.sAudio.ulCodecType);

    TEST_INFO("%s:%s: TkId = %u CH= %u  SR= %u BD=%u", TAG, __func__, ulTkId,
        m_sTrackInfo_.sAudio.ulChCount, m_sTrackInfo_.sAudio.ulSampleRate,
        m_sTrackInfo_.sAudio.ulBitDepth);

    // Get track CSD data len
    eFS_Status = m_pDemux_->GetFormatBlock(ulTkId,
                                          nullptr,
                                          &m_sTrackInfo_.sAudio.sCSD.ulLen,
                                          FALSE);
    BAIL_ON_ERROR(eFS_Status);

    // Get track CSD data if CSD len is valid
    if (0 != m_sTrackInfo_.sAudio.sCSD.ulLen) {
      TEST_INFO("%s:%s: CSD Len = %u", TAG, __func__,
        m_sTrackInfo_.sAudio.sCSD.ulLen);

      m_sTrackInfo_.sAudio.sCSD.pucData = \
        (uint8*)MM_Malloc(sizeof(uint8)* m_sTrackInfo_.sAudio.sCSD.ulLen);
      if (!m_sTrackInfo_.sAudio.sCSD.pucData) {
        eErr = MM_STATUS_ErrorMemAllocFail;
        BAIL_ON_ERROR(eErr);
      }
      eFS_Status = m_pDemux_->GetFormatBlock(ulTkId,
                                            m_sTrackInfo_.sAudio.sCSD.pucData,
                                            &m_sTrackInfo_.sAudio.sCSD.ulLen,
                                            FALSE);
      BAIL_ON_ERROR(eFS_Status);
    }
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);

ERROR_BAIL:
  if (FILE_SOURCE_SUCCESS != eFS_Status) {
    eErr = MM_STATUS_ErrorDefault;
  }
  TEST_ERROR("%s:%s: Return Status %u", TAG, __func__, eErr);
  return eErr;
}

uint32_t PlayerTest::ReadVideoTrackMediaInfo(
    uint32 ulTkId,
    FileSourceMnMediaType eTkMnType) {

  TEST_INFO("%s:%s: Enter", TAG, __func__);

  MM_STATUS_TYPE eErr = MM_STATUS_ErrorNone;
  FileSourceStatus eFS_Status = FILE_SOURCE_FAIL;
  MediaTrackInfo sMediaInfo;
  memset(&sMediaInfo, 0, sizeof(MediaTrackInfo));

  m_sTrackInfo_.sVideo.sSampleBuf.ulMaxLen = \
    m_pDemux_->GetTrackMaxFrameBufferSize(ulTkId);
  eFS_Status = m_pDemux_->GetMediaTrackInfo(ulTkId, &sMediaInfo);
  if(FILE_SOURCE_SUCCESS == eFS_Status){
    m_sTrackInfo_.sVideo.ulTkId = ulTkId;
    m_sTrackInfo_.sVideo.ulCodecType = sMediaInfo.videoTrackInfo.videoCodec;
    m_sTrackInfo_.sVideo.ulWidth = sMediaInfo.videoTrackInfo.frameWidth;
    m_sTrackInfo_.sVideo.ulHeight = sMediaInfo.videoTrackInfo.frameHeight;
    m_sTrackInfo_.sVideo.fFrameRate = sMediaInfo.videoTrackInfo.frameRate;
    m_sTrackInfo_.sVideo.ulBitRate = sMediaInfo.videoTrackInfo.bitRate;
    m_sTrackInfo_.sVideo.ullDuration = sMediaInfo.videoTrackInfo.duration;
    m_sTrackInfo_.sVideo.ulTimeScale = sMediaInfo.videoTrackInfo.timeScale;

    TEST_INFO("%s:%s:Video CodecType is = %u ", TAG, __func__,
        m_sTrackInfo_.sVideo.ulCodecType);

    TEST_INFO("%s:%s: TkId = %u Width= %u  Height= %u FR=%f", TAG, __func__,
        ulTkId, m_sTrackInfo_.sVideo.ulWidth, m_sTrackInfo_.sVideo.ulHeight,
        m_sTrackInfo_.sVideo.fFrameRate);

    // Get CSD data len
    eFS_Status = m_pDemux_->GetFormatBlock(ulTkId,
                                          nullptr,
                                          &m_sTrackInfo_.sVideo.sCSD.ulLen,
                                          FALSE);
    BAIL_ON_ERROR(eFS_Status);
    if (0 != m_sTrackInfo_.sVideo.sCSD.ulLen) {
      TEST_INFO("%s:%s: CSD Len = %u", TAG, __func__,
        m_sTrackInfo_.sVideo.sCSD.ulLen);

      m_sTrackInfo_.sVideo.sCSD.pucData = \
              (uint8*)MM_Malloc(sizeof(uint8)* m_sTrackInfo_.sVideo.sCSD.ulLen);
      if (!m_sTrackInfo_.sVideo.sCSD.pucData) {
        eErr = MM_STATUS_ErrorMemAllocFail;
        TEST_ERROR("%s %s CSD Alloc failure", TAG, __func__);
        BAIL_ON_ERROR(eErr);
      }
      eFS_Status = m_pDemux_->GetFormatBlock(ulTkId,
                                            m_sTrackInfo_.sVideo.sCSD.pucData,
                                            &m_sTrackInfo_.sVideo.sCSD.ulLen,
                                            FALSE);
      BAIL_ON_ERROR(eFS_Status);
    }
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);

ERROR_BAIL:
  if(FILE_SOURCE_SUCCESS != eFS_Status){
    eErr = MM_STATUS_ErrorDefault;
  }

  TEST_ERROR("%s:%s: Return Status %u", TAG, __func__, eErr);
  return eErr;
}

void CmdMenu::HelpMenu(const char * test_name){
  printf("\n Player Test Usage:\n");
  printf("1) %s <mp4 file> \n", test_name);
  printf("2) %s <mp4 file> -o video, for video only playback \n", test_name);
  printf("3) %s <mp4 file> -o audio, for audio only playback \n", test_name);
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
  printf("   %c. SetTrickMode\n", CmdMenu::TRICK_MODE_CMD);
  printf("   %c. GrabPicture\n", CmdMenu::GRAB_PICTURE);
  printf("   %c. Seek\n", CmdMenu::SEEK_CMD);
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

  PlayerTest test_context(argv[1]);

  CmdMenu cmd_menu(test_context);

  bool is_print_menu = true;
  int32_t exit_test = false;

  if (argc == 4) {
    if (strcmp(argv[2], "-o")) {
      cmd_menu.HelpMenu(argv[0]);
      exit_test = true;
    } else {
      if (!(strcmp(argv[3], "video"))) {
        test_context.config_track_type_ = TrackTypes::kVideoOnly;
      } else if (!(strcmp(argv[3], "audio"))) {
        test_context.config_track_type_ = TrackTypes::kAudioOnly;
      }
    }
  }

  if (argc == 2 || (argc == 4 && !exit_test)) {
    char *extn = strrchr(argv[1], '.');
    TEST_INFO("%s: exten is: %s", TAG, extn);
    if (!((strcmp(extn, ".mp4") == 0) || (strcmp(extn, ".MP4") == 0))) {
      TEST_ERROR("%s:%s Player support .mp4/.MP4 extn only", TAG,__func__);
      cmd_menu.HelpMenu(argv[0]);
      exit_test = true;
    }
  } else {
    cmd_menu.HelpMenu(argv[0]);
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
      case CmdMenu::TRICK_MODE_CMD: {
        test_context.SetTrickMode();
      }
      break;
      case CmdMenu::GRAB_PICTURE: {
        test_context.GrabPicture();
      }
      break;
      case CmdMenu::SEEK_CMD: {
        test_context.SetPosition();
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
