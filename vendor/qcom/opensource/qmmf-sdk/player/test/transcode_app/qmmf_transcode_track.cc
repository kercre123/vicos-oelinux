/*
* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#include "qmmf_transcode_track.h"

#define TAG "TranscoderTrack"

namespace qmmf {
namespace transcode {

using ::std::string;
using ::std::make_shared;
using ::std::lock_guard;
using ::std::mutex;
using ::std::thread;

TranscoderTrack::TranscoderTrack(const string& file)
    : isFirstFrame_(true),
      isLastFrame_(false),
      input_stop_(false),
      deliver_thread_(nullptr),
      receiver_thread_(nullptr),
      stop_thread_(nullptr),
      num_delivered_frames_(0),
      num_received_frames_(0) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  if (!file.empty())
    params_.track_file = file;
  else {
    QMMF_ERROR("%s:%s Track fileName not Provided", TAG, __func__);
    assert(0);
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

TranscoderTrack::~TranscoderTrack() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

status_t TranscoderTrack::PreparePipeline() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  ret = ParseFile(params_.track_file);
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to parse file", TAG, __func__);
    return ret;
  }

  transcoder_pipe_ = make_shared<TranscoderPipe>(params_.track_type);

  transcoder_source_ = make_shared<TranscoderSource>(
      params_.source_codec_type, params_.source_params, transcoder_pipe_);

  transcoder_sink_ = make_shared<TranscoderSink>(
      params_.sink_codec_type, params_.sink_params, transcoder_pipe_);

  ret = transcoder_source_->PreparePipeline();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to Prepare pipeline on core side", TAG, __func__);
    goto release_resources;
  }

  ret = transcoder_sink_->PreparePipeline();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to Prepare pipeline on sink side", TAG, __func__);
    goto release_resources;
  }

  ret = transcoder_pipe_->PreparePipeline();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to Prepare pipeline on pipe side", TAG, __func__);
    goto release_resources;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;

release_resources:
  ReleaseResources();
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

void TranscoderTrack::ReleaseResources() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  if (transcoder_source_ != nullptr) {
    transcoder_source_->ReleaseResources();
    transcoder_source_.reset();
  }

  if (transcoder_sink_ != nullptr) {
    transcoder_sink_->ReleaseResources();
    transcoder_sink_.reset();
  }

  if (transcoder_pipe_ != nullptr) {
    transcoder_pipe_->ReleaseResources();
    transcoder_pipe_.reset();
  }

  if (m_pIStreamPort_ != nullptr) {
    delete m_pIStreamPort_;
    m_pIStreamPort_ = nullptr;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

status_t TranscoderTrack::FillParams() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  CreateDataSource();

  ::qmmf::player::AudioTrackCreateParam audio_track_param_;
  ::qmmf::player::VideoTrackCreateParam video_track_param_;

  if (track_type_ == TrackTypes::kAudioVideo ||
      track_type_ == TrackTypes::kAudioOnly) {
    audio_track_param_.sample_rate = m_sTrackInfo_.sAudio.ulSampleRate;
    audio_track_param_.channels = m_sTrackInfo_.sAudio.ulChCount;
    audio_track_param_.bit_depth = 16;  // TODO: m_sTrackInfo_.sAudio.ulBitDepth;

    if (m_sTrackInfo_.sAudio.ulCodecType == FILE_SOURCE_MN_TYPE_AAC) {
      audio_track_param_.codec = ::qmmf::player::AudioCodecType::kAAC;
      audio_track_param_.codec_params.aac.bit_rate =
          m_sTrackInfo_.sAudio.ulBitRate;
      audio_track_param_.codec_params.aac.format = AACFormat::kRaw;
      audio_track_param_.codec_params.aac.mode = AACMode::kAALC;
    } else if (m_sTrackInfo_.sAudio.ulCodecType == FILE_SOURCE_MN_TYPE_CONC_AMR) {
      audio_track_param_.codec = ::qmmf::player::AudioCodecType::kAMR;
      audio_track_param_.codec_params.amr.isWAMR = 0;
    } else if (m_sTrackInfo_.sAudio.ulCodecType == FILE_SOURCE_MN_TYPE_AMR_WB) {
      audio_track_param_.codec = ::qmmf::player::AudioCodecType::kAMR;
      audio_track_param_.codec_params.amr.isWAMR = 1;
    }
    audio_track_param_.out_device = AudioOutSubtype::kBuiltIn;

    QMMF_INFO("%s:%s sample rate : %d channel %d bitdepth %d, bitrate %d ",
              TAG, __func__,
              audio_track_param_.sample_rate,
              audio_track_param_.channels,
              audio_track_param_.bit_depth,
              m_sTrackInfo_.sAudio.ulBitRate);
  }

  if (track_type_ == TrackTypes::kAudioVideo ||
      track_type_ == TrackTypes::kVideoOnly) {
    if (m_sTrackInfo_.sVideo.ulCodecType == 11) {
      video_track_param_.codec = ::qmmf::player::VideoCodecType::kAVC;
    } else if (m_sTrackInfo_.sVideo.ulCodecType == 12) {
      video_track_param_.codec = ::qmmf::player::VideoCodecType::kHEVC;
    }

    video_track_param_.frame_rate = m_sTrackInfo_.sVideo.fFrameRate;
    video_track_param_.height = m_sTrackInfo_.sVideo.ulHeight;
    video_track_param_.width = m_sTrackInfo_.sVideo.ulWidth;
    video_track_param_.bitrate = m_sTrackInfo_.sVideo.ulBitRate;
    video_track_param_.num_buffers = 1;
    video_track_param_.out_device = VideoOutSubtype::kHDMI;

    QMMF_INFO("%s:%s height : %d width %d frame_rate %d, bitrate %d ",
              TAG, __func__,
              video_track_param_.height, video_track_param_.width,
              video_track_param_.frame_rate,
              video_track_param_.bitrate);
  }

  if (params_.track_type == TranscodeType::kVideoDecodeVideoEncode)
    params_.source_params.video_dec_param = video_track_param_;
  else {
    QMMF_ERROR("%s:%s Demuxer can only be used in case of "
               "VideoDecodeVideoEncode", TAG, __func__);
    return -1;
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return 0;
}

status_t TranscoderTrack::CreateDataSource() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  int32_t eErr = MM_STATUS_ErrorNone;
  m_pDemux_ = CMM_MediaDemuxInt::New(*m_pIStreamPort_, FILE_SOURCE_MPEG4);

  if (m_pDemux_ == nullptr) {
    QMMF_ERROR("%s %s DataSource CreationFAILURE!!", TAG, __func__);
    BAIL_ON_ERROR(MM_STATUS_ErrorDefault);
  }

  QMMF_INFO("%s:%s: DataSource Creation SUCCESS!!", TAG, __func__);

  // Read file meta-data
  eErr = ReadMediaInfo();
  BAIL_ON_ERROR(eErr);
  QMMF_INFO("%s:%s Exit", TAG, __func__);

ERROR_BAIL:
  return eErr;
}

status_t TranscoderTrack::ReadMediaInfo() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  status_t eErr = 0;
  FileSourceTrackIdInfoType aTrackList[MM_SOURCE_MAX_TRACKS];
  FileSourceMjMediaType eMjType = FILE_SOURCE_MJ_TYPE_UNKNOWN;
  FileSourceMnMediaType eMnType = FILE_SOURCE_MN_TYPE_UNKNOWN;
  FileSourceStatus eFS_Status = FILE_SOURCE_FAIL;
  track_type_ = TrackTypes::kAudioVideo;

  // Get total number of tracks available.
  m_sTrackInfo_.ulNumTracks = m_pDemux_->GetWholeTracksIDList(aTrackList);
  QMMF_INFO("%s:%s: NumTracks = %u", TAG, __func__, m_sTrackInfo_.ulNumTracks);

  for (uint32 ulIdx = 0; ulIdx < m_sTrackInfo_.ulNumTracks; ulIdx++) {
    FileSourceTrackIdInfoType sTrackInfo = aTrackList[ulIdx];

    // Get MimeType
    eFS_Status = m_pDemux_->GetMimeType(sTrackInfo.id, eMjType, eMnType);
    if (FILE_SOURCE_SUCCESS != eFS_Status) {
      QMMF_INFO("%s:%s: Unable to get MIME_TYPE = %u",
                TAG, __func__, eFS_Status);
      continue;
    }

    if (FILE_SOURCE_SUCCESS == eFS_Status) {
      if (FILE_SOURCE_MJ_TYPE_AUDIO == eMjType) {
        QMMF_INFO("%s:%s: TRACK_AUDIO @MIME_TYPE = %u", TAG, __func__, eMnType);
        m_sTrackInfo_.sAudio.bTrackSelected = sTrackInfo.selected;

        QMMF_INFO("%s : %s id:%d ", __func__, TAG, sTrackInfo.id);

        eErr = ReadAudioTrackMediaInfo(sTrackInfo.id, eMnType);
        if (m_sTrackInfo_.ulNumTracks == 1) {
          track_type_ = TrackTypes::kAudioOnly;
        }

      } else if (FILE_SOURCE_MJ_TYPE_VIDEO == eMjType) {
        QMMF_INFO("%s:%s: TRACK_VIDEO @MIME_TYPE = %u", TAG, __func__, eMnType);

        m_sTrackInfo_.sVideo.bTrackSelected = sTrackInfo.selected;

        QMMF_INFO("%s : %s id:%d ", __func__, TAG, sTrackInfo.id);

        eErr = ReadVideoTrackMediaInfo(sTrackInfo.id, eMnType);
        if (m_sTrackInfo_.ulNumTracks == 1) {
          track_type_ = TrackTypes::kVideoOnly;
        }
      }
    } else {
      eErr = MM_STATUS_ErrorStreamCorrupt;
      QMMF_ERROR("%s %s Failed to identify Tracks Error= %u",
                 TAG, __func__, eFS_Status);
      BAIL_ON_ERROR(eErr);
    }
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);

ERROR_BAIL:
  return eErr;
}

status_t TranscoderTrack::ReadAudioTrackMediaInfo(
    const uint32 ulTkId,
    const FileSourceMnMediaType eTkMnType) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  MM_STATUS_TYPE eErr = MM_STATUS_ErrorNone;
  FileSourceStatus eFS_Status = FILE_SOURCE_FAIL;
  MediaTrackInfo sMediaInfo;
  memset(&sMediaInfo, 0, sizeof(MediaTrackInfo));

  // Get max buffer size
  m_sTrackInfo_.sAudio.sSampleBuf.ulMaxLen =
      m_pDemux_->GetTrackMaxFrameBufferSize(ulTkId);

  // Get track media information
  eFS_Status = m_pDemux_->GetMediaTrackInfo(ulTkId, &sMediaInfo);

  // Update track media information
  if (FILE_SOURCE_SUCCESS == eFS_Status) {
    m_sTrackInfo_.sAudio.ulTkId = ulTkId;
    m_sTrackInfo_.sAudio.ulCodecType = eTkMnType;
    m_sTrackInfo_.sAudio.ulChCount = sMediaInfo.audioTrackInfo.numChannels;
    m_sTrackInfo_.sAudio.ulBitRate = sMediaInfo.audioTrackInfo.bitRate;
    m_sTrackInfo_.sAudio.ulSampleRate = sMediaInfo.audioTrackInfo.samplingRate;
    m_sTrackInfo_.sAudio.ulBitDepth = sMediaInfo.audioTrackInfo.nBitsPerSample;
    m_sTrackInfo_.sAudio.ullDuration = sMediaInfo.audioTrackInfo.duration;
    m_sTrackInfo_.sAudio.ulTimeScale = sMediaInfo.audioTrackInfo.timeScale;

    QMMF_INFO("%s:%s:Audio CodecType is = %u ", TAG, __func__,
              m_sTrackInfo_.sAudio.ulCodecType);

    QMMF_INFO("%s:%s: TkId = %u CH= %u  SR= %u BD=%u", TAG, __func__, ulTkId,
              m_sTrackInfo_.sAudio.ulChCount, m_sTrackInfo_.sAudio.ulSampleRate,
              m_sTrackInfo_.sAudio.ulBitDepth);

    // Get track CSD data len
    eFS_Status = m_pDemux_->GetFormatBlock(ulTkId, nullptr,
                                           &m_sTrackInfo_.sAudio.sCSD.ulLen,
                                           FALSE);
    BAIL_ON_ERROR(eFS_Status);

    // Get track CSD data if CSD len is valid
    if (0 != m_sTrackInfo_.sAudio.sCSD.ulLen) {
      QMMF_INFO("%s:%s: CSD Len = %u",
                TAG, __func__, m_sTrackInfo_.sAudio.sCSD.ulLen);

      m_sTrackInfo_.sAudio.sCSD.pucData =
          reinterpret_cast<uint8*>
                          (MM_Malloc(sizeof(uint8) *
                                     m_sTrackInfo_.sAudio.sCSD.ulLen));
      if (!m_sTrackInfo_.sAudio.sCSD.pucData) {
        eErr = MM_STATUS_ErrorMemAllocFail;
        BAIL_ON_ERROR(eErr);
      }
      eFS_Status =
          m_pDemux_->GetFormatBlock(ulTkId, m_sTrackInfo_.sAudio.sCSD.pucData,
                                    &m_sTrackInfo_.sAudio.sCSD.ulLen, FALSE);
      BAIL_ON_ERROR(eFS_Status);
    }
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);

ERROR_BAIL:
  if (FILE_SOURCE_SUCCESS != eFS_Status) {
    eErr = MM_STATUS_ErrorDefault;
  }
  QMMF_ERROR("%s:%s: Return Status %u", TAG, __func__, eErr);
  return eErr;
}

status_t TranscoderTrack::ReadVideoTrackMediaInfo(
    const uint32 ulTkId,
    const FileSourceMnMediaType eTkMnType) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  MM_STATUS_TYPE eErr = MM_STATUS_ErrorNone;
  FileSourceStatus eFS_Status = FILE_SOURCE_FAIL;
  MediaTrackInfo sMediaInfo;
  memset(&sMediaInfo, 0, sizeof(MediaTrackInfo));

  m_sTrackInfo_.sVideo.sSampleBuf.ulMaxLen =
      m_pDemux_->GetTrackMaxFrameBufferSize(ulTkId);
  eFS_Status = m_pDemux_->GetMediaTrackInfo(ulTkId, &sMediaInfo);
  if (FILE_SOURCE_SUCCESS == eFS_Status) {
    m_sTrackInfo_.sVideo.ulTkId = ulTkId;
    m_sTrackInfo_.sVideo.ulCodecType = sMediaInfo.videoTrackInfo.videoCodec;
    m_sTrackInfo_.sVideo.ulWidth = sMediaInfo.videoTrackInfo.frameWidth;
    m_sTrackInfo_.sVideo.ulHeight = sMediaInfo.videoTrackInfo.frameHeight;
    m_sTrackInfo_.sVideo.fFrameRate = sMediaInfo.videoTrackInfo.frameRate;
    m_sTrackInfo_.sVideo.ulBitRate = sMediaInfo.videoTrackInfo.bitRate;
    m_sTrackInfo_.sVideo.ullDuration = sMediaInfo.videoTrackInfo.duration;
    m_sTrackInfo_.sVideo.ulTimeScale = sMediaInfo.videoTrackInfo.timeScale;

    QMMF_INFO("%s:%s:Video CodecType is = %u ",
              TAG, __func__, m_sTrackInfo_.sVideo.ulCodecType);

    QMMF_INFO("%s:%s: TkId = %u Width= %u  Height= %u FR=%f bitrate = %u"
              "duration  = %llu", TAG, __func__, ulTkId,
              m_sTrackInfo_.sVideo.ulWidth, m_sTrackInfo_.sVideo.ulHeight,
              m_sTrackInfo_.sVideo.fFrameRate,
              m_sTrackInfo_.sVideo.ulBitRate,
              m_sTrackInfo_.sVideo.ullDuration);

    // Get CSD data len
    eFS_Status = m_pDemux_->GetFormatBlock(ulTkId, nullptr,
                                           &m_sTrackInfo_.sVideo.sCSD.ulLen,
                                           FALSE);
    BAIL_ON_ERROR(eFS_Status);
    if (0 != m_sTrackInfo_.sVideo.sCSD.ulLen) {
      QMMF_INFO("%s:%s: CSD Len = %u", TAG, __func__,
                m_sTrackInfo_.sVideo.sCSD.ulLen);

      m_sTrackInfo_.sVideo.sCSD.pucData =
          reinterpret_cast<uint8*>
                          (MM_Malloc(sizeof(uint8) *
                                     m_sTrackInfo_.sVideo.sCSD.ulLen));
      if (m_sTrackInfo_.sVideo.sCSD.pucData == nullptr) {
        eErr = MM_STATUS_ErrorMemAllocFail;
        QMMF_ERROR("%s %s CSD Alloc failure", TAG, __func__);
        BAIL_ON_ERROR(eErr);
      }
      eFS_Status = m_pDemux_->GetFormatBlock(ulTkId,
                                             m_sTrackInfo_.sVideo.sCSD.pucData,
                                             &m_sTrackInfo_.sVideo.sCSD.ulLen,
                                             FALSE);
      BAIL_ON_ERROR(eFS_Status);
    }
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);

ERROR_BAIL:
  if (FILE_SOURCE_SUCCESS != eFS_Status) {
    eErr = MM_STATUS_ErrorDefault;
  }

  QMMF_ERROR("%s:%s: Return Status %u", TAG, __func__, eErr);
  return eErr;
}

status_t TranscoderTrack::Start() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  ret = transcoder_source_->StartCodec();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to Start on core side", TAG, __func__);
    return ret;
  }

  ret = transcoder_sink_->StartCodec();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to Start on pipe side", TAG, __func__);
    return ret;
  }

  {
    lock_guard<mutex> lg(input_stop_mutex_);
    input_stop_ = false;
  }

  deliver_thread_ = new thread(TranscoderTrack::DeliverInput,
                               reinterpret_cast<void*>(this));
  if (deliver_thread_ == nullptr) {
    QMMF_ERROR("%s:%s unable to allocate thread", TAG, __func__);
    return -ENOMEM;
  }

  receiver_thread_ = new thread(TranscoderTrack::ReceiveOutput,
                                reinterpret_cast<void*>(this));
  if (receiver_thread_ == nullptr) {
    QMMF_ERROR("%s:%s unable to allocate thread", TAG, __func__);
    return -ENOMEM;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

void* TranscoderTrack::DeliverInput(void* arg) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  TranscoderTrack* track = reinterpret_cast<TranscoderTrack*>(arg);
  status_t ret = 0;
  while (1) {
    TranscodeBuffer buffer;
    ret = track->getInputBufferSource()->DequeTranscodeBuffer(&buffer);
    uint32_t csd_data_size = 0;
    FileSourceSampleInfo sSampleInfo;
    FileSourceMediaStatus eMediaStatus = FILE_SOURCE_DATA_ERROR;
    memset(&sSampleInfo, 0, sizeof(FileSourceSampleInfo));
    (track->m_sTrackInfo_).sVideo.sSampleBuf.ulLen =
        (track->m_sTrackInfo_).sVideo.sSampleBuf.ulMaxLen;
    if (track->isFirstFrame_) {
      uint32_t status = track->m_pDemux_->m_pFileSource->GetFormatBlock(
          (track->m_sTrackInfo_).sVideo.ulTkId,
          nullptr,
          &csd_data_size);

      QMMF_INFO("%s:%s Video CSD data Size = %u", TAG, __func__, csd_data_size);
      assert(FILE_SOURCE_SUCCESS == status);

      status = track->m_pDemux_->m_pFileSource->GetFormatBlock(
          (track->m_sTrackInfo_).sVideo.ulTkId,
          reinterpret_cast<uint8_t*>(buffer.GetVaddr()),
          &csd_data_size);

      assert(FILE_SOURCE_SUCCESS == status);
      track->isFirstFrame_ = false;
    }

    eMediaStatus = track->m_pDemux_->GetNextMediaSample(
        (track->m_sTrackInfo_).sVideo.ulTkId,
        reinterpret_cast<uint8_t*>(buffer.GetVaddr()) + csd_data_size,
        &((track->m_sTrackInfo_).sVideo.sSampleBuf.ulLen),
        sSampleInfo);

    buffer.SetFilledSize((track->m_sTrackInfo_).sVideo.sSampleBuf.ulLen +
                         csd_data_size);
    // Multiplying the timestamp to convert it into nano seconds
    buffer.SetTimestamp(1000 * (sSampleInfo.startTime));
    buffer.SetFlag(0x0);
    buffer.SetOffset(0x0);

    if (FILE_SOURCE_DATA_END == eMediaStatus || track->IsInputPortStop()) {
      QMMF_INFO("%s:%s: File read completed", TAG, __func__);
      buffer.SetFilledSize(0);
      buffer.SetFlag(static_cast<uint32_t>(BufferFlags::kFlagEOS));
      buffer.SetOffset(0x0);

      track->isLastFrame_ = true;
      track->num_delivered_frames_++;

      QMMF_INFO("%s:%s  Video Ts[%llu] filled_size[%u] flags[0x%x]  fd[%d]"
                " frames_delivered[%d]", TAG, __func__,
                (buffer.GetTimestamp()) / 1000,
                buffer.GetFilledSize(),
                buffer.GetFlag(), buffer.GetFd(),
                track->num_delivered_frames_);

      ret = track->getInputBufferSource()->QueueTranscodeBuffer(buffer);
      assert(ret == 0);

      if (FILE_SOURCE_DATA_END == eMediaStatus && !track->IsInputPortStop()) {
        track->stop_thread_ = new thread(TranscoderTrack::StopTransCoding,
                                         reinterpret_cast<void*>(track));
        if (track->stop_thread_ == nullptr) {
          QMMF_ERROR("%s:%s unable to allocate thread", TAG, __func__);
          assert(0);
        }
      }
      break;
    }

    track->num_delivered_frames_++;

    QMMF_INFO("%s:%s  Video Ts[%llu] filled_size[%u] flags[0x%x]  fd[%d]"
              " frames_delivered[%d]", TAG, __func__,
              (buffer.GetTimestamp()) / 1000,
              buffer.GetFilledSize(),
              buffer.GetFlag(), buffer.GetFd(),
              track->num_delivered_frames_);

    ret = track->getInputBufferSource()->QueueTranscodeBuffer(buffer);
    assert(ret == 0);
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return nullptr;
}

void* TranscoderTrack::ReceiveOutput(void* arg) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  TranscoderTrack* track = reinterpret_cast<TranscoderTrack*>(arg);
  FILE* file = fopen(track->params_.output_file.c_str(), "w");
  while (1) {
    TranscodeBuffer buffer;
    ret = track->getOutputBufferSource()->DequeTranscodeBuffer(&buffer);
    assert(ret == 0);
    track->num_received_frames_++;

    QMMF_INFO("%s:%s Video Ts[%llu] filled_size[%u] flags[0x%x]  fd[%d]"
              " frames_received[%d]", TAG, __func__,
              buffer.GetTimestamp(), buffer.GetFilledSize(),
              buffer.GetFlag(), buffer.GetFd(), track->num_received_frames_);

    fwrite(reinterpret_cast<void*>
                           (reinterpret_cast<uint32_t*>(buffer.GetVaddr()) +
                            buffer.GetOffset()),
           1, buffer.GetFilledSize(), file);

    ret = track->getOutputBufferSource()->QueueTranscodeBuffer(buffer);
    assert(ret == 0);

    if (buffer.GetFlag() & static_cast<uint32_t>(BufferFlags::kFlagEOS)) {
      fclose(file);
      break;
    }
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return nullptr;
}

void* TranscoderTrack::StopTransCoding(void* arg) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  TranscoderTrack* track = reinterpret_cast<TranscoderTrack*>(arg);
  ret = track->Stop();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Automatic stop failed", TAG, __func__);
    assert(0);
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return nullptr;
}

status_t TranscoderTrack::Stop() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  if (IsInputPortStop()) {
    QMMF_WARN("%s:%s Stop is already done", TAG, __func__);
    QMMF_INFO("%s:%s Exit", TAG, __func__);
    return ret;
  }

  {
    lock_guard<mutex> lg(input_stop_mutex_);
    input_stop_ = true;
  }

  deliver_thread_->join();
  receiver_thread_->join();

  delete deliver_thread_;
  deliver_thread_ = nullptr;
  delete receiver_thread_;
  receiver_thread_ = nullptr;

  ret = transcoder_source_->StopCodec();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to stop on core side", TAG, __func__);
    return ret;
  }

  ret = transcoder_sink_->StopCodec();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to stop on pipe side", TAG, __func__);
    return ret;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderTrack::Delete() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  if (stop_thread_ != nullptr) {
    stop_thread_->join();
    delete stop_thread_;
    stop_thread_ = nullptr;
  }

  ret = transcoder_source_->DeleteCodec();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to delete on core side", TAG, __func__);
    goto release_resources;
  }

  ret = transcoder_sink_->DeleteCodec();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to delete on pipe side", TAG, __func__);
    goto release_resources;
  }

  ret = transcoder_pipe_->RemovePipe();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to remove Pipe", TAG, __func__);
    goto release_resources;
  }

  // TODO: Release these resources
  // MM_FREE(m_sTrackInfo_.sAudio.sCSD.pucData);
  // MM_FREE(m_sTrackInfo_.sVideo.sCSD.pucData);

  if (m_pIStreamPort_ != nullptr) {
    delete m_pIStreamPort_;
    m_pIStreamPort_ = nullptr;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;

release_resources:
  ReleaseResources();
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

/* This function will fill the following parameters
1. Sink Codec Params structure or Core Codec Params structure
2. track_file_name, input_file_name and output_file_name
3. Transcode Type, Core Codec Type, Sink Codec Type
4. What ever has been chossen for filling in 1, the opposite one
will be filled by demuxer
*/
status_t TranscoderTrack::ParseFile(const string& fileName) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  if (fileName.empty()) {
    QMMF_ERROR("%s:%s Invalid Parameters", TAG, __func__);
    return -1;
  }

  FILE* fp;
  const int MAX_LINE = 128;
  char line[MAX_LINE];
  char value[50];
  char key[25];
  uint32_t id = 0;
  bool avc = false;
  bool readCoreParams = false;
  bool readSinkParams = false;
  bool readTrackParams = false;

  if (!(fp = fopen(fileName.c_str(), "r"))) {
    QMMF_ERROR("%s:%s failed to open config file: %s",
               TAG, __func__, fileName.c_str());
    return -1;
  }

  while (fgets(line, MAX_LINE - 1, fp)) {
    if ((line[0] == '\n') || (line[0] == '/') || line[0] == ' ')
      continue;
    memset(value, 0x0, sizeof(value));
    memset(key, 0x0, sizeof(key));
    int len = strlen(line);
    int i, j = 0;

    // This assumes that the line is a comment
    if (!strcspn(line, "#")) {
      id++;
      continue;
    }

    // signifies start or end of a params
    if (!readTrackParams && strstr(line, "TRACK-PARAMS-START")) {
      readTrackParams = true;
      continue;
    }
    if (!readCoreParams && strstr(line, "SOURCE-PARAMS-START")) {
      readCoreParams = true;
      continue;
    }
    if (!readSinkParams && strstr(line, "SINK-PARAMS-START")) {
      readSinkParams = true;
      continue;
    }
    if (readTrackParams && strstr(line, "TRACK-PARAMS-END")) {
      readTrackParams = false;
      continue;
    }
    if (readCoreParams && strstr(line, "SOURCE-PARAMS-END")) {
      readCoreParams = false;
      continue;
    }
    if (readSinkParams && strstr(line, "SINK-PARAMS-END")) {
      readSinkParams = false;
      continue;
    }

    int pos = strcspn(line, ":");
    for (i = 0; i < pos; i++) {
      if (line[i] != ' ') {
        key[j] = line[i];
        j++;
      }
    }

    key[j] = '\0';
    j = 0;
    for (i = pos + 1; i < len - 1; i++) {
      if (line[i] != ' ') {
        value[j] = line[i];
        j++;
      }
    }
    value[j] = '\0';

    if (readTrackParams && !readCoreParams && !readSinkParams)
      goto READ_TRACK;
    if (!readTrackParams && readCoreParams && !readSinkParams)
      goto READ_SOURCE;
    if (!readTrackParams && !readCoreParams && readSinkParams)
      goto READ_SINK;
    else
      goto READ_FAILED;

  READ_TRACK:
    if (!strncmp("InputFile", key, strlen("InputFile"))) {
      params_.input_file = value;
    } else if (!strncmp("OutputFile", key, strlen("OutputFile"))) {
      params_.output_file = value;
    } else if (!strncmp("TranscodeType", key, strlen("TranscodeType"))) {
      if (!strncmp("VideoDecodeVideoEncode", value,
                   strlen("VideoDecodeVideoEncode"))) {
        params_.track_type = TranscodeType::kVideoDecodeVideoEncode;
        params_.source_codec_type = CodecType::kVideoDecoder;
        params_.sink_codec_type = CodecType::kVideoEncoder;
        params_.sink_params.video_enc_param.camera_id = kNotRequired;
        params_.sink_params.video_enc_param.low_power_mode = kNotRequired;
        if (!((params_).input_file.empty())) {
          m_pIStreamPort_ = new CMM_MediaSourcePort(
              const_cast<char*>((params_).input_file.c_str()));
          if (m_pIStreamPort_ == nullptr) {
            QMMF_ERROR("%s:%s Failed to allocate demuxer", TAG, __func__);
            goto READ_FAILED;
          }
          ret = FillParams();
          if (ret != 0) {
            QMMF_ERROR("%s:%s Failed to get videodecoder params from demuxer",
                       TAG, __func__);
            goto READ_FAILED;
          }
        } else {
          QMMF_ERROR("%s:%s Input file name not found", TAG, __func__);
          goto READ_FAILED;
        }
      } else {
        QMMF_ERROR("%s:%s Unknown TranscodeType(%s)", TAG, __func__, value);
        goto READ_FAILED;
      }
    } else if (!strncmp("EnableVQzip", key, strlen("EnableVQzip"))) {
      params_.enable_vqzip = atoi(value);
      if (params_.track_type == TranscodeType::kVideoDecodeVideoEncode &&
          params_.enable_vqzip) {
        QMMF_INFO("%s:%s Opening VQZipInfoExtractor", TAG, __func__);
        VQZipInfoExtractor* vqzip_extractor =
            new VQZipInfoExtractor(params_.source_params.video_dec_param,
                                   m_sTrackInfo_, m_pDemux_);
        ret = vqzip_extractor->Init();
        if (ret != 0) {
          QMMF_ERROR("%s:%s VQZipInfoExtractor Init Failed", TAG, __func__);
          delete vqzip_extractor;
          vqzip_extractor = nullptr;
          goto READ_FAILED;
        }
        params_.sink_params.video_enc_param.vqzip_params.format =
            VideoFormat::kAVC;
        ret = vqzip_extractor->ExtractVQZipInfo(
            &params_.sink_params.video_enc_param.vqzip_params);
        if (ret != 0) {
          QMMF_ERROR("%s:%s VQZipInfo Extraction Failed", TAG, __func__);
          delete vqzip_extractor;
          vqzip_extractor = nullptr;
          goto READ_FAILED;
        }
        ret = vqzip_extractor->DeInit();
         if (ret != 0) {
          QMMF_ERROR("%s:%s VQZipInfoExtractor DeInit Failed", TAG, __func__);
          delete vqzip_extractor;
          vqzip_extractor = nullptr;
          goto READ_FAILED;
        }

        delete vqzip_extractor;
        vqzip_extractor = nullptr;

        params_.source_params.video_dec_param.enable_vqzip_extradata = true;
        params_.sink_params.video_enc_param.do_vqzip = true;

      } else if (params_.track_type != TranscodeType::kVideoDecodeVideoEncode &&
                 params_.enable_vqzip) {
        QMMF_ERROR("%s:%s Wrong combination of TransCodeType and VQZip",
                   TAG, __func__);
        goto READ_FAILED;
      }
    } else {
      QMMF_ERROR("%s:%s Unknown Key %s found", TAG, __func__, key);
      goto READ_FAILED;
    }
    continue;

  READ_SOURCE:
    if (!strncmp("FPS", key, strlen("FPS"))) {
      params_.source_params.video_dec_param.frame_rate = atoi(value);
    } else if (!strncmp("Enable_Downscalar", key,
                        strlen("Enable_Downscalar"))) {
      params_.source_params.video_dec_param.enable_downscalar = atoi(value);
    } else if (!strncmp("Downscale_Width", key, strlen("Downscale_Width"))) {
      params_.source_params.video_dec_param.output_width = atoi(value);
    } else if (!strncmp("Downscale_Height", key, strlen("Downscale_Height"))) {
      params_.source_params.video_dec_param.output_height = atoi(value);
    } else {
      QMMF_ERROR("%s:%s Unknown Key %s found", TAG, __func__, key);
      goto READ_FAILED;
    }
    continue;

  READ_SINK:
    if (!strncmp("Width", key, strlen("Width"))) {
      params_.sink_params.video_enc_param.width = atoi(value);
    } else if (!strncmp("Height", key, strlen("Height"))) {
      params_.sink_params.video_enc_param.height = atoi(value);
    } else if (!strncmp("FPS", key, strlen("FPS"))) {
      params_.sink_params.video_enc_param.frame_rate = atoi(value);
    } else if (!strncmp("Codec", key, strlen("Codec"))) {
      if (!strncmp("AVC", value, strlen("AVC"))) {
        avc = true;
        params_.sink_params.video_enc_param.format_type = VideoFormat::kAVC;
        params_.sink_params.video_enc_param.codec_param.avc
            .prepend_sps_pps_to_idr = true;
        params_.sink_params.video_enc_param.codec_param.avc
            .insert_aud_delimiter = true;
        params_.sink_params.video_enc_param.codec_param.avc.sar_enabled = false;
        params_.sink_params.video_enc_param.codec_param.avc.sar_width = 0;
        params_.sink_params.video_enc_param.codec_param.avc.sar_height = 0;
      } else if (!strncmp("HEVC", value, strlen("HEVC"))) {
        params_.sink_params.video_enc_param.format_type = VideoFormat::kHEVC;
        params_.sink_params.video_enc_param.codec_param.hevc
            .prepend_sps_pps_to_idr = true;
        params_.sink_params.video_enc_param.codec_param.hevc
            .sar_enabled = false;
        params_.sink_params.video_enc_param.codec_param.hevc.sar_width = 0;
        params_.sink_params.video_enc_param.codec_param.hevc.sar_height = 0;
      } else {
        QMMF_ERROR("%s:%s Unknown Video CodecType(%s)", TAG, __func__, value);
        goto READ_FAILED;
      }
    } else if (!strncmp("IFR", key, strlen("IFR"))) {
      if (avc)
        params_.sink_params.video_enc_param.codec_param.avc
            .idr_interval = atoi(value);
      else
        params_.sink_params.video_enc_param.codec_param.hevc
            .idr_interval = atoi(value);
    } else if (!strncmp("Bitrate", key, strlen("Bitrate"))) {
      if (avc)
        params_.sink_params.video_enc_param.codec_param.avc.bitrate =
            atoi(value);
      else
        params_.sink_params.video_enc_param.codec_param.hevc.bitrate =
            atoi(value);
    } else if (!strncmp("Profile", key, strlen("Profile"))) {
      if (avc) {
        if (!strncmp("BaseProfile", value, strlen("BaseProfile")))
          params_.sink_params.video_enc_param.codec_param.avc.profile =
              AVCProfileType::kBaseline;
        else if (!strncmp("MainProfile", value, strlen("MainProfile")))
          params_.sink_params.video_enc_param.codec_param.avc.profile =
              AVCProfileType::kMain;
        else if (!strncmp("HighProfile", value, strlen("HighProfile")))
          params_.sink_params.video_enc_param.codec_param.avc.profile =
              AVCProfileType::kHigh;
        else {
          QMMF_ERROR("%s:%s Unknown AVC Profile(%s)", TAG, __func__, value);
          goto READ_FAILED;
        }
      } else {
        if (!strncmp("MainProfile", value, strlen("MainProfile")))
          params_.sink_params.video_enc_param.codec_param.hevc.profile =
              HEVCProfileType::kMain;
        else {
          QMMF_ERROR("%s:%s Unknown HEVC Profile(%s)", TAG, __func__, value);
          goto READ_FAILED;
        }
      }
    } else if (!strncmp("Level", key, strlen("Level"))) {
      if (avc) {
        if (!strncmp("Level-1", value, strlen("Level-1")))
          params_.sink_params.video_enc_param.codec_param.avc.level =
              AVCLevelType::kLevel1;
        else if (!strncmp("Level-1_3", value, strlen("Level-1_3")))
          params_.sink_params.video_enc_param.codec_param.avc.level =
              AVCLevelType::kLevel1_3;
        else if (!strncmp("Level-2", value, strlen("Level-2")))
          params_.sink_params.video_enc_param.codec_param.avc.level =
              AVCLevelType::kLevel2;
        else if (!strncmp("Level-2_1", value, strlen("Level-2_1")))
          params_.sink_params.video_enc_param.codec_param.avc.level =
              AVCLevelType::kLevel2_1;
        else if (!strncmp("Level-2_2", value, strlen("Level-2_2")))
          params_.sink_params.video_enc_param.codec_param.avc.level =
              AVCLevelType::kLevel2_2;
        else if (!strncmp("Level-3", value, strlen("Level-3")))
          params_.sink_params.video_enc_param.codec_param.avc.level =
              AVCLevelType::kLevel3;
        else if (!strncmp("Level-3_1", value, strlen("Level-3_1")))
          params_.sink_params.video_enc_param.codec_param.avc.level =
              AVCLevelType::kLevel3_1;
        else if (!strncmp("Level-3_2", value, strlen("Level-3_2")))
          params_.sink_params.video_enc_param.codec_param.avc.level =
              AVCLevelType::kLevel3_2;
        else if (!strncmp("Level-4", value, strlen("Level-4")))
          params_.sink_params.video_enc_param.codec_param.avc.level =
              AVCLevelType::kLevel4;
        else if (!strncmp("Level-4_1", value, strlen("Level-4_2")))
          params_.sink_params.video_enc_param.codec_param.avc.level =
              AVCLevelType::kLevel4_1;
        else if (!strncmp("Level-5", value, strlen("Level-5")))
          params_.sink_params.video_enc_param.codec_param.avc.level =
              AVCLevelType::kLevel5;
        else if (!strncmp("Level-5_1", value, strlen("Level-5_1")))
          params_.sink_params.video_enc_param.codec_param.avc.level =
              AVCLevelType::kLevel5_1;
        else if (!strncmp("Level-5_2", value, strlen("Level-5_2")))
          params_.sink_params.video_enc_param.codec_param.avc.level =
              AVCLevelType::kLevel5_2;
        else {
          QMMF_ERROR("%s:%s Unknown AVC Level(%s)", TAG, __func__, value);
          goto READ_FAILED;
        }
      } else {
        if (!strncmp("Level-3", value, strlen("Level-3")))
          params_.sink_params.video_enc_param.codec_param.hevc.level =
              HEVCLevelType::kLevel3;
        else if (!strncmp("Level-4", value, strlen("Level-4")))
          params_.sink_params.video_enc_param.codec_param.hevc.level =
              HEVCLevelType::kLevel4;
        else if (!strncmp("Level-5", value, strlen("Level-5")))
          params_.sink_params.video_enc_param.codec_param.hevc.level =
              HEVCLevelType::kLevel5;
        else if (!strncmp("Level-5_1", value, strlen("Level-5_1")))
          params_.sink_params.video_enc_param.codec_param.hevc.level =
              HEVCLevelType::kLevel5_1;
        else if (!strncmp("Level-5_2", value, strlen("Level-5_2")))
          params_.sink_params.video_enc_param.codec_param.hevc.level =
              HEVCLevelType::kLevel5_2;
        else {
          QMMF_ERROR("%s:%s Unknown HEVC Level(%s)", TAG, __func__, value);
          goto READ_FAILED;
        }
      }
    } else if (!strncmp("RateControl", key, strlen("RateControl"))) {
      if (avc) {
        if (!strncmp("RC_OFF", value, strlen("RC_OFF")))
          params_.sink_params.video_enc_param.codec_param.avc
              .ratecontrol_type = VideoRateControlType::kDisable;
        else if (!strncmp("VBR_VFR", value, strlen("VBR_VFR")))
          params_.sink_params.video_enc_param.codec_param.avc
              .ratecontrol_type = VideoRateControlType::kVariableSkipFrames;
        else if (!strncmp("VBR_CFR", value, strlen("VBR_CFR")))
          params_.sink_params.video_enc_param.codec_param.avc
              .ratecontrol_type = VideoRateControlType::kVariable;
        else if (!strncmp("CBR_VFR", value, strlen("CBR_VFR")))
          params_.sink_params.video_enc_param.codec_param.avc
              .ratecontrol_type = VideoRateControlType::kConstantSkipFrames;
        else if (!strncmp("CBR_CFR", value, strlen("CBR_CFR")))
          params_.sink_params.video_enc_param.codec_param.avc
              .ratecontrol_type = VideoRateControlType::kConstant;
        else if (!strncmp("MBR_CFR", value, strlen("MBR_CFR")))
          params_.sink_params.video_enc_param.codec_param.avc
              .ratecontrol_type = VideoRateControlType::kMaxBitrate;
        else if (!strncmp("MBR_VFR", value, strlen("MBR_VFR")))
          params_.sink_params.video_enc_param.codec_param.avc
              .ratecontrol_type = VideoRateControlType::kMaxBitrateSkipFrames;
        else {
          QMMF_ERROR("%s:%s Unknown RC Mode(%s)", TAG, __func__, value);
          goto READ_FAILED;
        }
      } else {
        if (!strncmp("RC_OFF", value, strlen("RC_OFF")))
          params_.sink_params.video_enc_param.codec_param.hevc
              .ratecontrol_type = VideoRateControlType::kDisable;
        else if (!strncmp("VBR_VFR", value, strlen("VBR_VFR")))
          params_.sink_params.video_enc_param.codec_param.hevc
              .ratecontrol_type = VideoRateControlType::kVariableSkipFrames;
        else if (!strncmp("VBR_CFR", value, strlen("VBR_CFR")))
          params_.sink_params.video_enc_param.codec_param.hevc
              .ratecontrol_type = VideoRateControlType::kVariable;
        else if (!strncmp("CBR_VFR", value, strlen("CBR_VFR")))
          params_.sink_params.video_enc_param.codec_param.hevc
              .ratecontrol_type = VideoRateControlType::kConstantSkipFrames;
        else if (!strncmp("CBR_CFR", value, strlen("CBR_CFR")))
          params_.sink_params.video_enc_param.codec_param.hevc
              .ratecontrol_type = VideoRateControlType::kConstant;
        else if (!strncmp("MBR_CFR", value, strlen("MBR_CFR")))
          params_.sink_params.video_enc_param.codec_param.hevc
              .ratecontrol_type = VideoRateControlType::kMaxBitrate;
        else if (!strncmp("MBR_VFR", value, strlen("MBR_VFR")))
          params_.sink_params.video_enc_param.codec_param.hevc
              .ratecontrol_type = VideoRateControlType::kMaxBitrateSkipFrames;
        else {
          QMMF_ERROR("%s:%s Unknown RC Mode(%s)", TAG, __func__, value);
          goto READ_FAILED;
        }
      }
    } else if (!strncmp("InitQpI", key, strlen("InitQpI"))) {
      if (avc) {
        params_.sink_params.video_enc_param.codec_param.avc.qp_params
            .init_qp.init_IQP = atoi(value);
        params_.sink_params.video_enc_param.codec_param.avc.qp_params
            .enable_init_qp = true;
        params_.sink_params.video_enc_param.codec_param.avc.qp_params
            .init_qp.init_QP_mode = 0x7;
      } else {
        params_.sink_params.video_enc_param.codec_param.hevc.qp_params
            .init_qp.init_IQP = atoi(value);
        params_.sink_params.video_enc_param.codec_param.hevc.qp_params
            .enable_init_qp = true;
        params_.sink_params.video_enc_param.codec_param.hevc.qp_params
            .init_qp.init_QP_mode = 0x7;
      }
    } else if (!strncmp("InitQpP", key, strlen("InitQpP"))) {
      if (avc)
        params_.sink_params.video_enc_param.codec_param.avc.qp_params
            .init_qp.init_PQP = atoi(value);
      else
        params_.sink_params.video_enc_param.codec_param.hevc.qp_params
            .init_qp.init_PQP = atoi(value);
    } else if (!strncmp("InitQpB", key, strlen("InitQpB"))) {
      if (avc)
        params_.sink_params.video_enc_param.codec_param.avc.qp_params
            .init_qp.init_BQP = atoi(value);
      else
        params_.sink_params.video_enc_param.codec_param.hevc.qp_params
            .init_qp.init_BQP = atoi(value);
    } else if (!strncmp("MinQp", key, strlen("MinQp"))) {
      if (avc) {
        params_.sink_params.video_enc_param.codec_param.avc.qp_params
            .qp_range.min_QP = atoi(value);
        params_.sink_params.video_enc_param.codec_param.avc.qp_params
            .enable_qp_range = true;
      } else {
        params_.sink_params.video_enc_param.codec_param.hevc.qp_params
            .qp_range.min_QP = atoi(value);
        params_.sink_params.video_enc_param.codec_param.hevc.qp_params
            .enable_qp_range = true;
      }
    } else if (!strncmp("MaxQp", key, strlen("MaxQp"))) {
      if (avc)
        params_.sink_params.video_enc_param.codec_param.avc.qp_params
            .qp_range.max_QP = atoi(value);
      else
        params_.sink_params.video_enc_param.codec_param.hevc.qp_params
            .qp_range.max_QP = atoi(value);
    } else if (!strncmp("IPBQPRangeMin_IQP", key,
                        strlen("IPBQPRangeMin_IQP"))) {
      if (avc) {
        params_.sink_params.video_enc_param.codec_param.avc.qp_params
            .qp_IBP_range.min_IQP = atoi(value);
        params_.sink_params.video_enc_param.codec_param.avc.qp_params
            .enable_qp_IBP_range = true;
      } else {
        params_.sink_params.video_enc_param.codec_param.hevc.qp_params
            .qp_IBP_range.min_IQP = atoi(value);
        params_.sink_params.video_enc_param.codec_param.hevc.qp_params
            .enable_qp_IBP_range = true;
      }
    } else if (!strncmp("IPBQPRangeMax_IQP", key,
                        strlen("IPBQPRangeMax_IQP"))) {
      if (avc)
        params_.sink_params.video_enc_param.codec_param.avc.qp_params
            .qp_IBP_range.max_IQP = atoi(value);
      else
        params_.sink_params.video_enc_param.codec_param.hevc.qp_params
            .qp_IBP_range.max_IQP = atoi(value);
    } else if (!strncmp("IPBQPRangeMin_PQP", key,
                        strlen("IPBQPRangeMin_PQP"))) {
      if (avc)
        params_.sink_params.video_enc_param.codec_param.avc.qp_params
            .qp_IBP_range.min_PQP = atoi(value);
      else
        params_.sink_params.video_enc_param.codec_param.hevc.qp_params
            .qp_IBP_range.min_PQP = atoi(value);
    } else if (!strncmp("IPBQPRangeMax_PQP", key,
                        strlen("IPBQPRangeMax_PQP"))) {
      if (avc)
        params_.sink_params.video_enc_param.codec_param.avc.qp_params
            .qp_IBP_range.max_PQP = atoi(value);
      else
        params_.sink_params.video_enc_param.codec_param.hevc.qp_params
            .qp_IBP_range.max_PQP = atoi(value);
    } else if (!strncmp("IPBQPRangeMin_BQP", key,
                        strlen("IPBQPRangeMin_BQP"))) {
      if (avc)
        params_.sink_params.video_enc_param.codec_param.avc.qp_params
            .qp_IBP_range.min_BQP = atoi(value);
      else
        params_.sink_params.video_enc_param.codec_param.hevc.qp_params
            .qp_IBP_range.min_BQP = atoi(value);
    } else if (!strncmp("IPBQPRangeMax_BQP", key,
                        strlen("IPBQPRangeMax_BQP"))) {
      if (avc)
        params_.sink_params.video_enc_param.codec_param.avc.qp_params
            .qp_IBP_range.max_BQP = atoi(value);
      else
        params_.sink_params.video_enc_param.codec_param.hevc.qp_params
            .qp_IBP_range.max_BQP = atoi(value);
    } else if (!strncmp("Ltr_Count", key, strlen("Ltr_Count"))) {
      if (avc)
        params_.sink_params.video_enc_param.codec_param.avc.ltr_count =
            atoi(value);
      else
        params_.sink_params.video_enc_param.codec_param.hevc.ltr_count =
            atoi(value);
    } else if (!strncmp("Hier_Layer", key, strlen("Hier_Layer"))) {
      if (avc)
        params_.sink_params.video_enc_param.codec_param.avc.hier_layer =
            atoi(value);
      else
        params_.sink_params.video_enc_param.codec_param.hevc.hier_layer =
            atoi(value);
    } else {
      QMMF_ERROR("%s:%s Unknown Key %s found", TAG, __func__, key);
      goto READ_FAILED;
    }
    continue;
  }

  if (params_.enable_vqzip) {
    if ((params_.sink_params.video_enc_param.codec_param.avc.profile !=
         params_.sink_params.video_enc_param.vqzip_params.avc_vqzip_info
         .profile) ||
        (params_.sink_params.video_enc_param.codec_param.avc.level !=
         params_.sink_params.video_enc_param.vqzip_params.avc_vqzip_info
         .level)) {
      QMMF_WARN("%s:%s Using Profile and Level Values as the ones from VQZipInfoExtractor",
                TAG, __func__);
      params_.sink_params.video_enc_param.codec_param.avc.profile =
          params_.sink_params.video_enc_param.vqzip_params.avc_vqzip_info.profile;
      params_.sink_params.video_enc_param.codec_param.avc.level =
          params_.sink_params.video_enc_param.vqzip_params.avc_vqzip_info.level;
    }

    if (params_.sink_params.video_enc_param.codec_param.avc.ratecontrol_type !=
        VideoRateControlType::kDisable) {
      QMMF_WARN("%s:%s Disabling the RC for VQZIP", TAG, __func__);
      params_.sink_params.video_enc_param.codec_param.avc.ratecontrol_type =
          VideoRateControlType::kDisable;
    }
  }

  // Print all the parameters
  QMMF_INFO("%s:%s TransCodeParams[%s]",
            TAG, __func__, params_.ToString().c_str());

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return 0;

READ_FAILED:
  fclose(fp);
  ReleaseResources();
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return -1;
}

};  // namespace transcode
};  // namespace qmmf
