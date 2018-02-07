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

#include <utils/Log.h>
#include <utils/Errors.h>
#include "qmmf_mux_interface.h"

#define  LIMIT_NEAR_THRESHOLD_HIGH      40
#define  LIMIT_IMMINENT_THRESHOLD_HIGH  12
#define  OPTIMAL_STREAM_BUF_SIZE_FACTOR 2
#define  FRAGMENT_TABLE_SIZE_FACTOR     110
#define  DEFAULT_SAMPLES_TABLE_SIZE     16000
#define  DEFAULT_CHUNKS_TABLE_SIZE      600
#define  MAX_HEADER_SIZE                1024
#define  MAX_FOOTER_SIZE                1024
#define  MMI_MAX_SAMPLES_PER_STORE      16000
#define  MMI_MAX_STREAM_DURATION        (3600 * 3)
#define  STSC_ALGO_RESET_RATE           5
#define  AVC_L1b_PROFILE_COM            0x90
#define  VOCODER_SAMPLING_RATE          8000
#define  DEAFULT_INTERLACE_PERIOD_MS    3000
#define  MIN_AUDIO_MUX_BUFFER_SIZE      24000
#define  QCP_FILE_HEADER_SIZE           162
#define  VOCODER_PACKET_RATE            50
#define  AAC_SAMPLES_PER_FRAME          1024

#define  OPTIMAL_CHUNK_DURATION(bitrate) (bitrate >  8000000 ? 1  :\
                                       bitrate >  4000000 ? 2  :\
                                       bitrate >  2000000 ? 3  :\
                                       3)
#define  DESIRED_INTERLACE_RATE(bitrate) (OPTIMAL_CHUNK_DURATION(bitrate)*1000)
#define  GET_VIDEO_CHUNKS_TABLE_SIZE(space_limit_threshold)               \
         (                                                                \
             MAX (DEFAULT_CHUNKS_TABLE_SIZE,                              \
                 ((space_limit_threshold) * 1000)                         \
                  / DESIRED_INTERLACE_RATE  + 1)                          \
         )
#define  OPTIMAL_CHUNK_SIZE(bitrate)    ((bitrate + 4) / 8) *           \
                                         OPTIMAL_CHUNK_DURATION(bitrate)
#define  GET_SAMPLE_DELTA(x,y,z)                                         \
         (                                                               \
                 (( (uint64_t)(x) - (uint64_t)(y)) * (uint64_t)(z)       \
                  + (1000000ULL >> 1)) / 1000000ULL                      \
         )

namespace qmmf {

namespace muxinterface {

using namespace android;

const char MuxInterface::kMP4MajorBrand[] = "isom";
const char *MuxInterface::kMP4CompatBrands [] = {"mp41", "isom"};
const char MuxInterface::k3GPMajorBrand[] = "3gp5";
const char *MuxInterface::k3GPCompatBrands[] = {"3gp5", "isom" };

MuxInterface::MuxInterface(MUX_brand_type brand, const char *file_name) :
    mux_(NULL),
    release_cb_(nullptr),
    brand_type_(brand),
    file_name_(file_name),
    video_start_time_(-1),
    first_timestamp_(-1),
    video_previous_time_(-1),
    video_track_id_(0),
    audio_track_id_(0),
    close_completed_(false),
    flush_completed_(false),
    close_status_(NO_ERROR),
    flush_status_(NO_ERROR) {
  switch(brand_type_) {
    case MUX_BRAND_MP4:
    case MUX_BRAND_3GP:
      format_type_ = MUX_FMT_MP4;
      break;
    case MUX_BRAND_MP2TS:
      format_type_ = MUX_FMT_MP2;
      break;
    case MUX_BRAND_3G2:
    case MUX_BRAND_AMC:
    case MUX_BRAND_SKM:
    case MUX_BRAND_K3G:
    case MUX_BRAND_ADTS:
    case MUX_BRAND_EVRC:
    case MUX_BRAND_ADIF:
    case MUX_BRAND_QCELP13K_FIXED_FULL_RATE:
    case MUX_BRAND_QCELP13K_FIXED_HALF_RATE:
    case MUX_BRAND_QCELP13K_VAR_FULL_RATE:
    case MUX_BRAND_QCELP13K_VAR_HALF_RATE:
    case MUX_BRAND_AMRNB:
    case MUX_BRAND_AMRWB:
    case MUX_BRAND_PCMLINEAR:
    case MUX_BRAND_PCMALaw:
    case MUX_BRAND_PCMMULaw:
    case MUX_BRAND_PCMGSMFR:
    case MUX_BRAND_G723:
    case MUX_BRAND_G729:
    case MUX_BRAND_FRAG_3G2:
    case MUX_BRAND_INVALID:
    default:
      ALOGE("%s: Unsupported brand type: %d\n", __func__, brand_type_);
  }

  pthread_mutex_init(&stop_lock_, NULL);
  pthread_cond_init(&stop_cond_, NULL);
  pthread_mutex_init(&flush_lock_, NULL);
  pthread_cond_init(&flush_cond_, NULL);
}

MuxInterface::~MuxInterface() {
  if (NULL != mux_) {
    delete mux_;
    mux_ = NULL;
  }

  pthread_mutex_destroy(&stop_lock_);
  pthread_cond_destroy(&stop_cond_);
  pthread_mutex_destroy(&flush_lock_);
  pthread_cond_destroy(&flush_cond_);
}

int32_t MuxInterface::Init(const qmmf_muxer_init & init_params) {
  int32_t ret = NO_ERROR;

  if (NULL != mux_) {
    ALOGE("%s: Muxer already initialized!\n", __func__);
    return INVALID_OPERATION;
  }

  memset(&create_params_, 0, sizeof(create_params_));
  memset(&streams_, 0, sizeof(streams_));
  memset(&mux_file_handle_, 0, sizeof(mux_file_handle_));

  if (init_params.audio_stream_present) {
    create_params_.num_streams++;
  }
  if (init_params.video_stream_present) {
    create_params_.num_streams++;
  }
  create_params_.streams = streams_;

  ret = SetMainMuxerParams(create_params_, init_params);
  if (NO_ERROR != ret) {
    ALOGE("%s: Failed to set main muxer parameters!\n", __func__);
    return ret;
  }

  ret = SetAudioMuxerParams(create_params_, init_params);
  if (NO_ERROR != ret) {
    ALOGE("%s: Failed to set audio muxer parameters!\n", __func__);
    return ret;
  }

  ret = SetVideoMuxerParams(create_params_, init_params);
  if (NO_ERROR != ret) {
    ALOGE("%s: Failed to set video muxer parameters!\n", __func__);
    return ret;
  }

  mux_file_handle_.efs.method = MUX_METHOD_EFS;
  memset(mux_file_handle_.efs.filename, '\0',
         sizeof(mux_file_handle_.efs.filename));
  strncpy(mux_file_handle_.efs.filename, file_name_.string(),
          sizeof(mux_file_handle_.efs.filename)-1);

  mux_ = new FileMux(&create_params_, format_type_,
                     brand_type_, &mux_file_handle_,
                     FALSE, FileMuxCb, this);
  if(NULL == mux_) {
      ALOGE("%s: Muxer instantiation failed!", __func__);
      ret = NO_MEMORY;
  } else if (MUX_SUCCESS != mux_->MUX_get_Status()) {
    ALOGE("%s: Muxer status invalid!", __func__);
    ret = ConvertStatus(mux_->MUX_get_Status());
    delete mux_;
    mux_ = NULL;
  }

  release_cb_ = init_params.release_cb;

  return ret;
}

int32_t MuxInterface::Flush() {
  int32_t ret;
  if (NULL == mux_) {
    ALOGE("%s: Muxer not initialized!\n", __func__);
    return INVALID_OPERATION;
  }

  flush_completed_ = false;
  mux_->MUX_Flush();

  pthread_mutex_lock(&flush_lock_);
  while(!flush_completed_) {
    pthread_cond_wait(&flush_cond_, &flush_lock_);
  }

  ret = flush_status_;
  pthread_mutex_unlock(&flush_lock_);

  return ret;
}

int32_t MuxInterface::Stop() {
  int32_t ret;
  if (NULL == mux_) {
    ALOGE("%s: Muxer not initialized!\n", __func__);
    return INVALID_OPERATION;
  }

  ret = Flush();
  if (NO_ERROR != ret) {
    ALOGE("%s: Muxer flush failed!\n", __func__);
    return ret;
  }

  pthread_mutex_lock(&stop_lock_);
  close_completed_ = false;

  auto stat = mux_->MUX_end_Processing(this);
  if (MUX_SUCCESS != stat) {
    ret = ConvertStatus(stat);
    ALOGE("%s: Failed to stop processing: %d\n", __func__, ret);
    pthread_mutex_unlock(&stop_lock_);

    return ret;
  }

  while(!close_completed_) {
    pthread_cond_wait(&stop_cond_, &stop_lock_);
  }

  ret = close_status_;
  pthread_mutex_unlock(&stop_lock_);

  return ret;
}

int32_t MuxInterface::WriteAudioBuffer(uint32_t track_id, uint32_t session_id,
                                       BufferDescriptor &buffer,
                                       __attribute__((unused)) MetaData &meta_data,
                                       int32_t idx) {
  MUX_sample_info_type *sample_info;
  MUX_STATUS status = MUX_FAIL;
  qmmf_muxer_client_data *client_data;
  size_t length = buffer.size;
  uint64_t ts = buffer.timestamp;

  client_data = (qmmf_muxer_client_data *) malloc(sizeof(*client_data));
  if (NULL == client_data) {
    ALOGE("%s: Unable to allocate client data!", __func__);
    return NO_MEMORY;
  }
  memset(client_data, 0, sizeof(*client_data));

  sample_info = (MUX_sample_info_type *) malloc(sizeof(*sample_info));
  if (NULL == sample_info) {
    ALOGE("%s: Unable to allocate sample info!", __func__);
    if (NULL != client_data) {
      free(client_data);
    }
    return NO_MEMORY;
  }
  memset(sample_info, 0, sizeof(*sample_info));

  client_data->track_id = track_id;
  client_data->session_id = session_id;
  client_data->buffer = buffer;

  sample_info->time = (uint32)
                       GET_SAMPLE_DELTA(ts, first_timestamp_,
                                        streams_[idx].media_timescale);

  sample_info->delta = (uint32) streams_[idx].sample_delta;

  sample_info->is_time_valid = true;
  sample_info->size = length;

  uint8_t *data_ptr = reinterpret_cast<uint8_t *>(buffer.data);

  status = mux_->MUX_Process_Sample(idx, 1, sample_info,
                                    data_ptr, client_data);
  if (MUX_SUCCESS != status) {
    ALOGE("%s: Failed to process audio buffer: %d\n", __func__, status);
    free(sample_info);
    free(client_data);
    return ConvertStatus(status);
  }

  return ConvertStatus(status);
}

int32_t MuxInterface::WriteVideoBuffer(uint32_t track_id, uint32_t session_id,
                                       BufferDescriptor &buffer,
                                       MetaData &meta_data, int32_t idx) {
  MUX_STATUS status = MUX_FAIL;
  MUX_sample_info_type *sample_info;
  qmmf_muxer_client_data *client_data;
  bool first_frame = false;

  if (0 > video_start_time_) {
    video_start_time_ = buffer.timestamp;
    first_frame = true;
  }

  client_data = (qmmf_muxer_client_data *) malloc(sizeof(*client_data));
  if (NULL == client_data) {
    ALOGE("%s: Unable to allocate client data!", __func__);
    return NO_MEMORY;
  }
  memset(client_data, 0, sizeof(*client_data));
  client_data->buffer = buffer;
  client_data->track_id = track_id;
  client_data->session_id = session_id;

  if (first_frame) {
    status = mux_->MUX_write_header(idx, true, buffer.size,
                                    (uint8_t *)buffer.data, client_data);
    if (MUX_SUCCESS != status) {
      ALOGE("%s: Failed to writer header : %d\n", __func__, status);
      free(client_data);

    }
  } else {
    if (0 == video_start_time_) {
      video_start_time_ = buffer.timestamp;
      first_frame = true;
    }
    sample_info = (MUX_sample_info_type *) malloc(sizeof(*sample_info));
    if (NULL == sample_info) {
      ALOGE("%s: Unable to allocate sample info!", __func__);
      if (NULL != client_data) {
        free(client_data);
      }
      return NO_MEMORY;
    }
    memset(sample_info, 0, sizeof(*sample_info));

    sample_info->time = (uint32)
                         GET_SAMPLE_DELTA(buffer.timestamp,
                                          first_timestamp_,
                                          streams_[idx].media_timescale);

    if (first_frame) {
      sample_info->delta = (uint32)
                            GET_SAMPLE_DELTA(buffer.timestamp,
                                             first_timestamp_,
                                             streams_[idx].media_timescale);
    } else {
      //TODO: Handle negative timestamps in case of B-Frames
      sample_info->delta = (uint32)
                            GET_SAMPLE_DELTA(buffer.timestamp,
                                             video_previous_time_,
                                             streams_[idx].media_timescale);
    }

    video_previous_time_ = buffer.timestamp;

    sample_info->is_time_valid = true;
    sample_info->size = buffer.size;

    if (meta_data.video_frame_type_info &
        static_cast<uint32_t>(BufferFlags::kFlagIDRFrame)) {
      sample_info->sync = 1;
    } else  {
      sample_info->sync = 0;
    }

    status = mux_->MUX_Process_Sample(idx, 1,
                                      sample_info,
                                      (uint8_t *) buffer.data,
                                      client_data);

    if (MUX_SUCCESS != status) {
      ALOGE("%s: Failed to process buffer: %d\n", __func__, status);
      free(sample_info);
      free(client_data);
    }
  }

  return ConvertStatus(status);
}

int32_t MuxInterface::WriteBuffer(uint32_t track_id, uint32_t session_id,
                                  BufferDescriptor &buffer, MetaData &meta_data) {
  int32_t ret;
  int32_t idx;
  if (NULL == mux_) {
    ALOGE("%s: Muxer not initialized!\n", __func__);
    return INVALID_OPERATION;
  }

  if (close_completed_ || flush_completed_) {
    return DEAD_OBJECT;
  }

  if (video_track_id_ == track_id) {
    idx = VIDEO_STREAM_INDEX;
  } else if (audio_track_id_ == track_id) {
    idx = AUDIO_STREAM_INDEX;
  } else {
    ALOGE("%s: Unknown track_id: %d\n", __func__, track_id);
    return BAD_VALUE;
  }

  if (first_timestamp_ <= 0) {
      first_timestamp_ = buffer.timestamp;
  }

  if (AUDIO_STREAM_INDEX == idx) {
    ret = WriteAudioBuffer(track_id, session_id, buffer, meta_data, idx);
  } else {
    ret = WriteVideoBuffer(track_id, session_id, buffer, meta_data, idx);
  }

  return ret;
}

int32_t MuxInterface::ConvertStatus(MUX_STATUS status) {
  switch (status) {
    case MUX_SUCCESS:
      return NO_ERROR;
    case MUX_FAIL:
      return UNKNOWN_ERROR;
    case MUX_NOTAVAILABLE:
      return INVALID_OPERATION;
    case MUX_INVALID:
      return BAD_VALUE;
    case MUX_QUEUE_FULL:
      return BAD_INDEX;
    case MUX_DONE:
      return NO_ERROR;
    case MUX_SPACE_LIMIT_REACHED:
      return BAD_INDEX;
    case MUX_WRITE_FAILED:
    case MUX_OUTDATED:
    case MUX_SECURITY_FAULT:
      return UNKNOWN_ERROR;
    default:
      ALOGE("%s: Unknown status: %d\n", __func__, status);
      return BAD_VALUE;
  }

  return NO_ERROR;
}

int32_t MuxInterface::SetMainMuxerParams(MUX_create_params_type &params,
                                         const qmmf_muxer_init &init_params) {
  int32_t ret = NO_ERROR;

  params.drm_distribution_rights = 0;
  params.enable_fixupdata = FALSE;
  params.file_duration_limit = 0;
  params.file_size_limit = 0;
  params.force_stsz_table = FALSE;
  params.include_drm = FALSE;
  params.include_user_data = TRUE;
  params.movie_size_warning_imminent_threshold = 3;
  params.movie_size_warning_near_threshold = 10;
  params.movie_timescale = 1000;
  params.output_unit_size = 65536;
  params.version_major = 0;
  params.version_minor = 0;
  if (init_params.audio_stream_present) {
    params.stream_bitrate += init_params.audio_stream.bitrate;
  }
  if (init_params.video_stream_present) {
    params.stream_bitrate += init_params.video_stream.bitrate;
  }
  params.encrypt_param.streamEncrypted = false;
  params.encrypt_param.type = MUX_ENCRYPT_TYPE_INVALID;

  switch (brand_type_) {
    case MUX_BRAND_MP4:
      params.major_brand = kMP4MajorBrand;
      params.num_compat_brands = (uint32)(sizeof (kMP4CompatBrands)
                                    / sizeof (kMP4CompatBrands [0]));
      params.compat_brands = &kMP4CompatBrands [0];
      break;
    case MUX_BRAND_3GP:
      params.major_brand = k3GPMajorBrand;
      streams_[VIDEO_STREAM_INDEX].media_timescale = 1000;
      params.num_compat_brands = sizeof (k3GPCompatBrands)
              / sizeof (k3GPCompatBrands [0]);
      params.compat_brands = &k3GPCompatBrands[0];
      break;
    case MUX_BRAND_MP2TS:
      //TODO:
      break;
    default:
      ALOGW("%s: Unsupported brand type: %d\n", __func__, brand_type_);
      ret = BAD_VALUE;
  }

  return ret;
}

int32_t MuxInterface::SetAudioMuxerParams(MUX_create_params_type &params,
                                          const qmmf_muxer_init &init_params) {
  if (!init_params.audio_stream_present) {
    return NO_ERROR;
  }

  size_t  space_limit_threshold = params.movie_size_warning_near_threshold;
  size_t idx = AUDIO_STREAM_INDEX;
  size_t bitrate = init_params.audio_stream.bitrate;
  streams_[idx].interlace = AUDIO_STREAM_INDEX;
  streams_[idx].type = MUX_STREAM_AUDIO;
  streams_[idx].handler = "soun";
  streams_[idx].media_timescale =  VOCODER_SAMPLING_RATE;
  size_t buffer_size = 3 * MIN(DEAFULT_INTERLACE_PERIOD_MS,
                               OPTIMAL_CHUNK_DURATION(bitrate)) * bitrate/8;
  streams_[idx].buffer_size = (uint32)
                           MAX(buffer_size , MIN_AUDIO_MUX_BUFFER_SIZE);
  streams_[idx].max_header = QCP_FILE_HEADER_SIZE;
  streams_[idx].sample_delta = VOCODER_SAMPLING_RATE / VOCODER_PACKET_RATE;
  streams_[idx].max_samples = MMI_MAX_SAMPLES_PER_STORE;
  streams_[idx].chunk_size = (uint32) (bitrate/8) *
      MIN(DEAFULT_INTERLACE_PERIOD_MS,
          OPTIMAL_CHUNK_DURATION(bitrate));
  streams_[idx].max_chunks = MAX(DEFAULT_CHUNKS_TABLE_SIZE,
                                 (space_limit_threshold * 1000) /
                                 OPTIMAL_CHUNK_DURATION(bitrate) * 1000 + 1);
  streams_[idx].max_samples_rate = 50;
  streams_[idx].max_table_stores = ((MMI_MAX_STREAM_DURATION * 1000) /
      OPTIMAL_CHUNK_DURATION(bitrate) * 1000) / streams_[idx].max_chunks +
      MMI_MAX_STREAM_DURATION *
      streams_[idx].max_samples_rate / streams_[idx].max_samples ;
  streams_[idx].stsc_reset_rate = STSC_ALGO_RESET_RATE;
  streams_[idx].frames_per_sample = 1;

  switch (init_params.audio_stream.codec) {
    case CODEC_AAC:
      streams_[idx].subinfo.audio.format = MUX_STREAM_AUDIO_MPEG4_AAC;
      streams_[idx].subinfo.audio.num_channels =
          init_params.audio_stream.num_channels;
      streams_[idx].media_timescale = init_params.audio_stream.sample_rate;
      streams_[idx].max_header = 2;
      streams_[idx].sample_delta = AAC_SAMPLES_PER_FRAME;
      streams_[idx].sample_size = 0;
      streams_[idx].subinfo.audio.sampling_frequency =
          init_params.audio_stream.sample_rate;
      break;
    case CODEC_AMR:
      streams_[idx].subinfo.audio.format = MUX_STREAM_AUDIO_AMR;
      streams_[idx].subinfo.audio.num_channels =
          init_params.audio_stream.num_channels;
      streams_[idx].media_timescale = init_params.audio_stream.sample_rate;
      streams_[idx].subinfo.audio.sampling_frequency =
          init_params.audio_stream.sample_rate;
      streams_[idx].sample_size = 0;
      streams_[idx].sample_delta = init_params.audio_stream.sample_rate /
          VOCODER_PACKET_RATE;
      break;
    case CODEC_AMRWB:
      streams_[idx].subinfo.audio.format = MUX_STREAM_AUDIO_AMR_WB;
      streams_[idx].subinfo.audio.num_channels =
          init_params.audio_stream.num_channels;
      streams_[idx].media_timescale = init_params.audio_stream.sample_rate;
      streams_[idx].subinfo.audio.sampling_frequency =
          init_params.audio_stream.sample_rate;
      streams_[idx].sample_size = 0;
      streams_[idx].sample_delta = init_params.audio_stream.sample_rate /
          VOCODER_PACKET_RATE;
      break;
    case CODEC_PCM:
    default:
      ALOGE("%s: Unsupported audio codec: %d\n", __func__,
            init_params.audio_stream.codec);
      return BAD_VALUE;
  }

  audio_track_parms_ = init_params.audio_stream;
  audio_track_id_ = init_params.audio_stream.track_id;

  return NO_ERROR;
}

int32_t MuxInterface::SetVideoMuxerParams(MUX_create_params_type &params,
                                          const qmmf_muxer_init &init_params) {
  if (!init_params.video_stream_present) {
    return NO_ERROR;
  }

  if (0 == init_params.video_stream.framerate) {
    ALOGE("%s: Video frame rate invalid!\n", __func__);
    return BAD_VALUE;
  }

  size_t idx = VIDEO_STREAM_INDEX;
  size_t width = init_params.video_stream.width;
  size_t height = init_params.video_stream.height;
  size_t framerate = init_params.video_stream.framerate;
  size_t bitrate = init_params.video_stream.bitrate;
  size_t space_limit_threshold  = params.movie_size_warning_near_threshold;

  streams_[idx].type = MUX_STREAM_VIDEO;
  streams_[idx].subinfo.video.width  = width;
  streams_[idx].subinfo.video.height = height;
  streams_[idx].subinfo.video.frame_rate = framerate;
  streams_[idx].width = (width << 16);
  streams_[idx].height= (height << 16);
  streams_[idx].handler = "video";
  streams_[idx].media_timescale = 90000;
  streams_[idx].interlace = (uint32)idx;
  streams_[idx].interlace_rate = MAX(streams_[idx].media_timescale*
                                     DESIRED_INTERLACE_RATE(bitrate)/ 1000, 1);
  streams_[idx].chunk_size = OPTIMAL_CHUNK_SIZE(bitrate);
  streams_[idx].buffer_size = OPTIMAL_CHUNK_SIZE(bitrate) *
      OPTIMAL_STREAM_BUF_SIZE_FACTOR;
  streams_[idx].max_header   = MAX_HEADER_SIZE;
  streams_[idx].max_footer   = MAX_FOOTER_SIZE;
  streams_[idx].inter_frames = true;
  streams_[idx].chunks_out_near_threshold =
      (params.movie_size_warning_near_threshold * 1000) /
      DESIRED_INTERLACE_RATE(bitrate);
  streams_[idx].chunks_out_imminent_threshold =
      (params.movie_size_warning_imminent_threshold * 1000)
      / DESIRED_INTERLACE_RATE(bitrate);
  streams_[idx].samples_out_near_threshold =
      params.movie_size_warning_near_threshold * framerate;
  streams_[idx].samples_out_imminent_threshold =
      params.movie_size_warning_imminent_threshold * framerate;
  streams_[idx].max_samples  = MAX(MMI_MAX_SAMPLES_PER_STORE,
                                   space_limit_threshold * framerate);
  streams_[idx].max_chunks = MAX( DEFAULT_CHUNKS_TABLE_SIZE,
                                  (space_limit_threshold * 1000) /
                                  DESIRED_INTERLACE_RATE(bitrate) + 1);
  streams_[idx].max_samples_rate = framerate;
  streams_[idx].max_table_stores = (MMI_MAX_STREAM_DURATION * (bitrate >> 3) /
      streams_[idx].chunk_size /
      streams_[idx].max_chunks + ((MMI_MAX_STREAM_DURATION * framerate)/
          streams_[idx].max_samples) *3);
  streams_[idx].stsc_reset_rate = STSC_ALGO_RESET_RATE;

  streams_[idx].subinfo.video.format = init_params.video_stream.format;
  streams_[idx].subinfo.video.profile_comp = AVC_L1b_PROFILE_COM;
  streams_[idx].subinfo.video.profile = init_params.video_stream.codec_profile;
  streams_[idx].subinfo.video.level = init_params.video_stream.codec_level;
  video_track_id_ = init_params.video_stream.track_id;

  return NO_ERROR;
}

void MuxInterface::ReleaseBuffer(uint32_t track_id, uint32_t session_id,
                                 BufferDescriptor &buffer) {
  if (nullptr != release_cb_) {
    release_cb_(track_id, session_id, buffer);
  } else {
    ALOGE("%s: Release callback not set!\n", __func__);
  }
}

void MuxInterface::FileMuxCb(int status, void *client_data, void *sample_info,
                             void *buffer) {
  qmmf_muxer_client_data *data;
  MuxInterface *muxer_intf;

  if (NULL == client_data) {
    ALOGE("%s: invalid client data!\n", __func__);
    return;
  }
  muxer_intf = static_cast<MuxInterface*> (client_data);

  switch (status) {
    case PROCESS_SAMPLE_COMPLETE:
    case PROCESS_SAMPLE_FAIL:
    case PROCESS_HEADER_COMPLETE:
    case PROCESS_HEADER_FAIL:
    case PROCESS_SAMPLE_OUTDATED:
    case SPACE_LIMIT_REACHED:
    case WRITE_FAILED:
    case PROCESS_SAMPLE_FLUSH:
      if ((SPACE_LIMIT_REACHED == status) || (WRITE_FAILED == status)) {
        ALOGE("%s: Failed writing samples: %d!\n", __func__, status);
      }

      if (PROCESS_SAMPLE_OUTDATED == status) {
        ALOGE("%s: Outdated sample returned!\n", __func__);
      }

      if (NULL == sample_info) {
        ALOGE("%s: invalid sample info\n", __func__);
      } else {
        free(sample_info);
      }

      if (NULL == buffer) {
        ALOGE("%s: invalid buffer\n", __func__);
        return;
      }

      data = static_cast<qmmf_muxer_client_data*> (buffer);
      if ((0 < data->track_id) && (0 < data->session_id)) {
        muxer_intf->ReleaseBuffer(data->track_id, data->session_id,
                                  data->buffer);
      }

      free(data);
      break;
    case FLUSH_FAILED:
    case FLUSH_COMPLETED:
      pthread_mutex_lock(&muxer_intf->flush_lock_);

      muxer_intf->close_status_ = (FLUSH_COMPLETED == status) ?
          NO_ERROR : UNKNOWN_ERROR;
      muxer_intf->flush_completed_ = true;
      pthread_cond_signal(&muxer_intf->flush_cond_);

      pthread_mutex_unlock(&muxer_intf->flush_lock_);
      break;
    case CLOSE_MUX_COMPLETE:
    case CLOSE_MUX_FAIL:
      pthread_mutex_lock(&muxer_intf->stop_lock_);

      muxer_intf->close_status_ = (CLOSE_MUX_COMPLETE == status) ?
          NO_ERROR : UNKNOWN_ERROR;
      muxer_intf->close_completed_ = true;
      pthread_cond_signal(&muxer_intf->stop_cond_);

      pthread_mutex_unlock(&muxer_intf->stop_lock_);
      break;
    default:
      ALOGE("%s: Status: %d received!\n", __func__, status);
      break;
  }
}

} //namespace muxinterface ends
} //namespace qmmf ends
