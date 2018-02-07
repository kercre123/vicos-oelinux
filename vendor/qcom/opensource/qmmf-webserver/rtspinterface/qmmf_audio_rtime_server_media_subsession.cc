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
#include <pthread.h>
#include <utils/Log.h>
#include "qmmf_amr_rtime_framed_source.h"
#include "qmmf_adts_rtime_framed_source.h"
#include "qmmf_audio_rtime_server_media_subsession.h"

/******************************************************************************
  class functions define
 ******************************************************************************/
AudioRTimeServerMediaSubsession *AudioRTimeServerMediaSubsession::CreateNew(
    UsageEnvironment &env, const tEsFmt codec_id, const CH_CFG_et ch_cfg,
    const FS_IDX_et fs_idx, const PROFILE_et profile, const Boolean wide_band,
    const u_int8_t ch_num, AVQueue *que) {

  return new AudioRTimeServerMediaSubsession(env, codec_id, ch_cfg, fs_idx,
                                             profile, wide_band, ch_num, que);
}

AudioRTimeServerMediaSubsession::AudioRTimeServerMediaSubsession(
    UsageEnvironment &env, const tEsFmt codec_id, const CH_CFG_et ch_cfg,
    const FS_IDX_et fs_idx, const PROFILE_et profile, const Boolean wide_band,
    const u_int8_t ch_num, AVQueue *que)
    : OnDemandServerMediaSubsession(env, True) {
  codec_id_ = codec_id;

  que_ = que;

  ch_cfg_ = ch_cfg;
  rtsp_db_info("m_eChCfg=%d", ch_cfg_);
  fs_idx_ = fs_idx;
  rtsp_db_info("m_eFsIdx=%d", fs_idx_);
  profile_ = profile;
  rtsp_db_info("m_eProfile=%d", profile_);
  wide_band_ = wide_band;
  rtsp_db_info("m_bWideBand=%d", wide_band_);
  ch_num_ = ch_num;
  rtsp_db_info("m_ucChNum=%d", ch_num_);
}

FramedSource *AudioRTimeServerMediaSubsession::createNewStreamSource(
    unsigned, unsigned &) {
  FramedSource *pctFrmSource = NULL;

  /* Check codec type */
  switch ((codec_id_)) {
    case AUDIO_FORMAT_ADTS:
      pctFrmSource = ADtsRTimeFramedSource::CreateNew(envir(), ch_cfg_, fs_idx_,
                                                      profile_, que_);
      break;
    case AUDIO_FORMAT_AMR:
      pctFrmSource =
          AmrRTimeFramedSource::CreateNew(envir(), wide_band_, ch_num_, que_);
      break;
    default:
      ALOGE("%s: unsupported codec type\n", __func__);
      break;
  }
  return pctFrmSource;
}

RTPSink *AudioRTimeServerMediaSubsession::createNewRTPSink(
    Groupsock *rtp_groupsock, unsigned char rtp_payload_type_if_dynamic,
    FramedSource *input_source) {
  RTPSink *pctRtpSink = NULL;

  /* Check codec type */
  switch ((codec_id_)) {
    case AUDIO_FORMAT_ADTS: {
      ADtsRTimeFramedSource *adtsSource = (ADtsRTimeFramedSource *)input_source;

      pctRtpSink = MPEG4GenericRTPSink::createNew(
          envir(), rtp_groupsock, rtp_payload_type_if_dynamic,
          adtsSource->SamplingFrequency(), "audio", "AAC-hbr",
          adtsSource->ConfigStr(), adtsSource->NumChannels());
    } break;
    case AUDIO_FORMAT_AMR: {
      AmrRTimeFramedSource *amrSource = (AmrRTimeFramedSource *)input_source;
      pctRtpSink = AMRAudioRTPSink::createNew(
          envir(), rtp_groupsock, rtp_payload_type_if_dynamic,
          amrSource->isWideband(), amrSource->numChannels());
    } break;
    default:
      ALOGE("%s: unsupported codec type\n", __func__);
      break;
  }
  return pctRtpSink;
}
