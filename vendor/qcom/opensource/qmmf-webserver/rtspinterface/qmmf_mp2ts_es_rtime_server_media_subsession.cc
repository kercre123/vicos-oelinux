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
#include "qmmf_mp2ts_rtime_framed_source.h"
#include "qmmf_video_rtime_framed_source.h"
#include "qmmf_adts_rtime_framed_source.h"
#include "qmmf_mp2ts_es_rtime_server_media_subsession.h"

/******************************************************************************
  class functions define
 ******************************************************************************/
Mp2TsESRTimeServerMediaSubsession *Mp2TsESRTimeServerMediaSubsession::CreateNew(
    UsageEnvironment &env, tEsFmt video_codec_id, u_int8_t frm_rate,
    AVQueue *video_queue, tEsFmt audio_codec_id, CH_CFG_et ch_cfg,
    FS_IDX_et fs_idx, PROFILE_et profile, AVQueue *audio_queue) {
  return new Mp2TsESRTimeServerMediaSubsession(
      env, video_codec_id, frm_rate, video_queue, audio_codec_id, ch_cfg,
      fs_idx, profile, audio_queue);
}

Mp2TsESRTimeServerMediaSubsession::Mp2TsESRTimeServerMediaSubsession(
    UsageEnvironment &env, tEsFmt video_codec_id, u_int8_t frm_rate,
    AVQueue *video_queue, tEsFmt audio_codec_id, CH_CFG_et ch_cfg,
    FS_IDX_et fs_idx, PROFILE_et profile, AVQueue *audio_queue)
    : OnDemandServerMediaSubsession(env, True),
      video_codec_id_(video_codec_id),
      frm_rate_(frm_rate),
      video_queue_(video_queue),
      audio_codec_id_(audio_codec_id),
      ch_cfg_(ch_cfg),
      fs_idx_(fs_idx),
      profile_(profile),
      audio_queue_(audio_queue) {}

FramedSource *Mp2TsESRTimeServerMediaSubsession::createNewStreamSource(
    unsigned /*clientSessionId*/, unsigned & /*estBitrate*/) {
  MPEG_VERSION_et videoMpegVersion, audioMpegVersion;
  MPEG2TransportStreamFromESSource *pctSource = NULL;

  switch (video_codec_id_) {
    case VIDEO_FORMAT_H264:
      videoMpegVersion = MPEG_VERSION_H264;
      break;
    case VIDEO_FORMAT_H265:
      videoMpegVersion = MPEG_VERSION_H265;
      break;
    default:
      return NULL;
  }

  if (FS_IDX_MAX != fs_idx_) {
    switch (audio_codec_id_) {
      case AUDIO_FORMAT_ADTS:
        audioMpegVersion = MPEG_VERSION_AAC;
        break;
      default:
        ALOGE("%s: Audio codec: %d is not yet supported!\n", __func__,
              audio_codec_id_);
        return NULL;
    }
  }

  pctSource = MPEG2TransportStreamFromESSource::createNew(envir());
  if (NULL == pctSource) {
    ALOGE("%s: Unable to create MPEG2TransportStreamFromESSource!\n",
               __func__);
    return NULL;
  }

  pctSource->addNewVideoSource(
      new VideoRTimeFramedSource(envir(), frm_rate_, video_queue_),
      videoMpegVersion);
  if (FS_IDX_MAX != fs_idx_) {
    pctSource->addNewAudioSource(
        ADtsRTimeFramedSource::CreateNew(envir(), ch_cfg_, fs_idx_, profile_,
                                         audio_queue_),
        audioMpegVersion);
  }

  return pctSource;
}

RTPSink *Mp2TsESRTimeServerMediaSubsession::createNewRTPSink(
    Groupsock *rtp_groupsock, unsigned char rtp_payload_type_if_dynamic,
    FramedSource * /*input_source*/) {
  RTPSink *pctRtpSink = SimpleRTPSink::createNew(
      envir(), rtp_groupsock, rtp_payload_type_if_dynamic, 90000, "video",
      "MP2T", 2, True, True);
  return pctRtpSink;
}
