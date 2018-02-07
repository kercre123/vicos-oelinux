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
#ifndef _AUDIO_RTIME_SERVER_MEDIA_SUBSESSION_H
#define _AUDIO_RTIME_SERVER_MEDIA_SUBSESSION_H

#include <liveMedia.hh>
#include "qmmf_realtime_rtsp_server_common.h"

/******************************************************************************
  class declaration
 ******************************************************************************/
class AudioRTimeServerMediaSubsession : public OnDemandServerMediaSubsession {
 public:
  /* Create new server media session object */
  static AudioRTimeServerMediaSubsession *CreateNew(
      UsageEnvironment &env, const tEsFmt codec_id = AUDIO_FORMAT_ADTS,
      const CH_CFG_et ch_cfg = CH_CFG_2, const FS_IDX_et fs_idx = FS_IDX_48,
      const PROFILE_et profile = PROFILE_0, const Boolean wide_band = False,
      const u_int8_t ch_num = 1, AVQueue *que = NULL);

 protected:
  AudioRTimeServerMediaSubsession(UsageEnvironment &env, const tEsFmt codec_id,
                                  const CH_CFG_et ch_cfg,
                                  const FS_IDX_et fs_idx,
                                  const PROFILE_et profile,
                                  const Boolean wide_band,
                                  const u_int8_t ch_num, AVQueue *que);

  virtual ~AudioRTimeServerMediaSubsession(void) {};

  /* "estBitrate" is the stream's estimated bitrate, in kbps */
  virtual FramedSource *createNewStreamSource(unsigned client_session_id,
                                              unsigned &bitrate);

  virtual RTPSink *createNewRTPSink(Groupsock *rtp_groupsock,
                                    unsigned char rtp_payload_type_if_dynamic,
                                    FramedSource *input_source);

 private:
  AVQueue *que_;
  /* Codec identity */
  tEsFmt codec_id_;
  /* Channel configuration (aac using) */
  CH_CFG_et ch_cfg_;
  /* FS index (aac using) */
  FS_IDX_et fs_idx_;
  /* Profile identity (aac using) */
  PROFILE_et profile_;
  /* Wide band (amr using) */
  Boolean wide_band_;
  /* Channel number (amr using) */
  u_int8_t ch_num_;
};

#endif /* _AUDIO_RTIME_SERVER_MEDIA_SUBSESSION_H */
