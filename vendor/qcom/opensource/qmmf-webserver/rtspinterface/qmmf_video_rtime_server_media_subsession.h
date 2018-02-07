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
#ifndef _VIDEO_RTIME_SERVER_MEDIA_SUBSESSION_H
#define _VIDEO_RTIME_SERVER_MEDIA_SUBSESSION_H

#include <liveMedia.hh>
#include "qmmf_realtime_rtsp_server_common.h"
/******************************************************************************
  class declaration
 ******************************************************************************/
class VideoRTimeServerMediaSubsession : public OnDemandServerMediaSubsession {
 public:
  /* Create new server media session object */
  static VideoRTimeServerMediaSubsession *CreateNew(
      UsageEnvironment &env, const tEsFmt codec_id = VIDEO_FORMAT_H264,
      const u_int8_t frm_rate = FRAME_PER_SEC, AVQueue *que = NULL);

  void AfterPlayingDummy1();
  void ChkForAuxSDPLine1();

 protected:
  VideoRTimeServerMediaSubsession(UsageEnvironment &env, const tEsFmt codec_id,
                                  const u_int8_t frm_rate, AVQueue *que);
  virtual ~VideoRTimeServerMediaSubsession(void);

  virtual char const *getAuxSDPLine(RTPSink *rtpSink,
                                    FramedSource *inputSource);

  /* "estBitrate" is the stream's estimated bitrate, in kbps */
  virtual FramedSource *createNewStreamSource(unsigned client_session_id,
                                              unsigned &bitrate);

  virtual RTPSink *createNewRTPSink(Groupsock *rtp_groupsock,
                                    unsigned char rtp_payload_type_if_dynamic,
                                    FramedSource *input_source);
  void SetDoneFlag() { done_flag_ = ~0; }

 private:
  AVQueue *que_;
  /* RTP sink pointer */
  RTPSink *dummy_rtp_sink_;
  /* SDP line pointer */
  char *sd_line_;
  /* Done flag */
  char done_flag_;
  /* Codec identity */
  tEsFmt codec_id_;
  /* Frame per second */
  u_int8_t frm_rate_;
};

#endif /* _VIDEO_RTIME_SERVER_MEDIA_SUBSESSION_H */
