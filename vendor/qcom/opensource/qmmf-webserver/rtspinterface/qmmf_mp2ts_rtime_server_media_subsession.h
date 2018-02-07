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
#ifndef _MP2TS_RTIME_SERVER_MEDIA_SUBSESSION_H
#define _MP2TS_RTIME_SERVER_MEDIA_SUBSESSION_H

#include <liveMedia.hh>
#include "qmmf_realtime_rtsp_server_common.h"
/******************************************************************************
  class declaration
 ******************************************************************************/
class Mp2TsRTimeServerMediaSubsession : public OnDemandServerMediaSubsession {
 public:
  /* Create new server media session object */
  static Mp2TsRTimeServerMediaSubsession *CreateNew(UsageEnvironment &env,
                                                    AVQueue *que = NULL);

 protected:
  Mp2TsRTimeServerMediaSubsession(UsageEnvironment &env, AVQueue *que);
  virtual ~Mp2TsRTimeServerMediaSubsession(void) {};

 private:
  /* "estBitrate" is the stream's estimated bitrate, in kbps */
  virtual FramedSource *createNewStreamSource(unsigned client_session_id,
                                              unsigned &bitrate);

  virtual RTPSink *createNewRTPSink(Groupsock *rtp_groupsock,
                                    unsigned char rtp_payload_type_if_dynamic,
                                    FramedSource *input_source);

 private:
  AVQueue *que_;
};
#endif /* _MP2TS_RTIME_SERVER_MEDIA_SUBSESSION_H */
