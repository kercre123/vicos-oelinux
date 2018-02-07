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
#include "qmmf_metadata_server_media_subsession.h"
#include "qmmf_metadata_framed_source.h"

/******************************************************************************
  class functions define
 ******************************************************************************/
MetadataServerMediaSubsession *MetadataServerMediaSubsession::CreateNew(
    UsageEnvironment &env, AVQueue *que, size_t fps) {
  return new MetadataServerMediaSubsession(env, que, fps);
}

MetadataServerMediaSubsession::MetadataServerMediaSubsession(
    UsageEnvironment &env, AVQueue *que, size_t fps)
    : OnDemandServerMediaSubsession(env, True), queue_(que), fps_(fps) {}

FramedSource *MetadataServerMediaSubsession::createNewStreamSource(
    unsigned /*clientSessionId*/, unsigned & /*estBitrate*/) {
  return new MetadataFramedSource(envir(), queue_, fps_);
}

RTPSink *MetadataServerMediaSubsession::createNewRTPSink(
    Groupsock *rtp_groupsock, unsigned char rtp_payload_type_if_dynamic,
    FramedSource * /*inputSource*/) {
  RTPSink *pctRtpSink = SimpleRTPSink::createNew(
      envir(), rtp_groupsock, rtp_payload_type_if_dynamic, 90000, "application",
      "metadata", 1, False, True);
  return pctRtpSink;
}
