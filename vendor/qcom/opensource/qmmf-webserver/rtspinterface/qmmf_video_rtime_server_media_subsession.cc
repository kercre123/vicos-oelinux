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
#include "qmmf_video_rtime_framed_source.h"
#include "qmmf_video_rtime_server_media_subsession.h"

/******************************************************************************
  global variable declaration
 ******************************************************************************/

/******************************************************************************
  class functions define
 ******************************************************************************/
VideoRTimeServerMediaSubsession *VideoRTimeServerMediaSubsession::CreateNew(
    UsageEnvironment &env, const tEsFmt codec_id, const u_int8_t frm_rate,
    AVQueue *que) {

  return new VideoRTimeServerMediaSubsession(env, codec_id, frm_rate, que);
}

VideoRTimeServerMediaSubsession::VideoRTimeServerMediaSubsession(
    UsageEnvironment &env, const tEsFmt codec_id, const u_int8_t frm_rate,
    AVQueue *que)
    : OnDemandServerMediaSubsession(env, True) {
  codec_id_ = codec_id;
  que_ = que;
  frm_rate_ = frm_rate;
  dummy_rtp_sink_ = NULL;
  sd_line_ = NULL;
  done_flag_ = 0;
}

VideoRTimeServerMediaSubsession::~VideoRTimeServerMediaSubsession() {
  if ((NULL != sd_line_)) {
    delete[] sd_line_;
  }
}

FramedSource *VideoRTimeServerMediaSubsession::createNewStreamSource(
    unsigned /*clientSessionId*/, unsigned & /*estBitrate*/) {
  FramedSource *pctFrmSource = NULL;

  /* Check codec type */
  switch ((codec_id_)) {
    case VIDEO_FORMAT_H264:
      pctFrmSource = H264VideoStreamDiscreteFramer::createNew(
          envir(), new VideoRTimeFramedSource(envir(), frm_rate_, que_));
      break;

    case VIDEO_FORMAT_H265:
      pctFrmSource = H265VideoStreamDiscreteFramer::createNew(
          envir(), new VideoRTimeFramedSource(envir(), frm_rate_, que_));
      break;

    case VIDEO_FORMAT_MPEG4:
      pctFrmSource = MPEG4VideoStreamFramer::createNew(
          envir(), new VideoRTimeFramedSource(envir(), frm_rate_, que_));
      break;
    default:
      break;
  }
  return pctFrmSource;
}

RTPSink *VideoRTimeServerMediaSubsession::createNewRTPSink(
    Groupsock *rtp_groupsock, unsigned char rtp_payload_type_if_dynamic,
    FramedSource * /*inputSource*/) {
  RTPSink *pctRtpSink = NULL;

  /* Check codec type */
  switch ((codec_id_)) {
    case VIDEO_FORMAT_H264:
      pctRtpSink = H264VideoRTPSink::createNew(envir(), rtp_groupsock,
                                               rtp_payload_type_if_dynamic);
      break;
    case VIDEO_FORMAT_H265:
      pctRtpSink = H265VideoRTPSink::createNew(envir(), rtp_groupsock,
                                               rtp_payload_type_if_dynamic);
      break;
    case VIDEO_FORMAT_MPEG4:
      pctRtpSink = MPEG4ESVideoRTPSink::createNew(envir(), rtp_groupsock,
                                                  rtp_payload_type_if_dynamic);
      break;
    default:
      break;
  }
  return pctRtpSink;
}

static void afterPlayingDummy(void *client_data) {
  VideoRTimeServerMediaSubsession *subsess =
      (VideoRTimeServerMediaSubsession *)client_data;
  subsess->AfterPlayingDummy1();
}

void VideoRTimeServerMediaSubsession::AfterPlayingDummy1() {
  /* Unschedule any pending 'checking' task */
  envir().taskScheduler().unscheduleDelayedTask(nextTask());
  /* Signal the event loop that we're done */
  SetDoneFlag();
}

static void chkForAuxSDPLine(void *client_data) {
  VideoRTimeServerMediaSubsession *subsess =
      (VideoRTimeServerMediaSubsession *)client_data;
  subsess->ChkForAuxSDPLine1();
}

void VideoRTimeServerMediaSubsession::ChkForAuxSDPLine1() {
  char const *dasl = NULL;

  if ((NULL != sd_line_)) {
    /* Signal the event loop that we're done */
    SetDoneFlag();
  } else if ((NULL != dummy_rtp_sink_) &&
             ((dasl = dummy_rtp_sink_->auxSDPLine()) != NULL)) {
    sd_line_ = strDup(dasl);
    dummy_rtp_sink_ = NULL;
    /* Signal the event loop that we're done */
    SetDoneFlag();
  } else if ((!done_flag_)) {
    int to_delay = 100000;  // 100 ms
    if ((frm_rate_ > 0)) {
      to_delay = 1000000 / frm_rate_;
    }
    nextTask() = envir().taskScheduler().scheduleDelayedTask(
        to_delay, (TaskFunc *)chkForAuxSDPLine, this);
  }
}

char const *VideoRTimeServerMediaSubsession::getAuxSDPLine(
    RTPSink *rtp_sink, FramedSource *input_source) {
  /* It's already been set up (for a previous client) */
  if ((NULL != sd_line_)) {
    return sd_line_;
  }

  if (NULL == dummy_rtp_sink_) {
    dummy_rtp_sink_ = rtp_sink;

    /* Start reading */
    dummy_rtp_sink_->startPlaying(*input_source, afterPlayingDummy, this);

    /* Check whether the sink's 'auxSDPLine()' is ready */
    chkForAuxSDPLine(this);
  }
  envir().taskScheduler().doEventLoop(&done_flag_);
  return sd_line_;
}
