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
#include <inttypes.h>
#include "qmmf_video_rtime_framed_source.h"

/******************************************************************************
  macro define
 ******************************************************************************/

/******************************************************************************
  class functions define
 ******************************************************************************/
VideoRTimeFramedSource::VideoRTimeFramedSource(UsageEnvironment &env,
                                               const u_int8_t frm_rate,
                                               AVQueue *que)
    : FramedSource(env) {

  /* Clear class members */
  token_ = NULL;
  que_ = que;

  /* Get frame rate of video stream */
  frm_rate_ = frm_rate;
  left_time_ = 0;
  last_timestamp_ = 0;
}

VideoRTimeFramedSource::~VideoRTimeFramedSource() {
  envir().taskScheduler().unscheduleDelayedTask(token_);
  token_ = NULL;
}

#ifdef NOCARE_FRM_RATE
#else
void VideoRTimeFramedSource::getNextFrame(void *clientData) {
  ((VideoRTimeFramedSource *)clientData)->GetFrameData();
}
#endif /* NOCARE_FRM_RATE */

void VideoRTimeFramedSource::doGetNextFrame() {
#ifdef NOCARE_FRM_RATE
  GetFrameData();
#else
  /* Calculate wait time with frame rate */
  double delay = 1000.0 / frm_rate_;  // ms
  int to_delay = delay * 1000;        // us
  token_ =
#ifdef SCHED_NODELAY
      envir().taskScheduler().scheduleDelayedTask(0, (TaskFunc *)getNextFrame,
                                                  this);
#else
      envir().taskScheduler().scheduleDelayedTask(
          to_delay, (TaskFunc *)getNextFrame, this);
#endif /* SCHED_NODELAY */
#endif /* NOCARE_FRM_RATE */
}

void VideoRTimeFramedSource::GetFrameData() {
  /* Initialize require read size */
  size_t read = fMaxSize;
  fFrameSize = 0;
  int64_t current_ts_delta = 0;

  AVPacket *pkt = (AVPacket *)AVQueuePopTail(que_);
  if ((NULL != pkt)) {
    if ((fPresentationTime.tv_sec == 0 && fPresentationTime.tv_usec == 0)) {
      /* This is the first frame, so use the current time */
      gettimeofday(&fPresentationTime, NULL);
      last_timestamp_ = pkt->timestamp;
    } else {
      current_ts_delta = pkt->timestamp - last_timestamp_;
      if (0 < current_ts_delta) {
        fPresentationTime.tv_usec += current_ts_delta;
        fPresentationTime.tv_sec += (fPresentationTime.tv_usec / 1000000);
        fPresentationTime.tv_usec = (fPresentationTime.tv_usec % 1000000);
      } else {
        ALOGE("%s: Invalid timestamp: %" PRId64 " first timestamp: %"
                   PRId64 "\n", __func__, pkt->timestamp, last_timestamp_);
        read = 0;
        return;
      }
      last_timestamp_ = pkt->timestamp;
    }

    if ((pkt->size <= read)) {
      fNumTruncatedBytes = 0;
      read = pkt->size;
    } else {
      fNumTruncatedBytes = pkt->size - read;
    }
    if ((NULL != pkt->data)) {
      memcpy(fTo, pkt->data, read);
      free(pkt->data);
    }
    free(pkt);
  } else {
    read = 0;
  }

  fFrameSize = read;

#ifdef NOCARE_FRM_RATE
  nextTask() = envir().taskScheduler().scheduleDelayedTask(
      0, (TaskFunc *)FramedSource::afterGetting, this);
#else
  /* It is read successfully */
  afterGetting(this);
#endif /* NOCARE_FRM_RATE */
}
