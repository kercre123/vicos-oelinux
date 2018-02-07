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
#include "MediaSink.hh"
#include "qmmf_metadata_framed_source.h"

/******************************************************************************
  class functions define
 ******************************************************************************/
MetadataFramedSource::MetadataFramedSource(UsageEnvironment &env, AVQueue *que,
                                           size_t fps)
    : FramedSource(env), queue_(que), last_timestamp_(0), fps_(fps) {
  token_ = NULL;
  OutPacketBuffer::increaseMaxSizeTo(MAX_PACKET_SIZE);
}

MetadataFramedSource::~MetadataFramedSource() {
  envir().taskScheduler().unscheduleDelayedTask(token_);
  token_ = NULL;
}

void MetadataFramedSource::GetNextFrame(void *client_data) {
  ((MetadataFramedSource *)client_data)->GetFrameData();
}

void MetadataFramedSource::doGetNextFrame() {
  /* Calculate wait time with frame rate */

  double delay = 1000.0 / fps_;  // ms
  int to_delay = delay * 1000;   // us
  token_ = envir().taskScheduler().scheduleDelayedTask(
      to_delay, (TaskFunc *)GetNextFrame, this);
}

void MetadataFramedSource::GetFrameData() {
  /* Initialize require read size */
  size_t read = fMaxSize;
  fFrameSize = 0;

  AVPacket *pkt = (AVPacket *)AVQueuePopTail(queue_);
  if ((NULL != pkt)) {
    if ((fPresentationTime.tv_sec == 0 && fPresentationTime.tv_usec == 0)) {
      /* This is the first frame, so use the current time */
      gettimeofday(&fPresentationTime, NULL);
      last_timestamp_ = pkt->timestamp;
    } else {
      int64_t current_ts_delta = pkt->timestamp - last_timestamp_;
      if (0 < current_ts_delta) {
        fPresentationTime.tv_usec += current_ts_delta;
        fPresentationTime.tv_sec += (fPresentationTime.tv_usec / 1000000);
        fPresentationTime.tv_usec = (fPresentationTime.tv_usec % 1000000);
      } else {
        ALOGE("%s: Invalid timestamp: %lld first timestamp: %lld\n", __func__,
              pkt->timestamp, last_timestamp_);
      }
      last_timestamp_ = pkt->timestamp;
    }

    if (pkt->size <= read) {
      fNumTruncatedBytes = 0;
      read = pkt->size;
    } else {
      fNumTruncatedBytes = pkt->size - read;
    }

    if (pkt->data != NULL) {
      unsigned char *ptr = fTo;
      memcpy(ptr, pkt->data, read);
      free(pkt->data);
    }
    free(pkt);
  } else {
    read = 0;
  }

  fDurationInMicroseconds = 1000000.0 / fps_;
  fFrameSize = read;

  afterGetting(this);
}
