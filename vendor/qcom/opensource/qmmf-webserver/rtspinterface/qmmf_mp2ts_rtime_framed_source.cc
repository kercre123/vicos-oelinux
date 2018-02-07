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
#include "qmmf_mp2ts_rtime_framed_source.h"

/******************************************************************************
  static class member define
 ******************************************************************************/
int Mp2TsRTimeFramedSource::hand_id_ = 110;

/******************************************************************************
  class functions define
 ******************************************************************************/
Mp2TsRTimeFramedSource *Mp2TsRTimeFramedSource::CreateNew(
    UsageEnvironment &env, AVQueue *que, unsigned preferred_frame_size) {
  return new Mp2TsRTimeFramedSource(env, que, preferred_frame_size);
}

Mp2TsRTimeFramedSource::Mp2TsRTimeFramedSource(UsageEnvironment &env,
                                               AVQueue *que,
                                               unsigned preferred_frame_size)
    : FramedSource(env) {
  /* Clear class members */
  preferred_frame_size_ = preferred_frame_size;
  have_started_reading_ = False;
  obj_hand_id_ = hand_id_++;
  que_ = que;
}

Mp2TsRTimeFramedSource::~Mp2TsRTimeFramedSource() {
#ifndef READ_FROM_FILES_SYNCHRONOUSLY
  envir().taskScheduler().turnOffBackgroundReadHandling(obj_hand_id_);
#endif /* READ_FROM_FILES_SYNCHRONOUSLY */
}

void Mp2TsRTimeFramedSource::doGetNextFrame() {

#ifdef READ_FROM_FILES_SYNCHRONOUSLY

  DoRealTimeRead();
#else
  if ((!have_started_reading_)) {
    envir().taskScheduler().turnOnBackgroundReadHandling(
        obj_hand_id_,
        (TaskScheduler::BackgroundHandlerProc *)&RTimeReadableHandler, this);
    have_started_reading_ = True;
  }
#endif /* READ_FROM_FILES_SYNCHRONOUSLY */
}

void Mp2TsRTimeFramedSource::doStopGettingFrames() {
  envir().taskScheduler().unscheduleDelayedTask(nextTask());
#ifndef READ_FROM_FILES_SYNCHRONOUSLY
  envir().taskScheduler().turnOffBackgroundReadHandling(obj_hand_id_);
  have_started_reading_ = False;
#endif /* READ_FROM_FILES_SYNCHRONOUSLY */
}

void Mp2TsRTimeFramedSource::RTimeReadableHandler(
    Mp2TsRTimeFramedSource *source, int /*mask*/) {
  if ((!source->isCurrentlyAwaitingData())) {
    source->doStopGettingFrames();
    return;
  }
  source->DoRealTimeRead();
}

void Mp2TsRTimeFramedSource::DoRealTimeRead() {
  size_t read = 0;
  fFrameSize = 0;

  /* Checking frame size */
  if (preferred_frame_size_ > 0 && preferred_frame_size_ < fMaxSize) {
    fMaxSize = preferred_frame_size_;
  }

  /* Initialize require read size */
  read = fMaxSize;
  AVPacket *pkt = (AVPacket *)AVQueuePopTail(que_);
  if ((NULL != pkt)) {
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
  if (fFrameSize == 0) {
    handleClosure();
    return;
  }

  /* Set the 'presentation time' */
  gettimeofday(&fPresentationTime, NULL);

#ifdef READ_FROM_FILES_SYNCHRONOUSLY
  /* To avoid possible infinite recursion, we need
   * to return to the event loop to do this */
  nextTask() = envir().taskScheduler().scheduleDelayedTask(
      0, (TaskFunc *)FramedSource::afterGetting, this);
#else
  FramedSource::afterGetting(this);
#endif /* READ_FROM_FILES_SYNCHRONOUSLY */
}
