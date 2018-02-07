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
#ifndef _VIDEO_FRAMED_RTIME_SOURCE_H
#define _VIDEO_FRAMED_RTIME_SOURCE_H

#include <FramedSource.hh>
#include "qmmf_realtime_rtsp_server_common.h"
/******************************************************************************
  macro define
 ******************************************************************************/
#define MAX_VFRAME_SIZE (1024 * 1024)
#define NOCARE_FRM_RATE
#define SCHED_NODELAY

/******************************************************************************
  class declaration
 ******************************************************************************/
class VideoRTimeFramedSource : public FramedSource {
 public:
  virtual unsigned int maxFrameSize() const {
    return MAX_VFRAME_SIZE;
  };
  /* Constructor function */
  VideoRTimeFramedSource(UsageEnvironment &env, const u_int8_t frm_rate,
                         AVQueue *que);

  /* Destructor function */
  virtual ~VideoRTimeFramedSource();
  virtual void doGetNextFrame();
  virtual void GetFrameData();

#ifdef NOCARE_FRM_RATE
#else
 private:
  static void getNextFrame(void *client_data);
#endif /* NOCARE_FRM_RATE */

 private:
  AVQueue *que_;
  /* Left time value */
  u_int32_t left_time_;
  /* Frame rate */
  u_int32_t frm_rate_;
  void *token_;
  int64_t last_timestamp_;
};

#endif /* _VIDEO_FRAMED_RTIME_SOURCE_H */
