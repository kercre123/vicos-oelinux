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
#ifndef _AMR_FRAMED_RTIME_SOURCE_H
#define _AMR_FRAMED_RTIME_SOURCE_H

#include <AMRAudioSource.hh>
#include "qmmf_realtime_rtsp_server_common.h"

//#define NOCMP_FRAME   /* not complete frame */
/******************************************************************************
  class declaration
 ******************************************************************************/
class AmrRTimeFramedSource : public AMRAudioSource {
 public:
  static AmrRTimeFramedSource *CreateNew(UsageEnvironment &env,
                                         const Boolean wide_band,
                                         const u_int8_t ch_num, AVQueue *que);

 private:
  /* Constructor function */
  AmrRTimeFramedSource(UsageEnvironment &env, const Boolean wide_band,
                       const u_int8_t ch_num, AVQueue *que);
  /* Destructor function */
  virtual ~AmrRTimeFramedSource(void);
  virtual void doGetNextFrame();

 private:
  AVQueue *que_;

  /* Local buffer for frame check */
  u_int8_t *local_buffer_;
#ifdef NOCMP_FRAME
  /* Local buffer write index */
  u_int16_t m_usWrite;
#endif /* NOCMP_FRAME */
};
#endif /* _AMR_FRAMED_RTIME_SOURCE_H */
