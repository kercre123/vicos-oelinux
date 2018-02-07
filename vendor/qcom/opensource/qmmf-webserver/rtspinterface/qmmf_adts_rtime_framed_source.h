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
#ifndef _ADTS_FRAMED_RTIME_SOURCE_H
#define _ADTS_FRAMED_RTIME_SOURCE_H

#include <FramedSource.hh>
#include "qmmf_realtime_rtsp_server_common.h"

/******************************************************************************
  macro define
 ******************************************************************************/
#define MAX_AFRAME_SIZE (4 * 1024)
#define ADTS_HEADER_SIZE 7
/******************************************************************************
  enum declaration
 ******************************************************************************/

/******************************************************************************
  class declaration
 ******************************************************************************/
class ADtsRTimeFramedSource : public FramedSource {
 public:
  u_int32_t SamplingFrequency() const { return fs_; }
  u_int8_t NumChannels() const { return ch_; }
  char const *ConfigStr() const { return config_str_; }

  static ADtsRTimeFramedSource *CreateNew(UsageEnvironment &env,
                                          const CH_CFG_et ch_cfg,
                                          const FS_IDX_et fs_idx,
                                          const PROFILE_et profile, AVQueue *que);

  virtual unsigned int maxFrameSize() const {
    return MAX_AFRAME_SIZE;
  };

 private:
  /* Constructor function */

  ADtsRTimeFramedSource(UsageEnvironment &env, const CH_CFG_et ch_cfg,
                        const FS_IDX_et fs_idx, const PROFILE_et profile,
                        AVQueue *que);

  /* Destructor function */
  virtual ~ADtsRTimeFramedSource(void);
  virtual void doGetNextFrame();

 private:
  AVQueue *que_;
  /* Microsecond each frame */
  u_int32_t usec_per_frm_;
  u_int32_t usec_remain_;
  /* Sample frequency */
  u_int32_t fs_;
  /* Channel number */
  u_int8_t ch_;
  /* AudioSpecificConfig */
  char config_str_[5];
  PROFILE_et profile_;
  FS_IDX_et sampling_idx_;
  CH_CFG_et channels_;
  int64_t last_timestamp_;
};
#endif /* _ADTS_FRAMED_RTIME_SOURCE_H */
