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
#include <inttypes.h>
#include <utils/Log.h>
#include "qmmf_adts_rtime_framed_source.h"

/******************************************************************************
  static global array define
 ******************************************************************************/
static u_int32_t const sampling_frequency_table[] = {
    96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050,
    16000, 12000, 11025, 8000,  7350,  0,     0,     0};

/******************************************************************************
  class functions define
 ******************************************************************************/
ADtsRTimeFramedSource *ADtsRTimeFramedSource::CreateNew(
    UsageEnvironment &env, const CH_CFG_et ch_cfg, const FS_IDX_et fs_idx,
    const PROFILE_et profile, AVQueue *que) {
  return new ADtsRTimeFramedSource(env, ch_cfg, fs_idx, profile, que);
}

ADtsRTimeFramedSource::ADtsRTimeFramedSource(UsageEnvironment &env,
                                             const CH_CFG_et ch_cfg,
                                             const FS_IDX_et fs_idx,
                                             const PROFILE_et profile,
                                             AVQueue *que)
    : FramedSource(env),
      profile_(profile),
      sampling_idx_(fs_idx),
      channels_(ch_cfg),
      last_timestamp_(0) {
  que_ = que;

  /* Get sampling frequency with fs index */
  fs_ = sampling_frequency_table[(int)fs_idx];
  if (0 == fs_) {
    return;
  }

  /* Get channel number with channel config */
  ch_ = ((int)ch_cfg == 0) ? 2 : (int)ch_cfg;

  /* Calculate microsecond each frame */
  usec_per_frm_ = (1024 * 1000000) / fs_;
  usec_remain_ = (1024 * 1000000) % fs_;

  {/* Construct the 'AudioSpecificConfig' */
    unsigned char audioSpecificConfig[2];
    u_int8_t const audioObjectType = (int)profile + 1;
    audioSpecificConfig[0] = (audioObjectType << 3) | ((int)fs_idx >> 1);

    audioSpecificConfig[1] = ((int)fs_idx << 7) | ((int)ch_cfg << 3);

    snprintf(config_str_, sizeof(config_str_), "%02X%02x",
             audioSpecificConfig[0], audioSpecificConfig[1]);
  }
}

ADtsRTimeFramedSource::~ADtsRTimeFramedSource() {}

void ADtsRTimeFramedSource::doGetNextFrame() {
  /* Initialize require read size */
  size_t read = fMaxSize;
  fFrameSize = 0;
  size_t adtsPacketSize = 0;
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
        ALOGE("%s: Invalid timestamp: %" PRId64
               " first timestamp:"
               "%" PRId64 "\n",
               __func__, pkt->timestamp, last_timestamp_);
      }
      last_timestamp_ = pkt->timestamp;
    }

    adtsPacketSize = pkt->size + ADTS_HEADER_SIZE;
    if (adtsPacketSize <= read) {
      fNumTruncatedBytes = 0;
      read = adtsPacketSize;
    } else {
      fNumTruncatedBytes = adtsPacketSize - read;
    }

    if (pkt->data != NULL) {
      unsigned char *ptr = fTo;

      /* Generate ADTS header */
      *ptr++ = 0xff;
      *ptr++ = 0xf1; /* b11110001, ID=0, layer=0, protection_absent=1 */
      *ptr++ = profile_ << 6 | sampling_idx_ << 2 |
               ((channels_ >> 2) & 1);  // private_bit=0

      // original_copy=0, home=0, copyright_id_bit=0, copyright_id_start=0
      *ptr++ = (channels_ & 3) << 6 | adtsPacketSize >> 11;
      *ptr++ = (adtsPacketSize >> 3) & 0xff;
      *ptr++ = (adtsPacketSize & 7) << 5;

      // adts_buffer_fullness=0, number_of_raw_data_blocks_in_frame=0
      *ptr++ = 0;
      memcpy(ptr, pkt->data, read - ADTS_HEADER_SIZE);

      free(pkt->data);
    }
    free(pkt);
  } else {
    read = 0;
  }

  if (0 < current_ts_delta) {
    fDurationInMicroseconds = current_ts_delta;
  } else {
    fDurationInMicroseconds = (1024 * 1000000.0) / fs_;
  }

  fFrameSize = read;

  nextTask() = envir().taskScheduler().scheduleDelayedTask(
      0, (TaskFunc *)FramedSource::afterGetting, this);
}
