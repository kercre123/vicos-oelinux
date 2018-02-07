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
#include "qmmf_amr_rtime_framed_source.h"

/******************************************************************************
  macro define
 ******************************************************************************/
#define AMR_MAXFRM_SIZE (512)
#ifdef NOCMP_FRAME
#define FT_INVALID (65535)
#endif /* NOCMP_FRAME */

/******************************************************************************
  static variables define
 ******************************************************************************/
#ifdef NOCMP_FRAME
static unsigned short const frameSize[] = {
    12,         13,         15,         17,         19,         20,
    26,         31,         5,          FT_INVALID, FT_INVALID, FT_INVALID,
    FT_INVALID, FT_INVALID, FT_INVALID, 0};

static unsigned short const frameSizeWideband[] = {
    17, 23, 32,         36,         40,         46,         50, 58,
    60, 5,  FT_INVALID, FT_INVALID, FT_INVALID, FT_INVALID, 0,  0};
#endif /* NOCMP_FRAME */

/******************************************************************************
  class functions define
 ******************************************************************************/
AmrRTimeFramedSource *AmrRTimeFramedSource::CreateNew(UsageEnvironment &env,
                                                      const Boolean wide_band,
                                                      const u_int8_t ch_num,
                                                      AVQueue *que) {
  return new AmrRTimeFramedSource(env, wide_band, ch_num, que);
}

AmrRTimeFramedSource::AmrRTimeFramedSource(UsageEnvironment &env,
                                           const Boolean wide_band,
                                           const u_int8_t ch_num, AVQueue *que)
    : AMRAudioSource(env, wide_band, ch_num) {
  que_ = que;

  local_buffer_ = new u_int8_t[AMR_MAXFRM_SIZE];
  if ((NULL == local_buffer_)) {
    ALOGE("%s: malloc local buffer failed\n", __func__);
    return;
  }
#ifdef NOCMP_FRAME
  m_usWrite = 0;
#endif /* NOCMP_FRAME */
}

AmrRTimeFramedSource::~AmrRTimeFramedSource() {
  if ((NULL != local_buffer_)) {
    delete[] local_buffer_;
    local_buffer_ = NULL;
  }
}

void AmrRTimeFramedSource::doGetNextFrame() {
#ifndef NOCMP_FRAME
  size_t read = AMR_MAXFRM_SIZE;
  fFrameSize = 0;

  AVPacket *pkt = (AVPacket *)AVQueuePopTail(que_);
  if ((NULL != pkt)) {
    if ((pkt->size <= read)) {
      fNumTruncatedBytes = 0;
      read = pkt->size;
    } else {
      fNumTruncatedBytes = pkt->size - read;
    }

    if ((NULL != pkt->data)) {
      memcpy(local_buffer_, pkt->data, read);
      free(pkt->data);
    }
    free(pkt);
  } else {
    read = 0;
  }

  fLastFrameHeader = local_buffer_[0];
  if ((read > 1)) {
    memcpy(fTo, local_buffer_ + 1, read - 1);
    fFrameSize = read - 1;
  }

  /* Update presentation time with frame rate */
  if ((fPresentationTime.tv_sec == 0 && fPresentationTime.tv_usec == 0)) {
    /* This is the first frame, so use the current time */
    gettimeofday(&fPresentationTime, NULL);
  } else {
    u_int32_t uSeconds = fPresentationTime.tv_usec + 20000;
    fPresentationTime.tv_sec += (uSeconds / 1000000);
    fPresentationTime.tv_usec = (uSeconds % 1000000);
  }
  fDurationInMicroseconds = 20000;

  nextTask() = envir().taskScheduler().scheduleDelayedTask(
      0, (TaskFunc *)FramedSource::afterGetting, this);
#else
  volatile u_int16_t usFrmSize = 0;
  volatile u_int16_t usLeft = m_usWrite;

  fFrameSize = 0;

  /* Read a valid frame */
  while ((1)) {
    u_int8_t *pucData = (u_int8_t *)local_buffer_;
    rtsp_db_info("usLeft=%d", (int)usLeft);

    /* Check invalid frame header */
    while ((usLeft > 0)) {
      u_int8_t ucData = *pucData++;
      usLeft = usLeft - 1;
      if ((0 == (ucData & 0x83))) {
        u_int8_t ft = (ucData & 0x78) >> 3;
        usFrmSize = fIsWideband ? frameSizeWideband[ft] : frameSize[ft];
        if ((usFrmSize == FT_INVALID)) {
          ALOGE("%s: Invalid FT field\n", __func__);
          usFrmSize = 0;
        } else {
          usFrmSize *= fNumChannels;
          fLastFrameHeader = ucData;
          break;
        }
      }
    }

    rtsp_db_info("usLeft=%d, usFrmSize=%d", usLeft, (int)usFrmSize);

    /* Check reamain data size > frame size? */
    if ((0 == usFrmSize) || ((usFrmSize > 0) && (usLeft < usFrmSize))) {
      int read = AMR_MAXFRM_SIZE - usLeft;
      AVPacket *pkt = (AVPacket *)q_pop_tail(que_);
      if ((NULL != pkt)) {
        if ((pkt->size <= read)) {
          read = pkt->size;
        }
        if ((NULL != pkt->data)) {
          memcpy(local_buffer_, pkt->data, read);
          free(pkt->data);
        }
        free(pkt);
      } else {
        read = 0;
        break;
      }
      usLeft = usLeft + read;
      if ((0 == usFrmSize)) {
        continue;
      }
    }

    /* Re-setting write index of local buffer */
    m_usWrite = usLeft - usFrmSize;

    /* Check frame size > max frame size? */
    if ((usFrmSize > fMaxSize)) {
      fNumTruncatedBytes = usFrmSize - fMaxSize;
      usFrmSize = fMaxSize;
    } else {
      fNumTruncatedBytes = 0;
    }

    fFrameSize = usFrmSize;
    rtsp_db_info("m_usWrite=%d", m_usWrite);

    /* Copy data into frame buffer */
    if ((fFrameSize > 0)) {
      memcpy(fTo, pucData, fFrameSize);
    }

    if ((m_usWrite > 0)) {
      memcpy(local_buffer_, pucData + fFrameSize, m_usWrite);
    }

    /* Update presentation time with frame rate */
    if ((fPresentationTime.tv_sec == 0 && fPresentationTime.tv_usec == 0)) {
      /* This is the first frame, so use the current time */
      gettimeofday(&fPresentationTime, NULL);
    } else {
      u_int32_t uSeconds = fPresentationTime.tv_usec + 20000;
      fPresentationTime.tv_sec += (uSeconds / 1000000);
      fPresentationTime.tv_usec = (uSeconds % 1000000);
    }
    fDurationInMicroseconds = 20000;

    nextTask() = envir().taskScheduler().scheduleDelayedTask(
        0, (TaskFunc *)FramedSource::afterGetting, this);
    break;
  }
#endif /* NOCMP_FRAME */
}
