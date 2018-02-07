/*
* Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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
#ifndef _REAL_TIME_RTSP_SERVER_COMMON_H
#define _REAL_TIME_RTSP_SERVER_COMMON_H

#include <pthread.h>
#include <qmmf-sdk/qmmf_queue.h>

using namespace qmmf;
/******************************************************************************
  macro define
 ******************************************************************************/
#define FRAME_PER_SEC (25)
#define rtsp_db_info(fmt, args...)

/******************************************************************************
  enum declaration
 ******************************************************************************/
/* rtsp server support format */
typedef enum {
  AUDIO_FORMAT_ADTS = 0,
  AUDIO_FORMAT_AMR,
  VIDEO_FORMAT_H264,
  VIDEO_FORMAT_H265,
  VIDEO_FORMAT_MPEG4,
  VIDEO_FORMAT_MP2TS,
  VIDEO_FORMAT_YUV,
  ES_FORMAT_MAX
} tEsFmt;

/* aac using,
Sampling frequency index */
typedef enum _FS_IDX_et {
  FS_IDX_96 = 0,
  FS_IDX_88,
  FS_IDX_64,
  FS_IDX_48,
  FS_IDX_44,
  FS_IDX_32,
  FS_IDX_24,
  FS_IDX_22,
  FS_IDX_16,
  FS_IDX_12,
  FS_IDX_11,
  FS_IDX_08,
  FS_IDX_07,
  FS_IDX_MAX
} FS_IDX_et;

/* aac using,
Channel configuration */
typedef enum _CH_CFG_et {
  CH_CFG_0 = 0,
  CH_CFG_1,
  CH_CFG_2,
  CH_CFG_3,
  CH_CFG_4,
  CH_CFG_5,
  CH_CFG_6,
  CH_CFG_7,
  CH_CFG_MAX
} CH_CFG_et;

/* aac using,
Profile identity */
typedef enum _PROFILE_et {
  PROFILE_0 = 0,
  PROFILE_1,
  PROFILE_2,
  PROFILE_MAX
} PROFILE_et;

typedef enum _MPEG_VERSION_et {
  MPEG_VERSION_AAC = 4,
  MPEG_VERSION_H264 = 5,
  MPEG_VERSION_H265 = 6
} MPEG_VERSION_et;

/******************************************************************************
  global variable declaration
 ******************************************************************************/
#endif /* _REAL_TIME_RTSP_SERVER_COMMON_H */
