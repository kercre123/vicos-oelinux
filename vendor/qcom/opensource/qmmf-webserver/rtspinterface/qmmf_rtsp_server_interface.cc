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
#include <string.h>
#include "qmmf_rtsp_server_interface.h"

/******************************************************************************
  macro define
 ******************************************************************************/
#define SMSS_CLASS1 AudioRTimeServerMediaSubsession
#define SMSS_CLASS2 VideoRTimeServerMediaSubsession
#define SMSS_CLASS3 Mp2TsRTimeServerMediaSubsession
#define SMSS_CLASS4 Mp2TsESRTimeServerMediaSubsession
#define SMSS_CLASS5 MetadataServerMediaSubsession

/******************************************************************************
  thread functions define (static)
 ******************************************************************************/
static void *start_task_scheduler(void *thread_arg) {
  RealTimeRtspServer *pctSvr = NULL;
  if ((NULL != thread_arg)) {
    pctSvr = (RealTimeRtspServer *)thread_arg;
    pctSvr->StartTaskScheduler();
  }
  return NULL;
}

/******************************************************************************
  class functions define
 ******************************************************************************/
void RtspServerInterface::QueueInit(AVQueue **que, QueueType type, int size,
                                    int delay) {
  AVQueueInit(que, type, size, delay);
}

#ifdef DEBUG_PACKETNO
void RtspServerInterface::QuePushData(int number, const char *enc_type,
                                      const uint8_t *buffer, size_t size,
                                      queue_t *que)
#else
void RtspServerInterface::QuePushData(const char *enc_type,
                                      const uint8_t *buffer, size_t size,
                                      int64_t timestamp, AVQueue *que)
#endif /* DEBUG_PACKETNO */
{
  int bufferSize = 0;
  uint8_t *pucTmp = NULL;
  AVPacket *packet = NULL;

  if ((size <= 5) || (NULL == que)) {
    return;
  }
  if (!strncmp(enc_type, "AVC", strlen("AVC"))) {
    if (buffer[0] == 0x00 && buffer[1] == 0x00 && buffer[2] == 0x00 &&
        buffer[3] == 0x01 && buffer[4] == 0x67) {/* SPS,PPS*/
      if (que->pps != NULL) {
        free(que->pps);
        que->pps = NULL;
      }
      que->pps = (char *)malloc(sizeof(char) * (size));
      memcpy(que->pps, buffer, size);
      que->pps_size = size;
      que->is_pps = true;
      return;
    }
    if (buffer[0] == 0x00 && buffer[1] == 0x00 && buffer[2] == 0x00 &&
        buffer[3] == 0x01 && buffer[4] == 0x65) {
      if (que->is_pps != true) {
        bufferSize = que->pps_size;
      }
      que->is_pps = false;
    }
  } else if (!strncmp(enc_type, "HEVC", strlen("HEVC"))) {
    if (buffer[0] == 0x00 && buffer[1] == 0x00 && buffer[2] == 0x00 &&
        buffer[3] == 0x01 && buffer[4] == 0x40) {/* VPS,SPS,PPS*/
      if (que->pps != NULL) {
        free(que->pps);
        que->pps = NULL;
      }
      que->pps = (char *)malloc(sizeof(char) * (size));
      memcpy(que->pps, buffer, size);
      que->pps_size = size;
      que->is_pps = true;
      return;
    }

    if (buffer[0] == 0x00 && buffer[1] == 0x00 && buffer[2] == 0x00 &&
        buffer[3] == 0x01 && buffer[4] == 0x26) {
      if (que->is_pps != true) {
        bufferSize = que->pps_size;
      }
      que->is_pps = false;
    }
  }

  /* Set pointer to start address */
  packet = (AVPacket *)malloc(sizeof(AVPacket));
  if ((NULL == packet)) {
    return;
  }

  /* Allocate a new frame object. */
  packet->data = pucTmp = (uint8_t *)malloc((size + bufferSize));
  if ((NULL == packet->data)) {
    free(packet);
    return;
  }

#ifdef DEBUG_PACKETNO
  packet->number = number;
#endif /* DEBUG_PACKETNO */

  if ((0 != bufferSize)) {
    memcpy(pucTmp, que->pps, que->pps_size);
    pucTmp += que->pps_size;
  }
  memcpy(pucTmp, buffer, size);
  packet->size = size + bufferSize;
  packet->timestamp = timestamp;
  AVQueuePushHead(que, packet);
}

void RtspServerInterface::QueueDInit(AVQueue **que) {
  if (((*que) != NULL)) {
    AVQueueFree(que, AVFreePacket);
    *que = NULL;
  }
}
/******************************************************************************
  class functions define
 ******************************************************************************/
RtspServerInterface::RtspServerInterface(u_int16_t port)
    : url_(NULL), usg_env_(NULL), svr_(NULL) {
  /* Create rtsp server */
  svr_ = new RealTimeRtspServer(port);

  /* Clear thread identity */
  thread_id_ = 0;
  thread_running_ = false;
}

void RtspServerInterface::CreateSMS() {

  if ((NULL != svr_)) {
    /* Create SMS */
    svr_->CreateSMS("live", NULL, "www rtsp live", false, NULL);

    /* Get usage environment */
    usg_env_ = svr_->GetUsageEnv();
  }
}

void RtspServerInterface::AddAudioSMSSToSMS(tEsFmt codec_id, CH_CFG_et ch_cfg,
                                            FS_IDX_et fs_idx,
                                            PROFILE_et profile,
                                            Boolean wide_band, u_int8_t ch_num,
                                            AVQueue *que) {
  if ((NULL != svr_) && (NULL != usg_env_)) {
    svr_->AddSMSSToSMS(SMSS_CLASS1::CreateNew(
        *usg_env_, codec_id, ch_cfg, fs_idx, profile, wide_band, ch_num, que));
  }
}

void RtspServerInterface::AddMetaSMSSToSMS(AVQueue *que, size_t fps) {
  if ((NULL != svr_) && (NULL != usg_env_)) {
    svr_->AddSMSSToSMS(SMSS_CLASS5::CreateNew(*usg_env_, que, fps));
  }
}

void RtspServerInterface::AddVideoSMSSToSMS(tEsFmt codec_id, u_int8_t frm_rate,
                                            AVQueue *que) {
  if ((NULL != svr_) && (NULL != usg_env_)) {
    svr_->AddSMSSToSMS(
        SMSS_CLASS2::CreateNew(*usg_env_, codec_id, frm_rate, que));
  }
}

void RtspServerInterface::AddESTsSMSSToSMS(
    tEsFmt video_codec_id, u_int8_t frm_rate, AVQueue *video_queue,
    tEsFmt audio_codec_id, CH_CFG_et ch_cfg, FS_IDX_et fs_idx,
    PROFILE_et profile, AVQueue *audio_queue) {
  if ((NULL != svr_) && (NULL != usg_env_)) {
    svr_->AddSMSSToSMS(SMSS_CLASS4::CreateNew(
        *usg_env_, video_codec_id, frm_rate, video_queue, audio_codec_id,
        ch_cfg, fs_idx, profile, audio_queue));
  }
}

void RtspServerInterface::AddTsSMSSToSMS(AVQueue *que) {
  if ((NULL != svr_) && (NULL != usg_env_)) {
    svr_->AddSMSSToSMS(SMSS_CLASS3::CreateNew(*usg_env_, que));
  }
}

void RtspServerInterface::StartTaskScheduler() {
  if ((NULL != svr_)) {
    /* Add SMS to rtsp server */
    svr_->AddSMSToRtspSvr();
    /* Create thread start task scheduler */
    pthread_create(&thread_id_, NULL, start_task_scheduler, (void *)svr_);
    thread_running_ = true;
  }

  if (NULL == url_) {
    url_ = svr_->GetRtspServerUrl();
  }
}

/*Query RTSP URL */
size_t RtspServerInterface::GetURLSize() {
  if (url_) {
    return strlen(url_) + 1;
  }

  return 0;
}

void RtspServerInterface::GetURL(char *url, size_t size) {
  if (url && (0 < size)) {
    strncpy(url, url_, size);
  }
}

void RtspServerInterface::StopTaskScheduler() {
  if ((NULL != svr_)) {
    svr_->StopTaskScheduler();
    if (thread_running_) {
      pthread_join(thread_id_, NULL);
      thread_id_ = 0;
      thread_running_ = false;
    }
  }
}

void RtspServerInterface::ResetRtspServer() {
  if ((NULL != svr_)) {
    svr_->ResetRtspServer();
  }
}

RtspServerInterface::~RtspServerInterface() {
  if ((NULL != svr_)) {
    svr_->ResetRtspServer();
    svr_->StopTaskScheduler();
    if (thread_running_) {
      pthread_join(thread_id_, NULL);
      thread_id_ = 0;
      thread_running_ = false;
    }
    delete svr_;
    svr_ = NULL;
  }

  if (NULL != url_) {
    delete url_;
    url_ = NULL;
  }
}
