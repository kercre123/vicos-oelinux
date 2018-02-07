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
#ifndef _RTSP_SERVER_INTERFACE_H
#define _RTSP_SERVER_INTERFACE_H
#include <pthread.h>

#include <qmmf-sdk/qmmf_queue.h>
#include "qmmf_realtime_rtsp_server.h"
#include "qmmf_audio_rtime_server_media_subsession.h"
#include "qmmf_video_rtime_server_media_subsession.h"
#include "qmmf_mp2ts_rtime_server_media_subsession.h"
#include "qmmf_mp2ts_es_rtime_server_media_subsession.h"
#include "qmmf_metadata_server_media_subsession.h"

/******************************************************************************
  macro define
 ******************************************************************************/
#define DEFAULT_PORT (8899)
#define MAX_QUEUE_SIZE (8)
#define DELAY_SIZE (1)

using namespace qmmf;

/******************************************************************************
  class declaration
 ******************************************************************************/
class RtspServerInterface {
 public:
  static void QueueInit(AVQueue **que, QueueType type = REALTIME,
                        int size = MAX_QUEUE_SIZE, int delay = DELAY_SIZE);

#ifdef DEBUG_PACKETNO
  static void QuePushData(int number, const char *enc_type,
                          const uint8_t *buffer, size_t size, queue_t *que);
#else
  static void QuePushData(const char *enc_type, const uint8_t *buffer,
                          size_t size, int64_t timestamp, AVQueue *que);
#endif /* DEBUG_PACKETNO */
  static void QueueDInit(AVQueue **que);

  RtspServerInterface(u_int16_t port = 0);
  ~RtspServerInterface();

  /* Create sms */
  void CreateSMS();

  /* Add amr or adts smss to sms */
  void AddAudioSMSSToSMS(tEsFmt codec_id, CH_CFG_et ch_cfg, FS_IDX_et fs_idx,
                         PROFILE_et profile, Boolean wide_band, u_int8_t ch_num,
                         AVQueue *que);

  /* Add h264 or h265 smss to sms */
  void AddVideoSMSSToSMS(tEsFmt codec_id, u_int8_t frm_rate, AVQueue *que);

  /* Add ts smss to sms */
  void AddTsSMSSToSMS(AVQueue *que);

  void AddMetaSMSSToSMS(AVQueue *que, size_t fps);

  /* Add ES ts smss to sms*/
  void AddESTsSMSSToSMS(tEsFmt video_codec_id, u_int8_t frm_rate,
                        AVQueue *video_queue, tEsFmt audio_codec_id,
                        CH_CFG_et ch_cfg, FS_IDX_et fs_idx, PROFILE_et profile,
                        AVQueue *audio_queue);

  /* Reset rtsp server  */
  void ResetRtspServer();

  /* Start task sheduler */
  void StartTaskScheduler();

  /* Stop task sheduler */
  void StopTaskScheduler();

  /*Query RTSP URL */
  size_t GetURLSize();
  void GetURL(char *url, size_t size);

 private:
  const char *url_;
  /* Usage environment pointer  */
  UsageEnvironment *usg_env_;
  /* Real time rtsp server pointer */
  RealTimeRtspServer *svr_;
  /* The thread indentity */
  pthread_t thread_id_;
  bool thread_running_;
};
#endif /* _RTSP_SERVER_INTERFACE_H */
