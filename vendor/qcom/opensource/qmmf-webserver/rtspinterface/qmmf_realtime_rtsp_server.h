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
#ifndef _REAL_TIME_RTSP_SERVER_H
#define _REAL_TIME_RTSP_SERVER_H

#include <RTSPServer.hh>
#include <ServerMediaSession.hh>
#include <BasicUsageEnvironment.hh>

/******************************************************************************
  macro define
 ******************************************************************************/
#define DEFAULT_PORT (8899)

/******************************************************************************
  enum declaration
 ******************************************************************************/
/* rtsp server error identity */
typedef enum _RTSPSVR_ERR_et {
  RTSPSVR_ERR_NOERR = 0,
  RTSPSVR_ERR_SMS,
  RTSPSVR_ERR_ASMSS,
  RTSPSVR_ERR_ASMS,
  RTSPSVR_ERR_TASK
} RTSPSVR_ERR_et;

/******************************************************************************
  class declaration
 ******************************************************************************/
class RealTimeRtspServer {
 public:
  UsageEnvironment *GetUsageEnv() const { return env_; }
  RealTimeRtspServer(u_int16_t port = 0);
  ~RealTimeRtspServer();

  /* Create server media session instance */
  RTSPSVR_ERR_et CreateSMS(char const *strm_name = NULL,
                           char const *info = NULL,
                           char const *description = NULL,
                           Boolean is_ssm = False,
                           char const *misc_sdp_lines = NULL);

  /* Add server media subsession to server media session */
  RTSPSVR_ERR_et AddSMSSToSMS(ServerMediaSubsession *smss);

  /* Add server media session to rtsp server (called multiply) */
  RTSPSVR_ERR_et AddSMSToRtspSvr();

  /* Get url address of rtsp server (need delete return pointer ) */
  const char *GetRtspServerUrl();

  /* Start task scheduler loop (block, call in thread or process) */
  RTSPSVR_ERR_et StartTaskScheduler();

  /* Closes (from the server) all RTSP client sessions */
  void ResetRtspServer();

  /* Stop task scheduler loop */
  void StopTaskScheduler() { exit_flag_ = 1; }

 private:
  static pthread_mutex_t mutex_;
  static u_int16_t port_;
  TaskScheduler *scheduler_;
  UsageEnvironment *env_;
  RTSPServer *rtsp_svr_;
  ServerMediaSession *sms_;
  char exit_flag_;
};
#endif /* _REAL_TIME_RTSP_SERVER_H */
