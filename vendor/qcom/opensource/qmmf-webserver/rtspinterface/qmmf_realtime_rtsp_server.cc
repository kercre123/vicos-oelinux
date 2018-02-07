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
#include <pthread.h>
#include <utils/Log.h>
#include <MediaSink.hh>
#include "qmmf_realtime_rtsp_server.h"

/******************************************************************************
  static member initialize
 ******************************************************************************/
pthread_mutex_t RealTimeRtspServer::mutex_ = PTHREAD_MUTEX_INITIALIZER;
u_int16_t RealTimeRtspServer::port_ = DEFAULT_PORT;

/******************************************************************************
  class functions define
 ******************************************************************************/
RealTimeRtspServer::RealTimeRtspServer(u_int16_t port) {

  sms_ = NULL;
  exit_flag_ = 0;
  rtsp_svr_ = NULL;

  /* Begin by setting up our usage environment */
  scheduler_ = BasicTaskScheduler::createNew();
  if ((NULL == scheduler_)) {
    ALOGE("%s: create BasicTaskScheduler failed\n", __func__);
    goto FAIL;
  }

  env_ = BasicUsageEnvironment::createNew(*scheduler_);
  if ((NULL == env_)) {
    ALOGE("%s: create UsageEnvironment failed\n", __func__);
    goto FAIL;
  }

  /* Get rtsp server port */
  pthread_mutex_lock(&mutex_);
  if ((0 == port)) {
    port = port_++;
  }
  pthread_mutex_unlock(&mutex_);

  /* Create the RTSP server */
  OutPacketBuffer::maxSize = 1024 * 1024;
  rtsp_svr_ = RTSPServer::createNew(*env_, port, NULL);
  if ((NULL == rtsp_svr_)) {
    ALOGE("%s: create RTSPServer failed\n", __func__);
    goto FAIL;
  }
  return;

FAIL:
  if ((NULL != env_)) {
    env_->reclaim();
    env_ = NULL;
  }

  if ((NULL != scheduler_)) {
    delete scheduler_;
    scheduler_ = NULL;
  }
}

#define CLOSESMS closeAllClientSessionsForServerMediaSession
#define REMOVSMS removeServerMediaSession

void RealTimeRtspServer::ResetRtspServer() {
  if ((NULL != rtsp_svr_)) {
    if ((NULL != sms_)) {
      rtsp_svr_->CLOSESMS(sms_);
      sms_->deleteAllSubsessions();
      rtsp_svr_->REMOVSMS(sms_);
      sms_ = NULL;
    }
  }
}

RealTimeRtspServer::~RealTimeRtspServer() {
  if ((NULL != rtsp_svr_)) {
    Medium::close(rtsp_svr_);
    rtsp_svr_ = NULL;
  }

  if ((NULL != env_)) {
    env_->reclaim();
    env_ = NULL;
  }

  if ((NULL != scheduler_)) {
    delete scheduler_;
    scheduler_ = NULL;
  }
}

RTSPSVR_ERR_et RealTimeRtspServer::CreateSMS(char const* strm_name,
                                             char const* info,
                                             char const* description,
                                             Boolean is_ssm,
                                             char const* misc_sdp_lines) {
  RTSPSVR_ERR_et eRet = RTSPSVR_ERR_NOERR;

  if ((NULL != env_) && (NULL == sms_)) {
    sms_ = ServerMediaSession::createNew(*env_, strm_name, info, description,
                                         is_ssm, misc_sdp_lines);
    if ((NULL == sms_)) {
      ALOGE("%s: create SMS failed\n", __func__);
      eRet = RTSPSVR_ERR_SMS;
    }
  } else {
    if ((NULL == env_)) {
      ALOGE("%s: pctEnv is null\n", __func__);
      eRet = RTSPSVR_ERR_SMS;
    }
  }
  return eRet;
}

RTSPSVR_ERR_et RealTimeRtspServer::AddSMSSToSMS(ServerMediaSubsession* smss) {
  RTSPSVR_ERR_et eRet = RTSPSVR_ERR_NOERR;

  if ((NULL != sms_) && (NULL != smss)) {
    sms_->addSubsession(smss);
  } else {
    ALOGE("%s: add SMSS failed\n", __func__);
    eRet = RTSPSVR_ERR_ASMSS;
  }
  return eRet;
}

RTSPSVR_ERR_et RealTimeRtspServer::AddSMSToRtspSvr() {
  RTSPSVR_ERR_et eRet = RTSPSVR_ERR_NOERR;

  if ((NULL != sms_) && (NULL != rtsp_svr_)) {
    rtsp_svr_->addServerMediaSession(sms_);
  } else {
    ALOGE("%s: add SMSS failed\n", __func__);
    eRet = RTSPSVR_ERR_ASMS;
  }

  return eRet;
}

const char* RealTimeRtspServer::GetRtspServerUrl() {
  char* url = NULL;

  if ((NULL != sms_) && (NULL != rtsp_svr_)) {
    url = rtsp_svr_->rtspURL(sms_);
  } else {
    ALOGE("%s: get rtsp server url failed\n", __func__);
  }

  return url;
}

RTSPSVR_ERR_et RealTimeRtspServer::StartTaskScheduler() {
  RTSPSVR_ERR_et eRet = RTSPSVR_ERR_NOERR;
  exit_flag_ = 0;

  if ((NULL != env_)) {
    env_->taskScheduler().doEventLoop(&exit_flag_);
  } else {
    ALOGE("%s: pctEnv is null\n", __func__);
    eRet = RTSPSVR_ERR_TASK;
  }

  return eRet;
}
