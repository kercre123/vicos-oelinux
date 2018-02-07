/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#define TAG "RecorderPostProcThread"

#include <sys/prctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "qmmf_postproc_thread.h"
#include "common/qmmf_log.h"

namespace qmmf {

namespace recorder {

int32_t PostProcThread::Run(const std::string &name) {
  int32_t res = 0;

  std::lock_guard<std::mutex> lock(lock_);
  if (running_) {
    QMMF_ERROR("%s: Thread %s already started!\n", __func__, name_.c_str());
    res = -ENOSYS;
    goto exit;
  }

  running_ = true;
  thread_ = new std::thread(MainLoop, this);
  if (thread_ == nullptr) {
    QMMF_ERROR("%s: Unable to create thread\n", __func__);
    running_ = false;
    goto exit;
  }

  if (name.empty()) {
    // use thread id as name
    std::stringstream ss;
    ss << thread_->get_id();
    name_ = ss.str();
  } else {
    name_ = name;
  }

  QMMF_INFO("%s: Thread %s is running\n", __func__, name_.c_str());

exit:
  return res;
}

void PostProcThread::RequestExit() {
  std::lock_guard<std::mutex> lock(lock_);
  if (thread_ == nullptr || running_ == false) {
    QMMF_ERROR("%s: Thread %s is not running\n", __func__, name_.c_str());
    return;
  }

  abort_ = true;
}

void PostProcThread::RequestExitAndWait() {
  std::lock_guard<std::mutex> lock(lock_);
  if (thread_ == nullptr) {
    QMMF_ERROR("%s: Thread %s is stopped\n", __func__, name_.c_str());
    return;
  }

  abort_ = true;
  thread_->join();
  delete(thread_);
  thread_ = nullptr;
}

void *PostProcThread::MainLoop(void *userdata) {
  PostProcThread *pme = reinterpret_cast<PostProcThread *>(userdata);
  if (NULL == pme) {
    pme->running_ = false;
    return NULL;
  }

  bool run = true;
  while (pme->abort_ == false && run == true) {
    run = pme->ThreadLoop();
  }

  pme->running_ = false;
  return NULL;
}

bool PostProcThread::ExitPending() {
  std::lock_guard<std::mutex> lock(lock_);
  return (abort_ == true && running_ == true)  ? false : true;
}

}  // namespace recorder ends here

}  // namespace qmmf ends here
