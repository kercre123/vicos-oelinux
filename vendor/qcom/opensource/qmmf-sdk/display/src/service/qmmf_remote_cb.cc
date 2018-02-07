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

#define TAG "DisplayRemoteCallBack"

#include "display/src/service/qmmf_display_common.h"
#include "display/src/service/qmmf_remote_cb.h"

namespace qmmf {

namespace display {

RemoteCallBack::RemoteCallBack(const sp<IDisplayServiceCallback>&
    remote_client): client_cb_handle_(remote_client) {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

RemoteCallBack::~RemoteCallBack() {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

void RemoteCallBack::notifyDisplayEvent(DisplayEventType event_type,
    void *event_data, size_t event_data_size) {

  assert(client_cb_handle_.get() != nullptr);
  client_cb_handle_->notifyDisplayEvent(event_type, event_data,
      event_data_size);
}

void RemoteCallBack::notifySessionEvent(DisplayEventType event_type,
    void *event_data, size_t event_data_size) {

  assert(client_cb_handle_.get() != NULL);
  client_cb_handle_->notifySessionEvent(event_type, event_data,
      event_data_size);
}

void RemoteCallBack::notifyVSyncEvent(int64_t time_stamp) {

  assert(client_cb_handle_.get() != NULL);
  client_cb_handle_->notifyVSyncEvent(time_stamp);
}

}; // namespace display

}; // namespace qmmf
