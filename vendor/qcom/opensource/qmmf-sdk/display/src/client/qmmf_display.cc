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

#define TAG "Display"

#include "qmmf-sdk/qmmf_display.h"
#include "qmmf-sdk/qmmf_display_params.h"
#include "display/src/client/qmmf_display_client.h"
#include "display/src/service/qmmf_display_common.h"
namespace qmmf {

namespace display {

Display::Display()
    : display_client_(nullptr) {

  display_client_ = new DisplayClient();
  assert( display_client_ != NULL);
}

Display::~Display() {

  if (display_client_) {
      delete display_client_;
      display_client_ = nullptr;
  }
}

status_t Display::Connect() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(display_client_ != nullptr);

  auto ret = display_client_->Connect();
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s: Connect failed!", __func__);
  }
  return ret;
}

status_t Display::Disconnect() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  auto ret = display_client_->Disconnect();
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s:%s Disconnect failed!", TAG, __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Display::CreateDisplay(DisplayType type, DisplayCb& cb) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(cb.VSyncCb != nullptr);

  auto ret = display_client_->CreateDisplay(type, cb);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s: CreateDisplay failed!", __func__);
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Display::DestroyDisplay(DisplayType type) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(display_client_ != nullptr);

  auto ret = display_client_->DestroyDisplay(type);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s:%s DestroyDisplay failed!", TAG, __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Display::CreateSurface(const SurfaceConfig &surface_config,
  uint32_t* surface_id) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(&surface_config != NULL);
  assert(surface_id != NULL);

  auto ret = display_client_->CreateSurface((SurfaceConfig &)surface_config,
      surface_id);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s: CreateSurface failed!", __func__);
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Display::DestroySurface(const uint32_t surface_id) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  auto ret = display_client_->DestroySurface(surface_id);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s: DestroySurface failed!", __func__);
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Display::DequeueSurfaceBuffer(const uint32_t surface_id,
        SurfaceBuffer &surface_buffer) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(&surface_buffer != NULL);

  auto ret = display_client_->DequeueSurfaceBuffer(surface_id, surface_buffer);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s: DequeueSurfaceBuffer failed!", __func__);
  }

  QMMF_INFO("%s:%s: EXIT", TAG, __func__);
  return ret;
}

status_t Display::QueueSurfaceBuffer(const uint32_t surface_id,
      const SurfaceBuffer &surface_buffer,
      const SurfaceParam &surface_param) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(&surface_buffer != NULL);
  assert(&surface_param != NULL);

  auto ret = display_client_->QueueSurfaceBuffer(surface_id,
    (SurfaceBuffer &)surface_buffer, (SurfaceParam &)surface_param);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s: QueueSurfaceBuffer failed!", __func__);
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Display::GetDisplayParam(DisplayParamType param_type, void *param,
      size_t param_size) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(param != NULL);
  assert(param_size != 0);

  auto ret = display_client_->GetDisplayParam(param_type, param, param_size);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s: GetDisplayParam failed!", __func__);
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Display::SetDisplayParam(DisplayParamType param_type,
      const void *param, size_t param_size) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(param != NULL);
  assert(param_size != 0);

  auto ret = display_client_->SetDisplayParam(param_type, (void *)param,
      param_size);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s: SetDisplayParam failed!", __func__);
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Display::DequeueWBSurfaceBuffer(const uint32_t surface_id,
      SurfaceBuffer &surface_buffer) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(&surface_buffer != NULL);

  auto ret = display_client_->DequeueWBSurfaceBuffer(surface_id,
      surface_buffer);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s: DequeueWBSurfaceBuffer failed!", __func__);
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Display::QueueWBSurfaceBuffer(const uint32_t surface_id,
      const SurfaceBuffer &surface_buffer) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(&surface_buffer != NULL);

  auto ret = display_client_->QueueWBSurfaceBuffer(surface_id,
      (SurfaceBuffer &)surface_buffer);
  if(NO_ERROR != ret) {
    QMMF_ERROR("%s: QueueWBSurfaceBuffer failed!", __func__);
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

}; //namespace display.

}; //namespace qmmf.
