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

#pragma once

#include <utils/KeyedVector.h>
#include <utils/Log.h>
#include <libgralloc/gralloc_priv.h>

#include "recorder/src/service/qmmf_recorder_common.h"

namespace qmmf {

namespace recorder {

class IBufferConsumer;

class CameraInterface : public virtual RefBase {
 public:

  virtual ~CameraInterface() {};

  virtual status_t OpenCamera(const uint32_t camera_id,
                              const CameraStartParam &param,
                              const ResultCb &cb = nullptr,
                              const ErrorCb &errcb = nullptr) = 0;

  virtual status_t CloseCamera(const uint32_t camera_id) = 0;

  virtual status_t WaitAecToConverge(const uint32_t timeout) = 0;

  virtual status_t SetUpCapture(const ImageParam &param,
                                const uint32_t num_images) = 0;

  virtual status_t CaptureImage(const std::vector<CameraMetadata> &meta,
                                const StreamSnapshotCb& cb) = 0;

  virtual status_t ConfigImageCapture(const ImageConfigParam &config) = 0;

  virtual status_t CancelCaptureImage() = 0;

  virtual status_t CreateStream(const CameraStreamParam& param,
                                const VideoExtraParam& extra_param) = 0;

  virtual status_t DeleteStream(const uint32_t track_id) = 0;

  virtual status_t AddConsumer(const uint32_t& track_id,
                               sp<IBufferConsumer>& consumer) = 0;

  virtual status_t RemoveConsumer(const uint32_t& track_id,
                                  sp<IBufferConsumer>& consumer) = 0;

  virtual status_t StartStream(const uint32_t track_id) = 0;

  virtual status_t StopStream(const uint32_t track_id) = 0;

  virtual status_t SetCameraParam(const CameraMetadata &meta) = 0;

  virtual status_t GetCameraParam(CameraMetadata &meta) = 0;

  virtual status_t GetDefaultCaptureParam(CameraMetadata &meta) = 0;

  virtual status_t ReturnImageCaptureBuffer(const uint32_t camera_id,
                                            const int32_t buffer_id) = 0;

  virtual CameraStartParam& GetCameraStartParam() = 0;

  virtual Vector<int32_t>& GetSupportedFps() = 0;

};

}; //namespace recorder.

}; //namespace qmmf.
