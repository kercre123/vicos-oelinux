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

#define TAG "Recorder"

#include <binder/IPCThreadState.h>

#include "qmmf-sdk/qmmf_recorder.h"
#include "qmmf-sdk/qmmf_recorder_params.h"
#include "recorder/src/client/qmmf_recorder_client.h"
#include "recorder/src/service/qmmf_recorder_common.h"

namespace qmmf {

namespace recorder {

Recorder::Recorder()
  : recorder_client_(nullptr) {

  recorder_client_ = new RecorderClient();
  assert( recorder_client_ != NULL);
}

Recorder::~Recorder() {

  if (recorder_client_) {
    delete recorder_client_;
    recorder_client_ = nullptr;
  }
}

status_t Recorder::Connect(const RecorderCb& callback) {

  auto ret = recorder_client_->Connect(callback);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Init failed!", __func__);
  }
  return ret;
}

status_t Recorder::Disconnect() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  assert(recorder_client_ != nullptr);

  auto ret = recorder_client_->Disconnect();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s Disconnect failed!", TAG, __func__);
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t Recorder::StartCamera(const uint32_t camera_id,
                               const CameraStartParam &params,
                               const CameraResultCb &cb) {

  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->StartCamera(camera_id, params, cb);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: StartCamera failed!", __func__);
  }

  return ret;
}

status_t Recorder::StopCamera(const uint32_t camera_id) {

  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->StopCamera(camera_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: StopCamera failed!", __func__);
  }

  return ret;
}

status_t Recorder::CreateSession(const SessionCb& cb, uint32_t *session_id) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->CreateSession(cb, session_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: CreateSession failed!", __func__);
  }
  return ret;
}

status_t Recorder::DeleteSession(const uint32_t session_id) {

  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->DeleteSession(session_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: DeleteSession failed!", __func__);
  }

  return ret;
}

status_t Recorder::StartSession(const uint32_t session_id) {

  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->StartSession(session_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: StartSession failed!", __func__);
  }

  return ret;
}

status_t Recorder::StopSession(const uint32_t session_id, bool do_flush) {

  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->StopSession(session_id, do_flush);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: StopSession failed!", __func__);
  }

  return ret;
}

status_t Recorder::PauseSession(const uint32_t session_id) {

  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->PauseSession(session_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: PauseSession failed!", __func__);
  }

  return ret;
}

status_t Recorder::ResumeSession(const uint32_t session_id) {

  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->ResumeSession(session_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: ResumeSession failed!", __func__);
  }

  return ret;
}

status_t Recorder::GetSupportedPlugins(SupportedPlugins *plugins) {

  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->GetSupportedPlugins(plugins);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: GetSupportedPlugins failed!", __func__);
  }

  return ret;
}

status_t Recorder::CreatePlugin(uint32_t *uid, const PluginInfo &plugin) {

  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->CreatePlugin(uid, plugin);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: CreatePlugin failed!", __func__);
  }

  return ret;
}

status_t Recorder::DeletePlugin(const uint32_t &uid) {

  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->DeletePlugin(uid);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: DeletePlugin failed!", __func__);
  }

  return ret;
}

status_t Recorder::ConfigPlugin(const uint32_t &uid,
                                const std::string &json_config) {

  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->ConfigPlugin(uid, json_config);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: ConfigPlugin failed!", __func__);
  }

  return ret;
}

status_t Recorder::CreateAudioTrack(const uint32_t session_id,
                                    const uint32_t track_id,
                                    const AudioTrackCreateParam& params,
                                    const TrackCb& cb) {

  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->CreateAudioTrack(session_id, track_id,
                                                params, cb);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: CreateAudioTrack failed!", __func__);
  }

  return ret;
}

status_t Recorder::CreateVideoTrack(const uint32_t session_id,
                                    const uint32_t track_id,
                                    const VideoTrackCreateParam& param,
                                    const TrackCb& cb) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->CreateVideoTrack(session_id, track_id, param,
                                                cb);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: CreateVideoTrack failed!", __func__);
  }
  return ret;
}

status_t Recorder::CreateVideoTrack(const uint32_t session_id,
                                    const uint32_t track_id,
                                    const VideoTrackCreateParam& param,
                                    const VideoExtraParam& extra_param,
                                    const TrackCb& cb) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->CreateVideoTrack(session_id,track_id, param,
                                                extra_param, cb);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: CreateVideoTrackWithExtraParam failed!", __func__);
  }
  return ret;
}

status_t Recorder::ReturnTrackBuffer(const uint32_t session_id,
                                     const uint32_t track_id,
                                     std::vector<BufferDescriptor> &buffers) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->ReturnTrackBuffer(session_id, track_id, buffers);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: ReturnTrackBuffer failed!", __func__);
  }
  return ret;
}

status_t Recorder::SetAudioTrackParam(const uint32_t session_id,
                                      const uint32_t track_id,
                                      CodecParamType type,
                                      const void *params,
                                      size_t param_size) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->SetAudioTrackParam(session_id, track_id,
                                                  type, params, param_size);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: SetAudioTrackParam failed!", __func__);
  }
  return ret;
}

status_t Recorder::SetVideoTrackParam(const uint32_t session_id,
                                      const uint32_t track_id,
                                      CodecParamType type,
                                      const void *params,
                                      size_t param_size) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->SetVideoTrackParam(session_id, track_id,
                                                  type, params, param_size);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: SetVideoTrackParam failed!", __func__);
  }
  return ret;
}

status_t Recorder::DeleteAudioTrack(const uint32_t session_id,
                                    const uint32_t track_id) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->DeleteAudioTrack(session_id, track_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: DeleteAudioTrack failed!", __func__);
  }
  return ret;
}

status_t Recorder::DeleteVideoTrack(const uint32_t session_id,
                                    const uint32_t track_id) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->DeleteVideoTrack(session_id, track_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: DeleteVideoTrack failed!", __func__);
  }

  return ret;
}

status_t Recorder::CaptureImage(const uint32_t camera_id,
                                const ImageParam &param,
                                const uint32_t num_images,
                                const std::vector<CameraMetadata> &meta,
                                const ImageCaptureCb& cb) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->CaptureImage(camera_id, param, num_images,
                                            meta, cb);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: CaptureImage failed!", __func__);
  }
  return ret;
}

status_t Recorder::ConfigImageCapture(const uint32_t camera_id,
                                      const ImageConfigParam &config) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->ConfigImageCapture(camera_id, config);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: ConfigImageCapture failed!", __func__);
  }
  return ret;
}

status_t Recorder::CancelCaptureImage(const uint32_t camera_id) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->CancelCaptureImage(camera_id);
  if(NO_ERROR != ret) {
      QMMF_ERROR("%s: CancelCaptureImage failed!", __func__);
  }
  return ret;
}

status_t Recorder::ReturnImageCaptureBuffer(const uint32_t camera_id,
                                            const BufferDescriptor &buffer) {

  QMMF_DEBUG("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->ReturnImageCaptureBuffer(camera_id, buffer);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: ReturnImageCaptureBuffer failed!", __func__);
  }
  QMMF_DEBUG("%s: Exit" ,__func__);
  return ret;
}

status_t Recorder::SetCameraParam(const uint32_t camera_id,
                                  const CameraMetadata &meta) {

  QMMF_INFO("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->SetCameraParam(camera_id, meta);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s: SetCameraParam failed!", __func__);
  }
  QMMF_INFO("%s: Exit" ,__func__);
  return ret;
}

status_t Recorder::GetCameraParam(const uint32_t camera_id,
                                  CameraMetadata &meta) {

  QMMF_INFO("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->GetCameraParam(camera_id, meta);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s: GetCameraParam failed!", __func__);
  }

  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Recorder::GetDefaultCaptureParam(const uint32_t camera_id,
                                          CameraMetadata &meta) {

  QMMF_INFO("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->GetDefaultCaptureParam(camera_id, meta);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s: GetDefaultCaptureParam failed!", __func__);
  }

  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Recorder::CreateOverlayObject(const uint32_t track_id,
                                       const OverlayParam &param,
                                       uint32_t *overlay_id) {

  QMMF_INFO("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->CreateOverlayObject(track_id, param, overlay_id);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s: CreateOverlayObject failed!", __func__);
  }

  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Recorder::DeleteOverlayObject(const uint32_t track_id,
                                       const uint32_t overlay_id) {

  QMMF_INFO("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->DeleteOverlayObject(track_id, overlay_id);
  if (NO_ERROR != ret) {
      QMMF_ERROR("%s: DeleteOverlayObject failed!", __func__);
  }

  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Recorder::GetOverlayObjectParams(const uint32_t track_id,
                                          const uint32_t overlay_id,
                                          OverlayParam &param) {

  QMMF_DEBUG("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->GetOverlayObjectParams(track_id, overlay_id,
                                                      param);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: GetOverlayObjectParams failed!", __func__);
  }

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t Recorder::UpdateOverlayObjectParams(const uint32_t track_id,
                                             const uint32_t overlay_id,
                                             const OverlayParam &param) {

  QMMF_DEBUG("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->UpdateOverlayObjectParams(track_id, overlay_id,
                                                         param);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: UpdateOverlayObjectParams failed!", __func__);
  }

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t Recorder::SetOverlay(const uint32_t track_id,
                              const uint32_t overlay_id) {

  QMMF_INFO("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->SetOverlay(track_id, overlay_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: SetOverlay failed!", __func__);
  }

  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Recorder::RemoveOverlay(const uint32_t track_id,
                                 const uint32_t overlay_id) {
  QMMF_INFO("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->RemoveOverlay(track_id, overlay_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: RemoveOverlay failed!", __func__);
  }

  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Recorder::CreateMultiCamera(const std::vector<uint32_t> camera_ids,
                                     uint32_t *virtual_camera_id) {

  QMMF_INFO("%s: Enter" , __func__);
  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->CreateMultiCamera(camera_ids, virtual_camera_id);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: CreateMultiCamera failed!", __func__);
  }

  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Recorder::ConfigureMultiCamera(const uint32_t virtual_camera_id,
                                        const MultiCameraConfigType type,
                                        const void *param,
                                        const uint32_t param_size) {

  QMMF_INFO("%s: Enter" , __func__);
  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->ConfigureMultiCamera(virtual_camera_id, type,
                                                    param, param_size);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: ConfigureMultiCamera failed!", __func__);
  }

  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

}; //namespace recoder.

}; //namespace qmmf.
