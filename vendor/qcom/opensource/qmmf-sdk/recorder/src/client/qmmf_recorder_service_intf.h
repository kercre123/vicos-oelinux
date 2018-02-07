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

#include <iomanip>
#include <sstream>
#include <string>

#include <unistd.h>

#include <binder/IBinder.h>
#include <binder/IServiceManager.h>
#include <binder/Parcel.h>
#include <camera/CameraMetadata.h>

#include "qmmf-sdk/qmmf_recorder_params.h"
#include "qmmf-sdk/qmmf_recorder_extra_param.h"
#include "qmmf-sdk/qmmf_overlay.h"

namespace qmmf {
namespace recorder {

using namespace android;
using ::std::setbase;
using ::std::string;
using ::std::stringstream;
using namespace overlay;

#define QMMF_RECORDER_SERVICE_NAME "qmmf_recorder.service"

enum QMMF_RECORDER_SERVICE_CMDS {
  RECORDER_CONNECT = IBinder::FIRST_CALL_TRANSACTION,
  RECORDER_DISCONNECT,
  RECORDER_START_CAMERA,
  RECORDER_STOP_CAMERA,
  RECORDER_CREATE_SESSION,
  RECORDER_DELETE_SESSION,
  RECORDER_START_SESSION,
  RECORDER_STOP_SESSION,
  RECORDER_PAUSE_SESSION,
  RECORDER_RESUME_SESSION,
  RECORDER_GET_SUPPORTED_PLUGINS,
  RECORDER_CREATE_PLUGIN,
  RECORDER_DELETE_PLUGIN,
  RECORDER_CONFIGURE_PLUGIN,
  RECORDER_CREATE_AUDIOTRACK,
  RECORDER_CREATE_VIDEOTRACK,
  RECORDER_CREATE_VIDEOTRACK_EXTRAPARAMS,
  RECORDER_DELETE_AUDIOTRACK,
  RECORDER_DELETE_VIDEOTRACK,
  RECORDER_RETURN_TRACKBUFFER,
  RECORDER_SET_AUDIOTRACK_PARAMS,
  RECORDER_SET_VIDEOTRACK_PARAMS,
  RECORDER_CAPTURE_IMAGE,
  RECORDER_CONFIG_IMAGECAPTURE,
  RECORDER_CANCEL_IMAGECAPTURE,
  RECORDER_RETURN_IMAGECAPTURE_BUFFER,
  RECORDER_SET_CAMERA_PARAMS,
  RECORDER_GET_CAMERA_PARAMS,
  RECORDER_GET_DEFAULT_CAPTURE_PARAMS,
  RECORDER_CREATE_OVERLAYOBJECT,
  RECORDER_DELETE_OVERLAYOBJECT,
  RECORDER_GET_OVERLAYOBJECT_PARAMS,
  RECORDER_UPDATE_OVERLAYOBJECT_PARAMS,
  RECORDER_SET_OVERLAYOBJECT,
  RECORDER_REMOVE_OVERLAYOBJECT,
  RECORDER_CREATE_MULTICAMERA,
  RECORDER_CONFIGURE_MULTICAMERA,
};

struct BnBuffer {
  uint32_t  ion_fd;
  uint32_t  size;
  uint64_t  timestamp;
  uint32_t  width;
  uint32_t  height;
  uint32_t  buffer_id;
  uint32_t  flag;
  uint32_t  capacity;

  string ToString() const {
    stringstream stream;
    stream << "ion_fd[" << ion_fd << "] ";
    stream << "size[" << size << "] ";
    stream << "timestamp[" << timestamp << "] ";
    stream << "width[" << width << "] ";
    stream << "height[" << height << "] ";
    stream << "buffer_id[" << buffer_id << "] ";
    stream << "flag[" << setbase(16) << flag << setbase(10) << "] ";
    stream << "capacity[" << capacity << "]";
    return stream.str();
  }

  void ToParcel(Parcel* parcel, bool writeFileDescriptor) const {
    if (writeFileDescriptor)
      parcel->writeFileDescriptor(ion_fd);
    else
      parcel->writeUint32(ion_fd);
    parcel->writeUint32(size);
    parcel->writeInt64(timestamp);
    parcel->writeUint32(width);
    parcel->writeUint32(height);
    parcel->writeUint32(buffer_id);
    parcel->writeUint32(flag);
    parcel->writeUint32(capacity);
  }

  void FromParcel(const Parcel& parcel, bool readFileDescriptor) {
    if (readFileDescriptor)
      ion_fd = dup(parcel.readFileDescriptor());
    else
      ion_fd = parcel.readUint32();
    size = parcel.readUint32();
    timestamp = parcel.readInt64();
    width = parcel.readUint32();
    height = parcel.readUint32();
    buffer_id = parcel.readUint32();
    flag = parcel.readUint32();
    capacity = parcel.readUint32();
  }
};

class IRecorderServiceCallback;
class IRecorderService : public IInterface {
 public:
  DECLARE_META_INTERFACE(RecorderService);

  virtual status_t Connect(const sp<IRecorderServiceCallback>& service_cb,
                           uint32_t* client_id) = 0;

  virtual status_t Disconnect(const uint32_t client_id) = 0;

  virtual status_t StartCamera(const uint32_t client_id,
                               const uint32_t camera_id,
                               const CameraStartParam &param,
                               bool enable_result_cb = false) = 0;

  virtual status_t StopCamera(const uint32_t client_id,
                              const uint32_t camera_id) = 0;

  virtual status_t CreateSession(const uint32_t client_id,
                                 uint32_t *session_id) = 0;

  virtual status_t DeleteSession(const uint32_t client_id,
                                 const uint32_t session_id) = 0;

  virtual status_t StartSession(const uint32_t client_id,
                                const uint32_t session_id) = 0;

  virtual status_t StopSession(const uint32_t client_id,
                               const uint32_t session_id, bool do_flush) = 0;

  virtual status_t PauseSession(const uint32_t client_id,
                                const uint32_t session_id) = 0;

  virtual status_t ResumeSession(const uint32_t client_id,
                                 const uint32_t session_id) = 0;

  virtual status_t GetSupportedPlugins(const uint32_t client_id,
                                       SupportedPlugins *plugins) = 0;

  virtual status_t CreatePlugin(const uint32_t client_id, uint32_t *uid,
                                const PluginInfo &plugin) = 0;

  virtual status_t DeletePlugin(const uint32_t client_id,
                                const uint32_t &uid) = 0;

  virtual status_t ConfigPlugin(const uint32_t client_id, const uint32_t &uid,
                                const std::string &json_config) = 0;

  virtual status_t CreateAudioTrack(const uint32_t client_id,
                                    const uint32_t session_id,
                                    const uint32_t track_id,
                                    const AudioTrackCreateParam& param) = 0;

  virtual status_t CreateVideoTrack(const uint32_t client_id,
                                    const uint32_t session_id,
                                    const uint32_t track_id,
                                    const VideoTrackCreateParam& param) = 0;

  virtual status_t CreateVideoTrack(const uint32_t client_id,
                                    const uint32_t session_id,
                                    const uint32_t track_id,
                                    const VideoTrackCreateParam& param,
                                    const VideoExtraParam& extra_param) = 0;

  virtual status_t DeleteAudioTrack(const uint32_t client_id,
                                    const uint32_t session_id,
                                    const uint32_t track_id) = 0;

  virtual status_t DeleteVideoTrack(const uint32_t client_id,
                                    const uint32_t session_id,
                                    const uint32_t track_id) = 0;

  virtual status_t ReturnTrackBuffer(const uint32_t client_id,
                                     const uint32_t session_id,
                                     const uint32_t track_id,
                                     std::vector<BnBuffer> &buffers) = 0;

  virtual status_t SetAudioTrackParam(const uint32_t client_id,
                                      const uint32_t session_id,
                                      const uint32_t track_id,
                                      CodecParamType type,
                                      void *param,
                                      size_t param_size) = 0;

  virtual status_t SetVideoTrackParam(const uint32_t client_id,
                                      const uint32_t session_id,
                                      const uint32_t track_id,
                                      CodecParamType type,
                                      void *param,
                                      size_t param_size) = 0;

  virtual status_t CaptureImage(const uint32_t client_id,
                                const uint32_t camera_id,
                                const ImageParam &param,
                                const uint32_t num_images,
                                const std::vector<CameraMetadata> &meta) = 0;

  virtual status_t ConfigImageCapture(const uint32_t client_id,
                                      const uint32_t camera_id,
                                      const ImageConfigParam &config) = 0;

  virtual status_t CancelCaptureImage(const uint32_t client_id,
                                      const uint32_t camera_id) = 0;

  virtual status_t ReturnImageCaptureBuffer(const uint32_t client_id,
                                            const uint32_t camera_id,
                                            const int32_t buffer_id) = 0;

  virtual status_t SetCameraParam(const uint32_t client_id,
                                  const uint32_t camera_id,
                                  const CameraMetadata &meta) = 0;

  virtual status_t GetCameraParam(const uint32_t client_id,
                                  const uint32_t camera_id,
                                  CameraMetadata &meta) = 0;

  virtual status_t GetDefaultCaptureParam(const uint32_t client_id,
                                          const uint32_t camera_id,
                                          CameraMetadata &meta) = 0;

  virtual status_t CreateOverlayObject(const uint32_t client_id,
                                       const uint32_t track_id,
                                       OverlayParam *param,
                                       uint32_t *overlay_id) = 0;

  virtual status_t DeleteOverlayObject(const uint32_t client_id,
                                       const uint32_t track_id,
                                       const uint32_t overlay_id) = 0;

  virtual status_t GetOverlayObjectParams(const uint32_t client_id,
                                          const uint32_t track_id,
                                          const uint32_t overlay_id,
                                          OverlayParam &param) = 0;

  virtual status_t UpdateOverlayObjectParams(const uint32_t client_id,
                                             const uint32_t track_id,
                                             const uint32_t overlay_id,
                                             OverlayParam *param) = 0;

  virtual status_t SetOverlayObject(const uint32_t client_id,
                                    const uint32_t track_id,
                                    const uint32_t overlay_id) = 0;

  virtual status_t RemoveOverlayObject(const uint32_t client_id,
                                       const uint32_t track_id,
                                       const uint32_t overlay_id) = 0;

  virtual status_t CreateMultiCamera(const uint32_t client_id,
                                     const std::vector<uint32_t> camera_ids,
                                     uint32_t *virtual_camera_id) = 0;

  virtual status_t ConfigureMultiCamera(const uint32_t client_id,
                                        const uint32_t virtual_camera_id,
                                        const MultiCameraConfigType type,
                                        const void *param,
                                        const uint32_t param_size) = 0;
};

enum RECORDER_SERVICE_CB_CMDS{
  RECORDER_NOTIFY_EVENT=IBinder::FIRST_CALL_TRANSACTION,
  RECORDER_NOTIFY_SESSION_EVENT,
  RECORDER_NOTIFY_SNAPSHOT_DATA,
  RECORDER_NOTIFY_VIDEO_TRACK_DATA,
  RECORDER_NOTIFY_VIDEO_TRACK_EVENT,
  RECORDER_NOTIFY_AUDIO_TRACK_DATA,
  RECORDER_NOTIFY_AUDIO_TRACK_EVENT,
  RECORDER_NOTIFY_CAMERA_RESULT,
};

//Binder interface for callbacks from RecorderService to RecorderClient.
class IRecorderServiceCallback : public IInterface {
 public:
  DECLARE_META_INTERFACE(RecorderServiceCallback);

  virtual void NotifyRecorderEvent(EventType event_type, void *event_data,
                                   size_t event_data_size) = 0;

  virtual void NotifySessionEvent(EventType event_type, void *event_data,
                                  size_t event_data_size) = 0;

  virtual void NotifySnapshotData(uint32_t camera_id,
                                  uint32_t image_sequence_count,
                                  BnBuffer& buffer, MetaData& meta_data) = 0;

  virtual void NotifyVideoTrackData(uint32_t track_id,
                                    std::vector<BnBuffer>& buffers,
                                    std::vector<MetaData>& meta_buffers) = 0;

  virtual void NotifyVideoTrackEvent(uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) = 0;

  virtual void NotifyAudioTrackData(uint32_t track_id,
                                    const std::vector<BnBuffer>& buffers,
                                    const std::vector<MetaData>&
                                    meta_buffers) = 0;

  virtual void NotifyAudioTrackEvent(uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) = 0;

  virtual void NotifyCameraResult(uint32_t camera_id,
                                  const CameraMetadata &result) = 0;

  // This method is not exposed to client as a callback, it is just to update
  // Internal data structure, ServiceCallbackHandler is not forced to implement
  // this method.
  virtual void NotifyDeleteVideoTrack(uint32_t track_id
      __attribute__((__unused__))) {}
};

//This class is responsible to provide callbacks from recoder service.
class BnRecorderServiceCallback : public BnInterface<IRecorderServiceCallback> {
 public:
  virtual status_t onTransact(uint32_t code, const Parcel& data,
                              Parcel* reply, uint32_t flags = 0) override;
};

}; //namespace recorder

}; //namespace qmmf
