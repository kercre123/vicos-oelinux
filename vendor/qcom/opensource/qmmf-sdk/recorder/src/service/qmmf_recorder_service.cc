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

#define TAG "RecorderService"

#include "recorder/src/client/qmmf_recorder_params_internal.h"
#include "recorder/src/service/qmmf_recorder_service.h"

namespace qmmf {

namespace recorder {

RecorderService::RecorderService()
    : recorder_(nullptr), unique_client_id_(0) {

  QMMF_KPI_GET_MASK();
  QMMF_INFO("%s:%s: RecorderService Instantiated! ", TAG, __func__);
  QMMF_KPI_DETAIL();
}

RecorderService::~RecorderService() {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  QMMF_INFO("%s:%s: Exit ", TAG, __func__);
  QMMF_KPI_DETAIL();
}

status_t RecorderService::onTransact(uint32_t code, const Parcel& data,
                                     Parcel* reply, uint32_t flag) {

  QMMF_DEBUG("%s:%s: Enter:(BnRecorderService::onTransact)", TAG, __func__);
  CHECK_INTERFACE(IRecorderService, data, reply);
  int32_t ret = 0;

  switch (code) {
    case RECORDER_CONNECT: {
      sp<IRecorderServiceCallback> client_cb_handle = interface_cast
          <IRecorderServiceCallback>(data.readStrongBinder());
      uint32_t client_id;
      ret = Connect(client_cb_handle, &client_id);
      reply->writeUint32(client_id);
      reply->writeInt32(ret);
      return NO_ERROR;
    }
    break;
      case RECORDER_DISCONNECT: {
        uint32_t client_id;
        data.readUint32(&client_id);
        ret = Disconnect(client_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_START_CAMERA: {
        uint32_t camera_id, enable_flag;
        bool enable_result_cb;
        uint32_t client_id;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        data.readUint32(&enable_flag);
        enable_result_cb = (1 == enable_flag) ? true : false;
        uint32_t blob_size;
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        void* params = const_cast<void*>(blob.data());
        CameraStartParam camera_start_params;
        memset(&camera_start_params, 0x0, sizeof camera_start_params);
        memcpy(&camera_start_params, params, blob_size);
        ret = StartCamera(client_id, camera_id, camera_start_params,
                          enable_result_cb);
        blob.release();
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_STOP_CAMERA: {
        uint32_t client_id, camera_id;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        ret = StopCamera(client_id, camera_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_CREATE_SESSION: {
        uint32_t client_id, session_id;
        data.readUint32(&client_id);
        ret = CreateSession(client_id, &session_id);
        reply->writeUint32(session_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_DELETE_SESSION: {
        uint32_t client_id, session_id;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        ret = DeleteSession(client_id, session_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_START_SESSION: {
        uint32_t client_id, session_id;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        ret = StartSession(client_id, session_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_STOP_SESSION: {
        uint32_t client_id, session_id;
        int32_t flush;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        data.readInt32(&flush);
        ret = StopSession(client_id, session_id, flush);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_PAUSE_SESSION: {
        uint32_t client_id, session_id;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        ret = PauseSession(client_id, session_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_RESUME_SESSION: {
        uint32_t client_id, session_id;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        ret = ResumeSession(client_id, session_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_GET_SUPPORTED_PLUGINS: {
        uint32_t client_id;
        data.readUint32(&client_id);
        SupportedPlugins plugins;
        auto ret = GetSupportedPlugins(client_id, &plugins);
        uint32_t num_plugins = plugins.size();
        reply->writeUint32(num_plugins);
        for (auto const& plugin : plugins) {
          size_t blob_size = plugin.Size();
          reply->writeUint32(blob_size);
          android::Parcel::WritableBlob blob;
          reply->writeBlob(blob_size, false, &blob);
          memset(blob.data(), 0x0, blob_size);
          memcpy(blob.data(), plugin.ToBlob().get(), blob_size);
        }
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_CREATE_PLUGIN: {
        uint32_t client_id;
        data.readUint32(&client_id);
        uint32_t blob_size;
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        PluginInfo plugin(blob.data(), blob_size);
        uint32_t uid;
        ret = CreatePlugin(client_id, &uid, plugin);
        reply->writeInt32(ret);
        reply->writeUint32(uid);
        return NO_ERROR;
      }
      break;
      case RECORDER_DELETE_PLUGIN: {
        uint32_t client_id, uid;
        data.readUint32(&client_id);
        data.readUint32(&uid);
        ret = DeletePlugin(client_id, uid);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_CONFIGURE_PLUGIN: {
        uint32_t client_id, uid;
        data.readUint32(&client_id);
        data.readUint32(&uid);
        uint32_t blob_size;
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        const char *string = reinterpret_cast<const char *>(blob.data());
        std::string json_config(string, blob_size);
        ret = ConfigPlugin(client_id, uid, json_config);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_CREATE_AUDIOTRACK: {
        uint32_t client_id, session_id, track_id;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        data.readUint32(&track_id);
        AudioTrackCreateParamInternal params;
        params.FromParcel(data);
        ret = CreateAudioTrack(client_id, session_id, track_id, params);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_CREATE_VIDEOTRACK: {
        uint32_t client_id, session_id, track_id;
        uint32_t blob_size;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        data.readUint32(&track_id);
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        void* params = const_cast<void*>(blob.data());
        VideoTrackCreateParam video_track_param;
        memset(&video_track_param, 0x0, sizeof video_track_param);
        memcpy(&video_track_param, params, blob_size);
        ret = CreateVideoTrack(client_id, session_id, track_id,
                               video_track_param);
        blob.release();
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_CREATE_VIDEOTRACK_EXTRAPARAMS: {
        uint32_t client_id, session_id, track_id;
        uint32_t blob_size, extra_blob_size;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        data.readUint32(&track_id);
        android::Parcel::ReadableBlob blob;
        data.readUint32(&blob_size);
        data.readBlob(blob_size, &blob);
        android::Parcel::ReadableBlob extra_blob;
        data.readUint32(&extra_blob_size);
        data.readBlob(extra_blob_size, &extra_blob);
        VideoTrackCreateParam video_track_param;
        memset(&video_track_param, 0x0, sizeof video_track_param);
        memcpy(&video_track_param, blob.data(), blob_size);
        VideoExtraParam extra_param(extra_blob.data(), extra_blob_size);
        ret = CreateVideoTrack(client_id, session_id, track_id,
                               video_track_param, extra_param);
        blob.release();
        extra_blob.release();
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_DELETE_AUDIOTRACK: {
        uint32_t client_id, session_id, track_id;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        data.readUint32(&track_id);
        ret = DeleteAudioTrack(client_id, session_id, track_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_DELETE_VIDEOTRACK: {
        uint32_t client_id, session_id, track_id;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        data.readUint32(&track_id);
        ret = DeleteVideoTrack(client_id, session_id, track_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_RETURN_TRACKBUFFER: {
        uint32_t client_id, session_id, track_id;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        data.readUint32(&track_id);

        std::vector<BnBuffer> buffers;
        if (track_id < 100) {
          uint32_t vector_size;
          data.readUint32(&vector_size);
          for (uint32_t i = 0; i < vector_size; i++)  {
            uint32_t size;
            data.readUint32(&size);
            android::Parcel::ReadableBlob blob;
            data.readBlob(size, &blob);
            void* buffer = const_cast<void*>(blob.data());
            BnBuffer track_buffer;
            assert(size == sizeof(track_buffer));
            memset(&track_buffer, 0x0, sizeof track_buffer);
            memcpy(&track_buffer, buffer, size);
            buffers.push_back(track_buffer);
            blob.release();
          }
        } else {
          size_t num_buffers = data.readInt32();
          for (size_t index = 0; index < num_buffers; ++index) {
            BnBuffer buffer;
            buffer.FromParcel(data, false);
            buffers.push_back(buffer);
          }
        }
        ret = ReturnTrackBuffer(client_id, session_id, track_id, buffers);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_SET_AUDIOTRACK_PARAMS: {
        uint32_t client_id, session_id, track_id;
        uint32_t param_type, blob_size;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        data.readUint32(&track_id);
        data.readUint32(&param_type);
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        void* param = const_cast<void*>(blob.data());
        ret = SetAudioTrackParam(client_id, session_id, track_id,
                                 static_cast<CodecParamType>(param_type),
                                 param, blob_size);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_SET_VIDEOTRACK_PARAMS: {
        uint32_t client_id, session_id, track_id;
        uint32_t param_type, blob_size;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        data.readUint32(&track_id);
        data.readUint32(&param_type);
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        void* param = const_cast<void*>(blob.data());
        ret = SetVideoTrackParam(client_id, session_id, track_id,
                                 static_cast<CodecParamType>(param_type),
                                 param, blob_size);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_CAPTURE_IMAGE: {
        uint32_t client_id, camera_id, blob_size, num_images, meta_size;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        ImageParam param;
        memcpy(&param, blob.data(), blob_size);
        data.readUint32(&num_images);
        data.readUint32(&meta_size);
        std::vector<CameraMetadata> meta_array;
        for (uint32_t i = 0; i < meta_size; ++i) {
          CameraMetadata meta;
          camera_metadata_t *m = nullptr;
          ret = meta.readFromParcel(data, &m);
          if ((NO_ERROR != ret) || (nullptr == m)) {
            QMMF_ERROR("%s: Metadata parcel read failed: %d meta(%p)",
                __func__, ret, m);
            reply->writeInt32(ret);
            return ret;
          }
          meta.clear();
          meta.append(m);
          meta_array.push_back(meta);
          //We need to release this memory as meta.append() makes copy of this memory
          free(m);
        }
        ret = CaptureImage(client_id, camera_id, param,
                           num_images, meta_array);

        // Clear the metadata buffers and free all storage used by it
        for (auto meta:meta_array) {
          meta.clear();
        }

        reply->writeInt32(ret);
        return ret;
      }
      break;
      case RECORDER_CONFIG_IMAGECAPTURE: {
        uint32_t client_id, camera_id, blob_size;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        ImageConfigParam config(blob.data(), blob_size);
        ret = ConfigImageCapture(client_id, camera_id, config);
        reply->writeInt32(ret);
        return ret;
      }
      break;
      case RECORDER_CANCEL_IMAGECAPTURE: {
        uint32_t client_id, camera_id;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        ret = CancelCaptureImage(client_id, camera_id);
        reply->writeInt32(ret);
        return ret;
      }
      break;
      case  RECORDER_RETURN_IMAGECAPTURE_BUFFER: {
        uint32_t client_id, camera_id, buffer_id;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        data.readUint32(&buffer_id);
        ret = ReturnImageCaptureBuffer(client_id, camera_id, buffer_id);
        return ret;
      }
      break;
      case RECORDER_SET_CAMERA_PARAMS: {
        uint32_t client_id, camera_id;
        data.readUint32(&client_id);
        CameraMetadata meta;
        camera_metadata_t *m = nullptr;
        data.readUint32(&camera_id);
        ret = meta.readFromParcel(data, &m);
        if ((NO_ERROR != ret) || (nullptr == m)) {
          QMMF_ERROR("%s: Metadata parcel read failed: %d meta: %p\n",
              __func__, ret, m);
          reply->writeInt32(ret);
          return ret;
        }
        meta.clear();
        meta.append(m);
        ret = SetCameraParam(client_id, camera_id, meta);

        // Clear the metadata buffer and free all storage used by it
        meta.clear();
        //We need to release this memory as meta.append() makes copy of this memory
        free(m);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_GET_CAMERA_PARAMS: {
        uint32_t client_id, camera_id;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        CameraMetadata meta;
        ret = GetCameraParam(client_id, camera_id, meta);
        reply->writeInt32(ret);
        if (NO_ERROR == ret) {
          ret = meta.writeToParcel(reply);
          if (NO_ERROR != ret) {
            QMMF_ERROR("%s: Metadata parcel write failed: %d\n",
                       __func__, ret);
          }
        }
        return ret;
      }
      break;
      case RECORDER_GET_DEFAULT_CAPTURE_PARAMS: {
        uint32_t client_id, camera_id;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        CameraMetadata meta;
        ret = GetDefaultCaptureParam(client_id, camera_id, meta);
        reply->writeInt32(ret);
        if (NO_ERROR == ret) {
          ret = meta.writeToParcel(reply);
          if (NO_ERROR != ret) {
            QMMF_ERROR("%s: Metadata parcel write failed: %d\n",
                       __func__, ret);
          }
        }
        return ret;
      }
      break;
      case RECORDER_CREATE_OVERLAYOBJECT: {
        uint32_t client_id, blob_size, track_id;
        android::Parcel::ReadableBlob image_blob;
        uint32_t image_size;

        data.readUint32(&client_id);
        data.readUint32(&track_id);

        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        void* params = const_cast<void*>(blob.data());

        OverlayParam  overlay_params;
        memset(&overlay_params, 0x0, sizeof(OverlayParam));
        memcpy(&overlay_params, static_cast<OverlayParam*>(params),
            sizeof(OverlayParam));

        if (overlay_params.type ==  OverlayType::kStaticImage &&
            overlay_params.image_info.image_type == OverlayImageType::kBlobType) {
          data.readUint32(&image_size);
          data.readBlob(image_size, &image_blob);
          overlay_params.image_info.image_size = image_size;
          overlay_params.image_info.image_buffer =
              reinterpret_cast<char*>(const_cast<void*>(image_blob.data()));
        }

        uint32_t overlay_id;
        ret = CreateOverlayObject(client_id, track_id, &overlay_params,
                                  &overlay_id);
        blob.release();
        image_blob.release();
        reply->writeUint32(overlay_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_DELETE_OVERLAYOBJECT: {
        uint32_t client_id, overlay_id, track_id;
        data.readUint32(&client_id);
        data.readUint32(&track_id);
        data.readUint32(&overlay_id);
        ret = DeleteOverlayObject(client_id, track_id, overlay_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_GET_OVERLAYOBJECT_PARAMS: {
        uint32_t client_id, overlay_id, track_id;
        data.readUint32(&client_id);
        data.readUint32(&track_id);
        data.readUint32(&overlay_id);
        OverlayParam overlay_param;
        memset(&overlay_param, 0x0, sizeof overlay_param);
        ret = GetOverlayObjectParams(client_id, track_id, overlay_id,
                                     overlay_param);
        reply->writeInt32(ret);
        if (NO_ERROR == ret) {
          uint32_t param_size = sizeof overlay_param;
          reply->writeUint32(param_size);
          android::Parcel::WritableBlob blob;
          reply->writeBlob(param_size, false, &blob);
          memset(blob.data(), 0x0, param_size);
          memcpy(blob.data(), reinterpret_cast<void*>(&overlay_param),
              sizeof overlay_param);
        }
        return NO_ERROR;
      }
      break;
      case RECORDER_UPDATE_OVERLAYOBJECT_PARAMS: {
        uint32_t client_id, track_id, overlay_id, blob_size;
        android::Parcel::ReadableBlob image_blob;
        uint32_t image_size;

        data.readUint32(&client_id);
        data.readUint32(&track_id);
        data.readUint32(&overlay_id);
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        void* params = const_cast<void*>(blob.data());

        OverlayParam  overlay_params;
        memset(&overlay_params, 0x0, sizeof(OverlayParam));
        memcpy(&overlay_params, static_cast<OverlayParam*>(params),
            sizeof(OverlayParam));

        if (overlay_params.type ==  OverlayType::kStaticImage &&
            overlay_params.image_info.image_type == OverlayImageType::kBlobType
            && overlay_params.image_info.buffer_updated == true) {
          data.readUint32(&image_size);
          data.readBlob(image_size, &image_blob);
          overlay_params.image_info.image_size = image_size;
          overlay_params.image_info.image_buffer =
              reinterpret_cast<char*>(const_cast<void*>(image_blob.data()));
        }

        ret = UpdateOverlayObjectParams(client_id, track_id, overlay_id,
                                        &overlay_params);
        blob.release();
        image_blob.release();
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_SET_OVERLAYOBJECT: {
        uint32_t client_id, track_id, overlay_id;
        data.readUint32(&client_id);
        data.readUint32(&track_id);
        data.readUint32(&overlay_id);
        ret = SetOverlayObject(client_id, track_id, overlay_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_REMOVE_OVERLAYOBJECT: {
        uint32_t client_id, track_id, overlay_id;
        data.readUint32(&client_id);
        data.readUint32(&track_id);
        data.readUint32(&overlay_id);
        ret = RemoveOverlayObject(client_id, track_id, overlay_id);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_CREATE_MULTICAMERA: {
        uint32_t client_id, vector_size;
        data.readUint32(&client_id);
        data.readUint32(&vector_size);
        std::vector<uint32_t> camera_ids;
        for (uint32_t i = 0; i < vector_size; ++i) {
          camera_ids.push_back(data.readUint32());
        }
        uint32_t virtual_camera_id;
        ret = CreateMultiCamera(client_id, camera_ids, &virtual_camera_id);
        QMMF_INFO("%s:%s: virtual_camera_id=%d", TAG, __func__,
            virtual_camera_id);
        reply->writeUint32(virtual_camera_id);
        reply->writeInt32(ret);
      }
      break;
      case RECORDER_CONFIGURE_MULTICAMERA: {
        uint32_t client_id, virtual_camera_id, config_type, param_size;
        data.readUint32(&client_id);
        data.readUint32(&virtual_camera_id);
        data.readUint32(&config_type);
        data.readUint32(&param_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(param_size, &blob);
        void* param = const_cast<void*>(blob.data());
        ret = ConfigureMultiCamera(client_id, virtual_camera_id,
            static_cast<MultiCameraConfigType>(config_type), param, param_size);
        blob.release();
        reply->writeUint32(ret);
        return NO_ERROR;
      }
      break;
      default: {
        QMMF_ERROR("RecorderService:%s:Method is not supported !",__func__);
        reply->writeInt32(-1);
      }
      break;
  }
  return NO_ERROR;
}

status_t RecorderService::Connect(const sp<IRecorderServiceCallback>&
                                  service_cb, uint32_t* client_id) {
  QMMF_DEBUG("%s:%s: Enter ", TAG, __func__);
  QMMF_KPI_DETAIL();

  std::lock_guard<std::mutex> lock(lock_);
  status_t ret = NO_ERROR;

  if (!recorder_) {
    recorder_ = RecorderImpl::CreateRecorder();
    if (!recorder_) {
        QMMF_ERROR("%s:%s: Can't create Recorder Instance!!", TAG, __func__);
        return NO_MEMORY;
    }
    std::function< const sp<RemoteCallBack>& (uint32_t id)>
      remote_cb_handle = [&] (uint32_t id) {
        QMMF_VERBOSE("%s:%s: Remote Callback request for client(%d)", TAG,
            __func__, id);
        ssize_t idx = remote_cb_list_.indexOfKey(id);
        assert(idx >= 0);
        return remote_cb_list_.valueFor(id);
    };
    ret = recorder_->Init(remote_cb_handle);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: Recorder Initialization failed!", TAG, __func__);
      delete recorder_;
      recorder_ = nullptr;
      return ret;
    }
  }

  ++unique_client_id_;
  *client_id = unique_client_id_;

  sp<RemoteCallBack> remote_callback;
  remote_callback = new RemoteCallBack(*client_id, service_cb);
  if (!remote_callback.get()) {
      QMMF_ERROR("%s:%s: Unable to allocate remote callback!", TAG, __func__);
      return NO_INIT;
  }
  remote_cb_list_.add(*client_id, remote_callback);

  sp<DeathNotifier> death_notifier;
  death_notifier = new DeathNotifier();
  if (!death_notifier.get()) {
      QMMF_ERROR("%s:%s: Unable to allocate death notifier!", TAG, __func__);
    remote_cb_list_.removeItem(*client_id);
    return NO_INIT;
  }
  NotifyClientDeath notify_death = [this, capture_client_id = *client_id] {
      ClientDeathHandler(capture_client_id);
  };
  death_notifier->SetDeathNotifyCB(notify_death);

  // Link death notifier to remote handle.
  IInterface::asBinder(remote_callback->getRemoteClient())
      ->linkToDeath(death_notifier);

  death_notifier_list_.add(*client_id, death_notifier);

  recorder_->RegisterClient(*client_id);

  QMMF_INFO("%s:%s:Service is connected with client (%d)", TAG, __func__,
      *client_id);

  QMMF_DEBUG("%s:%s: Exit client_id(%d)", TAG, __func__, *client_id);
  return ret;
}

status_t RecorderService::Disconnect(uint32_t client_id) {

  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);
  QMMF_KPI_DETAIL();
  std::lock_guard<std::mutex> lock(lock_);

  int32_t ret = NO_ERROR;
  ssize_t idx = death_notifier_list_.indexOfKey(client_id);
  if (idx < 0) {
    QMMF_ERROR("%s:%s: Client doesn't exist! Wrong id", TAG, __func__);
    return BAD_VALUE;
  }

  sp<DeathNotifier> death_notifier = death_notifier_list_.valueFor(client_id);
  assert(death_notifier.get() != nullptr);

  sp<RemoteCallBack> remote_callback = remote_cb_list_.valueFor(client_id);
  assert(remote_callback.get() != nullptr);

  IInterface::asBinder(remote_callback->getRemoteClient())
      ->unlinkToDeath(death_notifier);

  death_notifier_list_.removeItem(client_id);

  remote_cb_list_.removeItem(client_id);

  assert(recorder_ != nullptr);
  recorder_->DeRegisterClient(client_id);

  if ( (death_notifier_list_.size() == 0) &&
       (remote_cb_list_.size() == 0) ) {
    QMMF_INFO("%s:%s: No client is connected! de-initialize the recorder!", TAG,
        __func__);
    recorder_->DeInit();
    delete recorder_;
    recorder_ = nullptr;
    unique_client_id_ = 0;
  }

  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::StartCamera(const uint32_t client_id,
                                      const uint32_t camera_id,
                                      const CameraStartParam &params,
                                      bool enable_result_cb) {

  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->StartCamera(client_id, camera_id, params,
                                    enable_result_cb);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: Can't start Camera!!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return NO_ERROR;
}

status_t RecorderService::StopCamera(const uint32_t client_id,
                                     const uint32_t camera_id) {

  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->StopCamera(client_id, camera_id);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: Can't Stop Camera!!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return NO_ERROR;
}

status_t RecorderService::CreateSession(const uint32_t client_id,
                                        uint32_t *session_id) {

  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  uint32_t id;
  auto ret = recorder_->CreateSession(client_id, &id);
  assert(ret == NO_ERROR);
  *session_id = id;

  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::DeleteSession(const uint32_t client_id,
                                        const uint32_t session_id) {

  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->DeleteSession(client_id, session_id);
  assert(ret == NO_ERROR);
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::StartSession(const uint32_t client_id,
                                       const uint32_t session_id) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  QMMF_INFO("%s:%s: Session_id(%d) to be Start", TAG, __func__, session_id);

  assert(recorder_ != nullptr);
  auto ret = recorder_->StartSession(client_id, session_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: StartSession failed!", TAG, __func__);
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::StopSession(const uint32_t client_id,
                                      const uint32_t session_id,
                                      bool do_flush) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  QMMF_INFO("%s:%s: Session_id(%d) to be Stop with flash=%d", TAG, __func__,
                                      session_id, do_flush);

  assert(recorder_ != nullptr);
  auto ret = recorder_->StopSession(client_id, session_id, do_flush);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: StopSession failed!", TAG, __func__);
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::PauseSession(const uint32_t client_id,
                                       const uint32_t session_id) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  QMMF_INFO("%s:%s: Session_id(%d) to be Pause", TAG, __func__, session_id);

  assert(recorder_ != nullptr);
  auto ret = recorder_->PauseSession(client_id, session_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: PauseSession failed!", TAG, __func__);
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::ResumeSession(const uint32_t client_id,
                                        const uint32_t session_id) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);
  QMMF_KPI_DETAIL();
  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  QMMF_INFO("%s:%s: Session_id(%d) to be Resume", TAG, __func__, session_id);

  assert(recorder_ != nullptr);
  auto ret = recorder_->ResumeSession(client_id, session_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: ResumeSession failed!", TAG, __func__);
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);

  return ret;
}

status_t RecorderService::GetSupportedPlugins(const uint32_t client_id,
                                              SupportedPlugins *plugins) {

  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }

  assert(recorder_ != nullptr);
  auto ret = recorder_->GetSupportedPlugins(client_id, plugins);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: GetSupportedPlugins failed: %d", TAG, __func__, ret);
    return ret;
  }

  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return NO_ERROR;
}

status_t RecorderService::CreatePlugin(const uint32_t client_id, uint32_t *uid,
                                       const PluginInfo &plugin) {

  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }

  assert(recorder_ != nullptr);
  auto ret = recorder_->CreatePlugin(client_id, uid, plugin);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreatePlugin %s failed: %d", TAG, __func__,
        plugin.name.c_str(), ret);
    return ret;
  }

  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return NO_ERROR;
}

status_t RecorderService::DeletePlugin(const uint32_t client_id,
                                       const uint32_t &uid) {

  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }

  assert(recorder_ != nullptr);
  auto ret = recorder_->DeletePlugin(client_id, uid);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: DeletePlugin uid(%d) failed: %d", TAG, __func__,
        uid, ret);
    return ret;
  }

  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return NO_ERROR;
}

status_t RecorderService::ConfigPlugin(const uint32_t client_id,
                                       const uint32_t &uid,
                                       const std::string &json_config) {

  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_WARN("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }

  assert(recorder_ != nullptr);
  auto ret = recorder_->ConfigPlugin(client_id, uid, json_config);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: ConfigPlugin uid(%d) failed: %d", TAG, __func__,
        uid, ret);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return NO_ERROR;
}

status_t RecorderService::CreateAudioTrack(const uint32_t client_id,
                                           const uint32_t session_id,
                                           const uint32_t track_id,
                                           const AudioTrackCreateParam& param) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  uint32_t id = track_id & 0xffff0000;
  if (id > 0) {
    QMMF_INFO("%s:%s: track_id should be 16 bit number!", TAG, __func__);
    return BAD_VALUE;
  }

  assert(recorder_ != nullptr);
  auto ret = recorder_->CreateAudioTrack(client_id, session_id, track_id,
                                         param);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: CreateAudioTrack failed: %d", TAG, __func__, ret);
    return BAD_VALUE;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return NO_ERROR;
}

status_t RecorderService::CreateVideoTrack(const uint32_t client_id,
                                           const uint32_t session_id,
                                           const uint32_t track_id,
                                           const VideoTrackCreateParam& param) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);
  QMMF_KPI_DETAIL();
  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  uint32_t id = track_id & 0xffff0000;
  if (id > 0) {
    QMMF_INFO("%s:%s: track_id should be 16 bit number!", TAG, __func__);
    return BAD_VALUE;
  }

  assert(recorder_ != nullptr);
  auto ret = recorder_->CreateVideoTrack(client_id, session_id, track_id,
                                         param);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: CreateVideoTrack failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::CreateVideoTrack(const uint32_t client_id,
                                           const uint32_t session_id,
                                           const uint32_t track_id,
                                           const VideoTrackCreateParam& param,
                                           const VideoExtraParam& extra_param) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  uint32_t id = track_id & 0xffff0000;
  if (id > 0) {
    QMMF_INFO("%s:%s: track_id should be 16 bit number!", TAG, __func__);
    return BAD_VALUE;
  }

  assert(recorder_ != nullptr);
  auto ret = recorder_->CreateVideoTrack(client_id, session_id, track_id,
                                         param, extra_param);

  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: CreateVideoTrackWithExtraParam failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::DeleteAudioTrack(const uint32_t client_id,
                                           const uint32_t session_id,
                                           const uint32_t track_id) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->DeleteAudioTrack(client_id, session_id, track_id);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: DeleteAudioTrack failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return NO_ERROR;
}

status_t RecorderService::DeleteVideoTrack(const uint32_t client_id,
                                           const uint32_t session_id,
                                           const uint32_t track_id) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->DeleteVideoTrack(client_id, session_id, track_id);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: DeleteVideoTrack failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::ReturnTrackBuffer(const uint32_t client_id,
                                            const uint32_t session_id,
                                            const uint32_t track_id,
                                            std::vector<BnBuffer> &buffers) {
  QMMF_VERBOSE("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->ReturnTrackBuffer(client_id, session_id, track_id,
                                          buffers);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: ReturnTrackBuffer failed!", TAG, __func__);
    return BAD_VALUE;
  }
  QMMF_VERBOSE("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::SetAudioTrackParam(const uint32_t client_id,
                                             const uint32_t session_id,
                                             const uint32_t track_id,
                                             CodecParamType type,
                                             void *param,
                                             size_t param_size) {
  // NOT IMPLEMENTED YET.
  return NO_ERROR;
}

status_t RecorderService::SetVideoTrackParam(const uint32_t client_id,
                                             const uint32_t session_id,
                                             const uint32_t track_id,
                                             CodecParamType type,
                                             void *param,
                                             size_t param_size) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->SetVideoTrackParam(client_id, session_id, track_id,
                                           type, param, param_size);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CaptureImage failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::CaptureImage(const uint32_t client_id,
                                       const uint32_t camera_id,
                                       const ImageParam &param,
                                       const uint32_t num_images, const
                                       std::vector<CameraMetadata> &meta) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->CaptureImage(client_id, camera_id, param,
                                     num_images, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CaptureImage failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::ConfigImageCapture(const uint32_t client_id,
                                             const uint32_t camera_id,
                                             const ImageConfigParam &config) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->ConfigImageCapture(client_id, camera_id, config);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: ConfigImageCapture failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return NO_ERROR;
}

status_t RecorderService::CancelCaptureImage(const uint32_t client_id,
                                             const uint32_t camera_id) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != NULL);
  auto ret = recorder_->CancelCaptureImage(client_id, camera_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CancelCaptureImage failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return NO_ERROR;
}


status_t RecorderService::ReturnImageCaptureBuffer(const uint32_t client_id,
                                                   const uint32_t camera_id,
                                                   const int32_t buffer_id) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->ReturnImageCaptureBuffer(client_id, camera_id,
                                                 buffer_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: ReturnImageCaptureBuffer failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::SetCameraParam(const uint32_t client_id,
                                         const uint32_t camera_id,
                                         const CameraMetadata &meta) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->SetCameraParam(client_id, camera_id, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: SetCameraParam failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::GetCameraParam(const uint32_t client_id,
                                         const uint32_t camera_id,
                                         CameraMetadata &meta) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->GetCameraParam(client_id, camera_id, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: GetCameraParam failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::GetDefaultCaptureParam(const uint32_t client_id,
                                                const uint32_t camera_id,
                                                 CameraMetadata &meta) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->GetDefaultCaptureParam(client_id, camera_id, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: GetDefaultCaptureParam failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::CreateOverlayObject(const uint32_t client_id,
                                              const uint32_t track_id,
                                              OverlayParam *param,
                                              uint32_t *overlay_id) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->CreateOverlayObject(client_id, track_id, param,
                                            overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreateOverlayObject failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::DeleteOverlayObject(const uint32_t client_id,
                                              const uint32_t track_id,
                                              const uint32_t overlay_id) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->DeleteOverlayObject(client_id, track_id, overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: DeleteOverlayObject failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::GetOverlayObjectParams(const uint32_t client_id,
                                                 const uint32_t track_id,
                                                 const uint32_t overlay_id,
                                                 OverlayParam &param) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->GetOverlayObjectParams(client_id, track_id, overlay_id,
                                               param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: GetOverlayObjectParams failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::UpdateOverlayObjectParams(const uint32_t client_id,
                                                    const uint32_t track_id,
                                                    const uint32_t overlay_id,
                                                    OverlayParam *param) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->UpdateOverlayObjectParams(client_id, track_id,
                                                  overlay_id, param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: UpdateOverlayObjectParams failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::SetOverlayObject(const uint32_t client_id,
                                           const uint32_t track_id,
                                           const uint32_t overlay_id) {

  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->SetOverlayObject(client_id, track_id, overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: SetOverlayObject failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::RemoveOverlayObject(const uint32_t client_id,
                                              const uint32_t track_id,
                                              const uint32_t overlay_id) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->RemoveOverlayObject(client_id, track_id, overlay_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: RemoveOverlayObject failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::CreateMultiCamera(const uint32_t client_id,
                                            const std::vector<uint32_t>
                                            camera_ids,
                                            uint32_t *virtual_camera_id) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->CreateMultiCamera(client_id, camera_ids,
                                          virtual_camera_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: CreateMultiCamera failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

status_t RecorderService::ConfigureMultiCamera(const uint32_t client_id,
                                               const uint32_t virtual_camera_id,
                                               const MultiCameraConfigType type,
                                               const void *param,
                                               const uint32_t param_size) {
  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s:%s: Client (%d) is not valid!", TAG, __func__, client_id);
    return BAD_VALUE;
  }
  assert(recorder_ != nullptr);
  auto ret = recorder_->ConfigureMultiCamera(client_id, virtual_camera_id, type,
                                             param, param_size);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: ConfigureMultiCamera failed!", TAG, __func__);
    return ret;
  }
  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

void RecorderService::ClientDeathHandler(const uint32_t client_id) {
  QMMF_INFO("%s:%s: client_id(%d) died in battle!", TAG, __func__, client_id);
  // Internal disconnect, it would trigger resource cleanup belongs to died
  // client.
  DisconnectInternal(client_id);
}

bool RecorderService::IsClientValid(const uint32_t client_id) {

  std::lock_guard<std::mutex> lock(lock_);
  ssize_t idx = remote_cb_list_.indexOfKey(client_id);
  return idx < 0 ? false : true;
}

status_t RecorderService::DisconnectInternal(const uint32_t client_id) {

  QMMF_INFO("%s:%s: Enter client_id(%d)", TAG, __func__, client_id);
  std::lock_guard<std::mutex> lock(lock_);

  int32_t ret = NO_ERROR;
  ssize_t idx = death_notifier_list_.indexOfKey(client_id);
  if (idx < 0) {
    QMMF_ERROR("%s:%s: Client doesn't exist! Wrong id", TAG, __func__);
    return BAD_VALUE;
  }
  // Forceful cleanup.
  assert(recorder_ != nullptr);
  recorder_->DeRegisterClient(client_id, true);

  sp<DeathNotifier> death_notifier = death_notifier_list_.valueFor(client_id);
  assert(death_notifier.get() != nullptr);

  sp<RemoteCallBack> remote_callback = remote_cb_list_.valueFor(client_id);
  assert(remote_callback.get() != nullptr);

  IInterface::asBinder(remote_callback->getRemoteClient())
      ->unlinkToDeath(death_notifier);

  death_notifier_list_.removeItem(client_id);

  remote_cb_list_.removeItem(client_id);

  if ( (death_notifier_list_.size() == 0) &&
       (remote_cb_list_.size() == 0) ) {
    QMMF_INFO("%s:%s: No client is connected! de-initialize the recorder!", TAG,
        __func__);
    recorder_->DeInit();
    delete recorder_;
    recorder_ = nullptr;
    unique_client_id_ = 0;
  }

  QMMF_INFO("%s:%s: Exit client_id(%d)", TAG, __func__, client_id);
  return ret;
}

}; //namespace recorder

}; //namespace qmmf
