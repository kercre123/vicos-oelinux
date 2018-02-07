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

#include "qmmf_vam_interface.h"
#include "qmmf_vam_config_parser.h"
#include <qmmf-sdk/qmmf_queue.h>

#include <utils/Log.h>
#include <utils/Errors.h>
#include <utils/Mutex.h>
#include <utils/Condition.h>
#include <cutils/properties.h>

#include <VAM/vaapi.h>
#include <VAM/VAMUtilities.h>

#include <qmmf_jpeg_encoder.h>
#include <qmmf_database.h>

#define DEFAULT_VAM_VIDEO_LOG_DURATION 5000
#define VAM_VIDEO_LOG_DURATION_PROP "vam.video_log.duration"

namespace qmmf {

namespace vaminterface {

using namespace jpegencoder;
using namespace database;

typedef struct VAMContext_t {
  VAMContext_t() = delete;
  VAMContext_t(bool in_present, bool in_active, uint32_t in_session_id,
               uint32_t in_track_id) :
      present(in_present), active(in_active), session_id(in_session_id),
      track_id(in_track_id) {};

  bool present;
  bool active;
  uint32_t session_id;
  uint32_t track_id;
} VAMContext;

typedef struct VAMPendingBuffer_t {
  VAMPendingBuffer_t() :
      track_id(0), session_id(0), buffer() {};
  VAMPendingBuffer_t(uint32_t in_track_id, uint32_t in_session_id,
                     BufferDescriptor in_buffer) :
      track_id(in_track_id), session_id(in_session_id), buffer(in_buffer) {};

  uint32_t track_id;
  uint32_t session_id;
  BufferDescriptor buffer;
} VAMPendingBuffer;

class VAMInstance : public VAMInterface {
public:
  VAMInstance(VAMCb *vam_cb);
  virtual ~VAMInstance();

private:
  int32_t InitVAM(const uint32_t session_id,
                  const uint32_t track_id) override;
  int32_t CloseVAM() override;

  int32_t StartVAM(CameraBufferMetaData &meta_data) override;
  int32_t VAMEnroll(VAMEnrollmentInfo &enroll_info) override;
  int32_t VAMDisenroll(uint32_t event_type, const char *id) override;
  int32_t VAMConfig(const char *json_config) override;
  int32_t VAMRemoveConfig(const char *json_config) override;
  int32_t QueueVAMBuffers(uint32_t track_id, uint32_t session_id,
                          std::vector<BufferDescriptor> &buffers,
                          std::vector<MetaData> &meta_data) override;
  int32_t InitVAMVLog(uint32_t framerate) override;
  int32_t DeinitVAMVLog() override;
  int32_t ProcessVLogBuffers(std::vector<BufferDescriptor> &buffers) override;
  int32_t CheckDBParams(const VAMDatabaseCmdParams *params);
  int32_t DatabaseCommand(const VAMDatabaseCmdParams *params,
                          VAMDatabaseCmdResult *result) override;

private:
  int32_t VAMWaitForInit();
  int32_t VAMFrameProcessed(uint64_t time_stamp);
  static int32_t VAMFrameProcessedCb(uint64_t time_stamp, void *usr_data);
  int32_t VAMEvent(struct vaapi_event *event);
  static int32_t VAMEventCb(struct vaapi_event *event, void* usr_data);
  int32_t VAMMetadata(struct vaapi_metadata_frame *frame);
  static int32_t VAMMetadataCb(struct vaapi_metadata_frame *frame,
                               void* usr_data);
  int32_t VAMSnapshot(struct vaapi_snapshot_info *snapshot_info);
  static int32_t VAMSnapshotCb(struct vaapi_snapshot_info *snapshot_info,
                               void* usr_data);
  int32_t RcConvert(int32_t ret_val);

private:
  static const char kVAMDynamicPath[];
  static const char kVAMDataPath[];
  static const nsecs_t kWaitDuration;

  Mutex vam_pending_buffers_lock_;
  Mutex vam_config_context_lock_;
  Condition wait_for_vam_to_init_;

  VAMCb *vam_cb_;

  uint32_t vam_width_;
  uint32_t vam_height_;

  VAMContext vam_context_;
  KeyedVector<uint64_t, VAMPendingBuffer> vam_pending_buffers_;
  struct vaapi_configuration vam_config_;

  JpegEncoder *jpeg_encoder_;
  EventDB vam_event_db_;
  AVQueue *vam_video_que_;
  Mutex vam_video_queue_lock_;
  uint32_t vam_video_log_duration_;
  int32_t vam_video_log_size_;
  EnrolledFacesDB vam_enroll_db_;
};

VAMInterface *VAMInterfaceFactory::NewInstance(VAMCb *vam_cb) {
  VAMInstance *new_instance = new VAMInstance(vam_cb);
  return static_cast<VAMInterface *>(new_instance);
}

const char VAMInstance::kVAMDynamicPath[] = "/usr/lib/vam_engines";
const char VAMInstance::kVAMDataPath[] = "/data/misc/camera";
const nsecs_t VAMInstance::kWaitDuration = 1000000000; // 1 sec.

VAMInstance::VAMInstance(VAMCb *vam_cb) :
    vam_cb_(vam_cb),
    vam_context_(false, false, 0, 0),
    jpeg_encoder_(NULL),
    vam_video_que_(NULL),
    vam_video_log_duration_(DEFAULT_VAM_VIDEO_LOG_DURATION),
    vam_video_log_size_(0){
  memset(&vam_config_, 0, sizeof(vam_config_));
}

VAMInstance::~VAMInstance() {
}

int32_t VAMInstance::VAMWaitForInit() {
  Mutex::Autolock l(vam_config_context_lock_);
  if (!vam_context_.active) {
    ALOGW("%s: VAM is not yet active, Wait!", __func__);
    auto ret = wait_for_vam_to_init_.waitRelative(vam_config_context_lock_,
                                                  kWaitDuration);
    if (ret == TIMED_OUT) {
      ALOGE("%s: Timed out! VAM is not initialized!", __func__);
      return NO_INIT;
    }
  }
  ALOGD("%s: VAM is in init state!", __func__);
  return NO_ERROR;
}

int32_t VAMInstance::VAMEnroll(VAMEnrollmentInfo &enroll_info) {
  int32_t ret = VAM_OK;

  if (NULL == enroll_info.data) {
    ALOGE("%s: Enroll data is missing!\n", __func__);
    return BAD_VALUE;
  }

  ret = VAMWaitForInit();
  if (NO_ERROR != ret)
    return ret;

  vaapi_enrollment_info eInfo;
  memset(&eInfo, 0, sizeof(eInfo));

  if (enroll_info.id) {
    strncpy(eInfo.id, enroll_info.id, sizeof(eInfo.id) - 1);
    eInfo.id[sizeof(eInfo.id) - 1] = '\0';
  }
  if (enroll_info.display_name) {
    strncpy(eInfo.display_name, enroll_info.display_name,
            sizeof(eInfo.display_name)-1);
    eInfo.display_name[sizeof(eInfo.display_name)-1] = '\0';
  }
  if (enroll_info.img_id) {
    strncpy(eInfo.img_id, enroll_info.img_id, sizeof(eInfo.img_id)-1);
    eInfo.img_id[sizeof(eInfo.img_id)-1] = '\0';
  }

  eInfo.type = (vaapi_object_type)enroll_info.object_type;
  eInfo.img_format = (vaapi_img_format)enroll_info.image_format;

  switch (eInfo.img_format) {
    case vaapi_format_GRAY8:
      eInfo.img_width[0] = enroll_info.image_width;
      eInfo.img_height[0] = enroll_info.image_height;
      eInfo.img_pitch[0] = enroll_info.image_width;
      eInfo.img_data[0] = enroll_info.data;
      break;
    case vaapi_format_nv12:
    case vaapi_format_nv21:
    case vaapi_format_yv12:
      //TODO: Add support for additional pixelformats
    default:
      ALOGE("%s: Unsupported format: %d\n", __func__, eInfo.img_format);
      return BAD_VALUE;
  }

  ret = vaapi_enroll_obj((vaapi_event_type)enroll_info.event_type, &eInfo);
  if (VAM_OK != ret) {
    ALOGE("%s: Failed to entroll data to VAM: %d\n", __func__, ret);
  }

  if (ret == VAM_OK && vam_enroll_db_.OpenDB() == SQLITE_OK) {
#ifdef USE_JPEG_ENCODER
    JpegEncoderConfiguration jcfg;
    jcfg.width[0] = enroll_info.image_width;
    jcfg.height[0] = enroll_info.image_height;
    jcfg.stride[0] = enroll_info.image_width;
    jcfg.scanline[0] = (enroll_info.image_height + 7) & (~7);
    jcfg.num_planes = 1;
    jcfg.format = JpegEncoderFormat::NV12;
    JpegEncoder encoder(&jcfg);

    struct JpegFrameInfo encInfo = { 0 };
    size_t lumaSize = jcfg.stride[0] * jcfg.scanline[0];
    encInfo.plane_addr[0] = new uint8_t[3 * lumaSize / 2];
    memcpy(encInfo.plane_addr[0], enroll_info.data, jcfg.width[0] * jcfg.height[0]);
    memset(&encInfo.plane_addr[0][lumaSize], 0x7F, lumaSize / 2);

    size_t eJpegSize;
    vaapi_enrollment_info e = eInfo;
    e.img_data[0] = (uint8_t *)encoder.Encode(&encInfo, &eJpegSize, 0.0f);
    e.img_width[0] = e.img_pitch[0] = eJpegSize;
    e.img_height[0] = 1;

    delete [] encInfo.plane_addr[0];

    if (vam_enroll_db_.Insert(&e) != SQLITE_OK) {
#else
    if (vam_enroll_db_.Insert(&eInfo) != SQLITE_OK) {
#endif
      ALOGE("%s: could not save enroll info in global database", __func__);
    } else {
      ALOGE("%s: saved enroll info in global database", __func__);
    }

    vam_enroll_db_.CloseDB();
  } else {
    if (ret == VAM_OK) {
      ALOGE("%s: could not open global enroll database", __func__);
    }
  }

  return RcConvert(ret);
}

int32_t VAMInstance::VAMDisenroll(uint32_t event_type, const char *id) {
  int32_t ret = VAMWaitForInit();
  if (NO_ERROR != ret)
    return ret;

  ret = vaapi_disenroll_obj((vaapi_event_type)event_type, id);
  if (VAM_OK != ret) {
    ALOGE("%s: Failed to disentroll data to VAM: %d\n", __func__, ret);
  }

  return ret;
}

int32_t VAMInstance::VAMConfig(const char *json_config) {
  int32_t ret = NO_ERROR;
  if (NULL == json_config) {
    ALOGE("%s: Invalid json config!\n", __func__);
    return BAD_VALUE;
  }

  ret = VAMWaitForInit();
  if (NO_ERROR != ret)
    return ret;

  VAMConfigParser parser;
  ret = parser.Init();
  if (NO_ERROR == ret) {
    Mutex::Autolock l(vam_config_context_lock_);
    memset(&vam_config_, 0, sizeof(vam_config_));

    ret = parser.ParseConfig(json_config, vam_config_);
    if (NO_ERROR == ret) {
        for (uint32_t i = 0; i < vam_config_.rule_size; i++) {
          vam_event_db_.InsertRule(&vam_config_.rules[i]);
        }
        ret = vaapi_set_config(&vam_config_);
        if (VAM_OK != ret) {
          ALOGE("%s: Failed to configure VAM: %d\n", __func__, ret);
        }
        ret = RcConvert(ret);
        memset(&vam_config_, 0, sizeof(vam_config_));
    } else {
      ALOGE("%s: Configuration parsing failed: %d\n", __func__, ret);
    }
  } else {
    ALOGE("%s: Failed to initialize json config parser: %d\n",
          __func__, ret);
  }

  return ret;
}

int32_t VAMInstance::VAMRemoveConfig(const char *json_config) {
  int32_t ret = NO_ERROR;
  if (NULL == json_config) {
    ALOGE("%s: Invalid json config!\n", __func__);
    return BAD_VALUE;
  }

  ret = VAMWaitForInit();
  if (NO_ERROR != ret)
    return ret;

  VAMConfigParser parser;
  ret = parser.Init();
  if (NO_ERROR == ret) {
    Mutex::Autolock l(vam_config_context_lock_);
    memset(&vam_config_, 0, sizeof(vam_config_));

    ret = parser.ParseConfig(json_config, vam_config_);
    if (NO_ERROR == ret) {
        ret = vaapi_del_config(&vam_config_);
        if (VAM_OK != ret) {
          ALOGE("%s: Failed to remove VAM config: %d\n", __func__, ret);
        }
        ret = RcConvert(ret);
        memset(&vam_config_, 0, sizeof(vam_config_));
    } else {
      ALOGE("%s: Configuration parsing failed: %d\n", __func__, ret);
    }
  } else {
    ALOGE("%s: Failed to initialize json config parser: %d\n",
          __func__, ret);
  }

  return ret;
}

int32_t VAMInstance::InitVAM(const uint32_t session_id,
                             const uint32_t track_id) {
  int32_t ret = NO_ERROR;
  Mutex::Autolock l(vam_config_context_lock_);
  if (!vam_context_.present) {
    vam_context_.present = true;
    vam_context_.session_id = session_id;
    vam_context_.track_id = track_id;
    vam_context_.active = false;
  } else {
    ALOGE("%s: VAM enabled track with id: %d already present!\n",
          __func__, vam_context_.track_id);
    ret = INVALID_OPERATION;
  }

  return ret;
}

int32_t VAMInstance::StartVAM(CameraBufferMetaData &meta_data) {
  vaapi_source_info info;

  memset(&info, 0, sizeof(info));
  snprintf(info.data_folder, sizeof(info.data_folder), kVAMDataPath);
  info.frame_l_enable = 1;

  switch (meta_data.format) {
    case BufferFormat::kNV12:
      info.img_format = vaapi_format_nv12;
      assert(2 == meta_data.num_planes);
      info.frame_l_width[0] = meta_data.plane_info[0].width;
      info.frame_l_height[0] = meta_data.plane_info[0].height;
      info.frame_l_pitch[0] = meta_data.plane_info[0].stride;
      info.frame_l_scanline[0] = meta_data.plane_info[0].scanline;
      info.frame_l_width[1] = meta_data.plane_info[1].width;
      info.frame_l_height[1] = meta_data.plane_info[1].height;
      info.frame_l_pitch[1] = meta_data.plane_info[1].stride;
      info.frame_l_scanline[1] = meta_data.plane_info[1].scanline;
      break;
    case BufferFormat::kNV21:
      info.img_format = vaapi_format_nv21;
      assert(2 == meta_data.num_planes);
      info.frame_l_width[0] = meta_data.plane_info[0].width;
      info.frame_l_height[0] = meta_data.plane_info[0].height;
      info.frame_l_pitch[0] = meta_data.plane_info[0].stride;
      info.frame_l_scanline[0] = meta_data.plane_info[0].scanline;
      info.frame_l_width[1] = meta_data.plane_info[1].width;
      info.frame_l_height[1] = meta_data.plane_info[1].height;
      info.frame_l_pitch[1] = meta_data.plane_info[1].stride;
      info.frame_l_scanline[1] = meta_data.plane_info[1].scanline;
      break;
    case BufferFormat::kBLOB:// These don't seem supported
    case BufferFormat::kRAW10:// by VAM currently!
    case BufferFormat::kRAW16:
    default:
      ALOGE("%s: Unsupported format: %d\n", __func__,
            meta_data.format);
      return BAD_VALUE;
  }

  vam_width_ = info.frame_l_width[0];
  vam_height_ = info.frame_l_height[0];

  {
    Mutex::Autolock l(vam_pending_buffers_lock_);
    vam_pending_buffers_.clear();
  }
  auto ret = vaapi_init(&info, kVAMDynamicPath);
  if (VAM_OK != ret) {
    ALOGE("%s: Failed to initialize VAM: %d\n", __func__, ret);
    return RcConvert(ret);
  }

  //TODO: Add callback for snapshot
  ret = vaapi_register_frame_processed_cb(VAMFrameProcessedCb, this);
  if (VAM_OK != ret) {
    ALOGE("%s: Failed to register processed callback: %d\n", __func__, ret);
    goto exit;
  }

  ret = vaapi_register_event_cb(VAMEventCb, this);
  if (VAM_OK != ret) {
    ALOGE("%s: Failed to register event callback: %d\n", __func__, ret);
    goto exit;
  }

  ret = vaapi_register_metadata_cb(VAMMetadataCb, this);
  if (VAM_OK != ret) {
    ALOGE("%s: Failed to register metadata callback: %d\n", __func__, ret);
    goto exit;
  }

  ret = vaapi_register_snapshot_cb(VAMSnapshotCb, this);
  if (VAM_OK != ret) {
    ALOGE("%s: Failed to register snapshot callback: %d\n", __func__, ret);
    goto exit;
  }

  ret = vaapi_run();
  if (VAM_OK != ret) {
    ALOGE("%s: Failed to start VAM: %d\n", __func__, ret);
    goto exit;
  }

  if (jpeg_encoder_) {
    delete jpeg_encoder_;
  }
  JpegEncoderConfiguration jpeg_cfg;
  for (uint32_t i = 0; i < meta_data.num_planes; i++) {
    jpeg_cfg.width[i] = meta_data.plane_info[i].width;
    jpeg_cfg.height[i] = meta_data.plane_info[i].height;
    jpeg_cfg.stride[i] = meta_data.plane_info[i].stride;
    jpeg_cfg.scanline[i] = meta_data.plane_info[i].scanline;
  }
  jpeg_cfg.num_planes = meta_data.num_planes;
  switch (meta_data.format) {
  case BufferFormat::kNV12:
    jpeg_cfg.format = JpegEncoderFormat::NV12;
    break;
  case BufferFormat::kNV21:
    jpeg_cfg.format = JpegEncoderFormat::NV21;
    break;
  default:
    jpeg_cfg.format = JpegEncoderFormat::UNKNOWN;
    ALOGE("%s: unknown buffer format (%d) passed to JPEG encoder.",
            __func__, (int)meta_data.format);
    break;
  }

  jpeg_encoder_ = new JpegEncoder(&jpeg_cfg);
  if (!jpeg_encoder_) {
    ret = VAM_FAIL;
    ALOGE("%s: Failed to allocate VAM jpeg encoder: %d\n", __func__, ret);
    goto exit;
  }

  ret = vam_event_db_.OpenDB();
  if (ret) {
    ALOGE("%s: Failed to open VAM event database: %d\n", __func__, ret);
    ret = VAM_FAIL;
    goto exit;
  }

  {
    Mutex::Autolock l(vam_config_context_lock_);
    vam_context_.active = true;
    wait_for_vam_to_init_.signal();
  }

  return RcConvert(ret);

exit:

  vaapi_deinit();
  vam_event_db_.CloseDB();
  delete jpeg_encoder_;
  jpeg_encoder_ = NULL;

  return RcConvert(ret);
}

int32_t VAMInstance::QueueVAMBuffers(uint32_t track_id,
                                     uint32_t session_id,
                                     std::vector<BufferDescriptor> &buffers,
                                     std::vector<MetaData> &meta_data) {
  int32_t ret = NO_ERROR;
  uint32_t camera_flag = static_cast<uint32_t>(MetaParamType::kCamBufMetaData);
  if (vam_context_.present) {
    if (meta_data.empty() || (buffers.size() != meta_data.size())) {
      ALOGE("%s: Invalid meta data!\n",
            __func__);
      return BAD_VALUE;
    }

    for (size_t i = 0; i < buffers.size(); i++) {
      BufferDescriptor &iter = buffers[i];
      MetaData &meta_buffer = meta_data[i];
      if (0 == (meta_buffer.meta_flag &= camera_flag)) {
        ALOGE("%s: No valid meta data for buffer %d!\n", __func__, i);
        continue;
      }
      CameraBufferMetaData camera_meta = meta_buffer.cam_buffer_meta_data;

      struct vaapi_frame_info buffer_info;
      memset(&buffer_info, 0, sizeof(buffer_info));

      switch (camera_meta.format) {
        case BufferFormat::kNV12:
        case BufferFormat::kNV21:
          buffer_info.frame_l_data[0] = (uint8_t *) iter.data;
          buffer_info.frame_l_data[1] = ((uint8_t *) iter.data) +
              (camera_meta.plane_info[0].stride *
                  camera_meta.plane_info[0].scanline);
          break;
        case BufferFormat::kBLOB:// These don't seem supported
        case BufferFormat::kRAW10:// by VAM currently!
        case BufferFormat::kRAW16:
        default:
          ALOGE("%s: Unsupported format: %d\n", __func__,
                camera_meta.format);
          return BAD_VALUE;
      }
      buffer_info.time_stamp = iter.timestamp;

      {
        Mutex::Autolock l(vam_pending_buffers_lock_);
        VAMPendingBuffer pending_buffer(track_id, session_id, iter);
        vam_pending_buffers_.add(buffer_info.time_stamp, pending_buffer);
      }

      ret = vaapi_process(&buffer_info);
      if (VAM_OK != ret) {
        if (VAM_BUSY != ret) {
          ALOGE("%s: VAM process failed: %d\n", __func__,
                ret);
        }
        {
          Mutex::Autolock l(vam_pending_buffers_lock_);
          vam_pending_buffers_.removeItem(buffer_info.time_stamp);
        }
      }
      ret = RcConvert(ret);
    }
  } else {
    ALOGE("%s: VAM context not present!\n", __func__);
    return NO_INIT;
  }

  return ret;
}

int32_t VAMInstance::VAMFrameProcessed(uint64_t time_stamp) {
  Mutex::Autolock l(vam_pending_buffers_lock_);
  ssize_t idx = vam_pending_buffers_.indexOfKey(time_stamp);
  if (0 <= idx) {
    VAMPendingBuffer pending_buffer = vam_pending_buffers_.valueAt(idx);
    vam_pending_buffers_.removeItem(time_stamp);
    auto ret = vam_cb_->ReturnTrackBuffer(pending_buffer.track_id,
                                          pending_buffer.session_id,
                                          pending_buffer.buffer);
    if(NO_ERROR != ret) {
      ALOGE("%s: ReturnTrackBuffer failed: %d!", __func__, ret);
      return VAM_FAIL;
    }
  } else {
    ALOGE("%s: No pending buffers found!\n", __func__);
    return VAM_NOTFOUND;
  }

  return VAM_OK;
}

int32_t VAMInstance::VAMFrameProcessedCb(uint64_t time_stamp, void *usr_data) {
  int32_t ret;
  VAMInstance *ctx = static_cast<VAMInstance *> (usr_data);
  if (NULL != ctx) {
    ret = ctx->VAMFrameProcessed(time_stamp);
  } else {
    ALOGE("%s: Invalid user data!\n", __func__);
    ret = VAM_NULLPTR;
  }

  return ret;
}

int32_t VAMInstance::VAMEvent(struct vaapi_event *event) {
  if (NULL == event) {
    return VAM_NULLPTR;
  }

  std::string eventString = getStrFromEvent(event);
  vam_cb_->SendVAMMeta(vam_context_.session_id, vam_context_.track_id,
                        eventString.c_str(), eventString.size(), event->pts);

  return VAM_OK;
}

int32_t VAMInstance::VAMEventCb(struct vaapi_event *event, void* usr_data) {
  int32_t ret;
  VAMInstance *ctx = static_cast<VAMInstance *> (usr_data);
  if (NULL != ctx) {
    ret = ctx->VAMEvent(event);
  } else {
    ALOGE("%s: Invalid user data!\n", __func__);
    ret = VAM_NULLPTR;
  }

  return ret;
}

int32_t VAMInstance::VAMMetadata(struct vaapi_metadata_frame *frame) {
  if (NULL == frame) {
    return VAM_NULLPTR;
  }

  vam_event_db_.InsertMeta(frame);

  std::string metaString = getStrFromMetadataFrame(frame);
  vam_cb_->SendVAMMeta(vam_context_.session_id, vam_context_.track_id,
                       metaString.c_str(), metaString.size(), frame->pts);

  return VAM_OK;
}

int32_t VAMInstance::VAMMetadataCb(struct vaapi_metadata_frame *frame,
                                   void* usr_data) {
  int32_t ret;
  VAMInstance *ctx = static_cast<VAMInstance *> (usr_data);
  if (NULL != ctx) {
    ret = ctx->VAMMetadata(frame);
  } else {
    ALOGE("%s: Invalid user data!\n", __func__);
    ret = VAM_NULLPTR;
  }

  return ret;
}

int32_t VAMInstance::VAMSnapshot(struct vaapi_snapshot_info *snapshot_info) {
    if (NULL == snapshot_info || NULL == jpeg_encoder_) {
      return VAM_NULLPTR;
    }

    Mutex::Autolock l(vam_video_queue_lock_);
    if (vam_video_log_size_ <= 0) {
      ALOGE("%s: video log file size <= 0", __func__);
      return VAM_FAIL;
    }

    ssize_t q_size = AVQueueSize(vam_video_que_);
    if (q_size <= 0) {
      ALOGE("%s: video log buffer count <= 0", __func__);
      return VAM_FAIL;
    }

    uint8_t *video_data = new uint8_t[vam_video_log_size_];
    if (!video_data) {
      ALOGE("%s: Could not allocate buffer of size %d", __func__, vam_video_log_size_);
      return VAM_FAIL;
    }

    uint32_t video_size = 0;
    std::vector<AVPacket *> packets;
    for (ssize_t i = 0; i < q_size; i++) {
      AVPacket *p = (AVPacket *)AVQueuePopTail(vam_video_que_);
      memcpy(&video_data[video_size], p->data, p->size);
      video_size += p->size;
      packets.push_back(p);
    }
    for (ssize_t i = 0; i < q_size; i++) {
      AVQueuePushHead(vam_video_que_, packets[i]);
    }

#ifdef USE_JPEG_ENCODER
    JpegFrameInfo jpeg_frame;
    for (uint32_t i = 0; i < JPEG_MAX_BUFFER_PLANES; i++) {
      jpeg_frame.plane_addr[i] = snapshot_info->img_data[i];
    }

    // make scale coefficient such to scale down to VGA
    float scale = (float)vam_width_ / 640.0f;

    size_t thumb_size;
    uint8_t *thumb_data_temp = (uint8_t *)jpeg_encoder_->Encode(&jpeg_frame, &thumb_size, scale);
    uint8_t *thumb_data = nullptr;
    if (thumb_data_temp) {
        thumb_data = new uint8_t[thumb_size];
        memcpy(thumb_data, thumb_data_temp, thumb_size);
    }


    size_t image_size;
    uint8_t *image_data = (uint8_t *)jpeg_encoder_->Encode(&jpeg_frame, &image_size, 0.0f);
#else
    size_t thumb_size = vam_width_ * vam_height_;
    uint8_t *thumb_data = snapshot_info->img_data[0];
    size_t image_size = vam_width_ * vam_height_;
    uint8_t *image_data = snapshot_info->img_data[0];
#endif
    if (image_data) {
        struct vaapi_snapshot_info info;
        memcpy(&info, snapshot_info, sizeof(info));
        info.img_data[0] = image_data;
        info.img_data[1] = video_data;
        info.img_data[2] = thumb_data;
        info.data_size[0] = image_size;
        info.data_size[1] = video_size;
        info.data_size[2] = thumb_size;
      int32_t res = vam_event_db_.Insert(&info);
      if (res) {
        ALOGE("%s: could not write frame to database: %d", __func__, res);
      }
    } else {
      ALOGE("%s: could not create jpeg file.", __func__);
    }

#ifdef USE_JPEG_ENCODER
    delete [] thumb_data;
#endif
    delete [] video_data;
  return VAM_OK;
}

int32_t VAMInstance::VAMSnapshotCb(struct vaapi_snapshot_info *snapshot_info,
                                   void* usr_data) {
  int32_t ret;
  VAMInstance *ctx = static_cast<VAMInstance *> (usr_data);
  if (NULL != ctx) {
    ret = ctx->VAMSnapshot(snapshot_info);
  } else {
    ALOGE("%s: Invalid user data!\n", __func__);
    ret = VAM_NULLPTR;
  }

  return ret;
}

int32_t VAMInstance::InitVAMVLog(uint32_t framerate) {
  char property[PROPERTY_VALUE_MAX];
  uint32_t frame_interval;

  vam_video_queue_lock_.lock();
  frame_interval = 1000 / framerate;
  if (property_get(VAM_VIDEO_LOG_DURATION_PROP, property, NULL) > 0) {
    vam_video_log_duration_ = atoi(property);
  } else {
    vam_video_log_duration_ = DEFAULT_VAM_VIDEO_LOG_DURATION;
  }

  if (vam_video_que_) {
    AVQueueFree(&vam_video_que_, AVFreePacket);
    vam_video_log_size_ = 0;
  }

  if (frame_interval < vam_video_log_duration_) {
    uint32_t buffer_count = (vam_video_log_duration_ / frame_interval) * 2;
    AVQueueInit(&vam_video_que_, REALTIME, buffer_count + 1, buffer_count);
  }
  vam_video_queue_lock_.unlock();
  return OK;
}

int32_t VAMInstance::DeinitVAMVLog() {
  Mutex::Autolock l(vam_video_queue_lock_);
  if (vam_video_que_) {
      AVQueueFree(&vam_video_que_, AVFreePacket);
  }
  return OK;
}

int32_t VAMInstance::ProcessVLogBuffers(std::vector<BufferDescriptor> &buffers) {
  int32_t ret = VAM_OK;
  Mutex::Autolock l(vam_video_queue_lock_);
  if (vam_video_que_) {
    AVQueue *q = vam_video_que_;
    for (size_t i = 0; i < buffers.size(); i++) {
      uint32_t size = buffers[i].size;
      uint8_t *buffer = (uint8_t *)buffers[i].data;

      int pps_size = 0;
      uint8_t *tmp_buffer = NULL;
      AVPacket *packet = NULL;

      if (AVQueueSize(q) == q->max_size) {
        AVPacket *p = (AVPacket *)AVQueuePopTail(q);
        vam_video_log_size_ -= p->size;
        delete [] p->data;
        delete p;
      }

      if (buffer[0] == 0x00 && buffer[1] == 0x00 && buffer[2] == 0x00 &&
          buffer[3] == 0x01 && buffer[4] == 0x67) {/* VPS,SPS,PPS*/
        if (q->pps != NULL) {
          free(q->pps);
          q->pps = NULL;
        }
        q->pps = new char[size];
        memcpy(q->pps, buffer, size);
        q->pps_size = size;
        q->is_pps = true;
        continue;
      }

      if (buffer[0] == 0x00 && buffer[1] == 0x00 && buffer[2] == 0x00 &&
          buffer[3] == 0x01 && buffer[4] == 0x65) {
        if (q->is_pps != true) {
          pps_size = q->pps_size;
        }
        q->is_pps = false;
      }

      /* Set pointer to start address */
      packet = new AVPacket;
      if (!packet) {
        continue;
      }

      /* Allocate a new frame object. */
      packet->data = tmp_buffer = new uint8_t[size + pps_size];
      if (!packet->data) {
        delete packet;
        continue;
      }

      if (pps_size) {
        memcpy(tmp_buffer, q->pps, q->pps_size);
        tmp_buffer += q->pps_size;
      }

      memcpy(tmp_buffer, buffer, size);
      packet->size = size + pps_size;
      packet->timestamp = buffers[i].timestamp;
      if (!AVQueuePushHead(q, packet)) {
          vam_video_log_size_ += packet->size;
      }
    }
  } else {
    ALOGE("%s: VAM video logger queue not initialized.\n", __func__);
  }

  return RcConvert(ret);
}

int32_t VAMInstance::CheckDBParams(const VAMDatabaseCmdParams *params) {
  if (!params) {
    ALOGE("%s: null input parameter", __func__);
    return BAD_VALUE;
  }

  int32_t res = OK;
  switch (params->command) {
    case VAMDatabaseCommand::GET_ALL_SESSIONS:
    case VAMDatabaseCommand::GET_ALL_EVENT_IDS:
    case VAMDatabaseCommand::GET_ALL_FRAME_IDS:
    case VAMDatabaseCommand::GET_ALL_RULES_IDS:
    case VAMDatabaseCommand::GET_CURRENT_SESSION:
    case VAMDatabaseCommand::GET_EVENT_PAGE:
    case VAMDatabaseCommand::GET_EVENTS_TIMESTAMPS:
    case VAMDatabaseCommand::GET_EVENT_COUNT:
    case VAMDatabaseCommand::GET_EVENT_PAGE_SIZE:
    case VAMDatabaseCommand::FRDB_GET_IMAGE_IDS:
    case VAMDatabaseCommand::FRDB_GET_FEATURE_IDS:
      break;

    case VAMDatabaseCommand::GET_EVENT:
    case VAMDatabaseCommand::GET_METADATA_FOR_EVENT:
    case VAMDatabaseCommand::GET_METADATA_FOR_FRAME:
    case VAMDatabaseCommand::GET_VIDEO:
    case VAMDatabaseCommand::GET_SNAPSHOT:
    case VAMDatabaseCommand::GET_RULE:
    case VAMDatabaseCommand::GET_EVENTS_FOR_FRAME:
    case VAMDatabaseCommand::GET_EVENTS_FOR_RULE:
    case VAMDatabaseCommand::GET_EVENTS_FOR_RULE_TS_LIMIT:
    case VAMDatabaseCommand::REMOVE_EVENT:
    case VAMDatabaseCommand::REMOVE_RULE:
    case VAMDatabaseCommand::GET_SNAPSHOT_ID_FOR_EVENT:
    case VAMDatabaseCommand::GET_SNAPSHOT_FOR_EVENT:
    case VAMDatabaseCommand::GET_VIDEO_FOR_EVENT:
    case VAMDatabaseCommand::GET_THUMBNAIL:
    case VAMDatabaseCommand::GET_THUMBNAIL_FOR_EVENT:
    case VAMDatabaseCommand::FRDB_GET_IMAGE_IDS_FOR_FEATURE:
    case VAMDatabaseCommand::FRDB_GET_IMAGE_IDS_FOR_DISPLAY_NAME:
    case VAMDatabaseCommand::FRDB_GET_FEATURE_IDS_FOR_IMAGE:
    case VAMDatabaseCommand::FRDB_GET_FEATURE_IDS_FOR_DISPLAY_NAME:
    case VAMDatabaseCommand::FRDB_GET_ENROLL_INFO:
    case VAMDatabaseCommand::FRDB_GET_ENROLL_IMAGE_DATA:
    case VAMDatabaseCommand::FRDB_REMOVE_ENROLLED_IMAGE:
      if (!params->id || !strlen(params->id)) {
        res = NOT_ENOUGH_DATA;
      }
      break;

    case VAMDatabaseCommand::GET_EVENTS_FOR_TS_LIMIT:
    case VAMDatabaseCommand::GET_LAST_EVENTS:
      if (!params->event_type || !params->num_event_types) {
        res = NOT_ENOUGH_DATA;
      }
      break;

    case VAMDatabaseCommand::SET_EVENT_PAGE_SIZE:
      if (!params->page_index) {
        res = BAD_VALUE;
      }
      break;

    case VAMDatabaseCommand::GET_EVENTS_BY_INDEX_RANGE:
    case VAMDatabaseCommand::GET_EVENTS_BY_INDEX_RANGE_BEFORE_TS:
    case VAMDatabaseCommand::GET_EVENTS_BY_INDEX_RANGE_AFTER_TS:
      if (!params->index_range || !params->num_index_ranges ||
          !params->event_type || !params->num_event_types) {
        res = NOT_ENOUGH_DATA;
      }
      break;

    default:
      res = INVALID_OPERATION;
      break;
  }

  return res;
}

int32_t VAMInstance::DatabaseCommand(const VAMDatabaseCmdParams *params,
                                     VAMDatabaseCmdResult *result) {
  if (!params || !result) {
    return BAD_VALUE;
  }

  int32_t ret = OK;
  std::vector<std::string> results;
  uint8_t *blob_ptr = NULL;
  size_t blob_size = 0;
  std::vector<std::string> all_sessions;
  std::vector<std::string> session_list;
  std::string current_session;
  bool isEnrollCmd = false;

  ret = CheckDBParams(params);
  if (ret != OK) {
    memset(result, 0, sizeof(*result));
    result->status = SQLITE_ERROR;
    result->command = params->command;
    ALOGE("%s: invalid input parameters", __func__);
    return ret;
  }

  if (params->command == VAMDatabaseCommand::FRDB_GET_IMAGE_IDS ||
      params->command == VAMDatabaseCommand::FRDB_GET_IMAGE_IDS_FOR_FEATURE ||
      params->command == VAMDatabaseCommand::FRDB_GET_IMAGE_IDS_FOR_DISPLAY_NAME ||
      params->command == VAMDatabaseCommand::FRDB_GET_FEATURE_IDS ||
      params->command == VAMDatabaseCommand::FRDB_GET_FEATURE_IDS_FOR_IMAGE ||
      params->command == VAMDatabaseCommand::FRDB_GET_FEATURE_IDS_FOR_DISPLAY_NAME ||
      params->command == VAMDatabaseCommand::FRDB_GET_ENROLL_INFO ||
      params->command == VAMDatabaseCommand::FRDB_GET_ENROLL_IMAGE_DATA ||
      params->command == VAMDatabaseCommand::FRDB_REMOVE_ENROLLED_IMAGE) {
      isEnrollCmd = true;
  }

  EventDB::GetAllSessions(&all_sessions);
  vam_event_db_.GetCurrentSession(&current_session);

  if (!params->session || !strlen(params->session)) {
    if (params->command != VAMDatabaseCommand::GET_ALL_SESSIONS &&
        params->command != VAMDatabaseCommand::GET_CURRENT_SESSION) {
      session_list = all_sessions;
    } else {
      session_list.push_back(current_session);
    }
  } else {
    if (params->command != VAMDatabaseCommand::GET_ALL_SESSIONS &&
        params->command != VAMDatabaseCommand::GET_CURRENT_SESSION) {
      int session_idx = -1;
      for (size_t i = 0; i < all_sessions.size(); i++) {
        if (!strcmp(params->session, all_sessions[i].c_str())) {
          session_idx = (int)i;
          break;
        }
      }

      if (session_idx >= 0) {
        if (!strcmp(current_session.c_str(), params->session)) {
            session_list.push_back(current_session);
        } else {
          session_list.push_back(params->session);
        }
      } else {
        memset(result, 0, sizeof(*result));
        result->status = SQLITE_ERROR;
        result->command = params->command;
        ALOGE("%s: session '%s' does not exist", __func__, params->session);
        return OK;
      }
    } else {
      session_list.push_back(current_session);
    }
  }

  if (isEnrollCmd) {
    session_list.clear();
    session_list.push_back(current_session);
  }

  vaapi_enrollment_info eInfo;
  for (size_t i = 0; i < session_list.size(); i++) {
    vaapi_metadata_frame meta;
    std::vector<vaapi_event> events;
    EventDB database, *db_ptr;

    if (!isEnrollCmd) {
      if (!session_list[i].compare(current_session)) {
        db_ptr = &vam_event_db_;
      } else {
        database.OpenDB(session_list[i].c_str());
        db_ptr = &database;
      }
    } else {
      vam_enroll_db_.OpenDB();
    }

    switch (params->command) {
      case VAMDatabaseCommand::GET_ALL_SESSIONS:
        ret = EventDB::GetAllSessions(&results);
        break;
      case VAMDatabaseCommand::GET_CURRENT_SESSION: {
        std::string session;
        ret = db_ptr->GetCurrentSession(&session);
        if (ret == SQLITE_OK) {
          results.push_back(session);
        }
        break;
      }
      case VAMDatabaseCommand::GET_ALL_EVENT_IDS: {
        std::vector<std::string> tmp_res;
        ret = db_ptr->GetAllEventIds(&tmp_res);
        results.insert(results.end(), tmp_res.begin(), tmp_res.end());
        break;
      }
      case VAMDatabaseCommand::GET_ALL_FRAME_IDS: {
        std::vector<std::string> tmp_res;
        ret = db_ptr->GetAllFrameIds(&tmp_res);
        results.insert(results.end(), tmp_res.begin(), tmp_res.end());
        break;
      }
      case VAMDatabaseCommand::GET_ALL_RULES_IDS: {
        std::vector<std::string> tmp_res;
        ret = db_ptr->GetAllRulesIds(&tmp_res);
        results.insert(results.end(), tmp_res.begin(), tmp_res.end());
        break;
      }
      case VAMDatabaseCommand::GET_METADATA_FOR_EVENT:
        ret = db_ptr->GetMetadataForEvent(params->id, &meta);
        if (ret == SQLITE_OK && meta.object_num) {
          results.push_back(getStrFromMetadataFrame(&meta));
          delete [] meta.objects;
        }
        break;
      case VAMDatabaseCommand::GET_METADATA_FOR_FRAME:
        ret = db_ptr->GetMetadataForFrame(params->id, &meta);
        if (ret == SQLITE_OK && meta.object_num) {
          results.push_back(getStrFromMetadataFrame(&meta));
          delete [] meta.objects;
        }
        break;
      case VAMDatabaseCommand::GET_SNAPSHOT_ID_FOR_EVENT: {
        std::string frameId;
        ret = db_ptr->GetFrameIdForEvent(params->id, &frameId);
        if (ret == SQLITE_OK) {
          results.push_back(frameId);
        }
        break;
      }
      case VAMDatabaseCommand::GET_SNAPSHOT_FOR_EVENT:
        ret = db_ptr->GetFrameForEvent(params->id, &blob_ptr, &blob_size);
        break;
      case VAMDatabaseCommand::GET_VIDEO_FOR_EVENT:
        ret = db_ptr->GetVideoForEvent(params->id, &blob_ptr, &blob_size);
        break;
      case VAMDatabaseCommand::GET_SNAPSHOT:
        ret = db_ptr->GetImage(params->id, &blob_ptr, &blob_size);
        break;
      case VAMDatabaseCommand::GET_VIDEO:
        ret = db_ptr->GetVideo(params->id, &blob_ptr, &blob_size);
        break;
      case VAMDatabaseCommand::GET_THUMBNAIL:
        ret = db_ptr->GetThumbnail(params->id, &blob_ptr, &blob_size);
      break;
      case VAMDatabaseCommand::GET_THUMBNAIL_FOR_EVENT:
        ret = db_ptr->GetThumbnailForEvent(params->id, &blob_ptr, &blob_size);
      break;
      case VAMDatabaseCommand::GET_RULE: {
        vaapi_rule rule;
        ret = db_ptr->GetRule(params->id, &rule);
        if (ret == SQLITE_OK) {
          results.push_back(getStrFromRule(&rule));
        }
        break;
      }
      case VAMDatabaseCommand::GET_EVENT: {
        vaapi_event ev;
        ret = db_ptr->GetEvent(params->id, &ev);
        if (ret == SQLITE_OK) {
          results.push_back(getStrFromEvent(&ev));
        }
        break;
      }
      case VAMDatabaseCommand::GET_EVENTS_FOR_TS_LIMIT:
        ret = db_ptr->GetEventsForInterval(params->max_count,
                                           params->pts, params->pts1,
                                           &events,
                                           params->event_type,
                                           params->num_event_types);
        for (size_t i = 0; i < events.size(); i++) {
          results.push_back(getStrFromEvent(&events[i]));
        }
        break;
      case VAMDatabaseCommand::GET_LAST_EVENTS:
        ret = db_ptr->GetLastEvents(params->max_count, &events,
                                    params->event_type,
                                    params->num_event_types);
        for (size_t i = 0; i < events.size(); i++) {
          results.push_back(getStrFromEvent(&events[i]));
        }
        break;
      case VAMDatabaseCommand::GET_EVENTS_FOR_FRAME:
        ret = db_ptr->GetEventsForFrame(params->id, params->max_count, &events);
        for (size_t i = 0; i < events.size(); i++) {
          results.push_back(getStrFromEvent(&events[i]));
        }
        break;
      case VAMDatabaseCommand::GET_EVENTS_FOR_RULE:
        ret = db_ptr->GetEventsForRule(params->id, params->max_count, &events);
        for (size_t i = 0; i < events.size(); i++) {
          results.push_back(getStrFromEvent(&events[i]));
        }
        break;
      case VAMDatabaseCommand::GET_EVENTS_FOR_RULE_TS_LIMIT:
        ret = db_ptr->GetEventsForRule(params->id, params->max_count,
                                       params->pts, params->pts1, &events);
        for (size_t i = 0; i < events.size(); i++) {
          results.push_back(getStrFromEvent(&events[i]));
        }
        break;
      case VAMDatabaseCommand::GET_EVENTS_TIMESTAMPS: {
        std::vector<std::string> tmp_res;
        ret = db_ptr->GetEventsTimestamps(&tmp_res);
        results.insert(results.end(), tmp_res.begin(), tmp_res.end());
        break;
      }
      case VAMDatabaseCommand::GET_EVENT_COUNT: {
        std::stringstream ss;
        uint32_t tmp_res;
        ret = db_ptr->GetEventCount(&tmp_res);
        results.push_back(ss.str());
        break;
      }
      case VAMDatabaseCommand::SET_EVENT_PAGE_SIZE: {
        ret = db_ptr->SetEventPageSize(params->page_index);
        break;
      }
      case VAMDatabaseCommand::GET_EVENT_PAGE_SIZE: {
        std::stringstream ss;
        uint32_t tmp_res;
        ret = db_ptr->GetEventPageSize(&tmp_res);
        results.push_back(ss.str());
        break;
      }
      case VAMDatabaseCommand::GET_EVENT_PAGE: {
        ret = db_ptr->GetEventPage(params->page_index, &events,
                               params->event_type, params->num_event_types);
        for (size_t i = 0; i < events.size(); i++) {
          results.push_back(getStrFromEvent(&events[i]));
        }
        break;
      }
      case VAMDatabaseCommand::GET_EVENTS_BY_INDEX_RANGE: {
        ret = db_ptr->GetEventsByIndexRange(&events,
                params->event_type, params->num_event_types,
                params->index_range, params->num_index_ranges);
        for (size_t i = 0; i < events.size(); i++) {
          results.push_back(getStrFromEvent(&events[i]));
        }
        break;
      }
      case VAMDatabaseCommand::GET_EVENTS_BY_INDEX_RANGE_BEFORE_TS: {
        ret = db_ptr->GetEventsByIndexRangeBeforeTS(&events,
               params->event_type, params->num_event_types,
               params->index_range, params->num_index_ranges, params->pts);
        for (size_t i = 0; i < events.size(); i++) {
          results.push_back(getStrFromEvent(&events[i]));
        }
        break;
      }
      case VAMDatabaseCommand::GET_EVENTS_BY_INDEX_RANGE_AFTER_TS: {
        ret = db_ptr->GetEventsByIndexRangeAfterTS(&events,
               params->event_type, params->num_event_types,
               params->index_range, params->num_index_ranges, params->pts);
        for (size_t i = 0; i < events.size(); i++) {
          results.push_back(getStrFromEvent(&events[i]));
        }
        break;
      }
      case VAMDatabaseCommand::REMOVE_EVENT:
        ret = db_ptr->RemoveEvent(params->id);
        break;
      case VAMDatabaseCommand::REMOVE_RULE:
        ret = db_ptr->RemoveRule(params->id);
        break;
      case VAMDatabaseCommand::FRDB_GET_IMAGE_IDS:
        ret = vam_enroll_db_.GetImageIds(&results);
        break;
      case VAMDatabaseCommand::FRDB_GET_IMAGE_IDS_FOR_FEATURE:
        ret = vam_enroll_db_.GetImageIdsForFeature(params->id, &results);
        break;
      case VAMDatabaseCommand::FRDB_GET_IMAGE_IDS_FOR_DISPLAY_NAME:
        ret = vam_enroll_db_.GetImageIdsForDisplayName(params->id, &results);
        break;
      case VAMDatabaseCommand::FRDB_GET_FEATURE_IDS:
        ret = vam_enroll_db_.GetFeatureIds(&results);
        break;
      case VAMDatabaseCommand::FRDB_GET_FEATURE_IDS_FOR_IMAGE:
        ret = vam_enroll_db_.GetFeatureIdForImage(params->id, &results);
        break;
      case VAMDatabaseCommand::FRDB_GET_FEATURE_IDS_FOR_DISPLAY_NAME:
        ret = vam_enroll_db_.GetFeatureIdsForDisplayName(params->id, &results);
        break;
      case VAMDatabaseCommand::FRDB_GET_ENROLL_INFO:
        ret = vam_enroll_db_.GetEnrollInfo(params->id, &eInfo, false);
        results.push_back(getStrFromEnrollInfo(&eInfo));
        break;
      case VAMDatabaseCommand::FRDB_GET_ENROLL_IMAGE_DATA:
        ret = vam_enroll_db_.GetEnrollInfo(params->id, &eInfo, true);
        break;
      case VAMDatabaseCommand::FRDB_REMOVE_ENROLLED_IMAGE:
        ret = vam_enroll_db_.Remove(params->id);
        break;
      default:
        ALOGE("%s: unsupported database operation: %d", __func__, params->command);
        ret = SQLITE_ERROR;
        break;
    }
  }

  memset(result, 0, sizeof(*result));
  result->status = ret;
  result->command = params->command;
  if (ret == SQLITE_OK) {
    switch (result->command) {
      case VAMDatabaseCommand::GET_ALL_SESSIONS:
      case VAMDatabaseCommand::GET_CURRENT_SESSION:
      case VAMDatabaseCommand::GET_METADATA_FOR_EVENT:
      case VAMDatabaseCommand::GET_METADATA_FOR_FRAME:
      case VAMDatabaseCommand::GET_ALL_EVENT_IDS:
      case VAMDatabaseCommand::GET_ALL_FRAME_IDS:
      case VAMDatabaseCommand::GET_ALL_RULES_IDS:
      case VAMDatabaseCommand::GET_RULE:
      case VAMDatabaseCommand::GET_EVENT:
      case VAMDatabaseCommand::GET_EVENTS_FOR_TS_LIMIT:
      case VAMDatabaseCommand::GET_LAST_EVENTS:
      case VAMDatabaseCommand::GET_EVENTS_FOR_FRAME:
      case VAMDatabaseCommand::GET_EVENTS_FOR_RULE:
      case VAMDatabaseCommand::GET_EVENTS_FOR_RULE_TS_LIMIT:
      case VAMDatabaseCommand::GET_SNAPSHOT_ID_FOR_EVENT:
      case VAMDatabaseCommand::GET_EVENTS_TIMESTAMPS:
      case VAMDatabaseCommand::GET_EVENT_COUNT:
      case VAMDatabaseCommand::GET_EVENT_PAGE_SIZE:
      case VAMDatabaseCommand::GET_EVENT_PAGE:
      case VAMDatabaseCommand::GET_EVENTS_BY_INDEX_RANGE:
      case VAMDatabaseCommand::GET_EVENTS_BY_INDEX_RANGE_BEFORE_TS:
      case VAMDatabaseCommand::GET_EVENTS_BY_INDEX_RANGE_AFTER_TS:
      case VAMDatabaseCommand::FRDB_GET_IMAGE_IDS:
      case VAMDatabaseCommand::FRDB_GET_IMAGE_IDS_FOR_FEATURE:
      case VAMDatabaseCommand::FRDB_GET_IMAGE_IDS_FOR_DISPLAY_NAME:
      case VAMDatabaseCommand::FRDB_GET_FEATURE_IDS:
      case VAMDatabaseCommand::FRDB_GET_FEATURE_IDS_FOR_IMAGE:
      case VAMDatabaseCommand::FRDB_GET_FEATURE_IDS_FOR_DISPLAY_NAME:
      case VAMDatabaseCommand::FRDB_GET_ENROLL_INFO: {
        std::stringstream ss;
        result->elements_count = results.size();
        ss << "{";
        for (uint32_t i = 0; i < result->elements_count; i++) {
            ss << "\"result" << i << "\":";
            if (results[i].c_str()[0] != '{') ss << "\"";
            ss << results[i].c_str();
            if (results[i].c_str()[0] != '{') ss << "\"";
            if (i < result->elements_count - 1)
                ss << ",";
        }
        ss << "}";

        result->data_size = ss.str().length();
        result->data = new char[result->data_size];
        if (result->data) {
          memcpy(result->data, ss.str().c_str(), result->data_size);
        } else {
            result->status = SQLITE_NOMEM;
        }
        break;
      }
      case VAMDatabaseCommand::GET_VIDEO:
      case VAMDatabaseCommand::GET_SNAPSHOT:
      case VAMDatabaseCommand::GET_SNAPSHOT_FOR_EVENT:
      case VAMDatabaseCommand::GET_VIDEO_FOR_EVENT:
      case VAMDatabaseCommand::GET_THUMBNAIL:
      case VAMDatabaseCommand::GET_THUMBNAIL_FOR_EVENT:
        result->data = (char *)blob_ptr;
        result->data_size = blob_size;
        result->elements_count = (result->data_size) ? 1 : 0;
        break;
      case VAMDatabaseCommand::FRDB_GET_ENROLL_IMAGE_DATA: {
          blob_size = 0;
          for (int i = 0; i < 3; i++) {
            blob_size += eInfo.img_pitch[i] * eInfo.img_height[i];
          }
          result->data = (char *)eInfo.img_data[0];
          result->data_size = blob_size;
          result->elements_count = (result->data_size) ? 1 : 0;
        }
        break;
      default:
        break;
    }
  }

  if (isEnrollCmd) {
    vam_enroll_db_.CloseDB();
  }

  return OK;
}

int32_t VAMInstance::CloseVAM() {
  int32_t ret = VAM_OK;
  Mutex::Autolock l(vam_config_context_lock_);

  if (vam_context_.present) {
    if (vam_context_.active) {
      ret = vaapi_stop();
      if (VAM_OK != ret) {
        ALOGE("%s: Error trying to stop VAM: %d\n", __func__, ret);
      }

      ret = vaapi_deinit();
      if (VAM_OK != ret) {
        ALOGE("%s: Error trying to de-initialize VAM: %d\n",
              __func__, ret);
      }

      vam_event_db_.CloseDB();

      delete jpeg_encoder_;
      jpeg_encoder_ = NULL;
    }
    memset(&vam_context_, 0, sizeof(vam_context_));
  }

  return RcConvert(ret);
}

int32_t VAMInstance::RcConvert(int32_t ret) {
  int32_t rc;

  switch (ret) {
  case VAM_OK:
    rc = NO_ERROR;
    break;
  case VAM_FAIL:
    rc = UNKNOWN_ERROR;
    break;
  case VAM_NOTFOUND:
    rc = NAME_NOT_FOUND;
    break;
  case VAM_TIMEOUT:
    rc = TIMED_OUT;
    break;
  case VAM_NULLPTR:
    rc = BAD_VALUE;
    break;
  case VAM_NOT_INITED:
    rc = NO_INIT;
    break;
  case VAM_IMG_FORMAT_NOT_SUPPORTED:
    rc = BAD_TYPE;
    break;
  case VAM_LOCK_FAIL:
    rc = WOULD_BLOCK;
    break;
  case VAM_UNLOCK_FAIL:
    rc = INVALID_OPERATION;
    break;
  case VAM_FRAME_QUEUE_FULL:
    rc = NO_MEMORY;
    break;
  case VAM_ENGINE_IS_NOT_RUNNING:
    rc = DEAD_OBJECT;
    break;
  case VAM_ENGINE_HAS_NO_CONFIG:
    rc = NOT_ENOUGH_DATA;
    break;
  case VAM_ENGINE_DYLIB_PATH_NOT_SET:
    rc = PERMISSION_DENIED;
    break;
  case VAM_ENGINE_DYLIB_NOT_EXIST:
    rc = UNKNOWN_TRANSACTION;
    break;
  case VAM_ENGINE_SYMBOL_ERROR:
    rc = FAILED_TRANSACTION;
    break;
  case VAM_SETRULE_FAIL:
    rc = FDS_NOT_ALLOWED;
    break;
  case VAM_BUSY:
    rc = ALREADY_EXISTS;
    break;
  case VAM_STR_OVERSIZE:
    rc = INVALID_OPERATION;
    break;
  default:
    ALOGE("%s: Error not converted VAM return code: %d\n", __func__, ret);
    rc = UNKNOWN_ERROR + ret + 100;
  }

  return rc;
}
} //namespace vaminterface ends here

} //namespace qmmf ends here
