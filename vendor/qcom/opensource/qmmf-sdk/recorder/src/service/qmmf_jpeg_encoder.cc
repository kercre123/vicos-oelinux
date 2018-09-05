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

#include <mutex>
#include <dlfcn.h>

#include <mm_jpeg_interface.h>
#include <hardware/camera3.h>
#include <qmmf_jpeg_encoder.h>

#include "common/utils/qmmf_condition.h"

using namespace qmmf;

typedef uint32_t (*jpeg_open_proc_t)(mm_jpeg_ops_t *,
                                     mm_jpeg_mpo_ops_t *,
                                     mm_dimension,
                                     cam_related_system_calibration_data_t *);

typedef struct {
  jpeg_open_proc_t jpeg_open_proc_;
  uint32_t handle_;
  mm_dimension pic_size_;
  mm_jpeg_ops_t ops_;
  mm_jpeg_encode_params_t params_;
  mm_jpeg_job_t job_;
  uint32_t job_id_;
  std::mutex encode_lock_;
  std::mutex enc_done_lock_;
  QCondition enc_done_cond_;
} JpegEncoderParams;

#define JE_GET_PARAMS(x) JpegEncoderParams *(x) = (JpegEncoderParams *)cfg_

namespace qmmf {

namespace jpegencoder {

JpegEncoder *JpegEncoder::encoder_instance_ = 0;

uint8_t JpegEncoder::DEFAULT_QTABLE_0[] = {
  16, 11, 10, 16,  24,  40,  51,  61,
  12, 12, 14, 19,  26,  58,  60,  55,
  14, 13, 16, 24,  40,  57,  69,  56,
  14, 17, 22, 29,  51,  87,  80,  62,
  18, 22, 37, 56,  68, 109, 103,  77,
  24, 35, 55, 64,  81, 104, 113,  92,
  49, 64, 78, 87, 103, 121, 120, 101,
  72, 92, 95, 98, 112, 100, 103,  99
};

uint8_t JpegEncoder::DEFAULT_QTABLE_1[] = {
  17, 18, 24, 47, 99, 99, 99, 99,
  18, 21, 26, 66, 99, 99, 99, 99,
  24, 26, 56, 99, 99, 99, 99, 99,
  47, 66, 99, 99, 99, 99, 99, 99,
  99, 99, 99, 99, 99, 99, 99, 99,
  99, 99, 99, 99, 99, 99, 99, 99,
  99, 99, 99, 99, 99, 99, 99, 99,
  99, 99, 99, 99, 99, 99, 99, 99
};

void EncodeCbGlobal(jpeg_job_status_t status, uint32_t /*client_hdl*/,
                    uint32_t /*jobId*/, mm_jpeg_output_t *p_output,
                    void *userData) {
  if (status == JPEG_JOB_STATUS_ERROR) {
    ALOGE("%s: Encoder ran into an error", __func__);
  } else {
    JpegEncoder::EncodeCb(p_output, userData);
  }
}

JpegEncoder *JpegEncoder::getInstance() {
  if (!JpegEncoder::encoder_instance_) {
      JpegEncoder::encoder_instance_ = new JpegEncoder();
  }
  return JpegEncoder::encoder_instance_;
}

void JpegEncoder::releaseInstance() {
  delete JpegEncoder::encoder_instance_;
  JpegEncoder::encoder_instance_ = nullptr;
}

JpegEncoder::JpegEncoder() :
    cfg_(NULL),
    job_result_ptr_(NULL),
    job_result_size_(0),
    libjpeg_interface_(nullptr) {
  cfg_ = new JpegEncoderParams;
  JE_GET_PARAMS(cfg);
  cfg->handle_ = 0;
  cfg->handle_ = 0;
  cfg->job_id_ = 0;

  libjpeg_interface_ = dlopen("libmmjpeg_interface.so", RTLD_NOW);
  if (!libjpeg_interface_) {
    ALOGE("%s: could not open jpeg library", __func__);
  } else {
    cfg->jpeg_open_proc_ =
        (jpeg_open_proc_t)dlsym(libjpeg_interface_, "jpeg_open");
    if(!cfg->jpeg_open_proc_) {
      ALOGE("%s: could not dlsym jpeg_open", __func__);
    }
  }

  // setup internal config structures. performed only once
  cfg->params_ = {};
  cfg->job_ = {};

  cfg->params_.jpeg_cb = EncodeCbGlobal;
  cfg->params_.userdata = this;

  cfg->params_.num_dst_bufs = 1;
  cfg->params_.dest_buf[0].buf_vaddr = nullptr;
  cfg->params_.dest_buf[0].fd = -1;
  cfg->params_.dest_buf[0].index = 0;

  cfg->params_.num_src_bufs = 1;
  cfg->params_.num_tmb_bufs = 0;

  cfg->params_.encode_thumbnail = 0;
  cfg->params_.encode_second_thumbnail = 0;
  if (cfg->params_.encode_thumbnail) {
    cfg->params_.num_tmb_bufs = cfg->params_.num_src_bufs;
  }
  cfg->params_.thumb_quality = 75;

  cfg->job_.encode_job.dst_index = 0;
  cfg->job_.encode_job.src_index = 0;
  cfg->job_.encode_job.rotation = 0;

  cfg->job_.encode_job.exif_info.numOfEntries = 0;
  cfg->params_.burst_mode = 0;

  /* Qtable */
  cfg->job_.encode_job.qtable[0].eQuantizationTable =
      OMX_IMAGE_QuantizationTableLuma;
  cfg->job_.encode_job.qtable[1].eQuantizationTable =
      OMX_IMAGE_QuantizationTableChroma;
  cfg->job_.encode_job.qtable_set[0] = 1;
  cfg->job_.encode_job.qtable_set[1] = 1;

  for (int i = 0; i < (int)sizeof(JpegEncoder::DEFAULT_QTABLE_0); i++) {
    cfg->job_.encode_job.qtable[0].nQuantizationMatrix[i] =
        JpegEncoder::DEFAULT_QTABLE_0[i];
    cfg->job_.encode_job.qtable[1].nQuantizationMatrix[i] =
        JpegEncoder::DEFAULT_QTABLE_1[i];
  }

  cfg->job_.job_type = JPEG_JOB_TYPE_ENCODE;
  cfg->job_.encode_job.src_index = 0;
  cfg->job_.encode_job.dst_index = 0;
  cfg->job_.encode_job.thumb_index = 0;
}

JpegEncoder::~JpegEncoder() {
  JE_GET_PARAMS(cfg);
  std::lock_guard<std::mutex> al(cfg->encode_lock_);
  if (nullptr != libjpeg_interface_) {
    dlclose(libjpeg_interface_);
  }
  if (cfg) {
    delete cfg;
    cfg = nullptr;
  }
}

void JpegEncoder::FillImgData(const CameraBufferMetaData& source_info) {
  JE_GET_PARAMS(cfg);

  size_t size = source_info.plane_info[0].stride *
                source_info.plane_info[0].scanline;
  cfg->params_.src_main_buf[0].buf_size = 3 * size / 2;
  cfg->params_.src_main_buf[0].format = MM_JPEG_FMT_YUV;
  cfg->params_.src_main_buf[0].fd = -1;
  cfg->params_.src_main_buf[0].index = 0;
  cfg->params_.src_main_buf[0].offset.mp[0].len = (uint32_t)size;
  cfg->params_.src_main_buf[0].offset.mp[0].stride = source_info.plane_info[0].stride;
  cfg->params_.src_main_buf[0].offset.mp[0].scanline = source_info.plane_info[0].scanline;
  cfg->params_.src_main_buf[0].offset.mp[1].len = (uint32_t)(size >> 1);

  cfg->params_.src_thumb_buf[0] = cfg->params_.src_main_buf[0];

  switch (source_info.format) {
    case BufferFormat::kNV12:
      cfg->params_.color_format = MM_JPEG_COLOR_FORMAT_YCBCRLP_H2V2;
      break;
    case BufferFormat::kNV21:
      cfg->params_.color_format = MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V2;
      break;
    default:
      break;
  }
  cfg->params_.thumb_color_format = cfg->params_.color_format;

  cfg->params_.dest_buf[0].buf_size = cfg->params_.src_main_buf[0].buf_size;

  cfg->job_.encode_job.main_dim.src_dim.width = source_info.plane_info[0].stride;
  cfg->job_.encode_job.main_dim.src_dim.height = source_info.plane_info[0].scanline;
  cfg->job_.encode_job.main_dim.dst_dim.width = source_info.plane_info[0].width;
  cfg->job_.encode_job.main_dim.dst_dim.height = source_info.plane_info[0].height;
  cfg->job_.encode_job.main_dim.crop.top = 0;
  cfg->job_.encode_job.main_dim.crop.left = 0;
  cfg->job_.encode_job.main_dim.crop.width = source_info.plane_info[0].width;
  cfg->job_.encode_job.main_dim.crop.height = source_info.plane_info[0].height;
  cfg->params_.main_dim = cfg->job_.encode_job.main_dim;

  cfg->job_.encode_job.thumb_dim.src_dim.width = source_info.plane_info[0].stride;
  cfg->job_.encode_job.thumb_dim.src_dim.height = source_info.plane_info[0].scanline;
  cfg->job_.encode_job.thumb_dim.dst_dim.width = 960;
  cfg->job_.encode_job.thumb_dim.dst_dim.height = 480;
  cfg->job_.encode_job.thumb_dim.crop.top = 0;
  cfg->job_.encode_job.thumb_dim.crop.left = 0;
  cfg->job_.encode_job.thumb_dim.crop.width = 0;
  cfg->job_.encode_job.thumb_dim.crop.height = 0;
  cfg->params_.thumb_dim = cfg->job_.encode_job.thumb_dim;

  cfg->job_.encode_job.second_thumb_dim.src_dim.width = source_info.plane_info[0].stride;
  cfg->job_.encode_job.second_thumb_dim.src_dim.height = source_info.plane_info[0].scanline;
  cfg->job_.encode_job.second_thumb_dim.dst_dim.width = 320;
  cfg->job_.encode_job.second_thumb_dim.dst_dim.height = 240;
  cfg->job_.encode_job.second_thumb_dim.crop.top = 0;
  cfg->job_.encode_job.second_thumb_dim.crop.left = 0;
  cfg->job_.encode_job.second_thumb_dim.crop.width = 0;
  cfg->job_.encode_job.second_thumb_dim.crop.height = 0;
  cfg->params_.second_thumb_dim = cfg->job_.encode_job.second_thumb_dim;

  cfg->pic_size_.w = source_info.plane_info[0].width;
  cfg->pic_size_.h = source_info.plane_info[0].height;

  cfg->job_.encode_job.hal_version = CAM_HAL_V1;
  cfg->job_.encode_job.cam_exif_params.sensor_params.sens_type =
      (cam_sensor_t) 1;  /* CAM_SENSOR_YUV */
}

void JpegEncoder::UpdateThumbnailData(const snapshot_info& snapshot) {
  JE_GET_PARAMS(cfg);

  auto &thumbnail_data = snapshot.thumbnails;
  if (!thumbnail_data.empty()) {
    auto thumb_cnt = thumbnail_data.size();
    if (thumb_cnt > 2) {
      ALOGE("%s: Max supported thumbnails is 2. In is %d", __func__, thumb_cnt);
      /* clip to 2 */
      thumb_cnt = 2;
    }
    /* validate data */
    for (uint32_t i = 0; i < thumb_cnt; i++) {
      if ((thumbnail_data[i].width > snapshot.source_info.plane_info[0].width) ||
          (thumbnail_data[i].width == 0) ||
          (thumbnail_data[i].height > snapshot.source_info.plane_info[0].height) ||
          (thumbnail_data[i].height == 0) ||
          (thumbnail_data[i].thumb_quality > 100) ||
          (thumbnail_data[i].thumb_quality == 0)) {
        ALOGE("%s: Invalid thumbnail input paramethers (%d x %d %d)", __func__,
            thumbnail_data[i].width, thumbnail_data[i].height,
            thumbnail_data[i].thumb_quality);
        return;
      }
    }

    for (uint32_t i = 0; i < thumb_cnt; i++) {
      ALOGI("%s: thumbnail input paramethers (%d x %d %d)", __func__,
          thumbnail_data[i].width, thumbnail_data[i].height,
          thumbnail_data[i].thumb_quality);
    }
    cfg->params_.encode_thumbnail = 1;
    cfg->params_.num_tmb_bufs = cfg->params_.num_src_bufs;

    cfg->job_.encode_job.thumb_dim.src_dim.width =
        snapshot.source_info.plane_info[0].stride;
    cfg->job_.encode_job.thumb_dim.src_dim.height =
        snapshot.source_info.plane_info[0].scanline;
    cfg->job_.encode_job.thumb_dim.dst_dim.width = thumbnail_data[0].width;
    cfg->job_.encode_job.thumb_dim.dst_dim.height = thumbnail_data[0].height;
    cfg->job_.encode_job.thumb_dim.crop.top = 0;
    cfg->job_.encode_job.thumb_dim.crop.left = 0;
    cfg->job_.encode_job.thumb_dim.crop.width = 0;
    cfg->job_.encode_job.thumb_dim.crop.height = 0;
    cfg->params_.thumb_dim = cfg->job_.encode_job.thumb_dim;

    if (thumb_cnt == 2) {
      cfg->params_.encode_second_thumbnail = 1;

      cfg->job_.encode_job.second_thumb_dim.src_dim.width =
          snapshot.source_info.plane_info[0].stride;
      cfg->job_.encode_job.second_thumb_dim.src_dim.height =
          snapshot.source_info.plane_info[0].scanline;
      cfg->job_.encode_job.second_thumb_dim.dst_dim.width =
          thumbnail_data[1].width;
      cfg->job_.encode_job.second_thumb_dim.dst_dim.height =
          thumbnail_data[1].height;
      cfg->job_.encode_job.second_thumb_dim.crop.top = 0;
      cfg->job_.encode_job.second_thumb_dim.crop.left = 0;
      cfg->job_.encode_job.second_thumb_dim.crop.width = 0;
      cfg->job_.encode_job.second_thumb_dim.crop.height = 0;
      cfg->params_.second_thumb_dim = cfg->job_.encode_job.second_thumb_dim;
      ALOGI("%s: Encode second thumbnail is enabled", __func__);
    }
  }
}

void *JpegEncoder::Encode(const snapshot_info& in_buffer, size_t &jpeg_size,
                          const uint32_t quality) {
  JE_GET_PARAMS(cfg);
  std::lock_guard<std::mutex> al(cfg->encode_lock_);
  job_result_ptr_ = NULL;

  if (!in_buffer.img_data[0] || !in_buffer.out_data[0]) {
    ALOGE("%s: can't pass NULL plane pointer", __func__);
    goto jpeg_encode_exit;
  }

  FillImgData(in_buffer.source_info);
  cfg->params_.quality = quality;
  cfg->params_.src_main_buf[0].buf_vaddr = in_buffer.img_data[0];
  cfg->params_.src_thumb_buf[0].buf_vaddr = in_buffer.img_data[0];
  cfg->params_.dest_buf[0].buf_vaddr = in_buffer.out_data[0];
  UpdateThumbnailData(in_buffer);

  cfg->job_id_ = 0;

  cfg->job_.encode_job.exif_info.numOfEntries = in_buffer.exif_size;
  cfg->job_.encode_job.exif_info.exif_data =
      reinterpret_cast<QEXIF_INFO_DATA*>(in_buffer.exif_data);

  cfg->handle_ = cfg->jpeg_open_proc_(&cfg->ops_, NULL, cfg->pic_size_, NULL);
  if (cfg->handle_ == 0) {
    ALOGE("%s: could not open a jpeg handle", __func__);
    goto jpeg_encode_exit;
  }

  cfg->ops_.create_session(cfg->handle_,
                           &cfg->params_,
                           &cfg->job_.encode_job.session_id);
  if (cfg->job_.encode_job.session_id == 0) {
    ALOGE("%s: could not create jpeg session", __func__);
    goto jpeg_encode_exit;
  }

  if (!cfg->ops_.start_job(&cfg->job_, &cfg->job_id_)) {
    std::unique_lock<std::mutex> ul(cfg->enc_done_lock_);
    cfg->enc_done_cond_.Wait(ul);
    ul.unlock();
    jpeg_size = job_result_size_;

  } else {
    ALOGE("%s: could not start encode job", __func__);
    goto jpeg_encode_exit;
  }

jpeg_encode_exit:

  if(cfg->job_.encode_job.session_id) {
    cfg->ops_.destroy_session(cfg->job_.encode_job.session_id);
  }

  if (cfg->handle_) {
    cfg->ops_.close(cfg->handle_);
    cfg->handle_ = 0;
  }

  return job_result_ptr_;
}

void JpegEncoder::EncodeCb(void *p_output, void *userData) {
  JpegEncoder *enc = (JpegEncoder *)userData;
  mm_jpeg_output_t *output = (mm_jpeg_output_t *)p_output;

  enc->job_result_ptr_ = output->buf_vaddr;
  enc->job_result_size_ = output->buf_filled_len;

  JpegEncoderParams *cfg = (JpegEncoderParams *)enc->cfg_;
  cfg->enc_done_cond_.Signal();
}

} //namespace jpegencoder ends here

} //namespace qmmf ends here
