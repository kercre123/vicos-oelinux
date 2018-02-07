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

#include <qmmf_jpeg_encoder.h>
#include <dlfcn.h>
#include <cutils/properties.h>
#include <mutex>
#include <condition_variable>
#include <mm_jpeg_interface.h>

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
  std::condition_variable enc_done_cond_;
} JpegEncoderParams;

#define JE_GET_PARAMS(x) JpegEncoderParams *(x) = (JpegEncoderParams *)cfg_

namespace qmmf {

namespace jpegencoder {

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


void OnEncodeDoneGlobal(jpeg_job_status_t status, uint32_t /*client_hdl*/,
                        uint32_t /*jobId*/, mm_jpeg_output_t *p_output,
                        void *userData) {
  if (status == JPEG_JOB_STATUS_ERROR) {
    ALOGE("%s: Encoder ran into an error", __func__);
  } else {
    JpegEncoder::OnEncodeDone(p_output, userData);
  }
}

JpegEncoder::JpegEncoder(struct JpegEncoderConfiguration *params) :
    cfg_(nullptr),
    job_result_ptr_(nullptr),
    job_result_size_(0) {
  cfg_ = new JpegEncoderParams;
  JE_GET_PARAMS(cfg);
  cfg->handle_ = 0;
  cfg->handle_ = 0;
  cfg->job_id_ = 0;

  void *libjpeg_interface = dlopen("libmmjpeg_interface.so", RTLD_NOW);
  if(!libjpeg_interface) {
    ALOGE("%s: could not open jpeg library", __func__);
  } else {
    cfg->jpeg_open_proc_ = (jpeg_open_proc_t)dlsym(libjpeg_interface, "jpeg_open");
    if(!cfg->jpeg_open_proc_) {
      ALOGE("%s: could not dlsym jpeg_open", __func__);
    }
  }

  // setup internal config structures. performed only once
  memset(&cfg->params_, 0, sizeof(cfg->params_));
  memset(&cfg->job_, 0, sizeof(cfg->job_));

  size_t size = params->stride[0] * params->scanline[0];
  cfg->params_.src_main_buf[0].buf_size = 3 * size / 2;
  cfg->params_.src_main_buf[0].format = MM_JPEG_FMT_YUV;
  cfg->params_.src_main_buf[0].fd = -1;
  cfg->params_.src_main_buf[0].index = 0;
  cfg->params_.src_main_buf[0].offset.mp[0].len = (uint32_t)size;
  cfg->params_.src_main_buf[0].offset.mp[0].stride = params->stride[0];
  cfg->params_.src_main_buf[0].offset.mp[0].scanline = params->scanline[0];
  cfg->params_.src_main_buf[0].offset.mp[1].len = (uint32_t)(size >> 1);

  cfg->params_.src_thumb_buf[0] = cfg->params_.src_main_buf[0];

  cfg->params_.jpeg_cb = OnEncodeDoneGlobal;
  cfg->params_.userdata = this;

  switch (params->format) {
    case JpegEncoderFormat::NV12:
      cfg->params_.color_format = MM_JPEG_COLOR_FORMAT_YCBCRLP_H2V2;
      break;
    case JpegEncoderFormat::NV21:
      cfg->params_.color_format = MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V2;
      break;
    default:
      break;
  }
  cfg->params_.thumb_color_format = cfg->params_.color_format;

  cfg->params_.num_dst_bufs = 1;
  cfg->params_.dest_buf[0].buf_size = cfg->params_.src_main_buf[0].buf_size;
  cfg->params_.dest_buf[0].buf_vaddr = new uint8_t[cfg->params_.dest_buf[0].buf_size];
  cfg->params_.dest_buf[0].fd = -1;
  cfg->params_.dest_buf[0].index = 0;

  cfg->params_.num_src_bufs = 1;
  cfg->params_.num_tmb_bufs = 0;

  cfg->params_.encode_thumbnail = 0;
  if (cfg->params_.encode_thumbnail) {
    cfg->params_.num_tmb_bufs = cfg->params_.num_src_bufs;
  }

  {
    char property[PROPERTY_VALUE_MAX];

    if (property_get(JPEG_QUALITY_PROP, property, NULL) > 0) {
      cfg->params_.quality = atoi(property);
    } else {
      cfg->params_.quality = JPEG_DEFAULT_QUALITY;
    }

    if (property_get(JPEG_THUMBNAIL_QUALITY_PROP, property, NULL) > 0) {
      cfg->params_.thumb_quality = atoi(property);
    } else {
      cfg->params_.thumb_quality = JPEG_THUMBNAIL_DEFAULT_QUALITY;
    }
  }

  cfg->job_.encode_job.dst_index = 0;
  cfg->job_.encode_job.src_index = 0;
  cfg->job_.encode_job.rotation = 0;

  cfg->job_.encode_job.main_dim.src_dim.width = params->stride[0];
  cfg->job_.encode_job.main_dim.src_dim.height = params->scanline[0];
  cfg->job_.encode_job.main_dim.dst_dim.width = params->width[0];
  cfg->job_.encode_job.main_dim.dst_dim.height = params->height[0];
  cfg->job_.encode_job.main_dim.crop.top = 0;
  cfg->job_.encode_job.main_dim.crop.left = 0;
  cfg->job_.encode_job.main_dim.crop.width = params->width[0];
  cfg->job_.encode_job.main_dim.crop.height = params->height[0];
  cfg->params_.main_dim = cfg->job_.encode_job.main_dim;

  cfg->job_.encode_job.thumb_dim.src_dim.width = params->stride[0];
  cfg->job_.encode_job.thumb_dim.src_dim.height = params->scanline[0];
  cfg->job_.encode_job.thumb_dim.dst_dim.width = JPEG_THUMBNAIL_WIDTH;
  cfg->job_.encode_job.thumb_dim.dst_dim.height = JPEG_THUMBNAIL_HEIGHT;
  cfg->job_.encode_job.thumb_dim.crop.top = 0;
  cfg->job_.encode_job.thumb_dim.crop.left = 0;
  cfg->job_.encode_job.thumb_dim.crop.width = 0;
  cfg->job_.encode_job.thumb_dim.crop.height = 0;
  cfg->params_.thumb_dim = cfg->job_.encode_job.thumb_dim;

  cfg->job_.encode_job.exif_info.numOfEntries = 0;
  cfg->params_.burst_mode = 0;

  /* Qtable */
  cfg->job_.encode_job.qtable[0].eQuantizationTable = OMX_IMAGE_QuantizationTableLuma;
  cfg->job_.encode_job.qtable[1].eQuantizationTable = OMX_IMAGE_QuantizationTableChroma;
  cfg->job_.encode_job.qtable_set[0] = 1;
  cfg->job_.encode_job.qtable_set[1] = 1;

  for (int i = 0; i < (int)sizeof(JpegEncoder::DEFAULT_QTABLE_0); i++) {
    cfg->job_.encode_job.qtable[0].nQuantizationMatrix[i] = JpegEncoder::DEFAULT_QTABLE_0[i];
    cfg->job_.encode_job.qtable[1].nQuantizationMatrix[i] = JpegEncoder::DEFAULT_QTABLE_1[i];
  }

  cfg->pic_size_.w = params->width[0];
  cfg->pic_size_.h = params->height[0];

  cfg->job_.job_type = JPEG_JOB_TYPE_ENCODE;
  cfg->job_.encode_job.src_index = 0;
  cfg->job_.encode_job.dst_index = 0;
  cfg->job_.encode_job.thumb_index = 0;
}

JpegEncoder::~JpegEncoder() {
  JE_GET_PARAMS(cfg);
  std::lock_guard<std::mutex> al(cfg->encode_lock_);

  for (uint32_t i = 0; i < cfg->params_.num_dst_bufs; i++) {
    delete [] cfg->params_.dest_buf[i].buf_vaddr;
  }
}

void *JpegEncoder::Encode(struct JpegFrameInfo *pFrame, size_t *jpeg_size, float destScale) {
  JE_GET_PARAMS(cfg);
  std::lock_guard<std::mutex> al(cfg->encode_lock_);
  job_result_ptr_ = nullptr;

  uint32_t dst_width = cfg->job_.encode_job.main_dim.src_dim.width;
  uint32_t dst_height = cfg->job_.encode_job.main_dim.src_dim.height;

  if (!pFrame) {
    ALOGE("%s: can't pass NULL input pointer", __func__);
    goto jpeg_encode_exit;
  }

  if (!pFrame->plane_addr[0]) {
    ALOGE("%s: can't pass NULL plane pointer", __func__);
    goto jpeg_encode_exit;
  }

  if (destScale > 1.0f) {
    if (destScale > JPEG_MAX_DESTINATION_SCALE) {
      destScale = JPEG_MAX_DESTINATION_SCALE;
    }
    dst_width = (uint32_t)((float)dst_width / destScale);
    dst_height = (uint32_t)((float)dst_height / destScale);
    if (!dst_width || !dst_height) {
      dst_width = cfg->job_.encode_job.main_dim.src_dim.width;
      dst_height = cfg->job_.encode_job.main_dim.src_dim.height;
    }
  }

  cfg->job_.encode_job.main_dim.dst_dim.width = dst_width;
  cfg->job_.encode_job.main_dim.dst_dim.height = dst_height;

  cfg->params_.src_main_buf[0].buf_vaddr = pFrame->plane_addr[0];
  cfg->params_.src_thumb_buf[0].buf_vaddr = pFrame->plane_addr[0];
  cfg->job_id_ = 0;

  cfg->handle_ = cfg->jpeg_open_proc_(&cfg->ops_, nullptr, cfg->pic_size_, nullptr);
  if (cfg->handle_ == 0) {
    ALOGE("%s: could not open a jpeg handle", __func__);
    goto jpeg_encode_exit;
  }

  cfg->ops_.create_session(cfg->handle_, &cfg->params_, &cfg->job_.encode_job.session_id);
  if (cfg->job_.encode_job.session_id == 0) {
    ALOGE("%s: could not create jpeg session", __func__);
    goto jpeg_encode_exit;
  }

  if (!cfg->ops_.start_job(&cfg->job_, &cfg->job_id_)) {
    std::unique_lock<std::mutex> ul(cfg->enc_done_lock_);
    cfg->enc_done_cond_.wait(ul);

    if (jpeg_size) {
      *jpeg_size = job_result_size_;
    }
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

void JpegEncoder::OnEncodeDone(void *p_output, void *userData) {
  JpegEncoder *enc = (JpegEncoder *)userData;
  mm_jpeg_output_t *output = (mm_jpeg_output_t *)p_output;

  enc->job_result_ptr_ = output->buf_vaddr;
  enc->job_result_size_ = output->buf_filled_len;

  JpegEncoderParams *cfg = (JpegEncoderParams *)enc->cfg_;
  cfg->enc_done_cond_.notify_one();
}

} //namespace jpegencoder ends here
} //namespace qmmf ends here
