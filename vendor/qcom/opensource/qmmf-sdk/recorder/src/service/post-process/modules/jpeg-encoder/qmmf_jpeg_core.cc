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

#define TAG "RecorderJpeg"

#include <condition_variable>
#include <dlfcn.h>
#include <hardware/camera3.h>
#include <mm_jpeg_interface.h>
#include <mutex>
#include <qmmf_jpeg_core.h>
#include <utils/Log.h>

typedef uint32_t (*jpeg_open_proc_t)(mm_jpeg_ops_t *,
                                     mm_jpeg_mpo_ops_t *,
                                     mm_dimension,
                                     cam_related_system_calibration_data_t *);

typedef struct {
  jpeg_open_proc_t jpeg_open_proc;
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

namespace reprocjpegencoder {

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
    JpegEncoder *enc = (JpegEncoder *)userData;
    if (enc != nullptr) {
      enc->EncodeCb(p_output, userData);
    }
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
    cfg->jpeg_open_proc = (jpeg_open_proc_t)dlsym(libjpeg_interface_, "jpeg_open");
    if (!cfg->jpeg_open_proc) {
      ALOGE("%s: could not dlsym jpeg_open", __func__);
    }
  }

  // setup internal config structures. performed only once
  memset(&cfg->params_, 0, sizeof(cfg->params_));
  memset(&cfg->job_, 0, sizeof(cfg->job_));

  cfg->params_.jpeg_cb = EncodeCbGlobal;
  cfg->params_.userdata = this;

  cfg->params_.num_dst_bufs = 1;

  cfg->params_.dest_buf[0].buf_vaddr = nullptr;
  cfg->params_.dest_buf[0].fd = -1;
  cfg->params_.dest_buf[0].index = 0;

  cfg->params_.num_src_bufs = 1;
  cfg->params_.num_tmb_bufs = 0;

  cfg->params_.encode_thumbnail = 0;
  if (cfg->params_.encode_thumbnail) {
    cfg->params_.num_tmb_bufs = cfg->params_.num_src_bufs;
  }
  cfg->params_.quality = 85;
  cfg->params_.thumb_quality = 75;

  cfg->job_.encode_job.dst_index = 0;
  cfg->job_.encode_job.src_index = 0;
  cfg->job_.encode_job.rotation = 0;

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

  cfg->job_.job_type = JPEG_JOB_TYPE_ENCODE;
  cfg->job_.encode_job.src_index = 0;
  cfg->job_.encode_job.dst_index = 0;
  cfg->job_.encode_job.thumb_index = 0;
}

JpegEncoder::~JpegEncoder() {
  JE_GET_PARAMS(cfg);
  /* std::lock_guard<std::mutex> al(cfg->encode_lock_); */
  delete cfg;
  if (nullptr != libjpeg_interface_) {
    dlclose(libjpeg_interface_);
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
  cfg->job_.encode_job.thumb_dim.dst_dim.width = 320;
  cfg->job_.encode_job.thumb_dim.dst_dim.height = 240;
  cfg->job_.encode_job.thumb_dim.crop.top = 0;
  cfg->job_.encode_job.thumb_dim.crop.left = 0;
  cfg->job_.encode_job.thumb_dim.crop.width = 0;
  cfg->job_.encode_job.thumb_dim.crop.height = 0;
  cfg->params_.thumb_dim = cfg->job_.encode_job.thumb_dim;

  cfg->pic_size_.w = source_info.plane_info[0].width;
  cfg->pic_size_.h = source_info.plane_info[0].height;
}

void *JpegEncoder::Encode(size_t *jpeg_size) {
  JE_GET_PARAMS(cfg);
  std::lock_guard<std::mutex> al(cfg->encode_lock_);
  job_result_ptr_ = NULL;

  if (!in_buffer_.img_data[0] || !in_buffer_.out_data[0]) {
    ALOGE("%s: can't pass NULL plane pointer", __func__);
    goto jpeg_encode_exit;
  }

  FillImgData(in_buffer_.source_info);
  cfg->params_.src_main_buf[0].buf_vaddr = in_buffer_.img_data[0];
  cfg->params_.src_thumb_buf[0].buf_vaddr = in_buffer_.img_data[0];
  cfg->params_.dest_buf[0].buf_vaddr = in_buffer_.out_data[0];
  cfg->job_id_ = 0;

  cfg->handle_ = cfg->jpeg_open_proc(&cfg->ops_, NULL, cfg->pic_size_, NULL);
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
    ul.unlock();

    if (jpeg_size) {
      /* add a valid jpeg header */
      camera3_jpeg_blob_t jpegHeader;
      jpegHeader.jpeg_blob_id = CAMERA3_JPEG_BLOB_ID;
      jpegHeader.jpeg_size = (uint32_t) job_result_size_;
      uint8_t *jpegEof = &cfg->params_.dest_buf[0].buf_vaddr[job_result_size_];
      memcpy(jpegEof, &jpegHeader, sizeof(jpegHeader));

      *jpeg_size = job_result_size_ + sizeof(jpegHeader) + 1;
    }
  } else {
    ALOGE("%s: could not start encode job", __func__);
    goto jpeg_encode_exit;
  }

jpeg_encode_exit:

  if (cfg->job_.encode_job.session_id) {
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
  cfg->enc_done_cond_.notify_one();
}

} //namespace reprocjpegencoder ends here

} //namespace qmmf ends here
