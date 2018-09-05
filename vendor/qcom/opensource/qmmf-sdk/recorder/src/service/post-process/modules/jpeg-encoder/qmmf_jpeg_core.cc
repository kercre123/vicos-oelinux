/*
* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "RecorderJpeg"

#include <mutex>
#include <dlfcn.h>

#include <qmmf_jpeg_core.h>
#include "common/utils/qmmf_log.h"


using namespace qmmf;

namespace qmmf {

namespace reprocjpegencoder {


const uint32_t JpegEncoder::kDefaultMainThumbWidth  = 960;
const uint32_t JpegEncoder::kDefaultMainThumbHeight = 480;

const uint32_t JpegEncoder::kDefaultSecondThumbWidth  = 320;
const uint32_t JpegEncoder::kDefaultSecondThumbHeight = 240;

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
    JpegEncoder *enc = static_cast<JpegEncoder *>(userData);
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
    job_result_ptr_(NULL),
    job_result_size_(0),
    libjpeg_interface_(nullptr) {
  cfg_.handle_ = 0;
  cfg_.handle_ = 0;
  cfg_.job_id_ = 0;

  libjpeg_interface_ = dlopen("libmmjpeg_interface.so", RTLD_NOW);
  if (!libjpeg_interface_) {
    ALOGE("%s: could not open jpeg library", __func__);
  } else {
    cfg_.jpeg_open_proc = (jpeg_open_proc_t)dlsym(libjpeg_interface_, "jpeg_open");
    if (!cfg_.jpeg_open_proc) {
      ALOGE("%s: could not dlsym jpeg_open", __func__);
    }
  }

  // setup internal config structures. performed only once
  memset(&cfg_.params_, 0, sizeof(cfg_.params_));
  memset(&cfg_.job_, 0, sizeof(cfg_.job_));

  cfg_.params_.jpeg_cb = EncodeCbGlobal;
  cfg_.params_.userdata = this;

  cfg_.params_.num_dst_bufs = 1;

  cfg_.params_.dest_buf[0].buf_vaddr = nullptr;
  cfg_.params_.dest_buf[0].fd = -1;
  cfg_.params_.dest_buf[0].index = 0;

  cfg_.params_.num_src_bufs = 1;
  cfg_.params_.num_tmb_bufs = 0;

  cfg_.params_.encode_thumbnail = 0;
  cfg_.params_.encode_second_thumbnail = 0;

  cfg_.params_.quality = 85;
  cfg_.params_.thumb_quality = 75;

  cfg_.job_.encode_job.dst_index = 0;
  cfg_.job_.encode_job.src_index = 0;
  cfg_.job_.encode_job.rotation = 0;

  cfg_.job_.encode_job.exif_info.numOfEntries = 0;
  cfg_.params_.burst_mode = 0;

  /* Qtable */
  cfg_.job_.encode_job.qtable[0].eQuantizationTable = OMX_IMAGE_QuantizationTableLuma;
  cfg_.job_.encode_job.qtable[1].eQuantizationTable = OMX_IMAGE_QuantizationTableChroma;
  cfg_.job_.encode_job.qtable_set[0] = 1;
  cfg_.job_.encode_job.qtable_set[1] = 1;

  for (int i = 0; i < (int)sizeof(JpegEncoder::DEFAULT_QTABLE_0); i++) {
    cfg_.job_.encode_job.qtable[0].nQuantizationMatrix[i] = JpegEncoder::DEFAULT_QTABLE_0[i];
    cfg_.job_.encode_job.qtable[1].nQuantizationMatrix[i] = JpegEncoder::DEFAULT_QTABLE_1[i];
  }

  cfg_.job_.job_type = JPEG_JOB_TYPE_ENCODE;
  cfg_.job_.encode_job.src_index = 0;
  cfg_.job_.encode_job.dst_index = 0;
  cfg_.job_.encode_job.thumb_index = 0;
}

JpegEncoder::~JpegEncoder() {
  if (nullptr != libjpeg_interface_) {
    dlclose(libjpeg_interface_);
  }
}

int32_t JpegEncoder::ConfigureMainImage(const encode_params &params) {

  auto &plane_info = params.source_info.plane_info[0];

  size_t size = plane_info.stride * plane_info.scanline;

  cfg_.params_.src_main_buf[0].buf_size = 3 * size / 2;
  cfg_.params_.src_main_buf[0].format = MM_JPEG_FMT_YUV;
  cfg_.params_.src_main_buf[0].fd = -1;
  cfg_.params_.src_main_buf[0].index = 0;
  cfg_.params_.src_main_buf[0].offset.mp[0].len = static_cast<uint32_t>(size);
  cfg_.params_.src_main_buf[0].offset.mp[0].stride = plane_info.stride;
  cfg_.params_.src_main_buf[0].offset.mp[0].scanline = plane_info.scanline;
  cfg_.params_.src_main_buf[0].offset.mp[1].len =
      static_cast<uint32_t>(size >> 1);

  cfg_.params_.src_thumb_buf[0] = cfg_.params_.src_main_buf[0];

  switch (params.source_info.format) {
    case BufferFormat::kNV12:
      cfg_.params_.color_format = MM_JPEG_COLOR_FORMAT_YCBCRLP_H2V2;
      break;
    case BufferFormat::kNV21:
      cfg_.params_.color_format = MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V2;
      break;
    default:
      break;
  }

  cfg_.params_.thumb_color_format = cfg_.params_.color_format;

  cfg_.params_.dest_buf[0].buf_size = cfg_.params_.src_main_buf[0].buf_size;

  cfg_.job_.encode_job.main_dim.src_dim.width = plane_info.stride;
  cfg_.job_.encode_job.main_dim.src_dim.height = plane_info.scanline;
  cfg_.job_.encode_job.main_dim.dst_dim.width = plane_info.width;
  cfg_.job_.encode_job.main_dim.dst_dim.height = plane_info.height;
  cfg_.job_.encode_job.main_dim.crop.top = 0;
  cfg_.job_.encode_job.main_dim.crop.left = 0;
  cfg_.job_.encode_job.main_dim.crop.width = plane_info.width;
  cfg_.job_.encode_job.main_dim.crop.height = plane_info.height;
  cfg_.params_.main_dim = cfg_.job_.encode_job.main_dim;
  cfg_.params_.quality = params.image_quality;

  return 0;
}

int32_t JpegEncoder::ConfigureThumbnails(const encode_params &params) {
  cfg_.params_.encode_thumbnail = 0;
  cfg_.params_.encode_second_thumbnail = 0;

  auto &plane_info = params.source_info.plane_info[0];
  auto &thumbnail_data = params.thumbnail_data;
  if (thumbnail_data.empty()) {
    QMMF_VERBOSE("%s: no thumbnails", __func__);
    return 0;
  }

  auto thumb_cnt = thumbnail_data.size();
  if (thumb_cnt > 2) {
    QMMF_ERROR("%s: Max supported thumbnails is 2. In is %d", __func__,
        thumb_cnt);
    // clip to 2
    thumb_cnt = 2;
  }

  // validate data
  for (uint32_t i = 0; i < thumb_cnt; i++) {
    if ((thumbnail_data[i].width > plane_info.width) ||
        (thumbnail_data[i].width == 0) ||
        (thumbnail_data[i].height > plane_info.height) ||
        (thumbnail_data[i].height == 0) ||
        (thumbnail_data[i].thumb_quality > 100) ||
        (thumbnail_data[i].thumb_quality == 0)) {
      QMMF_ERROR("%s: Invalid thumbnail input paramethers (%d x %d %d)",
          __func__, thumbnail_data[i].width, thumbnail_data[i].height,
          thumbnail_data[i].thumb_quality);
      return -1;
    }
  }

  for (uint32_t i = 0; i < thumb_cnt; i++) {
    QMMF_VERBOSE("%s: thumbnail input paramethers (%d x %d %d)", __func__,
        thumbnail_data[i].width, thumbnail_data[i].height,
        thumbnail_data[i].thumb_quality);
  }

  cfg_.params_.encode_thumbnail = 1;
  cfg_.params_.num_tmb_bufs = cfg_.params_.num_src_bufs;

  cfg_.job_.encode_job.thumb_dim.src_dim.width = plane_info.stride;
  cfg_.job_.encode_job.thumb_dim.src_dim.height = plane_info.scanline;
  cfg_.job_.encode_job.thumb_dim.dst_dim.width = thumbnail_data[0].width;
  cfg_.job_.encode_job.thumb_dim.dst_dim.height = thumbnail_data[0].height;
  cfg_.job_.encode_job.thumb_dim.crop.top = 0;
  cfg_.job_.encode_job.thumb_dim.crop.left = 0;
  cfg_.job_.encode_job.thumb_dim.crop.width = plane_info.width;
  cfg_.job_.encode_job.thumb_dim.crop.height = plane_info.height;
  cfg_.params_.thumb_dim = cfg_.job_.encode_job.thumb_dim;
  cfg_.params_.thumb_quality = thumbnail_data[0].thumb_quality;
  QMMF_INFO("%s: Encode thumbnail is enabled", __func__);

  if (thumb_cnt < 2) {
    return 0;
  }

  cfg_.params_.encode_second_thumbnail = 1;
  cfg_.job_.encode_job.second_thumb_dim.src_dim.width = plane_info.stride;
  cfg_.job_.encode_job.second_thumb_dim.src_dim.height = plane_info.scanline;
  cfg_.job_.encode_job.second_thumb_dim.dst_dim.width = thumbnail_data[1].width;
  cfg_.job_.encode_job.second_thumb_dim.dst_dim.height = thumbnail_data[1].height;
  cfg_.job_.encode_job.second_thumb_dim.crop.top = 0;
  cfg_.job_.encode_job.second_thumb_dim.crop.left = 0;
  cfg_.job_.encode_job.second_thumb_dim.crop.width = plane_info.width;
  cfg_.job_.encode_job.second_thumb_dim.crop.height = plane_info.height;
  cfg_.params_.second_thumb_dim = cfg_.job_.encode_job.second_thumb_dim;
  QMMF_INFO("%s: Encode second thumbnail is enabled", __func__);

  return 0;
}

int32_t JpegEncoder::Init(uint32_t width, uint32_t height) {
  cfg_.pic_size_.w = width;
  cfg_.pic_size_.h = height;

  cfg_.handle_ = cfg_.jpeg_open_proc(&cfg_.ops_, NULL, cfg_.pic_size_, NULL);
  if (cfg_.handle_ == 0) {
    ALOGE("%s: could not open a jpeg handle", __func__);
    return -1;
  }

  return 0;
}

int32_t JpegEncoder::DeInit() {
  if (!cfg_.handle_) {
    ALOGE("%s: jpeg handle is 0", __func__);
    return -1;
  }

  cfg_.ops_.close(cfg_.handle_);
  cfg_.handle_ = 0;

  return 0;
}

int32_t JpegEncoder::Encode(encode_params &params, size_t &jpeg_size) {
  std::lock_guard<std::mutex> al(cfg_.encode_lock_);

  int32_t ret = 0;
  jpeg_size = 0;

  if (!params.img_data[0] || !params.out_data[0]) {
    ALOGE("%s: can't pass NULL plane pointer", __func__);
    return -1;
  }

  ret = ConfigureMainImage(params);
  if (ret) {
    ALOGE("%s: failed to configure main image", __func__);
    return ret;
  }

  ret = ConfigureThumbnails(params);
  if (ret) {
    ALOGE("%s: failed to configure thumbnails", __func__);
    return ret;
  }

  // buffer data
  cfg_.params_.src_main_buf[0].buf_vaddr = params.img_data[0];
  cfg_.params_.src_thumb_buf[0].buf_vaddr = params.img_data[0];
  cfg_.params_.dest_buf[0].buf_vaddr = params.out_data[0];
  cfg_.job_id_ = 0;

  cfg_.ops_.create_session(cfg_.handle_, &cfg_.params_, &cfg_.job_.encode_job.session_id);
  if (cfg_.job_.encode_job.session_id == 0) {
    ALOGE("%s: could not create jpeg session", __func__);
    return -1;
  }

  // start JPEG encode
  ret = cfg_.ops_.start_job(&cfg_.job_, &cfg_.job_id_);
  if (ret) {
    ALOGE("%s: could not start encode job", __func__);
    cfg_.ops_.destroy_session(cfg_.job_.encode_job.session_id);
    return -1;
  }

  // wait JPEG encode to finish
  {
    std::unique_lock<std::mutex> ul(cfg_.enc_done_lock_);
    cfg_.enc_done_cond_.Wait(ul);
  }

  // add a valid JPEG header
  camera3_jpeg_blob_t jpegHeader;
  jpegHeader.jpeg_blob_id = CAMERA3_JPEG_BLOB_ID;
  jpegHeader.jpeg_size = static_cast<uint32_t>(job_result_size_);
  uint8_t *jpegEof = &cfg_.params_.dest_buf[0].buf_vaddr[job_result_size_];
  memcpy(jpegEof, &jpegHeader, sizeof(jpegHeader));
  jpeg_size = job_result_size_ + sizeof(jpegHeader) + 1;

  cfg_.ops_.destroy_session(cfg_.job_.encode_job.session_id);

  return 0;
}

void JpegEncoder::EncodeCb(void *p_output, void *userData) {
  JpegEncoder *enc = static_cast<JpegEncoder *>(userData);
  mm_jpeg_output_t *output = static_cast<mm_jpeg_output_t *>(p_output);

  enc->job_result_ptr_ = output->buf_vaddr;
  enc->job_result_size_ = output->buf_filled_len;

  enc->cfg_.enc_done_cond_.Signal();
}

} //namespace reprocjpegencoder ends here

} //namespace qmmf ends here
