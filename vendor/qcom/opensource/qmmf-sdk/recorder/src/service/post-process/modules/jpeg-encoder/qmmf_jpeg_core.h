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

#pragma once

#include <vector>

#include <hardware/camera3.h>
#include <mm_jpeg_interface.h>

#include "common/utils/qmmf_condition.h"


#include <qmmf-sdk/qmmf_codec.h>

namespace qmmf {

namespace reprocjpegencoder {


typedef uint32_t (*jpeg_open_proc_t)(mm_jpeg_ops_t *,
                                     mm_jpeg_mpo_ops_t *,
                                     mm_dimension,
                                     cam_related_system_calibration_data_t *);

class JpegEncoder {
 public:

  struct jpeg_thumbnail {
    // thumbnail dimension
    uint32_t width;
    uint32_t height;
    // jpeg thumbnail quality: range 0~100
    uint32_t thumb_quality;

    jpeg_thumbnail(uint32_t width, uint32_t height, uint32_t thumb_quality) :
        width(width), height(height), thumb_quality(thumb_quality) {}
  };

  struct encode_params {
    uint8_t *img_data[3];
    uint8_t *out_data[3];
    CameraBufferMetaData source_info;
    uint32_t image_quality;
    std::vector<jpeg_thumbnail> thumbnail_data;
  };

 private:
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
    QCondition enc_done_cond_;
  } JpegEncoderParams;

  int32_t ConfigureMainImage(const encode_params &params);

  int32_t ConfigureThumbnails(const encode_params &params);

  JpegEncoderParams cfg_;
  void *job_result_ptr_;
  size_t job_result_size_;

  static uint8_t DEFAULT_QTABLE_0[];
  static uint8_t DEFAULT_QTABLE_1[];
  static JpegEncoder *encoder_instance_;

  void *libjpeg_interface_;

  static const uint32_t kDefaultMainThumbWidth;
  static const uint32_t kDefaultMainThumbHeight;

  static const uint32_t kDefaultSecondThumbWidth;
  static const uint32_t kDefaultSecondThumbHeight;

 public:

  JpegEncoder();

  ~JpegEncoder();

  int32_t Init(uint32_t width, uint32_t height);

  int32_t DeInit();

  int32_t Encode(encode_params &params, size_t &jpeg_size /* output */);

  void EncodeCb(void *p_output, void *userData);

  static JpegEncoder *getInstance();

  static void releaseInstance();
};

}; //namespace reprocjpegencoder ends here

}; //namespace qmmf ends here
