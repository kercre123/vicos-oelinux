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

#pragma once

#include "qmmf-sdk/qmmf_codec.h"

namespace qmmf {

namespace reprocjpegencoder {

class JpegEncoder {

 private:

  void FillImgData(const CameraBufferMetaData& source_info);

  void *cfg_;
  void *job_result_ptr_;
  size_t job_result_size_;

  static uint8_t DEFAULT_QTABLE_0[];
  static uint8_t DEFAULT_QTABLE_1[];
  static JpegEncoder *encoder_instance_;

 public:

  struct snapshot_info {
    uint8_t *img_data[3];
    uint8_t *out_data[3];
    CameraBufferMetaData source_info;
  };

  JpegEncoder();

  ~JpegEncoder();

  void *Encode(size_t *jpeg_size);

  void EncodeCb(void *p_output, void *userData);

  static JpegEncoder *getInstance();

  static void releaseInstance();

  snapshot_info in_buffer_;

  void *libjpeg_interface_;

};

}; //namespace reprocjpegencoder ends here

}; //namespace qmmf ends here