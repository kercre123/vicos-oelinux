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

#include <mutex>
#include <vector>
#include <utils/Log.h>

#include <qmmf-sdk/qmmf_codec.h>

namespace qmmf {

namespace jpegencoder {

struct jpeg_thumbnail {
  // thumbnail dimension
  uint32_t width;
  uint32_t height;
  // jpeg thumbnail quality: range 0~100
  uint32_t thumb_quality;

  jpeg_thumbnail(uint32_t width, uint32_t height, uint32_t thumb_quality) :
      width(width), height(height), thumb_quality(thumb_quality) {}
};

struct snapshot_info
{
  uint8_t *img_data[3];
  uint8_t *out_data[3];
  CameraBufferMetaData source_info;
  uint32_t exif_size;
  void*    exif_data;
  std::vector<jpeg_thumbnail> thumbnails;
};

class JpegEncoder {

private:
  void FillImgData(const CameraBufferMetaData& source_info);

  void UpdateThumbnailData(const snapshot_info& snapshot);

  void *cfg_;
  void *job_result_ptr_;
  size_t job_result_size_;

  static uint8_t DEFAULT_QTABLE_0[];
  static uint8_t DEFAULT_QTABLE_1[];
  static JpegEncoder *encoder_instance_;

public:

  JpegEncoder();

  ~JpegEncoder();

  void *Encode(const snapshot_info& in_buffer, size_t &jpeg_size,
               const uint32_t quality);

  static JpegEncoder *getInstance();

  static void releaseInstance();

  static void EncodeCb(void *p_output, void *userData);

  void *libjpeg_interface_;
};

}; //namespace jpegencoder ends here

}; //namespace qmmf ends here
