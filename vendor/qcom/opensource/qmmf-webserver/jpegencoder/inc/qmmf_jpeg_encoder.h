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
#ifndef QMMF_JPEG_ENCODER_H_
#define QMMF_JPEG_ENCODER_H_

#include <utils/Log.h>
#include <utils/Errors.h>

#define JPEG_MAX_BUFFER_PLANES 3
#define JPEG_THUMBNAIL_WIDTH 320
#define JPEG_THUMBNAIL_HEIGHT 240
#define JPEG_THUMBNAIL_DEFAULT_QUALITY 75
#define JPEG_DEFAULT_QUALITY 95

#define JPEG_QUALITY_PROP "qmmf.jpegenc.quality"
#define JPEG_THUMBNAIL_QUALITY_PROP "qmmf.jpegenc.thumbnail.quality"

#define JPEG_MAX_DESTINATION_SCALE 16.0f

namespace qmmf {

namespace jpegencoder {

enum JpegEncoderFormat {
    NV12,
    NV21,
    UNKNOWN,
};

struct JpegEncoderConfiguration {
  enum JpegEncoderFormat format;
  uint32_t  num_planes;
  uint32_t stride[JPEG_MAX_BUFFER_PLANES];
  uint32_t scanline[JPEG_MAX_BUFFER_PLANES];
  uint32_t width[JPEG_MAX_BUFFER_PLANES];
  uint32_t height[JPEG_MAX_BUFFER_PLANES];
};

struct JpegFrameInfo {
    uint8_t *plane_addr[JPEG_MAX_BUFFER_PLANES];
};

class JpegEncoder {
private:
  void *cfg_;
  void *job_result_ptr_;
  size_t job_result_size_;

  static uint8_t DEFAULT_QTABLE_0[];
  static uint8_t DEFAULT_QTABLE_1[];

public:
  JpegEncoder(struct JpegEncoderConfiguration *params);
  ~JpegEncoder();

  void *Encode(struct JpegFrameInfo *pFrame, size_t *jpeg_size, float destScale);

  static void OnEncodeDone(void *p_output, void *userData);
};

} //namespace jpegencoder ends here
} //namespace qmmf ends here

#endif /* QMMF_JPEG_ENCODER_H_ */
