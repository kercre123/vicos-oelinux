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
#include <jpeglib.h>

namespace qmmf {

namespace jpegencoder {

JpegEncoder::JpegEncoder(struct JpegEncoderConfiguration *params) :
    tmpRowBuf_(nullptr) {
  cfg_ = *params;
  cfg_.width[0] &= ~1;
  tmpRowBuf_ = new uint8_t[cfg_.width[0] * 3];
}

JpegEncoder::~JpegEncoder() {
    cleanUp();
}

void JpegEncoder::cleanUp() {
  std::lock_guard<std::mutex> al(encode_lock_);

  if (tmpRowBuf_) delete [] tmpRowBuf_;
  tmpRowBuf_ = nullptr;
}

void *JpegEncoder::Encode(struct JpegFrameInfo *pFrame, size_t *jpeg_size, float destScale) {
  std::lock_guard<std::mutex> al(encode_lock_);
  struct jpeg_error_mgr jerr;
  struct jpeg_compress_struct cinfo;
  uint8_t *outBuf = nullptr;
  unsigned long outSize = 0;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);
  jpeg_mem_dest(&cinfo, &outBuf, &outSize);

  uint32_t step = (uint32_t)destScale;
  if (!step) {
    step = 1;
  } else if (step > JPEG_MAX_DESTINATION_SCALE) {
    step = JPEG_MAX_DESTINATION_SCALE;
  }

  cinfo.image_width = cfg_.width[0] / step;
  cinfo.image_height = cfg_.height[0] / step;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_YCbCr;

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, cfg_.quality, (boolean)1);

  JSAMPROW row[1];
  row[0] = tmpRowBuf_;

  uint32_t i, j;
  uint32_t offsetY, offsetCrCb;
  uint32_t crIdx, cbIdx;

  if (pFrame->format == JpegEncoderFormat::NV12) {
      crIdx = 0; cbIdx = 1;
  } else {
      crIdx = 1; cbIdx = 0;
  }

  jpeg_start_compress(&cinfo, TRUE);
  while (cinfo.next_scanline < cinfo.image_height) {
      offsetY = step * cinfo.next_scanline * cfg_.stride[0];
      offsetCrCb = (step * cinfo.next_scanline / 2) * cfg_.stride[0];
      for (i = 0, j = 0; i < cfg_.width[0]; i += 2 * step, j += 6) {
          tmpRowBuf_[j + 0] = pFrame->plane_addr[0][offsetY + i];
          tmpRowBuf_[j + 1] = pFrame->plane_addr[1][offsetCrCb + i + crIdx];
          tmpRowBuf_[j + 2] = pFrame->plane_addr[1][offsetCrCb + i + cbIdx];

          tmpRowBuf_[j + 3] = pFrame->plane_addr[0][offsetY + i + 1];
          tmpRowBuf_[j + 4] = pFrame->plane_addr[1][offsetCrCb + i + crIdx];
          tmpRowBuf_[j + 5] = pFrame->plane_addr[1][offsetCrCb + i + cbIdx];
      }
      jpeg_write_scanlines(&cinfo, row, 1);
  }
  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);

  if (jpeg_size) {
    *jpeg_size = (size_t)outSize;
  }

  return outBuf;
}

} //namespace jpegencoder ends here
} //namespace qmmf ends here
