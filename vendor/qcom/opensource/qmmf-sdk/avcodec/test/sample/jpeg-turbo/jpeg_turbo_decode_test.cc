/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *     Neither the name of The Linux Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.

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

#define LOG_TAG "JpegTurboDecodeTest"

#include <stdio.h>
#include <turbojpeg.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utils/Log.h>
#include "avcodec/test/sample/jpeg-turbo/jpeg_turbo_decode_test.h"
using namespace std;

JpegDecode::JpegDecode()
    : dhandle_(nullptr), TD_BU_(0), srcBuf_(nullptr), dstBuf_(nullptr) {
  ALOGI("Object Created Successfully");
}

JpegDecode::~JpegDecode() {
  tjDestroy(this->dhandle_);
  tjFree(this->srcBuf_);
  tjFree(this->dstBuf_);
}

int JpegDecode::InitialiseDecoder() {
  ALOGI("%s: Enter", __func__);
  if ((this->dhandle_ = tjInitDecompress()) == nullptr) {
    return -1;
  }
  ALOGI("%s: Exit", __func__);
  return 0;
}

int JpegDecode::ReadJpeg(char *filename) {
  ALOGI("%s: Enter", __func__);
  FILE *fp;
  fp = fopen(filename, "r");
  if (fp == nullptr) {
    return -1;
  }
  fseek(fp, 0, SEEK_END);
  this->jpegSize_ = ftell(fp);
  rewind(fp);
  this->srcBuf_ = new unsigned char[this->jpegSize_];
  if (this->srcBuf_ == nullptr) {
    ALOGE("%s: Memory not allocated to source buffer", __func__);
    fclose(fp);
    return -1;
  }
  fread(this->srcBuf_, 1, this->jpegSize_, fp);
  fclose(fp);
  ALOGI("%s: Exit", __func__);
  return 0;
}

int JpegDecode::Configure() {
  ALOGI("%s: Enter", __func__);
  if (tjDecompressHeader2(this->dhandle_, this->srcBuf_, this->jpegSize_,
                          &(this->width_), &(this->height_),
                          &(this->subsamp_)) != 0) {
    return -1;
  }
  ALOGI("%s: width:%d, height: %d", __func__, this->width_, this->height_);
  ALOGI("%s: Exit", __func__);
  return 0;
}

int JpegDecode::AllocateDestBuffer() {
  ALOGI("%s: Enter", __func__);
  this->yuvSize_ = static_cast<int>(tjBufSizeYUV(this->width_, this->height_, this->subsamp_));
  this->dstBuf_ = new unsigned char[this->yuvSize_];
  if (this->dstBuf_ == nullptr) {
    return -1;
  }
  ALOGI("%s: Exit", __func__);
  return 0;
}

int JpegDecode::Decode() {
  ALOGI("%s: Enter", __func__);
  if (tjDecompressToYUV(this->dhandle_, this->srcBuf_, this->jpegSize_,
                        this->dstBuf_, this->TD_BU_) != 0) {
    ALOGE("%s: Decompress from Jpeg to YUV failed", __func__);
    free(this->srcBuf_);
    free(this->dstBuf_);
    return -1;
  }
  ALOGI("%s: Exit", __func__);
  return 0;
}

int JpegDecode::DumpYUV(char *name) {
  ALOGI("%s: Enter", __func__);
  string file_path("/data/misc/qmmf/");
  file_path += name;
  file_path += ".yuv";
  size_t written_len;
  FILE *file = fopen(file_path.c_str(), "w+");
  if (file == nullptr) {
    return -1;
  }
  written_len =
      fwrite(this->dstBuf_, sizeof(uint8_t),
             tjBufSizeYUV(this->width_, this->height_, this->subsamp_), file);
  if (file != nullptr) {
    fclose(file);
  }
  if (written_len == 0) {
    ALOGE("%s: Write to YUV file failed", __func__);
    return -1;
  }
  ALOGI("%s: Exit", __func__);
  return 0;
}

int main(int argc, char *argv[]) {

  if (argc == 1) {
    ALOGE("Please provide a jpeg image as input");
    return -1;
  } else if (argc > 2) {
    ALOGE("Number of arguments mismatch: Please provide one jpeg image as input");
    return -1;
  }

  char *extn = strrchr(argv[1], '.');

  if (!((strcmp(extn, ".jpg") == 0) || (strcmp(extn, ".JPG") == 0) ||
        (strcmp(extn, ".jpeg") == 0) || (strcmp(extn, ".JPEG") == 0))) {
    ALOGE("File is not Jpeg file");
    return -1;
  }

  JpegDecode jpeg;
  char *name;
  auto ret = jpeg.InitialiseDecoder();
  if (0 != ret) {
    ALOGE("Configure failed");
    return -1;
  }

  ret = jpeg.ReadJpeg(argv[1]);
  if (0 != ret) {
    ALOGE("ReadJpeg failed");
    return -1;
  }

  ret = jpeg.Configure();
  if (0 != ret) {
    ALOGE("Configure failed");
    return -1;
  }

  ret = jpeg.AllocateDestBuffer();
  if (0 != ret) {
    ALOGE("Allocate Destination Buffer failed");
    return -1;
  }

  ALOGI("Begin Decoding");

  ret = jpeg.Decode();
  if (0 != ret) {
    ALOGE("Decode failed");
    return -1;
  }

  ALOGI("Decoding Completed");
  name = strtok(argv[1], ".");
  ret = jpeg.DumpYUV(name);
  if (0 != ret) {
    ALOGE("DumpYUV failed");
    return -1;
  }
  return 0;
}
