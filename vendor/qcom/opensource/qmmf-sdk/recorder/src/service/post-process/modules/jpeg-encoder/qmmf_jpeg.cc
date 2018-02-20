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

#define LOG_TAG "RecorderJpeg"

#include <sys/mman.h>
#include <sstream>
#include <json/json.h>

#include "qmmf_jpeg.h"

namespace qmmf {

namespace recorder {

const uint32_t PostProcJpeg::kMinWidth  = 160;
const uint32_t PostProcJpeg::kMinHeight = 120;
const uint32_t PostProcJpeg::kMaxWidth  = 5104;
const uint32_t PostProcJpeg::kMaxHeight = 4092;

const int32_t PostProcJpeg::kSupportedInputFormat = HAL_PIXEL_FORMAT_YCbCr_420_888;
const int32_t PostProcJpeg::kSupportedOutputFormat = HAL_PIXEL_FORMAT_BLOB;

PostProcJpeg::PostProcJpeg()
    : jpeg_encoder_(nullptr),
      image_quality_(95),
      state_(State::CREATED),
      abort_(nullptr) {
  QMMF_VERBOSE("%s: Enter", __func__);
  jpeg_encoder_ = reprocjpegencoder::JpegEncoder::getInstance();
  QMMF_VERBOSE("%s: Exit (0x%p)", __func__, this);
}

PostProcJpeg::~PostProcJpeg() {
  QMMF_VERBOSE("%s: Enter ", __func__);
  reprocjpegencoder::JpegEncoder::releaseInstance();
  jpeg_encoder_ = nullptr;
  QMMF_VERBOSE("%s: Exit (0x%p)", __func__, this);
}

status_t PostProcJpeg::Initialize(const PostProcIOParam &in_param,
                                  const PostProcIOParam &out_param) {
  QMMF_VERBOSE("%s: Enter", __func__);

  std::lock_guard<std::mutex> lock(state_lock_);
  state_ = State::INITIALIZED;

  thumbnail_data_.clear();

  return NO_ERROR;
}

PostProcIOParam PostProcJpeg::GetInput(const PostProcIOParam &out) {
  PostProcIOParam input_param = out;
  input_param.format = Common::FromHalToQmmfFormat(kSupportedInputFormat);

  // set number of needed buffers for rotation if client does not limit it
  if (out.buffer_max > 0 && out.buffer_max < kBufCount) {
    input_param.buffer_count = out.buffer_max;
  } else {
    input_param.buffer_count = kBufCount;
  }
  return input_param;
}

status_t PostProcJpeg::ValidateOutput(const PostProcIOParam &output) {
  if (output.format != Common::FromHalToQmmfFormat(kSupportedOutputFormat)) {
    QMMF_ERROR("%s: Output format(%d) not supported", __func__, output.format);
    return BAD_TYPE;
  }

  if ((output.width < kMinWidth || output.width > kMaxWidth) ||
      (output.height < kMinHeight || output.height > kMaxHeight)) {
    QMMF_ERROR("%s: Output dimensions(%dx%d) not supported", __func__,
        output.width, output.height);
    return BAD_VALUE;
  }

  return NO_ERROR;
}

status_t PostProcJpeg::GetCapabilities(PostProcCaps &caps) {
  caps.output_buff_        = 1;
  caps.min_width_          = kMinWidth;
  caps.min_height_         = kMinHeight;
  caps.max_width_          = kMaxWidth;
  caps.max_height_         = kMaxHeight;
  caps.crop_support_       = false;
  caps.scale_support_      = false;
  caps.inplace_processing_ = false;
  caps.usage_              = 0;

  caps.formats_.insert(BufferFormat::kBLOB);

  return NO_ERROR;
}

status_t PostProcJpeg::Start(const int32_t stream_id) {
  QMMF_VERBOSE("%s: Enter %p", __func__, this);
  std::lock_guard<std::mutex> lock(state_lock_);
  state_ = State::ACTIVE;
  return NO_ERROR;
}

status_t PostProcJpeg::Stop() {
  QMMF_INFO("%s: Enter %p", __func__, this);
  std::lock_guard<std::mutex> lock(state_lock_);
  state_ = State::INITIALIZED;
  return NO_ERROR;
}

status_t PostProcJpeg::Abort(std::shared_ptr<void> &abort) {
  QMMF_INFO("%s: Enter %p", __func__, this);
  std::lock_guard<std::mutex> lock(state_lock_);
  if (state_ == State::RUNING) {
    QMMF_VERBOSE("%s: Acquire abort done handler", __func__);
    abort_ = abort;
  }
  state_ = State::ABORTED;
  return NO_ERROR;
}

status_t PostProcJpeg::Delete() {
  QMMF_VERBOSE("%s: Enter %p", __func__, this);
  std::lock_guard<std::mutex> lock(state_lock_);
  state_ = State::CREATED;
  return NO_ERROR;
}

status_t PostProcJpeg::Configure(const std::string config_json_data) {
  QMMF_VERBOSE("%s: Enter %p", __func__, this);
  Json::Reader r;
  Json::Value root;

  auto ret = r.parse(config_json_data, root);
  if (ret == 0) {
    QMMF_INFO("%s: no json data", __func__);
    return NO_ERROR;
  }

  if (!root.isMember("jpeg quality") || root["jpeg quality"].empty()) {
    QMMF_INFO("%s:no jpeg quality configuration", __func__);
  } else {
    image_quality_ = root["jpeg quality"].asUInt();
  }

  if (!root.isMember("thumbnail") || root["thumbnail"].empty()) {
    QMMF_INFO("%s:no thumbnail configuration", __func__);
  } else {
    thumbnail_data_.clear();
    for (Json::Value::ArrayIndex i = 0; i < root["thumbnail"].size(); i++) {
      QMMF_INFO("%s:add thumbnail[%d] dim %dx%d quality %d", __func__, i,
          root["thumbnail"][i]["width"].asUInt(),
          root["thumbnail"][i]["height"].asUInt(),
          root["thumbnail"][i]["quality"].asUInt());

      thumbnail_data_.emplace_back(
          root["thumbnail"][i]["width"].asUInt(),
          root["thumbnail"][i]["height"].asUInt(),
          root["thumbnail"][i]["quality"].asUInt());
    }
  }

  QMMF_VERBOSE("%s: Exit %p", __func__, this);

  return NO_ERROR;
}

status_t PostProcJpeg::Process(const std::vector<StreamBuffer> &in_buffers,
                               const std::vector<StreamBuffer> &out_buffers) {
  StreamBuffer in_buffer = in_buffers.front();
  StreamBuffer out_buffer = out_buffers.front();

  QMMF_VERBOSE("%s: %d: Enter in FD: %d out FD: %d ",
      __func__, __LINE__, in_buffer.fd, out_buffer.fd);

  {
    std::lock_guard<std::mutex> lock(state_lock_);
    if (state_ != State::ACTIVE) {
      listener_->OnFrameReady(out_buffer);
      listener_->OnFrameProcessed(in_buffer);
      return NO_ERROR;
    }
    state_ = State::RUNING;
  }

  void *buf_vaaddr = nullptr;
  if (in_buffer.data == nullptr) {
    buf_vaaddr = mmap(nullptr, in_buffer.size, PROT_READ  | PROT_WRITE,
        MAP_SHARED, in_buffer.fd, 0);
  } else {
    buf_vaaddr = in_buffer.data;
  }

  if (buf_vaaddr == MAP_FAILED) {
      QMMF_ERROR("%s  ION mmap failed: %s (%d)", __func__,
          strerror(errno), errno);
  }

  void *out_vaaddr = nullptr;
  if (out_buffer.data == nullptr) {
    out_vaaddr = mmap(nullptr, out_buffer.size, PROT_READ  | PROT_WRITE,
        MAP_SHARED, out_buffer.fd, 0);
  } else {
    out_vaaddr = out_buffer.data;
  }

  if (out_vaaddr == MAP_FAILED) {
      QMMF_ERROR("%s  ION mmap failed: %s (%d)", __func__,
          strerror(errno), errno);
  }

  if (buf_vaaddr != MAP_FAILED && out_vaaddr != MAP_FAILED) {
    size_t jpeg_size = 0;
    jpeg_encoder_->in_buffer_.img_data[0] = (uint8_t*)buf_vaaddr;
    jpeg_encoder_->in_buffer_.out_data[0] = (uint8_t*)out_vaaddr;
    jpeg_encoder_->in_buffer_.source_info = in_buffer.info;
    jpeg_encoder_->in_buffer_.image_quality = image_quality_;
    jpeg_encoder_->in_buffer_.thumbnail_data = thumbnail_data_;
    auto buf_vaddr = jpeg_encoder_->Encode(&jpeg_size);
    if (!buf_vaddr) {
      QMMF_VERBOSE("%s: Jpeg out buffer is NULL", __func__);
    }

    if (in_buffer.data == nullptr) {
      munmap(buf_vaaddr, in_buffer.size);
    }
    if (out_buffer.data == nullptr) {
      munmap(out_vaaddr, out_buffer.size);
    }

    out_buffer.info.format = BufferFormat::kBLOB;
    out_buffer.info.plane_info[0].width = jpeg_size;
    out_buffer.data = nullptr;
    out_buffer.filled_length = jpeg_size;
    out_buffer.timestamp = in_buffer.timestamp;

  } else {
    QMMF_VERBOSE("%s: SKIPP JPEG", __func__);
  }

  listener_->OnFrameReady(out_buffer);
  listener_->OnFrameProcessed(in_buffer);

  std::lock_guard<std::mutex> lock(state_lock_);
  if (state_ == State::ABORTED) {
    QMMF_VERBOSE("%s: Release abort done handler", __func__);
    abort_ = nullptr;
  }
  state_ = State::ACTIVE;

  QMMF_VERBOSE("%s: Exit", __func__);

  return NO_ERROR;
}

}; // namespace recoder

}; // namespace qmmf
