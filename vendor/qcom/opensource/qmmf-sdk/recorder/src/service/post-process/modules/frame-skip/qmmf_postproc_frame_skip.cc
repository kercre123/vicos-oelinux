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

#define LOG_TAG "PostProcFrameSkip"

#include "recorder/src/service/qmmf_recorder_utils.h"

#include "qmmf_postproc_frame_skip.h"

#include <json/json.h>

namespace qmmf {

namespace recorder {

PostProcFrameSkip::PostProcFrameSkip()
    : state_(State::CREATED),
      frame_skip_(1),
      frame_counter_(0) {
  QMMF_INFO("%s: Enter", __func__);
}

PostProcFrameSkip::~PostProcFrameSkip() {
  QMMF_INFO("%s: Enter ", __func__);
}

status_t PostProcFrameSkip::Initialize(const PostProcIOParam &in_param,
                                       const PostProcIOParam &out_param) {
  QMMF_INFO("%s: Enter", __func__);

  std::lock_guard<std::mutex> lock(state_lock_);
  frame_skip_ = 1;
  state_ = State::INITIALIZED;

  return NO_ERROR;
}

PostProcIOParam PostProcFrameSkip::GetInput(const PostProcIOParam &out) {
  QMMF_INFO("%s: Enter", __func__);
  return out;
}

status_t PostProcFrameSkip::ValidateOutput(const PostProcIOParam &output) {
  QMMF_INFO("%s: Enter", __func__);
  return NO_ERROR;
}

status_t PostProcFrameSkip::GetCapabilities(PostProcCaps &caps) {
  caps.output_buff_        = 0;
  caps.min_width_          = 160;
  caps.min_height_         = 120;
  caps.max_width_          = 5104;
  caps.max_height_         = 4092;
  caps.crop_support_       = false;
  caps.scale_support_      = false;
  caps.inplace_processing_ = true;
  caps.usage_              = 0;

  caps.formats_.insert(BufferFormat::kNV12);
  caps.formats_.insert(BufferFormat::kNV12UBWC);
  caps.formats_.insert(BufferFormat::kNV21);
  caps.formats_.insert(BufferFormat::kBLOB);
  caps.formats_.insert(BufferFormat::kRAW10);
  caps.formats_.insert(BufferFormat::kRAW12);
  caps.formats_.insert(BufferFormat::kRAW16);

  return NO_ERROR;
}

status_t PostProcFrameSkip::Start(const int32_t stream_id) {
  QMMF_INFO("%s: Enter", __func__);

  std::lock_guard<std::mutex> lock(state_lock_);
  state_ = State::ACTIVE;
  frame_counter_ = 0;

  return NO_ERROR;
}

status_t PostProcFrameSkip::Stop() {
  QMMF_INFO("%s: Enter", __func__);

  std::lock_guard<std::mutex> lock(state_lock_);
  state_ = State::INITIALIZED;
  frame_counter_ = 0;

  return NO_ERROR;
}

status_t PostProcFrameSkip::Abort(std::shared_ptr<void> &abort) {
  QMMF_INFO("%s: Enter", __func__);

  std::lock_guard<std::mutex> lock(state_lock_);
  state_ = State::ABORTED;

  return NO_ERROR;
}


status_t PostProcFrameSkip::Delete() {
  QMMF_INFO("%s: Enter ", __func__);

  std::lock_guard<std::mutex> lock(state_lock_);
  state_ = State::CREATED;

  return NO_ERROR;
}

status_t PostProcFrameSkip::Configure(const std::string config_json_data) {
  QMMF_INFO("%s: Enter ", __func__);
  Json::Reader r;
  Json::Value root;

  auto ret = r.parse(config_json_data, root);
  if (ret == 0) {
    QMMF_INFO("%s: no json data", __func__);
    return NO_ERROR;
  }

  if (!root.isMember("frameskip") || root["frameskip"].empty()) {
    QMMF_INFO("%s:no frame skip configuration", __func__);
    return NO_ERROR;
  }

  frame_skip_ = root["frameskip"].asInt();

  QMMF_INFO("%s: Update frame skip to %d", __func__, frame_skip_);

  return NO_ERROR;
}

status_t PostProcFrameSkip::Process(
    const std::vector<StreamBuffer> &in_buffers,
    const std::vector<StreamBuffer> &out_buffers) {

  QMMF_INFO("%s: E", __func__);
  std::lock_guard<std::mutex> lock(state_lock_);

  for (auto buf : in_buffers) {
    if (state_ != State::ACTIVE || SkipFrame()) {
      QMMF_INFO("%s: skip frame. state %d", __func__, state_);
      Listener_->OnFrameProcessed(buf);
    } else {
      QMMF_INFO("%s: process frame", __func__);
      Listener_->OnFrameReady(buf);
    }
  }

  return NO_ERROR;
}

bool PostProcFrameSkip::SkipFrame(void) {
  return (frame_counter_++ % frame_skip_) != 0;
}

}; // namespace recoder

}; // namespace qmmf
