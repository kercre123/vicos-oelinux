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

#define TAG "CameraSimple"

#include <sys/mman.h>

#include "recorder/src/service/qmmf_recorder_utils.h"

#include "qmmf_postproc_test.h"

namespace qmmf {

namespace recorder {

PostProcTest::PostProcTest() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
}

PostProcTest::~PostProcTest() {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
}

status_t PostProcTest::Initialize(const PostProcIOParam &in_param,
                                  const PostProcIOParam &out_param) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  return NO_ERROR;
}

PostProcIOParam PostProcTest::GetInput(const PostProcIOParam &out) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  return out;
}

status_t PostProcTest::ValidateOutput(const PostProcIOParam &output) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  return NO_ERROR;
}

status_t PostProcTest::GetCapabilities(PostProcCaps &caps) {
  caps.min_width_          = 160;
  caps.min_height_         = 120;
  caps.max_width_          = 5104;
  caps.max_height_         = 4092;
  caps.usage_              = 0;
  caps.crop_support_       = false;
  caps.scale_support_      = false;
  caps.inplace_processing_ = true;

  caps.formats_.insert(BufferFormat::kNV12);
  caps.formats_.insert(BufferFormat::kNV21);

  return NO_ERROR;
}

status_t PostProcTest::Start(const int32_t stream_id) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  return NO_ERROR;
}

status_t PostProcTest::Stop() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  return NO_ERROR;
}

status_t PostProcTest::Delete() {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  return NO_ERROR;
}

status_t PostProcTest::Configure(const std::string config_json_data) {
  return NO_ERROR;
}

status_t PostProcTest::Process(
    const std::vector<StreamBuffer> &in_buffers,
    const std::vector<StreamBuffer> &out_buffers) {

  QMMF_INFO("%s:%s: Simple done", TAG, __func__);

  // no process call directly client
  for (auto buf : in_buffers) {
    Listener_->OnFrameReady(buf);
  }

  return NO_ERROR;
}

}; // namespace recoder

}; // namespace qmmf
