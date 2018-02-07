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
#include <list>
#include <condition_variable>

#include "qmmf-sdk/qmmf_recorder_params.h"

#include "../../interface/qmmf_postproc_module.h"

#include "common/cameraadaptor/qmmf_camera3_device_client.h"

namespace qmmf {

using namespace cameraadaptor;

namespace recorder {

enum class PostProcHalMode {
  kJpegEncode,
  kRawReprocess,
};

class CameraContext;

class CameraHalReproc : public IPostProcModule {

 public:

   CameraHalReproc(IPostProc* context);

   ~CameraHalReproc();

   status_t Initialize(const PostProcIOParam &in_param,
                       const PostProcIOParam &out_param) override;

   status_t Delete() override;

   void SetCallbacks(IPostProcEventListener *cb) override {listener_ = cb;};

   status_t Configure(const std::string config_json_data) override;

   status_t Process(const std::vector<StreamBuffer> &in_buffers,
                    const std::vector<StreamBuffer> &out_buffers) override;

   void AddResult(const void* result) override;

   status_t ReturnBuff(StreamBuffer &buffer) override;

   status_t Start(const int32_t stream_id) override;

   status_t Stop() override;

   PostProcIOParam GetInput(const PostProcIOParam &out) override;

   status_t ValidateOutput(const PostProcIOParam &output) override;

   status_t GetCapabilities(PostProcCaps &caps) override;

 private:

   static const int64_t kMetaTimeout = 1000000000; // 1 second

   struct ReprocessBundle {
     StreamBuffer   buffer;
     CameraMetadata metadata;
     int64_t        timestamp;
   };

   void ReturnAllInputBuffers();

   void ReturnInputBuffer(StreamBuffer &buffer);

   void GetInputBuffer(StreamBuffer &buffer);

   void ReprocessCallback(StreamBuffer buffer);

   status_t ValidateInput(const PostProcIOParam& input,
                          const PostProcIOParam& output);

   status_t StartProcessing();

   void StreamCb(StreamBuffer &in_buff);

   void AddBuff(const StreamBuffer buf);

   void AddMeta(const CameraMetadata &metadata);

   IPostProc*                   context_;
   IPostProcEventListener       *listener_;

   std::mutex                   module_lock_;
   bool                         ready_to_start_;
   int32_t                      input_stream_id_;

   Camera3Request               reprocess_request_;

   PostProcIOParam              input_param_;
   PostProcIOParam              output_param_;

   std::list<StreamBuffer>      input_buffer_;
   std::list<StreamBuffer>      input_buffer_done_;

   std::list<ReprocessBundle>   reproc_partial_list_;
   std::list<ReprocessBundle>   reproc_ready_list_;
   std::mutex                   reproc_lock_;
};

}; //namespace recorder

}; //namespace qmmf
