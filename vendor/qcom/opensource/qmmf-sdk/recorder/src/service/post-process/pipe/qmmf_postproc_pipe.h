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

#include <string>
#include <vector>

#include "../node/qmmf_postproc_node.h"
#include "../factory/qmmf_postproc_factory.h"

namespace qmmf {

namespace recorder {

class IBufferConsumer;
class IBufferProducer;

struct PipeIOParam {
  uint32_t width;
  uint32_t height;
  uint32_t stride;
  uint32_t scanline;
  uint32_t frame_rate;
  int32_t format;
  uint32_t image_quality;
  int32_t gralloc_flags;
  uint32_t buffer_count;
};

enum class PostProcPipeState {
  CREATED,
  INITIALIZE,
  INITIALIZED,
  CONFIGURED,
  READYTOSTART,
  READYTOSTOP,
};

class PostProcPipe : public virtual  RefBase {

 public:

   PostProcPipe(IPostProc* context);

   ~PostProcPipe();

   status_t CreatePipe(const PipeIOParam &pipe_out_param,
       const std::vector<uint32_t> &plugins, PipeIOParam &pipe_in_param);

   status_t AddConsumer(sp<IBufferConsumer>& consumer);

   status_t RemoveConsumer(sp<IBufferConsumer>& consumer);

   void AddResult(const void* result);

   status_t Start(const int32_t stream_id);

   status_t Stop();

   sp<IBufferConsumer>& GetConsumerIntf();

   void PipeNotifyBufferReturn(StreamBuffer& buffer);

 private:

   void LinkPipe(sp<IBufferConsumer>& consumer);

   void UnlinkPipe(sp<IBufferConsumer>& consumer);

   sp<PostProcNode> FindInternalNode(const PostProcIOParam &output);

   bool IsRAWFormat(const BufferFormat &format);

   bool IsYUVFormat(const BufferFormat &format);

   bool IsJPEGFormat(const BufferFormat &format);

   bool IsFormatSupported(const std::set<BufferFormat> &formats,
                          const BufferFormat &format);

   bool IsFormatSupported(const std::set<BufferFormat> &formats,
                          const int32_t format);

   bool SupportsRAWFormat(const std::set<BufferFormat> &formats);

   bool SupportsYUVFormat(const std::set<BufferFormat> &formats);

   bool SupportsJPEGFormat(const std::set<BufferFormat> &formats);

   PostProcPipeState             state_;

   std::vector<sp<PostProcNode>> pipe_;

   IPostProc*                    context_;

   sp<PostProcFactory>           factory_;

   bool                          use_hal_jpeg_;

};

}; //namespace recorder

}; //namespace qmmf
