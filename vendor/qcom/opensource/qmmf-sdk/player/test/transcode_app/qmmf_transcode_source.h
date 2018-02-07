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

#include "qmmf_transcode_params.h"
#include "qmmf_transcode_pipe.h"

namespace qmmf {
namespace transcode {

class TranscoderSource
    : public ::qmmf::avcodec::ICodecSource,
      public ::std::enable_shared_from_this<TranscoderSource> {
 public:
  TranscoderSource(const CodecType type,
                   const ::qmmf::avcodec::CodecParam& codec_params,
                   const ::std::shared_ptr<TranscoderPipe>& pipe);
  ~TranscoderSource();

  status_t PreparePipeline();
  status_t StartCodec();
  status_t StopCodec();
  status_t DeleteCodec();
  status_t PauseCodec();
  status_t ResumeCodec();
  status_t SetCodecParameters();

  void ReleaseResources();

  status_t DequeTranscodeBuffer(TranscodeBuffer* buffer);
  status_t QueueTranscodeBuffer(const TranscodeBuffer& buffer);

  status_t GetBuffer(BufferDescriptor& buffer_descriptor,
                     void* client_data) override;
  status_t ReturnBuffer(BufferDescriptor& buffer_descriptor,
                        void* client_data) override;
  status_t NotifyPortEvent(::qmmf::avcodec::PortEventType event_type,
                           void* event_data) override;

 private:
  void AddBufferList(const ::std::vector<TranscodeBuffer>& list);

  ::std::shared_ptr<TranscoderPipe>             pipe_;
  ::std::shared_ptr<::qmmf::avcodec::IAVCodec>  avcodec_;
  CodecType                                     codec_type_;
  ::qmmf::avcodec::CodecParam                   params_;
  CodecMimeType                                 mime_;
  ::std::vector<TranscodeBuffer>                buffer_list_;
  const uint32_t                                port_index_;
  TSQueue<TranscodeBuffer>                      free_buffer_queue_;
  TSQueue<TranscodeBuffer>                      occupy_buffer_queue_;
  TSQueue<TranscodeBuffer>                      unfilled_frame_queue_;
  TSQueue<TranscodeBuffer>                      being_filled_frame_queue_;
  ::std::mutex                                  wait_for_frame_mutex_;
  ::std::condition_variable                     wait_for_frame_;
  ::std::mutex                                  wait_for_unfilled_frame_mutex_;
  ::std::condition_variable                     wait_for_unfilled_frame_;
};  // class TranscoderSource

};  // namespace transcode
};  // namespace qmmf
