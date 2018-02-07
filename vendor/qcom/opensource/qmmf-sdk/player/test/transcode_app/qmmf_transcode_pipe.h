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

namespace qmmf {
namespace transcode {

class TranscoderPipe : public ::std::enable_shared_from_this<TranscoderPipe> {
 public:
  TranscoderPipe(const TranscodeType track_type);
  ~TranscoderPipe();

  status_t PreparePipeline();
  status_t RemovePipe();

  void ReleaseResources() {
    pipe_in_->ReleaseResources();
    pipe_out_->ReleaseResources();
  }

  inline void PassInputCodec(
      const ::std::shared_ptr<::qmmf::avcodec::IAVCodec>& arg) {
    in_avcodec_ = arg;
  }

  inline void PassOutputCodec(
      const ::std::shared_ptr<::qmmf::avcodec::IAVCodec>& arg) {
    out_avcodec_ = arg;
  }

  void Sendforward(const TranscodeBuffer& buffer);
  void Sendbackward(const TranscodeBuffer& buffer);
  static void* Transport(void* arg);

 private:
  class TranscoderPipeIn
      : public ::qmmf::avcodec::ICodecSource,
        public ::std::enable_shared_from_this<TranscoderPipeIn> {
   public:
    TranscoderPipeIn(const ::std::shared_ptr<::qmmf::avcodec::IAVCodec>& arg,
                     const ::std::shared_ptr<TranscoderPipe>& parent,
                     const CodecType type);
    ~TranscoderPipeIn();

    status_t PreparePipeline();
    ::std::string QueueSizes();
    void ReleaseResources();
    void ReceiveBuffer(const TranscodeBuffer& buffer);

    status_t GetBuffer(BufferDescriptor& buffer_descriptor,
                       void* client_data) override;
    status_t ReturnBuffer(BufferDescriptor& buffer_descriptor,
                          void* client_data) override;
    status_t NotifyPortEvent(::qmmf::avcodec::PortEventType event_type,
                             void* event_data) override;

   private:
    status_t AddBufferList(const ::std::vector<TranscodeBuffer>& list);

    ::std::vector<TranscodeBuffer>               buffer_list_;
    TSQueue<TranscodeBuffer>                     free_buffer_queue_;
    TSQueue<TranscodeBuffer>                     occupy_buffer_queue_;
    ::std::shared_ptr<::qmmf::avcodec::IAVCodec> avcodec_;
    ::std::weak_ptr<TranscoderPipe>              pipe_;
    const uint32_t                               port_index_;
    ::std::mutex                                 wait_for_frame_mutex_;
    ::std::condition_variable                    wait_for_frame_;
    CodecType                                    codec_type_;
    FramerateCalculator*                         fps_clr_input_side_;
    FramerateCalculator*                         fps_clr_output_side_;
  };

  class TranscoderPipeOut
      : public ::qmmf::avcodec::ICodecSource,
        public ::std::enable_shared_from_this<TranscoderPipeOut> {
   public:
    TranscoderPipeOut(const ::std::shared_ptr<::qmmf::avcodec::IAVCodec>& arg,
                      const ::std::shared_ptr<TranscoderPipe>& parent,
                      const CodecType type);
    ~TranscoderPipeOut();

    status_t PreparePipeline();
    ::std::string QueueSizes();
    void ReleaseResources();
    void ReceiveBuffer(const TranscodeBuffer& buffer);

    status_t GetBuffer(BufferDescriptor& buffer_descriptor,
                       void* client_data) override;
    status_t ReturnBuffer(BufferDescriptor& buffer_descriptor,
                          void* client_data) override;
    status_t NotifyPortEvent(::qmmf::avcodec::PortEventType event_type,
                             void* event_data) override;

   private:
    status_t AddBufferList(const ::std::vector<TranscodeBuffer>& list);

    ::std::vector<TranscodeBuffer>               buffer_list_;
    TSQueue<TranscodeBuffer>                     free_buffer_queue_;
    TSQueue<TranscodeBuffer>                     occupy_buffer_queue_;
    ::std::shared_ptr<::qmmf::avcodec::IAVCodec> avcodec_;
    ::std::weak_ptr<TranscoderPipe>              pipe_;
    const uint32_t                               port_index_;
    ::std::mutex                                 wait_for_frame_mutex_;
    ::std::condition_variable                    wait_for_frame_;
    CodecType                                    codec_type_;
    FramerateCalculator*                         fps_clr_input_side_;
    FramerateCalculator*                         fps_clr_output_side_;
  };

  TranscodeType                                  track_type_;
  ::std::shared_ptr<TranscoderPipeIn>            pipe_in_;
  ::std::shared_ptr<TranscoderPipeOut>           pipe_out_;
  ::std::weak_ptr<::qmmf::avcodec::IAVCodec>     in_avcodec_;
  ::std::weak_ptr<::qmmf::avcodec::IAVCodec>     out_avcodec_;
  TSQueue<TranscodeBuffer>                       forward_dir_queue_;
  TSQueue<TranscodeBuffer>                       backward_dir_queue_;
  ::std::mutex                                   wait_frame_fwqueue_mutex_;
  ::std::condition_variable                      wait_frame_fwqueue_;
  ::std::mutex                                   wait_frame_bwqueue_mutex_;
  ::std::condition_variable                      wait_frame_bwqueue_;
  ::std::thread*                                 transport_thread_;
  bool                                           stop_transport_;
};  // class TranscoderPipe

};  // namespace transcode
};  // namespace qmmf
