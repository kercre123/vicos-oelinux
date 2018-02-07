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

#include "qmmf_transcode_sink.h"

#define TAG "TranscoderSink"

namespace qmmf {
namespace transcode {

using ::qmmf::avcodec::CodecParam;
using ::qmmf::avcodec::IAVCodec;
using ::qmmf::avcodec::ICodecSource;
using ::qmmf::avcodec::kPortALL;
using ::qmmf::avcodec::kPortIndexInput;
using ::qmmf::avcodec::kPortIndexOutput;
using ::qmmf::avcodec::PortEventType;
using ::std::lock_guard;
using ::std::mutex;
using ::std::shared_ptr;
using ::std::static_pointer_cast;
using ::std::unique_lock;
using ::std::vector;

TranscoderSink::TranscoderSink(const CodecType type,
                               const CodecParam& codec_params,
                               const shared_ptr<TranscoderPipe>& pipe)
    : pipe_(pipe),
      codec_type_(type),
      params_(codec_params),
      port_index_(kPortIndexOutput),
      fps_clr_(nullptr) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

TranscoderSink::~TranscoderSink() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

status_t TranscoderSink::PreparePipeline() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  avcodec_.reset(IAVCodec::CreateAVCodec());

  switch (codec_type_) {
    case CodecType::kVideoEncoder:
      mime_ = CodecMimeType::kMimeTypeVideoEncAVC;
      break;
    case CodecType::kVideoDecoder:
      mime_ = CodecMimeType::kMimeTypeVideoDecAVC;
      break;
    case CodecType::kAudioEncoder:
      mime_ = CodecMimeType::kMimeTypeAudioEncAAC;
      break;
    case CodecType::kAudioDecoder:
      mime_ = CodecMimeType::kMimeTypeAudioDecAAC;
      break;
    case CodecType::kImageEncoder:
    case CodecType::kImageDecoder:
    default:
      QMMF_ERROR("%s:%s CodecType not supported", TAG, __func__);
      return -1;
  }

  ret = avcodec_->ConfigureCodec(mime_, params_);
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to configure codec", TAG, __func__);
    return ret;
  }

  ret = TranscodeBuffer::CreateTranscodeBuffersVector(
      avcodec_, BufferOwner::kTranscoderSink, port_index_, &buffer_list_);
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to allocate sink buffers", TAG, __func__);
    return ret;
  }

  vector<BufferDescriptor> temp_out;
  for (auto& iter : buffer_list_) {
    BufferDescriptor temp_buffer;
    ret = iter.getAVCodecBuffer(AVCodecBufferType::kNormal, &temp_buffer);
    assert(ret == 0);
    temp_out.push_back(temp_buffer);
  }

  ret = avcodec_->RegisterOutputBuffers(temp_out);
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to register sink buffers", TAG, __func__);
    goto release_resources;
  }

  ret = avcodec_->AllocateBuffer(
      port_index_, 0, 0,
      static_pointer_cast<ICodecSource>(shared_from_this()),
      temp_out);
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to call allocate buffer on sink side",
               TAG, __func__);
    goto release_resources;
  }

  pipe_->PassOutputCodec(avcodec_);

  AddBufferList(buffer_list_);

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;

release_resources:
  ReleaseResources();
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSink::StartCodec() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  fps_clr_ = new FramerateCalculator(kFrameRatePeriod,
                                     "TranscoderSink:: Transcoding at FPS");
  if (fps_clr_ == nullptr) {
    QMMF_ERROR("%s:%s Unable to allocate FramerateCalculator", TAG, __func__);
    return -ENOMEM;
  }

  ret = avcodec_->StartCodec();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to start sink side AVCodec", TAG, __func__);
    return ret;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSink::StopCodec() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  ret = avcodec_->StopCodec();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to stop sink side AVCodec", TAG, __func__);
    return ret;
  }

  if (fps_clr_ != nullptr) {
    delete fps_clr_;
    fps_clr_ = nullptr;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSink::DeleteCodec() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  ret = avcodec_->ReleaseBuffer();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to release buffers of sink side AVCodec",
               TAG, __func__);
    return ret;
  }

  TranscodeBuffer::FreeTranscodeBuffersVector(&buffer_list_);
  assert(buffer_list_.empty());

  avcodec_.reset();

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSink::PauseCodec() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  ret = avcodec_->PauseCodec();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to pause sink side AVCodec", TAG, __func__);
    return ret;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSink::ResumeCodec() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  ret = avcodec_->ResumeCodec();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to resume sink side AVCodec", TAG, __func__);
    return ret;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSink::SetCodecParameters() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return 0;
}

void TranscoderSink::AddBufferList(const vector<TranscodeBuffer>& list) {
  QMMF_DEBUG("%s:%s Enter", TAG, __func__);

  free_buffer_queue_.Clear();
  occupy_buffer_queue_.Clear();
  filled_buffer_queue_.Clear();
  being_read_buffer_queue_.Clear();

  for (auto& iter : list)
    free_buffer_queue_.PushBack(iter);

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
}

void TranscoderSink::ReleaseResources() {
  QMMF_DEBUG("%s:%s Enter", TAG, __func__);

  status_t ret = 0;

  if (avcodec_ != nullptr) {
    ret = avcodec_->ReleaseBuffer();
    if (ret != 0) {
      QMMF_ERROR("%s:%s Failed to release buffers of sink side AVCodec",
                 TAG, __func__);
    }
    avcodec_.reset();
  }

  TranscodeBuffer::FreeTranscodeBuffersVector(&buffer_list_);
  assert(buffer_list_.empty());

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
}

status_t TranscoderSink::GetBuffer(BufferDescriptor& buffer_descriptor,
                                   void* client_data) {
  QMMF_VERBOSE("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  if (free_buffer_queue_.Size() <= 0) {
    QMMF_DEBUG("%s:%s No buffer available to notify wait for new buffer",
               TAG, __func__);
    unique_lock<mutex> ul(wait_for_frame_mutex_);
    if (free_buffer_queue_.Size() <= 0)
      wait_for_frame_.wait(ul);
  }

  TSQueue<TranscodeBuffer>::iterator pbuf = free_buffer_queue_.Begin();
  ret = pbuf->getAVCodecBuffer(AVCodecBufferType::kNormal, &buffer_descriptor);
  assert(ret == 0);
  occupy_buffer_queue_.PushBack(*pbuf);
  free_buffer_queue_.Erase(free_buffer_queue_.Begin());

  QMMF_VERBOSE("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSink::ReturnBuffer(BufferDescriptor& buffer_descriptor,
                                      void* client_data) {
  QMMF_VERBOSE("%s:%s Enter", TAG, __func__);

  fps_clr_->Trigger();

  status_t ret = 0;
  bool found = false;
  auto it = occupy_buffer_queue_.Begin();
  for (; it != occupy_buffer_queue_.End(); ++it) {
    // TODO: to use BufId instead of Vaddr
    if (it->GetVaddr() == buffer_descriptor.data) {
      QMMF_VERBOSE("%s:%s Buffer found", TAG, __func__);
      it->UpdateTranscodeBuffer(buffer_descriptor);
      filled_buffer_queue_.PushBack(*it);
      occupy_buffer_queue_.Erase(it);
      lock_guard<mutex> lg(wait_for_filled_frame_mutex_);
      wait_for_filled_frame_.notify_one();
      found = true;
      break;
    }
  }

  assert(found == true);

  QMMF_VERBOSE("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSink::NotifyPortEvent(PortEventType event_type,
                                         void* event_data) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return 0;
}

status_t TranscoderSink::DequeTranscodeBuffer(TranscodeBuffer* buffer) {
  QMMF_VERBOSE("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  if (filled_buffer_queue_.Size() <= 0) {
    QMMF_DEBUG("%s:%s No buffer available to notify wait for new buffer",
               TAG, __func__);
    unique_lock<mutex> ul(wait_for_filled_frame_mutex_);
    if (filled_buffer_queue_.Size() <= 0)
      wait_for_filled_frame_.wait(ul);
  }

  *buffer = *filled_buffer_queue_.Begin();
  filled_buffer_queue_.Erase(filled_buffer_queue_.Begin());
  being_read_buffer_queue_.PushBack(*buffer);

  QMMF_VERBOSE("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSink::QueueTranscodeBuffer(const TranscodeBuffer& buffer) {
  QMMF_VERBOSE("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  bool found = false;
  auto it = being_read_buffer_queue_.Begin();
  for (; it != being_read_buffer_queue_.End(); ++it) {
    if (it->GetBufId() == buffer.GetBufId()) {
      QMMF_VERBOSE("%s:%s Buffer found", TAG, __func__);
      *it = buffer;
      free_buffer_queue_.PushBack(*it);
      being_read_buffer_queue_.Erase(it);
      lock_guard<mutex> lg(wait_for_frame_mutex_);
      wait_for_frame_.notify_one();
      found = true;
      break;
    }
  }

  assert(found == true);

  QMMF_VERBOSE("%s:%s Exit", TAG, __func__);
  return ret;
}

};  // namespace transcode
};  // namespace qmmf
