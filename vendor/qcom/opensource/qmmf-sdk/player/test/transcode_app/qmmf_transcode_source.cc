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

#include "qmmf_transcode_source.h"

#define TAG "TranscoderSource"

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

TranscoderSource::TranscoderSource(const CodecType type,
                                   const CodecParam& codec_params,
                                   const shared_ptr<TranscoderPipe>& pipe)
    : pipe_(pipe),
      codec_type_(type),
      params_(codec_params),
      port_index_(kPortIndexInput) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

TranscoderSource::~TranscoderSource() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

status_t TranscoderSource::PreparePipeline() {
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
      avcodec_, BufferOwner::kTranscoderSource, port_index_, &buffer_list_);
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to allocate source side buffers", TAG, __func__);
    return ret;
  }

  vector<BufferDescriptor> temp_in;
  for (auto& iter : buffer_list_) {
    BufferDescriptor temp_buffer;
    ret = iter.getAVCodecBuffer(AVCodecBufferType::kNormal, &temp_buffer);
    assert(ret == 0);
    temp_in.push_back(temp_buffer);
  }

  ret = avcodec_->RegisterInputBuffers(temp_in);
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to register source side buffers", TAG, __func__);
    goto release_resources;
  }

  // TODO: As of now these values (0, 0) after port_index_ will be ignored by
  // AVCodec. Hence, support is needed in this function in AVCodec to modify
  // the buffer requirements as per the wish of AVcodec Client
  // This means that in case of VIDEO ENCODER Input Buffer Requirements
  // and Output Buffer Requiremnts are HARDCODED. Although still the
  // avcodec client for input port can allocate the buffers as suggested
  // by GetBufferRequirements function, This flexibility is not there in
  // case of Output Port. That is why AllocateBuffers function in
  // qmmf_transcode_utils.cc has an If statement
  ret = avcodec_->AllocateBuffer(
      port_index_, 0, 0,
      static_pointer_cast<ICodecSource>(shared_from_this()),
      temp_in);
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to call allocate buffer on source side",
               TAG, __func__);
    goto release_resources;
  }

  pipe_->PassInputCodec(avcodec_);

  AddBufferList(buffer_list_);

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;

release_resources:
  ReleaseResources();
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSource::StartCodec() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  ret = avcodec_->StartCodec();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to start source side AVCodec", TAG, __func__);
    return ret;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSource::StopCodec() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  ret = avcodec_->StopCodec();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to stop source side AVCodec", TAG, __func__);
    return ret;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

void TranscoderSource::ReleaseResources() {
  QMMF_DEBUG("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  if (avcodec_ != nullptr) {
    ret = avcodec_->ReleaseBuffer();
    if (ret != 0) {
      QMMF_ERROR("%s:%s Failed to release buffers of source side AVCodec",
                 TAG, __func__);
    }
    avcodec_.reset();
  }

  TranscodeBuffer::FreeTranscodeBuffersVector(&buffer_list_);
  assert(buffer_list_.empty());

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
}

status_t TranscoderSource::DeleteCodec() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  ret = avcodec_->ReleaseBuffer();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to release buffers of source side AVCodec",
               TAG, __func__);
    return ret;
  }

  TranscodeBuffer::FreeTranscodeBuffersVector(&buffer_list_);
  assert(buffer_list_.empty());

  avcodec_.reset();

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSource::PauseCodec() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  ret = avcodec_->PauseCodec();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to pause source side AVCodec", TAG, __func__);
    return ret;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSource::ResumeCodec() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  ret = avcodec_->ResumeCodec();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to resume source side AVCodec", TAG, __func__);
    return ret;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSource::SetCodecParameters() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return 0;
}

void TranscoderSource::AddBufferList(const vector<TranscodeBuffer>& list) {
  QMMF_DEBUG("%s:%s Enter", TAG, __func__);

  free_buffer_queue_.Clear();
  occupy_buffer_queue_.Clear();
  unfilled_frame_queue_.Clear();
  being_filled_frame_queue_.Clear();

  for (auto& iter : list) {
    unfilled_frame_queue_.PushBack(iter);
  }

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
}

status_t TranscoderSource::GetBuffer(BufferDescriptor& buffer_descriptor,
                                     void* client_data) {
  QMMF_VERBOSE("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  // The size of free_buffer_queue_ has been queried twice because after
  // checking the size once in the first "if condition" when the programme
  // counter proceeds to acquire the mutex (i.e. wait_for_frame_mutex)
  // it might not be able to acquire it because "some other thread" calling
  // "function QueueTranscodeBuffer" can acquire it first, which then pushes a
  // buffer to the free_buffer_queue_ and hence making the size of
  // free_buffer_queue_ non-zero. The same logic applies everywhere in this
  // application where there is a twice check
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

  if (buffer_descriptor.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS)) {
    QMMF_INFO("%s:%s Last buffer", TAG, __func__);
    // return value of -1 signifies here end of stream according to avcodec
    ret = -1;
  }

  QMMF_VERBOSE("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSource::ReturnBuffer(BufferDescriptor& buffer_descriptor,
                                        void* client_data) {
  QMMF_VERBOSE("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  bool found = false;

  auto it = occupy_buffer_queue_.Begin();
  for (; it != occupy_buffer_queue_.End(); ++it) {
    // TODO: to use BufId instead of Vaddr
    if (it->GetVaddr() == buffer_descriptor.data) {
      QMMF_VERBOSE("%s:%s Buffer found", TAG, __func__);
      unfilled_frame_queue_.PushBack(*it);
      occupy_buffer_queue_.Erase(it);
      lock_guard<mutex> lg(wait_for_unfilled_frame_mutex_);
      wait_for_unfilled_frame_.notify_one();
      found = true;
      break;
    }
  }

  assert(found == true);

  QMMF_VERBOSE("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSource::NotifyPortEvent(PortEventType event_type,
                                           void* event_data) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return 0;
}

status_t TranscoderSource::DequeTranscodeBuffer(TranscodeBuffer* buffer) {
  QMMF_VERBOSE("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  if (unfilled_frame_queue_.Size() <= 0) {
    QMMF_DEBUG("%s:%s No buffer available to notify wait for new buffer",
               TAG, __func__);
    unique_lock<mutex> ul(wait_for_unfilled_frame_mutex_);
    if (unfilled_frame_queue_.Size() <= 0)
      wait_for_unfilled_frame_.wait(ul);
  }

  *buffer = *unfilled_frame_queue_.Begin();
  unfilled_frame_queue_.Erase(unfilled_frame_queue_.Begin());
  being_filled_frame_queue_.PushBack(*buffer);

  QMMF_VERBOSE("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderSource::QueueTranscodeBuffer(const TranscodeBuffer& buffer) {
  QMMF_VERBOSE("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  bool found = false;
  auto it = being_filled_frame_queue_.Begin();
  for (; it != being_filled_frame_queue_.End(); ++it) {
    if (it->GetBufId() == buffer.GetBufId()) {
      QMMF_VERBOSE("%s:%s Buffer found", TAG, __func__);
      *it = buffer;
      free_buffer_queue_.PushBack(*it);
      being_filled_frame_queue_.Erase(it);
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
