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

#include "qmmf_transcode_pipe.h"

#define TAG "TranscoderPipe"

// Uncomment to print the logs for FPS through Pipe
// #define DEBUG_PIPE_SPEED

namespace qmmf {
namespace transcode {

// specifies the time period in microseconds
static const uint64_t kPipeFrameRatePeriod = 1000000;

using ::qmmf::avcodec::IAVCodec;
using ::qmmf::avcodec::ICodecSource;
using ::qmmf::avcodec::kPortALL;
using ::qmmf::avcodec::kPortIndexInput;
using ::qmmf::avcodec::kPortIndexOutput;
using ::qmmf::avcodec::PortEventType;
using ::qmmf::avcodec::PortreconfigData;
using ::std::lock_guard;
using ::std::make_shared;
using ::std::mutex;
using ::std::shared_ptr;
using ::std::static_pointer_cast;
using ::std::string;
using ::std::stringstream;
using ::std::thread;
using ::std::unique_lock;
using ::std::vector;

TranscoderPipe::TranscoderPipe(const TranscodeType track_type)
    : track_type_(track_type),
      transport_thread_(nullptr),
      stop_transport_(false) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

TranscoderPipe::~TranscoderPipe() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

status_t TranscoderPipe::PreparePipeline() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  CodecType pipe_in_codec_type;
  CodecType pipe_out_codec_type;

  switch (track_type_) {
    case TranscodeType::kVideoDecodeVideoEncode:
      pipe_in_codec_type = CodecType::kVideoDecoder;
      pipe_out_codec_type = CodecType::kVideoEncoder;
      break;
    case TranscodeType::kVideoEncodeVideoDecode:
    case TranscodeType::kImageDecodeVideoEncode:
    default:
      QMMF_ERROR("%s:%s Unsupported TranscodeType", TAG, __func__);
      return -1;
  }

  pipe_in_ = make_shared<TranscoderPipeIn>(in_avcodec_.lock(),
                                           shared_from_this(),
                                           pipe_in_codec_type);

  pipe_out_ = make_shared<TranscoderPipeOut>(out_avcodec_.lock(),
                                             shared_from_this(),
                                             pipe_out_codec_type);

  ret = pipe_in_->PreparePipeline();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to prepare pipeline for Pipe input",
               TAG, __func__);
    return ret;
  }

  ret = pipe_out_->PreparePipeline();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to prepare pipeline for Pipe output",
               TAG, __func__);
    return ret;
  }

  transport_thread_ = new thread(TranscoderPipe::Transport,
                                 reinterpret_cast<void*>(this));
  if (transport_thread_ == nullptr) {
    QMMF_ERROR("%s:%s Unable to allocate thread", TAG, __func__);
    return -ENOMEM;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderPipe::RemovePipe() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  stop_transport_ = true;
  wait_frame_fwqueue_.notify_one();
  wait_frame_bwqueue_.notify_one();

  transport_thread_->join();
  delete transport_thread_;
  transport_thread_ = nullptr;
  QMMF_INFO("%s:%s Decoder and Encoder got disconnected", TAG, __func__);

  pipe_in_->ReleaseResources();
  pipe_out_->ReleaseResources();

  if (in_avcodec_.expired() && out_avcodec_.expired()) {
    QMMF_INFO("%s:%s Both the CodecAdaptors has been released", TAG, __func__);
  } else {
    QMMF_ERROR("%s:%s CodecAdaptors are not relased", TAG, __func__);
    return -1;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return 0;
}

void TranscoderPipe::Sendforward(const TranscodeBuffer& buffer) {
  lock_guard<mutex> lg(wait_frame_fwqueue_mutex_);
  forward_dir_queue_.PushBack(buffer);
  wait_frame_fwqueue_.notify_one();
}

void TranscoderPipe::Sendbackward(const TranscodeBuffer& buffer) {
  lock_guard<mutex> lg(wait_frame_bwqueue_mutex_);
  backward_dir_queue_.PushBack(buffer);
  wait_frame_bwqueue_.notify_one();
}

void* TranscoderPipe::Transport(void* arg) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  TranscoderPipe* ptr = reinterpret_cast<TranscoderPipe*>(arg);

  while (1) {
    QMMF_DEBUG("%s:%s %s ForwardQueueSize[%d] BackwardQueueSize[%d] %s",
               TAG, __func__,
               ptr->pipe_in_->QueueSizes().c_str(),
               ptr->forward_dir_queue_.Size(),
               ptr->backward_dir_queue_.Size(),
               ptr->pipe_out_->QueueSizes().c_str());

    if (ptr->forward_dir_queue_.Size() <= 0) {
      QMMF_DEBUG("%s:%s No buffer available in forward Queue", TAG, __func__);
      unique_lock<mutex> ul(ptr->wait_frame_fwqueue_mutex_);
      if (ptr->forward_dir_queue_.Size() <= 0)
        ptr->wait_frame_fwqueue_.wait(ul, [=] {
          return !(ptr->forward_dir_queue_.Empty()) || ptr->stop_transport_;
        });
      if (ptr->stop_transport_) {
        QMMF_INFO("%s:%s Transporting is Aborted in midway", TAG, __func__);
        break;
      }
    }
    TranscodeBuffer iter_forward = *(ptr->forward_dir_queue_.Begin());

    if (ptr->backward_dir_queue_.Size() <= 0) {
      QMMF_DEBUG("%s:%s No buffer available in backward Queue", TAG, __func__);
      unique_lock<mutex> ul(ptr->wait_frame_bwqueue_mutex_);
      if (ptr->backward_dir_queue_.Size() <= 0)
        ptr->wait_frame_bwqueue_.wait(ul, [=] {
          return !(ptr->backward_dir_queue_.Empty()) || ptr->stop_transport_;
        });
      if (ptr->stop_transport_) {
        QMMF_INFO("%s:%s Transporting is Aborted in midway", TAG, __func__);
        break;
      }
    }
    TranscodeBuffer iter_backward = *(ptr->backward_dir_queue_.Begin());

    if (ptr->track_type_ == TranscodeType::kVideoDecodeVideoEncode) {
      // Buffer can be shared
      if (iter_forward.GetFilledSize() <= 0 &&
          !(iter_forward.GetFlag() &
            static_cast<uint32_t>(BufferFlags::kFlagEOS))) {
        ptr->pipe_in_->ReceiveBuffer(iter_forward);
      } else {
        ptr->pipe_out_->ReceiveBuffer(iter_forward);
      }
      ptr->forward_dir_queue_.Erase(ptr->forward_dir_queue_.Begin());
      ptr->pipe_in_->ReceiveBuffer(iter_backward);
      ptr->backward_dir_queue_.Erase(ptr->backward_dir_queue_.Begin());
      if (iter_forward.GetFlag() &
          static_cast<uint32_t>(BufferFlags::kFlagEOS)) {
        QMMF_INFO("%s:%s Transporting Last Buffer", TAG, __func__);
        break;
      }
    }
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return nullptr;
}

TranscoderPipe::TranscoderPipeIn::TranscoderPipeIn(
    const shared_ptr<IAVCodec>& arg,
    const shared_ptr<TranscoderPipe>& parent,
    const CodecType type)
    : avcodec_(arg),
      pipe_(parent),
      port_index_(kPortIndexOutput),
      codec_type_(type),
      fps_clr_input_side_(nullptr),
      fps_clr_output_side_(nullptr) {
  QMMF_INFO("%s:(pipe_input)%s Enter", TAG, __func__);
  QMMF_INFO("%s:(pipe_input)%s Exit", TAG, __func__);
}

TranscoderPipe::TranscoderPipeIn::~TranscoderPipeIn() {
  QMMF_INFO("%s:(pipe_input)%s Enter", TAG, __func__);
  QMMF_INFO("%s:(pipe_input)%s Exit", TAG, __func__);
}

status_t TranscoderPipe::TranscoderPipeIn::PreparePipeline() {
  QMMF_INFO("%s:(pipe_input)%s Enter", TAG, __func__);

  status_t ret = 0;
  ret = TranscodeBuffer::CreateTranscodeBuffersVector(
      avcodec_, BufferOwner::kTranscoderPipeIn, port_index_, &buffer_list_);
  if (ret != 0) {
    QMMF_ERROR("%s:(pipe_input)%s Failed to allocate PipeIn buffers",
               TAG, __func__);
    return ret;
  }

  vector<BufferDescriptor> temp_pipe_in;
  for (auto& iter : buffer_list_) {
    BufferDescriptor temp_buffer;
    memset(&temp_buffer, 0x0, sizeof(temp_buffer));
    ret = iter.getAVCodecBuffer(AVCodecBufferType::kNormal, &temp_buffer);
    assert(ret == 0);
    temp_pipe_in.push_back(temp_buffer);
  }

  ret = avcodec_->RegisterOutputBuffers(temp_pipe_in);
  if (ret != 0) {
    QMMF_ERROR("%s:(pipe_input)%s Failed to register PipeIn buffers",
               TAG, __func__);
    goto release_resources;
  }

  ret = avcodec_->AllocateBuffer(
      port_index_, 0, 0,
      static_pointer_cast<ICodecSource>(shared_from_this()),
      temp_pipe_in);
  if (ret != 0) {
    QMMF_ERROR("%s:(pipe_input)%s Failed to call allocate buffer on PipeIn side",
               TAG, __func__);
    goto release_resources;
  }

  ret = AddBufferList(buffer_list_);
  if (ret != 0) {
    QMMF_ERROR("%s:(pipe_input)%s Buffer queue failed", TAG, __func__);
    goto release_resources;
  }

  fps_clr_input_side_ = new FramerateCalculator(
      kPipeFrameRatePeriod, "TranscoderPipe::(pipe_input)GetBuffer fps");
  if (fps_clr_input_side_ == nullptr) {
     QMMF_ERROR("%s:(pipe_input)%s Unable to allocate FramerateCalculator",
                TAG, __func__);
     ret = -ENOMEM;
     goto release_resources;
  }

  fps_clr_output_side_ = new FramerateCalculator(
      kPipeFrameRatePeriod, "TranscoderPipe::(pipe_input) ReturnBuffer fps");
  if (fps_clr_output_side_ == nullptr) {
    QMMF_ERROR("%s:(pipe_input)%s Unable to allocate FramerateCalculator",
               TAG, __func__);
    goto release_resources;
  }

  QMMF_INFO("%s:(pipe_input)%s Exit", TAG, __func__);
  return ret;

release_resources:
  ReleaseResources();
  QMMF_INFO("%s:(pipe_input)%s Exit", TAG, __func__);
  return ret;
}

void TranscoderPipe::TranscoderPipeIn::ReleaseResources() {
  QMMF_DEBUG("%s:(pipe_input)%s Enter", TAG, __func__);

  status_t ret = 0;

  TranscodeBuffer::FreeTranscodeBuffersVector(&buffer_list_);
  assert(buffer_list_.empty());

  if (avcodec_ != nullptr) {
    ret = avcodec_->ReleaseBuffer();
    if (ret != 0) {
      QMMF_ERROR("%s:(pipe_input)%s Failed to release buffers of source side AVCodec",
                 TAG, __func__);
    }
    avcodec_.reset();
  }

  if (fps_clr_input_side_ != nullptr) {
    delete fps_clr_input_side_;
    fps_clr_input_side_ = nullptr;
  }

  if (fps_clr_output_side_ != nullptr) {
    delete fps_clr_output_side_;
    fps_clr_output_side_ = nullptr;
  }

  QMMF_DEBUG("%s:(pipe_input)%s Exit", TAG, __func__);
}

status_t TranscoderPipe::TranscoderPipeIn::AddBufferList(
    const vector<TranscodeBuffer>& list) {
  QMMF_DEBUG("%s:(pipe_input)%s Enter", TAG, __func__);

  free_buffer_queue_.Clear();
  occupy_buffer_queue_.Clear();
  for (auto& iter : list)
    free_buffer_queue_.PushBack(iter);

  QMMF_DEBUG("%s:(pipe_input)%s Exit", TAG, __func__);
  return 0;
}

string TranscoderPipe::TranscoderPipeIn::QueueSizes() {
  stringstream stream;
  stream << "pipe_in_free_buffer_queue_size["<< free_buffer_queue_.Size()
         << "] ";
  stream << "pipe_in_occupy_buffer_queue_size[" << occupy_buffer_queue_.Size()
         << "]";
  return stream.str();
}

void TranscoderPipe::TranscoderPipeIn::ReceiveBuffer(
    const TranscodeBuffer& buffer) {
  lock_guard<mutex> lg(wait_for_frame_mutex_);
  free_buffer_queue_.PushBack(buffer);
  wait_for_frame_.notify_one();
}

status_t TranscoderPipe::TranscoderPipeIn::GetBuffer(
    BufferDescriptor& buffer_descriptor, void* client_data) {
  QMMF_VERBOSE("%s:(pipe_input)%s Enter", TAG, __func__);

  status_t ret = 0;
  if (free_buffer_queue_.Size() <= 0) {
    QMMF_DEBUG("%s:(pipe_input)%s No buffer available to notify Wait for new buffer",
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

#ifdef DEBUG_PIPE_SPEED
  fps_clr_input_side_->Trigger();
#endif

  QMMF_VERBOSE("%s:(pipe_input)%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderPipe::TranscoderPipeIn::ReturnBuffer(
    BufferDescriptor& buffer_descriptor, void* client_data) {
  QMMF_VERBOSE("%s:(pipe_input)%s Enter", TAG, __func__);

  status_t ret = 0;
  bool found = false;

#ifdef DEBUG_PIPE_SPEED
  fps_clr_output_side_->Trigger();
#endif

  auto it = occupy_buffer_queue_.Begin();

  for (; it != occupy_buffer_queue_.End(); ++it) {
    // TODO: to use BufId instead of Fd
    if (it->GetFd() == buffer_descriptor.fd) {
      QMMF_VERBOSE("%s:(pipe_input)%s Buffer found", TAG, __func__);
      if (codec_type_ == CodecType::kVideoDecoder)
        buffer_descriptor.size =
            (buffer_descriptor.size == 0 ? buffer_descriptor.size
                                         : buffer_descriptor.capacity);
      it->UpdateTranscodeBuffer(buffer_descriptor);
      shared_ptr<TranscoderPipe> sp_pipe = pipe_.lock();
      if (sp_pipe) {
        sp_pipe->Sendforward(*it);
      } else {
        QMMF_ERROR("%s:(pipe_input)%s Could not find pipe to send the buffer forward",
                   TAG, __func__);
        assert(0);
      }
      occupy_buffer_queue_.Erase(it);
      found = true;
      break;
    }
  }

  assert(found == true);

  QMMF_VERBOSE("%s:(pipe_input)%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderPipe::TranscoderPipeIn::NotifyPortEvent(
    PortEventType event_type, void* event_data) {
  QMMF_INFO("%s:(pipe_input)%s Enter", TAG, __func__);

  status_t ret = 0;
  TSQueue<TranscodeBuffer>::iterator it;
  vector<BufferDescriptor> temp_pipe_in;
  switch (event_type) {
    case PortEventType::kPortStatus:
      break;
    case PortEventType::kPortSettingsChanged:
      switch (static_cast<PortreconfigData*>(event_data)->reconfig_type) {
        case PortreconfigData::PortReconfigType::kCropParametersChanged:
          break;
        case PortreconfigData::PortReconfigType::kBufferRequirementsChanged:
          QMMF_INFO("%s:(pipe_input)%s Releasing pipeIn buffer_list_",
                    TAG, __func__);
          TranscodeBuffer::FreeTranscodeBuffersVector(&buffer_list_);
          assert(buffer_list_.empty());
          QMMF_INFO("%s:(pipe_input)%s Allocating New set of Buffers",
                    TAG, __func__);
          ret = TranscodeBuffer::CreateTranscodeBuffersVector(
              avcodec_, BufferOwner::kTranscoderPipeIn, port_index_,
              &buffer_list_);
          if (ret != 0) {
            QMMF_ERROR("%s:(pipe_input)%s Failed to allocate PipeIn buffers",
                       TAG, __func__);
            return ret;
          }
          for (auto& iter : buffer_list_) {
            BufferDescriptor temp_buffer;
            memset(&temp_buffer, 0x0, sizeof(temp_buffer));
            ret = iter.getAVCodecBuffer(AVCodecBufferType::kNormal,
                                        &temp_buffer);
            assert(ret == 0);
            temp_pipe_in.push_back(temp_buffer);
          }
          ret = avcodec_->RegisterOutputBuffers(temp_pipe_in);
          if (ret != 0) {
            QMMF_ERROR("%s:(pipe_input)%s Buffer Registration Failed",
                       TAG, __func__);
            return ret;
          }
          it = free_buffer_queue_.Begin();
          for (; it != free_buffer_queue_.End(); ++it) {
            if (it->GetOwner() == BufferOwner::kTranscoderPipeIn) {
              free_buffer_queue_.Erase(it);
            }
          }
          it = occupy_buffer_queue_.Begin();
          for (; it != occupy_buffer_queue_.End(); ++it) {
            if (it->GetOwner() == BufferOwner::kTranscoderPipeIn) {
              occupy_buffer_queue_.Erase(it);
            }
          }
          for (auto& iter : buffer_list_) {
            free_buffer_queue_.PushBack(iter);
          }
          wait_for_frame_.notify_one();
          break;
        default:
          return -1;
      }
      break;
    default:
      return -1;
  }

  QMMF_INFO("%s:(pipe_input)%s Exit", TAG, __func__);
  return 0;
}

TranscoderPipe::TranscoderPipeOut::TranscoderPipeOut(
    const shared_ptr<IAVCodec>& arg,
    const shared_ptr<TranscoderPipe>& parent,
    const CodecType type)
    : avcodec_(arg),
      pipe_(parent),
      port_index_(kPortIndexInput),
      codec_type_(type),
      fps_clr_input_side_(nullptr),
      fps_clr_output_side_(nullptr) {
  QMMF_INFO("%s:(pipe_output)%s Enter", TAG, __func__);
  QMMF_INFO("%s:(pipe_output)%s Exit", TAG, __func__);
}

TranscoderPipe::TranscoderPipeOut::~TranscoderPipeOut() {
  QMMF_INFO("%s:(pipe_output)%s Enter", TAG, __func__);
  QMMF_INFO("%s:(pipe_output)%s Exit", TAG, __func__);
}

status_t TranscoderPipe::TranscoderPipeOut::PreparePipeline() {
  QMMF_INFO("%s:(pipe_output)%s Enter", TAG, __func__);
  status_t ret = 0;

  ret = TranscodeBuffer::CreateTranscodeBuffersVector(
      avcodec_, BufferOwner::kTranscoderPipeOut, port_index_, &buffer_list_);
  if (ret != 0) {
    QMMF_ERROR("%s:(pipe_output)%s Failed to allocate PipeOut buffers",
               TAG, __func__);
    return ret;
  }

  vector<BufferDescriptor> temp_pipe_out;
  for (auto& iter : buffer_list_) {
    BufferDescriptor temp_buffer;
    memset(&temp_buffer, 0x0, sizeof(temp_buffer));
    ret = iter.getAVCodecBuffer(AVCodecBufferType::kNormal, &temp_buffer);
    assert(ret == 0);
    temp_pipe_out.push_back(temp_buffer);
  }

  ret = avcodec_->RegisterInputBuffers(temp_pipe_out);
  if (ret != 0) {
    QMMF_ERROR("%s:(pipe_output)%s Failed to register PipeOut buffers",
               TAG, __func__);
    goto release_resources;
  }

  ret = avcodec_->AllocateBuffer(
      port_index_, 0, 0,
      static_pointer_cast<ICodecSource>(shared_from_this()),
      temp_pipe_out);
  if (ret != 0) {
    QMMF_ERROR("%s:(pipe_output)%s Failed to call allocate buffer on PipeIn side",
               TAG, __func__);
    goto release_resources;
  }

  ret = AddBufferList(buffer_list_);
  if (ret != 0) {
    QMMF_ERROR("%s:(pipe_output)%s Buffer queue failed", TAG, __func__);
    goto release_resources;
  }

  fps_clr_input_side_ = new FramerateCalculator(
      kPipeFrameRatePeriod, "TranscoderPipe::(pipe_output)GetBuffer fps");
  if (fps_clr_input_side_ == nullptr) {
    QMMF_ERROR("%s:(pipe_output)%s Unable to allocate FramerateCalculator",
               TAG, __func__);
    ret = -ENOMEM;
    goto release_resources;
  }

  fps_clr_output_side_ = new FramerateCalculator(
      kPipeFrameRatePeriod, "TranscoderPipe::(pipe_output)ReturnBuffer fps");
  if (fps_clr_output_side_ == nullptr) {
    QMMF_ERROR("%s:(pipe_output)%s Unable to allocate FramerateCalculator",
               TAG, __func__);
    ret = -ENOMEM;
    goto release_resources;
  }

  QMMF_INFO("%s:(pipe_output)%s Exit", TAG, __func__);
  return ret;

release_resources:
  ReleaseResources();
  QMMF_INFO("%s:(pipe_output)%s Exit", TAG, __func__);
  return ret;
}

void TranscoderPipe::TranscoderPipeOut::ReleaseResources() {
  QMMF_DEBUG("%s:(pipe_output)%s Enter", TAG, __func__);

  status_t ret = 0;

  TranscodeBuffer::FreeTranscodeBuffersVector(&buffer_list_);
  assert(buffer_list_.empty());

  if (avcodec_ != nullptr) {
    ret = avcodec_->ReleaseBuffer();
    if (ret != 0) {
      QMMF_ERROR("%s:(pipe_output)%s Failed to release buffers of source side AVCodec",
                 TAG, __func__);
    }
    avcodec_.reset();
  }

  if (fps_clr_input_side_ != nullptr) {
    delete fps_clr_input_side_;
    fps_clr_input_side_ = nullptr;
  }

  if (fps_clr_output_side_ != nullptr) {
    delete fps_clr_output_side_;
    fps_clr_output_side_ = nullptr;
  }

  QMMF_DEBUG("%s:(pipe_output)%s Exit", TAG, __func__);
}

status_t TranscoderPipe::TranscoderPipeOut::AddBufferList(
    const vector<TranscodeBuffer>& list) {
  QMMF_DEBUG("%s:(pipe_output)%s Enter", TAG, __func__);

  status_t ret = 0;
  free_buffer_queue_.Clear();
  occupy_buffer_queue_.Clear();
  shared_ptr<TranscoderPipe> sp_pipe = pipe_.lock();
  if (sp_pipe) {
    for (auto& iter : list)
      sp_pipe->Sendbackward(iter);
  } else {
    QMMF_ERROR("%s:(pipe_output)%s Pipe doesn't exist, sp_pipe = nullptr",
               TAG, __func__);
    ret = -1;
  }

  QMMF_DEBUG("%s:(pipe_output)%s Exit", TAG, __func__);
  return ret;
}

string TranscoderPipe::TranscoderPipeOut::QueueSizes() {
  stringstream stream;
  stream << "pipe_out_free_buffer_queue_size[" << free_buffer_queue_.Size()
         << "] ";
  stream << "pipe_out_occupy_buffer_queue_size[" << occupy_buffer_queue_.Size()
         << "]";
  return stream.str();
}

void TranscoderPipe::TranscoderPipeOut::ReceiveBuffer(
    const TranscodeBuffer& buffer) {
  unique_lock<mutex> lg(wait_for_frame_mutex_);
  free_buffer_queue_.PushBack(buffer);
  wait_for_frame_.notify_one();
}

status_t TranscoderPipe::TranscoderPipeOut::GetBuffer(
    BufferDescriptor& buffer_descriptor, void* client_data) {
  QMMF_VERBOSE("%s:(pipe_output)%s Enter", TAG, __func__);

  status_t ret = 0;
  if (free_buffer_queue_.Size() <= 0) {
    QMMF_DEBUG("%s:(pipe_output)%s No buffer available to notify Wait for new buffer",
               TAG, __func__);
    unique_lock<mutex> ul(wait_for_frame_mutex_);
    if (free_buffer_queue_.Size() <= 0)
      wait_for_frame_.wait(ul);
  }

  TSQueue<TranscodeBuffer>::iterator pbuf = free_buffer_queue_.Begin();
  ret = pbuf->getAVCodecBuffer(AVCodecBufferType::kNativeHandle,
                              &buffer_descriptor);
  assert(ret == 0);
  occupy_buffer_queue_.PushBack(*pbuf);
  free_buffer_queue_.Erase(free_buffer_queue_.Begin());

  if (buffer_descriptor.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS)) {
    QMMF_INFO("%s:(pipe_output)%s Last Buffer", TAG, __func__);
    // return value of -1 signifies the end of stream on input of AVCodec
    ret = -1;
  }

#ifdef DEBUG_PIPE_SPEED
  fps_clr_input_side_->Trigger();
#endif

  QMMF_VERBOSE("%s:(pipe_output)%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderPipe::TranscoderPipeOut::ReturnBuffer(
    BufferDescriptor& buffer_descriptor, void* client_data) {
  QMMF_VERBOSE("%s:(pipe_output)%s Enter", TAG, __func__);

  status_t ret = 0;
  bool found = false;

#ifdef DEBUG_PIPE_SPEED
  fps_clr_output_side_->Trigger();
#endif

  auto it = occupy_buffer_queue_.Begin();

  for (; it != occupy_buffer_queue_.End(); ++it) {
    // TODO: to use BufId instead of MetaHandle
    if (it->GetMetaHandle() == buffer_descriptor.data) {
      QMMF_VERBOSE("%s:(pipe_output)%s Buffer found", TAG, __func__);
      shared_ptr<TranscoderPipe> sp_pipe = pipe_.lock();
      if (sp_pipe) {
        sp_pipe->Sendbackward(*it);
      } else {
        QMMF_ERROR("%s:(pipe_output)%s Could not find pipe to send the buffer forward",
                   TAG, __func__);
        assert(0);
      }
      occupy_buffer_queue_.Erase(it);
      found = true;
      break;
    }
  }

  assert(found == true);

  QMMF_VERBOSE("%s:(pipe_output)%s Exit", TAG, __func__);
  return ret;
}

status_t TranscoderPipe::TranscoderPipeOut::NotifyPortEvent(
    PortEventType event_type, void* event_data) {
  QMMF_INFO("%s:(pipe_output)%s Enter", TAG, __func__);
  QMMF_INFO("%s:(pipe_output)%s Exit", TAG, __func__);
  return 0;
}

};  // namespace transcode
};  // namespace qmmf
