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

#define LOG_TAG "RecorderPostProcPipe"

#include "recorder/src/service/qmmf_recorder_utils.h"

#include "../interface/qmmf_postproc.h"
#include "../node/qmmf_postproc_node.h"

#include "qmmf_postproc_pipe.h"

namespace qmmf {

namespace recorder {

PostProcPipe::PostProcPipe(IPostProc* context)
    : context_(context),
      use_hal_jpeg_(false) {

  QMMF_VERBOSE("%s: Enter", __func__);

  factory_ = PostProcFactory::getInstance();
  assert(factory_.get() != nullptr);

  char prop_val[PROPERTY_VALUE_MAX];
  property_get("persist.qmmf.postproc.haljpeg", prop_val, "0");
  use_hal_jpeg_ = (0 == atoi(prop_val)) ? false : true;

  state_ = PostProcPipeState::CREATED; // todo

  QMMF_VERBOSE("%s: Exit (%p)", __func__, this);
}

PostProcPipe::~PostProcPipe() {
  QMMF_INFO("%s: Enter", __func__);

  QMMF_INFO("%s: Exit (%p)", __func__, this);
}

status_t PostProcPipe::CreatePipe(const PipeIOParam &pipe_out_param,
                                  const std::vector<uint32_t> &plugins,
                                  PipeIOParam &pipe_in_param) {
  std::shared_ptr<PostProcNode> node;

  // Add frame skip module at the begin of pipe
  if (pipe_out_param.frame_skip) {
    node = factory_->GetProcNode("FrameSkip");
    assert(node.get() != nullptr);
    pipe_.push_back(node);
  }

  // Add all required plugins to pipe
  for (auto& plugin_uid : plugins) {
    std::shared_ptr<PostProcNode> node = factory_->GetProcNode(plugin_uid);
    assert(node.get() != nullptr);
    pipe_.push_back(node);
  }

  PostProcIOParam node_out_param = {};
  node_out_param.width         = pipe_out_param.width;
  node_out_param.height        = pipe_out_param.height;
  node_out_param.stride        = pipe_out_param.stride;
  node_out_param.scanline      = pipe_out_param.scanline;
  node_out_param.frame_rate    = pipe_out_param.frame_rate;
  node_out_param.gralloc_flags = pipe_out_param.gralloc_flags;
  node_out_param.buffer_count  = pipe_out_param.buffer_count;
  node_out_param.buffer_max    = pipe_out_param.max_internal_buffers;
  node_out_param.format = Common::FromHalToQmmfFormat(pipe_out_param.format);

  // Add format conversion node if pipe is empty
  if (pipe_.empty()) {
    std::shared_ptr<PostProcNode> node = FindInternalNode(node_out_param);
    assert(node.get() != nullptr);
    QMMF_VERBOSE("%s: insert helper node %s", __func__,
        node->GetName().c_str());
    pipe_.push_back(node);
  }

  // If pipe is still empty return error
  if (pipe_.empty()) {
    QMMF_ERROR("%s: Pipe is empty", __func__);
    return BAD_VALUE;
  }

  // Check if output format is supported by last node in pipe
  PostProcCaps caps = pipe_.back()->GetCapabilities();
  if (!IsFormatSupported(caps.formats_, pipe_out_param.format)) {
    std::shared_ptr<PostProcNode> node = FindInternalNode(node_out_param);
    assert(node.get() != nullptr);
    QMMF_VERBOSE("%s: insert helper node %s", __func__,
        node->GetName().c_str());
    pipe_.push_back(node);
  }

  // Backward iteration over the pipe
  ssize_t idx = pipe_.size() - 1;

  // Validate pipeline and create internal processing nodes if necessary
  while (idx >= 0) {
    node = pipe_.at(idx);

    // Check compatibility with the previous node or pipe output
    auto ret = node->ValidateOutput(node_out_param);
    if (ret == BAD_TYPE) {
      // Unsupported format, try to fix this
      std::shared_ptr<PostProcNode> new_node =
          FindInternalNode(node_out_param);
      if (new_node.get() == nullptr) {
        QMMF_ERROR("%s: Node format incompatibility!", __func__);
        return ret;
      }

      // Increment index and insert new internal post processing node
      ++idx;
      pipe_.insert(pipe_.begin() + idx, new_node);

      QMMF_VERBOSE("%s: insert helper node %s", __func__,
          new_node->GetName().c_str());

      continue;
    } else if (ret != NO_ERROR) {
      QMMF_ERROR("%s: Node dimensions incompatibility!", __func__);
      return ret;
    }

    // todo: Get all possible inputs and iterate until the good one is found
    // The function returns only one input today
    PostProcIOParam node_in_param = node->GetInput(node_out_param);

    // Initialize node
    ret = node->Initialize(node_in_param, node_out_param);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: Failed to initialize node!", __func__);
      return ret;
    }

    // Update the output parameters for the next node
    node_out_param = node_in_param;

    // Decrement node index
    --idx;
  }

  // Save the input params from the first node in the pipe
  pipe_in_param.width         = node_out_param.width;
  pipe_in_param.height        = node_out_param.height;
  pipe_in_param.stride        = node_out_param.stride;
  pipe_in_param.scanline      = node_out_param.scanline;
  pipe_in_param.frame_rate    = node_out_param.frame_rate;
  pipe_in_param.gralloc_flags = node_out_param.gralloc_flags;
  pipe_in_param.buffer_count  = node_out_param.buffer_count;
  pipe_in_param.format = Common::FromQmmfToHalFormat(node_out_param.format);

  state_ = PostProcPipeState::INITIALIZED;

  std::string nodes;
  for (auto node : pipe_) {
    nodes.append(node->GetName());
    nodes.append(", ");
  }
  QMMF_INFO("%s: Reprocess pipe: %s", __func__, nodes.c_str());


  return NO_ERROR;
}

status_t PostProcPipe::DeletePipe() {
  auto iter = pipe_.end();
  while (iter != pipe_.begin()) {
    --iter;
    QMMF_INFO("%s: return node uid: %d name: %s", __func__,
        (*iter)->GetId(), (*iter)->GetName().c_str());
    (*iter)->Delete();
    factory_->ReturnProcNode((*iter)->GetId());
  }
  pipe_.clear();
  return NO_ERROR;
}

std::shared_ptr<PostProcNode>
PostProcPipe::FindInternalNode(const PostProcIOParam &output) {
  std::shared_ptr<PostProcNode> node;

  /* Check if a RAW re-processing or JPEG encoding node is required */
  if (IsYUVFormat(output.format)) {
    node = factory_->GetProcNode("HALReprocess", context_);
  } else if (IsJPEGFormat(output.format)) {
    if (use_hal_jpeg_) {
      node = factory_->GetProcNode("HALReprocess", context_);
    } else {
      node = factory_->GetProcNode("JpegEncode", context_);
    }
  }

  return node;
}

sp<IBufferConsumer>& PostProcPipe::GetConsumerIntf() {
  if (pipe_.empty()) {
    assert(0);
  }
  return pipe_.front()->GetConsumerIntf();
}

void PostProcPipe::PipeNotifyBufferReturn(StreamBuffer& buffer) {
  QMMF_VERBOSE("%s: Enter", __func__);
  if (pipe_.empty() == false) {
    pipe_.back()->NotifyBufferReturned(buffer);
  }
  QMMF_VERBOSE("%s: Exit", __func__);
}

void PostProcPipe::LinkPipe(sp<IBufferConsumer>& consumer) {
  sp<IBufferConsumer> tmp_c = consumer;
  auto iter = pipe_.end();
  while (iter != pipe_.begin()) {
    --iter;
    (*iter)->AddConsumer(tmp_c);
    tmp_c = (*iter)->GetConsumerIntf();
  }
  QMMF_INFO("%s: Pipe is linked! last node consumer (%p)", __func__,
      consumer.get());
}

void PostProcPipe::UnlinkPipe(sp<IBufferConsumer>& consumer) {
  sp<IBufferConsumer> tmp_c = consumer;
  auto iter = pipe_.end();
  while (iter != pipe_.begin()) {
    --iter;
    (*iter)->RemoveConsumer(tmp_c);
    tmp_c = (*iter)->GetConsumerIntf();
  }
  QMMF_INFO("%s: Pipe is unlinked!", __func__);
}

bool PostProcPipe::IsRAWFormat(const BufferFormat &format) {
  switch (format) {
    case BufferFormat::kRAW10:
    case BufferFormat::kRAW12:
    case BufferFormat::kRAW16:
      return true;
    default:
      return false;
  }
}

bool PostProcPipe::IsYUVFormat(const BufferFormat &format) {
  switch (format) {
    case BufferFormat::kNV12:
    case BufferFormat::kNV21:
    case BufferFormat::kNV12UBWC:
      return true;
    default:
      return false;
  }
}

bool PostProcPipe::IsJPEGFormat(const BufferFormat &format) {
  switch (format) {
    case BufferFormat::kBLOB:
      return true;
    default:
      return false;
  }
}

bool PostProcPipe::IsFormatSupported(const std::set<BufferFormat> &formats,
                                     const BufferFormat &format) {
  return formats.count(format) > 0 ? true : false;
}

bool PostProcPipe::IsFormatSupported(const std::set<BufferFormat> &formats,
                                     const int32_t format) {
  return formats.count(Common::FromHalToQmmfFormat(format)) > 0 ? true : false;
}

bool PostProcPipe::SupportsRAWFormat(const std::set<BufferFormat> &formats) {
  if (formats.count(BufferFormat::kRAW10) != 0 ||
      formats.count(BufferFormat::kRAW12) != 0 ||
      formats.count(BufferFormat::kRAW16) != 0) {
    return true;
  }
  return false;
}

bool PostProcPipe::SupportsYUVFormat(const std::set<BufferFormat> &formats) {
  if (formats.count(BufferFormat::kNV12) != 0 ||
      formats.count(BufferFormat::kNV21) != 0 ||
      formats.count(BufferFormat::kNV12UBWC) != 0) {
    return true;
  }
  return false;
}

bool PostProcPipe::SupportsJPEGFormat(const std::set<BufferFormat> &formats) {
  if (formats.count(BufferFormat::kBLOB) != 0) {
    return true;
  }
  return false;
}

status_t PostProcPipe::Start(const int32_t stream_id) {
  if (pipe_.empty()) {
    QMMF_ERROR("%s: Pipe is empty", __func__);
    return BAD_VALUE;
  }
  auto iter = pipe_.end();
  while (iter != pipe_.begin()) {
    --iter;
    (*iter)->Start(stream_id);
  }
  return NO_ERROR;
}

status_t PostProcPipe::Stop() {
  if (pipe_.empty()) {
    QMMF_ERROR("%s: Pipe is empty", __func__);
    return BAD_VALUE;
  }

  auto iter = pipe_.end();
  while (iter != pipe_.begin()) {
    --iter;
    (*iter)->Stop();
  }
  return NO_ERROR;
}

status_t PostProcPipe::Abort() {
  QMMF_ERROR("%s: E", __func__);

  if (pipe_.empty()) {
    QMMF_ERROR("%s: Pipe is empty", __func__);
    return BAD_VALUE;
  }

  abort_done_ = false;

  // shared pointer is used for abort reference counting
  // each node will release abort shared pointer when abort is ready
  // The last release will trigger smart pointer deleter
  std::shared_ptr<void> abort(new int(0), [this](void const *) {
    std::unique_lock<std::mutex> lock(abort_lock_);
    abort_done_ = true;
    abort_signal_.Signal();
  });

  // Send abort signal to all nodes
  auto iter = pipe_.end();
  while (iter != pipe_.begin()) {
    --iter;
    (*iter)->Abort(abort);
  }

  // release abort
  abort = nullptr;

  // wait all nodes to finish abort
  std::chrono::nanoseconds wait_time(kWaitAbortTimeout);
  std::unique_lock<std::mutex> lock(abort_lock_);
  while (abort_done_ == false) {
    auto ret = abort_signal_.WaitFor(lock, wait_time);
    if (ret != 0) {
      QMMF_ERROR("%s: Timed out on Wait", __func__);
      return TIMED_OUT;
    }
  }

  QMMF_ERROR("%s: X", __func__);

  return NO_ERROR;
}

status_t PostProcPipe::AddConsumer(sp<IBufferConsumer>& consumer) {

  QMMF_INFO("%s: Enter (%p)", __func__, consumer.get());
  if (state_ != PostProcPipeState::INITIALIZED) {
    QMMF_ERROR("%s: Incorrect state: %d", __func__, state_);
    return INVALID_OPERATION;
  }

  if (consumer == nullptr) {
    QMMF_ERROR("%s: Consumer is NULL", __func__);
    return BAD_VALUE;
  }

  LinkPipe(consumer);

  QMMF_VERBOSE("%s: Consumer(%p) has been added.", __func__,
             consumer.get());

  state_ = PostProcPipeState::READYTOSTART;
  QMMF_INFO("%s: Exit (%p)", __func__, consumer.get());
  return NO_ERROR;
}

status_t PostProcPipe::RemoveConsumer(sp<IBufferConsumer>& consumer) {

  QMMF_INFO("%s: Enter consumer=%p", __func__, consumer.get());
  if (state_ != PostProcPipeState::READYTOSTART) {
    QMMF_ERROR("%s: Incorrect state: %d", __func__, state_);
    return INVALID_OPERATION;
  }

  UnlinkPipe(consumer);

  state_ = PostProcPipeState::READYTOSTOP;
  QMMF_INFO("%s: Exit consumer=%p", __func__, consumer.get());
  return NO_ERROR;
}

void PostProcPipe::AddResult(const void* result) {
  for (auto const& node : pipe_) {
    node->AddResult(result);
  }
}

status_t PostProcPipe::Configure(const std::string &config_json_data) {
  status_t ret = NO_ERROR;
  if (state_ != PostProcPipeState::INITIALIZED) {
    QMMF_ERROR("%s: Incorrect state: %d", __func__, state_);
    return INVALID_OPERATION;
  }

  for (auto const& node : pipe_) {
    ret = node->Configure(config_json_data);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: Configuration failed for (%s) ret: %d", __func__,
          node->GetName().c_str(), ret);
    }
  }
  return ret;
}

}; //namespace recorder.

}; //namespace qmmf.

