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

#define TAG "RecorderPostProcNode"

#include <sys/mman.h>
#include <qcom/display/gralloc_priv.h>

#include "qmmf_postproc_node.h"

namespace qmmf {

namespace recorder {

PostProcNode::PostProcNode(int32_t Id, std::string name,
                           sp<IPostProcModule> module)
    : PostProcPlugin<PostProcNode>(this),
      in_(this),
      out_(this),
      module_(module),
      id_(Id),
      name_(name) {
  QMMF_INFO("%s:%s: Enter name %s", TAG, __func__, name_.c_str());

  module_->SetCallbacks(this);
  module_->GetCapabilities(caps_);

  mem_pool_ = new MemPool();

  state_ = PostProcNodeState::CREATED;
  QMMF_INFO("%s:%s: Exit (%p) name: %s", TAG, __func__, this, name_.c_str());
}

PostProcNode::~PostProcNode() {
  QMMF_INFO("%s:%s: Enter %s", TAG, __func__, name_.c_str());

  in_.RequestExitAndWait();
  in_.UnMapBufs();

  out_.RequestExitAndWait();

  module_->Delete();

  if (mem_pool_.get() != nullptr) {
    mem_pool_.clear();
  }

  QMMF_INFO("%s:%s: Exit (%p) name: %s", TAG, __func__, this, name_.c_str());
}

status_t PostProcNode::Initialize(const PostProcIOParam &in_param,
                                  const PostProcIOParam &out_param) {
  QMMF_VERBOSE("%s:%s:%s: Enter", TAG, __func__, name_.c_str());
  std::lock_guard<std::mutex> lock(state_lock_);
  if (state_ != PostProcNodeState::CREATED) {
    QMMF_ERROR("%s:%s: wrong state: %d", TAG, __func__, state_);
    return BAD_VALUE;
  }

  memset(&mem_pool_params_, 0x0, sizeof(mem_pool_params_));
  mem_pool_params_.width = out_param.width;
  mem_pool_params_.height = out_param.height;
  mem_pool_params_.format = Common::FromQmmfToHalFormat(out_param.format);

  mem_pool_params_.max_buffer_count = out_param.buffer_count;
  mem_pool_params_.gralloc_flags = out_param.gralloc_flags;

  if (out_param.format == BufferFormat::kBLOB) {
    mem_pool_params_.max_size = out_param.width * out_param.height;
  }
  auto ret = mem_pool_->Initialize(mem_pool_params_);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s:%s: Failed to initialize Memory pool: %d", TAG, __func__,
        name_.c_str(), ret);
    return ret;
  }

  QMMF_INFO("%s:%s:%s: Input:  dim: %dx%d fmt: %x", TAG, __func__, name_.c_str(),
      in_param.width, in_param.height, in_param.format);
  QMMF_INFO("%s:%s:%s: Output: dim: %dx%d fmt: %x", TAG, __func__, name_.c_str(),
      out_param.width, out_param.height, out_param.format);

  ret = module_->Initialize(in_param, out_param);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s:%s: fail to create ret: %d", TAG, __func__,
        name_.c_str(), ret);
    return ret;
  }

  QMMF_VERBOSE("%s:%s:%s: Exit id: %d", TAG, __func__, name_.c_str(), id_);

  state_ = PostProcNodeState::INITIALIZED;
  return NO_ERROR;
}

status_t PostProcNode::Configure(const std::string &config_json_data) {

  std::lock_guard<std::mutex> lock(state_lock_);
  auto ret = module_->Configure(config_json_data);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s:%s: fail to configure ret: %d", TAG, __func__,
        name_.c_str(), ret);
    return ret;
  }

  return NO_ERROR;
}

PostProcIOParam PostProcNode::GetInput(const PostProcIOParam &out) {
  QMMF_VERBOSE("%s:%s:%s: Enter", TAG, __func__, name_.c_str());
  return module_->GetInput(out);
}

status_t PostProcNode::ValidateOutput(const PostProcIOParam &out) {
  QMMF_VERBOSE("%s:%s:%s: Enter", TAG, __func__, name_.c_str());
  return module_->ValidateOutput(out);
}

status_t PostProcNode::AddConsumer(sp<IBufferConsumer>& consumer) {
  QMMF_VERBOSE("%s:%s:%s: Enter", TAG, __func__, name_.c_str());

  std::lock_guard<std::mutex> lock(state_lock_);
  if (state_ != PostProcNodeState::INITIALIZED) {
    QMMF_ERROR("%s:%s:%s: Incorrect state: %d", TAG, __func__,
        name_.c_str(), state_);
    return INVALID_OPERATION;
  }

  if (consumer == nullptr) {
    QMMF_ERROR("%s:%s:%s: Consumer is NULL", TAG, __func__, name_.c_str());
    return BAD_VALUE;
  }

  AttachConsumer(consumer);

  state_ = PostProcNodeState::LINKED;

  QMMF_INFO("%s:%s:%s: Consumer(%p) has been added.", TAG, __func__,
      name_.c_str(), consumer.get());

  return NO_ERROR;
}

status_t PostProcNode::RemoveConsumer(sp<IBufferConsumer>& consumer) {
  QMMF_VERBOSE("%s:%s:%s: Enter", TAG, __func__, name_.c_str());

  QMMF_VERBOSE("%s:%s:%s Enter consumer(%p)", TAG, __func__,name_.c_str(),
      consumer.get());
  std::lock_guard<std::mutex> lock(state_lock_);
  if (state_ != PostProcNodeState::LINKED) {
    QMMF_ERROR("%s:%s:%s: Incorrect state: %d", TAG, __func__,
        name_.c_str(), state_);
    return INVALID_OPERATION;
  }

  DetachConsumer(consumer);

  state_ = PostProcNodeState::INITIALIZED;

  QMMF_VERBOSE("%s:%s:%s Exit", TAG, __func__, name_.c_str());
  return NO_ERROR;
}

status_t PostProcNode::Start(const int32_t stream_id) {
  QMMF_VERBOSE("%s:%s:%s: Enter", TAG, __func__, name_.c_str());
  status_t ret = NO_ERROR;

  QMMF_INFO("%s:%s:%s: Enter Start. State: %d", TAG, __func__,
      name_.c_str(), state_);

  std::lock_guard<std::mutex> lock(state_lock_);
  if (state_ != PostProcNodeState::LINKED) {
    QMMF_ERROR("%s:%s:%s: wrong state_: %d ", TAG, __func__,
        name_.c_str(), state_);
    return BAD_VALUE;
  }

  state_ = PostProcNodeState::STARTING;

  ret = module_->Start(stream_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s:%s: fail to start module ret: %d", TAG, __func__,
        name_.c_str(), ret);
    return ret;
  }

  in_.Run("InputHandler");
  out_.Run("OutputHandler");

  state_ = PostProcNodeState::ACTIVE;

  QMMF_INFO("%s:%s:%s: Exit", TAG, __func__, name_.c_str());

  return ret;
}

status_t PostProcNode::Stop() {
  QMMF_VERBOSE("%s:%s:%s: Enter", TAG, __func__, name_.c_str());
  status_t ret = NO_ERROR;

  QMMF_INFO("%s:%s:%s: Enter stop. State: %d", TAG, __func__,
      name_.c_str(), state_);

  {
    std::lock_guard<std::mutex> lock(state_lock_);
    state_ = PostProcNodeState::STOPPING;
  }

  in_.RequestExitAndWait();
  in_.FlushBufs([this] (StreamBuffer &buf) -> void
             { NotifyBufferReturn(buf); } );

  QMMF_VERBOSE("%s:%s:%s: The Lip thread is stopped Id_: %d", TAG, __func__,
      name_.c_str(), id_);

  ret = module_->Stop();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s:%s: fail to stop module ret: %d", TAG, __func__,
        name_.c_str(), ret);
    return ret;
  }

  out_.RequestExitAndWait();
  QMMF_VERBOSE("%s:%s:%s: The Node thread is stopped", TAG, __func__,
      name_.c_str());

  out_.FlushBufs([this] (StreamBuffer &buf) -> void
          { NotifyBufferReturned(buf); } );

  {
    std::lock_guard<std::mutex> lock(state_lock_);
    state_ = PostProcNodeState::LINKED;
  }

  QMMF_INFO("%s:%s:%s: Exit stop. State: %d", TAG, __func__,
      name_.c_str(), state_);

  return ret;
}

void PostProcNode::OnFrameAvailable(StreamBuffer& buffer) {
  QMMF_VERBOSE("%s:%s:%s: StreamBuffer(0x%p) fd: %d stream_id: %d ts: %lld",
      TAG, __func__, name_.c_str(), buffer.handle, buffer.fd,
      buffer.stream_id, buffer.timestamp);

  std::lock_guard<std::mutex> lock(state_lock_);
  if (state_ != PostProcNodeState::ACTIVE) {
    QMMF_ERROR("%s:%s:%s: Buffer not processed. Incorrect state: %d", TAG,
        __func__, name_.c_str(), state_);
    NotifyBufferReturn(buffer);
  } else {
    in_.AddBuf(buffer);
  }
  QMMF_VERBOSE("%s:%s:%s: Exit", TAG, __func__, name_.c_str());
}

void PostProcNode::OnFrameProcessed(const StreamBuffer &input_buffer) {
  QMMF_VERBOSE("%s:%s:%s: StreamBuffer(0x%p) fd: %d stream_id: %d ts: %lld",
      TAG, __func__, name_.c_str(), input_buffer.handle, input_buffer.fd,
      input_buffer.stream_id, input_buffer.timestamp);

  NotifyBufferReturn(const_cast<StreamBuffer&>(input_buffer));
}

void PostProcNode::OnFrameReady(const StreamBuffer &output_buffer) {
  QMMF_VERBOSE("%s:%s:%s: StreamBuffer(0x%p) fd: %d stream_id: %d ts: %lld",
      TAG, __func__, name_.c_str(), output_buffer.handle, output_buffer.fd,
      output_buffer.stream_id, output_buffer.timestamp);

  out_.AddBuf(const_cast<StreamBuffer&>(output_buffer));
}

void PostProcNode::OnError(RuntimeError err) {
  QMMF_ERROR("%s:%s:%s: Error %d", TAG, __func__, name_.c_str(), err);
}

void PostProcNode::AddResult(const void* result) {
  module_->AddResult(result);
}

void PostProcNode::NotifyBufferReturned(StreamBuffer& buffer) {
  QMMF_VERBOSE("%s:%s:%s: StreamBuffer(0x%p) fd: %d stream_id: %d ts: %lld",
      TAG, __func__, name_.c_str(), buffer.handle, buffer.fd, buffer.stream_id,
      buffer.timestamp);

  if (caps_.inplace_processing_ == false) {
    if (caps_.output_buff_ == 0) {
      // Return buffer back to module.
      module_->ReturnBuff(buffer);
    } else {
      // Return buffer back to mem pool.
      status_t ret = mem_pool_->ReturnBufferLocked(buffer);
      if (ret != NO_ERROR) {
        QMMF_ERROR("%s:%s:%s Buffer return Error", TAG, __func__, name_.c_str());
        assert(0);
      }
    }
  } else {
    NotifyBufferReturn(buffer);
  }
  QMMF_VERBOSE("%s:%s:%s: Exit", TAG, __func__, name_.c_str());
}

status_t PostProcNode::ProcessOutputBuffer(StreamBuffer &buffer) {
  QMMF_VERBOSE("%s:%s:%s: StreamBuffer(0x%p) fd: %d stream_id: %d ts: %lld",
      TAG, __func__, name_.c_str(), buffer.handle, buffer.fd,
      buffer.stream_id, buffer.timestamp);

  // Give buffer ownership to the CameraSource
  std::lock_guard<std::mutex> lock(state_lock_);
  if (state_ == PostProcNodeState::ACTIVE) {
  QMMF_VERBOSE("%s:%s:%s: StreamBuffer(0x%p) fd: %d stream_id: %d ts: %lld "
      "Notify to next node!", TAG, __func__, name_.c_str(), buffer.handle,
      buffer.fd, buffer.stream_id, buffer.timestamp);
    NotifyBuffer(buffer);
  } else {
    NotifyBufferReturned(buffer);
  }
  return NO_ERROR;
}

void InputHandler::AddBuf(StreamBuffer& buffer) {
  std::unique_lock<std::mutex> lock(wait_lock_);
  bufs_list_.push_back(buffer);
  wait_.notify_one();
}


void InputHandler::FlushBufs(std::function<void(StreamBuffer&)> BuffHandler) {
  std::unique_lock<std::mutex> lock(wait_lock_);
  for (auto& buffer : bufs_list_) {
    QMMF_INFO("%s:%s: back to client node: FD: %d", TAG, __func__, buffer.fd);
    BuffHandler(buffer);
  }
  bufs_list_.clear();
  UnMapBufs();
}

status_t InputHandler::MapBuf(StreamBuffer& buffer) {
  void *vaaddr = nullptr;

  if (buffer.fd == -1) {
    QMMF_ERROR("%s:%s: Error Invalid FD", TAG, __func__);
    return BAD_VALUE;
  }

  QMMF_DEBUG("%s:%s:%s buffer.fd=%d buffer.size=%d", TAG, __func__,
      node_->GetName().c_str(), buffer.fd, buffer.size);

  if (mapped_buffs_.count(buffer.fd) == 0) {
    vaaddr = mmap(nullptr, buffer.size, PROT_READ  | PROT_WRITE,
        MAP_SHARED, buffer.fd, 0);
    if (vaaddr == MAP_FAILED) {
        QMMF_ERROR("%s:%s:%s  ION mmap failed: error(%s):(%d)", TAG, __func__,
            node_->GetName().c_str(), strerror(errno), errno);;
        return BAD_VALUE;
    }
    buffer.data = vaaddr;
    map_data_t map;
    map.addr = vaaddr;
    map.size = buffer.size;
    mapped_buffs_[buffer.fd] = map;
    buffer.data = vaaddr;
  } else {
    buffer.data = mapped_buffs_[buffer.fd].addr;
  }

  return NO_ERROR;
}

void InputHandler::UnMapBufs() {
  for (auto iter : mapped_buffs_) {
    auto map = iter.second;
    if (map.addr) {
      QMMF_INFO("%s:%s:%s Unmap addr(%p) size(%d)", TAG, __func__,
          node_->GetName().c_str(), map.addr, map.size);
      munmap(map.addr, map.size);
    }
  }
  mapped_buffs_.clear();
}

status_t InputHandler::GetInputBuffers(std::vector<StreamBuffer> &in_buffs) {
  std::unique_lock<std::mutex> lock(wait_lock_);

  std::chrono::nanoseconds wait_time(kFrameTimeout);
  while (bufs_list_.empty()) {
    auto ret = wait_.wait_for(lock, wait_time);
    if (ret == std::cv_status::timeout) {
      QMMF_DEBUG("%s:%s: Wait for frame available timed out", TAG, __func__);
      return BAD_VALUE;
    }
  }

  StreamBuffer buffer = bufs_list_.front();
  bufs_list_.pop_front();

  auto ret = MapBuf(buffer);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s:%s: fail to map buffer", TAG, __func__,
        node_->name_.c_str());
    return ret;
  }

  in_buffs.push_back(buffer);
  return NO_ERROR;
}

status_t InputHandler::GetOutputBuffers(std::vector<StreamBuffer> &out_buffs,
                                        const std::vector<StreamBuffer>
                                          &in_buffs) {
  if (node_->caps_.output_buff_ == 0) {
    return NO_ERROR;
  }

  for (auto buff : in_buffs) {
    StreamBuffer out_buff;
    memset(&out_buff, 0x0, sizeof(out_buff));
    auto ret = node_->mem_pool_->GetBuffer(&out_buff);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s:%s: fail to get buffer", TAG, __func__,
          node_->name_.c_str());
      return ret;
    }

    out_buff.stream_id = node_->id_;
    out_buff.timestamp = buff.timestamp;
    out_buff.frame_number = buff.frame_number;
    out_buff.camera_id = buff.camera_id;
    out_buff.flags = buff.flags;
    out_buff.info = buff.info;

    ret = MapBuf(out_buff);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s:%s: fail to map buffer", TAG, __func__,
          node_->name_.c_str());
      return ret;
    }

    out_buffs.push_back(out_buff);
  }

  return NO_ERROR;
}

bool InputHandler::ThreadLoop() {

  {
    std::lock_guard<std::mutex> lock(node_->state_lock_);
    if (node_->state_ != PostProcNodeState::ACTIVE) {
      // exit from main loop
      return false;
    }
  }

  std::vector<StreamBuffer> in_buffs;
  auto ret = GetInputBuffers(in_buffs);
  if (ret != NO_ERROR) {
    // timeout loop again
    return true;
  }

  std::vector<StreamBuffer> out_buffs;
  ret = GetOutputBuffers(out_buffs, in_buffs);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s Fail to get output buffer %d", TAG, __func__, ret);
    assert(0);
  }

  QMMF_VERBOSE("%s:%s: Process: FD: %d %d name: %s", TAG, __func__,
    in_buffs[0].fd, out_buffs.size() == 0 ? -1 : out_buffs[0].fd,
    node_->name_.c_str());

  ret = node_->module_->Process(in_buffs, out_buffs);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s Error %d while algo process", TAG, __func__, ret);
    assert(0);
  }

  // loop again
  return true;
}

void OutputHandler::FlushBufs(std::function<void(StreamBuffer&)> BuffHandler) {
  std::unique_lock<std::mutex> lock(wait_lock_);
  for (auto& buffer : bufs_list_) {
    QMMF_INFO("%s:%s: back to memory pool: FD: %d", TAG, __func__, buffer.fd);
    BuffHandler(buffer);
  }
  bufs_list_.clear();
}

void OutputHandler::AddBuf(StreamBuffer& buffer) {
  std::unique_lock<std::mutex> lock(wait_lock_);
  bufs_list_.push_back(buffer);
  wait_.notify_one();
}

bool OutputHandler::ThreadLoop() {
  {
    std::lock_guard<std::mutex> lock(node_->state_lock_);
    if (node_->state_ != PostProcNodeState::ACTIVE) {
      // exit main loop
      return false;
    }
  }

  StreamBuffer buffer;
  {
    std::unique_lock<std::mutex> lock(wait_lock_);

    std::chrono::nanoseconds wait_time(kFrameTimeout);
    while (bufs_list_.empty()) {
      auto ret = wait_.wait_for(lock, wait_time);
      if (ret == std::cv_status::timeout) {
        QMMF_DEBUG("%s:%s: Wait for frame available timed out", TAG, __func__);
        return true;
      }
    }

    buffer = bufs_list_.back();
    bufs_list_.pop_back();
  }

  node_->ProcessOutputBuffer(buffer);
  // loop again
  return true;
}

}; //namespace recorder.

}; //namespace qmmf.
