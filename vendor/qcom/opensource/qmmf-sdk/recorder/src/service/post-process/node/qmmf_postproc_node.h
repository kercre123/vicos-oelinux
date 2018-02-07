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

#include <utils/RefBase.h>
#include <utils/Log.h>
#include <map>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include "recorder/src/service/qmmf_recorder_common.h"
#include "common/qmmf_common_utils.h"

#include "../interface/qmmf_postproc.h"
#include "../interface/qmmf_postproc_module.h"
#include "../plugin/qmmf_postproc_plugin.h"
#include "../memory/qmmf_postproc_memory_pool.h"
#include "../common/qmmf_postproc_thread.h"

namespace qmmf {

namespace recorder {

class IBufferConsumer;
class IBufferProducer;


enum class PostProcNodeState {
  CREATED,
  INITIALIZED,
  LINKED,
  STARTING,
  ACTIVE,
  STOPPING,
};

class PostProcNode;

class InputHandler : public PostProcThread {
 public:

  InputHandler(PostProcNode *node) : node_(node) {}

  void AddBuf(StreamBuffer& buffer);

  void FlushBufs(std::function<void(StreamBuffer&)> BuffHandler);

  status_t MapBuf(StreamBuffer& buffer);

  void UnMapBufs();

 protected:

  bool ThreadLoop() override;

 private:

  status_t GetInputBuffers(std::vector<StreamBuffer> &in_buffs);

  status_t GetOutputBuffers(std::vector<StreamBuffer> &out_buffs,
                            const std::vector<StreamBuffer> &in_buffs);

  struct map_data_t {
    void* addr;
    size_t size;
  };

  static const nsecs_t              kFrameTimeout  = 50000000;  // 50 ms.
  PostProcNode                      *node_;
  std::map<uint32_t, map_data_t>    mapped_buffs_;
  std::deque<StreamBuffer>          bufs_list_;
  std::mutex                        wait_lock_;
  std::condition_variable           wait_;

};

class OutputHandler : public PostProcThread {
 public:

  OutputHandler(PostProcNode *node) : node_(node) {}

  void AddBuf(StreamBuffer& buffer);

  void FlushBufs(std::function<void(StreamBuffer&)> BuffHandler);

 protected:

  bool ThreadLoop() override;

 private:

  static const nsecs_t              kFrameTimeout  = 50000000;  // 50 ms.
  PostProcNode                      *node_;
  std::vector<StreamBuffer>         bufs_list_;
  std::mutex                        wait_lock_;
  std::condition_variable           wait_;

};

class PostProcNode : public PostProcPlugin<PostProcNode>,
                     public IPostProcEventListener,
                     public RefBase {
   friend class InputHandler;
   friend class OutputHandler;

 public:
   PostProcNode(int32_t Id, std::string name, sp<IPostProcModule> module);

   ~PostProcNode();

   status_t Initialize(const PostProcIOParam &in_param,
                       const PostProcIOParam &out_param);

   status_t Configure(const std::string &config_json_data);

   PostProcIOParam GetInput(const PostProcIOParam &out);

   status_t ValidateOutput(const PostProcIOParam &out);

   void OnFrameAvailable(StreamBuffer& buffer) override;

   void NotifyBufferReturned(StreamBuffer& buffer) override;

   void OnFrameProcessed(const StreamBuffer &input_buffer)  override;

   void OnFrameReady(const StreamBuffer &output_buffer) override;

   void OnError(RuntimeError err) override;

   status_t AddConsumer(sp<IBufferConsumer>& consumer);

   status_t RemoveConsumer(sp<IBufferConsumer>& consumer);

   void AddResult(const void* result);

   status_t Start(const int32_t stream_id);

   status_t Stop();

   std::string& GetName() { return name_; }

   uint32_t GetId() { return id_; };

   PostProcCaps GetCapabilities() { return caps_; }

 private:

   StreamBuffer GetStreamBuffer(const AlgBuffer &algo_buf);

   status_t ProcessOutputBuffer(StreamBuffer &buffer);

   status_t ReturnBuffers();

   static const int32_t              circulation_buffers_ = 2;

   InputHandler                      in_;
   OutputHandler                     out_;

   sp<MemPool>                       mem_pool_;
   sp<IPostProcModule>               module_;

   MemPoolParams                     mem_pool_params_;

   int32_t                           id_;
   std::string                       name_;
   PostProcCaps                      caps_;

   PostProcNodeState                 state_;
   std::mutex                        state_lock_;
};

}; //namespace recorder

}; //namespace qmmf
