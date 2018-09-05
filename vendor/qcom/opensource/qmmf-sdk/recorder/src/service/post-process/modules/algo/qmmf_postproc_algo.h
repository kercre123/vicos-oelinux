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

#include <map>
#include <mutex>
#include <string>

#include "qmmf-plugin/qmmf_alg_plugin.h"

#include "common/utils/qmmf_common_utils.h"

#include "../../interface/qmmf_postproc_module.h"

namespace qmmf {

namespace recorder {

using namespace qmmf_alg_plugin;

class PostProcAlg : public IPostProcModule,
                    public IEventListener {

 public:

  PostProcAlg(std::string lib);

  ~PostProcAlg();

  status_t Initialize(const PostProcIOParam &in_param,
                      const PostProcIOParam &out_param) override;

  status_t Delete() override;

  void SetCallbacks(IPostProcEventListener *cb) override {listener_ = cb;};

  status_t Configure(const std::string config_json_data) override;

  status_t Process(const std::vector<StreamBuffer> &in_buffers,
                   const std::vector<StreamBuffer> &out_buffers) override;

  void AddResult(const void* result) override {};

  status_t ReturnBuff(StreamBuffer &buffer) override { return NO_ERROR; };

  status_t Start(const int32_t stream_id) override;

  status_t Stop() override;

  status_t Abort(std::shared_ptr<void> &abort) override;

  PostProcIOParam GetInput(const PostProcIOParam &out) override;

  status_t ValidateOutput(const PostProcIOParam &output) override;

  status_t GetCapabilities(PostProcCaps &caps) override;

  void OnFrameProcessed(const AlgBuffer &input_buffer) override;

  void OnFrameReady(const AlgBuffer &output_buffer) override;

  void OnError(RuntimeError err) override;

 private:

  class recursive_lock_guard {
  public:
    recursive_lock_guard(std::recursive_mutex &lock) : lock_(lock) {
      lock_.lock();
    }
    ~recursive_lock_guard() {
      lock_.unlock();
    }
  private:
    std::recursive_mutex &lock_;
  };

  enum class State {
    CREATED,
    INITIALIZED,
    ACTIVE,
    RUNING,
    ABORTED
  };

  static const int32_t kBufCount = 3; // count for buffer rotation

  PixelFormat GetAlgFormat(BufferFormat format);

  BufferFormat GetQmmfFormat(PixelFormat format);

  status_t PrepareAlgBuffer(std::vector<AlgBuffer> &algo_buffs,
      const std::vector<StreamBuffer> stream_buffs);

  StreamBuffer GetStreamBuffer(const AlgBuffer &algo_buf);

  void DumpFrame(AlgBuffer buf, bool input);

  std::string                       Lib_;
  bool                              dump_in_frame_;
  bool                              dump_out_frame_;
  bool                              pass_through_;
  IPostProcEventListener            *listener_;
  void*                             lib_handle_;
  IAlgPlugin                        *algo_;
  Capabilities                      algo_caps_;

  std::map<int32_t, StreamBuffer>   buffs_;
  std::mutex                        buffs_lock_;

  std::recursive_mutex              lock_;
  State                             state_;
  std::shared_ptr<void>             abort_;
  uint32_t                          in_fight_count_;

};

}; //namespace recorder

}; //namespace qmmf
