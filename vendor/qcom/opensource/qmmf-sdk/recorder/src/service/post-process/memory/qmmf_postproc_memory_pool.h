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

#include <libgralloc/gralloc_priv.h>
#include <utils/KeyedVector.h>
#include <utils/Mutex.h>
#include <memory>

#include "common/qmmf_common_utils.h"

#include "../plugin/qmmf_postproc_plugin.h"

namespace qmmf {

namespace recorder {

struct MemPoolParams {
  uint32_t width;
  uint32_t height;
  int32_t  format;
  int32_t  gralloc_flags;
  uint32_t max_buffer_count;
  uint32_t max_size;
};

class MemPool : public RefBase {

 public:

   MemPool();

   ~MemPool();

   int32_t Initialize(const MemPoolParams &params);

   status_t ReturnBufferLocked(const StreamBuffer &buffer);

   status_t GetBuffer(StreamBuffer* buffer);

 private:

   status_t GetBufferLocked(StreamBuffer* buffer);

   status_t PopulateMetaInfo(CameraBufferMetaData &info,
                             struct private_handle_t *priv_handle);

   status_t AllocGrallocBuffer(buffer_handle_t *buf);

   status_t FreeGrallocBuffer(buffer_handle_t buf);

   alloc_device_t               *gralloc_device_;
   buffer_handle_t              *gralloc_slots_;
   uint32_t                      buffers_allocated_;
   uint32_t                      pending_buffer_count_;
   KeyedVector<buffer_handle_t, bool> gralloc_buffers_;

   MemPoolParams            params_;

   std::mutex               buffer_lock_;
   std::condition_variable  wait_for_buffer_;

   static const uint32_t kBufferWaitTimeout = 1000000000; // 1 s.
};

}; //namespace recorder

}; //namespace qmmf
