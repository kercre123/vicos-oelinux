/*
* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#include <sys/mman.h>
#include <fcntl.h>

#include <sdm/core/layer_buffer.h>
#include <sdm/core/buffer_allocator.h>

namespace gralloc {

class IAllocController;

}  // namespace gralloc

namespace qmmf {

namespace display {

using namespace sdm;

class DisplayBufferAllocator : public BufferAllocator {
 public:
  DisplayBufferAllocator();

  DisplayError AllocateBuffer(BufferInfo *buffer_info) override;
  DisplayError FreeBuffer(BufferInfo *buffer_info) override;
  uint32_t GetBufferSize(BufferInfo *buffer_info) override;
  DisplayError GetBufferInfo(BufferInfo *buffer_info, int32_t &aligned_width,
      int32_t &aligned_height);
  DisplayError GetAllocatedBufferInfo(const BufferConfig &buffer_config,
      AllocatedBufferInfo *allocated_buffer_info) override;

 private:
  struct MetaBufferInfo {
    int alloc_type;  //!< Specifies allocation type set by the buffer allocator.
    void *base_addr; //!< Specifies base address of the allocated output buffer.
  };

  int SetBufferInfo(LayerBufferFormat format, int *target, int *flags);

  gralloc::IAllocController *alloc_controller_;
};

}; // namespace display

}; //namespace qmmf
