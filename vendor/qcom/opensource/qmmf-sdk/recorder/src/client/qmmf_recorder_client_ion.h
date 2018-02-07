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

#include <iomanip>
#include <map>
#include <sstream>
#include <string>

#include <linux/msm_ion.h>

#include "include/qmmf-sdk/qmmf_recorder_params.h"
#include "recorder/src/client/qmmf_recorder_service_intf.h"

namespace qmmf {
namespace recorder {

class RecorderClientIon
{
 public:
  RecorderClientIon();
  ~RecorderClientIon();

  int32_t Associate(uint32_t track_id, const BnBuffer& bn_buffer,
                    BufferDescriptor* buffer);
  int32_t Release(uint32_t track_id);

 private:
  struct RecorderClientIonBuffer {
    void* data;
    int32_t capacity;
    struct ion_fd_data share_data;
    struct ion_handle_data free_data;

    ::std::string ToString() const {
      ::std::stringstream stream;
      stream << "data[" << data << "] ";
      stream << "capacity[" << capacity << "] ";
      stream << "share_data[";
      stream << "handle[" << share_data.handle << "] ";
      stream << "fd[" << share_data.fd << "] ";
      stream << "free_data[";
      stream << "handle[" << free_data.handle << "]]";
      return stream.str();
    }
  };

  typedef ::std::map<int32_t,
                     RecorderClientIonBuffer> RecorderClientIonBufferMap;

  int32_t ion_device_;
  ::std::map<uint32_t, RecorderClientIonBufferMap> buffer_map_;

  // disable copy, assignment, and move
  RecorderClientIon(const RecorderClientIon&) = delete;
  RecorderClientIon(RecorderClientIon&&) = delete;
  RecorderClientIon& operator=(const RecorderClientIon&) = delete;
  RecorderClientIon& operator=(const RecorderClientIon&&) = delete;
};

}; // namespace recorder
}; // namespace qmmf
