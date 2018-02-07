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
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#include <linux/msm_ion.h>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "recorder/src/service/qmmf_recorder_common.h"

namespace qmmf {
namespace recorder {

class RecorderIon
{
 public:
  RecorderIon();
  ~RecorderIon();

  int32_t Allocate(const int32_t number, const int32_t size);
  int32_t Deallocate();

  int32_t GetList(::std::vector<::qmmf::common::audio::AudioBuffer>* buffers);
  int32_t GetList(::std::queue<BufferDescriptor>* buffers);

  int32_t Import(const BnBuffer& bn_buffer,
                 ::qmmf::common::audio::AudioBuffer* audio_buffer);
  int32_t Export(const ::qmmf::common::audio::AudioBuffer& audio_buffer,
                 BnBuffer* bn_buffer);

  int32_t Import(const BufferDescriptor& stream_buffer,
                 ::qmmf::common::audio::AudioBuffer* audio_buffer);
  int32_t Export(const ::qmmf::common::audio::AudioBuffer& audio_buffer,
                 BufferDescriptor* stream_buffer);

  int32_t Import(const BnBuffer& bn_buffer, BufferDescriptor* codec_buffer);
  int32_t Export(const BufferDescriptor& codec_buffer, BnBuffer* bn_buffer);

 private:
  struct RecorderIonBuffer {
    void* data;
    struct ion_allocation_data allocate_data;
    struct ion_fd_data share_data;
    struct ion_handle_data free_data;

    ::std::string ToString() const {
      ::std::stringstream stream;
      stream << "data[" << data << "] ";
      stream << "allocate_data[";
      stream << "len[" << allocate_data.len << "] ";
      stream << "align[" << allocate_data.align << "] ";
      stream << ::std::setbase(16);
      stream << "heap_id_mask[" << allocate_data.heap_id_mask << "] ";
      stream << "flags[" << allocate_data.flags << "] ";
      stream << ::std::setbase(10);
      stream << "handle[" << allocate_data.handle << "]] ";
      stream << "share_data[";
      stream << "handle[" << share_data.handle << "] ";
      stream << "fd[" << share_data.fd << "]] ";
      stream << "free_data[";
      stream << "handle[" << free_data.handle << "]]";
      return stream.str();
    }
  };

  typedef ::std::map<int32_t, RecorderIonBuffer> RecorderIonBufferMap;

  RecorderIonBufferMap ion_buffer_map_;
  int32_t ion_device_;
  int32_t buffer_size_;
  int32_t request_size_;

  // disable copy, assignment, and move
  RecorderIon(const RecorderIon&) = delete;
  RecorderIon(RecorderIon&&) = delete;
  RecorderIon& operator=(const RecorderIon&) = delete;
  RecorderIon& operator=(const RecorderIon&&) = delete;
};

}; // namespace recorder
}; // namespace qmmf
