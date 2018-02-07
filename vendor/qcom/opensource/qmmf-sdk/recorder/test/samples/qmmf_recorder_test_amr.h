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

#include <cstdint>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>

#include "include/qmmf-sdk/qmmf_buffer.h"
#include "include/qmmf-sdk/qmmf_recorder_params.h"

class RecorderTestAmr
{
 public:
  RecorderTestAmr();
  ~RecorderTestAmr();

  int32_t Configure(const ::std::string& filename_prefix,
                    const uint32_t track_id,
                    const ::qmmf::recorder::AudioTrackCreateParam& params);

  int32_t Open();
  void Close();

  int32_t Write(const ::qmmf::BufferDescriptor& buffer);

 private:
  ::std::mutex lock_;
  ::std::string filename_;
  ::std::ofstream output_;
  ::qmmf::recorder::AudioTrackCreateParam params_;

  // disable copy, assignment, and move
  RecorderTestAmr(const RecorderTestAmr&) = delete;
  RecorderTestAmr(RecorderTestAmr&&) = delete;
  RecorderTestAmr& operator=(const RecorderTestAmr&) = delete;
  RecorderTestAmr& operator=(const RecorderTestAmr&&) = delete;
};
