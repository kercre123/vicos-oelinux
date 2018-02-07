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

#define TAG "SystemTestWav"

#include "system/test/samples/qmmf_system_test_wav.h"

#include <cerrno>
#include <fstream>
#include <ios>
#include <iostream>
#include <cstdint>
#include <string>

#include "common/qmmf_log.h"

namespace qmmf_test {
namespace system {

using ::std::ifstream;
using ::std::ios;
using ::std::ofstream;
using ::std::streampos;
using ::std::string;

static const uint32_t kIdRiff = 0x46464952;
static const uint32_t kIdWave = 0x45564157;
static const uint32_t kIdFmt  = 0x20746d66;
static const uint32_t kIdData = 0x61746164;
static const uint16_t kFormatPcm = 1;

static const char *kFilenameKeytone = "_keytone";
static const char *kFilenameSuffix = ".wav";

const int SystemTestWav::kEOF = 1;

SystemTestWav::SystemTestWav() : input_data_size_(0) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
}

SystemTestWav::~SystemTestWav() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
}

int32_t SystemTestWav::Configure(const string& filename_prefix,
                                 size_t* buffer_size) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: filename_prefix[%s]", TAG, __func__,
               filename_prefix.c_str());

  filename_ = filename_prefix;
  filename_.append(kFilenameKeytone);
  filename_.append(kFilenameSuffix);

  input_.open(filename_.c_str(), ios::in | ios::binary);
  if (!input_.is_open()) {
    QMMF_ERROR("%s: %s() error opening file[%s]", TAG, __func__,
               filename_.c_str());
    return -EBADF;
  }

  input_.read(reinterpret_cast<char*>(&header_.riff_header),
              sizeof header_.riff_header);
  if (header_.riff_header.riff_id != kIdRiff ||
      header_.riff_header.wave_id != kIdWave) {
    input_.close();
    QMMF_ERROR("%s: %s() file[%s] is not WAV format", TAG, __func__,
               filename_.c_str());
    return -EDOM;
  }

  bool read_more_chunks = true;
  do {
    input_.read(reinterpret_cast<char*>(&header_.chunk_header),
                sizeof header_.chunk_header);
    switch (header_.chunk_header.format_id) {
      case kIdFmt:
        input_.read(reinterpret_cast<char*>(&header_.chunk_format),
                    sizeof header_.chunk_format);
        // if the format header is larger, skip the rest
        if (header_.chunk_header.format_size > sizeof header_.chunk_format)
          input_.seekg(header_.chunk_header.format_size -
                       sizeof header_.chunk_format, ios::cur);
        break;
      case kIdData:
        // stop looking for chunks
        input_data_size_ = header_.chunk_header.format_size;
        input_start_position_ = input_.tellg();
        read_more_chunks = false;
        *buffer_size = input_data_size_;
        break;
      default:
        // unknown chunk, skip bytes
        input_.seekg(header_.chunk_header.format_size, ios::cur);
    }
  } while (read_more_chunks);

  if (header_.chunk_format.audio_format != kFormatPcm ||
      header_.chunk_format.num_channels != 1 ||
      header_.chunk_format.sample_rate != 48000 ||
      header_.chunk_format.bits_per_sample != 16) {
    QMMF_ERROR("%s: %s() WAV file[%s] is incorrect PCM format", TAG, __func__,
               filename_.c_str());
    return -EDOM;
  }

  input_.close();
  QMMF_VERBOSE("%s: %s() OUTPARAM: buffer_size[%zu]", TAG, __func__,
               *buffer_size);
  return 0;
}

int32_t SystemTestWav::Open() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  if (filename_.empty()) {
    QMMF_ERROR("%s: %s() called in unconfigured state", TAG, __func__);
    return -EPERM;
  }

  input_.open(filename_.c_str(), ios::in | ios::binary);
  if (!input_.is_open()) {
    QMMF_ERROR("%s: %s() error opening file[%s]", TAG, __func__,
               filename_.c_str());
    return -EBADF;
  }

  input_.seekg(input_start_position_);

  return 0;
}

void SystemTestWav::Close() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  if (input_.is_open()) input_.close();
}

int32_t SystemTestWav::Read(void* buffer) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: buffer[%p]", TAG, __func__, buffer);

  input_.read(reinterpret_cast<char*>(buffer), input_data_size_);
  if (input_.gcount() != input_data_size_) {
    QMMF_ERROR("%s: %s() could not read entire buffer", TAG, __func__);
    return -EIO;
  }

  return kEOF;
}

}; // namespace system
}; // namespace qmmf_test
