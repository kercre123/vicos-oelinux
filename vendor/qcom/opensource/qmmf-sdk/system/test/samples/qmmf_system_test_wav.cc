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

#define LOG_TAG "SystemTestWav"

#include "system/test/samples/qmmf_system_test_wav.h"

#include <cerrno>
#include <fstream>
#include <ios>
#include <iostream>
#include <cstdint>
#include <string>

#include "common/utils/qmmf_log.h"

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
static const char *kFilenameTrigger = "_trigger";
static const char *kFilenameSuffix = ".wav";

const int SystemTestWav::kEOF = 1;

SystemTestWav::SystemTestWav() : data_size_(0), direction_(-1) {
  QMMF_DEBUG("%s() TRACE", __func__);
}

SystemTestWav::~SystemTestWav() {
  QMMF_DEBUG("%s() TRACE", __func__);
}

int32_t SystemTestWav::Configure(const string& filename_prefix,
                                 size_t* buffer_size) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: filename_prefix[%s]", __func__,
               filename_prefix.c_str());

  filename_ = filename_prefix;
  filename_.append(kFilenameKeytone);
  filename_.append(kFilenameSuffix);

  input_.open(filename_.c_str(), ios::in | ios::binary);
  if (!input_.is_open()) {
    QMMF_ERROR("%s() error opening file[%s]", __func__,
               filename_.c_str());
    return -EBADF;
  }

  input_.read(reinterpret_cast<char*>(&header_.riff_header),
              sizeof header_.riff_header);
  if (header_.riff_header.riff_id != kIdRiff ||
      header_.riff_header.wave_id != kIdWave) {
    input_.close();
    QMMF_ERROR("%s() file[%s] is not WAV format", __func__,
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
        data_size_ = header_.chunk_header.format_size;
        input_start_position_ = input_.tellg();
        read_more_chunks = false;
        *buffer_size = data_size_;
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
    QMMF_ERROR("%s() WAV file[%s] is incorrect PCM format", __func__,
               filename_.c_str());
    return -EDOM;
  }

  input_.close();
  direction_ = 0;

  QMMF_VERBOSE("%s() OUTPARAM: buffer_size[%zu]", __func__,
               *buffer_size);
  return 0;
}

int32_t SystemTestWav::Configure(const string& filename_prefix) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: filename_prefix[%s]", __func__,
               filename_prefix.c_str());

  filename_ = filename_prefix;
  filename_.append(kFilenameTrigger);
  filename_.append(kFilenameSuffix);

  header_.riff_header.riff_id = kIdRiff;
  header_.riff_header.riff_size = 0;
  header_.riff_header.wave_id = kIdWave;

  header_.chunk_header.format_id = kIdFmt;
  header_.chunk_header.format_size = sizeof header_.chunk_format;

  int32_t num_channels = 1;
  int32_t sample_rate = 16000;
  int32_t sample_size = 16;
  header_.chunk_format.audio_format = kFormatPcm;
  header_.chunk_format.num_channels = num_channels;
  header_.chunk_format.sample_rate = sample_rate;
  header_.chunk_format.bits_per_sample = sample_size;
  header_.chunk_format.byte_rate = (sample_size / 8) * num_channels *
                                   sample_rate;
  header_.chunk_format.block_align = num_channels * (sample_size / 8);

  header_.data_header.data_id = kIdData;

  direction_ = 1;
  return 0;
}

int32_t SystemTestWav::Open() {
  QMMF_DEBUG("%s() TRACE", __func__);

  if (filename_.empty()) {
    QMMF_ERROR("%s() called in unconfigured state", __func__);
    return -EPERM;
  }

  if (direction_) {
    output_.open(filename_.c_str(), ios::out | ios::binary | ios::trunc);
    if (!output_.is_open()) {
      QMMF_ERROR("%s() error opening file[%s]", __func__,
                 filename_.c_str());
      return -EBADF;
    }

    output_.seekp(sizeof header_, ios::beg);
  } else {
    input_.open(filename_.c_str(), ios::in | ios::binary);
    if (!input_.is_open()) {
      QMMF_ERROR("%s() error opening file[%s]", __func__,
                 filename_.c_str());
      return -EBADF;
    }

    input_.seekg(input_start_position_);
  }

  return 0;
}

void SystemTestWav::Close() {
  QMMF_DEBUG("%s() TRACE", __func__);

  if (direction_) {
    if (output_.is_open()) {
      int frames = data_size_ / (header_.chunk_format.num_channels *
                                 header_.chunk_format.bits_per_sample / 8);
      QMMF_INFO("%s() wrote %d frames", __func__, frames);

      // finalize the file
      header_.data_header.data_size = frames *
                                      header_.chunk_format.block_align;
      header_.riff_header.riff_size = header_.data_header.data_size +
                                      sizeof(header_) - 8;
      output_.seekp(0, ios::beg);
      output_.write(reinterpret_cast<char*>(&header_), sizeof header_);

      output_.close();
    }
  } else {
    if (input_.is_open()) input_.close();
  }

  direction_ = -1;
}

int32_t SystemTestWav::Read(void* buffer) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: buffer[%p]", __func__, buffer);

  input_.read(reinterpret_cast<char*>(buffer), data_size_);
  if (input_.gcount() != data_size_) {
    QMMF_ERROR("%s() could not read entire buffer", __func__);
    return -EIO;
  }

  return kEOF;
}

int32_t SystemTestWav::Write(void* buffer, const size_t buffer_size) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: buffer[%p]", __func__, buffer);
  QMMF_VERBOSE("%s() INPARAM: buffer_size[%zu]", __func__,
               buffer_size);

  streampos before = output_.tellp();
  output_.write(reinterpret_cast<const char*>(buffer), buffer_size);
  streampos after = output_.tellp();

  data_size_ += after - before;

  return 0;
}

}; // namespace system
}; // namespace qmmf_test
