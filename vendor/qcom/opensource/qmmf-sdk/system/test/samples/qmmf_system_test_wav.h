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

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <sstream>
#include <string>

namespace qmmf_test {
namespace system {

class SystemTestWav
{
 public:
  static const int32_t kEOF;

  SystemTestWav();
  ~SystemTestWav();

  int32_t Configure(const ::std::string& filename_prefix, size_t* buffer_size);

  int32_t Open();
  void Close();

  int32_t Read(void* buffer);

 private:
  struct __attribute__((packed)) WavRiffHeader {
    uint32_t riff_id;
    uint32_t riff_size;
    uint32_t wave_id;

    ::std::string ToString() const {
      ::std::stringstream stream;
      stream << "riff_id[" << ::std::setbase(16) << riff_id
             << ::std::setbase(10) << "] ";
      stream << "riff_size[" << riff_size << "] ";
      stream << "wave_id[" << ::std::setbase(16) << wave_id
             << ::std::setbase(10) << "] ";
      return stream.str();
    }
  };

  struct __attribute__((packed)) WavChunkHeader {
    uint32_t format_id;
    uint32_t format_size;

    ::std::string ToString() const {
      ::std::stringstream stream;
      stream << "format_id[" << ::std::setbase(16) << format_id
             << ::std::setbase(10) << "] ";
      stream << "format_size[" << format_size << "] ";
      return stream.str();
    }
  };

  struct __attribute__((packed)) WavChunkFormat {
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;

    ::std::string ToString() const {
      ::std::stringstream stream;
      stream << "audio_format[" << audio_format << "] ";
      stream << "num_channels[" << num_channels << "] ";
      stream << "sample_rate[" << sample_rate << "] ";
      stream << "byte_rate[" << byte_rate << "] ";
      stream << "block_align[" << block_align << "] ";
      stream << "bits_per_sample[" << bits_per_sample << "] ";
      return stream.str();
    }
  };

  struct __attribute__((packed)) WavDataHeader {
    uint32_t data_id;
    uint32_t data_size;

    ::std::string ToString() const {
      ::std::stringstream stream;
      stream << "data_id[" << ::std::setbase(16) << data_id
             << ::std::setbase(10) << "] ";
      stream << "data_size[" << data_size << "] ";
      return stream.str();
    }
  };

  struct __attribute__((packed)) WavHeader {
    WavRiffHeader riff_header;
    WavChunkHeader chunk_header;
    WavChunkFormat chunk_format;
    WavDataHeader data_header;

    ::std::string ToString() const {
      ::std::stringstream stream;
      stream << "riff_header[" << riff_header.ToString() << "] ";
      stream << "chunk_header[" << chunk_header.ToString() << "] ";
      stream << "chunk_format[" << chunk_format.ToString() << "] ";
      stream << "data_header[" << data_header.ToString() << "] ";
      return stream.str();
    }
  };

  ::std::string filename_;
  WavHeader header_;
  ::std::ifstream input_;
  ::std::streampos input_start_position_;
  int32_t input_data_size_;

  // disable copy, assignment, and move
  SystemTestWav(const SystemTestWav&) = delete;
  SystemTestWav(SystemTestWav&&) = delete;
  SystemTestWav& operator=(const SystemTestWav&) = delete;
  SystemTestWav& operator=(const SystemTestWav&&) = delete;
};

}; // namespace system
}; // namespace qmmf_test
