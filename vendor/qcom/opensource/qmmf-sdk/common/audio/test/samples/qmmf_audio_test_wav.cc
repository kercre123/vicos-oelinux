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

#define TAG "AudioTestWav"

#include "common/audio/test/samples/qmmf_audio_test_wav.h"

#include <cerrno>
#include <fstream>
#include <ios>
#include <iostream>
#include <cstdint>
#include <string>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/qmmf_log.h"

namespace qmmf_test {
namespace common {
namespace audio {

using ::qmmf::AudioFormat;
using ::qmmf::common::audio::AudioBuffer;
using ::qmmf::common::audio::AudioEndPointType;
using ::qmmf::common::audio::AudioMetadata;
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

static const char *kFilenameInput = "_input";
static const char *kFilenameOutput = "_output";
static const char *kFilenameSuffix = ".wav";

const int AudioTestWav::kEOF = 1;

AudioTestWav::AudioTestWav() : input_data_size_(0), current_data_size_(0) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
}

AudioTestWav::~AudioTestWav() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
}

int32_t AudioTestWav::Configure(const string& filename_prefix,
                                const AudioEndPointType type,
                                AudioMetadata* metadata) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: filename_prefix[%s]", TAG, __func__,
               filename_prefix.c_str());
  QMMF_VERBOSE("%s: %s() INPARAM: type[%d]", TAG, __func__,
               static_cast<int>(type));

  type_ = type;

  switch (type_) {
    case AudioEndPointType::kSource:
      QMMF_VERBOSE("%s: %s() INPARAM: metadata[%s]", TAG, __func__,
                   metadata->ToString().c_str());
      filename_ = filename_prefix;
      filename_.append(kFilenameOutput);
      filename_.append(kFilenameSuffix);

      header_.riff_header.riff_id = kIdRiff;
      header_.riff_header.riff_size = 0;
      header_.riff_header.wave_id = kIdWave;

      header_.chunk_header.format_id = kIdFmt;
      header_.chunk_header.format_size = sizeof header_.chunk_format;

      header_.chunk_format.audio_format = kFormatPcm;
      header_.chunk_format.num_channels = metadata->num_channels;
      header_.chunk_format.sample_rate = metadata->sample_rate;
      header_.chunk_format.bits_per_sample = metadata->sample_size;
      header_.chunk_format.byte_rate = (metadata->sample_size / 8) *
                                       metadata->num_channels *
                                       metadata->sample_rate;
      header_.chunk_format.block_align = metadata->num_channels *
                                         (metadata->sample_size / 8);

      header_.data_header.data_id = kIdData;
      break;

    case AudioEndPointType::kSink:
      filename_ = filename_prefix;
      filename_.append(kFilenameInput);
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
            break;
        default:
            // unknown chunk, skip bytes
            input_.seekg(header_.chunk_header.format_size, ios::cur);
        }
      } while (read_more_chunks);

      if (header_.chunk_format.audio_format == kFormatPcm) {
        metadata->format = AudioFormat::kPCM;
        metadata->num_channels = header_.chunk_format.num_channels;
        metadata->sample_rate = header_.chunk_format.sample_rate;
        metadata->sample_size = header_.chunk_format.bits_per_sample;
      } else {
        QMMF_ERROR("%s: %s() WAV file[%s] is not PCM format", TAG, __func__,
                   filename_.c_str());
        return -EDOM;
      }

      input_.close();
      QMMF_VERBOSE("%s: %s() OUTPARAM: metadata[%s]", TAG, __func__,
                   metadata->ToString().c_str());
      break;
  }
  return 0;
}

int32_t AudioTestWav::Open() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  if (filename_.empty()) {
    QMMF_ERROR("%s: %s() called in unconfigured state", TAG, __func__);
    return -EPERM;
  }

  switch (type_) {
    case AudioEndPointType::kSource:
      output_.open(filename_.c_str(), ios::out | ios::binary | ios::trunc);
      if (!output_.is_open()) {
        QMMF_ERROR("%s: %s() error opening file[%s]", TAG, __func__,
                   filename_.c_str());
        return -EBADF;
      }

      output_.seekp(sizeof header_, ios::beg);
      current_data_size_ = 0;
      break;

    case AudioEndPointType::kSink:
      input_.open(filename_.c_str(), ios::in | ios::binary);
      if (!input_.is_open()) {
        QMMF_ERROR("%s: %s() error opening file[%s]", TAG, __func__,
                   filename_.c_str());
        return -EBADF;
      }

      input_.seekg(input_start_position_);
      current_data_size_ = input_data_size_;
      break;
  }
  return 0;
}

void AudioTestWav::Close() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  switch (type_) {
    case AudioEndPointType::kSource:
      if (output_.is_open()) {
        int frames = current_data_size_ / (header_.chunk_format.num_channels *
                                           header_.chunk_format.bits_per_sample
                                           / 8);
        QMMF_INFO("%s: %s() captured %d frames", TAG, __func__, frames);

        // finalize the file
        header_.data_header.data_size = frames *
                                        header_.chunk_format.block_align;
        header_.riff_header.riff_size = header_.data_header.data_size +
                                        sizeof(header_) - 8;
        output_.seekp(0, ios::beg);
        output_.write(reinterpret_cast<char*>(&header_), sizeof header_);

        output_.close();
      }
      break;

    case AudioEndPointType::kSink:
      if (input_.is_open()) input_.close();
      break;
  }
}

int32_t AudioTestWav::Read(AudioBuffer* buffer) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
               buffer->ToString().c_str());

  input_.read(reinterpret_cast<char*>(buffer->data), buffer->capacity);

  buffer->size = input_.gcount();
  current_data_size_ -= buffer->size;

  if (current_data_size_ <= 0 || input_.eof()) return kEOF;
  else return 0;
}

int32_t AudioTestWav::Write(const AudioBuffer& buffer) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
               buffer.ToString().c_str());

  streampos before = output_.tellp();
  output_.write(reinterpret_cast<const char*>(buffer.data),
                buffer.size);
  streampos after = output_.tellp();

  current_data_size_ += after - before;

  return 0;
}

}; // namespace audio
}; // namespace common
}; // namespace qmmf_test
