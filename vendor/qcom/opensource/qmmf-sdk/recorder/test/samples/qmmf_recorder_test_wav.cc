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

#define TAG "RecorderTestWav"

#include "recorder/test/samples/qmmf_recorder_test_wav.h"

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <ios>
#include <iostream>
#include <mutex>
#include <string>

#include "include/qmmf-sdk/qmmf_recorder_params.h"
#include "common/qmmf_log.h"

using ::qmmf::AudioFormat;
using ::qmmf::BufferDescriptor;
using ::qmmf::BufferFlags;
using ::qmmf::G711Mode;
using ::qmmf::recorder::AudioTrackCreateParam;
using ::std::ios;
using ::std::lock_guard;
using ::std::mutex;
using ::std::ofstream;
using ::std::streampos;
using ::std::string;
using ::std::to_string;

static const uint32_t kIdRiff = 0x46464952;
static const uint32_t kIdWave = 0x45564157;
static const uint32_t kIdFmt  = 0x20746d66;
static const uint32_t kIdData = 0x61746164;
static const uint32_t kIdFact = 0x74636166;
static const uint16_t kFormatPcm   = 0x0001;
static const uint16_t kFormatALaw  = 0x0006;
static const uint16_t kFormatMuLaw = 0x0007;

static const char *kFilenameSuffix = ".wav";

RecorderTestWav::RecorderTestWav()
    : current_data_size_(0),
      close_requested_(false) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
}

RecorderTestWav::~RecorderTestWav() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
}

int32_t RecorderTestWav::Configure(const string& filename_prefix,
                                   const uint32_t track_id,
                                   const AudioTrackCreateParam& params) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: filename_prefix[%s]", TAG, __func__,
               filename_prefix.c_str());
  QMMF_VERBOSE("%s: %s() INPARAM: track_id[%u]", TAG, __func__, track_id);
  QMMF_VERBOSE("%s: %s() INPARAM: params[%s]", TAG, __func__,
               params.ToString().c_str());
  lock_guard<mutex> lock(lock_);

  if (params.format != AudioFormat::kPCM &&
      params.format != AudioFormat::kG711) {
    QMMF_ERROR("%s: %s() non-WAV format given: %d", TAG, __func__,
               static_cast<int>(params.format));
    return -EINVAL;
  }

  filename_ = filename_prefix;
  filename_.append(to_string(track_id));
  filename_.append(kFilenameSuffix);

  params_ = params;

  return 0;
}

int32_t RecorderTestWav::Open() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  lock_guard<mutex> lock(lock_);

  if (filename_.empty()) {
    QMMF_ERROR("%s: %s() called in unconfigured state", TAG, __func__);
    return -EPERM;
  }

  output_.open(filename_.c_str(), ios::out | ios::binary | ios::trunc);
  if (!output_.is_open()) {
    QMMF_ERROR("%s: %s() error opening file[%s]", TAG, __func__,
               filename_.c_str());
    return -EBADF;
  }

  if (params_.format == AudioFormat::kPCM)
    output_.seekp(sizeof(WavPCMHeader), ios::beg);
  else
    output_.seekp(sizeof(WavG711Header), ios::beg);

  current_data_size_ = 0;
  close_requested_ = false;

  return 0;
}

void RecorderTestWav::Close() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  lock_guard<mutex> lock(lock_);

  if (output_.is_open())
    close_requested_ = true;
}

int32_t RecorderTestWav::Write(const BufferDescriptor& buffer) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
               buffer.ToString().c_str());
  lock_guard<mutex> lock(lock_);

  if (!output_.is_open()) {
    QMMF_WARN("%s: %s() handle is not open, skipping", TAG, __func__);
    return 0;
  }

  if (buffer.size > 0) {
    streampos before = output_.tellp();
    output_.write(reinterpret_cast<const char *>(buffer.data), buffer.size);
    streampos after = output_.tellp();
    if (after - before != buffer.size)
      QMMF_WARN("%s: %s() failed to write AAC data: size[%u] written[%llu]",
                TAG, __func__, buffer.size, after - before);

    current_data_size_ += after - before;
  }

  // finalize the file
  if (buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS) ||
      close_requested_ == true) {
    output_.seekp(0, ios::beg);
    if (params_.format == AudioFormat::kPCM)
      WritePCMHeader();
    else
      WriteG711Header();
    output_.close();
  }

  return 0;
}

void RecorderTestWav::WritePCMHeader() {
  WavPCMHeader header;
  memset(&header, 0x0, sizeof header);

  int frames = current_data_size_ / (params_.channels * params_.bit_depth / 8);
  QMMF_INFO("%s: %s() captured %d frames", TAG, __func__, frames);

  header.riff_header.riff_id = kIdRiff;
  header.riff_header.wave_id = kIdWave;

  header.chunk_header.format_id = kIdFmt;
  header.chunk_header.format_size = sizeof header.chunk_format;

  header.chunk_format.audio_format = kFormatPcm;
  header.chunk_format.num_channels = params_.channels;
  header.chunk_format.sample_rate = params_.sample_rate;
  header.chunk_format.bits_per_sample = params_.bit_depth;
  header.chunk_format.byte_rate = (params_.bit_depth / 8) * params_.channels *
                                  params_.sample_rate;
  header.chunk_format.block_align = params_.channels * (params_.bit_depth / 8);

  header.data_header.data_id = kIdData;
  header.data_header.data_size = frames * header.chunk_format.block_align;
  header.riff_header.riff_size = header.data_header.data_size +
                                 sizeof(header) - 8;

  QMMF_VERBOSE("%s: %s() writing PCM riff_header[%s]", TAG, __func__,
               header.riff_header.ToString().c_str());
  QMMF_VERBOSE("%s: %s() writing PCM chunk_header[%s]", TAG, __func__,
               header.chunk_header.ToString().c_str());
  QMMF_VERBOSE("%s: %s() writing PCM chunk_format[%s]", TAG, __func__,
               header.chunk_format.ToString().c_str());
  QMMF_VERBOSE("%s: %s() writing PCM data_header[%s]", TAG, __func__,
               header.data_header.ToString().c_str());
  output_.write(reinterpret_cast<const char*>(&header), sizeof header);
}

void RecorderTestWav::WriteG711Header() {
  WavG711Header header;
  memset(&header, 0x0, sizeof header);

  int frames = current_data_size_ / params_.channels;
  QMMF_INFO("%s: %s() captured %d frames", TAG, __func__, frames);

  header.riff_header.riff_id = kIdRiff;
  header.riff_header.wave_id = kIdWave;

  header.chunk_header.format_id = kIdFmt;
  header.chunk_header.format_size = sizeof header.chunk_format;

  if (params_.codec_params.g711.mode == G711Mode::kALaw)
    header.chunk_format.audio_format = kFormatALaw;
  else
    header.chunk_format.audio_format = kFormatMuLaw;
  header.chunk_format.num_channels = params_.channels;
  header.chunk_format.sample_rate = params_.sample_rate;
  header.chunk_format.bits_per_sample = 8;
  header.chunk_format.byte_rate = params_.channels * params_.sample_rate;
  header.chunk_format.block_align = params_.channels;

  header.fact_header.fact_id = kIdFact;
  header.fact_header.fact_size = 4;
  header.fact_header.sample_length = frames * header.chunk_format.block_align;

  header.data_header.data_id = kIdData;
  header.data_header.data_size = frames * header.chunk_format.block_align;
  header.riff_header.riff_size = header.data_header.data_size +
                                 sizeof(header) - 8;

  QMMF_VERBOSE("%s: %s() writing G711 riff_header[%s]", TAG, __func__,
               header.riff_header.ToString().c_str());
  QMMF_VERBOSE("%s: %s() writing G711 chunk_header[%s]", TAG, __func__,
               header.chunk_header.ToString().c_str());
  QMMF_VERBOSE("%s: %s() writing G711 chunk_format[%s]", TAG, __func__,
               header.chunk_format.ToString().c_str());
  QMMF_VERBOSE("%s: %s() writing G711 fact_header[%s]", TAG, __func__,
               header.fact_header.ToString().c_str());
  QMMF_VERBOSE("%s: %s() writing G711 data_header[%s]", TAG, __func__,
               header.data_header.ToString().c_str());
  output_.write(reinterpret_cast<const char*>(&header), sizeof header);
}
