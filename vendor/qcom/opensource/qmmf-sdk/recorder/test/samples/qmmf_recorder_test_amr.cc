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

#define TAG "RecorderTestAmr"

#include "recorder/test/samples/qmmf_recorder_test_amr.h"

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <ios>
#include <iostream>
#include <mutex>
#include <string>

#include "include/qmmf-sdk/qmmf_codec.h"
#include "include/qmmf-sdk/qmmf_recorder_params.h"
#include "common/qmmf_log.h"

using ::qmmf::AudioFormat;
using ::qmmf::BufferDescriptor;
using ::qmmf::recorder::AudioTrackCreateParam;
using ::std::ios;
using ::std::lock_guard;
using ::std::mutex;
using ::std::ofstream;
using ::std::streampos;
using ::std::string;
using ::std::to_string;

static const char *kFilenameSuffix = ".amr";

static const char kNarrowBandHeader[] = { 0x23,
                                          0x21,
                                          0x41,
                                          0x4D,
                                          0x52,
                                          0x0A };
static const char kWideBandHeader[] = { 0x23,
                                        0x21,
                                        0x41,
                                        0x4D,
                                        0x52,
                                        0x2D,
                                        0x57,
                                        0x42,
                                        0x0A};

RecorderTestAmr::RecorderTestAmr() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
}

RecorderTestAmr::~RecorderTestAmr() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
}

int32_t RecorderTestAmr::Configure(const string& filename_prefix,
                                   const uint32_t track_id,
                                   const AudioTrackCreateParam& params) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: filename_prefix[%s]", TAG, __func__,
               filename_prefix.c_str());
  QMMF_VERBOSE("%s: %s() INPARAM: track_id[%u]", TAG, __func__, track_id);
  QMMF_VERBOSE("%s: %s() INPARAM: params[%s]", TAG, __func__,
               params.ToString().c_str());
  lock_guard<mutex> lock(lock_);

  if (params.format != AudioFormat::kAMR) {
    QMMF_ERROR("%s: %s() non-AMR format given: %d", TAG, __func__,
               static_cast<int>(params.format));
    return -EINVAL;
  }

  filename_ = filename_prefix;
  filename_.append(to_string(track_id));
  filename_.append(kFilenameSuffix);

  params_ = params;

  return 0;
}

int32_t RecorderTestAmr::Open() {
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

  const char* header;
  size_t size;
  if (params_.codec_params.amr.isWAMR) {
    header = kWideBandHeader;
    size = sizeof kWideBandHeader;
  } else {
    header = kNarrowBandHeader;
    size = sizeof kNarrowBandHeader;
  }
  streampos before = output_.tellp();
  output_.write(header, size);
  streampos after = output_.tellp();
  if (after - before != size) {
    QMMF_ERROR("%s: %s() failed to write AMR header", TAG, __func__);
    return -EIO;
  }

  return 0;
}

void RecorderTestAmr::Close() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  lock_guard<mutex> lock(lock_);

  if (output_.is_open())
    output_.close();
}

int32_t RecorderTestAmr::Write(const BufferDescriptor& buffer) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
               buffer.ToString().c_str());
  lock_guard<mutex> lock(lock_);

  if (buffer.size == 0) {
    QMMF_WARN("%s: %s() buffer size is 0", TAG, __func__);
    return 0;
  }
  if (!output_.is_open()) {
    QMMF_WARN("%s: %s() handle is not open, skipping", TAG, __func__);
    return 0;
  }

  streampos before = output_.tellp();
  output_.write(reinterpret_cast<const char*>(buffer.data), buffer.size);
  streampos after = output_.tellp();
  if (after - before != buffer.size)
    QMMF_WARN("%s: %s() failed to write AAC data: size[%u] written[%llu]",
              TAG, __func__, buffer.size, after - before);

  return 0;
}
