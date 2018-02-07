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

#define TAG "RecorderIon"

#include "recorder/src/service/qmmf_recorder_ion.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <map>
#include <vector>

#include <linux/msm_ion.h>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/qmmf_log.h"
#include "recorder/src/service/qmmf_recorder_common.h"

namespace qmmf {
namespace recorder {

using ::qmmf::common::audio::AudioBuffer;
using ::std::map;
using ::std::queue;
using ::std::vector;

static const char* kIonFilename = "/dev/ion";
static const int kBufferAlign   = 4096;

RecorderIon::RecorderIon()
    : ion_device_(-1), buffer_size_(0), request_size_(0) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
}

RecorderIon::~RecorderIon() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  if (!ion_buffer_map_.empty()) {
    int32_t result = Deallocate();
    assert(result == 0);
    QMMF_INFO("%s: %s() deallocated all ion buffers", TAG, __func__);
  }
}

int32_t RecorderIon::Allocate(const int32_t number, const int32_t size) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: number[%d]", TAG, __func__, number);
  QMMF_VERBOSE("%s: %s() INPARAM: size[%d]", TAG, __func__, size);

  if (number <= 0) return -EINVAL;
  if (size <= 0) return -EINVAL;

  request_size_ = size;
  buffer_size_ = (size + kBufferAlign - 1) & ~(kBufferAlign - 1);

  // open ion device
  ion_device_ = open(kIonFilename, O_RDONLY);
  if (ion_device_ < 0) {
    QMMF_ERROR("%s: %s() error opening ion device: %d[%s]", TAG, __func__,
               errno, strerror(errno));
    return errno;
  }

  for (int32_t index = 0; index < number; ++index) {
    RecorderIonBuffer buffer;
    int result;

    buffer.allocate_data.len = buffer_size_;
    buffer.allocate_data.align = kBufferAlign;
    buffer.allocate_data.heap_id_mask = ION_HEAP(ION_SYSTEM_HEAP_ID);
    buffer.allocate_data.flags = 0;
    buffer.allocate_data.handle = 0;

    // allocate ion buffer
    result = ioctl(ion_device_, ION_IOC_ALLOC, &buffer.allocate_data);
    if (result < 0) {
      QMMF_ERROR("%s: %s() ION_IOC_ALLOC ioctl command failed: %d[%s]", TAG,
                 __func__, errno, strerror(errno));
      return errno;
    }

    buffer.free_data.handle = buffer.allocate_data.handle;
    buffer.share_data.handle = buffer.allocate_data.handle;
    buffer.share_data.fd = -1;

    // obtain unique fd for sharing
    result = ioctl(ion_device_, ION_IOC_SHARE, &buffer.share_data);
    if (result < 0) {
      QMMF_ERROR("%s: %s() ION_IOC_SHARE ioctl command failed: %d[%s]", TAG,
                 __func__, errno, strerror(errno));

      // on error, attempt to deallocate the ion buffer
      result = ioctl(ion_device_, ION_IOC_FREE, &buffer.free_data);
      if (result < 0) {
        QMMF_ERROR("%s: %s() ION_IOC_FREE ioctl command failed: %d[%s]", TAG,
                    __func__, errno, strerror(errno));
        QMMF_ERROR("%s: %s() [CRITICAL] ion memory has leaked", TAG, __func__);
      }
      return errno;
    }

    ion_buffer_map_.insert({buffer.share_data.fd, buffer});
  }

  // map buffers into address space
  for (RecorderIonBufferMap::value_type& buffer_value : ion_buffer_map_) {
    buffer_value.second.data = mmap(NULL, buffer_size_, PROT_READ | PROT_WRITE,
                                    MAP_SHARED,
                                    buffer_value.second.share_data.fd, 0);
    if (buffer_value.second.data == MAP_FAILED) {
      QMMF_ERROR("%s: %s() unable to map buffer[%d]: %d[%s]", TAG, __func__,
                 buffer_value.second.share_data.fd, errno, strerror(errno));
      return errno;
    }

    QMMF_VERBOSE("%s: %s() allocated ion buffer[%d][%s]", TAG, __func__,
                 buffer_value.first, buffer_value.second.ToString().c_str());
  }

  return 0;
}

int32_t RecorderIon::Deallocate() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  int result;

  if (ion_device_ == -1) {
    QMMF_WARN("%s: %s() ion device is not opened", TAG, __func__);
    return 0;
  }

  for (RecorderIonBufferMap::value_type& buffer_value : ion_buffer_map_) {
    QMMF_VERBOSE("%s: %s() deallocating ion buffer[%s]", TAG, __func__,
                 buffer_value.second.ToString().c_str());

    // unmap buffer from address space
    result = munmap(buffer_value.second.data, buffer_size_);
    if (result < 0)
      QMMF_ERROR("%s: %s() unable to unmap buffer[%d]: %d[%s]", TAG, __func__,
                 buffer_value.second.share_data.fd, errno, strerror(errno));
    buffer_value.second.data = nullptr;

    // close fd
    result = close(buffer_value.second.share_data.fd);
    if (result < 0) {
      QMMF_ERROR("%s: %s() error closing shared fd[%d]: %d[%s]", TAG, __func__,
                 buffer_value.second.share_data.fd, errno, strerror(errno));
      return errno;
    }
    buffer_value.second.share_data.fd = -1;

    // free ion buffer
    result = ioctl(ion_device_, ION_IOC_FREE, &buffer_value.second.free_data);
    if (result < 0) {
      QMMF_ERROR("%s: %s() ION_IOC_FREE ioctl command failed: %d[%s]", TAG,
                  __func__, errno, strerror(errno));
      QMMF_ERROR("%s: %s() [CRITICAL] ion memory has leaked", TAG, __func__);
    }
  }

  ion_buffer_map_.clear();
  QMMF_INFO("%s: %s() deallocated all ion buffers", TAG, __func__);

  // close ion device
  result = close(ion_device_);
  if (result < 0) {
    QMMF_ERROR("%s: %s() error closing ion device[%d]: %d[%s]", TAG, __func__,
               ion_device_, errno, strerror(errno));
    return errno;
  }

  ion_device_ = -1;
  buffer_size_ = 0;
  request_size_ = 0;

  return 0;
}

int32_t RecorderIon::GetList(vector<AudioBuffer>* buffers) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  if (ion_buffer_map_.empty()) {
    QMMF_WARN("%s: %s() no ion buffers allocated", TAG, __func__);
    return 0;
  }

  for (RecorderIonBufferMap::value_type& buffer_value : ion_buffer_map_) {
    AudioBuffer buffer = { buffer_value.second.data,
                           buffer_value.second.share_data.fd,
                           buffer_value.second.share_data.fd,
                           request_size_, 0, 0, 0 };

    QMMF_VERBOSE("%s: %s() OUTPARAM: audio_buffer[%s]", TAG, __func__,
                 buffer.ToString().c_str());
    buffers->push_back(buffer);
  }

  return 0;
}

int32_t RecorderIon::GetList(queue<BufferDescriptor>* buffers) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  if (ion_buffer_map_.empty()) {
    QMMF_WARN("%s: %s() no ion buffers allocated", TAG, __func__);
    return 0;
  }

  for (RecorderIonBufferMap::value_type& buffer_value : ion_buffer_map_) {
    BufferDescriptor buffer = { buffer_value.second.data,
      buffer_value.second.share_data.fd,
      static_cast<uint32_t>(buffer_value.second.share_data.fd), 0U,
      static_cast<uint32_t>(request_size_), 0U, 0U, 0U };

    QMMF_VERBOSE("%s: %s() OUTPARAM: codec_buffer[%s]", TAG, __func__,
                 buffer.ToString().c_str());
    buffers->push(buffer);
  }

  return 0;
}

int32_t RecorderIon::Import(const BnBuffer& bn_buffer,
                            AudioBuffer* audio_buffer) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s INPARAM: bn_buffer[%s]", TAG, __func__,
               bn_buffer.ToString().c_str());

  RecorderIonBufferMap::iterator ion_buffer_iterator =
      ion_buffer_map_.find(bn_buffer.buffer_id);
  if (ion_buffer_iterator == ion_buffer_map_.end()) {
    QMMF_ERROR("%s: %s() no ion buffer for key[%d]", TAG, __func__,
               bn_buffer.buffer_id);
    return -EINVAL;
  }

  audio_buffer->data = ion_buffer_iterator->second.data;
  audio_buffer->ion_fd = -1;
  audio_buffer->buffer_id = bn_buffer.buffer_id;
  audio_buffer->capacity = bn_buffer.capacity;
  audio_buffer->size = bn_buffer.size;
  audio_buffer->timestamp = bn_buffer.timestamp;
  audio_buffer->flags = bn_buffer.flag;

  QMMF_VERBOSE("%s: %s() OUTPARAM: audio_buffer[%s]", TAG, __func__,
               audio_buffer->ToString().c_str());
  return 0;
}

int32_t RecorderIon::Export(const AudioBuffer& audio_buffer,
                            BnBuffer* bn_buffer) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s INPARAM: audio_buffer[%s]", TAG, __func__,
               audio_buffer.ToString().c_str());

  RecorderIonBufferMap::iterator ion_buffer_iterator =
      ion_buffer_map_.find(audio_buffer.buffer_id);
  if (ion_buffer_iterator == ion_buffer_map_.end()) {
    QMMF_ERROR("%s: %s() no ion buffer for key[%d]", TAG, __func__,
               audio_buffer.buffer_id);
    return -EINVAL;
  }

  bn_buffer->ion_fd = ion_buffer_iterator->second.share_data.fd;
  bn_buffer->size = audio_buffer.size;
  bn_buffer->timestamp = audio_buffer.timestamp;
  bn_buffer->width = 0;
  bn_buffer->height = 0;
  bn_buffer->buffer_id = audio_buffer.buffer_id;
  bn_buffer->flag = audio_buffer.flags;
  bn_buffer->capacity = audio_buffer.capacity;

  QMMF_VERBOSE("%s: %s() OUTPARAM: bn_buffer[%s]", TAG, __func__,
               bn_buffer->ToString().c_str());
  return 0;
}

int32_t RecorderIon::Import(const BufferDescriptor& stream_buffer,
                            AudioBuffer* audio_buffer) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s:%s INPARAM: stream_buffer[%s]", TAG, __func__,
               stream_buffer.ToString().c_str());

  RecorderIonBufferMap::iterator ion_buffer_iterator =
      ion_buffer_map_.find(stream_buffer.fd);
  if (ion_buffer_iterator == ion_buffer_map_.end()) {
    QMMF_ERROR("%s: %s() no ion buffer for key[%d]", TAG, __func__,
               stream_buffer.fd);
    return -EINVAL;
  }

  audio_buffer->data = ion_buffer_iterator->second.data;
  audio_buffer->ion_fd = -1;
  audio_buffer->buffer_id = stream_buffer.fd;
  audio_buffer->capacity = request_size_;
  audio_buffer->size = stream_buffer.size;
  audio_buffer->timestamp = stream_buffer.timestamp;
  audio_buffer->flags = 0;

  QMMF_VERBOSE("%s: %s() OUTPARAM: audio_buffer[%s]", TAG, __func__,
               audio_buffer->ToString().c_str());
  return 0;
}

int32_t RecorderIon::Export(const AudioBuffer& audio_buffer,
                            BufferDescriptor* stream_buffer) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s:%s INPARAM: audio_buffer[%s]", TAG, __func__,
               audio_buffer.ToString().c_str());

  RecorderIonBufferMap::iterator ion_buffer_iterator =
      ion_buffer_map_.find(audio_buffer.buffer_id);
  if (ion_buffer_iterator == ion_buffer_map_.end()) {
    QMMF_ERROR("%s: %s() no ion buffer for key[%d]", TAG, __func__,
               audio_buffer.buffer_id);
    return -EINVAL;
  }

  stream_buffer->data = ion_buffer_iterator->second.data;
  stream_buffer->fd = audio_buffer.buffer_id;
  stream_buffer->size = audio_buffer.size;
  stream_buffer->timestamp = audio_buffer.timestamp;
  stream_buffer->flag = audio_buffer.flags;

  QMMF_VERBOSE("%s: %s() OUTPARAM: stream_buffer[%s]", TAG, __func__,
               stream_buffer->ToString().c_str());
  return 0;
}

int32_t RecorderIon::Import(const BnBuffer& bn_buffer,
                            BufferDescriptor* codec_buffer) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s:%s INPARAM: bn_buffer[%s]", TAG, __func__,
               bn_buffer.ToString().c_str());

  RecorderIonBufferMap::iterator ion_buffer_iterator =
      ion_buffer_map_.find(bn_buffer.buffer_id);
  if (ion_buffer_iterator == ion_buffer_map_.end()) {
    QMMF_ERROR("%s: %s() no ion buffer for key[%d]", TAG, __func__,
               bn_buffer.buffer_id);
    return -EINVAL;
  }

  codec_buffer->data = ion_buffer_iterator->second.data;
  codec_buffer->fd = bn_buffer.buffer_id;
  codec_buffer->capacity = request_size_;
  codec_buffer->size = bn_buffer.size;
  codec_buffer->timestamp = bn_buffer.timestamp;
  codec_buffer->flag = 0;

  QMMF_VERBOSE("%s: %s() OUTPARAM: codec_buffer[%s]", TAG, __func__,
               codec_buffer->ToString().c_str());
  return 0;
}

int32_t RecorderIon::Export(const BufferDescriptor& codec_buffer,
                            BnBuffer* bn_buffer) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s:%s INPARAM: codec_buffer[%s]", TAG, __func__,
               codec_buffer.ToString().c_str());

  RecorderIonBufferMap::iterator ion_buffer_iterator =
      ion_buffer_map_.find(codec_buffer.fd);
  if (ion_buffer_iterator == ion_buffer_map_.end()) {
    QMMF_ERROR("%s: %s() no ion buffer for key[%d]", TAG, __func__,
               codec_buffer.fd);
    return -EINVAL;
  }

  bn_buffer->ion_fd = codec_buffer.fd;
  bn_buffer->size = codec_buffer.size;
  bn_buffer->timestamp = codec_buffer.timestamp;
  bn_buffer->width = -1;
  bn_buffer->height = -1;
  bn_buffer->buffer_id = codec_buffer.fd;
  bn_buffer->flag = codec_buffer.flag;
  bn_buffer->capacity = codec_buffer.capacity;

  QMMF_VERBOSE("%s: %s() OUTPARAM: bn_buffer[%s]", TAG, __func__,
               bn_buffer->ToString().c_str());
  return 0;
}

}; // namespace recorder
}; // namespace qmmf
