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

#define TAG "RecorderClientIon"

#include "recorder/src/client/qmmf_recorder_client_ion.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <map>

#include <linux/msm_ion.h>

#include "common/qmmf_log.h"
#include "include/qmmf-sdk/qmmf_recorder_params.h"
#include "recorder/src/client/qmmf_recorder_params_internal.h"
#include "recorder/src/client/qmmf_recorder_service_intf.h"

namespace qmmf {
namespace recorder {

using ::std::map;

static const char* ion_filename = "/dev/ion";

RecorderClientIon::RecorderClientIon() : ion_device_(-1) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  // open ion device
  ion_device_ = open(ion_filename, O_RDONLY);
  if (ion_device_ < 0)
    QMMF_ERROR("%s: %s() error opening ion device: %d[%s]", TAG, __func__,
               errno, strerror(errno));
}

RecorderClientIon::~RecorderClientIon() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  int32_t result;

  if (ion_device_ == -1)
    QMMF_WARN("%s: %s() ion device is not opened", TAG, __func__);

  // release all ion buffers
  for (auto& client_map : buffer_map_) {
    result = Release(client_map.first);
    if (result < 0) {
      QMMF_ERROR("%s: %s() unable to release buffers for client[%d]: %d", TAG,
          __func__, client_map.first, result);
    }
  }
  // close ion device
  result = close(ion_device_);
  if (result < 0) {
    QMMF_ERROR("%s: %s() error closing ion device[%d]: %d[%s]", TAG, __func__,
        ion_device_, errno, strerror(errno));
  }
}

int32_t RecorderClientIon::Associate(uint32_t track_id,
                                     const BnBuffer& bn_buffer,
                                     BufferDescriptor* buffer) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: track_id[%d]", TAG, __func__, track_id);
  QMMF_VERBOSE("%s: %s() INPARAM: bn_buffer[%s]", TAG, __func__,
               bn_buffer.ToString().c_str());
  int result;

  if (ion_device_ == -1) {
    QMMF_ERROR("%s: %s() ion device is not opened", TAG, __func__);
    return -ENODEV;
  }

  auto client_map = buffer_map_.find(track_id);
  if (client_map != buffer_map_.end()) {
    auto ion_buffer = client_map->second.find(bn_buffer.buffer_id);
    if (ion_buffer != client_map->second.end()) {
      // found the ion buffer
      result = close(bn_buffer.ion_fd);
      if (result < 0) {
        QMMF_ERROR("%s: %s() error closing ion_fd[%d]: %d[%s]", TAG, __func__,
                   bn_buffer.ion_fd, errno, strerror(errno));
        QMMF_ERROR("%s: %s() [CRITICAL] ion fd has leaked", TAG, __func__);
      }

      buffer->data = ion_buffer->second.data;
      buffer->size = bn_buffer.size;
      buffer->timestamp = bn_buffer.timestamp;
      buffer->flag = bn_buffer.flag;
      buffer->buf_id = bn_buffer.buffer_id;
      buffer->capacity = bn_buffer.capacity;
      buffer->fd = ion_buffer->second.share_data.fd;
      QMMF_VERBOSE("%s: %s() OUTPARAM: buffer[%s]", TAG, __func__,
                   buffer->ToString().c_str());
      return 0;
    }
  } else {
    // create new client map
    buffer_map_.insert({track_id, RecorderClientIonBufferMap()});
    client_map = buffer_map_.find(track_id);
  }

  RecorderClientIonBuffer ion_buffer;

  ion_buffer.capacity = bn_buffer.capacity;
  ion_buffer.share_data.handle = 0;
  ion_buffer.share_data.fd = bn_buffer.ion_fd;

  // import ion handle from shared fd
  result = ioctl(ion_device_, ION_IOC_IMPORT, &ion_buffer.share_data);
  if (result < 0) {
    QMMF_ERROR("%s: %s() ION_IOC_IMPORT ioctl command failed: %d[%s]", TAG,
               __func__, errno, strerror(errno));
    return errno;
  }

  ion_buffer.free_data.handle = ion_buffer.share_data.handle;

  // map buffers into address space
  ion_buffer.data = mmap(NULL, ion_buffer.capacity, PROT_READ | PROT_WRITE,
                         MAP_SHARED, ion_buffer.share_data.fd, 0);
  if (ion_buffer.data == MAP_FAILED) {
    QMMF_ERROR("%s: %s() unable to map buffer[%d]: %d[%s]", TAG, __func__,
               ion_buffer.share_data.fd, errno, strerror(errno));
    return errno;
  }
  buffer->data = ion_buffer.data;
  buffer->size = bn_buffer.size;
  buffer->timestamp = bn_buffer.timestamp;
  buffer->flag = bn_buffer.flag;
  buffer->buf_id = bn_buffer.buffer_id;
  buffer->capacity = bn_buffer.capacity;
  buffer->fd = bn_buffer.ion_fd;

  QMMF_VERBOSE("%s: %s() mapped ion buffer[%s]", TAG, __func__,
               ion_buffer.ToString().c_str());

  // save ion buffer
  client_map->second.insert({bn_buffer.buffer_id, ion_buffer});

  QMMF_VERBOSE("%s: %s() OUTPARAM: buffer[%s]", TAG, __func__,
               buffer->ToString().c_str());
  return 0;
}

int32_t RecorderClientIon::Release(uint32_t track_id) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: track_id[%u]", TAG, __func__, track_id);

  auto client_map = buffer_map_.find(track_id);
  if (client_map == buffer_map_.end()) {
    QMMF_INFO("%s: %s() no ion buffers for track_id", TAG, __func__);
    return 0;
  }

  for (auto& buffer : client_map->second) {
    int result;

    QMMF_VERBOSE("%s: %s() releasing ion buffer[%s]", TAG, __func__,
                 buffer.second.ToString().c_str());

    // unmap buffer from address space
    result = munmap(buffer.second.data, buffer.second.capacity);
    if (result < 0)
      QMMF_ERROR("%s: %s() unable to unmap buffer[%d]: %d[%s]", TAG, __func__,
                 buffer.second.share_data.fd, errno, strerror(errno));
    buffer.second.data = nullptr;

    // close fd
    result = close(buffer.second.share_data.fd);
    if (result < 0) {
      QMMF_ERROR("%s: %s() error closing shared fd[%d]: %d[%s]", TAG, __func__,
                 buffer.second.share_data.fd, errno, strerror(errno));
      return errno;
    }
    buffer.second.share_data.fd = -1;

    // free ion buffer
    result = ioctl(ion_device_, ION_IOC_FREE, &buffer.second.free_data);
    if (result < 0) {
      QMMF_ERROR("%s: %s() ION_IOC_FREE ioctl command failed: %d[%s]", TAG,
                 __func__, errno, strerror(errno));
      return errno;
    }
  }

  client_map->second.clear();
  buffer_map_.erase(client_map->first);
  QMMF_INFO("%s: %s() released all ion buffers", TAG, __func__);

  return 0;
}

}; // namespace recorder
}; // namespace qmmf
