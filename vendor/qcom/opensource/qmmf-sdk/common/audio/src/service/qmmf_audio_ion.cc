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

#define TAG "AudioIon"

#include "common/audio/src/service/qmmf_audio_ion.h"

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

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/qmmf_log.h"

namespace qmmf {
namespace common {
namespace audio {

using ::std::map;

static const char* ion_filename = "/dev/ion";

AudioIon::AudioIon() : ion_device_(-1) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
}

AudioIon::~AudioIon() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  int result;

  if (ion_device_ == -1)
    return;

  for (AudioIonClientMap::value_type& client_value : client_map_) {
    result = Release(client_value.first);
    if (result < 0)
      QMMF_ERROR("%s: %s() unable to release buffers for client[%d]: %d", TAG,
                 __func__, client_value.first, result);
  }

  result = close(ion_device_);
  if (result < 0)
    QMMF_ERROR("%s: %s() error closing ion device[%d]: %d[%s]", TAG, __func__,
               ion_device_, errno, strerror(errno));
  QMMF_DEBUG("%s: %s() closed ion device[%d]", TAG, __func__, ion_device_);
}

int32_t AudioIon::Associate(const AudioHandle audio_handle,
                            AudioBuffer* buffer) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
               audio_handle);
  QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
               buffer->ToString().c_str());
  int result;

  if (ion_device_ == -1) {
    ion_device_ = open(ion_filename, O_RDONLY);
    if (ion_device_ < 0) {
      QMMF_ERROR("%s: %s() error opening ion device: %d[%s]", TAG, __func__,
                 errno, strerror(errno));
      return -ENODEV;
    }
    QMMF_DEBUG("%s: %s() opened ion device[%d]", TAG, __func__, ion_device_);
  }

  AudioIonClientMap::iterator client_iterator = client_map_.find(audio_handle);
  if (client_iterator != client_map_.end()) {
    AudioIonBufferMap::iterator buffer_iterator =
        client_iterator->second.find(buffer->buffer_id);
    if (buffer_iterator != client_iterator->second.end()) {
      result = close(buffer->ion_fd);
      if (result < 0) {
        QMMF_ERROR("%s: %s() error closing ion_fd[%d]: %d[%s]", TAG, __func__,
                   buffer->ion_fd, errno, strerror(errno));
        QMMF_ERROR("%s: %s() [CRITICAL] ion fd has leaked", TAG, __func__);
      }

      buffer->data = buffer_iterator->second.data;
      buffer->ion_fd = buffer_iterator->second.share_data.fd;
      return 0;
    }
  } else {
    client_map_.insert({audio_handle, AudioIonBufferMap()});
    client_iterator = client_map_.find(audio_handle);
  }

  AudioIonBuffer ion_buffer;

  ion_buffer.capacity = buffer->capacity;
  ion_buffer.share_data.handle = 0;
  ion_buffer.share_data.fd = buffer->ion_fd;

  result = ioctl(ion_device_, ION_IOC_IMPORT, &ion_buffer.share_data);
  if (result < 0) {
    QMMF_ERROR("%s: %s() ION_IOC_IMPORT ioctl command failed: %d[%s]", TAG,
               __func__, errno, strerror(errno));
    return errno;
  }

  ion_buffer.free_data.handle = ion_buffer.share_data.handle;

  ion_buffer.data = mmap(NULL, ion_buffer.capacity, PROT_READ | PROT_WRITE,
                         MAP_SHARED, ion_buffer.share_data.fd, 0);
  if (ion_buffer.data == MAP_FAILED) {
    QMMF_ERROR("%s: %s() unable to map buffer[%d]: %d[%s]", TAG, __func__,
               ion_buffer.share_data.fd, errno, strerror(errno));
    result = close(ion_buffer.share_data.fd);
    if (result < 0) {
      QMMF_ERROR("%s: %s() error closing mapping fd[%d]: %d[%s]", TAG, __func__,
                 ion_buffer.share_data.fd, errno, strerror(errno));
      QMMF_ERROR("%s: %s() [CRITICAL] ion fd has leaked", TAG, __func__);
    }
    return errno;
  }
  buffer->data = ion_buffer.data;

  QMMF_VERBOSE("%s: %s() mapped ion buffer[%s]", TAG, __func__,
               ion_buffer.ToString().c_str());

  client_iterator->second.insert({buffer->buffer_id, ion_buffer});

  return 0;
}

int32_t AudioIon::Release(const AudioHandle audio_handle) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: audio_handle[%d]", TAG, __func__,
               audio_handle);
  int result = 0;

  if (ion_device_ == -1) {
    QMMF_ERROR("%s: %s() ion device is not opened", TAG, __func__);
    return -ENODEV;
  }

  AudioIonClientMap::iterator client_iterator = client_map_.find(audio_handle);
  if (client_iterator == client_map_.end()) {
    QMMF_INFO("%s: %s() no ion buffers for audio client[%d]", TAG, __func__,
              audio_handle);
    return 0;
  }

  for (AudioIonBufferMap::value_type& buffer_value : client_iterator->second) {
    QMMF_VERBOSE("%s: %s() releasing ion buffer[%s]", TAG, __func__,
                 buffer_value.second.ToString().c_str());

    result = munmap(buffer_value.second.data, buffer_value.second.capacity);
    if (result < 0)
      QMMF_ERROR("%s: %s() unable to unmap buffer[%d]: %d[%s]", TAG, __func__,
                 buffer_value.second.share_data.fd, errno, strerror(errno));
    buffer_value.second.data = nullptr;

    result = close(buffer_value.second.share_data.fd);
    if (result < 0) {
      QMMF_ERROR("%s: %s() error closing shared fd[%d]: %d[%s]", TAG, __func__,
                 buffer_value.second.share_data.fd, errno, strerror(errno));
      return errno;
    }
    buffer_value.second.share_data.fd = -1;

    result = ioctl(ion_device_, ION_IOC_FREE, &buffer_value.second.free_data);
    if (result < 0) {
      QMMF_ERROR("%s: %s() ION_IOC_FREE ioctl command failed: %d[%s]", TAG,
                  __func__, errno, strerror(errno));
      QMMF_ERROR("%s: %s() [CRITICAL] ion memory has leaked", TAG, __func__);
    }
  }

  client_iterator->second.clear();
  QMMF_INFO("%s: %s() released all ion buffers", TAG, __func__);

  client_map_.erase(client_iterator->first);

  if (client_map_.empty()) {
    result = close(ion_device_);
    if (result < 0)
      QMMF_ERROR("%s: %s() error closing ion device[%d]: %d[%s]", TAG, __func__,
                 ion_device_, errno, strerror(errno));
    else
      QMMF_DEBUG("%s: %s() closed ion device[%d]", TAG, __func__, ion_device_);
    ion_device_ = -1;
  }

  return result;
}

}; // namespace audio
}; // namespace common
}; // namespace qmmf
