/*
 * Copyright (c) 2016-2017 The Linux Foundation. All rights reserved.
 * Not a Contribution.
 */

/*
 * Copyright (C) 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <libgralloc/gralloc_priv.h>

#include "qmmf_camera3_utils.h"
#include "qmmf_camera3_monitor.h"
#include "qmmf_camera3_stream.h"
#include "recorder/src/service/qmmf_recorder_common.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))

namespace qmmf {

namespace cameraadaptor {

Camera3Stream::Camera3Stream(int id, size_t maxSize,
                             const CameraStreamParameters &outputConfiguration,
                             alloc_device_t *grallocDevice,
                             Camera3Monitor &monitor)
    : camera3_stream(),
      gralloc_device_(grallocDevice),
      current_buffer_stride_(0),
      id_(id),
      max_size_(maxSize),
      status_(STATUS_INTIALIZED),
      total_buffer_count_(0),
      pending_buffer_count_(0),
      callbacks_(outputConfiguration.cb),
      old_usage_(0),
      client_usage_(outputConfiguration.grallocFlags),
      old_max_buffers_(0),
      client_max_buffers_(outputConfiguration.bufferCount),
      gralloc_slots_(NULL),
      gralloc_buffer_allocated_(0),
      monitor_(monitor),
      monitor_id_(Camera3Monitor::INVALID_ID),
      is_stream_active_(false),
      prepared_buffers_count_(0) {
  camera3_stream::stream_type = CAMERA3_STREAM_OUTPUT;
  camera3_stream::width = outputConfiguration.width;
  camera3_stream::height = outputConfiguration.height;
  camera3_stream::format = outputConfiguration.format;
  camera3_stream::data_space = outputConfiguration.data_space;
  camera3_stream::rotation = outputConfiguration.rotation;
  camera3_stream::usage = outputConfiguration.grallocFlags;
  camera3_stream::max_buffers = outputConfiguration.bufferCount;
  camera3_stream::priv = NULL;

  if ((HAL_PIXEL_FORMAT_BLOB == format) && (0 == maxSize)) {
    QMMF_ERROR("%s: blob with zero size\n", __func__);
    status_ = STATUS_ERROR;
  }

  if (NULL == gralloc_device_) {
    QMMF_ERROR("%s: Gralloc device is invalid!\n", __func__);
    status_ = STATUS_ERROR;
  }

  pthread_mutex_init(&lock_, NULL);
  pthread_cond_init(&output_buffer_returned_signal_, NULL);
}

Camera3Stream::~Camera3Stream() {
  if (0 <= monitor_id_) {
    monitor_.ReleaseMonitor(monitor_id_);
    monitor_id_ = Camera3Monitor::INVALID_ID;
  }

  CloseLocked();

  pthread_mutex_destroy(&lock_);
  pthread_cond_destroy(&output_buffer_returned_signal_);
  if (NULL != gralloc_slots_) {
    delete[] gralloc_slots_;
  }
}

camera3_stream *Camera3Stream::BeginConfigure() {
  pthread_mutex_lock(&lock_);

  switch (status_) {
    case STATUS_ERROR:
      QMMF_ERROR("%s: Error status\n", __func__);
      goto exit;
    case STATUS_INTIALIZED:
      break;
    case STATUS_CONFIG_ACTIVE:
    case STATUS_RECONFIG_ACTIVE:
      goto done;
    case STATUS_CONFIGURED:
      break;
    default:
      QMMF_ERROR("%s: Unknown status %d", __func__, status_);
      goto exit;
  }

  camera3_stream::usage = client_usage_;
  camera3_stream::max_buffers = client_max_buffers_;

  if (monitor_id_ != Camera3Monitor::INVALID_ID) {
    monitor_.ReleaseMonitor(monitor_id_);
    monitor_id_ = Camera3Monitor::INVALID_ID;
  }

  if (status_ == STATUS_INTIALIZED) {
    status_ = STATUS_CONFIG_ACTIVE;
  } else {
    if (status_ != STATUS_CONFIGURED) {
      QMMF_ERROR("%s: Invalid state: 0x%x \n", __func__, status_);
      goto exit;
    }
    status_ = STATUS_RECONFIG_ACTIVE;
  }

done:
  pthread_mutex_unlock(&lock_);

  return this;

exit:
  pthread_mutex_unlock(&lock_);
  return NULL;
}

bool Camera3Stream::IsConfigureActive() {
  pthread_mutex_lock(&lock_);
  bool ret =
      (status_ == STATUS_CONFIG_ACTIVE) || (status_ == STATUS_RECONFIG_ACTIVE);
  pthread_mutex_unlock(&lock_);
  return ret;
}

int32_t Camera3Stream::EndConfigure() {
  int32_t res;
  pthread_mutex_lock(&lock_);
  switch (status_) {
    case STATUS_ERROR:
      QMMF_ERROR("%s: Error status\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATUS_CONFIG_ACTIVE:
    case STATUS_RECONFIG_ACTIVE:
      break;
    case STATUS_INTIALIZED:
    case STATUS_CONFIGURED:
      QMMF_ERROR("%s: Configuration didn't start before!\n", __func__);
      res = -ENOSYS;
      goto exit;
    default:
      QMMF_ERROR("%s: Unknown status", __func__);
      res = -ENOSYS;
      goto exit;
  }

  monitor_id_ = monitor_.AcquireMonitor();
  if (0 > monitor_id_) {
    QMMF_ERROR("%s: Unable to acquire monitor: %d\n", __func__, monitor_id_);
    res = monitor_id_;
    goto exit;
  }

  if (status_ == STATUS_RECONFIG_ACTIVE && old_usage_ == camera3_stream::usage &&
      old_max_buffers_ == camera3_stream::max_buffers) {
    status_ = STATUS_CONFIGURED;
    res = 0;
    goto exit;
  }

  res = ConfigureLocked();
  if (0 != res) {
    QMMF_ERROR("%s: Unable to configure stream %d\n", __func__, id_);
    status_ = STATUS_ERROR;
    goto exit;
  }

  status_ = STATUS_CONFIGURED;
  old_usage_ = camera3_stream::usage;
  old_max_buffers_ = camera3_stream::max_buffers;

exit:
  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3Stream::AbortConfigure() {
  int32_t res;
  pthread_mutex_lock(&lock_);
  switch (status_) {
    case STATUS_ERROR:
      QMMF_ERROR("%s: Error status\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATUS_CONFIG_ACTIVE:
    case STATUS_RECONFIG_ACTIVE:
      break;
    case STATUS_INTIALIZED:
    case STATUS_CONFIGURED:
      QMMF_ERROR("%s: Cannot abort configure that is not started\n", __func__);
      res = -ENOSYS;
      goto exit;
    default:
      QMMF_ERROR("%s: Unknown status\n", __func__);
      res = -ENOSYS;
      goto exit;
  }

  camera3_stream::usage = old_usage_;
  camera3_stream::max_buffers = old_max_buffers_;

  status_ = (status_ == STATUS_RECONFIG_ACTIVE) ? STATUS_CONFIGURED
                                                : STATUS_INTIALIZED;
  pthread_mutex_unlock(&lock_);
  return 0;

exit:

  pthread_mutex_unlock(&lock_);
  return res;
}

int32_t Camera3Stream::BeginPrepare() {
  int32_t res = 0;

  pthread_mutex_lock(&lock_);

  if (STATUS_CONFIGURED != status_) {
    QMMF_ERROR("%s: Stream %d: Cannot prepare unconfigured stream with "
        "status: %d\n", __func__, id_, status_);
      res = -ENOSYS;
      goto exit;
  }

  if (is_stream_active_) {
    QMMF_ERROR("%s: Stream %d: Cannot prepare already active stream\n",
               __func__, id_);
    res = -ENOSYS;
    goto exit;
  }

  if (0 < GetPendingBufferCountLocked()) {
    QMMF_ERROR("%s: Stream %d: Cannot prepare stream with pending buffers\n",
               __func__, id_);
    res = -ENOSYS;
    goto exit;
  }

  prepared_buffers_count_ = 0;
  status_ = STATUS_PREPARE_ACTIVE;
  res = -ENODATA;

exit:

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3Stream::PrepareBuffer() {
  int32_t res = 0;

  pthread_mutex_lock(&lock_);

  if (STATUS_PREPARE_ACTIVE != status_) {
    QMMF_ERROR("%s: Stream %d: Invalid status: %d\n", __func__, id_, status_);
    res = -ENOSYS;
    goto exit;
  }

  res = GetBufferLocked();
  if (0 != res) {
    QMMF_ERROR("%s: Stream %d: Failed to pre-allocate buffer %d", __func__,
               id_, prepared_buffers_count_);
    res = -ENODEV;
    goto exit;
  }

  prepared_buffers_count_++;

  if (prepared_buffers_count_ < total_buffer_count_) {
    res = -ENODATA;
    goto exit;
  }

  res = EndPrepareLocked();

exit:

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3Stream::EndPrepare() {
  pthread_mutex_lock(&lock_);

  int32_t res = EndPrepareLocked();

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3Stream::EndPrepareLocked() {
  if (STATUS_PREPARE_ACTIVE != status_) {
    QMMF_ERROR("%s: Stream %d: Cannot abort stream prepare with wrong"
        "status: %d\n", __func__, id_, status_);
    return -ENOSYS;
  }

  prepared_buffers_count_ = 0;
  status_ = STATUS_CONFIGURED;

  return 0;
}

bool Camera3Stream::IsPrepareActive() {
  pthread_mutex_lock(&lock_);

  bool res = (STATUS_PREPARE_ACTIVE == status_);

  pthread_mutex_unlock(&lock_);

  return res;
}

bool Camera3Stream::IsStreamActive() {
  pthread_mutex_lock(&lock_);

  bool res = is_stream_active_;

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3Stream::TearDown() {
  int32_t res = 0;

  pthread_mutex_lock(&lock_);

  if (status_ != STATUS_CONFIGURED) {
    QMMF_ERROR(
        "%s: Stream %d: Cannot be torn down when stream"
        "is still un-configured: %d\n",
        __func__, id_, status_);
    res = -ENOSYS;
    goto exit;
  }

  if (0 < GetPendingBufferCountLocked()) {
    QMMF_ERROR(
        "%s: Stream %d: Cannot be torn down while buffers are still pending\n",
        __func__, id_);
    res = -ENOSYS;
    goto exit;
  }

  if (0 < gralloc_buffer_allocated_) {
    for (uint32_t i = 0; i < gralloc_buffers_.size(); i++) {
      FreeGrallocBuffer(gralloc_buffers_.keyAt(i));
    }
    gralloc_buffers_.clear();
    gralloc_buffer_allocated_ = 0;
  }

  for (uint32_t i = 0; i < total_buffer_count_; i++) {
    gralloc_slots_[i] = NULL;
  }

  is_stream_active_ = false;

exit:

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3Stream::GetBuffer(camera3_stream_buffer *buffer) {
  int32_t res = 0;

  pthread_mutex_lock(&lock_);

  if (status_ != STATUS_CONFIGURED) {
    QMMF_ERROR(
        "%s: Stream %d: Can't retrieve buffer when stream"
        "is not configured%d\n",
        __func__, id_, status_);
    res = -ENOSYS;
    goto exit;
  }

  if (GetPendingBufferCountLocked() == total_buffer_count_) {
      QMMF_DEBUG(
        "%s: Already retrieved maximum buffers (%d), waiting on a"
        "free one\n",
        __func__, total_buffer_count_);
    res = cond_wait_relative(&output_buffer_returned_signal_, &lock_,
                             BUFFER_WAIT_TIMEOUT);
    if (res != 0) {
      if (-ETIMEDOUT == res) {
        QMMF_ERROR("%s: wait for output buffer return timed out\n", __func__);
      }
      goto exit;
    }
  }

  res = GetBufferLocked(buffer);

exit:

  pthread_mutex_unlock(&lock_);
  return res;
}

int32_t Camera3Stream::PopulateMetaInfo(CameraBufferMetaData &info,
                                        struct private_handle_t *priv_handle,
                                        alloc_device_t *gralloc_device) {
  int alignedW, alignedH;
  if (NULL == gralloc_device) {
    QMMF_ERROR("%s: Invalid gralloc device!\n", __func__);
    return -EINVAL;
  }

  if (NULL == priv_handle) {
    QMMF_ERROR("%s: Invalid private handle!\n", __func__);
    return -EINVAL;
  }

  gralloc_module_t const *mapper = reinterpret_cast<gralloc_module_t const *>(
          gralloc_device_->common.module);
  auto ret = mapper->perform(
      mapper,
      GRALLOC_MODULE_PERFORM_GET_CUSTOM_STRIDE_AND_HEIGHT_FROM_HANDLE,
      priv_handle, &alignedW, &alignedH);
  if (0 != ret) {
    QMMF_ERROR("%s: Unable to query stride&scanline: %d\n", __func__, ret);
    return ret;
  }

  switch (priv_handle->format) {
    case HAL_PIXEL_FORMAT_BLOB:
      info.format = BufferFormat::kBLOB;
      info.num_planes = 1;
      info.plane_info[0].width = max_size_;
      info.plane_info[0].height = 1;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
      info.format = BufferFormat::kNV12;
      info.num_planes = 2;
      info.plane_info[0].width = width;
      info.plane_info[0].height = height;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      info.plane_info[1].width = width;
      info.plane_info[1].height = height/2;
      info.plane_info[1].stride = alignedW;
      info.plane_info[1].scanline = alignedH/2;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
      info.format = BufferFormat::kNV12UBWC;
      info.num_planes = 2;
      info.plane_info[0].width = width;
      info.plane_info[0].height = height;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      info.plane_info[1].width = width;
      info.plane_info[1].height = height/2;
      info.plane_info[1].stride = alignedW;
      info.plane_info[1].scanline = alignedH/2;
      break;
    case HAL_PIXEL_FORMAT_NV21_ZSL:
      info.format = BufferFormat::kNV21;
      info.num_planes = 2;
      info.plane_info[0].width = width;
      info.plane_info[0].height = height;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      info.plane_info[1].width = width;
      info.plane_info[1].height = height/2;
      info.plane_info[1].stride = alignedW;
      info.plane_info[1].scanline = alignedH/2;
      break;
    case HAL_PIXEL_FORMAT_RAW10:
      info.format = BufferFormat::kRAW10;
      info.num_planes = 1;
      info.plane_info[0].width = width;
      info.plane_info[0].height = height;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      break;
    case HAL_PIXEL_FORMAT_RAW12:
      info.format = BufferFormat::kRAW12;
      info.num_planes = 1;
      info.plane_info[0].width = width;
      info.plane_info[0].height = height;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      break;
    case HAL_PIXEL_FORMAT_RAW16:
      info.format = BufferFormat::kRAW16;
      info.num_planes = 1;
      info.plane_info[0].width = width;
      info.plane_info[0].height = height;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      break;
    default:
      QMMF_ERROR("%s: Unsupported format: %d\n", __func__,
                 priv_handle->format);
      return -ENOENT;
  }

  return 0;
}

void Camera3Stream::ReturnBufferToClient(const camera3_stream_buffer &buffer,
                                         int64_t timestamp,
                                         int64_t frame_number) {
  struct private_handle_t *priv_handle = (struct private_handle_t *)
      *buffer.buffer;
  assert(nullptr != callbacks_);

  pthread_mutex_lock(&lock_);

  StreamBuffer b;
  memset(&b, 0, sizeof(b));
  b.timestamp = timestamp;
  b.frame_number = frame_number;
  b.stream_id = id_;
  b.data_space = data_space;
  b.handle = *buffer.buffer;
  b.fd = priv_handle->fd;
  b.size = priv_handle->size;
  PopulateMetaInfo(b.info, priv_handle, gralloc_device_);
  is_stream_active_ = true;

  pthread_mutex_unlock(&lock_);

  if (CAMERA3_BUFFER_STATUS_OK == buffer.status) {
    callbacks_(b);
  } else {
    ReturnBuffer(b);
  }
}

int32_t Camera3Stream::ReturnBuffer(const StreamBuffer &buffer) {
  pthread_mutex_lock(&lock_);

  int32_t res = ReturnBufferLocked(buffer);
  if (res == 0) {
    pthread_cond_signal(&output_buffer_returned_signal_);
  }

  pthread_mutex_unlock(&lock_);
  return res;
}

int32_t Camera3Stream::Close() {
  pthread_mutex_lock(&lock_);
  int32_t res = CloseLocked();

  if (res == -ENOTCONN) {
    res = 0;
  }

  pthread_mutex_unlock(&lock_);
  return res;
}

int32_t Camera3Stream::ReturnBufferLocked(const StreamBuffer &buffer) {
  if (status_ == STATUS_INTIALIZED) {
    QMMF_ERROR(
        "%s: Stream %d: Can't return buffer when we only "
        "got initialized %d\n",
        __func__, id_, status_);
    return -ENOSYS;
  }

  if (pending_buffer_count_ == 0) {
    QMMF_ERROR("%s: Stream %d: Not expecting any buffers!\n", __func__, id_);
    return -ENOSYS;
  }

  int32_t idx = gralloc_buffers_.indexOfKey(buffer.handle);
  if (-ENOENT == idx) {
    QMMF_ERROR(
        "%s: Buffer %p returned that wasn't allocated by this"
        " stream!\n",
        __func__, buffer.handle);
    return -EINVAL;
  } else {
    gralloc_buffers_.replaceValueFor(buffer.handle, true);
  }

  pending_buffer_count_--;

  if (pending_buffer_count_ == 0 && status_ != STATUS_CONFIG_ACTIVE &&
      status_ != STATUS_RECONFIG_ACTIVE) {
    monitor_.ChangeStateToIdle(monitor_id_);
  }

  return 0;
}

int32_t Camera3Stream::GetBufferLocked(camera3_stream_buffer *streamBuffer) {
  status_t res;
  int32_t idx = -1;
  if ((status_ != STATUS_CONFIGURED) && (status_ != STATUS_CONFIG_ACTIVE) &&
      (status_ != STATUS_RECONFIG_ACTIVE) &&
      (status_ != STATUS_PREPARE_ACTIVE)) {
    QMMF_ERROR(
        "%s: Stream %d: Can't get buffers before being"
        " configured  or preparing %d\n",
        __func__, id_, status_);
    return -ENOSYS;
  }

  buffer_handle_t handle = NULL;
  //Only pre-allocate buffers in case no valid streamBuffer
  //is passed as an argument.
  if (NULL != streamBuffer) {
    for (uint32_t i = 0; i < gralloc_buffers_.size(); i++) {
      if (gralloc_buffers_.valueAt(i)) {
        handle = gralloc_buffers_.keyAt(i);
        gralloc_buffers_.replaceValueAt(i, false);
        break;
      }
    }
  }

  if (NULL != handle) {
    for (uint32_t i = 0; i < gralloc_buffer_allocated_; i++) {
      if (gralloc_slots_[i] == handle) {
        idx = i;
        break;
      }
    }
  } else if ((NULL == handle) &&
             (gralloc_buffer_allocated_ < total_buffer_count_)) {
    res = AllocGrallocBuffer(&handle);
    if (0 != res) {
      return res;
    }
    idx = gralloc_buffer_allocated_;
    gralloc_slots_[idx] = handle;
    gralloc_buffers_.add(gralloc_slots_[idx], (NULL == streamBuffer));
    gralloc_buffer_allocated_++;
  }

  if ((NULL == handle) || (0 > idx)) {
    QMMF_ERROR("%s: Unable to allocate or find a free buffer!\n", __func__);
    return -ENOSYS;
  }

  if (NULL != streamBuffer) {
    streamBuffer->stream = this;
    streamBuffer->acquire_fence = -1;
    streamBuffer->release_fence = -1;
    streamBuffer->status = CAMERA3_BUFFER_STATUS_OK;
    streamBuffer->buffer = &gralloc_slots_[idx];

    if (pending_buffer_count_ == 0 && status_ != STATUS_CONFIG_ACTIVE &&
        status_ != STATUS_RECONFIG_ACTIVE) {
      monitor_.ChangeStateToActive(monitor_id_);
    }

    pending_buffer_count_++;
  }

  return 0;
}

int32_t Camera3Stream::ConfigureLocked() {
  int32_t res;

  switch (status_) {
    case STATUS_RECONFIG_ACTIVE:
      res = CloseLocked();
      if (0 != res) {
        return res;
      }
      break;
    case STATUS_CONFIG_ACTIVE:
      break;
    default:
      QMMF_ERROR("%s: Bad status: %d\n", __func__, status_);
      return -ENOSYS;
  }

  total_buffer_count_ = MAX(client_max_buffers_, camera3_stream::max_buffers);
  pending_buffer_count_ = 0;
  gralloc_buffer_allocated_ = 0;
  is_stream_active_ = false;
  if (NULL != gralloc_slots_) {
    delete[] gralloc_slots_;
  }

  if (!gralloc_buffers_.isEmpty()) {
    for (uint32_t i = 0; i < gralloc_buffers_.size(); i++) {
      FreeGrallocBuffer(gralloc_buffers_.keyAt(i));
    }
    gralloc_buffers_.clear();
  }

  gralloc_slots_ = new buffer_handle_t[total_buffer_count_];
  if (NULL == gralloc_slots_) {
    QMMF_ERROR("%s: Unable to allocate buffer handles!\n", __func__);
    status_ = STATUS_ERROR;
    return -ENOMEM;
  }

  return 0;
}

int32_t Camera3Stream::CloseLocked() {
  switch (status_) {
    case STATUS_RECONFIG_ACTIVE:
    case STATUS_CONFIGURED:
      break;
    default:
      QMMF_ERROR("%s: Stream %d is already closed!\n", __func__, id_);
      return -ENOTCONN;
  }

  if (pending_buffer_count_ > 0) {
    QMMF_ERROR("%s: Can't disconnect with %zu buffers still dequeued!\n",
               __func__, pending_buffer_count_);
    for (uint32_t i = 0; i < gralloc_buffers_.size(); i++) {
      QMMF_ERROR("%s: buffer[%d] = %p status: %d\n", __func__, i,
                 gralloc_buffers_.keyAt(i), gralloc_buffers_.valueAt(i));
    }
    return -ENOSYS;
  }

  for (uint32_t i = 0; i < gralloc_buffers_.size(); i++) {
    FreeGrallocBuffer(gralloc_buffers_.keyAt(i));
  }
  gralloc_buffers_.clear();

  status_ = (status_ == STATUS_RECONFIG_ACTIVE) ? STATUS_CONFIG_ACTIVE
                                                : STATUS_INTIALIZED;
  return 0;
}

int32_t Camera3Stream::AllocGrallocBuffer(buffer_handle_t *buf) {
  int32_t res = 0;

  assert(NULL != gralloc_device_);

  if (!width || !height) width = height = 1;

  // Filter out any usage bits that should not be passed to the gralloc
  // module.
  usage &= GRALLOC_USAGE_ALLOC_MASK;

  int outStride = 0;
  if (0 < max_size_) {
    // Blob buffers are expected to get allocated with width equal to blob
    // max size and height equal to 1.
    res = gralloc_device_->alloc(gralloc_device_, static_cast<int>(max_size_),
                                static_cast<int>(1), format,
                                static_cast<int>(usage), buf, &outStride);
  } else {
    res = gralloc_device_->alloc(gralloc_device_, static_cast<int>(width),
                                static_cast<int>(height), format,
                                static_cast<int>(usage), buf, &outStride);
  }
  if (0 != res) {
    QMMF_ERROR("%s: Unable to allocate gralloc buffer: %d\n", __func__, res);
    return res;
  }
  current_buffer_stride_ = static_cast<uint32_t>(outStride);

  return res;
}

int32_t Camera3Stream::FreeGrallocBuffer(buffer_handle_t buf) {
  assert(NULL != gralloc_device_);

  return gralloc_device_->free(gralloc_device_, buf);
}

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here
