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
#ifdef TARGET_USES_GRALLOC1
#include <libgralloc1/gralloc_priv.h>
#include <grallocusage/GrallocUsageConversion.h>
#else
#include <qcom/display/gralloc_priv.h>
#endif

#include "qmmf_camera3_utils.h"
#include "qmmf_camera3_monitor.h"
#include "qmmf_camera3_stream.h"
#include "recorder/src/service/qmmf_recorder_common.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))

namespace qmmf {

namespace cameraadaptor {

Camera3Stream::Camera3Stream(int id, size_t maxSize,
                             const CameraStreamParameters &outputConfiguration,
                             mem_alloc_device device,
                             Camera3Monitor &monitor)
    : camera3_stream(),
      mem_alloc_interface_(nullptr),
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
#ifdef ANDROID_O_OR_ABOVE
  if (HAL_PIXEL_FORMAT_BLOB == outputConfiguration.format) {
    camera3_stream::data_space = HAL_DATASPACE_V0_JFIF;
  }
  camera3_stream::priv = nullptr;
#endif

  if ((HAL_PIXEL_FORMAT_BLOB == format) && (0 == maxSize)) {
    QMMF_ERROR("%s: blob with zero size\n", __func__);
    status_ = STATUS_ERROR;
  }

  if (NULL == device) {
    QMMF_ERROR("%s: Gralloc device is invalid!\n", __func__);
    status_ = STATUS_ERROR;
  }

  mem_alloc_interface_ = IMemAllocator::CreateMemAllocator(device);
  if (nullptr == mem_alloc_interface_) {
    QMMF_ERROR("%s: Gralloc Interface creation failed!\n", __func__);
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

  if (nullptr != mem_alloc_interface_) {
    delete mem_alloc_interface_;
    mem_alloc_interface_ = nullptr;
  }

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
    assert(nullptr != mem_alloc_interface_);
    for (uint32_t i = 0; i < gralloc_buffers_.size(); i++) {
      mem_alloc_interface_->FreeBuffer(gralloc_buffers_.keyAt(i));
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
                                        struct private_handle_t *priv_handle) {
  if (NULL == priv_handle) {
    QMMF_ERROR("%s: Invalid private handle!\n", __func__);
    return -EINVAL;
  }

  int alignedW, alignedH;
  auto ret = mem_alloc_interface_->GetStrideAndHeightFromHandle(priv_handle,
                                                                &alignedW,
                                                                &alignedH);
  if (0 != ret) {
    QMMF_ERROR("%s: Error in GetStrideAndHeightFromHandle() : %d\n", __func__,
               ret);
    return -EINVAL;
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
    case HAL_PIXEL_FORMAT_RAW8:
      info.format = BufferFormat::kRAW8;
      info.num_planes = 1;
      info.plane_info[0].width = width;
      info.plane_info[0].height = height;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
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
  PopulateMetaInfo(b.info, priv_handle);
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
    assert(nullptr != mem_alloc_interface_);
    // Blob buffers are expected to get allocated with width equal to blob
    // max size and height equal to 1.
    int32_t buf_width, buf_height;
    if (HAL_PIXEL_FORMAT_BLOB == camera3_stream::format) {
      buf_width = max_size_;
      buf_height = 1;
    } else {
      buf_width = camera3_stream::width;
      buf_height = camera3_stream::height;
    }
    res = mem_alloc_interface_->AllocBuffer(&handle,
                                            buf_width,
                                            buf_height,
                                            camera3_stream::format,
                                            camera3_stream::usage,
                                            &current_buffer_stride_);
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
    assert(nullptr != mem_alloc_interface_);
    for (uint32_t i = 0; i < gralloc_buffers_.size(); i++) {
      mem_alloc_interface_->FreeBuffer(gralloc_buffers_.keyAt(i));
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

  assert(nullptr != mem_alloc_interface_);
  for (uint32_t i = 0; i < gralloc_buffers_.size(); i++) {
    mem_alloc_interface_->FreeBuffer(gralloc_buffers_.keyAt(i));
  }
  gralloc_buffers_.clear();

  status_ = (status_ == STATUS_RECONFIG_ACTIVE) ? STATUS_CONFIG_ACTIVE
                                                : STATUS_INTIALIZED;
  return 0;
}

#ifdef TARGET_USES_GRALLOC1
Gralloc1Allocator::Gralloc1Allocator(mem_alloc_device gralloc1_device)
    : IMemAllocator(gralloc1_device) {
  assert(nullptr != gralloc1_device);

  CreateDescriptor  = reinterpret_cast<GRALLOC1_PFN_CREATE_DESCRIPTOR>(
      gralloc1_device->getFunction(gralloc1_device,
      GRALLOC1_FUNCTION_CREATE_DESCRIPTOR));

  DestroyDescriptor = reinterpret_cast<GRALLOC1_PFN_DESTROY_DESCRIPTOR>(
      gralloc1_device->getFunction(gralloc1_device,
      GRALLOC1_FUNCTION_DESTROY_DESCRIPTOR));

  SetDimensions     = reinterpret_cast<GRALLOC1_PFN_SET_DIMENSIONS>(
      gralloc1_device->getFunction(gralloc1_device,
      GRALLOC1_FUNCTION_SET_DIMENSIONS));

  SetFormat         = reinterpret_cast<GRALLOC1_PFN_SET_FORMAT>(
      gralloc1_device->getFunction(gralloc1_device,
      GRALLOC1_FUNCTION_SET_FORMAT));

  SetProducerUsage  = reinterpret_cast<GRALLOC1_PFN_SET_PRODUCER_USAGE>(
      gralloc1_device->getFunction(gralloc1_device,
      GRALLOC1_FUNCTION_SET_PRODUCER_USAGE));

  SetConsumerUsage  = reinterpret_cast<GRALLOC1_PFN_SET_CONSUMER_USAGE>(
      gralloc1_device->getFunction(gralloc1_device,
      GRALLOC1_FUNCTION_SET_CONSUMER_USAGE));

  Allocate          = reinterpret_cast<GRALLOC1_PFN_ALLOCATE>(
      gralloc1_device->getFunction(gralloc1_device,
      GRALLOC1_FUNCTION_ALLOCATE));

  GetStride         = reinterpret_cast<GRALLOC1_PFN_GET_STRIDE>(
      gralloc1_device->getFunction(gralloc1_device,
      GRALLOC1_FUNCTION_GET_STRIDE));

  Release           = reinterpret_cast<GRALLOC1_PFN_RELEASE>(
      gralloc1_device->getFunction(gralloc1_device,
      GRALLOC1_FUNCTION_RELEASE));

  Lock              = reinterpret_cast<GRALLOC1_PFN_LOCK>(
      gralloc1_device->getFunction(gralloc1_device,
      GRALLOC1_FUNCTION_LOCK));

  UnLock            = reinterpret_cast<GRALLOC1_PFN_UNLOCK>(
      gralloc1_device->getFunction(gralloc1_device,
      GRALLOC1_FUNCTION_UNLOCK));

  Perform           = reinterpret_cast<GRALLOC1_PFN_PERFORM>(
      gralloc1_device->getFunction(gralloc1_device,
      GRALLOC1_FUNCTION_PERFORM));

  if ((nullptr == CreateDescriptor) || (nullptr == DestroyDescriptor)
      || (nullptr == SetDimensions) || (nullptr == SetFormat)
      || (nullptr == SetProducerUsage) || (nullptr == SetConsumerUsage)
      || (nullptr == Allocate) || (nullptr == GetStride)
      || (nullptr == Release) || (nullptr == Lock)
      || (nullptr == UnLock) || (nullptr == Perform)) {
    QMMF_ERROR("%s: Gralloc device is invalid!\n", __func__);
  }
}

mem_alloc_error Gralloc1Allocator::AllocBuffer(buffer_handle_t *buf,
                                               int32_t width,
                                               int32_t height,
                                               int32_t format,
                                               int32_t usage,
                                               uint32_t *stride) {
  mem_alloc_device gralloc1_device = GetDevice();
  assert(nullptr != gralloc1_device);

  int32_t res = GRALLOC1_ERROR_NONE;
  gralloc1_buffer_descriptor_t buf_desc;
  uint64_t producer_flags = 0;
  uint64_t consumer_flags = 0;
  if (!width || !height) width = height = 1;

  android_convertGralloc0To1Usage(static_cast<int32_t>(usage),
                                  &producer_flags,
                                  &consumer_flags);
  QMMF_INFO("%s: width:%d height:%d format:%d p_flags:0x%x c_flags:0x%x\n",
            __func__, width, height, format,
            static_cast<uint32_t>(producer_flags),
            static_cast<uint32_t>(consumer_flags));

  res = CreateDescriptor(gralloc1_device, &buf_desc);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in CreateDescriptor\n", __func__);
    return GRALLOC1_ERROR_BAD_VALUE;
  }

  res = SetDimensions(gralloc1_device, buf_desc, width, height);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in SetDimensions\n", __func__);
    return GRALLOC1_ERROR_BAD_VALUE;
  }

  res = SetFormat(gralloc1_device, buf_desc, format);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in SetFormat\n", __func__);
    return GRALLOC1_ERROR_BAD_VALUE;
  }

  res = SetProducerUsage(gralloc1_device, buf_desc, producer_flags);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in SetProducerUsage\n", __func__);
    return GRALLOC1_ERROR_BAD_VALUE;
  }

  res = SetConsumerUsage(gralloc1_device, buf_desc, consumer_flags);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in SetConsumerUsage\n", __func__);
    return GRALLOC1_ERROR_BAD_VALUE;
  }

  res = Allocate(gralloc1_device, 1, &buf_desc, &buf[0]);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in Allocate\n", __func__);
    return GRALLOC1_ERROR_BAD_VALUE;
  }

  res = GetStride(gralloc1_device, *buf, stride);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in GetStride\n", __func__);
    return GRALLOC1_ERROR_BAD_VALUE;
  }

  res = DestroyDescriptor(gralloc1_device, buf_desc);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in DestroyDescriptor\n", __func__);
    return GRALLOC1_ERROR_BAD_VALUE;
  }

  return GRALLOC1_ERROR_NONE;
}

mem_alloc_error Gralloc1Allocator::FreeBuffer(buffer_handle_t buf) {
  mem_alloc_device gralloc1_device = GetDevice();
  assert(nullptr != gralloc1_device);

  if (nullptr != buf) {
    int32_t res  = Release(gralloc1_device, buf);
    if (GRALLOC1_ERROR_NONE != res) {
      QMMF_ERROR("%s: Error in Release\n", __func__);
      return GRALLOC1_ERROR_BAD_VALUE;
    }
    buf = nullptr;
  }
  return GRALLOC1_ERROR_NONE;
}

mem_alloc_error Gralloc1Allocator::GetStrideAndHeightFromHandle(
       struct private_handle_t* const priv_handle,
       int32_t* stride,
       int32_t* height) {
  mem_alloc_device gralloc1_device = GetDevice();
  assert(nullptr != gralloc1_device);

  int32_t res = Perform(gralloc1_device,
      GRALLOC_MODULE_PERFORM_GET_CUSTOM_STRIDE_AND_HEIGHT_FROM_HANDLE,
      priv_handle, stride, height);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in Perform\n", __func__);
    return GRALLOC1_ERROR_BAD_VALUE;
  }

  return GRALLOC1_ERROR_NONE;
}

IMemAllocator* IMemAllocator::CreateMemAllocator(mem_alloc_device device) {
  return (new Gralloc1Allocator(device));
};
#else
GrallocAllocator::GrallocAllocator(mem_alloc_device gralloc_device)
    : IMemAllocator(gralloc_device) {
  assert(nullptr != gralloc_device);
}


mem_alloc_error GrallocAllocator::AllocBuffer(buffer_handle_t *buf,
                                              int32_t width,
                                              int32_t height,
                                              int32_t format,
                                              int32_t usage,
                                              uint32_t *stride) {
  mem_alloc_device gralloc_device = GetDevice();
  assert(nullptr != gralloc_device);

  if (!width || !height) width = height = 1;

  // Filter out any usage bits that should not be passed
  // to the Gralloc module.
  usage &= GRALLOC_USAGE_ALLOC_MASK;

  int32_t buf_stride;
  int32_t res = gralloc_device->alloc(gralloc_device, width, height,
                              format, usage, buf, &buf_stride);
  if (0 != res) {
    QMMF_ERROR("%s: Unable to allocate Gralloc buffer: %d\n", __func__, res);
    return -EINVAL;
  }
  *stride = static_cast<uint32_t>(buf_stride);

  return 0;
}

mem_alloc_error GrallocAllocator::FreeBuffer(buffer_handle_t buf) {
  mem_alloc_device gralloc_device = GetDevice();
  assert(nullptr != gralloc_device);

  if (nullptr != buf) {
    int32_t res  = gralloc_device->free(gralloc_device, buf);
    if (0 != res) {
      QMMF_ERROR("%s: Error in Free\n", __func__);
      return -EINVAL;
    }
    buf = nullptr;
  }
  return 0;
}

mem_alloc_error GrallocAllocator::GetStrideAndHeightFromHandle(
       struct private_handle_t* const priv_handle,
       int32_t* stride,
       int32_t* height) {
  mem_alloc_device gralloc_device = GetDevice();
  assert(nullptr != gralloc_device);

  gralloc_module_t const *mapper = reinterpret_cast<gralloc_module_t const *>(
      gralloc_device->common.module);
  int32_t res = mapper->perform(mapper,
        GRALLOC_MODULE_PERFORM_GET_CUSTOM_STRIDE_AND_HEIGHT_FROM_HANDLE,
        priv_handle, stride, height);
  if (0 != res) {
    QMMF_ERROR("%s: Error in querying stride & height: %d\n", __func__, res);
    return -EINVAL;
  }

  return 0;
}

IMemAllocator* IMemAllocator::CreateMemAllocator(mem_alloc_device device) {
  return (new GrallocAllocator(device));
};
#endif  // TARGET_USES_GRALLOC1
}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here
