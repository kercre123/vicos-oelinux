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

#define TAG "RecorderPostProcMemPool"

#include <dlfcn.h>
#include <hardware/hardware.h>

#include "../interface/qmmf_postproc.h"
#include "../factory/qmmf_postproc_factory.h"

#include "qmmf_postproc_memory_pool.h"

namespace qmmf {

namespace recorder {

MemPool::MemPool()
    : buffers_allocated_(0),
      pending_buffer_count_(0) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  QMMF_INFO("%s:%s: Exit (%p)", TAG, __func__, this);
}

MemPool::~MemPool() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  if (!gralloc_buffers_.isEmpty()) {
    for (uint32_t i = 0; i < gralloc_buffers_.size(); i++) {
      FreeGrallocBuffer(gralloc_buffers_.keyAt(i));
    }
    gralloc_buffers_.clear();
  }
  delete[] gralloc_slots_;

  if (nullptr != gralloc_device_) {
    gralloc_device_->common.close(&gralloc_device_->common);
  }

  QMMF_INFO("%s:%s: Exit (%p)", TAG, __func__, this);
}

int32_t MemPool::Initialize(const MemPoolParams &params) {
  status_t ret = NO_ERROR;
  hw_module_t const *module = nullptr;

  params_ = params;

  ret = hw_get_module(GRALLOC_HARDWARE_MODULE_ID, &module);
  if ((NO_ERROR != ret) || (nullptr == module)) {
    QMMF_ERROR("%s:%s: Unable to load GrallocHal module: %d", TAG,
               __func__, ret);
    return ret;
  }

  ret = module->methods->open(module, GRALLOC_HARDWARE_GPU0,
                              (struct hw_device_t **)&gralloc_device_);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Could not open Gralloc module: %s (%d)", TAG,
               __func__, strerror(-ret), ret);
    goto FAIL;
  }

  QMMF_INFO("%s:%s: Gralloc Module author: %s, version: %d name: %s",
            TAG, __func__,
            gralloc_device_->common.module->author,
            gralloc_device_->common.module->hal_api_version,
            gralloc_device_->common.module->name);

  // Allocate gralloc slots.
  if (params_.max_buffer_count > 0) {
    gralloc_slots_ = new buffer_handle_t[params_.max_buffer_count];
    if (gralloc_slots_ == nullptr) {
      QMMF_ERROR("%s:%s: Unable to allocate buffer handles!", TAG, __func__);
      ret = NO_MEMORY;
      goto FAIL;
    }
  } else {
    gralloc_slots_ = nullptr;
  }

  return NO_ERROR;

FAIL:
  if (nullptr != gralloc_device_) {
    gralloc_device_->common.close(&gralloc_device_->common);
  }
  return -1;
}

status_t MemPool::ReturnBufferLocked(const StreamBuffer &buffer) {

  if (pending_buffer_count_ == 0) {
    QMMF_ERROR("%s:%s: Not expecting any buffers!", TAG, __func__);
    return INVALID_OPERATION;
  }
  std::lock_guard<std::mutex> lock(buffer_lock_);

  int32_t idx = gralloc_buffers_.indexOfKey(buffer.handle);
  if (-ENOENT == idx) {
    QMMF_ERROR("%s:%s: Buffer %p returned that wasn't allocated by this node",
        TAG, __func__, buffer.handle);
    return BAD_VALUE;
  }

  gralloc_buffers_.replaceValueFor(buffer.handle, true);
  pending_buffer_count_--;

  wait_for_buffer_.notify_one();
  return NO_ERROR;
}

status_t MemPool::GetBuffer(StreamBuffer* buffer) {

  std::unique_lock<std::mutex> lock(buffer_lock_);
  std::chrono::nanoseconds wait_time(kBufferWaitTimeout);

  buffer->fd = -1;

  if (gralloc_slots_ == nullptr) {
    QMMF_ERROR("%s:%s: Error gralloc slots!", TAG, __func__);
    return NO_ERROR;
  }

  if (pending_buffer_count_ == params_.max_buffer_count) {
    QMMF_VERBOSE("%s:%s: Already retrieved maximum buffers (%d), waiting"
        " on a free one", TAG, __func__, params_.max_buffer_count);

    auto ret = wait_for_buffer_.wait_for(lock, wait_time);
    if (ret == std::cv_status::timeout) {
      QMMF_ERROR("%s:%s: Wait for output buffer return timed out", TAG,
                 __func__);
      return TIMED_OUT;
    }
  }
  auto ret = GetBufferLocked(buffer);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Failed to retrieve output buffer", TAG, __func__);
    return ret;
  }

  return NO_ERROR;
}

status_t MemPool::GetBufferLocked(StreamBuffer* buffer) {

  status_t ret = NO_ERROR;
  int32_t idx = -1;
  buffer_handle_t handle = nullptr;

  //Only pre-allocate buffers in case no valid streamBuffer
  //is passed as an argument.
  if (nullptr != buffer) {
    for (uint32_t i = 0; i < gralloc_buffers_.size(); i++) {
      if (gralloc_buffers_.valueAt(i)) {
        handle = gralloc_buffers_.keyAt(i);
        gralloc_buffers_.replaceValueAt(i, false);
        break;
      }
    }
  }
  // Find the slot of the available gralloc buffer.
  if (nullptr != handle) {
    for (uint32_t i = 0; i < buffers_allocated_; i++) {
      if (gralloc_slots_[i] == handle) {
        idx = i;
        break;
      }
    }
  } else if ((nullptr == handle) &&
             (buffers_allocated_ < params_.max_buffer_count)) {
    ret = AllocGrallocBuffer(&handle);
    if (NO_ERROR != ret) {
      return ret;
    }
    idx = buffers_allocated_;
    gralloc_slots_[idx] = handle;
    gralloc_buffers_.add(gralloc_slots_[idx], (nullptr == buffer));
    buffers_allocated_++;
  }

  if ((nullptr == handle) || (0 > idx)) {
    QMMF_ERROR("%s:%s: Unable to allocate or find a free buffer!", TAG,
               __func__);
    return INVALID_OPERATION;
  }

  if (nullptr != buffer) {
    struct private_handle_t *priv_handle = (struct private_handle_t *)
        gralloc_slots_[idx];
    ret = PopulateMetaInfo(buffer->info, priv_handle);
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s:%s: Failed to populate buffer meta info", TAG, __func__);
      return ret;
    }
    buffer->handle = gralloc_slots_[idx];
    buffer->fd = priv_handle->fd;
    buffer->size = priv_handle->size;
    pending_buffer_count_++;
  }

  return ret;
}

status_t MemPool::PopulateMetaInfo(CameraBufferMetaData &info,
                                   struct private_handle_t *priv_handle) {

  if (nullptr == priv_handle) {
    QMMF_ERROR("%s:%s: Invalid private handle!\n", TAG, __func__);
    return BAD_VALUE;
  }

  int alignedW, alignedH;
  gralloc_module_t const *mapper = reinterpret_cast<gralloc_module_t const *>(
          gralloc_device_->common.module);
  status_t ret = mapper->perform(mapper,
      GRALLOC_MODULE_PERFORM_GET_CUSTOM_STRIDE_AND_HEIGHT_FROM_HANDLE,
      priv_handle, &alignedW, &alignedH);

  if (0 != ret) {
    QMMF_ERROR("%s:%s: Unable to query stride&scanline: %d\n", TAG, __func__,
               ret);
    return ret;
  }

  switch (priv_handle->format) {
    case HAL_PIXEL_FORMAT_BLOB:
      info.format = BufferFormat::kBLOB;
      info.num_planes = 1;
      info.plane_info[0].width = params_.max_size;
      info.plane_info[0].height = 1;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
      info.format = BufferFormat::kNV12;
      info.num_planes = 2;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      info.plane_info[1].width = params_.width;
      info.plane_info[1].height = params_.height/2;
      info.plane_info[1].stride = alignedW;
      info.plane_info[1].scanline = alignedH/2;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
      info.format = BufferFormat::kNV12UBWC;
      info.num_planes = 2;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      info.plane_info[1].width = params_.width;
      info.plane_info[1].height = params_.height/2;
      info.plane_info[1].stride = alignedW;
      info.plane_info[1].scanline = alignedH/2;
      break;
    case HAL_PIXEL_FORMAT_NV21_ZSL:
      info.format = BufferFormat::kNV21;
      info.num_planes = 2;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      info.plane_info[1].width = params_.width;
      info.plane_info[1].height = params_.height/2;
      info.plane_info[1].stride = alignedW;
      info.plane_info[1].scanline = alignedH/2;
      break;
    case HAL_PIXEL_FORMAT_RAW10:
      info.format = BufferFormat::kRAW10;
      info.num_planes = 1;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      break;
    case HAL_PIXEL_FORMAT_RAW12:
      info.format = BufferFormat::kRAW12;
      info.num_planes = 1;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      break;
    case HAL_PIXEL_FORMAT_RAW16:
      info.format = BufferFormat::kRAW16;
      info.num_planes = 1;
      info.plane_info[0].width = params_.width;
      info.plane_info[0].height = params_.height;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      break;
    default:
      QMMF_ERROR("%s:%s: Unsupported format: %d", TAG, __func__,
                 priv_handle->format);
      return NAME_NOT_FOUND;
  }

  return NO_ERROR;
}

status_t MemPool::AllocGrallocBuffer(buffer_handle_t *buf) {

  status_t ret      = NO_ERROR;
  uint32_t width    = params_.width;
  uint32_t height   = params_.height;
  int32_t  format   = params_.format;
  int32_t  usage    = params_.gralloc_flags;
  uint32_t max_size = params_.max_size;

  // Filter out any usage bits that shouldn't be passed to the gralloc module.
  usage &= GRALLOC_USAGE_ALLOC_MASK;

  if (!width || !height) {
    width = height = 1;
  }

  int stride = 0;
  if (0 < max_size) {
    // Blob buffers are expected to get allocated with width equal to blob
    // max size and height equal to 1.
    ret = gralloc_device_->alloc(gralloc_device_, static_cast<int>(max_size),
                                 static_cast<int>(1), format,
                                 static_cast<int>(usage), buf, &stride);
  } else {
    ret = gralloc_device_->alloc(gralloc_device_, static_cast<int>(width),
                                 static_cast<int>(height), format,
                                 static_cast<int>(usage), buf, &stride);
  }
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Failed to allocate gralloc buffer", TAG, __func__);
  }
  return ret;
}

status_t MemPool::FreeGrallocBuffer(buffer_handle_t buf) {
  return gralloc_device_->free(gralloc_device_, buf);
}

}; //namespace recorder.

}; //namespace qmmf.
