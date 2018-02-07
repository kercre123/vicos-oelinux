/*
 * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#include <chrono>
#include <condition_variable>
#include <iomanip>
#include <list>
#include <map>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>

#include <system/graphics.h>
#include <system/window.h>
#include <utils/List.h>
#include <qcom/display/gralloc_priv.h>

#include "common/qmmf_log.h"
#include "qmmf-sdk/qmmf_codec.h"

namespace qmmf {

typedef int32_t status_t;

const int64_t kWaitDelay = 2000000000;  // 2 sec

struct StreamBuffer {
  CameraBufferMetaData info;
  int64_t  timestamp;
  uint32_t frame_number;
  uint32_t camera_id;
  int32_t  stream_id;
  android_dataspace data_space;
  buffer_handle_t handle;
  int32_t fd;
  uint32_t size;
  void *data;
  uint32_t flags;
  uint32_t filled_length;
  uint32_t frame_length;
  uint32_t pending_encodes_per_frame;
  uint32_t encodes_per_frame_count;
  bool needs_return;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "camera[" << camera_id << "] ";
    stream << "stream[" << stream_id << "] ";
    stream << "data[" << data << "] ";
    stream << "fd[" << fd << "] ";
    stream << "size[" << size << "] ";
    stream << "timestamp[" << timestamp << "] ";
    stream << "flags[" << ::std::setbase(16) << flags << ::std::setbase(10)
           << "]";
    stream << "pending_encodes_per_frame[" << pending_encodes_per_frame << "] ";
    stream << "encodes_per_frame_count[" << encodes_per_frame_count << "] ";
    stream << "needs_return[" << ::std::boolalpha << needs_return
           << ::std::noboolalpha << "] ";
    return stream.str();
  }
};

class Common {
 public:
  /** FromQmmfToHalFormat
   *
   * Translates QMMF format to HAL format
   *
   * return: HAL format
   **/
  static int32_t FromQmmfToHalFormat(const BufferFormat &format) {
    switch (format) {
      case BufferFormat::kBLOB:
        return HAL_PIXEL_FORMAT_BLOB;
        break;
      case BufferFormat::kNV12UBWC:
      case BufferFormat::kNV12:
        return HAL_PIXEL_FORMAT_YCbCr_420_888;
        break;
      case BufferFormat::kNV21:
        return HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
        break;
      case BufferFormat::kRAW10:
        return HAL_PIXEL_FORMAT_RAW10;
        break;
      case BufferFormat::kRAW12:
        return HAL_PIXEL_FORMAT_RAW12;
        break;
      case BufferFormat::kRAW16:
        return HAL_PIXEL_FORMAT_RAW16;
        break;
      default:
        /* Format not supported */
        QMMF_ERROR("%s: error: unsupported format %d (0x%x)", __func__, format,
          (unsigned int) format);
        return -1;
    }
  }

  /** FromHalToQmmfFormat
   *
   * Translates HAL format to QMMF format
   *
   * return: QMMF format
   **/
  static BufferFormat FromHalToQmmfFormat(const int32_t &format) {
    switch (format) {
      case HAL_PIXEL_FORMAT_BLOB:
        return BufferFormat::kBLOB;
        break;
      case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
        return BufferFormat::kNV12UBWC;
        break;
      case HAL_PIXEL_FORMAT_YCbCr_420_888:
        return BufferFormat::kNV12;
        break;
      case HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED:
        return BufferFormat::kNV21;
        break;
      case HAL_PIXEL_FORMAT_RAW10:
        return BufferFormat::kRAW10;
        break;
      case HAL_PIXEL_FORMAT_RAW12:
        return BufferFormat::kRAW12;
        break;
      case HAL_PIXEL_FORMAT_RAW16:
        return BufferFormat::kRAW16;
        break;
      default:
        /* Format not supported */
        QMMF_ERROR("%s: error: unsupported format %d (0x%x)", __func__, format,
          (unsigned int) format);
        return BufferFormat::kUnsupported;
    }
  }
};

// Thread safe Queue
template <class T>
class TSQueue {
 public:
  typedef typename ::android::List<T>::iterator iterator;

  iterator Begin() {
    ::std::lock_guard<::std::mutex> lg(lock_);
    return queue_.begin();
  }

  void PushBack(const T& item) {
    ::std::lock_guard<::std::mutex> lg(lock_);
    queue_.push_back(item);
  }

  int32_t Size() {
    ::std::lock_guard<::std::mutex> lg(lock_);
    return queue_.size();
  }

  bool Empty() {
    ::std::lock_guard<::std::mutex> lg(lock_);
    return queue_.empty();
  }

  iterator End() {
    ::std::lock_guard<::std::mutex> lg(lock_);
    return queue_.end();
  }

  void Erase(iterator it) {
    ::std::lock_guard<::std::mutex> lg(lock_);
    queue_.erase(it);
  }

  void Clear() {
    ::std::lock_guard<::std::mutex> lg(lock_);
    queue_.clear();
  }

 private:
  ::android::List<T> queue_;
  ::std::mutex lock_;
};

template <class T>
class SignalQueue {
 public:
  explicit SignalQueue(uint32_t size) : max_size_(size) {
    QMMF_DEBUG("%s: Enter", __func__);
    QMMF_DEBUG("%s: Exit", __func__);
  }

  ~SignalQueue() {
    QMMF_DEBUG("%s: Enter", __func__);
    QMMF_DEBUG("%s: Exit", __func__);
  }

  uint32_t Size() {
    QMMF_DEBUG("%s: Enter", __func__);
    ::std::lock_guard<::std::mutex> lg(cmd_queue_mutex_);
    QMMF_DEBUG("%s: Exit", __func__);
    return cmd_queue_.size();
  }

  status_t Pop(T* item) {
    QMMF_DEBUG("%s: Enter", __func__);
    if (!item) {
      QMMF_ERROR("%s: Invalid Parameters", __func__);
      return -1;
    }
    {
      ::std::unique_lock<::std::mutex> lock(cmd_queue_mutex_);
      auto ret = wait_for_cmd_.wait_for(lock,
          ::std::chrono::nanoseconds(kWaitDelay),
          [this]{return (cmd_queue_.size() > 0);});
      if (!ret) {
        QMMF_ERROR("%s: Wait for cmd.. timed out", __func__);
        return -1;
      } else {
        *item  = cmd_queue_.front();
        cmd_queue_.pop();
        QMMF_DEBUG("%s: Updated SignalQueue Size = %u", __func__,
            cmd_queue_.size());
      }
    }
    QMMF_DEBUG("%s: Exit", __func__);
    return 0;
  }

  status_t Push(const T& item) {
    QMMF_DEBUG("%s: Enter", __func__);
    {
      ::std::lock_guard<::std::mutex> lg(cmd_queue_mutex_);
      uint32_t size = cmd_queue_.size();
      if (max_size_ <= size) {
        QMMF_ERROR("%s: command queue size full", __func__);
        return -1;
      }
      cmd_queue_.push(item);
      QMMF_DEBUG("%s: Updated SignalQueue Size = %u", __func__,
          cmd_queue_.size());
    }
    wait_for_cmd_.notify_one();
    QMMF_DEBUG("%s: Exit", __func__);
    return 0;
  }

  void Clear() {
    QMMF_INFO("%s: Enter", __func__);
    ::std::lock_guard<::std::mutex> lg(cmd_queue_mutex_);
    while(!cmd_queue_.empty())
      cmd_queue_.pop();
    QMMF_INFO("%s: Exit", __func__);
  }

 private:
  ::std::condition_variable  wait_for_cmd_;
  ::std::mutex               cmd_queue_mutex_;
  ::std::queue<T>            cmd_queue_;
  uint32_t                   max_size_;
};  // SignalQueue

// Thread safe KeyedVector
template <class T1, class T2>
class TSKeyedVector {
 public:
  void Add(StreamBuffer& buffer) {
    ::std::lock_guard<::std::mutex> lg(lock_);
    map_.insert(std::make_pair(buffer.handle, 1));
  }

  uint32_t ValueFor(StreamBuffer& buffer) {
    ::std::lock_guard<::std::mutex> lg(lock_);
    return map_.at(buffer.handle);
  }

  void RemoveItem(StreamBuffer& buffer) {
    ::std::lock_guard<::std::mutex> lg(lock_);
    map_.erase(buffer.handle);
  }

  int32_t Size() {
    ::std::lock_guard<::std::mutex> lg(lock_);
    return map_.size();
  }

  bool IsEmpty() {
    ::std::lock_guard<::std::mutex> lg(lock_);
    return map_.empty();
  }

  void ReplaceValueFor(StreamBuffer& buffer, uint32_t value) {
    ::std::lock_guard<::std::mutex> lg(lock_);
    map_[buffer.handle] = value;
  }

  void Clear() {
    ::std::lock_guard<::std::mutex> lg(lock_);
    map_.clear();
  }

  bool IsExist(const StreamBuffer& buffer) {
    ::std::lock_guard<::std::mutex> lg(lock_);
    auto search = map_.find(buffer.handle);
    if(search != map_.end()) {
      return true;
    } else {
      return false;
    }
  }

 private:
  ::std::map<T1, T2> map_;
  ::std::mutex lock_;
};

};  // namespace qmmf.
