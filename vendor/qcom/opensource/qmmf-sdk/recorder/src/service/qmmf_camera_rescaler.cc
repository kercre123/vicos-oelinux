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

#define TAG "RecorderCameraSourceCopy"

#include <math.h>
#include <sys/mman.h>
#include <dlfcn.h>
#include <hardware/hardware.h>
#include <fastcv/fastcv.h>
#include <adreno/c2d2.h>
#include <linux/msm_kgsl.h>
#include <chrono>
#include <map>

#include "recorder/src/service/qmmf_camera_rescaler.h"

#include "recorder/src/service/qmmf_recorder_utils.h"

namespace qmmf {

namespace recorder {

using ::std::chrono::high_resolution_clock;
using ::std::chrono::microseconds;
using ::std::chrono::nanoseconds;
using ::std::chrono::time_point;
using ::std::chrono::duration_cast;


#define FPS_CHANGE_THRESHOLD  (0.005)
#define FRAME_SKIP_THRESHOLD_PERCENT (0.05)

using namespace android;

C2dRescaler::C2dRescaler() {
  QMMF_INFO("%s:%s: Enter (%p)", TAG, __func__, this);
  src_surface_id_ = -1;
  target_surface_id_ = -1;
  char prop[PROPERTY_VALUE_MAX];
  memset(prop, 0, sizeof(prop));
  property_get("persist.qipcam.rescaler.perf", prop, "0");
  uint32_t value = (uint32_t) atoi(prop);
  print_process_time_ = (value == 1) ? true : false;
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

C2dRescaler::~C2dRescaler() {
  QMMF_INFO("%s:%s: Enter (%p)", TAG, __func__, this);
  if(target_surface_id_) {
    c2dDestroySurface(target_surface_id_);
    target_surface_id_ = 0;
  }
  if(src_surface_id_) {
    c2dDestroySurface(src_surface_id_);
    src_surface_id_ = 0;
  }
  QMMF_INFO("%s:%s: Exit ", TAG, __func__);
}

int32_t C2dRescaler::Init() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  C2D_YUV_SURFACE_DEF surface_def = {
    C2D_COLOR_FORMAT_420_NV12,
    1 * 4,
    1 * 4,
    (void*)0xaaaaaaaa,
    (void*)0xaaaaaaaa,
    1 * 4,
    (void*)0xaaaaaaaa,
    (void*)0xaaaaaaaa,
    1 * 4,
    (void*)0xaaaaaaaa,
    (void*)0xaaaaaaaa,
    1 * 4,
  };

  //target C2dSurface for output buffers. at this point surface
  //is dummy, it is not mapped to GPU, it will be mapped
  //to GPU once input buffers (camera stream buffers) will be
  //available for copy.
  auto ret = c2dCreateSurface(&target_surface_id_, C2D_TARGET,
                           (C2D_SURFACE_TYPE)(C2D_SURFACE_YUV_HOST
                           |C2D_SURFACE_WITH_PHYS
                           |C2D_SURFACE_WITH_PHYS_DUMMY),
                           &surface_def);
  if(ret != C2D_STATUS_OK) {
    QMMF_ERROR("%s:%s: c2dCreateSurface failed!", TAG, __func__);
    return ret;
  }

  //Dummy surface for camera stream buffers, this surface will be
  //updated by actual camera buffers.
  ret = c2dCreateSurface(&src_surface_id_, C2D_SOURCE,
                               (C2D_SURFACE_TYPE)(C2D_SURFACE_YUV_HOST
                               |C2D_SURFACE_WITH_PHYS
                               |C2D_SURFACE_WITH_PHYS_DUMMY),
                               &surface_def);
  if(ret != C2D_STATUS_OK) {
    QMMF_ERROR("%s:%s: c2dCreateSurface failed!", TAG, __func__);
    return ret;
  }

  QMMF_INFO("%s:%s: Exit streamId", TAG, __func__);
  return ret;
}

int32_t C2dRescaler::CopyBuffer(StreamBuffer& src_buffer,
                                StreamBuffer& dst_buffer) {
  QMMF_DEBUG("%s:%s: Enter (%p)", TAG, __func__, this);
  time_point<high_resolution_clock>   start_time;
  if (print_process_time_) {
    start_time = high_resolution_clock::now();
  }

  int32_t ret = 0;
  int32_t src_buf_fd       = 0;
  int32_t src_buf_frame_len = 0;
  int32_t data_offset     = 0;
  void* src_buf_vaddr      = nullptr;
  void* src_buf_gpu_addr    = nullptr;
  void* target_buf_gpu_addr = nullptr;
  int32_t plane_y_len      = 0;
  C2D_OBJECT draw_object[1];

  C2D_YUV_SURFACE_DEF src_surface_def;
  C2D_YUV_SURFACE_DEF target_surface_def;
  uint32_t c2d_color_format = C2D_COLOR_FORMAT_420_NV12;

  src_buf_fd       = src_buffer.fd;
  src_buf_frame_len = src_buffer.size;

  QMMF_DEBUG("%s:%s: src_buf_fd = %d", TAG, __func__, src_buf_fd);
  QMMF_DEBUG("%s:%s: src_buf_frame_len = %d", TAG, __func__, src_buf_frame_len);

  src_buf_vaddr = src_buffer.data;
  if(src_buf_vaddr == nullptr) {
   QMMF_ERROR("%s:%s: Invalid src_buf_vaddr!", TAG, __func__);
   goto EXIT_2;
  }

  //STEP2: Map Input Camera stream buffer to GPU.
  data_offset  = 0;
  src_buf_gpu_addr = nullptr;
  ret = c2dMapAddr(src_buf_fd, src_buf_vaddr, src_buf_frame_len, data_offset,
                   KGSL_USER_MEM_TYPE_ION, &src_buf_gpu_addr);
  if(ret != C2D_STATUS_OK) {
   QMMF_ERROR("%s:%s: c2dMapAddr failed!", TAG, __func__);
   goto EXIT_2;
  }

  if(src_buf_gpu_addr == nullptr) {
   QMMF_ERROR("%s:%s: Invalid src_buf_gpu_addr!", TAG, __func__);
   goto EXIT_2;
  }

  //STEP3: Map target ION buffer to GPU.
  target_buf_gpu_addr = nullptr;

  ret = c2dMapAddr(dst_buffer.fd, dst_buffer.data, dst_buffer.size,
                   data_offset, KGSL_USER_MEM_TYPE_ION, &target_buf_gpu_addr);
  if(ret != C2D_STATUS_OK) {
   QMMF_ERROR("%s:%s: c2dMapAddr failed!", TAG, __func__);
   goto EXIT_1;
  }

  if(target_buf_gpu_addr == nullptr) {
   QMMF_ERROR("%s:%s: Invalid target_buf_gpu_addr!", TAG, __func__);
   goto EXIT_1;
  }

  //STEP4: Create source C2dSurface for input Camera stream buffer.
  if ((src_buffer.info.plane_info[0].width == 0) ||
      (src_buffer.info.plane_info[0].height == 0)) {
   QMMF_ERROR("%s:%s: Invalid Src size!", TAG, __func__);
   goto EXIT;
  }

  src_surface_def.width   = src_buffer.info.plane_info[0].width;
  src_surface_def.height  = src_buffer.info.plane_info[0].height;

  switch (src_buffer.info.format) {
    case BufferFormat::kNV21:
      c2d_color_format = C2D_COLOR_FORMAT_420_NV21;
      break;
    case BufferFormat::kNV12:
      c2d_color_format = C2D_COLOR_FORMAT_420_NV12;
      break;
    default:
      QMMF_ERROR("%s:%s: Unsupported format: %d", TAG,  __func__,
          src_buffer.info.format);
      ret = BAD_VALUE;
      goto EXIT;
  }

  src_surface_def.format  = c2d_color_format;

  //Y plane stride.
  src_surface_def.stride0 = src_buffer.info.plane_info[0].stride;

  //UV plane stride.
  src_surface_def.stride1 = src_buffer.info.plane_info[0].stride;

  //UV plane hostptr.
  plane_y_len = src_surface_def.stride0 * src_buffer.info.plane_info[0].scanline;

  //Y plane hostptr.
  src_surface_def.plane0 = src_buf_vaddr;

  QMMF_DEBUG("%s:%s: src_surface_def.width = %d ", TAG, __func__,
                              src_surface_def.width);
  QMMF_DEBUG("%s:%s: src_surface_def.height = %d ", TAG, __func__,
                             src_surface_def.height);
  QMMF_DEBUG("%s:%s: src_surface_def.stride0 = %d ", TAG, __func__,
                            src_surface_def.stride0);
  QMMF_DEBUG("%s:%s: src_surface_def.stride1 = %d ", TAG, __func__,
                            src_surface_def.stride1);
  QMMF_DEBUG("%s:%s: plane_y_len = %d", TAG, __func__,
                                        plane_y_len);

  //Y plane Gpu address.
  src_surface_def.phys0   = src_buf_gpu_addr;

  src_surface_def.plane1  = (void*)((intptr_t)src_buf_vaddr + plane_y_len);

  //UV plane Gpu address.
  src_surface_def.phys1 = (void*)((intptr_t)src_buf_gpu_addr + plane_y_len);

  ret = c2dUpdateSurface(src_surface_id_, C2D_SOURCE,
                          (C2D_SURFACE_TYPE)(C2D_SURFACE_YUV_HOST
                          |C2D_SURFACE_WITH_PHYS), &src_surface_def);

  if(ret != C2D_STATUS_OK) {
   QMMF_ERROR("%s:%s: c2dUpdateSurface failed!", TAG, __func__);
   goto EXIT;
  }
  QMMF_DEBUG("%s:%s: src_surface_id_ = %d", TAG, __func__, src_surface_id_);

  //STEP5: Update target C2dSurface.
  target_surface_def.format  = c2d_color_format;
  target_surface_def.width   = dst_buffer.info.plane_info[0].width;
  target_surface_def.height  = dst_buffer.info.plane_info[0].height;
  QMMF_DEBUG("%s:%s: target_surface_def.width = %d ", TAG, __func__,
                           target_surface_def.width);
  QMMF_DEBUG("%s:%s: target_surface_def.height = %d ", TAG, __func__,
                         target_surface_def.height);
  //Y plane stride.
  target_surface_def.stride0 = dst_buffer.info.plane_info[0].stride;

  QMMF_DEBUG("%s:%s: target_surface_def.stride0 = %d ", TAG,  __func__,
                         target_surface_def.stride0);
  //Y plane hostptr.
  target_surface_def.plane0  = dst_buffer.data;

  //Y plane Gpu address.
  target_surface_def.phys0   = target_buf_gpu_addr;

  //UV plane stride.
  target_surface_def.stride1 = dst_buffer.info.plane_info[0].stride;

  QMMF_DEBUG("%s:%s: target_surface_def.stride1 = %d ", TAG,  __func__,
                         target_surface_def.stride1);

  plane_y_len =
      target_surface_def.stride0 * dst_buffer.info.plane_info[0].scanline;

  //UV plane hostptr.
  target_surface_def.plane1  = (void*)((intptr_t)dst_buffer.data + plane_y_len);
  //UV plane Gpu address.
  target_surface_def.phys1 = (void*)((intptr_t)target_buf_gpu_addr + plane_y_len);

  ret = c2dUpdateSurface(target_surface_id_, C2D_SOURCE,
                          (C2D_SURFACE_TYPE)(C2D_SURFACE_YUV_HOST
                          |C2D_SURFACE_WITH_PHYS), &target_surface_def);
  if(ret != C2D_STATUS_OK) {
   QMMF_ERROR("%s:%s: c2dUpdateSurface failed!", TAG, __func__);
   goto EXIT;
  }

  //STEP6: Create C2dObject outof source surface and fill target rectangle
  //values.
  draw_object[0].surface_id  = src_surface_id_;
  draw_object[0].config_mask = C2D_ALPHA_BLEND_NONE
                             |C2D_TARGET_RECT_BIT;

  if ((0 < dst_buffer.info.plane_info[0].width) &&
      (0 < dst_buffer.info.plane_info[0].height)) {
    {
      std::lock_guard<std::mutex> l(crop_lock_);
      draw_object[0].config_mask |= C2D_SOURCE_RECT_BIT;
      draw_object[0].source_rect.x = 0;
      draw_object[0].source_rect.y = 0;
      draw_object[0].source_rect.width =
          (src_buffer.info.plane_info[0].width) << 16;
      draw_object[0].source_rect.height =
          (src_buffer.info.plane_info[0].height)<< 16;
    }
  }

  draw_object[0].target_rect.width  = dst_buffer.info.plane_info[0].width << 16;
  draw_object[0].target_rect.height = dst_buffer.info.plane_info[0].height << 16;
  draw_object[0].target_rect.x      = 0;
  draw_object[0].target_rect.y      = 0;

  //STEP7: Draw C2dObject on target surface.
  ret = c2dDraw(target_surface_id_, 0, 0, 0, 0, draw_object, 1);
  if(ret != C2D_STATUS_OK) {
   QMMF_ERROR("%s:%s: c2dDraw failed!", TAG, __func__);
   goto EXIT;
  }

  ret = c2dFinish(target_surface_id_);
  if(ret != C2D_STATUS_OK) {
     QMMF_ERROR("%s:%s: c2dFinish failed!", TAG, __func__);
     goto EXIT;
  }

EXIT:
  //STEP8: Unmap input src_buffer and targetBuffer from GPU.
  ret = c2dUnMapAddr(src_buf_gpu_addr);
  if(ret != C2D_STATUS_OK) {
   QMMF_ERROR("%s:%s: c2dUnMapAddr failed!", TAG, __func__);
  }

EXIT_1:
  ret = c2dUnMapAddr(target_buf_gpu_addr);
  if(ret != C2D_STATUS_OK) {
     QMMF_ERROR("%s:%s: c2dUnMapAddr failed!", TAG, __func__);
  }

EXIT_2:
  if(print_process_time_) {
    time_point<high_resolution_clock> curr_time = high_resolution_clock::now();
    uint64_t time_diff = duration_cast<microseconds>
                             (curr_time - start_time).count();
    QMMF_INFO("%s:%s: stream_id(%d) C2D Full ProcessingTime=%lld",
        TAG, __func__, src_buffer.stream_id, time_diff);
  }

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
}


FastCVRescaler::FastCVRescaler()
  : fastcv_level_(FASTCV_OP_CPU_PERFORMANCE) {
  char prop[PROPERTY_VALUE_MAX];
  memset(prop, 0, sizeof(prop));
  property_get("persist.qmmf.rescaler.perf", prop, "0");
  uint32_t value = (uint32_t) atoi(prop);
  print_process_time_ = (value == 1) ? true : false;
}

int32_t FastCVRescaler::Init() {

  char prop[PROPERTY_VALUE_MAX];
  memset(prop, 0, sizeof(prop));
  property_get("persist.qmmf.fastcv.level", prop, "3");
  uint32_t level = (uint32_t) atoi(prop);

  if ((level == FASTCV_OP_LOW_POWER) ||
      (level == FASTCV_OP_PERFORMANCE) ||
      (level == FASTCV_OP_CPU_OFFLOAD) ||
      (level == FASTCV_OP_CPU_PERFORMANCE)) {
    fastcv_level_ = level;
  } else {
    fastcv_level_ = FASTCV_OP_CPU_PERFORMANCE;
  }

  int stat = fcvSetOperationMode(FASTCV_OP_LOW_POWER);
  QMMF_INFO("%s:%s: set fcvSetOperationMode %d",TAG , __func__, fastcv_level_);

  if (0 != stat) {
    QMMF_ERROR("%s:%s: Unable to set FastCV operation mode: %d", TAG, __func__,
        stat);
  }

  return 0;
}

int32_t FastCVRescaler::CopyBuffer(StreamBuffer& src_buffer,
                                   StreamBuffer& dst_buffer) {
  time_point<high_resolution_clock>   start_time;
  if (print_process_time_) {
    start_time = high_resolution_clock::now();
  }

  int32_t ret = NO_ERROR;
  uint8_t *src_buffer_y, *dst_buffer_y;
  uint8_t *src_buffer_uv, *dst_buffer_uv;
  size_t src_stride_y, dst_stride_y;
  size_t src_plane_y_len, dst_plane_y_len;

  if ((src_buffer.info.format != BufferFormat::kNV21) &&
      (src_buffer.info.format != BufferFormat::kNV12)) {
    QMMF_ERROR("%s:%s: Unsupported input format: 0x%x!",TAG, __func__,
        src_buffer.info.format);
    QMMF_ERROR("%s:%s: Only NV12/NV21 are supported currently!", TAG, __func__);
    ret = BAD_VALUE;
    goto EXIT;
  }

  src_buffer_y = reinterpret_cast<uint8_t*>(src_buffer.data);

  src_stride_y = src_buffer.info.plane_info[0].stride;
  src_plane_y_len = src_stride_y * src_buffer.info.plane_info[0].scanline;

  src_buffer_uv = src_buffer_y + src_plane_y_len;

  dst_buffer_y = reinterpret_cast<uint8_t*>(dst_buffer.data);
  dst_stride_y = dst_buffer.info.plane_info[0].stride;
  dst_plane_y_len = dst_stride_y * dst_buffer.info.plane_info[0].scanline;
  dst_buffer_uv = dst_buffer_y + dst_plane_y_len;

  //STEP2: Scale down the two planes
  fcvScaleu8_v2(src_buffer_y,
      src_buffer.info.plane_info[0].width,
      src_buffer.info.plane_info[0].height,
      src_stride_y,
      dst_buffer_y, dst_buffer.info.plane_info[0].width ,
      dst_buffer.info.plane_info[0].height,
      dst_stride_y
      ,FASTCV_INTERPOLATION_TYPE_NEAREST_NEIGHBOR ,
      FASTCV_BORDER_REPLICATE,
      0
      );

  if(print_process_time_) {
    time_point<high_resolution_clock> curr_time = high_resolution_clock::now();
    uint64_t time_diff = duration_cast<microseconds>
                             (curr_time - start_time).count();
    QMMF_INFO("%s:%s: stream_id(%d) FastCV Y ProcessingTime=%lld", __func__,
        TAG, src_buffer.stream_id, time_diff);
  }

  fcvScaleDownMNInterleaveu8(src_buffer_uv,
      src_buffer.info.plane_info[0].width >> 1,
      src_buffer.info.plane_info[0].height >> 1,
      src_stride_y,
      dst_buffer_uv, dst_buffer.info.plane_info[0].width >> 1,
      dst_buffer.info.plane_info[0].height >> 1, dst_stride_y
      );

EXIT:

  if(print_process_time_) {
    time_point<high_resolution_clock> curr_time = high_resolution_clock::now();
    uint64_t time_diff = duration_cast<microseconds>
                             (curr_time - start_time).count();
    QMMF_INFO("%s:%s: stream_id(%d) FastCV Full ProcessingTime=%lld",
        TAG, __func__, src_buffer.stream_id, time_diff);
  }
  return ret;
}

CameraRescalerBase::CameraRescalerBase()
    :CameraRescalerThread() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  char prop[PROPERTY_VALUE_MAX];
  memset(prop, 0, sizeof(prop));
  property_get("persist.qmmf.rescaler.c2d", prop, "1");
  uint32_t value = (uint32_t) atoi(prop);

  if (value == 1) {
    rescaler_ = new C2dRescaler();
  } else {
    rescaler_ = new FastCVRescaler();
  }

  rescaler_->Init();
  QMMF_INFO("%s:%s: Exit (%p)", TAG, __func__, this);
}

CameraRescalerBase::~CameraRescalerBase() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  delete rescaler_;
  QMMF_INFO("%s:%s: Exit (%p)", TAG, __func__, this);
}

status_t CameraRescalerBase::ReturnBufferToBufferPool(
    const StreamBuffer &buffer) {
  status_t ret = ReturnBufferLocked(buffer);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Failed to return buffer to memory pool", TAG, __func__);
  }
  return ret;
}

void CameraRescalerBase::FlushBufs() {
  std::unique_lock<std::mutex> lock(wait_lock_);

  StreamBuffer buffer;
  auto iter = bufs_list_.begin();
  while (iter != bufs_list_.end()) {
    iter = bufs_list_.begin();
    buffer = *iter;
    bufs_list_.erase(iter);
    QMMF_INFO("%s:%s: back to client node: FD: %d", TAG, __func__, buffer.fd);
    ReturnBufferToProducer(buffer);
  }
}

void CameraRescalerBase::AddBuf(StreamBuffer& buffer) {
  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  std::unique_lock<std::mutex> lock(wait_lock_);
  bufs_list_.push_back(buffer);
  wait_.notify_one();
}

bool CameraRescalerBase::ThreadLoop() {
  bool status = true;

  StreamBuffer in_buffer;
  {
    std::unique_lock<std::mutex> lock(wait_lock_);
    std::chrono::nanoseconds wait_time(kFrameTimeout);
    while (bufs_list_.empty()) {
      auto ret = wait_.wait_for(lock, wait_time);
      if (ret == std::cv_status::timeout) {
        QMMF_DEBUG("%s:%s: Wait for frame available timed out", TAG, __func__);
        // timeout loop again
        return true;
      }
    }
    auto iter = bufs_list_.begin();
    in_buffer = *iter;
    bufs_list_.erase(iter);
  }

  StreamBuffer out_buffer;
  memset(&out_buffer, 0x0, sizeof(out_buffer));
  GetFreeOutputBuffer(&out_buffer);

  out_buffer.stream_id    = 0x55aa;
  out_buffer.timestamp    = in_buffer.timestamp;
  out_buffer.frame_number = in_buffer.frame_number;
  out_buffer.camera_id    = in_buffer.camera_id;
  out_buffer.flags        = in_buffer.flags;

  QMMF_DEBUG("%s:%s: Thread map", TAG, __func__);
  auto ret = MapBuf(out_buffer);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: fail to map in_buffer", TAG, __func__);
    ReturnBufferToBufferPool(out_buffer);
    ReturnBufferToProducer(in_buffer);
    return true;
  }

  void *vaaddr = nullptr;
  bool in_buff_map = false;
  if (in_buffer.data == nullptr) {
    vaaddr = mmap(nullptr, in_buffer.size, PROT_READ  | PROT_WRITE,
        MAP_SHARED, in_buffer.fd, 0);
    QMMF_DEBUG("%s:%s: Thread map in buff done", TAG, __func__);
    in_buff_map = true;
    in_buffer.data = vaaddr;
  }

  rescaler_->CopyBuffer(in_buffer, out_buffer);

  if (in_buff_map) {
    munmap(in_buffer.data, in_buffer.size);
    in_buffer.data = nullptr;
  }

  ReturnBufferToProducer(in_buffer);
  NotifyBufferToClient(out_buffer);

  return status;
}

status_t CameraRescalerBase::MapBuf(StreamBuffer& buffer) {
  void *vaaddr = nullptr;

  if (buffer.fd == -1) {
    QMMF_ERROR("%s:%s: Error Invalid FD", TAG, __func__);
    return BAD_VALUE;
  }

  QMMF_DEBUG("%s:%s: buffer.fd=%d buffer.size=%d", TAG, __func__,
      buffer.fd, buffer.size);

  if (mapped_buffs_.count(buffer.fd) == 0) {
    vaaddr = mmap(nullptr, buffer.size, PROT_READ  | PROT_WRITE,
        MAP_SHARED, buffer.fd, 0);
    if (vaaddr == MAP_FAILED) {
        QMMF_ERROR("%s:%s: ION mmap failed: error(%s):(%d)", TAG, __func__,
            strerror(errno), errno);
        return BAD_VALUE;
    }
    buffer.data = vaaddr;
    map_data_t map;
    map.addr = vaaddr;
    map.size = buffer.size;
    mapped_buffs_[buffer.fd] = map;
    buffer.data = vaaddr;
  } else {
    buffer.data = mapped_buffs_[buffer.fd].addr;
  }

  return NO_ERROR;
}

void CameraRescalerBase::UnMapBufs() {
  for (auto iter : mapped_buffs_) {
    auto map = iter.second;
    if (map.addr) {
      QMMF_INFO("%s:%s: Unmap addr(%p) size(%d)", TAG, __func__,
          map.addr, map.size);
      munmap(map.addr, map.size);
    }
  }
  mapped_buffs_.clear();
}

int32_t CameraRescalerThread::Run(const std::string &name) {
  int32_t res = 0;

  std::lock_guard<std::mutex> lock(lock_);
  if (running_) {
    QMMF_ERROR("%s: Thread %s already started!\n", __func__, name_.c_str());
    res = -ENOSYS;
    goto exit;
  }

  running_ = true;
  thread_ = new std::thread(MainLoop, this);
  if (thread_ == nullptr) {
    QMMF_ERROR("%s: Unable to create thread\n", __func__);
    running_ = false;
    goto exit;
  }

  if (name.empty()) {
    // use thread id as name
    std::stringstream ss;
    ss << thread_->get_id();
    name_ = ss.str();
  } else {
    name_ = name;
  }

  QMMF_INFO("%s: Thread %s is running\n", __func__, name_.c_str());

exit:
  return res;
}

void CameraRescalerThread::RequestExit() {
  std::lock_guard<std::mutex> lock(lock_);
  if (thread_ == nullptr || running_ == false) {
    QMMF_ERROR("%s: Thread %s is not running\n", __func__, name_.c_str());
    return;
  }

  abort_ = true;
}

void CameraRescalerThread::RequestExitAndWait() {
  std::lock_guard<std::mutex> lock(lock_);
  if (thread_ == nullptr) {
    QMMF_ERROR("%s: Thread %s is stopped\n", __func__, name_.c_str());
    return;
  }

  abort_ = true;
  thread_->join();
  delete(thread_);
  thread_ = nullptr;
}

void *CameraRescalerThread::MainLoop(void *userdata) {
  CameraRescalerThread *pme = reinterpret_cast<CameraRescalerThread *>(userdata);
  if (nullptr == pme) {
    pme->running_ = false;
    return nullptr;
  }

  bool run = true;
  while (pme->abort_ == false && run == true) {
    run = pme->ThreadLoop();
  }

  pme->running_ = false;
  return nullptr;
}

bool CameraRescalerThread::ExitPending() {
  std::lock_guard<std::mutex> lock(lock_);
  return (abort_ == true && running_ == true)  ? false : true;
}

#define RESCALER_BUFFERS_CNT (5)

CameraRescalerMemPool::CameraRescalerMemPool()
    : gralloc_device_(nullptr),
      gralloc_slots_(nullptr),
      buffers_allocated_(0),
      pending_buffer_count_(0),
      buffer_cnt_(RESCALER_BUFFERS_CNT) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  QMMF_INFO("%s:%s: Exit (%p)", TAG, __func__, this);
}

CameraRescalerMemPool::~CameraRescalerMemPool() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  if (gralloc_device_ != nullptr) {
    if (!gralloc_buffers_.empty()) {
      for (auto const& it : gralloc_buffers_) {
        FreeGrallocBuffer(it.first);
      }
      gralloc_buffers_.clear();
    }
    if (nullptr != gralloc_slots_) {
      delete[] gralloc_slots_;
    }

   if (nullptr != gralloc_device_->common.close) {
      gralloc_device_->common.close(&gralloc_device_->common);
    }
  }
  QMMF_INFO("%s:%s: Exit (%p)", TAG, __func__, this);
}

int32_t CameraRescalerMemPool::Initialize(uint32_t width,
                                          uint32_t height,
                                          int32_t  format) {
  status_t ret = NO_ERROR;
  hw_module_t const *module = nullptr;

  init_params_.width = width;
  init_params_.height = height;
  init_params_.format = format;

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
  if (buffer_cnt_ > 0) {
    gralloc_slots_ = new buffer_handle_t[buffer_cnt_];
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

status_t CameraRescalerMemPool::ReturnBufferLocked(const StreamBuffer &buffer) {

  if (pending_buffer_count_ == 0) {
    QMMF_ERROR("%s:%s: Not expecting any buffers!", TAG, __func__);
    return INVALID_OPERATION;
  }

  std::lock_guard<std::mutex> lock(buffer_lock_);

  if (gralloc_buffers_.find(buffer.handle) == gralloc_buffers_.end()) {
    QMMF_ERROR("%s:%s: Buffer %p returned that wasn't allocated by this node",
        TAG, __func__, buffer.handle);
    return BAD_VALUE;
  }

  gralloc_buffers_.at(buffer.handle) = true;
  --pending_buffer_count_;

  wait_for_buffer_.notify_one();
  return NO_ERROR;
}

status_t CameraRescalerMemPool::GetFreeOutputBuffer(StreamBuffer* buffer) {

  status_t ret = NO_ERROR;
  std::unique_lock<std::mutex> lock(buffer_lock_);

  buffer->fd = -1;

  if (gralloc_slots_ == nullptr) {
    QMMF_ERROR("%s:%s: Error gralloc slots!", TAG, __func__);
    return NO_ERROR;
  }

  if (pending_buffer_count_ == buffer_cnt_) {
    QMMF_VERBOSE("%s:%s: Already retrieved maximum buffers (%d), waiting"
        " on a free one", TAG, __func__, buffer_cnt_);

    std::chrono::nanoseconds wait_time(kBufferWaitTimeout);
    auto status = wait_for_buffer_.wait_for(lock, wait_time);
    if (status == std::cv_status::timeout) {
      QMMF_ERROR("%s:%s: Wait for output buffer return timed out", TAG,
                 __func__);
      return TIMED_OUT;
    }
  }
  ret = GetBufferLocked(buffer);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Failed to retrieve output buffer", TAG, __func__);
  }

  return ret;
}

status_t CameraRescalerMemPool::GetBufferLocked(StreamBuffer* buffer) {
  status_t ret = NO_ERROR;
  int32_t idx = -1;
  buffer_handle_t handle = nullptr;

  //Only pre-allocate buffers in case no valid stream Buffer
  //is passed as an argument.
  if (nullptr != buffer) {
    for(auto& it : gralloc_buffers_) {
      if(it.second == true) {
        handle = it.first;
        it.second = false;
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
             (buffers_allocated_ < buffer_cnt_)) {
    ret = AllocGrallocBuffer(&handle);
    if (NO_ERROR != ret) {
      return ret;
    }
    idx = buffers_allocated_;
    gralloc_slots_[idx] = handle;
    gralloc_buffers_.emplace(gralloc_slots_[idx], (nullptr == buffer));
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
    ++pending_buffer_count_;
  }

  return ret;
}

status_t CameraRescalerMemPool::PopulateMetaInfo(CameraBufferMetaData &info,
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
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
      info.format = BufferFormat::kNV12;
      info.num_planes = 2;
      info.plane_info[0].width = init_params_.width;
      info.plane_info[0].height = init_params_.height;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      info.plane_info[1].width = init_params_.width;
      info.plane_info[1].height = init_params_.height/2;
      info.plane_info[1].stride = alignedW;
      info.plane_info[1].scanline = alignedH/2;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
      info.format = BufferFormat::kNV12UBWC;
      info.num_planes = 2;
      info.plane_info[0].width = init_params_.width;
      info.plane_info[0].height = init_params_.height;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      info.plane_info[1].width = init_params_.width;
      info.plane_info[1].height = init_params_.height/2;
      info.plane_info[1].stride = alignedW;
      info.plane_info[1].scanline = alignedH/2;
      break;
    case HAL_PIXEL_FORMAT_NV21_ZSL:
      info.format = BufferFormat::kNV21;
      info.num_planes = 2;
      info.plane_info[0].width = init_params_.width;
      info.plane_info[0].height = init_params_.height;
      info.plane_info[0].stride = alignedW;
      info.plane_info[0].scanline = alignedH;
      info.plane_info[1].width = init_params_.width;
      info.plane_info[1].height = init_params_.height/2;
      info.plane_info[1].stride = alignedW;
      info.plane_info[1].scanline = alignedH/2;
      break;
    default:
      QMMF_ERROR("%s:%s: Unsupported format: %d", TAG, __func__,
                 priv_handle->format);
      return NAME_NOT_FOUND;
  }

  return NO_ERROR;
}

status_t CameraRescalerMemPool::AllocGrallocBuffer(buffer_handle_t *buf) {

  status_t ret      = NO_ERROR;
  uint32_t width    = init_params_.width;
  uint32_t height   = init_params_.height;
  int32_t  format   = init_params_.format;
  int32_t usage = 0;

  usage &= GRALLOC_USAGE_ALLOC_MASK;
  usage |= GRALLOC_USAGE_SW_WRITE_OFTEN | GRALLOC_USAGE_SW_READ_OFTEN;
  usage |= GRALLOC_USAGE_HW_FB | private_handle_t::PRIV_FLAGS_VIDEO_ENCODER;

  if (!width || !height) {
    width = height = 1;
  }

  int stride = 0;

  ret = gralloc_device_->alloc(gralloc_device_, static_cast<int>(width),
                               static_cast<int>(height), format,
                               static_cast<int>(usage), buf, &stride);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Failed to allocate gralloc buffer", TAG, __func__);
  }
  return ret;
}

status_t CameraRescalerMemPool::FreeGrallocBuffer(buffer_handle_t buf) {
  return gralloc_device_->free(gralloc_device_, buf);
}

CameraRescaler::CameraRescaler()
  : CameraRescalerBase(),
    is_stop_(false) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  BufferConsumerImpl<CameraRescaler> *impl;
  impl = new BufferConsumerImpl<CameraRescaler>(this);
  buffer_consumer_impl_ = impl;

  BufferProducerImpl<CameraRescaler> *producer_impl;
  producer_impl = new BufferProducerImpl<CameraRescaler>(this);
  buffer_producer_impl_ = producer_impl;

  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

CameraRescaler::~CameraRescaler() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  buffer_producer_impl_.clear();
  buffer_consumer_impl_.clear();
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}


status_t CameraRescaler::AddConsumer(const sp<IBufferConsumer>& consumer) {
  if (consumer.get() == nullptr) {
    QMMF_ERROR("%s:%s: Input consumer is nullptr", TAG, __func__);
    return BAD_VALUE;
  }

  buffer_producer_impl_->AddConsumer(consumer);
  consumer->SetProducerHandle(buffer_producer_impl_);
  QMMF_VERBOSE("%s:%s: Consumer(%p) has been added.", TAG, __func__,
      consumer.get());

  return NO_ERROR;
}

uint32_t CameraRescaler::GetNumConsumer() {
  return buffer_producer_impl_->GetNumConsumer();
}

status_t CameraRescaler::RemoveConsumer(sp<IBufferConsumer>& consumer) {
  if(buffer_producer_impl_->GetNumConsumer() == 0) {
    QMMF_ERROR("%s:%s: There are no connected consumers!", TAG, __func__);
    return INVALID_OPERATION;
  }
  buffer_producer_impl_->RemoveConsumer(consumer);

  return NO_ERROR;
}

sp<IBufferConsumer>& CameraRescaler::GetCopyConsumerIntf() {
  return buffer_consumer_impl_;
}

void CameraRescaler::OnFrameAvailable(StreamBuffer& buffer) {
  QMMF_DEBUG("%s:%s: Camera %u: Frame %d is available", TAG,
      __func__, buffer.camera_id, buffer.frame_number);

  if (IsStop()) {
    QMMF_INFO("%s:%s: IsStop", TAG, __func__);
    return;
  }

  AddBuf(buffer);
}

void CameraRescaler::NotifyBufferReturned(const StreamBuffer& buffer) {
  QMMF_DEBUG("%s:%s: Stream buffer(handle %p) returned", TAG, __func__,
      buffer.handle);
  ReturnBufferToBufferPool(buffer);
}

status_t CameraRescaler::NotifyBufferToClient(StreamBuffer &buffer) {
  status_t ret = NO_ERROR;
  std::lock_guard<std::mutex> lock(consumer_lock_);
  if(buffer_producer_impl_->GetNumConsumer() > 0) {
    buffer_producer_impl_->NotifyBuffer(buffer);
  } else {
    QMMF_DEBUG("%s:%s: No consumer, simply return buffer back to"
        " memory pool!", TAG, __func__);
    ret = ReturnBufferToBufferPool(buffer);
  }
  return ret;
}

status_t CameraRescaler::ReturnBufferToProducer(StreamBuffer &buffer) {
  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  const sp<IBufferConsumer> consumer = buffer_consumer_impl_;
  if (consumer.get() == nullptr) {
    QMMF_ERROR("%s:%s: Failed to retrieve buffer consumer for camera(%d)!",
               TAG, __func__, buffer.camera_id);
    return BAD_VALUE;
  }
  consumer->GetProducerHandle()->NotifyBufferReturned(buffer);

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t CameraRescaler::Start() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  std::lock_guard<std::mutex> lock(stop_lock_);
  is_stop_ = false;
  QMMF_INFO("%s:%s: Start thread", TAG, __func__);
  Run(TAG);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t CameraRescaler::Stop() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  {
    std::lock_guard<std::mutex> lock(stop_lock_);
    is_stop_ = true;
  }
  QMMF_INFO("%s:%s: Stop thread", TAG, __func__);
  RequestExitAndWait();
  QMMF_INFO("%s:%s: Stop thread done", TAG, __func__);
  UnMapBufs();
  QMMF_INFO("%s:%s: unmap done", TAG, __func__);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

bool CameraRescaler::IsStop() {
  std::lock_guard<std::mutex> lock(stop_lock_);
  return is_stop_;
}

status_t CameraRescaler::Init(const VideoTrackParams& track_params) {

  auto ret = Initialize(track_params.params.width,
                        track_params.params.height,
                        HAL_PIXEL_FORMAT_YCbCr_420_888);
  return ret;
}

}; //namespace recorder

}; //namespace qmmf
