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

#define LOG_TAG "Overlay"

#include <utils/Log.h>
#include <algorithm>
#include <fcntl.h>
#include <dirent.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <media/msm_media_info.h>
#include <string.h>
#include <cstring>
#include <string>
#include <assert.h>
#include <sys/time.h>
#include <chrono>
#if USE_SKIA
#include <SkSurface.h>
#include <SkString.h>
#include <SkBitmap.h>
#include <SkBlurMaskFilter.h>
#endif

#include "qmmf-sdk/qmmf_overlay.h"
#include "qmmf_overlay_item.h"

namespace qmmf {

namespace overlay {

using namespace android;

#define ROUND_TO(val, round_to) (val + round_to - 1) & ~(round_to - 1)

Overlay::Overlay()
    : target_c2dsurface_id_(-1), ion_device_(-1),
     id_(0) {
}

Overlay::~Overlay() {

  OVDBG_INFO("%s: Enter ",__func__);
  for (auto &iter : overlay_items_) {
    if (iter.second)
    delete iter.second;
  }
  overlay_items_.clear();

  if(target_c2dsurface_id_) {
    c2dDestroySurface(target_c2dsurface_id_);
    target_c2dsurface_id_ = 0;
    OVDBG_INFO("%s: Destroyed c2d Target Surface", __func__);
  }

  if(ion_device_)
    close(ion_device_);

  OVDBG_INFO("%s: Exit ",__func__);
}

int32_t Overlay::Init(const TargetBufferFormat& format) {

  OVDBG_VERBOSE("%s:Enter",__func__);
  uint32_t c2dColotFormat = GetC2dColorFormat(format);
  // Create dummy C2D surface, it is required to Initialize
  // C2D driver before calling any c2d Apis.
  C2D_YUV_SURFACE_DEF surface_def = {
    c2dColotFormat,
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

  auto ret = c2dCreateSurface(&target_c2dsurface_id_, C2D_TARGET,
                              (C2D_SURFACE_TYPE)(C2D_SURFACE_YUV_HOST
                              |C2D_SURFACE_WITH_PHYS
                              |C2D_SURFACE_WITH_PHYS_DUMMY),
                               &surface_def);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s: c2dCreateSurface failed!",__func__);
    return ret;
  }

  ion_device_ = open("/dev/ion", O_RDONLY);
  if (ion_device_ < 0) {
    OVDBG_ERROR("%s: Ion dev open failed %s\n", __func__,strerror(errno));
    c2dDestroySurface(target_c2dsurface_id_);
    target_c2dsurface_id_ = 0;
    return -1;
  }

  OVDBG_VERBOSE("%s: Exit",__func__);
  return ret;
}

int32_t Overlay::CreateOverlayItem(OverlayParam& param, uint32_t* overlay_id) {

  OVDBG_VERBOSE("%s:Enter ", __func__);
  OverlayItem* overlayItem = nullptr;
  switch(param.type) {
    case OverlayType::kDateType:
      overlayItem = new OverlayItemDateAndTime(ion_device_);
      break;
    case OverlayType::kUserText:
      overlayItem = new OverlayItemText(ion_device_);
      break;
    case OverlayType::kStaticImage:
      overlayItem = new OverlayItemStaticImage(ion_device_);
      break;
    case OverlayType::kBoundingBox:
      overlayItem = new OverlayItemBoundingBox(ion_device_);
      break;
    case OverlayType::kPrivacyMask:
      overlayItem = new OverlayItemPrivacyMask(ion_device_);
      break;
    default:
      OVDBG_ERROR("%s: OverlayType(%d) not supported!", __func__,
           param.type);
      break;
  }

  if(!overlayItem) {
    OVDBG_ERROR("%s: OverlayItem type(%d) failed!", __func__, param.type);
    return NO_INIT;
  }

  auto ret = overlayItem->Init(param);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s:OverlayItem failed of type(%d)", __func__, param.type);
    delete overlayItem;
    return ret;
  }

  // StaticImage type overlayItem never be dirty as its contents are static,
  // all other items are dirty at Init time and will be marked as dirty whenever
  // their configuration changes at run time after first draw.
  if(param.type == OverlayType::kStaticImage) {
    overlayItem->MarkDirty(false);
  } else {
    overlayItem->MarkDirty(true);
  }

  *overlay_id = ++id_;
  overlay_items_.insert({*overlay_id, overlayItem});
  OVDBG_INFO("%s:OverlayItem Type(%d) Id(%d) Created Successfully !",__func__,
      param.type, *overlay_id);

  OVDBG_VERBOSE("%s:Exit ", __func__);
  return ret;
}

int32_t Overlay::DeleteOverlayItem(uint32_t overlay_id) {

  OVDBG_VERBOSE("%s:Enter ", __func__);
  std::lock_guard<std::mutex> lock(lock_);

  int32_t ret = 0;
  if(!IsOverlayItemValid(overlay_id)) {
      OVDBG_ERROR("%s: overlay_id(%d) is not valid!",__func__, overlay_id);
      return BAD_VALUE;
  }
  OverlayItem* overlayItem = overlay_items_.at(overlay_id);
  assert(overlayItem != nullptr);
  delete overlayItem;
  overlay_items_.erase(overlay_id);
  OVDBG_INFO("%s: overlay_id(%d) & overlayItem(0x%p) Removed from map",
      __func__, overlay_id, overlayItem);

  OVDBG_VERBOSE("%s:Exit ", __func__);
  return ret;
}

int32_t Overlay::GetOverlayParams(uint32_t overlay_id,
                                  OverlayParam& param) {
  int32_t ret = 0;
  if(!IsOverlayItemValid(overlay_id)) {
      OVDBG_ERROR("%s: overlay_id(%d) is not valid!",__func__, overlay_id);
      return BAD_VALUE;
  }
  OverlayItem* overlayItem = overlay_items_.at(overlay_id);
  assert(overlayItem != nullptr);

  memset(&param, 0x0, sizeof param);
  overlayItem->GetParameters(param);
  return ret;
}

int32_t Overlay::UpdateOverlayParams(uint32_t overlay_id,
                                     OverlayParam& param) {

  OVDBG_VERBOSE("%s:Enter ", __func__);
  std::lock_guard<std::mutex> lock(lock_);

  if(!IsOverlayItemValid(overlay_id)) {
      OVDBG_ERROR("%s: overlay_id(%d) is not valid!",__func__, overlay_id);
      return BAD_VALUE;
  }
  OverlayItem* overlayItem = overlay_items_.at(overlay_id);
  assert(overlayItem != nullptr);

  OVDBG_VERBOSE("%s:Exit ", __func__);
  return overlayItem->UpdateParameters(param);
}

int32_t Overlay::EnableOverlayItem(uint32_t overlay_id) {

  OVDBG_VERBOSE("%s: Enter", __func__);
  std::lock_guard<std::mutex> lock(lock_);

  int32_t ret = 0;
  if(!IsOverlayItemValid(overlay_id)) {
      OVDBG_ERROR("%s: overlay_id(%d) is not valid!",__func__, overlay_id);
      return BAD_VALUE;
  }
  OverlayItem* overlayItem = overlay_items_.at(overlay_id);
  assert(overlayItem != nullptr);

  overlayItem->Activate(true);
  OVDBG_DEBUG("%s: OverlayItem Id(%d) Activated", __func__, overlay_id);

  OVDBG_VERBOSE("%s: Exit", __func__);
  return ret;
}

int32_t Overlay::DisableOverlayItem(uint32_t overlay_id) {

  OVDBG_VERBOSE("%s: Enter", __func__);
  std::lock_guard<std::mutex> lock(lock_);

  int32_t ret = 0;
  if(!IsOverlayItemValid(overlay_id)) {
      OVDBG_ERROR("%s: overlay_id(%d) is not valid!",__func__, overlay_id);
      return BAD_VALUE;
  }
  OverlayItem* overlayItem = overlay_items_.at(overlay_id);
  assert(overlayItem != nullptr);

  overlayItem->Activate(false);
  OVDBG_DEBUG("%s: OverlayItem Id(%d) DeActivated", __func__, overlay_id);

  OVDBG_VERBOSE("%s: Exit", __func__);
  return ret;
}

int32_t Overlay::ApplyOverlay(const OverlayTargetBuffer& buffer) {

  OVDBG_VERBOSE("%s: Enter", __func__);

#ifdef DEBUG_BLIT_TIME
  auto start_time = ::std::chrono::high_resolution_clock::now();
#endif
  int32_t ret = 0;
  int32_t obj_idx = 0;

  std::lock_guard<std::mutex> lock(lock_);

  size_t numActiveOverlays = 0;
  bool isItemsActive = false;
  for (auto &iter : overlay_items_) {
    if ((iter).second->IsActive()) {
      isItemsActive = true;
    }
  }
  if(!isItemsActive) {
    OVDBG_VERBOSE("%s: No overlayItem is Active!", __func__);
    return ret;
  }
  assert(buffer.ion_fd != 0);
  assert(buffer.width != 0 && buffer.height != 0);
  assert(buffer.frame_len != 0);

  OVDBG_VERBOSE("%s:OverlayTargetBuffer: ion_fd = %d",__func__, buffer.ion_fd);
  OVDBG_VERBOSE("%s:OverlayTargetBuffer: Width = %d & Height = %d & frameLength"
      " =% d", __func__, buffer.width, buffer.height, buffer.frame_len);
  OVDBG_VERBOSE("%s: OverlayTargetBuffer: format = %d", __func__, buffer.format);

  void* bufVaddr = mmap(nullptr, buffer.frame_len, PROT_READ  | PROT_WRITE,
                                              MAP_SHARED, buffer.ion_fd, 0);
  if(!bufVaddr) {
    OVDBG_ERROR("%s: mmap failed!", __func__);
    return UNKNOWN_ERROR;
  }

  // Map input YUV buffer to GPU.
  void *gpuAddr = nullptr;
  ret = c2dMapAddr(buffer.ion_fd, bufVaddr, buffer.frame_len, 0,
                   KGSL_USER_MEM_TYPE_ION, &gpuAddr);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s: c2dMapAddr failed!",__func__);
    goto EXIT;
  }

  // Target surface format.
  C2D_YUV_SURFACE_DEF surface_def;
  surface_def.format  = GetC2dColorFormat(buffer.format);
  surface_def.width   = buffer.width;
  surface_def.height  = buffer.height;
  int32_t planeYLen;
  switch (surface_def.format) {
    case C2D_COLOR_FORMAT_420_NV12:
      //Y plane stride.
      surface_def.stride0 = VENUS_Y_STRIDE(COLOR_FMT_NV12,
              surface_def.width);

      //UV plane stride.
      surface_def.stride1 = VENUS_UV_STRIDE(COLOR_FMT_NV12,
              surface_def.width);

      //UV plane hostptr.
      planeYLen = surface_def.stride0 * VENUS_Y_SCANLINES(COLOR_FMT_NV12,
              surface_def.height);

      break;
    case C2D_COLOR_FORMAT_420_NV21:
      //Y plane stride.
      surface_def.stride0 = VENUS_Y_STRIDE(COLOR_FMT_NV21,
              surface_def.width);

      //UV plane stride.
      surface_def.stride1 = VENUS_UV_STRIDE(COLOR_FMT_NV21,
              surface_def.width);

      //UV plane hostptr.
      planeYLen = surface_def.stride0 * VENUS_Y_SCANLINES(COLOR_FMT_NV21,
              surface_def.height);

      break;
    default:
      OVDBG_ERROR("%s: Unknown format: %d", __func__, surface_def.format);
      goto EXIT;
  }

  OVDBG_DEBUG("%s: surface_def.stride0 = %d ",__func__, surface_def.stride0);
  OVDBG_DEBUG("%s: planeYLen = %d",__func__, planeYLen);

  //Y plane hostptr.
  surface_def.plane0  = (void*)bufVaddr;
  //Y plane Gpu address.
  surface_def.phys0   = (void*)gpuAddr;

  surface_def.plane1  = (void*)((intptr_t)bufVaddr + planeYLen);

  //UV plane Gpu address.
  surface_def.phys1 = (void*)((intptr_t)gpuAddr + planeYLen);

  //Create C2d target surface outof camera buffer. camera buffer
  //is target surface where c2d blits different types of overlays
  //static logo, system time and date.
  ret = c2dUpdateSurface(target_c2dsurface_id_, C2D_SOURCE,
                         (C2D_SURFACE_TYPE)(C2D_SURFACE_YUV_HOST
                         |C2D_SURFACE_WITH_PHYS), &surface_def);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s: c2dUpdateSurface failed!",__func__);
    goto EXIT;
  }

  // Iterate all dirty overlay Items, and update them.
  for (auto &iter : overlay_items_) {
    if ((iter).second->IsActive()) {
      ret = (iter).second->UpdateAndDraw();
      if(ret != 0) {
        OVDBG_ERROR("%s: Update & Draw failed for Item=%d", __func__,
            (iter).first);
      }
    }
  }

  C2dObjects c2d_objects;
  memset(&c2d_objects, 0x0, sizeof c2d_objects);
  // Iterate all updated overlayItems, and get coordinates.
  for (auto &iter : overlay_items_) {
    DrawInfo draw_info;
    memset(&draw_info, 0x0, sizeof draw_info);
    OverlayItem *overlay_item = (iter).second;
    if(overlay_item->IsActive()) {
      overlay_item->GetDrawInfo(buffer.width, buffer.height,
          &draw_info);
      c2d_objects.objects[obj_idx].surface_id  = draw_info.c2dSurfaceId;
      c2d_objects.objects[obj_idx].config_mask = C2D_ALPHA_BLEND_SRC_ATOP
                                           |C2D_TARGET_RECT_BIT;
      c2d_objects.objects[obj_idx].target_rect.x       = draw_info.x << 16;
      c2d_objects.objects[obj_idx].target_rect.y       = draw_info.y << 16;
      c2d_objects.objects[obj_idx].target_rect.width   = draw_info.width << 16;
      c2d_objects.objects[obj_idx].target_rect.height  = draw_info.height << 16;

      OVDBG_VERBOSE("%s: c2d_objects[%d].surface_id=%d", __func__, obj_idx,
          c2d_objects.objects[obj_idx].surface_id);
      OVDBG_VERBOSE("%s: c2d_objects[%d].target_rect.x=%d", __func__, obj_idx,
          draw_info.x);
      OVDBG_VERBOSE("%s: c2d_objects[%d].target_rect.y=%d", __func__, obj_idx,
          draw_info.y);
      OVDBG_VERBOSE("%s: c2d_objects[%d].target_rect.width=%d", __func__,
          obj_idx, draw_info.width);
      OVDBG_VERBOSE("%s: c2d_objects[%d].target_rect.height=%d", __func__,
          obj_idx, draw_info.height);
      ++numActiveOverlays;
      ++obj_idx;
    }
  }

  OVDBG_VERBOSE("%s: numActiveOverlays=%d", __func__, numActiveOverlays);
  for(size_t i = 0; i < (numActiveOverlays-1); i++) {
    c2d_objects.objects[i].next = &c2d_objects.objects[i+1];
  }

  ret = c2dDraw(target_c2dsurface_id_, 0, 0, 0, 0, c2d_objects.objects,
                numActiveOverlays);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s: c2dDraw failed!",__func__);
    goto EXIT;
  }

  ret = c2dFinish(target_c2dsurface_id_);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s: c2dFinish failed!",__func__);
    goto EXIT;
  }
  // Unmap camera buffer from GPU after draw is completed.
  ret = c2dUnMapAddr(gpuAddr);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s: c2dUnMapAddr failed!",__func__);
    goto EXIT;
  }

EXIT:
  if (bufVaddr) {
    munmap(bufVaddr, buffer.frame_len);
  }
#ifdef DEBUG_BLIT_TIME
  auto end_time = ::std::chrono::high_resolution_clock::now();
  auto diff = ::std::chrono::duration_cast<::std::chrono::milliseconds>
                  (end_time - start_time).count();
  OVDBG_INFO("%s: Time taken in 2D draw + Blit=%lld ms", __func__, diff);
#endif
  OVDBG_VERBOSE("%s: Exit ",__func__);
  return ret;
}

uint32_t Overlay::GetC2dColorFormat(const TargetBufferFormat& format) {

  uint32_t c2dColorFormat = C2D_COLOR_FORMAT_420_NV12;
  switch (format) {
    case TargetBufferFormat::kYUVNV12:
      c2dColorFormat = C2D_COLOR_FORMAT_420_NV12;
      break;
    case TargetBufferFormat::kYUVNV21:
      c2dColorFormat = C2D_COLOR_FORMAT_420_NV21;
      break;
    default:
      OVDBG_ERROR("%s: Unsupported buffer format: %d", __func__, format);
      break;
  }
  OVDBG_VERBOSE("%s:Selected C2D ColorFormat=%d",__func__, c2dColorFormat);
  return c2dColorFormat;
}

bool Overlay::IsOverlayItemValid(uint32_t overlay_id) {

  OVDBG_DEBUG("%s: Enter overlay_id(%d)",__func__, overlay_id);
  bool valid = false;
  for (auto& iter : overlay_items_) {
    if (overlay_id == (iter).first) {
      valid = true;
      break;
    }
  }
  OVDBG_DEBUG("%s: Exit overlay_id(%d)",__func__, overlay_id);
  return valid;
}

OverlayItem::OverlayItem(int32_t ion_device)
    :x_(0), y_(0), width_(0), height_(0),
     c2dsurface_id_(-1), gpu_addr_(nullptr),
     vaddr_(nullptr), ion_fd_(0), size_(0),
     dirty_(false), ion_device_(ion_device),
     is_active_(false) {
  OVDBG_VERBOSE("%s:Enter ", __func__);
  memset(&handle_data_, 0x0, sizeof handle_data_);
  location_type_ = OverlayLocationType::kBottomLeft;
#if USE_CAIRO
  cr_surface_ = nullptr;
  cr_context_ = nullptr;
#endif
  OVDBG_VERBOSE("%s:Exit ", __func__);
}

OverlayItem::~OverlayItem() {

  //Unmap overlay gpu address.
  if(gpu_addr_) {
    c2dUnMapAddr(gpu_addr_);
    gpu_addr_ = nullptr;
    OVDBG_INFO("%s: Unmapped GPU address type(%d)", __func__, type_);
  }
  if(vaddr_) {
    munmap(vaddr_, size_);
    vaddr_ = nullptr;
  }
  //Destroy source overlay surface.
  if(c2dsurface_id_) {
    c2dDestroySurface(c2dsurface_id_);
    c2dsurface_id_ = -1;
    OVDBG_INFO("%s: Destroyed c2d Surface type(%d)",__func__, type_);
  }
  //Free overlay ION memory.
  if(ion_fd_) {
    ioctl(ion_device_, ION_IOC_FREE, &handle_data_);
    close(ion_fd_);
    ion_fd_ = -1;
    OVDBG_INFO("%s: Destroyed ION buffer type(%d)",__func__, type_);
  }
#if USE_CAIRO
  if (cr_surface_) {
    cairo_surface_destroy(cr_surface_);
  }
  if (cr_context_) {
    cairo_destroy(cr_context_);
  }
#endif
}

void OverlayItem::MarkDirty(bool dirty) {
  dirty_ = dirty;
  OVDBG_VERBOSE("%s: OverlayItem Type(%d) marked dirty!", __func__, type_);
}

void OverlayItem::Activate(bool value) {
  is_active_ = value;
  OVDBG_VERBOSE("%s: OverlayItem Type(%d) Activated!", __func__, type_);
}

int32_t OverlayItem::AllocateIonMemory(IonMemInfo& mem_info, uint32_t size) {

  OVDBG_VERBOSE("%s:Enter",__func__);
  struct ion_allocation_data alloc;
  struct ion_fd_data ionFdData;
  void *data = nullptr;
  int ionType = 0x1 << ION_IOMMU_HEAP_ID;
  int32_t ret = 0;

  memset(&alloc, 0, sizeof(ion_allocation_data));
  alloc.len = size;
  alloc.len = (alloc.len + 4095) & (~4095);
  alloc.align = 4096;
  alloc.flags = ION_FLAG_CACHED;
  alloc.heap_id_mask = ionType;
  ret = ioctl(ion_device_, ION_IOC_ALLOC, &alloc);
  if (ret < 0) {
    OVDBG_ERROR("%s:ION allocation failed\n",__func__);
    goto ION_ALLOC_FAILED;
  }

  memset(&ionFdData, 0, sizeof(ion_fd_data));
  ionFdData.handle = alloc.handle;
  ret = ioctl(ion_device_, ION_IOC_SHARE, &ionFdData);
  if (ret < 0) {
    OVDBG_ERROR("%s:ION map failed %s\n",__func__,strerror(errno));
    goto ION_MAP_FAILED;
  }

  data = mmap(nullptr, alloc.len, PROT_READ | PROT_WRITE, MAP_SHARED,
              ionFdData.fd, 0);

  if (data == MAP_FAILED) {
    OVDBG_ERROR("%s:ION mmap failed: %s (%d)\n",__func__, strerror(errno),
        errno);
    goto ION_MAP_FAILED;
  }

  memset(&mem_info.handle_data, 0, sizeof(mem_info.handle_data));
  mem_info.handle_data.handle = ionFdData.handle;
  mem_info.fd                 = ionFdData.fd;
  mem_info.size               = alloc.len;
  mem_info.vaddr              = data;

  OVDBG_VERBOSE("%s:Exit ",__func__);
  return ret;

ION_MAP_FAILED:
  memset(&mem_info.handle_data, 0, sizeof(mem_info.handle_data));
  mem_info.handle_data.handle = ionFdData.handle;
  ioctl(ion_device_, ION_IOC_FREE, &mem_info.handle_data);
ION_ALLOC_FAILED:
  close(ion_device_);
  return -1;
}

void OverlayItem::ExtractColorValues(uint32_t hex_color, RGBAValues* color) {

  color->red   = ((hex_color >> 24) & 0xff) / 255.0;
  color->green = ((hex_color >> 16) & 0xff) / 255.0;
  color->blue  = ((hex_color >> 8) & 0xff) / 255.0;
  color->alpha = ((hex_color) & 0xff) / 255.0;
}

void OverlayItem::ClearSurface() {

#if USE_CAIRO
  RGBAValues bg_color;
  memset(&bg_color, 0x0, sizeof bg_color);
  // Painting entire surface with background color or with fully transparent
  // color doesn't work since cairo uses the OVER compositing operator
  // by default, and blending something entirely transparent OVER something
  // else has no effect at all until compositing operator is changed to SOURCE,
  // the SOURCE operator copies both color and alpha values directly from the
  // source to the destination instead of blending.
#ifdef DEBUG_BACKGROUND_SURFACE
  ExtractColorValues(BG_DEBUG_COLOR, &bg_color);
  cairo_set_source_rgba(cr_context_, bg_color.red, bg_color.green,
                        bg_color.blue, bg_color.alpha);
  cairo_set_operator(cr_context_, CAIRO_OPERATOR_SOURCE);
#else
  cairo_set_operator(cr_context_, CAIRO_OPERATOR_CLEAR);
#endif
  cairo_paint(cr_context_);
  cairo_surface_flush(cr_surface_);
  cairo_set_operator(cr_context_, CAIRO_OPERATOR_OVER);
  assert(CAIRO_STATUS_SUCCESS == cairo_status(cr_context_));
  // After flush, atleast 5ms is required to avoid flickers.
  usleep(5000);
#endif
}

OverlayItemStaticImage::OverlayItemStaticImage(int32_t ion_device)
    :OverlayItem(ion_device), image_path_() {
  OVDBG_VERBOSE("%s: Enter", __func__);
  type_ = OverlayType::kStaticImage;
  OVDBG_VERBOSE("%s: Exit", __func__);
}

OverlayItemStaticImage::~OverlayItemStaticImage() {
  OVDBG_VERBOSE("%s: Enter", __func__);
  image_path_.clear();
  OVDBG_VERBOSE("%s: Exit", __func__);
}

void OverlayItemStaticImage::DestroySurface() {
  //Unmap overlay gpu address.
  if(gpu_addr_) {
    c2dUnMapAddr(gpu_addr_);
    gpu_addr_ = nullptr;
    OVDBG_INFO("%s: Unmapped GPU address type(%d)", __func__, type_);
  }
  if(vaddr_) {
    munmap(vaddr_, size_);
    vaddr_ = nullptr;
  }
  //Destroy source overlay surface.
  if(c2dsurface_id_) {
    c2dDestroySurface(c2dsurface_id_);
    c2dsurface_id_ = -1;
    OVDBG_INFO("%s: Destroyed c2d Surface type(%d)",__func__, type_);
  }
  //Free overlay ION memory.
  if(ion_fd_) {
    ioctl(ion_device_, ION_IOC_FREE, &handle_data_);
    close(ion_fd_);
    ion_fd_ = -1;
    OVDBG_INFO("%s: Destroyed ION buffer type(%d)",__func__, type_);
  }
}

int32_t OverlayItemStaticImage::Init(OverlayParam& param) {

  OVDBG_VERBOSE("%s: Enter", __func__);
  int32_t ret = 0;

  if(param.dst_rect.width <= 0 || param.dst_rect.height <= 0) {
    OVDBG_ERROR("%s: Image Width & Height is not correct!", __func__);
    return BAD_VALUE;
  }

  location_type_ = param.location;
  x_             = param.dst_rect.start_x;
  y_             = param.dst_rect.start_y;
  width_         = param.dst_rect.width;
  height_        = param.dst_rect.height;
  image_type_    = param.image_info.image_type;

  if (param.image_info.image_type == OverlayImageType::kFilePath) {

    image_path_.setTo(param.image_info.image_location,
        strlen(param.image_info.image_location) + 1);
  } else if (param.image_info.image_type == OverlayImageType::kBlobType) {

    image_buffer_  = param.image_info.image_buffer;
    image_size_    = param.image_info.image_size;
    image_width_   = param.image_info.source_rect.width;
    image_height_  = param.image_info.source_rect.height;
    OVDBG_VERBOSE("%s: image blob  image_buffer_::0x%p  image_size_::%u "
        "image_width_::%u image_height_::%u ",
        __func__, image_buffer_, image_size_, image_width_, image_height_);

    char prop_val[PROPERTY_VALUE_MAX];
    property_get(PROP_DUMP_BLOB_IMAGE, prop_val, "0");
    blob_image_dump_enabled_ = (atoi(prop_val) == 0) ? false : true;

    if (blob_image_dump_enabled_) {
      FILE* pFile;
      pFile = fopen("/data/misc/qmmf/overlay_image_blob.rgb","wb");
      if (pFile ){
        fwrite(image_buffer_, sizeof(char), image_size_, pFile);
      }
      fclose(pFile);
    }

    crop_rect_x_      = param.image_info.source_rect.start_x;
    crop_rect_y_      = param.image_info.source_rect.start_y;
    crop_rect_width_  = param.image_info.source_rect.width;
    crop_rect_height_ = param.image_info.source_rect.height;
    OVDBG_VERBOSE("%s: image blob  crop_rect_x_::%u  crop_rect_y_::%u "
        "crop_rect_width_::%u  crop_rect_height_::%u",
        __func__, crop_rect_x_, crop_rect_y_,crop_rect_width_, crop_rect_height_);
  }

  ret = CreateSurface();
  if(ret != 0) {
    OVDBG_ERROR("%s: createLogoSurface failed!", __func__);
    return ret;
  }
  OVDBG_VERBOSE("%s: Exit", __func__);
  return ret;
}

int32_t OverlayItemStaticImage::UpdateAndDraw() {
  // Nothing to update, contents are static.
  // Never marked as dirty.
  std::lock_guard<std::mutex> lock(update_param_lock_);
  if (blob_buffer_updated_) {
    c2dSurfaceUpdated(c2dsurface_id_, nullptr);
  }
  return OK;
}

void OverlayItemStaticImage::GetDrawInfo(uint32_t targetWidth,
                                         uint32_t targetHeight,
                                         DrawInfo* draw_info) {

  OVDBG_VERBOSE("%s: Enter", __func__);
  draw_info->width  = width_;
  draw_info->height = height_;
  int32_t xMargin = targetWidth * OVERLAYITEM_X_MARGIN_PERCENT/100;
  int32_t yMargin = targetHeight * OVERLAYITEM_Y_MARGIN_PERCENT/100;
  int32_t x = 0;
  int32_t y = 0;

  switch (location_type_) {
    case OverlayLocationType::kTopLeft:
      x = xMargin;
      y = yMargin;
      break;
    case OverlayLocationType::kTopRight:
      x = targetWidth - (width_ + xMargin);
      y = yMargin;
      break;
    case OverlayLocationType::kCenter:
      x = (targetWidth - width_)/2;
      y = (targetHeight - height_)/2;
      break;
    case OverlayLocationType::kBottomLeft:
      x = xMargin;
      y = targetHeight - (height_ + yMargin);
      break;
    case OverlayLocationType::kBottomRight:
      x = targetWidth - (width_ + xMargin);
      y = targetHeight - (height_ + yMargin);
      break;
    case OverlayLocationType::kRandom:
      x = x_;
      y = y_;
      break;
    case OverlayLocationType::kNone:
    default:
      x = x_;
      y = y_;
      break;
  }
  draw_info->x            = x;
  draw_info->y            = y;
  draw_info->c2dSurfaceId = c2dsurface_id_;

  OVDBG_VERBOSE("%s: Exit", __func__);
}

void OverlayItemStaticImage::GetParameters(OverlayParam& param) {

  OVDBG_VERBOSE("%s:Enter ",__func__);
  param.type              = OverlayType::kStaticImage;
  param.location          = location_type_;
  param.dst_rect.start_x  = x_;
  param.dst_rect.start_y  = y_;
  param.dst_rect.width    = width_;
  param.dst_rect.height   = height_;
  std::string str(image_path_.string());
  str.copy(param.image_info.image_location, image_path_.length());
  OVDBG_VERBOSE("%s:Exit ",__func__);
}

int32_t OverlayItemStaticImage::UpdateParameters(OverlayParam& param) {

  OVDBG_VERBOSE("%s:Enter ",__func__);
  std::lock_guard<std::mutex> lock(update_param_lock_);
  int32_t ret = 0;

  if(strcmp(image_path_.string(), param.image_info.image_location) != 0) {
    OVDBG_ERROR("%s: Image Path Can't be changed at run time!!", __func__);
    return BAD_VALUE;
  }

  if(param.dst_rect.width <= 0 || param.dst_rect.height <= 0) {
    OVDBG_ERROR("%s: Image Width & Height is not correct!", __func__);
    return BAD_VALUE;
  }

  location_type_ = param.location;
  x_             = param.dst_rect.start_x;
  y_             = param.dst_rect.start_y;
  width_         = param.dst_rect.width;
  height_        = param.dst_rect.height;

  if (image_type_ == OverlayImageType::kBlobType) {

    image_buffer_  = param.image_info.image_buffer;
    image_width_   = param.image_info.source_rect.width;
    image_height_  = param.image_info.source_rect.height;
    OVDBG_DEBUG("%s: updated image blob  image_buffer_::0x%p image_size_::%u "
        "image_width_::%u image_height_::%u ",
        __func__, image_buffer_, param.image_info.image_size,
        image_width_, image_height_);

    crop_rect_x_      = param.image_info.source_rect.start_x;
    crop_rect_y_      = param.image_info.source_rect.start_y;
    crop_rect_width_  = param.image_info.source_rect.width;
    crop_rect_height_ = param.image_info.source_rect.height;
    OVDBG_DEBUG("%s: updated image blob  crop_rect_x_::%u crop_rect_y_::%u "
        "crop_rect_width_::%u  crop_rect_height_::%u",
        __func__, crop_rect_x_, crop_rect_y_,crop_rect_width_, crop_rect_height_);

    if (blob_image_dump_enabled_) {
      String8 blobbuffer_filepath;
      struct timeval tv;
      gettimeofday(&tv, nullptr);

      blobbuffer_filepath.appendFormat("/data/misc/qmmf/overlay_blob_buffer_%lu.%s",
          tv.tv_sec, "rgb");

      blob_buffer_file_fd_ = open(blobbuffer_filepath.string(), O_CREAT |
          O_WRONLY | O_TRUNC, 0655);
      assert(blob_buffer_file_fd_ >= 0);

      uint32_t bytes_written;
      bytes_written  = write(blob_buffer_file_fd_, image_buffer_,
          param.image_info.image_size);

      if (bytes_written != param.image_info.image_size) {
        OVDBG_ERROR("Bytes written != %d and written = %u", bytes_written,
            param.image_info.image_size);
      }
      close(blob_buffer_file_fd_);
    }

    // only buffer content is changed not buffer size
    if (param.image_info.buffer_updated &&
        (param.image_info.image_size == image_size_)) {
      OVDBG_DEBUG("%s: updated image_size_:: %u param.image_info.image_size:: %u ",
          __func__, image_size_, param.image_info.image_size);
      uint32_t size = param.image_info.image_size;
      uint32_t* pixels = static_cast<uint32_t*>(vaddr_);
      memcpy(pixels, image_buffer_, size);
      blob_buffer_updated_ = param.image_info.buffer_updated;
      MarkDirty(true);
    } else if (param.image_info.image_size != image_size_) {

      image_size_ = param.image_info.image_size;
      DestroySurface();
      ret = CreateSurface();
      if (ret != 0) {
        OVDBG_ERROR("%s: CreateSurface failed!", __func__);
        return ret;
      }
    }
    image_size_= param.image_info.image_size;
  }

  OVDBG_VERBOSE("%s:Exit ",__func__);
  return ret;
}

int32_t OverlayItemStaticImage::CreateSurface() {

  OVDBG_VERBOSE("%s:Enter ",__func__);
  int32_t   ret = 0;
  uint32_t size;
  IonMemInfo mem_info;
  memset(&mem_info, 0x0, sizeof(IonMemInfo));

  if (image_type_ == OverlayImageType::kFilePath)  {

    size = width_ * height_ * 4;
    ret = AllocateIonMemory(mem_info, size);
    if(0 != ret) {
      OVDBG_ERROR("%s:AllocateIonMemory failed",__func__);
      return ret;
    }
    uint32_t* pixels = (uint32_t*)mem_info.vaddr;

    //Load raw logo image file.
    FILE *file = 0;
    size_t bytes;

    file = fopen(image_path_.string(), "rb");
    if(file) {
      bytes = fread(pixels, 1, size, file);
      OVDBG_INFO("%s: Total btyes = %d",__func__,bytes);
      if(bytes != size) {
        OVDBG_ERROR("%s: Raw file format is not correct",__func__);
        fclose(file);
        goto ERROR;
      }
      fclose(file);
    } else {
      OVDBG_ERROR("%s: (%s)File open Failed!!",__func__, image_path_.string());
      goto ERROR;
    }
  } else if(image_type_ == OverlayImageType::kBlobType){

    size = image_size_;
    ret = AllocateIonMemory(mem_info, size);
    if(0 != ret) {
      OVDBG_ERROR("%s:AllocateIonMemory failed",__func__);
      return ret;
    }
    uint32_t* pixels = static_cast<uint32_t*>(mem_info.vaddr);
    memcpy(pixels, image_buffer_, size);
  }

  //Map ARGB ION buffer to GPU.
  ret = c2dMapAddr(mem_info.fd, mem_info.vaddr, mem_info.size, 0,
                   KGSL_USER_MEM_TYPE_ION, &gpu_addr_);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s: c2dMapAddr failed!",__func__);
    goto ERROR;
  }

  C2D_RGB_SURFACE_DEF c2dSurfaceDef;
  c2dSurfaceDef.format = C2D_FORMAT_SWAP_ENDIANNESS| C2D_COLOR_FORMAT_8888_RGBA;
  c2dSurfaceDef.width  = width_;
  c2dSurfaceDef.height = height_;
  c2dSurfaceDef.buffer = mem_info.vaddr;
  c2dSurfaceDef.phys   = gpu_addr_;
  c2dSurfaceDef.stride = width_ * 4;

  //Create source c2d surface.
  ret = c2dCreateSurface(&c2dsurface_id_, C2D_SOURCE,
                         (C2D_SURFACE_TYPE)(C2D_SURFACE_RGB_HOST
                         |C2D_SURFACE_WITH_PHYS), &c2dSurfaceDef);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s: c2dCreateSurface failed!",__func__);
    goto ERROR;
  }
  ion_fd_      = mem_info.fd;
  vaddr_       = mem_info.vaddr;
  size_        = mem_info.size;
  handle_data_ = mem_info.handle_data;

  OVDBG_VERBOSE("%s: Exit ",__func__);
  return ret;
ERROR:
  ioctl(ion_device_, ION_IOC_FREE, &handle_data_);
  close(ion_fd_);
  ion_fd_ = -1;
  return ret;
}

OverlayItemDateAndTime::OverlayItemDateAndTime(int32_t ion_device)
    :OverlayItem(ion_device) {
  OVDBG_VERBOSE("%s:Enter ", __func__);
  memset(&date_time_type_, 0x0, sizeof date_time_type_);
  date_time_type_.time_format = OverlayTimeFormatType::kHHMM_24HR;
  date_time_type_.date_format = OverlayDateFormatType::kMMDDYYYY;
  type_                       = OverlayType::kDateType;
  OVDBG_VERBOSE("%s:Exit", __func__);
}

OverlayItemDateAndTime::~OverlayItemDateAndTime() {
  OVDBG_VERBOSE("%s:Enter ", __func__);
  OVDBG_VERBOSE("%s:Exit ", __func__);
}

int32_t OverlayItemDateAndTime::Init(OverlayParam& param) {

  OVDBG_VERBOSE("%s: Enter", __func__);
  location_type_ = param.location;
  text_color_    = param.color;
  x_             = param.dst_rect.start_x;
  y_             = param.dst_rect.start_y;
  width_         = param.dst_rect.width;
  height_        = param.dst_rect.height;

  date_time_type_.date_format = param.date_time.date_format;
  date_time_type_.time_format = param.date_time.time_format;

  if (param.dst_rect.width == 0 || param.dst_rect.height == 0) {
    width_  = DATETIME_TEXT_BUF_WIDTH;
    height_ = DATETIME_TEXT_BUF_HEIGHT;
  }
  auto ret = CreateSurface();
  if(ret != 0) {
    OVDBG_ERROR("%s: createLogoSurface failed!", __func__);
    return ret;
  }
  OVDBG_VERBOSE("%s: Exit", __func__);
  return ret;
}

int32_t OverlayItemDateAndTime::UpdateAndDraw() {

  OVDBG_VERBOSE("%s: Enter", __func__);
  int32_t ret = 0;
  if(!dirty_)
      return ret;

  struct timeval tv;
  time_t now_time;
  struct tm *time;
  char date_buf[40];
  char time_buf[40];

  gettimeofday(&tv, nullptr);
  now_time = tv.tv_sec;
  time = localtime(&now_time);

  switch(date_time_type_.date_format) {
    case OverlayDateFormatType::kYYYYMMDD:
      strftime(date_buf, sizeof date_buf, "%Y/%m/%d", time);
      break;
    case OverlayDateFormatType::kMMDDYYYY:
    default:
      strftime(date_buf, sizeof date_buf, "%m/%d/%Y", time);
      break;
  }
  switch(date_time_type_.time_format) {
    case OverlayTimeFormatType::kHHMMSS_24HR:
      strftime(time_buf, sizeof time_buf, "%H:%M:%S", time);
      break;
    case OverlayTimeFormatType::kHHMMSS_AMPM:
      strftime(time_buf, sizeof time_buf, "%r", time);
      break;
    case OverlayTimeFormatType::kHHMM_24HR:
      strftime(time_buf, sizeof time_buf, "%H:%M", time);
      break;
    case OverlayTimeFormatType::kHHMM_AMPM:
    default:
      strftime(time_buf, sizeof time_buf, "%I:%M %p", time);
      break;
  }
  OVDBG_VERBOSE("%s: date:time (%s:%s)", __func__, date_buf, time_buf);

  double x_date, x_time, y_date, y_time;
  x_date = x_time = y_date = y_time = 0.0;

#if USE_CAIRO
  // Clear the privous drawn contents.
  ClearSurface();
  cairo_select_font_face(cr_context_, "@cairo:Georgia", CAIRO_FONT_SLANT_NORMAL,
                          CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size (cr_context_, DATETIME_PIXEL_SIZE);
  cairo_set_antialias (cr_context_, CAIRO_ANTIALIAS_BEST);
  assert(CAIRO_STATUS_SUCCESS == cairo_status(cr_context_));

  cairo_font_extents_t font_extent;
  cairo_font_extents (cr_context_, &font_extent);
  OVDBG_VERBOSE("%s: ascent=%f, descent=%f, height=%f, max_x_advance=%f,"
      " max_y_advance = %f", __func__, font_extent.ascent, font_extent.descent,
       font_extent.height, font_extent.max_x_advance,
       font_extent.max_y_advance);

  cairo_text_extents_t date_text_extents;
  cairo_text_extents (cr_context_, date_buf, &date_text_extents);

  OVDBG_VERBOSE("%s: Date: te.x_bearing=%f, te.y_bearing=%f, te.width=%f,"
      " te.height=%f, te.x_advance=%f, te.y_advance=%f", __func__,
      date_text_extents.x_bearing, date_text_extents.y_bearing,
      date_text_extents.width, date_text_extents.height,
      date_text_extents.x_advance, date_text_extents.y_advance);

  cairo_font_options_t *options;
  options = cairo_font_options_create ();
  cairo_font_options_set_antialias (options, CAIRO_ANTIALIAS_DEFAULT);
  cairo_set_font_options (cr_context_, options);
  cairo_font_options_destroy (options);

  //(0,0) is at topleft corner of draw buffer.
  y_date = height_/2.0; // height is buffer height.
  y_date = std::max(y_date, date_text_extents.height - (font_extent.descent/2.0));
  OVDBG_VERBOSE("%s: x_date=%f, y_date=%f, ref=%f", __func__, x_date, y_date,
      date_text_extents.height - (font_extent.descent/2.0));
  cairo_move_to (cr_context_, x_date, y_date);

  // Draw date.
  RGBAValues text_color;
  memset(&text_color, 0x0, sizeof text_color);
  ExtractColorValues(text_color_, &text_color);
  cairo_set_source_rgba (cr_context_, text_color.red, text_color.green,
                         text_color.blue, text_color.alpha);

  cairo_show_text (cr_context_, date_buf);
  assert(CAIRO_STATUS_SUCCESS == cairo_status(cr_context_));

  // Draw time.
  cairo_text_extents_t time_text_extents;
  cairo_text_extents (cr_context_, time_buf, &time_text_extents);
  OVDBG_VERBOSE("%s: Time: te.x_bearing=%f, te.y_bearing=%f, te.width=%f,"
      " te.height=%f, te.x_advance=%f, te.y_advance=%f", __func__,
      time_text_extents.x_bearing, time_text_extents.y_bearing,
      time_text_extents.width, time_text_extents.height,
      time_text_extents.x_advance, time_text_extents.y_advance);
  // Calculate the x_time to draw the time text extact middle of buffer.
  // Use x_width which usally few pixel less than the width of the actual
  // drawn text.
  x_time = (width_ - time_text_extents.width)/2.0; // width_ is buffer width.
  y_time = y_date + (date_text_extents.height - (font_extent.descent/2));
  cairo_move_to (cr_context_, x_time, y_time);
  cairo_show_text (cr_context_, time_buf);
  assert(CAIRO_STATUS_SUCCESS == cairo_status(cr_context_));

  cairo_surface_flush(cr_surface_);
  // After flush, atleast 5ms is required to avoid flickers.
  usleep(5000);

#elif USE_SKIA

#ifndef DEBUG_BACKGROUND_SURFACE
  canvas_->clear(SK_AlphaOPAQUE);
#else
  canvas_->clear(SK_ColorDKGRAY);
#endif
  int32_t date_len = strlen(date_buf);
  int32_t time_len = strlen(time_buf);

  SkPaint paint;
  paint.setColor(text_color_);
  paint.setTextSize(SkIntToScalar(DATETIME_PIXEL_SIZE));
  paint.setAntiAlias(true);
  paint.setTextScaleX(1);

  SkString dateText(date_buf, date_len);
  canvas_->drawText(dateText.c_str(), dateText.size(), x_date, y_date, paint);

  SkString timeText(time_buf, time_len);
  int32_t perCharSize = DATETIME_TEXT_BUF_WIDTH/dateText.size();
  int32_t xTime = (DATETIME_TEXT_BUF_WIDTH - (timeText.size() * perCharSize));
  xTime = xTime > 0 ? (xTime) : 0;
  int32_t yTime = DATETIME_TEXT_BUF_HEIGHT - DATETIME_PIXEL_SIZE/2;
  canvas_->drawText(timeText.c_str(), timeText.size(), xTime, yTime, paint);
  canvas_->flush();
  usleep(1000);
#endif

  MarkDirty(true);
  OVDBG_VERBOSE("%s: Exit", __func__);
  return ret;
}

void OverlayItemDateAndTime::GetDrawInfo(uint32_t targetWidth,
                                         uint32_t targetHeight,
                                         DrawInfo* draw_info) {

  OVDBG_VERBOSE("%s:Enter ",__func__);
  draw_info->width  = targetWidth * DATETIME_TARGET_WIDTH_PERCENT/100;
  draw_info->height = targetHeight * DATETIME_TARGET_HEIGHT_PERCENT/100;

  int32_t xMargin = targetWidth * OVERLAYITEM_X_MARGIN_PERCENT/100;
  int32_t yMargin = targetHeight * OVERLAYITEM_Y_MARGIN_PERCENT/100;
  int32_t x = 0;
  int32_t y = 0;

  //(0,0) is at topleft corner.
  switch (location_type_) {
    case OverlayLocationType::kTopLeft:
      x = xMargin;
      y = yMargin;
      break;
    case OverlayLocationType::kTopRight:
      x = targetWidth - (draw_info->width + xMargin);
      y = yMargin;
      break;
    case OverlayLocationType::kCenter:
      x = (targetWidth - draw_info->width)/2;
      y = (targetHeight - draw_info->height)/2;
      break;
    case OverlayLocationType::kBottomLeft:
      x = xMargin;
      y = targetHeight - (draw_info->height + yMargin);
      break;
    case OverlayLocationType::kBottomRight:
      x = targetWidth - (draw_info->width + xMargin);
      y = targetHeight - (draw_info->height + yMargin);
      break;
    case OverlayLocationType::kRandom:
      x = x_;
      y = y_;
      break;
    case OverlayLocationType::kNone:
    default:
      break;
  }
  draw_info->x            = x;
  draw_info->y            = y;
  draw_info->c2dSurfaceId = c2dsurface_id_;
  OVDBG_VERBOSE("%s:Exit ",__func__);
}

void OverlayItemDateAndTime::GetParameters(OverlayParam& param) {

  OVDBG_VERBOSE("%s:Enter ",__func__);
  param.type             = OverlayType::kDateType;
  param.location         = location_type_;
  param.color            = text_color_;
  param.dst_rect.start_x = x_;
  param.dst_rect.start_y = y_;
  param.dst_rect.width   = width_;
  param.dst_rect.height  = height_;
  param.date_time.date_format = date_time_type_.date_format;
  param.date_time.time_format = date_time_type_.time_format;
  OVDBG_VERBOSE("%s:Exit ",__func__);
}

int32_t OverlayItemDateAndTime::UpdateParameters(OverlayParam& param) {

  OVDBG_VERBOSE("%s:Enter ",__func__);
  int32_t ret = 0;
  location_type_ = param.location;
  text_color_    = param.color;
  x_             = param.dst_rect.start_x;
  y_             = param.dst_rect.start_y;
  width_         = param.dst_rect.width;
  height_        = param.dst_rect.height;

  date_time_type_.date_format = param.date_time.date_format;
  date_time_type_.time_format = param.date_time.time_format;
  OVDBG_VERBOSE("%s:Exit ",__func__);
  return ret;
}

int32_t OverlayItemDateAndTime::CreateSurface() {

  OVDBG_VERBOSE("%s: Enter", __func__);
  int32_t ret = 0;
  int32_t size = width_ * height_ * 4;
  IonMemInfo mem_info;
  memset(&mem_info, 0x0, sizeof(IonMemInfo));

  ret = AllocateIonMemory(mem_info, size);
  if(0 != ret) {
    OVDBG_ERROR("%s:AllocateIonMemory failed",__func__);
    return ret;
  }
  OVDBG_INFO("%s: ION memory allocated fd = %d",__func__,mem_info.fd);

#if USE_CAIRO
  cr_surface_ = cairo_image_surface_create_for_data(static_cast<unsigned char*>
                                                    (mem_info.vaddr),
                                                    CAIRO_FORMAT_ARGB32, width_,
                                                    height_, width_ * 4);
  assert (cr_surface_ != nullptr);

  cr_context_ = cairo_create (cr_surface_);
  assert (cr_context_ != nullptr);

#elif USE_SKIA
  //Create Skia canvas outof ION memory.
  SkImageInfo imageInfo;
  imageInfo.Make(width_, height_, kRGBA_8888_SkColorType, kPremul_SkAlphaType);

  canvas_ = SkCanvas::NewRasterDirect(imageInfo, mem_info.vaddr,
                                      width_ *4);
  if(!canvas_) {
    OVDBG_ERROR("%s: Skia Creation failed!!",__func__);
    goto ERROR;
  }
#endif
  //Draw system time on Skia canvas.
  UpdateAndDraw();

  //Setup c2d.
  ret = c2dMapAddr(mem_info.fd, mem_info.vaddr, mem_info.size,
                     0, KGSL_USER_MEM_TYPE_ION, &gpu_addr_);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s: c2dMapAddr failed!",__func__);
    goto ERROR;
  }

  C2D_RGB_SURFACE_DEF c2dSurfaceDef;
#if USE_CAIRO
  c2dSurfaceDef.format = C2D_COLOR_FORMAT_8888_ARGB;
#elif USE_SKIA
  c2dSurfaceDef.format = C2D_FORMAT_SWAP_ENDIANNESS| C2D_COLOR_FORMAT_8888_RGBA;
#endif
  c2dSurfaceDef.width  = width_;
  c2dSurfaceDef.height = height_;
  c2dSurfaceDef.buffer = mem_info.vaddr;
  c2dSurfaceDef.phys   = gpu_addr_;
  c2dSurfaceDef.stride = width_ * 4;

  //Create source c2d surface.
  ret = c2dCreateSurface(&c2dsurface_id_, C2D_SOURCE,
                         (C2D_SURFACE_TYPE)(C2D_SURFACE_RGB_HOST
                         |C2D_SURFACE_WITH_PHYS), &c2dSurfaceDef);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s: c2dCreateSurface failed!",__func__);
    goto ERROR;
  }

  ion_fd_      = mem_info.fd;
  vaddr_       = mem_info.vaddr;
  size_        = mem_info.size;
  handle_data_ = mem_info.handle_data;

  OVDBG_VERBOSE("%s: Exit", __func__);
  return ret;
ERROR:
  ioctl(ion_device_, ION_IOC_FREE, &handle_data_);
  close(ion_fd_);
  ion_fd_ = -1;
  return ret;
}

OverlayItemBoundingBox::OverlayItemBoundingBox(int32_t ion_device)
    :OverlayItem(ion_device), bbox_name_(), text_height_(0) {
  OVDBG_INFO("%s: Enter", __func__);
  type_ = OverlayType::kBoundingBox;
  OVDBG_INFO("%s: Exit", __func__);
}

OverlayItemBoundingBox::~OverlayItemBoundingBox() {
  OVDBG_INFO("%s: Enter", __func__);
  bbox_name_.clear();
  OVDBG_INFO("%s: Exit", __func__);
}

int32_t OverlayItemBoundingBox::Init(OverlayParam& param) {

  OVDBG_VERBOSE("%s: Enter", __func__);
  if ((param.dst_rect.width <= 0) || (param.dst_rect.height <= 0)) {
    return BAD_VALUE;
  }
  if (param.dst_rect.start_x < 0 || param.dst_rect.start_y < 0) {
    return BAD_VALUE;
  }

  x_          = param.dst_rect.start_x;
  y_          = param.dst_rect.start_y;
  width_      = param.dst_rect.width;
  height_     = param.dst_rect.height;
  bbox_color_ = param.color;

  float scaled_width  = static_cast<float>(width_) / DOWNSCALE_FACTOR;
  float scaled_height = static_cast<float>(height_) / DOWNSCALE_FACTOR;

  float aspect_ratio = scaled_width / scaled_height;

  OVDBG_INFO("%s: BoundingBox(W:%dxH:%d), aspect_ratio(%f), scaled(W:%fxH:%f)",
      __func__, param.dst_rect.width, param.dst_rect.height,
      aspect_ratio, scaled_width, scaled_height);

  int32_t width = static_cast<int32_t>(round(scaled_width));
  width = ROUND_TO(width, 16); // Round to multiple of 16.
  width = width > BOUNDING_BOX_BUF_WIDTH ? width : BOUNDING_BOX_BUF_WIDTH;
  int32_t height = static_cast<int32_t>(round(width / aspect_ratio));

  buffer_width_  = width;
  buffer_height_ = height;

  OVDBG_INFO("%s: Offscreen buffer:(%dx%d)",__func__, buffer_width_,
      buffer_height_);

  int32_t textLen = strlen(param.bounding_box.box_name);

  int32_t textLimit = std::min(textLen + 1, BOUNDING_BOX_TEXT_LIMIT);
  bbox_name_.setTo(param.bounding_box.box_name, textLimit);
  auto ret = CreateSurface();
  if (ret != 0) {
    OVDBG_ERROR("%s: CreateSurface failed!", __func__);
    return NO_INIT;
  }
  OVDBG_VERBOSE("%s: Exit", __func__);
  return ret;
}

int32_t OverlayItemBoundingBox::UpdateAndDraw() {

  OVDBG_VERBOSE("%s: Enter ", __func__);
  int32_t ret = 0;

  if(!dirty_) {
    OVDBG_DEBUG("%s: Item is not dirty! Don't draw!", __func__);
    return ret;
  }
  //  Bounding Box and text are drawn on same buffer.
  //  ----------
  //  | TEXT   |
  //  ----------
  //  |        |
  //  |  BOX   |
  //  |        |
  //  ----------

#if USE_CAIRO
  OVDBG_INFO("%s: Draw bounding box and text!", __func__);
  ClearSurface();
  // Draw text first.
  cairo_select_font_face(cr_context_, "@cairo:Georgia", CAIRO_FONT_SLANT_NORMAL,
                         CAIRO_FONT_WEIGHT_BOLD);

  cairo_set_font_size (cr_context_, BOUNDING_BOX_TEXT_SIZE);
  cairo_set_antialias(cr_context_, CAIRO_ANTIALIAS_BEST);

  cairo_font_extents_t font_extents;
  cairo_font_extents (cr_context_, &font_extents);
  OVDBG_VERBOSE("%s: BBox Font: ascent=%f, descent=%f, height=%f, "
      "max_x_advance=%f, max_y_advance = %f", __func__, font_extents.ascent,
      font_extents.descent, font_extents.height, font_extents.max_x_advance,
      font_extents.max_y_advance);

  cairo_text_extents_t text_extents;
  cairo_text_extents (cr_context_, bbox_name_.string(), &text_extents);

  OVDBG_VERBOSE("%s: BBox Text: te.x_bearing=%f, te.y_bearing=%f, te.width=%f,"
      " te.height=%f, te.x_advance=%f, te.y_advance=%f", __func__,
      text_extents.x_bearing, text_extents.y_bearing,
      text_extents.width, text_extents.height,
      text_extents.x_advance, text_extents.y_advance);

  cairo_font_options_t *options;
  options = cairo_font_options_create ();
  cairo_font_options_set_antialias (options, CAIRO_ANTIALIAS_BEST);
  cairo_set_font_options (cr_context_, options);
  cairo_font_options_destroy (options);

  double x_text = 0.0;
  double y_text = text_extents.height;
  cairo_move_to (cr_context_, x_text, y_text);

  RGBAValues bbox_color;
  memset(&bbox_color, 0x0, sizeof bbox_color);
  ExtractColorValues(bbox_color_, &bbox_color);
  cairo_set_source_rgba (cr_context_, bbox_color.red, bbox_color.green,
                         bbox_color.blue, bbox_color.alpha);
  cairo_show_text (cr_context_, bbox_name_.string());
  assert(CAIRO_STATUS_SUCCESS == cairo_status(cr_context_));

  // Draw rectangle
  cairo_set_line_width (cr_context_, BOUNDING_BOX_STROKE_WIDTH);
  cairo_set_source_rgba (cr_context_, bbox_color.red, bbox_color.green,
                         bbox_color.blue, bbox_color.alpha);
  double x_rect = 0.0;
  double y_rect = text_extents.height + (font_extents.descent/2);
  cairo_rectangle (cr_context_, x_rect, y_rect, buffer_width_,
                   buffer_height_ - y_rect);
  cairo_stroke (cr_context_);
  assert(CAIRO_STATUS_SUCCESS == cairo_status(cr_context_));

  cairo_surface_flush (cr_surface_);

#elif USE_SKIA
  if (width_ > 0 && height_ > 0) {

#ifndef DEBUG_BACKGROUND_SURFACE
  canvas_->clear(SK_AlphaOPAQUE);
#else
  canvas_->clear(SK_ColorDKGRAY);
#endif
    SkPaint paintBox, paintText;
    paintText.setColor(bbox_color_);
    paintBox.setColor(bbox_color_);

    paintText.setTextSize(SkIntToScalar(BOUNDING_BOX_TEXT_SIZE));
    paintText.setAntiAlias(true);

    paintBox.setStrokeWidth(BOUNDING_BOX_STROKE_WIDTH);
    paintBox.setStyle(SkPaint::kStroke_Style);

    int32_t xText = 0, yText = 0;
    int32_t xBBox = 0, yBBox = 0;
    if(bbox_name_.length() > 1) {
      SkString text(bbox_name_.string(), bbox_name_.length());
      // Text size is always 20% of buffer height.
      yText  = BOUNDING_BOX_BUF_HEIGHT * BOUNDING_BOX_TEXT_PERCENT/100;
      // Margin between text and bouding box rect.
      yText  = yText - BOUNDING_BOX_TEXT_MARGIN;
      canvas_->drawText(text.c_str(), text.size(), xText, yText, paintText);
    }
    yBBox = yText > 0 ? BOUNDING_BOX_TEXT_SIZE : 0;
    int32_t boxWidth  = BOUNDING_BOX_BUF_WIDTH;
    int32_t boxHeight = BOUNDING_BOX_BUF_HEIGHT - yBBox;
    text_height_ = yText;
    canvas_->drawRect(SkRect::MakeXYWH(xBBox, yBBox, boxWidth, boxHeight),
                      paintBox);
    canvas_->flush();
    usleep(3000);
  }
#endif
  MarkDirty(false);
  OVDBG_VERBOSE("%s: Exit", __func__);
  return ret;
}

void OverlayItemBoundingBox::GetDrawInfo(uint32_t targetWidth,
                                         uint32_t targetHeight,
                                         DrawInfo* draw_info) {
  OVDBG_VERBOSE("%s: Enter", __func__);
  //Cut Text portion while scaling up bounding box to stream size.
  int32_t textPortion = buffer_height_ * BOUNDING_BOX_TEXT_PERCENT/100;
  textPortion        += BOUNDING_BOX_TEXT_MARGIN;
  int32_t ratio           = height_ / buffer_height_;
  draw_info->x            = x_;
  draw_info->y            = y_ - (ratio * textPortion);
  draw_info->width        = width_;
  draw_info->height       = height_ + (ratio * textPortion);
  draw_info->c2dSurfaceId = c2dsurface_id_;
  OVDBG_VERBOSE("%s: Exit", __func__);
}

void OverlayItemBoundingBox::GetParameters(OverlayParam& param) {

  OVDBG_VERBOSE("%s:Enter ",__func__);
  param.type             = OverlayType::kBoundingBox;
  param.location         = OverlayLocationType::kNone;
  param.color            = bbox_color_;
  param.dst_rect.start_x = x_;
  param.dst_rect.start_y = y_;
  param.dst_rect.width   = width_;
  param.dst_rect.height  = height_;
  std::string str(bbox_name_.string());
  str.copy(param.bounding_box.box_name, bbox_name_.length());
  OVDBG_VERBOSE("%s:Exit ",__func__);
}

int32_t OverlayItemBoundingBox::UpdateParameters(OverlayParam& param) {

  OVDBG_VERBOSE("%s:Enter ",__func__);
  int32_t ret = 0;

  if((param.dst_rect.width <= 0) || (param.dst_rect.height <= 0)) {
      return BAD_VALUE;
  }
  if(param.dst_rect.start_x < 0 || param.dst_rect.start_y < 0) {
      return BAD_VALUE;
  }
  x_          = param.dst_rect.start_x;
  y_          = param.dst_rect.start_y;
  width_      = param.dst_rect.width;
  height_     = param.dst_rect.height;

  if ( (bbox_color_ != param.color)
     || strcmp(bbox_name_.string(), param.bounding_box.box_name)) {

    bbox_color_ = param.color;
    bbox_name_.clear();
    int32_t textLen = strlen(param.bounding_box.box_name);

    int32_t textLimit = std::min(textLen + 1, BOUNDING_BOX_TEXT_LIMIT);
    bbox_name_.setTo(param.bounding_box.box_name, textLimit);
    MarkDirty(true);
  }

  OVDBG_VERBOSE("%s:Exit ",__func__);
  return ret;
}

int32_t OverlayItemBoundingBox::CreateSurface() {

  OVDBG_VERBOSE("%s: Enter", __func__);
  int32_t size = buffer_width_ * buffer_height_ * 4;

  IonMemInfo mem_info;
  memset(&mem_info, 0x0, sizeof(IonMemInfo));
  auto ret = AllocateIonMemory(mem_info, size);
  if(0 != ret) {
    OVDBG_ERROR("%s:AllocateIonMemory failed",__func__);
    return ret;
  }
  OVDBG_DEBUG("%s: Ion memory allocated fd(%d)", __func__, mem_info.fd);

#if USE_CAIRO
  cr_surface_ = cairo_image_surface_create_for_data(static_cast<unsigned char*>
                                                    (mem_info.vaddr),
                                                    CAIRO_FORMAT_ARGB32,
                                                    buffer_width_,
                                                    buffer_height_,
                                                    buffer_width_ * 4);
  assert (cr_surface_ != nullptr);

  cr_context_ = cairo_create (cr_surface_);
  assert (cr_context_ != nullptr);

#elif USE_SKIA
  //Create Skia canvas outof ION memory.
  SkImageInfo imageInfo;
  imageInfo.Make(BOUNDING_BOX_BUF_WIDTH, BOUNDING_BOX_BUF_HEIGHT,
                 kRGBA_8888_SkColorType, kPremul_SkAlphaType);

  canvas_ = SkCanvas::NewRasterDirect(imageInfo, mem_info.vaddr,
                                      BOUNDING_BOX_BUF_WIDTH *4);
  if(!canvas_) {
    OVDBG_ERROR("%s: Skia Creation failed!!", __func__);
    goto ERROR;
  }
#endif
  //Setup c2d.
  ret = c2dMapAddr(mem_info.fd, mem_info.vaddr, mem_info.size, 0,
                   KGSL_USER_MEM_TYPE_ION, &gpu_addr_);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s: c2dMapAddr failed!", __func__);
    goto ERROR;
  }

  C2D_RGB_SURFACE_DEF c2dSurfaceDef;
#if USE_CAIRO
  c2dSurfaceDef.format = C2D_COLOR_FORMAT_8888_ARGB;
#elif USE_SKIA
  c2dSurfaceDef.format = C2D_FORMAT_SWAP_ENDIANNESS| C2D_COLOR_FORMAT_8888_RGBA;
#endif
  c2dSurfaceDef.width  = buffer_width_;
  c2dSurfaceDef.height = buffer_height_;
  c2dSurfaceDef.buffer = mem_info.vaddr;
  c2dSurfaceDef.phys   = gpu_addr_;
  c2dSurfaceDef.stride = buffer_width_ * 4;

  //Create source c2d surface.
  ret = c2dCreateSurface(&c2dsurface_id_, C2D_SOURCE,
                         (C2D_SURFACE_TYPE)(C2D_SURFACE_RGB_HOST
                         |C2D_SURFACE_WITH_PHYS), &c2dSurfaceDef);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s: c2dCreateSurface failed!", __func__);
    goto ERROR;
  }

  ion_fd_      = mem_info.fd;
  vaddr_       = mem_info.vaddr;
  size_        = mem_info.size;
  handle_data_ = mem_info.handle_data;

  OVDBG_VERBOSE("%s: Exit", __func__);
  return ret;
ERROR:
  ioctl(ion_device_, ION_IOC_FREE, &handle_data_);
  close(ion_fd_);
  ion_fd_ = -1;
  return ret;
}

OverlayItemText::OverlayItemText(int32_t ion_device)
    :OverlayItem(ion_device), text_() {
  OVDBG_VERBOSE("%s:Enter ", __func__);
  type_ = OverlayType::kUserText;
  OVDBG_VERBOSE("%s:Exit ", __func__);
}

OverlayItemText::~OverlayItemText() {
  OVDBG_VERBOSE("%s:Enter ", __func__);
  text_.clear();
  OVDBG_VERBOSE("%s:Exit ", __func__);
}

int32_t OverlayItemText::Init(OverlayParam& param) {

  OVDBG_VERBOSE("%s: Enter", __func__);

  location_type_ = param.location;
  text_color_    = param.color;
  x_             = param.dst_rect.start_x;
  y_             = param.dst_rect.start_y;
  width_         = param.dst_rect.width;
  height_        = param.dst_rect.height;

  text_.setTo(param.user_text, strlen(param.user_text) + 1);

  if (param.dst_rect.width == 0 || param.dst_rect.height == 0) {
    width_  = TEXT_BUF_WIDTH;
    height_ = TEXT_BUF_HEIGHT;
  }
  auto ret = CreateSurface();
  if(ret != 0) {
    OVDBG_ERROR("%s: CreateSurface failed!", __func__);
    return ret;
  }
  OVDBG_VERBOSE("%s: Exit", __func__);
  return ret;
}

int32_t OverlayItemText::UpdateAndDraw() {

  OVDBG_VERBOSE("%s: Enter", __func__);
  int32_t ret = 0;

  if(!dirty_)
    return ret;

#if USE_CAIRO
  ClearSurface();
  cairo_select_font_face(cr_context_, "@cairo:Georgia", CAIRO_FONT_SLANT_NORMAL,
                          CAIRO_FONT_WEIGHT_NORMAL);
  cairo_set_font_size (cr_context_, TEXT_SIZE);
  cairo_set_antialias (cr_context_, CAIRO_ANTIALIAS_BEST);
  assert(CAIRO_STATUS_SUCCESS == cairo_status(cr_context_));

  cairo_font_extents_t font_extent;
  cairo_font_extents (cr_context_, &font_extent);
  OVDBG_VERBOSE("%s: ascent=%f, descent=%f, height=%f, max_x_advance=%f,"
      " max_y_advance = %f", __func__, font_extent.ascent, font_extent.descent,
       font_extent.height, font_extent.max_x_advance,
       font_extent.max_y_advance);

  cairo_text_extents_t text_extents;
  cairo_text_extents (cr_context_, text_.string(), &text_extents);

  OVDBG_VERBOSE("%s: Custom text: te.x_bearing=%f, te.y_bearing=%f,"
      " te.width=%f, te.height=%f, te.x_advance=%f, te.y_advance=%f", __func__,
      text_extents.x_bearing, text_extents.y_bearing,
      text_extents.width, text_extents.height,
      text_extents.x_advance, text_extents.y_advance);

  cairo_font_options_t *options;
  options = cairo_font_options_create ();
  cairo_font_options_set_antialias (options, CAIRO_ANTIALIAS_DEFAULT);
  cairo_set_font_options (cr_context_, options);
  cairo_font_options_destroy (options);

  //(0,0) is at topleft corner of draw buffer.
  double x_text = 0.0;
  double y_text = text_extents.height - (font_extent.descent/2.0);
  OVDBG_VERBOSE("%s: x_text=%f, y_text=%f", __func__, x_text, y_text);
  cairo_move_to (cr_context_, x_text, y_text);

  // Draw Text.
  RGBAValues text_color;
  memset(&text_color, 0x0, sizeof text_color);
  ExtractColorValues(text_color_, &text_color);
  cairo_set_source_rgba (cr_context_, text_color.red, text_color.green,
                         text_color.blue, text_color.alpha);

  cairo_show_text (cr_context_, text_.string());
  assert(CAIRO_STATUS_SUCCESS == cairo_status(cr_context_));
  cairo_surface_flush(cr_surface_);

#elif USE_SKIA

#ifndef DEBUG_BACKGROUND_SURFACE
  canvas_->clear(SK_AlphaOPAQUE);
#else
  canvas_->clear(SK_ColorDKGRAY);
#endif

  SkPaint paint;
  paint.setColor(text_color_);
  paint.setTextSize(SkIntToScalar(TEXT_SIZE));
  paint.setAntiAlias(true);

  int32_t x = 0;
  int32_t y = TEXT_BUF_HEIGHT - TEXT_SIZE/2;
  SkString skText(text_.string(), text_.length());
  canvas_->drawText(skText.c_str(), skText.size(), x, y, paint);
  canvas_->flush();
  usleep(1000);
#endif
  dirty_ = false;
  OVDBG_VERBOSE("%s: Exit", __func__);
  return ret;
}

void OverlayItemText::GetDrawInfo(uint32_t targetWidth,
                                  uint32_t targetHeight, DrawInfo* draw_info) {

  OVDBG_VERBOSE("%s: Enter", __func__);
  draw_info->width  = targetWidth * TEXT_TARGET_WIDTH_PERCENT/100;
  draw_info->height = targetHeight * TEXT_TARGET_HEIGHT_PERCENT/100;

  int32_t xMargin = targetWidth * OVERLAYITEM_X_MARGIN_PERCENT/100;
  int32_t yMargin = targetHeight * OVERLAYITEM_Y_MARGIN_PERCENT/100;
  int32_t x = 0;
  int32_t y = 0;

  // (0,0) is at topleft corner.
  switch (location_type_) {
    case OverlayLocationType::kTopLeft:
      x = xMargin;
      y = yMargin;
      break;
    case OverlayLocationType::kTopRight:
      x = targetWidth - (draw_info->width + xMargin);
      y = yMargin;
      break;
    case OverlayLocationType::kCenter:
      x = (targetWidth - draw_info->width)/2;
      y = (targetHeight - draw_info->height)/2;
      break;
    case OverlayLocationType::kBottomLeft:
      x = xMargin;
      y = targetHeight - (draw_info->height + yMargin);
      break;
    case OverlayLocationType::kBottomRight:
      x = targetWidth - (draw_info->width + xMargin);
      y = targetHeight - (draw_info->height + yMargin);
      break;
    case OverlayLocationType::kRandom:
      x = x_;
      y = y_;
      break;
    case OverlayLocationType::kNone:
    default:
      x = x_;
      y = y_;
      break;
  }
  draw_info->x            = x;
  draw_info->y            = y;
  draw_info->c2dSurfaceId = c2dsurface_id_;

  OVDBG_VERBOSE("%s: Exit", __func__);
}

void OverlayItemText::GetParameters(OverlayParam& param) {

  OVDBG_VERBOSE("%s:Enter ",__func__);
  param.type             = OverlayType::kUserText;
  param.location         = location_type_;
  param.color            = text_color_;
  param.dst_rect.start_x = x_;
  param.dst_rect.start_y = y_;
  param.dst_rect.width   = width_;
  param.dst_rect.height  = height_;
  std::string str(text_.string());
  str.copy(param.user_text, text_.length());
  OVDBG_VERBOSE("%s:Exit ",__func__);
}

int32_t OverlayItemText::UpdateParameters(OverlayParam& param) {

  OVDBG_VERBOSE("%s:Enter ",__func__);
  int32_t ret = 0;
  location_type_ = param.location;
  text_color_    = param.color;
  x_             = param.dst_rect.start_x;
  y_             = param.dst_rect.start_y;
  width_         = param.dst_rect.width;
  height_        = param.dst_rect.height;
  text_.clear();
  text_.setTo(param.user_text, strlen(param.user_text) + 1);
  MarkDirty(true);
  OVDBG_VERBOSE("%s:Exit ",__func__);
  return ret;
}

int32_t OverlayItemText::CreateSurface() {

  OVDBG_VERBOSE("%s: Enter", __func__);
  int32_t size = width_ * height_ * 4;
  IonMemInfo mem_info;
  memset(&mem_info, 0x0, sizeof(IonMemInfo));

  auto ret = AllocateIonMemory(mem_info, size);
  if(0 != ret) {
    OVDBG_ERROR("%s:AllocateIonMemory failed",__func__);
    return ret;
  }
  OVDBG_INFO("%s: Ion memory allocated fd = %d", __func__, mem_info.fd);
#if USE_CAIRO
  cr_surface_ = cairo_image_surface_create_for_data(static_cast<unsigned char*>
                                                    (mem_info.vaddr),
                                                    CAIRO_FORMAT_ARGB32, width_,
                                                    height_, width_ * 4);
  assert (cr_surface_ != nullptr);

  cr_context_ = cairo_create (cr_surface_);
  assert (cr_context_ != nullptr);

#elif USE_SKIA
  //Create Skia canvas outof ION memory.
  SkImageInfo imageInfo;
  imageInfo.Make(width_, height_, kRGBA_8888_SkColorType, kPremul_SkAlphaType);

  canvas_ = SkCanvas::NewRasterDirect(imageInfo, mem_info.vaddr,
                                      width_ * 4);
  if(!canvas_) {
    OVDBG_ERROR("%s: Skia Creation failed!!",__func__);
    goto ERROR;
  }
#endif
  //Draw system time on Skia canvas.
  UpdateAndDraw();

  //Setup c2d.
  ret = c2dMapAddr(mem_info.fd, mem_info.vaddr, mem_info.size, 0,
                   KGSL_USER_MEM_TYPE_ION, &gpu_addr_);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s: c2dMapAddr failed!",__func__);
    goto ERROR;
  }

  C2D_RGB_SURFACE_DEF c2dSurfaceDef;
#if USE_CAIRO
  c2dSurfaceDef.format = C2D_COLOR_FORMAT_8888_ARGB;
#elif USE_SKIA
  c2dSurfaceDef.format = C2D_FORMAT_SWAP_ENDIANNESS| C2D_COLOR_FORMAT_8888_RGBA;
#endif
  c2dSurfaceDef.width  = width_;
  c2dSurfaceDef.height = height_;
  c2dSurfaceDef.buffer = mem_info.vaddr;
  c2dSurfaceDef.phys   = gpu_addr_;
  c2dSurfaceDef.stride = width_ * 4;

  //Create source c2d surface.
  ret = c2dCreateSurface(&c2dsurface_id_, C2D_SOURCE,
                         (C2D_SURFACE_TYPE)(C2D_SURFACE_RGB_HOST
                         |C2D_SURFACE_WITH_PHYS), &c2dSurfaceDef);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s: c2dCreateSurface failed!",__func__);
    goto ERROR;
  }

  ion_fd_       = mem_info.fd;
  vaddr_        = mem_info.vaddr;
  size_         = mem_info.size;
  handle_data_  = mem_info.handle_data;

  OVDBG_INFO("%s: Exit", __func__);
  return ret;

ERROR:
  ioctl(ion_device_, ION_IOC_FREE, &handle_data_);
  close(ion_fd_);
  ion_fd_ = -1;
  return ret;
}


OverlayItemPrivacyMask::OverlayItemPrivacyMask(int32_t ion_device)
    :OverlayItem(ion_device) {
  OVDBG_VERBOSE("%s: Enter", __func__);
  type_ = OverlayType::kPrivacyMask;
  OVDBG_VERBOSE("%s: Exit", __func__);
}

OverlayItemPrivacyMask::~OverlayItemPrivacyMask() {
  OVDBG_VERBOSE("%s: Enter", __func__);
  OVDBG_VERBOSE("%s: Exit", __func__);
}

int32_t OverlayItemPrivacyMask::Init(OverlayParam& param) {

  OVDBG_VERBOSE("%s: Enter", __func__);

  if((param.dst_rect.width <= 0) || (param.dst_rect.height <= 0)) {
    return BAD_VALUE;
  }
  if(param.dst_rect.start_x < 0 || param.dst_rect.start_y < 0) {
    return BAD_VALUE;
  }

  x_          = param.dst_rect.start_x;
  y_          = param.dst_rect.start_y;
  width_      = param.dst_rect.width;
  height_     = param.dst_rect.height;
  mask_color_ = param.color;

  auto ret = CreateSurface();
  if(ret != 0) {
    OVDBG_ERROR("%s: CreateSurface failed!", __func__);
    return NO_INIT;
  }
  OVDBG_VERBOSE("%s: Exit", __func__);
  return ret;
}

int32_t OverlayItemPrivacyMask::UpdateAndDraw() {

  OVDBG_VERBOSE("%s: Enter ", __func__);
  int32_t ret = 0;

  if(!dirty_) {
    OVDBG_DEBUG("%s: Item is not dirty! Don't draw!", __func__);
    return ret;
  }
#if USE_CAIRO
  ClearSurface();
  RGBAValues mask_color;
  ExtractColorValues(mask_color_, &mask_color);

  // Paint entire rectangle with color.
  cairo_set_source_rgba (cr_context_, mask_color.red, mask_color.green,
                         mask_color.blue, mask_color.alpha);
  cairo_paint(cr_context_);
  assert(CAIRO_STATUS_SUCCESS == cairo_status(cr_context_));
  cairo_surface_flush (cr_surface_);
#elif USE_SKIA
  //Create Skia canvas outof ION memory.
  SkPaint paintBox;

#ifndef DEBUG_BACKGROUND_SURFACE
  canvas_->clear(SK_AlphaOPAQUE);
#else
  canvas_->clear(SK_ColorDKGRAY);
#endif

  paintBox.setColor(mask_color_);
  paintBox.setStyle(SkPaint::kFill_Style);
  //For blurring effect
  paintBox.setMaskFilter(SkBlurMaskFilter::Create(kNormal_SkBlurStyle,5.0f, 0));
  OVDBG_VERBOSE(" x_ %d y_ %d width_ %d height_ %d",x_,y_,width_,height_);
  canvas_->drawRect(SkRect::MakeXYWH(0,0, width_, height_), paintBox);
  canvas_->flush();
#endif
  // Don't paint until params gets updated by app(UpdateParameters).
  MarkDirty(false);
  return OK;
}

void OverlayItemPrivacyMask::GetDrawInfo(uint32_t targetWidth,
                                         uint32_t targetHeight,
                                         DrawInfo* draw_info) {

  OVDBG_VERBOSE("%s: Enter", __func__);
  draw_info->x            = x_;
  draw_info->y            = y_;
  draw_info->width        = width_;
  draw_info->height       = height_;
  draw_info->c2dSurfaceId = c2dsurface_id_;
  OVDBG_VERBOSE("%s: Exit", __func__);
}

void OverlayItemPrivacyMask::GetParameters(OverlayParam& param) {

  OVDBG_VERBOSE("%s:Enter ",__func__);
  param.type             = OverlayType::kPrivacyMask;
  param.location         = OverlayLocationType::kNone;
  param.dst_rect.start_x = x_;
  param.dst_rect.start_y = y_;
  param.dst_rect.width   = width_;
  param.dst_rect.height  = height_;
  param.color            = mask_color_;
  OVDBG_VERBOSE("%s:Exit ",__func__);
}

int32_t OverlayItemPrivacyMask::UpdateParameters(OverlayParam& param) {

  OVDBG_VERBOSE("%s:Enter ",__func__);
  int32_t ret = 0;

  if((param.dst_rect.width <= 0) || (param.dst_rect.height <= 0)) {
    return BAD_VALUE;
  }
  if(param.dst_rect.start_x < 0 || param.dst_rect.start_y < 0) {
    return BAD_VALUE;
  }
  x_          = param.dst_rect.start_x;
  y_          = param.dst_rect.start_y;
  width_      = param.dst_rect.width;
  height_     = param.dst_rect.height;
  mask_color_ = param.color;

  // Mark dirty, updated contents would be re-painted in next paint cycle.
  MarkDirty(true);
  OVDBG_VERBOSE("%s:Exit ",__func__);
  return ret;
}

int32_t OverlayItemPrivacyMask::CreateSurface() {

  OVDBG_VERBOSE("%s: Enter", __func__);

  int32_t size = PMASK_BOX_BUF_WIDTH * PMASK_BOX_BUF_HEIGHT * 4;
  IonMemInfo mem_info;
  memset(&mem_info, 0x0, sizeof(IonMemInfo));

  auto ret = AllocateIonMemory(mem_info, size);
  if(0 != ret) {
    OVDBG_ERROR("%s:AllocateIonMemory failed",__func__);
    return ret;
  }
  OVDBG_DEBUG("%s: Ion memory allocated fd(%d)", __func__, mem_info.fd);
#if USE_CAIRO
  cr_surface_ = cairo_image_surface_create_for_data(static_cast<unsigned char*>
                                                    (mem_info.vaddr),
                                                    CAIRO_FORMAT_ARGB32,
                                                    PMASK_BOX_BUF_WIDTH,
                                                    PMASK_BOX_BUF_HEIGHT,
                                                    PMASK_BOX_BUF_WIDTH * 4);
  assert (cr_surface_ != nullptr);

  cr_context_ = cairo_create (cr_surface_);
  assert (cr_context_ != nullptr);
#elif USE_SKIA
  //Create Skia canvas outof ION memory.
  SkImageInfo imageInfo;
  imageInfo.Make(PMASK_BOX_BUF_WIDTH, PMASK_BOX_BUF_HEIGHT,
                 kRGBA_8888_SkColorType, kPremul_SkAlphaType);

  canvas_ = SkCanvas::NewRasterDirect(imageInfo, mem_info.vaddr,
                                      PMASK_BOX_BUF_WIDTH *4);
  if(!canvas_) {
    OVDBG_ERROR("%s: Skia Creation failed!!", __func__);
    goto ERROR;
  }
#endif
  //Setup c2d.
  ret = c2dMapAddr(mem_info.fd, mem_info.vaddr, mem_info.size, 0,
                   KGSL_USER_MEM_TYPE_ION, &gpu_addr_);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s: c2dMapAddr failed!", __func__);
    goto ERROR;
  }

  C2D_RGB_SURFACE_DEF c2dSurfaceDef;
  c2dSurfaceDef.format = C2D_COLOR_FORMAT_8888_ARGB;
  c2dSurfaceDef.width  = PMASK_BOX_BUF_WIDTH;
  c2dSurfaceDef.height = PMASK_BOX_BUF_HEIGHT;
  c2dSurfaceDef.buffer = mem_info.vaddr;
  c2dSurfaceDef.phys   = gpu_addr_;
  c2dSurfaceDef.stride = PMASK_BOX_BUF_WIDTH *4;

  //Create source c2d surface.
  ret = c2dCreateSurface(&c2dsurface_id_, C2D_SOURCE,
                         (C2D_SURFACE_TYPE)(C2D_SURFACE_RGB_HOST
                         |C2D_SURFACE_WITH_PHYS), &c2dSurfaceDef);
  if(ret != C2D_STATUS_OK) {
    OVDBG_ERROR("%s: c2dCreateSurface failed!", __func__);
    goto ERROR;
   }

  ion_fd_      = mem_info.fd;
  vaddr_       = mem_info.vaddr;
  size_        = mem_info.size;
  handle_data_ = mem_info.handle_data;

  OVDBG_VERBOSE("%s: Exit", __func__);
  return ret;

ERROR:
  ioctl(ion_device_, ION_IOC_FREE, &handle_data_);
  close(ion_fd_);
  ion_fd_ = -1;
  return ret;
}

}; // namespace overlay

}; // namespace qmmf
