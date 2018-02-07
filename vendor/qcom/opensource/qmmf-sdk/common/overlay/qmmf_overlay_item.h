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

#pragma once

#include <linux/msm_kgsl.h>
#include <linux/msm_ion.h>
#include <adreno/c2d2.h>
#include <utils/String8.h>
#include <cutils/properties.h>

#if USE_SKIA
#include <SkCanvas.h>
#elif USE_CAIRO
#include <cairo/cairo.h>
#endif

namespace qmmf {

namespace overlay {

/**
OVDBG_INFO, ERROR and WARN logs are enabled all the time by default.
*/
#define OVDBG_INFO(fmt, args...)  ALOGD(fmt, ##args)
#define OVDBG_ERROR(fmt, args...) ALOGE(fmt, ##args)
#define OVDBG_WARN(fmt, args...)  ALOGW(fmt, ##args)

// Remove comment markers to define LOG_LEVEL_DEBUG for debugging-related logs
//#define LOG_LEVEL_DEBUG

// Remove comment markers to define LOG_LEVEL_VERBOSE for complete logs
//#define LOG_LEVEL_VERBOSE

#ifdef LOG_LEVEL_DEBUG
#define OVDBG_DEBUG(fmt, args...)  ALOGD(fmt, ##args)
#else
#define OVDBG_DEBUG(...) ((void)0)
#endif

#ifdef LOG_LEVEL_VERBOSE
#define OVDBG_VERBOSE(fmt, args...)  ALOGD(fmt, ##args)
#else
#define OVDBG_VERBOSE(...) ((void)0)
#endif

#define OVERLAYITEM_X_MARGIN_PERCENT  0.5
#define OVERLAYITEM_Y_MARGIN_PERCENT  0.5
#define MAX_LEN       128
#define MAX_OVERLAYS  10
#define BG_TRANSPARENT_COLOR 0xFFFFFF00
#define BG_DEBUG_COLOR       0xFFE5CC80 //Light gray.
#define DOWNSCALE_FACTOR     4

// Remove comment marker to enable backgroud surface drawing of overlay objects.
//#define DEBUG_BACKGROUND_SURFACE

// Remove comment marker to measure time taken in overlay drawing.
//#define DEBUG_BLIT_TIME

#define PROP_DUMP_BLOB_IMAGE        "persist.qmmf.overlay.dump.blob"

struct DrawInfo {
    uint32_t width;
    uint32_t height;
    uint32_t x;
    uint32_t y;
    uint32_t c2dSurfaceId;
};

struct RGBAValues {
  double red;
  double green;
  double blue;
  double alpha;
};

struct C2dObjects {
  C2D_OBJECT objects[MAX_OVERLAYS];
};

//Base class for all types of overlays.
class OverlayItem {
 public:
  OverlayItem(int32_t ion_device);

  virtual ~OverlayItem();

  virtual int32_t Init(OverlayParam& param) = 0 ;

  virtual int32_t UpdateAndDraw() = 0;

  virtual void GetDrawInfo(uint32_t target_width, uint32_t target_height,
                           DrawInfo* draw_info) = 0 ;

  virtual void GetParameters(OverlayParam& param) = 0;

  virtual int32_t UpdateParameters(OverlayParam& param) = 0;

  OverlayType& GetItemType() {return type_; }

  void MarkDirty(bool dirty);

  void Activate(bool value);

  bool IsActive() { return is_active_; }

 protected:

  struct IonMemInfo {
      uint32_t               size;
      int32_t                fd;
      void *                 vaddr;
      struct ion_handle_data handle_data;
  };

  int32_t AllocateIonMemory(IonMemInfo& mem_info, uint32_t size);

  void ExtractColorValues(uint32_t hex_color, RGBAValues* color);

  void ClearSurface();

  int32_t                x_;
  int32_t                y_;
  uint32_t               width_;
  uint32_t               height_;
  uint32_t               c2dsurface_id_;
  void *                 gpu_addr_;
  void *                 vaddr_;
  int32_t                ion_fd_;
  uint32_t               size_;
  struct ion_handle_data handle_data_;
  OverlayLocationType    location_type_;
  bool                   dirty_;
  int32_t                ion_device_;
  OverlayType            type_;
#if USE_CAIRO
  cairo_surface_t*       cr_surface_;
  cairo_t*               cr_context_;
#endif
 private:
  bool                   is_active_;
};

class OverlayItemStaticImage : public OverlayItem {

 public:
  OverlayItemStaticImage(int32_t ion_device);

  virtual ~OverlayItemStaticImage();

  int32_t Init(OverlayParam& param) override;

  int32_t UpdateAndDraw() override;

  void GetDrawInfo(uint32_t target_width, uint32_t target_height,
      DrawInfo* draw_info) override;

  void GetParameters(OverlayParam& param) override;

  int32_t UpdateParameters(OverlayParam& param) override;
 private:
  int32_t CreateSurface();

  void DestroySurface();

  android::String8 image_path_;
  OverlayImageType image_type_;
  char *   image_buffer_;
  uint32_t image_size_;
  uint32_t image_width_;
  uint32_t image_height_;
  uint32_t crop_rect_x_;
  uint32_t crop_rect_y_;
  uint32_t crop_rect_width_;
  uint32_t crop_rect_height_;
  bool     blob_image_dump_enabled_;
  int32_t  blob_buffer_file_fd_;
  bool blob_buffer_updated_;
  std::mutex update_param_lock_;
};

#define DATETIME_TEXT_BUF_WIDTH         192
#define DATETIME_TEXT_BUF_HEIGHT        108
#define DATETIME_TARGET_WIDTH_PERCENT   12
#define DATETIME_TARGET_HEIGHT_PERCENT  12
#define DATETIME_PIXEL_SIZE             30

class OverlayItemDateAndTime: public OverlayItem {
 public:
  OverlayItemDateAndTime(int32_t ion_device);

  virtual ~OverlayItemDateAndTime();

  int32_t Init(OverlayParam& param) override;

  int32_t UpdateAndDraw() override;

  void GetDrawInfo(uint32_t target_width, uint32_t target_height,
                   DrawInfo* draw_info) override;

  void GetParameters(OverlayParam& param) override;

  int32_t UpdateParameters(OverlayParam& param) override;

 private:
  int32_t CreateSurface();

  OverlayDateTimeType date_time_type_;
  uint32_t            text_color_;
#if USE_SKIA
  SkCanvas*           canvas_;
#endif
};

#define BOUNDING_BOX_BUF_WIDTH     480
#define BOUNDING_BOX_BUF_HEIGHT    270
#define BOUNDING_BOX_STROKE_WIDTH  4
#define BOUNDING_BOX_TEXT_LIMIT    20
#define BOUNDING_BOX_TEXT_SIZE     25
#define BOUNDING_BOX_TEXT_PERCENT  20
#define BOUNDING_BOX_TEXT_MARGIN   5

class OverlayItemBoundingBox: public OverlayItem {
 public:
  OverlayItemBoundingBox(int32_t ion_device);

  virtual ~OverlayItemBoundingBox();

  int32_t Init(OverlayParam& param) override;

  int32_t UpdateAndDraw() override;

  void GetDrawInfo(uint32_t target_width, uint32_t target_height,
                   DrawInfo* draw_info) override;

  void GetParameters(OverlayParam& param) override;

  int32_t UpdateParameters(OverlayParam& param) override;
 private:

  int32_t CreateSurface();

  uint32_t    bbox_color_;
#if USE_SKIA
  SkCanvas*            canvas_;
#endif
  android::String8  bbox_name_;
  uint32_t          text_height_   = 0;
  int32_t           buffer_width_  = 0;
  int32_t           buffer_height_ = 0;
};

#define TEXT_BUF_WIDTH              480
#define TEXT_BUF_HEIGHT             60
#define TEXT_TARGET_WIDTH_PERCENT   30
#define TEXT_TARGET_HEIGHT_PERCENT  10
#define TEXT_SIZE                   40

class OverlayItemText: public OverlayItem {
 public:
  OverlayItemText(int32_t ion_device);

  virtual ~OverlayItemText();

  int32_t Init(OverlayParam& param) override;

  int32_t UpdateAndDraw() override;

  void GetDrawInfo(uint32_t target_width, uint32_t target_height,
                   DrawInfo* draw_info) override;

  void GetParameters(OverlayParam& param) override;

  int32_t UpdateParameters(OverlayParam& param) override;

 private:
  int32_t CreateSurface();

  uint32_t          text_color_;
  android::String8  text_;
#if USE_SKIA
  SkCanvas*         canvas_;
#endif

};

#define PMASK_BOX_BUF_WIDTH     240
#define PMASK_BOX_BUF_HEIGHT    135

class OverlayItemPrivacyMask: public OverlayItem {
 public:

  OverlayItemPrivacyMask(int32_t ion_device);

  virtual ~OverlayItemPrivacyMask();

  int32_t Init(OverlayParam& param) override;

  int32_t UpdateAndDraw() override;

  void GetDrawInfo(uint32_t target_width, uint32_t target_height,
                   DrawInfo * draw_info) override;

  void GetParameters(OverlayParam& param) override;

  int32_t UpdateParameters(OverlayParam& param) override;

 private:
  int32_t CreateSurface();
#if USE_SKIA
  SkCanvas*   canvas_;
#endif
  uint32_t    mask_color_;
};

}; // namespace overlay
}; // namespace qmmf
