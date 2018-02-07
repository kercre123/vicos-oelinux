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

#include <sys/types.h>
#include <map>
#include <mutex>

namespace qmmf {

namespace overlay {

#define MAX_STRING_LENGTH 128

enum class OverlayType {
  kDateType,
  kUserText,
  kStaticImage,
  kBoundingBox,
  kPrivacyMask
};

enum class OverlayLocationType {
  kTopLeft,
  kTopRight,
  kCenter,
  kBottomLeft,
  kBottomRight,
  kRandom,
  kNone
};

enum class OverlayTimeFormatType {
  kHHMMSS_24HR,
  kHHMMSS_AMPM,
  kHHMM_24HR,
  kHHMM_AMPM
};

enum class OverlayDateFormatType { kYYYYMMDD, kMMDDYYYY };

struct OverlayDateTimeType {
  OverlayTimeFormatType time_format;
  OverlayDateFormatType date_format;
};

struct BoundingBox {
  char box_name[MAX_STRING_LENGTH];
};

enum class OverlayImageType {
  kFilePath,
  kBlobType
};

struct OverlayRect {
  int32_t start_x;
  int32_t start_y;
  int32_t width;
  int32_t height;
};

struct OverlayImageInfo {
  OverlayImageType image_type;
  char image_location[MAX_STRING_LENGTH];
  char * image_buffer;
  uint32_t image_size;
  OverlayRect source_rect;
  bool buffer_updated;
};

struct OverlayParam {
  OverlayType type;
  OverlayLocationType location;
  uint32_t color;
  OverlayRect dst_rect;
  union {
    OverlayDateTimeType date_time;
    char user_text[MAX_STRING_LENGTH];
    OverlayImageInfo image_info;
    BoundingBox bounding_box;
  };
};

enum class TargetBufferFormat {
  kYUVNV12,
  kYUVNV21,
};

struct OverlayTargetBuffer {
  TargetBufferFormat format;
  uint32_t  width;
  uint32_t  height;
  uint32_t  ion_fd;
  uint32_t  frame_len;
};

class OverlayItem;

// This class provides facility to embed different
// Kinds of overlay on topof Camera stream buffers.
class Overlay {
 public:
  Overlay();

  ~Overlay();

  // Initialise overlay with format of buffer.
  int32_t Init(const TargetBufferFormat& format);

  // Create overlay item of type static image, date/time, bounding box,
  // simple text, or privacy mask. this Api provides overlay item id which
  // can be use for further configurartion change to item.
  int32_t CreateOverlayItem(OverlayParam& param, uint32_t* overlay_id);

  // Overlay item can be deleted at any point of time after creation.
  int32_t DeleteOverlayItem(uint32_t overlay_id);

  // Overlay item's parameters can be queried using this Api, it is recommended
  // to call get parameters first before setting new parameters using Api
  // updateOverlayItem.
  int32_t GetOverlayParams(uint32_t overlay_id, OverlayParam& param);

  // Overlay item's configuration can be change at run time using this Api.
  // user has to provide overlay Id and updated parameters.
  int32_t UpdateOverlayParams(uint32_t overlay_id, OverlayParam& param);

  // Overlay Item can be enable/disable at run time.
  int32_t EnableOverlayItem(uint32_t overlay_id);
  int32_t DisableOverlayItem(uint32_t overlay_id);

  // Provide input YUV buffer to apply overlay.
  int32_t ApplyOverlay(const OverlayTargetBuffer& buffer);

 private:

  uint32_t GetC2dColorFormat(const TargetBufferFormat& format);

  bool IsOverlayItemValid(uint32_t overlay_id);

  std::map <uint32_t, OverlayItem* > overlay_items_;

  uint32_t     target_c2dsurface_id_;
  int32_t      ion_device_;
  uint32_t     id_;
  std::mutex   lock_;
};

}; // namespace overlay
}; // namespace qmmf
