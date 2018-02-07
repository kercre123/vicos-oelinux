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

#define TAG "RecorderCameraJpeg"

#include <chrono>
#include <algorithm>
#include <fcntl.h>
#include <sys/mman.h>
#include <cutils/properties.h>

#include "recorder/src/service/qmmf_recorder_utils.h"

#include "qmmf_camera_jpeg.h"

namespace qmmf {

namespace recorder {

static const char *kVendorNameProp = "persist.qmmf.jpeg.vendor.name";
static const char *kProductNameProp = "persist.qmmf.jpeg.product.name";

CameraJpeg::CameraJpeg()
    : ExifGenerator(),
      reprocess_flag_(false),
      ready_to_start_(false),
      jpeg_encoder_(nullptr) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  char prop[PROPERTY_VALUE_MAX];

  property_get(kVendorNameProp, prop, "QTI");
  vendor_name_ = prop;

  property_get(kProductNameProp, prop, "QMMF-CAMERA");
  product_name_ = prop;

  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

CameraJpeg::~CameraJpeg() {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  Delete();
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

int32_t CameraJpeg::Create(const int32_t stream_id,
                           const PostProcParam& input,
                           const PostProcParam& output,
                           const uint32_t frame_rate,
                           const uint32_t num_images,
                           const uint32_t jpeg_quality,
                           const void* static_meta,
                           const PostProcCb& cb,
                           const void* context) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  if (ready_to_start_) {
    QMMF_ERROR("%s:%s: Failed: Already configured.", TAG, __func__);
    return BAD_VALUE;
  }

  if (reprocess_flag_) {
    QMMF_ERROR("%s:%s: Failed: Wrong state.", TAG, __func__);
    return BAD_VALUE;
  }
  CameraBufferMetaData meta_info;
  memset(&meta_info, 0, sizeof(CameraBufferMetaData));
  if (FillMetaInfo(input, &meta_info) != NO_ERROR) {
    return BAD_VALUE;
  }

  jpeg_encoder_ = JpegEncoder::getInstance();
  results_.clear();

  capture_client_cb_ = cb;
  input_stream_id_   = stream_id;
  num_images_        = num_images;
  jpeg_quality_      = jpeg_quality;
  ready_to_start_    = true;

  Run("Camera Jpeg");

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return 55; //TODO use reprocess ID
}

status_t CameraJpeg::GetCapabilities(PostProcCaps *caps) {
  caps->output_buff_ = 1;
  return NO_ERROR;
}

status_t CameraJpeg::Start() {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  if (!ready_to_start_) {
    return BAD_VALUE;
  }
  reprocess_flag_ = true;

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t CameraJpeg::Stop() {
  return NO_ERROR;
}

status_t CameraJpeg::Delete() {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);

  RequestExitAndWait();

  JpegEncoder::releaseInstance();
  jpeg_encoder_ = nullptr;

  reprocess_flag_ = false;
  ready_to_start_ = false;

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

void CameraJpeg::Process(StreamBuffer& in_buffer, StreamBuffer& out_buffer) {
  QMMF_INFO("%s:%s: %d: Enter ", TAG, __func__, __LINE__);

  void *buf_vaaddr = mmap(nullptr, in_buffer.size, PROT_READ  | PROT_WRITE,
      MAP_SHARED, in_buffer.fd, 0);
  void *out_vaaddr = mmap(nullptr, out_buffer.size, PROT_READ  | PROT_WRITE,
      MAP_SHARED, out_buffer.fd, 0);

  CameraMetadata meta;
  {
    std::unique_lock<std::mutex> lock(result_lock_);
     while (results_.count(in_buffer.timestamp) == 0) {
       std::chrono::nanoseconds timeout(kWaitJPEGTimeout);
       auto ret = wait_for_result_.wait_for(lock, timeout);
       if (std::cv_status::timeout == ret) {
          QMMF_ERROR("%s%s: Wait for jpeg result timed out", TAG, __func__);
          break;
       }
     }
     meta = results_.at(in_buffer.timestamp);
  }

  snapshot_info img_buffer;
  if (buf_vaaddr != MAP_FAILED || out_vaaddr != MAP_FAILED) {
    if (!meta.isEmpty()) {
      unsigned char *exif_buffer = new unsigned char[kMaxExifApp1Length];
      int exif_size = Generate(meta, vendor_name_, product_name_,
                               out_buffer.info.plane_info[0].width,
                               out_buffer.info.plane_info[0].height,
                               exif_buffer, kMaxExifApp1Length);
      convertExifBinaryToExifInfoStruct(exif_buffer);

      delete[] exif_buffer;

      if (exif_size != 0) {
        img_buffer.exif_size = exif_entities_.size();
        img_buffer.exif_data = (void*)exif_entities_.begin();
      } else {
        QMMF_ERROR("%s Empty exif section!", __func__);
        img_buffer.exif_size = 0;
        img_buffer.exif_data = (void*)0;
      }
    }

    size_t jpeg_size = 0;
    img_buffer.img_data[0] = static_cast<uint8_t*>(buf_vaaddr);
    img_buffer.out_data[0] = static_cast<uint8_t*>(out_vaaddr);
    img_buffer.source_info = in_buffer.info;
    auto buf_vaddr = reinterpret_cast<uint8_t *>(
        jpeg_encoder_->Encode(img_buffer, jpeg_size, jpeg_quality_));

    if (0 == jpeg_size) {
      QMMF_ERROR("%s:%s: JPEG size is 0!", TAG, __func__);
    } else {
      out_buffer.info.plane_info[0].width = jpeg_size;
      out_buffer.data = out_vaaddr;
      out_buffer.filled_length = jpeg_size;
      AddJpegHeader(out_buffer);
    }

    memcpy(buf_vaaddr, buf_vaddr, jpeg_size);
    munmap(buf_vaaddr, in_buffer.size);
    munmap(out_vaaddr, out_buffer.size);
    out_buffer.data = nullptr;
  } else {
    QMMF_INFO("%s:%s: SKIPP JPEG", TAG, __func__);
  }

  if (!meta.isEmpty()) {
    std::unique_lock<std::mutex> lock(result_lock_);
    results_.erase(in_buffer.timestamp);
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

status_t CameraJpeg::AddJpegHeader(StreamBuffer &buffer) {
  // JPEG header.
  camera3_jpeg_blob_t header;
  header.jpeg_blob_id = CAMERA3_JPEG_BLOB_ID;
  header.jpeg_size    = buffer.filled_length;

  // The header must be appended at the end of the allocated buffer.
  uint32_t offset = buffer.filled_length - sizeof(header);
  uintptr_t data  = reinterpret_cast<uintptr_t>(buffer.data);
  void *vaddr     = reinterpret_cast<void *>(data + offset);
  memcpy(vaddr, &header, sizeof(header));

  return NO_ERROR;
}

void CameraJpeg::AddBuff(StreamBuffer in_buff, StreamBuffer out_buff) {
  std::unique_lock<std::mutex> lock(buffer_lock_);
  Buff buff;
  buff.in = in_buff;
  buff.out = out_buff;
  input_buffer_.push_back(buff);
  wait_for_buffer_.notify_all();
}

void CameraJpeg::AddResult(const void* result) {

  CameraMetadata meta = *(reinterpret_cast<const CameraMetadata *>(result));
  if (!meta.exists(ANDROID_SENSOR_TIMESTAMP)) {
    QMMF_ERROR("%s:%s Sensor timestamp tag missing in result!", TAG, __func__);
    return;
  }
  int64_t timestamp = meta.find(ANDROID_SENSOR_TIMESTAMP).data.i64[0];

  std::unique_lock<std::mutex> lock(result_lock_);
  results_.emplace(timestamp, meta);
  wait_for_result_.notify_all();
}

status_t CameraJpeg::ReturnBuff(StreamBuffer buffer) {
  QMMF_INFO("%s:%s: StreamBuffer(0x%p) ts: %lld", TAG,
       __func__, buffer.handle, buffer.timestamp);
  return NO_ERROR;
}


bool CameraJpeg::ThreadLoop() {
  Buff buffer;
  {
    std::unique_lock<std::mutex> lock(buffer_lock_);
    while (input_buffer_.empty()) {
      std::chrono::nanoseconds timeout(kFrameTimeout);
      auto ret = wait_for_buffer_.wait_for(lock, timeout);
      if (std::cv_status::timeout == ret) {
         QMMF_ERROR("%s: Wait for pending buffers timed out", __func__);
        return true;
      }
    }
    auto iter = input_buffer_.begin();
    buffer = *iter;
    input_buffer_.erase(iter);
  }
  Process(buffer.in, buffer.out);
  capture_client_cb_(buffer.in, buffer.out);
  return true;
}

status_t CameraJpeg::FillMetaInfo(const PostProcParam& input,
                                  CameraBufferMetaData* info) {
  int aligned_width = input.width;
  int aligned_height = input.height;

  switch (input.format) {
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
    case HAL_PIXEL_FORMAT_YCbCr_420_888:
      info->format = BufferFormat::kNV12;
      info->num_planes = 2;
      info->plane_info[0].width = input.width;
      info->plane_info[0].height = input.height;
      info->plane_info[0].stride = aligned_width;
      info->plane_info[0].scanline = aligned_height;
      info->plane_info[1].width = input.width;
      info->plane_info[1].height = input.height/2;
      info->plane_info[1].stride = aligned_width;
      info->plane_info[1].scanline = aligned_height/2;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
      info->format = BufferFormat::kNV12UBWC;
      info->num_planes = 2;
      info->plane_info[0].width = input.width;
      info->plane_info[0].height = input.height;
      info->plane_info[0].stride = aligned_width;
      info->plane_info[0].scanline = aligned_height;
      info->plane_info[1].width = input.width;
      info->plane_info[1].height = input.height/2;
      info->plane_info[1].stride = aligned_width;
      info->plane_info[1].scanline = aligned_height/2;
      break;
    case HAL_PIXEL_FORMAT_NV21_ZSL:
      info->format = BufferFormat::kNV21;
      info->num_planes = 2;
      info->plane_info[0].width = input.width;
      info->plane_info[0].height = input.height;
      info->plane_info[0].stride = aligned_width;
      info->plane_info[0].scanline = aligned_height;
      info->plane_info[1].width = input.width;
      info->plane_info[1].height = input.height/2;
      info->plane_info[1].stride = aligned_width;
      info->plane_info[1].scanline = aligned_height/2;
      break;
    default:
      QMMF_ERROR("%s:%s: Unsupported format: 0x%x", TAG, __func__,
                 input.format);
      return NAME_NOT_FOUND;
  }
  return NO_ERROR;
}

status_t CameraJpeg::getTagDataByTagType(const uint8_t *binary,
                                         uint32_t &offset,
                                         qmmf_exif_tag_t *tag) {
  if (offset >= kMaxExifApp1Length || tag == nullptr) {
    QMMF_ERROR("%s: %s buffer overflow", TAG, __func__);
    return BAD_VALUE;
  }
  status_t res = NO_ERROR;
  uint32_t tag_data_offset;

  switch (tag->entry.type) {
  case QMMF_EXIF_SHORT:
    tag->entry.data._short = readU16(binary, offset);
    constructExifTag(tag->id, tag->entry.count, tag->entry.type,
        &tag->entry.data._short);
    offset += kTagDataSize;
    break;
  case QMMF_EXIF_LONG:
    tag->entry.data._long = readU32(binary, offset);
    if (tag->id == QMMF_EXIFTAGID_EXIF_IFD_PTR) {
      exif_ifd_ptr_offset_ = tag->entry.data._long + 10;
    } else if (tag->id == QMMF_EXIFTAGID_GPS_IFD_PTR) {
      gps_ifd_ptr_offset_ = tag->entry.data._long + 10;
    } else if (tag->id == QMMF_EXIFTAGID_INTEROP_IFD_PTR) {
      interop_ifd_ptr_offset_ = tag->entry.data._long + 10;
    } else {
      constructExifTag(tag->id, tag->entry.count, tag->entry.type,
        &tag->entry.data._long);
    }
    offset += kTagDataSize;
    break;
  case QMMF_EXIF_RATIONAL:
    tag_data_offset = readU32(binary, offset);
    tag_data_offset += tiff_header_offset_;
    if (tag->entry.count > 1) {
      tag->entry.data._rats = new qmmf_exif_rat_t[tag->entry.count];
      for (uint32_t i = 0; i < tag->entry.count; i++) {
        tag->entry.data._rats[i].num = readU32(binary, tag_data_offset);
        tag_data_offset += kTagDataSize;
        tag->entry.data._rats[i].denom = readU32(binary, tag_data_offset);
        tag_data_offset += kTagDataSize;
      }
      constructExifTag(tag->id, tag->entry.count, tag->entry.type,
        tag->entry.data._rats);
    } else {
      tag->entry.data._rat.num = readU32(binary, tag_data_offset);
      tag_data_offset += kTagDataSize;
      tag->entry.data._rat.denom = readU32(binary, tag_data_offset);
      constructExifTag(tag->id, tag->entry.count, tag->entry.type,
          &tag->entry.data._rat);
    }
    offset += kTagDataSize;
    break;
  case QMMF_EXIF_ASCII:
    if (tag->entry.count <= 4) {
      /* QTI encoder requires allocation even for ASCII symbols smaller
       * than 4 bytes. This is not aligned with the exif standard, but
       * we dont want to change the encoder code.
       * So only option is to keep this workaround inplace */
      tag->entry.data._ascii = new char[tag->entry.count];
      memcpy(tag->entry.data._ascii, binary + offset, tag->entry.count);
    } else {
      tag_data_offset = readU32(binary, offset);
      tag_data_offset += tiff_header_offset_;
      tag->entry.data._ascii = new char[tag->entry.count];
      memcpy(tag->entry.data._ascii, binary + tag_data_offset, tag->entry.count);
    }
    constructExifTag(tag->id, tag->entry.count, tag->entry.type,
        tag->entry.data._ascii);
    offset += kTagDataSize;
    break;
  case QMMF_EXIF_BYTE:
    tag->entry.data._byte = (uint8_t) ((uint8_t) binary[offset] << 8);
    constructExifTag(tag->id, tag->entry.count, tag->entry.type,
        &tag->entry.data._byte);
    offset += kTagDataSize;
    break;
  case QMMF_EXIF_UNDEFINED:
  case QMMF_EXIF_SLONG:
  case QMMF_EXIF_SRATIONAL:
    QMMF_ERROR("%s: %s There should not be a tag with this type!", TAG,
        __func__);
    res = BAD_VALUE;
    break;
  default:
    QMMF_ERROR("%s: %s Unknown tag type.", TAG, __func__);
    res = BAD_VALUE;
    break;
  }
  return res;
}

uint32_t CameraJpeg::getTagIdByExifId(uint32_t exif_id) {
  switch (exif_id) {
  case QMMF_ID_ORIENTATION:
    return QMMF_EXIFTAGID_ORIENTATION;
  case QMMF_ID_EXIF_IFD_PTR:
    return QMMF_EXIFTAGID_EXIF_IFD_PTR;
  case QMMF_ID_GPS_IFD_PTR:
    return QMMF_EXIFTAGID_GPS_IFD_PTR;
  case QMMF_ID_EXPOSURE_TIME:
    return QMMF_EXIFTAGID_EXPOSURE_TIME;
  case QMMF_ID_F_NUMBER:
    return QMMF_EXIFTAGID_F_NUMBER;
  case QMMF_ID_ISO_SPEED_RATING:
    return QMMF_EXIFTAGID_ISO_SPEED_RATING;
  case QMMF_ID_APERTURE:
    return QMMF_EXIFTAGID_APERTURE;
  case QMMF_ID_FOCAL_LENGTH:
    return QMMF_EXIFTAGID_FOCAL_LENGTH;
  case QMMF_ID_EXIF_PIXEL_X_DIMENSION:
    return QMMF_EXIFTAGID_EXIF_PIXEL_X_DIMENSION;
  case QMMF_ID_EXIF_PIXEL_Y_DIMENSION:
    return QMMF_EXIFTAGID_EXIF_PIXEL_Y_DIMENSION;
  case QMMF_ID_INTEROP_IFD_PTR:
    return QMMF_EXIFTAGID_INTEROP_IFD_PTR;
  case QMMF_CONSTRUCT_TAGID(QMMF_EXIF_TAG_MAX_OFFSET, 0x0001):
    return QMMF_CONSTRUCT_TAGID(QMMF_EXIF_TAG_MAX_OFFSET, 0x0001);
  case QMMF_CONSTRUCT_TAGID(QMMF_EXIF_TAG_MAX_OFFSET, 0x0002):
    return QMMF_CONSTRUCT_TAGID(QMMF_EXIF_TAG_MAX_OFFSET, 0x0002);
  case QMMF_ID_WHITE_BALANCE:
    return QMMF_EXIFTAGID_WHITE_BALANCE;
  case QMMF_ID_EXPOSURE_MODE:
    return QMMF_EXIFTAGID_EXPOSURE_MODE;
  case QMMF_ID_DATE_TIME:
    return QMMF_EXIFTAGID_DATE_TIME;
  case QMMF_ID_EXIF_DATE_TIME_ORIGINAL:
    return QMMF_EXIFTAGID_EXIF_DATE_TIME_ORIGINAL;
  case QMMF_ID_EXIF_DATE_TIME_DIGITIZED:
    return QMMF_EXIFTAGID_EXIF_DATE_TIME_DIGITIZED;
  case QMMF_ID_SUBSEC_TIME:
    return QMMF_EXIFTAGID_SUBSEC_TIME;
  case QMMF_ID_SUBSEC_TIME_ORIGINAL:
    return QMMF_EXIFTAGID_SUBSEC_TIME_ORIGINAL;
  case QMMF_ID_SUBSEC_TIME_DIGITIZED:
    return QMMF_EXIFTAGID_SUBSEC_TIME_DIGITIZED;
  case QMMF_ID_MAKE:
    return QMMF_EXIFTAGID_MAKE;
  case QMMF_ID_MODEL:
    return QMMF_EXIFTAGID_MODEL;
  case QMMF_ID_SOFTWARE:
    return QMMF_EXIFTAGID_SOFTWARE;
  case QMMF_ID_GPS_LATITUDE_REF:
    return QMMF_EXIFTAGID_GPS_LATITUDE_REF;
  case QMMF_ID_GPS_LATITUDE:
    return QMMF_EXIFTAGID_GPS_LATITUDE;
  case QMMF_ID_GPS_LONGITUDE_REF:
    return QMMF_EXIFTAGID_GPS_LONGITUDE_REF;
  case QMMF_ID_GPS_LONGITUDE:
    return QMMF_EXIFTAGID_GPS_LONGITUDE;
  case QMMF_ID_GPS_ALTITUDE_REF:
    return QMMF_EXIFTAGID_GPS_ALTITUDE_REF;
  case QMMF_ID_GPS_ALTITUDE:
    return QMMF_EXIFTAGID_GPS_ALTITUDE;
  case QMMF_ID_GPS_TIMESTAMP:
    return QMMF_EXIFTAGID_GPS_TIMESTAMP;
  case QMMF_ID_GPS_DATESTAMP:
    return QMMF_EXIFTAGID_GPS_DATESTAMP;
  case QMMF_ID_GPS_PROCESSINGMETHOD:
    return QMMF_EXIFTAGID_GPS_PROCESSINGMETHOD;
  default: {
    QMMF_ERROR("%s: %s Unexpected exifId: %d", TAG, __func__, exif_id);
    return 0;
  }
  }
}

status_t CameraJpeg::parseIfd(const uint8_t *binary, uint32_t &offset) {
  if (offset >= kMaxExifApp1Length) {
    QMMF_ERROR("%s: %s buffer overflow", TAG, __func__);
    return BAD_VALUE;
  }
  uint32_t exif_id;
  qmmf_exif_tag_t tag;
  memset(&tag, 0, sizeof(qmmf_exif_tag_t));

  uint32_t parsed_tags_count = 0;
  uint32_t tags_count = readU16(binary, offset);
  offset += 2;
  while (parsed_tags_count < tags_count) {
    exif_id = readU16(binary, offset);
    offset += kTagIdSize;
    tag.id = getTagIdByExifId(exif_id);
    tag.entry.type = (qmmf_exif_tag_type_t) readU16(binary, offset);
    offset += kTagTypeSize;
    tag.entry.count = readU32(binary, offset);
    offset += kTagCountSize;
    status_t res = getTagDataByTagType(binary, offset, &tag);
    if (res != NO_ERROR) {
      return BAD_VALUE;
    }
    parsed_tags_count++;
  }
  return NO_ERROR;
}

status_t CameraJpeg::convertExifBinaryToExifInfoStruct(const uint8_t *binary) {
  if (binary == nullptr) {
    QMMF_ERROR("%s: %s No exif buffer found,", TAG, __func__);
    return BAD_VALUE;
  }
  exif_entities_.clear();
  exif_entities_.setCapacity(kMaxExifEntries);
  exif_ifd_ptr_offset_ = 0;
  interop_ifd_ptr_offset_ = 0;
  gps_ifd_ptr_offset_ = 0;
  tiff_header_offset_ = 0;
  uint32_t offset = 0;

  uint32_t tmp = readU16(binary, offset);
  if (tmp == (0xFF00 | APP1_MARKER)) {
    offset += 2;
  } else {
    QMMF_ERROR("%s: %s Error: APP1 marker not found in exif section!", TAG,
        __func__);
    return BAD_VALUE;
  }

  tmp = readU16(binary, offset);
  offset += 2;
  QMMF_ERROR("%s: %s Exif section size : %d", TAG, __func__, tmp);

  tmp = readU32(binary, offset);
  if (tmp == EXIF_HEADER) {
    /* Offset for EXIF_HEADER + 2 bytes that seperate it from TIFF header*/
    offset += 6;
  } else {
    QMMF_ERROR("%s: %s Error: EXIF_HEADER marker not found in exif section!",
        TAG, __func__);
    return BAD_VALUE;
  }
  tiff_header_offset_ = offset;

  tmp = readU16(binary, offset);
  if (tmp == TIFF_BIG_ENDIAN) {
      offset += 2;
  } else {
    QMMF_ERROR("%s: %s Error: TIFF_BIG_ENDIAN marker not found in exif section",
        TAG, __func__);
    return BAD_VALUE;
  }

  tmp = readU16(binary, offset);
  if (tmp == TIFF_HEADER) {
    offset += 2;
  } else {
    QMMF_ERROR("%s: %s Error: TIFF_HEADER marker not found in exif section!",
        TAG, __func__);
    return BAD_VALUE;
  }

  tmp = readU32(binary, offset);
  if (tmp == ZERO_IFD_OFFSET) {
    offset += 4;
  } else {
    QMMF_ERROR("%s: %s Error: ZERO_IFD_OFFSET not found in exif section!", TAG,
        __func__);
    return BAD_VALUE;
  }

  /* Parse 0th IFD*/
  status_t res = parseIfd(binary, offset);
  if (res != NO_ERROR) {
    return BAD_VALUE;
  }
  /* Parse Exif IFD*/
  offset = exif_ifd_ptr_offset_;
  res = parseIfd(binary, offset);
  if (res != NO_ERROR) {
    return BAD_VALUE;
  }
  /* Parse Gps IFD*/
  offset = gps_ifd_ptr_offset_;
  res = parseIfd(binary, offset);
  if (res != NO_ERROR) {
    return BAD_VALUE;
  }
  return NO_ERROR;
}

uint32_t CameraJpeg::readU32(const uint8_t *buffer, uint32_t offset) {
  return (uint32_t) (((uint32_t) buffer[offset] << 24)
      + ((uint32_t) buffer[offset + 1] << 16)
      + ((uint32_t) buffer[offset + 2] << 8)
      + (uint32_t) buffer[offset + 3]);
}

uint16_t CameraJpeg::readU16(const uint8_t *buffer, uint32_t offset) {
  return (uint16_t) (((uint16_t) buffer[offset] << 8)
      + (uint16_t) buffer[offset + 1]);
}

void CameraJpeg::constructExifTag(uint32_t id, uint32_t count,
    uint16_t type, uint8_t *data) {
  qmmf_exif_tag_t tag;
  memset(&tag, 0, sizeof(qmmf_exif_tag_t));
  tag.id = id;
  tag.entry.type = (qmmf_exif_tag_type_t) type;
  tag.entry.count = count;
  tag.entry.data._byte = *data;
  exif_entities_.push_back(tag);
}

void CameraJpeg::constructExifTag(uint32_t id, uint32_t count,
    uint16_t type, uint16_t *data) {
  qmmf_exif_tag_t tag;
  memset(&tag, 0, sizeof(qmmf_exif_tag_t));
  tag.id = id;
  tag.entry.type = (qmmf_exif_tag_type_t) type;
  tag.entry.count = count;
  tag.entry.data._short = *data;
  exif_entities_.push_back(tag);
}

void CameraJpeg::constructExifTag(uint32_t id, uint32_t count,
                                  uint16_t type, uint32_t *data) {
  qmmf_exif_tag_t tag;
  memset(&tag, 0, sizeof(qmmf_exif_tag_t));
  tag.id = id;
  tag.entry.type = (qmmf_exif_tag_type_t) type;
  tag.entry.count = count;
  tag.entry.data._long = *data;
  exif_entities_.push_back(tag);
}

void CameraJpeg::constructExifTag(uint32_t id, uint32_t count,
                                  uint16_t type, qmmf_exif_rat_t *data) {
  qmmf_exif_tag_t tag;
  memset(&tag, 0, sizeof(qmmf_exif_tag_t));
  tag.id = id;
  tag.entry.type = (qmmf_exif_tag_type_t) type;
  tag.entry.count = count;
  if (count > 1) {
    tag.entry.data._rats = data;
  } else {
    tag.entry.data._rat = *data;
  }
  exif_entities_.push_back(tag);
}

void CameraJpeg::constructExifTag(uint32_t id, uint32_t count,
                                  uint16_t type, char *data) {
  qmmf_exif_tag_t tag;
  memset(&tag, 0, sizeof(qmmf_exif_tag_t));
  tag.id = id;
  tag.entry.type = (qmmf_exif_tag_type_t) type;
  tag.entry.count = count;
  tag.entry.data._ascii = data;
  exif_entities_.push_back(tag);
}

}; // namespace recoder

}; // namespace qmmf

