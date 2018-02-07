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

#define TAG "ExifGenerator"

#include <cmath>
#include <ctime>
#include <cstring>
#include <sys/time.h>

#include "common/qmmf_log.h"
#include "qmmf_exif_generator.h"

namespace qmmf {

namespace recorder {

namespace exif {

static const char *kDefaultVersionName = "R98";
static const uint8_t  kDefaultVersion[] = {0x30, 0x31, 0x30, 0x30};
static const char kASCIICharacterCode[] =
        { 'A', 'S', 'C', 'I', 'I', '\0', '\0', '\0' };
static const uint32_t kASCIICharacterCodeSize = 8;

ExifGenerator::ExifGenerator()
  : exif_buffer_(nullptr),
    helper_buffer_(nullptr),
    width_(0),
    height_(0),
    max_exif_size_(0),
    current_offset_(0),
    helper_buffer_offset_(0),
    tiff_header_begin_offset_(0),
    app_1begin_offset_(0),
    gps_ifd_pointer_offset_(0),
    new_gps_ifd_pointer_offset_(0),
    interop_ifd_pointer_offset_(0),
    new_interop_ifd_pointer_offset_(0),
    exif_ifd_pointer_offset_(0),
    new_exif_ifd_pointer_offset_(0),
    tag_count_(0),
    tag_count_offset_(0),
    app1_present_(false),
    overflow_flag_(false),
    gps_coords_present_(false),
    gps_timestamp_present_(false) {
}

ExifGenerator::~ExifGenerator() {
  delete[] helper_buffer_;
}

int32_t ExifGenerator::Generate(const CameraMetadata &meta,
                                const std::string &vendor_name,
                                const std::string &product_name,
                                int32_t width, int32_t height,
                                unsigned char *buffer, uint32_t size) {
  if (buffer == nullptr) {
    QMMF_ERROR("%s: %s Invalid buffer passed for the addition of "
        "exif section outputBuf: %p.", TAG, __func__, buffer);
    return BAD_VALUE;
  }
  exif_buffer_    = buffer;

  helper_buffer_  = new unsigned char[size];
  if (helper_buffer_ == nullptr) {
    QMMF_ERROR("%s: %s Unable to allocate helper buffer!", TAG, __func__);
    return NO_MEMORY;
  }

  meta_           = &meta;
  vendor_name_    = vendor_name;
  product_name_   = product_name;
  max_exif_size_  = size;
  width_          = width;
  height_         = height;
  current_offset_ = 0;

  return WriteExifData();
}

void ExifGenerator::WriteExifTag(qmmf_exif_tag_t *tag) {
  uint16_t tag_id;
  uint32_t write_length;

  if (!tag) {
    return;
  }

  tag_id = tag->id & 0xFFFF;

  WriteShort(tag_id, exif_buffer_, &current_offset_);
  WriteShort(static_cast<uint16_t>(tag->entry.type), exif_buffer_, &current_offset_);
  WriteLong(static_cast<int32_t>(tag->entry.count), exif_buffer_,
            &current_offset_);

  // Calculate the size of the information that need's to be written
  // in tag's data field, based on the count and size of the data.
  write_length = tag_type_sizes[tag->entry.type] * tag->entry.count;
  QMMF_VERBOSE("%s: %s write_length %d", TAG, __func__, write_length);

  // Write tag's data value, depending on the size of the info.
  // A helperBuffer is used for the case of size > 4 bytes:
  //  - write in the exifBuffer the offset of the helperBuffer
  //  - write in the helperBuffer the actual value
  if (write_length <= 4) {
      WriteTagValue(tag, exif_buffer_, &current_offset_);
      for (uint32_t i = write_length; i < 4; i++)
          WriteByte(0, exif_buffer_, &current_offset_);
  } else {
      if (helper_buffer_offset_ & 1) {
          ++helper_buffer_offset_;
      }
      WriteLong(static_cast<int32_t>(helper_buffer_offset_), exif_buffer_,
                &current_offset_);
      WriteTagValue(tag, helper_buffer_, &helper_buffer_offset_);
  }
  ++tag_count_;
  return;
}

void ExifGenerator::EndWritingIfd() {
  int32_t helper_buffer_destination;

  // Write 4 bytes of 0's at the end of each IFD
  WriteLong(0, exif_buffer_, &current_offset_);
  // Write the number of exif tags in the IFD at the beginning of the IFD
  WriteShort((int16_t) tag_count_, exif_buffer_, &tag_count_offset_);

  // Calculate the destination in the exifBuffer to which the info from the
  // helperBuffer should be copied to. This location is right after all the
  // tags for the current IFD have been written and after the 4 0's at the
  // end of each IFD.
  helper_buffer_destination = tag_count_offset_ + tag_count_ * kTagSize + 4;
  for (uint32_t i = 0; i < tag_count_; i++) {
    uint32_t tag_data_offset = kTagIdSize + kTagTypeSize + kTagCountSize;
    uint32_t write_offset = tag_count_offset_ + i * kTagSize + tag_data_offset;
    const uint32_t type = ReadU16(exif_buffer_,
        write_offset - (kTagTypeSize + kTagCountSize));
    const uint32_t count = ReadU32(exif_buffer_, write_offset - kTagCountSize);

    if (type >= sizeof(tag_type_sizes)/sizeof(tag_type_sizes[0])) {
        QMMF_ERROR("%s: %s invalid tag type %d in entity %d\n",
            TAG, __func__, type, i);
    } else if (count * tag_type_sizes[type] > 4) {
      uint32_t helper_offset = ReadU32(exif_buffer_, write_offset);
      // Calculate the offset for each exif tag to where the
      // tag data actual value will be stored in the exifBuffer.
      uint32_t adjusted_offset = helper_offset + helper_buffer_destination
          - tiff_header_begin_offset_;
      WriteLong(static_cast<int>(adjusted_offset), exif_buffer_, &write_offset);
    }
  }
  memcpy(exif_buffer_ + helper_buffer_destination, helper_buffer_,
         helper_buffer_offset_);
  current_offset_ = helper_buffer_destination + helper_buffer_offset_;
  helper_buffer_offset_ = 0;
  tag_count_ = 0;
  return;
}

void ExifGenerator::WriteGpsIfd() {
  qmmf_exif_tag_t tag;
  memset(&tag, 0, sizeof(qmmf_exif_tag_t));
  tag_count_ = 0;
  tag_count_offset_ = current_offset_;
  current_offset_ += 2;

  if (gps_coords_present_) {
    tag.entry.count = ARRAY_SIZE(tag_values_.latitude);
    tag.entry.data._rats = tag_values_.latitude;
    tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_RATIONAL;
    tag.id = QMMF_EXIFTAGID_GPS_LATITUDE;
    WriteExifTag(&tag);

    tag.entry.count = ARRAY_SIZE(tag_values_.latitude_reference);
    tag.entry.data._ascii = tag_values_.latitude_reference;
    tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_ASCII;
    tag.id = QMMF_EXIFTAGID_GPS_LATITUDE_REF;
    WriteExifTag(&tag);

    tag.entry.count = ARRAY_SIZE(tag_values_.longitude);
    tag.entry.data._rats = tag_values_.longitude;
    tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_RATIONAL;
    tag.id = QMMF_EXIFTAGID_GPS_LONGITUDE;
    WriteExifTag(&tag);

    tag.entry.count = ARRAY_SIZE(tag_values_.longitude_reference);
    tag.entry.data._ascii = tag_values_.longitude_reference;
    tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_ASCII;
    tag.id = QMMF_EXIFTAGID_GPS_LONGITUDE_REF;
    WriteExifTag(&tag);

    tag.entry.count = 1;
    tag.entry.data._rat = tag_values_.altitude;
    tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_RATIONAL;
    tag.id = QMMF_EXIFTAGID_GPS_ALTITUDE;
    WriteExifTag(&tag);

    tag.entry.count = 1;
    tag.entry.data._byte = tag_values_.altitude_reference;
    tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_BYTE;
    tag.id = QMMF_EXIFTAGID_GPS_ALTITUDE_REF;
    WriteExifTag(&tag);
  }

  if (gps_timestamp_present_) {
    tag.entry.count = 3;
    tag.entry.data._rats = tag_values_.gps_timestamp;
    tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_RATIONAL;
    tag.id = QMMF_EXIFTAGID_GPS_TIMESTAMP;
    WriteExifTag(&tag);

    tag.entry.count = (uint32_t)(strlen(tag_values_.gps_datestamp) + 1);
    tag.entry.data._ascii = tag_values_.gps_datestamp;
    tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_ASCII;
    tag.id = QMMF_EXIFTAGID_GPS_DATESTAMP;
    WriteExifTag(&tag);
  }

  if (gps_proc_method_present_) {
    tag.entry.count = gps_proc_method_size_;
    tag.entry.data._ascii = tag_values_.gps_processing_method;
    tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_ASCII;
    tag.id = QMMF_EXIFTAGID_GPS_PROCESSINGMETHOD;
    WriteExifTag(&tag);
  }

  EndWritingIfd();
  return;
}

void ExifGenerator::WriteInteropIfd() {
  qmmf_exif_tag_t tag;
  memset(&tag, 0, sizeof(qmmf_exif_tag_t));
  tag_count_ = 0;
  tag_count_offset_ = current_offset_;
  current_offset_ += 2;

  tag.entry.type = QMMF_EXIF_ASCII;
  tag.entry.copy = 0;
  tag.entry.count = 4;
  tag.entry.data._ascii = (char*)kDefaultVersionName;
  tag.id = QMMF_CONSTRUCT_TAGID(QMMF_EXIF_TAG_MAX_OFFSET, 0x0001);
  WriteExifTag(&tag);

  tag.entry.type = QMMF_EXIF_UNDEFINED;
  tag.entry.copy = 0;
  tag.entry.count = 4;
  tag.entry.data._ascii = (char*)kDefaultVersion;
  tag.id = QMMF_CONSTRUCT_TAGID(QMMF_EXIF_TAG_MAX_OFFSET, 0x0002);
  WriteExifTag(&tag);

  EndWritingIfd();
  if (current_offset_ & 1) {
    WriteByte(0, exif_buffer_, &current_offset_);
  }
  uint32_t tag_data_offset = kTagIdSize + kTagTypeSize + kTagCountSize;
  gps_ifd_pointer_offset_ += tag_data_offset;
  new_gps_ifd_pointer_offset_ = current_offset_ - tiff_header_begin_offset_;
  return;
}

void ExifGenerator::WriteExifIfd() {
  qmmf_exif_tag_t tag;
  memset(&tag, 0, sizeof(qmmf_exif_tag_t));
  tag_count_ = 0;
  tag_count_offset_ = current_offset_;
  current_offset_ += 2;

  tag.entry.count = 1;
  tag.entry.data._rat = tag_values_.exposure_time;
  tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_RATIONAL;
  tag.id = QMMF_EXIFTAGID_EXPOSURE_TIME;
  WriteExifTag(&tag);

  tag.entry.count = 1;
  tag.entry.data._rat = tag_values_.aperture;
  tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_RATIONAL;
  tag.id = QMMF_EXIFTAGID_F_NUMBER;
  WriteExifTag(&tag);

  tag.entry.count = 1;
  tag.entry.data._short = tag_values_.iso_speed;
  tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_SHORT;
  tag.id = QMMF_EXIFTAGID_ISO_SPEED_RATING;
  WriteExifTag(&tag);

  std::string date_time;
  std::string subsec_time;
  GetDateTime(date_time, subsec_time);

  tag.entry.count = static_cast<uint32_t>(date_time.length() + 1);
  tag.entry.data._ascii = const_cast<char *>(date_time.c_str());
  tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_ASCII;
  tag.id = QMMF_EXIFTAGID_DATE_TIME;
  WriteExifTag(&tag);

  tag.id = QMMF_EXIFTAGID_EXIF_DATE_TIME_ORIGINAL;
  WriteExifTag(&tag);

  tag.id = QMMF_EXIFTAGID_EXIF_DATE_TIME_DIGITIZED;
  WriteExifTag(&tag);

  tag.entry.count = 1;
  tag.entry.data._rat = tag_values_.aperture;
  tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_RATIONAL;
  tag.id = QMMF_EXIFTAGID_APERTURE;
  WriteExifTag(&tag);

  tag.entry.data._rat = tag_values_.focal_length;
  tag.id = QMMF_EXIFTAGID_FOCAL_LENGTH;
  WriteExifTag(&tag);

  tag.entry.count = static_cast<uint32_t>(subsec_time.length() + 1);
  tag.entry.data._ascii = const_cast<char *>(subsec_time.c_str());
  tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_ASCII;
  tag.id = QMMF_EXIFTAGID_SUBSEC_TIME;
  WriteExifTag(&tag);

  tag.id = QMMF_EXIFTAGID_SUBSEC_TIME_ORIGINAL;
  WriteExifTag(&tag);

  tag.id = QMMF_EXIFTAGID_SUBSEC_TIME_DIGITIZED;
  WriteExifTag(&tag);

  tag.entry.count = 1;
  tag.entry.data._long = width_;
  tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_LONG;
  tag.id = QMMF_EXIFTAGID_EXIF_PIXEL_X_DIMENSION;
  WriteExifTag(&tag);

  tag.entry.data._long = height_;
  tag.id = QMMF_EXIFTAGID_EXIF_PIXEL_Y_DIMENSION;
  WriteExifTag(&tag);

  interop_ifd_pointer_offset_ = current_offset_;
  tag.entry.data._long = 0;
  tag.id = QMMF_EXIFTAGID_INTEROP_IFD_PTR;
  WriteExifTag(&tag);

  tag.entry.count = 1;
  tag.entry.data._short = tag_values_.aeMode;
  tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_SHORT;
  tag.id = QMMF_EXIFTAGID_EXPOSURE_MODE;
  WriteExifTag(&tag);

  tag.entry.count = 1;
  tag.entry.data._short = tag_values_.awb_mode;
  tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_SHORT;
  tag.id = QMMF_EXIFTAGID_WHITE_BALANCE;
  WriteExifTag(&tag);

  EndWritingIfd();
  uint32_t tag_data_offset = kTagIdSize + kTagTypeSize + kTagCountSize;
  interop_ifd_pointer_offset_ += tag_data_offset;
  new_interop_ifd_pointer_offset_ = current_offset_ - tiff_header_begin_offset_;
  return;
}

void ExifGenerator::Write0thIfd() {
  qmmf_exif_tag_t tag;
  memset(&tag, 0, sizeof(qmmf_exif_tag_t));
  tag_count_ = 0;
  tag_count_offset_ = current_offset_;
  current_offset_ += 2;

  tag.entry.count = static_cast<uint32_t>(vendor_name_.length() + 1);
  tag.entry.data._ascii = const_cast<char *>(vendor_name_.c_str());
  tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_ASCII;
  tag.id = QMMF_EXIFTAGID_MAKE;
  WriteExifTag(&tag);

  tag.entry.count = static_cast<uint32_t>(product_name_.length() + 1);
  tag.entry.data._ascii = const_cast<char *>(product_name_.c_str());
  tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_ASCII;
  tag.id = QMMF_EXIFTAGID_MODEL;
  WriteExifTag(&tag);

  tag.entry.count = static_cast<uint32_t>(product_name_.length() + 1);
  tag.entry.data._ascii = const_cast<char *>(product_name_.c_str());
  tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIF_ASCII;
  tag.id = QMMF_EXIFTAGID_SOFTWARE;
  WriteExifTag(&tag);

  exif_ifd_pointer_offset_ = current_offset_;
  tag.entry.count = 1;
  tag.entry.data._long = 0;
  tag.entry.type = (qmmf_exif_tag_type_t)QMMF_EXIFTAGTYPE_EXIF_IFD_PTR;
  tag.id = QMMF_EXIFTAGID_EXIF_IFD_PTR;
  WriteExifTag(&tag);

  gps_ifd_pointer_offset_ = current_offset_;
  tag.id = QMMF_EXIFTAGID_GPS_IFD_PTR;
  WriteExifTag(&tag);

  EndWritingIfd();
  if (current_offset_ & 1) {
      WriteByte(0, exif_buffer_, &current_offset_);
  }
  uint32_t tag_data_offset = kTagIdSize + kTagTypeSize + kTagCountSize;
  exif_ifd_pointer_offset_ += tag_data_offset;
  new_exif_ifd_pointer_offset_ = current_offset_ - tiff_header_begin_offset_;
  return;
}

void ExifGenerator::UpdateFields() {
  WriteLong(static_cast<int32_t>(new_exif_ifd_pointer_offset_), exif_buffer_,
      &exif_ifd_pointer_offset_);
  WriteLong(static_cast<int32_t>(new_interop_ifd_pointer_offset_),
      exif_buffer_, &interop_ifd_pointer_offset_);
  WriteLong(static_cast<int32_t>(new_gps_ifd_pointer_offset_), exif_buffer_,
      &gps_ifd_pointer_offset_);
  OverwriteShort(static_cast<uint16_t>(current_offset_ - app_1begin_offset_),
      exif_buffer_, app_1begin_offset_);
}

void ExifGenerator::GetGpsProcessingMethod(const char *gps_proc_method) {
    if (gps_proc_method != nullptr) {
        memcpy(tag_values_.gps_processing_method, kASCIICharacterCode,
                kASCIICharacterCodeSize);
        gps_proc_method_size_ = kASCIICharacterCodeSize;
        memcpy(tag_values_.gps_processing_method + kASCIICharacterCodeSize,
               gps_proc_method, strlen(gps_proc_method));
        gps_proc_method_size_ += static_cast<uint32_t>(strlen(gps_proc_method));
        tag_values_.gps_processing_method[gps_proc_method_size_++] = '\0';
    } else {
        QMMF_ERROR("%s: %s No gps processing method found.", TAG, __func__);
    }
}

void ExifGenerator::GetGpsDateTimestamp(int64_t utc_time) {
  time_t unix_time = static_cast<time_t>(utc_time);
  struct tm *timestamp = gmtime(&unix_time);
  if (timestamp != nullptr) {
    strftime(tag_values_.gps_datestamp, 20, "%Y:%m:%d", timestamp);

    tag_values_.gps_timestamp[0].num = timestamp->tm_hour;
    tag_values_.gps_timestamp[0].denom = 1;
    tag_values_.gps_timestamp[1].num = timestamp->tm_min;
    tag_values_.gps_timestamp[1].denom = 1;
    tag_values_.gps_timestamp[2].num = timestamp->tm_sec;
    tag_values_.gps_timestamp[2].denom = 1;
  } else {
    QMMF_ERROR("%s: %s: Unable to get UTC timestamp", TAG, __func__);
  }
}

void ExifGenerator::ParseGpsCoordinates(double coord, qmmf_exif_rat_t *degree) {
  double deg = coord;
  if (deg < 0) {
      deg = -deg;
  }
  double min = (deg - static_cast<int>(deg)) * 60;
  double sec = (min - static_cast<int>(min)) * 60;

  degree[0].num = deg;
  degree[0].denom = 1;
  degree[1].num = min;
  degree[1].denom = 1;
  degree[2].num = sec * 10000;
  degree[2].denom = 10000;
}

void ExifGenerator::GetGpsCoordinates(double latitude, double longitude,
                                      double altitude) {
  // Construct latitude and latitude ref tag values.
  ParseGpsCoordinates(latitude, tag_values_.latitude);
  if (latitude < 0.0f) {
    tag_values_.latitude_reference[0] = 'S';
  } else {
    tag_values_.latitude_reference[0] = 'N';
  }
  tag_values_.latitude_reference[1] = '\0';

  // Construct longitude and longitude ref tag values.
  ParseGpsCoordinates(longitude, tag_values_.longitude);
  if (longitude < 0.0f) {
    tag_values_.longitude_reference[0] = 'W';
  } else {
    tag_values_.longitude_reference[0] = 'E';
  }
  tag_values_.longitude_reference[1] = '\0';

  // Construct altitude and altitude ref tag values.
  tag_values_.altitude_reference = 0;
  if (altitude < 0) {
    tag_values_.altitude_reference = 1;
    altitude = -altitude;
  }
  tag_values_.altitude.num = altitude * 1000;
  tag_values_.altitude.denom = 1000;
}

void ExifGenerator::GetDateTime(std::string &date_time,
                                std::string &subsec_time) {
  // Get time and date from system.
  struct timeval tv;
  struct tm time_info;

  gettimeofday(&tv, nullptr);
  localtime_r(&tv.tv_sec, &time_info);

  char tmp[20];
  // Extract datetime value according to EXIF Spec
  // "YYYY:MM:DD HH:MM:SS" (20 chars including \0).
  sprintf(tmp, "%04u:%02u:%02u %02u:%02u:%02u",
          time_info.tm_year + 1900, time_info.tm_mon + 1,
          time_info.tm_mday, time_info.tm_hour,
          time_info.tm_min, time_info.tm_sec);
  date_time = tmp;
  // Extract subsec according to EXIF Sepc
  sprintf(tmp, "%06lu", tv.tv_usec);
  subsec_time = tmp;
}

void ExifGenerator::GetOrientation(int32_t orientation_in_degrees) {
  switch (orientation_in_degrees) {
  case 0:
  default:
    tag_values_.orientation = TOP_LEFT;
    break;
  case 90:
    tag_values_.orientation = RIGHT_TOP;
    break;
  case 180:
    tag_values_.orientation = BOTTOM_RIGHT;
    break;
  case 270:
    tag_values_.orientation = LEFT_BOTTOM;
    break;
  }
  return;
}

void ExifGenerator::ExpTimeToFraction(double value, uint32_t *numerator,
                                      uint32_t *denominator) {
  // Exposure time
  if (value <= 0.0f) {
    *numerator = 0;
    *denominator = 0;
  } else if (value < 1.0f) {
    *numerator = 1;
    *denominator = round(1.0/value);
  } else {
    *numerator = round(value);
    *denominator = 1;
  }
}

status_t ExifGenerator::ExtractExifTagValues() {
  if (meta_->exists(ANDROID_JPEG_ORIENTATION)) {
    GetOrientation(meta_->find(ANDROID_JPEG_ORIENTATION).data.i32[0]);
  } else {
    QMMF_ERROR("%s: %s No orientation setting has been found in metadata!",
        TAG, __func__);
  }

  // Exposure time is received in nanoseconds from the metadata.
  // The value in the exif tag needs to be in seconds,
  // represented as a rational number.
  if (meta_->exists(ANDROID_SENSOR_EXPOSURE_TIME)) {
      int64_t exposure_time =
          meta_->find(ANDROID_SENSOR_EXPOSURE_TIME).data.i64[0];
      if (exposure_time != 0) {
        ExpTimeToFraction(exposure_time / 1000000000.0,
            &tag_values_.exposure_time.num,
            &tag_values_.exposure_time.denom);
      } else {
        QMMF_ERROR("%s: %s Invalid exposure time value!", TAG, __func__);
      }
  } else {
    QMMF_ERROR("%s: %s No exposure time setting has been found in metadata!",
        TAG, __func__);
  }

  if (meta_->exists(ANDROID_SENSOR_SENSITIVITY)) {
    int32_t sensor_sensitivity =
        meta_->find(ANDROID_SENSOR_SENSITIVITY).data.i32[0];
    tag_values_.iso_speed = static_cast<int16_t>(sensor_sensitivity);
  } else {
    QMMF_ERROR("%s: %s No iso speed setting has been found in metadata!", TAG,
        __func__);
  }

  if (meta_->exists(ANDROID_LENS_FOCAL_LENGTH)) {
    float focal_length = meta_->find(ANDROID_LENS_FOCAL_LENGTH).data.f[0];
    tag_values_.focal_length.num = focal_length * 1000;
    tag_values_.focal_length.denom = 1000;
  } else {
    QMMF_ERROR("%s: %s No focal length setting has been found in metadata!",
        TAG, __func__);
  }

  // Aperture is received as f-number unit from the metadata.
  // The value in the exif tag needs to be represented by the
  // corresponding apex value.
  if (meta_->exists(ANDROID_LENS_APERTURE)) {
    float aperture = meta_->find(ANDROID_LENS_APERTURE).data.f[0];
    float apex_value = 2 * log2(aperture);
    tag_values_.aperture.num = static_cast<int32_t>(apex_value * 1000);
    tag_values_.aperture.denom = 1000;
  } else {
    QMMF_ERROR("%s: %s No aperture setting has been found in metadata!", TAG,
        __func__);
  }

  if (meta_->exists(ANDROID_CONTROL_AE_MODE)) {
    uint8_t ae_mode = meta_->find(ANDROID_CONTROL_AE_MODE).data.u8[0];
    if (ae_mode == ANDROID_CONTROL_AE_MODE_ON) {
      tag_values_.aeMode = 0;
    } else {
      tag_values_.aeMode = 1;
    }
  } else {
    QMMF_ERROR("%s: %s No AE mode setting has been found in metadata!", TAG,
        __func__);
  }

  if (meta_->exists(ANDROID_CONTROL_AWB_MODE)) {
    uint8_t awb_mode = meta_->find(ANDROID_CONTROL_AWB_MODE).data.u8[0];
    if (awb_mode == ANDROID_CONTROL_AWB_MODE_AUTO) {
      tag_values_.awb_mode = 0;
    } else {
      tag_values_.awb_mode = 1;
    }
  } else {
    QMMF_ERROR("%s: %s No AWB mode setting has been found in metadata!", TAG,
        __func__);
  }

  if (meta_->exists(ANDROID_JPEG_GPS_COORDINATES)) {
    gps_coords_present_ = true;

    double latitude = meta_->find(ANDROID_JPEG_GPS_COORDINATES).data.d[0];
    double longitude = meta_->find(ANDROID_JPEG_GPS_COORDINATES).data.d[1];
    double altitude = meta_->find(ANDROID_JPEG_GPS_COORDINATES).data.d[2];

    GetGpsCoordinates(latitude, longitude, altitude);
  } else {
    gps_coords_present_ = false;
    QMMF_ERROR("%s: %s No GPS settings have been found in metadata!", TAG,
        __func__);
  }

  if (meta_->exists(ANDROID_JPEG_GPS_TIMESTAMP)) {
    gps_timestamp_present_ = true;
    int64_t gps_timestamp = meta_->find(ANDROID_JPEG_GPS_TIMESTAMP).data.i64[0];
    GetGpsDateTimestamp(gps_timestamp);
  } else {
    gps_timestamp_present_ = false;
    QMMF_ERROR("%s: %s No GPS timestamp has been found in metadata!", TAG,
        __func__);
  }

  if (meta_->exists(ANDROID_JPEG_GPS_PROCESSING_METHOD)) {
    gps_proc_method_present_ = true;

    const uint8_t *gps_proc_method =
        meta_->find(ANDROID_JPEG_GPS_PROCESSING_METHOD).data.u8;
    GetGpsProcessingMethod(reinterpret_cast<const char *>(gps_proc_method));
  } else {
    gps_proc_method_present_ = false;
    QMMF_ERROR("%s: %s: No gps processing method has been found in metadata!",
        TAG, __func__);
  }

  return NO_ERROR;
}

uint32_t ExifGenerator::WriteExifData() {
  // Write APP1 marker
  WriteShort((uint16_t) (0xFF00 | APP1_MARKER), exif_buffer_, &current_offset_);

  // Leave space for writing the APP1 length,
  // remember the current offset as beginning
  // and calculate the length at the end.
  app_1begin_offset_ = current_offset_;
  current_offset_ += 2;

  // Write Exif identifier code
  WriteLong(static_cast<int>(EXIF_HEADER), exif_buffer_, &current_offset_);
  WriteShort(0, exif_buffer_, &current_offset_);

  // Write TIFF header data
  tiff_header_begin_offset_ = current_offset_;
  WriteShort((uint16_t)(TIFF_BIG_ENDIAN), exif_buffer_, &current_offset_);
  WriteShort((uint8_t)(TIFF_HEADER), exif_buffer_, &current_offset_);
  WriteLong(static_cast<int>(ZERO_IFD_OFFSET), exif_buffer_, &current_offset_);

  // Write Ifds - write all the tags (corresponding to each idf),
  // that were set to the output binary file.
  memset(&tag_values_, 0, sizeof(ExifTagValues));
  status_t res = ExtractExifTagValues();
  if (res != NO_ERROR) {
    QMMF_ERROR("%s: %s Error extracting exif tag values!\n", TAG, __func__);
  }
  Write0thIfd();
  WriteExifIfd();
  WriteInteropIfd();
  WriteGpsIfd();

  UpdateFields();

  if (!overflow_flag_) {
    app1_present_ = true;
  }
  if (overflow_flag_ && !app1_present_) {
    QMMF_ERROR("%s: %s Error writing exif data - overflow: %d", TAG, __func__,
        current_offset_);
    return 0;
  }
  return current_offset_;
}

void ExifGenerator::WriteTagValue(qmmf_exif_tag_t *tag, unsigned char *buffer,
                                  uint32_t *offset) {
  uint32_t write_length = tag_type_sizes[tag->entry.type] * tag->entry.count;

  if (tag->entry.type == QMMF_EXIF_ASCII ||
      tag->entry.type == QMMF_EXIF_UNDEFINED) {
    if (write_length < 4) {
        WriteNBytes(reinterpret_cast<uint8_t *>(tag->entry.data._ascii),
                    write_length, buffer, offset);
    } else {
        WriteNBytes(reinterpret_cast<uint8_t *>(tag->entry.data._ascii),
                    tag->entry.count, buffer, offset);
    }
  } else if (tag->entry.count > 1) {
    for (uint32_t i = 0; i < tag->entry.count; ++i) {
      switch (tag->entry.type) {
      case QMMF_EXIF_BYTE:
        WriteByte((uint8_t) tag->entry.data._bytes[i], buffer, offset);
        break;
      case QMMF_EXIF_SHORT:
        WriteShort((uint16_t) tag->entry.data._shorts[i],
            buffer, offset);
        break;
      case QMMF_EXIF_LONG:
        WriteLong(static_cast<int>(tag->entry.data._longs[i]),
            buffer, offset);
        break;
      case QMMF_EXIF_SLONG:
        WriteLong(static_cast<int>(tag->entry.data._slongs[i]),
            buffer, offset);
        break;
      case QMMF_EXIF_RATIONAL:
        WriteLong(static_cast<int>(tag->entry.data._rats[i].num),
            buffer, offset);
        WriteLong(
            static_cast<int>(tag->entry.data._rats[i].denom),
            buffer, offset);
        break;
      case QMMF_EXIF_SRATIONAL:
        WriteLong(
            static_cast<int>(tag->entry.data._srats[i].num),
            buffer, offset);
        WriteLong(
            static_cast<int>(tag->entry.data._srats[i].denom),
            buffer, offset);
        break;
      default:
        break;
      }
    }
  } else {
    switch (tag->entry.type) {
    case QMMF_EXIF_BYTE:
      WriteByte((uint8_t) tag->entry.data._byte, buffer, offset);
      break;
    case QMMF_EXIF_SHORT:
      WriteShort((uint16_t) tag->entry.data._short,
          buffer, offset);
      break;
    case QMMF_EXIF_LONG:
      WriteLong(static_cast<int>(tag->entry.data._long),
          buffer, offset);
      break;
    case QMMF_EXIF_SLONG:
      WriteLong(static_cast<int>(tag->entry.data._slong),
          buffer, offset);
      break;
    case QMMF_EXIF_RATIONAL:
      WriteLong(static_cast<int>(tag->entry.data._rat.num),
          buffer, offset);
      WriteLong(static_cast<int>(tag->entry.data._rat.denom),
          buffer, offset);
      break;
    case QMMF_EXIF_SRATIONAL:
      WriteLong(static_cast<int>(tag->entry.data._srat.num),
          buffer, offset);
      WriteLong(static_cast<int>(tag->entry.data._srat.denom),
          buffer, offset);
      break;
    default:
      break;
    }
  }
}

void ExifGenerator::OverwriteShort(uint16_t value, uint8_t *buffer,
                                   uint32_t offset) {
  if (offset + 1 >= max_exif_size_) {
    overflow_flag_ = true;
    return;
  }
  buffer[offset] = (uint8_t) ((value >> 8) & 0xFF);
  buffer[offset + 1] = (uint8_t) (value & 0xFF);
}

void ExifGenerator::WriteByte(uint8_t value, uint8_t *buffer,
                              uint32_t *offset) {
  if (*offset >= max_exif_size_) {
      overflow_flag_ = true;
      return;
  }
  buffer[*offset] = value;
  (*offset) += 1;
}

void ExifGenerator::WriteShort(uint16_t value, uint8_t *buffer,
                               uint32_t *offset) {
  if (*offset + 1 >= max_exif_size_) {
      overflow_flag_ = true;
      return;
  }
  buffer[*offset] = (uint8_t) ((value >> 8) & 0xFF);
  buffer[*offset + 1] = (uint8_t) (value & 0xFF);
  (*offset) += 2;
}

void ExifGenerator::WriteLong(int32_t value, uint8_t *buffer,
                              uint32_t *offset) {
  if (*offset + 3 >= max_exif_size_) {
      overflow_flag_ = true;
      return;
  }
  buffer[*offset] = (uint8_t) ((value >> 24) & 0xFF);
  buffer[*offset + 1] = (uint8_t) ((value >> 16) & 0xFF);
  buffer[*offset + 2] = (uint8_t) ((value >> 8) & 0xFF);
  buffer[*offset + 3] = (uint8_t) (value & 0xFF);
  (*offset) += 4;
}

void ExifGenerator::WriteNBytes(const uint8_t *data, uint32_t count,
                                uint8_t* buffer, uint32_t *offset) {
  if (*offset + count >= max_exif_size_) {
      overflow_flag_ = true;
      return;
  }
  (void) memcpy(buffer + *offset, data, count);
  (*offset) += count;
}


uint16_t ExifGenerator::ReadU16(const uint8_t *buffer, int offset) {
  return (uint16_t) (((uint16_t) buffer[offset] << 8)
      + (uint16_t) buffer[offset + 1]);
}

uint32_t ExifGenerator::ReadU32(const uint8_t *buffer, int offset) {
  return (uint32_t) (((uint32_t) buffer[offset] << 24)
      + ((uint32_t) buffer[offset + 1] << 16)
      + ((uint32_t) buffer[offset + 2] << 8)
      + (uint32_t) buffer[offset + 3]);
}

}  // namespace exif

}  // namespace recorder

}  // namespace qmmf
