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

#pragma once

#include <string>

#include <camera/CameraMetadata.h>

#include "qmmf_exif_base_types.h"

namespace qmmf {

namespace recorder {

namespace exif {

using namespace android;

class ExifGenerator {

public:

  ExifGenerator();

  ~ExifGenerator();

  int32_t Generate(const CameraMetadata &meta,
               const std::string &vendor_name,
               const std::string &product_name,
               int32_t width, int32_t height,
               unsigned char *buffer, uint32_t size);

private:

  /* Write App1 exif data. */
  uint32_t WriteExifData();

  /**
   * After finishing writing the data, updates the pointers
   * to the IFD's with the correct offset. Updates also the
   * app1 offset so that there is information about the size
   * of the written exif section.
   */
  void UpdateFields();
  void Write0thIfd();
  void WriteExifIfd();
  void WriteInteropIfd();
  void WriteGpsIfd();

  /**
   * This function writes the data from the exifEntity structure
   * to exifBuffer in the following order:
   *  - entity/tag id    - 2 bytes
   *  - entity/tag type  - 2 bytes
   *  - entity/tag count - 4 bytes
   *  - entity/tag data  - 4 bytes
   */
  void WriteExifTag(qmmf_exif_tag_t *tag);
  /**
   * Writes the tag's data value to buffer. Handles a special case,
   * specified by the Exif standart:
   * if the size of the information that needs to be written in
   * tag data's field is less than 4 bytes write the value.
   * If it's larger than 4 bytes write the offset to where the
   * data is actually located and where it can be found/read from.
   */
  void WriteTagValue(qmmf_exif_tag_t *tag, unsigned char *buffer,
                     uint32_t *offset);
  /**
   * This function is used to do finish-up work after info for
   * each IFD has been written to the buffer:
   *  - adds offset at the end of each IFD
   *  - updates the value for the number of entities in the IFD
   *  - handles the case of writing a tag's data value bigger than 4 bytes
   */
  void EndWritingIfd();

  void OverwriteShort(uint16_t value, uint8_t *buffer, uint32_t offset);
  void WriteByte(uint8_t value, uint8_t *buffer, uint32_t *offset);
  void WriteShort(uint16_t value, uint8_t *buffer, uint32_t *offset);
  void WriteLong(int32_t value, uint8_t *buffer, uint32_t *offset);
  void WriteNBytes(const uint8_t *data, uint32_t count, uint8_t *buffer,
                   uint32_t *offset);
  uint16_t ReadU16(const uint8_t* buffer, int offset);
  uint32_t ReadU32(const uint8_t *buffer, int offset);

  /**
   * Extract values needed for exif from the metadata.
   * For more detailed explanation of how exif tags are
   * constructed please refer to the Exif standart.
   */
  status_t ExtractExifTagValues();
  void GetOrientation(int32_t orientation_in_degrees);
  /**
   * Use an algorithm to represent the value for exposure time:
   * expTime(ns)/10e9 as a simple fraction.
   */
  void ExpTimeToFraction(double value, uint32_t *numerator,
                         uint32_t *denominator);
  void GetDateTime(std::string &date_time, std::string &subsec_time);
  /**
   * Retrieve values for latitude, latitude ref, longitude,
   * longitude ref, altitude, altitude ref.
   */
  void GetGpsCoordinates(double latitude, double longitude, double altitude);
  /**
   * Converts latitude/longitude values to three rational values,
   * giving the degrees, minutes and seconds.
   */
  void ParseGpsCoordinates(double coord, qmmf_exif_rat_t *degree);
  /**
   * Parses the given time (UTC in seconds since January 1, 1970) to
   * exif tags in UTC (Coordinated Universal Time):
   * TimeStamp is expressed as three RATIONAL values giving the hour,
   *           minute, and second.
   * DateStamp is recording the date information. The format is "YYYY:MM:DD."
   *           The length of the string is 11 bytes including NULL.
   */
  void GetGpsDateTimestamp(int64_t utc_time);
  /**
   * Specifies the exif tag to be in ASCII format. Adds character code
   * (a fixed 8-byte area) at the beginning of the tag data area.
   * TODO: Change GPS processing method tag to UNDEFINED type.
   */
  void GetGpsProcessingMethod(const char *proc_method);

  const CameraMetadata *meta_;

  unsigned char *exif_buffer_;
  unsigned char *helper_buffer_;
  uint32_t width_;
  uint32_t height_;
  uint32_t max_exif_size_;
  uint32_t current_offset_;
  uint32_t helper_buffer_offset_;

  uint32_t tiff_header_begin_offset_;
  uint32_t app_1begin_offset_;
  uint32_t gps_ifd_pointer_offset_;
  uint32_t new_gps_ifd_pointer_offset_;
  uint32_t interop_ifd_pointer_offset_;
  uint32_t new_interop_ifd_pointer_offset_;
  uint32_t exif_ifd_pointer_offset_;
  uint32_t new_exif_ifd_pointer_offset_;

  uint32_t tag_count_;
  uint32_t tag_count_offset_;
  uint8_t app1_present_;
  uint8_t overflow_flag_;

  /**
   * Structure needed for intermediate step
   * for extracting values from metadata
   */
  ExifTagValues tag_values_;
  std::string vendor_name_;
  std::string product_name_;
  bool gps_coords_present_;
  bool gps_timestamp_present_;
  bool gps_proc_method_present_;
  uint32_t gps_proc_method_size_;

  static const uint32_t kMaxExifApp1Length = 0xFFFF;
  static const uint32_t kMarkerSize = 2;
  static const uint32_t kTagSize = 12;
  static const uint32_t kTagIdSize = 2;
  static const uint16_t kTagTypeSize = 2;
  static const uint32_t kTagCountSize = 4;
  static const uint32_t kTagDataSize = 4;
};

}  // namespace exif

}  // namespace recorder

}  // namespace qmmf
