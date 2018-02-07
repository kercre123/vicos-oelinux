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

#include <mutex>
#include <condition_variable>

#include <camera/CameraMetadata.h>
#include <libgralloc/gralloc_priv.h>

#include "qmmf-sdk/qmmf_recorder_params.h"
#include "recorder/src/service/post-process/interface/qmmf_postproc_module.h"
#include "recorder/src/service/qmmf_camera_reprocess.h"
#include "common/cameraadaptor/qmmf_camera3_device_client.h"

#include "qmmf_jpeg_encoder.h"
#include "qmmf_exif_generator.h"


namespace qmmf {

namespace recorder {

using namespace jpegencoder;

class CameraJpeg : public Camera3Thread , public ICameraPostProcess, public exif::ExifGenerator {

 public:

  CameraJpeg();

  ~CameraJpeg();

  status_t Create(const int32_t stream_id,
                  const PostProcParam& input,
                  const PostProcParam& output,
                  const uint32_t frame_rate,
                  const uint32_t num_images,
                  const uint32_t jpeg_quality,
                  const void* static_meta,
                  const PostProcCb& cb,
                  const void* context) override;
  status_t Delete() override;

  void Process(StreamBuffer& in_buffer, StreamBuffer& out_buffer);

  void AddBuff(StreamBuffer in_buffer, StreamBuffer out_buff) override;

  void AddResult(const void* result) override;

  status_t ReturnBuff(StreamBuffer buffer) override;

  status_t GetCapabilities(PostProcCaps *caps) override;

  status_t Start() override;

  status_t Stop();

 private:

  struct Buff {
    StreamBuffer in;
    StreamBuffer out;
  };

  status_t AddJpegHeader(StreamBuffer &buffer);

  /**
   * Get tag's data depending on tag's type. When tag's data is larger than 4
   * bytes the tag data field contains a value that is the offset to where the
   * tag data actually is located in the buffer.
   */
  status_t getTagDataByTagType(const uint8_t *binary, uint32_t &offset,
                               qmmf_exif_tag_t *tag);

  uint32_t readU32(const uint8_t *buffer, uint32_t offset);
  uint16_t readU16(const uint8_t *buffer, uint32_t offset);
  void constructExifTag(uint32_t id, uint32_t count, uint16_t type,
                        uint8_t *data);
  void constructExifTag(uint32_t id, uint32_t count, uint16_t type,
                        uint16_t *data);
  void constructExifTag(uint32_t id, uint32_t count, uint16_t type,
                        uint32_t *data);
  void constructExifTag(uint32_t id, uint32_t count, uint16_t type,
                        qmmf_exif_rat_t *data);
  void constructExifTag(uint32_t id, uint32_t count, uint16_t type,
                        char *data);
  uint32_t getTagIdByExifId(uint32_t exif_id);
  status_t convertExifBinaryToExifInfoStruct(const uint8_t *binary);
  status_t parseIfd(const uint8_t *binary, uint32_t &offset);

  status_t FillMetaInfo(const PostProcParam& input, CameraBufferMetaData* info);

  bool ThreadLoop() override;


  int32_t                  input_stream_id_;

  bool                     reprocess_flag_;
  bool                     ready_to_start_;
  uint32_t                 num_images_;
  uint32_t                 jpeg_quality_;

  JpegEncoder*             jpeg_encoder_;
  PostProcCb               capture_client_cb_;

  std::condition_variable  wait_for_buffer_;
  std::mutex               buffer_lock_;

  std::condition_variable  wait_for_result_;
  std::mutex               result_lock_;

  List<Buff>              input_buffer_;
  std::map<int64_t, CameraMetadata> results_;

  //IExifGenerator*                    exif_generator_;
  Vector<qmmf_exif_tag_t> exif_entities_;
  uint32_t  exif_ifd_ptr_offset_;
  uint32_t  interop_ifd_ptr_offset_;
  uint32_t  gps_ifd_ptr_offset_;
  uint32_t  tiff_header_offset_;
  static const uint32_t kMaxExifApp1Length = 0xFFFF;
  static const uint32_t kTagSize = 12;
  static const uint32_t kTagDataSize = 4;
  static const uint32_t kMaxExifEntries = 23;
  static const uint32_t kTagIdSize = 2;
  static const uint16_t kTagTypeSize = 2;
  static const uint32_t kTagCountSize = 4;

  std::string vendor_name_;
  std::string product_name_;

  static const int32_t kWaitJPEGTimeout = 100000000; // 100 ms
  static const int32_t kFrameTimeout    = 50000000;  // 50 ms.
};

}; //namespace recorder

}; //namespace qmmf
