/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *     Neither the name of The Linux Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.

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

#include <media/msm_media_info.h>
#include <sys/mman.h>
#include <mutex>
#include <thread>
#include "common/codecadaptor/src/qmmf_avcodec.h"

namespace qmmf {
namespace avcodec {
using ::std::thread;

class JPEGEncoder : public IAVCodec {
 public:
  JPEGEncoder();
  ~JPEGEncoder();
  // methods of IAVCodec
  status_t GetComponentName(CodecMimeType mime_type, uint32_t* num_comps,
                            std::vector<std::string>& comp_names) override;

  status_t ConfigureCodec(CodecMimeType codec_type, CodecParam& codec_param,
                          std::string comp_name = "") override;

  status_t GetBufferRequirements(uint32_t port_type, uint32_t* buf_count,
                                 uint32_t* buf_size) override;

  status_t AllocateBuffer(
      uint32_t port_type, uint32_t buf_count, uint32_t buf_size,
      const ::std::shared_ptr<ICodecSource>& source,
      ::std::vector<BufferDescriptor>& buffer_list) override;

  status_t ReleaseBuffer() override;

  status_t SetParameters(CodecParamType param_type, void* codec_param,
                         size_t param_size) override;

  status_t GetParameters(const CodecParamType param_type, void* codec_param,
                         size_t* param_size) override;

  status_t StartCodec() override;

  status_t StopCodec(bool do_flush) override;

  status_t PauseCodec() override;

  status_t ResumeCodec() override;

  status_t RegisterOutputBuffers(std::vector<BufferDescriptor>& list) override;

  status_t RegisterInputBuffers(::std::vector<BufferDescriptor>& list) override;

  status_t Flush(uint32_t port_type) override;

  static void EncodeCb(void* p_output, void* userData);

 private:
  struct snapshot_info {
    BufferDescriptor img_in_buf;
    BufferDescriptor img_out_buf;
    uint32_t stride;
    uint32_t scanline;
    uint32_t width;
    uint32_t height;
    BufferFormat format;
  };

  struct mapped_buffer_info {
    void* vaddr;
    uint32_t size;
  };

  static void* JpegEncodeThread(void* arg);

  status_t Encode(const snapshot_info& in_buffer, size_t& jpeg_size);

  void FillImgData(const snapshot_info& in_buffer);

  std::shared_ptr<ICodecSource>& getInputBufferSource() {
    return input_source_;
  }

  std::shared_ptr<ICodecSource>& getOutputBufferSource() {
    return output_source_;
  }

  void FreeMappedBuffers();

  bool IsInputStop();

  std::vector<BufferDescriptor> output_buffer_list_;
  std::vector<BufferDescriptor> input_buffer_list_;
  std::shared_ptr<ICodecSource> input_source_;
  std::shared_ptr<ICodecSource> output_source_;

  // Mapping of fd and virtual address
  std::map<int32_t, mapped_buffer_info> input_buffers_map_;

  bool stop_jpeg_;
  std::mutex stop_jpeg_mutex_;

  std::thread jpeg_thread_id_;

  CodecType format_type_;
  CodecParam codec_params_;

  void* cfg_;
  size_t job_result_size_;

  uint32_t    jpeg_quality_;
  uint32_t    thumbnail_width_;
  uint32_t    thumbnail_height_;
  uint32_t    thumbnail_quality_;
  bool        enable_thumbnail_;
  std::mutex  param_lock_;

  static uint8_t kDefautlQTable0[];
  static uint8_t kDefautlQTable1[];
  static const uint32_t kJPEGBufferCount = 5;
  static const uint32_t kDefaultJPEGQuality = 95;
  static const uint32_t kDefaultThumbnailWidth = 320;
  static const uint32_t kDefaultThumbnailHeight = 240;
  static const uint32_t kDefaultThumbnailQuality = 75;
  static const uint32_t kJPEGEncodeWaitTime = 100000000;  // 100ms
};
};  // namespace avcodec
};  // namespace qmmf
