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

#define LOG_TAG "AVCodecJpegEncode"

#include <cmath>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <dlfcn.h>
#include <fcntl.h>

#include <utils/Errors.h>
#include <utils/RefBase.h>
#include <cutils/properties.h>
#include <hardware/camera3.h>
#include <qcom/display/gralloc_priv.h>
#include <linux/msm_ion.h>
#include <media/hardware/HardwareAPI.h>
#include <mm_jpeg_interface.h>

#include "common/codecadaptor/src/qmmf_avcodec_common.h"
#include "common/codecadaptor/src/qmmf_jpeg_encode.h"
#include "common/utils/qmmf_log.h"

namespace qmmf {
namespace avcodec {
using std::string;
using std::vector;
using std::shared_ptr;
using namespace android;

static const char *kJPEGEncodeLibName = "libmmjpeg_interface.so";
uint8_t JPEGEncoder::kDefautlQTable0[] = {
    16, 11, 10, 16, 24,  40,  51,  61,  12, 12, 14, 19, 26,  58,  60,  55,
    14, 13, 16, 24, 40,  57,  69,  56,  14, 17, 22, 29, 51,  87,  80,  62,
    18, 22, 37, 56, 68,  109, 103, 77,  24, 35, 55, 64, 81,  104, 113, 92,
    49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99};

uint8_t JPEGEncoder::kDefautlQTable1[] = {
    17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99, 47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99};

typedef uint32_t (*jpeg_open_proc_t)(mm_jpeg_ops_t *, mm_jpeg_mpo_ops_t *,
                                     mm_dimension,
                                     cam_related_system_calibration_data_t *);

typedef struct {
  jpeg_open_proc_t jpeg_open_proc_;
  uint32_t handle_;
  mm_dimension pic_size_;
  mm_jpeg_ops_t ops_;
  mm_jpeg_encode_params_t params_;
  mm_jpeg_job_t job_;
  uint32_t job_id_;
  std::mutex encode_lock_;
  std::mutex enc_done_lock_;
  std::condition_variable enc_done_cond_;
} JpegEncoderParams;

void JPEGEncodeCb(jpeg_job_status_t status, uint32_t, uint32_t,
                  mm_jpeg_output_t *output, void *user_data) {
  if (status == JPEG_JOB_STATUS_ERROR) {
    QMMF_ERROR("%s Jpeg Encoder ran into an error", __func__);
  } else {
    JPEGEncoder::EncodeCb(output, user_data);
  }
}

JPEGEncoder::JPEGEncoder()
    : stop_jpeg_(false), cfg_(nullptr), job_result_size_(0) {
  cfg_ = new JpegEncoderParams;
  JpegEncoderParams *cfg = static_cast<JpegEncoderParams *>(cfg_);

  cfg->handle_ = 0;
  cfg->job_id_ = 0;
  // Set Default Values
  jpeg_quality_ = kDefaultJPEGQuality;
  thumbnail_width_ = kDefaultThumbnailWidth;
  thumbnail_height_ = kDefaultThumbnailHeight;
  thumbnail_quality_ = kDefaultThumbnailQuality;
  enable_thumbnail_ = false;

  void *libjpeg_interface = dlopen(kJPEGEncodeLibName, RTLD_NOW);
  if (!libjpeg_interface) {
    QMMF_ERROR("%s could not open jpeg library", __func__);
  } else {
    cfg->jpeg_open_proc_ =
        (jpeg_open_proc_t)dlsym(libjpeg_interface, "jpeg_open");
    if (!cfg->jpeg_open_proc_) {
      QMMF_ERROR("%s could not dlsym jpeg_open", __func__);
    }
  }
  // setup internal config structures. performed only once
  memset(&cfg->params_, 0, sizeof(cfg->params_));
  memset(&cfg->job_, 0, sizeof(cfg->job_));

  cfg->params_.jpeg_cb = JPEGEncodeCb;
  cfg->params_.userdata = this;

  cfg->params_.num_dst_bufs = 1;
  cfg->params_.dest_buf[0].buf_vaddr = nullptr;
  cfg->params_.dest_buf[0].fd = -1;
  cfg->params_.dest_buf[0].index = 0;

  cfg->params_.num_src_bufs = 1;
  cfg->params_.num_tmb_bufs = 0;

  cfg->params_.thumb_quality = thumbnail_quality_;
  cfg->job_.encode_job.dst_index = 0;
  cfg->job_.encode_job.src_index = 0;
  cfg->job_.encode_job.rotation = 0;

  cfg->job_.encode_job.exif_info.numOfEntries = 0;
  cfg->params_.burst_mode = 0;

  // Update Qtables.
  cfg->job_.encode_job.qtable[0].eQuantizationTable =
      OMX_IMAGE_QuantizationTableLuma;
  cfg->job_.encode_job.qtable[1].eQuantizationTable =
      OMX_IMAGE_QuantizationTableChroma;
  cfg->job_.encode_job.qtable_set[0] = 1;
  cfg->job_.encode_job.qtable_set[1] = 1;

  for (int i = 0; i < (int)sizeof(JPEGEncoder::kDefautlQTable0); i++) {
    cfg->job_.encode_job.qtable[0].nQuantizationMatrix[i] =
        JPEGEncoder::kDefautlQTable0[i];
    cfg->job_.encode_job.qtable[1].nQuantizationMatrix[i] =
        JPEGEncoder::kDefautlQTable1[i];
  }
  cfg->job_.job_type = JPEG_JOB_TYPE_ENCODE;
  cfg->job_.encode_job.src_index = 0;
  cfg->job_.encode_job.dst_index = 0;
  cfg->job_.encode_job.thumb_index = 0;
}

JPEGEncoder::~JPEGEncoder() {
  JpegEncoderParams *cfg = static_cast<JpegEncoderParams *>(cfg_);

  if (cfg->handle_) {
    cfg->ops_.close(cfg->handle_);
    cfg->handle_ = 0;
  }
  if (cfg) delete cfg;
}

void JPEGEncoder::FreeMappedBuffers() {
  for (auto it : input_buffers_map_) {
    munmap(static_cast<void *>(it.second.vaddr),
           static_cast<size_t>(it.second.size));
  }
  input_buffers_map_.clear();
}

void JPEGEncoder::FillImgData(const snapshot_info &in_buffer) {
  JpegEncoderParams *cfg = static_cast<JpegEncoderParams *>(cfg_);

  // setting buffers parameters
  uint32_t size = in_buffer.stride * in_buffer.scanline;
  cfg->params_.src_main_buf[0].buf_size = 3 * size / 2;
  cfg->params_.src_main_buf[0].format = MM_JPEG_FMT_YUV;
  cfg->params_.src_main_buf[0].fd = -1;
  cfg->params_.src_main_buf[0].index = 0;
  cfg->params_.src_main_buf[0].offset.mp[0].len = size;
  cfg->params_.src_main_buf[0].offset.mp[0].stride = in_buffer.stride;
  cfg->params_.src_main_buf[0].offset.mp[0].scanline = in_buffer.scanline;
  cfg->params_.src_main_buf[0].offset.mp[1].len = (size >> 1);

  cfg->params_.src_thumb_buf[0] = cfg->params_.src_main_buf[0];
  {
    // this lock is required to protect jpep quality parameter as this parameter
    // can be changed at runtime.
    std::lock_guard<std::mutex> lock(param_lock_);
    cfg->params_.quality = jpeg_quality_;
  }
  switch (in_buffer.format) {
    case BufferFormat::kNV12:
      cfg->params_.color_format = MM_JPEG_COLOR_FORMAT_YCBCRLP_H2V2;
      break;
    case BufferFormat::kNV21:
      cfg->params_.color_format = MM_JPEG_COLOR_FORMAT_YCRCBLP_H2V2;
      break;
    default:
      break;
  }
  cfg->params_.thumb_color_format = cfg->params_.color_format;
  cfg->params_.dest_buf[0].buf_size = cfg->params_.src_main_buf[0].buf_size;

  cfg->job_.encode_job.main_dim.src_dim.width = in_buffer.stride;
  cfg->job_.encode_job.main_dim.src_dim.height = in_buffer.scanline;
  cfg->job_.encode_job.main_dim.dst_dim.width = in_buffer.width;
  cfg->job_.encode_job.main_dim.dst_dim.height = in_buffer.height;

  cfg->job_.encode_job.main_dim.crop.top = 0;
  cfg->job_.encode_job.main_dim.crop.left = 0;
  cfg->job_.encode_job.main_dim.crop.width = in_buffer.width;
  cfg->job_.encode_job.main_dim.crop.height = in_buffer.height;
  cfg->params_.main_dim = cfg->job_.encode_job.main_dim;

  // Thumbnail settings
  cfg->params_.encode_thumbnail = enable_thumbnail_;
  if (cfg->params_.encode_thumbnail) {
    cfg->params_.num_tmb_bufs = cfg->params_.num_src_bufs;
    cfg->job_.encode_job.thumb_dim.src_dim.width =
        VENUS_Y_STRIDE(COLOR_FMT_NV12, thumbnail_width_);
    cfg->job_.encode_job.thumb_dim.src_dim.height =
        VENUS_Y_STRIDE(COLOR_FMT_NV12, thumbnail_height_);
    cfg->job_.encode_job.thumb_dim.dst_dim.width = thumbnail_width_;
    cfg->job_.encode_job.thumb_dim.dst_dim.height = thumbnail_height_;
    cfg->job_.encode_job.thumb_dim.crop.top = 0;
    cfg->job_.encode_job.thumb_dim.crop.left = 0;
    cfg->job_.encode_job.thumb_dim.crop.width = 0;
    cfg->job_.encode_job.thumb_dim.crop.height = 0;
    cfg->params_.thumb_dim = cfg->job_.encode_job.thumb_dim;
    cfg->params_.thumb_quality = thumbnail_quality_;
    cfg->params_.src_thumb_buf[0].buf_vaddr =
        static_cast<uint8_t *>(in_buffer.img_in_buf.data);
  }
  // Actual Pic Size
  cfg->pic_size_.w = in_buffer.width;
  cfg->pic_size_.h = in_buffer.height;

  // Buffer assignment
  cfg->params_.src_main_buf[0].buf_vaddr =
      static_cast<uint8_t *>(in_buffer.img_in_buf.data);

  cfg->params_.dest_buf[0].buf_vaddr =
      static_cast<uint8_t *>(in_buffer.img_out_buf.data);
  cfg->job_id_ = 0;
}

status_t JPEGEncoder::Encode(const snapshot_info &in_buffer, size_t &jpeg_size) {

  JpegEncoderParams *cfg = static_cast<JpegEncoderParams *>(cfg_);

  std::lock_guard<std::mutex> l(cfg->encode_lock_);
  if (in_buffer.img_in_buf.data == nullptr ||
      in_buffer.img_out_buf.data == nullptr) {
    QMMF_ERROR("%s can't pass nullptr plane pointer", __func__);
    return BAD_VALUE;
  }

  FillImgData(in_buffer);

  // Create OMX session
  auto ret = cfg->ops_.create_session(cfg->handle_, &cfg->params_,
                                      &cfg->job_.encode_job.session_id);
  if (cfg->job_.encode_job.session_id == 0) {
    QMMF_ERROR("%s Could not create Jpeg Session", __func__);
    return ret;
  }

  // Start Encoding Job
  if (!cfg->ops_.start_job(&cfg->job_, &cfg->job_id_)) {
    std::unique_lock<std::mutex> ul(cfg->enc_done_lock_);
    std::chrono::nanoseconds wait_time(kJPEGEncodeWaitTime);
    if (cfg->enc_done_cond_.wait_for(ul, wait_time) ==
        std::cv_status::timeout) {
      QMMF_ERROR("%s JPEG Encode Time Out Happened", __func__);
      ret = TIMED_OUT;
      goto jpeg_encode_exit;
    }
    jpeg_size = job_result_size_;
    job_result_size_ = 0;
  } else {
    QMMF_ERROR("%s could not start encode job", __func__);
    goto jpeg_encode_exit;
  }
// Clean Up
jpeg_encode_exit:
  if (cfg->job_.encode_job.session_id) {
    cfg->ops_.destroy_session(cfg->job_.encode_job.session_id);
  }
  return ret;
}

void JPEGEncoder::EncodeCb(void *output, void *user_data) {
  JPEGEncoder *enc = static_cast<JPEGEncoder *>(user_data);
  mm_jpeg_output_t *jpeg_output = static_cast<mm_jpeg_output_t *>(output);
  enc->job_result_size_ = jpeg_output->buf_filled_len;

  JpegEncoderParams *cfg = static_cast<JpegEncoderParams *>(enc->cfg_);
  std::unique_lock<std::mutex> ul(cfg->enc_done_lock_);
  cfg->enc_done_cond_.notify_one();
}

bool JPEGEncoder::IsInputStop() {
  std::lock_guard<std::mutex> l(stop_jpeg_mutex_);
  return stop_jpeg_;
}

void *JPEGEncoder::JpegEncodeThread(void *arg) {
  status_t ret = 0;
  JPEGEncoder *jpeg_encode = static_cast<JPEGEncoder *>(arg);
  BufferDescriptor buffer_in, buffer_out;
  snapshot_info img_buffer;
  bool thread_stop = false;

  // The purpose of this function is to Handle JPEG Encode.
  // Steps :
  // 1.Take input stream buffer.
  // 2.Map the buffer to current process.
  // 3.For mapping, caching has been enabled.
  // 4.Take the output stream buffer.
  // 5.Send the buffers for Jpeg Encode
  // 6.Stop logic

  while (thread_stop != true) {
    if (jpeg_encode->IsInputStop()) {
      thread_stop = true;
      break;
    }
    memset(&buffer_in, 0x0, sizeof(buffer_in));
    // Get a YUV Buffer from Track source
    ret = jpeg_encode->getInputBufferSource()->GetBuffer(buffer_in, nullptr);
    if (ret != 0) {
      QMMF_ERROR("%s InputSource Read failed", __func__);
      thread_stop = true;
    }

    buffer_handle_t native_handle_in;
    memset(&native_handle_in, 0x0, sizeof native_handle_in);
    native_handle_in = static_cast<buffer_handle_t>(buffer_in.data);

    // Get a Empty Buffer from Output Buffer source
    memset(&buffer_out, 0x0, sizeof(buffer_out));
    ret = jpeg_encode->getOutputBufferSource()->GetBuffer(buffer_out, nullptr);
    assert(buffer_out.data != nullptr);

    void *buf_vaaddr = nullptr;
    // We need to map input buffers.
    // Before mapping we are checking whether that buffer is already mapped.
    // If yes then get the vaddr.
    // If not then do mmap and add the mapping to map.
    auto it = jpeg_encode->input_buffers_map_.find(buffer_in.fd);
    if (it != jpeg_encode->input_buffers_map_.end()) {
      buf_vaaddr = it->second.vaddr;
    } else {
      buf_vaaddr = mmap(nullptr, buffer_in.size, PROT_READ | PROT_WRITE,
                        MAP_SHARED, native_handle_in->data[0], 0);
      if (buf_vaaddr == MAP_FAILED) {
        QMMF_ERROR("%s MAP_FAILED for input", __func__);
        break;
      }
      // Add Entry to map
      mapped_buffer_info instance = {buf_vaaddr, buffer_in.size};
      jpeg_encode->input_buffers_map_.emplace(buffer_in.fd, instance);
    }

    // Set input-output buffer dimension
    size_t jpeg_size = 0;
    memset(&img_buffer, 0x0, sizeof(img_buffer));
    img_buffer.img_in_buf.data = buf_vaaddr;
    img_buffer.img_out_buf.data = buffer_out.data;
    img_buffer.stride = VENUS_Y_STRIDE(
        COLOR_FMT_NV12, jpeg_encode->codec_params_.video_enc_param.width);
    img_buffer.scanline = VENUS_Y_SCANLINES(
        COLOR_FMT_NV12, jpeg_encode->codec_params_.video_enc_param.height);
    img_buffer.width = jpeg_encode->codec_params_.video_enc_param.width;
    img_buffer.height = jpeg_encode->codec_params_.video_enc_param.height;
    img_buffer.format = BufferFormat::kNV12;

    // Send buffer for encode
    jpeg_encode->Encode(img_buffer, jpeg_size);
    if (0 == jpeg_size) {
      QMMF_ERROR("%s: JPEG size is 0!", __func__);
    } else {
      buffer_out.timestamp = buffer_in.timestamp;
      buffer_out.size = jpeg_size;
      camera3_jpeg_blob_t header;
      header.jpeg_blob_id = CAMERA3_JPEG_BLOB_ID;
      header.jpeg_size = jpeg_size;
      // The Jpeg header must be appended at the end of the allocated buffer.
      uint32_t offset = jpeg_size - sizeof(header);
      uintptr_t data = reinterpret_cast<uintptr_t>(buffer_out.data);
      void *vaddr = reinterpret_cast<void *>(data + offset);
      memcpy(vaddr, &header, sizeof(header));
    }

    // send back the buffer to Track Source
    jpeg_encode->getInputBufferSource()->ReturnBuffer(buffer_in, nullptr);
    jpeg_encode->getOutputBufferSource()->ReturnBuffer(buffer_out, nullptr);
  }

  // Stop Logic
  if (thread_stop == true) {
    jpeg_encode->FreeMappedBuffers();
    CodecPortStatus status = CodecPortStatus::kPortStop;
    jpeg_encode->getInputBufferSource()->NotifyPortEvent(
        PortEventType::kPortStatus, static_cast<void *>(&status));
    status = CodecPortStatus::kPortIdle;
    jpeg_encode->getInputBufferSource()->NotifyPortEvent(
        PortEventType::kPortStatus, static_cast<void *>(&status));
  }
  return nullptr;
}

status_t JPEGEncoder::GetComponentName(CodecMimeType mime_type,
                                       uint32_t *num_comps,
                                       vector<string> &comp_names) {
  return 0;
}

status_t JPEGEncoder::ConfigureCodec(CodecMimeType codec_type,
                                     CodecParam &codec_param,
                                     string comp_name) {
  JpegEncoderParams *cfg = static_cast<JpegEncoderParams *>(cfg_);
  codec_params_ = codec_param;
  format_type_ = CodecType::kImageEncoder;
  jpeg_quality_ = codec_params_.video_enc_param.codec_param.jpeg.quality;

  // thumbnail settings
  enable_thumbnail_ =
      codec_params_.video_enc_param.codec_param.jpeg.enable_thumbnail;
  thumbnail_width_ =
      codec_params_.video_enc_param.codec_param.jpeg.thumbnail_width;
  thumbnail_height_ =
      codec_params_.video_enc_param.codec_param.jpeg.thumbnail_height;
  thumbnail_quality_ =
      codec_params_.video_enc_param.codec_param.jpeg.thumbnail_quality;

  cfg->pic_size_.w = codec_param.video_enc_param.width;
  cfg->pic_size_.h = codec_param.video_enc_param.height;

  cfg->handle_ =
      cfg->jpeg_open_proc_(&cfg->ops_, nullptr, cfg->pic_size_, nullptr);
  if (cfg->handle_ == 0) {
    QMMF_ERROR("%s could not open a jpeg handle", __func__);
    if (cfg->handle_) {
      cfg->ops_.close(cfg->handle_);
      cfg->handle_ = 0;
    }
  }
  return NO_ERROR;
}

status_t JPEGEncoder::GetBufferRequirements(uint32_t port_type,
                                            uint32_t *buf_count,
                                            uint32_t *buf_size) {
  if (port_type == kPortIndexOutput) {
    *buf_count = kJPEGBufferCount;
    *buf_size =
        (codec_params_.video_enc_param.width *
         codec_params_.video_enc_param.height * 1.5);  // We need to fix this
  } else {
    // for input port not required.
  }
  QMMF_INFO("%s %s: buf count(%d), buf size(%d)", __func__,
            PORT_NAME(port_type), *buf_count, *buf_size);
  return NO_ERROR;
}

status_t JPEGEncoder::AllocateBuffer(uint32_t port_type, uint32_t buf_count,
                                     uint32_t buf_size,
                                     const shared_ptr<ICodecSource> &source,
                                     vector<BufferDescriptor> &buffer_list) {
  if (port_type == kPortIndexInput) {
    assert(source.get() != nullptr);
    input_source_ = source;
  } else {
    assert(source.get() != nullptr);
    output_source_ = source;
  }
  return NO_ERROR;
}

status_t JPEGEncoder::ReleaseBuffer() {
  input_source_ = nullptr;
  output_source_ = nullptr;
  return NO_ERROR;
}

status_t JPEGEncoder::SetParameters(CodecParamType param_type,
                                    void *codec_param, size_t param_size) {
  uint32_t* quality = nullptr;
  switch (param_type) {
    case CodecParamType::kJPEGQuality:
      quality = static_cast<uint32_t*>(codec_param);
      {
        std::lock_guard<std::mutex> lock(param_lock_);
        jpeg_quality_ = *quality;
      }
      break;
    default:
      QMMF_ERROR("%s Unknown param type", __func__);
      return -1;
  }
  return NO_ERROR;
}

status_t JPEGEncoder::GetParameters(const CodecParamType param_type,
                                    void *codec_param, size_t *param_size) {
  return NO_ERROR;
}

status_t JPEGEncoder::StartCodec() {
  std::lock_guard<std::mutex> l(stop_jpeg_mutex_);
  stop_jpeg_ = false;
  jpeg_thread_id_ = thread(JpegEncodeThread, this);
  return NO_ERROR;
}

status_t JPEGEncoder::StopCodec(bool do_flush) {
  std::lock_guard<std::mutex> l(stop_jpeg_mutex_);
  stop_jpeg_ = true;
  jpeg_thread_id_.join();
  return NO_ERROR;
}

status_t JPEGEncoder::PauseCodec() { return 0; }
status_t JPEGEncoder::ResumeCodec() { return 0; }
status_t JPEGEncoder::RegisterOutputBuffers(vector<BufferDescriptor> &list) {
  output_buffer_list_ = list;
  return NO_ERROR;
}

status_t JPEGEncoder::RegisterInputBuffers(vector<BufferDescriptor> &list) {
  input_buffer_list_ = list;
  return NO_ERROR;
}

status_t JPEGEncoder::Flush(uint32_t port_type) { return NO_ERROR; }

};  // namespace avcodec
};  // namespace qmmf
