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

#define TAG "RecorderPostProcAlg"

#include <stdio.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <iomanip>

#include "qmmf_postproc_algo.h"

namespace qmmf {

namespace recorder {

using namespace qmmf_alg_plugin;

PostProcAlg::PostProcAlg(std::string lib)
    : Lib_(lib),
      reprocess_flag_(false),
      ready_to_start_(false),
      pass_through_(false) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  try {
    Utils::LoadLib(Lib_, lib_handle_);

    QmmfAlgLoadPlugin LoadPluginFunc;
    Utils::LoadLibHandler(lib_handle_, QMMF_ALG_LIB_LOAD_FUNC, LoadPluginFunc);
    std::vector<uint8_t> calibration_data;
    algo_ = LoadPluginFunc(calibration_data);
  } catch (const std::exception &e) {
    QMMF_ERROR("%s:%s: Error loading: %s exception: %s", TAG, __func__,
        Lib_.c_str(), e.what());
    throw e;
  }

  char prop_val[PROPERTY_VALUE_MAX];
  property_get("persist.qmmf.postproc.skipalgo", prop_val, "0");
  pass_through_ = (0 == atoi(prop_val)) ? false : true;

  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

PostProcAlg::~PostProcAlg() {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);

  buffs_.clear();

  try {
    Utils::UnloadLib(lib_handle_);
  } catch (const std::exception &e) {
    QMMF_ERROR("%s:%s: Error releasing: %s exception: %s", TAG, __func__,
        Lib_.c_str(), e.what());
    throw e;
  }

  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

status_t PostProcAlg::Initialize(const PostProcIOParam &in_param,
                                 const PostProcIOParam &out_param) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  if (ready_to_start_) {
    QMMF_ERROR("%s:%s: Failed: Already configured.", TAG, __func__);
    return BAD_VALUE;
  }

  if (reprocess_flag_) {
    QMMF_ERROR("%s:%s: Failed: Wrong state.", TAG, __func__);
    return BAD_VALUE;
  }

  algo_->SetCallbacks(this);

  ready_to_start_ = true;

  QMMF_INFO("%s:%s: Exit", TAG, __func__);

  return NO_ERROR;
}

PostProcIOParam PostProcAlg::GetInput(const PostProcIOParam &out) {
  Requirements requirements;
  PostProcIOParam input_param = out;

  if (pass_through_) {
    return input_param;
  }

  requirements.width_    = out.width;
  requirements.height_   = out.height;
  requirements.stride_   = out.stride;
  requirements.scanline_ = out.scanline;
  requirements.formats_.push_back(GetAlgFormat(out.format));

  std::vector<Requirements> alg_out = {requirements};
  requirements = algo_->GetInputRequirements(alg_out);

  input_param.width    = requirements.width_;
  input_param.height   = requirements.height_;
  input_param.stride   = requirements.stride_;
  input_param.scanline = requirements.scanline_;
  input_param.format   = GetQmmfFormat(requirements.formats_.front());

  return input_param;
}

status_t PostProcAlg::ValidateOutput(const PostProcIOParam &output) {
  Capabilities caps = algo_->GetCaps();

  if (caps.out_buffer_requirements_.pixel_formats_.
        count(GetAlgFormat(output.format)) == 0) {
    QMMF_ERROR("%s:%s: Output format %d alg %x not supported", TAG, __func__,
        output.format, (unsigned int)GetAlgFormat(output.format));
    return BAD_TYPE;
  }

  return NO_ERROR;
}

status_t PostProcAlg::GetCapabilities(PostProcCaps &caps) {
  Capabilities algo_caps = algo_->GetCaps();

  caps.output_buff_        = algo_caps.out_buffer_requirements_.count_;
  caps.min_width_          = algo_caps.out_buffer_requirements_.min_width_;
  caps.min_height_         = algo_caps.out_buffer_requirements_.min_height_;
  caps.max_width_          = algo_caps.out_buffer_requirements_.min_width_;
  caps.max_height_         = algo_caps.out_buffer_requirements_.min_height_;
  caps.crop_support_       = algo_caps.crop_support_;
  caps.scale_support_      = algo_caps.scale_support_;
  caps.inplace_processing_ = algo_caps.inplace_processing_;
  caps.usage_              = 0;

  for (auto fmt : algo_caps.out_buffer_requirements_.pixel_formats_) {
    caps.formats_.insert(GetQmmfFormat(fmt));
  }

  if (pass_through_) {
    caps.output_buff_        = 0;
    caps.crop_support_       = false;
    caps.scale_support_      = false;
    caps.inplace_processing_ = true;
  }

  return NO_ERROR;
}

status_t PostProcAlg::Start(const int32_t stream_id) {
  QMMF_INFO("%s:%s: Enter %p", TAG, __func__, this);
  if (!ready_to_start_) {
    return BAD_VALUE;
  }

  reprocess_flag_ = true;

  char prop[PROPERTY_VALUE_MAX];
  property_get("persist.qmmf.postproc.dump.in", prop, "0");
  if(atoi(prop) == 0) {
    dump_in_frame_ = false;
  } else {
    dump_in_frame_ = true;
    QMMF_INFO("%s:%s: Enable input frame dump", TAG, __func__);
  }

  property_get("persist.qmmf.postproc.dump.out", prop, "0");
  if(atoi(prop) == 0) {
    dump_out_frame_ = false;
  } else {
    dump_out_frame_ = true;
    QMMF_INFO("%s:%s: Enable output frame dump", TAG, __func__);
  }

  QMMF_INFO("%s:%s: Exit %p", TAG, __func__, this);

  return NO_ERROR;
}

status_t PostProcAlg::Stop() {
  QMMF_INFO("%s:%s: Enter %p", TAG, __func__, this);

  ready_to_start_ = false;

  reprocess_flag_ = false;

  QMMF_INFO("%s:%s: Exit %p", TAG, __func__, this);
  return NO_ERROR;
}

status_t PostProcAlg::Delete() {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);

  algo_->Abort();
  delete algo_;
  algo_ = nullptr;

  reprocess_flag_ = false;
  ready_to_start_ = false;

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;
}

status_t PostProcAlg::Configure(const std::string config_json_data) {
  try {
    algo_->Configure(config_json_data);
  } catch (const std::exception &e) {
    QMMF_ERROR("%s:%s: Error while configuring exception: %s", TAG,
        __func__, e.what());
    return BAD_VALUE;
  }

  return NO_ERROR;
}

status_t PostProcAlg::Process(
    const std::vector<StreamBuffer> &in_buffers,
    const std::vector<StreamBuffer> &out_buffers) {

  if (pass_through_) {
    for (auto iter : in_buffers) {
      listener_->OnFrameReady(iter);
    }
    return NO_ERROR;
  }

  if (reprocess_flag_ == true) {
    std::vector<AlgBuffer> in_alg_buffers;
    auto ret = PrepareAlgBuffer(in_alg_buffers, in_buffers);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: Fail to prepare in buffers", TAG, __func__);
      return BAD_VALUE;
    }

    std::vector<AlgBuffer> out_alg_buffers;
    ret = PrepareAlgBuffer(out_alg_buffers, out_buffers);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: Fail to prepare out buffers", TAG, __func__);
      return BAD_VALUE;
    }

    try {
      algo_->RegisterInputBuffers(in_alg_buffers);
      algo_->RegisterOutputBuffers(out_alg_buffers);
    } catch (const std::exception &e) {
      QMMF_ERROR("%s:%s: Error registering buffers exception: %s", TAG,
          __func__, e.what());
      throw e;
    }

    if (dump_in_frame_ == true) {
      for (auto buf : in_alg_buffers) {
        DumpFrame(buf, true);
      }
    }

    try {
      algo_->Process(in_alg_buffers, out_alg_buffers);
    } catch (const std::exception &e) {
      QMMF_ERROR("%s:%s: Error while processing exception: %s", TAG,
          __func__, e.what());
      algo_->UnregisterInputBuffers(in_alg_buffers);
      algo_->UnregisterOutputBuffers(out_alg_buffers);
      return BAD_VALUE;
    }
  } else {
    for (auto iter : in_buffers) {
      listener_->OnFrameReady(iter);
    }
    for (auto iter : out_buffers ) {
      listener_->OnFrameProcessed(iter);
    }
  }

  return NO_ERROR;
}

void PostProcAlg::OnFrameProcessed(const AlgBuffer &input_buffer) {

  const std::vector<AlgBuffer> buffers = {input_buffer};
  algo_->UnregisterInputBuffers(buffers);

  // return stream buffer to upper layer
  StreamBuffer buf = GetStreamBuffer(input_buffer);
  listener_->OnFrameProcessed(buf);
}

void PostProcAlg::OnFrameReady(const AlgBuffer &output_buffer) {
  if (dump_out_frame_ == true) {
    DumpFrame(output_buffer, false);
  }

  const std::vector<AlgBuffer> buffers = {output_buffer};
  algo_->UnregisterOutputBuffers(buffers);

  // return stream buffer to upper layer
  StreamBuffer buf = GetStreamBuffer(output_buffer);
  listener_->OnFrameReady(buf);
}

void PostProcAlg::OnError(RuntimeError err) {
  QMMF_ERROR("%s:%s: Error %d", TAG, __func__, err);
  listener_->OnError(err);
}

PixelFormat PostProcAlg::GetAlgFormat(BufferFormat format) {
  switch (format) {
  case BufferFormat::kNV12UBWC:
    return kNv12UBWC;
  case BufferFormat::kNV12:
    return kNv12;
  case BufferFormat::kNV21:
    return kNv21;
  case BufferFormat::kBLOB:
    return kJpeg;
  case BufferFormat::kRAW10:
    return kRawBggrMipi10;
  case BufferFormat::kRAW12:
    return kRawBggrMipi12;
  case BufferFormat::kRAW16:
    return kRawBggr16;
  default:
    return kNv12;
  }
}

BufferFormat PostProcAlg::GetQmmfFormat(PixelFormat format) {
  switch (format) {
  case kNv12UBWC:
    return BufferFormat::kNV12UBWC;
  case kNv12:
    return BufferFormat::kNV12;
  case kNv21:
    return BufferFormat::kNV21;
  case kJpeg:
    return BufferFormat::kBLOB;
  case kRawBggrMipi10:
    return BufferFormat::kRAW10;
  case kRawBggrMipi12:
    return BufferFormat::kRAW12;
  case kRawBggr16:
    return BufferFormat::kRAW16;
  default:
    return BufferFormat::kNV12;
  }
}

status_t PostProcAlg::PrepareAlgBuffer(
    std::vector<AlgBuffer> &algo_buffs,
    const std::vector<StreamBuffer> stream_buffs) {

  for (auto stream_buffer : stream_buffs) {
    if (stream_buffer.fd == -1 || stream_buffer.data == nullptr) {
      QMMF_ERROR("%s:%s buffer FD %d address %p", TAG, __func__,
          stream_buffer.fd, stream_buffer.data);
      return BAD_VALUE;
    }

    uint32_t offset = 0;
    std::vector<BufferPlane> planes;
    for (uint32_t i = 0; i < stream_buffer.info.num_planes; i++) {
      // gralloc report stride in pixels except mipi formats, because
      // mipi stride cannot be represent in pixels.
      uint32_t stride_in_bytes = stream_buffer.info.plane_info[i].stride;
      if (stream_buffer.info.format == BufferFormat::kRAW16) {
        stride_in_bytes *= 2; // two bytes per pixel
      }
      BufferPlane plane(stream_buffer.info.plane_info[i].width,
                        stream_buffer.info.plane_info[i].height,
                        stride_in_bytes,
                        offset,
                        stream_buffer.info.plane_info[i].scanline *
                            stride_in_bytes);
      planes.push_back(plane);
      offset += stream_buffer.info.plane_info[i].scanline *
          stride_in_bytes;
    }

    QMMF_INFO("%s:%s Buffer format: %d", TAG, __func__, stream_buffer.info.format);
    AlgBuffer buf(reinterpret_cast<uint8_t*>(stream_buffer.data),
                  stream_buffer.fd,
                  stream_buffer.size,
                  false,
                  GetAlgFormat(stream_buffer.info.format),
                  stream_buffer.timestamp,
                  stream_buffer.frame_number,
                  planes);

    algo_buffs.push_back(buf);

    std::lock_guard<std::mutex> lock(buffs_lock_);

    // store stream buffer because we need to return this buffer to upper layer
    if (buffs_.count(stream_buffer.fd) != 0) {
      QMMF_ERROR("%s:%s Failed to add FD %d", TAG, __func__, stream_buffer.fd);
      return BAD_VALUE;
    }
    buffs_[stream_buffer.fd] = stream_buffer;
  }

  return NO_ERROR;
}

void PostProcAlg::DumpFrame(AlgBuffer buf, bool input) {
  int32_t start = Lib_.find("libqmmf_alg_") + sizeof("libqmmf_alg_") - 1;
  int32_t size = Lib_.find(".so") - start;

  std::string module_name;
  if (start < 0 || size < 0) {
    module_name = "unknown";
  } else {
    module_name = Lib_.substr(start, size);
  }

  std::string file_name = "/data/misc/qmmf/img_algo_" + module_name + "_";

  switch (buf.pix_fmt_) {
    case kRawBggrMipi10:
    case kRawGbrgMipi10:
    case kRawGrbgMipi10:
    case kRawRggbMipi10:
      file_name += "mipi10";
      break;
    case kRawBggrMipi12:
    case kRawGbrgMipi12:
    case kRawGrbgMipi12:
    case kRawRggbMipi12:
      file_name += "mipi12";
      break;
    case kRawBggr10:
    case kRawGbrg10:
    case kRawGrbg10:
    case kRawRggb10:
      file_name += "plain16_10bit";
      break;
    case kRawBggr12:
    case kRawGbrg12:
    case kRawGrbg12:
    case kRawRggb12:
      file_name += "plain16_12bit";
      break;
    case kNv12:
      file_name += "nv12";
      break;
    case kNv12UBWC:
      file_name += "nv12bwc";
      break;
    case kNv21:
      file_name += "nv21";
      break;
    case kNv21UBWC:
      file_name += "nv21bwc";
      break;
    case kJpeg:
      file_name += "jpeg";
      break;
    default:
      std::stringstream sstream;
      sstream << std::hex << buf.pix_fmt_;
      file_name += sstream.str();
      break;
  }

  file_name +=
      "_dim_"      + std::to_string(buf.plane_[0].width_) +
      "x"          + std::to_string(buf.plane_[0].height_) +
      "_stride_"   + std::to_string(buf.plane_[0].stride_) +
      "_scanline_" + std::to_string(buf.plane_[0].length_ /
                                    buf.plane_[0].stride_) +
      "_frame_"    + std::to_string(buf.frame_number_) +
      "_"          + (input ? "input" : "output");

  switch (buf.pix_fmt_) {
    case kRawBggrMipi8:
    case kRawGbrgMipi8:
    case kRawGrbgMipi8:
    case kRawRggbMipi8:
    case kRawBggrMipi10:
    case kRawGbrgMipi10:
    case kRawGrbgMipi10:
    case kRawRggbMipi10:
    case kRawBggrMipi12:
    case kRawGbrgMipi12:
    case kRawGrbgMipi12:
    case kRawRggbMipi12:
    case kRawBggr10:
    case kRawGbrg10:
    case kRawGrbg10:
    case kRawRggb10:
    case kRawBggr12:
    case kRawGbrg12:
    case kRawGrbg12:
    case kRawRggb12:
    case kRawBggr16:
    case kRawGbrg16:
    case kRawGrbg16:
    case kRawRggb16:
      file_name += ".raw";
      break;
    case kNv12:
    case kNv12UBWC:
    case kNv21:
    case kNv21UBWC:
    case kYuyv422i:
    case kYvyu422i:
    case kUyvy422i:
    case kVyuy422i:
    case kYuv420:
    case kYvu420:
    case kYuv420p:
    case kYvu420p:
      file_name += ".yuv";
      break;
    case kJpeg:
      file_name += ".jpg";
      break;
    default:
      file_name += ".bin";
      break;
  }

  FILE *file = fopen(file_name.c_str(), "w+");
  if (!file) {
    QMMF_ERROR("%s:%s Unable to open: %s", TAG, __func__, file_name.c_str());
    return;
  }

  auto written_len = fwrite(buf.vaddr_, sizeof(uint8_t), buf.size_, file);
  if (buf.size_ != written_len) {
    QMMF_ERROR("%s:%s Bad Write error %d", TAG, __func__, errno);
    fclose(file);
    return;
  }
  QMMF_INFO("%s:%s: Dump %s frame to %s\n", TAG, __func__,
      input ? "input" : "output", file_name.c_str());

  fclose(file);
}

StreamBuffer PostProcAlg::GetStreamBuffer(const AlgBuffer &algo_buf) {
  std::lock_guard<std::mutex> lock(buffs_lock_);

  int32_t fd = algo_buf.fd_;
  if (buffs_.count(fd) == 0) {
    QMMF_ERROR("%s:%s Failed to find entry for FD %d ", TAG, __func__, fd);
    assert(0);
  }

  StreamBuffer stream_buf = buffs_.at(fd);
  buffs_.erase(fd);

  return stream_buf;
}

}; // namespace recoder

}; // namespace qmmf
