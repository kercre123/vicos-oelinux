/*
 * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <type_traits>

namespace qmmf {

typedef int32_t CodecId;

#define MAX_PLANE 3
#define QMMF_ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))

enum class CodecType {
  kVideoEncoder,
  kVideoDecoder,
  kAudioEncoder,
  kAudioDecoder,
  kImageEncoder,
  kImageDecoder,
};

enum class CodecMimeType {
  kMimeTypeVideoEncAVC,
  kMimeTypeVideoEncHEVC,
  kMimeTypeVideoEncMPEG,
  kMimeTypeVideoDecAVC,
  kMimeTypeVideoDecHEVC,
  kMimeTypeVideoDecMPEG,
  kMimeTypeAudioEncAAC,
  kMimeTypeAudioEncAMR,
  kMimeTypeAudioEncG711,
  kMimeTypeAudioDecAAC,
  kMimeTypeAudioDecAMR,
  kMimeTypeAudioDecG711
};

enum class VideoFormat {
  kHEVC,
  kAVC,
  kYUV,
  kBayerRDI10BIT,
  kBayerRDI12BIT,
  kBayerIdeal,
};

enum class CodecParamType {
  kBitRateType,
  kFrameRateType,
  kInsertIDRType,
  kIDRIntervalType,
  kCamFrameCropType,
  kMarkLtrType,
  kUseLtrType,
  kAudioEffectsParamType,
  kAudioVolumeParamType,
  kDecodeOperatingRate,
  kEnableFrameRepeat,
  kVQZipInfo,
};

enum class AVCProfileType {
  kBaseline,
  kMain,
  kHigh,
};

enum class AVCLevelType {
  kLevel1,
  kLevel1_3,
  kLevel2,
  kLevel2_1,
  kLevel2_2,
  kLevel3,
  kLevel3_1,
  kLevel3_2,
  kLevel4,
  kLevel4_1,
  kLevel4_2,
  kLevel5,
  kLevel5_1,
  kLevel5_2,
};

enum class HEVCProfileType {
  kMain,
};

enum class HEVCLevelType {
  kLevel3,
  kLevel4,
  kLevel5,
  kLevel5_1,
  kLevel5_2,
};

/// @brief Data structure for specifying the initial quantization values to
/// video encoder
struct VideoEncodeInitQP {
  uint32_t    init_IQP;       ///< First Iframe QP
  uint32_t    init_PQP;       ///< First Pframe QP
  uint32_t    init_BQP;       ///< First Bframe QP
  uint32_t    init_QP_mode;   ///< Bit field indicating which frame type(s) shall
                              ///< use the specified initial QP.
                              ///< Bit 0: Enable initial QP for I/IDR
                              ///<       and use value specified in init_IQP
                              ///< Bit 1: Enable initial QP for P
                              ///<       and use value specified in init_PQP
                              ///< Bit 2: Enable initial QP for B
                              ///<       and use value specified in init_BQP

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "init_IQP[" << init_IQP << "] ";
    stream << "init_PQP[" << init_PQP << "] ";
    stream << "init_BQP[" << init_BQP << "] ";
    stream << "init_QP_mode[" << ::std::setbase(16) << init_QP_mode
           << ::std::setbase(10) << "]";
    return stream.str();
  }
};

struct VideoEncodeQPRange {
  uint32_t    min_QP;
  uint32_t    max_QP;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "min_QP[" << min_QP << "] ";
    stream << "max_QP[" << max_QP << "]";
    return stream.str();
  }
};

struct VideoEncodeIPBQPRange {
  uint32_t    min_IQP;
  uint32_t    max_IQP;
  uint32_t    min_PQP;
  uint32_t    max_PQP;
  uint32_t    min_BQP;
  uint32_t    max_BQP;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "min_IQP[" << min_IQP << "] ";
    stream << "max_IQP[" << max_IQP << "] ";
    stream << "min_PQP[" << min_PQP << "] ";
    stream << "max_PQP[" << max_PQP << "] ";
    stream << "min_BQP[" << min_BQP << "] ";
    stream << "max_BQP[" << max_BQP << "]";
    return stream.str();
  }
};

typedef struct VideoQPParams {
  bool                  enable_init_qp;
  VideoEncodeInitQP     init_qp;
  bool                  enable_qp_range;
  VideoEncodeQPRange    qp_range;
  bool                  enable_qp_IBP_range;
  VideoEncodeIPBQPRange qp_IBP_range;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "enable_init_qp[" << ::std::boolalpha << enable_init_qp
           << ::std::noboolalpha << "] ";
    stream << "init_qp[" << init_qp.ToString() << "]";
    stream << "enable_qp_range[" << ::std::boolalpha << enable_qp_range
           << ::std::noboolalpha << "] ";
    stream << "qp_range[" << qp_range.ToString() << "]";
    stream << "enable_qp_IBP_range[" << ::std::boolalpha << enable_qp_IBP_range
           << ::std::noboolalpha << "] ";
    stream << "qp_IBP_range[" << qp_IBP_range.ToString() << "]";
    return stream.str();
  }
} VideoQPParams;

enum class VideoRateControlType {
  kDisable,
  kVariableSkipFrames,
  kVariable,
  kConstantSkipFrames,
  kConstant,
  kMaxBitrate,
  kMaxBitrateSkipFrames,
};

struct AVCParams {
  uint32_t             idr_interval;
  uint32_t             bitrate;
  AVCProfileType       profile;
  AVCLevelType         level;
  VideoRateControlType ratecontrol_type;
  VideoQPParams        qp_params;
  uint32_t             ltr_count;
  uint32_t             hier_layer;
  bool                 prepend_sps_pps_to_idr;
  bool                 insert_aud_delimiter;
  bool                 sar_enabled;
  uint32_t             sar_width;
  uint32_t             sar_height;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "idr_interval[" << idr_interval << "] ";
    stream << "bitrate[" << bitrate << "] ";
    stream << "profile["
           << static_cast<::std::underlying_type<AVCProfileType>::type>(profile)
           << "] ";
    stream << "level["
           << static_cast<::std::underlying_type<AVCLevelType>::type>(level)
           << "] ";
    stream << "ratecontrol_type["
           << static_cast<::std::underlying_type<VideoRateControlType>::type>
                         (ratecontrol_type)
           << "] ";
    stream << "qp_params[" << qp_params.ToString() << "] ";
    stream << "ltr_count[" << ltr_count << "] ";
    stream << "hier_layer[" << hier_layer << "]";
    stream << "prepend_sps_pps_to_idr[" << prepend_sps_pps_to_idr << "]";
    stream << "insert_aud_delimiter[" << insert_aud_delimiter << "]";
    stream << "sar_enabled[" << sar_enabled << "]";
    stream << "sar_width[" << sar_width << "]";
    stream << "sar_height[" << sar_height << "]";
    return stream.str();
  }
};

struct HEVCParams {
  int32_t              idr_interval;
  uint32_t             bitrate;
  HEVCProfileType      profile;
  HEVCLevelType        level;
  VideoRateControlType ratecontrol_type;
  VideoQPParams        qp_params;
  uint32_t             ltr_count;
  uint32_t             hier_layer;
  bool                 prepend_sps_pps_to_idr;
  bool                 sar_enabled;
  uint32_t             sar_width;
  uint32_t             sar_height;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "idr_interval[" << idr_interval << "] ";
    stream << "bitrate[" << bitrate << "] ";
    stream << "profile["
           << static_cast<::std::underlying_type<HEVCProfileType>::type>
                         (profile)
           << "] ";
    stream << "level["
           << static_cast<::std::underlying_type<HEVCLevelType>::type>(level)
           << "] ";
    stream << "ratecontrol_type["
           << static_cast<::std::underlying_type<VideoRateControlType>::type>
                         (ratecontrol_type)
           << "] ";
    stream << "qp_params[" << qp_params.ToString() << "] ";
    stream << "ltr_count[" << ltr_count << "] ";
    stream << "hier_layer[" << hier_layer << "]";
    stream << "prepend_sps_pps_to_idr[" << prepend_sps_pps_to_idr << "]";
    stream << "sar_enabled[" << sar_enabled << "]";
    stream << "sar_width[" << sar_width << "]";
    stream << "sar_height[" << sar_height << "]";
    return stream.str();
  }
};

struct JPEGParams {
  int32_t quality;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "quality[" << quality << "]";
    return stream.str();
  }
};

union VideoCodecParams {
  HEVCParams hevc;
  AVCParams  avc;
  JPEGParams jpeg;

  ::std::string ToString(const VideoFormat key) const {
    ::std::stringstream stream;
    switch (key) {
      case VideoFormat::kHEVC:
        stream << "hevc[" << hevc.ToString() << "]";
        break;
      case VideoFormat::kAVC:
        stream << "avc[" << avc.ToString() << "]";
        break;
      case VideoFormat::kYUV:
      case VideoFormat::kBayerRDI10BIT:
      case VideoFormat::kBayerRDI12BIT:
      case VideoFormat::kBayerIdeal:
        stream << "N/A";
        break;
      default:
        stream << "Invalid Key["
               << static_cast<::std::underlying_type<VideoFormat>::type>(key)
               << "]";
        break;
    }
    return stream.str();
  }
};

struct VideoEncodeIDRInterval {
  int32_t    idr_period;
  int32_t    num_P_frames;
  int32_t    num_B_frames;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "idr_period[" << idr_period << "] ";
    stream << "num_P_frames[" << num_P_frames << "] ";
    stream << "num_B_frames[" << num_B_frames << "]";
    return stream.str();
  }
};

struct VideoEncLtrUse {
  int32_t id;
  int32_t frame;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "id[" << id << "] ";
    stream << "frame[" << frame << "]";
    return stream.str();
  }
};

struct VideoEncIdrInterval {
  int32_t idr_period;
  int32_t num_pframes;
  int32_t num_bframes;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "idr_period[" << idr_period << "] ";
    stream << "num_pframes[" << num_pframes << "] ";
    stream << "num_bframes[" << num_bframes << "]";
    return stream.str();
  }
};

/// @brief Data structure for VQZIp info to initialize the vqzip transcoding
/// with obtained profile,level,Cabac bool from GetParameters function
/// \param: format: fill in this variable, the format of video(Input Variable)
///         avc_vqzip_info: structure for vqzip info for avc(Output variable)
///         avc_vqzip_info.profile: profile of avc
///         avc_vqzip_info.level: level of avc
///         avc_vqzip_info.is_cabac_used: specifies whether cabac encoding is
///                                       used in the bitstream
/// TODO: currently vqzip is supported for AVC format videos only.Once HEVC
/// support comes, the struture for hevc_vqzip_info will be added.
struct VQZipInfo {
  VideoFormat format;
  struct {
    AVCProfileType profile;
    AVCLevelType level;
    bool is_cabac_used;
  } avc_vqzip_info;

  ::std::string ToString() const {
    ::std::stringstream stream;
    switch (format) {
      case VideoFormat::kAVC:
        stream  << "profile["
                << static_cast<::std::underlying_type<AVCProfileType>::type>
                              (avc_vqzip_info.profile)
                << "] ";
        stream  << "level["
                << static_cast<::std::underlying_type<AVCLevelType>::type>
                              (avc_vqzip_info.level)
                << "] ";
        stream  << "is_cabac_used[" << ::std::boolalpha
                << avc_vqzip_info.is_cabac_used << ::std::noboolalpha << "] ";
        break;
      default:
        stream << "Unsupported VideoFormat["
               << static_cast<::std::underlying_type<VideoFormat>::type>
                             (format)
               << "] for VQZip";
        break;
    }
    return stream.str();
  }
};

enum class ImageFormat {
  kJPEG,
  kNV12,
  kBayerIdeal,
  kBayerRDI10BIT,
  kBayerRDI12BIT,
};

enum class ImageMetaDataType {
  kMainImage,
  kThumbnailImage,
  kRawImage,
  kCameraMeta,
};

struct PlaneInfo {
  uint32_t stride;
  uint32_t scanline;
  uint32_t width;
  uint32_t height;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "stride[" << stride << "] ";
    stream << "scanline[" << scanline << "] ";
    stream << "width[" << width << "] ";
    stream << "height[" << height << "]";
    return stream.str();
  }
};

enum class BufferFormat {
  kNV12,
  kNV12UBWC,
  kNV21,
  kBLOB,
  kRAW10,
  kRAW12,
  kRAW16,
  kUnsupported,
};

struct CameraBufferMetaData {
  BufferFormat format;
  uint32_t  num_planes;
  PlaneInfo plane_info[MAX_PLANE];

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "format["
           << static_cast<::std::underlying_type<BufferFormat>::type>(format)
           << "] ";
    stream << "num_planes[" << num_planes << "] ";
    stream << "plane_info[";
    for (uint32_t idx = 0; idx < num_planes; ++idx)
      stream << "[" << plane_info[idx].ToString() << "], ";
    return stream.str();
  }
};

enum class AudioFormat {
  kPCM,
  kAAC,
  kAMR,
  kG711,
};

union CodecFormat {
  VideoFormat video;
  AudioFormat audio;
  ImageFormat image;

  ::std::string ToString(const CodecType key) const {
    ::std::stringstream stream;
    switch (key) {
      case CodecType::kVideoEncoder:
      case CodecType::kVideoDecoder:
        stream << "video["
               << static_cast<::std::underlying_type<VideoFormat>::type>(video)
               << "]";
        break;
      case CodecType::kAudioEncoder:
      case CodecType::kAudioDecoder:
        stream << "audio["
               << static_cast<::std::underlying_type<AudioFormat>::type>(audio)
               << "]";
        break;
      case CodecType::kImageEncoder:
      case CodecType::kImageDecoder:
        stream << "image["
               << static_cast<::std::underlying_type<ImageFormat>::type>(image)
               << "]";
        break;
      default:
        stream << "Invalid Key["
               << static_cast<::std::underlying_type<CodecType>::type>(key)
               << "]";
        break;
    }
    return stream.str();
  }
};

struct CodecInfo {
  CodecType type;
  CodecFormat format;
  CodecId id;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "type["
           << static_cast<::std::underlying_type<CodecType>::type>(type)
           << "] ";
    stream << "format[" << format.ToString(type) << "] ";
    stream << "id[" << id << "]";
    return stream.str();
  }
};

enum class AACFormat {
  kADTS,
  kADIF,
  kRaw,
  kMP4FF,
};

enum class AACMode {
  kAALC,
  kHEVC_v1,
  kHEVC_v2,
};

struct AACParams {
  AACFormat format;
  AACMode mode;
  int32_t frame_length;
  int32_t bit_rate;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "format["
           << static_cast<::std::underlying_type<AACFormat>::type>(format)
           << "]";
    stream << "mode["
           << static_cast<::std::underlying_type<AACMode>::type>(mode)
           << "]";
    stream << "frame_length[" << frame_length << "] ";
    stream << "bit_rate[" << bit_rate << "]";
    return stream.str();
  }
};

struct AMRParams {
  bool    isWAMR;
  int32_t bit_rate;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "frame_length[" << ::std::boolalpha << isWAMR
           << ::std::noboolalpha << "] ";
    stream << "bit_rate[" << bit_rate << "]";
    return stream.str();
  }
};

enum class G711Mode {
  kALaw,
  kMuLaw,
};

struct G711Params {
  G711Mode mode;
  int32_t  bit_rate;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "mode["
           << static_cast<::std::underlying_type<G711Mode>::type>(mode)
           << "]";
    stream << "bit_rate[" << bit_rate << "]";
    return stream.str();
  }
};

union AudioCodecParams {
  AACParams  aac;
  AMRParams  amr;
  G711Params g711;

  ::std::string ToString(const AudioFormat key) const {
    ::std::stringstream stream;
    switch (key) {
      case AudioFormat::kPCM:
       stream << "N/A";
      break;
    case AudioFormat::kAAC:
      stream << "aac[" << aac.ToString() << "]";
      break;
    case AudioFormat::kAMR:
      stream << "amr[" << amr.ToString() << "]";
      break;
    case AudioFormat::kG711:
      stream << "g711[" << g711.ToString() << "]";
      break;
      default:
        stream << "Invalid Key["
               << static_cast<::std::underlying_type<AudioFormat>::type>(key)
               << "]";
        break;
    }
    return stream.str();
  }
};

}; // namespace qmmf
