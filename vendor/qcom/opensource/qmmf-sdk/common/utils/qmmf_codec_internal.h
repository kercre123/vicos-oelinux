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

#include <cstdint>

#include <binder/Parcel.h>

#include "include/qmmf-sdk/qmmf_codec.h"

namespace qmmf {

struct VideoEncodeInitQPInternal : public VideoEncodeInitQP {
  VideoEncodeInitQPInternal() {}
  VideoEncodeInitQPInternal(VideoEncodeInitQP& base)
      : VideoEncodeInitQP(base) {}
  VideoEncodeInitQPInternal(const VideoEncodeInitQP& base)
      : VideoEncodeInitQP(const_cast<VideoEncodeInitQP&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeUint32(init_IQP);
    parcel->writeUint32(init_PQP);
    parcel->writeUint32(init_BQP);
    parcel->writeUint32(init_QP_mode);
  }

  VideoEncodeInitQPInternal& FromParcel(const ::android::Parcel& parcel) {
    init_IQP = parcel.readUint32();
    init_PQP = parcel.readUint32();
    init_BQP = parcel.readUint32();
    init_QP_mode = parcel.readUint32();
    return *this;
  }
};

struct VideoEncodeQPRangeInternal : public VideoEncodeQPRange {
  VideoEncodeQPRangeInternal() {}
  VideoEncodeQPRangeInternal(VideoEncodeQPRange& base)
      : VideoEncodeQPRange(base) {}
  VideoEncodeQPRangeInternal(const VideoEncodeQPRange& base)
      : VideoEncodeQPRange(const_cast<VideoEncodeQPRange&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeUint32(min_QP);
    parcel->writeUint32(max_QP);
  }

  VideoEncodeQPRangeInternal& FromParcel(const ::android::Parcel& parcel) {
    min_QP = parcel.readUint32();
    max_QP = parcel.readUint32();
    return *this;
  }
};

struct VideoEncodeIPBQPRangeInternal : public VideoEncodeIPBQPRange {
  VideoEncodeIPBQPRangeInternal() {}
  VideoEncodeIPBQPRangeInternal(VideoEncodeIPBQPRange& base)
      : VideoEncodeIPBQPRange(base) {}
  VideoEncodeIPBQPRangeInternal(const VideoEncodeIPBQPRange& base)
      : VideoEncodeIPBQPRange(const_cast<VideoEncodeIPBQPRange&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeUint32(min_IQP);
    parcel->writeUint32(max_IQP);
    parcel->writeUint32(min_PQP);
    parcel->writeUint32(max_PQP);
    parcel->writeUint32(min_BQP);
    parcel->writeUint32(max_BQP);
  }

  VideoEncodeIPBQPRangeInternal& FromParcel(const ::android::Parcel& parcel) {
    min_IQP = parcel.readUint32();
    max_IQP = parcel.readUint32();
    min_PQP = parcel.readUint32();
    max_PQP = parcel.readUint32();
    min_BQP = parcel.readUint32();
    max_BQP = parcel.readUint32();
    return *this;
  }
};

struct VideoQPParamsInternal : public VideoQPParams {
  VideoQPParamsInternal() {}
  VideoQPParamsInternal(VideoQPParams& base) : VideoQPParams(base) {}
  VideoQPParamsInternal(const VideoQPParams& base)
      : VideoQPParams(const_cast<VideoQPParams&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeInt32(static_cast<int32_t>(enable_init_qp));
    VideoEncodeInitQPInternal(init_qp).ToParcel(parcel);
    parcel->writeInt32(static_cast<int32_t>(enable_qp_range));
    VideoEncodeQPRangeInternal(qp_range).ToParcel(parcel);
    parcel->writeInt32(static_cast<int32_t>(enable_qp_IBP_range));
    VideoEncodeIPBQPRangeInternal(qp_IBP_range).ToParcel(parcel);
  }

  VideoQPParamsInternal& FromParcel(const ::android::Parcel& parcel) {
    enable_init_qp = static_cast<bool>(parcel.readInt32());
    init_qp = VideoEncodeInitQPInternal().FromParcel(parcel);
    enable_qp_range = static_cast<bool>(parcel.readInt32());
    qp_range = VideoEncodeQPRangeInternal().FromParcel(parcel);
    enable_qp_IBP_range = static_cast<bool>(parcel.readInt32());
    qp_IBP_range = VideoEncodeIPBQPRangeInternal().FromParcel(parcel);
    return *this;
  }
};

struct AVCParamsInternal : public AVCParams {
  AVCParamsInternal() {}
  AVCParamsInternal(AVCParams& base) : AVCParams(base) {}
  AVCParamsInternal(const AVCParams& base)
      : AVCParams(const_cast<AVCParams&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeUint32(idr_interval);
    parcel->writeUint32(bitrate);
    parcel->writeInt32(static_cast<int32_t>(profile));
    parcel->writeInt32(static_cast<int32_t>(level));
    parcel->writeInt32(static_cast<int32_t>(ratecontrol_type));
    VideoQPParamsInternal(qp_params).ToParcel(parcel);
    parcel->writeUint32(ltr_count);
    parcel->writeUint32(hier_layer);
  }

  AVCParamsInternal& FromParcel(const ::android::Parcel& parcel) {
    idr_interval = parcel.readUint32();
    bitrate = parcel.readUint32();
    profile = static_cast<AVCProfileType>(parcel.readInt32());
    level = static_cast<AVCLevelType>(parcel.readInt32());
    ratecontrol_type = static_cast<VideoRateControlType>(parcel.readInt32());
    qp_params = VideoQPParamsInternal().FromParcel(parcel);
    ltr_count = parcel.readUint32();
    hier_layer = parcel.readUint32();
    return *this;
  }
};

struct HEVCParamsInternal : public HEVCParams {
  HEVCParamsInternal() {}
  HEVCParamsInternal(HEVCParams& base) : HEVCParams(base) {}
  HEVCParamsInternal(const HEVCParams& base)
      : HEVCParams(const_cast<HEVCParams&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeUint32(idr_interval);
    parcel->writeUint32(bitrate);
    parcel->writeInt32(static_cast<int32_t>(profile));
    parcel->writeInt32(static_cast<int32_t>(level));
    parcel->writeInt32(static_cast<int32_t>(ratecontrol_type));
    VideoQPParamsInternal(qp_params).ToParcel(parcel);
    parcel->writeUint32(ltr_count);
    parcel->writeUint32(hier_layer);
  }

  HEVCParamsInternal& FromParcel(const ::android::Parcel& parcel) {
    idr_interval = parcel.readUint32();
    bitrate = parcel.readUint32();
    profile = static_cast<HEVCProfileType>(parcel.readInt32());
    level = static_cast<HEVCLevelType>(parcel.readInt32());
    ratecontrol_type = static_cast<VideoRateControlType>(parcel.readInt32());
    qp_params = VideoQPParamsInternal().FromParcel(parcel);
    ltr_count = parcel.readUint32();
    hier_layer = parcel.readUint32();
    return *this;
  }
};

struct JPEGParamsInternal : public JPEGParams {
  JPEGParamsInternal() {}
  JPEGParamsInternal(JPEGParams& base) : JPEGParams(base) {}
  JPEGParamsInternal(const JPEGParams& base)
      : JPEGParams(const_cast<JPEGParams&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeInt32(quality);
  }

  JPEGParamsInternal& FromParcel(const ::android::Parcel& parcel) {
    quality = parcel.readInt32();
    return *this;
  }
};

struct VideoEncodeIDRIntervalInternal : public VideoEncodeIDRInterval {
  VideoEncodeIDRIntervalInternal() {}
  VideoEncodeIDRIntervalInternal(VideoEncodeIDRInterval& base)
      : VideoEncodeIDRInterval(base) {}
  VideoEncodeIDRIntervalInternal(const VideoEncodeIDRInterval& base)
      : VideoEncodeIDRInterval(const_cast<VideoEncodeIDRInterval&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeInt32(idr_period);
    parcel->writeInt32(num_P_frames);
    parcel->writeInt32(num_B_frames);
  }

  VideoEncodeIDRIntervalInternal& FromParcel(const ::android::Parcel& parcel) {
    idr_period = parcel.readInt32();
    num_P_frames = parcel.readInt32();
    num_B_frames = parcel.readInt32();
    return *this;
  }
};

struct VideoEncLtrUseInternal : public VideoEncLtrUse {
  VideoEncLtrUseInternal() {}
  VideoEncLtrUseInternal(VideoEncLtrUse& base) : VideoEncLtrUse(base) {}
  VideoEncLtrUseInternal(const VideoEncLtrUse& base)
      : VideoEncLtrUse(const_cast<VideoEncLtrUse&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeInt32(id);
    parcel->writeInt32(frame);
  }

  VideoEncLtrUseInternal& FromParcel(const ::android::Parcel& parcel) {
    id = parcel.readInt32();
    frame = parcel.readInt32();
    return *this;
  }
};

struct VideoEncIdrIntervalInternal : public VideoEncIdrInterval {
  VideoEncIdrIntervalInternal() {}
  VideoEncIdrIntervalInternal(VideoEncIdrInterval& base)
      : VideoEncIdrInterval(base) {}
  VideoEncIdrIntervalInternal(const VideoEncIdrInterval& base)
      : VideoEncIdrInterval(const_cast<VideoEncIdrInterval&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeInt32(idr_period);
    parcel->writeInt32(num_pframes);
    parcel->writeInt32(num_bframes);
  }

  VideoEncIdrIntervalInternal& FromParcel(const ::android::Parcel& parcel) {
    idr_period = parcel.readInt32();
    num_pframes = parcel.readInt32();
    num_bframes = parcel.readInt32();
    return *this;
  }
};

struct PlaneInfoInternal : public PlaneInfo {
  PlaneInfoInternal() {}
  PlaneInfoInternal(PlaneInfo& base) : PlaneInfo(base) {}
  PlaneInfoInternal(const PlaneInfo& base)
      : PlaneInfo(const_cast<PlaneInfo&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeUint32(stride);
    parcel->writeUint32(scanline);
    parcel->writeUint32(width);
    parcel->writeUint32(height);
  }

  PlaneInfoInternal& FromParcel(const ::android::Parcel& parcel) {
    stride = parcel.readUint32();
    scanline = parcel.readUint32();
    width = parcel.readUint32();
    height = parcel.readUint32();
    return *this;
  }
};

struct CameraBufferMetaDataInternal : public CameraBufferMetaData {
  CameraBufferMetaDataInternal() {}
  CameraBufferMetaDataInternal(CameraBufferMetaData& base)
      : CameraBufferMetaData(base) {}
  CameraBufferMetaDataInternal(const CameraBufferMetaData& base)
      : CameraBufferMetaData(const_cast<CameraBufferMetaData&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeInt32(static_cast<int32_t>(format));
    parcel->writeUint32(num_planes);
    for (uint32_t idx = 0; idx < num_planes; ++idx)
      PlaneInfoInternal(plane_info[idx]).ToParcel(parcel);
  }

  CameraBufferMetaDataInternal& FromParcel(const ::android::Parcel& parcel) {
    format = static_cast<BufferFormat>(parcel.readInt32());
    num_planes = parcel.readUint32();
    for (uint32_t idx = 0; idx < num_planes; ++idx)
      plane_info[idx] = PlaneInfoInternal().FromParcel(parcel);
    return *this;
  }
};

struct CodecInfoInternal : public CodecInfo {
  CodecInfoInternal() {}
  CodecInfoInternal(CodecInfo& base) : CodecInfo(base) {}
  CodecInfoInternal(const CodecInfo& base)
      : CodecInfo(const_cast<CodecInfo&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeInt32(static_cast<int32_t>(type));
    switch (type) {
      case CodecType::kVideoEncoder:
      case CodecType::kVideoDecoder:
        parcel->writeInt32(static_cast<int32_t>(format.video));
        break;
      case CodecType::kAudioEncoder:
      case CodecType::kAudioDecoder:
        parcel->writeInt32(static_cast<int32_t>(format.audio));
        break;
      case CodecType::kImageEncoder:
      case CodecType::kImageDecoder:
        parcel->writeInt32(static_cast<int32_t>(format.image));
        break;
    }
    parcel->writeInt32(static_cast<int32_t>(id));
  }

  CodecInfoInternal& FromParcel(const ::android::Parcel& parcel) {
    type = static_cast<CodecType>(parcel.readInt32());
    switch (type) {
      case CodecType::kVideoEncoder:
      case CodecType::kVideoDecoder:
        format.video = static_cast<VideoFormat>(parcel.readInt32());
        break;
      case CodecType::kAudioEncoder:
      case CodecType::kAudioDecoder:
        format.audio = static_cast<AudioFormat>(parcel.readInt32());
        break;
      case CodecType::kImageEncoder:
      case CodecType::kImageDecoder:
        format.image = static_cast<ImageFormat>(parcel.readInt32());
        break;
    }
    id = static_cast<CodecId>(parcel.readInt32());
    return *this;
  }
};

struct AACParamsInternal : public AACParams {
  AACParamsInternal() {}
  AACParamsInternal(AACParams& base) : AACParams(base) {}
  AACParamsInternal(const AACParams& base)
      : AACParams(const_cast<AACParams&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeInt32(static_cast<int32_t>(format));
    parcel->writeInt32(static_cast<int32_t>(mode));
    parcel->writeInt32(frame_length);
    parcel->writeInt32(bit_rate);
  }

  AACParamsInternal& FromParcel(const ::android::Parcel& parcel) {
    format = static_cast<AACFormat>(parcel.readInt32());
    mode = static_cast<AACMode>(parcel.readInt32());
    frame_length = parcel.readInt32();
    bit_rate = parcel.readInt32();
    return *this;
  }
};

struct AMRParamsInternal : public AMRParams {
  AMRParamsInternal() {}
  AMRParamsInternal(AMRParams& base) : AMRParams(base) {}
  AMRParamsInternal(const AMRParams& base)
      : AMRParams(const_cast<AMRParams&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeInt32(static_cast<int32_t>(isWAMR));
    parcel->writeInt32(bit_rate);
  }

  AMRParamsInternal& FromParcel(const ::android::Parcel& parcel) {
    isWAMR = static_cast<bool>(parcel.readInt32());
    bit_rate = parcel.readInt32();
    return *this;
  }
};

struct G711ParamsInternal : public G711Params {
  G711ParamsInternal() {}
  G711ParamsInternal(G711Params& base) : G711Params(base) {}
  G711ParamsInternal(const G711Params& base)
      : G711Params(const_cast<G711Params&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeInt32(static_cast<int32_t>(mode));
    parcel->writeInt32(bit_rate);
  }

  G711ParamsInternal& FromParcel(const ::android::Parcel& parcel) {
    mode = static_cast<G711Mode>(parcel.readInt32());
    bit_rate = parcel.readInt32();
    return *this;
  }
};

}; // namespace qmmf
