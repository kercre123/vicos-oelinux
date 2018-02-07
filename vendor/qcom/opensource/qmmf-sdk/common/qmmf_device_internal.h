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
#include <utility>

#include <binder/Parcel.h>

#include "include/qmmf-sdk/qmmf_device.h"

namespace qmmf {

struct DeviceInfoInternal : public DeviceInfo {
  DeviceInfoInternal() {}
  DeviceInfoInternal(DeviceInfo& base) : DeviceInfo(base) {}
  DeviceInfoInternal(const DeviceInfo& base)
      : DeviceInfo(const_cast<DeviceInfo&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeInt32(static_cast<int32_t>(type));
    switch (type) {
      case DeviceType::kVideoIn:
        parcel->writeInt32(static_cast<int32_t>(subtype.video_in));
        break;
      case DeviceType::kVideoOut:
        parcel->writeInt32(static_cast<int32_t>(subtype.video_out));
        break;
      case DeviceType::kAudioIn:
        parcel->writeInt32(static_cast<int32_t>(subtype.audio_in));
        break;
      case DeviceType::kAudioOut:
        parcel->writeInt32(static_cast<int32_t>(subtype.audio_out));
        break;
    }
    parcel->writeInt32(static_cast<int32_t>(id));
  }

  DeviceInfoInternal& FromParcel(const ::android::Parcel& parcel) {
    type = static_cast<DeviceType>(parcel.readInt32());
    switch (type) {
      case DeviceType::kVideoIn:
        subtype.video_in = static_cast<VideoInSubtype>(parcel.readInt32());
        break;
      case DeviceType::kVideoOut:
        subtype.video_out = static_cast<VideoOutSubtype>(parcel.readInt32());
        break;
      case DeviceType::kAudioIn:
        subtype.audio_in = static_cast<AudioInSubtype>(parcel.readInt32());
        break;
      case DeviceType::kAudioOut:
        subtype.audio_out = static_cast<AudioOutSubtype>(parcel.readInt32());
        break;
    }
    id = static_cast<DeviceId>(parcel.readInt32());
    return *this;
  }
};

struct DimensionInternal : public Dimension {
  DimensionInternal() {}
  DimensionInternal(Dimension& base) : Dimension(base) {}
  DimensionInternal(const Dimension& base)
      : Dimension(const_cast<Dimension&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeInt32(::std::get<0>(dimension));
    parcel->writeInt32(::std::get<1>(dimension));
  }

  DimensionInternal& FromParcel(const ::android::Parcel& parcel) {
    int32_t width = parcel.readInt32();
    int32_t height = parcel.readInt32();
    dimension = ::std::pair<int32_t, int32_t>(width, height);
    return *this;
  }
};

struct VideoCapsInternal : public VideoCaps {
  VideoCapsInternal() {}
  VideoCapsInternal(VideoCaps& base) : VideoCaps(base) {}
  VideoCapsInternal(const VideoCaps& base)
      : VideoCaps(const_cast<VideoCaps&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeUint32(static_cast<uint32_t>(dimensions.size()));
    for (const Dimension& dimension : dimensions)
      DimensionInternal(dimension).ToParcel(parcel);
    parcel->writeUint32(static_cast<uint32_t>(frame_rates.size()));
    for (int32_t frame_rate : frame_rates)
      parcel->writeInt32(frame_rate);
    parcel->writeUint32(static_cast<uint32_t>(formats.size()));
    for (ImageFormat format : formats)
      parcel->writeInt32(static_cast<int32_t>(format));
  }

  VideoCapsInternal& FromParcel(const ::android::Parcel& parcel) {
    size_t number_of_elements = static_cast<size_t>(parcel.readUint32());
    for (size_t index = 0; index < number_of_elements; ++index)
      dimensions.push_back(DimensionInternal().FromParcel(parcel));
    number_of_elements = static_cast<size_t>(parcel.readUint32());
    for (size_t index = 0; index < number_of_elements; ++index)
      frame_rates.push_back(parcel.readInt32());
    number_of_elements = static_cast<size_t>(parcel.readUint32());
    for (size_t index = 0; index < number_of_elements; ++index)
      formats.push_back(static_cast<ImageFormat>(parcel.readInt32()));
    return *this;
  }
};

struct AudioCapsInternal : public AudioCaps {
  AudioCapsInternal() {}
  AudioCapsInternal(AudioCaps& base) : AudioCaps(base) {}
  AudioCapsInternal(const AudioCaps& base)
      : AudioCaps(const_cast<AudioCaps&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeUint32(static_cast<uint32_t>(formats.size()));
    for (AudioFormat format : formats)
      parcel->writeInt32(static_cast<int32_t>(format));
    parcel->writeUint32(static_cast<uint32_t>(sample_rates.size()));
    for (int32_t sample_rate : sample_rates)
      parcel->writeInt32(sample_rate);
    parcel->writeUint32(static_cast<uint32_t>(channels.size()));
    for (int32_t channel : channels)
      parcel->writeInt32(channel);
    parcel->writeUint32(static_cast<uint32_t>(bit_depths.size()));
    for (int32_t bit_depth : bit_depths)
      parcel->writeInt32(bit_depth);
  }

  AudioCapsInternal& FromParcel(const ::android::Parcel& parcel) {
    size_t number_of_elements = static_cast<size_t>(parcel.readUint32());
    for (size_t index = 0; index < number_of_elements; ++index)
      formats.push_back(static_cast<AudioFormat>(parcel.readInt32()));
    number_of_elements = static_cast<size_t>(parcel.readUint32());
    for (size_t index = 0; index < number_of_elements; ++index)
      sample_rates.push_back(parcel.readInt32());
    number_of_elements = static_cast<size_t>(parcel.readUint32());
    for (size_t index = 0; index < number_of_elements; ++index)
      channels.push_back(parcel.readInt32());
    number_of_elements = static_cast<size_t>(parcel.readUint32());
    for (size_t index = 0; index < number_of_elements; ++index)
      bit_depths.push_back(parcel.readInt32());
    return *this;
  }
};

struct DeviceCapsInternal : public DeviceCaps {
  DeviceCapsInternal() {}
  DeviceCapsInternal(DeviceCaps& base) : DeviceCaps(base) {}
  DeviceCapsInternal(const DeviceCaps& base)
      : DeviceCaps(const_cast<DeviceCaps&>(base)) {}

  void ToParcel(::android::Parcel* parcel) const {
    parcel->writeInt32(static_cast<int32_t>(type));
    switch (type) {
      case DeviceType::kVideoIn:
      case DeviceType::kVideoOut:
        VideoCapsInternal(video).ToParcel(parcel);
        break;
      case DeviceType::kAudioIn:
      case DeviceType::kAudioOut:
        AudioCapsInternal(audio).ToParcel(parcel);
        break;
    }
  }

  DeviceCapsInternal& FromParcel(const ::android::Parcel& parcel) {
    type = static_cast<DeviceType>(parcel.readInt32());
    switch (type) {
      case DeviceType::kVideoIn:
      case DeviceType::kVideoOut:
        video = VideoCapsInternal().FromParcel(parcel);
        break;
      case DeviceType::kAudioIn:
      case DeviceType::kAudioOut:
        audio = AudioCapsInternal().FromParcel(parcel);
        break;
    }
    return *this;
  }
};

}; // namespace qmmf
