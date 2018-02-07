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

#include <sys/types.h>

#include <cstdint>
#include <cstring>

#include <binder/Parcel.h>

#include "include/qmmf-sdk/qmmf_system_params.h"

namespace qmmf {
namespace system {

struct SoundModelInternal : public SoundModel {
  SoundModelInternal() {}
  SoundModelInternal(SoundModel& base) : SoundModel(base) {}
  SoundModelInternal(const SoundModel& base)
      : SoundModel(const_cast<SoundModel&>(base)) {}

  void ToParcel(::android::Parcel* parcel,
                ::android::Parcel::WritableBlob* blob) const {
    parcel->writeInt32(static_cast<int32_t>(device));
    parcel->writeUint32(keywords);
    parcel->writeUint32(size);
    parcel->writeBlob(size, false, blob);
    memcpy(blob->data(), data, size);
  }

  SoundModelInternal& FromParcel(const ::android::Parcel& parcel,
                                 ::android::Parcel::ReadableBlob* blob) {
    device = static_cast<DeviceId>(parcel.readInt32());
    keywords = parcel.readUint32();
    size = parcel.readUint32();
    parcel.readBlob(size, blob);
    data = const_cast<void*>(blob->data());
    return *this;
  }
};

struct ToneInternal : public Tone {
  ToneInternal() {}
  ToneInternal(Tone& base) : Tone(base) {}
  ToneInternal(const Tone& base) : Tone(const_cast<Tone&>(base)) {}

  void ToParcel(::android::Parcel* parcel,
                ::android::Parcel::WritableBlob* blob) const {
    parcel->writeUint32(delay);
    parcel->writeUint32(loop_num);
    parcel->writeUint32(size);
    parcel->writeBlob(size, false, blob);
    memcpy(blob->data(), buffer, size);
  }

  ToneInternal& FromParcel(const ::android::Parcel& parcel,
                                 ::android::Parcel::ReadableBlob* blob) {
    delay = parcel.readUint32();
    loop_num = parcel.readUint32();
    size = parcel.readUint32();
    parcel.readBlob(size, blob);
    buffer = const_cast<void*>(blob->data());
    return *this;
  }
};

}; // namespace system
}; // namespace qmmf
