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
#ifndef QMMF_CAMERA_CONFIGURATION_H_
#define QMMF_CAMERA_CONFIGURATION_H_

#include <qmmf-sdk/qmmf_recorder.h>
#include <qmmf-sdk/qmmf_recorder_params.h>
#include <camera/CameraMetadata.h>
#include <utils/Vector.h>
#include <utils/KeyedVector.h>
#include <utils/String8.h>

namespace qmmf {

namespace httpinterface {

using namespace android;

class CameraConfiguration {
public:

  CameraConfiguration(CameraMetadata &static_info);
  ~CameraConfiguration();

  int32_t SetNRMode(const char *mode, CameraMetadata &meta);
  int32_t SetHDRMode(const char *mode, CameraMetadata &meta);
  int32_t SetIRMode(const char *mode, CameraMetadata &meta);
  char *GetSupportedNRModes() {return nr_modes_supported_;}
  char *GetSupportedHDRModes() {return hdr_modes_supported_;}
  char *GetSupportedIRModes() {return ir_modes_supported_;}

private:
  void InitSupportedNRModes();
  void InitSupportedHDRModes();
  void InitSupportedIRModes();

  /**Not allowed */
  CameraConfiguration(const CameraConfiguration &);
  CameraConfiguration &operator=(const CameraConfiguration &);

  KeyedVector<String8, camera_metadata_enum_android_noise_reduction_mode_t> nr_parms_to_mode_;
  char * nr_modes_supported_;
  KeyedVector<String8, int32_t> hdr_parms_to_mode_;
  char * hdr_modes_supported_;
  KeyedVector<String8, int32_t> ir_parms_to_mode_;
  char * ir_modes_supported_;
  CameraMetadata &static_info_;
  static const char kModeOff[];
  static const char kModeOn[];
  static const char kNRModeFast[];
  static const char kNRModeHighQuality[];
  static const char kNRModeMinimal[];
  static const char kNRModeZSL[];
};


} //namespace httpinterface ends here
} //namespace qmmf ends here
#endif /* QMMF_CAMERA_CONFIGURATION_H_ */
