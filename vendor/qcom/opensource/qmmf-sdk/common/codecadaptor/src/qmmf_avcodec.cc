/* Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#define TAG "AVCodec"

#include "common/codecadaptor/src/qmmf_avcodec.h"

#include <fcntl.h>

#include <cstdlib>
#include <iomanip>
#include <memory>
#include <string>
#include <sstream>

#include <cutils/properties.h>
#include <utils/String8.h>
#include <OMX_QCOMExtns.h>
#include <utils/RefBase.h>
#include <linux/msm_ion.h>
#include <OMX_VideoExt.h>
#include <OMX_IndexExt.h>
#include <media/hardware/HardwareAPI.h>
#include <gralloc_priv.h>
#include <math.h>

#include "common/codecadaptor/src/qmmf_avcodec_common.h"
#include "common/codecadaptor/src/qmmf_omx_client.h"
#include "common/qmmf_log.h"

#define OMX_SPEC_VERSION 0x00000101

#define Log2(number, power)                   \
  { OMX_U32 temp = number; power = 0;         \
  while( (0 == (temp & 0x1)) &&  power < 16)  \
  { temp >>=0x1; power++; } }

#define FractionToQ16(q,num,den)     \
  { OMX_U32 power; Log2(den,power);  \
  q = num << (16 - power); }

#define OMX_STATE_NAME(state)                            \
  (state == OMX_StateInvalid ? "OMX_StateInvalid" :      \
  (state == OMX_StateLoaded ? "OMX_StateLoaded" :        \
  (state == OMX_StateIdle ? "OMX_StateIdle" :            \
  (state == OMX_StateExecuting ? "OMX_StateExecuting" :  \
  (state == OMX_StatePause ? "OMX_StatePause" :          \
  "Unknown")))))

namespace qmmf {
namespace avcodec {

using ::std::setbase;
using ::std::shared_ptr;
using ::std::string;
using ::std::stringstream;
using ::std::underlying_type;
using ::std::vector;

template<class T>
static void InitOMXParams(T *params) {
  memset(params, 0x0, sizeof(T));
  params->nSize = sizeof(T);
  params->nVersion.nVersion = OMX_SPEC_VERSION;
}

struct __attribute__((packed)) AudioEncoderMetadata {
  uint32_t offset_to_frame;
  uint32_t frame_size;
  uint32_t encoded_pcm_samples;
  uint32_t lsw_ts;
  uint32_t msw_ts;
  uint32_t nflags;

  string ToString() const {
    stringstream stream;
    stream << "offset_to_frame[" << offset_to_frame << "] ";
    stream << "frame_size[" << frame_size << "] ";
    stream << "encoded_pcm_samples[" << encoded_pcm_samples << "] ";
    stream << "lsw_ts[" << lsw_ts << "] ";
    stream << "msw_ts[" << msw_ts << "] ";
    stream << "nflags[" << ::std::setbase(16) << nflags << ::std::setbase(10)
           << "]";
    return stream.str();
  }
};

// static
OMX_CALLBACKTYPE AVCodec::callbacks_ = {
    &OnEvent, &OnEmptyBufferDone, &OnFillBufferDone};

AVCodec::AVCodec()
    : state_(OMX_StateLoaded),
      state_pending_(OMX_StateLoaded),
      input_stop_(false),
      output_stop_(false),
      port_status_(true),
      signal_queue_(CMD_BUF_MAX_COUNT),
      bPortReconfig_(false) {

  QMMF_INFO("%s:%s Enter", TAG, __func__);

  omx_client_ = new OmxClient();
  if (nullptr == omx_client_.get())
      QMMF_ERROR("%s:%s OMX-IL createtion failed", TAG, __func__);
  else
      QMMF_INFO("%s:%s created OMX-IL instance(%p)", TAG, __func__,
          omx_client_.get());

  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

AVCodec::~AVCodec() {

  QMMF_INFO("%s:%s Enter", TAG, __func__);

  if (omx_client_.get()) {
      DeleteHandle();
  }

  state_ = OMX_StateInvalid;
  state_pending_ = OMX_StateInvalid;
  input_source_ = nullptr;
  output_source_ = nullptr;
  if (in_buff_hdr_) {
    delete []in_buff_hdr_;
    in_buff_hdr_ = nullptr;
  }
  if (out_buff_hdr_) {
    delete []out_buff_hdr_;
    out_buff_hdr_ = nullptr;
  }

  signal_queue_.Clear();

  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

status_t AVCodec::CreateHandle(char* component_name) {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;

  ret = omx_client_->CreateOmxHandle(component_name, this, callbacks_);
  if (ret != OK) {
    QMMF_ERROR("%s:%s failed to create OMX component(%s)", TAG, __func__,
        component_name);
    return ret;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t AVCodec::DeleteHandle() {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;

  if ((state_ != OMX_StateLoaded)) {
      QMMF_INFO("%s:%s Move state to Loaded state", TAG, __func__);
      ret = SetState(OMX_StateLoaded, OMX_TRUE);
      assert(ret == OK);
  }

  ret = omx_client_->ReleaseOmxHandle();
  assert(ret == OK);

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t AVCodec::GetComponentName(CodecMimeType mime_type,
                                   uint32_t *num_comps,
                                   vector<string>& comp_names) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = android::NO_ERROR;

  string role;
  switch (mime_type) {
    case CodecMimeType::kMimeTypeVideoEncAVC:
      role.append("video_encoder.avc");
      break;
    case CodecMimeType::kMimeTypeVideoEncHEVC:
      role.append("video_encoder.hevc");
      break;
    case CodecMimeType::kMimeTypeVideoEncMPEG:
      role.append("video_encoder.mpeg4");
      break;
    case CodecMimeType::kMimeTypeVideoDecAVC:
      role.append("video_decoder.avc");
      break;
    case CodecMimeType::kMimeTypeVideoDecHEVC:
      role.append("video_decoder.hevc ");
      break;
    case CodecMimeType::kMimeTypeVideoDecMPEG:
      role.append("video_decoder.mpeg4 ");
      break;
    case CodecMimeType::kMimeTypeAudioEncAAC:
      role.append("audio_encoder.aac");
      break;
    case CodecMimeType::kMimeTypeAudioEncAMR:
      role.append("audio_encoder.amrnb");
      break;
    case CodecMimeType::kMimeTypeAudioEncG711:
      role.append("audio_encoder.g711");
      break;
    case CodecMimeType::kMimeTypeAudioDecAAC:
      role.append("audio_decoder.aac");
      break;
    case CodecMimeType::kMimeTypeAudioDecAMR:
      role.append("audio_decoder.amr");
      break;
    case CodecMimeType::kMimeTypeAudioDecG711:
      role.append("audio_decoder.g711");
      break;
    default:
      QMMF_ERROR("%s:%s mime type not supported", TAG, __func__);
      return android::BAD_VALUE;
  }

  ret = omx_client_->GetComponentsOfRole(const_cast<char*>(role.c_str()),
                                         num_comps, 0);
  if (*num_comps) {
   QMMF_DEBUG("%s:%s total number of components = %d\n", TAG, __func__,
              *num_comps);
   /* Allocate memory for pointers to component name */
   OMX_U8** comp_name = new OMX_U8*[*num_comps];
   if (comp_name == nullptr)
     return android::NO_MEMORY;

   memset(comp_name, 0, (sizeof(OMX_U8))*(*num_comps));
   for (uint32_t i = 0; i < *num_comps; ++i)
   {
      comp_name[i] = new OMX_U8[OMX_MAX_STRINGNAME_SIZE];
      if (comp_name[i] == nullptr)
      {
         delete []comp_name;
         comp_name = nullptr;
         return android::NO_MEMORY;
      }
      memset(comp_name[i], 0, sizeof(OMX_U8)*OMX_MAX_STRINGNAME_SIZE);
   }

   ret = omx_client_->GetComponentsOfRole(const_cast<char*>(role.c_str()),
                                          num_comps, comp_name);
   for (uint32_t i = 0; i < *num_comps; ++i) {
     string comp((char*)comp_name[i]);
     comp_names.push_back(comp);
   }

   for (uint32_t i = 0; i< *num_comps; ++i) {
     delete[] comp_name[i];
   }
   delete[] comp_name;
   comp_name = nullptr;
  }

  return ret;
}

status_t AVCodec::ConfigureCodec(CodecMimeType codec_type,
                                 CodecParam& codec_param, string comp_name) {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  String8 component_name;
  status_t ret = 0;

  switch (codec_type) {
    case CodecMimeType::kMimeTypeVideoEncAVC:
    case CodecMimeType::kMimeTypeVideoEncHEVC:
    case CodecMimeType::kMimeTypeVideoEncMPEG:
      format_type_ = CodecType::kVideoEncoder;
      break;
    case CodecMimeType::kMimeTypeVideoDecAVC:
    case CodecMimeType::kMimeTypeVideoDecHEVC:
    case CodecMimeType::kMimeTypeVideoDecMPEG:
      format_type_ = CodecType::kVideoDecoder;
      break;
    case CodecMimeType::kMimeTypeAudioEncAAC:
    case CodecMimeType::kMimeTypeAudioEncAMR:
    case CodecMimeType::kMimeTypeAudioEncG711:
      format_type_ = CodecType::kAudioEncoder;
      break;
    case CodecMimeType::kMimeTypeAudioDecAAC:
    case CodecMimeType::kMimeTypeAudioDecAMR:
    case CodecMimeType::kMimeTypeAudioDecG711:
      format_type_ = CodecType::kAudioDecoder;
      break;
    default:
      QMMF_ERROR("%s:%s invalid codec_type given", TAG, __func__);
      return -1;
      break;
  }

  switch (format_type_) {
    case CodecType::kVideoEncoder:
      switch(codec_param.video_enc_param.format_type) {
        case VideoFormat::kAVC:
          component_name.appendFormat("OMX.qcom.video.encoder.avc");
          break;
        case VideoFormat::kHEVC:
          component_name.appendFormat("OMX.qcom.video.encoder.hevc");
          break;
        //TODO: JPEG/YUV/Bayer
        default:
          QMMF_ERROR("%s:%s Unknown Video Codec", TAG, __func__);
          return -1;
        }
      break;
    case CodecType::kVideoDecoder:
      switch(codec_param.video_dec_param.codec) {
        case ::qmmf::player::VideoCodecType::kHEVC:
          component_name.appendFormat("OMX.qcom.video.decoder.hevc");
          break;
        case ::qmmf::player::VideoCodecType::kAVC:
          component_name.appendFormat("OMX.qcom.video.decoder.avc");
          break;
        default:
          QMMF_ERROR("%s:%s Unknown Video Codec", TAG, __func__);
          return -1;
      }
      break;
    case CodecType::kAudioEncoder:
      switch(codec_param.audio_enc_param.format) {
        case AudioFormat::kAAC:
          component_name.appendFormat("OMX.qcom.audio.encoder.aac");
          break;
        case AudioFormat::kAMR:
          if (codec_param.audio_enc_param.codec_params.amr.isWAMR)
            component_name.appendFormat("OMX.qcom.audio.encoder.amrwb");
          else
            component_name.appendFormat("OMX.qcom.audio.encoder.amrnb");
          break;
        case AudioFormat::kG711:
          switch(codec_param.audio_enc_param.codec_params.g711.mode) {
            case G711Mode::kALaw:
              component_name.appendFormat("OMX.qcom.audio.encoder.g711alaw");
              break;
            case G711Mode::kMuLaw:
              component_name.appendFormat("OMX.qcom.audio.encoder.g711mlaw");
              break;
            default:
              QMMF_ERROR("%s:%s Unknown Audio Codec", TAG, __func__);
              return -1;
          }
          break;
        default:
          QMMF_ERROR("%s:%s Unknown Audio Codec", TAG, __func__);
          return -1;
        }
      break;
    case CodecType::kAudioDecoder:
      switch(codec_param.audio_dec_param.codec) {
        case ::qmmf::player::AudioCodecType::kAAC:
          component_name.appendFormat("OMX.qcom.audio.decoder.multiaac");
          break;
        case ::qmmf::player::AudioCodecType::kAMR:
          if (codec_param.audio_dec_param.codec_params.amr.isWAMR)
            component_name.appendFormat("OMX.qcom.audio.decoder.amrwb");
          else
            component_name.appendFormat("OMX.qcom.audio.decoder.amrnb");
          break;
        case ::qmmf::player::AudioCodecType::kG711:
          switch(codec_param.audio_dec_param.codec_params.g711.mode) {
            case G711Mode::kALaw:
              component_name.appendFormat("OMX.qcom.audio.decoder.g711alaw");
              break;
            case G711Mode::kMuLaw:
              component_name.appendFormat("OMX.qcom.audio.decoder.g711mlaw");
              break;
            default:
              QMMF_ERROR("%s:%s Unknown Audio Codec", TAG, __func__);
              return -1;
          }
          break;
        default:
          QMMF_ERROR("%s:%s Unknown Audio Codec", TAG, __func__);
          return -1;
        }
      break;
    case CodecType::kImageEncoder:
      break;
    case CodecType::kImageDecoder:
      break;
    default:
      QMMF_ERROR("%s:%s Unimplemented requested Codec", TAG, __func__);
      return -1;
  }

  ret = CreateHandle(const_cast<char *>(component_name.string()));
  if (ret != OK) {
      QMMF_ERROR("%s:%s failed to create omx handle", TAG, __func__);
      return ret;
  } else {
    QMMF_INFO("%s:%s Component(%s) created", TAG, __func__,
      const_cast<char *>(component_name.string()));
  }

  if (format_type_ == CodecType::kVideoEncoder) {
    ret = ConfigureVideoEncoder(codec_param);
  } else if (format_type_ == CodecType::kVideoDecoder) {
    ret = ConfigureVideoDecoder(codec_param);
  } else if (format_type_ == CodecType::kAudioEncoder) {
    ret = ConfigureAudioEncoder(codec_param);
  } else if (format_type_ == CodecType::kAudioDecoder) {
    ret = ConfigureAudioDecoder(codec_param);
  } else {
    QMMF_ERROR("%s:%s codec type not implemented", TAG, __func__);
    ret = -1;
  }

  // set component to Idle state
  ret = SetState(OMX_StateIdle, OMX_FALSE);

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t AVCodec::RegisterOutputBuffers(vector<BufferDescriptor>& list) {

  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;

  output_buffer_list_ = list;
  outputpParam_enc_.clear();
  outputpParam_dec_.clear();

  if (format_type_ == CodecType::kVideoEncoder) {
    for (auto& iter : list) {
      OMX_QCOM_PLATFORM_PRIVATE_PMEM_INFO pParam;
      pParam.pmem_fd = iter.fd;
      pParam.offset = 0;
      outputpParam_enc_.push_back(pParam);
    }
  }

  if (format_type_ == CodecType::kVideoDecoder) {
    for (auto& iter : list) {
      struct VideoDecoderOutputMetaData pParam;
      pParam.pHandle = (buffer_handle_t)malloc(sizeof(struct private_handle_t));
      private_handle_t* temp =  const_cast<private_handle_t*>
                                    (static_cast<const private_handle_t*>
                                    (pParam.pHandle));
      temp->fd = iter.fd;
      // temp->size should contain the size of the memory being allocated using
      // ion driver
      temp->size = iter.capacity;
      temp->flags = 0x0;
      outputpParam_dec_.push_back(pParam);
    }
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t AVCodec::RegisterInputBuffers(vector<BufferDescriptor>& list) {

  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;

  input_buffer_list_ = list;

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t AVCodec::ConfigureVideoEncoder(CodecParam& codec_param) {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;

  bool enable_init_qp = false;
  bool enable_qp_range = false;
  bool enable_qp_IBP_range = false;
  uint32_t width = codec_param.video_enc_param.width;
  uint32_t height = codec_param.video_enc_param.height;
  float    frame_rate = codec_param.video_enc_param.frame_rate;
  uint32_t init_IQP, init_PQP, init_BQP;
  uint32_t min_QP, max_QP;
  uint32_t min_IQP, max_IQP,  min_PQP, max_PQP, min_BQP, max_BQP;
  uint32_t ltr_count, hier_num_layer;
  VideoRateControlType rate_control;

  PrependSPSPPSToIDRFramesParams param;
  memset(&param, 0, sizeof(PrependSPSPPSToIDRFramesParams));
  param.nSize = sizeof(PrependSPSPPSToIDRFramesParams);
  param.bEnable = OMX_FALSE;
  switch (codec_param.video_enc_param.format_type) {
    case VideoFormat::kAVC:
      if (codec_param.video_enc_param.codec_param.avc.prepend_sps_pps_to_idr) {
        param.bEnable = OMX_TRUE;
      }

      OMX_QCOM_VIDEO_CONFIG_AUD param_aud;
      memset(&param_aud, 0, sizeof(param_aud));
      param_aud.nSize = sizeof(OMX_QCOM_VIDEO_CONFIG_AUD);
      param_aud.bEnable = OMX_FALSE;
      if (codec_param.video_enc_param.codec_param.avc.insert_aud_delimiter) {
        param_aud.bEnable = OMX_TRUE;
      }
      ret = omx_client_->SetParameter(
          static_cast<OMX_INDEXTYPE>(OMX_QcomIndexParamAUDelimiter),
          reinterpret_cast<OMX_PTR>(&param_aud));
      if (ret != OMX_ErrorNone) {
          QMMF_ERROR("%s:%s Failed to configure AUD delimiter",
                     TAG, __func__);
          return ret;
      }

      if (codec_param.video_enc_param.codec_param.avc.sar_enabled) {
        ret = ConfigureSAR(
            codec_param.video_enc_param.codec_param.avc.sar_width,
            codec_param.video_enc_param.codec_param.avc.sar_height);
        if (ret != OMX_ErrorNone) {
            QMMF_ERROR("%s:%s Failed to configure SAR",
                       TAG, __func__);
            return ret;
        }
      }
      break;

    case VideoFormat::kHEVC:
      if (codec_param.video_enc_param.codec_param.hevc.prepend_sps_pps_to_idr) {
        param.bEnable = OMX_TRUE;
      }

      if (codec_param.video_enc_param.codec_param.hevc.sar_enabled) {
        ret = ConfigureSAR(
            codec_param.video_enc_param.codec_param.hevc.sar_width,
            codec_param.video_enc_param.codec_param.hevc.sar_height);
        if (ret != OMX_ErrorNone) {
            QMMF_ERROR("%s:%s Failed to configure SAR",
                       TAG, __func__);
            return ret;
        }
      }
      break;

    default:
      QMMF_ERROR("%s:%s Codec Type does not support", TAG, __func__);
      return BAD_VALUE;
  }
  ret = omx_client_->SetParameter(
      static_cast<OMX_INDEXTYPE>(OMX_QcomIndexParamSequenceHeaderWithIDR),
      reinterpret_cast<OMX_PTR>(&param));
  if (ret != 0) {
      QMMF_ERROR("%s:%s Failed to configure in band sps/pps", TAG, __func__);
      return ret;
  }

  ret = SetPortParams(kPortIndexInput, width, height, frame_rate);
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to set port definiton on %s", TAG, __func__,
        PORT_NAME(kPortIndexInput));
    return ret;
  }

  ret = SetPortParams(kPortIndexOutput, width, height, frame_rate);
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to set port definiton on %s", TAG, __func__,
        PORT_NAME(kPortIndexOutput));
    return ret;
  }

  char prop[PROPERTY_VALUE_MAX];
  property_get("media.msm8953.version", prop, "0");
  if (atoi(prop) == 1) {
    QMMF_INFO("%s:%s Setting the Low Power Encode mode", TAG, __func__);
    QOMX_EXTNINDEX_VIDEO_PERFMODE perf_param;
    InitOMXParams(&perf_param);
    perf_param.nPerfMode = 2;
    ret = omx_client_->SetConfig(
        static_cast<OMX_INDEXTYPE>(OMX_QcomIndexConfigVideoVencPerfMode),
        reinterpret_cast<OMX_PTR>(&perf_param));
    if (ret != 0) {
      QMMF_ERROR("%s:%s Failed to set Low Power Mode", TAG, __func__);
      return ret;
    }
  }

  switch (codec_param.video_enc_param.format_type) {
    case VideoFormat::kAVC:
      enable_init_qp =
          codec_param.video_enc_param.codec_param.avc.qp_params.enable_init_qp;
      enable_qp_range =
          codec_param.video_enc_param.codec_param.avc.qp_params.enable_qp_range;
      enable_qp_IBP_range =
          codec_param.video_enc_param.codec_param.avc.qp_params
          .enable_qp_IBP_range;
      init_IQP =
          codec_param.video_enc_param.codec_param.avc.qp_params.init_qp
          .init_IQP;
      init_PQP =
          codec_param.video_enc_param.codec_param.avc.qp_params.init_qp
          .init_PQP;
      init_BQP =
          codec_param.video_enc_param.codec_param.avc.qp_params.init_qp
          .init_BQP;
      min_QP =
          codec_param.video_enc_param.codec_param.avc.qp_params.qp_range.min_QP;
      max_QP =
          codec_param.video_enc_param.codec_param.avc.qp_params.qp_range.max_QP;
      min_IQP =
          codec_param.video_enc_param.codec_param.avc.qp_params.qp_IBP_range
          .min_IQP;
      max_IQP =
          codec_param.video_enc_param.codec_param.avc.qp_params.qp_IBP_range
          .max_IQP;
      min_PQP =
          codec_param.video_enc_param.codec_param.avc.qp_params.qp_IBP_range
          .min_PQP;
      max_PQP =
          codec_param.video_enc_param.codec_param.avc.qp_params
          .qp_IBP_range.max_PQP;
      min_BQP = codec_param.video_enc_param.codec_param.avc.qp_params
          .qp_IBP_range.min_BQP;
      max_BQP = codec_param.video_enc_param.codec_param.avc.qp_params
          .qp_IBP_range.max_BQP;
      ltr_count = codec_param.video_enc_param.codec_param.avc.ltr_count;
      hier_num_layer = codec_param.video_enc_param.codec_param.avc.hier_layer;
      rate_control =
          codec_param.video_enc_param.codec_param.avc.ratecontrol_type;

      ret = SetupAVCEncoderParameters(codec_param);
      break;

    case VideoFormat::kHEVC:
      enable_init_qp =
          codec_param.video_enc_param.codec_param.hevc.qp_params.enable_init_qp;
      enable_qp_range =
          codec_param.video_enc_param.codec_param.hevc.qp_params
          .enable_qp_range;
      enable_qp_IBP_range =
          codec_param.video_enc_param.codec_param.hevc.qp_params
          .enable_qp_IBP_range;
      init_IQP =
          codec_param.video_enc_param.codec_param.hevc.qp_params.init_qp
          .init_IQP;
      init_PQP =
          codec_param.video_enc_param.codec_param.hevc.qp_params.init_qp
          .init_PQP;
      init_BQP =
          codec_param.video_enc_param.codec_param.hevc.qp_params.init_qp
          .init_BQP;
      min_QP =
          codec_param.video_enc_param.codec_param.hevc.qp_params.qp_range
          .min_QP;
      max_QP =
          codec_param.video_enc_param.codec_param.hevc.qp_params.qp_range
          .max_QP;
      min_IQP =
          codec_param.video_enc_param.codec_param.hevc.qp_params
          .qp_IBP_range.min_IQP;
      max_IQP =
          codec_param.video_enc_param.codec_param.hevc.qp_params
          .qp_IBP_range.max_IQP;
      min_PQP =
          codec_param.video_enc_param.codec_param.hevc.qp_params
          .qp_IBP_range.min_PQP;
      max_PQP =
          codec_param.video_enc_param.codec_param.hevc.qp_params
          .qp_IBP_range.max_PQP;
      min_BQP =
          codec_param.video_enc_param.codec_param.hevc.qp_params
          .qp_IBP_range.min_BQP;
      max_BQP =
          codec_param.video_enc_param.codec_param.hevc.qp_params
          .qp_IBP_range.max_BQP;
      ltr_count = codec_param.video_enc_param.codec_param.hevc.ltr_count;
      hier_num_layer = codec_param.video_enc_param.codec_param.hevc.hier_layer;
      rate_control =
          codec_param.video_enc_param.codec_param.hevc.ratecontrol_type;

      ret = SetupHEVCEncoderParameters(codec_param);
      break;
    default:
      QMMF_ERROR("%s:%s Codec Type does not support", TAG, __func__);
      return -1;
  }

  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to set up codec parameter", TAG, __func__);
    return ret;
  }

  //SetUp QP parameter.
  if (enable_init_qp) {
    if (rate_control != VideoRateControlType::kDisable) {
      // RC ON
      QOMX_EXTNINDEX_VIDEO_INITIALQP initqp;
      InitOMXParams(&initqp);
      initqp.nPortIndex = kPortIndexOutput;
      initqp.nQpI = init_IQP;
      initqp.nQpP = init_PQP;
      initqp.nQpB = init_BQP;
      initqp.bEnableInitQp = 0x7; // Intial QP applied to all frame
      ret = omx_client_->SetParameter(
          static_cast<OMX_INDEXTYPE>(QOMX_IndexParamVideoInitialQp),
          reinterpret_cast<OMX_PTR>(&initqp));
      if (ret != 0) {
        QMMF_ERROR("%s:%s Failed to set Initial QP parameter", TAG, __func__);
        return ret;
      }
    } else {
      // RC OFF
      OMX_VIDEO_PARAM_QUANTIZATIONTYPE initqp;
      InitOMXParams(&initqp);
      initqp.nPortIndex = kPortIndexOutput;
      initqp.nQpI = init_IQP;
      initqp.nQpP = init_PQP;
      initqp.nQpB = init_BQP;
      ret = omx_client_->SetParameter(
          static_cast<OMX_INDEXTYPE>(OMX_IndexParamVideoQuantization),
          reinterpret_cast<OMX_PTR>(&initqp));
      if (ret != 0) {
        QMMF_ERROR("%s:%s Failed to set Initial QP parameter", TAG, __func__);
        return ret;
      }
    }
  }

  if (enable_qp_range) {
    OMX_QCOM_VIDEO_PARAM_QPRANGETYPE qp_range;
    InitOMXParams(&qp_range);
    qp_range.nPortIndex = kPortIndexOutput;
    ret = omx_client_->GetParameter(
        static_cast<OMX_INDEXTYPE>(OMX_QcomIndexParamVideoQPRange),
        reinterpret_cast<OMX_PTR>(&qp_range));
    if (ret != 0) {
      QMMF_ERROR("%s:%s Failed to get QP Min/Max Range", TAG, __func__);
      return ret;
    }
    qp_range.minQP = min_QP;
    qp_range.maxQP = max_QP;

    ret = omx_client_->SetParameter(
        static_cast<OMX_INDEXTYPE>(OMX_QcomIndexParamVideoQPRange),
        reinterpret_cast<OMX_PTR>(&qp_range));
    if (ret != OK) {
      QMMF_ERROR("%s:%s Failed to set QP Min/Max Range", TAG, __func__);
      return ret;
    }
  }

  if (enable_qp_IBP_range) {
    OMX_QCOM_VIDEO_PARAM_IPB_QPRANGETYPE qp_range;
    InitOMXParams(&qp_range);
    qp_range.nPortIndex = kPortIndexOutput;

    ret = omx_client_->GetParameter(
        static_cast<OMX_INDEXTYPE>(OMX_QcomIndexParamVideoIPBQPRange),
        reinterpret_cast<OMX_PTR>(&qp_range));
    if (ret != 0) {
      QMMF_ERROR("%s:%s Failed to get IPBQP Range parameter", TAG, __func__);
      return ret;
    }
    qp_range.minIQP = min_IQP;
    qp_range.maxIQP = max_IQP;
    qp_range.minPQP = min_PQP;
    qp_range.maxPQP = max_PQP;
    qp_range.minBQP = min_BQP;
    qp_range.maxBQP = max_BQP;

    ret = omx_client_->SetParameter(
        static_cast<OMX_INDEXTYPE>(OMX_QcomIndexParamVideoIPBQPRange),
        reinterpret_cast<OMX_PTR>(&qp_range));
    if (ret != 0) {
      QMMF_ERROR("%s:%s Failed to set IPBQP Range parameter", TAG, __func__);
      return ret;
    }
  }

  if (ltr_count > 0) {
    QOMX_VIDEO_PARAM_LTRCOUNT_TYPE ltr_frame;
    InitOMXParams(&ltr_frame);
    ltr_frame.nPortIndex = kPortIndexOutput;

    ret = omx_client_->GetParameter(
        static_cast<OMX_INDEXTYPE>(QOMX_IndexParamVideoLTRCount),
        reinterpret_cast<OMX_PTR>(&ltr_frame));
    if (ret != 0) {
        QMMF_ERROR("%s:%s Failed to get ltr count parameter", TAG, __func__);
        return ret;
    }

    ltr_frame.nCount = ltr_count;
    ret = omx_client_->SetParameter(
        static_cast<OMX_INDEXTYPE>(QOMX_IndexParamVideoLTRCount),
        reinterpret_cast<OMX_PTR>(&ltr_frame));
    if (ret != 0) {
        QMMF_ERROR("%s:%s Failed to set ltr count parameter", TAG, __func__);
        return ret;
    }
  }

  if (hier_num_layer > 0) {
    QOMX_VIDEO_HIERARCHICALLAYERS hier_layer;
    InitOMXParams(&hier_layer);
    hier_layer.nPortIndex = kPortIndexOutput;

    ret = omx_client_->GetParameter(
        static_cast<OMX_INDEXTYPE>(OMX_QcomIndexHierarchicalStructure),
        reinterpret_cast<OMX_PTR>(&hier_layer));
    if (ret != 0) {
        QMMF_ERROR("%s:%s Failed to get hierarchial parameter", TAG, __func__);
        return ret;
    }

    hier_layer.eHierarchicalCodingType = QOMX_HIERARCHICALCODING_P;
    hier_layer.nNumLayers = hier_num_layer;

    ret = omx_client_->SetParameter(
        static_cast<OMX_INDEXTYPE>(OMX_QcomIndexHierarchicalStructure),
        reinterpret_cast<OMX_PTR>(&hier_layer));
    if (ret != 0) {
        QMMF_ERROR("%s:%s Failed to set hierarchial parameter", TAG, __func__);
        return ret;
    }
  }

  ret = ConfigureBitrate(codec_param);
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to configure bitrate", TAG, __func__);
    return ret;
  }

  QMMF_INFO("%s:%s setupVideoEncoder succeeded", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t AVCodec::ConfigureVideoDecoder(CodecParam& codec_param) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;

  OMX_QCOM_PARAM_PORTDEFINITIONTYPE inputPortFmt;
  InitOMXParams(&inputPortFmt);
  inputPortFmt.nPortIndex = kPortIndexInput;
  inputPortFmt.nFramePackingFormat = OMX_QCOM_FramePacking_OnlyOneCompleteFrame;
  ret = omx_client_->SetParameter(
      static_cast<OMX_INDEXTYPE>(OMX_QcomIndexPortDefn),
      reinterpret_cast<OMX_PTR>(&inputPortFmt));
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to Set Frame Packing Format", TAG, __func__);
    return ret;
  }

  OMX_QTI_VIDEO_PARAM_FORCE_UNCOMPRESSED_FOR_OPB_TYPE pParam_forceyuv;
  InitOMXParams(&pParam_forceyuv);
  pParam_forceyuv.bEnable = OMX_TRUE;
  ret = omx_client_->SetParameter(
      static_cast<OMX_INDEXTYPE>(OMX_QTIIndexParamForceUnCompressedForOPB),
      reinterpret_cast<OMX_PTR>(&pParam_forceyuv));
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to force YUV(Uncompressed) OPB Data on %s",
               TAG, __func__, PORT_NAME(kPortIndexOutput));
    return ret;
  }

  OMX_VIDEO_PARAM_PORTFORMATTYPE videoPortFmt;
  InitOMXParams(&videoPortFmt);
  videoPortFmt.nPortIndex = kPortIndexOutput;
  videoPortFmt.eColorFormat =
      static_cast<OMX_COLOR_FORMATTYPE>
                 (QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m);
  ret = omx_client_->SetParameter(OMX_IndexParamVideoPortFormat,
                                  reinterpret_cast<OMX_PTR>(&videoPortFmt));
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to Set ColorFormat", TAG, __func__);
    return ret;
  }

  if (codec_param.video_dec_param.enable_downscalar &&
      codec_param.video_dec_param.enable_vqzip_extradata) {
    QMMF_ERROR("%s:%s Venus Downscalar and SEI extradata can not be enabled together",
               TAG, __func__);
    return -1;
  }

  if (codec_param.video_dec_param.enable_downscalar) {
    QMMF_INFO("%s:%s Enabling downcalar", TAG, __func__);
    QOMX_INDEXDOWNSCALAR downscalar_params;
    InitOMXParams(&downscalar_params);
    downscalar_params.bEnable = OMX_TRUE;
    downscalar_params.nPortIndex = kPortIndexOutput;
    ret = omx_client_->SetParameter(
        static_cast<OMX_INDEXTYPE>(OMX_QcomIndexParamVideoDownScalar),
        reinterpret_cast<OMX_PTR>(&downscalar_params));
    if (ret != 0) {
      QMMF_ERROR("%s:%s Failed to Enable Downscalar", TAG, __func__);
      return ret;
    }
  }

  if (codec_param.video_dec_param.enable_vqzip_extradata) {
    QMMF_INFO("%s:%s Enabling SEI extradata for VQZIP", TAG, __func__);
    OMX_QTI_VIDEO_PARAM_VQZIP_SEI_TYPE enable_sei_params;
    InitOMXParams(&enable_sei_params);
    enable_sei_params.bEnable = OMX_TRUE;
    ret = omx_client_->SetParameter(
        static_cast<OMX_INDEXTYPE>(OMX_QTIIndexParamVQZIPSEIType),
        reinterpret_cast<OMX_PTR>(&enable_sei_params));
    if (ret != 0) {
      QMMF_ERROR("%s:%s Failed to set the SEI extradata for Decoder", TAG, __func__);
      return ret;
    }
  }

  OMX_PARAM_PORTDEFINITIONTYPE input_port;
  InitOMXParams(&input_port);
  input_port.nPortIndex = kPortIndexInput;
  input_port.eDir = OMX_DirInput;
  ret = omx_client_->GetParameter(OMX_IndexParamPortDefinition,
                                  reinterpret_cast<OMX_PTR>(&input_port));
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to Get Port param definiton on %s",
               TAG, __func__, PORT_NAME(kPortIndexInput));
    return ret;
  }

  QMMF_INFO("%s: %s input_port.nBufferCountMin[%u]", TAG, __func__,
            input_port.nBufferCountMin);
  QMMF_INFO("%s: %s input_port.nBufferSize[%u]", TAG, __func__,
            input_port.nBufferSize);

  if (input_port.eDir != OMX_DirInput) {
    QMMF_ERROR("%s:%s Error: Expected Input port", TAG, __func__);
    return OMX_ErrorUndefined;
  }

  input_port.format.video.nFrameHeight = codec_param.video_dec_param.height;
  input_port.format.video.nFrameWidth  = codec_param.video_dec_param.width;
  FractionToQ16(input_port.format.video.xFramerate,
                (int)(codec_param.video_dec_param.frame_rate* 2), 2);
  ret = omx_client_->SetParameter(
      static_cast<OMX_INDEXTYPE>(OMX_IndexParamPortDefinition),
      reinterpret_cast<OMX_PTR>(&input_port));
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to set port definiton(H,W,FPS) on %s",
               TAG, __func__, PORT_NAME(kPortIndexInput));
    return ret;
  }

  ret = omx_client_->GetParameter(
      static_cast<OMX_INDEXTYPE>(OMX_IndexParamPortDefinition),
      reinterpret_cast<OMX_PTR>(&input_port));
  if(ret != 0) {
    QMMF_ERROR("%s:%s Failed to Get Param port definiton on %s",
               TAG, __func__, PORT_NAME(kPortIndexInput));
    return ret;
  }

  QMMF_INFO("%s: %s input_port.nBufferCountActual[%u]", TAG, __func__,
            input_port.nBufferCountActual);
  QMMF_INFO("%s: %s input_port.nBufferSize[%u]", TAG, __func__,
            input_port.nBufferSize);

  in_buff_hdr_size_ = input_port.nBufferCountActual;

  OMX_PARAM_PORTDEFINITIONTYPE output_port;
  InitOMXParams(&output_port);
  output_port.nPortIndex = kPortIndexOutput;
  output_port.eDir = OMX_DirOutput;
  ret = omx_client_->GetParameter(OMX_IndexParamPortDefinition,
                                  reinterpret_cast<OMX_PTR>(&output_port));
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to Get Port param definiton on %s",
               TAG, __func__, PORT_NAME(kPortIndexOutput));
    return ret;
  }

  QMMF_INFO("%s: %s output_port.nBufferCountMin[%u]", TAG, __func__,
            output_port.nBufferCountMin);
  QMMF_INFO("%s: %s output_port.nBufferCountActual[%u]", TAG, __func__,
            output_port.nBufferCountActual);
  QMMF_INFO("%s: %s output_port.nBufferSize[%u]", TAG, __func__,
            output_port.nBufferSize);

  if (codec_param.video_dec_param.enable_downscalar) {
    QMMF_INFO("%s:%s Setting up Downscalar height and width", TAG, __func__);
    output_port.format.video.nFrameHeight =
        codec_param.video_dec_param.output_height;
    output_port.format.video.nFrameWidth  =
        codec_param.video_dec_param.output_width;
  } else {
    output_port.format.video.nFrameHeight = codec_param.video_dec_param.height;
    output_port.format.video.nFrameWidth  = codec_param.video_dec_param.width;
  }

  ret = omx_client_->SetParameter(OMX_IndexParamPortDefinition,
                                  reinterpret_cast<OMX_PTR>(&output_port));
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to set port definiton(H,W) on %s",
               TAG, __func__, PORT_NAME(kPortIndexOutput));
    return ret;
  }

  ret = omx_client_->GetParameter(OMX_IndexParamPortDefinition,
                                  reinterpret_cast<OMX_PTR>(&output_port));
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to Get Port param definiton on %s",
               TAG, __func__, PORT_NAME(kPortIndexOutput));
    return ret;
  }

  if (output_port.eDir != OMX_DirOutput) {
    QMMF_ERROR("%s:%s Error: Expected Output Port", TAG, __func__);
    return OMX_ErrorUndefined;
  }

  QMMF_INFO("%s: %s output_port.nBufferCountMin[%u]", TAG, __func__,
            output_port.nBufferCountMin);
  QMMF_INFO("%s: %s output_port.nBufferCountActual[%u]", TAG, __func__,
            output_port.nBufferCountActual);
  QMMF_INFO("%s: %s output_port.nBufferSize[%u]", TAG, __func__,
            output_port.nBufferSize);

  out_buff_hdr_size_ = output_port.nBufferCountActual;

  // set picture order
  QOMX_VIDEO_DECODER_PICTURE_ORDER picture_order;
  InitOMXParams(&picture_order);
  picture_order.eOutputPictureOrder =
      QOMX_VIDEO_PICTURE_ORDER::QOMX_VIDEO_DISPLAY_ORDER;
  picture_order.nPortIndex = kPortIndexOutput;
  ret = omx_client_->SetParameter(
      static_cast<OMX_INDEXTYPE>(OMX_QcomIndexParamVideoDecoderPictureOrder),
      reinterpret_cast<OMX_PTR>(&picture_order));
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to Set picture order!", TAG, __func__);
    return ret;
  }

  QMMF_INFO("%s:%s Decoder: Video format: W x H (%u x %u)", TAG, __func__,
            (uint32_t)output_port.format.video.nFrameWidth,
            (uint32_t)output_port.format.video.nFrameHeight);

  codec_params_ = codec_param;

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

OMX_ERRORTYPE AVCodec::prepareForAdaptivePlayback(
        OMX_U32 portIndex, OMX_BOOL enable, OMX_U32 maxFrameWidth,
        OMX_U32 maxFrameHeight) {

  QMMF_INFO("%s:%s Enter", TAG, __func__);

  QMMF_INFO("%s:%s %s:%u isEnable=%d max(WxH)=%ux%u", TAG, __func__,
      PORT_NAME(portIndex), portIndex, enable, maxFrameWidth,
      maxFrameHeight);

  OMX_INDEXTYPE index;
  OMX_STRING name = const_cast<OMX_STRING>(
                        "OMX.google.android.index.prepareForAdaptivePlayback");

  auto err = omx_client_->GetExtensionIndex(name, &index);
  if (err != OMX_ErrorNone) {
    QMMF_ERROR("%s:%s Failed to get extension index", TAG, __func__);
    return err;
  }
  PrepareForAdaptivePlaybackParams params;
  InitOMXParams(&params);
  params.nPortIndex = portIndex;
  params.bEnable = enable;
  params.nMaxFrameWidth = maxFrameWidth;
  params.nMaxFrameHeight = maxFrameHeight;

  err = omx_client_->SetParameter(index, &params);
  if(err != OMX_ErrorNone) {
    QMMF_ERROR("%s:%s Failed to Enable Adaptive PlayBack", TAG, __func__);
    return err;
  }

  QMMF_INFO("%s:%s %s(%#x): %s:%u en=%d max=%ux%u",TAG, __func__ , name, index,
      PORT_NAME(portIndex), portIndex, enable, maxFrameWidth, maxFrameHeight);

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return err;
}

status_t AVCodec::ConfigureAudioEncoder(CodecParam& codec_param) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  OMX_ERRORTYPE result;

  // get the port information
  OMX_PORT_PARAM_TYPE audio_ports;
  InitOMXParams(&audio_ports);
  result = omx_client_->GetParameter(OMX_IndexParamAudioInit,
                                     static_cast<OMX_PTR>(&audio_ports));
  if (result != OMX_ErrorNone) {
    QMMF_ERROR("%s: %s() failed to get audio_port parameters: %d", TAG,
               __func__, result);
    return ::android::FAILED_TRANSACTION;
  }
  QMMF_VERBOSE("%s: %s() audio_ports.nPorts[%u]", TAG, __func__,
               audio_ports.nPorts);
  QMMF_VERBOSE("%s: %s() audio_ports.nStartPortNumber[%u]", TAG, __func__,
               audio_ports.nStartPortNumber);

  // query the encoder input buffer requirements
  OMX_PARAM_PORTDEFINITIONTYPE input_port;
  InitOMXParams(&input_port);
  input_port.nPortIndex = audio_ports.nStartPortNumber;
  result = omx_client_->GetParameter(OMX_IndexParamPortDefinition,
                                     static_cast<OMX_PTR>(&input_port));
  if (result != OMX_ErrorNone) {
    QMMF_ERROR("%s: %s() failed to get input_port parameters: %d", TAG,
               __func__, result);
    return ::android::FAILED_TRANSACTION;
  }
  if (input_port.eDir != OMX_DirInput) {
    QMMF_ERROR("%s: %s() input_port is not configured for input", TAG,
               __func__);
    return ::android::BAD_VALUE;
  }
  QMMF_VERBOSE("%s: %s() input_port.nBufferCountMin[%u]", TAG, __func__,
               input_port.nBufferCountMin);
  QMMF_VERBOSE("%s: %s() input_port.nBufferSize[%u]", TAG, __func__,
               input_port.nBufferSize);

  // hardcode the number of input buffers
  uint32_t buf_count = INPUT_MAX_COUNT;
  if (input_port.nBufferCountActual != buf_count) {
    input_port.nBufferCountActual = buf_count;
    result = omx_client_->SetParameter(OMX_IndexParamPortDefinition,
                                       static_cast<OMX_PTR>(&input_port));
    if (result != OMX_ErrorNone) {
      QMMF_ERROR("%s: %s() failed to set new buffer count[%d] on %s", TAG,
                 __func__, input_port.nBufferCountActual,
                 PORT_NAME(input_port.nPortIndex));
      return result;
    }
    result = omx_client_->GetParameter(OMX_IndexParamPortDefinition,
                                       &input_port);
    if (result != OMX_ErrorNone) {
      QMMF_ERROR("%s: %s() failed to getParameter on %s", TAG, __func__,
                 PORT_NAME(input_port.nPortIndex));
      return result;
    }
    QMMF_DEBUG("%s: %s() new buffer specs: count[%d] size[%d]", TAG, __func__,
               input_port.nBufferCountActual, input_port.nBufferSize);
    if (buf_count != input_port.nBufferCountActual) {
      QMMF_ERROR("%s: %s() failed to confirm count on %s", TAG, __func__,
                 PORT_NAME(input_port.nPortIndex));
      return ::android::BAD_VALUE;
    }
  }
  in_buff_hdr_size_ = input_port.nBufferCountActual;

  // set the PCM input parameters
  OMX_AUDIO_PARAM_PCMMODETYPE pcm_params;
  InitOMXParams(&pcm_params);
  pcm_params.nPortIndex = kPortIndexInput;
  pcm_params.nChannels = codec_param.audio_enc_param.channels;
  pcm_params.nSamplingRate = codec_param.audio_enc_param.sample_rate;
  pcm_params.bInterleaved = OMX_TRUE;
  result = omx_client_->SetParameter(OMX_IndexParamAudioPcm,
                                     static_cast<OMX_PTR>(&pcm_params));
  if (result != OMX_ErrorNone) {
    QMMF_ERROR("%s: %s() failed to set PCM parameters: %d", TAG, __func__,
               result);
    return ::android::FAILED_TRANSACTION;
  }

  // query the encoder output buffer requirements
  OMX_PARAM_PORTDEFINITIONTYPE output_port;
  InitOMXParams(&output_port);
  output_port.nPortIndex = audio_ports.nStartPortNumber + 1;
  result = omx_client_->GetParameter(OMX_IndexParamPortDefinition,
                                     static_cast<OMX_PTR>(&output_port));
  if (result != OMX_ErrorNone) {
    QMMF_ERROR("%s: %s() failed to get output_port parameters: %d", TAG,
               __func__, result);
    return ::android::FAILED_TRANSACTION;
  }
  if (output_port.eDir != OMX_DirOutput) {
    QMMF_ERROR("%s: %s() output_port is not configured for output", TAG,
               __func__);
    return ::android::BAD_VALUE;
  }
  QMMF_VERBOSE("%s: %s() output_port.nBufferCountMin[%u]", TAG, __func__,
               output_port.nBufferCountMin);
  QMMF_VERBOSE("%s: %s() output_port.nBufferSize[%u]", TAG, __func__,
               output_port.nBufferSize);
  out_buff_hdr_size_ = output_port.nBufferCountActual;

  switch (codec_param.audio_enc_param.format) {
    case AudioFormat::kAAC: {
      // set the AAC output parameters
      OMX_AUDIO_PARAM_AACPROFILETYPE aac_params;
      InitOMXParams(&aac_params);
      aac_params.nPortIndex = kPortIndexOutput;
      aac_params.nChannels = codec_param.audio_enc_param.channels;
      aac_params.nSampleRate = codec_param.audio_enc_param.sample_rate;
      aac_params.nBitRate = codec_param.audio_enc_param.codec_params.aac.bit_rate;
      switch (codec_param.audio_enc_param.channels) {
        case 1:
          aac_params.eChannelMode = OMX_AUDIO_ChannelModeMono;
          break;
        case 2:
          aac_params.eChannelMode = OMX_AUDIO_ChannelModeStereo;
          break;
        default:
          QMMF_ERROR("%s: %s() unsupported number of channels: %d", TAG,
                     __func__, codec_param.audio_enc_param.channels);
          return ::android::BAD_VALUE;
      }
      switch (codec_param.audio_enc_param.codec_params.aac.format) {
        case AACFormat::kADTS:
          aac_params.eAACStreamFormat = OMX_AUDIO_AACStreamFormatMP4ADTS;
          break;
        case AACFormat::kRaw:
          aac_params.eAACStreamFormat = OMX_AUDIO_AACStreamFormatRAW;
          break;
       case AACFormat::kMP4FF:
          aac_params.eAACStreamFormat = OMX_AUDIO_AACStreamFormatMP4FF;
          break;
        default:
          QMMF_ERROR("%s: %s() unsupported AAC format: %d", TAG, __func__,
                     codec_param.audio_enc_param.codec_params.aac.format);
          return ::android::BAD_VALUE;
      }
      switch (codec_param.audio_enc_param.codec_params.aac.mode) {
        case AACMode::kAALC:
          aac_params.eAACProfile = OMX_AUDIO_AACObjectLC;
          break;
        case AACMode::kHEVC_v1:
          aac_params.eAACProfile = OMX_AUDIO_AACObjectHE;
          break;
        case AACMode::kHEVC_v2:
          aac_params.eAACProfile = OMX_AUDIO_AACObjectHE_PS;
          break;
        default:
          QMMF_ERROR("%s: %s() unsupported AAC mode: %d", TAG, __func__,
                     codec_param.audio_enc_param.codec_params.aac.mode);
          return ::android::BAD_VALUE;
      }
      result = omx_client_->SetParameter(OMX_IndexParamAudioAac,
                                         static_cast<OMX_PTR>(&aac_params));
      if (result != OMX_ErrorNone) {
        QMMF_ERROR("%s: %s() failed to set AAC parameters: %d", TAG, __func__,
                   result);
        return ::android::FAILED_TRANSACTION;
      }
      break;
    }
    case AudioFormat::kAMR:
      // set the AMR output parameters
      OMX_AUDIO_PARAM_AMRTYPE amr_params;
      InitOMXParams(&amr_params);
      amr_params.nPortIndex = kPortIndexOutput;
      amr_params.nChannels = codec_param.audio_enc_param.channels;
      if (codec_param.audio_enc_param.codec_params.amr.isWAMR)
        amr_params.eAMRBandMode = OMX_AUDIO_AMRBandModeWB8;
      else
        amr_params.eAMRBandMode = OMX_AUDIO_AMRBandModeNB7;
      result = omx_client_->SetParameter(OMX_IndexParamAudioAmr,
                                         static_cast<OMX_PTR>(&amr_params));
      if (result != OMX_ErrorNone) {
        QMMF_ERROR("%s: %s() failed to set AMR parameters: %d", TAG, __func__,
                   result);
        return ::android::FAILED_TRANSACTION;
      }
      break;
    case AudioFormat::kG711:
      // set the G711 output parameters
      OMX_AUDIO_PARAM_PCMMODETYPE pcm_params;
      InitOMXParams(&pcm_params);
      pcm_params.nPortIndex = kPortIndexInput;
      pcm_params.nChannels = codec_param.audio_enc_param.channels;
      pcm_params.nSamplingRate = codec_param.audio_enc_param.sample_rate;
      result = omx_client_->SetParameter(OMX_IndexParamAudioPcm,
                                         static_cast<OMX_PTR>(&pcm_params));
      if (result != OMX_ErrorNone) {
        QMMF_ERROR("%s: %s() failed to set G711 parameters: %d", TAG, __func__,
                   result);
        return ::android::FAILED_TRANSACTION;
      }
      break;
    default:
      QMMF_ERROR("%s: %s() unknown audio codec: %d", TAG, __func__,
                 static_cast<int>(codec_param.audio_enc_param.format));
      return ::android::BAD_VALUE;
  }

  return ::android::NO_ERROR;
}

static OMX_AUDIO_AMRBANDMODETYPE pickModeFromBitRate(bool isAMRWB, uint32_t bps) {
    if (isAMRWB) {
        if (bps <= 6600) {
            return OMX_AUDIO_AMRBandModeWB0;
        } else if (bps <= 8850) {
            return OMX_AUDIO_AMRBandModeWB1;
        } else if (bps <= 12650) {
            return OMX_AUDIO_AMRBandModeWB2;
        } else if (bps <= 14250) {
            return OMX_AUDIO_AMRBandModeWB3;
        } else if (bps <= 15850) {
            return OMX_AUDIO_AMRBandModeWB4;
        } else if (bps <= 18250) {
            return OMX_AUDIO_AMRBandModeWB5;
        } else if (bps <= 19850) {
            return OMX_AUDIO_AMRBandModeWB6;
        } else if (bps <= 23050) {
            return OMX_AUDIO_AMRBandModeWB7;
        }

        // 23850 bps
        return OMX_AUDIO_AMRBandModeWB8;
    } else {  // AMRNB
        if (bps <= 4750) {
            return OMX_AUDIO_AMRBandModeNB0;
        } else if (bps <= 5150) {
            return OMX_AUDIO_AMRBandModeNB1;
        } else if (bps <= 5900) {
            return OMX_AUDIO_AMRBandModeNB2;
        } else if (bps <= 6700) {
            return OMX_AUDIO_AMRBandModeNB3;
        } else if (bps <= 7400) {
            return OMX_AUDIO_AMRBandModeNB4;
        } else if (bps <= 7950) {
            return OMX_AUDIO_AMRBandModeNB5;
        } else if (bps <= 10200) {
            return OMX_AUDIO_AMRBandModeNB6;
        }

        // 12200 bps
        return OMX_AUDIO_AMRBandModeNB7;
    }
}

status_t AVCodec::ConfigureAudioDecoder(CodecParam& codec_param) {
  QMMF_INFO("%s:%s Enter",TAG,__func__);
  OMX_ERRORTYPE result;

  // get the port information
  OMX_PORT_PARAM_TYPE audio_ports;
  InitOMXParams(&audio_ports);
  result = omx_client_->GetParameter(OMX_IndexParamAudioInit,
                                     static_cast<OMX_PTR>(&audio_ports));
  if (result != OMX_ErrorNone) {
    QMMF_ERROR("%s: %s() failed to get audio_port parameters: %d", TAG,
               __func__, result);
    return ::android::FAILED_TRANSACTION;
  }
  QMMF_VERBOSE("%s: %s() audio_ports.nPorts[%u]", TAG, __func__,
               audio_ports.nPorts);
  QMMF_VERBOSE("%s: %s() audio_ports.nStartPortNumber[%u]", TAG, __func__,
               audio_ports.nStartPortNumber);

  // query the decoder input buffer requirements
  OMX_PARAM_PORTDEFINITIONTYPE input_port;
  InitOMXParams(&input_port);
  input_port.nPortIndex = audio_ports.nStartPortNumber;
  result = omx_client_->GetParameter(OMX_IndexParamPortDefinition,
                                     static_cast<OMX_PTR>(&input_port));
  if (result != OMX_ErrorNone) {
    QMMF_ERROR("%s: %s() failed to get input_port parameters: %d", TAG,
               __func__, result);
    return ::android::FAILED_TRANSACTION;
  }
  if (input_port.eDir != OMX_DirInput) {
    QMMF_ERROR("%s: %s() input_port is not configured for input", TAG,
               __func__);
    return ::android::BAD_VALUE;
  }
  QMMF_VERBOSE("%s: %s() input_port.nBufferCountMin[%u]", TAG, __func__,
               input_port.nBufferCountMin);
  QMMF_VERBOSE("%s: %s() input_port.nBufferSize[%u]", TAG, __func__,
               input_port.nBufferSize);
  in_buff_hdr_size_ = input_port.nBufferCountActual;

  // query the decoder output buffer requirements
  OMX_PARAM_PORTDEFINITIONTYPE output_port;
  InitOMXParams(&output_port);
  output_port.nPortIndex = audio_ports.nStartPortNumber + 1;
  result = omx_client_->GetParameter(OMX_IndexParamPortDefinition,
                                     static_cast<OMX_PTR>(&output_port));
  if (result != OMX_ErrorNone) {
    QMMF_ERROR("%s: %s() failed to get output_port parameters: %d", TAG,
               __func__, result);
    return ::android::FAILED_TRANSACTION;
  }
  if (output_port.eDir != OMX_DirOutput) {
    QMMF_ERROR("%s: %s() output_port is not configured for output", TAG,
               __func__);
    return ::android::BAD_VALUE;
  }
  QMMF_VERBOSE("%s: %s() output_port.nBufferCountMin[%u]", TAG, __func__,
               output_port.nBufferCountMin);
  QMMF_VERBOSE("%s: %s() output_port.nBufferSize[%u]", TAG, __func__,
               output_port.nBufferSize);
  out_buff_hdr_size_ = output_port.nBufferCountActual;


  //Confuguring Input Port Parameters
  switch (codec_param.audio_dec_param.codec) {
    case ::qmmf::player::AudioCodecType::kAAC: {
      // set the AAC Input parameters
      OMX_AUDIO_PARAM_AACPROFILETYPE aac_params;
      InitOMXParams(&aac_params);
      aac_params.nPortIndex = audio_ports.nStartPortNumber;
      aac_params.nChannels = codec_param.audio_dec_param.channels;
      aac_params.nSampleRate = codec_param.audio_dec_param.sample_rate;
      aac_params.nBitRate = codec_param.audio_dec_param.bitrate;
      switch (codec_param.audio_dec_param.channels) {
        case 1:
          aac_params.eChannelMode = OMX_AUDIO_ChannelModeMono;
          break;
        case 2:
          aac_params.eChannelMode = OMX_AUDIO_ChannelModeStereo;
          break;
        default:
          QMMF_ERROR("%s: %s() unsupported number of channels: %d", TAG,
                     __func__, codec_param.audio_dec_param.channels);
          return ::android::BAD_VALUE;
      }
      switch (codec_param.audio_dec_param.codec_params.aac.format) {
        case AACFormat::kADTS:
          aac_params.eAACStreamFormat = OMX_AUDIO_AACStreamFormatMP4ADTS;
          break;
        case AACFormat::kRaw:
          aac_params.eAACStreamFormat = OMX_AUDIO_AACStreamFormatRAW;
          break;
        default:
          QMMF_ERROR("%s: %s() unsupported AAC format: %d", TAG, __func__,
                     codec_param.audio_dec_param.codec_params.aac.format);
          return ::android::BAD_VALUE;
      }
      switch (codec_param.audio_dec_param.codec_params.aac.mode) {
        case AACMode::kAALC:
          aac_params.eAACProfile = OMX_AUDIO_AACObjectLC;
          break;
        case AACMode::kHEVC_v1:
          aac_params.eAACProfile = OMX_AUDIO_AACObjectHE;
          break;
        case AACMode::kHEVC_v2:
          aac_params.eAACProfile = OMX_AUDIO_AACObjectHE_PS;
          break;
        default:
          QMMF_ERROR("%s: %s() unsupported AAC mode: %d", TAG, __func__,
                     codec_param.audio_dec_param.codec_params.aac.mode);
          return ::android::BAD_VALUE;
      }
      result = omx_client_->SetParameter(OMX_IndexParamAudioAac,
                                         static_cast<OMX_PTR>(&aac_params));
      if (result != OMX_ErrorNone) {
        QMMF_ERROR("%s: %s() failed to set AAC parameters: %d", TAG, __func__,
                   result);
        return ::android::FAILED_TRANSACTION;
      }
      break;
    }

    case ::qmmf::player::AudioCodecType::kAMR:
      // set the AMR output parameters
      OMX_AUDIO_PARAM_AMRTYPE amr_params;
      InitOMXParams(&amr_params);
      amr_params.nPortIndex = audio_ports.nStartPortNumber;
      amr_params.nChannels = codec_param.audio_dec_param.channels;
      amr_params.nBitRate = codec_param.audio_dec_param.bitrate;
      amr_params.eAMRBandMode =
        pickModeFromBitRate(codec_param.audio_dec_param.codec_params.amr.isWAMR,
            codec_param.audio_dec_param.bitrate);
      result = omx_client_->SetParameter(OMX_IndexParamAudioAmr,
                                         static_cast<OMX_PTR>(&amr_params));
      if (result != OMX_ErrorNone) {
        QMMF_ERROR("%s: %s() failed to set AMR parameters: %d", TAG, __func__,
                   result);
        return ::android::FAILED_TRANSACTION;
      }
      break;
    case ::qmmf::player::AudioCodecType::kG711:
      // set the G711 output parameters
      OMX_AUDIO_PARAM_G711TYPE g711_params;
      InitOMXParams(&g711_params);
      g711_params.nPortIndex = audio_ports.nStartPortNumber;
      g711_params.nChannels = codec_param.audio_dec_param.channels;
      g711_params.nSamplingRate = codec_param.audio_dec_param.sample_rate;
      result = omx_client_->SetParameter(OMX_IndexParamAudioG711,
                                         static_cast<OMX_PTR>(&g711_params));
      if (result != OMX_ErrorNone) {
        QMMF_ERROR("%s: %s() failed to set G711 parameters: %d", TAG, __func__,
                   result);
        return ::android::FAILED_TRANSACTION;
      }
      break;
    default:
      QMMF_ERROR("%s: %s() unknown audio codec: %d", TAG, __func__,
                 static_cast<int>(codec_param.audio_dec_param.codec));
      return ::android::BAD_VALUE;
  }

  //Confuguring Output Port Parameters
  status_t ret;
  InitOMXParams(&output_port);
  output_port.nPortIndex = audio_ports.nStartPortNumber + 1;
  ret = omx_client_->GetParameter(OMX_IndexParamPortDefinition,&output_port);

  if (ret != 0) {
    return ret;
  }

  output_port.format.audio.eEncoding = OMX_AUDIO_CodingPCM;

  ret = omx_client_->SetParameter(OMX_IndexParamPortDefinition,&output_port);
  if(ret != 0) {
    return ret;
  }

  OMX_AUDIO_PARAM_PCMMODETYPE pcm_params;
  InitOMXParams(&pcm_params);
  pcm_params.nPortIndex = audio_ports.nStartPortNumber + 1;
  ret = omx_client_->GetParameter(OMX_IndexParamAudioPcm,&pcm_params);
  if(ret != 0) {
    return ret;
  }

  pcm_params.nChannels = codec_param.audio_dec_param.channels;
  pcm_params.eNumData = OMX_NumericalDataSigned;
  pcm_params.bInterleaved = OMX_TRUE;
  /*Ignore whatever the above layer is providing the value of bit_depth
   * since only 16 is supported in DSP
   */
  pcm_params.nBitPerSample = 16;
  pcm_params.nSamplingRate = codec_param.audio_dec_param.sample_rate;
  pcm_params.ePCMMode = OMX_AUDIO_PCMModeLinear;

  //TODO: initialize the data member eChannelMapping of structure pcm_params
  ret = omx_client_->SetParameter(OMX_IndexParamAudioPcm,&pcm_params);
  if(ret != 0) {
    return ret;
  }

  QMMF_INFO("%s:%s Exit",TAG,__func__);
  return ::android::NO_ERROR;
}

status_t AVCodec::SetPortParams(OMX_U32 port, OMX_U32 width, OMX_U32 height,
                                float frame_rate) {

  status_t ret = 0;
  OMX_PARAM_PORTDEFINITIONTYPE port_def;
  InitOMXParams(&port_def);

  port_def.nPortIndex = port;
  ret = omx_client_->GetParameter(OMX_IndexParamPortDefinition,
                                 (OMX_PTR)&port_def);
  if(ret != 0) {
    QMMF_ERROR("%s:%s Failed to get OMX_IndexParamPortDefinition",TAG,__func__);
    return ret;
  }

  port_def.format.video.xFramerate = (frame_rate * (1 << 16));
  port_def.format.video.nFrameWidth = width;
  port_def.format.video.nFrameHeight = height;

  ret = omx_client_->SetParameter(OMX_IndexParamPortDefinition,
                                 (OMX_PTR)&port_def);
  if(ret != 0) {
    QMMF_ERROR("%s:%s failed to set OMX_IndexParamPortDefinition",TAG,__func__);
    return ret;
  }

  if (port == kPortIndexOutput && format_type_ == CodecType::kVideoEncoder) {
    OMX_QCOM_PARAM_PORTDEFINITIONTYPE outputPortFmt;
    InitOMXParams(&outputPortFmt);
    outputPortFmt.nPortIndex = kPortIndexOutput;
    outputPortFmt.nMemRegion = OMX_QCOM_MemRegionSMI;
    status_t result = omx_client_->SetParameter(
        (OMX_INDEXTYPE)OMX_QcomIndexPortDefn, (OMX_PTR)&outputPortFmt);
    if (result != OK) {
      QMMF_ERROR("%s:%s Failed to initialize OMX_QCOM_PARAM_PORTDEFINITIONTYPE"
          " for %s", TAG, __func__ , PORT_NAME(kPortIndexOutput));
      assert(0);
    }
  }

  return ret;
}

status_t AVCodec::GetBufferRequirements(uint32_t port_type, uint32_t *buf_count,
                                        uint32_t *buf_size) {

  status_t ret = 0;

  OMX_PARAM_PORTDEFINITIONTYPE port_def;
  InitOMXParams(&port_def);

  port_def.nPortIndex = port_type;
  ret = omx_client_->GetParameter(OMX_IndexParamPortDefinition,
                                 (OMX_PTR)&port_def);
  if(ret != 0) {
    QMMF_ERROR("%s:%s Failed to get OMX_IndexParamPortDefinition",
        TAG, __func__);
    return ret;
  }

  *buf_count = port_def.nBufferCountActual;
  *buf_size = port_def.nBufferSize;

  QMMF_INFO("%s:%s %s: buf count(%d), buf size(%d)", TAG, __func__,
      PORT_NAME(port_type), port_def.nBufferCountActual, port_def.nBufferSize);
  return ret;
}

status_t AVCodec::SetupAVCEncoderParameters(CodecParam& param) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  uint32_t frame_rate = ceil(param.video_enc_param.frame_rate);
  uint32_t iframe_interval = param.video_enc_param.codec_param.avc.idr_interval;

  OMX_VIDEO_PARAM_AVCTYPE h264_type;
  InitOMXParams(&h264_type);
  h264_type.nPortIndex = kPortIndexOutput;

  ret = omx_client_->GetParameter(OMX_IndexParamVideoAvc,
                                  reinterpret_cast<void*>(&h264_type));
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to get AVC video param", TAG, __func__);
    return ret;
  }

  h264_type.nAllowedPictureTypes =
      OMX_VIDEO_PictureTypeI | OMX_VIDEO_PictureTypeP;
  h264_type.eProfile =
      static_cast<OMX_VIDEO_AVCPROFILETYPE>(QmmftoOmxProfile(param));
  h264_type.eLevel = static_cast<OMX_VIDEO_AVCLEVELTYPE>(QmmftoOmxLevel(param));

  if (param.video_enc_param.do_vqzip) {
      if ((param.video_enc_param.codec_param.avc.profile !=
           param.video_enc_param.vqzip_params.avc_vqzip_info.profile) ||
          (param.video_enc_param.codec_param.avc.level !=
           param.video_enc_param.vqzip_params.avc_vqzip_info.level)) {
        QMMF_ERROR("%s:%s values of profile/level in vqzip_params and the codec_param differ",
                   TAG, __func__);
        return -1;
      } else {
        OMX_QTI_VIDEO_PARAM_VQZIP_SEI_TYPE enable_sei_params;
        InitOMXParams(&enable_sei_params);
        enable_sei_params.bEnable = OMX_TRUE;
        ret = omx_client_->SetParameter(
            static_cast<OMX_INDEXTYPE>(OMX_QTIIndexParamVQZIPSEIType),
            reinterpret_cast<OMX_PTR>(&enable_sei_params));
        if (ret != 0) {
          QMMF_ERROR("%s:%s Failed to Set SEI extradata data for Encoder",
                     TAG, __func__);
          return ret;
        }
      }
  }

  if(h264_type.eProfile == OMX_VIDEO_AVCProfileBaseline) {
    h264_type.nSliceHeaderSpacing = 0;
    h264_type.bUseHadamard = OMX_TRUE;
    h264_type.nRefFrames = 1;
    h264_type.nBFrames = 0;
    h264_type.nPFrames = frame_rate*iframe_interval;
    if(h264_type.nPFrames == 0) {
      h264_type.nAllowedPictureTypes = OMX_VIDEO_PictureTypeI;
    }
    h264_type.nRefIdx10ActiveMinus1 = 0;
    h264_type.nRefIdx11ActiveMinus1 = 0;
    // A moment of truth: By default all the AVC videos(recorded through qmmf-sdk)
    // having Baseline as the profile are using CAVALC instead of CABAC.
    // Hence this is_cabac_used will always be false here
    if (param.video_enc_param.do_vqzip)
      h264_type.bEntropyCodingCABAC =
          param.video_enc_param.vqzip_params.avc_vqzip_info.is_cabac_used ?
          OMX_TRUE : OMX_FALSE;
    else
      h264_type.bEntropyCodingCABAC = OMX_FALSE;
    h264_type.bWeightedPPrediction = OMX_FALSE;
    h264_type.bconstIpred = OMX_FALSE;
    h264_type.bDirect8x8Inference = OMX_FALSE;
    h264_type.bDirectSpatialTemporal = OMX_FALSE;
    h264_type.nCabacInitIdc = 0;
  } else {
    h264_type.nSliceHeaderSpacing = 0;
    h264_type.bUseHadamard = OMX_TRUE;
    h264_type.nRefFrames = 2;
    h264_type.nBFrames = 0;
    h264_type.nPFrames = frame_rate*iframe_interval;
    h264_type.nAllowedPictureTypes =
        OMX_VIDEO_PictureTypeI | OMX_VIDEO_PictureTypeP;
    h264_type.nRefIdx10ActiveMinus1 = 0;
    h264_type.nRefIdx11ActiveMinus1 = 0;
    // A moment of truth: By default all the AVC videos(recorded through qmmf-sdk)
    // having the profile other than Baseline are using CABAC.
    // Hence this is_cabac_used will always be true here
    if (param.video_enc_param.do_vqzip)
        h264_type.bEntropyCodingCABAC =
            param.video_enc_param.vqzip_params.avc_vqzip_info.is_cabac_used ?
            OMX_TRUE : OMX_FALSE;
    else
      h264_type.bEntropyCodingCABAC = OMX_TRUE;
    h264_type.bWeightedPPrediction = OMX_TRUE;
    h264_type.bconstIpred = OMX_TRUE;
    h264_type.bDirect8x8Inference = OMX_TRUE;
    h264_type.bDirectSpatialTemporal = OMX_TRUE;
    h264_type.nCabacInitIdc = 1;
  }

  if (h264_type.nBFrames != 0) {
      h264_type.nAllowedPictureTypes |= OMX_VIDEO_PictureTypeB;
  }

  h264_type.bEnableUEP = OMX_FALSE;
  h264_type.bEnableFMO = OMX_FALSE;
  h264_type.bEnableASO = OMX_FALSE;
  h264_type.bEnableRS = OMX_FALSE;
  h264_type.bFrameMBsOnly = OMX_TRUE;
  h264_type.bMBAFF = OMX_FALSE;
  h264_type.eLoopFilterMode = OMX_VIDEO_AVCLoopFilterEnable;

  ret = omx_client_->SetParameter(OMX_IndexParamVideoAvc,
                                  reinterpret_cast<void*>(&h264_type));
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to set AVC codec parameter", TAG, __func__);
    return ret;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t AVCodec::SetupHEVCEncoderParameters(CodecParam& param) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  uint32_t frame_rate = ceil(param.video_enc_param.frame_rate);
  uint32_t iframe_interval = param.video_enc_param.codec_param.hevc.idr_interval;

  OMX_VIDEO_PARAM_HEVCTYPE hevc_type;
  InitOMXParams(&hevc_type);
  hevc_type.nPortIndex = kPortIndexOutput;

  ret = omx_client_->GetParameter(
      static_cast<OMX_INDEXTYPE>(OMX_IndexParamVideoHevc),
      reinterpret_cast<void*>(&hevc_type));
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to get HEVC video param", TAG, __func__);
    return ret;
  }

  hevc_type.eProfile =
      static_cast<OMX_VIDEO_HEVCPROFILETYPE>(QmmftoOmxProfile(param));
  hevc_type.eLevel =
      static_cast<OMX_VIDEO_HEVCLEVELTYPE>(QmmftoOmxLevel(param));

  ret = omx_client_->SetParameter(
      static_cast<OMX_INDEXTYPE>(OMX_IndexParamVideoHevc),
      reinterpret_cast<void*>(&hevc_type));
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to get HEVC video param", TAG, __func__);
    return ret;
  }

  QOMX_VIDEO_INTRAPERIODTYPE intra;
  InitOMXParams(&intra);
  intra.nPortIndex = kPortIndexOutput;
  ret = omx_client_->GetConfig(
      static_cast<OMX_INDEXTYPE>(QOMX_IndexConfigVideoIntraperiod),
      reinterpret_cast<OMX_PTR>(&intra));
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to get video intra period", TAG, __func__);
    return ret;
  }
  intra.nPFrames = frame_rate * iframe_interval;
  // TODO: remove hard code B frame value
  intra.nBFrames = 0;
  ret = omx_client_->SetConfig(
      static_cast<OMX_INDEXTYPE>(QOMX_IndexConfigVideoIntraperiod),
      reinterpret_cast<OMX_PTR>(&intra));
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to set video intra period", TAG, __func__);
    return ret;
  }

  if (param.video_enc_param.do_vqzip) {
    QMMF_ERROR("%s:%s VQZip feature is not supported for HEVC encoded videos",
               TAG, __func__);
    return -1;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t AVCodec::QmmftoOmxProfile(CodecParam& param) {

  int32_t profile = -1;
  VideoFormat codec_format = param.video_enc_param.format_type;
  switch (codec_format) {
    case VideoFormat::kAVC:
      switch(param.video_enc_param.codec_param.avc.profile) {
        case AVCProfileType::kBaseline:
          profile = OMX_VIDEO_AVCProfileBaseline;
         break;
        case AVCProfileType::kMain:
          profile = OMX_VIDEO_AVCProfileMain;
          break;
        case AVCProfileType::kHigh:
          profile = OMX_VIDEO_AVCProfileHigh ;
          break;
      }
      break;

    case VideoFormat::kHEVC:
      switch (param.video_enc_param.codec_param.hevc.profile) {
        case HEVCProfileType::kMain:
          profile = OMX_VIDEO_HEVCProfileMain;
          break;
      }
      break;

    default:
      QMMF_ERROR("%s:%s Unknown codec type(%d)", TAG, __func__, codec_format);
      break;
  }
  return profile;
}

status_t AVCodec::OmxtoQmmfProfile(const VideoFormat format,
                                  const int32_t profile, void *param) {

  switch (format) {
    case VideoFormat::kAVC: {
      AVCProfileType* avc_profile = reinterpret_cast<AVCProfileType*>(param);
      switch (profile) {
        case OMX_VIDEO_AVCProfileBaseline:
          *avc_profile = AVCProfileType::kBaseline;
          break;
        case OMX_VIDEO_AVCProfileMain:
          *avc_profile = AVCProfileType::kMain;
          break;
        case OMX_VIDEO_AVCProfileHigh:
          *avc_profile = AVCProfileType::kHigh;
          break;
        default:
          QMMF_ERROR("%s:%s Unknown Profile from OMX %d", TAG, __func__,
                     profile);
          return -1;
      }
      break;
    }

    case VideoFormat::kHEVC: {
      HEVCProfileType* hevc_profile = reinterpret_cast<HEVCProfileType*>
                                                      (param);
      switch (profile) {
        case OMX_VIDEO_HEVCProfileMain:
          *hevc_profile = HEVCProfileType::kMain;
          break;
        default:
          QMMF_ERROR("%s:%s Unknown Profile from OMX %d", TAG, __func__,
                     profile);
          return -1;
      }
      break;
    }

    default:
      QMMF_ERROR("%s:%s Unknown VideoFormat(%d)", TAG, __func__,
                 static_cast<underlying_type<VideoFormat>::type>(format));
      return -1;
  }
  return 0;
}

status_t AVCodec::QmmftoOmxLevel(CodecParam& param) {

  int32_t level = -1;
  VideoFormat codec_format = param.video_enc_param.format_type;
  switch (codec_format) {
    case VideoFormat::kAVC:
      switch (param.video_enc_param.codec_param.avc.level) {
        case AVCLevelType::kLevel1:
          level = OMX_VIDEO_AVCLevel1;
          break;
        case AVCLevelType::kLevel1_3:
          level = OMX_VIDEO_AVCLevel13;
          break;
        case AVCLevelType::kLevel2:
          level = OMX_VIDEO_AVCLevel2;
          break;
        case AVCLevelType::kLevel2_1:
          level = OMX_VIDEO_AVCLevel21;
          break;
        case AVCLevelType::kLevel2_2:
          level = OMX_VIDEO_AVCLevel22;
          break;
        case AVCLevelType::kLevel3:
          level = OMX_VIDEO_AVCLevel3;
          break;
        case AVCLevelType::kLevel3_1:
          level = OMX_VIDEO_AVCLevel31;
          break;
        case AVCLevelType::kLevel3_2:
          level = OMX_VIDEO_AVCLevel32;
          break;
        case AVCLevelType::kLevel4:
          level = OMX_VIDEO_AVCLevel4;
          break;
        case AVCLevelType::kLevel4_1:
          level = OMX_VIDEO_AVCLevel41;
          break;
        case AVCLevelType::kLevel4_2:
          level = OMX_VIDEO_AVCLevel42;
          break;
        case AVCLevelType::kLevel5:
          level = OMX_VIDEO_AVCLevel5;
          break;
        case AVCLevelType::kLevel5_1:
          level = OMX_VIDEO_AVCLevel51;
          break;
        case AVCLevelType::kLevel5_2:
          level = OMX_VIDEO_AVCLevel52;
          break;
      }
      break;

    case VideoFormat::kHEVC:
        switch (param.video_enc_param.codec_param.hevc.level) {
          case HEVCLevelType::kLevel3:
            level = OMX_VIDEO_HEVCMainTierLevel3;
            break;
          case HEVCLevelType::kLevel4:
            level = OMX_VIDEO_HEVCMainTierLevel4;
            break;
          case HEVCLevelType::kLevel5:
            level = OMX_VIDEO_HEVCMainTierLevel5;
            break;
          case HEVCLevelType::kLevel5_1:
            level = OMX_VIDEO_HEVCMainTierLevel41;
            break;
          case HEVCLevelType::kLevel5_2:
            level = OMX_VIDEO_HEVCMainTierLevel52;
            break;
        }
        break;

    default:
      QMMF_ERROR("%s:%s Unknown codec type(%d)", TAG, __func__, codec_format);
      break;
  }
  return level;
}

status_t AVCodec::OmxtoQmmfLevel(const VideoFormat format, const int32_t level,
                                void *param) {
  switch (format) {
    case VideoFormat::kAVC: {
      AVCLevelType* avc_level = reinterpret_cast<AVCLevelType*>(param);
      switch (level) {
        case OMX_VIDEO_AVCLevel1:
          *avc_level = AVCLevelType::kLevel1;
          break;
        case OMX_VIDEO_AVCLevel13:
          *avc_level = AVCLevelType::kLevel1_3;
          break;
        case OMX_VIDEO_AVCLevel2:
          *avc_level = AVCLevelType::kLevel2;
          break;
        case OMX_VIDEO_AVCLevel21:
          *avc_level = AVCLevelType::kLevel2_1;
          break;
        case OMX_VIDEO_AVCLevel22:
          *avc_level = AVCLevelType::kLevel2_2;
          break;
        case OMX_VIDEO_AVCLevel3:
          *avc_level = AVCLevelType::kLevel3;
          break;
        case OMX_VIDEO_AVCLevel31:
          *avc_level = AVCLevelType::kLevel3_1;
          break;
        case OMX_VIDEO_AVCLevel32:
          *avc_level = AVCLevelType::kLevel3_2;
          break;
        case OMX_VIDEO_AVCLevel4:
          *avc_level = AVCLevelType::kLevel4;
          break;
        case OMX_VIDEO_AVCLevel41:
          *avc_level = AVCLevelType::kLevel4_1;
          break;
        case OMX_VIDEO_AVCLevel42:
          *avc_level = AVCLevelType::kLevel4_2;
          break;
        case OMX_VIDEO_AVCLevel5:
          *avc_level = AVCLevelType::kLevel5;
          break;
        case OMX_VIDEO_AVCLevel51:
          *avc_level =  AVCLevelType::kLevel5_1;
          break;
        case OMX_VIDEO_AVCLevel52:
          *avc_level = AVCLevelType::kLevel5_2;
          break;
        default:
          QMMF_ERROR("%s:%s Unknown level from OMX %d", TAG, __func__, level);
          return -1;
      }
      break;
    }

    case VideoFormat::kHEVC: {
      HEVCLevelType* hevc_level = reinterpret_cast<HEVCLevelType*>(param);
      switch (level) {
        case OMX_VIDEO_HEVCMainTierLevel3:
          *hevc_level = HEVCLevelType::kLevel3;
          break;
        case OMX_VIDEO_HEVCMainTierLevel4:
          *hevc_level = HEVCLevelType::kLevel4;
          break;
        case OMX_VIDEO_HEVCMainTierLevel5:
          *hevc_level = HEVCLevelType::kLevel5;
          break;
        case OMX_VIDEO_HEVCMainTierLevel41:
          *hevc_level = HEVCLevelType::kLevel5_1;
          break;
        case OMX_VIDEO_HEVCMainTierLevel52:
          *hevc_level = HEVCLevelType::kLevel5_2;
          break;
        default:
          QMMF_ERROR("%s:%s Unknown level from OMX %d", TAG, __func__, level);
          return -1;
      }
      break;
    }

    default:
      QMMF_ERROR("%s:%s Unknown VideoFormat(%d)", TAG, __func__,
                 static_cast<underlying_type<VideoFormat>::type>(format));
      return -1;
  }
  return 0;
}

OMX_ERRORTYPE AVCodec::ConfigureSAR(uint32_t width, uint32_t height) {
  QOMX_EXTNINDEX_VIDEO_VENC_SAR sar;
  memset(&sar, 0, sizeof(sar));
  sar.nSize = sizeof(QOMX_EXTNINDEX_VIDEO_VENC_SAR);
  sar.nSARWidth = width;
  sar.nSARHeight = height;
  OMX_ERRORTYPE ret = omx_client_->SetParameter(
            (OMX_INDEXTYPE)OMX_QcomIndexParamVencAspectRatio,
            (OMX_PTR)&sar);
  if (ret != OMX_ErrorNone) {
      QMMF_ERROR("%s:%s Failed to configure SAR",
                 TAG, __func__);
      return ret;
  }

  return ret;
}

status_t AVCodec::ConfigureBitrate(CodecParam& param) {

  uint32_t bitrate = 0;
  VideoRateControlType mode;

  VideoFormat codec_format = param.video_enc_param.format_type;
  switch(codec_format) {
    case VideoFormat::kAVC:
      bitrate = param.video_enc_param.codec_param.avc.bitrate;
      mode = param.video_enc_param.codec_param.avc.ratecontrol_type;
      break;
    case VideoFormat::kHEVC:
      bitrate = param.video_enc_param.codec_param.hevc.bitrate;
      mode = param.video_enc_param.codec_param.hevc.ratecontrol_type;
      break;
    default:
      QMMF_ERROR("%s:%s Unknown codec type(%d)", TAG, __func__, codec_format);
      return -1;
  }

  if (param.video_enc_param.do_vqzip && mode != VideoRateControlType::kDisable) {
    QMMF_ERROR("%s:%s RC shold be disabled in VQZip", TAG, __func__);
    return -1;
  }

  OMX_VIDEO_CONTROLRATETYPE control_rate;
  switch(mode) {
    case VideoRateControlType::kDisable:
      control_rate = OMX_Video_ControlRateDisable;
      break;
    case VideoRateControlType::kVariableSkipFrames:
      control_rate = OMX_Video_ControlRateVariableSkipFrames;
      break;
    case VideoRateControlType::kVariable:
      control_rate = OMX_Video_ControlRateVariable;
      break;
    case VideoRateControlType::kConstantSkipFrames:
      control_rate = OMX_Video_ControlRateConstantSkipFrames;
      break;
    case VideoRateControlType::kConstant:
      control_rate = OMX_Video_ControlRateConstant;
      break;
    case VideoRateControlType::kMaxBitrate:
      control_rate = static_cast<OMX_VIDEO_CONTROLRATETYPE>
                        QOMX_Video_ControlRateMaxBitrate; //MBR_CFR
      break;
    case VideoRateControlType::kMaxBitrateSkipFrames:
      control_rate = static_cast<OMX_VIDEO_CONTROLRATETYPE>
                        QOMX_Video_ControlRateMaxBitrateSkipFrames; //MBR_VFR
      break;
    default:
      control_rate = OMX_Video_ControlRateVariable;
      break;
  }

  OMX_VIDEO_PARAM_BITRATETYPE bitrate_type;
  InitOMXParams(&bitrate_type);
  bitrate_type.nPortIndex = kPortIndexOutput;

  status_t ret = omx_client_->GetParameter(OMX_IndexParamVideoBitrate,
                                           &bitrate_type);
  if (ret != OK) {
    QMMF_ERROR("%s:%s Failed to get OMX_IndexParamVideoBitrate", TAG, __func__);
    return ret;
  }

  bitrate_type.eControlRate = control_rate;
  bitrate_type.nTargetBitrate = bitrate;

  ret = omx_client_->SetParameter(OMX_IndexParamVideoBitrate, &bitrate_type);
  if (ret != OK) {
    QMMF_ERROR("%s:%s Failed to set OMX_IndexParamVideoBitrate", TAG, __func__);
    return ret;
  }

  return ret;
}

status_t AVCodec::AllocateBuffer(uint32_t port_type, uint32_t buf_count,
                                 uint32_t buf_size,
                                 const shared_ptr<ICodecSource>& source,
                                 vector<BufferDescriptor> &buffer_list) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;

  OMX_PARAM_PORTDEFINITIONTYPE port_def;
  InitOMXParams(&port_def);
  port_def.nPortIndex = port_type;
  ret = omx_client_->GetParameter(OMX_IndexParamPortDefinition, &port_def);
  if(ret != OK) {
      QMMF_ERROR("%s:%s Failed to getParameter on %s", TAG, __func__,
          PORT_NAME(port_type));
      return ret;
  }

  if (format_type_ == CodecType::kVideoEncoder) {
    uint32_t buf_count = (port_type == kPortIndexInput) ?
                             INPUT_MAX_COUNT : port_def.nBufferCountActual;

    if(port_def.nBufferCountActual != buf_count) {

      port_def.nBufferCountActual = buf_count;
      ret = omx_client_->SetParameter(OMX_IndexParamPortDefinition,
                                     (OMX_PTR)&port_def);
      if(ret != OK) {
        QMMF_ERROR("%s:%s Failed to set new buffer count(%d) on %s", TAG,
            __func__, port_def.nBufferCountActual, PORT_NAME(port_type));
        return ret;
      }
      ret = omx_client_->GetParameter(OMX_IndexParamPortDefinition, &port_def);
      if(ret != OK) {
        QMMF_ERROR("%s:%s Failed to getParameter on %s", TAG, __func__,
            PORT_NAME(port_type));
        return ret;
      }
      QMMF_INFO("%s:%s New Buf count(%d), size(%d)", TAG, __func__,
          port_def.nBufferCountActual, port_def.nBufferSize);
      assert(buf_count == port_def.nBufferCountActual);
    }

    if (port_type == kPortIndexInput)
      in_buff_hdr_size_ = port_def.nBufferCountActual;
    else
      out_buff_hdr_size_ = port_def.nBufferCountActual;
  }

  if(port_type == kPortIndexInput) {
    assert(source.get() != nullptr);
    input_source_ = source;

    //allocate memory for buffer header
    in_buff_hdr_ = new OMX_BUFFERHEADERTYPE*[port_def.nBufferCountActual];
    if(in_buff_hdr_ ==  nullptr) {
      QMMF_ERROR("%s:%s Failed to allocate buffer header on %s", TAG, __func__,
          PORT_NAME(kPortIndexInput));
      return NO_MEMORY;
    }

    if (format_type_ == CodecType::kVideoEncoder) {
      StoreMetaDataInBuffersParams meta_mode;
      InitOMXParams(&meta_mode);
      meta_mode.nPortIndex = kPortIndexInput;
      meta_mode.bStoreMetaData = OMX_TRUE;
      ret = omx_client_->SetParameter(
                (OMX_INDEXTYPE)OMX_QcomIndexParamVideoMetaBufferMode,
                (OMX_PTR)&meta_mode);
      if(ret != OK) {
        QMMF_ERROR("%s:%s Failed to set VideoEncode MetaBufferMode",
            TAG, __func__);
        return ret;
      }
    }

  } else {
    assert(source.get() != nullptr);
    output_source_ = source;

    out_buff_hdr_ = new OMX_BUFFERHEADERTYPE*[port_def.nBufferCountActual];
    if(out_buff_hdr_ ==  nullptr) {
        QMMF_ERROR("%s:%s Failed to allocate buffer header on %s",
                   TAG, __func__, PORT_NAME(kPortIndexOutput));
        return NO_MEMORY;
    }

    if (format_type_ == CodecType::kVideoDecoder) {
      StoreMetaDataInBuffersParams meta_mode;
      InitOMXParams(&meta_mode);
      meta_mode.nPortIndex = kPortIndexOutput;
      meta_mode.bStoreMetaData = OMX_TRUE;
      ret = omx_client_->SetParameter(
                (OMX_INDEXTYPE)OMX_QcomIndexParamVideoMetaBufferMode,
                (OMX_PTR)&meta_mode);
      if (ret != OK) {
        QMMF_ERROR("%s:%s Failed to set VideoDecode MetaBufferMode",
            TAG, __func__);
        return ret;
      }
    }
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t AVCodec::ReleaseBuffer() {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;

  DeleteHandle();

  delete []in_buff_hdr_;
  in_buff_hdr_ = nullptr;

  delete []out_buff_hdr_;
  out_buff_hdr_ = nullptr;

  assert(signal_queue_.Size() == 0);

  signal_queue_.Clear();

  for (auto& iter: outputpParam_dec_) {
    delete iter.pHandle;
    iter.pHandle = nullptr;
  }

  outputpParam_dec_.clear();
  outputpParam_enc_.clear();

  input_source_ = nullptr;
  output_source_ = nullptr;
  port_status_ = true;

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t AVCodec::StartCodec() {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;

  isEOSonOutput_ = false;

  QMMF_INFO("%s:%s current state(%s), pending state(%s)", TAG, __func__,
      OMX_STATE_NAME(state_), OMX_STATE_NAME(state_pending_));

  if(port_status_ == false) {
    ret = omx_client_->SendCommand(OMX_CommandPortEnable, kPortIndexInput,
                                   nullptr);
    if(ret != 0) {
        QMMF_ERROR("%s:%s Failed to enable port on %s", TAG, __func__,
            PORT_NAME(kPortIndexInput));
        return ret;
    }

    ret = omx_client_->SendCommand(OMX_CommandPortEnable, kPortIndexOutput,
                                   nullptr);
    if(ret != 0) {
        QMMF_ERROR("%s:%s Failed to enable port on %s", TAG, __func__,
            PORT_NAME(kPortIndexOutput));
        return ret;
    }

    if (format_type_ == CodecType::kVideoDecoder) {
      ConfigureVideoDecoder(codec_params_);
    }
  }

  OMX_PARAM_PORTDEFINITIONTYPE port_def;
  InitOMXParams(&port_def);
  port_def.nPortIndex = kPortIndexInput;
  ret = omx_client_->GetParameter(OMX_IndexParamPortDefinition,
            (OMX_PTR)&port_def);
  if(ret != OK) {
    QMMF_ERROR("%s:%s Failed to get port definiton on %s", TAG, __func__,
        PORT_NAME(kPortIndexInput));
    return ret;
  }
  uint32_t buf_size = port_def.nBufferSize;

  if (format_type_ == CodecType::kVideoEncoder) {
    for (uint32_t i = 0; i < port_def.nBufferCountActual; ++i) {
      buf_size = sizeof(encoder_media_buffer_type);
      ret = omx_client_->AllocateBuffer(&in_buff_hdr_[i], kPortIndexInput,
                                        nullptr, buf_size);
      if(ret != OK) {
          QMMF_ERROR("%s:%s Failed to allocate buffer on %s", TAG, __func__,
              PORT_NAME(kPortIndexInput));
          return ret;
      }

      QMMF_INFO("%s:%s allocate buffer on %s pBuffer %p  i %d", TAG, __func__,
            PORT_NAME(kPortIndexInput), in_buff_hdr_[i]->pBuffer, i);

      encoder_media_buffer_type* mediaBuffer =
          (encoder_media_buffer_type*)in_buff_hdr_[i]->pBuffer;
      assert(mediaBuffer != nullptr);
      mediaBuffer->buffer_type =
          MetadataBufferType::kMetadataBufferTypeGrallocSource;
      mediaBuffer->meta_handle = nullptr;
      std::lock_guard<std::mutex> lock(queue_lock_);
      free_input_buffhdr_list_.PushBack(in_buff_hdr_[i]);
    }
  } else if (format_type_ == CodecType::kVideoDecoder) {
    for (uint32_t i = 0; i < port_def.nBufferCountActual; ++i) {
      ret = omx_client_->UseBuffer(&in_buff_hdr_[i], kPortIndexInput, nullptr,
                            buf_size,
                            static_cast<OMX_U8*>(input_buffer_list_[i].data));
      if (ret != OK) {
          QMMF_ERROR("%s:%s Failed to allocate buffer on %s", TAG, __func__,
              PORT_NAME(kPortIndexInput));
          return ret;
      }
    }
  } else {
    for (uint32_t i = 0; i < port_def.nBufferCountActual; ++i) {
      ret = omx_client_->UseBuffer(&in_buff_hdr_[i], kPortIndexInput, nullptr,
                                   buf_size, nullptr);
      if(ret != OK) {
          QMMF_ERROR("%s:%s Failed to allocate buffer on %s", TAG, __func__,
                     PORT_NAME(kPortIndexInput));
          return ret;
      }
    }
  }

  InitOMXParams(&port_def);
  port_def.nPortIndex = kPortIndexOutput;
  ret = omx_client_->GetParameter(OMX_IndexParamPortDefinition,
            (OMX_PTR)&port_def);
  if(ret != OK) {
    QMMF_ERROR("%s:%s Failed to get port definiton on %s", TAG, __func__,
        PORT_NAME(kPortIndexOutput));
    return ret;
  }
  buf_size = port_def.nBufferSize;

  if (format_type_ == CodecType::kVideoEncoder) {
    for (uint32_t i = 0; i < port_def.nBufferCountActual; ++i) {
      ret = omx_client_->UseBuffer(&out_buff_hdr_[i], kPortIndexOutput,
                            static_cast<OMX_PTR>(&(outputpParam_enc_[i])),
                            buf_size,
                            static_cast<OMX_U8*>(output_buffer_list_[i].data));
      if(ret != OK) {
        QMMF_ERROR("%s:%s Failed to allocate buffer on %s", TAG, __func__,
                   PORT_NAME(kPortIndexOutput));
        return ret;
      }
    }
  } else if (format_type_ == CodecType::kVideoDecoder) {
    for (uint32_t i = 0; i < port_def.nBufferCountActual; ++i) {
      buf_size = sizeof(struct VideoDecoderOutputMetaData);
      ret = omx_client_->UseBuffer(&out_buff_hdr_[i], kPortIndexOutput, nullptr,
                              buf_size,
                              reinterpret_cast<OMX_U8*>(&outputpParam_dec_[i]));
      if(ret != OK) {
          QMMF_ERROR("%s:%s Failed to allocate buffer on %s", TAG, __func__,
              PORT_NAME(kPortIndexOutput));
          return ret;
      }

      struct VideoDecoderOutputMetaData* pParam =
          (struct VideoDecoderOutputMetaData*)out_buff_hdr_[i]->pBuffer;
      assert(pParam != nullptr);
      free_output_buffhdr_list_.PushBack(out_buff_hdr_[i]);
    }
  } else if(format_type_ == CodecType::kAudioEncoder) {
    for (uint32_t i = 0; i < port_def.nBufferCountActual; ++i) {
      ret = omx_client_->AllocateBuffer(&out_buff_hdr_[i], kPortIndexOutput,
                                        nullptr, buf_size);
      if(ret != OK) {
        QMMF_ERROR("%s:%s Failed to allocate buffer on %s", TAG, __func__,
                   PORT_NAME(kPortIndexInput));
        return ret;
      }

      BufferDescriptor* buffer = new BufferDescriptor;
      buffer->data = nullptr;
      buffer->fd = -1;
      out_buff_hdr_[i]->pAppPrivate = reinterpret_cast<OMX_PTR>(buffer);
      QMMF_VERBOSE("%s:%s allocated pBuffer[%p] and pAppPrivate[%p]", TAG,
                   __func__, out_buff_hdr_[i]->pBuffer,
                   out_buff_hdr_[i]->pAppPrivate);
    }
  } else {
      for (uint32_t i = 0; i < port_def.nBufferCountActual; ++i) {
        ret = omx_client_->UseBuffer(&out_buff_hdr_[i], kPortIndexOutput,
                                     nullptr, buf_size, nullptr);
        if(ret != OK) {
          QMMF_ERROR("%s:%s Failed to allocated buffer on %s", TAG, __func__,
          PORT_NAME(kPortIndexOutput));
          return ret;
        }
      }
  }

  if(port_status_ == false) {
    CodecCmdType cmd;
    ret = signal_queue_.Pop(&cmd);
    if (ret != OK) {
      QMMF_ERROR("%s:%s Pop from SignalQueue Failed, size(%u)",
          TAG, __func__, signal_queue_.Size());
      return ret;
    }

    QMMF_INFO("%s:%s Popped buffer from cmd queue, size(%u)", TAG,
        __func__, signal_queue_.Size());

    if((cmd.event_result != OMX_ErrorNone) ||
        (cmd.event_type != OMX_EventCmdComplete) ||
        (cmd.event_cmd != OMX_CommandPortEnable)) {
      QMMF_ERROR("%s:%s Expecting Cmd complete vs command found(%d)", TAG,
          __func__, cmd.event_cmd);
      return cmd.event_result;
    }

    ret = signal_queue_.Pop(&cmd);
    if (ret != OK) {
      QMMF_ERROR("%s:%s Pop from SignalQueue Failed, size(%u)",
          TAG, __func__, signal_queue_.Size());
      return ret;
    }

    QMMF_INFO("%s:%s Popped buffer from cmd queue, size(%u)", TAG,
        __func__, signal_queue_.Size());

    if((cmd.event_result != OMX_ErrorNone) ||
       (cmd.event_type != OMX_EventCmdComplete) ||
       (cmd.event_cmd != OMX_CommandPortEnable)) {
      QMMF_ERROR("%s:%s Expecting Cmd complete vs command found(%d)", TAG,
          __func__, cmd.event_cmd);
      return cmd.event_result;
    }

    port_status_ = true;
  }

  ret = WaitState(OMX_StateIdle);
  if(ret != OK) {
    QMMF_ERROR("%s:%s Wait for state %s failed", TAG, __func__,
        OMX_STATE_NAME(OMX_StateIdle));
    return ret;
  }

  QMMF_INFO("%s:%s Move to Component to Executing state", TAG, __func__);
  ret = SetState(OMX_StateExecuting, OMX_TRUE);
  assert(ret == OK);

  {
    Mutex::Autolock autoLock(input_stop_lock_);
    input_stop_ = false;
  }

  {
    Mutex::Autolock autoLock(output_stop_lock_);
    output_stop_ = false;
  }

  pthread_create(&deliver_input_thread_id_, nullptr, DeliverInput, (void*)this);
  pthread_create(&deliver_output_thread_id_, nullptr, DeliverOutput, (void*)this);
  if (format_type_ == CodecType::kVideoDecoder) {
    pthread_create(&port_reconfig_thread_id_, nullptr, ThreadRun, (void*)this);
  }

  QMMF_INFO("%s:%s current state(%s), pending state(%s)", TAG, __func__,
      OMX_STATE_NAME(state_), OMX_STATE_NAME(state_pending_));
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t AVCodec::StopCodec() {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;

  if ((state_ == OMX_StateIdle) || (state_pending_ == OMX_StateIdle)) {
    QMMF_WARN("%s:%s Encoder is already in Idle state", TAG, __func__);
    return ret;
  }

  {
    Mutex::Autolock autoLock(input_stop_lock_);
    input_stop_ = true;
  }

  ret = pthread_join(deliver_input_thread_id_, nullptr);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: Failed to join DeliverInput Thread", TAG, __func__);
  }

  ret = pthread_join(deliver_output_thread_id_, nullptr);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: Failed to join DeliverOutput Thread", TAG, __func__);
  }

  if (format_type_ == CodecType::kVideoDecoder) {
    ret = pthread_join(port_reconfig_thread_id_, nullptr);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s:%s: Failed to join ThreadRun Thread", TAG, __func__);
    }
  }

  CodecCmdType cmd;
  ret = signal_queue_.Pop(&cmd);
  if (ret != OK) {
    QMMF_ERROR("%s:%s Pop from SignalQueue Failed, size(%u)",
        TAG, __func__, signal_queue_.Size());
    return ret;
  }

  QMMF_INFO("%s:%s Popped buffer from cmd queue, size(%u)", TAG,
      __func__, signal_queue_.Size());

  if((cmd.event_result != OMX_ErrorNone) ||
     (cmd.event_flags != OMX_BUFFERFLAG_EOS)) {
      QMMF_ERROR("%s:%s Expecting EOS and found(%d) flag", TAG, __func__,
          cmd.event_flags);
      return OMX_ErrorUndefined;
  }

  ret =  SetState(OMX_StateIdle, OMX_TRUE);
  if(ret != 0) {
   QMMF_ERROR("%s:%s Failed to move to OMX_StateIdle state!", TAG, __func__);
   return ret;
  }

  //Disable both port
  ret = omx_client_->SendCommand(OMX_CommandPortDisable, kPortIndexOutput, 0);
  if(ret != 0) {
   QMMF_ERROR("%s:%s Failed to disbale port on %s", TAG, __func__,
       PORT_NAME(kPortIndexOutput));
   return ret;
  }

  ret = omx_client_->SendCommand(OMX_CommandPortDisable, kPortIndexInput, 0);
  if(ret != 0) {
   QMMF_ERROR("%s:%s Failed to disbale port on %s", TAG, __func__,
       PORT_NAME(kPortIndexInput));
   return ret;
  }

  port_status_ = false;

  //DeRegister buffer on both port
  for (uint32_t i = 0; i < in_buff_hdr_size_; i++) {
    ret = omx_client_->FreeBuffer(in_buff_hdr_[i], kPortIndexInput);
    if(ret != 0) {
      QMMF_ERROR("%s:%s Failed to free buffer on %s", TAG, __func__,
          PORT_NAME(kPortIndexInput));
      return ret;
    }
  }
  if (!free_input_buffhdr_list_.Empty())
    free_input_buffhdr_list_.Clear();

  if (!used_input_buffhdr_list_.Empty())
    used_input_buffhdr_list_.Clear();

  if (!free_output_buffhdr_list_.Empty())
    free_output_buffhdr_list_.Clear();

  if (!used_output_buffhdr_list_.Empty())
    used_output_buffhdr_list_.Clear();

  for (uint32_t i = 0; i < out_buff_hdr_size_; i++) {
    if (format_type_ == CodecType::kAudioEncoder)
      delete reinterpret_cast<BufferDescriptor*>(out_buff_hdr_[i]->pAppPrivate);
    ret = omx_client_->FreeBuffer(out_buff_hdr_[i], kPortIndexOutput);
    if(ret != 0) {
      QMMF_ERROR("%s:%s Failed to free buffer on %s", TAG, __func__,
          PORT_NAME(kPortIndexOutput));
      return ret;
    }
  }

  ret = signal_queue_.Pop(&cmd);
  if (ret != OK) {
    QMMF_ERROR("%s:%s Pop from SignalQueue Failed, size(%u)",
        TAG, __func__, signal_queue_.Size());
    return ret;
  }

  QMMF_INFO("%s:%s Popped buffer from cmd queue, size(%u)", TAG,
      __func__, signal_queue_.Size());

  if((cmd.event_result != OMX_ErrorNone) ||
      (cmd.event_type != OMX_EventCmdComplete) ||
      (cmd.event_cmd != OMX_CommandPortDisable)) {
    QMMF_ERROR("%s:%s Expecting Cmd complete vs command found(%d)", TAG,
        __func__, cmd.event_cmd);
    return cmd.event_result;
  }

  ret = signal_queue_.Pop(&cmd);
  if (ret != OK) {
    QMMF_ERROR("%s:%s Pop from SignalQueue Failed, size(%u)",
        TAG, __func__, signal_queue_.Size());
    return ret;
  }

  QMMF_INFO("%s:%s Popped buffer from cmd queue, size(%u)", TAG,
      __func__, signal_queue_.Size());

  if((cmd.event_result != OMX_ErrorNone) ||
     (cmd.event_type != OMX_EventCmdComplete) ||
     (cmd.event_cmd != OMX_CommandPortDisable)) {
    QMMF_ERROR("%s:%s Expecting Cmd complete vs command found(%d)", TAG,
        __func__, cmd.event_cmd);
    return cmd.event_result;
  }

  QMMF_INFO("%s:%s current state(%s), pending state(%s)", TAG, __func__,
      OMX_STATE_NAME(state_), OMX_STATE_NAME(state_pending_));
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

void AVCodec::StopOutput() {

  Mutex::Autolock autoLock(output_stop_lock_);
  output_stop_ = true;
}

status_t AVCodec::PauseCodec() {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;

  ret = omx_client_->SendCommand(OMX_CommandStateSet, OMX_StatePause, 0);
  if (ret != OK) {
    QMMF_ERROR("%s:%s Failed to Pause the Codec/Component", TAG, __func__);
    return ret;
  }
  ret = WaitState(OMX_StatePause);

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t AVCodec::ResumeCodec() {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;

  ret = omx_client_->SendCommand(OMX_CommandStateSet, OMX_StateExecuting, 0);
  if (ret != OK) {
    QMMF_ERROR("%s:%s Failed to Resume Codec/Component", TAG, __func__);
    return ret;
  }
  ret = WaitState(OMX_StateExecuting);

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t AVCodec::SetParameters(CodecParamType param_type, void *codec_param,
                                size_t param_size) {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;
  uint32_t *value;
  float *fps = nullptr;
  VideoEncIdrInterval *idr_interval;
  VideoEncLtrUse *ltr_use;
  OMX_INDEXTYPE index;
  OMX_PARAM_U32TYPE operating_rate_params;

  switch (param_type) {
    case CodecParamType::kBitRateType:
      value = static_cast<uint32_t*>(codec_param);
      OMX_VIDEO_CONFIG_BITRATETYPE bitrate_params;
      InitOMXParams(&bitrate_params);
      bitrate_params.nSize = sizeof(bitrate_params);
      bitrate_params.nPortIndex = kPortIndexOutput;
      bitrate_params.nEncodeBitrate = *value;
      index = OMX_IndexConfigVideoBitrate;
      ret = omx_client_->SetConfig(index, &bitrate_params);
      QMMF_INFO("%s: Setting Bitrate: %u ret: %d\n",
                __func__, bitrate_params.nEncodeBitrate, ret);
      break;
    case CodecParamType::kFrameRateType:
      fps = static_cast<float*>(codec_param);
      OMX_CONFIG_FRAMERATETYPE framerate;
      InitOMXParams(&framerate);
      framerate.nPortIndex = kPortIndexInput;
      ret = omx_client_->GetConfig(OMX_IndexConfigVideoFramerate, &framerate);
      if (ret != NO_ERROR) {
        QMMF_ERROR("%s:%s: GetConfig for type(%d) failed!", TAG, __func__,
            param_type);
        return ret;
      }
      framerate.xEncodeFramerate = ((*fps) * (1 << 16));
      ret = omx_client_->SetConfig(OMX_IndexConfigVideoFramerate, &framerate);
      break;
    case CodecParamType::kInsertIDRType:
      OMX_CONFIG_INTRAREFRESHVOPTYPE idr_params;
      InitOMXParams(&idr_params);
      idr_params.nPortIndex = kPortIndexOutput;
      idr_params.IntraRefreshVOP = OMX_TRUE;
      index = OMX_IndexConfigVideoIntraVOPRefresh;
      ret = omx_client_->SetConfig(index, &idr_params);
      break;
    case CodecParamType::kIDRIntervalType:
      idr_interval = static_cast<VideoEncIdrInterval*>(codec_param);
      QOMX_VIDEO_INTRAPERIODTYPE intra_params;
      InitOMXParams(&intra_params);
      intra_params.nPortIndex = kPortIndexOutput;
      intra_params.nPFrames = idr_interval->num_pframes;
      intra_params.nBFrames = idr_interval->num_bframes;
      intra_params.nIDRPeriod = idr_interval->idr_period;
      index = (OMX_INDEXTYPE)QOMX_IndexConfigVideoIntraperiod;
      ret = omx_client_->SetConfig(index, &intra_params);
      break;
    case CodecParamType::kMarkLtrType:
      value = static_cast<uint32_t*>(codec_param);
      QOMX_VIDEO_CONFIG_LTRMARK_TYPE  matkltr_params;
      InitOMXParams(&matkltr_params);
      matkltr_params.nPortIndex = kPortIndexInput;
      matkltr_params.nID = *value;
      index = (OMX_INDEXTYPE)QOMX_IndexConfigVideoLTRMark;
      ret = omx_client_->SetConfig(index, &matkltr_params);
      break;
    case CodecParamType::kUseLtrType:
      ltr_use = static_cast<VideoEncLtrUse*>(codec_param);
      QOMX_VIDEO_CONFIG_LTRUSE_TYPE useltr_params;
      InitOMXParams(&useltr_params);
      useltr_params.nPortIndex = kPortIndexInput;
      useltr_params.nID = ltr_use->id;
      useltr_params.nFrames = ltr_use->frame;
      index = (OMX_INDEXTYPE)QOMX_IndexConfigVideoLTRUse;
      ret = omx_client_->SetConfig(index, &useltr_params);
      break;
    case CodecParamType::kDecodeOperatingRate:
      value = static_cast<uint32_t*>(codec_param);
      InitOMXParams(&operating_rate_params);
      operating_rate_params.nPortIndex = kPortIndexOutput;
      FractionToQ16(operating_rate_params.nU32,
          (int32_t)((*value)* 2), 2);
      ret = omx_client_->SetConfig((OMX_INDEXTYPE)OMX_IndexConfigOperatingRate,
          static_cast<OMX_PTR>(&operating_rate_params));
      break;
    case CodecParamType::kEnableFrameRepeat:
      break;
    default:
      QMMF_ERROR("%s:%s Unknown param type", TAG, __func__);
      return -1;
  }

  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s Failed to set codec param of type(%d)", TAG, __func__,
        param_type);
    return ret;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t AVCodec::GetParameters(const CodecParamType param_type,
                                void *codec_param, size_t *param_size) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  if (codec_param == nullptr || param_size == nullptr) {
    QMMF_ERROR("%s:%s Invalid Parameters", TAG, __func__);
    return -1;
  }

  switch (param_type) {
    case CodecParamType::kVQZipInfo: {
      VQZipInfo* vqzip_info = reinterpret_cast<VQZipInfo*>(codec_param);
      switch (vqzip_info->format) {
        case VideoFormat::kAVC: {
          OMX_VIDEO_PARAM_PROFILELEVELTYPE avc_profile_level;
          InitOMXParams(&avc_profile_level);
          ret = omx_client_->GetParameter(
              OMX_IndexParamVideoProfileLevelCurrent,
              reinterpret_cast<OMX_PTR>(&avc_profile_level));
          if (ret != 0) {
            QMMF_ERROR("%s:%s Failed to Get Profile and Level Values from OMX",
                       TAG, __func__);
            return ret;
          }

          ret = OmxtoQmmfProfile(
              VideoFormat::kAVC, avc_profile_level.eProfile,
              reinterpret_cast<void*>
                              (&(vqzip_info->avc_vqzip_info.profile)));
          if (ret != 0) {
            QMMF_ERROR("%s:%s Failed to Get Profile and Level Values from OMX",
                       TAG, __func__);
            return ret;
          }

          ret = OmxtoQmmfLevel(
              VideoFormat::kAVC, avc_profile_level.eLevel,
              reinterpret_cast<void*>(&(vqzip_info->avc_vqzip_info.level)));
          if (ret != 0) {
            QMMF_ERROR("%s:%s Failed to Get Profile and Level Values from OMX",
                       TAG, __func__);
            return ret;
          }

          QOMX_VIDEO_H264ENTROPYCODINGTYPE h264_cabac_info;
          InitOMXParams(&h264_cabac_info);
          ret = omx_client_->GetConfig(
              static_cast<OMX_INDEXTYPE>
                         (OMX_QcomIndexConfigH264EntropyCodingCabac),
              reinterpret_cast<OMX_PTR>(&h264_cabac_info));
          if (ret != 0) {
            QMMF_ERROR("%s:%s Failed to get the Cabac Info for H264 from OMX",
                       TAG, __func__);
            return ret;
          }

          vqzip_info->avc_vqzip_info.is_cabac_used = h264_cabac_info.bCabac;
          *param_size = sizeof(*vqzip_info);
          break;
        }

        default:
          QMMF_ERROR("%s:%s VQZip feature is Supported only for AVC Format videos",
                     TAG, __func__);
          return -1;
      }
      break;
    }

    default:
      QMMF_ERROR("%s:%s Unknown param type %d", TAG, __func__,
                 static_cast<underlying_type<CodecParamType>::type>
                            (param_type));
      return -1;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

bool AVCodec::IsInputPortStop() {

  Mutex::Autolock autoLock(input_stop_lock_);
  return input_stop_;
}

bool inline AVCodec::IsOutputPortStop() {

  Mutex::Autolock autoLock(output_stop_lock_);
  return output_stop_;
}

bool inline AVCodec::IsPortReconfig() {

  Mutex::Autolock autoLock(port_reconfig_lock_);
  return bPortReconfig_;
}

status_t AVCodec::Flush(uint32_t index) {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;

  ret = omx_client_->SendCommand(OMX_CommandFlush, index, 0);
  if(ret != 0) {
    QMMF_ERROR("%s:%s Failed to call flush command on %s", TAG, __func__,
        PORT_NAME(index));
    return ret;
  }

  CodecCmdType cmd;
  ret = signal_queue_.Pop(&cmd);
  if (ret != OK) {
    QMMF_ERROR("%s:%s Pop from SignalQueue Failed, size(%u)",
        TAG, __func__, signal_queue_.Size());
    return ret;
  }

  QMMF_INFO("%s:%s Popped buffer from cmd queue, size(%u)", TAG,
      __func__, signal_queue_.Size());

  if((cmd.event_result != OMX_ErrorNone) ||
     (cmd.event_type != OMX_EventCmdComplete) ||
     (cmd.event_cmd != OMX_CommandFlush)) {
    QMMF_ERROR("%s:%s Expecting Cmd complete for flush vs command found(%d)",
        TAG, __func__, cmd.event_cmd);
    return cmd.event_result;
  }

  /* Wait for flush complete for both ports */
  if (index == OMX_ALL) {
    ret = signal_queue_.Pop(&cmd);
    if (ret != OK) {
      QMMF_ERROR("%s:%s Pop from SignalQueue Failed, size(%u)",
          TAG, __func__, signal_queue_.Size());
      return ret;
    }

    QMMF_INFO("%s:%s Popped buffer from cmd queue, size(%u)", TAG,
        __func__, signal_queue_.Size());

    if((cmd.event_result != OMX_ErrorNone) ||
       (cmd.event_type != OMX_EventCmdComplete) ||
       (cmd.event_cmd != OMX_CommandFlush)) {
      QMMF_ERROR("%s:%s Expecting Cmd complete for flush vs command found(%d)",
        TAG, __func__, cmd.event_cmd);
      return cmd.event_result;
    }
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

void* AVCodec::DeliverInput(void *arg) {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;

  AVCodec *avcodec = static_cast<AVCodec*>(arg);
  BufferDescriptor stream_buffer;

  OMX_BUFFERHEADERTYPE *buf_header;
  bool thread_stop = false;

  while(1) {
    memset(&stream_buffer, 0x0, sizeof(stream_buffer));
    ret = avcodec->getInputBufferSource()->GetBuffer(stream_buffer, nullptr);
    buffer_handle_t native_handle;
    memset(&native_handle, 0x0, sizeof native_handle);
    if (avcodec->format_type_ == CodecType::kVideoEncoder) {
      native_handle = reinterpret_cast<buffer_handle_t>(stream_buffer.data);
      assert(native_handle != nullptr);
      assert(native_handle->data[0] != 0);
    }

    buf_header = avcodec->GetInputBufferHdr(stream_buffer);
    assert(buf_header != nullptr);

    if(ret != 0)  {
      QMMF_ERROR("%s:%s InputSource Read failed. Send EOS", TAG, __func__);
      buf_header->nFlags = OMX_BUFFERFLAG_EOS;
      thread_stop = true;
    }

    if(avcodec->IsInputPortStop()) {
      QMMF_INFO("%s:%s Encoder is stopped. Send EOS", TAG, __func__);
      buf_header->nFlags = OMX_BUFFERFLAG_EOS;
      thread_stop = true;
    }

    if (avcodec->format_type_ == CodecType::kVideoEncoder) {
      buf_header->nFilledLen = stream_buffer.size;
      buf_header->nTimeStamp = stream_buffer.timestamp / 1000;
    } else if(avcodec->format_type_ == CodecType::kAudioEncoder) {
      buf_header->nFilledLen = stream_buffer.size;
      buf_header->nTimeStamp  = stream_buffer.timestamp;
    } else {
      buf_header->nFilledLen = stream_buffer.size;
      buf_header->nTimeStamp = stream_buffer.timestamp;
    }

    if (avcodec->format_type_ == CodecType::kVideoEncoder)
      QMMF_VERBOSE("%s:%s ETB buffer fd(%d), ts(%lld)", TAG, __func__,
                   native_handle->data[0], stream_buffer.timestamp);
    else if(avcodec->format_type_ == CodecType::kAudioEncoder)
      QMMF_VERBOSE("%s:%s ETB buffer data(%p), fd(%d), ts(%lld)", TAG, __func__,
                   stream_buffer.data, stream_buffer.fd,
                   stream_buffer.timestamp);
    else
      QMMF_VERBOSE("%s:%s ETB buffer data(%p), fd(%d), ts(%lld) "
                   "filled_length(%d) frame_length(%d)",
                   TAG, __func__, stream_buffer.data, stream_buffer.fd,
                   stream_buffer.timestamp, stream_buffer.size,
                   stream_buffer.capacity);
    ret = avcodec->EmptyThisBuffer(buf_header);
    if(ret != 0) {
        QMMF_ERROR("%s:%s ETB failed for buffer(%p)", TAG, __func__,
                   buf_header->pBuffer);
        break;
    }

    if(thread_stop == true) {
      CodecPortStatus status = CodecPortStatus::kPortStop;
      avcodec->getInputBufferSource()->NotifyPortEvent(
          PortEventType::kPortStatus,static_cast<void*>(&status));
      break;
    }
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return nullptr;
}

void* AVCodec::ThreadRun(void *arg) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;
  AVCodec *avcodec = static_cast<AVCodec*>(arg);
  while(!avcodec->IsOutputPortStop()) {
    if(avcodec->IsPortReconfig()) {
      QMMF_INFO("%s:%s Port Reconfig Occured", TAG, __func__);
      ret = avcodec->PortReconfigOutput();
      if (ret != 0) {
        QMMF_ERROR("%s:%s Port Reconfig Failed", TAG, __func__);
        assert(0);
      }
      QMMF_INFO("%s:%s PortReconfig is Successfull", TAG, __func__);
      (avcodec->wait_for_header_output_).notify_one();
      (avcodec->wait_for_threadrun).signal();
    }
  }
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return nullptr;
}

void* AVCodec::DeliverOutput(void *arg) {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;

  BufferDescriptor codec_buffer;
  OMX_BUFFERHEADERTYPE *buf_header;
  AVCodec *avcodec = static_cast<AVCodec*>(arg);
  while(1) {
    memset(&codec_buffer, 0x0, sizeof(codec_buffer));
    ret = avcodec->getOutputBufferSource()->GetBuffer(codec_buffer, nullptr);

    assert(codec_buffer.data != nullptr);

    buf_header = avcodec->GetOutputBufferHdr(codec_buffer);
    assert(buf_header != nullptr);
    buf_header->nFlags = 0x0;

    if(avcodec->IsOutputPortStop()) {
      QMMF_INFO("%s:%s Encoder is stop. exit from thread", TAG, __func__);
      if (avcodec->format_type_ == CodecType::kVideoDecoder) {
        avcodec->UpdateBufferHeaderList(buf_header);
      }
      avcodec->getOutputBufferSource()->ReturnBuffer(codec_buffer, nullptr);
      break;
    }

    if (avcodec->format_type_ == CodecType::kVideoDecoder
          && avcodec->IsPortReconfig()) {
      avcodec->UpdateBufferHeaderList(buf_header);
      codec_buffer.size = 0;
      avcodec->getOutputBufferSource()->ReturnBuffer(codec_buffer, nullptr);
      Mutex::Autolock autoLock(avcodec->threadrun_port_reconfig_lock_);
      (avcodec->wait_for_threadrun).wait(avcodec->threadrun_port_reconfig_lock_);
      QMMF_INFO("%s:%s Signal from threadrun has been received", TAG, __func__);
      continue;
    }

    ret = avcodec->FillThisBuffer(buf_header);
    if(ret != 0) {
      QMMF_ERROR("%s:%s FTB failed for buffer(%p)", TAG, __func__,
          buf_header->pBuffer);
      break;
    }

    QMMF_VERBOSE("%s:%s FTB buf_header(%p) buffer(%p), fd(%d)", TAG, __func__,
        buf_header, codec_buffer.data, codec_buffer.fd);
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return nullptr;
}

OMX_BUFFERHEADERTYPE *AVCodec::GetInputBufferHdr(BufferDescriptor& buffer) {

  bool found = false;
  if (format_type_ == CodecType::kVideoEncoder) {
    bool timeout = false;
    std::unique_lock<std::mutex> queue_lock(queue_lock_);
    if (free_input_buffhdr_list_.Size() == 0) {
      QMMF_WARN("%s:%s: Wait for free header at input port!!", TAG, __func__);
      auto ret = wait_for_header_.wait_for(queue_lock,
          std::chrono::nanoseconds(kWaitDelay));
      if (ret == std::cv_status::timeout) {
        QMMF_ERROR("%s:%s: No free buffer header at input port!,"
          " Timed out happend!", TAG, __func__);
        timeout = true;
      }
    }
    assert(timeout == false);
    OMX_BUFFERHEADERTYPE* header = nullptr;

    header = *free_input_buffhdr_list_.Begin();
    encoder_media_buffer_type* media_buffer =
        (encoder_media_buffer_type*)header->pBuffer;
    media_buffer->meta_handle =
        reinterpret_cast<buffer_handle_t>(buffer.data);
    used_input_buffhdr_list_.PushBack(header);
    free_input_buffhdr_list_.Erase(free_input_buffhdr_list_.Begin());
    QMMF_VERBOSE("%s:%s free_input_buffhdr_list_.Size = %d", TAG, __func__,
        free_input_buffhdr_list_.Size());
    QMMF_VERBOSE("%s:%s used_input_buffhdr_list_.Size = %d", TAG, __func__,
        used_input_buffhdr_list_.Size());

    assert(header != nullptr);
    return header;
  } else {
    for (uint32_t i = 0; i < in_buff_hdr_size_; i++) {
      void* buf = static_cast<void*>(in_buff_hdr_[i]->pBuffer);
      if(buf == nullptr) {
        QMMF_INFO("%s:%s Register Input buffer(%p), fd(%d) in buffer list(%p)",
                  TAG, __func__, buffer.data, buffer.fd, in_buff_hdr_[i]);
        in_buff_hdr_[i]->pBuffer = static_cast<OMX_U8*>(buffer.data);
        in_buff_hdr_[i]->pAppPrivate = reinterpret_cast<OMX_PTR>(buffer.fd);
        return in_buff_hdr_[i];
      }
      if(buf == buffer.data) {
        return in_buff_hdr_[i];
      }
    }
    QMMF_ERROR("%s:%s No Input Buffer header found for (%p)", TAG, __func__,
              buffer.data);
  }
  assert(found == true);
  return nullptr;
}

OMX_BUFFERHEADERTYPE *AVCodec::GetOutputBufferHdr(BufferDescriptor& buffer) {

  bool found = false;
  if (format_type_ == CodecType::kVideoEncoder) {
    for (uint32_t i = 0; i < out_buff_hdr_size_; i++) {
      void* buf = static_cast<void*>(out_buff_hdr_[i]->pBuffer);
      if(buf == buffer.data) {
        return out_buff_hdr_[i];
      }
    }
  } else if (format_type_ == CodecType::kVideoDecoder) {
    bool timeout = false;
    if (free_output_buffhdr_list_.Size() == 0) {
      QMMF_WARN("%s:%s: Wait for free header at output port!!", TAG, __func__);
      std::unique_lock<std::mutex> lock(lock_output_);
      auto ret = wait_for_header_output_.wait_for(lock,
          std::chrono::nanoseconds(kWaitDelay));
      if (ret == std::cv_status::timeout) {
        QMMF_ERROR("%s:%s: No free buffer header at output port!,"
          " Timed out happend!", TAG, __func__);
        timeout = true;
      }
    }
    assert(timeout == false);
    OMX_BUFFERHEADERTYPE* header = nullptr;
    std::lock_guard<std::mutex> lock(queue_lock_output_);
    {
      header = *free_output_buffhdr_list_.Begin();
      struct VideoDecoderOutputMetaData* pParam =
          reinterpret_cast<struct VideoDecoderOutputMetaData*>(header->pBuffer);
      private_handle_t *handle = const_cast<private_handle_t*>
          (static_cast<const private_handle_t*>(pParam->pHandle));
      handle->fd = buffer.fd;
      handle->size = buffer.capacity;
      used_output_buffhdr_list_.PushBack(header);
      free_output_buffhdr_list_.Erase(free_output_buffhdr_list_.Begin());
      QMMF_VERBOSE("%s:%s free_output_buffhdr_list_.Size = %d", TAG, __func__,
          free_output_buffhdr_list_.Size());
      QMMF_VERBOSE("%s:%s used_output_buffhdr_list_.Size = %d", TAG, __func__,
          used_output_buffhdr_list_.Size());
    }
    assert(header != nullptr);
    return header;
  } else if(format_type_ == CodecType::kAudioEncoder) {
    for (uint32_t i = 0; i < out_buff_hdr_size_; i++) {
      BufferDescriptor* buf = reinterpret_cast<BufferDescriptor*>
                                         (out_buff_hdr_[i]->pAppPrivate);
      if(buf->data == nullptr) {
        QMMF_INFO("%s:%s Register buffer(%p), fd(%d) in buffer list(%p)", TAG,
            __func__, buffer.data, buffer.fd, out_buff_hdr_[i]);
        buf->data = buffer.data;
        buf->fd = buffer.fd;
        return out_buff_hdr_[i];
      }
      if(buf->data == buffer.data) {
        return out_buff_hdr_[i];
      }
    }
  } else {
    for (uint32_t i = 0; i < out_buff_hdr_size_; i++) {
      void* buf = static_cast<void*>(out_buff_hdr_[i]->pBuffer);
      if(buf == nullptr) {
        QMMF_INFO("%s:%s Register Output buffer(%p), fd(%d) in buffer list(%p)",
            TAG, __func__, buffer.data, buffer.fd, out_buff_hdr_[i]);
        out_buff_hdr_[i]->pBuffer = static_cast<OMX_U8 *>(buffer.data);
        out_buff_hdr_[i]->pAppPrivate = reinterpret_cast<OMX_PTR>(buffer.fd);
        return out_buff_hdr_[i];
      }
      if(buf == buffer.data) {
        return out_buff_hdr_[i];
      }
    }
  }
  QMMF_ERROR("%s:%s No Output Buffer header found for (%p)", TAG, __func__
      , buffer.data);
  assert(found == true);

  return nullptr;
}

status_t AVCodec::PushEventCommand(OMX_EVENTTYPE event, OMX_COMMANDTYPE command,
                                   OMX_U32 data, OMX_U32 flag) {

  status_t ret = 0;

  CodecCmdType cmd_buffer_val;

  cmd_buffer_val.event_type = event;
  cmd_buffer_val.event_cmd = command;
  cmd_buffer_val.event_data= data;
  cmd_buffer_val.event_flags = flag;
  cmd_buffer_val.event_result = OMX_ErrorNone;

  QMMF_INFO("%s:%s Pushing cmd buffer", TAG, __func__);
  ret = signal_queue_.Push(cmd_buffer_val);
  if (ret != OK) {
    QMMF_ERROR("%s:%s Failed to push cmd buffer, size(%u)", TAG, __func__,
        signal_queue_.Size());
    return ret;
  }

  return ret;
}

status_t AVCodec::EmptyThisBuffer(OMX_BUFFERHEADERTYPE *buffer) {

  return omx_client_->EmptyThisBuffer(buffer);
}

status_t AVCodec::FillThisBuffer(OMX_BUFFERHEADERTYPE *buffer) {

  return omx_client_->FillThisBuffer(buffer);
}

status_t AVCodec::SetState(OMX_STATETYPE state, OMX_BOOL synchronous) {

  status_t ret = OK;

  QMMF_INFO("%s:%s current state(%s), pending state(%s)", TAG, __func__,
      OMX_STATE_NAME(state_), OMX_STATE_NAME(state_pending_));

  if (state == state_) {
    QMMF_WARN("%s:%s Current state is already %s", TAG, __func__,
        OMX_STATE_NAME(state));
    return ret;
  }

  // check for pending state transition
  if(state_ != state_pending_) {
    ret = WaitState(state_pending_);
    if(ret != OK) {
      QMMF_ERROR("%s:%s Wait for %s failed", TAG, __func__,
          OMX_STATE_NAME(state_pending_));
      return ret;
    }
  }

  // check for invalid transition
  if(((state == OMX_StateLoaded) && (state_ != OMX_StateIdle)) ||
      ((state == OMX_StateExecuting) && (state_ != OMX_StateIdle))) {
    QMMF_ERROR("%s:%s Invalid state tranisition: state %s to %s", TAG, __func__,
        OMX_STATE_NAME(state), OMX_STATE_NAME(state_));
    return OMX_ErrorIncorrectStateTransition;
  }

  QMMF_INFO("%s:%s Moving to state(%s) from(%s)", TAG, __func__,
      OMX_STATE_NAME(state), OMX_STATE_NAME(state_));
  ret = omx_client_->SendCommand(OMX_CommandStateSet, state, 0);

  if (ret != OK) {
    QMMF_ERROR("%s:%s Failed to set state(%s)", TAG, __func__,
        OMX_STATE_NAME(state));
    return ret;
  }

  state_pending_ = state;

  if (synchronous == OMX_TRUE) {
    return WaitState(state);
  }

  QMMF_INFO("%s:%s current state(%s), pending state(%s)", TAG, __func__,
      OMX_STATE_NAME(state_), OMX_STATE_NAME(state_pending_));
  return ret;
}

status_t AVCodec::WaitState(OMX_STATETYPE state) {

  status_t ret = OK;

  if(state_ == state) {
    QMMF_INFO("%s:%s State is already in %s", TAG, __func__,
        OMX_STATE_NAME(state));
    return ret;
  }

  CodecCmdType cmd;
  ret = signal_queue_.Pop(&cmd);
  if (ret != OK) {
    QMMF_ERROR("%s:%s Pop from SignalQueue Failed, size(%u)",
        TAG, __func__, signal_queue_.Size());
    return ret;
  }

  QMMF_INFO("%s:%s Popped buffer from cmd queue, size(%u)", TAG,
      __func__, signal_queue_.Size());

  ret = cmd.event_result;

  if((cmd.event_type != OMX_EventCmdComplete) ||
      (cmd.event_cmd != OMX_CommandStateSet)) {
     QMMF_ERROR("%s:%s Expecting state change", TAG, __func__);
    return ret;
  }

  if((OMX_STATETYPE)cmd.event_data != state) {
    QMMF_ERROR("%s:%s Wrong state found(%s)", TAG, __func__,
        OMX_STATE_NAME((OMX_STATETYPE)cmd.event_data));
    return OMX_ErrorUndefined;
  }

  state_ = (OMX_STATETYPE)cmd.event_data;
  QMMF_INFO("%s:%s Reached state(%s)", TAG, __func__, OMX_STATE_NAME(state));

  return ret;
}

status_t AVCodec::PortReconfigOutput() {

    status_t ret = 0;
    PortreconfigData reconfig_data;
    memset(&reconfig_data, 0x0, sizeof(reconfig_data));
    QMMF_INFO("%s:%s PortReconfig Calling Flush on output port", TAG, __func__);
    ret = Flush(kPortIndexOutput);
    if (ret != OK) {
      QMMF_ERROR("%s:%s Flush Output Port failed", TAG, __func__);
      return ret;
    }

    QMMF_INFO("%s:%s PortReconfig OMX_CommandPortDisable", TAG, __func__);
    ret = omx_client_->SendCommand(OMX_CommandPortDisable, kPortIndexOutput, 0);
    if (ret != OK) {
      QMMF_ERROR("%s:%s Disable Output Port failed", TAG, __func__);
      return ret;
    }

    // Wait for OMX_comp/sink to return all buffers
    int32_t list_size;
    while ((list_size = used_output_buffhdr_list_.Size()) != 0) {
      QMMF_INFO("%s:%s used_output_buffhdr_list_.size = %d", TAG, __func__,
          list_size);
    }

    QMMF_INFO("%s:%s All FillBufferDone Recieved", TAG, __func__);

    // Free all old buffers
    QMMF_INFO("%s:%s Free OUTPUT buffers", TAG, __func__);
    for (uint32_t i = 0; i < out_buff_hdr_size_; i++) {
      ret = omx_client_->FreeBuffer(out_buff_hdr_[i], kPortIndexOutput);
      if(ret != 0) {
        QMMF_ERROR("%s:%s Failed to free buffer on %s", TAG, __func__,
            PORT_NAME(kPortIndexOutput));
        return ret;
      }
    }

    // wait for OMX_comp to respond OMX_CommandPortDisable
    // this only happens once all buffers are freed
    CodecCmdType cmd;
    ret = signal_queue_.Pop(&cmd);
    if (ret != OK) {
      QMMF_ERROR("%s:%s Pop from SignalQueue Failed, size(%u)",
          TAG, __func__, signal_queue_.Size());
      return ret;
    }

    QMMF_INFO("%s:%s Popped buffer from cmd queue, size(%u)", TAG,
        __func__, signal_queue_.Size());

    if((cmd.event_result != OMX_ErrorNone) ||
        (cmd.event_type != OMX_EventCmdComplete) ||
        (cmd.event_cmd != OMX_CommandPortDisable)) {
      QMMF_ERROR("%s:%s Expecting Cmd complete vs command found(%d)", TAG,
          __func__, cmd.event_cmd);
      assert(0);
      return cmd.event_result;
    }

    // ask OMX_comp for new settings
    QMMF_INFO("%s:%s PortReconfig get new settings", TAG, __func__);
    OMX_PARAM_PORTDEFINITIONTYPE output_port;
    InitOMXParams(&output_port);
    output_port.nPortIndex = kPortIndexOutput;
    ret = omx_client_->GetParameter((OMX_INDEXTYPE)OMX_IndexParamPortDefinition,
              &output_port);
    if (output_port.eDir != OMX_DirOutput) {
        QMMF_ERROR("%s:%s Error - Expected Output Port\n", TAG, __func__);
        assert(0);
        return OMX_ErrorUndefined;
    }

    out_buff_hdr_size_ = output_port.nBufferCountActual;

    reconfig_data.reconfig_type =
        PortreconfigData::PortReconfigType::kBufferRequirementsChanged;
    reconfig_data.rect.left = 0;
    reconfig_data.rect.top = 0;
    reconfig_data.rect.width = output_port.format.video.nFrameWidth;
    reconfig_data.rect.height = output_port.format.video.nFrameHeight;
    reconfig_data.buf_reqs.buf_count = out_buff_hdr_size_;
    reconfig_data.buf_reqs.buf_size = output_port.nBufferSize;

    QMMF_INFO("%s:%s PortReconfig Min Buffer Count = %u", TAG, __func__,
        (uint32_t)out_buff_hdr_size_);
    QMMF_INFO("%s:%s PortReconfig Buffer Size = %u", TAG, __func__,
        (uint32_t)output_port.nBufferSize);
    QMMF_INFO("%s:%s PortReconfig width : %u, height : %u", TAG, __func__,
        (uint32_t)reconfig_data.rect.width,
        (uint32_t)reconfig_data.rect.height);

    //Free the phandles of Vector<struct VideoDecoderOutputMetaData> outputpParam_dec_
    for (auto& iter: outputpParam_dec_) {
      delete iter.pHandle;
      iter.pHandle = nullptr;
    }

    delete []out_buff_hdr_;
    out_buff_hdr_ = nullptr;

    out_buff_hdr_ = new OMX_BUFFERHEADERTYPE*[out_buff_hdr_size_];
    if(out_buff_hdr_ ==  nullptr) {
        QMMF_ERROR("%s:%s Failed to allocate buffer header on %s",
                   TAG, __func__, PORT_NAME(kPortIndexOutput));
        assert(0);
        return NO_MEMORY;
    }

    if (!free_output_buffhdr_list_.Empty())
      free_output_buffhdr_list_.Clear();

    if (!used_output_buffhdr_list_.Empty())
      used_output_buffhdr_list_.Clear();

    // notify sink that PortReconfig event has occured
    QMMF_INFO("%s:%s PortReconfig Informing Sink", TAG, __func__);

    ret = getOutputBufferSource()->NotifyPortEvent(
              PortEventType::kPortSettingsChanged,
              static_cast<void*>(&reconfig_data));
    if (ret != 0) {
        QMMF_ERROR("%s:%s Informing Sink Failed", TAG, __func__);
        return ret;
    }

    QMMF_INFO("%s:%s PortReconfig re-enabling port", TAG, __func__);
    ret = omx_client_->SendCommand(OMX_CommandPortEnable, kPortIndexOutput, 0);
    if(ret != 0) {
        QMMF_ERROR("%s:%s Failed to enable port on %s", TAG, __func__,
            PORT_NAME(kPortIndexInput));
        assert(0);
        return ret;
    }

    // re-allocate all buffers on the port using use-buffer mode
    for (uint32_t i = 0; i < out_buff_hdr_size_; ++i) {
      uint32_t buf_size = sizeof(struct VideoDecoderOutputMetaData);
      ret = omx_client_->UseBuffer(&out_buff_hdr_[i], kPortIndexOutput, nullptr,
                              buf_size,
                              reinterpret_cast<OMX_U8*>(&outputpParam_dec_[i]));
      if(ret != OK) {
          QMMF_ERROR("%s:%s Failed to allocate buffer on %s", TAG, __func__,
              PORT_NAME(kPortIndexOutput));
          return ret;
      }

      struct VideoDecoderOutputMetaData* pParam =
          (struct VideoDecoderOutputMetaData*)out_buff_hdr_[i]->pBuffer;
      assert(pParam != nullptr);
      free_output_buffhdr_list_.PushBack(out_buff_hdr_[i]);
    }

    // wait for OMX_comp to respond OMX_CommandPortEnabled
    // this only happens once all buffers are allocated
    ret = signal_queue_.Pop(&cmd);
    if (ret != OK) {
      QMMF_ERROR("%s:%s Pop from SignalQueue Failed, size(%u)",
          TAG, __func__, signal_queue_.Size());
      return ret;
    }

    QMMF_INFO("%s:%s Popped buffer from cmd queue, size(%u)", TAG,
        __func__, signal_queue_.Size());

    if((cmd.event_result != OMX_ErrorNone) ||
        (cmd.event_type != OMX_EventCmdComplete) ||
        (cmd.event_cmd != OMX_CommandPortEnable)) {
      QMMF_ERROR("%s:%s Expecting Cmd complete vs command found(%d)", TAG,
          __func__, cmd.event_cmd);
      assert(0);
      return cmd.event_result;
    }

    {
      Mutex::Autolock autoLock(port_reconfig_lock_);
      bPortReconfig_ = false;
    }

    QMMF_INFO("%s:%s port-reconfig done", TAG, __func__);
    return ret;
}

status_t AVCodec::HandleOutputPortSettingsChange(OMX_U32 data2) {

    status_t ret = 0;
    OMX_CONFIG_RECTTYPE rect;
    InitOMXParams(&rect);
    PortreconfigData reconfig_data;
    memset(&reconfig_data, 0x0, sizeof(reconfig_data));
    if (data2 == OMX_IndexConfigCommonOutputCrop
        || data2 == OMX_IndexConfigCommonScale) {
      rect.nPortIndex = kPortIndexOutput;
      ret = omx_client_->GetConfig(
                (OMX_INDEXTYPE)OMX_IndexConfigCommonOutputCrop,
                static_cast<OMX_PTR>(&rect));
      if (ret != OK) {
        QMMF_ERROR("%s:%s Failed to get crop rectangle", TAG, __func__);
        return ret;
      }

      QMMF_INFO("%s:%s Got Crop Rect: (%d, %d) (%u x %u)", TAG, __func__,
          (int)rect.nLeft, (int)rect.nTop, (uint32_t)rect.nWidth,
          (uint32_t)rect.nHeight);
      reconfig_data.reconfig_type =
          PortreconfigData::PortReconfigType::kCropParametersChanged;
      reconfig_data.rect.left = rect.nLeft;
      reconfig_data.rect.top = rect.nTop;
      reconfig_data.rect.width = rect.nWidth;
      reconfig_data.rect.height = rect.nHeight;

      //A callback to AVcodec Client to notify
      //that the Port Settings has changed
      ret = getOutputBufferSource()->NotifyPortEvent(
                PortEventType::kPortSettingsChanged,
                static_cast<void*>(&reconfig_data));
      if (ret != 0) {
        QMMF_ERROR("%s:%s Failed to Set CropParameters", TAG, __func__);
        return ret;
      }
    } else if (data2 == 0 || data2 == OMX_IndexParamPortDefinition) {
      QMMF_INFO("%s:%s Reconfiguring output port", TAG, __func__);
      {
        Mutex::Autolock autoLock(port_reconfig_lock_);
        bPortReconfig_ = true;
      }
    }
    return ret;
}



OMX_ERRORTYPE AVCodec::OnEvent(
                    OMX_IN OMX_HANDLETYPE component __attribute__((__unused__)),
                    OMX_IN OMX_PTR app_data,
                    OMX_IN OMX_EVENTTYPE event,
                    OMX_IN OMX_U32 data1,
                    OMX_IN OMX_U32 data2,
                    OMX_IN OMX_PTR event_data __attribute__((__unused__))) {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  AVCodec *avcodec = (AVCodec *)app_data;
  assert(avcodec != nullptr);

  if (event == OMX_EventCmdComplete) {
    if ((OMX_COMMANDTYPE)data1 == OMX_CommandStateSet) {
      QMMF_INFO("%s:%s Event callback: state is %s", TAG, __func__,
          OMX_STATE_NAME((OMX_STATETYPE)data2));
      avcodec->PushEventCommand(event, OMX_CommandStateSet, data2, 0x0);

    } else if ((OMX_COMMANDTYPE)data1 == OMX_CommandFlush) {
      QMMF_INFO("%s:%s Event callback: flush complete on port : %s", TAG,
          __func__, PORT_NAME(data2));
      avcodec->PushEventCommand(event, OMX_CommandFlush, data2, 0x0);

    } else if ((OMX_COMMANDTYPE)data1 == OMX_CommandPortDisable) {
      QMMF_INFO("%s:%s Event callback: %s port disable", TAG, __func__,
              PORT_NAME(data2));
      avcodec->PushEventCommand(event, OMX_CommandPortDisable, data2, 0x0);

    } else if ((OMX_COMMANDTYPE)data1 == OMX_CommandPortEnable) {
      QMMF_INFO("%s:%s Event callback: %s port enable", TAG, __func__,
                PORT_NAME(data2));
      avcodec->PushEventCommand(event, OMX_CommandPortEnable, data2, 0x0);

    } else {
      QMMF_WARN("%s:%s Unimplemented command", TAG, __func__);
    }
  } else if (event == OMX_EventError) {
    assert(0);

  } else if (event == OMX_EventBufferFlag) {
    QMMF_INFO("%s:%s Event callback: Buffer flag received", TAG, __func__);

  } else if (event ==  OMX_EventPortSettingsChanged) {
    QMMF_INFO("%s:%s Event callback: Port Settings Changed %s", TAG, __func__,
        PORT_NAME(data1));
    if (data1 == kPortIndexOutput) {
      auto ret = avcodec->HandleOutputPortSettingsChange(data2);
      if (ret != 0) {
        QMMF_ERROR("%s:%s HandleOutputPortSettingsChange Failed", TAG, __func__);
        assert(0);
      }
    } else {
      QMMF_ERROR("%s:%s Reconfig not supported on %s", TAG, __func__,
        PORT_NAME(data1));
      assert(0);
    }
  } else {
    QMMF_WARN("%s:%s Unimplemented event", TAG, __func__);
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return OMX_ErrorNone;
}

OMX_ERRORTYPE AVCodec::OnEmptyBufferDone(
                      OMX_IN OMX_HANDLETYPE handle __attribute__((__unused__)),
                      OMX_IN OMX_PTR app_data,
                      OMX_IN OMX_BUFFERHEADERTYPE* buf_header) {
  QMMF_DEBUG("%s:%s Enter", TAG, __func__);
  QMMF_VERBOSE("%s:%s buf_header[%p]", TAG, __func__, buf_header);
  QMMF_VERBOSE("%s:%s buf_header->pBuffer[%p]", TAG, __func__,
               buf_header->pBuffer);
  QMMF_VERBOSE("%s:%s buf_header->pAppPrivate[%p]", TAG, __func__,
               buf_header->pAppPrivate);

  //TODO: use pBuffer
  AVCodec *avcodec = (AVCodec *)app_data;
  BufferDescriptor stream_buffer;
  memset(&stream_buffer, 0x0, sizeof stream_buffer);
  if (avcodec->format_type_ == CodecType::kVideoEncoder) {
    encoder_media_buffer_type* mediaBuffer =
        (encoder_media_buffer_type*)buf_header->pBuffer;
    assert(mediaBuffer->meta_handle != nullptr);

    stream_buffer.data =
        const_cast<void*>(reinterpret_cast<const void*>
        (mediaBuffer->meta_handle));
    avcodec->UpdateBufferHeaderList(buf_header);

    QMMF_DEBUG("%s:%s EBD fd(%d), ts(%lld)", TAG, __func__,
        mediaBuffer->meta_handle->data[0], buf_header->nTimeStamp);
  } else if(avcodec->format_type_ == CodecType::kAudioEncoder) {
    assert(buf_header->pBuffer != nullptr);
    stream_buffer.data = buf_header->pBuffer;
    stream_buffer.fd = reinterpret_cast<int32_t>(buf_header->pAppPrivate);
    QMMF_DEBUG("%s:%s EBD buffer[%s]", TAG, __func__,
               stream_buffer.ToString().c_str());
  } else {
    assert(buf_header->pBuffer != nullptr);
    stream_buffer.data = buf_header->pBuffer;
    stream_buffer.fd = reinterpret_cast<int32_t>(buf_header->pAppPrivate);
    QMMF_DEBUG("%s:%s EBD data(%p), ts(%lld)", TAG, __func__,
        stream_buffer.data, buf_header->nTimeStamp);
  }

  avcodec->getInputBufferSource()->ReturnBuffer(stream_buffer, nullptr);
  if(buf_header->nFlags & OMX_BUFFERFLAG_EOS) {
    QMMF_INFO("%s:%s No more buffer to process on input port", TAG, __func__);
    CodecPortStatus status = CodecPortStatus::kPortIdle;
    avcodec->getInputBufferSource()->NotifyPortEvent(PortEventType::kPortStatus,
        static_cast<void*>(&status));
  }

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
  return OMX_ErrorNone;
}

OMX_ERRORTYPE AVCodec::OnFillBufferDone(
                      OMX_IN OMX_HANDLETYPE handle __attribute__((__unused__)),
                      OMX_IN OMX_PTR app_data,
                      OMX_IN OMX_BUFFERHEADERTYPE* buf_header) {
  QMMF_DEBUG("%s:%s Enter", TAG, __func__);
  QMMF_VERBOSE("%s:%s buf_header[%p]", TAG, __func__, buf_header);
  QMMF_VERBOSE("%s:%s buf_header->pBuffer[%p]", TAG, __func__,
               buf_header->pBuffer);
  QMMF_VERBOSE("%s:%s buf_header->pAppPrivate[%p]", TAG, __func__,
               buf_header->pAppPrivate);

  AVCodec *avcodec = (AVCodec *)app_data;
  assert(buf_header->pBuffer);
  BufferDescriptor codec_buffer;
  memset(&codec_buffer, 0x0, sizeof codec_buffer);

  codec_buffer.data = buf_header->pBuffer;
  codec_buffer.size = buf_header->nFilledLen;
  codec_buffer.timestamp = buf_header->nTimeStamp;
  codec_buffer.offset = buf_header->nOffset;

  if (buf_header->nFlags & QOMX_VIDEO_PictureTypeIDR)
    codec_buffer.flag |= static_cast<uint32_t>(BufferFlags::kFlagIDRFrame);

  if (buf_header->nFlags & OMX_BUFFERFLAG_SYNCFRAME)
    codec_buffer.flag |= static_cast<uint32_t>(BufferFlags::kFlagIFrame);

  if (buf_header->nFlags & OMX_VIDEO_PictureTypeP)
    codec_buffer.flag |= static_cast<uint32_t>(BufferFlags::kFlagPFrame);

  if (buf_header->nFlags & OMX_VIDEO_PictureTypeB)
    codec_buffer.flag |= static_cast<uint32_t>(BufferFlags::kFlagBFrame);

  if (buf_header->nFlags & OMX_BUFFERFLAG_CODECCONFIG)
    codec_buffer.flag |= static_cast<uint32_t>(BufferFlags::kFlagCodecConfig);

  if (buf_header->nFlags & OMX_BUFFERFLAG_EOS)
    codec_buffer.flag |= static_cast<uint32_t>(BufferFlags::kFlagEOS);

  if (buf_header->nFlags & OMX_BUFFERFLAG_EXTRADATA)
    codec_buffer.flag |= static_cast<uint32_t>(BufferFlags::kFlagExtraData);

  if (buf_header->nFlags & OMX_BUFFERFLAG_ENDOFFRAME)
    codec_buffer.flag |= static_cast<uint32_t>(BufferFlags::kFlagEOF);

  QMMF_DEBUG("%s:%s codec_buffer.flag(%u)", TAG, __func__, codec_buffer.flag);

  if(avcodec->format_type_ == CodecType::kVideoDecoder) {
    struct VideoDecoderOutputMetaData *pParam =
        reinterpret_cast<struct VideoDecoderOutputMetaData*>(codec_buffer.data);
    assert(pParam->pHandle != nullptr);
    private_handle_t *handle = const_cast<private_handle_t*>
        (static_cast<const private_handle_t*>(pParam->pHandle));
    codec_buffer.fd = handle->fd;
    codec_buffer.capacity = handle->size;
    avcodec->UpdateBufferHeaderList(buf_header);
    QMMF_DEBUG("%s:%s VideoDecoder fd(%d)", TAG, __func__, codec_buffer.fd);
    if (avcodec->IsPortReconfig()) {
      return OMX_ErrorNone;
    }
  }

  if (avcodec->format_type_ == CodecType::kAudioEncoder) {
    BufferDescriptor* buf =
        reinterpret_cast<BufferDescriptor*>(buf_header->pAppPrivate);
    codec_buffer.data = buf->data;
    codec_buffer.fd = buf->fd;
    codec_buffer.size = 0;
    codec_buffer.capacity = buf_header->nAllocLen;
    memset(codec_buffer.data, 0x0, sizeof codec_buffer.capacity);

    uint8_t* src = reinterpret_cast<uint8_t*>(buf_header->pBuffer);
    unsigned int num_of_frames = src[0];
    QMMF_VERBOSE("%s:%s number of audio frames[%u]", TAG, __func__,
                 num_of_frames);
    ++src;

    const uint8_t* source_ptr = nullptr;
    uint8_t* dest_ptr = nullptr;
    size_t length;

    if (!((codec_buffer.flag) & static_cast<uint32_t>(BufferFlags::kFlagEOS))) {
      if ((codec_buffer.flag) &
          static_cast<uint32_t>(BufferFlags::kFlagCodecConfig)) {
        QMMF_DEBUG("%s:%s codec config frame size[%u]", TAG, __func__,
                   buf_header->nFilledLen);
        source_ptr = reinterpret_cast<const uint8_t*>(buf_header->pBuffer);
        dest_ptr =
            reinterpret_cast<uint8_t*>(codec_buffer.data) + codec_buffer.size;
        length = buf_header->nFilledLen;
        memcpy(dest_ptr, source_ptr, length);
        codec_buffer.size += length;
        codec_buffer.timestamp = 0;
      } else if (num_of_frames > 0) {
        AudioEncoderMetadata* meta =
            reinterpret_cast<AudioEncoderMetadata*>(src);
        QMMF_VERBOSE("%s:%s audio metadata[%s]", TAG, __func__,
                     meta->ToString().c_str());
        length = meta->frame_size;
        source_ptr = reinterpret_cast<const uint8_t*>(buf_header->pBuffer) + 1 +
                     meta->offset_to_frame;
        dest_ptr =
            reinterpret_cast<uint8_t*>(codec_buffer.data) + codec_buffer.size;
        memcpy(dest_ptr, source_ptr, length);
        codec_buffer.size += length;
        codec_buffer.timestamp =
            ((uint64_t)(meta->msw_ts) << 32) | (uint64_t)(meta->lsw_ts);
        src += sizeof(meta);
        --num_of_frames;

        if (num_of_frames > 0)
          QMMF_WARN("%s:%s multiple audio frames were found in one buffer", TAG,
                    __func__);
      }
    }
  }

  if(avcodec->format_type_ == CodecType::kAudioDecoder) {
    codec_buffer.fd = reinterpret_cast<int32_t>(buf_header->pAppPrivate);
    codec_buffer.capacity = buf_header->nAllocLen;
  }

  if ((codec_buffer.flag) & static_cast<uint32_t>(BufferFlags::kFlagEOS)) {
    QMMF_INFO("%s:%s received output port EOS", TAG, __func__);
    avcodec->StopOutput();
    if(!(avcodec->isEOSonOutput_)) {
      avcodec->PushEventCommand((OMX_EVENTTYPE)0, (OMX_COMMANDTYPE)0, 0,
          OMX_BUFFERFLAG_EOS);
      avcodec->isEOSonOutput_ = true;
    }
  }

  if(avcodec->format_type_ == CodecType::kAudioDecoder) {
    QMMF_DEBUG("%s:%s FBD buffer(%p), filled length(%d), ts(%lld)  offset(%d)"
        "  flag(0x%x)", TAG, __func__, codec_buffer.data, codec_buffer.size,
        codec_buffer.timestamp, codec_buffer.offset,
        (unsigned int)buf_header->nFlags);
  } else {
    QMMF_DEBUG("%s:%s FBD buffer[%s]", TAG, __func__,
               codec_buffer.ToString().c_str());
  }

  avcodec->getOutputBufferSource()->ReturnBuffer(codec_buffer, nullptr);
  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
  return OMX_ErrorNone;
}

void AVCodec::UpdateBufferHeaderList(OMX_BUFFERHEADERTYPE* buf_header) {
  QMMF_DEBUG("%s:%s Enter", TAG, __func__);

  bool found = false;
  if (format_type_ == CodecType::kVideoEncoder) {
    std::lock_guard<std::mutex> lock(queue_lock_);
    List<OMX_BUFFERHEADERTYPE*>::iterator it = used_input_buffhdr_list_.Begin();
    for (; it != used_input_buffhdr_list_.End(); ++it) {
      if ((*it) == buf_header) {
        QMMF_VERBOSE("%s:%s Found the header!", TAG, __func__);
        found = true;
        break;
      }
    }
    if (found) {
      free_input_buffhdr_list_.PushBack(*it);
      used_input_buffhdr_list_.Erase(it);
      wait_for_header_.notify_one();
    }
  } else if (format_type_ == CodecType::kVideoDecoder) {
    std::lock_guard<std::mutex> lock(queue_lock_output_);
    List<OMX_BUFFERHEADERTYPE*>::iterator it = used_output_buffhdr_list_.Begin();
    for (; it != used_output_buffhdr_list_.End(); ++it) {
      if ((*it) == buf_header) {
        QMMF_VERBOSE("%s:%s Found the header!", TAG, __func__);
        found = true;
        break;
      }
    }
    if (found) {
      free_output_buffhdr_list_.PushBack(*it);
      used_output_buffhdr_list_.Erase(it);
      wait_for_header_output_.notify_one();
    }
  }
  assert(found == true);
}

}; // namespace avcodec
}; // namespace qmmf
