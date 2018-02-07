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

#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>

#include <linux/msm_ion.h>
#include <OMX_QCOMExtns.h>
#include <utils/Mutex.h>
#include <utils/RefBase.h>
#include <utils/Vector.h>
#include <utils/Condition.h>

#include <libstagefrighthw/QComOMXMetadata.h>
#include <media/hardware/HardwareAPI.h>

#include "common/qmmf_common_utils.h"
#include "qmmf-sdk/qmmf_avcodec_params.h"
#include "qmmf-sdk/qmmf_avcodec.h"
#include "qmmf_avcodec_common.h"

namespace qmmf {
namespace avcodec {

struct CodecBuffer {
  void*     pointer;
  int32_t   fd;
  size_t    frame_length;
  size_t    filled_length;
  uint64_t  ts;
  int32_t   flag;
  struct    ion_handle_data handle_data;
  uint32_t  offset_to_frame;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "pointer[" << pointer << "] ";
    stream << "fd[" << fd << "] ";
    stream << "frame_length[" << frame_length << "] ";
    stream << "filled_length[" << filled_length << "] ";
    stream << "ts[" << ts << "] ";
    stream << "flag[" << ::std::setbase(16) << flag << ::std::setbase(10)
           << "] ";
    return stream.str();
  }
};

class OmxClient;

class AVCodec : public IAVCodec {
 public:
  AVCodec();
  ~AVCodec();

  // methods of IAVCodec
  status_t GetComponentName(CodecMimeType mime_type, uint32_t *num_comps,
      ::std::vector<::std::string>& comp_names) override;
  status_t ConfigureCodec(CodecMimeType codec_type, CodecParam& codec_param,
                          ::std::string comp_name = "") override;
  status_t GetBufferRequirements(uint32_t port_type, uint32_t* buf_count,
                                 uint32_t* buf_size) override;
  status_t AllocateBuffer(uint32_t port_type, uint32_t buf_count,
      uint32_t buf_size, const ::std::shared_ptr<ICodecSource>& source,
      ::std::vector<BufferDescriptor>& buffer_list) override;
  status_t ReleaseBuffer() override;
  status_t SetParameters(CodecParamType param_type, void *codec_param,
                         size_t param_size) override;
  status_t GetParameters(const CodecParamType param_type, void *codec_param,
                         size_t *param_size) override;
  status_t StartCodec() override;
  status_t StopCodec() override;
  status_t PauseCodec() override;
  status_t ResumeCodec() override;
  status_t RegisterOutputBuffers(
      ::std::vector<BufferDescriptor>& list) override;
  status_t RegisterInputBuffers(
      ::std::vector<BufferDescriptor>& list) override;
  status_t Flush(uint32_t port_type) override;

 private:
  // create OMX handle
  status_t CreateHandle(char *component_Name);
  status_t DeleteHandle();

  status_t ConfigureVideoEncoder(CodecParam& codec_param);
  status_t ConfigureVideoDecoder(CodecParam& codec_param);
  status_t ConfigureAudioEncoder(CodecParam& codec_param);
  status_t ConfigureAudioDecoder(CodecParam& codec_param);

  status_t ConfigureAudioCodec(uint32_t sample_rate, uint32_t channels,
                               uint32_t bit_depth, AudioFormat format_type,
                               AudioCodecParams codec_param);

  status_t SetupAVCEncoderParameters(CodecParam& codec_param);
  status_t SetupHEVCEncoderParameters(CodecParam& codec_param);
  status_t ConfigureBitrate(CodecParam& codec_param);
  OMX_ERRORTYPE ConfigureSAR(uint32_t width, uint32_t height);
  status_t SetPortParams(OMX_U32 ePortIndex, OMX_U32 nWidth, OMX_U32 nHeight,
                         float nFrameRate);
  status_t QmmftoOmxProfile(CodecParam& codec_param);
  status_t OmxtoQmmfProfile(const VideoFormat format, const int32_t profile,
                            void *param);
  status_t QmmftoOmxLevel(CodecParam& codec_param);
  status_t OmxtoQmmfLevel(const VideoFormat format, const int32_t level,
                          void *param);
  OMX_ERRORTYPE prepareForAdaptivePlayback(OMX_U32 portIndex, OMX_BOOL enable,
                                           OMX_U32 maxFrameWidth,
                                           OMX_U32 maxFrameHeight);


  status_t SetState(OMX_STATETYPE eState, OMX_BOOL bSynchronous);
  status_t WaitState(OMX_STATETYPE state);

  status_t PushEventCommand(OMX_EVENTTYPE event, OMX_COMMANDTYPE command,
                            OMX_U32, OMX_U32 flag);

  status_t EmptyThisBuffer(OMX_BUFFERHEADERTYPE *buffer);
  status_t FillThisBuffer(OMX_BUFFERHEADERTYPE *buffer);

  OMX_BUFFERHEADERTYPE* GetInputBufferHdr(BufferDescriptor& buffer);
  OMX_BUFFERHEADERTYPE* GetOutputBufferHdr(BufferDescriptor& buffer);

  bool IsInputPortStop();
  bool IsOutputPortStop();
  bool IsPortReconfig();

  void StopOutput();

  status_t FreeBufferPool();

  ::std::shared_ptr<ICodecSource>& getInputBufferSource() {
    return input_source_;
  }
  ::std::shared_ptr<ICodecSource>& getOutputBufferSource() {
    return output_source_;
  }

  // DeliverInput thread will pull data to be encoded
  static void* DeliverInput(void *ptr);

  // DeliverOutput thread will pull bitstream encoded data from Encoder
  static void* DeliverOutput(void *ptr);

  // Will check for PortReconfig Event
  static void* ThreadRun(void *arg);

  status_t HandleOutputPortSettingsChange(OMX_U32 nData2);

  status_t PortReconfigOutput();

  static OMX_ERRORTYPE OnEvent(OMX_IN OMX_HANDLETYPE component,
                               OMX_IN OMX_PTR app_data,
                               OMX_IN OMX_EVENTTYPE event,
                               OMX_IN OMX_U32 data1,
                               OMX_IN OMX_U32 data2,
                               OMX_IN OMX_PTR event_data);

  static OMX_ERRORTYPE OnEmptyBufferDone(OMX_IN OMX_HANDLETYPE component,
                                         OMX_IN OMX_PTR app_data,
                                         OMX_IN OMX_BUFFERHEADERTYPE *buffer);

  static OMX_ERRORTYPE OnFillBufferDone(OMX_IN OMX_HANDLETYPE component,
                                        OMX_IN OMX_PTR app_data,
                                        OMX_IN OMX_BUFFERHEADERTYPE *puffer);

  void UpdateBufferHeaderList(OMX_BUFFERHEADERTYPE* header);

  ::android::sp<OmxClient>        omx_client_;
  OMX_STATETYPE                   state_;
  OMX_STATETYPE                   state_pending_;
  bool                            input_stop_;
  bool                            output_stop_;
  bool                            port_status_;  // for both ports
  ::android::Mutex                input_stop_lock_;
  ::android::Mutex                output_stop_lock_;
  pthread_t                       deliver_input_thread_id_;
  pthread_t                       deliver_output_thread_id_;
  pthread_t                       port_reconfig_thread_id_;
  ::std::shared_ptr<ICodecSource> input_source_;
  ::std::shared_ptr<ICodecSource> output_source_;
  OMX_BUFFERHEADERTYPE**          in_buff_hdr_;
  OMX_BUFFERHEADERTYPE**          out_buff_hdr_;

  TSQueue<OMX_BUFFERHEADERTYPE*>  free_input_buffhdr_list_;
  TSQueue<OMX_BUFFERHEADERTYPE*>  used_input_buffhdr_list_;
  ::std::condition_variable       wait_for_header_;
  ::std::mutex                    queue_lock_;

  TSQueue<OMX_BUFFERHEADERTYPE*>  free_output_buffhdr_list_;
  TSQueue<OMX_BUFFERHEADERTYPE*>  used_output_buffhdr_list_;
  ::std::mutex                    lock_output_;
  ::std::condition_variable       wait_for_header_output_;
  ::std::mutex                    queue_lock_output_;

  uint32_t                        in_buff_hdr_size_;
  uint32_t                        out_buff_hdr_size_;
  SignalQueue<CodecCmdType>       signal_queue_;
  static OMX_CALLBACKTYPE  callbacks_;
  CodecType                format_type_;
  // to handle the two EOS callbacks from Audio OMX component
  bool                     isEOSonOutput_;
  // For registration of Buffers
  ::std::vector<BufferDescriptor> output_buffer_list_;
  ::std::vector<BufferDescriptor> input_buffer_list_;
  ::std::vector<OMX_QCOM_PLATFORM_PRIVATE_PMEM_INFO> outputpParam_enc_;
  ::std::vector<::android::VideoDecoderOutputMetaData> outputpParam_dec_;
  // For Port Reconfig
  bool                      bPortReconfig_;
  ::android::Mutex          port_reconfig_lock_;
  ::android::Mutex          threadrun_port_reconfig_lock_;
  ::android::Condition      wait_for_threadrun;
  CodecParam                codec_params_;
};

};  // namespace avcodec
};  // namespace qmmf
