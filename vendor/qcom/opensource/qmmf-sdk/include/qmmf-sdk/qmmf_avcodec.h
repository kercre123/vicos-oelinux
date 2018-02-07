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

#include <memory>
#include <string>
#include <vector>

#include "qmmf-sdk/qmmf_avcodec_params.h"

namespace qmmf {
namespace avcodec {

/// \brief ICodecSource interface called on input and output port of AVCodec
///
/// Methods of ICodecSource are implemented by the client and AVCodec calls
/// these methods on input and ouput ports on a separate thread.
/// In case of input port, GetBuffer method should return a filled buffer to
/// be processed by AVCodec.
/// In case of output port, ReturnBuffer will return a codec-processed buffer
/// to client.
class ICodecSource {
 public:
  virtual ~ICodecSource() {};
  virtual status_t GetBuffer(BufferDescriptor& buffer_descriptor,
                             void* client_data) = 0;
  virtual status_t ReturnBuffer(BufferDescriptor& buffer_descriptor,
                                void* client_data) = 0;
  virtual status_t NotifyPortEvent(PortEventType event_type,
                                   void* event_data) = 0;
};

class IAVCodec {
 public:
  virtual ~IAVCodec() {};

  /// \brief Enables clients to get Codec components name that support the given
  /// mimetype.
  virtual status_t GetComponentName(CodecMimeType mime_type,
      uint32_t *num_comps, ::std::vector<::std::string>& comp_names) = 0;

  /// \brief Enables clients to configure Codec
  ///
  /// This API will configure codec and moves component in Idle state.
  /// Client will get error notification through callback specified in
  /// CodecParam.
  ///
  /// @param mime_type: Codec type to be configure.
  /// @param codec_param: Details parameter of codec.
  /// @param comp_name: Name of the specific codec component to use.
  virtual status_t ConfigureCodec(CodecMimeType mime_type,
                                  CodecParam& codec_param,
                                  ::std::string comp_name = "") = 0;

  /// \brief Enables clients to Get buffer requirement on a given port.
  virtual status_t GetBufferRequirements(uint32_t port_type,
                                         uint32_t* buf_count,
                                         uint32_t* buf_size) = 0;

  /// \brief This API will request avcodec to allocate buffer on a given port.
  /// Buffer can be meta or non meta mode and specified in CodecParam
  /// parameter. avcodec will create Bufferheader based on buf count.
  ///
  /// @param buffer_list: avcodec will fill this list with new allocated buffer.
  virtual status_t AllocateBuffer(uint32_t port_type, uint32_t buf_count,
      uint32_t buf_size, const ::std::shared_ptr<ICodecSource>& source,
      ::std::vector<BufferDescriptor> &buffer_list) = 0;

  /// \brief This API will release Bufferheader on both port and deallocate
  /// buffer in case of AllocateBuffer mode.
  virtual status_t ReleaseBuffer() = 0;

  /// \brief This API will set codec run time parameter.
  virtual status_t SetParameters(CodecParamType param_type, void *codec_param,
                                 size_t param_size) = 0;

  /// \brief This API will query codec run time parameter.
  virtual status_t GetParameters(const CodecParamType param_type,
                                 void *codec_param, size_t *param_size) = 0;

  /// \brief This API will move the codec in executing state and create two
  /// thread, one for inpurt port and one for output port.
  /// Input thread will pull data from client using method GetBuffer to be
  /// processed by codec and return buffer using method ReturnBuffer.
  /// Output thread gets empty buffer using method GetBuffer from client and
  /// return filled buffer using method ReturnBuffer.
  /// to client.
  virtual status_t StartCodec() = 0;

  /// \brief This API will wait for EOS on output port and move the codec in
  /// Idle state. It disables both port and deregister buffer header on
  /// both port.
  virtual status_t StopCodec() = 0;

  virtual status_t PauseCodec() = 0;

  virtual status_t ResumeCodec() = 0;

  virtual status_t RegisterOutputBuffers(std::vector<BufferDescriptor>& list) = 0;

  virtual status_t RegisterInputBuffers(std::vector<BufferDescriptor>& list) = 0;

  virtual status_t Flush(uint32_t port_type) = 0;

  static IAVCodec* CreateAVCodec();
};


}; // namespace avcodec
}; // namespace qmmf
