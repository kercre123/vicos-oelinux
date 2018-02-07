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

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <queue>

#include "common/codecadaptor/src/qmmf_avcodec.h"
#include "recorder/src/service/qmmf_audio_track_source.h"
#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_recorder_ion.h"

namespace qmmf {
namespace recorder {

class AudioTrackEncoder : public ::qmmf::avcodec::ICodecSource {
 public:
  AudioTrackEncoder();
  virtual ~AudioTrackEncoder();

  status_t Init(const AudioTrackParams& params);

  status_t Start(const ::std::shared_ptr<ICodecSource> &track_source,
                 const ::std::shared_ptr<ICodecSource> &track_encoder);
  status_t Stop();
  status_t Pause();
  status_t Resume();

  status_t SetParam(CodecParamType param_type, void* param,
                    uint32_t param_size);

  // methods of IOutputCodecSource
  status_t GetBuffer(BufferDescriptor& codec_buffer,
                     void* client_data) override;
  status_t ReturnBuffer(BufferDescriptor& codec_buffer,
                        void* client_data) override;
  status_t NotifyPortEvent(::qmmf::avcodec::PortEventType event_type,
                           void* event_data) override;

  // handle returned buffers from client
  status_t OnBufferReturnFromClient(const std::vector<BnBuffer> &buffers);

 private:
  AudioTrackParams track_params_;
  ::qmmf::avcodec::AVCodec* avcodec_;
  ::std::queue<BufferDescriptor> buffers_;
  RecorderIon ion_;

  ::std::mutex mutex_;
  ::std::condition_variable signal_;
};

class AudioEncoderCore {
 public:
  static AudioEncoderCore* CreateAudioEncoderCore();

  virtual ~AudioEncoderCore();

  status_t AddSource(const AudioTrackParams& params);
  status_t DeleteTrackEncoder(const uint32_t track_id);

  status_t StartTrackEncoder(const uint32_t track_id,
                             const ::std::shared_ptr<IAudioTrackSource>&
                             track_source);
  status_t StopTrackEncoder(const uint32_t track_id);
  status_t PauseTrackEncoder(const uint32_t track_id);
  status_t ResumeTrackEncoder(const uint32_t track_id);

  status_t SetTrackEncoderParam(const uint32_t track_id,
                                const CodecParamType param_type, void* param,
                                const uint32_t param_size);

  status_t ReturnTrackBuffer(const uint32_t track_id,
                             const std::vector<BnBuffer>& buffers);

 private:
  typedef ::std::map<uint32_t, ::std::shared_ptr<AudioTrackEncoder>>
          AudioTrackEncoderMap;

  AudioEncoderCore();
  static AudioEncoderCore* instance_;

  AudioTrackEncoderMap track_encoder_map_;

  // disable copy, assignment, and move
  AudioEncoderCore(const AudioEncoderCore&) = delete;
  AudioEncoderCore(AudioEncoderCore&&) = delete;
  AudioEncoderCore& operator=(const AudioEncoderCore&) = delete;
  AudioEncoderCore& operator=(const AudioEncoderCore&&) = delete;
};

}; // namespace recorder
}; // namespace qmmf
