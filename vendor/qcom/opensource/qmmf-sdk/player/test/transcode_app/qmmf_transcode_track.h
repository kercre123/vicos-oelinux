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

#include "qmmf_transcode_params.h"
#include "qmmf_transcode_pipe.h"
#include "qmmf_transcode_sink.h"
#include "qmmf_transcode_source.h"

namespace qmmf {
namespace transcode {

static const ::std::string kDefaultConfigFile =
     "/data/misc/qmmf/transcode_config.txt";

class TranscoderTrack {
 public:
  TranscoderTrack(const ::std::string& file = kDefaultConfigFile);
  ~TranscoderTrack();

  status_t PreparePipeline();
  status_t Start();
  status_t Stop();
  status_t Delete();

  void ReleaseResources();

 private:

  status_t ParseFile(const ::std::string& filename);

  status_t FillParams();
  status_t CreateDataSource();
  status_t ReadMediaInfo();
  status_t ReadAudioTrackMediaInfo(const uint32 ulTkId,
                                   const FileSourceMnMediaType eTkMnType);
  status_t ReadVideoTrackMediaInfo(const uint32 ulTkId,
                                   const FileSourceMnMediaType eTkMnType);

  static void* DeliverInput(void* ptr);
  static void* ReceiveOutput(void* ptr);
  static void* StopTransCoding(void* ptr);

  inline ::std::shared_ptr<TranscoderSource>& getInputBufferSource() {
    return transcoder_source_;
  }

  inline ::std::shared_ptr<TranscoderSink>& getOutputBufferSource() {
    return transcoder_sink_;
  }

  inline bool IsInputPortStop() {
    ::std::lock_guard<::std::mutex> lg(input_stop_mutex_);
    return input_stop_;
  }

  ::std::shared_ptr<TranscoderSource>   transcoder_source_;
  ::std::shared_ptr<TranscoderSink>     transcoder_sink_;
  ::std::shared_ptr<TranscoderPipe>     transcoder_pipe_;
  TranscodeParams                       params_;

  MM_TRACK_INFOTYPE                     m_sTrackInfo_;
  CMM_MediaSourcePort*                  m_pIStreamPort_;
  CMM_MediaDemuxInt*                    m_pDemux_;
  TrackTypes                            track_type_;

  bool                                  isFirstFrame_;
  bool                                  isLastFrame_;
  bool                                  input_stop_;
  ::std::mutex                          input_stop_mutex_;
  ::std::thread*                        deliver_thread_;
  ::std::thread*                        receiver_thread_;
  ::std::thread*                        stop_thread_;
  int32_t                               num_delivered_frames_;
  int32_t                               num_received_frames_;
};  // class TranscoderTrack

};  // namespace transcode
};  // namespace qmmf
