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

#ifndef QMMF_MUX_INTERACE_H_
#define QMMF_MUX_INTERACE_H_

#include <functional>
#include <stdint.h>
#include <qmmf-sdk/qmmf_recorder_params.h>
#include <filemux.h>
#include <filemuxtypes.h>
#include <qmmf_http_interface.h>
#include <utils/String8.h>

#define  MAX_STREAMS                     2
#define  VIDEO_STREAM_INDEX              0
#define  AUDIO_STREAM_INDEX              1
#define  ADTS_HEADER_SIZE                7

class FileMux;

namespace qmmf {

namespace muxinterface {

using namespace android;
using namespace recorder;

class MuxInterface;

typedef struct qmmf_muxer_video_stream_t {
  size_t width;
  size_t height;
  size_t bitrate;
  size_t framerate;
  MUX_stream_video_type format;
  uint8_t codec_profile;
  uint8_t codec_level;
  uint32_t track_id;
} qmmf_muxer_video_stream;

typedef std::function<void(uint32_t session_id, uint32_t track_id,
                           BufferDescriptor &buffer)> ReleaseCb;

typedef struct qmmf_muxer_init_t {
  bool audio_stream_present;
  qmmf_audio_track_param audio_stream;
  bool video_stream_present;
  qmmf_muxer_video_stream video_stream;
  ReleaseCb release_cb;
  MUX_brand_type brand;
} qmmf_muxer_init;

typedef struct qmmf_muxer_client_data_t {
  uint32_t session_id;
  uint32_t track_id;
  BufferDescriptor buffer;
} qmmf_muxer_client_data;

class MuxInterface {
 public:

  MuxInterface(MUX_brand_type brand, const char *file_name);
  virtual ~MuxInterface();

  int32_t Init(const qmmf_muxer_init & params);
  int32_t WriteBuffer(uint32_t track_id, uint32_t session_id,
                      BufferDescriptor &buffer, MetaData &meta_data);
  int32_t Stop();

 private:

  int32_t Flush();
  int32_t SetMainMuxerParams(MUX_create_params_type &params,
                             const qmmf_muxer_init &init_params);
  int32_t SetAudioMuxerParams(MUX_create_params_type &params,
                              const qmmf_muxer_init &init_params);
  int32_t SetVideoMuxerParams(MUX_create_params_type &params,
                              const qmmf_muxer_init &init_params);
  int32_t ConvertStatus(MUX_STATUS status);
  static void FileMuxCb(int status, void *client_data,
                        void *sample_info, void *buffer);
  void ReleaseBuffer(uint32_t track_id, uint32_t session_id,
                     BufferDescriptor &buffer);
  int32_t WriteAudioBuffer(uint32_t track_id, uint32_t session_id,
                           BufferDescriptor &buffer,
                           MetaData &meta_data, int32_t idx);
  int32_t WriteVideoBuffer(uint32_t track_id, uint32_t session_id,
                           BufferDescriptor &buffer,
                           MetaData &meta_data, int32_t idx);

  FileMux *mux_;
  ReleaseCb release_cb_;
  MUX_brand_type brand_type_;
  MUX_fmt_type format_type_;
  String8 file_name_;
  MUX_create_params_type create_params_;
  MUX_stream_create_params_type streams_[MAX_STREAMS];
  MUX_handle_type mux_file_handle_;
  int64_t video_start_time_;
  int64_t first_timestamp_;
  int64_t video_previous_time_;
  uint32_t video_track_id_;
  uint32_t audio_track_id_;
  qmmf_audio_track_param audio_track_parms_;

  pthread_mutex_t stop_lock_;
  pthread_mutex_t flush_lock_;
  bool close_completed_;
  bool flush_completed_;
  int32_t close_status_;
  int32_t flush_status_;
  pthread_cond_t stop_cond_;
  pthread_cond_t flush_cond_;

  uint8_t adts_header_[ADTS_HEADER_SIZE];

  static const char kLibPath[];
  static const char kLibInstantiate[];
  static const char kMP4MajorBrand[];
  static const char *kMP4CompatBrands[];
  static const char k3GPMajorBrand[];
  static const char *k3GPCompatBrands[];

  /**Not allowed */
  MuxInterface(const MuxInterface &);
  MuxInterface &operator=(const MuxInterface &);
};

} //namespace muxinterface ends
} //namespace qmmf ends

#endif /* QMMF_MUX_INTERACE_H_ */
