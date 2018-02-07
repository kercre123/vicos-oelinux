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

#include <unistd.h>

#include <memory>
#include <mutex>
#include <vector>

#include <utils/KeyedVector.h>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/inc/qmmf_audio_endpoint.h"
#include "common/codecadaptor/src/qmmf_avcodec.h"
#include "player/src/service/qmmf_player_common.h"

using ::qmmf::common::audio::AudioEndPoint;
using ::qmmf::common::audio::AudioEndPointType;
using ::qmmf::common::audio::AudioMetadata;
using ::qmmf::common::audio::AudioBuffer;
using ::qmmf::common::audio::AudioEventHandler;
using ::qmmf::common::audio::AudioEventType;
using ::qmmf::common::audio::AudioEventData;

#define NUMBER_OF_SINK_BUFFERS 4

namespace qmmf {
namespace player {

class AudioTrackSink;

class AudioSink {
 public:
  static AudioSink* CreateAudioSink();

  ~AudioSink();

  status_t CreateTrackSink(uint32_t track_id, AudioTrackParams& param);

  const ::std::shared_ptr<AudioTrackSink>& GetTrackSink(uint32_t track_id);

  status_t StartTrackSink(uint32_t track_id);

  status_t StopTrackSink(uint32_t track_id);

  status_t DeleteTrackSink(uint32_t track_id);

 private:
  AudioSink();

  static AudioSink* instance_;

  // Map of track id and TrackSink.
  ::android::DefaultKeyedVector<uint32_t, ::std::shared_ptr<AudioTrackSink>>
      audio_track_sinks;
};

class AudioTrackSink : public ::qmmf::avcodec::ICodecSource {
 public:
  AudioTrackSink();

  ~AudioTrackSink();

  status_t Init(AudioTrackParams& param);

  status_t StartSink();

  status_t StopSink();

  status_t PauseSink();

  status_t ResumeSink();

  status_t DeleteSink();

  void AddBufferList(::android::Vector<::qmmf::avcodec::CodecBuffer>& list);

  status_t GetBuffer(BufferDescriptor& codec_buffer,
                     void* client_data) override;
  status_t ReturnBuffer(BufferDescriptor& codec_buffer,
                        void* client_data) override;
  status_t NotifyPortEvent(::qmmf::avcodec::PortEventType event_type,
                           void* event_data) override;

 private:
  int32_t TrackId() { return track_params_.track_id; }

  status_t ConfigureSink(AudioTrackParams& track_param);

  int32_t AllocateSinkBuffer(const int32_t number, const int32_t size);

  int32_t GetSinkBuffer(std::vector<AudioBuffer>& buffers);

  int32_t FillSinkBuffer(BufferDescriptor& codec_buffer);

  void ErrorHandler(const int32_t error);

  void BufferHandler(const AudioBuffer& buffer);

  AudioTrackParams       track_params_;
  AudioEndPoint*         end_point_;

  // For decoded frame
  ::android::Vector<::qmmf::avcodec::CodecBuffer>  output_buffer_list_;
  TSQueue<::qmmf::avcodec::CodecBuffer>            output_free_buffer_queue_;
  TSQueue<::qmmf::avcodec::CodecBuffer>            output_occupy_buffer_queue_;

  ::android::Mutex                  wait_for_frame_lock_;
  ::android::Condition              wait_for_frame_;
  int32_t                           ion_device_;
  ::android::Mutex                  queue_lock_;

  // For Sink
  int32_t sink_buffer_size_;
  int32_t number_of_sink_buffer = NUMBER_OF_SINK_BUFFERS;

  ::android::Vector<AudioBuffer>    audio_sink_buffer_list_;

  typedef  struct ion_allocation_data IonHandleData;
  ::android::Vector<IonHandleData>  ion_handle_data;

  TSQueue<AudioBuffer>              sink_buffer_queue_;
  ::android::Mutex                  sink_queue_lock_;
  ::android::Mutex                  wait_for_sink_queue_lock_;
  ::android::Condition              wait_for_sink_frame_;
  bool                              stopplayback_;
  bool                              paused_;
  uint32_t                          decoded_frame_number_;

  enum class AudioMessageType {
    kMessageStop,
    kMessagePause,
    kMessageResume,
    kMessageBuffer,
  };

  struct AudioMessage {
    AudioMessageType type;
    AudioBuffer buffer;
  };

#ifdef DUMP_PCM_DATA
  int32_t               file_fd_;
  void DumpPCMData(BufferDescriptor& codec_buffer);
#endif

  uint32_t               total_bytes_decoded_;
  std::mutex             state_change_lock_;
};

};  // namespace player
};  // namespace qmmf
