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

#include <mutex>
#include <condition_variable>
#include <memory>
#include <vector>
#include <sys/time.h>

#include <utils/KeyedVector.h>

#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_camera_source.h"
#include "common/codecadaptor/src/qmmf_avcodec.h"

namespace qmmf {

namespace recorder {

class TrackEncoder;

class EncoderCore {
 public:

  static EncoderCore* CreateEncoderCore();

  ~EncoderCore();

  status_t AddSource(const ::std::shared_ptr<TrackSource>& track_source,
                     VideoTrackParams& params);

  status_t StartTrackEncoder(uint32_t track_id);

  status_t StopTrackEncoder(uint32_t track_id, bool is_force_cleanup = false);

  status_t SetTrackEncoderParams(uint32_t track_id,
                                 CodecParamType param_type, void* param,
                                 uint32_t param_size);

  status_t DeleteTrackEncoder(uint32_t track_id);

  status_t ReturnTrackBuffer(const uint32_t track_id,
                             std::vector<BnBuffer> &buffers);
 private:

  bool isTrackValid(uint32_t track_id);

  // vector <track_id, shared_ptr<TrackEncoder> >
  DefaultKeyedVector<uint32_t, ::std::shared_ptr<TrackEncoder>> track_encoders_;

  int32_t ion_device_;

  // Not allowed
  EncoderCore();
  EncoderCore(const EncoderCore&);
  EncoderCore& operator=(const EncoderCore&);
  static EncoderCore* instance_;
};

class TrackEncoder : public ICodecSource {
 public:

  TrackEncoder(int32_t ion_device);

  ~TrackEncoder();

  status_t Init(const ::std::shared_ptr<TrackSource>& track_source,
                const ::std::shared_ptr<TrackEncoder>& track_encoder,
                VideoTrackParams& params);

  status_t Start();

  status_t Stop(bool is_force_cleanup = false);

  status_t SetParams(CodecParamType param_type, void* param,
                     uint32_t param_size);

  status_t ReleaseHeaders();

  // Methods of AVCodec
  // This method provides free output port buffer to AVCodec.
  status_t GetBuffer(BufferDescriptor& codec_buffer,
                     void* client_data) override;

  // This method provides filled output buffer to TrackEncoder.
  status_t ReturnBuffer(BufferDescriptor& codec_buffer,
                        void* client_data) override;

  status_t NotifyPortEvent(PortEventType event_type,
                           void* event_data) override;

  // Method to handle returned buffers from client.
  status_t OnBufferReturnFromClient(std::vector<BnBuffer> &buffers);

 private:

  status_t AllocOutputPortBufs();

  // Value of flag can be ION_IOC_CLEAN_CACHES to clean cache or
  // ION_IOC_CLEAN_INV_CACHES to invalidate cache
  status_t SynchronizeCache(const struct ion_handle_data& ion_handle,
                            const BufferDescriptor& buffer,
                            const unsigned int flag);

  // This methos Notifies bitstream buffer to remote client.
  void NotifyBufferToClient(BufferDescriptor& codec_buffer);

#ifdef DUMP_BITSTREAM
  void DumpBitStream(BufferDescriptor& codec_buffer);
#endif

  uint32_t TrackId() { return track_params_.track_id; }

  VideoTrackParams track_params_;
  AVCodec*         avcodec_;

  ::std::vector<BufferDescriptor> output_buffer_list_;
  ::std::vector<struct ion_handle_data> output_ion_list_;

  TSQueue<BufferDescriptor>  output_free_buffer_queue_;
  TSQueue<BufferDescriptor>  output_occupy_buffer_queue_;
  int32_t                    ion_device_;
  bool                       eos_atoutput_;
#ifdef DUMP_BITSTREAM
  int32_t                    file_fd_;
#endif
  bool                       is_force_cleanup_;

  // Encoded stream Dynamic FPS measurement
  uint32_t                   debug_fps_;
  uint32_t                   num_bytes_;
  struct timeval             prevtv_;
  uint32_t                   count_;

  // FD and IonHandle map
  ::std::map<int32_t, struct ion_handle_data> fd_ion_handle_map_;

  std::mutex                 queue_lock_;
  std::condition_variable    wait_for_frame_;
};

}; // namespace recorder

}; // name space qmmf
