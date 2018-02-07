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

#define TAG "AudioSink"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <memory>

#include <linux/msm_ion.h>

#include "player/src/service/qmmf_player_audio_sink.h"

namespace qmmf {
namespace player {

using ::android::List;
using ::android::Mutex;
using ::android::NO_ERROR;
using ::android::Vector;
using ::qmmf::avcodec::CodecBuffer;
using ::qmmf::avcodec::CodecPortStatus;
using ::qmmf::avcodec::PortEventType;
using ::qmmf::avcodec::PortreconfigData;
using ::std::make_shared;
using ::std::shared_ptr;

AudioSink* AudioSink::instance_ = nullptr;

AudioSink* AudioSink::CreateAudioSink() {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  if (!instance_) {
    instance_ = new AudioSink();
    if (!instance_) {
      QMMF_ERROR("%s:%s: Can't Create Audio sink Instance!", TAG, __func__);
      return NULL;
    }
  }

  QMMF_INFO("%s:%s: Audio Sink Instance Created Successfully(0x%p)", TAG,
      __func__, instance_);

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
  return instance_;
}

AudioSink::AudioSink() {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
}

AudioSink::~AudioSink() {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  if (!audio_track_sinks.isEmpty()) {
    audio_track_sinks.clear();
  }
  instance_ = NULL;
  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
}

status_t AudioSink::CreateTrackSink(uint32_t track_id,
                                    AudioTrackParams& param) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  shared_ptr<AudioTrackSink> track_sink;

  if (param.params.out_device == AudioOutSubtype::kBuiltIn)
    track_sink = make_shared<AudioTrackSink>();

  audio_track_sinks.add(track_id,track_sink);
  track_sink->Init(param);
  QMMF_DEBUG("%s:%s Exit", TAG, __func__);

  return 0;
}

const shared_ptr<AudioTrackSink>& AudioSink::GetTrackSink(
    uint32_t track_id) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  int32_t idx = audio_track_sinks.indexOfKey(track_id);
  assert(idx >= 0);
  return audio_track_sinks.valueFor(track_id);
  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
}

status_t AudioSink::StartTrackSink(uint32_t track_id) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  shared_ptr<AudioTrackSink> track_sink = audio_track_sinks.valueFor(track_id);
  assert(track_sink.get() != NULL);

  auto ret = track_sink->StartSink();
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: track_id(%d) StartSink failed!", TAG, __func__,
      track_id);
    return ret;
  }

  QMMF_INFO("%s:%s: track_id(%d) StartSink Successful!", TAG,
    __func__, track_id);

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t AudioSink::StopTrackSink(uint32_t track_id) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  shared_ptr<AudioTrackSink> track_sink = audio_track_sinks.valueFor(track_id);
  assert(track_sink.get() != NULL);

  auto ret = track_sink->StopSink();
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: track_id(%d) StopSink failed!", TAG, __func__,
      track_id);
    return ret;
  }

  QMMF_INFO("%s:%s: track_id(%d) StopSink Successful!", TAG,
    __func__, track_id);

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t AudioSink::DeleteTrackSink(uint32_t track_id) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  shared_ptr<AudioTrackSink> track_sink = audio_track_sinks.valueFor(track_id);
  assert(track_sink.get() != NULL);

  auto ret = track_sink->DeleteSink();
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: track_id(%d) DeleteSink failed!", TAG, __func__,
      track_id);
    return ret;
  }

  audio_track_sinks.removeItem(track_id);

  QMMF_INFO("%s:%s: track_id(%d) DeleteSink Successful!", TAG,
    __func__, track_id);

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
  return ret;
}

AudioTrackSink::AudioTrackSink()
    : end_point_(nullptr), stopplayback_(false),
      paused_(false), decoded_frame_number_(0),
      total_bytes_decoded_(0) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

#ifdef DUMP_PCM_DATA
  file_fd_ = -1;
#endif

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
}

AudioTrackSink::~AudioTrackSink() {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  delete end_point_;

  uint32_t i = 0;
  for(auto& iter : audio_sink_buffer_list_) {
    if ((iter).data) {
      munmap((iter).data, (iter).size);
      (iter).data = nullptr;
    }
    if ((iter).ion_fd) {
      QMMF_INFO("%s:%s track_id(%d) (iter).fd =%d Free", TAG, __func__,
                                 TrackId(), (iter).ion_fd);
      ioctl(ion_device_, ION_IOC_FREE, &(ion_handle_data[i]));
      close((iter).ion_fd);
      (iter).ion_fd = -1;
    }
    i++;
  }

  audio_sink_buffer_list_.clear();
  ion_handle_data.clear();

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
}

void AudioTrackSink::ErrorHandler(const int32_t error) {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  QMMF_VERBOSE("%s: %s() INPARAM: error[%d]", TAG, __func__, error);

  assert(false);
}

void AudioTrackSink::BufferHandler(const AudioBuffer& buffer) {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  QMMF_VERBOSE("%s: %s() INPARAM: buffer[%s]", TAG, __func__,
               buffer.ToString().c_str());

  Mutex::Autolock lock(sink_queue_lock_);
  sink_buffer_queue_.PushBack(buffer);
  wait_for_sink_frame_.signal();
}

status_t AudioTrackSink::Init(AudioTrackParams& track_param) {
  QMMF_INFO("%s:%s: Enter track_id(%d)", TAG, __func__, track_param.track_id);

  track_params_.track_id = track_param.track_id;

  ConfigureSink(track_param);

#ifdef DUMP_PCM_DATA
  file_fd_ = open("/data/misc/qmmf/audio_track.pcm", O_CREAT | O_WRONLY | O_TRUNC, 0655);
#endif

  QMMF_INFO("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());

  return NO_ERROR;
}

//Audio Sink Connect and Initilization
status_t AudioTrackSink::ConfigureSink(AudioTrackParams& track_param) {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());

  int32_t result;
  int32_t latency;

  if (end_point_ != nullptr) {
    QMMF_ERROR("%s: %s() endpoint already exists", TAG, __func__);
    return ::android::ALREADY_EXISTS;
  }

  end_point_ = new AudioEndPoint;
  if (end_point_ == nullptr) {
    QMMF_ERROR("%s: %s() could not instantiate endpoint", TAG, __func__);
    return ::android::NO_MEMORY;
  }

  AudioEventHandler audio_handler =
    [this] (AudioEventType event_type, const AudioEventData& event_data)
           -> void {
      switch (event_type) {
        case AudioEventType::kError:
          ErrorHandler(event_data.error);
          break;
        case AudioEventType::kBuffer:
          BufferHandler(event_data.buffer);
          break;
      }
    };

  result = end_point_->Connect(audio_handler);
  if (result < 0) {
    QMMF_ERROR("%s: %s() endpoint->Connect failed: %d[%s]", TAG, __func__,
               result, strerror(result));
    //goto error_free;
  }

  std::vector<DeviceId> devices;
  devices.push_back(0);

  AudioMetadata metadata;
  memset(&metadata, 0x0, sizeof metadata);

  metadata.format       =  AudioFormat::kPCM;
  metadata.num_channels  = track_param.params.channels;
  metadata.sample_rate  =  track_param.params.sample_rate;
  metadata.sample_size  =  track_param.params.bit_depth;

  DebugAudioSinkParam(__func__, track_param);

  result = end_point_->Configure(AudioEndPointType::kSink, devices, metadata);
  assert(result == 0);

  result = end_point_->GetLatency(&latency);
  assert(result == 0);
  QMMF_INFO("%s: %s() latency is %d", TAG, __func__, latency);

  result = end_point_->GetBufferSize(&sink_buffer_size_);
  assert(result == 0);
  QMMF_INFO("%s: %s() buffer_size is %d", TAG, __func__, sink_buffer_size_);

  AllocateSinkBuffer(number_of_sink_buffer, 8192);

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());

  return NO_ERROR;

  /*
error_free:
delete end_point_;
end_point_ = nullptr;
*/
}

status_t AudioTrackSink::StartSink() {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  std::lock_guard<std::mutex> lock(state_change_lock_);

#ifdef DUMP_PCM_DATA
 if (file_fd_ == -1)
    file_fd_ = open("/data/misc/qmmf/audio_track.pcm", O_CREAT | O_WRONLY | O_TRUNC, 0655);
#endif

  auto ret = end_point_->Start();
  stopplayback_ = false;
  assert(ret == NO_ERROR);
  if (ret != NO_ERROR) {
   QMMF_ERROR("%s:%s: track_id(%d) StartSink failed!", TAG, __func__,
       TrackId());
   return ret;
  }
  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t AudioTrackSink::StopSink() {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  std::lock_guard<std::mutex> lock(state_change_lock_);

  stopplayback_ = true;
  QMMF_DEBUG("%s:%s: Total number of audio frames decoded %d", TAG, __func__,
      decoded_frame_number_);
  decoded_frame_number_ = 0;

  QMMF_DEBUG("%s:%s: Total number of audio bytes decoded %d", TAG, __func__,
      total_bytes_decoded_);
  total_bytes_decoded_ = 0;

  auto ret = end_point_->Stop(true);
  assert(ret == NO_ERROR);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: track_id(%d) StopSink failed!", TAG, __func__,
        TrackId());
    return ret;
  }

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t AudioTrackSink::PauseSink() {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  std::lock_guard<std::mutex> lock(state_change_lock_);

  paused_ = true;
  auto ret = end_point_->Pause();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: track_id(%d) PauseSink failed!", TAG, __func__,
        TrackId());
    return ret;
  }

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t AudioTrackSink::ResumeSink() {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  std::lock_guard<std::mutex> lock(state_change_lock_);

  paused_ = false;
  auto ret = end_point_->Resume();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: track_id(%d) ResumeSink failed!", TAG, __func__,
        TrackId());
    return ret;
  }

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t AudioTrackSink::DeleteSink() {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  std::lock_guard<std::mutex> lock(state_change_lock_);

  auto ret = end_point_->Disconnect();
  assert(ret == 0);

  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: track_id(%d) Disconnect failed!", TAG, __func__,
        TrackId());
    return ret;
  }

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

void AudioTrackSink::AddBufferList(Vector<CodecBuffer>& list) {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());

  output_buffer_list_ = list;
  output_free_buffer_queue_.Clear();
  output_occupy_buffer_queue_.Clear();

  //decoded buffer queue
  for(auto& iter : output_buffer_list_) {
          QMMF_INFO("%s:%s: track_id(%d) Adding buffer fd(%d) to "
              "output_free_buffer_queue_", TAG, __func__,TrackId() ,
              iter.fd);
          output_free_buffer_queue_.PushBack(iter);
  }

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
}

//********************************************************************//

status_t AudioTrackSink::GetBuffer(BufferDescriptor& codec_buffer,
                                   void* client_data) {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  // Give available free buffer to decoder to use on output port.

  if(output_free_buffer_queue_.Size() <= 0) {
    QMMF_DEBUG("%s:%s track_id(%d) No buffer available to notify,"
      " Wait for new buffer", TAG, __func__, TrackId());
    Mutex::Autolock autoLock(wait_for_frame_lock_);
    wait_for_frame_.wait(wait_for_frame_lock_);
  }

  CodecBuffer iter = *output_free_buffer_queue_.Begin();
  codec_buffer.fd = (iter).fd;
  codec_buffer.data = (iter).pointer;
  output_free_buffer_queue_.Erase(output_free_buffer_queue_.Begin());
  {
    Mutex::Autolock lock(queue_lock_);
    output_occupy_buffer_queue_.PushBack(iter);
  }
  QMMF_DEBUG("%s:%s track_id(%d) Sending buffer(0x%p) fd(%d) for FTB", TAG,
      __func__, TrackId(), codec_buffer.data, codec_buffer.fd);

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return NO_ERROR;
}

status_t AudioTrackSink::ReturnBuffer(BufferDescriptor& codec_buffer,
                                      void* client_data) {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  status_t ret = 0;

  assert(codec_buffer.data != NULL);

  QMMF_VERBOSE("%s:%s: track_id(%d) Received buffer(0x%p) from FBD", TAG,
      __func__, TrackId(), codec_buffer.data);

#ifdef DUMP_PCM_DATA
  DumpPCMData(codec_buffer);
#endif

  if (!((codec_buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS)) ||
      stopplayback_ || !(codec_buffer.size) || paused_)) {
    QMMF_DEBUG("%s:%s: track_id(%d) For decoded/rendered audio frame number %d"
        " timestamps is %llu ",TAG, __func__, TrackId(), ++decoded_frame_number_,
        codec_buffer.timestamp);
    FillSinkBuffer(codec_buffer);
  }

  List<CodecBuffer>::iterator it = output_occupy_buffer_queue_.Begin();
  bool found = false;
  for (; it != output_occupy_buffer_queue_.End(); ++it) {
    QMMF_VERBOSE("%s:%s track_id(%d) Checking match (0x%p)vs(0x%p) ", TAG,
        __func__, TrackId(), (*it).pointer,  codec_buffer.data);
    if (((*it).pointer) == (codec_buffer.data)) {
      QMMF_VERBOSE("%s:%s track_id(%d) Buffer found", TAG, __func__, TrackId());
      output_free_buffer_queue_.PushBack(*it);
      output_occupy_buffer_queue_.Erase(it);
      wait_for_frame_.signal();
      found = true;
      break;
    }
  }
  assert(found == true);

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t AudioTrackSink::NotifyPortEvent(PortEventType event_type,
                                         void* event_data) {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return 0;
}

int32_t AudioTrackSink::FillSinkBuffer(BufferDescriptor& codec_buffer) {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());

  int32_t result = 0;

  std::vector<AudioBuffer> sinkbuffers;

  // For Audio Sink
  AudioBuffer buffer;
  memset(&buffer, 0x0, sizeof (AudioBuffer));
  sinkbuffers.push_back(buffer);

  GetSinkBuffer(sinkbuffers);

  memcpy(sinkbuffers[0].data, ((uint8_t*)codec_buffer.data +
    codec_buffer.offset), codec_buffer.size);

  sinkbuffers[0].size = codec_buffer.size;
  sinkbuffers[0].capacity = codec_buffer.size;

  total_bytes_decoded_ = total_bytes_decoded_ + sinkbuffers[0].size;

  QMMF_VERBOSE("%s:%s: sink buffer vaddr = 0x%p", TAG, __func__,
      sinkbuffers[0].data);
  QMMF_VERBOSE("%s:%s: sink buffer size = %d", TAG, __func__,
      sinkbuffers[0].size);

  {
    std::lock_guard<std::mutex> lock(state_change_lock_);
    if(!paused_)
      result = end_point_->SendBuffers(sinkbuffers);
  }

  assert(result == 0);

  sinkbuffers.clear();

  if (paused_) {
    Mutex::Autolock lock(sink_queue_lock_);
    sinkbuffers[0].size = 0;
    sinkbuffers[0].timestamp = 0;
    sink_buffer_queue_.PushBack(sinkbuffers[0]);
    wait_for_sink_frame_.signal();
  }

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());

  return 0;
}

int32_t AudioTrackSink::GetSinkBuffer(std::vector<AudioBuffer>& buffers) {
  QMMF_VERBOSE("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());

  if(sink_buffer_queue_.Size() <= 0) {
    QMMF_DEBUG("%s:%s track_id(%d) No buffer available for Audio sink,"
      " Wait for new buffer", TAG, __func__, TrackId());
    Mutex::Autolock autoLock(sink_queue_lock_);
    wait_for_sink_frame_.wait(sink_queue_lock_);
    //TODO: change simple wait to relative wait.
  }

  int32_t size = buffers.size();

  for (int32_t i = 0; i < size; i++) {

    AudioBuffer iter = *sink_buffer_queue_.Begin();

    buffers[i].ion_fd = (iter).ion_fd;
    buffers[i].buffer_id = (iter).ion_fd;
    buffers[i].data = (iter).data;
    buffers[i].size = (iter).size;
    buffers[i].capacity = (iter).capacity;

    sink_buffer_queue_.Erase(sink_buffer_queue_.Begin());
  }
  QMMF_VERBOSE("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return NO_ERROR;
}

int32_t AudioTrackSink::AllocateSinkBuffer(const int32_t number,
                                           const int32_t size) {
  QMMF_VERBOSE("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  QMMF_VERBOSE("%s: %s() number[%d]", TAG, __func__, number);
  QMMF_VERBOSE("%s: %s() size[%d]", TAG, __func__, size);

  if (number <= 0) return -EINVAL;
  if (size <= 0) return -EINVAL;
  int32_t ret = 0;

  // open ion device
  ion_device_ = open("/dev/ion", O_RDONLY);
  if (ion_device_ < 0) {
    QMMF_ERROR("%s: %s() error opening ion device: %d[%s]", TAG, __func__,
               errno, strerror(errno));
    return errno;
  }

  int32_t ion_type = 0x1 << ION_IOMMU_HEAP_ID;
  void *vaddr      = NULL;

  struct ion_allocation_data alloc;
  struct ion_fd_data         ion_fddata;

  for (int32_t index = 0; index < number; ++index) {
    vaddr = NULL;
    AudioBuffer buffer;
    memset(&buffer, 0x0, sizeof (AudioBuffer));
    memset(&alloc, 0x0, sizeof(ion_allocation_data));
    memset(&ion_fddata, 0x0, sizeof(ion_fddata));

    alloc.len = size;
    alloc.len = (alloc.len + 4095) & (~4095);
    alloc.align = 4096;
    alloc.flags = ION_FLAG_CACHED;
    alloc.heap_id_mask = ion_type;

    ret = ioctl(ion_device_, ION_IOC_ALLOC, &alloc);
    if (ret < 0) {
      QMMF_ERROR("%s:%s ION allocation failed!", TAG, __func__);
      goto ION_ALLOC_FAILED;
    }

    ion_fddata.handle = alloc.handle;
    ret = ioctl(ion_device_, ION_IOC_SHARE, &ion_fddata);
    if (ret < 0) {
        QMMF_ERROR("%s:%s ION map failed %s", TAG, __func__, strerror(errno));
        goto ION_MAP_FAILED;
    }

    vaddr = mmap(NULL, alloc.len, PROT_READ  | PROT_WRITE, MAP_SHARED,
                 ion_fddata.fd, 0);

    if (vaddr == MAP_FAILED) {
        QMMF_ERROR("%s:%s  ION mmap failed: %s (%d)", TAG, __func__,
            strerror(errno), errno);
        goto ION_MAP_FAILED;
    }

    buffer.ion_fd    = ion_fddata.fd;
    buffer.buffer_id = ion_fddata.fd;
    buffer.data      = vaddr;
    buffer.capacity  = alloc.len;
    buffer.size      = alloc.len;

    ion_handle_data.push_back(alloc);
    audio_sink_buffer_list_.push_back(buffer);
  }

  //sink buffer queue
  for(auto& iter : audio_sink_buffer_list_) {
    QMMF_INFO("%s:%s: track_id(%d) Adding buffer fd(%d) for "
            "audio_sink", TAG, __func__,TrackId() ,
            iter.ion_fd);
    sink_buffer_queue_.PushBack(iter);
  }

  QMMF_INFO("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;

ION_MAP_FAILED:
  struct ion_handle_data ionHandleData;
  memset(&ionHandleData, 0x0, sizeof(ionHandleData));
  ionHandleData.handle = ion_fddata.handle;
  ioctl(ion_device_, ION_IOC_FREE, &ionHandleData);
ION_ALLOC_FAILED:
  QMMF_ERROR("%s:%s ION Buffer allocation failed!", TAG, __func__);
  return -1;
}

#ifdef DUMP_PCM_DATA
void AudioTrackSink::DumpPCMData(BufferDescriptor& codec_buffer) {
  QMMF_VERBOSE("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  /*if(eos_atoutput_) {
    return;
  }*/

  if (file_fd_ > 0) {
    ssize_t exp_size = (ssize_t) codec_buffer.size;
    QMMF_INFO("%s:%s Got decoded buffer of size(%d)", TAG, __func__,
        codec_buffer.size);

    if (exp_size != write(file_fd_, (uint8_t*)codec_buffer.data +
        codec_buffer.offset, codec_buffer.size)) {

      QMMF_INFO("%s:%s: Bad Write error (%d) %s", TAG, __func__, errno,
          strerror(errno));
      close(file_fd_);
      file_fd_ = -1;
    }
  } else {
    QMMF_ERROR("%s:%s File is not open fd = %d", TAG, __func__, file_fd_);
  }

  if (codec_buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS)) {
    QMMF_ERROR("%s:%s This is last buffer from decoder.close file", TAG,
        __func__);
    close(file_fd_);
    file_fd_ = -1;
  }

  QMMF_VERBOSE("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
}
#endif

};  // namespace player
};  // namespace qmmf
