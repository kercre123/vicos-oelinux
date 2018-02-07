/*
* Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#define TAG "RecorderEncoderCore"

#include <memory>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/msm_ion.h>

#include "recorder/src/service/qmmf_encoder_core.h"

namespace qmmf {

namespace recorder {

using ::std::make_shared;
using ::std::shared_ptr;
using ::std::vector;

static const int32_t kDebugTrackFps = 1<<0;

EncoderCore* EncoderCore::instance_ = NULL;

EncoderCore* EncoderCore::CreateEncoderCore() {

  if(!instance_) {
    instance_ = new EncoderCore;
    if(!instance_) {
      QMMF_ERROR("%s:%s: Can't Create EncoderCore Instance", TAG, __func__);
      return nullptr;
    }
  }
  QMMF_INFO("%s:%s: EncoderCore Instance Created Successfully(0x%p)", TAG,
      __func__, instance_);
  return instance_;
}

EncoderCore::EncoderCore() : ion_device_(-1) {

  QMMF_KPI_GET_MASK();
  QMMF_KPI_DETAIL();
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

EncoderCore::~EncoderCore() {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  QMMF_KPI_DETAIL();
  if (!track_encoders_.isEmpty()) {
    track_encoders_.clear();
  }
  instance_ = NULL;

  if (ion_device_ > 0) {
    close(ion_device_);
    ion_device_ = -1;
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

status_t EncoderCore::AddSource(const shared_ptr<TrackSource>& track_source,
                                VideoTrackParams& params) {

  QMMF_DEBUG("%s:%s: Enter", TAG, __func__);
  assert(track_source.get() != nullptr);

  if(ion_device_ < 0) {
    ion_device_ = open("/dev/ion", O_RDONLY);
    assert(ion_device_ >=0 );
  }

  shared_ptr<TrackEncoder> track_encoder =
      make_shared<TrackEncoder>(ion_device_);
  if (!track_encoder.get()) {
    QMMF_ERROR("%s:%s: track_id(%x) Can't instantiate TrackEncoder", TAG,
        __func__, params.track_id);
    return NO_MEMORY;
  }

  auto ret = track_encoder->Init(track_source, track_encoder, params);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: track_id(%x) TrackEncoder Init failed!", TAG, __func__,
        params.track_id);
    return BAD_VALUE;
  }

  track_encoders_.add(params.track_id, track_encoder);
  QMMF_INFO("%s:%s: TrackEncoder(0x%p) for track_id(%x) Instantiated!", TAG,
      __func__, track_encoder.get(), params.track_id);

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t EncoderCore::StartTrackEncoder(uint32_t track_id) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, track_id);
  QMMF_KPI_DETAIL();

  if (!isTrackValid(track_id)) {
    QMMF_ERROR("%s:%s: Invalid track_id(%x)", TAG, __func__, track_id);
    return BAD_VALUE;
  }
  shared_ptr<TrackEncoder> track_encoder = track_encoders_.valueFor(track_id);
  assert(track_encoder.get() != NULL);

  auto ret = track_encoder->Start();
  // Initial debug purpose.
  assert(ret == NO_ERROR);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: track_id(%x) TrackEncoder Start failed!", TAG, __func__,
      track_id);
    return ret;
  }

  QMMF_INFO("%s:%s: track_id(%x) TrackEncoder Started Successfully!", TAG,
      __func__, track_id);
  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t EncoderCore::StopTrackEncoder(uint32_t track_id,
                                       bool is_force_cleanup) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, track_id);
  QMMF_KPI_DETAIL();

  if (!isTrackValid(track_id)) {
    QMMF_ERROR("%s:%s: Invalid track_id(%x)", TAG, __func__, track_id);
    return BAD_VALUE;
  }
  shared_ptr<TrackEncoder> track_encoder = track_encoders_.valueFor(track_id);
  assert(track_encoder.get() != NULL);

  auto ret = track_encoder->Stop(is_force_cleanup);
  // Initial debug purpose.
  assert(ret == NO_ERROR);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: track_id(%x) TrackEncoder Stop failed!", TAG, __func__,
      track_id);
    return ret;
  }

  QMMF_INFO("%s:%s: track_id(%x) TrackEncoder Stopped Successfully!", TAG,
      __func__, track_id);
  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t EncoderCore::SetTrackEncoderParams(uint32_t track_id,
                                            CodecParamType param_type,
                                            void* param, uint32_t param_size) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, track_id);
  if (!isTrackValid(track_id)) {
    QMMF_ERROR("%s:%s: Invalid track_id(%x)", TAG, __func__, track_id);
    return BAD_VALUE;
  }
  shared_ptr<TrackEncoder> track_encoder = track_encoders_.valueFor(track_id);
  assert(track_encoder.get() != NULL);

  auto ret = track_encoder->SetParams(param_type, param, param_size);
  // Initial debug purpose.
  assert(ret == NO_ERROR);

  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: track_id(%x) TrackEncoder SetParams failed!", TAG,
        __func__, track_id);
    return ret;
  }

  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t EncoderCore::DeleteTrackEncoder(uint32_t track_id) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, track_id);
  QMMF_KPI_DETAIL();

  if (!isTrackValid(track_id)) {
    QMMF_ERROR("%s:%s: Invalid track_id(%x)", TAG, __func__, track_id);
    return BAD_VALUE;
  }
  shared_ptr<TrackEncoder> track_encoder = track_encoders_.valueFor(track_id);
  assert(track_encoder.get() != nullptr);

  auto ret = track_encoder->ReleaseHeaders();
  assert(ret == NO_ERROR);

  track_encoders_.removeItem(track_id);

  QMMF_INFO("%s:%s: track_id(%x) TrackEncoder Deleted Successfully!", TAG,
      __func__, track_id);
  QMMF_DEBUG("%s:%s: Exit", TAG, __func__);
  return NO_ERROR;

}

status_t EncoderCore::ReturnTrackBuffer(const uint32_t track_id,
                                        std::vector<BnBuffer> &buffers) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, track_id);

  if (!isTrackValid(track_id)) {
    QMMF_ERROR("%s:%s: Invalid track_id(%x)", TAG, __func__, track_id);
    return BAD_VALUE;
  }
  shared_ptr<TrackEncoder> track_encoder = track_encoders_.valueFor(track_id);
  assert(track_encoder.get() != nullptr);

  // Return buffer back to track encoder's output bitstream buffer queue.
  auto ret = track_encoder->OnBufferReturnFromClient(buffers);

  QMMF_DEBUG("%s:%s: Exit track_id(%x)", TAG, __func__, track_id);
  return ret;
}

bool EncoderCore::isTrackValid(uint32_t track_id) {

  QMMF_DEBUG("%s: Number of Tracks exist = %d",__func__,
      track_encoders_.size());
  assert(track_encoders_.size() > 0);
  return track_encoders_.indexOfKey(track_id) >= 0 ? true : false;
}

TrackEncoder::TrackEncoder(int32_t ion_device)
    : ion_device_(ion_device),
      eos_atoutput_(false),
      is_force_cleanup_(false),
      num_bytes_(0),
      prevtv_{0, 0},
      count_(0) {

  QMMF_INFO("%s:%s: Enter", TAG, __func__);

  char prop_val[PROPERTY_VALUE_MAX];
  property_get(PROP_DEBUG_FPS, prop_val, "1");
  debug_fps_ = atoi(prop_val);

  memset(&track_params_, 0x0, sizeof track_params_);
  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

TrackEncoder::~TrackEncoder() {

  QMMF_INFO("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());

  int i = 0;
  for(auto& iter : output_buffer_list_) {

    if((iter).data) {
        munmap((iter).data, (iter).capacity);
        (iter).data = NULL;
    }
    if((iter).fd) {
        QMMF_INFO("%s:%s track_id(%x) (iter).fd =%d Free", TAG, __func__,
                                   TrackId(), (iter).fd);
        ioctl(ion_device_, ION_IOC_FREE, &(output_ion_list_[i]));
        close((iter).fd);
        (iter).fd = 0;
    }
    ++i;
  }
  output_buffer_list_.clear();
  output_ion_list_.clear();

  if(avcodec_ != nullptr)
    delete avcodec_;

  QMMF_INFO("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

status_t TrackEncoder::Init(const shared_ptr<TrackSource>& track_source,
                            const shared_ptr<TrackEncoder>& track_encoder,
                            VideoTrackParams& track_params) {

  QMMF_INFO("%s:%s: Enter track_id(%x)", TAG, __func__, track_params.track_id);
  track_params_ = track_params;

  avcodec_ = new AVCodec();
  if(avcodec_ == nullptr) {
    QMMF_ERROR("%s:%s: track_id(%x) AVCodec failed", TAG, __func__,
        track_params.track_id);
    return NO_MEMORY;
  }

  CodecParam codec_param;
  memset(&codec_param, 0x0, sizeof(codec_param));

  codec_param.video_enc_param = track_params.params;

  QMMF_INFO("%s:%s: track_id(%x) W(%d) H(%d) format_type(%d)", TAG, __func__,
      track_params.track_id, track_params.params.width,
      track_params.params.height, track_params.params.format_type);

  auto ret = avcodec_->ConfigureCodec(CodecMimeType::kMimeTypeVideoEncAVC,
                                      codec_param);
  assert(ret == NO_ERROR);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s track_id(%x) Failed to configure AVCodec!", TAG, __func__,
        track_params.track_id);
    return ret;
  }

  // TODO: Modify UseBuffer Api to take sp pointer as a reference.
  vector<BufferDescriptor> dummy_list;
  ret = avcodec_->AllocateBuffer(kPortIndexInput, 0, 0,
                                 shared_ptr<ICodecSource>(track_source),
                                 dummy_list);
  assert(ret == NO_ERROR);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s track_id(%x) AllocateBuffer Failed at input port!", TAG,
        __func__, track_params.track_id);
    return ret;
  }

  //Output port configuration
  ret = AllocOutputPortBufs();
  assert(ret == NO_ERROR);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s track_id(%x) output buffer allocation failed!!", TAG,
        __func__, track_params.track_id);
    return ret;
  }

  ret = avcodec_->RegisterOutputBuffers(output_buffer_list_);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s track_id(%x) output buffers failed to register to AVCodec",
               TAG, __func__, track_params.track_id);
    return ret;
  }

  ret = avcodec_->AllocateBuffer(kPortIndexOutput, 0, 0,
                                 shared_ptr<ICodecSource>(track_encoder),
                                 dummy_list);
  if(ret != NO_ERROR) {
    QMMF_ERROR("%s:%s track_id(%x) AllocateBuffer Failed at output port!", TAG,
        __func__, track_params.track_id);
    // TODO: Deallocate ouput port buffers.
    //ReleaseBuffer();
    return ret;
  }

  QMMF_INFO("%s:%s: track_id(%x) AVCodec(0x%p) Instantiated!" , TAG, __func__,
      track_params.track_id, avcodec_);

  for(auto& iter : output_buffer_list_) {
      QMMF_INFO("%s:%s: track_id(%x) Adding buffer fd(%d) to "
          "output_free_buffer_queue list", TAG, __func__, track_params.track_id,
          iter.fd);
      output_free_buffer_queue_.PushBack(iter);
  }

#ifdef DUMP_BITSTREAM
  String8 bitstream_filepath;
  VideoFormat fmt_type = track_params.params.format_type;
  const char* type_string = (fmt_type == VideoFormat::kAVC) ? "h264" : "h265";
  String8 extension(type_string);
  bitstream_filepath.appendFormat(FRAME_DUMP_PATH"/track_enc_%x.%s",
      track_params.track_id, type_string);
  file_fd_ = open(bitstream_filepath.string(), O_CREAT | O_WRONLY | O_TRUNC,
       0655);
#endif
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t TrackEncoder::Start() {

  QMMF_INFO("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());

  assert(avcodec_ != nullptr);
  auto ret = avcodec_->StartCodec();
  // Initial debug purpose.
  assert(ret == NO_ERROR);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: track_id(%x) StartCodec failed!", TAG, __func__,
        TrackId());
    return ret;
  }

  eos_atoutput_ = false;

  QMMF_INFO("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackEncoder::Stop(bool is_force_cleanup) {

  QMMF_INFO("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  if (is_force_cleanup) {
    QMMF_INFO("%s:%s track_id(%x) Force cleanup", TAG, __func__, TrackId());
    std::lock_guard<std::mutex> lock(queue_lock_);
    is_force_cleanup_ = true;
    List<BufferDescriptor>::iterator it = output_occupy_buffer_queue_.Begin();
    for (; it != output_occupy_buffer_queue_.End(); ++it) {
      output_free_buffer_queue_.PushBack(*it);
      output_occupy_buffer_queue_.Erase(it);
      wait_for_frame_.notify_one();
    }
  }
  assert(avcodec_ != nullptr);
  auto ret = avcodec_->StopCodec();
  // Initial debug purpose.
  assert(ret == NO_ERROR);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: track_id(%x) StopCodec failed!", TAG, __func__,
        TrackId());
    return ret;
  }

  QMMF_INFO("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackEncoder::SetParams(CodecParamType param_type, void* param,
                                 uint32_t param_size) {

  QMMF_INFO("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  assert(avcodec_ != nullptr);
  auto ret = avcodec_->SetParameters(param_type, param, param_size);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: set parameter failed for track(%x)", TAG, __func__,
        TrackId());
  }
  QMMF_INFO("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return ret;
}


status_t TrackEncoder::ReleaseHeaders() {

  QMMF_INFO("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  assert(avcodec_ != nullptr);
  auto ret = avcodec_->ReleaseBuffer();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: ReleaseBuffer failed!", TAG, __func__);
  }
  QMMF_INFO("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return ret;
}

status_t TrackEncoder::SynchronizeCache(
    const struct ion_handle_data& ion_handle,
    const BufferDescriptor& buffer,
    const unsigned int flag) {
  QMMF_DEBUG("%s:%s Enter track_id(%d)", TAG, __func__, TrackId());

  struct ion_flush_data flush_data;
  struct ion_custom_data custom_data;

  memset(&flush_data, 0x0, sizeof(flush_data));
  memset(&custom_data, 0x0, sizeof(custom_data));

  flush_data.vaddr = buffer.data;
  flush_data.fd = buffer.fd;
  flush_data.handle = ion_handle.handle;
  flush_data.length = buffer.capacity;
  custom_data.cmd = flag;
  custom_data.arg = reinterpret_cast<unsigned long>(&flush_data);
  QMMF_DEBUG("Cache %s: fd=%d handle=%d va=%p size=%d",
      (flag == ION_IOC_CLEAN_CACHES) ? "Clean" : "Invalidate", flush_data.fd,
      flush_data.handle, flush_data.vaddr, flush_data.length);
  auto ret = ioctl(ion_device_, ION_IOC_CUSTOM, &custom_data);
  if (ret < 0) {
    QMMF_ERROR("%s:%s Cache %s failed", TAG, __func__,
        (flag == ION_IOC_CLEAN_CACHES) ? "Clean" : "Invalidate");
    return ret;
  }
  QMMF_DEBUG("%s:%s Exit track_id(%d)", TAG, __func__, TrackId());
  return NO_ERROR;
}

status_t TrackEncoder::GetBuffer(BufferDescriptor& codec_buffer,
                                 void* client_data) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());

  // Give available free buffer to encoder to use on output port.
  while (output_free_buffer_queue_.Size() <= 0) {
    QMMF_DEBUG("%s:%s track_id(%x) No buffer available to notify,"
      " Wait for new buffer", TAG, __func__, TrackId());
    std::unique_lock<std::mutex> lock(queue_lock_);
    wait_for_frame_.wait(lock);
    //TODO: change simple wait to relative wait.
  }

  BufferDescriptor iter = *output_free_buffer_queue_.Begin();

  auto ret = SynchronizeCache(fd_ion_handle_map_[iter.fd], iter,
                              ION_IOC_CLEAN_CACHES);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s Cache Synchronization Failed with error %d", TAG,
        __func__, ret);
    return ret;
  }

  codec_buffer.fd = (iter).fd;
  codec_buffer.data = (iter).data;

  output_free_buffer_queue_.Erase(output_free_buffer_queue_.Begin());
  {
    std::lock_guard<std::mutex> lock(queue_lock_);
    output_occupy_buffer_queue_.PushBack(iter);
  }
  QMMF_DEBUG("%s:%s track_id(%x) Sending buffer(0x%p) fd(%d) for FTB", TAG,
      __func__, TrackId(), codec_buffer.data, codec_buffer.fd);

  QMMF_DEBUG("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return NO_ERROR;
}

status_t TrackEncoder::ReturnBuffer(BufferDescriptor& codec_buffer,
                                    void* client_data) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  assert(codec_buffer.data != NULL);

  QMMF_VERBOSE("%s:%s: track_id(%x) Received buffer(0x%p) from FBD", TAG,
      __func__, TrackId(), codec_buffer.data);

  if (debug_fps_ & kDebugTrackFps) {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    uint64_t time_diff = (uint64_t)((tv.tv_sec * 1000000 + tv.tv_usec) -
        (prevtv_.tv_sec * 1000000 + prevtv_.tv_usec));

    size_t size = codec_buffer.size;
    num_bytes_ += size;

    count_++;
    if (time_diff >= FPS_TIME_INTERVAL) {
        float framerate = (count_ * 1000000) / (float)time_diff;
        uint32_t bitrate = (num_bytes_ * 8/count_) * framerate;
        QMMF_INFO(" %s:%s: track_id(%x): encoded fps: = %0.2f bitrate=%d",
                  TAG, __func__, TrackId(), framerate, bitrate);
        prevtv_ = tv;
        count_ = 0;
        num_bytes_ = 0;
    }
  }

#ifdef DUMP_BITSTREAM
  DumpBitStream(codec_buffer);
#endif

#ifdef DONT_NOTIFY
  // This change is only for debug purpose, it will circulate buffers without
  // sending/mapping them to another process.
  List<BufferDescriptor>::iterator it = output_occupy_buffer_queue_.Begin();
  bool found = false;
  for (; it != output_occupy_buffer_queue_.End(); ++it) {
    QMMF_VERBOSE("%s:%s track_id(%x) Checking match (0x%p)vs(0x%p) ", TAG,
        __func__, TrackId(), (*it).data,  codec_buffer.data);
    if (((*it).data) == (codec_buffer.data)) {
      QMMF_VERBOSE("%s:%s track_id(%x) Buffer found", TAG, __func__, TrackId());
      output_free_buffer_queue_.PushBack(*it);
      output_occupy_buffer_queue_.Erase(it);
      wait_for_frame_.notify_one();
      found = true;
      break;
    }
  }
  assert(found == true);
#else
  if (eos_atoutput_ == true) {
    //  If EOS happend on output port then don't notify buffers to application.
    //  simply remove the buffer from output queue in input queue, note last
    //  buffer with EOS is already notified to application before setting
    //  eos_atoutput_ to true.
    {
      std::lock_guard<std::mutex> lock(queue_lock_);
      List<BufferDescriptor>::iterator it = output_occupy_buffer_queue_.Begin();
      for (; it != output_occupy_buffer_queue_.End(); ++it) {
        if (((*it).data) == (codec_buffer.data)) {
          QMMF_INFO("%s:%s track_id(%x) EOS is already done! moving buffer from"
              " Out to In queue!", TAG, __func__, TrackId());
          output_free_buffer_queue_.PushBack(*it);
          output_occupy_buffer_queue_.Erase(it);
          break;
        }
      }
    }
  } else {
    NotifyBufferToClient(codec_buffer);
  }
#endif

  QMMF_DEBUG("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return NO_ERROR;
}

status_t TrackEncoder::NotifyPortEvent(PortEventType event_type,
                                       void* event_data) {

  QMMF_DEBUG("%s:%s Enter", TAG, __func__);
  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
  return 0;
}

status_t TrackEncoder::OnBufferReturnFromClient(std::vector<BnBuffer>
                                                &bn_buffers) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  int32_t ret = NO_ERROR;

  //Buffer came back from client, now put this buffer in free queue.
  QMMF_DEBUG("%s:%s: track_id(%x) Number of buffers(%d) returned from client",
      TAG, __func__, TrackId(), bn_buffers.size());

  assert(output_occupy_buffer_queue_.Size() > 0);

  for (auto& iter : bn_buffers) {
    bool match = false;
    QMMF_DEBUG("%s:%s track_id(%x) output_occupy_buffer_queue_.size(%d)",
        TAG, __func__, TrackId(), output_occupy_buffer_queue_.Size());
    {
      std::lock_guard<std::mutex> lock(queue_lock_);
      List<BufferDescriptor>::iterator it = output_occupy_buffer_queue_.Begin();
      for (; it != output_occupy_buffer_queue_.End(); ++it) {
        if ((*it).fd == static_cast<int32_t>(iter.buffer_id)) {
          QMMF_DEBUG("%s:%s: track_id(%x) buffer_id(%d) found in list", TAG,
              __func__, TrackId(), iter.buffer_id);
          // Move buffer to free queue, and signal AVCodec's output thread if it
          // is waiting for buffer.
          output_free_buffer_queue_.PushBack((*it));
          // Erase buffer from occupy queue.
          output_occupy_buffer_queue_.Erase(it);
          wait_for_frame_.notify_one();
          match = true;
          break;
        }
      }
      // Make sure all buffers are part of occupy queue.
      assert(match == true);
      QMMF_DEBUG("%s:%s track_id(%x) output_occupy_buffer_queue_.size(%d)", TAG,
          __func__, TrackId(), output_occupy_buffer_queue_.Size());
      QMMF_DEBUG("%s:%s track_id(%x) output_free_buffer_queue_.size(%d)", TAG,
          __func__, TrackId(), output_free_buffer_queue_.Size());
    }
  }
  QMMF_DEBUG("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
  return ret;
}

void TrackEncoder::NotifyBufferToClient(BufferDescriptor& codec_buffer) {

  QMMF_DEBUG("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  assert(track_params_.data_cb != nullptr);

  bool found = false;
  BnBuffer bn_buffer;
  memset(&bn_buffer, 0x0, sizeof bn_buffer);

  if(codec_buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS)) {
    eos_atoutput_ = true;
    QMMF_INFO("%s:%s: EOS is received for track(%x)", TAG, __func__, TrackId());
  }

  {
    std::lock_guard<std::mutex> lock(queue_lock_);
    if (is_force_cleanup_) {
      QMMF_WARN("%s:%s: Force cleanup is triggered! client may not exist!",
        TAG, __func__);
      return;
    }
    List<BufferDescriptor>::iterator it = output_occupy_buffer_queue_.Begin();
    for (; it != output_occupy_buffer_queue_.End(); ++it) {
      QMMF_VERBOSE("%s:%s track_id(%x) Checking match (0x%p) vs (0x%p) ", TAG,
          __func__, TrackId(), (*it).data,  codec_buffer.data);
      if (((*it).data) ==  (codec_buffer.data)) {
        QMMF_VERBOSE("%s:%s track_id(%x) fd(%d):size(%d):timestamp(%lld):"
            "capacity(%d)", TAG, __func__, TrackId(), (*it).fd,
            codec_buffer.size, codec_buffer.timestamp, (*it).capacity);
        bn_buffer.ion_fd    = (*it).fd;
        bn_buffer.size      = codec_buffer.size;
        bn_buffer.timestamp = codec_buffer.timestamp;
        bn_buffer.width     = -1;
        bn_buffer.height    = -1;
        bn_buffer.buffer_id = (*it).fd;
        bn_buffer.flag      = codec_buffer.flag;
        bn_buffer.capacity  = (*it).capacity;
        found = true;
        break;
      }
    }
  }
  assert(found == true);
  std::vector<BnBuffer> bn_buffers;
  bn_buffers.push_back(bn_buffer);

  MetaData meta_data;
  memset(&meta_data, 0x0, sizeof meta_data);
  meta_data.meta_flag = static_cast<uint32_t>(MetaParamType::kVideoFrameType);

  if (codec_buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagIDRFrame))
    meta_data.video_frame_type_info |=
        static_cast<uint32_t>(BufferFlags::kFlagIDRFrame);

  if (codec_buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagIFrame))
    meta_data.video_frame_type_info |=
        static_cast<uint32_t>(BufferFlags::kFlagIFrame);

  if (codec_buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagPFrame))
    meta_data.video_frame_type_info =
        static_cast<uint32_t>(BufferFlags::kFlagPFrame);

  if (codec_buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagBFrame))
    meta_data.video_frame_type_info =
        static_cast<uint32_t>(BufferFlags::kFlagBFrame);

  QMMF_VERBOSE("%s: Video Frame Type: %u\n", __func__,
      static_cast<uint32_t>(meta_data.video_frame_type_info));

  std::vector<MetaData> meta_buffers;
  meta_buffers.push_back(meta_data);

  track_params_.data_cb(bn_buffers, meta_buffers);

  QMMF_DEBUG("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());

}

status_t TrackEncoder::AllocOutputPortBufs() {

  QMMF_INFO("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  int32_t ret = 0;
  uint32_t count, size;

  assert(avcodec_ != nullptr);
  ret = avcodec_->GetBufferRequirements(kPortIndexOutput,  &count, &size);
  assert(ret == NO_ERROR);

  assert(ion_device_ >= 0);
  int32_t ion_type = 0x1 << ION_IOMMU_HEAP_ID;
  void *vaddr      = NULL;

  struct ion_allocation_data alloc;
  struct ion_fd_data         ion_fddata;

  for(uint32_t i = 0; i < count; i++) {

    BufferDescriptor buffer;
    struct ion_handle_data ionHandleData;
    vaddr = NULL;
    memset(&buffer, 0x0, sizeof(buffer));
    memset(&alloc, 0x0, sizeof(ion_allocation_data));
    memset(&ion_fddata, 0x0, sizeof(ion_fddata));
    memset(&ionHandleData, 0x0, sizeof(ionHandleData));

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

    ionHandleData.handle = ion_fddata.handle;
    output_ion_list_.push_back(ionHandleData);

    buffer.fd       = ion_fddata.fd;
    buffer.capacity = alloc.len;
    buffer.data     = vaddr;

    QMMF_INFO("%s:%s buffer.Fd(%d)", TAG, __func__, buffer.fd);
    QMMF_DEBUG("%s:%s buffer.capacity(%d)", TAG, __func__, buffer.capacity);
    QMMF_DEBUG("%s:%s buffer.vaddr(%p)", TAG, __func__, buffer.data);

    output_buffer_list_.push_back(buffer);
    fd_ion_handle_map_.insert(::std::make_pair(buffer.fd, ionHandleData));
  }

  QMMF_INFO("%s:%s: Exit track_id(%x)", TAG, __func__, TrackId());
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

#ifdef DUMP_BITSTREAM
void TrackEncoder::DumpBitStream(BufferDescriptor& codec_buffer) {

  QMMF_VERBOSE("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
  if(eos_atoutput_) {
    return;
  }
  if(file_fd_ > 0) {
    ssize_t exp_size = (ssize_t) codec_buffer.size;
    QMMF_INFO("%s:%s Got encoded buffer of size(%d)", TAG, __func__,
        codec_buffer.size);

    if (exp_size != write(file_fd_, codec_buffer.data, codec_buffer.size)) {
      QMMF_INFO("%s:%s: Bad Write error (%d) %s", TAG, __func__, errno,
          strerror(errno));
      close(file_fd_);
      file_fd_ = -1;
    }
  } else {
    QMMF_ERROR("%s:%s File is not open fd = %d", TAG, __func__, file_fd_);
  }

  if (codec_buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS)) {
    QMMF_ERROR("%s:%s This is last buffer from encoder.close file", TAG,
        __func__);

    close(file_fd_);
    file_fd_ = -1;
  }
  QMMF_VERBOSE("%s:%s: Enter track_id(%x)", TAG, __func__, TrackId());
}
#endif

};  // namespace recorder

};  // namespace qmmf
