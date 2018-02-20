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


#define LOG_TAG "VideoDecoderCore"

#include <memory>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/msm_ion.h>

#include "player/src/service/qmmf_player_video_decoder_core.h"

namespace qmmf {
namespace player {

using ::qmmf::avcodec::AVCodec;
using ::qmmf::avcodec::CodecBuffer;
using ::qmmf::avcodec::CodecParam;
using ::qmmf::avcodec::CodecPortStatus;
using ::qmmf::avcodec::kPortIndexInput;
using ::qmmf::avcodec::kPortIndexOutput;
using ::qmmf::avcodec::kPortALL;
using ::qmmf::avcodec::PortreconfigData;
using ::qmmf::avcodec::PortEventType;
using ::std::make_shared;
using ::std::shared_ptr;
using ::std::vector;

VideoDecoderCore* VideoDecoderCore::instance_ = nullptr;

VideoDecoderCore* VideoDecoderCore::CreateVideoDecoderCore() {
  if (!instance_) {
     instance_ = new VideoDecoderCore();
  if (!instance_) {
    QMMF_ERROR("%s: Can't Create VideoDecoderCore Instance", __func__);
    return nullptr;
  }
  }
  QMMF_INFO("%s: VideoDecoderCore Instance Created Successfully(0x%p)",
       __func__, instance_);

  return instance_;
}

VideoDecoderCore::VideoDecoderCore(): ion_device_(-1) {
  QMMF_INFO("%s: Enter", __func__);
  QMMF_INFO("%s: Exit", __func__);
}

VideoDecoderCore::~VideoDecoderCore()
{
  QMMF_INFO("%s: Enter", __func__);
  if (!video_track_decoders_.isEmpty()) {
    video_track_decoders_.clear();
  }
  instance_ = NULL;

  if (ion_device_ > 0) {
    close(ion_device_);
    ion_device_ = -1;
  }
  QMMF_INFO("%s: Exit", __func__);
}

status_t VideoDecoderCore::CreateVideoTrack(VideoTrackParams& params) {
   QMMF_DEBUG("%s: Enter", __func__);

   status_t ret = NO_ERROR;

  if (ion_device_ < 0) {
    ion_device_ = open("/dev/ion", O_RDONLY);
    assert(ion_device_ >=0 );
  }

  shared_ptr<VideoTrackDecoder> video_track_decoder =
      make_shared<VideoTrackDecoder>(ion_device_);
  if (!video_track_decoder.get()) {
    QMMF_ERROR("%s: track_id(%d) Can't instantiate VideoTrackEncoder",
        __func__, params.track_id);
    return NO_MEMORY;
  }

  ret = video_track_decoder->ConfigureTrackDecoder(params);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: track_id(%d) VideoTrackDecoder Init failed!",
        __func__, params.track_id);
    return BAD_VALUE;
  }

  video_track_decoders_.add(params.track_id, video_track_decoder);

  QMMF_INFO("%s: VideoTrackDecoder(0x%p) for track_id(%d) Instantiated!",
      __func__, video_track_decoder.get(), params.track_id);

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t VideoDecoderCore::PrepareTrackPipeline(
    uint32_t track_id,
    const shared_ptr<VideoTrackSink>& video_track_sink) {
  QMMF_DEBUG("%s: Enter track_id(%d)", __func__, track_id);

  if (!isTrackValid(track_id)) {
    QMMF_ERROR("%s: Invalid track_id(%d)", __func__, track_id);
    return BAD_VALUE;
  }

   shared_ptr<VideoTrackDecoder> track_decoder =
      video_track_decoders_.valueFor(track_id);
   assert(track_decoder.get() != NULL);

   auto ret = track_decoder->PreparePipeline(video_track_sink, track_decoder);
   if (ret != NO_ERROR) {
   QMMF_INFO("%s: track_id(%d) PreparePipeline failed!", __func__,
     track_id);
   return ret;
  }

  QMMF_INFO("%s: track_id(%d) PreparePipeline Successful!",
     __func__, track_id);
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t VideoDecoderCore::DequeueTrackInputBuffer(
    uint32_t track_id,
    std::vector<AVCodecBuffer>& buffers) {
  QMMF_DEBUG("%s: Enter track_id(%d)", __func__, track_id);

  if (!isTrackValid(track_id)) {
   QMMF_ERROR("%s: Invalid track_id(%d)", __func__, track_id);
   return BAD_VALUE;
  }

  shared_ptr<VideoTrackDecoder> track_decoder =
      video_track_decoders_.valueFor(track_id);
  assert(track_decoder.get() != NULL);

  auto ret = track_decoder->DequeueInputBuffer(buffers);
  if (ret != NO_ERROR) {
   QMMF_INFO("%s: track_id(%d) DequeueInputBuffer failed!", __func__,
     track_id);
   return ret;
  }

  QMMF_INFO("%s: track_id(%d) DequeueInputBuffer Successful!",
     __func__, track_id);
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t VideoDecoderCore::QueueTrackInputBuffer(
    uint32_t track_id,
    std::vector<AVCodecBuffer>& buffers) {
  QMMF_DEBUG("%s: Enter track_id(%d)", __func__, track_id);

  if (!isTrackValid(track_id)) {
    QMMF_ERROR("%s: Invalid track_id(%d)", __func__, track_id);
    return BAD_VALUE;
  }

   shared_ptr<VideoTrackDecoder> track_decoder =
      video_track_decoders_.valueFor(track_id);
   assert(track_decoder.get() != NULL);

   auto ret = track_decoder->QueueInputBuffer(buffers);
   if (ret != NO_ERROR) {
    QMMF_INFO("%s: track_id(%d) QueueInputBuffer failed!", __func__,
      track_id);
    return ret;
   }

   QMMF_INFO("%s: track_id(%d) QueueInputBuffer Successful!",
      __func__, track_id);
   QMMF_DEBUG("%s: Exit", __func__);
   return ret;
}

status_t VideoDecoderCore::StartTrackDecoder(uint32_t track_id) {
  QMMF_DEBUG("%s: Enter track_id(%d)", __func__, track_id);

  if (!isTrackValid(track_id)) {
   QMMF_ERROR("%s: Invalid track_id(%d)", __func__, track_id);
   return BAD_VALUE;
  }

  shared_ptr<VideoTrackDecoder> track_decoder =
      video_track_decoders_.valueFor(track_id);
  assert(track_decoder.get() != NULL);

  auto ret = track_decoder->StartDecoder();
  if (ret != NO_ERROR) {
   QMMF_INFO("%s: track_id(%d) StartDecoder failed!", __func__,
     track_id);
   return ret;
  }

  QMMF_INFO("%s: track_id(%d) StartDecoder Successful!",
     __func__, track_id);
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t VideoDecoderCore::StopTrackDecoder(uint32_t track_id,
                                            const PictureParam& params,
                                            BufferDescriptor* grab_buffer) {
  QMMF_DEBUG("%s: Enter track_id(%d)", __func__, track_id);

  if (!isTrackValid(track_id)) {
   QMMF_ERROR("%s: Invalid track_id(%d)", __func__, track_id);
   return BAD_VALUE;
  }

  shared_ptr<VideoTrackDecoder> track_decoder =
      video_track_decoders_.valueFor(track_id);
  assert(track_decoder.get() != NULL);

  auto ret = track_decoder->StopDecoder(params, grab_buffer);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s: track_id(%d) StopDecoder failed!", __func__,
      track_id);
    return ret;
  }

  QMMF_INFO("%s: track_id(%d) StopDecoder Successful!",
      __func__, track_id);
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t VideoDecoderCore::PauseTrackDecoder(uint32_t track_id,
                                             const PictureParam& params,
                                             BufferDescriptor* grab_buffer) {
  QMMF_DEBUG("%s: Enter track_id(%d)", __func__, track_id);

  if (!isTrackValid(track_id)) {
    QMMF_ERROR("%s: Invalid track_id(%d)", __func__, track_id);
    return BAD_VALUE;
  }

  shared_ptr<VideoTrackDecoder> track_decoder =
      video_track_decoders_.valueFor(track_id);
  assert(track_decoder.get() != NULL);

  auto ret = track_decoder->PauseDecoder(params, grab_buffer);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s: track_id(%d) PauseDecoder failed!", __func__,
     track_id);
    return ret;
  }

  QMMF_INFO("%s: track_id(%d) PauseDecoder Successful!",
     __func__, track_id);
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t VideoDecoderCore::ResumeTrackDecoder(uint32_t track_id) {
  QMMF_DEBUG("%s: Enter track_id(%d)", __func__, track_id);

  if (!isTrackValid(track_id)) {
    QMMF_ERROR("%s: Invalid track_id(%d)", __func__, track_id);
    return BAD_VALUE;
  }

  shared_ptr<VideoTrackDecoder> track_decoder =
      video_track_decoders_.valueFor(track_id);
  assert(track_decoder.get() != NULL);

  auto ret = track_decoder->ResumeDecoder();
  if (ret != NO_ERROR) {
    QMMF_INFO("%s: track_id(%d) ResumeDecoder failed!", __func__,
     track_id);
   return ret;
  }

  QMMF_INFO("%s: track_id(%d) ResumeDecoder Successful!",
     __func__, track_id);
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t VideoDecoderCore::SetVideoTrackDecoderParams(
    uint32_t track_id,
    CodecParamType param_type,
    void* param,
    uint32_t param_size) {
  QMMF_DEBUG("%s: Enter track_id(%d)", __func__, track_id);

  if (!isTrackValid(track_id)) {
    QMMF_ERROR("%s: Invalid track_id(%d)", __func__, track_id);
    return BAD_VALUE;
  }

  shared_ptr<VideoTrackDecoder> track_decoder =
      video_track_decoders_.valueFor(track_id);
  assert(track_decoder.get() != NULL);

  auto ret =  track_decoder->SetVideoDecoderParams(param_type, param, param_size);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s: track_id(%d) SetVideoDecoderParams failed!", __func__,
     track_id);
    return ret;
  }

  QMMF_INFO("%s: track_id(%d) SetVideoDecoderParams Successful!",
     __func__, track_id);
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t VideoDecoderCore::DeleteTrackDecoder(uint32_t track_id) {
  QMMF_DEBUG("%s: Enter track_id(%d)", __func__, track_id);

  if (!isTrackValid(track_id)) {
     QMMF_ERROR("%s: Invalid track_id(%d)", __func__, track_id);
     return BAD_VALUE;
  }

  shared_ptr<VideoTrackDecoder> track_decoder =
      video_track_decoders_.valueFor(track_id);
  assert(track_decoder.get() != NULL);

  auto ret = track_decoder->DeleteDecoder();
  if (ret != NO_ERROR) {
   QMMF_INFO("%s: track_id(%d) DeleteDecoder failed!", __func__,
     track_id);
   return ret;
  }

  video_track_decoders_.removeItem(track_id);

  QMMF_INFO("%s: track_id(%d) DeleteDecoder Successful!",
     __func__, track_id);
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t VideoDecoderCore::SetTrackTrickMode(uint32_t track_id,
                                             TrickModeSpeed speed,
                                             TrickModeDirection dir) {
  QMMF_DEBUG("%s: Enter track_id(%d)", __func__, track_id);

  if (!isTrackValid(track_id)) {
    QMMF_ERROR("%s: Invalid track_id(%d)", __func__, track_id);
    return BAD_VALUE;
  }

  shared_ptr<VideoTrackDecoder> track_decoder =
      video_track_decoders_.valueFor(track_id);
  assert(track_decoder.get() != NULL);

  auto ret =  track_decoder->SetTrickMode(speed, dir);
  if (ret != NO_ERROR) {
    QMMF_INFO("%s: track_id(%d) SetTrackTrickMode failed!", __func__,
     track_id);
    return ret;
  }

  QMMF_INFO("%s: track_id(%d) SetTrackTrickMode Successful!",
     __func__, track_id);
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

bool VideoDecoderCore::isTrackValid(uint32_t track_id) {
  QMMF_INFO("%s: Number of Tracks exist = %d",__func__,
      video_track_decoders_.size());
  assert(video_track_decoders_.size() > 0);
  return video_track_decoders_.indexOfKey(track_id) >= 0 ? true : false;
}


/************************* Video Decoding ********************************/

VideoTrackDecoder::VideoTrackDecoder(int32_t ion_device)
    : output_buffer_count_(0),
      output_buffer_size_(0),
      ion_device_(ion_device),
      stop_received_(false) {
  QMMF_INFO("%s: Enter", __func__);

  memset(&video_track_params_, 0x0, sizeof video_track_params_);

#ifdef DUMP_VIDEO_BITSTREAM
   file_fd_video_ = open("/data/misc/qmmf/video_track.bitstream",
       O_CREAT | O_WRONLY | O_TRUNC, 0655);
#endif

  player_decode_profile_ = GetPlayerDecodeProfileProperty();

  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

VideoTrackDecoder::~VideoTrackDecoder() {
  QMMF_INFO("%s: Enter track_id(%d)", __func__, TrackId());

   uint32_t i = 0;
   for (auto& iter : input_buffer_list_) {
      if ((iter).data) {
        munmap((iter).data, (iter).frame_length);
        (iter).data = nullptr;
      }
      if ((iter).fd) {
        QMMF_INFO("%s track_id(%d) (iter).fd =%d Free", __func__,
                                   TrackId(), (iter).fd);
        ioctl(ion_device_, ION_IOC_FREE, &(ion_handle_data[i]));
        close((iter).fd);
        (iter).fd = -1;
      }
      i++;
  }

  input_buffer_list_.clear();
  ion_handle_data.clear();

  for (auto& iter : output_buffer_list_) {

    if ((iter).pointer) {
        munmap((iter).pointer, (iter).frame_length);
        (iter).pointer = NULL;
    }
    if ((iter).fd) {
        QMMF_INFO("%s track_id(%d) (iter).fd =%d Free", __func__,
                                   TrackId(), (iter).fd);
        ioctl(ion_device_, ION_IOC_FREE, &((iter).handle_data.handle));
        close((iter).fd);
        (iter).fd = 0;
    }
  }

  output_buffer_list_.clear();

#ifdef DUMP_VIDEO_BITSTREAM
    close(file_fd_video_);
#endif

  delete avcodec_;

  QMMF_DEBUG("%s: Exit (0x%p)", __func__, this);
}

status_t VideoTrackDecoder::ConfigureTrackDecoder(
    VideoTrackParams& track_params) {
  QMMF_DEBUG("%s: Enter track_id(%d)", __func__, track_params.track_id);
  video_track_params_ = track_params;

  avcodec_ = new AVCodec();

  status_t ret = NO_ERROR;

  CodecParam codec_param;
  memset(&codec_param, 0x0, sizeof(codec_param));
  codec_param.video_dec_param = track_params.params;
  ret = avcodec_->ConfigureCodec(CodecMimeType::kMimeTypeVideoDecAVC,
                                 codec_param);
  assert(ret == NO_ERROR);
  if (ret != NO_ERROR) {
  QMMF_ERROR("%s track_id(%d) Failed to configure AVCodec!", __func__,
      track_params.track_id);
  return ret;
  }

  QMMF_INFO("%s: Exit track_id(%d)", __func__, TrackId());
  return ret;
}

status_t VideoTrackDecoder::PreparePipeline(
    const shared_ptr<VideoTrackSink>& video_track_sink,
    const shared_ptr<VideoTrackDecoder>& video_track_decoder) {
  QMMF_DEBUG("%s: Enter track_id(%d)", __func__, TrackId());
  assert(avcodec_ != nullptr);

  status_t ret = NO_ERROR;

  video_track_sink_ = video_track_sink;

  //This function will get the port buffer requirment and will allocate buffer
  AllocInputPortBufs();

  vector<BufferDescriptor> dummy_list;
  ret = avcodec_->AllocateBuffer(kPortIndexInput, 0, 0,
                                 shared_ptr<ICodecSource>(video_track_decoder),
                                 dummy_list);
  assert(ret == NO_ERROR);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s track_id(%d) AllocateBuffer Failed at input port!",
               __func__, TrackId());
  }

  std::vector<BufferDescriptor> temp_in;
  for (auto& iter: input_buffer_list_) {
    BufferDescriptor temp_buffer;
    temp_buffer.data = iter.data;
    temp_in.push_back(temp_buffer);
  }
  avcodec_->RegisterInputBuffers(temp_in);

  //This function will get the port buffer requirment and will allocate buffer
  AllocOutputPortBufs();

  ret = avcodec_->AllocateBuffer(kPortIndexOutput, 0, 0,
                                 shared_ptr<ICodecSource>(video_track_sink),
                                 dummy_list);

  std::vector<BufferDescriptor> temp_out;
  for (auto& iter: output_buffer_list_) {
    BufferDescriptor temp_buffer;
    temp_buffer.fd = iter.fd;
    temp_buffer.capacity = iter.frame_length;
    temp_out.push_back(temp_buffer);
  }
  avcodec_->RegisterOutputBuffers(temp_out);

  assert(ret == NO_ERROR);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s track_id(%d) AllocateBuffer Failed at Output port!",
               __func__, TrackId());
  }

  video_track_sink->AddBufferList(output_buffer_list_);

  video_track_sink->PassTrackDecoder(video_track_decoder);

  QMMF_DEBUG("%s: Exit track_id(%d)", __func__, TrackId());
  return ret;
}

//service will send unfilled buffer fd/pointer to the application to fill it
status_t VideoTrackDecoder::DequeueInputBuffer(
    std::vector<AVCodecBuffer>& buffers) {
  QMMF_DEBUG("%s: Enter", __func__);

  int32_t size = buffers.size();

  for (int32_t i = 0; i < size; i++) {

    if (unfilled_frame_queue_.Size() <= 0) {
     QMMF_DEBUG("%s track_id(%d) No Empty buffer available", __func__,
         TrackId());
      std::unique_lock<std::mutex> lock(wait_for_empty_frame_lock_);
      wait_for_empty_frame_.Wait(lock);
    }

    StreamBuffer iter = *unfilled_frame_queue_.Begin();
    buffers[i].fd = (iter).fd;
    buffers[i].data = (iter).data;
    buffers[i].frame_length = (iter).frame_length;
    buffers[i].buf_id = (iter).fd;

    {
      std::lock_guard<std::mutex> lock(queue_lock_);
      filled_frame_queue_.PushBack(iter);
    }

    unfilled_frame_queue_.Erase(unfilled_frame_queue_.Begin());

    QMMF_DEBUG("%s track_id(%d) Sending buffer(0x%p) fd(%d) to client",
        __func__, TrackId(), (iter).data, (iter).fd);

  }
  QMMF_DEBUG("%s: Exit track_id(%d)", __func__, TrackId());
  return NO_ERROR;
}

//application will fill the data in the buffer and will send to service
status_t VideoTrackDecoder::QueueInputBuffer(
    std::vector<AVCodecBuffer>& buffers) {
  QMMF_DEBUG("%s: Enter", __func__);

  int32_t size = buffers.size();

  for (int32_t i = 0; i < size; i++) {

    StreamBuffer iter = *filled_frame_queue_.Begin();

    QMMF_DEBUG("%s: fd from client %d" , __func__,buffers[i].buf_id);

    BufInfo bufinfo;
    memset(&bufinfo, 0x0, sizeof bufinfo);

    bufinfo = buf_info_map.valueFor(buffers[i].buf_id);

    (iter).fd                 = buffers[i].buf_id;
    (iter).data               = bufinfo.vaddr;
    (iter).filled_length      = buffers[i].filled_length;
    (iter).frame_length       = buffers[i].frame_length;
    (iter).flags              = buffers[i].flag;
    (iter).timestamp          = buffers[i].time_stamp;

    QMMF_DEBUG("%s: fd %d", __func__,buffers[i].buf_id);
    QMMF_DEBUG("%s: filled_length %d", __func__,buffers[i].filled_length);
    QMMF_DEBUG("%s: frame_length %d", __func__,buffers[i].frame_length);
    QMMF_DEBUG("%s: vaddr 0x%p", __func__,bufinfo.vaddr);
    QMMF_DEBUG("%s: flag %d", __func__,buffers[i].flag);
    QMMF_DEBUG("%s: time stamp %llu", __func__,buffers[i].time_stamp);

#ifdef DUMP_VIDEO_BITSTREAM
  write(file_fd_video_,(iter).data,(iter).filled_length );
#endif

    {
      std::lock_guard<std::mutex> lock(queue_lock_);
      frames_to_decode_.PushBack(iter);
      wait_for_frame_.Signal();
    }
    filled_frame_queue_.Erase(filled_frame_queue_.Begin());

    QMMF_DEBUG("%s track_id(%d) received buffer(0x%p) fd(%d) from client",
      __func__, TrackId(), (iter).data, (iter).fd);
  }

  QMMF_DEBUG("%s: Exit track_id(%d)", __func__, TrackId());
  return NO_ERROR;
}

status_t VideoTrackDecoder::StartDecoder() {
  QMMF_INFO("%s: Enter track_id(%d)", __func__, TrackId());
  auto ret = 0;

  assert(avcodec_ != nullptr);

  stop_received_ = true;

  // Initial debug purpose.
  ret = avcodec_->StopCodec(false);
  assert(ret == NO_ERROR);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: track_id(%d) StopCodec failed!", __func__,
        TrackId());
    return ret;
  }

  //bitstream buffer queue
  for (auto& iter : input_buffer_list_) {
    QMMF_INFO("%s: track_id(%d) Adding buffer fd(%d) to unfilled_frame_queue_",
              __func__, TrackId() , iter.fd);
    unfilled_frame_queue_.PushBack(iter);
  }

  stop_received_ = false;

  ret = avcodec_->StartCodec();
  // Initial debug purpose.
  assert(ret == NO_ERROR);
  if (ret != NO_ERROR) {
   QMMF_ERROR("%s: track_id(%d) StartCodec failed!", __func__,
       TrackId());
   return ret;
  }

  video_track_sink_->StartSink();
  // Initial debug purpose.
  assert(ret == NO_ERROR);
  if (ret != NO_ERROR) {
   QMMF_ERROR("%s: track_id(%d) StartSink failed!", __func__,
       TrackId());
   return ret;
  }

  QMMF_INFO("%s: Exit track_id(%d)", __func__, TrackId());
  return ret;
}

status_t VideoTrackDecoder::StopDecoder(const PictureParam& params,
                                        BufferDescriptor* grab_buffer) {
  QMMF_INFO("%s: Enter track_id(%d)", __func__, TrackId());
  auto ret = 0;

  assert(avcodec_ != nullptr);

  stop_received_ = true;

  ret = avcodec_->StopCodec(false);
  // Initial debug purpose.
  assert(ret == NO_ERROR);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: track_id(%d) StopCodec failed!", __func__,
        TrackId());
    return ret;
  }

  ret = video_track_sink_->StopSink(params, grab_buffer);
  // Initial debug purpose.
  assert(ret == NO_ERROR);
  if (ret != NO_ERROR) {
      QMMF_ERROR("%s: track_id(%d) StopSink failed!", __func__,
          TrackId());
    return ret;
  }

  if (!unfilled_frame_queue_.Empty())
    unfilled_frame_queue_.Clear();
  if (!filled_frame_queue_.Empty())
    filled_frame_queue_.Clear();
  if (!unfilled_frame_queue_.Empty())
    unfilled_frame_queue_.Clear();
  if (!frames_to_decode_.Empty())
    frames_to_decode_.Clear();
  if (!frames_being_decoded_.Empty())
    frames_being_decoded_.Clear();

  QMMF_INFO("%s: Exit track_id(%d)", __func__, TrackId());
  return ret;
}

status_t VideoTrackDecoder::PauseDecoder(const PictureParam& params,
                                         BufferDescriptor* grab_buffer) {
  QMMF_INFO("%s: Enter track_id(%d)", __func__, TrackId());

  auto ret = video_track_sink_->PauseSink(params, grab_buffer);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: track_id(%d) PauseSink failed!", __func__,
        TrackId());
    return ret;
  }

  assert(avcodec_ != nullptr);
  ret = avcodec_->PauseCodec();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: track_id(%d) PauseCodec failed!", __func__,
        TrackId());
    return ret;
  }

  QMMF_INFO("%s: Exit track_id(%d)", __func__, TrackId());
  return ret;
}

status_t VideoTrackDecoder::ResumeDecoder() {
  QMMF_INFO("%s: Enter track_id(%d)", __func__, TrackId());

  auto ret = video_track_sink_->ResumeSink();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: track_id(%d) ResumeSink failed!", __func__,
        TrackId());
    return ret;
  }

  assert(avcodec_ != nullptr);
  ret = avcodec_->ResumeCodec();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: track_id(%d) ResumeCodec failed!", __func__,
        TrackId());
    return ret;
  }

  QMMF_INFO("%s: Exit track_id(%d)", __func__, TrackId());
  return ret;
}

status_t VideoTrackDecoder::SetVideoDecoderParams(
    CodecParamType param_type,
    void* param,
    uint32_t param_size) {
  QMMF_INFO("%s: Enter track_id(%d)", __func__, TrackId());

  assert(avcodec_ != nullptr);
  auto ret = avcodec_->SetParameters(param_type, param, param_size);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: track_id(%d) SetParameters failed!", __func__,
        TrackId());
    return ret;
  }

  QMMF_INFO("%s: Exit track_id(%d)", __func__, TrackId());
  return ret;
}

status_t VideoTrackDecoder::DeleteDecoder()
{
  QMMF_DEBUG("%s: Enter track_id(%d)", __func__, TrackId());
  assert(avcodec_ != nullptr);

  stop_received_ = true;

  auto ret = avcodec_->StopCodec(false);
  // Initial debug purpose.
  assert(ret == NO_ERROR);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: track_id(%d) StopCodec failed!", __func__,
        TrackId());
    return ret;
  }

  ret = avcodec_->ReleaseBuffer();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: ReleaseBuffer failed!", __func__);
  }

  QMMF_DEBUG("%s: Exit track_id(%d)", __func__, TrackId());
  return ret;
}

status_t VideoTrackDecoder::SetTrickMode(TrickModeSpeed speed,
                                         TrickModeDirection dir) {
  QMMF_INFO("%s: Enter track_id(%d)", __func__, TrackId());
  QMMF_DEBUG("%s: Speed (%u) Dir (%u)", __func__,
      static_cast<uint32_t>(speed), static_cast<uint32_t>(dir));

  auto ret = video_track_sink_->SetTrickMode(speed, dir);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: track_id(%d) SetTrickMode failed!", __func__,
        TrackId());
    return ret;
  }

  uint32_t value = video_track_params_.params.frame_rate;

  if (dir == TrickModeDirection::kFastForward
      || dir == TrickModeDirection::kFastRewind) {
    value = video_track_params_.params.frame_rate * static_cast<uint32_t>(speed);
  }

  ret = SetVideoDecoderParams(CodecParamType::kDecodeOperatingRate, &value,
      sizeof(value));
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: track_id(%d) SetVideoDecoderParams failed!",
        __func__, TrackId());
    return ret;
  }

  QMMF_INFO("%s: Exit track_id(%d)", __func__, TrackId());
  return ret;
}

// This method provides an input buffer to the AVCodec
status_t VideoTrackDecoder::GetBuffer(BufferDescriptor& stream_buffer,
                                      void* client_data) {
  QMMF_DEBUG("%s: Enter track_id(%d) frames_to_decode_.Size(%d) ",
      __func__, TrackId(),frames_to_decode_.Size());

  while (frames_to_decode_.Size() <= 0 && !stop_received_) {
    QMMF_DEBUG("%s track_id(%d) No Filled buffer available for AVCodec, wait for new buffer",
               __func__, TrackId());
    std::unique_lock<std::mutex> lock(wait_for_frame_lock_);
    std::chrono::seconds wait_time(1);
    wait_for_frame_.WaitFor(lock, wait_time);
  }
  if (stop_received_) return NO_ERROR;

  StreamBuffer iter = *frames_to_decode_.Begin();

  stream_buffer.fd       = (iter).fd;
  stream_buffer.data     = (iter).data;
  stream_buffer.capacity = (iter).frame_length;
  stream_buffer.size     = (iter).filled_length;
  stream_buffer.flag     = (iter).flags;
  stream_buffer.timestamp = (iter).timestamp;

  QMMF_VERBOSE("%s: track_id(%d)", __func__,TrackId());

  {
    std::lock_guard<std::mutex> lock(queue_lock_);
    frames_being_decoded_.PushBack(iter);
  }
  frames_to_decode_.Erase(frames_to_decode_.Begin());
  QMMF_DEBUG("%s track_id(%d) Sending buffer(0x%p) fd(%d) to avcodec for"
      " decoding ",  __func__, TrackId(), (iter).data, (iter).fd);

  QMMF_DEBUG("%s track_id(%d) frame_length(%d) filled_length(%d) to avcodec for"
      " decoding ",  __func__, TrackId(), (iter).frame_length, (iter).filled_length);

  QMMF_DEBUG("%s: Exit track_id(%d)", __func__, TrackId());
  if ((iter).flags & static_cast<uint32_t>(BufferFlags::kFlagEOS))
    return -1; // For EOS and stop case
  return NO_ERROR;
}

// This method is used by AVCodec to return buffer after decoding
status_t VideoTrackDecoder::ReturnBuffer(BufferDescriptor& stream_buffer,
                                         void* client_data) {
  QMMF_DEBUG("%s: Enter track_id(%d)", __func__, TrackId());

  QMMF_VERBOSE("%s: track_id(%d) frames_being_decoded_.size(%d)",
      __func__, TrackId(), frames_being_decoded_.Size());

  if (player_decode_profile_) {
    time_point<high_resolution_clock> curr_time = high_resolution_clock::now();
    uint64_t time_diff = duration_cast<microseconds>
                             (curr_time - prev_time_).count();

    QMMF_INFO("%s: EBD profile for Video Decoding :: %llu", __func__,
        time_diff);
    prev_time_ = curr_time;
  }

  bool found = false;

  std::list<StreamBuffer>::iterator it = frames_being_decoded_.Begin();
  for (; it != frames_being_decoded_.End(); ++it) {
    if ((*it).data ==  stream_buffer.data) {
      {
        std::lock_guard<std::mutex> lock(queue_lock_);
        unfilled_frame_queue_.PushBack(*it);
        wait_for_empty_frame_.Signal();
      }
      found = true;
      break;
    }
  }

  assert(found == true);
  frames_being_decoded_.Erase(it);

  QMMF_VERBOSE("%s: frames_being_decoded_.Size(%d)", __func__,
      frames_being_decoded_.Size());

  QMMF_DEBUG("%s Exit track_id(%d)", __func__, TrackId());
  return NO_ERROR;
}

// This method is used by AVCodec to notify stop
status_t VideoTrackDecoder::NotifyPortEvent(PortEventType event_type,
                                            void* event_data) {
  return 0;
}

status_t VideoTrackDecoder::ReconfigOutputPort(void* arg) {
  QMMF_INFO("%s: Enter track_id(%d)", __func__, TrackId());
  status_t ret = 0;
  PortreconfigData* reconfig_data = static_cast<PortreconfigData*>(arg);
  ::std::vector<BufferDescriptor> temp_out;
  switch (reconfig_data->reconfig_type) {
    case PortreconfigData::PortReconfigType::kCropParametersChanged:
      ret = video_track_sink_->UpdateCropParameters(
                static_cast<void*>(reconfig_data));
      if (ret != 0) {
        QMMF_ERROR("%s Failed to Update CropParameters", __func__);
        return ret;
      }
    break;
    case PortreconfigData::PortReconfigType::kBufferRequirementsChanged:
      QMMF_INFO("%s Releasing output_buffer_list_", __func__);
      ret = ReleaseOutputBuffers();
      if (ret != 0) {
        QMMF_ERROR("%s Buffer Release Failed", __func__);
        return ret;
      }
      QMMF_INFO("%s Allocating New set of Buffers", __func__);
      AllocOutputPortBufs();
      for (auto& iter: output_buffer_list_) {
        BufferDescriptor temp_buffer;
        temp_buffer.fd = iter.fd;
        temp_buffer.capacity = iter.frame_length;
        temp_out.push_back(temp_buffer);
      }
      ret = avcodec_->RegisterOutputBuffers(temp_out);
      if (ret != 0) {
        QMMF_ERROR("%s Buffer Registration Failed", __func__);
        return ret;
      }
      video_track_sink_->AddBufferList(output_buffer_list_);
      ret = video_track_sink_->UpdateCropParameters(static_cast<void*>(reconfig_data));
      if (ret != 0) {
        QMMF_ERROR("%s Failed to Update CropParameters", __func__);
        return ret;
      }
    break;
    default:
      QMMF_ERROR("%s Unknown PortReconfigType", __func__);
      assert(0);
  }
  QMMF_INFO("%s: Exit track_id(%d)", __func__, TrackId());

  return ret;
}

status_t VideoTrackDecoder::ReleaseOutputBuffers() {

  status_t ret = 0;

  for (auto& iter : output_buffer_list_) {
    if ((iter).pointer) {
      munmap((iter).pointer, (iter).frame_length);
      (iter).pointer = NULL;
    }
    if ((iter).fd) {
      QMMF_INFO("%s track_id(%d) (iter).fd =%d Free", __func__,
          TrackId(), (iter).fd);
      ioctl(ion_device_, ION_IOC_FREE, &((iter).handle_data.handle));
      close((iter).fd);
      (iter).fd = 0;
    }
  }

  output_buffer_list_.clear();
  return ret;
}

status_t VideoTrackDecoder::AllocInputPortBufs() {
  QMMF_INFO("%s: Enter track_id(%d)", __func__, TrackId());
  int32_t ret = 0;
  uint32_t count, size;

  assert(avcodec_ != nullptr);
  ret = avcodec_->GetBufferRequirements(kPortIndexInput,  &count, &size);
  QMMF_DEBUG("%s: BufferRequirements count(%d) size(%d)", __func__,
      count, size);
  assert(ret == NO_ERROR);

  assert(ion_device_ >= 0);
  int32_t ion_type = 0x1 << ION_IOMMU_HEAP_ID;
  void *vaddr      = NULL;

  struct ion_allocation_data alloc;
  struct ion_fd_data         ion_fddata;

  for (uint32_t i = 0; i < count; i++) {

    StreamBuffer buffer;
    vaddr = NULL;
    memset(&buffer, 0x0, sizeof(buffer));
    memset(&alloc, 0x0, sizeof(ion_allocation_data));
    memset(&ion_fddata, 0x0, sizeof(ion_fddata));

    alloc.len = size;
    alloc.len = (alloc.len + 4095) & (~4095);
    alloc.align = 4096;
    alloc.flags = ION_FLAG_CACHED;
    alloc.heap_id_mask = ion_type;

    ret = ioctl(ion_device_, ION_IOC_ALLOC, &alloc);
    if (ret < 0) {
      QMMF_ERROR("%s ION allocation failed!", __func__);
      goto ION_ALLOC_FAILED;
    }

    ion_fddata.handle = alloc.handle;
    ret = ioctl(ion_device_, ION_IOC_SHARE, &ion_fddata);
    if (ret < 0) {
      QMMF_ERROR("%s ION map failed %s", __func__, strerror(errno));
      goto ION_MAP_FAILED;
    }

    vaddr = mmap(NULL, alloc.len, PROT_READ  | PROT_WRITE, MAP_SHARED,
                 ion_fddata.fd, 0);

    if (vaddr == MAP_FAILED) {
      QMMF_ERROR("%s  ION mmap failed: %s (%d)", __func__,
          strerror(errno), errno);
        goto ION_MAP_FAILED;
    }

    buffer.fd                 =  ion_fddata.fd;
    buffer.frame_length       =  alloc.len;
    buffer.filled_length      =  0;
    buffer.data               =  vaddr;
    ion_handle_data.push_back(alloc);

    QMMF_VERBOSE("%s buffer.Fd(%d)", __func__, buffer.fd);
    QMMF_VERBOSE("%s buffer.frame_length(%d)", __func__,
        buffer.frame_length);
    QMMF_VERBOSE("%s buffer.vaddr(%p)", __func__, buffer.data);

    BufInfo bufinfo;
    memset(&bufinfo,0x0,sizeof bufinfo);

    bufinfo.vaddr   = vaddr;
    bufinfo.buf_id  = ion_fddata.fd;

    buf_info_map.add(ion_fddata.fd,bufinfo);

    input_buffer_list_.push_back(buffer);
  }

  for (uint32_t j = 0; j < buf_info_map.size(); j++) {
    QMMF_VERBOSE("%s: buf_info_map:idx(%d) :key(%d) :fd:%d :data:"
        "0x%p",  __func__, j, buf_info_map.keyAt(j), buf_info_map[j].buf_id,
        buf_info_map[j].vaddr);
  }

  QMMF_INFO("%s: Exit track_id(%d)", __func__, TrackId());
  return ret;

ION_MAP_FAILED:
  struct ion_handle_data ionHandleData;
  memset(&ionHandleData, 0x0, sizeof(ionHandleData));
  ionHandleData.handle = ion_fddata.handle;
  ioctl(ion_device_, ION_IOC_FREE, &ionHandleData);
ION_ALLOC_FAILED:
  QMMF_ERROR("%s ION Buffer allocation failed!", __func__);
  return -1;
}

status_t VideoTrackDecoder::AllocOutputPortBufs()
{
  QMMF_INFO("%s: Enter track_id(%d)", __func__, TrackId());
  int32_t ret = 0;
  uint32_t count, size;

  assert(avcodec_ != nullptr);
  ret = avcodec_->GetBufferRequirements(kPortIndexOutput,  &count, &size);
  QMMF_DEBUG("%s: BufferRequirements count(%d) size(%d)", __func__,
      count, size);
  assert(ret == NO_ERROR);

  output_buffer_count_ = count;
  output_buffer_size_ = size;

  assert(ion_device_ >= 0);
  int32_t ion_type = 0x1 << ION_IOMMU_HEAP_ID;
  void *vaddr      = NULL;

  struct ion_allocation_data alloc;
  struct ion_fd_data         ion_fddata;

  for (uint32_t i = 0; i < count; i++) {

    CodecBuffer buffer;
    vaddr = NULL;
    memset(&buffer, 0x0, sizeof(buffer));
    memset(&alloc, 0x0, sizeof(ion_allocation_data));
    memset(&ion_fddata, 0x0, sizeof(ion_fddata));

    alloc.len = size;
    alloc.len = (alloc.len + 4095) & (~4095);
    alloc.align = 4096;
    alloc.flags = ION_FLAG_CACHED;
    alloc.heap_id_mask = ion_type;

    ret = ioctl(ion_device_, ION_IOC_ALLOC, &alloc);
    if (ret < 0) {
      QMMF_ERROR("%s ION allocation failed!", __func__);
      goto ION_ALLOC_FAILED;
    }

    ion_fddata.handle = alloc.handle;
    ret = ioctl(ion_device_, ION_IOC_SHARE, &ion_fddata);
    if (ret < 0) {
      QMMF_ERROR("%s ION map failed %s", __func__, strerror(errno));
      goto ION_MAP_FAILED;
    }

    vaddr = mmap(NULL, alloc.len, PROT_READ  | PROT_WRITE, MAP_SHARED,
                 ion_fddata.fd, 0);

    if (vaddr == MAP_FAILED) {
      QMMF_ERROR("%s  ION mmap failed: %s (%d)", __func__,
          strerror(errno), errno);
      goto ION_MAP_FAILED;
    }

    buffer.handle_data.handle = ion_fddata.handle;
    buffer.fd                 = ion_fddata.fd;
    buffer.frame_length       = alloc.len;
    buffer.filled_length      = 0;
    buffer.pointer            = vaddr;

    QMMF_INFO("%s buffer.Fd(%d)", __func__, buffer.fd);
    QMMF_INFO("%s buffer.size(%d)", __func__, buffer.frame_length);
    QMMF_INFO("%s buffer.vaddr(%p)", __func__, buffer.pointer);

    output_buffer_list_.push_back(buffer);
  }

  QMMF_INFO("%s: Exit track_id(%d)", __func__, TrackId());
  return ret;

  ION_MAP_FAILED:
    struct ion_handle_data ionHandleData;
    memset(&ionHandleData, 0x0, sizeof(ionHandleData));
    ionHandleData.handle = ion_fddata.handle;
    ioctl(ion_device_, ION_IOC_FREE, &ionHandleData);
  ION_ALLOC_FAILED:
    QMMF_ERROR("%s ION Buffer allocation failed!", __func__);
    return -1;
}


};  // namespace player
};  // namespace qmmf
