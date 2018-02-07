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

#define TAG "VideoSink"

#include "player/src/service/qmmf_player_video_sink.h"
#include "player/src/service/qmmf_player_audio_sink.h"

#include <memory>
#define ROUND_TO(val, round_to) (val + round_to - 1) & ~(round_to - 1)
static const nsecs_t kWaitDuration = 1000000000; // 1 s.

namespace qmmf {
namespace player {

using ::qmmf::avcodec::AVCodec;
using ::qmmf::avcodec::CodecBuffer;
using ::qmmf::avcodec::CodecParam;
using ::qmmf::avcodec::CodecPortStatus;
using ::qmmf::avcodec::PortreconfigData;
using ::qmmf::avcodec::PortEventType;
using ::std::make_shared;
using ::std::shared_ptr;
using ::std::thread;
using ::std::mutex;
using ::std::lock_guard;
using ::std::this_thread::sleep_for;

VideoSink* VideoSink::instance_ = nullptr;

VideoSink* VideoSink::CreateVideoSink() {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  if(!instance_) {
      instance_ = new VideoSink();
      if(!instance_) {
          QMMF_ERROR("%s:%s: Can't Create VideoSink Instance!", TAG, __func__);
          return NULL;
        }
      }

  QMMF_INFO("%s:%s: VideoSink Instance Created Successfully(0x%p)", TAG,
      __func__, instance_);

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
  return instance_;
}

VideoSink::VideoSink() {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
}

VideoSink::~VideoSink() {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  if (!video_track_sinks.isEmpty()) {
    video_track_sinks.clear();
  }
  instance_ = NULL;
  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
}

status_t VideoSink::CreateTrackSink(uint32_t track_id,
                                    VideoTrackParams& track_param) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  shared_ptr<VideoTrackSink> track_sink;

  if (track_param.params.out_device == VideoOutSubtype::kHDMI)
    track_sink =  make_shared<VideoTrackSink>();

  video_track_sinks.add(track_id,track_sink);
  track_sink->Init(track_param);

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
  return NO_ERROR;
}

const shared_ptr<VideoTrackSink>& VideoSink::GetTrackSink(
    uint32_t track_id) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  int32_t idx = video_track_sinks.indexOfKey(track_id);
  assert(idx >= 0);
  return video_track_sinks.valueFor(track_id);
  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
}

status_t VideoSink::StartTrackSink(uint32_t track_id) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  shared_ptr<VideoTrackSink> track_sink = video_track_sinks.valueFor(track_id);
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

status_t VideoSink::StopTrackSink(uint32_t track_id) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  shared_ptr<VideoTrackSink> track_sink = video_track_sinks.valueFor(track_id);
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

status_t VideoSink::DeleteTrackSink(uint32_t track_id) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  shared_ptr<VideoTrackSink> track_sink = video_track_sinks.valueFor(track_id);
  assert(track_sink.get() != NULL);

  auto ret = track_sink->DeleteSink();
  if (ret != NO_ERROR) {
    QMMF_INFO("%s:%s: track_id(%d) DeleteSink failed!", TAG, __func__,
      track_id);
    return ret;
  }

  video_track_sinks.removeItem(track_id);

  QMMF_INFO("%s:%s: track_id(%d) DeleteSink Successful!", TAG,
      __func__, track_id);

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
  return ret;
}

VideoTrackSink::VideoTrackSink()
    : current_width(0), current_height(0),
      stopplayback_(false), paused_(false),
      decoded_frame_number_(0), display_started_(0),
      playback_speed_(TrickModeSpeed::kSpeed_1x),
      playback_dir_(TrickModeDirection::kNormalForward),
      displayed_frames_(0), grab_picture_(false),
      ion_device_(-1), grabpicture_file_fd_(-1), snapshot_dumps_(0) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
#ifdef DUMP_YUV_FRAMES
  file_fd_ = open("/data/misc/qmmf/video_track.yuv", O_CREAT | O_WRONLY | O_TRUNC, 0655);
  if(file_fd_ < 0) {
    QMMF_ERROR("%s:%s Failed to open o/p yuv dump file ", TAG, __func__);
  }
#endif

  // open ion device
  ion_device_ = open("/dev/ion", O_RDONLY);
  if (ion_device_ < 0) {
    QMMF_ERROR("%s: %s() error opening ion device: %d[%s]", TAG, __func__,
        errno, strerror(errno));
  }

  player_decode_profile_ = GetPlayerDecodeProfileProperty();

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
}

VideoTrackSink::~VideoTrackSink() {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);

  if (grab_picture_buffer_.data) {
    munmap(grab_picture_buffer_.data, grab_picture_buffer_.capacity);
    grab_picture_buffer_.data = nullptr;
  }

  if (grab_picture_buffer_.fd) {
    QMMF_INFO("%s:%s track_id(%d) grab_picture_buffer_.fd =%d Free",
        TAG, __func__, TrackId(), grab_picture_buffer_.fd);
    ioctl(ion_device_, ION_IOC_FREE, &(grab_picture_ion_handle_.handle));
    close(grab_picture_buffer_.fd);
    grab_picture_buffer_.fd = 0;
  }

  if (snapshot_dumps_) {
    if (grabpicture_file_fd_ > 0) {
        close(grabpicture_file_fd_);
    }
  }

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
}

status_t VideoTrackSink::Init(VideoTrackParams& track_param) {
  QMMF_INFO("%s:%s: Enter track_id(%d)", TAG, __func__, track_param.track_id);

  track_params_.track_id = track_param.track_id;
  track_params_.params = track_param.params;
  crop_data_.top = 0;
  crop_data_.left = 0;
  crop_data_.width = track_param.params.width;
  crop_data_.height = track_param.params.height;
  current_width = track_param.params.width;
  current_height = track_param.params.height;

  auto ret = CreateDisplay(display::DisplayType::kPrimary, track_param);
  if (ret != 0) {
    QMMF_ERROR("%s:%s CreateDisplay Failed!!", TAG, __func__);
    return ret;
  }

  uint32_t buffer_size = VENUS_BUFFER_SIZE(COLOR_FMT_NV12,
      track_param.params.width, track_param.params.height);
  QMMF_DEBUG("%s:%s: buffer_size is (%d)", TAG, __func__,buffer_size);

  AllocateGrabPictureBuffer(buffer_size);

  ret = fcvSetOperationMode(FASTCV_OP_CPU_OFFLOAD);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s Unable to set FastCV operation mode: %d",TAG,__func__,
        ret);
    return ret;
  }

  GetSnapShotDumpsProperty();

  QMMF_INFO("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());

  return NO_ERROR;
}

status_t VideoTrackSink::StartSink() {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  auto ret = 0;
 lock_guard<mutex> lock(state_change_lock_);

  stopplayback_ = false;

  renderer_thread_ = new thread(VideoTrackSink::RendererThread, this);
  if (renderer_thread_ == nullptr) {
    QMMF_ERROR("%s: %s() unable to allocate Renderer thread", TAG, __func__);
    return -ENOMEM;
  }

  displayed_buffer_thread_ = new thread(VideoTrackSink::DisplayedBufferThread, this);
  if (displayed_buffer_thread_ == nullptr) {
    QMMF_ERROR("%s: %s() unable to allocate DisplayedBuffer thread", TAG,
        __func__);
    return -ENOMEM;
  }

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t VideoTrackSink::StopSink() {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  auto ret = 0;
  lock_guard<mutex> lock(state_change_lock_);

  stopplayback_ = true;

  renderer_thread_->join();
  delete renderer_thread_;

  displayed_buffer_thread_->join();
  delete displayed_buffer_thread_;

  QMMF_DEBUG("%s:%s: Total number of video frames decoded %d", TAG, __func__,
      decoded_frame_number_);
  QMMF_DEBUG("%s:%s: Total number of video frames displayed %d", TAG, __func__,
       displayed_frames_);

  decoded_frame_number_ = 0;
  displayed_frames_ = 0;

  if (!decoded_buffer_queue_.Empty())
    decoded_buffer_queue_.Clear();

  if (!displayed_buffer_queue_.Empty())
    displayed_buffer_queue_.Clear();

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t VideoTrackSink::PauseSink() {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  auto ret = 0;
  lock_guard<mutex> lock(state_change_lock_);

  paused_ = true;

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t VideoTrackSink::ResumeSink() {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  auto ret = 0;
  lock_guard<mutex> lock(state_change_lock_);

  paused_ = false;

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t VideoTrackSink::DeleteSink() {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  auto ret = 0;

  ret = DeleteDisplay(display::DisplayType::kPrimary);
  if (ret != 0) {
    QMMF_ERROR("%s:%s DeleteDisplay Failed!!", TAG, __func__);
    return ret;
  }

  video_track_decoder_.reset();

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t VideoTrackSink::SetTrickMode(TrickModeSpeed speed,
                                      TrickModeDirection dir) {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  QMMF_DEBUG("%s:%s: Speed (%u) Type (%u)", TAG, __func__,
      static_cast<uint32_t>(speed), static_cast<uint32_t>(dir));

  playback_speed_ = speed;
  playback_dir_ = dir;

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return NO_ERROR;
}

void VideoTrackSink::AddBufferList(Vector<CodecBuffer>& list) {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());

  output_buffer_list_ = list;
  output_free_buffer_queue_.Clear();
  output_occupy_buffer_queue_.Clear();
  buf_info_map.clear();

  //decoded buffer queue
  for (auto& iter : output_buffer_list_) {
    QMMF_INFO("%s:%s: track_id(%d) Adding buffer fd(%d) to "
        "output_free_buffer_queue_", TAG, __func__,TrackId() ,
        iter.fd);
    output_free_buffer_queue_.PushBack(iter);
    BufInfo bufinfo_temp;
    bufinfo_temp.vaddr = iter.pointer;
    buf_info_map.add(iter.fd,bufinfo_temp);
  }

  for (uint32_t j = 0; j < buf_info_map.size(); j++) {
    QMMF_VERBOSE("%s:%s: buf_info_map:idx(%d) :key(%d) :fd:%d :data:"
        "0x%p", TAG, __func__, j, buf_info_map.keyAt(j), buf_info_map[j].buf_id,
        buf_info_map[j].vaddr);
  }

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
}

void VideoTrackSink::PassTrackDecoder(
    const shared_ptr<VideoTrackDecoder>& video_track_decoder) {
  video_track_decoder_ = video_track_decoder;
}

status_t VideoTrackSink::GetBuffer(BufferDescriptor& codec_buffer,
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
  codec_buffer.capacity = (iter).frame_length;
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

status_t VideoTrackSink::ReturnBuffer(BufferDescriptor& codec_buffer,
                                      void* client_data) {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  status_t ret = 0;

  if (player_decode_profile_) {
    time_point<high_resolution_clock> curr_time = high_resolution_clock::now();
    uint64_t time_diff = duration_cast<microseconds>
                             (curr_time - prev_time_).count();

    QMMF_INFO("%s:%s: FBD profile for Video Decoding :: %llu", TAG,__func__,
        time_diff);
    prev_time_ = curr_time;
  }

  assert(codec_buffer.data != NULL);

  QMMF_VERBOSE("%s:%s: track_id(%d) Received buffer(0x%p) from FBD", TAG,
      __func__, TrackId(), codec_buffer.data);

  if (!((codec_buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS)) ||
      stopplayback_ || !(codec_buffer.size) || (paused_))) {
    Dispatcher(codec_buffer);

#ifdef DUMP_YUV_FRAMES
  DumpYUVData(codec_buffer);
#endif
  }

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return ret;
}

status_t VideoTrackSink::ReturnBufferToCodec(BufferDescriptor& codec_buffer) {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());

  List<CodecBuffer>::iterator it = (output_occupy_buffer_queue_).Begin();
  bool found = false;
  for (; it != output_occupy_buffer_queue_.End(); ++it) {
    QMMF_DEBUG("%s:%s track_id(%d) Checking match %d vs %d ", TAG,
        __func__, TrackId(), (*it).fd,  codec_buffer.fd);
    if (((*it).fd) == (codec_buffer.fd)) {
      QMMF_DEBUG("%s:%s track_id(%d) Buffer found", TAG, __func__, TrackId());
      output_free_buffer_queue_.PushBack(*it);
      output_occupy_buffer_queue_.Erase(it);
      wait_for_frame_.signal();
      found = true;
      break;
    }
  }

  if (codec_buffer.fd == 0) {
    return NO_ERROR;
  }

  assert(found == true);
  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return NO_ERROR;
}

status_t VideoTrackSink::SkipFrame() {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());

  if (playback_speed_ == TrickModeSpeed::kSpeed_1x ||
      playback_dir_ == TrickModeDirection::kSlowForward ) {
    return false;

  } else if (playback_speed_ == TrickModeSpeed::kSpeed_2x) {
    return (((decoded_frame_number_%2) == 0)? false: true);

  } else if (playback_speed_ == TrickModeSpeed::kSpeed_4x) {
    return (((decoded_frame_number_%4) == 0)? false: true);

  } else if (playback_speed_ == TrickModeSpeed::kSpeed_8x) {
    return (((decoded_frame_number_%8) == 0)? false: true);
  }

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return NO_ERROR;
}

status_t VideoTrackSink::Dispatcher(const BufferDescriptor& codec_buffer) {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());

  QMMF_DEBUG("%s:%s:Adding decoded buffer fd(%d) to queue", TAG, __func__,
      codec_buffer.fd);

  BufferDescriptor codec_buffer_sink;
  memset(&codec_buffer_sink, 0x0, sizeof codec_buffer_sink);

  codec_buffer_sink = codec_buffer;
  decoded_buffer_queue_.PushBack(codec_buffer_sink);

  QMMF_DEBUG("%s:%s: dispatcherqueue size(%d)", TAG, __func__,
      decoded_buffer_queue_.Size());

  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return NO_ERROR;
}

void VideoTrackSink::RendererThread(VideoTrackSink* video_sink) {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, video_sink->TrackId());
  video_sink->Renderer();
  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, video_sink->TrackId());
}

void VideoTrackSink::Renderer() {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);

  status_t ret = 0;
  int64_t sleep_time_us = 1000000/(track_params_.params.frame_rate);

  while (!stopplayback_) {

    if (decoded_buffer_queue_.Size() > 0) {

      if (paused_) {
        continue;
      }

      BufferDescriptor codec_buffer;
      BufferDescriptor iter = *decoded_buffer_queue_.Begin();
      codec_buffer = iter;

      ++(decoded_frame_number_);
      QMMF_DEBUG("%s:%s: track_id(%d) For decoded/rendered video frame number %u"
          " timestamps is %llu buffer.fd is %d ", TAG, __func__, TrackId(),
          decoded_frame_number_, codec_buffer.timestamp, codec_buffer.fd);

      if (SkipFrame()) {
        QMMF_DEBUG("%s:%s:Skipping frame number %d to display", TAG, __func__,
            decoded_frame_number_);

        ReturnBufferToCodec(codec_buffer);
        decoded_buffer_queue_.Erase(decoded_buffer_queue_.Begin());
      } else {
        QMMF_DEBUG("%s:%s PushFrameToDisplay codec_buffer.fd ::  %d", TAG,
            __func__, codec_buffer.fd);
        ret = PushFrameToDisplay(codec_buffer);
        if (ret != 0) {
          QMMF_ERROR("%s:%s PushFrameToDisplay Failed!!", TAG, __func__);
        }

        if (playback_dir_ == TrickModeDirection::kSlowForward) {
          QMMF_DEBUG("%s:%s Sleeping for %0.2f ms in Slow Forward", TAG, __func__,
              (float)(sleep_time_us *
                  static_cast<uint32_t>(playback_speed_))/(float)1000);
          sleep_for(microseconds(sleep_time_us *
              static_cast<uint32_t>(playback_speed_)));

        } else if (playback_dir_ == TrickModeDirection::kNormalRewind){
          QMMF_DEBUG("%s:%s Sleeping for %0.2f ms in Normal Rewind", TAG,
              __func__, (float)((sleep_time_us) * 6)/(float)1000);
          sleep_for(microseconds(sleep_time_us * 6));

        } else {
          QMMF_DEBUG("%s:%s Sleeping for %0.2f ms in Normal Playback", TAG,
              __func__, (float)(sleep_time_us)/(float)1000);
          sleep_for(microseconds(sleep_time_us-500));
        }

        ++(displayed_frames_);
        QMMF_DEBUG("%s:%s: track_id(%d) displayed video frame number %d",
            TAG, __func__, TrackId(), displayed_frames_);

        QMMF_DEBUG("%s:%s codec buff.fd :: %d", TAG, __func__, codec_buffer.fd);

        decoded_buffer_queue_.Erase(decoded_buffer_queue_.Begin());
        displayed_buffer_queue_.PushBack(codec_buffer);
      }
    }
  }

  QMMF_INFO("%s:%s: Exit ", TAG, __func__);
}

void VideoTrackSink::DisplayedBufferThread(VideoTrackSink* video_sink) {
  QMMF_DEBUG("%s:%s: Enter track_id(%d)", TAG, __func__, video_sink->TrackId());
  video_sink->DisplayedBuffer();
  QMMF_DEBUG("%s:%s: Exit track_id(%d)", TAG, __func__, video_sink->TrackId());
}

void VideoTrackSink::DisplayedBuffer() {
  QMMF_INFO("%s:%s: Enter ", TAG, __func__);

  while (!stopplayback_) {

    if (displayed_buffer_queue_.Size() > 0) {

      if (paused_) {
        continue;
      }

      //sleep to avoid any flickering issue on display
      //TODO: when display fence will come this sleep will be removed
      usleep(30000);

      BufferDescriptor codec_buffer;
      BufferDescriptor iter = *displayed_buffer_queue_.Begin();
      codec_buffer = iter;
      ReturnBufferToCodec(codec_buffer);
      displayed_buffer_queue_.Erase(displayed_buffer_queue_.Begin());
    }
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

status_t VideoTrackSink::NotifyPortEvent(PortEventType event_type,
                                         void* event_data) {
  QMMF_INFO("%s:%s Enter track_id(%d)", TAG, __func__, TrackId());
  if (event_type == PortEventType::kPortSettingsChanged) {
    status_t ret = 0;
    ret = video_track_decoder_->ReconfigOutputPort(event_data);
    assert(ret == 0);
    return ret;
  }
  QMMF_INFO("%s:%s Exit track_id(%d)", TAG, __func__, TrackId());
  return 0;
}

status_t VideoTrackSink::UpdateCropParameters(void* arg) {
  QMMF_INFO("%s:%s Enter track_id(%d)", TAG, __func__, TrackId());
  PortreconfigData *reconfig_data = static_cast<PortreconfigData*>(arg);

  switch (reconfig_data->reconfig_type) {
    case PortreconfigData::PortReconfigType::kBufferRequirementsChanged:
      surface_config.width = current_width =
          (static_cast<PortreconfigData::CropData>(reconfig_data->rect)).width;
      surface_config.height = current_height =
          (static_cast<PortreconfigData::CropData>(reconfig_data->rect)).height;
      wait_for_frame_.signal();
      break;
    case PortreconfigData::PortReconfigType::kCropParametersChanged:
      crop_data_ = static_cast<PortreconfigData::CropData>(reconfig_data->rect);
      break;
    default:
      QMMF_ERROR("%s:%s Unknown PortReconfigType", TAG, __func__);
      return -1;
  }
  QMMF_INFO("%s:%s Exit track_id(%d)", TAG, __func__, TrackId());
  return NO_ERROR;
}

status_t VideoTrackSink::CreateDisplay(
    display::DisplayType display_type,
    VideoTrackParams& track_param) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  int32_t res;
  DisplayCb  display_status_cb;

  display_= new Display();
  assert(display_ != nullptr);
  res = display_->Connect();
  if (res != 0) {
    QMMF_ERROR("%s:%s Connect Failed!!", TAG, __func__);
    delete display_;
    display_ = nullptr;
    return res;
  }

  display_status_cb.EventCb = [&] ( DisplayEventType event_type,
      void *event_data, size_t event_data_size) { DisplayCallbackHandler
      (event_type, event_data, event_data_size); };

  display_status_cb.VSyncCb = [&] ( int64_t time_stamp)
      { DisplayVSyncHandler(time_stamp); };

  res = display_->CreateDisplay(display_type, display_status_cb);
  if (res != 0) {
    QMMF_ERROR("%s:%s CreateDisplay Failed!!", TAG, __func__);
    display_->Disconnect();
    delete display_;
    display_ = nullptr;
    return res;
  }

  memset(&surface_config, 0x0, sizeof surface_config);
  surface_config.width = track_param.params.width;
  surface_config.height = track_param.params.height;

  surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
  surface_config.buffer_count = track_param.params.num_buffers;
  surface_config.cache = 0;
  surface_config.use_buffer = 1;
  surface_config.context = 1;
  res = display_->CreateSurface(surface_config, &surface_id_);
  if (res != 0) {
    QMMF_ERROR("%s:%s CreateSurface Failed!!", TAG, __func__);
    DeleteDisplay(display_type);
    return res;
  }
  display_started_ = 1;

  surface_param_.src_rect = { 0.0, 0.0,
      static_cast<float>(track_param.params.width),
      static_cast<float>(track_param.params.height)};

  surface_param_.dst_rect = { 0.0, 0.0, DISPLAY_WIDTH, DISPLAY_HEIGHT};

  surface_param_.surface_blending =
      SurfaceBlending::kBlendingCoverage;
  surface_param_.surface_flags.cursor = 0;
  surface_param_.frame_rate=track_param.params.frame_rate;
  surface_param_.z_order = 0;
  surface_param_.solid_fill_color = 0;
  surface_param_.surface_transform.rotation = 0.0f;
  surface_param_.surface_transform.flip_horizontal = 0;
  surface_param_.surface_transform.flip_vertical = 0;

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return res;

}

status_t VideoTrackSink::DeleteDisplay(display::DisplayType display_type) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  int32_t res = 0;

  if (display_started_ == 1) {
    display_started_ =0;
    res = display_->DestroySurface(surface_id_);
    if (res != 0) {
      QMMF_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
    }

    res = display_->DestroyDisplay(display_type);
    if (res != 0) {
      QMMF_ERROR("%s:%s DestroyDisplay Failed!!", TAG, __func__);
    }

    res = display_->Disconnect();

    if (display_ != nullptr) {
      QMMF_INFO("%s:%s: DELETE display_:%p", TAG, __func__, display_);
      delete display_;
      display_ = nullptr;
    }
  }
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return res;
}

void VideoTrackSink::DisplayCallbackHandler(DisplayEventType
    event_type, void *event_data, size_t event_data_size) {
  QMMF_DEBUG("%s:%s Enter ", TAG, __func__);
  QMMF_DEBUG("%s:%s Exit ", TAG, __func__);
}

void VideoTrackSink::DisplayVSyncHandler(int64_t time_stamp) {
  QMMF_VERBOSE("%s:%s: Enter", TAG, __func__);
  QMMF_VERBOSE("%s:%s: Exit", TAG, __func__);
}

status_t VideoTrackSink::PushFrameToDisplay(BufferDescriptor& codec_buffer) {
  BufInfo bufinfo;
  memset(&bufinfo, 0x0, sizeof bufinfo);

  bufinfo = buf_info_map.valueFor(codec_buffer.fd);

   uint8_t*vaddr = (uint8_t*)bufinfo.vaddr;
   if (display_started_) {
     surface_buffer_.plane_info[0].ion_fd = codec_buffer.fd;
     surface_buffer_.buf_id = static_cast<int32_t>(codec_buffer.fd);
     surface_buffer_.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
     surface_buffer_.plane_info[0].stride = ROUND_TO(surface_config.width, 128);
     surface_buffer_.plane_info[0].size = codec_buffer.size;
     surface_buffer_.plane_info[0].width = surface_config.width;
     surface_buffer_.plane_info[0].height = surface_config.height;
     surface_buffer_.plane_info[0].offset = codec_buffer.offset;
     surface_buffer_.plane_info[0].buf = (void*)vaddr;
     surface_param_.src_rect = { (float)crop_data_.left,
                                 (float)crop_data_.top,
                                 (float)crop_data_.width + (float)crop_data_.left,
                                 (float)crop_data_.height + (float)crop_data_.top};

    QMMF_DEBUG("%s:%s CropData Used for Display L(%u) T(%u) R(%u) B(%u)"
        " surface_config.width(%u) surface_config.height(%d) stride(%d)", TAG,
        __func__,crop_data_.left, crop_data_.top, crop_data_.width,
        crop_data_.height, surface_config.width, surface_config.height,
        surface_buffer_.plane_info[0].stride);

    {
      lock_guard<mutex> lock(grab_picture_lock);
      if (grab_picture_) {
        uint32_t size = VENUS_BUFFER_SIZE(COLOR_FMT_NV12,
            surface_config.width, surface_config.height);
        CopyGrabPictureBuffer(surface_buffer_, size);
        grab_picture_ = false;
      }
    }

    auto ret = display_->QueueSurfaceBuffer(surface_id_, surface_buffer_,
         surface_param_);
    if(ret != 0) {
      QMMF_ERROR("%s:%s QueueSurfaceBuffer Failed!!", TAG, __func__);
      return ret;
    }

    ret = display_->DequeueSurfaceBuffer( surface_id_, surface_buffer_);
    if(ret != 0) {
      QMMF_ERROR("%s:%s DequeueSurfaceBuffer Failed!!", TAG, __func__);
       return ret;
    }
   }

   return NO_ERROR;
}

status_t VideoTrackSink::GrabPicture(PictureParam param,
                                     BufferDescriptor& buffer) {
  QMMF_INFO("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());

  if (grab_picture_) {
    QMMF_WARN("%s:%s Previous request is not complete!", TAG, __func__);
    return -EBUSY;
  }

  grab_picture_ = true;

  {
    Mutex::Autolock auto_lock(grab_picture_buffer_copy_lock_);
    auto ret = wait_for_grab_picture_buffer_copy_.waitRelative(
        grab_picture_buffer_copy_lock_, kWaitDuration);
    if (ret == TIMED_OUT) {
      QMMF_ERROR("%s:%s: track_id(%d) Failed to copy grab picture buffer",
          TAG, __func__, TrackId());
      return ret;
    }
  }

  buffer = grab_picture_buffer_;

  QMMF_INFO("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return NO_ERROR;
}

status_t VideoTrackSink::CopyGrabPictureBuffer(SurfaceBuffer& buffer,
                                                uint32_t size) {
  QMMF_INFO("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());

  size_t scanline =  ROUND_TO(surface_config.height, 32);

  uint32_t  yplane_length = scanline * buffer.plane_info[0].stride;

  fcvScaleDownMNu8(static_cast<uint8_t*>(buffer.plane_info[0].buf),
      track_params_.params.width,
      track_params_.params.height,
      buffer.plane_info[0].stride,
      static_cast<uint8_t*>(grab_picture_buffer_.data),
      track_params_.params.width,
      track_params_.params.height,
      buffer.plane_info[0].stride);

  fcvScaleDownMNInterleaveu8((static_cast<uint8_t*>(buffer.plane_info[0].buf) +
      yplane_length),
      track_params_.params.width >> 1,
      track_params_.params.height >> 1,
      buffer.plane_info[0].stride,
      (static_cast<uint8_t*>(grab_picture_buffer_.data) +
      track_params_.params.width*track_params_.params.height),
      track_params_.params.width >> 1,
      track_params_.params.height >> 1,
      buffer.plane_info[0].stride);

  grab_picture_buffer_.capacity = size;

  QMMF_DEBUG("%s:%s: Buffer data(0x%p)", TAG, __func__,grab_picture_buffer_.data);
  QMMF_DEBUG("%s:%s: Buffer size(%d)", TAG, __func__,size);

 if (snapshot_dumps_) {
    String8 snapshot_filepath;
    struct timeval tv;
    gettimeofday(&tv, NULL);

    snapshot_filepath.appendFormat("/data/misc/qmmf/player_service_snapshot_%dx%d_%lu.%s",
        surface_config.width, surface_config.height, tv.tv_sec, "yuv");

    grabpicture_file_fd_ = open(snapshot_filepath.string(), O_CREAT |
        O_WRONLY | O_TRUNC, 0655);
    assert(grabpicture_file_fd_ >= 0);

    uint32_t bytes_written;
    bytes_written  = write(grabpicture_file_fd_, grab_picture_buffer_.data, size);
    if (bytes_written != size) {
      QMMF_ERROR("Bytes written != %d and written = %u", size, bytes_written);
    }

    close(grabpicture_file_fd_);
  }

  wait_for_grab_picture_buffer_copy_.signal();
  QMMF_INFO("%s:%s: Exit track_id(%d)", TAG, __func__, TrackId());
  return NO_ERROR;
}

int32_t VideoTrackSink::AllocateGrabPictureBuffer(const uint32_t size) {
  QMMF_INFO("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());
  QMMF_DEBUG("%s:%s: size(%d)", TAG, __func__, size);
  int32_t ret = 0;

  assert(ion_device_ >= 0);
  int32_t ion_type = 0x1 << ION_IOMMU_HEAP_ID;
  void *vaddr      = NULL;

  struct ion_allocation_data alloc;
  struct ion_fd_data         ion_fddata;

  vaddr = NULL;
  memset(&grab_picture_buffer_, 0x0, sizeof(grab_picture_buffer_));
  memset(&alloc, 0x0, sizeof(alloc));
  memset(&ion_fddata, 0x0, sizeof(ion_fddata));
  memset(&grab_picture_ion_handle_, 0x0, sizeof(grab_picture_ion_handle_));

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

  grab_picture_ion_handle_.handle = ion_fddata.handle;
  grab_picture_buffer_.fd         = ion_fddata.fd;
  grab_picture_buffer_.capacity   = alloc.len;
  grab_picture_buffer_.data       = vaddr;

  QMMF_DEBUG("%s:%s Fd(%d)", TAG, __func__, grab_picture_buffer_.fd);
  QMMF_DEBUG("%s:%s size(%d)", TAG, __func__, grab_picture_buffer_.capacity);
  QMMF_DEBUG("%s:%s vaddr(%p)", TAG, __func__, grab_picture_buffer_.data);

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

void VideoTrackSink::GetSnapShotDumpsProperty() {
  char prop[PROPERTY_VALUE_MAX];
  memset(prop, 0, sizeof(prop));
  property_get("persist.player.snapshot.dumps", prop, "0");
  snapshot_dumps_ = (uint32_t) atoi(prop);
}

#ifdef DUMP_YUV_FRAMES
void VideoTrackSink::DumpYUVData(BufferDescriptor& codec_buffer) {
  QMMF_VERBOSE("%s:%s: Enter track_id(%d)", TAG, __func__, TrackId());

  if (file_fd_ > 0) {
    QMMF_DEBUG("%s:%s Got decoded buffer of capacity(%d)", TAG, __func__,
        codec_buffer.capacity);

    BufInfo bufinfo;
    memset(&bufinfo, 0x0, sizeof bufinfo);

    bufinfo = buf_info_map.valueFor(codec_buffer.fd);

    uint8_t*vaddr = static_cast<uint8_t*>(bufinfo.vaddr);
    uint32_t bytes_written;

    uint8_t  *pSrc = vaddr;
    bytes_written  = write(file_fd_, pSrc,
        codec_buffer.capacity);
    if(bytes_written != codec_buffer.capacity) {
      QMMF_ERROR("Bytes written != %d and written = %u",codec_buffer.capacity,
          bytes_written);
    }
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
