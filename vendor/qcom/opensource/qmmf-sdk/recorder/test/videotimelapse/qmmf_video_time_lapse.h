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

#include <cutils/native_handle.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/msm_ion.h>
#include <media/msm_media_info.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <system/graphics.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <future>
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <qmmf-sdk/qmmf_avcodec.h>
#include <qmmf-sdk/qmmf_codec.h>
#include <qmmf-sdk/qmmf_recorder.h>
#include <qmmf-sdk/qmmf_recorder_params.h>

typedef struct ion_allocation_data IonHandleData;

static const std::string yuv_snapshot_file_name_preamble =
    "/data/misc/qmmf/video_time_lapse_sanpshot_";
static const std::string video_time_lapse_file_name_preamble =
    "/data/misc/qmmf/video_time_lapse_";

// Prop to enable YUV snapshot dumping from YUV track
#define PROP_DUMP_YUV_SNAPSHOT "persist.qmmf.time.lapse.dumpyuv"

enum class TimeLapseMode {
  kVideoTimeLapse,
  kPhotoTimeLapse,
};

enum class VideoEncodeFormat {
  kAVC,
  kHEVC,
  kMJPEG,
};

enum class ImageEncodeFormat {
  kJPEG,
};

struct TimeLapseParams {
  uint32_t camera_id;
  uint32_t time_lapse_mode;
  uint32_t time_lapse_interval;  // [ms.]
  uint32_t width;
  uint32_t height;
  VideoEncodeFormat video_encode_format;
  ImageEncodeFormat image_encode_format;
  uint32_t fps;
  uint32_t record_duration;
};

typedef std::function<void(qmmf::BufferDescriptor&)> ReturnBufferCB;

class EncoderSource : public qmmf::avcodec::ICodecSource {
 public:
  EncoderSource(ReturnBufferCB& return_buffer_cb, const uint32_t fps,
                uint32_t buffer_size);
  ~EncoderSource();
  int32_t GetBuffer(qmmf::BufferDescriptor& codec_buffer,
                    void* client_data) override;
  int32_t ReturnBuffer(qmmf::BufferDescriptor& codec_buffer,
                       void* client_data) override;
  int32_t NotifyPortEvent(qmmf::avcodec::PortEventType event_type,
                          void* event_data) override;
  void ConsumeBuffer(qmmf::BufferDescriptor& buffer);
  void SetEOS();
  int32_t BufferStatus();

 private:
  std::atomic<bool> atomic_eos_;
  std::mutex wait_for_frame_lock_;
  std::condition_variable wait_for_frame_;
  std::list<qmmf::BufferDescriptor> input_free_buffer_list_;
  std::list<qmmf::BufferDescriptor> input_occupy_buffer_list_;
  ReturnBufferCB return_buffer_cb_;
  uint64_t time_stamp_;
  uint32_t buffer_size_;
  uint32_t encode_fps_;
  static const uint32_t kEOSFlag;
};  // Class EncoderSource

class EncoderSink : public qmmf::avcodec::ICodecSource {
 public:
  EncoderSink(std::string file_name);
  ~EncoderSink();
  int32_t GetBuffer(qmmf::BufferDescriptor& codec_buffer,
                    void* client_data) override;
  int32_t ReturnBuffer(qmmf::BufferDescriptor& codec_buffer,
                       void* client_data) override;
  int32_t NotifyPortEvent(qmmf::avcodec::PortEventType event_type,
                          void* event_data) override;
  void AddBufferList(std::vector<qmmf::BufferDescriptor>& list);
  int32_t BufferStatus();

 private:
  std::ofstream output_file_;
  std::mutex wait_for_frame_lock_;
  std::condition_variable wait_for_frame_;
  std::vector<qmmf::BufferDescriptor> output_list_;
  std::list<qmmf::BufferDescriptor> output_free_buffer_list_;
  std::list<qmmf::BufferDescriptor> output_occupy_buffer_list_;
};  // Class EncoderSink

class TimeLapse {
 public:
  TimeLapse(const TimeLapseParams& params);
  ~TimeLapse();

  TimeLapse(const TimeLapse&) = delete;
  TimeLapse& operator=(const TimeLapse&) = delete;

  int32_t Start();
  int32_t Stop();

 private:
  static void TimeLapseModeThread(TimeLapse* timelapse);
  int32_t StartTimeLapseModeOne();
  int32_t StartTimeLapseModeTwo();
  int32_t TakeYUVSnapshotandEnqueuetoEncoder();
  void YUVSnapshotCb(uint32_t camera_id, uint32_t image_sequence_count,
                     qmmf::BufferDescriptor buffer,
                     qmmf::recorder::MetaData meta_data);
  void DumpYUVSnapShot(const qmmf::BufferDescriptor& buffer);
  void ReturnYUVSnapshotBuffer(qmmf::BufferDescriptor& buffer);
  int32_t StartCamera();
  int32_t StopCamera();
  int32_t CreateSession();
  int32_t DeleteSession();
  int32_t StartSession();
  int32_t StopSession();
  int32_t SetupAVCodec(const TimeLapseParams& params);
  int32_t StartAVCodec();
  int32_t StopAVCodec();
  int32_t AllocateBuffer(uint32_t port);
  int32_t ReleaseBuffer();
  int32_t CreateLPMTrack();
  int32_t DeleteLPMTrack();
  bool ResolutionSupported(uint32_t width, uint32_t height);

  enum class VideoTimeLapseMode {
    kModeOne,
    kModeTwo,
  };

  qmmf::recorder::Recorder recorder_;
  qmmf::avcodec::IAVCodec* avcodec_;
  std::shared_ptr<EncoderSource> encoder_source_;
  std::shared_ptr<EncoderSink> encoder_sink_;
  android::CameraMetadata static_info_;
  TimeLapseParams params_;
  VideoTimeLapseMode video_time_lapse_mode_;
  std::thread time_lapse_thread_;
  uint32_t session_id_;
  int32_t ion_device_;
  uint64_t snapshot_count_;
  static const uint32_t kLPMTrackId;
  static const uint32_t kLPMTrackWidth;
  static const uint32_t kLPMTrackHeight;
  static const uint32_t kThresholdTime;
  static const uint32_t kSanpShotBufferReturnedWaitTimeOut;
  std::atomic<bool> atomic_stop_;
  std::promise<int32_t> snapshot_buffer_returned_promise_;
  std::future<int32_t> snapshot_buffer_returnerd_future_;
  bool is_dump_yuv_snapshot_enabled_;
  std::queue<qmmf::BufferDescriptor> yuv_sanpshot_queue_;
  std::vector<qmmf::BufferDescriptor> input_buffer_list_;
  std::vector<qmmf::BufferDescriptor> output_buffer_list_;
  std::vector<IonHandleData> output_ion_handle_data_;
};  // TimeLapse
