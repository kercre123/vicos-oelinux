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

#include <assert.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <cutils/native_handle.h>
#include <linux/msm_ion.h>
#include <utils/Log.h>

#include "common/qmmf_common_utils.h"
#include "common/qmmf_log.h"
#include "player/test/demuxer/qmmf_demuxer_intf.h"
#include "player/test/demuxer/qmmf_demuxer_mediadata_def.h"
#include "player/test/demuxer/qmmf_demuxer_sourceport.h"
#include "qmmf-sdk/qmmf_avcodec.h"
#include "qmmf-sdk/qmmf_avcodec_params.h"
#include "qmmf-sdk/qmmf_buffer.h"
#include "qmmf-sdk/qmmf_codec.h"
#include "qmmf-sdk/qmmf_player_params.h"
#include "qmmf-sdk/qmmf_recorder_params.h"

namespace qmmf {
namespace transcode {

typedef struct ion_allocation_data IonHandleData;
typedef int32_t status_t;

// specifies the time period in microseconds
static const uint64_t kFrameRatePeriod = 3000000;

static const uint32_t kNotRequired = 0;

enum class TranscodeType {
  kVideoDecodeVideoEncode,
  kVideoEncodeVideoDecode,
  kImageDecodeVideoEncode,
};

enum class BufferOwner {
  kTranscoderSource,
  kTranscoderSink,
  kTranscoderPipeIn,
  kTranscoderPipeOut,
  kVQZipInputPort,
  kVQZipOutputPort
};

static inline uint32_t OwnerIndex(const BufferOwner owner) {
  uint32_t index = 0xffffffff;
  switch (owner) {
    case BufferOwner::kTranscoderSource:
      index = 0x00000000;
      break;
    case BufferOwner::kTranscoderSink:
      index = 0x01000000;
      break;
    case BufferOwner::kTranscoderPipeIn:
      index = 0x10000000;
      break;
    case BufferOwner::kTranscoderPipeOut:
      index = 0x11000000;
      break;
    case BufferOwner::kVQZipInputPort:
      index = 0x00100000;
      break;
    case BufferOwner::kVQZipOutputPort:
      index = 0x01100000;
      break;
    default:
      QMMF_ERROR("%s: Unknown BufferOwner", __func__);
  }
  return index;
}

enum class TrackTypes {
  kAudioVideo,
  kAudioOnly,
  kVideoOnly,
  kInvalid,
};

struct BufInfo {
  uint32_t capacity;
  uint32_t buf_size;
  int32_t fd;
  void* vaddr;
  IonHandleData ion_handle;
};

enum class AVCodecBufferType {
  kNormal,
  kNativeHandle,
};

class TSInt32 {
 public:
  TSInt32() {
    ::std::lock_guard<::std::mutex> lg(value_mutex_);
    value_ = 0;
  }

  inline TSInt32& operator=(const int32_t t) {
    ::std::lock_guard<::std::mutex> lg(value_mutex_);
    value_ = t;
    return *this;
  }

  inline int32_t value() {
    ::std::lock_guard<::std::mutex> lg(value_mutex_);
    return value_;
  }

  inline TSInt32& operator++(int) {
    ::std::lock_guard<::std::mutex> lg(value_mutex_);
    value_++;
    return *this;
  }

  inline TSInt32& operator--(int) {
    ::std::lock_guard<::std::mutex> lg(value_mutex_);
    value_--;
    return *this;
  }

 private:
  int32_t value_;
  ::std::mutex value_mutex_;
};

struct TranscodeParams {
  TranscodeType                  track_type;
  bool                           enable_vqzip;
  ::std::string                  track_file;
  ::std::string                  input_file;
  ::std::string                  output_file;
  ::qmmf::CodecType              source_codec_type;
  ::qmmf::avcodec::CodecParam    source_params;
  ::qmmf::CodecType              sink_codec_type;
  ::qmmf::avcodec::CodecParam    sink_params;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "track_type["
           << static_cast<::std::underlying_type<TranscodeType>::type>
                         (track_type)
           << "] ";
    stream << "track_file[" << track_file << "] ";
    stream << "input_file[" << input_file << "] ";
    stream << "output_file[" << output_file << "] ";
    stream << "source_codec_type["
           << static_cast<::std::underlying_type<CodecType>::type>
                         (source_codec_type)
           << "] ";
    stream << "EnableVqzip[" << ::std::boolalpha << enable_vqzip
           << ::std::noboolalpha << "] ";
    stream << "source_params[";
    switch (source_codec_type) {
      case CodecType::kVideoDecoder:
        stream <<  source_params.video_dec_param.ToString();
        break;
      case CodecType::kVideoEncoder:
        stream << source_params.video_enc_param.ToString();
        break;
      default:
        stream << "UnknownCodecType";
    }
    stream << "] ";
    stream << "sink_codec_type["
           << static_cast<::std::underlying_type<CodecType>::type>
                         (sink_codec_type)
           << "] ";
    stream << "sink_params[";
    switch (sink_codec_type) {
      case CodecType::kVideoDecoder:
        stream << sink_params.video_dec_param.ToString();
        break;
      case CodecType::kVideoEncoder:
        stream << sink_params.video_enc_param.ToString();
        break;
      default:
        stream << "UnknownCodecType";
    }
    stream << "]";
    return stream.str();
  }
};

class TranscodeBuffer {
 public:
  TranscodeBuffer() { num_instances_++; }
  TranscodeBuffer& operator=(const TranscodeBuffer& rhs);
  TranscodeBuffer(const TranscodeBuffer& obj);
  TranscodeBuffer(const BufferOwner owner, const uint32_t buf_id);
  ~TranscodeBuffer();

  static status_t CreateTranscodeBuffersVector(
      const ::std::shared_ptr<::qmmf::avcodec::IAVCodec>& avcodec,
      const BufferOwner owner, const uint32_t port_index,
      ::std::vector<TranscodeBuffer>* list);

  static void FreeTranscodeBuffersVector(::std::vector<TranscodeBuffer>* list);

  inline status_t getAVCodecBuffer(const AVCodecBufferType type,
                                   BufferDescriptor* buffer) {
    if (buffer == nullptr) {
      QMMF_ERROR("TranscodeBuffer:%s Invalid Parameters, buffer = nullptr",
                 __func__);
      return -1;
    }

    if (type == AVCodecBufferType::kNormal) {
      buffer->data = buf_info_.vaddr;
      buffer->fd = buf_info_.fd;
      buffer->buf_id = buf_id_;
      buffer->size = filled_size_;
      buffer->capacity = buf_info_.capacity;
      buffer->offset = offset_;
      buffer->timestamp = timestamp_;
      buffer->flag = flag_;
    } else if (type == AVCodecBufferType::kNativeHandle) {
      buffer->data = reinterpret_cast<void*>(meta_handle_);
      buffer->fd = buf_info_.fd;
      buffer->buf_id = buf_id_;
      buffer->size = filled_size_;
      buffer->capacity = buf_info_.capacity;
      buffer->offset = offset_;
      buffer->timestamp = timestamp_;
      buffer->flag = flag_;
    } else {
      QMMF_ERROR("TranscodeBuffer:%s Unknown AVCodecBufferType", __func__);
      return -1;
    }
    return 0;
  }

  inline void UpdateTranscodeBuffer(const BufferDescriptor& buffer) {
    timestamp_ = buffer.timestamp;
    flag_ = buffer.flag;
    filled_size_ = buffer.size;
    offset_ = buffer.offset;
  }

  inline void* GetMetaHandle() const {
    return reinterpret_cast<void*>(meta_handle_);
  }

  inline void* GetVaddr() const { return buf_info_.vaddr; }
  inline uint64_t GetTimestamp() { return timestamp_; }
  inline uint32_t GetFilledSize() const { return filled_size_; }
  inline uint32_t GetFlag() const { return flag_; }
  inline uint32_t GetOffset() const { return offset_; }
  inline uint32_t GetBufId() const { return buf_id_; }
  inline int32_t GetFd() const { return buf_info_.fd; }
  inline BufferOwner GetOwner() const { return owner_; }

  inline void SetTimestamp(const uint64_t value) { timestamp_ = value; }
  inline void SetFilledSize(const uint32_t value) { filled_size_ = value; }
  inline void SetFlag(const uint32_t value) { flag_ = value; }
  inline void SetOffset(const uint32_t value) { offset_ = value; }

 private:
  status_t Allocate(const uint32_t size);
  void Release();

  native_handle_t*           meta_handle_;
  BufferOwner                owner_;
  BufInfo                    buf_info_;
  uint32_t                   buf_id_;
  uint64_t                   timestamp_;
  uint32_t                   flag_;
  uint32_t                   filled_size_;
  uint32_t                   offset_;
  static int32_t             ion_device_;
  static TSInt32             num_instances_;
};  // class TranscodeBuffer

class FramerateCalculator {
 public:
  FramerateCalculator(uint64_t period, ::std::string str);
  ~FramerateCalculator();
  void Trigger();
  void Reset();

 private:
  uint32_t                   count_;
  uint64_t                   period_;
  uint64_t                   time_diff_;
  ::std::string              log_str_;
  bool                       is_first_trigger_;
  struct timeval             prevtv_;
  struct timeval             currtv_;
};  // class FrameRateCalculator

class VQZipInfoExtractor {
public:
  VQZipInfoExtractor(const ::qmmf::player::VideoTrackCreateParam& param,
                     const MM_TRACK_INFOTYPE& track_info,
                     CMM_MediaDemuxInt* const demuxer);
  ~VQZipInfoExtractor();

  status_t Init();
  status_t DeInit();
  status_t ExtractVQZipInfo(VQZipInfo* vqzip_info);

  void ReadNextFrame(TSQueue<TranscodeBuffer>::iterator buffer);
  void NotiftyFBD(const TranscodeBuffer& buffer);

private:
  void ReleaseResources();

  class InputCodecSourceImpl:
    public ::qmmf::avcodec::ICodecSource,
    public ::std::enable_shared_from_this<InputCodecSourceImpl> {
  public:
    InputCodecSourceImpl(
        const ::std::shared_ptr<::qmmf::avcodec::IAVCodec>& avcodec,
        VQZipInfoExtractor* const src);
    ~InputCodecSourceImpl();

    status_t Prepare();
    void ReleaseBuffer();

    status_t GetBuffer(BufferDescriptor& buffer_descriptor,
                       void* client_data) override;
    status_t ReturnBuffer(BufferDescriptor& buffer_descriptor,
                          void* client_data) override;
    status_t NotifyPortEvent(::qmmf::avcodec::PortEventType event_type,
                             void* event_data) override;
  private:
    ::std::vector<TranscodeBuffer>                 buffer_list_;
    ::std::shared_ptr<::qmmf::avcodec::IAVCodec>   avcodec_;
    VQZipInfoExtractor*                            source_;
    TSQueue<TranscodeBuffer>                       free_buffer_queue_;
    TSQueue<TranscodeBuffer>                       occupy_buffer_queue_;
    ::std::mutex                                   wait_for_frame_mutex_;
    ::std::condition_variable                      wait_for_frame_;
  };

  class OutputCodecSourceImpl:
    public ::qmmf::avcodec::ICodecSource,
    public ::std::enable_shared_from_this<OutputCodecSourceImpl> {
  public:
    OutputCodecSourceImpl(
        const ::std::shared_ptr<::qmmf::avcodec::IAVCodec>& avcodec,
        VQZipInfoExtractor* const sink);
    ~OutputCodecSourceImpl();

    status_t Prepare();
    void ReleaseBuffer();

    status_t GetBuffer(BufferDescriptor& buffer_descriptor,
                       void* client_data) override;
    status_t ReturnBuffer(BufferDescriptor& buffer_descriptor,
                          void* client_data) override;
    status_t NotifyPortEvent(::qmmf::avcodec::PortEventType event_type,
                             void* event_data) override;
  private:
    ::std::vector<TranscodeBuffer>                 buffer_list_;
    ::std::shared_ptr<::qmmf::avcodec::IAVCodec>   avcodec_;
    VQZipInfoExtractor*                            sink_;
    TSQueue<TranscodeBuffer>                       free_buffer_queue_;
    TSQueue<TranscodeBuffer>                       occupy_buffer_queue_;
    ::std::mutex                                   wait_for_frame_mutex_;
    ::std::condition_variable                      wait_for_frame_;
  };

  ::std::shared_ptr<::qmmf::avcodec::IAVCodec>     avcodec_;
  ::qmmf::player::VideoTrackCreateParam            avcodec_decode_params_;
  ::std::shared_ptr<InputCodecSourceImpl>          input_source_impl_;
  ::std::shared_ptr<OutputCodecSourceImpl>         output_source_impl_;
  MM_TRACK_INFOTYPE                                m_sTrackInfo_;
  CMM_MediaDemuxInt*                               m_pDemux_;
  ::std::mutex                                     wait_for_fbd_mutex_;
  ::std::condition_variable                        wait_for_fbd_;
  bool                                             isFirstFrame_;
}; // class VQZipInfoExtractor

};  // namespace transcode
};  // namespace qmmf
