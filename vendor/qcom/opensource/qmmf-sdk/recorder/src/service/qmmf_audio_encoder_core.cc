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

#define LOG_TAG "RecorderAudioEncoderCore"

#include "recorder/src/service/qmmf_audio_encoder_core.h"

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <queue>

#include "common/utils/qmmf_log.h"
#include "common/utils/qmmf_condition.h"
#include "common/codecadaptor/src/qmmf_avcodec.h"
#include "recorder/src/service/qmmf_audio_track_source.h"
#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_recorder_ion.h"

namespace qmmf {
namespace recorder {

using ::qmmf::avcodec::AVCodec;
using ::qmmf::avcodec::CodecParam;
using ::qmmf::avcodec::CodecPortStatus;
using ::qmmf::avcodec::PortEventType;
using ::qmmf::avcodec::PortreconfigData;
using ::qmmf::avcodec::kPortIndexInput;
using ::qmmf::avcodec::kPortIndexOutput;
using ::std::chrono::seconds;
using ::std::static_pointer_cast;
using ::std::map;
using ::std::make_shared;
using ::std::mutex;
using ::std::queue;
using ::std::shared_ptr;
using ::std::unique_lock;
using ::std::vector;

AudioEncoderCore* AudioEncoderCore::instance_ = nullptr;

AudioEncoderCore* AudioEncoderCore::CreateAudioEncoderCore() {
  if(instance_ == nullptr) {
    instance_ = new AudioEncoderCore;
    if(instance_ == nullptr)
      QMMF_ERROR("%s() can't instantiate AudioEncoderCore", __func__);
  }
  QMMF_INFO("%s() AudioEncoderCore successfully retrieved", __func__);

  return instance_;
}

AudioEncoderCore::AudioEncoderCore() {
  QMMF_GET_LOG_LEVEL();
  QMMF_DEBUG("%s:() TRACE", __func__);
  QMMF_KPI_GET_MASK();
  QMMF_KPI_DETAIL();
}

AudioEncoderCore::~AudioEncoderCore() {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_KPI_DETAIL();

  if (!track_encoder_map_.empty())
    track_encoder_map_.clear();

  instance_ = nullptr;
}

status_t AudioEncoderCore::AddSource(const AudioTrackParams& params) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: params[%s]", __func__,
               params.ToString().c_str());

  shared_ptr<AudioTrackEncoder> track_encoder = make_shared<AudioTrackEncoder>();
  if (track_encoder.get() == nullptr) {
    QMMF_ERROR("%s() could not instantiate track encoder", __func__);
    return ::android::NO_MEMORY;
  }

  status_t result = track_encoder->Init(params);
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() track_encoder[%u]->Init failed: %d", __func__,
               params.track_id, result);
    return result;
  }

  std::lock_guard<std::mutex> l(track_encoder_map_lock_);
  track_encoder_map_.insert({params.track_id, track_encoder});

  return ::android::NO_ERROR;
}

status_t AudioEncoderCore::DeleteTrackEncoder(const uint32_t track_id) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);

  std::lock_guard<std::mutex> l(track_encoder_map_lock_);
  AudioTrackEncoderMap::iterator track_encoder_iterator =
      track_encoder_map_.find(track_id);
  if (track_encoder_iterator == track_encoder_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  track_encoder_map_.erase(track_encoder_iterator->first);

  return ::android::NO_ERROR;
}

status_t AudioEncoderCore::StartTrackEncoder(const uint32_t track_id,
                                             const shared_ptr<IAudioTrackSource>&
                                             track_source) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_KPI_BASE();
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);

  shared_ptr<AudioTrackEncoder> track_encoder;
  AudioTrackEncoderMap::iterator track_encoder_iterator;
  {
    std::lock_guard<std::mutex> l(track_encoder_map_lock_);
    track_encoder_iterator = track_encoder_map_.find(track_id);
    if (track_encoder_iterator == track_encoder_map_.end()) {
      QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
                 track_id);
      return ::android::BAD_VALUE;
    }
    track_encoder = track_encoder_iterator->second;
  }

  status_t result = track_encoder->Start(
      static_pointer_cast<AudioEncodedTrackSource>(track_source),
      track_encoder);
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() track_encoder[%u]->Start failed: %d", __func__,
               track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioEncoderCore::StopTrackEncoder(const uint32_t track_id) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);
  QMMF_KPI_BASE();

  shared_ptr<AudioTrackEncoder> track_encoder;
  AudioTrackEncoderMap::iterator track_encoder_iterator;
  {
    std::lock_guard<std::mutex> l(track_encoder_map_lock_);
    track_encoder_iterator = track_encoder_map_.find(track_id);
    if (track_encoder_iterator == track_encoder_map_.end()) {
      QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
                 track_id);
      return ::android::BAD_VALUE;
    }
    track_encoder = track_encoder_iterator->second;
  }

  status_t result = track_encoder->Stop();
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() track_encoder[%u]->Stop failed: %d", __func__,
               track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioEncoderCore::PauseTrackEncoder(const uint32_t track_id) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);
  QMMF_KPI_DETAIL();

  shared_ptr<AudioTrackEncoder> track_encoder;
  AudioTrackEncoderMap::iterator track_encoder_iterator;
  {
    std::lock_guard<std::mutex> l(track_encoder_map_lock_);
    track_encoder_iterator = track_encoder_map_.find(track_id);
    if (track_encoder_iterator == track_encoder_map_.end()) {
      QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
                 track_id);
      return ::android::BAD_VALUE;
    }
    track_encoder = track_encoder_iterator->second;
  }

  status_t result = track_encoder->Pause();
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() track_encoder[%u]->Pause failed: %d", __func__,
               track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioEncoderCore::ResumeTrackEncoder(const uint32_t track_id) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);
  QMMF_KPI_DETAIL();

  shared_ptr<AudioTrackEncoder> track_encoder;
  AudioTrackEncoderMap::iterator track_encoder_iterator;
  {
    std::lock_guard<std::mutex> l(track_encoder_map_lock_);
    track_encoder_iterator = track_encoder_map_.find(track_id);
    if (track_encoder_iterator == track_encoder_map_.end()) {
      QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
                 track_id);
      return ::android::BAD_VALUE;
    }
    track_encoder = track_encoder_iterator->second;
  }

  status_t result = track_encoder->Resume();
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() track_encoder[%u]->Resume failed: %d", __func__,
               track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioEncoderCore::SetTrackEncoderParam(const uint32_t track_id,
    const CodecParamType param_type, void* param, const uint32_t param_size) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);
  QMMF_VERBOSE("%s() INPARAM: param_type[%d]", __func__,
               static_cast<int>(param_type));
  QMMF_VERBOSE("%s() INPARAM: param[%p]", __func__, param);
  QMMF_VERBOSE("%s() INPARAM: param_size[%u]", __func__, param_size);

  shared_ptr<AudioTrackEncoder> track_encoder;
  AudioTrackEncoderMap::iterator track_encoder_iterator;
  {
    std::lock_guard<std::mutex> l(track_encoder_map_lock_);
    track_encoder_iterator =track_encoder_map_.find(track_id);
    if (track_encoder_iterator == track_encoder_map_.end()) {
      QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
                 track_id);
      return ::android::BAD_VALUE;
    }
    track_encoder = track_encoder_iterator->second;
  }

  status_t result = track_encoder->SetParam(param_type, param,
                                                             param_size);
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() track_encoder[%u]->SetParam failed: %d", __func__,
               track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioEncoderCore::ReturnTrackBuffer(const uint32_t track_id,
    const std::vector<BnBuffer>& buffers) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);
  for (const BnBuffer& buffer : buffers)
    QMMF_VERBOSE("%s() INPARAM: bn_buffer[%s]", __func__,
                 buffer.ToString().c_str());

  shared_ptr<AudioTrackEncoder> track_encoder;
  AudioTrackEncoderMap::iterator track_encoder_iterator;
  {
    std::lock_guard<std::mutex> l(track_encoder_map_lock_);
    track_encoder_iterator = track_encoder_map_.find(track_id);
    if (track_encoder_iterator == track_encoder_map_.end()) {
      QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
                 track_id);
      return ::android::BAD_VALUE;
    }
    track_encoder = track_encoder_iterator->second;
  }

  status_t result = track_encoder->OnBufferReturnFromClient(buffers);
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() track_encoder[%u]->OnBufferReturnFromClient failed: %d",
               __func__, track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

AudioTrackEncoder::AudioTrackEncoder()
    : avcodec_(nullptr),
      eos_received_(false) {
  QMMF_GET_LOG_LEVEL();
  QMMF_DEBUG("%s:() TRACE", __func__);
}

AudioTrackEncoder::~AudioTrackEncoder() {
  QMMF_DEBUG("%s() TRACE", __func__);

  int32_t iresult = ion_.Deallocate();
  if (iresult < 0)
    QMMF_ERROR("%s() ion->Deallocate failed: %d[%s]", __func__,
               iresult, strerror(iresult));
}

status_t AudioTrackEncoder::Init(const AudioTrackParams& track_params) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_params[%s]", __func__,
               track_params.ToString().c_str());

  track_params_ = track_params;

  return ::android::NO_ERROR;
}

status_t AudioTrackEncoder::Start(const shared_ptr<ICodecSource> &track_source,
                                  const shared_ptr<ICodecSource> &track_encoder) {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  status_t result;
  int32_t iresult;
  uint32_t count, size;
  vector<BufferDescriptor> dummy_list;

  eos_received_ = false;

  avcodec_ = new AVCodec();
  if(avcodec_ == nullptr) {
    QMMF_ERROR("%s() could not instantiate avcoded", __func__);
    return ::android::NO_MEMORY;
  }

  CodecParam codec_param{};
  codec_param.audio_enc_param = track_params_.params;

  result = avcodec_->ConfigureCodec(CodecMimeType::kMimeTypeAudioEncAAC,
                                    codec_param);
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() avcodec->ConfigureCodec failed: %d", __func__,
               result);
    goto error_start_avcodec;
  }

  result = avcodec_->AllocateBuffer(kPortIndexInput, 0, 0,
                                    track_source,
                                    dummy_list);
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() avcodec->AllocateBuffer(input) failed: %d",
               __func__, result);
    goto error_start_headers;
  }

  result = avcodec_->GetBufferRequirements(kPortIndexInput, &count, &size);
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() avcodec->GetBufferRequirements failed: %d",
               __func__, result);
    goto error_start_headers;
  }

  result = avcodec_->AllocateBuffer(kPortIndexOutput, 0, 0,
                                    track_encoder,
                                    dummy_list);
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() avcodec->AllocateBuffer(output) failed: %d",
               __func__, result);
    goto error_start_headers;
  }

  result = avcodec_->GetBufferRequirements(kPortIndexOutput, &count, &size);
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() avcodec->GetBufferRequirements failed: %d",
               __func__, result);
    goto error_start_headers;
  }

  ion_.Deallocate();
  iresult = ion_.Allocate(count, size);
  if (iresult < 0) {
    QMMF_ERROR("%s() ion->Allocate failed: %d[%s]", __func__, iresult,
               strerror(iresult));
    result = ::android::FAILED_TRANSACTION;
    goto error_start_ion;
  }

  while (!buffers_.empty())
    buffers_.pop();
  iresult = ion_.GetList(&buffers_);
  if (iresult < 0) {
    QMMF_ERROR("%s() ion->GetList failed: %d[%s]", __func__, iresult,
               strerror(iresult));
    result = ::android::FAILED_TRANSACTION;
    goto error_start_buffers;
  }

  result = avcodec_->StartCodec();
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() avcodec->StartCodec failed: %d", __func__,
               result);
    goto error_start_stop;
  }

  return ::android::NO_ERROR;

error_start_stop:
  avcodec_->StopCodec(true);

error_start_buffers:
  while (!buffers_.empty())
    buffers_.pop();

error_start_ion:
  ion_.Deallocate();

error_start_headers:
  avcodec_->ReleaseBuffer();

error_start_avcodec:
  delete avcodec_;
  avcodec_ = nullptr;

  return result;
}

status_t AudioTrackEncoder::Stop() {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  status_t return_value = ::android::NO_ERROR;
  status_t result;

  if (avcodec_ == nullptr) {
    QMMF_ERROR("%s() track encoder has not been initialized",
               __func__);
    return ::android::NO_INIT;
  }

  result = avcodec_->StopCodec(true);
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() avcodec->StopCodec failed: %d", __func__,
               result);
    return_value = result;
  }

  result = avcodec_->ReleaseBuffer();
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() avcodec->ReleaseBuffer failed: %d", __func__,
               result);
    return_value = result;
  }

  delete avcodec_;
  avcodec_ = nullptr;

  if (!eos_received_) {
    unique_lock<mutex> lk(mutex_);
    if (eos_signal_.WaitFor(lk, seconds(5)) != 0)
      QMMF_WARN("%s() timed out waiting for EOS-marked buffer to return",
                __func__);
  }

  return return_value;
}

status_t AudioTrackEncoder::Pause() {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);

  if (avcodec_ == nullptr) {
    QMMF_ERROR("%s() track encoder has not been initialized",
               __func__);
    return ::android::NO_INIT;
  }

  status_t result = avcodec_->PauseCodec();
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() avcodec->PauseCodec failed: %d", __func__,
               result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioTrackEncoder::Resume() {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);

  if (avcodec_ == nullptr) {
    QMMF_ERROR("%s() track encoder has not been initialized",
               __func__);
    return ::android::NO_INIT;
  }

  status_t result = avcodec_->ResumeCodec();
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() avcodec->ResumeCodec failed: %d", __func__,
               result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioTrackEncoder::SetParam(CodecParamType param_type, void* param,
                                     uint32_t param_size) {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  QMMF_VERBOSE("%s() INPARAM: param_type[%d]", __func__,
               static_cast<int>(param_type));
  QMMF_VERBOSE("%s() INPARAM: param[%p]", __func__, param);
  QMMF_VERBOSE("%s() INPARAM: param_size[%u]", __func__, param_size);

  return ::android::NO_ERROR;
}


status_t AudioTrackEncoder::GetBuffer(BufferDescriptor& codec_buffer,
                                      void* client_data) {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);

  while (buffers_.empty()) {
    unique_lock<mutex> lk(mutex_);
    if (signal_.WaitFor(lk, seconds(1)) != 0)
      QMMF_WARN("%s() timed out on wait", __func__);
  }

  mutex_.lock();
  codec_buffer = buffers_.front();
  buffers_.pop();
  mutex_.unlock();

  QMMF_VERBOSE("%s() OUTPARAM: buffer[%s]", __func__,
               codec_buffer.ToString().c_str());
  return ::android::NO_ERROR;
}

status_t AudioTrackEncoder::ReturnBuffer(BufferDescriptor& codec_buffer,
                                         void* client_data) {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  QMMF_VERBOSE("%s() INPARAM: buffer[%s]", __func__,
               codec_buffer.ToString().c_str());

  assert(codec_buffer.data != nullptr);

  BnBuffer bn_buffer{};

  int32_t result = ion_.Export(codec_buffer, &bn_buffer);
  if (result < 0) {
    QMMF_ERROR("%s() ion->Export failed: %d[%s]", __func__, result,
               strerror(result));
    return ::android::FAILED_TRANSACTION;
  }
  std::vector<BnBuffer> bn_buffers;
  bn_buffers.push_back(bn_buffer);

  MetaData meta_data{};
  meta_data.meta_flag = static_cast<uint32_t>(MetaParamType::kNone);
  std::vector<MetaData> meta_buffers;
  meta_buffers.push_back(meta_data);

  track_params_.data_cb(bn_buffers, meta_buffers);

  return ::android::NO_ERROR;
}

status_t AudioTrackEncoder::NotifyPortEvent(PortEventType event_type,
                                            void* event_data)
{
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  return ::android::NO_ERROR;
}

status_t AudioTrackEncoder::OnBufferReturnFromClient(
    const std::vector<BnBuffer>& bn_buffers) {
  QMMF_DEBUG("%s() TRACE: track_id[%u]", __func__,
             track_params_.track_id);
  for (const BnBuffer& bn_buffer : bn_buffers)
    QMMF_VERBOSE("%s() INPARAM: bn_buffer[%s]", __func__,
                 bn_buffer.ToString().c_str());

  for (const BnBuffer& bn_buffer : bn_buffers) {
    BufferDescriptor codec_buffer{};

    int32_t result = ion_.Import(bn_buffer, &codec_buffer);
    if (result < 0) {
      QMMF_ERROR("%s() ion->Import failed: %d[%s]", __func__, result,
                 strerror(result));
      return ::android::FAILED_TRANSACTION;
    }

    memset(codec_buffer.data, 0x00, codec_buffer.capacity);
    codec_buffer.size = 0;
    codec_buffer.timestamp = 0;

    mutex_.lock();
    buffers_.push(codec_buffer);
    mutex_.unlock();
    signal_.Signal();

    if (codec_buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS)) {
      QMMF_DEBUG("%s() received buffer with EOS", __func__);
      eos_mutex_.lock();
      eos_received_ = true;
      eos_mutex_.unlock();
      eos_signal_.Signal();
    }
  }

  return ::android::NO_ERROR;
}

};  // namespace recorder
};  // namespace qmmf
