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

#define LOG_TAG "RecorderAudioSource"

#include "recorder/src/service/qmmf_audio_source.h"

#include <memory>
#include <map>
#include <string>

#include "common/utils/qmmf_log.h"
#include "include/qmmf-sdk/qmmf_codec.h"
#include "recorder/src/service/qmmf_audio_track_source.h"
#include "recorder/src/service/qmmf_recorder_common.h"

namespace qmmf {
namespace recorder {

using ::std::make_shared;
using ::std::map;
using ::std::shared_ptr;
using ::std::string;

AudioSource* AudioSource::instance_ = nullptr;

AudioSource* AudioSource::CreateAudioSource() {
  if(instance_ == nullptr) {
    instance_ = new AudioSource;
    if(instance_ == nullptr)
      QMMF_ERROR("%s() can't instantiate AudioSource", __func__);
  }
  QMMF_INFO("%s() AudioSource successfully retrieved", __func__);

  return instance_;
}

AudioSource::AudioSource() {
  QMMF_GET_LOG_LEVEL();
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_KPI_GET_MASK();
  QMMF_KPI_DETAIL();
}

AudioSource::~AudioSource() {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_KPI_DETAIL();

  if (!track_source_map_.empty())
    track_source_map_.clear();

  instance_ = nullptr;
}

status_t AudioSource::CreateTrackSource(const uint32_t track_id,
                                        AudioTrackParams& params) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);
  QMMF_VERBOSE("%s() INPARAM: params[%s]", __func__,
               params.ToString().c_str());

  AudioTrackSourceMap::iterator track_source_iterator =
      track_source_map_.find(track_id);
  if (track_source_iterator != track_source_map_.end()) {
    QMMF_ERROR("%s() track already exists for track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  if (params.params.format == AudioFormat::kPCM) {
    shared_ptr<AudioRawTrackSource> track_source =
        make_shared<AudioRawTrackSource>(params);
    if (track_source == nullptr) {
      QMMF_ERROR("%s() could not instantiate track_source[%u]",
                 __func__, track_id);
      return ::android::NO_MEMORY;
    }

    status_t result = track_source->Init();
    if (result != ::android::NO_ERROR) {
      QMMF_ERROR("%s() track_source[%u]->Init failed: %d", __func__,
                 track_id, result);
      return result;
    }

    track_source_map_.insert({track_id,
                             shared_ptr<IAudioTrackSource>(track_source)});
  } else {
    shared_ptr<AudioEncodedTrackSource> track_source =
        make_shared<AudioEncodedTrackSource>(params);
    if (track_source == nullptr) {
      QMMF_ERROR("%s() could not instantiate track_source[%u]",
                 __func__, track_id);
      return ::android::NO_MEMORY;
    }

    status_t result = track_source->Init();
    if (result != ::android::NO_ERROR) {
      QMMF_ERROR("%s() track_source[%u]->Init failed: %d", __func__,
                 track_id, result);
      return result;
    }

    track_source_map_.insert({track_id,
                             shared_ptr<IAudioTrackSource>(track_source)});
  }

  return ::android::NO_ERROR;
}

status_t AudioSource::DeleteTrackSource(const uint32_t track_id) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);

  AudioTrackSourceMap::iterator track_source_iterator =
      track_source_map_.find(track_id);
  if (track_source_iterator == track_source_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  status_t result = track_source_iterator->second->DeInit();
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() track_source[%u]->Deinit failed: %d", __func__,
               track_id, result);
    return result;
  }

  track_source_iterator->second = nullptr;
  track_source_map_.erase(track_source_iterator->first);

  return ::android::NO_ERROR;
}

status_t AudioSource::StartTrackSource(const uint32_t track_id) {
  QMMF_DEBUG("%s(): TRACE", __func__);
  QMMF_KPI_BASE();
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);

  AudioTrackSourceMap::iterator track_source_iterator =
      track_source_map_.find(track_id);
  if (track_source_iterator == track_source_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  status_t result = track_source_iterator->second->StartTrack();
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() track_source[%u]->StartTrack failed: %d",
               __func__, track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioSource::StopTrackSource(const uint32_t track_id) {
  QMMF_DEBUG("%s(): TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);
  QMMF_KPI_BASE();

  AudioTrackSourceMap::iterator track_source_iterator =
      track_source_map_.find(track_id);
  if (track_source_iterator == track_source_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  status_t result = track_source_iterator->second->StopTrack();
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() track_source[%u]->StopTrack failed: %d",
               __func__, track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioSource::PauseTrackSource(const uint32_t track_id) {
  QMMF_DEBUG("%s(): TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);
  QMMF_KPI_DETAIL();

  AudioTrackSourceMap::iterator track_source_iterator =
      track_source_map_.find(track_id);
  if (track_source_iterator == track_source_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  status_t result = track_source_iterator->second->PauseTrack();
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() track_source[%u]->PauseTrack failed: %d",
               __func__, track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioSource::ResumeTrackSource(const uint32_t track_id) {
  QMMF_DEBUG("%s(): TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);
  QMMF_KPI_DETAIL();

  AudioTrackSourceMap::iterator track_source_iterator =
      track_source_map_.find(track_id);
  if (track_source_iterator == track_source_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  status_t result = track_source_iterator->second->ResumeTrack();
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() track_source[%u]->ResumeTrack failed: %d",
               __func__, track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioSource::SetParameter(const uint32_t track_id, const string& key,
                                   const string& value) {
  QMMF_DEBUG("%s(): TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);
  QMMF_VERBOSE("%s() INPARAM: key[%s]", __func__, key.c_str());
  QMMF_VERBOSE("%s() INPARAM: value[%s]", __func__, value.c_str());
  QMMF_KPI_DETAIL();

  AudioTrackSourceMap::iterator track_source_iterator =
      track_source_map_.find(track_id);
  if (track_source_iterator == track_source_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  status_t result = track_source_iterator->second->SetParameter(key, value);
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() track_source[%u]->SetParameter failed: %d",
               __func__, track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioSource::ReturnTrackBuffer(const uint32_t track_id,
                                        const std::vector<BnBuffer>& buffers) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);
  for (const BnBuffer& buffer : buffers)
    QMMF_VERBOSE("%s() INPARAM: bn_buffer[%s]", __func__,
                 buffer.ToString().c_str());

  AudioTrackSourceMap::iterator track_source_iterator =
      track_source_map_.find(track_id);
  if (track_source_iterator == track_source_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    return ::android::BAD_VALUE;
  }

  status_t result = track_source_iterator->second->ReturnTrackBuffer(buffers);
  if (result != ::android::NO_ERROR) {
    QMMF_ERROR("%s() track_source[%u]->ReturnTrackBuffer failed: %d",
               __func__, track_id, result);
    return result;
  }

  return ::android::NO_ERROR;
}

status_t AudioSource::getTrackSource(const uint32_t track_id,
    shared_ptr<IAudioTrackSource>* track_source) {
  QMMF_DEBUG("%s() TRACE", __func__);
  QMMF_VERBOSE("%s() INPARAM: track_id[%u]", __func__, track_id);

  AudioTrackSourceMap::iterator track_source_iterator =
      track_source_map_.find(track_id);
  if (track_source_iterator == track_source_map_.end()) {
    QMMF_ERROR("%s() no track exists with track_id[%u]", __func__,
               track_id);
    track_source = NULL;
    return ::android::BAD_VALUE;
  }

  *track_source = track_source_iterator->second;
  return ::android::NO_ERROR;
}

}; // namespace recorder
}; // namespace qmmf
