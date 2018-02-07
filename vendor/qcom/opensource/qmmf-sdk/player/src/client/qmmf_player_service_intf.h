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

#include <iomanip>
#include <sstream>
#include <string>
#include <map>

#include <binder/IBinder.h>
#include <binder/IServiceManager.h>
#include <binder/Parcel.h>
#include <camera/CameraMetadata.h>

#include "qmmf-sdk/qmmf_player_params.h"
#include "player/src/service/qmmf_player_common.h"


namespace qmmf {
namespace player {

using namespace android;
using ::std::setbase;
using ::std::string;
using ::std::stringstream;

#define QMMF_PLAYER_SERVICE_NAME "qmmf_player.service"

enum QMMF_PLAYER_SERVICE_CMDS {
  PLAYER_CONNECT = IBinder::FIRST_CALL_TRANSACTION,
  PLAYER_DISCONNECT,
  PLAYER_CREATE_AUDIOTRACK,
  PLAYER_CREATE_VIDEOTRACK,
  PLAYER_DELETE_AUDIOTRACK,
  PLAYER_DELETE_VIDEOTRACK,
  PLAYER_SET_AUDIOTRACK_PARAMS,
  PLAYER_SET_VIDEOTRACK_PARAMS,
  PLAYER_DEQUEUE_INPUT_BUFFER,
  PLAYER_QUEUE_INPUT_BUFFER,
  PLAYER_PREPARE,
  PLAYER_START,
  PLAYER_STOP,
  PLAYER_PAUSE,
  PLAYER_RESUME,
  PLAYER_SET_POSITION,
  PLAYER_SET_TRICKMODE,
  PLAYER_GRAB_PICTURE,
};


typedef struct BnTrackBuffer {
  uint32_t  ion_fd;
  uint64_t  size;
  int64_t   timestamp;
  uint32_t  width;
  uint32_t  height;
  uint32_t  buffer_id;
  uint32_t  flag;
  uint64_t  capacity;

  string ToString() const {
    stringstream stream;
    stream << "ion_fd[" << ion_fd << "] ";
    stream << "size[" << size << "] ";
    stream << "timestamp[" << timestamp << "] ";
    stream << "width[" << width << "] ";
    stream << "height[" << height << "] ";
    stream << "buffer_id[" << buffer_id << "] ";
    stream << "flag[" << setbase(16) << flag << setbase(10) << "]";
    stream << "capacity[" << capacity << "] ";
    return stream.str();
  }

  void ToParcel(Parcel* parcel, bool writeFileDescriptor) const {
    if (writeFileDescriptor)
      parcel->writeFileDescriptor(ion_fd);
    else
      parcel->writeUint32(ion_fd);
    parcel->writeUint64(size);
    parcel->writeInt64(timestamp);
    parcel->writeUint32(width);
    parcel->writeUint32(height);
    parcel->writeUint32(buffer_id);
    parcel->writeUint32(flag);
    parcel->writeUint64(capacity);
  }

  void FromParcel(const Parcel& parcel, bool readFileDescriptor) {
    if (readFileDescriptor)
      ion_fd = parcel.readFileDescriptor();
    else
      ion_fd = parcel.readUint32();
    size = parcel.readUint64();
    timestamp = parcel.readInt64();
    width = parcel.readUint32();
    height = parcel.readUint32();
    buffer_id = parcel.readUint32();
    flag = parcel.readUint32();
    capacity = parcel.readUint64();
  }
} BnTrackBuffer;


class IPlayerServiceCallback;
class IPlayerService : public IInterface {
 public:
  DECLARE_META_INTERFACE(PlayerService);

  virtual status_t Connect(const sp<IPlayerServiceCallback>& service_cb) = 0;

  virtual status_t Disconnect() = 0;

  virtual status_t CreateAudioTrack(
      uint32_t track_id,
      AudioTrackCreateParam& param) = 0;

  virtual status_t CreateVideoTrack(
      uint32_t track_id,
      VideoTrackCreateParam& param) = 0;

  virtual status_t DeleteAudioTrack(uint32_t track_id) = 0;

  virtual status_t DeleteVideoTrack(uint32_t track_id) = 0;

  virtual status_t Prepare() = 0;

  virtual status_t DequeueInputBuffer(
      uint32_t track_id,
      std::vector<AVCodecBuffer>& buffers) = 0;

  virtual status_t QueueInputBuffer(
      uint32_t track_id,
      std::vector<AVCodecBuffer>& buffers,
      void *meta_param,
      size_t meta_size,
      TrackMetaBufferType meta_type) = 0;

  virtual status_t Start() = 0;

  virtual status_t Stop(bool do_flush) = 0;

  virtual status_t Pause() = 0;

  virtual status_t Resume() = 0;

  virtual status_t SetPosition(int64_t seek_time) = 0;

  virtual status_t SetTrickMode(TrickModeSpeed speed, TrickModeDirection dir) = 0;

  virtual status_t GrabPicture(PictureParam param) = 0;

  virtual status_t SetAudioTrackParam(uint32_t track_id,
                                      CodecParamType type,
                                      void *param,
                                      size_t param_size) = 0;

  virtual status_t SetVideoTrackParam(uint32_t track_id,
                                      CodecParamType type,
                                      void *param,
                                      size_t param_size) = 0;
};


enum PLAYER_SERVICE_CB_CMDS {
  PLAYER_NOTIFY_EVENT=IBinder::FIRST_CALL_TRANSACTION,
  PLAYER_NOTIFY_VIDEO_TRACK_DATA,
  PLAYER_NOTIFY_VIDEO_TRACK_EVENT,
  PLAYER_NOTIFY_AUDIO_TRACK_DATA,
  PLAYER_NOTIFY_AUDIO_TRACK_EVENT,
  PLAYER_NOTIFY_GRAB_PICTURE_DATA,
};

//Binder interface for callbacks from PlayerService to PlayerClient.
class IPlayerServiceCallback : public IInterface {
 public:
  DECLARE_META_INTERFACE(PlayerServiceCallback);

  virtual void NotifyPlayerEvent(EventType event_type, void *event_data,
                                 size_t event_data_size) = 0;

  virtual void NotifyVideoTrackData(uint32_t track_id,
                                    std::vector<BnTrackBuffer> &buffers,
                                    void *meta_param,
                                    TrackMetaBufferType meta_type,
                                    size_t meta_size) = 0;

  virtual void NotifyVideoTrackEvent(uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) = 0;

  virtual void NotifyAudioTrackData(uint32_t track_id,
                                    const std::vector<BnTrackBuffer>& buffers,
                                    void *meta_param,
                                    TrackMetaBufferType meta_type,
                                    size_t meta_size) = 0;

  virtual void NotifyAudioTrackEvent(uint32_t track_id, EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) = 0;

  virtual void NotifyGrabPictureData(BufferDescriptor& buffer) = 0;

  // This method is not exposed to client as a callback, it is just to update
  // Internal data structure, ServiceCallbackHandler is not forced to implement
  // this method.
  virtual void NotifyDeleteVideoTrack(uint32_t track_id
      __attribute__((__unused__))) {}

  virtual void NotifyDeleteAudioTrack(uint32_t track_id
      __attribute__((__unused__))) {}
};

// This class is responsible to provide callbacks from player service.
class BnPlayerServiceCallback : public BnInterface<IPlayerServiceCallback> {
 public:
  virtual status_t onTransact(uint32_t code, const Parcel& data,
                              Parcel* reply, uint32_t flags = 0) override;
};


};  // namespace player
};  // namespace qmmf
