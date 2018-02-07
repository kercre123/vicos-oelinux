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

#pragma once

#include <sys/types.h>

#include <cstdint>
#include <functional>
#include <vector>

#include "include/qmmf-sdk/qmmf_codec.h"
#include "include/qmmf-sdk/qmmf_device.h"
#include "qmmf-sdk/qmmf_buffer.h"

namespace qmmf {
namespace player {

typedef struct TrackBuffer {
    void *data;
    size_t size;
    size_t filled_size;
    uint64_t time_stamp;
    uint32_t flag;
    uint32_t buf_id;
} TrackBuffer;

enum class TrackMetaBufferType {
    kNone,
    kVideoCrop,
    kMultipleFrame
};

// When TrackMetaBufferType is set kVideoCrop
// in QueueInputBuffer API, meta_buffer can be used
// to specify the crop parameters for display
typedef struct VideoStreamCrop {
    uint32_t x, y;
    uint32_t width, height;
} VideoStreamCrop;

enum class EventType {
    kError,
    kStateChanged,
    kEOSRendered,
    kInputBufferNotify
};

enum class VideoCodecType {
    kHEVC,
    kAVC,
    kJPEG
};

enum class AudioCodecType {
    kPCM,
    kAAC,
    kAMR,
    kG711
};

// Video track create time parameters
// buffer_size and num_buffers is an optional parameter if the clients
// know what the optimal size for the track input buffers are.
// If the track wants to make the player to make a decision on number
// of buffers to allocation and size - set these two values to 0
typedef struct VideoTrackCreateParam {
    size_t buffer_size;
    uint32_t num_buffers;
    uint32_t width;
    uint32_t height;
    uint32_t frame_rate;
    uint32_t bitrate;
    bool enable_downscalar;
    uint32_t output_height;
    uint32_t output_width;
    bool enable_vqzip_extradata;
    VideoCodecType codec;
    VideoOutSubtype out_device;

    ::std::string ToString() const {
      ::std::stringstream stream;
      stream << "buffer_size[" << buffer_size << "] ";
      stream << "num_buffers[" << num_buffers << "] ";
      stream << "width[" << width << "] ";
      stream << "height[" << height << "] ";
      stream << "frame_rate[" << frame_rate << "] ";
      stream << "bitrate[" << bitrate << "] ";
      stream << "enable_downscalar[" << ::std::boolalpha << enable_downscalar
             << ::std::noboolalpha << "] ";
      if (enable_downscalar) {
        stream << "output_width[" << output_width << "] ";
        stream << "output_height[" << output_height << "] ";
      }
      stream << "enable_vqzip_extradata[" << ::std::boolalpha
             << enable_vqzip_extradata << ::std::noboolalpha <<"] ";
      stream << "VideoCodecType["
             << static_cast<::std::underlying_type<VideoCodecType>::type>(codec)
             << "] ";
      stream << "out_device["
             << static_cast<::std::underlying_type<VideoOutSubtype>::type>
                           (out_device)
             << "]";
      return stream.str();
    }
} VideoTrackCreateParam;

// Audio track create time parameters
// buffer_size and num_buffers is an optional parameter if the clients
// know what the optimal size for the track input buffers are.
// If the track wants to make the player to make a decision on number
// of buffers to allocation and size - set these two values to 0
typedef struct AudioTrackCreateParam {
    size_t buffer_size;
    uint32_t num_buffers;
    uint32_t sample_rate;
    uint32_t channels;
    uint32_t bit_depth;
    uint32_t bitrate;
    AudioCodecType codec;
    AudioCodecParams codec_params;
    AudioOutSubtype out_device;
} AudioTrackCreateParam;

typedef struct PictureParam {
    uint32_t width;
    uint32_t height;
    uint32_t quality;
} PictureParam;

enum class TrickModeSpeed {
  kSpeed_1x = 1 << 0,
  kSpeed_2x = 1 << 1,
  kSpeed_4x = 1 << 2,
  kSpeed_8x = 1 << 3,
};

enum class TrickModeDirection {
  kNormalForward  = 1,    // normal forward means 1x forward i.e. normal playback
  kFastForward    = 2,
  kSlowForward    = 3,
  kNormalRewind   = 4,    // normal rewind means 1x rewind
  kFastRewind     = 5,
  kSlowRewind     = 6,
};

typedef struct PlayerCb {
    std::function<void( EventType event_type,
                        void *event_data,
                        size_t event_data_size)> event_cb;
} PlayerCb;

typedef struct TrackCb {
    std::function<void( EventType event_type,
                        void *event_data,
                        size_t event_data_size)> event_cb;
} TrackCb;

typedef struct PictureCallback {
    std::function<void( EventType event_type,
                        void *event_data,
                        size_t event_data_size)> event_cb;
    std::function<void(BufferDescriptor& buffer)> data_cb;
} PictureCallback;

};
}; //namespace qmmf:player
