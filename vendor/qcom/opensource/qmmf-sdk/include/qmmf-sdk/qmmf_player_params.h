/*
 * Copyright (c) 2016-2018, The Linux Foundation. All rights reserved.
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

#include "qmmf-sdk/qmmf_codec.h"
#include "qmmf-sdk/qmmf_device.h"
#include "qmmf-sdk/qmmf_buffer.h"

namespace qmmf {
namespace player {

typedef int32_t status_t;

struct TrackBuffer {
  void *data;
  size_t size;
  size_t filled_size;
  uint64_t time_stamp;
  uint32_t flag;
  uint32_t buf_id;
};

enum class TrackMetaBufferType {
  kNone,
  kVideoCrop,
  kMultipleFrame,
};

// When TrackMetaBufferType is set kVideoCrop
// in QueueInputBuffer API, meta_buffer can be used
// to specify the crop parameters for display
struct VideoStreamCrop {
  uint32_t x;
  uint32_t y;
  uint32_t width;
  uint32_t height;
};

enum class EventType {
  kError,
  kEOSRendered,
  kStopped,
  kInputBufferNotify,
  kPresentationTimestamp,
};

enum class VideoCodecType {
  kHEVC,
  kAVC,
  kJPEG,
  kYUV,
};

struct Rect {
  float start_x;
  float start_y;
  uint32_t width;
  uint32_t height;

  Rect()
   : start_x(0.0),
     start_y(0.0),
     width(1920),
     height(1080) {}

  Rect(uint32_t start_x, uint32_t start_y, uint32_t width, uint32_t height)
   : start_x(start_x),
     start_y(start_y),
     width(width),
     height(height) {}
};

// Video track create time parameters
// buffer_size and num_buffers is an optional parameter if the clients
// know what the optimal size for the track input buffers are.
// If the track wants to make the player to make a decision on number
// of buffers to allocation and size - set these two values to 0
struct VideoTrackCreateParam {
  size_t buffer_size;
  uint32_t num_buffers;
  uint32_t pts_callback_interval; // milliseconds; set to 0 to disable
  Rect srcRect;
  Rect destRect;
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
  uint32_t rotation;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "buffer_size[" << buffer_size << "] ";
    stream << "num_buffers[" << num_buffers << "] ";
    stream << "pts_callback_interval[" << pts_callback_interval << "] ";
    stream << "width[" << width << "] ";
    stream << "height[" << height << "] ";
    stream << "dest start_x[" << destRect.start_x << "] ";
    stream << "dest start_y[" << destRect.start_y << "] ";
    stream << "dest width[" << destRect.width << "] ";
    stream << "dest height[" << destRect.height << "] ";
    stream << "Rotation[" << rotation << "] ";
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
};

// Audio track create time parameters
// buffer_size and num_buffers is an optional parameter if the clients
// know what the optimal size for the track input buffers are.
// If the track wants to make the player to make a decision on number
// of buffers to allocation and size - set these two values to 0
struct AudioTrackCreateParam {
  size_t buffer_size;
  uint32_t num_buffers;
  uint32_t pts_callback_interval; // milliseconds; set to 0 to disable
  uint32_t sample_rate;
  uint32_t channels;
  uint32_t bit_depth;
  uint32_t bitrate;
  AudioFormat codec;
  AudioCodecParams codec_params;
  AudioOutSubtype out_device;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "buffer_size[" << buffer_size << "] ";
    stream << "num_buffers[" << num_buffers << "] ";
    stream << "pts_callback_interval[" << pts_callback_interval << "] ";
    stream << "sample_rate[" << sample_rate << "] ";
    stream << "channels[" << channels << "] ";
    stream << "bit_depth[" << bit_depth << "] ";
    stream << "bitrate[" << bitrate << "] ";
    stream << "codec["
           << static_cast<::std::underlying_type<AudioFormat>::type>(codec)
           << "] ";
    stream << "codec_params[" << codec_params.ToString(codec) << "] ";
    stream << "out_device["
           << static_cast<::std::underlying_type<AudioOutSubtype>::type>
                         (out_device)
           << "]";
    return stream.str();
  }
};

struct PictureParam {
  bool enable;
  VideoCodecType format;
  uint32_t width;
  uint32_t height;
  uint32_t quality;
};

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

struct PlayerCb {
  std::function<void(EventType event_type,
                     void *event_data,
                     size_t event_data_size)> event_cb;
};

struct TrackCb {
  std::function<void(uint32_t track_id,
                     EventType event_type,
                     void *event_data,
                     size_t event_data_size)> event_cb;
};

struct PictureCallback {
  std::function<void(uint32_t track_id,
                     EventType event_type,
                     void *event_data,
                     size_t event_data_size)> event_cb;
  std::function<void(uint32_t track_id,
                     BufferDescriptor& buffer)> data_cb;
};

}; // namespace player
}; // namespace qmmf
