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

#include <utils/List.h>
#include <utils/Mutex.h>

#include "qmmf-sdk/qmmf_recorder_params.h"
#include "common/qmmf_common_utils.h"
#include "common/qmmf_log.h"

#include "recorder/src/client/qmmf_recorder_service_intf.h"
#include "common/cameraadaptor/qmmf_camera3_device_client.h"
#include "recorder/src/service/qmmf_remote_cb.h"

#define FRAME_DUMP_PATH        "/data/misc/qmmf"

// Enable ENABLE_FRAME_DUMP to dump YUV frame at TrackSource level. it will
// Start dumping every 100th frame for all active tracks, and file name
// Would be track_(track_id)_(timestamp).yuv
//#define ENABLE_FRAME_DUMP

#define FPS_TIME_INTERVAL 3000000

//#define NO_FRAME_PROCESS

#define BUFFER_WAIT_TIMEOUT 500000000  // 500 ms

// Enable DUMP_BITSTREAM to enable encoded data at TrackEncoder layer.
//#define DUMP_BITSTREAM

// Prop to enable debugging FPS
#define PROP_DEBUG_FPS        "persist.qmmf.rec.debug.fps"

namespace qmmf {

namespace recorder {

using namespace cameraadaptor;

enum class TrackType {
  kVideo,
  kAudio
};

enum class CameraStreamFormat {
  kNV12,
  kNV21,
  kRAW10,
  kRAW12,
};

struct CameraStreamDim {
    uint32_t width;
    uint32_t height;
};

typedef std::function<void(std::vector<BnBuffer>& buffers,
    std::vector<MetaData>& meta_buffers)> buffer_callback;

typedef std::function<void(uint32_t camera_id, uint32_t image_sequence_count,
    BnBuffer& buffer, MetaData& meta_data)>  SnapshotCb;

typedef std::function<void(uint32_t image_sequence_count,
    StreamBuffer& buffer)> StreamSnapshotCb;

typedef std::function<void(uint32_t camera_id,
    const CameraMetadata &result)> ResultCb;

typedef std::function< const sp<RemoteCallBack>& (uint32_t client_id)>
    RemoteCallbackHandle;

typedef std::function<void(RecorderErrorData &error)> ErrorCb;

struct VideoTrackParams {
  VideoTrackCreateParam  params;
  VideoExtraParam        extra_param;
  uint32_t               track_id;
  buffer_callback        data_cb;
};

struct AudioTrackParams {
  AudioTrackCreateParam  params;
  uint32_t               track_id;
  buffer_callback        data_cb;

  string ToString() const {
    stringstream stream;
    stream << "params[" << params.ToString() << "] ";
    stream << "track_id[" << track_id << "] ";
    return stream.str();
  }
};

struct CameraStreamParam {
  CameraStreamDim    cam_stream_dim;
  CameraStreamFormat cam_stream_format;
  float              frame_rate;
  uint32_t           id;
  bool               low_power_mode;
};

struct Buffer {
  CameraStreamParam  stream_param;
  StreamBuffer       stream_buffer;
};

}; //namespace recorder.

}; //namespace qmmf.
