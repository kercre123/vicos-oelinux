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

#include <cstddef>
#include <memory>
#include <iomanip>
#include <functional>
#include <sstream>
#include <string>
#include <cstdint>
#include <type_traits>
#include <vector>

#include <camera/CameraMetadata.h>

#include "qmmf-sdk/qmmf_buffer.h"
#include "qmmf-sdk/qmmf_codec.h"
#include "qmmf-sdk/qmmf_device.h"

namespace qmmf {

namespace recorder {

#define MAX_IN_DEVICES 4

#define SENSOR_VENDOR_MODE_OFFSET (24)
#define SENSOR_VENDOR_MODE_MASK (0xff)
#define MAX_AUDIO_INPUT_DEVICES (10)
#define MAX_THUMBNAIL_IMAGE_PARAM (2)

typedef int32_t status_t;

enum class EventType {
  kServerDied = 1,
  kCameraError = 2,
};

typedef std::function<void(EventType event_type, void *event_data,
                           size_t event_data_size)> EventCb;

/// \brief Recorder callback is called to notify non track
/// and non session specific event notifications
///
/// Only error event types are expected as of now
struct RecorderCb {
  EventCb event_cb;
};

/// \brief RecorderErrorData is used to determine the type of recorer errors.
struct RecorderErrorData {
  uint32_t        camera_id;
  int32_t         error_code;
};

/// \brief Session cb is used to return state changes i.e. to indicate
/// start, stop, pause state transition completions
struct SessionCb {
  EventCb event_cb;
};

/// \brief MetaParamType flag is used to determine type of meta data set in
/// MetaData structure.
enum class MetaParamType {
  kNone               = (1 << 0),
  kCamBufMetaData     = (1 << 1),
  kVideoFrameType     = (1 << 2),
  kCamMetaFrameNumber = (1 << 3)
};

/// \brief MultiCameraConfigType is used to determine the type and purpose
/// of created MultiCamera.
enum class MultiCameraConfigType {
  k360Stitch,
  kSideBySide,
};

/// \brief This struct is used to report different types of meta data associated
/// with BufferDescriptor.
struct MetaData {
  uint32_t meta_flag;
  CameraBufferMetaData cam_buffer_meta_data;
  uint32_t video_frame_type_info;
  uint32_t cam_meta_frame_number;
};

/// \brief Plugin related information exposed to the client
///
/// \param name: plugin name
/// \param version: version of the underlying library
/// \param togglable: indicating whether runtime enable/disable is supported
struct PluginInfo {
  std::string name;
  float       version;
  bool        togglable;

  PluginInfo()
    : name(), version(0.0), togglable(false) {}

  PluginInfo(const std::string name,
             const float version,
             const bool togglable)
    : name(name),
      version(version),
      togglable(togglable) {}

  PluginInfo(const void *blob, const size_t size) { FromBlob(blob, size); }

  std::string ToString(uint32_t indent = 0) const {
    std::stringstream indentation;
    for (uint32_t i = 0; i < indent; i++) indentation << '\t';
    indent++;

    std::stringstream stream;
    stream << indentation.str()
           << "\"name\" : " << name << '\n';
    stream << indentation.str()
           << "\"version\" : " << version << '\n';
    stream << indentation.str()
           << "\"togglable\" : " << togglable << '\n';
    return stream.str();
  }

  std::shared_ptr<void> ToBlob() const {
    std::shared_ptr<void> blob(new uint8_t[Size()]);

    uintptr_t dest = reinterpret_cast<uintptr_t>(blob.get());
    memcpy(reinterpret_cast<void *>(dest), name.data(), name.size());

    dest += name.size();
    memcpy(reinterpret_cast<void *>(dest), &version, sizeof(version));

    dest += sizeof(version);
    memcpy(reinterpret_cast<void *>(dest), &togglable, sizeof(togglable));

    return blob;
  }

  void FromBlob(const void *blob, const size_t size) {
    size_t name_size = size - sizeof(version) - sizeof(togglable);

    uintptr_t src = reinterpret_cast<uintptr_t>(blob);
    name.assign(reinterpret_cast<const char *>(src), name_size);

    src += name_size;
    memcpy(&version, reinterpret_cast<void *>(src), sizeof(version));

    src += sizeof(version);
    memcpy(&togglable, reinterpret_cast<void *>(src), sizeof(togglable));
  }

  size_t Size() const {
    return (name.size() + sizeof(version) + sizeof(togglable));
  }
};

typedef std::vector<PluginInfo> SupportedPlugins;

/// \brief Both data and event callbacks should be set by the client.
/// event_cb is called to notify track specific errors and data_cb
/// to notify availability of output data from track to clients
///
/// TrackMetaParam in data cb is an optional parameter. This parameter is
/// expected
/// to be used in case multiple frames are passed in the same buffer and
/// in that case meta_param can describe the respective frame offsets
/// and timestamps in the buffer
/// When the data cb is called by recorder, the buffer ownership is transfered
/// to
/// client. To return the buffer back to recoder, clients should call
/// ReturnTrackBuffer
/// API. However, clients needs to ensure that buffers returned within the frame
/// rate
/// of track - else recording pipeline will stall.
/// Track event_cb returns async error events and data_cb returns periodic
/// data
/// Note that both callback implementations need to be re-entrant.
struct TrackCb {
  std::function<void(uint32_t track_id, ::std::vector<BufferDescriptor> buffers,
                     ::std::vector<MetaData> meta_data)> data_cb;
  std::function<void(uint32_t track_id, EventType event_type, void *event_data,
                     size_t event_data_size)> event_cb;
};

/// @brief Createtime parameters for audio track
///
/// Audio output device is used for routing audio to output
/// to external devices say through HDMI. In all other usecases
/// out_device will be set to AUDIO_DEVICE_NONE
struct AudioTrackCreateParam {
  uint32_t                in_devices_num;
  DeviceId                in_devices[MAX_AUDIO_INPUT_DEVICES];
  uint32_t                sample_rate;
  uint32_t                channels;
  uint32_t                bit_depth;
  AudioFormat             format;
  AudioCodecParams        codec_params;
  DeviceId                out_device;
  uint32_t                flags;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "in_devices[";
    for (uint32_t i = 0; i < in_devices_num; i++)
      stream << in_devices[i] << ", ";
    stream << "SIZE[" << in_devices_num << "]], ";
    stream << "sample_rate[" << sample_rate << "] ";
    stream << "channels[" << channels << "] ";
    stream << "bit_depth[" << bit_depth << "] ";
    stream << "format["
           << static_cast<::std::underlying_type<AudioFormat>::type>(format)
           << "] ";
    stream << "codec_params[" << codec_params.ToString(format) << "] ";
    stream << "out_device[" << out_device << "] ";
    stream << "flags[" << flags << "]";
    return stream.str();
  }
};

/// \brief create time parameters for a video track
/// For 360 degree capture, camera_id vector should contain the id of
/// multiple cameras involved in 360 capture
/// \param low_power_mode: true indicates that track to be output by VFE
///        (Video Front End) camera block without any post-processing.
///        Currently atmost one track can be a low_power_mode track.
/// \TODO: define VideoOutDevice
struct VideoTrackCreateParam {
  uint32_t camera_id;
  uint32_t width;
  uint32_t height;
  float frame_rate;
  VideoFormat format_type;
  VideoCodecParams codec_param;
  bool low_power_mode;
  bool do_vqzip;
  VQZipInfo vqzip_params;

  VideoTrackCreateParam(uint32_t cam_id = 0,
                        VideoFormat fmt = VideoFormat::kYUV, uint32_t w = 3840,
                        uint32_t h = 1920, float frm_rate = 30) {
    camera_id = cam_id;
    width = w;
    height = h;
    frame_rate = frm_rate;
    format_type = fmt;
    switch (format_type) {
      case VideoFormat::kAVC:
        setAVCDefaultVideoParam();
        break;
      case VideoFormat::kHEVC:
        setHEVCDefaultVideoParam();
        break;
      default: {
        // Nothing to do for other formats
      }
    }

    // Setting LPM,VQZipInfo parameters
    low_power_mode = false;
    do_vqzip = false;
    memset(&vqzip_params, 0x00, sizeof(vqzip_params));
  }

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "camera_id[" << camera_id << "] ";
    stream << "width[" << width << "] ";
    stream << "height[" << height << "] ";
    stream << "frame_rate[" << frame_rate << "] ";
    stream << "format_type["
           << static_cast<::std::underlying_type<VideoFormat>::type>(
                  format_type)
           << "] ";
    stream << "codec_params[" << codec_param.ToString(format_type) << "] ";
    stream << "do_vqzip[" << ::std::boolalpha << do_vqzip << ::std::noboolalpha
           << "] ";
    stream << "vqzip_params[" << vqzip_params.ToString() << "]";
    return stream.str();
  }

  void setAVCDefaultVideoParam() {
    // Setting default Parameters for AVC
    codec_param.avc.idr_interval = 1;
    codec_param.avc.bitrate = 6000000;
    codec_param.avc.profile = AVCProfileType::kHigh;
    codec_param.avc.level = AVCLevelType::kLevel5_1;
    codec_param.avc.ratecontrol_type = VideoRateControlType::kMaxBitrate;
    codec_param.avc.qp_params.enable_init_qp = true;
    codec_param.avc.qp_params.init_qp.init_IQP = 27;
    codec_param.avc.qp_params.init_qp.init_PQP = 28;
    codec_param.avc.qp_params.init_qp.init_BQP = 28;
    codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
    codec_param.avc.qp_params.enable_qp_range = true;
    codec_param.avc.qp_params.qp_range.min_QP = 10;
    codec_param.avc.qp_params.qp_range.max_QP = 51;
    codec_param.avc.qp_params.enable_qp_IBP_range = true;
    codec_param.avc.qp_params.qp_IBP_range.min_IQP = 10;
    codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
    codec_param.avc.qp_params.qp_IBP_range.min_PQP = 10;
    codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
    codec_param.avc.qp_params.qp_IBP_range.min_BQP = 10;
    codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
    codec_param.avc.ltr_count = 0;
    codec_param.avc.insert_aud_delimiter = true;
    codec_param.avc.hier_layer = 0;
    codec_param.avc.prepend_sps_pps_to_idr = false;
    codec_param.avc.sar_enabled = false;
    codec_param.avc.sar_width = 0;
    codec_param.avc.sar_height = 0;
  }

  void setHEVCDefaultVideoParam() {
    // Setting default Parameters for HEVC
    codec_param.hevc.idr_interval = 1;
    codec_param.hevc.bitrate = 6000000;
    codec_param.hevc.profile = HEVCProfileType::kMain;
    codec_param.hevc.level = HEVCLevelType::kLevel5_1;
    codec_param.hevc.ratecontrol_type = VideoRateControlType::kMaxBitrate;
    codec_param.hevc.qp_params.enable_init_qp = true;
    codec_param.hevc.qp_params.init_qp.init_IQP = 27;
    codec_param.hevc.qp_params.init_qp.init_PQP = 28;
    codec_param.hevc.qp_params.init_qp.init_BQP = 28;
    codec_param.hevc.qp_params.init_qp.init_QP_mode = 0x7;
    codec_param.hevc.qp_params.enable_qp_range = true;
    codec_param.hevc.qp_params.qp_range.min_QP = 10;
    codec_param.hevc.qp_params.qp_range.max_QP = 51;
    codec_param.hevc.qp_params.enable_qp_IBP_range = true;
    codec_param.hevc.qp_params.qp_IBP_range.min_IQP = 10;
    codec_param.hevc.qp_params.qp_IBP_range.max_IQP = 51;
    codec_param.hevc.qp_params.qp_IBP_range.min_PQP = 10;
    codec_param.hevc.qp_params.qp_IBP_range.max_PQP = 51;
    codec_param.hevc.qp_params.qp_IBP_range.min_BQP = 10;
    codec_param.hevc.qp_params.qp_IBP_range.max_BQP = 51;
    codec_param.hevc.ltr_count = 0;
    codec_param.hevc.hier_layer = 0;
    codec_param.hevc.prepend_sps_pps_to_idr = false;
    codec_param.hevc.sar_enabled = false;
    codec_param.hevc.sar_width = 0;
    codec_param.hevc.sar_height = 0;
  }
};

/// \brief Result callback passed to StartCamera API
///
/// Optional result callback which will get triggered
/// by service once there is at least one started session
/// which includes a video track.
typedef std::function<void(uint32_t camera_id,
                           const android::CameraMetadata &res)> CameraResultCb;

/// \brief Parameters passed to StartCamera API
///
/// When the zsl mode is set to true during StartCamera, recorder
/// would start capturing images of resolution max_snapshot_width
/// and max_snapshot_height from camera at the frame_rate specified.
/// In non-zsl mode, snapshot resolution and frame rate parameter is
/// ignored.
/// flags provide a mechanism to provide a custom initialization
/// parameter to camera
struct CameraStartParam {
  bool     zsl_mode;
  uint32_t zsl_queue_depth;
  uint32_t zsl_width;
  uint32_t zsl_height;
  uint32_t frame_rate;
  uint32_t flags;

  CameraStartParam()
      : zsl_mode(false),
        zsl_queue_depth(10),
        zsl_width(3840),
        zsl_height(2160),
        frame_rate(30),
        flags(0) {}

  CameraStartParam(bool zsl_mode, uint32_t zsl_queue_depth, uint32_t zsl_width,
                   uint32_t zsl_height, uint32_t frame_rate, uint32_t flags)
      : zsl_mode(zsl_mode),
        zsl_queue_depth(zsl_queue_depth),
        zsl_width(zsl_width),
        zsl_height(zsl_height),
        frame_rate(frame_rate),
        flags(flags) {}

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "zsl_mode[" << ::std::boolalpha << zsl_mode << ::std::noboolalpha
           << "]";
    stream << "zsl_queue_depth[" << zsl_queue_depth << "] ";
    stream << "zsl_width[" << zsl_width << "] ";
    stream << "zsl_height[" << zsl_height << "] ";
    stream << "frame_rate[" << frame_rate << "] ";
    stream << "flags[" << flags << "]";
    return stream.str();
  };

  void setSensorVendorMode(int32_t sensor_vendor_mode) {
    flags &= ~(SENSOR_VENDOR_MODE_MASK);
    flags |= sensor_vendor_mode << SENSOR_VENDOR_MODE_OFFSET;
  };

  int32_t getSensorVendorMode() const {
    return flags >> SENSOR_VENDOR_MODE_OFFSET;
  }
};

/// \brief For thumbnail images only kJPEG is supported
/// For YUV and Bayer formats, quality is ignored
struct ImageParam {
  uint32_t    width;
  uint32_t    height;
  uint32_t    image_quality;
  ImageFormat image_format;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "width[" << width << "]";
    stream << "height[" << height << "] ";
    stream << "image_quality[" << image_quality << "] ";
    stream << "image_format["
           << static_cast<::std::underlying_type<ImageFormat>::type>
                         (image_format)
           << "]";
    return stream.str();
  }
};

/// \brief Advance configuration for image capture
///
/// \param sensor_frame_skip_interval: When multiple images needs to be captured
///       clients can set the sample_rate of capture through this parameter.
///       If this value is set to 1, every alternate image is captured,
///       if 2, every 3rd image is captured and so on_event_id
/// \param with_exif: Applies only when image codec is set to JPEG. If set to
///       true, EXIF along with with thumbnails are embedded with the image.
///       Else thumbnails and camera meta information is send separately through
///       metadata
/// \param with_camera_meta: Applies only with with_exif is set to false. In
///       this case, camera metadata is send separately
/// \param with_raw: Enables clients to take a RAW image along with JPEG. This
///       can be set to true only when ImageFormat is NOT RAW
///
/// \param raw_image_type: Could be either of RDI RAW or IDEAL Raw
/// \param thumbnail_image_param: Thumbnail image characteristics. This is
/// array since a single image can contain more than one thumbnail
struct ImageCaptureConfig {
  uint32_t sensor_frame_skip_interval;
  bool with_exif;
  bool with_camera_meta;
  bool with_raw;
  ImageFormat raw_image_format;
  uint32_t num_thumbnail_image_param;
  ImageParam thumbnail_image_param[MAX_THUMBNAIL_IMAGE_PARAM];

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "sensor_frame_skip_interval[" << sensor_frame_skip_interval
           << "] ";
    stream << "with_exif[" << ::std::boolalpha << with_exif
           << ::std::noboolalpha << "] ";
    stream << "with_camera_meta[" << ::std::boolalpha << with_camera_meta
           << ::std::noboolalpha << "] ";
    stream << "with_raw[" << ::std::boolalpha << with_raw << ::std::noboolalpha
           << "] ";
    stream << "raw_image_format["
           << static_cast<::std::underlying_type<ImageFormat>::type>
                         (raw_image_format)
           << "] ";
    stream << "thumbnail_image_param[";
    for (uint32_t i = 0; i < num_thumbnail_image_param; i++)
      stream << "thumbnail_image_param[" << thumbnail_image_param[i].ToString() << "] ";
    stream << "SIZE[" << num_thumbnail_image_param << "]], ";
    return stream.str();
  }
};

typedef std::function<void(uint32_t camera_id, uint32_t image_sequence_count,
                           BufferDescriptor buffer, MetaData meta_data)>
    ImageCaptureCb;

};
};  // namespace qmmf::recorder
