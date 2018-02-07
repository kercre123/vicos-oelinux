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

#define LOG_TAG "HTTPInterface"

#include <qmmf-sdk/qmmf_display.h>
#include <qmmf-sdk/qmmf_display_params.h>
#include <qmmf-sdk/qmmf_recorder.h>
#include <qmmf_rtsp_server_interface.h>
#include <qmmf_mux_interface.h>
#include <qmmf_vam_interface.h>
#include <utils/Log.h>
#include <utils/Mutex.h>
#include <utils/KeyedVector.h>
#include <utils/Errors.h>
#include <media/msm_media_info.h>
#ifdef QMMF_LE_BUILD
#include <fpv_rave/fpv_config.hpp>
#include <fpv_rave/fpv_queue.hpp>
#include <fpv_rave/fpv_dbg.hpp>
#include <fpv_rave/fpv_ra.hpp>
#include <fpv_rave/fpv_utils.hpp>
#include <fpv_rave/fpv_rave.hpp>
#endif
#include "qmmf_camera_configuration.h"
#include "qmmf_http_interface.h"
#include <fstream>

#define STORE_SNAPSHOT 1
#define TABLE_SIZE(table) sizeof(table)/sizeof(table[0])

namespace qmmf {

namespace httpinterface {

using namespace recorder;
using namespace android;
using namespace muxinterface;
using namespace vaminterface;
using namespace std;

using ::qmmf::display::DisplayEventType;
using ::qmmf::display::DisplayType;
using ::qmmf::display::Display;
using ::qmmf::display::DisplayCb;
using ::qmmf::display::SurfaceBuffer;
using ::qmmf::display::SurfaceParam;
using ::qmmf::display::SurfaceConfig;
using ::qmmf::display::SurfaceBlending;
using ::qmmf::display::SurfaceFormat;

typedef struct RTSPContext_t {
    RtspServerInterface *rtsp_server;
    AVQueue *rtsp_video_queue;
    AVQueue *rtsp_audio_queue;
    AVQueue *rtsp_meta_queue;
    uint32_t meta_track_id;
    tEsFmt audio_codec_id;
    FS_IDX_et audio_idx;
    CH_CFG_et audio_channels;
    PROFILE_et audio_profile;
    uint32_t video_track_id;
    tEsFmt video_codec_id;
    size_t frame_rate;
    bool is_mp2ts;
    uint16_t rtsp_port;
    char *rtsp_url;
} RTSPContext;

typedef struct AVCProfileMap_t {
  AVCProfileType profile;
  uint8_t value;
} AVCProfileMap;

typedef struct AVCLevelMap_t {
  AVCLevelType level;
  uint8_t value;
} AVCLevelMap;

typedef struct HEVCProfileMap_t {
  HEVCProfileType profile;
  uint8_t value;
} HEVCProfileMap;

typedef struct HEVCLevelMap_t {
  HEVCLevelType level;
  uint8_t value;
} HEVCLevelMap;

typedef struct AudioSamplingES_t {
    FS_IDX_et fs_Idx;
    size_t sampling_rate;
} AudioSamplingES;

typedef enum RTSPInput_t {
  RTSP_VIDEO = 0,
  RTSP_AUDIO,
  RTSP_META,
} RTSPInput;

typedef struct OverlayTypeEntry_t {
  OverlayType qmmf_entry;
  qmmf_overlay_type entry;
} OverlayTypeEntry;

typedef struct OverlayPositionEntry_t {
  OverlayLocationType qmmf_entry;
  qmmf_overlay_position entry;
} OverlayPositionEntry;

typedef struct OverlayDateEntry_t {
  OverlayDateFormatType qmmf_entry;
  qmmf_overlay_date entry;
} OverlayDateEntry;

typedef struct OverlayTimeEntry_t {
  OverlayTimeFormatType qmmf_entry;
  qmmf_overlay_time entry;
} OverlayTimeEntry;

typedef struct OverlayStaticImageEntry_t {
  OverlayImageType qmmf_entry;
  qmmf_overlay_image_type entry;
} OverlayStaticImageEntry;

static const int kSnapshotTimeout = 4; // 4 sec.

class HTTPInterface : public VAMCb {
 public:

  //Module functionality
  static int32_t Open(struct qmmf_module_t *module);
  static int32_t Close(struct qmmf_module_t *module);

  //Interface functionality
  static int32_t ConnectOp(struct qmmf_module_t *module);
  static int32_t DisconnectOp(struct qmmf_module_t *module);
  static int32_t StartCameraOp(struct qmmf_module_t *module, uint32_t camera_id,
                               qmmf_camera_start_param start_parms);
  static int32_t StopCameraOp(struct qmmf_module_t *module,
                              uint32_t camera_id);
  static int32_t CreateSessionOp(struct qmmf_module_t *module,
                                 uint32_t *session_id);
  static int32_t DeleteSessionOp(struct qmmf_module_t *module,
                                 uint32_t session_id);
  static int32_t CreateVideoTrackOp(struct qmmf_module_t *module,
                                    qmmf_video_track_param track_parm);
  static int32_t DeleteVideoTrackOp(struct qmmf_module_t *module,
                               uint32_t session_id, uint32_t track_id);
  static int32_t SetVideoTrackParamOp(struct qmmf_module_t *module,
                                      uint32_t session_id, uint32_t track_id,
                                      qmmf_track_param param);
  static int32_t CreateAudioTrackOp(struct qmmf_module_t *module,
                                    qmmf_audio_track_param track_parm);
  static int32_t DeleteAudioTrackOp(struct qmmf_module_t *module,
                                    uint32_t session_id, uint32_t track_id);
  static int32_t SetAudioTrackParamOp(struct qmmf_module_t *module,
                                      uint32_t session_id, uint32_t track_id,
                                      qmmf_track_param param);
  static int32_t StartSessionOp(struct qmmf_module_t *module,
                                uint32_t session_id);
  static int32_t StopSessionOp(struct qmmf_module_t *module,
                               uint32_t session_id, uint32_t flush);
  static qmmf_image_result CaptureImageOp(struct qmmf_module_t *module,
                                          qmmf_image_param image_param);
  static qmmf_status * GetStatusOp(struct qmmf_module_t *module);
  static int32_t VAMConfigOp(struct qmmf_module_t *module,
                             const char *json_config);
  static int32_t VAMRemoveConfigOp(struct qmmf_module_t *module,
                                   const char *json_config);
  static int32_t VAMEnrollOp(struct qmmf_module_t *module,
                             qmmf_vam_enrollment_info enroll_info);
  static int32_t VAMDisenrollOp(struct qmmf_module_t *module,
                                uint32_t evnet_type,
                                const char *id);
  static int32_t SetCameraParamsOp(struct qmmf_module_t *mode,
                                   qmmf_camera_parameters params);
  static int32_t CreateOverlayOp(struct qmmf_module_t *module,
                                 uint32_t track_id, uint32_t *overlay_id,
                                 struct qmmf_overlay_param_t *params);
  static int32_t DeleteOverlayOp(struct qmmf_module_t *module,
                                 uint32_t track_id, uint32_t overlay_id);
  static int32_t SetOverlayOp(struct qmmf_module_t *module, uint32_t track_id,
                              uint32_t overlay_id);
  static int32_t RemoveOverlayOp(struct qmmf_module_t *module,
                                 uint32_t track_id, uint32_t overlay_id);
  static int32_t GetOverlayOp(struct qmmf_module_t *module, uint32_t track_id,
                              uint32_t overlay_id,
                              struct qmmf_overlay_param_t *params);
  static int32_t UpdateOverlayOp(struct qmmf_module_t *module,
                                 uint32_t track_id, uint32_t overlay_id,
                                 struct qmmf_overlay_param_t *params);
#ifdef QMMF_LE_BUILD
  static int32_t RaveTrackResolutionChangeCbOp(void *handle, uint32_t width,
                                               uint32_t height,
                                               uint32_t framerate,
                                               uint32_t bitrate);
  static int32_t RaveTrackQualityChangeCbOp(void *handle, uint32_t framerate,
                                            uint32_t bitrate);
#endif

  static int32_t CreateMultiCameraOp(struct qmmf_module_t *module,
                                     const uint32_t *camera_ids,
                                     uint32_t num_camera,
                                     uint32_t *virtual_camera_id);

  static int32_t ConfigureMultiCameraOp(struct qmmf_module_t *module,
                                        qmmf_multi_camera_param_t *params);

  static qmmf_db_result DatabaseCommandOp(struct qmmf_module_t *module,
                                          struct qmmf_db_param_t *params);

 private:
  HTTPInterface();
  ~HTTPInterface();
  int32_t Connect();
  int32_t Disconnect();
  int32_t StartCamera(uint32_t camera_id, qmmf_camera_start_param start_parms);
  int32_t StopCamera(uint32_t camera_id);
  int32_t CreateSession(uint32_t *session_id);
  int32_t DeleteSession(uint32_t session_id);
  int32_t CreateVideoTrack(qmmf_video_track_param track_parms);
  int32_t DeleteVideoTrack(uint32_t session_id, uint32_t track_id);
  int32_t SetVideoTrackParam(uint32_t session_id, uint32_t track_id,
                             qmmf_track_param param);
  int32_t CreateAudioTrack(qmmf_audio_track_param track_parms);
  int32_t DeleteAudioTrack(uint32_t session_id, uint32_t track_id);
  int32_t SetAudioTrackParam(uint32_t session_id, uint32_t track_id,
                             qmmf_track_param param);
  int32_t StartSession(uint32_t session_id);
  int32_t StopSession(uint32_t session_id, uint32_t flush);
  qmmf_image_result CaptureImage(qmmf_image_param image_param);
  qmmf_status * GetStatus();
  int32_t SetCameraParams(qmmf_camera_parameters params);
  int32_t CreateOverlay(uint32_t track_id, uint32_t *overlay_id,
                        struct qmmf_overlay_param_t *params);
  int32_t DeleteOverlay(uint32_t track_id, uint32_t overlay_id);
  int32_t SetOverlay(uint32_t track_id, uint32_t overlay_id);
  int32_t RemoveOverlay(uint32_t track_id, uint32_t overlay_id);
  int32_t GetOverlay(uint32_t track_id, uint32_t overlay_id,
                     struct qmmf_overlay_param_t *params);
  int32_t UpdateOverlay(uint32_t track_id, uint32_t overlay_id,
                        struct qmmf_overlay_param_t *params);
  int32_t CreateMultiCamera(const uint32_t *camera_ids, uint32_t num_camera,
                            uint32_t *virtual_camera_id);
  int32_t ConfigureMultiCamera(qmmf_multi_camera_param_t *params);
  int32_t convertOvParams2QMMF(qmmf_overlay_param &ovParams,
                                    OverlayParam &params);
  int32_t convertQMMF2OvParams(qmmf_overlay_param &ovParams,
                               OverlayParam &params);

  int32_t InitRTSPServerLocked(uint32_t session_id);
  int32_t QueueRTSPBuffersLocked(uint32_t session_id,
                                 std::vector<BufferDescriptor> &buffers,
                                 RTSPInput input);
  int32_t CloseRTSPServerLocked(uint32_t session_id);
  int32_t AddRTSPVideoLocked(uint32_t session_id,
                             uint32_t track_id,
                             const VideoTrackCreateParam &video,
                             qmmf_video_track_output output);
  int32_t UpdateTrackRTSPURLLocked(uint32_t track_id, const char *url);
  int32_t RemoveRTSPVideoLocked(uint32_t session_id);
  FS_IDX_et FindAudioSampleIndex(size_t audio_rate);
  int32_t AddRTSPAudioLocked(uint32_t session_id,
                             const qmmf_audio_track_param &audio);
  int32_t RemoveRTSPAudioLocked(uint32_t session_id);

  int32_t InitMuxerLocked(uint32_t session_id,
                          qmmf_muxer_init &init_params);
  int32_t AddAudMuxParmsLocked(const qmmf_audio_track_param &audio);
  int32_t RemoveAudMuxParmsLocked(uint32_t session_id);
  int32_t AddVidDisplayLocked(DisplayType display_type, qmmf_video_track_param track_parms);
  int32_t RemoveVidDisplayLocked(DisplayType display_type);
  void DisplayCallbackHandler(DisplayEventType event_type,
    void *event_data, size_t event_data_size);
  void DisplayVSyncHandler(int64_t time_stamp);
  void PushFrameToDisplay(BufferDescriptor& buffer,
    CameraBufferMetaData& meta_data);
  int32_t getAVCProfileLevel(const VideoTrackCreateParam &video,
                             uint8_t &level, uint8_t &profile);
  int32_t getHEVCProfileLevel(const VideoTrackCreateParam &video,
                              uint8_t &level, uint8_t &profile);
  int32_t AddVidMuxParmsLocked(uint32_t session_id,
                               uint32_t track_id,
                               const VideoTrackCreateParam &video,
                               qmmf_video_track_output output);
  int32_t RemoveVidMuxParmsLocked(uint32_t session_id);
  int32_t QueueMuxBuffersLocked(uint32_t track_id, uint32_t session_id,
                                std::vector<BufferDescriptor> &buffers,
                                std::vector<MetaData> &meta_data);
  int32_t ReturnTrackBuffer(uint32_t track_id, uint32_t session_id,
                            BufferDescriptor &buffer) override;
  int32_t StartVAMVideoLogLocked(const qmmf_video_track_param &param);
  int32_t SendVAMMeta(uint32_t session_id, uint32_t track_id,
                      const char *metaString, size_t size, int64_t pts) override;
  param_type ValidateParamType(CodecParamType param);
  CodecParamType SetCodecParamType(param_type type);
  void RecorderEventCb(EventType event_type, void *event_data,
                       size_t event_data_size);
  void SessionEventCb(EventType event_type, void *event_data,
                      size_t event_data_size);
  void AudioTrackCb(uint32_t track_id, std::vector<BufferDescriptor> buffers,
                    std::vector<MetaData> meta_data);
  void VideoTrackCb(uint32_t track_id, std::vector<BufferDescriptor> buffers,
                    std::vector<MetaData> meta_data);
  void SnapshotCb(uint32_t camera_id, uint32_t image_sequence_count,
                  BufferDescriptor buffer, MetaData meta_data);
  void CameraCb(uint32_t camera_id,
                const CameraMetadata &result);
#ifdef QMMF_LE_BUILD
  int32_t RaveTrackResolutionChangeCb(uint32_t width, uint32_t height,
                                      uint32_t framerate, uint32_t bitrate);
  int32_t RaveTrackQualityChangeCb(uint32_t framerate, uint32_t bitrate);
  int32_t RaveInit();
  int32_t RaveStart();
  int32_t RaveExit();
#endif

  /**Not allowed */
  HTTPInterface(const HTTPInterface &);
  HTTPInterface &operator=(const HTTPInterface &);

  Mutex lock_;
  KeyedVector<uint32_t, uint32_t> session_map_;
  Recorder recorder_;
  KeyedVector<uint32_t, qmmf_camera_start_param> cameras_;
  KeyedVector<uint32_t, CameraConfiguration*> camera_configs_;
  KeyedVector<uint32_t, RTSPContext> rtsp_servers_;
  KeyedVector<uint32_t, qmmf_video_track_status> video_tracks_;
  KeyedVector<uint32_t, qmmf_audio_track_param> audio_tracks_;
  KeyedVector<uint32_t, MuxInterface *> muxers_;
  KeyedVector<uint32_t, qmmf_muxer_init> muxer_params_;
  pthread_mutex_t snapshot_lock_;
  pthread_mutex_t aec_converge_lock_;
  qmmf_image_result snapshot_result_;
  bool snapshot_completed_;
  pthread_cond_t snapshot_cond_;
  pthread_cond_t aec_converge_cond_;
#ifdef QMMF_LE_BUILD
  qmmf_video_track_param rave_track_param_store_;
  bool rave_track_initialized_ = false;
  bool rave_track_started_ = false;
  bool rave_stopsession_ = false;
#endif

  VAMInterface *vam_interface_;
  qmmf_video_track_param vam_video_log_params_;

  Display* display_;
  uint32_t surface_id_;
  SurfaceParam surface_param_;
  SurfaceBuffer surface_buffer_;
  bool display_started_;
  uint32_t dsi_counter_ = 0;
  bool aec_converged_;
  static const char kMuxedFileName[];
  static const AVCProfileMap kAVCMuxerProfiles[];
  static const AVCLevelMap kAVCMuxerLevels[];
  static const HEVCProfileMap kHEVCMuxerProfiles[];
  static const HEVCLevelMap kHEVCMuxerLevels[];
  static const AudioSamplingES kAudioSamplingIndices[];
  static const OverlayTypeEntry kOverlayTypeTable[];
  static const OverlayPositionEntry kOverlayPositionTable[];
  static const OverlayDateEntry kOverlayDateTable[];
  static const OverlayTimeEntry kOverlayTimeTable[];
  static const OverlayStaticImageEntry kOverlayStaticImageTable[];
  static const char videoIdFileName[];
  static const char imageIdFileName[];
  static const uint32_t kRepeat = 3;
  bool camera_error_;
};

const char HTTPInterface::kMuxedFileName[] = "/data/misc/qmmf/qmmf_video_%d.%s";
const char HTTPInterface::videoIdFileName[] = "/data/misc/qmmf/video_id";
const char HTTPInterface::imageIdFileName[] = "/data/misc/qmmf/image_id";

const AVCProfileMap HTTPInterface::kAVCMuxerProfiles[] = {
    {AVCProfileType::kBaseline, 66},
    {AVCProfileType::kMain, 77},
    {AVCProfileType::kHigh, 100},
};

const AVCLevelMap HTTPInterface::kAVCMuxerLevels[] = {
    {AVCLevelType::kLevel3, 30},
    {AVCLevelType::kLevel4, 40},
    {AVCLevelType::kLevel5, 50},
    {AVCLevelType::kLevel5_1, 51},
    {AVCLevelType::kLevel5_2, 52},
};

const HEVCProfileMap HTTPInterface::kHEVCMuxerProfiles[] = {
    {HEVCProfileType::kMain, 77},
};

const HEVCLevelMap HTTPInterface::kHEVCMuxerLevels[] = {
    {HEVCLevelType::kLevel3, 30},
    {HEVCLevelType::kLevel4, 40},
    {HEVCLevelType::kLevel5, 50},
    {HEVCLevelType::kLevel5_1, 51},
    {HEVCLevelType::kLevel5_2, 52},
};

const AudioSamplingES HTTPInterface::kAudioSamplingIndices [] = {
        {FS_IDX_96, 96000},
        {FS_IDX_88, 88200},
        {FS_IDX_64, 64000},
        {FS_IDX_48, 48000},
        {FS_IDX_44, 44100},
        {FS_IDX_32, 32000},
        {FS_IDX_24, 24000},
        {FS_IDX_22, 22050},
        {FS_IDX_16, 16000},
        {FS_IDX_12, 12000},
        {FS_IDX_11, 11025},
        {FS_IDX_08, 8000},
        {FS_IDX_07, 7350},
        {FS_IDX_MAX, 0}
};

const OverlayTypeEntry HTTPInterface::kOverlayTypeTable [] = {
        {OverlayType::kDateType, DATE_TIME},
        {OverlayType::kUserText, USERTEXT},
        {OverlayType::kStaticImage, STATICIMAGE},
        {OverlayType::kBoundingBox, BOUNDINGBOX},
        {OverlayType::kPrivacyMask, PRIVACYMASK},
};

const OverlayPositionEntry HTTPInterface::kOverlayPositionTable [] = {
        {OverlayLocationType::kTopLeft, TOPLEFT},
        {OverlayLocationType::kTopRight, TOPRIGHT},
        {OverlayLocationType::kCenter, CENTER},
        {OverlayLocationType::kBottomLeft, BOTTOMLEFT},
        {OverlayLocationType::kBottomRight, BOTTOMRIGHT},
        {OverlayLocationType::kRandom, RANDOM},
        {OverlayLocationType::kNone, NONE},
};

const OverlayDateEntry HTTPInterface::kOverlayDateTable [] = {
        {OverlayDateFormatType::kMMDDYYYY, MMDDYYYY},
        {OverlayDateFormatType::kYYYYMMDD, YYYYMMDD},
};

const OverlayTimeEntry HTTPInterface::kOverlayTimeTable [] = {
        {OverlayTimeFormatType::kHHMMSS_24HR, HHMMSS_24HR},
        {OverlayTimeFormatType::kHHMMSS_AMPM, HHMMSS_AMPM},
        {OverlayTimeFormatType::kHHMM_24HR, HHMM_24HR},
        {OverlayTimeFormatType::kHHMM_AMPM, HHMM_AMPM},
};

const OverlayStaticImageEntry HTTPInterface::kOverlayStaticImageTable [] = {
        {OverlayImageType::kFilePath, FILEPATH},
        {OverlayImageType::kBlobType, BLOBTYPE},
};

extern "C" {
qmmf_http_interface QMMF_MODULE = {
    HTTPInterface::Open,
    HTTPInterface::Close,
};
}

HTTPInterface::HTTPInterface() :
    snapshot_completed_(false),
    aec_converged_(false),
    camera_error_(false) {
  pthread_condattr_t snapshot_cond_attr;
  memset(&snapshot_result_, 0, sizeof(snapshot_result_));
  pthread_mutex_init(&snapshot_lock_, nullptr);
  pthread_condattr_init(&snapshot_cond_attr);
  pthread_condattr_setclock(&snapshot_cond_attr,CLOCK_MONOTONIC);
  pthread_cond_init(&snapshot_cond_, &snapshot_cond_attr);
  pthread_condattr_destroy(&snapshot_cond_attr);
  pthread_mutex_init(&aec_converge_lock_, nullptr);
  pthread_cond_init(&aec_converge_cond_, nullptr);
  memset(&vam_video_log_params_, 0, sizeof(vam_video_log_params_));
  vam_interface_ = VAMInterfaceFactory::NewInstance(this);
  assert(nullptr != vam_interface_);
}

HTTPInterface::~HTTPInterface() {
  pthread_mutex_destroy(&snapshot_lock_);
  pthread_cond_destroy(&snapshot_cond_);
  pthread_mutex_destroy(&aec_converge_lock_);
  pthread_cond_destroy(&aec_converge_cond_);
  if (!muxers_.isEmpty()) {
    for (size_t i = 0; i < muxers_.size(); i++) {
      MuxInterface *mux = muxers_.valueAt(i);
      delete mux;
    }
    muxers_.clear();
  }

  if (!rtsp_servers_.isEmpty()) {
    size_t rtsp_server_count = rtsp_servers_.size();
    for (size_t i = 0; i< rtsp_server_count; i++) {
      uint32_t session_id = rtsp_servers_.keyAt(i);
      CloseRTSPServerLocked(session_id);
    }
    rtsp_servers_.clear();
  }

  if (!camera_configs_.isEmpty()) {
    for (size_t i = 0; i < camera_configs_.size(); i++) {
      CameraConfiguration *config = camera_configs_.valueAt(i);
      delete config;
    }
    camera_configs_.clear();
  }

  delete vam_interface_;
}

int32_t HTTPInterface::Open(struct qmmf_module_t *module) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  memset(module, 0, sizeof(struct qmmf_module_t));
  module->priv = new HTTPInterface();
  if (nullptr == module->priv) {
    ALOGE("%s: No memory for module!", __func__);
    return NO_MEMORY;
  }

  module->connect = HTTPInterface::ConnectOp;
  module->disconnect = HTTPInterface::DisconnectOp;
  module->start_camera = HTTPInterface::StartCameraOp;
  module->stop_camera = HTTPInterface::StopCameraOp;
  module->create_session = HTTPInterface::CreateSessionOp;
  module->delete_session = HTTPInterface::DeleteSessionOp;
  module->create_video_track = HTTPInterface::CreateVideoTrackOp;
  module->delete_video_track = HTTPInterface::DeleteVideoTrackOp;
  module->set_video_track_param = HTTPInterface::SetVideoTrackParamOp;
  module->start_session = HTTPInterface::StartSessionOp;
  module->stop_session = HTTPInterface::StopSessionOp;
  module->capture_image = HTTPInterface::CaptureImageOp;
  module->get_status = HTTPInterface::GetStatusOp;
  module->create_audio_track = HTTPInterface::CreateAudioTrackOp;
  module->delete_audio_track = HTTPInterface::DeleteAudioTrackOp;
  module->set_audio_track_param = HTTPInterface::SetAudioTrackParamOp;
  module->vam_config = HTTPInterface::VAMConfigOp;
  module->vam_remove_config = HTTPInterface::VAMRemoveConfigOp;
  module->vam_enroll_data = HTTPInterface::VAMEnrollOp;
  module->vam_disenroll_data = HTTPInterface::VAMDisenrollOp;
  module->set_camera_params = HTTPInterface::SetCameraParamsOp;
  module->create_overlay = HTTPInterface::CreateOverlayOp;
  module->delete_overlay = HTTPInterface::DeleteOverlayOp;
  module->set_overlay = HTTPInterface::SetOverlayOp;
  module->remove_overlay = HTTPInterface::RemoveOverlayOp;
  module->get_overlay = HTTPInterface::GetOverlayOp;
  module->update_overlay = HTTPInterface::UpdateOverlayOp;
  module->create_multicamera = HTTPInterface::CreateMultiCameraOp;
  module->configure_multicamera = HTTPInterface::ConfigureMultiCameraOp;
  module->database_command = HTTPInterface::DatabaseCommandOp;

  return NO_ERROR;
}

int32_t HTTPInterface::Close(struct qmmf_module_t *module) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;
  delete httpIntf;
  memset(module, 0, sizeof(struct qmmf_module_t));

  return NO_ERROR;
}

int32_t HTTPInterface::ConnectOp(struct qmmf_module_t *module) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->Connect();
}

int32_t HTTPInterface::StartCameraOp(struct qmmf_module_t *module,
                                     uint32_t camera_id,
                                     qmmf_camera_start_param start_parms) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->StartCamera(camera_id, start_parms);
}

int32_t HTTPInterface::CreateSessionOp(struct qmmf_module_t *module,
                                       uint32_t *session_id) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == session_id) {
    ALOGE("%s: Invalid session Id!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->CreateSession(session_id);
}

int32_t HTTPInterface::DeleteSessionOp(struct qmmf_module_t *module,
                                       uint32_t session_id) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->DeleteSession(session_id);
}

int32_t HTTPInterface::StopCameraOp(struct qmmf_module_t *module,
                                    uint32_t camera_id) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->StopCamera(camera_id);
}

int32_t HTTPInterface::CreateVideoTrackOp(struct qmmf_module_t *module,
                                          qmmf_video_track_param track_parm) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->CreateVideoTrack(track_parm);
}

int32_t HTTPInterface::DeleteVideoTrackOp(struct qmmf_module_t *module,
                                          uint32_t session_id, uint32_t track_id) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->DeleteVideoTrack(session_id, track_id);
}

int32_t HTTPInterface::CreateAudioTrackOp(struct qmmf_module_t *module,
                                          qmmf_audio_track_param track_parm) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->CreateAudioTrack(track_parm);
}

int32_t HTTPInterface::DeleteAudioTrackOp(struct qmmf_module_t *module,
                                          uint32_t session_id, uint32_t track_id) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->DeleteAudioTrack(session_id, track_id);
}

int32_t HTTPInterface::SetAudioTrackParamOp(struct qmmf_module_t *module,
                                            uint32_t session_id,
                                            uint32_t track_id,
                                            qmmf_track_param param) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->SetAudioTrackParam(session_id, track_id, param);
}

int32_t HTTPInterface::SetVideoTrackParamOp(struct qmmf_module_t *module,
                                            uint32_t session_id,
                                            uint32_t track_id,
                                            qmmf_track_param param) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->SetVideoTrackParam(session_id, track_id, param);
}

int32_t HTTPInterface::StartSessionOp(struct qmmf_module_t *module,
                                      uint32_t session_id) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->StartSession(session_id);
}

int32_t HTTPInterface::StopSessionOp(struct qmmf_module_t *module,
                                     uint32_t session_id, uint32_t flush) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->StopSession(session_id, flush);
}

qmmf_image_result HTTPInterface::CaptureImageOp(struct qmmf_module_t *module,
                                                qmmf_image_param image_param) {
  qmmf_image_result ret;
  memset(&ret, 0, sizeof(ret));

  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return ret;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return ret;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->CaptureImage(image_param);
}

int32_t HTTPInterface::VAMConfigOp(struct qmmf_module_t *module,
                                   const char *json_config) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  VAMInterface *vamIntf = ((HTTPInterface *)module->priv)->vam_interface_;

  return vamIntf->VAMConfig(json_config);
}

int32_t HTTPInterface::VAMRemoveConfigOp(struct qmmf_module_t *module,
                                         const char *json_config) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  VAMInterface *vamIntf = ((HTTPInterface *)module->priv)->vam_interface_;

  return vamIntf->VAMRemoveConfig(json_config);
}

int32_t HTTPInterface::VAMEnrollOp(struct qmmf_module_t *module,
                                   qmmf_vam_enrollment_info enroll_info) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  VAMInterface *vamIntf = ((HTTPInterface *)module->priv)->vam_interface_;
  VAMEnrollmentInfo vam_enrollment_info(enroll_info.id,
                                        enroll_info.display_name,
                                        enroll_info.img_id,
                                        enroll_info.data,
                                        enroll_info.object_type,
                                        enroll_info.event_type,
                                        enroll_info.image_format,
                                        enroll_info.image_width,
                                        enroll_info.image_height);

  return vamIntf->VAMEnroll(vam_enrollment_info);
}

int32_t HTTPInterface::VAMDisenrollOp(struct qmmf_module_t *module,
                                      uint32_t event_type,
                                      const char *id) {
  if (NULL == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (NULL == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  VAMInterface *vamIntf = ((HTTPInterface *)module->priv)->vam_interface_;

  return vamIntf->VAMDisenroll(event_type, id);
}

int32_t HTTPInterface::SetCameraParamsOp(struct qmmf_module_t *module,
                                         qmmf_camera_parameters params) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->SetCameraParams(params);
}

int32_t HTTPInterface::CreateOverlayOp(struct qmmf_module_t *module,
                                       uint32_t track_id,
                                       uint32_t *overlay_id,
                                       struct qmmf_overlay_param_t *params) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->CreateOverlay(track_id, overlay_id, params);
}

int32_t HTTPInterface::DeleteOverlayOp (struct qmmf_module_t *module,
                                        uint32_t track_id,
                                        uint32_t overlay_id) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->DeleteOverlay(track_id, overlay_id);
}

int32_t HTTPInterface::SetOverlayOp(struct qmmf_module_t *module,
                                    uint32_t track_id,
                                    uint32_t overlay_id) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->SetOverlay(track_id, overlay_id);
}

int32_t HTTPInterface::RemoveOverlayOp(struct qmmf_module_t *module,
                                       uint32_t track_id,
                                       uint32_t overlay_id) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->RemoveOverlay(track_id, overlay_id);
}

int32_t HTTPInterface::GetOverlayOp(struct qmmf_module_t *module,
                                    uint32_t track_id,
                                    uint32_t overlay_id,
                                    struct qmmf_overlay_param_t *params) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->GetOverlay(track_id, overlay_id, params);
}

int32_t HTTPInterface::UpdateOverlayOp(struct qmmf_module_t *module,
                                       uint32_t track_id, uint32_t overlay_id,
                                       struct qmmf_overlay_param_t *params) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->UpdateOverlay(track_id, overlay_id, params);
}

int32_t HTTPInterface::CreateMultiCameraOp(struct qmmf_module_t *module,
                                           const uint32_t *camera_ids,
                                           uint32_t num_camera,
                                           uint32_t *virtual_camera_id) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }
  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->CreateMultiCamera(camera_ids, num_camera, virtual_camera_id);
}

int32_t HTTPInterface::ConfigureMultiCameraOp(struct qmmf_module_t *module,
                                              qmmf_multi_camera_param_t *params)
                                                  {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }
  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->ConfigureMultiCamera(params);
}

#ifdef QMMF_LE_BUILD
int32_t HTTPInterface::RaveTrackResolutionChangeCbOp(void *handle,
                                                     uint32_t width,
                                                     uint32_t height,
                                                     uint32_t framerate,
                                                     uint32_t bitrate) {
  HTTPInterface *httpIntf = static_cast<HTTPInterface*>(handle);
  return httpIntf->RaveTrackResolutionChangeCb(width, height,
                                               framerate, bitrate);
}

int32_t HTTPInterface::RaveTrackQualityChangeCbOp(void *handle,
                                                  uint32_t framerate,
                                                  uint32_t bitrate) {
  HTTPInterface *httpIntf = static_cast<HTTPInterface*>(handle);;
  return httpIntf->RaveTrackQualityChangeCb(framerate, bitrate);
}
#endif

qmmf_db_result HTTPInterface::DatabaseCommandOp(struct qmmf_module_t *module,
                                                struct qmmf_db_param_t *params) {
  qmmf_db_result res;
  memset(&res, 0, sizeof(res));
  res.status = BAD_VALUE;
  if (NULL == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return res;
  }

  if (NULL == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return res;
  }

  VAMDatabaseCmdParams vam_params;
  VAMDatabaseCmdResult vam_result;
  vam_params.pts           = params->pts;
  vam_params.pts1          = params->pts1;
  vam_params.max_count     = params->max_count;
  vam_params.command       = params->command;
  vam_params.event_type    = params->event_type;
  vam_params.num_event_types = params->num_event_type;
  vam_params.id            = params->id;
  vam_params.session       = params->session;

  VAMInterface *vamIntf = ((HTTPInterface *) module->priv)->vam_interface_;

  vamIntf->DatabaseCommand(&vam_params, &vam_result);

  res.command        = vam_result.command;
  res.data           = vam_result.data;
  res.data_size      = vam_result.data_size;
  res.elements_count = vam_result.elements_count;
  res.status         = vam_result.status;
  return res;
}

qmmf_status * HTTPInterface::GetStatusOp(struct qmmf_module_t *module) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return nullptr;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return nullptr;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->GetStatus();
}

int32_t HTTPInterface::DisconnectOp(struct qmmf_module_t *module) {
  if (nullptr == module) {
    ALOGE("%s: Bad module parameter!", __func__);
    return BAD_VALUE;
  }

  if (nullptr == module->priv) {
    ALOGE("%s: No valid module!", __func__);
    return BAD_VALUE;
  }

  HTTPInterface *httpIntf = (HTTPInterface *) module->priv;

  return httpIntf->Disconnect();
}

int32_t HTTPInterface::Connect() {
  RecorderCb recorder_status_cb;
  recorder_status_cb.event_cb = [&] ( EventType event_type, void *event_data,
          size_t event_data_size) { RecorderEventCb(event_type,event_data,
                                                    event_data_size); };

  return recorder_.Connect(recorder_status_cb);
}

int32_t HTTPInterface::StartCamera(uint32_t camera_id,
                                   qmmf_camera_start_param start_parms) {
  CameraStartParam params;
  memset(&params, 0, sizeof(params));
  params.flags = start_parms.flags;
  params.frame_rate = start_parms.frame_rate;
  params.zsl_height = start_parms.zsl_height;
  params.zsl_width = start_parms.zsl_width;
  params.zsl_mode = start_parms.zsl_mode;
  params.zsl_queue_depth = start_parms.zsl_queue_depth;
  CameraResultCb result_cb = [&] (uint32_t camera_id,
                                  const CameraMetadata &result) {
                                  CameraCb(camera_id, result);};
  auto ret = recorder_.StartCamera(camera_id, params, result_cb);
  if (NO_ERROR == ret) {
    Mutex::Autolock l(lock_);
    ssize_t idx = cameras_.indexOfKey(camera_id);
    if (NAME_NOT_FOUND == idx) {
      cameras_.add(camera_id, start_parms);
    } else {
      cameras_.replaceValueAt(camera_id, start_parms);
    }
  }

  CameraMetadata static_info;
  ret = recorder_.GetDefaultCaptureParam(camera_id, static_info);
  if (NO_ERROR != ret) {
    ALOGE("%s Unable to query static camera parameters!\n",
          __func__);
    return ret;
  } else {
    Mutex::Autolock l(lock_);
    ssize_t idx = camera_configs_.indexOfKey(camera_id);
    if (NAME_NOT_FOUND == idx) {
      CameraConfiguration *config = new CameraConfiguration(static_info);
      if (nullptr == config) {
        ALOGE("%s: Unable to instantiate camera configuration!\n", __func__);
        return NO_MEMORY;
      }
      camera_configs_.add(camera_id, config);
    }
  }

  return ret;
}

int32_t HTTPInterface::StopCamera(uint32_t camera_id) {
  auto ret = recorder_.StopCamera(camera_id);
  if (NO_ERROR == ret) {
    Mutex::Autolock l(lock_);
    ssize_t idx = cameras_.indexOfKey(camera_id);
    if (NAME_NOT_FOUND == idx) {
      ALOGE("%s: Camera with id: %d not found in status!\n",
            __func__, camera_id);
    } else {
      cameras_.removeItemsAt(camera_id, 1);
    }

    idx = camera_configs_.indexOfKey(camera_id);
    if (NAME_NOT_FOUND == idx) {
      ALOGE("%s: Camera with id: %d doesn't have configuration!\n",
            __func__, camera_id);
    } else {
      CameraConfiguration *config = camera_configs_.valueAt(idx);
      delete config;
      camera_configs_.removeItemsAt(camera_id, 1);
    }
  }

  return ret;
}

int32_t HTTPInterface::CreateSession(uint32_t *session_id) {
  SessionCb cb;
  cb.event_cb = [&] (EventType event_type, void *event_data,
      size_t event_data_size) { SessionEventCb(event_type, event_data,
                                              event_data_size); };

  return recorder_.CreateSession(cb, session_id);
}

int32_t HTTPInterface::DeleteSession(uint32_t session_id) {
  auto ret = recorder_.DeleteSession(session_id);

  Mutex::Autolock l(lock_);
  ssize_t idx = muxers_.indexOfKey(session_id);
  if (NAME_NOT_FOUND != idx) {
    MuxInterface *mux = muxers_.valueAt(idx);
    muxers_.removeItem(session_id);
    delete mux;
    muxer_params_.removeItem(session_id);
  }

  idx = rtsp_servers_.indexOfKey(session_id);
  if (NAME_NOT_FOUND != idx) {
    rtsp_servers_.removeItem(session_id);
  }

  return ret;
}

int32_t HTTPInterface::CreateVideoTrack(qmmf_video_track_param track_parms) {
  struct VideoTrackCreateParam video_track_param;
  memset(&video_track_param, 0x0, sizeof video_track_param);

  video_track_param.camera_id     = track_parms.camera_id;
  video_track_param.width         = track_parms.width;
  video_track_param.height        = track_parms.height;
  video_track_param.frame_rate    = track_parms.framerate;
  video_track_param.low_power_mode = (0 == track_parms.low_power_mode) ?
      false : true;
  switch (track_parms.codec) {
    case CODEC_HEVC:
      video_track_param.format_type = VideoFormat::kHEVC;
      video_track_param.codec_param.hevc.idr_interval = 1;
      video_track_param.codec_param.hevc.bitrate = track_parms.bitrate;
      video_track_param.codec_param.hevc.profile = HEVCProfileType::kMain;
      video_track_param.codec_param.hevc.level = HEVCLevelType::kLevel3;
      video_track_param.codec_param.hevc.ratecontrol_type =
          VideoRateControlType::kMaxBitrate;
      video_track_param.codec_param.hevc.qp_params.enable_init_qp = true;
      video_track_param.codec_param.hevc.qp_params.init_qp.init_IQP = 27;
      video_track_param.codec_param.hevc.qp_params.init_qp.init_PQP = 28;
      video_track_param.codec_param.hevc.qp_params.init_qp.init_BQP = 28;
      video_track_param.codec_param.hevc.qp_params.init_qp.init_QP_mode = 0x7;
      video_track_param.codec_param.hevc.qp_params.enable_qp_range = true;
      video_track_param.codec_param.hevc.qp_params.qp_range.min_QP = 10;
      video_track_param.codec_param.hevc.qp_params.qp_range.max_QP = 51;
      video_track_param.codec_param.hevc.qp_params.enable_qp_IBP_range = true;
      video_track_param.codec_param.hevc.qp_params.qp_IBP_range.min_IQP = 10;
      video_track_param.codec_param.hevc.qp_params.qp_IBP_range.max_IQP = 51;
      video_track_param.codec_param.hevc.qp_params.qp_IBP_range.min_PQP = 10;
      video_track_param.codec_param.hevc.qp_params.qp_IBP_range.max_PQP = 51;
      video_track_param.codec_param.hevc.qp_params.qp_IBP_range.min_BQP = 10;
      video_track_param.codec_param.hevc.qp_params.qp_IBP_range.max_BQP = 51;
      break;
    case CODEC_AVC:
      video_track_param.format_type = VideoFormat::kAVC;
      video_track_param.codec_param.avc.idr_interval = 1;
      video_track_param.codec_param.avc.bitrate  = track_parms.bitrate;
      video_track_param.codec_param.avc.profile = AVCProfileType::kHigh;
      video_track_param.codec_param.avc.level = AVCLevelType::kLevel3;
#ifdef QMMF_LE_BUILD
      if ((track_parms.session_id == rave_track_param_store_.session_id)
        && rave_track_started_
        && (TRACK_OUTPUT_RTSP == track_parms.output)) {
        video_track_param.codec_param.avc.ratecontrol_type =
            VideoRateControlType::kConstantSkipFrames;
      } else {
        video_track_param.codec_param.avc.ratecontrol_type =
            VideoRateControlType::kMaxBitrate;
      }
#else
      video_track_param.codec_param.avc.ratecontrol_type =
        VideoRateControlType::kVariableSkipFrames;
#endif
      video_track_param.codec_param.avc.qp_params.enable_init_qp = true;
      video_track_param.codec_param.avc.qp_params.init_qp.init_IQP = 27;
      video_track_param.codec_param.avc.qp_params.init_qp.init_PQP = 28;
      video_track_param.codec_param.avc.qp_params.init_qp.init_BQP = 28;
      video_track_param.codec_param.avc.qp_params.init_qp.init_QP_mode = 0x7;
      video_track_param.codec_param.avc.qp_params.enable_qp_range = true;
      video_track_param.codec_param.avc.qp_params.qp_range.min_QP = 10;
      video_track_param.codec_param.avc.qp_params.qp_range.max_QP = 51;
      video_track_param.codec_param.avc.qp_params.enable_qp_IBP_range = true;
      video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_IQP = 10;
      video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_IQP = 51;
      video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_PQP = 10;
      video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_PQP = 51;
      video_track_param.codec_param.avc.qp_params.qp_IBP_range.min_BQP = 10;
      video_track_param.codec_param.avc.qp_params.qp_IBP_range.max_BQP = 51;
      break;
    case CODEC_YUV:
      video_track_param.format_type = VideoFormat::kYUV;
      break;
    case CODEC_RDI:
      video_track_param.format_type = VideoFormat::kBayerRDI10BIT;
      break;
    case CODEC_RAWIDEAL:
      video_track_param.format_type = VideoFormat::kBayerIdeal;
      break;
    case CODEC_MAX:
    default:
      ALOGE("%s: Unsupported codec: %d", __func__, track_parms.codec);
      return BAD_VALUE;
  }

  TrackCb video_track_cb;
  video_track_cb.data_cb = [&] (uint32_t track_id, std::vector<BufferDescriptor>
                                buffers, std::vector<MetaData> meta_data)
                                {VideoTrackCb(track_id, buffers, meta_data);};

  video_track_cb.event_cb = [&] (__attribute__((unused)) uint32_t track_id,
                                 __attribute__((unused)) EventType event_type,
                                 __attribute__((unused)) void *event_data,
                                 __attribute__((unused)) size_t event_data_size)
                                 { /* TODO */ };

  auto ret = recorder_.CreateVideoTrack(track_parms.session_id,
                                        track_parms.track_id,
                                        video_track_param,
                                        video_track_cb);
  if (NO_ERROR != ret) {
    ALOGE("%s: Unable to create video track: %d", __func__,
               ret);
    return ret;
  }
  Mutex::Autolock l(lock_);

  qmmf_video_track_status status;
  memset(&status, 0, sizeof(status));
  status.width = track_parms.width;
  status.height = track_parms.height;
  status.framerate = track_parms.framerate;
  status.track_id = track_parms.track_id;
  status.session_id = track_parms.session_id;
  status.codec = track_parms.codec;
  status.camera_id = track_parms.camera_id;
  status.output = track_parms.output;
  status.bitrate = track_parms.bitrate;
  status.low_power_mode = track_parms.low_power_mode;

  int32_t stat;
  switch (track_parms.output) {
    case TRACK_OUTPUT_RTSP:
    case TRACK_OUTPUT_MPEGTS:
#ifdef QMMF_LE_BUILD
    if ((!rave_stopsession_)
        ||(rave_stopsession_
          && (track_parms.session_id != rave_track_param_store_.session_id))) {
      stat = AddRTSPVideoLocked(track_parms.session_id, track_parms.track_id,
                                video_track_param,
                                (qmmf_video_track_output) track_parms.output);
    } else {
        stat = NO_ERROR;
    }

      if (TRACK_OUTPUT_RTSP == track_parms.output) {
        //init rave
        if (false == rave_track_initialized_) {
          rave_track_param_store_ = track_parms;
          if (FPV_NO_ERROR == RaveInit()) {
            ALOGV("%s: fpv_rave_init Success.\n", __func__);
          } else {
            ALOGE("%s: fpv_rave_init disabled/failed or inited.\n", __func__);
          }
        }
      }
#else
      stat = AddRTSPVideoLocked(track_parms.session_id, track_parms.track_id,
                                video_track_param,
                                (qmmf_video_track_output) track_parms.output);

#endif
      if (INVALID_OPERATION == stat) {
        ALOGV("%s: RTSP link not supported!", __func__);
      } else if (NO_ERROR != stat) {
        ALOGE("%s: Unable to add video track to RTSP session: %d", __func__,
              stat);
        ret = stat;
        goto exit;
      }
      break;
    case TRACK_OUTPUT_VAM:
      ret = vam_interface_->InitVAM(status.session_id, status.track_id);
      if (NO_ERROR != ret) {
        ALOGE("%s: Unable to initialize VAM!", __func__);
        goto exit;
      }
      CameraBufferMetaData meta_data;
      memset(&meta_data, 0, sizeof(meta_data));

      ALOGD("%s: VAM track: width(%d)xhieght(%d)", __func__,
          track_parms.width, track_parms.height);
      //TODO: Remove format hardcoding once interface changes are in place.
      meta_data.format = BufferFormat::kNV12;
      meta_data.num_planes = 2;
      meta_data.plane_info[0].width = meta_data.plane_info[1].width =
          track_parms.width;
      meta_data.plane_info[0].height = meta_data.plane_info[1].height =
          track_parms.height;
      meta_data.plane_info[0].stride = VENUS_Y_STRIDE(COLOR_FMT_NV12,
          track_parms.width);
      meta_data.plane_info[0].scanline = VENUS_Y_SCANLINES(COLOR_FMT_NV12,
          track_parms.height);
      meta_data.plane_info[1].stride = VENUS_UV_STRIDE(COLOR_FMT_NV12,
          track_parms.width);
      meta_data.plane_info[1].scanline = VENUS_UV_SCANLINES(COLOR_FMT_NV12,
          track_parms.height);
      ret = vam_interface_->StartVAM(meta_data);
      if (NO_ERROR != ret) {
        ALOGE("%s: Failed to start VAM!", __func__);
        goto exit;
      }
      ALOGD("%s: VAM track started successfully!", __func__);
      stat = AddRTSPVideoLocked(track_parms.session_id, track_parms.track_id,
                                video_track_param,
                                (qmmf_video_track_output) track_parms.output);
      if (INVALID_OPERATION == stat) {
        ALOGV("%s: RTSP link not supported!", __func__);
      } else if (NO_ERROR != stat) {
        ALOGE("%s: Unable to add video track to RTSP server: %d", __func__,
              stat);
        ret = stat;
        goto exit;
      }

      stat = StartVAMVideoLogLocked(track_parms);
      if (stat) {
        ALOGE("%s: Unable to start vam video log track: %d", __func__, stat);
      }
      break;
    case TRACK_OUTPUT_MP4:
    case TRACK_OUTPUT_3GP:
      ret = AddVidMuxParmsLocked(track_parms.session_id, track_parms.track_id,
                                 video_track_param,
                                 (qmmf_video_track_output) track_parms.output);
      if (NO_ERROR != ret) {
        ALOGE("%s: Unable to setup video muxer parameters!\n", __func__);
        goto exit;
      }
      break;
    case TRACK_OUT_DSI:
      if (dsi_counter_ > 0) {
        ALOGE("%s: Only one DSI track is allowed!\n", __func__);
        ret = BAD_VALUE;
        goto exit;
      }
      ret = AddVidDisplayLocked(DisplayType::kPrimary, track_parms);
      if (NO_ERROR != ret) {
        ALOGE("%s: Unable to setup dsi parameters!\n", __func__);
        goto exit;
      }
      dsi_counter_++;
    case TRACK_OUTPUT_EVENT_LOGGER: // Do Nothing
      break;
    default:
      ALOGE("%s: Unsupported track output: %d\n", __func__,
            track_parms.output);
      ret = BAD_VALUE;
      goto exit;
  }
  session_map_.add(track_parms.track_id, track_parms.session_id);
  video_tracks_.add(track_parms.track_id, status);

  return ret;

exit:

  recorder_.DeleteVideoTrack(track_parms.session_id, track_parms.track_id);

  return ret;
}

int32_t HTTPInterface::DeleteVideoTrack(uint32_t session_id,
                                        uint32_t track_id) {
  Mutex::Autolock l(lock_);
  auto ret = recorder_.DeleteVideoTrack(session_id, track_id);
  if (NO_ERROR != ret) {
    ALOGE("%s: Unable to delete video track: %d\n", __func__, ret);
    return ret;
  }
  session_map_.removeItem(track_id);
  ssize_t idx = video_tracks_.indexOfKey(track_id);
  if (0 <= idx) {
    qmmf_video_track_status status = video_tracks_.valueAt(idx);
    video_tracks_.removeItem(track_id);

    switch(status.output) {
      case TRACK_OUTPUT_RTSP:
      case TRACK_OUTPUT_MPEGTS:
#ifdef QMMF_LE_BUILD
        if ((!rave_stopsession_)
            ||(rave_stopsession_
              && (session_id != rave_track_param_store_.session_id))) {
          ret = RemoveRTSPVideoLocked(session_id);
          if (NO_ERROR != ret) {
            ALOGE("%s: Failed to remove video track from RTSP Session:%d\n",
                  __func__, ret);
          }
        }
#else
        ret = RemoveRTSPVideoLocked(session_id);
        if (NO_ERROR != ret) {
          ALOGE("%s: Failed to remove video track from RTSP Session:%d\n",
                __func__, ret);
        }
#endif
        break;
      case TRACK_OUTPUT_VAM:
        if (vam_video_log_params_.session_id) {
          lock_.unlock();
          StopSession(vam_video_log_params_.session_id, true);
          DeleteVideoTrack(vam_video_log_params_.session_id,
                           vam_video_log_params_.track_id);
          DeleteSession(vam_video_log_params_.session_id);
          lock_.lock();
          memset(&vam_video_log_params_, 0, sizeof(vam_video_log_params_));

          vam_interface_->DeinitVAMVLog();
        }

        ret = vam_interface_->CloseVAM();
        if (NO_ERROR != ret) {
          ALOGE("%s: Unable to close VAM: %d", __func__,
                     ret);
        }

        ret = RemoveRTSPVideoLocked(session_id);
        if (NO_ERROR != ret) {
          ALOGE("%s: Failed to remove video track from RTSP Session:%d\n",
                __func__, ret);
        }
        break;
      case TRACK_OUTPUT_MP4:
      case TRACK_OUTPUT_3GP:
        ret = RemoveVidMuxParmsLocked(session_id);
        if (NO_ERROR != ret) {
          ALOGE("%s: Failed to remote video track from muxer!\n", __func__);
        }

        break;
      case TRACK_OUT_DSI:
        if (display_started_ == 1) {
          ret = RemoveVidDisplayLocked(DisplayType::kPrimary);
          if (ret != NO_ERROR) {
            ALOGE("%s Stop Display Failed\n", __func__);
          }
          dsi_counter_--;
        }
      case TRACK_OUTPUT_EVENT_LOGGER: // Do Nothing
        break;
      default:
        ALOGE("%s: Unsupported track output: %d\n", __func__,
              status.output);
        ret = BAD_VALUE;
    }
  } else {
    ALOGE("%s: No status data for track id: %d", __func__,
               track_id);
    ret = BAD_VALUE;
  }
  return ret;
}

// The purpose of this method is to ensure that CodecParamType and param_type
// enums are exactly the same.
param_type HTTPInterface::ValidateParamType(CodecParamType param) {
  param_type type;
  switch (param) {
    case CodecParamType::kBitRateType:
      type = kBitRateType;
      break;
    case CodecParamType::kFrameRateType:
      type = kFrameRateType;
      break;
    case CodecParamType::kInsertIDRType:
      type = kInsertIDRType;
      break;
    case CodecParamType::kIDRIntervalType:
      type = kIDRIntervalType;
      break;
    case CodecParamType::kCamFrameCropType:
      type = kCamFrameCropType;
      break;
    case CodecParamType::kMarkLtrType:
      type = kMarkLtrType;
      break;
    case CodecParamType::kUseLtrType:
      type = kUseLtrType;
      break;
    case CodecParamType::kAudioEffectsParamType:
      type = kAudioEffectsParamType;
      break;
    case CodecParamType::kAudioVolumeParamType:
      type = kAudioVolumeParamType;
      break;
    case CodecParamType::kDecodeOperatingRate:
      type = kDecodeOperatingRate;
      break;
    case CodecParamType::kEnableFrameRepeat:
      type = kEnableFrameRepeat;
      break;
    case CodecParamType::kVQZipInfo:
      type = kVQZipInfoType;
      break;
  }
  return type;
}

CodecParamType HTTPInterface::SetCodecParamType(param_type type) {
  CodecParamType param;
  switch (type) {
    case kBitRateType:
      param = CodecParamType::kBitRateType;
      break;
    case kFrameRateType:
      param = CodecParamType::kFrameRateType;
      break;
    case kInsertIDRType:
      param = CodecParamType::kInsertIDRType;
      break;
    case kIDRIntervalType:
      param = CodecParamType::kIDRIntervalType;
      break;
    case kCamFrameCropType:
      param = CodecParamType::kCamFrameCropType;
      break;
    case kMarkLtrType:
      param = CodecParamType::kMarkLtrType;
      break;
    case kUseLtrType:
      param = CodecParamType::kUseLtrType;
      break;
    case kAudioEffectsParamType:
      param = CodecParamType::kAudioEffectsParamType;
      break;
    case kAudioVolumeParamType:
      param = CodecParamType::kAudioVolumeParamType;
      break;
    case kDecodeOperatingRate:
      param = CodecParamType::kDecodeOperatingRate;
      break;
    case kEnableFrameRepeat:
      param = CodecParamType::kEnableFrameRepeat;
      break;
    case kVQZipInfoType:
      param = CodecParamType::kVQZipInfo;
      break;
  }
  return param;
}

int32_t HTTPInterface::SetVideoTrackParam(uint32_t session_id, uint32_t track_id,
                                          qmmf_track_param param) {
  CodecParamType param_type = SetCodecParamType(param.type);

  auto ret = recorder_.SetVideoTrackParam(session_id, track_id, param_type,
                                          param.param, param.param_size);
  if (NO_ERROR != ret) {
    ALOGE("%s: Unable to set video track param: %d", __func__, ret);
    return ret;
  }

  return ret;
}

int32_t HTTPInterface::CreateAudioTrack(qmmf_audio_track_param track_parms) {
  AudioTrackCreateParam audio_track_params;
  memset(&audio_track_params, 0, sizeof(audio_track_params));

  switch(track_parms.codec) {
    case CODEC_PCM:
      audio_track_params.format = AudioFormat::kPCM;
      break;
    case CODEC_AAC:
      audio_track_params.format = AudioFormat::kAAC;
      audio_track_params.codec_params.aac.format = AACFormat::kADTS;
      audio_track_params.codec_params.aac.mode = AACMode::kAALC;
      audio_track_params.codec_params.aac.bit_rate = track_parms.bitrate;
      break;
    case CODEC_AMR:
      audio_track_params.format = AudioFormat::kAMR;
      audio_track_params.codec_params.amr.isWAMR = false;
      audio_track_params.codec_params.amr.bit_rate = track_parms.bitrate;
      break;
    case CODEC_AMRWB:
      audio_track_params.format = AudioFormat::kAMR;
      audio_track_params.codec_params.amr.isWAMR = true;
      audio_track_params.codec_params.amr.bit_rate = track_parms.bitrate;
      break;
    default:
      ALOGE("%s: Unsupported audio codec type: %d\n",
            __func__, track_parms.codec);
      return BAD_VALUE;
  }

  //TODO: Check if any these should be user configurable at some point
  audio_track_params.in_devices_num = 0;
  audio_track_params.in_devices[audio_track_params.in_devices_num++] =
    static_cast<DeviceId>(AudioDeviceId::kBuiltIn);
  audio_track_params.flags       = 0;

  audio_track_params.sample_rate = track_parms.sample_rate;
  audio_track_params.channels = track_parms.num_channels;
  audio_track_params.bit_depth = track_parms.bit_depth;

  TrackCb audio_track_cb;
  audio_track_cb.data_cb = [&] (uint32_t track_id, std::vector<BufferDescriptor>
                                buffers, std::vector<MetaData> meta_data)
                                {AudioTrackCb(track_id, buffers, meta_data);};

  audio_track_cb.event_cb = [&] (__attribute__((unused)) uint32_t track_id,
                                 __attribute__((unused)) EventType event_type,
                                 __attribute__((unused)) void *event_data,
                                 __attribute__((unused)) size_t event_data_size)
                                 { /* TODO */ };

  auto ret = recorder_.CreateAudioTrack(track_parms.session_id,
                                        track_parms.track_id,
                                        audio_track_params,
                                        audio_track_cb);
  if (NO_ERROR != ret) {
    ALOGE("%s: Unable to create video track: %d", __func__,
               ret);
    return ret;
  }

  Mutex::Autolock l(lock_);
  session_map_.add(track_parms.track_id, track_parms.session_id);

  int32_t stat;
  switch (track_parms.output) {
    case AUDIO_TRACK_OUTPUT_MPEGTS:
      stat = AddRTSPAudioLocked(track_parms.session_id, track_parms);
      if (INVALID_OPERATION == stat) {
        ALOGV("%s: RTSP link not supported!", __func__);
      } else if (NO_ERROR != stat) {
        ALOGE("%s: Unable to add audio track to RTSP session: %d", __func__,
              stat);
        ret = stat;
        goto exit;
      }
      break;
    case AUDIO_TRACK_OUTPUT_MP4:
    case AUDIO_TRACK_OUTPUT_3GP:
      ret = AddAudMuxParmsLocked(track_parms);
      if (NO_ERROR != ret) {
        ALOGE("%s: Unable to setup audio muxer parameters!\n", __func__);
        goto exit;
      }
      break;
    default:
      ALOGE("%s: Unsupported track output: %d\n", __func__,
            track_parms.output);
      ret = BAD_VALUE;
      goto exit;
  }
  audio_tracks_.add(track_parms.track_id, track_parms);

  return ret;

exit:

  recorder_.DeleteAudioTrack(track_parms.session_id, track_parms.track_id);

  return ret;
}

int32_t HTTPInterface::DeleteAudioTrack(uint32_t session_id,
                                        uint32_t track_id) {
  auto ret = recorder_.DeleteAudioTrack(session_id, track_id);
  if (NO_ERROR != ret) {
    ALOGE("%s: Unable to delete audio track: %d\n", __func__, ret);
    return ret;
  }

  Mutex::Autolock l(lock_);
  session_map_.removeItem(track_id);
  ssize_t idx = audio_tracks_.indexOfKey(track_id);
  if (0 <= idx) {
    qmmf_audio_track_param status = audio_tracks_.valueAt(idx);
    audio_tracks_.removeItem(track_id);

    switch(status.output) {
      case AUDIO_TRACK_OUTPUT_MPEGTS:
        ret = RemoveRTSPAudioLocked(session_id);
        if (NO_ERROR !=ret ) {
          ALOGE("%s: Failed to remove audio track from RTSP session!\n",
                __func__);
        }
        break;
      case AUDIO_TRACK_OUTPUT_MP4:
      case AUDIO_TRACK_OUTPUT_3GP:
        ret = RemoveAudMuxParmsLocked(session_id);
        if (NO_ERROR != ret) {
          ALOGE("%s: Failed to remove audio track from muxer!\n", __func__);
        }
        break;
      default:
        ALOGE("%s: Unsupported track output: %d\n", __func__,
              status.output);
        ret = BAD_VALUE;
    }
  } else {
    ALOGE("%s: No status data for track id: %d", __func__,
               track_id);
    ret = BAD_VALUE;
  }

  return ret;
}

int32_t HTTPInterface::SetAudioTrackParam(uint32_t session_id, uint32_t track_id,
                                          qmmf_track_param param) {
  CodecParamType param_type = SetCodecParamType(param.type);

  auto ret = recorder_.SetAudioTrackParam(session_id, track_id, param_type,
                                          param.param, param.param_size);
  if (NO_ERROR != ret) {
    ALOGE("%s: Unable to set audio track param: %d", __func__, ret);
    return ret;
  }

  return ret;
}

int32_t HTTPInterface::Disconnect() {
  return recorder_.Disconnect();
}

#ifdef QMMF_LE_BUILD
int32_t HTTPInterface::RaveInit() {
  if (false == rave_track_initialized_) {
    parse_config_file();
    rave_track_initialized_ = true;
    if (true == get_rave_status()) {
      rave_init();
      register_id_to_rave(HTTPInterface::RaveTrackResolutionChangeCbOp,
                          HTTPInterface::RaveTrackQualityChangeCbOp, this);
      return FPV_NO_ERROR;
    } else {
      return FPV_RAVE_DISABLED;
    }
  }
  return FPV_NO_ERROR;
}

int32_t HTTPInterface::RaveStart(){
  int32_t status = FPV_NO_ERROR;
  if (!ra_get_fpv_started()) {
    status = start_fpv_ra(FPV_RAVE_THREAD_PRI);
  }
  return status;
}

int32_t HTTPInterface::RaveExit(){
  int32_t status = FPV_NO_ERROR;
  if (ra_get_fpv_started()) {
    status = stop_fpv_ra();
    ALOGV("%s: rave_exit Success.\n", __func__);
  }
  rave_stopsession_ = false;
  rave_track_started_ = false;
  rave_track_initialized_ = false;
  set_rave_status(false);
  return status;
}
#endif

int32_t HTTPInterface::StartSession(uint32_t session_id) {
  {
    Mutex lock_;
    ssize_t muxer_idx = muxer_params_.indexOfKey(session_id);
    if (NAME_NOT_FOUND != muxer_idx) {
      qmmf_muxer_init init_params = muxer_params_.valueAt(muxer_idx);
      auto ret = InitMuxerLocked(session_id, init_params);
      if (NO_ERROR != ret) {
        ALOGE("%s: Unable to initialize muxer!\n", __func__);
        return ret;
      }
    }

#ifdef QMMF_LE_BUILD
    ssize_t rtsp_idx = rtsp_servers_.indexOfKey(session_id);
    if (NAME_NOT_FOUND != rtsp_idx) {
      if ((!rave_stopsession_)
          ||(rave_stopsession_
            && (session_id != rave_track_param_store_.session_id))) {
        auto ret = InitRTSPServerLocked(session_id);
        if (NO_ERROR != ret) {
          ALOGE("%s: Unable to initialize RTSP server: %d!\n", __func__, ret);
          return ret;
        }
      }
      //start FPV rave
      if ((true == rave_track_initialized_)
          && (session_id == rave_track_param_store_.session_id)
          && (false == rave_track_started_)
          && (true == get_rave_status())) {
        if (FPV_NO_ERROR == RaveStart()) {
          ALOGV("%s: rave_start Success.\n", __func__);
          rave_track_started_ = true;
        } else {
          ALOGE("%s: rave_start failed.\n", __func__);
        }
      } else {
        ALOGV("%s: uninited or session mismatch or already started.\n", __func__);
      }
    }
#else
    auto ret = InitRTSPServerLocked(session_id);
    if (NO_ERROR != ret) {
      ALOGE("%s: Unable to initialize RTSP server: %d!\n", __func__, ret);
      return ret;
    }
#endif
  }

  return recorder_.StartSession(session_id);
}

int32_t HTTPInterface::StopSession(uint32_t session_id, uint32_t flush) {
  int32_t status = NO_ERROR;
  Mutex lock_;
  ssize_t idx = muxers_.indexOfKey(session_id);
  if (NAME_NOT_FOUND != idx) {
    MuxInterface *mux = muxers_.valueAt(idx);
    status = mux->Stop();
    if (NO_ERROR != status) {
      ALOGE("%s: Failed to stop muxer: %d\n", __func__, status);
    }
  }

#ifdef QMMF_LE_BUILD
  idx = rtsp_servers_.indexOfKey(session_id);
  if (NAME_NOT_FOUND != idx) {
    if ((!rave_stopsession_)
        && (session_id == rave_track_param_store_.session_id)
        && (true == rave_track_started_)) {
      RaveExit();
    }
    if (!rave_stopsession_) {
      status = CloseRTSPServerLocked(session_id);
      if (NO_ERROR != status) {
        ALOGE("%s: Failed to close RTSP Server: %d\n", __func__, status);
      }
    }
  }
#else
  status = CloseRTSPServerLocked(session_id);
  if (NO_ERROR != status) {
    ALOGE("%s: Failed to close RTSP Server: %d\n", __func__, status);
  }
#endif

  auto ret = recorder_.StopSession(session_id, flush);

  return ret | status;
}

qmmf_image_result HTTPInterface::CaptureImage(qmmf_image_param image_args) {
  ALOGI("%s : %s : Enter ",LOG_TAG,__func__);
  std::vector<CameraMetadata> meta;
  ImageParam image_param;
  memset(&image_param, 0x0, sizeof image_param);
  meta.clear();
  memset(&snapshot_result_, 0, sizeof(snapshot_result_));

  image_param.width = image_args.width;
  image_param.height = image_args.height;
  image_param.image_format = ImageFormat::kJPEG;
  image_param.image_quality = image_args.quality;

  ImageCaptureCb cb = [&] (uint32_t camera_id, uint32_t image_sequence_count,
      BufferDescriptor buffer, MetaData meta_data) {
      SnapshotCb(camera_id, image_sequence_count, buffer, meta_data); };
  ALOGD("%s : %s : Waiting For AEC ",LOG_TAG,__func__);
  pthread_mutex_lock(&aec_converge_lock_);
  while(!aec_converged_) {
    pthread_cond_wait(&aec_converge_cond_, &aec_converge_lock_);
  }
  pthread_mutex_unlock(&aec_converge_lock_);
  ALOGD("%s : %s : Waiting For AEC Done ",LOG_TAG,__func__);
  pthread_mutex_lock(&snapshot_lock_);


  int32_t repeat = kRepeat;
  do {
    camera_error_ = false;
    snapshot_completed_ = false;
    // Only 1 image at a time is supported for now.
    ALOGD("%s :%s Capture Image %d",LOG_TAG, __func__, repeat);
    auto status = recorder_.CaptureImage(image_args.camera_id, image_param,
                                         1, meta, cb);
    if (NO_ERROR != status) {
      ALOGE("%s : %s: Capture image failed: %d\n",LOG_TAG, __func__, status);
      goto exit;
    }

    int ret;
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    tv.tv_sec += kSnapshotTimeout;
    while(!snapshot_completed_) {
      ret = pthread_cond_timedwait(&snapshot_cond_, &snapshot_lock_, &tv);
      if (ret == ETIMEDOUT) {
          ALOGD("%s : %s: Failed! Capture image timeout in %d seconds\n",LOG_TAG, __func__, kSnapshotTimeout);
          break;
      }
    }
    if (!camera_error_) {
      ALOGD("%s :%s Capture Image Done",LOG_TAG, __func__);
      break;
    }
  } while (repeat-- > 0);

exit:

  ALOGD("%s :%s Capture Image END %d",LOG_TAG, __func__, repeat);
  pthread_mutex_unlock(&snapshot_lock_);

  return snapshot_result_;
}

qmmf_status * HTTPInterface::GetStatus() {
  Mutex::Autolock l(lock_);
  qmmf_status *ret = nullptr;
  qmmf_camera_status camera_status;
  size_t camera_count;
  size_t video_track_count;
  size_t audio_track_count;

  ret = (qmmf_status *) malloc(sizeof(*ret));
  if (nullptr == ret) {
    ALOGE("%s: Unable to allocate status structure!\n", __func__);
    goto EXIT;
  }
  memset(ret, 0, sizeof(*ret));

  video_track_count = video_tracks_.size();
  if (0 < video_track_count) {
    ret->tracks = (qmmf_video_track_status *) malloc(
        sizeof(qmmf_video_track_status)*video_track_count);
    if (nullptr == ret->tracks) {
      ALOGE("%s: Unable to allocate video track status!\n", __func__);
      goto EXIT;
    }

    ret->num_tracks = video_track_count;
    for (size_t i = 0; i < video_track_count; i++) {
      ret->tracks[i] = video_tracks_.valueAt(i);
    }
  }

  camera_count = cameras_.size();
  if (0 < camera_count) {
    ret->cameras = (qmmf_camera_status *) malloc(
        sizeof(qmmf_camera_status) * camera_count);
    if (nullptr == ret->cameras) {
      ALOGE("%s: Unable to allocate camera status!\n", __func__);
      goto EXIT;
    }

    ret->num_cameras = camera_count;
    for (size_t i = 0; i < camera_count; i++) {
      memset(&camera_status, 0, sizeof(camera_status));
      camera_status.camera_id = cameras_.keyAt(i);
      camera_status.param = cameras_.valueAt(i);
      ssize_t idx = camera_configs_.indexOfKey(i);
      if (NAME_NOT_FOUND != idx) {
        CameraConfiguration *config = camera_configs_.valueAt(idx);
        camera_status.supported_nr_modes = config->GetSupportedNRModes();
        camera_status.supported_hdr_modes = config->GetSupportedHDRModes();
        camera_status.supported_ir_modes = config->GetSupportedIRModes();
      }
      ret->cameras[i] = camera_status;
    }
  }

  audio_track_count = audio_tracks_.size();
  if (0 < audio_track_count) {
    ret->audio_tracks = (qmmf_audio_track_param *) malloc(
        sizeof(qmmf_audio_track_param) * audio_track_count);
    if (nullptr == ret->audio_tracks) {
      ALOGE("%s: Unable to allocate audio track status!\n", __func__);
      goto EXIT;
    }

    ret->num_audio_tracks = audio_track_count;
    for (size_t i = 0; i < audio_track_count; i++) {
      ret->audio_tracks[i] = audio_tracks_[i];
    }
  }

  return ret;

EXIT:

  if (nullptr != ret) {

    if (nullptr != ret->tracks) {
      free(ret->tracks);
    }

    if (nullptr != ret->cameras) {
      free(ret->cameras);
    }

    if (nullptr != ret->audio_tracks) {
      free(ret->audio_tracks);
    }

    free(ret);
  }

  return nullptr;
}

int32_t HTTPInterface::UpdateTrackRTSPURLLocked(uint32_t track_id,
                                                const char *url) {
  ssize_t idx = video_tracks_.indexOfKey(track_id);
  if (0 <= idx) {
    qmmf_video_track_status status = video_tracks_.valueAt(idx);
    status.rtsp_url = url;
    video_tracks_.replaceValueAt(idx, status);
  } else {
    ALOGE("%s: Track with id %d not present!\n", __func__, track_id);
    return BAD_VALUE;
  }

  return NO_ERROR;
}

int32_t HTTPInterface::InitRTSPServerLocked(uint32_t session_id) {
  RTSPContext rtsp_ctx;
  ssize_t idx = rtsp_servers_.indexOfKey(session_id);
  if (NAME_NOT_FOUND != idx) {
    rtsp_ctx = rtsp_servers_.valueAt(idx);
  } else {
    ALOGE("%s: RTSP server context missing!\n", __func__);
    return NO_INIT;
  }

  if (0 == rtsp_ctx.rtsp_port) {
    ALOGE("%s: Invalid RTSP port!\n", __func__);
    return NO_INIT;
  }

  if (nullptr != rtsp_ctx.rtsp_server) {
    ALOGD("%s: RTSP server already initialized!\n", __func__);
    return NO_ERROR;
  }

  rtsp_ctx.rtsp_server = new RtspServerInterface(rtsp_ctx.rtsp_port);
  if (nullptr == rtsp_ctx.rtsp_server) {
    ALOGE("%s: No memory for RTSP server!", __func__);
    return NO_MEMORY;
  }
  rtsp_ctx.rtsp_server->CreateSMS();

  if (rtsp_ctx.is_mp2ts) {
    rtsp_ctx.rtsp_server->AddESTsSMSSToSMS(rtsp_ctx.video_codec_id,
                                           rtsp_ctx.frame_rate,
                                           rtsp_ctx.rtsp_video_queue,
                                           rtsp_ctx.audio_codec_id,
                                           rtsp_ctx.audio_channels,
                                           rtsp_ctx.audio_idx,
                                           rtsp_ctx.audio_profile,
                                           rtsp_ctx.rtsp_audio_queue);
  } else {
    switch (rtsp_ctx.video_codec_id) {
      case VIDEO_FORMAT_H264:
      case VIDEO_FORMAT_H265:
        rtsp_ctx.rtsp_server->AddVideoSMSSToSMS(rtsp_ctx.video_codec_id,
                                                rtsp_ctx.frame_rate,
                                                rtsp_ctx.rtsp_video_queue);
        break;
      case VIDEO_FORMAT_YUV:
        rtsp_ctx.rtsp_server->AddMetaSMSSToSMS(rtsp_ctx.rtsp_meta_queue,
                                               rtsp_ctx.frame_rate);
        break;
      default:
        ALOGE("%s: Unsupported codec: %d\n",
              __func__, rtsp_ctx.video_codec_id);
        delete rtsp_ctx.rtsp_server;
        return INVALID_OPERATION;
    }
  }

  rtsp_ctx.rtsp_server->StartTaskScheduler();
  size_t url_size = rtsp_ctx.rtsp_server->GetURLSize();
  if (url_size) {
      rtsp_ctx.rtsp_url = (char *) malloc(url_size);
      if (rtsp_ctx.rtsp_url) {
          rtsp_ctx.rtsp_server->GetURL(rtsp_ctx.rtsp_url, url_size);

          if (0 < rtsp_ctx.video_track_id) {
            UpdateTrackRTSPURLLocked(rtsp_ctx.video_track_id,
                                     rtsp_ctx.rtsp_url);
          }

          if (0 < rtsp_ctx.meta_track_id) {
            UpdateTrackRTSPURLLocked(rtsp_ctx.meta_track_id,
                                     rtsp_ctx.rtsp_url);
          }
      } else {
        ALOGE("%s: No resources for URL string!", __func__);
      }
  } else {
    ALOGE("%s: RTSP URL size is invalid!", __func__);
  }

  rtsp_servers_.replaceValueAt(idx, rtsp_ctx);

  return NO_ERROR;
}

int32_t HTTPInterface::CloseRTSPServerLocked(uint32_t session_id)
{
  ssize_t idx = rtsp_servers_.indexOfKey(session_id);
  if (NAME_NOT_FOUND != idx) {
    RTSPContext rtsp_ctx = rtsp_servers_.valueAt(idx);
    if (nullptr != rtsp_ctx.rtsp_server) {
      rtsp_ctx.rtsp_server->StopTaskScheduler();
      rtsp_ctx.rtsp_server->ResetRtspServer();
      delete rtsp_ctx.rtsp_server;
      rtsp_ctx.rtsp_server = nullptr;
    }

    if (nullptr != rtsp_ctx.rtsp_url) {
      free(rtsp_ctx.rtsp_url);
      rtsp_ctx.rtsp_url = nullptr;
    }

    rtsp_servers_.replaceValueAt(idx, rtsp_ctx);
  } else {
    ALOGE("%s: No RTSP server present for session id: %d", __func__,
          session_id);
  }

  return NO_ERROR;
}

int32_t HTTPInterface::AddAudMuxParmsLocked(const qmmf_audio_track_param &audio) {
  qmmf_muxer_init params;
  MUX_brand_type brand;
  switch(audio.output) {
    case AUDIO_TRACK_OUTPUT_MP4:
      brand = MUX_BRAND_MP4;
      break;
    case AUDIO_TRACK_OUTPUT_3GP:
      brand = MUX_BRAND_3GP;
      break;
    default:
      ALOGE("%s: Unsupported output:%d\n", __func__, audio.output);
      return BAD_VALUE;
  }

  ssize_t param_idx = muxer_params_.indexOfKey(audio.session_id);
  if (NAME_NOT_FOUND != param_idx) {
    params = muxer_params_.valueAt(param_idx);
    if (params.audio_stream_present) {
      ALOGE("%s: Audio stream already present in the session muxer!\n",
            __func__);
      return ALREADY_EXISTS;
    }
    if ((brand != params.brand) && (MUX_BRAND_INVALID != params.brand)) {
      ALOGE("%s: Muxer cannot support two different brands: %d vs. %d\n",
            __func__, brand, params.brand);
      return BAD_VALUE;
    }
  } else {
    memset(&params, 0, sizeof(params));
  }

  params.brand = brand;
  params.audio_stream = audio;
  params.audio_stream_present = true;

  if (NAME_NOT_FOUND != param_idx) {
    muxer_params_.replaceValueAt(param_idx, params);
  } else {
    muxer_params_.add(audio.session_id, params);
  }

  return NO_ERROR;
}

int32_t HTTPInterface::RemoveAudMuxParmsLocked(uint32_t session_id) {
  qmmf_muxer_init params;
  ssize_t param_idx = muxer_params_.indexOfKey(session_id);
  if (NAME_NOT_FOUND != param_idx) {
    params = muxer_params_.valueAt(param_idx);
    if (!params.audio_stream_present) {
      ALOGE("%s: Audio stream not present in session muxer!\n",
            __func__);
      return NO_INIT;
    }

    params.audio_stream_present = false;
    if (!params.video_stream_present) {
      params.brand = MUX_BRAND_INVALID;
    }
    muxer_params_.replaceValueAt(param_idx, params);
  } else {
    ALOGE("%s: No muxer found for this stream!\n", __func__);
    return BAD_VALUE;
  }

  return NO_ERROR;
}

int32_t HTTPInterface::getAVCProfileLevel(const VideoTrackCreateParam &video,
                                          uint8_t &level, uint8_t &profile) {
  size_t profile_count = TABLE_SIZE(kAVCMuxerProfiles);
  size_t profile_idx, level_idx;
  bool found = false;
  for (profile_idx = 0; profile_idx < profile_count; profile_idx++) {
    if (kAVCMuxerProfiles[profile_idx].profile ==
        video.codec_param.avc.profile) {
      found = true;
      break;
    }
  }
  if (!found) {
    ALOGE("%s: AVC profile not found!\n", __func__);
    return INVALID_OPERATION;
  }

  size_t level_count = TABLE_SIZE(kAVCMuxerLevels);
  found = false;
  for (level_idx = 0; level_idx < level_count; level_idx++) {
    if (kAVCMuxerLevels[level_idx].level == video.codec_param.avc.level) {
      found = true;
      break;
    }
  }
  if (!found) {
    ALOGE("%s: AVC level not found!\n", __func__);
    return INVALID_OPERATION;
  }

  profile = kAVCMuxerProfiles[profile_idx].value;
  level = kAVCMuxerLevels[level_idx].value;

  return NO_ERROR;
}

int32_t HTTPInterface::getHEVCProfileLevel(const VideoTrackCreateParam &video,
                                          uint8_t &level, uint8_t &profile) {
  size_t profile_count = TABLE_SIZE(kHEVCMuxerProfiles);
  size_t profile_idx, level_idx;
  bool found = false;
  for (profile_idx = 0; profile_idx < profile_count; profile_idx++) {
    if (kHEVCMuxerProfiles[profile_idx].profile ==
        video.codec_param.hevc.profile) {
      found = true;
      break;
    }
  }
  if (!found) {
    ALOGE("%s: HEVC profile not found!\n", __func__);
    return INVALID_OPERATION;
  }

  size_t level_count = TABLE_SIZE(kHEVCMuxerLevels);
  found = false;
  for (level_idx = 0; level_idx < level_count; level_idx++) {
    if (kHEVCMuxerLevels[level_idx].level == video.codec_param.hevc.level) {
      found = true;
      break;
    }
  }
  if (!found) {
    ALOGE("%s: HEVC level not found!\n", __func__);
    return INVALID_OPERATION;
  }

  profile = kHEVCMuxerProfiles[profile_idx].value;
  level = kHEVCMuxerLevels[level_idx].value;

  return NO_ERROR;
}

int32_t HTTPInterface::AddVidDisplayLocked(DisplayType display_type,
                                           qmmf_video_track_param track_parms) {
  int32_t ret = BAD_VALUE;
  SurfaceConfig surface_config;
  DisplayCb  display_status_cb;

  display_ = new Display();
  if (display_ == NULL) {
    ALOGE("%s: new Display failed\n", __func__);
    return NO_MEMORY;
  }

  ret = display_->Connect();
  if (ret != NO_ERROR) {
    ALOGE("%s: Connect to display error\n", __func__);
    delete display_;
    display_ = NULL;
    return ret;
  }

  display_status_cb.EventCb = [&] (__attribute__((unused)) DisplayEventType event_type,
                                   __attribute__((unused)) void *event_data,
                                   __attribute__((unused)) size_t event_data_size)
                                  { };

  display_status_cb.VSyncCb = [&] ( __attribute__((unused)) int64_t time_stamp)
                                  { };

  ret = display_->CreateDisplay(display_type, display_status_cb);
  if (ret != NO_ERROR) {
    ALOGE("%s: Create Display error\n", __func__);
    display_->Disconnect();
    delete display_;
    display_ = NULL;
    return ret;
  }

  memset(&surface_config, 0x0, sizeof surface_config);

  surface_config.width = track_parms.width;
  surface_config.height = track_parms.height;
  surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
  surface_config.buffer_count = 1;
  surface_config.cache = 0;
  surface_config.use_buffer = 1;
  ret = display_->CreateSurface(surface_config, &surface_id_);
  if (ret != NO_ERROR) {
    ALOGE("%s: Create Surface error\n", __func__);
    display_->DestroyDisplay(display_type);
    display_->Disconnect();
    delete display_;
    display_ = NULL;
    return ret;
  }

  surface_param_.src_rect = { 0.0, 0.0, (float)track_parms.width, (float)track_parms.height };
  surface_param_.dst_rect = { 0.0, 0.0, (float)track_parms.width, (float)track_parms.height };
  surface_param_.surface_blending = SurfaceBlending::kBlendingCoverage;
  surface_param_.surface_flags.cursor = 0;
  surface_param_.frame_rate = track_parms.framerate;
  surface_param_.z_order = 0;
  surface_param_.solid_fill_color = 0;
  surface_param_.surface_transform.rotation = 0.0f;
  surface_param_.surface_transform.flip_horizontal = 0;
  surface_param_.surface_transform.flip_vertical = 0;
  display_started_ = 1;

  return NO_ERROR;
}

int32_t HTTPInterface::RemoveVidDisplayLocked(DisplayType display_type) {
  if (display_ == NULL) {
    ALOGE("%s: display_ is NULL", __func__);
    return NO_INIT;
  }
  display_->DestroySurface(surface_id_);
  display_->DestroyDisplay(display_type);
  display_->Disconnect();

  ALOGD("%s: DELETE display_ : %p\n", __func__, display_);
  delete display_;
  display_ = NULL;
  display_started_ = 0;
  return NO_ERROR;
}

void HTTPInterface::PushFrameToDisplay(BufferDescriptor& buffer,
                                       CameraBufferMetaData& meta_data) {
  surface_buffer_.plane_info[0].ion_fd = buffer.fd;
  surface_buffer_.buf_id = 0;
  surface_buffer_.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
  surface_buffer_.plane_info[0].stride = meta_data.plane_info[0].stride;
  surface_buffer_.plane_info[0].size = buffer.size;
  surface_buffer_.plane_info[0].width = meta_data.plane_info[0].width;
  surface_buffer_.plane_info[0].height = meta_data.plane_info[0].height;
  surface_buffer_.plane_info[0].offset = 0;
  surface_buffer_.plane_info[0].buf = buffer.data;

  auto ret = display_->QueueSurfaceBuffer(surface_id_,
                    surface_buffer_, surface_param_);
  if (ret != 0) {
    ALOGE("%s QueueSurfaceBuffer Failed!!", __func__);
    return;
  }

  ret = display_->DequeueSurfaceBuffer(surface_id_, surface_buffer_);
  if (ret != 0) {
    ALOGE("%s DequeueSurfaceBuffer Failed!!", __func__);
  }
}

int32_t HTTPInterface::AddRTSPVideoLocked(uint32_t session_id,
                                          uint32_t track_id,
                                          const VideoTrackCreateParam &video,
                                          qmmf_video_track_output output) {
  RTSPContext rtsp_ctx;
  ssize_t idx = rtsp_servers_.indexOfKey(session_id);
  if (NAME_NOT_FOUND != idx) {
    rtsp_ctx = rtsp_servers_.valueAt(idx);
    if (nullptr != rtsp_ctx.rtsp_video_queue) {
      ALOGE("%s: Video stream already present in the RTSP session!\n",
            __func__);
      return ALREADY_EXISTS;
    }
  } else {
    memset(&rtsp_ctx, 0, sizeof(rtsp_ctx));
  }

  rtsp_ctx.rtsp_port = DEFAULT_PORT + track_id;
  rtsp_ctx.is_mp2ts = (output == TRACK_OUTPUT_MPEGTS) ? true : false;

  switch (video.format_type) {
    case VideoFormat::kAVC:
      rtsp_ctx.video_codec_id = VIDEO_FORMAT_H264;
      rtsp_ctx.video_track_id = track_id;
      RtspServerInterface::QueueInit(&rtsp_ctx.rtsp_video_queue);
      break;
    case VideoFormat::kHEVC:
      rtsp_ctx.video_codec_id = VIDEO_FORMAT_H265;
      rtsp_ctx.video_track_id = track_id;
      RtspServerInterface::QueueInit(&rtsp_ctx.rtsp_video_queue);
      break;
    case VideoFormat::kYUV:
      if (rtsp_ctx.is_mp2ts) {
        ALOGE("%s: Mpeg2TS dosn't support YUV tracks!\n", __func__);
        return INVALID_OPERATION;
      }
      rtsp_ctx.meta_track_id = track_id;
      rtsp_ctx.video_codec_id = VIDEO_FORMAT_YUV;
      RtspServerInterface::QueueInit(&rtsp_ctx.rtsp_meta_queue);
      break;
    case VideoFormat::kBayerRDI10BIT:
    case VideoFormat::kBayerRDI12BIT:
    case VideoFormat::kBayerIdeal:
    default:
      ALOGE("%s: Unsupported codec: %u\n", __func__,
          static_cast<uint32_t>(video.format_type));
      return INVALID_OPERATION;
  }
  rtsp_ctx.frame_rate = video.frame_rate;


  if (NAME_NOT_FOUND != idx) {
    rtsp_servers_.replaceValueAt(idx, rtsp_ctx);
  } else {
    rtsp_servers_.add(session_id, rtsp_ctx);
  }

  return NO_ERROR;
}

int32_t HTTPInterface::RemoveRTSPVideoLocked(uint32_t session_id) {
  RTSPContext rtsp_ctx;
  ssize_t idx = rtsp_servers_.indexOfKey(session_id);
  if (NAME_NOT_FOUND != idx) {
    rtsp_ctx = rtsp_servers_.valueAt(idx);
    if ((nullptr == rtsp_ctx.rtsp_video_queue) &&
        (nullptr == rtsp_ctx.rtsp_meta_queue)) {
      ALOGE("%s: Video stream not present in RTSP session!\n",
            __func__);
      return NO_INIT;
    }

    if (nullptr != rtsp_ctx.rtsp_video_queue) {
      RtspServerInterface::QueueDInit(&rtsp_ctx.rtsp_video_queue);
      rtsp_ctx.rtsp_video_queue = nullptr;
    }

    if (nullptr != rtsp_ctx.rtsp_meta_queue) {
      RtspServerInterface::QueueDInit(&rtsp_ctx.rtsp_meta_queue);
      rtsp_ctx.rtsp_meta_queue = nullptr;
    }

    rtsp_ctx.video_codec_id = ES_FORMAT_MAX;
    rtsp_ctx.is_mp2ts = false;
    rtsp_ctx.rtsp_port = 0;
    rtsp_ctx.video_track_id = 0;
    rtsp_ctx.meta_track_id = 0;
    if (nullptr != rtsp_ctx.rtsp_url) {
      free(rtsp_ctx.rtsp_url);
      rtsp_ctx.rtsp_url = nullptr;
    }

    rtsp_servers_.replaceValueAt(idx, rtsp_ctx);
  } else {
    ALOGE("%s: No RTSP session found!\n", __func__);
    return BAD_VALUE;
  }

  return NO_ERROR;
}

int32_t HTTPInterface::RemoveRTSPAudioLocked(uint32_t session_id) {
  RTSPContext rtsp_ctx;
  ssize_t idx = rtsp_servers_.indexOfKey(session_id);
  if (NAME_NOT_FOUND != idx) {
    rtsp_ctx = rtsp_servers_.valueAt(idx);
    if (nullptr == rtsp_ctx.rtsp_audio_queue) {
      ALOGE("%s: Audio stream not present in RTSP session!\n",
            __func__);
      return NO_INIT;
    }

    RtspServerInterface::QueueDInit(&rtsp_ctx.rtsp_audio_queue);
    rtsp_ctx.rtsp_audio_queue = nullptr;
    rtsp_ctx.audio_codec_id = ES_FORMAT_MAX;
    rtsp_ctx.audio_idx = FS_IDX_MAX;
    rtsp_ctx.audio_channels = CH_CFG_MAX;
    rtsp_ctx.audio_profile = PROFILE_MAX;

    rtsp_servers_.replaceValueAt(idx, rtsp_ctx);
  } else {
    ALOGE("%s: No RTSP session found!\n", __func__);
    return BAD_VALUE;
  }

  return NO_ERROR;
}

FS_IDX_et HTTPInterface::FindAudioSampleIndex(size_t audio_rate)
{
  FS_IDX_et ret = FS_IDX_MAX;
  size_t idxCount = TABLE_SIZE(kAudioSamplingIndices);
  size_t i = 0;
  for (; i < idxCount; i++) {
    if (kAudioSamplingIndices[i].sampling_rate == audio_rate) {
      ret = kAudioSamplingIndices[i].fs_Idx;
      break;
    }
  }

  return ret;
}

int32_t HTTPInterface::AddRTSPAudioLocked(uint32_t session_id,
                                          const qmmf_audio_track_param &audio) {
  RTSPContext rtsp_ctx;
  ssize_t idx = rtsp_servers_.indexOfKey(session_id);
  if (NAME_NOT_FOUND != idx) {
    rtsp_ctx = rtsp_servers_.valueAt(idx);
    if (nullptr != rtsp_ctx.rtsp_audio_queue) {
      ALOGE("%s: Audio stream already present in the RTSP session!\n",
            __func__);
      return ALREADY_EXISTS;
    }
  } else {
    memset(&rtsp_ctx, 0, sizeof(rtsp_ctx));
  }

  switch (audio.codec) {
    case CODEC_AAC:
      rtsp_ctx.audio_codec_id = AUDIO_FORMAT_ADTS;
      rtsp_ctx.audio_profile = PROFILE_1; //TODO: map audio codec profile
      break;
    case CODEC_AMR:
    case CODEC_AMRWB:
    case CODEC_PCM:
    default:
      ALOGE("%s: Audio codec %d is currently not supported for RTSP!",
            __func__, audio.codec);
      return BAD_VALUE;
  }

  FS_IDX_et audio_idx = FindAudioSampleIndex(audio.sample_rate);
  if (FS_IDX_MAX == audio_idx) {
      ALOGE("%s: Not able to find matching audio sample rate index!",
              __func__);
      return BAD_VALUE;
  }
  rtsp_ctx.audio_idx = audio_idx;
  rtsp_ctx.audio_channels = (CH_CFG_et) audio.num_channels;

  RtspServerInterface::QueueInit(&rtsp_ctx.rtsp_audio_queue);
  rtsp_ctx.is_mp2ts = (audio.output == AUDIO_TRACK_OUTPUT_MPEGTS) ?
      true : false;

  if (NAME_NOT_FOUND != idx) {
    rtsp_servers_.replaceValueAt(idx, rtsp_ctx);
  } else {
    rtsp_servers_.add(session_id, rtsp_ctx);
  }

  return NO_ERROR;
}

int32_t HTTPInterface::AddVidMuxParmsLocked(uint32_t session_id,
                                            uint32_t track_id,
                                            const VideoTrackCreateParam &video,
                                            qmmf_video_track_output output) {
  qmmf_muxer_init params;
  MUX_brand_type brand;
  int32_t ret;
  uint8_t codec_level, codec_profile;
  MUX_stream_video_type format;

  switch (video.format_type) {
    case VideoFormat::kAVC:
      ret = getAVCProfileLevel(video, codec_level, codec_profile);
      if (NO_ERROR != ret) {
        ALOGE("%s: Failed to during codec profile/level query!: %d\n",
              __func__, ret);
        return ret;
      }
      format = MUX_STREAM_VIDEO_H264;
      break;
    case VideoFormat::kHEVC:
      ret = getHEVCProfileLevel(video, codec_level, codec_profile);
      if (NO_ERROR != ret) {
        ALOGE("%s: Failed to during codec profile/level query!: %d\n",
              __func__, ret);
        return ret;
      }
      format = MUX_STREAM_VIDEO_HEVC;
      break;
    default:
      ALOGE("%s: Unsupported video format: %d\n", __func__, video.format_type);
      return BAD_VALUE;
  }

  switch(output) {
    case TRACK_OUTPUT_MP4:
      brand = MUX_BRAND_MP4;
      break;
    case TRACK_OUTPUT_3GP:
      brand = MUX_BRAND_3GP;
      break;
    case TRACK_OUTPUT_MPEGTS:
    case TRACK_OUTPUT_RTSP:
    case TRACK_OUTPUT_VAM:
    case TRACK_OUTPUT_EVENT_LOGGER:
    default:
      ALOGE("%s: Unsupported output:%d\n", __func__, output);
      return BAD_VALUE;
  }

  ssize_t param_idx = muxer_params_.indexOfKey(session_id);
  if (NAME_NOT_FOUND != param_idx) {
    params = muxer_params_.valueAt(param_idx);
    if (params.video_stream_present) {
      ALOGE("%s: Video stream already present in the session muxer!\n",
            __func__);
      return ALREADY_EXISTS;
    }
    if ((brand != params.brand) && (MUX_BRAND_INVALID != params.brand)) {
      ALOGE("%s: Muxer cannot support two different brands: %d vs. %d\n",
            __func__, brand, params.brand);
      return BAD_VALUE;
    }
  } else {
    memset(&params, 0, sizeof(params));
  }

  params.brand = brand;
  params.video_stream.codec_profile = codec_profile;
  params.video_stream.codec_level = codec_level;
  params.video_stream.format = format;
  params.video_stream_present = true;
  params.video_stream.width = video.width;
  params.video_stream.height = video.height;
  params.video_stream.bitrate = video.codec_param.avc.bitrate;
  params.video_stream.framerate = video.frame_rate;
  params.video_stream.track_id = track_id;

  if (NAME_NOT_FOUND != param_idx) {
    muxer_params_.replaceValueAt(param_idx, params);
  } else {
    muxer_params_.add(session_id, params);
  }

  return NO_ERROR;
}

int32_t HTTPInterface::RemoveVidMuxParmsLocked(uint32_t session_id) {
  qmmf_muxer_init params;
  ssize_t param_idx = muxer_params_.indexOfKey(session_id);
  if (NAME_NOT_FOUND != param_idx) {
    params = muxer_params_.valueAt(param_idx);
    if (!params.video_stream_present) {
      ALOGE("%s: Video stream not present in session muxer!\n",
            __func__);
      return NO_INIT;
    }

    params.video_stream_present = false;
    if (!params.audio_stream_present) {
      params.brand = MUX_BRAND_INVALID;
    }
    muxer_params_.replaceValueAt(param_idx, params);
  } else {
    ALOGE("%s: No muxer found for this stream!\n", __func__);
    return BAD_VALUE;
  }

  return NO_ERROR;
}

int32_t HTTPInterface::InitMuxerLocked(uint32_t session_id,
                                       qmmf_muxer_init &init_params) {
  // Add by TS
  uint32_t video_id = 0;
  ifstream in_file;
  in_file.open(videoIdFileName);
  if (in_file) {
    in_file >> video_id;
  }
  in_file.close();

  String8 file;
  MUX_brand_type brand = init_params.brand;
  switch(brand) {
    case MUX_BRAND_MP4:
      file.appendFormat(kMuxedFileName, video_id, "mp4");
      break;
    case MUX_BRAND_3GP:
      file.appendFormat(kMuxedFileName, video_id, "3gp");
      break;
    case MUX_BRAND_MP2TS:
      file.appendFormat(kMuxedFileName, video_id, "ts");
      break;
    default:
      ALOGE("%s: Unsupported brand:%d\n", __func__, brand);
      return BAD_VALUE;
  }

  init_params.release_cb = [&] (uint32_t track_id, uint32_t session_id,
                                BufferDescriptor &buffer) {
    ReturnTrackBuffer(track_id, session_id, buffer);
  };

  MuxInterface *mux = new MuxInterface(brand, file.string());
  if (nullptr == mux) {
    ALOGE("%s: Unable to allocate muxer!\n", __func__);
    return NO_MEMORY;
  }

  auto ret = mux->Init(init_params);
  if (NO_ERROR == ret) {
    muxers_.add(session_id, mux);
    video_id++;
  } else {
    ALOGE("%s: Unable to initialize muxer!\n", __func__);
    delete mux;
  }

  ofstream out_file;
  out_file.open(videoIdFileName);
  if (out_file) {
    out_file << video_id;
  }
  out_file.close();

  return ret;
}

int32_t HTTPInterface::QueueMuxBuffersLocked(uint32_t track_id, uint32_t session_id,
                                             std::vector<BufferDescriptor> &buffers,
                                             std::vector<MetaData> &meta_data) {
  int32 ret = NO_ERROR;

  if (!muxers_.isEmpty()) {
    ssize_t idx = muxers_.indexOfKey(session_id);
    if (NAME_NOT_FOUND != idx) {
      MuxInterface *mux = muxers_.valueAt(idx);
      size_t buffer_index = 0;
      for (auto& iter : buffers) {
        ret = mux->WriteBuffer(track_id, session_id, (iter),
                               meta_data[buffer_index]);
        if (DEAD_OBJECT == ret) {
          break; //Muxer stopped
        } else if (NO_ERROR != ret) {
          ALOGE("%s: Muxer write failed: %d\n", __func__, ret);
          break;
        }
        ++buffer_index;
      }
    } else {
      ALOGE("%s: Muxer for track id: %d not found!\n", __func__, track_id);
      return NO_INIT;
    }
  } else {
    ALOGE("%s: No active muxers!\n", __func__);
    return NO_INIT;
  }

  return ret;
}

int32_t HTTPInterface::QueueRTSPBuffersLocked(uint32_t session_id,
                                              std::vector<BufferDescriptor> &buffers,
                                              RTSPInput input) {
  if (!rtsp_servers_.isEmpty()) {
    ssize_t idx = rtsp_servers_.indexOfKey(session_id);
    if (NAME_NOT_FOUND != idx) {
      RTSPContext rtsp_ctx = rtsp_servers_.valueAt(idx);

      for (auto& iter : buffers) {
        switch (input) {
          case RTSP_VIDEO:
          {
            const char *codec = (VIDEO_FORMAT_H264 == rtsp_ctx.video_codec_id)?
                "AVC" : "HEVC";
            RtspServerInterface::QuePushData(codec,
                                             (uint8_t *) iter.data,
                                             iter.size,
                                             iter.timestamp,
                                             rtsp_ctx.rtsp_video_queue);
          }
           break;
          case RTSP_META:
            RtspServerInterface::QuePushData("META",
                                             (uint8_t *) iter.data,
                                             iter.size,
                                             iter.timestamp,
                                             rtsp_ctx.rtsp_meta_queue);
            break;
          case RTSP_AUDIO:
            RtspServerInterface::QuePushData("AAC",
                                             (uint8_t *) iter.data,
                                             iter.size,
                                             iter.timestamp,
                                             rtsp_ctx.rtsp_audio_queue);
           break;
          default:
            ALOGE("%s: Unsupported rtsp input: %d\n", __func__, input);
            return BAD_VALUE;
        }
      }
    } else {
      ALOGE("%s: Session id: %d not found in RTSP server map!\n",
            __func__, session_id);
      return NO_INIT;
    }
  } else {
    ALOGE("%s: No active RTSP streams!", __func__);
    return NO_INIT;
  }

  return NO_ERROR;
}

int32_t HTTPInterface::StartVAMVideoLogLocked(const qmmf_video_track_param &param) {
  int32_t ret = NO_ERROR;
  bool bStopSession = false;
  if (!vam_video_log_params_.session_id) {
    ret = CreateSession(&vam_video_log_params_.session_id);
    if (ret) {
      memset(&vam_video_log_params_, 0, sizeof(vam_video_log_params_));
      ALOGE("%s: Failed to create session for event logging: %d", __func__, ret);
      goto exit;
    }

    vam_video_log_params_.bitrate         = param.bitrate;
    vam_video_log_params_.camera_id       = param.camera_id;
    vam_video_log_params_.framerate       = param.framerate;
    vam_video_log_params_.width           = param.width;
    vam_video_log_params_.height          = param.height;
    vam_video_log_params_.low_power_mode  = param.low_power_mode;
    vam_video_log_params_.track_id        = vam_video_log_params_.session_id;
    vam_video_log_params_.codec           = CODEC_AVC;
    vam_video_log_params_.output          = TRACK_OUTPUT_EVENT_LOGGER;

    lock_.unlock();
    ret = CreateVideoTrack(vam_video_log_params_);
    lock_.lock();
    if (ret) {
      ALOGE("%s: Failed to create video track for event logging: %d", __func__, ret);
      goto exit;
    }

    ret = StartSession(vam_video_log_params_.session_id);
    if (ret) {
      memset(&vam_video_log_params_, 0, sizeof(vam_video_log_params_));
      ALOGE("%s: Failed to start session for event logging: %d\n", __func__, ret);
      goto exit;
    }

    ret = vam_interface_->InitVAMVLog(vam_video_log_params_.framerate);
    if (ret) {
      ALOGE("%s: Failed to initialize VAM video logging: %d\n", __func__, ret);
      bStopSession = true;
      goto exit;
    }
  }

  return ret;

  exit:
  if (bStopSession) {
    StopSession(vam_video_log_params_.session_id, true);
  }
  if (vam_video_log_params_.track_id) {
    DeleteVideoTrack(vam_video_log_params_.session_id,
                     vam_video_log_params_.track_id);
  }
  if (vam_video_log_params_.track_id) {
    DeleteSession(vam_video_log_params_.session_id);
  }
  memset(&vam_video_log_params_, 0, sizeof(vam_video_log_params_));

  return ret;
}

int32_t HTTPInterface::SendVAMMeta(uint32_t session_id, uint32_t track_id,
                                   const char *metaString, size_t size,
                                   int64_t pts) {
  int32_t ret;
  Mutex::Autolock l(lock_);
  if (0 < track_id) {
    std::vector<BufferDescriptor> buffers;
    BufferDescriptor meta_buffer;
    memset(&meta_buffer, 0, sizeof(meta_buffer));
    meta_buffer.data = const_cast<char *>(metaString);
    meta_buffer.size = size;
    meta_buffer.timestamp = ns2us(pts);
    buffers.push_back(meta_buffer);
    ret = QueueRTSPBuffersLocked(session_id, buffers, RTSP_META);
    if (NO_ERROR != ret) {
      ALOGE("%s: Unable to queue buffers in RTSP server: %d!", __func__,
            ret);
    }
  } else {
    ALOGE("%s: VAM track id is invalid!\n", __func__);
    ret = NO_INIT;
  }

  return ret;
}

int32_t HTTPInterface::SetCameraParams(qmmf_camera_parameters params) {
  Mutex::Autolock l(lock_);

  ssize_t idx = camera_configs_.indexOfKey(params.camera_id);
  if (NAME_NOT_FOUND == idx) {
    ALOGE("%s: No camera configuration present for camera: %d\n", __func__,
          params.camera_id);
    return NO_INIT;
  }
  CameraConfiguration *config = camera_configs_.valueAt(idx);

  CameraMetadata meta;
  auto ret = recorder_.GetCameraParam(params.camera_id, meta);
  if (NO_ERROR != ret) {
    ALOGE("%s: Failed to query camera parameters!\n", __func__);
    return ret;
  }

  if (params.nr_mode_set) {
    ret = config->SetNRMode(params.nr_mode, meta);
    if (NO_ERROR != ret) {
      ALOGE("%s: Failed to update NR camera parameters!\n", __func__);
      return ret;
    }
  }

  if (params.hdr_mode_set) {
    ret = config->SetHDRMode(params.hdr_mode, meta);
    if (NO_ERROR != ret) {
      ALOGE("%s: Failed to update HDR camera parameters!\n", __func__);
      return ret;
    }
  }

  if (params.ir_mode_set) {
    ret = config->SetIRMode(params.ir_mode, meta);
    if (NO_ERROR != ret) {
      ALOGE("%s: Failed to update IR camera parameters!\n", __func__);
      return ret;
    }
  }

  ret = recorder_.SetCameraParam(params.camera_id, meta);
  if (NO_ERROR != ret) {
    ALOGE("%s: Failed to set camera parameters!\n", __func__);
    return ret;
  }

  return ret;
}

int32_t HTTPInterface::CreateOverlay(uint32_t track_id, uint32_t *overlay_id,
                                     struct qmmf_overlay_param_t *ov_params) {
  if ((nullptr == ov_params) || (nullptr == overlay_id)) {
      return BAD_VALUE;
  }

  OverlayParam params;
  memset(&params, 0, sizeof(params));
  auto ret = convertOvParams2QMMF(*ov_params, params);
  if (NO_ERROR != ret) {
    ALOGE("%s: Error converting qmmf overlay parameters: %d",
          __func__, ret);
    return ret;
  }

  ret = recorder_.CreateOverlayObject(track_id, params, overlay_id);
  if (NO_ERROR != ret) {
    ALOGE("%s: Overlay create failed: %d", __func__, ret);
  }

  return ret;
}

int32_t HTTPInterface::DeleteOverlay(uint32_t track_id, uint32_t overlay_id) {
  return recorder_.DeleteOverlayObject(track_id, overlay_id);
}

int32_t HTTPInterface::SetOverlay(uint32_t track_id, uint32_t overlay_id) {
  return recorder_.SetOverlay(track_id, overlay_id);
}

int32_t HTTPInterface::RemoveOverlay(uint32_t track_id, uint32_t overlay_id) {
  return recorder_.RemoveOverlay(track_id, overlay_id);
}

int32_t HTTPInterface::GetOverlay(uint32_t track_id, uint32_t overlay_id,
                                  struct qmmf_overlay_param_t *ov_params) {
  if (nullptr == ov_params) {
    return BAD_VALUE;
  }

  OverlayParam params;
  memset(&params, 0, sizeof(params));
  auto ret = recorder_.GetOverlayObjectParams(track_id, overlay_id, params);
  if (NO_ERROR != ret) {
    ALOGE("%s: Overlay query failed: %d", __func__, ret);
    return ret;
  }

  memset(ov_params, 0, sizeof(qmmf_overlay_param_t));
  ret = convertQMMF2OvParams(*ov_params, params);
  if (NO_ERROR != ret) {
    ALOGE("%s: Error converting overlay parameters: %d",
          __func__, ret);
  }

  return ret;
}

int32_t HTTPInterface::UpdateOverlay(uint32_t track_id, uint32_t overlay_id,
                                     struct qmmf_overlay_param_t *ov_params) {
  if (nullptr == ov_params) {
    return BAD_VALUE;
  }

  OverlayParam params;
  memset(&params, 0, sizeof(params));
  auto ret = convertOvParams2QMMF(*ov_params, params);
  if (NO_ERROR != ret) {
    ALOGE("%s: Error converting qmmf overlay parameters: %d",
          __func__, ret);
    return ret;
  }

  ret = recorder_.UpdateOverlayObjectParams(track_id, overlay_id, params);
  if (NO_ERROR != ret) {
    ALOGE("%s: Overlay update failed: %d", __func__, ret);
  }

  return ret;
}

template <typename entryType, class qmmfType, class entry> int32_t LookupQMMFValue(
    entryType *table, size_t entryCount, entry val, qmmfType &found) {
  int32_t ret = NAME_NOT_FOUND;
  for (size_t i = 0; i < entryCount; i++) {
    if (table[i].entry == val) {
      found = table[i].qmmf_entry;
      ret = NO_ERROR;
      break;
    }
  }

  return ret;
}

template <typename entryType, class qmmfType, class entry> int32_t LookupValue(
    entryType *table, size_t entryCount, qmmfType val, entry &found) {
  int32_t ret = NAME_NOT_FOUND;
  for (size_t i = 0; i < entryCount; i++) {
    if (table[i].qmmf_entry == val) {
      found = table[i].entry;
      ret = NO_ERROR;
      break;
    }
  }

  return ret;
}

int32_t HTTPInterface::CreateMultiCamera(const uint32_t *camera_ids,
                                         uint32_t num_camera,
                                         uint32_t *virtual_camera_id) {
  if (num_camera <= 1) {
    ALOGE("%s: Invalid num camera(%d)", __func__, num_camera);
    return BAD_VALUE;
  }
  std::vector<uint32_t> camera_id_vec;
  for (uint32_t i = 0; i < num_camera; ++i) {
    camera_id_vec.push_back(camera_ids[i]);
  }

  auto ret = recorder_.CreateMultiCamera(camera_id_vec, virtual_camera_id);
  ALOGD("%s: virtual camera id=%d",__func__, *virtual_camera_id);
  return ret;
}

int32_t HTTPInterface::ConfigureMultiCamera(qmmf_multi_camera_param_t *params) {

  MultiCameraConfigType type = static_cast<MultiCameraConfigType>(params->type);
  auto ret = recorder_.ConfigureMultiCamera(params->virtual_camera_id,
                                            type,
                                            params->param, params->param_size);
  if (NO_ERROR != ret) {
    ALOGE("%s: ConfigureMultiCamera failed!", __func__);
  }
  return ret;
}

int32_t HTTPInterface::convertOvParams2QMMF(qmmf_overlay_param &ovParams,
                                            OverlayParam &params) {
  auto ret = LookupQMMFValue(kOverlayTypeTable,
                             TABLE_SIZE(kOverlayTypeTable),
                             ovParams.ov_type, params.type);
  if (NO_ERROR != ret) {
    return ret;
  }

  switch(params.type) {
    case OverlayType::kDateType:
      ret = LookupQMMFValue(kOverlayPositionTable,
                            TABLE_SIZE(kOverlayPositionTable),
                            ovParams.position, params.location);
      if (NO_ERROR != ret) {
        return ret;
      }

      params.color = ovParams.color;
      params.dst_rect.start_x = ovParams.start_x;
      params.dst_rect.start_y = ovParams.start_y;
      params.dst_rect.width = ovParams.width;
      params.dst_rect.height = ovParams.height;
      ret = LookupQMMFValue(kOverlayDateTable,
                            TABLE_SIZE(kOverlayDateTable),
                            ovParams.date,
                            params.date_time.date_format);
      if (NO_ERROR != ret) {
        return ret;
      }

      ret = LookupQMMFValue(kOverlayTimeTable,
                            TABLE_SIZE(kOverlayTimeTable),
                            ovParams.time,
                            params.date_time.time_format);
      if (NO_ERROR != ret) {
        return ret;
      }
      break;
    case OverlayType::kUserText:
      ret = LookupQMMFValue(kOverlayPositionTable,
                            TABLE_SIZE(kOverlayPositionTable),
                            ovParams.position, params.location);
      if (NO_ERROR != ret) {
        return ret;
      }

      params.color = ovParams.color;
      params.dst_rect.start_x = ovParams.start_x;
      params.dst_rect.start_y = ovParams.start_y;
      params.dst_rect.width = ovParams.width;
      params.dst_rect.height = ovParams.height;
      memset(params.user_text, '\0', sizeof(params.user_text));
      strncpy(params.user_text, ovParams.user_text,
              sizeof(params.user_text)-1);
      break;
    case OverlayType::kStaticImage:
      ret = LookupQMMFValue(kOverlayPositionTable,
                            TABLE_SIZE(kOverlayPositionTable),
                            ovParams.position, params.location);
      if (NO_ERROR != ret) {
        return ret;
      }

      params.dst_rect.start_x = ovParams.start_x;
      params.dst_rect.start_y = ovParams.start_y;
      params.dst_rect.width = ovParams.width;
      params.dst_rect.height = ovParams.height;
      ret = LookupQMMFValue(kOverlayStaticImageTable,
                            TABLE_SIZE(kOverlayStaticImageTable),
                            ovParams.image_type, params.image_info.image_type);
      if (NO_ERROR != ret) {
        return ret;
      }
      memset(params.image_info.image_location, '\0',
             sizeof(params.image_info.image_location));
      strncpy(params.image_info.image_location, ovParams.image_location,
              sizeof(params.image_info.image_location) -1);

      if (ovParams.image_type == BLOBTYPE) {
        params.image_info.image_size = ovParams.image_size;
        params.image_info.image_buffer =
            (char *) malloc(sizeof(ovParams.image_size));
        if (params.image_info.image_buffer) {
          memcpy(params.image_info.image_buffer, ovParams.image_buffer,
                 ovParams.image_size);
        } else {
          ALOGE("%s: No memory for overlay image buffer!", __func__);
          return NO_MEMORY;
        }
        params.image_info.buffer_updated = ovParams.image_buffer_updated;
      }
      break;
    case OverlayType::kBoundingBox:
      params.color = ovParams.color;
      params.dst_rect.start_x = ovParams.start_x;
      params.dst_rect.start_y = ovParams.start_y;
      params.dst_rect.width = ovParams.width;
      params.dst_rect.height = ovParams.height;
      memset(params.bounding_box.box_name, '\0',
             sizeof(params.bounding_box.box_name));
      strncpy(params.bounding_box.box_name, ovParams.box_name,
              sizeof(params.bounding_box.box_name)-1);
      break;
    case OverlayType::kPrivacyMask:
      params.color = ovParams.color;
      params.dst_rect.start_x = ovParams.start_x;
      params.dst_rect.start_y = ovParams.start_y;
      params.dst_rect.width = ovParams.width;
      params.dst_rect.height = ovParams.height;
      break;
    default:
      ALOGE("%s: Unsupported overlay type: %d", __func__, params.type);
      return BAD_VALUE;
  }

  return ret;
}

int32_t HTTPInterface::convertQMMF2OvParams(qmmf_overlay_param &ovParams,
                                            OverlayParam &params) {
  auto ret = LookupValue(kOverlayTypeTable,
                         TABLE_SIZE(kOverlayTypeTable),
                         params.type, ovParams.ov_type);
  if (NO_ERROR != ret) {
    return ret;
  }


  switch(params.type) {

    case OverlayType::kDateType:
      ret = LookupValue(kOverlayPositionTable,
                        TABLE_SIZE(kOverlayPositionTable),
                        params.location, ovParams.position);
      if (NO_ERROR != ret) {
        return ret;
      }

      ovParams.color = params.color;
      ovParams.start_x = params.dst_rect.start_x;
      ovParams.start_y = params.dst_rect.start_y;
      ovParams.width = params.dst_rect.width;
      ovParams.height = params.dst_rect.height;
      ret = LookupValue(kOverlayDateTable,
                        TABLE_SIZE(kOverlayDateTable),
                        params.date_time.date_format, ovParams.date);
      if (NO_ERROR != ret) {
        return ret;
      }

      ret = LookupValue(kOverlayTimeTable,
                        TABLE_SIZE(kOverlayTimeTable),
                        params.date_time.time_format, ovParams.time);
      if (NO_ERROR != ret) {
        return ret;
      }
      break;
    case OverlayType::kUserText:
      ret = LookupValue(kOverlayPositionTable,
                        TABLE_SIZE(kOverlayPositionTable),
                        params.location, ovParams.position);
      if (NO_ERROR != ret) {
        return ret;
      }

      ovParams.color = params.color;
      ovParams.start_x = params.dst_rect.start_x;
      ovParams.start_y = params.dst_rect.start_y;
      ovParams.width = params.dst_rect.width;
      ovParams.height = params.dst_rect.height;
      ovParams.user_text = (char *) malloc(sizeof(params.user_text));
      if (ovParams.user_text) {
        memset(ovParams.user_text, '\0', sizeof(params.user_text));
        strncpy(ovParams.user_text, params.user_text,
                sizeof(params.user_text) - 1);
      } else {
        ALOGE("%s: No memory for overlay user text!", __func__);
        return NO_MEMORY;
      }
      break;
    case OverlayType::kStaticImage:
      ret = LookupValue(kOverlayPositionTable,
                        TABLE_SIZE(kOverlayPositionTable),
                        params.location, ovParams.position);
      if (NO_ERROR != ret) {
        return ret;
      }

      ovParams.start_x = params.dst_rect.start_x;
      ovParams.start_y = params.dst_rect.start_y;
      ovParams.width = params.dst_rect.width;
      ovParams.height = params.dst_rect.height;
      ret = LookupValue(kOverlayStaticImageTable,
                        TABLE_SIZE(kOverlayStaticImageTable),
                        params.image_info.image_type, ovParams.image_type);
      if (NO_ERROR != ret) {
        return ret;
      }

      ovParams.image_location = (char *) malloc(
          sizeof(params.image_info.image_location));
      if (ovParams.image_location) {
        memset(ovParams.image_location, '\0',
               sizeof(params.image_info.image_location));
        strncpy(ovParams.image_location, params.image_info.image_location,
                sizeof(params.image_info.image_location) - 1);
      } else {
        ALOGE("%s: No memory for overlay image location!", __func__);
        return NO_MEMORY;
      }

      if (ovParams.image_type == BLOBTYPE) {
        ovParams.image_size = params.image_info.image_size;
        ovParams.image_buffer = (char *) malloc(
            sizeof(params.image_info.image_size));
        if (ovParams.image_buffer) {
          memcpy(ovParams.image_buffer, params.image_info.image_buffer,
                  params.image_info.image_size);
        } else {
          ALOGE("%s: No memory for overlay image buffer!", __func__);
          return NO_MEMORY;
        }
        ovParams.image_buffer_updated = params.image_info.buffer_updated;
      }
      break;
    case OverlayType::kBoundingBox:
      ovParams.color = params.color;
      ovParams.start_x = params.dst_rect.start_x;
      ovParams.start_y = params.dst_rect.start_y;
      ovParams.width = params.dst_rect.width;
      ovParams.height = params.dst_rect.height;
      ovParams.box_name = (char *) malloc(
          sizeof(params.bounding_box.box_name));
      if (ovParams.box_name) {
        memset(ovParams.box_name, '\0',
               sizeof(params.bounding_box.box_name));
        strncpy(ovParams.box_name, params.bounding_box.box_name,
                sizeof(params.bounding_box.box_name) - 1);
      } else {
        ALOGE("%s: No memory for bounding box name!", __func__);
        return NO_MEMORY;
      }
      break;
    case OverlayType::kPrivacyMask:
      ovParams.color = params.color;
      ovParams.start_x = params.dst_rect.start_x;
      ovParams.start_y = params.dst_rect.start_y;
      ovParams.width = params.dst_rect.width;
      ovParams.height = params.dst_rect.height;
      break;
    default:
      ALOGE("%s: Unsupported overlay type: %d", __func__, params.type);
      return BAD_VALUE;
  }

    return ret;
}

void HTTPInterface::AudioTrackCb(uint32_t track_id,
                                 std::vector<BufferDescriptor> buffers,
                                 __attribute__((unused)) std::vector<MetaData> meta_data) {
  uint32_t session_id = 0;
  bool return_buffer = true;
  {
    Mutex::Autolock l(lock_);
    ssize_t idx = session_map_.indexOfKey(track_id);
    if (NAME_NOT_FOUND == idx) {
      ALOGE("%s: Track id: %d not found in session map!\n", __func__, track_id);
      return;
    }
    session_id = session_map_.valueAt(idx);

    idx = audio_tracks_.indexOfKey(track_id);
    if (NAME_NOT_FOUND == idx) {
      ALOGE("%s: Track id: %d status not found!\n", __func__, track_id);
      return;
    }

    qmmf_audio_track_param status = audio_tracks_.valueAt(idx);
    switch(status.output) {
      case AUDIO_TRACK_OUTPUT_MPEGTS:
      {
        auto ret = QueueRTSPBuffersLocked(session_id, buffers, RTSP_AUDIO);
        if (NO_ERROR != ret) {
          ALOGE("%s: Unable to queue buffers in RTSP server: %d!", __func__,
                ret);
        }
        return_buffer = true;
      }
        break;
      case AUDIO_TRACK_OUTPUT_MP4:
      case AUDIO_TRACK_OUTPUT_3GP:
      {
        auto ret = QueueMuxBuffersLocked(track_id, session_id, buffers, meta_data);
        if (NO_ERROR == ret) {
          return_buffer = false;
        } else if (DEAD_OBJECT == ret) {
          return_buffer = true;
        } else {
          ALOGE("%s: Unable to queue buffers in muxer: %d!", __func__,
                ret);
        }
      }
        break;
      default:
        ALOGE("%s: Unsupported track output: %d\n", __func__,
              status.output);
    }
  }

  if (return_buffer) {
    auto ret = recorder_.ReturnTrackBuffer(session_id, track_id, buffers);
    if(ret != 0) {
      ALOGE("%s: ReturnTrackBuffer failed: %d!", __func__, ret);
    }
  }
}

void HTTPInterface::VideoTrackCb(uint32_t track_id,
                                 std::vector<BufferDescriptor> buffers,
                                 std::vector<MetaData> meta_data) {
  uint32_t session_id = 0;
  bool return_buffer = true;

  {
    Mutex::Autolock l(lock_);
    ssize_t idx = session_map_.indexOfKey(track_id);
    if (NAME_NOT_FOUND == idx) {
      ALOGE("%s: Track id: %d not found in session map!\n", __func__, track_id);
      return;
    }
    session_id = session_map_.valueAt(idx);

    idx = video_tracks_.indexOfKey(track_id);
    if (NAME_NOT_FOUND == idx) {
      ALOGE("%s: Track id: %d status not found!\n", __func__, track_id);
      return;
    }

    qmmf_video_track_status status = video_tracks_.valueAt(idx);
    switch(status.output) {
      case TRACK_OUTPUT_RTSP:
      case TRACK_OUTPUT_MPEGTS:
      {
        auto ret = QueueRTSPBuffersLocked(session_id, buffers, RTSP_VIDEO);
        if (NO_ERROR != ret) {
          ALOGE("%s: Unable to queue buffers in RTSP server: %d!", __func__,
                ret);
        }
      }
        break;
      case TRACK_OUTPUT_VAM:
      {
        auto ret = vam_interface_->QueueVAMBuffers(track_id, session_id,
                                                   buffers, meta_data);
        if (NO_ERROR == ret) {
          return_buffer = false;
        }
      }
        break;
      case TRACK_OUTPUT_MP4:
      case TRACK_OUTPUT_3GP:
      {
        auto ret = QueueMuxBuffersLocked(track_id, session_id, buffers, meta_data);
        if (NO_ERROR == ret) {
          return_buffer = false;
        } else if (DEAD_OBJECT == ret) {
          return_buffer = true;
        } else {
          ALOGE("%s: Unable to queue buffers in muxer: %d!", __func__,
                ret);
        }
      }
        break;
      case TRACK_OUT_DSI:
         if (!display_started_)
           break;
         for (uint32_t i = 0; i < meta_data.size(); i++) {
           MetaData meta_data_dsi = meta_data[i];
           if (meta_data_dsi.meta_flag &
                  static_cast<uint32_t>(MetaParamType::kCamBufMetaData)) {
             CameraBufferMetaData cam_buf_meta =
                  meta_data_dsi.cam_buffer_meta_data;
             PushFrameToDisplay(buffers[i], cam_buf_meta);
           }
         }
       break;
      case TRACK_OUTPUT_EVENT_LOGGER:
      {
        vam_interface_->ProcessVLogBuffers(buffers);
        return_buffer = true;
      }
        break;
      default:
        ALOGE("%s: Unsupported track output: %d\n", __func__,
              status.output);
    }
  }

  if (return_buffer) {
    auto ret = recorder_.ReturnTrackBuffer(session_id, track_id, buffers);
    if(ret != 0) {
      ALOGE("%s: ReturnTrackBuffer failed: %d!", __func__, ret);
    }
  }
}

int32_t HTTPInterface::ReturnTrackBuffer(uint32_t track_id,
                                         uint32_t session_id,
                                         BufferDescriptor &buffer) {
  std::vector<BufferDescriptor> buffers;
  buffers.push_back(buffer);

  auto ret = recorder_.ReturnTrackBuffer(session_id, track_id, buffers);
  if(ret != 0) {
    ALOGE("%s: ReturnTrackBuffer failed: %d!", __func__, ret);
  }
  return NO_ERROR;
}

void HTTPInterface::SnapshotCb(uint32_t camera_id,
                               __attribute__((unused)) uint32_t image_sequence_count,
                               BufferDescriptor buffer,
                               __attribute__((unused)) MetaData meta_data) {
#if STORE_SNAPSHOT
  String8 file_path;
  size_t written_len;
  // Add by TS
  uint32_t image_id = 0;
  ifstream in_file;
  in_file.open(imageIdFileName);
  if (in_file) {
    in_file >> image_id;
  }
  in_file.close();

  file_path.appendFormat("/data/misc/qmmf/qmmf_snapshot_%u.jpg", image_id);

  FILE *file = fopen(file_path.string(), "w+");
  if (!file) {
    ALOGE("%s: Unable to open file(%s)", __func__,
        file_path.string());
    goto FAIL;
  }
  image_id++;
  written_len = fwrite(buffer.data, sizeof(uint8_t), buffer.size, file);
  if (buffer.size != written_len) {
    ALOGE("%s: Bad Write error (%d):(%s)\n", __func__, errno,
          strerror(errno));
    goto FAIL;
  }

FAIL:
  if (file != nullptr) {
    fclose(file);
  }

  ofstream out_file;
  out_file.open(imageIdFileName);
  if (out_file) {
    out_file << image_id;
  }
  out_file.close();
#endif

  snapshot_result_.snapshot_buffer = (uint8_t *) malloc(buffer.size);
  if (nullptr == snapshot_result_.snapshot_buffer) {
    ALOGE("%s: Unable to allocate snapshot buffer!\n", __func__);
    goto exit;
  }

  memcpy(snapshot_result_.snapshot_buffer, buffer.data, buffer.size);
  snapshot_result_.timestamp = buffer.timestamp;
  snapshot_result_.snapshot_size = buffer.size;

exit:

  pthread_mutex_lock(&snapshot_lock_);
  snapshot_completed_ = true;
  aec_converged_ = false;
  pthread_cond_signal(&snapshot_cond_);
  pthread_mutex_unlock(&snapshot_lock_);

  recorder_.ReturnImageCaptureBuffer(camera_id, buffer);
}

void HTTPInterface::CameraCb(uint32_t camera_id,
              const CameraMetadata &result) {
  if (aec_converged_ == false) {
    camera_metadata_ro_entry aec_stat;
    aec_stat = result.find(ANDROID_CONTROL_AE_STATE);
    if ((aec_stat.count > 0) && ((aec_stat.data.u8[0]
        == ANDROID_CONTROL_AE_STATE_CONVERGED) || (aec_stat.data.u8[0]
        == ANDROID_CONTROL_AE_STATE_LOCKED))) {
      ALOGD("%s:%s: AEC Value Converged for Camera id(%d)", LOG_TAG, __func__,camera_id);
      aec_converged_ = true;
      pthread_cond_signal(&aec_converge_cond_);
    } else {
      ALOGD("%s:%s: AEC Value still not converged for Camera id(%d)", LOG_TAG, __func__,camera_id);
    }
  }
}

#ifdef QMMF_LE_BUILD
int32_t HTTPInterface::RaveTrackResolutionChangeCb(uint32_t width,
                                      uint32_t height, uint32_t framerate,
                                      uint32_t bitrate) {
  int32_t status = NO_ERROR;
  qmmf_video_track_param track_parms = rave_track_param_store_;
  ssize_t rtsp_idx = rtsp_servers_.indexOfKey(track_parms.session_id);
  if ((NAME_NOT_FOUND == rtsp_idx) || (!ra_get_fpv_started())) {
      RaveExit();
      ALOGE("%s: session not found or stoped, rave exit!", __func__);
      return status;
  }
  track_parms.bitrate = bitrate;
  track_parms.framerate = framerate;
  track_parms.width = width;
  track_parms.height = height;
  rave_stopsession_ = true;
  if ((ra_get_fpv_started())
      && (NO_ERROR == (status = StopSession(track_parms.session_id, TRUE)))) {
    if ((ra_get_fpv_started())
        && (NO_ERROR == (status = DeleteVideoTrack(track_parms.session_id,
                                                   track_parms.track_id)))) {
      if ((ra_get_fpv_started())
          && (NO_ERROR == (status = CreateVideoTrack(track_parms)))) {
        if ((ra_get_fpv_started())
            && (NO_ERROR == (status = StartSession(track_parms.session_id)))) {
          ALOGV("%s: StartSession success: %d!", __func__, status);
          rave_track_param_store_ = track_parms;
          rave_stopsession_ = false;
          return status;
        } else {
          rave_stopsession_ = false;
          RaveExit();
          ALOGE("%s: StartSession failed: %d, rave exit!", __func__, status);
          return status;
        }
      } else {
        rave_stopsession_ = false;
        RaveExit();
        ALOGE("%s: CreateVideoTrack failed: %d, rave exit!", __func__, status);
        return status;
      }
    } else {
      rave_stopsession_ = false;
      RaveExit();
      ALOGE("%s: DeleteVideoTrack failed: %d, rave exit!", __func__, status);
      return status;
    }
  } else {
    rave_stopsession_ = false;
    RaveExit();
    ALOGE("%s: StopSession failed: %d, rave exit!", __func__, status);
    return status;
  }

  return status;
}

int32_t HTTPInterface::RaveTrackQualityChangeCb(uint32_t framerate,
                                                   uint32_t bitrate) {
  int32_t status = NO_ERROR;
  qmmf_video_track_param track_parms;
  track_parms = rave_track_param_store_;
  track_parms.bitrate = bitrate;
  track_parms.framerate = framerate;
  CodecParamType param_type;
  param_type = CodecParamType::kBitRateType;
  if (NO_ERROR == (status = recorder_.SetVideoTrackParam(track_parms.session_id,
                                                          track_parms.track_id,
                                                          param_type,
                                            &(track_parms.bitrate),
                                       sizeof(track_parms.bitrate)))) {
    param_type = CodecParamType::kFrameRateType;
    if (NO_ERROR == (status = recorder_.SetVideoTrackParam(track_parms.session_id,
                                                            track_parms.track_id,
                                                            param_type,
                                              &(track_parms.framerate),
                                         sizeof(track_parms.framerate)))) {
      ALOGV("%s: SetVideoTrackParam success: %d!", __func__, status);
      rave_track_param_store_ = track_parms;
      return status;
    } else {
      RaveExit();
      ALOGE("%s: SetVideoTrackParam framerate failed: %d!", __func__, status);
      return status;
    }
  } else {
    ALOGE("%s: SetVideoTrackParam bitrate failed: %d!", __func__, status);
    RaveExit();
    return status;
  }
}
#endif

void HTTPInterface::RecorderEventCb(EventType event_type,
                                    __attribute__((unused)) void *event_data,
                                    __attribute__((unused)) size_t event_data_size) {
  if (event_type == EventType::kCameraError) {
    pthread_mutex_lock(&snapshot_lock_);
    camera_error_ = true;
    pthread_mutex_unlock(&snapshot_lock_);
  }
}

void HTTPInterface::SessionEventCb(__attribute__((unused)) EventType event_type,
                                   __attribute__((unused)) void *event_data,
                                   __attribute__((unused)) size_t event_data_size) {
  //TBD: Once support for this callback is present in QMMF
}

} //namespace httpinterface ends here
} //namespace qmmf ends here
