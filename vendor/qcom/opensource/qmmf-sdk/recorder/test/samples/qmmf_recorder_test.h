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

#include <chrono>
#include <condition_variable>
#include <map>
#include <mutex>
#include <vector>

#include "recorder/test/samples/qmmf_recorder_test_wav.h"
#include "recorder/test/samples/qmmf_recorder_test_aac.h"
#include "recorder/test/samples/qmmf_recorder_test_amr.h"

#include <camera/CameraMetadata.h>
#include <qmmf-sdk/qmmf_buffer.h>
#include <qmmf-sdk/qmmf_codec.h>
#include <qmmf-sdk/qmmf_display.h>
#include <qmmf-sdk/qmmf_display_params.h>

#include <condition_variable>
#include <QCamera3VendorTags.h>
#include <cutils/properties.h>
#include <cutils/trace.h>
#include <linux/input.h>

#if USE_SKIA
#include <SkCanvas.h>
#elif USE_CAIRO
#include <cairo/cairo.h>
#endif

#include <qmmf-sdk/qmmf_recorder.h>
#include <qmmf-sdk/qmmf_recorder_params.h>

// Enable this define to dump YUV data from YUV track
#define DUMP_YUV_FRAMES

//#define DEBUG
//Logging related defines
#define TEST_INFO(fmt, args...)  ALOGD(fmt, ##args)
#define TEST_ERROR(fmt, args...) ALOGE(fmt, ##args)
#ifdef DEBUG
#define TEST_DBG  TEST_INFO
#else
#define TEST_DBG(...) ((void)0)
#endif


#define KPI_DISABLE 0
#define KPI_ONLY 1
extern volatile uint32_t kpi_debug_mask;

#define TEST_KPI_GET_MASK() ({\
char prop[PROPERTY_VALUE_MAX];\
property_get("persist.qmmf.kpi.debug", prop, "0"); \
kpi_debug_mask = atoi (prop);})

#define TEST_KPI_BEGIN(name) ({\
if (kpi_debug_mask & KPI_ONLY) { \
  atrace_begin(ATRACE_TAG_ALWAYS, name); \
}\
})

#define TEST_KPI_END() ({\
if (kpi_debug_mask & KPI_ONLY) { \
  atrace_end(ATRACE_TAG_ALWAYS); \
}\
})

#define TEST_KPI_ASYNC_BEGIN(name, cookie) ({\
if (kpi_debug_mask & KPI_ONLY) { \
  atrace_async_begin(ATRACE_TAG_ALWAYS, name, cookie); \
}\
})

#define TEST_KPI_ASYNC_END(name, cookie) ({\
if (kpi_debug_mask & KPI_ONLY) { \
  atrace_async_end(ATRACE_TAG_ALWAYS, name, cookie); \
}\
})

// Enable this define to dump encoded bit stream data.
#define DUMP_BITSTREAM

// Sleep for specified seconds to allow settling after parameter change
#define PARAMETER_SETTLE_INTERVAL(x) sleep(x)

#define FEATURE_NOT_AVAILABLE  "Not available"

#define TEXT_SIZE                   40

using namespace qmmf;
using namespace recorder;
using namespace android;
using namespace qcamera;
using ::qmmf::display::DisplayEventType;
using ::qmmf::display::DisplayType;
using ::qmmf::display::Display;
using ::qmmf::display::DisplayCb;
using ::qmmf::display::SurfaceBuffer;
using ::qmmf::display::SurfaceParam;
using ::qmmf::display::SurfaceConfig;
using ::qmmf::display::SurfaceBlending;
using ::qmmf::display::SurfaceFormat;

#define AEC_SETTLE_INTERVAL 2
#define MAX_NUM_CAMERAS 3

#define DEFAULT_DUMP_FRAME_FREQ  "200"

// Prop to enable YUV data dumping from YUV track
#define PROP_DUMP_YUV          "persist.qmmf.rec.test.dumpyuv"
// Prop to enable encoded bitstream data dumping
#define PROP_DUMP_BITSTREAM    "persist.qmmf.rec.test.dumpstrm"
// Prop to enable JPEG (BLOB) dumping
#define PROP_DUMP_JPEG         "persist.qmmf.rec.gtest.dumpjpeg"
// Prop to enable RAW Snapshot dumping
#define PROP_DUMP_RAW          "persist.qmmf.rec.gtest.dumpraw"
// Prop to set frequency of YUV data dumping
#define PROP_DUMP_FRAME_FREQ   "persist.qmmf.rec.test.dumpfreq"

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

#ifndef CLIP
#define CLIP(X, L, U) MIN(MAX((X), (L)), (U))
#endif

enum class AfMode {
  kNone,
  kOff,
  kAuto,
  kMacro,
  kContinousVideo,
  kContinuousPicture,
  kEdof
};

enum class SnapshotType {
   kNone,
   kJpeg,
   kRawYuv,
   kRawRdi,
   kJpegBurst
};

struct SnapshotInfo {
   SnapshotType   type;
   uint32_t       width;
   uint32_t       height;
   uint32_t       count;
   int32_t        camera_id;
};

enum class TrackType {
  kNone,
  kAudioPCM,
  kAudioAAC,
  kAudioAMR,
  kAudioG711,
  kVideoYUV,
  kVideoRDI,
  kVideoAVC,
  kVideoHEVC,
  kVideoPreview
};

struct TrackInfo {
  uint32_t  width;
  uint32_t  height;
  float     fps;
  TrackType track_type;
  uint32_t  bitrate;
  int32_t  ltr_count;
  uint32_t  session_id;
  uint32_t  track_id;
  int32_t   camera_id;
  uint32_t  low_power_mode;
  DeviceId  device_id;

  TrackInfo()
      : width(3840),
        height(2160),
        fps(30),
        track_type(TrackType::kVideoAVC),
        bitrate(6000000),
        ltr_count(0),
        session_id(-1),
        track_id(1),
        camera_id(0),
        low_power_mode(0),
        device_id(0) {}

  TrackInfo(uint32_t width, uint32_t height, float fps, TrackType track_type,
            uint32_t bitrate, int32_t ltr_count, uint32_t session_id,
            uint32_t track_id, int32_t camera_id, uint32_t low_power_mode,
            DeviceId device_id)
      : width(width),
        height(height),
        fps(fps),
        track_type(track_type),
        bitrate(bitrate),
        ltr_count(ltr_count),
        session_id(session_id),
        track_id(track_id),
        camera_id(camera_id),
        low_power_mode(low_power_mode),
        device_id(device_id) {}
};


struct RGBAValues {
  double red;
  double green;
  double blue;
  double alpha;
};

class CameraMetaDataParser {
public:
  CameraMetaDataParser();
  ~CameraMetaDataParser();
  bool IsIREnabled(const CameraMetadata& metadata);
  bool IsTNREnabled(const CameraMetadata& metadata);
  bool IsSVHDREnabled(const CameraMetadata& metadata);
  int64_t ParseFrameTime(const CameraMetadata& metadata);
};

class CheckKPITime {
public:
  CheckKPITime();
  ~CheckKPITime();
  void SetUp();
  void CheckSwicthTime(const CameraMetadata& metadata);
  void ParseCameraMetaData(const CameraMetadata& metadata);

private:
  CameraMetaDataParser     cam_metadata_parser_;
  bool                     prev_nr_mode_;
  bool                     prev_ir_mode_;
  bool                     prev_svhdr_mode_;
  bool                     new_nr_mode_;
  bool                     new_ir_mode_;
  bool                     new_svhdr_mode_;
  int64_t                  last_frame_time_;
  int64_t                  new_frame_time_;
  bool                     mark_first_frame_time_;
};

enum class DynamicCameraParamsCmd {
  kExit                = 'X',
  kSharpness           = '1',
  kAEGain              = '2',
  kExposureTime        = '3',
  kWNRStrength         = '4',
  kTNRTuning           = '5',
  kDumpHistogramStats  = '6',
  kDumpAECAWBStats     = '7'
};

enum class TNRTuningCmd {
  kExit                        = 'X',
  kTNRIntensity                = '1',
  kMotionDetectionSensitivity  = '2'
};

enum AutoModeOptions : char {
  kWidth      = 'w',
  kHeight     = 'h',
  kFps        = 'f',
  kTrackType  = 't'
};

class TestTrack;
class CmdMenu;

 struct CameraInitInfo {
   int32_t                           camera_id;
   uint32_t                          camera_fps;
   uint32_t                          numStream;
   AfMode                            af_mode;
   bool                              tnr;
   bool                              vhdr;
   bool                              binning_correct;
   CameraInitInfo():
        camera_id(-1),
        camera_fps(0),
        numStream(0),
        af_mode(AfMode::kOff),
        tnr(false),
        vhdr(false),
        binning_correct(false) {};
 };

class TestInitParams {
public:
    int32_t                           num_cameras;
    uint32_t                          recordTime;
    SnapshotInfo                      snapshot_info;
    std::vector<CameraInitInfo*>      cam_init_infos;

    TestInitParams() :
            num_cameras(1),
            recordTime(0),
            snapshot_info {
              SnapshotType::kNone,
              0,
              0,
              0,
              0
            } {};

    ~TestInitParams() {
      for(std::vector<uint32_t>::size_type i = 0;
          i < cam_init_infos.size(); i++) {
        delete cam_init_infos.at(i);
      }
      cam_init_infos.clear();
    }
};

typedef struct StreamDumpInfo {
  VideoFormat   format;
  uint32_t      track_id;
  int32_t       width;
  int32_t       height;
} StreamDumpInfo;

class DumpBitStream {
 public:
  DumpBitStream() : file_fd_(-1) {};

  ~DumpBitStream() {};

  status_t SetUp(const StreamDumpInfo& dumpinfo);

  void Close();

  status_t Dump(const std::vector<BufferDescriptor>& buffers);

  int32_t file_fd_;
};

typedef struct ROIRegion {
  int32_t roi_coordinates[5];
  int32_t rgb_color[3];
  ROIRegion() {
    roi_coordinates[0] = roi_coordinates[1] = roi_coordinates[2] =
    roi_coordinates[3] = roi_coordinates[4] = 1;

    rgb_color[0] = rgb_color[1] = rgb_color[2] = 1;
  }
} ROIRegion;

typedef std::chrono::high_resolution_clock clk;
typedef std::chrono::milliseconds milliseconds;

class RecorderTest {
 public:
  RecorderTest();

  ~RecorderTest();

  status_t Connect();

  status_t Disconnect();

  status_t StartCamera();

  status_t StopCamera();

  status_t TakeSnapshot();

  status_t TakeSnapshotWithConfig(const SnapshotInfo& snapshot_info);

  status_t CancelTakeSnapshot();

  status_t StartMultiCameraMode();

  status_t StopMultiCameraMode();

  status_t Session4KAnd1080pYUVTracks();

  status_t Session4KEncTrack(const TrackType& type);

  status_t Session1080pEncTrack(const TrackType& type);

  status_t Session1080pEnc1080YUV(const TrackType& type);

  status_t Session4KHEVCAnd1080pYUVTracks(const TrackType& type);

  status_t Session4KYUVAnd1080pEncTracks(const TrackType& type);

  status_t SessionTwo1080pEncTracks(const TrackType& type);

  status_t Session720pLPMTrack(const TrackType& type);

  status_t Session1080pEnc1080pLPMTracks(const TrackType& track_type);

  status_t Session1080pYUVTrackWithDisplay();

  status_t Session1080pYUVTrackWithPreview();

  status_t CreateAudioPCMTrack();

  status_t CreateAudio2PCMTrack();

  status_t CreateAudioSCOTrack();

  status_t CreateAudioPCMSCOTrack();

  status_t CreateAudioA2DPTrack();

  status_t CreateAudioPCMA2DPTrack();

  status_t CreateAudioAACTrack();

  status_t CreateAudio2AACTrack();

  status_t CreateAudioPCMAACTrack();

  status_t CreateAudioAMRTrack();

  status_t CreateAudio2AMRTrack();

  status_t CreateAudioPCMAMRTrack();

  status_t CreateAudioG711Track();

  status_t CreateAudio2G711Track();

  status_t CreateAudioPCMG711Track();

  status_t SessionRDITrack();

  status_t StartSession();

  status_t StopSession();

  status_t SetParams();

  status_t SetDynamicCameraParam();

  status_t PauseSession();

  status_t ResumeSession();

  status_t DeleteSession();

  status_t EnableOverlay();

  status_t DisableOverlay();

  status_t HandleAWBROIRequest();

  void GetMaxResolutionTrack(TrackInfo &);

  void PrintAWBROIHelp();

  void printInitParamAndTtrackInfo(
                                   const TestInitParams& initParams,
                                   const std::vector<TrackInfo>& infos);

  status_t AddPreviewTrack();
  status_t RemovePreviewTrack();
  void PreviewTrackHandler(uint32_t session_id, uint32_t track_id,
                           std::vector<BufferDescriptor> buffers,
                           std::vector<MetaData> meta_buffers);
  void CameraResultCallbackHandler(uint32_t camera_id,
                                   const CameraMetadata &result);

  int32_t ToggleNR();
  int32_t ToggleVHDR();
  int32_t ToggleIR();
  status_t ToggleAFMode(int32_t camera_id, const AfMode& af_mode);
  int32_t ToggleBinningCorrectionMode();
  int32_t ChooseCamera();
  int32_t SetAntibandingMode();
  std::string GetCurrentNRMode();
  std::string GetCurrentVHDRMode();
  std::string GetCurrentIRMode();
  std::string GetCurrentBinningCorrectionMode(int32_t camera_id);
  status_t GetCurrentAFMode(int32_t& mode);
  status_t GetSharpnessStrength(int32_t &strength);
  status_t SetSharpnessStrength(const int32_t& val);
  status_t GetSensorSensitivity(int32_t *sensitivity);
  status_t SetSensorSensitivity(const int32_t& val);
  status_t GetExposureTime(int64_t *time_ns);
  status_t SetExposureTime(const int64_t& val);
  status_t GetWNRStrength(int32_t *wnr_strength);
  status_t SetWNRStrength(const int32_t& val);
  status_t SetTNRLevel();
  status_t GetTNRIntensity(float *intensity);
  status_t SetTNRIntensity(const float& intensity);
  status_t GetTNRMotionDetectionSensitivity(float *sensitivity);
  status_t SetTNRMotionDetectionSensitivity(const float& sensitivity);
  status_t GetRawHistogramStatistic(const CameraMetadata& meta);
  status_t GetRawAECAWBStatistic(const CameraMetadata& meta);
  status_t GetCurrentAFMode(int32_t camera_id, int32_t& mode);

  uint32_t get_snapshot_cb_wait_time() {
    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.qmmf.rec.test.snaptime",prop,"10");
    return atoi (prop);
  }

  bool is_test_cancel_snapshot() {
    char prop[PROPERTY_VALUE_MAX];
    property_get("persist.qmmf.rec.test.canclsnap", prop, "0");
    return atoi (prop);
  }

  // Auto Mode
  int32_t ParseAutoModeParams(int32_t argc, char *argv[],
                              VideoTrackCreateParam *track_param);

  int32_t RunAutoMode(int32_t argc, char* argv[]);

  int32_t RunWarmBootMode(int32_t argc, char* argv[]);

  // Config file related.
  int32_t RunFromConfig(int32_t argc, char *argv[]);

  int32_t ParseConfig(char *fileName, TestInitParams* initParams,
                      std::vector<TrackInfo>* infos);

  void SnapshotCb(uint32_t camera_id, uint32_t image_sequence_count,
                  BufferDescriptor buffer, MetaData meta_data);

  void RecorderEventCallbackHandler(EventType event_type, void *event_data,
                                    size_t event_data_size);

  void SessionCallbackHandler(EventType event_type,
                              void *event_data, size_t event_data_size);

  status_t DumpFrameToFile(BufferDescriptor& buffer,
                           CameraBufferMetaData& meta_data, String8& file_name);

  Recorder& GetRecorder() { return recorder_; }

  bool is_dump_yuv_enabled_;
  bool is_dump_raw_enabled_;
  bool is_dump_bitstream_enabled_;
  bool is_dump_jpg_enabled_;
  uint32_t dump_frame_freq_;

 private:
  ROIRegion roi_region_;
  Recorder recorder_;

  friend class CmdMenu;

  typedef std::map<uint8_t, std::string> nr_modes_map;
  typedef std::map<uint8_t, std::string>::iterator nr_modes_iter;
  typedef std::map<int32_t, std::string> vhdr_modes_map;
  typedef std::map<int32_t, std::string>::iterator vhdr_modes_iter;
  typedef std::map<int32_t, std::string> ir_modes_map;
  typedef std::map<int32_t, std::string>::iterator ir_modes_iter;
  typedef std::map<int32_t, std::string> bc_modes_map;
  typedef std::map<int32_t, std::string>::iterator bc_modes_iter;
  void InitSupportedNRModes();
  void InitSupportedVHDRModes();
  void InitSupportedIRModes();
  void InitSupportedBinningCorrectionModes();
  int32_t SetBinningCorrectionMode(int32_t camera_id, const bool& mode);

  int32_t ParseWarmBootTestParams(int32_t argc, char* argv[],
                                  TrackInfo* track_info);
  bool IsKeyEventShort(const milliseconds keypress_duration);
  int32_t StartRecording(const VideoTrackCreateParam& video_track_param);
  int32_t StopRecording();

  nr_modes_map supported_nr_modes_;
  vhdr_modes_map supported_hdr_modes_;
  ir_modes_map supported_ir_modes_;
  bc_modes_map supported_bc_modes_;

  std::map<uint32_t, std::vector<TestTrack*> > sessions_;
  typedef std::map<uint32_t, std::vector<TestTrack*> >::iterator session_iter_;
  uint32_t camera_id_;
  bool session_enabled_;
  CameraMetadata static_info_;
  uint32_t preview_session_id_;
  SnapshotType snapshot_choice_;

  uint32_t current_session_id_;
  bool use_display;
  bool dump_aec_awb_stats_;
  bool dump_histogram_stats_;

  CheckKPITime kpi_marker_;
  ::std::condition_variable signal_;
  ::std::mutex message_lock_;
  ::std::condition_variable signal_cb_;
  ::std::mutex callback_lock_;
  uint32_t num_images_;
  bool aec_converged_;
  bool in_suspend_;

  int32_t ltr_count_;

  std::mutex               snapshot_wait_lock_;
  std::condition_variable  snapshot_wait_signal_;
  uint32_t                 burst_snapshot_count_;
  std::mutex               error_lock_;
  bool                     camera_error_;
};

// Track can be types of Audio or Video, this class is responsible for creating
// tracks, setting required parameters, registering the data/event callback,
// dumping the data for verification purpose etc.
class TestTrack {

 public:
  TestTrack(RecorderTest* recorder_test);

  ~TestTrack();

  TrackType& GetTrackType() { return track_info_.track_type; }

  uint32_t GetTrackId() { return track_info_.track_id; }

  uint32_t GetCameraId() { return track_info_.camera_id; }

  status_t SetUp(TrackInfo& track_info);

  // Set up file to dump track data.
  status_t Prepare();

  // Clean up file.
  status_t CleanUp();

  status_t EnableOverlay();

  status_t DisableOverlay();

  status_t DrawOverlay(void *data, int32_t width, int32_t height);

  void ExtractColorValues(uint32_t hex_color, RGBAValues* color);

  void DisplayCallbackHandler(DisplayEventType event_type, void *event_data,
      size_t event_data_size);

  void DisplayVSyncHandler(int64_t time_stamp);

  status_t StartDisplay(DisplayType display_type);

  status_t StopDisplay(DisplayType display_type);

  const TrackInfo& GetTrackHandle(){return track_info_;}

 private:

  void TrackEventCB(uint32_t track_id, EventType event_type, void *event_data,
                    size_t event_data_size);

  void TrackDataCB(uint32_t track_id, std::vector<BufferDescriptor> buffers,
                   std::vector<MetaData> meta_buffers);

  status_t PushFrameToDisplay(BufferDescriptor& buffer,
    CameraBufferMetaData& meta_data);

  TrackInfo track_info_;

  // One track can have multiple overlay objects.
  std::vector<uint32_t> overlay_ids_;

  RecorderTest* recorder_test_;

  RecorderTestWav wav_output_;
  RecorderTestAac aac_output_;
  RecorderTestAmr amr_output_;

  uint32_t num_yuv_frames_;

  Display*   display_;
  uint32_t   surface_id_;
  SurfaceParam surface_param_;
  SurfaceBuffer surface_buffer_;
  bool display_started_;

  DumpBitStream dump_bitstream_;
#if USE_SKIA
  SkCanvas*                canvas_;
#elif USE_CAIRO
  cairo_surface_t*         cr_surface_;
  cairo_t*                 cr_context_;
#endif
};

class CmdMenu
{
public:
    enum CommandType {
        CONNECT_CMD                             = '1',
        DISCONNECT_CMD                          = '2',
        START_CAMERA_CMD                        = '3',
        STOP_CAMERA_CMD                         = '4',
        START_MULTICAMERA_CMD                   = 's',
        STOP_MULTICAMERA_CMD                    = 't',
        CREATE_YUV_SESSION_CMD                  = '5',
        CREATE_4KENC_AVC_SESSION_CMD            = '6',
        CREATE_4KENC_HEVC_SESSION_CMD           = '7',
        CREATE_1080pENC_AVC_SESSION_CMD         = '8',
        CREATE_1080pENC_HEVC_SESSION_CMD        = '9',
        CREATE_4KYUV_1080pENC_SESSION_CMD       = 'V',
        CREATE_TWO_1080pENC_SESSION_CMD         = 'M',
        CREATE_1080pENC_AVC_1080YUV_SESSION_CMD = 'E',
        CREATE_4KHEVC_AVC_1080YUV_SESSION_CMD   = 'F',
        CREATE_720pLPM_SESSION_CMD              = 'G',
        CREATE_1080pENC_AVC_1080LPM_SESSION_CMD = 'J',
        CREATE_PCM_AUD_SESSION_CMD              = 'a',
        CREATE_2PCM_AUD_SESSION_CMD             = 'b',
        CREATE_SCO_AUD_SESSION_CMD              = 'c',
        CREATE_PCM_SCO_AUD_SESSION_CMD          = 'd',
        CREATE_A2DP_AUD_SESSION_CMD             = 'e',
        CREATE_PCM_A2DP_AUD_SESSION_CMD         = 'f',
        CREATE_AAC_AUD_SESSION_CMD              = 'g',
        CREATE_2AAC_AUD_SESSION_CMD             = 'h',
        CREATE_PCM_AAC_AUD_SESSION_CMD          = 'i',
        CREATE_AMR_AUD_SESSION_CMD              = 'j',
        CREATE_2AMR_AUD_SESSION_CMD             = 'k',
        CREATE_PCM_AMR_AUD_SESSION_CMD          = 'l',
        CREATE_G7ll_AUD_SESSION_CMD             = 'm',
        CREATE_2G7ll_AUD_SESSION_CMD            = 'n',
        CREATE_PCM_G7ll_AUD_SESSION_CMD         = 'o',
        CREATE_RDI_SESSION_CMD                  = 'r',
        CREATE_YUV_SESSION_DISPLAY_CMD          = 'Z',
        CREATE_YUV_SESSION_PREVIEW_CMD          = 'Y',
        START_SESSION_CMD                       = 'A',
        STOP_SESSION_CMD                        = 'B',
        TAKE_SNAPSHOT_CMD                       = 'S',
        SET_PARAM_CMD                           = 'T',
        SET_DYNAMIC_CAMERA_PARAM_CMD            = '~',
        PAUSE_SESSION_CMD                       = 'P',
        RESUME_SESSION_CMD                      = 'R',
        ENABLE_OVERLAY_CMD                      = 'O',
        DISABLE_OVERLAY_CMD                     = 'L',
        DELETE_SESSION_CMD                      = 'D',
        NOISE_REDUCTION_CMD                     = 'N',
        VIDEO_HDR_CMD                           = 'H',
        IR_MODE_CMD                             = 'I',
        EXIT_CMD                                = 'X',
        CHOOSE_CAMERA_CMD                       = 'C',
        SET_ANTIBANDING_MODE_CMD                = 'W',
        BINNING_CORRECTION_CMD                  = '#',
        AWB_ROI_CMD                             = '$',
        NEXT_CMD                                = '\n',
        INVALID_CMD                             = '0'
    };

    struct Command {
        Command( CommandType cmd)
        : cmd(cmd) {}
        Command()
        : cmd(INVALID_CMD) {}
        CommandType cmd;
    };

    CmdMenu(RecorderTest &ctx) :  ctx_(ctx) {};

    ~CmdMenu() {};

    Command GetCommand(bool& is_print_menu);

    void PrintMenu();

    RecorderTest &ctx_;
};
