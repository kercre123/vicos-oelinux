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

#include <cstddef>
#include <cstdlib>
#include <vector>
#include <string>

#include <camera/CameraMetadata.h>

#include "qmmf-sdk/qmmf_recorder_params.h"
#include "qmmf-sdk/qmmf_recorder_extra_param.h"
#include "qmmf-sdk/qmmf_overlay.h"

namespace qmmf {
namespace recorder {

using namespace overlay;

class RecorderClient;

/// \brief Client interface for audio, video recording and image capture.
///
/// To start an audio or/and video recording clients first should create a session.
/// A session can contain multiple audio and video tracks.
/// All tracks in a session changes state change (say start, stop, pause etc) together.
///
/// Multiple sessions can be created and run at the same time.
/// Recorder provides elementary stream callback to clients and clients
/// manage the muxer. A/V muxer is not part of the Recorder class and hence clients
/// need to use an external muxer to save the elementary streams
///
/// For image capture, Recorder API provides single, burst and timed image capture
/// Image capture APIs are  not associated with session and can
/// be triggered independent of session.
///
/// Some of the APIs in the class are sync and others async.
/// The API documentation explicitly says whether the API is async.
///
/// Callbacks used for async communication from recorder class are expected
/// to be lambda functions. The client is expected to capture the context
/// while setting the callback.
class Recorder {
 public:
  Recorder();

  ~Recorder();

  /// Connect to recoder service and set callback.
  status_t Connect(const RecorderCb &cb);

  /// Disconnect from recorder service.
  status_t Disconnect();

  /// \brief Initializes camera and prepares camera for image capture and/or
  /// video record.
  ///
  /// This API must be called before calling create video track or
  /// calling CaptureImage API. The result callback is optional. In case
  /// it is set by client, notifications will get triggered on each
  /// incoming streaming request along with the camera results.
  status_t StartCamera(const uint32_t camera_id, const CameraStartParam &param,
                       const CameraResultCb &cb = nullptr);

  /// \brief Stops camera. This API should be called to free up all resources
  /// associated with camera.
  ///
  /// StopCamera cannot be called when there is an
  /// active video record session or image capture session.
  status_t StopCamera(const uint32_t camera_id);

  /// \brief Creates session and returns session id.
  /// session id is used to idenfity the session in subsequent API calls.
  ///
  /// A session can contain multiple audio and video tracks. All tracks within
  /// a session changes states (like start, stop, pause) together.
  ///
  /// A callback(cb) must be registered along with session creation. This cb
  /// is used by the recorder to notify state change event notification and
  /// session specific errors back to clients asynchronously.
  ///
  /// Clients may create and run multiple sessions concurrently. All tracks
  /// within a session changes state (start, pause, stop etc) together.
  status_t CreateSession(const SessionCb &cb, uint32_t* session_id);

  /// \brief Deletes session corresponding to id. Session must be stopped before
  /// calling delete.
  ///
  /// All tracks within the sesion must be deleted before calling DeleteSession
  status_t DeleteSession(const uint32_t session_id);

  /// \brief Starts session corresponding to id. Session shall contain at least one
  /// track before calling start.
  ///
  /// This is an async API. When start is completed, session specific event cb
  /// is called by recorder
  status_t StartSession(const uint32_t session_id);

  /// \brief Stops session corresponding to id. If the do_flush flag is set to true,
  /// the pending buffers in the pipeline is discarded else stop waits till
  /// buffers in its pipeline is encoded.
  ///
  /// EOS flag is set along with last buffer cb of each track within the
  /// session. This is an async API. When stop is completed, session specific
  /// event cb is called by recorder
  status_t StopSession(const uint32_t session_id, bool do_flush);

  /// \brief Pause session corresponding to id. When the pause is called the camera
  /// device or audio device is not paused, but only encoding of the tracks
  /// is paused.
  ///
  /// This is an async API. When pause is completed, session
  /// specific event cb is called by recoder
  status_t PauseSession(const uint32_t session_id);

  /// \brief Resumes session corresponding to id. When the session is resumed,
  /// the timestamp for all tracks in the session are resumed ignoring the
  /// duration of pause.
  ///
  /// This is an async API. When resume is completed,
  /// session specific event cb is called by recorder
  status_t ResumeSession(const uint32_t session_id);

  /// \brief Provides list with information regarding all existing plugins.
  ///*
  /// This is an async API.
  status_t GetSupportedPlugins(SupportedPlugins *plugins);

  /// \brief Create a instance of the plugin and map it to a unique ID.
  ///
  /// The unique ID is set by the underlying layers.
  status_t CreatePlugin(uint32_t *uid, const PluginInfo &plugin);

  /// \brief Delete the plugin instance with the given unique ID.
  ///
  /// Must be called after DeleteVideoTrack or CancelCaptureImage.
  status_t DeletePlugin(const uint32_t &uid);

  /// \brief Set plugin specific configuration data.
  ///
  /// This is an async API.
  status_t ConfigPlugin(const uint32_t &uid, const std::string &config);

  /// \brief Creates an audio track and associates it to the session id provided.
  /// User must specify the unqiue track_id for the session.
  ///
  /// params must specify the audio track characteristics such as codec,
  /// bitrate etc. cb is used by the recorder to inform clients about track
  /// data availability and track specific async errors.
  status_t CreateAudioTrack(const uint32_t session_id, const uint32_t track_id,
                            const AudioTrackCreateParam &param,
                            const TrackCb &cb);

  /// \brief Creates an video track and associates it to the session uuid provided.
  /// User must specify the unique track_id for the session.
  ///
  /// params must specify the video track characteristics such as codec,
  /// bitrate etc.
  /// cb is used by the recorder to inform clients about track data
  /// availability and track specific async errors
  status_t CreateVideoTrack(const uint32_t session_id, const uint32_t track_id,
                            const VideoTrackCreateParam &param,
                            const TrackCb &cb);

  /// \brief Creates an video track with additional configurations set in
  /// extra_param container and associates it to the session uuid provided.
  /// User must specify the unique track_id for the session.
  ///
  /// params must specify the video track characteristics such as codec,
  /// bitrate etc.
  /// extra_param may contain additional parameters for setting crop, source
  /// surface dimensions, multi-camera mode, etc.
  /// cb is used by the recorder to inform clients about track data
  /// availability and track specific async errors
  status_t CreateVideoTrack(const uint32_t session_id, const uint32_t track_id,
                            const VideoTrackCreateParam &param,
                            const VideoExtraParam& extra_param,
                            const TrackCb &cb);

  /// \brief Returns the track buffer back to recoder
  ///
  /// Track output buffers are passed to clietns through track callbacks.
  /// Clients returns these buffers through this API
  status_t ReturnTrackBuffer(const uint32_t session_id,
                             const uint32_t track_id,
                             ::std::vector<BufferDescriptor> &buffers);

  /// \brief Changes runtime audio track parameters such as audio source device.
  ///
  /// The type of *param* data depends on the enum value of *type*
  /// param_size should be set to sizeof the param data structure
  status_t SetAudioTrackParam(const uint32_t session_id,
                              const uint32_t track_id,
                              CodecParamType type, const void *param,
                              size_t param_size);

  /// \brief Changes runtime video track params such as video encoder
  /// bitrate, framerate, IDR insertion etc
  ///
  /// The type of *param* data depends on the enum value of *type*
  /// param_size should be set to sizeof the param data structure
  status_t SetVideoTrackParam(const uint32_t session_id,
                              const uint32_t track_id,
                              CodecParamType type, const void *param,
                              size_t param_size);

  /// \brief Deletes Audio track. The track can be deleted only when the session
  /// the track is associated with is in stopped state
  status_t DeleteAudioTrack(const uint32_t session_id,
                            const uint32_t track_id);

  /// \brief Deletes video track. The track can be deleted only when the session
  /// the track is associated with is in stopped state
  status_t DeleteVideoTrack(const uint32_t session_id,
                            const uint32_t track_id);

  /// \brief Capture burst or single images from a camera
  ///
  /// This is an async API. When the image is ready, data callback specified
  /// through cb is called which enables clients to process the image data.
  /// If multiple images are captured, data cb is called for every image.
  /// If CancelCaptureImage() is called before all images are returned to
  /// clients, the Recorder stops the burst or timed image capture and returns
  /// an event indicating CANCEL is complete
  ///
  /// @param camera_id: ID of camera
  /// @param param: Details dimensions, format of the image
  /// @param num_images: Number of images to be captured
  /// @param meta: Optional camera meta parameter for each image to be captured
  ///        If this vector is empty default parameters are used for for image
  ///        capture.
  /// @param cb: Callbacks for data and error notifications
  status_t CaptureImage(const uint32_t camera_id,
                        const ImageParam &param,
                        const uint32_t num_images,
                        const std::vector<::android::CameraMetadata> &meta,
                        const ImageCaptureCb &cb);

  /// \brief Enables clients to configure image capture parameters.
  ///
  /// This is an optional API to be used only for advanced post processing
  /// features.
  ///
  /// @param camera_id: ID of camera
  /// @param param: Additional parameters for setting crop, reprocess
  ///        plugins, multi-camera mode, etc.
  status_t ConfigImageCapture(const uint32_t camera_id,
                              const ImageConfigParam &config);

  /// \brief Cancels an ongoing image capture
  ///
  /// CaptureImage is a async API. Clients can call CancelCaptureImage anytime
  /// after CaptureImage to cancel pending image captures
  status_t CancelCaptureImage(const uint32_t camera_id);

  status_t ReturnImageCaptureBuffer(const uint32_t camera_id,
                                    const BufferDescriptor &buffer);

  /// \brief Sets camera parameters
  ///
  /// Camera parameters are controlled through CameraMetadata class
  /// defined camera/CameraMetadata.h. Typical usage would be to get
  /// meta param through GetCameraParam, then update it using update
  /// function of CameraMetadata and then set this through SetCameraParam
  status_t SetCameraParam(const uint32_t camera_id,
                          const android::CameraMetadata &meta);

  /// Compelementary API of SetCameraParam. Clients generally calls
  /// GetCameraParam to get default camerametadata params
  status_t GetCameraParam(const uint32_t camera_id,
                          android::CameraMetadata &meta);

  /// Complementary API of CaptureImage. Clients generally calls
  /// GetDefaultCaptureParam to get default capture params. this API is
  /// Different then GetCameraParam, it gives default params for image capture
  /// While GetCameraParam gives default params for video record. This API is
  /// applicable only for non-zsl capture.
  status_t GetDefaultCaptureParam(const uint32_t camera_id,
                                  android::CameraMetadata &meta);

  /// \brief Create overlay object of type static image, date/time, bounding box,
  /// simple text, and privacy mask.
  ///
  /// This Api returns the object id which
  /// can be use for configuration change at runtime.
  status_t CreateOverlayObject(const uint32_t track_id,
                               const OverlayParam &param, uint32_t *overlay_id);

  /// Overlay object can be deleted at any point after creation.
  status_t DeleteOverlayObject(const uint32_t track_id,
                               const uint32_t overlay_id);

  /// \brief Overlay object's parameters can be queried after creation, it is
  /// recommended to call get parameters first before setting any new
  /// parameters using Api updateOverlayObject.
  status_t GetOverlayObjectParams(const uint32_t track_id,
                                  const uint32_t overlay_id,
                                  OverlayParam &param);

  /// \brief Overlay object's configuration can be updated at run time using this Api.
  /// Client has to provide overlay Id and updated parameters.
  ///
  /// It is recommended to call getOverlayObjectParams first to get current
  /// parameters then update them using this Api.
  status_t UpdateOverlayObjectParams(const uint32_t track_id,
                                     const uint32_t overlay_id,
                                     const OverlayParam &param);

  /// Overlay Object can be set and removed per track at runtime
  status_t SetOverlay(const uint32_t track_id, const uint32_t overlay_id);

  /// Overlay object can be dynamically removed
  status_t RemoveOverlay(const uint32_t track_id, const uint32_t overlay_id);

  /// \brief Creates a virtual camera by bundling the given camera IDs and
  /// mapping them to a virtual ID.
  ///
  /// This API must be called before calling ConfigureMultiCamera and
  /// StartCamera in order to work with the virtual camera. The virtual
  /// camera ID is set by the underlying layers.
  status_t CreateMultiCamera(const std::vector<uint32_t> camera_ids,
                             uint32_t *virtual_camera_id);

  /// \brief Configure a virtual camera with the given ID.
  ///
  /// This API must be called after CreateMultiCamera but before calling
  /// StartCamera.
  status_t ConfigureMultiCamera(const uint32_t virtual_camera_id,
                                const MultiCameraConfigType type,
                                const void *param,
                                const uint32_t param_size);
 private:
  RecorderClient* recorder_client_;

};

};  // namespace recorder

};  // namespace qmmf
