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

#include <queue>
#include <map>
#include <set>
#include <future>
#include <mutex>
#include <condition_variable>

#include <utils/KeyedVector.h>
#include <utils/Log.h>
#include <libgralloc/gralloc_priv.h>

#include "include/qmmf-plugin/qmmf_alg_intf.h"

#include "qmmf-sdk/qmmf_recorder_extra_param.h"
#include "qmmf-sdk/qmmf_recorder_extra_param_tags.h"
#include "recorder/src/service/qmmf_camera_context.h"
#include "recorder/src/service/qmmf_recorder_utils.h"
#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_camera_reprocess.h"
#include "recorder/src/service/qmmf_camera_jpeg.h"

namespace qmmf {

namespace recorder {

static const uint32_t kVirtualCameraIdOffset = 1000;

class StreamStitching;
class SnapshotStitching;
class GrallocMemory;

class MultiCameraManager : public CameraInterface {
 public:
  MultiCameraManager();

  ~MultiCameraManager();

  // This Api will map actaul camera Ids to virtual camera id.
  status_t CreateMultiCamera(const std::vector<uint32_t> camera_ids,
                             uint32_t* virtual_camera_id);

  status_t ConfigureMultiCamera(const uint32_t virtual_camera_id,
                                const MultiCameraConfigType type,
                                const void *param, const size_t param_size);

  status_t OpenCamera(const uint32_t camera_id, const CameraStartParam &param,
                      const ResultCb &cb = nullptr,
                      const ErrorCb &errcb = nullptr) override;

  status_t CloseCamera(const uint32_t camera_id) override;

  status_t WaitAecToConverge(const uint32_t timeout) override;

  status_t SetUpCapture(const ImageParam &param,
                        const uint32_t num_images) override;

  status_t CaptureImage(const std::vector<CameraMetadata> &meta,
                        const StreamSnapshotCb& cb) override;

  status_t ConfigImageCapture(const ImageConfigParam &config) override;

  status_t CancelCaptureImage() override;

  status_t CreateStream(const CameraStreamParam& param,
                        const VideoExtraParam& extra_param) override;

  status_t DeleteStream(const uint32_t track_id) override;

  status_t AddConsumer(const uint32_t& track_id,
                       sp<IBufferConsumer>& consumer) override;

  status_t RemoveConsumer(const uint32_t& track_id,
                          sp<IBufferConsumer>& consumer) override;

  status_t StartStream(const uint32_t track_id) override;

  status_t StopStream(const uint32_t track_id) override;

  status_t SetCameraParam(const CameraMetadata &meta) override;

  status_t GetCameraParam(CameraMetadata &meta) override;

  status_t GetDefaultCaptureParam(CameraMetadata &meta) override;

  status_t ReturnImageCaptureBuffer(const uint32_t camera_id,
                                    const int32_t buffer_id) override;

  CameraStartParam& GetCameraStartParam() override;

  Vector<int32_t>& GetSupportedFps() override;

 private:
  void ResultCallback(uint32_t camera_id, const CameraMetadata &meta);

  status_t SetDefaultSurfaceDim(uint32_t& w, uint32_t& h);

  int32_t ImageToHalFormat(const ImageFormat &image);

  status_t CreateJpegEncoder(const ImageParam &param);
  void EncodeJpegImage(const StreamBuffer &buffer);
  void OnStitchedFrameAvailable(StreamBuffer buffer);
  void OnJpegImageAvailable(StreamBuffer in_buffer, StreamBuffer out_buffer);
  status_t ReturnJpegBuffer(const int32_t buffer_id);

  // Create Stitching stream is identified with param.id, make sure
  // that same id is passed on DeleteStreamStitching
  status_t CreateStreamStitching(const CameraStreamParam& param);
  status_t DeleteStreamStitching(const uint32_t id);

  status_t CreateCameraStream(const uint32_t& cam_idx,
                              const CameraStreamParam& param,
                              const VideoExtraParam& extra_param);
  status_t DeleteCameraStream(const uint32_t& cam_idx,
                              const uint32_t& track_id);

  status_t FillDualCamMetadata(CameraMetadata& meta, const uint32_t& cam_idx);
  status_t FillCropMetadata(CameraMetadata& meta, const uint32_t& cam_idx);

  uint32_t                 virtual_camera_id_;
  CameraStartParam         start_params_;
  MultiCameraConfigType    multicam_type_;
  Vector<int32_t>          supported_fps_;
  ResultCb                 result_cb_;
  ErrorCb                  error_cb_;

  //Non zsl capture request.
  ImageParam               snapshot_param_;
  uint32_t                 sequence_cnt_;
  bool                     jpeg_encoding_enabled_;

  sp<SnapshotStitching>    snapshot_stitch_algo_;
  sp<ICameraPostProcess>   jpeg_encoder_;
  StreamSnapshotCb         client_snapshot_cb_;
  sp<GrallocMemory>        jpeg_memory_pool_;

  std::map<int32_t, SourceSurfaceDesc> source_surface_;
  std::map<int32_t, SurfaceCrop> surface_crop_;

  std::vector<uint32_t>    active_streams_;

  // map of virtual camera id and its corresponding actual camera Ids.
  // <virtual camera id, Vector of actual camera id >
  KeyedVector<uint32_t, Vector<uint32_t> > virtual_camera_map_;

  // Map of camera id and CameraContext.
  KeyedVector<uint32_t, sp<CameraContext>> camera_contexts_;

  // Map of track id and StreamStitching class
  KeyedVector<uint32_t, sp<StreamStitching> > stream_stitch_algos_;

  // Map of output_buffer's fd to StreamBuffer
  KeyedVector<uint32_t, StreamBuffer> jpeg_buffers_map_;

  std::mutex               jpeg_lock_;
  std::condition_variable  wait_for_jpeg_;

  std::mutex               lock_;

  static const uint32_t kWaitJPEGTimeout = 100000000; // 100 ms
  static const uint32_t kAecConvergeTimeout = 500000000; // 500 ms

  static const uint32_t kWidth4K  = 3840;
  static const uint32_t kHeight4K = 1920;
};

class GrallocMemory : public RefBase {
 public:
  struct BufferParams {
    uint32_t width;
    uint32_t height;
    int32_t  format;
    int32_t  gralloc_flags;
    uint32_t max_size;
    uint32_t max_buffer_count;
  };

  GrallocMemory(alloc_device_t *gralloc_device = nullptr);
  ~GrallocMemory();

  status_t Initialize();
  status_t Configure(BufferParams &params);

  status_t GetBuffer(buffer_handle_t &buffer);
  status_t ReturnBuffer(const buffer_handle_t &buffer);

  status_t PopulateMetaInfo(CameraBufferMetaData &info,
                            buffer_handle_t &buffer);

 private:
  status_t GetBufferLocked(buffer_handle_t &buffer);
  status_t ReturnBufferLocked(const buffer_handle_t &buffer);

  status_t AllocGrallocBuffer(buffer_handle_t *buf);
  status_t FreeGrallocBuffer(buffer_handle_t buf);

  BufferParams             params_;
  alloc_device_t           *gralloc_device_;
  buffer_handle_t          *gralloc_slots_;
  uint32_t                 buffers_allocated_;
  uint32_t                 pending_buffer_count_;

  // Pool with allocated gralloc buffers, the bool value indicates
  // if the buffer has been returned to the producer and is available
  // to be used.
  KeyedVector<buffer_handle_t, bool> gralloc_buffers_;

  std::mutex               buffer_lock_;
  std::condition_variable  wait_for_buffer_;

  static const uint32_t kBufferWaitTimeout = 1000000000; // 1 s.
};

class StitchingBase : public Camera3Thread, public RefBase  {
 public:
  struct InitParams {
    uint32_t                       multicam_id;
    Vector<uint32_t>               camera_ids;
    MultiCameraConfigType          stitch_mode;
    std::map<int32_t, SurfaceCrop> surface_crop;
    uint32_t                       frame_rate;
  };

  StitchingBase(InitParams &param);
  virtual ~StitchingBase();

  status_t Initialize();
  status_t Configure(GrallocMemory::BufferParams &param);

  int32_t Run();
  void RequestExitAndWait() override;

 protected:
  // Thread for preparing synced and output buffers for processing by the
  // stitch library and passing them to the same library for stitching.
  bool ThreadLoop() override;

  // Pure virtual methods for handling the return of stream buffers
  // to their corresponding point of origin.
  virtual status_t NotifyBufferToClient(StreamBuffer &buffer) = 0;
  virtual status_t ReturnBufferToCamera(StreamBuffer &buffer) = 0;

  // Method for handling the synchronization between frames.
  status_t FrameSync(StreamBuffer& buffer);

  // Method for returning an output buffer back to the memory pool.
  status_t ReturnBufferToBufferPool(const StreamBuffer &buffer);

  InitParams               params_;
  bool                     stop_frame_sync_;
  bool                     use_frame_sync_timeout;
  String8                  *work_thread_name_;

  uint32_t                 skip_camera_id_;
  bool                     single_camera_mode_;

  Mutex                    frame_lock_;

 private:
  struct StitchLibInterface {
    void        *handle;
    void        *context;
    bool        initialized;
    bool        configured;
    qmmf_alg_status_t (*init)(void **handle,
                              qmmf_alg_blob_t *calibration_data);
    void              (*deinit)(void *handle);
    qmmf_alg_status_t (*get_caps)(void *handle, qmmf_alg_caps_t *caps);
    qmmf_alg_status_t (*set_tuning)(void *handle, qmmf_alg_blob_t *blob);
    qmmf_alg_status_t (*config)(void *handle, qmmf_alg_config_t *config);
    qmmf_alg_status_t (*register_bufs)(void *handle, qmmf_alg_buf_list_t bufs);
    qmmf_alg_status_t (*unregister_bufs)(void *handle,
                                         qmmf_alg_buf_list_t bufs);
    qmmf_alg_status_t (*flush)(void *handle);
    qmmf_alg_status_t (*process)(void *handle,
                                 qmmf_alg_process_data_t *proc_data);
    qmmf_alg_status_t (*get_debug_info_log)(void *handle, char **log);
  };

  status_t StopFrameSync();

  status_t ReturnUnsyncedBuffers(uint32_t camera_id);
  status_t ReturnProcessedBuffer(buffer_handle_t &handle,
                                 qmmf_alg_status_t status);

  status_t InitLibrary();
  status_t DeInitLibrary();
  status_t FlushLibrary();
  status_t Configlibrary(Vector<StreamBuffer> &input_buffers,
                         Vector<StreamBuffer> &output_buffers);
  status_t ProcessBuffers(Vector<StreamBuffer> &input_buffers,
                          Vector<StreamBuffer> &output_buffers);
  status_t ParseCalibFile(void **data, uint32_t &size);
  status_t PopulateImageFormat(qmmf_alg_format_t &fmt,
                               const StreamBuffer *buffer);
  status_t PrepareBuffer(qmmf_alg_buf_list_t &reg_buf_list,
                         qmmf_alg_buffer_t &img_buffer,
                         const StreamBuffer *buffer);
  status_t UnregisterBuffers(std::set<int32_t> buffer_fds);

  static void ProcessCallback(qmmf_alg_cb_t *cb_data);

  StitchLibInterface       stitch_lib_;
  sp<GrallocMemory>        memory_pool_;

  // Map of incoming filled buffers for each of the actual cameras
  // that have not yet been synchronized.
  KeyedVector<uint32_t, Vector<StreamBuffer> > unsynced_buffer_map_;

  // List with buffers ready to go through stitch processing.
  // The uint32_t is the camera id to which this buffer belongs to.
  std::queue<KeyedVector<uint32_t, StreamBuffer> > synced_buffer_queue_;

  // Map of the stream buffers that are given to the library for processing.
  std::map<buffer_handle_t, StreamBuffer> process_buffers_map_;

  // List containing all gralloc buffers file descriptors that have been
  // registered by the library.
  std::set<int32_t> registered_buffers_;

  // The maximum interval in which two frames are thought of as syncable.
  // It is calculated, based on the frame rate.
  int32_t timestamp_max_delta_;

  std::future<status_t>    init_library_status_;

  std::mutex               register_buffer_lock_;

  std::mutex               buffers_lock_;
  std::condition_variable  wait_for_buffers_;

  std::mutex               sync_lock_;
  std::condition_variable  wait_for_sync_frames_;

  static const uint32_t kWaitBuffersTimeout = 100000000; // 100 ms
  static const uint32_t kFrameSyncTimeout   = 50000000;  // 50 ms

  static const uint8_t kUnsyncedQueueMaxSize = 3;
};

class StreamStitching : public StitchingBase {
 public:
  StreamStitching(InitParams &param);
  ~StreamStitching();

  // Methods for establishing buffer communication link between the
  // consumer of the client and buffer producer of the stitching pipeline.
  status_t AddConsumer(const sp<IBufferConsumer>& consumer);
  status_t RemoveConsumer(sp<IBufferConsumer>& consumer);

  // Method to provide consumer interface, it would be used by a CameraContext
  // port producer to post buffers.
  sp<IBufferConsumer>& GetConsumerIntf(uint32_t camera_id);

  // Method for handling incoming buffers from CameraPort buffer producer.
  void OnFrameAvailable(StreamBuffer& buffer);

  // Method for handling a buffer returned back from the CameraSource.
  void NotifyBufferReturned(const StreamBuffer& buffer);

 protected:
  status_t NotifyBufferToClient(StreamBuffer &buffer) override;
  status_t ReturnBufferToCamera(StreamBuffer &buffer) override;

 private:
  bool IsConnected(const sp<IBufferConsumer>& consumer);

  sp<IBufferProducer>      buffer_producer_impl_;

  Mutex                    consumer_lock_;

  std::map<uintptr_t, sp<IBufferConsumer> > stitching_consumers_;

  // Map of camera id and it's corresponding buffer consumer.
  KeyedVector<uint32_t, sp<IBufferConsumer> > camera_consumers_map_;
};

class SnapshotStitching : public StitchingBase {
 public:
  SnapshotStitching(InitParams &param,
                    KeyedVector<uint32_t, sp<CameraContext> > &contexts);
  ~SnapshotStitching();

  void SetClientCallback(const StreamSnapshotCb& cb) {
    client_snapshot_cb_ = cb;
  }

  // A callback method for handling incoming buffers from CameraContexts.
  void FrameAvailableCb(uint32_t count, StreamBuffer &buffer);

  // Method for handling a buffer returned back from the CameraSource.
  status_t ImageBufferReturned(const int32_t buffer_id);

 protected:
  status_t NotifyBufferToClient(StreamBuffer &buffer) override;
  status_t ReturnBufferToCamera(StreamBuffer &buffer) override;

 private:
  // Maps of buffer Id and Buffer.
  KeyedVector<uint32_t, StreamBuffer> snapshot_buffer_list_;

  // Map of camera id and CameraContext taken from MultiCameraManager.
  KeyedVector<uint32_t, sp<CameraContext> > camera_contexts_;

  StreamSnapshotCb         client_snapshot_cb_;
  Mutex                    snapshot_lock;
};

}; // recorder.

}; // qmmf.
