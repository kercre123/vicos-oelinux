/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "RecorderHALReproc"

#include "recorder/src/service/qmmf_recorder_utils.h"
#include "recorder/src/service/qmmf_camera_context.h"

#include "qmmf_camera_hal_reproc.h"

uint32_t qmmf_log_level;

namespace qmmf {

namespace recorder {

CameraHalReproc::CameraHalReproc(IPostProc* context)
    : context_(context),
      abort_(nullptr),
      frame_processing_(false) {
  QMMF_VERBOSE("%s: Enter ", __func__);
  static_meta_ = context_->GetCameraStaticMeta();
  state_ = PostProcHalState::CREATED;
  QMMF_VERBOSE("%s: Exit (0x%p)", __func__, this);
}

CameraHalReproc::~CameraHalReproc() {
  QMMF_VERBOSE("%s: Enter ", __func__);
  QMMF_VERBOSE("%s: Exit (0x%p)", __func__, this);
}

void CameraHalReproc::GetInputBuffer(StreamBuffer &buffer) {
  std::lock_guard<std::mutex> input_bufs_lock(input_buffer_lock_);
  auto iter = input_buffer_.begin();
  input_buffer_done_.push_back((*iter));
  buffer = (*iter);
  input_buffer_.erase(iter);
}

void CameraHalReproc::ReturnInputBuffer(StreamBuffer &buffer) {
  std::lock_guard<std::mutex> input_bufs_lock(input_buffer_lock_);
  auto iter = input_buffer_done_.begin();
  for (; iter != input_buffer_done_.end(); iter++) {
    if ((*iter).handle == buffer.handle) {
      listener_->OnFrameProcessed(*iter);
      input_buffer_done_.erase(iter);
      break;
    }
  }
}

void CameraHalReproc::ReturnAllInputBuffers() {
  std::lock_guard<std::mutex> lock(reproc_lock_);
  std::lock_guard<std::mutex> input_bufs_lock(input_buffer_lock_);
  auto iter = input_buffer_done_.begin();
  for (; iter != input_buffer_done_.end(); iter++) {
    listener_->OnFrameProcessed(*iter);
  }
  input_buffer_done_.clear();
  input_buffer_.clear();
  reproc_partial_list_.clear();
  reproc_ready_list_.clear();
}

void CameraHalReproc::ReprocessCallback(StreamBuffer in_buff) {
  QMMF_INFO("%s: HAL reprocess is done! StreamBuffer(0x%p) fd: %d stream_id:"
      "%d ts: %lld",   __func__, in_buff.handle, in_buff.fd,
      in_buff.stream_id, in_buff.timestamp);

  listener_->OnFrameReady(in_buff);

  auto ret = StartProcessing(true);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Failed: Reprocess is not started.\n", __func__);
  }
}

status_t CameraHalReproc::Initialize(const PostProcIOParam &in_param,
                                     const PostProcIOParam &out_param) {
  {
    std::lock_guard<std::mutex> lock(module_lock_);
    if (state_ != PostProcHalState::CREATED) {
      QMMF_ERROR("%s: Failed: Wrong state %d.", __func__, state_);
      return BAD_VALUE;
    }
    state_ = PostProcHalState::INITIALIZED;
  }

  auto ret = ValidateInput(in_param, out_param);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed: Wrong input parameters.", __func__);
    return BAD_VALUE;
  }

  input_param_ = in_param;
  output_param_ = out_param;

  ret = CreateDeviceStreams();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to setup camera: %d\n", __func__, ret);
    return ret;
  }

  return NO_ERROR;
}

PostProcIOParam CameraHalReproc::GetInput(const PostProcIOParam &out) {
  PostProcIOParam input_param = out;

  if (!static_meta_.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
    QMMF_ERROR("%s: Hal does not report supported sizes\n", __func__);
    assert(0);
  }

  switch (out.format) {
  case BufferFormat::kBLOB:
    // We have to simulate ZLS path in order to reduce camera load
    // otherwise we cannot achieve 4K JPEG re-processing with one
    // ISP because of camera limitations
    input_param.format = BufferFormat::kNV21;
    input_param.gralloc_flags |= GRALLOC_USAGE_HW_CAMERA_ZSL;
    break;
  case BufferFormat::kNV12:
  case BufferFormat::kNV12UBWC:
  case BufferFormat::kNV21:
    input_param.format = BufferFormat::kRAW16;
    break;
  default:
    QMMF_ERROR("%s: Error: output format %d is not supported\n",
        __func__, out.format);
    assert(0);
    break;
  }

  auto ret = ValidateFormat(input_param.format);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Hal does not support required format\n", __func__);
    assert(0);
  }

  // Query RAW dimensions if input format is RAW
  if (input_param.format == BufferFormat::kRAW10 ||
      input_param.format == BufferFormat::kRAW12 ||
      input_param.format == BufferFormat::kRAW16) {
    camera_metadata_entry_t entry;
    entry = static_meta_.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES);
    uint32_t i;
    for (i = 0; i < entry.count; i += 2) {
      input_param.width = entry.data.i32[i + 0];
      input_param.height = entry.data.i32[i + 1];
      if (input_param.width >= out.width &&
          input_param.height >= out.height) {
        break;
      }
    }
    if (i >= entry.count) {
      QMMF_ERROR("%s: Required resolution %dx%d is not supported. Max: %dx%d\n",
          __func__, out.width, out.height, input_param.width,
          input_param.height);
      assert(0);
    }
  }

  // set number of needed buffers for rotation if client does not limit it
  if (out.buffer_max > 0 && out.buffer_max < kBufCount) {
    input_param.buffer_count = out.buffer_max;
  } else {
    input_param.buffer_count = kBufCount;
  }

  QMMF_VERBOSE("%s: input dim %dx%d format %x", __func__,
      input_param.width, input_param.height, input_param.format);

  return input_param;
}

status_t CameraHalReproc::ValidateFormat(BufferFormat fmt) {
  int32_t hal_format = Common::FromQmmfToHalFormat(fmt);

  if (!static_meta_.exists(ANDROID_SCALER_AVAILABLE_FORMATS)) {
    QMMF_ERROR("%s: Hal does not report supported formats\n", __func__);
    return BAD_VALUE;
  }

  camera_metadata_entry_t entry;
  entry = static_meta_.find(ANDROID_SCALER_AVAILABLE_FORMATS);
  for (uint32_t i = 0; i < entry.count; i++) {
    if (entry.data.i32[i] == hal_format) {
      return NO_ERROR;
    }
  }

  QMMF_ERROR("%s: Hal does not support 0x%x format. qmmf format: %d\n",
      __func__, hal_format, fmt);
  return BAD_VALUE;
}

status_t CameraHalReproc::ValidateDimensions(uint32_t width, uint32_t height) {
  if (!static_meta_.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
    QMMF_ERROR("%s: HAL does not report supported sizes!", __func__);
    return NAME_NOT_FOUND;
  }

  camera_metadata_entry_t entry;
  entry = static_meta_.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
  for (uint32_t i = 0; i < entry.count; i += 4) {
    uint32_t scalar_format = entry.data.i32[i];
    uint32_t config_type = entry.data.i32[i + 3];

    if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == scalar_format &&
        ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT == config_type) {
      uint32_t w = entry.data.i32[i + 1];
      uint32_t h = entry.data.i32[i + 2];
      if (w == width && h == height) {
        return NO_ERROR;
      }
    }
  }

  QMMF_ERROR("%s: Dimensions (%dx%d) not supported", __func__,
      width, height);
  return BAD_VALUE;
}

status_t CameraHalReproc::ValidateInput(const PostProcIOParam& input,
                                        const PostProcIOParam& output) {
  camera_metadata_entry_t entry;
  int32_t in_format, num_output_formats;

  CameraMetadata static_meta = context_->GetCameraStaticMeta();

  if (static_meta_.exists(ANDROID_SCALER_AVAILABLE_INPUT_OUTPUT_FORMATS_MAP)) {
    entry = static_meta_.find(ANDROID_SCALER_AVAILABLE_INPUT_OUTPUT_FORMATS_MAP);
    for (uint32_t i = 0 ; i < entry.count; i++) {
      in_format = entry.data.i32[i++];
      num_output_formats = entry.data.i32[i++];
      if (in_format != Common::FromQmmfToHalFormat(input.format)) {
        i +=  (num_output_formats - 1);
        continue;
      }
      for (int32_t f = 0; f < num_output_formats; f++) {
        i += f;
        if (Common::FromQmmfToHalFormat(output.format) == entry.data.i32[i])
          return NO_ERROR;
      }
    }
  } else {
    QMMF_ERROR("%s: Failed ANDROID_SCALER_AVAILABLE_INPUT_OUTPUT_FORMATS_MAP\n",
        __func__);
  }

  QMMF_ERROR("%s: Failed: input format: 0x%x out format 0x%x\n",
      __func__, input.format, output.format);

  return BAD_VALUE;
}

status_t CameraHalReproc::ValidateOutput(const PostProcIOParam &output) {
  auto ret = ValidateFormat(output.format);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Output format(%d) not supported",
        __func__, output.format);
    return BAD_TYPE;
  }

  ret = ValidateDimensions(output.width, output.height);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Output dimensions(%dx%d) not supported", __func__,
        output.width, output.height);
    return BAD_VALUE;
  }

  return NO_ERROR;
}

status_t CameraHalReproc::GetCapabilities(PostProcCaps &caps) {
  camera_metadata_entry_t entry;
  CameraMetadata static_meta = context_->GetCameraStaticMeta();

  if (!static_meta_.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
    QMMF_ERROR("%s: HAL does not report supported raw sizes!",
        __func__);
    return NAME_NOT_FOUND;
  }

  if (!static_meta_.exists(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES)) {
    QMMF_ERROR("%s: HAL does not report supported  proc sizes!",
        __func__);
    return NAME_NOT_FOUND;
  }

  // find max supported resolution
  entry = static_meta_.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES);
  caps.max_width_  = entry.data.i32[0];
  caps.max_height_ = entry.data.i32[1];

  // find min supported resolution
  entry = static_meta_.find(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES);
  caps.min_width_  = entry.data.i32[entry.count - 2];
  caps.min_height_ = entry.data.i32[entry.count - 1];

  QMMF_VERBOSE("%s: supported dim: min %dx%d max %dx%d", __func__,
    caps.min_width_, caps.min_height_, caps.max_width_, caps.max_height_);

  caps.output_buff_        = 0;
  caps.crop_support_       = false;
  caps.inplace_processing_ = false;
  caps.scale_support_      = true;
  caps.usage_              = 0;

  caps.formats_.clear();
  entry = static_meta_.find(ANDROID_SCALER_AVAILABLE_FORMATS);
  for (uint32_t i = 0; i < entry.count; i++) {
    auto format = Common::FromHalToQmmfFormat(entry.data.i32[i]);
    if (!caps.formats_.count(format)) {
      caps.formats_.insert(format);
      QMMF_VERBOSE("%s: supports format %d", __func__, format);
    }
  }

  return NO_ERROR;
}

status_t CameraHalReproc::Start(const int32_t stream_id) {
  QMMF_INFO("%s: Enter", __func__);
  std::lock_guard<std::mutex> lock(module_lock_);
  if (state_ != PostProcHalState::INITIALIZED &&
      state_ != PostProcHalState::ABORTED) {
    QMMF_ERROR("%s: Failed: Already configured.", __func__);
    return BAD_VALUE;
  }
  //state_ = PostProcHalState::STARTING;

  state_ = PostProcHalState::ACTIVE;

  return NO_ERROR;
}

status_t CameraHalReproc::Stop() {
  QMMF_VERBOSE("%s: Enter ", __func__);

  std::lock_guard<std::mutex> lock(module_lock_);
  if (state_ != PostProcHalState::ACTIVE &&
      state_ != PostProcHalState::ABORTED) {
    QMMF_ERROR("%s: Failed: Wrong state %d.", __func__, state_);
    return BAD_VALUE;
  }
  state_ = PostProcHalState::STOPPING;

  ReturnAllInputBuffers();

  state_ = PostProcHalState::INITIALIZED;

  return NO_ERROR;
}

status_t CameraHalReproc::Abort(std::shared_ptr<void> &abort) {
  QMMF_VERBOSE("%s: Enter ", __func__);

  std::lock_guard<std::mutex> lock(module_lock_);
  if (state_ != PostProcHalState::ACTIVE) {
    QMMF_ERROR("%s: Failed: Wrong state %d.", __func__, state_);
    return BAD_VALUE;
  }

  std::unique_lock<std::mutex> processing_lock(abort_lock_);
  if (frame_processing_) {
    QMMF_VERBOSE("%s: Acquire abort done handler", __func__);
    abort_ = abort;
  }

  state_ = PostProcHalState::ABORTED;

  QMMF_VERBOSE("%s: Exit ", __func__);
  return NO_ERROR;
}

status_t CameraHalReproc::Delete() {
  std::lock_guard<std::mutex> lock(module_lock_);
  if (state_ != PostProcHalState::INITIALIZED) {
    QMMF_ERROR("%s: Failed: Wrong state %d.", __func__, state_);
    return BAD_VALUE;
  }

  auto ret = DeleteDeviceStreams();
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to setup camera: %d\n", __func__, ret);
    return ret;
  }

  state_ = PostProcHalState::CREATED;
  return NO_ERROR;
}

status_t CameraHalReproc::Configure(const std::string config_json_data) {
  return NO_ERROR;
}

status_t CameraHalReproc::Process(
    const std::vector<StreamBuffer> &in_buffers,
    const std::vector<StreamBuffer> &out_buffers) {
  std::lock_guard<std::mutex> lock(module_lock_);
  if (state_ != PostProcHalState::ACTIVE) {
    QMMF_VERBOSE("%s: state is not active %d. skip processing",
        __func__, state_);
    for (auto buf : in_buffers) {
      listener_->OnFrameProcessed(buf);
    }
    return NO_ERROR;
  }

  for (auto buf : in_buffers) {
    AddBuff(buf);
  }

  auto ret = StartProcessing(false);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Failed: Reprocess is not started.\n", __func__);
    return ret;
  }
  return NO_ERROR;
}

status_t CameraHalReproc::ReturnBuff(StreamBuffer &buffer) {
  QMMF_DEBUG("%s: StreamBuffer(0x%p) fd: %d stream_id: %d ts: %lld",
    __func__, buffer.handle, buffer.fd,
    buffer.stream_id, buffer.timestamp);
  return context_->ReturnStreamBuffer(buffer);
}

void CameraHalReproc::AddBuff(const StreamBuffer buf) {
  std::lock_guard<std::mutex> lock(reproc_lock_);

  bool append = true;
  // try to add to existing meta
  if (!reproc_partial_list_.empty()) {
    auto it = reproc_partial_list_.begin();
    auto end = reproc_partial_list_.end();
    while (it != end) {
      if (it->timestamp == buf.timestamp) {
        it->buffer = buf;
        reproc_ready_list_.push_back(*it);
        reproc_partial_list_.erase(it);
        append = false;
        break;
      }
      it++;
    }
  }

  if (append) {
    // meta is missing append to queue directly
    ReprocessBundle new_entry;
    new_entry.buffer = buf;
    new_entry.timestamp = buf.timestamp;
    new_entry.metadata.clear();
    reproc_partial_list_.push_back(new_entry);
  } else {
    // clean up older metadata in partial list
    if (!reproc_partial_list_.empty()) {
      auto it = reproc_partial_list_.begin();
      auto end = reproc_partial_list_.end();
      while (it != end) {
        if (it->timestamp >= buf.timestamp) {
          // clean up only first entries which has lower than buf time stamp
          break;
        }
        it = reproc_partial_list_.erase(it);
      }
    }
  }
}

void CameraHalReproc::AddMeta(const CameraMetadata &metadata) {
  std::lock_guard<std::mutex> lock(reproc_lock_);

  int64_t timestamp;
  if (!metadata.exists(ANDROID_SENSOR_TIMESTAMP)) {
    QMMF_ERROR("%s Sensor timestamp tag missing in metadata!\n",
        __func__);
    return;
  }
  timestamp = metadata.find(ANDROID_SENSOR_TIMESTAMP).data.i64[0];

  // try to add to existing buffer
  bool append = true;
  if (!reproc_partial_list_.empty()) {
    auto it = reproc_partial_list_.begin();
    auto end = reproc_partial_list_.end();
    while (it != end) {
      if (it->timestamp == timestamp) {
        it->metadata.append(metadata);
        reproc_ready_list_.push_back(*it);
        reproc_partial_list_.erase(it);
        append = false;
        break;
      }
      it++;
    }
  }

  if (append) {
    // Buffer is missing append to queue directly
    ReprocessBundle new_entry{};
    new_entry.metadata.append(metadata);
    new_entry.timestamp = timestamp;
    reproc_partial_list_.push_back(new_entry);
  } else {
    // clean up older metadata in partial list
    if (!reproc_partial_list_.empty()) {
      int64_t timeout = timestamp - kMetaTimeout;
      auto it = reproc_partial_list_.begin();
      auto end = reproc_partial_list_.end();
      while (it != end) {
        if (it->timestamp >= timeout) {
          // clean up only first entries which has lower than timeout timestamp
          break;
        }
        it = reproc_partial_list_.erase(it);
      }
    }
  }
}

void CameraHalReproc::AddResult(const void* result_in) {
  std::lock_guard<std::mutex> lock(module_lock_);
  if (state_ != PostProcHalState::ACTIVE) {
    QMMF_VERBOSE("%s: state is not active %d. skip result",
        __func__, state_);
    return;
  }

  const CaptureResult &result =
      *const_cast<CaptureResult*>(static_cast<const CaptureResult*>(result_in));

  AddMeta(result.metadata);

  auto ret = StartProcessing(false);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Failed: Reprocess is not started.\n", __func__);
  }
}

status_t CameraHalReproc::StartProcessing(bool from_cb) {

  std::lock_guard<std::mutex> lock(reproc_lock_);

  if (state_ != PostProcHalState::ACTIVE) {
    std::unique_lock<std::mutex> processing_lock(abort_lock_);
    if (state_ == PostProcHalState::ABORTED && from_cb && frame_processing_) {
      frame_processing_ = false;
      QMMF_VERBOSE("%s: Release abort done handler", __func__);
      abort_ = nullptr;
    }

    QMMF_VERBOSE("%s: state is not active %d. skip processing",
        __func__, state_);
    return NO_ERROR;
  }

  if (from_cb) {
    frame_processing_ = false;
  }

  if (frame_processing_) {
    QMMF_VERBOSE("%s: processing on going", __func__);
    return NO_ERROR;
  }

  ReprocessBundle reproc_bundle;
  // get one reprocess bundle
  if (reproc_ready_list_.empty()) {
    QMMF_DEBUG("%s: no reprocess bundle", __func__);
    return NO_ERROR;
  }

  reproc_bundle = reproc_ready_list_.front();
  reproc_ready_list_.pop_front();

  {
    std::lock_guard<std::mutex> input_bufs_lock(input_buffer_lock_);
    input_buffer_.push_back(reproc_bundle.buffer);
  }

  reprocess_request_.metadata.clear();
  reprocess_request_.metadata.append(reproc_bundle.metadata);

  QMMF_VERBOSE("%s: Submit reprocess request.", __func__);


  int64_t last_frame_number = -1;
  auto ret = context_->SubmitRequest(reprocess_request_,
                                     false,
                                     &last_frame_number);
  if (ret < 0) {
    QMMF_ERROR("%s: Failed to submit reprocess request.", __func__);
    reproc_ready_list_.clear();
    return BAD_VALUE;
  }

  std::unique_lock<std::mutex> processing_lock(abort_lock_);
  frame_processing_ = true;

  return NO_ERROR;
}

status_t CameraHalReproc::CreateDeviceStreams() {
  QMMF_INFO("%s: Enter", __func__);
  status_t ret = NO_ERROR;
  int32_t stream_id_p;

  CameraInputStreamParameters in_stream_params = {};
  in_stream_params.format = Common::FromQmmfToHalFormat(input_param_.format);
  in_stream_params.width = input_param_.width;
  in_stream_params.height = input_param_.height;
  in_stream_params.get_input_buffer = [&] (StreamBuffer &buffer)
      { GetInputBuffer(buffer); };
  in_stream_params.return_input_buffer = [&] (StreamBuffer &buffer)
      { ReturnInputBuffer(buffer); };

  QMMF_VERBOSE("%s: in stream dim %dx%d hal_fmt 0x%x qmmf_fmt %d",
    __func__, output_param_.width, output_param_.height,
    in_stream_params.format, input_param_.format);

  ret = context_->CreateDeviceInputStream(in_stream_params, &stream_id_p);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to create input reprocess stream: %d\n",
        __func__, ret);
    return ret;
  }
  assert(stream_id_p >= 0);
  reprocess_request_.streamIds.add(stream_id_p);

  CameraStreamParameters out_stream_params = {};
  out_stream_params.bufferCount = output_param_.buffer_count;
  out_stream_params.format = Common::FromQmmfToHalFormat(output_param_.format);
  out_stream_params.width = output_param_.width;
  out_stream_params.height = output_param_.height;
  out_stream_params.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  out_stream_params.cb = [&](StreamBuffer buffer)
      { ReprocessCallback(buffer); };
  out_stream_params.is_pp_enabled = true;

  QMMF_VERBOSE("%s: out stream dim %dx%d hal_fmt 0x%x qmmf_fmt %d count %d",
    __func__, output_param_.width, output_param_.height,
    out_stream_params.format, output_param_.format, output_param_.buffer_count);

  ret = context_->CreateDeviceStream(out_stream_params,
                                     output_param_.frame_rate,
                                     &stream_id_p);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to create output reprocess stream: %d\n",
        __func__, ret);
    return ret;
  }
  assert(stream_id_p >= 0);
  reprocess_request_.streamIds.add(stream_id_p);

  return ret;
}


status_t CameraHalReproc::DeleteDeviceStreams() {
  if (!reprocess_request_.streamIds.isEmpty()) {
    for (auto streamId : reprocess_request_.streamIds) {
      if (NO_ERROR != context_->DeleteDeviceStream(streamId, true)) {
        QMMF_ERROR("%s: Failed to delete non-zsl snapshot stream",
          __func__);
        return BAD_VALUE;
      }
    }
    reprocess_request_.streamIds.clear();
  }

  return NO_ERROR;
}

}; // namespace recoder

}; // namespace qmmf
