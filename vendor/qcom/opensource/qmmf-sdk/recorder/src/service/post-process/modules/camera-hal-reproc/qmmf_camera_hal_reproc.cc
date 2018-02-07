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

#define TAG "RecorderHALReproc"

#include "recorder/src/service/qmmf_recorder_utils.h"
#include "recorder/src/service/qmmf_camera_context.h"

#include "qmmf_camera_hal_reproc.h"

namespace qmmf {

namespace recorder {

CameraHalReproc::CameraHalReproc(IPostProc* context)
    : context_(context),
      ready_to_start_(false) {
  QMMF_VERBOSE("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

CameraHalReproc::~CameraHalReproc() {
  QMMF_VERBOSE("%s:%s: Enter ", TAG, __func__);
  QMMF_VERBOSE("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

void CameraHalReproc::GetInputBuffer(StreamBuffer &buffer) {
  std::lock_guard<std::mutex> lock(reproc_lock_);
  auto iter = input_buffer_.begin();
  input_buffer_done_.push_back((*iter));
  buffer = (*iter);
  input_buffer_.erase(iter);
}

void CameraHalReproc::ReturnInputBuffer(StreamBuffer &buffer) {
  std::lock_guard<std::mutex> lock(reproc_lock_);
  auto iter = input_buffer_done_.begin();
  for (; iter != input_buffer_done_.end(); iter++) {
    if ((*iter).handle ==  buffer.handle) {
      (*iter).stream_id = input_stream_id_;
      listener_->OnFrameProcessed(*iter);
      input_buffer_done_.erase(iter);
      break;
    }
  }
}

void CameraHalReproc::ReturnAllInputBuffers() {
  std::lock_guard<std::mutex> lock(reproc_lock_);
  auto iter = input_buffer_done_.begin();
  for (; iter != input_buffer_done_.end(); iter++) {
    (*iter).stream_id = input_stream_id_;
    listener_->OnFrameProcessed(*iter);
  }
  input_buffer_done_.clear();
  input_buffer_.clear();
  reproc_partial_list_.clear();
  reproc_ready_list_.clear();
}

void CameraHalReproc::ReprocessCallback(StreamBuffer in_buff) {
  QMMF_INFO("%s:%s: HAL reprocess is done! StreamBuffer(0x%p) fd: %d stream_id:"
      "%d ts: %lld", TAG,  __func__, in_buff.handle, in_buff.fd,
      in_buff.stream_id, in_buff.timestamp);
  listener_->OnFrameReady(in_buff);
}

status_t CameraHalReproc::Initialize(const PostProcIOParam &in_param,
                                     const PostProcIOParam &out_param) {
  auto ret = ValidateInput(in_param, out_param);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s:%s: Failed: Wrong input parameters.", TAG, __func__);
    return BAD_VALUE;
  }

  input_param_ = in_param;
  output_param_ = out_param;
  return NO_ERROR;
}

PostProcIOParam CameraHalReproc::GetInput(const PostProcIOParam &out) {
  PostProcIOParam input_param = out;

  CameraMetadata meta = context_->GetCameraStaticMeta();

  camera_metadata_entry_t entry;
  if (!meta.exists(ANDROID_SCALER_AVAILABLE_FORMATS)) {
    QMMF_ERROR("%s: Hal does not report supported formats\n", __func__);
    assert(0);
  }

  if (!meta.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
    QMMF_ERROR("%s: Hal does not report supported sizes\n", __func__);
    assert(0);
  }

  entry = meta.find(ANDROID_SCALER_AVAILABLE_FORMATS);
  uint32_t i;
  for (i = 0; i < entry.count; i++) {
    if (entry.data.i32[i] == HAL_PIXEL_FORMAT_RAW16) {
      input_param.format = Common::FromHalToQmmfFormat(entry.data.i32[i]);
      break;
    }
  }
  if (i == entry.count) {
    QMMF_ERROR("%s: Hal does not support required format\n", __func__);
    assert(0);
  }

  entry = meta.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES);
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
      __func__, out.width, out.height, input_param.width, input_param.height);
    assert(0);
  }

  QMMF_VERBOSE("%s:%s: input dim %dx%d format %x RAW10 %x RAW12 %x RAW16 %x",
    TAG, __func__, input_param.width, input_param.height, input_param.format,
    BufferFormat::kRAW10, BufferFormat::kRAW12, BufferFormat::kRAW16);

  return input_param;
}

status_t CameraHalReproc::ValidateInput(const PostProcIOParam& input,
                                        const PostProcIOParam& output) {
  camera_metadata_entry_t entry;
  int32_t in_format, num_output_formats;

  CameraMetadata static_meta = context_->GetCameraStaticMeta();

  if (static_meta.exists(ANDROID_SCALER_AVAILABLE_INPUT_OUTPUT_FORMATS_MAP)) {
    entry = static_meta.find(ANDROID_SCALER_AVAILABLE_INPUT_OUTPUT_FORMATS_MAP);
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

  QMMF_ERROR("%s: Failed: input format: 0x%x out format 0x%x\n",  __func__,
      input.format, output.format);

  return BAD_VALUE;
}

status_t CameraHalReproc::ValidateOutput(const PostProcIOParam &output) {
  CameraMetadata meta = context_->GetCameraStaticMeta();
  int32_t hal_format = Common::FromQmmfToHalFormat(output.format);
  bool supported = false;

  camera_metadata_entry_t entry;
  if (!meta.exists(ANDROID_SCALER_AVAILABLE_FORMATS)) {
    QMMF_ERROR("%s: HAL does not report supported formats!", __func__);
    return NAME_NOT_FOUND;
  }

  if (!meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
    QMMF_ERROR("%s: HAL does not report supported sizes!", __func__);
    return NAME_NOT_FOUND;
  }

  entry = meta.find(ANDROID_SCALER_AVAILABLE_FORMATS);
  for (uint32_t i = 0 ; i < entry.count; i++) {
    if (entry.data.i32[i] == hal_format &&
        HAL_PIXEL_FORMAT_YCbCr_420_888 == hal_format) {
      supported = true;
      break;
    }
  }

  if (!supported) {
    QMMF_ERROR("%s: Output format(%d) not supported", __func__, output.format);
    return BAD_TYPE;
  }
  supported = false;

  uint32_t w = 0, h = 0, scalar_format = 0, config_type = 0;
  entry = meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
  for (uint32_t i = 0 ; i < entry.count; i += 4) {
    scalar_format = entry.data.i32[i];
    config_type = entry.data.i32[i+3];
    if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == scalar_format &&
        ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT == config_type) {
      w = entry.data.i32[i+1];
      h = entry.data.i32[i+2];
      if(w == output.width && h == output.height) {
        supported = true;
        break;
      }
    }
  }

  if (!supported) {
    QMMF_ERROR("%s: Output dimensions(%dx%d) not supported", __func__,
        output.width, output.height);
    return BAD_VALUE;
  }

  return NO_ERROR;
}

status_t CameraHalReproc::GetCapabilities(PostProcCaps &caps) {
  CameraMetadata static_meta = context_->GetCameraStaticMeta();

  if (!static_meta.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
    QMMF_ERROR("%s: HAL does not report supported sizes!", __func__);
    return NAME_NOT_FOUND;
  }

  camera_metadata_entry_t entry;
  entry = static_meta.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES);
  for (uint32_t i = 0 ; i < entry.count; i += 2) {
    caps.min_width_  = entry.data.i32[i+0];
    caps.min_height_ = entry.data.i32[i+1];
    caps.max_width_  = entry.data.i32[i+0];
    caps.max_height_ = entry.data.i32[i+1];
  }

  caps.output_buff_        = 0;
  caps.crop_support_       = false;
  caps.inplace_processing_ = false;
  caps.scale_support_      = true;
  caps.usage_              = 0;

  caps.formats_.insert(BufferFormat::kNV12);
  caps.formats_.insert(BufferFormat::kNV21);

  return NO_ERROR;
}

status_t CameraHalReproc::Start(const int32_t stream_id) {
  QMMF_INFO("%s:%s: Enter", TAG, __func__);
  status_t ret = NO_ERROR;
  int32_t stream_id_p;

  if (ready_to_start_) {
    QMMF_ERROR("%s:%s: Failed: Already configured.", TAG, __func__);
    return BAD_VALUE;
  }

  std::lock_guard<std::mutex> lock(module_lock_);

  input_stream_id_ = stream_id;

  CameraInputStreamParameters in_stream_params = {};
  in_stream_params.format = Common::FromQmmfToHalFormat(input_param_.format);
  in_stream_params.width = input_param_.width;
  in_stream_params.height = input_param_.height;
  in_stream_params.get_input_buffer = [&] (StreamBuffer &buffer)
      { GetInputBuffer(buffer); };
  in_stream_params.return_input_buffer = [&] (StreamBuffer &buffer)
      { ReturnInputBuffer(buffer); };
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
  ret = context_->CreateDeviceStream(out_stream_params,
                                     output_param_.frame_rate,
                                     &stream_id_p, true);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to create output reprocess stream: %d\n",
               __func__, ret);
    return ret;
  }
  assert(stream_id_p >= 0);
  reprocess_request_.streamIds.add(stream_id_p);

  ready_to_start_ = true;

  return ret;
}

status_t CameraHalReproc::Stop() {
  QMMF_VERBOSE("%s:%s: Enter ", TAG, __func__);

  assert(context_ != nullptr);

  std::lock_guard<std::mutex> lock(module_lock_);

  ReturnAllInputBuffers();

  if (!reprocess_request_.streamIds.isEmpty()) {
    for (auto streamId : reprocess_request_.streamIds) {
      if (NO_ERROR != context_->DeleteDeviceStream(streamId, false)) {
        QMMF_ERROR("%s: Failed to delete non-zsl snapshot stream",
                   __func__);
        return BAD_VALUE;
      }
    }
    reprocess_request_.streamIds.clear();
  }
  ready_to_start_ = false;
  return NO_ERROR;
}

status_t CameraHalReproc::Delete() {
  return NO_ERROR;
}

status_t CameraHalReproc::Configure(const std::string config_json_data) {
  return NO_ERROR;
}

status_t CameraHalReproc::Process(
    const std::vector<StreamBuffer> &in_buffers,
    const std::vector<StreamBuffer> &out_buffers) {

  for (auto buf : in_buffers) {
    AddBuff(buf);
  }

  auto ret = StartProcessing();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: Failed: Reprocess is not started.\n", TAG, __func__);
    return ret;
  }
  return NO_ERROR;
}

status_t CameraHalReproc::ReturnBuff(StreamBuffer &buffer) {
  QMMF_DEBUG("%s:%s: StreamBuffer(0x%p) fd: %d stream_id: %d ts: %lld", TAG,
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
    QMMF_ERROR("%s:%s Sensor timestamp tag missing in metadata!\n",
        TAG, __func__);
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
    ReprocessBundle new_entry;
    new_entry.metadata.append(metadata);
    new_entry.timestamp = timestamp;
    memset(&new_entry.buffer, 0, sizeof(new_entry.buffer));
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
  const CaptureResult &result =
      *const_cast<CaptureResult*>(static_cast<const CaptureResult*>(result_in));

  AddMeta(result.metadata);

  auto ret = StartProcessing();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s:%s: Failed: Reprocess is not started.\n", TAG, __func__);
  }
}

status_t CameraHalReproc::StartProcessing() {

  if (!ready_to_start_) {
    QMMF_ERROR("%s:%s: Bad state.", TAG, __func__);
    return BAD_VALUE;
  }

  std::lock_guard<std::mutex> lock(reproc_lock_);
  if (reproc_ready_list_.empty()) {
    QMMF_DEBUG("%s:%s: no reprocess bundle", TAG, __func__);
    return NO_ERROR;
  }

  auto it = reproc_ready_list_.begin();
  input_buffer_.push_back(it->buffer);
  reprocess_request_.metadata.clear();
  reprocess_request_.metadata.append(it->metadata);
  reproc_ready_list_.erase(it);

  int64_t last_frame_mumber;
  auto ret = context_->SubmitRequest(reprocess_request_,
                                     false,
                                     &last_frame_mumber);
  if (ret < 0) {
    QMMF_ERROR("%s:%s: Failed to submit reprocess request.", TAG, __func__);
    return BAD_VALUE;
  }

  return NO_ERROR;
}

}; // namespace recoder

}; // namespace qmmf
