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

#define TAG "RecorderHalJpeg"

#include "recorder/src/service/qmmf_recorder_utils.h"
#include "recorder/src/service/qmmf_camera_context.h"

#include "qmmf_camera_hal_jpeg.h"

namespace qmmf {

namespace recorder {

PostProcHalJpeg::PostProcHalJpeg(IPostProc* context)
    : context_(context),
      reprocess_flag_(false),
      ready_to_start_(false),
      burst_cnt_(0) {
  QMMF_VERBOSE("%s:%s: Enter", TAG, __func__);
  QMMF_VERBOSE("%s:%s: Exit (0x%p)", TAG, __func__, this);
}

PostProcHalJpeg::~PostProcHalJpeg() {
  QMMF_VERBOSE("%s:%s: Enter ", TAG, __func__);
  QMMF_VERBOSE("%s:%s: Exit (0x%p)", TAG, __func__, this);
}


void PostProcHalJpeg::GetInputBuffer(StreamBuffer &buffer) {
  Mutex::Autolock lock(burst_queue_lock_);
  auto iter = input_buffer_.begin();
  input_buffer_done_.push_back((*iter));
  buffer = (*iter);
  input_buffer_.erase(iter);
}

void PostProcHalJpeg::ReturnInputBuffer(StreamBuffer &buffer) {
  Mutex::Autolock lock(burst_queue_lock_);
  auto iter = input_buffer_done_.begin();
  for (; iter != input_buffer_done_.end(); iter++) {
    if ((*iter).handle ==  buffer.handle) {
      (*iter).stream_id = input_stream_id_;
      auto ret = context_->ReturnStreamBuffer((*iter));
      if (NO_ERROR != ret) {
        QMMF_ERROR("%s: Failed to return input buffer: %d\n", __func__, ret);
      }
      input_buffer_done_.erase(iter);
      break;
    }
  }
}

void PostProcHalJpeg::ReturnAllInputBuffers() {
  Mutex::Autolock lock(burst_queue_lock_);
  auto iter = input_buffer_done_.begin();
  for (; iter != input_buffer_done_.end(); iter++) {
    (*iter).stream_id = input_stream_id_;
    auto ret = context_->ReturnStreamBuffer((*iter));
    if (NO_ERROR != ret) {
      QMMF_ERROR("%s: Failed to return input buffer: %d\n", __func__, ret);
    }
  }
  input_buffer_done_.clear();
  input_buffer_.clear();
  burst_queue_.clear();
  input_burst_queue_.clear();
}

void PostProcHalJpeg::ReprocessCallback(StreamBuffer in_buff) {
  Listener_->OnFrameReady(in_buff);

  //start next frame reprocess
  if (StartProcessing() != NO_ERROR) {
    QMMF_ERROR("%s: Failed: Wrong state. Reprocess is not started.\n",
        __func__);
  }

  // last frame
  if (++burst_cnt_ == output_param_.buffer_count) {
    ReturnAllInputBuffers();
    burst_cnt_= 0;
    reprocess_flag_ = false;
  }
}

status_t PostProcHalJpeg::Initialize(const PostProcIOParam &in_param,
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

PostProcIOParam PostProcHalJpeg::GetInput(const PostProcIOParam &out) {
  PostProcIOParam input_param = out;
  input_param.format =
      Common::FromHalToQmmfFormat(HAL_PIXEL_FORMAT_YCbCr_420_888);
  return input_param;
}

status_t PostProcHalJpeg::ValidateInput(const PostProcIOParam& input,
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

status_t PostProcHalJpeg::ValidateOutput(const PostProcIOParam &output) {
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
        HAL_PIXEL_FORMAT_BLOB == hal_format) {
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

// TODO: extract min/max width/height from camera metadata
status_t PostProcHalJpeg::GetCapabilities(PostProcCaps &caps) {
  caps.output_buff_        = 0;
  caps.min_width_          = 160;
  caps.min_height_         = 120;
  caps.max_width_          = 5104;
  caps.max_height_         = 4092;
  caps.crop_support_       = false;
  caps.scale_support_      = true;
  caps.inplace_processing_ = false;
  caps.usage_              = 0;

  caps.formats_.insert(BufferFormat::kBLOB);

  return NO_ERROR;
}

status_t PostProcHalJpeg::Start(const int32_t stream_id) {
  int32_t ret = NO_ERROR;
  int32_t stream_id_p;

  if (ready_to_start_) {
    QMMF_ERROR("%s:%s: Failed: Already configured.", TAG, __func__);
    return BAD_VALUE;
  }

  if (reprocess_flag_) {
    QMMF_ERROR("%s:%s: Failed: Wrong state.", TAG, __func__);
    return BAD_VALUE;
  }

  Mutex::Autolock lock(reprocess_lock_);

  input_stream_id_ = stream_id;
  burst_cnt_ = 0;

  CameraInputStreamParameters inputStreamParams = {};
  inputStreamParams.format = Common::FromQmmfToHalFormat(input_param_.format);
  inputStreamParams.width = input_param_.width;
  inputStreamParams.height = input_param_.height;
  inputStreamParams.get_input_buffer = [&] (StreamBuffer &buffer)
      { GetInputBuffer(buffer); };
  inputStreamParams.return_input_buffer  = [&] (StreamBuffer &buffer)
      { ReturnInputBuffer(buffer); };
  ret = context_->CreateDeviceInputStream(inputStreamParams, &stream_id_p);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Failed to create input reprocess stream: %d\n",
               __func__, ret);
    return ret;
  }
  assert(stream_id_p >= 0);
  reprocess_request_.streamIds.add(stream_id_p);

  CameraStreamParameters streamParams = {};
  streamParams.bufferCount = output_param_.buffer_count;
  streamParams.format = Common::FromQmmfToHalFormat(output_param_.format);
  streamParams.width = output_param_.width;
  streamParams.height = output_param_.height;
  streamParams.grallocFlags = GRALLOC_USAGE_SW_READ_OFTEN;
  streamParams.cb = [&](StreamBuffer buffer) { ReprocessCallback(buffer); };
  ret = context_->CreateDeviceStream(streamParams, output_param_.frame_rate,
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

status_t PostProcHalJpeg::Stop() {
  return NO_ERROR;
}

status_t PostProcHalJpeg::StartProcessing() {
  if (!ready_to_start_) {
    return BAD_VALUE;
  }

  assert(context_ != nullptr);

  Mutex::Autolock lock(burst_queue_lock_);

  reprocess_flag_ = true;

  if (!input_burst_queue_.empty()) {
    List<BurstData>::iterator it = input_burst_queue_.begin();
    input_buffer_.push_back(it->buffer);
    reprocess_request_.metadata.clear();
    reprocess_request_.metadata.append(it->result);
    input_burst_queue_.erase(it);
  } else {
    QMMF_ERROR("%s:%s: Buffer is not ready or no more buffers", TAG, __func__);
    return NO_ERROR;
  }
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

status_t PostProcHalJpeg::Delete() {
  QMMF_VERBOSE("%s:%s: Enter ", TAG, __func__);

  assert(context_ != nullptr);

  Mutex::Autolock lock(reprocess_lock_);

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
  reprocess_flag_ = false;
  ready_to_start_ = false;
  return NO_ERROR;
}

status_t PostProcHalJpeg::Configure(const std::string config_json_data) {
  return NO_ERROR;
}

status_t PostProcHalJpeg::Process(
    const std::vector<StreamBuffer> &in_buffers,
    const std::vector<StreamBuffer> &out_buffers) {

  for (auto buf : in_buffers) {
    AddBuff(buf);
  }
  return NO_ERROR;
}

void PostProcHalJpeg::AddBuff(StreamBuffer in_buff) {
  {
    Mutex::Autolock lock(burst_queue_lock_);
    bool append = true;
    if (!burst_queue_.empty()) {
      List<BurstData>::iterator it = burst_queue_.begin();
      List<BurstData>::iterator end = burst_queue_.end();
      while (it != end) {
        if (it->timestamp == in_buff.timestamp) {
          it->buffer = in_buff;
          input_burst_queue_.push_back(*it);
          burst_queue_.erase(it);
          append = false;
          break;
        }
        it++;
      }
    }

    if (append) {
      //Result is missing append to queue directly
      BurstData new_entry;
      new_entry.buffer = in_buff;
      new_entry.timestamp = in_buff.timestamp;
      new_entry.result.clear();
      burst_queue_.push_back(new_entry);
    }
  }

  if (input_burst_queue_.size() >= output_param_.buffer_count) {
    if (StartProcessing() != NO_ERROR) {
      QMMF_ERROR("%s: Failed: Wrong state. Reprocess is not started.\n",
          __func__);
    }
  }
}

status_t PostProcHalJpeg::ReturnBuff(StreamBuffer &buffer) {
  QMMF_VERBOSE("%s:%s: StreamBuffer(0x%p) ts: %lld", TAG,
       __func__, buffer.handle, buffer.timestamp);
  return context_->ReturnStreamBuffer(buffer);
}

void PostProcHalJpeg::AddResult(const void* result_in) {
  int64_t timestamp;
  const CaptureResult &result = *const_cast<CaptureResult*>(
                                static_cast<const CaptureResult*>(result_in));

  if (result.metadata.exists(ANDROID_SENSOR_TIMESTAMP)) {
    timestamp = result.metadata.find(ANDROID_SENSOR_TIMESTAMP).data.i64[0];
  } else {
    QMMF_ERROR("%s:%s Sensor timestamp tag missing in result!\n",
        TAG, __func__);
    return;
  }

  {
    Mutex::Autolock lock(burst_queue_lock_);
    bool append = true;
    if (!burst_queue_.empty()) {
      List<BurstData>::iterator it = burst_queue_.begin();
      List<BurstData>::iterator end = burst_queue_.end();
      while (it != end) {
        if (it->timestamp == timestamp) {
          it->result.append(result.metadata);
          input_burst_queue_.push_back(*it);
          burst_queue_.erase(it);
          append = false;
          break;
        }
        it++;
      }
    }

    if (append) {
      //Buffer is missing append to queue directly
      BurstData new_entry;
      new_entry.result.append(result.metadata);
      new_entry.timestamp = timestamp;
      memset(&new_entry.buffer, 0, sizeof(new_entry.buffer));
      burst_queue_.push_back(new_entry);
    }
  }

  if (input_burst_queue_.size() >= output_param_.buffer_count) {
    if (StartProcessing() != NO_ERROR) {
      QMMF_ERROR("%s: Failed: Wrong state. Reprocess is not started.\n",
          __func__);
    }
  }
}

}; // namespace recoder

}; // namespace qmmf
