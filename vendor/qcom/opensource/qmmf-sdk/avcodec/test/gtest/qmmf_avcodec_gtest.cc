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

#define TAG "CodecGTest"

#include <memory>

#include "avcodec/test/gtest/qmmf_avcodec_gtest.h"

using ::std::shared_ptr;
using ::std::make_shared;

static const int32_t kIterationCount = 50;
static const int32_t kRecordDuration = 2; // 2 min for each iteration.

void CodecGtest::SetUp() {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);

  test_info_ = ::testing::UnitTest::GetInstance()->current_test_info();

  avcodec_ = IAVCodec::CreateAVCodec();
  assert(avcodec_ != nullptr);

  ion_device_ = open("/dev/ion", O_RDONLY);
  assert(ion_device_ > 0);

  QMMF_INFO("%s:%s Exit ", TAG, __func__);
}

status_t CodecGtest::CreateCodec() {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);

  CodecParam param;
  memset(&param, 0x0, sizeof(param));

  param.video_enc_param.width = 1280;
  param.video_enc_param.height = 720;
  param.video_enc_param.frame_rate = 30;
  param.video_enc_param.format_type = VideoFormat::kAVC;
  param.video_enc_param.codec_param.avc.idr_interval = 1;
  param.video_enc_param.codec_param.avc.bitrate      = 10000000;
  param.video_enc_param.codec_param.avc.profile = AVCProfileType::kBaseline;
  param.video_enc_param.codec_param.avc.level   = AVCLevelType::kLevel3;
  param.video_enc_param.codec_param.avc.ratecontrol_type =
      VideoRateControlType::kConstant;

  auto ret = avcodec_->ConfigureCodec(CodecMimeType::kMimeTypeVideoEncAVC,
                                      param);
  if(ret != OK) {
    QMMF_ERROR("%s:%s Failed to configure Codec",  TAG, __func__);
    return ret;
  }

  ret = AllocateBuffer(kPortIndexInput);
  if(ret != OK) {
    QMMF_ERROR("%s:%s Failed to allocate buffer",  TAG, __func__);
    return ret;
  }

  input_source_impl_= make_shared<InputCodecSourceImpl>();
  if(input_source_impl_.get() == nullptr) {
    QMMF_ERROR("%s:%s Failed to create input source",  TAG, __func__);
    return NO_MEMORY;
  }

  ret = avcodec_->AllocateBuffer(kPortIndexInput, 0, 0,
                                 shared_ptr<ICodecSource>(input_source_impl_),
                                 input_buffer_list_);
  if(ret != OK) {
    QMMF_ERROR("%s:%s Failed to allocate omx buffer on %s",  TAG, __func__,
        PORT_NAME(kPortIndexInput));
    return ret;
  }

  input_source_impl_->AddBufferList(input_buffer_list_);

  ret = AllocateBuffer(kPortIndexOutput);
  if(ret != OK) {
    QMMF_ERROR("%s:%s Failed to allocate buffer",  TAG, __func__);
    return ret;
  }

  output_source_impl_ = make_shared<OutputCodecSourceImpl>();
  if(input_source_impl_.get() == nullptr) {
    QMMF_ERROR("%s:%s Failed to create output source",  TAG, __func__);
    return NO_MEMORY;
  }

  ret = avcodec_->AllocateBuffer(kPortIndexOutput, 0, 0,
                                 shared_ptr<ICodecSource>(output_source_impl_),
                                 output_buffer_list_);
  if(ret != OK) {
    QMMF_ERROR("%s:%s Failed to use omx buffer on %s",  TAG, __func__,
        PORT_NAME(kPortIndexOutput));
    return ret;
  }

  output_source_impl_->AddBufferList(output_buffer_list_);

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t CodecGtest::AllocateBuffer(uint32_t index) {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);
  status_t ret = 0;

  assert(ion_device_ > 0);
  int32_t ionType =  ION_HEAP(ION_IOMMU_HEAP_ID);

  uint32_t count, size;
  ret = avcodec_->GetBufferRequirements(index,  &count, &size);
  if(ret != OK) {
    QMMF_INFO("%s:%s Failed to get Buffer Requirements on %s", TAG, __func__,
        PORT_NAME(index));
    return ret;
  }

  struct ion_allocation_data alloc;
  struct ion_fd_data ionFdData;

  if(index == kPortIndexInput) {
    int nFds = 1;
    int nInts = 3;
    count = INPUT_MAX_COUNT;

    for(uint32_t i = 0; i < count; i++) {
      BufferDescriptor buffer;
      memset(&buffer, 0x0, sizeof(buffer));
      memset(&alloc, 0x0, sizeof(ion_allocation_data));
      memset(&ionFdData, 0x0, sizeof(ion_fd_data));

      alloc.len = size;
      alloc.len = (alloc.len + 4095) & (~4095);
      alloc.align = 4096;
      alloc.flags = ION_FLAG_CACHED;
      alloc.heap_id_mask = ionType;

      ret = ioctl(ion_device_, ION_IOC_ALLOC, &alloc);
      if (ret < 0) {
        QMMF_ERROR("%s:%s ION allocation failed", TAG, __func__);
        goto ION_ALLOC_FAILED;
      }

      ionFdData.handle = alloc.handle;
      ret = ioctl(ion_device_, ION_IOC_SHARE, &ionFdData);
      if (ret < 0) {
        QMMF_ERROR("%s:%s ION map failed %s", TAG, __func__, strerror(errno));
        goto ION_MAP_FAILED;
      }

      input_ion_handle_data.push_back(alloc);

      //Allocate buffer for MetaData Handle
      native_handle_t* meta_handle = (native_handle_create(1, 16));
      if(meta_handle == nullptr) {
        QMMF_ERROR("%s:%s failed to allocated metabuffer handle", TAG, __func__);
        return NO_MEMORY;
      }

      meta_handle->version = sizeof(native_handle_t);
      meta_handle->numFds  = nFds;
      meta_handle->numInts = nInts;
      meta_handle->data[0] = ionFdData.fd;
      meta_handle->data[1] = 0; //offset
      meta_handle->data[4] = alloc.len;
      buffer.data = meta_handle;

      QMMF_INFO("%s:%s  buffer native handle = %p", TAG, __func__, meta_handle);
      QMMF_INFO("%s:%s  buffer ionFd = %d", TAG, __func__, meta_handle->data[0]);
      QMMF_INFO("%s:%s  buffer frameLen = %d",TAG,__func__,meta_handle->data[4]);
      input_buffer_list_.push_back(buffer);
    }
  } else {
    count = OUTPUT_MAX_COUNT;
    void *vaddr = nullptr;

    for(uint32_t i = 0; i < count; i++) {
      BufferDescriptor buffer;
      vaddr = nullptr;

      memset(&buffer, 0x0, sizeof(buffer));
      memset(&alloc, 0x0, sizeof(ion_allocation_data));
      memset(&ionFdData, 0x0, sizeof(ion_fd_data));

      alloc.len = size;
      alloc.len = (alloc.len + 4095) & (~4095);
      alloc.align = 4096;
      alloc.flags = ION_FLAG_CACHED;
      alloc.heap_id_mask = ionType;

      ret = ioctl(ion_device_, ION_IOC_ALLOC, &alloc);
      if (ret < 0) {
        QMMF_ERROR("%s:%s ION allocation failed", TAG, __func__);
        goto ION_ALLOC_FAILED;
      }

      ionFdData.handle = alloc.handle;
      ret = ioctl(ion_device_, ION_IOC_SHARE, &ionFdData);
      if (ret < 0) {
        QMMF_ERROR("%s:%s ION map failed %s", TAG, __func__, strerror(errno));
        goto ION_MAP_FAILED;
      }

      vaddr = mmap(nullptr, alloc.len, PROT_READ  | PROT_WRITE, MAP_SHARED,
                  ionFdData.fd, 0);
      if(vaddr == MAP_FAILED) {
        QMMF_ERROR("%s:%s  ION mmap failed: %s (%d)", TAG, __func__,
            strerror(errno), errno);
        goto ION_MAP_FAILED;
      }

      output_ion_handle_data.push_back(alloc);

      buffer.fd       = ionFdData.fd;
      buffer.capacity = alloc.len;
      buffer.size     = alloc.len;
      buffer.data     = vaddr;

      QMMF_INFO("%s:%s buffer.Fd = %d", TAG, __func__, buffer.fd );
      QMMF_INFO("%s:%s buffer.capacity = %d", TAG,__func__, buffer.capacity);
      QMMF_INFO("%s:%s buffer.vaddr = %p", TAG, __func__, buffer.data);
      output_buffer_list_.push_back(buffer);
    }
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;

ION_MAP_FAILED:
  struct ion_handle_data ionHandleData;
  memset(&ionHandleData, 0x0, sizeof(ionHandleData));
  ionHandleData.handle = ionFdData.handle;
  ioctl(ion_device_, ION_IOC_FREE, &ionHandleData);
ION_ALLOC_FAILED:
  close(ion_device_);
  ion_device_ = -1;
  QMMF_ERROR("%s:%s ION Buffer allocation failed!", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return -1;
}

status_t CodecGtest::ReleaseBuffer() {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);

  for(auto& iter : input_ion_handle_data) {
      ioctl(ion_device_, ION_IOC_FREE, &iter);
  }

  for(auto& iter: input_buffer_list_) {
    native_handle_t* meta_handle = reinterpret_cast<native_handle_t*>((iter).data);
    if(meta_handle->data[0]) {
      close(meta_handle->data[0]);
      meta_handle->data[0] = -1;
      native_handle_delete(meta_handle);
      meta_handle = nullptr;
    }
  }

  int i = 0;
  for(auto& iter : output_buffer_list_) {
      if((iter).data) {
          munmap((iter).data, (iter).capacity);
          (iter).data = nullptr;
      }
      if((iter).fd) {
          ioctl(ion_device_, ION_IOC_FREE, &(output_ion_handle_data[i]));
          close((iter).fd);
          (iter).fd = -1;
      }
      ++i;
  }

  input_buffer_list_.clear();
  output_buffer_list_.clear();
  input_ion_handle_data.clear();
  output_ion_handle_data.clear();

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return 0;
}

status_t CodecGtest::DeleteCodec() {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  status_t ret = 0;

  ret = avcodec_->ReleaseBuffer();
  assert(ret == OK);

  ret = ReleaseBuffer();
  assert(ret == OK);

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

void CodecGtest::TearDown() {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);
  if(avcodec_)
    delete avcodec_;

  close(ion_device_);
  ion_device_ = -1;

  QMMF_INFO("%s:%s Exit ", TAG, __func__);
}

InputCodecSourceImpl::InputCodecSourceImpl() {

  QMMF_INFO("%s:%s  Enter",TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

InputCodecSourceImpl::~InputCodecSourceImpl() {

  QMMF_INFO("%s:%s  Enter", TAG, __func__);
  QMMF_INFO("%s:%s  Exit", TAG, __func__);
}

void InputCodecSourceImpl::AddBufferList(vector<BufferDescriptor>& list) {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);

  input_list_ = list;
  input_free_buffer_queue_.Clear();
  input_occupy_buffer_queue_.Clear();
  for(auto& iter : input_list_) {
    input_free_buffer_queue_.PushBack(iter);
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

status_t InputCodecSourceImpl::NotifyPortEvent(PortEventType event_type,
                                               void* event_data)  {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return 0;
}

status_t InputCodecSourceImpl::GetBuffer(BufferDescriptor& stream_buffer,
                                         void* client_data) {

  status_t ret = 0;

  static uint64_t time_stamp = 0;

  if(input_free_buffer_queue_.Size() <= 0) {
    QMMF_WARN("%s:%s No buffer available on %s. Wait for new buffer", TAG,
        __func__, PORT_NAME(kPortIndexInput));
    Mutex::Autolock autoLock(wait_for_frame_lock_);
    wait_for_frame_.wait(wait_for_frame_lock_);
  }

  BufferDescriptor buffer = *input_free_buffer_queue_.Begin();
  assert(buffer.data != nullptr);

  stream_buffer.data = buffer.data;

  input_occupy_buffer_queue_.PushBack(buffer);
  input_free_buffer_queue_.Erase(input_free_buffer_queue_.Begin());

  time_stamp = time_stamp + (uint64_t)(1000000 / 30);
  stream_buffer.timestamp = time_stamp;

  return ret;
}

status_t InputCodecSourceImpl::ReturnBuffer(BufferDescriptor& buffer,
                                            void* client_data) {

  status_t ret = 0;

  bool found = false;
  List<BufferDescriptor>::iterator it = input_occupy_buffer_queue_.Begin();
  for (; it != input_occupy_buffer_queue_.End(); ++it) {
    if ((*it).data ==  buffer.data) {
      input_free_buffer_queue_.PushBack(*it);
      wait_for_frame_.signal();
      found = true;
      break;
    }
  }
  assert(found == true);
  input_occupy_buffer_queue_.Erase(it);

  return ret;
}

void InputCodecSourceImpl::BufferStatus() {

  QMMF_INFO("%s:%s Total Buffer(%d), free(%d), occupy(%d)", TAG, __func__,
      input_list_.size(), input_free_buffer_queue_.Size(),
      input_occupy_buffer_queue_.Size());
  assert(input_occupy_buffer_queue_.Size() == 0);
}

OutputCodecSourceImpl::OutputCodecSourceImpl() {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

OutputCodecSourceImpl::~OutputCodecSourceImpl() {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

void OutputCodecSourceImpl::AddBufferList(vector<BufferDescriptor>& list) {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);

  output_list_ = list;
  output_free_buffer_queue_.Clear();
  output_occupy_buffer_queue_.Clear();

  for(auto& iter : output_list_) {
    output_free_buffer_queue_.PushBack(iter);
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

status_t OutputCodecSourceImpl::NotifyPortEvent(PortEventType event_type,
                                                void* event_data)  {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return 0;
}

status_t OutputCodecSourceImpl::GetBuffer(BufferDescriptor& codec_buffer,
                                          void* client_data) {

  status_t ret = 0;

  if(output_free_buffer_queue_.Size() <= 0) {
    QMMF_WARN("%s:%s No buffer available on %s. Wait for new buffer", TAG,
        __func__, PORT_NAME(kPortIndexOutput));
    Mutex::Autolock autoLock(wait_for_frame_lock_);
    wait_for_frame_.wait(wait_for_frame_lock_);
  }

  BufferDescriptor iter = *output_free_buffer_queue_.Begin();
  codec_buffer.fd = (iter).fd;
  codec_buffer.data = (iter).data;
  output_occupy_buffer_queue_.PushBack(iter);
  output_free_buffer_queue_.Erase(output_free_buffer_queue_.Begin());

  return ret;
}

status_t OutputCodecSourceImpl::ReturnBuffer(BufferDescriptor& codec_buffer,
                                             void* client_data) {

  status_t ret = 0;

  assert(codec_buffer.data != nullptr);
  if(codec_buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS)) {
    QMMF_INFO("%s:%s This is last buffer from encoder.Close file", TAG,__func__);
  }

  List<BufferDescriptor>::iterator it = output_occupy_buffer_queue_.Begin();
  bool found = false;
  for (; it != output_occupy_buffer_queue_.End(); ++it) {
    if (((*it).data) ==  (codec_buffer.data)) {
      output_free_buffer_queue_.PushBack(*it);
      output_occupy_buffer_queue_.Erase(it);
      wait_for_frame_.signal();
      found = true;
      break;
    }
  }

  assert(found == true);
  return ret;
}

void OutputCodecSourceImpl::BufferStatus() {

  QMMF_INFO("%s:%s Total Buffer(%d), free(%d), occupy(%d)", TAG, __func__,
      output_list_.size(), output_free_buffer_queue_.Size(),
      output_occupy_buffer_queue_.Size());
  assert(output_occupy_buffer_queue_.Size() == 0);
}

// StartStopCodec: This test case will test Start & Stop Codec Api.
TEST_F(CodecGtest, StartStopCodec) {

  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = CreateCodec();
  assert(ret == OK);

  for(uint32_t i = 1; i <= kIterationCount; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, kIterationCount);
    QMMF_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    ret = avcodec_->StartCodec();
    assert(ret == OK);
    sleep(kRecordDuration*60);

    ret = avcodec_->StopCodec();
    assert(ret == OK);

    input_source_impl_->BufferStatus();
    output_source_impl_->BufferStatus();
  }
  ret = DeleteCodec();
  assert(ret == OK);
  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

// CreateDeleteStopCodec: This test case will test Create->Start->Stop->Delete
// Codec Api.
TEST_F(CodecGtest, CreateDeleteCodec) {

  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  for(uint32_t i = 1; i <= kIterationCount; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, kIterationCount);
    QMMF_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);

    auto ret = CreateCodec();
    assert(ret == OK);

    ret = avcodec_->StartCodec();
    assert(ret == OK);
    sleep(kRecordDuration*60);

    ret = avcodec_->StopCodec();
    assert(ret == OK);

    input_source_impl_->BufferStatus();
    output_source_impl_->BufferStatus();

    ret = DeleteCodec();
    assert(ret == OK);
  }
  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}
