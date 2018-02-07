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

#include "qmmf_transcode_params.h"

#define TAG "TranscodeBuffer"

namespace qmmf {
namespace transcode {

using ::qmmf::avcodec::CodecParam;
using ::qmmf::avcodec::IAVCodec;
using ::qmmf::avcodec::kPortALL;
using ::qmmf::avcodec::kPortIndexInput;
using ::qmmf::avcodec::kPortIndexOutput;
using ::qmmf::avcodec::PortEventType;
using ::qmmf::avcodec::PortreconfigData;
using ::qmmf::player::VideoTrackCreateParam;
using ::std::lock_guard;
using ::std::make_shared;
using ::std::mutex;
using ::std::shared_ptr;
using ::std::static_pointer_cast;
using ::std::string;
using ::std::unique_lock;
using ::std::vector;

int32_t TranscodeBuffer::ion_device_ = -1;
TSInt32 TranscodeBuffer::num_instances_;

TranscodeBuffer& TranscodeBuffer::operator=(const TranscodeBuffer& rhs) {
  meta_handle_  = rhs.meta_handle_;
  owner_        = rhs.owner_;
  buf_info_     = rhs.buf_info_;
  buf_id_       = rhs.buf_id_;
  timestamp_    = rhs.timestamp_;
  flag_         = rhs.flag_;
  filled_size_  = rhs.filled_size_;
  offset_       = rhs.offset_;

  return *this;
}

TranscodeBuffer::TranscodeBuffer(const TranscodeBuffer& obj) {
  meta_handle_  = obj.meta_handle_;
  owner_        = obj.owner_;
  buf_info_     = obj.buf_info_;
  buf_id_       = obj.buf_id_;
  timestamp_    = obj.timestamp_;
  flag_         = obj.flag_;
  filled_size_  = obj.filled_size_;
  offset_       = obj.offset_;

  num_instances_++;
}

TranscodeBuffer::TranscodeBuffer(const BufferOwner owner,
                                 const uint32_t buf_id = 0)
    : owner_(owner), buf_id_(buf_id) {
  memset(&buf_info_, 0x0, sizeof(buf_info_));
  buf_info_.fd = -1;
  meta_handle_ = nullptr;
  if (num_instances_.value() == 0 && ion_device_ < 0) {
    ion_device_ = open("/dev/ion", O_RDONLY);
    if (ion_device_ < 0) {
      QMMF_ERROR("%s:%s Ion dev open failed %s", TAG, __func__,
                 strerror(errno));
      ion_device_ = -1;
      assert(0);
    } else {
      QMMF_INFO("%s:%s Ion dev open success %d", TAG, __func__, ion_device_);
    }
  }
  num_instances_++;
}

TranscodeBuffer::~TranscodeBuffer() {
  num_instances_--;
  if (num_instances_.value() <= 0) {
    close(ion_device_);
    ion_device_ = -1;
    QMMF_INFO("%s:%s Ion Device Closed", TAG, __func__);
  }
}

status_t TranscodeBuffer::Allocate(const uint32_t size) {
  QMMF_DEBUG("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  if (size <= 0) {
    QMMF_ERROR("%s:%s Undefined size(%u)", TAG, __func__, size);
    return -1;
  }

  int nFds = 1;
  int nInts = 3;
  int32_t ionType = ION_HEAP(ION_IOMMU_HEAP_ID);
  struct ion_allocation_data alloc;
  void* data = nullptr;
  struct ion_fd_data ionFdData;
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

  data = mmap(nullptr, alloc.len, PROT_READ | PROT_WRITE, MAP_SHARED,
              ionFdData.fd, 0);
  if (data == MAP_FAILED) {
    QMMF_ERROR("%s:%s  ION mmap failed: %s (%d)", TAG, __func__,
               strerror(errno), errno);
    goto ION_MAP_FAILED;
  }

  memset(&buf_info_, 0x0, sizeof(buf_info_));
  buf_info_.ion_handle = alloc;
  buf_info_.fd = ionFdData.fd;
  buf_info_.vaddr = data;
  // size and alloc.len might be different
  // In general alloc.len >= size
  buf_info_.buf_size = alloc.len;
  buf_info_.capacity = size;

  meta_handle_ = (native_handle_create(1, 16));
  if (meta_handle_ == nullptr) {
    QMMF_ERROR("%s:%s Failed to allocate metabuffer handle", TAG, __func__);
    goto NATIVE_HANDLE_CREATION_FAILED;
  }

  meta_handle_->version = sizeof(native_handle_t);
  meta_handle_->numFds  = nFds;
  meta_handle_->numInts = nInts;
  meta_handle_->data[0] = ionFdData.fd;
  meta_handle_->data[1] = 0;  // offset
  meta_handle_->data[2] = alloc.len;
  meta_handle_->data[3] = 0x200000;
  meta_handle_->data[4] = alloc.len;

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
  return ret;

NATIVE_HANDLE_CREATION_FAILED:
ION_MAP_FAILED:
  struct ion_handle_data ionHandleData;
  memset(&ionHandleData, 0x0, sizeof(ionHandleData));
  ionHandleData.handle = ionFdData.handle;
  ioctl(ion_device_, ION_IOC_FREE, &ionHandleData);

ION_ALLOC_FAILED:
  QMMF_ERROR("%s:%s ION Buffer allocation failed!", TAG, __func__);
  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
  return -1;
}

void TranscodeBuffer::Release() {
  QMMF_DEBUG("%s:%s Enter", TAG, __func__);

  if (buf_info_.vaddr) {
    munmap(buf_info_.vaddr, buf_info_.buf_size);
    buf_info_.vaddr = nullptr;
  }

  QMMF_DEBUG("%s:%s Releasing buffer with fd(%d)", TAG, __func__, buf_info_.fd);

  if (buf_info_.fd) {
    ioctl(ion_device_, ION_IOC_FREE, &(buf_info_.ion_handle));
    close(buf_info_.fd);
    buf_info_.fd = -1;
  }

  if (meta_handle_->data[0]) {
    close(meta_handle_->data[0]);
    meta_handle_->data[0] = -1;
    native_handle_delete(meta_handle_);
    meta_handle_ = nullptr;
  }

  QMMF_DEBUG("%s:%s Exit", TAG, __func__);
}

status_t TranscodeBuffer::CreateTranscodeBuffersVector(
    const shared_ptr<IAVCodec>& avcodec,
    const BufferOwner owner, const uint32_t port_index,
    vector<TranscodeBuffer>* buffer_list) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  status_t ret = 0;
  if (buffer_list == nullptr) {
    QMMF_ERROR("%s:%s Invalid Parameters : buffer_list = nullptr",
               TAG, __func__);
    return -1;
  }

  if (!buffer_list->empty()) {
    QMMF_ERROR("%s:%s buffer_list_ is not Empty", TAG, __func__);
    return -1;
  }

  uint32_t count, size;
  ret = avcodec->GetBufferRequirements(port_index, &count, &size);
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to get Buffer Requirements", TAG, __func__);
    return ret;
  }

  for (uint32_t i = 0; i < count; i++) {
    TranscodeBuffer buffer(owner, OwnerIndex(owner) | i);
    ret = buffer.Allocate(size);
    if (ret != 0) {
      QMMF_ERROR("%s:%s Failed to allocate %uth buffer", TAG, __func__, i + 1);
      goto release_buffer;
    }
    QMMF_INFO("%s:%s Buffer allocated with fd(%d)",
              TAG, __func__, buffer.GetFd());
    buffer_list->push_back(buffer);
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;

release_buffer:
  TranscodeBuffer::FreeTranscodeBuffersVector(buffer_list);
  assert(buffer_list->empty());
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

void TranscodeBuffer::FreeTranscodeBuffersVector(
    vector<TranscodeBuffer>* buffer_list) {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  if (buffer_list == nullptr)
    return;
  if (buffer_list->empty())
    return;

  while (!buffer_list->empty()) {
    buffer_list->begin()->Release();
    buffer_list->erase(buffer_list->begin());
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

FramerateCalculator::FramerateCalculator(uint64_t period, string str)
    : count_(0), period_(period), log_str_(str), is_first_trigger_(true) {
  QMMF_DEBUG("%s: Enter", __func__);
  QMMF_DEBUG("%s: Exit", __func__);
}

FramerateCalculator::~FramerateCalculator() {
  QMMF_DEBUG("%s: Enter", __func__);
  QMMF_DEBUG("%s: Exit", __func__);
}

void FramerateCalculator::Trigger() {
  QMMF_VERBOSE("%s: Enter", __func__);

  if (is_first_trigger_) {
    is_first_trigger_ = false;
    gettimeofday(&prevtv_, nullptr);
  } else {
    count_++;
    gettimeofday(&currtv_, nullptr);
    uint64_t time_diff =
        (uint64_t)((currtv_.tv_sec * 1000000 + currtv_.tv_usec) -
                   (prevtv_.tv_sec * 1000000 + prevtv_.tv_usec));
    if (time_diff >= period_) {
      float framerate = (count_ * 1000000) / (float)time_diff;
      QMMF_INFO("%s(%0.2f)", log_str_.c_str(), framerate);
      count_ = 0;
      prevtv_ = currtv_;
    }
  }

  QMMF_VERBOSE("%s: Exit", __func__);
}

void FramerateCalculator::Reset() {
  QMMF_DEBUG("%s: Enter", __func__);

  is_first_trigger_ = true;
  count_ = 0;

  QMMF_DEBUG("%s: Exit", __func__);
}

VQZipInfoExtractor::VQZipInfoExtractor(const VideoTrackCreateParam& param,
                                       const MM_TRACK_INFOTYPE& track_info,
                                       CMM_MediaDemuxInt* const demuxer)
    : avcodec_decode_params_(param),
      m_sTrackInfo_(track_info),
      m_pDemux_(demuxer),
      isFirstFrame_(true) {
  QMMF_INFO("VQZipInfoExtractor:%s Enter", __func__);
  QMMF_INFO("VQZipInfoExtractor:%s Exit", __func__);
}

VQZipInfoExtractor::~VQZipInfoExtractor() {
  QMMF_INFO("VQZipInfoExtractor:%s Enter", __func__);
  QMMF_INFO("VQZipInfoExtractor:%s Exit", __func__);
}

status_t VQZipInfoExtractor::Init() {
  QMMF_INFO("VQZipInfoExtractor:%s Enter", __func__);

  status_t ret = 0;
  avcodec_.reset(IAVCodec::CreateAVCodec());
  CodecParam codec_param(avcodec_decode_params_);

  ret = avcodec_->ConfigureCodec(CodecMimeType::kMimeTypeVideoDecAVC,
                                 codec_param);
  if(ret != 0) {
    QMMF_ERROR("VQZipInfoExtractor:%s Failed to configure Codec", __func__);
    return ret;
  }

  input_source_impl_ = make_shared<InputCodecSourceImpl>(avcodec_, this);
  output_source_impl_ = make_shared<OutputCodecSourceImpl>(avcodec_, this);

  ret = input_source_impl_->Prepare();
  if (ret != 0) {
    QMMF_ERROR("VQZipInfoExtractor:%s Failed to prepare Inputport of VQZipExtractor",
               __func__);
    goto release_resources;
  }

  ret = output_source_impl_->Prepare();
  if (ret != 0) {
    QMMF_ERROR("VQZipInfoExtractor:%s Failed to prepare Outputport of VQZipExtractor",
               __func__);
    goto release_resources;
  }

  QMMF_INFO("VQZipInfoExtractor:%s Exit", __func__);
  return ret;

release_resources:
  ReleaseResources();
  QMMF_INFO("VQZipInfoExtractor:%s Exit", __func__);
  return ret;
}

status_t VQZipInfoExtractor::ExtractVQZipInfo(VQZipInfo* vqzip_info) {
  QMMF_INFO("VQZipInfoExtractor:%s Enter", __func__);

  status_t ret = 0;
  FileSourceStatus mFSStatus = FILE_SOURCE_FAIL;
  mFSStatus = m_pDemux_->SeekAbsolutePosition(0);
  if (mFSStatus == FILE_SOURCE_FAIL) {
    QMMF_ERROR("VQZipInfoExtractor:%s: Failed to seek", __func__);
    return -1;
  }

  QMMF_INFO("VQZipInfoExtractor:%s Waiting for FBD", __func__);

  ret = avcodec_->StartCodec();
  if (ret != 0) {
    QMMF_ERROR("VQZipInfoExtractor:%s Failed to Start Codec", __func__);
    return ret;
  }

  {
    unique_lock<mutex> ul(wait_for_fbd_mutex_);
    wait_for_fbd_.wait(ul);
  }

  QMMF_INFO("VQZipInfoExtractor:%s FBD occured", __func__);

  size_t param_size;
  ret = avcodec_->GetParameters(CodecParamType::kVQZipInfo,
                                reinterpret_cast<void*>(vqzip_info),
                                &param_size);
  if (ret != 0 || param_size != sizeof(*vqzip_info)) {
    QMMF_ERROR("VQZipInfoExtractor:%s Failed to get the VQzip info from AVCodec",
               __func__);
    return ret;
  }

  QMMF_INFO("VQZipInfoExtractor:%s VQZip parameters extracted, vqzip_info[%s]",
            __func__, vqzip_info->ToString().c_str());

  ret = avcodec_->StopCodec();
  if (ret != 0) {
    QMMF_ERROR("VQZipInfoExtractor:%s Failed to Stop Codec", __func__);
    return ret;
  }

  mFSStatus = m_pDemux_->SeekAbsolutePosition(0);
  if (mFSStatus == FILE_SOURCE_FAIL) {
    QMMF_ERROR("VQZipInfoExtractor:%s Failed to seek", __func__);
    return -1;
  }

  QMMF_INFO("VQZipInfoExtractor:%s Exit", __func__);
  return ret;
}

status_t VQZipInfoExtractor::DeInit() {
  QMMF_INFO("VQZipInfoExtractor:%s Enter", __func__);
  status_t ret = 0;

  ret = avcodec_->ReleaseBuffer();
  if (ret != 0) {
    QMMF_ERROR("VQZipInfoExtractor:%s Failed to release buffers of AVCodec",
               __func__);
  }

  input_source_impl_->ReleaseBuffer();

  output_source_impl_->ReleaseBuffer();

  avcodec_.reset();
  input_source_impl_.reset();
  output_source_impl_.reset();

  QMMF_INFO("VQZipInfoExtractor:%s Exit", __func__);
  return ret;
}

void VQZipInfoExtractor::ReadNextFrame(
    TSQueue<TranscodeBuffer>::iterator buffer) {
  QMMF_VERBOSE("VQZipInfoExtractor:%s Enter", __func__);

  uint32_t csd_data_size = 0;
  FileSourceSampleInfo sSampleInfo;
  FileSourceMediaStatus eMediaStatus = FILE_SOURCE_DATA_ERROR;
  memset(&sSampleInfo, 0, sizeof(FileSourceSampleInfo));
  (m_sTrackInfo_).sVideo.sSampleBuf.ulLen =
    (m_sTrackInfo_).sVideo.sSampleBuf.ulMaxLen;
  if (isFirstFrame_) {
    uint32_t status = m_pDemux_->m_pFileSource->GetFormatBlock(
        (m_sTrackInfo_).sVideo.ulTkId, nullptr, &csd_data_size);

    QMMF_INFO("VQZipInfoExtractor:%s Video CSD data size = %u", __func__,
              csd_data_size);
    assert(FILE_SOURCE_SUCCESS == status);

    status = m_pDemux_->m_pFileSource->GetFormatBlock(
        (m_sTrackInfo_).sVideo.ulTkId,
        reinterpret_cast<uint8_t*>(buffer->GetVaddr()),
        &csd_data_size);
    assert(FILE_SOURCE_SUCCESS == status);
    isFirstFrame_ = false;
  }

  eMediaStatus = m_pDemux_->GetNextMediaSample(
      (m_sTrackInfo_).sVideo.ulTkId,
      reinterpret_cast<uint8_t*>(buffer->GetVaddr()) + csd_data_size,
      &((m_sTrackInfo_).sVideo.sSampleBuf.ulLen), sSampleInfo);
  if (eMediaStatus == FILE_SOURCE_DATA_END) {
    QMMF_ERROR("VQZipInfoExtractor:%s File does not contain enough number of frames",
               __func__);
    assert(0);
  }
  buffer->SetFilledSize((m_sTrackInfo_).sVideo.sSampleBuf.ulLen + csd_data_size);
  // Multiplying the timestamp to convert it into nano seconds
  buffer->SetTimestamp(1000*(sSampleInfo.startTime));
  buffer->SetFlag(0x0);
  buffer->SetOffset(0x0);

  QMMF_DEBUG("VQZipInfoExtractor:%s  Video Ts[%llu] filled_size[%u] flags[0x%x]  fd[%d]",
             __func__, (buffer->GetTimestamp())/1000, buffer->GetFilledSize(),
             buffer->GetFlag(), buffer->GetFd());

  QMMF_VERBOSE("VQZipInfoExtractor:%s Exit", __func__);
}

void VQZipInfoExtractor::NotiftyFBD(const TranscodeBuffer& buffer) {
  QMMF_DEBUG("VQZipInfoExtractor:%s Enter", __func__);
  if (!(buffer.GetFlag() & static_cast<uint32_t>(BufferFlags::kFlagEOS))
      && buffer.GetFilledSize())
    wait_for_fbd_.notify_one();
  QMMF_DEBUG("VQZipInfoExtractor:%s Exit", __func__);
}

void VQZipInfoExtractor::ReleaseResources() {
  QMMF_DEBUG("VQZipInfoExtractor:%s Enter", __func__);

  status_t ret = avcodec_->ReleaseBuffer();
  if (ret != 0) {
    QMMF_ERROR("VQZipInfoExtractor:%s Failed to release buffers of AVCodec",
               __func__);
  }

  input_source_impl_->ReleaseBuffer();
  output_source_impl_->ReleaseBuffer();

  avcodec_.reset();
  input_source_impl_.reset();
  input_source_impl_.reset();

  QMMF_DEBUG("VQZipInfoExtractor:%s Exit", __func__);
}

VQZipInfoExtractor::InputCodecSourceImpl::InputCodecSourceImpl(
    const shared_ptr<IAVCodec>& avcodec, VQZipInfoExtractor* const src)
    : avcodec_(avcodec), source_(src) {
  QMMF_DEBUG("VQZipInfoExtractor:%s Enter", __func__);
  QMMF_DEBUG("VQZipInfoExtractor:%s Exit", __func__);
}

VQZipInfoExtractor::InputCodecSourceImpl::~InputCodecSourceImpl() {
  QMMF_DEBUG("VQZipInfoExtractor:%s Enter", __func__);
  QMMF_DEBUG("VQZipInfoExtractor:%s Exit", __func__);
}

status_t VQZipInfoExtractor::InputCodecSourceImpl::Prepare() {
  QMMF_DEBUG("VQZipInfoExtractor:%s Enter", __func__);

  status_t ret = 0;
  ret = TranscodeBuffer::CreateTranscodeBuffersVector(
      avcodec_, BufferOwner::kVQZipInputPort, kPortIndexInput, &buffer_list_);
  if (ret != 0) {
    QMMF_ERROR("VQZipInfoExtractor:%s Failed to alllocate Input buffers",
               __func__);
    return ret;
  }

  vector<BufferDescriptor> temp_in;
  for (auto& iter: buffer_list_) {
    BufferDescriptor temp_buffer;
    memset(&temp_buffer, 0x0, sizeof(temp_buffer));
    ret = iter.getAVCodecBuffer(AVCodecBufferType::kNormal, &temp_buffer);
    assert(ret == 0);
    temp_in.push_back(temp_buffer);
  }

  ret = avcodec_->RegisterInputBuffers(temp_in);
  if (ret != 0) {
    QMMF_ERROR("VQZipInfoExtractor:%s Failed to register Input Buffers",
               __func__);
    return ret;
  }

  ret = avcodec_->AllocateBuffer(
      kPortIndexInput, 0 , 0,
      static_pointer_cast<ICodecSource>(shared_from_this()),
      temp_in);
  if(ret != 0) {
    QMMF_ERROR("VQZipInfoExtractor:%s Failed to call allocate buffer on VQZipExtractor Input",
               __func__);
    return ret;
  }

  free_buffer_queue_.Clear();
  for (auto& iter : buffer_list_) {
    free_buffer_queue_.PushBack(iter);
  }

  QMMF_DEBUG("VQZipInfoExtractor:%s Exit", __func__);
  return ret;
}

void VQZipInfoExtractor::InputCodecSourceImpl::ReleaseBuffer() {
  QMMF_DEBUG("VQZipInfoExtractor:%s Enter", __func__);

  TranscodeBuffer::FreeTranscodeBuffersVector(&buffer_list_);
  assert(buffer_list_.empty());

  free_buffer_queue_.Clear();
  occupy_buffer_queue_.Clear();

  avcodec_.reset();

  QMMF_DEBUG("VQZipInfoExtractor:%s Exit", __func__);
}

status_t VQZipInfoExtractor::InputCodecSourceImpl::GetBuffer(
    BufferDescriptor& buffer_descriptor, void* client_data) {
  QMMF_VERBOSE("VQZipInfoExtractor:%s Enter", __func__);

  status_t ret = 0;
  if (free_buffer_queue_.Size() <= 0) {
    unique_lock<mutex> ul(wait_for_frame_mutex_);
      if (free_buffer_queue_.Size() <= 0)
        wait_for_frame_.wait(ul);
  }

  TSQueue<TranscodeBuffer>::iterator pbuf = free_buffer_queue_.Begin();
  source_->ReadNextFrame(pbuf);
  ret = pbuf->getAVCodecBuffer(AVCodecBufferType::kNormal, &buffer_descriptor);
  assert(ret == 0);

  occupy_buffer_queue_.PushBack(*pbuf);
  free_buffer_queue_.Erase(free_buffer_queue_.Begin());

  if (buffer_descriptor.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS)) {
    QMMF_DEBUG("VQZipInfoExtractor:%s Last buffer", __func__);
    ret = -1;
  }

  QMMF_VERBOSE("VQZipInfoExtractor:%s Exit", __func__);
  return ret;
}

status_t VQZipInfoExtractor::InputCodecSourceImpl::ReturnBuffer(
    BufferDescriptor& buffer_descriptor, void* client_data) {
  QMMF_VERBOSE("VQZipInfoExtractor:%s Enter", __func__);

  status_t ret = 0;
  bool found = false;
  TSQueue<TranscodeBuffer>::iterator it = occupy_buffer_queue_.Begin();

  for (; it != occupy_buffer_queue_.End(); ++it) {
    if (it->GetVaddr() == buffer_descriptor.data) {
      QMMF_VERBOSE("VQZipInfoExtractor:%s Buffer found", __func__);
      free_buffer_queue_.PushBack(*it);
      occupy_buffer_queue_.Erase(it);
      lock_guard<mutex> lg(wait_for_frame_mutex_);
      wait_for_frame_.notify_one();
      found = true;
      break;
    }
  }

  assert(found == true);

  QMMF_VERBOSE("VQZipInfoExtractor:%s Exit", __func__);
  return ret;
}

status_t VQZipInfoExtractor::InputCodecSourceImpl::NotifyPortEvent(
    PortEventType event_type, void* event_data) {
  QMMF_DEBUG("VQZipInfoExtractor:%s Enter", __func__);
  QMMF_DEBUG("VQZipInfoExtractor:%s Exit", __func__);
  return 0;
}

VQZipInfoExtractor::OutputCodecSourceImpl::OutputCodecSourceImpl(
    const shared_ptr<IAVCodec>& avcodec, VQZipInfoExtractor* const sink)
    : avcodec_(avcodec), sink_(sink) {
  QMMF_DEBUG("VQZipInfoExtractor:%s Enter", __func__);
  QMMF_DEBUG("VQZipInfoExtractor:%s Exit", __func__);
}

VQZipInfoExtractor::OutputCodecSourceImpl::~OutputCodecSourceImpl() {
  QMMF_DEBUG("VQZipInfoExtractor:%s Enter", __func__);
  QMMF_DEBUG("VQZipInfoExtractor:%s Exit", __func__);
}

status_t VQZipInfoExtractor::OutputCodecSourceImpl::Prepare() {
  QMMF_DEBUG("VQZipInfoExtractor:%s Enter", __func__);

  status_t ret = 0;
  ret = TranscodeBuffer::CreateTranscodeBuffersVector(
      avcodec_, BufferOwner::kVQZipOutputPort, kPortIndexOutput, &buffer_list_);
  if (ret != 0) {
    QMMF_ERROR("VQZipInfoExtractor:%s Failed to allocate Output buffers",
               __func__);
    return ret;
  }

  vector<BufferDescriptor> temp_out;
  for (auto& iter: buffer_list_) {
    BufferDescriptor temp_buffer;
    memset(&temp_buffer, 0x0, sizeof(temp_buffer));
    ret = iter.getAVCodecBuffer(AVCodecBufferType::kNormal, &temp_buffer);
    assert(ret == 0);
    temp_out.push_back(temp_buffer);
  }

  ret = avcodec_->RegisterOutputBuffers(temp_out);
  if (ret != 0) {
    QMMF_ERROR("VQZipInfoExtractor:%s Failed to register output buffers",
               __func__);
    return ret;
  }

  ret = avcodec_->AllocateBuffer(
      kPortIndexOutput, 0 , 0,
      static_pointer_cast<ICodecSource>(shared_from_this()),
      temp_out);
  if(ret != 0) {
    QMMF_ERROR("VQZipInfoExtractor:%s Failed to call allocate buffer on VQZipExtractor Output",
               __func__);
    return ret;
  }

  free_buffer_queue_.Clear();
  for (auto& iter : buffer_list_) {
    free_buffer_queue_.PushBack(iter);
  }

  QMMF_DEBUG("VQZipInfoExtractor:%s Exit", __func__);
  return ret;
}

void VQZipInfoExtractor::OutputCodecSourceImpl::ReleaseBuffer() {
  QMMF_DEBUG("VQZipInfoExtractor:%s Enter", __func__);

  TranscodeBuffer::FreeTranscodeBuffersVector(&buffer_list_);
  assert(buffer_list_.empty());

  free_buffer_queue_.Clear();
  occupy_buffer_queue_.Clear();

  avcodec_.reset();

  QMMF_DEBUG("VQZipInfoExtractor:%s Exit", __func__);
}

status_t VQZipInfoExtractor::OutputCodecSourceImpl::GetBuffer(
    BufferDescriptor& buffer_descriptor, void* client_data) {
  QMMF_VERBOSE("VQZipInfoExtractor:%s Enter", __func__);

  status_t ret = 0;
  if (free_buffer_queue_.Size() <= 0) {
    unique_lock<mutex> ul(wait_for_frame_mutex_);
    if (free_buffer_queue_.Size() <= 0)
      wait_for_frame_.wait(ul);
  }

  TSQueue<TranscodeBuffer>::iterator pbuf = free_buffer_queue_.Begin();

  ret = pbuf->getAVCodecBuffer(AVCodecBufferType::kNormal, &buffer_descriptor);
  assert(ret == 0);

  occupy_buffer_queue_.PushBack(*pbuf);
  free_buffer_queue_.Erase(free_buffer_queue_.Begin());

  QMMF_VERBOSE("VQZipInfoExtractor:%s Exit", __func__);
  return ret;
}

status_t VQZipInfoExtractor::OutputCodecSourceImpl::ReturnBuffer(
    BufferDescriptor& buffer_descriptor, void* client_data) {
  QMMF_VERBOSE("VQZipInfoExtractor:%s Enter", __func__);

  status_t ret = 0;
  bool found = false;
  TSQueue<TranscodeBuffer>::iterator it = occupy_buffer_queue_.Begin();

  for (; it != occupy_buffer_queue_.End(); ++it) {
    if (it->GetFd() == buffer_descriptor.fd) {
      QMMF_VERBOSE("VQZipInfoExtractor:%s Buffer found", __func__);
      it->UpdateTranscodeBuffer(buffer_descriptor);
      free_buffer_queue_.PushBack(*it);
      occupy_buffer_queue_.Erase(it);
      lock_guard<mutex> lg(wait_for_frame_mutex_);
      wait_for_frame_.notify_one();
      found = true;
      sink_->NotiftyFBD(*it);
      break;
    }
  }

  assert(found == true);
  QMMF_VERBOSE("VQZipInfoExtractor:%s Exit", __func__);
  return ret;
}

status_t VQZipInfoExtractor::OutputCodecSourceImpl::NotifyPortEvent(
    PortEventType event_type, void* event_data) {
  QMMF_DEBUG("VQZipInfoExtractor:%s Enter", __func__);

  status_t ret = 0;
  vector<BufferDescriptor> temp_out;
  switch (event_type) {
    case PortEventType::kPortStatus:
      break;
    case PortEventType::kPortSettingsChanged:
      switch (reinterpret_cast<PortreconfigData*>(event_data)->reconfig_type) {
        case PortreconfigData::PortReconfigType::kCropParametersChanged:
          break;
        case PortreconfigData::PortReconfigType::kBufferRequirementsChanged:
          QMMF_DEBUG("VQZipInfoExtractor:%s Releasing Output buffers",
                     __func__);
          TranscodeBuffer::FreeTranscodeBuffersVector(&buffer_list_);
          assert(buffer_list_.empty());

          QMMF_DEBUG("VQZipInfoExtractor:%s Allocating new set of buffers",
                     __func__);
          ret = TranscodeBuffer::CreateTranscodeBuffersVector(
              avcodec_, BufferOwner::kVQZipOutputPort, kPortIndexOutput,
              &buffer_list_);
          if (ret != 0) {
            QMMF_ERROR("VQZipInfoExtractor:%s Buffer allocation failed",
                       __func__);
            return ret;
          }
          for (auto& iter: buffer_list_) {
            BufferDescriptor temp_buffer;
            memset(&temp_buffer, 0x0, sizeof(temp_buffer));
            ret = iter.getAVCodecBuffer(AVCodecBufferType::kNormal,
                                        &temp_buffer);
            assert(ret == 0);
            temp_out.push_back(temp_buffer);
          }
          ret = avcodec_->RegisterOutputBuffers(temp_out);
          if (ret != 0) {
            QMMF_ERROR("VQZipInfoExtractor:%s Buffer registration failed",
                       __func__);
            return ret;
          }
          free_buffer_queue_.Clear();
          occupy_buffer_queue_.Clear();
          for (auto& iter : buffer_list_) {
            free_buffer_queue_.PushBack(iter);
          }
          wait_for_frame_.notify_one();
          break;
        default:
          QMMF_ERROR("VQZipInfoExtractor:%s Unknown PortReconfigType",
                     __func__);
          return -1;
      }
      break;
    default:
      QMMF_ERROR("VQZipInfoExtractor:%s Unknown PortEventType", __func__);
      return -1;
  }

  QMMF_DEBUG("VQZipInfoExtractor:%s Exit", __func__);
  return ret;
}

};  // namespace transcode
};  // namespace qmmf
