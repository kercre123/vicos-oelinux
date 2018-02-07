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

#include <dirent.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <functional>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <linux/msm_ion.h>
#include <media/msm_media_info.h>
#include <utils/Condition.h>
#include <utils/Mutex.h>

#include "common/qmmf_common_utils.h"
#include "qmmf-sdk/qmmf_avcodec.h"

using namespace android;
using namespace qmmf;
using namespace qmmf::avcodec;
using namespace std;

#define MAX_FILE_NAME 80
typedef  struct ion_allocation_data IonHandleData;

class InputCodecSourceImpl;
class OutputCodecSourceImpl;

class CodecGtest : public ::testing::Test {

 public:
  CodecGtest() {};

  ~CodecGtest() {};

 protected:
  const ::testing::TestInfo* test_info_;

  void SetUp() override;

  void TearDown() override;

  status_t CreateCodec();

  status_t DeleteCodec();

  status_t AllocateBuffer(uint32_t port);

  status_t ReleaseBuffer();

  IAVCodec*                         avcodec_;
  int32_t                           ion_device_;
  Mutex                             stop_lock_;
  bool                              stop_;
  vector<BufferDescriptor>          input_buffer_list_;
  vector<BufferDescriptor>          output_buffer_list_;
  vector<IonHandleData>             input_ion_handle_data;
  vector<IonHandleData>             output_ion_handle_data;
  shared_ptr<InputCodecSourceImpl>  input_source_impl_;
  shared_ptr<OutputCodecSourceImpl> output_source_impl_;
};

class InputCodecSourceImpl : public ICodecSource {

public:
  InputCodecSourceImpl();

  ~InputCodecSourceImpl();

  status_t GetBuffer(BufferDescriptor& stream_buffer,
                     void* client_data) override;

  status_t ReturnBuffer(BufferDescriptor& stream_buffer,
                        void* client_data) override;

  status_t NotifyPortEvent(PortEventType event_type,
                           void* event_data)  override;

  void BufferStatus();

  void AddBufferList(vector<BufferDescriptor>& list);

private:
  status_t   ReadFile(int32_t fd, uint32_t size, int32_t *byte_read);

  Mutex                     wait_for_frame_lock_;
  Condition                 wait_for_frame_;
  vector<BufferDescriptor>  input_list_;
  TSQueue<BufferDescriptor> input_free_buffer_queue_;
  TSQueue<BufferDescriptor> input_occupy_buffer_queue_;
};

class OutputCodecSourceImpl : public ICodecSource {

public:
  OutputCodecSourceImpl();

  ~OutputCodecSourceImpl();

  status_t GetBuffer(BufferDescriptor& codec_buffer,
                     void* client_data) override;

  status_t ReturnBuffer(BufferDescriptor& codec_buffer,
                        void* client_data) override;

  status_t NotifyPortEvent(PortEventType event_type,
                           void* event_data)  override;

  void BufferStatus();

  void AddBufferList(vector<BufferDescriptor>& list);

private:
  Mutex                     wait_for_frame_lock_;
  Condition                 wait_for_frame_;
  vector<BufferDescriptor>  output_list_;
  TSQueue<BufferDescriptor> output_free_buffer_queue_;
  TSQueue<BufferDescriptor> output_occupy_buffer_queue_;
};
