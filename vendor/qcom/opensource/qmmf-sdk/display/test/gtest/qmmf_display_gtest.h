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

#include <fcntl.h>
#include <dirent.h>
#include <functional>
#include <gtest/gtest.h>
#include <vector>
#include <map>

#include <qmmf-sdk/qmmf_display.h>
#include <qmmf-sdk/qmmf_display_params.h>
#include <hardware/gralloc.h>
#include "display/test/gtest/qmmf_display_buffer_allocator.h"

using namespace qmmf;
using namespace display;
using namespace android;

class DisplayGtest : public ::testing::Test {
 public:
  DisplayGtest() : display_() {};

  ~DisplayGtest() {};

 protected:
  const ::testing::TestInfo* test_info_;

  void SetUp() override;

  void TearDown() override;

  int32_t Init(DisplayType display_type);

  int32_t DeInit(DisplayType display_type);

  void Test1YUV();

  void Test1RGB();

  void Test1YUV_1RGB();

  void Test1YUV_ExternalBuffer();

  void Test1YUV_1RGB_ExternalBuffer();

  void DisplayCallbackHandler(DisplayEventType event_type, void *event_data,
                               size_t event_data_size);

  void SessionCallbackHandler(DisplayEventType event_type, void *event_data,
                               size_t event_data_size);

  void DisplayVSyncHandler(int64_t time_stamp);


  Display*              display_;
  DisplayCb            display_status_cb_;
  uint32_t             iteration_count_;
  DisplayBufferAllocator buffer_allocator_;
  pthread_mutex_t thread_lock_;
  pthread_t                    pid_;
  bool running_;
  static void* DisplayVSync(void *ptr);

  typedef struct BufInfo {
    BufferInfo buffer_info;
    void* buf;
    int32_t buf_id;
  } BufInfo;

  typedef struct SurfaceData {
  uint32_t   surface_id;
  FILE* file;
  SurfaceParam surface_param;
  SurfaceBuffer surface_buffer;
  bool buffer_ready;
  } SurfaceData;

  //Mapping of surface_id and BufInfo
  std::map<uint32_t, std::vector<BufInfo*>> buf_info;

  //Mapping of surface_id and SurfaceData
  typedef std::map <uint32_t , SurfaceData* > surface_data_map;
  surface_data_map surface_data_;

  std::map <uint32_t , std::vector<uint32_t> > sessions_;
};

