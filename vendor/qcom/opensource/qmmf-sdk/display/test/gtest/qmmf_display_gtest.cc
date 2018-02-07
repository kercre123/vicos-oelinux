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

#define TAG "DisplayGTest"


#include <utils/Log.h>
#include <utils/String8.h>
#include <utils/Errors.h>
#include <camera/CameraMetadata.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>

#include "display/test/gtest/qmmf_display_gtest.h"
#include <hardware/hardware.h>

//#define DEBUG
#define TEST_INFO(fmt, args...)  ALOGD(fmt, ##args)
#define TEST_ERROR(fmt, args...) ALOGE(fmt, ##args)
#ifdef DEBUG
#define TEST_DBG  TEST_INFO
#else
#define TEST_DBG(...) ((void)0)
#endif

#define ALIGNED_WIDTH(x) ((x)+((x%64)?(64-(x%64)):0))

static const int32_t kIterationCount = 5;

void DisplayGtest::SetUp() {

  TEST_INFO("%s:%s Enter ", TAG, __func__);

  test_info_ = ::testing::UnitTest::GetInstance()->current_test_info();

  display_status_cb_.EventCb = [&] ( DisplayEventType event_type,
      void *event_data, size_t event_data_size)
      { DisplayCallbackHandler(event_type, event_data, event_data_size); };

  display_status_cb_.VSyncCb = [&] ( int64_t time_stamp)
      { DisplayVSyncHandler(time_stamp); };

  iteration_count_ = kIterationCount;
  pthread_mutex_init(&thread_lock_, NULL);

  TEST_INFO("%s:%s Exit ", TAG, __func__);
}

void DisplayGtest::TearDown() {

  TEST_INFO("%s:%s Enter ", TAG, __func__);
  pthread_mutex_destroy(&thread_lock_);
  TEST_INFO("%s:%s Exit ", TAG, __func__);
}

int32_t DisplayGtest::Init(DisplayType display_type) {
  display_= new Display();
  assert(display_ != nullptr);
  auto ret = display_->Connect();
  if(ret != 0) {
    TEST_ERROR("%s:%s Connect Failed!!", TAG, __func__);
    return ret;
  }

  ret = display_->CreateDisplay(display_type, display_status_cb_);
  if(ret != 0) {
    display_->Disconnect();
    TEST_ERROR("%s:%s CreateDisplay Failed!!", TAG, __func__);
    return ret;
  }

  ret = pthread_create(&pid_, NULL, DisplayVSync, this);
  if (0 != ret) {
    display_->DestroyDisplay(display_type);
    display_->Disconnect();
    TEST_ERROR("%s: Unable to create HandleVSync thread\n", __func__);
    return ret;
  }
  running_ = 1;
  return ret;
}

int32_t DisplayGtest::DeInit(DisplayType display_type) {
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  if (0 != pid_) {
    pthread_mutex_lock(&thread_lock_);
    running_ = 0;
    pthread_mutex_unlock(&thread_lock_);

    pthread_join(pid_, NULL);

    pthread_mutex_lock(&thread_lock_);
    pid_ = 0;
    pthread_mutex_unlock(&thread_lock_);
  }

  auto ret = display_->DestroyDisplay(display_type);
  if(ret != 0) {
    TEST_ERROR("%s:%s DestroyDisplay Failed!!", TAG, __func__);
  }
  ret = display_->Disconnect();

  this->surface_data_.clear();
  if (display_ != nullptr) {

    TEST_INFO("%s:%s: DELETE display_:%p", TAG, __func__, display_);
    delete display_;
    display_ = nullptr;
  }
  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}


/*
* Test1YUV: This test case will test display of 1 YUV video.
* Api test sequence:
*  - Init
*  - CreateSurface
*  - DequeueSurfaceBuffer
*  - Go to Sleep and run the DisplayVSync thread
*  - DestroySurface
*  - DeInit
*/
TEST_F(DisplayGtest, Test1YUV) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  int32_t ret;
  uint32_t surface_id;
  SurfaceConfig surface_config;

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    pthread_mutex_lock(&thread_lock_);

    ret = Init(DisplayType::kPrimary);
    if(ret != 0) {
      TEST_ERROR("%s:%s Init Failed!!", TAG, __func__);
      goto exit;
    }

    memset(&surface_config, 0x0, sizeof surface_config);

    surface_config.width=1920;
    surface_config.height=1080;
    surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
    surface_config.buffer_count=4;
    surface_config.cache=0;
    surface_config.use_buffer=0;
    ret = display_->CreateSurface(surface_config, &surface_id);
    if(ret != 0) {
      TEST_ERROR("%s:%s CreateSurface Failed!!", TAG, __func__);
      ret = DeInit(DisplayType::kPrimary);
      goto exit;
    }

    SurfaceData* surface_data = new SurfaceData();
    assert(surface_data!=NULL);

    surface_data->surface_id = surface_id;
    ret = display_->DequeueSurfaceBuffer(surface_id,
        surface_data->surface_buffer);
    surface_data->file = fopen("/data/misc/qmmf/YUV/1080p_1.nv12.yuv", "r");
    if (!surface_data->file) {
      TEST_ERROR("%s:%s: Unable to open file", TAG, __func__);
      ret = display_->DestroySurface(surface_data->surface_id);
      if(ret != 0) {
        TEST_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
      }
      delete surface_data;
      ret = DeInit(DisplayType::kPrimary);
      goto exit;
    }

    uint8_t *src = (uint8_t *)surface_data->surface_buffer.plane_info[0].buf +
        surface_data->surface_buffer.plane_info[0].offset;
    uint32_t offset_temp = 0;
    uint32_t read_len = 0;

    for(uint32_t i=0;i<(surface_data->surface_buffer.plane_info[0].height);i++) {
      read_len = fread(src+offset_temp, sizeof(uint8_t),
          surface_data->surface_buffer.plane_info[0].width,
          surface_data->file);
      assert(read_len == surface_data->surface_buffer.plane_info[0].width);
      offset_temp += surface_data->surface_buffer.plane_info[0].stride;
    }
    offset_temp += surface_data->surface_buffer.plane_info[0].stride * 8;
    for(uint32_t i=0;i<(surface_data->surface_buffer.plane_info[0].height)/2;
        i++) {
      read_len = fread(src+offset_temp, sizeof(uint8_t),
          surface_data->surface_buffer.plane_info[0].width,
          surface_data->file);
      assert(read_len == surface_data->surface_buffer.plane_info[0].width);
      offset_temp += surface_data->surface_buffer.plane_info[0].stride;
    }
    surface_data->buffer_ready=1;

    surface_data->surface_param.src_rect = { 0.0, 0.0, 1920.0, 1080.0 };
    surface_data->surface_param.dst_rect = { 0.0, 0.0, 1920.0, 1080.0 };
    surface_data->surface_param.surface_blending =
        SurfaceBlending::kBlendingCoverage;
    surface_data->surface_param.surface_flags.cursor=0;
    surface_data->surface_param.frame_rate=20;
    surface_data->surface_param.z_order = 0;
    surface_data->surface_param.solid_fill_color=0;
    surface_data->surface_param.surface_transform.rotation=0.0f;
    surface_data->surface_param.surface_transform.flip_horizontal=0;
    surface_data->surface_param.surface_transform.flip_vertical=0;

    surface_data_.insert({surface_id, surface_data});
    running_=1;

    pthread_mutex_unlock(&thread_lock_);
    sleep(10);

    pthread_mutex_lock(&thread_lock_);
    for (surface_data_map::iterator it = surface_data_.begin();
        it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != NULL);

      ret = display_->DestroySurface(surface_data->surface_id);
      if(ret != 0) {
        TEST_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
      }
      if (surface_data->file) {
        fclose(surface_data->file);
        surface_data->file = NULL;
      }
      delete surface_data;
      surface_data_.erase (it);
    }
    pthread_mutex_unlock(&thread_lock_);
    if(!surface_data_.size()) {
      ret = DeInit(DisplayType::kPrimary);
      if(ret != 0) {
        TEST_ERROR("%s:%s Disconnect Failed!!", TAG, __func__);
      }
    }
  }
exit:

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Test1RGB: This test case will test display of 1 RGB Image.
* Api test sequence:
*  - Init
*  - CreateSurface
*  - DequeueSurfaceBuffer
*  - Go to Sleep and run the DisplayVSync thread
*  - DestroySurface
*  - DeInit
*/
TEST_F(DisplayGtest, Test1RGB) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  int32_t ret;
  uint32_t surface_id;
  SurfaceConfig surface_config;

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    pthread_mutex_lock(&thread_lock_);

    ret = Init(DisplayType::kPrimary);
    if(ret != 0) {
      TEST_ERROR("%s:%s Init Failed!!", TAG, __func__);
      goto exit;
    }

    memset(&surface_config, 0x0, sizeof surface_config);

    surface_config.width=352;
    surface_config.height=288;
    surface_config.format = SurfaceFormat::kFormatBGRA8888;
    surface_config.buffer_count=4;
    surface_config.cache=0;
    surface_config.use_buffer=0;
    ret = display_->CreateSurface(surface_config, &surface_id);
    if(ret != 0) {
      TEST_ERROR("%s:%s CreateSurface Failed!!", TAG, __func__);
      ret = DeInit(DisplayType::kPrimary);
      goto exit;
    }

    SurfaceData* surface_data = new SurfaceData();
    assert(surface_data!=NULL);

    surface_data->surface_id = surface_id;
    ret = display_->DequeueSurfaceBuffer(surface_id,
        surface_data->surface_buffer);

    surface_data->file = fopen("/data/misc/qmmf/Images/fasimo_352x288_bgra_8888.rgb",
        "r");
    if (!surface_data->file) {
      TEST_ERROR("%s:%s: Unable to open file", TAG, __func__);
      ret = display_->DestroySurface(surface_data->surface_id);
      if(ret != 0) {
        TEST_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
      }
      delete surface_data;
      ret = DeInit(DisplayType::kPrimary);
      goto exit;
    }

    int32_t offset=0;
    for(uint32_t i=0;i<surface_data->surface_buffer.plane_info[0].height;i++) {
      fread((uint8_t*)surface_data->surface_buffer.plane_info[0].buf +
        surface_data->surface_buffer.plane_info[0].offset + offset,
        sizeof(uint8_t), surface_data->surface_buffer.plane_info[0].width*4,
        surface_data->file);
    offset += ALIGNED_WIDTH(surface_data->surface_buffer.plane_info[0].width)*4;
    }
    fclose (surface_data->file);
    surface_data->buffer_ready=1;

    surface_data->surface_param.src_rect = { 0.0, 0.0, 352.0, 288.0 };
    surface_data->surface_param.dst_rect = { 0.0, 0.0, 352.0, 288.0 };
    surface_data->surface_param.surface_blending =
        SurfaceBlending::kBlendingCoverage;
    surface_data->surface_param.surface_flags.cursor=0;
    surface_data->surface_param.frame_rate=20;
    surface_data->surface_param.z_order = 0;
    surface_data->surface_param.solid_fill_color=0;
    surface_data->surface_param.surface_transform.rotation=0.0f;
    surface_data->surface_param.surface_transform.flip_horizontal=0;
    surface_data->surface_param.surface_transform.flip_vertical=0;

    surface_data_.insert({surface_id, surface_data});
    running_=1;
    pthread_mutex_unlock(&thread_lock_);

    sleep(10);

    pthread_mutex_lock(&thread_lock_);
    for (surface_data_map::iterator it = surface_data_.begin();
        it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != NULL);

      ret = display_->DestroySurface(surface_data->surface_id);
      if(ret != 0) {
        TEST_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
      }
      delete surface_data;
      surface_data_.erase (it);
    }
    if(!surface_data_.size()) {
      pthread_mutex_unlock(&thread_lock_);
      ret = DeInit(DisplayType::kPrimary);
      if(ret != 0) {
        TEST_ERROR("%s:%s Disconnect Failed!!", TAG, __func__);
      }
    }
  }
exit:

  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}


/*
* Test1YUV_1RGB: This will test display of 1 YUV Video and 1 RGB Image.
* Api test sequence:
*  - Init
*  - CreateSurface ---- for YUV
*  - DequeueSurfaceBuffer
*  - CreateSurface ---- for RGB
*  - DequeueSurfaceBuffer
*  - Go to Sleep and run the DisplayVSync thread
*  - DestroySurface ---- for YUV
*  - DestroySurface ---- for RGB
*  - DeInit
*/
TEST_F(DisplayGtest, Test1YUV_1RGB) {

  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  int32_t ret;
  uint32_t surface_id;
  SurfaceConfig surface_config;

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    pthread_mutex_lock(&thread_lock_);

    ret = Init(DisplayType::kPrimary);
    if(ret != 0) {
      TEST_ERROR("%s:%s Init Failed!!", TAG, __func__);
      goto exit;
    }

    {
      memset(&surface_config, 0x0, sizeof surface_config);

      surface_config.width=1920;
      surface_config.height=1080;
      surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
      surface_config.buffer_count=4;
      surface_config.cache=0;
      surface_config.use_buffer=0;
      ret = display_->CreateSurface(surface_config, &surface_id);
      if(ret != 0) {
        TEST_ERROR("%s:%s CreateSurface Failed!!", TAG, __func__);
        goto next;
      }

      SurfaceData* surface_data = new SurfaceData();
      assert(surface_data!=NULL);

      surface_data->surface_id = surface_id;
      ret = display_->DequeueSurfaceBuffer(surface_id,
          surface_data->surface_buffer);
      surface_data->file = fopen("/data/misc/qmmf/YUV/1080p_1.nv12.yuv", "r");
      if (!surface_data->file) {
        TEST_ERROR("%s:%s: Unable to open file", TAG, __func__);
        ret = display_->DestroySurface(surface_data->surface_id);
        if(ret != 0) {
          TEST_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
        }
        delete surface_data;
        goto next;
      }

      uint8_t *src = (uint8_t *)surface_data->surface_buffer.plane_info[0].buf +
          surface_data->surface_buffer.plane_info[0].offset;
      uint32_t offset_temp = 0;
      uint32_t read_len = 0;

      for(uint32_t i=0;i<(surface_data->surface_buffer.plane_info[0].height);
          i++) {
        read_len = fread(src+offset_temp, sizeof(uint8_t),
            surface_data->surface_buffer.plane_info[0].width,
            surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      offset_temp += surface_data->surface_buffer.plane_info[0].stride * 8;
      for(uint32_t i=0;i<(surface_data->surface_buffer.plane_info[0].height)/2;
          i++) {
        read_len = fread(src+offset_temp, sizeof(uint8_t),
            surface_data->surface_buffer.plane_info[0].width,
            surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      surface_data->buffer_ready=1;

      surface_data->surface_param.src_rect = { 0.0, 0.0, 1920.0, 1080.0 };
      surface_data->surface_param.dst_rect = { 0.0, 0.0, 1920.0, 1080.0 };
      surface_data->surface_param.surface_blending =
          SurfaceBlending::kBlendingCoverage;
      surface_data->surface_param.surface_flags.cursor=0;
      surface_data->surface_param.frame_rate=20;
      surface_data->surface_param.z_order = 0;
      surface_data->surface_param.solid_fill_color=0;
      surface_data->surface_param.surface_transform.rotation=0.0f;
      surface_data->surface_param.surface_transform.flip_horizontal=0;
      surface_data->surface_param.surface_transform.flip_vertical=0;

      surface_data_.insert({surface_id, surface_data});
    }
next:

    {
      memset(&surface_config, 0x0, sizeof surface_config);

      surface_config.width=352;
      surface_config.height=288;
      surface_config.format = SurfaceFormat::kFormatBGRA8888;
      surface_config.buffer_count=4;
      surface_config.cache=0;
      surface_config.use_buffer=0;
      ret = display_->CreateSurface(surface_config, &surface_id);
      if(ret != 0) {
        TEST_ERROR("%s:%s CreateSurface Failed!!", TAG, __func__);
        goto sleep;
      }

      SurfaceData* surface_data = new SurfaceData();
      assert(surface_data!=NULL);

      surface_data->surface_id = surface_id;
      ret = display_->DequeueSurfaceBuffer(surface_id,
          surface_data->surface_buffer);

      surface_data->file = fopen("/data/misc/qmmf/Images/fasimo_352x288_bgra_8888.rgb",
          "r");
      if (!surface_data->file) {
        TEST_ERROR("%s:%s: Unable to open file", TAG, __func__);
        ret = display_->DestroySurface(surface_data->surface_id);
        if(ret != 0) {
          TEST_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
        }
        delete surface_data;
        ret = DeInit(DisplayType::kPrimary);
        goto sleep;
      }

      int32_t offset=0;
      for(uint32_t i=0;i<surface_data->surface_buffer.plane_info[0].height;i++) {
      fread((uint8_t*)surface_data->surface_buffer.plane_info[0].buf +
          surface_data->surface_buffer.plane_info[0].offset + offset,
          sizeof(uint8_t), surface_data->surface_buffer.plane_info[0].width*4,
          surface_data->file);
      offset += ALIGNED_WIDTH(surface_data->surface_buffer.plane_info[0].width)
          *4;
      }
      fclose (surface_data->file);
      surface_data->buffer_ready=1;

      surface_data->surface_param.src_rect = { 0.0, 0.0, 352.0, 288.0 };
      surface_data->surface_param.dst_rect = { 0.0, 0.0, 352.0, 288.0 };
      surface_data->surface_param.surface_blending =
          SurfaceBlending::kBlendingCoverage;
      surface_data->surface_param.surface_flags.cursor=0;
      surface_data->surface_param.frame_rate=20;
      surface_data->surface_param.z_order = 0;
      surface_data->surface_param.solid_fill_color=0;
      surface_data->surface_param.surface_transform.rotation=0.0f;
      surface_data->surface_param.surface_transform.flip_horizontal=0;
      surface_data->surface_param.surface_transform.flip_vertical=0;

      surface_data_.insert({surface_id, surface_data});
    }
    running_ = 1;
    pthread_mutex_unlock(&thread_lock_);

sleep:

    sleep(10);

    pthread_mutex_lock(&thread_lock_);
    for (surface_data_map::iterator it = surface_data_.begin();
        it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != NULL);

      ret = display_->DestroySurface(surface_data->surface_id);
      if(ret != 0) {
        TEST_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
      }
      delete surface_data;
      surface_data_.erase (it);
    }
    if(!surface_data_.size()) {
      pthread_mutex_unlock(&thread_lock_);
      ret = DeInit(DisplayType::kPrimary);
      if(ret != 0) {
        TEST_ERROR("%s:%s Disconnect Failed!!", TAG, __func__);
      }
    }
  }
exit:
  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Test1YUV_ExternalBuffer: This will test display of 1 YUV Video where the
* buffer is allocated by the test app.
* Api test sequence:
*  - Init
*  - CreateSurface ---- for YUV
*  - AllocateBuffer ---- for YUV
*  - DequeueSurfaceBuffer
*  - Go to Sleep and run the DisplayVSync thread
*  - DestroySurface ---- for YUV
*  - Deallocate Buffer ---- for YUV
*  - DeInit
*/
TEST_F(DisplayGtest, Test1YUV_ExternalBuffer) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  int32_t ret;
  uint32_t surface_id;
  SurfaceConfig surface_config;

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    pthread_mutex_lock(&thread_lock_);

    ret = Init(DisplayType::kPrimary);
    if(ret != 0) {
      TEST_ERROR("%s:%s Init Failed!!", TAG, __func__);
      goto exit;
    }
    {
      memset(&surface_config, 0x0, sizeof surface_config);

      surface_config.width=1920;
      surface_config.height=1080;
      surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
      surface_config.buffer_count=4;
      surface_config.cache=0;
      surface_config.use_buffer=1;

      ret = display_->CreateSurface(surface_config, &surface_id);
      if(ret != 0) {
        TEST_ERROR("%s:%s CreateSurface Failed!!", TAG, __func__);
        ret = DeInit(DisplayType::kPrimary);
        goto exit;
      }

      SurfaceData* surface_data = new SurfaceData();
      assert(surface_data!=NULL);

      surface_data->surface_id = surface_id;

      std::vector<BufInfo*> buffers;

      for(uint32_t i=0; i<surface_config.buffer_count;i++) {
        BufInfo* new_buf_info = new BufInfo();
        new_buf_info->buffer_info.buffer_config.width =surface_config.width;
        new_buf_info->buffer_info.buffer_config.height = surface_config.height;
        new_buf_info->buffer_info.buffer_config.format =
            (LayerBufferFormat)surface_config.format;
        new_buf_info->buffer_info.buffer_config.buffer_count = 1;
        new_buf_info->buffer_info.buffer_config.cache = surface_config.cache;
        new_buf_info->buffer_info.alloc_buffer_info.fd = -1;
        new_buf_info->buffer_info.alloc_buffer_info.stride = 0;
        new_buf_info->buffer_info.alloc_buffer_info.size = 0;
        new_buf_info->buf_id = i;
        new_buf_info->buf = nullptr;
        buffers.push_back(new_buf_info);
        ret = buffer_allocator_.AllocateBuffer(&new_buf_info->buffer_info);
        if (ret != kErrorNone) {
          TEST_ERROR("%s:%s: AllocateBuffer Failed. Error = %d", TAG, __func__,
              ret);
          ret = display_->DestroySurface(surface_data->surface_id);
          if(ret != 0) {
            TEST_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
          }
          delete surface_data;
          for (std::vector<BufInfo*>::iterator iter = buffers.begin();
              iter != buffers.end(); ++iter) {

            buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
            buffers.erase(iter);
            delete *iter;
          }
          ret = DeInit(DisplayType::kPrimary);
          goto exit;
        }
      }
      buf_info.insert({surface_id, buffers});

      surface_data->file = fopen("/data/misc/qmmf/YUV/1080p_1.nv12.yuv", "r");
      if (!surface_data->file) {
        TEST_ERROR("%s:%s: Unable to open file", TAG, __func__);
        ret = display_->DestroySurface(surface_data->surface_id);
        if(ret != 0) {
          TEST_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
        }
        delete surface_data;
        for (std::map<uint32_t, std::vector<BufInfo*>>::iterator it =
            buf_info.begin(); it != buf_info.end(); ++it) {
          if(it != buf_info.end()) {
            for (std::vector<BufInfo*>::iterator iter = it->second.begin();
                iter != it->second.end(); ++iter) {
              buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
              it->second.erase(iter);
              delete *iter;
            }
          }
          buf_info.erase(it);
        }
        ret = DeInit(DisplayType::kPrimary);
        goto exit;
      }
      auto bufinfo = buf_info.find(surface_id);
      BufInfo* new_buf_info = bufinfo->second.at(0);
      BufferInfo *bufferinfo = &new_buf_info->buffer_info;
      surface_data->surface_buffer.plane_info[0].ion_fd =
          bufferinfo->alloc_buffer_info.fd;
      surface_data->surface_buffer.buf_id =0;
      surface_data->surface_buffer.format =
          (SurfaceFormat)bufferinfo->buffer_config.format;
      surface_data->surface_buffer.plane_info[0].stride =
          bufferinfo->alloc_buffer_info.stride;
      surface_data->surface_buffer.plane_info[0].size =
          bufferinfo->alloc_buffer_info.size;
      surface_data->surface_buffer.plane_info[0].width =
          bufferinfo->buffer_config.width;
      surface_data->surface_buffer.plane_info[0].height =
          bufferinfo->buffer_config.height;
      surface_data->surface_buffer.plane_info[0].offset = 0;
      surface_data->surface_buffer.plane_info[0].buf = new_buf_info->buf =
          mmap(NULL, (size_t)surface_data->surface_buffer.plane_info[0].size,
          PROT_READ | PROT_WRITE, MAP_SHARED,
          surface_data->surface_buffer.plane_info[0].ion_fd, 0);
      assert(surface_data->surface_buffer.plane_info[0].buf != NULL);

      uint8_t *src = (uint8_t *)surface_data->surface_buffer.plane_info[0].buf +
          surface_data->surface_buffer.plane_info[0].offset;
      uint32_t offset_temp = 0;
      uint32_t read_len = 0;

      for(uint32_t i=0;i<(surface_data->surface_buffer.plane_info[0].height);
          i++) {
        read_len = fread(src+offset_temp, sizeof(uint8_t),
            surface_data->surface_buffer.plane_info[0].width,
            surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      offset_temp += surface_data->surface_buffer.plane_info[0].stride * 8;
      for(uint32_t i=0;i<(surface_data->surface_buffer.plane_info[0].height)/2;
          i++) {
        read_len = fread(src+offset_temp, sizeof(uint8_t),
            surface_data->surface_buffer.plane_info[0].width,
            surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      surface_data->buffer_ready=1;

      surface_data->surface_param.src_rect = { 0.0, 0.0, 1920.0, 1080.0 };
      surface_data->surface_param.dst_rect = { 0.0, 0.0, 1920.0, 1080.0 };
      surface_data->surface_param.surface_blending =
          SurfaceBlending::kBlendingCoverage;
      surface_data->surface_param.surface_flags.cursor=0;
      surface_data->surface_param.frame_rate=20;
      surface_data->surface_param.z_order = 0;
      surface_data->surface_param.solid_fill_color=0;
      surface_data->surface_param.surface_transform.rotation=0.0f;
      surface_data->surface_param.surface_transform.flip_horizontal=0;
      surface_data->surface_param.surface_transform.flip_vertical=0;

      surface_data_.insert({surface_id, surface_data});
    }
    running_ = 1;
    pthread_mutex_unlock(&thread_lock_);

    sleep(10);
    pthread_mutex_lock(&thread_lock_);
    for (surface_data_map::iterator it = surface_data_.begin();
        it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != NULL);

      ret = display_->DestroySurface(surface_data->surface_id);
      if(ret != 0) {
        TEST_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
      }
      if (surface_data->file) {
        fclose(surface_data->file);
        surface_data->file = NULL;
      }

      delete surface_data;
      surface_data_.erase (it);
    }
    for (std::map<uint32_t, std::vector<BufInfo*>>::iterator it =
        buf_info.begin(); it != buf_info.end(); ++it) {
      if(it != buf_info.end()) {
        for (std::vector<BufInfo*>::iterator iter = it->second.begin();
            iter != it->second.end(); ++iter) {

          if ((*iter)->buf) {
            ret = munmap( (*iter)->buf,
                (*iter)->buffer_info.alloc_buffer_info.size);
            if(ret != 0) {
              TEST_ERROR("%s:%s munmap Failed!!", TAG, __func__);
            }
          }
          buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
          it->second.erase(iter);
          delete *iter;
        }
      }
      buf_info.erase(it);
    }
    if(!surface_data_.size()) {
      pthread_mutex_unlock(&thread_lock_);
      ret = DeInit(DisplayType::kPrimary);
      if(ret != 0) {
        TEST_ERROR("%s:%s Disconnect Failed!!", TAG, __func__);
      }
    }
  }
exit:
  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
* Test1YUV_1RGB_ExternalBuffer: This will test display of 1 YUV Video and 1 RGB
* where the YUV buffer is allocated by the test app and RBG buffer by
* Display Service
* Api test sequence:
*  - Init
*  - CreateSurface ---- for YUV
*  - AllocateBuffer ---- for YUV
*  - DequeueSurfaceBuffer
*  - CreateSurface ---- for RGB
*  - DequeueSurfaceBuffer
*  - Go to Sleep and run the DisplayVSync thread
*  - DestroySurface ---- for YUV
*  - DestroySurface ---- for RGB
*  - Deallocate Buffer ---- for YUV
*  - DeInit
*/
TEST_F(DisplayGtest, Test1YUV_1RGB_ExternalBuffer) {
  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  int32_t ret;
  uint32_t surface_id;
  SurfaceConfig surface_config;

  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s:%s: Running Test(%s) iteration = %d ", TAG, __func__,
        test_info_->name(), i);
    pthread_mutex_lock(&thread_lock_);

    ret = Init(DisplayType::kPrimary);
    if(ret != 0) {
      TEST_ERROR("%s:%s Init Failed!!", TAG, __func__);
      goto exit;
    }
    {
      memset(&surface_config, 0x0, sizeof surface_config);

      surface_config.width=1920;
      surface_config.height=1080;
      surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
      surface_config.buffer_count=4;
      surface_config.cache=0;
      surface_config.use_buffer=1;

      ret = display_->CreateSurface(surface_config, &surface_id);
      if(ret != 0) {
        TEST_ERROR("%s:%s CreateSurface Failed!!", TAG, __func__);
        goto next;
      }

      SurfaceData* surface_data = new SurfaceData();
      assert(surface_data!=NULL);

      surface_data->surface_id = surface_id;
      std::vector<BufInfo*> buffers;

      for(uint32_t i=0; i<surface_config.buffer_count;i++) {
        BufInfo* new_buf_info = new BufInfo();
        new_buf_info->buffer_info.buffer_config.width =surface_config.width;
        new_buf_info->buffer_info.buffer_config.height = surface_config.height;
        new_buf_info->buffer_info.buffer_config.format =
            (LayerBufferFormat)surface_config.format;
        new_buf_info->buffer_info.buffer_config.buffer_count = 1;
        new_buf_info->buffer_info.buffer_config.cache = surface_config.cache;
        new_buf_info->buffer_info.alloc_buffer_info.fd = -1;
        new_buf_info->buffer_info.alloc_buffer_info.stride = 0;
        new_buf_info->buffer_info.alloc_buffer_info.size = 0;
        new_buf_info->buf_id = i;
        new_buf_info->buf = nullptr;
        buffers.push_back(new_buf_info);
        ret = buffer_allocator_.AllocateBuffer(&new_buf_info->buffer_info);
        if (ret != kErrorNone) {
          TEST_ERROR("%s:%s: AllocateBuffer Failed. Error = %d", TAG, __func__,
              ret);
          ret = display_->DestroySurface(surface_data->surface_id);
          if(ret != 0) {
            TEST_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
          }
          delete surface_data;
          for (std::vector<BufInfo*>::iterator iter = buffers.begin();
              iter != buffers.end(); ++iter) {

            buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
            buffers.erase(iter);
            delete *iter;
          }
          goto next;
        }

      }

      buf_info.insert({surface_id, buffers});

      surface_data->file = fopen("/data/misc/qmmf/YUV/1080p_1.nv12.yuv", "r");
      if (!surface_data->file) {
        TEST_ERROR("%s:%s: Unable to open file", TAG, __func__);
        ret = display_->DestroySurface(surface_data->surface_id);
        if(ret != 0) {
          TEST_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
        }
        delete surface_data;
        for (std::map<uint32_t, std::vector<BufInfo*>>::iterator it =
            buf_info.begin(); it != buf_info.end(); ++it) {
          if(it != buf_info.end()) {
            for (std::vector<BufInfo*>::iterator iter = it->second.begin();
                iter != it->second.end(); ++iter) {
              buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
              it->second.erase(iter);
              delete *iter;
            }
          }
          buf_info.erase(it);
        }
        goto next;
      }

      auto bufinfo = buf_info.find(surface_id);
      BufInfo* new_buf_info = bufinfo->second.at(0);
      BufferInfo *bufferinfo = &new_buf_info->buffer_info;
      surface_data->surface_buffer.plane_info[0].ion_fd =
          bufferinfo->alloc_buffer_info.fd;
      surface_data->surface_buffer.buf_id =0;
      surface_data->surface_buffer.format =
          (SurfaceFormat)bufferinfo->buffer_config.format;
      surface_data->surface_buffer.plane_info[0].stride =
          bufferinfo->alloc_buffer_info.stride;
      surface_data->surface_buffer.plane_info[0].size =
          bufferinfo->alloc_buffer_info.size;
      surface_data->surface_buffer.plane_info[0].width =
          bufferinfo->buffer_config.width;
      surface_data->surface_buffer.plane_info[0].height =
          bufferinfo->buffer_config.height;
      surface_data->surface_buffer.plane_info[0].offset = 0;
      surface_data->surface_buffer.plane_info[0].buf = new_buf_info->buf =
          mmap(NULL, (size_t)surface_data->surface_buffer.plane_info[0].size,
          PROT_READ | PROT_WRITE, MAP_SHARED,
          surface_data->surface_buffer.plane_info[0].ion_fd, 0);
      assert(surface_data->surface_buffer.plane_info[0].buf != NULL);

      uint8_t *src = (uint8_t *)surface_data->surface_buffer.plane_info[0].buf +
          surface_data->surface_buffer.plane_info[0].offset;
      uint32_t offset_temp = 0;
      uint32_t read_len = 0;

      for(uint32_t i=0;i<(surface_data->surface_buffer.plane_info[0].height);i++)
          {
        read_len = fread(src+offset_temp, sizeof(uint8_t),
            surface_data->surface_buffer.plane_info[0].width,
            surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      offset_temp += surface_data->surface_buffer.plane_info[0].stride * 8;
      for(uint32_t i=0;i<(surface_data->surface_buffer.plane_info[0].height)/2;
          i++) {
        read_len = fread(src+offset_temp, sizeof(uint8_t),
            surface_data->surface_buffer.plane_info[0].width,
            surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      surface_data->buffer_ready=1;

      surface_data->surface_param.src_rect = { 0.0, 0.0, 1920.0, 1080.0 };
      surface_data->surface_param.dst_rect = { 0.0, 0.0, 1920.0, 1080.0 };
      surface_data->surface_param.surface_blending =
          SurfaceBlending::kBlendingCoverage;
      surface_data->surface_param.surface_flags.cursor=0;
      surface_data->surface_param.frame_rate=20;
      surface_data->surface_param.z_order = 0;
      surface_data->surface_param.solid_fill_color=0;
      surface_data->surface_param.surface_transform.rotation=0.0f;
      surface_data->surface_param.surface_transform.flip_horizontal=0;
      surface_data->surface_param.surface_transform.flip_vertical=0;

      surface_data_.insert({surface_id, surface_data});
    }
next:

    {
        memset(&surface_config, 0x0, sizeof surface_config);

        surface_config.width=352;
        surface_config.height=288;
        surface_config.format = SurfaceFormat::kFormatBGRA8888;
        surface_config.buffer_count=4;
        surface_config.cache=0;
        surface_config.use_buffer=0;
        ret = display_->CreateSurface(surface_config, &surface_id);
        if(ret != 0) {
          TEST_ERROR("%s:%s CreateSurface Failed!!", TAG, __func__);
          goto sleep;
        }

        SurfaceData* surface_data = new SurfaceData();
        assert(surface_data!=NULL);

        surface_data->surface_id = surface_id;
        ret = display_->DequeueSurfaceBuffer(surface_id,
            surface_data->surface_buffer);

        surface_data->file = fopen("/data/misc/qmmf/Images/fasimo_352x288_bgra_8888.rgb",
            "r");
        if (!surface_data->file) {
          TEST_ERROR("%s:%s: Unable to open file", TAG, __func__);
          ret = display_->DestroySurface(surface_data->surface_id);
          if(ret != 0) {
            TEST_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
          }
          delete surface_data;
          goto sleep;
        }

        int32_t offset=0;
        for(uint32_t i=0;i<surface_data->surface_buffer.plane_info[0].height;
            i++) {
        fread((uint8_t*)surface_data->surface_buffer.plane_info[0].buf
            + surface_data->surface_buffer.plane_info[0].offset + offset,
            sizeof(uint8_t), surface_data->surface_buffer.plane_info[0].width*4,
            surface_data->file);
        offset += ALIGNED_WIDTH(
            surface_data->surface_buffer.plane_info[0].width)*4;
        }
        fclose (surface_data->file);
        surface_data->buffer_ready=1;

        surface_data->surface_param.src_rect = { 0.0, 0.0, 352.0, 288.0 };
        surface_data->surface_param.dst_rect = { 0.0, 0.0, 352.0, 288.0 };
        surface_data->surface_param.surface_blending =
            SurfaceBlending::kBlendingCoverage;
        surface_data->surface_param.surface_flags.cursor=0;
        surface_data->surface_param.frame_rate=20;
        surface_data->surface_param.z_order = 0;
        surface_data->surface_param.solid_fill_color=0;
        surface_data->surface_param.surface_transform.rotation=0;
        surface_data->surface_param.surface_transform.flip_horizontal=0;
        surface_data->surface_param.surface_transform.flip_vertical=0;

        surface_data_.insert({surface_id, surface_data});
      }
    running_ = 1;
    pthread_mutex_unlock(&thread_lock_);
sleep:

    sleep(10);

    pthread_mutex_lock(&thread_lock_);
    for (surface_data_map::iterator it = surface_data_.begin();
        it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != NULL);


      ret = display_->DestroySurface(surface_data->surface_id);
      if(ret != 0) {
        TEST_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
      }
      if (surface_data->file) {
        fclose(surface_data->file);
        surface_data->file = NULL;
      }
      delete surface_data;
      surface_data_.erase (it);
    }
    for (std::map<uint32_t, std::vector<BufInfo*>>::iterator it =
        buf_info.begin(); it != buf_info.end(); ++it) {
      if(it != buf_info.end()) {
        for (std::vector<BufInfo*>::iterator iter = it->second.begin();
            iter != it->second.end(); ++iter) {
          if ((*iter)->buf) {
            ret = munmap( (*iter)->buf, (*iter)->buffer_info.alloc_buffer_info.size);
            if(ret != 0) {
              TEST_ERROR("%s:%s munmap Failed!!", TAG, __func__);
            }
          }
          buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
          it->second.erase(iter);
          delete *iter;
        }
      }
      buf_info.erase(it);
    }
    if(!surface_data_.size()) {
      pthread_mutex_unlock(&thread_lock_);
      ret = DeInit(DisplayType::kPrimary);
      if(ret != 0) {
        TEST_ERROR("%s:%s Disconnect Failed!!", TAG, __func__);
      }
    }
  }
exit:
  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

void DisplayGtest::DisplayCallbackHandler(DisplayEventType event_type,
    void *event_data, size_t event_data_size) {
  TEST_INFO("%s:%s Enter ", TAG, __func__);
  TEST_INFO("%s:%s Exit ", TAG, __func__);
}

void DisplayGtest::SessionCallbackHandler(DisplayEventType event_type,
    void *event_data, size_t event_data_size) {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

void DisplayGtest::DisplayVSyncHandler(int64_t time_stamp) {
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

/*
* This thread will queue and dequeue Buffers for all the surfaces
* Api test sequence:
*  - QueueSurfaceBuffer
*  - DequeueSurfaceBuffer
*  - fread
*/
void* DisplayGtest::DisplayVSync(void *userdata) {

  DisplayGtest *displaygtest = reinterpret_cast<DisplayGtest *>(userdata);
  if (NULL == displaygtest) {
    return NULL;
  }
  bool run = true;

  while (run) {
    pthread_mutex_lock(&displaygtest->thread_lock_);
    run = displaygtest->running_;

    int32_t ret;
    static int32_t buf_id = -1;
    for (surface_data_map::iterator it = displaygtest->surface_data_.begin();
        it != displaygtest->surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != NULL);

      if (surface_data->buffer_ready) {
        ret = displaygtest->display_->QueueSurfaceBuffer(
            surface_data->surface_id, surface_data->surface_buffer,
            surface_data->surface_param);
        buf_id = surface_data->surface_buffer.buf_id;
        surface_data->buffer_ready = 0;
        if(ret != 0) {
          TEST_ERROR("%s:%s QueueSurfaceBuffer Failed!!", TAG, __func__);
        }
      }
    }

    for (surface_data_map::iterator it = displaygtest->surface_data_.begin();
        it != displaygtest->surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != NULL);

      if (!surface_data->buffer_ready) {
        ret = displaygtest->display_->DequeueSurfaceBuffer(
            surface_data->surface_id,
            surface_data->surface_buffer);
        if(ret != 0) {
          TEST_ERROR("%s:%s DequeueSurfaceBuffer Failed!!", TAG, __func__);
        }

        if (surface_data->surface_buffer.buf_id == -1){
          TEST_INFO("%s:%s No buf available !!", TAG, __func__);
          if (displaygtest->buf_info.size()) {

            auto bufinfo = displaygtest->buf_info.find(surface_data->surface_id);
            BufInfo* new_buf_info = bufinfo->second.at((buf_id+1)%
                bufinfo->second.size());
            BufferInfo *bufferinfo = &new_buf_info->buffer_info;
            assert(bufferinfo!=NULL);
            surface_data->surface_buffer.plane_info[0].ion_fd =
                bufferinfo->alloc_buffer_info.fd;
            surface_data->surface_buffer.buf_id = (buf_id+1)%
                bufinfo->second.size();
            surface_data->surface_buffer.format =
                (SurfaceFormat)bufferinfo->buffer_config.format;
            surface_data->surface_buffer.plane_info[0].stride =
                bufferinfo->alloc_buffer_info.stride;
            surface_data->surface_buffer.plane_info[0].size =
                bufferinfo->alloc_buffer_info.size;
            surface_data->surface_buffer.plane_info[0].width =
                bufferinfo->buffer_config.width;
            surface_data->surface_buffer.plane_info[0].height =
                bufferinfo->buffer_config.height;
            surface_data->surface_buffer.plane_info[0].offset = 0;
            surface_data->surface_buffer.plane_info[0].buf = new_buf_info->buf =
                mmap(NULL, (size_t)surface_data->surface_buffer.plane_info[0].size,
                PROT_READ | PROT_WRITE, MAP_SHARED,
                surface_data->surface_buffer.plane_info[0].ion_fd, 0);

            assert(surface_data->surface_buffer.plane_info[0].buf != NULL);
          }
          else {
            continue;
          }
        }
        uint32_t read_len = 0;

        if(surface_data->surface_buffer.format ==
            (SurfaceFormat)kFormatYCbCr420SemiPlanarVenus) {
          if (surface_data->file) {
            uint8_t *src = (uint8_t *)surface_data->
              surface_buffer.plane_info[0].buf +
              surface_data->surface_buffer.plane_info[0].offset;
            uint32_t offset_temp = 0;
            for(uint32_t i=0;i<(surface_data->surface_buffer.plane_info[0].height);i++) {
              read_len = fread(src+offset_temp, sizeof(uint8_t),
                  surface_data->surface_buffer.plane_info[0].width,
                  surface_data->file);
              assert(read_len == surface_data->surface_buffer.plane_info[0].width);
              offset_temp += surface_data->surface_buffer.plane_info[0].stride;
            }
            offset_temp += surface_data->surface_buffer.plane_info[0].stride * 8;
            for(uint32_t i=0;i<(surface_data->surface_buffer.plane_info[0].height)/2;i++) {
              read_len = fread(src+offset_temp, sizeof(uint8_t),
                  surface_data->surface_buffer.plane_info[0].width,
                  surface_data->file);
              assert(read_len == surface_data->surface_buffer.plane_info[0].width);
              offset_temp += surface_data->surface_buffer.plane_info[0].stride;
            }
          }
        }
        else if((surface_data->surface_buffer.format ==
            (SurfaceFormat)kFormatBGRA8888)|| (surface_data->surface_buffer.format
            == (SurfaceFormat)kFormatRGBA8888)){
          surface_data->file = fopen("/data/misc/qmmf/Images/fasimo_352x288_bgra_8888.rgb",
                "r");
          if (!surface_data->file) {
            TEST_ERROR("%s:%s: Unable to open file", TAG, __func__);
          }
          int32_t offset=0;
          for(uint32_t i=0;i<surface_data->surface_buffer.plane_info[0].height;
              i++) {
          fread(static_cast<uint8_t*>(surface_data->surface_buffer.
              plane_info[0].buf) +
              surface_data->surface_buffer.plane_info[0].offset + offset,
              sizeof(uint8_t), surface_data->surface_buffer.plane_info[0].width
              *4, surface_data->file);
          offset += ALIGNED_WIDTH(
              surface_data->surface_buffer.plane_info[0].width)*4;
          }
          fclose (surface_data->file);
          surface_data->file = NULL;
        }
        surface_data->buffer_ready=1;
      }
    }
    pthread_mutex_unlock(&displaygtest->thread_lock_);
    usleep(33333);
  }

  return nullptr;
}

