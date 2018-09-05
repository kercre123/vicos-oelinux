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

#define LOG_TAG "DisplayGTest"

#include <assert.h>
#include <camera/CameraMetadata.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <utils/Errors.h>
#include <utils/Log.h>
#include <utils/String8.h>

#include <hardware/hardware.h>
#include "display/test/gtest/qmmf_display_gtest.h"

//#define DEBUG
#define TEST_INFO(fmt, args...) ALOGD(fmt, ##args)
#define TEST_ERROR(fmt, args...) ALOGE(fmt, ##args)
#ifdef DEBUG
#define TEST_DBG TEST_INFO
#else
#define TEST_DBG(...) ((void)0)
#endif

#define ALIGNED_WIDTH(x) ((x) + ((x % 64) ? (64 - (x % 64)) : 0))

void DisplayGtest::SetUp() {
  TEST_INFO("%s: Enter ", __func__);

  test_info_ = ::testing::UnitTest::GetInstance()->current_test_info();

  display_status_cb_.EventCb = [&](DisplayEventType event_type,
                                   void* event_data, size_t event_data_size) {
    DisplayCallbackHandler(event_type, event_data, event_data_size);
  };

  display_status_cb_.VSyncCb = [&](int64_t time_stamp) {
    DisplayVSyncHandler(time_stamp);
  };

  char prop_val[PROPERTY_VALUE_MAX];
  property_get(PROP_N_ITERATIONS, prop_val, DEFAULT_ITERATIONS);
  iteration_count_ = atoi(prop_val);

  TEST_INFO("%s Exit ", __func__);
}

void DisplayGtest::TearDown() {
  TEST_INFO("%s: Enter ", __func__);
  TEST_INFO("%s: Exit ", __func__);
}

int32_t DisplayGtest::Init(DisplayType display_type) {
  display_ = new Display();
  assert(display_ != nullptr);
  auto ret = display_->Connect();
  if (ret != 0) {
    TEST_ERROR("%s: Connect Failed!!", __func__);
    return ret;
  }

  ret = display_->CreateDisplay(display_type, display_status_cb_);
  if (ret != 0) {
    display_->Disconnect();
    TEST_ERROR("%s CreateDisplay Failed!!", __func__);
    return ret;
  }

  display_thread_ = new std::thread(DisplayGtest::DisplayThreadEntry, this);
  assert(display_thread_ != nullptr);

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
  TEST_INFO("%s: Enter", __func__);

  display_thread_->join();
  delete display_thread_;
  display_thread_ = nullptr;

  std::lock_guard<std::mutex> lock(lock_);
  running_ = 0;

  auto ret = display_->DestroyDisplay(display_type);
  if (ret != 0) {
    TEST_ERROR("%s: DestroyDisplay Failed!!", __func__);
  }
  ret = display_->Disconnect();

  this->surface_data_.clear();
  if (display_ != nullptr) {
    TEST_INFO("%s: DELETE display_:%p", __func__, display_);
    delete display_;
    display_ = nullptr;
  }
  TEST_INFO("%s: Exit", __func__);
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
TEST_F(DisplayGtest, TestYUV) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  int32_t ret;
  uint32_t surface_id;
  SurfaceConfig surface_config;

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);
    std::unique_lock<std::mutex> lock(lock_);

    ret = Init(DisplayType::kPrimary);
    if (ret != 0) {
      TEST_ERROR("%s: Init Failed!!", __func__);
      goto exit;
    }

    memset(&surface_config, 0x0, sizeof surface_config);

    surface_config.width = 1920;
    surface_config.height = 1080;
    surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
    surface_config.buffer_count = 4;
    surface_config.cache = 0;
    surface_config.use_buffer = 0;
    surface_config.z_order = 1;
    ret = display_->CreateSurface(surface_config, &surface_id);
    if (ret != 0) {
      TEST_ERROR("%s: CreateSurface Failed!!", __func__);
      ret = DeInit(DisplayType::kPrimary);
      goto exit;
    }

    SurfaceData* surface_data = new SurfaceData();
    assert(surface_data != nullptr);

    surface_data->surface_id = surface_id;
    ret = display_->DequeueSurfaceBuffer(surface_id,
                                         surface_data->surface_buffer);
    surface_data->file = fopen("/data/misc/qmmf/YUV/1080p_1.nv12.yuv", "r");
    if (!surface_data->file) {
      TEST_ERROR("%s: Unable to open file", __func__);
      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      delete surface_data;
      surface_data = nullptr;
      ret = DeInit(DisplayType::kPrimary);
      goto exit;
    }

    uint8_t* src =
        static_cast<uint8_t*>(surface_data->surface_buffer.plane_info[0].buf) +
        surface_data->surface_buffer.plane_info[0].offset;
    uint32_t offset_temp = 0;
    uint32_t read_len = 0;

    for (uint32_t i = 0;
         i < (surface_data->surface_buffer.plane_info[0].height); i++) {
      read_len = fread(src + offset_temp, sizeof(uint8_t),
                       surface_data->surface_buffer.plane_info[0].width,
                       surface_data->file);
      assert(read_len == surface_data->surface_buffer.plane_info[0].width);
      offset_temp += surface_data->surface_buffer.plane_info[0].stride;
    }
    offset_temp += surface_data->surface_buffer.plane_info[0].stride * 8;
    for (uint32_t i = 0;
         i < (surface_data->surface_buffer.plane_info[0].height) / 2; i++) {
      read_len = fread(src + offset_temp, sizeof(uint8_t),
                       surface_data->surface_buffer.plane_info[0].width,
                       surface_data->file);
      assert(read_len == surface_data->surface_buffer.plane_info[0].width);
      offset_temp += surface_data->surface_buffer.plane_info[0].stride;
    }
    surface_data->buffer_ready = 1;

    surface_data->surface_param.src_rect = {0.0, 0.0,
        static_cast<float>(surface_config.width),
        static_cast<float>(surface_config.height)};
    surface_data->surface_param.dst_rect = {0.0, 0.0,
        static_cast<float>(surface_config.width),
        static_cast<float>(surface_config.height)};
    surface_data->surface_param.surface_blending =
        SurfaceBlending::kBlendingCoverage;
    surface_data->surface_param.surface_flags.cursor = 0;
    surface_data->surface_param.frame_rate = 20;
    surface_data->surface_param.solid_fill_color = 0;
    surface_data->surface_param.surface_transform.rotation = 0.0f;
    surface_data->surface_param.surface_transform.flip_horizontal = 0;
    surface_data->surface_param.surface_transform.flip_vertical = 0;

    surface_data_.insert({surface_id, surface_data});
    running_ = 1;

    lock.unlock();
    sleep(8);

    lock.lock();
    running_ = 0;
    for (surface_data_map::iterator it = surface_data_.begin();
         it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != nullptr);

      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      if (surface_data->file) {
        fclose(surface_data->file);
        surface_data->file = nullptr;
      }
      delete surface_data;
      surface_data = nullptr;
    }
    surface_data_.clear();
    lock.unlock();
    if (!surface_data_.size()) {
      ret = DeInit(DisplayType::kPrimary);
      if (ret != 0) {
        TEST_ERROR("%s: Disconnect Failed!!", __func__);
      }
    }
  }
exit:

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* TestYUVWithRotation: This test case will test display of 1 YUV video rotated
* by 90 Degree.
* Api test sequence:
*  - Init
*  - CreateSurface
*  - DequeueSurfaceBuffer
*  - Go to Sleep and run the DisplayVSync thread
*  - DestroySurface
*  - DeInit
*/
TEST_F(DisplayGtest, TestYUVWithRotation) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  int32_t ret;
  uint32_t surface_id;
  SurfaceConfig surface_config;

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);
    std::unique_lock<std::mutex> lock(lock_);

    ret = Init(DisplayType::kPrimary);
    if (ret != 0) {
      TEST_ERROR("%s: Init Failed!!", __func__);
      goto exit;
    }

    memset(&surface_config, 0x0, sizeof surface_config);

    surface_config.width = 1920;
    surface_config.height = 1080;
    surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
    surface_config.buffer_count = 4;
    surface_config.cache = 0;
    surface_config.use_buffer = 0;
    surface_config.z_order = 1;
    surface_config.surface_transform.rotation = 90.0f;
    surface_config.surface_transform.flip_horizontal = 0;
    surface_config.surface_transform.flip_vertical = 0;
    ret = display_->CreateSurface(surface_config, &surface_id);
    if (ret != 0) {
      TEST_ERROR("%s: CreateSurface Failed!!", __func__);
      ret = DeInit(DisplayType::kPrimary);
      goto exit;
    }

    SurfaceData* surface_data = new SurfaceData();
    assert(surface_data != nullptr);

    surface_data->surface_id = surface_id;
    ret = display_->DequeueSurfaceBuffer(surface_id,
                                         surface_data->surface_buffer);
    surface_data->file = fopen("/data/misc/qmmf/YUV/1080p_1.nv12.yuv", "r");
    if (!surface_data->file) {
      TEST_ERROR("%s: Unable to open file", __func__);
      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      delete surface_data;
      surface_data = nullptr;
      ret = DeInit(DisplayType::kPrimary);
      goto exit;
    }

    uint8_t* src =
        static_cast<uint8_t*>(surface_data->surface_buffer.plane_info[0].buf) +
        surface_data->surface_buffer.plane_info[0].offset;
    uint32_t offset_temp = 0;
    uint32_t read_len = 0;

    for (uint32_t i = 0;
         i < (surface_data->surface_buffer.plane_info[0].height); i++) {
      read_len = fread(src + offset_temp, sizeof(uint8_t),
                       surface_data->surface_buffer.plane_info[0].width,
                       surface_data->file);
      assert(read_len == surface_data->surface_buffer.plane_info[0].width);
      offset_temp += surface_data->surface_buffer.plane_info[0].stride;
    }
    offset_temp += surface_data->surface_buffer.plane_info[0].stride * 8;
    for (uint32_t i = 0;
         i < (surface_data->surface_buffer.plane_info[0].height) / 2; i++) {
      read_len = fread(src + offset_temp, sizeof(uint8_t),
                       surface_data->surface_buffer.plane_info[0].width,
                       surface_data->file);
      assert(read_len == surface_data->surface_buffer.plane_info[0].width);
      offset_temp += surface_data->surface_buffer.plane_info[0].stride;
    }
    surface_data->buffer_ready = 1;

    surface_data->surface_param.src_rect = {0.0, 0.0,
        static_cast<float>(surface_config.width),
        static_cast<float>(surface_config.height)};
    surface_data->surface_param.dst_rect = {0.0, 0.0,
        static_cast<float>(surface_config.width),
        static_cast<float>(surface_config.height)};
    surface_data->surface_param.surface_blending =
        SurfaceBlending::kBlendingCoverage;
    surface_data->surface_param.surface_flags.cursor = 0;
    surface_data->surface_param.frame_rate = 20;
    surface_data->surface_param.solid_fill_color = 0;
    surface_data->surface_param.surface_transform.rotation = 90.0f;
    surface_data->surface_param.surface_transform.flip_horizontal = 0;
    surface_data->surface_param.surface_transform.flip_vertical = 0;

    surface_data_.insert({surface_id, surface_data});
    running_ = 1;

    lock.unlock();
    sleep(8);

    lock.lock();
    running_ = 0;
    for (surface_data_map::iterator it = surface_data_.begin();
         it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != nullptr);

      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      if (surface_data->file) {
        fclose(surface_data->file);
        surface_data->file = nullptr;
      }
      delete surface_data;
      surface_data = nullptr;
    }
    surface_data_.clear();
    lock.unlock();
    if (!surface_data_.size()) {
      ret = DeInit(DisplayType::kPrimary);
      if (ret != 0) {
        TEST_ERROR("%s: Disconnect Failed!!", __func__);
      }
    }
  }
exit:

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Test1BGRA: This test case will test display of 1 RGB Image.
* Api test sequence:
*  - Init
*  - CreateSurface
*  - DequeueSurfaceBuffer
*  - Go to Sleep and run the DisplayVSync thread
*  - DestroySurface
*  - DeInit
*/
TEST_F(DisplayGtest, TestBGRA) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  int32_t ret;
  uint32_t surface_id;
  SurfaceConfig surface_config;

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);
    std::unique_lock<std::mutex> lock(lock_);

    ret = Init(DisplayType::kPrimary);
    if (ret != 0) {
      TEST_ERROR("%s: Init Failed!!", __func__);
      goto exit;
    }

    memset(&surface_config, 0x0, sizeof surface_config);

    surface_config.width = 352;
    surface_config.height = 288;
    surface_config.format = SurfaceFormat::kFormatBGRA8888;
    surface_config.buffer_count = 4;
    surface_config.cache = 0;
    surface_config.use_buffer = 0;
    surface_config.z_order = 1;
    ret = display_->CreateSurface(surface_config, &surface_id);
    if (ret != 0) {
      TEST_ERROR("%s: CreateSurface Failed!!", __func__);
      ret = DeInit(DisplayType::kPrimary);
      goto exit;
    }

    SurfaceData* surface_data = new SurfaceData();
    assert(surface_data != nullptr);

    surface_data->surface_id = surface_id;
    ret = display_->DequeueSurfaceBuffer(surface_id,
                                         surface_data->surface_buffer);

    surface_data->file =
        fopen("/data/misc/qmmf/Images/fasimo_352x288_bgra_8888.rgb", "r");
    if (!surface_data->file) {
      TEST_ERROR("%s: Unable to open file", __func__);
      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      delete surface_data;
      surface_data = nullptr;
      ret = DeInit(DisplayType::kPrimary);
      goto exit;
    }

    int32_t offset = 0;
    for (uint32_t i = 0; i < surface_data->surface_buffer.plane_info[0].height;
         i++) {
      fread(static_cast<uint8_t*>(
                surface_data->surface_buffer.plane_info[0].buf) +
                surface_data->surface_buffer.plane_info[0].offset + offset,
            sizeof(uint8_t),
            surface_data->surface_buffer.plane_info[0].width * 4,
            surface_data->file);
      offset +=
          ALIGNED_WIDTH(surface_data->surface_buffer.plane_info[0].width) * 4;
    }
    fclose(surface_data->file);
    surface_data->buffer_ready = 1;

    surface_data->surface_param.src_rect = {0.0, 0.0,
        static_cast<float>(surface_config.width),
        static_cast<float>(surface_config.height)};
    surface_data->surface_param.dst_rect = {0.0, 0.0,
        static_cast<float>(surface_config.width),
        static_cast<float>(surface_config.height)};
    surface_data->surface_param.surface_blending =
        SurfaceBlending::kBlendingCoverage;
    surface_data->surface_param.surface_flags.cursor = 0;
    surface_data->surface_param.frame_rate = 20;
    surface_data->surface_param.solid_fill_color = 0;
    surface_data->surface_param.surface_transform.rotation = 0.0f;
    surface_data->surface_param.surface_transform.flip_horizontal = 0;
    surface_data->surface_param.surface_transform.flip_vertical = 0;

    surface_data_.insert({surface_id, surface_data});
    running_ = 1;
    lock.unlock();

    sleep(8);

    lock.lock();
    running_ = 0;
    for (surface_data_map::iterator it = surface_data_.begin();
         it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != nullptr);

      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      delete surface_data;
      surface_data = nullptr;
    }
    surface_data_.clear();
    lock.unlock();
    if (!surface_data_.size()) {
      ret = DeInit(DisplayType::kPrimary);
      if (ret != 0) {
        TEST_ERROR("%s: Disconnect Failed!!", __func__);
      }
    }
  }
exit:

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* TestRGBAWithRotation: This test case will test display of 1 RGB Image rotated
* by 90 Degree.
* Api test sequence:
*  - Init
*  - CreateSurface
*  - DequeueSurfaceBuffer
*  - Go to Sleep and run the DisplayVSync thread
*  - DestroySurface
*  - DeInit
*/
TEST_F(DisplayGtest, TestRGBAWithRotation) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  int32_t ret;
  uint32_t surface_id;
  SurfaceConfig surface_config;

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);
    std::unique_lock<std::mutex> lock(lock_);

    ret = Init(DisplayType::kPrimary);
    if (ret != 0) {
      TEST_ERROR("%s: Init Failed!!", __func__);
      goto exit;
    }

    memset(&surface_config, 0x0, sizeof surface_config);

    surface_config.width = 352;
    surface_config.height = 288;
    surface_config.format = SurfaceFormat::kFormatRGBA8888;
    surface_config.buffer_count = 4;
    surface_config.cache = 0;
    surface_config.use_buffer = 0;
    surface_config.z_order = 1;
    surface_config.surface_transform.rotation = 90.0f;
    surface_config.surface_transform.flip_horizontal = 0;
    surface_config.surface_transform.flip_vertical = 0;
    ret = display_->CreateSurface(surface_config, &surface_id);
    if (ret != 0) {
      TEST_ERROR("%s: CreateSurface Failed!!", __func__);
      ret = DeInit(DisplayType::kPrimary);
      goto exit;
    }

    SurfaceData* surface_data = new SurfaceData();
    assert(surface_data != nullptr);

    surface_data->surface_id = surface_id;
    ret = display_->DequeueSurfaceBuffer(surface_id,
                                         surface_data->surface_buffer);

    surface_data->file =
        fopen("/data/misc/qmmf/Images/fasimo_352x288_bgra_8888.rgb", "r");
    if (!surface_data->file) {
      TEST_ERROR("%s: Unable to open file", __func__);
      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      delete surface_data;
      surface_data = nullptr;
      ret = DeInit(DisplayType::kPrimary);
      goto exit;
    }

    int32_t offset = 0;
    for (uint32_t i = 0; i < surface_data->surface_buffer.plane_info[0].height;
         i++) {
      fread(static_cast<uint8_t*>(
                surface_data->surface_buffer.plane_info[0].buf) +
                surface_data->surface_buffer.plane_info[0].offset + offset,
            sizeof(uint8_t),
            surface_data->surface_buffer.plane_info[0].width * 4,
            surface_data->file);
      offset +=
          ALIGNED_WIDTH(surface_data->surface_buffer.plane_info[0].width) * 4;
    }
    fclose(surface_data->file);
    surface_data->buffer_ready = 1;

    surface_data->surface_param.src_rect = {0.0, 0.0,
        static_cast<float>(surface_config.width),
        static_cast<float>(surface_config.height)};
    surface_data->surface_param.dst_rect = {0.0, 0.0,
        static_cast<float>(surface_config.width),
        static_cast<float>(surface_config.height)};
    surface_data->surface_param.surface_blending =
        SurfaceBlending::kBlendingCoverage;
    surface_data->surface_param.surface_flags.cursor = 0;
    surface_data->surface_param.frame_rate = 20;
    surface_data->surface_param.solid_fill_color = 0;
    surface_data->surface_param.surface_transform.rotation = 90.0f;
    surface_data->surface_param.surface_transform.flip_horizontal = 0;
    surface_data->surface_param.surface_transform.flip_vertical = 0;

    surface_data_.insert({surface_id, surface_data});
    running_ = 1;
    lock.unlock();

    sleep(8);

    lock.lock();
    running_ = 0;
    for (surface_data_map::iterator it = surface_data_.begin();
         it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != nullptr);

      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      delete surface_data;
      surface_data = nullptr;
    }
    surface_data_.clear();
    lock.unlock();
    if (!surface_data_.size()) {
      ret = DeInit(DisplayType::kPrimary);
      if (ret != 0) {
        TEST_ERROR("%s: Disconnect Failed!!", __func__);
      }
    }
  }
exit:

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
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
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  int32_t ret;
  uint32_t surface_id;
  SurfaceConfig surface_config;

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);
    std::unique_lock<std::mutex> lock(lock_);

    ret = Init(DisplayType::kPrimary);
    if (ret != 0) {
      TEST_ERROR("%s: Init Failed!!", __func__);
      goto exit;
    }

    {
      memset(&surface_config, 0x0, sizeof surface_config);

      surface_config.width = 1920;
      surface_config.height = 1080;
      surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
      surface_config.buffer_count = 4;
      surface_config.cache = 0;
      surface_config.use_buffer = 0;
      surface_config.z_order = 1;
      ret = display_->CreateSurface(surface_config, &surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: CreateSurface Failed!!", __func__);
        goto next;
      }

      SurfaceData* surface_data = new SurfaceData();
      assert(surface_data != nullptr);

      surface_data->surface_id = surface_id;
      ret = display_->DequeueSurfaceBuffer(surface_id,
                                           surface_data->surface_buffer);
      surface_data->file = fopen("/data/misc/qmmf/YUV/1080p_1.nv12.yuv", "r");
      if (!surface_data->file) {
        TEST_ERROR("%s: Unable to open file", __func__);
        ret = display_->DestroySurface(surface_data->surface_id);
        if (ret != 0) {
          TEST_ERROR("%s: DestroySurface Failed!!", __func__);
        }
        delete surface_data;
        surface_data = nullptr;
        goto next;
      }

      uint8_t* src = static_cast<uint8_t*>(
                         surface_data->surface_buffer.plane_info[0].buf) +
                     surface_data->surface_buffer.plane_info[0].offset;
      uint32_t offset_temp = 0;
      uint32_t read_len = 0;

      for (uint32_t i = 0;
           i < (surface_data->surface_buffer.plane_info[0].height); i++) {
        read_len = fread(src + offset_temp, sizeof(uint8_t),
                         surface_data->surface_buffer.plane_info[0].width,
                         surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      offset_temp += surface_data->surface_buffer.plane_info[0].stride * 8;
      for (uint32_t i = 0;
           i < (surface_data->surface_buffer.plane_info[0].height) / 2; i++) {
        read_len = fread(src + offset_temp, sizeof(uint8_t),
                         surface_data->surface_buffer.plane_info[0].width,
                         surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      surface_data->buffer_ready = 1;

      surface_data->surface_param.src_rect = {0.0, 0.0,
          static_cast<float>(surface_config.width),
          static_cast<float>(surface_config.height)};
      surface_data->surface_param.dst_rect = {0.0, 0.0,
          static_cast<float>(surface_config.width),
          static_cast<float>(surface_config.height)};
      surface_data->surface_param.surface_blending =
          SurfaceBlending::kBlendingCoverage;
      surface_data->surface_param.surface_flags.cursor = 0;
      surface_data->surface_param.frame_rate = 20;
      surface_data->surface_param.solid_fill_color = 0;
      surface_data->surface_param.surface_transform.rotation = 0.0f;
      surface_data->surface_param.surface_transform.flip_horizontal = 0;
      surface_data->surface_param.surface_transform.flip_vertical = 0;

      surface_data_.insert({surface_id, surface_data});
    }
  next:

  {
    memset(&surface_config, 0x0, sizeof surface_config);

    surface_config.width = 352;
    surface_config.height = 288;
    surface_config.format = SurfaceFormat::kFormatBGRA8888;
    surface_config.buffer_count = 4;
    surface_config.cache = 0;
    surface_config.use_buffer = 0;
    surface_config.z_order = 2;
    ret = display_->CreateSurface(surface_config, &surface_id);
    if (ret != 0) {
      TEST_ERROR("%s: CreateSurface Failed!!", __func__);
      goto next2;
    }

    SurfaceData* surface_data = new SurfaceData();
    assert(surface_data != nullptr);

    surface_data->surface_id = surface_id;
    ret = display_->DequeueSurfaceBuffer(surface_id,
                                         surface_data->surface_buffer);

    surface_data->file =
        fopen("/data/misc/qmmf/Images/fasimo_352x288_bgra_8888.rgb", "r");
    if (!surface_data->file) {
      TEST_ERROR("%s: Unable to open file", __func__);
      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      delete surface_data;
      surface_data = nullptr;
      ret = DeInit(DisplayType::kPrimary);
      goto next2;
    }

    int32_t offset = 0;
    for (uint32_t i = 0; i < surface_data->surface_buffer.plane_info[0].height;
         i++) {
      fread(static_cast<uint8_t*>(
                surface_data->surface_buffer.plane_info[0].buf) +
                surface_data->surface_buffer.plane_info[0].offset + offset,
            sizeof(uint8_t),
            surface_data->surface_buffer.plane_info[0].width * 4,
            surface_data->file);
      offset +=
          ALIGNED_WIDTH(surface_data->surface_buffer.plane_info[0].width) * 4;
    }
    fclose(surface_data->file);
    surface_data->buffer_ready = 1;

    surface_data->surface_param.src_rect = {0.0, 0.0,
        static_cast<float>(surface_config.width),
        static_cast<float>(surface_config.height)};
    surface_data->surface_param.dst_rect = {0.0, 0.0,
        static_cast<float>(surface_config.width),
        static_cast<float>(surface_config.height)};
    surface_data->surface_param.surface_blending =
        SurfaceBlending::kBlendingCoverage;
    surface_data->surface_param.surface_flags.cursor = 0;
    surface_data->surface_param.frame_rate = 10;
    surface_data->surface_param.solid_fill_color = 0;
    surface_data->surface_param.surface_transform.rotation = 0.0f;
    surface_data->surface_param.surface_transform.flip_horizontal = 0;
    surface_data->surface_param.surface_transform.flip_vertical = 0;

    surface_data_.insert({surface_id, surface_data});
  }
    running_ = 1;
    lock.unlock();

  next2:

    sleep(8);

    lock.lock();
    running_ = 0;
    for (surface_data_map::iterator it = surface_data_.begin();
         it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != nullptr);

      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      delete surface_data;
      surface_data = nullptr;
    }
    surface_data_.clear();
    lock.unlock();
    if (!surface_data_.size()) {
      ret = DeInit(DisplayType::kPrimary);
      if (ret != 0) {
        TEST_ERROR("%s: Disconnect Failed!!", __func__);
      }
    }
  }
exit:
  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Test1YUV_Rotated_1RGB: This will test display of 1 YUV Video which is rotated
* by 90 Degree and 1 RGB Image.
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
TEST_F(DisplayGtest, Test1YUV_Rotated_1RGB) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  int32_t ret;
  uint32_t surface_id;
  SurfaceConfig surface_config;

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);
    std::unique_lock<std::mutex> lock(lock_);

    ret = Init(DisplayType::kPrimary);
    if (ret != 0) {
      TEST_ERROR("%s: Init Failed!!", __func__);
      goto exit;
    }

    {
      memset(&surface_config, 0x0, sizeof surface_config);

      surface_config.width = 1920;
      surface_config.height = 1080;
      surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
      surface_config.buffer_count = 4;
      surface_config.cache = 0;
      surface_config.use_buffer = 0;
      surface_config.z_order = 1;
      surface_config.surface_transform.rotation = 90.0f;
      surface_config.surface_transform.flip_horizontal = 0;
      surface_config.surface_transform.flip_vertical = 0;
      ret = display_->CreateSurface(surface_config, &surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: CreateSurface Failed!!", __func__);
        goto next;
      }

      SurfaceData* surface_data = new SurfaceData();
      assert(surface_data != nullptr);

      surface_data->surface_id = surface_id;
      ret = display_->DequeueSurfaceBuffer(surface_id,
                                           surface_data->surface_buffer);
      surface_data->file = fopen("/data/misc/qmmf/YUV/1080p_1.nv12.yuv", "r");
      if (!surface_data->file) {
        TEST_ERROR("%s: Unable to open file", __func__);
        ret = display_->DestroySurface(surface_data->surface_id);
        if (ret != 0) {
          TEST_ERROR("%s: DestroySurface Failed!!", __func__);
        }
        delete surface_data;
        surface_data = nullptr;
        goto next;
      }

      uint8_t* src = static_cast<uint8_t*>(
                         surface_data->surface_buffer.plane_info[0].buf) +
                     surface_data->surface_buffer.plane_info[0].offset;
      uint32_t offset_temp = 0;
      uint32_t read_len = 0;

      for (uint32_t i = 0;
           i < (surface_data->surface_buffer.plane_info[0].height); i++) {
        read_len = fread(src + offset_temp, sizeof(uint8_t),
                         surface_data->surface_buffer.plane_info[0].width,
                         surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      offset_temp += surface_data->surface_buffer.plane_info[0].stride * 8;
      for (uint32_t i = 0;
           i < (surface_data->surface_buffer.plane_info[0].height) / 2; i++) {
        read_len = fread(src + offset_temp, sizeof(uint8_t),
                         surface_data->surface_buffer.plane_info[0].width,
                         surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      surface_data->buffer_ready = 1;

      surface_data->surface_param.src_rect = {0.0, 0.0,
          static_cast<float>(surface_config.width),
          static_cast<float>(surface_config.height)};
      surface_data->surface_param.dst_rect = {0.0, 0.0,
          static_cast<float>(surface_config.width),
          static_cast<float>(surface_config.height)};
      surface_data->surface_param.surface_blending =
          SurfaceBlending::kBlendingCoverage;
      surface_data->surface_param.surface_flags.cursor = 0;
      surface_data->surface_param.frame_rate = 20;
      surface_data->surface_param.solid_fill_color = 0;
      surface_data->surface_param.surface_transform.rotation = 90.0f;
      surface_data->surface_param.surface_transform.flip_horizontal = 0;
      surface_data->surface_param.surface_transform.flip_vertical = 0;

      surface_data_.insert({surface_id, surface_data});
    }
  next:

  {
    memset(&surface_config, 0x0, sizeof surface_config);

    surface_config.width = 352;
    surface_config.height = 288;
    surface_config.format = SurfaceFormat::kFormatBGRA8888;
    surface_config.buffer_count = 4;
    surface_config.cache = 0;
    surface_config.use_buffer = 0;
    surface_config.z_order = 2;
    surface_config.surface_transform.rotation = 0.0f;
    surface_config.surface_transform.flip_horizontal = 0;
    surface_config.surface_transform.flip_vertical = 0;
    ret = display_->CreateSurface(surface_config, &surface_id);
    if (ret != 0) {
      TEST_ERROR("%s: CreateSurface Failed!!", __func__);
      goto next2;
    }

    SurfaceData* surface_data = new SurfaceData();
    assert(surface_data != nullptr);

    surface_data->surface_id = surface_id;
    ret = display_->DequeueSurfaceBuffer(surface_id,
                                         surface_data->surface_buffer);

    surface_data->file =
        fopen("/data/misc/qmmf/Images/fasimo_352x288_bgra_8888.rgb", "r");
    if (!surface_data->file) {
      TEST_ERROR("%s: Unable to open file", __func__);
      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      delete surface_data;
      surface_data = nullptr;
      ret = DeInit(DisplayType::kPrimary);
      goto next2;
    }

    int32_t offset = 0;
    for (uint32_t i = 0; i < surface_data->surface_buffer.plane_info[0].height;
         i++) {
      fread(static_cast<uint8_t*>(
                surface_data->surface_buffer.plane_info[0].buf) +
                surface_data->surface_buffer.plane_info[0].offset + offset,
            sizeof(uint8_t),
            surface_data->surface_buffer.plane_info[0].width * 4,
            surface_data->file);
      offset +=
          ALIGNED_WIDTH(surface_data->surface_buffer.plane_info[0].width) * 4;
    }
    fclose(surface_data->file);
    surface_data->buffer_ready = 1;

    surface_data->surface_param.src_rect = {0.0, 0.0,
        static_cast<float>(surface_config.width),
        static_cast<float>(surface_config.height)};
    surface_data->surface_param.dst_rect = {0.0, 0.0,
        static_cast<float>(surface_config.width),
        static_cast<float>(surface_config.height)};
    surface_data->surface_param.surface_blending =
        SurfaceBlending::kBlendingCoverage;
    surface_data->surface_param.surface_flags.cursor = 0;
    surface_data->surface_param.frame_rate = 10;
    surface_data->surface_param.solid_fill_color = 0;
    surface_data->surface_param.surface_transform.rotation = 0.0f;
    surface_data->surface_param.surface_transform.flip_horizontal = 0;
    surface_data->surface_param.surface_transform.flip_vertical = 0;

    surface_data_.insert({surface_id, surface_data});
  }
    running_ = 1;
    lock.unlock();

  next2:

    sleep(8);

    lock.lock();
    running_ = 0;
    for (surface_data_map::iterator it = surface_data_.begin();
         it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != nullptr);

      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      delete surface_data;
      surface_data = nullptr;
    }
    surface_data_.clear();
    lock.unlock();
    if (!surface_data_.size()) {
      ret = DeInit(DisplayType::kPrimary);
      if (ret != 0) {
        TEST_ERROR("%s: Disconnect Failed!!", __func__);
      }
    }
  }
exit:
  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Test1YUV_1RGB_Rotated: This will test display of 1 YUV Video and 1 RGB Image
* which is rotated by 90 Degree.
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
TEST_F(DisplayGtest, Test1YUV_1RGB_Rotated) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  int32_t ret;
  uint32_t surface_id;
  SurfaceConfig surface_config;

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);
    std::unique_lock<std::mutex> lock(lock_);

    ret = Init(DisplayType::kPrimary);
    if (ret != 0) {
      TEST_ERROR("%s: Init Failed!!", __func__);
      goto exit;
    }

    {
      memset(&surface_config, 0x0, sizeof surface_config);

      surface_config.width = 1920;
      surface_config.height = 1080;
      surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
      surface_config.buffer_count = 4;
      surface_config.cache = 0;
      surface_config.use_buffer = 0;
      surface_config.z_order = 1;
      surface_config.surface_transform.rotation = 0.0f;
      surface_config.surface_transform.flip_horizontal = 0;
      surface_config.surface_transform.flip_vertical = 0;
      ret = display_->CreateSurface(surface_config, &surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: CreateSurface Failed!!", __func__);
        goto next;
      }

      SurfaceData* surface_data = new SurfaceData();
      assert(surface_data != nullptr);

      surface_data->surface_id = surface_id;
      ret = display_->DequeueSurfaceBuffer(surface_id,
                                           surface_data->surface_buffer);
      surface_data->file = fopen("/data/misc/qmmf/YUV/1080p_1.nv12.yuv", "r");
      if (!surface_data->file) {
        TEST_ERROR("%s: Unable to open file", __func__);
        ret = display_->DestroySurface(surface_data->surface_id);
        if (ret != 0) {
          TEST_ERROR("%s: DestroySurface Failed!!", __func__);
        }
        delete surface_data;
        surface_data = nullptr;
        goto next;
      }

      uint8_t* src = static_cast<uint8_t*>(
                         surface_data->surface_buffer.plane_info[0].buf) +
                     surface_data->surface_buffer.plane_info[0].offset;
      uint32_t offset_temp = 0;
      uint32_t read_len = 0;

      for (uint32_t i = 0;
           i < (surface_data->surface_buffer.plane_info[0].height); i++) {
        read_len = fread(src + offset_temp, sizeof(uint8_t),
                         surface_data->surface_buffer.plane_info[0].width,
                         surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      offset_temp += surface_data->surface_buffer.plane_info[0].stride * 8;
      for (uint32_t i = 0;
           i < (surface_data->surface_buffer.plane_info[0].height) / 2; i++) {
        read_len = fread(src + offset_temp, sizeof(uint8_t),
                         surface_data->surface_buffer.plane_info[0].width,
                         surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      surface_data->buffer_ready = 1;

      surface_data->surface_param.src_rect = {0.0, 0.0,
          static_cast<float>(surface_config.width),
          static_cast<float>(surface_config.height)};
      surface_data->surface_param.dst_rect = {0.0, 0.0,
          static_cast<float>(surface_config.width),
          static_cast<float>(surface_config.height)};
      surface_data->surface_param.surface_blending =
          SurfaceBlending::kBlendingCoverage;
      surface_data->surface_param.surface_flags.cursor = 0;
      surface_data->surface_param.frame_rate = 20;
      surface_data->surface_param.solid_fill_color = 0;
      surface_data->surface_param.surface_transform.rotation = 0.0f;
      surface_data->surface_param.surface_transform.flip_horizontal = 0;
      surface_data->surface_param.surface_transform.flip_vertical = 0;

      surface_data_.insert({surface_id, surface_data});
    }
  next:

  {
    memset(&surface_config, 0x0, sizeof surface_config);

    surface_config.width = 352;
    surface_config.height = 288;
    surface_config.format = SurfaceFormat::kFormatBGRA8888;
    surface_config.buffer_count = 4;
    surface_config.cache = 0;
    surface_config.use_buffer = 0;
    surface_config.z_order = 2;
    surface_config.surface_transform.rotation = 90.0f;
    surface_config.surface_transform.flip_horizontal = 0;
    surface_config.surface_transform.flip_vertical = 0;
    ret = display_->CreateSurface(surface_config, &surface_id);
    if (ret != 0) {
      TEST_ERROR("%s: CreateSurface Failed!!", __func__);
      goto next2;
    }

    SurfaceData* surface_data = new SurfaceData();
    assert(surface_data != nullptr);

    surface_data->surface_id = surface_id;
    ret = display_->DequeueSurfaceBuffer(surface_id,
                                         surface_data->surface_buffer);

    surface_data->file =
        fopen("/data/misc/qmmf/Images/fasimo_352x288_bgra_8888.rgb", "r");
    if (!surface_data->file) {
      TEST_ERROR("%s: Unable to open file", __func__);
      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      delete surface_data;
      surface_data = nullptr;
      ret = DeInit(DisplayType::kPrimary);
      goto next2;
    }

    int32_t offset = 0;
    for (uint32_t i = 0; i < surface_data->surface_buffer.plane_info[0].height;
         i++) {
      fread(static_cast<uint8_t*>(
                surface_data->surface_buffer.plane_info[0].buf) +
                surface_data->surface_buffer.plane_info[0].offset + offset,
            sizeof(uint8_t),
            surface_data->surface_buffer.plane_info[0].width * 4,
            surface_data->file);
      offset +=
          ALIGNED_WIDTH(surface_data->surface_buffer.plane_info[0].width) * 4;
    }
    fclose(surface_data->file);
    surface_data->buffer_ready = 1;

    surface_data->surface_param.src_rect = {0.0, 0.0,
        static_cast<float>(surface_config.width),
        static_cast<float>(surface_config.height)};
    surface_data->surface_param.dst_rect = {0.0, 0.0,
        static_cast<float>(surface_config.height),
        static_cast<float>(surface_config.width)};
    surface_data->surface_param.surface_blending =
        SurfaceBlending::kBlendingCoverage;
    surface_data->surface_param.surface_flags.cursor = 0;
    surface_data->surface_param.frame_rate = 10;
    surface_data->surface_param.solid_fill_color = 0;
    surface_data->surface_param.surface_transform.rotation = 90.0f;
    surface_data->surface_param.surface_transform.flip_horizontal = 0;
    surface_data->surface_param.surface_transform.flip_vertical = 0;

    surface_data_.insert({surface_id, surface_data});
  }
    running_ = 1;
    lock.unlock();

  next2:

    sleep(8);

    lock.lock();
    running_ = 0;
    for (surface_data_map::iterator it = surface_data_.begin();
         it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != nullptr);

      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      delete surface_data;
      surface_data = nullptr;
    }
    surface_data_.clear();
    lock.unlock();
    if (!surface_data_.size()) {
      ret = DeInit(DisplayType::kPrimary);
      if (ret != 0) {
        TEST_ERROR("%s: Disconnect Failed!!", __func__);
      }
    }
  }
exit:
  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Test1YUV_1RGB_Both_Rotated: This will test display of 1 YUV Video and 1 RGB Image
* both rotated by 90 Degree.
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
TEST_F(DisplayGtest, Test1YUV_1RGB_Both_Rotated) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  int32_t ret;
  uint32_t surface_id;
  SurfaceConfig surface_config;

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);
    std::unique_lock<std::mutex> lock(lock_);

    ret = Init(DisplayType::kPrimary);
    if (ret != 0) {
      TEST_ERROR("%s: Init Failed!!", __func__);
      goto exit;
    }

    {
      memset(&surface_config, 0x0, sizeof surface_config);

      surface_config.width = 1920;
      surface_config.height = 1080;
      surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
      surface_config.buffer_count = 4;
      surface_config.cache = 0;
      surface_config.use_buffer = 0;
      surface_config.z_order = 1;
      surface_config.surface_transform.rotation = 90.0f;
      surface_config.surface_transform.flip_horizontal = 0;
      surface_config.surface_transform.flip_vertical = 0;
      ret = display_->CreateSurface(surface_config, &surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: CreateSurface Failed!!", __func__);
        goto next;
      }

      SurfaceData* surface_data = new SurfaceData();
      assert(surface_data != nullptr);

      surface_data->surface_id = surface_id;
      ret = display_->DequeueSurfaceBuffer(surface_id,
                                           surface_data->surface_buffer);
      surface_data->file = fopen("/data/misc/qmmf/YUV/1080p_1.nv12.yuv", "r");
      if (!surface_data->file) {
        TEST_ERROR("%s: Unable to open file", __func__);
        ret = display_->DestroySurface(surface_data->surface_id);
        if (ret != 0) {
          TEST_ERROR("%s: DestroySurface Failed!!", __func__);
        }
        delete surface_data;
        surface_data = nullptr;
        goto next;
      }

      uint8_t* src = static_cast<uint8_t*>(
                         surface_data->surface_buffer.plane_info[0].buf) +
                     surface_data->surface_buffer.plane_info[0].offset;
      uint32_t offset_temp = 0;
      uint32_t read_len = 0;

      for (uint32_t i = 0;
           i < (surface_data->surface_buffer.plane_info[0].height); i++) {
        read_len = fread(src + offset_temp, sizeof(uint8_t),
                         surface_data->surface_buffer.plane_info[0].width,
                         surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      offset_temp += surface_data->surface_buffer.plane_info[0].stride * 8;
      for (uint32_t i = 0;
           i < (surface_data->surface_buffer.plane_info[0].height) / 2; i++) {
        read_len = fread(src + offset_temp, sizeof(uint8_t),
                         surface_data->surface_buffer.plane_info[0].width,
                         surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      surface_data->buffer_ready = 1;

      surface_data->surface_param.src_rect = {0.0, 0.0,
          static_cast<float>(surface_config.width),
          static_cast<float>(surface_config.height)};
      surface_data->surface_param.dst_rect = {0.0, 0.0,
          static_cast<float>(surface_config.width),
          static_cast<float>(surface_config.height)};
      surface_data->surface_param.surface_blending =
          SurfaceBlending::kBlendingCoverage;
      surface_data->surface_param.surface_flags.cursor = 0;
      surface_data->surface_param.frame_rate = 20;
      surface_data->surface_param.solid_fill_color = 0;
      surface_data->surface_param.surface_transform.rotation = 90.0f;
      surface_data->surface_param.surface_transform.flip_horizontal = 0;
      surface_data->surface_param.surface_transform.flip_vertical = 0;

      surface_data_.insert({surface_id, surface_data});
    }
  next:

  {
    memset(&surface_config, 0x0, sizeof surface_config);

    surface_config.width = 352;
    surface_config.height = 288;
    surface_config.format = SurfaceFormat::kFormatBGRA8888;
    surface_config.buffer_count = 4;
    surface_config.cache = 0;
    surface_config.use_buffer = 0;
    surface_config.z_order = 2;
    surface_config.surface_transform.rotation = 90.0f;
    surface_config.surface_transform.flip_horizontal = 0;
    surface_config.surface_transform.flip_vertical = 0;
    ret = display_->CreateSurface(surface_config, &surface_id);
    if (ret != 0) {
      TEST_ERROR("%s: CreateSurface Failed!!", __func__);
      goto next2;
    }

    SurfaceData* surface_data = new SurfaceData();
    assert(surface_data != nullptr);

    surface_data->surface_id = surface_id;
    ret = display_->DequeueSurfaceBuffer(surface_id,
                                         surface_data->surface_buffer);

    surface_data->file =
        fopen("/data/misc/qmmf/Images/fasimo_352x288_bgra_8888.rgb", "r");
    if (!surface_data->file) {
      TEST_ERROR("%s: Unable to open file", __func__);
      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      delete surface_data;
      surface_data = nullptr;
      ret = DeInit(DisplayType::kPrimary);
      goto next2;
    }

    int32_t offset = 0;
    for (uint32_t i = 0; i < surface_data->surface_buffer.plane_info[0].height;
         i++) {
      fread(static_cast<uint8_t*>(
                surface_data->surface_buffer.plane_info[0].buf) +
                surface_data->surface_buffer.plane_info[0].offset + offset,
            sizeof(uint8_t),
            surface_data->surface_buffer.plane_info[0].width * 4,
            surface_data->file);
      offset +=
          ALIGNED_WIDTH(surface_data->surface_buffer.plane_info[0].width) * 4;
    }
    fclose(surface_data->file);
    surface_data->buffer_ready = 1;

    surface_data->surface_param.src_rect = {0.0, 0.0,
        static_cast<float>(surface_config.width),
        static_cast<float>(surface_config.height)};
    surface_data->surface_param.dst_rect = {0.0, 0.0,
        static_cast<float>(surface_config.height),
        static_cast<float>(surface_config.width)};
    surface_data->surface_param.surface_blending =
        SurfaceBlending::kBlendingCoverage;
    surface_data->surface_param.surface_flags.cursor = 0;
    surface_data->surface_param.frame_rate = 10;
    surface_data->surface_param.solid_fill_color = 0;
    surface_data->surface_param.surface_transform.rotation = 90.0f;
    surface_data->surface_param.surface_transform.flip_horizontal = 0;
    surface_data->surface_param.surface_transform.flip_vertical = 0;

    surface_data_.insert({surface_id, surface_data});
  }
    running_ = 1;
    lock.unlock();

  next2:

    sleep(8);

    lock.lock();
    running_ = 0;
    for (surface_data_map::iterator it = surface_data_.begin();
         it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != nullptr);

      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      delete surface_data;
      surface_data = nullptr;
    }
    surface_data_.clear();
    lock.unlock();
    if (!surface_data_.size()) {
      ret = DeInit(DisplayType::kPrimary);
      if (ret != 0) {
        TEST_ERROR("%s: Disconnect Failed!!", __func__);
      }
    }
  }
exit:
  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
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
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  int32_t ret;
  uint32_t surface_id;
  SurfaceConfig surface_config;
  std::vector<BufInfo*> buffers;

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);
    std::unique_lock<std::mutex> lock(lock_);

    ret = Init(DisplayType::kPrimary);
    if (ret != 0) {
      TEST_ERROR("%s: Init Failed!!", __func__);
      goto exit;
    }
    {
      memset(&surface_config, 0x0, sizeof surface_config);

      surface_config.width = 1920;
      surface_config.height = 1080;
      surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
      surface_config.buffer_count = 4;
      surface_config.cache = 0;
      surface_config.use_buffer = 1;
      ret = display_->CreateSurface(surface_config, &surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: CreateSurface Failed!!", __func__);
        ret = DeInit(DisplayType::kPrimary);
        goto exit;
      }

      SurfaceData* surface_data = new SurfaceData();
      assert(surface_data != nullptr);

      surface_data->surface_id = surface_id;

      for (uint32_t i = 0; i < surface_config.buffer_count; i++) {
        BufInfo* new_buf_info = new BufInfo();
        new_buf_info->buffer_info.buffer_config.width = surface_config.width;
        new_buf_info->buffer_info.buffer_config.height = surface_config.height;
        new_buf_info->buffer_info.buffer_config.format =
            static_cast<LayerBufferFormat>(surface_config.format);
        new_buf_info->buffer_info.buffer_config.buffer_count = 1;
        new_buf_info->buffer_info.buffer_config.cache = surface_config.cache;
        new_buf_info->buffer_info.alloc_buffer_info.fd = -1;
        new_buf_info->buffer_info.alloc_buffer_info.stride = 0;
        new_buf_info->buffer_info.alloc_buffer_info.size = 0;
        new_buf_info->buf = nullptr;
        buffers.push_back(new_buf_info);
        ret = buffer_allocator_.AllocateBuffer(&new_buf_info->buffer_info);
        if (ret != kErrorNone) {
          TEST_ERROR("%s: AllocateBuffer Failed. Error = %d", __func__,
                     ret);
          ret = display_->DestroySurface(surface_data->surface_id);
          if (ret != 0) {
            TEST_ERROR("%s: DestroySurface Failed!!", __func__);
          }
          delete surface_data;
          surface_data = nullptr;
          for (std::vector<BufInfo*>::iterator iter = buffers.begin();
               iter != buffers.end(); ++iter) {
            buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
            delete *iter;
            *iter = nullptr;
          }
          buffers.clear();
          ret = DeInit(DisplayType::kPrimary);
          goto exit;
        }
      }
      buf_info_.insert({surface_id, buffers});

      surface_data->file = fopen("/data/misc/qmmf/YUV/1080p_1.nv12.yuv", "r");
      if (!surface_data->file) {
        TEST_ERROR("%s: Unable to open file", __func__);
        ret = display_->DestroySurface(surface_data->surface_id);
        if (ret != 0) {
          TEST_ERROR("%s: DestroySurface Failed!!", __func__);
        }
        delete surface_data;
        surface_data = nullptr;
        for (std::map<uint32_t, std::vector<BufInfo*>>::iterator it =
                 buf_info_.begin();
             it != buf_info_.end(); ++it) {
          if (it != buf_info_.end()) {
            for (std::vector<BufInfo*>::iterator iter = it->second.begin();
                 iter != it->second.end(); ++iter) {
              buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
              delete *iter;
              *iter = nullptr;
            }
          }
        }
        buf_info_.clear();
        ret = DeInit(DisplayType::kPrimary);
        goto exit;
      }
      auto bufinfo = buf_info_.find(surface_id);
      BufInfo* new_buf_info = bufinfo->second.at(0);
      BufferInfo* bufferinfo = &new_buf_info->buffer_info;
      surface_data->surface_buffer.plane_info[0].ion_fd =
          bufferinfo->alloc_buffer_info.fd;
      surface_data->surface_buffer.buf_id = bufferinfo->alloc_buffer_info.fd;
      surface_data->surface_buffer.format =
          static_cast<SurfaceFormat>(bufferinfo->buffer_config.format);
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
          mmap(nullptr, (size_t)surface_data->surface_buffer.plane_info[0].size,
               PROT_READ | PROT_WRITE, MAP_SHARED,
               surface_data->surface_buffer.plane_info[0].ion_fd, 0);
      assert(surface_data->surface_buffer.plane_info[0].buf != nullptr);

      uint8_t* src = static_cast<uint8_t*>(
                         surface_data->surface_buffer.plane_info[0].buf) +
                     surface_data->surface_buffer.plane_info[0].offset;
      uint32_t offset_temp = 0;
      uint32_t read_len = 0;

      for (uint32_t i = 0;
           i < (surface_data->surface_buffer.plane_info[0].height); i++) {
        read_len = fread(src + offset_temp, sizeof(uint8_t),
                         surface_data->surface_buffer.plane_info[0].width,
                         surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      offset_temp += surface_data->surface_buffer.plane_info[0].stride * 8;
      for (uint32_t i = 0;
           i < (surface_data->surface_buffer.plane_info[0].height) / 2; i++) {
        read_len = fread(src + offset_temp, sizeof(uint8_t),
                         surface_data->surface_buffer.plane_info[0].width,
                         surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      surface_data->buffer_ready = 1;
      surface_data->surface_buffer_index = 0;

      surface_data->surface_param.src_rect = {0.0, 0.0,
          static_cast<float>(surface_config.width),
          static_cast<float>(surface_config.height)};
      surface_data->surface_param.dst_rect = {0.0, 0.0,
          static_cast<float>(surface_config.width),
          static_cast<float>(surface_config.height)};
      surface_data->surface_param.surface_blending =
          SurfaceBlending::kBlendingCoverage;
      surface_data->surface_param.surface_flags.cursor = 0;
      surface_data->surface_param.frame_rate = 10;
      surface_data->surface_param.solid_fill_color = 0;
      surface_data->surface_param.surface_transform.rotation = 0.0f;
      surface_data->surface_param.surface_transform.flip_horizontal = 0;
      surface_data->surface_param.surface_transform.flip_vertical = 0;

      surface_data_.insert({surface_id, surface_data});
    }
    running_ = 1;
    lock.unlock();

    sleep(8);
    lock.lock();
    running_ = 0;
    for (surface_data_map::iterator it = surface_data_.begin();
         it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != nullptr);

      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      if (surface_data->file) {
        fclose(surface_data->file);
        surface_data->file = nullptr;
      }
      delete surface_data;
      surface_data = nullptr;
    }
    surface_data_.clear();
    for (std::map<uint32_t, std::vector<BufInfo*>>::iterator it =
             buf_info_.begin();
         it != buf_info_.end(); ++it) {
      if (it != buf_info_.end()) {
        for (std::vector<BufInfo*>::iterator iter = it->second.begin();
             iter != it->second.end(); ++iter) {
          if ((*iter)->buf) {
            ret = munmap((*iter)->buf,
                         (*iter)->buffer_info.alloc_buffer_info.size);
            if (ret != 0) {
              TEST_ERROR("%s: munmap Failed!!", __func__);
            }
          }
          buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
          delete *iter;
          *iter = nullptr;
        }
      }
      buffers.clear();
      buf_info_.clear();
    }
    lock.unlock();
    if (!surface_data_.size()) {
      ret = DeInit(DisplayType::kPrimary);
      if (ret != 0) {
        TEST_ERROR("%s: Disconnect Failed!!", __func__);
      }
    }
  }
exit:
  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Test1YUV_External_1RGB_InternalBuffer: This will test display of 1 YUV Video
* and 1 RGB where the YUV buffer is allocated by the test app and RBG buffer by
* Display Service.
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
TEST_F(DisplayGtest, Test1YUV_External_1RGB_InternalBuffers) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  int32_t ret;
  uint32_t surface_id;
  SurfaceConfig surface_config;
  std::vector<BufInfo*> buffers;

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);
    std::unique_lock<std::mutex> lock(lock_);

    ret = Init(DisplayType::kPrimary);
    if (ret != 0) {
      TEST_ERROR("%s: Init Failed!!", __func__);
      goto exit;
    }
    {
      memset(&surface_config, 0x0, sizeof surface_config);

      surface_config.width = 1920;
      surface_config.height = 1080;
      surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
      surface_config.buffer_count = 4;
      surface_config.cache = 0;
      surface_config.use_buffer = 1;
      surface_config.z_order = 1;
      ret = display_->CreateSurface(surface_config, &surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: CreateSurface Failed!!", __func__);
        goto next;
      }

      SurfaceData* surface_data = new SurfaceData();
      assert(surface_data != nullptr);

      surface_data->surface_id = surface_id;

      for (uint32_t i = 0; i < surface_config.buffer_count; i++) {
        BufInfo* new_buf_info = new BufInfo();
        new_buf_info->buffer_info.buffer_config.width = surface_config.width;
        new_buf_info->buffer_info.buffer_config.height = surface_config.height;
        new_buf_info->buffer_info.buffer_config.format =
            static_cast<LayerBufferFormat>(surface_config.format);
        new_buf_info->buffer_info.buffer_config.buffer_count = 1;
        new_buf_info->buffer_info.buffer_config.cache = surface_config.cache;
        new_buf_info->buffer_info.alloc_buffer_info.fd = -1;
        new_buf_info->buffer_info.alloc_buffer_info.stride = 0;
        new_buf_info->buffer_info.alloc_buffer_info.size = 0;
        new_buf_info->buf = nullptr;
        buffers.push_back(new_buf_info);
        ret = buffer_allocator_.AllocateBuffer(&new_buf_info->buffer_info);
        if (ret != kErrorNone) {
          TEST_ERROR("%s: AllocateBuffer Failed. Error = %d", __func__,
                     ret);
          ret = display_->DestroySurface(surface_data->surface_id);
          if (ret != 0) {
            TEST_ERROR("%s: DestroySurface Failed!!", __func__);
          }
          delete surface_data;
          surface_data = nullptr;
          for (std::vector<BufInfo*>::iterator iter = buffers.begin();
               iter != buffers.end(); ++iter) {
            buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
            delete *iter;
            *iter = nullptr;
          }
          buffers.clear();
          goto next;
        }
      }

      buf_info_.insert({surface_id, buffers});

      surface_data->file = fopen("/data/misc/qmmf/YUV/1080p_1.nv12.yuv", "r");
      if (!surface_data->file) {
        TEST_ERROR("%s: Unable to open file", __func__);
        ret = display_->DestroySurface(surface_data->surface_id);
        if (ret != 0) {
          TEST_ERROR("%s: DestroySurface Failed!!", __func__);
        }
        delete surface_data;
        surface_data = nullptr;
        for (std::map<uint32_t, std::vector<BufInfo*>>::iterator it =
                 buf_info_.begin();
             it != buf_info_.end(); ++it) {
          if (it != buf_info_.end()) {
            for (std::vector<BufInfo*>::iterator iter = it->second.begin();
                 iter != it->second.end(); ++iter) {
              buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
              delete *iter;
              *iter = nullptr;
            }
          }
        }
        buf_info_.clear();
        goto next;
      }

      auto bufinfo = buf_info_.find(surface_id);
      BufInfo* new_buf_info = bufinfo->second.at(0);
      BufferInfo* bufferinfo = &new_buf_info->buffer_info;
      surface_data->surface_buffer.plane_info[0].ion_fd =
          bufferinfo->alloc_buffer_info.fd;
      surface_data->surface_buffer.buf_id = bufferinfo->alloc_buffer_info.fd;
      surface_data->surface_buffer.format =
          static_cast<SurfaceFormat>(bufferinfo->buffer_config.format);
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
          mmap(nullptr, (size_t)surface_data->surface_buffer.plane_info[0].size,
               PROT_READ | PROT_WRITE, MAP_SHARED,
               surface_data->surface_buffer.plane_info[0].ion_fd, 0);
      assert(surface_data->surface_buffer.plane_info[0].buf != nullptr);

      uint8_t* src = static_cast<uint8_t*>(
                         surface_data->surface_buffer.plane_info[0].buf) +
                     surface_data->surface_buffer.plane_info[0].offset;
      uint32_t offset_temp = 0;
      uint32_t read_len = 0;

      for (uint32_t i = 0;
           i < (surface_data->surface_buffer.plane_info[0].height); i++) {
        read_len = fread(src + offset_temp, sizeof(uint8_t),
                         surface_data->surface_buffer.plane_info[0].width,
                         surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      offset_temp += surface_data->surface_buffer.plane_info[0].stride * 8;
      for (uint32_t i = 0;
           i < (surface_data->surface_buffer.plane_info[0].height) / 2; i++) {
        read_len = fread(src + offset_temp, sizeof(uint8_t),
                         surface_data->surface_buffer.plane_info[0].width,
                         surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      surface_data->buffer_ready = 1;
      surface_data->surface_buffer_index = 0;

      surface_data->surface_param.src_rect = {0.0, 0.0,
          static_cast<float>(surface_config.width),
          static_cast<float>(surface_config.height)};
      surface_data->surface_param.dst_rect = {0.0, 0.0,
          static_cast<float>(surface_config.width),
          static_cast<float>(surface_config.height)};
      surface_data->surface_param.surface_blending =
          SurfaceBlending::kBlendingCoverage;
      surface_data->surface_param.surface_flags.cursor = 0;
      surface_data->surface_param.frame_rate = 20;
      surface_data->surface_param.solid_fill_color = 0;
      surface_data->surface_param.surface_transform.rotation = 0.0f;
      surface_data->surface_param.surface_transform.flip_horizontal = 0;
      surface_data->surface_param.surface_transform.flip_vertical = 0;

      surface_data_.insert({surface_id, surface_data});
    }
  next :

  {
    memset(&surface_config, 0x0, sizeof surface_config);

    surface_config.width = 352;
    surface_config.height = 288;
    surface_config.format = SurfaceFormat::kFormatBGRA8888;
    surface_config.buffer_count = 4;
    surface_config.cache = 0;
    surface_config.use_buffer = 0;
    surface_config.z_order = 2;
    ret = display_->CreateSurface(surface_config, &surface_id);
    if (ret != 0) {
      TEST_ERROR("%s: CreateSurface Failed!!", __func__);
      goto next2;
    }

    SurfaceData* surface_data = new SurfaceData();
    assert(surface_data != nullptr);

    surface_data->surface_id = surface_id;
    ret = display_->DequeueSurfaceBuffer(surface_id,
                                         surface_data->surface_buffer);

    surface_data->file =
        fopen("/data/misc/qmmf/Images/fasimo_352x288_bgra_8888.rgb", "r");
    if (!surface_data->file) {
      TEST_ERROR("%s: Unable to open file", __func__);
      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      delete surface_data;
      surface_data = nullptr;
      goto next2;
    }

    int32_t offset = 0;
    for (uint32_t i = 0; i < surface_data->surface_buffer.plane_info[0].height;
         i++) {
      fread(static_cast<uint8_t*>(
                surface_data->surface_buffer.plane_info[0].buf) +
                surface_data->surface_buffer.plane_info[0].offset + offset,
            sizeof(uint8_t),
            surface_data->surface_buffer.plane_info[0].width * 4,
            surface_data->file);
      offset +=
          ALIGNED_WIDTH(surface_data->surface_buffer.plane_info[0].width) * 4;
    }
    fclose(surface_data->file);
    surface_data->buffer_ready = 1;

    surface_data->surface_param.src_rect = {0.0, 0.0,
        static_cast<float>(surface_config.width),
        static_cast<float>(surface_config.height)};
    surface_data->surface_param.dst_rect = {0.0, 0.0,
        static_cast<float>(surface_config.width),
        static_cast<float>(surface_config.height)};
    surface_data->surface_param.surface_blending =
        SurfaceBlending::kBlendingCoverage;
    surface_data->surface_param.surface_flags.cursor = 0;
    surface_data->surface_param.frame_rate = 20;
    surface_data->surface_param.solid_fill_color = 0;
    surface_data->surface_param.surface_transform.rotation = 0;
    surface_data->surface_param.surface_transform.flip_horizontal = 0;
    surface_data->surface_param.surface_transform.flip_vertical = 0;

    surface_data_.insert({surface_id, surface_data});
  }
    running_ = 1;
    lock.unlock();

  next2:

    sleep(8);

    lock.lock();
    running_ = 0;
    for (surface_data_map::iterator it = surface_data_.begin();
         it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != nullptr);

      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      if (surface_data->file) {
        fclose(surface_data->file);
        surface_data->file = nullptr;
      }
      delete surface_data;
      surface_data = nullptr;
    }
    surface_data_.clear();
    for (std::map<uint32_t, std::vector<BufInfo*>>::iterator it =
             buf_info_.begin();
         it != buf_info_.end(); ++it) {
      if (it != buf_info_.end()) {
        for (std::vector<BufInfo*>::iterator iter = it->second.begin();
             iter != it->second.end(); ++iter) {
          if ((*iter)->buf) {
            ret = munmap((*iter)->buf,
                         (*iter)->buffer_info.alloc_buffer_info.size);
            if (ret != 0) {
              TEST_ERROR("%s: munmap Failed!!", __func__);
            }
          }
          buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
          delete *iter;
          *iter = nullptr;
        }
      }
    }
    buffers.clear();
    buf_info_.clear();
    lock.unlock();
    if (!surface_data_.size()) {
      ret = DeInit(DisplayType::kPrimary);
      if (ret != 0) {
        TEST_ERROR("%s: Disconnect Failed!!", __func__);
      }
    }
  }
exit:
  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

/*
* Test1YUV_1RGB_Both_ExternalBuffers: This will test display of 1 YUV Video and
* 1 RGB where both the YUV buffer and RBG buffer allocated by the test app.
* Api test sequence:
*  - Init
*  - CreateSurface ---- for YUV
*  - AllocateBuffer ---- for YUV
*  - DequeueSurfaceBuffer
*  - CreateSurface ---- for RGB
*  - AllocateBuffer ---- for RGB
*  - DequeueSurfaceBuffer
*  - Go to Sleep and run the DisplayVSync thread
*  - DestroySurface ---- for YUV
*  - DestroySurface ---- for RGB
*  - Deallocate Buffer ---- for YUV
*  - Deallocate Buffer ---- for RGB
*  - DeInit
*/
TEST_F(DisplayGtest, Test1YUV_1RGB_Both_ExternalBuffers) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
          test_info_->test_case_name(), test_info_->name());

  int32_t ret;
  uint32_t surface_id;
  SurfaceConfig surface_config;
  SurfaceData* surface_data = nullptr;
  std::vector<BufInfo*> buffers;

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);
    std::unique_lock<std::mutex> lock(lock_);

    ret = Init(DisplayType::kPrimary);
    if (ret != 0) {
      TEST_ERROR("%s: Init Failed!!", __func__);
      goto exit;
    }
    {
      memset(&surface_config, 0x0, sizeof surface_config);

      surface_config.width = 1920;
      surface_config.height = 1080;
      surface_config.format = SurfaceFormat::kFormatYCbCr420SemiPlanarVenus;
      surface_config.buffer_count = 4;
      surface_config.cache = 0;
      surface_config.use_buffer = 1;
      surface_config.z_order = 1;
      ret = display_->CreateSurface(surface_config, &surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: CreateSurface Failed!!", __func__);
        goto next;
      }

      surface_data = new SurfaceData();
      assert(surface_data != nullptr);

      surface_data->surface_id = surface_id;

      for (uint32_t i = 0; i < surface_config.buffer_count; i++) {
        BufInfo* new_buf_info = new BufInfo();
        new_buf_info->buffer_info.buffer_config.width = surface_config.width;
        new_buf_info->buffer_info.buffer_config.height = surface_config.height;
        new_buf_info->buffer_info.buffer_config.format =
            static_cast<LayerBufferFormat>(surface_config.format);
        new_buf_info->buffer_info.buffer_config.buffer_count = 1;
        new_buf_info->buffer_info.buffer_config.cache = surface_config.cache;
        new_buf_info->buffer_info.alloc_buffer_info.fd = -1;
        new_buf_info->buffer_info.alloc_buffer_info.stride = 0;
        new_buf_info->buffer_info.alloc_buffer_info.size = 0;
        new_buf_info->buf = nullptr;
        buffers.push_back(new_buf_info);
        ret = buffer_allocator_.AllocateBuffer(&new_buf_info->buffer_info);
        if (ret != kErrorNone) {
          TEST_ERROR("%s: AllocateBuffer Failed. Error = %d", __func__,
                     ret);
          ret = display_->DestroySurface(surface_data->surface_id);
          if (ret != 0) {
            TEST_ERROR("%s: DestroySurface Failed!!", __func__);
          }
          delete surface_data;
          surface_data = nullptr;
          for (std::vector<BufInfo*>::iterator iter = buffers.begin();
               iter != buffers.end(); ++iter) {
            buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
            delete *iter;
            *iter = nullptr;
          }
          buffers.clear();
          goto next;
        }
      }

      buf_info_.insert({surface_id, buffers});

      surface_data->file = fopen("/data/misc/qmmf/YUV/1080p_1.nv12.yuv", "r");
      if (!surface_data->file) {
        TEST_ERROR("%s: Unable to open file", __func__);
        ret = display_->DestroySurface(surface_data->surface_id);
        if (ret != 0) {
          TEST_ERROR("%s: DestroySurface Failed!!", __func__);
        }
        delete surface_data;
        surface_data = nullptr;
        for (std::map<uint32_t, std::vector<BufInfo*>>::iterator it =
                 buf_info_.begin();
             it != buf_info_.end(); ++it) {
          if (it != buf_info_.end()) {
            for (std::vector<BufInfo*>::iterator iter = it->second.begin();
                 iter != it->second.end(); ++iter) {
              buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
              delete *iter;
              *iter = nullptr;
            }
          }
        }
        buf_info_.clear();
        goto next;
      }

      auto bufinfo = buf_info_.find(surface_id);
      BufInfo* new_buf_info = bufinfo->second.at(0);
      BufferInfo* bufferinfo = &new_buf_info->buffer_info;
      surface_data->surface_buffer.plane_info[0].ion_fd =
          bufferinfo->alloc_buffer_info.fd;
      surface_data->surface_buffer.buf_id = bufferinfo->alloc_buffer_info.fd;
      surface_data->surface_buffer.format =
          static_cast<SurfaceFormat>(bufferinfo->buffer_config.format);
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
          mmap(nullptr, (size_t)surface_data->surface_buffer.plane_info[0].size,
               PROT_READ | PROT_WRITE, MAP_SHARED,
               surface_data->surface_buffer.plane_info[0].ion_fd, 0);
      assert(surface_data->surface_buffer.plane_info[0].buf != nullptr);

      uint8_t* src = static_cast<uint8_t*>(
                         surface_data->surface_buffer.plane_info[0].buf) +
                     surface_data->surface_buffer.plane_info[0].offset;
      uint32_t offset_temp = 0;
      uint32_t read_len = 0;

      for (uint32_t i = 0;
           i < (surface_data->surface_buffer.plane_info[0].height); i++) {
        read_len = fread(src + offset_temp, sizeof(uint8_t),
                         surface_data->surface_buffer.plane_info[0].width,
                         surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      offset_temp += surface_data->surface_buffer.plane_info[0].stride * 8;
      for (uint32_t i = 0;
           i < (surface_data->surface_buffer.plane_info[0].height) / 2; i++) {
        read_len = fread(src + offset_temp, sizeof(uint8_t),
                         surface_data->surface_buffer.plane_info[0].width,
                         surface_data->file);
        assert(read_len == surface_data->surface_buffer.plane_info[0].width);
        offset_temp += surface_data->surface_buffer.plane_info[0].stride;
      }
      surface_data->buffer_ready = 1;
      surface_data->surface_buffer_index = 0;

      surface_data->surface_param.src_rect = {0.0, 0.0,
          static_cast<float>(surface_config.width),
          static_cast<float>(surface_config.height)};
      surface_data->surface_param.dst_rect = {0.0, 0.0,
          static_cast<float>(surface_config.width),
          static_cast<float>(surface_config.height)};
      surface_data->surface_param.surface_blending =
          SurfaceBlending::kBlendingCoverage;
      surface_data->surface_param.surface_flags.cursor = 0;
      surface_data->surface_param.frame_rate = 20;
      surface_data->surface_param.solid_fill_color = 0;
      surface_data->surface_param.surface_transform.rotation = 0.0f;
      surface_data->surface_param.surface_transform.flip_horizontal = 0;
      surface_data->surface_param.surface_transform.flip_vertical = 0;

      surface_data_.insert({surface_id, surface_data});
    }
  next :

    {
      memset(&surface_config, 0x0, sizeof surface_config);

      surface_config.width = 352;
      surface_config.height = 288;
      surface_config.format = SurfaceFormat::kFormatBGRA8888;
      surface_config.buffer_count = 4;
      surface_config.cache = 0;
      surface_config.use_buffer = 1;
      surface_config.z_order = 2;
      ret = display_->CreateSurface(surface_config, &surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: CreateSurface Failed!!", __func__);
        goto next2;
      }

      surface_data = new SurfaceData();
      assert(surface_data != nullptr);

      surface_data->surface_id = surface_id;
      std::vector<BufInfo*> buffers;

      for (uint32_t i = 0; i < surface_config.buffer_count; i++) {
        BufInfo* new_buf_info = new BufInfo();
        new_buf_info->buffer_info.buffer_config.width = surface_config.width;
        new_buf_info->buffer_info.buffer_config.height = surface_config.height;
        new_buf_info->buffer_info.buffer_config.format =
            static_cast<LayerBufferFormat>(surface_config.format);
        new_buf_info->buffer_info.buffer_config.buffer_count = 1;
        new_buf_info->buffer_info.buffer_config.cache = surface_config.cache;
        new_buf_info->buffer_info.alloc_buffer_info.fd = -1;
        new_buf_info->buffer_info.alloc_buffer_info.stride = 0;
        new_buf_info->buffer_info.alloc_buffer_info.size = 0;
        new_buf_info->buf = nullptr;
        buffers.push_back(new_buf_info);
        ret = buffer_allocator_.AllocateBuffer(&new_buf_info->buffer_info);
        if (ret != kErrorNone) {
          TEST_ERROR("%s: AllocateBuffer Failed. Error = %d", __func__,
              ret);
          ret = display_->DestroySurface(surface_data->surface_id);
          if (ret != 0) {
            TEST_ERROR("%s: DestroySurface Failed!!", __func__);
          }
          delete surface_data;
          surface_data = nullptr;
          for (std::vector<BufInfo*>::iterator iter = buffers.begin();
              iter != buffers.end(); ++iter) {
            buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
            delete *iter;
            *iter = nullptr;
          }
          buffers.clear();
          goto next2;
        }
      }

      buf_info_.insert({surface_id, buffers});

      surface_data->file =
          fopen("/data/misc/qmmf/Images/fasimo_352x288_bgra_8888.rgb", "r");
      if (!surface_data->file) {
        TEST_ERROR("%s: Unable to open file", __func__);
        ret = display_->DestroySurface(surface_data->surface_id);
        if (ret != 0) {
          TEST_ERROR("%s: DestroySurface Failed!!", __func__);
        }
        delete surface_data;
        surface_data = nullptr;
        for (std::map<uint32_t, std::vector<BufInfo*>>::iterator it =
            buf_info_.begin(); it != buf_info_.end(); ++it) {
          if (it != buf_info_.end()) {
            for (std::vector<BufInfo*>::iterator iter = it->second.begin();
               iter != it->second.end(); ++iter) {
              buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
              *iter = nullptr;
              delete *iter;
            }
          }
        }
        buf_info_.clear();
        goto next2;
      }

      auto bufinfo = buf_info_.find(surface_id);
      BufInfo* new_buf_info = bufinfo->second.at(0);
      BufferInfo* bufferinfo = &new_buf_info->buffer_info;
      surface_data->surface_buffer.plane_info[0].ion_fd =
          bufferinfo->alloc_buffer_info.fd;
      surface_data->surface_buffer.buf_id = bufferinfo->alloc_buffer_info.fd;
      surface_data->surface_buffer.format =
          static_cast<SurfaceFormat>(bufferinfo->buffer_config.format);
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
          mmap(nullptr, (size_t)surface_data->surface_buffer.plane_info[0].size,
               PROT_READ | PROT_WRITE, MAP_SHARED,
               surface_data->surface_buffer.plane_info[0].ion_fd, 0);
      assert(surface_data->surface_buffer.plane_info[0].buf != nullptr);

      int32_t offset = 0;
      for (uint32_t i = 0; i < surface_data->surface_buffer.plane_info[0].height;
           i++) {
        fread(static_cast<uint8_t*>(
                  surface_data->surface_buffer.plane_info[0].buf) +
                  surface_data->surface_buffer.plane_info[0].offset + offset,
              sizeof(uint8_t),
              surface_data->surface_buffer.plane_info[0].width * 4,
              surface_data->file);
        offset +=
            ALIGNED_WIDTH(surface_data->surface_buffer.plane_info[0].width) * 4;
      }
      fclose(surface_data->file);
      surface_data->buffer_ready = 1;
      surface_data->surface_buffer_index = 0;

      surface_data->surface_param.src_rect = {0.0, 0.0,
          static_cast<float>(surface_config.width),
          static_cast<float>(surface_config.height)};
      surface_data->surface_param.dst_rect = {0.0, 0.0,
          static_cast<float>(surface_config.width),
          static_cast<float>(surface_config.height)};
      surface_data->surface_param.surface_blending =
          SurfaceBlending::kBlendingCoverage;
      surface_data->surface_param.surface_flags.cursor = 0;
      surface_data->surface_param.frame_rate = 20;
      surface_data->surface_param.solid_fill_color = 0;
      surface_data->surface_param.surface_transform.rotation = 0;
      surface_data->surface_param.surface_transform.flip_horizontal = 0;
      surface_data->surface_param.surface_transform.flip_vertical = 0;

      surface_data_.insert({surface_id, surface_data});
    }
    running_ = 1;
    lock.unlock();
  next2:

    sleep(8);

    lock.lock();
    running_ = 0;
    for (surface_data_map::iterator it = surface_data_.begin();
         it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != nullptr);

      ret = display_->DestroySurface(surface_data->surface_id);
      if (ret != 0) {
        TEST_ERROR("%s: DestroySurface Failed!!", __func__);
      }
      if (surface_data->file) {
        fclose(surface_data->file);
        surface_data->file = nullptr;
      }
      delete surface_data;
      surface_data = nullptr;
    }
    surface_data_.clear();
    for (std::map<uint32_t, std::vector<BufInfo*>>::iterator it =
             buf_info_.begin();
         it != buf_info_.end(); ++it) {
      if (it != buf_info_.end()) {
        for (std::vector<BufInfo*>::iterator iter = it->second.begin();
             iter != it->second.end(); ++iter) {
          if ((*iter)->buf) {
            ret = munmap((*iter)->buf,
                         (*iter)->buffer_info.alloc_buffer_info.size);
            if (ret != 0) {
              TEST_ERROR("%s: munmap Failed!!", __func__);
            }
          }
          buffer_allocator_.FreeBuffer(&((*iter)->buffer_info));
          delete *iter;
          *iter = nullptr;
        }
      }
    }
    buffers.clear();
    buf_info_.clear();
    lock.unlock();
    if (!surface_data_.size()) {
      ret = DeInit(DisplayType::kPrimary);
      if (ret != 0) {
        TEST_ERROR("%s: Disconnect Failed!!", __func__);
      }
    }
  }
exit:
  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
          test_info_->test_case_name(), test_info_->name());
}

void DisplayGtest::DisplayCallbackHandler(DisplayEventType event_type,
                                          void* event_data,
                                          size_t event_data_size) {
  TEST_INFO("%s: Enter ", __func__);
  TEST_INFO("%s: Exit ", __func__);
}

void DisplayGtest::SessionCallbackHandler(DisplayEventType event_type,
                                          void* event_data,
                                          size_t event_data_size) {
  TEST_INFO("%s: Enter", __func__);
  TEST_INFO("%s: Exit", __func__);
}

void DisplayGtest::DisplayVSyncHandler(int64_t time_stamp) {
  TEST_INFO("%s: Enter", __func__);
  TEST_INFO("%s: Exit", __func__);
}

void DisplayGtest::DisplayThreadEntry(DisplayGtest* display_gtest) {
  TEST_INFO("%s:() Enter", __func__);
  display_gtest->DisplayThread();
  TEST_INFO("%s: Exit", __func__);
}

/*
* This thread will queue and dequeue Buffers for all the surfaces
* Api test sequence:
*  - QueueSurfaceBuffer
*  - DequeueSurfaceBuffer
*  - fread
*/
void DisplayGtest::DisplayThread() {
  bool run = true;
  int32_t ret;
  while (run) {
    std::unique_lock<std::mutex> lock(lock_);
    if (!running_) {
      lock.unlock();
      break;
    }

    for (surface_data_map::iterator it = surface_data_.begin();
         it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != nullptr);

      if (surface_data->buffer_ready) {
        TEST_INFO("%s: For Surface ID::%u QueueSurfaceBuffer FD %d", __func__,
            surface_data->surface_id, surface_data->surface_buffer.buf_id);
        ret = display_->QueueSurfaceBuffer(surface_data->surface_id,
                                           surface_data->surface_buffer,
                                           surface_data->surface_param);
        surface_data->buffer_ready = 0;
        if (ret != 0) {
          TEST_ERROR("%s: QueueSurfaceBuffer Failed!!", __func__);
        }
      }
    }

    for (surface_data_map::iterator it = surface_data_.begin();
         it != surface_data_.end(); ++it) {
      SurfaceData* surface_data = it->second;
      assert(surface_data != nullptr);

      if (!surface_data->buffer_ready) {
        ret = display_->DequeueSurfaceBuffer(surface_data->surface_id,
                                             surface_data->surface_buffer);
        if (ret != 0) {
          TEST_ERROR("%s: DequeueSurfaceBuffer Failed!!", __func__);
        }

        TEST_INFO("%s: For Surface ID::%u DequeueSurfaceBuffer FD %d", __func__,
            surface_data->surface_id, surface_data->surface_buffer.buf_id);

        if (surface_data->surface_buffer.buf_id == -1) {
          TEST_INFO("%s: No buf available !!", __func__);
          if (buf_info_.size()) {
            auto bufinfo = buf_info_.find(surface_data->surface_id);
            BufInfo* new_buf_info =
                bufinfo->second.at((++surface_data->surface_buffer_index) %
                                   (bufinfo->second.size()));
            TEST_DBG("%s: For surface_id::%u queue buf_index::%u !!", __func__,
                surface_data->surface_id, surface_data->surface_buffer_index);
            BufferInfo* bufferinfo = &new_buf_info->buffer_info;
            assert(bufferinfo != nullptr);
            surface_data->surface_buffer.plane_info[0].ion_fd =
                bufferinfo->alloc_buffer_info.fd;
            surface_data->surface_buffer.buf_id =
                bufferinfo->alloc_buffer_info.fd;
            surface_data->surface_buffer.format =
                static_cast<SurfaceFormat>(bufferinfo->buffer_config.format);
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
                mmap(nullptr,
                     (size_t)surface_data->surface_buffer.plane_info[0].size,
                     PROT_READ | PROT_WRITE, MAP_SHARED,
                     surface_data->surface_buffer.plane_info[0].ion_fd, 0);

            assert(surface_data->surface_buffer.plane_info[0].buf != nullptr);
            if (surface_data->surface_buffer_index == bufinfo->second.size()) {
              surface_data->surface_buffer_index = -1;
            }
          } else {
            continue;
          }
        }
        uint32_t read_len = 0;

        if (surface_data->surface_buffer.format ==
            static_cast<SurfaceFormat>(kFormatYCbCr420SemiPlanarVenus)) {
          if (surface_data->file) {
            uint8_t* src = static_cast<uint8_t*>(
                               surface_data->surface_buffer.plane_info[0].buf) +
                           surface_data->surface_buffer.plane_info[0].offset;
            uint32_t offset_temp = 0;
            for (uint32_t i = 0;
                 i < (surface_data->surface_buffer.plane_info[0].height); i++) {
              read_len = fread(src + offset_temp, sizeof(uint8_t),
                               surface_data->surface_buffer.plane_info[0].width,
                               surface_data->file);
              if (read_len !=
                  surface_data->surface_buffer.plane_info[0].width) {
                TEST_ERROR("%s: Failed to read length read_len=%u width=%u",
                           __func__, read_len,
                           surface_data->surface_buffer.plane_info[0].width);
              }
              assert((read_len ==
                      surface_data->surface_buffer.plane_info[0].width) ||
                     (read_len ==
                      surface_data->surface_buffer.plane_info[0].stride));
              offset_temp += surface_data->surface_buffer.plane_info[0].stride;
            }
            offset_temp +=
                (surface_data->surface_buffer.plane_info[0].stride) * 8;
            TEST_DBG("%s: offset_temp = %u stride=%u", __func__,
                     offset_temp,
                     surface_data->surface_buffer.plane_info[0].stride);
            for (uint32_t i = 0;
                 i < (surface_data->surface_buffer.plane_info[0].height) / 2;
                 i++) {
              read_len = fread(src + offset_temp, sizeof(uint8_t),
                               surface_data->surface_buffer.plane_info[0].width,
                               surface_data->file);
              if (read_len !=
                  surface_data->surface_buffer.plane_info[0].width) {
                TEST_ERROR("%s: Failed to read length read_len=%u width=%u",
                           __func__, read_len,
                           surface_data->surface_buffer.plane_info[0].width);
              }
              assert((read_len ==
                      surface_data->surface_buffer.plane_info[0].width) ||
                     (read_len ==
                      surface_data->surface_buffer.plane_info[0].stride));
              offset_temp += surface_data->surface_buffer.plane_info[0].stride;
            }
          }
        } else if ((surface_data->surface_buffer.format ==
                    static_cast<SurfaceFormat>(kFormatBGRA8888)) ||
                   (surface_data->surface_buffer.format ==
                    static_cast<SurfaceFormat>(kFormatRGBA8888)) ||
                   (surface_data->surface_buffer.format ==
                    static_cast<SurfaceFormat>(kFormatRGBA8888Ubwc))) {
          surface_data->file =
              fopen("/data/misc/qmmf/Images/fasimo_352x288_bgra_8888.rgb", "r");
          if (!surface_data->file) {
            TEST_ERROR("%s: Unable to open file", __func__);
          }
          int32_t offset = 0;
          for (uint32_t i = 0;
               i < surface_data->surface_buffer.plane_info[0].height; i++) {
            fread(static_cast<uint8_t*>(
                      surface_data->surface_buffer.plane_info[0].buf) +
                      surface_data->surface_buffer.plane_info[0].offset +
                      offset,
                  sizeof(uint8_t),
                  surface_data->surface_buffer.plane_info[0].width * 4,
                  surface_data->file);
            offset += ALIGNED_WIDTH(
                          surface_data->surface_buffer.plane_info[0].width) *
                      4;
          }
          fclose(surface_data->file);
          surface_data->file = nullptr;
        }
        surface_data->buffer_ready = 1;
      }
    }
    usleep(33333);
    lock.unlock();
  }
}
