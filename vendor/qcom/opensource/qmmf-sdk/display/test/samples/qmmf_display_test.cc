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

#define TAG "DisplayTest"

#include <utils/Log.h>
#include <utils/String8.h>
#include <assert.h>

#include "qmmf_display_test.h"


//#define DEBUG
#define TEST_INFO(fmt, args...)  ALOGD(fmt, ##args)
#define TEST_ERROR(fmt, args...) ALOGE(fmt, ##args)
#ifdef DEBUG
#define TEST_DBG  TEST_INFO
#else
#define TEST_DBG(...) ((void)0)
#endif

DisplayTest::DisplayTest() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  display_= new Display();
  assert(display_ != nullptr);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

DisplayTest::~DisplayTest() {
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  if (display_ != nullptr) {
    delete display_;
    display_ = nullptr;
  }
  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

int32_t DisplayTest::Connect() {

  TEST_INFO("%s:%s: Enter", TAG, __func__);

  auto ret = display_->Connect();
  TEST_INFO("%s:%s: Exit", TAG, __func__);

  return ret;
}

int32_t DisplayTest::Disconnect() {

  TEST_INFO("%s:%s: Enter", TAG, __func__);
  auto ret = display_->Disconnect();
  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

int32_t DisplayTest::CreateDisplay(DisplayType display_type) {

  TEST_INFO("%s:%s: Enter", TAG, __func__);

  DisplayCb display_status_cb;
  display_status_cb.EventCb = [&] ( DisplayEventType event_type,
      void *event_data, size_t event_data_size) { DisplayCallbackHandler
      (event_type, event_data, event_data_size); };

  display_status_cb.VSyncCb = [&] ( int64_t time_stamp)
      { DisplayVSyncHandler(time_stamp); };

  auto ret = display_->CreateDisplay(display_type, display_status_cb);
  TEST_INFO("%s:%s: Exit", TAG, __func__);

  return ret;
}

int32_t DisplayTest::DestroyDisplay(DisplayType display_type) {

  TEST_INFO("%s:%s: Enter", TAG, __func__);
  auto ret = display_->DestroyDisplay(display_type);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

int32_t DisplayTest::CreateSurface() {

  TEST_INFO("%s:%s: Enter", TAG, __func__);

  memset(&surface_config, 0x0, sizeof surface_config);

  surface_config.width=352;
  surface_config.height=288;
  surface_config.format = SurfaceFormat::kFormatBGRA8888;
  surface_config.buffer_count=4;
  surface_config.cache=0;
  surface_config.use_buffer=0;
  auto ret = display_->CreateSurface(surface_config, &surface_id);
  if(ret != 0) {
      TEST_ERROR("%s:%s CreateSurface Failed!!", TAG, __func__);
  }
  TEST_INFO("%s:%s: surface_id: %u", TAG, __func__, surface_id);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return 0;
}

int32_t DisplayTest::DestroySurface() {

  TEST_INFO("%s:%s: Enter", TAG, __func__);

  auto ret = display_->DestroySurface(surface_id);
  if(ret != 0) {
    TEST_ERROR("%s:%s DestroySurface Failed!!", TAG, __func__);
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return 0;
}

int32_t DisplayTest::DequeueSurfaceBuffer() {

  TEST_INFO("%s:%s: Enter", TAG, __func__);
  memset(&surface_buffer, 0x0, sizeof surface_buffer);

  surface_buffer.format = SurfaceFormat::kFormatBGRA8888;
  surface_buffer.acquire_fence =0;
  surface_buffer.release_fence =0;

  auto ret = display_->DequeueSurfaceBuffer(surface_id, surface_buffer);
  if(ret != 0) {
    TEST_ERROR("%s:%s DequeueSurfaceBuffer Failed!!", TAG, __func__);
  }
  file = fopen("/data/misc/qmmf/Images/fasimo_352x288_bgra_8888.rgb", "r");
  if (!file) {
    TEST_ERROR("%s:%s: Unable to open file", TAG, __func__);
  }
  int32_t offset=0;
  for(uint32_t i=0;i<surface_buffer.plane_info[0].height;i++) {
  fread((uint8_t*)surface_buffer.plane_info[0].buf +
      surface_buffer.plane_info[0].offset + offset, sizeof(uint8_t),
      surface_buffer.plane_info[0].width*4, file);
  offset +=((surface_buffer.plane_info[0].width+(
      (surface_buffer.plane_info[0].width%64)?(64-
      (surface_buffer.plane_info[0].width%64)):0))*4);
  }
  fclose (file);

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return 0;
}

int32_t DisplayTest::QueueSurfaceBuffer() {

  TEST_INFO("%s:%s: Enter", TAG, __func__);

  memset(&surface_param, 0x0, sizeof surface_param);

  surface_param.src_rect = { 0.0, 0.0, 352.0, 288.0 };
  surface_param.dst_rect = { 0.0, 0.0, 352.0, 288.0 };
  surface_param.surface_blending = SurfaceBlending::kBlendingCoverage;
  surface_param.surface_flags.cursor=0;
  surface_param.frame_rate = 30;
  surface_param.z_order = 0;
  surface_param.solid_fill_color=0;

  auto ret = display_->QueueSurfaceBuffer(surface_id, surface_buffer,
      surface_param);
  if(ret != 0) {
    TEST_ERROR("%s:%s QueueSurfaceBuffer Failed!!", TAG, __func__);
  }

  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return 0;
}

int32_t DisplayTest::GetDisplayParam() {

  TEST_INFO("%s:%s: Enter", TAG, __func__);

  DisplayParamType param_type;
  void *param;
  size_t param_size = 0;

  param=operator new(param_size);
  param_type = DisplayParamType::kContrast;
  auto ret = display_->GetDisplayParam(param_type, param, param_size);
  if(ret != 0) {
    TEST_ERROR("%s:%s GetDisplayParam Failed!!", TAG, __func__);
  }
  operator delete(param);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return 0;
}

int32_t DisplayTest::SetDisplayParam() {

  TEST_INFO("%s:%s: Enter", TAG, __func__);
  DisplayParamType param_type;
  void *param;
  size_t param_size = 0;
  param=operator new(param_size);
  param_type = DisplayParamType::kSaturation;
  auto ret = display_->SetDisplayParam(param_type, param, param_size);
  if(ret != 0) {
    TEST_ERROR("%s:%s SetDisplayParam Failed!!", TAG, __func__);
  }
  operator delete(param);
  TEST_INFO("%s:%s: Exit", TAG, __func__);

  return 0;
}

int32_t DisplayTest::DequeueWBSurfaceBuffer() {

  TEST_INFO("%s:%s: Enter", TAG, __func__);
  //TBD
  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return 0;
}

int32_t DisplayTest::QueueWBSurfaceBuffer() {

  TEST_INFO("%s:%s: Enter", TAG, __func__);
  //TBD
  TEST_INFO("%s:%s: Exit", TAG, __func__);
  return 0;
}

void DisplayTest::DisplayCallbackHandler(DisplayEventType event_type,
    void *event_data, size_t event_data_size)
{
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

void DisplayTest::SessionCallbackHandler(DisplayEventType event_type,
    void *event_data, size_t event_data_size)
{
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

void DisplayTest::DisplayVSyncHandler(int64_t time_stamp)
{
  TEST_INFO("%s:%s: Enter", TAG, __func__);
  TEST_INFO("%s:%s: Exit", TAG, __func__);
}

void CmdMenu::PrintMenu()
{
  printf("\n\n=========== QMMF SDK MENU ===================\n\n");

  printf(" \n\nIPCam Display Test Application commands \n");
  printf(" -----------------------------\n");
  printf("   %c. Connect\n", CmdMenu::CONNECT_CMD);
  printf("   %c. Disconnect\n", CmdMenu::DISCONNECT_CMD);
  printf("   %c. Create Display\n", CmdMenu::CREATE_DISPLAY_CMD);
  printf("   %c. Destroy Display\n", CmdMenu::DESTROY_DISPLAY_CMD);
  printf("   %c. Create Surface\n", CmdMenu::CREATE_SURFACE_CMD);
  printf("   %c. Destroy Surface\n", CmdMenu::DESTROY_SURFACE_CMD);
  printf("   %c. Dequeue Surface Buffer\n",
      CmdMenu::DEQUEUE_SURFACE_BUFFER_CMD);
  printf("   %c. Queue Surface Buffer\n",
      CmdMenu::QUEUE_SURFACE_BUFFER_CMD);
  printf("   %c. Get Display Params\n", CmdMenu::GET_DISPLAY_PARAM_CMD);
  printf("   %c. Set Display Params\n", CmdMenu::SET_DISPLAY_PARAM_CMD);
  printf("   %c. Dequeue WriteBack Surface Buffer\n",
      CmdMenu::DEQUEUE_WBSURFACE_BUFFER_CMD);
  printf("   %c. Queue WriteBack Surface Buffer\n",
      CmdMenu::QUEUE_WBSURFACE_BUFFER_CMD);
  printf("   %c. Exit\n", CmdMenu::EXIT_CMD);
  printf("\n   Choice: ");
}

CmdMenu::Command CmdMenu::GetCommand()
{
    PrintMenu();
    return CmdMenu::Command(
        static_cast<CmdMenu::CommandType>(getchar()));
}

int main(int argc,char *argv[])
{
  TEST_INFO("%s:%s: Enter", TAG, __func__);

  DisplayTest test_context;
  CmdMenu cmd_menu(test_context);
  int32_t testRunning = true, ret = NO_ERROR;

  while (testRunning) {
    CmdMenu::Command command = cmd_menu.GetCommand();

    switch (command.cmd) {
      case CmdMenu::CONNECT_CMD:
      {
        ret=test_context.Connect();
      }
      break;

      case CmdMenu::DISCONNECT_CMD:
      {
        ret=test_context.Disconnect();
      }
      break;
      case CmdMenu::CREATE_DISPLAY_CMD:
      {
        ret=test_context.CreateDisplay(DisplayType::kPrimary);
      }
      break;

      case CmdMenu::DESTROY_DISPLAY_CMD:
      {
        ret=test_context.DestroyDisplay(DisplayType::kPrimary);
      }
      break;
      case CmdMenu::CREATE_SURFACE_CMD:
      {
        ret=test_context.CreateSurface();
      }
      break;

      case CmdMenu::DESTROY_SURFACE_CMD:
      {
        ret=test_context.DestroySurface();
      }
      break;
      case CmdMenu::DEQUEUE_SURFACE_BUFFER_CMD:
      {
        ret=test_context.DequeueSurfaceBuffer();
      }
      break;
      case CmdMenu::QUEUE_SURFACE_BUFFER_CMD:
      {
        ret=test_context.QueueSurfaceBuffer();
      }
      break;
      case CmdMenu::GET_DISPLAY_PARAM_CMD:
      {
        ret=test_context.GetDisplayParam();
      }
      break;
      case CmdMenu::SET_DISPLAY_PARAM_CMD:
      {
        ret=test_context.SetDisplayParam();
      }
      break;
      case CmdMenu::DEQUEUE_WBSURFACE_BUFFER_CMD:
      {
        ret=test_context.DequeueWBSurfaceBuffer();
      }
      break;
      case CmdMenu::QUEUE_WBSURFACE_BUFFER_CMD:
      {
        ret=test_context.QueueWBSurfaceBuffer();
      }
      break;
      case CmdMenu::EXIT_CMD:
      {
        testRunning = false;
      }
      break;
      default:
      break;
    }
  }
  return ret;
}
