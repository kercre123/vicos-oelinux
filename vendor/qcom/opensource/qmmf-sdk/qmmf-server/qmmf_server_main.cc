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

#include <stdlib.h>
#include <utils/Log.h>

#include <binder/IInterface.h>
#include <binder/IBinder.h>
#include <binder/ProcessState.h>
#include <binder/IServiceManager.h>
#include <binder/IPCThreadState.h>

#include "common/audio/src/service/qmmf_audio_service.h"
#include "recorder/src/service/qmmf_recorder_service.h"
#include "display/src/service/qmmf_display_service.h"
#include "player/src/service/qmmf_player_service.h"
#include "system/src/service/qmmf_system_service.h"

using namespace android;
using namespace qmmf;
using namespace qmmf::common::audio;
using namespace recorder;
using namespace display;
using namespace player;
using namespace system;

#define INFO(...) \
  do { \
    printf(__VA_ARGS__); \
    printf("\n"); \
    ALOGD(__VA_ARGS__); \
} while(0)

int32_t main(int32_t argc, char **argv) {

  //Add System service.
  defaultServiceManager()->addService(String16(QMMF_SYSTEM_SERVICE_NAME),
                  new qmmf::system::SystemService(), false);
  INFO("Service(%s) Added successfully!", QMMF_SYSTEM_SERVICE_NAME);

  // Add audio service.
  defaultServiceManager()->addService(String16(QMMF_AUDIO_SERVICE_NAME),
          new qmmf::common::audio::AudioService(), false);
  INFO("Service(%s) Added successfully!", QMMF_AUDIO_SERVICE_NAME);

  //Add Recorder service.
  defaultServiceManager()->addService(String16(QMMF_RECORDER_SERVICE_NAME),
                  new qmmf::recorder::RecorderService(), false);
  INFO("Service(%s) Added successfully!", QMMF_RECORDER_SERVICE_NAME);

  //Add Player service.
  defaultServiceManager()->addService(String16(QMMF_PLAYER_SERVICE_NAME),
                  new qmmf::player::PlayerService(), false);
  INFO("Service(%s) Added successfully!", QMMF_PLAYER_SERVICE_NAME);

  //Add Display service.
  defaultServiceManager()->addService(String16(QMMF_DISPLAY_SERVICE_NAME),
                  new qmmf::display::DisplayService(), false);
  INFO("Service(%s) Added successfully!", QMMF_DISPLAY_SERVICE_NAME);

  android::ProcessState::self()->startThreadPool();
  IPCThreadState::self()->joinThreadPool();
  return 0;
}
