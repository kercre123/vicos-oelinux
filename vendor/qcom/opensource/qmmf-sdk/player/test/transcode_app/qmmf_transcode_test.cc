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

#include "qmmf_transcode_test.h"

#define TAG "TranscodeTest"

using ::qmmf::transcode::TranscoderTrack;
using ::qmmf::transcode::status_t;

TranscodeTest* TranscodeTest::instance_ = nullptr;

TranscodeTest* TranscodeTest::Connect() {
  if (instance_ == nullptr) {
    instance_ = new TranscodeTest;
    if (instance_) {
      QMMF_INFO("%s:%s Created Transcode Instance(%p) successfully", TAG,
                __func__, instance_);
      return instance_;
    } else {
      QMMF_ERROR("%s:%s Failed to Crate Transcode(%p)", TAG, __func__,
                 instance_);
      assert(0);
    }
  } else {
    QMMF_WARN("%s:%s Transcode is Already Created(%p)", TAG, __func__,
              instance_);
    return instance_;
  }
}

status_t TranscodeTest::Disconnect() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  if (instance_ != nullptr) {
    delete instance_;
    instance_ = nullptr;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return 0;
}

status_t TranscodeTest::CreateTrack() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;

  int32_t track_id;
  // TODO: Read from the user input the track file name and track id
  // and then pass it to TranscoderTrack contructor instead of nullptr
  track_id = 1;

  TranscoderTrack* track = new TranscoderTrack;

  if (track == nullptr) {
    QMMF_ERROR("%s:%s TranscoderTrack Creation Failed track_id(%u)", TAG,
               __func__, track_id);
    assert(0);
  }

  track_map_.insert(std::make_pair(track_id, track));

  ret = track->PreparePipeline();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to Prepare Pipeline for track_id(%u)", TAG,
               __func__, track_id);
    track->ReleaseResources();
    track_map_.erase(track_id);
    delete track;
    track = nullptr;
    assert(0);
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscodeTest::StartTrack() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;
  int32_t track_id;
  // TODO: Read the track id info from user input
  track_id = 1;
  TranscoderTrack* track = track_map_[track_id];
  ret = track->Start();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to Start Track track_id(%u)", TAG, __func__,
               track_id);
    assert(0);
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscodeTest::StopTrack() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;
  int32_t track_id;
  // TODO: Read the track id info from user input
  track_id = 1;
  TranscoderTrack* track = track_map_[track_id];
  ret = track->Stop();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to Stop Track track_id(%u)", TAG, __func__,
               track_id);
    assert(0);
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscodeTest::DeleteTrack() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;
  int32_t track_id;
  // TODO: Read the track id info from user input
  track_id = 1;
  TranscoderTrack* track = track_map_[track_id];
  ret = track->Delete();
  if (ret != 0) {
    QMMF_ERROR("%s:%s Failed to Delete Track track_id(%u)", TAG, __func__,
               track_id);
    track->ReleaseResources();
    track_map_.erase(track_id);
    delete track;
    track = nullptr;
    assert(0);
  }

  track_map_.erase(track_id);
  delete track;
  track = nullptr;

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t TranscodeTest::SetTrackParams() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return 0;
}

void CmdMenu::PrintMenu() {
  printf("\n\n=========== QIPCAM TEST MENU ===================\n\n");

  printf(" \n\nTranscode Test Application commands \n");
  printf(" -----------------------------\n");
  printf("   %c. Connect\n", CmdMenu::CONNECT_CMD);
  printf("   %c. Disconnect\n", CmdMenu::DISCONNECT_CMD);
  printf("   %c. Create Track\n", CmdMenu::CREATE_TRACK_CMD);
  printf("   %c. Delete Track\n", CmdMenu::DELETE_TRACK_CMD);
  printf("   %c. Start Track\n", CmdMenu::START_TRACK_CMD);
  printf("   %c. Stop Track\n", CmdMenu::STOP_TRACK_CMD);
  printf("   %c. Exit\n", CmdMenu::EXIT_CMD);
  printf("\n   Choice: ");
}

CmdMenu::Command CmdMenu::GetCommand() {
  PrintMenu();
  return CmdMenu::Command(static_cast<CmdMenu::CommandType>(getchar()));
}

int main() {
  QMMF_INFO("%s:%s Enter", TAG, __func__);

  TranscodeTest* test_context_ptr = nullptr;
  CmdMenu cmd_menu;

  bool testRunning = true;

  while (testRunning) {
    CmdMenu::Command command = cmd_menu.GetCommand();

    switch (command.cmd) {
      case CmdMenu::CONNECT_CMD:
        test_context_ptr = TranscodeTest::Connect();
        break;
      case CmdMenu::CREATE_TRACK_CMD:
        test_context_ptr->CreateTrack();
        break;
      case CmdMenu::START_TRACK_CMD:
        test_context_ptr->StartTrack();
        break;
      case CmdMenu::STOP_TRACK_CMD:
        test_context_ptr->StopTrack();
        break;
      case CmdMenu::DELETE_TRACK_CMD:
        test_context_ptr->DeleteTrack();
        break;
      case CmdMenu::DISCONNECT_CMD:
        TranscodeTest::Disconnect();
        break;
      case CmdMenu::EXIT_CMD:
        QMMF_INFO("%s:%s exit from test", TAG, __func__);
        testRunning = false;
        break;
      default:
        break;
    }
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return 0;
}
