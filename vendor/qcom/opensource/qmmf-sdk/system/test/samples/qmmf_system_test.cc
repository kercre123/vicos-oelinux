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

#define TAG "SystemTest"

#include "system/test/samples/qmmf_system_test.h"

#include <cassert>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "common/qmmf_log.h"
#include "qmmf-sdk/qmmf_codec.h"
#include "qmmf-sdk/qmmf_device.h"
#include "qmmf-sdk/qmmf_system.h"
#include "qmmf-sdk/qmmf_system_params.h"
#include "system/test/samples/qmmf_system_test_wav.h"

namespace qmmf_test {
namespace system {

using ::qmmf::AudioDeviceId;
using ::qmmf::DeviceCaps;
using ::qmmf::DeviceId;
using ::qmmf::DeviceInfo;
using ::qmmf::system::DeviceCb;
using ::qmmf::system::SoundModel;
using ::qmmf::system::status_t;
using ::qmmf::system::System;
using ::qmmf::system::SystemCb;
using ::qmmf::system::Tone;
using ::qmmf::system::ToneCb;
using ::qmmf::system::TriggerCb;
using ::std::cin;
using ::std::condition_variable;
using ::std::cout;
using ::std::endl;
using ::std::mutex;
using ::std::queue;
using ::std::thread;
using ::std::unique_lock;
using ::std::vector;

static const char* kDefaultFilePrefix = "/data/misc/qmmf/system_test";

SystemTest::SystemTest()
    : filename_prefix_(kDefaultFilePrefix),
      thread_(nullptr) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_INFO("%s: %s() test instantiated", TAG, __func__);
}

SystemTest::~SystemTest() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_INFO("%s: %s() test destroyed", TAG, __func__);
}

void SystemTest::Connect() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  SystemCb error_handler =
    [this] (const int32_t error) -> void {
      ErrorHandler(error);
    };

  status_t result = system_.Connect(error_handler);
  assert(result == 0);
}

void SystemTest::Disconnect() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  status_t result = system_.Disconnect();
  assert(result == 0);
}

void SystemTest::EnableDeviceEvents() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  DeviceCb device_handler =
    [this] (const DeviceInfo& device) -> void {
      DeviceHandler(device);
    };

  status_t result = system_.RegisterForDeviceEvents(device_handler);
  assert(result == 0);
}

void SystemTest::QueryDevices() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  vector<DeviceInfo> devices;
  status_t result = system_.QueryDeviceInfo(&devices);
  assert(result == 0);

  for (const DeviceInfo& device : devices) {
    DeviceCaps caps;
    result = system_.QueryDeviceCapabilities(device.id, &caps);
    assert(result == 0);

    cout << endl;
    cout << "device info[" << device.ToString().c_str() << "]" << endl;
    cout << "device caps[" << caps.ToString().c_str() << "]" << endl;
  }
}

void SystemTest::EnableSoundTrigger() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  status_t result;

  FILE *fp = fopen("/data/misc/qmmf/system_trigger.uim", "rb");
  assert(fp != nullptr);

  fseek(fp, 0, SEEK_END);
  long size = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  QMMF_DEBUG("%s: %s() size[%ld]", TAG, __func__, size);

  SoundModel sound_model;
  sound_model.device = static_cast<DeviceId>(AudioDeviceId::kBuiltIn);
  sound_model.keywords = 1;
  sound_model.size = size;
  sound_model.data = malloc(sound_model.size);
  assert(sound_model.data != nullptr);
  QMMF_DEBUG("%s: %s() sound_model[%s]", TAG, __func__,
             sound_model.ToString().c_str());

  size_t bytes_read = fread(sound_model.data, 1, sound_model.size, fp);
  QMMF_DEBUG("%s: %s() bytes_read[%zd]", TAG, __func__, bytes_read);
  assert(bytes_read == sound_model.size);

  result = system_.LoadSoundModel(sound_model);
  assert(result == 0);

  free(sound_model.data);
  fclose(fp);

  TriggerCb trigger_handler =
    [this] (const int32_t error) -> void {
      TriggerHandler(error);
    };

  result = system_.EnableSoundTrigger(trigger_handler);
  assert(result == 0);
}

void SystemTest::DisableSoundTrigger() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  status_t result;

  result = system_.DisableSoundTrigger();
  assert(result == 0);

  result = system_.UnloadSoundModel();
  assert(result == 0);
}

void SystemTest::PlayTone(const bool multi_tone) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  assert(thread_ == nullptr);

  multi_tone_ = multi_tone;

  while (!messages_.empty())
    messages_.pop();

  thread_ = new thread(SystemTest::StaticThreadEntry, this);
  assert(thread_ != nullptr);
}

void SystemTest::ErrorHandler(const int32_t error) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: error[%d]", TAG, __func__, error);

  assert(false);
}

void SystemTest::DeviceHandler(const DeviceInfo& device) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: device[%s]", TAG, __func__,
               device.ToString().c_str());

  cout << endl;
  cout << "Received device event[" << device.ToString().c_str() << "]" << endl;
}

void SystemTest::TriggerHandler(const int32_t error) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: error[%d]", TAG, __func__, error);

  if (error == 0) {
    cout << endl;
    cout << "Received SoundTrigger event" << endl;
  }
}

void SystemTest::ToneHandler(const int32_t error) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  QMMF_VERBOSE("%s: %s() INPARAM: error[%d]", TAG, __func__, error);

  SystemMessage message;
  message.type = SystemMessageType::kMessageToneFinished;
  message.result = error;

  message_lock_.lock();
  messages_.push(message);
  message_lock_.unlock();
  signal_.notify_one();

  if (thread_ != nullptr) {
    thread_->join();
    delete thread_;
    thread_ = nullptr;
  }

  while (!messages_.empty())
    messages_.pop();
}

void SystemTest::StaticThreadEntry(SystemTest* test) {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);

  test->ToneThread();
}

void SystemTest::ToneThread() {
  QMMF_DEBUG("%s: %s() TRACE", TAG, __func__);
  bool keep_running = true;
  int32_t iresult;
  status_t result;

  // read in audio data from file
  size_t buffer_size;
  iresult = wav_.Configure(filename_prefix_, &buffer_size);
  assert(iresult == 0);

  iresult = wav_.Open();
  assert(iresult == 0);

  void* buffer = malloc(buffer_size);
  assert(buffer != NULL);

  iresult = wav_.Read(buffer);
  assert(iresult == SystemTestWav::kEOF);

  wav_.Close();

  // send audio data
  ToneCb tone_handler =
    [this] (const int32_t error) -> void {
      ToneHandler(error);
    };

  Tone tone;
  if (multi_tone_) {
    tone.delay = 1000;
    tone.loop_num = 4;
  } else {
    tone.delay = 0;
    tone.loop_num = 1;
  }
  tone.size = buffer_size;
  tone.buffer = buffer;

  result = system_.PlayTone({ static_cast<DeviceId>(AudioDeviceId::kBuiltIn) },
                            tone, tone_handler);
  assert(result == 0);

  while (keep_running) {
    // wait until there is something to do
    if (messages_.empty()) {
      unique_lock<mutex> lk(message_lock_);
      signal_.wait(lk);
    }

    // process the next pending message
    message_lock_.lock();
    if (!messages_.empty()) {
      SystemMessage message = messages_.front();

      switch (message.type) {
          break;

        case SystemMessageType::kMessageToneFinished:
          QMMF_DEBUG("%s: %s-MessageBuffer() TRACE", TAG, __func__);
          QMMF_VERBOSE("%s: %s() INPARAM: result[%d]", TAG, __func__,
                       message.result);
          keep_running = false;
          break;
      }
      messages_.pop();
    }
    message_lock_.unlock();
  }

  free(buffer);
}

void CommandMenu::PrintMenu() {
    cout << endl << endl;
    cout << "====== QMMF-SDK SYSTEM TEST MENU ======" << endl;

    cout << static_cast<char>(Command::kConnect) << ". Connect" << endl;
    cout << static_cast<char>(Command::kDisconnect) << ". Disconnect" << endl;
    cout << static_cast<char>(Command::kEnableDeviceEvents)
         << ". Enable Device Events" << endl;
    cout << static_cast<char>(Command::kQueryDevices) << ". Query Devices"
         << endl;
    cout << static_cast<char>(Command::kEnableSoundTrigger)
         << ". Enable SoundTrigger" << endl;
    cout << static_cast<char>(Command::kDisableSoundTrigger)
         << ". Disable SoundTrigger" << endl;
    cout << static_cast<char>(Command::kPlayTone) << ". Play Tone" << endl;
    cout << static_cast<char>(Command::kPlayMultiTone)
         << ". Play Multiple Tones" << endl;
    cout << static_cast<char>(Command::kExit) << ". Exit" << endl;

    cout << endl;
    cout << "Selection: ";
}

CommandMenu::Command CommandMenu::GetCommand() {
    PrintMenu();
    char selection;
    cin >> selection;
    return Command(static_cast<Command>(selection));
}

}; // namespace system
}; // namespace qmmf_test

using ::qmmf_test::system::SystemTest;
using ::qmmf_test::system::CommandMenu;
using ::std::cout;
using ::std::endl;

int main(const int argc, const char * const argv[]) {
  QMMF_INFO("%s: %s() TRACE", TAG, __func__);
  SystemTest test;
  CommandMenu menu;

  bool keep_running = true;
  while (keep_running) {
    CommandMenu::Command command = menu.GetCommand();

    switch(command) {
      case CommandMenu::Command::kConnect:
        test.Connect();
        break;
      case CommandMenu::Command::kDisconnect:
        test.Disconnect();
        break;
      case CommandMenu::Command::kEnableDeviceEvents:
        test.EnableDeviceEvents();
        break;
      case CommandMenu::Command::kQueryDevices:
        test.QueryDevices();
        break;
      case CommandMenu::Command::kEnableSoundTrigger:
        test.EnableSoundTrigger();
        break;
      case CommandMenu::Command::kDisableSoundTrigger:
        test.DisableSoundTrigger();
        break;
      case CommandMenu::Command::kPlayTone:
        test.PlayTone(false);
        break;
      case CommandMenu::Command::kPlayMultiTone:
        test.PlayTone(true);
        break;
      case CommandMenu::Command::kExit:
        test.Disconnect();
        keep_running = false;
        break;
      default:
        cout << endl;
        cout << "Invalid selection" << endl;
        break;
    }
  }
  return 0;
}
