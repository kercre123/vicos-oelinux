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

#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "qmmf-sdk/qmmf_device.h"
#include "qmmf-sdk/qmmf_system.h"
#include "qmmf-sdk/qmmf_system_params.h"
#include "system/test/samples/qmmf_system_test_wav.h"

namespace qmmf_test {
namespace system {

class SystemTest
{
 public:
  SystemTest();
  ~SystemTest();

  void Connect();
  void Disconnect();

  void EnableDeviceEvents();
  void QueryDevices();
  void EnableSoundTrigger();
  void DisableSoundTrigger();
  void PlayTone(const bool multi_tone);

 private:
  enum class SystemMessageType {
    kMessageToneFinished,
  };

  struct SystemMessage {
    SystemMessageType type;
    int32_t result;
  };

  void ErrorHandler(const int32_t error);
  void DeviceHandler(const qmmf::DeviceInfo& device);
  void TriggerHandler(const int32_t error);
  void ToneHandler(const int32_t error);

  static void StaticThreadEntry(SystemTest* test);
  void ToneThread();

  ::qmmf::system::System system_;

  SystemTestWav wav_;
  ::std::string filename_prefix_;
  bool multi_tone_;

  ::std::thread* thread_;
  ::std::mutex message_lock_;
  ::std::queue<SystemMessage> messages_;
  ::std::condition_variable signal_;

  // disable copy, assignment, and move
  SystemTest(const SystemTest&) = delete;
  SystemTest(SystemTest&&) = delete;
  SystemTest& operator=(const SystemTest&) = delete;
  SystemTest& operator=(const SystemTest&&) = delete;
};

class CommandMenu {
 public:
  enum class Command {
    kConnect             = '1',
    kDisconnect          = '2',
    kEnableDeviceEvents  = '3',
    kQueryDevices        = '4',
    kEnableSoundTrigger  = '5',
    kDisableSoundTrigger = '6',
    kPlayTone            = '7',
    kPlayMultiTone       = '8',
    kExit                = 'X',
    kInvalid             = '0'
  };

  CommandMenu() {};
  ~CommandMenu() {};
  Command GetCommand();
  void PrintMenu();

  // disable copy, assignment, and move
  CommandMenu(const CommandMenu&) = delete;
  CommandMenu(CommandMenu&&) = delete;
  CommandMenu& operator=(const CommandMenu&) = delete;
  CommandMenu& operator=(const CommandMenu&&) = delete;
};

}; // namespace system
}; // namespace qmmf_test
