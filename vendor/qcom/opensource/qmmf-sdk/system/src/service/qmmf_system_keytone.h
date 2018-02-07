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
#include <thread>
#include <vector>

#include "common/audio/inc/qmmf_audio_definitions.h"
#include "common/audio/inc/qmmf_audio_endpoint.h"
#include "qmmf-sdk/qmmf_device.h"
#include "qmmf-sdk/qmmf_system_params.h"
#include "system/src/service/qmmf_system_common.h"
#include "system/src/service/qmmf_system_ion.h"

namespace qmmf {
namespace system {

class SystemKeytone {
 public:
  SystemKeytone();
  ~SystemKeytone();

  status_t PlayTone(const SystemHandle system_handle,
                    const ::std::vector<::qmmf::DeviceId>& devices,
                    const Tone& tone,
                    const SystemToneHandler& handler);

 private:
  enum class SystemMessageType {
    kMessageError,
    kMessageBuffer,
  };

  struct SystemMessage {
    SystemMessageType type;
    union {
      int32_t error;
      ::qmmf::common::audio::AudioBuffer buffer;
    };
  };

  static void ThreadEntry(SystemKeytone* source);
  void Thread();

  void ErrorHandler(const int32_t error);
  void BufferHandler(const ::qmmf::common::audio::AudioBuffer& buffer);

  ::qmmf::common::audio::AudioEndPoint* end_point_;
  SystemIon ion_;

  ::std::thread* thread_;
  ::std::mutex message_lock_;
  ::std::queue<SystemMessage> messages_;
  ::std::condition_variable signal_;

  SystemHandle current_handle_;
  SystemToneHandler tone_handler_;
  Tone tone_;

  // disable copy, assignment, and move
  SystemKeytone(const SystemKeytone&) = delete;
  SystemKeytone(SystemKeytone&&) = delete;
  SystemKeytone& operator=(const SystemKeytone&) = delete;
  SystemKeytone& operator=(const SystemKeytone&&) = delete;
};

}; // namespace system
}; // namespace qmmf
