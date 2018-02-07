/*
 * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

#include <cstddef>
#include <cstdlib>
#include <vector>
#include <string>

#include "qmmf-sdk/qmmf_system_params.h"

namespace qmmf {
namespace system {

class SystemClient;

// Client interface for sound trigger, tones, and device and codec capability
// discovery.
//
// Some of the APIs in the class are sync and others async.
// The API documentation explicitly says whether the API is async.
//
// Callbacks used for async communication from system class are expected
// to be lambda functions. The client is expected to capture the context
// while setting the callback.
class System {
 public:
  System();

  ~System();

  // Connect to System service and set callback.
  status_t Connect(const SystemCb& callback);

  // Disconnect from System service.
  status_t Disconnect();

  // Loads a sound model into the sound trigger framework.
  //
  // There may only be one sound model loaded at any given point of time.
  // Any attempt to load a sound model without unloading the previously
  // loaded sound model will result in an error being returned.
  status_t LoadSoundModel(const SoundModel& soundmodel);

  // Unloads a sound model from the sound trigger framework.
  // The sound trigger framework must be disabled before calling this method.
  status_t UnloadSoundModel();

  // Enables the sound trigger framework, which begins listening for the
  // trigger utterance.
  //
  // A callback must be registered along with sound trigger enablement.
  // This callback is used by the system for trigger event notification and
  // sound trigger framework-specific errors (asynchronous).
  //
  // The sound trigger framework has only two states:  enabled and disabled.
  // The notion of multiple sound trigger sesions does not exist.
  status_t EnableSoundTrigger(const TriggerCb& callback);

  // Disables the sound trigger framework.
  status_t DisableSoundTrigger();

  // Register a callback, which is used by the system for device state
  // change notification and device-specific errors (asynchronous).
  status_t RegisterForDeviceEvents(const DeviceCb& callback);

  // Retrieves the list of connected audio and video devices.
  status_t QueryDeviceInfo(::std::vector<DeviceInfo>* devices);

  // Retrieves the capabilities for the given device.
  status_t QueryDeviceCapabilities(const DeviceId device,
                                   DeviceCaps* caps);

  // Retrieves the list of audio, video and image codecs.
  status_t QueryCodecInfo(::std::vector<CodecInfo>* codecs);

  // Plays the given tone on the given audio output devices.
  //
  // A callback must be registered along with tone playback.
  // This callback is used by the system for playback completion notification
  // and tone playback specific errors (asynchronous).
  //
  // There may only be one tone being played at any given point of time.
  // Any attempt to play a tone without waiting for the previous tone to
  // finish will result in an error being returned.
  status_t PlayTone(const ::std::vector<DeviceId>& devices,
                    const Tone& tone,
                    const ToneCb& callback);

private:
  SystemClient* system_client_;
};

}; // namespace system
}; // namespace qmmf
