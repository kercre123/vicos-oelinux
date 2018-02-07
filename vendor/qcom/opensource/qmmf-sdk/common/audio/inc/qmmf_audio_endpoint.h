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

#include <cstdint>
#include <vector>

#include "common/audio/inc/qmmf_audio_definitions.h"

namespace qmmf {
namespace common {
namespace audio {

class AudioEndPointClient;

//
// Client interface for audio playback or capture.
//
// An instance of this class represents the point from which either audio data
// will be produced (when configured as a source), or audio data will be
// consumed (when configured as a sink).  The endpoint, itself, needs to be
// linked with one or more audio devices.
//
// All methods are synchronous unless noted explicitly below.
//
// Typical usage of this API:
// ---
// foo() {
//   AudioEndPoint aep;
//
//   aep.Connect(...);
//   aep.Configure(...);
//   aep.GetLatency(...);
//   aep.GetBufferSize(...);
//   aep.SetParam(...);  NOTE: one or more times for each parameter
//   aep.Start();
//   aep.SendBuffers(...);  NOTE:  repeatedly call to stream data
//   aep.Stop(...);
//
//   aep.Disconnect();
// }
//
class AudioEndPoint {
 public:
  AudioEndPoint();
  ~AudioEndPoint();

  //
  // Connects to the audio service and sets callback handler.
  //
  // handler (input): lambda function serving as a handler for asynchronous
  //                  messages from the audio service. The client is expected
  //                  to capture the context while setting the handler.
  //
  // Returns error code.
  //
  int32_t Connect(const AudioEventHandler& handler);

  //
  // Disconnects from the audio service.  All configuration information for
  // this endpoint is removed.  The disconnection will fail if the endpoint is
  // not stopped, i.e. in the Idle or New state.
  //
  // Returns error code.
  //
  int32_t Disconnect();

  //
  // Configures the endpoint.  The configuration will fail if the endpoint is
  // not in the New state.
  //
  // type (input): indicates source or sink
  // devices (input): list of devices to link to the endpoint
  // metadata (input): description of audio data used by the endpoint
  //
  // Returns error code.
  //
  int32_t Configure(const AudioEndPointType type,
                    const ::std::vector<DeviceId>& devices,
                    const AudioMetadata& metadata);

  //
  // Notifies the endpoint to begin streaming audio.
  //
  // Returns error code.
  //
  int32_t Start();

  //
  // Notifies the endpoint to stop streaming audio.
  //
  // flush (input): indicates that all bufferes committed to the endpoint be
  //                streamed to the device before stopping.
  //
  // Returns error code.
  //
  int32_t Stop(const bool flush);

  //
  // Notifies the endpoint to pause the stream.  Committed buffers will be
  // retained.
  //
  // Returns error code.
  //
  int32_t Pause();

  //
  // Notifies the endpoint to resume the stream.
  //
  // Returns error code.
  //
  int32_t Resume();

  //
  // NOTE: Asynchronous
  // For a source endpoint, requests the endpoint to fill the given list of
  // empty buffers with audio data.  Once filled, each of the buffers will be
  // returned via the handler that was registered with Connect().
  // For a sink endpoint, delivers a list of buffers filled with audio data to
  // the endpoint to consume.  Once a buffer is consumed, it will be returned
  // via the handler that was registered with Connect().
  //
  // buffers (input): list of buffers to send to the endpoint
  //
  // Returns error code.
  //
  int32_t SendBuffers(const ::std::vector<AudioBuffer>& buffers);

  //
  // Request for the endpoint to device latency.  In cases of multiple linked
  // devices, the largest latency will be returned.
  //
  // latency (output): endpoint to device latency in milliseconds
  //
  // Returns error code.
  //
  int32_t GetLatency(int32_t* latency);

  //
  // Request for the optimal buffer size that the client should use with the
  // endpoint.
  //
  // buffer_size (output): recommended buffer_size for audio data
  //
  // Returns error code.
  //
  int32_t GetBufferSize(int32_t* buffer_size);

  //
  // Request for the new audio paramater to be set to the given value.
  //
  // type (input): audio parameter to set
  // data (input): new audio parameter value to use
  //
  // Returns error code.
  //
  int32_t SetParam(const AudioParamType type, const AudioParamData& data);

  //
  // Request for the rendering time taken by ADSP to render audio frames
  // since the output has exited standby.
  //
  // frames (output): number of frmaes rendered
  // time (output): time elapsed in millisecond
  //
  // Returns error code.
  //
  int32_t GetRenderedPosition(uint32_t* frames, uint64_t* time);

 private:
  AudioEndPointClient* audio_endpoint_client_;

  // Disable copy, assignment, and move
  AudioEndPoint(const AudioEndPoint&) = delete;
  AudioEndPoint(AudioEndPoint&&) = delete;
  AudioEndPoint& operator=(const AudioEndPoint&) = delete;
  AudioEndPoint& operator=(const AudioEndPoint&&) = delete;
};

}; // namespace audio
}; // namespace common
}; // namespace qmmf
