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

#include <cstring>

#include "qmmf-plugin/qmmf_alg_plugin.h"
#include "qmmf-plugin/qmmf_alg_utils.h"

namespace qmmf {

namespace qmmf_alg_plugin {

/** QmmfSimpleTestAlgo
 *    @registered_input_buffers_: vector of registered input buffers
 *    @registered_output_buffers_: vector of registered output buffers
 *    @config_json_data_: config data in JSON format
 *    @caps_: algorithm capabilities
 *
 * Test outplace algorithm
 *
 **/
class QmmfSimpleTestAlgo : public IAlgPlugin {
 public:
  QmmfSimpleTestAlgo()
      : listener_(nullptr),
        caps_("SimpleTest",
              BufferRequirements(160, 120, 3840, 2160,
                                 true, 1, 0, 0,
                                 {kNv12, kNv21}),
              BufferRequirements(160, 120, 3840, 2160,
                                 true, 1, 0, 0,
                                 {}),
              false, 0, false, false, false, 1.0) {}

 protected:
  /** GetCaps
   *
   * Gets algorithm capabilities and requirements
   *
   * return: algorithm capabilities
   **/
  Capabilities GetCaps() const { return caps_; }

  /** SetCallbacks
   *    @event_listener: event listener
   *
   * Set callbacks
   *
   * return: void
   **/
  void SetCallbacks(IEventListener *event_listener) {
    listener_ = event_listener;
  }

  /** Configure
   *
   * Set algorithm specific config data
   *    @config_json_data: config data in JSON format
   *
   * return: void
   **/
  void Configure(const std::string config_json_data) {
    config_json_data_ = config_json_data;
  }

  /** GetInputRequirements
   *    @out: output parameters
   *
   * This function returns the input requirements for given outputs
   *
   * return: Requirments
   **/
  Requirements GetInputRequirements(const std::vector<Requirements> &out) {
    if (out.size() != 1) {
      Utils::ThrowException(__func__, "only one output is supported");
    }
    return out.front();
  }

  /** RegisterInputBuffers
  *    @buffers: vector of input buffers to register
  *
  * Register input buffers to qmmf algo library. All buffers should be
  * registered before passed to the library for processing.
  *
  * return: void
  **/
  void RegisterInputBuffers(const std::vector<AlgBuffer> &buffers) {
    for (AlgBuffer b : buffers) registered_input_buffers_.push_back(b);
  }

  /** UnregisterInputBuffers
  *    @buffers: vector of input buffers to unregister
  *
  * Unregister input buffers from the library. After this call buffers
  * can not be used for processing. Library responsibility is to
  * free all references to this buffers. If buffers are in library
  * processing queue unregister should return an error.
  *
  * return: void
  **/
  void UnregisterInputBuffers(const std::vector<AlgBuffer> &buffers) {
    registered_input_buffers_.remove_if([&buffers](const AlgBuffer rb) {
      for (AlgBuffer b : buffers) {
        if (b.fd_ == rb.fd_) {
          return true;
        }
      };
      return false;
    });
  }

  /** RegisterOutputBuffers
  *    @buffers: vector of output buffers to register
  *
  * Register output buffers to qmmf algo library. All buffers should be
  * registered before passed to the library for processing.
  *
  * return: void
  **/
  void RegisterOutputBuffers(const std::vector<AlgBuffer> &buffers) {
    for (AlgBuffer b : buffers) registered_output_buffers_.push_back(b);
  }

  /** UnregisterOutputBuffers
  *    @buffers: vector of output buffers to unregister
  *
  * Unregister output buffers from the library. After this call buffers
  * can not be used for processing. Library responsibility is to
  * free all references to this buffers. If buffers are in library
  * processing queue unregister should return an error.
  *
  * return: void
  **/
  void UnregisterOutputBuffers(const std::vector<AlgBuffer> &buffers) {
    registered_output_buffers_.remove_if([&buffers](const AlgBuffer rb) {
      for (AlgBuffer b : buffers) {
        if (b.fd_ == rb.fd_) {
          return true;
        }
      };
      return false;
    });
  }

  /** Abort
  *
  * Aborts current processing in the earliest possible stage and
  * flushes all buffers from library processing queue.
  * All buffers should be returned with corresponding callbacks,
  * error status should be set if output buffers are released
  * and not yet processed.
  * After this call library processing queue should be empty.
  *
  * return: void
  **/
  void Abort() {}

  /** Process
   *    @input_buffers: vector of input buffers
   *    @output_buffers: vector of output buffers
   *
   * main qmmf algo function to process image data
   *
   * return: void
   **/
  void Process(const std::vector<AlgBuffer> &input_buffers,
               const std::vector<AlgBuffer> &output_buffers) {
    Validate(input_buffers, output_buffers);

    if (listener_ == nullptr) {
      Utils::ThrowException(__func__, "listener is not set");
    }

    for (AlgBuffer b : output_buffers) {
      if (b.size_ != input_buffers[0].size_) {
        Utils::ThrowException(
            __func__, "output buffer size is different than input buffer size");
      }

      memcpy(b.vaddr_, input_buffers[0].vaddr_, b.size_);
    }

    for (AlgBuffer b : input_buffers) {
      listener_->OnFrameProcessed(b);
    }
    for (AlgBuffer b : output_buffers) {
      listener_->OnFrameReady(b);
    }
  }

  /** Validate
   *    @input_buffers: vector of input buffers
   *    @output_buffers: vector of output buffers
   *
   * validate buffers
   *
   * return: void
   **/
  void Validate(const std::vector<AlgBuffer> &input_buffers,
                const std::vector<AlgBuffer> &output_buffers) {
    if (input_buffers.size() != caps_.in_buffer_requirements_.count_) {
      Utils::ThrowException(__func__, "Input buffer count is not correct");
    }
    if (input_buffers.size() < 1) {
      Utils::ThrowException(__func__,
                                "At least one input buffer is required");
    }
    for (AlgBuffer b : input_buffers) {
      b.Validate(caps_.in_buffer_requirements_);

      bool found = false;
      for (AlgBuffer rb : registered_input_buffers_) {
        if (b.fd_ == rb.fd_) {
          found = true;
          break;
        }
      }
      if (!found) {
        Utils::ThrowException(__func__, "Input buffer not mapped");
      }

      if (output_buffers.size() != caps_.out_buffer_requirements_.count_) {
        Utils::ThrowException(__func__,
                                  "Output buffer count is not correct");
      }
      for (AlgBuffer b : output_buffers) {
        b.Validate(caps_.out_buffer_requirements_);

        bool found = false;
        for (AlgBuffer rb : registered_output_buffers_) {
          if (b.fd_ == rb.fd_) {
            found = true;
            break;
          }
        }
        if (!found) {
          Utils::ThrowException(__func__, "Output buffer not mapped");
        }
      }
    }
  }

  std::list<AlgBuffer> registered_input_buffers_;
  std::list<AlgBuffer> registered_output_buffers_;
  std::string config_json_data_;
  IEventListener *listener_;
  Capabilities caps_;
};

}; // namespace qmmf_alg_plugin

}; // namespace qmmf
