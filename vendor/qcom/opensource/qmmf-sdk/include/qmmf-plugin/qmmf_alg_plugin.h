/*
* Copyright (c) 2017, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this vector of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this vector of conditions and the following
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

#include <stdint.h>
#include <sstream>
#include <string>
#include <vector>
#include <memory>

#include "qmmf_alg_types.h"

#define QMMF_ALG_LIB_LOAD_FUNC "QmmfAlgoNew"

namespace qmmf {

namespace qmmf_alg_plugin {

/** Capabilities:
 *    @input_buffer_requirements_: input buffer requirements
 *    @output_buffer_requirements_: output buffer requirements
 *    @plugin_name_ : plugin full name
 *    @inplace_processing_: inplace processing is required
 *    @history_buffer_count_: history buffer count
 *    @runtime_enable_disable_: flag indicating whether runtime enable/disable
 *                              is supported
 *    @crop_support_: image crop capability flag
 *    @scale_support_: image scale capability flag
 *    @lib_version_: library version
 *
 *  This class defines the qmmf algorithm capabilities
 **/
class Capabilities {
 public:
  Capabilities() {};

  Capabilities(std::string plugin_name,
               std::set<PixelFormat> in_pixel_formats,
               std::set<PixelFormat> out_pixel_formats,
               uint32_t buffer_count,
               const bool inplace_processing,
               const float lib_version) {
    in_buffer_requirements_.min_width_         = 16;
    in_buffer_requirements_.min_height_        = 16;
    in_buffer_requirements_.max_width_         = 16382;
    in_buffer_requirements_.max_height_        = 16382;
    in_buffer_requirements_.cached_            = true;
    in_buffer_requirements_.count_             = buffer_count;
    in_buffer_requirements_.stride_alignment_  = 16;
    in_buffer_requirements_.plane_alignment_   = 16;
    in_buffer_requirements_.pixel_formats_     = in_pixel_formats;

    out_buffer_requirements_.min_width_        = 16;
    out_buffer_requirements_.min_height_       = 16;
    out_buffer_requirements_.max_width_        = 16382;
    out_buffer_requirements_.max_height_       = 16382;
    out_buffer_requirements_.cached_           = true;
    out_buffer_requirements_.count_            = buffer_count;
    out_buffer_requirements_.stride_alignment_ = 16;
    out_buffer_requirements_.plane_alignment_  = 16;
    out_buffer_requirements_.pixel_formats_    = out_pixel_formats;

    plugin_name_ = plugin_name;
    inplace_processing_ = inplace_processing;
    runtime_enable_disable_ = true;
    history_buffer_count_ = 0;
    crop_support_ = false;
    scale_support_ = false;
    lib_version_ = lib_version;
  };

  Capabilities(const Capabilities &caps)
      : plugin_name_(caps.plugin_name_),
        in_buffer_requirements_(caps.in_buffer_requirements_),
        out_buffer_requirements_(caps.out_buffer_requirements_),
        inplace_processing_(caps.inplace_processing_),
        history_buffer_count_(caps.history_buffer_count_),
        runtime_enable_disable_(caps.runtime_enable_disable_),
        crop_support_(caps.crop_support_),
        scale_support_(caps.scale_support_),
        lib_version_(caps.lib_version_) {};

  Capabilities(
      const std::string plugin_name,
      const BufferRequirements &in_buffer_requirements,
      const BufferRequirements &out_buffer_requirements,
      const bool inplace_processing, const uint32_t history_buffer_count,
      const bool runtime_enable_disable, const bool crop_support,
      const bool scale_support, const float lib_version)
      : plugin_name_(plugin_name),
        in_buffer_requirements_(in_buffer_requirements),
        out_buffer_requirements_(out_buffer_requirements),
        inplace_processing_(inplace_processing),
        history_buffer_count_(history_buffer_count),
        runtime_enable_disable_(runtime_enable_disable),
        crop_support_(crop_support),
        scale_support_(scale_support),
        lib_version_(lib_version) {};

  virtual ~Capabilities() {};

  std::string ToString(uint32_t indent = 0) const {
    std::stringstream indentation;
    for (uint32_t i = 0; i < indent; i++) indentation << '\t';
    indent++;

    std::stringstream stream;
    stream << indentation.str()
           << "\"plugin_name_\" : " << plugin_name_ << '\n';
    stream << indentation.str() << "\"in_buffer_requirements_\" : {" << '\n'
           << in_buffer_requirements_.ToString(indent) << "}," << '\n';
    stream << indentation.str() << "\"out_buffer_requirements_\" : {" << '\n'
           << out_buffer_requirements_.ToString(indent) << "}," << '\n';
    stream << indentation.str()
           << "\"inplace_processing_\" : " << inplace_processing_ << ",\n";
    stream << indentation.str()
           << "\"history_buffer_count_\" : " << history_buffer_count_ << ",\n";
    stream << indentation.str()
           << "\"runtime_enable_disable_\" : " << runtime_enable_disable_
           << '\n';
    stream << indentation.str()
           << "\"crop_support_\" : " << crop_support_ << '\n';
    stream << indentation.str()
           << "\"scale_support_\" : " << scale_support_ << '\n';
    stream << indentation.str()
           << "\"lib_version_\" : " << lib_version_ << '\n';
    return stream.str();
  }

  std::string             plugin_name_;
  BufferRequirements      in_buffer_requirements_;
  BufferRequirements      out_buffer_requirements_;
  bool                    inplace_processing_;
  uint32_t                history_buffer_count_;
  bool                    runtime_enable_disable_;
  bool                    crop_support_;
  bool                    scale_support_;
  float                   lib_version_;
};

/** IEventListener
 *
 * Algorithm interface
 *
 **/
class IEventListener {
 public:
  virtual ~IEventListener(){};

  /** OnFrameProcessed
   *    @input_buffer: input buffer
   *
   * Indicates that input buffer is processed
   *
   * return: void
   **/
  virtual void OnFrameProcessed(const AlgBuffer &input_buffer) = 0;

  /** OnFrameReady
   *    @output_buffer: output buffer
   *
   * Indicates that output buffer is processed
   *
   * return: void
   **/
  virtual void OnFrameReady(const AlgBuffer &output_buffer) = 0;

  /** OnError
   *    @err: error id
   *
   * Indicates runtime error
   *
   * return: void
   **/
  virtual void OnError(RuntimeError err) = 0;
};

/** IAlgPlugin
 *
 * Algorithm interface
 *
 **/
class IAlgPlugin {
 public:
  virtual ~IAlgPlugin(){};

  /** GetCaps
   *
   * Gets algorithm capabilities and requirements
   *
   * return: algorithm capabilities
   **/
  virtual Capabilities GetCaps() const = 0;

  /** SetCallbacks
   *    @event_listener: event listener
   *
   * Set callbacks
   *
   * return: void
   **/
  virtual void SetCallbacks(IEventListener *event_listener) = 0;

  /** GetInputRequirements
   *    @out: output parameters
   *
   * This function returns the input requirements for given outputs
   *
   * return: input requirements
   **/
  virtual Requirements GetInputRequirements(
      const std::vector<Requirements> &out) = 0;

  /** Configure
   *
   * Set algorithm specific config data
   *    @config_json_data: config data in JSON format
   *
   * return: void
   **/
  virtual void Configure(const std::string config_json_data) = 0;

  /** RegisterInputBuffers
  *    @buffers: vector of input buffers to register
  *
  * Register input buffers to qmmf algo library. All buffers should be
  * registered before passed to the library for processing.
  *
  * return: void
  **/
  virtual void RegisterInputBuffers(
      const std::vector<AlgBuffer> &buffers) = 0;

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
  virtual void UnregisterInputBuffers(
      const std::vector<AlgBuffer> &buffers) = 0;

  /** RegisterOutputBuffers
  *    @buffers: vector of output buffers to register
  *
  * Register output buffers to qmmf algo library. All buffers should be
  * registered before passed to the library for processing.
  *
  * return: void
  **/
  virtual void RegisterOutputBuffers(
      const std::vector<AlgBuffer> &buffers) = 0;

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
  virtual void UnregisterOutputBuffers(
      const std::vector<AlgBuffer> &buffers) = 0;

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
  virtual void Abort() = 0;

  /** Process
   *    @input_buffers: vector of input buffers
   *    @output_buffers: vector of output buffers
   *
   * main qmmf algo function to process image data
   *
   * return: void
   **/
  virtual void Process(const std::vector<AlgBuffer> &input_buffers,
                       const std::vector<AlgBuffer> &output_buffers) = 0;
};

/* QmmfAlgLoadPlugin
 *    @calibration_data: calibration data
 *
 * Creates new algorithm instance
 *
 * return: pointer to new algorithm instance
 **/
typedef IAlgPlugin *(*QmmfAlgLoadPlugin)(
    const std::vector<uint8_t> &calibration_data);

}; // namespace qmmf_alg_plugin

}; // namespace qmmf
