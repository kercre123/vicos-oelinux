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

#include <cstddef>
#include <cstdlib>
#include <vector>
#include <string>

#include "qmmf_display_params.h"

namespace qmmf {
namespace display {

class DisplayClient;
class Display
{
public:
  Display();

  ~Display();

  // Connect to display service.
  status_t Connect();

  // Disconnect from display service.
  // All the surfaces should be deleted before calling
  // Disconnet Api.
  status_t Disconnect();

  //Create a Display based on Display type
  status_t CreateDisplay(DisplayType type, DisplayCb& cb);

  //Destroy a Display based on Display type
  status_t DestroyDisplay(DisplayType type);

  // This API internally calls prepare() which checks surface properties and
  // check whether one of the available pipe's can be assigned to this surface.
  // If surface properties meet the requirement of available pipe capabilities,
  // one of the available pipe is assigned to this layer
  // Surface represents the layer (YUV or RGB) associated with a display.
  status_t CreateSurface(const SurfaceConfig &surface_config,
      uint32_t* surface_id);

  status_t DestroySurface(const uint32_t surface_id);

  // This API gets the empty buffer to be used by the client for rendering.
  status_t DequeueSurfaceBuffer(const uint32_t surface_id,
      SurfaceBuffer &surface_buffer);

  // The client renders the data into the empty buffer and calls this API to
  // push this data for composition and display.
  status_t QueueSurfaceBuffer(const uint32_t surface_id,
      const SurfaceBuffer &surface_buffer, const SurfaceParam &surface_param);

  status_t GetDisplayParam(DisplayParamType param_type, void *param,
      size_t param_size);

  // Sets Dynamic display params
  status_t SetDisplayParam(DisplayParamType param_type, const void *param,
      size_t param_size);

  // This API gets the composed layers data for WFD usecase
  status_t DequeueWBSurfaceBuffer(const uint32_t surface_id,
      SurfaceBuffer &surface_buffer);

  // The client provides the empty writeback buffers to display.
  status_t QueueWBSurfaceBuffer(const uint32_t surface_id,
      const SurfaceBuffer &surface_buffer);

private:
  DisplayClient* display_client_;
};

};

}; // namespace qmmf::display

