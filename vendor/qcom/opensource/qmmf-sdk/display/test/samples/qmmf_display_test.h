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

#include "qmmf-sdk/qmmf_display.h"
#include "qmmf-sdk/qmmf_display_params.h"

#include <map>

using namespace qmmf;
using namespace display;
using namespace android;

class DisplayTest
{
 public:
  DisplayTest();

  ~DisplayTest();

  int32_t Connect();

  int32_t Disconnect();

  int32_t CreateDisplay(DisplayType display_type);

  int32_t DestroyDisplay(DisplayType display_type);

  int32_t CreateSurface();

  int32_t DestroySurface();

  int32_t DequeueSurfaceBuffer();

  int32_t QueueSurfaceBuffer();

  int32_t GetDisplayParam();

  int32_t SetDisplayParam();

  int32_t DequeueWBSurfaceBuffer();

  int32_t QueueWBSurfaceBuffer();

  void DisplayCallbackHandler(DisplayEventType event_type, void *event_data,
      size_t event_data_size);

  void SessionCallbackHandler(DisplayEventType event_type, void *event_data,
      size_t event_data_size);

  void DisplayVSyncHandler(int64_t time_stamp);

 private:

  Display* display_;

  uint32_t surface_id;
  SurfaceConfig surface_config;
  SurfaceBuffer surface_buffer;
  SurfaceParam surface_param;
  FILE *file;
  std::map <uint32_t , std::vector<uint32_t> > sessions_;
};

class CmdMenu
{
public:
  enum CommandType {
    CONNECT_CMD                   = '1',
    DISCONNECT_CMD                = '2',
    CREATE_DISPLAY_CMD            = '3',
    DESTROY_DISPLAY_CMD           = '4',
    CREATE_SURFACE_CMD            = '5',
    DESTROY_SURFACE_CMD           = '6',
    DEQUEUE_SURFACE_BUFFER_CMD    = '7',
    QUEUE_SURFACE_BUFFER_CMD      = '8',
    GET_DISPLAY_PARAM_CMD         = '9',
    SET_DISPLAY_PARAM_CMD         = 'A',
    DEQUEUE_WBSURFACE_BUFFER_CMD  = 'B',
    QUEUE_WBSURFACE_BUFFER_CMD    = 'C',
    EXIT_CMD                      = 'X',
    INVALID_CMD                   = '0'
  };

  struct Command {
    Command( CommandType cmd)
    : cmd(cmd) {}
    Command()
    : cmd(INVALID_CMD) {}
      CommandType cmd;
  };

  CmdMenu(DisplayTest &ctx) :  ctx_(ctx) {};

  ~CmdMenu() {};

  Command GetCommand();

  void PrintMenu();

  DisplayTest &ctx_;
};
