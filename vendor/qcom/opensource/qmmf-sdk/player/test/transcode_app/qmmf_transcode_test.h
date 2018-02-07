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

#include <map>

#include "qmmf_transcode_params.h"
#include "qmmf_transcode_track.h"

class TranscodeTest {
 public:
  static TranscodeTest* Connect();
  static ::qmmf::status_t Disconnect();

  ::qmmf::transcode::status_t CreateTrack();
  ::qmmf::transcode::status_t DeleteTrack();
  ::qmmf::transcode::status_t StartTrack();
  ::qmmf::transcode::status_t StopTrack();
  ::qmmf::transcode::status_t SetTrackParams();

 private:
  TranscodeTest() {}
  // map of track id's and corresponding Track Class
  ::std::map<int32_t, ::qmmf::transcode::TranscoderTrack*> track_map_;
  static TranscodeTest* instance_;
};  // class TranscodeTest

class CmdMenu {
 public:
  enum CommandType {
    CONNECT_CMD          = '1',
    DISCONNECT_CMD       = '2',
    CREATE_TRACK_CMD     = '3',
    DELETE_TRACK_CMD     = '4',
    START_TRACK_CMD      = '5',
    STOP_TRACK_CMD       = '6',
    EXIT_CMD             = 'X',
    INVALID_CMD          = '0'
  };

  struct Command {
    Command(CommandType cmd)
    : cmd(cmd) {}
    Command()
    : cmd(INVALID_CMD) {}
    CommandType cmd;
  };

  Command GetCommand();

  void PrintMenu();
};  // class CmdMenu
