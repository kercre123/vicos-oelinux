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

#include <memory>
#include <vector>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <utils/Log.h>
#include <linux/msm_ion.h>
#include <utils/Condition.h>
#include <utils/KeyedVector.h>
#include <utils/String8.h>
#include <cutils/native_handle.h>
#include <media/msm_media_info.h>

#include "common/qmmf_common_utils.h"
#include "qmmf-sdk/qmmf_avcodec.h"

using namespace android;
using namespace qmmf;
using namespace qmmf::avcodec;
using namespace std;

#define MAX_FILE_NAME 80

typedef  struct ion_allocation_data IonHandleData;

struct TestInitParams {
  uint32_t      record_frame;
  char          input_file[MAX_FILE_NAME];
  char          output_file[MAX_FILE_NAME];
  CodecMimeType codec_type;
  CodecParam    create_param;
};

class InputCodecSourceImpl;
class OutputCodecSourceImpl;

class CodecTest {

public:
  CodecTest();

  ~CodecTest();

  status_t CreateCodec(int argc, char *argv[]);

  status_t DeleteCodec();

  status_t StartCodec();

  status_t StopCodec();

  status_t PauseCodec();

  status_t ResumeCodec();

  status_t SetCodecParameters();

  DefaultKeyedVector<String8, uint32_t>& GetDynamicParam() {
    return dynamic_params_ ;}

private:
  bool IsStop();

  status_t ParseConfig(char *fileName, TestInitParams* params);

  status_t ParseDynamicConfig(char *fileName);

  status_t AllocateBuffer(uint32_t port);

  status_t ReleaseBuffer();

  IAVCodec*                             avcodec_;
  int32_t                               ion_device_;
  Mutex                                 stop_lock_;
  bool                                  stop_;
  vector<BufferDescriptor>              input_buffer_list_;
  vector<BufferDescriptor>              output_buffer_list_;
  vector<IonHandleData>                 input_ion_handle_data;
  vector<IonHandleData>                 output_ion_handle_data;
  shared_ptr<InputCodecSourceImpl>      input_source_impl_;
  shared_ptr<OutputCodecSourceImpl>     output_source_impl_;
  DefaultKeyedVector<String8, uint32_t> dynamic_params_;
}; //class CodecTest

class InputCodecSourceImpl : public ICodecSource {

public:
  InputCodecSourceImpl(char* file_name, uint32_t num_frame);

  ~InputCodecSourceImpl();

  status_t GetBuffer(BufferDescriptor& stream_buffer,
                     void* client_data) override;

  status_t ReturnBuffer(BufferDescriptor& stream_buffer,
                        void* client_data) override;

  status_t NotifyPortEvent(PortEventType event_type,
                           void* event_data) override;

  void BufferStatus();

  void AddBufferList(vector<BufferDescriptor>& list);

private:
  status_t ReadFile(int32_t fd, uint32_t size, int32_t *byte_read);

  FILE*                     input_file_;
  Mutex                     wait_for_frame_lock_;
  Condition                 wait_for_frame_;
  int32_t                   num_frame_read;
  vector<BufferDescriptor>  input_list_;
  TSQueue<BufferDescriptor> input_free_buffer_queue_;
  TSQueue<BufferDescriptor> input_occupy_buffer_queue_;
}; // Class InputCodecSourceImpl

class OutputCodecSourceImpl : public ICodecSource {

public:
  OutputCodecSourceImpl(char* file_name);

  ~OutputCodecSourceImpl();

  status_t GetBuffer(BufferDescriptor& codec_buffer,
                     void* client_data) override;

  status_t ReturnBuffer(BufferDescriptor& codec_buffer,
                        void* client_data) override;

  status_t NotifyPortEvent(PortEventType event_type,
                           void* event_data) override;

  void BufferStatus();

  void AddBufferList(vector<BufferDescriptor>& list);

private:
  int32_t                   file_fd_;
  Mutex                     wait_for_frame_lock_;
  Condition                 wait_for_frame_;
  vector<BufferDescriptor>  output_list_;
  TSQueue<BufferDescriptor> output_free_buffer_queue_;
  TSQueue<BufferDescriptor> output_occupy_buffer_queue_;
}; // Class OutputCodecSourceImpl

class CmdMenu {

public:
  enum CommandType {
    CREATE_CODEC_CMD     = '1',
    DELETE_CODEC_CMD     = '2',
    START_CODEC_CMD      = '3',
    STOP_CODEC_CMD       = '4',
    PAUSE_CODEC_CMD      = '5',
    RESUME_CODEC_CMD     = '6',
    SET_CODEC_PARAM_CMD  = '7',
    EXIT_CMD             = 'X',
    INVALID_CMD          = '0'
  };

  struct Command {
    Command( CommandType cmd)
    : cmd(cmd) {}
    Command()
    : cmd(INVALID_CMD) {}
    CommandType cmd;
  };

  CmdMenu(CodecTest &ctx):ctx_(ctx) {};

  ~CmdMenu() {};

  Command GetCommand();

  void PrintMenu();

  void PrintDynamicParams();

  CodecTest &ctx_;
};
