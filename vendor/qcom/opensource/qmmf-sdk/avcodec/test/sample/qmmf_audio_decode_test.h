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

//for parser
#include <fstream>
#include <cstring>
#include <cstdint>
#include <bitset>
#include <iostream>

#include "common/qmmf_common_utils.h"
#include "qmmf-sdk/qmmf_avcodec.h"

using namespace android;
using namespace qmmf;
using namespace qmmf::avcodec;
using namespace std;

#define MAX_FILE_NAME 80

struct __attribute__((__packed__)) g711_header {
  uint32_t riff_id;
  uint32_t riff_sz;
  uint32_t riff_fmt;
  uint32_t fmt_id;
  uint32_t fmt_sz;
  uint16_t audio_format;
  uint16_t num_channels;
  uint32_t sample_rate;
  uint32_t byte_rate;       /* sample_rate * num_channels * bps / 8 */
  uint16_t block_align;     /* num_channels * bps / 8 */
  uint16_t bits_per_sample;
  uint16_t extension_size;
  uint32_t fact_id;
  uint32_t fact_sz;
  uint32_t sample_length;
  uint32_t data_id;
  uint32_t data_sz;
};

typedef  struct ion_allocation_data IonHandleData;

enum class AudioFileType{
  kAAC,
  kAMR,
  kG711
};

AudioFileType audiofiletype;

struct TestInitParams {
  uint32_t      record_frame;
  char          input_file[MAX_FILE_NAME];
  char          output_file[MAX_FILE_NAME];
  CodecMimeType codec_type;
  CodecParam    create_param;
};

class InputCodecSourceImpl;
class OutputCodecSourceImpl;

class AACfileIO {
public:
  status_t Fillparams(TestInitParams *params);
  status_t GetFrames(void*buffer,uint32_t size_buffer,int32_t* num_frames_read,uint32_t* bytes_read);
  bool isfileopen(){return infile.is_open();}
  static AACfileIO* createAACfileIOobj(const char* file);
  ~AACfileIO();
  int64_t currentTimeus;
  uint64_t Framedurationus;
private:
  AACfileIO(const char* file);

  size_t getAdtsFrameLength(uint64_t offset,size_t*headersize);
  uint32_t get_sample_rate(const uint8_t sf_index);

  vector<uint64_t> OffsetVector;
  vector<size_t> frameSize;
  vector<size_t> headerSize;

  ifstream infile;
  double confidence;
  uint64_t streamSize;
  uint64_t numFrames;
  uint8_t sf_index;
  uint32_t sr;    //sampling rate
  uint8_t profile;
  uint8_t channel;
  uint64_t duration;
  uint64_t starting_offset;
  bool read_completed;

  static AACfileIO* aacfileIO_;
};

class AMRfileIO {
public:
  status_t Fillparams(TestInitParams *params);
  status_t GetFrames(void*buffer,uint32_t size_buffer,int32_t* num_frames_read,uint32_t* bytes_read);
  bool isfileopen(){return infile.is_open();}
  static AMRfileIO* createAMRfileIOobj(const char* file);
  ~AMRfileIO();
  int64_t currentTimeus;
  uint64_t Framedurationus;
private:
  AMRfileIO(const char* file);

  size_t getFrameSize(bool isWide,unsigned int FT);
  status_t getFrameSizeByOffset(uint64_t offset, bool isWide, size_t *frameSize);

  vector<uint64_t> OffsetVector;
  vector<size_t> frameSize;

  ifstream infile;
  double confidence;
  uint64_t streamSize;
  uint64_t numFrames;
  uint8_t channel;      //number of channels is always 1
  uint32_t sr;    //sampling rate is 16000 if AMR is wide else it is 8000
  uint64_t duration;
  uint64_t starting_offset;
  bool read_completed;
  bool mIsWide;

  static AMRfileIO* amrfileIO_;
};

class G711fileIO {
public:
  status_t Fillparams(TestInitParams *params);
  status_t GetFrames(void*buffer,uint32_t size_buffer,uint32_t* bytes_read);
  bool isfileopen(){return infile.is_open();}
  static G711fileIO* createG711fileIOobj(const char* file);
  ~G711fileIO();
  int64_t currentTimeus;
  uint64_t Framedurationus;
private:
  G711fileIO(const char* file);

  ifstream infile;
  uint64_t streamSize;
  uint32_t sr;    //sampling rate is 16000 if AMR is wide else it is 8000
  uint8_t channel;      //number of channels is always 1
  uint64_t starting_offset;
  bool read_completed;
  bool isAlaw;
  bool isMulaw;

  static G711fileIO* g711fileIO_;
};

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
  AACfileIO*                            aacfileIO_;
  AMRfileIO*                            amrfileIO_;
  G711fileIO*                           g711fileIO_;
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
                           void* event_data)  override;

  void BufferStatus();

  void AddBufferList(vector<BufferDescriptor>& list);

private:
  AACfileIO*                aacfileIO_;
  AMRfileIO*                amrfileIO_;
  G711fileIO*               g711fileIO_;
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
