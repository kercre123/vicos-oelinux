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

#include <vector>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <utils/Log.h>
#include <linux/msm_ion.h>
#include <utils/Condition.h>
#include <utils/KeyedVector.h>
#include <cutils/native_handle.h>
#include <media/msm_media_info.h>

#include <fstream>
#include <cstring>
#include <cstdint>
#include <bitset>
#include <iostream>
#include <qmmf-sdk/qmmf_player_params.h>

using namespace std;
using namespace android;

using namespace qmmf;
using namespace player;


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

class AACfileIO {
 public:

  status_t Fillparams(AudioTrackCreateParam *params);

  status_t GetFrames(void*buffer,uint32_t size_buffer,
                         int32_t* num_frames_read,uint32_t* bytes_read);

  bool isfileopen(){return infile.is_open();}

  status_t writeRaw();

  AACfileIO(const char* file);

  ~AACfileIO();

  int64_t currentTimeus;
  uint64_t Framedurationus;

 private:
  size_t getAdtsFrameLength(uint64_t offset,size_t*headersize);

  uint32_t get_sample_rate(const uint8_t sf_index);

  vector<uint64_t> OffsetVector;
  vector<size_t> frameSize;
  vector<size_t> headerSize;
  vector<uint64_t>::iterator v_OffsetVector;
  vector<size_t>::iterator v_frameSize;
  vector<size_t>::iterator v_headerSize;
  ifstream infile;
  double confidence;
  uint64_t streamSize;
  uint64_t numFrames;
  uint8_t sf_index,profile,channel;
  uint32_t sr;    //sampling rate
  uint64_t duration;
  bool read_completed;
};

class G711fileIO {
 public:

  status_t Fillparams(AudioTrackCreateParam *params);

  status_t GetFrames(void*buffer,uint32_t size_buffer,uint32_t* bytes_read);

  bool isfileopen(){return infile.is_open();}

  status_t writeRaw();

  G711fileIO(const char* file);

  ~G711fileIO();

  int64_t currentTimeus;
  uint64_t Framedurationus;

 private:

  uint64_t starting_offset;
  ifstream infile;
  uint64_t streamSize;
  uint8_t channel;      //number of channels is always 1
  uint32_t sr;    //sampling rate is 16000 if AMR is wide else it is 8000
  bool read_completed;
  bool isAlaw;
  bool isMulaw;
  uint64_t offset;
};

class AMRfileIO {

 public:

  status_t Fillparams(AudioTrackCreateParam *params);

  status_t GetFrames(void*buffer,uint32_t size_buffer,
                         int32_t* num_frames_read,uint32_t* bytes_read);

  bool isfileopen(){return infile.is_open();}

  status_t writeRaw();

  AMRfileIO(const char* file);

  ~AMRfileIO();

  int64_t currentTimeus;
  uint64_t Framedurationus;

 private:

  size_t getFrameSize(bool isWide,unsigned int FT);

  status_t getFrameSizeByOffset(uint64_t offset, bool isWide,
                                         size_t *frameSize);

  vector<uint64_t> OffsetVector;
  vector<uint64_t>::iterator v_OffsetVector;
  vector<size_t> frameSize;
  vector<size_t>::iterator v_frameSize;
  uint64_t starting_offset;
  ifstream infile;
  double confidence;
  uint64_t streamSize;
  uint64_t numFrames;
  uint8_t channel;      //number of channels is always 1
  uint32_t sr;    //sampling rate is 16000 if AMR is wide else it is 8000
  uint64_t duration;
  bool read_completed;
  bool mIsWide;
};
