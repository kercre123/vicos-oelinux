/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *     Neither the name of The Linux Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORSAA
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define TAG "CodecTest"
#define TAG2 "AACfileIO"
#define TAG3 "AMRfileIO"
#define TAG4 "G711fileIO"

#include <memory>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "avcodec/test/sample/qmmf_audio_decode_test.h"

#define OFFSET_TABLE_LEN    300
#define MAX_NUM_FRAMES_PER_BUFF_AMR  64
#define FORMAT_ALAW  0x0006
#define FORMAT_MULAW 0x0007

using ::std::shared_ptr;
using ::std::make_shared;

AACfileIO* AACfileIO::aacfileIO_ = nullptr;

AACfileIO* AACfileIO::createAACfileIOobj(const char* file) {
  QMMF_INFO("%s:%s:%s Enter",TAG,TAG2,__func__);
  if (aacfileIO_ == nullptr) {
    if (file == nullptr) {
      QMMF_ERROR("%s:%s:%s  file pointer is NULL",TAG,TAG2,__func__);
      assert(0);
    }
    aacfileIO_ = new AACfileIO(file);
    if (aacfileIO_ == nullptr) {
        QMMF_ERROR("%s:%s:%s Could not create AACfileIO object",TAG,TAG2,__func__);
        assert(0);
    }
    QMMF_INFO("%s:%s:%s Exit",TAG,TAG2,__func__);
    return aacfileIO_;
  }
  QMMF_INFO("%s:%s:%s Exit",TAG,TAG2,__func__);
  return aacfileIO_;
}

AACfileIO::AACfileIO(const char*file)
  : currentTimeus(0),
    Framedurationus(0),
    infile(file),
    confidence(0),
    streamSize(0),
    numFrames(0),
    sf_index(0),
    sr(0),
    profile(0),
    channel(0),
    duration(0),
    starting_offset(0),
    read_completed(false) {
  QMMF_INFO("%s:%s:%s Enter",TAG,TAG2,__func__);
  QMMF_INFO("%s:%s:%s Exit",TAG,TAG2,__func__);
}

AACfileIO::~AACfileIO() {
  QMMF_INFO("%s:%s:%s Enter",TAG,TAG2,__func__);
  if (infile.is_open()) {
    infile.close();
  }
  QMMF_INFO("%s:%s:%s Exit",TAG,TAG2,__func__);
}


uint32_t AACfileIO::get_sample_rate(const uint8_t index) {
  static const uint32_t sample_rates[] =
  {
    96000, 88200, 64000, 48000, 44100, 32000,
    24000, 22050, 16000, 12000, 11025, 8000
  };

  if (index < sizeof(sample_rates) / sizeof(sample_rates[0])) {
        return sample_rates[index];
  }

  return 0;
}

size_t AACfileIO::getAdtsFrameLength(uint64_t offset,size_t*headersize) {
    const size_t kAdtsHeaderLengthNoCrc = 7;
    const size_t kAdtsHeaderLengthWithCrc = 9;

    size_t framesize = 0;

    char syncword[2];
    infile.seekg(offset);
    if (infile.read(syncword,2).gcount() != 2) {
        return 0;
    }

    infile.seekg(offset);

    if (((uint8_t)syncword[0] != 0xff) || (((uint8_t)syncword[1] & 0xf6) != 0xf0)) {
        return 0;
    }

    //lastbit of syncword[1] is the protection absent
    uint8_t protectionAbsent = (uint8_t)syncword[1] & 0x1;

    //ignoring the next byte after 2 bytes have been read as syncword
    infile.seekg(offset + 3);
    char header[3];

    if (infile.read(header,3).gcount() != 3) {
        return 0;
    }

    infile.seekg(offset);

    framesize = ((size_t)header[0] & 0x3) << 11 | (size_t)header[1] << 3 | (size_t)header[2] >> 5;

    // protectionAbsent is 0 if there is CRC
    size_t headsize = protectionAbsent ? kAdtsHeaderLengthNoCrc : kAdtsHeaderLengthWithCrc;
    if (headsize > framesize) {
        return 0;
    }
    if (headersize != nullptr) {
        *headersize = headsize;
    }

    return framesize;
}

status_t AACfileIO::Fillparams(TestInitParams *params) {
  QMMF_INFO("%s:%s:%s   Enter",TAG,TAG2,__func__);
  size_t pos = 0;
  uint64_t offset = 0;

    while(1) {
    char id3header[10];

    if (infile.read(id3header,sizeof(id3header)).gcount() < (ssize_t)sizeof(id3header)) {
      QMMF_ERROR("%s:%s:%s Error in reading id3header",TAG,TAG2,__func__);
      return -1;
    }

    if (memcmp("ID3",id3header,3)) {
      infile.seekg(pos);
      break;
    }

    // Skip the ID3v2 header.

    size_t len =  (((size_t)id3header[6] & 0x7f) << 21) | (((size_t)id3header[7] & 0x7f) << 14) | (((size_t)id3header[8] & 0x7f) << 7) | ((size_t)id3header[9] & 0x7f);
    len += 10;
    pos += len;
    infile.seekg(pos);
    QMMF_INFO("%s:%s:%s skipped ID3 tag, new starting offset is %lld",TAG,TAG2,__func__,(long long)pos);
  }

  char header[2];

  if (infile.read(header,2).gcount() != 2) {
    QMMF_ERROR("%s:%s:%s Error in reading header[2] / syncwords",TAG,TAG2,__func__);
    return -1;
  }

  infile.seekg(pos);

  if (((uint8_t)header[0] == 0xff) && (((uint8_t)header[1] & 0xf6) == 0xf0)) {
    QMMF_INFO("%s:%s:%s ADTS header found",TAG,TAG2,__func__);
    confidence = 0.2;
    offset = pos;
    starting_offset = pos;
  } else {
    QMMF_ERROR("%s:%s:%s ADTS header not found",TAG,TAG2,__func__);
      return -1;
  }

  infile.seekg(offset+2);
  if (infile.read(header,2).gcount() < 2) {
        QMMF_ERROR("%s:%s:%s Error in reading header[2]  at offset+2 ",TAG,TAG2,__func__);
    return -1;
  }
  infile.seekg(offset);
  //starting 2 bits of header[0] is profile
  profile = ((uint8_t)header[0] >> 6) & 0x3;
  //next 4 bits of header[0] is sf_index
  sf_index = ((uint8_t)header[0] >> 2) & 0xf;
  sr = get_sample_rate(sf_index);
  if (sr == 0) {
        QMMF_ERROR("%s:%s:%s Sampling rate could not be found",TAG,TAG2,__func__);
    return -1;
  }

  channel = ((uint8_t)header[0] & 0x1) << 2 | ((uint8_t)header[1] >> 6);

  infile.seekg(0,infile.end);
  streamSize = infile.tellg();
  infile.seekg(offset);
  size_t framesize;
  size_t headersize;
  QMMF_INFO("Size of size_t = %d",sizeof(size_t));
  while(offset < streamSize) {
      if ((framesize = getAdtsFrameLength(offset,&headersize)) == 0) {
          QMMF_ERROR("%s:%s:%s Error from AACfileIO::getAdtsFrameLength function",TAG,TAG2,__func__);
          return -1;
      }
      QMMF_INFO("%s:%s:%s Current Offset for the ADTS header %llu is %lld with"
                " framesize = %u and headersize = %u" , TAG, TAG2, __func__,
                numFrames + 1, (long long)offset, (uint32_t)framesize,
                (uint32_t)headersize);
      OffsetVector.push_back(offset);
      frameSize.push_back(framesize);
      headerSize.push_back(headersize);
      offset += (uint64_t)framesize;
      numFrames++;
  }

  QMMF_INFO("%s:%s:%s The aac file read completed",TAG,TAG2,__func__);

  Framedurationus = (1024 * 1000000ll + (sr - 1)) / sr;
  duration = numFrames * Framedurationus;

  params->create_param.audio_dec_param.codec = ::qmmf::player::AudioCodecType::kAAC;
  params->create_param.audio_dec_param.sample_rate = sr;
  params->create_param.audio_dec_param.channels = channel;
  params->create_param.audio_dec_param.bit_depth = 16;
  params->create_param.audio_dec_param.bitrate = 0;
  QMMF_INFO("Channel = %d, sampling rate = %d, profile = %d",channel,sr,profile);
  //this is only for ADTS as of now
  params->create_param.audio_dec_param.codec_params.aac.format = AACFormat::kADTS;
  switch(profile) {
    case 0:
      params->create_param.audio_dec_param.codec_params.aac.mode = AACMode::kAALC;
    break;
    case 1:
      params->create_param.audio_dec_param.codec_params.aac.mode = AACMode::kHEVC_v1;
    break;
    case 2:
      params->create_param.audio_dec_param.codec_params.aac.mode = AACMode::kHEVC_v2;
    break;
    default:
      QMMF_ERROR("%s:%s:%s unsupported AAC mode: %d", TAG, TAG2,__func__,profile);
  }

  QMMF_INFO("%s:%s:%s   Exit",TAG,TAG2,__func__);
  return 0;
}

//return value of -1 signifies the end of file i.e. OMX_BUFFERFLAGEOS

status_t AACfileIO::GetFrames(void*buffer,uint32_t size_buffer,int32_t*num_frames_read,uint32_t*bytes_read) {

  QMMF_INFO("%s:%s:%s Enter",TAG,TAG2,__func__);
  if (read_completed) {
        QMMF_ERROR("%s:%s:%s File read has already been completed",TAG,TAG2,__func__);
    return -1;
  }
  char*aac = (char*)buffer;
  static vector<uint64_t>::iterator v_OffsetVector = OffsetVector.begin();
  static vector<size_t>::iterator v_frameSize = frameSize.begin();
  static vector<size_t>::iterator v_headerSize = headerSize.begin();

  *num_frames_read = 0;
  *bytes_read = 0;

  //will read as many frames as can be read
  while(*bytes_read < size_buffer) {
    uint64_t offset =  *v_OffsetVector;
    size_t framesize = *v_frameSize;
    size_t headersize  = *v_headerSize;

    QMMF_INFO("%s:%s:%s offset = %lld frameSize = %u headerSize = %u",TAG,TAG2,__func__,(long long)offset,(uint32_t)framesize,(uint32_t)headersize);

    if (size_buffer - *bytes_read < (uint32_t)framesize) {
        QMMF_INFO("%s:%s:%s No space left in Buffer header(%p)",TAG,TAG2,__func__,buffer);
        return 0;
    }
    uint32_t read_bytes = 0;
    infile.seekg(offset);

    if ((read_bytes  = infile.read(aac,framesize).gcount()) != framesize) {
        QMMF_ERROR("%s:%s:%s Error in reading the %d bytes from aac file, read_bytes = %d",TAG,TAG2,__func__,framesize,read_bytes);
        assert(0);
    }
    aac += read_bytes;
    *bytes_read += read_bytes;
    *num_frames_read += 1;
    v_OffsetVector++;
    v_frameSize++;
    v_headerSize++;
    if ((v_OffsetVector == OffsetVector.end()) && (v_frameSize == frameSize.end()) && (v_headerSize == headerSize.end())) {
      read_completed = true;
      QMMF_INFO("%s:%s:%s The input file has been read completely. Now EOS has to be send",TAG,TAG2,__func__);
      QMMF_INFO("%s:%s:%s Exit",TAG,TAG2,__func__);
      return -1;
    }
  }

  QMMF_INFO("%s:%s:%s Exit",TAG,TAG2,__func__);
  return 0;
}

AMRfileIO* AMRfileIO::amrfileIO_ = nullptr;

AMRfileIO* AMRfileIO::createAMRfileIOobj(const char* file) {
  QMMF_INFO("%s:%s:%s Enter",TAG,TAG3,__func__);
  if (amrfileIO_ == nullptr) {
    if (file == nullptr) {
      QMMF_ERROR("%s:%s:%s  file pointer is NULL",TAG,TAG3,__func__);
      assert(0);
    }
    amrfileIO_ = new AMRfileIO(file);
    if (amrfileIO_ == nullptr) {
        QMMF_ERROR("%s:%s:%s Could not create AMRfileIO object",TAG,TAG3,__func__);
        assert(0);
    }
    QMMF_INFO("%s:%s:%s Exit",TAG,TAG3,__func__);
    return amrfileIO_;
  }
  QMMF_INFO("%s:%s:%s Exit",TAG,TAG3,__func__);
  return amrfileIO_;
}

AMRfileIO::AMRfileIO(const char*file)
  : currentTimeus(0),
    Framedurationus(20000),
    infile(file),
    confidence(0),
    streamSize(0),
    numFrames(0),
    channel(0),
    sr(0),
    duration(0),
    starting_offset(0),
    read_completed(false),
    mIsWide(false) {
  QMMF_INFO("%s:%s:%s Enter",TAG,TAG3,__func__);
  QMMF_INFO("%s:%s:%s Exit",TAG,TAG3,__func__);
}

AMRfileIO::~AMRfileIO() {
  QMMF_INFO("%s:%s:%s Enter",TAG,TAG3,__func__);
  if (infile.is_open()) {
    infile.close();
  }
  QMMF_INFO("%s:%s:%s Exit",TAG,TAG3,__func__);
}


size_t AMRfileIO::getFrameSize(bool isWide,unsigned int FT) {
  static const size_t kFrameSizeNB[16] = {
       95, 103, 118, 134, 148, 159, 204, 244,
        39, 43, 38, 37, // SID
        0, 0, 0, // future use
        0 // no data
    };
   static const size_t kFrameSizeWB[16] = {
        132, 177, 253, 285, 317, 365, 397, 461, 477,
       40, // SID
        0, 0, 0, 0, // future use
       0, // speech lost
        0 // no data
   };

   if (FT > 15 || (isWide && FT > 9 && FT < 14) || (!isWide && FT > 11 && FT < 15)) {
       QMMF_ERROR("%s:%s:%s illegal AMR frame type %d", TAG,TAG3,__func__,FT);
        return 0;
   }

    size_t framesize = isWide ? kFrameSizeWB[FT] : kFrameSizeNB[FT];

    // Round up bits to bytes and add 1 for the header byte.
    framesize = (framesize + 7) / 8 + 1;

    return framesize;
}

status_t AMRfileIO::getFrameSizeByOffset(uint64_t offset, bool isWide, size_t *framesize) {
  infile.seekg(offset);
  char header[1];
  if (infile.read(header,1).gcount() != 1) {
    QMMF_ERROR("%s:%s:%s Could not read the header of AMR at offset = %lld",TAG,TAG3,__func__,(long long)offset);
    assert(0);
    return 0;
  }
  unsigned int FT = ((uint8_t)header[0] >> 3) & 0x0f;
  *framesize = getFrameSize(isWide, FT);
  if (*framesize == 0) {
    QMMF_ERROR("%s:%s:%s AMR framesize is 0",TAG,TAG3,__func__);
    return 0;
  }
  return 1;
}

status_t AMRfileIO::Fillparams(TestInitParams *params) {
  QMMF_INFO("%s:%s:%s   Enter",TAG,TAG3,__func__);
  char header[9];
  if (infile.read(header,sizeof(header)).gcount() != (ssize_t)(sizeof(header))) {
    QMMF_ERROR("%s:%s:%s Could not get AMR MIME TYPE",TAG,TAG3,__func__);
    return -1;
  }
  if (!memcmp(header, "#!AMR\n", 6)) {
    mIsWide = false;
    confidence = 0.5;
    QMMF_INFO("%s:%s:%s AMR MIME TYPE is not Wide",TAG,TAG3,__func__);
  } else if (!memcmp(header, "#!AMR-WB\n", 9)) {
    mIsWide = true;
    confidence = 0.5;
    QMMF_INFO("%s:%s:%s AMR MIME TYPE is Wide",TAG,TAG3,__func__);
  } else {
    QMMF_ERROR("%s:%s:%s Could not get AMR MIME TYPE",TAG,TAG3,__func__);
    return -1;
  }
  starting_offset = mIsWide ? 9 : 6;
  uint64_t offset = starting_offset;
  infile.seekg(0,infile.end);
  streamSize = infile.tellg();
  infile.seekg(offset);
  size_t framesize;
  while(offset < streamSize) {
      if (getFrameSizeByOffset(offset, mIsWide, &framesize) == 0) {
        QMMF_ERROR("%s:%s:%s Could not get frame size by offset",TAG,TAG3,__func__);
        return -1;
      }
      OffsetVector.push_back(offset);
      frameSize.push_back(framesize);
      offset += (uint64_t)framesize;
      duration += Framedurationus;
      numFrames++;
  }

  sr = mIsWide ? 16000 : 8000;
  channel = 1;
  params->create_param.audio_dec_param.codec = ::qmmf::player::AudioCodecType::kAMR;
  params->create_param.audio_dec_param.sample_rate = sr;
  params->create_param.audio_dec_param.channels = channel;
  params->create_param.audio_dec_param.bit_depth = 16;
  params->create_param.audio_dec_param.codec_params.amr.isWAMR = mIsWide;
  params->create_param.audio_dec_param.bitrate = 0;
  if (mIsWide) {
      QMMF_INFO("Channel = %d, sampling rate = %d AMR is Wide",channel,sr);
  } else {
      QMMF_INFO("Channel = %d, sampling rate = %d AMR is not Wide",channel,sr);
  }

  QMMF_INFO("%s:%s:%s Exit",TAG,TAG3,__func__);
  return 0;
}

status_t AMRfileIO::GetFrames(void*buffer,uint32_t size_buffer,int32_t* num_frames_read,uint32_t* bytes_read) {

  QMMF_INFO("%s:%s:%s Enter",TAG,TAG3,__func__);
  if (read_completed) {
        QMMF_ERROR("%s:%s:%s File read has already been completed",TAG,TAG3,__func__);
    return -1;
  }
  char*amr = (char*)buffer;
  static vector<uint64_t>::iterator v_OffsetVector = OffsetVector.begin();
  static vector<size_t>::iterator v_frameSize = frameSize.begin();

  *num_frames_read = 0;
  *bytes_read = 0;

  //will read as many frames as can be read
  while(*bytes_read < size_buffer) {
    uint64_t offset =  *v_OffsetVector;
    size_t framesize = *v_frameSize;
    QMMF_INFO("%s:%s:%s offset = %lld frameSize = %u",TAG,TAG2,__func__,(long long)offset,(uint32_t)framesize);
    if (size_buffer - *bytes_read < (uint32_t)framesize) {

        QMMF_INFO("%s:%s:%s No space left in Buffer header(%p)",TAG,TAG3,__func__,buffer);
        QMMF_INFO("%s:%s:%s Exit",TAG,TAG2,__func__);
        return 0;
    }
    if (*num_frames_read == MAX_NUM_FRAMES_PER_BUFF_AMR) {
        QMMF_INFO("%s:%s:%s MAX_NUM_FRAMES_PER_BUFF_AMR in Buffer header(%p)",TAG,TAG3,__func__,buffer);
        QMMF_INFO("%s:%s:%s Exit",TAG,TAG2,__func__);
        return 0;
    }
    uint32_t read_bytes = 0;
    infile.seekg(offset);
    if ((read_bytes  = infile.read(amr,framesize).gcount()) != framesize) {
        QMMF_ERROR("%s:%s:%s Error in reading the %d bytes from aac file, read_bytes = %d",TAG,TAG3,__func__,framesize,read_bytes);
        assert(0);
    }
    amr += read_bytes;
    *bytes_read += read_bytes;
    *num_frames_read += 1;
    v_OffsetVector++;
    v_frameSize++;
    if ((v_OffsetVector == OffsetVector.end()) && (v_frameSize == frameSize.end())) {
      read_completed = true;
      QMMF_INFO("%s:%s:%s The input file has been read completely. Now EOS has to be send",TAG,TAG3,__func__);
      QMMF_INFO("%s:%s:%s Exit",TAG,TAG3,__func__);
      return -1;
    }
  }

  QMMF_INFO("%s:%s:%s Exit",TAG,TAG3,__func__);
  return 0;
}

G711fileIO* G711fileIO::g711fileIO_ = nullptr;

G711fileIO* G711fileIO::createG711fileIOobj(const char* file) {
    QMMF_INFO("%s:%s:%s Enter",TAG,TAG4,__func__);
    if (g711fileIO_ == nullptr) {
    if (file == nullptr) {
      QMMF_ERROR("%s:%s:%s  file pointer is NULL",TAG,TAG4,__func__);
      assert(0);
    }
    g711fileIO_ = new G711fileIO(file);
    if (g711fileIO_ == nullptr) {
        QMMF_ERROR("%s:%s:%s Could not create G711fileIO object",TAG,TAG4,__func__);
        assert(0);
    }
    QMMF_INFO("%s:%s:%s Exit",TAG,TAG4,__func__);
    return g711fileIO_;
  }
  QMMF_INFO("%s:%s:%s Exit",TAG,TAG4,__func__);
  return g711fileIO_;
}

G711fileIO::G711fileIO(const char*file)
  : currentTimeus(0),
    Framedurationus(0),
    infile(file),
    streamSize(0),
    sr(0),
    channel(0),
    starting_offset(0),
    read_completed(false),
    isAlaw(false),
    isMulaw(false) {
  QMMF_INFO("%s:%s:%s Enter",TAG,TAG4,__func__);
  QMMF_INFO("%s:%s:%s Exit",TAG,TAG4,__func__);
}

G711fileIO::~G711fileIO() {
  QMMF_INFO("%s:%s:%s Enter",TAG,TAG4,__func__);
  if (infile.is_open()) {
    infile.close();
  }
  QMMF_INFO("%s:%s:%s Exit",TAG,TAG4,__func__);
}

status_t G711fileIO::Fillparams(TestInitParams *params) {
  QMMF_ERROR("%s:%s:%s Enter",TAG,TAG4,__func__);
  struct g711_header g711hdr;
  uint32_t read_bytes;
  if ((read_bytes =  infile.read((char*)&g711hdr,sizeof(g711hdr)).gcount()) != (ssize_t)(sizeof(g711hdr))) {
    QMMF_ERROR("%s:%s:%s Error in reading the %d bytes from g711 file for g711 header, read_bytes = %d",TAG,TAG3,__func__,sizeof(g711hdr),read_bytes);
    return -1;
  }
  if ((g711hdr.audio_format != FORMAT_MULAW) && (g711hdr.audio_format != FORMAT_ALAW)) {
    QMMF_INFO("%s:%s:%s g711 file is not MULAW or ALAW format it's format is %d ",TAG,TAG4,__func__,g711hdr.audio_format);
    return -1;
  }

if ((g711hdr.sample_rate != 8000) && (g711hdr.sample_rate != 16000)) {
      QMMF_ERROR("%s:%s:%s samplerate = %d, not supported, Supported samplerates are 8000, 16000",TAG,TAG4,__func__,g711hdr.sample_rate);
    return -1;
  }

if (g711hdr.num_channels != 1) {
      QMMF_ERROR("%s:%s:%s stereo and multi channel are not supported, channels %d",TAG,TAG4,__func__,g711hdr.num_channels);
    return -1;
  }
  sr = g711hdr.sample_rate;
  channel = g711hdr.num_channels;
  starting_offset =  infile.tellg();
  infile.seekg(0,infile.end);
  streamSize = infile.tellg();
  infile.seekg(starting_offset);
  params->create_param.audio_dec_param.codec = ::qmmf::player::AudioCodecType::kG711;
  params->create_param.audio_dec_param.sample_rate = g711hdr.sample_rate;
  params->create_param.audio_dec_param.channels = g711hdr.num_channels;
  params->create_param.audio_dec_param.bit_depth = 16;
  params->create_param.audio_dec_param.bitrate = ((sr*channel)*(params->create_param.audio_dec_param.bit_depth))/8;
  QMMF_INFO(" %s:%s:%s Channel = %d, sampling rate = %d",TAG,TAG4,__func__,channel,sr);

  if (g711hdr.audio_format == FORMAT_MULAW) {
    params->create_param.audio_dec_param.codec_params.g711.mode = G711Mode::kMuLaw;
    isMulaw = true;
    QMMF_INFO("%s:%s:%s Format is MULAW",TAG,TAG4,__func__);
  } else {
    params->create_param.audio_dec_param.codec_params.g711.mode = G711Mode::kALaw;
    isAlaw = true;
    QMMF_INFO("%s:%s:%s Format is ALAW",TAG,TAG4,__func__);
  }
  QMMF_ERROR("%s:%s:%s Exit",TAG,TAG4,__func__);
  return 0;
}

status_t G711fileIO::GetFrames(void*buffer,uint32_t size_buffer,uint32_t* bytes_read) {
  QMMF_INFO("%s:%s:%s Enter",TAG,TAG4,__func__);
  if (read_completed) {
    QMMF_ERROR("%s:%s:%s File read has already been completed",TAG,TAG4,__func__);
    QMMF_INFO("%s:%s:%s Exit",TAG,TAG4,__func__);
    return -1;
  }

  size_buffer = 1024;
  static uint64_t offset = starting_offset;
  infile.seekg(offset);
  char*g711 = (char*)buffer;
  uint32_t bytes_to_read = (streamSize - offset) > size_buffer ? size_buffer : (streamSize - offset);
  uint32_t read_bytes;
  if ((read_bytes = infile.read(g711,bytes_to_read).gcount()) != bytes_to_read) {
    QMMF_ERROR("%s:%s:%s Could not read %d bytes requested, bytes read = %d",TAG,TAG4,__func__,bytes_to_read,read_bytes);
    assert(0);
  }
  *bytes_read = read_bytes;
  offset += (uint64_t)(*bytes_read);
  if (offset >= streamSize) {
    read_completed = true;
    QMMF_INFO("%s:%s:%s The input file has been read completely. Now EOS has to be send",TAG,TAG4,__func__);
    QMMF_INFO("%s:%s:%s Exit",TAG,TAG4,__func__);
    return -1;
  }

   QMMF_INFO("%s:%s:%s Exit",TAG,TAG4,__func__);
  return 0;
}


CodecTest::CodecTest()
    :avcodec_(nullptr),
     ion_device_(-1),
     stop_(false) {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);

  ion_device_ = open("/dev/ion", O_RDONLY);
  if (ion_device_ <= 0) {
    QMMF_ERROR("%s:%s Ion dev open failed %s", TAG, __func__,strerror(errno));
    ion_device_ = -1;
  }



  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

CodecTest::~CodecTest() {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);

  if (avcodec_)
    delete avcodec_;

  if (audiofiletype == AudioFileType::kAAC) {
    QMMF_INFO("%s:%s Deleting AACfileIO class",TAG,__func__);
    delete aacfileIO_;
  } else if (audiofiletype == AudioFileType::kAMR) {
    QMMF_INFO("%s:%s Deleting AMRfileIO class",TAG,__func__);
    delete amrfileIO_;
  } else if (audiofiletype == AudioFileType::kG711) {
    QMMF_INFO("%s:%s Deleting G711fileIO class",TAG,__func__);
    delete g711fileIO_;
  }

  if (ion_device_ > 0) {
    close(ion_device_);
    ion_device_ = -1;
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

status_t CodecTest::CreateCodec(int argc, char *argv[]) {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  status_t ret = 0;

  if ((argc < 2 ) || (strcmp(argv[1], "-c"))) {
    QMMF_INFO("%s:%s Usage: %s -c [INPUT_AAC_FILE_PATH] -f [aac/amr/g711]", TAG, __func__, argv[0]);
    return -1;
  }

  if ((argc < 4)  || (strcmp(argv[3],"-f"))) {
    QMMF_INFO("%s:%s Usage: %s -c [INPUT_AAC_FILE_PATH] -f [aac/amr/g711]", TAG, __func__, argv[0]);
    return -1;
  }

  TestInitParams params;
  memset(&params, 0x0, sizeof(params));
  strncpy(params.input_file,argv[2],strlen(argv[2]));
  strncpy(params.output_file,"/data/misc/qmmf/pcm.wav",strlen("/data/misc/qmmf/pcm.wav"));

  if (!strncmp("aac",argv[4],strlen("aac"))) {
    audiofiletype = AudioFileType::kAAC;
  } else if (!strncmp("amr",argv[4],strlen("amr"))) {
    audiofiletype = AudioFileType::kAMR;
  } else if (!strncmp("g711",argv[4],strlen("g711"))) {
    audiofiletype = AudioFileType::kG711;
  } else {
    QMMF_INFO("%s:%s Usage: %s -c [INPUT_AAC_FILE_PATH] -f [aac/amr/g711]", TAG, __func__, argv[0]);
    assert(0);
    return -1;
  }

  params.codec_type = CodecMimeType::kMimeTypeAudioDecAAC;

  switch(audiofiletype) {
    case AudioFileType::kAAC:
  //Only for AAC
  aacfileIO_ = AACfileIO::createAACfileIOobj(params.input_file);
  if (aacfileIO_ == nullptr) {
    QMMF_ERROR("%s:%s AACfileIO creation failed",TAG,__func__);
    assert(0);
  }

  ret = aacfileIO_->Fillparams(&params);
  if (ret != 0) {
    QMMF_ERROR("%s:%s TestInitParams could not be filled",TAG,__func__);
    assert(0);
  }
  break;
  case AudioFileType::kAMR:
    //Only for AMR
  amrfileIO_ = AMRfileIO::createAMRfileIOobj(params.input_file);
  if (amrfileIO_ == nullptr) {
    QMMF_ERROR("%s:%s AMRfileIO creation failed",TAG,__func__);
    assert(0);
  }

  ret = amrfileIO_->Fillparams(&params);
  if (ret != 0) {
    QMMF_ERROR("%s:%s TestInitParams could not be filled",TAG,__func__);
    assert(0);
  }
   break;
  case AudioFileType::kG711:
      //Only for G711
  g711fileIO_ = G711fileIO::createG711fileIOobj(params.input_file);
  if (g711fileIO_ == nullptr) {
    QMMF_ERROR("%s:%s AMRfileIO creation failed",TAG,__func__);
    assert(0);
  }

  ret = g711fileIO_->Fillparams(&params);
  if (ret != 0) {
    QMMF_ERROR("%s:%s TestInitParams could not be filled",TAG,__func__);
    assert(0);
  }
  break;
default:
  QMMF_INFO("%s:%s Unknown AudioFileType", TAG, __func__);
  QMMF_INFO("%s:%s Usage: %s -c [INPUT_AAC_FILE_PATH] -f [aac/amr/g711]", TAG, __func__, argv[0]);
  return -1;
  }



  avcodec_ = IAVCodec::CreateAVCodec();
  if (avcodec_ ==  nullptr) {
    QMMF_ERROR("%s:%s avcodec creation failed", TAG, __func__);
    return NO_MEMORY;
  }

  ret = avcodec_->ConfigureCodec(params.codec_type, params.create_param);
  if (ret != OK) {
    QMMF_ERROR("%s:%s Failed to configure Codec", TAG, __func__);
    return ret;
  }

  ret = AllocateBuffer(kPortIndexInput);
  if (ret != OK) {
    QMMF_ERROR("%s:%s Failed to allocate buffer on PORT_NAME(%d)", TAG,
        __func__, kPortIndexInput);
    return ret;
  }

  input_source_impl_= make_shared<InputCodecSourceImpl>(params.input_file,
                                                        params.record_frame);
  if (input_source_impl_.get() == nullptr) {
    QMMF_ERROR("%s:%s failed to create input source", TAG, __func__);
    return NO_MEMORY;
  }

  ret = avcodec_->AllocateBuffer(kPortIndexInput, 0, 0,
                                 shared_ptr<ICodecSource>(input_source_impl_),
                                 input_buffer_list_);
  if (ret != OK) {
    QMMF_ERROR("%s:%s Failed to Call Allocate buffer on PORT_NAME(%d)",
               TAG, __func__, kPortIndexInput);
    ReleaseBuffer();
    return ret;
  }

  input_source_impl_->AddBufferList(input_buffer_list_);

  ret = AllocateBuffer(kPortIndexOutput);
  if (ret != OK) {
    QMMF_ERROR("%s:%s Failed to allocate buffer on PORT_NAME(%d)", TAG,
        __func__, kPortIndexOutput);
    ReleaseBuffer();
    return ret;
  }

  output_source_impl_ = make_shared<OutputCodecSourceImpl>(params.output_file);
  if (output_source_impl_.get() == nullptr) {
    QMMF_ERROR("%s:%s failed to create output source",TAG, __func__);
    return NO_MEMORY;
  }

  ret = avcodec_->AllocateBuffer(kPortIndexOutput, 0, 0,
                                 shared_ptr<ICodecSource>(output_source_impl_),
                                 output_buffer_list_);
  if (ret != OK) {
    QMMF_ERROR("%s:%s Failed to Call Allocate buffer on PORT_NAME(%d)",
               TAG, __func__, kPortIndexOutput);
    ReleaseBuffer();
    return ret;
  }

  output_source_impl_->AddBufferList(output_buffer_list_);

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t CodecTest::DeleteCodec() {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  status_t ret = 0;

  ret = avcodec_->ReleaseBuffer();
  assert(ret == OK);

  ret = ReleaseBuffer();
  assert(ret == OK);

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t CodecTest::StartCodec() {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);
  status_t ret = 0;

  {
    Mutex::Autolock l(stop_lock_);
    stop_ = false;
  }

  ret = avcodec_->StartCodec();
  assert(ret == OK);

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t CodecTest::StopCodec() {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);
  status_t ret = 0;

  {
    Mutex::Autolock l(stop_lock_);
    stop_ = true;
  }

  ret = avcodec_->StopCodec();
  assert(ret == OK);

  input_source_impl_->BufferStatus();
  output_source_impl_->BufferStatus();

  QMMF_INFO("%s:%s: Exit", TAG, __func__);
  return ret;
}

status_t CodecTest::ResumeCodec() {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);
  status_t ret = 0;

  ret = avcodec_->ResumeCodec();
  assert(ret == OK);

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t CodecTest::PauseCodec() {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);
  status_t ret = 0;

  ret = avcodec_->PauseCodec();
  assert(ret == OK);

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

status_t CodecTest::SetCodecParameters() {
  QMMF_INFO("%s:%s Enter ", TAG, __func__);
  status_t ret = 0;
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;
}

bool CodecTest::IsStop() {

   Mutex::Autolock l(stop_lock_);
   return stop_;
}

status_t CodecTest::AllocateBuffer(uint32_t index) {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  status_t ret = 0;

  assert(ion_device_ > 0);
  int32_t ionType =  ION_HEAP(ION_IOMMU_HEAP_ID);

  uint32_t count, size;
  ret = avcodec_->GetBufferRequirements(index,  &count, &size);
  if (ret != OK) {
    QMMF_INFO("%s:%s Failed to get Buffer Requirements on %s", TAG, __func__,
        PORT_NAME(index));
    return ret;
  }

  struct ion_allocation_data alloc;
  struct ion_fd_data ionFdData;
  void *vaddr = nullptr;

  for (uint32_t i = 0; i < count; i++) {
    if (index == kPortIndexInput) {
    BufferDescriptor buffer;
    vaddr = nullptr;

    memset(&buffer, 0x0, sizeof(buffer));
    memset(&alloc, 0x0, sizeof(ion_allocation_data));
    memset(&ionFdData, 0x0, sizeof(ion_fd_data));

    alloc.len = size;
    alloc.len = (alloc.len + 4095) & (~4095);
    alloc.align = 4096;
    alloc.flags = ION_FLAG_CACHED;
    alloc.heap_id_mask = ionType;

    ret = ioctl(ion_device_, ION_IOC_ALLOC, &alloc);
    if (ret < 0) {
      QMMF_ERROR("%s:%s ION allocation failed", TAG, __func__);
      goto ION_ALLOC_FAILED;
    }

    ionFdData.handle = alloc.handle;
    ret = ioctl(ion_device_, ION_IOC_SHARE, &ionFdData);
    if (ret < 0) {
      QMMF_ERROR("%s:%s ION map failed %s", TAG, __func__, strerror(errno));
      goto ION_MAP_FAILED;
    }

    vaddr = mmap(nullptr, alloc.len, PROT_READ  | PROT_WRITE, MAP_SHARED,
                ionFdData.fd, 0);
    if (vaddr == MAP_FAILED) {
      QMMF_ERROR("%s:%s  ION mmap failed: %s (%d)", TAG, __func__,
          strerror(errno), errno);
      goto ION_MAP_FAILED;
    }
    buffer.data     = vaddr;
    buffer.capacity = alloc.len;
    buffer.size     = 0;
    buffer.fd       = ionFdData.fd;
    input_ion_handle_data.push_back(alloc);
    QMMF_INFO("%s:%s buffer.Fd(%d)", TAG, __func__, buffer.fd );
    QMMF_INFO("%s:%s buffer.capacity(%d)", TAG,__func__, buffer.capacity);
    QMMF_INFO("%s:%s buffer.vaddr(%p)", TAG, __func__, buffer.data);
    input_buffer_list_.push_back(buffer);
    } else {
      BufferDescriptor buffer;
      memset(&buffer, 0x0, sizeof(buffer));
      memset(&alloc, 0x0, sizeof(ion_allocation_data));
      memset(&ionFdData, 0x0, sizeof(ion_fd_data));

      alloc.len = size;
      alloc.len = (alloc.len + 4095) & (~4095);
      alloc.align = 4096;
      alloc.flags = ION_FLAG_CACHED;
      alloc.heap_id_mask = ionType;

      ret = ioctl(ion_device_, ION_IOC_ALLOC, &alloc);
      if (ret < 0) {
        QMMF_ERROR("%s:%s ION allocation failed", TAG, __func__);
        goto ION_ALLOC_FAILED;
      }

      ionFdData.handle = alloc.handle;
      ret = ioctl(ion_device_, ION_IOC_SHARE, &ionFdData);
      if (ret < 0) {
        QMMF_ERROR("%s:%s ION map failed %s", TAG, __func__, strerror(errno));
        goto ION_MAP_FAILED;
      }

      vaddr = mmap(nullptr, alloc.len, PROT_READ  | PROT_WRITE, MAP_SHARED,
                  ionFdData.fd, 0);
      if (vaddr == MAP_FAILED) {
        QMMF_ERROR("%s:%s  ION mmap failed: %s (%d)", TAG, __func__,
            strerror(errno), errno);
        goto ION_MAP_FAILED;
      }

      buffer.data     = vaddr;
      buffer.capacity = alloc.len;
      buffer.size     = 0;
      buffer.fd       = ionFdData.fd;
      output_ion_handle_data.push_back(alloc);
      QMMF_INFO("%s:%s buffer.Fd(%d)", TAG, __func__, buffer.fd );
      QMMF_INFO("%s:%s buffer.capacity(%d)", TAG,__func__, buffer.capacity);
      QMMF_INFO("%s:%s buffer.vaddr(%p)", TAG, __func__, buffer.data);
      output_buffer_list_.push_back(buffer);
    }
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return ret;

  ION_MAP_FAILED:
    struct ion_handle_data ionHandleData;
    memset(&ionHandleData, 0x0, sizeof(ionHandleData));
    ionHandleData.handle = ionFdData.handle;
    ioctl(ion_device_, ION_IOC_FREE, &ionHandleData);
  ION_ALLOC_FAILED:
    close(ion_device_);
    ion_device_ = -1;
    QMMF_ERROR("%s:%s ION Buffer allocation failed!", TAG, __func__);
    QMMF_INFO("%s:%s Exit", TAG, __func__);
    return -1;
}

status_t CodecTest::ReleaseBuffer() {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);

  int i = 0;
  for (auto& iter : input_buffer_list_) {
      if ((iter).data) {
          munmap((iter).data, (iter).capacity);
          (iter).data = nullptr;
      }
      if ((iter).fd) {
          ioctl(ion_device_, ION_IOC_FREE, &(input_ion_handle_data[i]));
          close((iter).fd);
          (iter).fd = -1;
      }
      i++;
  }

  i = 0;
  for (auto& iter : output_buffer_list_) {
      if ((iter).data) {
          munmap((iter).data, (iter).capacity);
          (iter).data = nullptr;
      }
      if ((iter).fd) {
          ioctl(ion_device_, ION_IOC_FREE, &(output_ion_handle_data[i]));
          close((iter).fd);
          (iter).fd = -1;
      }
      ++i;
  }

  input_buffer_list_.clear();
  output_buffer_list_.clear();
  input_ion_handle_data.clear();
  output_ion_handle_data.clear();

  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return 0;
}

InputCodecSourceImpl::InputCodecSourceImpl(char* file_name,
                                           uint32_t num_frame = 0) {

  QMMF_INFO("%s:%s  Enter",TAG, __func__);
switch(audiofiletype) {
  case AudioFileType::kAAC:
    //Only for AAC
  aacfileIO_  = AACfileIO::createAACfileIOobj(file_name);
  if (aacfileIO_ == nullptr) {
    QMMF_ERROR("%s:%s failed to create ACfileIO object",TAG,__func__);
    assert(0);
  }
  break;
  case AudioFileType::kAMR:
    //Only for AMR
  amrfileIO_  = AMRfileIO::createAMRfileIOobj(file_name);
  if (amrfileIO_ == nullptr) {
    QMMF_ERROR("%s:%s failed to create AMRfileIO object",TAG,__func__);
    assert(0);
  }
  break;
  case AudioFileType::kG711:
      //Only for G711
  g711fileIO_  = G711fileIO::createG711fileIOobj(file_name);
  if (g711fileIO_ == nullptr) {
    QMMF_ERROR("%s:%s failed to create AMRfileIO object",TAG,__func__);
    assert(0);
  }
  break;
  default:
  QMMF_INFO("%s:%s Unknown AudioFileType", TAG, __func__);
  QMMF_INFO("%s:%s Usage: test_app -c [INPUT_AAC_FILE_PATH] -f [aac/amr/g711]", TAG, __func__);
  assert(0);
  }

  num_frame_read = num_frame;

  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

InputCodecSourceImpl::~InputCodecSourceImpl() {

  QMMF_INFO("%s:%s  Enter", TAG, __func__);
  QMMF_INFO("%s:%s  Exit", TAG, __func__);
}

void InputCodecSourceImpl::AddBufferList(vector<BufferDescriptor>& list) {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);

  input_list_ = list;
  input_free_buffer_queue_.Clear();
  input_occupy_buffer_queue_.Clear();
  for (auto& iter : input_list_) {
    input_free_buffer_queue_.PushBack(iter);
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

status_t InputCodecSourceImpl::NotifyPortEvent(PortEventType event_type,
                                               void* event_data)  {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return 0;
}

status_t InputCodecSourceImpl::GetBuffer(BufferDescriptor& stream_buffer,
                                         void* client_data) {

  status_t ret = 0;

  if (input_free_buffer_queue_.Size() <= 0) {
    QMMF_WARN("%s:%s No buffer available. Wait for new buffer", TAG, __func__);
    Mutex::Autolock autoLock(wait_for_frame_lock_);
    wait_for_frame_.wait(wait_for_frame_lock_);
  }

  BufferDescriptor buffer = *input_free_buffer_queue_.Begin();
  assert(buffer.data != nullptr);

  uint32_t bytes_read = 0;
  int32_t num_frames_read;
switch(audiofiletype) {

  case AudioFileType::kAAC:
    //Only for AAC
  if (aacfileIO_->isfileopen()) {
    ret = aacfileIO_->GetFrames(buffer.data,buffer.capacity,&num_frames_read,&bytes_read);
  } else {
    QMMF_ERROR("%s:%s input file is not opened", TAG, __func__);
    return -1;
  }
  break;
  case AudioFileType::kAMR:
    //Only for AMR
  if (amrfileIO_->isfileopen()) {
    ret = amrfileIO_->GetFrames(buffer.data,buffer.capacity,&num_frames_read,&bytes_read);
  } else {
    QMMF_ERROR("%s:%s input file is not opened", TAG, __func__);
    return -1;
  }
  break;
  case AudioFileType::kG711:
      //Only for G711
  if (g711fileIO_->isfileopen()) {
    ret = g711fileIO_->GetFrames(buffer.data,buffer.capacity,&bytes_read);
  } else {
    QMMF_ERROR("%s:%s input file is not opened", TAG, __func__);
    return -1;
  }
  break;
  default:
  QMMF_INFO("%s:%s Unknown AudioFileType", TAG, __func__);
  QMMF_INFO("%s:%s Usage: test_app -c [INPUT_AAC_FILE_PATH] -f [aac/amr/g711]", TAG, __func__);
  assert(0);
}


  if (ret != OK)
    QMMF_INFO("%s:%s Read completed.", TAG, __func__);

  buffer.size = bytes_read;
  stream_buffer.data = buffer.data;
  stream_buffer.size = buffer.size;
  stream_buffer.capacity = buffer.capacity;
  stream_buffer.fd = buffer.fd;

  input_occupy_buffer_queue_.PushBack(buffer);
  input_free_buffer_queue_.Erase(input_free_buffer_queue_.Begin());
switch(audiofiletype) {
  case AudioFileType::kAAC:
    //Only for AAC
  stream_buffer.timestamp = aacfileIO_->currentTimeus;
  aacfileIO_->currentTimeus = aacfileIO_->currentTimeus + num_frames_read*(aacfileIO_->Framedurationus);
  break;
  case AudioFileType::kAMR:
   //Only for AMR
  stream_buffer.timestamp = amrfileIO_->currentTimeus;
  amrfileIO_->currentTimeus = amrfileIO_->currentTimeus + num_frames_read*(amrfileIO_->Framedurationus);
  break;
  case AudioFileType::kG711:
   //Only for G711
  stream_buffer.timestamp = 0;
  break;
  default:
  QMMF_INFO("%s:%s Unknown AudioFileType", TAG, __func__);
  QMMF_INFO("%s:%s Usage: -c [INPUT_AAC_FILE_PATH] -f [aac/amr/g711]", TAG, __func__);
  assert(0);
}


  return ret;
}

status_t InputCodecSourceImpl::ReturnBuffer(BufferDescriptor& buffer,
                                            void* client_data) {

  status_t ret = 0;
  bool found = false;

  List<BufferDescriptor>::iterator it = input_occupy_buffer_queue_.Begin();
  for (; it != input_occupy_buffer_queue_.End(); ++it) {
    if ((*it).data ==  buffer.data) {
      input_free_buffer_queue_.PushBack(*it);
      wait_for_frame_.signal();
      found = true;
      break;
    }
  }
  assert(found == true);
  input_occupy_buffer_queue_.Erase(it);

  return ret;
}

void InputCodecSourceImpl::BufferStatus() {

  QMMF_INFO("%s:%s Total Buffer(%d), free(%d), occupy(%d)", TAG, __func__,
      input_list_.size(), input_free_buffer_queue_.Size(),
      input_occupy_buffer_queue_.Size());
  assert(input_occupy_buffer_queue_.Size() == 0);
}

OutputCodecSourceImpl::OutputCodecSourceImpl(char* file_name) {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);

  file_fd_ = open(file_name, O_CREAT | O_WRONLY | O_TRUNC, 0655);
  if (file_fd_ < 0) {
    QMMF_ERROR("%s:%s Failed to open o/p file(%s)", TAG, __func__, file_name);
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

OutputCodecSourceImpl::~OutputCodecSourceImpl() {

  QMMF_INFO("%s:%s Enter", TAG, __func__);

  if (file_fd_ > 0) {
    close(file_fd_);
    file_fd_ = -1;
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

void OutputCodecSourceImpl::AddBufferList(vector<BufferDescriptor>& list) {

  QMMF_INFO("%s:%s Enter ", TAG, __func__);

  output_list_ = list;
  output_free_buffer_queue_.Clear();
  output_occupy_buffer_queue_.Clear();

  for (auto& iter : output_list_) {
    output_free_buffer_queue_.PushBack(iter);
  }

  QMMF_INFO("%s:%s Exit", TAG, __func__);
}

status_t OutputCodecSourceImpl::NotifyPortEvent(PortEventType event_type,
                                                void* event_data)  {

  QMMF_INFO("%s:%s Enter", TAG, __func__);
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return 0;
}

status_t OutputCodecSourceImpl::GetBuffer(BufferDescriptor& codec_buffer,
                                          void* client_data) {

  status_t ret = 0;

  if (output_free_buffer_queue_.Size() <= 0) {
    QMMF_WARN("%s:%s No buffer available to notify. Wait for new buffer", TAG,
        __func__);
    Mutex::Autolock autoLock(wait_for_frame_lock_);
    wait_for_frame_.wait(wait_for_frame_lock_);
  }

  BufferDescriptor iter = *output_free_buffer_queue_.Begin();
  codec_buffer.fd = (iter).fd;
  codec_buffer.data = (iter).data;
  output_occupy_buffer_queue_.PushBack(iter);
  output_free_buffer_queue_.Erase(output_free_buffer_queue_.Begin());

  return ret;
}

status_t OutputCodecSourceImpl::ReturnBuffer(BufferDescriptor& codec_buffer,
                                             void* client_data) {

  status_t ret = 0;

  assert(codec_buffer.data != nullptr);

  if (file_fd_ > 0) {
    ssize_t expSize = (ssize_t) codec_buffer.size;
    QMMF_INFO("FillBufferDone size writen to file  %zd", expSize);
    if (expSize != write(file_fd_,
          reinterpret_cast<uint8_t*>(codec_buffer.data) + codec_buffer.offset,
          codec_buffer.size)) {
        QMMF_ERROR("%s:%s Bad Write error (%d) %s", TAG, __func__,
            errno, strerror(errno));
        close(file_fd_);
        file_fd_ = -1;
    }
  } else {
    QMMF_ERROR("%s:%s File is not open to write", TAG, __func__);
  }

  if (codec_buffer.flag & static_cast<uint32_t>(BufferFlags::kFlagEOS)) {
    QMMF_INFO("%s:%s This is last buffer from encoder.Close file", TAG,__func__);
    if (file_fd_ > 0) {
      close(file_fd_);
      file_fd_ = -1;
    }
  }

  List<BufferDescriptor>::iterator it = output_occupy_buffer_queue_.Begin();
  bool found = false;
  for (; it != output_occupy_buffer_queue_.End(); ++it) {
    if (((*it).data) ==  (codec_buffer.data)) {
      output_free_buffer_queue_.PushBack(*it);
      output_occupy_buffer_queue_.Erase(it);
      wait_for_frame_.signal();
      found = true;
      break;
    }
  }

  assert(found == true);
  return ret;
}

void OutputCodecSourceImpl::BufferStatus() {

  QMMF_INFO("%s:%s Total Buffer(%d), free(%d), occupy(%d)", TAG, __func__,
      output_list_.size(), output_free_buffer_queue_.Size(),
      output_occupy_buffer_queue_.Size());
  assert(output_occupy_buffer_queue_.Size() == 0);
}

void CmdMenu::PrintDynamicParams() {
  DefaultKeyedVector<String8, uint32_t> param = ctx_.GetDynamicParam();
  if (!param.isEmpty()) {
    for (size_t i = 0; i < param.size(); i++) {
      printf("   %c. Set Param:(%s : %d)\n", CmdMenu::SET_CODEC_PARAM_CMD,
          param.keyAt(i).string(), param.valueAt(i));
    }
  }
}

void CmdMenu::PrintMenu() {

  printf("\n\n=========== QIPCAM TEST MENU ===================\n\n");

  printf(" \n\nCodec Test Application commands \n");
  printf(" -----------------------------\n");
  printf("   %c. Create Codec\n", CmdMenu::CREATE_CODEC_CMD);
  printf("   %c. Delete Codec\n", CmdMenu::DELETE_CODEC_CMD);
  printf("   %c. Start Codec\n", CmdMenu::START_CODEC_CMD);
  printf("   %c. Stop Codec\n", CmdMenu::STOP_CODEC_CMD);
  printf("   %c. Pause Codec\n", CmdMenu::PAUSE_CODEC_CMD);
  printf("   %c. Resume Codec\n", CmdMenu::RESUME_CODEC_CMD);
  PrintDynamicParams();
  printf("   %c. Exit\n", CmdMenu::EXIT_CMD);
  printf("\n   Choice: ");
}

CmdMenu::Command CmdMenu::GetCommand() {

  PrintMenu();
  return CmdMenu::Command(static_cast<CmdMenu::CommandType>(getchar()));
}

int main(int argc,char *argv[]) {

  QMMF_INFO("%s:%s Enter", TAG, __func__);

  CodecTest test_context;

  CmdMenu cmd_menu(test_context);

  int32_t testRunning = true;

  while (testRunning) {
    CmdMenu::Command command = cmd_menu.GetCommand();

    switch (command.cmd) {
      case CmdMenu::CREATE_CODEC_CMD:
      {
        test_context.CreateCodec(argc, argv);
      }
      break;
      case CmdMenu::DELETE_CODEC_CMD:
      {
        test_context.DeleteCodec();
      }
      break;
      case CmdMenu::START_CODEC_CMD:
      {
        test_context.StartCodec();
      }
      break;
      case CmdMenu::STOP_CODEC_CMD:
      {
        test_context.StopCodec();
      }
      break;
      case CmdMenu::PAUSE_CODEC_CMD:
      {
        test_context.PauseCodec();
      }
      break;
      case CmdMenu::RESUME_CODEC_CMD:
      {
        test_context.ResumeCodec();
      }
      break;
      case CmdMenu::SET_CODEC_PARAM_CMD:
      {
        test_context.SetCodecParameters();
      }
      break;
       case CmdMenu::EXIT_CMD:
      {
        QMMF_INFO("%s:%s exit from test", TAG, __func__);
        testRunning = false;
      }
      break;
      default:
      break;
    }
  }
  QMMF_INFO("%s:%s Exit", TAG, __func__);
  return 0;
}
