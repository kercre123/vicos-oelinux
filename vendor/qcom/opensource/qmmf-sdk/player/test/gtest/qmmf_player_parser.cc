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

//      TEST_ERROR("%s:%s:%s",TAG,TAG2,__func__);


#define TAG "Player_Parser"
#define TAG2 "AACfileIO"
#define TAG3 "AMRfileIO"
#define TAG4 "G711fileIO"
#define OFFSET_TABLE_LEN    300
#define MAX_NUM_FRAMES_PER_BUFF_AMR  1
#define FORMAT_ALAW  0x0006
#define FORMAT_MULAW 0x0007

#include "qmmf_player_parser.h"

//#define DEBUG
#define TEST_INFO(fmt, args...)  ALOGD(fmt, ##args)
#define TEST_ERROR(fmt, args...) ALOGE(fmt, ##args)
#ifdef DEBUG
#define TEST_DBG  TEST_INFO
#else
#define TEST_DBG(...) ((void)0)
#endif

AACfileIO::AACfileIO(const char*file):currentTimeus(0),
                                Framedurationus(0),
                                infile(file),
                                confidence(0),
                                streamSize(0),
                                numFrames(0),
                                sf_index(0),
                                profile(0),
                                channel(0),
                                sr(0),
                                duration(0),
                                read_completed(false){
  TEST_INFO("%s:%s:%s Enter",TAG,TAG2,__func__);
  TEST_INFO("%s:%s:%s Exit",TAG,TAG2,__func__);
}

uint32_t AACfileIO::get_sample_rate(const uint8_t index){
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

size_t AACfileIO::getAdtsFrameLength(uint64_t offset,size_t*headersize){
    const size_t kAdtsHeaderLengthNoCrc = 7;
    const size_t kAdtsHeaderLengthWithCrc = 9;

    size_t framesize = 0;

    char syncword[2];
    infile.seekg(offset);
    if(infile.read(syncword,2).gcount() != 2){
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

    if(infile.read(header,3).gcount() != 3){
        return 0;
    }

    infile.seekg(offset);

    framesize = ((size_t)header[0] & 0x3) << 11 | (size_t)header[1] << 3 | (size_t)header[2] >> 5;

    // protectionAbsent is 0 if there is CRC
    size_t headsize = protectionAbsent ? kAdtsHeaderLengthNoCrc : kAdtsHeaderLengthWithCrc;
    if (headsize > framesize) {
        return 0;
    }
    if (headersize != nullptr){
        *headersize = headsize;
    }

    return framesize;
}
status_t AACfileIO::Fillparams(AudioTrackCreateParam *params){
  TEST_INFO("%s:%s:%s   Enter",TAG,TAG2,__func__);
  size_t pos = 0;
  uint64_t offset = 0;

    while(1){
    char id3header[10];

    if(infile.read(id3header,sizeof(id3header)).gcount() < (ssize_t)sizeof(id3header)){
      TEST_ERROR("%s:%s:%s   Error in reading id3header",TAG,TAG2,__func__);
      return -1;
    }

    if(memcmp("ID3",id3header,3)){
      infile.seekg(pos);
      break;
    }

    // Skip the ID3v2 header.

    size_t len =  (((size_t)id3header[6] & 0x7f) << 21) | (((size_t)id3header[7] & 0x7f) << 14) | (((size_t)id3header[8] & 0x7f) << 7) | ((size_t)id3header[9] & 0x7f);
    len += 10;
    pos += len;
    infile.seekg(pos);
  }

  char header[2];

  if (infile.read(header,2).gcount() != 2) {
    TEST_ERROR("%s:%s:%s Error in reading header[2] / syncwords",TAG,TAG2,__func__);
    return -1;
  }

  infile.seekg(pos);

  if(((uint8_t)header[0] == 0xff) && (((uint8_t)header[1] & 0xf6) == 0xf0)){
    TEST_DBG("%s:%s:%s ADTS header found",TAG,TAG2,__func__);
    confidence = 0.2;
    offset = pos;
  }
  else{
    TEST_ERROR("%s:%s:%s ADTS header not found",TAG,TAG2,__func__);
    return -1;
  }

  infile.seekg(offset+2);
  if(infile.read(header,2).gcount() < 2){
        TEST_ERROR("%s:%s:%s Error in reading header[2]  at offset+2 ",TAG,TAG2,__func__);
    return -1;
  }
  infile.seekg(offset);
  //starting 2 bits of header[0] is profile
  profile = ((uint8_t)header[0] >> 6) & 0x3;
  //next 4 bits of header[0] is sf_index
  sf_index = ((uint8_t)header[0] >> 2) & 0xf;
  sr = get_sample_rate(sf_index);
  if(sr == 0){
        TEST_ERROR("%s:%s:%s Sampling rate could not be found",TAG,TAG2,__func__);
    return -1;
  }

  //then ignore the next bit of hedaer[0]
  //and last bit of header[0] and first 2 bits of hedaer[1] represents the number of channels
  channel = ((uint8_t)header[0] & 0x1) << 2 | ((uint8_t)header[1] >> 6);

  infile.seekg(0,infile.end);
  streamSize = infile.tellg();
  infile.seekg(offset);
  size_t framesize;
  size_t headersize;
  TEST_DBG("Size of size_t = %d",sizeof(size_t));
  while(offset < streamSize){
      if((framesize = getAdtsFrameLength(offset,&headersize)) == 0){
          TEST_ERROR("%s:%s:%s Error from AACfileIO::getAdtsFrameLength function",TAG,TAG2,__func__);
          return -1;
      }
      TEST_DBG("%s:%s:%s Current Offset for the ADTS header %d is %lld with framesize = %u and headersize = %u" ,TAG,TAG2,__func__,numFrames + 1, (long long)offset, (uint32_t)framesize,(uint32_t)headersize);
      OffsetVector.push_back(offset);
      frameSize.push_back(framesize);
      headerSize.push_back(headersize);
      offset += (uint64_t)framesize;
      numFrames++;
  }

  v_OffsetVector = OffsetVector.begin();
  v_frameSize = frameSize.begin();
  v_headerSize = headerSize.begin();

  TEST_INFO("%s:%s:%s The aac file read completed",TAG,TAG2,__func__);

  //In DSP this 1024(number of samples per frame) is the default/hardcoded value look at omx_aac_adec.h and omx_aac_dec.cpp for reference
  Framedurationus = (1024 * 1000000ll + (sr - 1)) / sr;
  duration = numFrames * Framedurationus;

  params->sample_rate = sr;
  params->channels    = channel;
  params->bit_depth   = 16;
  params->codec       = (AudioCodecType)AudioFormat::kAAC;
  params->codec_params.aac.bit_rate = 55000;
  params->codec_params.aac.format = AACFormat::kADTS;

  TEST_INFO("Channel = %d, sampling rate = %d, profile = %d",channel,sr,profile);
  switch(profile){
    case 0:
      params->codec_params.aac.mode = AACMode::kAALC;
    break;
    case 1:
      params->codec_params.aac.mode = AACMode::kHEVC_v1;
    break;
    case 2:
      params->codec_params.aac.mode = AACMode::kHEVC_v2;
    break;
    default:
     TEST_ERROR("%s:%s:%s unsupported AAC mode: %d", TAG, TAG2,__func__,profile);
  }

  TEST_INFO("%s:%s:%s   Exit",TAG,TAG2,__func__);
  return 0;
}

//return value of -1 signifies the end of file i.e. OMX_BUFFERFLAGEOS
status_t AACfileIO::GetFrames(void*buffer,uint32_t size_buffer,int32_t*num_frames_read,uint32_t*bytes_read){

  TEST_INFO("%s:%s:%s Enter",TAG,TAG2,__func__);
  if(read_completed){
        TEST_ERROR("%s:%s:%s File read has already been completed",TAG,TAG2,__func__);
    return -1;
  }
  char* aac = (char*)buffer;

  *num_frames_read = 0;
  *bytes_read = 0;

  while(*bytes_read < size_buffer){
    uint64_t offset =  *v_OffsetVector;
    size_t framesize = *v_frameSize;
    TEST_DBG("%s:%s:%s offset = %lld frameSize = %u headerSize = %u",TAG,TAG2,__func__,(long long)offset,(uint32_t)framesize,(uint32_t)headersize);
    if(size_buffer - *bytes_read < (uint32_t)framesize){

        TEST_INFO("%s:%s:%s No space left in Buffer header(%p)",TAG,TAG2,__func__,buffer);
        return 0;
    }
    uint32_t read_bytes = 0;
    infile.seekg(offset);
    if((read_bytes  = infile.read(aac,framesize).gcount()) != framesize){
        TEST_ERROR("%s:%s:%s Error in reading the %d bytes from aac file, read_bytes = %d",TAG,TAG2,__func__,framesize,read_bytes);
        assert(0);
    }
    aac += read_bytes;
    *bytes_read += read_bytes;
    *num_frames_read += 1;
    v_OffsetVector++;
    v_frameSize++;
    v_headerSize++;
    if((v_OffsetVector == OffsetVector.end()) && (v_frameSize == frameSize.end()) && (v_headerSize == headerSize.end())){
      read_completed = true;
      TEST_INFO("%s:%s:%s The input file has been read completely. Now EOS has to be send",TAG,TAG2,__func__);
      return -1;
    }
  }

  TEST_INFO("%s:%s:%s Exit",TAG,TAG2,__func__);
  return 0;
}

AACfileIO::~AACfileIO(){
  TEST_INFO("%s:%s:%s Enter",TAG,TAG2,__func__);
  if(infile.is_open()){
    infile.close();
  }
  OffsetVector.clear();
  frameSize.clear();
  headerSize.clear();
  TEST_INFO("%s:%s:%s Exit",TAG,TAG2,__func__);
}

G711fileIO::G711fileIO(const char*file):currentTimeus(0),
                                Framedurationus(0),
                                infile(file),
                                streamSize(0),
                                channel(0),
                                sr(0),
                                read_completed(false),
                                isAlaw(false),
                                isMulaw(false){
  TEST_INFO("%s:%s:%s Enter",TAG,TAG4,__func__);
  TEST_INFO("%s:%s:%s Exit",TAG,TAG4,__func__);
}
G711fileIO::~G711fileIO(){
  TEST_INFO("%s:%s:%s Enter",TAG,TAG4,__func__);
  if(infile.is_open()){
    infile.close();
  }
  TEST_INFO("%s:%s:%s Exit",TAG,TAG4,__func__);
}

status_t G711fileIO::Fillparams(AudioTrackCreateParam *params){
  TEST_ERROR("%s:%s:%s Enter",TAG,TAG4,__func__);
  struct g711_header g711hdr;
  uint32_t read_bytes;
  if((read_bytes =  infile.read((char*)&g711hdr,sizeof(g711hdr)).gcount()) != (ssize_t)(sizeof(g711hdr))){
    TEST_ERROR("%s:%s:%s Error in reading the %d bytes from aac file, read_bytes = %d",TAG,TAG3,__func__,sizeof(g711hdr),read_bytes);
    return -1;
  }
    if ((g711hdr.audio_format != FORMAT_MULAW) && (g711hdr.audio_format != FORMAT_ALAW))
  {
      TEST_INFO("%s:%s:%s g711 file is not MULAW or ALAW format it's format is %d ",TAG,TAG4,__func__,g711hdr.audio_format);
      return -1;
  }

  if ((g711hdr.sample_rate != 8000) && (g711hdr.sample_rate != 16000)) {
        TEST_ERROR("%s:%s:%s samplerate = %d, not supported, Supported samplerates are 8000, 16000",TAG,TAG4,__func__,g711hdr.sample_rate);
      return -1;
  }

  if (g711hdr.num_channels != 1) {
        TEST_ERROR("%s:%s:%s stereo and multi channel are not supported, channels %d",TAG,TAG4,__func__,g711hdr.num_channels);
      return -1;
  }
  sr = g711hdr.sample_rate;
  channel = g711hdr.num_channels;
  starting_offset =  infile.tellg();

  offset = starting_offset;

  infile.seekg(0,infile.end);
  streamSize = infile.tellg();
  infile.seekg(starting_offset);

  params->sample_rate = sr;
  params->channels    = channel;
  params->bit_depth   = 16;
  params->codec       = (AudioCodecType)AudioFormat::kG711;
  TEST_INFO(" %s:%s:%s Channel = %d, sampling rate = %d",TAG,TAG4,__func__,channel,sr);

  if(g711hdr.audio_format == FORMAT_MULAW){
    params->codec_params.g711.mode = G711Mode::kMuLaw;
    isMulaw = true;
    TEST_INFO("%s:%s:%s Format is MULAW",TAG,TAG4,__func__);
  }
  else{
    params->codec_params.g711.mode = G711Mode::kALaw;
    isAlaw = true;
    TEST_INFO("%s:%s:%s Format is ALAW",TAG,TAG4,__func__);
  }

  return 0;
}

status_t G711fileIO::GetFrames(void*buffer,uint32_t size_buffer,uint32_t* bytes_read){
  TEST_INFO("%s:%s:%s Enter",TAG,TAG4,__func__);
  if(read_completed){
        TEST_ERROR("%s:%s:%s File read has already been completed",TAG,TAG4,__func__);
           TEST_INFO("%s:%s:%s Exit",TAG,TAG4,__func__);
    return -1;
  }
  size_buffer = 1024;

  infile.seekg(offset);
  char*g711 = (char*)buffer;
  uint32_t bytes_to_read = (streamSize - offset) > size_buffer ? size_buffer : (streamSize - offset);
  uint32_t read_bytes;
  if((read_bytes = infile.read(g711,bytes_to_read).gcount()) != bytes_to_read){
    TEST_ERROR("%s:%s:%s Could not read %d bytes requested, bytes read = %d",TAG,TAG4,__func__,bytes_to_read,read_bytes);
    assert(0);
  }
  *bytes_read = read_bytes;
  if(bytes_to_read < size_buffer){
    read_completed = true;
    offset += (uint64_t)(*bytes_read);
    TEST_INFO("%s:%s:%s The input file has been read completely. Now EOS has to be send",TAG,TAG4,__func__);
       TEST_INFO("%s:%s:%s Exit",TAG,TAG4,__func__);
    return -1;
  }
  offset += (uint64_t)(*bytes_read);
   TEST_INFO("%s:%s:%s Exit",TAG,TAG4,__func__);
  return 0;
}

AMRfileIO::AMRfileIO(const char*file):currentTimeus(0),
                                Framedurationus(20000),
                                starting_offset(0),
                                infile(file),
                                confidence(0),
                                streamSize(0),
                                numFrames(0),
                                channel(0),
                                sr(0),
                                duration(0),
                                read_completed(false),
                                mIsWide(false){
  TEST_INFO("%s:%s:%s Enter",TAG,TAG3,__func__);
  TEST_INFO("%s:%s:%s Exit",TAG,TAG3,__func__);
}

AMRfileIO::~AMRfileIO(){
  TEST_INFO("%s:%s:%s Enter",TAG,TAG3,__func__);
  if(infile.is_open()){
    infile.close();
  }
  OffsetVector.clear();
  frameSize.clear();
  TEST_INFO("%s:%s:%s Exit",TAG,TAG3,__func__);
}

size_t AMRfileIO::getFrameSize(bool isWide,unsigned int FT){
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
       TEST_ERROR("%s:%s:%s illegal AMR frame type %d", TAG,TAG3,__func__,FT);
        return 0;
   }

    size_t framesize = isWide ? kFrameSizeWB[FT] : kFrameSizeNB[FT];

    // Round up bits to bytes and add 1 for the header byte.
    framesize = (framesize + 7) / 8 + 1;

    return framesize;
}

status_t AMRfileIO::getFrameSizeByOffset(uint64_t offset, bool isWide, size_t *framesize){
  infile.seekg(offset);
  char header[1];
  if(infile.read(header,1).gcount() != 1){
    TEST_ERROR("%s:%s:%s Could not read the header of AMR at offset = %lld",TAG,TAG3,__func__,(long long)offset);
    assert(0);
    return 0;
  }
  unsigned int FT = ((uint8_t)header[0] >> 3) & 0x0f;
  *framesize = getFrameSize(isWide, FT);
  if(*framesize == 0){
    TEST_ERROR("%s:%s:%s AMR framesize is 0",TAG,TAG3,__func__);
    return 0;
  }
  return 1;
}

status_t AMRfileIO::Fillparams(AudioTrackCreateParam *params){
  TEST_INFO("%s:%s:%s   Enter",TAG,TAG3,__func__);
  char header[9];
  if(infile.read(header,sizeof(header)).gcount() != (ssize_t)(sizeof(header))){
    TEST_ERROR("%s:%s:%s Could not get AMR MIME TYPE",TAG,TAG3,__func__);
    return -1;
  }
  if(!memcmp(header, "#!AMR\n", 6)){
    mIsWide = false;
    confidence = 0.5;
    TEST_INFO("%s:%s:%s AMR MIME TYPE is not Wide",TAG,TAG3,__func__);
  }
  else if(!memcmp(header, "#!AMR-WB\n", 9)){
    mIsWide = true;
    confidence = 0.5;
    TEST_INFO("%s:%s:%s AMR MIME TYPE is Wide",TAG,TAG3,__func__);
  }
  else{
    TEST_ERROR("%s:%s:%s Could not get AMR MIME TYPE",TAG,TAG3,__func__);
    return -1;
  }
  starting_offset = mIsWide ? 9 : 6;
  uint64_t offset = starting_offset;
  infile.seekg(0,infile.end);
  streamSize = infile.tellg();
  infile.seekg(offset);
  size_t framesize;
  while(offset < streamSize){
      if (getFrameSizeByOffset(offset, mIsWide, &framesize) == 0){
        TEST_ERROR("%s:%s:%s Could not get frame size by offset",TAG,TAG3,__func__);
        return -1;
      }
      OffsetVector.push_back(offset);
      frameSize.push_back(framesize);
      offset += (uint64_t)framesize;
      duration += Framedurationus;
      numFrames++;
  }

  v_OffsetVector  = OffsetVector.begin();
  v_frameSize = frameSize.begin();

  sr = 16000;
  channel = 1;

  params->sample_rate               = sr;
  params->channels                  = channel;
  params->bit_depth                 = 16;
  params->codec                     = (AudioCodecType)AudioFormat::kAMR;
  params->codec_params.amr.isWAMR   = mIsWide;

  if(mIsWide){
      TEST_INFO("Channel = %d, sampling rate = %d AMR is Wide",channel,sr);
  }
  else{
      TEST_INFO("Channel = %d, sampling rate = %d AMR is not Wide",channel,sr);
  }

  TEST_INFO("%s:%s:%s   Enter",TAG,TAG3,__func__);
  return 0;
}

status_t AMRfileIO::GetFrames(void*buffer,uint32_t size_buffer,int32_t* num_frames_read,uint32_t* bytes_read){

  TEST_INFO("%s:%s:%s Enter",TAG,TAG3,__func__);
  if(read_completed){
        TEST_ERROR("%s:%s:%s File read has already been completed",TAG,TAG3,__func__);
    return -1;
  }
  char*amr = (char*)buffer;

  *num_frames_read = 0;
  *bytes_read = 0;

  while(*bytes_read < size_buffer){
    uint64_t offset =  *v_OffsetVector;
    size_t framesize = *v_frameSize;
    TEST_DBG("%s:%s:%s offset = %lld frameSize = %u",TAG,TAG3,__func__,(long long)offset,(uint32_t)framesize);
    if(size_buffer - *bytes_read < (uint32_t)framesize){

        TEST_DBG("%s:%s:%s No space left in Buffer header(%p)",TAG,TAG3,__func__,buffer);
        TEST_DBG("%s:%s:%s Exit",TAG,TAG3,__func__);
        return 0;
    }
    if(*num_frames_read == MAX_NUM_FRAMES_PER_BUFF_AMR){
        TEST_INFO("%s:%s:%s MAX_NUM_FRAMES_PER_BUFF_AMR in Buffer header(%p)",TAG,TAG3,__func__,buffer);
        TEST_DBG("%s:%s:%s Exit",TAG,TAG3,__func__);
        return 0;
    }
    uint32_t read_bytes = 0;
    infile.seekg(offset);
    if((read_bytes  = infile.read(amr,framesize).gcount()) != framesize){
        TEST_ERROR("%s:%s:%s Error in reading the %d bytes from amr file, read_bytes = %d",TAG,TAG3,__func__,framesize,read_bytes);
        assert(0);
    }
    amr += read_bytes;
    *bytes_read += read_bytes;
    *num_frames_read += 1;
    v_OffsetVector++;
    v_frameSize++;
    if((v_OffsetVector == OffsetVector.end()) && (v_frameSize == frameSize.end())){
      read_completed = true;
      TEST_INFO("%s:%s:%s The input file has been read completely. Now EOS has to be send",TAG,TAG3,__func__);
      return -1;
    }
  }

  TEST_INFO("%s:%s:%s Exit",TAG,TAG3,__func__);
  return 0;
}
