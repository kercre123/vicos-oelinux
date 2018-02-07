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

/**============================================================================
 *
 * INCLUDE FILES FOR MODULE
 *
 *=========================================================================== */
#include "AEEStdDef.h"
#include "MMMemory.h"
#include "MMDebugMsg.h"
#include <stdio.h>
#include <string.h>


/* ============================================================================
 **                          Macro Definitions
 * ========================================================================== */
/**!
 * CONSOLE PRINT
 * @brief Pring message on console
 */
#define MM_TEST_MSG(fmt, ...) \
  fprintf(stderr, "MM_MEDIA_TEST: %s::%d " fmt"\n",\
       __FUNCTION__, __LINE__, ##__VA_ARGS__)

/**!
 * BAIL_ON_ERROR()
 * @brief Return if there is any error
 */
#define BAIL_ON_ERROR(_e_) \
  if ( 0 != (_e_))\
{\
  goto ERROR_BAIL;\
}

/**!
 * SIZE_OF_ARRAY
 * @brief Return size of the array type _a_
 */

#define SIZE_OF_ARRAY(_a_) (sizeof((_a_))/sizeof(((_a_)[0])))

/**!
 * MM_TEST_MAX
 * @brief Return MAX of two number
 */
#define  MM_TEST_MAX( x, y ) ( ((x) > (y)) ? (x) : (y) )

/**!
 * MM_TEST_MIN
 * @brief Return MIN of two number
 */
#define  MM_TEST_MIN( x, y ) ( ((x) < (y)) ? (x) : (y) )

/**!
 * FLAG_SET
 * @brief Helper macro to set private/internal flag
 */
#define FLAG_SET(_nFlags_, _f_)    (_nFlags_  |= (_f_))

/**!
 * FLAG_ISSET
 * @brief Helper macro to check if a flag is set or not
 */
#define FLAG_ISSET(_nFlags_, _f_) ((_nFlags_ & (_f_)) ? TRUE : FALSE)

/**!
 * FLAG_CLEAR
 * @brief Helper macro to clear a private/internal flag
 */
#define FLAG_CLEAR(_nFlags_, _f_)   (_nFlags_ &= ~(_f_))


/**!
 * @brief Private flags for demuxer operation
 */
#define TEST_F_DEMUX_SAMPLE_OP   0x00000001
#define TEST_F_DEMUX_SEEK_OP     0x00000002
#define TEST_F_DEMUX_META_OP     0x00000002

/**!
 * @brief Macro related to files operation
 */
#define AUDIO_DUMP_FILE_EXT   "_ADump.bin"
#define VIDEO_DUMP_FILE_EXT   "_VDump.bin"
#define FILE_EXT_DOT '.'

/**!
 * @brief Macro related maximum number of tracks
 */
#define MM_SOURCE_MAX_TRACKS 12
/**!
 * @brief Macro related to MAX FILE PATH SIZE
 */
#ifndef MAX_PATH
#define MAX_PATH 512
#endif

/* ============================================================================
 **                          Type Definitions
 * ========================================================================== */

/**
 *  MM_TEST_STATUS
 * @ brief MM_TEST_STATUS error code
 */

typedef enum _MM_STATUS_TYPE_
{
  MM_STATUS_ErrorNone = 0,
  MM_STATUS_ErrorDefault = (int32)0x80001000,
  MM_STATUS_ErrorInvalidParam = (int32)0x80001001,
  MM_STATUS_ErrorMemAllocFail = (int32)0x80001002,
  MM_STATUS_ErrorUnsupported = (int32)0x80001003,
  MM_STATUS_ErrorReadFail = (int32)0x80001004,
  MM_STATUS_ErrorEndOfFile = (int32)0x80001005,
  MM_STATUS_ErrorStreamCorrupt = (int32)0x80001006

} MM_STATUS_TYPE;


/**
 *  MM_TRACK_TYPE
 * @ brief MM_TEST track types
 */

typedef enum _MM_TRACK_TYPE_
{
  TRACK_AUDIO = 0,
  TRACK_VIDEO
}MM_TRACK_TYPE;

/**
 * MM_TEST_SESSION_INFOTYPE
 * @ brief MM_TEST session type
 */

typedef enum _MM_TEST_SESSION_INFOTYPE_ {
  MM_TEST_SESSION_DEMUX = 1,
  MM_TEST_SESSION_MUX
}MM_TEST_SESSION_INFOTYPE;

/**
* MM_TEST_SOURCE_INFOTYPE
* @ brief MM_TEST Source type
*/

typedef enum _MM_TEST_SOURCE_INFOTYPE_ {
  MM_TEST_STREAM_PORT = 1,
  MM_TEST_LOCAL_FILE
}MM_TEST_SOURCE_INFOTYPE;
/**
 * MM_TRACK_CSD_INFOTYPE
 * @ brief Structure to hold codec configuration data
 */
typedef struct _MM_TRACK_CSD_INFOTYPE_{
  uint32 ulLen;
  uint8* pucData;
}MM_TRACK_CSD_INFOTYPE;

/**
 * MM_TRACK_SAMPLE_BUF_INFOTYPE
 * @ brief Structure to frames information
 */
typedef struct _MM_TRACK_SAMPLE_BUF_INFOTYPE_ {
  uint8* pucData1;    //! Data buffer1
  uint8* pucData2;    //! Data buffer2
  uint32 ulLen;       //! Frame length
  uint32 ulMaxLen;    //! Data buffer size
}MM_TRACK_SAMPLE_BUF_INFOTYPE;

/**
 * MM_AUDIO_TRACK_INFOTYPE
 * @ brief Structure to hold audio track meta information
 */
typedef struct _MM_AUDIO_TRACK_INFOTYPE_ {
  MM_TRACK_CSD_INFOTYPE sCSD;               //! CSD data
  MM_TRACK_SAMPLE_BUF_INFOTYPE sSampleBuf;  //! Sample data
  MM_HANDLE hAudioFile;;                    //! Dump file handle
  uint64  ullDuration;                      //! Track duration
  uint32  ulTimeScale;                      //! Track time scale unit
  uint32  ulTkId;                           //! Track id in use
  uint32  ulCodecType;                      //! Audio mime-type
  uint32  ulChCount;                        //! Audio channel count
  uint32  ulBitDepth;                       //! Audio bit depth
  uint32  ulBitRate;                        //! Audio bit rate
  uint32  ulSampleRate;                     //! Audio sample rate
  uint8   bTrackSelected;                   //! Is track selected for playback
}MM_AUDIO_TRACK_INFOTYPE;

/**
 * MM_VIDEO_TRACK_INFOTYPE
 * @ brief Structure to hold audio track meta information
 */
typedef struct _MM_VIDEO_TRACK_INFOTYPE_ {
  MM_TRACK_CSD_INFOTYPE sCSD;               //! CSD Data
  MM_TRACK_SAMPLE_BUF_INFOTYPE sSampleBuf;  //! Sample data
  MM_HANDLE hVideoFile;                     //! Dump file handle
  uint64  ullDuration;                      //! Track duration
  float   fFrameRate;                       //! Video frame rate
  uint32  ulTimeScale;                      //! Video track time scale unit
  uint32  ulTkId;                           //! Video track id
  uint32  ulCodecType;                      //! Video track mime-type
  uint32  ulWidth;                          //! Video track width
  uint32  ulHeight;                         //! Video track height
  uint32  ulBitRate;                        //! Video track bit rate
  uint8   bTrackSelected;                   //! Is track selected for playback
}MM_VIDEO_TRACK_INFOTYPE;

/**
 * MM_TRACK_INFOTYPE
 * @ brief Structure to hold tracks meta information
 */
typedef struct _MM_TRACK_INFOTYPE_ {
  uint32                  ulNumSelectedTrack; //! Number of selected tracks
  uint32                  ulNumTracks;        //! Total number of tracks
  MM_VIDEO_TRACK_INFOTYPE sVideo;             //! Video track meta infor
  MM_AUDIO_TRACK_INFOTYPE sAudio;             //! Audio track meta infor
}MM_TRACK_INFOTYPE;

/** Mandatory Track TYpe */
static const uint8 TRACK_TYPE[2] = {
  TRACK_AUDIO,
  TRACK_VIDEO
};
