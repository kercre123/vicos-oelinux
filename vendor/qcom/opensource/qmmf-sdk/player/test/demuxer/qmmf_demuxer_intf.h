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
#include "filesource.h"
#include "filesourcetypes.h"
#include "DataSourcePort.h"
#include "MMMalloc.h"
#include "qmmf_demuxer_sourceport.h"

/* ============================================================================
 **                        Class Declarations
 **========================================================================== */

class CMM_MediaDemuxInt {
 public:
  //! CTOR
  static CMM_MediaDemuxInt* New(
    const char* pFileName,
    FileSourceFileFormat eFileFormat);

  static CMM_MediaDemuxInt* New(
    iStreamPort& aStreamPort,
    FileSourceFileFormat eFileFormat);

  virtual ~CMM_MediaDemuxInt();
  //! Get whole track list entry
  uint32 GetWholeTracksIDList(
    FileSourceTrackIdInfoType *trackIdInfo);
  //! Get major & minor mimetype of stream
  FileSourceStatus GetMimeType(
    uint32 id,
    FileSourceMjMediaType& majorType,
    FileSourceMnMediaType& minorType);
  //! Get media track information
  FileSourceStatus GetMediaTrackInfo(
    uint32 id,
    MediaTrackInfo* info);

  uint64  GetTrackMediaDuration(uint32 id);

  uint64  GetClipDuration();

  FileSourceStatus GetFormatBlock(
    uint32 id,
    uint8* buf,
    uint32 *pbufSize,
    bool bRawCodec = false);

  bool GetWMACodecData(
    uint32 id,
    WmaCodecData* pCodecData);

  uint8 IsSeekDenied();

  FileSourceStatus SeekAbsolutePosition(
    int64 trackid,
    const int tAbsoluteTime,
    bool bSeekToSync = true,
    int64 nCurrPlayTime = -1,
    FS_SEEK_MODE eSeekMode = FS_SEEK_DEFAULT);

  FileSourceStatus SeekAbsolutePosition(
    const int tAbsoluteTime,
    bool bSeekToSync = true,
    int64 nCurrPlayTime = -1,
    FS_SEEK_MODE eSeekMode = FS_SEEK_DEFAULT);

  FileSourceStatus SeekRelativeSyncPoint(
    int currentPlaybacktime,
    const int numSync);

  int32 GetTrackMaxFrameBufferSize(uint32 id);

  FileSourceMediaStatus GetNextMediaSample(
    uint32 id,
    uint8 *buf,
    uint32 *size,
    FileSourceSampleInfo& pSampleInfo);

  FileSourceStatus GetFileFormat(FileSourceFileFormat& fileFormat);

  bool IsDrmProtection();

  FileSourceStatus SetConfiguration(
    uint32 id,
    FileSourceConfigItem* pItem,
    FileSourceConfigItemEnum ienumData);

  FileSourceStatus GetConfiguration(
    uint32 id,
    FileSourceConfigItem* pItem,
    FileSourceConfigItemEnum ienumData);

  FileSourceStatus GetStreamParameter(
    uint32 id,
    uint32 paramIndex,
    void* ptr);

  bool GetAACCodecData(
    uint32 id,
    AacCodecData* pCodecData);

  bool GetWavCodecData(
    uint32 id,
    WavFormatData* pCodecData);

  FileSourceStatus GetClipMetaData(
    wchar_t* pMetaData,
    uint32* size,
    FileSourceMetaDataType pSampleInfo,
    FS_TEXT_ENCODING_TYPE *pEncode = NULL);

  void WaitForSeekStatus();
  void WaitForOpenStatus();

  FileSource *  m_pFileSource;
 /* ---------------------------------------------------------------------------
  ** Private member/function
  ** ----------------------------------------------------------------------- */

 private:
  CMM_MediaDemuxInt();
  static void cbFileSourceStatus(
    FileSourceCallBackStatus status,
    void* pCbData);

  FileSourceStatus Init(
    const char* pFileName,
    FileSourceFileFormat eFileFormat);

  FileSourceStatus Init(
    iStreamPort& aStreamPort,
    FileSourceFileFormat eFileFormat);


 private:

  wchar_t*      m_pFileName;
  // The signal Q for the filter thread to wait on.
  MM_HANDLE     m_pSignalQ;

  // The signal associated with the open file success event.
  MM_HANDLE     m_pOpenFileSuccessSignal;

  // The signal associated with the open file fail event.
  MM_HANDLE     m_pOpenFileFailSignal;

  // The signal associated with the seek file success event.
  MM_HANDLE     m_pSeekFileSuccessSignal;

  // The signal associated with the seek file fail event.
  MM_HANDLE     m_pSeekFileFailSignal;

  FileSourceParserError m_eParseError;

  bool   iDrmEncrypted; //Indicates if clip is DRM encrypted
  // Mutex mMutex;
  FileSourceStatus m_eFSStatus;
  // Semaphore mWaitSeamaphore;
 public:
  FileSourceFileFormat mFileFormat;
  FileSourceMnMediaType mVideoMinorType;
};

