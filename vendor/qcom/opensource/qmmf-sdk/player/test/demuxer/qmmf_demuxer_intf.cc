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

/**============================================================================
*
* INCLUDE FILES FOR MODULE
*
*=========================================================================== */
#include<unistd.h>
#include<string.h>
#include "qmmf_demuxer_intf.h"
#include "MMDebugMsg.h"
#include "MMMemory.h"
#include "MMSignal.h"

//! Events the filter thread processes.
static const uint32 OPEN_FILE_SUCCESS_EVENT = 0;
static const uint32 OPEN_FILE_FAIL_EVENT = 1;
static const uint32 SEEK_FILE_SUCCESS_EVENT = 2;
static const uint32 SEEK_FILE_FAIL_EVENT = 3;

/*! ===========================================================================
  @brief      CMM_MediaDemuxInt::New

  @detail     Create a new instance of CMM_MediaDemuxInt

  @param[in]  aStreamPort:  iStreamPort interface
  @param[in]  eFileFormat   File Format of input file.

  @return     CMM_MediaDemuxInt instance
============================================================================= */
CMM_MediaDemuxInt* CMM_MediaDemuxInt::New(
  iStreamPort& aStreamPort,
  FileSourceFileFormat eFileFormat) {
  CMM_MediaDemuxInt* self = new CMM_MediaDemuxInt;
  if (!self) {
    return NULL;
  }

  FileSourceStatus eStatus = self->Init( aStreamPort, eFileFormat);
  if (eStatus != FILE_SOURCE_SUCCESS) {
    MM_MSG_PRIO1(MM_FILE_OPS, MM_PRIO_ERROR,
      "CMM_MediaDemuxInt::New Init return %d", eStatus);
    delete self;
    return NULL;
  }

  return self;
}

/*! ===========================================================================
  @brief      CMM_MediaDemuxInt::Init

  @detail     Create a new FileSource instance

  @param[in]  aStreamPort:  iStreamPort interface
  @param[in]  eFileFormat   File Format of input file.

  @return     FileSourceStatus status as FILE_SOURCE_SUCCESS in success orther
              wise respective failure.
============================================================================= */
FileSourceStatus CMM_MediaDemuxInt::Init(iStreamPort& aStreamPort,
  FileSourceFileFormat eFileFormat) {
  FileSourceStatus eStatus = FILE_SOURCE_FAIL;
  //! Create synchronous FileSource
  m_pFileSource = new FileSource(CMM_MediaDemuxInt::cbFileSourceStatus,
    (void*)this, true);
  if (!m_pFileSource) {
    return eStatus;
  }

  //! Open file for parsing
  eStatus = m_pFileSource->OpenFile(&aStreamPort, eFileFormat, TRUE);
  //! Wait for file parsing status i.e. open success/fail

  WaitForOpenStatus();
  eStatus = m_eFSStatus;
  MM_MSG_PRIO1(MM_FILE_OPS, MM_PRIO_ERROR,
    "CMM_MediaDemuxInt::Init Status %d", eStatus);

  return eStatus;
}

/*! ===========================================================================
  @brief      CMM_MediaDemuxInt::Init

  @detail     CMM_MediaDemuxInt CTOR

  @param[in]  NA

  @return     NA
============================================================================= */
CMM_MediaDemuxInt::CMM_MediaDemuxInt()
  : m_pFileSource(NULL)
  , m_pFileName(NULL)
  , m_pSignalQ(NULL)
  , m_pOpenFileSuccessSignal(NULL)
  , m_pOpenFileFailSignal(NULL)
  , m_pSeekFileSuccessSignal(NULL)
  , m_pSeekFileFailSignal(NULL)
  , m_eParseError(FILE_SOURCE_PARSER_UNKNOWN_ERROR)
  , iDrmEncrypted(false)
  , m_eFSStatus(FILE_SOURCE_FAIL)
  , mFileFormat(FILE_SOURCE_UNKNOWN)
  , mVideoMinorType(FILE_SOURCE_MN_TYPE_UNKNOWN) {

  bool bResult = false;
  /* Create the signal Q for the thread to wait on. */
  if (0 == MM_SignalQ_Create(&m_pSignalQ))
  {
    bResult = true;
  }

  /* This signal is set when OpenFile operation is successful and filter
   * thread receives callback from FileSource thread with status FileSource::
   * FILE_SOURCE_OPEN_COMPLETE
   */
  if ((bResult) && (0 == MM_Signal_Create(m_pSignalQ,
                                          (void *)&OPEN_FILE_SUCCESS_EVENT,
                                          NULL,
                                          &m_pOpenFileSuccessSignal)))
  {
    bResult = true;
  }

  /* This signal is set when OpenFile operation fails and filter thread
   * receives callback from FileSource thread with status FileSource::
   * FILE_SOURCE_OPEN_FAIL
   */
  if ((bResult) && (0 == MM_Signal_Create(m_pSignalQ,
                                          (void *)&OPEN_FILE_FAIL_EVENT,
                                          NULL,
                                          &m_pOpenFileFailSignal)))
  {
    bResult = true;
  }

  /* This signal is set when seek operation is successful and filter thread
   * receives callback from FileSource thread with status FileSource::
   * FILE_SOURCE_SEEK_COMPLETE
   */

  if ((bResult) && (0 == MM_Signal_Create(m_pSignalQ,
                                          (void *)&SEEK_FILE_SUCCESS_EVENT,
                                          NULL,
                                          &m_pSeekFileSuccessSignal)))
  {
    bResult = true;
  }
  /* This signal is set when seek operation fails and filter thread receives
   * callback from FileSource thread with status FileSource::
   * FILE_SOURCE_SEEK_FAIL
   */
  if ((bResult) && (0 == MM_Signal_Create(m_pSignalQ,
                                          (void *)&SEEK_FILE_FAIL_EVENT,
                                          NULL,
                                          &m_pSeekFileFailSignal)))
  {
    bResult = true;
  }

}

/*! ===========================================================================
  @brief      CMM_MediaDemuxInt::~CMM_MediaDemuxInt

  @detail     CMM_MediaDemuxInt::DTOR

  @param[in]  NA

  @return     NA
============================================================================= */
CMM_MediaDemuxInt::~CMM_MediaDemuxInt() {
  MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_ERROR,
    " CMM_MediaDemuxInt::~CMM_MediaDemuxInt");
  /* Release the open file success signal. */
  MM_Signal_Release(m_pOpenFileSuccessSignal);

  /* Release the open file fail signal. */
  MM_Signal_Release(m_pOpenFileFailSignal);

  /* Release the seek file success signal. */
  MM_Signal_Release(m_pSeekFileSuccessSignal);

  /* Release the seek file fail signal. */
  MM_Signal_Release(m_pSeekFileFailSignal);

  /* Release the signal Q. */
  MM_SignalQ_Release(m_pSignalQ);

  if (m_pFileName)
    MM_Delete_Array(m_pFileName);

  if(m_pFileSource){
    m_pFileSource->CloseFile();
    delete m_pFileSource;
  }
}

/*! ===========================================================================
  @brief      CMM_MediaDemuxInt::cbFileSourceStatus

  @detail     FileSource status CB

  @param[in]  status:  FileSourceCallBackStatus
  @param[in]  pCbData  Client data

  @return     NA
============================================================================= */
void CMM_MediaDemuxInt::cbFileSourceStatus(
  FileSourceCallBackStatus status,
  void* pCbData){
  CMM_MediaDemuxInt* pThis = (CMM_MediaDemuxInt *)pCbData;

  if (pThis)
  {
    //File Source event
    switch (status)
    {
    case FILE_SOURCE_OPEN_COMPLETE:
      /* Post the event to the filter thread */
      MM_Signal_Set(pThis->m_pOpenFileSuccessSignal);
      MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_LOW,
        " CB_STATUS:FILE_SOURCE_OPEN_COMPLETE");
      break;

    case FILE_SOURCE_OPEN_FAIL:
      /* Post the event to the filter thread */
      MM_Signal_Set(pThis->m_pOpenFileFailSignal);
      MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_LOW,
        " CB_STATUS:FILE_SOURCE_OPEN_FAIL");
      break;

    case FILE_SOURCE_SEEK_COMPLETE:
      /* Post the event to the filter thread */
      MM_Signal_Set(pThis->m_pSeekFileSuccessSignal);
      MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_LOW,
        " CB_STATUS:FILE_SOURCE_SEEK_COMPLETE");
      break;

    case FILE_SOURCE_SEEK_FAIL:
      /* Post the event to the filter thread */
      MM_Signal_Set(pThis->m_pSeekFileFailSignal);
      MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_LOW,
        " CB_STATUS:FILE_SOURCE_SEEK_FAIL");
      break;

    default:
      MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_LOW,
        " CB_STATUS:Unsupported!!");
      break;
    }
  }
}

/*! ===========================================================================
  @brief      CMM_MediaDemuxInt::WaitForOpenStatus

  @detail     Wait for parsing status from FileSource

  @param[in]  NA

  @return     NA

============================================================================= */
void CMM_MediaDemuxInt::WaitForOpenStatus(){
  uint32 *pEvent = NULL;
  MM_MSG_PRIO(MM_GENERAL, MM_PRIO_LOW,
    "CMM_MediaDemuxInt::WaitForOpenStatus");
  FileSourceStatus eStatus = FILE_SOURCE_FAIL;

  if ( ( 0 == MM_SignalQ_Wait(m_pSignalQ, (void **)&pEvent)))
  {
    switch (*pEvent)
    {
    case OPEN_FILE_SUCCESS_EVENT:
    {
      eStatus = FILE_SOURCE_SUCCESS;
      m_eFSStatus = eStatus;
      m_eParseError = m_pFileSource->GetFileError();
      MM_MSG_PRIO1(MM_GENERAL, MM_PRIO_LOW,
        "CMM_MediaDemuxInt::Received OPEN_FILE_SUCCESS_EVENT Error %d",
        m_eParseError);
      break;
    }

    case OPEN_FILE_FAIL_EVENT:
    {
      eStatus = FILE_SOURCE_FAIL;
      m_eFSStatus = eStatus;
      m_eParseError = m_pFileSource->GetFileError();
      MM_MSG_PRIO1(MM_GENERAL, MM_PRIO_LOW,
        "CMM_MediaDemuxInt::Received OPEN_FILE_FAIL_EVENT Error %d",
        m_eParseError);
      break;
    }

    default:
    {
      /* Not a recognized event, ignore it. */
      m_eFSStatus = eStatus;
      m_eParseError = m_pFileSource->GetFileError();
      MM_MSG_PRIO1(MM_GENERAL, MM_PRIO_ERROR,
        "CMM_MediaDemuxInt::Received Unsupported Event Error %d",
        m_eParseError);
      break;
    }
    }//switch(pEvent)
  }//if(MM_SignalQ_Wait)
}

/*! ===========================================================================
  @brief      CMM_MediaDemuxInt::WaitForSeekStatus

  @detail     Wait for seek operation status from FileSource

  @param[in]  NA

  @return     NA

============================================================================= */
void  CMM_MediaDemuxInt::WaitForSeekStatus(){
  uint32 *pEvent = NULL;

  MM_MSG_PRIO(MM_GENERAL, MM_PRIO_LOW,
    "CMM_MediaDemuxInt::WaitForSeekStatus");
  if (0 == MM_SignalQ_Wait(m_pSignalQ, (void **)&pEvent))
  {
    switch (*pEvent)
    {
      case SEEK_FILE_SUCCESS_EVENT:
      {
        MM_MSG_PRIO(MM_GENERAL, MM_PRIO_LOW,
          "CMM_MediaDemuxInt::WaitForSeekStatus SEEK successful");
        m_eFSStatus = FILE_SOURCE_S_SEEK_COMPLETE;
        break;
      }

      case SEEK_FILE_FAIL_EVENT:
      {
        MM_MSG_PRIO(MM_GENERAL, MM_PRIO_LOW,
          "CMM_MediaDemuxInt::WaitForSeekStatus seek failed");
        m_eFSStatus = FILE_SOURCE_S_SEEK_FAIL;
        break;
      }

      default:
      {
        MM_MSG_PRIO(MM_GENERAL, MM_PRIO_LOW,
          "CMM_MediaDemuxInt::WaitForSeekStatus unknown event received");
        m_eFSStatus = FILE_SOURCE_FAIL;
        break;
      }
    }//switch(pEvent)
  }//if(MM_SignalQ_Wait)
}

/*! ===========================================================================
  @brief      CMM_MediaDemuxInt::GetWholeTracksIDList

  @detail     Method to retrieve all valid trackID List

  @param[out]  trackIdInfo  a list of TrackIfInfo ( consisting of trackid's
                            and if a book if they are selected or not).
  @return      Number of valid audio, video and text tracks.

============================================================================= */
uint32 CMM_MediaDemuxInt::GetWholeTracksIDList(
  FileSourceTrackIdInfoType *trackIdInfo) {
  return m_pFileSource->GetWholeTracksIDList(trackIdInfo);
}

/*! ===========================================================================
  @brief      CMM_MediaDemuxInt::GetMimeType
  @detail     Method returns Major and Minor Type for the Clip.

  @param[in]  id         Track ID
  @param[out] majorType  Major Media Types ( Audio/Video/Text).
  @param[out] minorType  Minor Media Types ( Sub media type within Audio/
                          Video text) and its also referred as Codecs
  @return     filesource status.

============================================================================= */
FileSourceStatus CMM_MediaDemuxInt::GetMimeType(
  uint32 id,
  FileSourceMjMediaType& majorType,
  FileSourceMnMediaType& minorType) {
  return m_pFileSource->GetMimeType(id, majorType, minorType);
}

/*! ===========================================================================
  @brief    Provides information about the Track.

  @detail   Method to retrieve information about the Audio/Video Track. This
            interface is providedfor User to do channel selection OR use it as
            a criteria to select a particular track.

  @param[in]  id   track Id to select
  @param[out] info Information about the given track id
  @return     FILE_SOURCE_SUCCESS if track is valid else returns
              FILE_SOURCE_FAIL

============================================================================= */
FileSourceStatus CMM_MediaDemuxInt::GetMediaTrackInfo(
  uint32 id,
  MediaTrackInfo* info) {
  return m_pFileSource->GetMediaTrackInfo(id, info);
}

/*! ===========================================================================
  @brief     CMM_MediaDemuxInt::GetTrackMediaDuration

  @detail    This method retrives the duration from  the track.

  @param[in] id   track Id to select.

  @return    duration of the track in milli seconds

==============================================================================*/
uint64 CMM_MediaDemuxInt::GetTrackMediaDuration(uint32 id) {
  return m_pFileSource->GetTrackMediaDuration(id);
}

uint64 CMM_MediaDemuxInt::GetClipDuration() {
  return m_pFileSource->GetClipDuration();
}

/*! ===========================================================================
  @brief  CMM_MediaDemuxInt::GetFormatBlock

  @detail Method to retrieve the Decoder specific/Format Block information
          from the track. This interface is generic for Audio, Video and Text.
          If buf = NULL, then the function give the size of the required buffer.
          Following is an example of retrieving the format block.

          1. Invoke getFormatBlock API for a given track identifier by passing
             in NULL for buf.
          2. If a track is valid, *pbufSize will give you the size of format
             block.
          3. Allocate the memory and invoke getFormatBlock API for a given
             track identifier by passing handle to allocated memory.

  @param[in]   id   track Id to select.
  @param[out]  buf   Buffer provies the format block info to the caller
  @param[out]  pBufSize   Size of the FormatBlock buffer
  @param[in]   bRawCodec  Flag to indicate whether codec data required in input
                          format or converted to proper SPS/PPS format

  @return     file source status.

==============================================================================*/
FileSourceStatus CMM_MediaDemuxInt::GetFormatBlock(
  uint32 id,
  uint8* buf,
  uint32 *pbufSize,
  bool bRawCodec) {
  return m_pFileSource->GetFormatBlock(id, buf, pbufSize, bRawCodec);
}

/*! ===========================================================================
  @brief       CMM_MediaDemuxInt::GetFileFormat

  @detail      Method returns the file format.

  @param[out]  fileFormat   type of  FileFormat ( MPEG4/ASF/AVI).

  @return      filesource status.

============================================================================= */
FileSourceStatus CMM_MediaDemuxInt::GetFileFormat(
  FileSourceFileFormat& fileFormat) {
  FileSourceStatus status = FILE_SOURCE_FAIL;
  fileFormat = FILE_SOURCE_UNKNOWN;
  if (m_pFileSource)
  {
    status = m_pFileSource->GetFileFormat(fileFormat);
  }
  return status;
}

/*! ===========================================================================

  @brief         CMM_MediaDemuxInt::GetFileFormat
  @detail        Retrieve the wma codec data needed to configure WMA decoder.

  @param[in] id  Identifies the WMA track for which codec data needs to be retrieved
  @param[in,out] pBlock filled in by FileSource

  @return        TRUE is successful in retrieving the codec data else returns
                 FALSE

========================================================================== */
bool CMM_MediaDemuxInt::GetWMACodecData(
  uint32 id,
  WmaCodecData* pCodecData) {
  return m_pFileSource->GetWMACodecData(id, pCodecData);
}

/*! ===========================================================================

  @brief      CMM_MediaDemuxInt::IsSeekDenied
  @brief      Local function to check if Seek is allowed in the clip

  @param[in]  NA

  @return     TRUE or FALSE

============================================================================= */
uint8 CMM_MediaDemuxInt::IsSeekDenied() {
  return m_pFileSource->IsSeekDenied();
}

/*! ===========================================================================
  @brief    Reposition given track to an absolute position.

  @detail   This API will try to seek the track identified via trackid to
            given absolute time provided there is an I frame at that time.
            If there are no I frames in forward direction, forward seek
            request can fail. To allow parser to seek to non key frame,
            bSeekToSync can be set to false.

@param[in]  trackid       Track-Id to specify which track to seek
@param[in]  tAbsoluteTime Seek to the absolute position(ms).
@param[in]  bSeekToSync   When set to false, parser can seek to non sync frame
@param[in]  nCurrPlayTime Current playback time.(-1 indicates time unknown)

@return     file source status.

============================================================================= */
FileSourceStatus CMM_MediaDemuxInt::SeekAbsolutePosition(
  int64 trackid,
  const int tAbsoluteTime,
  bool bSeekToSync,
  int64 nCurrPlayTime,
  FS_SEEK_MODE eSeekMode) {

  FileSourceStatus err = \
    m_pFileSource->SeekAbsolutePosition(trackid,
                                        tAbsoluteTime,
                                        bSeekToSync,
                                        nCurrPlayTime,
                                        eSeekMode);
  if (err != FILE_SOURCE_SUCCESS) {
    return err;
  }
  (void)WaitForSeekStatus();
  return m_eFSStatus;
}
/*! ===========================================================================

  @brief    Reposition to an absolute position.

  @detail   The routine will seek/repositon to next valid sample based on the
            timestamp provided. This function Uses Video track as the primary
            source to find the time stamp and then sync's the Audio and Text
            accordingly. However, if Audio is not present it will use Audio as
            the reference.

  @param[in]  tAbsoluteTime Seek to the absolute position(ms).
  @param[in]  bSeekToSync   When set to false, parser can seek to non sync frame
  @param[in]  nCurrPlayTime Current playback time.(-1 indicates time unknown)

  @return     file source status.

============================================================================= */
FileSourceStatus CMM_MediaDemuxInt::SeekAbsolutePosition(
  const int tAbsoluteTime,
  bool bSeekToSync,
  int64 nCurrPlayTime,
  FS_SEEK_MODE eSeekMode) {

  FileSourceStatus err = m_pFileSource->SeekAbsolutePosition(tAbsoluteTime,
                                                             bSeekToSync,
                                                             nCurrPlayTime,
                                                             eSeekMode);
  if (err != FILE_SOURCE_SUCCESS) {
    return err;
  }
  (void)WaitForSeekStatus();
  return m_eFSStatus;
}

/*! ===========================================================================

  @brief    CMM_MediaDemuxInt::SeekRelativeSyncPoint

  @detail   Reposition to a relative Sync Sample/number of sync sample.
            The routine will seek/repositon to next sync sample based on the
            timestamp provided. The direction can be both positive and negative.

  @param[in]  currentPlaybacktime current playback time(ms) from the point of
                                  view of the User/Caller.
  @param[in]  numSync             number of sync sample to jump. The value can
                                  be both positive and negative, specifiying
                                  the direction to search for the Sync Sample.
  @return     file source status.

============================================================================= */
FileSourceStatus CMM_MediaDemuxInt::SeekRelativeSyncPoint(
  int currentPlaybacktime,
  const int numSync) {

  FileSourceStatus err = \
    m_pFileSource->SeekRelativeSyncPoint(currentPlaybacktime, numSync);
  if (err != FILE_SOURCE_SUCCESS) {
    return err;
  }
  (void)WaitForSeekStatus();
  return m_eFSStatus;
}

/*! ===========================================================================
  @brief     CMM_MediaDemuxInt::GetTrackMaxFrameBufferSize

  @detail    Maximum Buffer size required for the Track.Before we parse a clip
             we do not know the size of the Frame. There are two ways to solve
             this issue. One is to allocate a huge memory buffer, which is
             in-efficient use of memory OR use this method to retrieve the
             buffer size needed for the frame and then allocate/reallocate
             memory as needed.

  @param[in]  id   track Id to select.

  @return  largest frame size up to the frame we have parsed.

============================================================================= */
int32 CMM_MediaDemuxInt::GetTrackMaxFrameBufferSize(uint32 id) {
  return m_pFileSource->GetTrackMaxFrameBufferSize(id);
}

/*! ===========================================================================
   @brief    CMM_MediaDemuxInt::GetNextMediaSample

  @detail    Provides Media Samples for requested tracks

  @param[in]  id          The track ID of the track from which the method is to
                          retrieve the samples.
  @param[out] buf         A ptr to the buffer into which to place the sample.
  @param[out] size        The size of the data buffer.
  @param[out] pSampleInfo Provides Information( eg: timestamp etc) about the sample

  @return     The size in bytes of the data placed into the provided buffer.
              If the buffer is not large enough, the return value is the -ve
              of the size that is needed .

============================================================================= */
FileSourceMediaStatus CMM_MediaDemuxInt::GetNextMediaSample(
  uint32 id,
  uint8 *buf,
  uint32 *size,
  FileSourceSampleInfo& pSampleInfo) {

  return m_pFileSource->GetNextMediaSample(id, buf, size, pSampleInfo);
}

/*! ===========================================================================
  @brief    CMM_MediaDemuxInt::GetClipMetaData

  @detail   Method returns metadata about the Clip.

  @param[in]  ienumData  Identifies requested metadata type( eg: Title etc).
  @param[out] oMetaData  Returns a string associated with requested metadata

  @return     file source status

============================================================================= */
FileSourceStatus CMM_MediaDemuxInt::GetClipMetaData(
  wchar_t* pMetaData,
  uint32* size,
  FileSourceMetaDataType pSampleInfo,
  FS_TEXT_ENCODING_TYPE *pEncode) {

  return m_pFileSource->GetClipMetaData(pMetaData, size, pSampleInfo, pEncode);
}

/*! ===========================================================================
  @brief      Method to Set configuration item.

  @param[in]  id        Track id to identify track to which configuration data
  @param[out] pItem     Configuration data filled in by caller
  @param[in]  ienumData Identifies the configuration item.
                        Please refer to FileSourceConfigItemEnum.

  @return     file source status

============================================================================= */
FileSourceStatus CMM_MediaDemuxInt::SetConfiguration (
  uint32 id,
  FileSourceConfigItem* pItem,
  FileSourceConfigItemEnum ienumData) {
  return m_pFileSource->SetConfiguration(id, pItem, ienumData);
}

/*! ===========================================================================
  @brief    CMM_MediaDemuxInt::GetConfiguration
  @detail   Method to retrieve configuration item set previously via SetConfiguration.

  @param[in]  id        Track id to identify track to which configuration data belongs.
  @param[out] pItem     Configuration data filled in by parser
  @param[in]  ienumData Identifies the configuration item. Please refer to
                        FileSourceConfigItemEnum.

  @return         file source status

============================================================================= */
FileSourceStatus CMM_MediaDemuxInt::GetConfiguration(
  uint32 id,
  FileSourceConfigItem* pItem,
  FileSourceConfigItemEnum ienumData) {
  return m_pFileSource->GetConfiguration(id, pItem, ienumData);
}

/*! ===========================================================================
  @brief      CMM_MediaDemuxInt::GetStreamParameter
  @detail     Method to retrieve Stream specific parameters.

  @param[in]  ulTrackId     Track id to identify stream.
  @param[in]  ulParamIndex  Parameter Index to identify structure'
  @param[in]  pParamStruct  Pointer to the structure.

  @return     file source status

============================================================================== */
FileSourceStatus CMM_MediaDemuxInt::GetStreamParameter(
  uint32 id,
  uint32 paramIndex,
  void* ptr)
{
  return m_pFileSource->GetStreamParameter(id, paramIndex, ptr);
}

/*! ===========================================================================
  @brief CMM_MediaDemuxInt::GetWavCodecData.

  @brief Retrieve the wav codec data needed to configure wav decoder.

  @param[in] id  TrackId
  @param[in,out] pBlock filled in by FileSource

  @return true is successful in retrieving the codec data else returns false

============================================================================== */
bool CMM_MediaDemuxInt::GetWavCodecData(
  uint32 id,
  WavFormatData* pCodecData) {
  return m_pFileSource->GetWavCodecData(id, pCodecData);
}

/*! ===========================================================================
  @brief CMM_MediaDemuxInt::GetWavCodecData.

  @brief Retrieve the WAV codec data needed to configure WAV decoder

  @param[in]     id         TrackID
  @param[in,out] pCodecData Filled in by FileSource

  @return true is successful in retrieving the codec data else returns false

============================================================================== */
bool CMM_MediaDemuxInt::GetAACCodecData(
  uint32 id,
  AacCodecData* pCodecData) {
  return m_pFileSource->GetAACCodecData(id, pCodecData);
}
