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
#include "qmmf_demuxer_sourceport.h"

CMM_MediaSourcePort::CMM_MediaSourcePort(char* pFileName, int64 llBps)
  : m_llContentLength(0)
  , m_llBytesAvailable(0)
  , m_llBps(llBps)
  , m_ulRefCnt(0)
  , m_bRun(0)
  , m_pFilePtr(NULL)
  , m_pThreadHnd(NULL) {

  MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_LOW, "::CMM_MediaSourcePort");

  if (pFileName == NULL)
    return;

  if (0 != MM_File_Create((const char*)pFileName,
                          MM_FILE_CREATE_R,
                          &m_pFilePtr))
  {
    m_pFilePtr = NULL;
  }
  //! Get file size
  if (m_pFilePtr)
  {
    (void)MM_File_GetSize(m_pFilePtr, (unsigned long *)&m_llContentLength);
  }

  if (m_llBps < 8)
  {
    m_llBytesAvailable = m_llContentLength;
    MM_MSG_PRIO1(MM_FILE_OPS, MM_PRIO_LOW,
      "Entire file(bytes) %lld will be available!!!!", m_llBytesAvailable);
  }
  MM_MSG_PRIO1(MM_FILE_OPS, MM_PRIO_LOW,
    "::CMM_MediaSourcePort m_nContentLength %lld", m_llContentLength);

  if ((m_llBps > 8) &&
    (m_llContentLength) &&
    (0 != MM_Thread_CreateEx( MM_Thread_DefaultPriority,
                              0,
                              MM_MediaStrmPortThreadEntry,
                              (void *) this,
                              MM_MEDIA_STREAMPORT_THREAD_STACK_SIZE,
                              "MM_MEDIA_STRM_SIMULATOR",
                              &m_pThreadHnd))) {
    MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_LOW,
      "::CMM_MediaSourcePort Thread start failed..");
  }
}

CMM_MediaSourcePort::~CMM_MediaSourcePort()
{
  MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_LOW, "::~CMM_MediaSourcePort");
  if (m_pFilePtr)
  {
    (void)MM_File_Release(m_pFilePtr);
    m_pFilePtr = NULL;
  }
  if (m_pThreadHnd)
  {
    MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_LOW,
      "::~CMM_MediaSourcePort Setting m_bRun to false!!");
    m_bRun = false;
    while (m_llContentLength)
    {
      MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_LOW,
        "::~CMM_MediaSourcePort wait for Thread to stop!!");
      MM_Timer_Sleep(1000);
    }
    MM_Thread_Release(m_pThreadHnd);
  }
}

/**
 * Read data into the specified buffer (of given size) from given Offset.
 */
video::iSourcePort::DataSourceReturnCode CMM_MediaSourcePort::ReadAt(
   unsigned char* pBuf/*rout*/
  ,ssize_t nBufSize/*in*/
  ,const int64 nOffset/*in*/
  ,const int nWhence/*in*/
  ,ssize_t* pnRead/*rout*/) {

  int64 nTempOffset = nOffset;
  if (DS_SUCCESS == Seek(nOffset, nWhence, &nTempOffset))
  {
    return Read(pBuf, nBufSize, pnRead);
  }
  return DS_FAILURE;
}

/**
 * Read data into the specified buffer (of given size).
 */
video::iSourcePort::DataSourceReturnCode CMM_MediaSourcePort::Read(
   unsigned char* pBuf/*rout*/
  ,ssize_t nBufSize/*in*/
  ,ssize_t* pnRead/*rout*/) {

  iSourcePort::DataSourceReturnCode eRet = DS_FAILURE;
  if (!pBuf) {
    MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_ERROR,
      "::Read Buffer is NULL!!");
    return DS_FAILURE;
  }

  if (pnRead)
  {
    if ( 0 == \
         MM_File_Read(m_pFilePtr, (char*)pBuf, nBufSize, pnRead))
      eRet = DS_SUCCESS;
  }
  return eRet;
}

/**
 * Repositions the read/write point in an open port to the specified offset.
 * pnOutOffset gives the offset after a successful seek.
 */
video::iSourcePort::DataSourceReturnCode CMM_MediaSourcePort::Seek(
  const int64 nOffset/*in*/
  ,const int32 nWhence/*in*/
  ,int64* pnOutOffset/*rout*/) {
  // TODO : check whence
  iStreamPort::DataSourceReturnCode eRet = DS_FAILURE;
  MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_LOW, "CMM_MediaSourcePort::Seek");

  int32 posn = SEEK_END + 1;
  if (nWhence == DS_SEEK_SET) {
    posn = SEEK_SET;
  }
  if (nWhence == DS_SEEK_CUR) {
    posn = SEEK_CUR;
  }
  if (nWhence == DS_SEEK_END) {
    posn = SEEK_END;
  }
  if (!MM_File_SeekEx(m_pFilePtr, nOffset, posn)) {
    MM_MSG_PRIO1(MM_FILE_OPS, MM_PRIO_LOW,
      "::Seek OK nOffset Seeked %lld", nOffset);
    if (pnOutOffset) {
      MM_File_GetCurrentPosition(m_pFilePtr, (long unsigned int*)pnOutOffset);
    }
    eRet = DS_SUCCESS;
  }
  return eRet;
}

/**
 * Get the content length (in bytes).
 */
iSourcePort::DataSourceReturnCode CMM_MediaSourcePort::GetContentLength(
  /*rout*/ int64* pContentLength) {

  iSourcePort::DataSourceReturnCode eRet = DS_FAILURE;
  MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_LOW,
    "CMM_MediaSourcePort::GetContentLength");
  if (pContentLength) {
    *pContentLength = m_llContentLength;
    eRet = DS_SUCCESS;
  }
  return eRet;
}

/**
 * Get the underlying data source type.
 */
iSourcePort::DataSourceReturnCode CMM_MediaSourcePort::GetSourceType(
  /*rout*/ DataSourceType* pSourceType)
{
  iSourcePort::DataSourceReturnCode ret = DS_FAILURE;
  MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_LOW, "::GetSourceType");
  if (pSourceType) {
    *pSourceType = DS_STREAMING_SOURCE;
    ret = DS_SUCCESS;
    MM_MSG_PRIO1(MM_FILE_OPS, MM_PRIO_LOW, "::GetSourceType %d",
      *pSourceType);
  }
  return ret;
}

/**
 * Get the number of bytes available.
 */
iSourcePort::DataSourceReturnCode CMM_MediaSourcePort::GetNumBytesAvailable(
  int64* pNumBytesAvailable/*rout*/)
{
  iSourcePort::DataSourceReturnCode ret = DS_FAILURE;
  MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_LOW,
    "CMM_MediaSourcePort::GetNumBytesAvailable");
  if (pNumBytesAvailable) {
    *pNumBytesAvailable = m_llBytesAvailable;
    ret = DS_SUCCESS;
  }
  return ret;
}

/**
 * Get Start offset
*/

iSourcePort::DataSourceReturnCode CMM_MediaSourcePort::GetStartOffset(
  /*rout*/ int64* pStartOffset) {

  iSourcePort::DataSourceReturnCode ret = DS_FAILURE;
  if (pStartOffset) {
    *pStartOffset = 0;
    ret = DS_SUCCESS;
  }
  return ret;
}

iSourcePort::DataSourceReturnCode CMM_MediaSourcePort::GetAvailableOffset(
   int64* pAvailableOffset /*rout*/
  ,bool* pbEOS /*rout*/)
{

    *pbEOS = true;

    uint32_t eRet = GetContentLength(pAvailableOffset);
    ALOGE(" SourcePort::GetAvailableOffset pAvailableOffset = %lld \n",
      *pAvailableOffset);
    if((DS_SUCCESS != eRet) || (0 == *pAvailableOffset))
    {
        eRet = 0;
        *pAvailableOffset = -1;
    }
    return (0 == eRet ? DS_SUCCESS : DS_FAILURE);
}


/**
 * Register a callback interface to be invoked when data is available to be
 * read (i.e. Read() would return something other than DS_WAIT).
 */
iSourcePort::DataSourceReturnCode CMM_MediaSourcePort::Readable(
  /*in*/ iReadNotificationHandler const* /*pNotificationHandler*/) {
  return DS_SUCCESS;
}

/**
 * Write data from the specified buffer (of given size).
 */
iSourcePort::DataSourceReturnCode CMM_MediaSourcePort::Write(
  /*in*/ const unsigned char* /*pBuf*/,
  /*in*/ ssize_t /*nBufSize*/,
  /*rout*/ ssize_t* /*pnWritten*/) {
  return DS_SUCCESS;
}

void* CMM_MediaSourcePort::QueryInterface(const AEEIID /*iid*/) {
  return this;
}

uint32 CMM_MediaSourcePort::AddRef() {
  return ++m_ulRefCnt;
}

uint32 CMM_MediaSourcePort::Release() {
  return --m_ulRefCnt;
}

/**
Close the data source port - port becomes unusable after this call.
*/
iStreamPort::DataSourceReturnCode CMM_MediaSourcePort::Close() {
  return DS_SUCCESS;
}

int CMM_MediaSourcePort::MM_MediaStrmPortThreadEntry(void* ptr) {
  CMM_MediaSourcePort* portPtr = (CMM_MediaSourcePort*)ptr;
  if (ptr)
  {
    portPtr->MediaStreamPortThread();
  }
  return 0;
}

void CMM_MediaSourcePort::MediaStreamPortThread()
{
  MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_LOW, "::QCIStreamPortThread");
  MM_MSG_PRIO1(MM_FILE_OPS, MM_PRIO_LOW,
    "Simulating %lld bits per second....", m_llBps);
  m_bRun = true;
  int64 nBytesAvailable = 0;
  int64 nSecSinceRunning = 0;
  int64 nBytes = 0;
  bool bSkipBpsCalc = false;

  while (m_bRun)
  {
    MM_Timer_Sleep(1000);
    if ((m_llBps) && (!bSkipBpsCalc))
    {
      nBytes = m_llBps / 8;
      nBytesAvailable = (nSecSinceRunning * nBytes);
      if (nBytesAvailable > m_llContentLength)
      {
        MM_MSG_PRIO1(MM_FILE_OPS, MM_PRIO_LOW,
          "::QCIStreamPortThread nSecSinceRunning %lld", nSecSinceRunning);
        m_llBytesAvailable = m_llContentLength;
        bSkipBpsCalc = true;
        MM_MSG_PRIO(MM_FILE_OPS, MM_PRIO_LOW,
          "::QCIStreamPortThread Entire file is now available.....");
      }
      else
      {
        m_llBytesAvailable = nBytesAvailable;
      }
    }
    if (DS_SUCCESS == GetNumBytesAvailable(&nBytesAvailable))
    {
      MM_MSG_PRIO1(MM_FILE_OPS, MM_PRIO_LOW,
        "::QCIStreamPortThread nSecSinceRunning %lld", nSecSinceRunning);
      MM_MSG_PRIO1(MM_FILE_OPS, MM_PRIO_LOW,
        "::QCIStreamPortThread nBytesAvailable %lld", nBytesAvailable);
    }
    nSecSinceRunning++;
  }
  MM_MSG_PRIO1(MM_FILE_OPS, MM_PRIO_LOW,
    "::QCIStreamPortThread Simulation ran for %lld seconds.", nSecSinceRunning);
  m_llContentLength = 0;
}
