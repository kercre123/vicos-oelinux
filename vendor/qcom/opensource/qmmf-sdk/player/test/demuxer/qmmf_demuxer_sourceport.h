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

/* ============================================================================
**               Include files for MM_MediaSourcePort.h
** ========================================================================== */
#include <stdio.h>
#include <AEEStdDef.h>
#include "DataSourcePort.h"
#include "MMTimer.h"
#include "MMSignal.h"
#include "MMThread.h"
#include "MMDebugMsg.h"
#include "MMMalloc.h"
#include "MMMemory.h"
#include "MMFile.h"
#include <wchar.h>
#include <stdlib.h>

#ifdef WIN32
#define ssize_t int
#define int32   int
#endif

using namespace video;

/* ============================================================================
 **  Constant / Macro Declarations
 ** ========================================================================== */
#define  MM_MEDIA_STRMPORT_DEFAULT_BPS 384000
#define  MM_MEDIA_STREAMPORT_THREAD_STACK_SIZE (16*1024)

 /* ============================================================================
 ** Global Data Declarations
 ** ========================================================================== */

 /* ============================================================================
 *** Forward Declarations
 ** ========================================================================== */

/* ============================================================================
 **                        Class & Function Declarations
 *** ======================================================================= */

/**
 * CMM_MediaSourcePort
 * The iSourcePort interface is a generic data source interface that provides
 * uni-directional data transfer (only read). This is designed to be generic
 * across all kinds of data sources (e.g. File Source, Network Source etc).
 */
class CMM_MediaSourcePort : public video::iStreamPort {
 public:
  explicit CMM_MediaSourcePort(char*,
                               int64 bps = MM_MEDIA_STRMPORT_DEFAULT_BPS);
  ~CMM_MediaSourcePort();

  void* QueryInterface(const AEEIID iid);
  uint32 AddRef();
  uint32 Release();

  /**
   * Read data into the specified buffer (of given size).
   */
    DataSourceReturnCode Read(
      /*rout*/ unsigned char* pBuf,
      /*in*/   ssize_t nBufSize,
      /*rout*/ ssize_t* pnRead);

  /**
   * Read data into the specified buffer (of given size) from given Offset.
   */

    DataSourceReturnCode ReadAt(
      /*rout*/ unsigned char* pBuf,
      /*in*/ ssize_t nBufSize,
      /*in*/ const int64 nOffset,
      /*in*/ const int nWhence,
      /*rout*/ ssize_t* pnRead);

  /**
   * Register a callback interface to be invoked when data is available to be
   * read (i.e. Read() would return something other than DS_WAIT).
   */
    DataSourceReturnCode Readable(
      /*in*/ iReadNotificationHandler const* pNotificationHandler);

  /**
   * Write data from the specified buffer (of given size).
   */
   DataSourceReturnCode Write(
      /*in*/ const unsigned char* pBuf,
      /*in*/ ssize_t nBufSize,
      /*rout*/ ssize_t* pnWritten);

   virtual DataSourceReturnCode WriteBlockData(
      /*in*/   const unsigned char* /*pBuf*/,
      /*in*/   int /*nBufSize*/,
      /*in*/   int /*nTimeStamp*/,
      /*in*/   bool /*bEOF*/,
      /*rout*/ int* /*pnWritten*/)
    {
      return video::iSourcePort::DS_FAILURE;
    };

  /**
   * Repositions the read/write point in an open port to the specified offset.
   * pnOutOffset gives the offset after a successful seek.
    */
   DataSourceReturnCode Seek(
      /*in*/ const int64 nOffset,
      /*in*/ const int32 nWhence,
      /*rout*/ int64* pnOutOffset);

  /**
   * Close the data source port - port becomes unusable after this call.
   */
   DataSourceReturnCode Close();

  /**
   * Get the content length (in bytes).
   */
   DataSourceReturnCode GetContentLength(
      /*rout*/ int64* pContentLength);

  /**
   * Get the underlying data source type.
   */
   DataSourceReturnCode GetSourceType(
      /*rout*/ DataSourceType* pSourceType);

  /**
   * Get the current available data size (in bytes). e.g. number of bytes
   * downloaded for a network source.
   */
   DataSourceReturnCode GetNumBytesAvailable(
     /*rout*/ int64* pNumBytesAvailable);

   DataSourceReturnCode GetAvailableOffset(
     /*rout*/ int64* pAvailableOffset,
     /*rout*/ bool* pbEOS);

   virtual DataSourceReturnCode GetBufferLowerBound(
     /*rout*/ int64* /*pAvailableOffset*/,
     /*rout*/ bool* /*pbEOS*/)
   {
      return DS_FAILURE;
   };

   DataSourceReturnCode GetStartOffset(int64* pStartOffset);

 private:

  int64       m_llContentLength;
  int64       m_llBytesAvailable;
  int64       m_llBps;
  uint32      m_ulRefCnt;
  bool        m_bRun;
  MM_HANDLE   m_pFilePtr;
  MM_HANDLE   m_pThreadHnd;

  static int MM_MediaStrmPortThreadEntry(void* ptr);
  void MediaStreamPortThread(void);

};  /*CMM_MediaSourcePort*/