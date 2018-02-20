/*************************************************************************************************************
Copyright (c) 2012-2015, Symphony Teleca Corporation, a Harman International Industries, Incorporated company
All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS LISTED "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS LISTED BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
Attributions: The inih library portion of the source code is licensed from 
Brush Technology and Ben Hoyt - Copyright (c) 2009, Brush Technology and Copyright (c) 2009, Ben Hoyt. 
Complete license and copyright information can be found at 
https://github.com/benhoyt/inih/commit/74d2ca064fb293bc60a77b0bd068075b293cf175.
*************************************************************************************************************/

#include "openavb_types_pub.h"
#include "openavb_platform_pub.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "openavb_queue.h"

#include "openavb_log.h"

#define LOG_EXTRA_NEWLINE 1

#ifdef USE_GLIB
#include <glib.h>
#define strlcpy g_strlcpy
#define strlcat g_strlcat
#endif

typedef struct {
	U8 msg[LOG_QUEUE_MSG_SIZE];
  	bool bRT;						// TRUE = Details are in RT queue
} log_queue_item_t;

typedef struct {
	char *pFormat;
	log_rt_datatype_t dataType;
	union {
		struct timespec nowTS;
		U16 unsignedShortVar;
		S16 signedShortVar;
		U32 unsignedLongVar;
		S32 signedLongVar;
		U64 unsignedLongLongVar;
		S64 signedLongLongVar;
		float floatVar;
	} data;
	bool bEnd;
} log_rt_queue_item_t;

static openavb_queue_t logQueue;
static openavb_queue_t logRTQueue;

static char msg[LOG_MSG_LEN] = "";
static char time_msg[LOG_TIME_LEN] = "";
static char timestamp_msg[LOG_TIMESTAMP_LEN] = "";
static char file_msg[LOG_FILE_LEN] = "";
static char proc_msg[LOG_PROC_LEN] = "";
static char thread_msg[LOG_THREAD_LEN] = "";
static char full_msg[LOG_FULL_MSG_LEN] = "";

static char rt_msg[LOG_RT_MSG_LEN] = "";

static bool loggingThreadRunning = false;
extern void *loggingThreadFn(void *pv);
THREAD_TYPE(loggingThread);
THREAD_DEFINITON(loggingThread);

static MUTEX_HANDLE_ALT(gLogMutex);
#define LOG_LOCK() MUTEX_LOCK_ALT(gLogMutex)
#define LOG_UNLOCK() MUTEX_UNLOCK_ALT(gLogMutex)

void avbLogRTRender(log_queue_item_t *pLogItem)
{
	if (logRTQueue) {
	  	pLogItem->msg[0] = 0x00;
		bool bMore = TRUE;
		while (bMore) {
			openavb_queue_elem_t elem = openavbQueueTailLock(logRTQueue);
			if (elem) {
				log_rt_queue_item_t *pLogRTItem = (log_rt_queue_item_t *)openavbQueueData(elem);
								
				switch (pLogRTItem->dataType) {
					case LOG_RT_DATATYPE_CONST_STR:
						strlcat((char *)pLogItem->msg, pLogRTItem->pFormat, LOG_QUEUE_MSG_SIZE);
						break;
					case LOG_RT_DATATYPE_NOW_TS:
						snprintf(rt_msg, LOG_RT_MSG_LEN, "[%lu:%09lu] ", pLogRTItem->data.nowTS.tv_sec, pLogRTItem->data.nowTS.tv_nsec);
						strlcat((char *)pLogItem->msg, rt_msg, LOG_QUEUE_MSG_SIZE);
						break;
					case LOG_RT_DATATYPE_U16:
						snprintf(rt_msg, LOG_RT_MSG_LEN, pLogRTItem->pFormat, pLogRTItem->data.unsignedShortVar);
						strlcat((char *)pLogItem->msg, rt_msg, LOG_QUEUE_MSG_SIZE);
						break;
					case LOG_RT_DATATYPE_S16:
						snprintf(rt_msg, LOG_RT_MSG_LEN, pLogRTItem->pFormat, pLogRTItem->data.signedShortVar);
						strlcat((char *)pLogItem->msg, rt_msg, LOG_QUEUE_MSG_SIZE);
						break;
					case LOG_RT_DATATYPE_U32:
						snprintf(rt_msg, LOG_RT_MSG_LEN, pLogRTItem->pFormat, pLogRTItem->data.unsignedLongVar);
						strlcat((char *)pLogItem->msg, rt_msg, LOG_QUEUE_MSG_SIZE);
						break;
					case LOG_RT_DATATYPE_S32:
						snprintf(rt_msg, LOG_RT_MSG_LEN, pLogRTItem->pFormat, pLogRTItem->data.signedLongVar);
						strlcat((char *)pLogItem->msg, rt_msg, LOG_QUEUE_MSG_SIZE);
						break;
					case LOG_RT_DATATYPE_U64:
						snprintf(rt_msg, LOG_RT_MSG_LEN, pLogRTItem->pFormat, pLogRTItem->data.unsignedLongLongVar);
						strlcat((char *)pLogItem->msg, rt_msg, LOG_QUEUE_MSG_SIZE);
						break;
					case LOG_RT_DATATYPE_S64:
						snprintf(rt_msg, LOG_RT_MSG_LEN, pLogRTItem->pFormat, pLogRTItem->data.signedLongLongVar);
						strlcat((char *)pLogItem->msg, rt_msg, LOG_QUEUE_MSG_SIZE);
						break;
					case LOG_RT_DATATYPE_FLOAT:
						snprintf(rt_msg, LOG_RT_MSG_LEN, pLogRTItem->pFormat, pLogRTItem->data.floatVar);
						strlcat((char *)pLogItem->msg, rt_msg, LOG_QUEUE_MSG_SIZE);
						break;
					default:
						break;
				}

				if (pLogRTItem->bEnd) {
					if (LOG_EXTRA_NEWLINE)
						strlcat((char *)pLogItem->msg, "\n", LOG_QUEUE_MSG_SIZE);
					bMore = FALSE;
				}
				openavbQueueTailPull(logRTQueue);
			}
		  
		}
	}
}

extern U32 DLL_EXPORT avbLogGetMsg(U8 *pBuf, U32 bufSize)
{
	U32 dataLen = 0;
	if (logQueue) {
		openavb_queue_elem_t elem = openavbQueueTailLock(logQueue);
		if (elem) {
			log_queue_item_t *pLogItem = (log_queue_item_t *)openavbQueueData(elem);
			
			if (pLogItem->bRT)
				avbLogRTRender(pLogItem);
			
			dataLen = strlen((const char *)pLogItem->msg);
			if (dataLen <= bufSize)
				memcpy(pBuf, (U8 *)pLogItem->msg, dataLen);
			else
			  	memcpy(pBuf, (U8 *)pLogItem->msg, bufSize);
			openavbQueueTailPull(logQueue);
			return dataLen;
		}
	}
	return dataLen;
}

void *loggingThreadFn(void *pv)
{
	// The do/while loop ensures that this loop is executed at least once
	// even if loggingThreadRunning is set to false during startup
	// (as in the case of an initialization error).
	do {
		SLEEP_MSEC(LOG_QUEUE_SLEEP_MSEC);

		bool more = TRUE;

		while (more) {
			more = FALSE;
			openavb_queue_elem_t elem = openavbQueueTailLock(logQueue);
			if (elem) {
				log_queue_item_t *pLogItem = (log_queue_item_t *)openavbQueueData(elem);
				
				if (pLogItem->bRT)
					avbLogRTRender(pLogItem);
				
				fputs((const char *)pLogItem->msg, AVB_LOG_OUTPUT_FD);
				openavbQueueTailPull(logQueue);
				more = TRUE;
			}
		}
	} while (loggingThreadRunning);

	return NULL;
}

extern void DLL_EXPORT avbLogInit(void)
{
	MUTEX_CREATE_ALT(gLogMutex);
  
	logQueue = openavbQueueNewQueue(sizeof(log_queue_item_t), LOG_QUEUE_MSG_CNT);
	if (!logQueue) {
		printf("Failed to initialize logging facility\n");
	}
	
	logRTQueue = openavbQueueNewQueue(sizeof(log_rt_queue_item_t), LOG_RT_QUEUE_CNT);
	if (!logRTQueue) {
		printf("Failed to initialize logging RT facility\n");
	}

	// Start the logging task
	if (OPENAVB_LOG_FROM_THREAD) {
		bool errResult;
		loggingThreadRunning = true;
		THREAD_CREATE(loggingThread, loggingThread, NULL, loggingThreadFn, NULL);
		THREAD_CHECK_ERROR(loggingThread, "Thread / task creation failed", errResult);
		if (errResult);		// Already reported
	}
}

extern void DLL_EXPORT avbLogExit()
{
	if (OPENAVB_LOG_FROM_THREAD) {
		loggingThreadRunning = false;
		THREAD_JOIN(loggingThread, NULL);
	}
}

extern void DLL_EXPORT avbLogFn(
	int level, 
	const char *tag, 
	const char *company,
	const char *component,
	const char *path,
	int line,
	const char *fmt, 
	...)
{
	if (level <= AVB_LOG_LEVEL) {
		va_list args;
		va_start(args, fmt);

		LOG_LOCK();

		vsnprintf(msg, LOG_MSG_LEN, fmt, args);

		if (OPENAVB_LOG_FILE_INFO && path) {
			char* file = strrchr(path, '/');
			if (!file)
				file = strrchr(path, '\\');
			if (file)
				file += 1;
			else
				file = (char*)path;
			snprintf(file_msg, LOG_FILE_LEN, " %s:%d", file, line);
		}
		if (OPENAVB_LOG_PROC_INFO) {
			snprintf(proc_msg, LOG_PROC_LEN, " P:%5.5d", GET_PID());
		}
		if (OPENAVB_LOG_THREAD_INFO) {
			snprintf(thread_msg, LOG_THREAD_LEN, " T:%lu", THREAD_SELF());
		}
		if (OPENAVB_LOG_TIME_INFO) {
			time_t tNow = time(NULL);
			struct tm tmNow;
			localtime_r(&tNow, &tmNow);

			snprintf(time_msg, LOG_TIME_LEN, "%2.2d:%2.2d:%2.2d", tmNow.tm_hour, tmNow.tm_min, tmNow.tm_sec);
		}
		if (OPENAVB_LOG_TIMESTAMP_INFO) {
			struct timespec nowTS;
			CLOCK_GETTIME(OPENAVB_CLOCK_REALTIME, &nowTS);

			snprintf(timestamp_msg, LOG_TIMESTAMP_LEN, "%lu:%09lu", nowTS.tv_sec, nowTS.tv_nsec);
		}

		// using sprintf and puts allows using static buffers rather than heap.
		if (LOG_EXTRA_NEWLINE)
			/* S32 full_msg_len = */ snprintf(full_msg, LOG_FULL_MSG_LEN, "[%s%s%s%s %s %s%s] %s: %s\n", time_msg, timestamp_msg, proc_msg, thread_msg, company, component, file_msg, tag, msg);
		else
			/* S32 full_msg_len = */ snprintf(full_msg, LOG_FULL_MSG_LEN, "[%s%s%s%s %s %s%s] %s: %s", time_msg, timestamp_msg, proc_msg, thread_msg, company, component, file_msg, tag, msg);

		if (!OPENAVB_LOG_FROM_THREAD && !OPENAVB_LOG_PULL_MODE) {
			fputs(full_msg, AVB_LOG_OUTPUT_FD);
		}
		else {
			if (logQueue) {
				openavb_queue_elem_t elem = openavbQueueHeadLock(logQueue);
				if (elem) {
					log_queue_item_t *pLogItem = (log_queue_item_t *)openavbQueueData(elem);
					pLogItem->bRT = FALSE;
					strlcpy((char *)pLogItem->msg, full_msg, LOG_QUEUE_MSG_SIZE);
					openavbQueueHeadPush(logQueue);
				}
			}
		}

		va_end(args);

		LOG_UNLOCK();
	}
}

extern void DLL_EXPORT avbLogRT(int level, bool bBegin, bool bItem, bool bEnd, char *pFormat, log_rt_datatype_t dataType, void *pVar)
{
	if (level <= AVB_LOG_LEVEL) {
		if (logRTQueue) {
			if (bBegin) {
				LOG_LOCK();

				openavb_queue_elem_t elem = openavbQueueHeadLock(logRTQueue);
				if (elem) {
					log_rt_queue_item_t *pLogRTItem = (log_rt_queue_item_t *)openavbQueueData(elem);
					pLogRTItem->bEnd = FALSE;
					pLogRTItem->pFormat = NULL;
					pLogRTItem->dataType = LOG_RT_DATATYPE_NOW_TS;
					CLOCK_GETTIME(OPENAVB_CLOCK_REALTIME, &pLogRTItem->data.nowTS);
					openavbQueueHeadPush(logRTQueue);
				}
			}

			if (bItem) {
				openavb_queue_elem_t elem = openavbQueueHeadLock(logRTQueue);
				if (elem) {
					log_rt_queue_item_t *pLogRTItem = (log_rt_queue_item_t *)openavbQueueData(elem);
					if (bEnd)
						pLogRTItem->bEnd = TRUE;
					else
						pLogRTItem->bEnd = FALSE;
					pLogRTItem->pFormat = pFormat;
					pLogRTItem->dataType = dataType;

					switch (pLogRTItem->dataType) {
						case LOG_RT_DATATYPE_CONST_STR:
					  		break;
						case LOG_RT_DATATYPE_U16:
							pLogRTItem->data.unsignedLongVar = *(U16 *)pVar;
							break;
						case LOG_RT_DATATYPE_S16:
							pLogRTItem->data.signedLongVar = *(S16 *)pVar;
							break;
						case LOG_RT_DATATYPE_U32:
							pLogRTItem->data.unsignedLongVar = *(U32 *)pVar;
							break;
						case LOG_RT_DATATYPE_S32:
							pLogRTItem->data.signedLongVar = *(S32 *)pVar;
							break;
						case LOG_RT_DATATYPE_U64:
							pLogRTItem->data.unsignedLongLongVar = *(U64 *)pVar;
							break;
						case LOG_RT_DATATYPE_S64:
							pLogRTItem->data.signedLongLongVar = *(S64 *)pVar;
							break;
						case LOG_RT_DATATYPE_FLOAT:
							pLogRTItem->data.floatVar = *(float *)pVar;
							break;
						default:
							break;
					}
					openavbQueueHeadPush(logRTQueue);
				}
			}
			
			if (!bItem && bEnd) {
				openavb_queue_elem_t elem = openavbQueueHeadLock(logRTQueue);
				if (elem) {
					log_rt_queue_item_t *pLogRTItem = (log_rt_queue_item_t *)openavbQueueData(elem);
					pLogRTItem->bEnd = TRUE;
					pLogRTItem->pFormat = NULL;
					pLogRTItem->dataType = LOG_RT_DATATYPE_NONE;
					openavbQueueHeadPush(logRTQueue);
				}
			}
			
			if (bEnd) {
				if (logQueue) {
					openavb_queue_elem_t elem = openavbQueueHeadLock(logQueue);
					if (elem) {
						log_queue_item_t *pLogItem = (log_queue_item_t *)openavbQueueData(elem);
						pLogItem->bRT = TRUE;
						if (OPENAVB_LOG_FROM_THREAD) {
							openavbQueueHeadPush(logQueue);
						} else {
							avbLogRTRender(pLogItem);
							fputs((const char *)pLogItem->msg, AVB_LOG_OUTPUT_FD);
							openavbQueueHeadUnlock(logQueue);
						}
					}
				}
			  
				LOG_UNLOCK();
			}
		}
	}
}

