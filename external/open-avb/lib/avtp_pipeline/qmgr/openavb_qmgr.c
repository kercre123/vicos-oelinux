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

/*
 * MODULE : AVB Queue Manager
 */

#include <openavb_types.h>
#define AVB_LOG_COMPONENT "QMGR"
//#define AVB_LOG_LEVEL AVB_LOG_LEVEL_DEBUG
#include "openavb_log.h"
#include "openavb_trace.h"

#include "openavb_qmgr.h"
#include "avb_sched.h"

#if AVB_FEATURE_NEUTRINO
#include "neutrino.h"
#endif

#ifdef USE_GLIB
#include <glib.h>
#define strlcpy g_strlcpy
#define strlcat g_strlcat
#endif

#define AVB_DEFAULT_QDISC_MODE AVB_SHAPER_HWQ_PER_CLASS

// We have a singleton Qmgr, so we use file-static data here

// Qdisc configuration
typedef struct {
	int mode;
	int ifindex;
	char ifname[IFNAMSIZ];
	U32 linkKbit;
	U32 nsrKbit;
	U32 linkMTU;
	int ref;
} qdisc_data_t;

static qdisc_data_t qdisc_data;

// We do get accessed from multiple threads, so need a mutex
pthread_mutex_t qmgr_mutex = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
#define LOCK()  	pthread_mutex_lock(&qmgr_mutex)
#define UNLOCK()	pthread_mutex_unlock(&qmgr_mutex)

// Information for each SR class
typedef struct {
	unsigned classBytesPerSec;
} qmgrClass_t;

// Information for each active stream
typedef struct {
	unsigned streamBytesPerSec;
	unsigned classRate;
	unsigned maxIntervalFrames;
	unsigned maxFrameSize;
} qmgrStream_t;

// Arrays to hold info for classes and streams
static qmgrClass_t  qmgr_classes[MAX_AVB_SR_CLASSES];
static qmgrStream_t qmgr_streams[MAX_AVB_STREAMS];

// Make sure that the scheme we're using to encode the class/stream
// into the fwmark will work!  (we encode class and stream into 16-bits)
#if MAX_AVB_STREAMS_PER_CLASS > (1 << TC_AVB_CLASS_SHIFT)
#error MAX_AVB_STREAMS_PER_CLASS too large for FWMARK encoding
#endif

static bool setupHWQueue(int nClass, unsigned classBytesPerSec)
{
	int err = 0;
	AVB_TRACE_ENTRY(AVB_TRACE_QUEUE_MANAGER);

#if AVB_FEATURE_NEUTRINO
	AVB_LOGF_DEBUG("setupHWQueue: interface = %s", qdisc_data.ifname);
	err = ntn_set_class_bandwidth(nClass, classBytesPerSec, qdisc_data.ifname);
#endif

	if (err)
		AVB_LOGF_ERROR("Adding stream; set_class_bandwidth failed: %s", strerror(err));

	AVB_TRACE_EXIT(AVB_TRACE_QUEUE_MANAGER);
	return !err;
}

/* Add a stream.
 *
 * 	nClass = index of class (A, B, etc.)
 * 	classRate = class observation interval (8000, 4000, etc.)
 * 	maxIntervalFrames = frames per interval
 * 	maxFrameSize = max size of frames
 *
 */
U16 openavbQmgrAddStream(SRClassIdx_t nClass, unsigned classRate, unsigned maxIntervalFrames, unsigned maxFrameSize)
{
	unsigned fullFrameSize = maxFrameSize + OPENAVB_AVTP_ETHER_FRAME_OVERHEAD;
	unsigned long streamBytesPerSec = fullFrameSize * maxIntervalFrames * classRate;
	int idx, nStream;
	U16 fwmark = INVALID_FWMARK;

	AVB_TRACE_ENTRY(AVB_TRACE_QUEUE_MANAGER);
	LOCK();

	AVB_LOGF_DEBUG("Adding stream; class=%d, rate=%u frames=%u, size=%u(%u), bytes/sec=%lu",
				   nClass, classRate, maxIntervalFrames, maxFrameSize,
				   fullFrameSize, streamBytesPerSec);

	if ((int)nClass < 0 || nClass >= MAX_AVB_SR_CLASSES || streamBytesPerSec == 0) {
		AVB_LOG_ERROR("Adding stream; invalid argument");
	}
	else {
		// Find an unused stream in the appropriate SR class
		for (nStream = 0, idx = nClass * MAX_AVB_STREAMS_PER_CLASS; nStream < MAX_AVB_STREAMS_PER_CLASS; nStream++, idx++) {
			if (0 == qmgr_streams[idx].streamBytesPerSec) {
				fwmark = TC_AVB_MARK(nClass, nStream);
				break;
			}
		}

		if (fwmark == INVALID_FWMARK) {
			AVB_LOGF_ERROR("Adding stream; too many streams in class %d", nClass);
		} else {

			if (qdisc_data.mode != AVB_SHAPER_DISABLED) {
				if (!setupHWQueue(nClass, qmgr_classes[nClass].classBytesPerSec + streamBytesPerSec)) {
					fwmark = INVALID_FWMARK;
				}
			}

			if (fwmark != INVALID_FWMARK) {
				// good to go - update stream
				qmgr_streams[idx].streamBytesPerSec = streamBytesPerSec;
				qmgr_streams[idx].classRate = classRate;
				qmgr_streams[idx].maxIntervalFrames = maxIntervalFrames;
				qmgr_streams[idx].maxFrameSize = maxFrameSize;
				// and class
				qmgr_classes[nClass].classBytesPerSec += streamBytesPerSec;

				AVB_LOGF_DEBUG("Added stream; classBPS=%u, streamBPS=%u", qmgr_classes[nClass].classBytesPerSec, qmgr_streams[idx].streamBytesPerSec);
			}
		}
	}

	UNLOCK();
	AVB_TRACE_EXIT(AVB_TRACE_QUEUE_MANAGER);
	return fwmark;
}


void openavbQmgrRemoveStream(U16 fwmark)
{
	AVB_TRACE_ENTRY(AVB_TRACE_QUEUE_MANAGER);

	if (fwmark == INVALID_FWMARK) {
		AVB_LOG_ERROR("Removing stream; invalid argument");
		AVB_TRACE_EXIT(AVB_TRACE_QUEUE_MANAGER);
		return;
	}

	int nClass = TC_AVB_MARK_CLASS(fwmark);
	int nStream  = TC_AVB_MARK_STREAM(fwmark);
	int idx = nClass * MAX_AVB_STREAMS_PER_CLASS + nStream;

	LOCK();

	if (nStream < 0
		|| nStream >= MAX_AVB_STREAMS
		|| nClass < 0
		|| nClass >= MAX_AVB_SR_CLASSES
		|| idx >= MAX_AVB_STREAMS
		|| qmgr_streams[idx].streamBytesPerSec == 0)
	{
		// something is wrong
		AVB_LOG_ERROR("Removing stream; invalid argument or data");
	}
	else {
		if (qdisc_data.mode != AVB_SHAPER_DISABLED) {
			setupHWQueue(nClass, qmgr_classes[nClass].classBytesPerSec - qmgr_streams[idx].streamBytesPerSec);
		}

		// update class
		qmgr_classes[nClass].classBytesPerSec -= qmgr_streams[idx].streamBytesPerSec;
		AVB_LOGF_DEBUG("Removed strea; classBPS=%u, streamBPS=%u", qmgr_classes[nClass].classBytesPerSec, qmgr_streams[idx].streamBytesPerSec);
		// and stream
		memset(&qmgr_streams[idx], 0, sizeof(qmgrStream_t));
	}

	UNLOCK();
	AVB_TRACE_EXIT(AVB_TRACE_QUEUE_MANAGER);
}

bool openavbQmgrInitialize(int mode, int ifindex, const char* ifname, unsigned mtu, unsigned link_kbit, unsigned nsr_kbit)
{
	AVB_TRACE_ENTRY(AVB_TRACE_QUEUE_MANAGER);
	bool ret = FALSE;

	if (!ifname) {
		AVB_LOG_ERROR("Initializing QMgr; invalid argument");
		return FALSE;
	}

	LOCK();

	if (qdisc_data.ref++ > 0) {
		AVB_LOG_DEBUG("Already initialized");

		UNLOCK();
		AVB_TRACE_EXIT(AVB_TRACE_QUEUE_MANAGER);
		return TRUE;
	}

	if (mode >= 0)
		qdisc_data.mode = mode;
	else
		qdisc_data.mode = AVB_DEFAULT_QDISC_MODE;


	AVB_LOGF_DEBUG("Initializing QMgr; mode=%d, idx=%d, mtu=%u, link_kbit=%u, nsr_kbit=%u",
				   qdisc_data.mode, ifindex, mtu, link_kbit, nsr_kbit);

	// Initialize data for classes and streams
	memset(qmgr_classes, 0, sizeof(qmgr_classes));
	memset(qmgr_streams, 0, sizeof(qmgr_streams));

	// Save the configuration
	if (ifname)
		strlcpy(qdisc_data.ifname, ifname, IFNAMSIZ);

	AVB_LOGF_DEBUG("Initializing QMgr; interface=%s",qdisc_data.ifname);

	qdisc_data.ifindex = ifindex;
	qdisc_data.linkKbit = link_kbit;
	qdisc_data.linkMTU = mtu;
	qdisc_data.nsrKbit  = nsr_kbit;

	ret = TRUE;

	UNLOCK();
	AVB_TRACE_EXIT(AVB_TRACE_QUEUE_MANAGER);
	return ret;
}

void openavbQmgrFinalize(void)
{
	AVB_TRACE_ENTRY(AVB_TRACE_QUEUE_MANAGER);
	LOCK();

	if (--qdisc_data.ref == 0 && qdisc_data.mode != AVB_SHAPER_DISABLED) {
		int nClass;
		for (nClass = SR_CLASS_A; nClass < MAX_AVB_SR_CLASSES; nClass++) {
			int nStream, idx;
			for (nStream = 0, idx = nClass * MAX_AVB_STREAMS_PER_CLASS; nStream < MAX_AVB_STREAMS_PER_CLASS; nStream++, idx++) {
				if (qmgr_streams[idx].streamBytesPerSec) {
					U16 fwmark = TC_AVB_MARK(nClass, nStream);
					openavbQmgrRemoveStream(fwmark);
				}
			}
		}
	}

	UNLOCK();
	AVB_TRACE_EXIT(AVB_TRACE_QUEUE_MANAGER);
}
