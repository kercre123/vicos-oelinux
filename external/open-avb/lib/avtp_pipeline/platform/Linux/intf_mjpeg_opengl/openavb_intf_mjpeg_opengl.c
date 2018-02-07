/*
* Copyright (c) 2016, The Linux Foundation. All rights reserved.
*/

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
* MODULE SUMMARY : MJPEG OpenGL interface module.
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include "openavb_types_pub.h"
#include "openavb_trace_pub.h"
#include "openavb_mediaq_pub.h"
#include "openavb_intf_pub.h"
#include "openavb_map_mjpeg_pub.h"
#include "libmjpegavbsink.h"

#define	AVB_LOG_COMPONENT	"MJPEG OpenGL Interface"
#include "openavb_log_pub.h"

#define APPSINK_NAME "avbsink"
#define APPSRC_NAME "avbsrc"
#define PACKETS_PER_RX_CALL 20

#define FRAME_SIZE 600000 //600KB big enough for a frame?


typedef struct pvt_data_t
{
	char *file_name;
	bool ignoreTimestamp;
	U8 *fp;
	U32 bufwr;
	U32 bufrd;
	U32 seq;
	U32 bitrate;
	U8 rec_frame[FRAME_SIZE];
	bool asyncRx;
	bool blockingRx;
	struct stat statbuf;
	U32 read_size;
	U32 loc;
	bool find_soi;
	int fd;
	bool get_avtp_timestamp;        /*<! this flag indicates whether
                                        an avtp timestamp should be taken */
	U32 frame_timestamp;            /*<! this is a timestamp of a video frame */
} pvt_data_t;

// Each configuration name value pair for this mapping will result in this callback being called.
void openavbIntfMjpegOpenglCfgCB(media_q_t *pMediaQ, const char *name, const char *value)
{
	if (!pMediaQ) {
		AVB_LOG_DEBUG("mjpeg-file cfgCB: no mediaQ!");
		return;
	}

	char *pEnd;
	long tmp;

	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
	if (!pPvtData) {
		AVB_LOG_ERROR("Private interface module data not allocated.");
		return;
	}

	pPvtData->asyncRx = FALSE;

	if (strcmp(name, "intf_nv_file_name") == 0) {
		if (pPvtData->file_name) {
			free(pPvtData->file_name);
		}
		pPvtData->file_name = strdup(value);
	}
	else if (strcmp(name, "intf_nv_async_rx") == 0) {
		tmp = strtol(value, &pEnd, 10);
		if (*pEnd == '\0' && tmp == 1) {
			pPvtData->asyncRx = (tmp == 1);
		}
	}
	else if (strcmp(name, "intf_nv_blocking_rx") == 0) {
		tmp = strtol(value, &pEnd, 10);
		if (*pEnd == '\0' && tmp == 1) {
			pPvtData->blockingRx = (tmp == 1);
		}
	}
	else if (strcmp(name, "intf_nv_ignore_timestamp") == 0) {
		tmp = strtol(value, &pEnd, 10);
		if (*pEnd == '\0' && tmp == 1) {
			pPvtData->ignoreTimestamp = (tmp == 1);
		}
	}
	else if (strcmp(name, "intf_nv_bitrate") == 0) {
		/*tmp = strtol(value, &pEnd, 10);
		if (*pEnd == '\0' && tmp == 1)
		{
			pPvtData->bitrate = atoi(tmp) ;
		}*/
		pPvtData->bitrate = atoi(value);
	}
	pPvtData->fd = 0;
}

void openavbIntfMjpegOpenglGenInitCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);
	if (!pMediaQ) {
		AVB_LOG_DEBUG("mjpeg-file initCB: no mediaQ!");
		AVB_TRACE_EXIT(AVB_TRACE_INTF);
		return;
	}
	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}



// A call to this callback indicates that this interface module will be
// a listener. Any listener initialization can be done in this function.
void openavbIntfMjpegOpenglRxInitCB(media_q_t *pMediaQ)
{
	AVB_LOG_DEBUG("Rx Init callback.");
	if (!pMediaQ) {
		AVB_LOG_DEBUG("No MediaQ in MjpegFileRxInitCB");
		return;
	}

	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
	if (!pPvtData) {
		AVB_LOG_ERROR("Private interface module data not allocated.");
		return;
	}

	if (!pPvtData->fd) {
		mjpeg_avb_sink_init();
		pPvtData->fd = 1;
	}
}

// This callback is called when acting as a listener.
bool openavbIntfMjpegOpenglRxCB(media_q_t *pMediaQ)
{
	if (!pMediaQ) {
		AVB_LOG_DEBUG("RxCB: no mediaQ!");
		return TRUE;
	}
	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
	if (!pPvtData) {
		AVB_LOG_ERROR("Private interface module data not allocated.");
		return FALSE;
	}

	bool moreSourcePackets = TRUE;

	while (moreSourcePackets) {
		media_q_item_t *pMediaQItem = openavbMediaQTailLock(pMediaQ, pPvtData->ignoreTimestamp);
		// there are no packets available or they are from the future
		if (!pMediaQItem) {
			moreSourcePackets = FALSE;
			continue;
		}
		if (!pMediaQItem->dataLen) {
			AVB_LOG_DEBUG("No dataLen");
			openavbMediaQTailPull(pMediaQ);
			continue;
		}

		mjpeg_avb_sink_buf(pMediaQItem->pPubData, pMediaQItem->dataLen,
				((media_q_item_map_mjpeg_pub_data_t *)pMediaQItem->pPubMapData)->lastFragment);

		if (((media_q_item_map_mjpeg_pub_data_t *)pMediaQItem->pPubMapData)->lastFragment) {
			pPvtData->get_avtp_timestamp = TRUE;
			//in the case of a deocder, pass it up the timestamp as well
		}
		else {
			if (pPvtData->get_avtp_timestamp) {
				pPvtData->frame_timestamp = openavbAvtpTimeGetAvtpTimestamp(pMediaQItem->pAvtpTime);
				pPvtData->get_avtp_timestamp = FALSE;
			}
			else {
				U32 fragment_timestamp = openavbAvtpTimeGetAvtpTimestamp(pMediaQItem->pAvtpTime);
				// all fragments should have the same timestamp
				if (pPvtData->frame_timestamp != fragment_timestamp) {
					AVB_LOGF_ERROR("Mapping is wrong. Fragment timestamp should be %" PRIu32 " instead of %" PRIu32,
					               pPvtData->frame_timestamp, fragment_timestamp);
				}
			}

		}
		openavbMediaQTailPull(pMediaQ);

	}
	return TRUE;
}

// This callback will be called when the interface needs to be closed. All shutdown should
// occur in this function.
void openavbIntfMjpegOpenglEndCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
	if (!pPvtData) {
		AVB_LOG_ERROR("Private interface module data not allocated.");
		return;
	}

	mjpeg_avb_sink_end();
	pPvtData->fd = 0;

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

void openavbIntfMjpegOpenglGenEndCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);
	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

// Main initialization entry point into the interface module
extern DLL_EXPORT bool openavbIntfMjpegOpenglInitialize(media_q_t *pMediaQ, openavb_intf_cb_t *pIntfCB)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	if (!pMediaQ) {
		AVB_LOG_DEBUG("mjpeg-gst GstInitialize: no mediaQ!");
		AVB_TRACE_EXIT(AVB_TRACE_INTF);
		return TRUE;
	}
	// Memory freed by the media queue when the media queue is destroyed.
	pMediaQ->pPvtIntfInfo = calloc(1, sizeof(pvt_data_t));

	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;

	pPvtData->get_avtp_timestamp = TRUE;

	pIntfCB->intf_cfg_cb = openavbIntfMjpegOpenglCfgCB;
	pIntfCB->intf_gen_init_cb = openavbIntfMjpegOpenglGenInitCB;
	pIntfCB->intf_rx_init_cb = openavbIntfMjpegOpenglRxInitCB;
	pIntfCB->intf_rx_cb = openavbIntfMjpegOpenglRxCB;
	pIntfCB->intf_end_cb = openavbIntfMjpegOpenglEndCB;
	pIntfCB->intf_gen_end_cb = openavbIntfMjpegOpenglGenEndCB;

	pPvtData->ignoreTimestamp = FALSE;

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
	return TRUE;
}
