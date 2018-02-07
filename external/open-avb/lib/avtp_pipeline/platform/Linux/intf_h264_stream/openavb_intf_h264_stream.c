/*
* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include "openavb_types_pub.h"
#include "openavb_trace_pub.h"
#include "openavb_mediaq_pub.h"
#include "openavb_intf_pub.h"
#include "openavb_map_h264_pub.h"

#define	AVB_LOG_COMPONENT	"H264 stream Interface"
#include "openavb_log_pub.h"
#include "Avbh264Stream.h"


#define NBUFS 256
#define MAX_READ_SIZE 192

#define DEFAULT_WIDTH 1920
#define DEFAULT_HEIGHT 1080
#define DEFAULT_FRAMERATE 30

typedef struct pvt_data_t
{
	char *file_name;
	bool ignoreTimestamp;
	U8 *fp;
	char *pPipelineStr;
	U32 bufwr;
	U32 bufrd;
	U32 seq;
	bool asyncRx;
	bool blockingRx;
	U32 read_size;
	U32 loc;
	int fd;
    struct stat statbuf;
	bool get_avtp_timestamp;        /*<! this flag indicates whether
                                        an avtp timestamp should be taken */
	U32 frame_timestamp;            /*<! this is a timestamp of a video frame */
	U32 width;
	U32 height;
	U32 frameRate;
} pvt_data_t;

// Each configuration name value pair for this mapping will result in this callback being called.
void openavbIntfH264StreamCfgCB(media_q_t *pMediaQ, const char *name, const char *value)
{
	if (!pMediaQ) {
		AVB_LOG_DEBUG("H264-Stream cfgCB: no mediaQ!");
		return;
	}

	char *pEnd = NULL ;
	long tmp;

	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
	if (!pPvtData) {
		AVB_LOG_ERROR("Private interface module data not allocated.");
		return;
	}

	if (strcmp(name, "intf_nv_file_name") == 0) {
		if (pPvtData->file_name) {
			free(pPvtData->file_name);
		}
		pPvtData->file_name = strdup(value);
	}
	else if (strcmp(name, "intf_nv_async_rx") == 0)	{
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
	else if (strcmp(name, "intf_nv_frame_rate") == 0) {
		pPvtData->frameRate = atoi(value);
		AVB_LOGF_INFO("Frame rate is %d", pPvtData->frameRate);
	}
	else if (strcmp(name, "intf_nv_width") == 0) {
		pPvtData->width = atoi(value);
		AVB_LOGF_INFO("Width is %d", pPvtData->width);
	}
	else if (strcmp(name, "intf_nv_height") == 0) {
		pPvtData->height = atoi(value);
		AVB_LOGF_INFO("Height is %d", pPvtData->height);
	}
}

void openavbIntfH264StreamGenInitCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);
	if (!pMediaQ) {
		AVB_LOG_DEBUG("H264-Stream initCB: no mediaQ!");
		AVB_TRACE_EXIT(AVB_TRACE_INTF);
		return;
	}
	AVB_TRACE_EXIT(AVB_TRACE_INTF);

}

static int openavbMediaQGetItemSize(media_q_t *pMediaQ)
{
	int itemSize = 0;
	media_q_item_t *pMediaQItem = openavbMediaQHeadLock(pMediaQ);
	if (pMediaQItem) {
		itemSize = pMediaQItem->itemSize;
		openavbMediaQHeadUnlock(pMediaQ);
	}
	else {
		AVB_LOG_ERROR("pMediaQ item NULL in getMediaQItemSize");
	}

	return itemSize;
}



// A call to this callback indicates that this interface module will be
// a listener. Any listener initialization can be done in this function.
void openavbIntfH264StreamRxInitCB(media_q_t *pMediaQ)
{
	AVB_LOG_DEBUG("Rx Init callback.");
	if (!pMediaQ) {
		AVB_LOG_DEBUG("No MediaQ in H264StreamRxInitCB");
		return;
	}

	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
	if (!pPvtData) {
		AVB_LOG_ERROR("Private interface module data not allocated.");
		return;
	}
	int status = Avbh264StreamInitialize(pPvtData->width, pPvtData->height, pPvtData->frameRate);
	if (status < 0) {
	AVB_LOG_ERROR("unable to initialize the h264sink Thread");
	}
}

// This callback is called when acting as a listener.
bool openavbIntfH264StreamRxCB(media_q_t *pMediaQ)
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

	int err = 0;

	while (1) {
		media_q_item_t *pMediaQItem = openavbMediaQTailLock(pMediaQ, pPvtData->ignoreTimestamp);
		// there are no packets available or they are from the future
		if (!pMediaQItem) {
			break;
		}

		if (!pMediaQItem->dataLen) {
			AVB_LOG_DEBUG("No dataLen");
			openavbMediaQTailPull(pMediaQ);
			continue;
		}

		if (-1 == Avbh264DataSink(pMediaQItem->pPubData, pMediaQItem->dataLen)) {
			AVB_LOG_ERROR("RxCB: Failed to send data to sink");
			return FALSE;
		}

		openavbMediaQTailPull(pMediaQ);
	}
	return TRUE;
}

// This callback will be called when the interface needs to be closed. All shutdown should
// occur in this function.
void openavbIntfH264StreamEndCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);
	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
	if (!pPvtData) {
		AVB_LOG_ERROR("Private interface module data not allocated.");
		return;
	}
	Avbh264StreamClose();
	if (pPvtData->fd) {
		close(pPvtData->fd);
	}
	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

void openavbIntfH264StreamGenEndCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);
	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

// Main initialization entry point into the interface module
extern DLL_EXPORT bool openavbIntfH264StreamInitialize(media_q_t *pMediaQ, openavb_intf_cb_t *pIntfCB)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	if (!pMediaQ) {
		AVB_LOG_DEBUG("H264-gst GstInitialize: no mediaQ!");
		AVB_TRACE_EXIT(AVB_TRACE_INTF);
		return TRUE;
	}

	// Memory freed by the media queue when the media queue is destroyed.
	pMediaQ->pPvtIntfInfo = calloc(1, sizeof(pvt_data_t));

	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
	pPvtData->width = DEFAULT_WIDTH;
	pPvtData->height = DEFAULT_HEIGHT;
	pPvtData->frameRate = DEFAULT_FRAMERATE;

	pIntfCB->intf_cfg_cb = openavbIntfH264StreamCfgCB;
	pIntfCB->intf_gen_init_cb = openavbIntfH264StreamGenInitCB;
	pIntfCB->intf_rx_init_cb = openavbIntfH264StreamRxInitCB;
	pIntfCB->intf_rx_cb = openavbIntfH264StreamRxCB;
	pIntfCB->intf_end_cb = openavbIntfH264StreamEndCB;
	pIntfCB->intf_gen_end_cb = openavbIntfH264StreamGenEndCB;

	pPvtData->ignoreTimestamp = FALSE;

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
	return TRUE;
}
