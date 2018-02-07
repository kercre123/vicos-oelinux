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
* MODULE SUMMARY : MJPEG File interface module.
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include "openavb_types_pub.h"
#include "openavb_trace_pub.h"
#include "openavb_mediaq_pub.h"
#include "openavb_intf_pub.h"
#include "openavb_map_mjpeg_pub.h"

#define	AVB_LOG_COMPONENT	"MJPEG File Interface"
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
	int call_rate;
	bool get_avtp_timestamp;        /*<! this flag indicates whether
                                        an avtp timestamp should be taken */
	U32 frame_timestamp;            /*<! this is a timestamp of a video frame */
	U64 frame_time;  //used to pace the talking
	bool sendFrameDataOnly;
} pvt_data_t;

// Each configuration name value pair for this mapping will result in this callback being called.
void openavbIntfMjpegFileCfgCB(media_q_t *pMediaQ, const char *name, const char *value)
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
	else if (strcmp(name, "intf_nv_bitrate") == 0){
		/*tmp = strtol(value, &pEnd, 10);
		if (*pEnd == '\0' && tmp == 1)
		{
			pPvtData->bitrate = atoi(tmp) ;
		}*/
		pPvtData->bitrate = atoi(value);
	}
	else if (strcmp(name, "intf_nv_call_rate") == 0) {
		pPvtData->call_rate = atoi(value);
	}
	else if(strcmp(name, "intf_nv_frame_rate") == 0) {
        U32 val = (U32)atoi(value);
        if (val != 0) {
            pPvtData->frame_time = 1000000000/(U64)val;
            printf("Frame_Time %" PRIu64 " framePerSec %d\n",
                    pPvtData->frame_time, val);
        }
	}
	else if(strcmp(name, "intf_nv_only_send_frame_data") == 0) {
		tmp = strtol(value, &pEnd, 10);
		if (*pEnd == '\0' && tmp == 1) {
			pPvtData->sendFrameDataOnly = (tmp == 1);
		}
	}

}

void openavbIntfMjpegFileGenInitCB(media_q_t *pMediaQ)
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
// a talker. Any talker initialization can be done in this function.
void openavbIntfMjpegFileTxInitCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	if (!pMediaQ){
		AVB_LOG_DEBUG("mjpeg-file txinit: no mediaQ!");
		AVB_TRACE_EXIT(AVB_TRACE_INTF);
		return;
	}

	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
	if (!pPvtData) {
		AVB_LOG_ERROR("Private interface module data not allocated.");
		return;
	}

	//mmap file to read and setup private data
	pPvtData->fd = open(pPvtData->file_name, O_RDONLY);
	if(pPvtData->fd == -1) {
		AVB_LOG_DEBUG("mjpeg-file txinit: no file not found");
		AVB_TRACE_EXIT(AVB_TRACE_INTF);
		return;
	}
	fstat(pPvtData->fd, &pPvtData->statbuf);
	pPvtData->fp = (U8*)mmap(NULL, pPvtData->statbuf.st_size, PROT_READ, MAP_FILE|MAP_PRIVATE, pPvtData->fd, (off_t) 0);
	if(pPvtData->fp == (void*)-1) {
		AVB_LOG_DEBUG("mjpeg-file txinit: could not mmap file");
		AVB_TRACE_EXIT(AVB_TRACE_INTF);
		return;

	}
	pPvtData->read_size = pPvtData->bitrate / pPvtData->call_rate; //hardcoded to assume class B traffic
	AVB_LOGF_INFO("read_size %d bw %d\n", pPvtData->read_size, pPvtData->bitrate);
	pPvtData->find_soi = TRUE;
	pPvtData->loc = 0;
	pPvtData->get_avtp_timestamp = TRUE;

	AVB_TRACE_EXIT(AVB_TRACE_INTF);

	return;
}

// This callback will be called for each AVB transmit interval. Commonly this will be
// 4000 or 8000 times  per second.
bool openavbIntfMjpegFileTxCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF_DETAIL);

	if (!pMediaQ) {
		AVB_LOG_DEBUG("No MediaQ in MjpegGstTxCB");
		AVB_TRACE_EXIT(AVB_TRACE_INTF);
		return FALSE;
	}

	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
	if (!pPvtData) {
		AVB_LOG_ERROR("Private interface module data not allocated.");
		return FALSE;
	}
	bool eoi = FALSE;
	U32 read_size;
	U32 i = 0;
	U32 paySize = 0;
	static U64 next_tx_time;
	static U32 cnt = 0;
	avtp_time_t cur_time;
	U8 *buf;
	static int wait = 0;

	//Transmit data --BEGIN--
	media_q_item_t *pMediaQItem = openavbMediaQHeadLock(pMediaQ);
	if (pMediaQItem) {
		if (pPvtData->sendFrameDataOnly) {
			if (wait) {
				openavbAvtpTimeSetToWallTime(&cur_time);
				if(cur_time.timeNsec < next_tx_time){
					cnt++;
					return TRUE;
				}
				wait = 0;
				cnt = 0;
				//printf("cur time %lld next time %lld count %d\n", cur_time.timeNsec, next_tx_time, cnt);
			}
			//parse for FFD8 -> Start of Image marker
			while(pPvtData->find_soi){
				//make sure not to go out of bounds of the file
				if (pPvtData->loc + 1 == pPvtData->statbuf.st_size) {
					pPvtData->loc = 0;
				}
				if (pPvtData->fp[pPvtData->loc] == 0xFF && pPvtData->fp[pPvtData->loc+1] == 0xD8){
					//found SoI
					pPvtData->find_soi = FALSE;	
					// If FPS limited, calculate when the next frame should go out
					if (pPvtData->frame_time != 0) {
						openavbAvtpTimeSetToWallTime(&cur_time);
						next_tx_time = cur_time.timeNsec + pPvtData->frame_time;
					}
					break;
				}
				pPvtData->loc++;
			}
		}
		//copy over the a read_size amount of data, unless that'll hit eof, then copy up to eof
		//afterwards check for the End of Image flag
		if (pPvtData->loc + pPvtData->read_size > pPvtData->statbuf.st_size) {
			read_size = pPvtData->statbuf.st_size - pPvtData->loc;
		}
		else {
			read_size = pPvtData->read_size;
		}
		if (read_size > 0) {
			memcpy(pMediaQItem->pPubData, &pPvtData->fp[pPvtData->loc], read_size);
		}
		else {
			return FALSE;
		}
		buf = (U8*)pMediaQItem->pPubData;
		//check for FFD9 -> End of Image marker
		for (i = 0; i+1 < read_size; i++) {
			if (buf[i] == 0xFF && buf[i+1] == 0xD9) {
				eoi = TRUE;
				i++;
				break;
			}
		}
		pMediaQItem->dataLen = i+1;
		pPvtData->loc += i+1;//check if it's form the dof case or not?
		if (pPvtData->loc + pPvtData->read_size >= pPvtData->statbuf.st_size) {

			close(pPvtData->fd);
		}


		if (eoi) {
			((media_q_item_map_mjpeg_pub_data_t *)pMediaQItem->pPubMapData)->lastFragment = TRUE;
			// next time get avtp timestamp
			pPvtData->get_avtp_timestamp = TRUE;
			pPvtData->find_soi = TRUE;
			if (pPvtData->frame_time != 0) {
			    // If FPS limited, wait before sending next frame
			    wait = 1;
			}
		}
		else {
			// it means this is a new bunch of fragments
			// only first timestamp need to be taken
			if (pPvtData->get_avtp_timestamp) {
				openavbAvtpTimeSetToWallTime(pMediaQItem->pAvtpTime);
				pPvtData->get_avtp_timestamp = FALSE;
			}
			((media_q_item_map_mjpeg_pub_data_t *)pMediaQItem->pPubMapData)->lastFragment = FALSE;
		}

		openavbMediaQHeadPush(pMediaQ);


		AVB_TRACE_EXIT(AVB_TRACE_INTF_DETAIL);
		return TRUE;
	}
	else {
		AVB_TRACE_EXIT(AVB_TRACE_INTF_DETAIL);

		return FALSE;	// Media queue full
	}
	openavbMediaQHeadUnlock(pMediaQ);
	pMediaQItem->dataLen = 0;
	AVB_TRACE_EXIT(AVB_TRACE_INTF_DETAIL);
	return TRUE;
}


// A call to this callback indicates that this interface module will be
// a listener. Any listener initialization can be done in this function.
void openavbIntfMjpegFileRxInitCB(media_q_t *pMediaQ)
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

	pPvtData->loc = 0;
	pPvtData->fd = open(pPvtData->file_name, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
	if(pPvtData->fd == -1) {
		AVB_LOG_ERROR("Failed to create file");
	}

}

// This callback is called when acting as a listener.
bool openavbIntfMjpegFileRxCB(media_q_t *pMediaQ)
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


		memcpy(&pPvtData->rec_frame[pPvtData->loc], pMediaQItem->pPubData, pMediaQItem->dataLen);
		pPvtData->loc += pMediaQItem->dataLen;
		if ( ((media_q_item_map_mjpeg_pub_data_t *)pMediaQItem->pPubMapData)->lastFragment ) {
			pPvtData->get_avtp_timestamp = TRUE;
			//Found End of Image, write frame to the file
			write(pPvtData->fd, pPvtData->rec_frame, pPvtData->loc);
			pPvtData->loc = 0;
			//in the case of a deocder, pass it up the timestamp as well
		}
		else {
			if(pPvtData->get_avtp_timestamp) {
				pPvtData->frame_timestamp = openavbAvtpTimeGetAvtpTimestamp(pMediaQItem->pAvtpTime);
				pPvtData->get_avtp_timestamp = FALSE;
			}
			else {
				U32 fragment_timestamp = openavbAvtpTimeGetAvtpTimestamp(pMediaQItem->pAvtpTime);
				// all fragments should have the same timestamp
				if(pPvtData->frame_timestamp != fragment_timestamp) {
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
void openavbIntfMjpegFileEndCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
	if (!pPvtData) {
		AVB_LOG_ERROR("Private interface module data not allocated.");
		return;
	}


	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

void openavbIntfMjpegFileGenEndCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);
	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

// Main initialization entry point into the interface module
extern DLL_EXPORT bool openavbIntfMjpegFileInitialize(media_q_t *pMediaQ, openavb_intf_cb_t *pIntfCB)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	if (!pMediaQ) {
		AVB_LOG_DEBUG("mjpeg-gst GstInitialize: no mediaQ!");
		AVB_TRACE_EXIT(AVB_TRACE_INTF);
		return TRUE;
	}

	pMediaQ->pPvtIntfInfo = calloc(1, sizeof(pvt_data_t));		// Memory freed by the media queue when the media queue is destroyed.

	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;

	pPvtData->get_avtp_timestamp = TRUE;

	pIntfCB->intf_cfg_cb = openavbIntfMjpegFileCfgCB;
	pIntfCB->intf_gen_init_cb = openavbIntfMjpegFileGenInitCB;
	pIntfCB->intf_tx_init_cb = openavbIntfMjpegFileTxInitCB;
	pIntfCB->intf_tx_cb = openavbIntfMjpegFileTxCB;
	pIntfCB->intf_rx_init_cb = openavbIntfMjpegFileRxInitCB;
	pIntfCB->intf_rx_cb = openavbIntfMjpegFileRxCB;
	pIntfCB->intf_end_cb = openavbIntfMjpegFileEndCB;
	pIntfCB->intf_gen_end_cb = openavbIntfMjpegFileGenEndCB;

	pPvtData->ignoreTimestamp = FALSE;
	pPvtData->sendFrameDataOnly = FALSE;

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
	return TRUE;
}
