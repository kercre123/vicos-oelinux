/*******************************************************************************
Copyright (c) 2016, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/*
* MODULE SUMMARY : Clock Reference interface module.
*/

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "openavb_types_pub.h"
#include "openavb_trace_pub.h"
#include "openavb_mediaq_pub.h"
#include "openavb_map_clk_ref_pub.h"
#include "openavb_intf_pub.h"
#include "openavb_reference_clock_pub.h"
#include "openavb_osal.h"

#define	AVB_LOG_COMPONENT	"Clk Ref Interface"
#include "openavb_log_pub.h"

//#define DEBUG

typedef struct {
	// Config variables

	// intf_nv_timestampBufferSize
	U16 timestampBufferSize;

	// intf_nv_timestampNsecDelay
	U64 timestampNsecDelay ;

	// Talker variables
	U16 timestampsPerPacket;
	U16 timestampReadIdx;
	U16 timestampWriteIdx ;
	SEM_T(timestampSem);
	U64 *timestampBuffer;
} pvt_data_t;

#define NSEC_PER_USEC 1000

// Each configuration name value pair for this mapping will result in this
// callback being called.
void openavbIntfClkRefCfgCB(media_q_t *pMediaQ, const char *name,
			    const char *value)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);
	if (pMediaQ) {
		char *pEnd;

		pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
		if (!pPvtData) {
			AVB_LOG_ERROR("Private interface module data not allocated.");
			return;
		}

		media_q_pub_map_clk_ref_info_t *pPubMapClkRefInfo;
		pPubMapClkRefInfo = (media_q_pub_map_clk_ref_info_t *)pMediaQ->pPubMapInfo;
		if (!pPubMapClkRefInfo) {
			AVB_LOG_ERROR("Public map data for audio info not allocated.");
			return;
		}

		if (strcmp(name, "intf_nv_timestamp_usec_delay") == 0) {
			pPvtData->timestampNsecDelay  =
				strtoull(value, &pEnd, 10) * NSEC_PER_USEC;
		}
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

void openavbIntfClkRefGenInitCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);
	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

static inline U16 timestamp_buff_count(pvt_data_t *pPvtData)
{
	if (pPvtData == NULL) {
		AVB_LOG_ERROR("pPvtData is NULL");
		return 0;
	}

	return (pPvtData->timestampWriteIdx  +
		pPvtData->timestampBufferSize -
		pPvtData->timestampReadIdx) %
		pPvtData->timestampBufferSize;
}

static inline bool timestamp_buff_is_full(pvt_data_t *pPvtData)
{
	if (pPvtData == NULL) {
		AVB_LOG_ERROR("pPvtData is NULL");
		return TRUE;
	}

	// Note: because this is a circular buffer, there has to always be one
	// empty cell.
	return timestamp_buff_count(pPvtData) >=
		(pPvtData->timestampBufferSize - 1);
}

static inline bool timestamp_buff_is_empty(pvt_data_t *pPvtData)
{
	if (pPvtData == NULL) {
		AVB_LOG_ERROR("pPvtData is NULL");
	}

	return timestamp_buff_count(pPvtData) == 0;
}

// clockTickCallback is called whenever the clock tick occurs.
// Record the tick into the private data.
static void clockTickCallback(void *context, U64 timestamp,
	U16 ticks, U32 interval, bool restart_clock)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	assert(ticks == 1);

	pvt_data_t *pPvtData = context;

	if (timestamp_buff_is_full(pPvtData)) {
		AVB_LOG_ERROR("Timestamp buffer is full. Dropping clock reference timestamp.");
		AVB_TRACE_EXIT(AVB_TRACE_INTF);
		return;
	}

	pPvtData->timestampBuffer[pPvtData->timestampWriteIdx ] = timestamp;
	pPvtData->timestampWriteIdx  = (pPvtData->timestampWriteIdx  + 1) %
		pPvtData->timestampBufferSize;

	if (timestamp_buff_count(pPvtData) >= pPvtData->timestampsPerPacket) {
		SEM_ERR_T(err);
		SEM_POST(pPvtData->timestampSem, err);

		if (!SEM_IS_ERR_NONE(err)) {
			AVB_LOG_ERROR("Error posting to the semaphore for the timestamp buffer.");
		}
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

// A call to this callback indicates that this interface module will be
// a talker. Any talker initialization can be done in this function.
void openavbIntfClkRefTxInitCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	if (pMediaQ) {
		pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
		if (!pPvtData) {
			AVB_LOG_ERROR("Private interface module data not allocated.");
			return;
		}

		// Get the number of timestamps per packet from the map info.
		media_q_pub_map_clk_ref_info_t *pPubMapInfo =
			(media_q_pub_map_clk_ref_info_t *)pMediaQ->pPubMapInfo;
		pPvtData->timestampsPerPacket =
			pPubMapInfo->timestampsPerPacket;
		pPvtData->timestampBufferSize =
			pPvtData->timestampsPerPacket * 2;
		pPvtData->timestampBuffer = malloc(CRF_TIMESTAMP_SIZE *
			pPvtData->timestampBufferSize);
		if (pPvtData->timestampBuffer == NULL) {
			AVB_LOG_ERROR("Failed to allocate timestamp buffer");
		}

		// Initialize the timestamp buffer
		pPvtData->timestampReadIdx = 0;
		pPvtData->timestampWriteIdx  = 0;

		// Initialize the semaphore for the timestamp buffer
		SEM_ERR_T(err);
		SEM_INIT(pPvtData->timestampSem, 0, err);
		if (!SEM_IS_ERR_NONE(err)) {
			AVB_LOG_ERROR("Error initializing timestamp semaphore.");
			return;
		}

		// Register the clock observer to be notified of the clock ticks
		refClkRegisterObserver(clockTickCallback, pPvtData);
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

bool writeTimestampsToMediaQ(media_q_t *pMediaQ)
{
	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;

	// Check if we have enough timestamps.
	if (timestamp_buff_count(pPvtData) < pPvtData->timestampsPerPacket) {
		return FALSE;
	}

	// While we have more than a packet's worth of timestamps in the
	// buffer...
	bool ret = FALSE;
	media_q_item_t *pMediaQItem = NULL;
	while (timestamp_buff_count(pPvtData) >=
		pPvtData->timestampsPerPacket) {
		U16 timestamps_written = 0;


		// Write a packet's worth of timestamps to the media Q.
		for (timestamps_written = 0;
		     timestamps_written < pPvtData->timestampsPerPacket;
		     ++timestamps_written) {

			// Get the media Q item
			if (!pMediaQItem) {
				pMediaQItem =
					openavbMediaQHeadLock(pMediaQ);
				if (!pMediaQItem) {
					break; // Media queue full
				}
			}

			// The media Q item is not full.
			assert(pMediaQItem->itemSize + CRF_TIMESTAMP_SIZE <=
			       pMediaQItem->dataLen);

			// Write the timestamp (plus the delay)
			// to the media Q item
			*(U64 *)(pMediaQItem->pPubData +
				 pMediaQItem->dataLen) =
				pPvtData->timestampBuffer[
				pPvtData->timestampReadIdx] +
				pPvtData->timestampNsecDelay ;

			pMediaQItem->dataLen += CRF_TIMESTAMP_SIZE;
			pPvtData->timestampReadIdx =
				(pPvtData->timestampReadIdx + 1) %
				pPvtData->timestampBufferSize;

			// Push the media Q item if it's full
			if (pMediaQItem->dataLen + CRF_TIMESTAMP_SIZE >
			    pMediaQItem->itemSize) {
				openavbMediaQHeadPush(pMediaQ);
				pMediaQItem = NULL;
			}
		}

		// If the for loop exited early because of a full Media Q,
		// break out of the while loop.
		if (timestamps_written < pPvtData->timestampsPerPacket) {
			break;
		}

		ret = TRUE;
	}

	// At this point, either the media Q item is pushed, or it's
	// not full.
	if (pMediaQItem) {
		assert(pMediaQItem->dataLen + CRF_TIMESTAMP_SIZE <=
		       pMediaQItem->itemSize);
		openavbMediaQHeadUnlock(pMediaQ);
	}

	return ret;
}

// This callback will be called for each AVB transmit interval. 
bool openavbIntfClkRefTxCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF_DETAIL);

	// Validate parameters
	if (!pMediaQ) {
		AVB_TRACE_EXIT(AVB_TRACE_INTF_DETAIL);
		return FALSE;
	}

	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
	if (!pPvtData) {
		AVB_LOG_ERROR("Private interface module data not allocated.");
		AVB_TRACE_EXIT(AVB_TRACE_INTF_DETAIL);
		return FALSE;
	}


	// Wait until we have enough timestamps. Then read the
	// timestamps and write them to the media queue item.
	SEM_ERR_T(err);
	SEM_TIMEDWAIT(pPvtData->timestampSem, 200, err);
	if (!SEM_IS_ERR_NONE(err)) {
		if (SEM_IS_ERR_TIMEOUT(errno)) {
			// If there is no clock source, e.g. there is no
			// audio listener or the listener isn't configured
			// to generate clock ticks, then wait for the
			// timestamp semaphore will time out. This is normal
			// behavior, not an error.
#ifdef DEBUG
			AVB_LOG_ERROR("Timestamp semaphore timeout.");
#endif
			AVB_TRACE_EXIT(AVB_TRACE_INTF_DETAIL);
			return false;
		}
		else {
			AVB_LOG_ERROR("Error waiting on timestamp semaphore.");
			SEM_LOG_ERR(errno);

			return FALSE;
		}
	}

	bool ret = writeTimestampsToMediaQ(pMediaQ);

	AVB_TRACE_EXIT(AVB_TRACE_INTF_DETAIL);
	return ret;
}

// A call to this callback indicates that this interface module will be
// a listener. Any listener initialization can be done in this function.
void openavbIntfClkRefRxInitCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	if (pMediaQ) {
		pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
		if (!pPvtData) {
			AVB_LOG_ERROR("Private interface module data not allocated.");
			return;
		}
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

// This callback is called when acting as a listener.
bool openavbIntfClkRefRxCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF_DETAIL);

	if (pMediaQ) {
		// Remove 1 timestamp from the media queue and adjust the
		// media queue item timestamp so that this can be called when
		// the time arrives for the next timestamp.

		media_q_item_t *pMediaQItem = openavbMediaQTailLock(pMediaQ,
								    FALSE);
		if (pMediaQItem) {
			// Read the next timestamp. This is the timestamp
			// that has just elapsed.
			U64 timestamp = *(U64 *)(pMediaQItem->pPubData +
						 pMediaQItem->readIdx);
			pMediaQItem->readIdx += CRF_TIMESTAMP_SIZE;

			media_q_item_map_clk_ref_data_t *pItemPubMapData =
				(media_q_item_map_clk_ref_data_t *)
				pMediaQItem->pPubMapData;
			bool restartClock = pItemPubMapData->restartClock;
			pItemPubMapData->restartClock = FALSE;

			// If there are no more timestamps, pull the item.
			// If there are more timestamps, set the item timestamp
			// to the next timestamp and unlock the item.
			if (pMediaQItem->readIdx >= pMediaQItem->dataLen) {
				openavbMediaQTailPull(pMediaQ);
			} else {
				openavbAvtpTimeSetToU64Timestamp(
					pMediaQItem->pAvtpTime, *(U64 *)
					(pMediaQItem->pPubData +
					 pMediaQItem->readIdx));
				openavbMediaQTailUnlock(pMediaQ);
			}

			// Notify the clock observers (e.g. the audio talker)
			// that the tick has occurred.
			refClkSignalTick(timestamp, 1,
				pItemPubMapData->timestampInterval, restartClock);
		}
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF_DETAIL);
	return TRUE;
}

// This callback will be called when the interface needs to be closed. All
// shutdown should occur in this function.
void openavbIntfClkRefEndCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	// Unregister the reference clock observer. Note, the observer is only
	// registered if this interface is a talker, so this call won't do
	// anything if this interface is a listener.
	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
	refClkUnregisterObserver(pPvtData);

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

bool openavbIntfClkRefBlockingInIntfCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	return TRUE;

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

void openavbIntfClkRefGenEndCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	if (pMediaQ != NULL && pMediaQ->pPvtIntfInfo != NULL) {
		pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
		free(pPvtData->timestampBuffer);
		pPvtData->timestampBuffer = NULL;
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

// Main initialization entry point into the interface module
extern DLL_EXPORT bool openavbIntfClkRefInitialize(media_q_t *pMediaQ, openavb_intf_cb_t *pIntfCB)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	if (pMediaQ) {
		// Memory freed by the media queue when the media queue is destroyed.
		pMediaQ->pPvtIntfInfo = calloc(1, sizeof(pvt_data_t));

		if (!pMediaQ->pPvtIntfInfo) {
			AVB_LOG_ERROR("Unable to allocate memory for AVTP interface module.");
			return FALSE;
		}

		pIntfCB->intf_cfg_cb = openavbIntfClkRefCfgCB;
		pIntfCB->intf_gen_init_cb = openavbIntfClkRefGenInitCB;
		pIntfCB->intf_tx_init_cb = openavbIntfClkRefTxInitCB;
		pIntfCB->intf_tx_cb = openavbIntfClkRefTxCB;
		pIntfCB->intf_rx_init_cb = openavbIntfClkRefRxInitCB;
		pIntfCB->intf_rx_cb = openavbIntfClkRefRxCB;
		pIntfCB->intf_end_cb = openavbIntfClkRefEndCB;
		pIntfCB->intf_gen_end_cb = openavbIntfClkRefGenEndCB;
		pIntfCB->intf_tx_blocking_in_intf_cb =
			openavbIntfClkRefBlockingInIntfCB;
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
	return TRUE;
}
