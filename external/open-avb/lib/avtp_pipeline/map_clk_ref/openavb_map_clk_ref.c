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
* MODULE SUMMARY : Clock reference stream mapping module conforming to 1722-D15
* Ch 11 encapsulation.
*/

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "openavb_types_pub.h"
#include "openavb_trace_pub.h"
#include "openavb_avtp_time_pub.h"
#include "openavb_mediaq_pub.h"

#include "openavb_map_pub.h"
#include "openavb_map_clk_ref_pub.h"

// DEBUG Uncomment to turn on logging for just this module.
#define AVB_LOG_ON 1

#define	AVB_LOG_COMPONENT "Clk Ref Mapping"
#include "openavb_log_pub.h"

// Header sizes
#define AVTP_V0_HEADER_SIZE 12
#define MAP_HEADER_SIZE 8

#define TOTAL_HEADER_SIZE (AVTP_V0_HEADER_SIZE + MAP_HEADER_SIZE)

//////
// AVTP Version 0 Header
//////

// 1 Byte
//   Hide 4 bits : Stream valid and version bits will be set by
//                 fillAvtpHdr in openavb_avtp.c)
//   mr 1 bit    : Media Clock Restart (Section 11.2.2)
//                 This is toggled each time the media clock is restarted.
//   Hide 1 bit  : Reserved
//   fs 1 bit    : Frame Sync (Section 11.2.3)
//                 Used only for video line sync. For audio sample, video frame,
//                 and machine cycle, this is set to 0.
//   tu 1 bit    : Timing Uncertain (Section 11.2.4).
//                 Set to 0 when the timestamps in the data field are valid.
//                 Set to 1 when the timestamps in the data field may not be
//                 Valid for example if the talker detects a discontinuity in
//                 the gPTP time.
#define HIDX_AVTP_HIDE4_MR1_HIDE1_FS1_TU1 1
#define MEDIA_CLOCK_RESTART_MASK 0x8
#define FRAME_SYNC_MASK 0x2
#define TIMING_UNCERTAIN_MASK 0x1

// 1 Byte : sequence number
#define HIDX_AVTP_SEQUENCE_NUMBER8 2

// 1 Byte : type (Section 11.2.6)
#define HIDX_AVTP_TYPE 3

//////
// Mapping specific header
//////

// 4 bytes
//   pull 4 bits: (Section 11.2.8) Multiplier to apply to the base frequency
//                to arrive at the nominal sampling frequency.
//   baseFrequency 28 bits: (Section 11.2.9) From 1 Hz to 536,870,911 Hz
#define HIDX_AVTP_PULL4_BASE_FREQ28 12
#define PULL_OFFSET 28

// crf_data_length 2 bytes: (Section 11.2.10) length of the crf_data field.
// This must be a multiple of 8.
#define HIDX_DATALEN16				16

// timestampInterval 2 bytes: (Section 11.2.11) number of events (e.g.
// audio samples) between each timestamp in the crf_data field. The value of
// this field shall be static for the duration of the life of the stream and
// shall be nonzero.
#define HIDX_TIMESTAMP_INTERVAL16 18

// crf_data variable bytes: (Section 11.2.12) a sequence of timestamps where
// each timestamp is a 8 bytes and is a future gPTP timestamp in nanoseconds
// (mod 2^64). The length of this field is crf_data_length.
#define HIDX_DATA 20

typedef struct {
	// listener variables
	bool lastClockRestartFlag;
	U8 lastSequenceNumber;
} pvt_data_t;


// Each configuration name value pair for this mapping will result in this
// callback being called.
void openavbMapClkRefCfgCB(media_q_t *pMediaQ, const char *name,
			   const char *value)
{
	AVB_TRACE_ENTRY(AVB_TRACE_MAP);

	if (pMediaQ) {
		media_q_pub_map_clk_ref_info_t *pPubMapInfo =
			pMediaQ->pPubMapInfo;
		if (!pPubMapInfo) {
			AVB_LOG_ERROR("Public mapping module data not allocated.");
			return;
		}

		char *pEnd;
		if (strcmp(name, "map_nv_item_count") == 0) {
			pPubMapInfo->itemCount = strtol(value, &pEnd, 10);
		}
		else if (strcmp(name, "map_nv_packing_factor") == 0) {
			pPubMapInfo->packingFactor = strtol(value, &pEnd, 10);
		}
		else if (strcmp(name, "map_nv_timestamp_interval") == 0) {
			pPubMapInfo->timestampInterval =
				strtol(value, &pEnd, 10);
		}
		else if (strcmp(name, "map_nv_timestamps_per_packet") == 0) {
			pPubMapInfo->timestampsPerPacket =
				strtol(value, &pEnd, 10);
		}
		else if (strcmp(name, "map_nv_pull_multiplier") == 0) {
			pPubMapInfo->pullMultiplier = (avtp_pull_multiplier)
				strtol(value, &pEnd, 10);
		}
		else if (strcmp(name, "map_nv_base_frequency") == 0) {
			pPubMapInfo->baseFrequency =
				strtol(value, &pEnd, 10);
		}
		else if (strcmp(name, "map_nv_crf_type") == 0) {
			pPubMapInfo->crfType = (avtp_crf_type)
				strtol(value, &pEnd, 10);
		}
		else if (strcmp(name, "map_nv_tx_rate") == 0 ||
			 strcmp(name, "map_nv_tx_interval") == 0) {
			pPubMapInfo->txRate = strtol(value, &pEnd, 10);
		}
	}

	AVB_TRACE_EXIT(AVB_TRACE_MAP);
}

U8 openavbMapClkRefSubtypeCB()
{
	AVB_TRACE_ENTRY(AVB_TRACE_MAP);
	AVB_TRACE_EXIT(AVB_TRACE_MAP);
	return 0x04;        // Clock reference format subtype
}

// Returns the AVTP version used by this mapping
U8 openavbMapClkRefAvtpVersionCB()
{
	AVB_TRACE_ENTRY(AVB_TRACE_MAP_DETAIL);
	AVB_TRACE_EXIT(AVB_TRACE_MAP_DETAIL);
	return 0x00;        // Version 0
}

U16 openavbMapClkRefMaxDataSizeCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_MAP);

	if (pMediaQ) {
		media_q_pub_map_clk_ref_info_t *pPubMapInfo =
			pMediaQ->pPubMapInfo;
		if (!pPubMapInfo) {
			AVB_LOG_ERROR("Public mapping module data not allocated.");
			return 0;
		}

		AVB_TRACE_EXIT(AVB_TRACE_MAP);
		return TOTAL_HEADER_SIZE + (pPubMapInfo->timestampsPerPacket *
					    CRF_TIMESTAMP_SIZE);
	}
	AVB_TRACE_EXIT(AVB_TRACE_MAP);
	return 0;
}

// Returns the intended transmit interval (in frames per second).
// 0 = default for talker / class.
U32 openavbMapClkRefTransmitIntervalCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_MAP);
	if (pMediaQ) {
		media_q_pub_map_clk_ref_info_t *pPubMapInfo =
			pMediaQ->pPubMapInfo;
		if (!pPubMapInfo) {
			AVB_LOG_ERROR("Public mapping module data not allocated.");
			return 0;
		}

		AVB_TRACE_EXIT(AVB_TRACE_MAP);
		return pPubMapInfo->txRate;
	}
	AVB_TRACE_EXIT(AVB_TRACE_MAP);
	return 0;
}

void openavbMapClkRefGenInitCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_MAP);
	if (pMediaQ) {
		media_q_pub_map_clk_ref_info_t *pPubMapInfo =
			pMediaQ->pPubMapInfo;
		pvt_data_t *pPvtData = pMediaQ->pPvtMapInfo;
		if (!pPvtData) {
			AVB_LOG_ERROR("Private mapping module data not allocated.");
			return;
		}
		if (!pPubMapInfo) {
			AVB_LOG_ERROR("Public mapping module data not allocated.");
			return;
		}

		openavbMediaQSetSize(pMediaQ, pPubMapInfo->itemCount,
				     pPubMapInfo->packingFactor *
				     pPubMapInfo->timestampsPerPacket *
				     CRF_TIMESTAMP_SIZE);
		openavbMediaQAllocItemMapData(pMediaQ,
			sizeof(media_q_item_map_clk_ref_data_t), 0);
	}
	AVB_TRACE_EXIT(AVB_TRACE_MAP);
}

void openavbMapClkRefAVDECCInitCB(media_q_t *pMediaQ, U16 configIdx,
				  U16 descriptorType, U16 descriptorIdx)
{
	AVB_TRACE_ENTRY(AVB_TRACE_MAP);

	AVB_TRACE_EXIT(AVB_TRACE_MAP);
}

// A call to this callback indicates that this mapping module will be
// a talker. Any talker initialization can be done in this function.
void openavbMapClkRefTxInitCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_MAP);

	// Check the media queue for proper allocations to avoid doing it on
	// each tx callback.
	if (!pMediaQ) {
		AVB_LOG_ERROR("Media queue not allocated.");
		AVB_TRACE_EXIT(AVB_TRACE_MAP);
		return;
	}

	media_q_pub_map_clk_ref_info_t *pPubMapInfo = pMediaQ->pPubMapInfo;
	if (!pPubMapInfo) {
		AVB_LOG_ERROR("Public mapping module data not allocated.");
		AVB_TRACE_EXIT(AVB_TRACE_MAP);
		return;
	}

	pvt_data_t *pPvtData = pMediaQ->pPvtMapInfo;
	if (!pPvtData) {
		AVB_LOG_ERROR("Private mapping module data not allocated.");
		AVB_TRACE_EXIT(AVB_TRACE_MAP);
		return;
	}

	AVB_TRACE_EXIT(AVB_TRACE_MAP);
}

static bool writeTimestamps(media_q_t *pMediaQ, U8 *pPayload, U32 payloadLen)
{
	media_q_pub_map_clk_ref_info_t *pPubMapInfo = pMediaQ->pPubMapInfo;
	U32 timestamps_processed = 0;
	U8 *pAVTPDataUnit = pPayload;
	while (timestamps_processed <
	       pPubMapInfo->timestampsPerPacket) {
		media_q_item_t *pMediaQItem = openavbMediaQTailLock(pMediaQ, TRUE);
		if (pMediaQItem == NULL) {
			AVB_LOG_ERROR("Not enough timestamps for a packet.");
			return FALSE;
		}
		U8 *pItemData = (U8 *)pMediaQItem->pPubData +
			pMediaQItem->readIdx;
		while ((timestamps_processed < pPubMapInfo->timestampsPerPacket)
		       && (pMediaQItem->readIdx < pMediaQItem->dataLen)) {
			*(U64 *)(pAVTPDataUnit) = htonll(*(U64 *)pItemData);
			pAVTPDataUnit += CRF_TIMESTAMP_SIZE;
			pItemData += CRF_TIMESTAMP_SIZE;
			pMediaQItem->readIdx += CRF_TIMESTAMP_SIZE;
			timestamps_processed++;
		}
		if (pMediaQItem->readIdx >= pMediaQItem->dataLen) {
			// Read the entire item
			openavbMediaQTailPull(pMediaQ);
		}
		else {
			// More to read next interval
			openavbMediaQTailUnlock(pMediaQ);
		}
	}
	return TRUE;
}

// This talker callback will be called for each AVB observation interval.
tx_cb_ret_t openavbMapClkRefTxCB(media_q_t *pMediaQ, U8 *pData, U32 *dataLen)
{
	AVB_TRACE_ENTRY(AVB_TRACE_MAP_DETAIL);

	if (!pData || !dataLen) {
		AVB_LOG_ERROR("Mapping module data or data length argument incorrect.");
		return TX_CB_RET_PACKET_NOT_READY;
	}

	media_q_pub_map_clk_ref_info_t *pPubMapInfo = pMediaQ->pPubMapInfo;

	if (*dataLen < pPubMapInfo->timestampsPerPacket * CRF_TIMESTAMP_SIZE +
			TOTAL_HEADER_SIZE ) {
		AVB_LOG_ERROR("Not enough room in packet for timestamps frames.");
		AVB_TRACE_EXIT(AVB_TRACE_MAP_DETAIL);
		return TX_CB_RET_PACKET_NOT_READY;
	}

	if (openavbMediaQIsAvailableBytes(pMediaQ,
			pPubMapInfo->timestampsPerPacket * CRF_TIMESTAMP_SIZE,
			TRUE)) {
		U8 *pHdr = pData;
		U8 *pPayload = pData + TOTAL_HEADER_SIZE;
		// Set media clock restart to 0.
		// Set frame sync to 0.
		// Frame timing uncertain to 0.
		pHdr[HIDX_AVTP_HIDE4_MR1_HIDE1_FS1_TU1] &=
			~MEDIA_CLOCK_RESTART_MASK;
		pHdr[HIDX_AVTP_HIDE4_MR1_HIDE1_FS1_TU1] &= ~FRAME_SYNC_MASK;
		pHdr[HIDX_AVTP_HIDE4_MR1_HIDE1_FS1_TU1] &=
			~TIMING_UNCERTAIN_MASK;

		// Set the type of the CRF stream
		pHdr[HIDX_AVTP_TYPE] = pPubMapInfo->crfType;
		// Set the pull and base frequency
		U32 pull_and_base_freq = pPubMapInfo->baseFrequency;
		pull_and_base_freq |= pPubMapInfo->pullMultiplier <<
			PULL_OFFSET;
		*(U32 *)(&pHdr[HIDX_AVTP_PULL4_BASE_FREQ28]) =
			htonl(pull_and_base_freq);
		// Set the data length
		*(U16 *)(&pHdr[HIDX_DATALEN16]) = htons(
			pPubMapInfo->timestampsPerPacket *
			CRF_TIMESTAMP_SIZE);
		// Set the timestamp interval
		*(U16 *)(&pHdr[HIDX_TIMESTAMP_INTERVAL16]) = htons(
			pPubMapInfo->timestampInterval);
		// Write the timestamps
		if (!writeTimestamps(pMediaQ, pPayload,
			pPubMapInfo->timestampsPerPacket *
			CRF_TIMESTAMP_SIZE)) {
			AVB_LOG_ERROR("Unable to write timestamps to packet.");
			return TX_CB_RET_PACKET_NOT_READY;
		}

		// Set out bound data length (entire packet length)
		*dataLen = (pPubMapInfo->timestampsPerPacket *
			    CRF_TIMESTAMP_SIZE) + TOTAL_HEADER_SIZE;

		AVB_TRACE_LINE(AVB_TRACE_MAP_LINE);
		AVB_TRACE_EXIT(AVB_TRACE_MAP_DETAIL);
		return TX_CB_RET_PACKET_READY;

	}

	AVB_TRACE_EXIT(AVB_TRACE_MAP_DETAIL);
	return TX_CB_RET_PACKET_NOT_READY;
}

// A call to this callback indicates that this mapping module will be
// a listener. Any listener initialization can be done in this function.
void openavbMapClkRefRxInitCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_MAP);
	AVB_TRACE_EXIT(AVB_TRACE_MAP);
}

#define MAX_U8 256

// This callback occurs when running as a listener and data is available.
bool openavbMapClkRefRxCB(media_q_t *pMediaQ, U8 *pData, U32 dataLen)
{
	AVB_TRACE_ENTRY(AVB_TRACE_MAP_DETAIL);
	if (pMediaQ && pData) {
		U8 *pHdr = pData;
		U8 *pPayload = pData + TOTAL_HEADER_SIZE;
		pvt_data_t *pPvtData = pMediaQ->pPvtMapInfo;

		U16 payloadLen = ntohs(*(U16 *)(&pHdr[HIDX_DATALEN16]));
		bool clockRestartFlag = (pHdr[HIDX_AVTP_HIDE4_MR1_HIDE1_FS1_TU1] &
			MEDIA_CLOCK_RESTART_MASK) ? TRUE : FALSE;
		bool tsUncertain = (pHdr[HIDX_AVTP_HIDE4_MR1_HIDE1_FS1_TU1] &
				    TIMING_UNCERTAIN_MASK) ? TRUE : FALSE;
		U8 sequenceNumber = pHdr[HIDX_AVTP_SEQUENCE_NUMBER8];

		U16 timestampInterval = ntohs(*(U16 *)(&pHdr[
			HIDX_TIMESTAMP_INTERVAL16]));

		U8 *pAVTPDataUnit = pPayload;
		U8 *pAVTPDataUnitEnd = pPayload + payloadLen;

		bool restartClock =
			(pPvtData->lastClockRestartFlag != clockRestartFlag) ||
			((pPvtData->lastSequenceNumber + 1) % MAX_U8 != sequenceNumber);

		while (((pAVTPDataUnit + CRF_TIMESTAMP_SIZE) <=
			pAVTPDataUnitEnd)) {
			// Get item pointer in media queue
			media_q_item_t *pMediaQItem = openavbMediaQHeadLock(pMediaQ);

			// If we need to restart the clock, then we need an empty
			// media Q item, and we need to set the restart clock flag on that
			// item
			if (restartClock) {
				if (pMediaQItem && pMediaQItem->dataLen != 0) {
					openavbMediaQHeadPush(pMediaQ);
					pMediaQItem = openavbMediaQHeadLock(pMediaQ);
					assert(!pMediaQItem || pMediaQItem->dataLen == 0);
				}
				if (pMediaQItem) {
					media_q_item_map_clk_ref_data_t *pItemPubMapData =
						(media_q_item_map_clk_ref_data_t *)
						pMediaQItem->pPubMapData;
					pItemPubMapData->restartClock = TRUE;
				}
				restartClock = FALSE;
			} else if (pMediaQItem && pMediaQItem->dataLen == 0) {
				media_q_item_map_clk_ref_data_t *pItemPubMapData =
					(media_q_item_map_clk_ref_data_t *)
					pMediaQItem->pPubMapData;
				pItemPubMapData->restartClock = FALSE;
			}

			if (pMediaQItem) {
				U32 itemSizeWritten = 0;
				U8 *pItemData = (U8 *)pMediaQItem->pPubData +
					pMediaQItem->dataLen;
				U8 *pItemDataEnd = (U8 *)pMediaQItem->pPubData +
					pMediaQItem->itemSize;
				media_q_item_map_clk_ref_data_t *pItemPubMapData =
					(media_q_item_map_clk_ref_data_t *)
					pMediaQItem->pPubMapData;

				if (pMediaQItem->dataLen == 0) {
					// Set time stamp info on first data
					// write to the media queue
					// place it in the media queue item.
					openavbAvtpTimeSetToU64Timestamp(
						pMediaQItem->pAvtpTime,
						ntohll(*(U64*)pAVTPDataUnit));

					// Set timestamp valid and timestamp
					// uncertain flags
					openavbAvtpTimeSetTimestampValid(
						pMediaQItem->pAvtpTime, TRUE);
					openavbAvtpTimeSetTimestampUncertain(
						pMediaQItem->pAvtpTime,
						tsUncertain);

					// Set the timestamp interval
					pItemPubMapData->timestampInterval =
						timestampInterval;
				}

				while (((pAVTPDataUnit +
					CRF_TIMESTAMP_SIZE) <=
					pAVTPDataUnitEnd) && ((pItemData +
					CRF_TIMESTAMP_SIZE) <=
					pItemDataEnd)) {
					*(U64 *)(pItemData) =
						ntohll(*(U64 *)pAVTPDataUnit);
					pAVTPDataUnit += CRF_TIMESTAMP_SIZE;
					pItemData += CRF_TIMESTAMP_SIZE;
					itemSizeWritten += CRF_TIMESTAMP_SIZE;
				}

				pMediaQItem->dataLen += itemSizeWritten;

				if (pMediaQItem->dataLen < pMediaQItem->itemSize) {
					// More data can be written to the item
					openavbMediaQHeadUnlock(pMediaQ);
				}
				else {
					// The item is full push it.
					openavbMediaQHeadPush(pMediaQ);
				}

				pPvtData->lastClockRestartFlag = clockRestartFlag;
				pPvtData->lastSequenceNumber = sequenceNumber;
			}
			else {
				IF_LOG_INTERVAL(1000) AVB_LOG_INFO("Media queue full");
				AVB_TRACE_EXIT(AVB_TRACE_MAP_DETAIL);
				return FALSE;   // Media queue full
			}
		}
		AVB_TRACE_EXIT(AVB_TRACE_MAP_DETAIL);
		return TRUE;    // Normal exit
	}
	AVB_TRACE_EXIT(AVB_TRACE_MAP_DETAIL);
	return FALSE;
}

// This callback will be called when the mapping module needs to be closed.
// All cleanup should occur in this function.
void openavbMapClkRefEndCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_MAP);
	AVB_TRACE_EXIT(AVB_TRACE_MAP);
}

void openavbMapClkRefGenEndCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);
	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

// Initialization entry point into the mapping module. Will need to be included
// in the .ini file.
extern DLL_EXPORT bool openavbMapClkRefInitialize(media_q_t *pMediaQ,
	openavb_map_cb_t *pMapCB, U32 inMaxTransitUsec)
{
	AVB_TRACE_ENTRY(AVB_TRACE_MAP);

	if (pMediaQ) {
		pMediaQ->pMediaQDataFormat = strdup(MapClkRefMediaQDataFormat);
		// Memory freed by the media queue when the media queue is destroyed.
		pMediaQ->pPubMapInfo = calloc(1, sizeof(media_q_pub_map_clk_ref_info_t));
		// Memory freed by the media queue when the media queue is destroyed.
		pMediaQ->pPvtMapInfo = calloc(1, sizeof(pvt_data_t));

		if (!pMediaQ->pMediaQDataFormat || !pMediaQ->pPubMapInfo ||
		    !pMediaQ->pPvtMapInfo) {
			AVB_LOG_ERROR("Unable to allocate memory for mapping module.");
			return FALSE;
		}

		media_q_pub_map_clk_ref_info_t *pPubMapInfo =
			pMediaQ->pPubMapInfo;

		pMapCB->map_cfg_cb = openavbMapClkRefCfgCB;
		pMapCB->map_subtype_cb = openavbMapClkRefSubtypeCB;
		pMapCB->map_avtp_version_cb = openavbMapClkRefAvtpVersionCB;
		pMapCB->map_max_data_size_cb = openavbMapClkRefMaxDataSizeCB;
		pMapCB->map_transmit_interval_cb =
			openavbMapClkRefTransmitIntervalCB;
		pMapCB->map_gen_init_cb = openavbMapClkRefGenInitCB;
		pMapCB->map_avdecc_init_cb = openavbMapClkRefAVDECCInitCB;
		pMapCB->map_tx_init_cb = openavbMapClkRefTxInitCB;
		pMapCB->map_tx_cb = openavbMapClkRefTxCB;
		pMapCB->map_rx_init_cb = openavbMapClkRefRxInitCB;
		pMapCB->map_rx_cb = openavbMapClkRefRxCB;
		pMapCB->map_end_cb = openavbMapClkRefEndCB;
		pMapCB->map_gen_end_cb = openavbMapClkRefGenEndCB;

		pPubMapInfo->itemCount = 20;
		pPubMapInfo->txRate = 0;
		pPubMapInfo->packingFactor = 1;

		pPubMapInfo->timestampsPerPacket = 6;

		openavbMediaQSetMaxLatency(pMediaQ, inMaxTransitUsec);
	}

	AVB_TRACE_EXIT(AVB_TRACE_MAP);
	return TRUE;
}
