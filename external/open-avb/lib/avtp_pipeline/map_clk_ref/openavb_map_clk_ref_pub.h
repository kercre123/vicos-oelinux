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
* HEADER SUMMARY : Clock reference mapping module public interface
* 
* Refer to IEEE 1722-D15 Chapter 11 details of the "source packet" structure.
*/

#ifndef OPENAVB_MAP_CLK_REF_PUB_H
#define OPENAVB_MAP_CLK_REF_PUB_H 1

#include "openavb_types_pub.h"
#include "openavb_intf_pub.h"

/** \file
 * Clock reference mapping module public interface.
 *
 * Refer to IEEE 1722-D15 Chapter 11 for details of the "source packet"
 * structure.
 */

/** \note A define is used for the MediaQDataFormat identifier because it is
 * needed in separate execution units (static / dynamic libraries) that is why
 * a single static (static/extern pattern) definition can not be used.
 */
#define MapClkRefMediaQDataFormat "ClkRef"

/** Size of each clock reference timestamp in bytes */
#define CRF_TIMESTAMP_SIZE 8

// Multipliers to be used for the Pull field
typedef enum {
	AVTP_PULL_1            = 0,  // Multiply base freq by 1.0
	AVTP_PULL_1_OVER_1_001 = 1,  // Multiply base freq by 1/1.001
	AVTP_PULL_1_001        = 2,  // Multiply base freq by 1.001
	AVTP_PULL_24_OVER_25   = 3,  // Multiply base freq by 24/25
	AVTP_PULL_25_OVER_24   = 4,  // Multiply base freq by 25/24
	AVTP_PULL_1_OVER_8     = 5  // Multiply base freq by 1/8
} avtp_pull_multiplier;

// Clock reference format types for the Type field
typedef enum {
	AVTP_TYPE_CRF_USER          = 0,
	AVTP_TYPE_CRF_AUDIO_SAMPLE  = 1,
	AVTP_TYPE_CRF_VIDEO_FRAME   = 2,
	AVTP_TYPE_CRF_VIDEO_LINE    = 3,
	AVTP_TYPE_CRF_MACHINE_CYCLE = 4
} avtp_crf_type;

/** Contains detailed information of the clock reference data.
 * \note The mapping module will set these during the RX and TX
 * init callbacks. The interface module can use these during the RX and TX
 * callbacks.
 */
typedef struct {
	// The mapping module will set these during the RX and TX init callbacks
	// The interface module can use these during the RX and TX callbacks.

	// Begin configuration items
	/// Number of media queue items in the media queue
	U32 itemCount;

	/// Number of packets per media queue item
	U32 packingFactor;

	/// Number of events (e.g. audio samples) between each timestamp
	U16 timestampInterval;

	/// Number of timestamps for one data packet
	U16 timestampsPerPacket;

	/// Base frequency multiplier
	avtp_pull_multiplier pullMultiplier;

	/// Base frequency
	U32 baseFrequency;

	/// Clock reference type
	avtp_crf_type crfType;

	/// Transmit rate (packets per second)
	U32 txRate;

	// End configuration items

	/// CB for interface modules to do translations in place before data is moved into the mediaQ on rx.	
	openavb_intf_rx_translate_cb_t	intf_rx_translate_cb;
} media_q_pub_map_clk_ref_info_t;

/**
 * Contains information in the header of the CRF packet for use by the interface.
 */
typedef struct {
	bool restartClock;
	U16 timestampInterval;
} media_q_item_map_clk_ref_data_t;

#endif  // OPENAVB_MAP_CLK_REF_PUB_H
