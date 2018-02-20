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

#include "openavb_rawsock.h"
#include <malloc.h>
#include "simple_rawsock.h"
#include "ring_rawsock.h"
#ifndef ANDROID
#ifdef USE_GLIB
#include <glib.h>
#define strlcpy g_strlcpy
#define strlcat g_strlcat
#endif
#endif

#define AVB_FEATURE_PCAP 1
#if AVB_FEATURE_PCAP
#include "pcap_rawsock.h"
#endif

#include "openavb_trace.h"

#define	AVB_LOG_COMPONENT	"Raw Socket"
#include "openavb_log.h"


// Get information about an interface
bool openavbCheckInterface(const char *ifname_uri, if_info_t *info)
{
	AVB_TRACE_ENTRY(AVB_TRACE_RAWSOCK);

	const char* ifname = ifname_uri;
	char proto[IF_NAMESIZE] = {0};
	char *colon = strchr(ifname_uri, ':');
	if (colon) {
		ifname = colon + 1;
		memcpy(proto, ifname_uri, colon - ifname_uri);
	}

	AVB_LOGF_DEBUG("%s ifname_uri %s ifname %s proto %s", __func__, ifname_uri, ifname, proto);

	bool ret = simpleAvbCheckInterface(ifname, info);

	AVB_TRACE_EXIT(AVB_TRACE_RAWSOCK);
	return ret;
}

// Open a rawsock for TX or RX
void *openavbRawsockOpen(const char *ifname_uri, bool rx_mode, bool tx_mode, U16 ethertype, U32 frame_size, U32 num_frames)
{
	AVB_TRACE_ENTRY(AVB_TRACE_RAWSOCK);

	const char* ifname = ifname_uri;
#ifdef ANDROID
    char proto[IF_NAMESIZE] = "pcap";
#elif defined(AVB_FEATURE_PCAP)
    char proto[IF_NAMESIZE] = "pcap";
#else
    char proto[IF_NAMESIZE] = "ring";
#endif
	char *colon = strchr(ifname_uri, ':');
	if (colon) {
		ifname = colon + 1;
		*colon = 0;
		strlcpy(proto, ifname_uri, sizeof(proto));
	}

	AVB_LOGF_DEBUG("%s ifname_uri %s ifname %s proto %s", __func__, ifname_uri, ifname, proto);

	void *pvRawsock = NULL;

	if (strcmp(proto, "ring") == 0) {

		AVB_LOG_INFO("Using *ring* buffer implementation");

		// allocate memory for rawsock object
		ring_rawsock_t *rawsock = calloc(1, sizeof(ring_rawsock_t));
		if (!rawsock) {
			AVB_LOG_ERROR("Creating rawsock; malloc failed");
			return NULL;
		}

		// call constructor
		pvRawsock = ringRawsockOpen(rawsock, ifname, rx_mode, tx_mode, ethertype, frame_size, num_frames);

	} else if (strcmp(proto, "simple") == 0) {

		AVB_LOG_INFO("Using *simple* implementation");

		// allocate memory for rawsock object
		simple_rawsock_t *rawsock = calloc(1, sizeof(simple_rawsock_t));
		if (!rawsock) {
			AVB_LOG_ERROR("Creating rawsock; malloc failed");
			return NULL;
		}

		// call constructor
		pvRawsock = simpleRawsockOpen(rawsock, ifname, rx_mode, tx_mode, ethertype, frame_size, num_frames);
#if AVB_FEATURE_PCAP
	} else if (strcmp(proto, "pcap") == 0) {

		AVB_LOG_INFO("Using *pcap* implementation");

		// allocate memory for rawsock object
		pcap_rawsock_t *rawsock = calloc(1, sizeof(pcap_rawsock_t));
		if (!rawsock) {
			AVB_LOG_ERROR("Creating rawsock; malloc failed");
			return NULL;
		}

		// call constructor
		pvRawsock = pcapRawsockOpen(rawsock, ifname, rx_mode, tx_mode, ethertype, frame_size, num_frames);
#endif
	} else {
		AVB_LOGF_ERROR("Unknown proto %s specified.", proto);
	}

	AVB_TRACE_EXIT(AVB_TRACE_RAWSOCK);
	return pvRawsock;
}
