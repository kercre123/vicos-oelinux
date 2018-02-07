/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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
 *     * Neither the name of The Linux Foundatoin, nor the names of its
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

#ifndef NEUTRINO_H
#define NEUTRINO_H

/* Private IOCTLs parameters */
#define DWC_ETH_QOS_PRV_IOCTL		SIOCDEVPRIVATE
#define DWC_ETH_QOS_AVB_ALGORITHM	27
#define DWC_ETH_QOS_RWK_FILTER_LENGTH	8

/* common data structure between driver and application for
 * sharing info through ioctl
 * */
struct ifr_data_struct {
	unsigned int flags;
	unsigned int chInx; /* dma channel no to be configured */
	unsigned int cmd;
	unsigned int context_setup;
	unsigned int connected_speed;
	unsigned int rwk_filter_values[DWC_ETH_QOS_RWK_FILTER_LENGTH];
	unsigned int rwk_filter_length;
	int command_error;
	int test_done;
	void *ptr;
	unsigned int adrs;
	int bar_num;
        unsigned int unit;
        unsigned int addr;
        unsigned int data;
};

/* FQTSS related enums and structures */
typedef enum {
	AVB_SP = 0,
	AVB_CBS = 1,
} avb_algorithm;

typedef enum {
	QDISABLED = 0x0,
	QAVB,
	QDCB,
	QGENERIC
} queue_operating_mode;

struct avb_algorithm_params {
	unsigned int idle_slope;
	unsigned int send_slope;
	unsigned int hi_credit;
	unsigned int low_credit;
};

struct avb_algorithm {
	unsigned int chInx;
	unsigned int algorithm;
	unsigned int cc;
	struct avb_algorithm_params speed100params;
	struct avb_algorithm_params speed1000params;
	queue_operating_mode op_mode;
};

int ntn_set_class_bandwidth(int nClass, unsigned classBytesPerSec, char *ifname);

#endif // NEUTRINO_H
