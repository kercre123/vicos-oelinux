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

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <syslog.h>
#include "neutrino.h"

#define SPEED_1000 1000
#define SPEED_100 100

#define MAX_INTERFERENCE_SIZE (1542*8)
#define MAX_FRAME_SIZE (1542*8)

// Ethernet Frame overhead
// L2 includes MAC src/dest, VLAN tag, Ethertype
#define AVTP_L2_OVERHEAD 18
// L1 includes preamble, start-of-frame, FCS, interframe gap
#define AVTP_L1_OVERHEAD 24
// All of the above
#define AVTP_ETHER_FRAME_OVERHEAD (AVTP_L1_OVERHEAD + AVTP_L2_OVERHEAD)

typedef enum {
	// Stream reservation class A. 8000 packets per second
	SR_CLASS_A,
	// Stream reservation class B. 4000 packets per second
	SR_CLASS_B,
	// Number of supported stream reservation classes
	MAX_AVB_SR_CLASSES
} SRClassIdx_t;

static unsigned int get_idle_slope(float bw, int connected_speed)
{
	unsigned int multiplier = 1;
	unsigned int idle_slope = 0;

	if (connected_speed == SPEED_1000)
		multiplier = 2;

	idle_slope = ((int)((4 * multiplier * bw) * 1024));

	syslog(LOG_DEBUG, "idle_slope = 0x%08x\n", idle_slope);

	return idle_slope;
}

static unsigned int get_send_slope(float bw,  int connected_speed)
{
	unsigned int multiplier = 1;
	float idle_slope = 0;
	float send_slope = 0;

	if (connected_speed == SPEED_1000)
		multiplier = 2;

	idle_slope = (4 * multiplier * bw);
	send_slope = (((4 * multiplier) - idle_slope) * 1024);

	syslog(LOG_DEBUG, "send_slope = 0x%08x\n", ((int) send_slope));

	return ((int)(send_slope));
}

static unsigned int get_hi_credit(float bw, int connected_speed, int class)
{
	unsigned int hi_credit = 0;
	typedef unsigned long long u64;
	unsigned int multiplier = 1;

	if (connected_speed == SPEED_1000)
		multiplier = 2;

	hi_credit = (u64)(MAX_INTERFERENCE_SIZE * bw * 1024);
	syslog(LOG_DEBUG, "hi credit = 0x%08x\n", hi_credit);
	return hi_credit;
}

static unsigned int get_low_credit(float bw, int connected_speed, int class)
{
	int low_credit = 0;
	typedef unsigned long long u64;
	float idle_slope;
	float send_slope;
	unsigned int multiplier = 1;

	if (connected_speed == SPEED_1000)
		multiplier = 2;

	idle_slope = (4 * multiplier * bw);
	send_slope = ((4 * multiplier) - idle_slope);

	low_credit = -1*((int)(MAX_FRAME_SIZE * send_slope / (4 * multiplier) * 1024));
	syslog(LOG_DEBUG, "low credit = 0x%08x\n", low_credit);
	return low_credit;
}

int ntn_set_class_bandwidth(int nClass, unsigned classBytesPerSec, char *ifname)
{
	int ret;
	struct ifreq ifr;
	struct ifr_data_struct data;
	struct avb_algorithm avb_struct;
	int sockfd = -1;
	unsigned int classBitsPerSecond = classBytesPerSec * 8;
	float bw100 = 0;
	float bw1000 = 0;

	char *colon = strchr(ifname, ':');
	char *ifname_interface = NULL;
	if (colon) {
		ifname_interface = colon + 1;
	}
	else
		ifname_interface = ifname;

	if (classBitsPerSecond > 0) {
		classBitsPerSecond /= 1000 * 1000;
		classBitsPerSecond += 1;
		classBitsPerSecond *= 1000 * 1000;
	}

	bw1000 = ((float)classBitsPerSecond / 1000000) / 1000;
	bw100 = ((float)classBitsPerSecond / 1000000) / 100;

	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		syslog(LOG_ERR, "Configuring HW queue; can't open socket\n");
		return sockfd;
	}

	switch (nClass) {
	case SR_CLASS_A:
		avb_struct.chInx = 1;
		break;
	case SR_CLASS_B:
		avb_struct.chInx = 2;
		break;
	default:
		syslog(LOG_ERR, "Configuring HW queue; unsupported SR class (%d)\n", nClass);
		ret = -EINVAL;
		goto finish;
	}

	syslog(LOG_DEBUG, "Setting up hw queue: bw100 %f, bw1000 %f, index %d\n", bw100, bw1000, avb_struct.chInx);

	data.cmd = DWC_ETH_QOS_AVB_ALGORITHM;
	data.chInx = avb_struct.chInx;
	avb_struct.algorithm = classBitsPerSecond > 0 ? AVB_CBS : AVB_SP;
	avb_struct.cc = classBitsPerSecond > 0 ? 1 : 0;

	avb_struct.speed100params.idle_slope = get_idle_slope(bw100, SPEED_100);
	avb_struct.speed100params.send_slope = get_send_slope(bw100, SPEED_100);
	avb_struct.speed100params.hi_credit = get_hi_credit(bw100, SPEED_100, avb_struct.chInx);
	avb_struct.speed100params.low_credit = get_low_credit(bw100, SPEED_100, avb_struct.chInx);

	avb_struct.speed1000params.idle_slope = get_idle_slope(bw1000, SPEED_1000);
	avb_struct.speed1000params.send_slope = get_send_slope(bw1000, SPEED_1000);
	avb_struct.speed1000params.hi_credit = get_hi_credit(bw1000, SPEED_1000, avb_struct.chInx);
	avb_struct.speed1000params.low_credit = get_low_credit(bw1000, SPEED_1000, avb_struct.chInx);

	avb_struct.op_mode = QAVB;
	data.ptr = &avb_struct;
	strncpy(ifr.ifr_ifrn.ifrn_name, ifname_interface, IFNAMSIZ - 1);
	ifr.ifr_ifru.ifru_data = (void *)&data;

	errno = 0;
	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		syslog(LOG_ERR, "Configuring HW queue; ioctl failed (%d: %s)\n", errno, strerror(errno));
	else
		syslog(LOG_DEBUG, "Configured AVB Algorithm parameters successfully\n");

finish:
	close(sockfd);
	return ret;
}
