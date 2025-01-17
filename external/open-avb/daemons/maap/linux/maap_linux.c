/*************************************************************************
  Copyright (c) 2015 VAYAVYA LABS PVT LTD - http://vayavyalabs.com/
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

   3. Neither the name of the Vayavya labs nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

****************************************************************************/

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <linux/if.h>

#include <netinet/in.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <pthread.h>
#include <inttypes.h>
#include <unistd.h>

#ifdef USE_GLIB
#include <glib.h>
#define strlcpy g_strlcpy
#define strlcat g_strlcat
#endif

#include "maap_protocol.h"

uint8_t *maap_shm_mem;
struct sockaddr_ll saddrll;
pthread_t thread;
pthread_mutex_t lock;
int socketfd;

#define VERSION_STR	"0.0"

static const char *version_str =
    "maap_daemon v" VERSION_STR "\n" "Copyright (c) 2014, VAYAVYA LABS PVT LTD\n";

void usage(void)
{
	fprintf(stderr,
		"\n"
		"usage: maap_daemon [-d] -i interface-name"
		"\n"
		"options:\n"
		"    -d  run daemon in the background\n"
		"    -i  specify interface to monitor\n"
		"\n" "%s" "\n", version_str);
	exit(1);
}

int main(int argc, char *argv[]) 
{
	char *iface = NULL;
	struct ifreq buffer;
	int ifindex;
	struct packet_mreq mreq; 
	maap_info_t *maap_Info;
	uint32_t seed;
	uint8_t dest_mac[6];
	uint8_t src_mac[6];
	int daemonize = 0;
	int shmid;
	key_t key;
	int ret;
	int c;

		for (;;) {
		c = getopt(argc, argv, "hdi:");

		if (c < 0)
			break;

		switch (c) {
		case 'd':
			daemonize = 1;
			break;
		case 'i':
			if (iface) {
				printf
				    ("only one interface per daemon is supported\n");
				usage();
			}
			iface = strdup(optarg);
			break;
		case 'h':
		default:
			usage();
			break;
		}
	}
	if (optind < argc)
		usage();

	if (iface == NULL)
		usage();

	if (daemonize) {
		ret = daemon(1, 0);
		if (ret) {
			printf("Error: Failed to daemonize\n");
			return -1;
		}
	}

	if ((socketfd = socket(PF_PACKET, SOCK_RAW, htons(ETH_TYPE))) < 0 )
	{
		printf("Error: could not open socket %d\n",socketfd);
		return -1;
	}

	memset(&buffer, 0x00, sizeof(buffer));
	strlcpy(buffer.ifr_name, (char *)iface, IFNAMSIZ);
	if (ioctl(socketfd, SIOCGIFINDEX, &buffer) < 0)
	{
		printf("Error: could not get interface index\n");
		close(socketfd);
		return -1;
	}

	key = 1234;
	if ((shmid = shmget(key, MAC_ADDR_LEN, IPC_CREAT | 0666)) < 0) {
		printf("Error : Failed to allocate the Shared Memory\n");
		close(socketfd);
		return -1;
	}

	if ((maap_shm_mem = shmat(shmid, NULL, 0)) == (uint8_t *) -1) {
		printf("Error : Failed to attach the created segment id by function \
			shmget()");
		close(socketfd);
		return -1;
	}
  
	ifindex = buffer.ifr_ifindex;
	if (ioctl(socketfd, SIOCGIFHWADDR, &buffer) < 0) {
		printf("Error: could not get interface address\n");
		close(socketfd);
		return -1;
	}
	memcpy(src_mac, buffer.ifr_hwaddr.sa_data, MAC_ADDR_LEN);

	get_multicast_mac_adr(dest_mac);

	memset((void*)&saddrll, 0, sizeof(saddrll));
	saddrll.sll_family = AF_PACKET;
	saddrll.sll_ifindex = ifindex;
	saddrll.sll_halen = MAC_ADDR_LEN;
	memcpy((void*)(saddrll.sll_addr), (void*)dest_mac, MAC_ADDR_LEN);

	if (bind(socketfd, (struct sockaddr*)&saddrll, sizeof(saddrll))) {
		printf("Error: could not bind datagram socket\n");
		return -1;
	}

	/* filter multicast address */
	memset(&mreq, 0, sizeof(mreq));
	mreq.mr_ifindex = ifindex;
	mreq.mr_type = PACKET_MR_MULTICAST;
	mreq.mr_alen = 6;
	memcpy(mreq.mr_address, dest_mac, mreq.mr_alen);

	if (setsockopt(socketfd, SOL_PACKET, PACKET_ADD_MEMBERSHIP, &mreq,
		sizeof(mreq)) < 0) {
		printf("setsockopt PACKET_ADD_MEMBERSHIP failed\n");
		return -1;
	}

	pthread_mutex_init(&(lock), NULL);
	seed = src_mac[5] + time(NULL);
	srand(seed);

	maap_Info = (maap_info_t *)calloc(1,sizeof(maap_info_t));
	if (maap_Info == NULL) {
		printf("Error: out of memory\n");
		close(socketfd);
		return -1;
	}

	/* Initialize packet header and data */
	Init(maap_Info, src_mac);  

	while (1) {
		state_transit(maap_Info);
	}

	close(socketfd);

	free(maap_Info);

	return 0;
}

void delay(int seconds, int millisecond) 
{
	sleep(seconds);
	usleep(millisecond * 1000);
}

void create_thread(maap_info_t *maap_info, void *announce) 
{
	pthread_create(&thread, NULL, announce, (void *)(maap_info));
}

void destroy_thread() 
{
	pthread_cancel(thread); 
}

double rand_frange(double min_n, double max_n)
{
	return (double)rand()/RAND_MAX * (max_n - min_n) + min_n;
}

int rand_range(int min_n, int max_n)
{
	return rand() % (max_n - min_n + 1) + min_n;
}

void send_packet(ethpkt_t *pkt_tx) 
{
	int result;
	if ((result = (sendto(socketfd, pkt_tx, ETH_PKT_LEN, 0,
			(struct sockaddr*)&saddrll, sizeof(saddrll)) > 0)))
		DBG("send successful %d\n", result);
	else   
		DBG("Error: sending of packet failed\n");
}  

int recv_packet(ethpkt_t *pkt_rx, int flag)
{
	int result;
	socklen_t fmlen = sizeof(saddrll);

	if (flag == NON_BLOCK)
		flag = MSG_DONTWAIT;

	result = recvfrom(socketfd, pkt_rx, ETH_PKT_LEN, flag,
			(struct sockaddr*)&saddrll, &fmlen); 
	return result;
} 

void Lock()
{
	pthread_mutex_lock(&lock);
}

void UnLock()
{
	pthread_mutex_unlock(&lock);
}

uint16_t hton_s(uint16_t val)
{
	return htons(val);
}
