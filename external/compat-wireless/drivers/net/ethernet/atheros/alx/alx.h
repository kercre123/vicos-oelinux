/*
 * Copyright (c) 2012 Qualcomm Atheros, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef _ALX_H_
#define _ALX_H_

#include <linux/types.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/in.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/sctp.h>
#include <linux/pkt_sched.h>
#include <linux/ipv6.h>
#include <linux/slab.h>
#include <net/checksum.h>
#include <net/ip6_checksum.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <linux/mii.h>
#include <linux/cpumask.h>
#include <linux/aer.h>
#include <linux/version.h>
#include <asm/byteorder.h>
#ifdef MDM_PLATFORM
#include <linux/ipa_odu_bridge.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0))
#include <linux/msm_pcie.h>
#else
#include <mach/msm_pcie.h>
#endif
#endif

#include "alx_sw.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0))
#define __devinit
#define __devexit
#define __devexit_p
#endif

#define ALX_LINK_DOWN_CONFIG 1
#define ALX_HIB_TASK_CONFIG  1
#define ALX_HIB_TIMER_CONFIG 1
#define MIN_TX_DESC 128
#define MAX_TX_DESC 1024
#ifdef MDM_PLATFORM
#define MAX_AR8151_BW 900
#define ALX_IPA_SYS_PIPE_MAX_PKTS_DESC 200
#define ALX_IPA_SYS_PIPE_MIN_PKTS_DESC 5
#define ALX_IPA_SYS_PIPE_DNE_PKTS ALX_IPA_SYS_PIPE_MAX_PKTS_DESC*3
#define ALX_IPA_INACTIVITY_DELAY_MS 100
/* Protocol Specific Offsets*/
#define ALX_IP_OFFSET       14
#define ALX_IP_HEADER_SIZE  20
#define ALX_DHCP_SRV_PORT   67
#define ALX_DHCP_CLI_PORT   68
#endif

/*
 * Definition to enable some features
 */
#undef CONFIG_ALX_MSIX
#undef CONFIG_ALX_MSI
#undef CONFIG_ALX_MTQ
#undef CONFIG_ALX_MRQ
#undef CONFIG_ALX_RSS
/* #define CONFIG_ALX_MSIX */
#define CONFIG_ALX_MSI
#define CONFIG_ALX_MTQ
#define CONFIG_ALX_MRQ
#ifdef CONFIG_ALX_MRQ
#define CONFIG_ALX_RSS
#endif

#define ALX_MSG_DEFAULT         0

/* Logging functions and macros */
#define alx_err(adpt, fmt, ...)	\
	netdev_err(adpt->netdev, fmt, ##__VA_ARGS__)

#define ALX_VLAN_TO_TAG(_vlan, _tag) \
	do { \
		_tag =  ((((_vlan) >> 8) & 0xFF) | (((_vlan) & 0xFF) << 8)); \
	} while (0)

#define ALX_TAG_TO_VLAN(_tag, _vlan) \
	do { \
		_vlan = ((((_tag) >> 8) & 0xFF) | (((_tag) & 0xFF) << 8)) ; \
	} while (0)

/* Coalescing Message Block */
struct coals_msg_block {
	int test;
};


#define BAR_0   0

#define ALX_DEF_RX_BUF_SIZE	1536
#define ALX_MAX_JUMBO_PKT_SIZE	(9*1024)
#define ALX_MAX_TSO_PKT_SIZE	(7*1024)

#define ALX_MAX_ETH_FRAME_SIZE	ALX_MAX_JUMBO_PKT_SIZE
#define ALX_MIN_ETH_FRAME_SIZE	68


#define ALX_MAX_RX_QUEUES	8
#define ALX_MAX_TX_QUEUES	4
#define ALX_MAX_HANDLED_INTRS	5

#define ALX_WATCHDOG_TIME   (5 * HZ)

struct alx_cmb {
	char name[IFNAMSIZ + 9];
	void *cmb;
	dma_addr_t dma;
};
struct alx_smb {
	char name[IFNAMSIZ + 9];
	void *smb;
	dma_addr_t dma;
};


/*
 * RRD : definition
 */

/* general parameter format of rrd */
struct alx_sw_rrdes_general {
#if defined(__LITTLE_ENDIAN_BITFIELD)
	/* dword 0 */
	u32  xsum:16;
	u32  nor:4;  /* number of RFD */
	u32  si:12;  /* start index of rfd-ring */
	/* dword 1 */
	u32 hash;
	/* dword 2 */
	u32 vlan_tag:16; /* vlan-tag */
	u32 pid:8;       /* Header Length of Header-Data Split. WORD unit */
	u32 reserve0:1;
	u32 rss_cpu:3;   /* CPU number used by RSS */
	u32 rss_flag:4;  /* rss_flag 0, TCP(IPv6) flag for RSS hash algrithm
			  * rss_flag 1, IPv6 flag for RSS hash algrithm
			  * rss_flag 2, TCP(IPv4) flag for RSS hash algrithm
			  * rss_flag 3, IPv4 flag for RSS hash algrithm */
	/* dword 3 */
	u32 pkt_len:14;  /* length of the packet */
	u32 l4f:1;       /* L4(TCP/UDP) checksum failed */
	u32 ipf:1;       /* IP checksum failed */
	u32 vlan_flag:1; /* vlan tag */
	u32 proto:3;
	u32 res:1;       /* received error summary */
	u32 crc:1;       /* crc error */
	u32 fae:1;       /* frame alignment error */
	u32 trunc:1;     /* truncated packet, larger than MTU */
	u32 runt:1;      /* runt packet */
	u32 icmp:1;      /* incomplete packet due to insufficient rx-desc*/
	u32 bar:1;       /* broadcast address received */
	u32 mar:1;       /* multicast address received */
	u32 type:1;      /* ethernet type */
	u32 fov:1;       /* fifo overflow*/
	u32 lene:1;      /* length error */
	u32 update:1;    /* update*/
#elif defined(__BIG_ENDIAN_BITFIELD)
	/* dword 0 */
	u32  si:12;
	u32  nor:4;
	u32  xsum:16;
	/* dword 1 */
	u32 hash;
	/* dword 2 */
	u32 rss_flag:4;
	u32 rss_cpu:3;
	u32 reserve0:1;
	u32 pid:8;
	u32 vlan_tag:16;
	/* dword 3 */
	u32 update:1;
	u32 lene:1;
	u32 fov:1;
	u32 type:1;
	u32 mar:1;
	u32 bar:1;
	u32 icmp:1;
	u32 runt:1;
	u32 trunc:1;
	u32 fae:1;
	u32 crc:1;
	u32 res:1;
	u32 proto:3;
	u32 vlan_flag:1;
	u32 ipf:1;
	u32 l4f:1;
	u32 pkt_len:14;
#else
#error	"Please fix <asm/byteorder.h>"
#endif
};

union alx_hw_rrdesc {
	/* dword flat format */
	struct {
		__le32 dw0;
		__le32 dw1;
		__le32 dw2;
		__le32 dw3;
	} dfmt;

	/* qword flat format */
	struct {
		__le64 qw0;
		__le64 qw1;
	} qfmt;
};

/*
 * XXX: we should not use this guy, best to just
 * do all le32_to_cpu() conversions on the spot.
 */
union alx_sw_rrdesc {
	struct alx_sw_rrdes_general genr;

	/* dword flat format */
	struct {
		u32 dw0;
		u32 dw1;
		u32 dw2;
		u32 dw3;
	} dfmt;

	/* qword flat format */
	struct {
		u64 qw0;
		u64 qw1;
	} qfmt;
};

/*
 * RFD : definition
 */

/* general parameter format of rfd */
struct alx_sw_rfdes_general {
	u64   addr;
};

union alx_hw_rfdesc {
	/* dword flat format */
	struct {
		__le32 dw0;
		__le32 dw1;
	} dfmt;

	/* qword flat format */
	struct {
		__le64 qw0;
	} qfmt;
};

/*
 * XXX: we should not use this guy, best to just
 * do all le32_to_cpu() conversions on the spot.
 */
union alx_sw_rfdesc {
	struct alx_sw_rfdes_general genr;

	/* dword flat format */
	struct {
		u32 dw0;
		u32 dw1;
	} dfmt;

	/* qword flat format */
	struct {
		u64 qw0;
	} qfmt;
};

/* RRD Proto Defination as per spec*/
enum alx_rrd_proto {
	RRD_PROTO_NON_IP = 0,
	RRD_PROTO_IPv4,
	RRD_PROTO_IPv6_TCP,
	RRD_PROTO_IPv4_TCP,
	RRD_PROTO_IPv6_UDP,
	RRD_PROTO_IPv4_UDP,
	RRD_PROTO_IPv6,
	RRD_PROTO_LLDP
};

/*
 * TPD : definition
 */

/* general parameter format of tpd */
struct alx_sw_tpdes_general {
#if defined(__LITTLE_ENDIAN_BITFIELD)
	/* dword 0 */
	u32  buffer_len:16; /* include 4-byte CRC */
	u32  vlan_tag:16;
	/* dword 1 */
	u32  l4hdr_offset:8; /* l4 header offset to the 1st byte of packet */
	u32  c_csum:1;
	u32  ip_csum:1;
	u32  tcp_csum:1;
	u32  udp_csum:1;
	u32  lso:1;
	u32  lso_v2:1;
	u32  vtagged:1;   /* vlan-id tagged already */
	u32  instag:1;    /* insert vlan tag */

	u32  ipv4:1;      /* ipv4 packet */
	u32  type:1;      /* type of packet (ethernet_ii(0) or snap(1)) */
	u32  reserve:12;
	u32  epad:1;      /* even byte padding when this packet */
	u32  last_frag:1; /* last fragment(buffer) of the packet */

	u64  addr;
#elif defined(__BIG_ENDIAN_BITFIELD)
	/* dword 0 */
	u32  vlan_tag:16;
	u32  buffer_len:16;
	/* dword 1 */
	u32  last_frag:1;
	u32  epad:1;
	u32  reserve:12;
	u32  type:1;
	u32  ipv4:1;
	u32  instag:1;
	u32  vtagged:1;
	u32  lso_v2:1;
	u32  lso:1;
	u32  udp_csum:1;
	u32  tcp_csum:1;
	u32  ip_csum:1;
	u32  c_csum:1;
	u32  l4hdr_offset:8;

	u64  addr;
#else
#error	"Please fix <asm/byteorder.h>"
#endif
};

/* custom checksum parameter format of tpd */
struct alx_sw_tpdes_checksum {
#if defined(__LITTLE_ENDIAN_BITFIELD)
	/* dword 0 */
	u32  buffer_len:16;
	u32  vlan_tag:16;
	/* dword 1 */
	u32  payld_offset:8; /* payload offset to the 1st byte of packet */
	u32  c_csum:1;    /* do custom checksum offload */
	u32  ip_csum:1;   /* do ip(v4) header checksum offload */
	u32  tcp_csum:1;  /* do tcp checksum offload, both ipv4 and ipv6 */
	u32  udp_csum:1;  /* do udp checksum offlaod, both ipv4 and ipv6 */
	u32  lso:1;
	u32  lso_v2:1;
	u32  vtagged:1;   /* vlan-id tagged already */
	u32  instag:1;    /* insert vlan tag */
	u32  ipv4:1;      /* ipv4 packet */
	u32  type:1;      /* type of packet (ethernet_ii(0) or snap(1)) */
	u32  cxsum_offset:8;  /* checksum offset to the 1st byte of packet */
	u32  reserve:4;
	u32  epad:1;      /* even byte padding when this packet */
	u32  last_frag:1; /* last fragment(buffer) of the packet */

	u64 addr;
#elif defined(__BIG_ENDIAN_BITFIELD)
	/* dword 0 */
	u32  vlan_tag:16;
	u32  buffer_len:16;
	/* dword 1 */
	u32  last_frag:1;
	u32  epad:1;
	u32  reserve:4;
	u32  cxsum_offset:8;
	u32  type:1;
	u32  ipv4:1;
	u32  instag:1;
	u32  vtagged:1;
	u32  lso_v2:1;
	u32  lso:1;
	u32  udp_csum:1;
	u32  tcp_csum:1;
	u32  ip_csum:1;
	u32  c_csum:1;
	u32  payld_offset:8;

	u64  addr;
#else
#error	"Please fix <asm/byteorder.h>"
#endif
};


/* tcp large send format (v1/v2) of tpd */
struct alx_sw_tpdes_tso {
#if defined(__LITTLE_ENDIAN_BITFIELD)
	/* dword 0 */
	u32  buffer_len:16; /* include 4-byte CRC */
	u32  vlan_tag:16;
	/* dword 1 */
	u32  tcphdr_offset:8; /* tcp hdr offset to the 1st byte of packet */
	u32  c_csum:1;
	u32  ip_csum:1;
	u32  tcp_csum:1;
	u32  udp_csum:1;
	u32  lso:1;       /* do tcp large send (ipv4 only) */
	u32  lso_v2:1;    /* must be 0 in this format */
	u32  vtagged:1;   /* vlan-id tagged already */
	u32  instag:1;    /* insert vlan tag */
	u32  ipv4:1;      /* ipv4 packet */
	u32  type:1;      /* type of packet (ethernet_ii(1) or snap(0)) */
	u32  mss:13;      /* mss if do tcp large send */
	u32  last_frag:1; /* last fragment(buffer) of the packet */

	u32  pkt_len;     /* packet length in ext tpd */
	u32  reserve;
#elif defined(__BIG_ENDIAN_BITFIELD)
	/* dword 0 */
	u32  vlan_tag:16;
	u32  buffer_len:16;
	/* dword 1 */
	u32  last_frag:1;
	u32  mss:13;
	u32  type:1;
	u32  ipv4:1;
	u32  instag:1;
	u32  vtagged:1;
	u32  lso_v2:1;
	u32  lso:1;
	u32  udp_csum:1;
	u32  tcp_csum:1;
	u32  ip_csum:1;
	u32  c_csum:1;
	u32  tcphdr_offset:8;

	u32  pkt_len;
	u32  reserve;
#else
#error	"Please fix <asm/byteorder.h>"
#endif
};

union alx_hw_tpdesc {
	/* dword flat format */
	struct {
		__le32 dw0;
		__le32 dw1;
		__le32 dw2;
		__le32 dw3;
	} dfmt;

	/* qword flat format */
	struct {
		__le64 qw0;
		__le64 qw1;
	} qfmt;
};

/*
 * XXX: we should not use this guy, best to just
 * do all le32_to_cpu() conversions on the spot.
 */
union alx_sw_tpdesc {
	struct alx_sw_tpdes_general   genr;
	struct alx_sw_tpdes_checksum  csum;
	struct alx_sw_tpdes_tso       tso;

	/* dword flat format */
	struct {
		u32 dw0;
		u32 dw1;
		u32 dw2;
		u32 dw3;
	} dfmt;

	/* qword flat format */
	struct {
		u64 qw0;
		u64 qw1;
	} qfmt;
};

#define ALX_RRD(_que, _i)	\
		(&(((union alx_hw_rrdesc *)(_que)->rrq.rrdesc)[(_i)]))
#define ALX_RFD(_que, _i)	\
		(&(((union alx_hw_rfdesc *)(_que)->rfq.rfdesc)[(_i)]))
#define ALX_TPD(_que, _i)	\
		(&(((union alx_hw_tpdesc *)(_que)->tpq.tpdesc)[(_i)]))


/*
 * alx_ring_header represents a single, contiguous block of DMA space
 * mapped for the three descriptor rings (tpd, rfd, rrd) and the two
 * message blocks (cmb, smb) described below
 */
struct alx_ring_header {
	void        *desc;      /* virtual address */
	dma_addr_t   dma;       /* physical address*/
	unsigned int size;      /* length in bytes */
	unsigned int used;
};


/*
 * alx_buffer is wrapper around a pointer to a socket buffer
 * so a DMA handle can be stored along with the skb
 */
struct alx_buffer {
	struct sk_buff *skb;      /* socket buffer */
	u16             length;   /* rx buffer length */
	dma_addr_t      dma;
};

struct alx_sw_buffer {
	struct sk_buff *skb;   /* socket buffer */
	u32             vlan_tag:16;
	u32             vlan_flag:1;
	u32             reserved:15;
};

/* receive free descriptor (rfd) queue */
struct alx_rfd_queue {
	struct alx_buffer   *rfbuff;
	union alx_hw_rfdesc *rfdesc;   /* virtual address */
	dma_addr_t         rfdma;    /* physical address */
	u16 size;          /* length in bytes */
	u16 count;         /* number of descriptors in the ring */
	u16 produce_idx;   /* it's written to rxque->produce_reg */
	u16 consume_idx;   /* unused*/
};

/* receive return desciptor (rrd) queue */
struct alx_rrd_queue {
	union alx_hw_rrdesc *rrdesc;    /* virtual address */
	dma_addr_t          rrdma;     /* physical address */
	u16 size;          /* length in bytes */
	u16 count;         /* number of descriptors in the ring */
	u16 produce_idx;   /* unused */
	u16 consume_idx;   /* rxque->consume_reg */
};

/* software desciptor (swd) queue */
struct alx_swd_queue {
	struct alx_sw_buffer *swbuff;
	u16 count;         /* number of descriptors in the ring */
	u16 produce_idx;
	u16 consume_idx;
};

/* rx queue */
struct alx_rx_queue {
	struct device         *dev;      /* device for dma mapping */
	struct net_device     *netdev;   /* netdev ring belongs to */
	struct alx_msix_param *msix;
	struct alx_rrd_queue   rrq;
	struct alx_rfd_queue   rfq;
	struct alx_swd_queue   swq;

	u16 que_idx;       /* index in multi rx queues*/
	u16 max_packets;   /* max work per interrupt */
	u16 produce_reg;
	u16 consume_reg;
	u32 flags;
};
#define ALX_RX_FLAG_SW_QUE          0x00000001
#define ALX_RX_FLAG_HW_QUE          0x00000002
#define CHK_RX_FLAG(_flag)          CHK_FLAG(rxque, RX, _flag)
#define SET_RX_FLAG(_flag)          SET_FLAG(rxque, RX, _flag)
#define CLI_RX_FLAG(_flag)          CLI_FLAG(rxque, RX, _flag)

#define GET_RF_BUFFER(_rque, _i)    (&((_rque)->rfq.rfbuff[(_i)]))
#define GET_SW_BUFFER(_rque, _i)    (&((_rque)->swq.swbuff[(_i)]))


/* transimit packet descriptor (tpd) ring */
struct alx_tpd_queue {
	struct alx_buffer   *tpbuff;
	union alx_hw_tpdesc *tpdesc;   /* virtual address */
	dma_addr_t         tpdma;    /* physical address */

	u16 size;    /* length in bytes */
	u16 count;   /* number of descriptors in the ring */
	u16 produce_idx;
	u16 consume_idx;
	u16 last_produce_idx;
};

/* tx queue */
struct alx_tx_queue {
	struct device         *dev;	/* device for dma mapping */
	struct net_device     *netdev;	/* netdev ring belongs to */
	struct alx_tpd_queue   tpq;
	struct alx_msix_param *msix;

	u16 que_idx;       /* needed for multiqueue queue management */
	u16 max_packets;   /* max packets per interrupt */
	u16 produce_reg;
	u16 consume_reg;
};
#define GET_TP_BUFFER(_tque, _i)    (&((_tque)->tpq.tpbuff[(_i)]))


/*
 * definition for array allocations.
 */
#define ALX_MAX_MSIX_INTRS              16
#define ALX_MAX_RX_QUEUES               8
#define ALX_MAX_TX_QUEUES               4

enum alx_msix_type {
	alx_msix_type_rx,
	alx_msix_type_tx,
	alx_msix_type_other,
};
#define ALX_MSIX_TYPE_OTH_TIMER         0
#define ALX_MSIX_TYPE_OTH_ALERT         1
#define ALX_MSIX_TYPE_OTH_SMB           2
#define ALX_MSIX_TYPE_OTH_PHY           3

/* ALX_MAX_MSIX_INTRS of these are allocated,
 * but we only use one per queue-specific vector.
 */
struct alx_msix_param {
	struct alx_adapter *adpt;
	unsigned int        vec_idx; /* index in HW interrupt vector */
	char                name[IFNAMSIZ + 9];

	/* msix interrupts for queue */
	u8 rx_map[ALX_MAX_RX_QUEUES];
	u8 tx_map[ALX_MAX_TX_QUEUES];
	u8 rx_count;   /* Rx ring count assigned to this vector */
	u8 tx_count;   /* Tx ring count assigned to this vector */

	struct napi_struct napi;
	cpumask_var_t      affinity_mask;
	u32 flags;
};

#define ALX_MSIX_FLAG_RX0               0x00000001
#define ALX_MSIX_FLAG_RX1               0x00000002
#define ALX_MSIX_FLAG_RX2               0x00000004
#define ALX_MSIX_FLAG_RX3               0x00000008
#define ALX_MSIX_FLAG_RX4               0x00000010
#define ALX_MSIX_FLAG_RX5               0x00000020
#define ALX_MSIX_FLAG_RX6               0x00000040
#define ALX_MSIX_FLAG_RX7               0x00000080
#define ALX_MSIX_FLAG_TX0               0x00000100
#define ALX_MSIX_FLAG_TX1               0x00000200
#define ALX_MSIX_FLAG_TX2               0x00000400
#define ALX_MSIX_FLAG_TX3               0x00000800
#define ALX_MSIX_FLAG_TIMER             0x00001000
#define ALX_MSIX_FLAG_ALERT             0x00002000
#define ALX_MSIX_FLAG_SMB               0x00004000
#define ALX_MSIX_FLAG_PHY               0x00008000

#define ALX_MSIX_FLAG_RXS (\
		ALX_MSIX_FLAG_RX0       |\
		ALX_MSIX_FLAG_RX1       |\
		ALX_MSIX_FLAG_RX2       |\
		ALX_MSIX_FLAG_RX3       |\
		ALX_MSIX_FLAG_RX4       |\
		ALX_MSIX_FLAG_RX5       |\
		ALX_MSIX_FLAG_RX6       |\
		ALX_MSIX_FLAG_RX7)
#define ALX_MSIX_FLAG_TXS (\
		ALX_MSIX_FLAG_TX0       |\
		ALX_MSIX_FLAG_TX1       |\
		ALX_MSIX_FLAG_TX2       |\
		ALX_MSIX_FLAG_TX3)
#define ALX_MSIX_FLAG_ALL (\
		ALX_MSIX_FLAG_RXS       |\
		ALX_MSIX_FLAG_TXS       |\
		ALX_MSIX_FLAG_TIMER     |\
		ALX_MSIX_FLAG_ALERT     |\
		ALX_MSIX_FLAG_SMB       |\
		ALX_MSIX_FLAG_PHY)

#define CHK_MSIX_FLAG(_flag)    CHK_FLAG(msix, MSIX, _flag)
#define SET_MSIX_FLAG(_flag)    SET_FLAG(msix, MSIX, _flag)
#define CLI_MSIX_FLAG(_flag)    CLI_FLAG(msix, MSIX, _flag)

#ifdef MDM_PLATFORM
/**
 *  * struct alx_ipa_stats - ALX - ODU_Bridge/IPA Stats
 *  * @send_msg: MSG Send to ODU_Bridge
 *  * @recv_msg: MSG Recd from ODU_Bridge/IPA
 *  * @rx_ipa_excep: Exception packets send from ODU_Bridge/IPA; packets
 *  *                that need to be delivered to network stack
 *  * @rx_ipa_write_done: MSG Recd from ODU_Bridge/IPA when message posted to
 *  *			  IPA HW
 *  * This function sets the link local ipv6 address provided by IOCTL
 *  */
struct alx_ipa_stats {
	/* RX Side */
	uint64_t rx_ipa_excep;
	uint64_t rx_ipa_write_done;
	uint64_t rx_ipa_send;
	uint64_t rx_ipa_send_fail;

	/* TX Side*/
	uint64_t tx_ipa_send;
	uint64_t tx_ipa_send_err;

	/* Frag Stats */
	uint64_t non_ip_frag_pkt;

	/* Flow Control Stats */
	uint64_t flow_control_pkt_drop;
	uint64_t ipa_low_watermark_cnt;
};

enum alx_ipa_rm_state {
	ALX_IPA_RM_RELEASED,
	ALX_IPA_RM_REQUESTED,
	ALX_IPA_RM_GRANT_PENDING,
	ALX_IPA_RM_GRANTED,
};

/**
 * struct alx_ipa_ctx - ALX IPA Context
 *  @stats: ALX - IPA brigde stats
 *  @debugfs_dir: Debug FS handle for alx
 *  @ipa_prod_rm_state: IPA Producer Pipe RM state
 *  @ipa_cons_rm_state: IPA Consumer Pipe RM state
 *  @alx_ipa_perf_requested: Set to true when perf profile have been requested.
 *  @ipa_rm_lock: Lock to syncronize IPA Prod/Cons RM state access
 *  @rm_ipa_lock: Lock to syncronize ipa_rx_completion access
 *  @ipa_rx_completion: Keeps track of pending IPA WRITE DONE Evts
 *  @ipa_comp_wait: Wait source used to keep APPS awake
 *                  when packets are submitted to IPA
 **/
struct alx_ipa_ctx {
	struct alx_ipa_stats stats;
	struct dentry *debugfs_dir;
	enum alx_ipa_rm_state ipa_prod_rm_state;
	enum alx_ipa_rm_state ipa_cons_rm_state;
	bool alx_ipa_perf_requested;
	spinlock_t ipa_rm_state_lock;
	spinlock_t rm_ipa_lock;
	uint64_t ipa_rx_completion;
	uint64_t alx_tx_completion;
	bool acquire_wake_src;
	struct wakeup_source rm_ipa_wait;
	bool ipa_ready;
};

struct alx_ipa_rx_desc_node {
	struct list_head link;
	struct sk_buff * skb_ptr;
};
#endif

/*
 *board specific private data structure
 */
struct alx_adapter {
	struct net_device *netdev;
	struct pci_dev    *pdev;
#ifdef  MDM_PLATFORM
	struct alx_ipa_ctx *palx_ipa;
	struct msm_pcie_register_event msm_pcie_event;
#endif
	struct net_device_stats net_stats;
	bool netdev_registered;
	u16 bd_number;    /* board number;*/

	struct alx_msix_param *msix[ALX_MAX_MSIX_INTRS];
	struct msix_entry     *msix_entries;
	int num_msix_rxques;
	int num_msix_txques;
	int num_msix_noques;    /* true count of msix_noques for device */
	int num_msix_intrs;

	int min_msix_intrs;
	int max_msix_intrs;

	/* All Descriptor memory */
	struct alx_ring_header ring_header;

	/* TX */
	struct alx_tx_queue *tx_queue[ALX_MAX_TX_QUEUES];
	/* RX */
	struct alx_rx_queue *rx_queue[ALX_MAX_RX_QUEUES];

	u16 num_txques;
	u16 num_rxques; /* equal max(num_hw_rxques, num_sw_rxques) */
	u16 num_hw_rxques;
	u16 num_sw_rxques;
	u16 max_rxques;
	u16 max_txques;

	u16 num_txdescs;
	u16 num_rxdescs;

	u32 rxbuf_size;

	struct alx_cmb cmb;
	struct alx_smb smb;

	/* structs defined in alx_hw.h */
	struct alx_hw       hw;
	struct alx_hw_stats hw_stats;

	u32 *config_space;

	struct work_struct alx_task;
	struct timer_list  alx_timer;

	unsigned long link_jiffies;

	u32 wol;
	spinlock_t tx_lock;
	spinlock_t rx_lock;
	atomic_t irq_sem;

#ifdef  MDM_PLATFORM
	u16 ipa_high_watermark;
	u16 ipa_low_watermark;
	u16 pendq_cnt;
	u16 freeq_cnt;
	u16 ipa_free_desc_cnt;
	spinlock_t flow_ctrl_lock;
	struct list_head pend_queue_head;
	struct list_head free_queue_head;
	struct work_struct ipa_send_task;
#endif

	u16 msg_enable;
#ifdef MDM_PLATFORM
	unsigned long flags[3];
#else
	unsigned long flags[2];
#endif
};


#define ALX_ADPT_FLAG_0_MSI_CAP                 0x00000001
#define ALX_ADPT_FLAG_0_MSI_EN                  0x00000002
#define ALX_ADPT_FLAG_0_MSIX_CAP                0x00000004
#define ALX_ADPT_FLAG_0_MSIX_EN                 0x00000008
#define ALX_ADPT_FLAG_0_MRQ_CAP                 0x00000010
#define ALX_ADPT_FLAG_0_MRQ_EN                  0x00000020
#define ALX_ADPT_FLAG_0_MTQ_CAP                 0x00000040
#define ALX_ADPT_FLAG_0_MTQ_EN                  0x00000080
#define ALX_ADPT_FLAG_0_SRSS_CAP                0x00000100
#define ALX_ADPT_FLAG_0_SRSS_EN                 0x00000200
#define ALX_ADPT_FLAG_0_FIXED_MSIX              0x00000400

#define ALX_ADPT_FLAG_0_TASK_REINIT_REQ         0x00010000  /* reinit */
#define ALX_ADPT_FLAG_0_TASK_LSC_REQ            0x00020000

#define ALX_ADPT_FLAG_1_STATE_TESTING           0x00000001
#define ALX_ADPT_FLAG_1_STATE_RESETTING         0x00000002
#define ALX_ADPT_FLAG_1_STATE_DOWN              0x00000004
#define ALX_ADPT_FLAG_1_STATE_WATCH_DOG         0x00000008
#define ALX_ADPT_FLAG_1_STATE_DIAG_RUNNING      0x00000010
#define ALX_ADPT_FLAG_1_STATE_INACTIVE          0x00000020

#ifdef MDM_PLATFORM
#define ALX_ADPT_FLAG_2_ODU_CONNECT             0x00000001
#define ALX_ADPT_FLAG_2_IPA_RM                  0x00000002
#define ALX_ADPT_FLAG_2_DEBUGFS_INIT            0x00000004
#define ALX_ADPT_FLAG_2_ODU_INIT                0x00000008
#define ALX_ADPT_FLAG_2_WQ_SCHED                0x00000010
#endif

#define CHK_ADPT_FLAG(_idx, _flag)	\
		CHK_FLAG_ARRAY(adpt, _idx, ADPT, _flag)
#define SET_ADPT_FLAG(_idx, _flag)	\
		SET_FLAG_ARRAY(adpt, _idx, ADPT, _flag)
#define CLI_ADPT_FLAG(_idx, _flag)	\
		CLI_FLAG_ARRAY(adpt, _idx, ADPT, _flag)

/* default to trying for four seconds */
#define ALX_TRY_LINK_TIMEOUT (4 * HZ)


#define ALX_OPEN_CTRL_IRQ_EN            0x00000001
#define ALX_OPEN_CTRL_RESET_MAC         0x00000002
#define ALX_OPEN_CTRL_RESET_PHY         0x00000004
#define ALX_OPEN_CTRL_RESET_ALL (\
		ALX_OPEN_CTRL_RESET_MAC         |\
		ALX_OPEN_CTRL_RESET_PHY)

/* needed by alx_ethtool.c */
extern char alx_drv_name[];
extern void alx_reinit_locked(struct alx_adapter *adpt);
extern void alx_set_ethtool_ops(struct net_device *netdev);
extern void alx_update_hw_stats(struct alx_adapter *adpt);
int alx_resize_rings(struct net_device *netdev);
int alx_open_internal(struct alx_adapter *adpt, u32 ctrl);
void alx_stop_internal(struct alx_adapter *adpt, u32 ctrl);

#ifdef ETHTOOL_OPS_COMPAT
extern int ethtool_ioctl(struct ifreq *ifr);
#endif

#endif /* _ALX_H_ */
