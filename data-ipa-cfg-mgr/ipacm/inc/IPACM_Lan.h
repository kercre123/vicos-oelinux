/*
Copyright (c) 2013-2017, The Linux Foundation. All rights reserved.

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
*/
/*!
	@file
	IPACM_Lan.h

	@brief
	This file implements the LAN iface definitions

	@Author
	Skylar Chang

*/
#ifndef IPACM_LAN_H
#define IPACM_LAN_H

#include <stdio.h>
#include <linux/msm_ipa.h>

#include "IPACM_CmdQueue.h"
#include "IPACM_Iface.h"
#include "IPACM_Routing.h"
#include "IPACM_Filtering.h"
#include "IPACM_Config.h"
#include "IPACM_Conntrack_NATApp.h"

#define IPA_WAN_DEFAULT_FILTER_RULE_HANDLES  1
#define IPA_PRIV_SUBNET_FILTER_RULE_HANDLES  3
#define IPA_NUM_ODU_ROUTE_RULES 2
#define MAX_WAN_UL_FILTER_RULES MAX_NUM_EXT_PROPS
#define NUM_IPV4_ICMP_FLT_RULE 1
#define NUM_IPV6_ICMP_FLT_RULE 1

/* ndc bandwidth ipatetherstats <ifaceIn> <ifaceOut> */
/* <in->out_bytes> <in->out_pkts> <out->in_bytes> <out->in_pkts */

#define PIPE_STATS "%s %s %lu %lu %lu %lu"
#define IPA_PIPE_STATS_FILE_NAME "/data/misc/ipa/tether_stats"

/* store each lan-iface unicast routing rule and its handler*/
struct ipa_lan_rt_rule
{
	ipa_ip_type ip;
	uint32_t v4_addr;
	uint32_t v4_addr_mask;
	uint32_t v6_addr[4];
	uint32_t rt_rule_hdl[0];
};

/* Support multiple eth client */
typedef struct _eth_client_rt_hdl
{
	uint32_t eth_rt_rule_hdl_v4;
	uint32_t eth_rt_rule_hdl_v6[IPV6_NUM_ADDR];
	uint32_t eth_rt_rule_hdl_v6_wan[IPV6_NUM_ADDR];
}eth_client_rt_hdl;

typedef struct _ipa_eth_client
{
	uint8_t mac[IPA_MAC_ADDR_SIZE];
	uint32_t v4_addr;
	uint32_t v6_addr[IPV6_NUM_ADDR][4];
	uint32_t hdr_hdl_v4;
	uint32_t hdr_hdl_v6;
	bool route_rule_set_v4;
	int route_rule_set_v6;
	bool ipv4_set;
	int ipv6_set;
	bool ipv4_header_set;
	bool ipv6_header_set;
#ifdef FEATURE_IPACM_PER_CLIENT_STATS
	bool ipv4_ul_rules_set;
	bool ipv6_ul_rules_set;
	/* store ipv4 UL filter rule handlers from Q6*/
	uint32_t wan_ul_fl_rule_hdl_v4[MAX_WAN_UL_FILTER_RULES];
	/* store ipv6 UL filter rule handlers from Q6*/
	uint32_t wan_ul_fl_rule_hdl_v6[MAX_WAN_UL_FILTER_RULES];
	int8_t lan_stats_idx;
#endif
	eth_client_rt_hdl eth_rt_hdl[0]; /* depends on number of tx properties */
}ipa_eth_client;

#ifdef FEATURE_IPACM_UL_FIREWALL
typedef struct ul_firewall {
	uint32_t ul_firewall_handle[IPACM_MAX_FIREWALL_ENTRIES];
	bool ul_firewall_installed;
	bool ul_catch_installed;
	bool ul_frag_installed;
	uint32_t ul_frag_handle;
} ul_firewall_t;
#endif

#ifdef FEATURE_IPACM_PER_CLIENT_STATS
/* store each lan client index along with MAC. */
typedef struct ipa_lan_client_idx
{
	int8_t lan_stats_idx;
	uint8_t mac[IPA_MAC_ADDR_SIZE];
	/* IPACM interface id */
	int ipa_if_num;
}ipa_lan_client_idx;
#endif

/* lan iface */
class IPACM_Lan : public IPACM_Iface
{
public:

	IPACM_Lan(int iface_index);
	~IPACM_Lan();

	/* store lan's wan-up filter rule handlers */
	uint32_t lan_wan_fl_rule_hdl[IPA_WAN_DEFAULT_FILTER_RULE_HANDLES];

	/* store private-subnet filter rule handlers */
	uint32_t private_fl_rule_hdl[IPA_MAX_PRIVATE_SUBNET_ENTRIES];

#ifdef FEATURE_IPACM_UL_FIREWALL
	ul_firewall_t iface_ul_firewall;
#endif
	/* Number of Q6 UL IPv4 rules. */
	int num_wan_ul_fl_rule_v4;
	/* Number of Q6 UL IPv6 rules. */
	int num_wan_ul_fl_rule_v6;

	/* Header length. */
	uint8_t hdr_len;

#ifdef FEATURE_IPACM_PER_CLIENT_STATS
	/* Clients which take HW path. */
	ipa_lan_client_idx active_lan_client_index[IPA_MAX_NUM_HW_PATH_CLIENTS];
	/* Clients which take SW path. This will be used as a place holder to move clients back to HW path. */
	ipa_lan_client_idx inactive_lan_client_index[IPA_MAX_NUM_HW_PATH_CLIENTS];
#endif

	/* LAN-iface's callback function */
	void event_callback(ipa_cm_event_id event, void *data);

	virtual int handle_wan_up(ipa_ip_type ip_type);

	/* configure filter rule for wan_up event*/
	virtual int handle_wan_up_ex(ipacm_ext_prop* ext_prop, ipa_ip_type iptype, uint8_t xlat_mux_id);

	/* delete filter rule for wan_down event*/
	virtual int handle_wan_down(bool is_sta_mode);

	/* delete filter rule for wan_down event*/
	virtual int handle_wan_down_v6(bool is_sta_mode);

	/* configure private subnet filter rules*/
	virtual int handle_private_subnet(ipa_ip_type iptype);

	/* handle new_address event*/
	int handle_addr_evt(ipacm_event_data_addr *data);

	int handle_addr_evt_odu_bridge(ipacm_event_data_addr* data);

	int handle_del_ipv6_addr(ipacm_event_data_all *data);

	static bool odu_up;

	/* install UL filter rule from Q6 */
	virtual int handle_uplink_filter_rule(ipacm_ext_prop* prop, ipa_ip_type iptype, uint8_t xlat_mux_id);

#ifdef FEATURE_IPACM_UL_FIREWALL
	/* Re Configure and install the UL firewall rules */
	virtual int re_config_dft_firewall_rules_ul(ipa_ip_type iptype, ul_firewall_t *ul_firewall);

	/* Configure and install the UL firewall rules on other BH */
	virtual int config_dft_firewall_rules_ul(struct ipa_flt_rule_add *rules, ipa_ip_type iptype, ul_firewall_t *ul_firewall);

	/* Config UL firewall filter rules on LTE BH */
	virtual int config_dft_firewall_rules_ul_ex(struct ipa_flt_rule_add *rules, ipa_ip_type iptype);

	/* Config UL frag firewall filter rules */
	virtual int config_wan_frag_firewall_rule_ul_ex(bool install, ipa_ip_type iptype, ul_firewall_t *ul_firewall);

	/* Send the UL firewall rules to Q6 via QMI */
	virtual int install_wan_firewall_rule_ul(bool enable, ipa_ip_type iptype);

	/* Delete UL firewall filter rules */
	int delete_uplink_filter_rule_ul(ipa_ip_type iptype, ul_firewall_t *ul_firewall);
#endif
#ifdef FEATURE_IPACM_PER_CLIENT_STATS

	/* handle lan client connect event. */
	virtual int handle_lan_client_connect(uint8_t *mac_addr);

	/* handle lan client disconnect event. */
	virtual int handle_lan_client_disconnect(uint8_t *mac_addr);

	/* install UL filter rule from Q6 per client */
	virtual int install_uplink_filter_rule_per_client
	(
		ipacm_ext_prop* prop,
		ipa_ip_type iptype,
		uint8_t xlat_mux_id,
		uint8_t *mac_addr
	);

	/* install UL filter rule from Q6 for all clients */
	virtual int install_uplink_filter_rule
	(
		ipacm_ext_prop* prop,
		ipa_ip_type iptype,
		uint8_t xlat_mux_id
	);

	/* Delete UL filter rule from Q6 for all clients */
	virtual int delete_uplink_filter_rule
	(
		ipa_ip_type iptype
	);

	/* Delet UL filter rule from Q6 per client */
	virtual int delete_uplink_filter_rule_per_client
	(
		ipa_ip_type iptype,
		uint8_t *mac_addr
	);

	/* set lan client info. */
	virtual int set_lan_client_info(struct wan_ioctl_lan_client_info *client_info);

	/* set lan client info. */
	virtual int clear_lan_client_info(struct wan_ioctl_lan_client_info *client_info);

	/* Enable per client stats. */
	virtual int enable_per_client_stats(bool *status);
#endif

	int handle_cradle_wan_mode_switch(bool is_wan_bridge_mode);

	int install_ipv4_icmp_flt_rule();


	/* add header processing context and return handle to lan2lan controller */
	int eth_bridge_add_hdr_proc_ctx(ipa_hdr_l2_type peer_l2_hdr_type, uint32_t *hdl);

	/* add routing rule and return handle to lan2lan controller */
	int eth_bridge_add_rt_rule(uint8_t *mac, char *rt_tbl_name, uint32_t hdr_proc_ctx_hdl,
		ipa_hdr_l2_type peer_l2_hdr_type, ipa_ip_type iptype, uint32_t *rt_rule_hdl, int *rt_rule_count);

	/* modify routing rule*/
	int eth_bridge_modify_rt_rule(uint8_t *mac, uint32_t hdr_proc_ctx_hdl,
		ipa_hdr_l2_type peer_l2_hdr_type, ipa_ip_type iptype, uint32_t *rt_rule_hdl, int rt_rule_count);

	/* add filtering rule and return handle to lan2lan controller */
	int eth_bridge_add_flt_rule(uint8_t *mac, uint32_t rt_tbl_hdl, ipa_ip_type iptype, uint32_t *flt_rule_hdl);

	/* delete filtering rule */
	int eth_bridge_del_flt_rule(uint32_t flt_rule_hdl, ipa_ip_type iptype);

	/* delete routing rule */
	int eth_bridge_del_rt_rule(uint32_t rt_rule_hdl, ipa_ip_type iptype);

	/* delete header processing context */
	int eth_bridge_del_hdr_proc_ctx(uint32_t hdr_proc_ctx_hdl);

	/* add l2tp rt rule for l2tp client */
	int add_l2tp_rt_rule(ipa_ip_type iptype, uint8_t *dst_mac, ipa_hdr_l2_type peer_l2_hdr_type,
		uint32_t l2tp_session_id, uint32_t vlan_id, uint8_t *vlan_client_mac, uint32_t *vlan_iface_ipv6_addr,
		uint32_t *vlan_client_ipv6_addr, uint32_t *first_pass_hdr_hdl, uint32_t *first_pass_hdr_proc_ctx_hdl,
		uint32_t *second_pass_hdr_hdl, int *num_rt_hdl, uint32_t *first_pass_rt_rule_hdl, uint32_t *second_pass_rt_rule_hdl);

	/* delete l2tp rt rule for l2tp client */
	int del_l2tp_rt_rule(ipa_ip_type iptype, uint32_t first_pass_hdr_hdl, uint32_t first_pass_hdr_proc_ctx_hdl,
		uint32_t second_pass_hdr_hdl, int num_rt_hdl, uint32_t *first_pass_rt_rule_hdl, uint32_t *second_pass_rt_rule_hdl);

	/* add l2tp rt rule for non l2tp client */
	int add_l2tp_rt_rule(ipa_ip_type iptype, uint8_t *dst_mac, uint32_t *hdr_proc_ctx_hdl,
		int *num_rt_hdl, uint32_t *rt_rule_hdl);

	/* delete l2tp rt rule for non l2tp client */
	int del_l2tp_rt_rule(ipa_ip_type iptype, int num_rt_hdl, uint32_t *rt_rule_hdl);

	/* add l2tp flt rule on l2tp interface */
	int add_l2tp_flt_rule(uint8_t *dst_mac, uint32_t *flt_rule_hdl);

	/* delete l2tp flt rule on l2tp interface */
	int del_l2tp_flt_rule(uint32_t flt_rule_hdl);

	/* add l2tp flt rule on non l2tp interface */
	int add_l2tp_flt_rule(ipa_ip_type iptype, uint8_t *dst_mac, uint32_t *vlan_client_ipv6_addr,
		uint32_t *first_pass_flt_rule_hdl, uint32_t *second_pass_flt_rule_hdl);

	/* delete l2tp flt rule on non l2tp interface */
	int del_l2tp_flt_rule(ipa_ip_type iptype, uint32_t first_pass_flt_rule_hdl, uint32_t second_pass_flt_rule_hdl);

protected:

	int each_client_rt_rule_count[IPA_IP_MAX];

	uint32_t eth_bridge_flt_rule_offset[IPA_IP_MAX];

	/* mac address has to be provided for client related events */
	void eth_bridge_post_event(ipa_cm_event_id evt, ipa_ip_type iptype, uint8_t *mac,
		uint32_t *ipv6_addr, char *iface_name);

	/* check if the event is associated with vlan interface */
	bool is_vlan_event(char *event_iface_name);

	/* check if the event is associated with l2tp interface */
	bool is_l2tp_event(char *event_iface_name);

	/* check if the IPv6 address is unique local address */
	bool is_unique_local_ipv6_addr(uint32_t *ipv6_addr);

	virtual int add_dummy_private_subnet_flt_rule(ipa_ip_type iptype);

	int handle_private_subnet_android(ipa_ip_type iptype);

	int reset_to_dummy_flt_rule(ipa_ip_type iptype, uint32_t rule_hdl);

	virtual int install_ipv6_prefix_flt_rule(uint32_t* prefix);

	virtual void delete_ipv6_prefix_flt_rule();

	int install_ipv6_icmp_flt_rule();

	void post_del_self_evt();

	/* handle tethering stats */
	int handle_tethering_stats_event(ipa_get_data_stats_resp_msg_v01 *data);

	/* handle tethering client */
	int handle_tethering_client(bool reset, ipacm_client_enum ipa_client);

	/* add tcp syn flt rule */
	int add_tcp_syn_flt_rule(ipa_ip_type iptype);

	/* add tcp syn flt rule for l2tp interface*/
	int add_tcp_syn_flt_rule_l2tp(ipa_ip_type inner_ip_type);

#ifdef FEATURE_IPACM_PER_CLIENT_STATS
	inline bool is_lan_stats_index_available()
	{
		int cnt;

		for(cnt = 0; cnt < IPA_MAX_NUM_HW_PATH_CLIENTS; cnt++)
		{
			if (active_lan_client_index[cnt].lan_stats_idx == -1) {
				IPACMDBG_H("Available free index :%d\n", cnt);
				return true;
			}
		}

		IPACMDBG_H("No free index available\n");
		return false;
	}

	inline int8_t get_free_active_lan_stats_index(uint8_t *mac_addr)
	{
		int cnt;

		if (!IPACM_Iface::ipacmcfg->ipacm_lan_stats_enable)
		{
			IPACMDBG_H("LAN stats functionality is not enabled.\n");
			return -1;
		}

		IPACMDBG_H("Received mac_addr MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
				mac_addr[0], mac_addr[1], mac_addr[2],
				mac_addr[3], mac_addr[4], mac_addr[5]);

		for(cnt = 0; cnt < IPA_MAX_NUM_HW_PATH_CLIENTS; cnt++)
		{
			if (active_lan_client_index[cnt].lan_stats_idx == -1) {
				IPACMDBG_H("Got active lan stats index :%d, reserve it\n", cnt);
				active_lan_client_index[cnt].lan_stats_idx = cnt;
				memcpy(active_lan_client_index[cnt].mac,
						mac_addr,
						IPA_MAC_ADDR_SIZE);
				active_lan_client_index[cnt].ipa_if_num = ipa_if_num;
				return cnt;
			}
		}

		IPACMDBG_H("index not available\n");
		return -1;
	}

	inline int8_t get_free_inactive_lan_stats_index(uint8_t *mac_addr)
	{
		int cnt;

		if (!IPACM_Iface::ipacmcfg->ipacm_lan_stats_enable)
		{
			IPACMDBG_H("LAN stats functionality is not enabled.\n");
			return -1;
		}

		IPACMDBG_H("Received mac_addr MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
				mac_addr[0], mac_addr[1], mac_addr[2],
				mac_addr[3], mac_addr[4], mac_addr[5]);

		for(cnt = 0; cnt < IPA_MAX_NUM_HW_PATH_CLIENTS; cnt++)
		{
			if (inactive_lan_client_index[cnt].lan_stats_idx == -1) {
				IPACMDBG_H("Got inactive lan stats index :%d, reserve it\n", cnt);
				inactive_lan_client_index[cnt].lan_stats_idx = cnt;
				memcpy(inactive_lan_client_index[cnt].mac,
						mac_addr,
						IPA_MAC_ADDR_SIZE);
				inactive_lan_client_index[cnt].ipa_if_num = ipa_if_num;
				return cnt;
			}
		}

		IPACMDBG_H("index not available\n");
		return -1;
	}

	inline int8_t get_lan_stats_index(uint8_t *mac_addr)
	{
		int cnt;

		if (!IPACM_Iface::ipacmcfg->ipacm_lan_stats_enable)
		{
			IPACMDBG_H("LAN stats functionality is not enabled.\n");
			return -1;
		}

		IPACMDBG_H("Received mac_addr MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
				mac_addr[0], mac_addr[1], mac_addr[2],
				mac_addr[3], mac_addr[4], mac_addr[5]);

		for(cnt = 0; cnt < IPA_MAX_NUM_HW_PATH_CLIENTS; cnt++)
		{
			if (memcmp(active_lan_client_index[cnt].mac,
						mac_addr,
						IPA_MAC_ADDR_SIZE) == 0) {
				IPACMDBG_H("Got lan stats index :%d, return\n", cnt);
				active_lan_client_index[cnt].lan_stats_idx = cnt;
				memcpy(active_lan_client_index[cnt].mac,
						mac_addr,
						IPA_MAC_ADDR_SIZE);
				return cnt;
			}
		}

		IPACMDBG_H("index not available\n");
		return -1;
	}

	inline int get_available_inactive_lan_client(uint8_t *mac_addr)
	{
		int cnt;

		if (!IPACM_Iface::ipacmcfg->ipacm_lan_stats_enable)
		{
			IPACMDBG_H("LAN stats functionality is not enabled.\n");
			return IPACM_FAILURE;
		}

		IPACMDBG_H("Received mac_addr MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
				mac_addr[0], mac_addr[1], mac_addr[2],
				mac_addr[3], mac_addr[4], mac_addr[5]);

		for(cnt = 0; cnt < IPA_MAX_NUM_HW_PATH_CLIENTS; cnt++)
		{
			if (inactive_lan_client_index[cnt].lan_stats_idx != -1) {
				IPACMDBG_H("Got inactive lan stats index :%d, return the mac\n", cnt);
				memcpy(mac_addr, inactive_lan_client_index[cnt].mac, IPA_MAC_ADDR_SIZE);
				return IPACM_SUCCESS;
			}
		}

		IPACMDBG_H("No inactive client\n");
		return IPACM_FAILURE;
	}

	inline int8_t reset_active_lan_stats_index(int8_t idx, uint8_t *mac_addr)
	{
		if (!IPACM_Iface::ipacmcfg->ipacm_lan_stats_enable)
		{
			IPACMDBG_H("LAN stats functionality is not enabled.\n");
			return IPACM_FAILURE;
		}

		IPACMDBG_H("Received mac_addr MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
				mac_addr[0], mac_addr[1], mac_addr[2],
				mac_addr[3], mac_addr[4], mac_addr[5]);

		if (idx < 0 || idx >= IPA_MAX_NUM_HW_PATH_CLIENTS ||
			memcmp(active_lan_client_index[idx].mac,
							mac_addr,
							IPA_MAC_ADDR_SIZE))
		{
			IPACMDBG_H("Index :%d invalid\n", idx);
			return IPACM_FAILURE;
		}
		memset(&active_lan_client_index[idx], -1, sizeof(ipa_lan_client_idx));
		return IPACM_SUCCESS;
	}

	inline int8_t reset_inactive_lan_stats_index(uint8_t *mac_addr)
	{
		int cnt;

		if (!IPACM_Iface::ipacmcfg->ipacm_lan_stats_enable)
		{
			IPACMDBG_H("LAN stats functionality is not enabled.\n");
			return IPACM_FAILURE;
		}

		IPACMDBG_H("Received mac_addr MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
				mac_addr[0], mac_addr[1], mac_addr[2],
				mac_addr[3], mac_addr[4], mac_addr[5]);

		for(cnt = 0; cnt < IPA_MAX_NUM_HW_PATH_CLIENTS; cnt++)
		{
			if (memcmp(inactive_lan_client_index[cnt].mac,
							mac_addr,
							IPA_MAC_ADDR_SIZE) == 0)
			{
				memset(&inactive_lan_client_index[cnt], -1, sizeof(ipa_lan_client_idx));
				return IPACM_SUCCESS;
			}
		}
		return IPACM_FAILURE;
	}

	inline void reset_lan_stats_index()
	{
		int i;

		if (!IPACM_Iface::ipacmcfg->ipacm_lan_stats_enable)
		{
			IPACMDBG_H("LAN stats functionality is not enabled.\n");
			return;
		}

		/* Reset everything based on ipa_if_num. */
		for (i = 0; i < IPA_MAX_NUM_HW_PATH_CLIENTS; i++)
		{
			if (active_lan_client_index[i].ipa_if_num == ipa_if_num)
				memset(&active_lan_client_index[i], -1, sizeof(ipa_lan_client_idx));
			if (inactive_lan_client_index[i].ipa_if_num == ipa_if_num)
				memset(&inactive_lan_client_index[i], -1, sizeof(ipa_lan_client_idx));
		}
	}

#endif

	/* store ipv4 UL filter rule handlers from Q6*/
	uint32_t wan_ul_fl_rule_hdl_v4[MAX_WAN_UL_FILTER_RULES];

	/* store ipv6 UL filter rule handlers from Q6*/
	uint32_t wan_ul_fl_rule_hdl_v6[MAX_WAN_UL_FILTER_RULES];

	uint32_t ipv4_icmp_flt_rule_hdl[NUM_IPV4_ICMP_FLT_RULE];

	uint32_t ipv6_prefix_flt_rule_hdl[NUM_IPV6_PREFIX_FLT_RULE];
	uint32_t ipv6_icmp_flt_rule_hdl[NUM_IPV6_ICMP_FLT_RULE];

	bool is_active;
	bool modem_ul_v4_set;
	bool modem_ul_v6_set;

	uint32_t if_ipv4_subnet;

	uint32_t ipv6_prefix[2];

	uint32_t tcp_syn_flt_rule_hdl[IPA_IP_MAX];

private:

	/* get hdr proc ctx type given source and destination l2 hdr type */
	ipa_hdr_proc_type eth_bridge_get_hdr_proc_type(ipa_hdr_l2_type t1, ipa_hdr_l2_type t2);

	/* get partial header (header template of hdr proc ctx) */
	int eth_bridge_get_hdr_template_hdl(uint32_t* hdr_hdl);


	/* dynamically allocate lan iface's unicast routing rule structure */

	bool is_mode_switch; /* indicate mode switch, need post internal up event */

	int eth_client_len;

	ipa_eth_client *eth_client;

	int header_name_count;

	int num_eth_client;

	NatApp *Nat_App;

	int ipv6_set;

	uint32_t ODU_hdr_hdl_v4, ODU_hdr_hdl_v6;

	uint32_t *odu_route_rule_v4_hdl;

	uint32_t *odu_route_rule_v6_hdl;

	bool ipv4_header_set;

	bool ipv6_header_set;

	inline ipa_eth_client* get_client_memptr(ipa_eth_client *param, int cnt)
	{
	    char *ret = ((char *)param) + (eth_client_len * cnt);
		return (ipa_eth_client *)ret;
	}

	inline int get_eth_client_index(uint8_t *mac_addr)
	{
		int cnt;
		int num_eth_client_tmp = num_eth_client;

		IPACMDBG_H("Passed MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
						 mac_addr[0], mac_addr[1], mac_addr[2],
						 mac_addr[3], mac_addr[4], mac_addr[5]);

		for(cnt = 0; cnt < num_eth_client_tmp; cnt++)
		{
			IPACMDBG_H("stored MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
							 get_client_memptr(eth_client, cnt)->mac[0],
							 get_client_memptr(eth_client, cnt)->mac[1],
							 get_client_memptr(eth_client, cnt)->mac[2],
							 get_client_memptr(eth_client, cnt)->mac[3],
							 get_client_memptr(eth_client, cnt)->mac[4],
							 get_client_memptr(eth_client, cnt)->mac[5]);

			if(memcmp(get_client_memptr(eth_client, cnt)->mac,
								mac_addr,
								sizeof(get_client_memptr(eth_client, cnt)->mac)) == 0)
			{
				IPACMDBG_H("Matched client index: %d\n", cnt);
				return cnt;
			}
		}

		return IPACM_INVALID_INDEX;
	}

	inline int delete_eth_rtrules(int clt_indx, ipa_ip_type iptype)
	{
		uint32_t tx_index;
		uint32_t rt_hdl;
		int num_v6;

		if(iptype == IPA_IP_v4)
		{
		    for(tx_index = 0; tx_index < iface_query->num_tx_props; tx_index++)
		    {
		        if((tx_prop->tx[tx_index].ip == IPA_IP_v4) && (get_client_memptr(eth_client, clt_indx)->route_rule_set_v4==true)) /* for ipv4 */
				{
					IPACMDBG_H("Delete client index %d ipv4 RT-rules for tx:%d\n",clt_indx,tx_index);
					rt_hdl = get_client_memptr(eth_client, clt_indx)->eth_rt_hdl[tx_index].eth_rt_rule_hdl_v4;

					if(m_routing.DeleteRoutingHdl(rt_hdl, IPA_IP_v4) == false)
					{
						return IPACM_FAILURE;
					}
				}
		    } /* end of for loop */

		     /* clean the ipv4 RT rules for eth-client:clt_indx */
		     if(get_client_memptr(eth_client, clt_indx)->route_rule_set_v4==true) /* for ipv4 */
		     {
				get_client_memptr(eth_client, clt_indx)->route_rule_set_v4 = false;
		     }
		}

		if(iptype == IPA_IP_v6)
		{
			for(tx_index = 0; tx_index < iface_query->num_tx_props; tx_index++)
			{
				if((tx_prop->tx[tx_index].ip == IPA_IP_v6) && (get_client_memptr(eth_client, clt_indx)->route_rule_set_v6 != 0)) /* for ipv6 */
				{
					for(num_v6 =0;num_v6 < get_client_memptr(eth_client, clt_indx)->route_rule_set_v6;num_v6++)
					{
						IPACMDBG_H("Delete client index %d ipv6 RT-rules for %d-st ipv6 for tx:%d\n", clt_indx,num_v6,tx_index);
						rt_hdl = get_client_memptr(eth_client, clt_indx)->eth_rt_hdl[tx_index].eth_rt_rule_hdl_v6[num_v6];
						if(m_routing.DeleteRoutingHdl(rt_hdl, IPA_IP_v6) == false)
							{
								return IPACM_FAILURE;
							}

							rt_hdl = get_client_memptr(eth_client, clt_indx)->eth_rt_hdl[tx_index].eth_rt_rule_hdl_v6_wan[num_v6];
							if(m_routing.DeleteRoutingHdl(rt_hdl, IPA_IP_v6) == false)
							{
								return IPACM_FAILURE;
							}
						}
                    }
		    } /* end of for loop */

		    /* clean the ipv6 RT rules for eth-client:clt_indx */
		    if(get_client_memptr(eth_client, clt_indx)->route_rule_set_v6 != 0) /* for ipv6 */
		    {
		        get_client_memptr(eth_client, clt_indx)->route_rule_set_v6 = 0;
            }
		}

		return IPACM_SUCCESS;
	}

	/* handle eth client initial, construct full headers (tx property) */
	int handle_eth_hdr_init(uint8_t *mac_addr);

	/* handle eth client ip-address */
	int handle_eth_client_ipaddr(ipacm_event_data_all *data);

	/* handle eth client routing rule*/
	int handle_eth_client_route_rule(uint8_t *mac_addr, ipa_ip_type iptype);

#ifdef FEATURE_IPACM_PER_CLIENT_STATS
	/* handle eth client routing rule with rule id*/
	int handle_eth_client_route_rule_ext(uint8_t *mac_addr, ipa_ip_type iptype);
#endif

	/*handle eth client del mode*/
	int handle_eth_client_down_evt(uint8_t *mac_addr);

	/* handle odu client initial, construct full headers (tx property) */
	int handle_odu_hdr_init(uint8_t *mac_addr);

	/* handle odu default route rule configuration */
	int handle_odu_route_add();

	/* handle odu default route rule deletion */
	int handle_odu_route_del();

	/*handle lan iface down event*/
	int handle_down_evt();

	/*handle reset usb-client rt-rules */
	int handle_lan_client_reset_rt(ipa_ip_type iptype);
#ifdef FEATURE_IPACM_UL_FIREWALL
	void change_to_network_order(ipa_ip_type iptype, ipa_rule_attrib* attrib);
#endif
};

#endif /* IPACM_LAN_H */
