/*******************************************************************************
 Copyright (C) 2016, STMicroelectronics International N.V.
 All rights reserved.

 Use and Redistribution are permitted only in accordance with licensing terms at www.st.com
 and provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROTECTED BY STMICROELECTRONICS PATENT AND COPYRIGHTS.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF TCLOCK_MONOTICHE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/netlink.h>
#include <string.h>


/* This Define is used to identify the netlink protocol.
*/
#ifndef NETLINK_PROTOCOL_TYPE
#	define NETLINK_PROTOCOL_TYPE \
		"/sys/module/stmvl53l1/parameters/netlink_protocol_type"
#endif

/* This Define is used to identify the netlink protocol number.
*/
#ifndef STMVL531_CFG_NETLINK_USER
#define STMVL531_CFG_NETLINK_USER	23
#endif

/* This is a mono process/mono thread implementation of a netlink daemon
 * for vl53l1 device. It process one message at a time and then reply with
 * status and output of procedure call.
*/

//#define DEBUG	1
#ifdef DEBUG
#	define IPP_PRINT(...) printf(__VA_ARGS__)
#else
#	define IPP_PRINT(...) (void)0
#endif

#include "stmvl53l1_ipp.h"
#include "vl53l1_hist_funcs.h"
#include "vl53l1_xtalk.h"


#define nl_err(fmt, ...) fprintf(stderr, "ERROR : " fmt "\n", ##__VA_ARGS__)
#define nl_inf(fmt, ...) fprintf(stderr, "INFO  : " fmt "\n", ##__VA_ARGS__)
#ifdef DEBUG
#	define nl_dbg(fmt, ...) fprintf(stderr, "DEBUG :  " fmt "\n", ##__VA_ARGS__)
#else
#	define nl_dbg(fmt, ...) (void)0
#endif

static int input_payload[stmvl53l1_ipp_max];
static int nl_socket;
static struct msghdr msg_rcv;
static struct msghdr msg_snd;
static struct iovec iov_rcv;
static struct iovec iov_snd;
static struct nlmsghdr *nlh_msg_rcv = NULL;
static struct nlmsghdr *nlh_msg_snd = NULL;

static void ipp_reply_error(int err)
{
	struct ipp_work_t *p_in = NLMSG_DATA(iov_rcv.iov_base);
	struct ipp_work_t *p_out = NLMSG_DATA(iov_snd.iov_base);
	int rc;

	IPP_CPY_HEADER(p_out, p_in);
	p_out->status= err;
	p_out->payload = IPP_WORK_HDR_SIZE;

	iov_snd.iov_len = NLMSG_SPACE(p_out->payload);
	nl_dbg("reply with error %d", err);
	rc = sendmsg(nl_socket, &msg_snd, 0);
	if (rc <= 0)
		nl_err("fail to send error message(%d) => %d", err, rc);
}

static void ipp_reply_ok(int payload_size)
{
	struct ipp_work_t *p_in = NLMSG_DATA(iov_rcv.iov_base);
	struct ipp_work_t *p_out = NLMSG_DATA(iov_snd.iov_base);
	int rc;

	IPP_CPY_HEADER(p_out, p_in);
	p_out->payload = payload_size;
	p_out->status = 0;

	iov_snd.iov_len = NLMSG_SPACE(p_out->payload);
	nl_dbg("reply with success => %d bytes payload", p_out->payload);
	rc = sendmsg(nl_socket, &msg_snd, 0);
	if (rc <= 0)
		nl_err("fail to send ok message(%d) => %d", payload_size, rc);
}

static void ipp_reply(int rc, int payload_size)
{
	if (rc) {
		ipp_reply_error(rc + stmvl53l1_ipp_status_proc_code);
	} else {
		ipp_reply_ok(payload_size);
	}
}

static int is_bad_input_payload(int process_no)
{
	int res = 0;
	int payload = input_payload[process_no];
	struct ipp_work_t *p_in = NLMSG_DATA(iov_rcv.iov_base);

	if (p_in->payload < payload || p_in->payload > NLMSG_PAYLOAD((struct nlmsghdr *) iov_rcv.iov_base, 0)) {
		nl_err("baldy format hist paylaod bad %d < %d", p_in->payload, payload);
		ipp_reply_error(stmvl53l1_ipp_status_inv_payload);
	}

	return res;
}

static void fill_ipp_cal_hist_input_payload(int *payload)
{
	struct ipp_work_t pw;
	IPP_SERIALIZE_VAR;

	IPP_SERIALIZE_START(pw.data,5);
	*payload = (int)IPP_SERIALIZE_PAYLAOD();
	*payload += (int)IPP_ALIGN_OFFSET(sizeof(VL53L1_dmax_calibration_data_t));
	*payload += (int)IPP_ALIGN_OFFSET(sizeof(VL53L1_hist_gen3_dmax_config_t));
	*payload += (int)IPP_ALIGN_OFFSET(sizeof(VL53L1_hist_post_process_config_t));
	*payload += (int)IPP_ALIGN_OFFSET(sizeof(VL53L1_histogram_bin_data_t));
	*payload += (int)IPP_ALIGN_OFFSET(sizeof(VL53L1_xtalk_histogram_data_t));
}

static void ipp_call_hist_dser(int process_no)
/*	VL53L1_API VL53L1_Error VL53L1_hist_process_data(
	VL53L1_dmax_calibration_data_t    *pdmax_cal,
	VL53L1_hist_gen3_dmax_config_t    *pdmax_cfg,
	VL53L1_hist_post_process_config_t *ppost_cfg,
	VL53L1_histogram_bin_data_t       *pbins,
	VL53L1_xtalk_histogram_data_t     *pxtalk,
	VL53L1_range_results_t            *presults);
*/
{
	int rc;
	IPP_SERIALIZE_VAR;
	VL53L1_dmax_calibration_data_t    *pdmax_cal;
	VL53L1_hist_gen3_dmax_config_t    *pdmax_cfg;
	VL53L1_hist_post_process_config_t *ppost_cfg;
	VL53L1_histogram_bin_data_t       *pbins;
	VL53L1_xtalk_histogram_data_t     *pxtalk;
	VL53L1_range_results_t            *presults;
	struct ipp_work_t *p_in = NLMSG_DATA(iov_rcv.iov_base);
	struct ipp_work_t *p_out = NLMSG_DATA(iov_snd.iov_base);


	/* check payload is valid */
	if (is_bad_input_payload(process_no))
		return ;
	/* setup input */
	IPP_GET_ARG_PTR(p_in->data, 0, pdmax_cal);
	IPP_GET_ARG_PTR(p_in->data, 1, pdmax_cfg);
	IPP_GET_ARG_PTR(p_in->data, 2, ppost_cfg);
	IPP_GET_ARG_PTR(p_in->data, 3, pbins);
	IPP_GET_ARG_PTR(p_in->data, 4, pxtalk);
	/* setup output */
	IPP_SERIALIZE_START(p_out->data,1);
	IPP_OUT_ARG_PTR(p_out->data, 0, presults);

	rc = VL53L1_hist_process_data(pdmax_cal, pdmax_cfg, ppost_cfg, pbins,
		pxtalk, presults);
	ipp_reply(rc,  IPP_SERIALIZE_PAYLAOD());
}

static void fill_ipp_xtalk_calibration_input_payload(int *payload)
{
	struct ipp_work_t pw;
	IPP_SERIALIZE_VAR;

	IPP_SERIALIZE_START(pw.data,1);
	*payload = (int)IPP_SERIALIZE_PAYLAOD();
	*payload += (int)IPP_ALIGN_OFFSET(sizeof(VL53L1_xtalk_calibration_results_t));
}

static void ipp_call_xtalk_dser(int process_no)
/*	VL53L1_API VL53L1_Error VL53L1_xtalk_calibration_process_data(
	VL53L1_xtalk_range_results_t         *pxtalk_ranges,
	VL53L1_xtalk_histogram_data_t        *pxtalk_shape,
	VL53L1_xtalk_calibration_results_t   *pxtalk_cal);
*/
{
	int rc;
	IPP_SERIALIZE_VAR;
	VL53L1_xtalk_range_results_t *pxtalk_ranges;
	VL53L1_xtalk_histogram_data_t *pxtalk_shape;
	VL53L1_xtalk_calibration_results_t *pxtalk_cal;
	struct ipp_work_t *p_in = NLMSG_DATA(iov_rcv.iov_base);
	struct ipp_work_t *p_out = NLMSG_DATA(iov_snd.iov_base);

	/* check payload is valid */
	if (is_bad_input_payload(process_no))
		return ;
	/* setup input */
	IPP_GET_ARG_PTR(p_in->data, 0, pxtalk_ranges);
	/* setup output */
	IPP_SERIALIZE_START(p_out->data,2);
	IPP_OUT_ARG_PTR(p_out->data, 0, pxtalk_shape);
	IPP_OUT_ARG_PTR(p_out->data, 1, pxtalk_cal);

	rc = VL53L1_xtalk_calibration_process_data(pxtalk_ranges, pxtalk_shape,
		pxtalk_cal);

	ipp_reply(rc,  IPP_SERIALIZE_PAYLAOD());
}

static void fill_ipp_hist_ambient_dmax_input_payload(int *payload)
{
	struct ipp_work_t pw;
	IPP_SERIALIZE_VAR;

	IPP_SERIALIZE_START(pw.data, 4);
	*payload = (int)IPP_SERIALIZE_PAYLAOD();
	*payload += (int)IPP_ALIGN_OFFSET(sizeof(uint16_t));
	*payload += (int)IPP_ALIGN_OFFSET(sizeof(VL53L1_dmax_calibration_data_t));
	*payload += (int)IPP_ALIGN_OFFSET(sizeof(VL53L1_hist_gen3_dmax_config_t));
	*payload += (int)IPP_ALIGN_OFFSET(sizeof(VL53L1_histogram_bin_data_t));
}

static void ipp_call_dmax_dser(int process_no)
/*	VL53L1_Error VL53L1_hist_ambient_dmax(
	uint16_t                            target_reflectance,
	VL53L1_dmax_calibration_data_t     *pdmax_cal,
	VL53L1_hist_gen3_dmax_config_t     *pdmax_cfg,
	VL53L1_histogram_bin_data_t        *pbins,
	int16_t                            *pambient_dmax_mm);
*/
{
	int rc;
	IPP_SERIALIZE_VAR;
	uint16_t target_reflectance;
	VL53L1_dmax_calibration_data_t *pdmax_cal;
	VL53L1_hist_gen3_dmax_config_t *pdmax_cfg;
	VL53L1_histogram_bin_data_t *pbins;
	int16_t *pambient_dmax_mm;
	struct ipp_work_t *p_in = NLMSG_DATA(iov_rcv.iov_base);
	struct ipp_work_t *p_out = NLMSG_DATA(iov_snd.iov_base);

	/* check payload is valid */
	if (is_bad_input_payload(process_no))
		return ;
	/* setup input */
	IPP_GET_ARG(p_in->data, 0, target_reflectance);
	IPP_GET_ARG_PTR(p_in->data, 1, pdmax_cal);
	IPP_GET_ARG_PTR(p_in->data, 2, pdmax_cfg);
	IPP_GET_ARG_PTR(p_in->data, 3, pbins);
	/* setup output */
	IPP_SERIALIZE_START(p_out->data,1);
	IPP_OUT_ARG_PTR(p_out->data, 0, pambient_dmax_mm);

	rc = VL53L1_hist_ambient_dmax(target_reflectance, pdmax_cal, pdmax_cfg,
		pbins, pambient_dmax_mm);

	ipp_reply(rc,  IPP_SERIALIZE_PAYLAOD());
}

static void fill_ipp_generate_dual_reflectance_xtalk_samples_input_payload(int *payload)
{
	struct ipp_work_t pw;
	IPP_SERIALIZE_VAR;

	IPP_SERIALIZE_START(pw.data, 3);
	*payload = (int)IPP_SERIALIZE_PAYLAOD();
	*payload += (int)IPP_ALIGN_OFFSET(sizeof(VL53L1_xtalk_range_results_t));
	*payload += (int)IPP_ALIGN_OFFSET(sizeof(uint16_t));
	*payload += (int)IPP_ALIGN_OFFSET(sizeof(uint8_t));
}

static void ipp_call_dual_xtalk_dser(int process_no)
/*	VL53L1_Error VL53L1_generate_dual_reflectance_xtalk_samples (
	VL53L1_xtalk_range_results_t *pxtalk_results,
	uint16_t                      expected_target_distance_mm,
	uint8_t                       higher_reflectance,
	VL53L1_histogram_bin_data_t  *pxtalk_avg_samples);
*/
{
	int rc;
	IPP_SERIALIZE_VAR;
	VL53L1_xtalk_range_results_t *pxtalk_results;
	uint16_t expected_target_distance_mm;
	uint8_t higher_reflectance;
	VL53L1_histogram_bin_data_t *pxtalk_avg_samples;
	struct ipp_work_t *p_in = NLMSG_DATA(iov_rcv.iov_base);
	struct ipp_work_t *p_out = NLMSG_DATA(iov_snd.iov_base);

	/* check payload is valid */
	if (is_bad_input_payload(process_no))
		return ;
	/* setup input */
	IPP_GET_ARG_PTR(p_in->data, 0, pxtalk_results);
	IPP_GET_ARG(p_in->data, 1, expected_target_distance_mm);
	IPP_GET_ARG(p_in->data, 2, higher_reflectance);
	/* setup output */
	IPP_SERIALIZE_START(p_out->data,1);
	IPP_OUT_ARG_PTR(p_out->data, 0, pxtalk_avg_samples);

	rc = VL53L1_generate_dual_reflectance_xtalk_samples(pxtalk_results,
		expected_target_distance_mm, higher_reflectance, pxtalk_avg_samples);

	ipp_reply(rc,  IPP_SERIALIZE_PAYLAOD());
}

static int get_driver_netlink_protocol_type()
{
	int fd;
	int res = STMVL531_CFG_NETLINK_USER;

	fd = open(NETLINK_PROTOCOL_TYPE, O_RDONLY);
	if (fd >= 0) {
		char buf[32];

		if (read(fd, buf, sizeof(buf)) > 0) {
			res = atoi(buf);
		}
	}

	return res;
}

static void fill_ipp_ping_input_payload(int *payload)
{
	/* value will not be used */
	*payload = 0;
}

static void fill_input_payloads()
{
	fill_ipp_ping_input_payload(&input_payload[stmvl53l1_ipp_ping]);
	fill_ipp_cal_hist_input_payload(&input_payload[stmvl53l1_ipp_cal_hist]);
	fill_ipp_xtalk_calibration_input_payload(&input_payload[stmvl53l1_ipp_xtalk_calibration]);
	fill_ipp_hist_ambient_dmax_input_payload(&input_payload[stmvl53l1_ipp_hist_ambient_dmax]);
	fill_ipp_generate_dual_reflectance_xtalk_samples_input_payload(&input_payload[stmvl53l1_ipp_generate_dual_reflectance_xtalk_samples]);
}

static int init_msg_rcv()
{
	struct nlmsghdr *nlh;

	nlh = (struct nlmsghdr *) malloc(NLMSG_SPACE(IPP_WORK_MAX_PAYLOAD));
	if (!nlh) {
		nl_err("Fail to allocate memory for message reception buffer");
		return 1;
	}
	nlh_msg_rcv = nlh;

	memset(nlh, 0, NLMSG_SPACE(IPP_WORK_MAX_PAYLOAD));
	nlh->nlmsg_len = NLMSG_SPACE(IPP_WORK_MAX_PAYLOAD);
	nlh->nlmsg_pid = 0;
	nlh->nlmsg_flags = 0;

	memset(&iov_rcv, 0, sizeof(iov_rcv));
	iov_rcv.iov_base = nlh;
	iov_rcv.iov_len = nlh->nlmsg_len;

	memset(&msg_rcv, 0, sizeof(msg_rcv));
	msg_rcv.msg_iov = &iov_rcv;
	msg_rcv.msg_iovlen = 1;

	return 0;
}

static int init_msg_snd()
{
	struct nlmsghdr *nlh;

	nlh = (struct nlmsghdr *) malloc(NLMSG_SPACE(IPP_WORK_MAX_PAYLOAD));
	if (!nlh) {
		nl_err("Fail to allocate memory for message reception buffer");
		return 1;
	}

	nlh_msg_snd = nlh;

	memset(nlh, 0, NLMSG_SPACE(IPP_WORK_MAX_PAYLOAD));
	nlh->nlmsg_len = NLMSG_SPACE(IPP_WORK_MAX_PAYLOAD);
	nlh->nlmsg_pid = getpid();
	nlh->nlmsg_flags = 0;

	memset(&iov_snd, 0, sizeof(iov_snd));
	iov_snd.iov_base = nlh;
	iov_snd.iov_len = nlh->nlmsg_len;

	memset(&msg_snd, 0, sizeof(msg_snd));
	msg_snd.msg_iov = &iov_snd;
	msg_snd.msg_iovlen = 1;

	return 0;
}

static int send_ping()
{
	struct nlmsghdr *nlh = (struct nlmsghdr *) iov_snd.iov_base;
	struct ipp_work_t *p = NLMSG_DATA(nlh);
	int rc = 0;

	/* set payload */
	p->status = 0;
	p->payload = IPP_WORK_HDR_SIZE;
	p->dev_id = 0;
	p->process_no = stmvl53l1_ipp_ping;

	/* adjust message size to actual payload */
	iov_snd.iov_len = NLMSG_SPACE(p->payload);

	nl_dbg("ping driver");
	rc = sendmsg(nl_socket, &msg_snd, 0);
	if (rc > 0)
		rc = 0;

	return rc;
}

static void handle_message()
{
	struct nlmsghdr *nlh = (struct nlmsghdr *) iov_rcv.iov_base;
	struct ipp_work_t *p = NLMSG_DATA(nlh);

	nl_dbg("handle new message of type %d", p->process_no);
	switch(p->process_no) {
		case stmvl53l1_ipp_cal_hist:
			ipp_call_hist_dser(p->process_no);
			break;
		case stmvl53l1_ipp_xtalk_calibration:
			ipp_call_xtalk_dser(p->process_no);
			break;
		case stmvl53l1_ipp_hist_ambient_dmax:
			ipp_call_dmax_dser(p->process_no);
			break;
		case stmvl53l1_ipp_generate_dual_reflectance_xtalk_samples:
			ipp_call_dual_xtalk_dser(p->process_no);
			break;
		default:
			nl_err("Reveive an unknown process no %d", p->process_no);
			ipp_reply_error(stmvl53l1_ipp_status_inv_proc);
	}
}


int vl53l1_daemon_init()
{
	struct sockaddr_nl addr;
	int rc;
	int nl_id;

	fill_input_payloads();

	nl_id = get_driver_netlink_protocol_type();

	/* open socket */
	nl_socket = socket(PF_NETLINK, SOCK_RAW, nl_id);
	if (nl_socket < 0) {
		nl_err("fail to open netlink socket to %d",nl_id);
		return -1;
	}
	nl_inf("socket open to id %d", nl_id);

	/* bind it */
	memset(&addr, 0, sizeof(addr));
	addr.nl_family = AF_NETLINK;
	addr.nl_pid = getpid();
	rc = bind(nl_socket, (struct sockaddr*)&addr, sizeof(addr));
	if(rc) {
		nl_err("fail to bind socket to id %d",nl_id);
		close(nl_socket);
		return rc;
	}

	rc = init_msg_rcv();
	if (rc)
		return rc;
	rc = init_msg_snd();
	if (rc)
		return rc;

	return 0;
}

int vl53l1_daemon_deinit()
{
	/* Close the socket */
	if (nl_socket > 0) {
		nl_inf("Close Socket");
		close(nl_socket);
	}

	if (nlh_msg_rcv != NULL) {
		free(nlh_msg_rcv);
		nl_inf("free nlh_msg_rcv");
	}

	if (nlh_msg_snd != NULL) {
		free(nlh_msg_snd);
		nl_inf("free nlh_msg_snd");
	}

	return 0;
}

void vl53l1_daemon_run()
{
	/* register with the driver */
	if (send_ping()) {
		nl_err("send_ping failed");
		return ;
	}

	/* start to receive and process messages */
	while (1) {
		int rc;

		rc = recvmsg(nl_socket, &msg_rcv, 0);
		if(rc < 0) {
			nl_err("message receive failed with %d", rc);
			continue;
		}
		handle_message();
	}
}
