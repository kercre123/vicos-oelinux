/* Copyright (c) 2013-2014 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */


#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "mcm_common_v01.h"
#include "mcm_service_object_v01.h"
#include "mcm_loc_v01.h"
#include "location_test_interface_qmi_loc.h"
#include "loc_srv_hal_to_mcm_type_conversions.h"
#include "loc_srv_mcm_to_hal_type_conversions.h"
//#include "garden_utils.h"
#include "location_callbacks_qmi_loc.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "qmi_client.h"
#ifdef __cplusplus
}
#endif

#include "qmi_idl_lib.h"

static qmi_client_type client_handle;
static garden_ext_cb_t *garden_cb_ptr;

#define MCM_IND_MSG_ID_BASE 0x30e
#define LOC_QMI_MSG_TIMEOUT 250

void test_qmi_loc_location_cb(qmi_client_type hndl, unsigned int msg_id, void *ind_buf, unsigned int ind_len, void *ind_cb_data){

    qmi_client_error_type qmi_err;
    void *ind_c_struct;
    int   ind_c_struct_len = 0;
    mcm_loc_location_info_ind_msg_v01 *ind;
    LocGpsLocation l;
    ind_c_struct_len = sizeof(mcm_loc_location_info_ind_msg_v01);
    ind_c_struct = (void*)malloc(ind_c_struct_len);
    qmi_err =
    qmi_client_message_decode(hndl, QMI_IDL_INDICATION, msg_id, ind_buf, ind_len, ind_c_struct, ind_c_struct_len);
    if(ind_c_struct) {
        ind = (mcm_loc_location_info_ind_msg_v01*)ind_c_struct;
        loc_srv_conv_mcm_gps_location(&(ind->location), &l);

        if(garden_cb_ptr != NULL && garden_cb_ptr->garden_location_cb) {
            garden_cb_ptr->garden_location_cb(&l);
        }
    }
    if(ind_c_struct) {
        free(ind_c_struct);
    }
}
void test_qmi_loc_status_cb(qmi_client_type hndl, unsigned int msg_id, void *ind_buf, unsigned int ind_len, void *ind_cb_data){

    qmi_client_error_type qmi_err;
    void *ind_c_struct;
    int   ind_c_struct_len = 0;
    mcm_loc_status_info_ind_msg_v01 *ind;
    LocGpsStatus s;
    ind_c_struct_len = sizeof(mcm_loc_status_info_ind_msg_v01);
    ind_c_struct = (void*)malloc(ind_c_struct_len);
    qmi_err =
    qmi_client_message_decode(hndl, QMI_IDL_INDICATION, msg_id, ind_buf, ind_len, ind_c_struct, ind_c_struct_len);
    if(ind_c_struct) {
        ind = (mcm_loc_status_info_ind_msg_v01*)ind_c_struct;
        loc_srv_conv_mcm_gps_status(&(ind->status), &(s));
        if(garden_cb_ptr != NULL && garden_cb_ptr->garden_status_cb) {
            garden_cb_ptr->garden_status_cb(&s);
        }
    }
    if(ind_c_struct) {
        free(ind_c_struct);
    }
}

void test_qmi_loc_sv_cb(qmi_client_type hndl, unsigned int msg_id, void *ind_buf, unsigned int ind_len, void *ind_cb_data){

    qmi_client_error_type qmi_err;
    void *ind_c_struct;
    int   ind_c_struct_len = 0;
    mcm_loc_sv_info_ind_msg_v01 *ind;
    LocGpsSvStatus s;
    ind_c_struct_len = sizeof(mcm_loc_sv_info_ind_msg_v01);
    ind_c_struct = (void*)malloc(ind_c_struct_len);
    qmi_err =
    qmi_client_message_decode(hndl, QMI_IDL_INDICATION, msg_id, ind_buf, ind_len, ind_c_struct, ind_c_struct_len);
    if(ind_c_struct) {
        ind = (mcm_loc_sv_info_ind_msg_v01*)ind_c_struct;
        loc_srv_conv_mcm_gps_sv_status(&(ind->sv_info), &(s));
        if(garden_cb_ptr != NULL && garden_cb_ptr->garden_sv_status_cb) {
            garden_cb_ptr->garden_sv_status_cb(&s);
        }
    }
    if(ind_c_struct) {
        free(ind_c_struct);
    }
}

void test_qmi_loc_nmea_cb(qmi_client_type hndl, unsigned int msg_id, void *ind_buf, unsigned int ind_len, void *ind_cb_data){

    qmi_client_error_type qmi_err;
    void *ind_c_struct;
    int   ind_c_struct_len = 0;
    mcm_loc_nmea_info_ind_msg_v01 *ind;
    int64_t timestamp;
    const char *nmea;
    int length;
    ind_c_struct_len = sizeof(mcm_loc_nmea_info_ind_msg_v01);
    ind_c_struct = (void*)malloc(ind_c_struct_len);
    qmi_err =
    qmi_client_message_decode(hndl, QMI_IDL_INDICATION, msg_id, ind_buf, ind_len, ind_c_struct, ind_c_struct_len);
    if(ind_c_struct) {
        ind = (mcm_loc_nmea_info_ind_msg_v01*)ind_c_struct;
        timestamp = ind->timestamp;
        nmea = ind->nmea;
        length = ind->length;
        if(garden_cb_ptr != NULL && garden_cb_ptr->garden_nmea_cb) {
            garden_cb_ptr->garden_nmea_cb(timestamp, nmea, length);
        }
    }
    if(ind_c_struct) {
        free(ind_c_struct);
    }
}

void test_qmi_loc_cap_cb(qmi_client_type hndl, unsigned int msg_id, void *ind_buf, unsigned int ind_len, void *ind_cb_data){

    qmi_client_error_type qmi_err;
    void *ind_c_struct;
    int   ind_c_struct_len = 0;
    mcm_loc_capabilities_info_ind_msg_v01 *ind;
    uint32_t c;
    ind_c_struct_len = sizeof(mcm_loc_capabilities_info_ind_msg_v01);
    ind_c_struct = (void*)malloc(ind_c_struct_len);
    qmi_err =
    qmi_client_message_decode(hndl, QMI_IDL_INDICATION, msg_id, ind_buf, ind_len, ind_c_struct, ind_c_struct_len);
    if(ind_c_struct) {
        ind = (mcm_loc_capabilities_info_ind_msg_v01*)ind_c_struct;
        loc_srv_conv_mcm_gps_capabilities(&(ind->capabilities), &(c));
        if(garden_cb_ptr != NULL && garden_cb_ptr->garden_set_cap_cb) {
            garden_cb_ptr->garden_set_cap_cb(c);
        }
    }
    if(ind_c_struct) {
        free(ind_c_struct);
    }
}

void test_qmi_loc_utc_time_cb(qmi_client_type hndl, unsigned int msg_id, void *ind_buf, unsigned int ind_len, void *ind_cb_data){

    if(garden_cb_ptr != NULL && garden_cb_ptr->garden_request_utc_time_cb) {
        garden_cb_ptr->garden_request_utc_time_cb();
    }
}

void test_qmi_loc_xtra_data_cb(qmi_client_type hndl, unsigned int msg_id, void *ind_buf, unsigned int ind_len, void *ind_cb_data) {

    if(garden_cb_ptr != NULL && garden_cb_ptr->garden_xtra_download_request_cb) {
        garden_cb_ptr->garden_xtra_download_request_cb();
    }
}

void test_qmi_loc_xtra_report_server_cb(qmi_client_type hndl, unsigned int msg_id, void *ind_buf, unsigned int ind_len, void *ind_cb_data) {

    qmi_client_error_type qmi_err;
    void *ind_c_struct;
    int   ind_c_struct_len = 0;
    mcm_loc_xtra_report_server_ind_msg_v01 *ind;
    ind_c_struct_len = sizeof(mcm_loc_xtra_report_server_ind_msg_v01);
    ind_c_struct = malloc(ind_c_struct_len);
    qmi_err =
    qmi_client_message_decode(hndl, QMI_IDL_INDICATION, msg_id, ind_buf, ind_len, ind_c_struct, ind_c_struct_len);
    if(ind_c_struct) {
        ind = (mcm_loc_xtra_report_server_ind_msg_v01*)ind_c_struct;
    }
    if(ind_c_struct) {
        free(ind_c_struct);
    }
}

void test_qmi_loc_agps_status_cb(qmi_client_type hndl, unsigned int msg_id, void *ind_buf, unsigned int ind_len, void *ind_cb_data) {

    qmi_client_error_type qmi_err;
    void *ind_c_struct;
    int   ind_c_struct_len = 0;
    mcm_loc_agps_status_ind_msg_v01 *ind;
    AGpsExtStatus s;
    ind_c_struct_len = sizeof(mcm_loc_agps_status_ind_msg_v01);
    ind_c_struct = (void*)malloc(ind_c_struct_len);
    qmi_err =
    qmi_client_message_decode(hndl, QMI_IDL_INDICATION, msg_id, ind_buf, ind_len, ind_c_struct, ind_c_struct_len);
    if(ind_c_struct) {
        ind = (mcm_loc_agps_status_ind_msg_v01*)ind_c_struct;
        loc_srv_conv_mcm_agps_status(&(ind->status),&(s));
        if(garden_cb_ptr != NULL && garden_cb_ptr->garden_agps_ext_status_cb) {
            garden_cb_ptr->garden_agps_ext_status_cb(&s);
        }
    }
    if(ind_c_struct) {
        free(ind_c_struct);
    }
}

void test_qmi_loc_ni_notify_cb(qmi_client_type hndl, unsigned int msg_id, void *ind_buf, unsigned int ind_len, void *ind_cb_data){

    qmi_client_error_type qmi_err;
    void *ind_c_struct;
    int   ind_c_struct_len = 0;
    mcm_loc_ni_notification_ind_msg_v01 *ind;
    LocGpsNiNotification n;
    ind_c_struct_len = sizeof(mcm_loc_ni_notification_ind_msg_v01);
    ind_c_struct = (void*)malloc(ind_c_struct_len);
    qmi_err =
    qmi_client_message_decode(hndl, QMI_IDL_INDICATION, msg_id, ind_buf, ind_len, ind_c_struct, ind_c_struct_len);
    if(ind_c_struct) {
        ind = (mcm_loc_ni_notification_ind_msg_v01*)ind_c_struct;
        loc_srv_conv_mcm_gps_ni_notification(&(ind->notification), &(n));
        if(garden_cb_ptr != NULL && garden_cb_ptr->garden_ni_notify_cb) {
            garden_cb_ptr->garden_ni_notify_cb(&n);
        }
    }
    if(ind_c_struct) {
        free(ind_c_struct);
    }
}


typedef void (*test_qmi_loc_cb_func_t) (qmi_client_type hndl,
                                    unsigned int msg_id,
                                    void *ind_buf,
                                    unsigned int ind_len,
                                    void *ind_cb_data);

test_qmi_loc_cb_func_t qmi_loc_cb_func_table[] = {
    &test_qmi_loc_location_cb,
    &test_qmi_loc_status_cb,
    &test_qmi_loc_sv_cb,
    &test_qmi_loc_nmea_cb,
    &test_qmi_loc_cap_cb,
    &test_qmi_loc_utc_time_cb,
    &test_qmi_loc_xtra_data_cb,
    &test_qmi_loc_agps_status_cb,
    &test_qmi_loc_ni_notify_cb,
    &test_qmi_loc_xtra_report_server_cb
};

void test_qmi_loc_ind_cb (
    qmi_client_type hndl,
    unsigned int msg_id,
    void *ind_buf,
    unsigned int ind_len,
    void *ind_cb_data) {

    int func_table_id = 0;
    if(!ind_buf) {
        //garden_print(" Indication with Null indication buffer. Msg id = %d", msg_id);
    }
    //garden_print(" Received Indication for msg id # %d",msg_id);
    func_table_id = msg_id - MCM_IND_MSG_ID_BASE;
    if( (func_table_id >= 0) &&
        (func_table_id < (int)(sizeof(qmi_loc_cb_func_table)/sizeof(*qmi_loc_cb_func_table))) ) {

         qmi_loc_cb_func_table[func_table_id] (hndl, msg_id, ind_buf, ind_len, ind_cb_data);
    }
    else {
        //garden_print(" Error: Func table id is %d",func_table_id);
    }
}


//helpers
static int start_mcm_client() {
    int ret_val = 0;
    uint32_t num_services, num_entries = 0;
    qmi_client_type clnt, notifier;
    qmi_cci_os_signal_type os_params;
    qmi_service_info info[10];

    qmi_idl_service_object_type mcm_loc_service_obj =
        mcm_loc_get_service_object_v01( );

    ret_val =
        qmi_client_notifier_init(mcm_loc_service_obj, &os_params, &notifier);
    // Check if the service is up, if not wait on a signal
    while(1) {
        ret_val = qmi_client_get_service_list(mcm_loc_service_obj,
                                              NULL,
                                              NULL,
                                              &num_services);
        //garden_print("qmi_client_get_service_list() returned %d num_services = %d",ret_val, num_services);
        if(ret_val == QMI_NO_ERR) {
            break;
        }
        //garden_print(" Waiting for mcmlocserver to come up....");
        // Wait for server to come up
        QMI_CCI_OS_SIGNAL_WAIT(&os_params,0);
        QMI_CCI_OS_SIGNAL_CLEAR(&os_params);
    }
    num_entries = num_services;
    // The server has come up, store the information in info variable
    ret_val = qmi_client_get_service_list(mcm_loc_service_obj,
                                          info,
                                          &num_entries,
                                          &num_services);
    //garden_print("qmi_client_get_service_list() returned %d num_entries = %d num_services = %d ",ret_val, num_entries, num_services);
    if(num_services < 1) {
        //garden_print(" num_services is less than 1. exiting ...");
        return -1;
    }
    ret_val = qmi_client_init(&info[0], mcm_loc_service_obj, test_qmi_loc_ind_cb, NULL, NULL, &client_handle);
    return ret_val;
}

extern "C" location_test_interface_base * create_test_interface() {
    return new location_test_interface_qmi_loc;
}

extern "C" void destroy_test_interface(location_test_interface_base *p) {
    delete p;
}

// ioe mcm test interface

location_test_interface_qmi_loc::location_test_interface_qmi_loc() {
}

location_test_interface_qmi_loc::~location_test_interface_qmi_loc() {
}

int location_test_interface_qmi_loc::location_init(garden_ext_cb_t *pcb) {

    mcm_loc_set_indications_req_msg_v01 req;
    mcm_loc_set_indications_resp_msg_v01 resp;

    memset(&req, 0, sizeof(mcm_loc_set_indications_req_msg_v01));
    memset(&resp, 0, sizeof(mcm_loc_set_indications_resp_msg_v01));

    if(pcb != NULL) {
        garden_cb_ptr = pcb;
    }
    uint32_t ret_val = start_mcm_client();

    // Set Indications
    req.register_location_info_ind = 1;
    req.register_status_info_ind = 1;
    req.register_sv_info_ind = 1;
    req.register_nmea_info_ind = 1;
    req.register_capabilities_info_ind = 1;
    req.register_utc_time_req_ind = 1;
    req.register_xtra_data_req_ind = 1;
    req.register_agps_data_conn_cmd_req_ind = 1;
    req.register_ni_notify_user_response_req_ind = 1;
    req.register_xtra_report_server_ind_valid = 1;
    req.register_xtra_report_server_ind = 1;

    ret_val = qmi_client_send_msg_sync(client_handle,
                                           MCM_LOC_SET_INDICATIONS_REQ_V01,
                                           &req, sizeof(req),
                                           &resp, sizeof(resp), LOC_QMI_MSG_TIMEOUT);
    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_qmi_loc::location_set_position_mode(uint32_t mode,
                                                                uint32_t recurrence,
                                                                uint32_t min_interval,
                                                                uint32_t preferred_accuracy,
                                                                uint32_t preferred_time) {

    mcm_loc_set_position_mode_req_msg_v01 req;
    mcm_loc_set_position_mode_resp_msg_v01 resp;
    uint32_t ret_val;

    memset(&req, 0, sizeof(req));
    memset(&resp, 0, sizeof(resp));
    loc_srv_conv_gps_position_mode(&mode, &(req.mode));
    loc_srv_conv_gps_position_recurrence(&recurrence, &(req.recurrence));
    req.min_interval = min_interval;
    req.preferred_accuracy = preferred_accuracy;
    req.preferred_time = preferred_time;

    ret_val = qmi_client_send_msg_sync(client_handle,
                                          MCM_LOC_SET_POSITION_MODE_REQ_V01,
                                          &req, sizeof(req),
                                          &resp, sizeof(resp), LOC_QMI_MSG_TIMEOUT);
    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_qmi_loc::location_start_navigation() {

    mcm_loc_start_nav_req_msg_v01 req;
    mcm_loc_start_nav_resp_msg_v01 resp;
    uint32_t ret_val;

    memset(&req, 0, sizeof(req));
    memset(&resp, 0, sizeof(resp));
    ret_val = qmi_client_send_msg_sync(client_handle,
                                         MCM_LOC_START_NAV_REQ_V01,
                                         &req, sizeof(req),
                                         &resp, sizeof(resp), LOC_QMI_MSG_TIMEOUT);

    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_qmi_loc::location_stop_navigation() {

    mcm_loc_stop_nav_req_msg_v01 req;
    mcm_loc_stop_nav_resp_msg_v01 resp;
    uint32_t ret_val;

    memset(&req, 0, sizeof(req));
    memset(&resp, 0, sizeof(resp));
    ret_val = qmi_client_send_msg_sync(client_handle,
                                         MCM_LOC_STOP_NAV_REQ_V01,
                                         &req, sizeof(req),
                                         &resp, sizeof(resp), LOC_QMI_MSG_TIMEOUT);

    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_qmi_loc::location_close() {

    mcm_loc_set_indications_req_msg_v01 req;
    mcm_loc_set_indications_resp_msg_v01 resp;
    uint32_t ret_val;

    memset(&req, 0, sizeof(req));
    memset(&resp, 0, sizeof(resp));
    // Set Indications
    req.register_location_info_ind = 0;
    req.register_status_info_ind = 0;
    req.register_sv_info_ind = 0;
    req.register_nmea_info_ind = 0;
    req.register_capabilities_info_ind = 0;
    req.register_utc_time_req_ind = 0;
    req.register_xtra_data_req_ind = 0;
    req.register_agps_data_conn_cmd_req_ind = 0;
    req.register_ni_notify_user_response_req_ind = 0;

    ret_val = qmi_client_send_msg_sync(client_handle,
                                           MCM_LOC_SET_INDICATIONS_REQ_V01,
                                           &req, sizeof(req),
                                           &resp, sizeof(resp), LOC_QMI_MSG_TIMEOUT);
    // Disconnect from server
    qmi_client_release(client_handle);
    garden_cb_ptr = NULL;
    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_qmi_loc::location_delete_aiding_data(uint16_t flags) {

    mcm_loc_delete_aiding_data_req_msg_v01 req;
    mcm_loc_delete_aiding_data_resp_msg_v01 resp;
    uint32_t ret_val;

    memset(&req, 0, sizeof(req));
    memset(&resp, 0, sizeof(resp));
    loc_srv_conv_gps_aiding_data(&flags, &(req.flags));

    ret_val = qmi_client_send_msg_sync(client_handle,
                                           MCM_LOC_DELETE_AIDING_DATA_REQ_V01,
                                           &req, sizeof(req),
                                           &resp, sizeof(resp), LOC_QMI_MSG_TIMEOUT);

    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_qmi_loc::location_inject_time(int64_t time,
                                                          int64_t timeReference,
                                                          int uncertainty) {
    mcm_loc_inject_time_req_msg_v01 req;
    mcm_loc_inject_time_resp_msg_v01 resp;
    uint32_t ret_val;

    memset(&req, 0, sizeof(req));
    memset(&resp, 0, sizeof(resp));
    req.time = time;
    req.time_reference = timeReference;
    req.uncertainty = uncertainty;

    ret_val = qmi_client_send_msg_sync(client_handle,
                                           MCM_LOC_INJECT_TIME_REQ_V01,
                                           &req, sizeof(req),
                                           &resp, sizeof(resp), LOC_QMI_MSG_TIMEOUT);

    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_qmi_loc::location_inject_location(double latitude,
                                                              double longitude,
                                                              float accuracy) {

    mcm_loc_inject_location_req_msg_v01 req;
    mcm_loc_inject_location_resp_msg_v01 resp;
    uint32_t ret_val;

    memset(&req, 0, sizeof(req));
    memset(&resp, 0, sizeof(resp));
    req.latitude = latitude;
    req.longitude = longitude;
    req.accuracy = accuracy;

    ret_val = qmi_client_send_msg_sync(client_handle,
                                           MCM_LOC_INJECT_LOCATION_REQ_V01,
                                           &req, sizeof(req),
                                           &resp, sizeof(resp), LOC_QMI_MSG_TIMEOUT);

    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_qmi_loc::location_xtra_inject_data(char *data,
                                                               int length) {

    mcm_loc_xtra_inject_data_req_msg_v01 req;
    mcm_loc_xtra_inject_data_resp_msg_v01 resp;
    uint32_t ret_val;
    int len = 0;

    memset(&req, 0, sizeof(req));
    memset(&resp, 0, sizeof(resp));
    len = (length > (MCM_LOC_MAX_XTRA_DATA_LENGTH_CONST_V01)) ?
          (MCM_LOC_MAX_XTRA_DATA_LENGTH_CONST_V01) :
          (length);

    memcpy((void*)req.data, (const void*)data, len);

    req.data_len = len;

    ret_val = qmi_client_send_msg_sync(client_handle,
                                           MCM_LOC_XTRA_INJECT_DATA_REQ_V01,
                                           &req, sizeof(req),
                                           &resp, sizeof(resp), LOC_QMI_MSG_TIMEOUT);
    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_qmi_loc::location_agps_data_conn_open(
                                                       int16_t atype,
                                                       const char *apn,
                                                       int16_t btype) {

    mcm_loc_agps_data_conn_open_req_msg_v01 req;
    mcm_loc_agps_data_conn_open_resp_msg_v01 resp;
    uint32_t ret_val;

    memset(&req, 0, sizeof(req));
    memset(&resp, 0, sizeof(resp));
    loc_srv_conv_agps_type(&atype, &(req.agps_type));
    memcpy((void*)req.apn, (const void*)apn,
           MCM_LOC_MAX_APN_NAME_LENGTH_CONST_V01 + 1);
    loc_srv_conv_agps_bearer(&btype,
                            &(req.bearer_type));
    ret_val = qmi_client_send_msg_sync(client_handle,
                                           MCM_LOC_AGPS_DATA_CONN_OPEN_REQ_V01,
                                           &req, sizeof(req),
                                           &resp, sizeof(resp), LOC_QMI_MSG_TIMEOUT);
    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_qmi_loc::location_agps_data_conn_closed(int16_t atype) {

    mcm_loc_agps_data_conn_closed_req_msg_v01 req;
    mcm_loc_agps_data_conn_closed_resp_msg_v01 resp;
    uint32_t ret_val;
    memset(&req, 0, sizeof(req));
    memset(&resp, 0, sizeof(resp));
    loc_srv_conv_agps_type(&atype, &(req.agps_type));

    ret_val = qmi_client_send_msg_sync(client_handle,
                                          MCM_LOC_AGPS_DATA_CONN_CLOSED_REQ_V01,
                                          &req, sizeof(req),
                                          &resp, sizeof(resp), LOC_QMI_MSG_TIMEOUT);
    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_qmi_loc::location_agps_data_conn_failed(int16_t atype) {

    mcm_loc_agps_data_conn_failed_req_msg_v01 req;
    mcm_loc_agps_data_conn_failed_resp_msg_v01 resp;
    uint32_t ret_val;
    memset(&req, 0, sizeof(req));
    memset(&resp, 0, sizeof(resp));
    loc_srv_conv_agps_type(&atype, &(req.agps_type));

    ret_val = qmi_client_send_msg_sync(client_handle,
                                          MCM_LOC_AGPS_DATA_CONN_FAILED_REQ_V01,
                                          &req, sizeof(req),
                                          &resp, sizeof(resp), LOC_QMI_MSG_TIMEOUT);
    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_qmi_loc::location_agps_set_server(
                                                     int16_t atype,
                                                     const char * hostname,
                                                     int port) {

    mcm_loc_agps_set_server_req_msg_v01 req;
    mcm_loc_agps_set_server_resp_msg_v01 resp;
    uint32_t ret_val;
    memset(&req, 0, sizeof(req));
    memset(&resp, 0, sizeof(resp));
    loc_srv_conv_agps_type(&atype, &(req.agps_type));

    memcpy((void*)req.host_name, (const void*)hostname,
           MCM_LOC_MAX_SEVER_ADDR_LENGTH_CONST_V01 + 1);
    req.port = port;

    ret_val = qmi_client_send_msg_sync(client_handle,
                                          MCM_LOC_AGPS_SET_SERVER_REQ_V01,
                                          &req, sizeof(req),
                                          &resp, sizeof(resp), LOC_QMI_MSG_TIMEOUT);
    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_qmi_loc::location_ni_respond(int notif_id,
                                                         int user_response) {

    mcm_loc_ni_respond_req_msg_v01 req;
    mcm_loc_ni_respond_resp_msg_v01 resp;
    uint32_t ret_val;

    memset(&req, 0, sizeof(req));
    memset(&resp, 0, sizeof(resp));
    req.notif_id = notif_id;
    loc_srv_conv_gps_user_response(&user_response,
                                  &(req.user_response));
    ret_val = qmi_client_send_msg_sync(client_handle,
                                       MCM_LOC_NI_RESPOND_REQ_V01,
                                       &req, sizeof(req),
                                       &resp, sizeof(resp), LOC_QMI_MSG_TIMEOUT);

    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_qmi_loc::location_agps_ril_update_network_availability(
                                                    int available,
                                                    const char * apn) {

    mcm_loc_agps_ril_update_network_availability_req_msg_v01 req;
    mcm_loc_agps_ril_update_network_availability_resp_msg_v01 resp;
    uint32_t ret_val;
    memset(&req, 0, sizeof(req));
    memset(&resp, 0, sizeof(resp));
    memcpy((void*)req.apn, (const void*)apn,
            MCM_LOC_MAX_APN_NAME_LENGTH_CONST_V01 + 1);
    req.available = available;

    ret_val = qmi_client_send_msg_sync(client_handle,
                           MCM_LOC_AGPS_RIL_UPDATE_NETWORK_AVAILABILITY_REQ_V01,
                           &req, sizeof(req),
                           &resp, sizeof(resp), LOC_QMI_MSG_TIMEOUT);
    return (resp.resp.result) ? (-1) : (0);
}

