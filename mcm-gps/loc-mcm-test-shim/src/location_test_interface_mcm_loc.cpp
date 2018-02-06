/* Copyright (c) 2013-2014 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */


#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "mcm_client.h"
#include "mcm_common_v01.h"
#include "mcm_service_object_v01.h"
#include "mcm_loc_v01.h"
#include "location_test_interface_mcm_loc.h"
#include "loc_srv_hal_to_mcm_type_conversions.h"
#include "loc_srv_mcm_to_hal_type_conversions.h"
//#include "garden_utils.h"
#include "location_callbacks_mcm_loc.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "qmi_client.h"
#ifdef __cplusplus
}
#endif

#include "qmi_idl_lib.h"
#include "mcm_service_object_v01.h"

static mcm_client_handle_type client_handle;
static garden_ext_cb_t *garden_cb_ptr;

#define MCM_IND_MSG_ID_BASE 0x30e

void test_mcm_loc_location_cb(mcm_client_handle_type hndl, uint32_t msg_id, void *ind_c_struct, uint32_t ind_len){

    mcm_loc_location_info_ind_msg_v01 *ind = (mcm_loc_location_info_ind_msg_v01*)ind_c_struct;
    LocGpsLocation l;
    if(ind) {
        loc_srv_conv_mcm_gps_location(&(ind->location), &l);
        if(garden_cb_ptr != NULL && garden_cb_ptr->garden_location_cb) {
            garden_cb_ptr->garden_location_cb(&l);
        }
    }
}

void test_mcm_loc_status_cb(mcm_client_handle_type hndl, uint32_t msg_id, void *ind_c_struct, uint32_t ind_len){

    mcm_loc_status_info_ind_msg_v01 *ind = (mcm_loc_status_info_ind_msg_v01*)ind_c_struct;
    LocGpsStatus s;
    if(ind) {
        loc_srv_conv_mcm_gps_status(&(ind->status), &(s));
        if(garden_cb_ptr != NULL && garden_cb_ptr->garden_status_cb) {
            garden_cb_ptr->garden_status_cb(&s);
        }
    }
}

void test_mcm_loc_sv_cb(mcm_client_handle_type hndl, uint32_t msg_id, void *ind_c_struct, uint32_t ind_len){

    mcm_loc_sv_info_ind_msg_v01 *ind = (mcm_loc_sv_info_ind_msg_v01*)ind_c_struct;
    LocGpsSvStatus s;
    if(ind) {
        loc_srv_conv_mcm_gps_sv_status(&(ind->sv_info), &(s));
        if(garden_cb_ptr != NULL && garden_cb_ptr->garden_sv_status_cb) {
            garden_cb_ptr->garden_sv_status_cb(&s);
        }
    }
}

void test_mcm_loc_nmea_cb(mcm_client_handle_type hndl, uint32_t msg_id, void *ind_c_struct, uint32_t ind_len){

    mcm_loc_nmea_info_ind_msg_v01 *ind = (mcm_loc_nmea_info_ind_msg_v01*)ind_c_struct;
    if(ind) {
        if(garden_cb_ptr != NULL && garden_cb_ptr->garden_nmea_cb) {
            garden_cb_ptr->garden_nmea_cb(ind->timestamp, ind->nmea, ind->length);
        }
    }
}

void test_mcm_loc_cap_cb(mcm_client_handle_type hndl, uint32_t msg_id, void *ind_c_struct, uint32_t ind_len){

    mcm_loc_capabilities_info_ind_msg_v01 *ind = (mcm_loc_capabilities_info_ind_msg_v01*)ind_c_struct;
    uint32_t c;
    if(ind) {
        loc_srv_conv_mcm_gps_capabilities(&(ind->capabilities), &(c));
        if(garden_cb_ptr != NULL && garden_cb_ptr->garden_set_cap_cb) {
            garden_cb_ptr->garden_set_cap_cb(c);
        }
    }
}

void test_mcm_loc_utc_time_cb(mcm_client_handle_type hndl, uint32_t msg_id, void *ind_c_struct, uint32_t ind_len){
    if(garden_cb_ptr != NULL && garden_cb_ptr->garden_request_utc_time_cb) {
        garden_cb_ptr->garden_request_utc_time_cb();
    }
}

void test_mcm_loc_xtra_data_cb(mcm_client_handle_type hndl, uint32_t msg_id, void *ind_c_struct, uint32_t ind_len) {

    if(garden_cb_ptr != NULL && garden_cb_ptr->garden_xtra_download_request_cb) {
        garden_cb_ptr->garden_xtra_download_request_cb();
    }
}

void test_mcm_loc_xtra_report_server_cb(mcm_client_handle_type hndl, uint32_t msg_id, void *ind_c_struct, uint32_t ind_len) {


    mcm_loc_xtra_report_server_ind_msg_v01 *ind = (mcm_loc_xtra_report_server_ind_msg_v01*)ind_c_struct;
}

void test_mcm_loc_agps_status_cb(mcm_client_handle_type hndl, uint32_t msg_id, void *ind_c_struct, uint32_t ind_len) {

    mcm_loc_agps_status_ind_msg_v01 *ind = (mcm_loc_agps_status_ind_msg_v01*)ind_c_struct;
    AGpsExtStatus s;
    if(ind) {
        loc_srv_conv_mcm_agps_status(&(ind->status),&(s));
        if(garden_cb_ptr != NULL && garden_cb_ptr->garden_agps_ext_status_cb) {
            garden_cb_ptr->garden_agps_ext_status_cb(&s);
        }
    }
}

void test_mcm_loc_ni_notify_cb(mcm_client_handle_type hndl, uint32_t msg_id, void *ind_c_struct, uint32_t ind_len){

    mcm_loc_ni_notification_ind_msg_v01 *ind = (mcm_loc_ni_notification_ind_msg_v01*)ind_c_struct;
    LocGpsNiNotification n;
    if(ind) {
        loc_srv_conv_mcm_gps_ni_notification(&(ind->notification), &(n));
        if(garden_cb_ptr != NULL && garden_cb_ptr->garden_ni_notify_cb) {
            garden_cb_ptr->garden_ni_notify_cb(&n);
        }
    }
}


typedef void (*test_mcm_loc_cb_func_t) (mcm_client_handle_type hndl,
                                    uint32_t msg_id,
                                    void *ind_c_struct,
                                    uint32_t ind_len);

test_mcm_loc_cb_func_t mcm_loc_cb_func_table[] = {
    &test_mcm_loc_location_cb,
    &test_mcm_loc_status_cb,
    &test_mcm_loc_sv_cb,
    &test_mcm_loc_nmea_cb,
    &test_mcm_loc_cap_cb,
    &test_mcm_loc_utc_time_cb,
    &test_mcm_loc_xtra_data_cb,
    &test_mcm_loc_agps_status_cb,
    &test_mcm_loc_ni_notify_cb,
    &test_mcm_loc_xtra_report_server_cb
};

void test_mcm_loc_ind_cb (
    mcm_client_handle_type hndl,
    uint32_t msg_id,
    void *ind_c_struct,
    uint32_t ind_len) {

    int func_table_id = 0;
    if(!ind_c_struct) {
        //garden_print(" Indication with Null indication C struct . Msg id = %d", msg_id);
    }
    //garden_print(" Received Indication for msg id # %d",msg_id);
    func_table_id = msg_id - MCM_IND_MSG_ID_BASE;
    if( (func_table_id >= 0) &&
        (func_table_id < (int)(sizeof(mcm_loc_cb_func_table)/sizeof(*mcm_loc_cb_func_table))) ) {

         mcm_loc_cb_func_table[func_table_id] (hndl, msg_id, ind_c_struct, ind_len);
    }
    else {
        //garden_print(" Error: Func table id is %d",func_table_id);
    }
}


//helpers
static int start_mcm_client() {

    return mcm_client_init( &client_handle, test_mcm_loc_ind_cb, NULL);

}

extern "C" location_test_interface_base * create_test_interface() {

    return new location_test_interface_mcm_loc;
}

extern "C" void destroy_test_interface(location_test_interface_base *p) {
    delete p;
}

// ioe mcm test interface

location_test_interface_mcm_loc::location_test_interface_mcm_loc() {
}

location_test_interface_mcm_loc::~location_test_interface_mcm_loc() {
}

int location_test_interface_mcm_loc::location_init(garden_ext_cb_t *pcb) {

    mcm_loc_set_indications_req_msg_v01 req;
    mcm_loc_set_indications_resp_msg_v01 resp;

    memset(&req, 0, sizeof(mcm_loc_set_indications_req_msg_v01));
    memset(&resp, 0, sizeof(mcm_loc_set_indications_resp_msg_v01));

    if(pcb != NULL) {
        garden_cb_ptr = pcb;
    }
    uint32_t ret_val = start_mcm_client();
    if(MCM_SUCCESS_V01 != ret_val) {
        //garden_print("Failed to initialize mcm client: error - %d",ret_val);
    }

    // Define messages for dynamic loading of mcmlocserver
    mcm_client_require_req_msg_v01    require_req_msg;
    mcm_client_require_resp_msg_v01   require_resp_msg;

    memset(&require_req_msg, 0, sizeof(mcm_client_require_req_msg_v01));
    memset(&require_resp_msg, 0, sizeof(mcm_client_require_resp_msg_v01));

    // For Dynamic loading of mcmlocserver,indicate to mcm framework that we
    // would need mcmlocserver to be up and running before proceeding further
    require_req_msg.require_service = MCM_LOC_V01;

    ret_val = MCM_CLIENT_EXECUTE_COMMAND_SYNC(client_handle,
                                              MCM_CLIENT_REQUIRE_REQ_V01,
                                              &require_req_msg,
                                              &require_resp_msg);
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

    ret_val = mcm_client_execute_command_sync(client_handle,
                                           MCM_LOC_SET_INDICATIONS_REQ_V01,
                                           &req, sizeof(req),
                                           &resp, sizeof(resp));
    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_mcm_loc::location_set_position_mode(uint32_t mode,
                                           uint32_t recurrence,
                                           uint32_t min_interval,
                                           uint32_t preferred_accuracy,
                                           uint32_t preferred_time) {

    mcm_loc_set_position_mode_req_msg_v01 req;
    mcm_loc_set_position_mode_resp_msg_v01 resp;
    uint32_t ret_val;

    loc_srv_conv_gps_position_mode(&mode, &(req.mode));
    loc_srv_conv_gps_position_recurrence(&recurrence, &(req.recurrence));
    req.min_interval = min_interval;
    req.preferred_accuracy = preferred_accuracy;
    req.preferred_time = preferred_time;

    ret_val = mcm_client_execute_command_sync(client_handle,
                                          MCM_LOC_SET_POSITION_MODE_REQ_V01,
                                          &req, sizeof(req),
                                          &resp, sizeof(resp));
    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_mcm_loc::location_start_navigation() {

    mcm_loc_start_nav_req_msg_v01 req;
    mcm_loc_start_nav_resp_msg_v01 resp;
    uint32_t ret_val;

    ret_val = mcm_client_execute_command_sync(client_handle,
                                         MCM_LOC_START_NAV_REQ_V01,
                                         &req, sizeof(req),
                                         &resp, sizeof(resp));

    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_mcm_loc::location_stop_navigation() {

    mcm_loc_stop_nav_req_msg_v01 req;
    mcm_loc_stop_nav_resp_msg_v01 resp;
    uint32_t ret_val;
    ret_val = mcm_client_execute_command_sync(client_handle,
                                         MCM_LOC_STOP_NAV_REQ_V01,
                                         &req, sizeof(req),
                                         &resp, sizeof(resp));

    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_mcm_loc::location_close() {

    mcm_loc_set_indications_req_msg_v01 req;
    mcm_loc_set_indications_resp_msg_v01 resp;

    mcm_client_not_require_req_msg_v01 not_require_req_msg;
    mcm_client_not_require_resp_msg_v01 not_require_resp_msg;

    memset(&not_require_req_msg, 0, sizeof(mcm_client_not_require_req_msg_v01));
    memset(&not_require_resp_msg, 0, sizeof(mcm_client_not_require_resp_msg_v01));

    not_require_req_msg.not_require_service = MCM_LOC_V01;

    uint32_t ret_val;

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

    ret_val = mcm_client_execute_command_sync(client_handle,
                                           MCM_LOC_SET_INDICATIONS_REQ_V01,
                                           &req, sizeof(req),
                                           &resp, sizeof(resp));

    //Tell the framework that we don't need mcmlocserver anymore
    ret_val = MCM_CLIENT_EXECUTE_COMMAND_SYNC(client_handle,
                                               MCM_CLIENT_NOT_REQUIRE_REQ_V01,
                                               &not_require_req_msg,
                                               &not_require_resp_msg);

    // Disconnect from server
    mcm_client_release(client_handle);
    garden_cb_ptr = NULL;
    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_mcm_loc::location_delete_aiding_data(uint16_t flags) {

    mcm_loc_delete_aiding_data_req_msg_v01 req;
    mcm_loc_delete_aiding_data_resp_msg_v01 resp;
    uint32_t ret_val;
    loc_srv_conv_gps_aiding_data(&flags, &(req.flags));

    ret_val = mcm_client_execute_command_sync(client_handle,
                                           MCM_LOC_DELETE_AIDING_DATA_REQ_V01,
                                           &req, sizeof(req),
                                           &resp, sizeof(resp));

    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_mcm_loc::location_inject_time(int64_t time,
                                     int64_t timeReference,
                                     int uncertainty) {
    mcm_loc_inject_time_req_msg_v01 req;
    mcm_loc_inject_time_resp_msg_v01 resp;
    uint32_t ret_val;

    req.time = time;
    req.time_reference = timeReference;
    req.uncertainty = uncertainty;

    ret_val = mcm_client_execute_command_sync(client_handle,
                                           MCM_LOC_INJECT_TIME_REQ_V01,
                                           &req, sizeof(req),
                                           &resp, sizeof(resp));

    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_mcm_loc::location_inject_location(double latitude,
                                         double longitude,
                                         float accuracy) {

    mcm_loc_inject_location_req_msg_v01 req;
    mcm_loc_inject_location_resp_msg_v01 resp;
    uint32_t ret_val;

    req.latitude = latitude;
    req.longitude = longitude;
    req.accuracy = accuracy;

    ret_val = mcm_client_execute_command_sync(client_handle,
                                           MCM_LOC_INJECT_LOCATION_REQ_V01,
                                           &req, sizeof(req),
                                           &resp, sizeof(resp));

    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_mcm_loc::location_xtra_inject_data(char *data,
                                          int length) {

    mcm_loc_xtra_inject_data_req_msg_v01 req;
    mcm_loc_xtra_inject_data_resp_msg_v01 resp;
    uint32_t ret_val;
    int len = 0;

    len = (length > (MCM_LOC_MAX_XTRA_DATA_LENGTH_CONST_V01)) ?
          (MCM_LOC_MAX_XTRA_DATA_LENGTH_CONST_V01) :
          (length);

    memcpy((void*)req.data, (const void*)data, len);

    req.data_len = len;

    ret_val = mcm_client_execute_command_sync(client_handle,
                                           MCM_LOC_XTRA_INJECT_DATA_REQ_V01,
                                           &req, sizeof(req),
                                           &resp, sizeof(resp));
    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_mcm_loc::location_agps_data_conn_open(int16_t atype,
                                             const char *apn,
                                             int16_t btype) {

    mcm_loc_agps_data_conn_open_req_msg_v01 req;
    mcm_loc_agps_data_conn_open_resp_msg_v01 resp;
    uint32_t ret_val;

    loc_srv_conv_agps_type(&atype, &(req.agps_type));
    memcpy((void*)req.apn, (const void*)apn,
           MCM_LOC_MAX_APN_NAME_LENGTH_CONST_V01 + 1);
    loc_srv_conv_agps_bearer(&btype,
                            &(req.bearer_type));
    ret_val = mcm_client_execute_command_sync(client_handle,
                                           MCM_LOC_AGPS_DATA_CONN_OPEN_REQ_V01,
                                           &req, sizeof(req),
                                           &resp, sizeof(resp));
    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_mcm_loc::location_agps_data_conn_closed(int16_t atype) {

    mcm_loc_agps_data_conn_closed_req_msg_v01 req;
    mcm_loc_agps_data_conn_closed_resp_msg_v01 resp;
    uint32_t ret_val;
    loc_srv_conv_agps_type(&atype, &(req.agps_type));

    ret_val = mcm_client_execute_command_sync(client_handle,
                                          MCM_LOC_AGPS_DATA_CONN_CLOSED_REQ_V01,
                                          &req, sizeof(req),
                                          &resp, sizeof(resp));
    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_mcm_loc::location_agps_data_conn_failed(int16_t atype) {

    mcm_loc_agps_data_conn_failed_req_msg_v01 req;
    mcm_loc_agps_data_conn_failed_resp_msg_v01 resp;
    uint32_t ret_val;
    loc_srv_conv_agps_type(&atype, &(req.agps_type));

    ret_val = mcm_client_execute_command_sync(client_handle,
                                          MCM_LOC_AGPS_DATA_CONN_FAILED_REQ_V01,
                                          &req, sizeof(req),
                                          &resp, sizeof(resp));
    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_mcm_loc::location_agps_set_server(int16_t atype,
                                         const char * hostname,
                                         int port) {

    mcm_loc_agps_set_server_req_msg_v01 req;
    mcm_loc_agps_set_server_resp_msg_v01 resp;
    uint32_t ret_val;
    loc_srv_conv_agps_type(&atype, &(req.agps_type));

    memcpy((void*)req.host_name, (const void*)hostname,
           MCM_LOC_MAX_SEVER_ADDR_LENGTH_CONST_V01 + 1);
    req.port = port;

    ret_val = mcm_client_execute_command_sync(client_handle,
                                          MCM_LOC_AGPS_SET_SERVER_REQ_V01,
                                          &req, sizeof(req),
                                          &resp, sizeof(resp));
    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_mcm_loc::location_ni_respond(int notif_id,
                                    int user_response) {

    mcm_loc_ni_respond_req_msg_v01 req;
    mcm_loc_ni_respond_resp_msg_v01 resp;
    uint32_t ret_val;

    req.notif_id = notif_id;
    loc_srv_conv_gps_user_response(&user_response,
                                  &(req.user_response));
    ret_val = mcm_client_execute_command_sync(client_handle,
                                          MCM_LOC_NI_RESPOND_REQ_V01,
                                          &req, sizeof(req),
                                          &resp, sizeof(resp));

    return (resp.resp.result) ? (-1) : (0);
}

int location_test_interface_mcm_loc::location_agps_ril_update_network_availability
                                                   (int available,
                                                    const char * apn) {

    mcm_loc_agps_ril_update_network_availability_req_msg_v01 req;
    mcm_loc_agps_ril_update_network_availability_resp_msg_v01 resp;
    uint32_t ret_val;
    memcpy((void*)req.apn, (const void*)apn,
            MCM_LOC_MAX_APN_NAME_LENGTH_CONST_V01 + 1);
    req.available = available;

    ret_val = mcm_client_execute_command_sync(client_handle,
                           MCM_LOC_AGPS_RIL_UPDATE_NETWORK_AVAILABILITY_REQ_V01,
                           &req, sizeof(req),
                           &resp, sizeof(resp));
    return (resp.resp.result) ? (-1) : (0);
}

