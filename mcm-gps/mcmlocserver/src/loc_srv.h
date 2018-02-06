//=============================================================================
// FILE: loc_srv.h
//
// DESCRIPTION:
// Location service header
//
// Copyright (c) 2013-2014 Qualcomm Technologies, Inc.  All Rights Reserved.
// Qualcomm Technologies Proprietary and Confidential.
//=============================================================================

#ifndef LOC_SRV_H
#define LOC_SRV_H

#ifdef __cplusplus
extern "C" {
#endif

#define LOC_SRV_MAX_CLIENT_HANDLES          2

#define LOC_SRV_LOC_IND_FLAG                0
#define LOC_SRV_STATUS_IND_FLAG             1
#define LOC_SRV_SV_IND_FLAG                 2
#define LOC_SRV_NMEA_IND_FLAG               3
#define LOC_SRV_CAP_IND_FLAG                4
#define LOC_SRV_UTC_REQ_IND_FLAG            5
#define LOC_SRV_XTRA_DATA_REQ_IND_FLAG      6
#define LOC_SRV_AGPS_STATUS_IND_FLAG        7
#define LOC_SRV_NI_IND_FLAG                 8
#define LOC_SRV_XTRA_REPORT_SERVER_IND_FLAG 9


typedef struct {
    const LocGpsInterface *gps_iface_ptr;
    const LocAGpsInterface *agps_iface_ptr;
    const LocGpsXtraInterface *xtra_iface_ptr;
    const LocGpsNiInterface *ni_iface_ptr;
    const LocAGpsRilInterface *agps_ril_iface_ptr;
} loc_srv_hal_instance_t;


typedef struct {
    int loc_ind;
    int status_ind;
    int sv_ind;
    int nmea_ind;
    int cap_ind;
    int utc_req_ind;
    int xtra_data_req_ind;
    int xtra_report_server_ind;
    int agps_status_ind;
    int ni_ind;
} loc_srv_ind_flags_t;


typedef struct {
    qmi_client_handle client_handle;
    int enabled;
    int in_navigation;
    loc_srv_ind_flags_t ind_flags;
} loc_srv_client_info_t;


typedef struct {
    qmi_csi_service_handle service_handle;
    uint32_t num_clients;
    void * client_handle_list[LOC_SRV_MAX_CLIENT_HANDLES];
    int qmi_instance;
    int client_ref_count;
} loc_srv_state_info_cookie_t;


typedef qmi_csi_cb_error (* loc_srv_fwrk_proc_req_hdlr_func_t) (
    loc_srv_client_info_t         *client_info,
    qmi_req_handle                req_handle,
    unsigned int                  msg_id,
    void                          *req_c_struct,
    unsigned int                  req_c_struct_len,
    void                          *service_handle);



//=============================================================================
// FUNCTION: loc_srv_get_fwrk_proc_req_hdlr_table_size
//
// DESCRIPTION:
//  Returns the size of the Process request handler table
//
// @return
//       Size of the Process requst handler table
//=============================================================================
unsigned int loc_srv_get_fwrk_proc_req_hdlr_table_size();


//=============================================================================
// FUNCTION: loc_srv_get_fwrk_proc_req_hdlr_table_ele
//
// DESCRIPTION:
//  Returns a pointer to the handler functioin at the given index
//
// @return
//       Element at the given index
//=============================================================================
loc_srv_fwrk_proc_req_hdlr_func_t
          loc_srv_get_fwrk_proc_req_hdlr_table_ele(unsigned int msg_id);


//=============================================================================
// FUNCTION: loc_srv_get_gps_iface_ptr
//
// DESCRIPTION:
//  Returns a Pointer to the gps interface
//
// @return
//       Pointer to the gps interface
//=============================================================================
const LocGpsInterface* loc_srv_get_gps_iface_ptr();


//=============================================================================
// FUNCTION: loc_srv_get_agps_iface_ptr
//
// DESCRIPTION:
//  Returns a Pointer to the agps interface
//
// @return
//       Pointer to the agps interface
//=============================================================================
const LocAGpsInterface* loc_srv_get_agps_iface_ptr();


//=============================================================================
// FUNCTION: loc_srv_get_xtra_iface_ptr
//
// DESCRIPTION:
//  Returns a Pointer to the xtra interface
//
// @return
//       Pointer to the xtra interface
//=============================================================================
const LocGpsXtraInterface* loc_srv_get_xtra_iface_ptr();


//=============================================================================
// FUNCTION: loc_srv_get_ni_iface_ptr
//
// DESCRIPTION:
//  Returns a pointer to the ni interface
//
// @return
//       Pointer to the ni interface
//=============================================================================
const LocGpsNiInterface* loc_srv_get_ni_iface_ptr();


//=============================================================================
// FUNCTION: loc_srv_get_agps_ril_iface_ptr
//
// DESCRIPTION:
//  Returns a pointer to the agps ril interface
//
// @return
//       Pointer to the agps ril interface
//=============================================================================
const LocAGpsRilInterface* loc_srv_get_agps_ril_iface_ptr();


//=============================================================================
// FUNCTION: loc_srv_set_loc_indications
//
// DESCRIPTION:
//  Sets location indications
//
// @return
//       void
//=============================================================================
void loc_srv_set_loc_indications (
    int loc_ind,
    int status_ind,
    int sv_ind,
    int nmea_ind,
    int cap_ind,
    int utc_req_ind,
    int xtra_data_req_ind,
    int agps_status_ind,
    int ni_ind,
    int xtra_report_server_indi);

//=============================================================================
// FUNCTION: loc_srv_check_location_ind_registration
//
// DESCRIPTION:
//  Checks for registration
//
// @return
//       LOC_SRV_TRUE OR LOC_SRV_FALSE
//=============================================================================
int loc_srv_check_location_ind_registration();

//=============================================================================
// FUNCTION: loc_srv_check_status_ind_registration
//
// DESCRIPTION:
//   Checks for registration
//
// @return
//       LOC_SRV_TRUE OR LOC_SRV_FALSE
//=============================================================================
int loc_srv_check_status_ind_registration();

//=============================================================================
// FUNCTION: loc_srv_check_sv_ind_registration
//
// DESCRIPTION:
//  Checks for registration
//
// @return
//       LOC_SRV_TRUE OR LOC_SRV_FALSE
//=============================================================================
int loc_srv_check_sv_ind_registration();

//=============================================================================
// FUNCTION: loc_srv_check_nmea_ind_registration
//
// DESCRIPTION:
// Checks for registration
//
// @return
//       LOC_SRV_TRUE OR LOC_SRV_FALSE
//=============================================================================
int loc_srv_check_nmea_ind_registration();

//=============================================================================
// FUNCTION: loc_srv_check_cap_ind_registration
//
// DESCRIPTION:
//  Checks for registration
//
// @return
//       LOC_SRV_TRUE OR LOC_SRV_FALSE
//=============================================================================
int loc_srv_check_cap_ind_registration();

//=============================================================================
// FUNCTION: loc_srv_check_utc_req_ind_registration
//
// DESCRIPTION:
//   Checks for registration
//
// @return
//       LOC_SRV_TRUE OR LOC_SRV_FALSE
//=============================================================================
int loc_srv_check_utc_req_ind_registration();

//=============================================================================
// FUNCTION: loc_srv_check_xtra_data_req_ind_registration
//
// DESCRIPTION:
//  Checks for registration
//
// @return
//       LOC_SRV_TRUE OR LOC_SRV_FALSE
//=============================================================================
int loc_srv_check_xtra_data_req_ind_registration();

//=============================================================================
// FUNCTION: loc_srv_check_xtra_report_server_ind_registration
//
// DESCRIPTION:
//  Checks for registration
//
// @return
//       LOC_SRV_TRUE OR LOC_SRV_FALSE
//=============================================================================
int loc_srv_check_xtra_report_server_ind_registration();

//=============================================================================
//=============================================================================
// FUNCTION: loc_srv_check_agps_status_ind_registration
//
// DESCRIPTION:
//  Checks for registration
//
// @return
//       LOC_SRV_TRUE OR LOC_SRV_FALSE
//=============================================================================
int loc_srv_check_agps_status_ind_registration();

//=============================================================================
// FUNCTION: loc_srv_check_ni_ind_registration
//
// DESCRIPTION:
//  Checks for registration
//
// @return
//       LOC_SRV_TRUE OR LOC_SRV_FALSE
//=============================================================================
int loc_srv_check_ni_ind_registration();

//=============================================================================
// FUNCTION: loc_srv_check_registration
//
// DESCRIPTION:
//  Checks for registration at a specified index and flag
//
// @return
//       Pointer to the loc_srv_client_info_t or NULL if not registered
//=============================================================================
const loc_srv_client_info_t * loc_srv_check_registration(unsigned int index,
                                                         unsigned int flag);

#ifdef __cplusplus
}
#endif

#endif // LOC_SRV_H
