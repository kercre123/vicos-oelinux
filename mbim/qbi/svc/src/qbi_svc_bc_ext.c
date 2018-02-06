/*!
  @file
  qbi_svc_bc_ext.c

  @brief
  Basic Connectivity Extension device service definitions, based on the "MBIM
  LTE ATTACH APN and OTADM INTERFACE SPECIFICATION" and "Desktop multi-modem 
  multi-executor support" document from Microsoft.
*/

/*=============================================================================

  Copyright (c) 2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.

=============================================================================*/

/*=============================================================================

                        EDIT HISTORY FOR MODULE

This section contains comments describing changes made to the module.
Notice that changes are listed in reverse chronological order.

$Header$

when      who  what, where, why
--------  ---  ---------------------------------------------------------------
09/04/17  rv   1. Fixed Modem should fail query and set OIDs when No SIM is
               present. 2. Fixed Disabling class 2 DM config does not persist
               across factory restore. 3. Fixed Multiple APN Names can be 
               used for class 1.
08/11/17  rv   Prov Cntxt V2 Query to return both enabled/disabled profiles
07/26/17  mm   Fixed issue related to provisioning at the boot and hot swap
07/21/17  rv   Fixed crash when exec slot NV couldnt be created
07/20/17  mm   Fixed issue related to MEID
07/18/17  rv   Add logic to blow away cache if version is not present
07/11/17  vk   Fixed LTE Attach Status and LTE Attach Config
07/05/17  mm   Added logic to clear msuicc cache
07/05/17  vs   Added logic to close logical channel for slot mapping
06/28/17  rv   Fixed Prov Cntxt and LTE Attach Config Fatory Restore
06/26/17  vs   Fixed transition state handling during slot mapping
06/21/17  mm   Fixed issue related to slot info query
06/15/17  rv   Updated element count for LTE Attach query
06/15/17  mm   Fixed issue related to slot mapping
06/12/17  rv   Updated LTE attach handling for duplicate APNs
06/02/17  vk   Fixed double free of info pointer and cache access
05/30/17  rv   Updated LTE attach handling for set with duplicate APNs
05/23/17  mm   Added logic for getting slot state info
04/29/17  mm   Added device reset logic
04/25/17  rv   Fixed Profile Modify request when profile read is zero
05/05/17  rv   Handling empty username/password for LTE ATTACH CONFIG SET
04/28/17  rv   Fixed device caps query to return correct device type
03/22/17  mm   Code cleanup for DSSA feature and updated slot mapping feature
02/15/17  vk   Fixed provisoned context for create and modified operations
12/08/16  vk   Added module
=============================================================================*/

/*=============================================================================

  Include Files

=============================================================================*/

#include "qbi_svc_bc_ext.h"
#include "qbi_svc_bc_ext_mbim.h"
#include "qbi_svc_bc_common.h"
#include "qbi_svc_msuicc.h"
#include "qbi_svc_bc_spdp.h"

#include "qbi_common.h"
#include "qbi_mbim.h"
#include "qbi_nv_store.h"
#include "qbi_qmi_txn.h"
#include "qbi_svc.h"
#include "qbi_txn.h"
#include "qbi_msg_mbim.h"

#include "wireless_data_service_v01.h"
#include "data_system_determination_v01.h"
#include "persistent_device_configuration_v01.h"
#include "device_management_service_v01.h"
#include "user_identity_module_v01.h"
#include "network_access_service_v01.h"

/*=============================================================================

  Private Constants and Macros

=============================================================================*/

/*! This macro statically defines a QMI indication handler and fills in the
    fields that are common to all handlers in this device service */
#define QBI_SVC_BC_EXT_STATIC_IND_HDLR(qmi_svc_id, qmi_msg_id, cid, cb) \
  {qmi_svc_id, qmi_msg_id, cid, cb, NULL}

#define QBI_SVC_BC_EXT_MAP_DEFAULT_IP_TYPE(ip_type) \
  (((ip_type) == QBI_SVC_BC_IP_TYPE_DEFAULT) ? \
    QBI_SVC_BC_IP_TYPE_IPV4V6 : (ip_type))

/*! Validate DUAL IP Stack with MBIM IP type */
#define QBI_SVC_BC_EXT_IS_MBIM_DUAL_IP(ip_type) \
  (QBI_SVC_BC_EXT_MAP_DEFAULT_IP_TYPE(ip_type) == QBI_SVC_BC_IP_TYPE_IPV4V6 || \
   QBI_SVC_BC_EXT_MAP_DEFAULT_IP_TYPE(ip_type) == QBI_SVC_BC_IP_TYPE_IPV4_AND_IPV6)

/*! Returns TRUE if the given IP type includes IPv4 */
#define QBI_SVC_BC_EXT_IPV4_REQUESTED(ip_type) \
   (QBI_SVC_BC_EXT_MAP_DEFAULT_IP_TYPE(ip_type) == QBI_SVC_BC_IP_TYPE_IPV4 || \
    QBI_SVC_BC_EXT_IS_MBIM_DUAL_IP(ip_type))

/*! Returns TRUE if the given IP type includes IPv6 */
#define QBI_SVC_BC_EXT_IPV6_REQUESTED(ip_type) \
   (QBI_SVC_BC_EXT_MAP_DEFAULT_IP_TYPE(ip_type) == QBI_SVC_BC_IP_TYPE_IPV6 || \
    QBI_SVC_BC_EXT_IS_MBIM_DUAL_IP(ip_type))

#define QBI_SVC_EXT_ID_TO_INDEX(svc_id) (svc_id - QBI_SVC_ID_OFFSET)

/*! Map session ID to QMI WDS IPv4 service ID */
#define QBI_SVC_BC_EXT_SESSION_ID_TO_WDS_SVC_ID_IPV4(session_id) \
   (session_id < QBI_SVC_BC_EXT_MAX_SESSIONS ? \
    (qbi_qmi_svc_e)(QBI_QMI_SVC_WDS_FIRST + session_id * 2) : \
    (qbi_qmi_svc_e)QBI_QMI_SVC_WDS_FIRST)

/*! Map session ID to QMI WDS IPv6 service ID */
#define QBI_SVC_BC_EXT_SESSION_ID_TO_WDS_SVC_ID_IPV6(session_id) \
   (session_id < QBI_SVC_BC_EXT_MAX_SESSIONS ? \
    ((qbi_qmi_svc_e)(QBI_QMI_SVC_WDS_FIRST + session_id * 2 + 1)) : \
    (qbi_qmi_svc_e)(QBI_QMI_SVC_WDS_FIRST + 1))

/*! Map QMI WDS service ID to session ID */
#define QBI_SVC_BC_EXT_WDS_SVC_ID_TO_SESSION_ID(wds_svc_id) \
   ((wds_svc_id >= QBI_QMI_SVC_WDS_FIRST && wds_svc_id <= QBI_QMI_SVC_WDS_LAST) ? \
    (wds_svc_id - QBI_QMI_SVC_WDS_FIRST) / 2 : 0)

/*! Number of APN 's Rows */
#define QBI_SVC_BC_EXT_OPERATOR_APN_ROW 4
/*! Number of APN 's Columns */
#define QBI_SVC_BC_EXT_OPERATOR_APN_COL 2

/*! Map QMI WDS service ID to session ID */
#define QBI_SVC_BC_EXT_INVALID_PROFILE_INDEX               (0)

/*! @addtogroup QBI_SVC_BC_EXT_MBIM_CID_MS_LTE_ATTACH_CONFIG
    @{ */
#define QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_MAX_PROFILE     (3)

/*! @} */

/*! @addtogroup MBIM_CID_MS_DEVICE_CAPS_V2
    @{ */
#define QBI_SVC_BC_EXT_DEFAULT_EXECUTOR_INDEX              (0)

#define QBI_SVC_BC_EXT_MODEM_ID_INFO_MAX_LEN_BYTES         (36)
#define QBI_SVC_BC_EXT_MAX_SUBS_INFO_MAX_LEN_BYTES         (4)

#define MEIDCHAR_0 48 /* ascii character '0' */
#define MEIDCHAR_9 57 /* ascii character '9' */
#define MEIDCHAR_A 65 /* ascii character 'A' */
#define MEIDCHAR_F 70 /* ascii character 'F' */

/*! @} */

/*! @addtogroup MBIM_CID_MS_DEVICE_SLOT_MAPPING
    @{ */

#define QBI_SVC_BC_EXT_UIM_PROVISIONING_SESSION_INVALID   0xFFFF
#define QBI_SVC_BC_EXT_UIM_SLOT_INFO_MASK                 0xFF00
#define QBI_SVC_BC_EXT_UIM_APP_INFO_MASK                  0x00FF

#define QBI_SVC_BC_EXT_MBIM_PROVISIONING_SESSION_INVALID  0xFFFFFFFF
#define QBI_SVC_BC_EXT_MBIM_MAX_SLOT_SUPPORTED            (2)

#define QBI_SVC_BC_EXT_PRIMARY_GW                         0x00010000
#define QBI_SVC_BC_EXT_PRIMARY_1x                         0x00100000

#define QBI_SVC_BC_EXT_SLOT_INDEX_0                       (0)
#define QBI_SVC_BC_EXT_SLOT_INDEX_1                       (1)

#define QBI_SVC_BC_EXT_GET_APP_INDEX(app_idx)     \
          (app_idx & QBI_SVC_BC_EXT_UIM_APP_INFO_MASK)

#define QBI_SVC_BC_EXT_GET_SLOT_BYTE(slot_idx)\
          (slot_idx & QBI_SVC_BC_EXT_UIM_SLOT_INFO_MASK) >> 8

#define QBI_SVC_BC_EXT_MAX_SUPPORTED_EXECUTORS             (1)
#define QBI_SVC_BC_EXT_SLOT_ACTIVATE                       (1)
#define QBI_SVC_BC_EXT_SLOT_DEACTIVATE                     (0)
#define QBI_SVC_BC_EXT_SLOT_1                              (0)
#define QBI_SVC_BC_EXT_SLOT_2                              (1)


typedef enum {
 QBI_SVC_BC_EXT_APP_TYPE_GW = 1,
 QBI_SVC_BC_EXT_APP_TYPE_1X
}qbi_svc_bc_ext_app_type_enum;


/*! @} */

/* Provision context_v2 */
#define PROV_V2_IND_TOKEN 0x1234

/*=============================================================================

  Private Typedefs

=============================================================================*/

/*! Cache used locally by CIDs processed in this file. This is a child of the
main qbi_svc_bc_ext_cache_s structure */

typedef enum {
  QBI_SVC_BC_EXT_ROAMING_FLAG_MIN = 0x00,

  QBI_SVC_BC_EXT_ROAMING_FLAG_HOME = 0x01,
  QBI_SVC_BC_EXT_ROAMING_FLAG_PARTNER = 0x02,
  QBI_SVC_BC_EXT_ROAMING_FLAG_NON_PARTNER = 0x04,

  QBI_SVC_BC_EXT_ROAMING_FLAG_MAX
} qbi_svc_bc_ext_roaming_flag_e;

typedef enum {
  QBI_SVC_BC_EXT_CONTEXT_FLAG_MIN = 0,

  QBI_SVC_BC_EXT_CONTEXT_FLAG_MODEM = 1,
  QBI_SVC_BC_EXT_CONTEXT_FLAG_USER_DEFINED = 2,
  QBI_SVC_BC_EXT_CONTEXT_FLAG_USER_MODIFIED = 3,

  QBI_SVC_BC_EXT_CONTEXT_FLAG_MAX
} qbi_svc_bc_ext_context_flag_e;

typedef PACK(struct) {
  uint32 ip_type;
  uint32 source;
  uint32 roaming;
  uint32 media_type;
  uint32 enable;
  uint32 prov_active;
  uint32 lte_active;
  uint32 lte_attach_state;
  uint32 roaming_flag;
  uint32 context_flag;
} qbi_svc_bc_ext_cache_s;

static boolean cmd_in_progress_ignore_indication = FALSE;

/*! @addtogroup MBIM_CID_MS_PROVISIONED_CONTEXT_V2
    @{ */

/*! IMSI range check for operator specific customization */
#define QBI_SVC_BC_EXT_IMSI_311_480 "311480"
#define QBI_SVC_BC_EXT_IMSI_311_270 "311270"
#define QBI_SVC_BC_EXT_IMSI_312_770 "312770"

/*! IMSI Range max length to be compared */
#define QBI_SVC_BC_EXT_IMSI_311_480_MAX_LEN 6
#define QBI_SVC_BC_EXT_IMSI_311_270_MAX_LEN 6
#define QBI_SVC_BC_EXT_IMSI_312_770_MAX_LEN 6

/* APN Listed Class wise for specific operator 
   APN names in each row are synonymous as per operator requirement */
static const char* qbi_svc_bc_ext_provisioned_contexts_v2_operator_apn
[QBI_SVC_BC_EXT_OPERATOR_APN_ROW][QBI_SVC_BC_EXT_OPERATOR_APN_COL] = 
{
  {"vzwims", "ims"  },   /*! Class 1 */
  {"vzwadmin", ""   },   /*! Class 2 */
  {"vzwinternet", ""},   /*! Class 3 */
  {"vzwapp" , ""     }    /*! Class 4 */
};

/*! @brief Profile list containing basic profile info */
typedef struct {
  uint32                    num_of_profile;
  wds_profile_type_enum_v01 profile_type[QMI_WDS_PROFILE_LIST_MAX_V01];
  uint8_t                   profile_index[QMI_WDS_PROFILE_LIST_MAX_V01];
} qbi_svc_bc_ext_profile_list_s;

/*! Collection of 3gpp settings used for finding matching 3gpp2 profile */
typedef struct {
  uint8_t apn_name_valid;
  char    apn_name[QMI_WDS_APN_NAME_MAX_V01 + 1];
  uint8_t username_valid;
  char    username[QMI_WDS_USER_NAME_MAX_V01 + 1];
} qbi_svc_bc_ext_provisioned_contexts_v2_3gpp_profile_settings_s;

/*! Tracking information for retrieving profiles */
typedef struct {
  /*! Number of profiles that have been retrieved so far */
  uint32 profiles_read;

  /*! Store EPC profile scan status and matching profile index */
  uint8 profile_found_epc;
  uint8 profile_index_epc;

  /*! Store 3gpp profile scan status and matching profile index */
  uint8 profile_found_3gpp;
  uint8 profile_index_3gpp;

  /*! Store 3gpp2 profile scan status and matching profile index */
  uint32 profile_found_3gpp2;
  uint32 profile_index_3gpp2;

  /*! Store number of profiles from profile list */
  uint32 num_of_profile_epc;
  uint32 num_of_profile_3gpp;
  uint32 num_of_profile_3gpp2;

  /*! 3gpp profile settings for finding matching 3gpp2 profile */
  qbi_svc_bc_ext_provisioned_contexts_v2_3gpp_profile_settings_s profile_settings_3gpp;

  /*! Buffer to store profile index info from last QMI profile list query */
  qbi_svc_bc_ext_profile_list_s profile_list;

  /* This is used to keep track whether any matching profile was found while in set request */
  boolean profile_matched;

  /* This is used to save the profile index for the matched request while in set request */
  uint8_t matched_index;

  /* This keeps track whether the requested operation has been completed */
  boolean operation_completed;
} qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s;

/*! Collection of pointers to relevant TLVs in
    QMI_WDS_MODIFY_PROFILE_SETTINGS_REQ and QMI_WDS_CREATE_PROFILE_REQ */
typedef struct {
  uint8_t                            *apn_name_valid;
  char                               *apn_name;
  uint8_t                            *user_id_valid;
  char                               *user_id;
  uint8_t                            *auth_password_valid;
  char                               *auth_password;
  uint8_t                            *auth_protocol_valid;
  wds_profile_auth_protocol_enum_v01 *auth_protocol;
  uint8_t                            *authentication_preference_valid;
  wds_auth_pref_mask_v01             *authentication_preference;
  uint8_t                            *app_user_data_valid;
  uint32_t                           *app_user_data;
  uint8_t                            *pdp_data_compression_type_valid;
  wds_pdp_data_compr_type_enum_v01   *pdp_data_compression_type;
  uint8_t                            *pdp_hdr_compression_type_valid;
  wds_pdp_hdr_compr_type_enum_v01    *pdp_hdr_compression_type;
  uint8_t                            *common_apn_disabled_flag_valid;
  uint8_t                            *common_apn_disabled_flag;
  uint8_t                            *apn_disabled_flag_valid;
  uint8_t                            *apn_disabled_flag;
  uint8_t                            *apn_enabled_3gpp2_valid;
  uint8_t                            *apn_enabled_3gpp2;
  uint8_t                            *common_pdp_type_valid;
  wds_common_pdp_type_enum_v01       *common_pdp_type;
  uint8_t                            *pdp_type_valid;
  wds_pdp_type_enum_v01              *pdp_type;
} qbi_svc_bc_ext_provisioned_contexts_v2_profile_settings_ptrs_s;

/*! @} */

/*! @addtogroup MBIM_CID_MS_LTE_ATTACH_CONFIG
    @{ */

/*! Collection of pointers to relevant TLVs in
    QMI_WDS_MODIFY_PROFILE_SETTINGS_REQ and QMI_WDS_CREATE_PROFILE_REQ */
typedef struct {
  uint8_t                           roaming_disallowed_valid;
  uint8_t                           roaming_disallowed;
  uint8_t                           pdp_type_valid;
  wds_pdp_type_enum_v01             pdp_type;
  uint8_t                           *authentication_preference_valid;
  wds_auth_pref_mask_v01            *authentication_preference;
  uint8_t                           *pdp_data_compression_type_valid;
  wds_pdp_data_compr_type_enum_v01  *pdp_data_compression_type;
  uint8_t                           *pdp_hdr_compression_type_valid;
  wds_pdp_hdr_compr_type_enum_v01   *pdp_hdr_compression_type;
} qbi_svc_bc_ext_lte_attach_config_profile_settings_s;

/*! Tracking information for retrieving profiles */
typedef struct {
  /*! Maximum number of attached PDNs supported by the device. */
  uint32 max_supported_profile_num;

  /*! Number of sets of the attach_pdn_list elements */
  uint32 num_of_profile;

  /*! Number of profiles that have been retrieved so far */
  uint32 profiles_read;

  /*! Current element for which profile info is being retrieved */
  uint32 element_read;

  /*! Profile IDs to matched element mapping. Its size will be
  element_count of qbi_svc_bc_ext_lte_attach_config_info_rsp_s*/
  uint32 element_match_index[QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_MAX_PROFILE];

  /*! PDN profile IDs to attach to, listed in order of decreasing priority. */
  uint16 profile_index[QMI_WDS_ATTACH_PDN_MAX_V01];
} qbi_svc_bc_ext_lte_attach_config_info_s;

/*! @} */

/*! @addtogroup MBIM_CID_MS_LTE_ATTACH_STATUS
    @{ */

/*! Collection of pointers to relevant TLVs in
    QMI_WDS_GET_PROFILE_SETTINGS_REQ */
typedef struct {
  uint8_t                           pdp_type_valid;
  wds_pdp_type_enum_v01             pdp_type;
  uint8_t                           authentication_preference_valid;
  wds_auth_pref_mask_v01            authentication_preference;
  uint8_t                           pdp_data_compression_type_valid;
  wds_pdp_data_compr_type_enum_v01  pdp_data_compression_type;
  uint8_t                           pdp_hdr_compression_type_valid;
  wds_pdp_hdr_compr_type_enum_v01   pdp_hdr_compression_type;
  uint8_t                           apn_name_valid;
  char                              apn_name[QMI_WDS_APN_NAME_MAX_V01 + 1];
  uint8_t                           username_valid;
  char                              username[QMI_WDS_USER_NAME_MAX_V01 + 1];
  uint8_t                           password_valid;
  char                              password[QMI_WDS_PASSWORD_MAX_V01 + 1];
} qbi_svc_bc_ext_lte_attach_status_profile_settings_s;

/*! Tracking information for retrieving profiles */
typedef struct {
  /*! LTE attach state. */
  uint32  lte_attach_state;

  /*! Registeration status. */
  uint32  status_registered;

  /*! Maximum number of attached PDNs supported by the device. */
  uint32  max_supported_profile_num;

  /*! Number of sets of the LTE attach PDNs */
  uint32  num_of_profile;

  /*! Number of profiles that have been retrieved so far */
  uint32  profiles_read;

  /*! PDN profile index of LTE attach PDNs. */
  uint16  profile_index[QMI_WDS_ATTACH_PDN_MAX_V01];

  /*! PDN profile APN for LTE attach PDN. */
  char    apn_name[QMI_WDS_APN_NAME_MAX_V01 + 1];

  /*! PDN profile IP type for LTE attach PDN. */
  wds_ip_support_type_enum_v01 ip_type;
} qbi_svc_bc_ext_lte_attach_status_info_s;

/*! @} */

/*! @addtogroup MBIM_CID_MS_DEVICE_SLOT_MAPPING
    @{ */

typedef enum
{
  QBI_SVC_BC_EXT_PROV_SESSION_MIN = 0,
  
  QBI_SVC_BC_EXT_PROV_SESSION_DEACTIVATE_PRIMARY    = 1,
  QBI_SVC_BC_EXT_PROV_SESSION_DEACTIVATE_SECONDARY  = 2,
  QBI_SVC_BC_EXT_PROV_SESSION_ACTIVATE_PRIMARY      = 3,
  QBI_SVC_BC_EXT_PROV_SESSION_ACTIVATE_SECONDARY    = 4,

  QBI_SVC_BC_EXT_PROV_SESSION_MAX
}qbi_svc_bc_ext_prov_session_status_e;

typedef struct {

  uint8   appindex;
  boolean app_present;
  boolean deact_need;
  boolean act_need;
  boolean deact_done;
  boolean act_done;
  uint8 aid[QMI_UIM_AID_MAX_V01];
  uint8 aid_length;
} qbi_svc_bc_ext_app_info_s;

typedef struct {

  uint32 slot;
  qbi_svc_bc_ext_app_info_s info_gw;
  qbi_svc_bc_ext_app_info_s info_1x;

} qbi_svc_bc_ext_app_slot_info_s;

/*! Tracking UIM card slot information  */
typedef struct {

  qbi_svc_bc_ext_app_slot_info_s curr;
  qbi_svc_bc_ext_app_slot_info_s req;
  boolean cmd_info_extracted;
  uint8 total_num_slots;
} qbi_svc_bc_ext_slot_map_cmd_info_s;

/*! @} */

/*! @addtogroup QBI_SVC_BC_EXT_MBIM_CID_MS_SLOT_INFO_STATUS
    @{ */
typedef struct {
  uint32 slot_state;
  uint8 is_esim;
  uint32 card_state;
  uint32 app_info_len;
  uint8 card_error;
} qbi_svc_bc_ext_info_cache_each_slot;

typedef struct {
  qbi_svc_bc_ext_info_cache_each_slot card0;
  qbi_svc_bc_ext_info_cache_each_slot card1;
}qbi_svc_bc_ext_info_status_cache;

typedef struct {
  uint8 is_esim;
}qbi_svc_bc_ext_slot_info;

/*! @} */

/*=============================================================================

  Private Function Prototypes

=============================================================================*/

static void qbi_svc_bc_ext_update_version
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_open_uim2f_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_build_uim38_req
(
  qbi_qmi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_open_uim38_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_open_configure_qmi_inds
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_open_configure_qmi_inds_uim2e_cb
(
  qbi_qmi_txn_s *qmi_txn
);

/*! @addtogroup MBIM_CID_MS_PROVISIONED_CONTEXT_V2
    @{ */

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_q_req
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_s_req
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_q_wds2a_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_q_get_next_profile
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_q_wds2b_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static boolean qbi_svc_ext_provisioned_contexts_v2_q_add_context_to_rsp
(
  qbi_txn_s                                   *txn,
  qbi_mbim_offset_size_pair_s                 *field_desc,
  wds_profile_type_enum_v01                    profile_type,
  const wds_get_profile_settings_resp_msg_v01 *qmi_rsp,
  uint32                                       context_id
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds2a_req
(
  qbi_txn_s                *txn,
  wds_profile_type_enum_v01 profile_type
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_wds2a_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds28_req
(
  qbi_txn_s                *txn,
  wds_profile_type_enum_v01 profile_type,
  uint32                    index
);

static void qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status
(
  qbi_txn_s                          *txn,
  qmi_error_type_v01                  qmi_error,
  uint8_t                             qmi_error_ds_ext_valid,
  wds_ds_extended_error_code_enum_v01 qmi_error_ds_ext
);

static uint32 qbi_svc_bc_ext_qmi_profile_to_mbim_auth_proto
(
  wds_profile_type_enum_v01                    profile_type,
  const wds_get_profile_settings_resp_msg_v01 *qmi_rsp
);

static const uint8 *qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_to_context_type
(
  const wds_get_profile_settings_resp_msg_v01 *qmi_rsp
);

static boolean qbi_svc_bc_ext_populate_profile_list
(
  qbi_svc_bc_ext_profile_list_s         *profile_list,
  wds_get_profile_list_resp_msg_v01 *qmi_rsp
);

static uint32 qbi_svc_bc_ext_qmi_profile_to_mbim_compression
(
  wds_profile_type_enum_v01                    profile_type,
  const wds_get_profile_settings_resp_msg_v01 *qmi_rsp
);

static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_compare_3gpp_3gpp2_profiles
(
  qbi_svc_bc_ext_provisioned_contexts_v2_3gpp_profile_settings_s *profile_settings_3gpp,
  wds_get_profile_settings_resp_msg_v01                   *qmi_rsp_3gpp2
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_3gpp2_wds2b_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_get_next_3gpp2_profile
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds2a_req
(
  qbi_txn_s                *txn,
  wds_profile_type_enum_v01 profile_type
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_wds2b_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_get_next_profile
(
  qbi_txn_s *txn,
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_name_matched
(
  qbi_qmi_txn_s                         *qmi_txn,
  wds_get_profile_settings_resp_msg_v01 *profile_settings
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_wds28_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile
(
  qbi_txn_s                                               *txn,
  wds_profile_type_enum_v01                                profile_type,
  qbi_svc_bc_ext_provisioned_contexts_v2_profile_settings_ptrs_s *profile_settings
);

static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_str
(
  qbi_txn_s                         *txn,
  const qbi_mbim_offset_size_pair_s *field_desc,
  uint32                             field_max_size,
  char                              *qmi_field,
  uint32                             qmi_field_size
);

static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_get_profile_settings_ptrs_wds28
(
  wds_modify_profile_settings_req_msg_v01                 *qmi_req,
  qbi_svc_bc_ext_provisioned_contexts_v2_profile_settings_ptrs_s *profile_settings,
  wds_profile_type_enum_v01                                profile_type
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_wds27_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds27_req
(
  qbi_txn_s *txn
);

static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_get_profile_settings_ptrs_wds27
(
  wds_create_profile_req_msg_v01                 *qmi_req,
  qbi_svc_bc_ext_provisioned_contexts_v2_profile_settings_ptrs_s *profile_settings,
  wds_profile_type_enum_v01                                profile_type
);

static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile_compression
(
  const qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s           *req,
  wds_profile_type_enum_v01                                profile_type,
  qbi_svc_bc_ext_provisioned_contexts_v2_profile_settings_ptrs_s *profile_settings
);

static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile_auth_protocol
(
  qbi_txn_s                                               *txn,
  const qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s           *req,
  wds_profile_type_enum_v01                                profile_type,
  qbi_svc_bc_ext_provisioned_contexts_v2_profile_settings_ptrs_s *profile_settings
);

static wds_profile_auth_protocol_enum_v01 qbi_svc_bc_ext_connect_mbim_auth_pref_to_qmi_auth_protocol
(
  uint32 auth_protocol
);

static wds_auth_pref_mask_v01 qbi_svc_bc_ext_connect_mbim_auth_pref_to_qmi_auth_pref
(
  uint32 auth_protocol
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_wds29_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_is_pdp_type_matched
(
  qbi_qmi_txn_s                         *qmi_txn,
  wds_get_profile_settings_resp_msg_v01 *profile_settings
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_qmi_ind_reg_wdsa8_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_qmi_ind_reg_wdsa8_req
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_qmi_ind_reg_pdc20_res
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc2b_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc22_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc23_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc27_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_s_pdc27_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
);

static qbi_svc_action_e qbi_svc_bc_ext_pdc2f_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
);

static qbi_svc_action_e qbi_svc_bc_ext_wds29_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_wdsa8_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
);

static qbi_svc_action_e qbi_svc_bc_ext_open_configure_qmi_inds
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_factory_reset
(
  qbi_txn_s *txn
);

static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class1
(
  qbi_ctx_s    *ctx,
  const char *apn_name
);

static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class2
(
  const char *apn_name
);

static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_operator
(
  const char  *apn_name
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_pdc_get_next_profile
(
  qbi_txn_s *txn
);

static boolean qbi_svc_bc_ext_provisioned_context_v2_s_register_for_pdc_ind
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_wds29_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_wds29_req
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_prep_wds29_req
(
  qbi_txn_s *txn
);

/*! @} */

/*! @addtogroup MBIM_CID_MS_LTE_ATTACH_CONFIG
    @{ */

qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_static_ind_e_wds95_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
);

qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_nas34_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_q_wds92_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds93_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_q_wds94_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);
  
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds9f_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_q_wds2b_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds27_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_build_wds28_req
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds28_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_build_wds29_req
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds29_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_nas33_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_build_nas33_req
(
  qbi_txn_s *txn,
  boolean flag
);

static boolean qbi_svc_bc_ext_lte_attach_config_s_get_profile_list
(
  qbi_txn_s *txn,
  wds_get_lte_attach_pdn_list_resp_msg_v01 *qmi_rsp
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_operation_factory_restore
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_operation_default
(
  qbi_txn_s *txn
);

static boolean qbi_svc_bc_ext_lte_attach_config_s_register_for_detach_attach
(
  qbi_txn_s *txn
);

static void qbi_svc_bc_ext_lte_attach_config_set_mbim_error_status
(
  qbi_txn_s                          *txn,
  qmi_error_type_v01                  qmi_error,
  uint8_t                             qmi_error_ds_ext_valid,
  wds_ds_extended_error_code_enum_v01 qmi_error_ds_ext
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds94_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds2b_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_build_wds93_req
(
  qbi_txn_s *txn
);

boolean qbi_svc_bc_ext_lte_attach_config_s_is_profile_match_pending
(
  qbi_svc_bc_ext_lte_attach_config_info_s * info
);

qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds95_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
);

/*! @} */

/*! @addtogroup MBIM_CID_MS_LTE_ATTACH_STATUS
    @{ */

qbi_svc_action_e qbi_svc_bc_ext_lte_attach_status_e_dsd26_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_status_q_dsd24_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_status_q_wds85_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_status_q_wds92_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_status_q_wds94_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_status_q_wds2b_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_status_reg_ind_dsd25_rsp
(
  qbi_qmi_txn_s *qmi_txn
);

/*! @} */

/*! @addtogroup QBI_SVC_BC_EXT_MBIM_CID_MS_SYS_CAPS
    @{ */

static qbi_svc_action_e qbi_svc_bc_ext_sys_caps_info_q_uim2f_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_sys_caps_info_q_dms20_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_sys_caps_info_q_dms25_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static void qbi_svc_bc_ext_sys_caps_info_q_get_meid
(
  qbi_qmi_txn_s *qmi_txn
);

static uint32  qbi_svc_bc_ext_sys_caps_info_q_get_meid_char_to_int
(
  char meid_char
);

static qbi_svc_action_e qbi_svc_bc_ext_sys_caps_info_q_req
(
  qbi_txn_s *txn
);

/*! @} */

/*! @addtogroup QBI_SVC_BC_EXT_MBIM_CID_MS_DEVICE_CAPS_V2
    @{ */

static qbi_svc_action_e qbi_svc_bc_ext_device_caps_v2_q_req
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_device_caps_info_q_dms25_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_device_caps_q_dms22_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_device_caps_q_dms23_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_device_caps_q_dms20_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static uint32 qbi_svc_bc_ext_open_configure_qmi_radio_if_list_to_mbim_data_class
(
  const dms_radio_if_enum_v01 *radio_if_list,
  uint32                       radio_if_list_len
);

static qbi_svc_action_e qbi_svc_bc_ext_device_caps_q_rsp
(
  qbi_txn_s *txn
);

/*! @} */

/*! @addtogroup MBIM_CID_MS_DEVICE_SLOT_MAPPING
    @{ */

static qbi_svc_action_e qbi_svc_bc_ext_change_prov_session_deact_s_uim38_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_change_prov_session_act_s_uim38_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_slot_mapping_q_req
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_slot_mapping_s_req
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_slot_mapping_q_uim2f_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_slot_mapping_s_uim2f_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static void qbi_svc_bc_ext_populate_app_slot_info
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_slot_mapping_s_deact_uim2e_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_slot_mapping_s_deact_req
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_slot_mapping_s_act_req
(
  qbi_qmi_txn_s *qmi_txn
);

static boolean qbi_svc_bc_ext_slot_mapping_get_aidinfo
(
  card_info_type_v01            card_info,
  qbi_svc_bc_ext_app_info_s     *info,
  qbi_svc_bc_ext_app_type_enum  app_type
);

static qbi_svc_action_e qbi_svc_bc_ext_open_act_req
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_open_act_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static boolean qbi_svc_bc_ext_slot_mapping_close_logical_channels
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_slot_mapping_close_channel_s_uim3f_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

/*! @} */

/*! @addtogroup QBI_SVC_BC_EXT_MBIM_CID_MS_SLOT_INFO
    @{ */

static qbi_svc_action_e qbi_svc_bc_ext_slot_info_status_q_req
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_slot_info_status_q_atr_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_slot_info_q_uim2f_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static qbi_svc_action_e qbi_svc_bc_ext_slot_info_uim32_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
);

static void qbi_svc_bc_ext_slot_info_ind_q_msuicc_cache_release
(
  qbi_txn_s *txn
);

/*! @} */

/*! @addtogroup QBI_SVC_BC_EXT_MBIM_CID_MS_DEVICE_RESET
    @{ */

static qbi_svc_action_e qbi_svc_bc_ext_device_reset_s_req
(
  qbi_txn_s *txn
);

static qbi_svc_action_e qbi_svc_bc_ext_device_reset_s_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

/*! @} */

static boolean qbi_svc_bc_ext_slot_status_update_cache
(
  qbi_svc_bc_ext_info_cache_each_slot *cache,
  uint32 slot_state
);

static qbi_svc_bc_ext_info_status_cache *qbi_svc_bc_ext_slot_info_cache_get
(
  qbi_ctx_s *ctx
);

static void qbi_svc_bc_ext_slot_info_q_prepare_rsp
(
  qbi_txn_s           *txn,
  uint32              slot_idx,
  card_info_type_v01  card_info
);

static qbi_svc_action_e qbi_svc_bc_ext_slot_info_uim32_ind_q_card0_atr_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

static uint32 qbi_svc_bc_ext_slot_info_from_uicc_slot_state
(
  qbi_svc_bc_ext_info_cache_each_slot *cache
);

static void qbi_svc_bc_ext_slot_info_force_card1_event
(
  qbi_ctx_s *ctx
);

static qbi_svc_action_e qbi_svc_bc_ext_slot_info_uim32_ind_q_card1_atr_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
);

/*=============================================================================

  Private Variables

=============================================================================*/
/*! @brief Static QMI indication handlers (generally, CID event handlers)
*/
static const qbi_svc_ind_info_s qbi_svc_bc_ext_static_ind_hdlrs[] = {
  QBI_SVC_BC_EXT_STATIC_IND_HDLR(QBI_QMI_SVC_WDS, 
                                QMI_WDS_PROFILE_EVENT_REGISTER_IND_V01,
                                QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2,
                                qbi_svc_bc_ext_provisioned_context_v2_wdsa8_ind_cb),
  QBI_SVC_BC_EXT_STATIC_IND_HDLR(QBI_QMI_SVC_PDC, 
                                QMI_PDC_ACTIVATE_CONFIG_IND_V01,
                                QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2,
                                qbi_svc_bc_ext_provisioned_context_v2_s_pdc27_ind_cb),
  QBI_SVC_BC_EXT_STATIC_IND_HDLR(QBI_QMI_SVC_PDC, 
                                QMI_PDC_REFRESH_IND_V01,
                                QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2,
                                qbi_svc_bc_ext_pdc2f_ind_cb),
  QBI_SVC_BC_EXT_STATIC_IND_HDLR(QBI_QMI_SVC_WDS, 
                                QMI_WDS_LTE_ATTACH_PDN_LIST_IND_V01,
                                QBI_SVC_BC_EXT_MBIM_CID_MS_LTE_ATTACH_CONFIG,
                                qbi_svc_bc_ext_lte_attach_config_static_ind_e_wds95_ind_cb),
  QBI_SVC_BC_EXT_STATIC_IND_HDLR(QBI_QMI_SVC_DSD, 
                                QMI_DSD_SYSTEM_STATUS_IND_V01,
                                QBI_SVC_BC_EXT_MBIM_CID_MS_LTE_ATTACH_STATUS,
                                qbi_svc_bc_ext_lte_attach_status_e_dsd26_ind_cb),
  QBI_SVC_BC_EXT_STATIC_IND_HDLR(QBI_QMI_SVC_UIM, 
                                QMI_UIM_STATUS_CHANGE_IND_V01,
                                QBI_SVC_BC_EXT_MBIM_CID_MS_SLOT_INFO_STATUS,
                                qbi_svc_bc_ext_slot_info_uim32_ind_cb),
};

/*! @brief CID handler dispatch table
    @details Order must match qbi_svc_bc_ext_cid_e. Entries are
    {query_func, min_query_infobuf_len, set_func, min_set_infobuf_len}
*/
static const qbi_svc_cmd_hdlr_tbl_entry_s qbi_svc_bc_ext_cmd_hdlr_tbl[] = {
  /* MBIM_CID_MS_PROVISIONED_CONTEXT_V2 */
  {qbi_svc_bc_ext_provisioned_context_v2_q_req, 0,
   qbi_svc_bc_ext_provisioned_context_v2_s_req,
   sizeof(qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s)},
  /* MBIM_CID_MS_NETWORK_BLACKLIST */
  { NULL, 0, NULL, 0},
  /* MBIM_CID_MS_LTE_ATTACH_CONFIG */
  {qbi_svc_bc_ext_lte_attach_config_q_req, 0, 
  qbi_svc_bc_ext_lte_attach_config_s_req, 
  sizeof(qbi_svc_bc_ext_lte_attach_config_s_req_s)},
  /* MBIM_CID_MS_LTE_ATTACH_STATUS */
  { qbi_svc_bc_ext_lte_attach_status_q_req, 0, NULL, 0 },
  /* MBIM_CID_MS_SYS_CAPS */
  { qbi_svc_bc_ext_sys_caps_info_q_req, 0, NULL, 0 },
  /* MBIM_CID_MS_DEVICE_CAPS_V2 */
  { qbi_svc_bc_ext_device_caps_v2_q_req, 0, NULL, 0 },
  /* MBIM_CID_MS_DEVICE_SLOT_MAPPING */
  { qbi_svc_bc_ext_slot_mapping_q_req, 0, 
  qbi_svc_bc_ext_slot_mapping_s_req, 
  sizeof (qbi_svc_bc_ext_slot_mapping_info_s) },
  /* MBIM_CID_MS_SLOT_INFO_STATUS */
  { qbi_svc_bc_ext_slot_info_status_q_req, 0, NULL, 0 },
  /* MBIM_CID_PCO */
  { NULL, 0, NULL, 0},
  /* MBIM_CID_MS_DEVICE_RESET */
  { NULL, 0, qbi_svc_bc_ext_device_reset_s_req, 0 }
};

/*! @addtogroup MBIM_CID_MS_SLOT_INFO_STATUS
    @{ */

static uint32_t active_config_id_len;
static uint8_t active_config_id[PDC_CONFIG_ID_SIZE_MAX_V01];

/*! @} */

/*=============================================================================

Private Function Definitions

=============================================================================*/

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_cache_get
===========================================================================*/
/*!
    @brief Returns a pointer to the Basic Connectivity Extension device
    service's cache

    @details

    @param ctx

    @return qbi_svc_bc_cache_s* Pointer to cache, or NULL on error
*/
/*=========================================================================*/
static qbi_svc_bc_ext_cache_s *qbi_svc_bc_ext_cache_get
(
  qbi_ctx_s *ctx,
  const uint32 cache_index
)
{
  qbi_svc_bc_ext_cache_s *cache = NULL;
/*-------------------------------------------------------------------------*/
  cache = (qbi_svc_bc_ext_cache_s *)qbi_svc_cache_get(ctx, QBI_SVC_ID_BC_EXT);
  if (cache == NULL)
  {
    cache = qbi_svc_cache_alloc(ctx, QBI_SVC_ID_BC_EXT, 
      sizeof(qbi_svc_bc_ext_cache_s) * QMI_WDS_PROFILE_LIST_MAX_V01);

    if (cache == NULL)
    {
      QBI_LOG_E_0("Couldn't allocate cache!");
      return NULL;
    }

    if (qbi_nv_store_cfg_item_read(
        ctx, QBI_NV_STORE_CFG_ITEM_PROVISION_CONTEXT_PROFILE_DATA, cache,
        sizeof(qbi_svc_bc_ext_cache_s) * QMI_WDS_PROFILE_LIST_MAX_V01))
    {
        QBI_LOG_D_0("Read profile data");
    }
  }

  return (qbi_svc_bc_ext_cache_s *)((uint8*)cache + 
    sizeof(qbi_svc_bc_ext_cache_s) * (cache_index));
} /* qbi_svc_bc_cache_get() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_update_nv_store
===========================================================================*/
/*!
    @brief Returns a pointer to the Basic Connectivity Extension device
    service's cache

    @details

    @param ctx

    @return qbi_svc_bc_cache_s* Pointer to cache, or NULL on error
*/
/*=========================================================================*/
static void qbi_svc_bc_ext_update_nv_store
(
  qbi_ctx_s *ctx
)
{
  qbi_svc_bc_ext_cache_s *cache = NULL;
/*-------------------------------------------------------------------------*/
  cache = (qbi_svc_bc_ext_cache_s *)qbi_svc_cache_get(ctx, QBI_SVC_ID_BC_EXT);
  QBI_CHECK_NULL_PTR_RET(cache);

  if (!qbi_nv_store_cfg_item_write(
    ctx, QBI_NV_STORE_CFG_ITEM_PROVISION_CONTEXT_PROFILE_DATA,
    cache, sizeof(qbi_svc_bc_ext_cache_s) * QMI_WDS_PROFILE_LIST_MAX_V01))
  {
    QBI_LOG_E_0("Couldn't save profile data to NV!");
  }
} /* qbi_svc_bc_ext_update_nv_store() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_update_version
===========================================================================*/
/*!
    @brief Update QBI version if required

    @details
    Checks if QBI version is updated , if not creates version file

    @param txn

    @return void
*/
/*=========================================================================*/
static void qbi_svc_bc_ext_update_version
(
  qbi_txn_s *txn
)
{
  qbi_svc_bc_qbi_version_s qbi_version = { 0 };
  uint32 status;
/*-------------------------------------------------------------------------*/
  if (!qbi_nv_store_cfg_item_read(
    txn->ctx, QBI_NV_STORE_CFG_ITEM_QBI_VERSION,
    &qbi_version, sizeof(qbi_svc_bc_qbi_version_s)))
  {
    QBI_LOG_D_0("Unable to read build version ! Creating it");

    qbi_version.version = QBI_VERSION;
    if (qbi_nv_store_cfg_item_write(
      txn->ctx, QBI_NV_STORE_CFG_ITEM_QBI_VERSION,
      &qbi_version, sizeof(qbi_svc_bc_qbi_version_s)))
    {
      QBI_LOG_D_0("Success : Version Written !!");
    }
    else
    {
      QBI_LOG_D_0("Failed : Version Written !!");
    }

    //Avoid compatibility issues with older QBI versions deleting context cache
    status = qbi_nv_store_cfg_item_delete(txn->ctx,
      QBI_NV_STORE_CFG_ITEM_PROVISION_CONTEXT_PROFILE_DATA);
    if (status == 0)
    {
      QBI_LOG_D_0("context_profile_data deleted successfully");
    }
  }
  else
  {
      QBI_LOG_D_1("Current QBI version is %d",qbi_version.version);
  }
}/* qbi_svc_bc_ext_update_version() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_open
===========================================================================*/
/*!
    @brief Runtime intiailization of the Basic Connectivity Extension service

    @details
    This is invoked per-context when the device receives a MBIM_OPEN_MSG.
    It is in charge of performing all runtime initialization so that the
    service can be operational.

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_open
(
  qbi_txn_s *txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);

  QBI_LOG_I_0("Processing Basic Extension Connectivity device service open...");
  if (!qbi_qmi_alloc_svc_handle(
    txn->ctx, QBI_SVC_BC_EXT_SESSION_ID_TO_WDS_SVC_ID_IPV4(0)) ||
    !qbi_qmi_alloc_svc_handle(
      txn->ctx, QBI_SVC_BC_EXT_SESSION_ID_TO_WDS_SVC_ID_IPV6(0)) ||
      !qbi_qmi_alloc_svc_handle(txn->ctx, QBI_QMI_SVC_DSD) ||
      !qbi_qmi_alloc_svc_handle(txn->ctx, QBI_QMI_SVC_PDC) ||
      !qbi_qmi_alloc_svc_handle(txn->ctx, QBI_QMI_SVC_UIM))
  {
    QBI_LOG_E_0("Failure allocating QMI client service handle");
  }
  else if (!qbi_svc_ind_reg_static(
    txn->ctx, QBI_SVC_ID_BC_EXT, qbi_svc_bc_ext_static_ind_hdlrs,
    ARR_SIZE(qbi_svc_bc_ext_static_ind_hdlrs)))
  {
    QBI_LOG_E_0("Couldn't register QMI indication handlers");
  }
  else
  {
    //Create qbi version file if not present and blow away the cache
    //to avoid compatibility issues with older QBI versions
    qbi_svc_bc_ext_update_version(txn);
    action = qbi_svc_bc_ext_provision_card(txn);
  }

  return action;
} /* qbi_svc_bc_ext_open() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provision_card
===========================================================================*/
/*!
    @brief Configures QMI indications

    @details
    Other QMI indications will be registered on-demand depending on CID filter
    status (see qbi_svc_bc_qmi_reg_tbl).

    @param txn

    @return boolean TRUE on success, FALSE otherwise
*/
/*=========================================================================*/
qbi_svc_action_e qbi_svc_bc_ext_provision_card
(
  qbi_txn_s *txn
)
{
  uim_get_card_status_req_msg_v01 *qmi_req = NULL;
  qbi_svc_action_e  action = QBI_SVC_ACTION_ABORT;
  qbi_svc_bc_ext_exec_slot_config_s exec_slot_cfg = {0};
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);

  if (!qbi_nv_store_cfg_item_read(
    txn->ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_SLOT_CONFIG,
    &exec_slot_cfg, sizeof(qbi_svc_bc_ext_exec_slot_config_s)))
  {
    QBI_LOG_E_0("Missing executor slot mapping info");
  }
  else
  {
     txn->info = QBI_MEM_MALLOC_CLEAR(
                 sizeof(qbi_svc_bc_ext_slot_map_cmd_info_s));
     QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);
     

     ((qbi_svc_bc_ext_slot_map_cmd_info_s *)txn->info)->req.slot = 
       exec_slot_cfg.exec0_slot;

     QBI_LOG_D_1("Provisioning on slot %d", exec_slot_cfg.exec0_slot);

     qmi_req = (uim_get_card_status_req_msg_v01 *)
       qbi_qmi_txn_alloc_ret_req_buf(
         txn, QBI_QMI_SVC_UIM, QMI_UIM_GET_CARD_STATUS_REQ_V01,
         qbi_svc_bc_ext_open_uim2f_rsp_cb);
     QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

     action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }

  return action;
} /* qbi_svc_bc_ext_provision_card() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_open_uim2f_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_UIM_GET_CARD_STATUS_RESP for
            MBIM_CID_MS_DEVICE_SLOT_MAPPING query

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_open_uim2f_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  uim_get_card_status_resp_msg_v01            *qmi_rsp = NULL;
  qbi_svc_action_e                             action = QBI_SVC_ACTION_ABORT;
  qbi_svc_bc_ext_exec_slot_config_s            exec_slot_cfg;
  qbi_svc_bc_ext_slot_map_cmd_info_s          *info;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (uim_get_card_status_resp_msg_v01 *)qmi_txn->rsp.data;
  info = (qbi_svc_bc_ext_slot_map_cmd_info_s *)qmi_txn->parent->info;
  
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
  }
  else
  {
    action = qbi_svc_bc_ext_build_uim38_req(qmi_txn);
  }

  /* QBI_SVC_ACTION_SEND_QMI_REQ action implies provisioning is required
     QBI_SVC_ACTION_SEND_QMI_RSP or QBI_SVC_ACTION_ABORT action implies
     provisioning is done by modem, hence perform indication registration
     and exit */
     
  if (action != QBI_SVC_ACTION_SEND_QMI_REQ)
  {
    if (action == QBI_SVC_ACTION_ABORT)
    {
      QBI_LOG_E_0("Card provisioning session failed. Continue service init");
    }


    /* Provisioning at first boot attempted but not required. Hence updating NV store
       and registering for indications.
       Note - This write operation is not redudant. It is needed for first provisioning
       attempt when provisioning is being established. Hence it should not be removed */
       
    exec_slot_cfg.exec0_slot = info->req.slot;
    exec_slot_cfg.exec0_prov_complete = TRUE;
    
    if (!qbi_nv_store_cfg_item_write(
      qmi_txn->ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_SLOT_CONFIG,
      &exec_slot_cfg, sizeof(qbi_svc_bc_ext_exec_slot_config_s)))
    {
      QBI_LOG_E_0("Couldn't save executor_slot_config NV!");
    }

    action = qbi_svc_bc_ext_open_configure_qmi_inds(qmi_txn->parent);
  }

  return action;
} /* qbi_svc_bc_ext_open_uim2f_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_build_uim38_req
===========================================================================*/
/*!
    @brief  Allocates and populates a QMI_UIM_CHANGE_PROVISIONING_SESSION_REQ
            request

    @details

    @param txn
    @param profile_type

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_build_uim38_req
(
  qbi_qmi_txn_s *qmi_txn
)
{
  uim_get_card_status_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_slot_map_cmd_info_s *info = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  qbi_svc_bc_ext_exec_slot_config_s exec_slot_cfg;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (uim_get_card_status_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->card_status.card_info_len > 1)
  {
    info = (qbi_svc_bc_ext_slot_map_cmd_info_s *)qmi_txn->parent->info;

    qbi_svc_bc_ext_populate_app_slot_info(qmi_txn);

    if (info->req.info_gw.act_need || info->req.info_1x.act_need)
    {
      exec_slot_cfg.exec0_prov_complete = FALSE;
      exec_slot_cfg.exec0_slot = info->req.slot;

      if (!qbi_nv_store_cfg_item_write(
        qmi_txn->ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_SLOT_CONFIG,
        &exec_slot_cfg, sizeof(qbi_svc_bc_ext_exec_slot_config_s)))
      {
        QBI_LOG_E_0("Couldn't save exectuor_slot_config to NV!");
      }
      else
      {
        action = qbi_svc_bc_ext_open_act_req(qmi_txn);
        QBI_LOG_D_0("Slot Activation in progress");
      }
    }
  }
  else
  {
    QBI_LOG_E_0("There is only 1 SIM. Modem setting will take care of prov");
    action = QBI_SVC_ACTION_SEND_RSP;
  }

  return action;
} /* qbi_svc_bc_ext_build_uim38_req() */



/*===========================================================================
FUNCTION: qbi_svc_bc_ext_open_act_req
===========================================================================*/
/*!
    @brief Handles a QMI_UIM_GET_CARD_STATUS_RESP for
            MBIM_CID_MS_DEVICE_SLOT_MAPPING query

    @details This function handles deactivation of requested 1x_pri slot 

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_open_act_req
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_bc_ext_slot_map_cmd_info_s * info;
  qbi_svc_action_e  action = QBI_SVC_ACTION_ABORT;
  uim_change_provisioning_session_req_msg_v01 *qmi_req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);

  info = (qbi_svc_bc_ext_slot_map_cmd_info_s *)qmi_txn->parent->info;

  /* GW: Activating slot */
  if (info->req.info_gw.act_need)
  {
    qmi_req = (uim_change_provisioning_session_req_msg_v01 *) 
      qbi_qmi_txn_alloc_ret_req_buf(qmi_txn->parent, 
      QBI_QMI_SVC_UIM, QMI_UIM_CHANGE_PROVISIONING_SESSION_REQ_V01, 
      (info->req.info_1x.app_present == TRUE)?
        qbi_svc_bc_ext_open_act_rsp_cb : qbi_svc_bc_ext_open_uim38_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

    qmi_req->session_change.session_type = UIM_SESSION_TYPE_PRIMARY_GW_V01;
    qmi_req->session_change.activate = QBI_SVC_BC_EXT_SLOT_ACTIVATE;
    qmi_req->application_information_valid = TRUE;

    if(info->req.slot == QBI_SVC_BC_EXT_SLOT_1)
      qmi_req->application_information.slot = UIM_SLOT_1_V01;
    else if(info->req.slot == QBI_SVC_BC_EXT_SLOT_2)
      qmi_req->application_information.slot = UIM_SLOT_2_V01;

    qmi_req->application_information.aid_len = info->req.info_gw.aid_length;
    QBI_MEMSCPY(qmi_req->application_information.aid, 
      sizeof(qmi_req->application_information.aid), 
    info->req.info_gw.aid, info->req.info_gw.aid_length);
    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }
  else if(info->req.info_1x.act_need)
  {
    /* 1x : Activating requested slot */
    qmi_req = (uim_change_provisioning_session_req_msg_v01 *) 
      qbi_qmi_txn_alloc_ret_req_buf(qmi_txn->parent, QBI_QMI_SVC_UIM, 
      QMI_UIM_CHANGE_PROVISIONING_SESSION_REQ_V01, 
      qbi_svc_bc_ext_open_uim38_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

    qmi_req->session_change.session_type = UIM_SESSION_TYPE_PRIMARY_1X_V01;
    qmi_req->session_change.activate = QBI_SVC_BC_EXT_SLOT_ACTIVATE;
    qmi_req->application_information_valid = TRUE;

    if(info->req.slot == QBI_SVC_BC_EXT_SLOT_1)
      qmi_req->application_information.slot = UIM_SLOT_1_V01;
    else if(info->req.slot == QBI_SVC_BC_EXT_SLOT_2)
      qmi_req->application_information.slot = UIM_SLOT_2_V01;

    qmi_req->application_information.aid_len = info->req.info_1x.aid_length;
    QBI_MEMSCPY(qmi_req->application_information.aid, 
      sizeof(qmi_req->application_information.aid), 
      info->req.info_1x.aid, info->req.info_1x.aid_length);
    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }

  return action;
}

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_open_act_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_UIM_GET_CARD_STATUS_RESP for
            MBIM_CID_MS_DEVICE_SLOT_MAPPING query

    @details This function handles activation of requested 1x_pri slot 

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_open_act_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  uim_change_provisioning_session_req_msg_v01 *qmi_change_prov_req = NULL;
  qbi_svc_bc_ext_slot_map_cmd_info_s * info;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);

  info = (qbi_svc_bc_ext_slot_map_cmd_info_s *)qmi_txn->parent->info;

  qmi_change_prov_req = (uim_change_provisioning_session_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(qmi_txn->parent, 
    QBI_QMI_SVC_UIM, QMI_UIM_CHANGE_PROVISIONING_SESSION_REQ_V01,
    qbi_svc_bc_ext_open_uim38_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_change_prov_req);

  qmi_change_prov_req->session_change.session_type = UIM_SESSION_TYPE_PRIMARY_1X_V01;
  qmi_change_prov_req->session_change.activate = QBI_SVC_BC_EXT_SLOT_ACTIVATE;
  qmi_change_prov_req->application_information_valid = TRUE;

  if(info->req.slot == QBI_SVC_BC_EXT_SLOT_1)
    qmi_change_prov_req->application_information.slot = UIM_SLOT_1_V01;
  else if(info->req.slot == QBI_SVC_BC_EXT_SLOT_2)
    qmi_change_prov_req->application_information.slot = UIM_SLOT_2_V01;

  qmi_change_prov_req->application_information.aid_len = info->req.info_1x.aid_length;
  QBI_MEMSCPY(qmi_change_prov_req->application_information.aid, 
    sizeof(qmi_change_prov_req->application_information.aid), 
    info->req.info_1x.aid, info->req.info_1x.aid_length);

  return QBI_SVC_ACTION_SEND_QMI_REQ;
} /* qbi_svc_bc_ext_open_act_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_open_uim38_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_UIM_GET_CARD_STATUS_RESP for
            MBIM_CID_MS_DEVICE_SLOT_MAPPING query

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_open_uim38_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  uim_get_card_status_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_exec_slot_config_s exec_slot_cfg;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (uim_get_card_status_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
  }

  /* Provisioning at first boot completed successfully. Hence updating NV store
     and registering for indications 
     Note - This write operation is not redudant. It is needed for first provisioning
     attempt when provisioning is being established. Hence it should not be removed */

  exec_slot_cfg.exec0_slot = 
    ((qbi_svc_bc_ext_slot_map_cmd_info_s *)(qmi_txn->parent->info))->req.slot;
  exec_slot_cfg.exec0_prov_complete = TRUE;

  if (!qbi_nv_store_cfg_item_write(
    qmi_txn->ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_SLOT_CONFIG,
    &exec_slot_cfg, sizeof(qbi_svc_bc_ext_exec_slot_config_s)))
  {
    QBI_LOG_E_0("Couldn't save exectuor_slot_config to NV!");
  }

  return qbi_svc_bc_ext_open_configure_qmi_inds(qmi_txn->parent);
} /* qbi_svc_bc_ext_open_uim38_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_open_configure_qmi_inds
===========================================================================*/
/*!
    @brief Configures QMI indications

    @details
    Other QMI indications will be registered on-demand depending on CID filter
    status (see qbi_svc_bc_qmi_reg_tbl).

    @param txn

    @return boolean TRUE on success, FALSE otherwise
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_open_configure_qmi_inds
(
  qbi_txn_s *txn
)
{
  wds_indication_register_req_msg_v01 *qmi_req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);

  qmi_req = (wds_indication_register_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(
      txn, QBI_QMI_SVC_WDS, QMI_WDS_INDICATION_REGISTER_REQ_V01,
      qbi_svc_bc_ext_provisioned_context_v2_qmi_ind_reg_wdsa8_req);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  qmi_req->report_lte_attach_pdn_list_change_valid = TRUE;
  qmi_req->report_lte_attach_pdn_list_change = TRUE;

  qmi_req->report_profile_changed_events_valid = TRUE;
  qmi_req->report_profile_changed_events = TRUE;

  return QBI_SVC_ACTION_SEND_QMI_REQ;
} /* qbi_svc_bc_ext_open_configure_qmi_inds() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_open_configure_qmi_inds_uim2e_cb
===========================================================================*/
/*!
    @brief Handles QMI_UIM_EVENT_REG_RESP

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_open_configure_qmi_inds_uim2e_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  uim_event_reg_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  QBI_LOG_D_0("qbi_svc_bc_ext_open_configure_qmi_inds_uim2e_cb");
  qmi_rsp = (uim_event_reg_resp_msg_v01 *)qmi_txn->rsp.data;

  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    if (qmi_rsp->resp.error != QMI_ERR_NO_EFFECT_V01)
    {
       QBI_LOG_E_1("Error registering for UIM indications!!! Error code %d",
         qmi_rsp->resp.error);
    }
    else
    {
      QBI_LOG_D_0("UIM indication registration already complete");
    }

    action = QBI_SVC_ACTION_SEND_RSP;
  }
  else if (qmi_rsp->event_mask_valid &&
        !(qmi_rsp->event_mask & (1 << QMI_UIM_EVENT_CARD_STATUS_BIT_V01)))
  {
      QBI_LOG_E_1("QMI event registration failed. Returned event mask 0x%08x",
        qmi_rsp->event_mask);

      action = QBI_SVC_ACTION_ABORT;
  }
  else
  {
      action = QBI_SVC_ACTION_SEND_RSP;
  }

  return action;
} /* qbi_svc_bc_ext_open_configure_qmi_inds_uim2e_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_ip_type_to_pdp_type
===========================================================================*/
/*!
    @brief Translates MBIM IP type to QMI PDP type

  @details

  @param ip_type

  @return wds_pdp_type_enum_v01
*/
/*=========================================================================*/
static wds_pdp_type_enum_v01 qbi_svc_bc_ext_ip_type_to_pdp_type
(
  uint32 ip_type
)
{
/*-------------------------------------------------------------------------*/
  switch (ip_type)
  {
  case QBI_SVC_BC_IP_TYPE_IPV4:
    return WDS_PDP_TYPE_PDP_IPV4_V01;
  case QBI_SVC_BC_IP_TYPE_IPV6:
    return WDS_PDP_TYPE_PDP_IPV6_V01;
  case QBI_SVC_BC_IP_TYPE_IPV4V6:
  case QBI_SVC_BC_IP_TYPE_DEFAULT:
  default:
    return WDS_PDP_TYPE_PDP_IPV4V6_V01;
  }
} /* qbi_svc_bc_ext_ip_type_to_pdp_type */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_match_pdp_type
===========================================================================*/
/*!
    @brief Compares IP Type of requested profile with that of modem

    @details

    @param txn
    @param qmi_rsp

    @return TRUE/FALSE
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_match_pdp_type
(
  qbi_svc_bc_ext_lte_attach_context_s *context,
  wds_get_profile_settings_resp_msg_v01 *qmi_rsp
)
{
  boolean success = FALSE;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_rsp);
  QBI_CHECK_NULL_PTR_RET_FALSE(context);

  if (qmi_rsp->pdp_type_valid)
  {
    switch (context->ip_type)
    {
    case QBI_SVC_BC_IP_TYPE_IPV4:
      success = (qmi_rsp->pdp_type == WDS_PDP_TYPE_PDP_IPV4_V01 ||
        qmi_rsp->pdp_type == WDS_PDP_TYPE_PDP_IPV4V6_V01);
      break;
    case QBI_SVC_BC_IP_TYPE_IPV6:
      success = (qmi_rsp->pdp_type == WDS_PDP_TYPE_PDP_IPV6_V01 ||
        qmi_rsp->pdp_type == WDS_PDP_TYPE_PDP_IPV4V6_V01);
      break;
    case QBI_SVC_BC_IP_TYPE_IPV4V6:
    case QBI_SVC_BC_IP_TYPE_DEFAULT:
      success = (qmi_rsp->pdp_type == WDS_PDP_TYPE_PDP_IPV4_V01 ||
        qmi_rsp->pdp_type == WDS_PDP_TYPE_PDP_IPV6_V01 ||
        qmi_rsp->pdp_type == WDS_PDP_TYPE_PDP_IPV4V6_V01);
      break;
    }
  }

  QBI_LOG_D_3("LTEAttachConfig::S: Matching pdp_type(%d) <==> ip_type(%d), "
    "status %d", qmi_rsp->pdp_type, context->ip_type, success);
  return success;
}/* qbi_svc_bc_ext_match_pdp_type() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_match_string_field
===========================================================================*/
/*!
    @brief Compares modem and apps strings

    @details

    @param qmi_str_valid
    @param qmi_str
    @param field
    @param field_size

    @return boolean
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_match_string_field
(
  const uint8   qmi_str_valid,
  const char    *qmi_str,
  const uint8   *field,
  const uint32  field_size
)
{
  char    *str = NULL;
  uint32  str_size = 0;
  uint32  qmi_str_size = 0;
  boolean match_found = FALSE;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_str);

  if (qmi_str_valid)
  {
    str = QBI_MEM_MALLOC_CLEAR(field_size + 2);
    QBI_CHECK_NULL_PTR_RET_FALSE(str);

    qbi_util_utf16_to_ascii(field, field_size, str, field_size);

    qmi_str_size = QBI_STRLEN(qmi_str);
    str_size = QBI_STRLEN(str);

    QBI_LOG_STR_2("LTEAttachConfig::S: Matching %s <==> %s", qmi_str, str);
    // If unequal size return FALSE immediately.
    if (str_size != qmi_str_size)
    {
      QBI_LOG_I_2("LTEAttachConfig::S: String length did not match. %d <==> %d", 
        qmi_str_size, str_size);
      match_found = FALSE;
    }
    // If size of both string equals, return TRUE immediately.
    else if (!str_size && !qmi_str_size)
    {
      QBI_LOG_I_0("LTEAttachConfig::S: Matched NULL string");
      match_found = TRUE;
    }
    else
    {
      match_found = !QBI_STRNCMP(str, qmi_str, str_size);
    }

    QBI_MEM_FREE(str);
    str = NULL;
  }

  return match_found;
}/* qbi_svc_bc_ext_match_string_field() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_match_string
===========================================================================*/
/*!
    @brief Compares modem and apps strings

    @details

    @param qmi_str_valid
    @param qmi_str
    @param field
    @param field_size

    @return boolean
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_match_string
(
  const uint8   qmi_str_valid,
  const char    *qmi_str,
  const char    *str
)
{
  uint32  qmi_str_size = 0;
  uint32  str_size = 0;
  boolean match_found = FALSE;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_str);

  if (qmi_str_valid)
  {
    qmi_str_size = QBI_STRLEN(qmi_str);
    str_size = QBI_STRLEN(str);

    QBI_LOG_STR_2("LTEAttachStatus::Q: Matching %s <==> %s", qmi_str, str);
    // If unequal size return FALSE immediately.
    if (str_size != qmi_str_size)
    {
      QBI_LOG_I_2("LTEAttachStatus::Q: String length did not match. %d <==> %d",
        qmi_str_size, str_size);
      match_found = FALSE;
    }
    // If size of both string equals, return TRUE immediately.
    else if (!str_size && !qmi_str_size)
    {
      QBI_LOG_I_0("LTEAttachStatus::Q: Matched NULL string");
      match_found = TRUE;
    }
    else
    {
      match_found = !QBI_STRNCMP(str, qmi_str, str_size);
			QBI_LOG_I_1("LTEAttachStatus::Q: Match status %d", match_found);
		}
  }

  return match_found;
}/* qbi_svc_bc_ext_match_string() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_roam_type_to_roam_flag
===========================================================================*/
/*!
    @brief Maps MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL locally

    @details

    @param qmi_str_valid
    @param qmi_str
    @param field
    @param field_size

    @return boolean
*/
/*=========================================================================*/
static uint32 qbi_svc_bc_ext_roam_type_to_roam_flag
(
  const uint32  roam_type
)
{
/*-------------------------------------------------------------------------*/
  switch (roam_type)
  {
    case QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_HOME:
             return QBI_SVC_BC_EXT_ROAMING_FLAG_HOME;
    case QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_PARTNER:
             return QBI_SVC_BC_EXT_ROAMING_FLAG_PARTNER;
    case QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_NON_PARTNER:
             return QBI_SVC_BC_EXT_ROAMING_FLAG_NON_PARTNER;
  }

  return QBI_SVC_BC_EXT_ROAMING_FLAG_HOME;
}/* qbi_svc_bc_ext_roam_type_to_roam_flag() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_roam_flag_to_roam_type
===========================================================================*/
/*!
    @brief Compares modem and apps strings

    @details

    @param qmi_str_valid
    @param qmi_str
    @param field
    @param field_size

    @return boolean
*/
/*=========================================================================*/
static uint32 qbi_svc_bc_ext_roam_flag_to_roam_type
(
  const uint32 roam_flag,
	const uint32 roam_type
)
{
/*-------------------------------------------------------------------------*/
  switch (roam_type)
  {
  case QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_HOME:
    if (roam_flag & QBI_SVC_BC_EXT_ROAMING_FLAG_HOME)
    {
      return QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_HOME;
    }
    break;
  case QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_PARTNER:
    if (roam_flag & QBI_SVC_BC_EXT_ROAMING_FLAG_PARTNER)
    {
      return QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_PARTNER;
    }
    break;
  case QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_NON_PARTNER:
    if (roam_flag & QBI_SVC_BC_EXT_ROAMING_FLAG_NON_PARTNER)
    {
      return QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_NON_PARTNER;
    }
    break;
  }

  return QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_HOME;
}/* qbi_svc_bc_ext_roam_flag_to_roam_type() */

/*! @addtogroup QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2
    @{ */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_get_user_defined_profile_from_cache
===========================================================================*/
/*!
    @brief Returns a pointer to the Basic Connectivity Extension device
    service's cache

    @details

    @param ctx

    @return Profile index
*/
/*=========================================================================*/
static uint32 qbi_svc_bc_ext_get_user_defined_profile_from_cache
(
  qbi_ctx_s *ctx
)
{
  qbi_svc_bc_ext_cache_s *cache = NULL;
  qbi_svc_bc_spdp_cache_s cache_spdp = { 0 };
  uint32 cache_index = 0;
/*-------------------------------------------------------------------------*/
  while (QMI_WDS_PROFILE_LIST_MAX_V01 > cache_index)
  {
    cache = (qbi_svc_bc_ext_cache_s *)
      qbi_svc_bc_ext_cache_get(ctx, cache_index);
    if (cache == NULL)
    {
      QBI_LOG_E_0("Cache is empty !!");
      return FALSE;
    }

    qbi_svc_bc_spdp_read_nv_store(ctx, &cache_spdp);

    if (cache != NULL && 
       (cache->prov_active == TRUE || cache->lte_active == TRUE) && 
        cache->context_flag == QBI_SVC_BC_EXT_CONTEXT_FLAG_USER_DEFINED &&
       (cache->lte_active && QBI_SVC_BC_SPDP_OPERATOR_NONE == cache_spdp.spdp_support_flag) &&
       (cache->source == QBI_SVC_MBIM_MS_CONTEXT_SOURCE_USER || 
        cache->source == QBI_SVC_MBIM_MS_CONTEXT_SOURCE_OPERATOR ||
        cache->source == QBI_SVC_MBIM_MS_CONTEXT_SOURCE_DEVICE ||
        cache->source == QBI_SVC_MBIM_MS_CONTEXT_SOURCE_ADMIN))
    {
      QBI_LOG_I_1("Found user defined profile at index %d", cache_index);
      break;
    }
    cache_index++;
  }

  return cache_index;
} /* qbi_svc_bc_cache_get() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_clear_cache_profiles
===========================================================================*/
/*!
    @brief Handles a QMI_PDC_REFRESH_IND_V01

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_clear_cache_profiles
(
  qbi_txn_s *txn,
  uint32    slot_id
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_delete_profile_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_cache_s *cache = NULL;
  uint32 *info = NULL;
  uint32 profile_index = 0;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);

  txn->info = QBI_MEM_MALLOC_CLEAR(sizeof(uint32));
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);
  info = (uint32*)txn->info;

  // delete user defined profiles and reset cache
  profile_index = 
    qbi_svc_bc_ext_get_user_defined_profile_from_cache(txn->ctx);

  if (QBI_SVC_BC_EXT_INVALID_PROFILE_INDEX != profile_index)
  {
    qmi_req = (wds_delete_profile_req_msg_v01 *)
      qbi_qmi_txn_alloc_ret_req_buf(
        txn, QBI_QMI_SVC_WDS, QMI_WDS_DELETE_PROFILE_REQ_V01,
        qbi_svc_bc_ext_wds29_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

    qmi_req->profile.profile_index = profile_index;
    *info = profile_index;
    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }
  else
  {
    cache = (qbi_svc_bc_ext_cache_s *)
      qbi_svc_cache_get(txn->ctx, QBI_SVC_ID_BC_EXT);
    QBI_CHECK_NULL_PTR_RET_ABORT(cache);

    // clear cache
    QBI_MEMSET(cache, 0, 
      sizeof(qbi_svc_bc_ext_cache_s)  * QMI_WDS_PROFILE_LIST_MAX_V01);

    action = qbi_svc_bc_ext_provisioned_context_v2_q_req(txn);
  }

  QBI_LOG_D_1("ext_clear_cache_profiles %d",action);
  return action;
} /* qbi_svc_bc_ext_clear_cache_profiles() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_pdc2f_ind_cb
===========================================================================*/
/*!
    @brief Handles a QMI_PDC_REFRESH_IND_V01, looking for operating mode
    changes to trigger a QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2
    event

    @details

    @param ind

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_pdc2f_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  const pdc_refresh_ind_msg_v01 *qmi_ind = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->buf->data);

  qmi_ind = (const pdc_refresh_ind_msg_v01 *)ind->buf->data;

  switch (qmi_ind->refresh_event)
  {
  case PDC_EVENT_REFRESH_START_V01:
    break;
  case PDC_EVENT_REFRESH_COMPLETE_V01:
    if (qmi_ind->slot_id_valid)
    {
      // delete user defined profiles and reset cache
      QBI_LOG_E_0("PDC refresh completed. Clean up user defined profiles.");
      action = qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_wds29_req(ind->txn);
    }
    break;
  default:
    break;
  }

  QBI_LOG_D_1("Prov Contxt V2 : Received PDC 0x2f IND CB %d",action);
  return action;
} /* qbi_svc_bc_ext_pdc2f_ind_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_read_operator_nv
===========================================================================*/
/*!
    @brief Reads operator_config from NV

    @details

    @param ctx
    @param operator_config

    @return void
*/
/*=========================================================================*/
void qbi_svc_bc_ext_read_operator_nv
(
  qbi_ctx_s               *ctx,
  qbi_svc_bc_ext_operator_config_s *operator_cfg
)
{
/*-------------------------------------------------------------------------*/
  if (!qbi_nv_store_cfg_item_read(
    ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_OPERATOR_CONFIG, 
    operator_cfg, sizeof(qbi_svc_bc_ext_operator_config_s)))
  {
    QBI_LOG_E_0("Prov Contxt V2:: E:Couldn't read profile data from NV!");
  }
} /* qbi_svc_bc_ext_read_operator_nv() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_update_operator_nv
===========================================================================*/
/*!
    @brief Updates operator_config NV

    @details

    @param ctx
    @param operator_config

    @return void
*/
/*=========================================================================*/
void qbi_svc_bc_ext_update_operator_nv
(
  qbi_ctx_s               *ctx,
  qbi_svc_bc_ext_operator_config_s *operator_cfg
)
{
/*-------------------------------------------------------------------------*/
  if (!qbi_nv_store_cfg_item_write(
    ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_OPERATOR_CONFIG, 
    operator_cfg, sizeof(qbi_svc_bc_ext_operator_config_s)))
  {
    QBI_LOG_E_0("Prov Contxt V2:: E:Couldn't save profile data to NV!");
  }
} /* qbi_svc_bc_ext_update_operator_nv() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_compare_imsi_for_operator
===========================================================================*/
/*!
    @brief Compares I/P imsi with that of a particular operator

    @details

    @param imsi to be matched

    @return TRUE is match found else FALSE
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_compare_imsi_for_operator
(
  const char *imsi
)
{
  boolean match_found = FALSE;
/*-------------------------------------------------------------------------*/
  if (!QBI_STRNCMP(imsi, QBI_SVC_BC_EXT_IMSI_311_480,
      sizeof(char) * QBI_SVC_BC_EXT_IMSI_311_480_MAX_LEN))
  {
    match_found = TRUE;
  }
  else if (!QBI_STRNCMP(imsi, QBI_SVC_BC_EXT_IMSI_311_270,
      sizeof(char) * QBI_SVC_BC_EXT_IMSI_311_270_MAX_LEN))
  {
    match_found = TRUE;
  }
  else if (!QBI_STRNCMP(imsi, QBI_SVC_BC_EXT_IMSI_312_770,
      sizeof(char) * QBI_SVC_BC_EXT_IMSI_312_770_MAX_LEN))
  {
    match_found = TRUE;
  }

  return match_found;
} /* qbi_svc_bc_ext_compare_imsi_for_operator() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_prov_contxt_v2_update_operator_nv
===========================================================================*/
/*!
    @brief Updates class1/2 NV based on whether apn is enabled/disabled
           conditions

    @details

    @param ctx
    @param GET_PROFILE_SETTINGS QMI RSP
    @param operator_cfg NV
    @param class for which update is required

    @return void
*/
/*=========================================================================*/
static void qbi_svc_bc_ext_prov_contxt_v2_update_operator_nv
(
  qbi_ctx_s                              *ctx,
  wds_get_profile_settings_resp_msg_v01  *qmi_rsp,
  qbi_svc_bc_ext_operator_config_s       *operator_cfg
)
{
  qbi_svc_bc_spdp_cache_s cache = { 0 };
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(ctx);
  QBI_CHECK_NULL_PTR_RET_ABORT(operator_cfg);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_rsp);

  qbi_svc_bc_spdp_read_nv_store(ctx, &cache);
  if ((qmi_rsp->apn_disabled_flag_valid == TRUE) &&
       qbi_svc_bc_ext_compare_imsi_for_operator(cache.imsi))
  {
    if (qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class1(ctx,qmi_rsp->apn_name))
    {
      QBI_LOG_D_1("Prov Contxt V2 : Update Operator NV for class 1"
                  " with apn %d",qmi_rsp->apn_disabled_flag);
      if (qmi_rsp->apn_disabled_flag == FALSE)
      {
        operator_cfg->class1_disable = QBI_SVC_BC_EXT_OPERATOR_STATE_UNSET;
      }
      else
      {
        operator_cfg->class1_disable = QBI_SVC_BC_EXT_OPERATOR_STATE_SET;
      }
      qbi_svc_bc_ext_update_operator_nv(ctx,operator_cfg);
    }
    if(qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class2(qmi_rsp->apn_name))
    {
      QBI_LOG_D_1("Prov Contxt V2 : Update Operator NV for class 2"
                  " with apn %d",qmi_rsp->apn_disabled_flag);
      if (qmi_rsp->apn_disabled_flag == FALSE)
      {
        operator_cfg->class2_disable = QBI_SVC_BC_EXT_OPERATOR_STATE_UNSET;
      }
      else
      {
        operator_cfg->class2_disable = QBI_SVC_BC_EXT_OPERATOR_STATE_SET;
      }
      qbi_svc_bc_ext_update_operator_nv(ctx,operator_cfg);
    }
  }
}/*! qbi_svc_bc_ext_prov_contxt_v2_update_operator_nv */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_prov_contxt_v2_update_operator_nv_if_reqd
===========================================================================*/
/*!
    @brief Check the operator NV,if its NONE sets to to either SET or UNSET
           based on the current profile apn_disable_flag setting

    @details

    @param txn
    @param GET_PROFILE_SETTINGS QMI RSP
 
    @return void
*/
/*=========================================================================*/
static void qbi_svc_bc_ext_prov_contxt_v2_update_operator_nv_if_reqd
(
  qbi_txn_s                             *txn,
  wds_get_profile_settings_resp_msg_v01 *qmi_rsp,
  qbi_svc_bc_ext_operator_config_s      *operator_cfg
)
{
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_rsp);

  QBI_LOG_D_0("Prov Contxt V2 : Update Operator Status If Required.");
  if (operator_cfg->class1_disable == QBI_SVC_BC_EXT_OPERATOR_STATE_NONE ||
      operator_cfg->class2_disable == QBI_SVC_BC_EXT_OPERATOR_STATE_NONE)
  {
    qbi_svc_bc_ext_prov_contxt_v2_update_operator_nv(txn->ctx,
                                                     qmi_rsp,
                                                     operator_cfg);
  }
}/* qbi_svc_bc_ext_prov_contxt_v2_update_operator_nv_if_reqd */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_configure_nv_for_operator
===========================================================================*/
/*!
    @brief Configures NV for specific operator if required

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
qbi_svc_action_e qbi_svc_bc_ext_configure_nv_for_operator
(
  qbi_txn_s *txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  qbi_svc_bc_spdp_cache_s cache = { 0 };
  qbi_svc_bc_ext_operator_config_s operator_cfg = {0};
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);

  qbi_svc_bc_spdp_read_nv_store(txn->ctx, &cache);
  if (qbi_svc_bc_ext_compare_imsi_for_operator(cache.imsi))
  {
    qbi_svc_bc_ext_read_operator_nv(txn->ctx,&operator_cfg);
    if (operator_cfg.class1_disable == QBI_SVC_BC_EXT_OPERATOR_STATE_NONE ||
        operator_cfg.class2_disable == QBI_SVC_BC_EXT_OPERATOR_STATE_NONE)
    {
      QBI_LOG_D_0("Prov Contx V2 Sub ready: Operator Configuration Required.");
      if (txn->info)
      {
        QBI_MEM_FREE(txn->info);
        txn->info = NULL;
      }
      /* Performing profile updates as part of processing sub ready. 
         Hence ignoring profile update events. */
      cmd_in_progress_ignore_indication = TRUE;
      /* Leveraging the existing prov ctx factory restore processing 
         functions to perform the profile disable/enable updates */
      action = qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_prep_wds29_req(txn);
    }
    else
    {
      QBI_LOG_D_0("Prov Contx V2 Sub ready: Already Provisioned.");
      action = QBI_SVC_ACTION_SEND_RSP;
    }
  }
  else
  {
    QBI_LOG_D_0("Prov Contx V2 Sub ready: Customization Not Reqrd.");
    action = QBI_SVC_ACTION_SEND_RSP;
  }

  return action;
}/* qbi_svc_bc_ext_configure_nv_for_operator */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_wds29_rsp_cb
===========================================================================*/
/*!
    @brief Populates QMI profile settings for a MBIM_CID_PROVISIONED_CONTEXTS
    set request

    @details

    @param qmi_txn

    @return qbi_svc_action_e QBI_SVC_ACTION_SEND_QMI_REQ on success,
    QBI_SVC_ACTION_ABORT on failure
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_wds29_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  wds_delete_profile_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_cache_s * cache = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  uint32 *profile_index = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);

  qmi_rsp = (wds_delete_profile_resp_msg_v01 *)qmi_txn->rsp.data;
  if ((qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01) &&
    (qmi_rsp->resp.error == QMI_ERR_EXTENDED_INTERNAL_V01) &&
    (qmi_rsp->extended_error_code_valid && qmi_rsp->extended_error_code ==
      WDS_EEC_DS_PROFILE_REG_RESULT_ERR_INVAL_PROFILE_TYPE_V01))
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
    qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status(
      qmi_txn->parent, qmi_rsp->resp.error,
      qmi_rsp->extended_error_code_valid,
      qmi_rsp->extended_error_code);
  }

  profile_index = (uint32 *)qmi_txn->parent->info;
  cache = (qbi_svc_bc_ext_cache_s *)
    qbi_svc_bc_ext_cache_get(qmi_txn->ctx, *profile_index);
  QBI_CHECK_NULL_PTR_RET_ABORT(cache);

  cache->prov_active = FALSE;
  cache->context_flag = QBI_SVC_BC_EXT_CONTEXT_FLAG_MIN;
  cache->lte_active = FALSE;

  // TODO: slot ID hardcoding to be worked out
  action = qbi_svc_bc_ext_clear_cache_profiles(qmi_txn->parent, 0);

  return action; 
}/* qbi_svc_bc_ext_wds29_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_wds28_rsp_cb
===========================================================================*/
/*!
  @brief Handles a QMI_WDS_MODIFY_PROFILE_SETTINGS_RESP for
  MBIM_CID_PROVISIONED_CONTEXTS set request

  @details

  @param qmi_txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_wds28_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_modify_profile_settings_resp_msg_v01 *qmi_rsp = NULL;
  wds_modify_profile_settings_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->req.data);

  qmi_rsp = (wds_modify_profile_settings_resp_msg_v01 *)qmi_txn->rsp.data;
  qmi_req = (wds_modify_profile_settings_req_msg_v01 *)qmi_txn->req.data;

  QBI_LOG_I_0("Rsp Callback For PDC Modify Req");
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
    if (qmi_rsp->extended_error_code_valid)
    {
      QBI_LOG_E_1("Extended error code %d", qmi_rsp->extended_error_code);
    }
    if (QBI_SVC_BC_MBIM_CID_SUBSCRIBER_READY_STATUS == qmi_txn->parent->cid)
    {
      /* If modify profile settings QMI resp fails which causes transaction to ABORT
         we need to check if the D is subscriber ready.If so we need to send
         proper response to this CID so that SUBSCRIBER_READY_STATE functinality
         is not affected by this failure */
      QBI_LOG_D_0("Get Profile Setting Failed.Returning to Sub Ready.");
      cmd_in_progress_ignore_indication = FALSE;
      action = QBI_SVC_ACTION_SEND_RSP;
    }
    else
    {
      qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status(
        qmi_txn->parent, qmi_rsp->resp.error,
        qmi_rsp->extended_error_code_valid, qmi_rsp->extended_error_code);
    }
  }
  else
  {
     QBI_LOG_D_0("Modify Complete For PDC Going To Get Next Profile");
     action = qbi_svc_bc_ext_provisioned_contexts_v2_pdc_get_next_profile(
              qmi_txn->parent);
  }

  return action;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_wds28_rsp_cb */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_build_pdc_wds28_req
===========================================================================*/
/*!
    @brief Allocates and populates a QMI_WDS_MODIFY_PROFILE_SETTINGS request

    @details

    @param txn
    @param profile_type

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_build_pdc_wds28_req
(
  qbi_txn_s                *txn,
  wds_profile_type_enum_v01 profile_type,
  uint32 index
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_modify_profile_settings_req_msg_v01 *qmi_req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);

  QBI_LOG_D_0("Initiating Prov Context V2 PDC Profile Modify Req");
  qmi_req = (wds_modify_profile_settings_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(
      txn, QBI_QMI_SVC_WDS, QMI_WDS_MODIFY_PROFILE_SETTINGS_REQ_V01,
      qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_wds28_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  qmi_req->profile.profile_index = (uint8_t)index;
  qmi_req->profile.profile_type = profile_type;

  QBI_LOG_D_2("profile_type %d profile_index %d",
               qmi_req->profile.profile_type,qmi_req->profile.profile_index);
  
  qmi_req->apn_disabled_flag_valid = TRUE;
  qmi_req->apn_disabled_flag = TRUE;
  action = QBI_SVC_ACTION_SEND_QMI_REQ;

  return action;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_s_build_pdc_wds28_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_prov_ctx_v2_s_update_disable_flag
===========================================================================*/
/*!
  @brief Modify Operator Class APN profiles based on current NV setting

  @details

  @param qmi_txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_prov_ctx_v2_s_update_disable_flag
(
  qbi_qmi_txn_s                          *qmi_txn,
  wds_get_profile_settings_req_msg_v01   *qmi_req,
  wds_get_profile_settings_resp_msg_v01  *qmi_rsp
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  qbi_svc_bc_ext_operator_config_s operator_cfg = {0};
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_rsp);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);

  qbi_svc_bc_ext_read_operator_nv(qmi_txn->ctx,&operator_cfg);
  if ((QBI_SVC_BC_MBIM_CID_SUBSCRIBER_READY_STATUS == qmi_txn->parent->cid) &&
      (qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class1(qmi_txn->parent->ctx,qmi_rsp->apn_name) ||
       qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class2(qmi_rsp->apn_name)))
  {
    (void)qbi_svc_bc_ext_prov_contxt_v2_update_operator_nv_if_reqd(qmi_txn->parent,qmi_rsp,&operator_cfg);
    /* Need to read again as we might have changed the NV state */
    (void)qbi_svc_bc_ext_read_operator_nv(qmi_txn->ctx,&operator_cfg);
  }
  if ((operator_cfg.class1_disable == QBI_SVC_BC_EXT_OPERATOR_STATE_SET &&
      qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class1(qmi_txn->parent->ctx,qmi_rsp->apn_name) &&
     (qmi_rsp->apn_disabled_flag_valid == TRUE &&
      qmi_rsp->apn_disabled_flag == FALSE)) ||
     ((operator_cfg.class2_disable == QBI_SVC_BC_EXT_OPERATOR_STATE_SET) && 
      qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_operator(qmi_rsp->apn_name) &&
      (qmi_rsp->apn_disabled_flag_valid == TRUE &&
      qmi_rsp->apn_disabled_flag == FALSE))) 
  {
     QBI_LOG_D_0("Prov Context V2 : Sending Modify Req as part of PDC Seq");
     action = qbi_svc_bc_ext_provisioned_contexts_v2_s_build_pdc_wds28_req(
              qmi_txn->parent, qmi_req->profile.profile_type,
              qmi_req->profile.profile_index);
  }
  else
  {
    QBI_LOG_D_0("Prov Context V2 : Initiating get next profile");
    action = qbi_svc_bc_ext_provisioned_contexts_v2_pdc_get_next_profile(
               qmi_txn->parent);
  }

  return action;
}/* qbi_svc_bc_ext_prov_ctx_v2_s_update_disable_flag */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_pdc_wds2b_rsp_cb
===========================================================================*/
/*!
  @brief Handles a QMI_WDS_GET_PROFILE_SETTINGS_RESP for
  QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 Set

  @details

  @param qmi_txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_pdc_wds2b_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_get_profile_settings_req_msg_v01 *qmi_req = NULL;
  wds_get_profile_settings_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *info = NULL;
  qbi_svc_bc_ext_operator_config_s operator_cfg = {0};
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_req = (wds_get_profile_settings_req_msg_v01 *)qmi_txn->req.data;
  qmi_rsp = (wds_get_profile_settings_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
    qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status(
    qmi_txn->parent, qmi_rsp->resp.error,
    qmi_rsp->extended_error_code_valid, qmi_rsp->extended_error_code);
    if (QMI_ERR_EXTENDED_INTERNAL_V01 == qmi_rsp->resp.error && 
        qmi_rsp->extended_error_code_valid &&
        WDS_EEC_DS_PROFILE_REG_RESULT_ERR_INVAL_PROFILE_NUM_V01 == 
        qmi_rsp->extended_error_code)
    {
      QBI_LOG_D_0("Prov Context V2 Factory Op :: Bad profile. Continue to get next profile");
      info = (qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *)qmi_txn->parent->info;

      info->profiles_read++;
      action = qbi_svc_bc_ext_provisioned_contexts_v2_pdc_get_next_profile(qmi_txn->parent);
    }
    else if (QBI_SVC_BC_MBIM_CID_SUBSCRIBER_READY_STATUS == qmi_txn->parent->cid)
    {
      /* Leveraging existing function to enable/disable during sub ready processing.
         If get profile settings QMI resp fails which causes transaction to ABORT
         we need to check if the D is subscriber ready.If so we need to send
         proper response to this CID so that SUBSCRIBER_READY_STATE functinality
         is not affected by this failure */
       QBI_LOG_D_0("Get Profile Setting Failed! Returning to Sub Ready.");
       cmd_in_progress_ignore_indication = FALSE;
       action = QBI_SVC_ACTION_SEND_RSP;
    }
  }
  else
  {
    info = (qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *)
            qmi_txn->parent->info;

    info->profiles_read++;
    QBI_LOG_I_2("Received profile %d/%d", info->profiles_read,
                info->profile_list.num_of_profile);

    if (info->profiles_read < info->profile_list.num_of_profile)
    {
       if (qmi_rsp->apn_name_valid &&
           qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_operator(qmi_rsp->apn_name))
       {
         action = qbi_svc_bc_ext_prov_ctx_v2_s_update_disable_flag(qmi_txn,qmi_req,qmi_rsp);
       }
       else
       {
         action = qbi_svc_bc_ext_provisioned_contexts_v2_pdc_get_next_profile(
                  qmi_txn->parent);
       }
    }
    else
    {
      QBI_LOG_E_0("Profiles exhausted, Sending Response");

      if (qmi_txn->parent->qmi_txns_pending > 0)
      {
        QBI_LOG_D_0("Pending Transaction For Prov Cntxt Factory Set Req");
        action = QBI_SVC_ACTION_WAIT_ASYNC_RSP;
      }
      else
      {
        if (qmi_txn->parent->info != NULL)
        {
          QBI_MEM_FREE(qmi_txn->parent->info);
          qmi_txn->parent->info = NULL;
        }

        if (QBI_SVC_BC_EXT_MBIM_CID_MS_LTE_ATTACH_CONFIG == qmi_txn->parent->cid)
        {
          // Trigger detach
          QBI_LOG_D_0("Factory restore completed successfully. Trigger Detach.");
          cmd_in_progress_ignore_indication = FALSE;
          action = qbi_svc_bc_ext_lte_attach_config_q_req(qmi_txn->parent);
        }
        else if (QBI_SVC_BC_MBIM_CID_SUBSCRIBER_READY_STATUS == qmi_txn->parent->cid)
        {
           /* Leveraging existing function to enable/disable during sub ready processing */
           QBI_LOG_D_0("Operator Flag Validation Done.Returning to Subscriver Ready.");
           cmd_in_progress_ignore_indication = FALSE;
           action = QBI_SVC_ACTION_SEND_RSP;
        }
        else
        {
          action = qbi_svc_bc_ext_provisioned_context_v2_q_req(qmi_txn->parent);
        }
      }
    }
  }

  return action;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_pdc_wds2b_rsp_cb */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_pdc_get_next_profile
===========================================================================*/
/*!
  @brief Retrive next available configured profile.

  @details

  @param txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_pdc_get_next_profile
(
  qbi_txn_s *txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_get_profile_settings_req_msg_v01 *qmi_req_wds2b = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *info = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);

  info = (qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *)txn->info;
  if (info->profiles_read >= info->profile_list.num_of_profile)
  {
    cmd_in_progress_ignore_indication = FALSE;
    action = QBI_SVC_ACTION_SEND_RSP;
  }
  else
  {
    /* Issue a query to retrieve the profile details */
    qmi_req_wds2b = (wds_get_profile_settings_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(
        txn, QBI_QMI_SVC_WDS, QMI_WDS_GET_PROFILE_SETTINGS_REQ_V01,
        qbi_svc_bc_ext_provisioned_contexts_v2_pdc_wds2b_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req_wds2b);

    qmi_req_wds2b->profile.profile_type =
      info->profile_list.profile_type[info->profiles_read];
    qmi_req_wds2b->profile.profile_index =
      info->profile_list.profile_index[info->profiles_read];

    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }

  return action;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_pdc_get_next_profile */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_q_wds2a_rsp_cb
===========================================================================*/
/*!
  @brief Handles a QMI_WDS_GET_PROFILE_LIST_RESP for
  QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 Set

  @details

  @param qmi_txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_pdc_wds2a_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_get_profile_list_req_msg_v01 *qmi_req = NULL;
  wds_get_profile_list_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *info = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_req = (wds_get_profile_list_req_msg_v01 *)qmi_txn->req.data;
  qmi_rsp = (wds_get_profile_list_resp_msg_v01 *)qmi_txn->rsp.data;

  /* Modem may reject EPC profile type depending on modem configuration,
  proceed wihtout failing the query. */
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01 &&
    !(qmi_req->profile_type == WDS_PROFILE_TYPE_EPC_V01 &&
       qmi_rsp->resp.error == QMI_ERR_EXTENDED_INTERNAL_V01 &&
       qmi_rsp->extended_error_code_valid && qmi_rsp->extended_error_code ==
       WDS_EEC_DS_PROFILE_REG_RESULT_ERR_INVAL_PROFILE_TYPE_V01))
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
    qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status(
       qmi_txn->parent, qmi_rsp->resp.error, qmi_rsp->extended_error_code_valid,
       qmi_rsp->extended_error_code);
  }
  else if (qmi_rsp->profile_list_len > QMI_WDS_PROFILE_LIST_MAX_V01)
  {
    QBI_LOG_E_1("Invalid number of profiles %d", qmi_rsp->profile_list_len);
  }
  else
  {
    if (qmi_txn->parent->info == NULL)
    {
      qmi_txn->parent->info = QBI_MEM_MALLOC_CLEAR(
      sizeof(qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s));
      QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
    }

    /* Append new profile indexes to the profile list */
    info = (qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *)
            qmi_txn->parent->info;
    qbi_svc_bc_ext_populate_profile_list(&info->profile_list, qmi_rsp);

    if (qmi_req->profile_type == WDS_PROFILE_TYPE_EPC_V01)
    {
      /* Send another QMI_WDS_GET_PROFILE_LIST_REQ to obtain 3GPP profile
      list */
      qmi_req = (wds_get_profile_list_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(
        qmi_txn->parent, QBI_QMI_SVC_WDS, QMI_WDS_GET_PROFILE_LIST_REQ_V01,
        qbi_svc_bc_ext_provisioned_contexts_v2_pdc_wds2a_rsp_cb);
        QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

      qmi_req->profile_type_valid = TRUE;
      qmi_req->profile_type = WDS_PROFILE_TYPE_3GPP_V01;
      action = QBI_SVC_ACTION_SEND_QMI_REQ;
    }
    else
    {
      QBI_LOG_D_1("Received Total %d EPC/3GPP Profiles via GET_LIST",
                  info->profile_list.num_of_profile);

      info->profiles_read = 0;
      action = qbi_svc_bc_ext_provisioned_contexts_v2_pdc_get_next_profile(
               qmi_txn->parent);
    }
  }

  if (action == QBI_SVC_ACTION_ABORT &&
      QBI_SVC_BC_MBIM_CID_SUBSCRIBER_READY_STATUS == qmi_txn->parent->cid)
  {
    action = QBI_SVC_ACTION_SEND_RSP;
    cmd_in_progress_ignore_indication = FALSE;
  }
  return action;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_pdc_wds2a_rsp_cb */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_prep_wds29_req
===========================================================================*/
/*!
  @brief Handles a QMI_WDS_DELETE_PROFILE_LIST_REQ for
  QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 Set.Additionally
  Leveraging the existing prov ctx factory restore processing 
  functions to perform the profile disable/enable updates 

  @details

  @param qmi_txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_prep_wds29_req
(
  qbi_txn_s *txn
)
{
  wds_get_profile_list_req_msg_v01 *qmi_req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_LOG_D_0("Sending Request For Get Profile List");
  qmi_req = (wds_get_profile_list_req_msg_v01 *)
         qbi_qmi_txn_alloc_ret_req_buf(
         txn, QBI_QMI_SVC_WDS, QMI_WDS_GET_PROFILE_LIST_REQ_V01,
         qbi_svc_bc_ext_provisioned_contexts_v2_pdc_wds2a_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  qmi_req->profile_type_valid = TRUE;
  qmi_req->profile_type = WDS_PROFILE_TYPE_EPC_V01;

  return QBI_SVC_ACTION_SEND_QMI_REQ;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_prep_wds29_req */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_wds29_req
===========================================================================*/
/*!
  @brief Handles a QMI_WDS_DELETE_PROFILE_LIST_REQ for
  QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 Set

  @details

  @param qmi_txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_wds29_req
(
  qbi_txn_s *txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_delete_profile_req_msg_v01 *qmi_req_wds29 = NULL;
  wds_get_profile_list_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_cache_s *cache = NULL;
  uint32 *info = NULL;
  uint32 profile_index = 0;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);

  txn->info = QBI_MEM_MALLOC_CLEAR(sizeof(uint32));
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);
  info = (uint32*)txn->info;

  // delete user defined profiles and reset cache
  profile_index = 
      qbi_svc_bc_ext_get_user_defined_profile_from_cache(txn->ctx);
  QBI_LOG_D_1("Initiating Delete request for profile index %d",
               profile_index);
  if (QBI_SVC_BC_EXT_INVALID_PROFILE_INDEX != profile_index &&
      profile_index < QMI_WDS_PROFILE_LIST_MAX_V01)
  {
     qmi_req_wds29 = (wds_delete_profile_req_msg_v01 *)
      qbi_qmi_txn_alloc_ret_req_buf(
        txn, QBI_QMI_SVC_WDS, QMI_WDS_DELETE_PROFILE_REQ_V01,
        qbi_svc_bc_ext_provisioned_context_v2_wds29_rsp_cb);
     QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req_wds29);

     qmi_req_wds29->profile.profile_index = profile_index;
     *info = profile_index;

     action = QBI_SVC_ACTION_SEND_QMI_REQ;

     QBI_LOG_D_1("Initiating Delete request for profile index %d",
                 profile_index);
  }
  else
  {
      cache = (qbi_svc_bc_ext_cache_s *)
              qbi_svc_cache_get(txn->ctx, QBI_SVC_ID_BC_EXT);
      QBI_CHECK_NULL_PTR_RET_ABORT(cache);

       // clear cache
      QBI_MEMSET(cache, 0, 
         sizeof(qbi_svc_bc_ext_cache_s)  * QMI_WDS_PROFILE_LIST_MAX_V01);

      if(txn->info != NULL)
      {
         QBI_MEM_FREE(txn->info);
         txn->info = NULL;
      }

      action = qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_prep_wds29_req(txn);
  }

  return action;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_wds29_req */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_context_v2_s_pdc27_ind_cb
===========================================================================*/
/*!
    @brief Handles a QMI_PDC_ACTIVATE_REPORT_IND sttaic IND,
    looking for operating mode changes to trigger a
    QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 event

    @details

    @param ind

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_s_pdc27_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  const pdc_activate_config_ind_msg_v01 *act_ind;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->buf->data);

  act_ind = (const pdc_activate_config_ind_msg_v01 *)ind->buf->data;
  if (act_ind->error != 0)
  {
    QBI_LOG_E_1("Received error code %d from QMI", act_ind->error);
  }
  else
  {
     if (!cmd_in_progress_ignore_indication)
     {
       QBI_LOG_D_0("Static Ind : PDC Activation Ind Received,Triggering Query");
       cmd_in_progress_ignore_indication = TRUE;
       action = qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_wds29_req(ind->txn);
     }
  }

  QBI_LOG_I_0("Static Ind : Processed PDC Activate Indication");
  return action;
} /* qbi_svc_bc_ext_provisioned_context_v2_s_pdc27_ind_cb() */


/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_context_v2_pdc27_ind_cb
===========================================================================*/
/*!
    @brief Handles a QMI_PDC_ACTIVATE_REPORT_IND, looking for operating mode
    changes to trigger a QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 event

    @details

    @param ind

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_pdc27_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  const pdc_activate_config_ind_msg_v01 *act_ind;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->buf->data);

  act_ind = (const pdc_activate_config_ind_msg_v01 *)ind->buf->data;
  if (act_ind->error != 0)
  {
    QBI_LOG_E_1("Received error code %d from QMI", act_ind->error);
  }
  else
  {
    action = qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_wds29_req(ind->txn);
  }

  QBI_LOG_I_0("Processed PDC Activate Indication");
  return action;
} /* qbi_svc_bc_ext_provisioned_context_v2_pdc27_ind_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_context_v2_pdc23_ind_cb
===========================================================================*/
/*!
  @brief Handles a QMI_PDC_SET_SELECTED_CONFIG_REPORT_IND, looking for operating mode
  changes to trigger a QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 event

  @details

  @param ind

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_pdc23_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  const pdc_set_selected_config_ind_msg_v01 *setsel_ind;
  pdc_activate_config_req_msg_v01 *qmi_req_pdc27 = NULL;
  qbi_svc_bc_ext_exec_slot_config_s exec_slot_cfg = {0};
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->buf->data);

  setsel_ind = (const pdc_set_selected_config_ind_msg_v01 *)ind->buf->data;
  if (setsel_ind->error != 0)
  {
    QBI_LOG_E_1("Received error code %d from QMI", setsel_ind->error);
  }
  else
  {
    QBI_LOG_I_0("Initiating Req For PDC Activate Config");
    if (setsel_ind->ind_token_valid == TRUE && setsel_ind->ind_token == PROV_V2_IND_TOKEN)
    {
      qbi_nv_store_cfg_item_read(ind->txn->ctx,
        QBI_NV_STORE_CFG_ITEM_EXECUTOR_SLOT_CONFIG, &exec_slot_cfg,
        sizeof(qbi_svc_bc_ext_exec_slot_config_s));

      QBI_LOG_D_1("exec_slot_cfg.exec0_slot %d",exec_slot_cfg.exec0_slot);

      qmi_req_pdc27 = (pdc_activate_config_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(
          ind->txn, QBI_QMI_SVC_PDC, QMI_PDC_ACTIVATE_CONFIG_REQ_V01,
          qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc27_rsp_cb);
      QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req_pdc27);


      qmi_req_pdc27->config_type = PDC_CONFIG_TYPE_MODEM_SW_V01;
      qmi_req_pdc27->ind_token_valid = TRUE;
      qmi_req_pdc27->ind_token = PROV_V2_IND_TOKEN;
      qmi_req_pdc27->activation_type_valid = TRUE;
      qmi_req_pdc27->activation_type = PDC_ACTIVATION_REGULAR_V01;
      qmi_req_pdc27->subscription_id_valid = TRUE;
      qmi_req_pdc27->subscription_id = 0;
      qmi_req_pdc27->slot_id_valid = TRUE;
      qmi_req_pdc27->slot_id = exec_slot_cfg.exec0_slot;
      action = QBI_SVC_ACTION_SEND_QMI_REQ;
    }
  }

  return action;
} /* qbi_svc_bc_ext_provisioned_context_v2_pdc23_ind_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_context_v2_pdc22_ind_cb
===========================================================================*/
/*!
  @brief Handles a QMI_PDC_GET_SELECTED_CONFIG_REPORT_IND, looking for operating mode
  changes to trigger a QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 event

  @details

  @param ind

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_pdc22_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  const pdc_get_selected_config_ind_msg_v01 *getsel_ind;
  pdc_deactivate_config_req_msg_v01 *qmi_req_pdc2b = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->buf->data);

  getsel_ind = (const pdc_get_selected_config_ind_msg_v01 *)ind->buf->data;
  if (getsel_ind->error != 0)
  {
    QBI_LOG_E_1("Received error code %d from QMI", getsel_ind->error);
  }
  else
  {
    if (getsel_ind->ind_token_valid == TRUE && getsel_ind->ind_token == PROV_V2_IND_TOKEN)
    {
      QBI_LOG_I_0("Processing GET SELECTED CONFIG REPORT Ind");
      if (getsel_ind->active_config_id_valid == TRUE)
      {
        active_config_id_len = getsel_ind->active_config_id_len;
        QBI_MEMSCPY(active_config_id,PDC_CONFIG_ID_SIZE_MAX_V01,
                    getsel_ind->active_config_id, sizeof(getsel_ind->active_config_id)/sizeof(uint8_t));
      }

      qmi_req_pdc2b = (pdc_deactivate_config_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(
          ind->txn, QBI_QMI_SVC_PDC, QMI_PDC_DEACTIVATE_CONFIG_REQ_V01,
          qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc2b_rsp_cb);
      QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req_pdc2b);

      qmi_req_pdc2b->config_type = PDC_CONFIG_TYPE_MODEM_SW_V01;
      qmi_req_pdc2b->ind_token_valid = TRUE;
      qmi_req_pdc2b->ind_token = PROV_V2_IND_TOKEN;
      qmi_req_pdc2b->subscription_id_valid = TRUE;
      qmi_req_pdc2b->subscription_id = 0;
      action = QBI_SVC_ACTION_SEND_QMI_REQ;
    }
  }

  QBI_LOG_I_0("Exiting GET SELECTED CONFIG REPORT Ind");
  return action;
} /* qbi_svc_bc_ext_provisioned_context_v2_pdc22_ind_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_context_v2_pdc2b_ind_cb
===========================================================================*/
/*!
    @brief  Handles a QMI_PDC_DEACTIVATE_REPORT_IND, looking for operating mode 
    changes to trigger a QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 event

    @details

    @param ind

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_pdc2b_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  const pdc_deactivate_config_ind_msg_v01 *deact_ind;
  pdc_set_selected_config_req_msg_v01 *qmi_req_pdc23 = NULL;
  qbi_svc_bc_ext_exec_slot_config_s exec_slot_cfg = {0};
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->buf->data);

  deact_ind = (const pdc_deactivate_config_ind_msg_v01 *)ind->buf->data;
  if (deact_ind->error != 0)
  {
    QBI_LOG_E_1("Received error code %d from QMI", deact_ind->error);
  }
  else
  {
    QBI_LOG_I_0("Processing PDC DEACTIVATE REPORT Ind");
    if (deact_ind->ind_token_valid == TRUE && deact_ind->ind_token == PROV_V2_IND_TOKEN)
    {
      qbi_nv_store_cfg_item_read(ind->txn->ctx,
        QBI_NV_STORE_CFG_ITEM_EXECUTOR_SLOT_CONFIG, &exec_slot_cfg,
        sizeof(qbi_svc_bc_ext_exec_slot_config_s));

      QBI_LOG_D_1("exec_slot_cfg.exec0_slot %d",exec_slot_cfg.exec0_slot);

      qmi_req_pdc23 = (pdc_set_selected_config_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(
          ind->txn, QBI_QMI_SVC_PDC, QMI_PDC_SET_SELECTED_CONFIG_REQ_V01,
          qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc23_rsp_cb);
      QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req_pdc23);

      qmi_req_pdc23->new_config_info.config_type = PDC_CONFIG_TYPE_MODEM_SW_V01;

      if (active_config_id_len != 0)
      {
         qmi_req_pdc23->new_config_info.config_id_len = active_config_id_len;
         QBI_MEMSCPY(qmi_req_pdc23->new_config_info.config_id,PDC_CONFIG_ID_SIZE_MAX_V01,
                     active_config_id,sizeof(active_config_id)/sizeof(uint8_t));
         QBI_LOG_I_1("config_id_len %d", qmi_req_pdc23->new_config_info.config_id_len);
      }
      qmi_req_pdc23->ind_token_valid = TRUE;
      qmi_req_pdc23->ind_token = PROV_V2_IND_TOKEN;
      qmi_req_pdc23->subscription_id_valid = TRUE;
      qmi_req_pdc23->subscription_id = 0;
      qmi_req_pdc23->slot_id = exec_slot_cfg.exec0_slot;
      qmi_req_pdc23->slot_id_valid = TRUE;
      action = QBI_SVC_ACTION_SEND_QMI_REQ;
    }
  }

  return action;
} /* qbi_svc_bc_ext_provisioned_context_v2_pdc2b_ind_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_context_v2_wdsa8_ind_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_EVENT_REPORT_IND, looking for operating mode
    changes to trigger a QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 event

    @details

    @param ind

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_wdsa8_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  const wds_profile_changed_ind_msg_v01 *resp = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->buf->data);
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->txn);

  resp = (const wds_profile_changed_ind_msg_v01 *)ind->buf->data;

  if (resp->profile_changed_ind_valid)
  {
     QBI_LOG_D_3("Received Indication for Profile Type %d, Profile Index %d,Change Evt %d",
                  resp->profile_changed_ind.profile_type, 
                  resp->profile_changed_ind.profile_index,
                  resp->profile_changed_ind.profile_change_evt);
  }

  if (!cmd_in_progress_ignore_indication)
  {
     QBI_LOG_D_0("Profile Changed Ind Received,Triggering Query");
     action = qbi_svc_bc_ext_provisioned_context_v2_q_req(ind->txn);
  }

  return action;
} /* qbi_svc_bc_ext_provisioned_context_v2_wdsa8_ind_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_context_v2_qmi_ind_reg_wdsa8_rsp_cb
===========================================================================*/
/*!
  @brief Handles QMI_DMS_SET_EVENT_REPORT_RESP

  @details

  @param qmi_txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_qmi_ind_reg_wdsa8_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_config_profile_list_resp_msg_v01 *qmi_rsp = NULL;
  pdc_indication_register_req_msg_v01 *qmi_req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  QBI_LOG_D_0("Received QMI_WDS_CONFIGURE_PROFILE_EVENT_LIST_RESP_V01");
  
  qmi_rsp = (wds_config_profile_list_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
    action = QBI_SVC_ACTION_ABORT;
  }
  else
  {
    qmi_req = (pdc_indication_register_req_msg_v01 *)
      qbi_qmi_txn_alloc_ret_req_buf(
        qmi_txn->parent, QBI_QMI_SVC_PDC, QMI_PDC_INDICATION_REGISTER_REQ_V01,
        qbi_svc_bc_ext_provisioned_context_v2_qmi_ind_reg_pdc20_res);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

    qmi_req->reg_config_change_valid = TRUE;
    qmi_req->reg_config_change = TRUE;

    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }
  return action;
} /* qbi_svc_bc_ext_provisioned_context_v2_qmi_ind_reg_wdsa8_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc27_rsp_cb
===========================================================================*/
/*!
  @brief Handles QMI_PDC_ACTIVATE_RESP

  @details

  @param qmi_txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc27_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  pdc_activate_config_resp_msg_v01 *qmi_resp_pdc27 = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_resp_pdc27 = (pdc_activate_config_resp_msg_v01 *)qmi_txn->rsp.data;

  if (qmi_resp_pdc27->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_resp_pdc27->resp.error);
  }
  else
  {
    action = QBI_SVC_ACTION_WAIT_ASYNC_RSP;
  }

  QBI_LOG_D_0("Received PDC Activation Response");
  return action;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc27_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc23_rsp_cb
===========================================================================*/
/*!
 @brief Handles QMI_PDC_SET_SELECTED_CONFIG_RESP

 @details

 @param qmi_txn

 @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc23_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  pdc_get_selected_config_resp_msg_v01 *qmi_resp_pdc23 = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_resp_pdc23 = (pdc_get_selected_config_resp_msg_v01 *)qmi_txn->rsp.data;

  if (qmi_resp_pdc23->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_resp_pdc23->resp.error);
  }
  else
  {
    QBI_LOG_D_0("Got PDC SELECT CONFIG resp Callback");
    action = QBI_SVC_ACTION_WAIT_ASYNC_RSP;
  }

 return action;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc23_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc22_rsp_cb
===========================================================================*/
/*!
 @brief Handles QMI_PDC_GET_SELECTED_CONFIG_RESP

 @details

 @param qmi_txn

 @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc22_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  pdc_get_selected_config_resp_msg_v01 *qmi_resp_pdc22 = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_resp_pdc22 = (pdc_get_selected_config_resp_msg_v01 *)qmi_txn->rsp.data;

  if (qmi_resp_pdc22->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_resp_pdc22->resp.error);
    action = QBI_SVC_ACTION_ABORT;
  }
  else
  {
    QBI_LOG_D_0("Got PDC SELECT CONFIG response");
    action = QBI_SVC_ACTION_WAIT_ASYNC_RSP;
  }

  return action;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc22_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc2b_rsp_cb
===========================================================================*/
/*!
    @brief Handles QMI_PDC_DEACTIVATE_RESP

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc2b_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  pdc_deactivate_config_resp_msg_v01 *qmi_rsp = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (pdc_deactivate_config_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
  }
  else
  {
     action = QBI_SVC_ACTION_WAIT_ASYNC_RSP;
  }

  QBI_LOG_D_0("Received PDC DeActivation Callback");
  return action;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc2b_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_context_v2_qmi_ind_reg_pdc20_res
===========================================================================*/
/*!
  @brief Handles QMI_PDC_INDICATION_REGISTER_RESP

  @details

  @param qmi_txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_qmi_ind_reg_pdc20_res
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  pdc_indication_register_resp_msg_v01 *qmi_rsp = NULL;
  dsd_system_status_change_req_msg_v01* qmi_req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  QBI_LOG_D_0("Received QMI_PDC_INDICATION_REGISTER_RESP");

  qmi_rsp = (pdc_indication_register_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
    action = QBI_SVC_ACTION_ABORT;
  }
  else
  {
    /* Register for QMI_UIM_STATUS_CHANGE_IND */
    qmi_req = (dsd_system_status_change_req_msg_v01 *)
      qbi_qmi_txn_alloc_ret_req_buf(
        qmi_txn->parent, QBI_QMI_SVC_DSD, QMI_DSD_SYSTEM_STATUS_CHANGE_REQ_V01,
        qbi_svc_bc_ext_lte_attach_status_reg_ind_dsd25_rsp);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);
    
    qmi_req->limit_so_mask_change_ind_valid = TRUE;
    qmi_req->limit_so_mask_change_ind = TRUE;

    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }
  return action;
}/* qbi_svc_bc_ext_provisioned_context_v2_qmi_ind_reg_pdc20_res() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_context_v2_qmi_ind_reg_wdsa8_req
===========================================================================*/
/*!
    @brief Handles QMI_DMS_SET_EVENT_REPORT_RESP

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_qmi_ind_reg_wdsa8_req
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_SEND_RSP;
  wds_config_profile_list_req_msg_v01 *qmi_req = NULL;
  wds_indication_register_resp_msg_v01 *qmi_rsp = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);

  qmi_rsp = (wds_indication_register_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Error registering for Profile change indications!!! Error code %d",
       qmi_rsp->resp.error);
    action = QBI_SVC_ACTION_ABORT;
  }
  else
  {
    QBI_LOG_D_0("Setting INDICATION Params");
    qmi_req = (wds_config_profile_list_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(
        qmi_txn->parent, QBI_QMI_SVC_WDS, QMI_WDS_CONFIGURE_PROFILE_EVENT_LIST_REQ_V01,
        qbi_svc_bc_ext_provisioned_context_v2_qmi_ind_reg_wdsa8_rsp_cb);

    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);
    qmi_req->profile_event_register_valid = TRUE;
    qmi_req->profile_event_register_len = 1;
    qmi_req->profile_event_register[0].profile_type = 0xFF;
    qmi_req->profile_event_register[0].profile_index = 0xFF;
    
    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }


  return action;
} /* qbi_svc_bc_ext_provisioned_context_v2_qmi_ind_reg_wdsa8_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status
===========================================================================*/
/*!
    @brief Attempts to map QMI error information into a descriptive MBIM
    error status for MBIM_CID_PROVISIONED_CONTEXT_V2

    @details

    @param txn
    @param qmi_error
    @param qmi_error_ds_ext_valid
    @param qmi_error_ds_ext
*/
/*=========================================================================*/
static void qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status
(
  qbi_txn_s                          *txn,
  qmi_error_type_v01                  qmi_error,
  uint8_t                             qmi_error_ds_ext_valid,
  wds_ds_extended_error_code_enum_v01 qmi_error_ds_ext
)
{
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET(txn);

  /* For all aborts setting the status, hence setting this to 
  FALSE to handle new request */
  cmd_in_progress_ignore_indication =  FALSE;

  /* Map extended error first, then map QMI error  */
  if (qmi_error == QMI_ERR_EXTENDED_INTERNAL_V01 && qmi_error_ds_ext_valid)
  {
    QBI_LOG_E_1("DS Profile extended error code 0x%x", qmi_error_ds_ext);
    switch (qmi_error_ds_ext)
    {
      case WDS_EEC_DS_PROFILE_REG_RESULT_ERR_LIB_NOT_INITED_V01:
        txn->status = QBI_MBIM_STATUS_NOT_INITIALIZED;
        break;

      case WDS_EEC_DS_PROFILE_REG_RESULT_ERR_LEN_INVALID_V01:
        txn->status = QBI_MBIM_STATUS_INVALID_PARAMETERS;
        break;

      case WDS_EEC_DS_PROFILE_REG_3GPP_ACCESS_ERR_V01:
      case WDS_EEC_DS_PROFILE_3GPP_ACCESS_ERR_V01:
        txn->status = QBI_MBIM_STATUS_READ_FAILURE;
        break;

      case WDS_EEC_DS_PROFILE_REG_3GPP_ERR_OUT_OF_PROFILES_V01:
      case WDS_EEC_DS_PROFILE_REG_3GPP_READ_ONLY_FLAG_SET_V01:
      case WDS_EEC_DS_PROFILE_3GPP_ERR_OUT_OF_PROFILES_V01:
      case WDS_EEC_DS_PROFILE_3GPP_READ_ONLY_FLAG_SET_V01:
        txn->status = QBI_MBIM_STATUS_WRITE_FAILURE;
        break;

      default:
        txn->status = QBI_MBIM_STATUS_FAILURE;
    }
  }
  else
  {
    switch (qmi_error)
    {
      case QMI_ERR_INVALID_PROFILE_V01:
        txn->status = QBI_MBIM_STATUS_INVALID_PARAMETERS;
        break;

      case QMI_ERR_NO_FREE_PROFILE_V01:
        txn->status = QBI_MBIM_STATUS_WRITE_FAILURE;
        break;

      default:
      txn->status = QBI_MBIM_STATUS_FAILURE;
    }
   }
} /* qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status() */


/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_to_mbim_auth_proto
===========================================================================*/
/*!
    @brief Extracts the authentication protocol information from a QMI
    profile (EPC or 3GPP), if available, and returns it as an MBIM value

    @details

    @param profile_type
    @param qmi_rsp

    @return uint32 MBIM_AUTH_PROTOCOL value
*/
/*=========================================================================*/
static uint32 qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_to_mbim_auth_proto
(
  wds_profile_type_enum_v01                    profile_type,
  const wds_get_profile_settings_resp_msg_v01 *qmi_rsp
)
{
  uint32 mbim_auth_proto = QBI_SVC_BC_AUTH_PROTOCOL_NONE;
/*-------------------------------------------------------------------------*/
  if (qmi_rsp == NULL)
  {
     QBI_LOG_E_0("Unexpected NULL pointer!");
  }
  else if (profile_type == WDS_PROFILE_TYPE_EPC_V01 &&
    qmi_rsp->common_auth_protocol_valid)
  {
    if (qmi_rsp->common_auth_protocol == WDS_PROFILE_AUTH_PROTOCOL_CHAP_V01 ||
      qmi_rsp->common_auth_protocol == WDS_PROFILE_AUTH_PROTOCOL_PAP_CHAP_V01)
    {
      mbim_auth_proto = QBI_SVC_BC_AUTH_PROTOCOL_CHAP;
    }
    else if (qmi_rsp->common_auth_protocol == WDS_PROFILE_AUTH_PROTOCOL_PAP_V01)
    {
      mbim_auth_proto = QBI_SVC_BC_AUTH_PROTOCOL_PAP;
    }
  }
  else if (profile_type == WDS_PROFILE_TYPE_3GPP_V01 &&
    qmi_rsp->authentication_preference_valid)
  {
    if (qmi_rsp->authentication_preference & QMI_WDS_MASK_AUTH_PREF_CHAP_V01)
   {
     mbim_auth_proto = QBI_SVC_BC_AUTH_PROTOCOL_CHAP;
   }
   else if (qmi_rsp->authentication_preference & QMI_WDS_MASK_AUTH_PREF_PAP_V01)
   {
     mbim_auth_proto = QBI_SVC_BC_AUTH_PROTOCOL_PAP;
   }
  }
  else
  {
    QBI_LOG_E_1("Invalid profile type %d", profile_type);
  }

  return mbim_auth_proto;
} /* qbi_svc_bc_provisioned_contexts_qmi_auth_pref_to_mbim_auth_proto() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_to_context_type
===========================================================================*/
/*!
  @brief Determines the ContextType UUID for QMI profile common app user
  data TLV

  @details

  @param context_id

  @return const uint8* Pointer to UUID, or NULL on unexpected error
*/
/*=========================================================================*/
static const uint8 *qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_to_context_type
(
  const wds_get_profile_settings_resp_msg_v01 *qmi_rsp
)
{
  const uint8 *context_type = NULL;
  uint32 app_user_data;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_NULL(qmi_rsp);

  if (qmi_rsp->common_app_user_data_valid)
  {
    app_user_data = qmi_rsp->common_app_user_data;
  }
  else if (qmi_rsp->app_user_data_3gpp_valid)
  {
    app_user_data = qmi_rsp->app_user_data_3gpp;
  }
  else if (qmi_rsp->app_user_data_3gpp2_valid)
  {
    app_user_data = qmi_rsp->app_user_data_3gpp2;
  }
  else
  {
    QBI_LOG_E_0("App User Data TLVs are not present in response!");
    app_user_data = QBI_SVC_BC_CONTEXT_TYPE_INTERNET;
  }

  context_type = qbi_svc_bc_context_type_id_to_uuid(app_user_data);
  if (context_type == NULL)
  {
    QBI_LOG_I_0("Couldn't determine ContextType UUID based on context type");
    context_type = qbi_svc_bc_context_type_id_to_uuid(
      QBI_SVC_BC_CONTEXT_TYPE_INTERNET);
  }

  return context_type;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_to_context_type() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_populate_profile_list
===========================================================================*/
/*!
  @brief Process WDS_GET_PROFILE_LIST_RESP to populate profile indexes

  @details
  profile_list may already contain profile indexes from previous profile
  list query. Append new profile indexes to the list.

  @param profile_list
  @param qmi_rsp

  @return boolean
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_populate_profile_list
(
  qbi_svc_bc_ext_profile_list_s         *profile_list,
  wds_get_profile_list_resp_msg_v01 *qmi_rsp
)
{
  uint32 i;
  boolean result = FALSE;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(profile_list);
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_rsp);

  /* When query all profile types and consolidate profile list, the total
  number of profiles should be within QMI_WDS_PROFILE_LIST_MAX_V01 */
  if (profile_list->num_of_profile + qmi_rsp->profile_list_len >
    QMI_WDS_PROFILE_LIST_MAX_V01)
  {
    QBI_LOG_E_1("Unexpected number of profiles %d",
       profile_list->num_of_profile);
  }
  else
  {
    QBI_LOG_D_1("Adding %d Profiles To Local List",qmi_rsp->profile_list_len);
    for (i = 0; i < qmi_rsp->profile_list_len; i++)
    {
        profile_list->profile_type[profile_list->num_of_profile + i] =
            qmi_rsp->profile_list[i].profile_type;
        profile_list->profile_index[profile_list->num_of_profile + i] =
            qmi_rsp->profile_list[i].profile_index;
        QBI_LOG_I_2("profile index %d profile type %d", qmi_rsp->profile_list[i].profile_index,
                     qmi_rsp->profile_list[i].profile_type);
    }
    profile_list->num_of_profile += qmi_rsp->profile_list_len;
    result = TRUE;
    QBI_LOG_D_1("Number of profiles added to list %d",profile_list->num_of_profile);
  }

  return result;
} /* qbi_svc_bc_ext_populate_profile_list() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_to_mbim_compression
===========================================================================*/
/*!
  @brief Extracts the compression information from a QMI profile (3GPP or
  3GPP2), if available, and returns it as an MBIM value

  @details

  @param profile_type
  @param qmi_rsp

  @return uint32 MBIM_COMPRESSION value
*/
/*=========================================================================*/
static uint32 qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_to_mbim_compression
(
  wds_profile_type_enum_v01                    profile_type,
  const wds_get_profile_settings_resp_msg_v01 *qmi_rsp
)
{
  uint32 mbim_compression = QBI_SVC_BC_COMPRESSION_NONE;
/*-------------------------------------------------------------------------*/
  if (qmi_rsp == NULL)
  {
    QBI_LOG_E_0("Unexpected NULL pointer!");
  }
  else if (profile_type == WDS_PROFILE_TYPE_EPC_V01 &&
    ((qmi_rsp->pdp_data_compression_type_valid &&
        qmi_rsp->pdp_data_compression_type !=
        WDS_PDP_DATA_COMPR_TYPE_OFF_V01) ||
        (qmi_rsp->pdp_hdr_compression_type_valid &&
            qmi_rsp->pdp_hdr_compression_type !=
            WDS_PDP_HDR_COMPR_TYPE_OFF_V01)))
  {
    mbim_compression = QBI_SVC_BC_COMPRESSION_ENABLE;
  }

  return mbim_compression;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_to_mbim_compression() */


/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_context_v2_q_req
===========================================================================*/
/*!
    @brief Handles a QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 query 
    request

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_q_req
(
  qbi_txn_s *txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_get_profile_list_req_msg_v01 *qmi_req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);

  if (!qbi_svc_bc_sim_subscriber_ready_status_is_ready(
        txn, TRUE))
  {
    QBI_LOG_D_0("Device Not Ready Unable to Process Query Request! ");
    txn->status = QBI_MBIM_STATUS_FAILURE;
    action = QBI_SVC_ACTION_ABORT;
  }
  else
  {
    cmd_in_progress_ignore_indication = TRUE;

    QBI_LOG_D_0("Received Query For Prov Context V2");
    /* Getting List for EPC,3GPP and then 3GPP2 sequentially */
    qmi_req = (wds_get_profile_list_req_msg_v01 *)
               qbi_qmi_txn_alloc_ret_req_buf(
               txn, QBI_QMI_SVC_WDS, QMI_WDS_GET_PROFILE_LIST_REQ_V01,
               qbi_svc_bc_ext_provisioned_contexts_v2_q_wds2a_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

    qmi_req->profile_type_valid = TRUE;
    qmi_req->profile_type = WDS_PROFILE_TYPE_EPC_V01;

    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }
  return action;
} /* qbi_svc_bc_ext_provisioned_context_v2_q_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_q_wds2a_rsp_cb
===========================================================================*/
/*!
  @brief Handles a QMI_WDS_GET_PROFILE_LIST_RESP for
  QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 query

  @details

  @param qmi_txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_q_wds2a_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  wds_get_profile_list_req_msg_v01 *qmi_req = NULL;
  wds_get_profile_list_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *info = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_list_s *rsp_list = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_req = (wds_get_profile_list_req_msg_v01 *)qmi_txn->req.data;
  qmi_rsp = (wds_get_profile_list_resp_msg_v01 *)qmi_txn->rsp.data;

  /* Modem may reject EPC profile type depending on modem configuration,
  proceed wihtout failing the query. */
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01 &&
    !(qmi_req->profile_type == WDS_PROFILE_TYPE_EPC_V01 &&
       qmi_rsp->resp.error == QMI_ERR_EXTENDED_INTERNAL_V01 &&
       qmi_rsp->extended_error_code_valid && qmi_rsp->extended_error_code ==
       WDS_EEC_DS_PROFILE_REG_RESULT_ERR_INVAL_PROFILE_TYPE_V01))
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
    qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status(
       qmi_txn->parent, qmi_rsp->resp.error, qmi_rsp->extended_error_code_valid,
       qmi_rsp->extended_error_code);
  }
  else if (qmi_rsp->profile_list_len > QMI_WDS_PROFILE_LIST_MAX_V01)
  {
    QBI_LOG_E_1("Invalid number of profiles %d", qmi_rsp->profile_list_len);
  }
  else
  {
    if (qmi_txn->parent->info == NULL)
    {
      qmi_txn->parent->info = QBI_MEM_MALLOC_CLEAR(
      sizeof(qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s));
      QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
    }

    /* Append new profile indexes to the profile list */
    info = (qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *)
      qmi_txn->parent->info;
    qbi_svc_bc_ext_populate_profile_list(&info->profile_list, qmi_rsp);

    if (qmi_req->profile_type == WDS_PROFILE_TYPE_EPC_V01)
    {
      /* Send another QMI_WDS_GET_PROFILE_LIST_REQ to obtain 3GPP profile
      list */
      qmi_req = (wds_get_profile_list_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(
        qmi_txn->parent, QBI_QMI_SVC_WDS, QMI_WDS_GET_PROFILE_LIST_REQ_V01,
        qbi_svc_bc_ext_provisioned_contexts_v2_q_wds2a_rsp_cb);
        QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

      qmi_req->profile_type_valid = TRUE;
      qmi_req->profile_type = WDS_PROFILE_TYPE_3GPP_V01;
      action = QBI_SVC_ACTION_SEND_QMI_REQ;
    }
    else
    {
      QBI_LOG_D_1("Received Total %d EPC/3GPP Profiles via GET_LIST",
                  info->profile_list.num_of_profile);
      QBI_LOG_D_0("Initiating QMI_WDS_GET_PROFILE_SETTINGS_REQ Req");
      /* Allocate the fixed-length and offset/size pair portion of the
      response now. */
      rsp_list = (qbi_svc_bc_ext_provisioned_contexts_v2_list_s *)
                  qbi_txn_alloc_rsp_buf(
                  qmi_txn->parent, (sizeof(qbi_svc_bc_ext_provisioned_contexts_v2_list_s) +
                  sizeof(qbi_mbim_offset_size_pair_s) *
                  info->profile_list.num_of_profile));
      QBI_CHECK_NULL_PTR_RET_ABORT(rsp_list);
      rsp_list->element_count = info->profile_list.num_of_profile;

      info->profiles_read = 0;
      action = qbi_svc_bc_ext_provisioned_contexts_v2_q_get_next_profile(
               qmi_txn->parent);
    }
  }

  return action;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_q_wds2a_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_q_get_next_profile
===========================================================================*/
/*!
  @brief Retrive next available configured profile.

  @details

  @param txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_q_get_next_profile
(
  qbi_txn_s *txn
)
{
  wds_get_profile_settings_req_msg_v01 *qmi_req_wds2b = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *info = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);

  info = (qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *)txn->info;
  if (info->profiles_read >= info->profile_list.num_of_profile)
  {
    cmd_in_progress_ignore_indication = FALSE;
    action = QBI_SVC_ACTION_SEND_RSP;
    txn->status = QBI_MBIM_STATUS_SUCCESS;
  }
  else
  {
    /* Issue a query to retrieve the profile details */
    qmi_req_wds2b = (wds_get_profile_settings_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(
        txn, QBI_QMI_SVC_WDS, QMI_WDS_GET_PROFILE_SETTINGS_REQ_V01,
        qbi_svc_bc_ext_provisioned_contexts_v2_q_wds2b_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req_wds2b);

    qmi_req_wds2b->profile.profile_type =
      info->profile_list.profile_type[info->profiles_read];
    qmi_req_wds2b->profile.profile_index =
      info->profile_list.profile_index[info->profiles_read];

    QBI_LOG_D_2("profile type %d index %d",
                 qmi_req_wds2b->profile.profile_type,qmi_req_wds2b->profile.profile_index);
    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }

  return action;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_q_get_next_profile */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_q_wds2b_rsp_cb
===========================================================================*/
/*!
  @brief Handles a QMI_WDS_GET_PROFILE_SETTINGS_RESP for
  QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 query

  @details

  @param qmi_txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_q_wds2b_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  wds_get_profile_settings_req_msg_v01 *qmi_req = NULL;
  wds_get_profile_settings_resp_msg_v01 *qmi_rsp = NULL;
  qbi_mbim_offset_size_pair_s *field_desc = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *info = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_req = (wds_get_profile_settings_req_msg_v01 *)qmi_txn->req.data;
  qmi_rsp = (wds_get_profile_settings_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
    qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status(
    qmi_txn->parent, qmi_rsp->resp.error,
    qmi_rsp->extended_error_code_valid, qmi_rsp->extended_error_code);
  }
  else
  {
    info = (qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *)
            qmi_txn->parent->info;

    field_desc = (qbi_mbim_offset_size_pair_s *)
                 ((uint8 *)qmi_txn->parent->rsp.data +
                 sizeof(qbi_svc_bc_ext_provisioned_contexts_v2_list_s) +
                 sizeof(qbi_mbim_offset_size_pair_s) * info->profiles_read);

    info->profiles_read++;
    QBI_LOG_I_2("Received profile %d/%d", info->profiles_read,
                info->profile_list.num_of_profile);

    if (!qbi_svc_ext_provisioned_contexts_v2_q_add_context_to_rsp(
        qmi_txn->parent, field_desc, qmi_req->profile.profile_type,
        qmi_rsp, qmi_req->profile.profile_index))
    {
      QBI_LOG_E_0("Couldn't add context to response!");
    }
    else
    {
      action = qbi_svc_bc_ext_provisioned_contexts_v2_q_get_next_profile(
               qmi_txn->parent);
    }
  }

  return action;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_q_wds2b_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_compare_3gpp_3gpp2_profiles
===========================================================================*/
/*!
    @brief Check if partially cached 3gpp profile and 3gpp2 profile have the
    same connectivity parameters

  @details

    @param profile_settings_3gpp
    @param qmi_rsp_3gpp2

    @return boolean
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_compare_3gpp_3gpp2_profiles
(
  qbi_svc_bc_ext_provisioned_contexts_v2_3gpp_profile_settings_s *profile_settings_3gpp,
  wds_get_profile_settings_resp_msg_v01                   *qmi_rsp_3gpp2
)
{
  boolean status = FALSE;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings_3gpp);
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_rsp_3gpp2);

  if (!profile_settings_3gpp->apn_name_valid ||
      !qmi_rsp_3gpp2->apn_string_valid ||
      QBI_STRNCMP(profile_settings_3gpp->apn_name, qmi_rsp_3gpp2->apn_string,
      QMI_WDS_APN_NAME_MAX_V01))
  {
    QBI_LOG_D_0("3GPP/3GPP2 APN names do not match!");
  }
  else if (!profile_settings_3gpp->username_valid ||
      !qmi_rsp_3gpp2->user_id_valid ||
      QBI_STRNCMP(profile_settings_3gpp->username, qmi_rsp_3gpp2->user_id,
      QMI_WDS_USER_NAME_MAX_V01))
  {
    QBI_LOG_D_0("3GPP/3GPP2 usernames do not match!");
  }
  else
  {
    status = TRUE;
  }

  return status;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_s_compare_3gpp_3gpp2_profiles() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_3gpp2_wds2b_rsp_cb
===========================================================================*/
/*!
  @brief Handles a QMI_WDS_GET_PROFILE_SETTINGS_RESP for
  QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 set request to retrieve settings for a
  3gpp2 profile

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_3gpp2_wds2b_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  wds_get_profile_settings_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *info = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (wds_get_profile_settings_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
    qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status(
            qmi_txn->parent, qmi_rsp->resp.error,
            qmi_rsp->extended_error_code_valid, qmi_rsp->extended_error_code);
  }
  else
  {
    info = (qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *)
            qmi_txn->parent->info;

    if (qbi_svc_bc_ext_provisioned_contexts_v2_s_compare_3gpp_3gpp2_profiles(
            &info->profile_settings_3gpp, qmi_rsp))
    {
     info->profile_found_3gpp2 = TRUE;
     info->profile_index_3gpp2 =
     info->profile_list.profile_index[info->profiles_read];

     action = qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds28_req(
              qmi_txn->parent, WDS_PROFILE_TYPE_3GPP_V01,info->profile_index_3gpp2);
    }
    else
    {
     info->profiles_read++;
     action = qbi_svc_bc_ext_provisioned_contexts_v2_s_get_next_3gpp2_profile(
     qmi_txn->parent);
    }
  }

  return action;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_s_3gpp2_wds2b_rsp_cb() */


/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_get_next_3gpp2_profile
===========================================================================*/
/*!
  @brief Retrive next available 3gpp2 configured profile.

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_get_next_3gpp2_profile
(
  qbi_txn_s *txn
)
{
  wds_get_profile_settings_req_msg_v01 *qmi_req_wds2b = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *info = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);

  info = (qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *)txn->info;
    
  if (info->profiles_read >= info->profile_list.num_of_profile)
  {
    action = qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds28_req(
             txn, WDS_PROFILE_TYPE_3GPP_V01, info->profile_index_3gpp);
  }
  else
  {
    /* Issue a query to retrieve the profile details */
    qmi_req_wds2b = (wds_get_profile_settings_req_msg_v01 *)
      qbi_qmi_txn_alloc_ret_req_buf(
        txn, QBI_QMI_SVC_WDS, QMI_WDS_GET_PROFILE_SETTINGS_REQ_V01,
        qbi_svc_bc_ext_provisioned_contexts_v2_s_3gpp2_wds2b_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req_wds2b);

    qmi_req_wds2b->profile.profile_type = WDS_PROFILE_TYPE_3GPP2_V01;
    qmi_req_wds2b->profile.profile_index = info->profile_index_3gpp2;

    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }

  return action;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_s_get_next_3gpp2_profile */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_enable_to_mbim_enable
===========================================================================*/
/*!
  @brief Extracts the compression information from a QMI profile (3GPP or
  3GPP2), if available, and returns it as an MBIM value

  @details

  @param profile_type
  @param qmi_rsp

  @return uint32 MBIM_COMPRESSION value
*/
/*=========================================================================*/
static uint32 qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_enable_to_mbim_enable
(
  const wds_get_profile_settings_resp_msg_v01 *qmi_rsp
)
{
  uint32 enable = QBI_SVC_MBIM_MS_CONTEXT_DISABLED;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_rsp);

  if (qmi_rsp->common_apn_disabled_flag_valid == TRUE)
  {
    enable = qmi_rsp->common_apn_disabled_flag == 1 ? 
    QBI_SVC_MBIM_MS_CONTEXT_DISABLED: QBI_SVC_MBIM_MS_CONTEXT_ENABLED;
    QBI_LOG_D_2("common apn disabled flag valid %d enable %d", qmi_rsp->common_apn_disabled_flag_valid,enable);
  }
  else if (qmi_rsp->apn_disabled_flag_valid == TRUE)
  {
    enable = qmi_rsp->apn_disabled_flag == 1 ?
    QBI_SVC_MBIM_MS_CONTEXT_DISABLED : QBI_SVC_MBIM_MS_CONTEXT_ENABLED;
    QBI_LOG_D_2("apn disabled flag valid %d enable %d", qmi_rsp->apn_disabled_flag_valid,enable);
  }
  else if (qmi_rsp->apn_enabled_3gpp2_valid == TRUE)
  {
    enable = qmi_rsp->apn_enabled_3gpp2 == 1 ?
    QBI_SVC_MBIM_MS_CONTEXT_ENABLED : QBI_SVC_MBIM_MS_CONTEXT_DISABLED;
    QBI_LOG_D_2("apn enabled 3gpp2 valid %d enable %d", qmi_rsp->apn_enabled_3gpp2_valid,enable);
  }

  return enable;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_enable_to_mbim_enable() */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_update_cache
===========================================================================*/
/*!
    @brief Allocates and populates a MBIM_CONTEXT_STATE structure on the
    response

    @details

    @param txn
    @param qmi_rsp

    @return boolean TRUE on success, FALSE on failure
*/
/*=========================================================================*/
qbi_svc_bc_ext_cache_s* qbi_svc_bc_ext_provisioned_contexts_v2_update_cache
(
  qbi_txn_s                                   *txn,
  const wds_get_profile_settings_resp_msg_v01 *qmi_rsp
)
{
  boolean set_default = TRUE;
  qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *info = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *req = NULL;
  qbi_svc_bc_ext_cache_s *cache = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_NULL(txn);
  QBI_CHECK_NULL_PTR_RET_NULL(txn->info);
  QBI_CHECK_NULL_PTR_RET_NULL(qmi_rsp);

  info = (qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *)txn->info;

  cache = qbi_svc_bc_ext_cache_get(txn->ctx, 
      info->profile_list.profile_index[info->profiles_read - 1]);
  QBI_CHECK_NULL_PTR_RET_NULL(cache);

  QBI_LOG_D_0("Prov Cntxt V2 : Updating Cache");
  if (txn->req.data)
  {
    // This is set case. update cache.
    QBI_LOG_D_0("Cache Updating As Part Of Set Operation");
    req = (qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s*)txn->req.data;

    switch(req->operation)
    {
     case QBI_SVC_MBIM_MS_CONTEXT_OPERATION_RESTORE_FACTORY:
      cache->prov_active = FALSE; 
      cache->lte_active = FALSE;
      set_default = TRUE;
      break;
     case QBI_SVC_MBIM_MS_CONTEXT_OPERATION_DELETE:
      cache->prov_active = FALSE;
      cache->lte_active = FALSE;
      set_default = FALSE;
      break;
     case QBI_SVC_MBIM_MS_CONTEXT_OPERATION_DEFAULT:
      cache->prov_active = TRUE;
      set_default = FALSE;

      cache->source = req->source;
      switch (req->roaming)
      {
        case QBI_SVC_MBIM_MS_CONTEXT_ROAMING_CONTROL_HOME_ONLY:
           cache->roaming_flag = QBI_SVC_BC_EXT_ROAMING_FLAG_HOME;
          break;

        case QBI_SVC_MBIM_MS_CONTEXT_ROAMING_CONTROL_HOME_AND_PARTNER:
           cache->roaming_flag = QBI_SVC_BC_EXT_ROAMING_FLAG_HOME | 
                           QBI_SVC_BC_EXT_ROAMING_FLAG_PARTNER;
          break;

        case QBI_SVC_MBIM_MS_CONTEXT_ROAMING_CONTROL_HOME_AND_NON_PARTNER:
           cache->roaming_flag = QBI_SVC_BC_EXT_ROAMING_FLAG_HOME |
                           QBI_SVC_BC_EXT_ROAMING_FLAG_NON_PARTNER;
          break;

        case QBI_SVC_MBIM_MS_CONTEXT_ROAMING_CONTROL_PARTNER_ONLY:
           cache->roaming_flag = QBI_SVC_BC_EXT_ROAMING_FLAG_PARTNER;
          break;

        case QBI_SVC_MBIM_MS_CONTEXT_ROAMING_CONTROL_PARTNER_AND_NON_PARTNER:
           cache->roaming_flag = QBI_SVC_BC_EXT_ROAMING_FLAG_PARTNER |
                           QBI_SVC_BC_EXT_ROAMING_FLAG_NON_PARTNER;
          break;
        case QBI_SVC_MBIM_MS_CONTEXT_ROAMING_CONTROL_NON_PARTNER_ONLY:
           cache->roaming_flag = QBI_SVC_BC_EXT_ROAMING_FLAG_NON_PARTNER;
          break;

        case QBI_SVC_MBIM_MS_CONTEXT_ROAMING_CONTROL_ALLOW_ALL:
        default:
           cache->roaming_flag = QBI_SVC_BC_EXT_ROAMING_FLAG_HOME |
                                 QBI_SVC_BC_EXT_ROAMING_FLAG_PARTNER |
                                 QBI_SVC_BC_EXT_ROAMING_FLAG_NON_PARTNER;
          break;
      }
      cache->media_type = req->media_type;
      cache->enable = 
        qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_enable_to_mbim_enable(qmi_rsp);
      break;
    }
  }
  
  if (set_default && !cache->prov_active)
  {
    // this is query case. Update default value.
    QBI_LOG_D_0("Cache Updating As Part Of Query Operation");
    cache->prov_active = TRUE;
    cache->source = QBI_SVC_MBIM_MS_CONTEXT_SOURCE_MODEM;
    cache->media_type = QBI_SVC_MBIM_MS_CONTEXT_MEDIA_TYPE_CELLULAR;
    cache->enable = 
       qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_enable_to_mbim_enable(qmi_rsp);
  }

  qbi_svc_bc_ext_update_nv_store(txn->ctx);

  return cache;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_update_cache() */

/*===========================================================================
  FUNCTION: qbi_svc_provisioned_contexts_add_context_to_rsp
===========================================================================*/
/*!
    @brief Allocates and populates a MBIM_CONTEXT_STATE structure on the
    response

    @details

    @param txn
    @param field_desc
    @param profile_type
    @param qmi_rsp
    @param context_id

    @return boolean TRUE on success, FALSE on failure
*/
/*=========================================================================*/
static boolean qbi_svc_ext_provisioned_contexts_v2_q_add_context_to_rsp
(
  qbi_txn_s                                   *txn,
  qbi_mbim_offset_size_pair_s                 *field_desc,
  wds_profile_type_enum_v01                    profile_type,
  const wds_get_profile_settings_resp_msg_v01 *qmi_rsp,
  uint32                                       context_id
)
{
  boolean success = FALSE;
  uint32 initial_offset = 0;
  qbi_svc_bc_ext_provisioned_contexts_context_v2_s *context = NULL;
  const uint8 *context_type = NULL;
  qbi_svc_bc_ext_cache_s *cache = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(txn);
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_rsp);

  initial_offset = txn->infobuf_len_total;
  context = (qbi_svc_bc_ext_provisioned_contexts_context_v2_s *)
    qbi_txn_rsp_databuf_add_field(
      txn, field_desc, 0, sizeof(qbi_svc_bc_ext_provisioned_contexts_context_v2_s),
      NULL);
  QBI_CHECK_NULL_PTR_RET_FALSE(context);

  cache = qbi_svc_bc_ext_provisioned_contexts_v2_update_cache(txn, qmi_rsp);
  QBI_CHECK_NULL_PTR_RET_FALSE(cache);

  context_type =
    qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_to_context_type(qmi_rsp);
  QBI_CHECK_NULL_PTR_RET_FALSE(context_type);

  QBI_MEMSCPY(context->context_type, sizeof(context->context_type),
              context_type, QBI_MBIM_UUID_LEN);

  context->context_id = context_id;

  if (qmi_rsp->pdp_type_valid == TRUE || qmi_rsp->pdn_type_valid == TRUE)
  {
      qmi_rsp->pdp_type_valid == TRUE ? (context->ip_type = qmi_rsp->pdp_type) :
      (context->ip_type = qmi_rsp->pdn_type);
  }

  context->enable = cache->enable;

  switch (cache->roaming_flag)
  {
    case 1:
     context->roaming = QBI_SVC_MBIM_MS_CONTEXT_ROAMING_CONTROL_HOME_ONLY;
     break;

    case 2:
     context->roaming = QBI_SVC_MBIM_MS_CONTEXT_ROAMING_CONTROL_PARTNER_ONLY;
     break;

    case 3:
     context->roaming = QBI_SVC_MBIM_MS_CONTEXT_ROAMING_CONTROL_HOME_AND_PARTNER;
     break;

    case 4:
     context->roaming = QBI_SVC_MBIM_MS_CONTEXT_ROAMING_CONTROL_NON_PARTNER_ONLY;
     break;

    case 5:
     context->roaming = QBI_SVC_MBIM_MS_CONTEXT_ROAMING_CONTROL_HOME_AND_NON_PARTNER;
     break;

    case 6:
     context->roaming = QBI_SVC_MBIM_MS_CONTEXT_ROAMING_CONTROL_PARTNER_AND_NON_PARTNER;
     break;

    case 0:
    case 7:
    default:
     context->roaming = QBI_SVC_MBIM_MS_CONTEXT_ROAMING_CONTROL_ALLOW_ALL;
     break;
  }
  context->media_type = cache->media_type;
  context->source = cache->source;

  context->compression =
    qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_to_mbim_compression(
      profile_type, qmi_rsp);
  context->auth_protocol =
    qbi_svc_bc_ext_provisioned_contexts_v2_qmi_profile_to_mbim_auth_proto(
      profile_type, qmi_rsp);

  QBI_LOG_D_0("Cache Status :");
  QBI_LOG_D_4("context_id %d enable %d roaming %d ip type %d",
               context->context_id, context->enable, context->roaming,
               context->ip_type);
  QBI_LOG_D_4("media_type %d source %d compression %d auth_protocol %d ",
               context->media_type, context->source,context->compression,
               context->auth_protocol);
  
  /* Populate the DataBuffer - note that the same information is contained
     in different TLVs for EPC, 3GPP and 3GPP2 profiles. For example, 3GPP
     username is in TLV 0x1B (username) while the 3GPP2 one is in TLV 0x9B
     (user_id) */
  if (profile_type == WDS_PROFILE_TYPE_EPC_V01)
  {
    if (qmi_rsp->apn_name_valid &&
        !qbi_txn_rsp_databuf_add_string_ascii_to_utf16(
          txn, &context->access_string, initial_offset,
          QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES, qmi_rsp->apn_name,
          sizeof(qmi_rsp->apn_name)))
    {
      QBI_LOG_E_0("Couldn't add EPC AccessString to response!");
    }
    else if (qmi_rsp->common_user_id_valid &&
             !qbi_txn_rsp_databuf_add_string_ascii_to_utf16(
               txn, &context->username, initial_offset,
               QBI_SVC_BC_USERNAME_MAX_LEN_BYTES, qmi_rsp->common_user_id,
               sizeof(qmi_rsp->common_user_id)))
    {
      QBI_LOG_E_0("Couldn't add EPC Username to response!");
    }
    else if (qmi_rsp->common_auth_password_valid &&
             !qbi_txn_rsp_databuf_add_string_ascii_to_utf16(
               txn, &context->password, initial_offset,
               QBI_SVC_BC_PASSWORD_MAX_LEN_BYTES,
               qmi_rsp->common_auth_password,
               sizeof(qmi_rsp->common_auth_password)))
    {
      QBI_LOG_E_0("Couldn't add EPC Password to response!");
    }
    else
    {
      success = TRUE;
    }
  }
  else if (profile_type == WDS_PROFILE_TYPE_3GPP_V01 ||
           profile_type == WDS_PROFILE_TYPE_3GPP2_V01)
  {
    if (qmi_rsp->apn_name_valid &&
        !qbi_txn_rsp_databuf_add_string_ascii_to_utf16(
          txn, &context->access_string, initial_offset,
          QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES, qmi_rsp->apn_name,
          sizeof(qmi_rsp->apn_name)))
    {
      QBI_LOG_E_0("Couldn't add 3GPP/3GPP2 AccessString to response!");
    }
    else if (qmi_rsp->username_valid &&
             !qbi_txn_rsp_databuf_add_string_ascii_to_utf16(
               txn, &context->username, initial_offset,
               QBI_SVC_BC_USERNAME_MAX_LEN_BYTES, qmi_rsp->username,
               sizeof(qmi_rsp->username)))
    {
      QBI_LOG_E_0("Couldn't add 3GPP/3GPP2 Username to response!");
    }
    else if (qmi_rsp->password_valid &&
             !qbi_txn_rsp_databuf_add_string_ascii_to_utf16(
               txn, &context->password, initial_offset,
               QBI_SVC_BC_PASSWORD_MAX_LEN_BYTES,
               qmi_rsp->password, sizeof(qmi_rsp->password)))
    {
      QBI_LOG_E_0("Couldn't add 3GPP/3GPP2 Password to response!");
    }
    else
    {
      success = TRUE;
    }
  }
  else
  {
    QBI_LOG_E_1("Unexpected profile type %d", profile_type);
  }

  if (success)
  {
    /* Update the size field to include DataBuffer items */
    QBI_LOG_D_0("Response Looks Good");
    field_desc->size = txn->infobuf_len_total - initial_offset;
    success = qbi_txn_rsp_databuf_consolidate(txn);
  }

  return success;
} /* qbi_svc_ext_provisioned_contexts_v2_q_add_context_to_rsp() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_wds27_rsp_cb
===========================================================================*/
/*!
  @brief Handles a QMI_WDS_CREATE_PROFILE_RESP for
  QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 set request

  @details

  @param qmi_txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_wds27_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_create_profile_req_msg_v01 *qmi_req = NULL;
  wds_create_profile_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_profile_settings_ptrs_s profile_settings = { 0 };
  qbi_svc_bc_ext_cache_s *cache = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *req = NULL;
  qbi_svc_bc_ext_operator_config_s operator_cfg = {0};
  char apn_name[QMI_WDS_APN_NAME_MAX_V01 + 1];
  const uint8 *field;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->req.data);

  qmi_req = (wds_create_profile_req_msg_v01 *)qmi_txn->req.data;
  qmi_rsp = (wds_create_profile_resp_msg_v01 *)qmi_txn->rsp.data;
  req = (qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *)qmi_txn->parent->req.data;

  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    if (!(qmi_req->profile_type == WDS_PROFILE_TYPE_EPC_V01 &&
          qmi_rsp->resp.error == QMI_ERR_EXTENDED_INTERNAL_V01 &&
          qmi_rsp->extended_error_code_valid && qmi_rsp->extended_error_code ==
          WDS_EEC_DS_PROFILE_REG_RESULT_ERR_INVAL_PROFILE_TYPE_V01))
    {
      QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
      if (qmi_rsp->extended_error_code_valid)
      {
        QBI_LOG_E_1("Extended error code %d", qmi_rsp->extended_error_code);
      }
      qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status(
                qmi_txn->parent, qmi_rsp->resp.error,
                qmi_rsp->extended_error_code_valid, qmi_rsp->extended_error_code);
    }
    else
    {
      qmi_req = (wds_create_profile_req_msg_v01 *)
      qbi_qmi_txn_alloc_ret_req_buf( 
                qmi_txn->parent, QBI_QMI_SVC_WDS, QMI_WDS_CREATE_PROFILE_REQ_V01,
                qbi_svc_bc_ext_provisioned_contexts_v2_s_wds27_rsp_cb);
      QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

      qmi_req->profile_type = WDS_PROFILE_TYPE_3GPP_V01;

      if (!qbi_svc_bc_ext_provisioned_contexts_v2_s_get_profile_settings_ptrs_wds27(
          qmi_req, &profile_settings, WDS_PROFILE_TYPE_3GPP_V01))
      {
          QBI_LOG_E_0("Couldn't collect profile setting pointers!");
      }
      else
      {
          action = qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile(
                   qmi_txn->parent, qmi_req->profile_type,&profile_settings); /* Need to check and remove further */
      }
    }
  }
  else
  {
    // update cache
    cache = qbi_svc_bc_ext_cache_get(
            qmi_txn->ctx, qmi_rsp->profile.profile_index);
    QBI_CHECK_NULL_PTR_RET_ABORT(cache);
    cache->context_flag = QBI_SVC_BC_EXT_CONTEXT_FLAG_USER_DEFINED;

    if (req->access_string.size != 0)
    {
       field = qbi_txn_req_databuf_get_field(
                       qmi_txn->parent, &req->access_string, 0,
                       QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES);
       QBI_CHECK_NULL_PTR_RET_FALSE(field);

       (void)qbi_util_utf16_to_ascii(
                field, req->access_string.size, apn_name, sizeof(apn_name));
       if (qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_operator(apn_name))
       {
         QBI_LOG_D_0("Requested APN is Operator APN");
         qbi_nv_store_cfg_item_read(
             qmi_txn->ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_OPERATOR_CONFIG,
             &operator_cfg,sizeof(qbi_svc_bc_ext_operator_config_s));
         if ((operator_cfg.class1_disable == QBI_SVC_BC_EXT_OPERATOR_STATE_UNSET &&
              qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class1(qmi_txn->parent->ctx,apn_name) &&
              req->enable == FALSE)) 
         {
               QBI_LOG_D_0("Setting class1_disable to TRUE");
               operator_cfg.class1_disable = QBI_SVC_BC_EXT_OPERATOR_STATE_SET;
               if (!qbi_nv_store_cfg_item_write(
                    qmi_txn->ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_OPERATOR_CONFIG,
                    &operator_cfg, sizeof(qbi_svc_bc_ext_operator_config_s)))
               {
                  QBI_LOG_E_0("Couldn't save operator_config NV for class 1!!");
               }
         }
         if (operator_cfg.class2_disable == QBI_SVC_BC_EXT_OPERATOR_STATE_UNSET &&
             qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class2(apn_name) &&
             req->enable == FALSE)
         {
              QBI_LOG_D_0("Setting class2_disable to TRUE");
              operator_cfg.class2_disable = QBI_SVC_BC_EXT_OPERATOR_STATE_SET;
              if (!qbi_nv_store_cfg_item_write(
                   qmi_txn->ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_OPERATOR_CONFIG,
                   &operator_cfg, sizeof(qbi_svc_bc_ext_operator_config_s)))
              {
                 QBI_LOG_E_0("Couldn't save operator_config NV for class 2!!");
              }
         }
       }
    }
    if (qmi_txn->parent->qmi_txns_pending > 0)
    {
      QBI_LOG_D_0("Pending Transaction For Prov Cntxt V2 Set Req");
      action = QBI_SVC_ACTION_WAIT_ASYNC_RSP;
    }
    else
    {
      if (qmi_txn->parent->info != NULL)
      {
          QBI_MEM_FREE(qmi_txn->parent->info);
          qmi_txn->parent->info = NULL;
      }
      QBI_LOG_D_0("Create Completed Now Quering");
      action = qbi_svc_bc_ext_provisioned_context_v2_q_req(qmi_txn->parent);
    }
  }

  return action;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_s_wds27_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds27_req
===========================================================================*/
/*!
  @brief Handles a QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 query request

  @details

  @param txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds27_req
(
  qbi_txn_s *txn
)
{
  wds_create_profile_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_profile_settings_ptrs_s profile_settings;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  qmi_req = (wds_create_profile_req_msg_v01 *)
             qbi_qmi_txn_alloc_ret_req_buf(
             txn, QBI_QMI_SVC_WDS, QMI_WDS_CREATE_PROFILE_REQ_V01,
             qbi_svc_bc_ext_provisioned_contexts_v2_s_wds27_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  qmi_req->profile_type = WDS_PROFILE_TYPE_EPC_V01;

  if (!qbi_svc_bc_ext_provisioned_contexts_v2_s_get_profile_settings_ptrs_wds27(
       qmi_req, &profile_settings, qmi_req->profile_type))
  {
    QBI_LOG_E_0("Couldn't collect profile setting pointers!"); 
  }
  else
  {
    QBI_LOG_D_1("Building Create request for profile_type %d", qmi_req->profile_type);
    action = qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile(
             txn, qmi_req->profile_type,&profile_settings);
  }
  
  return action;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds27_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_context_v2_wds29_rsp_cb
===========================================================================*/
/*!
    @brief Delete Profile Response for a 
    QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 set request

    @details

    @param qmi_txn

    @return qbi_svc_action_e QBI_SVC_ACTION_SEND_QMI_REQ on success,
    QBI_SVC_ACTION_ABORT on failure
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_wds29_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  wds_delete_profile_resp_msg_v01 *qmi_rsp = NULL;
  uint32 *profile_index;
  qbi_svc_bc_ext_cache_s *cache = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);

  qmi_rsp = (wds_delete_profile_resp_msg_v01 *)qmi_txn->rsp.data;
  if ((qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01) &&
    (qmi_rsp->resp.error == QMI_ERR_EXTENDED_INTERNAL_V01) &&
    (qmi_rsp->extended_error_code_valid && qmi_rsp->extended_error_code ==
      (WDS_EEC_DS_PROFILE_REG_RESULT_ERR_INVAL_PROFILE_TYPE_V01 ||
      WDS_EEC_DS_PROFILE_REG_RESULT_ERR_INVAL_PROFILE_NUM_V01)))
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
    qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status(
      qmi_txn->parent, qmi_rsp->resp.error,
      qmi_rsp->extended_error_code_valid,
      qmi_rsp->extended_error_code);
  }

  profile_index = (uint32 *)qmi_txn->parent->info;

  cache = (qbi_svc_bc_ext_cache_s *)
    qbi_svc_bc_ext_cache_get(qmi_txn->ctx, *profile_index);

  if (cache != NULL)
  {
     QBI_MEMSET(cache, 0, sizeof(qbi_svc_bc_ext_cache_s));
     cache = NULL;
  }

  QBI_LOG_D_1("Profile Index %d Deleted",*profile_index);
  //This is a recursive func so need to free info as we would reallocate
  if(qmi_txn->parent->info != NULL)
  {
    QBI_MEM_FREE(qmi_txn->parent->info);
    qmi_txn->parent->info = NULL;
  }

  action = qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc_wds29_req(
      qmi_txn->parent);

  return action; 
}/* qbi_svc_bc_ext_provisioned_context_v2_wds29_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_context_v2_s_register_for_pdc_ind
===========================================================================*/
/*!
  @brief Registers for QMI_PDC_GET_SELECTED_CONFIG_IND_V01,
  QMI_PDC_DEACTIVATE_CONFIG_IND_V01, QMI_PDC_SET_SELECTED_CONFIG_IND_V01
  and QMI_PDC_ACTIVATE_CONFIG_IND_V01 indications

  @details

  @param txn

  @return boolean returns true if registeration is successful
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_provisioned_context_v2_s_register_for_pdc_ind
(
  qbi_txn_s *txn
)
{
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);

  QBI_LOG_D_0("LTE attach config:: Registering for detach/attach indications");
  if (!qbi_svc_ind_reg_dynamic(
    txn->ctx, QBI_SVC_ID_BC_EXT,
    QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2,
    QBI_QMI_SVC_PDC, QMI_PDC_GET_SELECTED_CONFIG_IND_V01,
    qbi_svc_bc_ext_provisioned_context_v2_pdc22_ind_cb, txn, NULL))
  {
    QBI_LOG_E_0("Prov Contxt V2 :: Failed to register GET SELECT CONFIG indication.");
    return FALSE;
  }
  else if (!qbi_svc_ind_reg_dynamic(
    txn->ctx, QBI_SVC_ID_BC_EXT,
    QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2,
    QBI_QMI_SVC_PDC, QMI_PDC_DEACTIVATE_CONFIG_IND_V01,
    qbi_svc_bc_ext_provisioned_context_v2_pdc2b_ind_cb, txn, NULL))
  {
    QBI_LOG_E_0("Prov Contxt V2:: Failed to register PDC DEACTIVATE indication.");
    return FALSE;
  }
  else if (!qbi_svc_ind_reg_dynamic(
    txn->ctx, QBI_SVC_ID_BC_EXT,
    QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2,
    QBI_QMI_SVC_PDC, QMI_PDC_SET_SELECTED_CONFIG_IND_V01,
    qbi_svc_bc_ext_provisioned_context_v2_pdc23_ind_cb, txn, NULL))
  {
    QBI_LOG_E_0("Prov Contxt V2:: Failed to register SET SELECT CONFIG indication.");
    return FALSE;
  }
  else if (!qbi_svc_ind_reg_dynamic(
    txn->ctx, QBI_SVC_ID_BC_EXT,
    QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2,
    QBI_QMI_SVC_PDC, QMI_PDC_ACTIVATE_CONFIG_IND_V01,
    qbi_svc_bc_ext_provisioned_context_v2_pdc27_ind_cb, txn, NULL))
  {
    QBI_LOG_E_0("Prov Contxt V2:: Failed to register PDC ACTIVATE indication.");
    return FALSE;
  }

  return TRUE;
}/* qbi_svc_bc_ext_provisioned_context_v2_s_register_for_pdc_ind() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_context_v2_factory_reset
===========================================================================*/
/*!
    @brief Request QMI_WDS_DELETE_PROFILE_REQ_V01 and
    QMI_PDC_GET_SELECTED_CONFIG_REQ_V01

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_factory_reset
(
  qbi_txn_s *txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  pdc_get_selected_config_req_msg_v01 *qmi_req_pdc22 = NULL;
  qbi_svc_bc_ext_exec_slot_config_s exec_slot_cfg = {0};
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);

  QBI_LOG_D_0("Prov Cntxt V2:: Registering for PDC indications");
  if (!qbi_svc_bc_ext_provisioned_context_v2_s_register_for_pdc_ind(txn))
  {
     action = QBI_SVC_ACTION_ABORT;
  }
  else
  {
    qbi_nv_store_cfg_item_read(txn->ctx,
        QBI_NV_STORE_CFG_ITEM_EXECUTOR_SLOT_CONFIG, &exec_slot_cfg,
        sizeof(qbi_svc_bc_ext_exec_slot_config_s));

    QBI_LOG_D_1("exec_slot_cfg.exec0_slot %d",exec_slot_cfg.exec0_slot);

    qmi_req_pdc22 = (pdc_get_selected_config_req_msg_v01 *)
         qbi_qmi_txn_alloc_ret_req_buf(
           txn, QBI_QMI_SVC_PDC, QMI_PDC_GET_SELECTED_CONFIG_REQ_V01,
           qbi_svc_bc_ext_provisioned_contexts_v2_s_pdc22_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req_pdc22);

    qmi_req_pdc22->config_type = PDC_CONFIG_TYPE_MODEM_SW_V01;
    qmi_req_pdc22->ind_token_valid = TRUE;
    qmi_req_pdc22->ind_token = PROV_V2_IND_TOKEN;
    qmi_req_pdc22->subscription_id_valid = TRUE;
    qmi_req_pdc22->subscription_id = 0;
    qmi_req_pdc22->slot_id = exec_slot_cfg.exec0_slot;
    qmi_req_pdc22->slot_id_valid = TRUE;

    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }

  return action;
} /* qbi_svc_bc_ext_provisioned_context_v2_factory_reset() */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_provisioned_context_v2_s_req
===========================================================================*/
/*!
    @brief Handles a QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 set request

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_context_v2_s_req
(
  qbi_txn_s *txn
)
{
  qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *req = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  uint32 context_type_id;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->req.data);

  cmd_in_progress_ignore_indication = TRUE;

  req = (qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *) txn->req.data;
  if (!qbi_svc_bc_sim_subscriber_ready_status_is_ready(
        txn, TRUE))
  {
    QBI_LOG_D_0("Device Not Ready Unable to Process Set Request! ");
    txn->status = QBI_MBIM_STATUS_FAILURE;
    action = QBI_SVC_ACTION_ABORT;
  }
  else if (req->operation != 
    QBI_SVC_MBIM_MS_CONTEXT_OPERATION_DEFAULT &&
    req->operation != QBI_SVC_MBIM_MS_CONTEXT_OPERATION_DELETE &&
    req->operation != QBI_SVC_MBIM_MS_CONTEXT_OPERATION_RESTORE_FACTORY)
  {
    QBI_LOG_E_0("Invalid Operation!");
    return QBI_SVC_ACTION_ABORT;
  }
  else if (req->operation == 
           QBI_SVC_MBIM_MS_CONTEXT_OPERATION_RESTORE_FACTORY)
  {
    /* This factory reset case */
    QBI_LOG_D_0("Initiating factory reset of profiles");

    action = qbi_svc_bc_ext_provisioned_context_v2_factory_reset(txn);
  }
  else
  {
    if (!qbi_svc_bc_context_type_uuid_to_id(
      req->context_type, &context_type_id))
    {
      QBI_LOG_E_0("Received unsupported ContextType!");
      txn->status = QBI_MBIM_STATUS_CONTEXT_NOT_SUPPORTED;
    }
    else
    {
      txn->info = QBI_MEM_MALLOC_CLEAR(
        sizeof(qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s));
      QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);

      QBI_LOG_I_0("Initiating Prov Context V2 Set Request");
      action = qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds2a_req(
        txn, WDS_PROFILE_TYPE_EPC_V01);
    }
  }

  QBI_LOG_D_5("Received media_type %d ip_type %d source %d operation %d enable %d as Set Req",
               req->media_type, req->ip_type, req->source, req->operation, req->enable);

  return action;
} /* qbi_svc_bc_ext_provisioned_context_v2_s_req() */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds2a_req
===========================================================================*/
/*!
    @brief Allocates and populates a QMI_WDS_GET_PROFILE_LIST_REQ request
    for a QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 set request

    @details

    @param txn
    @param profile_type

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds2a_req
(
  qbi_txn_s                *txn,
  wds_profile_type_enum_v01 profile_type
)
{
  wds_get_profile_list_req_msg_v01 *qmi_req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);

  qmi_req = (wds_get_profile_list_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(
      txn, QBI_QMI_SVC_WDS, QMI_WDS_GET_PROFILE_LIST_REQ_V01,
      qbi_svc_bc_ext_provisioned_contexts_v2_s_wds2a_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  qmi_req->profile_type_valid = TRUE;
  qmi_req->profile_type = profile_type;

  return QBI_SVC_ACTION_SEND_QMI_REQ;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds2a_req() */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_wds2a_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_GET_PROFILE_LIST_RESP for a
    QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 set request

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_wds2a_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  wds_get_profile_list_req_msg_v01 *qmi_req = NULL;
  wds_get_profile_list_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *info = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_req = (wds_get_profile_list_req_msg_v01 *) qmi_txn->req.data;
  qmi_rsp = (wds_get_profile_list_resp_msg_v01 *) qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01 &&
      !(qmi_req->profile_type == WDS_PROFILE_TYPE_EPC_V01 &&
        qmi_rsp->resp.error == QMI_ERR_EXTENDED_INTERNAL_V01 &&
        qmi_rsp->extended_error_code_valid && qmi_rsp->extended_error_code ==
          WDS_EEC_DS_PROFILE_REG_RESULT_ERR_INVAL_PROFILE_TYPE_V01))
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
    qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status(
      qmi_txn->parent, qmi_rsp->resp.error,
      qmi_rsp->extended_error_code_valid,
      qmi_rsp->extended_error_code);
  }
  else if (qmi_rsp->profile_list_len > QMI_WDS_PROFILE_LIST_MAX_V01)
  {
    QBI_LOG_E_1("Invalid number of profiles %d", qmi_rsp->profile_list_len);
  }
  else
  {
    /* Append new profile indexes to the profile list */
    if (qmi_rsp->profile_list_len > QMI_WDS_PROFILE_LIST_MAX_V01)
    {
      QBI_LOG_E_1("Invalid number of profiles %d", qmi_rsp->profile_list_len);
    }
    else
    {
      info = (qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *)
              qmi_txn->parent->info;

      qbi_svc_bc_ext_populate_profile_list(&info->profile_list, qmi_rsp);
      if (qmi_req->profile_type == WDS_PROFILE_TYPE_EPC_V01)
      {
         info->num_of_profile_epc = qmi_rsp->profile_list_len;

         qmi_req = (wds_get_profile_list_req_msg_v01 *)
                   qbi_qmi_txn_alloc_ret_req_buf(
                   qmi_txn->parent, QBI_QMI_SVC_WDS,
                   QMI_WDS_GET_PROFILE_LIST_REQ_V01,
                   qbi_svc_bc_ext_provisioned_contexts_v2_s_wds2a_rsp_cb);
         QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

         qmi_req->profile_type_valid = TRUE;
         qmi_req->profile_type = WDS_PROFILE_TYPE_3GPP_V01;
         action = QBI_SVC_ACTION_SEND_QMI_REQ;
      }
      else if (qmi_req->profile_type == WDS_PROFILE_TYPE_3GPP_V01)
      {
         info->num_of_profile_3gpp = qmi_rsp->profile_list_len;

         /* Using BC Service API to get the device capability instead of creating a new one */
         if (qbi_svc_bc_device_supports_3gpp2(qmi_txn->ctx))
         {
            qmi_req = (wds_get_profile_list_req_msg_v01 *)
                qbi_qmi_txn_alloc_ret_req_buf(
                    qmi_txn->parent, QBI_QMI_SVC_WDS,
                    QMI_WDS_GET_PROFILE_LIST_REQ_V01,
                    qbi_svc_bc_ext_provisioned_contexts_v2_s_wds2a_rsp_cb);
            QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

            qmi_req->profile_type_valid = TRUE;
            qmi_req->profile_type = WDS_PROFILE_TYPE_3GPP2_V01;
            action = QBI_SVC_ACTION_SEND_QMI_REQ;
         }
      }
      else
      {
            info->num_of_profile_3gpp2 = qmi_rsp->profile_list_len;
      }
    }
    if (action != QBI_SVC_ACTION_SEND_QMI_REQ)
    {
       QBI_LOG_D_0("Complete Get Profile List Request");
       QBI_LOG_D_3("Number Of EPC / 3GPP / 3GPP2 profiles %d / %d / %d",
                    info->num_of_profile_epc,info->num_of_profile_3gpp,
                    info->num_of_profile_3gpp2);

       action = qbi_svc_bc_ext_provisioned_contexts_v2_s_get_next_profile(
       qmi_txn->parent, qmi_txn);
    }
  }

  return action;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_s_wds2a_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_get_next_profile
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_GET_PROFILE_SETTING_REQ for a
    QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 set request

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_get_next_profile
(
  qbi_txn_s *txn,
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *req = NULL;
  wds_delete_profile_req_msg_v01 *qmi_req_del = NULL;
  wds_get_profile_settings_req_msg_v01 *qmi_req_wds2b = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *info = NULL;
  qbi_svc_bc_ext_operator_config_s operator_cfg = {0};
  const uint8 *field = NULL;
  char apn_name[QMI_WDS_APN_NAME_MAX_V01 + 1];
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;

/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->req.data);

  info = (qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *)txn->info;
  req = (qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *)txn->req.data;

  QBI_LOG_D_2("profiles read %d profile matched %d",
              info->profiles_read,info->profile_matched);
  QBI_LOG_D_3("Index Of EPC / 3GPP / 3GPP2 Profiles -> %d / %d / %d", 
               info->profile_index_epc,info->profile_index_3gpp, 
               info->profile_index_3gpp2);

  /* For the first time this will never be true because we haven't called get settings on
  each profile index yet so qmi_rsp/qmi_txn param can be ignored for first iteration */
  if (info->profile_matched ==  TRUE && (info->profiles_read >= (info->num_of_profile_epc +
      info->num_of_profile_3gpp + info->num_of_profile_3gpp2)))
  {
   QBI_LOG_I_0("Matching profile found");
   /* We want to delete an existing profile */
   if(req->operation == QBI_SVC_MBIM_MS_CONTEXT_OPERATION_DELETE)
   {
     qmi_req_del = (wds_delete_profile_req_msg_v01 *)
                   qbi_qmi_txn_alloc_ret_req_buf(
                   txn, QBI_QMI_SVC_WDS, QMI_WDS_DELETE_PROFILE_REQ_V01,
                   qbi_svc_bc_ext_provisioned_contexts_v2_s_wds29_rsp_cb);
     QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req_del);
          
     if (info->profile_found_epc == TRUE)
     {
       qmi_req_del->profile.profile_index = (uint8_t)info->profile_index_epc;
       qmi_req_del->profile.profile_type = WDS_PROFILE_TYPE_EPC_V01;
     }
     else if (info->profile_found_3gpp == TRUE)
     {
       qmi_req_del->profile.profile_index = (uint8_t)info->profile_index_3gpp;
       qmi_req_del->profile.profile_type = WDS_PROFILE_TYPE_3GPP_V01;
     }
     else if (info->profile_found_3gpp2 == TRUE)
     {
       qmi_req_del->profile.profile_index = (uint8_t)info->profile_index_3gpp2;
       qmi_req_del->profile.profile_type = WDS_PROFILE_TYPE_3GPP2_V01;
     }

     action = QBI_SVC_ACTION_SEND_QMI_REQ;
     QBI_LOG_I_2("Deleting profile at index %d type %d", qmi_req_del->profile.profile_index, qmi_req_del->profile.profile_type);

     info->operation_completed = TRUE;
     return action;
   }
   else if(req->operation == QBI_SVC_MBIM_MS_CONTEXT_OPERATION_DEFAULT) /* Request seems to either create or modify a exisiting profile */
   {
    if (info->profile_found_epc == TRUE)
    {
     QBI_LOG_D_0("Sending EPC modify request");
     action = qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds28_req(
              qmi_txn->parent, WDS_PROFILE_TYPE_EPC_V01, info->profile_index_epc);
    }
    else
    {
     /* When modify a 3gpp profile, there may be a coresponding 3gpp2
        profile to be modified as well. Read 3gpp profile settings
        before modification, to be used for finding matching 3gpp2
        profile */
        if (info->profile_found_3gpp == TRUE)
        {
         QBI_LOG_D_0("Sending 3gpp modify request");
         action = qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds28_req(
                  qmi_txn->parent, WDS_PROFILE_TYPE_3GPP_V01, info->profile_index_3gpp);
        }
        else
        {
         if (info->profile_found_3gpp2 == TRUE)
         {
          QBI_LOG_D_0("Sending 3gpp2 modify request");
          action = qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds28_req(
                   qmi_txn->parent, WDS_PROFILE_TYPE_3GPP2_V01, info->profile_index_3gpp2);
         }
        }
    }
    info->operation_completed = TRUE;
   }
  }
  else if (info->profile_matched == FALSE && (info->profiles_read >= (info->num_of_profile_epc +
           info->num_of_profile_3gpp + info->num_of_profile_3gpp2)))
  {
    if (req->operation == QBI_SVC_MBIM_MS_CONTEXT_OPERATION_DEFAULT)
    {
       QBI_LOG_I_0("Profile did not match , Creating a new one");
       if (req->access_string.size != 0)
       {
         field = qbi_txn_req_databuf_get_field(
                 qmi_txn->parent, &req->access_string, 0,
                 QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES);
         QBI_CHECK_NULL_PTR_RET_FALSE(field);

         (void)qbi_util_utf16_to_ascii(
               field, req->access_string.size, apn_name, sizeof(apn_name));
         if (qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_operator(apn_name))
         {
           QBI_LOG_D_0("APN is Operator APN");
           qbi_nv_store_cfg_item_read(
               qmi_txn->ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_OPERATOR_CONFIG,
               &operator_cfg,sizeof(qbi_svc_bc_ext_operator_config_s));

            /* We dont allow user to create if class1/2 disable is TRUE */
           if ((operator_cfg.class1_disable == QBI_SVC_BC_EXT_OPERATOR_STATE_SET &&
                qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class1(qmi_txn->parent->ctx,apn_name) &&
                req->enable == TRUE) ||
                ((operator_cfg.class2_disable == QBI_SVC_BC_EXT_OPERATOR_STATE_SET) &&
                (qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_operator(apn_name))))
           {
                QBI_LOG_D_0("Create : Unable to enable profile !! Bad request, Aborting");
                qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status(
                         qmi_txn->parent, QMI_ERR_NONE_V01,
                         0,
                         QMI_ERR_NO_FREE_PROFILE_V01);
                action = QBI_SVC_ACTION_ABORT;
                return action;
           }
         }
       }
       action = qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds27_req(txn);
    }
    else
    {
       QBI_LOG_D_1("No Profile present with modem to perform operation %d,Aborting!",
                   req->operation);
       qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status(
                   qmi_txn->parent, QMI_ERR_NONE_V01,
                   0,
                   QMI_ERR_INVALID_PROFILE_V01);
       action = QBI_SVC_ACTION_ABORT;
    }
  }
  else
  {
    QBI_LOG_D_0("Getting Next profile");
    /* Issue a query to retrieve the profile settings */
    qmi_req_wds2b = (wds_get_profile_settings_req_msg_v01 *)
                     qbi_qmi_txn_alloc_ret_req_buf(
                     txn, QBI_QMI_SVC_WDS, QMI_WDS_GET_PROFILE_SETTINGS_REQ_V01,
                     qbi_svc_bc_ext_provisioned_contexts_v2_s_wds2b_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req_wds2b);

    if (info->profiles_read < info->num_of_profile_epc)
    {
      qmi_req_wds2b->profile.profile_type = WDS_PROFILE_TYPE_EPC_V01;
      qmi_req_wds2b->profile.profile_index =
      info->profile_list.profile_index[info->profiles_read];
    }
    else if (info->profiles_read < info->num_of_profile_epc +
             info->num_of_profile_3gpp)
    {
      qmi_req_wds2b->profile.profile_type = WDS_PROFILE_TYPE_3GPP_V01;
      qmi_req_wds2b->profile.profile_index =
      info->profile_list.profile_index[info->profiles_read];
    }
    else
    {
      qmi_req_wds2b->profile.profile_type = WDS_PROFILE_TYPE_3GPP2_V01;
      qmi_req_wds2b->profile.profile_index =
      info->profile_list.profile_index[info->profiles_read];
    }

    QBI_LOG_I_2("Reading profile profile type %d at profile index %d ",
                 qmi_req_wds2b->profile.profile_type,
                 qmi_req_wds2b->profile.profile_index);
    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }

  return action;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_get_next_profile() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_wds2b_rsp_cb
===========================================================================*/
/*!
 @brief Handles a QMI_WDS_GET_PROFILE_SETTING_RESP for a
 MBIM_CID_MS_PROVISIONED_CONTEXT_V2 set request

 @details

 @param qmi_txn

 @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_wds2b_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_get_profile_settings_resp_msg_v01 *qmi_rsp = NULL;
  wds_get_profile_settings_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *info = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *req = NULL;
  qbi_svc_bc_ext_operator_config_s operator_cfg = {0};
  char apn_name[QMI_WDS_APN_NAME_MAX_V01 + 1];
  const uint8 *field = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->req.data);

  qmi_rsp = (wds_get_profile_settings_resp_msg_v01 *)qmi_txn->rsp.data;
  qmi_req = (wds_get_profile_settings_req_msg_v01 *)qmi_txn->req.data;
  req = (qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *)qmi_txn->parent->req.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Prov Context V2 : Received error code %d from QMI", qmi_rsp->resp.error);
    qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status(qmi_txn->parent, 
      qmi_rsp->resp.error, qmi_rsp->extended_error_code_valid,
      qmi_rsp->extended_error_code);

    if (QMI_ERR_EXTENDED_INTERNAL_V01 == qmi_rsp->resp.error && 
      qmi_rsp->extended_error_code_valid &&
      WDS_EEC_DS_PROFILE_REG_RESULT_ERR_INVAL_PROFILE_NUM_V01 == 
      qmi_rsp->extended_error_code)
    {
      QBI_LOG_D_0("Prov Context V2:: Bad profile. Continue to get next profile");
      info = (qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *)qmi_txn->parent->info;

      info->profiles_read++;
      action = qbi_svc_bc_ext_provisioned_contexts_v2_s_get_next_profile(qmi_txn->parent,qmi_txn);
    }
  }
  else
  {
    info = (qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *)qmi_txn->parent->info;

    info->profiles_read++;

    QBI_LOG_I_2("Set : Received profile %d/%d", info->profiles_read,
    info->num_of_profile_epc + info->num_of_profile_3gpp +
    info->num_of_profile_3gpp2);

    /* Compare APN name/PDP Type in profile with the one in response req */
    if (qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_name_matched(qmi_txn,qmi_rsp) &&
        qbi_svc_bc_ext_provisioned_contexts_v2_s_is_pdp_type_matched(qmi_txn,qmi_rsp))
    {
            
        QBI_LOG_I_1("APN and IP type match found for Modem Index %d",
                     qmi_req->profile.profile_index);
        QBI_LOG_I_3("Profile Found EPC / 3GPP / 3GPP2 -> %d / %d / %d",
                     info->profile_found_epc, info->profile_found_3gpp, info->profile_found_3gpp2);
        QBI_LOG_I_4("Profiles Read %d Number Of EPC / 3GPP / 3GPP2 Profiles -> %d / %d / %d",
                     info->profiles_read, info->num_of_profile_epc, info->num_of_profile_3gpp,
                     info->num_of_profile_3gpp2);

        if (req->access_string.size != 0)
        {
           field = qbi_txn_req_databuf_get_field(
                       qmi_txn->parent, &req->access_string, 0,
                       QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES);
           QBI_CHECK_NULL_PTR_RET_FALSE(field);

           (void)qbi_util_utf16_to_ascii(
                field, req->access_string.size, apn_name, sizeof(apn_name));

           if (qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_operator(apn_name))
           {
             QBI_LOG_D_0("APN is Operator APN");
             qbi_nv_store_cfg_item_read(
                 qmi_txn->ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_OPERATOR_CONFIG,
                 &operator_cfg,sizeof(qbi_svc_bc_ext_operator_config_s));

             /* We dont allow profile modification if class 1/2 disabled is TRUE */
             if ((operator_cfg.class1_disable == QBI_SVC_BC_EXT_OPERATOR_STATE_SET &&
                  qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class1(qmi_txn->parent->ctx,apn_name) &&
                  req->enable == TRUE) ||
                 ((operator_cfg.class2_disable == QBI_SVC_BC_EXT_OPERATOR_STATE_SET) &&
                 (qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_operator(apn_name))))
             {
                 QBI_LOG_D_0("Modify : Unable to enable profile !! Bad request, Aborting");
                 qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status(
                    qmi_txn->parent, QMI_ERR_NONE_V01,
                    0,
                    QMI_ERR_NO_FREE_PROFILE_V01);
                 action = QBI_SVC_ACTION_ABORT;
                 return action;
             }
           }
        }
        if ((info->profiles_read <= info->num_of_profile_epc) &&
             !info->profile_found_epc)
        {
            info->profile_found_epc = TRUE;
            info->profile_index_epc =
            info->profile_list.profile_index[info->profiles_read-1];

            info->profiles_read = info->num_of_profile_epc;
        }
        else if ((info->profiles_read <= info->num_of_profile_epc +
                  info->num_of_profile_3gpp) &&
                  !info->profile_found_3gpp)
        {
            info->profile_found_3gpp = TRUE;
            info->profile_index_3gpp =
            info->profile_list.profile_index[info->profiles_read-1];
            info->profiles_read =
                info->num_of_profile_epc + info->num_of_profile_3gpp;
        }
        else if ((info->profiles_read > info->num_of_profile_epc +
                  info->num_of_profile_3gpp) &&
                  !info->profile_found_3gpp2)
        {
            info->profile_found_3gpp2 = TRUE;
            info->profile_index_3gpp2 =
            info->profile_list.profile_index[info->profiles_read-1];
            info->profiles_read = info->num_of_profile_epc +
            info->num_of_profile_3gpp + info->num_of_profile_3gpp2;
        }
            
        QBI_LOG_I_0("Updated Profile Data");
        QBI_LOG_I_3("Profile Found EPC / 3GPP / 3GPP2 -> %d / %d / %d",
                     info->profile_found_epc, info->profile_found_3gpp,info->profile_found_3gpp2);
        QBI_LOG_I_4("Profiles Read %d Number Of EPC / 3GPP / 3GPP2 Profiles -> %d / %d / %d",
                     info->profiles_read, info->num_of_profile_epc, info->num_of_profile_3gpp,
                     info->num_of_profile_3gpp2);

        /* Matching profile found in modem */
        info->profile_matched = TRUE;
    }
    action = qbi_svc_bc_ext_provisioned_contexts_v2_s_get_next_profile(qmi_txn->parent,qmi_txn);
  }

  return action;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_wds2b_rsp_cb */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_is_pdp_type_matched
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_GET_PROFILE_SETTING_REQ for a
    QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 set request

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_is_pdp_type_matched
(
  qbi_qmi_txn_s                         *qmi_txn,
  wds_get_profile_settings_resp_msg_v01 *profile_settings
)
{
  boolean result = TRUE;
  qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *req = NULL;
  
/*-------------------------------------------------------------------------*/
  req = (qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *)qmi_txn->parent->req.data;
  /* Compare IP type only if apn name matches */
  if (profile_settings->pdp_type_valid &&
     (profile_settings->pdp_type == WDS_PDP_TYPE_PDP_PPP_V01 ||
     (profile_settings->pdp_type == WDS_PDP_TYPE_PDP_IPV4_V01 &&
     !QBI_SVC_BC_EXT_IPV4_REQUESTED(req->ip_type)) ||
     (profile_settings->pdp_type == WDS_PDP_TYPE_PDP_IPV6_V01 &&
     !QBI_SVC_BC_EXT_IPV6_REQUESTED(req->ip_type)) ||
     (profile_settings->pdp_type != WDS_PDP_TYPE_PDP_IPV4V6_V01 &&
     QBI_SVC_BC_EXT_MAP_DEFAULT_IP_TYPE(req->ip_type) ==
     QBI_SVC_BC_IP_TYPE_IPV4_AND_IPV6)))
  {
     QBI_LOG_I_0("PDP type does not match");
     result = FALSE;
  }

  QBI_LOG_D_1("PDP type Match result %d", result);
  
  return result;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_is_pdp_type_matched */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class2
===========================================================================*/
/*!
    @brief Handles Request whether the APN name is Operator APN or not

    @details

    @param qmi_txn

    @return BOOLEAN
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class2
(
  const char  *apn_name
)
{
  boolean class2_apn_found = FALSE;
  uint8 i = 0;
  uint8 j = 0;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(apn_name);

  for (i = 0,j = 1; i < QBI_SVC_BC_EXT_OPERATOR_APN_COL; i++)
  {
    if (QBI_STRLEN(apn_name) &&
       (!QBI_STRNCMP(apn_name,qbi_svc_bc_ext_provisioned_contexts_v2_operator_apn[j][i],
        QBI_STRLEN(apn_name))))
    {
        class2_apn_found = TRUE;
        break;
    }
  }

  QBI_LOG_D_2("class2 apn found %d apn name length %d",
               class2_apn_found,QBI_STRLEN(apn_name));
  return class2_apn_found;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class2 */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class1
===========================================================================*/
/*!
    @brief Handles Request whether the APN name is Operator APN or not

    @details

    @param qmi_txn

    @return BOOLEAN
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class1
(
  qbi_ctx_s    *ctx,
  const char   *apn_name
)
{
  boolean class1_apn_found = FALSE;
  qbi_svc_bc_spdp_cache_s cache = { 0 };
  uint8 i = 0;
  uint8 j = 0;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(ctx);
  QBI_CHECK_NULL_PTR_RET_FALSE(apn_name);

  /* Compare APN name */
  qbi_svc_bc_spdp_read_nv_store(ctx, &cache);
  for (i = 0,j = 0; i < QBI_SVC_BC_EXT_OPERATOR_APN_COL; i++)
  {
    if ((QBI_STRLEN(apn_name) &&
        (!QBI_STRNCMP(apn_name,qbi_svc_bc_ext_provisioned_contexts_v2_operator_apn[j][i],
        QBI_STRLEN(apn_name)))) ||
        (!QBI_STRNCMP(apn_name,qbi_svc_bc_ext_provisioned_contexts_v2_operator_apn[j][i],
        QBI_STRLEN(apn_name)) &&
        qbi_svc_bc_ext_compare_imsi_for_operator(cache.imsi)))
    {
      class1_apn_found = TRUE;
      break;
    }
  }

  QBI_LOG_D_2("class1 apn found %d apn name length %d", 
              class1_apn_found,QBI_STRLEN(apn_name));
  return class1_apn_found;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class1 */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_operator
===========================================================================*/
/*!
    @brief Handles Request whether the APN name is Operator APN or not

    @details

    @param qmi_txn

    @return BOOLEAN
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_operator
(
  const char  *apn_name
)
{
  uint8 i = 0;
  uint8 j = 0;
  boolean operator_apn_found = FALSE;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(apn_name);

  for (i= 0;i < QBI_SVC_BC_EXT_OPERATOR_APN_ROW;i++)
  {
    for (j=0;j < QBI_SVC_BC_EXT_OPERATOR_APN_COL;j++)
    {
      if (QBI_STRLEN(apn_name) &&
          !QBI_STRNCMP(apn_name,qbi_svc_bc_ext_provisioned_contexts_v2_operator_apn[i][j],
          QBI_STRLEN(apn_name)))
      {
         QBI_LOG_STR_1("%s is a Operator apn",apn_name);
         operator_apn_found = TRUE;
         break;
      }
    }
  }

  QBI_LOG_D_2("operator apn found %d apn name length %d",
               operator_apn_found,QBI_STRLEN(apn_name));
  return operator_apn_found;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_operator */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_name_matched
===========================================================================*/
/*!
    @brief Handles Request whether the APN name matches with that of Request
    of QBI_SVC_BC_EXT_MBIM_CID_MS_PROVISIONED_CONTEXT_V2 set request

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_name_matched
(
  qbi_qmi_txn_s                         *qmi_txn,
  wds_get_profile_settings_resp_msg_v01 *profile_settings
)
{
  qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *req = NULL;;
  wds_get_profile_settings_req_msg_v01 *qmi_req = NULL;
  const uint8 *field = NULL;
  char apn_name[QMI_WDS_APN_NAME_MAX_V01 + 1] ={0,};

  boolean apn_found = TRUE;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_txn->parent->req.data);
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_txn->req.data);
  QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings);

  req = (qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *)qmi_txn->parent->req.data;
  qmi_req = (wds_get_profile_settings_req_msg_v01 *)qmi_txn->req.data;
  /* Compare APN name */
  if (req->access_string.size != 0)
  {
    field = qbi_txn_req_databuf_get_field(
            qmi_txn->parent, &req->access_string, 0,
    QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES);
    QBI_CHECK_NULL_PTR_RET_FALSE(field);

    (void)qbi_util_utf16_to_ascii(
          field, req->access_string.size, apn_name, sizeof(apn_name));

    if ((qmi_req->profile.profile_type != WDS_PROFILE_TYPE_3GPP2_V01 &&
       (!profile_settings->apn_name_valid ||
       QBI_STRNCMP(apn_name, profile_settings->apn_name,
       sizeof(apn_name)))) ||
       (qmi_req->profile.profile_type == WDS_PROFILE_TYPE_3GPP2_V01 &&
       (!profile_settings->apn_string_valid ||
       QBI_STRNCMP(apn_name, profile_settings->apn_string,
       sizeof(apn_name)))))
    {
      QBI_LOG_I_0("APN name does not match");
      apn_found = FALSE;
    }
  }
  else if ((qmi_req->profile.profile_type != WDS_PROFILE_TYPE_3GPP2_V01 &&
            profile_settings->apn_name_valid &&
            QBI_STRLEN(profile_settings->apn_name) != 0) ||
           (qmi_req->profile.profile_type == WDS_PROFILE_TYPE_3GPP2_V01 &&
            profile_settings->apn_string_valid &&
            QBI_STRLEN(profile_settings->apn_string) != 0))
  {
    QBI_LOG_I_0("APN name does not match !");
    apn_found = FALSE;
  }

  QBI_LOG_I_2("For Profile type %d apn found is %d",
              qmi_req->profile.profile_type, apn_found);

  return apn_found;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_name_matched */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds28_req
===========================================================================*/
/*!
    @brief Allocates and populates a QMI_WDS_MODIFY_PROFILE_SETTINGS request

    @details

    @param txn
    @param profile_type

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds28_req
(
  qbi_txn_s                *txn,
  wds_profile_type_enum_v01 profile_type,
  uint32 index
)
{
  wds_modify_profile_settings_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_profile_settings_ptrs_s profile_settings;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);

  QBI_LOG_D_0("Initiating Prov Context V2 Modify Req");

  qmi_req = (wds_modify_profile_settings_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(
      txn, QBI_QMI_SVC_WDS, QMI_WDS_MODIFY_PROFILE_SETTINGS_REQ_V01,
      qbi_svc_bc_ext_provisioned_contexts_v2_s_wds28_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  qmi_req->profile.profile_index = (uint8_t)index;
  qmi_req->profile.profile_type = profile_type;

  QBI_LOG_D_2("profile_type %d profile_index %d",
               qmi_req->profile.profile_type,qmi_req->profile.profile_index);
  
  if (!qbi_svc_bc_ext_provisioned_contexts_v2_s_get_profile_settings_ptrs_wds28(
        qmi_req, &profile_settings, profile_type))
  {
    QBI_LOG_E_0("Couldn't collect profile setting pointers!");
  }
  else
  {
    action = qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile(
      txn, qmi_req->profile.profile_type,&profile_settings);
  }

  return action;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds28_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_wds28_rsp_cb
===========================================================================*/
/*!
  @brief Handles a QMI_WDS_MODIFY_PROFILE_SETTINGS_RESP for
  MBIM_CID_PROVISIONED_CONTEXTS set request

  @details

  @param qmi_txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_wds28_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_modify_profile_settings_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *info = NULL;
  wds_modify_profile_settings_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *req = NULL;
  qbi_svc_bc_ext_cache_s *cache = NULL;
  qbi_svc_bc_ext_operator_config_s operator_cfg = {0};
  char apn_name[QMI_WDS_APN_NAME_MAX_V01 + 1] = {0,};
  const uint8 *field = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->req.data);

  qmi_rsp = (wds_modify_profile_settings_resp_msg_v01 *)qmi_txn->rsp.data;
  qmi_req = (wds_modify_profile_settings_req_msg_v01 *)qmi_txn->req.data;
  req = (qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *)qmi_txn->parent->req.data;

  QBI_LOG_I_0("Rsp Callback For Modify Req");
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
    if (qmi_rsp->extended_error_code_valid)
    {
      QBI_LOG_E_1("Extended error code %d", qmi_rsp->extended_error_code);
    }
    qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status(
      qmi_txn->parent, qmi_rsp->resp.error,
      qmi_rsp->extended_error_code_valid, qmi_rsp->extended_error_code);
  }
  else
  {
    cache = qbi_svc_bc_ext_cache_get(qmi_txn->ctx, qmi_req->profile.profile_index);
    QBI_CHECK_NULL_PTR_RET_ABORT(cache);
    cache->context_flag = QBI_SVC_BC_EXT_CONTEXT_FLAG_USER_MODIFIED;

    info = (qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *)qmi_txn->parent->info;
    QBI_LOG_D_3("For Profile Type %d Found 3GPP / 3GPP2 -> %d / %d",
                 qmi_req->profile.profile_type,info->profile_found_3gpp,info->profile_found_3gpp2);
    if (qmi_req->profile.profile_type == WDS_PROFILE_TYPE_EPC_V01 && info->profile_found_3gpp)
    {
        action = qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds28_req(
                 qmi_txn->parent, WDS_PROFILE_TYPE_3GPP_V01, info->profile_index_3gpp);

        return action;
    }
    else if (qmi_req->profile.profile_type == WDS_PROFILE_TYPE_3GPP_V01 && info->profile_found_3gpp2)
    {
        action = qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds28_req(
                 qmi_txn->parent, WDS_PROFILE_TYPE_3GPP2_V01, info->profile_index_3gpp2);

        return action;
    }
    else if (qmi_req->profile.profile_type == WDS_PROFILE_TYPE_EPC_V01 && info->profile_found_3gpp2)
    {
        action = qbi_svc_bc_ext_provisioned_contexts_v2_s_build_wds28_req(
                 qmi_txn->parent, WDS_PROFILE_TYPE_3GPP2_V01, info->profile_index_3gpp2);

        return action;
    }

    if (info->operation_completed == TRUE)
    {
       if (req->access_string.size != 0)
       {
         field = qbi_txn_req_databuf_get_field(
                       qmi_txn->parent, &req->access_string, 0,
                       QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES);
         QBI_CHECK_NULL_PTR_RET_FALSE(field);

         (void)qbi_util_utf16_to_ascii(
                field, req->access_string.size, apn_name, sizeof(apn_name));

         if (qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_operator(apn_name))
         {
            QBI_LOG_D_0("Requested APN is Operator APN");
            qbi_nv_store_cfg_item_read(
               qmi_txn->ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_OPERATOR_CONFIG,
               &operator_cfg,sizeof(qbi_svc_bc_ext_operator_config_s));

            if ((operator_cfg.class1_disable == QBI_SVC_BC_EXT_OPERATOR_STATE_UNSET &&
                 qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class1(qmi_txn->parent->ctx,apn_name) &&
                 req->enable == FALSE)) 
            {
               QBI_LOG_D_0("Setting class1_disable to TRUE");
               operator_cfg.class1_disable = QBI_SVC_BC_EXT_OPERATOR_STATE_SET;
               if (!qbi_nv_store_cfg_item_write(
                      qmi_txn->ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_OPERATOR_CONFIG,
                      &operator_cfg, sizeof(qbi_svc_bc_ext_operator_config_s)))
               {
                    QBI_LOG_E_0("Couldn't save operator_config NV for class 1!!");
               }
            }
            if (operator_cfg.class2_disable == QBI_SVC_BC_EXT_OPERATOR_STATE_UNSET &&
                   qbi_svc_bc_ext_provisioned_contexts_v2_s_is_apn_class2(apn_name) &&
                   req->enable == FALSE)
            {
               QBI_LOG_D_0("Setting class2_disable to TRUE");
               operator_cfg.class2_disable = QBI_SVC_BC_EXT_OPERATOR_STATE_SET;
               if (!qbi_nv_store_cfg_item_write(
                      qmi_txn->ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_OPERATOR_CONFIG,
                      &operator_cfg, sizeof(qbi_svc_bc_ext_operator_config_s)))
               {
                    QBI_LOG_E_0("Couldn't save operator_config NV for class 2!!");
               }
            }
         }
       }
      if (qmi_txn->parent->qmi_txns_pending > 0)
      {
        action = QBI_SVC_ACTION_WAIT_ASYNC_RSP;
      }
      else
      {
        if (qmi_txn->parent->info != NULL)
        {
          QBI_MEM_FREE(qmi_txn->parent->info);
          qmi_txn->parent->info = NULL;
        }

        QBI_LOG_D_0("Completed profile modify operation,Triggering Query");
        action = qbi_svc_bc_ext_provisioned_context_v2_q_req(qmi_txn->parent);
      }
    }
  }

  return action;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_wds28_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_wds29_rsp_cb
===========================================================================*/
/*!
 @brief Delete Response for a MBIM_CID_MS_PROVISIONED_CONTEXTS_V2
 delete request

 @details

    @param qmi_txn

 @return qbi_svc_action_e QBI_SVC_ACTION_SEND_QMI_REQ on success,
 QBI_SVC_ACTION_ABORT on failure
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_wds29_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  wds_delete_profile_req_msg_v01 *qmi_req = NULL;
  wds_delete_profile_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *info = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  qmi_req = (wds_delete_profile_req_msg_v01 *)qmi_txn->req.data;
  qmi_rsp = (wds_delete_profile_resp_msg_v01 *)qmi_txn->rsp.data;

  if ((qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01) &&
     (qmi_rsp->resp.error == QMI_ERR_EXTENDED_INTERNAL_V01) &&
     (qmi_rsp->extended_error_code_valid && (qmi_rsp->extended_error_code ==
      WDS_EEC_DS_PROFILE_REG_RESULT_ERR_INVAL_PROFILE_TYPE_V01 ||
      WDS_EEC_DS_PROFILE_REG_RESULT_ERR_INVAL_PROFILE_NUM_V01)))
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
    qbi_svc_bc_ext_provisioned_contexts_v2_set_mbim_error_status(
    qmi_txn->parent, qmi_rsp->resp.error,
    qmi_rsp->extended_error_code_valid,
    qmi_rsp->extended_error_code);
  }
  else
  {
      info = (qbi_svc_bc_ext_provisioned_contexts_v2_profiles_info_s *)
              qmi_txn->parent->info;
      if (qmi_req->profile.profile_type == WDS_PROFILE_TYPE_EPC_V01 && 
          info->profile_found_3gpp == TRUE)
      {
          qmi_req = (wds_delete_profile_req_msg_v01 *)
              qbi_qmi_txn_alloc_ret_req_buf(
              qmi_txn->parent, QBI_QMI_SVC_WDS, QMI_WDS_DELETE_PROFILE_REQ_V01,
              qbi_svc_bc_ext_provisioned_contexts_v2_s_wds29_rsp_cb);
          QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

          qmi_req->profile.profile_index = (uint8_t)info->profile_index_3gpp;
          qmi_req->profile.profile_type = WDS_PROFILE_TYPE_3GPP_V01;
          action = QBI_SVC_ACTION_SEND_QMI_REQ;
          QBI_LOG_I_2("Deleting profile at index %d of type %d", 
                       qmi_req->profile.profile_index,
                       qmi_req->profile.profile_type);
          return action;
      }
      else if (qmi_req->profile.profile_type == WDS_PROFILE_TYPE_3GPP_V01 && 
               info->profile_found_3gpp2 == TRUE)
      {
          qmi_req = (wds_delete_profile_req_msg_v01 *)
              qbi_qmi_txn_alloc_ret_req_buf(
              qmi_txn->parent, QBI_QMI_SVC_WDS, QMI_WDS_DELETE_PROFILE_REQ_V01,
              qbi_svc_bc_ext_provisioned_contexts_v2_s_wds29_rsp_cb);
          QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

          qmi_req->profile.profile_index = (uint8_t)info->profile_index_3gpp2;
          qmi_req->profile.profile_type = WDS_PROFILE_TYPE_3GPP2_V01;
          action = QBI_SVC_ACTION_SEND_QMI_REQ;
          QBI_LOG_I_2("Deleting profile at index %d type %d", qmi_req->profile.profile_index, 
                       qmi_req->profile.profile_type);
          return action;
      }
      else if (qmi_req->profile.profile_type == WDS_PROFILE_TYPE_EPC_V01 && 
               info->profile_found_3gpp2 == TRUE)
      {
          qmi_req = (wds_delete_profile_req_msg_v01 *)
              qbi_qmi_txn_alloc_ret_req_buf(
                  qmi_txn->parent, QBI_QMI_SVC_WDS, QMI_WDS_DELETE_PROFILE_REQ_V01,
                  qbi_svc_bc_ext_provisioned_contexts_v2_s_wds29_rsp_cb);
          QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

          qmi_req->profile.profile_index = (uint8_t)info->profile_index_3gpp2;
          qmi_req->profile.profile_type = WDS_PROFILE_TYPE_3GPP2_V01;
          action = QBI_SVC_ACTION_SEND_QMI_REQ;
          QBI_LOG_I_2("Deleting profile at index %d type %d", qmi_req->profile.profile_index, 
                       qmi_req->profile.profile_type);
          return action;
      }

      if (info->operation_completed == TRUE)
      {
        if (qmi_txn->parent->qmi_txns_pending > 0)
        {
          action = QBI_SVC_ACTION_WAIT_ASYNC_RSP;
        }
        else
        {
          if (qmi_txn->parent->info != NULL)
          {
            QBI_MEM_FREE(qmi_txn->parent->info);
            qmi_txn->parent->info = NULL;
          }

          QBI_LOG_D_0("Completed profile delete operation,Triggering Query");
          action = qbi_svc_bc_ext_provisioned_context_v2_q_req(qmi_txn->parent);
        }
      }
  }
  
  return action;
}/* qbi_svc_bc_ext_provisioned_contexts_v2_s_wds29_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile
===========================================================================*/
/*!
 @brief Populates QMI profile settings for a MBIM_CID_PROVISIONED_CONTEXTS
 set request

 @details

 @param txn
 @param profile_type
    @param profile_index

 @return qbi_svc_action_e QBI_SVC_ACTION_SEND_QMI_REQ on success,
 QBI_SVC_ACTION_ABORT on failure
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile
(
  qbi_txn_s                                               *txn,
  wds_profile_type_enum_v01                                profile_type,
  qbi_svc_bc_ext_provisioned_contexts_v2_profile_settings_ptrs_s *profile_settings
)
{
  const qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *req = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(profile_settings);
  QBI_CHECK_NULL_PTR_RET_ABORT(profile_settings->apn_name_valid);
  QBI_CHECK_NULL_PTR_RET_ABORT(profile_settings->user_id_valid);
  QBI_CHECK_NULL_PTR_RET_ABORT(profile_settings->auth_password_valid);
  QBI_CHECK_NULL_PTR_RET_ABORT(profile_settings->app_user_data_valid);
  QBI_CHECK_NULL_PTR_RET_ABORT(profile_settings->common_apn_disabled_flag_valid);
  QBI_CHECK_NULL_PTR_RET_ABORT(profile_settings->common_apn_disabled_flag);
  QBI_CHECK_NULL_PTR_RET_ABORT(profile_settings->common_pdp_type_valid);
  QBI_CHECK_NULL_PTR_RET_ABORT(profile_settings->common_pdp_type);

  req = (const qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *)txn->req.data;

  QBI_LOG_D_1("Populate Profile for Set Req with profile type %d", profile_type);
  QBI_LOG_D_5("Received media_type %d ip_type %d source %d operation %d enable %d as Set Req",
               req->media_type, req->ip_type, req->source, req->operation, req->enable);

  if (!qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile_compression(
      req, profile_type, profile_settings) ||
      !qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile_auth_protocol(
      txn, req, profile_type, profile_settings))
  {
    QBI_LOG_E_0("Couldn't populate compression or auth protocol!");
  }
  else if (profile_type == WDS_PROFILE_TYPE_EPC_V01 ||
           profile_type == WDS_PROFILE_TYPE_3GPP_V01)
  {
    *profile_settings->apn_name_valid = TRUE;
    *profile_settings->user_id_valid = TRUE;
    *profile_settings->auth_password_valid = TRUE;
    *profile_settings->common_pdp_type_valid = TRUE;
    *profile_settings->common_pdp_type = req->ip_type;

   if (req->access_string.offset != 0 &&
       !qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_str(
       txn, &req->access_string, QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES,
       profile_settings->apn_name, QMI_WDS_APN_NAME_MAX_V01))
   {
      QBI_LOG_E_0("Couldn't populate QMI request for APN name!");
   }
   else if (req->username.offset != 0 &&
            !qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_str(
            txn, &req->username, QBI_SVC_BC_USERNAME_MAX_LEN_BYTES,
            profile_settings->user_id, QMI_WDS_USER_NAME_MAX_V01))
   {
      QBI_LOG_E_0("Couldn't populate QMI request for username!");
   }
   else if (req->password.offset != 0 &&
            !qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_str(
            txn, &req->password, QBI_SVC_BC_PASSWORD_MAX_LEN_BYTES,
            profile_settings->auth_password,
            QMI_WDS_PASSWORD_MAX_V01))
   {
      QBI_LOG_E_0("Couldn't populate QMI request for password!");
   }
   else
   {
    *profile_settings->app_user_data_valid =
            qbi_svc_bc_context_type_uuid_to_id(
            req->context_type,
            (uint32 *)profile_settings->app_user_data);
    
     action = QBI_SVC_ACTION_SEND_QMI_REQ;
    }
  }
  else if (profile_type == WDS_PROFILE_TYPE_3GPP2_V01)
  {
    *profile_settings->apn_name_valid = TRUE;
    *profile_settings->user_id_valid = TRUE;
    *profile_settings->auth_password_valid = TRUE;
    *profile_settings->common_pdp_type_valid = TRUE;
    *profile_settings->common_pdp_type = req->ip_type;

    if (req->access_string.offset != 0 &&
        !qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_str(
        txn, &req->access_string, QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES,
        profile_settings->apn_name, QMI_WDS_APN_NAME_MAX_V01))
    {
      QBI_LOG_E_0("Couldn't populate QMI request for 3GPP2 APN string!");
    }
    else if (req->username.offset != 0 &&
        !qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_str(
        txn, &req->username, QBI_SVC_BC_USERNAME_MAX_LEN_BYTES,
        profile_settings->user_id, QMI_WDS_USER_NAME_MAX_V01))
    {
      QBI_LOG_E_0("Couldn't populate QMI request for 3GPP2 username!");
    }
    else if (req->password.offset != 0 &&
        !qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_str(
        txn, &req->password, QBI_SVC_BC_PASSWORD_MAX_LEN_BYTES,
        profile_settings->auth_password, QMI_WDS_PASSWORD_MAX_V01))
    {
      QBI_LOG_E_0("Couldn't populate QMI request for 3GPP2 password!");
    }
    else
    {
      *profile_settings->app_user_data_valid =
      qbi_svc_bc_context_type_uuid_to_id(
      req->context_type,
      (uint32 *)profile_settings->app_user_data);

      action = QBI_SVC_ACTION_SEND_QMI_REQ;
    }
  }

  if (req->enable == TRUE && (profile_type != WDS_PROFILE_TYPE_3GPP2_V01))
  {
    *profile_settings->common_apn_disabled_flag_valid = TRUE;
    *profile_settings->common_apn_disabled_flag = FALSE;
  }
  else if(req->enable == FALSE && (profile_type != WDS_PROFILE_TYPE_3GPP2_V01))
  {
    *profile_settings->common_apn_disabled_flag_valid = TRUE;
    *profile_settings->common_apn_disabled_flag = TRUE;
  }
  else if (profile_type == WDS_PROFILE_TYPE_3GPP2_V01)
  {
    *profile_settings->common_apn_disabled_flag_valid = TRUE;
    *profile_settings->common_apn_disabled_flag = req->enable;

  }

  return action;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_str
===========================================================================*/
/*!
    @brief Converts a UTF-16 string from the request DataBuffer into ASCII,
    then copies it into a QMI buffer

    @details
    Ensures that the UTF-16 string was not truncated when copying into the
    ASCII buffer.

    @param txn
    @param field_desc
    @param field_max_size
    @param qmi_field
    @param qmi_field_size

    @return boolean TRUE on success, FALSE on failure
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_str
(
  qbi_txn_s                         *txn,
  const qbi_mbim_offset_size_pair_s *field_desc,
  uint32                             field_max_size,
  char                              *qmi_field,
  uint32                             qmi_field_size
)
{
  const uint8 *req_str_utf16 = NULL;
  uint32 bytes_copied = 0;
  boolean success = FALSE;
/*-------------------------------------------------------------------------*/
  req_str_utf16 = qbi_txn_req_databuf_get_field(
                  txn, field_desc, 0, field_max_size);
  QBI_CHECK_NULL_PTR_RET_FALSE(req_str_utf16);

  bytes_copied = qbi_util_utf16_to_ascii(
  req_str_utf16, field_desc->size, qmi_field, qmi_field_size);
  if (bytes_copied > qmi_field_size)
  {
    QBI_LOG_E_2("Couldn't fit entire MBIM string into QMI request! Need %d "
      "bytes, have room for %d", bytes_copied, qmi_field_size);
  }
  else
  {
    success = TRUE;
  }

  return success;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_str() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_get_profile_settings_ptrs_wds27
===========================================================================*/
/*!
  @brief Collect pointers to all relevant TLVs in
  QMI_WDS_MODIFY_PROFILE_SETTINGS_REQ for a MBIM_CID_MS_PROVISIONED_CONTEXTS_V2
  set request

  @details

  @param qmi_req
  @param profile_settings
  @param profile_type

  @return boolean
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_get_profile_settings_ptrs_wds27
(
  wds_create_profile_req_msg_v01                 *qmi_req,
  qbi_svc_bc_ext_provisioned_contexts_v2_profile_settings_ptrs_s *profile_settings,
  wds_profile_type_enum_v01                       profile_type
)
{
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_req);
  QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings);

  QBI_LOG_D_1("Get Profile Setting Ptrs For Profile Type %d",profile_type);
  if (profile_type == WDS_PROFILE_TYPE_EPC_V01)
  {
    profile_settings->apn_name_valid = &qmi_req->apn_name_valid;
    profile_settings->apn_name = qmi_req->apn_name;
    profile_settings->user_id_valid = &qmi_req->common_user_id_valid;
    profile_settings->user_id = qmi_req->common_user_id;
    profile_settings->auth_password_valid = &qmi_req->common_auth_password_valid;
    profile_settings->auth_password = qmi_req->common_auth_password;
    profile_settings->auth_protocol_valid = &qmi_req->common_auth_protocol_valid;
    profile_settings->auth_protocol = &qmi_req->common_auth_protocol;

    profile_settings->app_user_data_valid = &qmi_req->common_app_user_data_valid;
    profile_settings->app_user_data = &qmi_req->common_app_user_data;

    profile_settings->pdp_data_compression_type_valid = &qmi_req->pdp_data_compression_type_valid;
    profile_settings->pdp_data_compression_type = &qmi_req->pdp_data_compression_type;
    profile_settings->pdp_hdr_compression_type_valid = &qmi_req->pdp_hdr_compression_type_valid;

    profile_settings->pdp_hdr_compression_type = &qmi_req->pdp_hdr_compression_type;
    profile_settings->common_apn_disabled_flag_valid = &qmi_req->apn_disabled_flag_valid;
    profile_settings->common_apn_disabled_flag = &qmi_req->apn_disabled_flag;
    profile_settings->common_pdp_type_valid = &qmi_req->pdp_type_valid;
    profile_settings->common_pdp_type = &qmi_req->pdp_type;
  }
  else if (profile_type == WDS_PROFILE_TYPE_3GPP_V01)
  {
    profile_settings->apn_name_valid = &qmi_req->apn_name_valid;
    profile_settings->apn_name = qmi_req->apn_name;
    profile_settings->user_id_valid = &qmi_req->username_valid;
    profile_settings->user_id = qmi_req->username;
    profile_settings->auth_password_valid = &qmi_req->password_valid;
    profile_settings->auth_password = qmi_req->password;

    profile_settings->authentication_preference_valid = &qmi_req->authentication_preference_valid;
    profile_settings->authentication_preference = &qmi_req->authentication_preference;

    profile_settings->app_user_data_valid = &qmi_req->app_user_data_3gpp_valid;
    profile_settings->app_user_data = &qmi_req->app_user_data_3gpp;

    profile_settings->pdp_data_compression_type_valid = &qmi_req->pdp_data_compression_type_valid;
    profile_settings->pdp_data_compression_type = &qmi_req->pdp_data_compression_type;
    profile_settings->pdp_hdr_compression_type_valid = &qmi_req->pdp_hdr_compression_type_valid;
    profile_settings->pdp_hdr_compression_type = &qmi_req->pdp_hdr_compression_type;
    profile_settings->common_apn_disabled_flag_valid = &qmi_req->apn_disabled_flag_valid;
    profile_settings->common_apn_disabled_flag = &qmi_req->apn_disabled_flag;
    profile_settings->common_pdp_type_valid = &qmi_req->pdp_type_valid;
    profile_settings->common_pdp_type = &qmi_req->pdp_type;
  }
  else if (profile_type == WDS_PROFILE_TYPE_3GPP2_V01)
  {
    profile_settings->apn_name_valid = &qmi_req->apn_string_valid;
    profile_settings->apn_name = qmi_req->apn_string;
    profile_settings->user_id_valid = &qmi_req->user_id_valid;
    profile_settings->user_id = qmi_req->user_id;
    profile_settings->auth_password_valid = &qmi_req->auth_password_valid;
    profile_settings->auth_password = qmi_req->auth_password;

    profile_settings->auth_protocol_valid = &qmi_req->auth_protocol_valid;
    profile_settings->auth_protocol = &qmi_req->auth_protocol;

    profile_settings->app_user_data_valid = &qmi_req->app_user_data_3gpp2_valid;
    profile_settings->app_user_data = &qmi_req->app_user_data_3gpp2;
    profile_settings->common_apn_disabled_flag_valid = &qmi_req->apn_enabled_3gpp2_valid;
    profile_settings->common_apn_disabled_flag = &qmi_req->apn_enabled_3gpp2;
    profile_settings->common_pdp_type_valid = &qmi_req->pdn_type_valid;
    profile_settings->common_pdp_type = &qmi_req->pdn_type;
  }

  return TRUE;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_s_get_profile_settings_ptrs_wds27() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_get_profile_settings_ptrs_wds28
===========================================================================*/
/*!
  @brief Collect pointers to all relevant TLVs in
  QMI_WDS_MODIFY_PROFILE_SETTINGS_REQ for a MBIM_CID_MS_PROVISIONED_CONTEXTS_V2
  set request

  @details

  @param qmi_req
  @param profile_settings
  @param profile_type

  @return boolean
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_get_profile_settings_ptrs_wds28
(
  wds_modify_profile_settings_req_msg_v01                 *qmi_req,
  qbi_svc_bc_ext_provisioned_contexts_v2_profile_settings_ptrs_s *profile_settings,
  wds_profile_type_enum_v01                                profile_type
)
{
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_req);
  QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings);

  QBI_LOG_D_1("Prepare Common Req Struct For Profile Type %d", profile_type);
  if (profile_type == WDS_PROFILE_TYPE_EPC_V01)
  {
    profile_settings->apn_name_valid = &qmi_req->apn_name_valid;
    profile_settings->apn_name = qmi_req->apn_name;
    profile_settings->user_id_valid = &qmi_req->common_user_id_valid;
    profile_settings->user_id = qmi_req->common_user_id;
    profile_settings->auth_password_valid = &qmi_req->common_auth_password_valid;
    profile_settings->auth_password = qmi_req->common_auth_password;

    profile_settings->auth_protocol_valid = &qmi_req->common_auth_protocol_valid;
    profile_settings->auth_protocol = &qmi_req->common_auth_protocol;

    profile_settings->app_user_data_valid = &qmi_req->common_app_user_data_valid;
    profile_settings->app_user_data = &qmi_req->common_app_user_data;

    profile_settings->pdp_data_compression_type_valid = &qmi_req->pdp_data_compression_type_valid;
    profile_settings->pdp_data_compression_type = &qmi_req->pdp_data_compression_type;
    profile_settings->pdp_hdr_compression_type_valid = &qmi_req->pdp_hdr_compression_type_valid;
    profile_settings->pdp_hdr_compression_type = &qmi_req->pdp_hdr_compression_type;
    profile_settings->common_apn_disabled_flag_valid = &qmi_req->apn_disabled_flag_valid;
    profile_settings->common_apn_disabled_flag = &qmi_req->apn_disabled_flag;
    profile_settings->common_pdp_type_valid = &qmi_req->pdp_type_valid;
    profile_settings->common_pdp_type = &qmi_req->pdp_type;
  }
  else if (profile_type == WDS_PROFILE_TYPE_3GPP_V01)
  {
    profile_settings->apn_name_valid = &qmi_req->apn_name_valid;
    profile_settings->apn_name = qmi_req->apn_name;
    profile_settings->user_id_valid = &qmi_req->username_valid;
    profile_settings->user_id = qmi_req->username;
    profile_settings->auth_password_valid = &qmi_req->password_valid;
    profile_settings->auth_password = qmi_req->password;

    profile_settings->authentication_preference_valid = &qmi_req->authentication_preference_valid;
    profile_settings->authentication_preference = &qmi_req->authentication_preference;

    profile_settings->app_user_data_valid = &qmi_req->app_user_data_3gpp_valid;
    profile_settings->app_user_data = &qmi_req->app_user_data_3gpp;

    profile_settings->pdp_data_compression_type_valid = &qmi_req->pdp_data_compression_type_valid;
    profile_settings->pdp_data_compression_type = &qmi_req->pdp_data_compression_type;
    profile_settings->pdp_hdr_compression_type_valid = &qmi_req->pdp_hdr_compression_type_valid;
    profile_settings->pdp_hdr_compression_type = &qmi_req->pdp_hdr_compression_type;
    profile_settings->common_apn_disabled_flag_valid = &qmi_req->apn_disabled_flag_valid;
    profile_settings->common_apn_disabled_flag = &qmi_req->apn_disabled_flag;
    profile_settings->common_pdp_type_valid = &qmi_req->pdp_type_valid;
    profile_settings->common_pdp_type = &qmi_req->pdp_type;
  }
  else if (profile_type == WDS_PROFILE_TYPE_3GPP2_V01)
  {
    profile_settings->apn_name_valid = &qmi_req->apn_string_valid;
    profile_settings->apn_name = qmi_req->apn_string;
    profile_settings->user_id_valid = &qmi_req->user_id_valid;
    profile_settings->user_id = qmi_req->user_id;
    profile_settings->auth_password_valid = &qmi_req->auth_password_valid;
    profile_settings->auth_password = qmi_req->auth_password;

    profile_settings->auth_protocol_valid = &qmi_req->auth_protocol_valid;
    profile_settings->auth_protocol = &qmi_req->auth_protocol;

    profile_settings->app_user_data_valid = &qmi_req->app_user_data_3gpp2_valid;
    profile_settings->app_user_data = &qmi_req->app_user_data_3gpp2;
    profile_settings->common_apn_disabled_flag_valid = &qmi_req->apn_enabled_3gpp2_valid;
    profile_settings->common_apn_disabled_flag = &qmi_req->apn_enabled_3gpp2;
    profile_settings->common_pdp_type_valid = &qmi_req->pdn_type_valid;
    profile_settings->common_pdp_type = &qmi_req->pdn_type;
  }

  return TRUE;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_s_get_profile_settings_ptrs_wds28() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile_compression
===========================================================================*/
/*!
  @brief Populates the compression TLVs of a
  QMI_WDS_MODIFY_PROFILE_SETTINGS_REQ or QMI_WDS_CREATE_PROFILE_REQ for
  a MBIM_CID_MS_PROVISIONED_CONTEXTS_V2 set request

  @details

  @param req
  @param profile_type
  @param profile_settings

  @return boolean TRUE on success, FALSE on failure
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile_compression
(
  const qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s           *req,
  wds_profile_type_enum_v01                                profile_type,
  qbi_svc_bc_ext_provisioned_contexts_v2_profile_settings_ptrs_s *profile_settings
)
{
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(req);
  QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings);

  if (profile_type == WDS_PROFILE_TYPE_EPC_V01 ||
  profile_type == WDS_PROFILE_TYPE_3GPP_V01)
  {
    QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings->pdp_data_compression_type_valid);
    QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings->pdp_data_compression_type);
    QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings->pdp_hdr_compression_type_valid);
    QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings->pdp_hdr_compression_type);

    *(profile_settings->pdp_data_compression_type_valid) = TRUE;
    *(profile_settings->pdp_hdr_compression_type_valid) = TRUE;
    if (req->compression == QBI_SVC_BC_COMPRESSION_ENABLE)
    {
      *(profile_settings->pdp_data_compression_type) =
        WDS_PDP_DATA_COMPR_TYPE_MANUFACTURER_PREF_V01;
      *(profile_settings->pdp_hdr_compression_type) =
        WDS_PDP_HDR_COMPR_TYPE_MANUFACTURER_PREF_V01;
    }
    else
    {
      *(profile_settings->pdp_data_compression_type) =
        WDS_PDP_DATA_COMPR_TYPE_OFF_V01;
      *(profile_settings->pdp_hdr_compression_type) =
        WDS_PDP_HDR_COMPR_TYPE_OFF_V01;
    }
  }

  return TRUE;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile_compression() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile_auth_protocol
===========================================================================*/
/*!
  @brief Populates the authentication preference/protocol TLV of a
  QMI_WDS_MODIFY_PROFILE_SETTINGS_REQ or QMI_WDS_CREATE_PROFILE_REQ using
  the information from the MBIM_CID_MS_PROVISIONED_CONTEXTS_V2 set request

  @details

  @param txn
  @param req
  @param profile_type
  @param profile_settings

  @return boolean TRUE on success, FALSE on failure. May set txn->status
  if the request contained an invalid value
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile_auth_protocol
(
  qbi_txn_s                                               *txn,
  const qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s    *req,
  wds_profile_type_enum_v01                                profile_type,
  qbi_svc_bc_ext_provisioned_contexts_v2_profile_settings_ptrs_s *profile_settings
)
{
  boolean success = TRUE;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(txn);
  QBI_CHECK_NULL_PTR_RET_FALSE(req);
  QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings);

  if (profile_type == WDS_PROFILE_TYPE_EPC_V01)
  {
    QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings->auth_protocol_valid);
    QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings->auth_protocol);
    *(profile_settings->auth_protocol_valid) = TRUE;
    *(profile_settings->auth_protocol) =
    qbi_svc_bc_ext_connect_mbim_auth_pref_to_qmi_auth_protocol(
    req->auth_protocol);
  }
  else if (profile_type == WDS_PROFILE_TYPE_3GPP_V01)
  {
    QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings->authentication_preference_valid);
    QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings->authentication_preference);
    *(profile_settings->authentication_preference_valid) = TRUE;
    *(profile_settings->authentication_preference) =
     qbi_svc_bc_ext_connect_mbim_auth_pref_to_qmi_auth_pref(req->auth_protocol);
  }
  else if (profile_type == WDS_PROFILE_TYPE_3GPP2_V01)
  {
    QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings->auth_protocol_valid);
    QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings->auth_protocol);
    if (req->auth_protocol == QBI_SVC_BC_AUTH_PROTOCOL_NONE)
    {
       QBI_LOG_W_0("Ignoring 3GPP2 AUTH_PROTOCOL_NONE setting");
    }
    else
    {
       *(profile_settings->auth_protocol_valid) = TRUE;
       if (req->auth_protocol == QBI_SVC_BC_AUTH_PROTOCOL_CHAP ||
           req->auth_protocol == QBI_SVC_BC_AUTH_PROTOCOL_MSCHAP_V2)
        {
            *(profile_settings->auth_protocol) = WDS_PROFILE_AUTH_PROTOCOL_CHAP_V01;
        }
        else if (req->auth_protocol == QBI_SVC_BC_AUTH_PROTOCOL_PAP)
        {
            *(profile_settings->auth_protocol) = WDS_PROFILE_AUTH_PROTOCOL_PAP_V01;
        }
        else
        {
            QBI_LOG_E_1("Invalid authentication protocol %d", req->auth_protocol);
            txn->status = QBI_MBIM_STATUS_INVALID_PARAMETERS;
            success = FALSE;
        }
    }
  }
  else
  {
    QBI_LOG_E_1("Invalid profile type %d", profile_type);
    success = FALSE;
  }

  QBI_LOG_D_1("Auth Protocol Returned %d", success);
  return success;
} /* qbi_svc_bc_ext_provisioned_contexts_v2_s_populate_profile_auth_protocol() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_connect_mbim_auth_pref_to_qmi_auth_protocol
===========================================================================*/
/*!
  @brief Converts MBIM_CID_AUTH_PROTOCOL to QMI authentication protocol

  @details

  @param auth_protocol

  @return wds_profile_auth_protocol_enum_v01
*/
/*=========================================================================*/
static wds_profile_auth_protocol_enum_v01 qbi_svc_bc_ext_connect_mbim_auth_pref_to_qmi_auth_protocol
(
  uint32 auth_protocol
)
{
  wds_profile_auth_protocol_enum_v01 qmi_auth_protocol;
/*-------------------------------------------------------------------------*/
  switch (auth_protocol)
  {
    case QBI_SVC_BC_AUTH_PROTOCOL_NONE:
     qmi_auth_protocol = WDS_PROFILE_AUTH_PROTOCOL_NONE_V01;
     break;

    case QBI_SVC_BC_AUTH_PROTOCOL_PAP:
     qmi_auth_protocol = WDS_PROFILE_AUTH_PROTOCOL_PAP_V01;
     break;

    case QBI_SVC_BC_AUTH_PROTOCOL_CHAP:
    case QBI_SVC_BC_AUTH_PROTOCOL_MSCHAP_V2:
     qmi_auth_protocol = WDS_PROFILE_AUTH_PROTOCOL_CHAP_V01;
     break;

    case QBI_SVC_BC_AUTH_PROTOCOL_AUTO:
     qmi_auth_protocol = WDS_PROFILE_AUTH_PROTOCOL_PAP_CHAP_V01;
     break;

    default:
     QBI_LOG_E_1("Unrecognized Authentitication Protocol %d - using PAP or "
     "CHAP", auth_protocol);
    /* May try PAP or CHAP */
    qmi_auth_protocol = WDS_PROFILE_AUTH_PROTOCOL_PAP_CHAP_V01;
  }

  return qmi_auth_protocol;
} /* qbi_svc_bc_ext_connect_mbim_auth_pref_to_qmi_auth_protocol() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_connect_mbim_auth_pref_to_qmi_auth_pref
===========================================================================*/
/*!
  @brief Converts MBIM_CID_AUTH_PROTOCOL to QMI authentication preference
  (3GPP only)

  @details

  @param auth_protocol

  @return uint8
*/
/*=========================================================================*/
static wds_auth_pref_mask_v01 qbi_svc_bc_ext_connect_mbim_auth_pref_to_qmi_auth_pref
(
  uint32 auth_protocol
)
{
  wds_auth_pref_mask_v01 qmi_auth_pref;
/*-------------------------------------------------------------------------*/
  switch (auth_protocol)
  {
    case QBI_SVC_BC_AUTH_PROTOCOL_NONE:
     qmi_auth_pref = 0;
     break;

    case QBI_SVC_BC_AUTH_PROTOCOL_PAP:
     qmi_auth_pref = QMI_WDS_MASK_AUTH_PREF_PAP_V01;
     break;

    case QBI_SVC_BC_AUTH_PROTOCOL_CHAP:
    case QBI_SVC_BC_AUTH_PROTOCOL_MSCHAP_V2:
     qmi_auth_pref = QMI_WDS_MASK_AUTH_PREF_CHAP_V01;
     break;

    case QBI_SVC_BC_AUTH_PROTOCOL_AUTO:
     qmi_auth_pref = WDS_PROFILE_AUTH_PROTOCOL_PAP_CHAP_V01;
     break;

    default:
     QBI_LOG_E_1("Unrecognized AuthentiticationProtocol %d - using PAP or "
     "CHAP", auth_protocol);
    /* May try PAP or CHAP */
    qmi_auth_pref = (QMI_WDS_MASK_AUTH_PREF_PAP_V01 |
     QMI_WDS_MASK_AUTH_PREF_CHAP_V01);
  }

  return qmi_auth_pref;
} /* qbi_svc_bc_ext_connect_mbim_auth_pref_to_qmi_auth_pref() */

/*! @addtogroup MBIM_CID_MS_LTE_ATTACH_CONFIG
    @{ */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_qmi_profile_to_mbim_auth_proto
===========================================================================*/
/*!
    @brief Extracts the authentication protocol information from a QMI
    profile (EPC or 3GPP), if available, and returns it as an MBIM value

    @details

    @param profile_type
    @param qmi_rsp

    @return uint32 MBIM_AUTH_PROTOCOL value
*/
/*=========================================================================*/
static uint32 qbi_svc_bc_ext_qmi_profile_to_mbim_auth_proto
(
  wds_profile_type_enum_v01                    profile_type,
  const wds_get_profile_settings_resp_msg_v01 *qmi_rsp
)
{
  uint32 mbim_auth_proto = QBI_SVC_BC_AUTH_PROTOCOL_NONE;
/*-------------------------------------------------------------------------*/
  if (qmi_rsp == NULL)
  {
    QBI_LOG_E_0("Unexpected NULL pointer!");
  }
  else if (profile_type == WDS_PROFILE_TYPE_EPC_V01 &&
    qmi_rsp->common_auth_protocol_valid)
  {
    if (qmi_rsp->common_auth_protocol == WDS_PROFILE_AUTH_PROTOCOL_CHAP_V01 ||
      qmi_rsp->common_auth_protocol == WDS_PROFILE_AUTH_PROTOCOL_PAP_CHAP_V01)
    {
      mbim_auth_proto = QBI_SVC_BC_AUTH_PROTOCOL_CHAP;
    }
    else if (qmi_rsp->common_auth_protocol == WDS_PROFILE_AUTH_PROTOCOL_PAP_V01)
    {
      mbim_auth_proto = QBI_SVC_BC_AUTH_PROTOCOL_PAP;
    }
  }
  else if (profile_type == WDS_PROFILE_TYPE_3GPP_V01 &&
    qmi_rsp->authentication_preference_valid)
  {
    if (qmi_rsp->authentication_preference == QMI_WDS_MASK_AUTH_PREF_CHAP_V01)
    {
      mbim_auth_proto = QBI_SVC_BC_AUTH_PROTOCOL_CHAP;
    }
    else if (qmi_rsp->authentication_preference == QMI_WDS_MASK_AUTH_PREF_PAP_V01)
    {
      mbim_auth_proto = QBI_SVC_BC_AUTH_PROTOCOL_PAP;
    }
    else if (qmi_rsp->authentication_preference == 
            (QMI_WDS_MASK_AUTH_PREF_CHAP_V01 | QMI_WDS_MASK_AUTH_PREF_PAP_V01))
    {
      mbim_auth_proto = QBI_SVC_BC_AUTH_PROTOCOL_AUTO;
    }
  }
  else
  {
    QBI_LOG_E_1("Invalid profile type %d", profile_type);
  }

  return mbim_auth_proto;
} /* qbi_svc_bc_ext_qmi_profile_to_mbim_auth_proto() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_qmi_profile_to_mbim_compression
===========================================================================*/
/*!
    @brief Extracts the compression information from a QMI profile (3GPP or
    3GPP2), if available, and returns it as an MBIM value

    @details

    @param profile_type
    @param qmi_rsp

    @return uint32 MBIM_COMPRESSION value
*/
/*=========================================================================*/
static uint32 qbi_svc_bc_ext_qmi_profile_to_mbim_compression
(
  wds_profile_type_enum_v01                    profile_type,
  const wds_get_profile_settings_resp_msg_v01 *qmi_rsp
)
{
  uint32 mbim_compression = QBI_SVC_BC_COMPRESSION_NONE;
  /*-------------------------------------------------------------------------*/
  if (qmi_rsp == NULL)
  {
    QBI_LOG_E_0("Unexpected NULL pointer!");
  }
  else if (profile_type == WDS_PROFILE_TYPE_EPC_V01 &&
          ((qmi_rsp->pdp_data_compression_type_valid &&
          qmi_rsp->pdp_data_compression_type !=
          WDS_PDP_DATA_COMPR_TYPE_OFF_V01) ||
          (qmi_rsp->pdp_hdr_compression_type_valid &&
          qmi_rsp->pdp_hdr_compression_type !=
          WDS_PDP_HDR_COMPR_TYPE_OFF_V01)))
  {
    mbim_compression = QBI_SVC_BC_COMPRESSION_ENABLE;
  }

  return mbim_compression;
} /* qbi_svc_bc_ext_qmi_profile_to_mbim_compression() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_update_cache_ip_type
===========================================================================*/
/*!
    @brief Extracts the compression information from a QMI profile (3GPP or
    3GPP2), if available, and returns it as an MBIM value

    @details

    @param profile_type
    @param qmi_rsp

    @return uint32 MBIM_COMPRESSION value
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_update_cache_ip_type
(
  const uint32 cmd_type,
  const uint32 operation,
  qbi_svc_bc_ext_cache_s *cache,
  const qbi_svc_bc_ext_lte_attach_context_s *context,
  const qbi_svc_bc_ext_lte_attach_config_profile_settings_s profile_settings
)
{
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(cache);

  if (profile_settings.pdp_type_valid)
  {
    switch (profile_settings.pdp_type)
    {
    case WDS_PDP_TYPE_PDP_IPV4_V01:
      cache->ip_type = QBI_SVC_BC_IP_TYPE_IPV4;
      break;
    case WDS_PDP_TYPE_PDP_IPV6_V01:
      cache->ip_type = QBI_SVC_BC_IP_TYPE_IPV6;
      break;
    case WDS_PDP_TYPE_PDP_IPV4V6_V01:
      if ((cmd_type == QBI_MSG_CMD_TYPE_SET ||
           cmd_type == QBI_TXN_CMD_TYPE_INTERNAL) && 
          context != NULL)
      {
        switch (operation)
        {
        case QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_OPERATION_DEFAULT:
          cache->ip_type = context->ip_type == QBI_SVC_BC_IP_TYPE_DEFAULT ?
            QBI_SVC_BC_IP_TYPE_DEFAULT : QBI_SVC_BC_IP_TYPE_IPV4V6;
          break;
        case QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_OPERATION_RESTORE_FACTORY:
          cache->ip_type = QBI_SVC_BC_IP_TYPE_IPV4V6;
          break;
        }
      }
      else
      {
        cache->ip_type = cache->ip_type == QBI_SVC_BC_IP_TYPE_DEFAULT ?
          QBI_SVC_BC_IP_TYPE_DEFAULT : QBI_SVC_BC_IP_TYPE_IPV4V6;
      }
      break;
    default:
      QBI_LOG_E_1("LTEAttachConfig::Cache update:E: Unknown IP type: %d", 
        profile_settings.pdp_type);
      return FALSE;
    }
  }
  else
  {
    QBI_LOG_E_0("LTEAttachConfig::Cache update:E: IP type TLV missing.");
    return FALSE;
  }

  return TRUE;
} /* qbi_svc_bc_ext_update_cache_ip_type() */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_lte_attach_config_update_cache
===========================================================================*/
/*!
    @brief Updates Basic Connectivity Extension device service's cache

    @details

    @param txn
    @param qmi_rsp

    @return Pointer to the cache
*/
/*=========================================================================*/
static qbi_svc_bc_ext_cache_s* qbi_svc_bc_ext_lte_attach_config_update_cache
(
  qbi_txn_s     *txn,
  const boolean is_user_defined,
  const qbi_svc_bc_ext_lte_attach_config_profile_settings_s profile_settings
)
{
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
  qbi_mbim_offset_size_pair_s *field_desc = NULL;
  qbi_svc_bc_ext_lte_attach_context_s *context = NULL;
  qbi_svc_bc_ext_cache_s *cache = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_NULL(txn);
  QBI_CHECK_NULL_PTR_RET_NULL(txn->info);

  info = (qbi_svc_bc_ext_lte_attach_config_info_s *)txn->info;

  cache = qbi_svc_bc_ext_cache_get(txn->ctx, 
    info->profile_index[info->profiles_read]);
  QBI_CHECK_NULL_PTR_RET_NULL(cache);
  
  if (txn->cmd_type == QBI_MSG_CMD_TYPE_SET || 
      (txn->cmd_type == QBI_TXN_CMD_TYPE_INTERNAL &&
       txn->req.data != NULL))
  {
    // This is set case. update cache.
    qbi_svc_bc_ext_lte_attach_config_s_req_s *req =
      (qbi_svc_bc_ext_lte_attach_config_s_req_s*)txn->req.data;
    QBI_CHECK_NULL_PTR_RET_NULL(req);

    switch (req->operation)
    {
    case QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_OPERATION_DEFAULT:
      field_desc = (qbi_mbim_offset_size_pair_s *)((uint8_t*)req +
        sizeof(qbi_svc_bc_ext_lte_attach_config_s_req_s) +
        sizeof(qbi_mbim_offset_size_pair_s) * info->element_read);
      QBI_CHECK_NULL_PTR_RET_NULL(field_desc);

      context = (qbi_svc_bc_ext_lte_attach_context_s *)
        qbi_txn_req_databuf_get_field(txn, field_desc, 0, field_desc->size);
      QBI_CHECK_NULL_PTR_RET_FALSE(context);

      cache->source = context->source;
      break;
    case QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_OPERATION_RESTORE_FACTORY:
      cache->source = QBI_SVC_MBIM_MS_CONTEXT_SOURCE_MODEM;
      break;
    }

    qbi_svc_bc_ext_update_cache_ip_type(txn->cmd_type, req->operation,
      cache, context, profile_settings);

    if (is_user_defined)
    {
      cache->context_flag = QBI_SVC_BC_EXT_CONTEXT_FLAG_USER_DEFINED;
    }
    else
    {
      if (QBI_SVC_BC_EXT_CONTEXT_FLAG_USER_DEFINED != cache->context_flag)
      {
        cache->context_flag = QBI_SVC_BC_EXT_CONTEXT_FLAG_USER_MODIFIED;
      }
    }
  }
  else
  {
    if (!cache->lte_active)
    {
      cache->context_flag = QBI_SVC_BC_EXT_CONTEXT_FLAG_MODEM;
    }

    if (QBI_SVC_BC_EXT_CONTEXT_FLAG_MODEM == cache->context_flag)
    {
      cache->source = QBI_SVC_MBIM_MS_CONTEXT_SOURCE_MODEM;
    }

    qbi_svc_bc_ext_update_cache_ip_type(txn->cmd_type, 0, cache,
      NULL, profile_settings);
  }

  cache->lte_active = TRUE;
  cache->roaming_flag |= qbi_svc_bc_ext_roam_type_to_roam_flag(cache->roaming);

  QBI_LOG_D_6("Cache update:: ip_type: %d, source: %d, roaming: %d, lte_active: %d, "
    "context_flag: %d, roaming_flag: %d", cache->ip_type, cache->source, cache->roaming,
    cache->lte_active, cache->context_flag, cache->roaming_flag);

  qbi_svc_bc_ext_update_nv_store(txn->ctx);

  return cache;
} /* qbi_svc_bc_ext_lte_attach_config_update_cache() */

/*===========================================================================
  FUNCTION: qbi_svc_lte_attach_config_add_context_to_rsp
===========================================================================*/
/*!
    @brief Allocates and populates a MBIM_MS_LTE_ATTACH_CONTEXT structure 
    on the response

    @details

    @param txn
    @param field_desc
    @param qmi_rsp

    @return boolean TRUE on success, FALSE on failure
*/
/*=========================================================================*/
static boolean qbi_svc_lte_attach_config_add_context_to_rsp
(
  qbi_qmi_txn_s *qmi_txn
)
{
  wds_get_profile_settings_req_msg_v01 *qmi_req = NULL;
  wds_get_profile_settings_resp_msg_v01 *qmi_rsp = NULL;
  boolean success = FALSE;
  uint32 initial_offset = 0;
  qbi_mbim_offset_size_pair_s *field_desc = NULL;
  qbi_svc_bc_ext_lte_attach_context_s *context = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
  qbi_svc_bc_ext_cache_s *cache = NULL;
  qbi_svc_bc_ext_lte_attach_config_profile_settings_s profile_settings = { 0 };
	uint32 roam_type = QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_HOME;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_txn->req.data);
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_txn->rsp.data);

  qmi_req = (wds_get_profile_settings_req_msg_v01 *)qmi_txn->req.data;
  qmi_rsp = (wds_get_profile_settings_resp_msg_v01 *)qmi_txn->rsp.data;
  info = (qbi_svc_bc_ext_lte_attach_config_info_s *)qmi_txn->parent->info;

  cache = qbi_svc_bc_ext_cache_get(qmi_txn->parent->ctx, 
    qmi_req->profile.profile_index);
  QBI_CHECK_NULL_PTR_RET_ABORT(cache);

  profile_settings.roaming_disallowed_valid = qmi_rsp->roaming_disallowed_valid;
  profile_settings.roaming_disallowed = qmi_rsp->roaming_disallowed;

  profile_settings.pdp_type_valid = qmi_rsp->pdp_type_valid;
  profile_settings.pdp_type = qmi_rsp->pdp_type;

  qbi_svc_bc_ext_lte_attach_config_update_cache(
    qmi_txn->parent, FALSE, profile_settings);

	roam_type = qbi_svc_bc_ext_roam_flag_to_roam_type(
		cache->roaming_flag, info->element_read);
  QBI_LOG_D_1(" LTEAttachConfig::Q: Adding response for roaming type %d", roam_type);

  if (!info->element_match_index[roam_type])
  {
    initial_offset = qmi_txn->parent->infobuf_len_total;
    field_desc = (qbi_mbim_offset_size_pair_s *)
      ((uint8 *)qmi_txn->parent->rsp.data +
      sizeof(qbi_svc_bc_ext_lte_attach_config_info_rsp_s) +
      sizeof(qbi_mbim_offset_size_pair_s) * roam_type);
    QBI_CHECK_NULL_PTR_RET_FALSE(field_desc);

    context = (qbi_svc_bc_ext_lte_attach_context_s *)
      qbi_txn_rsp_databuf_add_field(qmi_txn->parent, field_desc, 0,
        sizeof(qbi_svc_bc_ext_lte_attach_context_s), NULL);
    QBI_CHECK_NULL_PTR_RET_FALSE(context);

    context->ip_type = cache->ip_type;
    context->source = cache->source;
    context->roaming = roam_type;

    context->compression =
      qbi_svc_bc_ext_qmi_profile_to_mbim_compression(
        WDS_PROFILE_TYPE_EPC_V01, qmi_rsp);
    context->auth_protocol =
      qbi_svc_bc_ext_qmi_profile_to_mbim_auth_proto(
        WDS_PROFILE_TYPE_3GPP_V01, qmi_rsp);

    success = TRUE;

    /* Populate the DataBuffer */
    if (qmi_rsp->apn_name_valid &&
      !qbi_txn_rsp_databuf_add_string_ascii_to_utf16(
        qmi_txn->parent, &context->access_string, initial_offset,
        QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES, qmi_rsp->apn_name,
        sizeof(qmi_rsp->apn_name)))
    {
      success = FALSE;
      QBI_LOG_E_0("Couldn't add 3GPP AccessString to response!");
    }

    if (qmi_rsp->username_valid &&
      !qbi_txn_rsp_databuf_add_string_ascii_to_utf16(
        qmi_txn->parent, &context->username, initial_offset,
        QBI_SVC_BC_USERNAME_MAX_LEN_BYTES, qmi_rsp->username,
        sizeof(qmi_rsp->username)))
    {
      success = FALSE;
      QBI_LOG_E_0("Couldn't add 3GPP Username to response!");
    }

    if (qmi_rsp->password_valid &&
      !qbi_txn_rsp_databuf_add_string_ascii_to_utf16(
        qmi_txn->parent, &context->password, initial_offset,
        QBI_SVC_BC_PASSWORD_MAX_LEN_BYTES,
        qmi_rsp->password, sizeof(qmi_rsp->password)))
    {
      success = FALSE;
      QBI_LOG_E_0("Couldn't add 3GPP Password to response!");
    }

    QBI_LOG_D_5(" LTEAttachConfig::Q: ip_type: %d, roaming: %d, source: %d, "
      "compression: %d, auth_protocol: %d", context->ip_type, context->roaming, 
      context->source, context->compression, context->auth_protocol);

    if (success)
    {
      /* Update the size field to include DataBuffer items */
      field_desc->size = qmi_txn->parent->infobuf_len_total - initial_offset;
      success = qbi_txn_rsp_databuf_consolidate(qmi_txn->parent);
      info->element_match_index[roam_type] = qmi_req->profile.profile_index;
    }
  }
  else
  {
    success = TRUE;
  }

  info->element_read++;
  info->profiles_read++;

  return success;
} /* qbi_svc_lte_attach_config_add_context_to_rsp() */

/*===========================================================================
  FUNCTION: qbi_svc_lte_attach_config_add_dummy_context_to_rsp
===========================================================================*/
/*!
    @brief Allocates and populates a MBIM_MS_LTE_ATTACH_CONTEXT structure 
    on the response

    @details

    @param txn
    @param field_desc
    @param qmi_rsp

    @return boolean TRUE on success, FALSE on failure
*/
/*=========================================================================*/
static void qbi_svc_lte_attach_config_add_dummy_context_to_rsp
(
  qbi_txn_s *txn
)
{
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
  qbi_svc_bc_ext_lte_attach_context_s *context = NULL;
  qbi_mbim_offset_size_pair_s *field_desc = NULL;
  uint32 initial_offset = 0;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET(txn);
  QBI_CHECK_NULL_PTR_RET(txn->info);

  info = (qbi_svc_bc_ext_lte_attach_config_info_s *)txn->info;

  for (info->element_read = 0;
    info->element_read < QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_MAX_PROFILE;
    info->element_read++)
  {
    if (!info->element_match_index[info->element_read])
    {
      QBI_LOG_D_1("LTEAttachConfig::Q: Adding dummy context to roam type: %d",
        info->element_read);

      field_desc = (qbi_mbim_offset_size_pair_s *)((uint8 *)txn->rsp.data +
        sizeof(qbi_svc_bc_ext_lte_attach_config_info_rsp_s) +
        sizeof(qbi_mbim_offset_size_pair_s) * info->element_read);

      initial_offset = txn->infobuf_len_total;
      context = (qbi_svc_bc_ext_lte_attach_context_s *)
        qbi_txn_rsp_databuf_add_field(txn, field_desc, 0,
          sizeof(qbi_svc_bc_ext_lte_attach_context_s), NULL);
      QBI_CHECK_NULL_PTR_RET(context);

      QBI_MEMSET(context, 0, sizeof(qbi_svc_bc_ext_lte_attach_context_s));

      context->source = QBI_SVC_MBIM_MS_CONTEXT_SOURCE_MODEM;

      switch (info->element_read)
      {
      case QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_HOME:
        context->roaming = QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_HOME;
        break;
      case QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_PARTNER:
        context->roaming = QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_PARTNER;
        break;

      case QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_NON_PARTNER:
        context->roaming = QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_NON_PARTNER;
        break;
      }

      /* Update the size field to include DataBuffer items */
      field_desc->size = txn->infobuf_len_total - initial_offset;
      qbi_txn_rsp_databuf_consolidate(txn);

      info->element_match_index[info->element_read] = 1;
    }
  }
} /* qbi_svc_lte_attach_config_add_dummy_context_to_rsp() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_q_get_next_profile
===========================================================================*/
/*!
    @brief Retrive next available configured LTE attach profile.

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_q_get_next_profile
(
  qbi_txn_s *txn
)
{
  wds_get_profile_settings_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);

  QBI_LOG_D_0("LTEAttachConfig::Q: Get next profile");
  info = (qbi_svc_bc_ext_lte_attach_config_info_s *)txn->info;

  //Don't send more that 3 response
  if ((info->element_read > 2) || (info->profiles_read >= info->num_of_profile))
  {
    qbi_svc_lte_attach_config_add_dummy_context_to_rsp(txn);
    if(QBI_TXN_CMD_TYPE_INTERNAL == txn->cmd_type)
    {
      qbi_util_buf_s buf;
      qbi_util_buf_init(&buf);
      qbi_util_buf_alloc(&buf,txn->infobuf_len_total);
      QBI_CHECK_NULL_PTR_RET_FALSE(buf.data);
      (void)qbi_txn_rsp_databuf_extract(
        txn,buf.data,buf.size,0);
      qbi_util_buf_free(&txn->rsp);
      txn->rsp = buf;
    }
    action = QBI_SVC_ACTION_SEND_RSP;
    txn->status = QBI_MBIM_STATUS_SUCCESS;
  }
  else
  {
    /* Issue a query to retrieve the profile details */
    qmi_req = (wds_get_profile_settings_req_msg_v01 *)
      qbi_qmi_txn_alloc_ret_req_buf(
        txn, QBI_QMI_SVC_WDS, QMI_WDS_GET_PROFILE_SETTINGS_REQ_V01,
        qbi_svc_bc_ext_lte_attach_config_q_wds2b_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

    QBI_LOG_D_1("LTEAttachConfig::Q: Get profile Setting for index %d",
      info->profile_index[info->profiles_read]);
    qmi_req->profile.profile_index = (uint8_t)
      info->profile_index[info->profiles_read];

    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_config_q_get_next_profile */
  
/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_lte_attach_config_q_req
===========================================================================*/
/*!
    @brief Handles a MBIM_CID_MS_LTE_ATTACH_CONFIG query request

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_q_req
(
  qbi_txn_s *txn
)
{
  wds_get_lte_max_attach_pdn_num_req_msg_v01 *qmi_req;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);

  QBI_LOG_D_0("LTEAttachConfig::Q: Query request");

  qmi_req = (wds_get_lte_max_attach_pdn_num_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(txn, QBI_QMI_SVC_WDS, 
      QMI_WDS_GET_LTE_MAX_ATTACH_PDN_NUM_REQ_V01,
      qbi_svc_bc_ext_lte_attach_config_q_wds92_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  return QBI_SVC_ACTION_SEND_QMI_REQ;
} /* qbi_svc_bc_ext_lte_attach_config_q_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_q_wds92_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_GET_LTE_MAX_ATTACH_PDN_NUM_RESP for
    MBIM_CID_MS_LTE_ATTACH_CONFIG query request

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_q_wds92_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_get_lte_attach_pdn_list_req_msg_v01 *qmi_req = NULL;
  wds_get_lte_max_attach_pdn_num_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (wds_get_lte_max_attach_pdn_num_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("LTEAttachConfig::Q: Received error code %d from QMI",
      qmi_rsp->resp.error);
  }
  else
  {
    if (!qmi_txn->parent->info)
    {
      qmi_txn->parent->info = QBI_MEM_MALLOC_CLEAR(
        sizeof(qbi_svc_bc_ext_lte_attach_config_info_s));
      QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
    }
    else
    {
      QBI_MEMSET(qmi_txn->parent->info, 0,
        sizeof(qbi_svc_bc_ext_lte_attach_config_info_s));
    }

    info = (qbi_svc_bc_ext_lte_attach_config_info_s *)qmi_txn->parent->info;
    QBI_CHECK_NULL_PTR_RET_ABORT(info);

    if (qmi_rsp->max_attach_pdn_num_valid)
    {
      info->max_supported_profile_num = qmi_rsp->max_attach_pdn_num;

      QBI_LOG_D_1("LTEAttachConfig::Q: Max Supported Profile %d",
        info->max_supported_profile_num);

      qmi_req = (wds_get_lte_attach_pdn_list_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(
          qmi_txn->parent, QBI_QMI_SVC_WDS,
          QMI_WDS_GET_LTE_ATTACH_PDN_LIST_REQ_V01,
          qbi_svc_bc_ext_lte_attach_config_q_wds94_rsp_cb);
      QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

      action = QBI_SVC_ACTION_SEND_QMI_REQ;
    }
    else
    {
      QBI_LOG_E_0("LTEAttachConfig::Q: E: max_attach_pdn_num TLV missing.");
    }
  }
  return action;
} /* qbi_svc_bc_ext_lte_attach_config_q_wds92_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_q_wds94_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_GET_LTE_ATTACH_PDN_LIST_RESP for
    MBIM_CID_MS_LTE_ATTACH_CONFIG query request

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_q_wds94_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_get_lte_attach_pdn_list_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_rsp_s *rsp;
  uint32 profile_count = 0;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  QBI_LOG_D_0("LTEAttachConfig::Q: Processing get PDN list response");
  qmi_rsp = (wds_get_lte_attach_pdn_list_resp_msg_v01 *) qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1(" LTEAttachConfig::Q: Received error code %d from QMI",
      qmi_rsp->resp.error);
  }
  else
  {
    info = (qbi_svc_bc_ext_lte_attach_config_info_s *)qmi_txn->parent->info;
    QBI_CHECK_NULL_PTR_RET_ABORT(info);
    if (qmi_rsp->attach_pdn_list_valid)
    {
      info->num_of_profile = qmi_rsp->attach_pdn_list_len;

      QBI_LOG_D_1("LTEAttachConfig::Q: No Of Profiles %d", info->num_of_profile);
      if (info->num_of_profile < info->max_supported_profile_num)
      {
        for (profile_count = 0; profile_count < info->num_of_profile; profile_count++)
        {
          info->profile_index[profile_count] =
            qmi_rsp->attach_pdn_list[profile_count];
          QBI_LOG_D_2("LTEAttachConfig::Q: profile_list[%d].index: %d",
            profile_count, info->profile_index[profile_count]);
        }

        /* Allocate the fixed-length and offset/size pair portion of the
        response now. NOTE : Only 3 are needed because we return ONLY 3
        contexts home/partner/nonpartner*/
        rsp = (qbi_svc_bc_ext_lte_attach_config_info_rsp_s *)
          qbi_txn_alloc_rsp_buf(qmi_txn->parent,
            sizeof(qbi_svc_bc_ext_lte_attach_config_info_rsp_s) +
            sizeof(qbi_mbim_offset_size_pair_s) * 
            QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_MAX_PROFILE);
        QBI_CHECK_NULL_PTR_RET_ABORT(rsp);
        rsp->element_count = QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_MAX_PROFILE;

        info->element_read = 0;
        action = qbi_svc_bc_ext_lte_attach_config_q_get_next_profile(
          qmi_txn->parent);
      }
      else
      {
        QBI_LOG_E_2("LTEAttachConfig::Q: Attach PDN list exceeds max supported "
          "PDN. attach_pdn_list_len = %d and max_supported_profile_num = %d",
          info->num_of_profile, info->max_supported_profile_num);
        action = QBI_SVC_ACTION_ABORT;
      }
    }
    else
    {
      QBI_LOG_E_0("LTEAttachConfig::Q: No LTE attac profile exists");
      action = QBI_SVC_ACTION_ABORT;
    }
  }
  return action;
} /* qbi_svc_bc_ext_lte_attach_config_q_wds94_rsp_cb() */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_lte_attach_config_q_wds2b_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_GET_PROFILE_SETTINGS_RESP for
    MBIM_CID_MS_LTE_ATTACH_CONFIG query request

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_q_wds2b_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  wds_get_profile_settings_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  QBI_LOG_D_0("LTEAttachConfig::Q: Resp Received For Get Profile Settings");
  qmi_rsp = (wds_get_profile_settings_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1(" LTEAttachConfig::Q: Received error code %d from QMI",
      qmi_rsp->resp.error);
    qbi_svc_bc_ext_lte_attach_config_set_mbim_error_status(
      qmi_txn->parent, qmi_rsp->resp.error,
      qmi_rsp->extended_error_code_valid, qmi_rsp->extended_error_code);

    action = QBI_SVC_ACTION_SEND_RSP;
  }
  else
  {
    if (qbi_svc_lte_attach_config_add_context_to_rsp(qmi_txn))
    {
      action = qbi_svc_bc_ext_lte_attach_config_q_get_next_profile(
        qmi_txn->parent);
    }
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_config_q_wds2b_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_populate_profile_auth_protocol
===========================================================================*/
/*!
  @brief Populates the authentication preference/protocol TLV of a
  QMI_WDS_CREATE_PROFILE_REQ request

  @details

  @param txn
  @param req
  @param profile_type
  @param profile_settings

  @return boolean TRUE on success, FALSE on failure. May set txn->status
  if the request contained an invalid value
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_lte_attach_config_s_populate_profile_auth_protocol
(
  const qbi_svc_bc_ext_lte_attach_context_s           *req,
  qbi_svc_bc_ext_lte_attach_config_profile_settings_s profile_settings
)
{
  boolean success = FALSE;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(req);

  *profile_settings.authentication_preference_valid = TRUE;
  *profile_settings.authentication_preference =
    qbi_svc_bc_ext_connect_mbim_auth_pref_to_qmi_auth_pref(req->auth_protocol);
  success = TRUE;

  QBI_LOG_D_3("Success %d Requested Pref %d Returned Pref %d",success,
              req->auth_protocol,*profile_settings.authentication_preference);

  return success;
} /* qbi_svc_bc_ext_lte_attach_config_s_populate_profile_auth_protocol() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_populate_profile_compression
===========================================================================*/
/*!
  @brief Populates the compression TLVs of a
  QMI_WDS_CREATE_PROFILE_REQ

  @details

  @param req
  @param profile_type
  @param profile_settings

  @return boolean TRUE on success, FALSE on failure
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_lte_attach_config_s_populate_profile_compression
(
  const qbi_svc_bc_ext_lte_attach_context_s           *req,
  qbi_svc_bc_ext_lte_attach_config_profile_settings_s profile_settings
)
{
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(req);

  *profile_settings.pdp_data_compression_type_valid = TRUE;
  *profile_settings.pdp_hdr_compression_type_valid = TRUE;
  if (req->compression == QBI_SVC_BC_COMPRESSION_ENABLE)
  {
    *profile_settings.pdp_data_compression_type =
      WDS_PDP_DATA_COMPR_TYPE_MANUFACTURER_PREF_V01;
    *profile_settings.pdp_hdr_compression_type =
      WDS_PDP_HDR_COMPR_TYPE_MANUFACTURER_PREF_V01;
  }
  else
  {
    *profile_settings.pdp_data_compression_type =
      WDS_PDP_DATA_COMPR_TYPE_OFF_V01;
    *profile_settings.pdp_hdr_compression_type =
      WDS_PDP_HDR_COMPR_TYPE_OFF_V01;
  }

  return TRUE;
} /* qbi_svc_bc_ext_lte_attach_config_s_populate_profile_compression() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_build_wds27_req
===========================================================================*/
/*!
    @brief Allocates and populates a request for QMI_WDS_CREATE_PROFILE

  @details

  @param txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_build_wds27_req
(
  qbi_txn_s *txn
)
{
  wds_create_profile_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
  qbi_svc_bc_ext_lte_attach_context_s *context = NULL;
  qbi_mbim_offset_size_pair_s* field_desc = NULL;
  qbi_svc_bc_ext_lte_attach_config_profile_settings_s profile_settings = { 0 };
  char apn_name[QMI_WDS_APN_NAME_MAX_V01 + 1] = { 0, };
  char username[QMI_WDS_USER_NAME_MAX_V01 + 1] = { 0, };
  char password[QMI_WDS_PASSWORD_MAX_V01 + 1] = { 0, };
  const uint8 *field = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->req.data);

  info = (qbi_svc_bc_ext_lte_attach_config_info_s *)txn->info;

  QBI_LOG_D_1(" LTEAttachConfig::S: Create profile for roam type %d",
    info->element_read);

  field_desc = (qbi_mbim_offset_size_pair_s *)((uint8_t*)txn->req.data +
    sizeof(qbi_svc_bc_ext_lte_attach_config_s_req_s) +
    sizeof(qbi_mbim_offset_size_pair_s) * info->element_read);
  QBI_CHECK_NULL_PTR_RET_ABORT(field_desc);

  context = (qbi_svc_bc_ext_lte_attach_context_s *)
    qbi_txn_req_databuf_get_field(txn, field_desc, 0, field_desc->size);
  QBI_CHECK_NULL_PTR_RET_ABORT(context);

  if (context->access_string.size != 0)
  {
    field = qbi_txn_req_databuf_get_field(
      txn, &context->access_string, field_desc->offset,
      QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES);
    QBI_CHECK_NULL_PTR_RET_FALSE(field);

    qbi_util_utf16_to_ascii(field, context->access_string.size,
      apn_name, sizeof(apn_name));
  }

  if (context->username.size != 0)
  {
    field = qbi_txn_req_databuf_get_field(
      txn, &context->username, field_desc->offset,
      QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES);
    QBI_CHECK_NULL_PTR_RET_FALSE(field);

    qbi_util_utf16_to_ascii(field, context->username.size,
      username, sizeof(username));
  }

  if (context->password.size != 0)
  {
    field = qbi_txn_req_databuf_get_field(
      txn, &context->password, field_desc->offset,
      QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES);
    QBI_CHECK_NULL_PTR_RET_FALSE(field);

    qbi_util_utf16_to_ascii(field, context->password.size,
      password, sizeof(password));
  }
  /* Issue create profile */
  qmi_req = (wds_create_profile_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(
      txn, QBI_QMI_SVC_WDS, QMI_WDS_CREATE_PROFILE_REQ_V01,
      qbi_svc_bc_ext_lte_attach_config_s_wds27_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  qmi_req->apn_name_valid = TRUE;
  QBI_STRLCPY(qmi_req->apn_name, apn_name, sizeof(qmi_req->apn_name));

  qmi_req->username_valid = TRUE;
  QBI_STRLCPY(qmi_req->username, username, sizeof(qmi_req->username));

  qmi_req->password_valid = TRUE;
  QBI_STRLCPY(qmi_req->password, password, sizeof(qmi_req->password));

  qmi_req->pdp_type_valid = TRUE;
  qmi_req->pdp_type = qbi_svc_bc_ext_ip_type_to_pdp_type(context->ip_type);

  profile_settings.pdp_data_compression_type_valid = 
    &qmi_req->pdp_data_compression_type_valid;
  profile_settings.pdp_data_compression_type =
    &qmi_req->pdp_data_compression_type;
  profile_settings.pdp_hdr_compression_type_valid =
    &qmi_req->pdp_hdr_compression_type_valid;
  profile_settings.pdp_hdr_compression_type =
    &qmi_req->pdp_hdr_compression_type;
  profile_settings.authentication_preference_valid = 
    &qmi_req->authentication_preference_valid;
  profile_settings.authentication_preference =
    &qmi_req->authentication_preference;

  if (!qbi_svc_bc_ext_lte_attach_config_s_populate_profile_compression(
    context, profile_settings) ||
    !qbi_svc_bc_ext_lte_attach_config_s_populate_profile_auth_protocol(
      context, profile_settings))
  {
    QBI_LOG_E_0("LTEAttachConfig::S: Couldn't populate compression or auth protocol!");
  }

  return QBI_SVC_ACTION_SEND_QMI_REQ;
} /* qbi_svc_bc_ext_lte_attach_config_s_build_wds27_req */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_get_next_profile
===========================================================================*/
/*!
    @brief Retrive next available configured LTE attach profile.

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_get_next_profile
(
  qbi_txn_s *txn
)
{
  wds_get_profile_settings_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->ctx);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->req.data);

  info = (qbi_svc_bc_ext_lte_attach_config_info_s *)txn->info;

  QBI_LOG_D_2("LTEAttachConfig::S: Profiles Read %d/%d", 
    info->profiles_read, info->num_of_profile);
  if (info->profiles_read < info->num_of_profile)
  {
    /* Issue a query to retrieve the profile details */
    qmi_req = (wds_get_profile_settings_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(
        txn, QBI_QMI_SVC_WDS, QMI_WDS_GET_PROFILE_SETTINGS_REQ_V01,
        qbi_svc_bc_ext_lte_attach_config_s_wds2b_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

    qmi_req->profile.profile_index =
      (uint8_t)info->profile_index[info->profiles_read];

    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }
  else if (qbi_svc_bc_ext_lte_attach_config_s_is_profile_match_pending(info))
  {
    action = qbi_svc_bc_ext_lte_attach_config_s_build_wds27_req(txn);
  }
  else
  {
    QBI_LOG_D_0("LTEAttachConfig::S: Registering for detach/attach indications");
    if (!qbi_svc_bc_ext_lte_attach_config_s_register_for_detach_attach(txn))
    {
      action = QBI_SVC_ACTION_ABORT;
    }
    else
    {
      action = qbi_svc_bc_ext_lte_attach_config_s_build_wds93_req(txn);
    }
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_config_s_get_next_profile */
  
/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_input_validation
===========================================================================*/
/*!
    @brief Peroforms input validation

    @details

    @param req

    @return boolean
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_lte_attach_config_s_input_validation
(
  qbi_txn_s *txn
)
{
  qbi_svc_bc_ext_lte_attach_config_s_req_s *req = NULL;
  qbi_svc_bc_ext_lte_attach_context_s *context = NULL;
  qbi_mbim_offset_size_pair_s *field_desc = NULL;
  uint32 roam_type = QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_HOME;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(txn);
  QBI_CHECK_NULL_PTR_RET_FALSE(txn->req.data);

  req = (qbi_svc_bc_ext_lte_attach_config_s_req_s *)txn->req.data;

  if (req->element_count == QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_MAX_PROFILE)
  {
    for (roam_type = QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_ROAMING_CONTROL_HOME;
      roam_type < QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_MAX_PROFILE; roam_type++)
    {
      field_desc = (qbi_mbim_offset_size_pair_s *)((uint8_t*)req +
        sizeof(qbi_svc_bc_ext_lte_attach_config_s_req_s) +
        sizeof(qbi_mbim_offset_size_pair_s) * roam_type);
      QBI_CHECK_NULL_PTR_RET_FALSE(field_desc);

      context = (qbi_svc_bc_ext_lte_attach_context_s *)
        qbi_txn_req_databuf_get_field(txn, field_desc, 0,
          field_desc->size);
      QBI_CHECK_NULL_PTR_RET_FALSE(context);

      if (context->roaming != roam_type)
      {
        QBI_LOG_E_2("LTEAttachConfig::S: E: Invalid roam type (%d) for element (%d)", 
          context->roaming, roam_type);
        return FALSE;
      }
    }
  }
  else
  {
    QBI_LOG_E_0("LTEAttachConfig::S: E: Invalid element count");
    return FALSE;
  }

  return TRUE;
} /* qbi_svc_bc_ext_lte_attach_config_s_input_validation() */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_req
===========================================================================*/
/*!
    @brief Handles a MBIM_CID_MS_LTE_ATTACH_CONFIG set request

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_req
(
  qbi_txn_s *txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  qbi_svc_bc_ext_lte_attach_config_s_req_s *req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->req.data);

  cmd_in_progress_ignore_indication = TRUE;
  
  req = (qbi_svc_bc_ext_lte_attach_config_s_req_s *)txn->req.data;
  QBI_LOG_D_2("LTEAttachConfig::S: operation = %d, element_count = %d", 
    req->operation, req->element_count);

  /* Allocate the fixed-length and offset/size pair portion of the
  response now. */
  txn->info = QBI_MEM_MALLOC_CLEAR(
    sizeof(qbi_svc_bc_ext_lte_attach_config_info_s));
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);

  switch(req->operation) 
  {
  case QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_OPERATION_RESTORE_FACTORY:
    action = qbi_svc_bc_ext_lte_attach_config_s_operation_factory_restore(txn);
    break;
  case QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_OPERATION_DEFAULT:
    if (qbi_svc_bc_ext_lte_attach_config_s_input_validation(txn))
    {
      action = qbi_svc_bc_ext_lte_attach_config_s_operation_default(txn);
    }
    break;
  default:
    QBI_LOG_E_1("Operation %d not supported", req->operation);
    txn->status = QBI_MBIM_STATUS_INVALID_PARAMETERS;
    break;
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_config_s_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_build_wds94_req
===========================================================================*/
/*!
  @brief Handles a QMI_WDS_GET_LTE_ATTACH_PDN_LIST_REQ_V01 for
    MBIM_CID_MS_LTE_ATTACH_CONFIG set request

  @details

  @param qmi_txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_build_wds94_req
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_get_lte_attach_pdn_list_req_msg_v01 *qmi_req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);

  QBI_LOG_D_0("LTEAttachConfig::S: Getting PDN list");
  qmi_req = (wds_get_lte_attach_pdn_list_req_msg_v01 *)
  qbi_qmi_txn_alloc_ret_req_buf(
      qmi_txn->parent, QBI_QMI_SVC_WDS, 
      QMI_WDS_GET_LTE_ATTACH_PDN_LIST_REQ_V01,
      qbi_svc_bc_ext_lte_attach_config_s_wds94_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  action = QBI_SVC_ACTION_SEND_QMI_REQ;

  return action;
}/* qbi_svc_bc_ext_lte_attach_config_s_build_wds94_req */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_wds92_rsp_cb
===========================================================================*/
/*!
  @brief Handles a QMI_WDS_GET_LTE_MAX_ATTACH_PDN_NUM_RESP for
    MBIM_CID_MS_LTE_ATTACH_CONFIG set request

  @details

  @param qmi_txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds92_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_get_lte_max_attach_pdn_num_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (wds_get_lte_max_attach_pdn_num_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1(" LTEAttachConfig::Q: Received error code %d from QMI",
      qmi_rsp->resp.error);
    cmd_in_progress_ignore_indication = FALSE;
  }
  else
  {
    info = (qbi_svc_bc_ext_lte_attach_config_info_s *)qmi_txn->parent->info;
    QBI_CHECK_NULL_PTR_RET_ABORT(info);
    info->max_supported_profile_num = qmi_rsp->max_attach_pdn_num;

    QBI_LOG_D_1("LTEAttachConfig::S: max_supported_profile_num = %d",
      info->max_supported_profile_num);

    action = qbi_svc_bc_ext_lte_attach_config_s_build_wds94_req(qmi_txn);
  }
  return action;
} /* qbi_svc_bc_ext_lte_attach_config_s_wds92_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_register_for_detach_attach
===========================================================================*/
/*!
  @brief Registers for QMI_WDS_LTE_ATTACH_PDN_LIST_IND and
  QMI_NAS_SYSTEM_SELECTION_PREFERENCE_IND indications

  @details

  @param txn

  @return boolean returns true if registeration is successful
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_lte_attach_config_s_register_for_detach_attach
(
  qbi_txn_s *txn
)
{
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);

  QBI_LOG_D_0("LTEAttachConfig::S: Registering for detach/attach indications");
  if (!qbi_svc_ind_reg_dynamic(
    txn->ctx, QBI_SVC_ID_BC_EXT,
    QBI_SVC_BC_EXT_MBIM_CID_MS_LTE_ATTACH_CONFIG,
    QBI_QMI_SVC_WDS, QMI_WDS_LTE_ATTACH_PDN_LIST_IND_V01,
    qbi_svc_bc_ext_lte_attach_config_s_wds95_ind_cb, txn, NULL))
  {
    QBI_LOG_E_0("LTEAttachConfig::S: Failed to register PDN list indication.");
    return FALSE;
  }
  else if (!qbi_svc_ind_reg_dynamic(
    txn->ctx, QBI_SVC_ID_BC_EXT,
    QBI_SVC_BC_EXT_MBIM_CID_MS_LTE_ATTACH_CONFIG,
    QBI_QMI_SVC_NAS, QMI_NAS_SYSTEM_SELECTION_PREFERENCE_IND_MSG_V01,
    qbi_svc_bc_ext_lte_attach_config_s_nas34_ind_cb, txn, NULL))
  {
    QBI_LOG_E_0("LTEAttachConfig::S: Failed to register SSP indication.");
    return FALSE;
  }

  return TRUE;
} /* qbi_svc_bc_ext_lte_attach_config_s_register_for_detach_attach() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_add_profile_list_from_cache
===========================================================================*/
/*!
  @brief Allocates and populates a QMI_WDS_DELETE_PROFILE_REQ request

  @details

  @param txn

  @return boolean
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_lte_attach_config_s_add_profile_list_from_cache
(
  qbi_txn_s *txn
)
{
  qbi_svc_bc_ext_cache_s *cache = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
  uint32 i = 0;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(txn);
  QBI_CHECK_NULL_PTR_RET_FALSE(txn->info);

  info = (qbi_svc_bc_ext_lte_attach_config_info_s *)txn->info;

  for (i = 0; i < QMI_WDS_PROFILE_LIST_MAX_V01; i++)
  {
    cache = qbi_svc_bc_ext_cache_get(txn->ctx, i);
    QBI_CHECK_NULL_PTR_RET_FALSE(cache);

    if (cache->lte_active && info->num_of_profile < QMI_WDS_ATTACH_PDN_MAX_V01)
    {
      info->profile_index[info->num_of_profile++] = i;
    }
  }

  if (!info->num_of_profile)
  {
    QBI_LOG_E_0("LTEAttachConfig::S: E:No LTE attach profile exists.");
    return FALSE;
  }

  return TRUE;
} /* qbi_svc_bc_ext_lte_attach_config_s_add_profile_list_from_cache() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_add_profile_list_from_modem
===========================================================================*/
/*!
  @brief Allocates and populates a QMI_WDS_DELETE_PROFILE_REQ request

  @details

  @param txn

  @return boolean
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_lte_attach_config_s_add_profile_list_from_modem
(
  qbi_txn_s *txn,
  uint32    modem_pdn_list_len,
  uint16    *modem_pdn_list
)
{
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
  uint32 list_size = 0;
  uint32 i = 0;
  uint32 j = 0;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(txn);
  QBI_CHECK_NULL_PTR_RET_FALSE(txn->info);

  info = (qbi_svc_bc_ext_lte_attach_config_info_s *)txn->info;

  QBI_LOG_D_2("LTEAttachConfig::S: Modem PDN List Len %d Num Of Profile %d",
              modem_pdn_list_len,info->num_of_profile);

  if (!info->num_of_profile)
  {
    for (i = 0; i < modem_pdn_list_len; i++)
    {
      info->profile_index[j] = modem_pdn_list[i];
      info->num_of_profile++;

      QBI_LOG_D_1("LTEAttachConfig::S: Added Profile index %d to list.",
        info->profile_index[j]);
    }
  }
  else
  {
    for (i = 0; i < modem_pdn_list_len; i++)
    {
      for (j = 0; j < info->num_of_profile; j++)
      {
        // if the index match, do not add to list.
        if (modem_pdn_list[i] == info->profile_index[j])
        {
          QBI_LOG_D_1("LTEAttachConfig::S: Profile index %d, exists in cache.",
            modem_pdn_list[i]);
          break;
        }
        // List retrieved from cache is sorted. if index is lesser than the 
        // cache index, insert profile index to the list in sorted order.
        else if (modem_pdn_list[i] < info->profile_index[j])
        {
          if (info->num_of_profile < info->max_supported_profile_num)
          {
            list_size = sizeof(uint8_t) * (info->num_of_profile - j);

            QBI_MEMSCPY(&info->profile_index[j + 1], list_size,
              &info->profile_index[j], list_size);

            info->profile_index[j] = modem_pdn_list[i];
            info->num_of_profile++;

            QBI_LOG_D_1("LTEAttachConfig::S: Added Profile index %d to list.",
              info->profile_index[j]);
            break;
          }
          else
          {
            QBI_LOG_E_2("LTEAttachConfig::S: num_of_profile(%d) exceeds "
              "max_supported_profile_num (%d). Skipping profile add but "
              "continue comparision", info->num_of_profile,
              info->max_supported_profile_num);
            break;
          }
        }
      }
    }
  }

  return TRUE;
} /* qbi_svc_bc_ext_lte_attach_config_s_add_profile_list_from_modem() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_get_profile_list
===========================================================================*/
/*!
  @brief Allocates and populates a QMI_WDS_DELETE_PROFILE_REQ request

  @details

  @param txn

  @return boolean
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_lte_attach_config_s_get_profile_list
(
  qbi_txn_s *txn,
  wds_get_lte_attach_pdn_list_resp_msg_v01 *qmi_rsp
)
{
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
  qbi_svc_bc_ext_lte_attach_config_s_req_s *req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(txn);
  QBI_CHECK_NULL_PTR_RET_FALSE(txn->req.data);
  QBI_CHECK_NULL_PTR_RET_FALSE(txn->info);
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_rsp);
  
  req = (qbi_svc_bc_ext_lte_attach_config_s_req_s *)txn->req.data;
  info = (qbi_svc_bc_ext_lte_attach_config_info_s *)txn->info;

  // 0th profile is for 3GPP2. Skip for LTE profiles.
  qbi_svc_bc_ext_lte_attach_config_s_add_profile_list_from_cache(txn);
  QBI_LOG_D_1("LTEAttachConfig::S: No. of profiles from cache list: %d",
    info->num_of_profile);

  if (QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_OPERATION_DEFAULT == req->operation)
  {
    // Add active LTE profiles.
    if (qmi_rsp->attach_pdn_list_valid)
    {
      qbi_svc_bc_ext_lte_attach_config_s_add_profile_list_from_modem(
        txn, qmi_rsp->attach_pdn_list_len, qmi_rsp->attach_pdn_list);
      QBI_LOG_D_1("LTEAttachConfig::S: No. of profiles from active list: %d",
        info->num_of_profile);
    }

    // Add pending LTE profiles.
    if (qmi_rsp->pending_attach_pdn_list_valid)
    {
      qbi_svc_bc_ext_lte_attach_config_s_add_profile_list_from_modem(txn, 
        qmi_rsp->pending_attach_pdn_list_len, qmi_rsp->pending_attach_pdn_list);
      QBI_LOG_D_1("LTEAttachConfig::S: No. of profiles from pending list: %d",
        info->num_of_profile);
    }
  }

  if (!info->num_of_profile)
  {
    QBI_LOG_E_0("LTEAttachConfig::S: E:No LTE attach profile exists.");
    return FALSE;
  }

  return TRUE;
} /* qbi_svc_bc_ext_lte_attach_config_s_get_profile_list() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_operation_factory_restore
===========================================================================*/
/*!
  @brief Allocates and populates a QMI_WDS_DELETE_PROFILE_REQ request

  @details

  @param txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_operation_factory_restore
(
  qbi_txn_s *txn
)
{
  qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s *req = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->req.data);

  QBI_LOG_D_0("LTEAttachConfig::S: Factory restore.");

  if (!qbi_svc_bc_ext_lte_attach_config_s_register_for_detach_attach(txn))
  {
    action = QBI_SVC_ACTION_ABORT;
  }
  else
  {
    QBI_MEM_FREE(txn->req.data);
    txn->req.data = NULL;

    req = qbi_util_buf_alloc(&txn->req,
      sizeof(qbi_svc_bc_ext_provisioned_contexts_v2_s_req_s));
    QBI_CHECK_NULL_PTR_RET_ABORT(req);

    req->operation = QBI_SVC_MBIM_MS_CONTEXT_OPERATION_RESTORE_FACTORY;
    action = qbi_svc_bc_ext_provisioned_context_v2_s_req(txn);
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_config_s_operation_factory_restore() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_delete_user_defined_profile
===========================================================================*/
/*!
  @brief  Performs deletion of user created profiles for
          MBIM_CID_MS_LTE_ATTACH_CONFIG set request

  @details

  @param txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_delete_user_defined_profile
(
  qbi_txn_s *txn
)
{
  wds_get_lte_max_attach_pdn_num_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
  qbi_svc_bc_ext_cache_s *cache = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);

  info = (qbi_svc_bc_ext_lte_attach_config_info_s *)txn->info;
  while (info->profiles_read < info->num_of_profile)
  {
    cache = qbi_svc_bc_ext_cache_get(txn->ctx, 
      info->profile_index[info->profiles_read]);
    QBI_CHECK_NULL_PTR_RET_FALSE(cache);

    QBI_LOG_D_4(" LTEAttachConfig::S: Cached profile[%d/%d]: %d, flag: %d", 
      info->profiles_read, info->num_of_profile,
      info->profile_index[info->profiles_read], cache->context_flag);

    if (QBI_SVC_BC_EXT_CONTEXT_FLAG_USER_DEFINED == cache->context_flag)
    {
      QBI_LOG_D_1(" LTEAttachConfig::S: Deleting user defined profile %d",
        info->profile_index[info->profiles_read]);
      action = qbi_svc_bc_ext_lte_attach_config_s_build_wds29_req(txn);
      break;
    }
    else
    {
      info->profiles_read++;
    }
  }
  
  if(QBI_SVC_ACTION_ABORT == action)
  {
    qmi_req = (wds_get_lte_max_attach_pdn_num_req_msg_v01 *)
      qbi_qmi_txn_alloc_ret_req_buf(txn, QBI_QMI_SVC_WDS,
        QMI_WDS_GET_LTE_MAX_ATTACH_PDN_NUM_REQ_V01,
        qbi_svc_bc_ext_lte_attach_config_s_wds92_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_config_s_delete_user_defined_profile() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_operation_default
===========================================================================*/
/*!
  @brief Allocates and populates a QMI_WDS_DELETE_PROFILE_REQ request

  @details

  @param txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_operation_default
(
  qbi_txn_s *txn
)
{
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  
  qbi_svc_bc_ext_lte_attach_config_s_add_profile_list_from_cache(txn);

  return qbi_svc_bc_ext_lte_attach_config_s_delete_user_defined_profile(txn);
} /* qbi_svc_bc_ext_lte_attach_config_s_operation_default() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_wds94_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_GET_LTE_ATTACH_PDN_LIST_RESP for
    MBIM_CID_MS_LTE_ATTACH_CONFIG set request

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds94_rsp_cb
(
  qbi_qmi_txn_s  *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_get_lte_attach_pdn_list_resp_msg_v01 *qmi_rsp = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (wds_get_lte_attach_pdn_list_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1(" LTEAttachConfig::S: E: Received error code %d from QMI",
      qmi_rsp->resp.error);
    cmd_in_progress_ignore_indication = FALSE;
  }
  else
  {
    if (qbi_svc_bc_ext_lte_attach_config_s_get_profile_list(
      qmi_txn->parent, qmi_rsp))
    {
      action = qbi_svc_bc_ext_lte_attach_config_s_get_next_profile(
        qmi_txn->parent);
    }
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_config_s_wds94_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_match_profile
===========================================================================*/
/*!
    @brief Compares LTE attach profiles of modem and apps

    @details

    @param txn
    @param qmi_rsp

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_lte_attach_config_s_match_profile
(
  qbi_txn_s *txn,
  wds_get_profile_settings_resp_msg_v01 *qmi_rsp
)
{
  qbi_svc_bc_ext_lte_attach_config_s_req_s *req = NULL;
  qbi_svc_bc_ext_lte_attach_context_s *context = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
  qbi_mbim_offset_size_pair_s* field_desc = NULL;
  const uint8 *field = NULL;
  boolean match_found = FALSE;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(txn);
  QBI_CHECK_NULL_PTR_RET_FALSE(txn->req.data);
  QBI_CHECK_NULL_PTR_RET_FALSE(txn->info);
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_rsp);

  req = (qbi_svc_bc_ext_lte_attach_config_s_req_s *)txn->req.data;
  info = (qbi_svc_bc_ext_lte_attach_config_info_s *)txn->info;

  info->element_read = 0;

  do
  {
    if (info->element_match_index[info->element_read] == 0)
    {
      QBI_LOG_I_3("LTEAttachConfig::S: Matching profile for roam type: %d, Modem Index: %d",
        info->element_read, info->profile_index[info->profiles_read],
        info->element_match_index[info->element_read]);

      field_desc = (qbi_mbim_offset_size_pair_s*)((uint8 *)txn->req.data +
        sizeof(qbi_svc_bc_ext_lte_attach_config_s_req_s) +
        sizeof(qbi_mbim_offset_size_pair_s) * info->element_read);

      context = (qbi_svc_bc_ext_lte_attach_context_s *)
        qbi_txn_req_databuf_get_field(txn, field_desc, 0,
          field_desc->size);
      QBI_CHECK_NULL_PTR_RET_FALSE(context);

      match_found = qbi_svc_bc_ext_match_pdp_type(context, qmi_rsp) ? TRUE : FALSE;

      if (match_found)
      {
        QBI_LOG_I_0("LTEAttachConfig::S: Matching APN");

        field = qbi_txn_req_databuf_get_field(txn, &context->access_string,
          field_desc->offset, QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES);

        match_found = qbi_svc_bc_ext_match_string_field(qmi_rsp->apn_name_valid,
          qmi_rsp->apn_name, field, context->access_string.size);
      }

      if (match_found)
      {
        QBI_LOG_I_0("LTEAttachConfig::S: Matching username");

        field = qbi_txn_req_databuf_get_field(txn, &context->username,
          field_desc->offset, QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES);

        match_found = qbi_svc_bc_ext_match_string_field(qmi_rsp->username_valid,
          qmi_rsp->username, field, context->username.size);
      }

      if (match_found)
      {
        QBI_LOG_I_0("LTEAttachConfig::S: Matching password");

        field = qbi_txn_req_databuf_get_field(txn, &context->password,
          field_desc->offset, QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES);

        match_found = qbi_svc_bc_ext_match_string_field(qmi_rsp->password_valid,
          qmi_rsp->password, field, context->password.size);
      }

      if (match_found)
      {
        info->element_match_index[info->element_read] =
          info->profile_index[info->profiles_read];

        QBI_LOG_I_2("LTEAttachConfig::S: Match found. Added profile index: %d to roam type %d",
          info->element_match_index[info->element_read], info->element_read);

        return TRUE;
      }
      else
      {
        QBI_LOG_I_0("LTEAttachConfig::S: Match not found.");
      }
    }

    info->element_read++;
  } while (info->element_read < req->element_count);

  return FALSE;
}/* qbi_svc_bc_ext_lte_attach_config_s_match_profile() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_wds2b_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_GET_PROFILE_SETTINGS_RESP for
    MBIM_CID_MS_LTE_ATTACH_CONFIG set request

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds2b_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  wds_get_profile_settings_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  QBI_LOG_D_0("LTEAttachConfig::S: Received Resp For GET PROFILE SETTINGS Req");
  qmi_rsp = (wds_get_profile_settings_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("LTEAttachConfig::S: E:Received error code %d from QMI", 
      qmi_rsp->resp.error);

    qbi_svc_bc_ext_lte_attach_config_set_mbim_error_status(qmi_txn->parent, 
      qmi_rsp->resp.error, qmi_rsp->extended_error_code_valid,
      qmi_rsp->extended_error_code);

    if (QMI_ERR_EXTENDED_INTERNAL_V01 == qmi_rsp->resp.error && 
      qmi_rsp->extended_error_code_valid &&
      WDS_EEC_DS_PROFILE_REG_RESULT_ERR_INVAL_PROFILE_NUM_V01 == 
      qmi_rsp->extended_error_code)
    {
      QBI_LOG_D_0("LTEAttachConfig::S: Bad profile with modem. Continue to next profile");

      info = (qbi_svc_bc_ext_lte_attach_config_info_s *)qmi_txn->parent->info;
      info->profiles_read++;
      action = qbi_svc_bc_ext_lte_attach_config_s_get_next_profile(
        qmi_txn->parent);
    }
  }
  else
  {
    if (qbi_svc_bc_ext_lte_attach_config_s_match_profile(
      qmi_txn->parent, qmi_rsp))
    {
      action = qbi_svc_bc_ext_lte_attach_config_s_build_wds28_req(
        qmi_txn->parent);
    }
    else
    {
      info = (qbi_svc_bc_ext_lte_attach_config_info_s *)qmi_txn->parent->info;
      info->profiles_read++;
      action = qbi_svc_bc_ext_lte_attach_config_s_get_next_profile(
               qmi_txn->parent);
    }
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_config_s_wds2b_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_wds27_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_CREATE_PROFILE_RESP for
    MBIM_CID_MS_LTE_ATTACH_CONFIG set request

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds27_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  wds_create_profile_req_msg_v01 *qmi_req = NULL;
  wds_create_profile_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  qbi_svc_bc_ext_lte_attach_config_profile_settings_s profile_settings = { 0 };
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (wds_create_profile_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("LTEAttachConfig::S: Received error code %d from QMI", 
      qmi_rsp->resp.error);
    qbi_svc_bc_ext_lte_attach_config_set_mbim_error_status(
      qmi_txn->parent, qmi_rsp->resp.error,
      qmi_rsp->extended_error_code_valid, qmi_rsp->extended_error_code);
  }
  else
  {
    QBI_LOG_D_1("LTEAttachConfig::S: Profile created successfully at index %d",
      qmi_rsp->profile.profile_index);

    info = (qbi_svc_bc_ext_lte_attach_config_info_s*)qmi_txn->parent->info;
    info->element_match_index[info->element_read] = 
      qmi_rsp->profile.profile_index;
    info->profile_index[info->profiles_read] = qmi_rsp->profile.profile_index;

    QBI_LOG_D_2("LTEAttachConfig::S: Added profile index: %d to roam type: %d",
      info->profile_index[info->profiles_read], info->profiles_read);

    qmi_req = (wds_create_profile_req_msg_v01 *)qmi_txn->req.data;

    profile_settings.roaming_disallowed_valid = qmi_req->roaming_disallowed_valid;
    profile_settings.roaming_disallowed = qmi_req->roaming_disallowed;

    profile_settings.pdp_type_valid = qmi_req->pdp_type_valid;
    profile_settings.pdp_type = qmi_req->pdp_type;

    qbi_svc_bc_ext_lte_attach_config_update_cache(
      qmi_txn->parent, TRUE, profile_settings);

    if (qbi_svc_bc_ext_lte_attach_config_s_is_profile_match_pending(info))
    {
      info->profiles_read = 0;
    }

    info->num_of_profile++;
    action = qbi_svc_bc_ext_lte_attach_config_s_get_next_profile(
      qmi_txn->parent);
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_config_s_wds27_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_build_wds28_req
===========================================================================*/
/*!
    @brief Allocates and populates a QMI_WDS_MODIFY_PROFILE_SETTINGS request

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_build_wds28_req
(
  qbi_txn_s *txn
)
{
  wds_modify_profile_settings_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
  qbi_svc_bc_ext_lte_attach_config_profile_settings_s profile_settings = { 0 };
  qbi_svc_bc_ext_lte_attach_context_s *context = NULL;
  qbi_mbim_offset_size_pair_s* field_desc = NULL;
  const uint8 *field = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);

  QBI_LOG_D_0("Initiating Modify Profile Request");
  info = (qbi_svc_bc_ext_lte_attach_config_info_s *)txn->info;

  field_desc = (qbi_mbim_offset_size_pair_s *)((uint8_t*)txn->req.data +
    sizeof(qbi_svc_bc_ext_lte_attach_config_s_req_s) +
    sizeof(qbi_mbim_offset_size_pair_s) * info->element_read);
  QBI_CHECK_NULL_PTR_RET_ABORT(field_desc);

  context = (qbi_svc_bc_ext_lte_attach_context_s *)
    qbi_txn_req_databuf_get_field(txn, field_desc, 0, field_desc->size);
  QBI_CHECK_NULL_PTR_RET_ABORT(context);

  qmi_req = (wds_modify_profile_settings_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(txn, QBI_QMI_SVC_WDS,
      QMI_WDS_MODIFY_PROFILE_SETTINGS_REQ_V01,
      qbi_svc_bc_ext_lte_attach_config_s_wds28_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  qmi_req->apn_name_valid = TRUE;
  if (context->access_string.size != 0)
  {
    field = qbi_txn_req_databuf_get_field(txn, &context->access_string,
      field_desc->offset, QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES);
    QBI_CHECK_NULL_PTR_RET_FALSE(field);

    qbi_util_utf16_to_ascii(field, context->access_string.size,
      qmi_req->apn_name, sizeof(qmi_req->apn_name));
  }

  qmi_req->username_valid = TRUE;
  if(context->username.size != 0)
  {
    field = qbi_txn_req_databuf_get_field(txn, &context->username,
      field_desc->offset, QBI_SVC_BC_USERNAME_MAX_LEN_BYTES);
    QBI_CHECK_NULL_PTR_RET_FALSE(field);
    
    qbi_util_utf16_to_ascii(field, context->username.size,
      qmi_req->username, sizeof(qmi_req->username));
  }

  qmi_req->password_valid = TRUE;
  if(context->password.size != 0)
  {
    field = qbi_txn_req_databuf_get_field(txn, &context->password,
      field_desc->offset, QBI_SVC_BC_PASSWORD_MAX_LEN_BYTES);
    QBI_CHECK_NULL_PTR_RET_FALSE(field);
    
    qbi_util_utf16_to_ascii(field, context->password.size,
      qmi_req->password, sizeof(qmi_req->password));
  }

  qmi_req->profile.profile_index = 
    (uint8_t)info->profile_index[info->profiles_read];

  qmi_req->pdp_type_valid = TRUE;
  qmi_req->pdp_type = qbi_svc_bc_ext_ip_type_to_pdp_type(context->ip_type);

  qmi_req->roaming_disallowed_valid = TRUE;
  qmi_req->roaming_disallowed = FALSE;
  
  profile_settings.pdp_data_compression_type_valid =
    &qmi_req->pdp_data_compression_type_valid;
  profile_settings.pdp_data_compression_type =
    &qmi_req->pdp_data_compression_type;
  profile_settings.pdp_hdr_compression_type_valid =
    &qmi_req->pdp_hdr_compression_type_valid;
  profile_settings.pdp_hdr_compression_type =
    &qmi_req->pdp_hdr_compression_type;
  profile_settings.authentication_preference_valid =
    &qmi_req->authentication_preference_valid;
  profile_settings.authentication_preference =
    &qmi_req->authentication_preference;

  if (!qbi_svc_bc_ext_lte_attach_config_s_populate_profile_compression(
    context, profile_settings) ||
    !qbi_svc_bc_ext_lte_attach_config_s_populate_profile_auth_protocol(
      context, profile_settings))
  {
    QBI_LOG_E_0("LTEAttachConfig::S: Couldn't populate compression or auth protocol!");
  }
  
  return QBI_SVC_ACTION_SEND_QMI_REQ;
} /* qbi_svc_bc_ext_lte_attach_config_s_build_wds28_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_is_profile_match_pending
===========================================================================*/
/*!
    @brief Return TRUE/FALSE whether there is a pending profile create 

    @details

    @param qmi_txn

    @return boolean
*/
/*=========================================================================*/
boolean qbi_svc_bc_ext_lte_attach_config_s_is_profile_match_pending
(
  qbi_svc_bc_ext_lte_attach_config_info_s * info
)
{
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(info);

  info->element_read = 0;
  while (info->element_read < QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_MAX_PROFILE)
  {
    if (info->element_match_index[info->element_read] == 0)
    {
      return TRUE;
    }

    info->element_read++;
  }

  QBI_LOG_D_0("Created/modified Profiles for all roaming type completed.");
  return FALSE;
}/* qbi_svc_bc_ext_lte_attach_config_s_is_profile_match_pending() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_wds28_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_MODIFY_PROFILE_SETTINGS_RESP for
    MBIM_CID_MS_LTE_ATTACH_CONFIG set request

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds28_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_modify_profile_settings_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s * info = NULL;
  wds_modify_profile_settings_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_lte_attach_config_profile_settings_s profile_settings = { 0 };
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  QBI_LOG_D_0("LTEAttachConfig::S: Received Modified Complete Response.");
  qmi_rsp = (wds_modify_profile_settings_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("LTEAttachConfig::S: Received error code %d from QMI.",
      qmi_rsp->resp.error);
    qbi_svc_bc_ext_lte_attach_config_set_mbim_error_status(
      qmi_txn->parent, qmi_rsp->resp.error,
      qmi_rsp->extended_error_code_valid, qmi_rsp->extended_error_code);
  }
  else
  {
    //Send SET PDN LIST request for the modified profile
    qmi_req = (wds_modify_profile_settings_req_msg_v01 *)qmi_txn->req.data;
    info = (qbi_svc_bc_ext_lte_attach_config_info_s *)qmi_txn->parent->info;

    info->element_match_index[info->element_read] =
      qmi_req->profile.profile_index;

    profile_settings.roaming_disallowed_valid = qmi_req->roaming_disallowed_valid;
    profile_settings.roaming_disallowed = qmi_req->roaming_disallowed;

    profile_settings.pdp_type_valid = qmi_req->pdp_type_valid;
    profile_settings.pdp_type = qmi_req->pdp_type;

    (void)qbi_svc_bc_ext_lte_attach_config_update_cache(
      qmi_txn->parent, FALSE, profile_settings);

    if (qbi_svc_bc_ext_lte_attach_config_s_is_profile_match_pending(info))
    {
      info->profiles_read = 0;
    }

    action = qbi_svc_bc_ext_lte_attach_config_s_get_next_profile(
      qmi_txn->parent);
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_config_s_wds28_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_build_wds93_req
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_SET_LTE_ATTACH_PDN_LIST for
    MBIM_CID_MS_LTE_ATTACH_CONFIG set request

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_build_wds93_req
(
  qbi_txn_s *txn
)
{
  wds_set_lte_attach_pdn_list_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
  uint16 profile_index = 0;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);

  // Set profile as LTE profile
  QBI_LOG_D_0("LTEAttachConfig::S: Setting PDN List");
  qmi_req = (wds_set_lte_attach_pdn_list_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(txn, QBI_QMI_SVC_WDS,
      QMI_WDS_SET_LTE_ATTACH_PDN_LIST_REQ_V01,
      qbi_svc_bc_ext_lte_attach_config_s_wds93_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  info = (qbi_svc_bc_ext_lte_attach_config_info_s*)txn->info;

  //Copying as it is, we need three profiles in modem this might help in query
  for (profile_index = 0; 
    profile_index < QBI_SVC_MBIM_MS_LTE_ATTACH_CONTEXT_MAX_PROFILE; 
    profile_index++)
  {
    if (info->element_match_index[profile_index])
    {
      qmi_req->attach_pdn_list[qmi_req->attach_pdn_list_len] =
        info->element_match_index[profile_index];

      QBI_LOG_D_2("LTEAttachConfig::S: attach_pdn_list[%d] = %d ",
        qmi_req->attach_pdn_list_len, qmi_req->attach_pdn_list[profile_index]);

      qmi_req->attach_pdn_list_len++;
    }
  }

  return QBI_SVC_ACTION_SEND_QMI_REQ;
} /* qbi_svc_bc_ext_lte_attach_config_s_build_wds93_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_wds93_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_SET_LTE_ATTACH_PDN_LIST for
    MBIM_CID_MS_LTE_ATTACH_CONFIG set request

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds93_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_update_lte_attach_pdn_list_profiles_req_msg_v01* qmi_req = NULL;
  wds_set_lte_attach_pdn_list_resp_msg_v01 *qmi_rsp = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  QBI_LOG_D_0("LTEAttachConfig::S: Set PDN list response received");
  qmi_rsp = (wds_set_lte_attach_pdn_list_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("LTEAttachConfig::S: Received error code %d from QMI", 
      qmi_rsp->resp.error);
    cmd_in_progress_ignore_indication = FALSE;
  }
  else
  {
    qmi_req = (wds_update_lte_attach_pdn_list_profiles_req_msg_v01 *)
      qbi_qmi_txn_alloc_ret_req_buf(qmi_txn->parent, QBI_QMI_SVC_WDS,
        QMI_WDS_UPDATE_LTE_ATTACH_PDN_LIST_PROFILES_REQ_V01,
        qbi_svc_bc_ext_lte_attach_config_s_wds9f_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_config_s_wds93_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_wds9f_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_UPDATE_LTE_ATTACH_PDN_LIST_PROFILES_RESP for
    MBIM_CID_MS_LTE_ATTACH_CONFIG set request

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds9f_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_update_lte_attach_pdn_list_profiles_resp_msg_v01 *qmi_rsp = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);

  QBI_LOG_D_0("LTEAttachConfig::S: Received update PDN list response");
  qmi_rsp = (wds_update_lte_attach_pdn_list_profiles_resp_msg_v01 *)
    qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("LTEAttachConfig::S: Received error code %d from QMI", 
      qmi_rsp->resp.error);
    cmd_in_progress_ignore_indication = FALSE;
  }
  else
  {
    QBI_LOG_D_0("LTEAttachConfig::S: Initiating detach.");
    action = qbi_svc_bc_ext_lte_attach_config_s_build_nas33_req(
      qmi_txn->parent, TRUE);
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_config_s_wds9f_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_build_nas33_req
===========================================================================*/
/*!
  @brief Allocates and populates a QMI_WDS_MODIFY_PROFILE_SETTINGS request

  @details

  @param txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_build_nas33_req
(
  qbi_txn_s *txn,
  boolean flag
)
{
  nas_set_system_selection_preference_req_msg_v01 *qmi_req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->req.data);

  qmi_req = (nas_set_system_selection_preference_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(txn, QBI_QMI_SVC_NAS,
      QMI_NAS_SET_SYSTEM_SELECTION_PREFERENCE_REQ_MSG_V01,
      qbi_svc_bc_ext_lte_attach_config_s_nas33_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  qmi_req->change_duration_valid = TRUE;
  qmi_req->change_duration = NAS_POWER_CYCLE_V01;

  qmi_req->srv_domain_pref_valid = TRUE;

  qmi_req->srv_domain_pref = flag ? 
    QMI_SRV_DOMAIN_PREF_PS_DETACH_V01 : QMI_SRV_DOMAIN_PREF_PS_ATTACH_V01;

  QBI_LOG_STR_1("LTEAttachConfig::S: Initiating %s", 
    flag ? "Detach" : "Attach");

  return QBI_SVC_ACTION_SEND_QMI_REQ;
} /* qbi_svc_bc_ext_lte_attach_config_s_build_nas33_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_nas33_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_UPDATE_LTE_ATTACH_PDN_LIST_PROFILES_RESP for
    MBIM_CID_MS_LTE_ATTACH_CONFIG set request

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_nas33_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  nas_set_system_selection_preference_resp_msg_v01 *qmi_rsp = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (nas_set_system_selection_preference_resp_msg_v01 *)
    qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
    cmd_in_progress_ignore_indication = FALSE;
  }

  return QBI_SVC_ACTION_WAIT_ASYNC_RSP;
} /* qbi_svc_bc_ext_lte_attach_config_s_nas33_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_build_wds29_req
===========================================================================*/
/*!
  @brief Allocates and populates a QMI_WDS_DELETE_PROFILE_REQ request

  @details

  @param txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_build_wds29_req
(
  qbi_txn_s *txn
)
{
  wds_delete_profile_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s *info = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);

  info = (qbi_svc_bc_ext_lte_attach_config_info_s *)txn->info;

  qmi_req = (wds_delete_profile_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(
      txn, QBI_QMI_SVC_WDS, QMI_WDS_DELETE_PROFILE_REQ_V01,
      qbi_svc_bc_ext_lte_attach_config_s_wds29_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  qmi_req->profile.profile_index = (uint8_t)
    info->profile_index[info->profiles_read];

  return QBI_SVC_ACTION_SEND_QMI_REQ;
} /* qbi_svc_bc_ext_lte_attach_config_s_build_wds29_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_wds29_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_DELETE_PROFILE_RESP for
    MBIM_CID_MS_LTE_ATTACH_CONFIG set request

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds29_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  wds_delete_profile_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_lte_attach_config_info_s * info = NULL;
  qbi_svc_bc_ext_cache_s *cache = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (wds_delete_profile_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("LTEAttachConfig::S: E: Received error code %d from QMI", 
      qmi_rsp->resp.error);
    qbi_svc_bc_ext_lte_attach_config_set_mbim_error_status(
      qmi_txn->parent, qmi_rsp->resp.error,
      qmi_rsp->extended_error_code_valid, qmi_rsp->extended_error_code);
  }

  info = (qbi_svc_bc_ext_lte_attach_config_info_s *)qmi_txn->parent->info;

  cache = qbi_svc_bc_ext_cache_get(qmi_txn->parent->ctx,
    info->profile_index[info->profiles_read]);
  QBI_CHECK_NULL_PTR_RET_ABORT(cache);

  QBI_MEMSET(cache, 0, sizeof(qbi_svc_bc_ext_cache_s));

  info->profiles_read++;
  return qbi_svc_bc_ext_lte_attach_config_s_delete_user_defined_profile(
    qmi_txn->parent);
} /* qbi_svc_bc_ext_lte_attach_config_s_wds29_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_wds95_ind_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_LTE_ATTACH_PDN_LIST_IND, looking for changes to 
    the current channel rate or data system status that would trigger an
    MBIM_CID_PACKET_SERVICE event

    @details

    @param ind

    @return qbi_svc_action_e
*/
/*=========================================================================*/
qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_wds95_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  /*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(ind);
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->buf);
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->buf->data);

  cmd_in_progress_ignore_indication = FALSE;

  if (ind->qmi_svc_id < QBI_QMI_SVC_WDS_FIRST ||
    ind->qmi_svc_id > QBI_QMI_SVC_WDS_LAST)
  {
    QBI_LOG_E_1("LTEAttachConfig::I: qmi_svc_id %d is out of range", 
      ind->qmi_svc_id);
  }
  else
  {
    QBI_LOG_D_0("LTEAttachConfig::I: UPDATE PDN LIST.Triggering Query");
    action = qbi_svc_bc_ext_lte_attach_config_q_req(ind->txn);
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_config_s_wds95_ind_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_s_nas24_ind_cb
===========================================================================*/
/*!
    @brief Handles a QMI_NAS_SYSTEM_SELECTION_PREFERENCE_IND, looking for 
    changes to the current channel rate or data system status that would 
    trigger an MBIM_CID_PACKET_SERVICE event

    @details

    @param ind

    @return qbi_svc_action_e
*/
/*=========================================================================*/
qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_s_nas34_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  const nas_system_selection_preference_ind_msg_v01 *qmi_ind;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(ind);
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->buf);
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->buf->data);

  qmi_ind = (const nas_system_selection_preference_ind_msg_v01 *)ind->buf->data;
  
  if (qmi_ind->srv_domain_pref_valid)
  {
    if (QMI_SRV_DOMAIN_PREF_CS_ONLY_V01 == qmi_ind->srv_domain_pref)
    {
      QBI_LOG_D_0("LTEAttachConfig::S: Detach successful. Trigger attach.");
      action = qbi_svc_bc_ext_lte_attach_config_s_build_nas33_req(
        ind->txn, FALSE);
    }
    else if (QMI_SRV_DOMAIN_PREF_PS_ONLY_V01 == qmi_ind->srv_domain_pref ||
        QMI_SRV_DOMAIN_PREF_CS_PS_V01 == qmi_ind->srv_domain_pref)
    {
      QBI_LOG_D_0("LTEAttachConfig::S: Attach successful. "
        "Waiting For UPDATE PDN LIST Ind");
      action = QBI_SVC_ACTION_WAIT_ASYNC_RSP;
    }
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_config_s_nas24_ind_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_static_ind_e_wds95_ind_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_LTE_ATTACH_PDN_LIST_IND, looking for changes to 
    the current channel rate or data system status that would trigger an
    MBIM_CID_PACKET_SERVICE event

    @details

    @param ind

    @return qbi_svc_action_e
*/
/*=========================================================================*/
qbi_svc_action_e qbi_svc_bc_ext_lte_attach_config_static_ind_e_wds95_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  /*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(ind);
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->buf);
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->buf->data);

  QBI_LOG_D_0("LTEAttachConfig::I Static Ind Received for UPDATE LIST");
  if (!cmd_in_progress_ignore_indication)
  {
    if (ind->qmi_svc_id < QBI_QMI_SVC_WDS_FIRST ||
      ind->qmi_svc_id > QBI_QMI_SVC_WDS_LAST)
    {
      QBI_LOG_E_1("LTEAttachConfig::I: qmi_svc_id %d is out of range", 
        ind->qmi_svc_id);
    }
    else
    {
      QBI_LOG_D_0("LTEAttachConfig::I: Triggering Query");
      action = qbi_svc_bc_ext_lte_attach_config_q_req(ind->txn);
    }
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_config_static_ind_e_wds95_ind_cb() */

/*! @} */

/*! @addtogroup MBIM_CID_MS_LTE_ATTACH_STATUS
    @{ */

/*===========================================================================
FUNCTION: qbi_svc_lte_attach_status_q_match_ip_type
===========================================================================*/
/*!
    @brief Compares MBIM IP type with QMI PDP type

  @details

  @param ip_type

  @return wds_pdp_type_enum_v01
*/
/*=========================================================================*/
static wds_pdp_type_enum_v01 qbi_svc_lte_attach_status_q_match_ip_type
(
  wds_ip_support_type_enum_v01 ip_type,
  wds_pdp_type_enum_v01 pdp_type
)
{
/*-------------------------------------------------------------------------*/

  QBI_LOG_D_2("LTEAttachStatus::Q: Matching ip_type (%d) <==> pdp_type (%d).",
    ip_type, pdp_type);

  switch (ip_type)
  {
  case WDS_IP_SUPPORT_TYPE_IPV4_V01:
    if (WDS_PDP_TYPE_PDP_IPV4_V01 == pdp_type ||
      WDS_PDP_TYPE_PDP_IPV4V6_V01 == pdp_type)
    {
      return TRUE;
    }
  case WDS_IP_SUPPORT_TYPE_IPV6_V01:
    if (WDS_PDP_TYPE_PDP_IPV6_V01 == pdp_type ||
      WDS_PDP_TYPE_PDP_IPV4V6_V01 == pdp_type)
    {
      return TRUE;
    }
  case WDS_IP_SUPPORT_TYPE_IPV4V6_V01:
    if (WDS_PDP_TYPE_PDP_IPV4_V01 == pdp_type ||
      WDS_PDP_TYPE_PDP_IPV6_V01 == pdp_type ||
      WDS_PDP_TYPE_PDP_IPV4V6_V01 == pdp_type)
    {
      return TRUE;
    }
  default:
    break;
  }
  if ((WDS_IP_SUPPORT_TYPE_IPV4_V01 == ip_type &&
    WDS_PDP_TYPE_PDP_IPV4_V01 == pdp_type) ||
    (WDS_IP_SUPPORT_TYPE_IPV6_V01 == ip_type &&
      WDS_PDP_TYPE_PDP_IPV6_V01 == pdp_type) ||
      (WDS_IP_SUPPORT_TYPE_IPV4V6_V01 == ip_type &&
        WDS_PDP_TYPE_PDP_IPV4V6_V01 == pdp_type))
  {
    return TRUE;
  }

  QBI_LOG_D_0("LTEAttachStatus::Q: IP type did not match");
  return FALSE;
} /* qbi_svc_lte_attach_status_q_match_ip_type */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_ip_type_to_pdp_type
===========================================================================*/
/*!
    @brief Translates MBIM IP type to QMI PDP type

  @details

  @param ip_type

  @return wds_pdp_type_enum_v01
*/
/*=========================================================================*/
static wds_pdp_type_enum_v01 qbi_svc_lte_attach_status_q_ip_type_to_pdp_type
(
  wds_ip_support_type_enum_v01 ip_type
)
{
  wds_pdp_type_enum_v01 pdp_type = WDS_PDP_TYPE_PDP_NON_IP_V01;
/*-------------------------------------------------------------------------*/
   switch (ip_type)
   {
     case WDS_IP_SUPPORT_TYPE_IPV4_V01:
     pdp_type = WDS_PDP_TYPE_PDP_IPV4_V01;
     break;

    case WDS_IP_SUPPORT_TYPE_IPV6_V01:
     pdp_type = WDS_PDP_TYPE_PDP_IPV4_V01;
     break;

    case WDS_IP_SUPPORT_TYPE_IPV4V6_V01:
     pdp_type = WDS_PDP_TYPE_PDP_IPV4V6_V01;
     break;

    default:
     break;
   }

  QBI_LOG_D_2("LTEAttachStatus::Q: Returning PDP type (%d) for IP type (%d)",
               pdp_type, ip_type);

  return pdp_type;
} /* qbi_svc_bc_ext_ip_type_to_pdp_type */

/*===========================================================================
FUNCTION: qbi_svc_lte_attach_status_q_qmi_profile_to_mbim_auth_proto
===========================================================================*/
/*!
    @brief Extracts the authentication protocol information from a QMI
    profile (EPC or 3GPP), if available, and returns it as an MBIM value

    @details

    @param profile_type
    @param qmi_rsp

    @return uint32 MBIM_AUTH_PROTOCOL value
*/
/*=========================================================================*/
static uint32 qbi_svc_lte_attach_status_q_qmi_profile_to_mbim_auth_proto
(
  wds_profile_type_enum_v01                                 profile_type,
  const qbi_svc_bc_ext_lte_attach_status_profile_settings_s profile_settings
)
{
  uint32 mbim_auth_proto = QBI_SVC_BC_AUTH_PROTOCOL_NONE;
/*-------------------------------------------------------------------------*/
  if (profile_type == WDS_PROFILE_TYPE_3GPP_V01 &&
    profile_settings.authentication_preference_valid)
  {
    if (profile_settings.authentication_preference & 
      QMI_WDS_MASK_AUTH_PREF_CHAP_V01)
    {
      mbim_auth_proto = QBI_SVC_BC_AUTH_PROTOCOL_CHAP;
    }
    else if (profile_settings.authentication_preference & 
      QMI_WDS_MASK_AUTH_PREF_PAP_V01)
    {
      mbim_auth_proto = QBI_SVC_BC_AUTH_PROTOCOL_PAP;
    }
  }
  else
  {
    QBI_LOG_E_1("Invalid profile type %d", profile_type);
  }

  return mbim_auth_proto;
} /* qbi_svc_lte_attach_status_q_qmi_profile_to_mbim_auth_proto() */

/*===========================================================================
FUNCTION: qbi_svc_lte_attach_status_q_qmi_profile_to_mbim_compression
===========================================================================*/
/*!
    @brief Extracts the compression information from a QMI profile (3GPP or
    3GPP2), if available, and returns it as an MBIM value

    @details

    @param profile_type
    @param qmi_rsp

    @return uint32 MBIM_COMPRESSION value
*/
/*=========================================================================*/
static uint32 qbi_svc_lte_attach_status_q_qmi_profile_to_mbim_compression
(
  wds_profile_type_enum_v01                                 profile_type,
  const qbi_svc_bc_ext_lte_attach_status_profile_settings_s profile_settings
)
{
  uint32 mbim_compression = QBI_SVC_BC_COMPRESSION_NONE;
  /*-------------------------------------------------------------------------*/
  if (profile_type == WDS_PROFILE_TYPE_EPC_V01 &&
          ((profile_settings.pdp_data_compression_type_valid &&
          profile_settings.pdp_data_compression_type !=
          WDS_PDP_DATA_COMPR_TYPE_OFF_V01) ||
          (profile_settings.pdp_hdr_compression_type_valid &&
          profile_settings.pdp_hdr_compression_type !=
          WDS_PDP_HDR_COMPR_TYPE_OFF_V01)))
  {
    mbim_compression = QBI_SVC_BC_COMPRESSION_ENABLE;
  }

  return mbim_compression;
} /* qbi_svc_lte_attach_status_q_qmi_profile_to_mbim_compression() */

/*===========================================================================
FUNCTION: qbi_svc_lte_attach_status_q_get_last_active_profile_from_cache
===========================================================================*/
/*!
  @brief Allocates and populates a QMI_WDS_DELETE_PROFILE_REQ request

  @details

  @param txn

  @return boolean
*/
/*=========================================================================*/
static boolean qbi_svc_lte_attach_status_q_get_last_active_profile_from_cache
(
  qbi_txn_s *txn
)
{
  qbi_svc_bc_ext_lte_attach_status_info_s *info;
  qbi_svc_bc_ext_cache_s *cache = NULL;
  uint32 i = 0;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(txn);
  QBI_CHECK_NULL_PTR_RET_FALSE(txn->info);

  for (i = 0; i < QMI_WDS_PROFILE_LIST_MAX_V01; i++)
  {
    cache = qbi_svc_bc_ext_cache_get(txn->ctx, i);
    QBI_CHECK_NULL_PTR_RET_FALSE(cache);

    if (cache->lte_attach_state == QBI_SVC_MBIM_MS_LTE_ATTACH_STATE_ATTACHED)
    {
      info = (qbi_svc_bc_ext_lte_attach_status_info_s *)txn->info;

      info->num_of_profile = 1;
      info->profile_index[0] = i;     
      info->profiles_read = 0;
      
      QBI_LOG_D_1("LTEAttachStatus::Q: Last LTE Attach profile index %d.", i);
      return TRUE;
    }
  }

  return FALSE;
} /* qbi_svc_lte_attach_status_q_get_last_active_profile_from_cache() */

/*===========================================================================
FUNCTION: qbi_svc_lte_attach_status_q_update_cache
===========================================================================*/
/*!
  @brief Updates cache with LTE attach status

  @details

  @param txn

  @return boolean
*/
/*=========================================================================*/
static boolean qbi_svc_lte_attach_status_q_update_cache
(
  qbi_qmi_txn_s *qmi_txn
)
{
  wds_get_profile_settings_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_cache_s *cache = NULL;
  uint32 i = 0;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_txn->req.data);

  qmi_req = (wds_get_profile_settings_req_msg_v01 *)qmi_txn->req.data;
  for (i = 0; i < QMI_WDS_PROFILE_LIST_MAX_V01; i++)
  {
    cache = qbi_svc_bc_ext_cache_get(qmi_txn->ctx, i);
    QBI_CHECK_NULL_PTR_RET_FALSE(cache);

    cache->lte_attach_state = (i == qmi_req->profile.profile_index) ?
      QBI_SVC_MBIM_MS_LTE_ATTACH_STATE_ATTACHED :
      QBI_SVC_MBIM_MS_LTE_ATTACH_STATE_DETACHED;
  }

  qbi_svc_bc_ext_update_nv_store(qmi_txn->ctx);

  return TRUE;
} /* qbi_svc_lte_attach_status_q_update_cache() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_status_q_populate_profile_settings
===========================================================================*/
/*!
    @brief Populates profile settings from QMI_WDS_GET_PROFILE_SETTINGS_RESP
    for MBIM_CID_MS_LTE_ATTACH_CONFIG query

    @details

    @param qmi_rsp
    @param profile_settings

    @return boolean
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_lte_attach_status_q_populate_profile_settings
(
  wds_get_profile_settings_resp_msg_v01               *qmi_rsp,
  qbi_svc_bc_ext_lte_attach_status_profile_settings_s *profile_settings
)
{
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_rsp);
  QBI_CHECK_NULL_PTR_RET_FALSE(profile_settings);

  profile_settings->pdp_type_valid = qmi_rsp->pdp_type_valid;
  profile_settings->pdp_type = qmi_rsp->pdp_type;
  profile_settings->authentication_preference_valid = 
    qmi_rsp->authentication_preference_valid;
  profile_settings->authentication_preference =
    qmi_rsp->authentication_preference;
  profile_settings->pdp_data_compression_type_valid = 
    qmi_rsp->pdp_data_compression_type_valid;
  profile_settings->pdp_data_compression_type = 
    qmi_rsp->pdp_data_compression_type;
  profile_settings->pdp_hdr_compression_type_valid = 
    qmi_rsp->pdp_hdr_compression_type_valid;
  profile_settings->pdp_hdr_compression_type = 
    qmi_rsp->pdp_hdr_compression_type;
  profile_settings->apn_name_valid = qmi_rsp->apn_name_valid;
  profile_settings->username_valid = qmi_rsp->username_valid;
  profile_settings->password_valid = qmi_rsp->password_valid;

  if (profile_settings->apn_name_valid)
  {
    QBI_STRLCPY(profile_settings->apn_name, 
      qmi_rsp->apn_name, sizeof(qmi_rsp->apn_name));
  }

  if (profile_settings->username_valid)
  {
    QBI_STRLCPY(profile_settings->username, 
      qmi_rsp->username, sizeof(qmi_rsp->username));
  }
  if (profile_settings->password_valid)
  {
    QBI_STRLCPY(profile_settings->password, 
      qmi_rsp->password, sizeof(qmi_rsp->password));
  }

  return TRUE;
} /* qbi_svc_bc_ext_lte_attach_status_q_populate_profile_settings() */

/*===========================================================================
FUNCTION: qbi_svc_lte_attach_status_prepare_rsp
===========================================================================*/
/*!
    @brief Allocates and populates a MBIM_MS_LTE_ATTACH_STATUS structure on 
    the response

    @details

    @param qmi_txn
    @param profile_settings

    @return boolean TRUE on success, FALSE on failure
*/
/*=========================================================================*/
static boolean qbi_svc_lte_attach_status_prepare_rsp
(
  qbi_qmi_txn_s *qmi_txn,
  qbi_svc_bc_ext_lte_attach_status_profile_settings_s profile_settings
)
{
  boolean success = FALSE;
  qbi_svc_bc_ext_lte_attach_status_rsp_s *rsp;
  qbi_svc_bc_ext_lte_attach_status_info_s *info;
  uint32 initial_offset = 0;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_FALSE(qmi_txn->parent->info);

  rsp = (qbi_svc_bc_ext_lte_attach_status_rsp_s *)qbi_txn_alloc_rsp_buf(
    qmi_txn->parent, sizeof(qbi_svc_bc_ext_lte_attach_status_rsp_s));
  QBI_CHECK_NULL_PTR_RET_FALSE(rsp);

  info = (qbi_svc_bc_ext_lte_attach_status_info_s *)qmi_txn->parent->info;

  if (info->status_registered)
  {
    if (profile_settings.pdp_type_valid)
    {
      switch (profile_settings.pdp_type)
      {
      case WDS_PDP_TYPE_PDP_IPV4_V01:
        rsp->ip_type = QBI_SVC_BC_IP_TYPE_IPV4;
        break;
      case WDS_PDP_TYPE_PDP_IPV6_V01:
        rsp->ip_type = QBI_SVC_BC_IP_TYPE_IPV6;
        break;
      case WDS_PDP_TYPE_PDP_IPV4V6_V01:
        rsp->ip_type = QBI_SVC_BC_IP_TYPE_IPV4V6;
        break;
      default:
        break;
      }
    }

    rsp->lte_attach_state = info->lte_attach_state;

    rsp->compression = qbi_svc_lte_attach_status_q_qmi_profile_to_mbim_compression(
      WDS_PROFILE_TYPE_EPC_V01, profile_settings);
    rsp->auth_protocol = qbi_svc_lte_attach_status_q_qmi_profile_to_mbim_auth_proto(
			WDS_PROFILE_TYPE_3GPP_V01, profile_settings);

    /* Populate the DataBuffer */
    if (profile_settings.apn_name_valid &&
      !qbi_txn_rsp_databuf_add_string_ascii_to_utf16(
        qmi_txn->parent, &rsp->access_string, initial_offset,
        QBI_SVC_BC_ACCESS_STRING_MAX_LEN_BYTES, profile_settings.apn_name,
        sizeof(profile_settings.apn_name)))
    {
      QBI_LOG_E_0("Couldn't add 3GPP AccessString to response!");
      success = FALSE;
    }
    else if (profile_settings.username_valid &&
      !qbi_txn_rsp_databuf_add_string_ascii_to_utf16(
        qmi_txn->parent, &rsp->username, initial_offset,
        QBI_SVC_BC_USERNAME_MAX_LEN_BYTES, profile_settings.username,
        sizeof(profile_settings.username)))
    {
      QBI_LOG_E_0("Couldn't add 3GPP Username to response!");
      success = FALSE;
    }
    else if (profile_settings.password_valid &&
      !qbi_txn_rsp_databuf_add_string_ascii_to_utf16(
        qmi_txn->parent, &rsp->password, initial_offset,
        QBI_SVC_BC_PASSWORD_MAX_LEN_BYTES,
        profile_settings.password, sizeof(profile_settings.password)))
    {
      QBI_LOG_E_0("Couldn't add 3GPP Password to response!");
      success = FALSE;
    }
    else
    {
      success = TRUE;
    }

    if (success && QBI_TXN_CMD_TYPE_INTERNAL == qmi_txn->parent->cmd_type)
    {
      qbi_util_buf_s buf;
      qbi_util_buf_init(&buf);
      qbi_util_buf_alloc(&buf,qmi_txn->parent->infobuf_len_total);
      QBI_CHECK_NULL_PTR_RET_FALSE(buf.data);
      (void)qbi_txn_rsp_databuf_extract(
        qmi_txn->parent,buf.data,buf.size,0);
      qbi_util_buf_free(&qmi_txn->parent->rsp);
      qmi_txn->parent->rsp = buf;
    }
  }
  else
  {
    success = TRUE;
  }

  return success;
} /* qbi_svc_lte_attach_status_prepare_rsp() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_status_q_get_next_profile
===========================================================================*/
/*!
    @brief Retrive next available configured profile.

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_status_q_get_next_profile
(
  qbi_txn_s *txn
)
{
  wds_get_profile_settings_req_msg_v01 *qmi_req_wds2b;
  qbi_svc_bc_ext_lte_attach_status_info_s *info;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);

  info = (qbi_svc_bc_ext_lte_attach_status_info_s *)txn->info;
  if (info->profiles_read >= info->num_of_profile)
  {
    QBI_LOG_E_0("No LTE attach profile found.");
  }
  else
  {
    /* Issue a query to retrieve the profile details */
    qmi_req_wds2b = (wds_get_profile_settings_req_msg_v01 *)
      qbi_qmi_txn_alloc_ret_req_buf(
        txn, QBI_QMI_SVC_WDS, QMI_WDS_GET_PROFILE_SETTINGS_REQ_V01,
        qbi_svc_bc_ext_lte_attach_status_q_wds2b_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req_wds2b);

    qmi_req_wds2b->profile.profile_index = (uint8_t)
      info->profile_index[info->profiles_read];

    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_status_q_get_next_profile */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_lte_attach_status_q_req
===========================================================================*/
/*!
    @brief Handles a QBI_SVC_BC_EXT_MBIM_CID_MS_LTE_ATTACH_STATUS query
    request

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
qbi_svc_action_e qbi_svc_bc_ext_lte_attach_status_q_req
(
  qbi_txn_s *txn
)
{
  wds_get_lte_attach_params_req_msg_v01 *qmi_req;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);

  if (txn->info == NULL)
  {
    txn->info = QBI_MEM_MALLOC_CLEAR(
      sizeof(qbi_svc_bc_ext_lte_attach_status_info_s));
    QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);
  }

  qmi_req = (wds_get_lte_attach_params_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(txn, QBI_QMI_SVC_WDS, 
      QMI_WDS_GET_LTE_ATTACH_PARAMS_REQ_V01,
      qbi_svc_bc_ext_lte_attach_status_q_wds85_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  return QBI_SVC_ACTION_SEND_QMI_REQ;
} /* qbi_svc_bc_ext_lte_attach_status_q_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_status_q_dsd24_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_DSD_GET_SYSTEM_STATUS_RESP for
    MBIM_CID_MS_LTE_ATTACH_CONFIG query

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_status_q_dsd24_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  dsd_get_system_status_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_lte_attach_status_info_s *info = NULL;
  uint32 i = 0;
  qbi_svc_bc_ext_lte_attach_status_profile_settings_s profile_settings = { 0 };
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (dsd_get_system_status_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("LTEAttachStatus::Q: E:Received error code %d from QMI",
      qmi_rsp->resp.error);
  }
  else if (qmi_rsp->avail_sys_valid)
  {
    info = (qbi_svc_bc_ext_lte_attach_status_info_s *)qmi_txn->parent->info;

    for (i = 0; !info->status_registered && i < qmi_rsp->avail_sys_len; i++)
    {
      switch (qmi_rsp->avail_sys[i].rat_value)
      {
      case DSD_SYS_RAT_EX_3GPP_WCDMA_V01:
      case DSD_SYS_RAT_EX_3GPP_GERAN_V01:
      case DSD_SYS_RAT_EX_3GPP_TDSCDMA_V01:
      case DSD_SYS_RAT_EX_3GPP2_1X_V01:
      case DSD_SYS_RAT_EX_3GPP2_HRPD_V01:
      case DSD_SYS_RAT_EX_3GPP2_EHRPD_V01:
        QBI_LOG_D_1("LTEAttachStatus::Q: Device is registered on RAT: %d.",
          qmi_rsp->avail_sys[i].rat_value);
        info->status_registered = TRUE;
        break;
      default:
        break;
      }
    }

    info->lte_attach_state = QBI_SVC_MBIM_MS_LTE_ATTACH_STATE_DETACHED;
    if (info->status_registered)
    {
      if (qbi_svc_lte_attach_status_q_get_last_active_profile_from_cache(qmi_txn->parent))
      {
        action = qbi_svc_bc_ext_lte_attach_status_q_get_next_profile(qmi_txn->parent);
      }
      else
      {
        info->status_registered = FALSE;
        if (qbi_svc_lte_attach_status_prepare_rsp(qmi_txn, profile_settings))
        {
          QBI_LOG_D_0("LTEAttachStatus::Q: Device had never camped on LTE.");
          action = QBI_SVC_ACTION_SEND_RSP;
        }
      }
    }
    else
    {
      QBI_LOG_D_0("LTEAttachStatus::Q: Device is not registered on any RAT.");
      if (qbi_svc_lte_attach_status_prepare_rsp(qmi_txn, profile_settings))
      {
        action = QBI_SVC_ACTION_SEND_RSP;
      }
    }
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_status_q_dsd24_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_status_q_wds85_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_GET_LTE_ATTACH_PARAMS_RESP for
    MBIM_CID_MS_LTE_ATTACH_CONFIG query

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_status_q_wds85_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_get_lte_attach_params_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_lte_attach_status_info_s *info = NULL;
  dsd_get_system_status_req_msg_v01 *qmi_req_dsd = NULL;
  wds_get_lte_max_attach_pdn_num_req_msg_v01 *qmi_req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (wds_get_lte_attach_params_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    if (qmi_rsp->resp.error == QMI_ERR_INFO_UNAVAILABLE_V01)
    {
      QBI_LOG_E_0("LTEAttachStatus::Q: LTE not attached. "
        "Check for other available data capable RATs");
      qmi_req_dsd = (dsd_get_system_status_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(qmi_txn->parent, QBI_QMI_SVC_DSD,
          QMI_DSD_GET_SYSTEM_STATUS_REQ_V01,
          qbi_svc_bc_ext_lte_attach_status_q_dsd24_rsp_cb);
      QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req_dsd);

      action = QBI_SVC_ACTION_SEND_QMI_REQ;
    }
    else
    {
      QBI_LOG_E_1("LTEAttachStatus::Q: E:Received error code %d from QMI",
        qmi_rsp->resp.error);
    }
  }
  else
  {
    info = (qbi_svc_bc_ext_lte_attach_status_info_s *)qmi_txn->parent->info;
    QBI_CHECK_NULL_PTR_RET_ABORT(info);

    info->status_registered = TRUE;
    info->lte_attach_state = QBI_SVC_MBIM_MS_LTE_ATTACH_STATE_ATTACHED;

    if (qmi_rsp->apn_string_valid)
    {
      QBI_STRLCPY(info->apn_name, qmi_rsp->apn_string,
        sizeof(qmi_rsp->apn_string));

      info->ip_type = qmi_rsp->ip_type;
      QBI_LOG_STR_1("LTEAttachStatus::Q: APN: %s", info->apn_name);

      qmi_req = (wds_get_lte_max_attach_pdn_num_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(qmi_txn->parent, QBI_QMI_SVC_WDS,
          QMI_WDS_GET_LTE_MAX_ATTACH_PDN_NUM_REQ_V01,
          qbi_svc_bc_ext_lte_attach_status_q_wds92_rsp_cb);
      QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

      action = QBI_SVC_ACTION_SEND_QMI_REQ;
    }
    else
    {
      QBI_LOG_E_0("LTEAttachStatus::Q: E: APN TLV missing");
    }
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_status_q_wds85_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_status_q_wds92_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_GET_LTE_MAX_ATTACH_PDN_NUM_RESP for
    MBIM_CID_MS_LTE_ATTACH_CONFIG query

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_status_q_wds92_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_get_lte_attach_pdn_list_req_msg_v01 *qmi_req = NULL;
  wds_get_lte_max_attach_pdn_num_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_lte_attach_status_info_s *info = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (wds_get_lte_max_attach_pdn_num_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("LTEAttachStatus::Q: E:Received error code %d from QMI",
      qmi_rsp->resp.error);
  }
  else
  {
    info = (qbi_svc_bc_ext_lte_attach_status_info_s *)qmi_txn->parent->info;
    QBI_CHECK_NULL_PTR_RET_ABORT(info);
    info->max_supported_profile_num = qmi_rsp->max_attach_pdn_num;
    QBI_LOG_D_1("LTEAttachStatus::Q: max_supported_profile_num: %d",
      info->max_supported_profile_num);

    qmi_req = (wds_get_lte_attach_pdn_list_req_msg_v01 *)
      qbi_qmi_txn_alloc_ret_req_buf(
        qmi_txn->parent, QBI_QMI_SVC_WDS,
        QMI_WDS_GET_LTE_ATTACH_PDN_LIST_REQ_V01,
        qbi_svc_bc_ext_lte_attach_status_q_wds94_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }
  return action;
} /* qbi_svc_bc_ext_lte_attach_status_q_wds92_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_status_q_wds94_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_GET_LTE_ATTACH_PDN_LIST_RESP for
    MBIM_CID_MS_LTE_ATTACH_CONFIG query

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_status_q_wds94_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  wds_get_lte_attach_pdn_list_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_lte_attach_status_info_s *info = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (wds_get_lte_attach_pdn_list_resp_msg_v01 *) qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("LTEAttachStatus::Q: E:Received error code %d from QMI",
      qmi_rsp->resp.error);
  }
  else
  {
    info = (qbi_svc_bc_ext_lte_attach_status_info_s *)qmi_txn->parent->info;
    QBI_CHECK_NULL_PTR_RET_ABORT(info);
    if (qmi_rsp->attach_pdn_list_valid)
    {
      info->num_of_profile = qmi_rsp->attach_pdn_list_len;
    }

    if (info->num_of_profile <= info->max_supported_profile_num)
    {
      uint32 profile_count = 0;

      info->num_of_profile = qmi_rsp->attach_pdn_list_len;

      for (profile_count = 0; profile_count < info->num_of_profile; profile_count++)
      {
        info->profile_index[profile_count] =
          qmi_rsp->attach_pdn_list[profile_count];
        QBI_LOG_D_2("LTEAttachStatus::Q: Adding profile_index[%d]: %d",
          profile_count, info->profile_index[profile_count]);
        break;
      }

      QBI_LOG_D_1("LTEAttachStatus::Q: num_of_profile: %d", info->num_of_profile);

      info->profiles_read = 0;
      action = qbi_svc_bc_ext_lte_attach_status_q_get_next_profile(qmi_txn->parent);
    }
    else
    {
      QBI_LOG_E_2("LTEAttachStatus::Q: Attach PDN List is either empty or "
        "exceeds max supported PDN. attach_pdn_list_len = %d and "
        "max_supported_profile_num = %d", info->num_of_profile, 
        info->max_supported_profile_num);
    }
  }
  return action;
} /* qbi_svc_bc_ext_lte_attach_status_q_wds94_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_status_q_wds2b_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_WDS_GET_PROFILE_SETTINGS_RESP for
    MBIM_CID_MS_LTE_ATTACH_CONFIG query

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_status_q_wds2b_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  wds_get_profile_settings_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_lte_attach_status_info_s *info = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  qbi_svc_bc_ext_lte_attach_status_profile_settings_s profile_settings = { 0 };
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (wds_get_profile_settings_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("LTEAttachStatus::Q: Received error code %d from QMI",
      qmi_rsp->resp.error);
    qbi_svc_bc_ext_lte_attach_config_set_mbim_error_status(
      qmi_txn->parent, qmi_rsp->resp.error,
      qmi_rsp->extended_error_code_valid, qmi_rsp->extended_error_code);
  }
  else
  {
    info = (qbi_svc_bc_ext_lte_attach_status_info_s *)qmi_txn->parent->info;
    qbi_svc_bc_ext_lte_attach_status_q_populate_profile_settings(
      qmi_rsp, &profile_settings);

    if (qbi_svc_lte_attach_status_q_match_ip_type(
      info->ip_type, profile_settings.pdp_type))
    {
      if (qbi_svc_bc_ext_match_string(profile_settings.apn_name_valid,
        profile_settings.apn_name, info->apn_name))
      {
        QBI_LOG_D_0("LTEAttachStatus::Q: Attach profile found.");
        qbi_svc_lte_attach_status_q_update_cache(qmi_txn);
      }
      else
      {
        QBI_LOG_D_0("LTEAttachStatus::Q: Default attach profile. Add APN and "
          "IP type from LTE attach params to response.");

        QBI_MEMSET(&profile_settings, 0,
          sizeof(qbi_svc_bc_ext_lte_attach_status_profile_settings_s));

        profile_settings.apn_name_valid = 1;
        profile_settings.pdp_type_valid = 1;
        QBI_STRLCPY(profile_settings.apn_name,
          info->apn_name, sizeof(info->apn_name));
        profile_settings.pdp_type =
          qbi_svc_lte_attach_status_q_ip_type_to_pdp_type(info->ip_type);
      }

      if (qbi_svc_lte_attach_status_prepare_rsp(qmi_txn, profile_settings))
      {
        action = QBI_SVC_ACTION_SEND_RSP;
      }
    }
    else if (QBI_SVC_MBIM_MS_LTE_ATTACH_STATE_DETACHED == info->lte_attach_state)
    {
      if (qbi_svc_lte_attach_status_prepare_rsp(qmi_txn, profile_settings))
      {
        action = QBI_SVC_ACTION_SEND_RSP;
      }
    }
    else
    {
      info->profiles_read++;
      action = qbi_svc_bc_ext_lte_attach_status_q_get_next_profile(
        qmi_txn->parent);
    }
  }

  return action;
} /* qbi_svc_bc_ext_lte_attach_status_q_wds2b_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_status_reg_ind_dsd25_rsp
===========================================================================*/
/*!
  @brief Handles QMI_DSD_SYSTEM_STATUS_CHANGE_RESP

  @details

  @param qmi_txn

  @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_lte_attach_status_reg_ind_dsd25_rsp
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  dsd_system_status_change_resp_msg_v01 *qmi_rsp = NULL;
  uim_event_reg_req_msg_v01* qmi_req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  QBI_LOG_D_0("Received QMI_DSD_SYSTEM_STATUS_CHANGE_RESP");

  qmi_rsp = (dsd_system_status_change_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
  }
  else
  {
    /* Register for QMI_UIM_STATUS_CHANGE_IND */
    qmi_req = (uim_event_reg_req_msg_v01 *)
      qbi_qmi_txn_alloc_ret_req_buf(
        qmi_txn->parent, QBI_QMI_SVC_UIM, QMI_UIM_EVENT_REG_REQ_V01,
        qbi_svc_bc_ext_open_configure_qmi_inds_uim2e_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);
    qmi_req->event_mask = (1 << QMI_UIM_EVENT_CARD_STATUS_BIT_V01);

    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }
  return action;
}/* qbi_svc_bc_ext_lte_attach_status_reg_ind_dsd25_rsp() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_status_e_dsd26_ind_cb
===========================================================================*/
/*!
    @brief Handles a QMI_DSD_SYSTEM_STATUS_IND, looking for changes to the
    current channel rate or data system status that would trigger an
    MBIM_CID_PACKET_SERVICE event

    @details

    @param ind

    @return qbi_svc_action_e
*/
/*=========================================================================*/
qbi_svc_action_e qbi_svc_bc_ext_lte_attach_status_e_dsd26_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
)
{
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(ind);
  QBI_CHECK_NULL_PTR_RET_ABORT(ind->txn);

  QBI_LOG_D_0("Received DSD system status indication");
  return qbi_svc_bc_ext_lte_attach_status_q_req(ind->txn);
} /* qbi_svc_bc_ext_lte_attach_status_e_dsd26_ind_cb() */

/*! @} */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_lte_attach_config_set_mbim_error_status
===========================================================================*/
/*!
    @brief Attempts to map QMI error information into a descriptive MBIM
    error status for MBIM_CID_MS_LTE_ATTACH_CONFIG

    @details

    @param txn
    @param qmi_error
    @param qmi_error_ds_ext_valid
    @param qmi_error_ds_ext
*/
/*=========================================================================*/
static void qbi_svc_bc_ext_lte_attach_config_set_mbim_error_status
(
  qbi_txn_s                          *txn,
  qmi_error_type_v01                  qmi_error,
  uint8_t                             qmi_error_ds_ext_valid,
  wds_ds_extended_error_code_enum_v01 qmi_error_ds_ext
)
{
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET(txn);

  /* For all aborts setting the status, hence setting this to
  FALSE to handle new request */
  cmd_in_progress_ignore_indication = FALSE;
  
  /* Map extended error first, then map QMI error  */
  if (qmi_error == QMI_ERR_EXTENDED_INTERNAL_V01 && qmi_error_ds_ext_valid)
  {
    QBI_LOG_E_1("DS Profile extended error code 0x%x", qmi_error_ds_ext);
    switch (qmi_error_ds_ext)
    {
    case WDS_EEC_DS_PROFILE_REG_RESULT_ERR_LIB_NOT_INITED_V01:
      txn->status = QBI_MBIM_STATUS_NOT_INITIALIZED;
      break;

    case WDS_EEC_DS_PROFILE_REG_RESULT_ERR_LEN_INVALID_V01:
      txn->status = QBI_MBIM_STATUS_INVALID_PARAMETERS;
      break;

    case WDS_EEC_DS_PROFILE_REG_3GPP_ACCESS_ERR_V01:
    case WDS_EEC_DS_PROFILE_3GPP_ACCESS_ERR_V01:
      txn->status = QBI_MBIM_STATUS_READ_FAILURE;
      break;

    case WDS_EEC_DS_PROFILE_REG_3GPP_ERR_OUT_OF_PROFILES_V01:
    case WDS_EEC_DS_PROFILE_REG_3GPP_READ_ONLY_FLAG_SET_V01:
    case WDS_EEC_DS_PROFILE_3GPP_ERR_OUT_OF_PROFILES_V01:
    case WDS_EEC_DS_PROFILE_3GPP_READ_ONLY_FLAG_SET_V01:
      txn->status = QBI_MBIM_STATUS_WRITE_FAILURE;
      break;

    default:
      txn->status = QBI_MBIM_STATUS_FAILURE;
    }
  }
  else
  {
    switch (qmi_error)
    {
    case QMI_ERR_INVALID_PROFILE_V01:
      txn->status = QBI_MBIM_STATUS_INVALID_PARAMETERS;
      break;

    case QMI_ERR_NO_FREE_PROFILE_V01:
      txn->status = QBI_MBIM_STATUS_WRITE_FAILURE;
      break;

    default:
      txn->status = QBI_MBIM_STATUS_FAILURE;
    }
  }
} /* qbi_svc_bc_ext_lte_attach_config_set_mbim_error_status() */

/*! @addtogroup MBIM_CID_DEVICE_CAPS
    @{ */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_open_configure_qmi_radio_if_list_to_mbim_data_class
===========================================================================*/
/*!
    @brief Convert QMI radio interface array to MBIM data class mask

    @details

    @param dms_radio_if_enum_v01
    @param uint32

    @return uint32
*/
/*=========================================================================*/
static uint32 qbi_svc_bc_ext_open_configure_qmi_radio_if_list_to_mbim_data_class
(
    const dms_radio_if_enum_v01 *radio_if_list,
    uint32                       radio_if_list_len
)
{
  uint32 i;
  uint32 data_class = QBI_SVC_BC_DATA_CLASS_NONE;
/*-------------------------------------------------------------------------*/
  /* Construct data class according to available technologies */
  if (radio_if_list_len > QMI_DMS_RADIO_IF_LIST_MAX_V01)
  {
      QBI_LOG_E_1("Invalid radio IF list length %d", radio_if_list_len);
  }
  else
  {
      for (i = 0; i < radio_if_list_len; i++)
      {
          switch (radio_if_list[i])
          {
          case DMS_RADIO_IF_1X_V01:
              data_class |= QBI_SVC_BC_DATA_CLASS_1XRTT;
              break;

          case DMS_RADIO_IF_1X_EVDO_V01:
              data_class |= (QBI_SVC_BC_DATA_CLASS_1XRTT |
                QBI_SVC_BC_DATA_CLASS_1XEVDO |
                  QBI_SVC_BC_DATA_CLASS_1XEVDO_REVA |
                  QBI_SVC_BC_DATA_CLASS_1XEVDO_REVB);
              break;

          case DMS_RADIO_IF_GSM_V01:
              data_class |= (QBI_SVC_BC_DATA_CLASS_GPRS |
                  QBI_SVC_BC_DATA_CLASS_EDGE);
              break;

          case DMS_RADIO_IF_UMTS_V01:
              data_class |= (QBI_SVC_BC_DATA_CLASS_UMTS |
                  QBI_SVC_BC_DATA_CLASS_HSDPA |
                  QBI_SVC_BC_DATA_CLASS_HSUPA);
              break;

          case DMS_RADIO_IF_LTE_V01:
              data_class |= QBI_SVC_BC_DATA_CLASS_LTE;
              break;

          case DMS_RADIO_IF_TDS_V01:
              data_class |= QBI_SVC_BC_DATA_CLASS_CUSTOM;
              break;

          default:
              QBI_LOG_E_1("Couldn't identify data class %d!",
                  radio_if_list[i]);
              break;
          }
      }
  }

  return data_class;
} /* qbi_svc_bc_ext_open_configure_qmi_radio_if_list_to_mbim_data_class() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_sys_caps_info_q_uim2f_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QBI_SVC_BC_EXT_MBIM_CID_MS_SYS_CAPS response

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_sys_caps_info_q_uim2f_rsp_cb
(
    qbi_qmi_txn_s *qmi_txn
)
{
  uim_get_card_status_resp_msg_v01 *qmi_rsp;
  qbi_svc_bc_ext_sys_caps_rsp_s *rsp;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  QBI_LOG_D_0("qbi_svc_bc_ext_sys_caps_info_q_uim2f_rsp_cb");
  rsp = (qbi_svc_bc_ext_sys_caps_rsp_s *)qmi_txn->parent->rsp.data;
  qmi_rsp = (uim_get_card_status_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
      QBI_LOG_E_1("received error code %d from qmi", qmi_rsp->resp.error);
  }
  else
  {
      if (!qmi_rsp->card_status_valid)
      {
          QBI_LOG_E_0("leaving deviceid field blank: number of slots not provisioned");
      }
      else
      {
          rsp->num_of_slots = qmi_rsp->card_status.card_info_len;
          QBI_LOG_D_1("num_of_slots %d", rsp->num_of_slots);
      }
      action = qbi_svc_bc_ext_device_caps_q_rsp(qmi_txn->parent);
  }

  return action;
}/* qbi_svc_bc_ext_sys_caps_info_q_uim2f_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_sys_caps_info_q_dms20_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QBI_SVC_BC_EXT_MBIM_CID_MS_SYS_CAPS response

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_sys_caps_info_q_dms20_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  dms_get_device_cap_resp_msg_v01 *qmi_rsp;
  qbi_svc_bc_ext_sys_caps_rsp_s *rsp;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  QBI_LOG_D_0("qbi_svc_bc_ext_sys_caps_info_q_dms20_rsp_cb");
  rsp = (qbi_svc_bc_ext_sys_caps_rsp_s *)qmi_txn->parent->rsp.data;
  qmi_rsp = (dms_get_device_cap_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
      QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
  }
  else
  {
      if (!qmi_rsp->max_active_data_subscriptions_valid)
      {
          QBI_LOG_W_0("Leaving DeviceId field blank: invalid concurrency or num of executors");
      }
      else
      {
          rsp->concurrency = qmi_rsp->max_active_data_subscriptions;
          rsp->num_of_executors = qmi_rsp->max_active_data_subscriptions;
          QBI_LOG_D_1("max_active_data_subscriptions %d ", qmi_rsp->max_active_data_subscriptions);
      }
      action = qbi_svc_bc_ext_device_caps_q_rsp(qmi_txn->parent);

  }

  return action;
}/* qbi_svc_bc_ext_sys_caps_info_q_dms20_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_sys_caps_info_q_dms25_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QBI_SVC_BC_EXT_MBIM_CID_MS_SYS_CAPS response

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_sys_caps_info_q_dms25_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  dms_get_device_serial_numbers_resp_msg_v01 *qmi_rsp;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  QBI_LOG_D_0("qbi_svc_bc_ext_sys_caps_info_q_dms25_rsp_cb");

  qmi_rsp = (dms_get_device_serial_numbers_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
      QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
  }
  else
  {
      if (!qmi_rsp->meid_valid)
      {
          QBI_LOG_D_0("Leaving DeviceId field blank: invalid modemid");
      }
      else
      {
        qbi_svc_bc_ext_sys_caps_info_q_get_meid(qmi_txn);
      }
      action = qbi_svc_bc_ext_device_caps_q_rsp(qmi_txn->parent);

  }

  return action;
}/* qbi_svc_bc_ext_sys_caps_info_q_dms25_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_sys_caps_info_q_get_meid
===========================================================================*/
/*!
    @brief Handles a MEID extraction

    @details

    @param qmi_txn

    @return none
*/
/*=========================================================================*/
static void qbi_svc_bc_ext_sys_caps_info_q_get_meid
(
  qbi_qmi_txn_s *qmi_txn
)
{
  dms_get_device_serial_numbers_resp_msg_v01 *qmi_rsp;
  qbi_svc_bc_ext_sys_caps_rsp_s *rsp;
  int i = 0;
  uint32_t temp1 = 0;
  uint32_t temp2 = 0;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET(qmi_txn);
  QBI_CHECK_NULL_PTR_RET(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET(qmi_txn->parent->rsp.data);
  QBI_CHECK_NULL_PTR_RET(qmi_txn->rsp.data);

  rsp = (qbi_svc_bc_ext_sys_caps_rsp_s *)qmi_txn->parent->rsp.data;
  qmi_rsp = (dms_get_device_serial_numbers_resp_msg_v01 *)qmi_txn->rsp.data;

  /* An MEID is 56 bits long (14 hex digits) */
  /* Copying 1st hex digit into variable temp1 */
  temp1 =  qbi_svc_bc_ext_sys_caps_info_q_get_meid_char_to_int (qmi_rsp->meid[i]);
  /* Copying rest five hex digits into variable temp1, so repeating loop from
     1 to 5 hex digits */
  for (i = 1; i < 6;i++)
  {
    temp1 = temp1 << 4;
    temp1 |=  qbi_svc_bc_ext_sys_caps_info_q_get_meid_char_to_int (qmi_rsp->meid[i]);
  }

  /* Copying 6th hex digit into variable temp2 */
  temp2 =  qbi_svc_bc_ext_sys_caps_info_q_get_meid_char_to_int (qmi_rsp->meid[i]);
  /* Copying rest seven hex digits into variable temp2, so repeating loop from
     7 to 13 hex digits */
  for (i = 7; i < 14;i++)
  {
    temp2 = temp2 << 4;
    temp2 |=  qbi_svc_bc_ext_sys_caps_info_q_get_meid_char_to_int (qmi_rsp->meid[i]);
  }

  /* Shifting temp1 6 hex values to MSB of rsp->modem_id */
  rsp->modem_id = (0x00000000FFFFFFFFull & temp1) << 32;
  /* Inserting temp2 into LSB of rsp->modem_id */
  rsp->modem_id = rsp->modem_id | temp2;

} /* qbi_svc_bc_ext_sys_caps_info_q_get_meid */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_sys_caps_info_q_get_meid_char_to_int
===========================================================================*/
/*!
    @brief Handles a MEID extraction

    @details

    @param qmi_txn

    @return none
*/
/*=========================================================================*/
static uint32 qbi_svc_bc_ext_sys_caps_info_q_get_meid_char_to_int
(
  char meid_char
)
{
  uint32_t meid_int = 0;
/*-------------------------------------------------------------------------*/
  if ((meid_char >= MEIDCHAR_0) && (meid_char <= MEIDCHAR_9))
  {
   /* characters 0 to 9 conversion to integer*/
    meid_int |= (meid_char - MEIDCHAR_0);
  }
  else if ((meid_char >= MEIDCHAR_A) && (meid_char <= MEIDCHAR_F))
  {
    /* characters A to F conversion to integer*/
    meid_int |= (meid_char - (MEIDCHAR_A - 10));
  }
  return meid_int;
}/* qbi_svc_bc_ext_sys_caps_info_q_get_meid_char_to_int */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_sys_caps_info_q_req
===========================================================================*/
/*!
    @brief Handles a QBI_SVC_BC_EXT_MBIM_CID_MS_SYS_CAPS query request

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_sys_caps_info_q_req
(
  qbi_txn_s *txn
)
{
  qbi_qmi_txn_s *qmi_txn = NULL;
  uim_get_card_status_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_sys_caps_rsp_s *rsp = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;

/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  qbi_txn_req_buf_ignored(txn);

  rsp = qbi_txn_alloc_rsp_buf(txn, sizeof(qbi_svc_bc_ext_sys_caps_rsp_s));
  QBI_CHECK_NULL_PTR_RET_ABORT(rsp);

  /* QMI_DMS_GET_DEVICE_SERIAL_NUMBERS (0x25) */
  qmi_txn = qbi_qmi_txn_alloc(txn, QBI_QMI_SVC_DMS,
        QMI_DMS_GET_DEVICE_SERIAL_NUMBERS_REQ_V01,
        qbi_svc_bc_ext_sys_caps_info_q_dms25_rsp_cb);

  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);

  /* QMI_DMS_GET_DEVICE_CAP (0x20) */
  qmi_txn = qbi_qmi_txn_alloc(txn, QBI_QMI_SVC_DMS,
        QMI_DMS_GET_DEVICE_CAP_REQ_V01,
        qbi_svc_bc_ext_sys_caps_info_q_dms20_rsp_cb);

  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);

  /* QMI_UIM_GET_CARD_STATUS (0x2f) */
  qmi_req = (uim_get_card_status_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(
            txn, QBI_QMI_SVC_UIM, QMI_UIM_GET_CARD_STATUS_REQ_V01,
            qbi_svc_bc_ext_sys_caps_info_q_uim2f_rsp_cb);

  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  action = QBI_SVC_ACTION_SEND_QMI_REQ;

  return action;
}/* qbi_svc_bc_ext_sys_caps_info_q_req() */

/*! @} */

/*! @addtogroup MBIM_CID_DEVICE_CAPS_V2
    @{ */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_device_caps_v2_q_req
===========================================================================*/
/*!
    @brief Handles a MBIM_CID_DEVICE_CAPS_V2 query

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_device_caps_v2_q_req
(
  qbi_txn_s *txn
)
{
  qbi_svc_bc_ext_device_caps_v2_rsp_s *rsp;
  qbi_qmi_txn_s *qmi_txn = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  qbi_txn_req_buf_ignored(txn);

  rsp = qbi_txn_alloc_rsp_buf(txn, sizeof(qbi_svc_bc_ext_device_caps_v2_rsp_s));
  QBI_CHECK_NULL_PTR_RET_ABORT(rsp);

  rsp->voice_class = QBI_SVC_BC_VOICE_CLASS_NO_VOICE;
  rsp->sms_caps = (QBI_SVC_BC_SMS_CAPS_PDU_SEND |
      QBI_SVC_BC_SMS_CAPS_PDU_RECEIVE);
  rsp->sim_class = QBI_SVC_BC_SIM_CLASS_REMOVABLE;
  rsp->max_sessions = QBI_SVC_BC_EXT_MAX_SESSIONS;
  rsp->cellular_class = QBI_SVC_BC_CELLULAR_CLASS_GSM;
  rsp->ctrl_caps = QBI_SVC_BC_CTRL_CAPS_REG_MANUAL;

  /* DeviceType given from NV configuration item */
  if (!qbi_nv_store_cfg_item_read(
      txn->ctx, QBI_NV_STORE_CFG_ITEM_DEVICE_TYPE, (void *) &rsp->device_type,
      sizeof(rsp->device_type)))
  {
    rsp->device_type = QBI_SVC_BC_DEVICE_TYPE_UNKNOWN;
  }
  
  if (qbi_svc_bc_device_supports_3gpp2(txn->ctx))
  {
    rsp->cellular_class |= QBI_SVC_BC_CELLULAR_CLASS_CDMA;
    rsp->ctrl_caps |= (QBI_SVC_BC_CTRL_CAPS_CDMA_SIMPLE_IP |
        QBI_SVC_BC_CTRL_CAPS_CDMA_MOBILE_IP);
  }

  /* QMI_DMS_GET_DEVICE_SERIAL_NUMBERS (0x25) */
  qmi_txn = qbi_qmi_txn_alloc(txn, QBI_QMI_SVC_DMS,
      QMI_DMS_GET_DEVICE_SERIAL_NUMBERS_REQ_V01,
      qbi_svc_bc_ext_device_caps_info_q_dms25_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);

  /* QMI_DMS_GET_DEVICE_REV_ID (0x23) */
  qmi_txn = qbi_qmi_txn_alloc(txn, QBI_QMI_SVC_DMS,
      QMI_DMS_GET_DEVICE_REV_ID_REQ_V01,
      qbi_svc_bc_ext_device_caps_q_dms23_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);

  /* QMI_DMS_GET_DEVICE_MODEL_ID (0x22) */
  qmi_txn = qbi_qmi_txn_alloc(txn, QBI_QMI_SVC_DMS,
      QMI_DMS_GET_DEVICE_MODEL_ID_REQ_V01,
      qbi_svc_bc_ext_device_caps_q_dms22_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);

  /* QMI_DMS_GET_GET_DEVICE_CAP (0x20) */
  qmi_txn = qbi_qmi_txn_alloc(txn, QBI_QMI_SVC_DMS,
      QMI_DMS_GET_DEVICE_CAP_REQ_V01,
      qbi_svc_bc_ext_device_caps_q_dms20_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);

  action = QBI_SVC_ACTION_SEND_QMI_REQ;

  return action;
} /* qbi_svc_bc_ext_device_caps_v2_q_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_device_caps_q_rsp
===========================================================================*/
/*!
    @brief Perform query response processing for MBIM_CID_DEVICE_CAPS

    @details
    Checks whether we have the information required to send the response.

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_device_caps_q_rsp
(
  qbi_txn_s *txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);

  if (txn->qmi_txns_pending == 0)
  {
      action = QBI_SVC_ACTION_SEND_RSP;
  }
  else
  {
      action = QBI_SVC_ACTION_WAIT_ASYNC_RSP;
  }

  return action;
} /* qbi_svc_bc_ext_device_caps_q_rsp() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_device_caps_q_dms20_rsp_cb
===========================================================================*/
/*!
    @brief Process a QMI_DMS_GET_DEVICE_CAPS_RESP for
    MBIM_CID_DEVICE_CAPS query

    @details
    Populates the executor_index field in the response.

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_device_caps_q_dms20_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  dms_get_device_cap_resp_msg_v01 *qmi_rsp;
  qbi_svc_bc_ext_device_caps_v2_rsp_s *rsp;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  QBI_LOG_D_0("qbi_svc_bc_ext_device_caps_q_dms20_rsp_cb");
  rsp = (qbi_svc_bc_ext_device_caps_v2_rsp_s *)qmi_txn->parent->rsp.data;
  qmi_rsp = (dms_get_device_cap_resp_msg_v01 *)qmi_txn->rsp.data;

  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
            QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
  }
  else
  {
    if (!qmi_rsp->max_active_data_subscriptions_valid)
    {
            QBI_LOG_E_0("Leaving DeviceId field blank: invalid executor_index");
    }
    else
    {
     //Implementation currently supports Single SIM and DSSA hence using default executor index
      rsp->executor_index = QBI_SVC_BC_EXT_DEFAULT_EXECUTOR_INDEX;
      QBI_LOG_D_1("executor_index %d", rsp->executor_index);
     }

     rsp->data_class = qbi_svc_bc_ext_open_configure_qmi_radio_if_list_to_mbim_data_class(
                       qmi_rsp->device_capabilities.radio_if_list,
                       qmi_rsp->device_capabilities.radio_if_list_len);
     if ((rsp->data_class & QBI_SVC_BC_DATA_CLASS_CUSTOM) != 0 &&
         !qbi_txn_rsp_databuf_add_string_ascii_to_utf16(
         qmi_txn->parent, &rsp->cust_data_class, 0, 
         QBI_SVC_BC_CUST_DATA_CLASS_MAX_LEN_BYTES,
         QBI_SVC_BC_CUSTOM_DATA_CLASS_NAME_ASCII,
         sizeof(QBI_SVC_BC_CUSTOM_DATA_CLASS_NAME_ASCII)))
     {
       QBI_LOG_E_0("Couldn't add CustomDataClass string to response");
     }
     action = qbi_svc_bc_ext_device_caps_q_rsp(qmi_txn->parent);
  }
  return action;
} /* qbi_svc_bc_ext_device_caps_q_dms20_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_device_caps_q_dms22_rsp_cb
===========================================================================*/
/*!
    @brief Process a QMI_DMS_GET_DEVICE_MODEL_ID_RESP for
    MBIM_CID_DEVICE_CAPS query

    @details
    Populates the HardwareInfo field in the response.

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_device_caps_q_dms22_rsp_cb
(
    qbi_qmi_txn_s *qmi_txn
)
{
  dms_get_device_model_id_resp_msg_v01 *qmi_rsp;
  qbi_svc_bc_ext_device_caps_v2_rsp_s *rsp;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  QBI_LOG_D_0("qbi_svc_bc_ext_device_caps_q_dms22_rsp_cb");
  rsp = (qbi_svc_bc_ext_device_caps_v2_rsp_s *)qmi_txn->parent->rsp.data;
  qmi_rsp = (dms_get_device_model_id_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
      QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
  }
  else
  {
    if (!qbi_txn_rsp_databuf_add_string_ascii_to_utf16(
      qmi_txn->parent, &rsp->hw_info, 0, QBI_SVC_BC_HW_INFO_MAX_LEN_BYTES,
      qmi_rsp->device_model_id, sizeof(qmi_rsp->device_model_id)))
    {
      QBI_LOG_E_0("Couldn't add hardware information to response!");
    }
    else
    {
      action = qbi_svc_bc_ext_device_caps_q_rsp(qmi_txn->parent);
    }
  }

  return action;
} /* qbi_svc_bc_ext_device_caps_q_dms22_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_device_caps_q_dms23_rsp_cb
===========================================================================*/
/*!
    @brief Process a QMI_DMS_GET_DEVICE_REV_ID_RESP for
    MBIM_CID_DEVICE_CAPS query

    @details
    Populates the FirmwareInfo field in the response.

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_device_caps_q_dms23_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  dms_get_device_rev_id_resp_msg_v01 *qmi_rsp;
  qbi_svc_bc_ext_device_caps_v2_rsp_s *rsp;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  uint32 offset = 0;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  QBI_LOG_D_0("qbi_svc_bc_ext_device_caps_q_dms23_rsp_cb");
  rsp = (qbi_svc_bc_ext_device_caps_v2_rsp_s *)qmi_txn->parent->rsp.data;
  qmi_rsp = (dms_get_device_rev_id_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
      QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
  }
  else
  {
      /* The revision ID returned by QMI contains the boot block version and
      release date. These won't fit in the response, so truncate after the
      first space so that we only report the build ID */
      for (offset = 0; offset < sizeof(qmi_rsp->device_rev_id) &&
          qmi_rsp->device_rev_id[offset] != QBI_UTIL_ASCII_NULL; offset++)
      {
          if (qmi_rsp->device_rev_id[offset] == ' ')
          {
              qmi_rsp->device_rev_id[offset] = QBI_UTIL_ASCII_NULL;
              break;
          }
      }

      if (!qbi_txn_rsp_databuf_add_string_ascii_to_utf16(
          qmi_txn->parent, &rsp->fw_info, 0, QBI_SVC_BC_FW_INFO_MAX_LEN_BYTES,
          qmi_rsp->device_rev_id, sizeof(qmi_rsp->device_rev_id)))
      {
          QBI_LOG_E_0("Couldn't add firmware information to response!");
      }
      else
      {
          action = qbi_svc_bc_ext_device_caps_q_rsp(qmi_txn->parent);
      }
  }

  return action;
} /* qbi_svc_bc_ext_device_caps_q_dms23_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_device_caps_info_q_dms25_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QBI_SVC_BC_EXT_MBIM_CID_MS_DEVICE_CAPS response

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_device_caps_info_q_dms25_rsp_cb
(
    qbi_qmi_txn_s *qmi_txn
)
{
  dms_get_device_serial_numbers_resp_msg_v01 *qmi_rsp;
  qbi_svc_bc_ext_device_caps_v2_rsp_s *rsp;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  QBI_LOG_D_0("qbi_svc_bc_ext_device_caps_info_q_dms25_rsp_cb");
  rsp = (qbi_svc_bc_ext_device_caps_v2_rsp_s *)qmi_txn->parent->rsp.data;
  qmi_rsp = (dms_get_device_serial_numbers_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
      QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
  }
  else if (qmi_rsp->imei_valid &&
      !qbi_txn_rsp_databuf_add_string_ascii_to_utf16(
          qmi_txn->parent, &rsp->device_id, 0,
          QBI_SVC_BC_EXT_MODEM_ID_INFO_MAX_LEN_BYTES,
          qmi_rsp->imei, sizeof(qmi_rsp->imei)))
  {
      QBI_LOG_E_0("Couldn't populate device id in response!");
  }
  else
  {
      if (!qmi_rsp->imei_valid)
      {
          QBI_LOG_E_0("Leaving DeviceId field blank: invalid device id");
      }
      action = qbi_svc_bc_ext_device_caps_q_rsp(qmi_txn->parent);
  }

  return action;
}/* qbi_svc_bc_ext_device_caps_info_q_dms25_rsp_cb() */

/*! @} */
 
/*! @addtogroup MBIM_CID_MS_DEVICE_SLOT_MAPPING
    @{ */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_slot_mapping_q_req
===========================================================================*/
/*!
    @brief Handles a MBIM_CID_MS_DEVICE_SLOT_MAPPING query request

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_slot_mapping_q_req
(
  qbi_txn_s *txn
)
{
  uim_get_card_status_req_msg_v01 *qmi_req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);

  qmi_req = (uim_get_card_status_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(
      txn, QBI_QMI_SVC_UIM, QMI_UIM_GET_CARD_STATUS_REQ_V01,
      qbi_svc_bc_ext_slot_mapping_q_uim2f_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  return QBI_SVC_ACTION_SEND_QMI_REQ;
} /* qbi_svc_bc_ext_slot_mapping_q_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_slot_mapping_q_uim2f_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_UIM_GET_CARD_STATUS_RESP for
            MBIM_CID_MS_DEVICE_SLOT_MAPPING query

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_slot_mapping_q_uim2f_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  uim_get_card_status_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_slot_mapping_info_s *rsp = NULL;
  uint32 *slot_map_list = NULL;
  qbi_svc_bc_ext_exec_slot_config_s exec_slot_cfg;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (uim_get_card_status_resp_msg_v01 *)qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
    qmi_txn->parent->status = QBI_MBIM_STATUS_FAILURE;
  }
  else
  {
    if (qmi_rsp->card_status_valid)
    {
      if (!qbi_nv_store_cfg_item_read(qmi_txn->parent->ctx,
        QBI_NV_STORE_CFG_ITEM_EXECUTOR_SLOT_CONFIG, &exec_slot_cfg,
        sizeof(qbi_svc_bc_ext_exec_slot_config_s)))
      {
        QBI_LOG_E_0("Missing executor slot mapping info");
        qmi_txn->parent->status = QBI_MBIM_STATUS_FAILURE;
      }
      else
      {
        QBI_LOG_D_1("Total no. of slots = %d", qmi_rsp->card_status.card_info_len);
        QBI_LOG_D_1("Index gw pri = %d", qmi_rsp->card_status.index_gw_pri);
        QBI_LOG_D_1("Index 1x pri = %d", qmi_rsp->card_status.index_1x_pri);
        QBI_LOG_D_2("Card 1 info : num app = %d card state = %d",
          qmi_rsp->card_status.card_info[0].app_info_len,
          qmi_rsp->card_status.card_info[0].card_state);

        QBI_LOG_D_2("Card 2 info : num app = %d card state = %d",
          qmi_rsp->card_status.card_info[1].app_info_len,
          qmi_rsp->card_status.card_info[1].card_state);

        rsp = (qbi_svc_bc_ext_slot_mapping_info_s *)qbi_txn_alloc_rsp_buf(
          qmi_txn->parent, (sizeof(qbi_svc_bc_ext_slot_mapping_info_s)));
        QBI_CHECK_NULL_PTR_RET_ABORT(rsp);

        rsp->map_count = QBI_SVC_BC_EXT_MAX_SUPPORTED_EXECUTORS;

        slot_map_list = (uint32 *)qbi_txn_rsp_databuf_add_field(
          qmi_txn->parent, &rsp->slot_map_list, 0, sizeof(uint32), NULL);
        QBI_CHECK_NULL_PTR_RET_ABORT(slot_map_list);

        slot_map_list[0] = exec_slot_cfg.exec0_slot;

        qbi_txn_rsp_databuf_consolidate(qmi_txn->parent);
      }
    }
    else
    {
      qmi_txn->parent->status = QBI_MBIM_STATUS_FAILURE;
    }
  }

  return QBI_SVC_ACTION_SEND_RSP;
} /* qbi_svc_bc_ext_slot_mapping_q_uim2f_rsp_cb() */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_slot_mapping_s_req
===========================================================================*/
/*!
    @brief Handles a MBIM_CID_MS_DEVICE_SLOT_MAPPING set request

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_slot_mapping_s_req
(
  qbi_txn_s *txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  uim_get_card_status_req_msg_v01 *qmi_req = NULL;
  uint32 req_slot = 0;
  qbi_svc_bc_ext_slot_mapping_info_s *req = NULL;
  qbi_svc_bc_ext_exec_slot_config_s exec_slot_cfg;
  qbi_svc_bc_ext_slot_map_cmd_info_s *info = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);

  req = (qbi_svc_bc_ext_slot_mapping_info_s *)txn->req.data;

  qmi_req = (uim_get_card_status_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(
      txn, QBI_QMI_SVC_UIM, QMI_UIM_GET_CARD_STATUS_REQ_V01,
      qbi_svc_bc_ext_slot_mapping_s_uim2f_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  /* Extracting requested slot */
  req_slot = (uint32)(*((uint8*)req + req->slot_map_list.offset));
  if((req_slot != QBI_SVC_BC_EXT_SLOT_1) && (req_slot != QBI_SVC_BC_EXT_SLOT_2))
  {
    QBI_LOG_E_1("Error: Invalid Requested slot %d ", req_slot);
    action = QBI_SVC_ACTION_ABORT;
  }
  else
  {
     if (!qbi_nv_store_cfg_item_read(txn->ctx,
        QBI_NV_STORE_CFG_ITEM_EXECUTOR_SLOT_CONFIG, &exec_slot_cfg,
        sizeof(qbi_svc_bc_ext_exec_slot_config_s)))
    {
      QBI_LOG_E_0("Missing executor slot mapping info");
      txn->status = QBI_MBIM_STATUS_FAILURE;
      action = QBI_SVC_ACTION_ABORT;
    }
    else
    {
      txn->info = QBI_MEM_MALLOC_CLEAR(
       sizeof(qbi_svc_bc_ext_slot_map_cmd_info_s));
      QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);
      info = (qbi_svc_bc_ext_slot_map_cmd_info_s *)txn->info;
      info->curr.slot = exec_slot_cfg.exec0_slot;
      action = QBI_SVC_ACTION_SEND_QMI_REQ;
    }
  }

  return action;
} /* qbi_svc_bc_ext_slot_mapping_s_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_slot_mapping_s_uim2f_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_UIM_GET_CARD_STATUS_RESP for
            MBIM_CID_MS_DEVICE_SLOT_MAPPING query

    @details This function handles activation and deactivation of 
             requested GW or 1x_pri slot 

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_slot_mapping_s_uim2f_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  uim_get_card_status_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_slot_mapping_info_s *req = NULL;
  qbi_svc_bc_ext_slot_map_cmd_info_s *info = NULL;
  qbi_svc_bc_ext_exec_slot_config_s exec_slot_cfg;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);
  
  do
  {
    qmi_rsp = (uim_get_card_status_resp_msg_v01 *)qmi_txn->rsp.data;

    if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
    {
      QBI_LOG_E_1("Received error code %d from QMI", qmi_rsp->resp.error);
      break;
    }

    info = (qbi_svc_bc_ext_slot_map_cmd_info_s *)qmi_txn->parent->info;
    req = (qbi_svc_bc_ext_slot_mapping_info_s *)qmi_txn->parent->req.data;

    /* Currently supporting only 1 executor */
    if(req->map_count > QBI_SVC_BC_EXT_MAX_SUPPORTED_EXECUTORS)
    {
      /* Response*/
      qmi_txn->parent->status = QBI_MBIM_STATUS_FAILURE;
      QBI_LOG_E_1("Error: currently supporting only 1 executor %d", req->map_count);
      break;
    }

    if (!info->cmd_info_extracted)
    {
      info->cmd_info_extracted = TRUE;
      qbi_svc_bc_ext_populate_app_slot_info(qmi_txn);
    }

    if (info->total_num_slots <= 1)
    {
      action = qbi_svc_bc_ext_slot_mapping_q_req(qmi_txn->parent);
      QBI_LOG_E_1("Only 1 slot available %d. No change possible",info->req.slot);
      break;    
    }

    
    QBI_LOG_D_2("info->curr.slot %d info->req.slot %d",info->curr.slot,info->req.slot);

    if (info->curr.slot == info->req.slot)
    {
      action = qbi_svc_bc_ext_slot_mapping_q_req(qmi_txn->parent);
      QBI_LOG_E_2("Requested slot %d matches existing mapping %d",info->req.slot, info->curr.slot);
      break;
    }

    /* Setting prov_complete to FALSE to indicate de-activation/activation is in progess */
    exec_slot_cfg.exec0_prov_complete = FALSE;
    exec_slot_cfg.exec0_slot = info->curr.slot;
    if (!qbi_nv_store_cfg_item_write(
      qmi_txn->ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_SLOT_CONFIG,
      &exec_slot_cfg, sizeof(qbi_svc_bc_ext_exec_slot_config_s)))
    {
      QBI_LOG_E_0("Couldn't save new switched slot to NV!");
    }

    if (!info->curr.info_gw.deact_done || !info->curr.info_1x.deact_done)
    {
      action = qbi_svc_bc_ext_slot_mapping_s_deact_req(qmi_txn);
      QBI_LOG_D_0("Deactivation in progress");
      break;
    }

    if (qbi_svc_bc_ext_slot_mapping_close_logical_channels(qmi_txn))
    {
      QBI_LOG_D_0("Closing logical channels");
      action = QBI_SVC_ACTION_SEND_QMI_REQ;
      break;
    }

    /* Update mutlisim configuration NV to ensure it has latest slot mapping */
    exec_slot_cfg.exec0_slot = info->req.slot;
    /* Keeping prov_complete as FALSE since activation stage is not yet complete */
    exec_slot_cfg.exec0_prov_complete = FALSE;
    if (!qbi_nv_store_cfg_item_write(
      qmi_txn->ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_SLOT_CONFIG,
      &exec_slot_cfg, sizeof(qbi_svc_bc_ext_exec_slot_config_s)))
    {
      QBI_LOG_E_0("Couldn't save new switched slot to NV during deact");
    }

    if ((info->req.info_gw.act_need && !info->req.info_gw.act_done) ||
        (info->req.info_1x.act_need && !info->req.info_1x.act_done))
    {
      action = qbi_svc_bc_ext_slot_mapping_s_act_req(qmi_txn);
      QBI_LOG_D_0("Activation in progress");
      break;
    }

    /* Update mutlisim configuration NV to ensure it has latest slot mapping */
    exec_slot_cfg.exec0_slot = info->req.slot;
    /* Setting prov_complete to TRUE to indicate de-activation/activation completed */
    exec_slot_cfg.exec0_prov_complete = TRUE;
    if (!qbi_nv_store_cfg_item_write(
      qmi_txn->ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_SLOT_CONFIG,
      &exec_slot_cfg, sizeof(qbi_svc_bc_ext_exec_slot_config_s)))
    {
      QBI_LOG_E_0("Couldn't save new switched slot to NV!");
    }
    else
    {
      /* sending indication for subscriber ready state*/
      if (info->req.info_gw.act_need == FALSE && info->req.info_1x.act_need == FALSE)
      {
        qbi_svc_force_event(
          qmi_txn->parent->ctx, QBI_SVC_ID_BC,
          QBI_SVC_BC_MBIM_CID_SUBSCRIBER_READY_STATUS);
      }
    }

    action = qbi_svc_bc_ext_slot_mapping_q_req(qmi_txn->parent);
    QBI_LOG_D_2("Slot mapping complete. Switched from slot %d to slot %d",info->curr.slot, info->req.slot);

  }while(0);
  return action;
} /* qbi_svc_bc_ext_slot_mapping_s_uim2f_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_populate_app_slot_info
===========================================================================*/
/*!
    @brief Handles a QMI_UIM_GET_CARD_STATUS_RESP for
            MBIM_CID_MS_DEVICE_SLOT_MAPPING query

    @details This function handles deactivation of requested 1x_pri slot 

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static void qbi_svc_bc_ext_populate_app_slot_info
(
  qbi_qmi_txn_s *qmi_txn  
)
{
  qbi_svc_bc_ext_slot_mapping_info_s *req;
  qbi_svc_bc_ext_slot_map_cmd_info_s *info;
  uim_get_card_status_resp_msg_v01   *qmi_rsp = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET(qmi_txn);
  QBI_CHECK_NULL_PTR_RET(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET(qmi_txn->parent->info);
  QBI_CHECK_NULL_PTR_RET(qmi_txn->rsp.data);

  info = (qbi_svc_bc_ext_slot_map_cmd_info_s *)qmi_txn->parent->info;
  qmi_rsp = 
   (uim_get_card_status_resp_msg_v01 *)qmi_txn->rsp.data;
    
   /* Total number of slots */
  info->total_num_slots = qmi_rsp->card_status.card_info_len;
  QBI_LOG_D_1("Total number of slots %d", info->total_num_slots);

  QBI_LOG_D_1("info->curr.slot = %d", info->curr.slot);

  if(info->curr.slot != 0xFF)
  {
    info->curr.info_gw.appindex = 
      QBI_SVC_BC_EXT_GET_APP_INDEX(qmi_rsp->card_status.index_gw_pri);
    info->curr.info_1x.appindex = 
     QBI_SVC_BC_EXT_GET_APP_INDEX(qmi_rsp->card_status.index_1x_pri);
  }

  /* GW info populating */
  if((info->curr.slot != 0xFF) && (info->curr.info_gw.appindex != 0xFF))
  {
    info->curr.info_gw.app_present = TRUE;
    info->curr.info_gw.deact_need = TRUE;
      QBI_LOG_D_2("info->curr.info_gw.app_present  = %d info->curr.info_gw.deact_need = %d", info->curr.info_gw.app_present,info->curr.info_gw.deact_need);
  }

  /* 1x info populating */
  if((info->curr.slot != 0xFF) && (info->curr.info_1x.appindex != 0xFF))
  {
    info->curr.info_1x.app_present = TRUE;
    info->curr.info_1x.deact_need = TRUE;
      QBI_LOG_D_2("info->curr.info_1x.app_present   = %d info->curr.info_1x.deact_need = %d", info->curr.info_1x.app_present ,info->curr.info_1x.deact_need);
  }

  /* If request is from a CID (i.e. not internal with NULL data) then extract requested slot */
  if (qmi_txn->parent->req.data != NULL)
  {
    req = (qbi_svc_bc_ext_slot_mapping_info_s *)qmi_txn->parent->req.data;
    QBI_LOG_D_0("Extracting slot from request data");
    info->req.slot = (uint32)(*((uint8*)req + req->slot_map_list.offset));
  }
  QBI_LOG_D_1("info->req.slot    = %d ", info->req.slot);

  /* Next populating info regarding activation for requested slot */
  /* GW: Requested slot info populating */
  info->req.info_gw.app_present = 
    qbi_svc_bc_ext_slot_mapping_get_aidinfo(
      qmi_rsp->card_status.card_info[info->req.slot], &info->req.info_gw, QBI_SVC_BC_EXT_APP_TYPE_GW);

  if(info->req.info_gw.app_present)
  {
    info->req.info_gw.act_need = TRUE;
    QBI_LOG_D_2("info->req.info_gw.app_present   = %d info->req.info_gw.act_need = %d", info->req.info_gw.app_present ,info->req.info_gw.act_need);
  }

  /* 1x: Requested slot info populating */
  info->req.info_1x.app_present = 
    qbi_svc_bc_ext_slot_mapping_get_aidinfo(
      qmi_rsp->card_status.card_info[info->req.slot], &info->req.info_1x, QBI_SVC_BC_EXT_APP_TYPE_1X);

  if(info->req.info_1x.app_present)
  {
    info->req.info_1x.act_need = TRUE;
    QBI_LOG_D_2("info->req.info_1x.app_present   = %d info->req.info_1x.act_need  = %d", info->req.info_1x.app_present ,info->req.info_1x.act_need );
  }
} /* qbi_svc_bc_ext_populate_app_slot_info() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_slot_mapping_s_deact_uim2e_cb
===========================================================================*/
/*!
    @brief Handles QMI_UIM_EVENT_REG_RESP

    @details (De)registering the card status event during deactivation stage

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_slot_mapping_s_deact_uim2e_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  uim_event_reg_resp_msg_v01                  *qmi_rsp;
  qbi_svc_action_e                             action;
  qbi_svc_bc_ext_slot_map_cmd_info_s          *info;
  uim_change_provisioning_session_req_msg_v01 *qmi_req = NULL;
  uim_get_card_status_req_msg_v01             *qmi_card_req;
  qbi_svc_bc_subscriber_ready_status_rsp_s    *rsp;
  qbi_txn_s                                   *txn;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);

  QBI_LOG_D_0("Slot mapping - deact stage uim event reg resp");
  qmi_rsp = (uim_event_reg_resp_msg_v01 *)qmi_txn->rsp.data;

  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    if (qmi_rsp->resp.error != QMI_ERR_NO_EFFECT_V01)
    {
       QBI_LOG_E_1("Error registering for UIM indications!!! Error code %d",
         qmi_rsp->resp.error);

       action = QBI_SVC_ACTION_ABORT;
    }
    else
    {
      QBI_LOG_D_0("UIM registration action already complete");
    }
  }

  info = (qbi_svc_bc_ext_slot_map_cmd_info_s *)qmi_txn->parent->info;

  if ((info->curr.info_gw.deact_need == TRUE) && (info->curr.info_gw.deact_done == FALSE))
  {
    QBI_LOG_D_2("Deact req: gw deact needed %d gw deact done %d",
                info->curr.info_gw.deact_need,info->curr.info_gw.deact_done);

    info->curr.info_gw.deact_done = TRUE;
    qmi_req = (uim_change_provisioning_session_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(qmi_txn->parent,
      QBI_QMI_SVC_UIM, QMI_UIM_CHANGE_PROVISIONING_SESSION_REQ_V01,
      qbi_svc_bc_ext_change_prov_session_deact_s_uim38_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

    qmi_req->session_change.session_type = UIM_SESSION_TYPE_PRIMARY_GW_V01;
    qmi_req->session_change.activate = QBI_SVC_BC_EXT_SLOT_DEACTIVATE;
    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }
  else if((info->curr.info_1x.deact_need == TRUE) && (info->curr.info_1x.deact_done == FALSE))
  {
    QBI_LOG_D_2("Deact req: 1x deact needed %d 1x deact done %d",
                info->curr.info_1x.deact_need,info->curr.info_1x.deact_done);

    info->curr.info_1x.deact_done = TRUE;
    /* 1x pri: Requested slot different from existing mapping */
    qmi_req = (uim_change_provisioning_session_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(qmi_txn->parent,
      QBI_QMI_SVC_UIM, QMI_UIM_CHANGE_PROVISIONING_SESSION_REQ_V01,
      qbi_svc_bc_ext_change_prov_session_deact_s_uim38_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

    qmi_req->session_change.session_type = UIM_SESSION_TYPE_PRIMARY_1X_V01;
    qmi_req->session_change.activate = QBI_SVC_BC_EXT_SLOT_DEACTIVATE;
    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }
  else
  {
    info->curr.info_gw.deact_done = TRUE;
    info->curr.info_1x.deact_done = TRUE;

    /* Deactivation stage is now complete */
    QBI_LOG_D_0("Deact complete.");
    txn = qbi_txn_alloc_event(qmi_txn->parent->ctx, 
            QBI_SVC_ID_BC, QBI_SVC_BC_MBIM_CID_SUBSCRIBER_READY_STATUS);

    /* Allocate the fixed-length portion of the response now */
    rsp = qbi_txn_alloc_rsp_buf(txn, sizeof(qbi_svc_bc_subscriber_ready_status_rsp_s));
    QBI_CHECK_NULL_PTR_RET_ABORT(rsp);

    rsp->ready_state = QBI_SVC_BC_READY_STATE_NOT_INITIALIZED;
    QBI_LOG_D_1("rsp->ready_state = %d", rsp->ready_state);
    (void) qbi_svc_proc_action(txn, QBI_SVC_ACTION_SEND_RSP);

    qmi_card_req = (uim_get_card_status_req_msg_v01 *)
      qbi_qmi_txn_alloc_ret_req_buf(
        qmi_txn->parent, QBI_QMI_SVC_UIM, QMI_UIM_GET_CARD_STATUS_REQ_V01,
        qbi_svc_bc_ext_slot_mapping_s_uim2f_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_card_req);

    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }

  return action;
} /* qbi_svc_bc_ext_slot_mapping_s_deact_uim2e_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_slot_mapping_s_deact_req
===========================================================================*/
/*!
    @brief Handles QMI_UIM_CHANGE_PROVISIONING_SESSION_REQ for
            MBIM_CID_MS_DEVICE_SLOT_MAPPING set

    @details This function triggers the deactivation of subscriptions on
             current slot

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_slot_mapping_s_deact_req
(
  qbi_qmi_txn_s *qmi_txn
)
{
  uim_event_reg_req_msg_v01           *qmi_req = NULL;
  qbi_svc_bc_ext_slot_map_cmd_info_s  *info = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);

  info = (qbi_svc_bc_ext_slot_map_cmd_info_s *)qmi_txn->parent->info;

  /* De-register for QMI_UIM_STATUS_CHANGE_IND */
  qmi_req = (uim_event_reg_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(
      qmi_txn->parent, QBI_QMI_SVC_UIM, QMI_UIM_EVENT_REG_REQ_V01,
      qbi_svc_bc_ext_slot_mapping_s_deact_uim2e_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  if (info->curr.info_gw.deact_need || info->curr.info_1x.deact_need)
  {
    qmi_req->event_mask = (0 << QMI_UIM_EVENT_CARD_STATUS_BIT_V01);
  }
  else
  {
    qmi_req->event_mask = (1 << QMI_UIM_EVENT_CARD_STATUS_BIT_V01);
  }

  return QBI_SVC_ACTION_SEND_QMI_REQ;
} /* qbi_svc_bc_ext_slot_mapping_s_deact_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_change_prov_session_deact_s_uim38_rsp_cb
===========================================================================*/
/*!
    @brief Handles a QMI_UIM_CHANGE_PROVISIONONGS_SESSION_RESP for
            MBIM_CID_MS_DEVICE_SLOT_MAPPING query

    @details This function handles response for deactivation of subscription on
             current slot

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_change_prov_session_deact_s_uim38_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  uim_change_provisioning_session_req_msg_v01 *qmi_change_prov_req = NULL;
  qbi_svc_bc_ext_slot_map_cmd_info_s  *info;
  uim_event_reg_req_msg_v01* qmi_req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);

  info = (qbi_svc_bc_ext_slot_map_cmd_info_s *)qmi_txn->parent->info;

  if ((info->curr.info_1x.deact_need == TRUE) && (info->curr.info_1x.deact_done == FALSE))
  {
    QBI_LOG_D_2("Deact req: 1x deact needed %d 1x deact done %d",
                info->curr.info_1x.deact_need,info->curr.info_1x.deact_done);
    info->curr.info_1x.deact_done = TRUE;
    qmi_change_prov_req = (uim_change_provisioning_session_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(qmi_txn->parent, QBI_QMI_SVC_UIM,
      QMI_UIM_CHANGE_PROVISIONING_SESSION_REQ_V01,
      qbi_svc_bc_ext_change_prov_session_deact_s_uim38_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_change_prov_req);

    qmi_change_prov_req->session_change.session_type = UIM_SESSION_TYPE_PRIMARY_1X_V01;
    qmi_change_prov_req->session_change.activate = QBI_SVC_BC_EXT_SLOT_DEACTIVATE;
  }
  else
  {
    QBI_LOG_D_2("Sub deact complete. Re-register card status events",
                info->curr.info_1x.deact_need,info->curr.info_1x.deact_done);
    /* Re-register for QMI_UIM_STATUS_CHANGE_IND */
    qmi_req = (uim_event_reg_req_msg_v01 *)
      qbi_qmi_txn_alloc_ret_req_buf(
        qmi_txn->parent, QBI_QMI_SVC_UIM, QMI_UIM_EVENT_REG_REQ_V01,
        qbi_svc_bc_ext_slot_mapping_s_deact_uim2e_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

    qmi_req->event_mask = (1 << QMI_UIM_EVENT_CARD_STATUS_BIT_V01);
  }
  return QBI_SVC_ACTION_SEND_QMI_REQ;
} /* qbi_svc_bc_ext_change_prov_session_deact_s_uim38_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_slot_mapping_s_act_req
===========================================================================*/
/*!
    @brief Handles a QMI_UIM_GET_CARD_STATUS_RESP for
            MBIM_CID_MS_DEVICE_SLOT_MAPPING query

    @details This function handles deactivation of requested 1x_pri slot 

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_slot_mapping_s_act_req
(
  qbi_qmi_txn_s *qmi_txn
)
{
    qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
    uim_change_provisioning_session_req_msg_v01 *qmi_req = NULL;
    qbi_svc_bc_ext_slot_map_cmd_info_s  *info;
/*-------------------------------------------------------------------------*/
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);

    info = (qbi_svc_bc_ext_slot_map_cmd_info_s *)qmi_txn->parent->info;

    /* GW: Activating slot */
    if ((info->req.info_gw.act_need == TRUE) && (info->req.info_gw.act_done == FALSE))
    {
      info->req.info_gw.act_done = TRUE;
      qmi_req = (uim_change_provisioning_session_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(qmi_txn->parent,
        QBI_QMI_SVC_UIM, QMI_UIM_CHANGE_PROVISIONING_SESSION_REQ_V01,
        qbi_svc_bc_ext_change_prov_session_act_s_uim38_rsp_cb);
      QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

      qmi_req->session_change.session_type = UIM_SESSION_TYPE_PRIMARY_GW_V01;
      qmi_req->session_change.activate = QBI_SVC_BC_EXT_SLOT_ACTIVATE;
      qmi_req->application_information_valid = TRUE;

      if(info->req.slot == QBI_SVC_BC_EXT_SLOT_1)
        qmi_req->application_information.slot = UIM_SLOT_1_V01;
      else if(info->req.slot == QBI_SVC_BC_EXT_SLOT_2)
        qmi_req->application_information.slot = UIM_SLOT_2_V01;

      qmi_req->application_information.aid_len = info->req.info_gw.aid_length;
      QBI_MEMSCPY(qmi_req->application_information.aid, 
        sizeof(qmi_req->application_information.aid), 
        info->req.info_gw.aid, info->req.info_gw.aid_length);
      action = QBI_SVC_ACTION_SEND_QMI_REQ;
    }
    else if((info->req.info_1x.act_need == TRUE) && (info->req.info_1x.act_done == FALSE))
    {
      /* 1x : Activating requested slot */
      info->req.info_1x.act_done = TRUE;
      qmi_req = (uim_change_provisioning_session_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(qmi_txn->parent, QBI_QMI_SVC_UIM,
        QMI_UIM_CHANGE_PROVISIONING_SESSION_REQ_V01,
        qbi_svc_bc_ext_change_prov_session_act_s_uim38_rsp_cb);
      QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

      qmi_req->session_change.session_type = UIM_SESSION_TYPE_PRIMARY_1X_V01;
      qmi_req->session_change.activate = QBI_SVC_BC_EXT_SLOT_ACTIVATE;
      qmi_req->application_information_valid = TRUE;

      if(info->req.slot == QBI_SVC_BC_EXT_SLOT_1)
        qmi_req->application_information.slot = UIM_SLOT_1_V01;
      else if(info->req.slot == QBI_SVC_BC_EXT_SLOT_2)
        qmi_req->application_information.slot = UIM_SLOT_2_V01;

      qmi_req->application_information.aid_len = info->req.info_1x.aid_length;
      QBI_MEMSCPY(qmi_req->application_information.aid, 
        sizeof(qmi_req->application_information.aid), 
        info->req.info_1x.aid, info->req.info_1x.aid_length);
      action = QBI_SVC_ACTION_SEND_QMI_REQ;
    }

    return action;
} /* qbi_svc_bc_ext_slot_mapping_s_act_req() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_change_prov_session_act_s_uim38_rsp_cb
===========================================================================*/
/*!
    @brief Dispatches QMI_UIM_CHANGE_PROVISIONING_SESSION_REQ for
            MBIM_CID_MS_DEVICE_SLOT_MAPPING set

    @details This function triggers activation of subscription on the
             requested slot

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_change_prov_session_act_s_uim38_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
    qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
    uim_change_provisioning_session_req_msg_v01 *qmi_change_prov_req = NULL;
    uim_get_card_status_req_msg_v01 *qmi_card_req = NULL;
    qbi_svc_bc_ext_slot_map_cmd_info_s  *info;
    /*-------------------------------------------------------------------------*/
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);

    info = (qbi_svc_bc_ext_slot_map_cmd_info_s *)qmi_txn->parent->info;

    QBI_LOG_D_2("Activate rsp cb:1x act needed %d 1x act done %d",
                info->req.info_1x.act_need, info->req.info_1x.act_done);

    if((info->req.info_1x.act_need == TRUE) && (info->req.info_1x.act_done == FALSE))
    {
      info->req.info_1x.act_done = TRUE;
      qmi_change_prov_req = (uim_change_provisioning_session_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(qmi_txn->parent, 
        QBI_QMI_SVC_UIM, QMI_UIM_CHANGE_PROVISIONING_SESSION_REQ_V01,
        qbi_svc_bc_ext_change_prov_session_act_s_uim38_rsp_cb);
      QBI_CHECK_NULL_PTR_RET_ABORT(qmi_change_prov_req);

      qmi_change_prov_req->session_change.session_type = UIM_SESSION_TYPE_PRIMARY_1X_V01;
      qmi_change_prov_req->session_change.activate = QBI_SVC_BC_EXT_SLOT_ACTIVATE;
      qmi_change_prov_req->application_information_valid = TRUE;

      if(info->req.slot == QBI_SVC_BC_EXT_SLOT_1)
        qmi_change_prov_req->application_information.slot = UIM_SLOT_1_V01;
      else if(info->req.slot == QBI_SVC_BC_EXT_SLOT_2)
        qmi_change_prov_req->application_information.slot = UIM_SLOT_2_V01;

      qmi_change_prov_req->application_information.aid_len = info->req.info_1x.aid_length;
      QBI_MEMSCPY(qmi_change_prov_req->application_information.aid, 
        sizeof(qmi_change_prov_req->application_information.aid), 
        info->req.info_1x.aid, info->req.info_1x.aid_length);
      action = QBI_SVC_ACTION_SEND_QMI_REQ;
    }
    else
    {
      qmi_card_req = (uim_get_card_status_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(qmi_txn->parent, QBI_QMI_SVC_UIM, 
        QMI_UIM_GET_CARD_STATUS_REQ_V01, qbi_svc_bc_ext_slot_mapping_s_uim2f_rsp_cb);
      QBI_CHECK_NULL_PTR_RET_ABORT(qmi_card_req);
      action = QBI_SVC_ACTION_SEND_QMI_REQ;
    }

    return action;
} /* qbi_svc_bc_ext_change_prov_session_act_s_uim38_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_slot_mapping_get_aid_aidlength
===========================================================================*/
/*!
    @brief Handles a QMI_UIM_GET_CARD_STATUS_RESP for
            MBIM_CID_MS_DEVICE_SLOT_MAPPING query

    @details This function finds aid and aid length for requested slot

    @param card_info, info, app_type

    @return boolean
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_slot_mapping_get_aidinfo
(
  card_info_type_v01            card_info,
  qbi_svc_bc_ext_app_info_s     *info,
  qbi_svc_bc_ext_app_type_enum  app_type
)
{
  uint32 slot_idx = 0;
  boolean success = FALSE;
  uim_app_type_enum_v01 app_type_2g;
  uim_app_type_enum_v01 app_type_3g;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(info);

  switch(app_type)
  {
  case QBI_SVC_BC_EXT_APP_TYPE_GW:
    app_type_2g = UIM_APP_TYPE_SIM_V01;
    app_type_3g = UIM_APP_TYPE_USIM_V01;
    break;
  case QBI_SVC_BC_EXT_APP_TYPE_1X:
    app_type_2g = UIM_APP_TYPE_RUIM_V01;
    app_type_3g = UIM_APP_TYPE_CSIM_V01;
    break;
  default:
    QBI_LOG_E_1("Invalid app_type = %d ", app_type);
    return FALSE;
  }
  
  /* Getting aid info. */
  for (slot_idx = 0; slot_idx< card_info.app_info_len; slot_idx++)
  {
    if (card_info.app_info[slot_idx].app_type == app_type_2g ||
        card_info.app_info[slot_idx].app_type == app_type_3g)
    {
      info->aid_length = card_info.app_info[slot_idx].aid_value_len;
      QBI_MEMSCPY(info->aid, sizeof(info->aid), card_info.app_info[slot_idx].aid_value,
          card_info.app_info[slot_idx].aid_value_len);
      success = TRUE;
      QBI_LOG_D_1("info->aid_length  = %d ", info->aid_length);
      break;
    }

    success = FALSE;
  }
  
  return success;
} /* qbi_svc_bc_ext_slot_mapping_get_aidinfo() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_slot_mapping_close_logical_channels
===========================================================================*/
/*!
    @brief qbi_svc_bc_ext_slot_mapping_close_logical_channels handle closing
    of logical channels

    @details when slot mapping occurs all channels related to current slot
    are removed from the cache 

    @param qmi_txn

    @return TRUE or FALSE
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_slot_mapping_close_logical_channels
(
  qbi_qmi_txn_s *qmi_txn
)
{
  qbi_svc_bc_ext_exec_slot_config_s exec_slot_cfg;
  qbi_svc_msuicc_cache_s *cache;
  boolean found_open_channel = FALSE;
  qbi_svc_msuicc_logical_channel_s *logical_channel;
  qbi_util_list_iter_s iter;
  uim_logical_channel_req_msg_v01 *qmi_req;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);

  cache = qbi_svc_msuicc_cache_get(qmi_txn->ctx);

  if (cache == NULL)
    return FALSE;

  if (!qbi_nv_store_cfg_item_read(qmi_txn->parent->ctx,
    QBI_NV_STORE_CFG_ITEM_EXECUTOR_SLOT_CONFIG,&exec_slot_cfg,
    sizeof(qbi_svc_bc_ext_exec_slot_config_s)))
  {
    QBI_LOG_E_0("Missing executor slot mapping info");
  }
  else
  {
    qbi_util_list_iter_init(&cache->logical_channel_list, &iter);
    while ((logical_channel = (qbi_svc_msuicc_logical_channel_s *)
             qbi_util_list_iter_next(&iter)) != NULL)
    {
      qmi_req = (uim_logical_channel_req_msg_v01 *)
                qbi_qmi_txn_alloc_ret_req_buf(
                  qmi_txn->parent, QBI_QMI_SVC_UIM, QMI_UIM_LOGICAL_CHANNEL_REQ_V01,
                qbi_svc_bc_ext_slot_mapping_close_channel_s_uim3f_rsp_cb);
              QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

      qmi_req->slot = UIM_SLOT_1_V01 + exec_slot_cfg.exec0_slot;
      qmi_req->channel_id_valid = TRUE;
      qmi_req->channel_id = logical_channel->channel_id;
      found_open_channel = TRUE;
      break;
    }
  }

  return found_open_channel;
}/* qbi_svc_bc_ext_slot_mapping_close_logical_channels */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_slot_mapping_close_channel_s_uim3f_rsp_cb
===========================================================================*/
/*!
    @brief qbi_svc_bc_ext_slot_mapping_close_channel_s_uim3f_rsp_cb
    handle closing of logical channels

    @details when slot mapping occurs all channels related to current slot
    are removed from the cache 

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_slot_mapping_close_channel_s_uim3f_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  uim_get_card_status_req_msg_v01 *qmi_req = NULL;
  const uim_logical_channel_req_msg_v01 *qmi_req_logical_channel;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->req.data);

  qmi_req_logical_channel = (uim_logical_channel_req_msg_v01 *) qmi_txn->req.data;
  /*! Remove closed logical channel from cached active channel list
      after receiving normal processing status. */
  if (!qbi_svc_msuicc_logical_channel_remove_from_cache(
          qmi_txn->ctx, qmi_req_logical_channel->channel_id))
  {
    QBI_LOG_E_1("Cannot remove channel %d from cache",
                  qmi_req_logical_channel->channel_id);
  }

  if (!qbi_svc_bc_ext_slot_mapping_close_logical_channels(qmi_txn))
  {
    QBI_LOG_D_0("All channel close complete so performing get card status");
    qmi_req = (uim_get_card_status_req_msg_v01 *)
        qbi_qmi_txn_alloc_ret_req_buf(qmi_txn->parent,
                                      QBI_QMI_SVC_UIM, QMI_UIM_GET_CARD_STATUS_REQ_V01,
                                      qbi_svc_bc_ext_slot_mapping_s_uim2f_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);
  }

  return QBI_SVC_ACTION_SEND_QMI_REQ;

}/* qbi_svc_bc_ext_slot_mapping_close_channel_s_uim3f_rsp_cb */

/*! @} */

/*! @addtogroup MBIM_CID_MS_SLOT_INFO_STATUS
    @{ */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_slot_info_status_q_req
===========================================================================*/
/*!
    @brief Handles a MBIM_CID_MS_SLOT_INFO_STATUS query request

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_slot_info_status_q_req
(
    qbi_txn_s *txn
)
{
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  qbi_svc_bc_ext_slot_info_req_s *req = NULL;
  uim_get_atr_req_msg_v01 *qmi_req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(txn->req.data);

  req = (qbi_svc_bc_ext_slot_info_req_s *)txn->req.data;

  if (req->slot_index > 1)
  {
    QBI_LOG_E_0("Invalid Slot Index");
        txn->status = QBI_MBIM_STATUS_FAILURE;
  }
  else
  {
    qmi_req = (uim_get_atr_req_msg_v01 *)qbi_qmi_txn_alloc_ret_req_buf(
        txn, QBI_QMI_SVC_UIM, QMI_UIM_GET_ATR_REQ_V01,
        qbi_svc_bc_ext_slot_info_status_q_atr_rsp_cb);
    QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);
    qmi_req->slot = req->slot_index + UIM_SLOT_1_V01;

    /* Allocating memory for info. */
    txn->info = QBI_MEM_MALLOC_CLEAR(sizeof(qbi_svc_bc_ext_slot_info));
    QBI_CHECK_NULL_PTR_RET_ABORT(txn->info);

    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }

  return action;
}/* qbi_svc_bc_ext_slot_info_status_q_req */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_slot_info_q_prepare_rsp
===========================================================================*/
/*!
    @brief Handles a MBIM_CID_MS_SLOT_INFO_STATUS response

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static void qbi_svc_bc_ext_slot_info_q_prepare_rsp
(
  qbi_txn_s           *txn,
  uint32              slot_idx,
  card_info_type_v01  card_info
)
{
  qbi_svc_bc_ext_slot_info_rsp_s *rsp = NULL;
  qbi_svc_bc_ext_slot_info *info = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET(txn);

  info = (qbi_svc_bc_ext_slot_info *)txn->info;
  rsp = qbi_txn_alloc_rsp_buf(txn, sizeof(qbi_svc_bc_ext_slot_info_rsp_s));
  QBI_CHECK_NULL_PTR_RET(rsp);

  rsp->slot_index = slot_idx;

  switch (card_info.card_state)
  {
  case UIM_CARD_STATE_ABSENT_V01:
    rsp->state = QBI_SVC_MBIM_MS_UICC_SLOT_STATE_EMPTY;
    break;

  case UIM_CARD_STATE_ERROR_V01:
    rsp->state = card_info.error_code == UIM_CARD_ERROR_CODE_POWER_DOWN_V01 ?
      QBI_SVC_MBIM_MS_UICC_SLOT_STATE_OFF : QBI_SVC_MBIM_MS_UICC_SLOT_STATE_OFF_EMPTY;
    break;

  default:
      if ((info != NULL)&& (info->is_esim == TRUE))
    {
      rsp->state = QBI_SVC_MBIM_MS_UICC_SLOT_STATE_ACTIVE_ESIM_NOPROFILE;
      if (card_info.app_info_len > 0)
      {
        rsp->state = QBI_SVC_MBIM_MS_UICC_SLOT_STATE_ACTIVE_ESIM;
      }
      QBI_LOG_D_1("eUICC functions supported and esim state = %d",
        rsp->state);
    }
    else
    {
      rsp->state = QBI_SVC_MBIM_MS_UICC_SLOT_STATE_ACTIVE;
      QBI_LOG_D_1("eUICC functions not supported and esim state = %d",
        rsp->state);
    }
    break;
  }

  QBI_LOG_D_2("Slot info: Card state %d at slot index %d", 
    rsp->state, rsp->slot_index);
}/* qbi_svc_bc_ext_slot_info_q_prepare_rsp() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_slot_info_status_q_atr_rsp_cb
===========================================================================*/
/*!
    @brief Handles a MBIM_CID_MS_SLOT_INFO_STATUS response

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_slot_info_status_q_atr_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  uim_get_card_status_req_msg_v01 *qmi_req = NULL;
  uim_get_atr_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_slot_info *info = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->info);

  qmi_rsp = (uim_get_atr_resp_msg_v01 *) qmi_txn->rsp.data;
  info = (qbi_svc_bc_ext_slot_info *)qmi_txn->parent->info;

  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Error: Received error code %d from QMI", 
      qmi_rsp->resp.error);
  }

  if ((qmi_rsp->euicc_supported_valid) &&
      (qmi_rsp->euicc_supported == TRUE))
  {
    info->is_esim = TRUE;
  }
  else
  {
    info->is_esim = FALSE;
  }

  QBI_LOG_D_2("euicc supported valid = %d euicc supported = %d",
    qmi_rsp->euicc_supported_valid,qmi_rsp->euicc_supported);

  /* Get Card Status */
  qmi_req = (uim_get_card_status_req_msg_v01 *)
    qbi_qmi_txn_alloc(qmi_txn->parent, QBI_QMI_SVC_UIM,
      QMI_UIM_GET_CARD_STATUS_REQ_V01,
      qbi_svc_bc_ext_slot_info_q_uim2f_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  return QBI_SVC_ACTION_SEND_QMI_REQ;
}/* qbi_svc_bc_ext_slot_info_status_q_atr_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_slot_info_q_uim2f_rsp_cb
===========================================================================*/
/*!
    @brief Handles a MBIM_CID_MS_SLOT_INFO_STATUS response

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_slot_info_q_uim2f_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  uim_get_card_status_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_bc_ext_slot_info_req_s *req = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent->req.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  req = (qbi_svc_bc_ext_slot_info_req_s *)qmi_txn->parent->req.data;
  qmi_rsp = (uim_get_card_status_resp_msg_v01 *)qmi_txn->rsp.data;

  QBI_LOG_D_1("Slot info: Received request for slot index %d",
    req->slot_index);

  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01)
  {
    QBI_LOG_E_1("Error: Received error code %d from QMI",
      qmi_rsp->resp.error);

    qmi_txn->parent->status = QBI_MBIM_STATUS_FAILURE;
  }
  else
  {
    if (qmi_rsp->card_status_valid)
    {
      qbi_svc_bc_ext_slot_info_q_prepare_rsp(qmi_txn->parent,
        req->slot_index, qmi_rsp->card_status.card_info[req->slot_index]);
    }
    else
    {
      QBI_LOG_E_0("Error: Card status invalid");
      qmi_txn->parent->status = QBI_MBIM_STATUS_FAILURE;
    }
  }

  return QBI_SVC_ACTION_SEND_RSP;
}/* qbi_svc_bc_ext_slot_info_q_uim2f_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_slot_info_uim32_ind_cb
===========================================================================*/
/*!
    @brief Handles QMI_UIM_STATUS_CHANGE_IND

    @details

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_slot_info_uim32_ind_cb
(
  const qbi_svc_qmi_ind_data_s *ind
)
{
  const uim_status_change_ind_msg_v01 *qmi_ind = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  uim_get_atr_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_info_status_cache *cache;
  uint32 new_slot_state;
  qbi_svc_bc_ext_slot_info_rsp_s *rsp = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(ind);

  qmi_ind = (const uim_status_change_ind_msg_v01 *)ind->buf->data;

  cache = qbi_svc_bc_ext_slot_info_cache_get(ind->txn->ctx);
  QBI_CHECK_NULL_PTR_RET_FALSE(cache);

  if (qmi_ind->card_status_valid)
  {
    /* When indication comes , updating all cache entries for card 0 */
    cache->card0.card_state = qmi_ind->card_status.card_info[0].card_state;
    cache->card0.app_info_len = qmi_ind->card_status.card_info[0].app_info_len;
    cache->card0.card_error = qmi_ind->card_status.card_info[0].error_code;

    if (qmi_ind->card_status.card_info_len == 2)
    {
      /* When indication comes , updating all cache entries for card 1 */
      cache->card1.card_state = qmi_ind->card_status.card_info[1].card_state;
      cache->card1.app_info_len = qmi_ind->card_status.card_info[1].app_info_len;
      cache->card1.card_error = qmi_ind->card_status.card_info[1].error_code;
      qbi_svc_bc_ext_slot_info_force_card1_event(ind->txn->ctx);
    }

    if (cache->card0.card_state == UIM_CARD_STATE_PRESENT_V01)
    {
      qmi_req = (uim_get_atr_req_msg_v01 *)qbi_qmi_txn_alloc_ret_req_buf(
                   ind->txn, QBI_QMI_SVC_UIM, QMI_UIM_GET_ATR_REQ_V01,
      qbi_svc_bc_ext_slot_info_uim32_ind_q_card0_atr_rsp_cb);
      QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);
      qmi_req->slot = UIM_SLOT_1_V01;

      action = QBI_SVC_ACTION_SEND_QMI_REQ;
    }
    else
    {
       new_slot_state = qbi_svc_bc_ext_slot_info_from_uicc_slot_state(&cache->card0);
      /* updating cache and checking slot state*/

      if (qbi_svc_bc_ext_slot_status_update_cache(&cache->card0, new_slot_state))
      {
        /* clearing msuicc cache for card0 when its absent */
        qbi_svc_bc_ext_slot_info_ind_q_msuicc_cache_release(ind->txn);

        /* Forming response for card 0 */
        rsp = qbi_txn_alloc_rsp_buf(ind->txn, sizeof(qbi_svc_bc_ext_slot_info_rsp_s));
        QBI_CHECK_NULL_PTR_RET_ABORT(rsp);
        rsp->state = new_slot_state;
        rsp->slot_index = 0;
        action = QBI_SVC_ACTION_SEND_RSP;
      }
    }
  }

  return action;
} /* qbi_svc_bc_ext_slot_info_uim32_ind_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_slot_info_uim32_ind_q_card0_atr_rsp_cb
===========================================================================*/
/*!
    @brief Handles a MBIM_CID_MS_SLOT_INFO_STATUS response

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_slot_info_uim32_ind_q_card0_atr_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  uim_get_atr_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  qbi_svc_bc_ext_info_status_cache *cache;
  uint32 new_slot_state;
  qbi_svc_bc_ext_slot_info_rsp_s *rsp = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);

  qmi_rsp = (uim_get_atr_resp_msg_v01 *) qmi_txn->rsp.data;

  cache = qbi_svc_bc_ext_slot_info_cache_get(qmi_txn->parent->ctx);
  QBI_CHECK_NULL_PTR_RET_FALSE(cache);

  if ((qmi_rsp->euicc_supported_valid) &&
        (qmi_rsp->euicc_supported == TRUE))
  {
    cache->card0.is_esim = TRUE;
  }
  else
  {
    cache->card0.is_esim = FALSE;
  }

  /* Now checking slot info change state. Here response will
     be sent in case of change in slot state */
  new_slot_state = qbi_svc_bc_ext_slot_info_from_uicc_slot_state(&cache->card0);

  /* updating cache and checking slot state*/
  if (qbi_svc_bc_ext_slot_status_update_cache(&cache->card0, new_slot_state))
  {
    /* Forming response for card 0*/
    rsp = qbi_txn_alloc_rsp_buf(qmi_txn->parent, sizeof(qbi_svc_bc_ext_slot_info_rsp_s));
    QBI_CHECK_NULL_PTR_RET_ABORT(rsp);
    rsp->state = cache->card0.slot_state;
    rsp->slot_index = 0;
    action = QBI_SVC_ACTION_SEND_RSP;
  }

  return action;
}/* qbi_svc_bc_ext_slot_info_uim32_ind_q_card0_atr_rsp_cb() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_slot_info_from_uicc_slot_state
===========================================================================*/
/*!
    @brief Handles a MBIM_CID_MS_SLOT_INFO_STATUS response

    @details This function returns the current state of the card

    @param cache

    @return state
*/
/*=========================================================================*/
static uint32 qbi_svc_bc_ext_slot_info_from_uicc_slot_state
(
  qbi_svc_bc_ext_info_cache_each_slot *cache
)
{
  uint32 state;
/*-------------------------------------------------------------------------*/

  switch (cache->card_state)
  {
  case UIM_CARD_STATE_ABSENT_V01:
    state = QBI_SVC_MBIM_MS_UICC_SLOT_STATE_EMPTY;
    break;

  case UIM_CARD_STATE_ERROR_V01:
    state = cache->card_error == UIM_CARD_ERROR_CODE_POWER_DOWN_V01 ?
      QBI_SVC_MBIM_MS_UICC_SLOT_STATE_OFF : QBI_SVC_MBIM_MS_UICC_SLOT_STATE_OFF_EMPTY;
    break;

  default:
    if (cache->is_esim)
    {
      state = QBI_SVC_MBIM_MS_UICC_SLOT_STATE_ACTIVE_ESIM_NOPROFILE;
      if (cache->app_info_len > 0)
      {
        state = QBI_SVC_MBIM_MS_UICC_SLOT_STATE_ACTIVE_ESIM;
      }
    }
    else
    {
      state = QBI_SVC_MBIM_MS_UICC_SLOT_STATE_ACTIVE;
    }
    break;
  }

  return state;
}/* qbi_svc_bc_ext_slot_info_q_prepare_rsp() */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_slot_info_force_card1_event
===========================================================================*/
/*!
    @brief Forces sending an QBI_SVC_BC_EXT_MBIM_CID_MS_SLOT_INFO_STATUS
    event based on the current contents of the cache

    @details This function forces the event for card 1 if their is state
    change

    @param ctx

    @return none
*/
/*=========================================================================*/
static void qbi_svc_bc_ext_slot_info_force_card1_event
(
  qbi_ctx_s *ctx
)
{
  qbi_txn_s *txn;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  uim_get_atr_req_msg_v01 *qmi_req = NULL;
  qbi_svc_bc_ext_slot_info_rsp_s *rsp = NULL;
  qbi_svc_bc_ext_info_status_cache *cache;
  uint32 new_slot_state;
/*-------------------------------------------------------------------------*/
  txn = qbi_txn_alloc_event(ctx, QBI_SVC_ID_BC_EXT, QBI_SVC_BC_EXT_MBIM_CID_MS_SLOT_INFO_STATUS);
  QBI_CHECK_NULL_PTR_RET(txn);

  cache = qbi_svc_bc_ext_slot_info_cache_get(ctx);
  QBI_CHECK_NULL_PTR_RET(cache);

  if (cache->card1.card_state == UIM_CARD_STATE_PRESENT_V01)
  {
    qmi_req = (uim_get_atr_req_msg_v01 *)qbi_qmi_txn_alloc_ret_req_buf(
    txn, QBI_QMI_SVC_UIM, QMI_UIM_GET_ATR_REQ_V01,
    qbi_svc_bc_ext_slot_info_uim32_ind_q_card1_atr_rsp_cb);
    QBI_CHECK_NULL_PTR_RET(qmi_req);
    qmi_req->slot = UIM_SLOT_2_V01;

    action = QBI_SVC_ACTION_SEND_QMI_REQ;
  }
  else
  {

    new_slot_state = qbi_svc_bc_ext_slot_info_from_uicc_slot_state(&cache->card1);

    /* updating cache and checking slot state */
    if (qbi_svc_bc_ext_slot_status_update_cache(&cache->card1, new_slot_state))
    {
      /* clearing msuicc cache for card1 when its absent*/
      qbi_svc_bc_ext_slot_info_ind_q_msuicc_cache_release(txn);

      /* forming response for card 1*/
      rsp = qbi_txn_alloc_rsp_buf(txn, sizeof(qbi_svc_bc_ext_slot_info_rsp_s));
      QBI_CHECK_NULL_PTR_RET(rsp);
      rsp->state = new_slot_state;
      rsp->slot_index = 1;
      action = QBI_SVC_ACTION_SEND_RSP;
    }
  }

  (void) qbi_svc_proc_action(txn, action);
} /* qbi_svc_bc_ext_slot_info_force_card1_event() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_slot_info_uim32_ind_q_card1_atr_rsp_cb
===========================================================================*/
/*!
    @brief Handles a MBIM_CID_MS_SLOT_INFO_STATUS IND response

    @details This function updates the cache state if their is change in the
    card state and sends the reponse

    @param qmi_txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_slot_info_uim32_ind_q_card1_atr_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  uim_get_atr_resp_msg_v01 *qmi_rsp = NULL;
  qbi_svc_action_e action = QBI_SVC_ACTION_ABORT;
  qbi_svc_bc_ext_info_status_cache *cache;
  uint32 new_slot_state;
  qbi_svc_bc_ext_slot_info_rsp_s *rsp = NULL;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->parent);

  qmi_rsp = (uim_get_atr_resp_msg_v01 *) qmi_txn->rsp.data;

  cache = qbi_svc_bc_ext_slot_info_cache_get(qmi_txn->parent->ctx);
  QBI_CHECK_NULL_PTR_RET_FALSE(cache);

  if ((qmi_rsp->euicc_supported_valid) &&
        (qmi_rsp->euicc_supported == TRUE))
  {
    cache->card1.is_esim = TRUE;
  }
  else
  {
    cache->card1.is_esim = FALSE;
  }

  /* Now checking slot info change state. Here response will
     be sent in case of change in slot state */
  new_slot_state = qbi_svc_bc_ext_slot_info_from_uicc_slot_state(&cache->card1);

  /* updating cache and checking slot state*/
  if (qbi_svc_bc_ext_slot_status_update_cache(&cache->card1, new_slot_state))
  {
    /* Forming response for card 1*/
    rsp = qbi_txn_alloc_rsp_buf(qmi_txn->parent, sizeof(qbi_svc_bc_ext_slot_info_rsp_s));
    QBI_CHECK_NULL_PTR_RET_ABORT(rsp);
    rsp->state = cache->card1.slot_state;
    rsp->slot_index = 1;
    action = QBI_SVC_ACTION_SEND_RSP;
  }

  return action;
}/* qbi_svc_bc_ext_slot_info_uim32_ind_q_card1_atr_rsp_cb() */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_slot_status_update_cache
===========================================================================*/
/*!
    @brief Updates the cached information associated with
    MBIM_CID_SUBSCRIBER_READY_STATUS

    @details

    @param ctx
    @param ready_state New ReadyState
    @param is_perso_locked New flag indicating whether the SIM is locked
    due to a personalization key

    @return boolean TRUE if the cache changed, FALSE otherwise
*/
/*=========================================================================*/
static boolean qbi_svc_bc_ext_slot_status_update_cache
(
  qbi_svc_bc_ext_info_cache_each_slot *cache,
  uint32 slot_state
)
{
  boolean changed = FALSE;
/*-------------------------------------------------------------------------*/

  if (cache->slot_state != slot_state)
  {
    QBI_LOG_D_2("slot info changed from %d to %d ", 
      cache->slot_state, slot_state);
    cache->slot_state = slot_state;
    changed = TRUE;
  }

  return changed;
} /* qbi_svc_bc_ext_slot_status_update_cache() */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_slot_info_cache_get
===========================================================================*/
/*!
    @brief Returns a pointer to the Basic Connectivity Extension device
    service's cache

    @details

    @param ctx

    @return qbi_svc_bc_cache_s* Pointer to cache, or NULL on error
*/
/*=========================================================================*/
static qbi_svc_bc_ext_info_status_cache *qbi_svc_bc_ext_slot_info_cache_get
(
  qbi_ctx_s *ctx
)
{
  qbi_svc_bc_ext_info_status_cache *cache = NULL;
/*-------------------------------------------------------------------------*/
  cache = (qbi_svc_bc_ext_info_status_cache *)qbi_svc_cache_get(ctx, QBI_SVC_ID_MM);
  if (cache == NULL)
  {
    cache = qbi_svc_cache_alloc(ctx, QBI_SVC_ID_MM, 
      sizeof(qbi_svc_bc_ext_info_status_cache));

    if (cache == NULL)
    {
      QBI_LOG_E_0("Couldn't allocate cache for slot info status!");
      return NULL;
    }
  }

  return cache;
} /* qbi_svc_bc_ext_slot_info_cache_get() */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_slot_info_ind_q_msuicc_cache_release
===========================================================================*/
/*!
    @brief qbi_svc_bc_ext_slot_info_ind_q_msuicc_cache_release Handles
    flushing of msuicc cache

    @details This function flushes the msuicc cache if card state changes
    to absent

    @param txn

    @return none
*/
/*=========================================================================*/
static void qbi_svc_bc_ext_slot_info_ind_q_msuicc_cache_release
(
  qbi_txn_s *txn
)
{
  qbi_svc_msuicc_cache_s *msuicc_cache;
  qbi_svc_bc_ext_exec_slot_config_s exec_slot_cfg;
  qbi_svc_msuicc_logical_channel_s *logical_channel;
  qbi_util_list_iter_s iter;
  qbi_svc_bc_ext_info_status_cache *slot_info_cache;
/*-------------------------------------------------------------------------*/

  slot_info_cache = qbi_svc_bc_ext_slot_info_cache_get(txn->ctx);
  QBI_CHECK_NULL_PTR_RET(slot_info_cache);

  if (!qbi_nv_store_cfg_item_read(
      txn->ctx, QBI_NV_STORE_CFG_ITEM_EXECUTOR_SLOT_CONFIG,
      &exec_slot_cfg, sizeof(qbi_svc_bc_ext_exec_slot_config_s)))
  {
    QBI_LOG_E_0("Missing executor slot mapping info");
  }
  else if ((exec_slot_cfg.exec0_slot == QBI_SVC_BC_EXT_SLOT_1 &&
    slot_info_cache->card0.slot_state == QBI_SVC_MBIM_MS_UICC_SLOT_STATE_EMPTY)||
    (exec_slot_cfg.exec0_slot == QBI_SVC_BC_EXT_SLOT_2 &&
    slot_info_cache->card1.slot_state == QBI_SVC_MBIM_MS_UICC_SLOT_STATE_EMPTY))
  {
    msuicc_cache = qbi_svc_msuicc_cache_get(txn->ctx);
    QBI_CHECK_NULL_PTR_RET(msuicc_cache);

    qbi_util_list_iter_init(&msuicc_cache->logical_channel_list, &iter);
    while ((logical_channel = (qbi_svc_msuicc_logical_channel_s *)
             qbi_util_list_iter_next(&iter)) != NULL)
    {
      if (!qbi_svc_msuicc_logical_channel_remove_from_cache(
            txn->ctx, logical_channel->channel_id))
      {
        QBI_LOG_E_1("Cannot remove channel %d from cache",
          logical_channel->channel_id);
      }
    }
    QBI_MEMSET(msuicc_cache, 0, sizeof(qbi_svc_msuicc_cache_s));
    QBI_LOG_D_1("Flushing the msuicc cache for slot %d",
      exec_slot_cfg.exec0_slot);
  }
}/* qbi_svc_bc_ext_slot_info_ind_q_msuicc_cache_release */

/*! @} */

/*! @addtogroup QBI_SVC_BC_EXT_MBIM_CID_MS_DEVICE_RESET
    @{ */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_device_reset_s_req
===========================================================================*/
/*!
    @brief Handles a MBIM_CID_MS_DEVICE_RESET set request

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_device_reset_s_req
(
  qbi_txn_s *txn
)
{
  dms_set_operating_mode_req_msg_v01 *qmi_req;
/*-------------------------------------------------------------------------*/
  QBI_LOG_I_0("Device is Resetting");
  qmi_req = (dms_set_operating_mode_req_msg_v01 *)
    qbi_qmi_txn_alloc_ret_req_buf(
      txn, QBI_QMI_SVC_DMS, QMI_DMS_SET_OPERATING_MODE_REQ_V01,
      qbi_svc_bc_ext_device_reset_s_rsp_cb);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_req);

  qmi_req->operating_mode = DMS_OP_MODE_RESETTING_V01;

  return QBI_SVC_ACTION_SEND_QMI_REQ;
}/* qbi_svc_bc_ext_device_reset_s_req */

/*===========================================================================
FUNCTION: qbi_svc_bc_ext_device_reset_s_rsp_cb
===========================================================================*/
/*!
    @brief Handles a MBIM_CID_MS_DEVICE_RESET set request

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
static qbi_svc_action_e qbi_svc_bc_ext_device_reset_s_rsp_cb
(
  qbi_qmi_txn_s *qmi_txn
)
{
  dms_set_operating_mode_resp_msg_v01 *qmi_rsp;
/*-------------------------------------------------------------------------*/
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn);
  QBI_CHECK_NULL_PTR_RET_ABORT(qmi_txn->rsp.data);

  qmi_rsp = (dms_set_operating_mode_resp_msg_v01 *) qmi_txn->rsp.data;
  if (qmi_rsp->resp.result != QMI_RESULT_SUCCESS_V01 &&
      qmi_rsp->resp.error != QMI_ERR_NO_EFFECT_V01)
  {
    QBI_LOG_W_1("Received error code %d from QMI", qmi_rsp->resp.error);
    return QBI_SVC_ACTION_ABORT;
  }

  return QBI_SVC_ACTION_SEND_RSP;
}/* qbi_svc_bc_ext_device_reset_s_rsp_cb */

/*! @} */

/*=============================================================================

  Public Function Definitions

=============================================================================*/

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_init
===========================================================================*/
/*!
    @brief One-time initialization of the MSHOSTSHUTDOWN device service

    @details
*/
/*=========================================================================*/
void qbi_svc_bc_ext_init
(
  void
)
{
  static const qbi_svc_cfg_s qbi_svc_bc_ext_cfg = {
    {
      0x3d, 0x01, 0xdc, 0xc5, 0xfe, 0xf5, 0x4d, 0x05,
      0x0d, 0x3a, 0xbe, 0xf7, 0x05, 0x8e, 0x9a, 0xaf
    },
    QBI_SVC_ID_BC_EXT,
    FALSE,
    qbi_svc_bc_ext_cmd_hdlr_tbl,
    ARR_SIZE(qbi_svc_bc_ext_cmd_hdlr_tbl),
    qbi_svc_bc_ext_open,
    NULL
  };
/*-------------------------------------------------------------------------*/
  qbi_svc_reg(&qbi_svc_bc_ext_cfg);
} /* qbi_svc_bc_ext_init() */
