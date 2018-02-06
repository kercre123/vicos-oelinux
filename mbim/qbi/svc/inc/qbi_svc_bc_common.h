/*!
  @file
  qbi_svc_bc_common.h

  @brief
  Basic Connectivity common internal header file. Should not be included by
  files that aren't directly associated with the Basic Connectivity device
  service.
*/

/*=============================================================================

  Copyright (c) 2011-2013, 2017 Qualcomm Technologies, Inc.
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
07/12/17  rv   Updated data type for clas 1/2 status for compatibility with WD
06/28/17  rv   Add Structure for clas 1/2 status
06/07/17  mm   Moved executor slot configuration related structure here
11/18/13  bd   Combine PLMN name and SPN per 3GPP 22.101 A.4 (NV configurable)
03/13/13  hz   Add support for multiple data sessions
07/02/12  bd   Add ProviderId conversion functions to support QMI_NAS_SYS_INFO
10/18/11  bd   Added file based on MBIM v0.81c
=============================================================================*/

#ifndef QBI_SVC_BC_COMMON_H
#define QBI_SVC_BC_COMMON_H

/*=============================================================================

  Include Files

=============================================================================*/

#include "qbi_svc_bc_mbim.h"

#include "qbi_common.h"
#include "qbi_mbim.h"
#include "qbi_qmi.h"
#include "qbi_qmi_txn.h"
#include "qbi_txn.h"

#include "network_access_service_v01.h"

/*=============================================================================

  Constants and Macros

=============================================================================*/

/*! Do not change the cached value */
#define QBI_SVC_BC_CACHE_NO_CHANGE_U32 (0xFFFFFFFE)
#define QBI_SVC_BC_CACHE_NO_CHANGE_U16 (0xFFFF)

/* Maximum length of ProviderId and ProviderName when represented in ASCII */
#define QBI_SVC_BC_PROVIDER_ID_ASCII_MAX_LEN \
  ((QBI_SVC_BC_PROVIDER_ID_MAX_LEN_BYTES / 2) + 1)
#define QBI_SVC_BC_PROVIDER_NAME_ASCII_MAX_LEN \
  ((QBI_SVC_BC_PROVIDER_NAME_MAX_LEN_BYTES / 2) + 1)

/*! Minimum length of a 3GPP/3GPP2 ProviderId in characters (not including NULL
    termination) */
#define QBI_SVC_BC_PROVIDER_ID_MIN_LEN_CHARS (5)

/*! Maximum length of a 3GPP ProviderId in characters (not including NULL
    termination) */
#define QBI_SVC_BC_PROVIDER_ID_MAX_LEN_CHARS (6)

/*! Internal short representation of MBIM_CONTEXT_TYPES (nominally a UUID value)
    @see qbi_svc_bc_context_type_id_to_uuid, qbi_svc_bc_context_type_uuid_to_id */
#define QBI_SVC_BC_CONTEXT_TYPE_NONE        (0)
#define QBI_SVC_BC_CONTEXT_TYPE_INTERNET    (1)
#define QBI_SVC_BC_CONTEXT_TYPE_VPN         (2)
#define QBI_SVC_BC_CONTEXT_TYPE_VOICE       (3)
#define QBI_SVC_BC_CONTEXT_TYPE_VIDEO_SHARE (4)
#define QBI_SVC_BC_CONTEXT_TYPE_PURCHASE    (5)
#define QBI_SVC_BC_CONTEXT_TYPE_IMS         (6)
#define QBI_SVC_BC_CONTEXT_TYPE_MMS         (7)
#define QBI_SVC_BC_CONTEXT_TYPE_LOCAL       (8)

/*! This context type UUID is not defined in MBIM, but is used in Windows via
    their WWAN_CONTEXT_TYPE enum value WwanContextTypeCustom, exposed at the
    OID level */
#define QBI_SVC_BC_CONTEXT_TYPE_CUSTOM      (9)
#define QBI_SVC_BC_CONTEXT_TYPE_ADMIN       (10)
#define QBI_SVC_BC_CONTEXT_TYPE_APP         (11)
#define QBI_SVC_BC_CONTEXT_TYPE_XCAP        (12)
#define QBI_SVC_BC_CONTEXT_TYPE_TETHERING   (13)
#define QBI_SVC_BC_CONTEXT_TYPE_CALLING     (14)

#define QBI_SVC_BC_CONTEXT_TYPE_MAX         (15)

/* Name of the CustomDataClass */
#define QBI_SVC_BC_CUSTOM_DATA_CLASS_NAME_ASCII "TD-SCDMA"

/*=============================================================================

  Typedefs

=============================================================================*/

/*! Struct containing pointers to areas where basic connectivity modules can
    maintain their own cache. */
typedef struct {
  struct qbi_svc_bc_nas_cache_struct *nas;
  struct qbi_svc_bc_sim_cache_struct *sim;
} qbi_svc_bc_module_cache_s;

/*! Network name source preference */
typedef enum {
  /*! Select SPN or PLMN name depending on whether the registered network is a
      home network, and the display byte in EF-SPN (if available) */
  QBI_SVC_BC_PROVIDER_NAME_PREF_REGISTERED,

  /*! Same rules as REGISTERED, plus optionally concatenate SPN and PLMN name
      per 3GPP TS 22.101 A.4 */
  QBI_SVC_BC_PROVIDER_NAME_PREF_REGISTERED_WITH_CONCAT,

  /*! Prefer SPN over PLMN name */
  QBI_SVC_BC_PROVIDER_NAME_PREF_SPN,

  /*! Prefer PLMN name over SPN */
  QBI_SVC_BC_PROVIDER_NAME_PREF_PLMN_NAME
} qbi_svc_bc_provider_name_pref_e;

/*! Executor slot configuration related structure */
typedef PACK(struct) {
  uint32  exec0_slot;
  boolean exec0_prov_complete;
} qbi_svc_bc_ext_exec_slot_config_s;

/*! Operator configuration related structure */
typedef PACK(struct) {
  uint32 class1_disable;
  uint32 class2_disable;
} qbi_svc_bc_ext_operator_config_s;

/*! States Used by MBIM_CID_MS_PROVISIONED_CONTEXT_V2 CID 
    Query/Set Request to track operator specific 
    profile settings */
typedef enum {
  /*! This is the default state after OS Reset and 
      Cold Boot */
  QBI_SVC_BC_EXT_OPERATOR_STATE_NONE     = 0,

  /*! This state means NV is SET    (Profile Disabled) */
  QBI_SVC_BC_EXT_OPERATOR_STATE_SET      = 1,

  /*! This state means NV is not SET (Profile Enabled) */
  QBI_SVC_BC_EXT_OPERATOR_STATE_UNSET    = 2
} qbi_svc_bc_ext_operator_state_e;
/*=============================================================================

  Function Prototypes

=============================================================================*/

/*===========================================================================
  FUNCTION: qbi_svc_bc_provider_append_3gpp_provider_id
===========================================================================*/
/*!
    @brief Convert a binary MCC+MNC into MBIM ProviderId representation
    (UTF-16, 5 or 6 decimal characters), and append it to the response

    @details

    @param txn
    @param provider_id
    @param initial_offset
    @param mcc
    @param mnc
    @param mnc_is_3_digits
*/
/*=========================================================================*/
boolean qbi_svc_bc_provider_append_3gpp_provider_id
(
  qbi_txn_s                   *txn,
  qbi_mbim_offset_size_pair_s *provider_id,
  uint32                       initial_offset,
  uint16                       mcc,
  uint16                       mnc,
  boolean                      mnc_is_3_digits
);

/*===========================================================================
  FUNCTION: qbi_svc_bc_provider_append_3gpp2_provider_id
===========================================================================*/
/*!
    @brief Convert a binary SID into MBIM ProviderId representation
    (UTF-16, 5 decimal characters), and append it to the response

    @details

    @param txn
    @param provider_id
    @param initial_offset
    @param sid
*/
/*=========================================================================*/
boolean qbi_svc_bc_provider_append_3gpp2_provider_id
(
  qbi_txn_s                   *txn,
  qbi_mbim_offset_size_pair_s *provider_id,
  uint32                       initial_offset,
  uint16                       sid
);

/*! @addtogroup MBIM_CID_CONNECT
    @{ */

/*===========================================================================
  FUNCTION: qbi_svc_bc_connect_deactivate_all_sessions
===========================================================================*/
/*!
    @brief Issues an internal request to disconnect all active sessions

    @details

    @param ctx
*/
/*=========================================================================*/
void qbi_svc_bc_connect_deactivate_all_sessions
(
  qbi_ctx_s *ctx
);

/*===========================================================================
  FUNCTION: qbi_svc_bc_connect_get_first_connected_svc_id
===========================================================================*/
/*!
    @brief Gets the first connected WDS service ID

    @details
    Scan all sessions to find first WDS client with IPv4 connected. If not
    found, scan again for first WDS client with IPv6 connected.

    @param ctx

    @return qbi_qmi_svc_e
*/
/*=========================================================================*/
qbi_qmi_svc_e qbi_svc_bc_connect_get_first_connected_svc_id
(
  qbi_ctx_s *ctx
);

/*===========================================================================
  FUNCTION: qbi_svc_bc_connect_is_connected
===========================================================================*/
/*!
    @brief Checks whether the device has an active connection up (includes
    deactivating state)

    @details

    @param ctx

    @return boolean TRUE if connected, FALSE otherwise
*/
/*=========================================================================*/
boolean qbi_svc_bc_connect_is_connected
(
  const qbi_ctx_s *ctx
);

/*===========================================================================
  FUNCTION: qbi_svc_bc_connect_is_loopback
===========================================================================*/
/*!
    @brief Checks the loopback state of current connection from cache

    @details

    @param ctx

    @return boolean TRUE if in loopback mode, FALSE otherwise
*/
/*=========================================================================*/
boolean qbi_svc_bc_connect_is_loopback
(
  const qbi_ctx_s *ctx
);

/*===========================================================================
  FUNCTION: qbi_svc_bc_connect_s_req
===========================================================================*/
/*!
    @brief Handles a MBIM_CID_CONNECT set request

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
qbi_svc_action_e qbi_svc_bc_connect_s_req
(
  qbi_txn_s *txn
);

/*! @} */

/*===========================================================================
  FUNCTION: qbi_svc_bc_context_type_id_to_uuid
===========================================================================*/
/*!
    @brief Maps the internal short representation of a context type (internal
    constant, e.g. QBI_SVC_BC_CONTEXT_TYPE_NONE) to its UUID representation

    @details

    @param context_type_id

    @return const uint8* Pointer to array of QBI_MBIM_UUID_LEN bytes
    containing UUID, or NULL if invalid context_type_id provided
*/
/*=========================================================================*/
const uint8 *qbi_svc_bc_context_type_id_to_uuid
(
  uint32 context_type_id
);

/*===========================================================================
  FUNCTION: qbi_svc_bc_context_type_uuid_to_id
===========================================================================*/
/*!
    @brief Maps a UUID to its short internal representation (internal
    constant, e.g. QBI_SVC_BC_CONTEXT_TYPE_NONE)

    @details

    @param context_type_uuid
    @param context_type_id If this function returns TRUE, will be populated
    with the context type ID for the given UUID. If this function returns
    FALSE, then undefined.

    @return boolean TRUE if the mapping was successful, FALSE otherwise
*/
/*=========================================================================*/
boolean qbi_svc_bc_context_type_uuid_to_id
(
  const uint8 *context_type_uuid,
  uint32      *context_type_id
);

/*! @addtogroup MBIM_CID_DEVICE_CAPABILITIES
    @{ */

/*===========================================================================
  FUNCTION: qbi_svc_bc_device_caps_get_data_class
===========================================================================*/
/*!
    @brief Returns the DataClass reported in MBIM_CID_DEVICE_CAPABILITIES

    @details
    This reflects the data capabilites of the modem

    @param ctx

    @return uint32 MBIM_DATA_CLASS value, or zero on failure
*/
/*=========================================================================*/
uint32 qbi_svc_bc_device_caps_get_data_class
(
  const qbi_ctx_s *ctx
);

/*! @} */

/*===========================================================================
  FUNCTION: qbi_svc_bc_module_cache_get
===========================================================================*/
/*!
    @brief Gets a pointer to the structure containing pointers to the cache
    areas owned by the individual modules of the Basic Connectivity device
    service

    @details

    @param ctx

    @return qbi_svc_bc_module_cache_s* Pointer to module cache, or NULL on
    failure
*/
/*=========================================================================*/
qbi_svc_bc_module_cache_s *qbi_svc_bc_module_cache_get
(
  const qbi_ctx_s *ctx
);

/*===========================================================================
  FUNCTION: qbi_svc_bc_provider_add
===========================================================================*/
/*!
    @brief Allocates a new MBIM_PROVIDER structure in the data buffer of
    the response, and populates it with the given data

    @details

    @param txn
    @param provider_field
    @param mcc MCC if cellular_class is GSM, otherwise ignored
    @param mnc MNC if cellular_class is GSM, otherwise ignored
    @param mnc_is_3_digits
    @param sid SID if cellular_class is CDMA, otherwise ignored
    @param provider_state
    @param provider_name May be NULL if provider_name_len is 0
    @param provider_name_len Length of provider_name in bytes
    @param provider_name_is_ascii Set to TRUE if provider_name is encoded
    in ASCII, and should be converted to UTF-16
    @param cellular_class Must be either QBI_SVC_BC_CELLULAR_CLASS_GSM or
    QBI_SVC_BC_CELLULAR_CLASS_CDMA - can't be binary OR of the two!
    @param rssi
    @param error_rate

    @return boolean TRUE on success, FALSE on failure
*/
/*=========================================================================*/
boolean qbi_svc_bc_provider_add
(
  qbi_txn_s                   *txn,
  qbi_mbim_offset_size_pair_s *provider_field,
  uint16                       mcc,
  uint16                       mnc,
  boolean                      mnc_is_3_digits,
  uint16                       sid,
  uint32                       provider_state,
  const void                  *provider_name,
  uint32                       provider_name_len,
  boolean                      provider_name_is_ascii,
  uint32                       cellular_class,
  uint32                       rssi,
  uint32                       error_rate
);

/*===========================================================================
  FUNCTION: qbi_svc_bc_provider_id_ascii_to_mcc_mnc
===========================================================================*/
/*!
    @brief Converts an ASCII encoded 3GPP ProviderId to binary MCC and MNC

    @details
    Input is expected to be a 5 or 6 digit ProviderId per MBIM convention.

    @param provider_id_ascii
    @param provider_id_ascii_len Size of the provider_id_ascii buffer in
    bytes (must be at least QBI_SVC_BC_PROVIDER_ID_MIN_LEN_CHARS)
    @param mcc
    @param mnc
    @param mnc_is_3_digits

    @return boolean
*/
/*=========================================================================*/
boolean qbi_svc_bc_provider_id_ascii_to_mcc_mnc
(
  const char *provider_id_ascii,
  uint32      provider_id_ascii_len,
  uint16     *mcc,
  uint16     *mnc,
  boolean    *mnc_is_3_digits
);

/*===========================================================================
  FUNCTION: qbi_svc_bc_provider_id_to_mcc_mnc
===========================================================================*/
/*!
    @brief Parses a MBIM ProviderId UTF-16 string into a binary MCC and
    MNC

    @details

    @param txn
    @param provider_id
    @param initial_offset
    @param mcc
    @param mnc
    @param mnc_is_3_digits

    @return boolean TRUE on success, FALSE on failure
*/
/*=========================================================================*/
boolean qbi_svc_bc_provider_id_to_mcc_mnc
(
  qbi_txn_s                         *txn,
  const qbi_mbim_offset_size_pair_s *provider_id,
  uint32                             initial_offset,
  uint16                            *mcc,
  uint16                            *mnc,
  boolean                           *mnc_is_3_digits
);

/*===========================================================================
  FUNCTION: qbi_svc_bc_provider_populate
===========================================================================*/
/*!
    @brief Populates a MBIM_PROVIDER data structure with the given data

    @details
    Handles initial_offset appropriately whether the MBIM_PROVIDER is
    nested in a MBIM_PROVIDERS list or not. Performs DataBuffer
    consolidation when complete.

    @param txn
    @param provider
    @param mcc MCC if cellular_class is GSM, otherwise ignored
    @param mnc MNC if cellular_class is GSM, otherwise ignored
    @param mnc_is_3_digits
    @param sid SID if cellular_class is CDMA, otherwise ignored
    @param provider_state
    @param provider_name May be NULL if provider_name_len is 0
    @param provider_name_len Length of provider_name in bytes
    @param provider_name_is_ascii Set to TRUE if provider_name is encoded
    in ASCII, and should be converted to UTF-16
    @param cellular_class Must be either QBI_SVC_BC_CELLULAR_CLASS_GSM or
    QBI_SVC_BC_CELLULAR_CLASS_CDMA - can't be binary OR of the two!
    @param rssi
    @param error_rate

    @return boolean TRUE on success, FALSE on failure
*/
/*=========================================================================*/
boolean qbi_svc_bc_provider_populate
(
  qbi_txn_s             *txn,
  qbi_svc_bc_provider_s *provider,
  uint16                 mcc,
  uint16                 mnc,
  boolean                mnc_is_3_digits,
  uint16                 sid,
  uint32                 provider_state,
  const void            *provider_name,
  uint32                 provider_name_len,
  boolean                provider_name_is_ascii,
  uint32                 cellular_class,
  uint32                 rssi,
  uint32                 error_rate
);

/*===========================================================================
  FUNCTION: qbi_svc_bc_qmi_plmn_name_to_provider_name
===========================================================================*/
/*!
    @brief Converts the information returned by QMI_NAS_GET_PLMN_NAME into a
    UTF-16 encoded ProviderName

    @details

    @param nas44_rsp
    @param provider_name_utf16 Buffer which will be populated with the
    ProviderName
    @param provider_name_utf16_len Size of the provider_name_utf16 buffer in
    bytes
    @param name_pref Preferred network name source

    @return uint32 Number of bytes set in provider_name_utf16
*/
/*=========================================================================*/
uint32 qbi_svc_bc_qmi_plmn_name_to_provider_name
(
  const nas_get_plmn_name_resp_msg_v01 *nas44_rsp,
  uint8                                *provider_name_utf16,
  uint32                                provider_name_utf16_len,
  qbi_svc_bc_provider_name_pref_e       name_pref
);

/*===========================================================================
  FUNCTION: qbi_svc_bc_radio_state_is_radio_on
===========================================================================*/
/*!
    @brief Checks the effective state of the radio

    @details

    @param ctx

    @return boolean TRUE if both SW & HW radio switches are ON, FALSE
    otherwise
*/
/*=========================================================================*/
boolean qbi_svc_bc_radio_state_is_radio_on
(
  const qbi_ctx_s *ctx
);

#endif /* QBI_SVC_BC_COMMON_H */

