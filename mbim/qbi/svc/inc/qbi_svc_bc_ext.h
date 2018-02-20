/*!
  @file
  qbi_svc_bc_ext.h

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
10/09/17  nk   Fixed warnings and KW p1 issues
09/08/17  vk   Exposing BC EXT cache for usage in BC
07/26/17  mm   Added header files and moved function proto here
06/02/17  vk   Added module
=============================================================================*/

#ifndef QBI_SVC_BC_EXT_H
#define QBI_SVC_BC_EXT_H

/*=============================================================================

  Include Files

=============================================================================*/

#include "qbi_svc.h"
#include "qbi_txn.h"

/*=============================================================================

  Constants and Macros

=============================================================================*/

/*! Maximum number of simultaneous data sessions supported (per QBI context) */
#define QBI_SVC_BC_EXT_MAX_SESSIONS (8)

/*=============================================================================

  Typedefs

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

/*=============================================================================

  Function Prototypes

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
qbi_svc_bc_ext_cache_s *qbi_svc_bc_ext_cache_get
(
  qbi_ctx_s *ctx,
  const uint32 cache_index
);

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_provision_card
===========================================================================*/
/*!
    @brief Configures card provision sessions

    @details

    @param txn

    @return qbi_svc_action_e
*/
/*=========================================================================*/
qbi_svc_action_e qbi_svc_bc_ext_provision_card
(
  qbi_txn_s *txn
);

/*! @addtogroup MBIM_CID_MS_LTE_ATTACH_CONFIG
    @{ */

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
);

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
);


/*! @} */

/*! @addtogroup MBIM_CID_MS_LTE_ATTACH_STATUS
@{ */

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
);

/*! @} */

/*! @addtogroup
@{ */
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
);
/*! @} */

/*===========================================================================
  FUNCTION: qbi_svc_bc_ext_init
===========================================================================*/
/*!
    @brief One-time initialization of the Basic Connectivity Extension device
    service

    @details

*/
/*=========================================================================*/
void qbi_svc_bc_ext_init
(
  void
);

#endif /* QBI_SVC_BC_EXT_H */

