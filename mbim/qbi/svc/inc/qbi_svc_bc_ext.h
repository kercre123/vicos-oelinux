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
08/24/17  rv   Exposed LTE Attach API
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

/*=============================================================================

  Function Prototypes

=============================================================================*/

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

