/*!
  @file
  qbi_svc_msuicc.h

  @brief
  Microsoft UICC device service implementation, based on the "MBIM EXTENSION
  FOR LOW-LEVEL UICC ACCESS INTERFACE SPECIFICATION" document from Microsoft.
  This device service provides low-level access to the UICC.
*/

/*=============================================================================

  Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
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
07/05/17  mm   Moved structure and function proto here
04/29/17  mm   Added macros
07/28/16  hz   Added module
=============================================================================*/

#ifndef QBI_SVC_MSUICC_H
#define QBI_SVC_MSUICC_H

/*=============================================================================

  Include Files

=============================================================================*/

#include "user_identity_module_v01.h"
#include "qbi_util.h"
#include "qbi_common.h"

/*=============================================================================

  Constants and Macros

=============================================================================*/

/*=============================================================================

  Typedefs

=============================================================================*/

/*! Cache used locally by CIDs processed in this file. */
typedef struct qbi_svc_msuicc_cache_struct {
  /*! List containing one qbi_svc_msuicc_logical_channel_s element for each
      logical channel opened. */
  qbi_util_list_s logical_channel_list;

  /*! Store UIM_TERMINAL_CAPABILITY_RESP message from last sucessful
      MBIM_CID_MS_UICC_TERMINAL_CAPABILITY set request. */
  uim_terminal_capability_resp_msg_v01 qmi_terminal_capability_resp;

  /*! Store token and data trunks from QMI_UIM_SEND_APDU_RESP and
      QMI_UIM_SEND_APDU_IND for reconstructing the APDU response. */
  struct {
    uint32_t token;
    uint8   *apdu;
    uint16_t total_length;
    uint16_t bytes_copied;
  } apdu_response;
} qbi_svc_msuicc_cache_s;

/*! Logical channel ID and its associated tag assigned by host */
typedef struct {
  /*! Must be first as we alias */
  qbi_util_list_entry_s list_entry;

  uint32 channel_id;
  uint32 channel_group;
} qbi_svc_msuicc_logical_channel_s;

/*=============================================================================

  Function Prototypes

=============================================================================*/

qbi_svc_msuicc_cache_s *qbi_svc_msuicc_cache_get
(
  const qbi_ctx_s *ctx
);

boolean qbi_svc_msuicc_logical_channel_remove_from_cache
(
  qbi_ctx_s *ctx,
  uint32     channel_id
);

/*===========================================================================
  FUNCTION: qbi_svc_msuicc_init
===========================================================================*/
/*!
    @brief One-time initialization of the MS Low_Level UICC Access device
    service

    @details

*/
/*=========================================================================*/
void qbi_svc_msuicc_init
(
  void
);

#endif /* QBI_SVC_MSUICC_H */

