
/******************************************************************************
  ---------------------------------------------------------------------------

  Copyright (c) 2013-2014 Qualcomm Technologies, Inc. All Rights Reserved.
  Qualcomm Technologies Proprietary and Confidential.
  ---------------------------------------------------------------------------
******************************************************************************/

#ifndef CRI_DMS_UTILS
#define CRI_DMS_UTILS

#include "utils_common.h"
#include "device_management_service_v01.h"

// DMS cache for storing operating mode received from indication
typedef struct cri_dms_operating_mode
{
    int is_valid;
    dms_operating_mode_enum_v01 current_operating_mode;
}cri_dms_operating_mode;

/***************************************************************************************************
    @function
    cri_dms_utils_update_operating_mode

    @brief
    Update DMS cache.

    @param[in]
        opr_mode
            current operating mode.

    @param[out]
        none

    @retval
    none
***************************************************************************************************/
void cri_dms_utils_update_operating_mode(dms_operating_mode_enum_v01 opr_mode);

/***************************************************************************************************
    @function
    cri_dms_utils_get_current_operating_mode

    @brief
    Retrieve current operating modem from cache.

    @param[in]
        none

    @param[out]
        opr_mode
            current operating mode.

    @retval
    none
***************************************************************************************************/
void cri_dms_utils_get_current_operating_mode(dms_operating_mode_enum_v01 *opr_mode);

/***************************************************************************************************
    @function
    cri_dms_utils_is_valid_operating_mode

    @brief
    Check if DMS cache is valid or not.

    @param[in]
        none

    @param[out]
        none

    @retval
    uint32_t - true if cache is valid otherwise false.
***************************************************************************************************/
uint32_t cri_dms_utils_is_valid_operating_mode(void);

#endif

