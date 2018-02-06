/******************************************************************************
  ---------------------------------------------------------------------------

  Copyright (c) 2013-2014 Qualcomm Technologies, Inc. All Rights Reserved.
  Qualcomm Technologies Proprietary and Confidential.
  ---------------------------------------------------------------------------
******************************************************************************/

#include "cri_dms.h"
#include "cri_dms_utils.h"
#include "cri_dms_core.h"
#include "cri_core.h"
#include "device_management_service_v01.h"

// DMS cache
cri_dms_operating_mode modem_operating_mode;

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
void cri_dms_utils_update_operating_mode(dms_operating_mode_enum_v01 opr_mode)
{
    modem_operating_mode.is_valid = TRUE;
    modem_operating_mode.current_operating_mode = opr_mode;
}

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
void cri_dms_utils_get_current_operating_mode(dms_operating_mode_enum_v01 *opr_mode)
{
    if ( modem_operating_mode.is_valid )
    {
        *opr_mode = modem_operating_mode.current_operating_mode;
    }
}


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
uint32_t cri_dms_utils_is_valid_operating_mode(void)
{
    return modem_operating_mode.is_valid;
}



