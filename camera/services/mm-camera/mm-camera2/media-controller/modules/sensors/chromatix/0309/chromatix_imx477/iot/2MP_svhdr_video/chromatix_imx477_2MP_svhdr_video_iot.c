/*============================================================================

Copyright (c) 2018 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.

============================================================================*/

/*============================================================================
 *                      INCLUDE FILES
 *===========================================================================*/
#include "chromatix_iot.h"

static chromatix_iot_parms_type chromatix_imx477_parms = {
#ifdef ENABLED_IMX477_RAW10_MODES
#include "chromatix_imx477_raw10_2MP_svhdr_video_iot.h"
#else
#include "chromatix_imx477_raw12_2MP_svhdr_video_iot.h"
#endif
};

/*============================================================================
 * FUNCTION    - load_chromatix -
 *
 * DESCRIPTION:
 *==========================================================================*/
void *load_chromatix(void)
{
  return &chromatix_imx477_parms;
}
