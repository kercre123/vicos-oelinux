/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2015 Qualcomm Technologies, Inc.  All Rights Reserved.
  Qualcomm Technologies Proprietary and Confidential.
=============================================================================*/

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

                 DR Plugin Server generic definition header

GENERAL DESCRIPTION
 This file contains generic definition header for Dead Reckoning API interfaces.

=============================================================================*/
#ifndef __DRPLUGIN_SERVER_H
#define __DRPLUGIN_SERVER_H
/**
 * @brief DR Server initialization.
 *
 *
 * @return void
 *
 */
bool loc_drSrv_init(void);

/**
 * @brief DR Server de-initialization.
 *
 *
 * @return void
 *
 */
bool loc_drSrv_deInit(void);

#endif //__DRPLUGIN_SERVER_H
