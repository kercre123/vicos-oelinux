/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2015 Qualcomm Technologies, Inc.  All Rights Reserved.
  Qualcomm Technologies Proprietary and Confidential.
=============================================================================*/

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

                 DR Plugin Server generic definition header

GENERAL DESCRIPTION
 This file contains generic definition header for Dead Reckoning API interfaces.

=============================================================================*/
#ifndef __DRPLUGIN_SERVER_MAIN_H
#define __DRPLUGIN_SERVER_MAIN_H
#include "loc_drplugin_srvr_log.h"

#define SOCKET_NAME "/data/misc/location/mq/location-APDR-s"
#define LOGFILE_NAME "/data/misc/location/drPlugin.log"
#define QDRINFO_NAME "/data/misc/location/mq/qdr-info.txt"

#define MAX_SOCKET_CONNECTION_IN_Q 5
#define MAX_LOG_STRING_SIZE 1023  /* 1K Buffer Size */
#define MAX_LOG_FILE_SIZE   1048576 /* 1M */
#define MAX_DR_STORAGE_INDEX 1024 /* 1K node */
#define MAX_DR_DATA_SIZE     10240 /* 10 K node */

/**
 * @brief Provide DR Server Context.
 *
 *
 * @return drplugin_serverContext
 *
 */
drplugin_serverContext* loc_drSrv_contxt(void);

/**
 * @brief DR Plugin Callback.
 *
 *
 * @return dr_callbacks_t
 *
 */
dr_callbacks_t* loc_drSrv_extLib(void);

#endif //__DRPLUGIN_SERVER_MAIN_H
