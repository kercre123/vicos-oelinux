#ifndef __LOWI_DEFINES_H__
#define __LOWI_DEFINES_H__

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

        LOWI Defines Interface Header file

GENERAL DESCRIPTION
  This file contains the structure definitions and function prototypes for
  LOWIDefines

Copyright (c) 2012 Qualcomm Atheros, Inc.
  All Rights Reserved.
  Qualcomm Atheros Confidential and Proprietary.

Copyright (c) 2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.

Export of this technology or software is regulated by the U.S. Government.
Diversion contrary to U.S. law prohibited.
=============================================================================*/
#ifndef __STDC_FORMAT_MACROS
  #define __STDC_FORMAT_MACROS
#endif
#include <ctype.h>
#include <inttypes.h>
#include <stdint.h>

#if defined (LOWI_ON_ACCESS_POINT) || defined (IZAT_OFFLINE)
  #include <osal/commontypes.h>
  #include <base_util/string_routines.h>
#else
  #include <platform_lib_macros.h>
  #include "comdef.h"
#endif

// This is here because for some reason strlcat and strlcpy
// are not available on Ubuntu but on Android
#ifdef USE_GLIB
#include <glib.h>
#define strlcpy g_strlcpy
#define strlcat g_strlcat
#endif

// Config file
#ifdef __ANDROID__
#define CONFIG_NAME LOC_PATH_LOWI_CONF
#else
#ifndef LOWI_ON_ACCESS_POINT
    #ifndef ROOT_DIR
    #define ROOT_DIR ""
    #endif
    #define CONFIG_NAME ROOT_DIR "lowi.conf"
#else
    #define CONFIG_NAME "/etc/lowi/lowi.conf"
#endif
#endif // #ifdef __ANDROID__

#endif //#ifndef __LOWI_DEFINES_H__
