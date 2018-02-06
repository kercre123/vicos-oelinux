/**
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <stddef.h>
#include "TAInterface.h"

#ifndef LENGTH
#define LENGTH(X) (sizeof(X)/sizeof((X)[0]))
#endif

#ifdef TA_SAMPLEAUTH
DECLARE_SEND_CMD_FUNC(sampleauth_send_cmd);
#endif
#ifdef TA_SAMPLEEXTAUTH
DECLARE_SEND_CMD_FUNC(sampleextauth_send_cmd);
#endif

ta_interface_t const supportedTA[] = {
#ifdef TA_SAMPLEAUTH
  {"sampleauth", NULL, NULL, NULL, sampleauth_send_cmd, NULL},
#endif
#ifdef TA_SAMPLEEXTAUTH
  {"sampleextauth", NULL, NULL, NULL, sampleextauth_send_cmd, NULL},
#endif
  {"", NULL, NULL, NULL},
};

size_t const supportedTALen = LENGTH(supportedTA);

