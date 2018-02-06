/**
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <errno.h>
#include <utils/Log.h>
#include "TAInterface.h"
#include "QSEEComAPI.h"

DECLARE_SEND_CMD_FUNC(sampleextauth_send_cmd)
{
  int rv = 0;
  uint32_t cmdId = 0;

  do {
    if (cmdLen < sizeof(uint32_t)) {
      ALOGE("cmdLen too short: %u vs %u min", cmdLen, sizeof(uint32_t));
      rv = -EINVAL;
      break;
    }
    cmdId = *(uint32_t*)cmd;
    switch (cmdId) {
      case 1: // SECURE_EXT_AUTH_CMD_GET_TOKEN
        rv = 0;
        break;
      default:
        ALOGE("Unsupported command 0x%08X", cmdId);
        rv = -EINVAL;
        break;
    }
    if (rv) break;

    QSEECom_set_bandwidth((struct QSEECom_handle*)handle, 1);
    rv = QSEECom_send_cmd((struct QSEECom_handle*)handle, cmd, cmdLen, rsp, rspLen);
    QSEECom_set_bandwidth((struct QSEECom_handle*)handle, 0);

  } while (0);

  return rv;
}
