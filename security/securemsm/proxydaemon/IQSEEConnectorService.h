/**
 * Copyright (c) 2015 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#pragma once

#include <binder/IInterface.h>
#include <stdint.h>
#include <string>
#include <utility>
#include "QSEEComAPI.h"

#define QUOTE(x) #x
#define EXPAND_AND_QUOTE(str) QUOTE(str)

namespace android {

  class IQSEEConnectorServiceCb: public IInterface {
    public:
      static const char* getServiceName();

      enum {
        NOTIFY_CALLBACK = IBinder::FIRST_CALL_TRANSACTION,
      };

      virtual void notifyCallback(int32_t event) = 0;

      DECLARE_META_INTERFACE(QSEEConnectorServiceCb);
  };

  class IQSEEConnectorService : public IInterface {
    public:
      static const char* getServiceName();

      enum {
        LOAD = IBinder::FIRST_CALL_TRANSACTION,
        UNLOAD,
        SEND_COMMAND,
        SEND_MODIFIED_COMMAND,
        GET_APP_INFO,
      };

      virtual status_t load(const sp<IQSEEConnectorServiceCb> &notifier, const std::string &path, const std::string &name, uint32_t size, uint32_t &handle) = 0;
      virtual status_t unload(uint32_t handle) = 0;
      virtual status_t sendCommand(uint32_t handle, void * cmd, uint32_t cmdLen, void * rsp, uint32_t rspLen) = 0;
      virtual status_t sendModifiedCommand(uint32_t handle, void * cmd, uint32_t cmdLen, void * rsp, uint32_t rspLen, struct QSEECom_ion_fd_info *info) = 0;
      virtual status_t getAppInfo(uint32_t handle, struct qseecom_app_info* info) = 0;

      DECLARE_META_INTERFACE(QSEEConnectorService);
  };
}; // namespace android
