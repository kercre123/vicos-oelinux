/**
 * Copyright (c) 2015 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#pragma once

#include <binder/Parcel.h>
#include "IQSEEConnectorService.h"

// this is the client
namespace android {
  class BpQSEEConnectorService: public BpInterface<IQSEEConnectorService> {
    public:
      BpQSEEConnectorService(const sp<IBinder>& impl);

      virtual status_t load(const sp<IQSEEConnectorServiceCb> &notifier, const std::string &path, const std::string& name, uint32_t size, uint32_t &handle);
      virtual status_t unload(uint32_t handle);
      virtual status_t sendCommand(uint32_t handle, void * cmd, uint32_t cmdLen, void * rsp, uint32_t rspLen);
      virtual status_t sendModifiedCommand(uint32_t handle, void * cmd, uint32_t cmdLen, void * rsp, uint32_t rspLen, struct QSEECom_ion_fd_info *info);
      virtual status_t getAppInfo(uint32_t handle, struct qseecom_app_info* info);
  };

  class BnQSEEConnectorServiceCb: public BnInterface<IQSEEConnectorServiceCb> {
    public:
      virtual status_t onTransact(uint32_t code, const Parcel& data, Parcel* reply, uint32_t flags = 0);
  };
}; // namespace android

