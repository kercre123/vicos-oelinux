/**
 * Copyright (c) 2015 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#pragma once

#include <map>

#include <binder/Parcel.h>
#include <binder/BinderService.h>
#include "BnQSEEConnectorService.h"
#include "BpQSEEConnectorService.h"

namespace android {

  class QSEEConnectorService: public BinderService<QSEEConnectorService>,  public BnQSEEConnectorService
  {
    friend class BinderService<QSEEConnectorService>;

    private:

      class ClientDiedNotifier: public IBinder::DeathRecipient {
        public:
          ClientDiedNotifier(uint32_t handle, void *service) {
            mHandle = handle;
            mService = service;
          }
        protected:
          // Invoked by binder when client crash. Implemented by Server
          virtual void binderDied(const wp<IBinder> &who);

        private:
          uint32_t mHandle;
          // QSEEConnectorService instance
          void *mService;
      };

      struct service_data {
        // TODO: add PID checked via IPCThreadState::getCallingPid()
        int callingPid;
        size_t index;
        void* l_QSEEComHandle;
        uint32_t l_size;
        sp<IBinder> l_serviceCb;
        sp<QSEEConnectorService::ClientDiedNotifier> l_deathNotifier;
        uint8_t *buffer;
      };

      std::map<uint32_t, struct service_data> handlesMap;
      pthread_rwlock_t lock;

    public:
      QSEEConnectorService():lock(PTHREAD_RWLOCK_INITIALIZER) {};
      virtual ~QSEEConnectorService() {};
      virtual status_t onTransact(uint32_t code, const Parcel& data, Parcel* reply, uint32_t flags = 0);

      virtual status_t load(const sp<IQSEEConnectorServiceCb> &notifier, const std::string &path, const std::string &name, uint32_t size, uint32_t &handle);
      virtual status_t unload(uint32_t handle);
      virtual status_t sendCommand(uint32_t handle, void * cmd, uint32_t cmdLen, void * rsp, uint32_t rspLen);
      virtual status_t sendModifiedCommand(uint32_t handle, void * cmd, uint32_t cmdLen, void * rsp, uint32_t rspLen, struct QSEECom_ion_fd_info *info);
      virtual status_t getAppInfo(uint32_t handle, struct qseecom_app_info* info);

  };

  class QSEEConnectorServiceCb: public BnQSEEConnectorServiceCb {
    public:
      virtual void notifyCallback(int32_t event){};
  };
}; // namespace android

