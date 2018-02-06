/**
 * Copyright (c) 2015 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <binder/IPCThreadState.h>
#include <binder/IServiceManager.h>
#include <pthread.h>
#include <unistd.h>
#include "QSEEConnectorService.h"
#include "TAInterface.h"
#include "QSEEComAPI.h"

#ifndef SYSTEM_UID
#define SYSTEM_UID 1000
#endif

#ifndef ROOT_UID
#define ROOT_UID 0
#endif


#undef LOG_TAG
#define LOG_TAG EXPAND_AND_QUOTE(QSEE_CONNECTOR_SERVICE_NAME) EXPAND_AND_QUOTE(-daemon)

#define LOCKW() \
      rv = pthread_rwlock_wrlock(&lock); \
      if (rv) { \
        ALOGE("Error W-locking rwlock: %d (%s)", rv, strerror(rv)); \
        break; \
      } \
      locked = 1

#define LOCKR() \
      rv = pthread_rwlock_rdlock(&lock); \
      if (rv) { \
        ALOGE("Error R-locking rwlock: %d (%s)", rv, strerror(rv)); \
        break; \
      } \
      locked = 1

#define UNLOCK() \
      if (locked) { \
        pthread_rwlock_unlock(&lock); \
        locked = 0; \
      }

#define QSEECONNECTOR_ACCESS_PERMISSION EXPAND_AND_QUOTE(QSEE_CONNECTOR_SERVICE_NAME) EXPAND_AND_QUOTE(.permission.ACCESS_TA)

namespace android {

  status_t QSEEConnectorService::load(const sp<IQSEEConnectorServiceCb> &notifier, const std::string &path, const std::string &name, uint32_t size, uint32_t &handle) {
    status_t ret = BAD_VALUE;
    int rv = 0;
    int handleCheck = 0;
    size_t i = 0;
    int locked = 0;
    void* qseeComHandle = NULL;
    bool preLoadRun = false;
    ALOGD("load");
    do {
      int uid = IPCThreadState::self()->getCallingUid();
      uid %= 100000;
      if ((uid != SYSTEM_UID) && (uid != ROOT_UID)) {
        ALOGE("Bad UID: %d", uid);
        ret = PERMISSION_DENIED;
        break;
      }
      if (path.empty() || name.empty()) {
        ALOGE("Bad name/path");
        ret = BAD_VALUE;
        break;
      }
      for (i = 0; i < supportedTALen; i++) {
        if ((supportedTA[i].send_cmd == NULL) && (supportedTA[i].send_modified_cmd == NULL))
          continue;
        if ((supportedTA[i].name == NULL) || (strlen(supportedTA[i].name) == 0))
          continue;
        if ((supportedTA[i].name[strlen(supportedTA[i].name) - 1] == '*') &&
            (strlen(supportedTA[i].name) >= 2)) {
          if (strncmp(name.c_str(), supportedTA[i].name, strlen(supportedTA[i].name) - 1) == 0) {
            ALOGD("TA supported: %s (%s)", name.c_str(), supportedTA[i].name);
            break;
          }
        } else {
          if (name.compare(supportedTA[i].name) == 0) {
            ALOGD("TA supported: %s", name.c_str());
            break;
          }
        }
      }
      if (i == supportedTALen) {
        ALOGE("TA %s not supported", name.c_str());
        ret = BAD_VALUE;
        break;
      }
      LOCKW();
      do {
        handle = rand();
      } while ((handlesMap.find(handle) != handlesMap.end()) && (handleCheck++ < 1000));
      if (handlesMap.find(handle) != handlesMap.end()) {
        ALOGE("Cannot allocate handle");
        ret = BAD_VALUE;
        break;
      }
      // adjust size
      if (supportedTA[i].get_size) {
        uint32_t taSize = supportedTA[i].get_size();
        if (taSize > size) {
          ALOGI("Increasing size as requested by TA: %u -> %u bytes", size, taSize);
          size = taSize;
        }
      }
      if (supportedTA[i].pre_load) {
        rv = supportedTA[i].pre_load(path.c_str());
        if (rv) {
          ALOGE("Error executing pre_load() for TA %s: %d", name.c_str(), rv);
          ret = BAD_VALUE;
          break;
        }
        preLoadRun = true;
      }
      rv = QSEECom_start_app((struct QSEECom_handle**)&qseeComHandle, path.c_str(), name.c_str(), size);
      if (preLoadRun && supportedTA[i].post_load) {
        int rv1 = supportedTA[i].post_load();
        if (rv1) {
          ALOGE("Error executing post_load() for TA %s: %d", name.c_str(), rv1);
          if (rv == 0) {
            // unload the main app, ignore errors since we won't be able to deal with them
            QSEECom_shutdown_app((struct QSEECom_handle**)&qseeComHandle);
          }
          ret = BAD_VALUE;
          break;
        }
      }
      if (rv) {
        ALOGE("Loading app [%s] failed (%u bytes)\n", name.c_str(), size);
        ret = BAD_VALUE;
        break;
      }
      ALOGD("Loading app [%s] succeeded (%u bytes)\n", name.c_str(), size);
      struct service_data data = {0};
      data.buffer = ((struct QSEECom_handle*)qseeComHandle)->ion_sbuffer;
      data.callingPid = IPCThreadState::self()->getCallingPid();
      data.l_QSEEComHandle = qseeComHandle;
      data.l_size = size;
      data.index = i;
      data.l_serviceCb = asBinder(notifier);
      data.l_deathNotifier = new ClientDiedNotifier(handle, this);
      if (data.l_serviceCb == 0) {
        ALOGE("No notifier");
        ret = BAD_VALUE;
        break;
      }
      data.l_serviceCb->linkToDeath(data.l_deathNotifier);

      handlesMap.insert( std::pair<uint32_t, struct service_data>
              (handle, data));
      ret = NO_ERROR;

    } while (0);
    if (ret != NO_ERROR) {
      if (qseeComHandle) {
        QSEECom_shutdown_app((struct QSEECom_handle**)&qseeComHandle);
      }
    }
    UNLOCK();
    return ret;
  }

  status_t QSEEConnectorService::unload(uint32_t handle) {
    status_t ret = NO_ERROR;
    int rv = 0;
    int locked = 0;
    ALOGD("unload");
    do {
      int uid = IPCThreadState::self()->getCallingUid();
      uid %= 100000;
      if ((uid != SYSTEM_UID) && (uid != ROOT_UID)) {
        ALOGE("Bad UID: %d", uid);
        ret = PERMISSION_DENIED;
        break;
      }
      LOCKW();
      if (handlesMap.find(handle) == handlesMap.end()) {
        ALOGE("No matching handle");
        ret = BAD_VALUE;
        break;
      }
      if ((handlesMap[handle].callingPid != IPCThreadState::self()->getCallingPid()) &&
          (getpid() != IPCThreadState::self()->getCallingPid())) {
        ALOGE("Mismatched PID");
        ret = BAD_VALUE;
        break;
      }
      if (handlesMap[handle].l_QSEEComHandle == NULL) {
        ALOGW("App not loaded");
        if (handlesMap[handle].l_serviceCb != 0 && handlesMap[handle].l_deathNotifier != 0) {
          handlesMap[handle].l_serviceCb->unlinkToDeath(handlesMap[handle].l_deathNotifier);
        }
        handlesMap.erase(handle);
        break;
      }
      rv = QSEECom_shutdown_app((struct QSEECom_handle**)&(handlesMap[handle].l_QSEEComHandle));
      if (rv) {
        ALOGE("Failed to unload app [%s]: %d (%s)\n", supportedTA[handlesMap[handle].index].name, rv, strerror(errno));
        ret = BAD_VALUE;
        // nothing we can do about it, so continue and clear up
      }
      if (handlesMap[handle].l_serviceCb != 0 && handlesMap[handle].l_deathNotifier != 0) {
        handlesMap[handle].l_serviceCb->unlinkToDeath(handlesMap[handle].l_deathNotifier);
      }
      handlesMap.erase(handle);
    } while (0);
    UNLOCK();
    return ret;
  }

  status_t QSEEConnectorService::sendCommand(uint32_t handle, void * cmd, uint32_t cmdLen, void * rsp, uint32_t rspLen) {
    status_t ret = BAD_VALUE;
    int rv = 0;
    int locked = 0;
    struct service_data data = {0};
    ALOGD("sendCommand");
    do {
      int uid = IPCThreadState::self()->getCallingUid();
      uid %= 100000;
      if ((uid != SYSTEM_UID) && (uid != ROOT_UID)) {
        ALOGE("Bad UID: %d", uid);
        ret = PERMISSION_DENIED;
        break;
      }
      LOCKR();
      if (handlesMap.find(handle) == handlesMap.end()) {
        ALOGE("No matching handle");
        ret = BAD_VALUE;
        break;
      }
      data = handlesMap[handle];
      UNLOCK();
      if (data.callingPid != IPCThreadState::self()->getCallingPid()) {
        ALOGE("Mismatched PID");
        ret = BAD_VALUE;
        break;
      }
      if (supportedTA[data.index].send_cmd == NULL) {
        ALOGE("No matching function");
        ret = BAD_VALUE;
        break;
      }
      rv = supportedTA[data.index].send_cmd(data.l_QSEEComHandle, cmd, cmdLen, rsp, rspLen);
      if (rv) {
        ALOGE("Error in send_cmd: %d (%s)", rv, strerror(errno));
        ret = BAD_VALUE;
        break;
      }
      ret = NO_ERROR;
    } while (0);
    UNLOCK();
    return ret;
  }

  status_t QSEEConnectorService::sendModifiedCommand(uint32_t handle, void * cmd, uint32_t cmdLen, void * rsp, uint32_t rspLen, struct QSEECom_ion_fd_info *info) {
    status_t ret = BAD_VALUE;
    int rv = 0;
    int locked = 0;
    struct service_data data = {0};
    ALOGD("sendModifiedCommand");
    do {
      int uid = IPCThreadState::self()->getCallingUid();
      uid %= 100000;
      if ((uid != SYSTEM_UID) && (uid != ROOT_UID)) {
        ALOGE("Bad UID: %d", uid);
        ret = PERMISSION_DENIED;
        break;
      }
      LOCKR();
      if (handlesMap.find(handle) == handlesMap.end()) {
        ALOGE("No matching handle");
        ret = BAD_VALUE;
        break;
      }
      data = handlesMap[handle];
      UNLOCK();
      if (data.callingPid != IPCThreadState::self()->getCallingPid()) {
        ALOGE("Mismatched PID");
        ret = BAD_VALUE;
        break;
      }
      if (supportedTA[data.index].send_modified_cmd == NULL) {
        ALOGE("No matching function");
        ret = BAD_VALUE;
        break;
      }
      rv = supportedTA[data.index].send_modified_cmd(data.l_QSEEComHandle, cmd, cmdLen, rsp, rspLen, info);
      if (rv) {
        ALOGE("Error in send_modified_cmd: %d (%s)", rv, strerror(errno));
        ret = BAD_VALUE;
        break;
      }
      ret = NO_ERROR;
    } while (0);
    UNLOCK();
    return ret;
  }

  status_t QSEEConnectorService::getAppInfo(uint32_t handle, struct qseecom_app_info* info) {
    status_t ret = BAD_VALUE;
    int rv = 0;
    int locked = 0;
    struct service_data data = {0};
    ALOGD("getAppInfo");
    do {
      int uid = IPCThreadState::self()->getCallingUid();
      uid %= 100000;
      if ((uid != SYSTEM_UID) && (uid != ROOT_UID)) {
        ALOGE("Bad UID: %d", uid);
        ret = PERMISSION_DENIED;
        break;
      }
      LOCKR();
      if (handlesMap.find(handle) == handlesMap.end()) {
        ALOGE("No matching handle");
        ret = BAD_VALUE;
        break;
      }
      data = handlesMap[handle];
      UNLOCK();
      if (data.callingPid != IPCThreadState::self()->getCallingPid()) {
        ALOGE("Mismatched PID");
        ret = BAD_VALUE;
        break;
      }
      rv = QSEECom_get_app_info((struct QSEECom_handle*)data.l_QSEEComHandle, info);
      if (rv) {
        ALOGE("Error in QSEECom_get_app_info: %d (%s)", rv, strerror(errno));
        ret = BAD_VALUE;
        break;
      }
      ret = NO_ERROR;
    } while (0);
    UNLOCK();
    return ret;
  }

  status_t QSEEConnectorService::onTransact(uint32_t code, const Parcel& data, Parcel* reply, uint32_t flags)
  {
    return BnQSEEConnectorService::onTransact(code, data, reply, flags);
  }

  void QSEEConnectorService::ClientDiedNotifier::binderDied(const wp<IBinder> &who) {
    const wp<IBinder> unused = who;
    QSEEConnectorService *service = reinterpret_cast <QSEEConnectorService *>(mService);

    ALOGW("Client died");

    service->unload(mHandle);
  }

}; // namespace android
