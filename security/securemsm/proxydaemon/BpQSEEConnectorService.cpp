/**
 * Copyright (c) 2015 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <android/log.h>
#include <binder/Parcel.h>
#include "BpQSEEConnectorService.h"

#undef LOG_TAG
#define LOG_TAG EXPAND_AND_QUOTE(QSEE_CONNECTOR_SERVICE_NAME) EXPAND_AND_QUOTE(-client)

namespace android {

  status_t BpQSEEConnectorService::load(const sp<IQSEEConnectorServiceCb> &notifier, const std::string &path, const std::string &name, uint32_t size, uint32_t &handle) {
    Parcel data, reply;
    status_t rv = NO_ERROR;
    if (path.empty() || name.empty()) {
        ALOGE("NULL params");
        rv = BAD_VALUE;
        return rv;
    }
    data.writeInterfaceToken(BpQSEEConnectorService::getInterfaceDescriptor());
    data.writeStrongBinder(asBinder(notifier));
    data.writeCString(path.c_str());
    data.writeCString(name.c_str());
    data.writeInt32(size);

    // binder call
    rv = remote()->transact(LOAD, data, &reply);
    if (rv != NO_ERROR) {
      ALOGE("Couldn't contact remote: %d", rv);
      return rv;
    }
    int32_t err = reply.readExceptionCode();
    if (err < 0) {
      ALOGE("remote exception: %d", err);
      return err;
    }
    handle = reply.readInt32();
    return rv;
  }

  status_t BpQSEEConnectorService::unload(uint32_t handle) {
    Parcel data, reply;
    status_t rv = NO_ERROR;
    data.writeInterfaceToken(BpQSEEConnectorService::getInterfaceDescriptor());
    data.writeInt32(handle);

    // binder call
    rv = remote()->transact(UNLOAD, data, &reply);
    if (rv != NO_ERROR) {
      ALOGE("Couldn't contact remote: %d", rv);
      return rv;
    }
    int32_t err = reply.readExceptionCode();
    if (err < 0) {
      ALOGE("remote exception: %d", err);
      return err;
    }
    return rv;
  }

  status_t BpQSEEConnectorService::sendCommand(uint32_t handle, void * cmd, uint32_t cmdLen, void * rsp, uint32_t rspLen) {
    Parcel data, reply;
    status_t rv = NO_ERROR;
    do {
      if ((!cmd) || (!rsp) || (cmdLen == 0) || (rspLen == 0)) {
        ALOGE("NULL params");
        rv = BAD_VALUE;
        break;
      }
      data.writeInterfaceToken(BpQSEEConnectorService::getInterfaceDescriptor());
      data.writeInt32(handle);
      data.writeInt32(cmdLen);
      data.writeInt32(rspLen);
      data.write(cmd, cmdLen);
      // binder call
      rv = remote()->transact(SEND_COMMAND, data, &reply);
      if (rv != NO_ERROR) {
        ALOGE("Couldn't contact remote: %d", rv);
        break;
      }
      int32_t err = reply.readExceptionCode();
      if (err < 0) {
        ALOGE("remote exception: %d", err);
        rv = err;
        break;
      }
      reply.read(rsp, rspLen);
      ALOGD("Response read: %u bytes", rspLen);
    } while (0);
    return rv;
  }

  status_t BpQSEEConnectorService::sendModifiedCommand(uint32_t handle, void * cmd, uint32_t cmdLen, void * rsp, uint32_t rspLen, struct QSEECom_ion_fd_info *info)
  {
    Parcel data, reply;
    status_t rv = NO_ERROR;
    size_t i = 0;
    do {
      if ((!cmd) || (!rsp) || (cmdLen == 0) || (rspLen == 0) || (!info)) {
        ALOGE("NULL params");
        rv = BAD_VALUE;
        break;
      }
      data.writeInterfaceToken(BpQSEEConnectorService::getInterfaceDescriptor());
      data.writeInt32(handle);
      data.writeInt32(cmdLen);
      data.writeInt32(rspLen);
      data.write(cmd, cmdLen);
      for (i = 0; i < 4; i++) {
        if (info->data[i].fd < 0) {
          data.writeInt32(0);
        } else {
          data.writeInt32(1);
          data.writeDupFileDescriptor(info->data[i].fd);
        }
        data.writeInt32(info->data[i].cmd_buf_offset);
      }
      // binder call
      rv = remote()->transact(SEND_MODIFIED_COMMAND, data, &reply);
      if (rv != NO_ERROR) {
        ALOGE("Couldn't contact remote: %d", rv);
        break;
      }
      int32_t err = reply.readExceptionCode();
      if (err < 0) {
        ALOGE("remote exception: %d", err);
        rv = err;
        break;
      }
      reply.read(rsp, rspLen);
      ALOGD("Response read: %u bytes", rspLen);
    } while (0);
    return rv;
  }

  status_t BpQSEEConnectorService::getAppInfo(uint32_t handle, struct qseecom_app_info* info)
  {
    Parcel data, reply;
    status_t rv = NO_ERROR;
    size_t i = 0;
    do {
      if (!info) {
        ALOGE("NULL params");
        rv = BAD_VALUE;
        break;
      }
      data.writeInterfaceToken(BpQSEEConnectorService::getInterfaceDescriptor());
      data.writeInt32(handle);
      // binder call
      rv = remote()->transact(GET_APP_INFO, data, &reply);
      if (rv != NO_ERROR) {
        ALOGE("Couldn't contact remote: %d", rv);
        break;
      }
      int32_t err = reply.readExceptionCode();
      if (err < 0) {
        ALOGE("remote exception: %d", err);
        rv = err;
        break;
      }
      reply.read((void*)info, sizeof(struct qseecom_app_info));
      ALOGD("Info read: %u bytes", sizeof(struct qseecom_app_info));
    } while (0);
    return rv;
  }

  BpQSEEConnectorService::BpQSEEConnectorService(const sp<IBinder>& impl) : BpInterface<IQSEEConnectorService>(impl){}

  status_t BnQSEEConnectorServiceCb::onTransact(uint32_t code, const Parcel& data, Parcel* reply, uint32_t flags)
  {
    status_t ret = UNKNOWN_ERROR;

    switch(code) {

      case NOTIFY_CALLBACK: {
          CHECK_INTERFACE(IQSEEConnectorServiceCb, data, reply);
          int32_t event = data.readInt32();
          notifyCallback(event);
          reply->writeNoException();
        }
        break;

      default:
        ALOGD("Unknown binder command ID: %d", code);
        return BBinder::onTransact(code, data, reply, flags);
    }
    return ret;
  }

}; // namespace android
