/**
 * Copyright (c) 2015 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <android/log.h>
#include "BnQSEEConnectorService.h"
#include <string>

#undef LOG_TAG
#define LOG_TAG EXPAND_AND_QUOTE(QSEE_CONNECTOR_SERVICE_NAME) EXPAND_AND_QUOTE(-server)

namespace android {

  status_t BnQSEEConnectorService::onTransact(uint32_t code, const Parcel& data, Parcel* reply, uint32_t flags)
  {
    status_t ret = UNKNOWN_ERROR;

    switch(code) {

      case LOAD: {
          CHECK_INTERFACE(IQSEEConnectorService, data, reply);
          sp<IQSEEConnectorServiceCb> notifier = interface_cast <IQSEEConnectorServiceCb> (data.readStrongBinder());
          const char *path = data.readCString();
          const char *name = data.readCString();
          uint32_t size = data.readInt32();
          uint32_t handle = 0;
          ret = load(notifier, path, name, size, handle);
          reply->writeNoException();
          if (ret == NO_ERROR) {
            reply->writeInt32(handle);
          }
        }
        break;

      case UNLOAD: {
          CHECK_INTERFACE(IQSEEConnectorService, data, reply);
          uint32_t handle = data.readInt32();
          ret = unload(handle);
          reply->writeNoException();
        }
        break;

      case SEND_COMMAND: {
          CHECK_INTERFACE(IQSEEConnectorService, data, reply);
          uint32_t handle = data.readInt32();
          uint32_t cmdLen = data.readInt32();
          uint32_t rspLen = data.readInt32();
          uint8_t *cmd = NULL;
          uint8_t *rsp = NULL;
          int rv = 0;
          if (UINT32_MAX - cmdLen < rspLen) {
            ALOGE("Attempted buffer overflow: %u %u", cmdLen, rspLen);
            ret = BAD_VALUE;
            break;
          }
          if ((cmdLen == 0) || (rspLen == 0)) {
            ALOGE("Zero length: %u %u", cmdLen, rspLen);
            ret = BAD_VALUE;
            break;
          }
          rsp = new uint8_t[rspLen];
          if (rsp == NULL) {
            ALOGE("Zero length: %u %u", cmdLen, rspLen);
            ret = BAD_VALUE;
            break;
          }

          cmd = (uint8_t*) data.readInplace(cmdLen);

          ret = sendCommand(handle, cmd, cmdLen, rsp, rspLen);
          reply->writeNoException();

          if (ret == NO_ERROR) {
            reply->write(rsp, rspLen);
            ALOGD("Response written: %u bytes", rspLen);
          }
          delete [] rsp;
        }
        break;

      case SEND_MODIFIED_COMMAND: {
          CHECK_INTERFACE(IQSEEConnectorService, data, reply);
          uint32_t handle = data.readInt32();
          uint32_t cmdLen = data.readInt32();
          uint32_t rspLen = data.readInt32();
          uint8_t *cmd = NULL;
          uint8_t *rsp = NULL;
          int rv = 0;
          QSEECom_ion_fd_info info = {0};
          size_t i = 0;
          if (UINT32_MAX - cmdLen < rspLen) {
            ALOGE("Attempted buffer overflow: %u %u", cmdLen, rspLen);
            ret = BAD_VALUE;
            break;
          }
          if ((cmdLen == 0) || (rspLen == 0)) {
            ALOGE("Zero length: %u %u", cmdLen, rspLen);
            ret = BAD_VALUE;
            break;
          }
          rsp = new uint8_t[rspLen];
          if (rsp == NULL) {
            ALOGE("Zero length: %u %u", cmdLen, rspLen);
            ret = BAD_VALUE;
            break;
          }

          cmd = (uint8_t*) data.readInplace(cmdLen);

          for (i = 0; i < 4; i++) {
            if(data.readInt32() == 0) {
              info.data[i].fd = -1;
            } else {
              info.data[i].fd = data.readFileDescriptor();
            }
            info.data[i].cmd_buf_offset = data.readInt32();
          }

          ret = sendModifiedCommand(handle, cmd, cmdLen, rsp, rspLen, &info);
          reply->writeNoException();

          if (ret == NO_ERROR) {
            reply->write(rsp, rspLen);
            ALOGD("Response written: %u bytes", rspLen);
          }
          delete [] rsp;
        }
        break;

      case GET_APP_INFO: {
          CHECK_INTERFACE(IQSEEConnectorService, data, reply);
          uint32_t handle = data.readInt32();
          int rv = 0;
          struct qseecom_app_info info = {0};
          size_t i = 0;

          ret = getAppInfo(handle, &info);
          reply->writeNoException();

          if (ret == NO_ERROR) {
            reply->write(&info, sizeof(info));
            ALOGD("Info written: %u bytes", sizeof(info));
          }
        }
        break;

      default:
        ALOGD("Unknown binder command ID: %d", code);
        return BBinder::onTransact(code, data, reply, flags);
    }
    return ret;
  }

  BpQSEEConnectorServiceCb::BpQSEEConnectorServiceCb(const sp<IBinder>& impl) : BpInterface<IQSEEConnectorServiceCb>(impl){}

  void BpQSEEConnectorServiceCb::notifyCallback(int32_t event) {
    Parcel data, reply;
    status_t rv = NO_ERROR;
      data.writeInterfaceToken(BpQSEEConnectorServiceCb::getInterfaceDescriptor());
    data.writeInt32(event);
    // binder call
    rv = remote()->transact(NOTIFY_CALLBACK, data, &reply);
    if (rv != NO_ERROR) {
      ALOGE("Couldn't contact remote: %d", rv);
      return;
    }
    int32_t err = reply.readExceptionCode();
    if (err < 0) {
      ALOGE("remote exception: %d", err);
      return;
    }
  }

}; // namespace android
