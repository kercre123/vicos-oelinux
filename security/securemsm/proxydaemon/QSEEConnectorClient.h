/**
 * Copyright (c) 2015 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#pragma once

#ifndef QSEE_CONNECTOR_SERVICE_NAME
#error "Please define QSEE_CONNECTOR_SERVICE_NAME"
#endif

#include <android/log.h>
#include <QSEEConnectorService.h>
#include <stdlib.h>
#include <utils/RefBase.h>
#include <binder/IInterface.h>
#include <binder/IBinder.h>
#include <binder/ProcessState.h>
#include <binder/IServiceManager.h>
#include <binder/IPCThreadState.h>
#include <string>
#include <unistd.h>

#include <IQSEEConnectorService.cpp>
#include <BnQSEEConnectorService.cpp>
#include <BpQSEEConnectorService.cpp>

#ifndef LOG_TAG
#define LOG_TAG "QSEEConnectorClient: "
#define ENTER ALOGD("%s+",__func__)
#define EXIT  ALOGD("%s-",__func__)
#define EXITV(a) ALOGD("%s-: %x", __func__, (a)); return (a)
#define MY_LOG_TAG
#endif




namespace android {

/**
 *
 * @brief Connector class for QSEE Proxy daemon
 *
 * This class encapsulates the client side of the binder connection
 * to the QSEE Proxy daemon.
 * It tries to connect to the daemon on creation, and, in case of failure,
 * it will attempt again every time a command is issued.
 *
 * This class requires the pre-processor macro QSEE_CONNECTOR_SERVICE_NAME
 * to be defined as the C-string name of the QSEE proxy daemon service the
 * class will connect to.
 * */
class QSEEConnectorClient {
  private:
    sp<IQSEEConnectorService> proxy;
    uint32_t handle;
    std::string path;
    std::string name;
    uint32_t size;
    void getServ() {
      do {
        sp<IServiceManager> sm = defaultServiceManager();
        if (sm == 0) {
          ALOGD("NULL sm");
          break;
        }
        sp<IBinder> binder = sm->getService(String16(IQSEEConnectorService::getServiceName()));
        if (binder == 0) {
          ALOGD("NULL binder for service %s", IQSEEConnectorService::getServiceName());
          break;
        }
        proxy = interface_cast<IQSEEConnectorService>(binder);
        if (proxy == 0) {
          ALOGD("NULL proxy");
          break;
        }
      } while(0);
    }

  public:
    /**
     * @brief Constructor
     *
     * @param _path[in] - path in the HLOS file system where the QSEE App is located
     * @param _name[in] - name of the QSEE application to be loaded
     * @param _size[in] - size of the shared memory associated with the QSEE Application
     *
     * @note The size of the shared memory can be overridden by the daemon, if the
     * QSEE Application is configured to require a larger memory than here requested.
     *
     * @note The requested size MUST take into account alignment and padding of all the
     * command and response buffers which are going to be used in subsequent calls to
     * sendCommand and sendModifiedCommand.
     *
     * @note A connection with the daemon is attempted on creation. If such connection fails
     * (e.g. the daemon is not yet available), a connection is attempted again at each access
     * to any of the load/sendCommand/sendModifiedCommend functions.
     *
     * */
    QSEEConnectorClient(
      std::string const &_path,
      std::string const &_name,
      uint32_t _size):proxy(0),handle(0),path(_path),name(_name),size(_size) {
      getServ();
    };

    /**
     * @brief Destructor
     *
     * If the QSEE application is still loaded, the application is unloaded.
     * The connection with the daemon is closed as a result of this call.
     * */
    virtual ~QSEEConnectorClient() {
      unload();
      proxy = 0;
    };

    /**
     * @brief Load the QSEE Application
     *
     * @return true on success
     *
     * @note If a connection with the daemon hasn't been established already, it
     * is re-attempted here.
     * */
    bool load() {
      int rv = 0;
      if (handle) return true;
      if (proxy == 0) getServ();
      if (proxy == 0) return false;
      do {
        sp<QSEEConnectorServiceCb> cb = new QSEEConnectorServiceCb();

        if (cb == NULL) break;

        rv = proxy->load(cb, path, name, size, handle);
        if (rv) {
          ALOGE("Error loading %s (%u bytes): %s", name.c_str(), size, strerror(rv));
          handle = 0;
          break;
        }
      } while(0);
      return (handle != 0);
    }

    /**
     * @brief Unload the QSEE application
     * */
    void unload() {
      if ((handle != 0) && (proxy != 0)) {
        proxy->unload(handle);
        handle = 0;
      }
    }

    /**
     * @brief Send a command to the QSEE Application
     *
     * @param cmd[in]     - input buffer
     * @param cmdLen[in]  - input buffer length (in bytes)
     * @param rsp[in,out] - output buffer
     * @param rspLen[in]  - output buffer length (in bytes)
     *
     * @return  0 on success,
     *          INVALID_OPERATION (-ENOSYS) if it cannot connect to the daemon,
     *          NAME_NOT_FOUND (-ENOENT) if the TA cannot be loaded,
     *          BAD_VALUE (-EINVAL) if parameters are invalid
     * */
    status_t sendCommand(void * cmd, uint32_t cmdLen, void * rsp, uint32_t rspLen) {
      if (proxy == 0) getServ();
      if (proxy == 0) return INVALID_OPERATION;
      if (handle == 0) load();
      if (handle == 0) return NAME_NOT_FOUND;
      return proxy->sendCommand(handle,cmd,cmdLen,rsp,rspLen);
    }

    /**
     * @brief Send a modified command to the QSEE Application
     *
     * @param cmd[in]     - input buffer
     * @param cmdLen[in]  - input buffer length (in bytes)
     * @param rsp[in,out] - output buffer
     * @param rspLen[in]  - output buffer length (in bytes)
     * @param info[in]    - file descriptors to be mapped in the command buffer
     *
     * @return  0 on success,
     *          INVALID_OPERATION (-ENOSYS) if it cannot connect to the daemon,
     *          NAME_NOT_FOUND (-ENOENT) if the TA cannot be loaded,
     *          BAD_VALUE (-EINVAL) if parameters are invalid
     * */
    status_t sendModifiedCommand(void * cmd, uint32_t cmdLen, void * rsp, uint32_t rspLen, struct QSEECom_ion_fd_info *info) {
      if (proxy == 0) getServ();
      if (proxy == 0) return INVALID_OPERATION;
      if (handle == 0) load();
      if (handle == 0) return NAME_NOT_FOUND;
      return proxy->sendModifiedCommand(handle,cmd,cmdLen,rsp,rspLen,info);
    };

    /**
     * @brief Get information about the QSEE Application, including secure app
     * arch type, sg_buffer_size, etc
     *
     * @param info[in,out] - pointer to the qseecom_app_info
     *
     * @return  0 on success,
     *          INVALID_OPERATION (-ENOSYS) if it cannot connect to the daemon,
     *          NAME_NOT_FOUND (-ENOENT) if the TA cannot be loaded,
     *          BAD_VALUE (-EINVAL) if parameters are invalid
     * */
    status_t getAppInfo(struct qseecom_app_info* info) {
      if (proxy == 0) getServ();
      if (proxy == 0) return INVALID_OPERATION;
      if (handle == 0) load();
      if (handle == 0) return NAME_NOT_FOUND;
      return proxy->getAppInfo(handle,info);
    }

};

}; /* namespace android */

#ifdef MY_LOG_TAG
#undef LOG_TAG
#undef ENTER
#undef EXIT
#undef EXITV
#endif

