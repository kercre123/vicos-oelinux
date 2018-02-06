/**
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
=============================================================

                          EDIT HISTORY FOR FILE

when       who     what, where, why
--------   ---     ------------------------------------------
06/14/17   gs      Add support for test commands
04/25/17   gs      Initial version
=============================================================*/
#define QSEE_CONNECTOR_SERVICE_NAME com.qualcomm.qti.qseeproxy

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include "tz_app_vendor_if.h"
#include <QSEEConnectorClient.h>

using namespace android;

#ifdef LOG_TAG
    #undef LOG_TAG
#endif
#define LOG_TAG "SECCAM-LIB-TZ-APP-VENDOR-IF"

static const char *path1_ = "/firmware/image";
static const char *path2_ = "/system/etc/firmware";

//=========================================================================

int32_t tz_app_if_send_command(
    void* ta_qseecom_handle,
    tz_app_vendor_if_send_cmd_t* cmd,
    tz_app_vendor_if_send_cmd_rsp_t* cmd_rsp) {
    int32_t ret = 0;

    QSEEConnectorClient* client = (QSEEConnectorClient*)ta_qseecom_handle;

   if (client == NULL) {
        LOG_ERR(LOG_TAG,"client NULL");
        return -EINVAL;
    }

    ret = client->sendCommand( (void*)cmd, sizeof(tz_app_vendor_if_send_cmd_t),
            (void*)cmd_rsp, sizeof(tz_app_vendor_if_send_cmd_rsp_t));

    if (ret || cmd_rsp->ret) {
        LOG_ERR(LOG_TAG, "%s:%d - Failed - ret: %d, rsp_ret: %d",
            __FUNCTION__, __LINE__,
            ret,
            cmd_rsp->ret);
    }
    else {
        LOG_DEBUG(LOG_TAG, "%s:%d - Success - ret: %d, rsp_ret: %d, ret_data: %d",
            __FUNCTION__, __LINE__,
            ret,
            cmd_rsp->ret,
            cmd_rsp->ret_data);
    }
    return ret | cmd_rsp->ret;
}

//=========================================================================

int32_t tz_app_if_send_test_command(
    void* ta_qseecom_handle,
    tz_app_test_cmd_t* cmd,
    tz_app_test_rsp_t* cmd_rsp, bool isModified) {
    int32_t ret = 0;

    QSEEConnectorClient* client = (QSEEConnectorClient*)ta_qseecom_handle;

    if (!isModified) {
        if (client == NULL) {
            LOG_ERR(LOG_TAG,"client NULL");
            return -EINVAL;
        }

        ret = client->sendCommand((void*)cmd, sizeof(tz_app_test_cmd_t),
                (void*)cmd_rsp, sizeof(tz_app_test_rsp_t));
    }

    else {
        int fd_index = 0;
        struct QSEECom_ion_fd_info ion_fd_info;
        memset((void *)&ion_fd_info, 0, sizeof(struct QSEECom_ion_fd_info));

        ion_fd_info.data[0].fd = -1;
        ion_fd_info.data[1].fd = -1;
        ion_fd_info.data[2].fd = -1;
        ion_fd_info.data[3].fd = -1;

        if (cmd->payload.saveFrameStart.in_buffer.buffer != 0) {
            ion_fd_info.data[fd_index].fd = cmd->payload.saveFrameStart.in_buffer.buffer;
            ion_fd_info.data[fd_index++].cmd_buf_offset =
                    offsetof(struct tz_app_test_cmd_t, payload) +
                    offsetof(struct tz_app_test_cmd_save_frame_start, in_buffer);
        }

        if (client == NULL) {
            LOG_ERR(LOG_TAG,"client NULL");
            return -EINVAL;
        }
        ret = client->sendModifiedCommand((void*)cmd, sizeof(tz_app_test_cmd_t),
                (void*)cmd_rsp, sizeof(tz_app_test_rsp_t), &ion_fd_info);

    }

    if (ret) {
        LOG_ERR(LOG_TAG, "%s:%d - Failed - ret: %d",
            __FUNCTION__, __LINE__,
            ret);
    }
    else {
        LOG_DEBUG(LOG_TAG, "%s:%d - Success - ret: %d",
            __FUNCTION__, __LINE__,
            ret);
    }
    return ret;
}

//=========================================================================
int32_t tz_app_if_start_app(
        void** ta_qseecom_handle, const char* appname, int32_t buf_size){
    int32_t ret = 0;

    LOG_INFO(LOG_TAG, "tz_app_if_start_app: '%s'", appname);
    QSEEConnectorClient* client = new QSEEConnectorClient(path1_, appname, buf_size);
    if (!client->load()){
        delete client;
        client = new QSEEConnectorClient(path2_, appname, buf_size);
        if (!client->load()){
            LOG_ERR(LOG_TAG, "tz_app_if_start_app: start '%s' failed", appname);
            tz_app_if_shutdown_app(ta_qseecom_handle);
            return -EFAULT;
        }
    }
    *ta_qseecom_handle = (void*)client;

    LOG_DEBUG(LOG_TAG, "tz_app_if_start_app: start '%s' %s", appname,
            (!ret)?"succeeded":"failed");
    return ret;
}

//=========================================================================
int32_t tz_app_if_shutdown_app(void** ta_qseecom_handle){
    int32_t ret = 0;

    QSEEConnectorClient* client = *((QSEEConnectorClient**)ta_qseecom_handle);
    if (client != NULL) {
        LOG_INFO(LOG_TAG, "tz_app_if_shutdown_app");
        client->unload();
        delete client;
        *ta_qseecom_handle = NULL;
    }
    return ret;
}
