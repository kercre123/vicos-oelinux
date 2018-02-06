/**
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
=============================================================

                          EDIT HISTORY FOR FILE

when       who     what, where, why
--------   ---     ------------------------------------------
04/18/17   gs      Initial version
=============================================================*/
#ifndef TZ_APP_VENDOR_IF_H
#define TZ_APP_VENDOR_IF_H

#include "common.h"

#ifdef __cplusplus
    extern "C" {
#endif

typedef enum tz_app_vendor_if_cmd_id_t {
    TZ_APP_IF_CMD_VENDOR_EXCHANGE_TIMESTAMP = 2000,
    TZ_APP_IF_CMD_VENDOR_TEST_COMMAND = 2001,
    TZ_APP_IF_CMD_VENDOR_SIZE = 0x7FFFFFFF
} tz_app_vendor_if_cmd_id_t;

typedef enum tz_app_vendor_if_status_t {
    TZ_APP_IF_STATUS_SUCCESS = 0,
    TZ_APP_IF_STATUS_GENERAL_FAILURE = -1,
    TZ_APP_IF_STATUS_INVALID_INPUT_PARAMS = -2,
    TZ_APP_IF_STATUS_ERR_SIZE = 0x7FFFFFFF
} tz_app_vendor_if_status_t;

#pragma pack(push, tz_app_if, 1)

typedef struct tz_app_if_ion_buffer_t {
    uint64_t    buffer;
    uint32_t    size;
} tz_app_if_ion_buffer_t;

typedef struct tz_app_vendor_if_send_cmd_t {
    tz_app_vendor_if_cmd_id_t  cmd_id;
    uint64_t                   cmd_data;
} tz_app_vendor_if_send_cmd_t;

typedef struct tz_app_vendor_if_send_cmd_rsp_t {
    tz_app_vendor_if_status_t  ret;
    uint64_t                   ret_data;
} tz_app_vendor_if_send_cmd_rsp_t;

typedef struct tz_app_test_rsp_save_frame {
    tz_app_vendor_if_status_t ret;
} tz_app_test_rsp_save_frame;

typedef union {
    tz_app_test_rsp_save_frame saveFrame;
} tz_app_test_rsp_payload_t;

typedef struct tz_app_test_cmd_save_frame_start {
    uint32_t testCommandId;
    uint32_t frameIndex;
    uint32_t numberOfFrames;
    uint32_t frameInterval;
    tz_app_if_ion_buffer_t in_buffer;
} tz_app_test_cmd_save_frame_start;

typedef struct tz_app_test_cmd_save_frame_stop {
    uint32_t testCommandId;
} tz_app_test_cmd_save_frame_stop;

typedef struct tz_app_test_cmd_save_frame_ready {
    uint32_t testCommandId;
} tz_app_test_cmd_save_frame_ready;

typedef struct tz_app_test_cmd_save_frame_pause {
    uint32_t testCommandId;
} tz_app_test_cmd_save_frame_pause;

typedef struct tz_app_test_cmd_save_frame_continue {
    uint32_t testCommandId;
} tz_app_test_cmd_save_frame_continue;

typedef union {
    tz_app_test_cmd_save_frame_start saveFrameStart;
    tz_app_test_cmd_save_frame_stop saveFrameStop;
    tz_app_test_cmd_save_frame_ready saveFrameReady;
    tz_app_test_cmd_save_frame_pause saveFramePause;
    tz_app_test_cmd_save_frame_continue saveFrameContinue;
} tz_app_test_cmd_payload_t;

typedef struct tz_app_test_cmd_t {
    uint32_t cmd_id;
    tz_app_test_cmd_payload_t payload;
} tz_app_test_cmd_t;

typedef struct tz_app_test_rsp_t {
    tz_app_test_rsp_payload_t payload;
} tz_app_test_rsp_t;

#pragma pack(pop, tz_app_if)

int32_t tz_app_if_send_command(void* ta_qseecom_handle,
    tz_app_vendor_if_send_cmd_t* cmd, tz_app_vendor_if_send_cmd_rsp_t* cmd_rsp);
int32_t tz_app_if_send_test_command(void* ta_qseecom_handle,
    tz_app_test_cmd_t* cmd, tz_app_test_rsp_t* cmd_rsp, bool isModified);
int32_t tz_app_if_start_app(void** ta_qseecom_handle, const char* appname, int32_t buf_size);
int32_t tz_app_if_shutdown_app(void** ta_qseecom_handle);


#ifdef __cplusplus
}
#endif

#endif //TZ_APP_VENDOR_IF_H
