/**
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
=============================================================

                          EDIT HISTORY FOR FILE

when       who     what, where, why
--------   ---     ------------------------------------------
04/12/17   gs      Initial version
=============================================================*/
#include <jni.h>
#include <stdlib.h>
#include <string>
#include <map>
#include <pthread.h>
#include "tz_app_vendor_if.h"
#include "ion_vendor_if.h"
#include <stdio.h>
#include <string.h>

#ifdef LOG_TAG
    #undef LOG_TAG
#endif
#define LOG_TAG "SECCAM-VENDOR-LIB-JNI"

// Size of the shared buffer for sending requests between the HLOS client & TA
#define SECURE_COMMAND_BUFFER_SIZE 1024

// Vendor command IDs
#define TZ_APP_IF_CMD_VENDOR_GET_TA_TIMESTAMP 2000
#define TZ_APP_IF_CMD_VENDOR_TEST_COMMAND 2001

static void* qsc_app_handle_ = NULL;

// Test command IDs
#define TZ_APP_IF_CMD_TEST_SAVE_FRAME_START 3000
#define TZ_APP_IF_CMD_TEST_SAVE_FRAME_STOP 3001
#define TZ_APP_IF_CMD_TEST_SAVE_FRAME_PAUSE 3002
#define TZ_APP_IF_CMD_TEST_SAVE_FRAME_CONTINUE 3003
#define TZ_APP_IF_CMD_TEST_SAVE_FRAME_READY 3004

// Test Constants
#define MAX_FRAMES_INDEX 1000    // Allows saving up to 1000 frames during a session
#define MAX_FRAMES_INDEX_NUM_OF_DIGITS 3
#define FRAME_PATH "/sdcard/"    // Path where the frames will be saved, file names will be "frame<frame_id>"

// Test Definitions
#define SECCAM_MAX_CAMERAS (4)

typedef struct seccam_save_frame_test_params {
    uint32_t curr_frame_index;
    uint32_t frames_saved;
    uint32_t initial_frame_index;
    uint32_t number_of_frames;
    uint32_t frame_interval;
    uint32_t width;
    uint32_t height;
} seccam_save_frame_test_params;

//Test Globals
static ion_if_info_t frame_ion_handle;
static bool saveFrameTestActive = false;
static seccam_save_frame_test_params saveFrameParams[SECCAM_MAX_CAMERAS];


static pthread_mutex_t access_lock_ = PTHREAD_MUTEX_INITIALIZER;

//=========================================================================
inline void lock() {
    pthread_mutex_lock(&access_lock_);
}

//=========================================================================
inline void unlock() {
    pthread_mutex_unlock(&access_lock_);
}

//=========================================================================

static bool writeBufToFile(char* file_name, uint8_t* buffer, uint32_t len) {
    FILE* file = fopen(file_name,"w+");

    if (file == NULL) {
        LOG_ERR(LOG_TAG, "::writeBufToFile - File open failed");
    }
    else {
        uint32_t write_result = fwrite(buffer, sizeof(uint8_t), len, file);
        fflush(file);
        fclose(file);

        if(write_result != len) {
            LOG_ERR(LOG_TAG, "::writeBufToFile - Write failed, returned %d", write_result);
        }
        else {
            LOG_INFO(LOG_TAG, "::writeBufToFile - Write returned %d bytes", write_result);
            return true;
        }
    }
    return false;
}

//=========================================================================

extern "C" jlong Java_com_qualcomm_qti_seccamservice_SecCamServiceVendorHandler_exchangeTimestampWithTA(
    JNIEnv* env, jobject thiz, jlong hlosTimestamp) {

    jlong ret = 0;
    LOG_INFO(LOG_TAG, "exchangeTimestampWithTA - Enter");

    lock();

    tz_app_vendor_if_send_cmd_t cmd_;
    tz_app_vendor_if_send_cmd_rsp_t cmd_rsp_;
    memset(&cmd_, 0, sizeof(tz_app_vendor_if_send_cmd_t));
    memset(&cmd_rsp_, 0, sizeof(tz_app_vendor_if_send_cmd_rsp_t));
    cmd_.cmd_id = TZ_APP_IF_CMD_VENDOR_EXCHANGE_TIMESTAMP;
    cmd_.cmd_data = hlosTimestamp;

    int retval = tz_app_if_send_command(qsc_app_handle_, &cmd_, &cmd_rsp_);
    if (retval) {
        LOG_ERR(LOG_TAG, "::tz_app_if_send_command - send command to TZ failed (%d)",
            retval);
    }

    ret = cmd_rsp_.ret_data;
    LOG_ERR(LOG_TAG, "::tz_app_if_send_command - ret_data is (%d)",
            ret);
    unlock();

    return ret;
}

//=========================================================================

extern "C" jint Java_com_qualcomm_qti_seccamservice_SecCamServiceVendorHandler_handleTestCommand(
    JNIEnv* env, jobject thiz, jbyteArray byteArray) {

    jint ret = 0;
    LOG_INFO(LOG_TAG, "handleTestCommand - Enter");

    lock();

    tz_app_test_cmd_t cmd_;
    tz_app_test_rsp_t cmd_rsp_;

    memset(&cmd_, 0, sizeof(cmd_));
    memset(&cmd_rsp_, 0, sizeof(cmd_rsp_));
    cmd_.cmd_id = TZ_APP_IF_CMD_VENDOR_TEST_COMMAND;

    int byteArrayLength = env->GetArrayLength(byteArray);

    int8_t* paramsBuffer = new int8_t[byteArrayLength];
    env->GetByteArrayRegion(byteArray, 0, byteArrayLength, paramsBuffer);

    int32_t* paramsPtr = (int32_t*)paramsBuffer;
    uint32_t testCommandId = *(paramsPtr++);

    switch (testCommandId) {
        case TZ_APP_IF_CMD_TEST_SAVE_FRAME_START: {
            uint32_t width  = *(paramsPtr++);
            uint32_t height = *(paramsPtr++);
            uint32_t cameraId = *(paramsPtr++);
            uint32_t initial_frame_index = *(paramsPtr++);
            uint32_t number_of_frames = *(paramsPtr++);
            uint32_t frame_interval = *(paramsPtr);

            if ((initial_frame_index > MAX_FRAMES_INDEX) || (number_of_frames > MAX_FRAMES_INDEX)) {
                LOG_ERR(LOG_TAG, "Invalid params received for test command");
                break;
            }

            ret = ion_if_memalloc(ION_QSECOM_HEAP_ID, &frame_ion_handle, width * height, 0, ION_IF_SIZE_4K);
            if (ret) {
                LOG_ERR(LOG_TAG, "Unable to allocate ION memory for buffer");
                break;
            }

            tz_app_if_ion_buffer_t in_buffer;
            in_buffer.buffer = frame_ion_handle.ifd_data_fd_;
            in_buffer.size = frame_ion_handle.sbuf_len_;

            cmd_.payload.saveFrameStart.testCommandId = TZ_APP_IF_CMD_TEST_SAVE_FRAME_START;
            cmd_.payload.saveFrameStart.in_buffer.buffer = in_buffer.buffer;
            cmd_.payload.saveFrameStart.in_buffer.size = in_buffer.size;

            ret = tz_app_if_send_test_command(qsc_app_handle_, &cmd_, &cmd_rsp_, true);
            if (ret) {
                LOG_ERR(LOG_TAG, "::tz_app_if_send_test_command - send command to TZ failed (%d)",
                    ret);
                break;
            }

            saveFrameParams[cameraId].width  = width;
            saveFrameParams[cameraId].height = height;
            saveFrameParams[cameraId].initial_frame_index = initial_frame_index;
            saveFrameParams[cameraId].number_of_frames = number_of_frames;
            saveFrameParams[cameraId].frame_interval = frame_interval;

            saveFrameTestActive = true;
            break;
        }
        case TZ_APP_IF_CMD_TEST_SAVE_FRAME_STOP: {
            uint64_t cameraId = *(paramsPtr);
            cmd_.payload.saveFrameStop.testCommandId = TZ_APP_IF_CMD_TEST_SAVE_FRAME_STOP;

            ret = tz_app_if_send_test_command(qsc_app_handle_, &cmd_, &cmd_rsp_, false);
            if (ret) {
                LOG_ERR(LOG_TAG, "::tz_app_if_send_test_command - send command to TZ failed (%d)",
                    ret);
                break;
            }

            ion_if_dealloc(&frame_ion_handle);
            memset(&saveFrameParams[cameraId], 0, sizeof(seccam_save_frame_test_params));
            saveFrameTestActive = false;
            break;
        }
        case TZ_APP_IF_CMD_TEST_SAVE_FRAME_READY: {
            uint64_t cameraId = *(paramsPtr);

            saveFrameParams[cameraId].curr_frame_index++;

            if (saveFrameTestActive == false) {
                break;
            }

            if ((saveFrameParams[cameraId].frames_saved >= saveFrameParams[cameraId].number_of_frames) ||
                (saveFrameParams[cameraId].curr_frame_index < saveFrameParams[cameraId].initial_frame_index) ||
                (((saveFrameParams[cameraId].curr_frame_index - 1) % saveFrameParams[cameraId].frame_interval) != 0)) {
                break;
            }

            char curr_frame_str[MAX_FRAMES_INDEX_NUM_OF_DIGITS + 1] = {0};
            snprintf(curr_frame_str, (MAX_FRAMES_INDEX_NUM_OF_DIGITS), "%d", saveFrameParams[cameraId].curr_frame_index);
            char file_name[sizeof(FRAME_PATH) + sizeof("frame") + sizeof(curr_frame_str) + 1] = {0};
            strlcat(file_name, FRAME_PATH, sizeof(file_name));
            strlcat(file_name, "frame", sizeof(file_name));
            strlcat(file_name, curr_frame_str, sizeof(file_name));

            // Notify the TA to pause copying of the frame buffer, while writing the buffer to a file
            cmd_.payload.saveFrameStop.testCommandId = TZ_APP_IF_CMD_TEST_SAVE_FRAME_PAUSE;
            ret = tz_app_if_send_test_command(qsc_app_handle_, &cmd_, &cmd_rsp_, false);
            if (ret) {
                LOG_ERR(LOG_TAG, "::tz_app_if_send_test_command - send command to TZ failed (%d)",
                    ret);
                break;
            }

            if (writeBufToFile(file_name, frame_ion_handle.ion_sbuffer_, frame_ion_handle.sbuf_len_) == false) {
                LOG_ERR(LOG_TAG, "handleTestCommand - writeBufToFile failed");
                break;
            }

            // Notify the TA to resume copying of the frame buffer
            cmd_.payload.saveFrameStart.testCommandId = TZ_APP_IF_CMD_TEST_SAVE_FRAME_CONTINUE;
            ret = tz_app_if_send_test_command(qsc_app_handle_, &cmd_, &cmd_rsp_, false);
            if (ret) {
                LOG_ERR(LOG_TAG, "::tz_app_if_send_test_command - send command to TZ failed (%d)",
                    ret);
                break;
            }

            saveFrameParams[cameraId].frames_saved++;
            break;
        }
        default:
            LOG_ERR(LOG_TAG, "handleTestCommand - Invalid test command received (%d)", testCommandId);
    }

    delete[] paramsBuffer;
    unlock();

    return ret;
}

//=========================================================================

extern "C" jint Java_com_qualcomm_qti_seccamservice_SecCamServiceVendorHandler_startTzAppSession(
    JNIEnv* env, jobject thiz, jstring j_app_name) {

    int32_t ret;
    const char *s = env->GetStringUTFChars(j_app_name, 0);
    std::string app_name(s);
    env->ReleaseStringUTFChars(j_app_name, s);

    if ((ret = tz_app_if_start_app(&qsc_app_handle_, app_name.c_str(), SECURE_COMMAND_BUFFER_SIZE))) {
        return (jint)ret;
    }

    return ret;
}

//=========================================================================

extern "C" jint Java_com_qualcomm_qti_seccamservice_SecCamServiceVendorHandler_shutdownTzAppSession(
    JNIEnv* env, jobject thiz) {
    return (tz_app_if_shutdown_app(&qsc_app_handle_));
}
