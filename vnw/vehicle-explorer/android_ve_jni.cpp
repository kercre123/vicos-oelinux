/*
 *    Copyright (c) 2016 Qualcomm Technologies, Inc.
 *    All Rights Reserved.
 *    Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#define LOG_NDEBUG 0

#include "jni.h"
#include "android_runtime/AndroidRuntime.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <errno.h>

#undef  LOG_TAG
#define LOG_TAG "VE-JNI"

extern "C" JNIEXPORT jstring JNICALL
Java_com_qualcomm_qti_vehicle_explorer_TestingActivity_nativeSecurityTest(JNIEnv *env, jobject thiz) {
    int local_errno = 0;
    jstring retval;
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock == -1) {
        local_errno = errno;
        ALOGD("android_ve_jni_test sock is %d %d", sock, local_errno);
        ALOGD("android_ve_jni_test %s", strerror(local_errno));
        ALOGD("android_ve_jni_test SUCCESS");
        char *response  = (char *)malloc(strlen("SUCCESS") + strlen(strerror(local_errno)) + 1);
        if (response == NULL) {
            return NULL;
        }
        strcpy(response, "SUCCESS ");
        strcat(response, strerror(local_errno));
        retval = env->NewStringUTF(response);
        free(response);

        return retval;
    } else {
        ALOGE("android_ve_jni_test ERROR vehicle explorer managed to directly open CAN socket");
        return env->NewStringUTF("FAILED");
    }

}
