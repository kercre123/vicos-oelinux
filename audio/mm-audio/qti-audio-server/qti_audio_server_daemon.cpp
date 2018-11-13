/* Copyright (c) 2017-2018 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#define LOG_TAG "qti_audio_server_daemon"

#include <stdlib.h>
#include <utils/RefBase.h>
#include <utils/Log.h>
#include <assert.h>
#include <cutils/properties.h>
#include "qti_audio_server.h"
#include "qti_sva_server.h"

using namespace audiohal;
using namespace svahal;

int main(int argc, char **argv) {
    sp<IServiceManager> sm = defaultServiceManager();
    assert(sm != 0);
    sm->addService(String16(QTI_AUDIO_SERVER), new qti_audio_server(), false);
    ALOGD("QTI Audio Server is alive");
    sm->addService(String16(QTI_SVA_SERVER), new qti_sva_server(), false);
    ALOGD("QTI SVA Server is alive");
    android::ProcessState::self()->startThreadPool();
    IPCThreadState::self()->joinThreadPool();
}
