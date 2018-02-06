/*
 * Copyright (c) 2015,2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include "logfile.h"
#include "QvopServiceAndroid.h"

#include <stdio.h>
#include <sys/stat.h>
#include <cutils/properties.h>

static char const* const QVOP_DIRECTORY = "/data/misc/qvop";
static char const* const QVOP_LOG_FILE = "/data/misc/qvop/qvop_log.txt";


using namespace android;


#include <android/log.h>

/**
 * @brief daemon for starting the voiceprint service
 * @details VoicePrint daemon that starts the voice print service at startup.
 *
 * @param argc
 * @param argv
 *
 * @return
 */
int main(int argc, char* argv[]) {
    uint8_t logLevel = QVOP_LOG_VERBOSE;
    logfile_init(true);
    //logfile_print_i("%s: App build date: %s %s", QVOP_FN, __DATE__, __TIME__);

    char buf[PROPERTY_VALUE_MAX] = { };
    bool enable = property_get("persist.qvop", buf, "false") >= 0 &&
                strncmp(buf, "true", sizeof (buf)) == 0;
    if (!enable) {
        // voiceprint is not enabled, suspend
        logfile_print_i("%s: voiceprint not enabled, suspending", QVOP_FN);
        pause();
        return 0;
    }
    memset(buf, 0, sizeof(buf));

    //adjust log level
    //QVOP_LOG_INFO = 4
    //if (property_get("persist.qvop.tag.QVOP", buf, "3") >= 0) {
    //    logLevel = atoi(buf);
    //}
    logfile_set(logLevel);

    logfile_print_i("%s: voiceprint enabled, starting service at log level %d", QVOP_FN, logLevel);

    signal(SIGPIPE, SIG_IGN);

    // clear the log file
    FILE* fp = fopen(QVOP_LOG_FILE, "wb");
    if (fp) {
        fclose(fp);
    }


    QvopService::instantiate();
    ProcessState::self()->startThreadPool();
    IPCThreadState::self()->joinThreadPool();
    logfile_print_i("%s: exit", QVOP_FN);
    return 0;
}
