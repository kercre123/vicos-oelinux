/* Copyright (c) 2011,2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

//

#ifndef _RUNTIME_ANDROID_RUNTIME_H
#define _RUNTIME_ANDROID_RUNTIME_H

#include <pthread.h>
#include "utils/Log_garden.h"


namespace android {

class AndroidRuntime
{
public:
    /** create a new thread that is visible from Java */
    static pthread_t createJavaThread(const char* name, void (*start)(void *),
        void* arg);
};

}

#endif
