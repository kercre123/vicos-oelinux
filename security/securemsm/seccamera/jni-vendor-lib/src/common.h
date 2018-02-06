/**
 * Copyright (c) 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
=============================================================

                          EDIT HISTORY FOR FILE

when       who     what, where, why
--------   ---     ------------------------------------------
04/25/17   gs      Initial version
=============================================================*/
#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>

#include <android/log.h>

#define LOG_ERR(log_tag, fmt, ...) __android_log_print(ANDROID_LOG_ERROR, log_tag, fmt, ##__VA_ARGS__)
#define LOG_WARN(log_tag, fmt, ...) __android_log_print(ANDROID_LOG_WARN, log_tag, fmt, ##__VA_ARGS__)
#define LOG_INFO(log_tag, fmt, ...) __android_log_print(ANDROID_LOG_INFO, log_tag, fmt, ##__VA_ARGS__)
#define LOG_DEBUG(log_tag, fmt, ...) __android_log_print(ANDROID_LOG_DEBUG, log_tag, fmt, ##__VA_ARGS__)

#endif //COMMON_H
