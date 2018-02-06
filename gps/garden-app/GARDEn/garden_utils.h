/* Copyright (c) 2013 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#ifndef GARDEN_UTILS_H
#define GARDEN_UTILS_H

void mutex_init();
void mutex_destroy();
void garden_cleanup();
void garden_print(const char *fmt, ...);
int timeval_difference (struct timeval *result, struct timeval *x, struct timeval *y);

#endif //GARDEN_UTILS_H
