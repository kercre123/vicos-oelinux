/* sensor_init.h
 *
 * Copyright (c) 2013-2014 Qualcomm Technologies, Inc. All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#ifndef __SENSOR_INIT_H__
#define __SENSOR_INIT_H__

#include "sensor_common.h"

typedef enum {
  OV8856,
  BF2253L
} sensor_anki_t;

boolean sensor_init_probe(module_sensor_ctrl_t *module_ctrl);

#endif
