/* sensor_init.c
 *
 * Copyright (c) 2013-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include "sensor_init.h"
#include "server_debug.h"
#include <cutils/properties.h>

// Intentionally bad, we must patch in at runtime!
static char *sensor_libs[] = {
  "xxx"
};

/** sensor_init_probe: probe available sensors
 *
 *  @module_ctrl: sensor ctrl pointer
 *
 *  Return: 0 for success and negative error on failure
 *
 *  1) Find sensor_init subdev and it
 *  2) Open EEPROM subdev and check whether any sensor library
 *  is present in EEPROM
 *  3) Open sensor libraries present in dumped firware location
 *  4) Check library version of EEPROM and dumped firmware
 *  5) Load latest of both
 *  6) Pass slave information, power up and probe sensors
 *  7) If probe succeeds, create video node and sensor subdev
 *  8) Repeat step 2-8 for all sensor libraries present in
 *  EEPROM
 *  9) Repeat step 6-8 for all sensor libraries present in
 *  absolute path
 **/

// This will give us 16 subversions before we release a major rev 3.0
#define DDL_BASE_VERSION 0x20

sensor_anki_t sensor_get_anki_type() {
  int fd = -1;
  uint32_t emr_data[8]; // The emr header

  fd = open("/dev/block/bootdevice/by-name/emr",O_RDONLY);
  read(fd, &emr_data, sizeof(emr_data));

  // See emr-cat.c to determine how we got the offset
  uint32_t hw_ver = emr_data[1];
       
  if (hw_ver < DDL_BASE_VERSION) {
    return OV8856;
  } else {
    return BF2253L;
  };
}

boolean sensor_init_probe(module_sensor_ctrl_t *module_ctrl)
{
  int32_t                     rc = 0, dev_fd = 0, sd_fd = 0;
  uint32_t                    i = 0;
  struct media_device_info    mdev_info;
  int32_t                     num_media_devices = 0;
  char                        dev_name[32];
  char                        subdev_name[32];
  struct sensor_init_cfg_data cfg;
  boolean                     ret = TRUE;
  uint32_t                    mask = 0;
  char value[PROPERTY_VALUE_MAX];

  property_get("persist.camera.sensor.30fps", value, "0");
  mask = (uint32_t)atoi(value);

  while (1) {
    int32_t num_entities = 1;
    snprintf(dev_name, sizeof(dev_name), "/dev/media%d", num_media_devices);
    dev_fd = open(dev_name, O_RDWR | O_NONBLOCK);
    if (dev_fd >= MAX_FD_PER_PROCESS) {
      dump_list_of_daemon_fd();
      dev_fd = -1;
      break;
    }
    if (dev_fd < 0) {
      SLOW("Done enumerating media devices");
      break;
    }
    num_media_devices++;
    rc = ioctl(dev_fd, MEDIA_IOC_DEVICE_INFO, &mdev_info);
    if (rc < 0) {
      SLOW("Done enumerating media devices");
      close(dev_fd);
      break;
    }

    if (strncmp(mdev_info.model, "msm_config", sizeof(mdev_info.model) != 0)) {
      close(dev_fd);
      continue;
    }

    while (1) {
      struct media_entity_desc entity;
      memset(&entity, 0, sizeof(entity));
      entity.id = num_entities++;
      SLOW("entity id %d", entity.id);
      rc = ioctl(dev_fd, MEDIA_IOC_ENUM_ENTITIES, &entity);
      if (rc < 0) {
        SLOW("Done enumerating media entities");
        rc = 0;
        break;
      }
      SLOW("entity name %s type %d group id %d",
        entity.name, entity.type, entity.group_id);
      if (entity.type == MEDIA_ENT_T_V4L2_SUBDEV &&
          entity.group_id == MSM_CAMERA_SUBDEV_SENSOR_INIT) {
        snprintf(subdev_name, sizeof(dev_name), "/dev/%s", entity.name);
        break;
      }
    }
    close(dev_fd);
  }

  /* Open sensor_init subdev */
  sd_fd = open(subdev_name, O_RDWR);
  if (sd_fd >= MAX_FD_PER_PROCESS) {
    dump_list_of_daemon_fd();
    sd_fd = -1;
    return FALSE;
  }
  if (sd_fd < 0) {
    SHIGH("Open sensor_init subdev failed");
    return FALSE;
  }

  // Patch in camera at runtime depending on version
  switch (sensor_get_anki_type()) {
  case BF2253L:
    sensor_libs[0] = "bf2253L";
    break;
  case OV8856:
    sensor_libs[0] = "ov8856_f8v05a";
    break;
  }
  
  /* Open sensor libraries and get init information */
  for (i = 0; i < ARRAY_SIZE(sensor_libs); i++) {
    if (mask && !(strcmp(sensor_libs[i] , "ov8858_q8v19w")))
      ret = sensor_probe(sd_fd, "ov8858_q8v19w_30");
    else
      ret = sensor_probe(sd_fd, sensor_libs[i]);
    if (ret == FALSE) {
      SERR("failed: to load %s", sensor_libs[i]);
    }
  }

  close(sd_fd);

  return TRUE;
}
