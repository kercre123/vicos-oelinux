/*===========================================================================

  Copyright (c) 2012-2015 Qualcomm Technologies, Inc.  All Rights Reserved.
  Qualcomm Technologies Proprietary and Confidential.

===========================================================================*/

#ifndef __DEVICE_ACTIONS_H__
#define __DEVICE_ACTIONS_H__

#include "thermal.h"

#define MAX_WLAN_MITIGATION_LEVEL (4)
#define MAX_BATTERY_MITIGATION_LEVEL (255)
#define MAX_CAMERA_MITIGATION_LEVEL  (10)
#define MAX_CAMCORDER_MITIGATION_LEVEL  (10)
#define MAX_MDP_MITIGATION_LEVEL  (3)
#define MAX_VENUS_MITIGATION_LEVEL (3)
#define MAX_PROFILE_LEVEL (10)
#define PERF_DEFAULT_PROFILE -1
#define MAX_CPR_TEMPERATURE_BAND (4)

#define CPU_COOLING_DEV_NAME "thermal-cpufreq-%d"
#define GPU_COOLING_DEV_NAME "thermal-devfreq-0"
#define LCD_COOLING_DEV_NAME "panel0-backlight"
#define BATTERY_COOLING_DEV_NAME "battery"
#define LCD_DEV_NAME		"lcd"
#define BATTERY_DEV_NAME	"battery"
#define GPU_DEV_NAME		"gpu"

int cpufreq_init(void);
void clusterfreq_init(void);
int gpufreq_init(void);
void hotplug_init(void);
void vdd_dig_sw_mode_init(void);
void opt_curr_req_init(void);
void vdd_rstr_init(void);
int profile_switch_init(void);
void lcd_device_init(void);
void battery_mitigation_init(void);

int report_action(enum socket_type type, const char *msg);
int shutdown_action(int delay);
int cpufreq_request(int cpu, int frequency, int cdev_id);
int clusterfreq_request(int cluster, int frequency);
int cpu_voltage_request(void *data, uint32_t voltage);
int lcd_brightness_request(int value, int cdev_id);
int battery_request(int level, int cdev_id);
int gpufreq_request(int gpu, int level, int cdev_id);
int wlan_request(int level);
int hotplug_request(int cpu, int offline);
int vdd_restriction_request(int request);
int kernel_mitigation_request(int request);
int camera_request(int request);
int camcorder_request(int request);
int vdd_dig_sw_mode_request(int mode);
int optimum_current_request(int mode);
int mdp_request(int level);
int venus_request(int level);
int get_cluster_id_num_from_index(int idx);
int profile_switch_request(int level);

int get_cluster_id(int cluster_id);
int get_num_clusters(void);
int is_cluster_sync(int cluster_id);
int get_cores_in_cluster(int cluster_id);
int cpu_voltage_current_op(void *data,uint32_t *voltage_level);
int cpufreq_current_op(int cpu, int *lvl);
int gpufreq_current_op(int cpu, int *lvl);
int clusterfreq_current_op(int cluster_id, int *lvl);
#endif  /* __DEVICE_ACTIONS_H__ */
