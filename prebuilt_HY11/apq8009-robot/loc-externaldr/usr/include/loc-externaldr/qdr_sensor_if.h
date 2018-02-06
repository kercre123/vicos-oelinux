/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2014, 2016 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/
/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

                  qdr sensor i/f header file

GENERAL DESCRIPTION
  This file contains the data structure, functions and variables used for
  handling sensor data and form sensor message used by Automotive 6DoF DR core

NOTE
  All the function calls (API's) described in this file are non-blocking calls,
  the function should return as soon as the intended activity is completed.

=============================================================================*/
#ifndef _QDR_SENSOR_IF_H_
#define _QDR_SENSOR_IF_H_

#include <stdio.h>
#include <stdlib.h>
#if !defined(OFFLINE)
#include <unistd.h>
#include <inttypes.h>
#include <math.h>
#include <linux/stat.h>
#include <fcntl.h>
#include <linux/types.h>
#include <errno.h>
#include <semaphore.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <loc_gps.h>
#include <platform_lib_includes.h>
#endif
#include "qdr_diag_log.h"

/** QDR CORE Sensor Data structures*/
//#define MAX_NUMBER_OF_SENSORS          9


#define A6DOF_MAX_SENSORS       9
#define A6DOF_MAX_SENSOR_SETS  11
/**
* Maximum Sensor Data sets in one Message i.e. 100ms average samples
*/
//#define MAX_CAR_SENSOR_SETS_IN_MSG     11

/**
* For 100ms maximum of 10data sets@ 100Hz to be processed.
* Maximum maximum sensor data rate supported by platform 200Hz
* we should buffer sensor data from SLIM for 200ms @ 200Hz i.e 40
*/
#define MAX_SENSOR_DATA_BUFFER         40

/**
*  Sensor Data validity Bit mask
*/
// Data definitions for type           1-8 | 14
#define A6DOF_SENSOR_VALID_TIME_TAG  (0x0001)
#define A6DOF_SENSOR_VALID_DATA0     (0x0002)  // yaw gyro   |  gyro X [0.01 deg/s]
#define A6DOF_SENSOR_VALID_DATA1     (0x0004)  // odo speed  |  gyro Y [0.01 deg/s]
#define A6DOF_SENSOR_VALID_DATA2     (0x0008)  // reverse    |  gyro Z [0.01 deg/s]
#define A6DOF_SENSOR_VALID_DATA3     (0x0010)  // RF wheel   |  acc X [0.001 m/s^2]
#define A6DOF_SENSOR_VALID_DATA4     (0x0020)  // LF wheel   |  acc Y [0.001 m/s^2]
#define A6DOF_SENSOR_VALID_DATA5     (0x0040)  // RR wheel   |  acc Z [0.001 m/s^2]
#define A6DOF_SENSOR_VALID_DATA6     (0x0080)  // LR wheel   |  odo speed [0.01 m/s]
#define A6DOF_SENSOR_VALID_DATA7     (0x0100)  // steer ang  |  reverse, 0 = forward, 1 = reverse
#define A6DOF_SENSOR_VALID_DATA8     (0x0200)  // spare      |  temperature [0.01 deg C]

/* locally define the ODO variants supported */
#define ODO_SPEED_REV  1
#define ODO_WHTICK_REV 2

/**
* @brief  The tMEMS_Status enumaration provides a list of status return codes.
*/
typedef enum {
  MEMS_STATUS_MOD_NOT_CONFIGURED = 0,
  MEMS_STATUS_MOD_DISABLED,
  MEMS_STATUS_SUCCESS,
  MEMS_STATUS_FAILURE,
  MEMS_STATUS_FEATURE_DISABLED,
  MEMS_STATUS_CONFIG_ISSUE,
  MEMS_STATUS_INVALID_PARAMS,
  MEMS_STATUS_INVALID_CONFIG,
  MEMS_STATUS_DATA_READY
}tMEMS_Status;

/**
*  @This is the structure type of each data set will be used in QDR core
*/
typedef struct
{
   uint16_t validity; // bitmap
   uint32_t time;     // time in [ms]
   int16_t     data[A6DOF_MAX_SENSORS];
} tA6DOF_SensorSet;


/**
*  @This is the structure type which will be feed to QDR core
*/

typedef struct
{
   uint8_t            type;
   uint8_t           num_sets;
   uint16_t           reserved;
   tA6DOF_SensorSet DataSet[A6DOF_MAX_SENSOR_SETS];
} tA6DOF_SensorData;

//extern tQdrSensorData2NAV a6dofCarBusData;
/**~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/**QC Sensor Data Structures*/

typedef struct
{
  uint64_t sampleTime;
  /**<   Sample time . Sensor samping time. \n
       - Type: Unsigned integer \n
       - Units: milli seconds */

  float    sample[3];
  /**<   3-axis Sample
       - Type: Floating point \n
       - Axes: Device has a right-handed coordinate system: \n
               x = Right (when holding phone in hand and looking at the display)
               y = Up
               z = Out of the display
       - Units: eSLIM_SENSOR_TYPE_ACCEL: m/s2 \n
                eSLIM_SENSOR_TYPE_GYRO: rad/s \n
                eSLIM_SENSOR_TYPE_MAGNETOMETER: uTesla

         1-axis Sample
       - Type: Floating point \n
       - Axes: only sample[0] has valid measurement
       - Units: eSLIM_SENSOR_TYPE_ACCEL_TEMP: Celsius \n
                eSLIM_SENSOR_TYPE_GYRO_TEMP: Celsius \n
                eSLIM_SENSOR_TYPE_BAROMETER: hectopascal (hPa) */
} qdrSensorSampleStructT;

typedef struct
{
  uint64_t sampleTime;
  /**<   Sample time . Sensor samping time. \n
       - Type: Unsigned integer \n
       - Units:milli seconds */

  float    speed;
  /**< speed information in m/s \n*/
  uint8_t  reverse;
  /**< forward/reverse state information \n */
} qdrVehicleDataStructT;

/**
*  @This is the structure for sensor data buffer
*/
typedef struct
{
  qdrSensorSampleStructT  sensorBuf[MAX_SENSOR_DATA_BUFFER];
  /**< sensorBuf: Sensor data sets */
  uint16_t start;
  /**<   start: start of the data in the buffer */
  uint16_t count;
  /**<   count: number of sensor data in the buffer */
} qdrSensorBufferStructT;

/**
*  @This is the structure for sensor data buffer
*/
typedef struct
{
  qdrVehicleDataStructT  sensorBuf[MAX_SENSOR_DATA_BUFFER];
  /**< sensorBuf: Sensor data sets */
  uint16_t start;
  /**<   start: start of the data in the buffer */
  uint16_t countSpd;
  uint16_t countGear;
  /**<   count: number of sensor data in the buffer */
} qdrVehicleDataBufferStructT;

/**
* @Sensor Buffer Definition For Accelerometer and Gyroscope
*/
typedef struct
{
   qdrSensorBufferStructT gyro[2];
   qdrSensorBufferStructT accel[2];
   qdrVehicleDataBufferStructT carBus[2];
   uint16_t activeSensorBuf;
}qdrDoubleBufferStructT;

extern qdrDoubleBufferStructT sensorData;


void dr_accumulateSensorData(void);

void dr_AccumProcessSensor_Gyro(qdrSensorSampleStructT *paccumSensBufGyro, qdrDoubleBufferStructT *pSensorData, uint16_t *cbvalid, uint8_t bufferToProcess);

void dr_AccumProcessSensor_Accel(qdrSensorSampleStructT *paccumSensBufAcc, qdrDoubleBufferStructT *pSensorData, uint16_t *cbvalid, uint8_t bufferToProcess);

float dr_AccumProcessCarbus_Odo(uint32_t OdoType, qdrDoubleBufferStructT *pSensorData, uint16_t *cbvalidity, uint8_t bufferToProcess);

uint16_t dr_AccumProcessCarbus_Gear(qdrDoubleBufferStructT *pSensorData, uint16_t *cbvalidity, uint8_t bufferToProcess);

void dr_pushSensorDataToCarbus(tA6DOF_SensorSet *pData, uint16_t datasetComplete);

void clear_1HzSensorDataBuffer( void );

void clear_SensorDataBuffer(void);

void dr_convert_3A_DatatoCarBusData(  tA6DOF_SensorSet *cbdata, qdrSensorSampleStructT *accel);

void dr_convert_3G_DatatoCarBusData(  tA6DOF_SensorSet *cbdata, qdrSensorSampleStructT *gyro);

typedef struct
{
 /*0-30ms index 0 30ms-50ms index 1 >50ms index 2*/
   int sensor_delay_ag[3];
   int sensor_delay_ag_cum[3];
}qdrprofileSensorDelay;

typedef struct
{
    uint32_t Sensor_type;
    uint32_t Odo_type;
}qdrSensorDataTypeT;
extern qdrSensorDataTypeT SensorTypeConfig;

#endif /*_QDR_SENSOR_IF_H_*/
