/**
 * Copyright (c) 2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * Not a Contribution.
 * Apache license notifications and license are retained
 * for attribution purposes only.
 */

/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/* @file
 @brief Sensor HAL provider implementation file.

 This file defines Sensor HAL provider object implementation.
 */

#include <inttypes.h>
#include <slim_os_log_api.h>
#include <slim_os_api.h>
#include "SlimSensorHalProvider.h"

#include <sensors.h>
#include <sensors_poll_context.h>
#include <linux/input.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

//! @addtogroup slim_SensorHalProvider
//! @{

/*----------------------------------------------------------------------------
 * Preprocessor Definitions and Constants
 * -------------------------------------------------------------------------*/
#undef SLIM_LOG_LOCAL_MODULE
//! @brief Logging module for VNW provider.
#define SLIM_LOG_LOCAL_MODULE SLIM_LOG_MOD_HAL

using namespace slim;
using namespace std;

const uint32_t SensorHalProvider::s_qThreadId = THREAD_ID_SENSOR_HAL;
SensorHalProvider *SensorHalProvider::s_pzInstance;
Mutex SensorHalProvider::s_zInstanceMutex;
const slimAvailableServiceMaskT SensorHalProvider::s_qSupportedServices =
    (uint32_t(1) << eSLIM_SERVICE_SENSOR_ACCEL)|
    (uint32_t(1) << eSLIM_SERVICE_SENSOR_GYRO)|
    (uint32_t(1) << eSLIM_SERVICE_SENSOR_MAG_UNCALIB);


void SensorHalProvider::threadMain(void *pData)
{
  SLIM_LOGD("SensorHalProvider::threadMain(0x%"PRIxPTR")",
      (uintptr_t)pData);
  SensorHalProvider *pzSensorHalProvider =
      reinterpret_cast<SensorHalProvider*>(pData);
  slim_IpcStart(s_qThreadId);
  pzSensorHalProvider->m_pzTimer = slim_TimerCreate(eTIMER_DEFAULT, s_qThreadId);

  pzSensorHalProvider->m_qServiceMask = s_qSupportedServices;
  pzSensorHalProvider->routeConfigurationChange(s_qSupportedServices);
  pzSensorHalProvider->runEventLoop();
}

/*!
 * @brief Returns provider instance.
 *
 * Method provides access to provider instance. If necessary, the instance
 * is created and initialized.
 *
 * @return Provider instance.
 * @retval 0 On error.
 */
slim_ServiceProviderInterfaceType *SensorHalProvider::getProvider()
{
  MutexLock _l(s_zInstanceMutex);
  if (0 == s_pzInstance)
  {
    s_pzInstance = new SensorHalProvider;
    if (!s_pzInstance->initialize())
    {
      SLIM_LOGE("Sensor HAL provider initialization failed;");
      delete s_pzInstance;
      s_pzInstance = 0;
      return 0;
    }
  }
  return s_pzInstance->getSlimInterface();
}

SensorHalProvider::SensorHalProvider()
    : MultiplexingProvider(SLIM_PROVIDER_SENSOR_HAL, false, 0)
    , m_zThread(), m_zMutex(), m_pzTimer(0), mBufferTxStatus(KERNEL_BUFFER_DATA_INIT),
    mAccServiceStatus(false), mGyroServiceStatus(false)
{
  memset(&m_zThread, 0, sizeof(m_zThread));
}

SensorHalProvider::~SensorHalProvider()
{
}

struct sensors_poll_context_t* device = NULL;

/**
 * @brief set buffere data state.
 *
 * @param[in] tate.
 *
 * @return void.
 */
void SensorHalProvider::setBufferState(BufferTxStatus state)
{
  mBufferTxStatus = state;
}

/**
 * @brief Handler for Buffered data.
 *
 * Function for polling buffered data from sysfs interface
 * and send it for formatting and routing through slim core.
 *
 * @param[in] pData .
 *
 * @return void.
 */
void SensorHalProvider::bufferedDataHandler(void *pData)
{
  SensorHalProvider *pzHALProvider = reinterpret_cast<SensorHalProvider*>(pData);
  slim_IpcStart(THREAD_ID_SENSOR_HAL_BUFF);
  pzHALProvider->bufferDataprocessTask();
}

/**
 * @brief Intialization for Buffer data Handling.
 *
 * Function creates thread to handle buffered data and open required
 * sysfs interfaces to read buffered data.
 *
 * @param[in] void.
 *
 * @return void.
 */
bool SensorHalProvider::memsBufferDataInit()
{

  memset(&m_zBuffThread, 0, sizeof(m_zBuffThread));
  pthread_mutex_init (&mHalBuffMutex, NULL);
  pthread_cond_init (&mHalBuffCond, NULL);
  /* BUffer data task initilation */
  m_zAccBuffSensorData.provider = eSLIM_SERVICE_PROVIDER_NATIVE;
  m_zAccBuffSensorData.timeSource = eSLIM_TIME_SOURCE_UNSPECIFIED;
  m_zAccBuffSensorData.flags = 0;
  m_zAccBuffSensorData.sensorType = eSLIM_SENSOR_TYPE_ACCEL;

  m_zGyroBuffSensorData.provider = eSLIM_SERVICE_PROVIDER_NATIVE;
  m_zGyroBuffSensorData.timeSource = eSLIM_TIME_SOURCE_UNSPECIFIED;
  m_zGyroBuffSensorData.flags = 0;
  m_zGyroBuffSensorData.sensorType = eSLIM_SENSOR_TYPE_GYRO;

  /* Create Thread and process buffered data */
  if (!slim_ThreadCreate(&m_zBuffThread, bufferedDataHandler, this, "HALBuffData"))
  {
    SLIM_LOGE("memsBufferDataInit: Slim Thread create fail");
    return false;
  }
  return true;
}

void SensorHalProvider::memsBufferDataDeInit()
{
  setBufferState(COMPLETED_BUFFER_DATA);
  CLOSE_FILE_HANDLE(mfdBuffAccel);
  CLOSE_FILE_HANDLE(mfdBuffGyro);
  CLOSE_FILE_HANDLE(mfdBuffAccelE);
  CLOSE_FILE_HANDLE(mfdBuffGyroE);

  pthread_mutex_destroy (&mHalBuffMutex);
  pthread_cond_destroy (&mHalBuffCond);
}

/**
 * @brief baching and for formatting of buffered data.
 *
 * Function for converting buffered data to SLIM service sepecific format
 * Read queue until next EV_SYN , this will fill only newly received data
 * if some fields of sample are not received will maintain older data
 *
 * @param[in/out] zEventBuffer - input previous data, output new data,
 * if some fields are not received will hold previous data.
 * @param[in] fd - pointer to file descritor
 * @param[in] eSensorType - sensorType
 *
 * @return void.
 */
bool SensorHalProvider::getBufferedSample
(
  slimServiceEnumT eSensorType,
  FILE* fd,
  slimSensorSampleStructT *zEventBuffer
)
{
  struct input_event   ev[2];
  static int accCounter = 0, gyroCounter = 0;
  static int nanoSeconds, seca, secb;
  float scaleFactor;
  bool retVal = false;

  /* Get the scale factor based on sensor type */
  if (eSensorType == eSLIM_SERVICE_SENSOR_ACCEL)
    scaleFactor = BMI160_ACC_RESL * CONVERT_ACC;
  else
    scaleFactor = CONVERT_GYRO;

  /* Read queue until next EV_SYN */
  while(1)
  {
    if(0 == fread(ev, 1, sizeof(struct input_event), fd))
    {
      SLIM_LOGD("acc data read fail/no more data");
      retVal = false;
      break;
    }
    else
    {
      /* return a sample if read item is EV_SYN */
      if (ev[0].type == EV_SYN)
      {
        SLIM_LOGD("Sensor Type %d x=%f,y=%f,z=%f,timestamp=%d, sec= %d count:%d\n",
                 eSensorType, zEventBuffer->sample[0], zEventBuffer->sample[1],
                 zEventBuffer->sample[2], zEventBuffer->sampleTimeOffset,
                 eSensorType == eSLIM_SERVICE_SENSOR_ACCEL ? seca : secb,
                 eSensorType == eSLIM_SERVICE_SENSOR_ACCEL ? accCounter++ : gyroCounter++);
        if (ev[0].value != 0xFFFFFFFF)
        {
          retVal = true;
          break;
        } else
        {
          retVal = false;
          break;
        }
      } else {
        if (ev[0].code == ABS_X) {
          zEventBuffer->sample[0] = ev[0].value * scaleFactor;
        } else if (ev[0].code == ABS_Y) {
          zEventBuffer->sample[1] = ev[0].value * scaleFactor;
        } else if (ev[0].code == ABS_Z) {
          zEventBuffer->sample[2] = ev[0].value * scaleFactor;
        } else if (ev[0].code == ABS_RX) {
          if (eSensorType == eSLIM_SERVICE_SENSOR_ACCEL)
            seca = ev[0].value;
          else
            secb = ev[0].value;
        } else if (ev[0].code == ABS_RY) {
          /* update Nano seconds value from previous stored value */
          if (eSensorType == eSLIM_SERVICE_SENSOR_ACCEL)
            zEventBuffer->sampleTimeOffset = seca * 1000 + (ev[0].value) / 1000000;
          else
            zEventBuffer->sampleTimeOffset = secb * 1000 + (ev[0].value) / 1000000;
        }
      }
    }
  }
  return retVal;
}

/**
 * Scans the input device and stores the fhandle
 * for reading the buffer data from kernel
 *
 * return val: SUCCESS -  0
 *             FAIL - -1
 */
int SensorHalProvider::snsBufDevOpen(char *s_name, FILE** fhandle)
{
  struct dirent **namelist;
  int nNodes = 0, indx = 0;
  int fd = -1;
  FILE* handle = NULL;
  char buf[50];
  int read_bytes = -1;
  char sns_name[MAX_LENGTH]="";
  char data_path[MAX_LENGTH]="";
  char dev_path[MAX_LENGTH]="";

  strlcpy(sns_name, s_name, MAX_LENGTH);
  SLIM_LOGD("name of the sensor %s", sns_name);

  strlcpy(data_path, "/dev/input/", MAX_LENGTH);

  nNodes = scandir(PATH, &namelist, 0, alphasort);
  if (!nNodes) {
    SLIM_LOGE("no entries in /sys/class/input/");
    return -1;
  }

  strlcpy(dev_path, SNS_PATH, MAX_LENGTH);
  strlcat(dev_path, s_name, MAX_LENGTH);

  SLIM_LOGV("device name is :%s", dev_path);

  for (indx = 0; indx < nNodes; indx++) {
    if (!strncmp("event", namelist[indx]->d_name, strlen("event"))) {
      char temp_path[MAX_LENGTH] = PATH;
      strlcat(temp_path, namelist[indx]->d_name, MAX_LENGTH);
      strlcat(temp_path, "/device/name", MAX_LENGTH);
      fd = open(temp_path, 0);
      if (fd < 0) {
        SLIM_LOGE("failed to open %s\n", temp_path);
        break;
      }
      read_bytes = read(fd, buf, sizeof(buf)-1);
      if (read_bytes <= 0) {
        SLIM_LOGE("read failed \n");
        break;
      }
      buf[read_bytes] = '\0';
      if (!strncmp(sns_name, buf, strlen(sns_name))) {
        strlcat(data_path, namelist[indx]->d_name, sizeof(data_path));
        SLIM_LOGE("data path %s\n", data_path);
        break;
      }
    }
  }

  if (nNodes == indx) {
    SLIM_LOGE("no input device with %s found", s_name);
    *fhandle = NULL;
    return -1;
  }

  for(indx = 0; indx < nNodes; indx++)
    free(namelist[indx]);

  if ((handle = fopen(data_path, "r")) == 0) {
    SLIM_LOGE("failed to open %s errno %d, (%s)", data_path, errno, strerror(errno));
    return -1;
  }

  *fhandle = handle;

  return 0;
}

/**
 * @brief Buffered data processing task.
 *
 * Function for processing buffered data from sysfs interface
 * and send it for formatting and routing through slim core.
 *
 * @return void.
 */
void SensorHalProvider::bufferDataprocessTask()
{
  bool accelBuffDataTxProgress = false;
  bool gyroBuffDataTxProgress = false;
  // Init sysFs files for both ACCEL & GYRO buffered data
  mfdBuffAccel  = NULL;
  mfdBuffGyro   = NULL;
  mfdBuffAccelE = NULL;
  mfdBuffGyroE  = NULL;
  int buffSamplesReceived[2] = {0,0};

  // Buffer for reading raw events from the queue
  // The events are sorted by timestamp by event queue facility
  slimSensorSampleStructT samples[2];
  ssize_t accNumRead = 0, gyroNumRead = 0;
  if ((mfdBuffAccelE = fopen(ENABLE_ACCEL_BUFFER_DATA, "w")) == 0) {
    SLIM_LOGE("failed to open %s errno %d, (%s)",
               ENABLE_ACCEL_BUFFER_DATA, errno, strerror(errno));
    goto bail;
  }

  if ((mfdBuffGyroE = fopen(ENABLE_GYRO_BUFFER_DATA, "w")) == 0) {
    SLIM_LOGE("failed to open %s errno %d, (%s)",
              ENABLE_GYRO_BUFFER_DATA, errno, strerror(errno));
    goto bail;
  }

  /* Open Accel Bufferd Sensor input device */
  if (snsBufDevOpen(ACCEL_BUFF_FILE, &mfdBuffAccel) < 0) {
    SLIM_LOGE("failed to open %s errno %d, (%s)", ACCEL_BUFF_FILE, errno, strerror(errno));
    goto bail;
  }

  /* Open Gyro Bufferd Sensor input device */
  if (snsBufDevOpen(GYRO_BUFF_FILE, &mfdBuffGyro) < 0) {
    SLIM_LOGE("failed to open %s errno %d, (%s)", GYRO_BUFF_FILE, errno, strerror(errno));
    goto bail;
  }

  SLIM_LOGD("Wait for service enable request from clients");

  pthread_mutex_lock (&mHalBuffMutex);
  pthread_cond_wait (&mHalBuffCond, &mHalBuffMutex);
  pthread_mutex_unlock (&mHalBuffMutex);

  accelBuffDataTxProgress = true;
  gyroBuffDataTxProgress = true;

  setBufferState(KERNEL_BUFFER_DATA_IN_PROGRESS);

  if (fwrite("1", 1, 1, mfdBuffAccelE) != 1){
    SLIM_LOGE("failed to write data into %s, errno = %d (%s)",
      ENABLE_ACCEL_BUFFER_DATA, errno, strerror(errno));
    goto bail;
  } else {
    fflush(mfdBuffAccelE);
  }

  if (fwrite("1", 1, 1, mfdBuffGyroE) != 1){
    SLIM_LOGE("failed to write data into %s, errno = %d (%s)",
      ENABLE_GYRO_BUFFER_DATA, errno, strerror(errno));
    goto bail;
  } else {
    fflush(mfdBuffGyroE);
  }

  SLIM_LOGD("start reading kernel buffered data");
  // Read and process all buffered data from sysfs
  // Before reading and processing buffered data make sure client is available to receive data
  // interleave the data while transferring to client aplication as per batching size
  while(accelBuffDataTxProgress || gyroBuffDataTxProgress)
  {
    /* Fill the accel buffered data from kernel bufer */
    if (accelBuffDataTxProgress &&
        getBufferedSample(eSLIM_SERVICE_SENSOR_ACCEL, mfdBuffAccel, &samples[0])) {

      accNumRead += 1;
      buffSamplesReceived[0]++;
      if (accNumRead == 1) {
         m_zAccBuffSensorData.timeBase = samples[0].sampleTimeOffset;
         m_zAccBuffSensorData.samples[0] = samples[0];
         m_zAccBuffSensorData.samples[0].sampleTimeOffset = 0;
      } else if (accNumRead <= 50){
        m_zAccBuffSensorData.samples[accNumRead - 1] = samples[0];
        m_zAccBuffSensorData.samples[accNumRead - 1].sampleTimeOffset =
          slim_TimeToClockTicks(samples[0].sampleTimeOffset - m_zAccBuffSensorData.timeBase);
      }
    } else {
      accelBuffDataTxProgress = false;
      SLIM_LOGD("accelBuffDataTxProgress false!, total accel samples read %d", accNumRead);
    }
    /* Fill the gyro buffered data into from kernel buffer */
    if (gyroBuffDataTxProgress &&
        getBufferedSample(eSLIM_SERVICE_SENSOR_GYRO, mfdBuffGyro, &samples[1])) {

      gyroNumRead += 1;
      buffSamplesReceived[1]++;
      if (gyroNumRead == 1) {
        m_zGyroBuffSensorData.timeBase = samples[1].sampleTimeOffset;
        m_zGyroBuffSensorData.samples[0] = samples[1];
        m_zGyroBuffSensorData.samples[0].sampleTimeOffset = 0;
      } else if (gyroNumRead <= 50){
        m_zGyroBuffSensorData.samples[gyroNumRead - 1] = samples[1];
        m_zGyroBuffSensorData.samples[gyroNumRead - 1].sampleTimeOffset =
          slim_TimeToClockTicks(samples[1].sampleTimeOffset - m_zGyroBuffSensorData.timeBase);
      }
    } else {
      gyroBuffDataTxProgress = false;
      SLIM_LOGD("gyroBuffDataTxProgress false!, total gyro samples read %d", gyroNumRead);
    }

    /* once count become 50, send buffer data to clients*/
    if (accelBuffDataTxProgress && (accNumRead == 50)) {
      m_zAccBuffSensorData.samples_len = 50;
      m_pzSensorHalProvider->routeIndication(eSLIM_SERVICE_SENSOR_ACCEL, m_zAccBuffSensorData);
      accNumRead = 0;
    }
    if (gyroBuffDataTxProgress && (gyroNumRead == 50)) {
      m_zGyroBuffSensorData.samples_len = 50;
      m_pzSensorHalProvider->routeIndication(eSLIM_SERVICE_SENSOR_GYRO, m_zGyroBuffSensorData);
      gyroNumRead = 0;
    }
    /* sleep for 2 ms every batch of data sent to application */
    if (((accelBuffDataTxProgress) && (0 == (buffSamplesReceived[0] % 50))) ||
        ((gyroBuffDataTxProgress) && (0 == (buffSamplesReceived[1] % 50)))) {
      usleep(TIME_BW_BUFFERED_DATA_BATCHES);
    }
  }

  /* send remaining samples from buffered data*/
  if (accNumRead > 0) {
    m_zAccBuffSensorData.samples_len = accNumRead;
    m_pzSensorHalProvider->routeIndication(eSLIM_SERVICE_SENSOR_ACCEL, m_zAccBuffSensorData);
    accNumRead = 0;
  }
  if (gyroNumRead > 0) {
    m_zGyroBuffSensorData.samples_len = gyroNumRead;
    m_pzSensorHalProvider->routeIndication(eSLIM_SERVICE_SENSOR_GYRO, m_zGyroBuffSensorData);
    gyroNumRead = 0;
  }

  /* send isamples from seconday buffer data*/
  setBufferState(SLIM_BUFFER_DATA_IN_PROGRESS);
  slimSensorSampleStructT zSensorEvent;
  //Send secondary buffer data
  while (!mSecAccBufferList.empty() || !mSecGyroBufferList.empty())
  {
    //process acc data samples from secondary buffer
    if (!mSecAccBufferList.empty()){
      zSensorEvent = mSecAccBufferList.front();
      mSecAccBufferList.pop_front();
      accNumRead += 1;

      if (zSensorEvent.sampleTimeOffset < samples[0].sampleTimeOffset ) {
        SLIM_LOGD("ACCEL Igone this duplicate sample, time stamp %d", zSensorEvent.sampleTimeOffset);
        accNumRead -= 1;
      } else if (accNumRead == 1) {
         m_zAccBuffSensorData.timeBase = zSensorEvent.sampleTimeOffset;
         m_zAccBuffSensorData.samples[0] = zSensorEvent;
         m_zAccBuffSensorData.samples[0].sampleTimeOffset = 0;

      } else {
        m_zAccBuffSensorData.samples[accNumRead - 1] = zSensorEvent;
        m_zAccBuffSensorData.samples[accNumRead - 1].sampleTimeOffset =
          slim_TimeToClockTicks(zSensorEvent.sampleTimeOffset - m_zAccBuffSensorData.timeBase);
      }
    }
    //process gyro data samples from secondary buffer
    if (!mSecGyroBufferList.empty()) {
      zSensorEvent = mSecGyroBufferList.front();
      mSecGyroBufferList.pop_front();
      gyroNumRead += 1;

      if (zSensorEvent.sampleTimeOffset < samples[1].sampleTimeOffset ) {
        SLIM_LOGD("GYRO Igone this duplicate sample, time stamp %d", zSensorEvent.sampleTimeOffset);
        gyroNumRead -= 1;
      } else if (gyroNumRead == 1) {
        m_zGyroBuffSensorData.timeBase = zSensorEvent.sampleTimeOffset;
        m_zGyroBuffSensorData.samples[0] = zSensorEvent;
        m_zGyroBuffSensorData.samples[0].sampleTimeOffset = 0;

      } else {
        m_zGyroBuffSensorData.samples[gyroNumRead - 1] = zSensorEvent;
        m_zGyroBuffSensorData.samples[gyroNumRead - 1].sampleTimeOffset =
          slim_TimeToClockTicks(zSensorEvent.sampleTimeOffset - m_zGyroBuffSensorData.timeBase);
      }
    }
    /* once count becomes 50, send secondary buffer data to clients*/
    if (accNumRead == 50) {
      m_zAccBuffSensorData.samples_len = accNumRead;
      m_pzSensorHalProvider->routeIndication(eSLIM_SERVICE_SENSOR_ACCEL, m_zAccBuffSensorData);
      accNumRead = 0;
    }
    if (gyroNumRead == 50) {
      m_zGyroBuffSensorData.samples_len = gyroNumRead;
      m_pzSensorHalProvider->routeIndication(eSLIM_SERVICE_SENSOR_GYRO, m_zGyroBuffSensorData);
      gyroNumRead = 0;
    }
  }
  /* send remainig samples from secondary buffer */
  if (accNumRead > 0) {
    m_zAccBuffSensorData.samples_len = accNumRead;
    m_pzSensorHalProvider->routeIndication(eSLIM_SERVICE_SENSOR_ACCEL, m_zAccBuffSensorData);
  }
  if (gyroNumRead > 0) {
    m_zGyroBuffSensorData.samples_len = gyroNumRead;
    m_pzSensorHalProvider->routeIndication(eSLIM_SERVICE_SENSOR_GYRO, m_zGyroBuffSensorData);
  }

  setBufferState(COMPLETED_BUFFER_DATA);
  /*Trigger regular sensor data */
  doUpdateSensorStatus(true, mAccReportRateHz, mAccSampleCount, eSLIM_SERVICE_SENSOR_ACCEL);
  doUpdateSensorStatus(true, mGyroReportRateHz, mGyroSampleCount, eSLIM_SERVICE_SENSOR_GYRO);

bail:
  memsBufferDataDeInit();
  SLIM_LOGD("Exiting bufferDataprocessTask.. ");
}

bool SensorHalProvider::initialize()
{

  int err;

  device = new sensors_poll_context_t();
  if (NULL == device) {
      SLIM_LOGE("sensors_poll_context_t() failed \n");
      return false;
  }
  struct sensor_t const* list;
  int count = device->get_sensors_list(&list);
  SLIM_LOGD("%d sensors found:\n", count);
  for (int i=0 ; i<count ; i++) {
      SLIM_LOGD("%s\n"
              "\tvendor: %s\n"
              "\tversion: %d\n"
              "\thandle: %d\n"
              "\ttype: %d\n"
              "\tmaxRange: %f\n"
              "\tresolution: %f\n"
              "\tpower: %f mA\n",
              list[i].name,
              list[i].vendor,
              list[i].version,
              list[i].handle,
              list[i].type,
              list[i].maxRange,
              list[i].resolution,
              list[i].power);
  }

  if (!slim_ThreadCreate(&m_zThread, threadMain, this, "SensorHalPoll"))
  {
    return false;
  }

  slim_TaskReadyWait(THREAD_ID_SENSOR_HAL);
  // call Initialize buffere data
  if (memsBufferDataInit()) {
    SLIM_LOGD("waiting for slim_TaskReadyWait THREAD_ID_SENSOR_HAL_BUFF \n");
  } else {
     memsBufferDataDeInit();
  }

  SLIM_LOGD("Exiting ..\n");
  return true;
}

char const* SensorHalProvider::getSensorName(int type) {
    switch(type) {
        case SENSOR_TYPE_ACCELEROMETER:
            return "Acc";
        case SENSOR_TYPE_MAGNETIC_FIELD:
            return "Mag";
        case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
            return "MaU";
        case SENSOR_TYPE_ORIENTATION:
            return "Ori";
        case SENSOR_TYPE_GYROSCOPE:
            return "Gyr";
        case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
            return "GyU";
        case SENSOR_TYPE_LIGHT:
            return "Lux";
        case SENSOR_TYPE_PRESSURE:
            return "Bar";
        case SENSOR_TYPE_TEMPERATURE:
            return "Tmp";
        case SENSOR_TYPE_PROXIMITY:
            return "Prx";
        case SENSOR_TYPE_GRAVITY:
            return "Grv";
        case SENSOR_TYPE_LINEAR_ACCELERATION:
            return "Lac";
        case SENSOR_TYPE_ROTATION_VECTOR:
            return "Rot";
        case SENSOR_TYPE_RELATIVE_HUMIDITY:
            return "Hum";
        case SENSOR_TYPE_AMBIENT_TEMPERATURE:
            return "Tam";
    }
    return "ukn";
}

slimSensorTypeEnumT SensorHalProvider::SensorTypeToSlimSensorType(int sensorType)
{
    switch(sensorType)
    {
        case SENSOR_TYPE_ACCELEROMETER:
          return eSLIM_SENSOR_TYPE_ACCEL;

        case SENSOR_TYPE_MAGNETIC_FIELD:
          return eSLIM_SENSOR_TYPE_MAGNETOMETER;

        case SENSOR_TYPE_GYROSCOPE:
        case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
          return eSLIM_SENSOR_TYPE_GYRO;

        case SENSOR_TYPE_PRESSURE:
          return eSLIM_SENSOR_TYPE_BAROMETER;

        case SENSOR_TYPE_GRAVITY:
        case SENSOR_TYPE_LINEAR_ACCELERATION:
        case SENSOR_TYPE_ROTATION_VECTOR:
        case SENSOR_TYPE_LIGHT:
        case SENSOR_TYPE_ORIENTATION:
        case SENSOR_TYPE_TEMPERATURE:
        case SENSOR_TYPE_PROXIMITY:
        case SENSOR_TYPE_RELATIVE_HUMIDITY:
        case SENSOR_TYPE_AMBIENT_TEMPERATURE:
        default:
            return eSLIM_SENSOR_TYPE_MAX;
    }
}

slimServiceEnumT SensorHalProvider::SlimSensorTypeToService(slimSensorTypeEnumT slimSensorType)
{
    switch(slimSensorType)
    {
        case eSLIM_SENSOR_TYPE_ACCEL:
            return eSLIM_SERVICE_SENSOR_ACCEL;
        case eSLIM_SENSOR_TYPE_GYRO:
            return eSLIM_SERVICE_SENSOR_GYRO;
        case eSLIM_SENSOR_TYPE_ACCEL_TEMP:
            return eSLIM_SERVICE_SENSOR_ACCEL_TEMP;
        case eSLIM_SENSOR_TYPE_GYRO_TEMP:
            return eSLIM_SERVICE_SENSOR_GYRO_TEMP;
        case eSLIM_SENSOR_TYPE_MAGNETOMETER:
              //return eSLIM_SERVICE_SENSOR_MAG_CALIB;
              return eSLIM_SERVICE_SENSOR_MAG_UNCALIB;
              //return eSLIM_SERVICE_MAG_FIELD_DATA;
        case eSLIM_SENSOR_TYPE_BAROMETER:
            return eSLIM_SERVICE_SENSOR_BARO;
        case eSLIM_SENSOR_TYPE_MAX:
        default:
              return eSLIM_SERVICE_NONE;
    }
}

void SensorHalProvider::runEventLoop()
{

   SLIM_LOGD("Starting event loop");
   m_pzSensorHalProvider = this;
   slim_TaskReadyAck();
   m_zAccSensorData.samples_len = 0;
   m_zGyroSensorData.samples_len = 0;
   uint64_t tAccelDispatchTimestamp = 0ull;
   uint64_t tGyroDispatchTimestamp = 0ull;
   m_bIsAccelPPSTimeNotReached = true;
   m_bIsGyroPPSTimeNotReached = true;

   while (1)
   {
      slim_IpcMsgT* pz_Msg = NULL;
      static const size_t numEvents = 16;
      sensors_event_t buffer[numEvents];

      SLIM_LOGD("polling device(%p)", device);
      int n = device->pollEvents(buffer, numEvents);
      if (n < 0) {
         SLIM_LOGE("poll() failed (%s)\n", n);
         break;
      }

      SLIM_LOGD("read %d events:\n", n);
      for (int i=0 ; i<n ; i++) {
         const sensors_event_t& data = buffer[i];

         if (data.version != sizeof(sensors_event_t)) {
            SLIM_LOGE("incorrect event version (version=%d, expected=%d",
               data.version, sizeof(sensors_event_t));
            break;
         }

        switch(data.type) {
            case SENSOR_TYPE_ACCELEROMETER:
               /* If buffer data is in progress, keep these samples in secondary buffer */
                if (mBufferTxStatus == KERNEL_BUFFER_DATA_IN_PROGRESS ||
                       mBufferTxStatus == SLIM_BUFFER_DATA_IN_PROGRESS)
                {
                  SLIM_LOGD("sensor=%s, time=%lld, value=<%f,%f,%f>\n",
                          getSensorName(data.type),
                          (data.timestamp / 1000000),
                          data.data[0],
                          data.data[1],
                          data.data[2]);
                  m_zSensorData.samples[0].sample[0] = data.data[0];
                  m_zSensorData.samples[0].sample[1] = data.data[1];
                  m_zSensorData.samples[0].sample[2] = data.data[2];
                  m_zSensorData.samples[0].sampleTimeOffset = 0;
                  m_zSensorData.samples_len = 1;
                  m_zSensorData.provider = eSLIM_SERVICE_PROVIDER_NATIVE;
                  m_zSensorData.timeBase = data.timestamp / 1000000; //Convert it to msec;
                  m_zSensorData.timeSource = eSLIM_TIME_SOURCE_UNSPECIFIED;
                  m_zSensorData.flags = 0;
                  m_zSensorData.sensorType = SensorTypeToSlimSensorType(data.type);
                  SLIM_LOGD("Kernel/SLIM buffer data in progress,"
                       " store this ACC sample in secondary buffer \n");
                  m_zSensorData.samples[0].sampleTimeOffset = m_zSensorData.timeBase;
                  mSecAccBufferList.push_back(m_zSensorData.samples[0]);
            }
            else
            {
              if (!mBufferTxStatus)
              {
                 uint64_t tCurrentTimestamp = (data.timestamp / 1000000);
                 SLIM_LOGD("tCurrentTimestamp = %" PRIu64 " ms. sensor=%s, time=%lld, value=<%f,%f,%f>\n",
                     tCurrentTimestamp,
                     getSensorName(data.type),
                     (data.timestamp / 1000000),
                     data.data[0],
                     data.data[1],
                     data.data[2]);
                 slimSensorDataStructT zSensorData;
                 zSensorData.samples[0].sample[0] = data.data[0];
                 zSensorData.samples[0].sample[1] = data.data[1];
                 zSensorData.samples[0].sample[2] = data.data[2];
                 zSensorData.sensorType = SensorTypeToSlimSensorType(data.type);
                 zSensorData.timeBase = tCurrentTimestamp;
                 routeIndicationInBatch(eSLIM_SERVICE_SENSOR_ACCEL, m_zAccSensorData,
                                        zSensorData,
                                        tAccelDispatchTimestamp,
                                        m_bIsAccelPPSTimeNotReached,
                                        mAcc100HzBatchCount);
              }
            }
            break;
         case SENSOR_TYPE_GYROSCOPE:
         case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
            /* If buffer data is in progress, keep these samples in secondary buffer */
            if (mBufferTxStatus == KERNEL_BUFFER_DATA_IN_PROGRESS ||
                     mBufferTxStatus == SLIM_BUFFER_DATA_IN_PROGRESS)
            {
              SLIM_LOGD("sensor=%s, time=%lld, value=<%f,%f,%f>\n",
                     getSensorName(data.type),
                     (data.timestamp / 1000000),
                     data.data[0],
                     data.data[1],
                     data.data[2]);
              m_zSensorData.samples[0].sample[0] = data.data[0];
              m_zSensorData.samples[0].sample[1] = data.data[1];
              m_zSensorData.samples[0].sample[2] = data.data[2];
              m_zSensorData.samples[0].sampleTimeOffset = 0;
              m_zSensorData.samples_len = 1;
              m_zSensorData.provider = eSLIM_SERVICE_PROVIDER_NATIVE;
              m_zSensorData.timeBase = data.timestamp / 1000000; //Convert it to msec;
              m_zSensorData.timeSource = eSLIM_TIME_SOURCE_UNSPECIFIED;
              m_zSensorData.flags = 0;
              m_zSensorData.sensorType = SensorTypeToSlimSensorType(data.type);
              SLIM_LOGD("Kernel/SLIM buffer data in progress,"
                 " store this GYRO sample in secondary buffer \n");
              m_zSensorData.samples[0].sampleTimeOffset = m_zSensorData.timeBase;
              mSecGyroBufferList.push_back(m_zSensorData.samples[0]);
            }
            else
            {
              if (!mBufferTxStatus)
              {
                 uint64_t tCurrentTimestamp = (data.timestamp / 1000000);
                 SLIM_LOGD("tCurrentTimestamp = %" PRIu64 " ms. sensor=%s, time=%lld, value=<%f,%f,%f>\n",
                     tCurrentTimestamp,
                     getSensorName(data.type),
                     (data.timestamp / 1000000),
                     data.data[0],
                     data.data[1],
                     data.data[2]);
                 slimSensorDataStructT zSensorData;
                 zSensorData.samples[0].sample[0] = data.data[0];
                 zSensorData.samples[0].sample[1] = data.data[1];
                 zSensorData.samples[0].sample[2] = data.data[2];
                 zSensorData.sensorType = SensorTypeToSlimSensorType(data.type);
                 zSensorData.timeBase = tCurrentTimestamp;
                 routeIndicationInBatch(eSLIM_SERVICE_SENSOR_GYRO, m_zGyroSensorData,
                                        zSensorData,
                                        tGyroDispatchTimestamp,
                                        m_bIsGyroPPSTimeNotReached,
                                        mGyro100HzBatchCount);
              }
            }
            break;
         case SENSOR_TYPE_MAGNETIC_FIELD:
         case SENSOR_TYPE_ORIENTATION:
         case SENSOR_TYPE_GRAVITY:
         case SENSOR_TYPE_LINEAR_ACCELERATION:
         case SENSOR_TYPE_ROTATION_VECTOR:
         case SENSOR_TYPE_LIGHT:
         case SENSOR_TYPE_PRESSURE:
         case SENSOR_TYPE_TEMPERATURE:
         case SENSOR_TYPE_PROXIMITY:
         case SENSOR_TYPE_RELATIVE_HUMIDITY:
         case SENSOR_TYPE_AMBIENT_TEMPERATURE:
            SLIM_LOGD("sensor=%s, time=%lld, value=%f\n",
               getSensorName(data.type),
               data.timestamp,
               data.data[0]);
            break;

         default:
            SLIM_LOGD("sensor=%d, time=%lld, value=<%f,%f,%f, ...>\n",
               data.type,
               data.timestamp,
               data.data[0],
               data.data[1],
               data.data[2]);
            break;
         }

         /* Other than  ACCELEROMETER and GYROSCOPE will be handled here */
         if(data.type != SENSOR_TYPE_ACCELEROMETER && data.type != SENSOR_TYPE_GYROSCOPE &&
                  data.type != SENSOR_TYPE_GYROSCOPE_UNCALIBRATED)
         {
            slimServiceEnumT service = SlimSensorTypeToService(m_zSensorData.sensorType);
            if(eSLIM_SERVICE_NONE != service)
            {
               m_pzSensorHalProvider->routeIndication(service, m_zSensorData);
            }
         }
      }
   }

}

/**
 @brief Initiates time offset request.

 Function for making the time request. Successful response enable SLIM to
 calculate the offset between modem time and sensor time.

 @param[in] lTxnId Service transaction id.
 @return eSLIM_SUCCESS if time request is made successfully.
 */
slimErrorEnumT SensorHalProvider::getTimeUpdate(int32_t lTxnId)
{
/*  return MultiplexingProvider::getTimeUpdate(lTxnId); */
  SLIM_LOG_ENTRY();

  SLIM_LOGD("Requesting time update: txn=%"PRId32, lTxnId);
  slimErrorEnumT retVal = eSLIM_ERROR_INTERNAL;

#if 0

  if ( 0 != m_tLastBusControllerSyncTimestamp &&
       0 != m_tLastBusControllerSyncTime)
  {
    uint64_t t_Time = m_tLastBusControllerSyncTimestamp;
    t_Time += (slim_TimeTickGetMilliseconds() - m_tLastBusControllerSyncTime);

    routeTimeResponse(lTxnId, eSLIM_SUCCESS, t_Time, t_Time);
    SLIM_LOGD("Time update reported:%"PRIu64, t_Time);
    retVal = eSLIM_SUCCESS;
  }

  SLIM_LOGD("retVal= %d", retVal);
#endif
  return retVal;
}


/**
@brief Method for enabling or disabling sensor service.

Generic function to start/stop a sensor based on provided sampling rate,
batching rate, mounted state, and sensor information using Sensor1 Buffering
API.

@todo The sampling frequency is a mere suggestion to the sensor1 daemon.
Sensor 1 will stream at the requested minimum sampling frequency requested
by all AP clients combined. So we do see cases where buffering API does not
help and we get single sensor data for every indication. In that case should
SLIM AP do the batching?

@param[in] uEnable       Flag that indicates if the service shall be enabled
                         or disabled.
@param[in] wReportRateHz Requested reporting rate in Hz.
@param[in] wSampleCount  Requested sample count in one batch.
@param[in] eService      Service to control.

@return eSLIM_SUCCESS is operation succeeded.
*/
slimErrorEnumT SensorHalProvider::doUpdateSensorStatus
(
  bool uEnable,
  uint16_t wReportRateHz,
  uint16_t wSampleCount,
  slimServiceEnumT eService
)
{
  struct sensor_t const* list = NULL;
  int count = 0;
  int sensor = 0;
  int err = 0;

  SLIM_LOGD("sensorUpdate service %d enable %d rate %d sampleCount %d",
            eService, uEnable, wReportRateHz, wSampleCount);
  if(NULL != device)
  {
    count = device->get_sensors_list(&list);
    if( 0 == count )
    {
      SLIM_LOGE("Failed to get sensor list.");
      return eSLIM_ERROR_UNSUPPORTED;
    }
  } else {
      SLIM_LOGE("device is NULL -sensors_poll_context_t() failed \n");
      return eSLIM_ERROR_UNSUPPORTED;
  }

  switch(eService){
    case eSLIM_SERVICE_SENSOR_ACCEL:
      sensor = SENSOR_TYPE_ACCELEROMETER;
      mAccReportRateHz = wReportRateHz;
      mAccSampleCount = wSampleCount;
      /* In case of Count based batching for QDR, BatchCount should be
         always set to 100 Hz equivalent number. */
      if ((wReportRateHz * wSampleCount) > BATCHSIZE_100HZ)
      {
        mAcc100HzBatchCount = ((wReportRateHz * wSampleCount) / BATCHSIZE_100HZ);
      }
      else
      {
         mAcc100HzBatchCount = wSampleCount;
      }
      mAccServiceStatus = true;
    break;
    case eSLIM_SERVICE_SENSOR_GYRO:
      sensor = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
      mGyroReportRateHz = wReportRateHz;
      mGyroSampleCount = wSampleCount;
      /* In case of Count based batching for QDR, BatchCount should be
         always set to 100 Hz equivalent number. */
      if ((wReportRateHz * wSampleCount) > BATCHSIZE_100HZ)
      {
        mGyro100HzBatchCount = ((wReportRateHz * wSampleCount) / BATCHSIZE_100HZ);
      }
      else
      {
         mGyro100HzBatchCount = wSampleCount;
      }
      mGyroServiceStatus = true;
    break;
    case eSLIM_SERVICE_SENSOR_MAG_UNCALIB:
      sensor = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
    break;
    case eSLIM_SERVICE_SENSOR_BARO:
      sensor = SENSOR_TYPE_PRESSURE;
    break;
    case eSLIM_SERVICE_SENSOR_MAG_CALIB:
    case eSLIM_SERVICE_MAG_FIELD_DATA:
      sensor = SENSOR_TYPE_MAGNETIC_FIELD;
    break;
    default:
      sensor = SENSOR_TYPE_META_DATA;
  }
  /* Trigger buffer data thread once accel and gyro service requests received from clients */
  if (mBufferTxStatus && mAccServiceStatus && mGyroServiceStatus) {
    SLIM_LOGD("Triggering buffer data handler mBufferTxStatus %d \n", mBufferTxStatus);
       pthread_mutex_lock (&mHalBuffMutex);
       pthread_cond_signal (&mHalBuffCond);
       pthread_mutex_unlock (&mHalBuffMutex);
  } /* Allow sensor update rate on the fly*/
  else if (!mBufferTxStatus)
  {
    for (int i=0 ; i<count ; i++) {
      if ((NULL != list) && (list[i].type == sensor))
      {
        SLIM_LOGD("Action for '%s' (%d) is %s\n",
                list[i].name, sensor, ((uEnable==1)?"ENABLE":"DISABLE"));
        err = device->activate(list[i].handle, uEnable);
        if (err != 0) {
          SLIM_LOGE("Action(%d) for '%s' (%d) failed (%s)\n",
                     uEnable,list[i].name, sensor, strerror(-err));
          return eSLIM_ERROR_INTERNAL;
        }
        if ((1 == uEnable) && (wReportRateHz > 0))
        {
          SLIM_LOGD("Setting DelayRate for %s as RateHz %d val %d",
                    list[i].name, wReportRateHz, (1000000000 / (wSampleCount*wReportRateHz)));
          device->setDelay(list[i].handle,(1000000000 / (wSampleCount*wReportRateHz)) );
        }
      }
    }
  }

  /* Enabling the Batching */
  if (wSampleCount > 1)
  {
     SLIM_LOGD("enableBatching eService = %d", eService);
     enableBatching(eService);
  }
  return eSLIM_SUCCESS;
}

void SensorHalProvider::cleanBatchingBuffers()
{
   SLIM_LOGD("cleanBatchingBuffers");
   MutexLock _l(s_zInstanceMutex);
   m_zAccSensorData.samples_len = 0;
   m_zGyroSensorData.samples_len = 0;
   m_bIsAccelPPSTimeNotReached = true;
   m_bIsGyroPPSTimeNotReached = true;
}
