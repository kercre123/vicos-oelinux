/**
 * Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
**/

/**
 @file
 @brief Vehicle Network Service provider implementation file.

 This file defines VNS provider object implementation.
 */

#include <inttypes.h>
#include <slim_os_log_api.h>
#include "VNSProvider.h"
#include "VNSListener.h"
#include <hardware/vehicle.h>

//! @addtogroup VNSProvider
//! @{

/*----------------------------------------------------------------------------
 * Preprocessor Definitions and Constants
 * -------------------------------------------------------------------------*/
#undef SLIM_LOG_LOCAL_MODULE
//! @brief Logging module for VNS provider.
#define SLIM_LOG_LOCAL_MODULE SLIM_LOG_MOD_VNS

using namespace slim;

const uint32_t VNSProvider::s_qThreadId = THREAD_ID_VNS;
VNSProvider *VNSProvider::s_pzInstance;
slim::Mutex VNSProvider::s_zInstanceMutex;

const slimAvailableServiceMaskT VNSProvider::s_qSupportedServices =
        (uint32_t(1) << eSLIM_SERVICE_VEHICLE_ACCEL)
        | (uint32_t(1) << eSLIM_SERVICE_VEHICLE_GYRO)
        | (uint32_t(1) << eSLIM_SERVICE_VEHICLE_ODOMETRY);

void VNSProvider::threadMain(void *pData)
{
  SLIM_LOGD("VNSProvider::threadMain(0x%" PRIxPTR ")", (uintptr_t)pData);
  VNSProvider *pzVNSProvider =
      reinterpret_cast<VNSProvider*>(pData);
  slim_IpcStart(s_qThreadId);
  pzVNSProvider->m_pzTimer = slim_TimerCreate(eTIMER_DEFAULT, s_qThreadId);
  pzVNSProvider->m_qServiceMask = s_qSupportedServices;
  pzVNSProvider->routeConfigurationChange(s_qSupportedServices);
  pzVNSProvider->runEventLoop();
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
slim_ServiceProviderInterfaceType *VNSProvider::getProvider()
{
  MutexLock _l(s_zInstanceMutex);
  if (0 == s_pzInstance)
  {
    s_pzInstance = new VNSProvider;
    if (!s_pzInstance->initialize())
    {
      SLIM_LOGE("VNS provider initialization failed");
      delete s_pzInstance;
      s_pzInstance = 0;
      return 0;
    }
  }
  return s_pzInstance->getSlimInterface();
}

VNSProvider::VNSProvider()
    : MultiplexingProvider(SLIM_PROVIDER_VNS, true, 0)
    , m_zThread(), m_zMutex(), m_pzTimer(0)
    , m_qEnabledServices(0)
{
  memset(&m_zThread, 0, sizeof(m_zThread));
}

VNSProvider::~VNSProvider()
{
}

bool VNSProvider::initialize()
{
  if (!slim_ThreadCreate(&m_zThread, threadMain, this, "VNSPoll"))
  {
    return false;
  }
  slim_TaskReadyWait(THREAD_ID_VNS);
  return true;
}


void VNSProvider::FormatData(VNS_Event_Struct *pzNewEvent, int64_t time)
{
  switch (pzNewEvent->propertyType)
  {
    case VEHICLE_PROPERTY_PERF_VEHICLE_SPEED:
    {
      slimVehicleSensorDataStructT zData;
      SLIM_LOGD("Speed value %f",pzNewEvent->speed);
      memset(&zData, 0, sizeof(zData));
      zData.timeBase = pzNewEvent->timeStamp;
      zData.timeSource = eSLIM_TIME_SOURCE_UNSPECIFIED;
      zData.flags = 0;
      zData.provider = eSLIM_SERVICE_PROVIDER_NATIVE;
      zData.sensorType = eSLIM_VEHICLE_SENSOR_TYPE_ACCEL;
      zData.axesValidity = 0;
      zData.axesValidity |= SLIM_MASK_VEHICLE_SENSOR_X_AXIS;
      zData.axesValidity |= SLIM_MASK_VEHICLE_SENSOR_Y_AXIS;
      zData.axesValidity |= SLIM_MASK_VEHICLE_SENSOR_Z_AXIS;
      zData.samples_len = 1;
      zData.samples[0].sampleTimeOffset = 0;
      zData.samples[0].sample[0] = pzNewEvent->speed;
      zData.samples[0].sample[1] = 0;
      zData.samples[0].sample[2] = 0;
      routeIndication(eSLIM_SERVICE_VEHICLE_ACCEL, zData);
    }
    break;
    case VEHICLE_PROPERTY_CURRENT_GEAR:
    {
      slimVehicleOdometryDataStructT zData;
      SLIM_LOGD("Gear value %d",pzNewEvent->gear);
      memset(&zData, 0, sizeof(zData));
      zData.timeBase      = pzNewEvent->timeStamp;
      zData.timeSource    = eSLIM_TIME_SOURCE_UNSPECIFIED;
      zData.provider      = eSLIM_SERVICE_PROVIDER_NATIVE;
      zData.distanceTravelledBase = 0;
      zData.odometryFlags = 0;
      zData.wheelFlags    = SLIM_MASK_VEHICLE_ODOMETRY_LEFT_AND_RIGHT_AVERAGE;
      zData.samples_len   = 1;
      zData.samples[0].distanceTravelled[0] = pzNewEvent->gear;
      zData.samples[0].sampleTimeOffset = 0;
      routeIndication(zData);
    }
    break;
    default:
    {
      SLIM_LOGD("Unhandled Event received at VNS");
    }
    break;
  }
}

/* Processing loop for events */
void VNSProvider::runEventLoop()
{
  SLIM_LOGD("Starting event handler loop");

  /* Register for new event handlers with Subscribe method */
  sp<VNSListener>  vnNcarLis = new VNSListener();
  sp<VehicleNetworkListener> listener(vnNcarLis.get());
  sp<VehicleNetwork> vnNcar = VehicleNetwork::createVehicleNetwork(listener);
  if (vnNcar.get() != NULL)
  {
    /* Register call back for speed event with required sampling rate */
    if (NO_ERROR != vnNcar->subscribe(VEHICLE_PROPERTY_PERF_VEHICLE_SPEED,10))
    {
      SLIM_LOGE("Speed subscribe Failed");
    }
    else
    {
      SLIM_LOGD("Speed subscribe Success");
    }
    /* Register call back for gear event with required sampling rate */
    if (NO_ERROR != vnNcar->subscribe(VEHICLE_PROPERTY_CURRENT_GEAR,10))
    {
      SLIM_LOGE("Gear subscribe Failed");
    }
    else
    {
      SLIM_LOGD("Gear subscribe Success");
    }
  }

  // Notify controller that the task is ready to run.
  slim_TaskReadyAck();
  while (1)
  {
    slim_IpcMsgT* pz_Msg = NULL;
    while (NULL == (pz_Msg = slim_IpcReceive()));

    SLIM_LOGD("IPC message received. q_MsgId:%" PRIu32 ", q_SrcThreadId:%" PRIu32,
        (uint32_t)pz_Msg->q_MsgId,
        (uint32_t)pz_Msg->q_SrcThreadId);

    switch (pz_Msg->q_MsgId)
    {
      case eIPC_MSG_NEW_VNS_EVENT:
      {
        VNS_Event_Struct *pzNewEvent;
        MutexLock _l(m_zMutex);
        slim_Memscpy(&pzNewEvent, sizeof(pzNewEvent), pz_Msg->p_Data, pz_Msg->q_Size);
        FormatData(pzNewEvent,0);
        delete pzNewEvent;
      }
      break;
      default:
      {
        SLIM_LOGD("Unhandled runEvent:%d",pz_Msg->q_MsgId);
      }
      break;
    }
    slim_IpcDelete(pz_Msg);
  }
}

/**
 @brief Method for enabling or disabling vehicle services.

 @param[in] uEnable Flag that indicates if the service shall be enabled or
 disabled.
 @param[in] eService      Service to control.

 @return eSLIM_SUCCESS is operation succeeded.
 */
slimErrorEnumT VNSProvider::doUpdateVehicleDataStatus(
bool uEnable, slimServiceEnumT eService)
{
  MutexLock _l(m_zMutex);
  if (uEnable)
  {
    SLIM_MASK_SET(m_qEnabledServices, eService);
  }
  else
  {
    SLIM_MASK_CLEAR(m_qEnabledServices, eService);
  }
  if (0 == m_qEnabledServices)
  {
    slim_TimerStop(m_pzTimer);
  }
  else
  {
    slim_TimerStart(m_pzTimer, 500, eTIMER_DEFAULT);
  }
  return eSLIM_SUCCESS;
}

/**
 @brief Initiates time offset request.

 Function for making the time request. Successful response enable SLIM to
 calculate the offset between modem time and sensor time.

 @param[in] lTxnId Service transaction id.
 @return eSLIM_SUCCESS if time request is made successfully.
 */
slimErrorEnumT VNSProvider::getTimeUpdate(int32_t lTxnId)
{
  slimErrorEnumT retVal;
  retVal = eSLIM_SUCCESS;

  SLIM_LOGD("Requesting time update: txn=%" PRId32, lTxnId);
  routeTimeResponse(lTxnId, eSLIM_ERROR_UNSUPPORTED, 0, 0);
  SLIM_LOGD("retVal= %d", retVal);
  return retVal;
}

