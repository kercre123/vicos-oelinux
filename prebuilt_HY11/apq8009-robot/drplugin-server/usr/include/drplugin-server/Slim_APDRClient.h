/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2015-2017 Qualcomm Technologies, Inc.  All Rights Reserved.
  Qualcomm Technologies Proprietary and Confidential.
=============================================================================*/
#ifndef __SLIM_APDR_CLIENT_H_
#define __SLIM_APDR_CLIENT_H_

#include <slim_processor.h>
#include <slim_client_types.h>
#include <SlimClientBase.h>
#include "DR_AP_ServiceConfig.h"

using namespace slim;
class Slim_APDRClientStat;

class Slim_APDRClient : public ClientBase
{
private:
  slim_ClientEnumType   m_eClient;
  DR_AP_ServiceConfig     m_DRAPConfig;
  bool                  m_uAutoRestart;
  slimServiceStatusEventStructT m_SlimServiceStatus[eSLIM_SERVICE_LAST+1];

  slimErrorEnumT setServiceStatus
  (
    slimServiceEnumT eService,
    uint8_t eServiceStatus
  );

  slimErrorEnumT UpdateStartBatchTime
  (
    slimServiceEnumT eService,
    uint8_t eServiceStatus,
    uint64_t batchTimeStart
  );

  Slim_APDRClientStat *sensorStats;
public:

  Slim_APDRClient();
  virtual ~Slim_APDRClient();

  /**
   * @brief Initialize application object
   *
   * Function performs config file argument parsing and configuration loading.
   *
   * @param VOID
   */
  void initialize();

  bool startSensorData();
  bool stopSensorData();
  bool UpdateStartBatchTime(uint64_t startBatchTime);

protected:
  virtual void handleOpenResult
  (
    slimErrorEnumT eError
  );
  virtual void handleServiceEnableResult
  (
    slimServiceEnumT eService,
    uint8_t          uTxnId,
    slimErrorEnumT   eError
  );
  virtual void handleServiceError
  (
    slimServiceEnumT eService,
    slimErrorEnumT   eError
  );
  virtual void handleSensorData
  (
    slimServiceEnumT eService,
    const slimSensorDataStructT &zData
  );
  virtual void handleQmdData
  (
    slimServiceEnumT eService,
    const slimQmdDataStructT &zData
  );
  virtual void handlePedometerData
  (
    slimServiceEnumT eService,
    const slimPedometerDataStructT &zData
  );
  virtual void handleVehicleOdometryData
  (
    slimServiceEnumT eService,
    const slimVehicleOdometryDataStructT &zData
  );
  virtual void handleVehicleSensorData
  (
    slimServiceEnumT eService,
    const slimVehicleSensorDataStructT &zData
  );
  virtual void handleVehicleMotionData
  (
    slimServiceEnumT eService,
    const slimVehicleMotionDataStructT &zData
  );
  virtual void handleServiceStatus
  (
    const slimServiceStatusEventStructT &zData
  );
  virtual void handleServiceProviderChange
  (
    slimServiceEnumT eService,
    slimServiceProviderMaskT qRemoved,
    slimServiceProviderMaskT qAdded,
    slimServiceProviderMaskT qCurrent
  );
};

class Slim_APDRClientStat
{
private:
  DRPluginDiagLogger   *mSensorDiagPrnt;
  DR_AP_ServiceConfig   mSensorDRAPConfig;
public:
  Slim_APDRClientStat(DR_AP_ServiceConfig &sensorCondf);
  ~Slim_APDRClientStat(void);
  void logSensorStatistic(slimServiceEnumT eService);
};
#endif /* __SLIM_APDR_CLIENT_H_ */
