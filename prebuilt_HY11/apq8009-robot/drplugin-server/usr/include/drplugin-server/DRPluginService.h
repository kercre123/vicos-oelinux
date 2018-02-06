/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2015 Qualcomm Technologies, Inc.  All Rights Reserved.
  Qualcomm Technologies Proprietary and Confidential.
=============================================================================*/

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

                  DRPluginServiceLib definition header

GENERAL DESCRIPTION
 This file contains the definitions for DRPluginServiceLib.

=============================================================================*/
#ifndef _DRPLUGIN_SERVICE_LIB_H
#define _DRPLUGIN_SERVICE_LIB_H

#include<stdio.h>
#include<pthread.h>
#include<unistd.h>
//#include<iostream.h>
#include <list>
#include "Slim_APDRClient.h"
#include "DRExternal.h"

#define DR_STATISTIC_CONFIG_FILE  LOC_PATH_APDR_CONF
#define DR_CONSTELLATION_CONFIG_FILE  LOC_PATH_APDR_CONF

#define SEC_TO_MILI_SEC  1000    /* MilliSec */
#define NSEC_TO_MILI_SEC 1000000 /* NenoSec to MilliSec */
#define NSEC_COARSE      500000  /* NenoSec Coarse */

using namespace std;

class DRPluginServiceLib;
class DRPluginClientListener;
class DRPluginServiceLib;
class DRPluginLogger;
class DRPluginStatistic;
class DRPluginDiagLogger;
class DRGnssStatistic;
class DRStorage;

typedef struct {
   DRPluginClientListener *client_listener;
   pthread_t            clientThrdID;
   pthread_mutex_t      client_mutex;
   int                  client_sockfd;
   bool                 threadExit;

}drplugin_clientContext;
typedef struct{
   uint32_t maxNodeLength;
   uint32_t data_id;
   uint64_t crcInfo;
   bool     node_valid;
   bool     node_deleted;
   size_t   dataLength;
}drStroageNode_header;

typedef enum {
  DR_CLIENT_INIT = 0,
  DR_CLIENT_CHANNEL_OPEN,
  DR_CLIENT_IN_SESSION
} DR_CLIENT_STATE_T;

class DRPluginClientListener {
private:
    uint32_t mClientHandle;
    bool mActiveStatus;
    DR_CLIENT_STATE_T mClientState;
    DRPluginServiceLib *mDrPluginService;
    drplugin_clientContext *mClientContext;

    void* (*client_rcv_thread)(void *arg);
    void close_thread(void);
    void handle_start_session(dr_plugin_msg *dr_msg);
    void handle_stop_session(dr_plugin_msg *dr_msg);
    void handle_delete_SVData(dr_plugin_msg *dr_msg);
    void handle_set_posMode(dr_plugin_msg *dr_msg);
    void handle_sv_measurement(dr_plugin_msg *dr_msg);
    void handle_sv_polynomial(dr_plugin_msg *dr_msg);
    void handle_pos_report(dr_plugin_msg *dr_msg);
    void handle_sv_status(dr_plugin_msg *dr_msg);
    void handle_shutdown(dr_plugin_msg *dr_msg);

public:

    DRPluginClientListener(unsigned int clientHandle,
                     DRPluginServiceLib *pDRPluginServiceLib);
    ~DRPluginClientListener();
    bool setActive(bool activeStatus);
    bool getActiveState(void);
    void set_client_state(DR_CLIENT_STATE_T setClientState);
    DR_CLIENT_STATE_T get_client_state(void);
    bool handleDRPluginMsg(DRPluginClientListener *drpluginClientListener,
                     dr_plugin_msg *dr_msg);
    bool reportPosition(LocGpsLocation &location,
                     GpsLocationExtended &locationExtended,
                     enum loc_sess_status sessionStatus,
                     LocPosTechMask techMask);
    bool reportNMEA(LocGpsUtcTime timestamp,const char*  nmea,int nmeaLength);
    bool reportSVStatus(LocGnssSvStatus &svStatus,
                     GpsLocationExtended &gpsLocationExt);
    void handle_error(drplugin_clientContext *clientContext);
}; //DRPluginClientListener


typedef struct {
   DRPluginServiceLib   *server_lib;
   pthread_t            serverThrdID;
   pthread_mutex_t      server_mutex;
   int                  server_sockfd;
   bool                 server_threadExit;

}drplugin_serverContext;


typedef struct{
   unsigned int gnssMsrStatisEnble;
   unsigned int gnssPosStatisEnble;
   unsigned int gnssMsrStatisPrntMax;
   unsigned int gnssPosStatisPrntMax;
   unsigned int gpsMsrStatisPrntCntr;
   unsigned int gloMsrStatisPrntCntr;
   unsigned int gnssPolyStatisPrntCntr;
   unsigned int gnssPosStatisPrntCntr;
}DRPluginStatisticConf;

class DRPluginServiceLib {
private:
    unsigned int mActiveClients;
    unsigned int mActiveSessions;
    int mListenSocketFd;
    class list<DRPluginClientListener *>  mClientListenerList;
    DRPluginClientListener * mAtiveClientInfo;
    Slim_APDRClient  mSlimClient;
    void init_server_socket_connection(void);
    void* (*connection_listener_thread)(void *arg);
    DRPluginLogger *mDRPluginLogFile;
    DRPluginDiagLogger *mDRPluginDiagPrnt;
    DRGnssStatistic  *mDRGnssStatisticLog;
    unsigned int mDrConstellationMask;
public:
    DRPluginServiceLib(void);
    ~DRPluginServiceLib(void);
    bool handoverActive(DRPluginClientListener *drpluginClientListener);
    bool addClient(int socketfd);
    bool deleteClient(DRPluginClientListener *deltClient);
    //bool setNextActive(DRPluginClientListener *activeClint);
    int  getSrvrSocket(void);
    bool StartSession(DRPluginClientListener *drpluginClientListener);
    bool StopSession(DRPluginClientListener *drpluginClientListener);
    bool UpdateStartBatchTime(uint64_t startBatchTime);
    bool injectSvMeasurement(DRPluginClientListener *drpluginClientListener,
            GnssSvMeasurementSet &svMeasurement);
    bool injectSvPolynomial(DRPluginClientListener *drpluginClientListener,
            GnssSvPolynomial &scPolynomial);
    bool injectGpsPosition(DRPluginClientListener *drpluginClientListener,
            LocGpsLocation &location,GpsLocationExtended &locationExtended,
            enum loc_sess_status sessStatus, LocPosTechMask locTechnologyMask);
    bool injectSvStaus(DRPluginClientListener *drpluginClientListener,
            LocGnssSvStatus &svStatus,GpsLocationExtended &locationExt);
    bool deleteAidingData(DRPluginClientListener *drpluginClientListener,
            LocGpsAidingData aidingData);
    bool setPositionMode(DRPluginClientListener *drpluginClientListener,
            const LocGpsPositionMode  posMode,
            const LocGpsPositionRecurrence recurrence,
            const uint32_t  minInterval,
            const uint32_t  prefAccuracy,
            const uint32_t  prefTime);
    bool Shutdown(DRPluginClientListener *drpluginClientListener);
    bool reportDRPosition(LocGpsLocation &location,
                     GpsLocationExtended &locationExtended,
                     enum loc_sess_status sessionStatus,
                     LocPosTechMask techMask);
    bool reportDRNMEA(LocGpsUtcTime timestamp,const char*  nmea,int nmeaLength);
    bool reportDRSVStatus(LocGnssSvStatus &svStatus,
                     GpsLocationExtended &gpsLocationExt);
    int drStorageWrite(uint32_t data_id,const char* drInfo,size_t infoLength);
    int drStorageRead(uint32_t data_id,char* drInfo,size_t* infoLength);
    inline void startSlimClient(void){
        mSlimClient.initialize();
    }
    bool updateDRLogger(DRPluginLogger *drLoggerFilePtr);
    bool confStatistic(void);
    bool deleteConfStatistic(void);
    bool confConstellation(void);
}; //DRPluginServiceLib

typedef enum {
    DR_MIN_LOG = 0,
    DR_NO_LOG = 1,
    DR_SV_MSR,
    DR_SV_PLNML,
    DR_ALL_LOG
}DRLOGGER_LEVEL;

class DRPluginLogger{
private:
    FILE *mLoggerFilePtr;
    bool mMaxFileSize;
    bool mFileRollOver;
    DRLOGGER_LEVEL mloggerLevel;
    bool drLogger_openFile(const char *file_name);
    bool drLogger_writeFile(const char* logInfo,size_t logLength);
    bool drLogger_closeFile(void);
public:
    DRPluginLogger(const char *file_name,size_t maxFileSize,bool rollOver,
                               DRLOGGER_LEVEL logLevel);
    ~DRPluginLogger(void);
    bool drPluginLog(const char* logInfo,size_t logLength);
    bool drLoggerPrint(const char *format, ...);
    void drLogSvMeasurement(GnssSvMeasurementSet &svMeasurementSet);
    void drLogSvPolynomial(GnssSvPolynomial &svPolynomial);
    void drLogPositionReport(LocGpsLocation &location,
                               GpsLocationExtended &locationExtended);
};

class DRGnssStatistic{
private:
    DRPluginStatistic  *mGpsMsrStatistic;
    DRPluginStatistic  *mGloMsrStatistic;
    DRPluginStatistic  *mPosStatistic;
    DRPluginStatistic  *mSvPloyStatistic;
    DRPluginLogger     *mStatsLogger;
    DRPluginDiagLogger *mStatsDiag;
    DRPluginStatisticConf mGnssStatisConf;
public:
    DRGnssStatistic(DRPluginStatisticConf gnssStatisConf_in,
        DRPluginLogger *statsLogger,DRPluginDiagLogger *diagLogger);
    ~DRGnssStatistic(void);
    bool printStaticInDiag(const char *inType,apDr_diagLogGnssRptType logType,
        DRStatisticLog &statsDig);
    bool printStaticInLogger(const char *inType,DRStatisticLog &statsLogger);
    bool calculateGnssMsrStatistic(GnssSvMeasurementSet &svMeasurement);
    bool calculateSvPolyStatistic(void);
    bool calculatePosStatistic(void);
};

class DRStorage{
private:
    FILE *mQdrInfoFilePtr;
    bool mMaxNode;
    pthread_mutex_t drStorage_lock;
    bool drStorage_openFile(const char *file_name);
    bool drStorage_writeFile(drStroageNode_header nodeInfo,const unsigned char* dataInfo);
    bool drStorage_readFile(drStroageNode_header *nodeInfo,unsigned char* dataInfo);
    bool drStorage_deleteFile(drStroageNode_header deleteNodeInfo);
    bool drStorage_closeFile(void);
public:
    DRStorage(const char *file_name,size_t maxNode);
    ~DRStorage(void);
    int DrStorage_Write(QdrStorage_data_t& writeQdrNode);
    int DrStorage_Read(QdrStorage_data_t *readQdrNode);
    int DrStorage_Delete(QdrStorage_data_t& deleteQdrNode);
};
#endif //_DRPLUGIN_SERVICE_LIB_H
