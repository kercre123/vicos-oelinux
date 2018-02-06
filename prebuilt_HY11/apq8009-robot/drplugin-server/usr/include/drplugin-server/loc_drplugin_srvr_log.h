/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2015 Qualcomm Technologies, Inc.  All Rights Reserved.
  Qualcomm Technologies Proprietary and Confidential.
=============================================================================*/

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

                 DR Plugin Server generic definition header

GENERAL DESCRIPTION
 This file contains generic definition header for Dead Reckoning API interfaces.

=============================================================================*/
#ifndef __DRPLUGIN_SERVER_LOG_H
#define __DRPLUGIN_SERVER_LOG_H

/* Logging */
#define LOG_DEBUG 1

#ifdef LOG_DEBUG
#define DR_LOGV(...) LOC_LOGV("DR_SRV: V/" __VA_ARGS__)
#define DR_LOGD(...) LOC_LOGD("DR_SRV: D/" __VA_ARGS__)
#define DR_LOGI(...) LOC_LOGI("DR_SRV: I/" __VA_ARGS__)
#define DR_LOGW(...) LOC_LOGW("DR_SRV: W/" __VA_ARGS__)
#define DR_LOGE(...) LOC_LOGE("DR_SRV: E/" __VA_ARGS__)
#else /* log Debug enable */
#define DR_LOGV(...) printf(__VA_ARGS__)
#define DR_LOGD(...) printf(__VA_ARGS__)
#define DR_LOGI(...) printf(__VA_ARGS__)
#define DR_LOGW(...) printf(__VA_ARGS__)
#define DR_LOGE(...) printf(__VA_ARGS__)
#endif /* log Debug enable */

#define DRLOG_STATIS_PRNT_MASK       0x00000007
#define DRLOG_STATIS_DIAG_PRNT_ENBLE 0x00000001
#define DRLOG_STATIS_ADBL_PRNT_ENBLE 0x00000002
#define DRLOG_STATIS_FILE_PRNT_ENBLE 0x00000004

#define IS_DIAG_STATIS_PRNT_ENBLE(x)\
    ((((x)&(DRLOG_STATIS_DIAG_PRNT_ENBLE))== DRLOG_STATIS_DIAG_PRNT_ENBLE)\
       ?(true):(false))

#define IS_ADBL_STATIS_PRNT_ENBLE(x)\
    ((((x)&(DRLOG_STATIS_ADBL_PRNT_ENBLE))== DRLOG_STATIS_ADBL_PRNT_ENBLE)\
       ?(true):(false))

#define IS_FILE_STATIS_PRNT_ENBLE(x)\
    ((((x)&(DRLOG_STATIS_FILE_PRNT_ENBLE))== DRLOG_STATIS_FILE_PRNT_ENBLE)\
       ?(true):(false))

struct DRStatisticLog{
    double diff;
    double stdDeviation;
    double avg;
    double min;
    double max;
    unsigned int smpleNumbr;
};

class DRPluginStatistic{
private:
    unsigned int totalNumSample;
    double sumOfSample;
    double multiOfSample;
    double minSample;
    double maxSample;
    struct timespec lastKnowTime;
public:
    DRPluginStatistic(void);
    ~DRPluginStatistic(void);
    void resetTimeStatistic(void);
    double calculateTimeDiff(struct timespec time1, struct timespec time2);
    bool calculateTimeStatistic(DRStatisticLog* getStatInfo);
};

/*
class DRPluginDiagLogger{
private:
    unsigned int isDiagInit;
public:
    DRPluginDiagLogger(void);
    ~DRPluginDiagLogger(void);
    bool drDiagPrint(const char *format, ...);
    bool drDiagLogPkt_SensorStatistics( );
};
*/

/*** DIAG LOG Packet ***/

/** @brief Enum for APDR event type. */
typedef enum
{
  eAPDR_DIAGLOG_MSG_TYPE_REQUEST  = 0, /** Request received from client. */
  eAPDR_DIAGLOG_MSG_TYPE_RESPONSE,     /** Response back to client. */
  eAPDR_DIAGLOG_MSG_TYPE_INDICATION,   /** Indication to client. */
  eAPDR_DIAGLOG_MSG_TYPE_INTERNAL,     /** Internal APDR event. */

  eAPDR_DIAGLOG_MSG_TYPE_MAX = 2147483647 /** To force a 32 bit signed enum.
                                           Do not change or use */
 } apDr_diagLogMsgTypeEnumType;

/** @brief Enum for APDR Diag logger event id Version 01. */
typedef enum
{
   eAPDR_DIAGLOG_EVT_REQ_OPEN_V01 = 0,        /** Start logging. */
   eAPDR_DIAGLOG_EVT_REQ_CLOSE_V01,           /** Stop logging. */
   eAPDR_DIAGLOG_EVT_DRENG_STATUS_V01,        /** DR Engine Status. */
   eAPDR_DIAGLOG_EVT_GNSS_STATISTICS_V01,     /** GNSS Statistics */
   eAPDR_DIAGLOG_EVT_SENSOR_STATISTICS_V01,   /** sensor Statistics */

   eAPDR_DIAGLOG_EVT_MAX_EVENT = 2147483647   /** To force a 32 bit signed enum.
                                               Do not change or use */
}apDr_diagLogEvntEnumType_V01;

typedef enum
{
   /** Sensor is not present on Platfrom */
   eAPDR_SENSOR_NOTPRESENT = 0,
   /** Sensor is disabled via configuration */
   eAPDR_SENSOR_DISABLED,
   /** Sensor is enabled */
   eAPDR_SENSOR_ENABLED,
   /** Sensor is in error status */
   eAPDR_SENSOR_IN_ERROR,

   /** To force a 32 bit signed enum.
   Do not change or use */
   eAPDR_SENSOR_MAX_STATUS = 2147483647
}apDr_diagLogSensorStatus;

/** @brief Report DR GNSS Statistics. */
typedef enum
{
   /** GNSS Position Report */
   eAPDR_GNSS_POS_RPT = 0,
   /** GPS Measurment Report */
   eAPDR_GNSS_GPS_MSR,
   /** GALILEO Measurment Report*/
   eAPDR_GNSS_GLO_MSR,
  /** BeiDou Measurment Report*/
   eAPDR_GNSS_BDS_MSR,
   /** Galileo Measurment Report*/
   eAPDR_GNSS_GAL_MSR,
   /** Polynomial Measurment Report*/
   eAPDR_GNSS_PLY_MSR,
}apDr_diagLogGnssRptType;

/** @brief Enum for APDR Diag logger event id. */
typedef enum
{
   /** Start logging. */
   eAPDR_DIAGLOG_EVT_REQ_OPEN
                       = eAPDR_DIAGLOG_EVT_REQ_OPEN_V01,
   /** Stop logging. */
   eAPDR_DIAGLOG_EVT_REQ_CLOSE
                       = eAPDR_DIAGLOG_EVT_REQ_CLOSE_V01,
   /** DR Engine Status. */
   eAPDR_DIAGLOG_EVT_DRENG_STATUS
                       = eAPDR_DIAGLOG_EVT_DRENG_STATUS_V01,
   /**  GNSS Statistics */
   eAPDR_DIAGLOG_EVT_GNSS_STATISTICS
                       = eAPDR_DIAGLOG_EVT_GNSS_STATISTICS_V01,
   /** sensor Statistics */
   eAPDR_DIAGLOG_EVT_SENSOR_STATISTICS
                       = eAPDR_DIAGLOG_EVT_SENSOR_STATISTICS_V01
}apDr_diagLogEvntReqType;

/** @brief APDR Diag logger header for all messages. */
typedef PACKED struct PACKED_POST
{
  /** Used by Logging Module */
  log_hdr_type z_LogHeader;
  /** AP DR Message Version */
  uint8 u_Version;
  /** Process identification */
  int32 u_Process_id;
  /** time at logging in milliseconds */
  uint64 t_TimeTickMsec;
  /** Event type of the type apDr_diagLogMsgTypeEnumType */
  uint32 u_EventType;
  /** Event id of the type apDr_diagLogEvntReqType for Request only */
  uint32 u_EventId;
} apDr_diagLogGenericEventHeader;

/** @brief Statistics Report for Sensor and GNSS. */
typedef PACKED struct PACKED_POST
{
    uint32 validMask;
    double stdDeviation;
    double avg;
    double min;
    double max;
    uint32 sampleCount;
}apDr_diagLogStatisticsRpt;

/** @brief Sensor provider and service information . */
typedef PACKED struct PACKED_POST
{
   /** Sensor Provier Type is based on
       slimServiceProviderEnumT
       defined in slim_client_types.h */
   uint32_t u_DRSnsrProviderType;
   /** Sensor Provier Type is based on
       slimSensorTypeEnumT
       defined in slim_client_types.h */
   uint32_t u_DRSnsrServiceType;
} apDr_SensorType;

typedef PACKED struct PACKED_POST
{
   /** Request Event Header */
   apDr_diagLogGenericEventHeader req_header;
   /**< Max size is LOG_GNSS_APDR_DIALOG_EVENT_MAX_PAYLOAD_SIZE */
   uint8 z_Payload[1];
}apDr_diagLogDefultReq;

/** @brief DR Engine Status Report. */
/** LOG_GNSS_APDR_DIAG_DRENG_STATUS_EVENT_C */
typedef PACKED struct PACKED_POST
{
   /** Request Event Header */
   apDr_diagLogGenericEventHeader req_header;
   /** Indicates DR engine Status **/
   uint32_t  dr_engine_status;
   /** Indicates DR engine error count **/
   uint32_t  dr_engine_error_cnt;
   /** Indicates DR engine reset count **/
   uint32_t  dr_engine_reset_cnt;
}apDr_diagLogDREngStatusRpt;

/** @brief GNSS Statistics Report. */
/** LOG_GNSS_APDR_DIAG_GNSS_STATISTICS_EVENT_C */
typedef PACKED struct PACKED_POST
{
   /** Request Event Header */
   apDr_diagLogGenericEventHeader req_header;
   /** Statistics Report Type */
   apDr_diagLogGnssRptType    gnssRptType;
   /** GNSS Statistics Report */
   apDr_diagLogStatisticsRpt  gnssStatistics;
}apDr_diagLogGnssStatsRpt;

/** @brief Sensor Statistics Report. */
/** LOG_GNSS_APDR_DIAG_SENSOR_STATISTICS_EVENT_C */
typedef PACKED struct PACKED_POST
{
   /** Request Event Header */
   apDr_diagLogGenericEventHeader req_header;
   /** Sensor Info */
   apDr_SensorType            sensorType;
   /** Sensor Status  */
   apDr_diagLogSensorStatus   sensorCurrentStatus;
   /** Sensor Statistics */
   apDr_diagLogStatisticsRpt  sensorStatistics;
}apDr_diagLogSensorStatsRpt;

/**
 * Diag Logger
 */
class DRPluginDiagLogger{
private:
    unsigned int isDiagInit;
    bool drDiagLogPkt_fillHeadr(apDr_diagLogGenericEventHeader *hdr_p,
       apDr_diagLogMsgTypeEnumType logType,
       apDr_diagLogEvntReqType evntType);
public:
    DRPluginDiagLogger(void);
    ~DRPluginDiagLogger(void);
    bool drDiagPrint(const char *format, ...);
    bool drDiagLogPkt_start(void);
    bool drDiagLogPkt_stop(void);
    bool drDiagLogPkt_APDR_drEngStatus(uint32_t drEngine_status,
               uint32_t drEngine_errorCnt, uint32_t drEngine_resetCnt);
    bool drDiagLogPkt_GnssStatistics(apDr_diagLogGnssRptType GnssType,
               apDr_diagLogStatisticsRpt &StatisticsRpt);
    bool drDiagLogPkt_SensorStatistics(apDr_SensorType &SensorType,
               apDr_diagLogSensorStatus sensorStatus,
               apDr_diagLogStatisticsRpt &StatisticsRpt);

};

/**
 * @brief Maximum APDR LOG diagnostics event payload size in bytes.
 */
#define LOG_GNSS_APDR_DIG_MSG_MAX_PAYLOAD_SIZE (1024) /* 1KByte */
/**
 * @brief APDR LOG request/response event version
 */
#define LOG_GNSS_APDR_DIG_MSG_VERSION   (1)

#define LOG_GNSS_APDR_DIG_STD_DEVIATION (1)
#define LOG_GNSS_APDR_DIG_AVG           (2)
#define LOG_GNSS_APDR_DIG_MIN           (4)
#define LOG_GNSS_APDR_DIG_MAX           (8)
#define LOG_GNSS_APDR_DIG_SAMPLE_COUNT  (16)
#define LOG_GNSS_APDR_DIG_ALL_DISABLE   (0)
#define LOG_GNSS_APDR_DIG_ALL_ENABLE    (31)

#endif //__DRPLUGIN_SERVER_LOG_H
