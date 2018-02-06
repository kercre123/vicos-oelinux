/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2015 Qualcomm Technologies, Inc.  All Rights Reserved.
  Qualcomm Technologies Proprietary and Confidential.
=============================================================================*/
/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

                  DRPlugin Messages

GENERAL DESCRIPTION
  This file contains DRPlugin Message definitions.

=============================================================================*/
#ifndef __DRPLUGIN_MSG_H
#define __DRPLUGIN_MSG_H

enum dr_plugin_msg_type_t {
    DR_MSG_BASE_TO_DR = 0x300,
    DR_MSG_BASE_FROM_DR = 0x600,
    DR_MSG_START_SESSION = DR_MSG_BASE_TO_DR + 1,
    DR_MSG_STOP_SESSION,
    DR_MSG_DELETE_SV_DATA,
    DR_MSG_SET_POSITION_MODE,
    DR_MSG_SV_MEASUREMENT,
    DR_MSG_SV_POLYNOMIAL,
    DR_MSG_POSITION_REPORT_TO_DR,
    DR_MSG_GNSS_SV_STATUS_TO_DR,
    DR_MSG_SHUTDOWN_TO_DR,
    DR_MSG_POSITION_REPORT_FROM_DR = DR_MSG_BASE_FROM_DR + 1,
    DR_MSG_NMEA_REPORT_FROM_DR,
    DR_MSG_GNSS_SV_STATUS_FROM_DR,
    DR_MSG_MAX
};

struct dr_plugin_msg {
    const int msgid;
    inline dr_plugin_msg(int id) :
    msgid(id)
    {
        LOC_LOGV("creating msg 0X%x", msgid);
    }
    virtual inline ~dr_plugin_msg()
    {
        LOC_LOGV("deleting msg (0x%x)", msgid);
    }
};

struct dr_plugin_msg_start_session: public dr_plugin_msg {
    inline dr_plugin_msg_start_session () :
    dr_plugin_msg(DR_MSG_START_SESSION)
    {
        LOC_LOGV("DR_MSG_START_SESSION");
    }
};

struct dr_plugin_msg_stop_session: public dr_plugin_msg {
    inline dr_plugin_msg_stop_session () :
    dr_plugin_msg(DR_MSG_STOP_SESSION)
    {
        LOC_LOGV("DR_MSG_STOP_SESSION");
    }
};

struct dr_plugin_msg_set_pos_mode: public dr_plugin_msg {
    const LocGpsPositionMode  positionMode;
    const LocGpsPositionRecurrence recurrence;
    const uint32_t  minInterval;
    const uint32_t  prefAccuracy;
    const uint32_t  prefTime;
    inline dr_plugin_msg_set_pos_mode (LocGpsPositionMode posMode,
               LocGpsPositionRecurrence posRec,  uint32_t  minInt,
               const uint32_t  prefAcc, uint32_t pTime) :
        dr_plugin_msg(DR_MSG_SET_POSITION_MODE),
        positionMode(posMode),
        recurrence(posRec),
        minInterval(minInt),
        prefAccuracy(prefAcc),
        prefTime(pTime)
    {
        LOC_LOGV("DR_MSG_SET_POSITION_MODE");
    }
};

struct dr_plugin_msg_sv_meas: public dr_plugin_msg {
    const GnssSvMeasurementSet  svMeasurementSet;
    inline dr_plugin_msg_sv_meas (GnssSvMeasurementSet svMeasSet) :
        dr_plugin_msg(DR_MSG_SV_MEASUREMENT),
        svMeasurementSet(svMeasSet)
    {
        LOC_LOGV("DR_MSG_SV_MEASUREMENT");
    }
};

struct dr_plugin_msg_sv_poly: public dr_plugin_msg {
    const GnssSvPolynomial  svPolynomial;
    inline dr_plugin_msg_sv_poly (GnssSvPolynomial svPoly) :
        dr_plugin_msg(DR_MSG_SV_POLYNOMIAL),
        svPolynomial(svPoly)
    {
        LOC_LOGV("DR_MSG_SV_POLYNOMIAL");
    }
};

struct dr_plugin_msg_position_to_dr: public dr_plugin_msg {
    const LocGpsLocation  gpsLocation;
    const GpsLocationExtended gpsLocationExt;
    const enum loc_sess_status sessStatus;
    const LocPosTechMask locTechMask;
    inline dr_plugin_msg_position_to_dr (LocGpsLocation gpsLoc, GpsLocationExtended gpsLocExt,
                                               enum loc_sess_status status,
                                               LocPosTechMask loc_technology_mask) :
        dr_plugin_msg(DR_MSG_POSITION_REPORT_TO_DR),
        gpsLocation(gpsLoc),
        gpsLocationExt(gpsLocExt),
        sessStatus(status),
        locTechMask(loc_technology_mask)
    {
        LOC_LOGV("DR_MSG_POSITION_REPORT_TO_DR");
    }
};

struct dr_plugin_msg_sv_status_to_dr: public dr_plugin_msg {
    const LocGnssSvStatus  svStatus;
    const GpsLocationExtended gpsLocationExt;
    inline dr_plugin_msg_sv_status_to_dr (LocGnssSvStatus svStat, GpsLocationExtended gpsLocExt) :
        dr_plugin_msg(DR_MSG_GNSS_SV_STATUS_TO_DR),
        svStatus(svStat),
        gpsLocationExt(gpsLocExt)
    {
        LOC_LOGV("DR_MSG_GNSS_SV_STATUS_TO_DR");
    }
};

struct dr_plugin_msg_delete_sv_data: public dr_plugin_msg {
    const LocGpsAidingData  aidingData;
    inline dr_plugin_msg_delete_sv_data(LocGpsAidingData gpsAidingData) :
        dr_plugin_msg(DR_MSG_DELETE_SV_DATA),
        aidingData(gpsAidingData)
    {
        LOC_LOGV("DR_MSG_DELETE_SV_DATA");
    }
};

struct dr_plugin_msg_shutdown: public dr_plugin_msg {
    inline dr_plugin_msg_shutdown () :
    dr_plugin_msg(DR_MSG_SHUTDOWN_TO_DR)
    {
        LOC_LOGV("DR_MSG_SHUTDOWN_TO_DR");
    }
};

struct dr_plugin_msg_position_from_dr: public dr_plugin_msg {
    const LocGpsLocation  gpsLocation;
    const GpsLocationExtended gpsLocationExt;
    const enum loc_sess_status status;
    const LocPosTechMask technology_mask;
    inline dr_plugin_msg_position_from_dr(LocGpsLocation gpsLoc,
                               GpsLocationExtended gpsLocExt,
                               enum loc_sess_status sessionStatus,
                               LocPosTechMask techMask) :
        dr_plugin_msg(DR_MSG_POSITION_REPORT_FROM_DR),
        gpsLocation(gpsLoc),
        gpsLocationExt(gpsLocExt),
        status(sessionStatus),
        technology_mask(techMask)
    {
        LOC_LOGV("DR_MSG_POSITION_REPORT_FROM_DR");
    }
};

struct dr_plugin_msg_sv_status_from_dr: public dr_plugin_msg {
    const LocGnssSvStatus  svStatus;
    const GpsLocationExtended gpsLocationExt;
    inline dr_plugin_msg_sv_status_from_dr (LocGnssSvStatus svStat,
                               GpsLocationExtended gpsLocExt) :
        dr_plugin_msg(DR_MSG_GNSS_SV_STATUS_FROM_DR),
        svStatus(svStat),
        gpsLocationExt(gpsLocExt)
    {
        LOC_LOGV("DR_MSG_GNSS_SV_STATUS_FROM_DR");
    }
};

struct dr_plugin_msg_nmea_from_dr: public dr_plugin_msg {
    const int    nmeaLength;
    const LocGpsUtcTime mutcTimestamp;
    const char*  nmea;
    inline dr_plugin_msg_nmea_from_dr(const LocGpsUtcTime utcTimestamp,
                               const char* nmeaStr, int nmeaLen):
        dr_plugin_msg(DR_MSG_NMEA_REPORT_FROM_DR),
        nmeaLength(nmeaLen),
        mutcTimestamp(utcTimestamp),
        nmea(new char[nmeaLen])
    {
        if(NULL != nmea)
        {
            memcpy((void*)nmea, (void*)nmeaStr, nmeaLen);
        }
        LOC_LOGV("DR_MSG_NMEA_REPORT_FROM_DR");
    }
    inline ~dr_plugin_msg_nmea_from_dr()
    {
        if (NULL != nmea)
        {
          delete[] nmea;
        }
    }
};

#endif
