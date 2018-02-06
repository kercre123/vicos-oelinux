/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2013-2015 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/
/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

                  LocUlpProxy header file

GENERAL DESCRIPTION
  This file contains the data structure and variables used for handling requests
  sent by GPS HAL. This acts as a proxy for the ULP module and generally forwards
  the requests to the ULP msg queue
=============================================================================*/

#ifndef LOC_ULP_PROXY_H
#define LOC_ULP_PROXY_H

#include <LocAdapterBase.h>
#include "EventObserver.h"

using namespace loc_core;

class LocUlpProxy : public UlpProxyBase {
    LocAdapterBase* mAdapter;
    EventObserver  *mEventObserver;
public:
    unsigned long mCapabilities;
    void* mQ;
    LocUlpProxy();
    virtual ~LocUlpProxy();
    virtual bool sendStartFix();
    virtual bool sendStopFix();
    virtual bool sendFixMode(LocPosMode &params);
    inline virtual bool reportDeleteAidingData(LocGpsAidingData aidingData)
    {
        ENTRY_LOG();
        return false;
    }
    virtual bool reportPosition(UlpLocation &location,
                                GpsLocationExtended &locationExtended,
                                void* locationExt,
                                enum loc_sess_status status,
                                LocPosTechMask loc_technology_mask);
    virtual bool reportPositions(const FlpExtLocation * locations,
                                 int32_t number_of_locations);
    virtual bool reportBatchingSession(FlpExtBatchOptions & options,
                                       bool active);
    virtual bool reportSv(LocGnssSvStatus &svStatus,
                          GpsLocationExtended &locationExtended,
                          void* svExt);

    //TBD when measurement is needed by LA.
    inline virtual bool reportSvMeasurement(GnssSvMeasurementSet &svMeasurementSet)
    {
        ENTRY_LOG();
        return false;
    }
    inline virtual bool reportSvPolynomial(GnssSvPolynomial &svPolynomial)
    {
        ENTRY_LOG();
        return false;
    }

    virtual bool reportStatus(LocGpsStatusValue status);
    inline virtual void setAdapter(LocAdapterBase* adapter);
    inline virtual void setCapabilities(unsigned long capabilities) {
        mCapabilities = capabilities;
    }

    inline virtual bool DRInit(void * create_thread_cb) { return false; }

    inline void sendLocMsg(LocMsg* msg) {
        if (NULL != mAdapter)
            mAdapter->sendMsg(msg);
    }

    inline virtual LocAdapterBase* getAdapter() { return mAdapter; }

    /**++
    ** External DR APIs for AMTproxy inheritance
    --*/

    inline virtual bool sendFixModeToDr(LocPosMode &params) { return false; }
    inline virtual bool startDrSession() { return false; }
    inline virtual bool stopDrSession() { return false; }
    inline virtual bool reportDrPosition(UlpLocation &location,
                                GpsLocationExtended &locationExtended,
                                void* locationExt,
                                enum loc_sess_status status,
                                LocPosTechMask loc_technology_mask) { return false; }

    inline virtual bool sendGnssPositionToDr(UlpLocation &location,
                                            GpsLocationExtended &locationExtended,
                                            void* locationExt,
                                            enum loc_sess_status status,
                                            LocPosTechMask loc_technology_mask) { return false; }
    inline virtual bool sendGnssSvMeasurementToDr(GnssSvMeasurementSet &svMeasurementSet) {return false; }
    inline virtual bool sendGnssSvPolynomialToDr(GnssSvPolynomial &svPolynomial) { return false; }
    inline virtual bool sendDeleteAidingDataToDr(const LocGpsAidingData aidingData) { return false; }
    inline virtual bool sendGnssSvToDr(LocGnssSvStatus &svStat,
                                       GpsLocationExtended &locationExtended) { return false; }
    inline virtual int sendSystemEvent(unsigned int sysEvent){
        if (NULL != mEventObserver) {
            return mEventObserver->sendSystemEvent(sysEvent);
        }
        return 0;
    }
   virtual bool reportNmea(const char* nmea, int length);
};

#endif /* LOC_ULP_PROXY_H */
