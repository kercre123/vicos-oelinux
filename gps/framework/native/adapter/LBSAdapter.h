/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2013, 2016-2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/

#ifndef LBS_ADAPTER_H
#define LBS_ADAPTER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <WiperData.h>
#include <stdbool.h>
#include <string.h>
#include <LBSAdapterBase.h>
#include <LocDualContext.h>

using namespace loc_core;
using namespace lbs_core;
using namespace izat_manager;

enum zpp_fix_type {
    ZPP_FIX_WWAN,
    ZPP_FIX_BEST_AVAILABLE
};

struct WiperSsrInform : public LocMsg {
    inline WiperSsrInform() : LocMsg() {}
    virtual void proc() const;
};

struct WiperRequest : public LocMsg {
    const OdcpiRequest mRequest;
    inline WiperRequest(const OdcpiRequest &request) :
        LocMsg(), mRequest(request) {}
    virtual void proc() const;
};

struct WiperApDataRequest : public LocMsg {
    const WifiApDataRequest mRequest;
    inline WiperApDataRequest(const WifiApDataRequest &request) :
        LocMsg(), mRequest(request) {}
    virtual void proc() const;
};

struct WiperSrnRequest : public LocMsg {
    const GnssSrnDataReq mSrnEventData;
    inline WiperSrnRequest(const GnssSrnDataReq &srnEvtData) :
        LocMsg(), mSrnEventData(srnEvtData) {}
    virtual void proc() const;
};

struct ZppFixMsg : public LocMsg {
    ZppFixMsg(LocGpsLocation *gpsLocation, LocPosTechMask posTechMask);
    virtual void proc() const;
private:
    LocGpsLocation mGpsLocation;
    LocPosTechMask mPosTechMask;
};

struct WwanFixMsg : public LocMsg {
    WwanFixMsg(LocGpsLocation *gpsLocation);
    virtual void proc() const;
private:
    LocGpsLocation mGpsLocation;
};

struct TimeZoneInfoRequest : public LocMsg {
    inline TimeZoneInfoRequest() : LocMsg() {}
    virtual void proc() const;
};

class LBSAdapter : public LBSAdapterBase {
    static LBSAdapter* mMe;
    virtual void getZppFixSync(enum zpp_fix_type type);
protected:
    inline LBSAdapter(const LOC_API_ADAPTER_EVENT_MASK_T mask) :
        LBSAdapterBase(mask,
            LocDualContext::getLocFgContext(LocDualContext::mLocationHalName)) {
    }
    inline virtual ~LBSAdapter() {}
public:
    static LBSAdapter* get(const LOC_API_ADAPTER_EVENT_MASK_T mask);

    inline virtual bool requestWps(const OdcpiRequest &request) {
        sendMsg(new WiperRequest(request));
        return true;
    }

    inline virtual bool requestWifiApData(const WifiApDataRequest &request) {
        sendMsg(new WiperApDataRequest(request));
        return true;
    }

    inline virtual bool requestSrnData(const GnssSrnDataReq &gnssSrnDataReq) {
        sendMsg(new WiperSrnRequest(gnssSrnDataReq));
        return true;
    }

    inline virtual void handleEngineUpEvent() {
        sendMsg(new WiperSsrInform());
    }

    virtual bool requestTimeZoneInfo();

    int cinfoInject(int cid, int lac, int mnc, int mcc, bool roaming);
    int oosInform();
    int niSuplInit(char* supl_init, int length);
    int chargerStatusInject(int status);
    int wifiStatusInform();
    int wifiEnabledStatusInject(int status);
    int wifiAttachmentStatusInject(WifiSupplicantInfo &wifiSupplicantInfo);
    int injectCoarsePosition(CoarsePositionInfo &cpInfo);
    int injectWifiPosition(WifiLocation &wifiInfo);
    int injectWifiApInfo(WifiApInfo &wifiApInfo);
    int injectBtClassicDevScanData(BtDeviceInfo &btDevInfo);
    int injectBtLeDevScanData(BtDeviceInfo &btDevInfo);
    int setWifiWaitTimeoutValue(int timeout);
    int timeInfoInject(long curTimeMillis, int rawOffset, int dstOffset);

    // Zpp related
    int getZppFixRequest(enum zpp_fix_type type);
    bool reportWwanZppFix(LocGpsLocation &zppLoc);

};

#ifdef __cplusplus
}
#endif

#endif /* LBS_ADAPTER_H */
