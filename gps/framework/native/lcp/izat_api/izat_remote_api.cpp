/*=============================================================================
  Copyright (c) 2016-2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
  =============================================================================*/

#include <izat_remote_api.h>
#include <IzatRemoteApi.h>
#include <IzatTypes.h>
#include <string>
#include "izat_wifi_db_receiver.h"
#include "IzatWiFiDBReceiver.h"

using namespace izat_remote_api;
using namespace izat_manager;
// ======== LocationUpdater ========
class LocationUpdaterWrapper {
    const remoteClientInfo *mClientInfo;
    const void* mClientData;
    LocationUpdater *pLocUpdater = NULL;
    SvInfoUpdater *pSvInfoUpdater = NULL;
public:
    inline LocationUpdaterWrapper(remoteClientInfo *pClientInfo, void* clientData) :
                                  mClientInfo(pClientInfo), mClientData(clientData) {

        if(pClientInfo->locCb){
            pLocUpdater = new LocationUpdater(pClientInfo, clientData);
        }

        if(pClientInfo->svReportCb){
            pSvInfoUpdater = new SvInfoUpdater(pClientInfo, clientData);
        }
    }

    ~LocationUpdaterWrapper(){
        if (pLocUpdater)
            delete pLocUpdater;

        if (pSvInfoUpdater)
            delete pSvInfoUpdater;
    }
};

// ======== RawLocationUpdater ========
class RawLocationUpdaterWrapper {
    const remoteClientInfo *mClientInfo;
    const void* mClientData;
    RawLocationUpdater *pRawLocUpdater = NULL;
    RawSvInfoUpdater *pRawSvInfoUpdater = NULL;
public:
    inline RawLocationUpdaterWrapper(remoteClientInfo *pClientInfo, void* clientData) :
                                     mClientInfo(pClientInfo), mClientData(clientData) {

        if (pClientInfo->locCb){
            pRawLocUpdater = new RawLocationUpdater(pClientInfo, clientData);
        }
        if (pClientInfo->svReportCb){
            pRawSvInfoUpdater = new RawSvInfoUpdater(pClientInfo, clientData);
        }
    }

    ~RawLocationUpdaterWrapper(){
        if (pRawLocUpdater)
            delete pRawLocUpdater;

        if (pRawSvInfoUpdater)
            delete pRawSvInfoUpdater;
    }
};

void* registerLocationUpdater(remoteClientInfo *pClientInfo, void* clientData) {
    return (pClientInfo && clientData) ? new LocationUpdaterWrapper(pClientInfo, clientData) : NULL;
}

void unregisterLocationUpdater(void* locationUpdaterHandle) {
    if (locationUpdaterHandle) {
        delete (LocationUpdaterWrapper*)locationUpdaterHandle;
    }
}

void* registerRawLocationUpdater(remoteClientInfo *pClientInfo, void* clientData) {
    return (pClientInfo && clientData) ? new RawLocationUpdaterWrapper(pClientInfo, clientData) : NULL;
}

void unregisterRawLocationUpdater(void* locationUpdaterHandle) {
    if (locationUpdaterHandle) {
        delete (RawLocationUpdaterWrapper*)locationUpdaterHandle;
    }
}

// ======== SstpUpdater ========
class SstpUpdaterWrapper : public SstpUpdater {
    const sstpSiteUpdateCb mSiteCb;
    const sstpMccUpdateCb mMccCb;
    const errReportCb mErrCb;
    const void* mClientData;
public:
    inline SstpUpdaterWrapper(sstpSiteUpdateCb siteCb, sstpMccUpdateCb mccCb,
                              errReportCb errCb, void* clientData) :
        SstpUpdater(), mSiteCb(siteCb), mMccCb(mccCb), mErrCb(errCb),
        mClientData(clientData) {
    }
    inline virtual void errReport(const char* errStr) override {
        if (mErrCb != nullptr) mErrCb(errStr, (void*)mClientData);
    }
    inline virtual void siteUpdate(const char* name, double lat, double lon,
                                   float unc, int32_t uncConfidence) override {
        mSiteCb(name, lat, lon, unc, uncConfidence, (void*)mClientData);
    }
    inline virtual void mccUpdate(uint32_t mcc, const char* confidence) override {
        mMccCb(mcc, confidence, (void*)mClientData);
    }
};

void* registerSstpUpdater(sstpSiteUpdateCb siteCb, sstpMccUpdateCb mccCb,
                          errReportCb errCb, void* clientData) {
    return (siteCb && mccCb) ?
        new SstpUpdaterWrapper(siteCb, mccCb, errCb, clientData) :
        NULL;
}

void unregisterSstpUpdater(void* sstpUpdaterHandle) {
    if (sstpUpdaterHandle) {
        delete (SstpUpdaterWrapper*)sstpUpdaterHandle;
    }
}

void stopSstpUpdate(void* sstpUpdaterHandle) {
    if (sstpUpdaterHandle) {
        ((SstpUpdaterWrapper*)sstpUpdaterHandle)->stop();
    }
}

// ======== WiFiDBUpdater ========
class WiFiDBUpdaterWrapper : public WiFiDBUpdater {
    const wifiDBRecvApListUpdateCb mApListAvailCb;
    const wifiDBRecvStatusUpdateCb mStatusCb;
    const wifiDBRecvServiceRequestCb mServiceReqCb;
    const wifiDBRecvEnvNotifyCb mEnvNotifyCb;
    const void *mClientData;
public:
    inline WiFiDBUpdaterWrapper(wifiDBRecvApListUpdateCb apListAvailCb,
                                wifiDBRecvStatusUpdateCb statusCb,
                                wifiDBRecvServiceRequestCb serviceReqCb,
                                wifiDBRecvEnvNotifyCb envNotifyCb,
                                void *clientData) :
        WiFiDBUpdater(),
        mApListAvailCb(apListAvailCb),
        mStatusCb(statusCb),
        mServiceReqCb(serviceReqCb),
        mEnvNotifyCb(envNotifyCb),
        mClientData(clientData) {
    }
    inline virtual void statusUpdate(bool isSuccess, const char *errStr) override {
        mStatusCb(isSuccess, errStr, (void *)mClientData);
    }
    inline virtual void apListUpdate(std::vector<APInfo>* ap_list_ptr) override {
      if (nullptr != ap_list_ptr) {
        bool completeList = true;
        std::vector<APInfo>& ap_list = *ap_list_ptr;
        APInfo_s **ap_list_arr = new APInfo_s *[ap_list.size()];
        if (ap_list_arr!=NULL)
        {
          for (size_t ii = 0; ii < ap_list.size(); ++ii) {
            ap_list_arr[ii] = new APInfo_s;
            if (ap_list_arr[ii]!=NULL)
            {
              ap_list_arr[ii]->mac_R48b = ap_list[ii].mac_R48b;
              ap_list_arr[ii]->cell_type = ap_list[ii].cellType;
              ap_list_arr[ii]->cell_id1 = ap_list[ii].cellRegionID1;
              ap_list_arr[ii]->cell_id2 = ap_list[ii].cellRegionID2;
              ap_list_arr[ii]->cell_id3 = ap_list[ii].cellRegionID3;
              ap_list_arr[ii]->cell_id3 = ap_list[ii].cellRegionID4;
              memcpy(ap_list_arr[ii]->ssid, ap_list[ii].ssid,
                     sizeof(ap_list_arr[ii]->ssid));
              ap_list_arr[ii]->ssid_valid_byte_count = ap_list[ii].ssid_valid_byte_count;
            }
            else
            {
              completeList=false;
              break;
            }
          }
          if (true == completeList)
          {
            mApListAvailCb(ap_list_arr, ap_list.size(), (void *)mClientData);
          }
          for (size_t ii = 0; ii < ap_list.size(); ++ii) {
            if (ap_list_arr[ii]!=NULL)
            {
              delete ap_list_arr[ii];
            }
          }
          delete[] ap_list_arr;
        }
      }
    }
    inline virtual void serviceRequest() override {
        mServiceReqCb((void *)mClientData);
    }
    inline virtual void notifyCallbackEnv() override {
        if (nullptr != mEnvNotifyCb) {
            mEnvNotifyCb((void *)mClientData);
        }
    }
};

void* registerWiFiDBUpdater(wifiDBRecvApListUpdateCb apListAvailCb,
                            wifiDBRecvStatusUpdateCb statusCb,
                            wifiDBRecvServiceRequestCb serviceReqCb,
                            wifiDBRecvEnvNotifyCb envNotifyCb,
                            void* clientData) {
    return (statusCb && apListAvailCb && serviceReqCb) ?
           new WiFiDBUpdaterWrapper(apListAvailCb, statusCb, serviceReqCb, envNotifyCb, clientData) :
           NULL;
}

void unregisterWiFiDBUpdater(void* WiFiDBUpdaterHandle) {
    if (WiFiDBUpdaterHandle) {
        delete (WiFiDBUpdaterWrapper*)WiFiDBUpdaterHandle;
    }
}

void sendAPListRequest(int expire_in_days, void* WiFiDBUpdaterHandle) {
    if (WiFiDBUpdaterHandle) {
        ((WiFiDBUpdaterWrapper *)WiFiDBUpdaterHandle)->sendAPListReq(expire_in_days);
    }
}

void pushWiFiDB(APLocationData_s *ap_loc_list[], size_t ap_loc_list_size,
                APSpecialInfo_s* ap_spl_list[], size_t ap_spl_list_size,
                int days_valid, void* WiFiDBUpdaterHandle) {
    if (WiFiDBUpdaterHandle) {
        std::vector<WiFiDBUpdater::APLocationData> loc_list;
        std::vector<WiFiDBUpdater::APSpecialInfo> ap_list;

        for (size_t ii = 0; ii < ap_loc_list_size; ii++) {
            WiFiDBUpdater::APLocationData l_ad;
            l_ad.mac_R48b = ap_loc_list[ii]->mac_R48b;
            l_ad.latitude = ap_loc_list[ii]->latitude;
            l_ad.longitude = ap_loc_list[ii]->longitude;
            l_ad.max_antena_range = ap_loc_list[ii]->max_antena_range;
            l_ad.horizontal_error = ap_loc_list[ii]->horizontal_error;
            l_ad.reliability = ap_loc_list[ii]->reliability;
            l_ad.valid_mask = ap_loc_list[ii]->valid_bits;
            loc_list.push_back(l_ad);
        }
        for (size_t ii = 0; ii < ap_spl_list_size; ii++) {
            WiFiDBUpdater::APSpecialInfo s_ad;
            s_ad.mac_R48b = ap_spl_list[ii]->mac_R48b;
            s_ad.info = ap_spl_list[ii]->info;
            ap_list.push_back(s_ad);
        }
        ((WiFiDBUpdaterWrapper*)WiFiDBUpdaterHandle)->pushWiFiDB(&loc_list,
                                                                 &ap_list,
                                                                 days_valid);
    }
}
