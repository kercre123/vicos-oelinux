/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

GENERAL DESCRIPTION
  DataItemSerializerConcreteTypes Implementation

  Copyright (c) 2015-2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/

#include <comdef.h>
#include <DataItemConcreteTypeFieldNames.h>
#include <DataItemSerializerConcreteTypes.h>
#include <DataItemSerializerBase.h>
#include <DataItemDefines.h>
#include <postcard.h>
#include <DataItemId.h>
#include <IDataItem.h>
#include <DataItemConcreteTypes.h>
#include <loc_ril.h>
#include <platform_lib_log_util.h>

using namespace qc_loc_fw;
using namespace izat_manager;


#define DATA_ITEM_SERIALIZER_CTOR(S,CARD) \
S::S(IDataItem * d) : DataItemSerializerBase(d, CARD) {}

#define DATA_ITEM_SERIALIZER_DTOR(S) \
S::~S() {}

#define DATA_ITEM_SERIALIZE(T) \
int32 T::serialize (OutPostcard * oc)

#define BREAK_AND_DELETE_IF_NON_ZERO(ERR,X,OBJ) \
if(0!=(X)) {result = (ERR); delete (OBJ); break;}

#define ADD_INNER_CARD() \
    if (ic != NULL) { \
        BREAK_AND_DELETE_IF_NON_ZERO(100, ic->finalize(), ic); \
        BREAK_AND_DELETE_IF_NON_ZERO(101, oc->addCard(DataItemSerializerBase::getCardName().c_str(), ic), ic); \
    } \

#define CREATE_INNER_CARD(T,ID) \
    if((mD==NULL) || (oc==NULL)) { result = 1; break; } \
    if(mD->getId() != ID) { result = 2; break; } \
    if(DataItemSerializerBase::hasCardName()) { \
        ic = OutPostcard::createInstance(); \
        BREAK_IF_ZERO(3, ic); \
        BREAK_AND_DELETE_IF_NON_ZERO(4, ic->init(), ic); \
        card = ic; \
    } \
    T * d = reinterpret_cast<T *>(mD);



// Ctors
DATA_ITEM_SERIALIZER_CTOR(AirplaneModeDataItemSerializer, AIRPLANEMODE_CARD)
DATA_ITEM_SERIALIZER_CTOR(ENHDataItemSerializer, ENH_CARD)
DATA_ITEM_SERIALIZER_CTOR(GPSStateDataItemSerializer, GPSSTATE_CARD)
DATA_ITEM_SERIALIZER_CTOR(NLPStatusDataItemSerializer, NLPSTATUS_CARD)
DATA_ITEM_SERIALIZER_CTOR(WifiHardwareStateDataItemSerializer, WIFIHARDWARESTATE_CARD)
DATA_ITEM_SERIALIZER_CTOR(ScreenStateDataItemSerializer, SCREENSTATE_CARD)
DATA_ITEM_SERIALIZER_CTOR(PowerConnectStateDataItemSerializer, POWERCONNECTSTATE_CARD)
DATA_ITEM_SERIALIZER_CTOR(TimeZoneChangeDataItemSerializer, TIMEZONECHANGE_CARD)
DATA_ITEM_SERIALIZER_CTOR(TimeChangeDataItemSerializer, TIMEZONECHANGE_CARD)
DATA_ITEM_SERIALIZER_CTOR(ShutdownStateDataItemSerializer, TIMEZONECHANGE_CARD)
DATA_ITEM_SERIALIZER_CTOR(AssistedGpsDataItemSerializer, ASSISTEDGPS_CARD)
DATA_ITEM_SERIALIZER_CTOR(NetworkInfoDataItemSerializer, NETWORKINFO_CARD)
DATA_ITEM_SERIALIZER_CTOR(ServiceStatusDataItemSerializer, SERVICESTATUS_CARD)
DATA_ITEM_SERIALIZER_CTOR(ModelDataItemSerializer, MODEL_CARD)
DATA_ITEM_SERIALIZER_CTOR(ManufacturerDataItemSerializer, MANUFACTURER_CARD)
DATA_ITEM_SERIALIZER_CTOR(RilServiceInfoDataItemSerializer, RILSERVICEINFO_CARD)
DATA_ITEM_SERIALIZER_CTOR(RilCellInfoDataItemSerializer, RILCELLINFO_CARD)
DATA_ITEM_SERIALIZER_CTOR(WifiSupplicantStatusDataItemSerializer, WIFI_SUPPLICANT_STATUS_CARD)
DATA_ITEM_SERIALIZER_CTOR(TacDataItemSerializer, TAC_CARD)
DATA_ITEM_SERIALIZER_CTOR(MccmncDataItemSerializer, MCCMNC_CARD)
DATA_ITEM_SERIALIZER_CTOR(BtLeDeviceScanDetailsDataItemSerializer, BTLESCANDETAILS_CARD)
DATA_ITEM_SERIALIZER_CTOR(BtDeviceScanDetailsDataItemSerializer, BTSCANDETAILS_CARD)

// Dtors
DATA_ITEM_SERIALIZER_DTOR(AirplaneModeDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(ENHDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(GPSStateDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(NLPStatusDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(WifiHardwareStateDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(ScreenStateDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(PowerConnectStateDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(TimeZoneChangeDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(TimeChangeDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(ShutdownStateDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(AssistedGpsDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(NetworkInfoDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(ServiceStatusDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(ModelDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(ManufacturerDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(RilServiceInfoDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(RilCellInfoDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(WifiSupplicantStatusDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(TacDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(MccmncDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(BtLeDeviceScanDetailsDataItemSerializer)
DATA_ITEM_SERIALIZER_DTOR(BtDeviceScanDetailsDataItemSerializer)

// serialize
DATA_ITEM_SERIALIZE(AirplaneModeDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(AirplaneModeDataItem, AIRPLANEMODE_DATA_ITEM_ID);
        BREAK_AND_DELETE_IF_NON_ZERO(
                5, card->addBool(AIRPLANEMODE_FIELD_MODE, d->mMode), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS), ic);
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}
DATA_ITEM_SERIALIZE(ENHDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(ENHDataItem, ENH_DATA_ITEM_ID);
        BREAK_AND_DELETE_IF_NON_ZERO(
                5, card->addBool(ENH_FIELD_ENABLED, d->mEnabled), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS), ic);
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}
DATA_ITEM_SERIALIZE(GPSStateDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(GPSStateDataItem, GPSSTATE_DATA_ITEM_ID);
        BREAK_AND_DELETE_IF_NON_ZERO(
                5, card->addBool(GPSSTATE_FIELD_ENABLED, d->mEnabled), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS), ic);
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}
DATA_ITEM_SERIALIZE(NLPStatusDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(NLPStatusDataItem, NLPSTATUS_DATA_ITEM_ID);
        BREAK_AND_DELETE_IF_NON_ZERO(
                5, card->addBool(NLPSTATUS_FIELD_ENABLED, d->mEnabled), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS), ic);
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}
DATA_ITEM_SERIALIZE(WifiHardwareStateDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(WifiHardwareStateDataItem, WIFIHARDWARESTATE_DATA_ITEM_ID);
        BREAK_AND_DELETE_IF_NON_ZERO(
                5, card->addBool(WIFIHARDWARESTATE_FIELD_ENABLED, d->mEnabled), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS), ic);
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}

DATA_ITEM_SERIALIZE(ScreenStateDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(ScreenStateDataItem, SCREEN_STATE_DATA_ITEM_ID);
        BREAK_AND_DELETE_IF_NON_ZERO(
                5, card->addBool(SCREENSTATE_FIELD_ENABLED, d->mState), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS), ic);
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}

DATA_ITEM_SERIALIZE(PowerConnectStateDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(PowerConnectStateDataItem, POWER_CONNECTED_STATE_DATA_ITEM_ID);
        BREAK_AND_DELETE_IF_NON_ZERO(
                5, card->addBool(POWERCONNECTSTATE_FIELD_ENABLED, d->mState), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS), ic);
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}

DATA_ITEM_SERIALIZE(TimeZoneChangeDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(TimeZoneChangeDataItem, TIMEZONE_CHANGE_DATA_ITEM_ID);
        BREAK_AND_DELETE_IF_NON_ZERO(
                5, card->addBool(TIMEZONECHANGE_FIELD_ENABLED, true), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                7, card->addInt64(TIMECHANGE_FIELD_CURRENT_TIME_MILLIS, d->mCurrTimeMillis), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                8, card->addInt64(TIMECHANGE_FIELD_RAW_OFFSET_TZ, d->mRawOffsetTZ), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                9, card->addInt64(TIMECHANGE_FIELD_DST_OFFSET_TZ, d->mDstOffsetTZ), ic);
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}

DATA_ITEM_SERIALIZE(TimeChangeDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(TimeChangeDataItem, TIME_CHANGE_DATA_ITEM_ID);
        BREAK_AND_DELETE_IF_NON_ZERO(
                5, card->addBool(TIMECHANGE_FIELD_ENABLED, true), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                7, card->addInt64(TIMECHANGE_FIELD_CURRENT_TIME_MILLIS, d->mCurrTimeMillis), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                8, card->addInt64(TIMECHANGE_FIELD_RAW_OFFSET_TZ, d->mRawOffsetTZ), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                9, card->addInt64(TIMECHANGE_FIELD_DST_OFFSET_TZ, d->mDstOffsetTZ), ic);
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}

DATA_ITEM_SERIALIZE(ShutdownStateDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(ShutdownStateDataItem, SHUTDOWN_STATE_DATA_ITEM_ID);
        BREAK_AND_DELETE_IF_NON_ZERO(
                5, card->addBool(SHUTDOWN_FIELD_ENABLED, d->mState), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS), ic);
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}

DATA_ITEM_SERIALIZE(AssistedGpsDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(AssistedGpsDataItem, ASSISTED_GPS_DATA_ITEM_ID);
        BREAK_AND_DELETE_IF_NON_ZERO(
                5, card->addBool(ASSISTEDGPS_FIELD_ENABLED, d->mEnabled), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS), ic);
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}

DATA_ITEM_SERIALIZE(NetworkInfoDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(NetworkInfoDataItem, NETWORKINFO_DATA_ITEM_ID);
        BREAK_AND_DELETE_IF_NON_ZERO(
                5, oc->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, oc->addBool(NETWORKINFO_FIELD_CONNECTED, d->mConnected), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                7, card->addInt32(NETWORKINFO_FIELD_TYPE, d->mType), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                8, card->addString(NETWORKINFO_FIELD_TYPENAME, d->mTypeName.c_str()), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                9, card->addString(NETWORKINFO_FIELD_SUBTYPENAME, d->mSubTypeName.c_str()), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                10, card->addBool(NETWORKINFO_FIELD_AVAILABLE, d->mAvailable), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                11, card->addBool(NETWORKINFO_FIELD_CONNECTED, d->mConnected), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                12, card->addBool(NETWORKINFO_FIELD_ROAMING, d->mRoaming), ic);
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}
DATA_ITEM_SERIALIZE(ServiceStatusDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(ServiceStatusDataItem, SERVICESTATUS_DATA_ITEM_ID);
        if (d->mServiceState == LOC_NW_ROAMING) {
            BREAK_AND_DELETE_IF_NON_ZERO(
                    5, card->addString(SERVICESTATUS_FIELD_STATE, "ROAMING"), ic);
        } else if (d->mServiceState == LOC_NW_HOME) {
            BREAK_AND_DELETE_IF_NON_ZERO(
                    6, card->addString(SERVICESTATUS_FIELD_STATE, "HOME"), ic);
        } else {
            BREAK_AND_DELETE_IF_NON_ZERO(
                    7, card->addString(SERVICESTATUS_FIELD_STATE, "OOO"), ic);
        }
        BREAK_AND_DELETE_IF_NON_ZERO(
                8, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS), ic);
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}
DATA_ITEM_SERIALIZE(ModelDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(ModelDataItem, MODEL_DATA_ITEM_ID);
        BREAK_AND_DELETE_IF_NON_ZERO(
                5, card->addString(MODEL_FIELD_NAME, d->mModel.c_str()), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS), ic);
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}
DATA_ITEM_SERIALIZE(ManufacturerDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(ManufacturerDataItem, MANUFACTURER_DATA_ITEM_ID);
        BREAK_AND_DELETE_IF_NON_ZERO(
                5, card->addString(MANUFACTURER_FIELD_NAME, d->mManufacturer.c_str()), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS), ic);
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}
DATA_ITEM_SERIALIZE(RilServiceInfoDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        BREAK_IF_NON_ZERO(5, oc->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_RIL));
        CREATE_INNER_CARD(RilServiceInfoDataItem, RILSERVICEINFO_DATA_ITEM_ID);
        // check if the data we received is valid
        if(d->mServiceInfo->valid_mask) {
            if((d->mServiceInfo->valid_mask & LOC_RIL_SERVICE_INFO_HAS_AIR_IF_TYPE) ==
                LOC_RIL_SERVICE_INFO_HAS_AIR_IF_TYPE) {
                BREAK_AND_DELETE_IF_NON_ZERO(
                        6, card->addUInt32(
                                RILSERVICEINFO_FIELD_ARIF_TYPE_MASK, d->mServiceInfo->airIf_type), ic);
            }
            if((d->mServiceInfo->valid_mask & LOC_RIL_SERVICE_INFO_HAS_CARRIER_AIR_IF_TYPE) ==
                LOC_RIL_SERVICE_INFO_HAS_CARRIER_AIR_IF_TYPE) {
                if (d->mServiceInfo->carrierAirIf_type == LOC_RILAIRIF_GSM) {
                    BREAK_AND_DELETE_IF_NON_ZERO(
                            7, card->addString(RILSERVICEINFO_FIELD_CARRIER_ARIF_TYPE, "GSM"), ic);
                } else if (d->mServiceInfo->carrierAirIf_type == LOC_RILAIRIF_CDMA) {
                    BREAK_AND_DELETE_IF_NON_ZERO(
                            8, card->addString(RILSERVICEINFO_FIELD_CARRIER_ARIF_TYPE, "CDMA"), ic);
                } else if (d->mServiceInfo->carrierAirIf_type == LOC_RILAIRIF_WCDMA) {
                    BREAK_AND_DELETE_IF_NON_ZERO(
                            9, card->addString(RILSERVICEINFO_FIELD_CARRIER_ARIF_TYPE, "WCDMA"), ic);
                } else if (d->mServiceInfo->carrierAirIf_type == LOC_RILAIRIF_LTE) {
                    BREAK_AND_DELETE_IF_NON_ZERO(
                            10, card->addString(RILSERVICEINFO_FIELD_CARRIER_ARIF_TYPE, "LTE"), ic);
                } else if (d->mServiceInfo->carrierAirIf_type == LOC_RILAIRIF_EVDO) {
                    BREAK_AND_DELETE_IF_NON_ZERO(
                            11, card->addString(RILSERVICEINFO_FIELD_CARRIER_ARIF_TYPE, "EVDO"), ic);
                } else if (d->mServiceInfo->carrierAirIf_type == LOC_RILAIRIF_WIFI) {
                    BREAK_AND_DELETE_IF_NON_ZERO(
                            12, card->addString(RILSERVICEINFO_FIELD_CARRIER_ARIF_TYPE, "WIFI"), ic);
                }
            }
            if((d->mServiceInfo->valid_mask & LOC_RIL_SERVICE_INFO_HAS_CARRIER_MCC) ==
                LOC_RIL_SERVICE_INFO_HAS_CARRIER_MCC) {
                BREAK_AND_DELETE_IF_NON_ZERO(
                        13, card->addUInt16(RILSERVICEINFO_FIELD_CARRIER_MCC, d->mServiceInfo->carrierMcc), ic);
            }
            if((d->mServiceInfo->valid_mask & LOC_RIL_SERVICE_INFO_HAS_CARRIER_MNC) ==
                LOC_RIL_SERVICE_INFO_HAS_CARRIER_MNC) {
                BREAK_AND_DELETE_IF_NON_ZERO(
                        14, card->addUInt16(RILSERVICEINFO_FIELD_CARRIER_MNC, d->mServiceInfo->carrierMnc), ic);
            }
            if((d->mServiceInfo->valid_mask & LOC_RIL_SERVICE_INFO_HAS_CARRIER_NAME) ==
                LOC_RIL_SERVICE_INFO_HAS_CARRIER_NAME) {
                BREAK_AND_DELETE_IF_NON_ZERO(
                        15, card->addString(RILSERVICEINFO_FIELD_CARRIER_NAME, d->mServiceInfo->carrierName), ic);
            }
        }
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d", result);
    return result;
}
DATA_ITEM_SERIALIZE(RilCellInfoDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        BREAK_IF_NON_ZERO(5, oc->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_RIL));
        CREATE_INNER_CARD(RilCellInfoDataItem, RILCELLINFO_DATA_ITEM_ID);
        // check if the data we received is valid
        if(d->mCellInfo->valid_mask) {
            if((d->mCellInfo->valid_mask & LOC_RIL_CELLINFO_HAS_NW_STATUS) ==
                LOC_RIL_CELLINFO_HAS_NW_STATUS) {
                if (d->mCellInfo->nwStatus == LOC_NW_ROAMING) {
                    BREAK_AND_DELETE_IF_NON_ZERO(
                            6, card->addString(RILCELLINFO_FIELD_NETWORK_STATUS, "ROAMING"), ic);
                } else if (d->mCellInfo->nwStatus == LOC_NW_HOME) {
                    BREAK_AND_DELETE_IF_NON_ZERO(
                            7, card->addString(RILCELLINFO_FIELD_NETWORK_STATUS, "HOME"), ic);
                } else {
                    BREAK_AND_DELETE_IF_NON_ZERO(
                            8, card->addString(RILCELLINFO_FIELD_NETWORK_STATUS, "OOO"), ic);
                }
            }
            if((d->mCellInfo->valid_mask & LOC_RIL_CELLINFO_HAS_CELL_INFO) ==
                LOC_RIL_CELLINFO_HAS_CELL_INFO) {
                if((d->mCellInfo->valid_mask & LOC_RIL_CELLINFO_HAS_TECH_TYPE) ==
                    LOC_RIL_CELLINFO_HAS_TECH_TYPE) {
                    switch(d->mCellInfo->rtType) {
                        case LOC_RIL_TECH_CDMA:
                        BREAK_AND_DELETE_IF_NON_ZERO(
                                9, card->addString(RILCELLINFO_FIELD_RIL_TECH_TYPE, "CDMA"), ic);
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_CDMA_HAS_MCC) == LOC_RIL_TECH_CDMA_HAS_MCC) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    10, card->addUInt16(RILLCELLINFO_FIELD_MCC, d->mCellInfo->u.cdmaCinfo.mcc), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_CDMA_HAS_SID) == LOC_RIL_TECH_CDMA_HAS_SID) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    11, card->addUInt16(RILLCELLINFO_FIELD_SID, d->mCellInfo->u.cdmaCinfo.sid), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_CDMA_HAS_NID) == LOC_RIL_TECH_CDMA_HAS_NID) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    12, card->addUInt16(RILLCELLINFO_FIELD_NID, d->mCellInfo->u.cdmaCinfo.nid), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_CDMA_HAS_BSID) == LOC_RIL_TECH_CDMA_HAS_BSID) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    13, card->addUInt32(RILLCELLINFO_FIELD_BSID, d->mCellInfo->u.cdmaCinfo.bsid), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_CDMA_HAS_BSLAT) == LOC_RIL_TECH_CDMA_HAS_BSLAT) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    14, card->addInt32(RILLCELLINFO_FIELD_BSLAT,d->mCellInfo->u.cdmaCinfo.bslat), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_CDMA_HAS_BSLONG) == LOC_RIL_TECH_CDMA_HAS_BSLONG) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    15, card->addInt32(RILLCELLINFO_FIELD_BSLON, d->mCellInfo->u.cdmaCinfo.bslon), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_CDMA_HAS_TIMEZONE) == LOC_RIL_TECH_CDMA_HAS_TIMEZONE) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    16, card->addInt32(RILLCELLINFO_FIELD_UTC_TIME_OFFSET,
                                            d->mCellInfo->u.cdmaCinfo.local_timezone_offset_from_utc), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_CDMA_HAS_DAYLIGHT_SAVING) == LOC_RIL_TECH_CDMA_HAS_DAYLIGHT_SAVING) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    17, card->addBool(RILLCELLINFO_FIELD_DAYLIGHT_TIMEZONE,
                                            d->mCellInfo->u.cdmaCinfo.local_timezone_on_daylight_savings), ic);
                        }
                        break;
                        case LOC_RIL_TECH_GSM:
                        BREAK_AND_DELETE_IF_NON_ZERO(
                                18, card->addString(RILCELLINFO_FIELD_RIL_TECH_TYPE, "GSM"), ic);
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_GW_HAS_MCC) == LOC_RIL_TECH_GW_HAS_MCC) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    19, card->addUInt16(RILLCELLINFO_FIELD_MCC, d->mCellInfo->u.gsmCinfo.mcc), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_GW_HAS_MNC) == LOC_RIL_TECH_GW_HAS_MNC) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    20, card->addUInt16(RILLCELLINFO_FIELD_MNC, d->mCellInfo->u.gsmCinfo.mnc), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_GW_HAS_LAC) == LOC_RIL_TECH_GW_HAS_LAC) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    21, card->addUInt16(RILLCELLINFO_FIELD_LAC, d->mCellInfo->u.gsmCinfo.lac), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_GW_HAS_CID) == LOC_RIL_TECH_GW_HAS_CID) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    22, card->addUInt32(RILLCELLINFO_FIELD_CID, d->mCellInfo->u.gsmCinfo.cid), ic);
                        }
                        break;
                        case LOC_RIL_TECH_WCDMA:
                        BREAK_AND_DELETE_IF_NON_ZERO(
                                23, card->addString(RILCELLINFO_FIELD_RIL_TECH_TYPE, "WCDMA"), ic);
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_GW_HAS_MCC) == LOC_RIL_TECH_GW_HAS_MCC) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    24, card->addUInt16(RILLCELLINFO_FIELD_MCC, d->mCellInfo->u.wcdmaCinfo.mcc), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_GW_HAS_MNC) == LOC_RIL_TECH_GW_HAS_MNC) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    25, card->addUInt16(RILLCELLINFO_FIELD_MNC, d->mCellInfo->u.wcdmaCinfo.mnc), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_GW_HAS_LAC) == LOC_RIL_TECH_GW_HAS_LAC) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    26, card->addUInt16(RILLCELLINFO_FIELD_LAC, d->mCellInfo->u.wcdmaCinfo.lac), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_GW_HAS_CID) == LOC_RIL_TECH_GW_HAS_CID) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    27, card->addUInt32(RILLCELLINFO_FIELD_CID, d->mCellInfo->u.wcdmaCinfo.cid), ic);
                        }
                        break;
                        case LOC_RIL_TECH_LTE:
                        BREAK_AND_DELETE_IF_NON_ZERO(
                                28, card->addString(RILCELLINFO_FIELD_RIL_TECH_TYPE, "LTE"), ic);
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_LTE_HAS_MCC) == LOC_RIL_TECH_LTE_HAS_MCC) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    29, card->addUInt16(RILLCELLINFO_FIELD_MCC, d->mCellInfo->u.lteCinfo.mcc), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_LTE_HAS_MNC) == LOC_RIL_TECH_LTE_HAS_MNC) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    30, card->addUInt16(RILLCELLINFO_FIELD_MNC, d->mCellInfo->u.lteCinfo.mnc), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_LTE_HAS_TAC) == LOC_RIL_TECH_LTE_HAS_TAC) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    31, card->addUInt16(RILLCELLINFO_FIELD_TAC, d->mCellInfo->u.lteCinfo.tac), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_LTE_HAS_PCI) == LOC_RIL_TECH_LTE_HAS_PCI) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    32, card->addUInt16(RILLCELLINFO_FIELD_PCI, d->mCellInfo->u.lteCinfo.pci), ic);
                        }
                        if((d->mCellInfo->valid_mask & LOC_RIL_TECH_LTE_HAS_CID) == LOC_RIL_TECH_LTE_HAS_CID) {
                            BREAK_AND_DELETE_IF_NON_ZERO(
                                    33, card->addUInt32(RILLCELLINFO_FIELD_CID, d->mCellInfo->u.lteCinfo.cid), ic);
                        }
                        break;
                        case LOC_RIL_TECH_TD_SCDMA:
                        case LOC_RIL_TECH_MAX:
                        default:
                        break;
                    }; // end switch
                }
            }
        }
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d", result);
    return result;
}

DATA_ITEM_SERIALIZE(WifiSupplicantStatusDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(WifiSupplicantStatusDataItem, WIFI_SUPPLICANT_STATUS_DATA_ITEM_ID);
        BREAK_AND_DELETE_IF_NON_ZERO(
                5, card->addInt32(WIFI_SUPPLICANT_FIELD_STATE, d->mState), ic);
        // TODO: serialize remaining fields if required
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS), ic);
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}
DATA_ITEM_SERIALIZE(TacDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(TacDataItem, TAC_DATA_ITEM_ID);
        BREAK_IF_NON_ZERO(5, card->addString(TAC_FIELD_NAME, d->mValue.c_str()));
        BREAK_IF_NON_ZERO(6, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS));
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}
DATA_ITEM_SERIALIZE(MccmncDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(MccmncDataItem, MCCMNC_DATA_ITEM_ID);
        BREAK_IF_NON_ZERO(5, card->addString(MCCMNC_FIELD_NAME, d->mValue.c_str()));
        BREAK_IF_NON_ZERO(6, card->addString(RESPONSE_FIELD_NAME, RESPONSE_FIELD_VALUE_OS));
        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}
DATA_ITEM_SERIALIZE(BtLeDeviceScanDetailsDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(BtLeDeviceScanDetailsDataItem, BTLE_SCAN_DATA_ITEM_ID);

        BREAK_AND_DELETE_IF_NON_ZERO(
                5, card->addBool(BTLESCANDETAILS_FIELD_VALID, d->mValidSrnData), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, card->addInt32(BTLESCANDETAILS_FIELD_RSSI, d->mApSrnRssi), ic);
        const void* blob = (const void*) d->mApSrnMacAddress;
        BREAK_AND_DELETE_IF_NON_ZERO(
                7, card->addBlob(BTLESCANDETAILS_FIELD_MAC, blob, SRN_MAC_ADDRESS_LENGTH), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                8, card->addInt64(BTLESCANDETAILS_FIELD_SCANREQ, d->mApSrnTimestamp), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                9, card->addInt64(BTLESCANDETAILS_FIELD_SCANSTART, d->mRequestTimestamp), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                10, card->addInt64(BTLESCANDETAILS_FIELD_SCANRECV, d->mReceiveTimestamp), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                11, card->addInt32(BTLESCANDETAILS_FIELD_SCANERROR, d->mErrorCause), ic);

        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}
DATA_ITEM_SERIALIZE(BtDeviceScanDetailsDataItemSerializer) {
    int32 result = 0;
    OutPostcard * card = oc;
    OutPostcard * ic = NULL;
    ENTRY_LOG();
    do {
        CREATE_INNER_CARD(BtDeviceScanDetailsDataItem, BT_SCAN_DATA_ITEM_ID);

        BREAK_AND_DELETE_IF_NON_ZERO(
                5, card->addBool(BTSCANDETAILS_FIELD_VALID, d->mValidSrnData), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                6, card->addInt32(BTSCANDETAILS_FIELD_RSSI, d->mApSrnRssi), ic);
        const void* blob = (const void*) d->mApSrnMacAddress;
        BREAK_AND_DELETE_IF_NON_ZERO(
                7, card->addBlob(BTSCANDETAILS_FIELD_MAC, blob, SRN_MAC_ADDRESS_LENGTH), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                8, card->addInt64(BTSCANDETAILS_FIELD_SCANREQ, d->mApSrnTimestamp), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                9, card->addInt64(BTSCANDETAILS_FIELD_SCANSTART, d->mRequestTimestamp), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                10, card->addInt64(BTSCANDETAILS_FIELD_SCANRECV, d->mReceiveTimestamp), ic);
        BREAK_AND_DELETE_IF_NON_ZERO(
                11, card->addInt32(BTSCANDETAILS_FIELD_SCANERROR, d->mErrorCause), ic);

        ADD_INNER_CARD();
    } while(0);
    EXIT_LOG_WITH_ERROR("%d",result);
    return result;
}

