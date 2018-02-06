/**
 * Copyright (c) 2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
**/

#ifndef __VEHICLE_NETWORK_SERVICES_LISTENER_H_INCLUDED__
#define __VEHICLE_NETWORK_SERVICES_LISTENER_H_INCLUDED__

#include <VehicleNetwork.h>
#include <VehicleNetworkDataTypes.h>

using namespace android;
namespace slim
{

    class VNSListener : public VehicleNetworkListener
    {
     public:
        VNSListener() {};
        virtual ~VNSListener() {};
        virtual void onEvents(sp<VehiclePropValueListHolder>& events);
        virtual void onHalError(int32_t errorCode, int32_t property, int32_t operation);
        virtual void onHalRestart(bool inMocking);
        virtual void onPropertySet(const vehicle_prop_value_t& value);
      private:
        Mutex mLock;
    };


    typedef struct VNS_Event_Struct_t
    {
        int propertyType;
        int64_t timeStamp;
        int gear;
        float speed;
    }VNS_Event_Struct;

}
#endif /*  __VEHICLE_NETWORK_SERVICES_LISTENER_H_INCLUDED__ */
