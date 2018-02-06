/**
 * Copyright (c) 2016 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
**/

#include <hardware/vehicle.h>
#include <slim_os_log_api.h>
#include <slim_os_api.h>
#include <VNSProvider.h>
#include "VNSListener.h"

using namespace slim;

void VNSListener::onEvents(sp<VehiclePropValueListHolder>& events)
{
    /* Handle SPEED , GEAR (current[for non-mannual case] & Selected) , DWS (not available should   declare as custom property ), ODO events */
    /* create IPC message and post to local thread */
    MutexLock _l(mLock);
    for (auto& e : events->getList())
    {
        switch ( e->prop )
        {
            case VEHICLE_PROPERTY_PERF_VEHICLE_SPEED:
            {
                SLIM_LOGD("New Speed event received. id:%u, Val:%f , Time:%u ", (unsigned int )e->prop,
                    e->value.vehicle_speed,(unsigned int)(e->timestamp/1000000));

                /* create new message data */
                VNS_Event_Struct *pzNewEvent = new VNS_Event_Struct;
                memset(pzNewEvent , 0, sizeof(VNS_Event_Struct));
                pzNewEvent->propertyType = e->prop;
                pzNewEvent->speed        = e->value.vehicle_speed;
                pzNewEvent->timeStamp    = e->timestamp/1000000;
                /* send message to event processing loop */
                if (!slim_IpcSendData(THREAD_ID_VNS, eIPC_MSG_NEW_VNS_EVENT, &pzNewEvent,
                  sizeof(pzNewEvent)))
                {
                    SLIM_LOGE("Error sending IPC message to event processing loop");
                }
            }
            break;
            case VEHICLE_PROPERTY_CURRENT_GEAR:
            {
                SLIM_LOGD("New Gear event received. id:%u, Val:%d, Time:%d", (unsigned int )e->prop,
                    e->value.gear_current_gear,e->timestamp);

                /* create new message data */
                VNS_Event_Struct *pzNewEvent = new VNS_Event_Struct;
                memset(pzNewEvent , 0, sizeof(VNS_Event_Struct));
                pzNewEvent->propertyType = e->prop;
                pzNewEvent->gear         = e->value.gear_current_gear;
                pzNewEvent->timeStamp    = e->timestamp;
                /* send message to event processing loop */
                if (!slim_IpcSendData(THREAD_ID_VNS, eIPC_MSG_NEW_VNS_EVENT, &pzNewEvent,
                  sizeof(pzNewEvent)))
                {
                    SLIM_LOGE("Error sending gear IPC message to event processing loop");
                }
            }
            break;
            default:
                SLIM_LOGD("Unhandled Event received. id:%u", (unsigned int )e->prop);
            break;
        }
    }
}

void VNSListener::onHalError(int32_t errorCode, int32_t property , int32_t operation)
{
    SLIM_LOGD("VNSListener::onHalError code:%u  property:%d operation:%d", errorCode,property,operation);
}

void VNSListener::onHalRestart(bool inMocking)
{
    SLIM_LOGD("VNSListener::onHalRestart inMocking:%d",inMocking);
}

void VNSListener::onPropertySet(const vehicle_prop_value_t& value)
{
    SLIM_LOGD("VNSListener::onPropertySet value:%d", value);
}
