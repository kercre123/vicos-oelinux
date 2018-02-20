/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/
#include <errno.h>
#include <string.h>
#include <linux/input.h>
#include <fcntl.h>
#include "platform_lib_log_util.h"
#include "ulp_service.h"
#include "PowerEvtHandler.h"
#include "power_state.h"

#define ACK_TIMEOUT_US 300000 // 300 msec

PowerEvtHandler &PowerEvtHandler::getPwrEvtHandler() {
    static PowerEvtHandler instance;
    return instance;
}

PowerEvtHandler::PowerEvtHandler(): mProducerEventObserver(NULL) {

    mProducerEventObserver = new EventObserver();
    if (NULL == mProducerEventObserver) {
        LOC_LOGE("Failed to create EventObserver object!\n");
    } else {
        //register with power api framework
        pwr_state_notification_register(PowerEvtHandler::pwrStateCb);
    }
}

PowerEvtHandler::~PowerEvtHandler()
{
    if (mProducerEventObserver) {
        delete mProducerEventObserver;
    }
}

int PowerEvtHandler::pwrStateCb(power_state_t pwr_state)
{
    client_ack_t client_ack;
    client_ack.ack = ERR;
    PowerEvtHandler &handle = getPwrEvtHandler();
    LOC_LOGV("pwrStateCb: pwr_state %d\n", pwr_state.sys_state);
    if (handle.mProducerEventObserver && (SYS_SUSPEND == pwr_state.sys_state)) {
        getPwrEvtHandler().mProducerEventObserver->sendSystemEvent(ULP_LOC_SCREEN_OFF);
        client_ack.ack = SUSPEND_ACK;
    } else if (handle.mProducerEventObserver && (SYS_RESUME == pwr_state.sys_state)) {
        handle.mProducerEventObserver->sendSystemEvent(ULP_LOC_SCREEN_ON);
        client_ack.ack = RESUME_ACK;
    } else if (handle.mProducerEventObserver && (SYS_SHUTDOWN == pwr_state.sys_state)) {
        handle.mProducerEventObserver->sendSystemEvent(ULP_LOC_SCREEN_OFF);
        client_ack.ack = SHUTDOWN_ACK;
    }
    //Allow some time to stop the session and write calibration data NVM.
    usleep(ACK_TIMEOUT_US);

    LOC_LOGV("pwrStateCb: sending ack %d to power daemon\n", client_ack.ack);
    send_acknowledgement(client_ack);
}
