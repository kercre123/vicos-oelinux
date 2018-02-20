/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/
#ifndef _POWER_EVENT_HANDLER_H_
#define _POWER_EVENT_HANDLER_H_

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include "EventObserver.h"
#include "power_state.h"

class PowerEvtHandler {

public:
    static PowerEvtHandler & getPwrEvtHandler();
    ~PowerEvtHandler();

    EventObserver *mProducerEventObserver;
    static int     pwrStateCb(power_state_t pwr_state);

private:
    PowerEvtHandler();

};

#endif //_POWER_EVENT_HANDLER_H_
