/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

GENERAL DESCRIPTION
  loc service module

  Copyright (c) 2015-2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/

#ifndef __IZAT_MANAGER_IZATCONTEXT_H__
#define __IZAT_MANAGER_IZATCONTEXT_H__

#include <MsgTask.h>
#include <mq_client/IPCMessagingProxy.h>
#include <IFrameworkActionReq.h>


using namespace qc_loc_fw;

namespace izat_manager
{
class IOSFramework;
class IDataItemSubscription;
class OSObserver;

struct s_IzatContext {
    IOSFramework *mOSFrameworkObj;
    IDataItemSubscription *mSubscriptionObj;
    IFrameworkActionReq *mFrameworkActionReqObj;
    IPCMessagingProxy *mIPCMessagingProxyObj;
    OSObserver *mOSObserverObj;
    MsgTask *mMsgTask;

   inline s_IzatContext() : mOSFrameworkObj(NULL), mSubscriptionObj(NULL),
        mIPCMessagingProxyObj(NULL), mOSObserverObj(NULL), mMsgTask(NULL) {
    }
};

}// namespace izat_manager

#endif // #ifndef __IZAT_MANAGER_IZATCONTEXT_H__

