/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

GENERAL DESCRIPTION
  Subscription_jni header file.

  Copyright (c) 2015-2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/

#ifndef __IZAT_MANAGER_SUBSCRIPTION_JNI_H__
#define __IZAT_MANAGER_SUBSCRIPTION_JNI_H__

#include <DataItemId.h>
#include <list>

using namespace std;
using namespace izat_manager;

void update_subscribeJNI(const std::list<DataItemId> & l, bool subscribe);
void requestDataJNI(const std::list<DataItemId> & l);
void unsubscribeAllJNI();
void turnOnModule(DataItemId dit,int timeOut);
void turnOffModule(DataItemId dit);


#endif // #ifndef __IZAT_MANAGER_SUBSCRIPTION_JNI_H__
