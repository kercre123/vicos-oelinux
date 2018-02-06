/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

GENERAL DESCRIPTION
  loc service module

  Copyright  (c) 2015-2017 Qualcomm Technologies, Inc.
  All Rights Reserved. Qualcomm Technologies Proprietary and Confidential.
=============================================================================*/


#define LOG_TAG "Subscription"
#define LOG_NDEBUG 0
#ifdef __ANDROID__
#include "utils/Log.h"
#else
#include <cstddef>
#include <stddef.h>
#endif
#include "Subscription.h"
#include "Subscription_jni.h"
#include "IzatDefines.h"
#include <list>

Subscription* Subscription::mSubscriptionObj = NULL;
#ifdef USE_GLIB
LocNetIface* Subscription::mLocNetIfaceObj = NULL;
#endif
IDataItemObserver* Subscription::mObserverObj = NULL;

using namespace std;

// Subscription class implementation
Subscription* Subscription::getSubscriptionObj()
{
    int result = 0;

    ENTRY_LOG();
    do {
          // already initialized
          BREAK_IF_NON_ZERO(0, mSubscriptionObj);

          mSubscriptionObj = new (std::nothrow) Subscription();
          BREAK_IF_ZERO(2, mSubscriptionObj);
#ifdef USE_GLIB
          mLocNetIfaceObj = new (std::nothrow) LocNetIface();
          BREAK_IF_ZERO(2, mLocNetIfaceObj);
          mLocNetIfaceObj->registerDataItemNotifyCallback(
                  Subscription::locNetIfaceCallback, NULL);
#endif
          result = 0;
    } while(0);

    EXIT_LOG_WITH_ERROR("%d", result);
    return mSubscriptionObj;
}

void Subscription::destroyInstance()
{
    ENTRY_LOG();

    delete mSubscriptionObj;
    mSubscriptionObj = NULL;

    EXIT_LOG_WITH_ERROR("%d", 0);
}

//IDataItemSubscription overrides
void Subscription::subscribe(const std::list<DataItemId> & l, IDataItemObserver * observerObj)
{
    // Assign the observer object if required
    if ((Subscription::mObserverObj == NULL) && (observerObj)) {
        Subscription::mObserverObj = observerObj;
    }
#ifdef __ANDROID__
    update_subscribeJNI(l, true);
#elif defined(USE_GLIB)
    mLocNetIfaceObj->subscribe(l);
#endif
}

void Subscription::updateSubscription(const std::list<DataItemId> & l, IDataItemObserver * observerObj)
{
}

void Subscription::requestData(const std::list<DataItemId> & l, IDataItemObserver * observerObj)
{
    // Assign the observer object if required
    if ((Subscription::mObserverObj == NULL) && (observerObj)) {
        Subscription::mObserverObj = observerObj;
    }
#ifdef __ANDROID__
    requestDataJNI(l);
#elif defined(USE_GLIB)
    mLocNetIfaceObj->requestData(l);
#endif
}

void Subscription::unsubscribe(const std::list<DataItemId> & l, IDataItemObserver * observerObj)
{
     // Assign the observer object if required
    if ((Subscription::mObserverObj == NULL) && (observerObj)) {
        Subscription::mObserverObj = observerObj;
    }
#ifdef __ANDROID__
    update_subscribeJNI(l, false);
#elif defined(USE_GLIB)
    mLocNetIfaceObj->unsubscribe(l);
#endif
}

void Subscription::unsubscribeAll(IDataItemObserver * observerObj)
{
     // Assign the observer object if required
    if ((Subscription::mObserverObj == NULL) && (observerObj)) {
        Subscription::mObserverObj = observerObj;
    }
#ifdef __ANDROID__
    unsubscribeAllJNI();
#elif defined(USE_GLIB)
    mLocNetIfaceObj->unsubscribeAll();
#endif
}

#ifdef USE_GLIB
void Subscription::locNetIfaceCallback(void* userDataPtr, std::list<IDataItem*>& itemList)
{
    LOC_LOGV("Subscription::locNetIfaceCallback");
    if (Subscription::mObserverObj == NULL) {
        LOC_LOGE("NULL observer object");
        return;
    }
    Subscription::mObserverObj->notify(itemList);
}
#endif
