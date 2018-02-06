/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/

#include <IzatDefines.h>
#include <IzatManager.h>
#include <IzatPassiveSvInfoListener.h>
#include <Utils.h>

#define LOG_TAG "IzatSvc_PassiveSvInfoListener"
#include <platform_lib_log_util.h>

using namespace izat_manager;
using namespace std;

IzatPassiveSvInfoListener::IzatListenerMapType IzatPassiveSvInfoListener::sObjects;

void IzatPassiveSvInfoListener :: startPassiveSvInfoListener(
    const std::string clientName,
    const IzatListenerMask listensTo,
    IIzatManager *izatManager,
    IPCMessagingProxy *iPCMessagingProxy) {
    int result = -1;
    do {
        if (NULL == sObjects[clientName]) {
            sObjects[clientName] =
                new(nothrow) IzatPassiveSvInfoListener(clientName, listensTo,
                                                         izatManager, iPCMessagingProxy);
            BREAK_IF_ZERO(1, sObjects[clientName]);
            result = 0;
        }
    } while (0);
    EXIT_LOG_WITH_ERROR("%d", result);
}

void IzatPassiveSvInfoListener::stopAllPassiveSvInfoListeners(IIzatManager *izatManager) {
    for (IzatListenerMapType::iterator it = sObjects.begin(); it != sObjects.end(); it++) {
        if(0 != izatManager->unsubscribeListener(it->second)) {
            LOC_LOGI("Failed to unsubcribe svinfo listener for %s", it->first.c_str());
        }
        delete it->second;
    }
}


// ctor
IzatPassiveSvInfoListener :: IzatPassiveSvInfoListener(const std::string clientName,
                                                           IzatListenerMask listensTo,
                                                           IIzatManager *izatManager,
                                                           IPCMessagingProxy *iPCMessagingProxy)
    : mClientName(clientName)
    , mIpcName(clientName + "-SV-REPORT-LISTENER")
    , mListenerMask(listensTo)
    , mIPCMessagingProxy(iPCMessagingProxy) {
    int result = -1;

    do {
        BREAK_IF_ZERO(1, izatManager);
        izatManager->subscribeListener(this);
        result = 0;
    } while(0);

    EXIT_LOG_WITH_ERROR("%d", result);
}

// dtor
IzatPassiveSvInfoListener :: ~IzatPassiveSvInfoListener() { }

//overrides
IzatListenerMask IzatPassiveSvInfoListener :: listensTo() const {
    return mListenerMask;
}

void IzatPassiveSvInfoListener :: onSvReportChanged (const IzatSvInfo *svInfo)
{
    ENTRY_LOG();
    LocGnssSvStatus gnssSvStatus;
    GpsLocationExtended locExtended;
    memset(&gnssSvStatus, 0 , sizeof(LocGnssSvStatus));
    memset(&locExtended, 0 ,sizeof(GpsLocationExtended));

    OutPostcard *card = NULL;
    int result = 0;
    do {

        BREAK_IF_ZERO(1, svInfo);

        gnssSvStatus = svInfo->mGnssSvStatus;
        locExtended = svInfo->mLocExtended;
        BREAK_IF_ZERO(2, mIPCMessagingProxy);
        // send out the location update
        card = OutPostcard::createInstance();
        BREAK_IF_ZERO(5, card);
        BREAK_IF_NON_ZERO(6, card->init());
        BREAK_IF_NON_ZERO(7, card->addString("TO", mClientName.c_str()));
        BREAK_IF_NON_ZERO(8, card->addString("FROM", mIpcName.c_str()));
        BREAK_IF_NON_ZERO(9, card->addString("INFO", "SVINFO-UPDATE"));
        const void* blob = (const void*) &gnssSvStatus;
        size_t length = sizeof (LocGnssSvStatus);
        BREAK_IF_NON_ZERO(10, card->addBlob("GNSSSVSTATUS", blob, length));
        const void *extblob = (const void*) &locExtended;
        length = sizeof (locExtended);
        BREAK_IF_NON_ZERO(11, card->addBlob("LOCEXTENDED", extblob, length));
        BREAK_IF_NON_ZERO(12, card->finalize());
        BREAK_IF_NON_ZERO(13, mIPCMessagingProxy->sendMsg(card, mClientName.c_str()));
    } while(0);

    delete card;

    EXIT_LOG_WITH_ERROR("%d", result);
}

