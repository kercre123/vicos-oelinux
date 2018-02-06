/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

GENERAL DESCRIPTION
   Location Provider Engine JNI

  Copyright  (c) 2015 - 2016 Qualcomm Technologies, Inc.
  All Rights Reserved. Qualcomm Technologies Proprietary and Confidential.
=============================================================================*/

#ifndef __LOCATIONPROVIDERENGINEJNI_H__
#define __LOCATIONPROVIDERENGINEJNI_H__

#include <IzatTypes.h>

namespace izat_manager {
class IzatLocation;
}

using namespace izat_manager;

void onLocationChangedJNI
(
    const IzatLocation * location,
    IzatStreamType type
);

void onStatusChangedJNI
(
    const IzatProviderStatus status,
    IzatStreamType type
);

#endif // #ifndef __LOCATIONPROVIDERENGINE_H__
