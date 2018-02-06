/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

GENERAL DESCRIPTION
  Network Location Provider Glue

  Copyright (c) 2015 - 2016 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.

=============================================================================*/

#ifndef __LOCATIONPROVIDERGLUE_H__
#define __LOCATIONPROVIDERGLUE_H__

#include <comdef.h>
#include <IzatTypes.h>
#include <IOSListener.h>

namespace izat_manager {
class IzatRequest;
class IzatLocation;
class IIzatManager;
}

using namespace izat_manager;

class LocationProviderGlue
    : public IOSListener {
public:

    void onEnable ();
    void onDisable ();
    void onAddRequest (IzatRequest * request);
    void onRemoveRequest (IzatRequest * request);
    IzatProviderStatus getStatus ();
    int64 getStatusUpdateTime ();

    // override IOSListener
    IzatListenerMask listensTo () const;
    void onLocationChanged (const IzatLocation * location, const IzatLocationStatus status);
    void onStatusChanged (const IzatProviderStatus status);
    inline void onNmeaChanged(const IzatNmea *nmea) {}

protected:
    LocationProviderGlue (IzatStreamType streamType);
    ~LocationProviderGlue ();

    IzatStreamType mLocationStreamType;
    IIzatManager * mIzatManager;
    IzatProviderStatus mStatus;
};

#endif // #ifndef __LOCATIONPROVIDERGLUE_H__

