/*
 * Copyright (c) 2015,2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef ANDROID_QVOPSERVICEBN_H
#define ANDROID_QVOPSERVICEBN_H

#include <binder/Parcel.h>
#include "IQvopServiceAndroid.h"

namespace android {

class BnQvopService : public BnInterface<IQvopService> {
 public:
  virtual status_t onTransact(uint32_t code, const Parcel& data, Parcel* reply, uint32_t flags = 0);
};

}; // namespace android

#endif // ANDROID_QVOPSERVICEBN_H

