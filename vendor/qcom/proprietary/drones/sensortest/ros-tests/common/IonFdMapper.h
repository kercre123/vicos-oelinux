#pragma once
#include "IIonFdMapper.h"
using namespace android;

class IonFdMapper : public BnIonFdMapper {
 public:
  IonFdMapper();
  virtual ~IonFdMapper();
  virtual int32_t mapFd(int32_t fd_as_int);
  virtual int32_t releaseFd(int32_t fd_as_int);
  virtual status_t onTransact(uint32_t code, const Parcel& data, Parcel* reply, uint32_t flags);
};
