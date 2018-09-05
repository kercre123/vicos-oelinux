#pragma once
#include <binder/IInterface.h>
#include <binder/IBinder.h>
using namespace android;

class IIonFdMapper : public IInterface {
public:
  enum {
    MAPFD = IBinder::FIRST_CALL_TRANSACTION,
    RELEASEFD
  };
  // Takes a file descriptor and maps into caller's process
  virtual int32_t     mapFd(int32_t fd_as_int)          = 0;
  // Takes a file descriptor and "releases" the corresponding ION buffer
  virtual int32_t     releaseFd(int32_t fd_as_int)      = 0;
  
  DECLARE_META_INTERFACE(IonFdMapper);
};


class BnIonFdMapper : public BnInterface<IIonFdMapper> {
public:
  virtual status_t onTransact(uint32_t code, const Parcel& data, Parcel* reply, uint32_t flags = 0) = 0;
};


class BpIonFdMapper : public BpInterface<IIonFdMapper> {
public:
  BpIonFdMapper(const sp<IBinder>& impl);  
  virtual int32_t mapFd(int32_t data);
  virtual int32_t releaseFd(int32_t data);
};
