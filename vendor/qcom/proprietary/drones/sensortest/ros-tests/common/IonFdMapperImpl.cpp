#include "IonFdMapper.h"
#include <binder/Parcel.h>
#include <utils/String8.h>

#include <iostream>
#include <unistd.h>
IMPLEMENT_META_INTERFACE(IonFdMapper, "IonFdMapper");


IonFdMapper::IonFdMapper() {};
IonFdMapper::~IonFdMapper() {};

int32_t IonFdMapper::mapFd(int32_t fd_as_int) {
  // Nothing to do
  return 1;
}

int32_t IonFdMapper::releaseFd(int32_t fd_as_int) {
  // Nothing to do
  return 1;
}

status_t IonFdMapper::onTransact(uint32_t code, const Parcel& data, Parcel* reply, uint32_t flags) {
  data.checkInterface(this);
  switch(code) {
  case MAPFD: {
    int32_t inData = data.readInt32();
    std::cout << "MAPFD " << __func__ << ". PID: " << getpid() << ". Mapping ION_FD " << inData << std::endl;
    reply->writeFileDescriptor(inData);
    return NO_ERROR;
  } break;
  case RELEASEFD: {
    int32_t inData = data.readInt32();
    std::cout << "RELEASEFD " << __func__ << ". PID: " << getpid() << ". Releasing ION_FD ***NOT IMPLEMENTED YET***" << inData << std::endl;
    //reply->writeFileDescriptor(inData);
    return NO_ERROR;
  } break;
  default:
    return BBinder::onTransact(code, data, reply, flags);
  }
}
