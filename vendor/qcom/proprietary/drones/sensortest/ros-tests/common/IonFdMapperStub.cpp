#include <binder/Parcel.h>
#include "IIonFdMapper.h"
#include <utils/String8.h>

#include <iostream>

using namespace android;

BpIonFdMapper::BpIonFdMapper(const sp<IBinder>& impl) : BpInterface<IIonFdMapper>(impl) {
  // std::cout << "BpIonFdMapper::BpIonFdMapper()\n";
}

int32_t BpIonFdMapper::mapFd(int32_t fd_as_int) {
  Parcel data, reply;
  data.writeInterfaceToken(IIonFdMapper::getInterfaceDescriptor());
  data.writeInt32(fd_as_int);
  
  std::cout << "BpIonFdMapper::mapfd Sending: " << fd_as_int << std::endl;
  
  remote()->transact(MAPFD, data, &reply);
  int32_t fd = dup(reply.readFileDescriptor());
  std::cout << "BpIonFdMapper::mapfd parcel reply: " << fd << std::endl;
  return fd;
}

int32_t BpIonFdMapper::releaseFd(int32_t fd_as_int) {
  Parcel data, reply;
  data.writeInterfaceToken(IIonFdMapper::getInterfaceDescriptor());
  data.writeInt32(fd_as_int);
  
  std::cout << "BpIonFdMapper::releaseFd Sending: " << fd_as_int << std::endl;
  
  remote()->transact(MAPFD, data, &reply);
  return 0;
}
