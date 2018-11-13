#include "Publisher.h"
#include <cstdlib>
#include <cinttypes>
#include <linux/msm_ion.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <binder/ProcessState.h>
#include <binder/IServiceManager.h>
#include <cameraListener.h>
#include <IonFdMapper.h>
using namespace android;
StringPublisher::StringPublisher(ros::NodeHandle& n, int maxIterations) : 
  Publisher(n.advertise<std_msgs::String>("string", 100), maxIterations)
{
  std::cout << "In StringPublisher";
}

int StringPublisher::publishNextMsg() {
  std_msgs::String msg;
  std::stringstream ss;
  
  if (count >= maxIterations) {
    ss << "DONE";
  }
  else {
    ss << count++;
    ROSLogs::addLog("ServerMsgTx", ss.str());
  }
  msg.data = ss.str();
  ROS_DEBUG("Sending: %s", msg.data.c_str());
  publisher.publish(msg);
  return count;
}

BufferPublisher::BufferPublisher(ros::NodeHandle& n, int maxIterations, int bufSize) : 
  Publisher(n.advertise<std_msgs::UInt8MultiArray>("buffer", 100), maxIterations), size(bufSize)
{
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = bufSize;
  msg.layout.dim[0].stride = 1;
  msg.layout.dim[0].label = "bufferMsg";
  for (int i = 0; i < size; i++) {
    // Save the first few bytes for the sequence number
    msg.data.push_back( (i < 10) ? 0 : i );
  }
}

int BufferPublisher::publishNextMsg() {
  if (count >= maxIterations) {
    sprintf((char*)msg.data.data(), "%s",  "DONE");
  }
  else {
    snprintf((char*)msg.data.data(), sizeof(int), "%d", count);  
    ROSLogs::addLog("ServerMsgTx", (char*)msg.data.data());
  }
  ROS_DEBUG("Sending: %s", msg.data.data());
  publisher.publish(msg);
  count++;
  return count;
}

const char ION_BufferPublisher::STARTING_BUFFER_CHAR = '0';

ION_BufferPublisher::ION_BufferPublisher(ros::NodeHandle& n, int maxIterations, int bufSize, const char* msgTopic) : 
  Publisher(n.advertise<std_msgs::UInt8MultiArray>(msgTopic, 100), maxIterations), 
  size(bufSize), 
  bufferContent(STARTING_BUFFER_CHAR),
  msgType(msgTopic)
{
  // Metadata for the outgoing ROS message
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = bufSize;
  msg.layout.dim[0].stride = 1;
  msg.layout.dim[0].label = "ION_bufferMsg";
  for (int i = 0; i < bufSize; i++) {
    // Save the first few bytes for the sequence number
    msg.data.push_back( (i < 10) ? 0 : i );
  }

  // We open the ION device here.
  ion_fd = open("/dev/ion", O_RDONLY);
  if (ion_fd < 0) {
    ROS_ERROR("Failed to open ion device\n");
  }
  ROS_INFO_STREAM("ION_BufferPublisher: Opened ion_fd");

  // Finally, start a binder "service". The overall design is:
  //    - We publish the ION FD of the buffer to the ROS subscriber
  //    - The subscriber accesses us over binder, sending the FD back to us
  //    - We use binder (Parcel::writeFileDescriptor) to send them an FD in
  //      a way that process can actually use.
  //      NOTE: There are two ways (AFAIK) to send FDs to another process:
  //            - Use Unix Domain Sockets
  //            - Use binder
  ROS_INFO_STREAM("Starting ION_BufferPublisher's Binder server");
  ProcessState::self()->startThreadPool();
  IonFdMapper* svc = new IonFdMapper();
  sp<android::IServiceManager> sm(defaultServiceManager());
  sm->addService(String16("IonFdMapper"), svc, false);
}

#if 0
// Ideally we just need publishNextMsg to call virtual getNextBuffer, and
// send the FD and some buffer data to the subscriber. However, getNextBuffer
// doesn't work as expected for custom allocated buffers. It works perfectly
// for buffers we get from the camera.
// So disable this function for now and go with the more hacky version below
int ION_BufferPublisher::publishNextMsg() {
  ROS_INFO_STREAM("In ION_BufferPublisher::publishNextMsg");
  if (count >= maxIterations) {
    sprintf((char*)msg.data.data(), "%s",  "DONE");
  }
  else {
    ROSLogs::addLog("ION_ALLOC_Pre", (char*)msg.data.data());
    uint8_t* buffer=0;
    int fd;
    bool r =  getNextBuffer(fd, &buffer);
    snprintf((char*)msg.data.data(), sizeof(int), "%d", count);  
    *(int*)(msg.data.data()+12) = fd;
    memcpy(msg.data.data()+12+sizeof(int), buffer, 40);
  }
  
  ROS_DEBUG("Sending: Seq: %s, Data: 0x%x", msg.data.data(), *(int*)(msg.data.data() + 12));
  publisher.publish(msg);
  count++;
  return count;
}
#endif

// The hacky publishNextMsg that looks at the msgType to determine how to
// get the next buffer. For ion_camera_buffer case, it calls getNextBuffer, as
// expected, but for ion_buffer it actually does the buffer allocation itself,
// instead of calling getNextBuffer.
int ION_BufferPublisher::publishNextMsg() {
  ROS_DEBUG_STREAM("In ION_BufferPublisher::publishNextMsg");
  std::stringstream ss;
  
  if (count >= maxIterations) {
    sprintf((char*)msg.data.data(), "%s",  "DONE");
  }
  else {
    ss << count;
    if (msgType.compare("ion_camera_buffer") == 0) {
      uint8_t* buffer=0;
      int fd;
      uint64_t timeStamp;
      bool r =  getNextBuffer(fd, timeStamp, &buffer);
      if (!r) {
	return -1;
      }
      sprintf((char*)msg.data.data(), "%+" PRIu64, timeStamp);  
      *(int*)(msg.data.data()+20) = fd;
      memcpy(msg.data.data()+20+sizeof(int), buffer, 40);
    }
    else if (msgType.compare("ion_buffer") == 0) {
      // Allocate the buffer
      ROSLogs::addLog("ION_ALLOC_Pre", ss.str());
      struct ion_fd_data fd_data;
      struct ion_allocation_data alloc_data;
      
      alloc_data.len = (0x1000 + 4095) & (~4095);
      alloc_data.align = 4096;
      alloc_data.heap_id_mask= 0x1<<ION_IOMMU_HEAP_ID;;
      
      int rc = ioctl(ion_fd,ION_IOC_ALLOC, &alloc_data);
      ROSLogs::addLog("ION_ALLOC_Done", ss.str());
      
      if (rc) {
	ROS_ERROR_STREAM("ION_BufferPublisher: ION_ALLOC failed");
	return 0;
      }
      else {
	ROS_DEBUG_STREAM("ION_BufferPublisher: ION_ALLOC succeeded");
      }
      
      // Map the buffer and fill it
      memset(&fd_data, 0, sizeof(ion_fd_data));
      fd_data.handle = alloc_data.handle;
      rc = ioctl(ion_fd, ION_IOC_SHARE, &fd_data);
      if (rc < 0) {
	ROS_ERROR_STREAM("ION_BufferPublisher: ION_IOC_SHARE failed");
	return 0;
      }
      else {
	ROS_DEBUG_STREAM("ION_BufferPublisher: ION_IOC_SHARE succeeded. Got FD: " << fd_data.fd);
      }
      
      ROSLogs::addLog("ION_SHARE_Done", ss.str());
      void* data = mmap(NULL, 
			alloc_data.len, 
			PROT_READ | PROT_WRITE, 
			MAP_SHARED,
			fd_data.fd,
			0);
      if (data == 0) {
	ROS_ERROR_STREAM("mmap failed" << strerror(errno));
	return 0;
      }
      else {
	ROSLogs::addLog("ION_MMAP_Done", ss.str());
	ROS_DEBUG_STREAM("ION_BufferPublisher: mmap succeeded");
	// Fill the buffer
	for (int i = 0; i < alloc_data.len; i++) {
	  *((char*)data + i) = bufferContent;
	}
	// Fill each buffer with a new printable ASCII character
	bufferContent = ((uint8_t) bufferContent > 125) ? STARTING_BUFFER_CHAR : (uint8_t)bufferContent +1;
	
	// Finally unmap buffer
	//munmap(data, data_size);
	snprintf((char*)msg.data.data(), sizeof(int), "%d", count);  
	*(int*)(msg.data.data()+12) = fd_data.fd;
	memcpy(msg.data.data()+12+sizeof(int), data, 40);
      }
    }
    ROS_DEBUG_STREAM("Logging ServerMsgTx");
    ROSLogs::addLog("ServerMsgTx", (char*)msg.data.data());
  }

  ROS_INFO("Sending: Seq: %s, Data: 0x%x", msg.data.data(), *(int*)(msg.data.data() + 12));
  publisher.publish(msg);
  count++;
  return count;
}

bool ION_BufferPublisher::getNextBuffer(int& fd, uint64_t& timeStamp, uint8_t**buffer ) {
  // Calling this function from publishNextMsg doesn't work for this class. But I'm
  // keeping the function

  // Allocate the buffer
  ROS_DEBUG_STREAM("In ION_BufferPublisher::getNextBuffer");
  struct ion_fd_data fd_data;
  struct ion_allocation_data alloc_data;
  
  alloc_data.len = (0x1000 + 4095) & (~4095);
  alloc_data.align = 4096;
  //    alloc_data.heap_id_mask= 0x1<<ION_IOMMU_HEAP_ID;
  alloc_data.heap_id_mask= ION_HEAP(ION_SYSTEM_HEAP_ID);
  
  int rc = ioctl(ion_fd,ION_IOC_ALLOC, &alloc_data);
  void* data = 0;
  if (rc) {
    ROS_ERROR_STREAM("ION_BufferPublisher: ION_ALLOC failed");
    return false;
  }
  else {
    ROS_DEBUG_STREAM("ION_BufferPublisher: ION_ALLOC succeeded");
  }
  
  // Map the buffer and fill it
  memset(&fd_data, 0, sizeof(ion_fd_data));
  fd_data.handle = alloc_data.handle;
  rc = ioctl(ion_fd, ION_IOC_SHARE, &fd_data);
  if (rc < 0) {
    ROS_ERROR_STREAM("ION_BufferPublisher: ION_IOC_SHARE failed");
    return 0;
  }
  else {
    ROS_DEBUG_STREAM("ION_BufferPublisher: ION_IOC_SHARE succeeded. Got FD: " << fd_data.fd);
    data_fd = fd_data.fd;
  }
  
  data = mmap(NULL, 
	      alloc_data.len, 
	      PROT_READ | PROT_WRITE, 
	      MAP_SHARED,
	      fd_data.fd,
	      0);
  if (data == 0) {
    ROS_ERROR_STREAM("mmap failed" << strerror(errno));
    return false;
  }
  else {
    ROS_DEBUG_STREAM("ION_BufferPublisher: mmap succeeded. Len: " << alloc_data.len << ". Filling");
    // Fill the buffer
    for (int i = 0; i < alloc_data.len; i++) {
      *((char*)data + i) = bufferContent;
    }
    //sprintf((char*)data, "%s", "This is a string");
    // Use a new character to fill, each time this function is called
    bufferContent = ((uint8_t) bufferContent > 125) ? STARTING_BUFFER_CHAR : (uint8_t)bufferContent +1;
    
    // Finally unmap buffer
    //munmap(data, data_size);
  }
  fd = fd_data.fd;
  *buffer = static_cast<uint8_t*>(data);
  ROS_DEBUG_STREAM("In ION_BufferPublisher::getNextBuffer: Returning");
  return true;
}


ION_CameraBufferPublisher::ION_CameraBufferPublisher(ros::NodeHandle& n, int maxIterations) : 
  listener(0),
  ION_BufferPublisher(n, maxIterations, 100, "ion_camera_buffer")
{
  listener = new CameraTest();
  listener->initialize(0);
}

ION_CameraBufferPublisher::~ION_CameraBufferPublisher() {
  ROS_DEBUG_STREAM("In ION_CameraBufferPublisher dtor");
  if (listener) {
    delete listener;
  }
}

int ION_CameraBufferPublisher::publishNextMsg1() {
  ROS_DEBUG_STREAM("In ION_CameraBufferPublisher::publishNextMsg");
  
  if (count >= maxIterations) {
    sprintf((char*)msg.data.data(), "%s",  "DONE");
  }
  else {
    snprintf((char*)msg.data.data(), sizeof(int), "%d", count);  
    int fd;
    uint8_t* buffer;
    uint64_t timeStamp;
    bool r = getNextBuffer(data_fd, timeStamp, &buffer);
    if (!r) return -1;
    if (count >= maxIterations) {
      sprintf((char*)msg.data.data(), "%s",  "DONE");
    }
    else {
      snprintf((char*)msg.data.data(), sizeof(int), "%d", count);  
      *(int*)(msg.data.data()+12) = data_fd; 
      memcpy(msg.data.data()+12+sizeof(int), buffer, 40);
    }
  }
  ROS_INFO("Sending: Seq: %s, Data: 0x%x", msg.data.data(), *(int*)(msg.data.data() + 12));
  publisher.publish(msg);
  count++;
  return count;
}

bool ION_CameraBufferPublisher::getNextBuffer(int& fd, uint64_t& timeStamp, uint8_t** buffer) {
  ROS_DEBUG_STREAM("In ION_CameraBufferPublisher::getNextBuffer");
  ICameraFrame* frame = listener->getNextFrame();
  if (!frame) return false;
  fd = frame->fd;
  *buffer = frame->data;
  timeStamp = frame->timeStamp;
  std::stringstream ss;
  ss << timeStamp;
  ROSLogs::addLog("ION_CameraFrame_Dequeued", ss.str());
  return true;
}
