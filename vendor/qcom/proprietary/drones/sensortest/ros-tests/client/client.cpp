#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "logTypes.h"
#include <vector>
#include <linux/msm_ion.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <string.h>
#include <sstream>

#include <binder/IPCThreadState.h>
#include <binder/ProcessState.h>
#include <binder/IServiceManager.h>
#include <utils/String8.h>
#include "IIonFdMapper.h"
ROSLogs::LogRecord getLogRecord(const std::string& name);
std::vector<ROSLogs::LogRecord> ROSLogs::logs;
boost::mutex ROSLogs::mutex;
    sp<android::IServiceManager> sm = defaultServiceManager();
    sp<android::IBinder> binder;
    sp<IIonFdMapper> ionFdMapperProxy;
int ionfd;
void msgCallback_String(const std_msgs::String::ConstPtr& msg) 
{
  ROS_DEBUG("Received: %s", msg->data.c_str());
  if (msg->data == "DONE") {
    ROS_INFO("Received DONE. Quitting");
    ros::shutdown();
  }
  else {
    ROSLogs::addLog("ClientCallbackRx", msg->data.c_str());
  }
}

void msgCallback_Buffer(const std_msgs::UInt8MultiArray::ConstPtr& msg) 
{
  ROS_DEBUG_STREAM("Received: Size: " << msg->layout.dim[0].size << ", Data: " << (char*) msg->data.data());
  std::string s("DONE");
  if (s == (char*)msg->data.data()) {
    ROS_INFO("Received DONE. Quitting");
    ros::shutdown();
  }
  else {
    ROSLogs::addLog("ClientCallbackRx", (char*)msg->data.data());
  }
}

void msgCallback_ION_Buffer(const std_msgs::UInt8MultiArray::ConstPtr& msg) 
{
  ROS_DEBUG_STREAM("ION_Buffer: Received: Size: " << msg->layout.dim[0].size 
		   << ", Seq: " << (char*) msg->data.data() << ", Data: 0x" 
		   << std::hex << *(int*)(msg->data.data()+12) << std::dec);
  std::string s("DONE");
  if (s == (char*)msg->data.data()) {
    ROS_INFO("Received DONE. Quitting");
    ros::shutdown();
  }
  else {
    ROSLogs::addLog("ClientCallbackRx", (char*)msg->data.data());

    // Allocate the buffer
    struct ion_fd_data fd_data;
    memset(&fd_data, 0, sizeof(ion_fd_data));
    fd_data.fd = *(int*)(msg->data.data()+12);
    ROS_DEBUG_STREAM("Client calling mapFd with 0x" << std::hex << fd_data.fd << std::dec);
    ROSLogs::addLog("ION_Binder_Pre", (char*)msg->data.data());
    int fd = ionFdMapperProxy->mapFd(fd_data.fd);
    ROSLogs::addLog("ION_Binder_Done", (char*)msg->data.data());
    ROS_DEBUG_STREAM("Client got reply 0x" << std::hex << fd << std::dec);

    fd_data.fd = fd;
    int rc = ioctl(ionfd, ION_IOC_IMPORT, &fd_data);
    ROSLogs::addLog("ION_IMPORT_Done", (char*)msg->data.data());
    if (rc < 0) {
      ROS_ERROR_STREAM("ION_Buffer Client: ION_IOC_MAP failed for handle: " << fd_data.fd << ": " << strerror(errno));
      return;
    }
    else {
      ROS_DEBUG_STREAM("ION_Buffer Client: ION_IOC_IMPORT succeeded " << fd_data.fd);
    }

    void* data = mmap(NULL, 
		      0x1000,
		      PROT_READ | PROT_WRITE, 
		      MAP_SHARED,
		      fd_data.fd,
		      0);
    ROSLogs::addLog("ION_MMAP_Client_Done", (char*)msg->data.data());
    if (data == 0) {
      ROS_ERROR_STREAM("mmap failed");
    }
    else {
      ROS_DEBUG_STREAM("ION_Buffer Client: mmap succeeded");
      uint8_t* b = static_cast<uint8_t*>(data);
      const uint8_t* r = msg->data.data();
      std::ostringstream os1, os2;
      for (int i = 0; i < 40; i++) 
	os1 << " " << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(b[i]);
      ROS_DEBUG_STREAM("Mapped buffer: " << os1.str());
      int offset = 12+sizeof(int);
      for (int i = offset; i < offset+40; i++) 
	os2 << " " << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(r[i]);
      ROS_DEBUG_STREAM("Rx'ed buffer : " << os2.str());
      if ( memcmp(data, msg->data.data()+offset, 40) == 0 ) {
	ROS_DEBUG_STREAM("Buffers identical");
      }
      else {
	ROS_ERROR_STREAM("Buffers differ");
      }
    }
  }
}

void msgCallback_ION_CameraBuffer(const std_msgs::UInt8MultiArray::ConstPtr& msg) 
{
  ROS_INFO_STREAM("ION_CameraBuffer: Received: Size: " << msg->layout.dim[0].size 
		   << ", Seq: " << (char*) msg->data.data() 
		   << ", Data: 0x" << std::hex << *(int*)(msg->data.data()+12));
  std::string s("DONE");
  if (s == (char*)msg->data.data()) {
    ROS_INFO("Received DONE. Quitting");
    ros::shutdown();
  }
  else {
    ROSLogs::addLog("ClientCallbackRx", (char*)msg->data.data());

    struct ion_fd_data fd_data;
    memset(&fd_data, 0, sizeof(ion_fd_data));
    fd_data.fd = *(int*)(msg->data.data()+20);
    ROS_DEBUG_STREAM("Client calling mapFd with 0x" << std::hex << fd_data.fd);
    ROSLogs::addLog("ION_Binder_Pre", (char*)msg->data.data());
    int fd = ionFdMapperProxy->mapFd(fd_data.fd);
    ROSLogs::addLog("ION_Binder_Done", (char*)msg->data.data());
    ROS_DEBUG_STREAM("Client got reply 0x" << std::hex << fd);
    fd_data.fd = fd;
    int rc = ioctl(ionfd, ION_IOC_IMPORT, &fd_data);
    ROSLogs::addLog("ION_IMPORT_Done", (char*)msg->data.data());
    if (rc < 0) {
      ROS_ERROR_STREAM("ION_Buffer Client: ION_IOC_MAP failed for handle: " << fd_data.fd << ": " << strerror(errno));
      return;
    }
    else {
      ROS_DEBUG_STREAM("ION_Buffer Client: ION_IOC_IMPORT succeeded " << fd_data.fd);
    }

    void* data = mmap(NULL, 
		      0x1000,
		      PROT_READ | PROT_WRITE, 
		      MAP_SHARED,
		      fd_data.fd,
		      0);
    ROSLogs::addLog("ION_MMAP_Client_Done", (char*)msg->data.data());
    if (data == 0) {
      ROS_INFO_STREAM("mmap failed");
    }
    else {
      ROS_DEBUG_STREAM("ION_Buffer Client: mmap succeeded");
      uint8_t* b = static_cast<uint8_t*>(data);
      const uint8_t* r = msg->data.data();

      std::ostringstream os1, os2;
      for (int i = 0; i < 40; i++) 
	os1 << " " << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(b[i]);
      ROS_DEBUG_STREAM("Mapped buffer: " << os1.str());
      int offset = 20+sizeof(int);
      for (int i = offset; i < offset+40; i++) 
	os2 << " " << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(r[i]);
      ROS_DEBUG_STREAM("Rx'ed buffer : " << os2.str());
      if ( memcmp(data, msg->data.data()+offset, 40) == 0 ) {
	ROS_DEBUG_STREAM("Buffers identical");
      }
      else {
	ROS_ERROR_STREAM("Buffers differ");
      }
    }
  }
}

void calcTransportHints(std::string& transport, ros::TransportHints& transportHints)
{
  if (transport == "tcp") {
    transportHints.tcp();
    transportHints.tcpNoDelay(false);
    ROS_INFO("Client Using tcp");
  }
  else if (transport == "tcpNoDelay") {
    transportHints.tcp();
    transportHints.tcpNoDelay(true);
    ROS_INFO("Client Using tcpNoDelay");
  }
  else if (transport == "udp") {
    transportHints.udp();
    ROS_INFO("Client Using udp");
  }
  else {
    transportHints.tcp();
    ROS_INFO("Client Using tcp");
  }
}

int doIONInit() {
  // All variables here are globals
  ionfd = open("/dev/ion", O_RDONLY);
  if (ionfd < 0) {
    ROS_ERROR("Failed to open ion device\n");
    return 1;
  }
  ROS_DEBUG_STREAM("ION_Buffer Client: Opened ionfd");
  
  
  do {
    binder = sm->getService(String16("IonFdMapper"));
    if (binder != 0)
      break;
    ROS_ERROR_STREAM("Service not published, waiting...\n");
    usleep(500000); // 0.5 s
  } while(true);
  
  // std::cout << "binder status: " << std::hex << binder->isBinderAlive() << std::endl;
  // std::cout << "ping status: " << std::hex << binder->pingBinder() << std::endl;
  ionFdMapperProxy = interface_cast<IIonFdMapper>(binder);
} 
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "client");
  ros::NodeHandle n;
  
  std::string logDir;
  if (n.hasParam("logDir")) {
    n.getParam("logDir", logDir);
  }
  std::string transport("tcp");
  if (n.hasParam("transport")) {
    n.getParam("transport", transport);
  }
  ros::TransportHints transportHints;
  calcTransportHints(transport, transportHints);

  ros::Subscriber msgSubscriber;
  std::string msgType;
  if (n.hasParam("msgType")) {
    n.getParam("msgType", msgType);
    ROS_INFO_STREAM("Client Using message type: " << msgType);
    if ( msgType == "string" ) {
      msgSubscriber = n.subscribe(msgType.c_str(), 100, msgCallback_String, transportHints);
    }
    else if ( msgType.find("ion_camera_buffer") != std::string::npos ) {
      doIONInit();
      msgSubscriber = n.subscribe(msgType.c_str(), 100, msgCallback_ION_CameraBuffer, transportHints);
    }
    else if ( msgType.find("ion_buffer") != std::string::npos ) {
      doIONInit();
      msgSubscriber = n.subscribe(msgType.c_str(), 100, msgCallback_ION_Buffer, transportHints);
    }
    else if ( msgType.find("buffer") != std::string::npos ) {
      msgSubscriber = n.subscribe(msgType.c_str(), 100, msgCallback_Buffer, transportHints);
    }
    else {
      return 1;
    }
  }
  else {
    return 1;
  }
  ROSLogs::addLog("Subscribed", "0");

  //  ros::Subscriber msgSubscriber = n.subscribe(msgType.c_str(), 100, msgCallback, transportHints);
  ros::spin();
  ROSLogs::printPyLogs("client", logDir);
  return 0;
}
