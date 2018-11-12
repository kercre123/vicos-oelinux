#pragma once
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include <sstream>
#include <pthread.h>
// Log types
#include "logTypes.h"
/**
   @class Publisher
   This class is used to hide the ROS::Publisher and provides a
   virtual function to publish the next message.
   Derived classes instantiate the right publisher and pass it up.
   count and maxIterations maintain a sequence number so that
   a final "DONE" message is published after maxIterations
**/
class Publisher {
 public:
  virtual int publishNextMsg() = 0;
 protected:
  ros::Publisher publisher;
  int count;
  int maxIterations;
  Publisher (ros::Publisher p, int iter) : publisher(p), count(0), maxIterations(iter)
  {}
};

/**
   @class StringPublisher
   This class publishes simple short strings.
**/
class StringPublisher : public Publisher {
 public:
  StringPublisher(ros::NodeHandle& n, int maxIterations);
  virtual int publishNextMsg();
};

/**
   @class BufferPublisher
   This class publishes buffers of the given size
**/
class BufferPublisher : public Publisher {
 public:
  BufferPublisher(ros::NodeHandle& n, int maxIterations, int bufSize);
  virtual int publishNextMsg();
 private:
  int size;
  std_msgs::UInt8MultiArray msg;
};
    
/**
   @class ION_BufferPublisher
   This class publishes custom allocated ION buffers of the given size
**/
class ION_BufferPublisher : public Publisher {
 public:
  static const char STARTING_BUFFER_CHAR;
  // msgTopic is set differently by the derived class ctor
  ION_BufferPublisher(ros::NodeHandle& n, int maxIterations, int bufSize, const char* msgTopic="ion_buffer");
  virtual ~ION_BufferPublisher() { close(ion_fd); }
  virtual int publishNextMsg();
 protected:
  virtual bool getNextBuffer(int& fd, uint64_t& timeStamp, uint8_t** buffer);
  //  virtual bool getNextBuffer(int& fd, uint8_t** buffer);

 private:
  int allocateBuffer();

 protected:
  int ion_fd;
  std_msgs::UInt8MultiArray msg;
  int data_fd;
  std::string msgType;
 private:
  // These are not needed by the derived class
  int size;
  char bufferContent;
};
  
class CameraTest;
  
/**
   @class ION_CameraBufferPublisher
   This class publishes ION buffers received from the camera
   The CameraTest member is an ICameraListener which puts camera
   frames onto a queue. This class reads those frames off the queue and
   publishes them
**/
class ION_CameraBufferPublisher : public ION_BufferPublisher {
 public:
  ION_CameraBufferPublisher(ros::NodeHandle& n, int maxIterations);
  virtual ~ION_CameraBufferPublisher();
  virtual int publishNextMsg1();
 protected:
  virtual bool getNextBuffer(int& fd, uint64_t& timeStamp, uint8_t** buffer);

 private:
  CameraTest* listener;
};
