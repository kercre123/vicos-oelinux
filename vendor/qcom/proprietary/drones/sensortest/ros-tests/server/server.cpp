#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/ByteMultiArray.h"
#include <sstream>
#include "logTypes.h"
#include "Publisher.h"

const double DEFAULT_LOOP_RATE = 10;
std::vector<ROSLogs::LogRecord> ROSLogs::logs;
boost::mutex ROSLogs::mutex;

int main(int argc, char **argv)
{
  ROSLogs l;
  ros::init(argc, argv, "server");
  ros::NodeHandle n;

  std::string logDir;
  if (n.hasParam("logDir")) {
    n.getParam("logDir", logDir);
    ROS_INFO_STREAM("Using logDir: " << logDir);
  }

  double loopRate;
  if (n.hasParam("loopRate")) {
    n.getParam("loopRate", loopRate);
    ROS_INFO_STREAM("Using loopRate: " << loopRate);
  }
  else {
    loopRate = DEFAULT_LOOP_RATE;
  }

  int iterations;
  if (n.hasParam("iterations")) {
    n.getParam("iterations", iterations);
    ROS_INFO_STREAM("Using iterations: " << iterations);
  }

  int msgSize;
  if (n.hasParam("msgSize")) {
    n.getParam("msgSize", msgSize);
    ROS_INFO_STREAM("Using msgSize: " << msgSize);
  }

  std::string msgType;
  Publisher* publisher = 0;
  if (n.hasParam("msgType")) {
    n.getParam("msgType", msgType);
    ROS_INFO_STREAM("Using message type: " << msgType);
    if ( msgType == "string" ) {
      publisher = new StringPublisher(n, iterations);
    }
    else if ( msgType.find("ion_camera_buffer") != std::string::npos ) {
      std::cout << "Constructing ION_CameraBufferPublisher" << std::endl;
      publisher = new ION_CameraBufferPublisher(n, iterations);
    }
    else if ( msgType.find("ion_buffer") != std::string::npos ) {
      std::cout << "Constructing ION_BufferPublisher" << std::endl;
      publisher = new ION_BufferPublisher(n, iterations, msgSize);
    }
    else if ( msgType.find("buffer") != std::string::npos ) {
      publisher = new BufferPublisher(n, iterations, msgSize);
    }
    else {
      return 1;
    }
  }
  else {
    return 1;
  }

  // Sleep to let the client be ready before we start sending
  ros::Rate initialSleep(1);
  initialSleep.sleep();
  ros::Rate loop_rate(loopRate);
  int count = 0;

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    if (publisher->publishNextMsg() > iterations) {
      break;
    }
  }
  delete publisher;
  ros::shutdown();
  ROSLogs::printPyLogs("server", logDir);
  return 0;
}
