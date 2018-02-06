/***************************************************************************//**
@file
   VISUALIZATION_ROS.h

@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef VISUALIZATION_ROS_H
#define VISUALIZATION_ROS_H

#include "Visualization.h"
#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

class Visualiser_ROS : public Visualiser
{

public:
   Visualiser_ROS( const char * outputPath, uint32_t outputEveryNFrame );
   virtual ~Visualiser_ROS();

   virtual void PublishOriginalImage( const uint64_t stamp, const uint8_t * image, int imageWidth, int imageHeight );
   virtual void PublishUndistortedImage( const uint64_t stamp, const uint8_t * image, int imageWidth, int imageHeight );
   virtual void ShowPoints( const mvWEFPoseStateTime & pose, const uint8_t * image, int imageWidth, int imageHeight );
   virtual void PublishRobotPose( const mvWEFPoseVelocityTime & pose );
   virtual void PublishCameraPose( const mvWEFPoseStateTime & pose, const uint8_t * image, int imageWidth, int imageHeight );
   virtual void PublishCorrectedCameraPose( const mvWEFPoseStateTime& );
   virtual void PublishExposureGain( float32_t exposure, float32_t gain, int exposureValue, int gainValue, float mean_brightness );


private:

   int frameIndex;
   uint32_t _outputEveryNFrame;

   image_transport::Publisher labelledImagePub;
   image_transport::Publisher originalImagePub;
   image_transport::Publisher undistortedImagePub;
   ros::Publisher pub_fusion;
   nav_msgs::Odometry odom_;

   ros::Publisher pubCameraOdom;
   int32_t cameraOdomCount;

   ros::Publisher pubCorrectedCameraOdom;
   int32_t correctedCameraOdomCount;

   ros::Publisher pubVSLAMOdom_Raw;

   void WEFPoseStateTime2ROSTopic( const mvWEFPoseStateTime & pose, nav_msgs::Odometry &  odom );
   void mvPose6DRT2ROSTopic( const mvPose6DRT & pose, nav_msgs::Odometry &  odom );

   void InitOdom( nav_msgs::Odometry & odom );

   ros::Publisher pub_keyframeNum;
   ros::Publisher pub_cpaParameter;
   ros::Publisher pub_state;

};

#endif // VISUALIZATION_ROS_H
