/***************************************************************************//**
@file
   VirtualWheel_ROS.cpp

@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#include "VirtualWheel_ROS.h"
#include "VSLAM_internal.h"
#include "Queue.h"
#include "mvWEF.h"

extern queue_mt<mvWEFPoseVelocityTime> gWEPoseQueueWheel2VSLAM;
VirtualWheel_ROS::VirtualWheel_ROS()
{
   ROS_INFO( "start wheel" );
   ros::NodeHandle nh_vslam;

   sub_wheel = nh_vslam.subscribe( "wheel_odom", 10,
                                   &VirtualWheel_ROS::wheelOdomCallback, this );
}

VirtualWheel_ROS::~VirtualWheel_ROS()
{

}

void VirtualWheel_ROS::wheelOdomCallback( const nav_msgs::OdometryConstPtr& msg )
{
   //ROS_INFO("Get wheel odom!!!");
   mvWEFPoseVelocityTime wheelodom;
   wheelodom.timestampUs = msg->header.stamp.toNSec() / 1000;
   wheelodom.pose.translation[0] = msg->pose.pose.position.x;
   wheelodom.pose.translation[1] = msg->pose.pose.position.y;
   wheelodom.pose.translation[2] = msg->pose.pose.position.z;
   double qx = msg->pose.pose.orientation.x;
   double qy = msg->pose.pose.orientation.y;
   double qz = msg->pose.pose.orientation.z;
   double qw = msg->pose.pose.orientation.w;
   double qyy = qy*qy;
   wheelodom.pose.euler[0] = (float)atan2( 2 * (qx*qw + qy*qz), 1 - 2 * (qx*qx + qyy) );
   wheelodom.pose.euler[1] = (float)asin( 2 * (qy*qw - qx*qz) );
   wheelodom.pose.euler[2] = (float)atan2( 2 * (qz*qw + qx*qy), 1 - 2 * (qyy + qz*qz) );
   wheelodom.velocityLinear = msg->twist.twist.linear.x;
   wheelodom.velocityAngular = msg->twist.twist.angular.z;
   gWEPoseQueue.check_push( wheelodom );
   gWEPoseQueueWheel2VSLAM.check_push( wheelodom );
}
/*
bool VirtualWheel::GetWheelOdom( mvWEFPoseVelocityTime& wheelodom )
{

   wheelodomStream >> wheelodom.timestampUs >> wheelodom.pose.translation[0] >> wheelodom.pose.translation[1] >> wheelodom.pose.translation[2];
   double qx, qy, qz, qw;
   wheelodomStream >> qx >> qy >> qz >> qw;
   double qyy = qy*qy;
   wheelodom.pose.euler[0] = (float)atan2( 2 * (qx*qw + qy*qz), 1 - 2 * (qx*qx + qyy) );
   wheelodom.pose.euler[1] = (float)asin( 2 * (qy*qw - qx*qz) );
   wheelodom.pose.euler[2] = (float)atan2( 2 * (qz*qw + qx*qy), 1 - 2 * (qyy + qz*qz) );
   wheelodomStream >> wheelodom.velocityLinear >> wheelodom.velocityAngular;

   return !wheelodomStream.eof();
}
*/

