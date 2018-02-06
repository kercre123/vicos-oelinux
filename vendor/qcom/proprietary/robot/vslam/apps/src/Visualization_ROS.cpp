/***************************************************************************//**
@file
   Visualization_ROS.cpp

@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "Visualization_ROS.h"
#include "opencv2/opencv.hpp"

#include <cv_bridge/cv_bridge.h>

#include <VSLAM.h>
#include <VSLAM_internal.h>
#include "mvvwslam_ros/OdometryVSLAM.h"
#include <ros/ros.h>
static void EtoQuaternion( double pitch, double roll, double yaw, double quaternion[4] );

Visualiser_ROS::Visualiser_ROS( const char * outputPath, uint32_t outputEveryNFrame ): Visualiser(outputPath)
{
   std::string output( outputPath );
   fpLogWEFInput = openLogFile( (output + "fusion_input.csv").c_str() );

   ros::NodeHandle nh_vslam;
   image_transport::ImageTransport it( nh_vslam );
   labelledImagePub = it.advertise( "/vslam/image_labelled", 1 );

   pub_fusion = nh_vslam.advertise<nav_msgs::Odometry>( "robot_odom", 10 );

   pub_state = nh_vslam.advertise<std_msgs::String>( "vslam_state", 10 );
   pub_cpaParameter  = nh_vslam.advertise<std_msgs::String>( "cpa_parameter", 10 );

   pubCameraOdom = nh_vslam.advertise<mvvwslam_ros::OdometryVSLAM>( "vslam_odom", 33 );
   pubVSLAMOdom_Raw = nh_vslam.advertise<nav_msgs::Odometry>( "vslam_odom_raw", 33 );
   cameraOdomCount = 0;

   pubCorrectedCameraOdom = nh_vslam.advertise<nav_msgs::Odometry>( "vslam_odom_unbiased", 33 );
   correctedCameraOdomCount = 0;

   odom_.child_frame_id = "base_link";
   odom_.header.frame_id = "odom";
   odom_.header.seq = 0;
   odom_.twist.twist.linear.y = 0;
   odom_.twist.twist.linear.z = 0;
   odom_.twist.twist.angular.x = 0;
   odom_.twist.twist.angular.y = 0;
   odom_.pose.covariance.fill( 0 );
   odom_.twist.covariance.fill( 0 );

   originalImagePub = it.advertise( "/vslam/image_raw", 1 );
   undistortedImagePub = it.advertise( "/vslam/image_undistorted", 1 );

   frameIndex = 0;
   _outputEveryNFrame = outputEveryNFrame;
}

Visualiser_ROS::~Visualiser_ROS()
{

}

void Visualiser_ROS::ShowPoints( const mvWEFPoseStateTime & poseWithTime, const uint8_t * image, int imageWidth, int imageHeight  )
{
   frameIndex++;
   if( frameIndex < _outputEveryNFrame )
   {
      return;
   }
   cv::Mat rview;
   DrawLabelledImage( poseWithTime, image, imageWidth, imageHeight, rview );

   if( labelledImagePub.getNumSubscribers() > 0 ) //  && (frameIndex % frameSkip) == 0)
   {
      //VSLAM_INFO("send image frame index = %d, stamp = %f", frameIndex, (double)timestamp*1e-9);
      sensor_msgs::ImagePtr image = cv_bridge::CvImage( std_msgs::Header(), "bgr8", rview ).toImageMsg();
      image->header.stamp.fromNSec( poseWithTime.timestampUs * 1000 );

      labelledImagePub.publish( *image );
   }
   frameIndex = 0;
}

void Visualiser_ROS::PublishOriginalImage( uint64_t stamp, const uint8_t * image, int imageWidth, int imageHeight )
{
   cv::Mat rview;
   GetOriginalImage( image, rview, imageWidth, imageHeight );
   if( originalImagePub.getNumSubscribers() > 0 ) //  && (frameIndex % frameSkip) == 0)
   {
      //VSLAM_INFO("send image frame index = %d, stamp = %f", frameIndex, (double)timestamp*1e-9);
      sensor_msgs::ImagePtr image = cv_bridge::CvImage( std_msgs::Header(), "mono8", rview ).toImageMsg();
      image->header.stamp.fromNSec( stamp );

      originalImagePub.publish( *image );
   }
}

void Visualiser_ROS::PublishUndistortedImage( const uint64_t stamp, const uint8_t * image, int imageWidth, int imageHeight )
{
   if( undistortedImagePub.getNumSubscribers() > 0 ) //  && (frameIndex % frameSkip) == 0)
   {
      cv::Mat rview( imageHeight, imageWidth, CV_8UC1 );
      memcpy( rview.data, image, imageHeight * imageWidth );    
      sensor_msgs::ImagePtr image = cv_bridge::CvImage( std_msgs::Header(), "mono8", rview ).toImageMsg();
      image->header.stamp.fromNSec( stamp );

      undistortedImagePub.publish( *image );
   }
}

void Visualiser_ROS::InitOdom( nav_msgs::Odometry & odom )
{
   odom.twist.twist.linear.x = 0;
   odom.twist.twist.linear.y = 0;
   odom.twist.twist.linear.z = 0;
   odom.twist.twist.angular.x = 0;
   odom.twist.twist.angular.y = 0;
   odom.twist.twist.angular.z = 0;
   odom.pose.covariance.fill( 0 );
   odom.twist.covariance.fill( 0 );
}

void Visualiser_ROS::PublishRobotPose( const mvWEFPoseVelocityTime & pose )
{
   odom_.header.seq++;
   uint64_t time;
   if( pose.timestampUs < 0 )
   {
      time = 0;
   }
   else
   {
      time = pose.timestampUs * 1000;
   }
   odom_.header.stamp.fromNSec( time );
   InitOdom( odom_ );
   odom_.child_frame_id = "base_link";
   odom_.header.frame_id = "odom";
   odom_.pose.pose.position.x = pose.pose.translation[0];
   odom_.pose.pose.position.y = pose.pose.translation[1];
   odom_.pose.pose.position.z = pose.pose.translation[2];
   double quaternion[4];
   EtoQuaternion( pose.pose.euler[0], pose.pose.euler[1], pose.pose.euler[2], quaternion );
   odom_.pose.pose.orientation.x = quaternion[1];
   odom_.pose.pose.orientation.y = quaternion[2];
   odom_.pose.pose.orientation.z = quaternion[3];
   odom_.pose.pose.orientation.w = quaternion[0];
   odom_.twist.twist.linear.x = pose.velocityLinear;
   odom_.twist.twist.angular.z = pose.velocityAngular;
   pub_fusion.publish( odom_ );

}

static void EtoQuaternion( double pitch, double roll, double yaw, double quaternion[4] )
{
   double t0 = std::cos( yaw * 0.5 );
   double t1 = std::sin( yaw * 0.5 );
   double t2 = std::cos( roll * 0.5 );
   double t3 = std::sin( roll * 0.5 );
   double t4 = std::cos( pitch * 0.5 );
   double t5 = std::sin( pitch * 0.5 );

   quaternion[0] = t0 * t2 * t4 + t1 * t3 * t5;  //w
   quaternion[1] = t0 * t3 * t4 - t1 * t2 * t5;  //x
   quaternion[2] = t0 * t2 * t5 + t1 * t3 * t4;  //y
   quaternion[3] = t1 * t2 * t4 - t0 * t3 * t5;  //z
}

void Visualiser_ROS::PublishCameraPose( const mvWEFPoseStateTime & pose, const uint8_t * image, int imageWidth, int imageHeight )
{
   mvvwslam_ros::OdometryVSLAM cameraOdom;

   cameraOdom.odom.header.seq = cameraOdomCount++;
   cameraOdom.odom.child_frame_id = "camera";
   cameraOdom.odom.header.frame_id = "odom";

   InitOdom( cameraOdom.odom );

   uint64_t time;
   if( pose.timestampUs < 0 )
   {
      time = 0;
   }
   else
   {
      time = pose.timestampUs * 1000;
   }
   cameraOdom.odom.header.stamp.fromNSec( time );

   if( image != NULL )
   {
      GetVSLAMStatus( image, imageWidth, imageHeight );
   }

   mvPose6DRT2ROSTopic( pose.poseWithState.pose, cameraOdom.odom );

   cameraOdom.matchedPointNum = status._MatchedMapPointNum;
   cameraOdom.mismatchedPointNum = status._MisMatchedMapPointNum;
   cameraOdom.meanIllumination = status._BrightnessMean;
   cameraOdom.stddevIllumination = status._BrightnessVar;
   cameraOdom.state = pose.poseWithState.poseQuality;

   nav_msgs::Odometry odom;
   odom = cameraOdom.odom;
   pubCameraOdom.publish( cameraOdom );
   pubVSLAMOdom_Raw.publish( odom );

   std::string state_string;
   switch( pose.poseWithState.poseQuality )
   {
      case MV_VSLAM_TRACKING_STATE_FAILED:
         state_string = "TRACKING_FAILED";
         break;
      case MV_VSLAM_TRACKING_STATE_INITIALIZING:
         state_string = "INITIALIZING";
         break;
      case MV_VSLAM_TRACKING_STATE_GREAT:
      case MV_VSLAM_TRACKING_STATE_GOOD:
      case MV_VSLAM_TRACKING_STATE_OK:
         state_string = "GOOD_QUALITY";
         break;
      case MV_VSLAM_TRACKING_STATE_BAD:
      case MV_VSLAM_TRACKING_STATE_APPROX:
         state_string = "BAD_QUALITY";
         break;
      case MV_VSLAM_TRACKING_STATE_RELOCATED:
      default:
         state_string = "RELOCATED";
   }
   std_msgs::String stateMsg;
   stateMsg.data = state_string.c_str();
   pub_state.publish( stateMsg );

}

void Visualiser_ROS::PublishCorrectedCameraPose( const mvWEFPoseStateTime & pose )
{
   odom_.header.seq = correctedCameraOdomCount++;

   uint64_t time;
   if( pose.timestampUs < 0 )
   {
      time = 0;
   }
   else
   {
      time = pose.timestampUs * 1000;
   }
   odom_.header.stamp.fromNSec( time );
   InitOdom( odom_ );
   mvPose6DRT2ROSTopic( pose.poseWithState.pose, odom_ );
   pubCorrectedCameraOdom.publish( odom_ );
}

void Visualiser_ROS::mvPose6DRT2ROSTopic( const mvPose6DRT & pose, nav_msgs::Odometry &  odom )
{
   odom.pose.pose.position.x = pose.matrix[0][3];
   odom.pose.pose.position.y = pose.matrix[1][3];
   odom.pose.pose.position.z = pose.matrix[2][3];
   double quaternion[4];
   RtoQuaternion( pose.matrix, quaternion );   

   odom.pose.pose.orientation.x = quaternion[1];
   odom.pose.pose.orientation.y = quaternion[2];
   odom.pose.pose.orientation.z = quaternion[3];
   odom.pose.pose.orientation.w = quaternion[0];
}

void Visualiser_ROS::PublishExposureGain( float32_t exposure, float32_t gain, int exposureValue, int gainValue, float mean_brightness )
{
   char strCPA[200];
   sprintf( strCPA, "Exposure:%f,Gain:%f, real exposure:%d, real gain:%d, mean_brightness:%f", exposure, gain, exposureValue,  gainValue, mean_brightness );
   std::string showCPA( strCPA );
   std_msgs::String cpa_status;
   cpa_status.data = showCPA;
   pub_cpaParameter.publish( cpa_status );
}
