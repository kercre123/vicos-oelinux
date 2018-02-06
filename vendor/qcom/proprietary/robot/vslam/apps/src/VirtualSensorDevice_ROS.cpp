/***************************************************************************//**
@file
   VirtualSensorDevice_ROS.cpp

@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#include "VirtualSensorDevice_ROS.h"
#include "VSLAM_internal.h"
#include "Queue.h"
#include "mvWEF.h"
#include "VirtualCameraFrame.h"


extern int sequence;
extern char wheelodomName[200];
extern queue_mt<mvWEFPoseVelocityTime> gWEPoseQueue;

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <camera.h>
#include "mv.h"

#include <fstream>

#ifndef WIN32
#include <unistd.h>
#endif

using namespace camera;

VirtualSensorDevice_ROS::VirtualSensorDevice_ROS()
{
   ROS_INFO( "camera init" );
   sequence = 1;
   imageBuf = NULL;
   mListener = NULL;


}

int VirtualSensorDevice_ROS::startPreview()
{
   ROS_INFO( "start preview" );
   ros::NodeHandle nh_vslam;
   image_transport::ImageTransport it_bag( nh_vslam );
   image_transport::TransportHints hints( "raw", ros::TransportHints(), nh_vslam );
   sub = it_bag.subscribe( "/vslam/image_raw_input", 1, &VirtualSensorDevice_ROS::imageReceived, this, hints );

   imageBuf = new uint8_t[480 * 640 * 5 / 4];

   return 0;
}

void VirtualSensorDevice_ROS::imageReceived( const sensor_msgs::ImageConstPtr & img )
{
   sequence++;

   ROS_INFO( "image received %d", sequence );

   int64_t t = img->header.stamp.toNSec();

   cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare( img );
   cv::Mat raw = imagePtr->image.clone();
   if( img->encoding == sensor_msgs::image_encodings::BGR8 || img->encoding == sensor_msgs::image_encodings::RGB8 )
   {
      cv::cvtColor( imagePtr->image, raw, CV_RGB2GRAY );
   }

   uint8_t * srcImage = (uint8_t *)raw.data;
   uint8_t * desImage = imageBuf;
   uint32_t pixelNum = 480 * 640;
   for( uint32_t i = 0; i < pixelNum; i += 4, srcImage += 4, desImage += 5 )
   {
      memcpy( desImage, srcImage, sizeof( uint8_t ) * 4 );
   }
   VirtualCameraFrame frame( imageBuf, t );
   mListener->onPreviewFrame( &frame );
}
