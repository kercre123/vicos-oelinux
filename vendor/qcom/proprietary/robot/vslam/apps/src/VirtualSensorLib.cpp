/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "mvVWSLAM_app.h"
#include "camera.h"
#ifndef ARM_BASED
#include "camera_parameters_win.h"
#else
#include "camera_parameters.h"
#endif
#ifdef ROS_BASED
#include "VirtualSensorDevice_ROS.h"
#else
#include "VirtualSensorDevice.h"
#endif //ROS_BASED
#include "VirtualCameraFrame.h"

using namespace camera;

#define VIDEO_FPS_30  30

int ICameraDevice::createInstance( int index, ICameraDevice** device )
{
#ifdef ROS_BASED   
   ( *device ) = new VirtualSensorDevice_ROS();
#else   
   (*device) = new VirtualSensorDevice( PLAYBACK_CONFIGURATION );
#endif
   return 0;
}


void ICameraDevice::deleteInstance( ICameraDevice** device )
{
   if( *device != NULL )
   {
      delete * device;
      *device = NULL;
   }
}

int camera::getNumberOfCameras()
{
   int result = 1;
   return result;
}

int camera::getCameraInfo( int idx, struct CameraInfo& info )
{
   info.func = CamFunction::CAM_FUNC_OPTIC_FLOW;
   return 0;
}

CameraParams::CameraParams()
{

}

CameraParams::~CameraParams()
{

}

int CameraParams::init( ICameraDevice* device )
{
   return 0;
}

int CameraParams::commit()
{
   return 0;
}

std::vector<ImageSize> CameraParams::getSupportedPreviewSizes() const
{
   ImageSize size;
   size.width = 640;
   size.height = 480;
   std::vector<ImageSize> result;
   result.push_back( size );
   return result;
}

void CameraParams::setPreviewSize( const ImageSize& /*size*/ )
{

}

std::vector<ImageSize> CameraParams::getSupportedVideoSizes() const
{
   ImageSize size;
   size.width = 640;
   size.height = 480;
   std::vector<ImageSize> result;
   result.push_back( size );
   return result;
}

void CameraParams::setVideoSize( const ImageSize& /*size*/ )
{

}

std::vector<std::string> CameraParams::getSupportedFocusModes() const
{
   std::vector<std::string> result;
   result.push_back( FOCUS_MODE_INFINITY );
   return result;
}

std::string CameraParams::getFocusMode() const
{
   return FOCUS_MODE_INFINITY;
}

void CameraParams::setFocusMode( const std::string& /*value*/ )
{

}

std::vector<std::string> CameraParams::getSupportedWhiteBalance() const
{
   std::vector<std::string> result;
   result.push_back( WHITE_BALANCE_AUTO );
   return result;
}

void CameraParams::setWhiteBalance( const std::string& /*value*/ )
{

}

std::vector<std::string> CameraParams::getSupportedISO() const
{
   std::vector<std::string> result;
   result.push_back( ISO_AUTO );
   return result;
}

void CameraParams::setISO( const std::string& /*value*/ )
{

}

Range CameraParams::getSupportedSharpness() const
{
   Range r;
   return r;
}

Range CameraParams::getSupportedBrightness() const
{
   Range r;
   return r;
}

Range CameraParams::getSupportedContrast() const
{
   Range r;
   return r;
}

std::vector<Range> CameraParams::getSupportedPreviewFpsRanges() const
{
   Range r( 15000, 15000, 0 );
   std::vector<Range> result;
   result.push_back( r );
   return result;
}

void CameraParams::setPreviewFpsRange( struct Range const & )
{

}

std::vector<VideoFPS> CameraParams::getSupportedVideoFps() const
{
   std::vector<VideoFPS> result;
   result.push_back( (enum VideoFPS)VIDEO_FPS_30 );
   return result;
}

void CameraParams::setVideoFPS( VideoFPS /*value*/ )
{

}

int CameraParams::writeObject( std::ostream& ps ) const
{
   return 0;
}

std::string CameraParams::get( const std::string& key ) const
{
   return "";
}

void CameraParams::set( const std::string& key, const std::string& value )
{

}


