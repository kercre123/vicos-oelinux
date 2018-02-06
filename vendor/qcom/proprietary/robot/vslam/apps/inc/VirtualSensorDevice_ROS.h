#ifndef  __VIRTUAL_CAMERA_DEVICE_H__
#define __VIRTUAL_CAMERA_DEVICE_H__

/***************************************************************************//**
@file
   VirtualSensorDevice_ROS.h

@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "camera.h"
#include <thread>
#include <image_transport/image_transport.h>

namespace camera
{

   class VirtualSensorDevice_ROS : public ICameraDevice
   {
   public:
      VirtualSensorDevice_ROS();


      ~VirtualSensorDevice_ROS()
      {
         if( imageBuf != NULL )
         {
            delete imageBuf;
         }
      }

      VirtualSensorDevice_ROS( const VirtualSensorDevice_ROS & ) = delete;
      VirtualSensorDevice_ROS operator = ( const VirtualSensorDevice_ROS & ) = delete;

      void addListener( ICameraListener *listener )
      {
         mListener = listener;
      }

      void removeListener( ICameraListener *listener )
      {
         mListener = NULL;
      }

      void subscribe( uint32_t eventMask )
      {

      }

      void unsubscribe( uint32_t eventMask )
      {

      }

      int setParameters( const ICameraParameters& params )
      {
         return 0;
      }

      int getParameters( uint8_t* buf, uint32_t bufSize,
                         int* bufSizeRequired = NULL )
      {
         return 0;
      }

      int startPreview();

      void stopPreview()
      {
         if( imageBuf != NULL )
         {
            delete imageBuf;
            imageBuf = NULL;
         }

      }

      /**
      * Start video recording stream
      *
      * @return int : 0 on success
      */
      int startRecording()
      {
         return 0;
      }

      /**
      * Stop video recording
      */
      void stopRecording()
      {

      }

   private:
      ICameraListener * mListener;
      int sequence;
      void imageReceived( const sensor_msgs::ImageConstPtr & img );
      //void wheelOdomCallback(const nav_msgs::OdometryConstPtr& msg);
      std::string wheelodomName;

      uint8_t * imageBuf;

      image_transport::Subscriber sub;

   };
}
#endif // ! __VIRTUAL_CAMERA_DEVICE_H__

