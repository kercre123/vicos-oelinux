/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef  __VIRTUAL_CAMERA_DEVICE_H__
#define __VIRTUAL_CAMERA_DEVICE_H__


#include "camera.h"
#include <thread>
#include <mutex>

class VirtualWheel;

namespace camera
{

   class VirtualSensorDevice : public ICameraDevice
   {
   public:
      VirtualSensorDevice( const char * configureFile );

      ~VirtualSensorDevice();

      VirtualSensorDevice( const VirtualSensorDevice & ) = delete;
      VirtualSensorDevice operator = ( const VirtualSensorDevice & ) = delete;

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

      void* getParameters()
      {
         return NULL;
      }

      int takePicture(uint32_t num_images)
      {
         return 0;
      }

      int startPreview();

      void stopPreview();
     

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

      std::thread cameraThread;
      void virtualSensorDeviceProc();
      bool ParsePlaybackParameters(const char *configFile);
      int sequence;
      std::vector<std::string> imageList;
      uint32_t  lastImageIndex;
      uint32_t  curImageIndex;

      VirtualWheel * mWheel;
      std::string wheelodomName;

      std::mutex cameraMutex;
   };
}
#endif // ! __VIRTUAL_CAMERA_DEVICE_H__

