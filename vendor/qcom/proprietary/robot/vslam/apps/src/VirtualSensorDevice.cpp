/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "VirtualSensorDevice.h"
#include "VirtualWheel.h"
#include "VirtualCameraFrame.h"
#include "mvVWSLAM_app.h"
#include "Queue.h"
#include "mvWEF.h"


extern std::vector<std::string> imageList;
extern int sequence;

extern queue_mt<mvWEFPoseVelocityTime> gWEPoseQueue;
extern queue_mt<mvWEFPoseVelocityTime> gWEPoseQueueWheel2VSLAM;
extern queue_mt<mvWEFPoseVelocityTime> gMotionWEPoseQueue;

#include <opencv2/opencv.hpp>

#include <camera.h>
#include "mv.h"

#include <fstream>

#ifndef WIN32
#include <unistd.h>
#endif

using namespace camera;
extern bool gVSALMRunning;
extern VSLAMParameter vslamPara;

VirtualSensorDevice::VirtualSensorDevice( const char * configureFile )
{
   sequence = 0;
   ParsePlaybackParameters( configureFile );
   mListener = NULL;
   mWheel = NULL;
   lastImageIndex = 0;
   curImageIndex = 0;

   if(vslamPara.wheelEnabled)
     mWheel = new VirtualWheel( wheelodomName );
}

VirtualSensorDevice::~VirtualSensorDevice()
{
   cameraMutex.lock();
   if( mWheel != NULL )
   {
      delete mWheel;
   }
   cameraMutex.unlock();
}
int VirtualSensorDevice::startPreview()
{
   cameraThread = std::thread( &VirtualSensorDevice::virtualSensorDeviceProc, this );
   cameraThread.detach();
   return 0;
}

void VirtualSensorDevice::stopPreview()
{
   uint32_t temp = curImageIndex + 10;
   if( temp > imageList.size() )
   {
      temp = imageList.size();
   }

   lastImageIndex = temp;

   //while( lastImageIndex > curImageIndex )
   //{
   //   VSLAM_MASTER_SLEEP( SleepTimeInMillisecond );
   //}

}

void VirtualSensorDevice::virtualSensorDeviceProc()
{
   cv::Mat image, iImage;
   size_t indexDot, index;
   int64_t timeStamp = 0;
   std::string stamp;
   bool isWheelodomValid = true; //change to false only when wheel.txt contains some errors. If there is no wheel.txt, its value keeps untouched

   const uint32_t imageWidth = 640;
   const uint32_t imageHeight = 480;
   const uint32_t pixelNum = imageWidth * imageHeight;
   uint8_t * imageBuf = new uint8_t[pixelNum / 4 * 5]; //RAW_FORMAT
   memset( imageBuf, 0, pixelNum / 4 * 5 );

   cameraMutex.lock();
   mvWEFPoseVelocityTime wheelodom;
   lastImageIndex = imageList.size();

   if(vslamPara.wheelEnabled)
   {
     for( curImageIndex = sequence; curImageIndex < lastImageIndex && isWheelodomValid; curImageIndex++ )
     {

      image = cv::imread( imageList[curImageIndex] );
      printf( "%s\n", imageList[curImageIndex].c_str() );
      if( nullptr == image.data )
      {
         printf( "Cannot read image with name: %s\n", imageList[curImageIndex].c_str() );
         imageList.resize( curImageIndex );
         break;
      }

      if( image.channels() != 1 )
      {
         cv::cvtColor( image, iImage, cv::COLOR_BGR2GRAY );
      }
      else
      {
         iImage = image;
      }
      uint8_t * srcImage = (uint8_t *)iImage.data;
      uint8_t * desImage = imageBuf;
      for( uint32_t i = 0; i < pixelNum && srcImage != NULL; i += 4, srcImage += 4, desImage += 5 )
      {
         memcpy( desImage, srcImage, sizeof( uint8_t ) * 4 );
      }

      indexDot = imageList[curImageIndex].find_last_of( '.' );
      index = imageList[curImageIndex].find_last_of( '_' );
      stamp = imageList[curImageIndex].substr( index + 1, indexDot - index - 1 );
      timeStamp = std::stoll( stamp );

      // skip images if wheel is ahead of camera too much
      static bool frameSkiped = false;
      if( mWheel && !frameSkiped )
      {
         frameSkiped = true;
         bool isWheelodomValid = mWheel->GetWheelOdom( wheelodom );
         if( isWheelodomValid )
         {
            int64_t timeDelta = timeStamp - wheelodom.timestampUs; // timestampUs is actually in Nanosecond here
            if( timeDelta < 0 )
            {
               curImageIndex += (uint32_t)(-timeDelta) / 66000000;
               continue;
            }
         }
      }


      VirtualCameraFrame frame( (uint8_t *)imageBuf, timeStamp );
      mListener->onPreviewFrame( &frame );

      if( mWheel )
      {
         // reading wheel odom data
         int64_t timeDelta;

         do
         {
            isWheelodomValid = mWheel->GetWheelOdom( wheelodom );
            if( isWheelodomValid )
            {
               timeDelta = timeStamp - wheelodom.timestampUs; // timestampUs is actually in Nanosecond here
               wheelodom.timestampUs /= 1000;
               gWEPoseQueueWheel2VSLAM.check_push( wheelodom );
               gWEPoseQueue.check_push( wheelodom );
            }
            else
            {
               delete mWheel;
               timeDelta = 0 - 1;
               mWheel = NULL;
               printf( "Close file %s for wheel odom reading!\n", wheelodomName.c_str() );
            }
         } while( timeDelta > -20000000 && isWheelodomValid );
      }

      VSLAM_MASTER_SLEEP( 50 ); // sleep to make sure both primary and secondary can access g_ImageBuf

      if( !gVSALMRunning )
          break;
     }
   }
   //infinite mode without wheel
   else
   {
       uint32_t img_idx = 0;
       uint32_t loop_time;

       for(loop_time = 0; loop_time < 10000; loop_time++ ) //more than 24 hours is enough
       {
          //from begin to end
          for( curImageIndex = sequence; curImageIndex < lastImageIndex; curImageIndex++ )
          {
           image = cv::imread( imageList[curImageIndex] );
           printf( "%s\n", imageList[curImageIndex].c_str() );
           if( nullptr == image.data )
           {
              printf( "Cannot read image with name: %s\n", imageList[curImageIndex].c_str() );
              imageList.resize( curImageIndex );
              break;
           }

           if( image.channels() != 1 )
           {
              cv::cvtColor( image, iImage, cv::COLOR_BGR2GRAY );
           }
           else
           {
              iImage = image;
           }
           uint8_t * srcImage = (uint8_t *)iImage.data;
           uint8_t * desImage = imageBuf;
           for( uint32_t i = 0; i < pixelNum; i += 4, srcImage += 4, desImage += 5 )
           {
              memcpy( desImage, srcImage, sizeof( uint8_t ) * 4 );
           }

           img_idx++;
           timeStamp = 66000ll * img_idx;

           VirtualCameraFrame frame( (uint8_t *)imageBuf, timeStamp );
           mListener->onPreviewFrame( &frame );

           VSLAM_MASTER_SLEEP( 50 ); // sleep to make sure both primary and secondary can access g_ImageBuf

           if( !gVSALMRunning )
               goto stop;
          }

          //from end to begin
          for( curImageIndex = lastImageIndex -1 ; curImageIndex >= 0; curImageIndex-- )
          {
           image = cv::imread( imageList[curImageIndex] );
           printf( "%s\n", imageList[curImageIndex].c_str() );
           if( nullptr == image.data )
           {
              printf( "Cannot read image with name: %s\n", imageList[curImageIndex].c_str() );
              imageList.resize( curImageIndex );
              break;
           }

           if( image.channels() != 1 )
           {
              cv::cvtColor( image, iImage, cv::COLOR_BGR2GRAY );
           }
           else
           {
              iImage = image;
           }
           uint8_t * srcImage = (uint8_t *)iImage.data;
           uint8_t * desImage = imageBuf;
           for( uint32_t i = 0; i < pixelNum; i += 4, srcImage += 4, desImage += 5 )
           {
              memcpy( desImage, srcImage, sizeof( uint8_t ) * 4 );
           }

           img_idx++;
           timeStamp = 66000ll * img_idx;

           VirtualCameraFrame frame( (uint8_t *)imageBuf, timeStamp );
           mListener->onPreviewFrame( &frame );

           VSLAM_MASTER_SLEEP( 50 ); // sleep to make sure both primary and secondary can access g_ImageBuf

           if( !gVSALMRunning )
               goto stop;
          }
       }
   }

stop:
   exitMainThread();
   delete[] imageBuf;

   cameraMutex.unlock();
   return;
}

bool VirtualSensorDevice::ParsePlaybackParameters( const char *configFile )
{
   bool sequenceGot = false, trajectoryGot = false, mapGot = false;

   std::string sequenceName;

   printf( "\nParsing paths......\n" );
   std::string fullName = Program_Root + configFile;
   std::ifstream cfg( fullName, std::ifstream::in );

   if( !cfg.is_open() )
   {
      printf( "Fail to open configuration file: %s\n", fullName.c_str() );
   }

   std::string line;
   std::string itemName;
   while( std::getline( cfg, line ) )
   {
      if( line.length() == 0 )
      {
         continue;
      }
      if( line[0] == '#' )
      {
         continue;
      }
      std::istringstream iss( line );
      itemName.clear();
      iss >> itemName;
      if( itemName.compare( "WheelOdom" ) == 0 )
      {
         iss >> wheelodomName;
         wheelodomName = Program_Root + wheelodomName;
         printf( "Using wheelOdom:       %s\n", wheelodomName.c_str() );
      }
      else if( itemName.compare( "Sequence" ) == 0 )
      {
         sequenceGot = true;
         iss >> sequenceName;
         sequenceName = Program_Root + sequenceName;
         printf( "Using sequence:       %s\n", sequenceName.c_str() );
      }
      else if( itemName.compare( "InitialIndex" ) == 0 )
      {
         iss >> sequence;
         printf( "Starting frame index is:       %d\n", sequence );
      }
   }
   cfg.close();

   cfg.open( sequenceName, std::ifstream::in );
   if( !cfg.is_open() )
   {
      printf( "Fail to open image list file!!!\n" );
   }
   printf( "Open image name list file: %s\n", sequenceName.c_str() );

   std::string root = sequenceName;

   // align all path separator into '/'
   static std::string search( "\\" ), replace( "/" );
   for( size_t pos = 0; pos = root.find( search, pos ), pos != std::string::npos; pos += replace.length() )
   {
      root.replace( pos, search.length(), replace );
   }

   std::string temp;
   size_t index = root.find_last_of( '/' );

   root = root.substr( 0, index + 1 );
   while( std::getline( cfg, line ) )
   {
      if( line.length() == 0 )
      {
         continue;
      }
      if( line[0] == '#' )
      {
         continue;
      }
      temp = root + line;
      imageList.push_back( temp );
      //if( imageList.size() > 100 )
      //{
      //   break;
      //}
   }
   cfg.close();


   if( !sequenceGot )
   {
      printf( "Error: not provide required path of Sequence!\n" );
      return false;
   }

   if( !mapGot )
   {
      printf( "Error: not provide required path of Map!\n" );
      return false;
   }

   return true;
}
