/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#include "VSLAMScheduler.h"
#include "mainThread.h"
#include <thread>
#include "Queue.h"

#include "Motion.h"
#include "WEF.h"

#include "VSLAM.h"
#include "Camera_VSLAM.h"
#include "mvVWSLAM_app.h"

#include "ScaleEstimation.h"

#include "mv.h"
#include "mvCPA.h"

#include "ReadImages.h"

#include "Visualization.h"
#include "VirtualCameraFrame.h"
#include <signal.h>

bool THREAD_RUNNING = true;

mvWEF *gWEF = NULL;
Camera_VSLAM * vslamCamera = NULL;

bool              mainThreadExited = false;

void     INThandler( int );

VSLAMParameter vslamPara;
WEFParameter   wefPara;
int32_t ParseCameraParameters( const char * parameterFile, VSLAMCameraParams & eagleCameraParams );
void ParseScaleEstimationParameters( const char * parameterFile, ScaleEstimator & scaleEstimationParams, const VSLAMParameter currentPara );

extern void cameraProc();
extern void cameraSecondaryProc();

extern queue_mt<mvWEFPoseVelocityTime> gWEPoseQueueWheel2VSLAM;

MapFocuser* mapFocuser;
MapFocuser* mapFocuserSecondary;

/**--------------------------------------------------------------------------
@brief
Queue for wheel encoder pose, vslam pose and fusion pose
Used for motion pattern
--------------------------------------------------------------------------**/
extern queue_mt<mvWEFPoseVelocityTime> gMotionWEPoseQueue;
extern queue_mt<mvWEFPoseStateTime> gMotionVSLAMPoseQueue;
extern queue_mt<mvPose6DRT> gMotionFsuionPoseQueue;

void wefProc()
{
   while( true )
   {
      if( THREAD_RUNNING == false )
      {
         break;
      }
      mvWEFPoseVelocityTime fusionPose = GetFusionPose( gWEF );
      gFsuionPoseQueue.check_push( fusionPose ); // no pop from this queue in this app, interface for other apps

      mvPose6DRT fusionPoseInVslam;
      bool result = mvWEF_RecoverPose( gWEF, fusionPoseInVslam );
      if( result )
      {
         gMotionFsuionPoseQueue.check_push( fusionPoseInVslam );
      }

      visualiser->PublishRobotPose( fusionPose );
   }

   printf( "wefProc Thread exit\n" );
}

void publishState( const char * str, int64_t timestampUs )
{
   char strTemp[100];
   sprintf( strTemp, "%lld,%s", timestampUs, str );
   visualiser->PublishVSLAMSchedulerState( std::string( strTemp ) );
}

void vslamProc()
{
   StartVSLAM();

   if( vslamCamera )
   {
      vslamCamera->start();
   }
   else
   {
      printf( "Camera object is not created.\n" );
      return;
   }

   VSLAMPoseWithFeedback pose = VSLAMPoseWithFeedback( mvWEFPoseStateTime() );
   VSLAMScheduler *instance = VSLAMScheduler::getInstance();
   while( true )
   {
      if( THREAD_RUNNING == false )
      {
         break;
      }

      switch( instance->getState() )
      {
         case VSLAMScheduler::kSTATE_PRIMARY:
         {
            gVSLAMPoseRawQueue.wait_and_pop( pose );
            if( VSLAMScheduler::kFB_MAPEXPORTED == pose.feedback )
            {
               instance->setState( VSLAMScheduler::kSTATE_SECONDARY_IMPORT );
               instance->activateSecondary();
            }
            if( gScaleEstimator.getScaleEstimationStatus() == ScaleEstimationStatus::IDLE )
               gVSLAMPoseQueue.check_push( pose.pose );

            gMotionVSLAMPoseQueue.check_push( pose.pose );

            publishState( "kSTATE_PRIMARY", pose.pose.timestampUs );
            break;
         }
         case VSLAMScheduler::kSTATE_SECONDARY_IMPORT:
         {
            gVSLAMPoseRawSecondaryQueue.wait_and_pop( pose );

            if( VSLAMScheduler::kFB_MAPIMPORTED == pose.feedback )
            {
               instance->setState( VSLAMScheduler::kSTATE_CONCURRENT );
            }
            gVSLAMPoseRawQueue.wait_and_pop( pose );

            gVSLAMPoseQueue.check_push( pose.pose );
            gMotionVSLAMPoseQueue.check_push( pose.pose );

            publishState( "kSTATE_SECONDARY_IMPORT", pose.pose.timestampUs );
            break;
         }
         case VSLAMScheduler::kSTATE_CONCURRENT:
         {
            VSLAMPoseWithFeedback poseSecondary = VSLAMPoseWithFeedback( mvWEFPoseStateTime() );
            gVSLAMPoseRawSecondaryQueue.wait_and_pop( poseSecondary );
            gVSLAMPoseRawQueue.wait_and_pop( pose );

            gMotionVSLAMPoseQueue.check_push( pose.pose );
            if( VSLAMScheduler::kFB_SCALEACQUIRED == pose.feedback && VSLAMScheduler::kFB_RELOCATED == poseSecondary.feedback )
            {
               //TODO: select between initialization and relocation
               instance->setState( VSLAMScheduler::kSTATE_SECONDARY );
               instance->deactivatePrimary();

               gVSLAMPoseQueue.check_push( poseSecondary.pose );
               gMotionVSLAMPoseQueue.check_push( poseSecondary.pose );

               publishState( "kSTATE_CONCURRENT_s", poseSecondary.pose.timestampUs );
            }
            else
            {
               if( VSLAMScheduler::kFB_SCALEACQUIRED == pose.feedback )
               {
                  instance->setState( VSLAMScheduler::kSTATE_PRIMARY );
                  instance->deactivateSecondary();
                  gVSLAMPoseQueue.check_push( pose.pose );
                  publishState( "kSTATE_CONCURRENT_p", pose.pose.timestampUs );
               }
               else
               {
                  if( VSLAMScheduler::kFB_RELOCATED == poseSecondary.feedback )
                     instance->setState( VSLAMScheduler::kSTATE_SECONDARY );
                  gVSLAMPoseQueue.check_push( poseSecondary.pose );
                  publishState( "kSTATE_CONCURRENT_s", poseSecondary.pose.timestampUs );
               }
            }
            break;
         }
         case VSLAMScheduler::kSTATE_SECONDARY:
         {
            if( ScaleEstimationStatus::IDLE == gScaleEstimator.getScaleEstimationStatus() )
            {
               // make sure primary vslam in IDLE state if deactivated
               instance->deactivatePrimary();
            }

            gVSLAMPoseRawSecondaryQueue.wait_and_pop( pose );
            if( VSLAMScheduler::kFB_MAPEXPORTED == pose.feedback )
            {
               assert( MV_VSLAM_TRACKING_STATE_FAILED == pose.pose.poseWithState.poseQuality );
               instance->setState( VSLAMScheduler::kSTATE_PRIMARY_IMPORT );
               instance->activatePrimary();
            }

            gVSLAMPoseQueue.check_push( pose.pose );
            gMotionVSLAMPoseQueue.check_push( pose.pose );
            publishState( "kSTATE_SECONDARY", pose.pose.timestampUs );
            break;
         }
         case VSLAMScheduler::kSTATE_PRIMARY_IMPORT:
         {
            gVSLAMPoseRawQueue.wait_and_pop( pose );

            if( VSLAMScheduler::kFB_MAPIMPORTED == pose.feedback )
            {
               instance->setState( VSLAMScheduler::kSTATE_CONCURRENT );
            }
            gVSLAMPoseRawSecondaryQueue.wait_and_pop( pose );

            gVSLAMPoseQueue.check_push( pose.pose );
            gMotionVSLAMPoseQueue.check_push( pose.pose );
            publishState( "kSTATE_PRIMARY_IMPORT", pose.pose.timestampUs );
            break;
         }
         default:
            break;
      }
   }

   gMotionPatternQueue.check_push( MOTION_PATTERN::EXIT_PATTERN );

   vslamCamera->stop();

   printf( "vslamproc Thread exit\n" );
}


void motionProc()
{
   //Execise the motion pattern
   while( THREAD_RUNNING )
   {
      MOTION_PATTERN motion_pattern;
      gMotionPatternQueue.wait_and_pop( motion_pattern );
      setMotionPattern( motion_pattern ); 
      MOTION_PATTERN n_motion_pattern = motion_pattern;
   }
   printf( "motionProc Thread exit\n" );
}


void mainProc()
{
   printf( "\nMV version: %s\n\n", mvVersion() );
   THREAD_RUNNING = true;
   int re = ParseVWSLAMConf( VWSLAM_Configuration, vslamPara, wefPara );
   if( re != 0 )
   {
      return;
   }
   signal( SIGINT, INThandler );

   mapFocuser = new MapFocuser;
   mapFocuserSecondary = new MapFocuser;
   mapFocuser->SetCrossCalibrationMatrix(vslamPara.baselinkInVSLAM[0]);
   mapFocuserSecondary->SetCrossCalibrationMatrix(vslamPara.baselinkInVSLAM[0]);

   VSLAMCameraParams eagleCameraParams;
   if( ParseCameraParameters( "Configuration/vslam.cfg", eagleCameraParams ) != 0 )
   {
      return;
   }

   vslamCamera = new Camera_VSLAM();
   vslamCamera->findClocksOffsetForCamera();
   vslamCamera->setCaptureParams( eagleCameraParams );
   if( !vslamCamera->init() )
   {
      printf( "Error in camera.init()!\n" );
      return;
   }
   vslamCamera->addCallback( VSLAMCameraCallback );

   mvCameraConfiguration & vslamCameraConfig = vslamPara.vslamCameraConfig;

   vslamCameraConfig.pixelWidth = eagleCameraParams.outputPixelWidth;
   vslamCameraConfig.pixelHeight = eagleCameraParams.outputPixelHeight;
   vslamCameraConfig.memoryStride = vslamCameraConfig.pixelWidth;
   vslamCameraConfig.principalPoint[0] = eagleCameraParams.outputCameraMatrix[2];
   vslamCameraConfig.principalPoint[1] = eagleCameraParams.outputCameraMatrix[5];
   vslamCameraConfig.focalLength[0] = eagleCameraParams.outputCameraMatrix[0];
   vslamCameraConfig.focalLength[1] = eagleCameraParams.outputCameraMatrix[4];
   vslamCameraConfig.distortion[0] = 0;
   vslamCameraConfig.distortion[1] = 0;
   vslamCameraConfig.distortion[2] = 0;
   vslamCameraConfig.distortion[3] = 0;
   vslamCameraConfig.distortion[4] = 0;
   vslamCameraConfig.distortion[5] = 0;
   vslamCameraConfig.distortion[6] = 0;
   vslamCameraConfig.distortion[7] = 0;
   vslamCameraConfig.distortionModel = 0;

   gWEF = StartFS( &wefPara.poseVB, RELOCALIZATION == vslamPara.initMode, wefPara.vslamStateBadAsFail, vslamPara.useExternalConstraint );
   InitializeVSLAM( vslamPara );

   SetInitMode( vslamPara, "8009Map" );
   if( vslamPara.alwaysOnRelocation )
   {
      SetInitModeSecondary();
   }

   ParseScaleEstimationParameters( "Configuration/vslam.cfg", gScaleEstimator, vslamPara ); 
    
   std::thread wefThread( wefProc );
   std::thread cameraThread( cameraProc );
   std::thread cameraSecondaryThread( cameraSecondaryProc );
   std::thread vslamThread( vslamProc );
   std::thread motionThread( motionProc );
   wefThread.join();
   cameraThread.join();
   cameraSecondaryThread.join();
   vslamThread.join();
   motionThread.join();

   GetPointCloud( "8009Map" );
   printf( "before ReleaseVSLAM\n" );
   ReleaseVSLAM();
   printf( "after ReleaseVSLAM\n" );
   ReleaseFS( gWEF );

   delete vslamCamera;
   delete mapFocuser;
   delete mapFocuserSecondary;

   mainThreadExited = true;
}

void  INThandler( int sig )
{
   printf( "Ctrl-C vwslam\n" );
   exitMainThread();
}

void exitMainThread()
{
   THREAD_RUNNING = false;

   StopVSLAM();

   mvWEFPoseVelocityTime wheelodom{ 0 };
   wheelodom.timestampUs = 0;
   gWEPoseQueueWheel2VSLAM.check_push( wheelodom );

   if( vslamCamera )
   {
      uint8_t image[16];
      uint64_t t = 0;
      VirtualCameraFrame frame( image, t );
      vslamCamera->onPreviewFrame( &frame );
   }

   gWEPoseQueue.check_push( wheelodom );

   mvWEFPoseVelocityTime curWEPose{ 0 };
   gMotionWEPoseQueue.check_push( curWEPose );
}


