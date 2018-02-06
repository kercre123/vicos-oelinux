/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
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
//static void exitMainThread();

VSLAMParameter vslamPara;
WEFParameter   wefPara;
int32_t ParseCameraParameters( const char * parameterFile, VSLAMCameraParams & eagleCameraParams );
extern void cameraProc();

int32_t failPoseNum = 100;

extern queue_mt<mvWEFPoseVelocityTime> gWEPoseQueueWheel2VSLAM;

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

void vslamProc()
{
   int failCnt = 0;

   StartVSLAM();

   gScaleEstimator.initialize();

   if( vslamCamera )
   {
      vslamCamera->start();
   }
   else
   {
      printf( "Camera object is not created.\n" );
      return;
   }

   while( true )
   {
      if( THREAD_RUNNING == false )
      {
         break;
      }

      // Get vslam pose
      mvWEFPoseStateTime pose = GetPose();

      if( pose.timestampUs == 0 )
      {
         continue;
      }
      else if( pose.timestampUs == -1 )
      {
         exitMainThread();
         continue;
      }

      // Change the save map flag from false to true in case 
      //   target-based initialization with the high quality VSLAM pose 
      if( gScaleEstimator.isSaveMapFlag == false
          && (pose.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_GREAT ||
               pose.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_GOOD ||
               pose.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_OK)
          && gScaleEstimator.getScaleEstimationStatus() == ScaleEstimationStatus::IDLE )
      {
         gScaleEstimator.isSaveMapFlag = true;
      }

      gVSLAMPoseQueue.check_push( pose );
      gMotionVSLAMPoseQueue.check_push( pose );

      switch( gScaleEstimator.getScaleEstimationStatus() )
      {
         case ScaleEstimationStatus::IDLE:
            if( pose.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_FAILED )
            {
               failCnt++;
               if( failCnt > failPoseNum ) // function handle 
               {
                  gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::PREPARE_FOR_SCALE_ESTIMATION );
                  failCnt = 0;
               }
            }
            else
            {
               failCnt = 0;
            }
            break;
         case ScaleEstimationStatus::PREPARE_FOR_SCALE_ESTIMATION:
            break;
         case ScaleEstimationStatus::READY_TO_MOTION_FOR_SCALE_ESTIMATION:
            break;
         case ScaleEstimationStatus::START_POSE_COLLECTION_FOR_SCALE_ESTIMAION:
            break;
         case ScaleEstimationStatus::SCALE_ESTIMATOR:
            gScaleEstimator.setReinitTransformAndScalar();
            if( !gScaleEstimator.failScaleFlag )
            {
               gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::SUCCESS_SCALE_ESTIMAION );
            }
            else
            {
               gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::FAIL_SCALE_ESTIMAION );
            }
            break;
         case ScaleEstimationStatus::SUCCESS_SCALE_ESTIMAION:
            printf( "Should not call here!\n" );
            break;
         case ScaleEstimationStatus::FAIL_SCALE_ESTIMAION:
            break;
         default: // IDLE
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
      if( (motion_pattern == TARGETLESS_INIT_PATTERN && gMotionPatternQueue.empty()
            && (gScaleEstimator.getScaleEstimationStatus() == ScaleEstimationStatus::READY_TO_MOTION_FOR_SCALE_ESTIMATION
                 || gScaleEstimator.getScaleEstimationStatus() == ScaleEstimationStatus::IDLE))
          || gScaleEstimator.getScaleEstimationStatus() == ScaleEstimationStatus::START_POSE_COLLECTION_FOR_SCALE_ESTIMAION )
      {
         gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::START_POSE_COLLECTION_FOR_SCALE_ESTIMAION );
         setMotionPattern( MOTION_PATTERN::SCALE_ESTIMATION_PATTERN );
      }
      MOTION_PATTERN n_motion_pattern = motion_pattern;
   }
   printf( "motionProc Thread exit\n" );
}


void mainProc()
{
   THREAD_RUNNING = true;
   ParseVWSLAMConf( VWSLAM_Configuration, vslamPara, wefPara );
   signal( SIGINT, INThandler );

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

   gWEF = StartFS( &wefPara.poseVB, RELOCALIZATION == vslamPara.initMode, wefPara.vslamStateBadAsFail );
   InitializeVSLAM( vslamPara );

   SetInitMode( vslamPara, "8009Map" );

   std::thread wefThread( wefProc );
   std::thread cameraThread( cameraProc );
   std::thread vslamThread( vslamProc );
   std::thread motionThread( motionProc );
   wefThread.join();
   cameraThread.join();
   vslamThread.join();
   motionThread.join();

   GetPointCloud( "8009Map" );
   printf( "before ReleaseVSLAM\n" );
   ReleaseVSLAM();
   printf( "after ReleaseVSLAM\n" );
   ReleaseFS( gWEF );

   delete vslamCamera;
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

   if( vslamCamera )
   {
      uint8_t image[16];
      uint64_t t = 0;
      VirtualCameraFrame frame( image, t );
      vslamCamera->onPreviewFrame( &frame );
   }

   mvWEFPoseVelocityTime wheelodom;
   wheelodom.timestampUs = 0;
   gWEPoseQueueWheel2VSLAM.check_push( wheelodom );
   gWEPoseQueue.check_push( wheelodom );

   mvWEFPoseVelocityTime curWEPose{ 0 };
   gMotionWEPoseQueue.check_push( curWEPose );
}
