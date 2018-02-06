/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "VSLAM.h"
#include "VSLAM_internal.h"
#include "mvVSLAM.h"
#include "string.h"
#include "ScaleEstimation.h"
#include "Visualization.h"
#include <thread>
#include <atomic> 
#include <math.h>
#include <condition_variable>

#include "ReadImages.h"
#include "Motion.h"

#ifdef WIN32
#define M_PI       3.14159265358979323846f   // pi
#endif

//for map loading and saving profiling
int64_t getRealTime();

#ifdef ARM_BASED
#define PRINTSTARTENDTIME(des, keyframeNum, startTime, endTime)      printf("MapProfiling %s keyframe=%d, start=%lld end=%lld diff=%f\n", des, keyframeNum, startTime, endTime, (endTime-startTime)*1e-6)
#else
#define PRINTSTARTENDTIME(des, keyframeNum, startTime, endTime)
#endif

queue_mt<mvWEFPoseVelocityTime> gWEPoseQueueWheel2VSLAM;
queue_mt<bool> gScaleQueue( 1 );
bool gVSALMRunning = true;
static bool gFirstImg = true;
static std::mutex firstImgLock;
static std::condition_variable cond;

extern mvWEF *gWEF;
extern queue_mt<mvWEFPoseVelocityTime> gWEPoseQueue;

///////////////////////////////////////////////////////////////////////////////////////

vslamparameterInternal parameter;

extern ScaleEstimator gScaleEstimator;

queue_mt<mvWEFPoseStateTime> gVSLAMPoseRawQueue;

uint8_t g_ImageBuf[640 * 480];
std::mutex g_ImageBuf_mutex;
int64_t   g_timestamp;
bool   g_FirstBoot = true;
static bool g_FirstFrame = true;

void InitializeVSLAM( const VSLAMParameter & vslamPara )
{
   //Get the parameters for vslam
   if( ParseEngineParameters( "Configuration/vslam_internal.cfg", parameter ) != 0 )
   {
      return;
   }
   parameter.externalPara = vslamPara;

   parameter.pVSlamObj = mvVSLAM_Initialize( &vslamPara.vslamCameraConfig );
   if( parameter.pVSlamObj == NULL )
   {
      printf( "Failed to initialize MV VSLAM object!\n" );
      return;
   }
   printf( "In the InitializeVSLAM() and would set gVSLAMRunning as false!\n" );
   gVSALMRunning = false;

   return;
}

//not protected by mutex
bool SetTarget( const char* name )
{
   if( parameter.externalPara.initMode != VSLAMInitMode::TARGET_INIT )
   {
      return false;
   }

   if( parameter.externalPara.targetImagePath[0] == 0 )
   {
      printf( "Please provide path of target image\n" );
      return false;
   }

   TargetImage targetImage;
   if( ReadGrayScaleBitmap( parameter.externalPara.targetImagePath, targetImage ) == false )
   {
      return false;
   }
   //if( mvVSLAM_AddTarget( parameter.pVSlamObj, name, targetImage.image, targetImage.width, targetImage.height, targetImage.stride,
   //                       parameter.externalPara.targetWidth, parameter.externalPara.targetHeight, parameter.externalPara.targetPose, parameter.externalPara.targetHomography ) == -1 )
   if( mvVSLAM_AddTarget( parameter.pVSlamObj, name, targetImage.image, targetImage.width, targetImage.height, targetImage.stride,
                          parameter.externalPara.targetWidth, parameter.externalPara.targetHeight, parameter.externalPara.targetPose ) == -1 )
   {
      printf( "Failed to add target image!\n" );
      if( targetImage.allocatedImage )
      {
         delete[] targetImage.image;
      }
      return false;
   }

   if( targetImage.allocatedImage )
   {
      delete[] targetImage.image;
   }

   return true;
}
void SetInitMode( VSLAMParameter& currentPara, const char * mapName ) //Why the parameter is needed
{
   switch( currentPara.initMode )
   {
      case VSLAMInitMode::RELOCALIZATION:
         parameter.externalPara.initMode = VSLAMInitMode::RELOCALIZATION;
         if( currentPara.mapPath[0] != 0 )
         {
            std::string mapPath = Program_Root + parameter.externalPara.mapPath + mapName + "_data.txt";
            mvVSLAM_SetMapPath( parameter.pVSlamObj, mapPath.c_str(), true );
            gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::IDLE );
            gMotionPatternQueue.check_push( MOTION_PATTERN::TARGET_INIT_PATTERN );
         }
         else
         {
            printf( "Error!  Lack of map path!\n" );
         }
         break;
      case VSLAMInitMode::TARGETLESS_INIT:
         mvVSLAM_EnableScaleFreeTracking( parameter.pVSlamObj, true );
         gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::FAIL_SCALE_ESTIMAION );
         gMotionPatternQueue.check_push( MOTION_PATTERN::TARGETLESS_INIT_PATTERN );
         mvVSLAM_EnableWheelInitialization( parameter.pVSlamObj, true );
         break;
      case VSLAMInitMode::TARGET_INIT:
         SetTarget( "" );
         mvVSLAM_EnableScaleFreeTracking( parameter.pVSlamObj, false );
         gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::IDLE );
         gMotionPatternQueue.check_push( MOTION_PATTERN::TARGET_INIT_PATTERN );
         break;
      default:
         printf( "Please set init mode explicitly!\n" );
         break;
   }

   mvVSLAM_EnableLoopClosure( parameter.pVSlamObj, parameter.externalPara.loopClosureEnabled );

   mvVSLAM_EnableDescriptorSaveLoad( parameter.pVSlamObj, false );
   mvVSLAM_SetKeyframeSelectorParameters( parameter.pVSlamObj, parameter.minDistance, parameter.minDistanceAngle, parameter.minAngleAngle, parameter.minDelay, parameter.externalPara.maxKeyFrame );
   // To do: check whether the directory exists or not. If not, should create it! 
   std::string mapPathDefault = Program_Root + currentPara.mapPath;
   bool result = mvVSLAM_SetWorkingDirectory( parameter.pVSlamObj, mapPathDefault.c_str() );
   if( result == false )
   {
      printf( "Current working directory is %s but it doesn't exist!\n", mapPathDefault.c_str() );
   }
   mvVSLAM_EnableMapperSynchronousMode( parameter.pVSlamObj, false );
   return;
}

void StartMapUpdate()
{
   printf( "StartMapUpdate not implemented yet\n" );
   return;
}
void StopMapUpdate()
{
   printf( "StopMapUpdate not implemented yet\n" );
   return;
}

bool GetPointCloud( const char* mapName )
{
   printf( "Saving map at %s\n", (Program_Root + parameter.externalPara.mapPath).c_str() );
   bool result = true;
   if( mvVSLAM_GetMapSize( parameter.pVSlamObj ) > 0 )
   {
      result = mvVSLAM_SaveMap( parameter.pVSlamObj, (Program_Root + parameter.externalPara.mapPath).c_str(), mapName );
   }
   printf( "Saving map end, result =%d\n", int32_t( result ) );
   return result;
}

mvWEFPoseStateTime GetPose()
{
   mvWEFPoseStateTime pose;
   pose.timestampUs = 0;
   if( gVSALMRunning )
   {
      gVSLAMPoseRawQueue.wait_and_pop( pose );
   }

   return pose;
}

void ReleaseVSLAM()
{
   printf( "before deinitiaize the mvVSLAM engine\n" );
   mvVSLAM_Deinitialize( parameter.pVSlamObj );
   printf( "after deinitiaize the mvVSLAM engine\n" );
}

extern VSLAMParameter vslamPara;

void PrepareforTargetlessInitMode(bool saveMap)
{
   // save map
   // delete vslam  // mvVSLAM_Deinitialize
   // new vslam  // mvVSLAM_Initialize
   // set targetless mode // mvVSLAM_SaveMap

   // mvVSLAM_DeepReset is not preferred since target image based initialization is still running after DeepReset
   mvVSLAM_DeepReset( parameter.pVSlamObj, saveMap );

   VSLAMParameter tempPara = vslamPara;
   tempPara.initMode = TARGETLESS_INIT;
   SetInitMode( tempPara );
   gScaleEstimator.vslamPoseQueue.clear();
}

void SetReinitTransformationAndScalar( float32_t scale, float poseMatrix[3][4] )
{
   bool result = false;

   result = mvVSLAM_TransformMap( parameter.pVSlamObj, scale, poseMatrix );
   //result = mvVSLAM_TransformMap( parameter.pVSlamObj, scale,
   //                               translation[0], translation[1], translation[2],
   //                               rotation[0], rotation[1], rotation[2] );

   if( result == false )
   {
      printf( "Failed to transform the map!\n" );
      //while (1);
   }

}


bool AnalyzeInitTiming()
{
   return true;
}

//#define DUMP_UNDISTORTED_IMAGE
#define WHEEL_BASED_INITIALIZATION_V2
//#define printLog2File
//#define SIM_TARGETLESS_INITIALIZATION

#ifdef SIM_TARGETLESS_INITIALIZATION
extern int32_t failPoseNum;
#endif
void AddOneImage( int64_t timeStamp, const uint8_t * imageBuf )
{
   static mvWEFPoseVelocityTime curWEPose{ 0 };

#ifdef printLog2File
   static FILE * poseFile = fopen( "./poseFile.txt", "wb" );
   // only for debug when error occurs
   static FILE * logFile = fopen( "./addOneImgLog.txt", "wb" );
   fprintf( logFile, "add one image! Its timeStamp is %lld\n", timeStamp );
   fprintf( poseFile, "gScaleState: %d ", gScaleEstimator.getScaleEstimationStatus() );
#endif
   printf( "add one image!!!timeStamp = %lld, gScaleState = %d\n", timeStamp, gScaleEstimator.getScaleEstimationStatus() );
   if( gScaleEstimator.getScaleEstimationStatus() == ScaleEstimationStatus::PREPARE_FOR_SCALE_ESTIMATION )
   {
      printf( "In the case of preparation for scale estimation!!!\n" );
      int64_t time_SaveMapStart = getRealTime();     
      PrepareforTargetlessInitMode( gScaleEstimator.isSaveMapFlag );
      int64_t time_SaveMapEnd = getRealTime();
      int keyframeNum = mvVSLAM_GetMapSize( parameter.pVSlamObj );
      PRINTSTARTENDTIME("map saving", keyframeNum, time_SaveMapStart, time_SaveMapEnd);      
      gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::READY_TO_MOTION_FOR_SCALE_ESTIMATION );
      gScaleEstimator.isRestartScaleFlag = false;
      gVSLAMPoseRawQueue.clear();
      gVSLAMPoseQueue.clear();
      gMotionVSLAMPoseQueue.clear();
   }
   else if( gScaleEstimator.getScaleEstimationStatus() == ScaleEstimationStatus::SUCCESS_SCALE_ESTIMAION )
   {
      if( !gScaleEstimator.isRestartScaleFlag )
      {
         printf( "In the case of SUCCESS_SCALE_ESTIMAION!\n" );
         int64_t time_ReinitMapStart = getRealTime();
         SetReinitTransformationAndScalar( gScaleEstimator.scale, gScaleEstimator.mPoseSP );
         int64_t time_ReinitMapEnd = getRealTime();
         PRINTSTARTENDTIME("map reinit", mvVSLAM_GetMapSize( parameter.pVSlamObj ), time_ReinitMapStart, time_ReinitMapEnd);

         printf( "Before load and merge the map, isSaveMapFlag = %d!\n", gScaleEstimator.isSaveMapFlag );
         if( gScaleEstimator.isSaveMapFlag )
         {
            int64_t time_MergeMapStart = getRealTime();
            bool result = mvVSLAM_LoadDefaultMapAndMerge( parameter.pVSlamObj );
            int64_t time_MergeMapEnd = getRealTime();
            PRINTSTARTENDTIME("map merging", mvVSLAM_GetMapSize( parameter.pVSlamObj ), time_MergeMapStart, time_MergeMapEnd);
            if( result == true )
               printf( "Successful load default map and merge with the new one.\n" );
            else
               printf( "Fail load default map and merge with the new one.\n" );
         }

         gVSLAMPoseRawQueue.clear();
         gVSLAMPoseQueue.clear();
         gMotionVSLAMPoseQueue.clear();

         gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::IDLE );
         gScaleEstimator.isRestartScaleFlag = false;

         //gScaleEstimator.setScaleEstimationStatus(ScaleEstimationStatus::START_POSE_COLLECTION_FOR_SCALE_ESTIMAION); 
         //gMotionPatternQueue.check_push(MOTION_PATTERN::TARGETLESS_INIT_PATTERN);
         //gScaleEstimator.isRestartScaleFlag = true;
#ifdef printLog2File			  
         fprintf( poseFile, "%s %lld %f %f %f %f %f %f %f\n", "SetFirstCallValue", timeStamp,
                  gScaleEstimator.scale, gScaleEstimator.mPoseVWt[0], gScaleEstimator.mPoseVWt[1], gScaleEstimator.mPoseVWt[2],
                  gScaleEstimator.mPoseVWr[0], gScaleEstimator.mPoseVWr[1], gScaleEstimator.mPoseVWr[2] );
#endif

      }
      else
      {
         gScaleEstimator.mPoseVWt[0] = 0;
         gScaleEstimator.mPoseVWt[1] = 0;
         gScaleEstimator.mPoseVWt[2] = 0;
         gScaleEstimator.mPoseVWr[0] = 2 * (float32_t)M_PI;
         gScaleEstimator.mPoseVWr[1] = 0.0;
         gScaleEstimator.mPoseVWr[2] = 0.0;

         SetReinitTransformationAndScalar( gScaleEstimator.scale, gScaleEstimator.mPoseSP );

         bool result = mvVSLAM_LoadDefaultMapAndMerge( parameter.pVSlamObj );
         if( result == true )
            printf( "Successful load default map and merge with the new one.\n" );
         else
            printf( "Fail load default map and merge with the new one.\n" );

         gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::IDLE );
         gScaleEstimator.isRestartScaleFlag = false;
#ifdef printLog2File
         fprintf( poseFile, "%s %f %f %f %f %f %f %f\n", "SetSecondCallValue",
                  gScaleEstimator.scale, gScaleEstimator.mPoseVWt[0], gScaleEstimator.mPoseVWt[1], gScaleEstimator.mPoseVWt[2],
                  gScaleEstimator.mPoseVWr[0], gScaleEstimator.mPoseVWr[1], gScaleEstimator.mPoseVWr[2] );
#endif
         gVSLAMPoseRawQueue.clear();
         gVSLAMPoseQueue.clear();
         gMotionVSLAMPoseQueue.clear();
      }
   }
   else if( gScaleEstimator.getScaleEstimationStatus() == ScaleEstimationStatus::FAIL_SCALE_ESTIMAION )
   {
      PrepareforTargetlessInitMode( false );
      gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::READY_TO_MOTION_FOR_SCALE_ESTIMATION );
      gScaleEstimator.isRestartScaleFlag = false;
      gVSLAMPoseRawQueue.clear();
      gVSLAMPoseQueue.clear();
      gMotionVSLAMPoseQueue.clear();
   }

   timeStamp /= 1000; // ns to us

   if( parameter.externalPara.wheelEnabled )
   {
      mvWEFPoseVelocityTime lastWEPose;
      do
      {
         lastWEPose = curWEPose;
         gWEPoseQueueWheel2VSLAM.wait_and_pop( curWEPose );
         gMotionWEPoseQueue.check_push( curWEPose );
      } while( curWEPose.timestampUs < timeStamp && !gWEPoseQueueWheel2VSLAM.empty() );
#ifdef WHEEL_BASED_INITIALIZATION_V2
      bool isSynchronized = false;
      const int64_t thDeltaUs = 20000; // TODO: 20ms, could be changed according to wheel odom rate
      int64_t curDelta = curWEPose.timestampUs - timeStamp;
      int64_t lastDelta = timeStamp - lastWEPose.timestampUs;

      if( curDelta < lastDelta )
      {
         if( curDelta < thDeltaUs )
         {
            float linearVelocity[3] = { curWEPose.velocityLinear, 0, 0 };
            float angularVelocity[3] = { 0, 0, curWEPose.velocityAngular };
            mvWEF_BodyToCameraPoint( gWEF, linearVelocity );
            mvWEF_BodyToCameraPoint( gWEF, angularVelocity );
            mvVSLAM_AddImageWithWheel( parameter.pVSlamObj, timeStamp, imageBuf,
                                       mvWEF_BodyToCameraPose( gWEF, curWEPose.pose ).matrix,
                                       linearVelocity, angularVelocity );
            isSynchronized = true;
         }
      }
      else
      {
         if( lastDelta < thDeltaUs )
         {
            float linearVelocity[3] = { lastWEPose.velocityLinear, 0, 0 };
            float angularVelocity[3] = { 0, 0, lastWEPose.velocityAngular };
            mvWEF_BodyToCameraPoint( gWEF, linearVelocity );
            mvWEF_BodyToCameraPoint( gWEF, angularVelocity );

            mvVSLAM_AddImageWithWheel( parameter.pVSlamObj, timeStamp, imageBuf,
                                       mvWEF_BodyToCameraPose( gWEF, lastWEPose.pose ).matrix,
                                       linearVelocity, angularVelocity );

            isSynchronized = true;
         }
      }
      if( !isSynchronized )
      {
         printf( "Drop image with timestamp %lld (us) since no corresponding wheel odom!\n", timeStamp );
         return;
      }
#else
      mvVSLAM_AddImage( parameter.pVSlamObj, timeStamp, imageBuf );
#endif
   }
   else
   {
      mvVSLAM_AddImage( parameter.pVSlamObj, timeStamp, imageBuf );
   }
   if( !g_FirstFrame && curWEPose.timestampUs != 0 )
   {
      if( fabs( timeStamp - curWEPose.timestampUs ) > 1e6 )
      {
         printf( "Camera/wheel clock error!!!!\n" );
         printf( "camera_stamp %lld, wheel_stamp %lld\n", timeStamp, curWEPose.timestampUs );
         mvWEFPoseStateTime pose;
         pose.timestampUs = -1;
         gVSLAMPoseRawQueue.check_push( pose );
         return;
      }
   }
   if( g_FirstFrame )
      g_FirstFrame = false;

   mvWEFPoseStateTime pose;
   pose.poseWithState = mvVSLAM_GetPose( parameter.pVSlamObj );
   pose.timestampUs = timeStamp;

#ifdef SIM_TARGETLESS_INITIALIZATION
   {
      static size_t cntFrame = 0;
      const size_t ndxFrame2ClaimFail = 100;
      cntFrame++;
      if( cntFrame > ndxFrame2ClaimFail && cntFrame < ndxFrame2ClaimFail + 2 * failPoseNum )
      {
         pose.poseWithState.poseQuality = MV_VSLAM_TRACKING_STATE_FAILED;
         pose.poseWithState.pose.matrix[0][3] = 0.F;
         pose.poseWithState.pose.matrix[1][3] = 0.F;
         pose.poseWithState.pose.matrix[2][3] = 0.F;
      }
   }
#endif

   gScaleEstimator.poseQuality = pose.poseWithState.poseQuality;
   printf( "Quality: %d \n", pose.poseWithState.poseQuality );
   if( pose.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_FAILED )
      printf( "tracking failed\n" );

   if( g_FirstBoot == true && (pose.poseWithState.poseQuality >= MV_VSLAM_TRACKING_STATE_GREAT) )
   {
      g_FirstBoot = false;
      //Print the initialization end flag into kernel log
      system( "echo vSLAM Initialization Completed > /dev/kmsg" );
   }

#ifdef printLog2File
   fprintf( poseFile, "Image timeStamp is %lld, Quality %d, poseX %f, poseY %f poseZ %f ", timeStamp, pose.poseWithState.poseQuality,
            pose.poseWithState.pose.matrix[0][3], pose.poseWithState.pose.matrix[1][3], pose.poseWithState.pose.matrix[2][3] );

   fprintf( poseFile, "Rotation %f %f %f %f %f %f %f %f %f \n ",
            pose.poseWithState.pose.matrix[0][0], pose.poseWithState.pose.matrix[1][0], pose.poseWithState.pose.matrix[2][0],
            pose.poseWithState.pose.matrix[0][1], pose.poseWithState.pose.matrix[1][1], pose.poseWithState.pose.matrix[2][1],
            pose.poseWithState.pose.matrix[0][2], pose.poseWithState.pose.matrix[1][2], pose.poseWithState.pose.matrix[2][2]
            );
#endif
   visualiser->ShowPoints( pose, imageBuf, parameter.externalPara.vslamCameraConfig.pixelWidth, parameter.externalPara.vslamCameraConfig.pixelHeight );
   visualiser->PublishCameraPose( pose );
   gVSLAMPoseRawQueue.check_push( pose );
}

//#define DebugCameraCallBack
void cameraProc()
{
   //wait for first image come
   std::unique_lock<std::mutex> lk( firstImgLock );
   cond.wait( lk, []
   {
      return !gFirstImg;
   } );
   lk.unlock();

   int bufferLength1 = parameter.externalPara.vslamCameraConfig.pixelHeight * parameter.externalPara.vslamCameraConfig.pixelWidth;
   int64_t current_timestamp = 0;
   unsigned char* current_ImageBuf = (unsigned char*)malloc( sizeof( unsigned char ) * bufferLength1 );
   unsigned int imageIndex = 0;
#ifdef DebugCameraCallBack
   printf( "In the front of while(true) to put One image to the VSLAM!\n" );
#endif
   while( true )
   {
      bool sleepOrNot = false; //In case the server is too fast in simulation.
      g_ImageBuf_mutex.lock();
      if( current_timestamp == g_timestamp )
      {
         sleepOrNot = true;
      }
      else
      {
         current_timestamp = g_timestamp;
         memcpy( current_ImageBuf, g_ImageBuf, sizeof( unsigned char ) * bufferLength1 );
      }
#ifdef DebugCameraCallBack
      printf( "PLAYBACK value: %d Description: False for online running, true for simulation with break points.\n", PLAYBACK );
#endif
      // Note: g_ImageBuf_mutex.unlock() has to be followed by VSLAM_MASTER_SLEEP_INTERNAL
      //       otherwise, the image preparation will be blocked by this lock->unlock loop in a while true loop
      if( !PLAYBACK ) //False for online running, true for simulation with break points.
      {

#ifdef DebugCameraCallBack
         printf( "This branch is for online running!\n" );
#endif
         g_ImageBuf_mutex.unlock();
         if( sleepOrNot )
         {
            VSLAM_MASTER_SLEEP_INTERNAL( SleepTimeInMillisecond );
         }
         else
         {
            //std::cout<<"===Begin algo Thread Executing: process current_frameid  = " <<current_frameid << "   current_stonesImage[0]=" << current_stonesImage[0] <<std::endl;
            AddOneImage( current_timestamp, current_ImageBuf );
            printf( "vslam image index = %u\n", imageIndex );
            imageIndex++;
            //std::cout<<"===End algo Thread Executing:   process  current_frameid = "<<current_frameid<<"   current_stonesImage[0]=" << current_stonesImage[0] << std::endl;
         }
      }
      else
      {

#ifdef DebugCameraCallBack
         printf( "This branch is for simulation offline!\n" );
#endif
         if( sleepOrNot )
         {
            printf( "Sleep in image processing\n" );
         }
         else
         {
            AddOneImage( current_timestamp, current_ImageBuf );
            imageIndex++;
            printf( "vslam image index = %u\n", imageIndex );
         }
         g_ImageBuf_mutex.unlock();
         VSLAM_MASTER_SLEEP_INTERNAL( SleepTimeInMillisecond );
      }
      if( gVSALMRunning == false )
      {
         printf( "gVSLAMRunning is false and camera proc exit\n" );
         break;
      }
   } // while(true)

   printf( "Add one image exits\n" );
   free( current_ImageBuf );
   mvWEFPoseStateTime pose;
   pose.timestampUs = 0;
   gVSLAMPoseRawQueue.check_push( pose );
   gMotionVSLAMPoseQueue.check_push( pose );
}

//#define DebugCameraCallBack
void VSLAMCameraCallback( const int64_t timeStamp, const uint8_t * imageBuf )
{
   printf( "new image come\n" );
   int bufferLength = parameter.externalPara.vslamCameraConfig.pixelHeight * parameter.externalPara.vslamCameraConfig.pixelWidth;
   g_ImageBuf_mutex.lock();
   g_timestamp = timeStamp;
   memcpy( g_ImageBuf, imageBuf, bufferLength );
   g_ImageBuf_mutex.unlock();

#ifdef DebugCameraCallBack
   printf( "noThreadruning should be true and its value is %d\n", noThreadruning );
   printf( "gVSALMRunning should be true and its value is %d\n", gVSALMRunning );
#endif

   if( gVSALMRunning && gFirstImg )
   {
      gFirstImg = false;
      cond.notify_one();
   }

   if( !gVSALMRunning )
   {
      printf( "Add one pose to exit\n" );
      mvWEFPoseStateTime pose;
      pose.timestampUs = 0;
      gVSLAMPoseRawQueue.check_push( pose );
      gMotionVSLAMPoseQueue.check_push( pose );
   }
}

void StartVSLAM()
{
   printf( "StartVSLAM and set gVSLAMRunning as true!\n" );
   gVSALMRunning = true;
}

void StopVSLAM()
{
   printf( "StopVSLAM and set gVSLAMRunning as false!\n" );
   gVSALMRunning = false;
}
