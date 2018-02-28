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

extern queue_mt<mvWEFPoseStateTime> gMotionVSLAMPoseQueue;
extern MapFocuser* mapFocuser;
extern MapFocuser* mapFocuserSecondary;

//for map loading and saving profiling
int64_t getRealTimeForLog()
{
#ifdef WIN32
   timespec ltime;
   timespec_get( &ltime, TIME_UTC );
   return (long long)ltime.tv_sec * (long long)1000000000l + ltime.tv_nsec;
#else
   struct timespec t;
   clock_gettime( CLOCK_REALTIME, &t );
   uint64_t timeNanoSecRealTime = t.tv_sec * 1000000000ULL + t.tv_nsec;
   return (int64_t)timeNanoSecRealTime;
#endif //WIN32

}

#define PRINTSTARTENDTIME(des, keyframeNum, startTime, endTime)      printf("MapProfiling %s keyframe=%d, start=%lld end=%lld diff=%f\n", des, keyframeNum, startTime, endTime, (endTime-startTime)*1e-6)

//#define DebugCameraCallBack
#ifdef DebugCameraCallBack
#define CAMERA_PRINT(s)        printf(s)
#define CAMERA_PRINT2(s,x)     printf(s, x)
#else
#define CAMERA_PRINT(s)        (void)(s)
#define CAMERA_PRINT2(s,x)     (void)(s, x)
#endif

queue_mt<mvWEFPoseVelocityTime> gWEPoseQueueWheel2VSLAM;
queue_mt<bool> gScaleQueue( 1 );
bool gVSALMRunning = false;
static bool gFirstImg = true;
static std::mutex firstImgLock;
static std::condition_variable cond;

extern mvWEF *gWEF;
extern queue_mt<mvWEFPoseVelocityTime> gWEPoseQueue;


///////////////////////////////////////////////////////////////////////////////////////

vslamparameterInternal parameter;

extern ScaleEstimator gScaleEstimator;

queue_mt<VSLAMPoseWithFeedback> gVSLAMPoseRawQueue;
queue_mt<VSLAMPoseWithFeedback> gVSLAMPoseRawSecondaryQueue;

uint8_t * g_ImageBuf = NULL;
std::mutex g_ImageBuf_mutex;
int64_t   g_timestamp;
bool   g_FirstBoot = true;
static bool g_FirstFrame = true;
bool g_EnableSetExternalConstraint = true;

std::mutex gMapBackupMutex;

void InitPose6DRT( mvPose6DRT & pose )
{
   pose.matrix[0][0] = 1.f;
   pose.matrix[0][1] = 0.f;
   pose.matrix[0][2] = 0.f;
   pose.matrix[0][3] = 0.f;

   pose.matrix[1][0] = 0.f;
   pose.matrix[1][1] = 1.f;
   pose.matrix[1][2] = 0.f;
   pose.matrix[1][3] = 0.f;

   pose.matrix[2][0] = 0.f;
   pose.matrix[2][1] = 0.f;
   pose.matrix[2][2] = 1.f;
   pose.matrix[2][3] = 0.f;
}

void InitializeVSLAM( const VSLAMParameter & vslamPara )
{
   //Get the parameters for vslam
   if( ParseEngineParameters( "Configuration/vslam_internal.cfg", parameter ) != 0 )
   {
      return;
   }
   parameter.externalPara = vslamPara;

   g_EnableSetExternalConstraint = true; // will enable external constraint check after SpatialInWorld is ready
   parameter.pVSlamObj = mvVSLAM_Initialize( &vslamPara.vslamCameraConfig );
   if( parameter.pVSlamObj == NULL )
   {
      printf( "Failed to initialize MV VSLAM object!\n" );
      return;
   }

   if( vslamPara.alwaysOnRelocation )
   {
      parameter.pVSlamSecondaryObj = mvVSLAM_Initialize( &vslamPara.vslamCameraConfig );

      //The map in the slam is built only after images are added.
      //If there is no map, we can't load backed up map from another vslam engine.
      int64_t timeStamp = 0;
      uint8_t * image = new uint8_t[vslamPara.vslamCameraConfig.pixelHeight * vslamPara.vslamCameraConfig.pixelWidth];
      memset( image, 0, vslamPara.vslamCameraConfig.pixelHeight * vslamPara.vslamCameraConfig.pixelWidth );
      mvVSLAM_AddImage( parameter.pVSlamSecondaryObj, timeStamp, image );
      delete[] image;
   }

   printf( "In the InitializeVSLAM() and would set gVSLAMRunning as false!\n" );
   g_ImageBuf_mutex.lock();
   if( g_ImageBuf != NULL )
   {
      free( g_ImageBuf );
   }
   int bufferLength1 = parameter.externalPara.vslamCameraConfig.pixelHeight * parameter.externalPara.vslamCameraConfig.pixelWidth;
   g_ImageBuf = (uint8_t *)malloc( bufferLength1 );
   g_ImageBuf_mutex.unlock();
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
            //gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::IDLE );
            //gMotionPatternQueue.check_push( MOTION_PATTERN::TARGET_INIT_PATTERN );
         }
         else
         {
            printf( "Error!  Lack of map path!\n" );
         }
         break;
      case VSLAMInitMode::TARGETLESS_INIT:
         mvVSLAM_EnableScaleFreeTracking( parameter.pVSlamObj, true );
         //gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::FAIL_SCALE_ESTIMAION );
         //gMotionPatternQueue.check_push( MOTION_PATTERN::TARGETLESS_INIT_PATTERN );
         mvVSLAM_EnableWheelInitialization( parameter.pVSlamObj, true );
         break;
      case VSLAMInitMode::TARGET_INIT:
         SetTarget( "" );
         mvVSLAM_EnableScaleFreeTracking( parameter.pVSlamObj, false );
         gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::TARGET_INITIALIZATION );
         break;
      default:
         printf( "Please set init mode explicitly!\n" );
         break;
   }

   mvVSLAM_EnableLoopClosure( parameter.pVSlamObj, parameter.externalPara.loopClosureEnabled );

   mvVSLAM_SetKeyframeSelectorParameters( parameter.pVSlamObj, parameter.minDistance, parameter.minAngle, parameter.minDelay, parameter.externalPara.maxKeyFrame, parameter.useDynamicThreshold, parameter.cutoffDepth, parameter.convexFactor, parameter.deadZone );
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

void SetInitModeSecondary()
{
   mvVSLAM_EnableLoopClosure( parameter.pVSlamSecondaryObj, parameter.externalPara.loopClosureEnabled );
   mvVSLAM_SetKeyframeSelectorParameters( parameter.pVSlamSecondaryObj, parameter.minDistance, parameter.minAngle, parameter.minDelay, parameter.externalPara.maxKeyFrame, parameter.useDynamicThreshold, parameter.cutoffDepth, parameter.convexFactor, parameter.deadZone );
   // To do: check whether the directory exists or not. If not, should create it! 
   std::string mapPathDefault = Program_Root + parameter.externalPara.mapPath;
   bool result = mvVSLAM_SetWorkingDirectory( parameter.pVSlamSecondaryObj, mapPathDefault.c_str() );
   if( result == false )
   {
      printf( "%s: Current working directory is %s but it doesn't exist!\n", __FUNCTION__, mapPathDefault.c_str() );
   }
   mvVSLAM_EnableMapperSynchronousMode( parameter.pVSlamSecondaryObj, false );
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
   VSLAMScheduler* instance = VSLAMScheduler::getInstance();
   VSLAMScheduler::State state = instance->getState();
   int keyframeNum = 0;
   switch( state )
   {
      case VSLAMScheduler::kSTATE_PRIMARY:
      case VSLAMScheduler::kSTATE_SECONDARY_IMPORT:
      {
         //call loadDefaultMapAndMerge
         //for 1 vslam case, when the vslam is still estimating map scale.
         //This is done in the engine before. Now we move this operation out.
         if( gScaleEstimator.getScaleEstimationStatus() != ScaleEstimationStatus::IDLE )
         {
            gMapBackupMutex.lock();
            mvVSLAM_LoadDefaultMapAndMerge( parameter.pVSlamObj, gScaleEstimator.pMapBackup );
            gMapBackupMutex.unlock();
         }
         keyframeNum = mvVSLAM_GetMapSize( parameter.pVSlamObj );
         if( keyframeNum > 0 )
         {
            result = mvVSLAM_SaveMap( parameter.pVSlamObj, (Program_Root + parameter.externalPara.mapPath).c_str(), mapName );
         }
         break;
      }
      case VSLAMScheduler::kSTATE_CONCURRENT:
      case VSLAMScheduler::kSTATE_SECONDARY:
      case VSLAMScheduler::kSTATE_PRIMARY_IMPORT:
      {
         if( parameter.externalPara.alwaysOnRelocation )
         {
            keyframeNum = mvVSLAM_GetMapSize( parameter.pVSlamSecondaryObj );
            if( keyframeNum > 0 )
            {
               result = mvVSLAM_SaveMap( parameter.pVSlamSecondaryObj, (Program_Root + parameter.externalPara.mapPath).c_str(), mapName );
            }
         }
         break;
      }
      default:
         break;
   }
   printf( "Saving map end, result = %d, state = %d, keyframeNum = %d\n", (int32_t)result, (int)state, keyframeNum );
   fflush( stdout );
   return result;
}

bool RemoveKeyframes( mvVSLAM* currentVSLAM )
{
   bool result = true;
   MV_ActiveKeyframe* currentKFs = NULL;
   int activeKFs = 0;
   VSLAMScheduler* instance = VSLAMScheduler::getInstance();
   VSLAMScheduler::State state = instance->getState();
   int mapSize = 0;

   mapSize = mvVSLAM_GetMapSize(currentVSLAM);
   if (gScaleEstimator.getScaleEstimationStatus() >= IDLE)
   {
	   if (mapFocuser->TransformationReady() && mapSize > 0)
	   {
		   MV_ActiveKeyframe* currentKFs = new MV_ActiveKeyframe[mapSize];
		   int activeKFs = mvVSLAM_GetKeyframes(parameter.pVSlamObj, currentKFs, mapSize);
		   mapFocuser->UpdateKeyframeState(currentKFs, activeKFs, true);
		   std::set<int> keyframesToLoad = mapFocuser->GetKFsIdToLoad();
		   std::set<int> keyframesToDeactivate = mapFocuser->GetKFsIdToDeactivate();
		   std::set<int> keyframesToRemove = mapFocuser->GetKFsIdToRemove();
		   delete[]currentKFs;

		   std::vector<int> keyframeIds;
		   keyframeIds.clear();
		   for (std::set<int>::iterator i = keyframesToRemove.begin(); i != keyframesToRemove.end(); ++i)
		   {
			   keyframeIds.push_back(*i);
		   }
		   mvVSLAM_RemoveKeyframe(currentVSLAM, keyframeIds.data(), keyframeIds.size());
	   }
   }
   else
   {
	   if (mapSize > 0)
	   {
		   MV_ActiveKeyframe* currentKFs = new MV_ActiveKeyframe[mapSize];
		   int activeKFs = mvVSLAM_GetKeyframes(parameter.pVSlamObj, currentKFs, mapSize);
		   mapFocuser->UpdateKeyframeState(currentKFs, activeKFs, false);
		   std::set<int> keyframesToLoad = mapFocuser->GetKFsIdToLoad();
		   std::set<int> keyframesToDeactivate = mapFocuser->GetKFsIdToDeactivate();
		   std::set<int> keyframesToRemove = mapFocuser->GetKFsIdToRemove();
		   delete[]currentKFs;

		   std::vector<int> keyframeIds;
		   keyframeIds.clear();
		   for (std::set<int>::iterator i = keyframesToRemove.begin(); i != keyframesToRemove.end(); ++i)
		   {
			   keyframeIds.push_back(*i);
		   }
		   mvVSLAM_RemoveKeyframe(currentVSLAM, keyframeIds.data(), keyframeIds.size());
	   }
   }

   return result;
}


bool RemoveKeyframesSecondary(mvVSLAM* currentVSLAM)
{
	bool result = true;
	MV_ActiveKeyframe* currentKFs = NULL;
	int activeKFs = 0;
	VSLAMScheduler* instance = VSLAMScheduler::getInstance();
	VSLAMScheduler::State state = instance->getState();
	int mapSize = 0;

	mapSize = mvVSLAM_GetMapSize(currentVSLAM);
	if (mapFocuserSecondary->TransformationReady() && mapSize > 0)
	{
		MV_ActiveKeyframe* currentKFs = new MV_ActiveKeyframe[mapSize];
		int activeKFs = mvVSLAM_GetKeyframes(currentVSLAM, currentKFs, mapSize);
		mapFocuserSecondary->UpdateKeyframeState(currentKFs, activeKFs, true);
		std::set<int> keyframesToLoad = mapFocuserSecondary->GetKFsIdToLoad();
		std::set<int> keyframesToDeactivate = mapFocuserSecondary->GetKFsIdToDeactivate();
		std::set<int> keyframesToRemove = mapFocuserSecondary->GetKFsIdToRemove();
		delete[]currentKFs;

		std::vector<int> keyframeIds;
		keyframeIds.clear();
		for (std::set<int>::iterator i = keyframesToRemove.begin(); i != keyframesToRemove.end(); ++i)
		{
			keyframeIds.push_back(*i);
		}
		mvVSLAM_RemoveKeyframe(currentVSLAM, keyframeIds.data(), keyframeIds.size());
	}		

	return result;
}


void ReleaseVSLAM()
{
   printf( "before deinitiaize the mvVSLAM engine\n" );
   mvVSLAM_Deinitialize( parameter.pVSlamObj );

   if( parameter.externalPara.alwaysOnRelocation )
   {
      mvVSLAM_Deinitialize( parameter.pVSlamSecondaryObj );
   }

   g_ImageBuf_mutex.lock();
   if( g_ImageBuf != NULL )
   {
      free( g_ImageBuf );
   }
   g_ImageBuf = NULL;
   g_ImageBuf_mutex.unlock();

   printf( "after deinitiaize the mvVSLAM engine\n" );
}

extern VSLAMParameter vslamPara;

void PrepareforTargetlessInitMode( bool saveMap )
{
   // save map
   // delete vslam  // mvVSLAM_Deinitialize
   // new vslam  // mvVSLAM_Initialize
   // set targetless mode // mvVSLAM_SaveMap

   // mvVSLAM_DeepReset is not preferred since target image based initialization is still running after DeepReset
   g_EnableSetExternalConstraint = false; // not set constraint in case of targetless initialization
   //Set some parameters kept in backed up map for further mapping, but the backed up map is not loaded here
   mvVSLAM_DeepReset( parameter.pVSlamObj, saveMap, gScaleEstimator.pMapBackup );

   VSLAMParameter tempPara = vslamPara;
   tempPara.initMode = TARGETLESS_INIT;
   SetInitMode( tempPara );
   //gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::FAIL_SCALE_ESTIMAION );

   gScaleEstimator.vslamPoseQ.clear();
   gScaleEstimator.wePoseQ.clear();
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

void resetParamBuffer( int32_t &failFrameCnt, int32_t &successFrameCnt,
                       mvWEFPoseVelocityTime &previousWEPose,
                       bool &hasPoseWithScale, float &increamentalTrajectoryDist,
                       int32_t &increamentalFrameCnt )
{
   failFrameCnt = 0;
   successFrameCnt = 0;
   previousWEPose.timestampUs = -1;
   hasPoseWithScale = false;
   increamentalTrajectoryDist = 0.8f;
   increamentalFrameCnt = 10;

   gScaleEstimator.vslamPoseQ.clear();
   gScaleEstimator.wePoseQ.clear();
   gScaleEstimator.scaleVerificationV.failTimes = 0;
   gScaleEstimator.scaleVerificationV.passTimes = 0;
   gScaleEstimator.scaleVerificationV.isVerifiedSmall = false;
}

void scaleEstimatorStatusTransform( const mvWEFPoseStateTime pose,
                                    const mvWEFPoseVelocityTime selectedWEPose,
                                    VSLAMScheduler::Feedback &feedback,
                                    const VSLAMScheduler::State state )
{
   static mvWEFPoseVelocityTime previousWEPose;   // set its inital value in kSTATE_SECONDARY or START_POSE_COLLECTION_FOR_SCALE_ESTIMAION case
   static float32_t trajDistSquareFromWEPose;     // set its intial value based on previousWEPose constraint
   static int32_t successFrameCnt = 0;
   static int32_t failFrameCnt = 0;
   static bool hasPoseWithScale = false;
   static float increamentalTrajectoryDist = 0.f;
   static int32_t increamentalFrameCnt = 0;

   // primary is deactived in kSTATE_SECONDARY state when ScaleEstimationStatus::IDLE is detected
   if( state == VSLAMScheduler::kSTATE_SECONDARY )
   {
      resetParamBuffer( failFrameCnt, successFrameCnt, previousWEPose, hasPoseWithScale, increamentalTrajectoryDist, increamentalFrameCnt );
      gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::IDLE );
      return;
   }

   switch( gScaleEstimator.getScaleEstimationStatus() )
   {
      case ScaleEstimationStatus::IDLE:
      {
         // Change the save map flag from false to true in case 
         //   target-based initialization with the high quality VSLAM pose 
         bool isVSLAMPoseHighQuality = IsPoseHighQuality( pose.poseWithState );
         if( isVSLAMPoseHighQuality && gScaleEstimator.isSaveMapFlag == false )
         {
            gScaleEstimator.isSaveMapFlag = true;
         }

         // From secondary to primary
         if( VSLAMScheduler::kSTATE_PRIMARY_IMPORT == state )
         {
            assert( nullptr != gScaleEstimator.pMapBackup );
            //Set some parameters kept in backed up map for further mapping, but the backed up map is not loaded here
            gMapBackupMutex.lock();
            mvVSLAM_DeepReset( parameter.pVSlamObj, false, gScaleEstimator.pMapBackup );
            gMapBackupMutex.unlock();
            VSLAMParameter tempPara = vslamPara;
            tempPara.initMode = TARGETLESS_INIT;
            SetInitMode( tempPara );
            gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::START_POSE_COLLECTION_FOR_SCALE_ESTIMAION );
            feedback = VSLAMScheduler::kFB_MAPIMPORTED;
         }
         else
         {
            static int32_t failCnt = 0;
            if( pose.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_FAILED )
            {
               failCnt++;
               if( failCnt > gScaleEstimator.failPoseNum2startTargetless ) // function handle 
               {
                  failCnt = 0;
                  // make sure # keyframe can be limited around vslamPara.maxKeyFrame
                  if( mvVSLAM_GetMapSize( parameter.pVSlamObj ) < (int32_t)vslamPara.maxKeyFrame - 5 )
                  {
                     gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::PREPARE_FOR_SCALE_ESTIMATION );
                  }
               }
            }
            else
            {
               failCnt = 0;
            }
         }

         break;
      }
      case ScaleEstimationStatus::PREPARE_FOR_SCALE_ESTIMATION:
      {
         printf( "In the case of preparation for scale estimation!!!\n" );
         if( gScaleEstimator.isSaveMapFlag )
         {
            gMapBackupMutex.lock();
            gScaleEstimator.pMapBackup = mvVSLAM_ExportMapBackup( parameter.pVSlamObj );
            gMapBackupMutex.unlock();
         }
         //Keep the number of keyframes before clearing the map.
         int32_t keyframeNum = mvVSLAM_GetMapSize( parameter.pVSlamObj );
         int64_t time_SaveMapStart = getRealTimeForLog();
         PrepareforTargetlessInitMode( gScaleEstimator.isSaveMapFlag );
         int64_t time_SaveMapEnd = getRealTimeForLog();
         feedback = vslamPara.alwaysOnRelocation && keyframeNum > 2 ? VSLAMScheduler::kFB_MAPEXPORTED : VSLAMScheduler::kFB_NONE;
         PRINTSTARTENDTIME( "map saving", keyframeNum, time_SaveMapStart, time_SaveMapEnd );

         resetParamBuffer( failFrameCnt, successFrameCnt, previousWEPose, hasPoseWithScale, increamentalTrajectoryDist, increamentalFrameCnt );
         gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::START_POSE_COLLECTION_FOR_SCALE_ESTIMAION );
         // break; get into ScaleEstimationStatus::START_POSE_COLLECTION_FOR_SCALE_ESTIMAION: directly
      }
      case ScaleEstimationStatus::START_POSE_COLLECTION_FOR_SCALE_ESTIMAION:
      {
         // Compute the distance
         //pose, const mvWEFPoseVelocityTime selectedWEPose
         bool isVSLAMPoseHighQuality = IsPoseHighQuality( pose.poseWithState );
         if( !isVSLAMPoseHighQuality )
         {
            if( gScaleEstimator.countFailNnmAfterSuccessTrack && successFrameCnt != 0 )
               failFrameCnt++;
            else if( !gScaleEstimator.countFailNnmAfterSuccessTrack )
               failFrameCnt++;
         }
         else
         {
            failFrameCnt = 0;
         }

         // Fail to collect data for continous failure frames
         //if( failFrameCnt > gScaleEstimator.failPoseNum2RestartTargetless && successFrameCnt != 0 )
         if( failFrameCnt > gScaleEstimator.failPoseNum2RestartTargetless )
         {
            gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::FAIL_SCALE_ESTIMAION );
            printf( "Stop the current motion pattern for scale estimation for continuing failing to get a good VSLAM pose!\n" );
         }
         else if( isVSLAMPoseHighQuality )
         {
            // Collect data
            gScaleEstimator.vslamPoseQ.push_back( pose );
            gScaleEstimator.wePoseQ.push_back( selectedWEPose );

            // Number of successful to track frame
            successFrameCnt++;

            // Trajectory distance computation
            if( previousWEPose.timestampUs == -1 )
            {
               previousWEPose = selectedWEPose;
               trajDistSquareFromWEPose = 0.0;
            }
            else
            {
               float32_t distSquareFromWEPose = sqrt( (previousWEPose.pose.translation[0] - selectedWEPose.pose.translation[0]) * (previousWEPose.pose.translation[0] - selectedWEPose.pose.translation[0])
                                                      + (previousWEPose.pose.translation[1] - selectedWEPose.pose.translation[1]) * (previousWEPose.pose.translation[1] - selectedWEPose.pose.translation[1])
                                                      + (previousWEPose.pose.translation[2] - selectedWEPose.pose.translation[2]) * (previousWEPose.pose.translation[2] - selectedWEPose.pose.translation[2]) );
               trajDistSquareFromWEPose += distSquareFromWEPose;
               previousWEPose = selectedWEPose;
            }

            // Successful to collect data
            if( successFrameCnt > gScaleEstimator.successPoseNum2StopTargetless
                && trajDistSquareFromWEPose > gScaleEstimator.successTrajectory2StopTargetless )
            {// deadline to estimate scale 
               gScaleEstimator.failScaleFlag = true;
               gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::SCALE_ESTIMATOR );
            }
            else if( successFrameCnt > increamentalFrameCnt
                     && trajDistSquareFromWEPose > increamentalTrajectoryDist )
            {// try to early terminate pose collection stage
               // at least one more Frame and 0.1m distance required for new try
               increamentalFrameCnt++;
               increamentalTrajectoryDist += 0.1f;
               bool result = gScaleEstimator.estimateScale( 0.01f );
               if( result )
               {
                  gScaleEstimator.failScaleFlag = false;
                  gScaleEstimator.wePoseQ.clear();
                  gScaleEstimator.vslamPoseQ.clear();

                  //gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::SCALE_VERIFICATION );
                  if( gScaleEstimator.scaleVerificationV.scaleEnable )
                     gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::SCALE_VERIFICATION );
                  else
                     gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::SUCCESS_SCALE_ESTIMAION );
               }
            }
         }
         break;
      }
      case ScaleEstimationStatus::SCALE_ESTIMATOR:
      {
         // if not compute the scale, then determine the scale first before transforming to other status
         // should always do the scale computation in the status
         if( gScaleEstimator.failScaleFlag )
         {
            gScaleEstimator.setReinitTransformAndScalar();
         }

         //gScaleEstimator.setReinitTransformAndScalar();
         printf( "gScaleEstimator.failScaleFlag = %d\n", gScaleEstimator.failScaleFlag );
         if( !gScaleEstimator.failScaleFlag )
         {
            if( gScaleEstimator.scaleVerificationV.scaleEnable )
            {
               gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::SCALE_VERIFICATION );
            }
            else
            {
               gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::SUCCESS_SCALE_ESTIMAION );
            }
         }
         else
         {
            gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::FAIL_SCALE_ESTIMAION );
         }
         break;
      }
      case ScaleEstimationStatus::SCALE_VERIFICATION:
      {
         // Collect data 
         static int16_t failFrameCnt = 0;
         bool isVSLAMPoseHighQuality = IsPoseHighQuality( pose.poseWithState );
         if( isVSLAMPoseHighQuality )
         {
            gScaleEstimator.vslamPoseQ.push_back( pose );
            gScaleEstimator.wePoseQ.push_back( selectedWEPose );
            failFrameCnt = 0;
         }
         else
         {
            failFrameCnt++;
         }

         // Check the status ( ongoing/pass/fail)
         ScaleVerificationStatus verficationStatus = gScaleEstimator.scaleVerification();

         if( failFrameCnt > gScaleEstimator.scaleVerificationV.failFrameNum4Verfi || verficationStatus == ScaleVerificationStatus::ScaleVerificationFail )
         {
            gScaleEstimator.vslamPoseQ.clear();
            gScaleEstimator.wePoseQ.clear();
            gScaleEstimator.scaleVerificationV.failTimes = 0;
            gScaleEstimator.scaleVerificationV.passTimes = 0;
            gScaleEstimator.scaleVerificationV.isVerifiedSmall = false;
            gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::FAIL_SCALE_ESTIMAION );
         }
         else if( ScaleVerificationStatus::ScaleVerificationPass == verficationStatus )
         {
            gScaleEstimator.vslamPoseQ.clear();
            gScaleEstimator.wePoseQ.clear();
            gScaleEstimator.scaleVerificationV.failTimes = 0;
            gScaleEstimator.scaleVerificationV.passTimes = 0;
            gScaleEstimator.scaleVerificationV.isVerifiedSmall = false;
            gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::SUCCESS_SCALE_ESTIMAION );
         }

         break;
      }
      case ScaleEstimationStatus::SUCCESS_SCALE_ESTIMAION:
      {
         printf( "In the case of SUCCESS_SCALE_ESTIMAION!\n" );
         if( !hasPoseWithScale )
         {
            hasPoseWithScale = true;

            int64_t time_ReinitMapStart = getRealTimeForLog();
            SetReinitTransformationAndScalar( gScaleEstimator.scale, gScaleEstimator.mPoseSP );
            int64_t time_ReinitMapEnd = getRealTimeForLog();
            PRINTSTARTENDTIME( "map setScale&Transformation", mvVSLAM_GetMapSize( parameter.pVSlamObj ), time_ReinitMapStart, time_ReinitMapEnd );

            if( gScaleEstimator.isSaveMapFlag )
            {
               int64_t time_MergeMapStart = getRealTimeForLog();
               gMapBackupMutex.lock();
               if( parameter.externalPara.alwaysOnRelocation )
               {
                  gScaleEstimator.pMapBackup = mvVSLAM_ExportMapBackup( parameter.pVSlamSecondaryObj );
                  mvVSLAM_DeepReset( parameter.pVSlamSecondaryObj, false, gScaleEstimator.pMapBackup );
               }
               mvVSLAM_LoadDefaultMapAndMerge( parameter.pVSlamObj, gScaleEstimator.pMapBackup );
               mvVSLAM_ClearMapBackup( parameter.pVSlamObj );
               if( parameter.externalPara.alwaysOnRelocation )
               {
                  mvVSLAM_ClearMapBackup( parameter.pVSlamSecondaryObj );
               }
               gMapBackupMutex.unlock();
               int64_t time_MergeMapEnd = getRealTimeForLog();
               PRINTSTARTENDTIME( "map merging", mvVSLAM_GetMapSize( parameter.pVSlamObj ), time_MergeMapStart, time_MergeMapEnd );
            }
         }
         else
         {
            g_EnableSetExternalConstraint = true; // will enable external constraint check after SpatialInWorld is ready
            hasPoseWithScale = false;
            feedback = VSLAMScheduler::kFB_SCALEACQUIRED;
            gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::IDLE );
         }

         break;
      }
      case ScaleEstimationStatus::FAIL_SCALE_ESTIMAION:
      {
         PrepareforTargetlessInitMode( false );

         resetParamBuffer( failFrameCnt, successFrameCnt, previousWEPose, hasPoseWithScale, increamentalTrajectoryDist, increamentalFrameCnt );
         gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::START_POSE_COLLECTION_FOR_SCALE_ESTIMAION );
         break;
      }
      case ScaleEstimationStatus::TARGET_INITIALIZATION:
      {
         if( IsPoseHighQuality( pose.poseWithState ) )
         {
            gScaleEstimator.setScaleEstimationStatus( ScaleEstimationStatus::IDLE );
         }
         break;
      }
      default:
      {
         printf( "Invalid ScaleEstimationStatus!\n" );
         break;
      }
   }
}

//#define DUMP_UNDISTORTED_IMAGE
//#define printLog2File
//#define SIM_TARGETLESS_INITIALIZATION

void AddOneImage( int64_t timeStamp, const uint8_t * imageBuf )
{
   static mvWEFPoseVelocityTime curWEPose{ 0 };
   mvWEFPoseVelocityTime selectedWEPose;
   VSLAMScheduler::Feedback feedback = VSLAMScheduler::kFB_NONE;

   VSLAMScheduler* instance = VSLAMScheduler::getInstance();
   VSLAMScheduler::State state = instance->getState();

#ifdef printLog2File
   static FILE * poseFile = fopen( "./poseFile.txt", "wb" );
   fprintf( poseFile, "gScaleState: %d ", gScaleEstimator.getScaleEstimationStatus() );
   fflush( poseFile );
#endif

   printf( "add one image!!!timeStamp = %lld, gScaleState = %d\n", timeStamp, gScaleEstimator.getScaleEstimationStatus() );

   timeStamp /= 1000; // ns to us

   // constraint can be applied only if in ScaleEstimationStatus::IDLE state where VSLAM pose can be constrained by world coordinates
   if( parameter.externalPara.useExternalConstraint && g_EnableSetExternalConstraint )
   {
      mvPose6DET poseSpatialInWorld;
      if( mvWEF_GetTargetPose( gWEF, poseSpatialInWorld ) )
      {
         float rotation[9];
         float spatialInWorld[3][4];
         eulerToSO3( poseSpatialInWorld.euler, rotation );
         memcpy( spatialInWorld[0], rotation, sizeof( float ) * 3 );
         memcpy( spatialInWorld[1], rotation + 3, sizeof( float ) * 3 );
         memcpy( spatialInWorld[2], rotation + 6, sizeof( float ) * 3 );
         spatialInWorld[0][3] = poseSpatialInWorld.translation[0];
         spatialInWorld[1][3] = poseSpatialInWorld.translation[1];
         spatialInWorld[2][3] = poseSpatialInWorld.translation[2];
         mvVSLAM_SetExternalConstraint( parameter.pVSlamObj, true, parameter.externalPara.baselinkInVSLAM, spatialInWorld, parameter.externalPara.heightConstraint, parameter.externalPara.rollConstraint, parameter.externalPara.pitchConstraint );
         g_EnableSetExternalConstraint = false; // to avoid API call every frame
      }
   }
   else if( g_EnableSetExternalConstraint )
   {
      float nothing[3][4];
      mvVSLAM_SetExternalConstraint( parameter.pVSlamObj, false, parameter.externalPara.baselinkInVSLAM, nothing, parameter.externalPara.heightConstraint, parameter.externalPara.rollConstraint, parameter.externalPara.pitchConstraint );
      g_EnableSetExternalConstraint = false; // to avoid API call every frame
   }

   /*Lei: to check partial saving/loading here*/
   if (mapFocuser->NeedSpatialInWorld())  /*Lei: must work in world coordinate system*/
   {
      mvPose6DET poseSpatialInWorld;
      if (mvWEF_GetTargetPose(gWEF, poseSpatialInWorld))
      {
         float rotation[9];
         float spatialInWorld[3][4];
         eulerToSO3(poseSpatialInWorld.euler, rotation);
         memcpy(spatialInWorld[0], rotation, sizeof(float) * 3);
         memcpy(spatialInWorld[1], rotation + 3, sizeof(float) * 3);
         memcpy(spatialInWorld[2], rotation + 6, sizeof(float) * 3);
         spatialInWorld[0][3] = poseSpatialInWorld.translation[0];
         spatialInWorld[1][3] = poseSpatialInWorld.translation[1];
         spatialInWorld[2][3] = poseSpatialInWorld.translation[2];
         mapFocuser->SetSpatialInWorldMatrix(spatialInWorld[0]);
      }
   }
   int32_t mapSize = mvVSLAM_GetMapSize( parameter.pVSlamObj );
   printf( "+++++++keyfram =%d\n", mapSize );
   if( mapSize > 0 )
   {
      RemoveKeyframes( parameter.pVSlamObj );
      visualiser->ShowKeyframeLocationAndTrajectory( *mapFocuser, "Primary vslam" );
   }

   if( parameter.externalPara.wheelEnabled )
   {
      mvWEFPoseVelocityTime lastWEPose;
      do
      {
         lastWEPose = curWEPose;
         gWEPoseQueueWheel2VSLAM.wait_and_pop( curWEPose );

         if( !g_FirstFrame && curWEPose.timestampUs != 0 )
         {
            if( fabs( timeStamp - curWEPose.timestampUs ) > 1e6 )
            {
               printf( "Camera/wheel clock error!!!!\n" );
               printf( "camera_stamp %lld, wheel_stamp %lld\n", timeStamp, curWEPose.timestampUs );
               mvWEFPoseStateTime pose;
               InitPose6DRT( pose.poseWithState.pose );
               pose.timestampUs = -1;
               gVSLAMPoseRawQueue.check_push( VSLAMPoseWithFeedback( pose ) );
               return;
            }
         }
         if( g_FirstFrame )
            g_FirstFrame = false;
      } while( curWEPose.timestampUs < timeStamp && gVSALMRunning );

      const int64_t thDeltaUs = 20000; // TODO: 20ms, could be changed according to wheel odom rate
      int64_t curDelta = curWEPose.timestampUs - timeStamp;
      int64_t lastDelta = timeStamp - lastWEPose.timestampUs;

      bool isSynchronized = false;
      if( curDelta < lastDelta )
      {
         if( curDelta < thDeltaUs && curDelta > 0 )
         {
            selectedWEPose = curWEPose;
            isSynchronized = true;
         }
      }
      else
      {
         if( lastDelta < thDeltaUs && lastDelta > 0 )
         {
            selectedWEPose = lastWEPose;
            isSynchronized = true;
         }
      }

      if( !isSynchronized )
      {
         printf( "Drop image with timestamp %lld (us) since no corresponding wheel odom!\n", timeStamp );
         return;
      }

      static float linearVelocity[3] = { 0, 0, 0 }, angularVelocity[3] = { 0, 0, 0 };
      linearVelocity[0] = selectedWEPose.velocityLinear;
      angularVelocity[2] = selectedWEPose.velocityAngular;
      mvWEF_BodyToCameraPoint( gWEF, linearVelocity );
      mvWEF_BodyToCameraPoint( gWEF, angularVelocity );
      int64_t time_BeforeAddImage = getRealTimeForLog();
      mvVSLAM_AddImageWithWheel( parameter.pVSlamObj, timeStamp, imageBuf,
                                 mvWEF_BodyToCameraPose( gWEF, selectedWEPose.pose ).matrix,
                                 linearVelocity, angularVelocity );
      int64_t time_AfterAddImage = getRealTimeForLog();
      printf( "Process one image!!!timeStamp = %lld, gScaleState = %d duration = %fms\n", timeStamp, gScaleEstimator.getScaleEstimationStatus(), (time_AfterAddImage - time_BeforeAddImage)*1e-6 );
   }
   else
   {
      int64_t time_BeforeAddImage = getRealTimeForLog();
      mvVSLAM_AddImage( parameter.pVSlamObj, timeStamp, imageBuf );
      int64_t time_AfterAddImage = getRealTimeForLog();
      printf( "Process one image without wheel, duration = %fms\n", (time_AfterAddImage - time_BeforeAddImage)*1e-6 );
   }

   mvWEFPoseStateTime pose;
   pose.poseWithState = mvVSLAM_GetPose( parameter.pVSlamObj );
   pose.timestampUs = timeStamp;

   if (gScaleEstimator.getScaleEstimationStatus() < IDLE)
   {
	   mapFocuser->AddScalessPose(pose.poseWithState);
   }

#ifdef SIM_TARGETLESS_INITIALIZATION
   {
      static size_t cntFrame = 0;
      const size_t ndxFrame2ClaimFail = 200;
      cntFrame++;
      if( cntFrame > ndxFrame2ClaimFail
          && cntFrame < ndxFrame2ClaimFail + 2 * gScaleEstimator.failPoseNum2startTargetless )
      {
         pose.poseWithState.poseQuality = MV_VSLAM_TRACKING_STATE_FAILED;
         pose.poseWithState.pose.matrix[0][3] = 0.F;
         pose.poseWithState.pose.matrix[1][3] = 0.F;
         pose.poseWithState.pose.matrix[2][3] = 0.F;
      }
   }
#endif

   // Update scale estimator if needed
   scaleEstimatorStatusTransform( pose, selectedWEPose, feedback, state );

   gScaleEstimator.poseQuality = pose.poseWithState.poseQuality;
   printf( "Quality: %d \n", pose.poseWithState.poseQuality );
   //if( pose.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_FAILED )
   //   printf( "tracking failed\n" );

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
   fflush( poseFile );
#endif
   vslamStatus status = visualiser->ShowPoints( pose, imageBuf, parameter.externalPara.vslamCameraConfig.pixelWidth, parameter.externalPara.vslamCameraConfig.pixelHeight, "primary" );
   visualiser->PublishCameraPose( pose, status, "primary" );
   gVSLAMPoseRawQueue.check_push( VSLAMPoseWithFeedback( pose, feedback ) );
}

mvWEFPoseStateTime AddOneImageSecondary( int64_t timeStamp, const uint8_t * imageBuf )
{
	if (mapFocuserSecondary->NeedSpatialInWorld())  /*Lei: must work in world coordinate system*/
	{
		mvPose6DET poseSpatialInWorld;
		if (mvWEF_GetTargetPose(gWEF, poseSpatialInWorld))
		{
			float rotation[9];
			float spatialInWorld[3][4];
			eulerToSO3(poseSpatialInWorld.euler, rotation);
			memcpy(spatialInWorld[0], rotation, sizeof(float) * 3);
			memcpy(spatialInWorld[1], rotation + 3, sizeof(float) * 3);
			memcpy(spatialInWorld[2], rotation + 6, sizeof(float) * 3);
			spatialInWorld[0][3] = poseSpatialInWorld.translation[0];
			spatialInWorld[1][3] = poseSpatialInWorld.translation[1];
			spatialInWorld[2][3] = poseSpatialInWorld.translation[2];
			mapFocuserSecondary->SetSpatialInWorldMatrix(spatialInWorld[0]);
		}
	}

   int64_t time_BeforeAddImage = getRealTimeForLog();
   mvVSLAM_AddImage( parameter.pVSlamSecondaryObj, timeStamp, imageBuf );
   int64_t time_AfterAddImage = getRealTimeForLog();
   printf( "Process one image at secondary vslam !!!timeStamp = %lld, gScaleState = %d duration = %fms\n", timeStamp, gScaleEstimator.getScaleEstimationStatus(), (time_AfterAddImage - time_BeforeAddImage)*1e-6 );

   mvWEFPoseStateTime pose;
   pose.poseWithState = mvVSLAM_GetPose( parameter.pVSlamSecondaryObj );
   pose.timestampUs = timeStamp / 1000;
   int32_t keyframe2 = mvVSLAM_GetMapSize( parameter.pVSlamSecondaryObj );
   printf( "+++++++keyfram2 =%d\n", keyframe2 );
#ifdef SIM_TARGETLESS_INITIALIZATION
#if 1 // To simulate state transition from kSTATE_SECONDARY to kSTATE_PRIMARY_IMPORT
   static size_t highQualityCnt = 0;
   VSLAMScheduler* instance = VSLAMScheduler::getInstance();

   if( MV_VSLAM_TRACKING_STATE_FAILED != pose.poseWithState.poseQuality )
   {
      highQualityCnt++;
      if( 200 < highQualityCnt )
      {
         pose.poseWithState.poseQuality = MV_VSLAM_TRACKING_STATE_FAILED;
         if( 1000 == highQualityCnt )
            highQualityCnt = 0;
      }
   }
#endif
#endif

   printf( "Quality: %d \n", pose.poseWithState.poseQuality );

   vslamStatus status = visualiser->ShowPoints( pose, imageBuf, parameter.externalPara.vslamCameraConfig.pixelWidth, parameter.externalPara.vslamCameraConfig.pixelHeight, "secondary" );

   visualiser->PublishCameraPose( pose, status, "secondary" );
   int32_t mapSize = mvVSLAM_GetMapSize( parameter.pVSlamSecondaryObj );
   if( mapSize > 0 )
   {
      RemoveKeyframesSecondary( parameter.pVSlamSecondaryObj );
      visualiser->ShowKeyframeLocationAndTrajectory( *mapFocuserSecondary, "Sencondary VSLAM" );
   }
   return pose;
}


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

   CAMERA_PRINT( "In the front of while(true) to put One image to the VSLAM!\n" );


   VSLAMScheduler* instance = VSLAMScheduler::getInstance();
   while( true )
   {
      instance->listenPrimary();

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

      CAMERA_PRINT2( "PLAYBACK value: %d Description: False for online running, true for simulation with break points.\n", PLAYBACK );

      // Note: g_ImageBuf_mutex.unlock() has to be followed by VSLAM_MASTER_SLEEP_INTERNAL
      //       otherwise, the image preparation will be blocked by this lock->unlock loop in a while true loop
      if( !PLAYBACK ) //False for online running, true for simulation with break points.
      {

         CAMERA_PRINT( "This branch is for online running!\n" );

         g_ImageBuf_mutex.unlock();
         if( sleepOrNot )
         {
            VSLAM_MASTER_SLEEP_INTERNAL( SleepTimeInMillisecond );
         }
         else
         {
            //std::cout<<"===Begin algo Thread Executing: process current_frameid  = " <<current_frameid << "   current_stonesImage[0]=" << current_stonesImage[0] <<std::endl;
            AddOneImage( current_timestamp, current_ImageBuf );
            imageIndex++;
            printf( "%s: vslam image index = %u\n", __FUNCTION__, imageIndex );
            //std::cout<<"===End algo Thread Executing:   process  current_frameid = "<<current_frameid<<"   current_stonesImage[0]=" << current_stonesImage[0] << std::endl;
         }
      }
      else
      {

         CAMERA_PRINT( "This branch is for simulation offline!\n" );

         if( sleepOrNot )
         {
            //printf( "%s: Sleep in image processing\n", __FUNCTION__ );
         }
         else
         {
			if (imageIndex == 1500)
			{
				 //mapFocuser->AddCleanArea(-5.2f, -0.9f, -2.8f, 2.2f);
				 //mapFocuserSecondary->AddCleanArea(-5.2f, -0.9f, -2.8f, 2.2f);
			}
               
            AddOneImage( current_timestamp, current_ImageBuf );
            imageIndex++;
            printf( "%s: vslam image index = %u\n", __FUNCTION__, imageIndex );
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


   free( current_ImageBuf );
   printf( "Camera proc exit\n" );
}

void cameraSecondaryProc()
{
   if( !parameter.externalPara.alwaysOnRelocation )
   {
      return;
   }

   int bufferLength1 = parameter.externalPara.vslamCameraConfig.pixelHeight * parameter.externalPara.vslamCameraConfig.pixelWidth;
   int64_t current_timestamp = 0;
   unsigned char* current_ImageBuf = (unsigned char*)malloc( sizeof( unsigned char ) * bufferLength1 );
   unsigned int imageIndex = 0;

   VSLAMScheduler* instance = VSLAMScheduler::getInstance();
   VSLAMScheduler::Feedback feedback = VSLAMScheduler::kFB_NONE;
   while( true )
   {
      instance->listenSecondary();
      VSLAMScheduler::State state = instance->getState();

      feedback = VSLAMScheduler::kFB_NONE;

      static VSLAMScheduler::State previousState = VSLAMScheduler::kSTATE_PRIMARY;
      if( VSLAMScheduler::kSTATE_SECONDARY_IMPORT == state && VSLAMScheduler::kSTATE_SECONDARY_IMPORT != previousState )
      {
         int64_t time_LoadBackupStart = getRealTimeForLog();
         assert( nullptr != gScaleEstimator.pMapBackup );
         gMapBackupMutex.lock();
         bool result = mvVSLAM_LoadDefaultMapAndMerge( parameter.pVSlamSecondaryObj, gScaleEstimator.pMapBackup );
         gMapBackupMutex.unlock();
         if( result == true )
            printf( "%s: Successful load default map and merge with the new one.\n", __FUNCTION__ );
         else
            printf( "%s: Fail load default map and merge with the new one.\n", __FUNCTION__ );
         int64_t time_LoadBackupEnd = getRealTimeForLog();
         PRINTSTARTENDTIME( "secondary map loading", mvVSLAM_GetMapSize( parameter.pVSlamSecondaryObj ), time_LoadBackupStart, time_LoadBackupEnd );
         feedback = VSLAMScheduler::kFB_MAPIMPORTED;
      }
      previousState = state;

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

      mvWEFPoseStateTime pose;
      // Note: g_ImageBuf_mutex.unlock() has to be followed by VSLAM_MASTER_SLEEP_INTERNAL
      //       otherwise, the image preparation will be blocked by this lock->unlock loop in a while true loop
      if( !PLAYBACK ) //False for online running, true for simulation with break points.
      {
         g_ImageBuf_mutex.unlock();
         if( sleepOrNot )
         {
            VSLAM_MASTER_SLEEP_INTERNAL( SleepTimeInMillisecond );
         }
         else
         {
            //std::cout<<"===Begin algo Thread Executing: process current_frameid  = " <<current_frameid << "   current_stonesImage[0]=" << current_stonesImage[0] <<std::endl;
            pose = AddOneImageSecondary( current_timestamp, current_ImageBuf );
            imageIndex++;
            printf( "%s: vslam image index = %u\n", __FUNCTION__, imageIndex );
            //std::cout<<"===End algo Thread Executing:   process  current_frameid = "<<current_frameid<<"   current_stonesImage[0]=" << current_stonesImage[0] << std::endl;
         }
      }
      else
      {
         if( sleepOrNot )
         {
            //printf( "%s: Sleep in image processing\n", __FUNCTION__ );
         }
         else
         {
            pose = AddOneImageSecondary( current_timestamp, current_ImageBuf );
            imageIndex++;
            printf( "%s: vslam image index = %u\n", __FUNCTION__, imageIndex );
         }
         g_ImageBuf_mutex.unlock();
         VSLAM_MASTER_SLEEP_INTERNAL( SleepTimeInMillisecond );
      }

      if( !sleepOrNot )
      {
         bool isHighQuality = IsPoseHighQuality( pose.poseWithState );
         if( VSLAMScheduler::kSTATE_CONCURRENT == state && isHighQuality )
         {
            feedback = VSLAMScheduler::kFB_RELOCATED;
            gMapBackupMutex.lock();
            mvVSLAM_ClearMapBackup( parameter.pVSlamObj );
            mvVSLAM_ClearMapBackup( parameter.pVSlamSecondaryObj );
            gMapBackupMutex.unlock();
         }

         static int32_t numFailPose = 0;
         if( VSLAMScheduler::kSTATE_SECONDARY == state && MV_VSLAM_TRACKING_STATE_FAILED == pose.poseWithState.poseQuality )
         {
            numFailPose++;
            if( numFailPose == gScaleEstimator.failPoseNum2startTargetless )
            {
               // make sure # keyframe can be limited around vslamPara.maxKeyFrame
               if( mvVSLAM_GetMapSize( parameter.pVSlamSecondaryObj ) < (int32_t)vslamPara.maxKeyFrame - 5 )
               {
                  if( gScaleEstimator.isSaveMapFlag )
                  {
                     gMapBackupMutex.lock();
                     gScaleEstimator.pMapBackup = mvVSLAM_ExportMapBackup( parameter.pVSlamSecondaryObj );
                     gMapBackupMutex.unlock();
                  }
                  feedback = VSLAMScheduler::kFB_MAPEXPORTED;
               }
            }
         }
         else
         {
            numFailPose = 0;
         }


         gVSLAMPoseRawSecondaryQueue.check_push( VSLAMPoseWithFeedback( pose, feedback ) );
      }

      if( gVSALMRunning == false )
      {
         printf( "gVSLAMRunning is false and cameraSecondaryProc exit\n" );
         break;
      }
   } // while(true)

   free( current_ImageBuf );
}

void VSLAMCameraCallback( const int64_t timeStamp, const uint8_t * imageBuf )
{
   //printf( "new image come\n" );
   int bufferLength = parameter.externalPara.vslamCameraConfig.pixelHeight * parameter.externalPara.vslamCameraConfig.pixelWidth;
   g_ImageBuf_mutex.lock();
   g_timestamp = timeStamp;
   if( g_ImageBuf != NULL )
   {
      memcpy( g_ImageBuf, imageBuf, bufferLength );
   }
   g_ImageBuf_mutex.unlock();

   //CAMERA_PRINT2( "noThreadruning should be true and its value is %d\n", (int) noThreadruning );
   CAMERA_PRINT2( "gVSALMRunning should be true and its value is %d\n", (int)gVSALMRunning );

   if( gVSALMRunning && gFirstImg )
   {
      gFirstImg = false;
      cond.notify_one();
   }
}

void StartVSLAM()
{
   printf( "In the StartVSLAM() and would set gVSLAMRunning as true!\n" );
   gVSALMRunning = true;
}

void StopVSLAM()
{
   printf( "In the StopVSLAM() and would set gVSLAMRunning as false!\n" );
   if( gFirstImg )
   {
      gFirstImg = false;
      cond.notify_one();
   }

   VSLAMScheduler* instance = VSLAMScheduler::getInstance();
   gVSALMRunning = false;
   instance->release();

   printf( "Add one pose to exit\n" );
   mvWEFPoseStateTime pose;
   pose.timestampUs = 0;
   gVSLAMPoseRawQueue.check_push( VSLAMPoseWithFeedback( pose ) );
   gVSLAMPoseRawSecondaryQueue.check_push( VSLAMPoseWithFeedback( pose ) );
   gMotionVSLAMPoseQueue.check_push( pose );
}
