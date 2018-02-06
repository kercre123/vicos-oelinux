/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#pragma once

#include <string>
#include <atomic>

#include "VSLAM.h"

#ifdef WIN32
#include <windows.h>
#define VSLAM_MASTER_SLEEP_INTERNAL(x)  Sleep(x)
#else
#include <unistd.h>
#define VSLAM_MASTER_SLEEP_INTERNAL(x)  usleep(x*1000)
#endif //WIN32

#ifdef ARM_BASED
const bool PLAYBACK = false;
#else
#ifdef ROS_BASED
const bool PLAYBACK = false;
#else
const bool PLAYBACK = false;
#endif
#endif
const int SleepTimeInMillisecond = 2;

extern class Visualiser * visualiser;

extern std::atomic <bool> noThreadruning;


typedef struct _vslamparameter
{
public:
   // for keyframe creation
   float minDistance;
   float minDistanceAngle;
   float minAngleAngle;
   int minDelay;

   //for partial map loading
   bool enablePartialLoading;
   float radiusLargerSlice;
   float radiusEssentialSlice;
   float ratioEssentialSlice;
   int sizeEssentialMapSlice;
   int sizeCenterMapSlice;
   int sizeMaxMapSlice;
   int delayLoading; /*minimum interval (# of frames) between two loading operations*/

   mvVSLAM* pVSlamObj;

   VSLAMParameter externalPara;

public:
   _vslamparameter()
   {
      minDistance = 0.25f;
      minDistanceAngle = 0.1f;
      minAngleAngle = 0.15f;
      minDelay = 8;

      enablePartialLoading = false;
      radiusLargerSlice = 2;
      radiusEssentialSlice = 1.2f;
      ratioEssentialSlice = 0.8f;

      sizeEssentialMapSlice = 20;
      sizeCenterMapSlice = 40;
      sizeMaxMapSlice = 60;
      delayLoading = 8;

      pVSlamObj = NULL;
   }

} vslamparameterInternal;

extern vslamparameterInternal parameter;

/*
* 0 success
*/
int32_t ParseEngineParameters( const char * parameterFile, vslamparameterInternal & parameter );

extern std::string Program_Root;

#define SLAM_MIN(x,y) (x)<(y)?(x):(y)
#define SLAM_MAX(x,y) (x)>(y)?(x):(y)

/**--------------------------------------------------------------------------
@brief
Set the target for targetless initialization
--------------------------------------------------------------------------**/
bool SetTarget( const char* name );


/**--------------------------------------------------------------------------
@brief
Start map update
--------------------------------------------------------------------------**/
void StartMapUpdate();


/**--------------------------------------------------------------------------
@brief
Stop map update
--------------------------------------------------------------------------**/
void StopMapUpdate();


/**--------------------------------------------------------------------------
@brief
Preparation for targetless init mode
--------------------------------------------------------------------------**/
void PrepareforTargetlessInitMode(bool saveMap);

/**--------------------------------------------------------------------------
@brief
Analyze the scene whether targetless initialization is essential
--------------------------------------------------------------------------**/
bool AnalyzeInitTiming();



/**--------------------------------------------------------------------------
@brief
Set scalar and translation, rotation after targetless initilizaiton
--------------------------------------------------------------------------**/
void SetReinitTransformationAndScalar( float32_t scale, float poseMatrix[3][4] );
