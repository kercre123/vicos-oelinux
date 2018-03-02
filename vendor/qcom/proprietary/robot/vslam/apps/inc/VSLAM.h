/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#pragma once
#include "VSLAMScheduler.h"
#include <string>
#include <Queue.h>
#include "WEF.h"
#include "MapFocuser.h"

struct VSLAMPoseWithFeedback
{
   mvWEFPoseStateTime pose;
   VSLAMScheduler::Feedback feedback;

   VSLAMPoseWithFeedback( const mvWEFPoseStateTime & pose, VSLAMScheduler::Feedback feedback = VSLAMScheduler::kFB_NONE ) :pose( pose ), feedback( feedback )
   {
   }
};

enum VSLAMInitMode
{
   TARGET_INIT = 0,
   TARGETLESS_INIT = 1,
   RELOCALIZATION = 2
};

extern queue_mt<bool> gScaleQueue;

extern queue_mt<VSLAMPoseWithFeedback> gVSLAMPoseRawQueue;
extern queue_mt<VSLAMPoseWithFeedback> gVSLAMPoseRawSecondaryQueue;


typedef struct _VSLAMParameter
{
public:
   char mapPath[200];

   mvPose6DRT targetPose;
   char targetImagePath[200];
   uint32_t maxKeyFrame;
   VSLAMInitMode initMode;
   mvCameraConfiguration vslamCameraConfig;
   float32_t targetWidth;
   float32_t targetHeight;
   bool wheelEnabled;
   bool loopClosureEnabled;
   bool alwaysOnRelocation;
   bool useExternalConstraint;
   float heightConstraint; //Unit: meter
   float rollConstraint; //Unit: rad
   float pitchConstraint; //Unit: rad
   float32_t baselinkInVSLAM[3][4]; //also the cross-calibration matrix
   float32_t targetHomography[9];

public:
   _VSLAMParameter()
   {
      mapPath[0] = 0;
      targetWidth = -1.0f;
      targetHeight = -1.0f;

      targetImagePath[0] = 0;
      initMode = VSLAMInitMode::TARGETLESS_INIT;
      wheelEnabled = false;
      loopClosureEnabled = false;
      alwaysOnRelocation = false;
	  useExternalConstraint = false;
	  heightConstraint = 10000.0f;
	  rollConstraint = 10.0f;
	  pitchConstraint = 10.0f;
      // Rotate to world coordinates:  X-Y on ground plane and Z coming out of ground
      targetPose.matrix[0][0] = 1.0f; targetPose.matrix[0][1] = 0.0f; targetPose.matrix[0][2] = 0.0f; targetPose.matrix[0][3] = 0.0f;
      targetPose.matrix[1][0] = 0.0f; targetPose.matrix[1][1] = 1.0f; targetPose.matrix[1][2] = 0.0f; targetPose.matrix[1][3] = 0.0f;
      targetPose.matrix[2][0] = 0.0f; targetPose.matrix[2][1] = 0.0f; targetPose.matrix[2][2] = 1.0f; targetPose.matrix[2][3] = 0.0f;
      targetHomography[0] = 1.0f; targetHomography[1] = 0.0f; targetHomography[2] = 0.0f;
      targetHomography[3] = 0.0f; targetHomography[4] = 1.0f; targetHomography[5] = 0.0f;
      targetHomography[6] = 0.0f; targetHomography[7] = 0.0f; targetHomography[8] = 1.0f;
   }

} VSLAMParameter;

/**--------------------------------------------------------------------------
@brief
Set the initial mode of vslam. The parameter map should be provide if
relocalization is need.
--------------------------------------------------------------------------**/
void SetInitMode( VSLAMParameter& currentPara, const char * map = NULL );

/**--------------------------------------------------------------------------
@brief
Set the initial mode of secondary vslam
--------------------------------------------------------------------------**/
void SetInitModeSecondary();

/**--------------------------------------------------------------------------
@brief
Start vslam
--------------------------------------------------------------------------**/
void InitializeVSLAM( const VSLAMParameter & para );

/**--------------------------------------------------------------------------
@brief
Save current map
--------------------------------------------------------------------------**/
bool GetPointCloud( const char* mapName );

/**--------------------------------------------------------------------------
@brief
Release vslam structure
--------------------------------------------------------------------------**/
void ReleaseVSLAM();

/**--------------------------------------------------------------------------
@brief
Callback function for camera
--------------------------------------------------------------------------**/
void VSLAMCameraCallback( const int64_t timeStamp, const uint8_t * imageBuf );

/**--------------------------------------------------------------------------
@brief
Start the vlsam algorithms
--------------------------------------------------------------------------**/
void StartVSLAM();

/**--------------------------------------------------------------------------
@brief
Stop the vslam algorithms
--------------------------------------------------------------------------**/
void StopVSLAM();

#define IsPoseHighQuality( poseWithState )    ( MV_VSLAM_TRACKING_STATE_GREAT == poseWithState.poseQuality \
                                             || MV_VSLAM_TRACKING_STATE_GOOD == poseWithState.poseQuality \
                                             || MV_VSLAM_TRACKING_STATE_OK == poseWithState.poseQuality )


