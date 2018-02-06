/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#pragma once

#include "Queue.h"
#include <mutex>
#include "WEF.h"

enum ScaleEstimationStatus
{
   PREPARE_FOR_SCALE_ESTIMATION,                     // To save map, Deinitialize VSLAM obj and create a new one
   READY_TO_MOTION_FOR_SCALE_ESTIMATION,
   START_POSE_COLLECTION_FOR_SCALE_ESTIMAION,        // Collect the two type poses for further scale computation
   SCALE_ESTIMATOR,								  // Compute the scale 
   SUCCESS_SCALE_ESTIMAION, // Successful to compute the scale
   FAIL_SCALE_ESTIMAION,   // fail to compute the scale 
   IDLE
};


#define AR_PI_OVER_2_D                1.5707963267948966192313216916397514    // pi / 2 
#define PI 3.1415926

void eulerToSO3(const float* euler, float * rotation);
void eulerFromSO3(const float *rotation, float * euler);
float32_t getOriDist(float32_t ori1, float32_t ori2);

class ScaleEstimator
{
public:
   ScaleEstimator() :wePoseQueue( INFINITE_QUEUE ),
      vslamPoseQueue( INFINITE_QUEUE )
   {
      scaleEstimationStatus = ScaleEstimationStatus::IDLE;
      isSaveMapFlag = false;
      // initialize the two poses' transform for targetless initialization at the very beginning
      lastGoodvslamPose.timestampUs = -1;
      lastGoodvslamPose.poseWithState.pose.matrix[0][0] = 1.0;
      lastGoodvslamPose.poseWithState.pose.matrix[0][1] = 0.0;
      lastGoodvslamPose.poseWithState.pose.matrix[0][2] = 0.0;
      lastGoodvslamPose.poseWithState.pose.matrix[1][0] = 0.0;
      lastGoodvslamPose.poseWithState.pose.matrix[1][1] = 1.0;
      lastGoodvslamPose.poseWithState.pose.matrix[1][2] = 0.0;
      lastGoodvslamPose.poseWithState.pose.matrix[2][0] = 0.0;
      lastGoodvslamPose.poseWithState.pose.matrix[2][1] = 0.0;
      lastGoodvslamPose.poseWithState.pose.matrix[2][2] = 1.0;

      lastGoodvslamPose.poseWithState.pose.matrix[0][3] = 0.0;
      lastGoodvslamPose.poseWithState.pose.matrix[1][3] = 0.0;
      lastGoodvslamPose.poseWithState.pose.matrix[2][3] = 0.0;

      lastwheelPose.timestampUs = -1;
      lastwheelPose.pose.translation[0] = 0.0;
      lastwheelPose.pose.translation[1] = 0.0;
      lastwheelPose.pose.translation[2] = 0.0;
      lastwheelPose.pose.euler[0] = 0.0;
      lastwheelPose.pose.euler[1] = 0.0;
      lastwheelPose.pose.euler[2] = 0.0;
   }

   /**--------------------------------------------------------------------------
   @brief
   Load paramters for cross calibration between base link and camera from the configuration file
   --------------------------------------------------------------------------**/
   bool initialize();

   /**--------------------------------------------------------------------------
   @brief
   Determine transform for targetless initialization
   --------------------------------------------------------------------------**/
   bool determineTransform( std::vector<mvWEFPoseStateTime> vslamPoseQ, std::vector<mvWEFPoseVelocityTime> wePoseQ );

   /**--------------------------------------------------------------------------
   @brief
   Set the scale, rotation and translation
   --------------------------------------------------------------------------**/
   void setReinitTransformAndScalar();

   /**--------------------------------------------------------------------------
   @brief
   Get the status of current scale estimater
   --------------------------------------------------------------------------**/
   ScaleEstimationStatus getScaleEstimationStatus();

   /**--------------------------------------------------------------------------
   @brief
   Set the status of current scale estimater
   --------------------------------------------------------------------------**/
   void setScaleEstimationStatus( ScaleEstimationStatus );

   /**--------------------------------------------------------------------------
   @brief
   Queue for wheel encoder pose
   --------------------------------------------------------------------------**/
   queue_mt<mvWEFPoseVelocityTime> wePoseQueue;

   /**--------------------------------------------------------------------------
   @brief
   Queue for vslam pose
   --------------------------------------------------------------------------**/
   queue_mt<mvWEFPoseStateTime> vslamPoseQueue;

   /**--------------------------------------------------------------------------
   @brief
   Last good vslam pose with scale information before scale compuation
   --------------------------------------------------------------------------**/
   mvWEFPoseStateTime lastGoodvslamPose;

   /**--------------------------------------------------------------------------
   @brief
   Wheel pose corresponding to the lastGoodvslamPose
   --------------------------------------------------------------------------**/
   mvWEFPoseVelocityTime lastwheelPose;

   /**--------------------------------------------------------------------------
   @brief
   Start pose of the scale estimation in the VSLAM coordination
   --------------------------------------------------------------------------**/
   float32_t mPoseVWt[3];
   float32_t mPoseVWr[3];


   /**--------------------------------------------------------------------------
   @brief
   Start VSLAM pose of the scale estimation in the predefined VSLAM coordination
   --------------------------------------------------------------------------**/
   float32_t mPoseSP[3][4];

   /**--------------------------------------------------------------------------
   @brief
   Estimated scale
   --------------------------------------------------------------------------**/
   float32_t scale;

   /**--------------------------------------------------------------------------
   @brief
   Decide the scale estimation time ( first or second?)
   --------------------------------------------------------------------------**/
   bool isRestartScaleFlag;

   /**--------------------------------------------------------------------------
   @brief
   Fail the scale estimation
   --------------------------------------------------------------------------**/
   bool failScaleFlag;

   /**--------------------------------------------------------------------------
   @brief
   Flag of Save map or not (Load and merge map)
   --------------------------------------------------------------------------**/
   bool isSaveMapFlag;

   /**--------------------------------------------------------------------------
   @brief
   Save the pose quality of VSLAM
   --------------------------------------------------------------------------**/
   int32_t poseQuality;


   // only for debug
   mvWEFPoseVelocityTime debugWePose[300];
   mvWEFPoseStateTime debugVSLAMPose[300];

private:
   /**--------------------------------------------------------------------------
   @brief
   calculate the scale
   --------------------------------------------------------------------------**/
   bool estimateScale();
   std::mutex mut;

   /**--------------------------------------------------------------------------
   @brief
   Current status of scale estimater
   --------------------------------------------------------------------------**/
   ScaleEstimationStatus scaleEstimationStatus;

   /**--------------------------------------------------------------------------
   @brief
   Start pose of the scale estimation
   --------------------------------------------------------------------------**/
   mvWEFPoseVelocityTime startPose;

   /**--------------------------------------------------------------------------
   @brief
   Cross calibration paramters from configuration file
   --------------------------------------------------------------------------**/
   float32_t mPoseVBt[3];
   float32_t mPoseVBr[3];

};
extern ScaleEstimator gScaleEstimator;

