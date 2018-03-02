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
   PREPARE_FOR_SCALE_ESTIMATION = 0,                     // To save map, Deinitialize VSLAM obj and create a new one 
   START_POSE_COLLECTION_FOR_SCALE_ESTIMAION,            // Collect the two type poses for further scale computation
   SCALE_ESTIMATOR,								               // Compute the scale 
   SCALE_VERIFICATION,                                   // Scale verification
   SUCCESS_SCALE_ESTIMAION,                              // Successful to compute the scale
   FAIL_SCALE_ESTIMAION,                                 // fail to compute the scale 
   IDLE,                                                 // tracking or relocalization succeed or failed for only some frames ( < failPoseNum2startTargetless )
   TARGET_INITIALIZATION                                 // Set target based initilization mode but not succeed yet.
};


enum ScaleVerificationStatus
{ 
   ScaleVerificationOngoing = 0, 
   ScaleVerificationPass,
   ScaleVerificationFail
};


struct ScaleVerification
{ 
   int16_t failFrameNum4Verfi; 
   int16_t verfiNum; 

   float32_t scaleRatioThreshold;
   float32_t largeDistThreshold;
   float32_t smallDistThreshold;
   bool scaleEnable;

   int16_t failTimes = 0;
   int16_t passTimes = 0;

   bool isVerifiedSmall = false;
   //mvWEFPoseVelocityTime firstWePose;
   //mvWEFPoseStateTime firstvslamPose;
};


#define AR_PI_OVER_2_D                1.5707963267948966192313216916397514    // pi / 2 
#define PI 3.1415926

void eulerToSO3(const float* euler, float * rotation);
void eulerFromSO3(const float *rotation, float * euler);
float32_t getOriDist(float32_t ori1, float32_t ori2);

class ScaleEstimator
{
public:
   ScaleEstimator()
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
       
      successTrajectory2StopTargetless = 1.0f; 
      successPoseNum2StopTargetless = 150;
      failPoseNum2RestartTargetless = 100;
      failPoseNum2startTargetless = 50;
      countFailNnmAfterSuccessTrack = 0;
         
      scaleVerificationV.failTimes = 0;
      scaleVerificationV.passTimes = 0;
      scaleVerificationV.isVerifiedSmall = false; 
      scaleVerificationV.failFrameNum4Verfi = 50;
      scaleVerificationV.verfiNum = 1; 
      scaleVerificationV.scaleRatioThreshold = 0.6f;
      scaleVerificationV.largeDistThreshold = 0.4f;
      scaleVerificationV.smallDistThreshold = 0.2f;
      scaleVerificationV.scaleEnable = false;

      pMapBackup = nullptr;
   } 

   ScaleVerification scaleVerificationV;

   /**--------------------------------------------------------------------------
   @brief
   Determine transform for targetless initialization
   --------------------------------------------------------------------------**/
   bool determineTransform();

   /**--------------------------------------------------------------------------
   @brief
   Determine transform for targetless initialization
   --------------------------------------------------------------------------**/
   ScaleVerificationStatus scaleVerification();

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
   calculate the scale
   --------------------------------------------------------------------------**/
   bool estimateScale( const float minDist2 );
   
   /**--------------------------------------------------------------------------
   @brief
   Queue for wheel encoder pose
   --------------------------------------------------------------------------**/
   std::vector<mvWEFPoseVelocityTime> wePoseQ;

   /**--------------------------------------------------------------------------
   @brief
   Queue for vslam pose
   --------------------------------------------------------------------------**/
   std::vector<mvWEFPoseStateTime> vslamPoseQ;

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

   /**--------------------------------------------------------------------------
   @brief
   pointer to maintain map
   --------------------------------------------------------------------------**/
   void* pMapBackup;

   /**--------------------------------------------------------------------------
   @brief
   Threshold of fail to track frames for starting targetless initialization from configuration file
   --------------------------------------------------------------------------**/
   int32_t failPoseNum2startTargetless;

   /**--------------------------------------------------------------------------
   @brief
   Threshold of fail to track frames for re-starting targetless initialization from configuration file
   --------------------------------------------------------------------------**/
   int32_t failPoseNum2RestartTargetless;
    
   /**--------------------------------------------------------------------------
   @brief
   Only count # of failing to track after getting a successful to track pose
   --------------------------------------------------------------------------**/
   int32_t countFailNnmAfterSuccessTrack;

   /**--------------------------------------------------------------------------
   @brief
   Threshold of successful to track frames for stopping targetless initialization from configuration file
   --------------------------------------------------------------------------**/
   int32_t successPoseNum2StopTargetless;
     
   /**--------------------------------------------------------------------------
   @brief
   Threshold of trajectory distance for stopping targetless initialization from configuration file
   --------------------------------------------------------------------------**/
   float32_t successTrajectory2StopTargetless;
    
   /**--------------------------------------------------------------------------
   @brief
   Cross calibration paramters from configuration file
   --------------------------------------------------------------------------**/
   float32_t mPoseVBt[3];
   float32_t mPoseVBr[3];


private:
   std::mutex mut;

   /**--------------------------------------------------------------------------
   @brief
   Current status of scale estimater
   --------------------------------------------------------------------------**/
   ScaleEstimationStatus scaleEstimationStatus; 
};
extern ScaleEstimator gScaleEstimator;

