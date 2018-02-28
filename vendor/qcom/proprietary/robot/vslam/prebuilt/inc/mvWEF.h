#ifndef MVWEF_H
#define MVWEF_H

/***************************************************************************//**
@file
   mvWEF.h

@brief
   Machine Vision,
   Wheel Encoder Fusion (WEF)

@section Overview
   This feature takes a pose (i.e., location + orientation) from other MV
   features (e.g., VISLAM) along with the data from a robots wheel encoder HW
   and fuses the two together for a better and more reliable pose estimate.

@section Limitations
   The following list are some of the known limitations:
   - Only tested with VSLAM.

@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/


//==============================================================================
// Defines
//==============================================================================


//==============================================================================
// Includes
//==============================================================================

#include <mvVSLAM.h>

//==============================================================================
// Declarations
//==============================================================================

#ifdef __cplusplus
extern "C"
{
#endif


   /************************************************************************//**
   @brief
      Pose and velocity information with timestamp (microseconds).
   @param timestampUs
      Timestamp in microseconds.
   @param pose
      Pose in intrinsic Tait-Bryan format.
   @param velocityLinear
      Linear velocity along with X (heading) direction.
   @param velocityAngular
      Angular velocity along with Z (anti-clockwise) direction.
   ****************************************************************************/
   typedef struct
   {
      int64_t    timestampUs;
      mvPose6DET pose;
      float32_t  velocityLinear;
      float32_t  velocityAngular;
   } mvWEFPoseVelocityTime;


   /************************************************************************//**
   @brief
      Pose information along with quality indicator and timestamp (microsecond).
   @param timestampUs
      Timestamp in microseconds.
   @param poseWithState
      Pose information along with quality indicator.
   ****************************************************************************/
   typedef struct
   {
      int64_t               timestampUs;
      mvVSLAMTrackingPose   poseWithState;
   } mvWEFPoseStateTime;


   /************************************************************************//**
   @brief
      Pose pair with time synchronized and outlier flag.
   @param pose
      Pose information along with quality indicator and timestamp (microsecond).
   @param wePose
      Pose and velocity information with timestamp (microseconds).
   @param isOutlier
      Exist outlier pose or not in this pair.
   ****************************************************************************/
   typedef struct
   {
      mvWEFPoseStateTime pose;
      mvWEFPoseVelocityTime wePose;
      bool isOutlier;
   } posePair;


   /************************************************************************//**
   @brief
      Wheel Encoder Fusion (WEF).
   ****************************************************************************/
   typedef struct mvWEF mvWEF;


   /************************************************************************//**
   @brief
      Return string of version information.
   ****************************************************************************/
   MV_API const char* mvWEF_Version( void );


   /************************************************************************//**
   @brief
      Initialize WEF.
   @param poseVB
      Cross calibration parameters, pose of Body under Camera frame.
   @param loadMapFirst
      Load or build map first?  0: build; 1: load
   @param stateBadAsFail
      Treat state Bad as Fail?  1: yes; 0: no
   @param dofRestriction
      restrict DoF from 6 to 3? 1: yes; 0: no
   @returns
      Pointer to WEF object; returns NULL if failed.
   ****************************************************************************/
   MV_API mvWEF* mvWEF_Initialize( const mvPose6DET* poseVB, const bool loadMapFirst, const bool stateBadAsFail, const bool dofRestriction );


   /************************************************************************//**
   @brief
      Deinitialize WEF object.
   @param pObj
      Pointer to WEF object.
   ****************************************************************************/
   MV_API void mvWEF_Deinitialize( mvWEF* pObj );


   /************************************************************************//**
   @brief
      Get pose of target image in world frame.
   @param pObj
      Pointer to WEF object.
   @param pose
      Pose of target image in world frame.
   @returns
      Successful or not.
   ****************************************************************************/
   MV_API bool mvWEF_GetTargetPose( mvWEF* pObj, mvPose6DET& pose );


   /************************************************************************//**
   @brief
      Set pose of target image in world frame.
   @param pObj
      Pointer to WEF object.
   @param pose
      Pose of target image in world frame.
   @returns
      Successful or not.
   ****************************************************************************/
   MV_API bool mvWEF_SetTargetPose( mvWEF* pObj, mvPose6DET& pose );


   /************************************************************************//**
   @brief
      Pass odometry from Wheel Encoder to the WEF object.
   @param pObj
      Pointer to WEF object.
   @param data
      Single odometry measurement data from Wheel Encoder.
   ****************************************************************************/
   MV_API void mvWEF_AddWheelOdom( mvWEF* pObj, mvWEFPoseVelocityTime& data );


   /************************************************************************//**
   @brief
      Pass pose to the WEF object.
   @param pObj
      Pointer to WEF object.
   @param poseWithState
      Single pose measurement data with quality indicator.
   @param timestampUs
      Timestamp of image capturing time in microsecond.
   ****************************************************************************/
   MV_API void mvWEF_AddPose( mvWEF* pObj, mvVSLAMTrackingPose& poseWithState,
                              const int64_t timestampUs );


   /************************************************************************//**
   @brief
      Get pose from the WEF object.
   @param pObj
      Pointer to WEF object.
   @param data
      Single pose estimated from WEF.
   @returns
      If the estimated pose has been updated or not.
   ****************************************************************************/
   MV_API bool mvWEF_GetPose( mvWEF* pObj, mvWEFPoseVelocityTime& data );


   /************************************************************************//**
   @brief
      Get corrected pose represented in World frame.
   @param pObj
      Pointer to WEF object.
   @param pose
      Single pose with quality indicator, represented in World frame.
   ****************************************************************************/
   MV_API void mvWEF_GetCorrectedPose( mvWEF* pObj, mvVSLAMTrackingPose& pose );


   /************************************************************************//**
   @brief
      Recover pose represented in frame same as pose measurement data.
   @param pObj
      Pointer to WEF object.
   @param pose
      Single pose measurement data recovered.
   ****************************************************************************/
   MV_API bool mvWEF_RecoverPose( mvWEF* pObj, mvPose6DRT& pose );


   /************************************************************************//**
   @brief
      Transform pose represented in Body frame to Camera frame.
   @param pObj
      Pointer to WEF object.
   @param pose6DET
      Single pose measurement represented in Body frame.
   @returns
      Single pose measurement represented in Camera frame.
   ****************************************************************************/
   MV_API mvPose6DRT mvWEF_BodyToCameraPose( mvWEF* pObj, 
                                             mvPose6DET& pose6DET );


   /************************************************************************//**
   @brief
      Rotate point represented in Body frame to Camera frame.
   @param pObj
      Pointer to WEF object.
   @param vec
      3x1 vector represented in Body frame (in), and in Camera frame (out).
   ****************************************************************************/
   MV_API void mvWEF_BodyToCameraPoint( mvWEF* pObj, float32_t* vec );


   /************************************************************************//**
   @brief
      Estimate scale of pose based on a vector of pose and Wheel data.
   @param poseQ
      Vector of pose.
   @param poseQLen
      Length of poseQ.
   @param wheelQ
      Vector of wheel data with real scale.
   @param wheelQLen
      Length of wheelQ.
   @param minDist2
      Square of minimal distance to compute scale.
   @param scale
      Estimated scale of pose.
   @param posePairQ
      Vector of pose pair with outlier rejected.
   @param posePairQLen
      Length of posePairQ.
   ****************************************************************************/
   MV_API bool mvWEF_EstimateScale( const mvWEFPoseStateTime* poseQ,
                                    const unsigned int poseQLen,
                                    const mvWEFPoseVelocityTime* wheelQ,
                                    const unsigned int wheelQLen,
                                    const float minDist2,
                                    float32_t* scale, posePair* posePairQ,
                                    unsigned int* posePairQLen );


#ifdef __cplusplus
}
#endif


#endif
