/***************************************************************************//**
@copyright
Copyright (c) 2017 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "WEF.h"
#include "ScaleEstimation.h"
#include <iostream>
#include <mvVWSLAM_app.h>
#include <Visualization.h>


queue_mt<mvWEFPoseVelocityTime> gMotionWEPoseQueue;
queue_mt<mvWEFPoseStateTime> gMotionVSLAMPoseQueue;
queue_mt<mvPose6DRT> gMotionFsuionPoseQueue;

queue_mt<mvWEFPoseVelocityTime> gWEPoseQueue;
queue_mt<mvWEFPoseStateTime> gVSLAMPoseQueue;
queue_mt<mvWEFPoseVelocityTime> gFsuionPoseQueue;

mvWEF * StartFS( const mvPose6DET* poseVB, const bool loadMapFirst, const bool vslamStateBadAsFail )
{
   return mvWEF_Initialize( poseVB, loadMapFirst, vslamStateBadAsFail );
}

void saveLastWheelData( mvWEFPoseVelocityTime WEPose )
{
   int64_t timeofvslamPose;
   int64_t curDT, preDT;

   timeofvslamPose = gScaleEstimator.lastGoodvslamPose.timestampUs;
   if( timeofvslamPose == -1 ) // no pose in this case
   {
      return;
   }

   curDT = abs( WEPose.timestampUs - timeofvslamPose );
   preDT = abs( gScaleEstimator.lastwheelPose.timestampUs - timeofvslamPose );
   if( curDT < preDT )
      gScaleEstimator.lastwheelPose = WEPose;
}

mvWEFPoseVelocityTime GetFusionPose( mvWEF *wef )
{
   mvWEFPoseVelocityTime WEPose;
   mvWEFPoseStateTime VSLAMPose;
   mvWEFPoseStateTime VSLAMPoseCorrected;

   bool new_pose = false;
   mvWEFPoseVelocityTime fpose;
   while( !new_pose )
   {
      if( THREAD_RUNNING == false )
      {
         fpose.timestampUs = 0;
         break;
      }

      if( gWEPoseQueue.try_pop( WEPose ) )
      {
         new_pose = true;
         mvWEF_AddWheelOdom( wef, WEPose );
         if( gScaleEstimator.getScaleEstimationStatus() == ScaleEstimationStatus::START_POSE_COLLECTION_FOR_SCALE_ESTIMAION )
         {
            gScaleEstimator.wePoseQueue.check_push( WEPose );
         }

         // Save the wheel pose based on the timestamps of lastGoodvslamPose
         saveLastWheelData( WEPose );

         visualiser->RecordWheelOdom( WEPose );
      }

      if( gVSLAMPoseQueue.try_pop( VSLAMPose ) )
      {
         new_pose = true;
         //mvWEF_AddPose( wef, VSLAMPose.poseWithState, VSLAMPose.timestampUs );
         if( gScaleEstimator.getScaleEstimationStatus() == ScaleEstimationStatus::IDLE )
         {
            mvWEF_AddPose( wef, VSLAMPose.poseWithState, VSLAMPose.timestampUs );

            // to save the last good quality vslam pose for post-processing (determine the transform after targetless initialization)
            //if( VSLAMPose.poseWithState.poseQuality >= 0 )
            if( VSLAMPose.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_GREAT ||
                VSLAMPose.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_GOOD ||
                VSLAMPose.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_OK )
            {
               gScaleEstimator.lastGoodvslamPose = VSLAMPose;
            }

            visualiser->RecordVSLAMOdom( VSLAMPose, kReadingVSLAM );

            mvWEF_GetCorrectedPose( wef, VSLAMPoseCorrected.poseWithState );
            VSLAMPoseCorrected.timestampUs = VSLAMPose.timestampUs;
            VSLAMPoseCorrected.poseWithState.poseQuality = VSLAMPose.poseWithState.poseQuality;
            
            if( MV_VSLAM_TRACKING_STATE_FAILED != VSLAMPoseCorrected.poseWithState.poseQuality &&
                MV_VSLAM_TRACKING_STATE_INITIALIZING != VSLAMPoseCorrected.poseWithState.poseQuality )
               visualiser->PublishCorrectedCameraPose( VSLAMPoseCorrected );
         }
         if( gScaleEstimator.getScaleEstimationStatus() == ScaleEstimationStatus::START_POSE_COLLECTION_FOR_SCALE_ESTIMAION )
         {
            if( VSLAMPose.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_GREAT ||
                VSLAMPose.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_GOOD ||
                VSLAMPose.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_OK )
            {
               gScaleEstimator.vslamPoseQueue.check_push( VSLAMPose );
            }
            visualiser->RecordVSLAMOdom( VSLAMPose, kScaleEsVSLAM );
         }
      }

      if( new_pose )
      {
         new_pose = mvWEF_GetPose( wef, fpose );
         visualiser->RecordFusedPose();
      }
      //std::cout << "Output fusion pose\n";
      VSLAM_MASTER_SLEEP( 10 ); // sleep time is a variable.
   }

   return fpose;
}
void ReleaseFS( mvWEF* pObj )
{
   mvWEF_Deinitialize( pObj );
}
