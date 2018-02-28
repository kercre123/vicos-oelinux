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
#include <queue>
#include <cmath>


queue_mt<mvWEFPoseVelocityTime> gWEPoseQueue;
queue_mt<mvWEFPoseStateTime> gVSLAMPoseQueue;
queue_mt<mvWEFPoseVelocityTime> gFsuionPoseQueue;
extern VSLAMParameter vslamPara;
extern MapFocuser* mapFocuser;
extern MapFocuser* mapFocuserSecondary;

inline void eulerToMatrix(const float euler[3], float matrix[3][4])
{
	float cr = cos(euler[0]);
	float sr = sin(euler[0]);
	float cp = cos(euler[1]);
	float sp = sin(euler[1]);
	float cy = cos(euler[2]);
	float sy = sin(euler[2]);
	matrix[0][0] = cy*cp;
	matrix[0][1] = cy*sp*sr - sy*cr;
	matrix[0][2] = cy*sp*cr + sy*sr;
	matrix[1][0] = sy*cp;
	matrix[1][1] = sy*sp*sr + cy*cr;
	matrix[1][2] = sy*sp*cr - cy*sr;
	matrix[2][0] = -sp;
	matrix[2][1] = cp*sr;
	matrix[2][2] = cp*cr;
}

mvWEF * StartFS( const mvPose6DET* poseVB, const bool loadMapFirst, const bool vslamStateBadAsFail, const bool dofRestriction )
{
   return mvWEF_Initialize( poseVB, loadMapFirst, vslamStateBadAsFail, dofRestriction );
}

mvWEFPoseVelocityTime GetFusionPose( mvWEF *wef )
{
   mvWEFPoseVelocityTime WEPose;
   mvWEFPoseStateTime VSLAMPose;
   mvWEFPoseStateTime VSLAMPoseCorrected;

   bool new_pose = false;
   bool new_vslam_pose = false;
   mvWEFPoseVelocityTime fpose;
   static std::queue<mvWEFPoseVelocityTime> WEPoseQueue;
   while( !new_pose )
   {
	   new_vslam_pose = false;

      if( THREAD_RUNNING == false )
      {
         fpose.timestampUs = 0;
         break;
      }

      if( gWEPoseQueue.try_pop( WEPose ) )
      {
         new_pose = true;
         mvWEF_AddWheelOdom( wef, WEPose );

         // Save the wheel pose based on the timestamps of lastGoodvslamPose
         while( WEPoseQueue.size() >= 8 )
         {
            WEPoseQueue.pop();
         }
         WEPoseQueue.push( WEPose );

         visualiser->RecordWheelOdom( WEPose );
      }

      if( gVSLAMPoseQueue.try_pop( VSLAMPose ) )
      {
         new_pose = true;
		 new_vslam_pose = true;
         bool isVSLAMPoseHighQuality = (VSLAMPose.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_GREAT ||
                                            VSLAMPose.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_GOOD ||
                                            VSLAMPose.poseWithState.poseQuality == MV_VSLAM_TRACKING_STATE_OK);
         // to save the last good quality vslam pose for post-processing (determine the transform after targetless initialization)
         if( isVSLAMPoseHighQuality )
         {
            int64_t dtThreshold = 20000;
            while( WEPoseQueue.size() > 1 )
            {
               mvWEFPoseVelocityTime lastWE = WEPoseQueue.front();
               int64_t dtWheel2VSLAM = VSLAMPose.timestampUs - lastWE.timestampUs;
               WEPoseQueue.pop();
               mvWEFPoseVelocityTime currentWE = WEPoseQueue.front();
               int64_t dtVSLAM2Wheel = currentWE.timestampUs - VSLAMPose.timestampUs;
               if( dtWheel2VSLAM >= 0 && dtVSLAM2Wheel >= 0 )
               {
                  if( dtWheel2VSLAM < dtVSLAM2Wheel && dtWheel2VSLAM < dtThreshold )
                  {
                     gScaleEstimator.lastGoodvslamPose = VSLAMPose;
                     gScaleEstimator.lastwheelPose = lastWE;
                     //printf( "set lastGoodPose at timestamp: %lld\n", gScaleEstimator.lastGoodvslamPose.timestampUs );
                     break;
                  }
                  if( dtVSLAM2Wheel < dtWheel2VSLAM && dtVSLAM2Wheel < dtThreshold )
                  {
                     gScaleEstimator.lastGoodvslamPose = VSLAMPose;
                     gScaleEstimator.lastwheelPose = currentWE;
                     //printf( "set lastGoodPose at timestamp: %lld\n", gScaleEstimator.lastGoodvslamPose.timestampUs );
                     break;
                  }
               }
            }
         }

         mvWEF_AddPose( wef, VSLAMPose.poseWithState, VSLAMPose.timestampUs );

         visualiser->RecordVSLAMOdom( VSLAMPose, kReadingVSLAM );

         mvWEF_GetCorrectedPose( wef, VSLAMPoseCorrected.poseWithState );
         VSLAMPoseCorrected.timestampUs = VSLAMPose.timestampUs;
         VSLAMPoseCorrected.poseWithState.poseQuality = VSLAMPose.poseWithState.poseQuality;

         if( MV_VSLAM_TRACKING_STATE_FAILED != VSLAMPoseCorrected.poseWithState.poseQuality &&
             MV_VSLAM_TRACKING_STATE_INITIALIZING != VSLAMPoseCorrected.poseWithState.poseQuality )
            visualiser->PublishCorrectedCameraPose( VSLAMPoseCorrected );
         // commented out this block since WEF cannot guarrantee to receive pose without scale
         //if( gScaleEstimator.getScaleEstimationStatus() == ScaleEstimationStatus::START_POSE_COLLECTION_FOR_SCALE_ESTIMAION )
         //{
         //   visualiser->RecordVSLAMOdom( VSLAMPose, kScaleEsVSLAM );
         //}
      }

      if( new_pose )
      {
         new_pose = mvWEF_GetPose( wef, fpose );
         visualiser->RecordFusedPose();

		 if (new_vslam_pose)
		 {
			 mvVSLAMTrackingPose slamPose;
			 slamPose.poseQuality = VSLAMPose.poseWithState.poseQuality;
			 eulerToMatrix(fpose.pose.euler, slamPose.pose.matrix);
			 slamPose.pose.matrix[0][3] = fpose.pose.translation[0];
			 slamPose.pose.matrix[1][3] = fpose.pose.translation[1];
			 slamPose.pose.matrix[2][3] = fpose.pose.translation[2];
			 mapFocuser->AddPoseInWorld(slamPose);
			 mapFocuserSecondary->AddPoseInWorld(slamPose);
		 }			 

      }
      //std::cout << "Output fusion pose\n";
      VSLAM_MASTER_SLEEP( 2 ); // sleep time is a variable.
   }

   return fpose;
}
void ReleaseFS( mvWEF* pObj )
{
   mvWEF_Deinitialize( pObj );
}
